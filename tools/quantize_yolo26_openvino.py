#!/usr/bin/env python3
"""Quantize deploy_dev's YOLO26 pose model with representative images.

This is an offline tool.  The generated OpenVINO IR can be loaded by the
existing C++ Yolo26Detector because its input/output contract is unchanged.
Do not treat a model calibrated only on one simulator scene as match-ready.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import openvino as ov

# NNCF 2.19 targets the newer top-level OpenVINO Python API, while this machine
# intentionally uses OpenVINO 2024.6 to match deploy_dev. 2024.6 exposes these
# two public graph types through openvino.runtime instead.
if not hasattr(ov, "Node") or not hasattr(ov, "Output"):
    import openvino.runtime as ov_runtime
    import openvino.runtime.op as ov_runtime_op
    import openvino.runtime.opset13 as ov_runtime_opset13
    import openvino.runtime.utils.node_factory as ov_runtime_node_factory

    if not hasattr(ov, "Node"):
        ov.Node = ov_runtime.Node
    if not hasattr(ov, "Output"):
        ov.Output = ov_runtime.Output
    if not hasattr(ov, "Input"):
        ov.Input = ov_runtime.Input
    if not hasattr(ov, "opset13"):
        ov.opset13 = ov_runtime_opset13
    # Same public-module rename: NNCF imports openvino.op directly.
    sys.modules.setdefault("openvino.op", ov_runtime_op)
    sys.modules.setdefault("openvino.opset13", ov_runtime_opset13)
    sys.modules.setdefault("openvino.utils.node_factory", ov_runtime_node_factory)

import nncf


INPUT_H = 640
INPUT_W = 640
EXPECTED_OUTPUT = (1, 30, 18)
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png"}


def preprocess(bgr: np.ndarray) -> np.ndarray:
    """Match deploy_dev/detection/yolo26_detector.cpp exactly."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("expected a BGR image with three channels")

    height, width = bgr.shape[:2]
    scale = min(INPUT_W / width, INPUT_H / height)
    resized_w = int(width * scale)
    resized_h = int(height * scale)
    pad_x = (INPUT_W - resized_w) // 2
    pad_y = (INPUT_H - resized_h) // 2

    resized = cv2.resize(bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((INPUT_H, INPUT_W, 3), 114, dtype=np.uint8)
    canvas[pad_y : pad_y + resized_h, pad_x : pad_x + resized_w] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0


def load_images(source_dir: Path, limit: int) -> list[np.ndarray]:
    paths = sorted(
        path for path in source_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if limit > 0:
        paths = paths[:limit]
    images: list[np.ndarray] = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is not None:
            images.append(image)
    if not images:
        raise RuntimeError(f"no readable images under {source_dir}")
    return images


def collect_talos_images(shm_dir: Path, count: int, interval_ms: float,
                         timeout_s: float) -> list[np.ndarray]:
    # Reuse the already-tested Talos v2 reader. It releases every bundle before
    # this function sleeps, so calibration never backpressures the simulator.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from deploy_dev_talos_bridge import TalosShm  # pylint: disable=import-outside-toplevel

    images: list[np.ndarray] = []
    deadline = time.perf_counter() + timeout_s
    next_sample = 0.0
    last_seq = -1
    with TalosShm(shm_dir) as talos:
        while len(images) < count and time.perf_counter() < deadline:
            bundle = talos.read_bundle()
            if bundle is None:
                time.sleep(0.001)
                continue
            now = time.perf_counter()
            if bundle.frame_seq != last_seq and now >= next_sample:
                images.append(bundle.image_bgr)
                last_seq = bundle.frame_seq
                next_sample = now + interval_ms / 1000.0
                print(f"\r[collect] {len(images)}/{count} seq={bundle.frame_seq}",
                      end="", flush=True)
    print()
    if len(images) < count:
        raise RuntimeError(
            f"collected only {len(images)}/{count} frames in {timeout_s:.1f}s"
        )
    return images


def validate_contract(model: ov.Model) -> None:
    input_shape = tuple(model.input().shape)
    output_shape = tuple(model.output().shape)
    if input_shape != (1, 3, INPUT_H, INPUT_W):
        raise RuntimeError(f"unexpected model input {input_shape}")
    if output_shape != EXPECTED_OUTPUT:
        raise RuntimeError(f"unexpected model output {output_shape}")


def infer_many(core: ov.Core, model: ov.Model, samples: list[np.ndarray],
               iterations: int) -> tuple[list[np.ndarray], list[float]]:
    compiled = core.compile_model(
        model,
        "CPU",
        {ov.properties.hint.performance_mode: ov.properties.hint.PerformanceMode.LATENCY},
    )
    request = compiled.create_infer_request()
    request.infer(samples[0])
    outputs: list[np.ndarray] = []
    latencies_ms: list[float] = []
    for index in range(iterations):
        sample = samples[index % len(samples)]
        started = time.perf_counter()
        result = request.infer(sample)
        latencies_ms.append((time.perf_counter() - started) * 1000.0)
        outputs.append(np.asarray(result[compiled.output(0)]).copy())
    return outputs, latencies_ms


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def compare_outputs(reference: list[np.ndarray], quantized: list[np.ndarray],
                    confidence: float) -> dict[str, float | int]:
    abs_errors: list[np.ndarray] = []
    conf_errors: list[float] = []
    keypoint_errors: list[np.ndarray] = []
    count_mismatches = 0
    class_mismatches = 0
    for fp32, int8 in zip(reference, quantized):
        abs_errors.append(np.abs(fp32 - int8).reshape(-1))
        fp_rows = fp32[0]
        int_rows = int8[0]
        fp_mask = fp_rows[:, 4] >= confidence
        int_mask = int_rows[:, 4] >= confidence
        if int(fp_mask.sum()) != int(int_mask.sum()):
            count_mismatches += 1
        common = fp_mask & int_mask
        if np.any(common):
            conf_errors.extend(np.abs(fp_rows[common, 4] - int_rows[common, 4]).tolist())
            class_mismatches += int(
                np.count_nonzero(np.rint(fp_rows[common, 5]) != np.rint(int_rows[common, 5]))
            )
            keypoint_errors.append(
                np.abs(fp_rows[common, 10:18] - int_rows[common, 10:18]).reshape(-1)
            )
    all_errors = np.concatenate(abs_errors)
    all_keypoint_errors = (
        np.concatenate(keypoint_errors) if keypoint_errors else np.asarray([], dtype=np.float32)
    )
    return {
        "output_mae": float(np.mean(all_errors)),
        "output_max_abs": float(np.max(all_errors)),
        "detection_count_mismatch_frames": count_mismatches,
        "class_mismatches": class_mismatches,
        "confidence_mae": float(np.mean(conf_errors)) if conf_errors else float("nan"),
        "keypoint_mae_input_px": (
            float(np.mean(all_keypoint_errors)) if all_keypoint_errors.size else float("nan")
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="FP32 YOLO26 ONNX/XML")
    parser.add_argument("--output", type=Path, required=True, help="output INT8 .xml path")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--source-dir", type=Path, help="representative image directory")
    source.add_argument("--shm-dir", type=Path, help="live Talos v2 shared-memory directory")
    parser.add_argument("--frames", type=int, default=128)
    parser.add_argument("--interval-ms", type=float, default=75.0)
    parser.add_argument("--timeout-seconds", type=float, default=45.0)
    parser.add_argument("--benchmark-iterations", type=int, default=50)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument(
        "--preset", choices=("performance", "mixed"), default="mixed",
        help="mixed is the safer starting point for pose/keypoint accuracy",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.frames <= 0 or args.benchmark_iterations <= 0:
        raise ValueError("frames and benchmark-iterations must be positive")
    if args.output.suffix.lower() != ".xml":
        raise ValueError("--output must end in .xml")

    images = (
        load_images(args.source_dir, args.frames)
        if args.source_dir is not None
        else collect_talos_images(
            args.shm_dir, args.frames, args.interval_ms, args.timeout_seconds
        )
    )
    samples = [preprocess(image) for image in images]
    core = ov.Core()
    model = core.read_model(args.model)
    validate_contract(model)

    preset = (
        nncf.QuantizationPreset.MIXED
        if args.preset == "mixed"
        else nncf.QuantizationPreset.PERFORMANCE
    )
    dataset = nncf.Dataset(samples)
    print(f"[quantize] samples={len(samples)} preset={args.preset}")
    int8_model = nncf.quantize(
        model,
        dataset,
        subset_size=len(samples),
        preset=preset,
    )
    validate_contract(int8_model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ov.save_model(int8_model, args.output, compress_to_fp16=False)

    compare_count = min(args.benchmark_iterations, len(samples))
    fp_outputs, fp_ms = infer_many(core, model, samples, args.benchmark_iterations)
    int_outputs, int_ms = infer_many(core, int8_model, samples, args.benchmark_iterations)
    metrics = compare_outputs(
        fp_outputs[:compare_count], int_outputs[:compare_count], args.confidence
    )
    print(
        "[benchmark] "
        f"FP32 median={percentile(fp_ms, 50):.2f} ms p95={percentile(fp_ms, 95):.2f} ms; "
        f"INT8 median={percentile(int_ms, 50):.2f} ms p95={percentile(int_ms, 95):.2f} ms; "
        f"speedup={percentile(fp_ms, 50) / percentile(int_ms, 50):.2f}x"
    )
    print("[compare] " + " ".join(f"{key}={value}" for key, value in metrics.items()))
    print(f"[saved] {args.output}")
    print("[warning] Validate on diverse labeled match footage before enabling fire.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
