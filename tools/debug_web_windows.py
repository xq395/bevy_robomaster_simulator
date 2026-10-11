#!/usr/bin/env python3
"""Windows host for deploy_dev's existing HNU Vision Debug frontend."""
from __future__ import annotations

import argparse
import importlib.util
import json
import mmap
import os
import struct
import tempfile
import time
from pathlib import Path

from flask import Response, jsonify, request, send_file


MAGIC = b"HNUDBG1\0"
VERSION = 1
HEADER = struct.Struct("<8sIIQQQIIII")


class WindowsShmReader:
    def __init__(self, name: str):
        self.name = name.lstrip("/").replace("/", "_").replace("\\", "_")
        self.directory = Path(
            os.environ.get(
                "HNU_DEBUG_WEB_DIR",
                str(Path(tempfile.gettempdir()) / "hnu_vision_debug"),
            )
        )
        self.path = self.directory / f"{self.name}.mmap"
        self.state_path = self.directory / f"{self.name}.json"
        self.mapping: mmap.mmap | None = None
        self.file = None

    def close(self) -> None:
        if self.mapping is not None:
            self.mapping.close()
        if self.file is not None:
            self.file.close()
        self.mapping = None
        self.file = None

    def connect(self) -> bool:
        if self.mapping is not None:
            return True
        try:
            self.file = self.path.open("rb", buffering=0)
            self.mapping = mmap.mmap(
                self.file.fileno(), 0, access=mmap.ACCESS_READ
            )
            return True
        except (FileNotFoundError, OSError, ValueError):
            self.close()
            return False

    def frame(self) -> bytes | None:
        if not self.connect():
            return None
        try:
            for _ in range(5):
                first = HEADER.unpack_from(self.mapping, 0)
                magic, version, header_size, capacity, sequence, timestamp_ns, size, _, _, _ = first
                if magic != MAGIC or version != VERSION or header_size != HEADER.size:
                    self.close()
                    return None
                if sequence == 0 or sequence & 1 or size < 4 or size > capacity:
                    time.sleep(0.002)
                    continue
                jpeg = self.mapping[header_size : header_size + size]
                second = HEADER.unpack_from(self.mapping, 0)
                if (
                    sequence == second[4]
                    and not second[4] & 1
                    and jpeg[:2] == b"\xff\xd8"
                    and jpeg[-2:] == b"\xff\xd9"
                ):
                    # A stopped publisher leaves a valid JPEG in the mapping.
                    # Reject that last frame instead of advertising a live feed.
                    if not 0 <= time.time_ns() - timestamp_ns <= 2_000_000_000:
                        return None
                    return jpeg
            return None
        except (OSError, ValueError, struct.error):
            self.close()
            return None

    def state(self) -> dict:
        try:
            with self.state_path.open("r", encoding="utf-8") as source:
                value = json.load(source)
            return value if isinstance(value, dict) else {"value": value}
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            return {}

    def health(self) -> dict:
        frame = self.frame()
        state = self.state()
        return {
            "ok": frame is not None,
            "shm": f"/{self.name}",
            "mapping": str(self.path),
            "frame_available": frame is not None,
            "state_available": bool(state),
        }


def tuning_paths(shm_name: str) -> tuple[Path, Path]:
    safe = shm_name.lstrip("/").replace("/", "_").replace("\\", "_")
    safe = safe or "hnu_vision_debug"
    temporary = Path(tempfile.gettempdir())
    return (
        temporary / f"{safe}.tuning_request.yaml",
        temporary / f"{safe}.tuning_state.json",
    )


def load_frontend(deploy_dir: Path):
    app_path = deploy_dir / "debug_web" / "app.py"
    if not app_path.is_file():
        raise FileNotFoundError(f"deploy_dev web app not found: {app_path}")
    spec = importlib.util.spec_from_file_location("hnu_debug_web_app", app_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {app_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.ShmReader = WindowsShmReader
    module.tuning_paths = tuning_paths
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deploy-dir",
        type=Path,
        default=Path(r"D:\HNU_RM\代码栈\deploy_dev"),
    )
    parser.add_argument(
        "--web-dir",
        type=Path,
        default=Path(r"F:\talos_web_debug"),
    )
    parser.add_argument(
        "--talos-shm-dir",
        type=Path,
        default=Path(r"F:\talos_shm"),
    )
    parser.add_argument("--shm-name", default="/hnu_vision_debug")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    args.web_dir.mkdir(parents=True, exist_ok=True)
    os.environ["HNU_DEBUG_WEB_DIR"] = str(args.web_dir.resolve())
    deploy_dir = args.deploy_dir.resolve()
    frontend = load_frontend(deploy_dir)
    app = frontend.create_app(args.shm_name)
    web_root = deploy_dir / "debug_web"
    app.template_folder = str(web_root / "templates")
    app.static_folder = str(web_root / "static")
    snapshot_readers = {
        "raw": WindowsShmReader(args.shm_name),
        "annotated": WindowsShmReader(f"{args.shm_name}_annotated"),
        "gray": WindowsShmReader(f"{args.shm_name}_gray"),
        "binary": WindowsShmReader(f"{args.shm_name}_binary"),
    }
    projectile_statistics_path = args.talos_shm_dir / "projectile_statistics.json"
    projectile_statistics_reset_path = (
        args.talos_shm_dir / "projectile_statistics_reset.request"
    )
    target_lock_request_path = args.talos_shm_dir / "target_lock.request"

    @app.get("/snapshot/<view>")
    def snapshot(view: str):
        reader = snapshot_readers.get(view)
        if reader is None:
            return Response(status=404)
        frame = reader.frame()
        if frame is None:
            return Response(status=503)
        return Response(
            frame,
            mimetype="image/jpeg",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    @app.get("/windows-stream.js")
    def windows_stream_script():
        return send_file(
            Path(__file__).with_name("debug_web_windows_stream.js"),
            mimetype="application/javascript",
            max_age=0,
        )

    @app.get("/simulator/stats")
    def simulator_stats():
        try:
            with projectile_statistics_path.open("r", encoding="utf-8") as source:
                payload = json.load(source)
            if not isinstance(payload, dict):
                raise ValueError("statistics payload is not an object")
            return jsonify({"ok": True, **payload})
        except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError) as error:
            return jsonify({"ok": False, "message": str(error)}), 503

    @app.post("/simulator/stats/reset")
    @app.post("/simulator/hit-count/reset")
    def reset_simulator_statistics():
        try:
            args.talos_shm_dir.mkdir(parents=True, exist_ok=True)
            projectile_statistics_reset_path.write_text(
                str(time.time_ns()), encoding="ascii"
            )
            return jsonify({"ok": True, "message": "reset requested"}), 202
        except OSError as error:
            return jsonify({"ok": False, "message": str(error)}), 500

    @app.post("/simulator/target-lock")
    def target_lock():
        payload = request.get_json(silent=True) or {}
        action = payload.get("action")
        if action not in {"lock_current", "next", "unlock"}:
            return jsonify({"ok": False, "message": "invalid target-lock action"}), 400
        try:
            args.talos_shm_dir.mkdir(parents=True, exist_ok=True)
            temporary = target_lock_request_path.with_suffix(
                f".request.{time.time_ns()}.tmp"
            )
            temporary.write_text(action, encoding="ascii")
            os.replace(temporary, target_lock_request_path)
            return jsonify({"ok": True, "message": "target-lock request queued"}), 202
        except OSError as error:
            return jsonify({"ok": False, "message": str(error)}), 500

    original_index = app.view_functions["index"]

    def windows_index():
        response = app.make_response(original_index())
        html = response.get_data(as_text=True)
        response.set_data(
            html.replace(
                "</body>",
                '<script src="/windows-stream.js"></script></body>',
            )
        )
        return response

    app.view_functions["index"] = windows_index
    print(
        f"HNU Vision Debug http://{args.host}:{args.port} "
        f"mapping_dir={args.web_dir.resolve()}",
        flush=True,
    )
    app.run(
        host=args.host,
        port=args.port,
        threaded=True,
        use_reloader=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
