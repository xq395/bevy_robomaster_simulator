"""测视觉循环的真实吞吐：起一个桥接，轮询 /health 里的帧计数，算出实际 fps。

用法（先让仿真跑起来）：

    python tools/rm_aim_sim/measure_fps.py --vision-hz 20 --seconds 6
    python tools/rm_aim_sim/measure_fps.py --vision-hz 200 --variant "流水线"
"""

import argparse
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def health(port):
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=1.0) as reply:
            return json.loads(reply.read())
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description="测视觉循环实际 fps")
    parser.add_argument("--channel", default="default")
    parser.add_argument("--vision-hz", type=float, default=20.0)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--warmup", type=float, default=12.0, help="等模型加载 + 稳态")
    parser.add_argument("--detector", default="rp0526")
    parser.add_argument("--operator-port", type=int, default=5301)
    parser.add_argument("--video-port", type=int, default=5300)
    parser.add_argument("--python", default="")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    python = args.python or sys.executable
    # 桥接可能在上级目录（源码树 tools/）或同级（部署包 components/aim/）
    script = None
    for candidate in (Path(__file__).resolve().parent,
                      Path(__file__).resolve().parent.parent):
        candidate = candidate / "rm_aim_sim_bridge.py"
        if candidate.exists():
            script = candidate
            break
    if script is None:
        print(json.dumps({"label": args.label, "error": "找不到桥接脚本"}, ensure_ascii=False))
        return 2
    command = [python, str(script), "--channel", args.channel,
               "--operator-port", str(args.operator_port),
               "--video-port", str(args.video_port),
               "--backend", "vision", "--detector", args.detector,
               "--vision-hz", str(args.vision_hz)]
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + args.warmup
        first = None
        while time.monotonic() < deadline:
            info = health(args.video_port)
            if info and info.get("frames"):
                first = (time.monotonic(), info["frames"])
            time.sleep(0.25)
        if first is None:
            print(json.dumps({"label": args.label, "error": "bridge 没取到帧"}, ensure_ascii=False))
            return 1
        start, frames = first
        time.sleep(args.seconds)
        end = time.monotonic()
        info = health(args.video_port) or {}
        total = int(info.get("frames", frames))
        print(json.dumps({"label": args.label, "vision_hz": args.vision_hz,
                          "fps": round((total - frames) / (end - start), 2),
                          "loop_hz": info.get("hz"), "aim_hz": info.get("aim_hz"),
                          "infer_ms": info.get("infer_ms"),
                          "capture_ms": info.get("capture_ms"),
                          "convert_ms": info.get("convert_ms"), "size": info.get("size"),
                          "frames": total, "error": info.get("error", "")},
                         ensure_ascii=False), flush=True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        time.sleep(0.6)
    return 0


if __name__ == "__main__":
    sys.exit(main())
