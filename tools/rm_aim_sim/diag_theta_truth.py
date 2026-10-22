"""判别实验：用真值 armor center 跑 BodyTracker._theta_from_pair，
解出的 theta 与真值 theta（板 0/2 中心差）是否一致。

一致 → 问题在 PnP 噪声；不一致 → PLATES 常数与仿真真值定义有偏差。
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from vision_backend import BodyTracker

channel_dir = Path("C:/Users/34741/AppData/Local/Temp/rm_aim_sim_default")
tracker = BodyTracker()
errors = []
count = 0

deadline = time.monotonic() + 6.0
while time.monotonic() < deadline:
    try:
        state = json.loads((channel_dir / "state.json").read_text())
    except (FileNotFoundError, PermissionError, ValueError):
        time.sleep(0.05)
        continue
    truth = state.get("truth") or {}
    armors = {a["id"]: a["center"] for a in truth.get("armors") or []}
    if 0 in armors and 2 in armors:
        c0, c2 = armors[0], armors[2]
        theta_truth = math.atan2(c0[1] - c2[1], c0[0] - c2[0])
        # 真值直接解（板 0 对板 2，id 完全正确）
        theta_solved = tracker._theta_from_pair(armors[0], armors[2], 0, 2)
        if theta_solved is not None:
            error = abs(math.degrees(math.remainder(theta_solved - theta_truth, 2 * math.pi)))
            errors.append(error)
            count += 1
            if count <= 5 or count % 20 == 0:
                # 顺便验证常数：板 0/2 中心距应是 2*0.2481=0.4962
                distance = math.sqrt(sum((c0[k] - c2[k]) ** 2 for k in range(3)))
                print(f"[{count:3d}] theta_truth={math.degrees(theta_truth):8.2f} "
                      f"solved={math.degrees(theta_solved):8.2f} err={error:6.2f} "
                      f"板0-2距={distance:.4f} (常数 0.4962)", flush=True)
    time.sleep(0.1)

if errors:
    errors.sort()
    print(f"\nn={len(errors)} p50={errors[len(errors) // 2]:.3f} "
          f"p90={errors[int(len(errors) * 0.9)]:.3f} max={errors[-1]:.3f}")
