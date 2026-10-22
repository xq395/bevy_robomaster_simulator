# rm_aim_sim 接入（Godot 自瞄仿真 → 操作终端）

把 [ForgetWW520/rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim) 当成"轻量模拟器"接进
`tools/operator_web.py` 那套自定义客户端。终端一行不改：浏览器 HUD、接管、自瞄授权、
全手动、画面全部复用现有实现。

## 0. 打包分发（推荐）

把「自定义客户端 + 本机自瞄 + 仿真」打成一个可分发的包，三者可一键起，也可单独起：

```powershell
# 生成 dist\rm_aim_sim_bundle\ 与 dist\rm_aim_sim_bundle_<日期>.zip
powershell -ExecutionPolicy Bypass -File tools\build_rm_aim_sim_bundle.ps1
# 不带 180 MB 引擎（对方自己下载）：加 -NoEngine
```

包内结构、逐组件部署、跨机边界见 `bundle/README.md` 与 `bundle/docs/DEPLOY.md`。
打好的包在 `dist/`（已被 .gitignore 忽略），实测 84 MB / 135 个条目。

## 1. 目录职责

| 文件 | 作用 |
|---|---|
| `run_rm_aim_sim.ps1` | 一键拉起仿真 + 桥接 + 终端，并打开页面（单机调试版） |
| `bundle/` | 部署包模板：`deploy.ps1`、`lib/common.ps1`、三个组件各自的 `start.ps1`、文档 |
| `../../tools/build_rm_aim_sim_bundle.ps1` | 打包脚本（模板 + 载荷 + 引擎 → dist/zip） |
| `operator.cfg` | 给终端看的场景（曝光 1.0、平移+自转），不是视觉标定用 |
| `0001-local-ipc-cross-platform.patch` | 上游 `local_ipc.gd` 的 Windows 适配补丁 |
| `../rm_aim_sim_bridge.py` | 桥接本体：共享内存邮箱 ↔ UDP 5001 + HTTP 5000 |

## 2. 首次部署

```bash
# 1) 克隆（13 MB，不含引擎）
git clone --depth 1 https://github.com/ForgetWW520/rm_aim_sim.git F:/rm_aim_sim_ws/rm_aim_sim

# 2) 打 Windows 补丁（上游只支持 Linux：/dev/shm、/proc、0700 权限）
cd F:/rm_aim_sim_ws/rm_aim_sim
git apply F:/bevy_robomaster_simulator/tools/rm_aim_sim/0001-local-ipc-cross-platform.patch

# 3) 装 Godot 4.7.2 win64（仓库不带引擎；GitHub 直连慢可挂镜像）
#    https://github.com/godotengine/godot/releases/download/4.7.2-stable/Godot_v4.7.2-stable_win64.exe.zip

# 4) 首次导入资源（之后不用重复）
Godot_v4.7.2-stable_win64_console.exe --headless --path F:/rm_aim_sim_ws/rm_aim_sim --import
```

补丁改了什么（36 行，只动 `scripts/local_ipc.gd`）：

- 邮箱目录：Linux 仍用 `/dev/shm`，其它平台用 `OS.get_temp_dir()`
  （Windows 上是 `%TEMP%\rm_aim_sim_<channel>`，同样"同机同用户"）；
- 权限：`0700` 只在 Linux 设；
- 存活判定：Linux 查 `/proc/<pid>/status`，Windows 用 `OS.is_process_running()`；
- 原子替换：Windows 上 `rename` 遇到读者还开着旧文件会报共享冲突，加 4 次 1 ms 重试。
  桥接侧对应地用 `FILE_SHARE_DELETE` 打开，让仿真能直接覆盖。

## 3. 每次启动

```powershell
cd F:\bevy_robomaster_simulator\tools\rm_aim_sim
powershell -ExecutionPolicy Bypass -File .\run_rm_aim_sim.ps1
```

脚本按顺序做四件事：起仿真（`--remote` 打开外部控制）→ 等邮箱目录 → 起桥接 →
起终端并打开 <http://127.0.0.1:5002/>。

参数：`-Godot`、`-Project`、`-Python` 手动指定路径；`-Headless` 只跑控制不留图；
`-NoSim`（仿真已开着）、`-NoTerminal`（只起前面两段）、`-Check`（只查依赖）。

## 4. 端口与链路

```
浏览器 ─HTTP─> operator_web(5002) ─UDP 5001─> rm_aim_sim_bridge ─command.json─> rm_aim_sim
   ↑ 20Hz 心跳        └──HTTP 5000/snapshot/raw──┘        ↑ state.json / frame-*.raw
   └──────────── /frame(JPEG) ────────────────────────────┘
```

| 端口 | 协议 | 用途 |
|---|---|---|
| 5001/UDP | JSON 租约 | 接管、session/seq/ttl、状态回报 |
| 5000/HTTP | `/snapshot/raw` | 当前相机画面（JPEG，宽 ≤1280） |
| 5002/HTTP | 终端页面 | 和老 Daedalus 链路完全一致 |

`%TEMP%\rm_aim_sim_default\` 是仿真与桥接之间的邮箱，仿真退出时会自行清空。

## 5. 验收（本机已跑通）

1. `run_rm_aim_sim.ps1 -Check` → 依赖通过。
2. 启动后 `curl http://127.0.0.1:5002/api/state` 有 `age_ms`，`error` 为空。
3. 页面点「接管模式控制」→ `remote: true`、`mode: command`。
4. 开「自瞄」、按住「授权自动射击」→ 画面里云台跟枪，`shot_count` 增长，
   `reason` 在 `firing` / `tracking 3.8m 156ms` 之间切换。
5. 松手或关页面 → 250 ms 后 `reason: operator_timeout`、`remote: false`。

实测（`operator.cfg`，目标 1.5 m/s 平移 + 2 rad/s 自转）：6 秒 9 发 4 中，
扣血 80，帧耗时 37–51 ms/帧（1440×1080 渲染 + 回读 + 拷出）。

## 6. 已知边界

- **别最小化仿真窗口**：Godot 最小化后不渲染，`frame_post_draw` 不触发，画面会停；
  恢复窗口即自动继续（无需重启）。
- rm_aim_sim **没有底盘**：`movement` 轴、小陀螺在桥接里是空操作，状态里固定回 `false`。
- 小陀螺开关不会报错，只在 `reason` 里追加「小陀螺无底盘」。
- 自瞄用的是**仿真真值**（`truth`）+ 弹道解算 + 目标运动预测，不是视觉算法。
  它的用途是"把链路验收通"，接自己的检测网络时替换 `Tracker.solve()` 即可。

## 6.5 命中率与调参（实测）

默认参数就是扫出来的最优解，**不需要改**：

| 参数 | 默认 | 作用 |
|---|---|---|
| `--lead-window` | `0.10` | 估目标速度的取样窗口；越小越跟手、越大越稳 |
| `--tolerance-deg` | `0.45` | 开火角度门限（相对**预测点**） |
| `--release-latency` | `0` | 额外时延补偿；实测加正值反而变差 |
| `--rate-tol` | `1.2` | 开火前允许的角速度偏差 rad/s |

标准场景（1.5 m/s 平移 + 2 rad/s 自转，3.8 m 左右）：**96.9 / 98.6 / 98.5%**（三次）。
高难度（2 m/s + 4 rad/s）：**100 / 97.1%**。

提升是两处，按贡献排序：

1. **目标运动用二次外推**（`_motion_at_now`）。目标换向时是 3 m/s² 加减速，
   线性外推在 156 ms 飞行里偏 3.6 cm —— 3.8 m 处正好一个装甲板宽。加上二次项后
   从 **44% → 71%**。
2. **把"状态年龄"补进预测**（`ahead = now - 状态采样时刻`）。状态是 60 Hz 快照，
   云台追的永远是十几毫秒前的点；4 rad/s 下 16 ms 就是 3.7°，而装甲半宽只有 1°。
   这一项把 **71% → 98%**。

调参工具（都在 `tools/rm_aim_sim/`，需要先起仿真）：

```bash
# 单组基准：跑 N 秒固定时长，输出一行 JSON（fired/hit/hit_rate）
python tools/rm_aim_sim/bench_hit_rate.py --seconds 12 --tolerance 0.45 --window 0.10

# 参数扫描：每组配置自动起一个干净的仿真
bash tools/rm_aim_sim/sweep.sh 12 F:/.../bench.cfg

# 提前量消融：预测 vs 真实未来位姿，分离 ω / v 的贡献
python tools/rm_aim_sim/diag_lead.py --seconds 4

# 逐发归因：拿 events 里的出膛角 vs 命中时刻真正需要角，看漏在哪一轴
python tools/rm_aim_sim/attribute_misses.py --seconds 6
```

`bench.cfg` / `bench_hard.cfg` 把血量拉到 10000，避免靶车被打死后停下来变成静止靶、
把后续命中率虚高。

- 命中判定是 4 块装甲的矩形近似，不是整车；没有车体碰撞、散布、相机噪声。
- 桥接只支持同机；跨机要改成 TCP/HTTP，别把 `%TEMP%` 共享出去。

## 6.6 接自己的视觉（`--backend vision`）

桥接有两套自瞄后端：

| 后端 | 数据源 | 用途 |
|---|---|---|
| `truth`（默认） | 仿真真值 `truth` | 验收脚手架，97–100% |
| `vision` | **自己那套视觉管线** | 检测 → PnP → 跟踪 → 弹道 → 云台 |

```bash
python tools/rm_aim_sim_bridge.py --backend vision --detector rp0526 --vision-hz 20
```

检测器三选一：

- `rp0526`：深大 RobotPilots 魔改 YOLOv5（`D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx`）。
  输出 `[1,25200,22]` = 4 关键点 + conf + 颜色 + 数字，float16 输入、letterbox 左上对齐。
- `yolo26`：我们 `configs/infantry.yaml` 用的那个（`[1,300,18]`）。
- `ideal`：用仿真理想投影角点，**只验链路**，用来把"检测器质量"和"PnP/跟踪/弹道"分开。

链路和真机同源（`tools/deploy_dev_talos_bridge.py`）：`HebeiDetector/Rp0526Detector`
→ `solve_pnp` → 世界坐标跟踪 → 弹道迭代 → 云台绝对角。适配层在
`tools/rm_aim_sim/vision_backend.py`，只换了"帧从哪来、指令往哪写"。

### 实测状态（2026-10-20）

- **检测**：rp0526 在仿真画面 20/20 帧检出，置信度中位 0.88–0.94，
  `class_id=3`（正好 infantry_3）、`color_id=1`（蓝方）正确，重投影 0.10–0.11 px。
  yolo26 同场景只有零星检出且颜色头给 0/1/3 混着，**不适合仿真图像**。
- **ROI 裁剪**（默认开，`--no-roi` 关）：按上一帧目标框放大 2.2 倍裁一块正方形再送检测。
  网络输入还是 640×640，省不了算力，但**有效分辨率**上去了：PnP 距离比从 0.949
  回到 **1.019**（原来短 5%，是角点回归偏小导致的）。
- **几何**：用仿真理想角点喂 `solve_pnp`，世界坐标与真值差 **0.0 cm**，距离完全一致。
- **帧率**：去掉每帧无用的 JPEG 编码、绕开 PIL 转换后，视觉循环从 ~12 fps 提到 **17.5 fps**。
- **搜索**：真值后端"永远知道目标在哪"，视觉不知道——目标不在视野里就是找不到。
  所以桥接里必须有搜索：没目标时云台单向慢扫（`--search-rate`，默认 0.4 rad/s），
  刚丢目标先惯性保持 `--coast-seconds`（默认 1.5 s）再回去扫，
  否则"扫到→还没转过去→丢帧→又回去扫"会死循环（实测会把云台锁在偏心的角度上抖）。
- **闭环**：`ideal` 5–6%、`rp0526` 0–30%（truth 后端 97–100%）。
  逐发归因显示漏的全是 yaw，出膛角偏差中位 1.1°、最差 4.8°，而装甲半宽只有 1°。

### 还没解决的核心（已量化，结论与直觉相反）

> **2026-10-21 续**：下面这段的结论已被推翻一半——真正的瓶颈是**打侧向板**和**提前量重复补偿**，
> 不是单帧噪声。当前最好配置见本节末尾「角域跟踪」。

**不是帧率问题。** 把采样从 6.5 fps 降到 1.6 fps（丢掉 3/4 的帧），0.18 s 预测误差
只从 37.7 → 41.7 cm，几乎不动。误差来自**单帧测量本身**，不是采样密度：

| 量 | 实测 |
|---|---|
| 单帧 3D 位置误差 | 中位 **22 cm**（σ 12 cm） |
| 其中带符号的深度（距离）误差 | 中位 **−0.18 m**（系统性偏近 ~5%） |
| 单帧重投影误差 | **0.13 px**（角点在图上非常准） |
| 预测 0.18 s 的误差分解 | 径向 15.5 cm / **切向 22.5 cm（4 m 处 3.2°）** |

也就是说：角点在图上的位置很准，但**深度是靠像素尺度算的**——40 px 宽的板差 1–2 px
就是 5% 的尺度误差，4 m 处就是 20 cm。这个径向误差再经"速度估计"泄漏到切向，
正好变成闭环里看到的 1–5° yaw 偏差。

试过的补偿：把四个角点沿离心方向**内收 1 px**，深度偏置从 −0.179 m 变成 **+0.023 m**
（标定有效），但散布从 0.155 涨到 0.205 m，闭环命中率没有改善（0–17%），
所以 `--corner-offset` 保留但默认 0。

另试过但**不可行**：用 PnP 解出的板法向估 ω（"四板绕心"先验）。法向 yaw 帧间中位跳 16°、
最大 177°，估出的 ω 是 **−0.757 rad/s（真值 2.0，符号都不对）**——四角点解 135×56 mm 板的面外
旋转本就不适定，法向几乎平行视线。这条路在当前检测精度下可以放弃。

### 下一步

1. **角点精度**（唯一真正的杠杆）：仿真的灯条是高对比度图元，ROI 内做阈值 + 灯条配对 +
   亚像素中心，精度能比模型回归高一个量级；
2. 或者在仿真图上重标/微调 rp0526 的角点；
3. 帧率和估计器都别再动了——数据已经说明它们不是瓶颈。

### 角域跟踪（2026-10-21 当前最好，活靶命中率 ~45%）

> ⚠️ **测命中率前先确认靶车活着**：`stats.hit` 仍会计数，但死靶不再移动，
> 命中率会虚高到 99%。bench.cfg 的 `max_health` 上限是 10000（500 发），
> 一轮约 70 发 × 20 伤害 = 1400，跑 6~7 轮必须重启仿真。

视觉侧改成"角域"：瞄准方向直接用**像素重心 → 相机射线 → 世界**，不经过 PnP 的
世界坐标（它的横向噪声 0.076 m @4 m = 1.1°/帧，而像素重心只有 0.04°）。配合三件事
把活靶命中率从 10% 提到 **~45%**：

- **只打正对的板**：用 `投影宽度 / (fx·0.135/距离)` 估 facing，`AIM_MIN_FACING=0.85`
  （侧向板子弹只会擦过板面——实测只有 14/64 发真正穿越装甲平面，交点横偏 +11~+39 cm）；
- **提前量只补一次**：关掉桥接的 consume-lead（实测 108 ms），合并成 `--vision-latency`，
  最优 0.10–0.12 s；
- **开火门限**：`--max-fire-gap 0.08` + `--min-settle 0.15` + `--rate-tol 0.6`。

```powershell
# 每次测量前重启仿真（活靶）
& 'F:\rm_aim_sim_ws\godot\Godot_v4.7.2-stable_win64_console.exe' --path 'F:\rm_aim_sim_ws\rm_aim_sim' `
  -- --remote --config=F:\bevy_robomaster_simulator\tools\rm_aim_sim\bench.cfg --quit-after-seconds=7200

$env:AIM_MODEL='angle'; $env:AIM_MIN_FACING='0.85'
python tools/rm_aim_sim/bench_hit_rate.py --seconds 30 --warmup 4 --search-rate 1.5 `
  --backend vision --detector rp0526 --max-fire-gap 0.08 --min-settle 0.15 `
  --no-lead-consume --vision-latency 0.10 --rate-tol 0.6 --muzzle-lag 0 `
  --truth-log truth_log.jsonl

# 逐发归因（弹道重建；不要用 attribute_misses，ground 弹的 impact.time 是落地时刻）
python tools/rm_aim_sim/attribute_flight.py truth_log.jsonl
```

注意：**仿真窗口必须可见**（被遮挡/最小化就不渲染，整轮 0 发）。跑前把窗口置顶。

### 当前状态（2026-10-21 收口，活靶实测）

| 口径 | 视觉 rp0526 | 真值 Tracker（同口径对照） |
|---|---|---|
| gated | **45.0%**（40 s / 91 发） | 96.1% |
| calibrate-fire | **~40%**（14 s / 42 发） | 96.3% |

与最初 10–12% 比已提升 4 倍，但**未达 80%**。逐发弹道重建（`attribute_flight.py`）显示
剩余误差：最近接近 p50 0.041 m（0.59°）、p90 0.095 m；横向 p50 2.5 cm、纵向 1.4 cm、
法向 ~3.7 cm（提前量还差 ~0.025 s）。

**瓶颈已定位到预测时域**：`gap + latency(0.10) + flight(0.18) ≈ 0.30 s`，而真值链路只要
`0.016 + 0.16 ≈ 0.18 s`；加速度主导的误差按 T² 走，与实测 0.65°/0.28°≈2.3 吻合。
latency 拆解：取帧/传输 31–47 ms + 预处理&推理 27–33 ms + 消费 16 ms。
CUDA 推理（隔离 9.4 ms）试过，但被预处理和 6 MB 传输吃掉，端到端反而更差。

**下一步（唯一结构性抓手）**：把 6 MB 裸图从临时文件改成共享内存/mmap（要改
`F:\rm_aim_sim_ws\rm_aim_sim\scripts\camera_stream.gd` 发布端），预计 latency 100→40 ms，
预测时域 0.30→0.24 s，命中率有望到 65–75%。

### 2026-10-22：到达朝向门 + 调参流程（当前最好）

**调参不用再重启仿真**：`bench_tune.cfg` = `bench.cfg` 但 `weapon.damage=0`，命中照常计数、
靶车永不死。所有数据都按它测。单轮 25 s 约 120 发，命中率 70% 附近的 95% 置信区间是 ±8 个点——
**单轮差 5 个点不说明问题**，要比较就多轮汇总，或者一轮录真值后离线分析（见 `gate_study.py`）。

逐发归因换成**板平面口径**（`attribute_plane.py`，和仿真判命中一致：穿板平面时交点在矩形内）。
旧 `attribute_flight.py` 看"离板心最近距离"，侧向板擦边会被当成"差 2 cm"。新口径下
状态源 + 理想角点的未命中 **19/20 是横向偏出矩形**，未命中弹到达时 facing 中位 0.44，
命中弹 0.68：子弹飞的 ~0.16 s 里自转靶转过 ~18°，开火时还正的板到达时已经侧了。

于是加了**到达朝向门** `AIM_MIN_ARRIVAL_FACING`（默认 0 = 关）：把板的 facing 外推到
`gap + delay + 飞行时间` 之后，低于门限不开火。实测：

| 链路（angle 模型） | 不加门 | 门 0.75 |
|---|---|---|
| 状态源 + 理想角点（`--state-source --detector ideal`，lat 0.05） | 67.6%（472 发） | **78.8%**（288 发，±4.7） |
| 图像 + 理想角点（lat 0.07） | ~56% | **66%**（281 发） |
| 图像 + rp0526（lat 0.10） | 33–42% | 44–56%（小样本，不稳） |

```powershell
$env:AIM_MODEL='angle'; $env:AIM_MIN_FACING='0.85'; $env:AIM_MIN_ARRIVAL_FACING='0.75'
python tools/rm_aim_sim/bench_hit_rate.py --seconds 25 --warmup 4 --search-rate 1.5 `
  --backend vision --detector ideal --state-source --max-fire-gap 0.08 --min-settle 0.15 `
  --no-lead-consume --vision-latency 0.05 --rate-tol 0.6 --muzzle-lag 0 --truth-log t.jsonl
python tools/rm_aim_sim/attribute_plane.py t.jsonl     # 未命中分类
python tools/rm_aim_sim/gate_study.py t.jsonl 25       # 离线比较各门限（真值口径，上限）
```

注意的坑：

- `AIM_MODEL=orbit` 在默认 `motion_window=0.25` 下**从没生效过**——`_orbit_model` 要求窗口 ≥0.35 s，
  不够就悄悄退回二次拟合；之前所谓 orbit 的结果其实是二次拟合。开 `--motion-window 0.5` 让它真生效，
  命中率反而掉到 53%。
- 门必须独立于 `FireGate`：bridge 有自己的对齐判定，不看后端 `gate_fire`，只认 `info["arrival_block"]`。
- `--vision-latency` 是链路相关的：状态源最优 0.05，图像链路 0.07，旧文档的 0.10 偏大。

**离 80% 还差什么**：状态源 + 理想角点已经到 79%，说明预测和门本身基本够了。剩下的差距来自
图像链路（+约 20–30 ms 取帧时延，再丢 13 个点）和 rp0526 角点精度（再丢 10–20 个点）。
这两项都不是改 yaml 能解决的：前者要改 `camera_stream.gd` 的发布端（共享内存/双缓冲），
后者要做亚像素角点精修。

## 7. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 页面「模拟器离线」 | 桥接没起或 5001 被占：`netstat -ano` 里查 `:5001` |
| 画面 503 | 仿真窗口最小化，或没带 `--remote`（无外部控制时不发图） |
| `session_mismatch` | 仿真重启/暂停过，页面上的「接管」重新点一次 |
| 仿真启动即退 | 同通道已有实例：换 `-Channel`，或关掉残留 Godot |
| 云台不动 | 仿真界面右下「本机自瞄控制」没开；外部控制开着时本地操作被接管 |
