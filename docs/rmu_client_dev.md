# RMU2026 自定义客户端 · 开发交接

本文是给「下一步开发」用的入口。协议细节见 `tools/rmu_protocol/README.md`，
页面使用说明见 `docs/controls.md`，这里只讲**在哪改、怎么跑、还差什么**。

## 1. 目标与边界

- 把操作手终端做成 RM2026 制作规范允许的**自定义客户端**：
  MQTT 3.1.1 + ProtoBuf 收发裁判数据，UDP 3334 收 HEVC 图传，浏览器里显示 HUD 并发送键鼠/指令。
- 正式比赛按 **S87 使用 Linux**；Windows 只用于开发和赛前联调。
- 客户端只做协议与界面，不做机器人固件、不做视觉算法。

## 2. 文件地图（改哪里）

| 想改什么 | 文件 |
|---|---|
| 裁判数据解析、键鼠/指令发送、限频 | `tools/rmu_protocol/client_link.py` |
| 协议字段定义 | `tools/rmu_protocol/rm_custom_client.proto`（改完跑 `generate.py`） |
| UDP 3334 分片重组、HEVC 解码 | `tools/rmu_protocol/video.py` |
| 0x0310 自定义数据流图传（300 B × 50 Hz） | `tools/rmu_protocol/custom_video.py` |
| 浏览器端到端测试 | `tools/e2e_rmu.py`（需 playwright + Chrome） |
| 服务端接口（/api/rmu/*、/rmu/frame） | `tools/operator_web.py` |
| 前端 HUD、场地图、接管逻辑 | `tools/operator_ui.html` |
| 本机 Mock 裁判系统 | `tools/rmu_protocol/mock_engine.py`、`mini_broker.py` |
| 轻量仿真接入（rm_aim_sim） | `tools/rm_aim_sim/`（启动脚本、场景、补丁）+ `tools/rm_aim_sim_bridge.py` |
| 三件套部署包 | `tools/rm_aim_sim/bundle/`（模板）→ `tools/build_rm_aim_sim_bundle.ps1` → `dist/rm_aim_sim_bundle_<日期>.zip` |
| 启动/打包 | `tools/operator_package/*`、`tools/build_operator_package.ps1` |
| 协议字段校验 | `tools/rmu_protocol/verify_proto.py` |

## 3. 本地运行

### 3.1 Mock（无需真实裁判系统）

```powershell
cd tools\operator_package
powershell -ExecutionPolicy Bypass -File .\run_operator.ps1 -Mode rmu-mock
```

打开 http://127.0.0.1:5002/ ，视频源选「裁判系统图传」。Mock 只绑定 127.0.0.1、无认证。

也可以分开启动，便于看日志：

```powershell
cd tools
python -m rmu_protocol.mock_engine --mqtt-port 3343 --video-to 127.0.0.1:3344
# 另一个终端（设置 RMU_* 环境变量后）
python operator_web.py
```

### 3.2 真实裁判系统（Windows 联调）

```powershell
cd tools\operator_package
powershell -ExecutionPolicy Bypass -File .\run_operator.ps1 -Mode rmu
```

默认连 `192.168.12.1:3333`，clientID 取 `operator_config.json` 的 `rmu.robot_id`。

### 3.3 Linux（比赛）

```bash
./run_operator.sh          # 连裁判系统
./run_operator.sh --mock   # 本机联调
./run_operator.sh --check  # 只检查依赖
```

环境变量（均可覆盖）：`RMU_ENABLED`、`RMU_MQTT_HOST`、`RMU_MQTT_PORT`、
`RMU_ROBOT_ID`、`RMU_VIDEO_HOST`、`RMU_VIDEO_PORT`。

### 3.3 轻量仿真（rm_aim_sim，Godot）

不用 Daedalus 那套重渲染时，可以换 Godot 的 rm_aim_sim：

```powershell
cd tools\rm_aim_sim
powershell -ExecutionPolicy Bypass -File .\run_rm_aim_sim.ps1
```

它跑在 **UDP 5001 + HTTP 5000** 这同一组本机接口上，终端不需要任何改动，
所以「模拟器」这一侧是可替换的。部署步骤、补丁内容、端口、已知边界见
`tools/rm_aim_sim/README.md`；语义映射见 `docs/rm_aim_sim_integration.md`。

### 3.4 仿真 + 真 RMU 协议（自定义客户端直接连仿真）

上面 3.3 是「冒充 Daedalus 本机接口」；如果要用**正式比赛的 RMU 协议**联调，
把 `mock_engine` 换成 `sim_engine`（同样的 MQTT + ProtoBuf + UDP 3334，数据源换成仿真）：

```powershell
# 1) 仿真（窗口必须可见/置顶，否则不渲染 → 没有图传）
& 'F:\rm_aim_sim_ws\godot\Godot_v4.7.2-stable_win64_console.exe' --path 'F:\rm_aim_sim_ws\rm_aim_sim' `
  -- --remote --config=F:\bevy_robomaster_simulator\tools\rm_aim_sim\bench.cfg --quit-after-seconds=7200

# 2) RMU 引擎（MQTT 3343 + HEVC/UDP 3344 + 0x0310，直接读仿真原始帧，不再需要桥接）
$env:PYTHONPATH='F:\bevy_robomaster_simulator\tools'
python -m rmu_protocol.sim_engine --mqtt-port 3343 --video-to 127.0.0.1:3344

# 3) 终端
$env:RMU_ENABLED='1'; $env:RMU_MQTT_PORT='3343'; $env:RMU_VIDEO_PORT='3344'; $env:RMU_ROBOT_ID='3'
python tools\operator_web.py        # http://127.0.0.1:5002/
```

一键脚本 `tools/operator_package/run_sim_rmu.ps1` 起三个进程：Godot 仿真 →
`rmu_protocol.sim_engine` → `operator_web`。仿真已在跑时加 `-SkipSim`；`-Size/-Fps/-Kbps`
调官方图传，`-CustomSize/-CustomFps/-CustomKbps` 调 0x0310（`-CustomSize none` 关闭）；
要同时用模拟器通路（UDP 5001 / HTTP 5000）加 `-WithBridge`。数据流：

```
仿真 state.json ──> SimModel ──> ProtoBuf ──MQTT 3343──> operator_web ──> 浏览器 HUD
仿真原始帧(IPC) ──> SimFrameSource ─┬─ HEVC(nvenc) ──UDP 3344──> 终端 /rmu/frame
                                    └─ H.264 300B×50Hz ──MQTT CustomByteBlock──> 终端 /rmu/custom_frame
浏览器键鼠 ──MQTT──> SimModel.handle ──> command.json ──> 仿真云台/开火
```

实测（2026-10-22，RTX 4060 笔记本，仿真 1440×1080）：

| 项 | 旧（经桥接 JPEG） | 现在 |
|---|---|---|
| 官方图传 3334 | 4–10 fps | **27–28 fps**，960×720，~1.5 Mbit/s，残帧 0 |
| 0x0310 画面 | 无 | **15 fps**，384×288，85–92 kbit/s，41–45 块/s，丢块 0 |
| 键鼠 → 仿真 | — | 鼠标转动云台 0.26 rad；按住左键 1.2 s 开火 13 发（仿真 10 Hz 射速） |

3334 的瓶颈原来是桥接每帧 JPEG 编码 44 ms + 取帧 34 ms 串行；现在取帧、缩放转 YUV、编码
分三条线程，吞吐由取帧（仿真等下一渲染帧再读回 6 MB）决定。HEVC 编码不是瓶颈：libx265
中位数 3 ms，但有 30–40 ms 尖峰；hevc_nvenc 稳定 5 ms，有 N 卡时自动用。

仿真窗口**不能最小化**（Godot 最小化不渲染，取帧超时）；引擎超时时会自动恢复窗口。

注意：仿真没有底盘，`movement` 被忽略，**鼠标横向/纵向分别驱动云台 yaw/pitch，
左键开火**；云台增量灵敏度用 `--mouse-yaw` / `--mouse-pitch` 调。

### 3.5 0x0310 自定义数据流图传（`custom_video.py`）

两条独立链路，别混：RJ45（1GbE）走 MQTT + 官方图传 UDP 3334，协议没给字节上限；
图传链路（VTM）走 0x0310（机器人→客户端，300 B，≤50 Hz，约 15 KB/s）和
0x0311（客户端→机器人，30 B，≤75 Hz），**无重传**（手册 V2.0.0 §1.4.2）。
客户端从 MQTT 的 `CustomByteBlock` 收到 0x0310 的内容。

- 编码：H.264，`intra-refresh`（不发周期 IDR，单帧不会突发超预算），VBV 限到 96 kbit/s。
  选型实测（仿真画面，10–15 fps）：x264 帧内刷新单帧最大 1.0–1.6 KB；HEVC 周期 IDR 单帧到 4 KB，
  会把 50 Hz 节拍堵住，所以没用。
- 块格式（自定义 v1，非官方/ACE 格式）：`u8 0x56 | u16 块序号 | u16 首记录偏移(0xFFFF=纯续传)`
  + 295 B 记录流；记录 = `u16 长度 | u8 标志 | u16 帧号` + Annex-B 访问单元，可跨块。
- 发送端按 50 Hz 每 20 ms 发一块；积压超过 0.3 s 跳帧，延迟有界。
- 丢块：丢掉断开的那一帧，之后照常解码，坏区域在 1 s 刷新周期内被补回，不占 0x0311 上行。
  离线模拟 384×288@15：丢块 2% 显示 96% 的帧（PSNR 中位 40.9 dB），5% 时 81%，10% 时 48%。
- **真机还没接**：仿真/Mock 里发送端直接发 MQTT；机器人端要把 `custom_video.Sender` 的
  `publish(block)` 换成经串口发 0x0310（每块即 300 B 数据段），编码跑在机器人的小电脑上。
- 背景：2027 规则取消了部署模式，0x0310 传画面不再是刚需（见队里的规则解读文档）；
  这里留作第二视角/官方图传中断时的兜底，默认在界面里不选中。

## 4. 数据流

```
浏览器 ──HTTP──> operator_web ──> RmuLink ──MQTT──> 裁判系统
   ↑ 20Hz 键鼠心跳                     │
   └── 200ms 轮询 /api/rmu/state       └── 300B CustomControl / 指令
浏览器 <──/rmu/frame(JPEG)── operator_web <──UDP 3334── 机器人图传(video.py)
```

- 键鼠：浏览器只上报「按了哪些键 + 本帧鼠标增量」，服务端按 75Hz 定频发
  `KeyboardMouseControl`；**没有接管就不发**。
- 指令：走白名单 + 字段校验 + 手册限频（指令 ≤10Hz，地图点击按身份 ≥0.5s/≥3s）。
- 图传：服务端收包 → 重组 → 解码 → 最新帧转 JPEG；页面按需拉取。

## 5. 必须保住的安全行为

改前端/服务端时不要破坏这些（已有自动化测试覆盖）：

- 只有明确「接管」后才发键鼠；释放接管发零帧。
- 输入 250ms 未刷新 → 自动改发全零帧。
- MQTT 断开 / 页面隐藏 / Esc → 撤销接管并清零。
- 模拟器 UDP 接管与官方键鼠接管互斥。
- 写接口校验同源（跨源返回 403）。

## 6. 已验证 / 未验证

**已验证（本机 Mock）**

- 27 项单元/集成测试通过（含 0x0310 块格式、预算、丢块、发送端重启）：
  `cd tools; python -W error::ResourceWarning -m unittest test_rmu_protocol`
- 浏览器端到端 28 项（对 Godot 仿真实测通过）：`set RMU_TEST_URL=http://127.0.0.1:5002 && python tools/e2e_rmu.py`
- 协议逐字段与手册比对 + 线上字节回环：`python rmu_protocol/verify_proto.py <手册txt> rmu_protocol`
- 浏览器端到端 23 项：HUD、3334 画面、75Hz、零帧、限频、地图点击、跨源拒绝。
- rm_aim_sim 通路（2026-10-20）：接管/撤销、自瞄跟踪、10Hz 开火、命中扣血、
  250ms 超时撤销、画面 37–51 ms/帧；细节见 `docs/rm_aim_sim_integration.md`。

**未验证（需要真实赛事引擎/机器人）**

- `RadarInfoToClient` 的字段布局（手册无字段号，现按 `robot_info = 1`、12 组处理）。
- `MapClickCmd/Info` 坐标单位与场地尺寸，以及字段 3/6 的 `mode`/`type` 语义。
- 鼠标增量、滚轮 `mouse_z`、灵敏度是否符合固件预期。
- 真实 1080p60 HEVC 的端到端性能（Mock 编码端是瓶颈，不是解码端）。

## 7. 下一步 TODO（建议顺序）

1. **真实引擎联调**：只连不收发控制 → 核对 HUD/ID/阶段 → 单独验证图传 → 再逐项验证键鼠与指令。
2. ~~前端重构~~（2026-10-22 已做：RMU 模式隐藏模拟器控件、画面铺满、HUD 叠加、接管边框、诊断折叠）。
3. **键位可配置 + 手柄支持**（当前键位写死在 `operator_ui.html` 的 `RMU_KEYS`）。
4. **链路状态面板**：延迟/丢包/帧率/码率 + 异常报警（当前只是文字串）。
5. **录制与复盘**：按时间戳落盘 ProtoBuf 与视频，赛后回放。
6. **声音告警**：血量/复活/判罚。
7. **.deps 重建**：OpenCV/OpenVINO 运行库 + `talos_sp_bridge.exe` + 视觉模型 IR，然后重打 `dist`。

### 前端重构要点（已实装，留作说明）

- 按模式拆界面：RMU 模式隐藏模拟器专用控件（自瞄、小陀螺、H.264 调参、自车小地图）。
- 视频铺满，HUD 半透明叠加：顶栏阶段/倒计时/双方基地前哨；两侧 5 台血量；底栏本机血量/热量/弹量/经济。
- 小地图折叠到角落，点击放大后标记；指令收进抽屉；调试数据默认折叠。
- 接管状态用明显边框 + 文字，Esc 撤销后立即消失。

## 8. 依赖与打包

- Python 依赖见 `tools/operator_package/requirements.txt`（`paho-mqtt==2.1.0`、`protobuf==6.32.1`、`av`、`Pillow`、`Flask`）。
- 重新生成 pb2：`python tools/rmu_protocol/generate.py`（需要 `grpcio-tools`）。
- 打包：`powershell -ExecutionPolicy Bypass -File tools\build_operator_package.ps1`；
  只要 RMU 客户端、不要图像桥接时加 `-SkipBridge`。

## 9. 常见问题

- **页面能开但没数据**：确认 Mock/裁判系统已启动，`RMU_ENABLED=1`，端口没被占用。
- **画面不刷新**：看 `/api/rmu/state` 的 `video` 字段（`fps/kbps/frame_age_ms/incomplete`）。
- **按键不动**：先点「接管键鼠」，再点画面锁定鼠标；未锁定时按键不发送。
- **地图点击无回显**：Mock 会回 `MapClickInfo`；真实系统需确认坐标单位。
- **代理导致 git 推送中断**：远程用 443 通道
  `ssh://git@ssh.github.com:443/xq395/bevy_robomaster_simulator.git`。
