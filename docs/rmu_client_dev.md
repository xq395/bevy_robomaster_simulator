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
| 服务端接口（/api/rmu/*、/rmu/frame） | `tools/operator_web.py` |
| 前端 HUD、场地图、接管逻辑 | `tools/operator_ui.html` |
| 本机 Mock 裁判系统 | `tools/rmu_protocol/mock_engine.py`、`mini_broker.py` |
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

- 23 项单元/集成测试通过：`cd tools; python -W error::ResourceWarning -m unittest test_rmu_protocol`
- 协议逐字段与手册比对 + 线上字节回环：`python rmu_protocol/verify_proto.py <手册txt> rmu_protocol`
- 浏览器端到端 23 项：HUD、3334 画面、75Hz、零帧、限频、地图点击、跨源拒绝。

**未验证（需要真实赛事引擎/机器人）**

- `RadarInfoToClient` 的字段布局（手册无字段号，现按 `robot_info = 1`、12 组处理）。
- `MapClickCmd/Info` 坐标单位与场地尺寸，以及字段 3/6 的 `mode`/`type` 语义。
- 鼠标增量、滚轮 `mouse_z`、灵敏度是否符合固件预期。
- 真实 1080p60 HEVC 的端到端性能（Mock 编码端是瓶颈，不是解码端）。

## 7. 下一步 TODO（建议顺序）

1. **真实引擎联调**：只连不收发控制 → 核对 HUD/ID/阶段 → 单独验证图传 → 再逐项验证键鼠与指令。
2. **前端重构**（见下）：按模式拆界面、画面铺满、HUD 叠加。
3. **键位可配置 + 手柄支持**（当前键位写死在 `operator_ui.html` 的 `RMU_KEYS`）。
4. **链路状态面板**：延迟/丢包/帧率/码率 + 异常报警（当前只是文字串）。
5. **录制与复盘**：按时间戳落盘 ProtoBuf 与视频，赛后回放。
6. **声音告警**：血量/复活/判罚。
7. **.deps 重建**：OpenCV/OpenVINO 运行库 + `talos_sp_bridge.exe` + 视觉模型 IR，然后重打 `dist`。

### 前端重构要点（未实装）

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
