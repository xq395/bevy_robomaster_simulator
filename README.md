<div align="center">

# RMU2026 自定义客户端 · 操作终端

**协议 → 图传 → 终端 → 电控 UI 验证**：没有实车也能把整条链路跑通的工具链

[![Python](https://img.shields.io/badge/Python-3.11-3776AB.svg?style=for-the-badge&logo=python)](https://www.python.org/)
[![RMU](https://img.shields.io/badge/RMU2026-协议%20V2.0.0-2E8B57.svg?style=for-the-badge)](#)
[![Godot](https://img.shields.io/badge/rm__aim__sim-Godot%204-478CBF.svg?style=for-the-badge&logo=godot-engine)](https://godotengine.org/)

</div>

> 仓库名还叫 `bevy_robomaster_simulator`（历史原因）。现在的主体是 RMU2026 **自定义客户端全链路**：
> 协议收发、官方图传、0x0310 低带宽图传、浏览器操作终端、场地底图、电控 0x0301 UI 的离线预览。
> 仓库里原有的 Bevy 模拟器 **Daedalus** 只是可选数据源之一，见 [文末](#daedalusbevy-仿真数据源之一)。

---

## 先分清四条链路（最容易反复混淆的地方）

| 链路 | 载体 | 带宽 | 谁负责画 |
|---|---|---|---|
| **自定义终端数据** | MQTT 3.1.1 + ProtoBuf over RJ45 | 协议未给字节上限，只有每消息频率限制 | 我们自己 |
| **官方图传** | UDP 3334，HEVC 1080p60 | 协议未定上限，无重传 | 官方选手端 |
| **0x0310 自定义数据流** | VTM 图传链路，机器人 → 客户端 | 300 B × ≤50 Hz ≈ **15 KB/s** | 我们自己解包 |
| **0x0301 自定义 UI** | 裁判系统串口 → 官方选手端叠加 | 30 帧/s 全命令共用、内容 ≤112 B → **3720 B/s × λ** | 官方选手端 |

自定义终端和自定义 UI 是**两件事**：前者是我们自己连 MQTT/图传画界面（本仓库主体），
后者是往官方选手端上叠图形（本仓库只提供离线验证工具，实际发送在电控的 C 板）。

---

## 这套东西现在能干什么

| 能力 | 状态 | 验证方式 |
|---|---|---|
| 25 个下行 topic 全订阅、键鼠 75 Hz 上行、按 1.3.1 限频 | ✅ | 单测 + e2e |
| 事件 / 判罚 / 增益 / 模块离线 / 能量机关 / 飞镖 / 空中支援 / 伤害 上屏 | ✅ | e2e 逐条断言文本 |
| 每条指令 3 秒内核对“已生效 / 未生效 + 原因” | ✅ | e2e 校验多类回执 |
| UDP 3334 HEVC 接收、分片重组、按需解码 | ✅ | 10 分钟浸泡 |
| 0x0310 图传：编码 → 300 B 分块 → 解包 → 掉块恢复 | ✅ | 仿真 15 fps / 丢块 0 |
| 场地几何底图（28×15 m，从 GLB 烘焙） | ✅ | 截图 |
| 电控 0x0301 UI 离线预览 + 帧校验 | ✅ | 11 帧 CRC 全对 |
| 真实裁判系统 / 官方引擎 | ❌ 未验证 | 见 [已知限制](#已知限制) |

---

## 快速开始

### 1. 没有硬件：Mock 裁判系统

```powershell
cd tools\operator_package
Start.bat -Mode rmu-mock
```

起本机 MQTT（3343）+ 合成裁判数据 + 测试图，浏览器开 <http://127.0.0.1:5002/>。
没有机器人也能点指令、看回执、看 HUD。

### 2. 接仿真：Godot `rm_aim_sim`（推荐演示用）

前置：Godot 4 和 `rm_aim_sim` 已在 `F:\rm_aim_sim_ws`（见 `tools/rm_aim_sim/README.md`）。

```powershell
cd tools\operator_package
run_sim_rmu.bat
```

依次起 **仿真 → RMU 引擎（MQTT 3343 + 两路图传）→ 操作终端**，2 秒后自动开浏览器。

| 参数 | 作用 |
|---|---|
| `run_sim_rmu.bat` | 全套，自动开浏览器 |
| `-NoOpen` | 不开浏览器 |
| `-NoWeb` | 只起仿真和引擎 |
| `-CustomSize none` | 关掉 0x0310 那一路 |
| `-Fps 30 -Kbps 6000` | 调 3334 图传 |
| `-CustomSize 384x288 -CustomFps 15` | 调 0x0310 |

> ⚠️ **仿真窗口不能最小化**：被遮挡/最小化时 Godot 不渲染，图传会没有画面。

### 3. 接真实裁判系统

Windows 联调：

```powershell
powershell -ExecutionPolicy Bypass -File .\run_operator.ps1 -Mode rmu
```

比赛部署（Linux，制作规范 S87）：

```bash
./run_operator.sh
```

默认连 `192.168.12.1:3333`（选手端自带的 MQTT 服务），监听 UDP 3334。
`clientID` = 所连机器人的 ID。**连上不等于能收发**，先只收数据看 HUD 是否正确。

### 启动脚本对照

| 脚本 | 起什么 |
|---|---|
| `tools\operator_package\run_sim_rmu.bat` | 仿真 + RMU 引擎 + 终端（**演示/联调用这个**） |
| `tools\operator_package\Start.bat` | 只起终端（`-Mode sim/rmu/rmu-mock`），可接外部数据源 |
| `run_daedalus.bat` | 只起 Bevy 模拟器 Daedalus 本体 |
| `tools\rm_aim_sim\bundle\一键部署.bat` | 分发用的三件套部署包（仿真 + 自瞄 + 终端） |

---

## 组件

### `tools/rmu_protocol/` —— 协议与图传核心

唯一认的协议定义是 `rm_custom_client.proto`（按手册 V2.0.0 第 2 章逐条整理，不抄开源项目）。

- `client_link.py`：MQTT 连接、25 个下行 topic、上行限频、键鼠零帧语义（250 ms 无输入即松扳机）
- `video.py`：UDP 3334 收包、按帧号重组、HEVC 解码
- `custom_video.py`：0x0310 编解码与分块，见下
- `avsafe.py`：PyAV 17 / FFmpeg 8 在并发 `reformat`/`to_image` 时会死锁或崩溃，
  所有转换都过这把全局锁——这是 3334 原来跑几分钟就停帧的根因
- `mock_engine.py`：本机 Mock 裁判系统（也能用 `MockTrigger` topic 注入事件）
- `sim_engine.py`：把数据源换成 Godot 仿真，键鼠翻成仿真的云台指令

### `tools/operator_web.py` + `operator_ui.html` —— 浏览器操作终端

单页终端，无框架依赖。RMU 模式下：

- 顶部 HUD：阶段、倒计时、血量/热量/弹量、基地与前哨站、双方血量条
- 事件流（中文播报）、判罚横幅、增益倒计时、模块离线、机制状态行、伤害统计
- 战术地图：场地底图 + 本机位置朝向 + 雷达点 + 点击发 `MapClickCmd`
- 指令面板：每条指令发出后对照回传数据判定是否生效，失败给原因
- 键鼠接管：指针锁定、Esc 撤销、跨源接管拒绝

### 0x0310 低带宽图传（300 B × 50 Hz）

给“官方图传断了”或“需要第二视角”准备的兜底通道，默认不在界面预选。

- 编码 H.264 `intra-refresh`（不发周期 IDR，单帧不会突发超预算），VBV 限 96 kbit/s
- 块格式（自定义 v1）：`u8 0x56 | u16 块序号 | u16 首记录偏移(0xFFFF=纯续传) | 295 B 记录流`
- 50 Hz 定速发送，积压超 0.3 s 跳帧；丢块只丢那一帧，1 s 内被刷新覆盖
- 实测 384×288@15 fps：丢块 2% 仍显示 96% 的帧（PSNR 中位 40.9 dB）

> 真机未接：机器人侧要把 `custom_video.Sender.publish(block)` 换成走串口发 0x0310。

### 场地底图

`assets/GROUND.glb` 俯视光栅化成 1120×600 高度阴影图（93 KB），终端战术地图直接铺底：

```bash
python tools/rmu_field_map.py     # -> tools/rmu_field.png，服务端 /rmu/field.png
```

### 0x0301 自定义 UI 离线预览（给电控用）

不碰 C 板，在电脑上调 UI 排版：

```bash
python tools/ui0301_host/build.py      # 用 MSVC 编译电控仓库里原样的 UI_print.c
python tools/ui0301_preview.py tools/ui0301_host/out/frames.bin -o ui.png
```

预览器校验 CRC8/CRC16、Data_Length、sender↔receiver、图层/颜色/坐标/字号范围，
按选手端坐标系（左下原点，1920×1080）出图，并给出字节预算。也能直接吃 USB 转串口抓的 UART6 原始字节。

**顺带查出电控 UI 库的三处问题**（详见 `docs/custom_client_plan_20261026.md` §3.2）：

1. `uiDrawGraphics` / `uiDrawIntData` / `uiDrawStringData` / `uiDeleteGraphic` **一个字节都不发**
   （`UI_SendByteS` 全被注释），只有 `ui*Seq` + 自己遍历 `UI_SendTCBSequence` 调 `UI_Draw()` 这条路能用；
   删除图层目前没有能发出去的路径。
2. `uiRefreshStringDataSeq` 用 `sizeof(chardata) - 1` 当长度（指针大小），板子上刷新后的字符串只剩 3 个字符。
3. 刷新函数里 `frequency` 没像创建时那样 `/10`，写调度器时要统一单位。

### `tools/rm_aim_sim/` —— 自瞄基准与调参（可选）

把仿真当真值源，做“预测-提前量-开火门”的离线评估，不是视觉检测器本身。
当前最好结果（`bench_tune.cfg`，靶车不死，单轮约 120 发 ±8 个点）：

| 链路 | 命中率 |
|---|---|
| 状态源 + 理想角点 + 到达朝向门 0.75 | **78.8%** |
| 图像 + 理想角点 | ~66% |
| 图像 + rp0526 检测 | 44–56%（不稳） |

剩余差距主要在图像链路 ~0.30 s 的预测时域（取帧传输 31–47 ms + 推理 27–33 ms + 消费 16 ms）。

### Daedalus（Bevy 仿真，数据源之一）

仓库里原有的 Bevy 模拟器：能量机关、前哨站、大小装甲模块、步兵/英雄行为，ROS2 与 Talos 零拷贝两种接口。
在本项目里它扮演和 Godot `rm_aim_sim` 同级的角色——**给终端提供画面与裁判数据的仿真源**，不是主角。

```powershell
run_daedalus.bat                       # 起模拟器
cd tools\operator_package && Start.bat # 起终端（mode=sim）
```

---

## 目录结构

```
tools/
  rmu_protocol/        协议、图传、0x0310、Mock、仿真接入     ← 客户端核心
  operator_web.py      终端服务（HTTP 5002 + /api/rmu/*）
  operator_ui.html     终端前端（HUD、地图、指令、接管）
  operator_package/    启动脚本与配置（run_sim_rmu.bat 等）
  e2e_rmu.py           浏览器端到端测试（Playwright）
  test_rmu_protocol.py 协议单测
  rm_aim_sim/          自瞄基准 + 仿真接入 + 分发包模板
  ui0301_host/         0x0301 的 PC 编译 harness
  ui0301_preview.py    0x0301 帧解码与渲染
  rmu_field_map.py     场地底图烘焙
docs/                  协议、集成、规则、现状文档
src/ crates/ assets/   Bevy 模拟器 Daedalus
```

---

## 测试

```bash
cd tools
python -m unittest test_rmu_protocol test_ui0301_preview     # 34 项
```

浏览器端到端（需要 Mock 在 3343、终端在 5012，Chrome + Playwright）：

```bash
python e2e_rmu.py                                            # 36 项
```

e2e 覆盖：HUD 各字段、事件/判罚/增益/模块离线/机制状态的显示文本、指令成功与失败回执、地图点击换算、接管与撤销、跨源拒绝、3334 画面不冻结。

---

## 已知限制

**只在仿真和 Mock 上验证过，没有接真实裁判系统。** 上真机前必须逐项确认：

1. `RadarInfoToClient` 字段布局（手册没给字段号，现在是推测）——错了整体错位
2. `MapClickCmd/Info` 的坐标单位与场地尺寸、字段 3/6 的 mode/type 语义
3. 鼠标增量、滚轮 `mouse_z`、灵敏度是否符合固件预期
4. 真实 1080p60 HEVC 的端到端解码性能（浏览器 H.265 支持）
5. 0x0310 真机链路（机器人侧串口发送）未接
6. 0x0301 UI 未上车（电控 `robot.c` 的裁判线程还被注释着）

另外：仿真画面依赖 6 MB 裸帧走临时文件，这是当前图像链路延迟的主因（改进方案见 `tools/rm_aim_sim/README.md`）。

---

## 文档

| 文档 | 内容 |
|---|---|
| `docs/rmu_client_dev.md` | 开发交接：跑法、数据流、文件地图、0x0310 细节 |
| `docs/custom_client_plan_20261026.md` | 现状与下一步（含电控侧实测结论 §3.1/§3.2） |
| `tools/rmu_protocol/README.md` | 协议与 topic 表 |
| `tools/rm_aim_sim/README.md` | 仿真接入、命中率与调参流程 |
| `docs/rm_aim_sim_integration.md` | 轻量仿真集成设计 |
| `docs/narrowband_video.md` | 窄带图传实验 |

**安全约定**：MQTT 按 RM2026 协议无账号、无 TLS，必须放在比赛专用网络；Mock 只绑定 `127.0.0.1`。

---

<div align="center">

原 Bevy 模拟器部分遵循仓库 `LICENSE`。协议实现按官方手册整理，规则引用以队内解读文档为准。

</div>
