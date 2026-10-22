# rm_aim_sim ↔ 操作终端：语义映射

运行步骤见 `tools/rm_aim_sim/README.md`。本文只讲**为什么这么接、字段怎么对、
哪些是实测过的、哪些还是猜的**。

## 1. 为什么不是"接 RMU 协议"

两条路都能叫"接入"，代价差一个数量级：

| 方案 | 要做的事 | 结论 |
|---|---|---|
| **A. 冒充 Daedalus 本机接口**（已采用） | 实现 UDP 5001 租约 + HTTP 5000 画面 | 终端零改动，一天落地 |
| B. 冒充裁判系统 | 合成 37 个 ProtoBuf 消息 + MQTT 服务器 + HEVC 编码/UDP 3334 分片 | **2026-10-21 已实装**：`tools/rmu_protocol/sim_engine.py` 复用 `mock_engine` 的消息构造与图传，只把数据源/控制换成仿真；入口 `tools/operator_package/run_sim_rmu.ps1`，说明见 `docs/rmu_client_dev.md` §3.4 |

通道 A 本来就是终端的"模拟器通路"（`DAEDALUS_SIM_*`），B 是比赛用的官方通路。
用 A 做算法/界面联调，B 留到真实赛事引擎联调时再验；两者在终端里是并列的两个视频源。

## 2. 契约对照

终端连的这份契约来自 `src/operator.rs`（Daedalus 侧），桥接必须逐条对齐，
否则前端会一直显示「✗ 指令未生效」。

### 2.1 终端 → 桥接（UDP 5001，JSON）

| 字段 | 桥接怎么用 |
|---|---|
| `session` | 与桥接自己维护的会话号严格相等，不等回 `session_mismatch` |
| `seq` | 必须严格递增，否则 `duplicate_or_reordered` |
| `sent_ms` / `ttl_ms` | `ttl` 只接受 1–500；`now-sent >= ttl` 判过期 |
| `aim_enabled` | 开自瞄：桥接开始用真值解算目标 |
| `fire_authorized` | 允许自动开火（必须先开自瞄，否则 `authorization_requires_aim`） |
| `spin_enabled` | 本仿真无底盘，只回 `false` 并在 `reason` 里注明 |
| `mode` | `command` = 指挥（自瞄），`manual` = 全手动 |
| `manual.gimbal` | 全手动时的云台速度轴（±1），桥接按 6 rad/s 积分成绝对角 |
| `manual.movement` / `shoot` | 底盘没有，`shoot` 映射成 `fire` |

应答是终端状态快照，字段名照抄 Daedalus：

```json
{"session": 1792494440278, "applied_seq": 338, "remote": true, "mode": "command",
 "aim_enabled": true, "fire_authorized": true, "spin_enabled": false,
 "reason": "firing | tracking 3.8m 156ms | operator_timeout | session_mismatch",
 "shot_count": 9, "position": [0.0, 0.0], "chassis_heading": 0.0, "gimbal_heading": -0.37,
 "manual": {"movement": [0,0], "gimbal": [0,0], "shoot": false}}
```

`session` 由仿真的 `session_id` 取 SHA-1 前 12 位十六进制转整数（终端只做相等比较）；
`applied_seq` 取仿真 `ipc.last_seq`；`shot_count` 取 `stats.fired`；
`position` 取云台原点世界坐标（RM 系 x/y）。

### 2.2 桥接 → 仿真（`command.json`）

| 终端意图 | `command.json` |
|---|---|
| 指挥 + 自瞄 | `control=true`，`yaw/pitch` = 解算出的绝对角 |
| 授权开火且已对准 | `fire=true`（门限默认 0.45°，`--tolerance-deg`） |
| 全手动 | `control=true`，`yaw/pitch` 由速度轴积分，`fire=manual.shoot` |
| 撤销/超时 | `control=false, fire=false` |

必须每次改 `seq`：仿真的 `bridge.gd` 按 `JSON.stringify` 去重，内容完全相同的包
不会被重新消费，250 ms 后会被它自己的看门狗判超时。桥接以 60 Hz 重写，天然满足。

### 2.3 图像（HTTP 5000）

终端要的是 `PIL.Image.open()` 能读的东西，桥接直接从 `frame-*.raw`（rgba8，
1440×1080，无压缩）转 JPEG，宽超过 1280 时缩放，缓存 60 ms。
单帧实测：渲染 3–9 ms、回读 9–12 ms、发布 6–10 ms，端到端 37–51 ms。

## 3. 自瞄怎么解（为什么用真值）

仿真在 `state.json` 里给了每块装甲的 `center` / `normal`，以及弹速、重力、
枪管长度、云台限位——和它自己 `aim_assist.gd` 用的是同一批量。桥接照它的弹道
公式（含枪管前伸、迭代飞行时间）解一次，再补三件事让它能打移动靶：

1. **速度用窗口最小二乘**（默认 0.10 s），不是最后两点差分——60 Hz 状态下
   两点差分噪声太大；
2. **加速度用二次拟合**：目标换向是 3 m/s² 的加减速，线性外推在 156 ms 飞行里
   偏 3.6 cm，在 3.8 m 处正好一个装甲板宽（这一项是 44% → 71% 的来源）；
3. **把状态年龄补进预测**：`ahead = now − 状态采样时刻`。状态是 60 Hz 快照，
   云台追的永远是十几毫秒前的点，4 rad/s 下 16 ms 就是 3.7°，而装甲半宽只有 1°
   （这一项是 71% → 98% 的来源）。另外给伺服喂解本身的角速度做前馈。

这块是**验收脚手架**，不是视觉算法：它证明"终端按钮 → 云台 → 出膛 → 命中 → 扣血"
这条链是通的。换自己的检测时，把 `Tracker.solve()` 换成
"图 → 检测 → PnP → 跟踪"即可，其余（租约、限频、开火门限）原样复用。

> 门限的盲区：开火门限比较的是"云台 vs 预测点"，**看不见预测本身错多少**。
> 所以收紧门限降不了漏枪率，只会减少开火次数——真正要修的是提前量。

## 4. 实测结论（2026-10-20，Windows 11 + RTX 4060）

- Godot 4.7.2 win64 直接吃 4.4 的 `project.godot`，OpenGL Compatibility 渲染，
  资源导入一次通过；无界面 97 MB，图形 400 MB 内存（同场景的 Bevy 版明显更重）。
- 租约：`status` 应答 < 5 ms；指令通过率 100%（`rejected=0`），
  停止心跳 250 ms 后 `operator_timeout`。
- 图像：`operator.cfg`（曝光 1.0）下单帧 37–51 ms，终端 `/frame` 端到端 110–190 ms。
- 命中率（默认参数，每组 12 s）：
  - 标准（1.5 m/s 平移 + 2 rad/s 自转）：**96.9 / 98.6 / 98.5%**
  - 高难度（2 m/s + 4 rad/s）：**100 / 97.1%**
  - 调参前的基线是 44%，两处改动见上面第 3 节。
- 逐发归因：漏掉的都是 yaw（0.5–1.2°），pitch 一直 < 0.4°——所以优化方向在
  "指令时延 ↔ 自转"这一项，而不是弹道。

## 5. 还没做 / 需要真实数据

- **跨机**：现在是同机文件邮箱，桥接只能跑在仿真那台机器上。
- **小陀螺**：rm_aim_sim 没有底盘，`spin_enabled` 永远是 false。
- **车体命中、散布、噪声、运动模糊**：仿真本身不提供，别把它当命中率的真值。
- **RMU 通路（方案 B）**：真实赛事引擎的 `RadarInfoToClient` 字段布局、
  `MapClickInfo` 坐标单位还没验，见 `docs/rmu_client_dev.md` 第 6 节。
