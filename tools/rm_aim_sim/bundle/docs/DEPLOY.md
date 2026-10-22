# 逐组件部署与验收

三个组件可以任意组合：**仿真 + 自瞄**是闭环，**客户端**是界面。它们之间只有两条接口
（UDP 5001 租约、HTTP 5000 画面），所以换掉的成本很低——比如把自瞄换成自己团队的视觉程序，
只要它同样实现这两个接口。

## 1. 仿真 `components/sim`

**依赖**：Windows x64 + OpenGL 3.3 显卡驱动；Godot 4.7.2（`third_party` 里自带）。
不需要 Python、ROS、CUDA。

```powershell
.\components\sim\start.ps1                     # 图形模式，出图
.\components\sim\start.ps1 -Headless           # 无界面：只跑运动/云台/状态
.\components\sim\start.ps1 -Scenario <path.cfg> -Channel other
```

- 启动参数实际是 `Godot --path <工程> -- --remote --config=<场景>`；
  `--remote` 打开「本机自瞄控制」，不打开的话外部指令全被拒。
- 首次运行会自己导入资源（`.godot`，约 20 秒）。
- 日志：`runtime\sim.log`、`runtime\sim.err.log`。
- 场景：`operator.cfg`（曝光 1.0、1.5 m/s 平移 + 2 rad/s 自转）。要换靶车行为就改它：
  `translate`/`speed`/`travel`/`rotate`/`omega`/`max_health`。

**单独验收**：窗口里能看到四块装甲在动；`%TEMP%\rm_aim_sim_default\state.json`
每秒都在变（`published` 递增）。

## 2. 本机自瞄 `components/aim`

**依赖**：Python 3.10+ 与 Pillow（只用来把原始帧转 JPEG；`-NoVideo` 时不需要）。
必须和仿真**同一台机器、同一个用户**，因为走的是文件版共享内存。

```powershell
.\components\aim\start.ps1                     # 带画面
.\components\aim\start.ps1 -NoVideo            # 只跑控制/租约
.\components\aim\start.ps1 -OperatorPort 5001 -VideoPort 5000 -ToleranceDeg 0.35
```

- 日志：`runtime\aim.log`、`runtime\aim.err.log`。
- 自瞄解算：仿真真值 + 弹道（枪管前伸、迭代飞行时间）+ 目标平移（含**加速度**）
  与自转预测，并把**状态年龄**补进预测。命中率实测 97–100%（见 README 的「命中率」）。
- 三个可调参数（都有实测过的默认值，一般不用动）：

  | 参数 | 默认 | 什么时候动 |
  |---|---|---|
  | `-ToleranceDeg` | 0.45 | 想更保守（少开火、几乎不空枪）就降到 0.3 |
  | `-LeadWindow` | 0.10 | 目标运动突变多就调小（更跟手），抖动多就调大 |
  | `-ReleaseLatency` | 0 | 加正值会让 4 rad/s 自转的靶更差，别加 |

- **这是验收脚手架**，不是视觉算法；换自己的检测时替换
  `rm_aim_sim_bridge.py` 里的 `Tracker.solve()`，其余（租约、限频、开火门限）复用。

**单独验收**（不需要页面）：

```powershell
python components\aim\rm_aim_sim_bridge.py --diagnose   # 读一次仿真状态
curl http://127.0.0.1:5000/snapshot/raw -o shot.jpg    # 当前画面
```

## 3. 自定义客户端 `components/client`

**依赖**：Python + Pillow（画面）；直连裁判系统时还要 `paho-mqtt`、`protobuf`、`av`。
`deploy.ps1 -Action install` 会按 `components/client/requirements.txt` 装进 `.venv`。

```powershell
.\components\client\start.ps1                  # 连本机自瞄（默认）
.\components\client\start.ps1 -Rmu             # 连官方裁判系统 192.168.12.1:3333
.\components\client\start.ps1 -Mock            # 本机 Mock 裁判系统（无真机自测）
.\components\client\start.ps1 -SimHost 127.0.0.1 -SimPort 5001 -ListenPort 5002
```

- 日志：`runtime\client.log`、`runtime\client.err.log`（Mock 另有 `runtime\mock.*`）。
- 纯客户端模式（`-Rmu`）下页面「显示链路」选「官方图传 UDP 3334（HEVC）」，
  不依赖仿真和自瞄，只在拿到真机/赛事引擎时用。

**单独验收**：
`curl http://127.0.0.1:5002/api/state` 返回 `error: ""` 且 `age_ms` 在 200 以内。

## 4. 常见组合

| 目的 | 命令 |
|---|---|
| 全套调试 | `.\deploy.ps1` |
| 只验自瞄闭环，不用浏览器 | `.\deploy.ps1 -Component sim,aim` |
| 跑基准（无界面，省 GPU） | `.\deploy.ps1 -Component sim,aim -Headless` |
| 只调界面，仿真在别人机器上 | `.\deploy.ps1 -Component client -Action start`（改 `config.json` 的 `bind`/端口指向对端） |
| 只连真机裁判系统 | `.\deploy.ps1 -Component client -Action start -Rmu` |
| 重新装依赖 | `.\deploy.ps1 -Action install` |

## 5. 跨机部署的边界

- **自瞄必须和仿真同机**：共享邮箱是文件，不是网络协议。想让自瞄跑在别的机器上，
  得把 `rm_aim_sim_bridge.py` 的 `Channel` 换成 TCP 或 HTTP 拉取——那是另一个工程。
- **客户端可以跨机**，但要注意两件事：
  1. 把 `config.json` 的 `bind` 改成 `0.0.0.0`（否则只监听本机回环）；
  2. `operator_web.py` 用 `DAEDALUS_OPERATOR_HOST` 做同源校验，
     客户端 start.ps1 已经把它设成 `bind` 的值；如果 `bind=0.0.0.0`，浏览器用局域网 IP 访问
     会被判跨源（403），这时把 `bind` 写成**具体网卡地址**（如 `192.168.1.20`）最省事。
- 图传（`/frame`）是拉取式 HTTP，跨机时带宽就是 JPEG 码率（960 宽约 20 KB/帧，5 fps 约 0.8 Mbit/s）。

## 6. 验收基线（本机实测，RTX 4060 / Windows 11）

| 指标 | 数值 |
|---|---|
| 仿真内存 | 无界面约 97 MB，图形约 400 MB |
| 单帧取图 | 渲染 3–9 ms + 回读 9–12 ms + 发布 6–10 ms，端到端 37–51 ms（1440×1080 rgba8） |
| 终端 `/frame` 端到端 | 110–190 ms（960 宽 JPEG） |
| 命中率（1.5 m/s 平移 + 2 rad/s 自转） | 12 秒 65–70 发，**97–98.6%**（三次） |
| 命中率（2 m/s + 4 rad/s） | 12 秒 65 发左右，**97–100%** |
| 撤销 | 停止心跳后 250 ms 转 `operator_timeout`，之后不再写指令 |

## 7. 清理

```powershell
.\deploy.ps1 -Action stop        # 停进程（按 runtime\pids.json）
Remove-Item .venv, runtime -Recurse -Force      # 删依赖与日志
```

`%TEMP%\rm_aim_sim_<channel>` 由仿真自己回收；如果进程被强杀留下残留，
删掉该目录即可（确认没有活着的仿真进程）。
