# RoboMaster 自瞄仿真部署包

三件套打在一个包里，**可以一键起全套，也可以只起其中任意一个**：

| 组件 | 目录 | 干什么 | 对外接口 |
|---|---|---|---|
| **仿真** | `components/sim` | Godot 4 的 RoboMaster 自瞄实验场（rm_aim_sim），出图、算弹道、判命中 | 共享邮箱 `%TEMP%\rm_aim_sim_default` |
| **本机自瞄** | `components/aim` | 从仿真读真值/画面，解算提前量，把云台指令写回仿真 | UDP 5001 租约、HTTP 5000 画面 |
| **自定义客户端** | `components/client` | 浏览器 HUD、键鼠/指令、图传显示；也能直连官方裁判系统 | HTTP 5002 页面 |

三个组件之间只靠 **UDP 5001 + HTTP 5000** 这一对接口说话，所以拆开部署不需要改代码。

## 一键起全套

双击 `一键部署.bat`，或者：

```powershell
powershell -ExecutionPolicy Bypass -File .\deploy.ps1
```

它会依次：起仿真（外部控制模式）→ 等邮箱就绪 → 起自瞄桥接 → 等端口 → 起客户端 → 开页面
<http://127.0.0.1:5002/>。

首次在新机器上先装一遍：

```powershell
.\deploy.ps1 -Action install     # .venv + Python 依赖 + 校验 Godot + 首次导入资源
.\deploy.ps1 -Action check       # 只检查，不动任何东西
```

## 分别部署

```powershell
# 只起仿真（自己用 tools/control_demo.py 之类接）
.\deploy.ps1 -Component sim -Action start

# 只起自瞄（仿真已经在别处开着）
.\deploy.ps1 -Component aim -Action start

# 只起客户端：连本机自瞄，或者连官方裁判系统
.\deploy.ps1 -Component client -Action start
.\deploy.ps1 -Component client -Action start -Rmu            # 需要再加 -Rmu 走真机

# 组合
.\deploy.ps1 -Component sim,aim -Action start                # 不起页面，纯跑闭环
.\deploy.ps1 -Component sim,aim,client -Headless -Action start  # 无界面：验控制链，不出图
```

每个组件也能单独跑，不依赖这个脚本：

```powershell
.\components\sim\start.ps1      [-Headless] [-Channel default] [-Godot <exe>]
.\components\aim\start.ps1      [-NoVideo] [-OperatorPort 5001] [-VideoPort 5000]
.\components\client\start.ps1   [-Rmu] [-Mock] [-ListenPort 5002] [-NoBrowser]
```

停止与查看：

```powershell
.\deploy.ps1 -Action status
.\deploy.ps1 -Action stop
```

## 端口与配置

都在 `config.json` 里改，三个组件共享：

| 键 | 默认 | 说明 |
|---|---|---|
| `channel` | `default` | 仿真与自瞄之间的邮箱名，两边必须一致 |
| `bind` | `127.0.0.1` | 监听地址；只在本机用不要改 |
| `operator_port` | 5001 | 租约（UDP），客户端 ↔ 自瞄 |
| `video_port` | 5000 | 画面（HTTP），客户端 ← 自瞄 |
| `client_port` | 5002 | 浏览器页面 |
| `tolerance_deg` | 0.45 | 开火角度门限 |
| `python` / `godot` | 空 | 留空则自动找（`.venv` → PATH → 常见路径） |

## 命中率

默认参数已经是调好的：标准靶（1.5 m/s 平移 + 2 rad/s 自转）实测 **97–98.6%**，
高难度靶（2 m/s + 4 rad/s）**97–100%**。三条经验：

- **别指望收紧门限提命中率**：门限比的是"云台 vs 预测点"，看不见预测本身的误差，
  收紧只会减少开火次数；
- 提前量必须带**加速度项**（目标换向是 3 m/s²，线性外推在 156 ms 飞行里偏一个装甲板宽）；
- 预测要补**状态年龄**（60 Hz 快照 + 指令生效延迟，4 rad/s 下 16 ms 就是 3.7°）。

要重测就用包里的 `components\aim\` 下那几个脚本，它们会自己起干净的仿真、按帧统计：

```powershell
python components\aim\bench_hit_rate.py --seconds 12     # 单组
bash   components\aim\sweep.sh 12                        # 扫一组参数
python components\aim\attribute_misses.py                # 逐发归因漏在哪
```

## 目录

```
deploy.ps1 / 一键部署.bat     统一入口
config.json                   共享配置
lib\common.ps1                公共函数（路径/端口/PID）
components\
  sim\  rm_aim_sim\           上游工程（含 Windows 适配补丁）
        operator.cfg          给终端看的场景（曝光 1.0、平移+自转）
        patches\              补丁原文，可复现
  aim\  rm_aim_sim_bridge.py  桥接本体
        bench_hit_rate.py     命中率基准（自带干净仿真）
        sweep.sh / diag_lead.py / attribute_misses.py   调参与归因
        bench.cfg / bench_hard.cfg                      基准场景（血量拉满）
  client\ app\                operator_web + 页面 + RMU 协议
third_party\
  Godot_v4.7.2-stable_win64.exe
  get_godot.ps1               没带引擎时自动下载
  NOTICE.md                   许可证
docs\
  DEPLOY.md                   逐组件细节、跨机、验收
  INTEGRATION.md              接口语义映射（为什么这么接）
runtime\                      日志与 PID（运行后生成）
```

## 排障速查

| 现象 | 处理 |
|---|---|
| `deploy.ps1` 报 `已添加相同键 HTTPS_PROXY` | 包里已自动清掉重复代理变量；若自己写脚本，别在 `Start-Process` 前保留大小写重复的 `*_proxy` |
| 页面「模拟器离线」 | `.\deploy.ps1 -Action status` 看 5001 是否监听；看 `runtime\aim.err.log` |
| 页面有 HUD 但画面 503 | 仿真窗口被最小化（Godot 最小化不渲染），恢复窗口即可 |
| 点「接管」后显示 `✗ 指令未生效` | 看 `runtime\aim.log`；多半是 `session_mismatch`（仿真重启过，重新点接管） |
| 命中率低 | 调 `config.json` 的 `tolerance_deg`（0.3 更准但更少开火） |
| 仿真起不来 | 同通道已有实例；`.\deploy.ps1 -Action stop` 或换 `-Channel` |

各组件的细节、单组件验收、跨机部署见 `docs\DEPLOY.md`；
接口字段为什么这么对，见 `docs\INTEGRATION.md`。
