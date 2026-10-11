# RMU2026 自定义客户端协议（V2.0.0）

`rm_custom_client.proto` 是我们唯一认的协议定义，按《RMU2026 通信协议 V2.0.0》第 2 章逐条整理。别从开源项目里复制 proto，它们都停在 V1.x。

## 传输约定

| 项 | 值 |
|---|---|
| MQTT | 3.1.1，服务器 `192.168.12.1:3333`，客户端 `192.168.12.2`，无 TLS、无账号 |
| clientID | 所连接机器人的 ID 编号（附录二：红 1~11，蓝 101~111） |
| topic | 消息名本身，例如 `GameStatus` |
| payload | ProtoBuf 二进制，需要 v3.15 以上 |
| QoS | 全部 ≤1 |
| 图传 | UDP 3334，客户端做服务端。HEVC 1080p60，无重传；每包 ≤1400B，前 8 字节大端：帧号 u16、分片序号 u16、帧总字节 u32 |

## Topic 表

| 方向 | 消息 | 频率 |
|---|---|---|
| 上行→机器人 | KeyboardMouseControl, CustomControl(≤30B) | 75Hz |
| 上行→服务器 | MapClickCmd | 触发，限频同 1.3.1 |
| 上行→服务器 | AssemblyCommand, RobotPerformanceSelectionCommand, CommonCommand, HeroDeployModeEventCommand, RuneActivateCommand, DartCommand, SentryCtrlCommand | 触发，≤10Hz |
| 上行→服务器 | AirSupportCommand | 触发 |
| 下行 | GameStatus | 5Hz |
| 下行 | RobotDynamicStatus | 10Hz |
| 下行 | CustomByteBlock(≤300B，来自机器人 0x0310) | 50Hz |
| 下行 | GlobalUnitStatus, GlobalLogisticsStatus, GlobalSpecialMechanism, RobotInjuryStat, RobotRespawnStatus, RobotStaticStatus, RobotModuleStatus, RobotPosition, TechCoreMotionStateSync, RobotPerformanceSelectionSync, DeployModeStatusSync, RuneStatusSync, SentryStatusSync, DartSelectTargetStatusSync, AirSupportStatusSync | 1Hz |
| 下行 | Buff | 获得时触发，之后 1Hz |
| 下行 | Event, PenaltyInfo, RobotPathPlanInfo, MapClickInfo, RadarInfoToClient, SentryCtrlResult | 触发 |

## 相对手册原文的改动

改动只为让文件能编译，线上字节格式保持不变：

1. `CustomControl` 原文用的是全角分号，已改成半角。
2. `RadarInfoToClient` 原文写成 `repeated uint32 RadarSingleRobotInfo;`，没有字段编号。现按「按顺序发送以下结构」理解为 `repeated RadarSingleRobotInfo robot_info = 1`，每次 12 组。**这一条未验证**，需要拿真实赛事引擎抓包确认。
3. 加了 `package rmu2026.client`。包名不进入线上字节。

阅读时的坑：`MapClickInfo` / `MapClickCmd` 里字段 3 叫 `mode`，实际含义是「标记类型」；字段 6 叫 `type`，实际含义是「标记模式」。线上只认编号，这里保留原名，含义以表格为准。

## 与 SharkDataServer（V1.0.0）的差异

对照的是本机 `D:\HNU_RM\代码栈\SharkDataSever-main\SharkDataSever-main\proto\messages.proto`。

- 改名：`RemoteControl` 拆成 `KeyboardMouseControl` 和 `CustomControl`；`MapClickInfoNotify` 拆成 `MapClickCmd`（上行）和 `MapClickInfo`（下行）；`GuardCtrl*` → `SentryCtrl*`；`SentinelStatusSync` → `SentryStatusSync`；`RaderInfoToClient` → `RadarInfoToClient`。
- 新增：`CommonCommand`（兑换发弹、复活、远程补血）。
- 编号重排：`GlobalUnitStatus` 从 6 号起全部变了，V2 加入了对方基地和前哨站字段，`robot_health` 从 6 移到 11。沿用旧 proto 的话，血量会被解析成对方基地血量。
- 类型变化：`DartSelectTargetStatusSync.open` 由 bool 改为 uint32（0/1/2）；`RuneStatusSync.average_rings` 由 uint32 改为 float。
- 追加字段：GameStatus 9/10、AirSupportStatusSync 4/5、DartCommand 3、RobotModuleStatus 11、RobotPosition 5、RobotPerformanceSelection* 3、TechCoreMotionStateSync 3~8。
- V2 删除：`Buff.msg_params`（6）。

## 校验

```powershell
python -m pip install "grpcio-tools==1.75.1" "protobuf==6.32.1"
python -m grpc_tools.protoc -I tools/rmu_protocol --python_out=<out> tools/rmu_protocol/rm_custom_client.proto
python tools/rmu_protocol/verify_proto.py <手册导出的 txt> <out>
```

`verify_proto.py` 会从手册文本里抽出全部 37 个 message、174 个字段，逐一比对编号、类型、名字和 repeated，并做几组线上字节回环测试。`RadarInfoToClient` 因为有意偏离原文，不参与比对。
