# 第三方组件与许可

本部署包把三部分装在一起，各自的许可互不影响：

| 组件 | 来源 | 许可 |
|---|---|---|
| `components/sim/rm_aim_sim` | [ForgetWW520/rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim)（含 1 个本地适配补丁） | MIT，Copyright (c) 2026 ForgetWW520 and contributors |
| `third_party/Godot_v4.7.2-stable_win64.exe` | [Godot Engine 4.7.2-stable](https://godotengine.org/download/windows/) | MIT，Copyright (c) 2014-present Godot Engine contributors；Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur |
| `components/client`、`components/aim`、`deploy.ps1` 等 | 本仓库（Daedalus 操作终端） | 见仓库根目录 `LICENSE`（AGPL-3.0） |

## rm_aim_sim 的第三方素材

上游把装甲数字贴图、历史车体模型单独声明了来源，随源码一起分发，正常启动不需要联网
下载。逐文件来源与哈希见上游 `THIRD_PARTY_NOTICES.md`、`assets/stickers/sources.json`、
`assets/third_party/` 下的许可文件——这些文件在本包内保留原样，请勿删除。

## Godot 引擎

引擎本体不在上游 Git 仓库里，本包额外打进了官方 4.7.2-stable 的 Windows 版本，
下载与校验由 `get_godot.ps1` 完成。Godot 采用 MIT，并附带一批第三方库声明，
完整清单见 <https://godotengine.org/license/>。**对外分发本包时请一并保留该声明。**

已在本机核对：`Godot_v4.7.2-stable_win64.exe --version` 输出
`4.7.2.stable.official.ed1daf0bf`，与 Release 页一致。

## 修改说明

`components/sim/rm_aim_sim` 相对上游只改了一个文件：`scripts/local_ipc.gd`
（共 36 行，让共享内存邮箱在非 Linux 平台可用）。补丁原文见
`components/sim/patches/0001-local-ipc-cross-platform.patch`，可直接
`git apply` 到干净的上游仓库复现。
