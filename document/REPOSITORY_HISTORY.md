# HUSTESROBO 组织与仓库整理

本文保留本次组织发布的历史记录，目录描述以发布时快照为准。当前工作区已将 `laptop_teleop/` 合并到 `teleoperation/`，并删除旧单机入口；维护说明见 [遥操作 README](../teleoperation/README.md)。组织介绍的发布源现由 `/home/yan/PhdResearch/HUSTESROBO/.github/` 独立维护，本工作区不再保存重复的 profile 副本。

## 目标与已确认选择

以 [XR-Robotics](https://github.com/XR-Robotics) 的组织展示结构为参考，建立“组织主页 → ESROBO 平台 → 独立应用项目”的导航。用户已确定组织地址为 **https://github.com/HUSTESROBO**，保留 `SaluteYan/esrobo` 原仓库，在新组织另建平台仓库，不转移或覆盖原仓库。

[已发布的组织介绍](https://github.com/HUSTESROBO/.github/blob/main/profile/README.md) 是组织主页的可发布正文，应放入 **`HUSTESROBO/.github` 仓库的 `profile/README.md`**。它不是当前 `esrobo` 仓库中的特殊 GitHub 配置。GitHub 的组织主页机制见 [官方说明](https://docs.github.com/en/organizations/collaborating-with-groups-in-organizations/customizing-your-organizations-profile)。

## 仓库职责

| 仓库 | 职责 | 来源／状态 |
| --- | --- | --- |
| `HUSTESROBO/.github` | 组织介绍、平台文档入口、项目目录 | 独立 `.github` 仓库中的 `profile/README.md`，已发布并核验组织 Overview |
| `HUSTESROBO/ESROBO` | 机器人驱动、设备文档、模型、通信协议与共用执行组件 | 从当前固定版本导出新平台快照；保留第三方来源与子模块版本 |
| `esrobo-teleop`（建议名称） | PICO 实机遥操作、网页、输入与重定向 | 当前 `laptop_teleop/`，待独立化；暂链接原仓库真实入口 |
| 仿真项目（名称待整理） | IsaacLab 环境、资产、遥操作与回放 | 已有 `SaluteYan/Dual-arm-teleoperation`，暂链接原仓库，不自动迁移 |

组织显示名称使用 ESROBO，GitHub 命名空间使用用户创建的 HUSTESROBO。只在新项目仓库实际建立并验证后更新链接，不建立没有内容的展示占位项目。

## 平台仓库的初始边界

首版平台快照包含 `src/`、`document/`、`robot_link/`、`teleoperation/` 及硬件初始化文件、平台 README 和硬件调试说明。`laptop_teleop/` 的应用源码不复制进新平台仓库，相关文档链接指向原仓库固定源码版本。

保留 `teleoperation/` 是为了保持机器人执行所需的公共模块完整，不代表把全部旧目录定位为仿真项目。当前依赖如下：

- `robot_link/esrobo_link/backends.py` 直接导入 `esrobo_teleop` 的配置、NERO／手部驱动、碰撞及诊断模块。
- `laptop_teleop` 导入 `esrobo_teleop` 的人体映射、IK 与配置，同时使用 `esrobo_link`；安装器安装三个相邻包。
- `laptop_teleop/config/laptop.yaml`、安装和启动脚本使用仓库相对路径；直接移动目录不能形成可独立安装的项目。
- 当前完整 IsaacLab 仿真基线在 `SaluteYan/Dual-arm-teleoperation`；本地 `teleoperation/scripts/sim_body_input.py` 只生成测试输入。

后续可将模型、配置及执行适配器提取为平台包，将应用算法独立为项目包。需要同时改造配置路径、包依赖、模型资源发现、安装脚本与两端合同，并验证独立检出能运行；不能仅靠文件移动或改名完成。这一轮展示整理不改动运行算法。

## 发布内容与版本

平台快照基于原仓库 `318b1d04786f1b6b11254e62b7628178910cd48c`（实验分支），叠加本次平台文档整理。新平台从独立初始提交开始，不复制旧仓库完整历史；源码来源在新平台 `SOURCE.md` 中记录。原仓库与原分支保持可用。

平台导出保留三个 gitlink 的固定 SHA，并补齐原 `.gitmodules` 缺失的 `ugv_sdk` 地址。不复制工作区未跟踪文件、密钥、标定、环境、日志或构建输出。模型所需大文件与外部资产仍按来源说明获取，不能把源码快照写成已完成硬件部署。

## 主页展示与维护

组织主页依次展示：平台介绍 → 硬件与基础资源 → 通信链路 → 应用项目 → 开始开发。平台仓库首页详述驱动、文档和协议；各项目 README 负责具体安装与算法实验。

建议先置顶 `esrobo`，待实机／仿真项目独立发布后再置顶这些仓库。组织的实际置顶需组织所有者在 GitHub 页面操作。未提供真实机器人照片或演示素材时，主页使用架构图和真实代码入口，不使用替代机器人图片。

## 2026-10-26 整理记录

- 重写平台 README，新增 `src/README.md` 与 `document/README.md`；补充通信模块开发导航，准备组织 profile。
- 原有 `laptop_teleop/docs/08_CHANGELOG.md` 中本地同步记录保留，未覆盖。
- 当前工具无组织／仓库创建能力且无可用浏览器，组织由用户创建；两个目标空仓库由用户建立后继续发布。
- 文档链接、导出清单和发布版本需完成后核验；本轮不执行机器人运动或重启设备服务。

## 本地交付目录

两个独立 Git 仓库准备在 `/home/yan/PhdResearch/HUSTESROBO/`：`.github/` 保存组织主页，`esrobo/` 保存平台源码快照。原工作区只修改文档，运行代码和原远程未修改。

本地入口文档 119 个相对链接已检查，平台导出入口文档 105 个相对链接已检查；非文档文件内容核对、语法检查及网关 CLI 检查通过。平台导出补齐 UGV SDK 子模块映射，未初始化或更新各子模块工作区。远程发布状态以最后的 Git 引用核验为准。

### 发布准备状态（历史）

- 平台初始提交：`c7467ef`，本地 `main`；目标 `git@github.com:HUSTESROBO/ESROBO.git`。
- 组织主页初始提交：`0e3aff3`，本地 `main`；目标 `git@github.com:HUSTESROBO/.github.git`。
- SSH 已确认认证为 `SaluteYan`；准备阶段两个目标均返回 `Repository not found`。尚未向新组织推送，不能将本地提交理解为主页已经上线。
- 两个仓库可访问后，先发布平台，再发布 profile，最后核对远程 main SHA 与组织 Overview。原 `SaluteYan/esrobo` 不作为此次推送目标。

### 2026-10-26 发布完成

- 用户已建立公开仓库 `HUSTESROBO/.github` 与 `HUSTESROBO/ESROBO`；实际平台名称为大写 ESROBO，组织主页链接及平台本地 origin 已使用该规范名称。
- 平台 `main` 已推送 `c7467ef0e145c8518f85e6ae3df0292298f83fde`；组织主页 `main` 已推送 `7a6f91b200e017a9e95e268f637ece8c2f64226d`（含规范仓库链接修订）。远程分支 SHA 与本地 HEAD 一致。
- 直接获取公开 HTML 验证组织主页和平台仓库均返回 HTTP 200；组织首页包含平台介绍、文档／驱动／协议入口和“平台上的项目”，平台首页包含基础开发资源与应用导航。
- 通过 GitHub 连接器进一步读取远程 `document/README.md`、`src/README.md`、`robot_link/PROTOCOL.md`，均成功返回对应内容。新仓库公开性与默认 main 分支也已核对。
- 访问入口：[组织主页](https://github.com/HUSTESROBO)、[平台仓库](https://github.com/HUSTESROBO/ESROBO)。两个发布工作区保持干净。原 `SaluteYan/esrobo` 未推送、未迁移；原工作区文档修改仍保留本地。
- 本次完成平台与组织主页发布；应用项目继续链接原仓库，本次没有声称完成应用代码独立化、组织置顶设置或实机验收。
