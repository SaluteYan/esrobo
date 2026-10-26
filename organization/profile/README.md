<div align="center">

# ESROBO

### 移动双臂灵巧手机器人开发平台

面向机器人开发与研究，连接硬件、控制、遥操作和仿真。

[机器人平台](https://github.com/HUSTESROBO/ESROBO) · [开发文档](https://github.com/HUSTESROBO/ESROBO/tree/main/document) · [设备驱动](https://github.com/HUSTESROBO/ESROBO/tree/main/src) · [通信协议](https://github.com/HUSTESROBO/ESROBO/blob/main/robot_link/PROTOCOL.md)

</div>

---

**ESROBO** 是一套集成移动底盘、双七轴机械臂和双五指灵巧手的机器人平台。我们围绕这套硬件维护设备驱动、机器人模型、开发文档与通信接口，并通过独立项目探索遥操作、仿真和机器人操作方法。

## 机器人平台

平台由 Ranger Mini 3 移动底盘、两台 NERO 七轴机械臂、双 LinkerHand L20Lite 灵巧手、三轴腰部和头部舵机组成，并配备头部／腕部 RGB-D 相机、激光雷达和双六维力传感器。主要软件环境为 Ubuntu 22.04 与 ROS 2 Humble。

| 基础资源 | 提供的内容 | 入口 |
| --- | --- | --- |
| **ESROBO 平台仓库** | 硬件组成、系统架构、开发路线和应用导航 | [esrobo](https://github.com/HUSTESROBO/ESROBO) |
| **硬件与开发文档** | 整机、机械臂、底盘及灵巧手手册；接线、CAN/串口与逐部件调试 | [文档中心](https://github.com/HUSTESROBO/ESROBO/blob/main/document/README.md) |
| **设备驱动与 SDK** | 机械臂、灵巧手、底盘、腰部、头部、相机、雷达与力传感器 | [驱动索引](https://github.com/HUSTESROBO/ESROBO/blob/main/src/README.md) |
| **机器人通信** | Python 客户端、认证 UDP、配置合同、目标与反馈、执行网关 | [robot_link](https://github.com/HUSTESROBO/ESROBO/tree/main/robot_link) |
| **机器人模型** | 整机 URDF、关节语义、灵巧手 mimic 联动与运动组件 | [模型与控制组件](https://github.com/HUSTESROBO/ESROBO/tree/main/teleoperation) |

### 开发一条完整控制链路

```mermaid
flowchart LR
    A[应用项目<br/>输入、算法与目标生成] <-->|robot_link<br/>关节目标与反馈| B[机器人主机<br/>校验、轨迹与执行]
    B <--> C[CAN / ROS 2<br/>设备驱动]
    C <--> D[ESROBO]
```

应用端可以独立迭代输入方式和算法，机器人端复用驱动、反馈检查及运动约束。先在 mock 网关上验证通信，再按硬件流程联调。具体协议覆盖的设备、控制模式和验证边界见平台文档。

## 平台上的项目

### PICO 实机遥操作

通过 PICO Body/Hand 采集身体和手部追踪，在电脑端完成标定、重定向与 IK，经机器人网关执行；网页提供输入预览、服务管理、运行状态和日志入口。当前实验包含左手向量重定向及几何修正。

**代码与文档：** [laptop_teleop](https://github.com/SaluteYan/esrobo/tree/experiment/dex-vector-retargeting-20261021/laptop_teleop)。目前保留在原仓库，独立项目仓库整理中；以该分支文档说明适用配置和验证状态。

### 仿真遥操作与回放

围绕 ESROBO 提供 IsaacLab 仿真基线、机器人资产、PICO/SenseGlove 输入、录制回放及复现说明，用于算法开发与对照验证。

**代码与文档：** [Dual-arm-teleoperation](https://github.com/SaluteYan/Dual-arm-teleoperation)。这是现有独立项目，当前仍位于原作者账号下。

后续项目将按“项目说明、实际仓库、平台版本、运行与验证入口”持续加入。各项目独立维护算法、环境和实验；通用硬件与通信说明集中在平台仓库。

## 从这里开始

1. **认识机器人**：[平台总览](https://github.com/HUSTESROBO/ESROBO)与[设备手册](https://github.com/HUSTESROBO/ESROBO/tree/main/document)。
2. **搭建硬件环境**：[逐部件调试](https://github.com/HUSTESROBO/ESROBO/blob/main/HARDWARE_BRINGUP.md)与[驱动索引](https://github.com/HUSTESROBO/ESROBO/blob/main/src/README.md)。
3. **开发新应用**：[通信模块](https://github.com/HUSTESROBO/ESROBO/tree/main/robot_link)、协议及 mock 联调。
4. **选择研究项目**：按上面的项目入口安装对应环境、模型和输入设备。

硬件／协议问题在 [平台 Issues](https://github.com/HUSTESROBO/ESROBO/issues)反馈；应用问题在对应项目反馈，并附代码版本、设备／环境和复现步骤。
