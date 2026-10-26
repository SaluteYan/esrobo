# ESROBO 开发文档

[返回平台首页](../README.md) · [设备驱动](../src/README.md) · [机器人通信](../robot_link/README.md)

本目录收录 ESROBO 机器人及部件手册，并提供平台开发的阅读路线。具体操作参数以当前硬件、固件和部署配置为准；历史调试记录用于解释已有配置。

## 整机与厂家手册

| 手册 | 覆盖内容 | 适用场景 |
| --- | --- | --- |
| [移动仿人形双臂机器人平台操作手册](移动仿人形双臂机器人平台操作手册.pdf) | ESROBO 整机组成与基础操作 | 初次认识平台、现场上电和设备操作 |
| [NERO 用户手册](NERO_用户手册.pdf) | 七轴机械臂说明 | 机械臂安装、接口与厂家操作流程 |
| [RANGER MINI 3.0 用户手册](<RANGER MINI 3.0_用户手册.pdf>) | 移动底盘说明 | 底盘接入与使用 |
| [Linker Hand L20 Lite 产品手册](<Linker Hand L20 Lite 产品手册202600509_1783585598663.pdf>) | 灵巧手产品说明 | 型号、结构与部件使用 |

保留原始文件名和厂家内容。软件中的关节映射、限位、零位及通道转换还需同时核对当前配置，手册不能替代平台标定。

## 平台开发指南

| 主题 | 入口 | 你可以找到的内容 |
| --- | --- | --- |
| 硬件接入 | [HARDWARE_BRINGUP.md](../HARDWARE_BRINGUP.md) | CAN/串口、设备命名、逐部件启动、反馈检查和现场问题 |
| 驱动选择 | [src/README.md](../src/README.md) | 驱动与设备对应关系、ROS 包及 SDK 入口 |
| 机器人模型 | [模型说明](../HARDWARE_BRINGUP.md#整机-urdf-模型)、[URDF](../teleoperation/urdf/esrobo_waist_with_head.urdf) | 整机连接、NERO 关节语义和灵巧手联动 |
| 网络接入 | [robot_link/README.md](../robot_link/README.md) | mock 端点联调、网关部署、客户端和图像通道 |
| 协议设计 | [PROTOCOL.md](../robot_link/PROTOCOL.md) | 认证、会话、合同、目标、反馈及有效期 |
| 执行行为 | [ROBOT_EXECUTION.md](../robot_link/ROBOT_EXECUTION.md) | 收到目标后的约束、下发、反馈与状态切换 |

## 平台应用资料

PICO 本机遥操作的环境、网络、网页、计算和验证资料位于 [应用文档目录](../laptop_teleop/docs/01_README.md)。IsaacLab 仿真、资产和回放说明见现有 [Dual-arm-teleoperation 仓库](https://github.com/SaluteYan/Dual-arm-teleoperation)。应用专用算法与实验记录由各项目维护，平台文档集中解释可复用设备与接口。

## 阅读顺序

1. **搭建机器人**：整机手册 → 逐部件调试 → 对应驱动 README。
2. **开发控制应用**：协议与关节语义 → mock 网关 → 执行流程 → 实机部署和验收。
3. **使用遥操作项目**：应用 README → 环境与连接 → 输入预览与标定 → 日常操作。
4. **开展仿真实验**：仿真仓库的版本清单 → 模型资产 → 仿真环境 → 复现与回放。

文档更新应说明适用源码版本及验证范围，区分静态检查、离线模拟、历史现场记录与本次实机结果。
