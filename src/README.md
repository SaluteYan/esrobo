# ESROBO 设备驱动

[返回平台首页](../README.md) · [硬件调试](../HARDWARE_BRINGUP.md) · [设备手册](../document/README.md) · [通信接口](../robot_link/README.md)

本目录是 ESROBO 机器人主机的 ROS 2 源码工作空间，包含设备驱动、厂家 SDK 和集成工具。主要环境为 Ubuntu 22.04 / ROS 2 Humble；包名与源码目录名不一定一致，构建和启动时以各 `package.xml` 与 launch 为准。

## 运动与操作设备

| 设备 | 源码 | 职责与接口 | 详细说明 |
| --- | --- | --- | --- |
| NERO 七轴双臂 | [agx_arm_ros2](agx_arm_ros2/) | `agx_arm_ctrl`、消息、模型和 MoveIt 配置；ROS 2 控制与反馈 | [驱动 README](agx_arm_ros2/README.md)、[平台机械臂调试](../HARDWARE_BRINGUP.md#机械臂测试) |
| 机械臂 CAN SDK | [pyAgxArm-master](pyAgxArm-master/) | Python 厂家 SDK；当前遥操作的 NERO 适配器直接调用此库 | [SDK README](pyAgxArm-master/README.md)、[NERO API](pyAgxArm-master/docs/nero/nero_api.md) |
| 双 LinkerHand | [linkerhand-ros2](linkerhand-ros2/) | 灵巧手控制、位置反馈、设备设置及可视化 | [中文 README](linkerhand-ros2/README_CN.md)、[话题参考](linkerhand-ros2/doc/Topic-Reference.md) |
| Ranger Mini 3 | [ranger_ros2](ranger_ros2/) | `ranger_base` / `ranger_bringup` 底盘节点与启动配置 | [驱动 README](ranger_ros2/README.md)、[底盘调试](../HARDWARE_BRINGUP.md#底盘) |
| 底盘 SDK | [ugv_sdk](ugv_sdk/) | 底盘 CAN 通信的 C++ SDK，供 Ranger 驱动使用 | [SDK README](ugv_sdk/README.md) |
| 三轴腰部 | [erob_can_joints](erob_can_joints/) | `erob_canopen` 自定义 CAN 驱动与 ROS 2 节点 | [节点实现](erob_can_joints/erob_driver_node.cpp)、[腰部调试](../HARDWARE_BRINGUP.md#腰部关节) |
| 头部舵机 | [ros2_rs485_servo](ros2_rs485_servo/) | `servo_driver`，RS485 舵机控制、状态与调整接口 | [节点实现](ros2_rs485_servo/src/rs485_servo_driver.cpp)、[头部调试](../HARDWARE_BRINGUP.md#头部舵机) |

NERO 的 ROS 控制节点和直接 CAN SDK 是两种接入路径。开发应用时先明确由哪个进程拥有设备控制权，不同时向同一机械臂启动多个命令源。L20Lite 的 URDF 包含联动关节，现有物理控制使用十主动通道；模型关节数与可独立控制通道数不同。

## 感知设备

| 设备 | 源码与 ROS 包 | 职责 | 详细说明 |
| --- | --- | --- | --- |
| Orbbec 头部 RGB-D 相机 | [OrbbecSDK_ROS2](OrbbecSDK_ROS2/)，`orbbec_camera` | Gemini 435Le 等设备的彩色、深度和相机配置 | [中文 README](OrbbecSDK_ROS2/README_CN.MD)、[平台调试](../HARDWARE_BRINGUP.md#头部-orbbec-gemini-435le-相机) |
| RealSense 腕部 RGB-D 相机 | [realsense-ros](realsense-ros/)，`realsense2_camera` | 左右腕部 RealSense 图像与深度采集 | [README](realsense-ros/README.md)、[平台调试](../HARDWARE_BRINGUP.md#腕部-realsense-相机) |
| RoboSense 雷达 | [rslidar_sdk-main](rslidar_sdk-main/)、[rslidar_msg-master](rslidar_msg-master/) | 雷达数据接收、解码、点云及消息定义 | [中文 README](rslidar_sdk-main/README_CN.md)、[平台调试](../HARDWARE_BRINGUP.md#激光雷达) |
| 双六维力传感器 | [ros2_rs422_ftsensor](ros2_rs422_ftsensor/)，`kw_ft_sensor_ros2` | 串口/TCP 数据读取、力/力矩和连接状态 | [节点实现](ros2_rs422_ftsensor/src/kw_ft_node.cpp)、[平台调试](../HARDWARE_BRINGUP.md#力传感器测试) |

相机驱动提供的数据与通信层已转发的数据需分别核对。例如 `robot_link` 的 MJPEG 图像服务不等于完整 RGB-D、内参和时钟同步接口。

## 标定、集成与保留配置

| 目录 | 用途与范围 |
| --- | --- |
| [aruco_ros-humble-devel](aruco_ros-humble-devel/) | ArUco 标记检测和位姿接口；见 [README](aruco_ros-humble-devel/README.md) |
| [easy_handeye2-master](easy_handeye2-master/) | 手眼标定节点、界面和 TF；见 [README](easy_handeye2-master/README.md) |
| [slam_launch](slam_launch/) | 底盘、雷达、TF、Cartographer 等 SLAM 启动组合 |
| [ros2_canopen_zero_diff](ros2_canopen_zero_diff/) | 另一套 CANopen 配置式接入；与当前自定义腰部驱动分开选择 |
| [esrobo_system](esrobo_system/) | 保留的系统集成资源，不能仅凭目录名作为完整整机启动入口 |
| [dual_ur_robot](dual_ur_robot/)、[ur_start](ur_start/) | 双 UR 机器人配置与启动工具，属于其他机器人配置，不是 ESROBO NERO 双臂主线 |

已有整机组合 launch 位于 [start_esrobo_system_launch.py](ros2_rs485_servo/launch/start_esrobo_system_launch.py)，会组合启动多种设备；腕部 RealSense 的启动片段当前被注释。首次部署使用 [逐部件调试顺序](../HARDWARE_BRINGUP.md#推荐逐部件调试顺序)，按现场连接和反馈确认各项。

## 工作空间与依赖

从平台仓库根目录使用 `colcon` 构建实际需要的 ROS 包；Python SDK 与 `robot_link` 的安装见各自说明。使用系统 ROS 2 环境构建 ROS 包，应用 Conda 环境另行管理。不要将整个 `src/` 假定为同一厂家、同一许可证或全部适用于当前硬件。

Ranger、RealSense 和 UGV SDK 在主仓库中以 gitlink 记录版本。当前 `.gitmodules` 仅列出前两者，UGV SDK 映射缺失；部署到新机器前需补齐来源与固定版本，不能把本地已有源码当作全新递归克隆验收。记录版本与来源见根 [.gitmodules](../.gitmodules)及各驱动说明。

## 驱动与平台通信的关系

应用通过 [`robot_link`](../robot_link/README.md)提交关节目标，网关调用执行适配器，再访问 CAN SDK 或 ROS 手桥。网关并不将这里所有设备统一包装为网络控制 API；目前的关节协议主要服务机械臂和灵巧手。新增底盘、腰部或其他控制能力时，应明确协议字段、单位、反馈与停止行为，并同步协议文档。
