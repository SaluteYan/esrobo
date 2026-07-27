# ESROBO 移动仿人形双臂机器人平台

本仓库是 ESROBO 移动双臂机器人 ROS 2 工作空间，运行环境以 Ubuntu 22.04 + ROS 2 Humble 为主。系统包含移动底盘、双 NERO 七轴机械臂、三轴腰部关节、头部云台舵机、双五指灵巧手、头部/腕部相机、激光雷达和双六维力传感器。

本文档根据当前代码和现场调试记录整理，重点服务于真机逐部件调试。第一次上电调试时不要直接启动整机总 launch，建议按本文档顺序逐项验证。

## 硬件组成

| 部件 | 型号/说明 | ROS 包 |
| --- | --- | --- |
| 移动底盘 | 松灵 Ranger Mini 3 | `ranger_bringup`, `ranger_base`, `ugv_sdk` |
| 左/右机械臂 | 松灵 NERO 七轴机械臂 | `agx_arm_ctrl`, `pyAgxArm-master` |
| 腰部关节 | 零差 3 关节，CANopen 风格驱动 | `erob_canopen` |
| 灵巧手 | 灵心巧手 L10，双手 CAN | `linker_hand_ros2_sdk` |
| 头部相机 | Orbbec Gemini 435Le | `orbbec_camera` |
| 腕部相机 | RealSense D435，左右各一台 | `realsense2_camera` |
| 头部舵机 | 飞特 SM40，RS485 | `servo_driver` |
| 六维力传感器 | 坤维 KW65/KW56B，RS422 | `kw_ft_sensor_ros2` |
| 激光雷达 | RoboSense Fairy | `rslidar_sdk` |

## 目录说明

主要目录如下：

```text
src/agx_arm_ros2             NERO/Piper 机械臂 ROS2 控制包
src/pyAgxArm-master          机械臂底层 Python SDK
src/erob_can_joints          腰部关节驱动，包名 erob_canopen
src/ranger_ros2              Ranger 底盘 ROS2 驱动
src/ugv_sdk                  底盘底层 SDK
src/linkerhand-ros2          灵巧手 ROS2 SDK
src/ros2_rs485_servo         头部 RS485 舵机驱动
src/ros2_rs422_ftsensor      六维力传感器驱动
src/OrbbecSDK_ROS2           Orbbec 头部相机驱动
src/realsense-ros            RealSense 腕部相机驱动
src/rslidar_sdk-main         RoboSense 激光雷达驱动
src/esrobo_system            系统集成相关代码，目前不建议作为首轮调试入口
debug_cmd.txt                原始现场调试命令备忘
start_can.sh                 当前真机 CAN 命名和波特率初始化脚本
99-fixed-can.rules           当前真机 USB/CAN/串口 udev 规则参考
```

## 基础环境

每个新终端先执行：

```bash
cd /home/esrobo/Projects/esrobo
source /opt/ros/humble/setup.bash
source install/setup.bash
```

重新构建单个包示例：

```bash
colcon build --packages-select kw_ft_sensor_ros2
source install/setup.bash
```

系统依赖建议确认：

```bash
sudo apt install ethtool can-utils
```

## CAN 和串口设备

当前现场 USB 根总线号可能在 `1-*` 和 `3-*` 之间变化。`start_can.sh` 会按稳定的 USB 端口路径后缀把 CAN 口重命名并设置波特率。

当前映射：

| 设备名 | USB 地址 | 波特率 | 用途 |
| --- | --- | --- | --- |
| `can_waist` | `*-7:1.0` | 1000000 | 腰部关节 |
| `can_car` | `*-8:1.0` | 500000 | Ranger 底盘 |
| `can_piper1` | `*-5.3:1.0` | 1000000 | 左 NERO 机械臂 |
| `can_piper2` | `*-5.4:1.0` | 1000000 | 右 NERO 机械臂 |
| `can_hand1` | `*-5.1.2:1.0` | 1000000 | 灵巧手 1 |
| `can_hand2` | `*-5.1.1:1.0` | 1000000 | 灵巧手 2 |

初始化 CAN：

```bash
cd /home/esrobo/Projects/esrobo
./start_can.sh
ip -br link show type can
```

正常应看到以上 6 个 CAN 口均为 `UP`。如果接口名丢失，例如右臂又变成 `can2`，重新运行 `./start_can.sh`。

检查 USB/CAN 物理映射：

```bash
for iface in $(ip -br link show type can | awk '{print $1}'); do
  printf '%-14s ' "$iface"
  ethtool -i "$iface" 2>/dev/null | awk '/driver:/ {driver=$2} /bus-info:/ {bus=$2} END {print "driver=" driver, "bus=" bus}'
done
```

串口当前映射：

| 别名 | 当前物理口 | 用途 |
| --- | --- | --- |
| `/dev/rs422_ft1` | `/dev/ttyUSB0` 对应 `*-5.1.3:1.0` | 左力传感器 |
| `/dev/rs422_ft2` | `/dev/ttyUSB1` 对应 `*-5.1.4:1.0` | 右力传感器 |
| `/dev/rs485_servo` | `/dev/ttyACM0` 对应 `*-5.2:1.0` | 头部舵机 |

安装 udev 规则：

```bash
sudo cp 99-fixed-can.rules /etc/udev/rules.d/99-fixed-can.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 不建议直接使用的总启动

存在整机总 launch：

```text
src/ros2_rs485_servo/launch/start_esrobo_system_launch.py
```

它会同时启动腰部、双臂、底盘、雷达、头部舵机、双手和力传感器。首次调试时不要直接运行，因为多个驱动存在启动即使能或启动即运动行为。

建议先逐部件调试，确认每个链路稳定后再考虑整机集成。

## 推荐逐部件调试顺序

推荐顺序：

```text
1. CAN/串口枚举
2. 力传感器
3. 头部相机、腕部相机、激光雷达
4. 腰部关节
5. 机械臂，先左后右，单臂测试
6. 头部舵机
7. 灵巧手
8. 底盘，最后测试
```

调试执行机构前确认：

```text
整机急停已弹起
机械臂/腰部/头部/灵巧手周围无遮挡
底盘测试时架空或放在空旷区域
旁边有人观察，发现异常立即急停或断电
```

## 力传感器测试

启动：

```bash
ros2 launch kw_ft_sensor_ros2 ft_sensor.launch.py
```

检查连接状态和数据：

```bash
ros2 topic echo /left/ft_sensor/connected
ros2 topic echo /right/ft_sensor/connected
ros2 topic echo /left/ft_sensor/data
ros2 topic echo /right/ft_sensor/data
```

如果 `/dev/rs422_ft1`、`/dev/rs422_ft2` 不存在，当前 launch 会自动回退到 `/dev/ttyUSB0`、`/dev/ttyUSB1`。

## 头部 Orbbec Gemini 435Le 相机

启动：

```bash
ros2 launch orbbec_camera gemini435_le.launch.py
```

验证：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

当前 launch 已将 image transport 发布插件收敛为 `raw`，避免缺少 `compressedDepth` 插件导致终端误报。大分辨率图像和点云会使 `ros2 topic hz` 测得偏低，调试阶段可关闭点云：

```bash
ros2 launch orbbec_camera gemini435_le.launch.py enable_point_cloud:=false
```

也可降低分辨率：

```bash
ros2 launch orbbec_camera gemini435_le.launch.py \
  color_width:=640 color_height:=400 \
  depth_width:=640 depth_height:=400 \
  color_fps:=10 depth_fps:=10 \
  enable_point_cloud:=false
```

## 腕部 RealSense 相机

左相机：

```bash
ros2 launch realsense2_camera rs_left_launch.py
```

右相机：

```bash
ros2 launch realsense2_camera rs_right_launch.py
```

查看话题：

```bash
ros2 topic list | grep left
ros2 topic list | grep right
```

## 激光雷达

启动：

```bash
ros2 launch rslidar_sdk start.py
```

该 launch 会启动雷达节点和 RViz。确认点云话题和 RViz 显示是否正常。

## 腰部关节

启动腰部驱动：

```bash
ros2 run erob_canopen erob_driver_node
```

查看状态：

```bash
ros2 topic echo /erobo/joint_states
ros2 topic echo /erobo/joint_errors
```

`/erobo/joint_states` 是 `Float32MultiArray`，每 5 个数一组：

```text
[关节ID, 位置角度deg, 速度, 电流, 温度]
```

垂直参考位：

```text
31号 ≈ 180°
32号 ≈ 180°
33号 ≈ 150°
```

使能：

```bash
ros2 topic pub /erobo/enable std_msgs/msg/Bool "{data: true}" -1
```

小幅位置测试，先测试 33 号从约 150° 到 151°：

```bash
ros2 topic pub /erobo/pos_target std_msgs/msg/Float32MultiArray "{data: [33.0, 151.0, 10.0, 20.0]}" -1
```

回到参考位：

```bash
ros2 topic pub /erobo/pos_target std_msgs/msg/Float32MultiArray "{data: [33.0, 150.0, 10.0, 20.0]}" -1
```

停止和失能：

```bash
ros2 topic pub /erobo/stop std_msgs/msg/Empty "{}" -1
ros2 topic pub /erobo/enable std_msgs/msg/Bool "{data: false}" -1
```

`/erobo/pos_target` 数据含义：

```text
[关节ID, 目标位置角度, 加速度, 速度]
```

注意：手册中把后两个字段描述成“速度、力矩”，但当前源码实际使用第三个数作为加速度，第四个数作为速度。

## 机械臂测试

机械臂必须先确认 CAN：

```bash
./start_can.sh
ip -br link show type can
candump can_piper1
candump can_piper2
```

正常情况下，机械臂 CAN 会持续刷出 `251/252/2A1/2A5` 等帧。若接口处于 `ERROR-PASSIVE` 或 `candump` 无输出，先排查电源、急停、线缆和控制器状态，不要启动驱动。

### 左臂启动

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=left \
  can_port:=can_piper1 \
  arm_type:=nero \
  effector_type:=none \
  auto_enable:=false \
  control_enabled:=false \
  fast_mode:=false \
  speed_percent:=5
```

查看反馈：

```bash
ros2 topic echo /left/feedback/joint_states
ros2 topic echo /left/feedback/arm_status
```

使能并打开控制：

```bash
ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/control_enable std_srvs/srv/SetBool "{data: true}"
```

小幅测试建议从当前反馈复制完整 7 关节位置，只改变一个末端关节约 `0.02 rad`。不要只发单个关节，当前驱动对缺失关节处理不够安全，`move_j` 必须一次给全 7 个关节。

基于 `debug_cmd.txt` 参考姿态的小幅 `joint7` 测试：

```bash
ros2 topic pub /left/control/move_j sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'], position: [-0.5427973973702365, 1.5582474094730574, 0.3045425011804906, 0.06129596333004085, -1.5061144247159868, 0.015271630954950384, -0.5038954815711379], velocity: [0.1], effort: []}" -1
```

回位：

```bash
ros2 topic pub /left/control/move_j sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'], position: [-0.5427973973702365, 1.5582474094730574, 0.3045425011804906, 0.06129596333004085, -1.5061144247159868, 0.015271630954950384, -0.5238954815711379], velocity: [0.1], effort: []}" -1
```

停止/关闭：

```bash
ros2 service call /left/emergency_stop std_srvs/srv/Empty "{}"
ros2 service call /left/control_enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

### 右臂启动

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=right \
  can_port:=can_piper2 \
  arm_type:=nero \
  effector_type:=none \
  auto_enable:=false \
  control_enabled:=false \
  fast_mode:=false \
  speed_percent:=5
```

使能：

```bash
ros2 service call /right/enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /right/control_enable std_srvs/srv/SetBool "{data: true}"
```

参考小幅 `joint7` 测试：

```bash
ros2 topic pub /right/control/move_j sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'], position: [-0.330268654354887, 1.477089599255321, 1.325682286644813, 0.9649925434276648, 0.24741787476271615, -0.06649704450098395, 0.1358568798587192], velocity: [0.1], effort: []}" -1
```

回位：

```bash
ros2 topic pub /right/control/move_j sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'], position: [-0.330268654354887, 1.477089599255321, 1.325682286644813, 0.9649925434276648, 0.24741787476271615, -0.06649704450098395, 0.1158568798587192], velocity: [0.1], effort: []}" -1
```

停止/关闭：

```bash
ros2 service call /right/emergency_stop std_srvs/srv/Empty "{}"
ros2 service call /right/control_enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /right/enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

### 当前右臂已知问题

现场调试中右臂出现过：

```text
Failed to get firmware version
agx_arm_ctrl_single process has died
```

同时观测到：

```text
can_piper1 左臂：candump 有持续 CAN 帧，状态 ERROR-ACTIVE
can_piper2 右臂：曾出现 candump 无输出，状态 ERROR-PASSIVE
```

这说明右臂驱动在初始化读取固件版本阶段失败。若再次出现，先确认：

```bash
ip -br link show type can
ip -details link show dev can_piper2
candump can_piper2
```

如果报 `Cannot find device "can_piper2"`，说明接口名丢失，运行：

```bash
./start_can.sh
```

如果 `can_piper2` 存在但 `ERROR-PASSIVE` 且 `candump` 无输出，则优先检查右臂电源、急停、CAN 线、终端电阻、控制器状态或联系供应商。

## 头部舵机

启动：

```bash
ros2 launch servo_driver start_servo.py
```

注意：当前舵机驱动启动时会自动发送：

```text
1号舵机 -> 1500
2号舵机 -> 3450
```

启动前必须确认头部周围无遮挡。

小幅测试：

```bash
ros2 topic pub /servo/ctrl std_msgs/msg/UInt16MultiArray "{data: [1, 1450, 5, 1]}" -1
ros2 topic pub /servo/ctrl std_msgs/msg/UInt16MultiArray "{data: [1, 1500, 5, 1]}" -1
ros2 topic pub /servo/ctrl std_msgs/msg/UInt16MultiArray "{data: [2, 3400, 5, 1]}" -1
ros2 topic pub /servo/ctrl std_msgs/msg/UInt16MultiArray "{data: [2, 3450, 5, 1]}" -1
```

舵机范围按当前代码限制：

```text
1号俯仰：1000 - 2700
2号左右：2000 - 5000
速度：<= 15
```

## 灵巧手

启动双手：

```bash
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
```

状态：

```bash
ros2 topic echo /cb_left_hand_state
ros2 topic echo /cb_right_hand_state
```

控制话题：

```text
/cb_left_hand_control_cmd
/cb_right_hand_control_cmd
```

消息类型为 `sensor_msgs/msg/JointState`，L10 需要一次给完整 10 个位置值。当前驱动启动时会设置速度/力矩并移动到预设姿态，第一次测试前确认手指无遮挡。

手册参考左手命令：

```bash
ros2 topic pub /cb_left_hand_control_cmd sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['thumb_cmc_pitch','thumb_cmc_yaw','index_mcp_pitch','middle_mcp_pitch','ring_mcp_pitch','pinky_mcp_pitch','index_mcp_roll','ring_mcp_roll','pinky_mcp_roll','thumb_cmc_roll'], position: [200.0, 200.0, 254.0, 254.0, 254.0, 254.0, 179.0, 180.0, 179.0, 30], velocity: [50, 50, 50, 50, 50, 50, 50.0, 50.0, 50.0, 50.0], effort: []}" -1
```

第一次建议从当前状态复制 10 个位置值，只微调一个手指 3-5 个单位。

## 底盘

启动：

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py
```

键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

初次测试建议先短脉冲，再立即发 0。底盘代码当前没有明确的命令 watchdog，测试时必须有人观察。

前进小脉冲：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

## 常用诊断命令

ROS 图：

```bash
ros2 node list
ros2 topic list
ros2 topic info -v <topic>
ros2 topic hz <topic>
```

CAN：

```bash
ip -br link show type can
ip -details link show dev can_piper1
ip -details link show dev can_piper2
candump can_piper1
candump can_piper2
```

串口：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/rs422* /dev/rs485* 2>/dev/null
udevadm info -q path -n /dev/ttyUSB0
udevadm info -q path -n /dev/ttyUSB1
udevadm info -q path -n /dev/ttyACM0
```

## 已知风险和注意事项

1. `start_esrobo_system_launch.py` 会同时启动多个执行机构，不适合首轮调试。
2. 头部舵机驱动启动时会自动发位置命令。
3. 灵巧手驱动启动时会设置较高力矩/速度并移动到预设姿态。
4. 机械臂 `move_j` 必须给完整 7 关节位置，避免缺失关节被错误处理。
5. 底盘 `/cmd_vel` 测试后必须主动发送 0 速度。
6. 腰部 `/erobo/enable`、`/erobo/brake` 作用于扫描到的全部腰部关节，不是单轴独立使能。
7. `debug_cmd.txt` 中部分命令幅度较大，第一次真机调试不要直接照抄。

## 通过标准

每个部件建议满足以下条件再进入下一项：

```text
传感器：话题存在，数据持续输出，无明显异常跳变
CAN：接口 UP，状态 ERROR-ACTIVE，有预期 CAN 帧
腰部：状态能读，小幅目标能到达并回位
机械臂：能读 firmware 和 joint_states，小幅单关节运动能回位
头部舵机：小幅运动方向正确，无卡滞
灵巧手：状态能读，单指小幅运动正常
底盘：低速短脉冲方向正确，停止命令有效
```
