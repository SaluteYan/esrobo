# 日常遥操作步骤

推荐使用统一网页入口：仓库根目录执行 `./laptop_teleop/scripts/run_dashboard.sh`，打开 `http://127.0.0.1:8080`。以下各终端的启动、标定回车和网关命令均已接入页面，具体流程见 [`05_WEB_CONSOLE.md`](05_WEB_CONSOLE.md)。下文保留为命令行操作和故障排查参考；不要同时启动网页与终端中的同一采集/计算程序。

以下流程默认双臂双手，仓库位于 `/home/ddc/DualArmTeleoperation`，机器人为 `192.168.10.100`。首次操作前必须完成环境、网络、共享密钥和机器人端硬件标定。

## 0. 开始前

- 清空机器人运动范围，确认急停可触及。
- 机器人双臂、双手已获得新鲜反馈，CAN 和 ROS 驱动无故障。
- PICO 电量充足，启用 Body 和 Hand 追踪，摘下手套并保持双手可见。
- 首次联调先使用单臂或 `--preview`，确认方向后再使能。

统一设置仓库变量：

```bash
export ESROBO_REPO=/home/ddc/DualArmTeleoperation
cd "$ESROBO_REPO"
```

## 1. 机器人终端 R1：底层驱动与网关

按 [`02_ROBOT_CONNECTION.md`](02_ROBOT_CONNECTION.md) 启动 CAN、LinkerHand 双手驱动、双手桥和硬件网关。网关看到有效反馈后保持 `IDLE`，此时不要输入 `e`。

当前三个机器人进程已运行在 `esrobo_hands`、`esrobo_hand_bridge`、`esrobo_gateway` tmux 会话中。每次操作前仍需执行本机 `run_laptop.sh inspect`；2026-09-22 的检查中左臂控制器处于 `EMERGENCY_STOP`，解除并确认 `controller_fault: null` 前禁止进入第 7 步。

## 2. 本机终端 A：PICO 服务

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/xrobotoolkit_service.sh start
./laptop_teleop/scripts/xrobotoolkit_service.sh status
```

在头显端启动全身追踪应用并连接本机 PC Service。然后启动采集适配器：

```bash
./laptop_teleop/scripts/run_pico.sh
```

默认向本机 `127.0.0.1:15050` 发布，骨架预览为 `http://127.0.0.1:8765`。终端应持续显示真实骨骼帧，而非只显示等待连接。

## 3. 旧版 SenseCom 流程（当前 PICO 模式跳过）

**当前版本第 3–5 节仅保留作旧手套软件维护参考，不属于遥操作启动流程。** 直接执行第 6 节；PICO 统一输入与升级命令见 [文档 06](06_PICO_HAND_INPUT.md)。计算端忽略 SenseGlove 数据。

固定手套为左手 `00885`、右手 `00892`。需要恢复 ROS 配置时执行：

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/configure_senseglove.sh 00885 00892
./laptop_teleop/scripts/run_sensecom.sh
```

Nova 2 v2.x 不要只在系统蓝牙设置中连接。关闭系统蓝牙设置页，在 SenseCom 左上角菜单选择 **Pair Devices**，确认两只手套出现在 **Nearby Devices** 后点击配对；两只设备进入 **Paired Devices** 且均显示 `Connected` 才算连接完成。v1.x Bluetooth Classic 需先完成 rfcomm 连接。

可以在启动 ROS 前用厂商 SDK 独立验证：

```bash
source /opt/ros/humble/setup.bash
source "$ESROBO_REPO/laptop_teleop/external/senseglove_ros_ws/install/setup.bash"
ros2 run senseglove_api sg_tester
```

正常结果应列出左右两只手套；若输出 `No SenseGloves detected`，先处理 SenseCom 配对，不要继续 ROS 启动提示。

## 4. 旧版 SenseGlove ROS（当前 PICO 模式跳过）

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/run_senseglove_ros.sh
```

launch 会提示启动 SenseCom 并按 Enter；终端 B 已连接后按 Enter。另开短命令验证话题：

```bash
source /opt/ros/humble/setup.bash
source "$ESROBO_REPO/laptop_teleop/external/senseglove_ros_ws/install/setup.bash"
ros2 topic list | grep senseglove_states
ros2 topic hz /senseglove/glove<左序列号>/lh/senseglove_states
ros2 topic hz /senseglove/glove<右序列号>/rh/senseglove_states
```

## 5. 旧版手套标定与输入桥（当前 PICO 模式跳过）

首次使用、重新佩戴或标定文件失效时：

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/run_senseglove.sh \
  --left-serial <左手套序列号> --right-serial <右手套序列号> --recalibrate
```

按终端提示依次保持张手/中性腕姿和握拳等姿态。只有左右话题都有非零实时数据才会完成标定。结果写入 `laptop_teleop/config/senseglove_calibration.json`，该文件不会提交到 Git。

日常已有有效标定时去掉 `--recalibrate`：

```bash
./laptop_teleop/scripts/run_senseglove.sh \
  --left-serial <左手套序列号> --right-serial <右手套序列号>
```

## 6. 本机终端 E：检查、预览、正式运行

先检查模型与 PICO SDK：

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/run_laptop.sh check
```

首次建议在机器人网关保持 `IDLE` 时预览计算，不发送目标：

```bash
./laptop_teleop/scripts/run_laptop.sh run --preview
```

确认 PICO 胸前抬臂参考、IK 输出及左右映射正确后退出预览，等待至少 1 秒，再正式启动：

```bash
./laptop_teleop/scripts/run_laptop.sh run
```

正式计算启动后，本机先发送机器人当前实测姿态的保持目标。此时无需先做参考标定；输入 `e` 后才重新采集本次运动参考。

## 7. 机器人终端 R1：人工使能

只有在本机各采集终端持续正常、网关状态为 `IDLE`、机器人周边安全时，才在机器人网关终端输入：

```text
e
```

`e` 后所选灵巧手先限速回到自然张开位并核对十轴反馈，机械臂再受检回零并验证失能。终端提示后，将前臂抬到胸前、肘部自然弯曲，双手自然张开并保持在头显视野内；PICO 连续检测稳定后自动建立参考、发送零位匹配首帧并进入 `ACTIVE`。双臂网关当前不自动执行缺少互碰扫掠检查的回零，必须按现场双臂恢复流程处理。运动过程中保持急停可触及。

## 8. 停止

在本机计算终端输入以下任一字符后回车：

```text
s
x
q
```

也可按 Ctrl+C。观察机器人网关退出 `ACTIVE` 并确认机械臂不再接收运动目标。随后按以下顺序停止采集：

1. Ctrl+C 停止 PICO 全身与双手采集。
2. 如仍在运行旧手套服务，停止手套桥、SenseGlove ROS 和 SenseCom；它们已不参与当前计算。
3. 按需停止 PC Service：

```bash
./laptop_teleop/scripts/xrobotoolkit_service.sh stop
```

4. 在机器人端按硬件文档失能并关闭底层驱动。

## 9. 单臂、纯手臂或仅灵巧手模式

纯左臂只需 PICO Body：

```bash
# 机器人端
python -m esrobo_link.gateway --hardware --side left --bind 192.168.10.100 \
  --config teleoperation/config/teleop_config.yaml --key-file ~/.config/esrobo/robot-link.key

# 本机
./laptop_teleop/scripts/run_laptop.sh run --side left --arm-only
```

右臂把 `left` 改成 `right`。单臂加同侧手时，机器人网关增加 `--with-hand`，本机不加 `--arm-only`，并确保同侧 PICO Hand 有效；无需启动手套桥。

仅控制一只灵巧手时仍需机器人端 LinkerHand ROS 驱动和手部通信桥，但不需要 PICO Body 标定、手臂重定向或 IK。以下以右手为例：

```bash
# 机器人端：保持右臂七轴失能，只允许右手目标
python -m esrobo_link.gateway --hardware --side right --with-hand --hand-only \
  --bind 192.168.10.100 --config teleoperation/config/teleop_config.yaml \
  --key-file ~/.config/esrobo/robot-link.key

# 本机：只消费 right_hand 并发送十轴手目标
./laptop_teleop/scripts/run_laptop.sh run --side right --hand-only
```

左手把两处 `right` 改成 `left`。使能前确认网页中对应机械臂仍为 `0 / 7`；网关发现任一轴已使能会拒绝启动或立即停止。

## 10. 无硬件自检

```bash
cd "$ESROBO_REPO"
/home/ddc/miniforge3/envs/esrobo_laptop/bin/python -m esrobo_laptop.demo
./laptop_teleop/scripts/test.sh
```

演示只使用本机临时端口、模拟网关和临时密钥，不连接 `192.168.10.100`，不会创建设备后端。
