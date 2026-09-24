# 首次环境与设备软件安装

当前遥操作改为 PICO Body + Hand 统一采集，SenseGlove/本机手套 ROS 安装步骤仅供旧设备维护。已有环境请按 [PICO 输入升级操作](06_PICO_HAND_INPUT.md) 更新 SDK，无需重新安装手套软件；机器人端灵巧手 ROS 驱动仍需保留。

## 1. 已在本机完成的安装

本机是 Ubuntu 22.04 amd64，当前已完成：

- Miniforge：`/home/ddc/miniforge3`
- Conda 环境：`esrobo_laptop`（Python 3.10）
- ROS 2 Humble：`/opt/ros/humble`
- XRoboToolkit PC Service：`/opt/apps/roboticsservice`
- XRoboToolkit Python SDK：安装在 `esrobo_laptop`
- SenseGlove ROS 2 Humble：已在 `laptop_teleop/external/senseglove_ros_ws` 构建 10 个包
- SenseCom：已随 SenseGlove 工作空间安装并通过共享库检查
- 外部源码：`laptop_teleop/external/`

外部源码与构建产物被 Git 忽略，不会提交到项目仓库。

## 2. 一键重建

在新的 Ubuntu 22.04 机器上克隆仓库后执行：

```bash
export ESROBO_REPO=/home/ddc/DualArmTeleoperation
cd "$ESROBO_REPO"
./laptop_teleop/scripts/setup_all.sh
```

该命令依次：安装用户目录下的 Miniforge、创建 Conda 环境、安装 PICO PC Service、编译 PICO Python SDK、安装 ROS 依赖并构建 SenseGlove 工作空间。安装系统包时会由 `sudo` 交互询问密码；脚本不会保存密码。

需要单独重跑时：

```bash
./laptop_teleop/scripts/install_env.sh
./laptop_teleop/scripts/install_pico.sh
./laptop_teleop/scripts/install_senseglove.sh
```

## 3. Conda 环境

环境定义位于 `laptop_teleop/environment.yml`，包含 Python 3.10、NumPy、SciPy、PyYAML、Pinocchio、CMake、pybind11、Ninja 和 pytest。激活命令：

```bash
source /home/ddc/miniforge3/etc/profile.d/conda.sh
conda activate esrobo_laptop
python --version
python -c 'import numpy, scipy, yaml, pinocchio; print("IK environment OK")'
```

脚本默认直接调用环境内 Python，不要求先 `conda activate`。若 Miniforge 装在其他位置：

```bash
MINIFORGE_DIR=/实际路径/miniforge3 ./laptop_teleop/scripts/install_env.sh
```

## 4. ROS 2 Humble 缺失时

SenseGlove ROS 必须使用 Ubuntu 22.04 的系统 ROS 2 Humble 和 `/usr/bin/python3`。先检查：

```bash
test -f /opt/ros/humble/setup.bash && echo "ROS 2 Humble OK"
```

若不存在，按 ROS 2 Humble 官方 Ubuntu deb 安装流程配置 ROS 软件源并安装桌面版；完成后执行：

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null && echo "ROS 2 Humble OK"
```

官方步骤：<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>。不要在 Conda 环境中构建 SenseGlove ROS；安装脚本会清理 Conda 路径并指定 `/usr/bin/python3`。

## 5. PICO / XRoboToolkit

安装脚本固定使用官方 Ubuntu 22.04 amd64 的 XRoboToolkit PC Service v1.0.0，并校验 deb 的 SHA-256；随后从官方源码编译 `libPXREARobotSDK.so` 和 Python 绑定。

启动并检查服务：

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/xrobotoolkit_service.sh start
./laptop_teleop/scripts/xrobotoolkit_service.sh status
ss -lnt | grep 60061
```

服务日志：

```bash
tail -f "$ESROBO_REPO/laptop_teleop/log/xrobotoolkit_pc_service.log"
```

PICO 头显与本机处于同一网络后，在头显端启动 XRoboToolkit/全身追踪应用，并把 PC Service 地址设为本机网卡 IP。`127.0.0.1` 仅是本机 Python SDK 到本机 PC Service 的连接，不是头显应填写的地址。

验证 Python SDK：

```bash
PY=/home/ddc/miniforge3/envs/esrobo_laptop/bin/python
LIB="$ESROBO_REPO/laptop_teleop/external/XRoboToolkit-PC-Service-Pybind/lib"
PYTHONNOUSERSITE=1 LD_LIBRARY_PATH="$LIB:/home/ddc/miniforge3/envs/esrobo_laptop/lib" \
  "$PY" -c 'import xrobotoolkit_sdk; print("PICO SDK OK")'
```

## 6. SenseGlove 软件与连接

源码位于 `laptop_teleop/external/senseglove_ros_ws/src/senseglove_ros`，使用 `humble-dev` 分支。SenseCom 可执行文件随仓库提供；ROS 工作空间使用系统 Python 构建。

把当前用户加入串口组，只需执行一次；重新登录后生效：

```bash
sudo usermod -aG dialout "$USER"
```

Nova 2 固件 v2.x 使用 BLE，必须从 SenseCom 左上角菜单进入 **Pair Devices**，在 **Nearby Devices** 中点击手套完成配对。GNOME/系统蓝牙显示“已连接”不代表 SenseCom SDK 已识别设备，也不要用系统蓝牙的旧式“配对”流程；系统蓝牙设置页持续扫描还会使 SenseCom 报 `org.bluez.Error.Busy`。固件 v1.x 使用 Bluetooth Classic，可运行官方脚本绑定 `/dev/rfcomm*`：

```bash
bash "$ESROBO_REPO/laptop_teleop/external/senseglove_ros_ws/src/senseglove_ros/senseglove/senseglove_bringup/scripts/glove_connect.sh"
```

本项目的采集手套固定为左手 `00885`、右手 `00892`。ROS 配置已经记录这两个序列号；需要恢复配置时执行：

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/configure_senseglove.sh 00885 00892
cat laptop_teleop/external/senseglove_ros_ws/src/senseglove_ros/senseglove/senseglove_bringup/config/gloves.yaml
```

SenseGlove 仓库的旧 `SenseCom/Linux/README.md` 留有“Nova 不兼容”的历史说明，而同一仓库当前 `USAGE.md` 已给出 Nova 1/2 的 BLE 与 Bluetooth Classic 流程。现场应以 `USAGE.md` 和实际固件为准；必须在 ROS 话题出现非零实时数据后才做遥操作标定。

## 7. 完整检查

```bash
cd "$ESROBO_REPO"
./laptop_teleop/scripts/check_environment.sh
./laptop_teleop/scripts/run_laptop.sh check
./laptop_teleop/scripts/test.sh
```

`check_environment.sh` 的每项都应为 `[OK]`；`run_laptop.sh check` 应显示机器人目标 `192.168.10.100:16000`、模型自由度 59、`xrobotoolkit_sdk: true`。
