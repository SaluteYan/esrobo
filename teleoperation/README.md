# ESROBO 双臂 + 灵巧手 实机遥操作

实际设备启动、标定、按键和故障排查步骤见
[`TELEOPERATION_RUNBOOK.md`](TELEOPERATION_RUNBOOK.md)。

独立的 **实机运动** 遥操作包，放在 ESROBO 项目根目录下的 `teleoperation/` 文件夹中，
与原有 esrobo ROS2 工作空间代码完全分开（不混源码、不改动 `src/` 里的任何东西）。

核心思路：沿用 [`Dual-arm-teleoperation`](https://github.com/SaluteYan/Dual-arm-teleoperation)
仓库的遥操作算法（PICO 全身骨骼 `arm_vector` 重定向 + Pink IK），但把 **IsaacLab / PhysX 仿真层**
替换为 ESROBO 真机的底层 SDK：

- 双臂 **松灵 NERO 七轴机械臂**：直接用 `pyAgxArm` SDK 驱动（左 `can_piper1`，右 `can_piper2`）。
- 双手 **LinkerHand L20Lite 灵巧手**：通过 ROS2 topic 控制（`/cb_left|right_hand_control_cmd`）。
- 手指/IMU 输入：**SenseGlove Nova 2**；手臂输入：**PICO 全身骨骼追踪**（PICO 负责手臂，SenseGlove 负责手指+IMU）。

## 目录结构

```text
teleoperation/
├── environment.yml                  # conda 环境定义（teleop_esrobo）
├── pyproject.toml                   # Python 包元数据
├── config/teleop_config.yaml        # 实机遥操作参数（CAN、重定向、IK、手部）
├── urdf/esrobo_waist_with_head.urdf # 整机 URDF（Pink IK 用，来自仓库）
├── src/esrobo_teleop/               # 实机遥操作核心包（纯 NumPy，不依赖 IsaacLab）
│   ├── config.py                    # 配置 dataclass + YAML 加载
│   ├── math_utils.py                # 四元数/旋转/手臂向量数学工具（移植）
│   ├── teleop_node.py               # 主循环：UDP→重定向→IK→双臂→手
│   ├── device/body_device.py        # UDP 接收 + arm_vector 重定向（PICO）
│   ├── ik/solver.py                 # Pink IK（整机 URDF，14 臂关节）
│   └── robot/
│       ├── nero_driver.py           # NERO 双臂驱动 + URDF↔物理关节映射
│       └── linker_hand_driver.py    # 灵巧手 10→20 关节扩展 + UDP/ROS2 输出
├── scripts/
│   ├── install_env.sh               # 创建 conda 环境并安装依赖
│   ├── run_teleop.sh                # 启动实机遥操作
│   ├── calibrate_arms.py            # 双臂上电/校准/自测
│   └── sim_body_input.py            # 无硬件仿真输入（离线测试）
└── bridges/                         # 桥接进程（依赖外部 SDK，见下文）
    ├── hand_ros_bridge.py           # 手部 UDP → LinkerHand ROS2 topic
    ├── xrobotoolkit_body_udp_bridge.py      # PICO→UDP（XRoboToolkit）
    ├── pico_full_body_udp_bridge.py         # PICO→UDP（IsaacTeleop）
    ├── senseglove_ros_to_esrobo_hand_bridge.py  # SenseGlove→UDP
    └── run_*.sh                     # 参考启动脚本
```

## 1. 安装 conda 环境

本项目使用 conda 环境 `teleop_esrobo`（Python 3.10，与系统 ROS2 Humble 一致）。

```bash
# 若本机还没有 miniconda（sudo 不可用时可装到用户目录）：
bash <(curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh) -b -p ~/miniconda3
# 若首次安装需先接受 Anaconda 渠道条款：
~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
~/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# 一键安装环境（创建 env、装 numpy/scipy/pinocchio/pink/python-can、装 pyAgxArm）：
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/install_env.sh
```

> `install_env.sh` 会从 `/home/esrobo/Projects/esrobo/src/pyAgxArm-master` 安装 NERO 底层 SDK
> （可用 `PYAGXARM_PATH` 覆盖）。`pink` 需从官方仓库安装（PyPI 上的 `pink` 0.8.1 是另一个格式化工具）。

激活环境：

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate teleop_esrobo
```

## 2. 硬件 / 通信准备

确认 CAN 接口已就绪（`start_can.sh` 已执行过，接口名见 `99-fixed-can.rules`）：

```bash
ip -br link show type can     # 应看到 can_piper1 / can_piper2 / can_hand1 / can_hand2 ...
```

只读硬件状态检查（不会使能或发送运动命令）：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
source ~/miniconda3/etc/profile.d/conda.sh && conda activate teleop_esrobo
export PYTHONPATH=src
python scripts/calibrate_arms.py probe --config config/teleop_config.yaml
python scripts/calibrate_arms.py validate --config config/teleop_config.yaml
```

若返回 `firmware=None joints=None`，说明 CAN 没有收到机械臂反馈。此时程序会拒绝使能，
不得用零角代替当前角度。灵巧手驱动启动后可只读检查：

```bash
ros2 topic echo /cb_left_hand_state --once
ros2 topic echo /cb_right_hand_state --once
```

## 3. 启动实机遥操作

架构为多个独立进程，通过 UDP/ROS2 通信：

```
PICO/XRoboToolkit ──► xrobotoolkit_body_udp_bridge.py ──► UDP:15050 ─┐
                                                                     ▼
SenseGlove Nova2 ──► senseglove_ros_to_esrobo_hand_bridge.py ──► UDP:15050 ──► teleop_node
                                                                            │
                                                    ┌────────────────────────┤
                                                    ▼                        ▼
                                        NERO 双臂 (pyAgxArm)      LinkerHand (ROS2 topic)
```

**进程 1 — 核心遥操作（conda 环境）**：接收 UDP，做重定向 + IK，驱动双臂和手。

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_teleop.sh            # 默认按 config/teleop_config.yaml
./scripts/run_teleop.sh --no-robot # 离线模式：只算 IK，不命令手臂
```

运行时按键：`e` 使能/开始跟随，`s` 停止/保持，`x` 发送电子急停，`q` 退出。
默认未使能；按 `e` 时必须先取得有效关节反馈，并以该反馈作为本次会话初始角。

**进程 2 — PICO 全身骨骼桥接（依赖 XRoboToolkit / IsaacTeleop）**：
见 `bridges/xrobotoolkit_body_udp_bridge.py`（参考启动 `bridges/run_xrobotoolkit_body_to_esrobo_teleop_bridge.sh`）。
这些桥接需要 PICO 头显 + XRoboToolkit PC Service，或 IsaacTeleop，属外部硬件 SDK，不在本环境内安装。

**进程 3 — SenseGlove 桥接（系统 ROS2）**：
```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/install/setup.bash   # 若已构建
ros2 run senseglove_ros senseglove_ros_node ...          # SenseGlove 官方驱动
python3 bridges/senseglove_ros_to_esrobo_hand_bridge.py --left-serial <左> --right-serial <右>
```

**进程 4 — 灵巧手 ROS 桥接（系统 ROS2）**：把核心进程（conda 内）发到 UDP:15051 的手指数据
转发给 LinkerHand ROS2 节点：
```bash
source /opt/ros/humble/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py   # 启动灵巧手驱动
python3 bridges/hand_ros_bridge.py --port 15051                 # UDP→ROS2 topic
```

### 3.1 SenseGlove 手指 + IMU 遥操作（仅灵巧手）

先只做手指运动 + IMU 数据遥操作，不驱动双臂：

```bash
# 终端1：LinkerHand 硬件驱动（系统 ROS2，若未随整机启动）
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/install/setup.bash   # esrobo 工作区（含 linker_hand_ros2_sdk）
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py

# 终端2：手部 UDP → LinkerHand topic（系统 ROS2）
source /opt/ros/humble/setup.bash
python3 bridges/hand_ros_bridge.py --port 15051

# 终端3：SenseGlove 硬件驱动（系统 ROS2，见第 7 节）
source /opt/ros/humble/setup.bash
source .../external/senseglove_ros/install/setup.bash
ros2 launch senseglove_bringup senseglove.launch.py

# 终端4：SenseGlove → UDP:15050（先做第 4.1 节标定；--recalibrate 只在首次/换手套时）
python3 bridges/senseglove_ros_to_esrobo_hand_bridge.py --left-serial <左> --right-serial <右>

# 终端5：手部遥操作核心（conda 环境）
./scripts/run_hand_teleop.sh            # 启动后按 'h' 使能灵巧手，'q' 退出
```

**安全限制（`hand.*` 配置）**：
- 手部默认**不使能**，必须按 `h`（或 `--enable`）才开始运动。
- 按 `h` 前必须同时收到左右手新鲜的 `/cb_*_hand_state`；反馈中断 0.5 秒自动停止下发。
- 输出严格钳位到 `[out_min, out_max]`（默认 0–255）。
- 每命令最大变化 `max_step`、每秒最大速度 `max_velocity`，防止手指突变。
- 首次使能从 `/cb_*_hand_state` 读取到的当前姿态起步，不使用假定的 `home` 状态。
- `scripts/hand_safe_test.py` 用于真机逐关节核对方向/顺序后再使能。

> 本阶段只控制灵巧手手指；IMU 提供的手腕朝向（`hand_orientation_deltas`）已随 UDP 一并
> 接收，但要驱动手腕/手臂跟随需开启双臂遥操作。

## 4. 标定流程

1. **手臂初始角 / 侧向映射**：先运行 `calibrate_arms.py validate`，只读确认物理角、
   URDF 角、SDK 软限位及映射往返一致。不要调用 SDK `reset()` 回零；该 API 是急停后复位并
   掉使能，不是轨迹规划回零。
2. **重定向原点标定**：启动核心遥操作后，保持自然站立、双手自然下垂约 2.5 秒，
   出现 `Calibration locked (natural-hold origin)` 即锁定为原点（对应
   `auto_start_reference_*` 参数）。之后手臂运动相对该原点映射。
3. **左右手交叉**：若机器人左右映射反了，把 `retarget.swap_left_right_targets` 改为 `true`。

### 4.1 SenseGlove 手指/IMU 标定（遥操作前必须做）

SenseGlove 的**两姿态标定**由桥接进程完成（生成 `config/senseglove_esrobo_calibration.json`）：

```bash
# 首次或手套佩戴变化后，用 --recalibrate 强制重新标定
python3 bridges/senseglove_ros_to_esrobo_hand_bridge.py \
    --left-serial <左> --right-serial <右> --recalibrate
```
按提示依次做两个姿态（每个约 10s，ENTER 开始）：
1. **1/2 张手 + IMU 中性**：双手自然张开、手指伸直，保持你希望映射到手部初始位姿的手腕朝向。
2. **2/2 握拳**：双手握拳。

标定文件存在后，日常运行无需重复标定（除非换手套/佩戴松紧变化），直接运行即可自动加载。

### 4.2 LinkerHand 关节顺序/方向安全标定（首次上真手）

由于物理手关节顺序、方向与 0-255 映射需真机核对，先做逐关节安全测试：

```bash
# 需 LinkerHand 驱动已启动；按关节逐个动一点，记录哪个索引对应哪根手指/哪个方向
python3 scripts/hand_safe_test.py --side left --joint 0 --delta 8 --confirm-motion
python3 scripts/hand_safe_test.py --side right --joint 2 --delta 8 --confirm-motion
```
L10 顺序已按硬件 SDK 固定映射。当前启用物理轴 `[0,1,2,3,4,5,9]`（拇指三轴及四指屈伸），
侧摆和拇指旋转保持读取到的当前位置；逐轴验证后再修改 `hand.enabled_physical_joints`。

## 5. 离线测试（无需硬件）

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
source ~/miniconda3/etc/profile.d/conda.sh && conda activate teleop_esrobo

# 终端1：仿真输入
python scripts/sim_body_input.py --hz 60 --swing

# 终端2：核心遥操作（不命令手臂）
./scripts/run_teleop.sh --no-robot
```

终端2 出现 `Calibration locked` 后即开始输出 IK 关节目标（控制台打印），验证整条
UDP→重定向→IK 链路。

## 6. 关键参数（config/teleop_config.yaml）

| 参数 | 说明 |
| --- | --- |
| `robot.command_mode` | 默认 `j`（SDK 平滑位置速度模式）；`js` 为无平滑高风险透传，默认禁止 |
| `robot.max_joint_step` | 单步关节最大变化（rad）安全钳位 |
| `robot.left/right_joint_offsets` | `physical = direction * URDF + offset`；关节2为 `+π/2` |
| `robot.max_joint_velocity` | J1-J4 首次联调为 `5/5/6/6 deg/s`，J5-J7 为 `10 deg/s`；同时受逐关节加速度和单步限制 |
| `robot.max_joint_deviation_from_start` | 初测时各关节不得偏离按 `e` 捕获的初始角超过 0.35 rad |
| `retarget.arm_vector_position_mode` | `segment_direction_relative`（仅用最小旋转对齐自然下垂的上臂方向，不用近共线的肩肘腕点重定义 H1/H2 水平轴） |
| `retarget.swap_left_right_targets` | 左右手映射交叉开关 |
| `ik.enable_elbow_tasks` | 是否用肘部任务辅助 IK |
| `ik.dt` | IK 步长 |
| `hand.mode` | `udp`（经 hand_ros_bridge 转发）/ `ros2`（直接发布） |

> **注意**：所有位姿 / 肩肘 / 臂长参数都来自仓库的 IsaacLab 标定值（`config.py` 默认值）。
> 首次上真机请务必先用 `calibrate_arms.py` 核对关节方向与限位，再逐步使能跟随，手始终放在急停上。

## 7. 外部 SDK 构建状态（XRoboToolkit / SenseGlove）

以下外部 SDK 已从 GitHub 克隆到 `teleoperation/external/` 并配置，可一键复现：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/build_external_sdks.sh
```

| 依赖 | 来源 | 状态 | 说明 |
| --- | --- | --- | --- |
| `XRoboToolkit-PC-Service` | [XR-Robotics](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) | ✅ 已构建 | 编译出 `libPXREARobotSDK.so` |
| `XRoboToolkit-PC-Service-Pybind` | [XR-Robotics](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) | ✅ 已构建 | `xrobotoolkit_sdk` 已装进 conda 环境 |
| `XRoboToolkit-Teleop-Sample-Python` | [XR-Robotics](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) | ✅ 已克隆 | 参考数据链路 |
| `senseglove_ros`（`humble-dev` @ `a14a468`） | [Adjuvo/senseglove_ros](https://github.com/Adjuvo/senseglove_ros) | ✅ 已构建 | 全工作区原生编译，含 `senseglove_msgs`/`senseglove_hardware`/`senseglove_api` |

**XRoboToolkit（PICO 双臂）— 已配置可用**
```bash
# conda 环境已含 xrobotoolkit_sdk；启动 PICO 全身骨骼 → UDP:15050
./bridges/run_xrobotoolkit_bridge.sh --rate-hz 90
```
> 运行时需 PICO 头显运行 XRoboToolkit 应用 + 电脑运行 XRoboToolkit-PC-Service。

**SenseGlove Nova2（手指/IMU）— 本机可直接使用 ✅**
本机为 Ubuntu 22.04（glibc 2.35）。**关键点**：必须用 **`humble-dev` 分支 @ commit `a14a468`**
（"hardware interface: humble port fixes"）。该版本自带 x86-64/release 预编译
`libsgcore.so`（md5 `5e73d2a8…`）**只要求 glibc 2.34（≤ 2.35）**，所以在本机原生
colcon build + 系统 `/usr/bin/python3`（ROS2 Humble）即可正常运行，无需 Docker/重编/换系统。

> ⚠️ 不要用上游 `humble` 分支或更新的 tag——它们的 `libsgcore.so` 用更新的工具链编译，
> 要求 glibc 2.38，在本机无法运行（厂商只给预编译库、无源码）。

启动（先跑硬件驱动，再跑桥接）：
```bash
# 终端1：SenseGlove 硬件驱动（系统 ROS2 Humble）
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
ros2 launch senseglove_bringup senseglove.launch.py

# 终端2：手指/IMU → UDP:15050
./bridges/run_senseglove_bridge.sh --left-serial <左> --right-serial <右>
```
> 若 topic 命名不是默认的 `/senseglove/glove<serial>/{lh|rh}/senseglove_states`，
> 用 `--left-topic/--right-topic` 覆盖。
> 手套通过蓝牙连接（Nova2 需先在 SenseCom 里配对/连接），`senseglove_bringup` 会加载
> `senseglove_bringup/config/gloves.yaml`（按 serial 配置每只手套）。

## 许可证

算法代码移植自 `Dual-arm-teleoperation` 仓库，其源码沿用各自 SPDX 头（多数 BSD-3-Clause）。
`pyAgxArm`、`LinkerHand`、SenseGlove 等 SDK 遵循各自第三方许可证。
