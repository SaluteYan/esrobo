# ESROBO 遥操作运行手册

> **当前主机稳定性说明（2026-09-21）**：左手联合启动期间，Fast DDS 的
> `libfastrtps.so.2.6.11` 已发生过进程级 general protection fault，随后同一次启动出现整机失联。
> SenseGlove 联合入口因此默认统一使用 Cyclone DDS。首次运行前安装：
>
> ```bash
> sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
> ```
>
> 启动输出必须显示 `ROS 2 middleware: rmw_cyclonedds_cpp`。该切换绕开已观测到的 Fast DDS
> 崩溃路径，但不能替代内存/主板稳定性检查；在硬件检查完成前只做有人托扶的小范围调试。

> 最近整理：2026-09-21
> 面向对象：第一次接触本项目、需要在机器人主机上启动现有遥操作模式的操作者

本文只说明现场准备、启动、按键、停止和故障排查。当前程序的数据流、重定向、IK、关节轨迹、碰撞检查和回零状态机见
[`TELEOPERATION_RUNTIME_LOGIC.md`](TELEOPERATION_RUNTIME_LOGIC.md)；修改原因和历史验证见
[`TELEOPERATION_CHANGELOG.md`](TELEOPERATION_CHANGELOG.md)。旧运行中的 Python 进程不会加载新代码，代码更新后必须按安全流程退出并重新启动。

## 目录

- [分布式部署：笔记本计算＋机器人执行](#分布式部署笔记本计算机器人执行)

- [1. 先确认要运行的模式](#1-先确认要运行的模式)
- [2. 每次启动前的安全检查](#2-每次启动前的安全检查)
- [3. SSH、tmux 与工作目录](#3-sshtmux-与工作目录)
- [4. 按键速查与正确停止方式](#4-按键速查与正确停止方式)
- [5. PICO 模式准备](#5-pico-模式准备)
  - [5.1 首次安装](#51-首次安装)
  - [5.2 PICO 网络与全身追踪](#52-pico-网络与全身追踪)
  - [5.3 服务和人体数据只读检查](#53-服务和人体数据只读检查)
  - [5.4 本地骨架网页](#54-本地骨架网页)
- [6. 启动 PICO 单臂和臂手联动](#6-启动-pico-单臂和臂手联动)
  - [6.1 纯左臂](#61-纯左臂)
  - [6.2 纯右臂](#62-纯右臂)
  - [6.3 左臂＋左手](#63-左臂左手)
  - [6.4 右臂＋右手](#64-右臂右手)
  - [6.5 共同启动顺序](#65-两类-pico-模式的共同启动顺序)
  - [6.6 PICO 参考重采](#66-pico-参考重采)
- [7. SenseGlove 和灵巧手模式](#7-senseglove-和灵巧手模式)
  - [7.1 启动前检查手套](#71-启动前检查手套)
  - [7.2 只控制左灵巧手](#72-只控制左灵巧手)
  - [7.3 只控制右灵巧手](#73-只控制右灵巧手)
  - [7.4 双灵巧手](#74-双灵巧手)
  - [7.5 左手腕 J5-J7＋左灵巧手](#75-左手腕-j5-j7左灵巧手)
  - [7.6 右手腕 J5-J7＋右灵巧手](#76-右手腕-j5-j7右灵巧手)
  - [7.7 手腕模式共同标定与操作](#77-手腕模式共同标定与操作)
  - [7.8 无硬件链路检查](#78-无硬件链路检查)
- [8. 安全回零与故障恢复](#8-安全回零与故障恢复)
- [9. 日志与只读检查](#9-日志与只读检查)
- [10. 常见故障：按现象排查](#10-常见故障按现象排查)
- [11. 头部相机与网页视角调整](#11-头部相机与网页视角调整)
  - [11.1 启动三个服务](#111-启动三个服务)
  - [11.2 访问与调整](#112-访问与调整)
  - [11.3 检查与录制](#113-检查与录制)
- [12. 当前安全限制参考](#12-当前安全限制参考)
- [13. 软件验证命令](#13-软件验证命令)

## 分布式部署：笔记本计算＋机器人执行

新通信入口位于 [`../robot_link/README.md`](../robot_link/README.md)：笔记本 Ubuntu 负责
PICO/SenseGlove、重定向、IK 和网页；机器人主机负责关节执行/反馈、安全回零和相机采集。

- 首次使用先运行模拟网关及只读监视器，验证密钥、侧别和 UDP 通信。
- 数据字段、坐标单位、有效期、状态机见 [`PROTOCOL.md`](../robot_link/PROTOCOL.md)。
- 收包后如何限速、下发、读取反馈和处理故障，见 [`ROBOT_EXECUTION.md`](../robot_link/ROBOT_EXECUTION.md)。
- 提供单臂及同侧手、`--side both` 双臂＋双手网关与笔记本 SDK；完整笔记本采集/计算/网页入口仍需接入。
- 本节之后的一键脚本仍是**原本机一体模式**，不能与真实网关同时启动；网关按键需要 Enter。
- 网络中断会闭锁并请求电子停止，不自动回零；单侧独立 `z` 启动避碰回零，双侧 `z` 暂不支持，见通信部署文档。

## 1. 先确认要运行的模式

当前优先使用下表中的一键脚本。不要直接拼接底层 ROS、CAN 和 Python 进程。

| 模式 | 人体输入 | 被控硬件 | 启动入口 |
| --- | --- | --- | --- |
| PICO 左臂 | PICO 左肩、肘、腕位置 | 左臂 J1-J4；J5-J7 保持实测位置 | `run_pico_left_arm_teleop.sh` |
| PICO 右臂 | PICO 右肩、肘、腕位置 | 右臂 J1-J4；J5-J7 保持实测位置 | `run_pico_right_arm_teleop.sh` |
| PICO 左臂＋左手 | PICO 左臂＋左手套 IMU/手指 | 左臂 J1-J7＋左灵巧手 | `run_pico_left_arm_hand_teleop.sh` |
| PICO 右臂＋右手 | PICO 右臂＋右手套 IMU/手指 | 右臂 J1-J7＋右灵巧手 | `run_pico_right_arm_hand_teleop.sh` |
| 左灵巧手 | 左手套手指 | 左灵巧手 | `run_senseglove_hand_teleop.sh --left-only` |
| 右灵巧手 | 右手套手指 | 右灵巧手 | `run_senseglove_hand_teleop.sh --right-only` |
| 双灵巧手 | 左右手套手指 | 左右灵巧手 | `run_senseglove_hand_teleop.sh` |
| 左手腕＋左手 | 左手套 IMU/手指 | 左臂 J5-J7＋左灵巧手 | `run_senseglove_hand_teleop.sh --left-wrist-imu` |
| 右手腕＋右手 | 右手套 IMU/手指 | 右臂 J5-J7＋右灵巧手 | `run_senseglove_hand_teleop.sh --right-wrist-imu` |
| 无硬件链路检查 | 仿真输入 | 不连接机器人 | 加 `--no-hardware` |
| 头部相机与视角调整 | 网页 | 头部两轴与 RGB-D 相机 | 见第 11 节 |

`scripts/run_teleop.sh` 是只启动核心节点的底层入口，不会自动启动 PICO、SenseGlove、LinkerHand 驱动或完成当前单臂启动门禁，因此不作为新操作者的一键入口。当前经过现场流程整理的是表中的分侧模式；双臂同时跟随仍需先完成独立的双侧输入编排和碰撞验收，不能把两个单臂脚本并行启动来代替。

左手套序列号为 `00885`，右手套序列号为 `00892`。硬件接口约定如下：

| 设备 | 左侧 | 右侧 |
| --- | --- | --- |
| NERO 机械臂 CAN | `can_piper1` | `can_piper2` |
| LinkerHand CAN | `can_hand2` | `can_hand1` |
| SenseGlove | `Nova 2-00885-L` | `Nova 2-00892-R` |

## 2. 每次启动前的安全检查

1. 清空机械臂、灵巧手、躯干和线缆周围空间，人员离开夹点。
2. 确认实体急停可立即触及。涉及机械臂时，安排一人从运行脚本前开始持续托稳当前侧机械臂。
3. 确认只运行本次需要的一侧。不要同时启动两个会争用同一机械臂、灵巧手、CAN 或端口的入口。
4. 机械臂失能后会受重力下落。`d`、启动检查失败和程序退出都可能导致失能，因此不能先松手再观察提示。
5. 启动时的 `Enter` 只用于确认托稳或开始采样；实时控制阶段按 `e/h/s/x/d/z/q/r/i` 都是单键操作，不需要再按 `Enter`。
6. 第一次实机联调从远离躯干的小幅单自由度动作开始。方向、关节顺序、网页目标和实体反馈一致后再扩大范围。

**当前正常遥操作跟随不做实时躯干几何检查**（`teleop_torso_collision_enabled: false`）；使能前一次姿态核验和慢速回零的规划、执行检查保留。关节限位、限速、反馈检查、力矩监测和控制器碰撞保护仍开启。正常跟随时不会由软件几何模型提前拦截向躯干运动。

当前躯干碰撞模型假定腰部和头部固定在 URDF 零位，不覆盖另一条机械臂、外部环境、线缆和所有外壳细节。软件检查通过不等于现场空间已经安全。

## 3. SSH、tmux 与工作目录

登录机器人主机后进入：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
```

每个遥操作入口使用独立的 `tmux` 会话。例如：

```bash
tmux new -s pico_right_arm
```

在会话中运行脚本。SSH 需要临时断开时，按 `Ctrl+b`，松开后按 `d`，只分离 tmux，不会给机器人发送 `d`。重新进入：

```bash
tmux ls
tmux attach -t pico_right_arm
```

不要在机械臂仍使能、回程进行中或失能未确认时直接执行 `tmux kill-session`。

## 4. 按键速查与正确停止方式

按键只在运行遥操作核心的前台终端有效。

| 按键 | 适用模式 | 当前行为 |
| --- | --- | --- |
| `e` | PICO 单臂、手腕 IMU、臂手联动 | 检查输入、反馈、控制器、碰撞和力矩状态，通过后使能并开始跟随。 |
| `h` | 含灵巧手的模式 | 切换灵巧手跟随；反馈不新鲜时拒绝。PICO 纯机械臂模式不初始化灵巧手，按此键只会提示不可用。 |
| `s` | 机械臂模式 | 正常停止。PICO 七轴单臂使用避碰规划回 URDF 零位并失能；手腕 IMU 模式只让 J5-J7 回零并失能。 |
| `x` | 机械臂模式 | 停止跟随并发送当前侧电子急停；必要时同时使用实体急停。 |
| `d` | PICO 七轴单臂模式 | 直接失能，不回零。必须先托稳，并等待 `DISABLED` 确认。 |
| `z` | PICO 七轴单臂模式 | 故障排除后，单独请求检查、规划、回零和失能；不会恢复遥操作。 |
| `q` | 所有模式 | 请求安全结束。PICO 七轴单臂在正常或故障闭锁状态下都先执行受检回零，零位与七轴失能均确认后才退出；手腕模式先让 J5-J7 回零并失能；纯手模式让参与控制的手平滑张开。 |
| `r` | PICO 单臂、臂手联动 | 在电机和跟随均关闭且无故障闭锁时，重新采集 PICO 自然下垂参考；不发送硬件命令。 |
| `i` | 左/右手腕 IMU | 开关 IMU 三轴诊断，只改变日志，不使能机械臂。 |
| `Ctrl+C` | 所有模式 | 请求清理。正常结束优先用 `q`，并等待回零、失能和退出结果。 |

正常结束 PICO 七轴单臂模式时：

1. 持续托稳当前侧机械臂。
2. 按 `q`。
3. 等待终端明确报告零位已验证且失能成功。
4. 回程失败且机械臂仍使能时，程序不会退出。保持托稳并查看 `RETURN <stage>` 的原因；排障后可再次按 `q`，只想回零不退出时按 `z`，无法安全运动时按 `d` 直接失能，异常运动按 `x`。

`s/q` 在无故障状态下共用安全回零规划器。发生力矩、碰撞、反馈或轨迹故障后会闭锁跟随：`s` 只报告停止已经生效，`q` 明确请求与 `z` 相同的受检恢复回零，并在成功失能后退出。`q/z` 都不会绕过仍存在的硬件故障、当前接触、过期反馈或不安全几何，也不会恢复遥操作。

左臂开机后若有界反馈唤醒仍得不到完整七轴角度和使能位，会显示 `POWER-CYCLE REQUIRED`。这种状态没有安全回零所需的实测起点，本进程内禁止再次 `e/z`。持续托稳并按 `q`；程序只发送电子急停后退出，不发送位置目标。随后给左臂控制器重新上电，恢复反馈并重新运行入口。

## 5. PICO 模式准备

### 5.1 首次安装

仅在 PC Service、Conda 环境或 XRoboToolkit Python SDK 缺失时运行：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/setup_xrobotoolkit.sh
```

该脚本安装和检查软件，不配置机械臂 CAN，也不发送运动命令。安装结果可只读检查：

```bash
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' roboticsservice
test -x /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python && echo "Conda 环境存在"
```

### 5.2 PICO 网络与全身追踪

推荐连接关系：

```text
操作电脑 <--网线/SSH--> 机器人主机 <--Wi-Fi 热点--> PICO
                       192.168.10.100       10.42.0.1
```

机器人热点连接名为 `MyHotspot`，SSID 为 `esrobo`。PICO 中 XRoboToolkit 的 PC Service 地址填写 `10.42.0.1`。只读检查：

```bash
nmcli -t -f NAME,TYPE,DEVICE,STATE connection show --active
ip -br address
ip neigh show dev wlp89s0
```

PICO 侧完成以下准备：

1. 连接 Wi-Fi `esrobo`。
2. 启动 XRoboToolkit 应用并连接 `10.42.0.1`。
3. 开启全身追踪，完成 Motion Tracker 校准。
4. 确认头显内人体骨架随动作连续更新。

### 5.3 服务和人体数据只读检查

一键遥操作脚本会自动启动服务并再次检查人体数据。需要在连接机器人前单独诊断时运行：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh start
./scripts/xrobotoolkit_pc_service.sh status
```

状态必须同时包含 `RUNNING` 和 `127.0.0.1:60061 LISTENING`。检查左侧人体数据：

```bash
env LD_LIBRARY_PATH="$PWD/external/XRoboToolkit-PC-Service-Pybind/lib:/home/esrobo/miniconda3/envs/teleop_esrobo/lib:${LD_LIBRARY_PATH:-}" \
  /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python \
  ./scripts/check_xrobotoolkit_body.py --side left --wait-seconds 20
```

右侧把 `--side left` 改为 `--side right`。只有出现 `PICO BODY DATA READY` 才继续。程序优先使用人体时间戳，其次使用关节时间戳；两者均为 0 时，只有肩、肘、腕位姿实际变化才算新骨架。通用 XR 时间戳即使持续前进，也不能证明人体追踪仍在更新。

### 5.4 本地骨架网页

启动 PICO 单臂脚本后，网页服务只绑定机器人主机 `127.0.0.1:8765`。在操作电脑的本地终端建立转发：

```bash
ssh -N -L 8765:127.0.0.1:8765 esrobo@192.168.10.100
```

浏览器打开 `http://127.0.0.1:8765`。不要在机器人 SSH shell 或机器人 tmux 中执行这条 `ssh -L`，否则可能反向占用端口。

| 网页视图 | 用途 |
| --- | --- |
| `PICO 原始` | 判断追踪输入、左右侧和抖动是否正确。 |
| `固定映射` | 检查固定坐标变换。 |
| `重定向目标` | 检查送入 IK 的机器人尺度目标。 |
| `IK 结果` | 检查 J1-J4 求解后的 URDF 骨架。 |
| `限速命令` | 检查实际交给 SDK 前的关节轨迹。 |
| `实际反馈` | 检查实体关节反馈的 FK 骨架。 |

网页超过 `500 ms` 没有新数据时会显示停止，此时不要按 `e`。若 `8765` 已被占用，可在启动脚本前设置 `PICO_SKELETON_VIEWER_PORT=8876`，同时把 SSH 转发和浏览器端口改为 `8876`。

标定前只有 `PICO 原始` 和 `固定映射` 有数据。若选择了尚未启动或已过期的控制诊断层，网页会自动回到 `PICO 原始` 并提示原因。若原始层也不动，同时人体/关节时间戳为 0，则是 PICO/XRoboToolkit 提供了缓存骨架；重新启动头显中的全身追踪、检查 Tracker 连接并重新校准，直到原始层连续随人体动作更新。

## 6. 启动 PICO 单臂和臂手联动

### 6.1 纯左臂

```bash
tmux new -s pico_left_arm
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_left_arm_teleop.sh
```

### 6.2 纯右臂

```bash
tmux new -s pico_right_arm
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_right_arm_teleop.sh
```

纯单臂模式不启动、不标定、不控制 SenseGlove 和 LinkerHand。PICO 肩、肘、腕位置只生成当前侧 J1-J4 目标，J5-J7 每周期保持最新实测位置。碰撞模型包含上臂、小臂、腕部、力传感器和刚性手掌，不包含活动手指；运行前必须确认手指不会形成夹点。

### 6.3 左臂＋左手

```bash
tmux new -s pico_left_arm_hand
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_left_arm_hand_teleop.sh --left-serial 00885
```

联合模式默认沿用左灵巧手单独遥操作已经使用的自然张开零位
`[255,255,255,255,255,255,128,128,128,128]`。程序会先限速回到该姿态并核验反馈，
通过后才允许继续使能左臂。

需要重新核对或替换该零位时，确认 `can_hand2` 已按第 4 节配置，然后在第一个终端启动
`move_on_start=false` 的左手驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/install/setup.bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args \
  -r __node:=linker_hand_sdk_left \
  -p hand_type:=left -p hand_joint:=L10 -p is_touch:=false \
  -p can:=can_hand2 -p move_on_start:=false -p modbus:=None
```

在第二个终端执行只读采集。该工具不会创建命令发布器，也不会驱动机械臂或灵巧手：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/install/setup.bash
cd /home/esrobo/Projects/esrobo/teleoperation
python3 scripts/capture_hand_open_feedback.py --side left
```

重新采集时让左手保持舒适自然张开。只有输出 `STABLE CANDIDATE` 才可把候选值人工核对后写入
`config/teleop_config.yaml` 的 `left_open_feedback`；程序不会自动修改配置。这个数值只解决自然张开
零位核对，不能替代活动手指的反馈到 URDF 几何标定。当前该几何标定尚未完成时，联合入口仍可能
因完整手指包络与躯干相交而拒绝机械臂使能。

### 6.4 右臂＋右手

```bash
tmux new -s pico_right_arm_hand
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_right_arm_hand_teleop.sh --right-serial 00892
```

联动模式中，PICO 控制 J1-J4，当前侧手套 IMU 控制 J5-J7，手指数据控制同侧灵巧手。该模式每次启动都会先按第 7.7 节的动作重新完成手套六步标定，再进入下面的 PICO 检查和标定。活动手指会加入躯干碰撞包络；缺少可靠的反馈到 URDF 几何标定时，程序按完整手指运动范围检查，可能拒绝启动。不要通过缩小碰撞余量或把未知手指角度假定为零来绕过拒绝。

### 6.5 两类 PICO 模式的共同启动顺序

1. 从运行脚本前开始托稳当前侧机械臂。
2. 脚本先检查 PICO 数据，再启动只读骨架网页，然后配置当前侧机械臂 CAN。
3. 按终端提示确认机器人周围安全并按一次 `Enter`，进入 PICO 自然下垂参考采集。
4. 人站直，被测手臂舒适下垂并允许自然微屈。默认是 5 秒准备、1 秒稳定、约 3 秒采集；数据缺失或晃动过大时不会接受参考。
5. 出现“参考零点已锁定”后，机械臂仍未运动。可以把人手臂移到希望机器人开始缓慢追踪的最新姿态，稳定保持并按一次 `e`。不要求使能时的人体姿态与标定姿态完全一致。
6. 按 `e` 后先进行最新 PICO 目标、七轴反馈、控制器、碰撞、力矩和启动姿态检查。机器人若偏离 URDF 零位，会先保持实测位置，再通过避碰路径缓慢完成启动对齐。
7. 只有终端明确显示当前侧 `ARMED` 或联动跟随已开启后，才开始小幅动作。

按 `e` 之后程序还会给出 5 秒准备时间，让操作者放下按键的手并稳定当前人体目标。它只验证目标完整、新鲜和稳定，不会用这次姿态覆盖正式标定参考。

### 6.6 PICO 参考重采

需要重新定义人体自然下垂参考时，先确保机械臂已失能、手臂与手均未跟随且没有故障闭锁，再按 `r`。重采结束后仍需按 `e`。诊断记录在：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
ls -lt log/pico_reference_*.jsonl
tail -n 8 log/pico_reference_<时间戳>.jsonl
```

## 7. SenseGlove 和灵巧手模式

### 7.1 启动前检查手套

一键脚本会自动启动或复用 SSH 无图形 SenseCom，并验证目标手套有实时数据。需要单独检查时：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
ros2 run senseglove_api sg_tester
```

必须看到目标序列号、`connected: true` 且 `rx` 持续大于 `0 pkt/s`。系统蓝牙显示已连接、SenseCom 进程存在或 `Detected` 但 `rx: 0` 都不足以开始标定。

首次 SSH 环境若缺少虚拟显示组件，只安装一次：

```bash
sudo apt-get update
sudo apt-get install -y xvfb x11-utils dbus-x11
```

### 7.2 只控制左灵巧手

首次测试、换手套佩戴方式或手套重新上电后：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-only --recalibrate
```

同一次手套上电期间复用已有标定：

```bash
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-only
```

### 7.3 只控制右灵巧手

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-only --recalibrate
```

同一次手套上电期间复用已有标定：

```bash
./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-only
```

纯灵巧手模式不初始化机械臂。按提示完成三步标定：自然张手、拇指与食指轻触、完整握拳。进入控制界面后按 `h` 开始或停止手指跟随，按 `q` 平滑张手并退出。

### 7.4 双灵巧手

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh \
  --left-serial 00885 --right-serial 00892 --recalibrate
```

双手模式要求左右手套和左右灵巧手反馈都有效。任一侧没有新鲜反馈时不会进入完整双手跟随。

### 7.5 左手腕 J5-J7＋左灵巧手

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-wrist-imu
```

### 7.6 右手腕 J5-J7＋右灵巧手

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-wrist-imu
```

### 7.7 手腕模式共同标定与操作

手腕模式每次启动都强制重新标定。先托稳当前侧机械臂，再按终端提示完成六步：

| 步骤 | 人体动作 |
| --- | --- |
| 1. 中立位 | 手臂自然下垂，手指朝地面，掌心朝身体，手腕平直。 |
| 2. 掌心转向 | 从中立位开始，将掌心转向身体正前方约 25-35°。 |
| 3. 前后侧摆 | 从中立位开始，将指尖向身体正前方抬约 25-35°。 |
| 4. 手腕弯曲 | 从中立位开始，将指尖向身体内侧弯约 25-35°。 |
| 5. 食指对指 | 张手后让拇指与食指指腹轻触。 |
| 6. 握拳 | 完整握拳，拇指屈曲并收向掌心。 |

每步都保持静止后按 `Enter`。标定完成后推荐顺序：

1. 按 `h`，只检查灵巧手各指方向。
2. 保持机械臂失能，按 `i` 开启诊断，分别做三个小幅单轴腕部动作。左侧应主要对应 `X→J5、Y→J6、Z→+J7`；右侧应主要对应 `X→J5、Y→J6、Z→+J7`（以当前配置为准）。
3. 若一个动作同时引起多个大幅目标，按 `q` 退出并重新标定，不要按 `e`。
4. 再次按 `i` 关闭诊断，持续托稳机械臂，按 `e`。程序先让 J5-J7 限速回到机械零位并验证，然后才开始跟随当前手套目标。
5. 正常停止按 `s`；它只让 J5-J7 回零并失能。灵巧手跟随不会由 `s` 自动关闭，需要时先按 `h`。结束程序按 `q`。

默认 `static-orthogonal` 标定用于正式操作。以下线性解耦模式只用于诊断对照：

```bash
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 \
  --left-wrist-imu --imu-calibration-mode static-linear
```

### 7.8 无硬件链路检查

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_senseglove_hand_teleop.sh \
  --left-serial 00885 --left-wrist-imu --no-hardware
```

该模式用仿真输入检查程序链路，不初始化 LinkerHand、SenseGlove 或 NERO，不能用于判断实机方向、间隙和反馈质量。

## 8. 安全回零与故障恢复

<a id="safe-arm-return"></a>

PICO 七轴单臂的最终目标是七轴 URDF 零位。由于 J2 有 `+π/2` SDK 偏置，同一姿态的 SDK 物理角接近 `[0, π/2, 0, 0, 0, 0, 0]`，不能把“物理七轴全为 0”当成自然下垂零位。

当前流程不再使用固定的“J4-J7 先回、J1-J3 后回”。程序从新鲜实测姿态开始：

1. 受控减速并确认停稳。
2. 检查当前姿态和零位；先尝试直接安全路径，必要时使用有界 RRT-Connect 绕行。
3. 对每条路径边检查整个关节区间和制动范围，不只检查途经点。
4. 逐个途经点低速执行，每点停稳后才继续。
5. 连续确认七轴进入 `1.5°` 终点容差，然后失能；不会用最后一帧跳到精确零位。

回程默认使用正常关节和末端速度上限的 50%，搜索预算为 5 秒、5,000 节点、随机种子 0。搜索失败表示本次预算内没有证明安全路径，不代表可以关闭碰撞保护强行运动。

故障后的处理：

1. 保持托稳，先处理实体急停、接触、障碍、线缆、控制器和反馈问题。
2. 如果机械臂已经接触躯干，先人工解除接触。程序拒绝从已碰撞或无法证明安全的姿态自动脱困。
3. 问题排除后，按 `z` 只请求受检回零，或按 `q` 请求相同回零并在成功失能后退出。程序会重新检查控制器、使能位、力矩、静止反馈和几何，再决定是否规划。
4. `z` 成功只会回零并失能，不会重新进入遥操作；需要继续时重新按 `e`。`q` 只有在回零与失能都确认后才结束程序。

回程进行中，`q` 会登记“回零成功后退出”并保持当前回程继续；`s` 会取消，`x` 会电子急停，`d` 会直接失能；`e/z/h/r` 不会插入新动作。取消回程不会自动失能，必须继续托稳并查看终端状态。

联动模式回程时暂停手部跟随，但不会自动张手或握手。纯机械臂模式不要求手套和手部标定；联动模式必须有同侧手部新鲜反馈及可靠几何包络。

## 9. 日志与只读检查

### 9.1 常用日志

```bash
cd /home/esrobo/Projects/esrobo/teleoperation

tail -f log/pico_left_arm_bridge.log
tail -f log/pico_right_arm_bridge.log
tail -f log/senseglove_driver.log
tail -f log/senseglove_bridge.log
tail -f log/linker_hand_driver.log
tail -f log/hand_ros_bridge.log
```

PICO 参考诊断为 `log/pico_reference_<时间戳>.jsonl`，完整臂重定向/IK/命令诊断为 `log/pico_elbow_<时间戳>.jsonl`。分析后一类日志：

```bash
cd /home/esrobo/Projects/esrobo
/home/esrobo/miniconda3/envs/teleop_esrobo/bin/python \
  teleoperation/scripts/analyze_command_trajectory.py \
  --log teleoperation/log/pico_elbow_<时间戳>.jsonl
```

### 9.2 ROS 和 CAN 只读检查

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash

ros2 topic list | grep senseglove
ros2 topic echo --once /senseglove/glove00885/lh/senseglove_states
ros2 topic echo --once /cb_left_hand_state
ip -details -statistics link show type can
```

右侧手套话题为 `/senseglove/glove00892/rh/senseglove_states`，右灵巧手反馈为 `/cb_right_hand_state`。

## 10. 常见故障：按现象排查

### 10.1 按单键没有立即响应

实时按键采用 cbreak 单键读取，不需要 `Enter`。若只有按 `Enter` 才响应，先确认焦点位于运行遥操作核心的 tmux 窗口，而不是日志窗口；再确认当前进程是代码更新后重新启动的新进程。不要连续敲多次 `e`。涉及已使能机械臂时，先根据终端状态安全停止或失能，再重启程序。

### 10.2 PICO 数据不可用或网页停止

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh status
ss -lntp | grep -E ':(60061|63901|8765)'
ip -br address
tail -n 80 log/xrobotoolkit_pc_service.log
```

- 没有 `60061`：本机 SDK 服务未正常启动，退出遥操作后执行 `./scripts/xrobotoolkit_pc_service.sh restart`。
- 有 `60061`、没有 `63901`：PICO 接入端未监听，检查服务日志。
- 两个端口都有但人体检查超时：检查 PICO 地址、热点、XRoboToolkit 应用、全身追踪和 Tracker 校准。
- `body tracking unavailable`：保持机械臂失能，不要按 `e`。
- `body tracking frozen`：SDK 仍返回旧人体缓存，但人体/关节时间戳未更新且坐标不变。保持机械臂
  失能，在 PICO 端重新启动全身追踪并重新校准 Tracker；不要用通用 XR 时间戳判断已经恢复。
- `Address already in use`：用 `ss` 确认占用 PID；只在确认是旧 PICO 数据桥后结束该 PID，或改用其他网页端口。

### 10.3 SenseCom 找不到手套或 `rx: 0`

确认手套开机且有电，等待 BLE 连接后重新运行 `sg_tester`。若 `sensecom.log` 出现：

```text
Failed to construct a SgBleAddr from the string '': IncorrectByteCount
```

说明旧进程中的 BLE 地址缓存无效。一键启动器会在机器人 CAN 初始化前尝试清理重复或无效的 SenseCom 并重启一次；仍失败时停止启动。退出所有遥操作后才可手动停止 SenseCom：

```bash
pkill -TERM -x SenseCom.x86_64
```

随后重新运行一键入口。手套重新上电、SenseCom 重启或佩戴方式改变后应重新标定。

### 10.4 `HAND ENABLE REFUSED` 或 LinkerHand 没有反馈

表示没有新鲜有效的灵巧手状态。检查对应 `/cb_left_hand_state` 或 `/cb_right_hand_state` 以及驱动日志。`embedded_version=None`、CAN 统计持续 `TX>0` 且 `RX=0` 时，检查灵巧手供电、CAN H/L、插头、终端电阻和 USB-CAN 映射，不能绕过反馈门禁。

### 10.5 IMU 串轴、跳变或标定被拒绝

先保持机械臂失能，按 `i` 做三个小幅单轴检查。若原始消息的 `imu_orientation.w` 与 `z` 持续相同，桥接器会重建缺失分量并选择连续支路；日志中的修复状态不应在转动时反复切换。三轴动作太小、方向不独立或拟合条件过差时程序会拒绝，不要放宽阈值，退出后重新完成动作清晰的六步标定。

### 10.6 按 `e` 后拒绝使能

根据终端报告的阶段处理，不要把所有拒绝都解释为硬件故障：

- `PICO ... missing/stale/unstable`：恢复完整、连续、稳定的人体骨架。
- `controller safety ... arm_status: 1` / `EMERGENCY_STOP`：控制器仍处于急停状态，程序在使能前拒绝，
  不是跟随过程中触发轨迹限制。`d` 显示 `DISABLED` 只证明七轴失能，不证明急停已清除；
  重启程序、重新标定或反复按 `e` 都不会清除控制器急停。保持托稳并先排除实体急停、接触及
  驱动故障。若终端明确报告七轴失能、反馈/驱动/碰撞检查均通过且提示 `Press z once`，可按一次
  `z` 请求受检复位和碰撞规划回零；程序会在复位前重新检查，条件变化就拒绝。没有该提示时按
  厂家流程处理，不要强行复位。`q` 在同一故障闭锁下执行相同恢复，且只在回零和失能均确认后退出。
  `e` 始终不会清除急停。新代码将拒绝写入 `pico_startup` 的
  `startup_refused/controller_preflight`。
- `startup collision guard rejected measured pose`：当前实测姿态或运动包络没有通过躯干碰撞检查；保持失能并调整现场姿态，不能缩小保护余量。
- `torque fault remains active`：力矩故障仍闭锁；先排除接触和负载问题，再用 `z` 检查恢复。
- `TARGET LIMITED`：人手屈肘目标超过当前 J4 上限与腕姿态共同确定的可达范围；程序限制目标并继续受控跟随。减小屈肘后自动恢复，不需要重新使能。左右臂 J4 上限均为 120°，不代表骨段夹角也恰好是 120°。
- `TARGET PAUSED at Jn upper/lower`：实体关节已经到达与有界 IK 相同的软件边界。程序保持使能并受控减速到最后安全位置，继续监测反馈、力矩和 PICO；将人体手臂向可达范围移回，目标连续 3 帧回到 25 mm 几何误差内后自动恢复，不需要重新按 `e`。不要反复向边界外推。
- `IK geometry rejected`：当前 IK 结果的肘点或腕点与工作范围限幅后的有效目标相差超过 30 mm，并且不满足上述“实体已到同一关节边界”的可恢复条件，程序不会下发该目标并锁止跟随。若在显示 `ARMED` 后立刻出现，先确认运行的是最新重启进程；不要放宽几何阈值，优先检查最新 `pico_startup_*.jsonl` 的 `ik_geometry_rejected` 精确故障事件（位置误差、活动边界、J3 是否固定、求解次数及候选关节），再对照 `pico_elbow_*.jsonl` 的重定向、IK 和实测层。周期日志约 10 Hz，可能漏掉真正越界的控制帧。当前联合位置求解详见[运行逻辑第 7 节](TELEOPERATION_RUNTIME_LOGIC.md#7-右臂分区-ik)。
- `outstanding command/feedback motion intersects torso safety envelope`：实测位置到上一命令的关节区间未通过躯干检查，不等于已发生实体碰撞。当前默认关闭跟随期间该检查；若旧进程仍出现，按安全流程退出重启并核对启动提示 `Live-following torso geometry checks: OFF`。回零检查仍保留。
- `diagnostic record dropped`：某条诊断数据无法序列化，后续日志继续写入；轨迹故障另存于 `pico_startup_*.jsonl` 的 `trajectory_fault`。
- `last command outside position/feedback-lead envelope`：实测反馈、上一命令和待发目标之间的轨迹包络不一致；程序已停止该路径，不要重复强试。
- `persistent MOVE_J waypoint on joints [...]`：这是旧单途经点模式的历史日志；当前新进程不再使用该门禁。若新进程仍出现此文本，说明运行的还是更新前代码，应先按当时状态安全停止并重新启动。
- `start kept moving during planning`：失能手臂在第一次规划后发生超过 0.2° 的起点变化，程序已从新的静止、安全实测姿态自动重规划一次，但第二次仍在移动。继续托稳并保持静止后重新按 `e` 或 `z`；程序不会沿已经失效的旧起点路径运动。
- 当前左侧 V112 实时跟随参考根目录 README 已完成真机运动的完整七轴 MOVE_J 控制方式；只参考控制接口，控制器速度保持项目原有 30%。右臂同样保持 MOVE_J、30%。
- 历史 `tracking control preparation failed`、CPV ACK 或模式未确认来自已停用的 CPV 试验路径。新进程启动日志应显示 `tracking_control_mode=j`，不应再进入 CPV 准备阶段。
- `returned no complete seven-joint feedback` / `POWER-CYCLE REQUIRED`：左臂有界唤醒后仍没有完整七轴角度或使能位。本进程内不再重复使能或回零；保持托稳，按 `q` 以电子急停结束且不发送位置，然后重新上电。新进程必须先恢复完整反馈并重新通过启动门禁。
- `No buffer space available [Error Code 105]` / `CAN TRANSPORT UNAVAILABLE`：SocketCAN 接口可能仍显示 `UP / ERROR-ACTIVE`，但机械臂控制器没有 ACK，发送队列已填满。不要反复按 `e`、增大发送队列或重试位置命令。使用实体急停，检查左臂控制器电源、CAN 插头和终端链路，退出程序并重新上电；新进程必须先恢复反馈。日志中的 `can2 -> can_piper1` 表示脚本按固定 USB 端口找到设备后改名，本身不代表映射错误。
- `disable_verified=False`：不能证明已经失能，继续托稳，不要关闭终端或杀 tmux；按现场状态选择 `d` 或 `x`。

左臂与右臂当前都使用连续 MOVE_J：每次新鲜反馈到达后发送通过限速的最新完整七轴目标，不再逐点停等。左臂仍会在每条位置前发送 V112 所需的 J 模式帧，控制器速度保持 30%。如果反馈持续不前进，公共轨迹领先保护仍会在 2 秒后闭锁，不通过扩大误差或延长超时恢复。

跟随期间遇到 `persistent feedback lead` 会额外请求一次电子急停并保持闭锁。
`Electronic stop send returned` **不代表已经停稳或失能**；如果显示
`ELECTRONIC STOP SEND FAILED`，使用实体急停。若电子急停确由本进程的该停滞保护发出，排障后按
`q` 会先核验姿态和七轴状态、确认失能，再清除该软件急停并执行受检回零；任一步不满足都会拒绝。
操作者按 `x`、实体急停或硬件控制器故障不会由 `q/z` 自动复位。无法证明可以安全运动时按 `d`，
并等待七轴失能确认后退出。不要反复按 `e`。此变更需要重新启动进程才能加载。

若出现“刚动一下就停”且报 `persistent feedback lead`，需要同时检查出站和反馈；下次诊断运行
可从 `teleoperation` 目录执行：

```bash
ESROBO_ARM_DIAGNOSTICS=1 ./scripts/run_pico_left_arm_teleop.sh
```

新进程才会加载发送监测。`pico_arm_probe_*.jsonl` 的 `native_can_send` 保存实际发送载荷和
结果；`socket_send_failed` 表示本机发送异常，`socket_send_returned` 不代表控制器已经执行。
`pico_can` 没有发送帧可能只是 SDK 关闭了本机回显。分析时关联同一编号的 startup、arm_probe
和 can 文件。诊断会增加写盘量，仅定位问题时开启；不能靠反复按 `e` 或放宽领先上限恢复。
周期 `pico_elbow` 只记录启动事件摘要，完整启动快照在同编号 `pico_startup` 中查看。

当前左臂使用连续 MOVE_J。诊断时检查 `controller_snapshot.transport.last_control_frames` 中每组
`0x151(J)`、`0x155/0x156/0x157/0x170`、控制器速度 30% 和七轴实测角度。日志中的
`move_j_waypoint_gate` 与 `command_backpressure` 应为 `null`；急停后的 `last_frame=0x150` 不会覆盖这些关键帧证据。

### 10.7 故障后无法退出

新进程中，故障闭锁后按 `q` 会打印 `EXIT REQUEST`，启动与 `z` 相同的受检恢复回零；只有出现零位验证成功和七轴失能确认后才退出。若当前接触、控制器/力矩故障、反馈过期、混合使能或避碰规划失败，程序保持运行并报告 `RETURN <stage>`，不会强行运动或直接断开。

保持托稳：异常运动按 `x`；已人工解除接触且希望安全回零退出时按 `q`；只回零不退出时按 `z`；需要立即释放且不能安全回零时按 `d`，等待 `OPERATOR DISABLE: DISABLED`。回程已经运行时再次按 `q` 不会取消，而是登记成功后退出；按 `s` 才会取消回程。若仍看到“q 被拒绝、先按 d”的旧提示，说明 tmux 中仍是更新前进程，应按当时已显示的安全状态处理，确认失能并重新启动后再测试。

若 CAN 完全没有反馈，`d` 始终显示 `DISABLE NOT CONFIRMED`，程序无法证明当前姿态、规划回零或确认失能。新进程会显示 `POWER-CYCLE REQUIRED`；持续托稳，按 `q` 或 Ctrl+C，程序先请求电子急停再退出，不发送回零位置。若 tmux 中仍是没有该提示的旧进程，不能直接 `kill`：优先使用实体急停；也可按 `x` 请求当前侧电子急停，看到 `ELECTRONIC E-STOP sent` 后再按 Ctrl+C。电子急停发送提示不等于七轴失能确认，退出后仍不得松开或重新使能，必须重新上电、恢复反馈并核验控制器/驱动状态。

## 11. 头部相机与网页视角调整

该功能是辅助观察和采集，不是机械臂遥操作入口。它会控制头部两轴，仍需单独检查夹点、线缆和实时反馈。

### 11.1 启动三个服务

每个 ROS 终端先加载环境：

```bash
cd /home/esrobo/Projects/esrobo
source /opt/ros/humble/setup.bash
source install/setup.bash
```

终端 A 启动头部驱动：

```bash
ros2 launch servo_driver start_servo.py allow_motion:=true
```

启动参数本身不证明当前力矩状态，必须看实时反馈。终端 B 启动 640×400、20 Hz 彩色和深度：

```bash
cd /home/esrobo/Projects/esrobo
bash teleoperation/scripts/run_head_camera_20hz.sh
```

终端 C 启动网页 ROS 桥。已有 PICO 骨架网页时：

```bash
cd /home/esrobo/Projects/esrobo
bash teleoperation/scripts/run_head_camera_web.sh
```

只调头部、没有 PICO 数据桥时改用：

```bash
bash teleoperation/scripts/run_head_camera_web.sh --standalone-viewer
```

两种网页模式二选一，不要同时运行两个 `8765` 网页服务或两个 `8766` ROS 桥。

### 11.2 访问与调整

在操作电脑本地终端执行：

```bash
ssh -N -L 8765:127.0.0.1:8765 esrobo@192.168.10.100
```

浏览器打开 `http://127.0.0.1:8765/head.html`。

1. 等待两轴反馈和相机图像均为新鲜状态。
2. 托稳头部、排除夹点，点击“开启调整”并确认。
3. 使用 ±10 小步或填写目标原始计数。一次只执行一个目标，等待到位后再继续。
4. 完成后点击“锁定调整”，确认 `pending=false`、反馈稳定，并记录实际位置。

当前轴范围为 ID1 `1000-2700`、ID2 `2000-5000`，网页每小步最多 10 计数，单步 5 秒超时，到位容差 5 计数。原始计数不是角度，软件总限位也不是无碰撞空间证明。页面关闭或心跳中断只会请求锁定，不等同于硬件急停，也不保证已接受的小步立即停止。

### 11.3 检查与录制

```bash
cd /home/esrobo/Projects/esrobo
source /opt/ros/humble/setup.bash
/usr/bin/python3 teleoperation/scripts/check_head_camera.py --seconds 15 --qos reliable
```

记录原始彩色、深度、内参和头部状态：

```bash
ros2 bag record -o "teleoperation/log/head_rgbd_$(date +%Y%m%d_%H%M%S)" \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info /head/state
```

网页预览旋转 180° 以修正倒装显示，但 ROS 原始图像、内参和录制数据不旋转。预览约 10 Hz，原始彩色和深度采集目标为 20 Hz。

## 12. 当前安全限制参考

下面只用于确认运行配置，不替代 [`TELEOPERATION_RUNTIME_LOGIC.md`](TELEOPERATION_RUNTIME_LOGIC.md) 的完整说明。

| 项目 | 当前值 |
| --- | --- |
| 控制循环 | 目标 50 Hz，按新机械臂反馈时间戳推进 |
| J1/J2 最大速度 | 26°/s |
| J3/J5/J6/J7 最大速度 | 24°/s |
| 左右臂 J4 位置范围 | −8°～120° |
| J4 最大速度 | 30°/s |
| J1-J7 最大加速度 | 40/40/45/50/50/50/50°/s² |
| 末端目标最大平移速度 | 0.22 m/s |
| 手腕 IMU J5-J7 | 8°/s、16°/s²、三轴合成偏移不超过 40° |
| 命令领先反馈 | J1-J4 3°，J5-J7 1.5° |
| 控制器速度比例 | 左右臂 30% |
| 回程速度 | 正常限值的 50% |
| 回程终点容差 | 1.5°，连续反馈确认 |
| PICO/手套数据过期 | 0.5 s 后不继续跟随 |

正常跟随持续检查控制器状态、软件力矩偏差、关节软限位、速度与加速度、命令/反馈领先量和对应制动距离。软件躯干几何检查按当前配置仅用于使能前姿态核验以及慢速回零的规划、执行；回零期间的间隙要求不变。

## 13. 软件验证命令

仅在没有运行实机遥操作进程时执行完整离线回归：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
PYTHONPATH=src:. /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python -m pytest tests -q
```

测试通过证明当前软件用例通过，不构成真机碰撞、线缆、负载或安装几何认证。现场仍需从安全姿态逐步低速验收。
