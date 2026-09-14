# ESROBO 遥操作运行手册

本文档记录通过 SSH 在机器人主机上启动 SenseGlove、LinkerHand 和 NERO 机械臂遥操作的步骤与命令。当前优先测试左手，左手套序列号为 `00885`。
影响实机行为的重要修改另见 `TELEOPERATION_CHANGELOG.md`。

## 1. 安全准备

1. 清空机械臂和灵巧手周围区域，人员远离夹点。
2. 确认急停可用，并安排一人持续托稳待测试机械臂。
3. 启动命令默认不会使能灵巧手或机械臂。
4. 不要使用 `--enable` 启动机械臂 IMU 模式。
5. 按 `s` 会立即停止跟随。末端模式让 J5-J7 回零；全臂模式先让 J4-J7 回零，再让 J1-J3
   回零，确认后使机械臂失能。联动模式随后让灵巧手缓慢张开。失能后机械臂可能受重力下落，
   按键前仍必须托稳。
6. 出现异常运动时立即按 `x` 发送电子急停，必要时使用实体急停。

### 1.1 第二阶段全臂联调软限位

全臂遥操作采用两层关节保护：URDF/SDK 硬限位只作为最后保护，正常运动还必须经过
逐关节软限位。当前第二阶段全臂联调使用以下范围，左右臂设置相同：

| 关节 | URDF 软限位 | 弧度范围 |
| --- | --- | --- |
| J1 肩部水平转动 | `-60 deg ~ +60 deg` | `-1.047198 ~ +1.047198` |
| J2 肩部前后摆动 | `-75 deg ~ +7 deg` | `-1.308997 ~ +0.122173` |
| J3 上臂旋转 | `-60 deg ~ +60 deg` | `-1.047198 ~ +1.047198` |
| J4 肘部屈伸 | `-8 deg ~ +85 deg` | `-0.139626 ~ +1.483530` |
| J5 前臂/末端旋转 | `-60 deg ~ +60 deg` | `-1.047198 ~ +1.047198` |
| J6 末端侧摆 | `-30 deg ~ +35 deg` | `-0.523599 ~ +0.610865` |
| J7 末端屈伸 | `-55 deg ~ +55 deg` | `-0.959931 ~ +0.959931` |

这些数值采用 **URDF 坐标**。J2 的 SDK 物理坐标带有 `+90 deg` 偏置，因此 J2 的
URDF 范围 `-75 deg ~ +7 deg` 对应 SDK 物理范围 `+15 deg ~ +97 deg`；查看反馈或
排查限位时不能混用这两套数值。驱动会根据每侧的方向和偏置自动转换，然后依次执行
SDK 硬限位余量、上述软限位、单周期步长和速度裁剪。

配置位于 `config/teleop_config.yaml` 的
`joint_soft_lower_limits_urdf` / `joint_soft_upper_limits_urdf`。旧的统一
`max_joint_deviation_from_start` 已设置为 `0.0`，避免它再次把七个关节统一限制为
相对启动姿态 `+/-20 deg`。扩大后的联调仍须从小动作开始，任何一轴方向错误、多人靠近、
接近身体/桌面或双臂可能相碰时立即按 `x`；在加入并验证自碰撞和桌面碰撞检测之前，
不要扩大本表范围。

本节软限位只用于完整双臂 IK 驱动。当前左/右手腕 IMU 单独调试模式仍使用各自的
J5/J6/J7 `40 deg` 专用角度限制以及 `8 deg/s`、`16 deg/s^2` 的速度和加速度保护，
不会因为全臂软限位配置而自动扩大范围。

### 1.2 第二阶段全臂联调速度限制

完整双臂 IK 驱动使用逐关节速度、加速度和单次反馈步长三层限制。当前参数如下：

| 关节 | 最大速度 | 最大加速度 | 单次反馈最大步长 |
| --- | --- | --- | --- |
| J1 | `18 deg/s` (`0.314159 rad/s`) | `45 deg/s^2` (`0.785398 rad/s^2`) | `2.2 deg` (`0.038397 rad`) |
| J2 | `18 deg/s` (`0.314159 rad/s`) | `45 deg/s^2` (`0.785398 rad/s^2`) | `2.2 deg` (`0.038397 rad`) |
| J3 | `18 deg/s` (`0.314159 rad/s`) | `50 deg/s^2` (`0.872665 rad/s^2`) | `2.2 deg` (`0.038397 rad`) |
| J4 | `20 deg/s` (`0.349066 rad/s`) | `60 deg/s^2` (`1.047198 rad/s^2`) | `2.5 deg` (`0.043633 rad`) |
| J5 | `20 deg/s` (`0.349066 rad/s`) | `60 deg/s^2` (`1.047198 rad/s^2`) | `2.5 deg` (`0.043633 rad`) |
| J6 | `20 deg/s` (`0.349066 rad/s`) | `60 deg/s^2` (`1.047198 rad/s^2`) | `2.5 deg` (`0.043633 rad`) |
| J7 | `20 deg/s` (`0.349066 rad/s`) | `60 deg/s^2` (`1.047198 rad/s^2`) | `2.5 deg` (`0.043633 rad`) |

左右 PICO 单臂调试覆盖值已与上表统一。`left_*` 与 `right_*` 三组七元素数组分别作用于对应机械臂，
同时用于跟随、停止及退出时的限速回零。
由于低速状态下从软限位边缘回零需要更长时间，`full_arm_return_timeout_s` 为 `30 s`；
该参数只延长回零验证等待时间，不会提高关节速度。
回零过程中允许最多 `1.0 s` 的短暂 CAN 反馈间隙，单帧缺失不会立即结束回零。关节进入
容差后必须连续取得 `3` 帧新鲜到位反馈；全臂第二阶段同时复核 J1-J7，确认所有关节仍在
零位容差内后才执行正常失能。相关参数为 `return_feedback_grace_s` 和
`return_verify_samples`。

配置项 `max_joint_velocity`、`max_joint_acceleration` 和 `max_joint_step` 均按
`J1...J7` 排列。驱动仍兼容旧配置中的单个标量，但首次全臂联调必须保留七元素数组。
执行顺序为：先裁剪软/硬角度范围，再根据反馈间隔限制速度，再从上一条实际命令速度
限制加速度，最后应用单次反馈步长。目标突然反向时，关节会先减速到零再反向，不会直接
翻转速度方向。

实时遥操作使用 `synchronize_joint_motion: false`。各关节分别在自己的速度、加速度和单步
限制内追踪连续目标，避免一个误差很大的关节把其他小角度关节按比例拖慢。最近一次右臂回零中，
J3 误差约 `57 deg`、J1 误差约 `5.6 deg` 时，旧的统一进度使 J1 只有约 `0.4 deg/s`；这种
“同时到达一个静态目标”的策略不适合持续变化的遥操作。关闭后仍不会跳过逐轴限速和限加速度，
回零也仍需所有相关关节连续三帧进入容差后才允许失能。

IK每周期执行一次Pink求解，但不再每帧完全退回机械臂反馈作为初值。程序从上一帧IK目标继续
收敛，并通过 `max_command_lead: 0.50` 和 `max_joint_position_delta: 0.50` 将计算目标限制在
实体反馈每轴 `+/-0.50 rad` 范围内。该前瞻只让网页 `IK结果` 更快跟上PICO目标；实际SDK命令
仍必须经过上表的逐关节速度、加速度、单步和软限位，不能以 `0.50 rad` 单步运动。

速度计算不再采用固定 `50 Hz`。左右机械臂分别读取 pyAgxArm 关节反馈携带的秒级
`timestamp`，并使用“当前已接受反馈时间戳 - 上一次命令所用反馈时间戳”作为各自
`dt`。时间戳没有前进、反馈无效或任一侧缺少新反馈时，本周期双臂命令被拒绝。固定
`max_joint_step` 仍作为反馈周期变慢或程序短时卡顿时的最后单周期保护。它不再按假定的
`50 Hz` 设置：实测 NERO CAN 关节反馈约为 `10 Hz` 时，旧的 `0.14~0.24 deg` 步长会把
实际速度额外压低到约 `1.4~2.4 deg/s`。当前 `2.2~2.5 deg` 只拦截异常大跳变，正常运动
主要由实际反馈 `dt`、上表速度和加速度共同决定。

PICO 产生的左右腕目标位置在进入 IK 前还会分别进行末端平移限速：

```yaml
retarget:
  max_endpoint_translation_velocity_m_s: 0.18
```

该限制将同侧肘点和腕点作为一条两连杆骨架同步推进：两个点共用同一个进度，过程中重新保持
机器人固定的大臂和前臂长度，并保证肘、腕各自的平移速度都不超过 `0.18 m/s`；目标腕姿态
四元数不变。重新按 `e` 以实测关节姿态重建机器人参考时，限速器会从新的实测肘腕位置重新
开始，不会继续追赶使能前的旧目标。SDK 的
`speed_percent: 25` 和平滑 `move_j` 模式共同生效。静默失能的左臂唤醒过程仍固定使用
`10%` 低速；只有自然下垂零位反馈验证通过后才切换到 `25%` 跟随速度。

## 2. SSH 登录与工作目录

登录机器人主机后进入项目：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
```

建议在 `tmux` 中运行，避免 SSH 中断后终端状态丢失：

```bash
tmux new -s senseglove_teleop
```

重新进入已有会话：

```bash
tmux attach -t senseglove_teleop
```

## 3. 通过 SSH 启动 SenseCom

当前连接方式是 SSH 登录机器人主机，SSH 会话不能访问 GDM 的图形桌面。即使
`/tmp/.X11-unix/` 中存在 `X0` 或 `X1001`，也不要据此设置 `DISPLAY=:0`、`:1` 或
`:1001`。项目使用独立虚拟显示 `Xvfb :99`，并以 Unity 的 `-batchmode -nographics`
模式运行 SenseCom。不要在 Xvfb 中运行普通 GUI 模式；SenseCom 1.9.2 会反复调整
窗口并可能因堆内存损坏以退出码 `139` 崩溃。

当前项目使用 SenseCom `1.9.2`，满足 Nova 2 BLE 所需的 `1.8.0` 或更新版本。官方
SenseCom 设置中没有“禁止休眠”或“永不关机”选项，`1.9.2` 的公开变更说明也没有
声明 BLE 空闲保活；因此即使 SenseCom 进程仍在运行，手套长时间完全静置后仍可能按
固件策略自行关机。代码不会用振动或力反馈命令冒充保活。先打开手套电源和蓝牙，
然后执行以下完整命令：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation

SENSECOM=/home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest
LOG_DIR=/home/esrobo/Projects/esrobo/teleoperation/log
mkdir -p "$LOG_DIR"

sed -n 's/.*"senseComVersion"[[:space:]]*:[[:space:]]*"\([0-9.]*\)".*/SenseCom version: \1/p' \
  "$SENSECOM/versionInfo.txt"

pgrep -f 'Xvfb :99([[:space:]]|$)' >/dev/null || \
  nohup setsid Xvfb :99 -screen 0 1280x720x24 -nolisten tcp \
    > "$LOG_DIR/xvfb-sensecom.log" 2>&1 < /dev/null &

sleep 2
DISPLAY=:99 xdpyinfo >/dev/null || {
  echo "ERROR: Xvfb :99 unavailable"
  exit 1
}

if ! pgrep -x SenseCom.x86_64 >/dev/null; then
  nohup setsid env DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 dbus-run-session -- \
    "$SENSECOM/SenseCom.x86_64" -batchmode -nographics \
    -logFile "$LOG_DIR/sensecom.log" \
    > "$LOG_DIR/sensecom-launch.log" 2>&1 < /dev/null &
fi

sleep 5
pgrep -a -f 'Xvfb :99([[:space:]]|$)'
pgrep -a -x SenseCom.x86_64
```

如果系统提示没有 `Xvfb` 或 `xdpyinfo`，只需安装一次：

```bash
sudo apt-get update
sudo apt-get install -y xvfb x11-utils dbus-x11
```

进程存在仅表示 SenseCom 已启动，不表示手套已经连接。纯 SSH 启动会自动重连曾在
SenseCom 中完成配对的手套；全新手套的首次配对需要进入可见桌面的 SenseCom 界面完成。

重启 SenseCom 时必须先退出遥操作，随后执行：

```bash
pkill -TERM -x SenseCom.x86_64
sleep 2
nohup setsid env DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 dbus-run-session -- \
  /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest/SenseCom.x86_64 \
  -batchmode -nographics \
  -logFile /home/esrobo/Projects/esrobo/teleoperation/log/sensecom.log \
  > /home/esrobo/Projects/esrobo/teleoperation/log/sensecom-launch.log 2>&1 < /dev/null &
```

完全关闭 SenseCom 及专用虚拟显示：

```bash
pkill -TERM -x SenseCom.x86_64
pkill -TERM -f 'Xvfb :99([[:space:]]|$)'
```

## 4. 检测 SenseGlove 数据连接

### 4.1 使用 SGCore 检测

SenseCom 启动后等待 5 至 10 秒，再在同一 SSH 终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
ros2 run senseglove_api sg_tester
```

左手连接成功时，应包含类似输出：

```text
Detected 1 glove(s):
 [0] ID: Nova 2-00885-L - isRight: false - deviceType: 3
  connected: true - rx: 90 pkt/s - tx: 1 pkt/s - connectionType: 2
  firmware: 2.0 - battery: 75% - charging: false
```

右手序列号应为 `00892`。允许实际包速率、电量和固件版本与示例不同，但必须同时满足：

- 输出包含目标序列号 `00885` 或 `00892`。
- `connected: true`。
- `rx` 持续大于 `0 pkt/s`。

`Detected` 但 `rx: 0 pkt/s` 仍表示没有实时数据，禁止开始标定。`bluetoothctl` 的
`Connected: yes` 仅代表系统蓝牙层，不能代替 SGCore 检测。

### 4.2 检测 ROS 手指和 IMU 数据

启动遥操作脚本前，可让配置脚本只检测手套，不修改文件：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/senseglove_auto_config.sh --no-write
```

只有需要自动更新 `gloves.yaml` 时才执行：

```bash
./scripts/senseglove_auto_config.sh
```

SenseGlove ROS 驱动启动后，检查左手状态消息：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
timeout 5 ros2 topic echo --once /senseglove/glove00885/lh/senseglove_states
```

消息中必须满足 `connected: true`、`packets_per_second_received` 大于 0，且
`hand_position` 与 `imu_orientation` 包含有效数据。右手检查命令为：

```bash
timeout 5 ros2 topic echo --once /senseglove/glove00892/rh/senseglove_states
```

结果处理：

- `SenseCom is NOT running`：回到第 3 节启动 SenseCom。
- `No SenseGloves detected`：确认手套开机、蓝牙开启，等待 10 秒后重试；仍失败则重启 SenseCom。
- `connected: false` 或 `rx: 0 pkt/s`：连接存在但无实时数据，禁止标定和使能。
- 出现 `SAFETY STOP`：手套已在遥操作中断连。重新连接后必须重启遥操作、重新标定并手动使能。

SenseGlove ROS 启动文件中的 Enter 自动确认只确认 SenseCom 进程已经运行，不会代替
蓝牙配对，也不能证明手套正在发送数据。

## 5. 只测试左灵巧手

该模式不会连接机械臂：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-only --recalibrate
```

标定流程：

1. 第一步让左臂和左手在身体侧面自然下垂，手指伸直并指向地面，掌心朝向身体，手腕保持平直，然后按 `Enter`。
2. 第二步让拇指指腹与食指指腹轻触；食指可自然弯曲，其余手指放松并尽量伸直，然后按 `Enter`。
3. 第三步完全握拳，拇指主动屈曲并收向掌心，然后按 `Enter`。中指和无名指使用张手到握拳的连续映射，小指跟随无名指。
4. 标定结束进入遥操作后，按 `h` 使能或停止灵巧手跟随。
5. 按 `q` 退出；程序会根据新鲜反馈将参与遥操作的灵巧手平滑恢复到自然张开位，然后关闭节点。

复用已有标定、不重新采集：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-only
```

## 6. 左手与左臂末端姿态遥操作

该模式使用左手 IMU 控制左机械臂末端第 5、6、7 关节，覆盖三个旋转自由度。其他机械臂关节保持实时反馈位置。末端三轴合成偏移限制为 `40°`，最大速度为 `8°/s`，最大加速度为 `16°/s²`，同时保留单步限制和机械臂硬限位。

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-wrist-imu
```

上面的默认模式为 `static-orthogonal`，标定保存到
`config/senseglove_esrobo_calibration_left.json`。它使用三个静态参考姿态拟合纯旋转矩阵 `C`，
不在运行时应用非正交剪切矩阵 `D`，与此前左手实际运行效果较好的处理一致。

完整保留旧式线性解耦的 `static-linear` 模式另存到
`config/senseglove_esrobo_calibration_left_linear.json`，只用于对照：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-wrist-imu --imu-calibration-mode static-linear
```

通常应使用默认的 `static-orthogonal`。`static-linear` 会在运行时额外应用非正交矩阵 `D`，
可能把标定动作中的人为串轴固化到遥操作结果中，仅用于诊断对照。

操作顺序：

1. 程序提示机械臂可能退出 leader/follower 模式并失能时，先人工托稳左臂，再按 `Enter`。程序先读取七轴使能状态：如果已经全部失能，则保留当前 CAN 反馈并直接继续，不重复发送模式或失能命令；如果仍有轴使能，才请求恢复 normal 模式、广播失能，并对仍使能的关节逐轴补发失能。只有取得七轴全部失能反馈后才继续，反馈缺失或任一关节仍使能都会中止并打印具体状态。
2. 完成六步标定。标定期间机械臂不会跟随手套：

| 步骤 | 应做动作 | 必须避免 |
| --- | --- | --- |
| 1. 中立位 | 手臂和手在身体两侧自然下垂；手指伸直朝地面，掌心朝身体，手腕平直 | 抬起手臂、手腕歪斜、手指弯曲 |
| 2. 掌心转向 | 先恢复中立位；把掌心转向身体正前方约 `25-35°`，保持该末端姿态 | 弯曲手腕、快速甩动手掌 |
| 3. 前后侧摆 | 先恢复中立位；把指尖向身体正前方抬约 `25-35°`，保持该末端姿态 | 为维持掌心朝身体而明显扭转前臂 |
| 4. 手腕弯曲 | 先恢复中立位；把指尖向身体内侧弯约 `25-35°`，保持该末端姿态 | 转动掌心、让手碰到腿部 |
| 5. 食指对指 | 先张手，拇指朝食指移动；食指可自然弯曲，直到两个指腹轻触 | 用力挤压、把其余手指握成拳 |
| 6. 握拳 | 回到舒适位置后完全握拳，拇指弯曲并收向掌心 | 拇指仍然伸直 |

这里的“正前方”和“身体内侧”都以操作者自身为准，不是机器人或屏幕画面的方向。
六步均在目标姿态保持静止。中指和无名指不采集单独对指端点，避免 SGCore 模型指尖距离及人为姿态差异扭曲连续屈曲映射；Nova 2 的小指屈曲与无名指联动，因此小指复用无名指进度。双臂机器人的默认姿态同样定义为手臂和手自然下垂、掌心朝向身体；因此第 1 步手套姿态与机器人 J5/J6/J7 零位姿态一致。
3. 标定结束后程序检查左臂 J5/J6/J7 和左灵巧手十个物理关节。机械臂目标为 `[0,0,0] rad`、容差为 `1 deg`；灵巧手目标为左手自然张开端点、每关节容差为 `12/255`。已经合格时不发送命令；存在超差时，托稳机械臂并按提示确认，程序才限速对齐，并在仍使能时验证到位，然后使七轴失能。失能后的重力漂移只打印诊断，不推翻失能前的零位验证；回零或失能未确认才会终止启动。
4. 进入遥操作后先按 `h`，单独确认灵巧手屈伸和拇指侧摆方向正确。
5. 暂时不要按 `e`。需要核对坐标方向时按 `i` 临时开启诊断，再小幅度依次转动手腕，观察 `[teleop IMU] ... [NO MOTION]`：沿手腕到中指方向的 X 轴扭转应只主要对应 J5；绕掌心法向 Y 轴的拇指/小指侧摆应只主要对应 J6；绕掌面横向 Z 轴的屈伸应只主要对应 J7，且左手套 Z 与机器人左臂 J7 的正方向相同。确认后再次按 `i` 关闭输出，此时机械臂保持失能。
6. 托稳左臂后按 `e`。按键不会重新定义零点：手套标定零点和当前相对角度会继续保留，机械臂的启动零位始终是末端物理关节 5/6/7 的 `[0, 0, 0]`；手套不要求回到标定中立姿态。
7. 按 `e` 后机械臂先在当前反馈位置使能，再按限速和限加速度策略将 J5/J6/J7 回到零位并确认；此阶段不应用手套目标。程序显示 `LEFT WRIST ZERO VERIFIED AND ARMED` 后，才从机械零位开始向手套当前的标定相对角度平滑跟随。机械臂回零、反馈或 IMU 数据任一无效都会失能且不进入遥操作。
8. 从小角度、低速度开始转动手腕，确认三个方向与灵巧手掌姿态一致。此时诊断行状态变为 `[ARMED]`。
9. 按 `s` 停止腕部跟随，将 J5/J6/J7 按限速、限加速度策略回到 `[0,0,0]`，确认后使左臂失能。IMU 超时或运动学映射失效时也采用先回零再失能；机械臂反馈无效时跳过回零并立即失能，避免盲发位置。按 `x` 立即发送电子急停，不等待回零；按 `q` 时采用相同回零流程，再使左臂失能、让左灵巧手平滑张开并退出。回零反馈失效或超过 10 秒时会放弃继续运动并执行失能。

左手 IMU 三轴采用程序规定的手套解剖 X/Y/Z 坐标，固定映射现为 `X -> J5`、`Y -> J6`、`Z -> +J7`；其中 Z/J7 方向根据最近一次左手单轴实测反转。Nova 2 每次上电后可能改变其外部参考基，因此标定先保存手臂自然下垂、手指朝地面且掌心朝身体时的 `q_neutral`，再用三个单轴参考姿态拟合本次开机的正交修正矩阵 `C`。左右手执行相同的直观动作，但由于掌心相向，其动作在各自解剖坐标中的符号不同；拟合程序分别使用左手 `[-X,+Y,+Z]` 和右手 `[+X,-Y,+Z]` 处理，不改变运行阶段的固定 XYZ 定义。当前 Nova 2 ROS 消息丢失四元数 `w` 的真实符号时，程序会用三轴动作同时比较 `+|w|` 与 `-|w|` 两个连续支路，选择残差更小的一支并保存；这一步只恢复缺失的四元数支路，不重新排列或动态翻转 XYZ。运行时先计算 `R_raw = inverse(R_neutral) * R_current`，再计算 `R_anatomical = C * R_raw * C^T`，之后才执行关节映射。终端诊断示例：

左手现在与已验证的右手采用相同的运行时抗串轴方法：从补偿后的标定零点开始，逐帧
计算手掌局部旋转增量并分别累计解剖 X/Y/Z 通道，然后对三个通道独立限幅。这里只共享
计算方法，不共享最终方向矩阵；左手现为 `X -> J5`、`Y -> J6`、`Z -> +J7`，右手
保持自身已经实测确认的方向。

`static-linear` 模式完整保留旧式线性方案：三个静态末端姿态先拟合 `C`，再计算非正交
`imu_axis_decoupling` 并在运行时应用，适合做 A/B 对照。最近两次左手静态标定得到的非对角项
约为 `6%-14%`，而且符号随标定动作变化，说明它会同时补偿手套基差和人的动作误差；因此不再
作为左手默认方案，但没有从代码或旧标定文件中删除。

这一结构参考 SenseGlove 官方 [`SG_TrackedHand.cs`](https://raw.githubusercontent.com/Adjuvo/SenseGlove-Unity/master/SenseGlove/Scripts/Tracking/SG_TrackedHand.cs) 的腕姿态修正方式：先从已知目标姿态与当前跟踪姿态计算并保存每只手独立的修正，再将修正持续应用到后续姿态。本项目增加三个单轴参考动作，是为了在只有 Nova 2 IMU、且开机参考基可能变化时同时辨认完整三轴坐标基，而不只修正一个中立姿态偏置。

```text
[teleop IMU] glove raw anatomical XYZ(deg)=[ 2.0 -1.0  3.0] => J5/J6/J7(deg)=[ 2.0 -1.0  3.0] [NO MOTION]
```

当前版本实际打印标签为 `glove raw anatomical XYZ(deg)`：左侧是限幅前的手套解剖轴累计角，右侧是经过每关节范围限制后的 J5/J6/J7 目标。因此到达关节范围时，右侧可以保持在限位，而左侧仍应随手套继续变化。正常的大幅手腕运动不会被当作通信中断；只有数据包实际停止更新才会触发超时保护。

IMU 诊断默认关闭。按 `i` 开启后，诊断中的角度是限幅后的目标偏移，不是机械臂当前反馈。正式使能前应分别做单轴小角度动作；若一个动作同时引起多个大幅关节目标，按 `q` 退出并重新标定，不要按 `e`。

左右灵巧手现在采用相同的手指标定和重定向数学过程：四指均使用
`gain=1.00, exponent=1.00`，在各自张手与握拳标定端点之间保持线性；拇指屈曲同样使用
`gain=1.00, exponent=1.00`，拇指侧摆/对掌都以各自的食指对指姿态作为方向端点，增益分别
为 `0.45/0.85`。中指和无名指不使用独立对指锚点，小指复用无名指的归一化进度。
左右手仍分别使用自己的采样数据、关节名称、URDF 镜像关系和实机伺服端点。当前左手 L10
配置尚未验证拇指 roll 的可用物理端点，因此该通道仍保持现有安全位置；不能为了数值对称而
直接复制右手的伺服端点。

## 7. 右手与右臂末端单独调试

右手模式只连接右手套、右灵巧手和右机械臂，不启动左手或左臂驱动。启动命令：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-wrist-imu
```

URDF 中左右臂的基座安装旋转为镜像，但在各自 `leftHand_link` / `rightHand_link` 局部坐标中，SDK 物理正向的 J5/J6/J7 均近似对应掌心局部 `Z/X/Y` 轴。多姿态标定先把本次开机的 Nova 2 测量基对齐到固定的解剖 XYZ；随后仍只在右手专用矩阵中执行既定的手套轴到机器人轴映射：

| 右手套实测分量 | 人手动作 | 右臂物理关节 |
| --- | --- | --- |
| `X` | 沿手腕到掌心的纵轴扭转 | `+J5` |
| `Y` | 绕掌心法向的拇指/小指侧摆 | `+J6` |
| `Z` | 绕掌面横向轴的手腕屈曲/伸展 | `-J7` |

右手标定数据的处理过程：

1. 每帧先将 ROS 消息中的 IMU 四元数归一化，并执行与 SenseGlove 官方 ROS 节点一致的 `unity_to_ros` 固定坐标修正；Nova 2 出现 `w=z` 异常时，先重建 `w` 并按上一帧选择连续符号。
2. 第一步让手臂和手自然下垂，手指伸直朝地面，掌心朝向身体，手腕平直。前 6 秒只用于稳定，随后约 4 秒采样；手指特征取逐项平均，四元数先统一正负号再平均并重新归一化，结果保存为 `imu_neutral`。
3. 第二至四步分别按照上表完成“掌心转向身体正前方”“指尖摆向身体正前方”“指尖弯向身体内侧”，每次动作前都必须先恢复第 1 步的自然下垂姿态。这三个右手动作依次定义为解剖轴 `+X/-Y/+Z`。程序先通过 SVD 正交拟合本次开机的纯旋转 `imu_frame_correction`，再计算带条件数限制的 `imu_axis_decoupling`，消除三个实测参考动作残留的非正交串轴；两个矩阵都不重新排列 XYZ。如果 ROS 数据中的 `w` 被 `z` 覆盖，还会同时拟合两个可能的 `w` 符号支路并保存唯一较优结果。动作小于 `10°`、三个动作无法组成可靠三轴、解耦条件数超过 `5` 或两个支路都超过 `25°` 时，不会带着错误矩阵进入遥操作。程序会保留第 1 步零点并显示 `XY/XZ/YZ` 轴夹角，只要求重新采集第 2-4 步；三个夹角越接近 `90°`，动作独立性越好。
   单手模式虽然会在桥接器内部镜像一份数据以保持双手数据包结构，但 IMU 坐标基只按实际连接的 `--single-side left/right` 拟合；镜像槽位直接复用该修正，不会用另一只手的动作符号重复判断。
4. 第五步只采集拇指与食指轻触的姿态，第六步握拳并采集闭合端点；这两步都不更新 IMU 零点或坐标基。食指对指姿态只用于确定拇指侧摆和对掌方向端点，不再根据实时指尖距离动态拉动拇指，也不再给食指、中指、无名指套用不同的分段屈曲曲线。四指统一使用张手到握拳的连续屈曲映射；小指复用无名指屈曲和侧摆进度，再分别映射到机器人自身的 URDF 范围。
5. 运行时先计算 `R_raw = inverse(R_neutral) * R_current`，再以 `C * R_raw * C^T` 补偿本次开机参考基差异，之后才应用固定的右手专用矩阵 `X -> J5`、`Y -> J6`、`Z -> -J7`。中立姿态只确定零点，三个参考动作只估计测量基到既定解剖基的差值，两者都不会修改固定轴定义。
6. 为降低屈伸和侧摆在复合姿态下的串轴，左右手都从各自补偿后的标定零点开始逐帧计算当前手掌局部坐标系中的旋转增量，并分别累计 X/Y/Z 通道，再使用当前侧的固定矩阵映射到 J5/J6/J7；右手保持已经验证的 J5/J6/-J7 方向，左手采用最新实测的 `J5/J6/+J7` 方向。左手只使用正交坐标基修正，不应用由非正交标定动作拟合出的剪切矩阵。
7. 当前 Nova 2 ROS 消息存在 `imu_orientation.w == imu_orientation.z` 的实测异常，桥接层会重建 `|w|`。程序不按正常大幅手腕运动拒绝数据；最终目标仍受末端关节角度、速度和加速度限制，只有 IMU 数据包实际中断并超过通用通信超时时间时，才执行限速回零和失能保护。

右臂末端 J5/J6/J7 各轴角度范围分别限制为 40°；某一轴到达限制时不会缩放另外两轴。最大跟随速度为 8°/s，最大加速度为 16°/s²，每周期最大步长约 0.23°。关节硬限位、软限位余量、实时反馈检查和全关节失能保护仍然生效。

右灵巧手四指使用 `gain=1.00, exponent=1.00`，在张手和握拳端点之间做连续线性映射，
不再使用中指、无名指对指中间锚点。食指对指姿态只作为拇指方向参考；拇指屈曲仍由
`thumb_mcp/pip/dip` 和少量骨架内部屈曲共同计算，侧摆及对掌分别使用该参考姿态归一化。

LinkerHand L20Lite 与当前 SDK 中标识为 L10 的实机是同一只手：L10 表示 10 路主动执行器
接口，L20Lite URDF 则把同一机构的主动关节和 mimic 随动指节完整展开。例如一个拇指
`thumb_cmc_pitch` 主动量会按 URDF 中的 `1.38/1.49` 比例同时驱动 MCP/IP。因此该 URDF 可以
作为实机正运动学模型使用。当前使用[厂商官方 L20Lite URDF](https://github.com/linker-bot/linkerhand-urdf/tree/075cc7d42cc1e756bdcbece0fc069a0779fc5237/L20lite)的关节链、mimic 比例和指尖网格
范围离线核对关节方向和限位；运行时不做在线无约束 IK，所有输出继续经过原有单步和速度限制。

### 拇指三个自由度的数据来源

SenseGlove ROS 的 `SenseGloveState.position` 来自 SGCore `HandPose.GetHandAngles()`。Nova 2
右手消息依次提供 `thumb_mcp`、`thumb_pip`、`thumb_dip` 和 `thumb_brake`：前三项是官方
手模型分解后的指节屈曲角，`thumb_brake` 在状态发布器中取 `-HandAngles[thumb][0].Z`，
表示拇指外展/内收。当前映射为：

| 灵巧手自由度 | SenseGlove 输入 | 处理方式 |
|---|---|---|
| `thumb_cmc_pitch`，拇指屈曲 | `thumb_mcp/pip/dip` | 三个官方屈曲角归一化，并以少量骨架内部屈曲补充 |
| `thumb_cmc_yaw`，拇指侧摆 | `thumb_brake` | 使用张手到食指对指姿态归一化，yaw 不超过当前安全增益 |
| `thumb_cmc_roll`，拇指对掌 | 近端骨段 elevation | 使用张手到食指对指姿态归一化 |

Nova 2 没有独立测量人手拇指轴向旋转的传感通道，因此 `thumb_cmc_roll` 是由 SGCore
手骨架近端姿态得到的对掌近似，不应解释成独立的实测轴向角。桥接器还直接根据
`finger_tip_position` 计算拇指尖到食指、中指、无名指和小指尖的四个欧氏距离；算法与
官方 `finger_tip_distance_node` 相同。这些距离继续显示在右手网页诊断页，仅用于诊断，
不再参与实时拇指或四指目标计算。

修改前使用的 `vector_thumb_tip_spread/elevation` 只保留为诊断量。单独弯曲拇指时，网页
中的 `SDK flex (pitch)` 可以变化，但 `SDK abduction (yaw)` 应基本不变；单独做拇指侧摆
时应主要看到 yaw 通道变化。如果手套本次穿戴位置与原标定差异明显，在启动命令后添加
`--recalibrate` 重新采集端点，再从小幅动作开始验证。

右 L10 的 SDK 标准握拳端点仍为 `[73,75,0,0,0,0,110,110,120,78]`；遥操作单独使用 `right_teleop_closed=[40,40,0,0,0,0,110,110,120,100]`，扩大物理索引 `0/1/9` 对应的拇指根部屈曲、侧摆和旋转行程。该范围位于厂商 L10 示例已经使用过的区间内，并继续受 `max_step=18` 和 `max_velocity=400` 限制。启动、零位检查和按 `q` 退出仍使用 `right_open=[255,210,255,255,255,255,110,110,120,33]`，所以扩大跟随范围不会改变自然张开零位。

实机顺序：

1. 启动器先自动启动或复用无图形 SenseCom，并验证右手套 `Nova 2-00892-R` 为 `connected: true` 且 `rx > 0`；验证失败时不会初始化机器人 CAN。
2. 程序提示右臂即将退出 leader/follower 模式并失能时，人工托稳右臂并排除夹点，再按 `Enter`。程序依次执行 `set_normal_mode()` 和全关节 `disable()`，不发送位置目标；未验证七轴全部失能时会立即中止。
3. 右手与左手采用相同的六步采集流程，不额外重置手套内部标定。第一步要求手和手臂自然下垂、手指朝地面且掌心朝身体，与机器人默认下垂姿态一致；第二至四步按上表拟合本次开机参考基与固定解剖 XYZ 基之间的差值；第五步采集食指对指方向端点，第六步采集握拳端点。中指和无名指不再采集独立对指端点，小指复用无名指运动进度。`--right-wrist-imu` 和 `--left-wrist-imu` 每次启动都会强制完成这套标定，避免复用手套重新上电前的 IMU 修正矩阵。
4. 标定结束后程序先执行机器人初始姿态门禁。NERO 的机械零位定义为 SDK 物理 J5/J6/J7=`[0,0,0] rad`；LinkerHand 的逻辑零位定义为配置中的自然张开端点，而不是十个 SDK 数值全部为 `0`。右 L10 当前稳定的自然张开反馈为 `[254,210,254,254,254,254,109,109,119,33]`，其中拇指屈伸轴约为 `254`；旧记录中的 `141` 是异常反馈，不能作为零位。程序使用 `right_open` 发送张开命令、使用 `right_open_feedback` 验证反馈零位。程序会打印每个物理关节的反馈、反馈零位目标及最大误差。机械臂容差为 `1 deg`，灵巧手每个关节容差为 `12/255`。
5. 若反馈已经处于容差内，程序只验证而不发送位置命令。若存在超差，会要求人工托稳机械臂、排除夹点并按 `Enter`：机械臂保持 J1-J4 的实时反馈，仅以既有限速和加速度限制将 J5-J7 对齐到零位，在使能状态下确认到位后立即全关节失能；灵巧手从实时反馈平滑移动到对应侧自然张开端点。机械臂失能后可能因重力离开零位，此反馈只作为诊断；失能前回零未确认、七轴失能未确认或灵巧手复核失败时才终止启动，`h/e` 不可用。
6. 进入遥操后先按 `h`，只验证右灵巧手各指屈伸、拇指屈曲和侧摆；右臂仍保持失能。
7. 暂时不按 `e`，按 `i` 开启诊断。依次做小幅纵轴扭转、拇指/小指侧摆、手腕屈伸，分别应主要看到 `J5`、`J6`、`J7` 变化；当前右手实测分量对应关系为 `X -> J5`、`Y -> J6`、`Z -> -J7`。
8. 如果一个单轴动作同时产生多个大幅关节目标，不要按 `e`；按 `q` 退出并保留诊断输出用于调整。
9. 三轴方向确认后关闭 `i` 并托稳右臂，再按 `e`；手套不要求回到标定中立姿态。程序读取并保留当前 IMU 相对角度，但先限速将右臂 J5/J6/J7 回零并确认，随后才显示 `RIGHT WRIST ZERO VERIFIED AND ARMED`，从机械零位开始向当前 IMU 目标平滑跟随。从小角度、低速度开始；正常停止按 `s`，程序会先将 J5/J6/J7 缓慢回零并确认，再使右臂失能；异常运动按 `x` 立即急停，并在必要时使用实体急停。
10. 按 `q` 退出时，程序将限速返回 J5/J6/J7 零位，再失能右臂并将右灵巧手平滑张开。

右手三维骨架诊断页面为 `http://192.168.10.100:8766`。左右 IMU 映射分别保存在 `left_hand_imu_local_rotvec_to_robot_hand` 和 `right_hand_imu_local_rotvec_to_robot_hand`；右手后续单独调整不会改变左手。

## 8. 其他启动命令

只测试右手套和右灵巧手，不连接右机械臂。首次测试或手套重新上电后执行：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-only --recalibrate
```

该命令只启动右手 SenseGlove 和右灵巧手链路，不初始化 `can_piper2`，也不会发送右机械臂
使能、失能或位置指令。纯灵巧手模式不会使用 IMU，依次采集“自然张手”、拇指与
食指轻触、以及“完整握拳”共三步。中指和无名指不采集独立对指端点，小指复用无名指进度。对指时食指允许自然弯曲，其余
手指放松并尽量伸直，两个指腹轻触即可，不要用力挤压。按照终端提示完成标定后，按 `h` 开始或停止右灵巧手跟随，按 `q`
让右灵巧手平滑恢复自然张开位置并退出。

只有显式加入 `--right-wrist-imu`（或左侧的 `--left-wrist-imu`）才会增加 IMU 三轴动作，
形成六步标定。若六步
模式提示源四元数的 `w` 持续缺失或与 `z` 重复，且三轴拟合被拒绝，程序会直接中止腕部
遥操作而不再无限重复第 2-4 步；不要放宽拟合阈值或按 `e`。此时仍可去掉
`--right-wrist-imu`，使用上面的三步命令安全测试手指，待 SenseCom/SGCore 提供有效四元数后
再恢复腕部测试。

同一次手套上电期间复用已有标定：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_senseglove_hand_teleop.sh --right-serial 00892 --right-only
```

双手灵巧手，不启动机械臂：

```bash
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --right-serial 00892 --recalibrate
```

只做无硬件链路测试：

```bash
./scripts/run_senseglove_hand_teleop.sh --left-serial 00885 --left-wrist-imu --no-hardware
```

## 9. 按键说明

按键监听仅在前台遥操作终端有效，按下后不需要再按 `Enter`。未列出的按键不会触发控制动作。

| 按键 | 适用模式 | 功能与安全行为 |
| --- | --- | --- |
| `h` | 所有灵巧手模式 | 切换灵巧手跟随。首次按下请求使能并显示 `HANDS ENABLED`，再次按下停止并显示 `HANDS DISABLED`，之后可以再次按下恢复；没有新鲜灵巧手状态反馈时会拒绝使能。 |
| `e` | 机械臂或手腕 IMU 模式 | 检查输入和机械臂反馈；在全关节失能状态下清除先前 `x` 留下的控制器急停锁存，再设置控制模式、预装实测姿态并复核七轴使能后开始跟随。手腕 IMU 模式只控制当前侧末端 J5/J6/J7；检查失败会拒绝使能。仅灵巧手模式下不要使用。 |
| `s` | 机械臂或手腕 IMU 模式 | 正常停止机械臂跟随。手腕 IMU 模式同时请求机械臂失能，机械臂可能在重力下落，按键前必须托稳。该键不关闭灵巧手跟随。 |
| `x` | 机械臂或手腕 IMU 模式 | 停止跟随并向参与遥操作的机械臂发送电子急停。异常运动时优先使用，必要时同时按实体急停。 |
| `i` | 左/右手腕 IMU 模式 | 开启或关闭 IMU 坐标诊断，默认关闭。开启后每 0.5 秒输出手掌局部 XYZ 和 J5/J6/J7 目标偏移；只改变日志显示，不使能机械臂、不改变控制目标。 |
| `d` | 全臂模式 | 人工确认后的直接失能。程序立即停止跟随并发送失能，不尝试回零；机械臂可能受重力下落，必须先托稳。 |
| `q` | 所有模式 | 安全退出。手腕 IMU 模式先保持前四轴实时位置，将 J5/J6/J7 限速回到 `[0,0,0]`；连续 3 帧确认到位后使机械臂失能，再让灵巧手平滑张开。全臂模式分阶段回零并复核 J1-J7；若回零未确认，则取消退出、锁住跟随并保持使能，等待按 `s` 重试、按 `d` 失能或按 `x` 急停。 |
| `Ctrl+C` | 所有模式 | 中断程序并进入清理流程。正常结束优先按 `q`，以执行完整的灵巧手张开流程。 |
| `Enter` | 启动和标定阶段 | 确认托稳机械臂、SenseCom 已启动或开始当前标定步骤；进入实时遥操作后没有控制作用。 |

推荐顺序：先按 `h` 单独验证灵巧手；IMU 模式下按 `i` 检查小幅三轴方向并关闭诊断；托稳机械臂后再按 `e`。正常停止用 `s`，异常时用 `x`，结束时用 `q`。

## 10. 通信与日志检查

查看当前 ROS 2 手套话题：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
ros2 topic list | grep senseglove
```

读取一次左手套状态：

```bash
ros2 topic echo --once /senseglove/glove00885/lh/senseglove_states
```

读取一次左灵巧手反馈：

```bash
ros2 topic echo --once /cb_left_hand_state
```

检查 CAN 接口：

```bash
ip -details -statistics link show type can
```

持续查看主要日志：

```bash
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/senseglove_driver.log
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/senseglove_bridge.log
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/linker_hand_driver.log
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/hand_ros_bridge.log
```

## 11. 常见故障

### SenseCom 找不到手套

确认手套未自动关机、蓝牙已开启，并重新启动 SenseCom。SenseCom 中连接成功后再运行 `sg_tester`，不要只依赖系统蓝牙设备列表。

如果 `sensecom.log` 反复出现以下内容，表示旧 SenseCom 仍记得手套名称，但其缓存的 BLE 地址已经为空，继续复用该进程通常无法恢复：

```text
Failed to construct a SgBleAddr from the string '': IncorrectByteCount
```

一键启动器会在机器人 CAN 初始化前等待目标手套；首次失败时，它会精确清理所有旧 SenseCom 主进程和重复实例，只启动一个新实例，等待 BLE 重新扫描并再次验证目标手套。第二次仍失败才停止启动，因此恢复过程不会触发灵巧手或机械臂运动。

### 静置后手套关机或蓝牙断开

1. 先用上面的版本命令确认 SenseCom 不低于 `1.8.0`，并确保运行的是项目内路径，而不是旧副本。SenseCom 设置页面没有关闭 Nova 2 自动休眠的开关。
2. 执行 `ros2 run senseglove_api sg_tester`，要求 `connected: true` 且 `rx` 大于 0。
3. 电量低于 20% 时先充电；低电量叠加触觉输出更容易造成连接中断。
4. 长时间调试时优先给手套接入 USB 充电，并偶尔活动手指和手腕。官方资料只明确建议在固件升级期间接充电器以防关机，不能据此保证所有固件在无限静置时永不休眠。
5. 不要周期性发送振动或力反馈作为软件保活：官方没有说明这种方式能关闭休眠，而且会造成额外耗电和非预期触觉输出。
6. 遥操作中检测到断连后，桥接器立即停止发送手指和 IMU 旧目标；持续 1 秒即退出。
7. 手套重新开机后，先在 SenseCom 中重连，再重新运行启动命令并完成标定。程序不会自动恢复遥操作，灵巧手需重新按 `h`，机械臂需重新按 `e`。

日志中的正常状态类似 `connected=True/rx=90pps`。出现 `SAFETY STOP` 后不要尝试绕过
退出保护，否则恢复连接时可能把断连前的旧目标重新发送给机器人。

纯 SSH 模式下检查日志：

```bash
tail -n 100 /home/esrobo/Projects/esrobo/teleoperation/log/xvfb-sensecom.log
tail -n 100 /home/esrobo/Projects/esrobo/teleoperation/log/sensecom.log
```

### 标定样本为 0

确认 `/senseglove/glove00885/lh/senseglove_states` 正在持续发布，并检查 `senseglove_driver.log`。只有收到新鲜手指和 IMU 数据后才能开始标定。

如果终端状态显示 `payload=all-zero/invalid`，表示 ROS 话题虽有发布频率，但 SenseCom/SGCore 没有提供真实姿态，不能继续标定。退出遥操作程序、重启 SenseCom、确认 `sg_tester` 成功列出手套后再试。不要通过继续增大拇指动作来绕过该错误。

### IMU 三轴串轴或出现大角度跳变

部分 Nova 2/SGCore 组合会把四元数 `w` 错误发布成与 `z` 完全相同的值。检查左手原始数据：

```bash
source /opt/ros/humble/setup.bash
source /home/esrobo/Projects/esrobo/teleoperation/external/senseglove_ros/install/setup.bash
ros2 topic echo --once /senseglove/glove00885/lh/senseglove_states senseglove_msgs/msg/SenseGloveState
```

若 `imu_orientation.w` 与 `imu_orientation.z` 完全相等，桥接器会重建缺失的 `w`，并利用上一帧选择连续分支。即使原始四元数范数碰巧接近 `1.0` 也必须修复；否则标定与运行阶段可能采用不同四元数，引起 Y/Z 串轴。此类数据下 `senseglove_bridge.log` 应持续显示 `imu_w_repair=left:True`，不能在手腕转动时反复切换 True/False。修改桥接代码后必须退出旧进程并重新启动、重新标定；Python 进程不会热加载修改。

### `HAND ENABLE REFUSED`

表示没有收到新鲜、有效的灵巧手状态反馈。检查 `/cb_left_hand_state` 和 `linker_hand_driver.log`，不要绕过反馈门控。

### `embedded_version` 为 `None` 或没有 CAN 响应

这表示 LinkerHand 驱动发出了固件版本查询，但没有收到灵巧手返回帧。新版驱动会最多进行三次只读查询；仍失败时在发送启动速度、力矩或位置配置前退出，不再触发 `len(None)`。

```bash
ip -details -statistics link show can_hand1
tail -n 60 /home/esrobo/Projects/esrobo/teleoperation/log/linker_hand_driver.log
```

右手应使用 `can_hand1`。接口为 `ERROR-ACTIVE` 只说明主机 CAN 控制器已上线；如果统计持续为 `TX > 0`、`RX = 0`，应检查右灵巧手供电、CAN 插头、线缆和终端电阻，以及 USB-CAN 端口是否映射到右手。此时禁止绕过反馈门禁或发送位置命令。

### 按 `e` 后拒绝使能

根据终端提示区分原因：

- `IMU is missing/stale`：手套 IMU 数据缺失或超时。
- `invalid left-arm joint feedback`：机械臂关节反馈无效。
- `coordinate mapping failed`：当前关节姿态无法建立末端三轴映射。
- `motor enable failed`：SDK 未确认七个关节全部使能。

排除原因后保持机械臂被托稳、手套处于中立姿态，再次按 `e`。禁止通过删除反馈检查或直接发送大角度位置命令来绕过故障。

## 12. PICO 左臂与 SenseGlove/左灵巧手遥操作

默认模式只初始化左机械臂 `can_piper1`，不会连接 SenseGlove、右机械臂或两只灵巧手。PICO 的
左肩、左肘、左腕空间位置只参与左臂 `J1...J4` 的位置 IK；`J5/J6/J7` 每周期保持各自最新
机械臂反馈位置，不使用 PICO 左腕四元数，也不会主动拉向固定末端角度。需要测试完整七轴时，
显式添加 `--with-left-imu`，或使用专用联动脚本，SenseGlove 标定后的掌心姿态才会投影到
J5/J6/J7，同时手指姿态控制左灵巧手。手套断流后不会自动退回 PICO 姿态。

位置 IK 使用完整 URDF 中的肩、肘和腕连杆，但硬锁定非机械臂关节、右臂以及左臂 J5-J7，
因此大臂方向由肩部 J1-J3 配合、前臂相对大臂的屈伸主要由 J4 完成，腕部姿态误差不会再被
分摊到肩肘关节。启用联动模式后，程序先用 PICO 位置目标计算 J1-J4 已经产生的掌心世界旋转，
再从手套 IMU 的目标掌心姿态中扣除这一部分，只把剩余的腕部相对旋转投影到 J5-J7 的实际
掌心局部轴，避免整条手臂转动被末端关节重复执行。NERO 七轴反馈仍由 pyAgxArm/CAN 读取，
用于 IK 初值、零位检查、实际反馈 `dt` 和安全限速。肩点、肘点和腕点必须全部来自同一次
实测关节状态的 URDF FK；禁止把旧硬编码肩坐标与当前 FK 肘腕坐标混用，否则会把约 `0.31 m`
的大臂误算成约 `0.65 m` 并产生不可达目标。

### 12.1 是否需要标定

默认纯 PICO 模式只进行自然下垂参考采集，不需要 SenseGlove 标定。PICO/OpenXR 每次连接后的
空间原点和人体比例可能变化，因此程序要记录本次会话中“左臂自然下垂”的左肩、左肘、左腕
关系。添加 `--with-left-imu` 或使用联动脚本时，会额外进行 SenseGlove 六步标定：第 1-4 步
建立 IMU 零点和本次上电坐标基，第 5-6 步采集对指、握拳和手指屈曲端点，随后用于控制左
灵巧手。第 1 步要求人手臂自然下垂、掌心朝向身体，这与机器人自然下垂和末端零位一致。

PICO 的肩、肘、腕点在进入标定前已通过固定坐标矩阵转换到机器人基座坐标。自然下垂标定只
使用“人体上臂向下方向 -> 机器人上臂零位方向”的最小旋转修正佩戴倾斜，不再从近共线的
肩、肘、腕三点推算绕手臂轴的方向。因此标定不会交换 H1 前抬与 H2 左侧抬的水平轴：H1
应主要驱动 J1，H2 应主要驱动 J2；J1/J2 仍会因串联机构和肘腕位置约束产生小幅组合运动。

采集时站直，左臂在身体左侧自然下垂，肘部自然伸直但不要用力锁死，手腕保持自然；保持静止
直到终端出现：

```text
[PICO 标定] 完成：自然下垂参考零点已锁定。机械臂仍未运动；请继续托稳机械臂，确认安全后按 e 开始遥操作。
```

稳定性门限为肩、肘、腕位置标准差不超过 `0.05 m`，至少采集 5 帧。PICO 参考未锁定时按 `e`
会被拒绝。纯 PICO 模式不检查手套；组合模式下左手套 IMU 超过 `0.5 s` 未更新时也会拒绝使能。
PICO 腕姿态字段仅随数据包保留供诊断，控制过程不读取它来生成 J5-J7 目标。

### 12.2 机器人初始零位

左臂上电失能后可能在重力作用下停在非零位置。若启动时能读取完整七关节反馈，但任一关节
超过启动零位容差，程序不再直接报错退出，而会提示人工托稳并按 `Enter`。确认后首先把
当前实测七关节角作为使能保持目标，防止恢复旧控制器目标；然后按左臂专用速度、加速度和
单步限制缓慢回到 URDF 自然下垂零位。只有回零已验证且七关节失能已确认后，才允许继续
标定和后续按 `e` 进入遥操作。没有完整反馈时仍禁止自动回零。

URDF 将自然下垂定义为左臂 `J1...J7 = [0,0,0,0,0,0,0] rad`。硬件 SDK 的 J2 坐标带
`+pi/2` 偏置，所以同一姿态的物理反馈应接近：

```text
[0, 1.5708, 0, 0, 0, 0, 0] rad
```

启动器读取真实七轴反馈并换算为 URDF 角度；任一轴偏离零位超过 `8 deg` 时立即退出，保持
机械臂失能，不会自动从未知姿态进行大范围回零。应在断电或确认失能、并持续托稳机械臂时，
人工将其放回自然下垂姿态后重新启动。

### 12.3 首次配置 XRoboToolkit 环境

本机首次使用，或 `/opt/apps/roboticsservice/RoboticsServiceProcess`、`teleop_esrobo` Conda 环境、
`xrobotoolkit_sdk` 任一项缺失时，执行：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/setup_xrobotoolkit.sh
```

该脚本执行以下操作：

1. 安装官方 Ubuntu 22.04 amd64 版 XRoboToolkit PC Service `1.0.0.0`。下载包使用固定 SHA-256
   校验，已经安装时不会重复安装。
2. 缺少 `teleop_esrobo` 时创建 Conda 环境；缺少 `xrobotoolkit_sdk` 时编译并安装仓库内的 Python
   客户端及 `libPXREARobotSDK.so`。
3. 以适合 SSH 无图形会话的方式启动 PC Service，并确认本地 SDK 可以连接
   `127.0.0.1:60061`。

此步骤需要联网，安装系统软件时会提示输入 `sudo` 密码。它不配置 CAN、不启动机械臂驱动，
也不发送任何机器人运动命令。安装结果可以只读检查：

```bash
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' roboticsservice
test -x /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python && echo "Conda 环境存在"
```

### 12.4 启动 PC Service 并连接 PICO

#### 当前机器的推荐网络拓扑

双臂机器人主机不需要加入外部无线网络。当前机器应同时使用两条互不冲突的连接：

```text
远程操作主机 <--网线/SSH--> 双臂机器人主机 <--Wi-Fi 热点--> PICO
                    192.168.10.100         10.42.0.1
```

本机已经配置 NetworkManager 热点，连接名为 `MyHotspot`、SSID 为 `esrobo`、热点地址为
`10.42.0.1/24`，并设置为开机自动启动。PICO 应连接 Wi-Fi `esrobo`，XRoboToolkit 中的 PC
Service 地址填写 `10.42.0.1`；不要填写 `127.0.0.1`，也不要填写仅网线侧可达的
`192.168.10.100`。SSH 仍通过网线工作，PICO 接入热点不会改变当前 SSH 路径。

只读确认当前网络状态：

```bash
nmcli -t -f NAME,TYPE,DEVICE,STATE connection show --active
ip -br address
ip neigh show dev wlp89s0
```

应看到 `MyHotspot` 绑定 `wlp89s0`，其地址为 `10.42.0.1/24`。PICO 接入后，最后一条命令应
出现一个 `10.42.0.x` 邻居。需要在 PICO 中重新输入热点密码时，可在机器人主机本地查询：

```bash
nmcli --show-secrets -g 802-11-wireless-security.psk connection show MyHotspot
```

如果 `MyHotspot` 没有启动，可执行：

```bash
nmcli connection up MyHotspot
```

不要把无线网卡切换成连接其他 Wi-Fi 的客户端模式，否则会关闭这个 PICO 接入热点。若必须让
PICO 连接远程操作主机创建的另一个热点，就需要在该远程主机上额外配置从无线网段到机器人
网线地址 `192.168.10.100:63901` 的路由/端口放行；这种跨主机方案更容易受防火墙和网络共享
规则影响，首次联调不采用。

每次主机重启后，在 SSH 终端启动服务：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh start
./scripts/xrobotoolkit_pc_service.sh status
```

正常结果必须同时包含 `RUNNING` 和 `127.0.0.1:60061 LISTENING`。常用管理命令：

```bash
# 查看状态
./scripts/xrobotoolkit_pc_service.sh status

# 服务异常时重启
./scripts/xrobotoolkit_pc_service.sh restart

# 查看服务日志
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/xrobotoolkit_pc_service.log

# 完成全部遥操作并退出后，按需关闭服务
./scripts/xrobotoolkit_pc_service.sh stop
```

`60061` 正常只代表本机 Python SDK 已连接 PC Service，不代表 PICO 已经输出人体数据。PICO 侧还要：

1. 让 PICO 连接本机 SSID `esrobo`；本机热点地址为 `10.42.0.1`。
2. 在 PICO 中启动 XRoboToolkit 应用，并选择/连接这台 PC Service 主机。PC Service 当前使用
   `10.42.0.1:63901` 接收设备连接，本机 Python SDK 则通过 `127.0.0.1:60061` 读取服务数据。
3. 开启全身追踪；按 XRoboToolkit 要求连接并校准人体追踪设备。完整全身追踪通常至少需要两只
   PICO Motion Tracker（Swift），且应在头显中先完成追踪器校准。
4. 站到可正常追踪的位置，确认头显应用内人体骨架随动作更新。

在不初始化 CAN 和机器人硬件的情况下，单独检查左肩、左肘、左腕数据：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh start
env LD_LIBRARY_PATH="$PWD/external/XRoboToolkit-PC-Service-Pybind/lib:/home/esrobo/miniconda3/envs/teleop_esrobo/lib:${LD_LIBRARY_PATH:-}" \
  /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python \
  ./scripts/check_xrobotoolkit_body.py --side left --wait-seconds 20
```

单独检查右肩、右肘、右腕时，将最后一行改为：

```bash
  ./scripts/check_xrobotoolkit_body.py --side right --wait-seconds 20
```

只有出现 `PICO BODY DATA READY` 才表示可以进入机械臂联调。部分 PICO/XRoboToolkit 版本会在
骨架正常更新时仍把专用 `Body.timeStampNs` 保持为 0；检查器此时会回退到通用 XR 时间戳，并且
只有连续两帧时间戳前进、左肩/肘/腕位姿不是全零占位值时才放行。成功提示中的
`(xr-fallback)` 表示采用了该兼容路径，不代表没有人体数据。如果显示
`body_data_available=False`、两个时间戳都不前进或等待超时，才说明 PICO 应用未提供新鲜骨架；
此时检查全身追踪、追踪器校准和应用状态，不要操作机械臂。

### 12.5 SSH/tmux 启动单臂遥操作

#### 本机实时查看双臂骨架

PICO 数据桥会同时启动一个只读网页调试服务。它直接读取桥内存中的同一份 PICO 数据，不监听
遥操作核心使用的 `15050/UDP`，不会抢占数据，也不会向机械臂、灵巧手或 PICO 发送控制命令。
网页仅绑定机器人主机的 `127.0.0.1:8765`，远程 SSH 调试时应先在**操作人员本机电脑的本地终端**
建立端口转发。不要在已经登录机器人后的 SSH shell、VS Code Remote SSH 终端或机器人 tmux 中
运行下面这条命令，否则该自连接会反过来占用机器人端 `8765`，导致 PICO 数据桥启动失败：

```bash
ssh -L 8765:127.0.0.1:8765 esrobo@192.168.10.100
```

在这个 SSH 会话中按下文启动遥操作，或者保持现有遥操作 tmux 会话不动，在本机另开终端执行：

```bash
ssh -N -L 8765:127.0.0.1:8765 esrobo@192.168.10.100
```

数据桥启动后，在本机浏览器打开：

```text
http://127.0.0.1:8765
```

页面同时绘制左、右肩肘腕骨架，并显示关键点坐标、左右大臂/前臂长度、数据年龄及真实源更新率。
视图下拉菜单用于逐层检查数据链路：

| 视图 | 表示的数据 |
| --- | --- |
| `PICO 原始` | XRoboToolkit 直接给出的肩、肘、腕位置。 |
| `固定映射` | 使用当前 `source_to_robot_rotation` 转换后的骨架；尚未经过自然下垂标定。 |
| `重定向目标` | 经过自然下垂最小旋转和机器人臂长缩放后，送给 IK 的肩、肘、腕目标。 |
| `IK 结果` | 用 J1-J4 IK 目标角度进行 URDF FK 后的机器人骨架。 |
| `限速命令` | 经过软限位、反馈 `dt`、逐关节速度/加速度/单步限制后实际交给 SDK 的骨架。 |
| `实际反馈` | 使用机械臂最新反馈角度进行 URDF FK 得到的实体机器人骨架。 |

右侧 `左臂关节角` 同时列出 J1-J7 的 `IK / 命令 / 反馈`。诊断数据通过本机
`127.0.0.1:15060/UDP` 从遥操作核心单向发送到网页服务，不参与机器人控制。
若原始视图已错位或跳动，问题位于 PICO 追踪或 XRoboToolkit 输入；若原始视图正确而映射视图
方向错误，则应检查固定坐标矩阵；若重定向目标正确但 IK 结果错误，检查 URDF/IK；若 IK 正确、
限速命令变化很慢或停在边界，检查软限位和速度限制；若命令正确而反馈关节不跟随，检查 SDK
关节顺序、方向、使能状态和机械臂硬件。
页面状态超过 `500 ms` 没有新数据会变为“数据已停止”，此时不要继续使能机械臂。

若本机 `8765` 已被占用，可在启动遥操作前于机器人 SSH 终端设置其他端口，例如：

```bash
export PICO_SKELETON_VIEWER_PORT=8876
```

同时把本机转发改为 `ssh -N -L 8876:127.0.0.1:8876 esrobo@192.168.10.100`，浏览器打开
`http://127.0.0.1:8876`。

1. 确认机器人左臂接近第 12.2 节的自然下垂零位。清空周围人员、桌面和线缆障碍，保证实体
   急停可触及，并由一人全程托稳左臂。
2. 完成第 12.4 节的 PICO 侧准备。启动脚本会再次自动检查 PC Service 和人体数据。
3. 新建独立 tmux 会话并运行最终启动命令：

```bash
tmux new -s pico_left_arm
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_pico_left_arm_teleop.sh
```

单独测试右臂时，必须由一人持续托稳右臂，再使用独立会话运行：

```bash
tmux new -s pico_right_arm
cd /home/esrobo/Projects/esrobo/teleoperation && ./scripts/run_pico_right_arm_teleop.sh
```

右臂当前可能上电即使能且默认关闭 CAN 主动上报。右臂脚本根据 USB 路径
`*-5.4:1.0` 动态建立 `can_piper2`，遥操作核心只发送主动反馈请求，取得完整七轴角度和使能位后
才执行失能；此过程不发送位置目标，但失能后机械臂会受重力下落，因此从按下启动命令前就要
持续托稳。若无法取得完整反馈，程序拒绝继续，不允许跳过反馈检查。右臂 PICO 输入固定使用
`right_shoulder/right_elbow/right_wrist`，不会读取左侧人体关节。

右臂实体 J3 与 URDF/SDK 使用同向映射，因此配置使用
`right_joint_directions: [1,1,1,1,1,1,1]`。J3 用于调整小臂屈曲平面，J4 才产生小臂相对
大臂的屈曲角。2026-09-14 的低速单轴检查中，物理 J4 从 `-0.57 deg` 到 `+1.41 deg` 后
返回原位，J3 反馈变化为 `0.00 deg`，确认 SDK 的 J3/J4 编号没有交换且 J4 执行器正常。

单臂 PICO 模式中，程序用“当前 J4 + 目标骨架夹角 - 当前机器人骨架夹角”计算 J4 屈曲量，
其中骨架夹角来自 `肩→肘` 与 `肘→腕` 两个向量。差值形式会抵消 URDF 末端固定偏置以及
保持中的 J5-J7 对腕点位置造成的影响，
并在这一帧 IK 中固定该 J4 目标；J1-J3 只负责上臂方向和肘部弯曲平面。这样 PICO 小臂相对
大臂屈曲不会再被笛卡尔 IK 分摊到 J1-J3。J4 命令仍经过逐帧位置差、速度、加速度、单步和
软限位约束，不会因为解析角度变化而绕过机械臂安全限制。

全臂模式正常停止、程序退出、启动前对齐或保护回零时，不再让七个关节同时回零。程序先固定
J1-J3 在回零开始时的反馈位置，按既有逐关节速度、加速度和单步限制使 J4-J7 回到自然下垂
零位；连续多帧确认 J4-J7 到位后，再保持其零位并使 J1-J3 回零。第二阶段不再只检查
J1-J3，而是连续复核 J1-J7 全部处于零位容差后才正常失能。短暂 CAN 反馈间隙会等待恢复；
若第一阶段未确认到位，不会继续移动 J1-J3，也不会自动失能。程序锁住跟随并保持控制器最后
目标，操作员必须继续托稳机械臂，再按 `s` 重试回零、按 `d` 明确失能或按 `x` 急停。
两个阶段分别建立反馈时间基准：终端显示 `Return phase 1/2` 后保持 J1-J3 并回 J4-J7；显示
`Return phase 2/2` 后才开始 J1-J3 的受控回零。第二阶段的第一条命令保持当时实测姿态，取得
下一帧新反馈后再按实际 `dt` 开始移动，不继承遥操作或第一阶段的旧时间戳与速度状态。
每个阶段最多进行两次完整的限速回零尝试，阶段间会重新请求 CAN 主动上报并清除速度状态；
终端每秒显示该阶段各关节相对零位的剩余角度。看到 `timed out` 或 `lost fresh joint feedback`
时，应继续托稳机械臂并根据输出判断是关节未到位还是 CAN 反馈中断；第一阶段未确认时程序
不会向 J1-J3 发送回零轨迹，也不会自动退出或失能。

左机械臂、左手套 IMU 和左灵巧手联动使用专用命令：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_left_arm_hand_teleop.sh --left-serial 00885
```

该命令等价于 `run_pico_left_arm_teleop.sh --with-left-imu --left-serial 00885`。首次联调建议放在
独立 tmux 会话中：

```bash
tmux new -s pico_left_arm_hand
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/run_pico_left_arm_hand_teleop.sh --left-serial 00885
```

SSH 断开前按 `Ctrl+b`，松开后按 `d`，可让会话在后台继续。重新进入和查看会话使用：

```bash
tmux ls
tmux attach -t pico_left_arm_hand
```

启动脚本严格按以下顺序执行：

1. 联动模式先启动或复用 SenseCom，确认左手套 `Nova 2-00885-L` 具有非零实时接收率，然后
   初始化左灵巧手 `can_hand2`、ROS 驱动和反馈桥；此时不会启动机械臂。
2. 完成 SenseGlove 六步标定。每一步按终端说明摆好动作、按 `Enter`，经过稳定缓冲后采样；
   本次组合启动强制重做标定，避免复用手套断电或 SenseCom 重启前的 IMU 参考系。
3. 启动 XRoboToolkit PC Service，确认 PICO 中已运行 XRoboToolkit、全身追踪有效，再等待
   `PICO BODY DATA READY`。PICO 未出数时不会初始化机械臂 CAN。
4. 左臂配置 `can_piper1`。上电失能的 V1.12 左臂可能没有主动上报；脚本在这一阶段不再跨进程
   执行“短暂使能后立即失能”，也不发送模式或位置命令，而是把通信唤醒延后到主程序内。
5. 要求人和机器人左臂均自然下垂，按 `Enter` 后启动 PICO 数据桥。程序只在完整的左肩、
   肘、腕数据出现后开始标定计时，终端会依次显示：5 秒动作准备倒计时、1 秒静止稳定
   缓冲、约 3 秒参考姿态采集倒计时。准备阶段和稳定缓冲阶段不记录样本；采集阶段必须继续
   保持站直、左臂自然下垂、肘部自然伸直且手腕放松。
6. 启动左侧联动遥操作核心。程序先核对左灵巧手自然张开反馈；PICO 标定完成后左臂仍保持失能。
   人工托稳机械臂并按 `e` 后，程序检查当前模式所需输入和机械臂反馈，再以 URDF
   自然下垂零位（SDK 物理角 `[0, pi/2, 0, 0, 0, 0, 0]`）作为首个低速目标，执行一次有界
   使能/CAN 反馈唤醒。该过程参考 SenseGlove 手腕遥操作已经使用的 SDK 使能方式，在最多
   8 秒内重复使能握手，并维持 normal/关节模式、`10%` 以内速度和同一个自然下垂保持目标；
   终端每秒显示握手次数和剩余时间。只有新鲜七轴角度、完整使能位和 `8 deg` 零位容差全部
   通过才开始跟随。
   验证失败时不进入遥操作，并只发送一次全关节失能，避免无反馈时连续刷写控制帧。

正常标定完成时会出现：

```text
[PICO 标定] 完成：自然下垂参考零点已锁定。机械臂仍未运动；请继续托稳机械臂，确认安全后按 e 开始遥操作。
```

如果采样期间肩、肘或手腕晃动超过阈值，程序会提示“本次姿态晃动过大”，丢弃该窗口并从
5 秒准备倒计时重新开始，不会沿用质量不足的参考零点。

连接正常时，`log/pico_left_arm_bridge.log` 或 `log/pico_right_arm_bridge.log` 应持续出现
`send_hz`、`xrt_update_hz`，`frames` 中应包含对应侧的 shoulder/elbow/wrist，且
`xrt_update_hz` 必须大于 0。另开 SSH 终端检查：

```bash
tail -f /home/esrobo/Projects/esrobo/teleoperation/log/pico_left_arm_bridge.log
# 右臂测试改为：tail -f /home/esrobo/Projects/esrobo/teleoperation/log/pico_right_arm_bridge.log
```

如果只出现 `body tracking unavailable`，说明数据桥没有取得 PICO 全身数据；保持机械臂失能，
检查 PICO 应用、追踪器校准、PC Service 和网络，不能按 `e`。

### 12.6 开始、停止和安全限制

出现 `[PICO 标定] 完成` 后，机械臂仍为失能；按 `e` 的反馈唤醒与零位验证通过后才会显示
`LEFT ARM ARMED` 并开始发送跟随目标。
人工托稳左臂，从远离身体的小幅动作开始：

| 按键 | 行为 |
| --- | --- |
| `e` | 联动模式同时检查 PICO、手套 IMU、手指目标、左灵巧手反馈和七轴反馈；全部通过后才同时开始左臂与左手跟随。 |
| `h` | 仅切换左灵巧手跟随，用于独立检查手指；不改变机械臂使能状态。 |
| `s` | 停止手指跟随；左臂先 J4-J7、后 J1-J3 回零并失能，随后左灵巧手缓慢张开。 |
| `x` | 立即发送左臂电子急停并冻结手指目标，不执行自动回零或张手。实体急停优先级更高。 |
| `q` | 正常退出；左臂分阶段回零并确认失能，随后左灵巧手缓慢张开，再关闭后台桥。 |
| `Ctrl+C` | 进入同一清理流程；正常结束优先使用 `q`。 |

该模式沿用第 1 节的首次联调软限位、逐关节速度/加速度/单步限制，以及末端最大平移速度
`0.10 m/s`。运动命令使用机械臂反馈时间戳计算实际 `dt`，时间戳不前进或反馈无效时拒绝
当前命令。PICO 数据超过 `0.5 s` 未更新时停止跟随；组合模式还监视左手套 IMU、手指目标和
左灵巧手硬件反馈。任一数据失效都会冻结手指跟随，并在机械臂反馈仍有效时限速回零、失能；
反馈已经无效时不得盲发回零位置，应立即使用急停并人工托稳。

首次联调分四段进行：先保持手腕和手指不动，仅抬左上臂 `5-10 deg`，确认 J1-J3 响应且
J5-J7 不会重复补偿整臂旋转；回到初始位后小幅弯肘，确认主要由 J4 响应；再固定肩肘，分别
小幅转动手腕三个自由度，确认主要驱动 J5-J7；最后保持整臂不动，缓慢逐指屈伸并检查左
灵巧手。任何关节方向错误、末端接近身体、出现振荡或多个关节突变时立即按 `x`，在完成方向核对
和碰撞检查前不要扩大软限位或提高速度。

### 12.7 连接排查与结束顺序

PC Service 启动失败或 PICO 找不到主机时，执行以下只读检查：

```bash
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh status
ss -lntp | grep -E ':(60061|63901)'
ip -br address
tail -n 80 log/xrobotoolkit_pc_service.log
```

- 没有 `60061`：本机 SDK 服务没有正常启动，运行 `./scripts/xrobotoolkit_pc_service.sh restart`。
- 有 `60061`、没有 `63901`：PC Service 的设备接入端未正常监听，检查日志并重启服务。
- 两个端口都存在但人体检查超时：优先检查 PICO 中填写的主机 IP、两端是否同网段、XRoboToolkit
  应用和全身追踪是否开启，以及 Motion Tracker 是否完成校准。
- `body_data_available=False`：不是机械臂故障，表示 PC Service 当前没有收到可用人体骨架帧。
- `Body.timeStampNs=0` 但 XR 时间戳持续递增且位姿变化：是当前发送端未填写专用人体时间戳，
  程序会使用 XR 时间戳兼容，不应仅因为这个字段为 0 判定断连。
- `CAN is silent in its configured power-on disabled state` 且 `candump` 没有新帧：这是当前左臂
  上电失能时停止 CAN push 的已知启动状态。保持机械臂自然下垂并完成 PICO 标定，托稳后按
  `e`，主程序会在同一 CAN 会话中用自然下垂零位执行一次有界唤醒和反馈验证。若仍提示
  `returned no complete seven-joint feedback`，检查机械臂绿灯、实体急停、CAN H/L、终端电阻和
  `can_piper1` 映射；不要反复按 `e` 或绕过反馈验证。

正常结束必须先回到运行遥操作核心的 tmux 窗口按 `q`，等待终端确认七轴回零和失能，再退出
tmux。不要在机械臂仍使能时直接执行 `tmux kill-session`。确认机械臂已经失能后，可执行：

```bash
tmux kill-session -t pico_left_arm 2>/dev/null || true
cd /home/esrobo/Projects/esrobo/teleoperation
./scripts/xrobotoolkit_pc_service.sh stop
```

PC Service 本身不控制机械臂，可以保留到下一次测试；关闭它只会结束 PICO/SDK 数据服务。
