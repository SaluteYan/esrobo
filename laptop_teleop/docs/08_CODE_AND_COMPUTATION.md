# laptop_teleop 代码结构与遥操作计算逻辑

本文依据当前仓库的 `laptop_teleop/src/esrobo_laptop/`、`teleoperation/src/esrobo_teleop/` 和 `robot_link/esrobo_link/` 实现整理。它描述**当前 PICO 输入路径**；`SenseGlove` 的安装脚本、采集入口和部分旧文档仍在仓库中，但 `FreshBodyDevice` 只接受 `pico`、`pico_hand` 两类输入。本文是代码路径分析，不代表本机环境、跨机器链路或真机运动已经验收。

## 1. 边界与目录

```text
PICO Body/Hand → XRoboToolkit PC Service → 本机 SDK
    → acquisition.py → UDP 127.0.0.1:15050 → inputs.py
    → 身体参考/骨段重定向 → 肘点与腕点目标 → 分区位置 IK
    → PICO 手腕旋转映射 + 十轴手指映射
    → app.py / RobotClient → HMAC UDP 16000 → 机器人 gateway
    → 机器人侧轨迹约束、反馈门禁和 CAN/ROS 驱动
```

| 位置 | 当前职责与关键入口 |
| --- | --- |
| `config/laptop.yaml` | 本机地址、模式、密钥路径、输入/反馈时限和控制频率。实际机器人关节与手部参数来自仓库根目录的 `teleoperation/config/teleop_config.yaml`。 |
| `src/esrobo_laptop/acquisition.py` | 动态载入已有 XRoboToolkit Body 桥，同时独立采集 PICO Hand；给 UDP 包附加来源、侧别和本机单调时钟。 |
| `src/esrobo_laptop/pico_hand.py` | 检查 26 点 Hand 数据，按手掌局部几何生成每侧十个主动关节弧度值。 |
| `src/esrobo_laptop/inputs.py` | 继承 `BodyDevice`，对 Body、Hand、手腕分别记时，拒绝旧样本，管理标定参考与新帧凭证。 |
| `src/esrobo_laptop/pipeline.py` | 管理参考初始化、人体到机器人目标、双臂 IK、手腕/手指映射和诊断发布。 |
| `src/esrobo_laptop/mapping.py` | 将十轴弧度映射到 L10 的 0–255 物理值；将 PICO 手腕相对旋转映射为 J5–J7。 |
| `src/esrobo_laptop/app.py` | `check/inspect/run` 入口、机器人状态与输入时效检查、控制周期、发包、日志和停止。 |
| `src/esrobo_laptop/contract.py` | 校验网关模式、关节顺序、单位、配置及 URDF 指纹。 |
| `src/esrobo_laptop/dashboard.py`、`web/` | 本机 HTTP 控制台，管理本机进程、SSH/tmux 机器人服务、模式、反馈、停止与失能。网页不承担 IK 控制周期。 |
| `src/esrobo_laptop/demo.py`、`tests/` | 无硬件模拟网关演示和本机回归测试。 |
| `scripts/` | 环境/SDK 安装、PICO 服务与采集、计算端、网页启动及测试包装脚本。 |
| `docs/01…07` | 环境、网络、日常操作、故障、网页和 PICO Hand 的操作文档。 |

`laptop_teleop` 是独立 Python 包，`pyproject.toml` 提供 `esrobo-laptop` 命令。`run_pico.sh`、`run_laptop.sh`、`run_dashboard.sh` 从 `common.sh` 找到 `esrobo_laptop` Conda Python，设置 `PYTHONPATH` 以复用相邻的 `teleoperation` 和 `robot_link` 源码。默认本机采集地址是 `127.0.0.1:15050`，网关是 `192.168.10.100:16000`。本机和机器人之间只传求解后的关节目标及实测状态；SSH 供网页管理机器人程序。

## 2. 输入采集：Body 与 Hand 是两路样本

`acquisition.pico()` 动态载入 `teleoperation/bridges/xrobotoolkit_body_udp_bridge.py`，包装其 `_build_packet()`。原桥生成包含肩、肘、腕、腰部等姿态的 `frames`；包装器先调用 `PicoHands.poll(sdk)` 取左右 Hand，再将两类数据分别发布：

| UDP `laptop_input.kind` | 载荷 | 样本时钟 |
| --- | --- | --- |
| `pico` | `frames` 和手追踪状态文本；左右肩、肘、腕用于手臂，腕部姿态用于带手模式 | 包装器本机 `time.monotonic()`；上游 Body 桥只应在 Body 样本前进时生成包 |
| `pico_hand` | `pico_hands[side].joints` 十个弧度、`sequence`、`age_s` | 本机采集时间减去该侧 SDK 报告的 Hand 接收年龄 |

两类包都有 `laptop_input={v:1, kind, sample_monotonic, physical_sides}`。Body 的 24 点骨架不能推导五指；Hand 来自独立的 26 点流。左右手有独立序号，某一侧丢帧不会用另一侧数据补齐。

`PicoHands.poll()` 要求该侧 SDK 提供 `get_left_hand_sample()` / `get_right_hand_sample()`，并逐项检查 `active`、正序号、`age_s ≤ 0.1`、26 个有效标志和新样本序号。它用 SDK 的原始 Hand 时间戳识别冻结帧；没有有效时间戳时用完整位姿签名判重。`retarget_hand()` 要求形状 `(26, 7)` 的有限位姿（位置 xyz + 四元数 xyzw），以腕到中指、小指到食指构造手掌局部前向/横向/法向：

- 五指屈曲：取该指相邻骨段夹角的平均值，除以 `π/2` 并截断到 `[0,1]`。
- 食指、无名指、小指侧摆：近端骨段在手掌横向与前向上的 `atan2` 角，按各自上限归一化。
- 拇指滚转与对掌：分别取拇指骨段在掌面法向上的分量，以及投影到掌面的方向和前向夹角。

生成的十个比例乘 `ACTIVE_JOINT_LIMITS_RAD`，得到 `ACTIVE_HAND_JOINTS` 顺序的十轴弧度。它是代码中的初始几何映射；关节遮挡、坐标方向与实物效果仍需逐指预览核对。上游 SDK 的扩展和数据来源见 [06_PICO_HAND_INPUT.md](06_PICO_HAND_INPUT.md)。

## 3. 输入时效与新帧门禁

`FreshBodyDevice` 在 `inputs.py` 中重写 `BodyDevice._handle_packet()`，仅接受版本 1 的 `pico` / `pico_hand`。因此旧 `acquisition.senseglove()` 入口虽然保留，发出的 `senseglove` 包不会刷新当前跟随输入。

Body 包必须包含所选侧完整肩、肘、腕姿态以及配置要求的腰部姿态，且时间戳是**本机**单调时钟、年龄在 `[0, 0.1]` 秒内。Hand 包还须为十个有限且处于关节限位内的弧度值，序号为递增正整数。`stamps` 分别记录 `left/right_arm`、`left/right_wrist`、`left/right_hand`；Body 不写手指缓存，Hand 不写身体帧。重复或倒退的时间戳、Hand 序号不会续命。`input_lock` 保证接收线程更新与主线程取帧是原子操作。

各模式必需的来源由 `FreshBodyDevice.required` 决定：

| 模式 | 每个所选侧必须有的新来源 |
| --- | --- |
| 仅机械臂 | `arm` |
| 机械臂加灵巧手 | `arm`、`wrist`、`hand` |
| 仅灵巧手 | `hand` |

`ticket(max_age)` 先检查所有必需来源年龄不超过默认 `0.10 s`，再检查身体参考（若需要），最后要求**每个来源**时间戳都大于上次 `consume()` 的值。缺少新样本时返回 `None`，不会重发缓存目标。`Pipeline.solve()` 内、以及 `Controller.step()` 计算结束后都再次检查原 ticket 的年龄；慢 IK 不能把旧样本标成新帧。网关反馈的时效另算：`feedback_cache_age_s + 本机收到状态后的计算时间 + network_margin_s`，再加每项机械臂/手反馈年龄，分别受默认 `0.15 s` 门限约束。这是保守时效预算，并非两机时钟同步测得的精确单程时延。仅灵巧手模式还要求所选侧七个机械臂使能位全部明确为 `False` 且无驱动故障。

## 4. 使能前的参考标定

`Controller.step()` 依网关状态驱动准备顺序。正式运行从网关 `IDLE` 开始：有新 PICO 样本时只回发实测机器人姿态，避免起始目标跳变。操作员显式请求准备后，网关 `RETURNING` 阶段由机器人端执行张手和回零；本机此时不采参考。看到 `CALIBRATING`，`Pipeline.restart_preparation()` 清除旧人体/手部参考、手腕缓存和上一 IK 位置解，开始新采集。

手臂参考复用 `BodyDevice._update_auto_reference()`。人体上臂相对下垂方向需抬起至少 `35°`，肘部弯曲在 `25°–125°`。默认先保持 `1.5 s`，随后有 `0.5 s` 稳定缓冲，采样阶段至 `1.5 s` 结束；不足最少样本数或各关节位置最大标准差超过 `0.05 m` 则重来。成功时保存肩/肘/腕平均位置及每侧手腕参考旋转。仅灵巧手模式省略身体参考。

带手模式还要求每侧最近 `0.6 s` 至少 12 个 Hand 样本。`natural_open_hand_reference()` 对拇指屈曲和四指屈曲取中位数，要求各轴低于配置比例门限，且这些轴的最大标准差不超过 `0.04 rad`。该中位数就是本次会话的自然张手零点，不额外保存手部标定文件。

`Pipeline.initialize(measured)` 将机器人当前实测 14 轴经 FK 转为肩/肘/腕基准，调用 `rebase_robot_reference()` 与 `IkSolver.initialize_arm_session()`；带手模式还构建每侧 `WristMapping`。网关 `CALIBRATING` 时只有准备姿态通过才求解并发送首目标；网关校验目标与零位匹配后转入 `ARMING`/`ACTIVE`。`ARMING` 期间本机发实测保持目标。进入 `ACTIVE` 时重新以实测值初始化一次，随后开始正式跟随。双臂自动回零受机器人端实现限制，不能把本机参考完成理解为双臂自动准备已可用于真机。

### 标定完成后，机械臂怎样开始运动

这里的“初始姿态”是机器人端回零并验证失能后，**当前反馈测得的七轴姿态** `q₀`；它不是一个仅由人体标定结果推算的电机角。人体参考主要用于上臂方向对齐；带手模式还用标定时的 PICO 手腕旋转计算后续相对旋转。`rebase_robot_reference()` 将 `FK(q₀)` 的肩、肘、腕设为机器人会话起点，并重置上一肘/腕目标位置和更新时钟。当前肘弯曲采用 `direct_absolute`，所以“人体保持标定姿态”并不在数学上保证求解出的七轴严格等于 `q₀`；首帧仍须经过下述位置增量限制和机器人端匹配门禁。

1. **首帧求解但尚不驱动。** `CALIBRATING` 时，`Pipeline.capture()` 将 PICO 骨段方向变成肘/腕位置目标。`BodyDevice._limit_arm_segment_translations()` 从刚设定的机器人 FK 起点推进，并限制本帧肘、腕位移不超过 `0.22 m/s × 实际 dt`。分区 IK 在固定 J5–J7 的同时求 J1–J4，输出七轴 `q_first`。本机检查源样本与反馈没有在求解期间过期，才通过 `RobotClient` 发给网关。此时网关只缓存该目标，机械臂仍保持零位且失能。
2. **使能前再次比对起点。** 网关在 `CALIBRATING` 收到有效目标后进入 `ARMING`，调用 `HardwareBackend.preflight_enable()` 重新读取机械臂反馈，要求七轴使能状态全为 `False`、控制器无故障，且每轴 `|q_first − q_measured| ≤ 1.5°`。带手模式还要求自然张手反馈已经标定、实际手在开位，首手目标与实测每轴相差不超过配置的 12 个 L10 单位。若首帧因人体动作、IK 或时延不符合条件，直接拒绝使能；不会把远处目标当作启动命令。每个目标还要通过合同、范围、会话、序号与约 200 ms lease 检查。
3. **使能过程保持当前关节角。** `capture_session_start()` 记录实测起点。`NeroSingleArmDriver.enable()` 检查启动几何与控制器状态，设置运动模式，先下发**实测姿态本身**作为预载；使能并核对七轴使能反馈后，再下发同一保持姿态，并在静止姿态采集力矩基线。之后设置当前配置速度百分比（30%），`initialize_command_trajectory()` 用新鲜实测位置和反馈时间戳重置轨迹状态：`position=q_measured`、`velocity=0`，并设置 `first_follow_hold=True`。网关进入 `ACTIVE` 之前，笔记本在 `ARMING` 持续发送实测保持目标，而非人体实时求解目标。
4. **进入 ACTIVE 的首个驱动周期仍保持起点。** 网关切换 `ACTIVE` 后只在执行线程的 `tick()` 中调用后端 `step()`，甚至可能在完成使能的同一次 `tick()` 内立即调用。该周期可能先看到尚未更新的 `ARMING` 保持目标；即便已经有更新目标，驱动的 `first_follow_hold` 也把本周期物理目标替换为轨迹初始化位置。它在成功发送并提交这次保持指令后才清除标志。笔记本首次收到 `ACTIVE` 状态时，`Controller.step()` 以当时新鲜实测反馈再次调用 `Pipeline.initialize()`，重设机器人 FK 起点和腕映射，然后开始发送新的人体 IK 目标。这次重设**不清除 PICO 已采集的人体参考**。首轮保持是驱动本地的一次命令门禁，**并非等待笔记本完成重设的握手**。
5. **后续周期才逐步追踪。** 网关的网络线程只保留最新有效目标；约每 20 ms 的执行周期读取新鲜反馈，把 URDF 角换成物理角 `q_physical=direction·q_urdf+offset`。`CommandTrajectory.propose()` 从上一条**已提交命令** `q_cmd`、上一速度 `v_cmd` 和本次反馈时间戳差 `dt` 生成候选 `q_next`，同时满足关节位置、速度、加速度、单步增量、命令相对反馈超前量与 FK 肘/腕端点速度限制；若 `dt=0` 跳过，若倒退或超过 `0.1 s` 拒绝。本周期候选经过检查后才经当前配置的连续 `MOVE_J` 路径发给 NERO SDK，发送成功才更新轨迹内部状态。若目标在途中变化，下一周期朝**最新**目标计算，不排队执行原目标的完整路径。

第 3 步中的几个“保持”各有作用。`capture_session_start()` 只读取七轴反馈、记录会话起点并将内部命令速度置零，**不下发运动命令**。`enable()` 又读取一次实测角 `q_enable`，以它作为使能前预载目标；失能状态下控制器可能忽略该帧，所以 SDK 报告使能成功后仍需核对七轴使能反馈，并重新发送 `q_enable` 保持目标。静止保持约 `1 s` 后采集至少 15 个力矩反馈取中位数作为后续异常监测基线；这不是给人体标定手臂，也不是让机器人前往人体目标。随后 `initialize_command_trajectory()` 再以当时**新鲜实测**关节角和反馈时间戳建立 `position=q_init、velocity=0`，因此轨迹器起点不依赖更早的预载角恰好等于当前反馈。30% 是下发给控制器的速度百分比配置，实际跟随还受软件关节速度/加速度等限制。

笔记本在 `ARMING` 中仍读取 PICO 帧，但 `Controller.step()` 不对它求人体 IK，而是把网关状态包里的机械臂/手实测值复制成保持目标。这个状态包在耗时的使能/力矩采样期间可能是缓存值；本机对 `ARMING` 放宽了反馈年龄判断。网关仅在切到 `ACTIVE` 后才执行 `step()`，且驱动的 `first_follow_hold` 会把**第一个成功提交的跟随命令**强制设为轨迹初始化位置 `q_init`，不把可能缓存的 `ARMING` 保持目标当成这次命令的运动方向。该标志在发送/提交成功后才清除；下一周期开始按网关中**最新有效目标**限速推进。如果笔记本尚未收到 `ACTIVE` 并发送新 IK 目标，该目标仍可能是 `ARMING` 的保持角；现有协议没有“已收到新人体 IK 目标”才放行第二周期的握手。

因此过渡是两层连续限制：本机先限制**由 PICO 姿态换算出的机器人坐标系肘/腕位置目标**相对上一目标的位移，机器人端再把七轴 IK 目标变成基于实测反馈的加速度/速度受限小步。前一层 `0.22 m/s` 的确切含义见下段；它没有“标定结束立即从 `q₀` 插值到一个固定终点”的预设总时长。以反馈间隔恰为 `20 ms`、某关节上次命令速度为零且加速度上限 `40°/s²` 为例，仅加速度约束就把下一步速度增量限制在 `0.8°/s`，对应位置增量至多约 `0.016°`；实际候选还可能被其他约束进一步缩小或保持。50 Hz 是目标周期，SDK/CAN 实际完成频率应看反馈和日志，不应从配置值推断。

具体地，`advance()` 用本机单调时钟计算两次 `advance()` 调用的实际间隔 `dt`，设 `D=0.22 m/s × dt`。对选中侧，限速器要求输出的肘点 `E_next`、腕点 `W_next` **分别**满足 `‖E_next−E_prev‖≤D`、`‖W_next−W_prev‖≤D`；`E_prev、W_prev` 是上一轮生成的目标，刚初始化时来自机器人实测 `q₀` 的 FK，**不是本轮机器人关节反馈对应的位置**。例如 `dt=20 ms` 时，每点最多变化 `4.4 mm`，`dt=50 ms` 时为 `11 mm`。这里限制的是每次计算出的三维位置目标位移，不是 PICO 原始坐标、人体实际手速、关节角速度或机械臂实际已走距离。

若完整人体目标满足两个位移界，直接采用；否则算法沿双骨段的连续几何状态前进：球面插值上臂方向与弯曲平面，线性插值肘弯曲角，再用机器人上臂/前臂长度重建肘腕点。它先依据旋转弧长估计进度；如果重建后任一点仍超出 `D`，就反复减半进度，必要时保持上轮位置。这样不会分别裁剪肘点、腕点而破坏骨长。该函数只改位姿的**位置前三维**，不对手腕四元数施加 `0.22 m/s`；手腕旋转和机器人关节运动各走其独立限制。这是离散目标增量约束，不能单独保证连续轨迹避碰。[实现](../../teleoperation/src/esrobo_teleop/device/body_device.py)

**为何方向跟踪还要限制位置目标？** 对方向映射本身，人体与机器人臂长不同**不是必须限速的理由**：程序本来就把人体大臂、小臂方向归一化，再乘机器人骨长，直接得到完整姿态目标。机器人骨长只决定该方向变化会在机器人肘、腕处产生多大位移：固定肩点、上臂长度为 `L` 时，大臂转过 `Δθ` 使肘点移动 `2L·sin(Δθ/2)`。`0.22 m/s` 是对这个*机器人目标位置变化速度*施加的响应上限，并不改变最终要跟踪的方向。

这层上限用于**从机器人当前姿态逐步进入映射姿态**，并抑制快速动作、追踪噪声或重获追踪造成的相邻目标跳变。当前 `direct_absolute` 肘弯曲还可能让人体静止时的完整目标与机器人零位不同；若立即把远处目标送给局部 IK，可能出现不可达/不连续解，使能前也可能被 `1.5°` 起点门槛拒绝。机器人端关节轨迹器会进一步限速，所以从纯运动限速角度看，本机 `0.22 m/s` 不是唯一可能的实现；去掉它仍需重新设计启动目标匹配与求解连续性，并验证机器人端响应。代价是快动作时机器人方向跟随存在滞后。该约束既不是碰撞检测，也不保证实际机械臂速度恰为 `0.22 m/s`。

这个自动准备与过渡流程适用于当前真实**单侧机械臂**网关。真实 `DualBackend` 的 `supports_prepare=False`：由于缺少双臂互碰扫掠检查，机器人端会在 `e` 时拒绝双臂自动回零/准备；模拟双臂演示只能证明软件状态流转。当前配置 `teleop_torso_collision_enabled=false`，实时跟随阶段不做逐周期躯干几何检查，启动检查和受检回零仍保留相应保护。相关实现依次见 [本机状态切换](../src/esrobo_laptop/app.py)、[人体位置限速](../../teleoperation/src/esrobo_teleop/device/body_device.py)、[网关使能](../../robot_link/esrobo_link/gateway.py)、[硬件预检查](../../robot_link/esrobo_link/backends.py)、[驱动首帧保持](../../teleoperation/src/esrobo_teleop/robot/nero_driver.py) 与 [轨迹器](../../teleoperation/src/esrobo_teleop/robot/command_trajectory.py)。

### 人体标定值究竟参与了什么计算

把标定时 PICO 的肩、肘、腕平均点记为 `Sₕ*、Eₕ*、Wₕ*`，标定手腕旋转记为 `Rₕ*`；把机器人启动反馈 `q₀` 的 FK 点记为 `Sᵣ₀、Eᵣ₀、Wᵣ₀`。两组参考**分别保存**：前者是人体传感器的坐标方向基准，后者是该次机器人会话的几何起点。标定通过抬臂、自然屈肘和稳定性筛选，让参考方向可观测；它既不记录“机械臂应到达的七轴角”，也不把人体肩、肘、腕的绝对坐标直接传给电机。当前使用腰部相对 PICO 坐标，并按配置做坐标轴符号变换。

每次 `Pipeline.initialize(measured)` 都会用最新机器人实测角重建 `Sᵣ₀、Eᵣ₀、Wᵣ₀`，初始化 IK 会话。`CALIBRATING` 首次初始化一次；首次看到 `ACTIVE` 再初始化一次，以使能后实测姿态为新的机器人起点。第二次初始化重置机器人参考、目标位置滤波状态、IK 会话和腕映射，**不会重新采集或清除已锁定的人体参考**。因此之后的方向对齐仍以同一组 `Sₕ*、Eₕ*、Wₕ*` 为准。

对任意新人体帧 `Sₕ(t)、Eₕ(t)、Wₕ(t)`，`BodyDevice._retarget_arm_segment_direction_positions()` 大致做以下计算。这里的 PICO 点已换到腰部相对坐标并应用轴符号变换，公式省略预测、滤波、死区和近直臂平面稳定的中间变量：

```text
uₕ* = Eₕ* − Sₕ*                 Uᵣ₀ = Eᵣ₀ − Sᵣ₀
R_align = minimal_rotation(uₕ*, Uᵣ₀)
u = R_align · (Eₕ(t) − Sₕ(t))
f = R_align · (Wₕ(t) − Eₕ(t))
û, f̂ = 经肘弯曲映射、方向滤波、死区和弯曲平面稳定后的单位方向
E_goal = Sᵣ₀ + |Uᵣ₀| · û
W_goal = E_goal + |Wᵣ₀ − Eᵣ₀| · f̂
```

这意味着人体上臂相对标定方向的变化驱动机器人上臂转向，人体前臂方向和实时肘弯曲决定腕点；机器人肩点固定在会话参考，目标骨长取机器人 FK 骨长，而非人体臂长。当前 `elbow_angle_mapping=direct_absolute`：目标几何肘弯曲取**当前人体上臂与前臂夹角**，不做 `当前人体弯曲 − 标定人体弯曲 + 机器人初始弯曲`。标定弯曲仍用于参考有效性与会话诊断，机器人初始前臂方向则在近乎伸直、人体弯曲平面不稳定时提供基准。由此，即使人一直保持采集姿态，人体肘角与机器人零位肘角不同时也会有向新姿态靠拢的目标，不能假定标定姿态对应严格静止的 `q₀`。

这里的“方向跟踪”还要区分**相对标定的方向变化**与**绝对空间方向相同**。当前配置是 `segment_direction_relative`：`R_align` 把标定人体上臂方向对齐到机器人 `q₀` 的上臂方向。若人体上臂一直维持标定方向，映射后的机器人上臂目标就是它的起始方向，即使两者在各自原始坐标中的绝对朝向不同。人体上臂后续转动才作为相对变化传给机器人。前臂在当前 `direct_absolute` 下尽量保持人体实时肘弯曲角和可观测弯曲平面；滤波、限速、关节限位和 IK 残差可能使瞬时实际姿态与目标方向有差距。因此当前实现不能解释为“标定结束即令机器人两段骨骼在空间中与人体两段骨骼绝对平行”。

例如标定时人体肘弯曲为 `60°`、机器人回零后 FK 几何弯曲为 `30°`：如果人不动，`direct_absolute` 仍请求接近 `60°` 的目标几何弯曲。机器人不会在标定完成的一刻跳到这个角度：首帧位置目标从 `FK(q₀)` 限速推进，使能前七轴还需与反馈相差不超过 `1.5°`，进入 `ACTIVE` 后才由关节轨迹器逐周期追踪。`60°` 是骨段夹角，并非可直接下发的某个电机 J4 角；实际七轴目标还取决于 IK、限位和末端三轴。

`_predicted_arm_points()` 可按近期 PICO 样本预测肩/肘/腕位置；方向再经过速度相关滤波和角度死区。肘接近伸直时，`cross(û,f̂)` 的方向会受噪声显著影响，因此在约 `4°` 以下沿用机器人参考弯曲平面，约 `10°` 以上使用人体可观测弯曲平面，中间平滑过渡。随后 `_limit_arm_segment_translations()` **从上一次已生成的肘/腕目标**推进，初次由机器人 FK 起点开始，并保持两段骨长、将肘和腕的单帧位移分别限制在 `0.22 m/s × dt` 内。这个限速让目标从 `q₀` 附近逐步展开；它并不判断肘或手掌到障碍物的距离。

#### 预测的作用与双臂场景取舍

`_predicted_arm_points()` 不是预测机器人关节或碰撞，也不在 PICO 断流期间持续推进。它从最近一次肩/肘/腕点和历史中时间差在 `8–100 ms`、最接近 `50 ms` 的一帧分别估算点速度 `v=(p_now−p_old)/Δt`，形成 `p_pred=p_now+gain(|v|)·v·0.03 s`；每个点的额外位移最多 `0.03 m`，速度估计上限取配置默认 `2.5 m/s`。`gain` 在 `0.06 m/s` 以下为零、`0.25 m/s` 以上为一，中间平滑增加。以稳定匀速 `0.25 m/s` 为例，理论额外前推 `7.5 mm`。历史时间戳记录的是**笔记本收到身体帧的本机单调时钟**，不是 PICO 原始曝光时刻；适配器另检查源样本时间的新鲜度。预测后才计算骨段方向、滤波并经过 `0.22 m/s` 位置目标限速与机器人关节轨迹限制，所以它只尝试减少传感/滤波链路的跟随滞后，不能突破实际执行约束。

预测并非方向遥操作必需：`arm_vector_prediction_horizon_s=0` 时直接用最新人体点，方向映射和 IK 仍可工作。预测对两侧肩/肘/腕点分别外推，没有双臂互碰约束，也没有保持预测人体骨段长度的联合模型；噪声、时间戳抖动、突然停手或反向时，前推方向可能短暂过冲，随后由滤波和限速纠正。双臂相互靠近时，即使几毫米的提前目标也应按**空间余量**评估，而不能把 `0.03 m` 上限当成安全距离。当前真实双臂网关还因缺少双臂互碰扫掠检查而拒绝自动准备，实时躯干几何检查也配置为关闭；现有代码不能证明该预测对双臂真机既有收益又安全。

因此双臂真机设计建议先以**无预测**作为基线：先用相同录制输入离线比较方向目标滞后、停手/反向过冲、IK 拒绝率和限速器进度，并用双臂模型检查最小间距；在双臂启动和互碰保护完善后，再测实机的输入到反馈延迟。只有确有延迟收益且空间余量足够时，才逐步试 `0.01–0.03 s`。这是工程建议，不是当前硬件验证结论；调整 `arm_vector_prediction_horizon_s` 会影响共用该配置的单臂路径，现有配置并非双臂专用。[预测实现](../../teleoperation/src/esrobo_teleop/device/body_device.py)与[当前参数](../../teleoperation/config/teleop_config.yaml)给出了可比较的两个设置入口。

带手模式另计算 `ΔR=Rₕ(t)·Rₕ*ᵀ`，把相对手腕转动叠加到机器人初始手腕姿态，限制相对总转角 `45°`、单次更新 `2°`；`WristMapping` 把姿态换成 J5–J7 目标。纯机械臂模式把 J5–J7 保持在当前机器人实测反馈值，标定的 PICO 手腕旋转**不用于驱动**这三轴。两种模式都由分区 IK 在 J5–J7 固定的前提下，用有界最小二乘拟合 `E_goal、W_goal`，求 J1–J4；位置残差过大、非有限或越界的目标会被拒绝。IK 优化目标只有肘/腕位置误差与关节限位，**没有碰撞距离项**。

### 从零位跟踪的碰撞保护边界

| 阶段 | 当前代码实际检查 | 边界 |
| --- | --- | --- |
| 机器人回零 `RETURNING` | `safe_return()` 检查当前姿态和零位，`ReturnPlanner` 尝试受检直线、细分或有界 RRT 路径；执行每个路点前再检查实测姿态到路点的关节运动区间 | 只规划当前单臂到零位，不规划标定后跟随人体的整条轨迹。 |
| 标定与 `ARMING` | 标定时机器人保持失能；首 IK 目标须与实测每轴相差不超过 `1.5°`。驱动使能前对**当前姿态 `q,q`**做躯干几何检查，再预载/发送实测保持角、核验使能与控制器保护配置 | `q,q` 只证明起始姿态满足几何检查，不能证明未来人体目标或运动路径安全。 |
| `ACTIVE` 实时跟随 | 笔记本限制笛卡尔目标变化；网关限制关节位置、速度、加速度、单步增量、指令领先反馈量及 FK 肘/腕速度，并检查反馈/控制器/力矩状态 | 当前 `teleop_torso_collision_enabled=false`，正常跟随和短暂反馈中断后的恢复均不调用逐步躯干路径几何检查；限速不是避障。 |

`TorsoCollisionGuard` 使用 URDF 碰撞网格：躯干等非手臂连杆转为凸包，运动侧手臂网格由外包盒保守近似；带手时还为手指构造包络。默认普通间隙 `0.03 m`，已安装肩部连接的特殊配对间隙 `0.005 m`。它对起止关节角围成的**关节区间盒**做证明，以覆盖 `MOVE_J` 各轴可能不同步的情况；最多检查 64 个细分盒，不能证明时拒绝。几何模型假定腰、头、底座等非手臂关节保持 URDF 零位，仅检查当前运动侧手臂/手与非手臂躯干连杆，**不覆盖另一机械臂、手臂自身连杆、人体或外部障碍物**。当前真机双臂自动准备因缺少双臂互碰扫掠检查而在网关入口拒绝。

配置中 `torso_collision_enabled=true` 让启动与回零具备上述几何检查；`teleop_torso_collision_enabled=false` 则明确关闭实时跟随的该项逐周期检查。如果为单臂实时跟随开启此开关，`CommandTrajectory.propose()` 才会检查反馈到上一命令的未完成运动，以及覆盖反馈、上一命令、下一候选和估算制动点的关节区间盒；候选不合格时尝试更小步或制动，仍找不到安全候选就报轨迹故障，**不会绕开障碍物规划另一条路**。这项能力不能替代对实际机型、安装几何和运行时延的验证。

控制器碰撞保护等级 `[4,4,4,4,5,5,5]` 的配置/读回、静止时采集的力矩基线偏差监测、状态和新鲜度看门狗仍起作用，但它们属于控制器/反馈异常检测，不能当作人体、双臂或障碍物的几何避碰保证。要判断一次真实动作是否安全，需分别看标定完成、回零日志、使能前几何结果、实时 `trajectory_diagnostics.torso_collision_check_active` 和实际机械臂反馈；当前配置下该诊断应为 `false`。实现见 [标定/映射](../../teleoperation/src/esrobo_teleop/device/body_device.py)、[分区 IK](../../teleoperation/src/esrobo_teleop/ik/solver.py)、[碰撞守卫](../../teleoperation/src/esrobo_teleop/robot/torso_collision.py)、[轨迹驱动](../../teleoperation/src/esrobo_teleop/robot/nero_driver.py) 与 [当前配置](../../teleoperation/config/teleop_config.yaml)。

## 5. 身体骨段到机器人末端位置

`Pipeline.capture()` 调用 `FreshBodyDevice.advance()`，它复用 `BodyDevice._retarget_arm_vector_pair()`，为左右侧产出 `left/right_elbow`、`left/right_wrist` 各一个 7 元目标位姿：前三项为位置、后四项为 wxyz 四元数。单侧模式另一侧保持上次模型位姿。当前 `arm_vector_position_mode=segment_direction_relative`：

1. 由 PICO 肩 `S`、肘 `E`、腕 `W` 算人体上臂 `u=E−S` 和前臂 `f=W−E`。`_predicted_arm_points()` 可在设定窗口内依据近期帧预测；配置预测时域 `0.03 s`、最大位移 `0.03 m`，低速时减弱预测。
2. 将标定姿态的人体上臂方向做最小旋转对齐到机器人的参考上臂方向，并以同一旋转处理当前 `u`、`f`。前臂再经 `_neutral_relative_forearm()` 使用当前人体肘弯曲与参考关系；当前 `elbow_angle_mapping=direct_absolute`，直接保留可观测的实时弯曲，而非简单减去标定弯曲。
3. 两骨段方向经速度相关滤波、角度死区与近直臂弯曲平面稳定处理。配置在约 `4°` 以下视平面不易观测，至 `10°` 完全放开。将单位方向乘机器人标定上臂/前臂长度：`E_target=S_robot+L_upper·û`，`W_target=E_target+L_forearm·f̂`。
4. `_limit_arm_segment_translations()` 对连续肘/腕位置限制，配置末端平移速度上限 `0.22 m/s`；保持骨段几何关系，避免分别裁剪位置造成骨长失真。手腕姿态则走下一节的独立映射。

人体臂长不会直接作为机器人末端距离；关键是对齐、滤波后的骨段方向和机器人自身的骨长。腰部、头部及底盘都不由此流程联动。

## 6. 分区 IK：J1–J4 解位置，J5–J7 管手腕

`Pipeline.__init__()` 为每个所选侧构造一个 `IkSolver`，先加载 Pinocchio/URDF，并把网关合同给出的关节上下限作为位置包络。实际调用 `IkSolver.solve(..., partitioned_side=side, terminal_joint_targets=...)`；当前配置同时打开 `partition_terminal_wrist_ik` 与 `enable_elbow_tasks`，因此进入 `_solve_partitioned_positions()` 的 **SciPy 有界最小二乘路径**。`solver.py` 中另有 Pink 速度 IK 路径，但不是这里的常规分区求解。

每侧输入为实测完整 `q∈R¹⁴`、目标肘点/腕点各 3 维，以及 J5–J7 的三个末端目标。未选侧七轴使用实测值；所选侧 J5–J7 在求解中固定。J1–J4 共同作为位置变量，近直臂时 J3 不可稳定观察，于是仅优化 J1、J2、J4 并保持 J3。求解目标可写为：

```text
min_q  ||w_e · (FK_elbow(q) − E_target)||²
     + ||w_w · (FK_hand_base(q) − W_target)||²
subject to 关节合同位置上下限；J5–J7 固定为末端目标
```

实现用 Pinocchio 前向运动学计算两个 frame 位置，并用其平移雅可比构造 `least_squares()` 的解析 Jacobian；每次有界求解最多 24 次函数评估。若落入不正确的肘部分支且位置误差超过 `0.03 m`，再尝试正负弯曲分支种子。若人体请求的弯曲超过当前机器人 J4 上限可达的几何弯曲，先沿原弯曲平面调整笛卡尔腕点，再求解；诊断记录请求和实际目标差。最终肘/腕任一位置误差仍超过 `0.03 m`，该解判无效。

`Pipeline.solve()` 对无效、非有限或不满足限位的解拒绝发包。仅**单侧纯机械臂 ACTIVE** 模式，在关节已实测到同一位置边界且判为可恢复的条件下，能发 `hold` 请求让网关刹车/保持；目标回到较小误差范围并连续满足配置帧数后恢复。带手或双侧模式不通过这个单侧 `hold` 特例放行。IK 得到的完整几何目标仍要经过机器人网关自己的轨迹、反馈和硬件检查；本机计算的 50 Hz 不是硬实时执行保证。

## 7. PICO 手腕姿态与十轴手指映射

### 手腕 J5–J7

`FreshBodyDevice._compose_wrist_rotation()` 在机器人/腰部坐标系内比较当前 PICO 手腕旋转 `R_now` 与标定参考 `R_ref`：`ΔR=R_now·R_refᵀ`。旋转向量先限制总角度 `45°`，再相对上一帧限制每步 `2°`，叠加到初始机器人腕姿态。纯机械臂模式返回初始姿态，`Pipeline.solve()` 直接保持 J5–J7 的实测反馈值。

带手模式的 `WristMapping.compensated()` 用上一有效位置解生成“J5–J7 置中”时的手腕 FK 基准，再求目标姿态相对于该基准的旋转。`imu_local_rotation_to_wrist_offsets()` 在当前代码里复用于 PICO 相对旋转到关节偏移；再按机械臂关节方向、零点偏置换算成 J5–J7 URDF 目标，并先裁剪到合同位置包络。这样位置 IK 可在末端三轴已确定的几何下拟合肘/腕点，而不需每周期再做第二次 IK。

### 手指十轴

`hand_units(active_rad, feedback, cfg, side, reference_rad)` 的输入为一侧十个主动关节弧度、十个物理轴实测值和自然张手参考。其核心计算为：

```text
fraction_active = clip((PICO_angle − open_reference) / (joint_upper − open_reference), 0, 1)
fraction_physical = fraction_active[[2,1,4,5,7,9,3,6,8,0]]
target = round(clip(open_endpoint + fraction_physical · (teleop_closed−open_endpoint), 0, 255))
```

置换后的 L10 物理顺序依次是：拇指屈曲、拇指侧摆、食指屈曲、中指屈曲、无名指屈曲、小指屈曲、食指侧摆、无名指侧摆、小指侧摆、拇指滚转。当前启用的物理索引是 `[0,1,2,3,4,5,9]`；未启用的 6–8 保持本帧机器人反馈。右手在已取得自然张手参考时，对四指屈曲与拇指屈曲比例乘 `1.25`，拇指侧摆乘 `0.60`，再裁剪到 `[0,1]`。左右侧开位、闭位端点来自 `teleop_config.yaml`，右手优先使用 `right_teleop_closed`；网关还会做物理轴步长、速度和反馈门禁。L20Lite 模型中其余关节由 URDF mimic 表达，并非额外十个独立命令。

## 8. 网关合同、控制循环与故障退出

`app.main()` 读取本机 YAML 与机器人配置，确认 URDF 存在、IK 配置匹配且手模型是 L10。`check` 只报告本地模型、所需来源和 SDK 可用性；`inspect` 连接网关、验证合同并读取状态；`run` 才创建 `Pipeline` 与循环。`contract.local_id()` 对机器人参数、手部参数、去掉路径字段的 IK 参数、末端速度参数及 URDF 文件 SHA-256 做规范化哈希；`verify_contract()` 再比对侧别、带手/仅手模式、关节顺序与单位。连接后若合同在会话中变化，`Controller.step()` 终止。

每周期最多等待网关新状态 `0.08 s`；新状态必须通过 HMAC、会话和递增序号检查。`Controller.step()` 验证反馈、拿输入 ticket、捕获帧、求解，并在**求解后**再次检查输入与反馈；正式模式按模式调用 `send_target()`、`send_dual_target()`、`send_hand_target()` 或单侧纯臂的 `send_hold()`。双侧目标在一个协议包内含左右臂和双手，不能只更新一侧；仅灵巧手目标包没有机械臂字段。发包前 `RobotClient` 还要求最新机器人状态仍在 lease 内。协议使用共享密钥 HMAC 提供认证和完整性，不加密数据。

默认周期 `1/50 s`。`ControlRateMonitor` 在 `ARMING`/`ACTIVE` 检查目标间隔 ≤ `0.08 s`，并检查最近 2 秒的输出频率：配置最低 40 Hz，允许 5 Hz 容差，因此持续低于 35 Hz 会停止。输入缺失、反馈过期、IK 失败、网络超时、合同变化或网关故障，均不会靠重复旧目标继续跟随；活跃会话会退出，本机 `finally` 尽力发送认证 STOP，网关另有约 200 ms 的目标 lease/看门狗。`--preview` 会连接和验证网关、计算目标但不发送运动命令；网页和日志并不参与 50 Hz 计算。

`run` 每约 `0.2 s` 写一条 JSONL 状态，包含来源年龄/频率、PICO 手输入与参考、目标、反馈、计算耗时和周期丢失数；可用 `--status-file` 原子写给网页。`Pipeline.publish_diagnostics()` 最多约 10 Hz 向 `127.0.0.1:15060` 发骨架/IK/反馈可视化，发送失败不打断控制。

## 9. 网页、脚本和验证入口

`dashboard.py` 在本机启动 HTTP 控制台（默认页面见 `web/`），用 `Process` 管本机 PC Service、PICO 采集和计算进程；用 Paramiko SSH 与机器人 `tmux` 管网关、手驱动、腰部、头部和相机。启动计算前，它检查网关状态新鲜且为 `IDLE`，以及网关模式与页面所选模式一致。网页停止先终止本机目标生成，再通过 SSH 请求网关停止；失能操作也先停止跟随。浏览器心跳、状态文件和机器人状态轮询是操作界面保护层，不能替代网关本身的反馈与 lease 门禁。

`scripts/install_env.sh` 和 `scripts/install_pico.sh` 分别准备 Conda 环境、PC Service/SDK；`check_environment.sh` 检查环境；`run_pico.sh` 发布当前 PICO 数据；`run_laptop.sh` 运行 `check/inspect/run`；`run_dashboard.sh` 启动网页。SenseGlove 相关安装/启动脚本作为旧输入路径保留，当前 `inputs.py` 会忽略其 UDP 包。`demo.py` 使用模拟网关和合成传感器，只用于离线验证；`scripts/test.sh` 执行本机、通信和选定 IK 回归测试。

推荐阅读顺序：[本机配置](../config/laptop.yaml) → [采集适配](../src/esrobo_laptop/acquisition.py)/[手部几何](../src/esrobo_laptop/pico_hand.py) → [输入门禁](../src/esrobo_laptop/inputs.py) → [计算流水线](../src/esrobo_laptop/pipeline.py)/[关节映射](../src/esrobo_laptop/mapping.py) → [主循环](../src/esrobo_laptop/app.py) → [人体重定向](../../teleoperation/src/esrobo_teleop/device/body_device.py) 与 [IK](../../teleoperation/src/esrobo_teleop/ik/solver.py) → [通信客户端](../../robot_link/esrobo_link/client.py)、[机器人网关](../../robot_link/esrobo_link/gateway.py)。日常启动和真机准备步骤以 [03_OPERATION_RUNBOOK.md](03_OPERATION_RUNBOOK.md) 与 [05_WEB_CONSOLE.md](05_WEB_CONSOLE.md) 为准。
