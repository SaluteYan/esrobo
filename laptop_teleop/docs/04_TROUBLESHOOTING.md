# 检查与故障排查

当前使用 PICO Body + Hand 输入。缺少 `*_arm` / `*_wrist` 时检查全身追踪，缺少 `*_hand` 时检查头显 Hand Tracking、双手可见性及新版 SDK。SenseCom/手套话题不再参与本机遥操作，以下手套排障仅供旧设备维护。具体命令见 [文档 06](06_PICO_HAND_INPUT.md)。

## 一次性状态检查

```bash
cd /home/ddc/DualArmTeleoperation
./laptop_teleop/scripts/check_environment.sh
./laptop_teleop/scripts/run_laptop.sh check
./laptop_teleop/scripts/xrobotoolkit_service.sh status
ping -c 3 192.168.10.100
```

## Conda 或 Python 包错误

确认使用项目环境且禁用用户 site-packages：

```bash
source /home/ddc/miniforge3/etc/profile.d/conda.sh
conda activate esrobo_laptop
PYTHONNOUSERSITE=1 python -c 'import numpy, scipy, yaml, pinocchio; print(pinocchio.__version__)'
```

缺包或环境被修改时重新同步：

```bash
./laptop_teleop/scripts/install_env.sh
```

`run_laptop.sh`、`run_pico.sh` 和 `test.sh` 会直接选择 `esrobo_laptop`，不会使用旧 `.venv`。

## PICO 服务或 SDK

```bash
./laptop_teleop/scripts/xrobotoolkit_service.sh restart
ss -lnt | grep 60061
tail -n 100 laptop_teleop/log/xrobotoolkit_pc_service.log
./laptop_teleop/scripts/run_pico.sh
```

如果 `run_laptop.sh check` 显示 `xrobotoolkit_sdk: false`，重新运行：

```bash
./laptop_teleop/scripts/install_pico.sh
```

若服务正常但无人体帧，检查头显和本机是否同网、头显配置的是否为本机局域网 IP、全身追踪应用是否运行。`192.168.10.100` 是机器人地址，不能当成本机 PICO Service 地址。

## SenseGlove 无话题

```bash
source /opt/ros/humble/setup.bash
source laptop_teleop/external/senseglove_ros_ws/install/setup.bash
ros2 pkg prefix senseglove_msgs
ros2 pkg prefix senseglove_bringup
ros2 topic list | grep senseglove
```

先区分系统蓝牙连接与 SenseCom SDK 连接：

```bash
ros2 run senseglove_api sg_tester
```

如果系统蓝牙能看到 Nova 2，但这里显示 `No SenseGloves detected`，关闭 GNOME 蓝牙设置页及其它扫描程序，重启 SenseCom，然后从 SenseCom 左上角菜单进入 **Pair Devices**，在 **Nearby Devices** 中配对两只手套。`Player.log` 中出现 `org.bluez.Error.Busy` 表示其它程序占用了蓝牙扫描；系统设置中的“已连接”本身不能代替 SenseCom 内部配对。

如果 SenseGlove ROS 日志出现 `std::invalid_argument`、`what(): stof`，随后只剩 `robot_state_publisher` 节点，表示 SGCore 在解析损坏或不完整的 Nova 2 BLE 帧时抛出了异常。当前工作空间的硬件适配层会丢弃该帧并继续运行；修改源码或重新安装后要重新执行 `install_senseglove.sh`，使本项目的异常保护重新编译生效。恢复现场进程时依次停止手套输入桥和 SenseGlove ROS，重启 SenseCom，确认两只手套连接后再启动 SenseGlove ROS。

无包时重建：

```bash
./laptop_teleop/scripts/install_senseglove.sh
```

有话题但数值全零时，按顺序处理：

1. SenseCom 的 **Paired Devices** 中确认两只实体手套均显示 `Connected`。
2. 核对 `gloves.yaml` 中型号、左右侧和序列号。
3. Nova 2 v1.x 检查 `/dev/rfcomm*`；v2.x 检查 BLE 配对。
4. 停止手套 ROS、重启 SenseCom、重新启动 ROS launch。
5. 用 `ros2 topic echo <topic> --once` 确认 `hand_position`、IMU 和包速率不是占位数据。

如果串口无权限：

```bash
sudo usermod -aG dialout "$USER"
```

注销并重新登录后再试。

## 无法连接机器人

```bash
ping -c 3 192.168.10.100
nc -zvu 192.168.10.100 16000
ssh esrobo@192.168.10.100
```

UDP 的 `nc` 结果不能证明网关一定应答；最终以 `run_laptop.sh inspect` 为准。检查机器人网关是否绑定 `192.168.10.100`、防火墙是否允许 UDP 16000、两端密钥是否一致。

合同不匹配时比较双方代码提交和配置：

```bash
git rev-parse HEAD
sha256sum teleoperation/config/teleop_config.yaml teleoperation/urdf/esrobo_waist_with_head.urdf
```

不要绕过合同检查。应把本机和机器人配置同步到同一版本后重启网关与客户端。

## 输入或反馈过期

本机默认人体输入最长 100 ms、机器人反馈最长 150 ms，并预留 20 ms 网络预算。出现 `stale`、`waiting for next complete source frame` 或会话退出时分别检查：

```bash
# PICO/手套采样频率
ros2 topic hz /senseglove/glove<序列号>/lh/senseglove_states

# 网络往返与抖动
ping -c 20 192.168.10.100

# 本机会话拒绝原因
ls -1t laptop_teleop/log/session_*.jsonl | head -1
python3 -m json.tool "$(ls -1t laptop_teleop/log/session_*.jsonl | head -1)" 2>/dev/null || tail -n 20 "$(ls -1t laptop_teleop/log/session_*.jsonl | head -1)"
```

JSONL 应逐行查看，通常直接使用：

```bash
tail -n 20 "$(ls -1t laptop_teleop/log/session_*.jsonl | head -1)"
```

不要先放宽超时来掩盖掉线。先确认采样率、CPU 负载、网络和机器人反馈链路。

若显示 `control rate … Hz below required 40.0 Hz`，表示 `ARMING`/`ACTIVE` 中目标生成平均频率低于门槛，本机会停止发送并请求网关停止。仅灵巧手模式采用最近两秒、35 Hz 的停止线（40 Hz 配置值减去 `hand_only_rate_tolerance_hz: 5`），以容纳 PICO 的短时采样波动；配置校验不允许将该停止线降到 35 Hz 以下。其他模式仍采用最近一秒、0.5 Hz 容差。80 ms 无目标间隔门禁、100 ms 输入新鲜度和机器人网关目标租约不变。先看最新 `session_*.jsonl` 中的 `rates.loop_hz`、`rates.compute_hz`、`rates.deadline_misses`、`source_hz` 和 `sent` 增量。`source_hz` 正常而 `sent` 掉速时，继续核对网关状态回包节拍；本机每轮须先收到新的网关状态才会计算和发送目标。网关状态发送使用固定 20 ms 节拍的修复需要在机器人上更新并重启网关进程，刷新网页不会更新已运行的网关。若更新后再次停止，应分别记录故障前两秒的目标数、PICO 输入率、网关日志和两端 CPU 负载。

灵巧手反馈应在机器人端保持约 59 Hz：

```bash
source /opt/ros/humble/setup.bash
source ~/Projects/esrobo/install/setup.bash
ros2 topic hz /cb_left_hand_state --window 100
ros2 topic hz /cb_right_hand_state --window 100
```

`hand_ros_bridge` 日志应周期性显示左右约 59 Hz。网页中的“状态距今”还包含网关 5 Hz 日志、SSH 轮询和浏览器刷新，通常为数百毫秒；它不表示手反馈进入底层控制器的延迟。

## 端口占用

```bash
ss -lunp | grep -E ':15050|:16000'
ss -lntp | grep -E ':60061|:8765|:8766|:18080'
```

本机 15050 由计算端接收 PICO/手套适配输入；机器人 16000 由网关监听；PICO PC Service 使用 TCP 60061。重复启动同一进程会产生端口冲突。

## 日志位置

- `laptop_teleop/log/xrobotoolkit_pc_service.log`：PICO PC Service。
- `laptop_teleop/log/pico_reference_*.jsonl`：PICO 参考采集。
- `laptop_teleop/log/session_*.jsonl`：约 5 Hz 的客户端状态、目标、状态新鲜度和输入拒绝原因。
- 机器人网关日志：见机器人端 `robot_link/README.md`。

会话日志中的 `targets` 是本机期望值，不代表机器人已经到位。故障后先停止所有运动入口、检查机器人实际状态和急停，再按机器人端手册恢复；客户端不会自动清故障或回零。

## 网页控制台

- 启动：`./laptop_teleop/scripts/run_dashboard.sh`，打开 `http://127.0.0.1:8080`。提示“控制台已在运行”时直接访问已有页面；若端口被其他程序占用，可用 `--port 8081`。旧版的 `Address already in use` 通常是控制台已在运行，新版会识别并提示，不会终止已有进程。
- SSH 连接失败：确认网线、机器人 IP 与密码，未知主机密钥按文档 02 验证；不要删除校验逻辑。
- 状态显示离线但 SSH 可连接：查看“机器人网关”日志，确认网关 Python 进程及 JSONL 日志持续更新。页面不靠反复 `inspect` 争抢控制会话。
- 相机无画面：依次启动机器人“头部两轴驱动”“RGB / 深度相机”“相机网页服务”；检查对应日志。相机与头部的 HTTP 通过 SSH 访问机器人 `127.0.0.1:8766`。
- PICO 骨架页显示“调试服务断开”：启动本机“PICO 数据采集”，确认头显全身追踪在产生新帧，并检查 `127.0.0.1:8765` 是否监听。相机页和骨架页的切换不会自动启动采集。
- 使能按钮不可用：检查是否已开始正式计算、输入/参考是否有效、当前是否为 `IDLE`、是否有 `EMERGENCY_STOP`，以及是否由另一个浏览器页面拥有会话。
- 独立失能提示已提交但状态未变：查看网关日志的 `disable_verified` 和对应臂的七轴 `enable_states`。`false` 或反馈过期表示无法证明硬件已失能，应继续托稳并排查 CAN/控制器；按钮请求完成不等于失能确认。
- 头部失能后位置发生变化：扭矩关闭后属于自由受力状态。操作前必须托稳相机，并以两轴 `torque_enabled=0` 为确认依据；需要重新调整时使用“开启调整”，不要尝试从失能服务启用。
- 标定停在提示：选择手套输入桥日志，按动作准备后点“姿态就绪 / 下一步”；ROS 的 SenseCom 提示使用独立的“ROS 继续”按钮。
- 显示模式不匹配：结束计算并关闭机器人网关，再使用所选模式启动网关；下拉框不会修改正在运行的进程。
- 启动脚本成功但服务没就绪：以实际反馈与进程日志为准。PICO PC Service 启动脚本正常退出属于预期，页面单独显示服务进程状态。
- 关闭/刷新控制页面导致停止：属于页面所有权和心跳机制；须检查机器人状态并按文档 05 显式恢复。
# 点击“使能并准备”后报告 `current pose unsafe/unverified`

若网关日志同时显示 `calibrated_hand_joints: 0`，这不是 PICO 输入或控制频率故障。它表示机器人仍加载旧的空 `hand.geometry_feedback_calibration`，安全模块只能按手指全运动范围检查。全范围包络可能与腰部模型相交，因此回零规划会在任何电机重新使能之前拒绝执行。

当前 L20Lite/L10 十轴映射加载成功后，网页显示 `10/10`。若仍为 `0/10`，同步 `teleoperation/config/teleop_config.yaml` 到机器人并重启网关；完成前可选择“仅左/右臂”或“仅左/右灵巧手”分别调试。不要通过降低碰撞间隙或关闭碰撞检查绕过该门禁。
