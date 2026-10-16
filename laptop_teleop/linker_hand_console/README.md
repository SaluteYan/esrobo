# 机器人端 Linker Hand Console

保存本会话使用的厂商 Linker Hand Console 26.4.0 的部署配置和管理脚本。脚本在笔记本运行，厂商程序及内置 CAN Bridge 在机器人上运行，浏览器通过网线访问机器人。

```text
本机 192.168.10.101 浏览器 → 机器人 192.168.10.100:7080
                                      ↓ 内置 CAN Bridge :5260
                              can_hand2 左手 / can_hand1 右手
```

`config.json` 保存 SSH 用户、主机、远端目录、程序校验值和接口映射。配置及脚本均不保存 SSH/sudo 密码；使用现有 SSH 公钥，sudo 密码在终端交互输入。依赖系统 Python 3.10+、ssh、scp；无需激活 conda。

## 当前部署

- 机器人程序：`/home/esrobo/linker-hand-console/dashboard`
- 后台服务：`linker-hand-console.service`，使用临时 systemd unit，SSH 退出后继续运行，机器人重启后不会自动启动。
- 当前双手配置：`CAN_PORTS=can_hand2,can_hand1`，均为 1 Mbps。
- 网页：<http://192.168.10.100:7080>，只有机器人自身使用内部 5260 端口。
- 2026-10-15 已确认项目配置及设备识别均为左/右 L10（worm_gear）。URDF 名称不能替代实物型号核对。

厂商原始程序和压缩包没有加入仓库。26.4.0 的原始下载地址：
<https://botdata.oss-cn-beijing.aliyuncs.com/upload/dashboard_26.4.0_linux_amd64_1782714255620.zip>

压缩包 SHA256：`3fa0a32488455155f16d80e9ecc510dee5a83b99a4798b9b4a53c4892ee1a95c`。程序 SHA256 保存在配置里。更换电脑后更新 `local_binary`；脚本不会自动下载或接受另一版本。

## 在本机运行

以下命令均从 `/home/ddc/DualArmTeleoperation` 执行：

```bash
# 检查后台服务和设备识别
python3 laptop_teleop/linker_hand_console/console.py status

# 首次复制程序；已有同一程序时只做校验
python3 laptop_teleop/linker_hand_console/console.py deploy

# 通过本机 8080 项目控制台交接已有网关、双手桥和双手 ROS 驱动
python3 laptop_teleop/linker_hand_console/console.py handover

# 启动双手；也可以 --side left 或 --side right
python3 laptop_teleop/linker_hand_console/console.py start --side both
python3 laptop_teleop/linker_hand_console/console.py open

# 日志和停止
python3 laptop_teleop/linker_hand_console/console.py logs
python3 laptop_teleop/linker_hand_console/console.py stop
```

`handover` 只在项目控制台已连接机器人、没有控制页面占用、遥操作计算已结束、网关有新鲜 `IDLE` 状态且所管理机械臂七轴均有失能反馈时退出网关，再停止手部桥和 ROS 驱动。其它状态会拒绝；按项目既有停止、支撑和恢复流程处理后再交接。它不自动回零、解除急停、使能或发送手势。

`start` 检查冲突的网关、双手 ROS 驱动、手部桥和厂商控制进程。发现冲突就拒绝启动，不强杀进程。只允许当前已核实的两个手部接口，不执行整机 `start_can.sh`，不重命名接口。已有服务以相同接口运行时复用；更改左右手须先 `stop`。

## 网页单独调试

页面可能列出 `can_car`、`can_waist`、`can_piper1/2` 等未知设备，因为厂商界面枚举了系统全部 CAN。只勾选实际识别为目标手的 `can_hand1/2`，不要使用“全选”。内置 CAN Bridge 实际仅初始化 `CAN_PORTS` 指定的手部接口；其余接口没有可用 CAN 发送连接。

先核对左右手和型号、检查反馈，再从单指小幅运动开始。脚本自身不发送运动目标；厂商网页直接控制手指，不经过本项目的手部映射、限速或碰撞保护。保持相关机械臂在现场已确认的安全状态。

关闭服务也可能关闭它管理的手部 CAN 接口。恢复项目遥操作前，先确认厂商服务已停止，通过项目控制台按所选模式恢复手部接口、双手 ROS 驱动、手部桥和网关。整机 `start_can.sh` 会重置机械臂、腰部和底盘总线；其它部件仍运行时不要直接执行。

如果适配器重新插拔，先 `stop`，确认机器人端仍按固定映射得到 `can_hand1/2` 后再 `start`。接口回到 `can0` 等名称时脚本会拒绝启动，先核对物理端口并按项目流程恢复命名。

参照：[`../docs/05_WEB_CONSOLE.md`](../docs/05_WEB_CONSOLE.md)、[`../../start_can.sh`](../../start_can.sh)、[`../../src/linkerhand-ros2/linker_hand_ros2_sdk/launch/linker_hand_double.launch.py`](../../src/linkerhand-ros2/linker_hand_ros2_sdk/launch/linker_hand_double.launch.py)。

## 本地检查

```bash
python3 -B laptop_teleop/linker_hand_console/console.py --help
python3 -B -m unittest discover -s laptop_teleop/linker_hand_console -p 'test_*.py'
```
