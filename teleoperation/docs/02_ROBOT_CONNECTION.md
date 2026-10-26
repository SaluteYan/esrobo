# 两机连接与机器人服务

本文负责网络、认证和服务部署。环境安装见 [本机 README](../README.md#环境安装与配置)，日常控制顺序见 [运行手册](03_OPERATION_RUNBOOK.md)。示例本机仓库为 `/home/ddc/DualArmTeleoperation`，机器人仓库为 `/home/esrobo/Projects/esrobo`；新部署按实际路径修改。

左手 v4 求解另需在笔记本构建数值内核：安装环境脚本包含 `build` 依赖（pybind11 头文件），系统需要 C++17 编译器和对应 Python 开发头文件。同步后可先执行 `bash teleoperation/scripts/build_hand_native.sh`；PICO 启动脚本也会先核对源码／Python ABI 缓存并构建。编译不在采集循环中运行，不把笔记本的二进制缓存复制给机器人。只更新源码不代表运行进程已加载新版本；仍须按停止、失能核验和指纹核对流程重启。

## 网络与 SSH

机器人默认地址为 `192.168.10.100`，运动协议使用 UDP `16000`。头显应连接运行 PC Service 的本机局域网地址，不能填写机器人地址或 `127.0.0.1`。

```bash
ip -br address
ping -c 3 192.168.10.100
ssh esrobo@192.168.10.100
```

需要独立机器人网口时，可配置本机 `192.168.10.101/24`；先核对网卡和现有连接，避免覆盖上网配置。网页使用本机 `~/.ssh/known_hosts`，不自动信任未知主机。首次连接或机器人重装后须核对主机指纹。可用 `ssh-copy-id` 安装公钥，网页 SSH 密码仅用于当前连接。

## 通信共享密钥

已有部署优先复制机器人现有密钥，避免让两端分别生成不同内容：

```bash
mkdir -p ~/.config/esrobo
scp esrobo@192.168.10.100:~/.config/esrobo/robot-link.key ~/.config/esrobo/robot-link.key
chmod 600 ~/.config/esrobo/robot-link.key
```

仅在两端都没有密钥时创建一次，再安全复制到机器人同一路径：

```bash
mkdir -p ~/.config/esrobo
(umask 077; python3 -c 'import secrets; from pathlib import Path; p=Path.home()/".config/esrobo/robot-link.key"; f=p.open("x"); f.write(secrets.token_hex(32)+"\n"); f.close()')
scp ~/.config/esrobo/robot-link.key esrobo@192.168.10.100:/tmp/robot-link.key
ssh esrobo@192.168.10.100 'mkdir -p ~/.config/esrobo && mv /tmp/robot-link.key ~/.config/esrobo/robot-link.key && chmod 600 ~/.config/esrobo/robot-link.key'
```

密钥、SSH 密码和会话令牌不能提交到仓库或写入诊断记录。HMAC 验证消息真实性，不加密运动数据。

## 同步版本与配置

本机和机器人须同步 `robot_link`、`teleoperation/src`、机器人 YAML、URDF 及相应驱动修改。只更新网页不会让已运行网关加载新源码。

```bash
git rev-parse HEAD
ssh esrobo@192.168.10.100 'cd /home/esrobo/Projects/esrobo && git rev-parse HEAD'
```

配置/URDF 指纹和接管协议由客户端检查。出现 fingerprint mismatch 时同步双方配置及模型，在现场确认停止和失能后重启网关；不绕过合同检查。机器人端 LinkerHand ROS 源码更新后还需重新构建该包并加载新的 `install/setup.bash`。

实验分支选择 `hand.left_retarget_solver: wrist5` 时，向量依赖仅安装在笔记本：执行 `bash teleoperation/scripts/install_dex_vector_experiment.sh`，生成 `teleoperation/external/dex-vector-venv/`。采集父进程继续使用原环境，独立子进程加载固定版本的 dex-retargeting、Torch、Pinocchio 和 NLopt；机器人网关不安装这些求解依赖。缺少环境或优化器源码校验不一致会拒绝启动，不自动回退 v13。

运行前同时更新两端配置及本机采集进程，核对采集映射版本和两端合同。当前配置含几何侧向修正，预期版本为 `left-dex-wrist5-anatomical-live-v3`；原始腕向量配置才使用 `left-dex-wrist5-live-v1`。只复制到 `.deploy-staging/` 表示已暂存，不能视为已生效。缺少新鲜七轴反馈或使能状态时不重启服务，也不通过自动失能取得部署条件。

## 机器人端启动

推荐在网页按所选模式启动服务。命令行替代流程如下；不要与网页启动的同名服务并行。

CAN 未就绪时，先确认机械臂停止且失能、相关驱动已退出，再执行：

```bash
cd /home/esrobo/Projects/esrobo
./start_can.sh
ip -br link show type can
```

CAN 端口命名和 USB 位置见 [硬件调试文档](../../HARDWARE_BRINGUP.md#can-和串口设备)。初始化 CAN 不会清除控制器电子急停。

带手模式先启动双手 ROS 驱动，再启动唯一的双手桥，各占一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 teleoperation/bridges/hand_ros_bridge.py --side both
```

另一个终端启动所选侧网关，例为右臂＋右手：

```bash
PYTHONPATH="$PWD/robot_link:$PWD/teleoperation/src" \
  /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python \
  -m esrobo_link.gateway --hardware --side right --with-hand \
  --bind 192.168.10.100 --config "$PWD/teleoperation/config/teleop_config.yaml" \
  --key-file ~/.config/esrobo/robot-link.key
```

左右模式及仅手参数见 [本机 README](../README.md#运行模式)。网关持有机械臂驱动，不同时运行另一个网关、旧遥操作入口或独立机械臂控制进程。双臂自动回零/准备因缺少互碰扫掠核验而被拒绝。

## 本机只读验证

```bash
./teleoperation/scripts/run_laptop.sh inspect --side right
```

核对合同、网关模式、控制器故障、七轴使能和反馈年龄。`inspect` 会短暂占用会话，退出后等待会话释放再启动正式计算。`IDLE`、tmux 存在或启动命令成功均不能替代新鲜反馈；急停恢复见 [网页恢复操作](05_WEB_CONSOLE.md#停止与受检恢复)。
