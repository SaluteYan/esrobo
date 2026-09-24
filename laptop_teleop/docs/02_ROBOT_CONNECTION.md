# 网络、SSH 与机器人网关

当前网络检查结果：本机 `enp12s0=192.168.10.101/24`，机器人 `192.168.10.100` 可达（约 0.2 ms）。2026-09-22 已在本机创建 `~/.config/esrobo/robot-link.key` 并安全复制到机器人，两端权限均为 `600`。

同日机器人已在三个 tmux 会话中启动 `esrobo_hands`、`esrobo_hand_bridge` 和 `esrobo_gateway`。网关保持 `IDLE`，双手及双臂七轴反馈均有效；右臂正常失能，左臂控制器报告继承的 `EMERGENCY_STOP`，七轴已确认失能。在现场按厂家流程解除左臂急停并重新执行只读检查前，不得向网关输入 `e`。

## 1. 网络检查

把本机连接到机器人局域网。机器人固定地址为 `192.168.10.100`；当前本机有线网卡 `enp12s0` 已配置为 `192.168.10.101/24`，并已实测可 ping 通机器人。先查看连接名：

```bash
nmcli connection show
ip -br address
```

需要设置静态地址时，把 `<连接名>` 替换为实际值：

```bash
sudo nmcli connection modify "<连接名>" ipv4.method manual \
  ipv4.addresses 192.168.10.101/24 ipv4.gateway "" ipv4.dns ""
sudo nmcli connection up "<连接名>"
ping -c 3 192.168.10.100
```

如果该网卡还承担上网任务，不要随意覆盖现有配置；可以为机器人网口新建独立 NetworkManager 连接。

## 2. SSH

机器人用户名以现场实际账户为准，下文用 `esrobo`：

```bash
ssh esrobo@192.168.10.100
```

可选地安装本机公钥，之后无需反复输入机器人密码：

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -C esrobo-laptop
ssh-copy-id -i ~/.ssh/id_ed25519.pub esrobo@192.168.10.100
ssh esrobo@192.168.10.100
```

## 3. 配置通信共享密钥

如果机器人已经部署 `~/.config/esrobo/robot-link.key`，优先把这份现有密钥安全复制到本机。先在本机确认文件：

```bash
mkdir -p ~/.config/esrobo
scp esrobo@192.168.10.100:~/.config/esrobo/robot-link.key ~/.config/esrobo/robot-link.key
chmod 600 ~/.config/esrobo/robot-link.key
```

如果两端都没有密钥，只在本机创建一次：

```bash
mkdir -p ~/.config/esrobo
(umask 077; python3 -c 'import secrets; from pathlib import Path; p=Path.home()/".config/esrobo/robot-link.key"; f=p.open("x"); f.write(secrets.token_hex(32)+"\n"); f.close()')
scp ~/.config/esrobo/robot-link.key esrobo@192.168.10.100:/tmp/robot-link.key
ssh esrobo@192.168.10.100 'mkdir -p ~/.config/esrobo && mv /tmp/robot-link.key ~/.config/esrobo/robot-link.key && chmod 600 ~/.config/esrobo/robot-link.key'
```

密钥不能写入 YAML、命令历史参数、日志或仓库。两端内容必须完全相同：

```bash
sha256sum ~/.config/esrobo/robot-link.key
ssh esrobo@192.168.10.100 'sha256sum ~/.config/esrobo/robot-link.key'
```

## 4. 同步代码与配置

机器人上应有同一版本仓库。查看双方提交：

```bash
cd /home/ddc/DualArmTeleoperation && git rev-parse HEAD
ssh esrobo@192.168.10.100 'cd /home/esrobo/Projects/esrobo && git rev-parse HEAD'
```

本机和机器人必须使用相同的 `teleoperation/config/teleop_config.yaml`、URDF、`robot_link` 协议代码。合同指纹不一致时客户端会拒绝发送。

## 5. 机器人端启动

登录机器人并进入仓库：

```bash
ssh esrobo@192.168.10.100
cd /home/esrobo/Projects/esrobo
```

先按机器人端文档初始化 CAN、启动双手底层驱动与一个双手桥：

```bash
./start_can.sh
ip -br link show type can
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
```

双手 UDP/ROS 桥另开机器人终端：

```bash
cd /home/esrobo/Projects/esrobo
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 teleoperation/bridges/hand_ros_bridge.py --side both
```

再开机器人网关终端：

```bash
cd /home/esrobo/Projects/esrobo
PYTHONPATH="$PWD/robot_link:$PWD/teleoperation/src" \
  /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python \
  -m esrobo_link.gateway --hardware --side both --bind 192.168.10.100 \
  --config "$PWD/teleoperation/config/teleop_config.yaml" \
  --key-file ~/.config/esrobo/robot-link.key
```

网关必须先保持 `IDLE`。不要在机器人上同时运行旧遥操作入口、另一个网关或第二个双手桥。详细硬件门禁与恢复命令以机器人端 [`../../robot_link/README.md`](../../robot_link/README.md) 为准。

当前部署使用 tmux 保持三个进程，SSH 断开后仍继续运行。查看会话：

```bash
ssh esrobo@192.168.10.100
tmux ls
tmux attach -t esrobo_hands
tmux attach -t esrobo_hand_bridge
tmux attach -t esrobo_gateway
```

从 tmux 会话退出但不停止进程，按 `Ctrl+B`，松开后按 `D`。网关终端的 `e/s/x/d/q` 都必须按 Enter；当前左臂急停未解除时不得输入 `e`。日志位于机器人仓库的 `robot_link/log/`。

## 6. 本机只读验证

机器人网关为 `IDLE` 时，在本机执行：

```bash
cd /home/ddc/DualArmTeleoperation
./laptop_teleop/scripts/run_laptop.sh inspect
```

输出应包含已验证的合同、`IDLE` 状态以及左右七轴和双手反馈。`inspect` 会短暂占用唯一会话，退出后等待至少 1 秒再启动正式计算端。

当前已验证的合同 ID 为 `734c63758a78df184838fa603454556387402e33c00eb29b129d74bd000f21ca`。若 `controller_fault.status_name` 为 `EMERGENCY_STOP`，不得反复启动客户端或输入 `e`；先保持七轴失能，排除实体急停、接触、线缆和控制器原因，按厂家流程复位，再重新运行 `inspect`，直到两侧 `controller_fault` 均为 `null`。
