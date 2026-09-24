# 操作文档

文档按首次部署和日常使用拆分：

1. [`01_ENVIRONMENT_SETUP.md`](01_ENVIRONMENT_SETUP.md)：首次安装 Miniforge/Conda、ROS 2 Humble、PICO XRoboToolkit、SenseGlove ROS。
2. [`02_ROBOT_CONNECTION.md`](02_ROBOT_CONNECTION.md)：配置本机到 `192.168.10.100` 的网络、SSH 和通信密钥，并启动机器人网关。
3. [`03_OPERATION_RUNBOOK.md`](03_OPERATION_RUNBOOK.md)：每次遥操作的启动顺序、标定、使能、停止和模式参数。
4. [`04_TROUBLESHOOTING.md`](04_TROUBLESHOOTING.md)：环境、设备话题、端口、时延和日志排查。
5. [`05_WEB_CONSOLE.md`](05_WEB_CONSOLE.md)：一条命令启动网页，在浏览器管理本机/机器人服务、标定、遥操作与相机。
6. [`06_PICO_HAND_INPUT.md`](06_PICO_HAND_INPUT.md)：当前 PICO 全身、五指与手腕统一输入，SDK 升级、启动步骤和故障语义。SenseGlove 安装资料仅供旧设备维护，当前遥操作不需要启动手套软件。

仓库路径按本机当前位置写为：

```bash
export ESROBO_REPO=/home/ddc/DualArmTeleoperation
cd "$ESROBO_REPO"
```

如果以后移动仓库，只需修改 `ESROBO_REPO`。所有脚本都会根据自身位置查找项目，不依赖固定仓库路径。
