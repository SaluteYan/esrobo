# ESROBO 本机遥操作

`laptop_teleop` 在电脑上采集 PICO Body/Hand，完成参考标定、重定向和 IK，再通过认证 UDP 将关节目标发送给机器人 `robot_link` 网关。网页负责服务管理、预览、人工准备、状态和日志；机器人主机运行 CAN/ROS 驱动、反馈校验和轨迹执行。当前输入使用 PICO，不需要 SenseGlove。

本文负责项目入口、**环境安装与配置**；日常操作和实现细节分别放在 [docs](docs/01_README.md)。

## 快速入口

环境已经就绪时，在仓库根目录运行：

```bash
./laptop_teleop/scripts/run_dashboard.sh
```

打开 **http://127.0.0.1:8080**。网页启动不自动使能或运动。按 [网页指南](docs/05_WEB_CONSOLE.md)选择模式、连接机器人、检查采集、预览，再人工准备。完整每日步骤见 [运行手册](docs/03_OPERATION_RUNBOOK.md)。

## 环境安装与配置

本机部署以 Ubuntu 22.04 amd64 为基准，默认 Miniforge 在 `/home/ddc/miniforge3`、环境为 `esrobo_laptop`，机器人为 Ubuntu/ROS 2 Humble。路径由 `scripts/common.sh` 等部署设置决定；新机器不要把示例路径当成已安装状态。

### 安装当前 PICO 计算端

从仓库根目录执行：

```bash
./laptop_teleop/scripts/install_env.sh
./laptop_teleop/scripts/install_pico.sh
```

`install_env.sh` 安装用户目录 Miniforge、创建 Conda 环境并安装本项目；`install_pico.sh` 校验并安装 PC Service，获取外部源码、应用仓库 SDK 补丁、构建原生库与 Python 绑定。系统安装使用交互 sudo，不保存密码。升级 SDK 也运行 `install_pico.sh`，随后重启唯一采集进程。

`setup_all.sh` 是兼容完整安装入口，还会构建旧 SenseGlove 工作空间；当前 PICO 部署不必执行这一旧设备安装。机器人端 LinkerHand ROS 驱动仍需保留。

环境文件与用途：

| 文件 | 用途 |
| --- | --- |
| [environment.yml](environment.yml) | Conda Python 3.10、NumPy/SciPy、Pinocchio、PyYAML、Paramiko、编译工具和测试依赖。 |
| [pyproject.toml](pyproject.toml) | 本机 Python 包元数据及 pip 安装依赖。 |
| [config/laptop.yaml](config/laptop.yaml) | 网关地址、模式、密钥路径、输入端口与时效、50 Hz 控制及频率门禁。 |
| [../teleoperation/config/teleop_config.yaml](../teleoperation/config/teleop_config.yaml) | 两端一致的模型、映射、IK、驱动与保护参数。 |
| `scripts/common.sh` | 脚本路径和 Miniforge/环境定位。 |
| `scripts/patch_pico_sdk.py`、`scripts/pico_component_sample.hpp` | SDK 组件快照、新帧通知和接口版本补丁。 |

这些机器可读文件保留为实际安装/配置入口，README 不替代它们。安装目录、密钥和运行日志不提交 Git。

### Python 与 SDK 验证

```bash
source /home/ddc/miniforge3/etc/profile.d/conda.sh
conda activate esrobo_laptop
python -c 'import numpy, scipy, yaml, pinocchio; print("IK environment OK")'
```

脚本默认直接调用环境 Python，不要求先激活。Miniforge 在其他位置时可设置：

```bash
MINIFORGE_DIR=/实际路径/miniforge3 ./laptop_teleop/scripts/install_env.sh
```

验证 SDK 通知接口：

```bash
PY=/home/ddc/miniforge3/envs/esrobo_laptop/bin/python
LIB="$PWD/laptop_teleop/external/XRoboToolkit-PC-Service-Pybind/lib"
PYTHONNOUSERSITE=1 LD_LIBRARY_PATH="$LIB:/home/ddc/miniforge3/envs/esrobo_laptop/lib" \
  "$PY" -c 'import xrobotoolkit_sdk as s; assert s.get_tracking_updates_version()==1; assert all(callable(getattr(s,n,None)) for n in ("wait_tracking_updates","get_tracking_diagnostics","close")); print("PICO SDK notifications v1 OK")'
```

旧 SDK 明确要求重建，不回退到读取缓存时刻作为新时间。通知/独立转发规则见 [PICO 输入](docs/06_PICO_HAND_INPUT.md)。

### ROS 与网络边界

机器人侧的双手、相机等使用 ROS 2 Humble 和已构建工作空间；构建用系统 Python，不在 Conda 中替代 ROS Python。机器人配置与部署见 [连接指南](docs/02_ROBOT_CONNECTION.md)及 [硬件调试](../HARDWARE_BRINGUP.md)。

头显连接本机当前局域网 IP，PC Service 接入 TCP 63901；本机 SDK 使用 127.0.0.1:60061。运动目标/反馈使用机器人 UDP 16000，本机 Body/Hand 默认 UDP 15050，手部机器人桥 15051/15052。共享密钥位于 `~/.config/esrobo/robot-link.key`，两端一致且权限 600，不能写入 YAML 或仓库。

```bash
./laptop_teleop/scripts/run_laptop.sh check --side right
./laptop_teleop/scripts/xrobotoolkit_service.sh status
```

`check_environment.sh` 仍检查旧 SenseGlove 完整安装，缺少旧手套组件不等同于当前 PICO 无法运行；当前模式以 `run_laptop.sh check` 的必要组件检查为准。

## 运行模式

| 模式 | 本机参数 | 网关参数 | 必需 PICO 输入 |
| --- | --- | --- | --- |
| 右臂＋右手 | `--side right` | `--side right --with-hand` | 右 Body/Hand/掌面。 |
| 左臂＋左手 | `--side left` | `--side left --with-hand` | 左 Body/Hand/掌面。 |
| 纯右臂 | `--side right --arm-only` | `--side right` | 右 Body。 |
| 纯左臂 | `--side left --arm-only` | `--side left` | 左 Body。 |
| 仅右手 | `--side right --hand-only` | `--side right --with-hand --hand-only` | 右 Hand，机械臂七轴失能。 |
| 仅左手 | `--side left --hand-only` | `--side left --with-hand --hand-only` | 左 Hand，机械臂七轴失能。 |
| 双臂双手 | 默认 | `--side both` | 双侧完整输入；不支持自动准备/回零。 |

网页按模式启动程序，已有网关不会随下拉框切换。本机和网关的侧别、手模式、协议、机器人配置及 URDF 指纹须一致。

## 源码边界

```text
laptop_teleop/
├── README.md                项目入口、环境安装与配置
├── environment.yml          Conda 依赖
├── pyproject.toml           Python 包安装
├── config/laptop.yaml       本机运行参数
├── docs/                    连接、操作、网页、PICO、排障、计算逻辑
├── scripts/                 安装、启动、SDK 补丁和回归入口
├── src/esrobo_laptop/       采集、输入、映射、IK 流水线、状态机和网页后台
├── web/                     控制台前端
├── tests/                   回归和测试专用性能分析
├── external/                外部依赖及构建，不提交
└── log/                     本机日志和状态，不提交
```

Body 和 Hand 由一个 SDK 订阅独立提交/转发，控制端按真实时间和组件序号配对。单臂准备先受检归零灵巧手、再回臂，完成后标定，首次接近先让 J1–J5 到位、再让 J6/J7 到达同一次标定目标，手指保持实测值，两段到位后进行全轴实时跟随；显式受检恢复则先在当前失能臂姿态把手归零，再执行七轴回零。阶段规则和保护见 [计算逻辑](docs/07_CODE_AND_COMPUTATION.md)。

源码更新、两机同步、进程加载和真机验收分别完成。软件回归不会启动真实运动接口：

```bash
./laptop_teleop/scripts/test.sh
```

性能耗时仅由独立 `tests/tools/profile_runtime.py` 等工具测量，正式运行不加载逐周期耗时或分位数统计。测试方式见 [排障指南](docs/04_TROUBLESHOOTING.md#独立测试与耗时测量)。

## 文档导航

| 需要解决的问题 | 文档 |
| --- | --- |
| 连接机器人、共享密钥、部署服务 | [02 两机连接](docs/02_ROBOT_CONNECTION.md) |
| 每次运行和结束顺序、命令行替代 | [03 日常操作](docs/03_OPERATION_RUNBOOK.md) |
| 超时、频率、控制器、手部和独立测量 | [04 故障排查](docs/04_TROUBLESHOOTING.md) |
| 网页按钮、模式、骨架层、恢复 | [05 网页控制台](docs/05_WEB_CONSOLE.md) |
| SDK、组件时间、Body/Hand 转发与同步 | [06 PICO 输入](docs/06_PICO_HAND_INPUT.md) |
| 源码职责、映射/IK、接近和回零实现 | [07 代码与计算](docs/07_CODE_AND_COMPUTATION.md) |

旧 SenseGlove 脚本保留用于设备维护，当前网页和 PICO 计算无需启动。认证密钥、现场运行记录、构建产物和历史测量汇总不上传远程仓库。

左手 L20Lite 十轴映射、官方手册约束及离线三维回放见 [左手十轴重定向](docs/07_CODE_AND_COMPUTATION.md#左手-l20lite-十轴重定向)。
