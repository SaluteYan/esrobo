# ESROBO 分布式遥操作

**当前实验分支选择：** `hand.left_retarget_solver: wrist5`，左手按手腕到五个指尖的向量联合求解；v13仍可配置切回。算法接入与验证边界见 [五向量运行接入](docs/07_CODE_AND_COMPUTATION.md#五向量运行接入2026-10-21)。磁盘配置不代表运行进程已更新，服务加载状态以本次部署记录为准。

2026-10-22运行配置显式选择 `left_vector_geometry: aligned_thumb_fingers`，接入统一掌部参考系及四指总骨长缩放（当前含侧向修正的版本为 `left-dex-wrist5-anatomical-live-v3`）；旧配置默认仍为 `original`。四指姿态改善和剩余对捏间隙见 [技术方案](docs/09_TECHNICAL_SCHEME.md#参考点与逐指尺寸修正离线对照2026-10-22)。

2026-10-22 12:16已同步几何v3侧向参数修正并重启网关／PICO，指纹一致。依据和剩余执行偏差见 [技术方案](docs/09_TECHNICAL_SCHEME.md#无名指小指侧向错位修正2026-10-22几何v3已同步)。

`teleoperation/` 是唯一遥操作目录，内部 `esrobo_laptop` 在电脑上采集 PICO Body/Hand，完成参考标定、重定向和 IK，再通过认证 UDP 将关节目标发送给机器人 `robot_link` 网关。网页负责服务管理、预览、人工准备、状态和日志；机器人主机运行 CAN/ROS 驱动、反馈校验和轨迹执行。当前输入使用 PICO，不需要 SenseGlove。

`esrobo_teleop` 保留机器人网关依赖的配置、模型、IK、轨迹及驱动组件；`robot_link/` 继续作为独立通信模块。旧的机器人单机采集／求解／控制入口已删除。目录迁移及删除清单见 [修改日志](docs/08_CHANGELOG.md)。

本文负责项目入口、**环境安装与配置**；日常操作和实现细节分别放在 [docs](docs/01_README.md)。

## 快速入口

环境已经就绪时，在仓库根目录运行：

```bash
./teleoperation/scripts/run_dashboard.sh
```

打开 **http://127.0.0.1:8080**。网页启动不自动使能或运动。按 [网页指南](docs/05_WEB_CONSOLE.md)选择模式、连接机器人、检查采集、预览，再人工准备。完整每日步骤见 [运行手册](docs/03_OPERATION_RUNBOOK.md)。

## 环境安装与配置

本机部署以 Ubuntu 22.04 amd64 为基准，默认 Miniforge 在当前用户的 `~/miniforge3`、环境为 `esrobo_laptop`，机器人为 Ubuntu/ROS 2 Humble。路径由 `scripts/common.sh` 等部署设置决定；新机器不要把示例路径当成已安装状态。

### 安装当前 PICO 计算端

从仓库根目录执行：

```bash
./teleoperation/scripts/install_env.sh
./teleoperation/scripts/install_pico.sh
```

`install_env.sh` 安装用户目录 Miniforge、创建 Conda 环境并安装本项目；`install_pico.sh` 校验并安装 PC Service，获取外部源码、应用仓库 SDK 补丁、构建原生库与 Python 绑定。系统安装使用交互 sudo，不保存密码。升级 SDK 也运行 `install_pico.sh`，随后重启唯一采集进程。

`setup_all.sh` 是兼容完整安装入口，还会构建旧 SenseGlove 工作空间；当前 PICO 部署不必执行这一旧设备安装。机器人端 LinkerHand ROS 驱动仍需保留。

环境文件与用途：

| 文件 | 用途 |
| --- | --- |
| [environment.yml](environment.yml) | Conda Python 3.10、NumPy/SciPy、Pinocchio、PyYAML、Paramiko、编译工具和测试依赖。 |
| [pyproject.toml](pyproject.toml) | 本机 Python 包元数据及 pip 安装依赖。 |
| [config/laptop.yaml](config/laptop.yaml) | 网关地址、模式、密钥路径、输入端口与时效、50 Hz 控制及频率门禁。 |
| [config/teleop_config.yaml](config/teleop_config.yaml) | 两端一致的模型、映射、IK、驱动与保护参数。 |
| `scripts/common.sh` | 脚本路径和 Miniforge/环境定位。 |
| `scripts/patch_pico_sdk.py`、`scripts/pico_component_sample.hpp` | SDK 组件快照、新帧通知和接口版本补丁。 |

这些机器可读文件保留为实际安装/配置入口，README 不替代它们。安装目录、密钥和运行日志不提交 Git。

### 机器人端环境

机器人仅安装执行组件与底层 SDK，不启动 PICO 采集或本机网页。`environment.robot.yml` 保留现有 `teleop_esrobo` 环境名，入口为：

```bash
CONDA_EXE=/实际路径/miniconda3/bin/conda ./teleoperation/scripts/install_robot_env.sh
```

ROS 驱动仍使用系统 ROS 2 环境；网关启动见 [robot_link](../robot_link/README.md)。两个 Python 包统一由 `esrobo-teleoperation` 安装。已有环境先移除旧 editable 分发注册，再重新安装（不删除环境、SDK、标定或日志）：

```bash
python -m pip uninstall esrobo-laptop-teleop esrobo-teleop-real
python -m pip install --no-deps -e ./teleoperation -e ./robot_link
```

这些命令从仓库根目录执行；首次安装使用上面的对应环境安装器。在停止运行程序后迁移旧目录中的 `external/`、本机配置及需保留的日志，禁止直接用新目录覆盖已有标定。本次整理的工作区没有这些外部依赖或现场数据需要迁移。

### Python 与 SDK 验证

```bash
source /home/ddc/miniforge3/etc/profile.d/conda.sh
conda activate esrobo_laptop
python -c 'import numpy, scipy, yaml, pinocchio; print("IK environment OK")'
```

脚本默认直接调用环境 Python，不要求先激活。Miniforge 在其他位置时可设置：

```bash
MINIFORGE_DIR=/实际路径/miniforge3 ./teleoperation/scripts/install_env.sh
```

验证 SDK 通知接口：

```bash
PY=/home/ddc/miniforge3/envs/esrobo_laptop/bin/python
LIB="$PWD/teleoperation/external/XRoboToolkit-PC-Service-Pybind/lib"
PYTHONNOUSERSITE=1 LD_LIBRARY_PATH="$LIB:/home/ddc/miniforge3/envs/esrobo_laptop/lib" \
  "$PY" -c 'import xrobotoolkit_sdk as s; assert s.get_tracking_updates_version()==1; assert all(callable(getattr(s,n,None)) for n in ("wait_tracking_updates","get_tracking_diagnostics","close")); print("PICO SDK notifications v1 OK")'
```

旧 SDK 明确要求重建，不回退到读取缓存时刻作为新时间。通知/独立转发规则见 [PICO 输入](docs/06_PICO_HAND_INPUT.md)。

### ROS 与网络边界

机器人侧的双手、相机等使用 ROS 2 Humble 和已构建工作空间；构建用系统 Python，不在 Conda 中替代 ROS Python。机器人配置与部署见 [连接指南](docs/02_ROBOT_CONNECTION.md)及 [硬件调试](../HARDWARE_BRINGUP.md)。

头显连接本机当前局域网 IP，PC Service 接入 TCP 63901；本机 SDK 使用 127.0.0.1:60061。运动目标/反馈使用机器人 UDP 16000，本机 Body/Hand 默认 UDP 15050，手部机器人桥 15051/15052。共享密钥位于 `~/.config/esrobo/robot-link.key`，两端一致且权限 600，不能写入 YAML 或仓库。

```bash
./teleoperation/scripts/run_laptop.sh check --side right
./teleoperation/scripts/xrobotoolkit_service.sh status
```

`check_environment.sh` 检查当前计算端与 PICO Service，不再把可选 SenseGlove 依赖作为必要条件。

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
teleoperation/
├── README.md / pyproject.toml       统一入口与一个 Python 分发包
├── environment.yml                 本机采集和计算环境
├── environment.robot.yml           机器人执行环境
├── config/                         laptop.yaml 与 teleop_config.yaml
├── src/esrobo_laptop/               本机采集、映射、IK 流水线、网页
├── src/esrobo_teleop/               共用模型配置、IK、轨迹及设备适配器
├── bridges/                        PICO／可选 SenseGlove 输入与机器人手桥
├── urdf/ / assets/                 模型与按固定版本获取的碰撞资源
├── scripts/                        分端安装、启动、诊断、分析与测试
├── web/                            控制台及 pico_skeleton_viewer
├── docs/                           当前运行文档与统一修改日志
├── tests/ / tools/                 现有回归、离线回放和分析工具
├── linker_hand_console/            独立手部诊断台
└── external/ / log/                本机依赖与日志，不提交
```

Body 和 Hand 由一个 SDK 订阅独立提交/转发，控制端按真实时间和组件序号配对。单臂准备先受检归零灵巧手、再回臂，完成后标定，首次接近先让 J1–J5 到位、再让 J6/J7 到达同一次标定目标，手指保持实测值，两段到位后进行全轴实时跟随；显式受检恢复则先在当前失能臂姿态把手归零，再执行七轴回零。阶段规则和保护见 [计算逻辑](docs/07_CODE_AND_COMPUTATION.md)。

源码更新、两机同步、进程加载和真机验收分别完成。软件回归不会启动真实运动接口：

```bash
./teleoperation/scripts/test.sh
# 默认布局、连续死区和 PICO 冒烟；指定文件只跑该文件
./teleoperation/scripts/test.sh teleoperation/tests/test_wrist_handover.py
# 跨模块变更／发布前显式运行完整功能回归
./teleoperation/scripts/test.sh --full
```

采集时序性能验收单独运行 `test.sh --performance`；完整基准使用独立 `tests/tools/profile_runtime.py` 等工具。正式运行不加载逐函数性能包装器。按改动选用 `--hand`、`--handover`、`--transport`、`--web`、`--arm` 或具体用例，避免每次小改动运行完整回归；详见 [验证范围](docs/04_TROUBLESHOOTING.md#按修改范围验证)。

`log/retained/` 中部分历史 JSON／JSONL 已改为同名 `.gz` 无损归档；需要回放时按[维护说明](docs/04_TROUBLESHOOTING.md#日志与测试目录维护)解压所需文件。运行状态、活动采集、模型、安装环境及有效测试样本不属于清理对象。`__pycache__/` 和 `.pytest_cache/` 可再生；清理它们不等于删除测试，也不需要重新执行全套运动回归。

## 文档导航

| 需要解决的问题 | 文档 |
| --- | --- |
| 连接机器人、共享密钥、部署服务 | [02 两机连接](docs/02_ROBOT_CONNECTION.md) |
| 每次运行和结束顺序、命令行替代 | [03 日常操作](docs/03_OPERATION_RUNBOOK.md) |
| 超时、频率、控制器、手部和独立测量 | [04 故障排查](docs/04_TROUBLESHOOTING.md) |
| 网页按钮、模式、骨架层、恢复 | [05 网页控制台](docs/05_WEB_CONSOLE.md) |
| SDK、组件时间、Body/Hand 转发与同步 | [06 PICO 输入](docs/06_PICO_HAND_INPUT.md) |
| 源码职责、映射/IK、接近和回零实现 | [07 代码与计算](docs/07_CODE_AND_COMPUTATION.md) |
| 灵巧手运动学、对捏机制与单指运动问题 | [09 遥操作与灵巧手技术方案](docs/09_TECHNICAL_SCHEME.md) |

旧 SenseGlove 脚本保留用于设备维护，当前网页和 PICO 计算无需启动。认证密钥、现场运行记录、构建产物和历史测量汇总不上传远程仓库。

左手 L20Lite 十轴映射、官方手册约束及离线三维回放见 [左手十轴重定向](docs/07_CODE_AND_COMPUTATION.md#左手-l20lite-十轴重定向)。
