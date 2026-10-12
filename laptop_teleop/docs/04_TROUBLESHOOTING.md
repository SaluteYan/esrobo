# 故障排查与独立测试

先根据发生阶段和第一条拒绝原因定位。网页中的毫秒数通常是样本年龄，不是网络延迟。故障后读新鲜反馈，不能根据旧日志推断机械臂已失能。

## 基本检查

```bash
./laptop_teleop/scripts/run_laptop.sh check --side left
./laptop_teleop/scripts/run_laptop.sh inspect --side left
./laptop_teleop/scripts/xrobotoolkit_service.sh status
ss -lunp | rg '15050|15051|15052|16000'
ss -lntp | rg '60061|63901|8080|8765|8766'
```

`inspect` 会占用一个短会话，退出后等会话释放；不要与正式计算同时运行。端口占用先确认进程所有者，不直接抢占正在运行的采集端口。环境依赖和 SDK 重建见 [本机 README](../README.md#环境安装与配置)。

## PICO 骨架不动、数据超时

按链路区分：头显 → PC Service → SDK 组件提交 → Body/Hand 转发 → UDP 接收 → 配对/计算 → 网页。

- 头显佩戴亮屏、追踪应用在前台，Body 和 Hand Tracking 均启用。手部 `inactive` 时即使 Body 正常，也不能生成带手模式的完整目标。
- PC Service TCP 63901 收到数据不代表 SDK gRPC 60061 已收到 Body/Hand。查看 SDK 组件计数、序号和真实年龄，不能只看进程是否存在。
- 比较 SDK Body 更新与 Body 实际转发频率。均匀 60/74/90 Hz 模拟输入的转发目标为至少 95%；突发只发送最新有效帧，合并计数增加是正常现象，不补发历史包。
- 本机转发正常而计算等待完整帧时，检查同侧 Body、Hand、掌面有效性和 40 ms 同步条件。100 ms 时效依据原接收时刻，映射或读取缓存不会刷新时间。
- 骨架“原始采集”“固定映射”层来自采集；重定向、IK、输出及机器人反馈层需要预览/正式计算发布。固定映射的“固定”指坐标变换，不代表静态模型。
- 网页骨架按 50 ms 轮询，即约 20 Hz 显示；不能用动画刷新率代替 SDK 或转发的新帧频率。选择计算层但计算未运行时，页面会显示等待原因。
- SDK 提交本身存在停顿时记录为上游空档；SDK 及时提交而转发停顿才属于本机处理问题。不要提高超时阈值或给缓存重标时间。

仅在计算已停止并确认无重复订阅后重启采集；若 PC Service 到 SDK 停滞，可按现场操作重启 PC Service 和唯一采集。旧 SDK 缺少通知接口会明确要求 `install_pico.sh` 重建。

## 标定未完成、预览没有 IK

带手标定需要真实有效的 26 点 Hand、非退化掌面、至少 12 帧稳定数据和同步 Body。让所选手在摄像头内自然张开，肩肘腕稳定；日志出现 calibrated 后还要确认计算计数、目标及 IK 图层更新。右/左手不能互相代替来源。仅手模式不要求 Body 或掌面参考。

## 合同或协议不匹配

`config/URDF fingerprint mismatch` 表示本机和网关使用不同配置/模型；接管协议不匹配常见于机器人仍运行旧 Python 进程。同步双方源码、YAML、URDF 和 SDK 配套修改，在确认停止和失能后加载新进程。不要降低协议版本或跳过认证；旧网页资源可刷新，旧 Python 进程须重启。

## 控制器急停与准备失败

`session preparation failed` 的 `stage=...` 指出回零或使能失败位置。`controller safety check failed after enable` 不等于 PICO 超时；核对首个控制器状态、七轴使能、模式、错误码、碰撞等级读回和力矩。控制器故障、软件 FAULT 和 CAN 接口状态是三层信息。

“关闭服务并初始化 CAN”不清除控制器电子急停。退出网关的 `q` 包含电子急停，重启后的 `IDLE` 也可能继承它。单臂显式恢复仅在新鲜七轴失能、当前姿态和几何有效、确认孤立电子急停且无其他控制器/驱动故障时复位；真实超力矩、实体急停、接触/卡阻和未知状态不应靠反复复位处理。

受检恢复流程及使能限制见 [网页操作](05_WEB_CONSOLE.md#停止与受检恢复)。手部阶段失败且机械臂已验证健康失能时，保持软件 FAULT，不额外重锁急停；显式停止和状态不明仍执行原停止流程。

## 接近、七轴回零与反馈间隔

错误 `feedback interval ... since last committed command state` 表示当前反馈时间与上次已发送、已提交轨迹状态的差，不一定是相邻 CAN 帧间隔。重复反馈、超期候选丢弃、发送失败或几何认证耗时都可能让它增长。相邻反馈间隔与轨迹状态间隔分别检查。

初始接近按 20 ms 固定期限运行，超期跳过错过的时刻，未发送候选不提交轨迹状态；读取已验证来源的非阻塞快照。几何认证只复用完整请求被单个有效区间盒覆盖的证书；上下文变化或越界必须拒绝/重规划，不能拼接证书或扩大证明范围。

当前标定后规划只移动 J1–J5、保持 J6/J7 和手指，使用仅臂/腕碰撞配置；七轴受检回零另有完整几何检查。`planning` 阶段失败意味着规划未通过，不说明实际发生碰撞。模型间隙和认证预算需用相同实测姿态离线复现。

## J6/J7、手指首次接管与恢复

J6/J7 首次接管从实测位置限速过渡；已跟随后输入中断恢复仍检查目标连续性。日志 `J7 target delta ... > 5deg` 是恢复目标与上次有效目标的差，不能自动把它当作首次接管。

`initial hand handover exceeded 10 s` 表示固定首次手目标未在期限内完成反馈核验。`goal` 是首次到位目标，`command` 是本机限步候选，`feedback` 是实测手状态。三个值应分别比较：

- 命令迟迟达不到 goal：检查步长、速度、领先量、反向等待及输入保持。
- 已发布命令达标但实测反馈落后：检查 ROS/UDP/CAN、电机或机构响应及该侧反馈标定。
- `feedback diverged from command`：对应物理索引的反馈与已提交命令偏差超过保护，不能以放大容差代替原因分析。

L10 物理索引：0 拇指屈曲，1 拇指侧摆，2–5 食/中/无名/小指屈曲，6–8 侧摆，9 拇指旋转。左右共享映射增益和过渡代码，反馈标定及右拇指硬止点仍分侧。历史右拇指出现过反馈基本不动；这类硬件响应不能归因于 PICO 新帧转发，也不能据此忽略其他关节的拒绝。

网页区分首次目标、实时输入、本机命令、网关发布命令和实测反馈。L10 启动速度寄存器设为厂商接口最大值 255，软件仍限制 400 单位/s、单步 18 及反馈领先量；255 不是实测关节速度。

## 控制频率不足与输入保持

本机上限 50 Hz，配置要求 40 Hz、允许 5 Hz 容差，两秒窗口低于 35 Hz 会停止；有效目标间隔上限 80 ms。ACTIVE 输入间隙进入短时保持，最多 500 ms；持续缺失或无法满足连续 3 帧恢复条件会 FAULT。网关独立检查目标 lease 和机器人反馈。

控制频率指成功完整目标发送频率，不是网页轮询频率。用独立工具区分 SDK 空档、Hand 无效、UDP 积压、Body/Hand 配对、IK 和发送；不能靠重复发送旧样本提高有效频率。

## 独立测试与耗时测量

正式运行不加载性能装饰器、逐周期耗时或分位数统计。测量工具在 `tests` 中运行时包装函数，结束恢复；记录进入有界异步队列。先停止正式计算，真实采集测试还须确认新鲜七轴失能反馈和唯一 SDK 订阅；不占用他人的 UDP 端口。

```bash
export PYTHONPATH="$PWD/laptop_teleop/src:$PWD/laptop_teleop/tests:$PWD/robot_link:$PWD/teleoperation/src:$PWD/teleoperation:$PWD/teleoperation/tests"
PY=/home/ddc/miniforge3/envs/esrobo_laptop/bin/python
"$PY" laptop_teleop/tests/profile_runtime.py preview --synthetic --seconds 60 --output /tmp/esrobo-preview.jsonl
```

真实采集测量需要停止普通采集，并加载 SDK 库路径：

```bash
export LD_LIBRARY_PATH="$PWD/laptop_teleop/external/XRoboToolkit-PC-Service-Pybind/lib:/home/ddc/miniforge3/envs/esrobo_laptop/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$PY" laptop_teleop/tests/profile_runtime.py acquisition --seconds 60 --output /tmp/esrobo-acquisition.jsonl
```

真实预览使用 `preview`，不加 `--synthetic`；它连接独立模拟网关，不调用物理运动接口，输入端口仍要求唯一接收者。验收必须有实际 arm/wrist IK 调用和有效目标，未完成参考不算计算负载通过。`benchmark_pico_forwarding.py` 可用同一模拟输入比较转发比例、慢手部计算和阻塞日志；其他 `benchmark_*.py` 是离线反馈/几何对照工具，参数用 `--help` 查看。

同机延迟用单调时钟；跨机器只比较各自间隔和序号，不直接相减时间戳。输出属于本机测试结果，不提交运行日志或临时报告。

## 日志与网页连接

本机日志在 `laptop_teleop/log/`，包括采集、计算 JSONL、参考及控制台状态；机器人日志在 `robot_link/log/`。异步日志保留队列丢弃数、错误和必要故障事件；队列拥堵不应阻塞实时路径。

相机 SSH `Connection refused` 常表示机器人 8766 图像服务未启动；PICO 骨架断开检查本机 8765。它们与运动 UDP 16000 独立。控制台或网关“已提交”只说明操作请求已发送，执行结果仍看状态和新鲜反馈。
