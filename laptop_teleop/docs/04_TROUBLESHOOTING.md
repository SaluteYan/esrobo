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

更换两个无线网络后仍有空档，说明需要按突发输入处理，但不能据此单独认定是 PICO 的固有缺陷。对照当前头显 TCP 连接的内核接收间隔、SDK 原始提交间隔和本机计算耗时；不要把旧网络遗留的 ESTAB 连接混入统计。软件合并积压和有界保持能减少额外延迟，不能生成尚未收到的真实姿态。

`input hold exceeded 500 ms` 也不一定表示连续 500 ms 没有数据：查看 `last recovery check`，新帧可能已经到达却不满足连续性。带手模式恢复使用未滤波掌面映射出的 J5–J7 源目标；腕部滤波只在成功发送后提交，避免把滤波追赶误判为人手跳变。原 100 ms 时效、40 ms 配对、连续 3 帧、5°/15 单位和 500 ms 上限继续有效。

可离线核验该边界（不连接物理网关）：

```bash
PYTHONPATH=teleoperation:teleoperation/src:robot_link:laptop_teleop/src:laptop_teleop/tests \
  python laptop_teleop/tests/audit_wrist_filter_recovery.py --output /tmp/wrist-recovery.json
```

输出文件必须尚不存在。恒定源目标应在第三个新鲜匹配帧恢复，真实 8° 源跳变应拒绝并达到原保持期限。脚本中 arm_only 组合是向通用控制器注入腕部滤波的边界测试，不表示纯臂生产模式会使用掌面映射。性能测量继续只通过下文的独立测试入口执行。

## 标定未完成、预览没有 IK

若在 `TARGET PAUSED` 后看到 `input reacquisition source outside fixed recovery envelope`，检查终止记录的 `input_recovery.check`：它保存固定基准、候选源目标与逐轴差值。终止提示也会列出超界的 J 轴及手部通道。J1–J4 重新接管范围为 10°、J5–J7 为 15°、手部为 30 单位；这是实时输入恢复检查，与初始标定无关。旧版将最后可跟随目标的时间同时用于空档计时，可能把正常不可达保持误当输入中断；当前已分离完整输入时间和恢复基准。

当前本机标定不再等待姿态稳定、倒计时或 12 帧窗口。若仍出现“掌面仍在移动”等旧提示，检查本机计算进程是否已退出并重新启动，以加载新代码。

带手标定需要真实有效的 26 点 Hand、非退化掌面和同步 Body；首组有效数据即可锁定，不要求自然张手。检查所选手是否可见、数据年龄是否不超过 100 ms、Body/Hand 配对是否不超过 40 ms；日志出现 calibrated 后还要确认计算计数、目标及 IK 图层更新。右/左手不能互相代替来源。仅手模式不要求 Body 或掌面参考。

## 合同或协议不匹配

`config/URDF fingerprint mismatch` 表示本机和网关使用不同配置/模型；接管协议不匹配常见于机器人仍运行旧 Python 进程。同步双方源码、YAML、URDF 和 SDK 配套修改，在确认停止和失能后加载新进程。不要降低协议版本或跳过认证；旧网页资源可刷新，旧 Python 进程须重启。

## 控制器急停与准备失败

`session preparation failed` 的 `stage=...` 指出回零或使能失败位置。`controller safety check failed after enable` 不等于 PICO 超时；核对首个控制器状态、七轴使能、模式、错误码、碰撞等级读回和力矩。控制器故障、软件 FAULT 和 CAN 接口状态是三层信息。

“关闭服务并初始化 CAN”不清除控制器电子急停。退出网关的 `q` 包含电子急停，重启后的 `IDLE` 也可能继承它。单臂显式恢复仅在新鲜七轴失能、当前姿态和几何有效、确认孤立电子急停且无其他控制器/驱动故障时复位；真实超力矩、实体急停、接触/卡阻和未知状态不应靠反复复位处理。

`controller_feedback_stale` 表示控制器状态缺失，`driver_feedback_stale` 表示七轴驱动反馈缺失；它们不是已确认的电子急停。接口 `UP` 只说明接口已启用，不证明机械臂正在发送。停止计算后使用“检查机械臂 CAN 接收”，对照左右臂帧数、CAN ID、BUS-OFF 和接收/发送错误。若 CAN 有正常反馈而网关没有，核对接口映射与 SDK 解析；若没有反馈，先检查供电与通信，不通过假定零位盲目使能。

网页受检恢复要求运行网关报告 `return_protocol=2`。只更新磁盘或已有 `entry_protocol=6` 不能证明恢复代码已加载；可先通过“同步机器人代码”备份、同步和校验，再在现场核验后重启网关。新版带手恢复仅在入口有界请求 CAN 遥测，不以等待时刻刷新旧反馈，不放宽时效/失能/故障检查。

CAN 推送请求会核验 SDK 是否实际调用底层发送；反馈缺失时，`feedback_request_diagnostic` 保留最近一次请求的报文和发送结果。`socket_send_returned=true` 只表示发送调用成功，不代表控制器已接受或开始上报。SDK 手册说明失能时控制器可能不应用推送设置，V1.12 的 `set_normal_mode()` 也是空操作。若持续抓包为 0 帧，应核对厂家上位机中的 CAN 推送与控制器状态；不要因接口已初始化或请求已发出，就认定反馈已恢复。重启后须重新核验新鲜七轴反馈。

受检恢复流程及使能限制见 [网页操作](05_WEB_CONSOLE.md#停止与受检恢复)。手部阶段失败且机械臂已验证健康失能时，保持软件 FAULT，不额外重锁急停；显式停止和状态不明仍执行原停止流程。

## 接近、七轴回零与反馈间隔

错误 `feedback interval ... since last committed command state` 表示当前反馈时间与上次已发送、已提交轨迹状态的差，不一定是相邻 CAN 帧间隔。重复反馈、超期候选丢弃、发送失败或几何认证耗时都可能让它增长。相邻反馈间隔与轨迹状态间隔分别检查。

初始接近按 20 ms 固定期限运行，超期跳过错过的时刻，未发送候选不提交轨迹状态；读取已验证来源的非阻塞快照。几何认证只复用完整请求被单个有效区间盒覆盖的证书；上下文变化或越界必须拒绝/重规划，不能拼接证书或扩大证明范围。

当前标定后规划只移动 J1–J5、保持 J6/J7 和手指，使用仅臂/腕碰撞配置；七轴受检回零另有完整几何检查。`planning` 阶段失败意味着规划未通过，不说明实际发生碰撞。模型间隙和认证预算需用相同实测姿态离线复现。

## J6/J7、手指首次接管与恢复

新增有界恢复的能力开关默认关闭。先看 `input_recovery_enabled`，不能因代码已经同步就认为 15°自动接管已启用。完整状态与阈值见 [计算逻辑](08_CODE_AND_COMPUTATION.md#输入恢复协议-1)。

分析新日志时对照 `input_recovery.check` 内的固定 anchor、候选 source、相邻帧判定与原始时间戳；不要把命令滤波追赶或实测反馈误差当作源目标跳变。`HOLD_LATCHED` 表示输入故障已由机器人端接管保持，恢复检查失败不再自动跟随；反馈中断时不能宣称保持有效。

腕部限速分析：在现有项目 PYTHONPATH 下运行 `python laptop_teleop/tests/summarize_wrist_limits.py <session.jsonl>`。报告 J5–J7 的限制占比及采样误差/速度统计。没有 `motion_diagnostics` 的历史日志不能生成有效提速依据。先完成左右臂各自的受控保持、低幅跟随验收，再决定是否修改速度；不要同时放宽速度、领先量与力矩保护。

`initial wrist handover exceeded 10 s` 是腕部首次过渡超时，与手指接管超时不同。到位检查使用原腕部容差，同时要求命令和反馈达标并连续确认三帧；旧版要求命令精确等于实时目标，持续变化的小目标也可能始终无法完成接管。

J6/J7 首次接管从实测位置限速过渡；已跟随后输入中断恢复仍检查目标连续性。日志 `J7 target delta ... > 5deg` 是恢复目标与上次有效目标的差，不能自动把它当作首次接管。

`initial hand handover exceeded 10 s` 表示固定首次手目标未在期限内完成反馈核验。`goal` 是首次到位目标，`command` 是本机限步候选，`feedback` 是实测手状态。三个值应分别比较：

- 命令迟迟达不到 goal：检查步长、速度、领先量、反向等待及输入保持。
- 已发布命令达标但实测反馈落后：检查 ROS/UDP/CAN、电机或机构响应及该侧反馈标定。
- `feedback diverged from command`：对应物理索引的反馈与已提交命令偏差超过保护，不能以放大容差代替原因分析。

L10 物理索引：0 拇指屈曲，1 拇指侧摆，2–5 食/中/无名/小指屈曲，6–8 侧摆，9 拇指旋转。左右共享映射增益和过渡代码，反馈标定及右拇指硬止点仍分侧。历史右拇指出现过反馈基本不动；这类硬件响应不能归因于 PICO 新帧转发，也不能据此忽略其他关节的拒绝。

网页区分首次目标、实时输入、本机命令、网关发布命令和实测反馈。L10 启动速度寄存器设为厂商接口最大值 255，软件仍限制 400 单位/s、单步 18 及反馈领先量；255 不是实测关节速度。

## 控制频率不足与输入保持

本机上限 50 Hz，配置要求 40 Hz、允许 5 Hz 容差，两秒窗口低于 35 Hz 会停止；有效目标间隔上限 80 ms。ACTIVE 输入间隙进入短时保持，最多 500 ms；持续缺失或无法满足连续 3 帧恢复条件会 FAULT。网关独立检查目标 lease 和机器人反馈。

`waiting for next complete source frame` 表示所需组件尚未全部产生未消费的新鲜样本，即使某个组件仍在更新也可能等待。`last recovery check: J5 target delta ... > 5deg` 表示恢复时已有计算目标，但与上一有效目标的差超过连续性门限；收到这些新帧不会重置 500 ms 保持期限。平均采集频率正常不能排除短时空档，应对照 SDK 真实输入年龄、序号和通知/转发记录。

急停后七轴可能仍报告使能，不能凭急停状态推断已失能。网页受检恢复在人工确认机械支撑后先对所选单臂失能，取得新鲜七轴失能反馈后才提交恢复；未取得反馈则停止。初始化 CAN 和重复点击回零均不能替代这一前置核验。

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

左臂真实预览指定 `--side left --compute-seconds 60`，计算窗口从首次成功求解开始，`--seconds` 是包括标定等待的总时限。需要同时测量采集时，在另一个终端启动 `acquisition --forward-only`：它只发送，接收端口由预览独占。普通采集须先停止，测量结束后恢复，两个采集进程不能同时订阅 SDK。

测试记录中的 `native_body/left/right` 保留 SDK 原始接收时刻和序号，包括 Python 等待返回前被合并的帧；后续映射记录带样本序号、开始时刻、墙钟及线程 CPU 耗时。判断原始接收间隔应使用 `receipt_ns`，不能使用测试写入队列的时间。自动摘要的中位数/P95仅覆盖最后 200 次，正式对照报告须从完整 JSONL 重算全程分布。

同机延迟用单调时钟；跨机器只比较各自间隔和序号，不直接相减时间戳。输出属于本机测试结果，不提交运行日志或临时报告。

## 日志与网页连接

本机日志在 `laptop_teleop/log/`，包括采集、计算 JSONL、参考及控制台状态；机器人日志在 `robot_link/log/`。异步日志保留队列丢弃数、错误和必要故障事件；队列拥堵不应阻塞实时路径。

日志保留规则：最近 7 天；带时间戳的同类 JSONL 最多 10 份，PICO 传输日志最多 3 份；每个 `log/` 目录按旧到新清理至 512 MiB 目标。正在打开的文件、最近一小时更新的文件、网页运行/腰部状态文件、`COLCON_IGNORE` 和带 `.keep` 文件的目录始终保留，因此占用可能暂时超过目标。符号链接不跟随，扫描后发生写入的文件延期处理。需要长期保留故障证据时，将其放入 `log/retained/` 并在该目录创建 `.keep`。

新版网页及实机网关启动后，在独立后台线程启动一次清理，随后每小时执行；不进入运动控制循环，不截断活跃日志。源码更新后需重启相应服务才会加载，推送 Git 不会重启机器人。也可在项目根目录、既有 Python 环境下手动预览/执行：

```bash
PYTHONPATH=teleoperation/src python -m esrobo_teleop.debug.log_retention
PYTHONPATH=teleoperation/src python -m esrobo_teleop.debug.log_retention --apply
```

四个管理目录为根目录 `log/`、`laptop_teleop/log/`、`teleoperation/log/`、`robot_link/log/`；仅处理执行机器上的目录。日志、状态快照和临时分析结果由 `.gitignore` 排除，不上传到远程仓库。

相机 SSH `Connection refused` 常表示机器人 8766 图像服务未启动；PICO 骨架断开检查本机 8765。它们与运动 UDP 16000 独立。控制台或网关“已提交”只说明操作请求已发送，执行结果仍看状态和新鲜反馈。

### 失能后电子急停无法清除

若旧版在 `controller_recovery_before_hand` 报 `current pose failed disabled recovery geometry/limits check`，可能是将静止复位与运动几何准入耦合。当前显式恢复先独立核验七轴失能、新鲜位置、关节范围和孤立 CAN 模式电子急停；无其他驱动故障才复位，并验证三帧健康失能反馈。此步骤不使能或运动。后续手部扫掠或机械臂路径仍必须通过原检查；几何拒绝会保留回零失败，不能报告已完成回零或跳过碰撞检查。启动时的自动故障识别不因此放宽。
