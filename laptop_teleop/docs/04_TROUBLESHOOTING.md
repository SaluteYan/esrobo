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

`Body/Hand 绝对姿态采样不同步（要求 ≤40 ms）` 表示当前 Body 与历史中最近的可用掌面源时间差超限，或所选掌面已过期；该轮不发送目标。ACTIVE 下按源输入暂缺处理：距上次有效输入不足 40 ms 且反馈健康时显示 `WAITING_INPUT`，只等待、不刷新源时间或发送计数；达到保持条件后显示 `PAUSED_INPUT`，沿用 500 ms 保持期限和连续 3 帧恢复。40 ms 配对门限与 40 ms 输入空档保持门限是不同检查。2026-10-19 修复了“尚未到保持时间返回 false，却被外层解释为不可恢复错误”的退出路径；非源输入异常、通信超时、反馈或控制器故障不会因此被放过。

`input hold exceeded 500 ms` 也不一定表示连续 500 ms 没有数据：查看 `last recovery check`，新帧可能已经到达却不满足有效性、可达性或首次 ACTIVE 输入准入要求。普通输入恢复不再以机械臂源角度差拒绝；未滤波七轴源目标仍用于诊断，腕部滤波只在成功发送后提交。100 ms 时效、40 ms 配对、连续 3 个新鲜有效帧和 500 ms 等待上限继续有效。首次 ACTIVE 到位后的输入资格与普通恢复一致，不再以固定基准或相邻帧角差拒绝；旧日志中的 `target delta ... > ...; return to previous pose` 不适用于更新后的代码。

可离线核验该边界（不连接物理网关）：

```bash
PYTHONPATH=teleoperation:teleoperation/src:robot_link:laptop_teleop/src:laptop_teleop/tests \
  python laptop_teleop/tests/tools/audit_wrist_filter_recovery.py --output /tmp/wrist-recovery.json
```

输出文件必须尚不存在。恒定源目标和有效的 8° 源变化均应在第三个新鲜有效帧恢复；源变化保留在诊断中，实际运动继续由机器人轨迹及保护约束。脚本中 arm_only 组合是向通用控制器注入腕部滤波的边界测试，不表示纯臂生产模式会使用掌面映射。性能测量继续只通过下文的独立测试入口执行。

## 标定未完成、预览没有 IK

旧日志中的 `input reacquisition source outside fixed recovery envelope` 来自此前的 J1–J4 10°、J5–J7 15°固定基准门限，当前已取消此项拒绝条件。`input_recovery.check` 仍保存固定基准、候选源目标与逐轴差值，但 `arm_limit_deg=null`，差值仅供诊断。普通恢复也已取消相邻源角度差门限，需连续 3 个新鲜有效帧；首次 ACTIVE 输入也已取消独立姿态边界和 5° 相邻变化检查。恢复过程的 3 秒期限、反馈健康及使能检查保持。手部不设相对中断前或相邻源帧的差值门限，新鲜有效输入通过限速和反馈约束衔接最新目标。回零几何计算已独立运行，状态包超长会精简可选诊断，详情见网关事件日志。

当前本机标定不再等待姿态稳定、倒计时或 12 帧窗口。若仍出现“掌面仍在移动”等旧提示，检查本机计算进程是否已退出并重新启动，以加载新代码。

带手标定需要真实有效的 26 点 Hand、非退化掌面和同步 Body；首组有效数据即可锁定，不要求自然张手。检查所选手是否可见、数据年龄是否不超过 100 ms、Body/Hand 配对是否不超过 40 ms；日志出现 calibrated 后还要确认计算计数、目标及 IK 图层更新。右/左手不能互相代替来源。仅手模式不要求 Body 或掌面参考。

## 到位后反馈过期

到位后立即出现 `following stopped: ... robot feedback stale` 时，分别检查关节原始年龄、`feedback_cache_age_s` 和本机处理时间，不应直接归因于 PICO。旧版在发布 `WAITING_FOR_ACTIVE_TARGET` 后同步做完整几何核验，可能阻塞反馈更新。当前完整核验在发布就绪前完成，ready 确认只做新鲜反馈和冻结证明范围检查；终止日志保留当时的反馈、缓存年龄和状态序号。真实过期反馈仍拒绝。

## 合同或协议不匹配

`config/URDF fingerprint mismatch` 表示本机和网关使用不同配置/模型；接管协议不匹配常见于机器人仍运行旧 Python 进程。同步双方源码、YAML、URDF 和 SDK 配套修改，在确认停止和失能后加载新进程。不要降低协议版本或跳过认证；旧网页资源可刷新，旧 Python 进程须重启。

## 控制器急停与准备失败

若“停止并保持”后点击受检回零，立即出现 `executor: RuntimeError: hold expired before hand command`，这是旧版保持转回零的并发切换缺陷：回零请求先将状态改为 `RETURNING`，尚未结束的保持动作将其误判为取消并触发急停，回零规划尚未开始。修复后，该动作将执行权交给排队中的受检回零；真实停止、反馈故障及硬件拒绝仍触发原有保护。需在现场托稳机械臂、确认安全停止及失能后同步并重启网关，不能靠重复回零或仅刷新网页加载修复。

`session preparation failed` 的 `stage=...` 指出回零或使能失败位置。`controller safety check failed after enable` 不等于 PICO 超时；核对首个控制器状态、七轴使能、模式、错误码、碰撞等级读回和力矩。控制器故障、软件 FAULT 和 CAN 接口状态是三层信息。

“关闭服务并初始化 CAN”不清除控制器电子急停。退出网关的 `q` 包含电子急停，重启后的 `IDLE` 也可能继承它。单臂显式恢复仅在新鲜七轴失能、当前姿态和几何有效、确认孤立电子急停且无其他控制器/驱动故障时复位；真实超力矩、实体急停、接触/卡阻和未知状态不应靠反复复位处理。

`controller_feedback_stale` 表示控制器状态缺失，`driver_feedback_stale` 表示七轴驱动反馈缺失；它们不是已确认的电子急停。接口 `UP` 只说明接口已启用，不证明机械臂正在发送。停止计算后，在机器人终端使用 `candump can_piper1`（左臂）或 `candump can_piper2`（右臂）被动查看反馈，使用 `ip -details -statistics link show can_piper1`（右臂替换接口名）检查 BUS-OFF 和接收/发送错误。若 CAN 有正常反馈而网关没有，核对接口映射与 SDK 解析；若没有反馈，先检查供电与通信，不通过假定零位盲目使能。

网页受检恢复要求运行网关报告 `return_protocol=3`。只更新磁盘或已有 `entry_protocol=7` 不能证明恢复代码已加载；需按部署流程在终端完成备份、源码与配置同步及校验，再在现场核验后重启网关。新版带手恢复仅在入口有界请求 CAN 遥测，不以等待时刻刷新旧反馈，不放宽时效/失能/故障检查。

CAN 推送请求会核验 SDK 是否实际调用底层发送；反馈缺失时，`feedback_request_diagnostic` 保留最近一次请求的报文和发送结果。`socket_send_returned=true` 只表示发送调用成功，不代表控制器已接受或开始上报。SDK 手册说明失能时控制器可能不应用推送设置，V1.12 的 `set_normal_mode()` 也是空操作。若持续抓包为 0 帧，应核对厂家上位机中的 CAN 推送与控制器状态；不要因接口已初始化或请求已发出，就认定反馈已恢复。重启后须重新核验新鲜七轴反馈。

受检恢复流程及使能限制见 [网页操作](05_WEB_CONSOLE.md#停止与受检恢复)。手部阶段失败且机械臂已验证健康失能时，保持软件 FAULT，不额外重锁急停；显式停止和状态不明仍执行原停止流程。

## 实时末端速度候选耗尽与停止方式

旧报错 `no safe elbow/wrist speed candidate or collision-free braking path` 是合并文案，不能据此认定发生碰撞。2026-10-19 的记录中碰撞拒绝次数为 0，原逐轴减速仍使腕部达到约 0.22294 m/s，超过 0.22 m/s；七轴协调存在满足约束的单周期候选。

当前快速搜索失败后增加七轴联合求解。日志中 `search_method=joint_slsqp` 表示使用后备搜索；优化器返回成功仍须真实 FK 和原保护检查通过。`collision_rejected` 只表示被检查的候选几何拒绝，`endpoint_search_exhausted` 表示当前搜索没找到通过的候选，均不能据此证明数学上所有轨迹无解。`solver_timeout` 与 `model_invalid` 分别表示预算超时和模型/计算异常。

网页显示“受控减速／保持（已锁存）”时，已提交合格减速命令且满足该侧保持资格；保持期间仍可能因后续真实故障转为故障停止。显示“故障停止”不代表已经核验失能，更不代表原位置承重保持。查看第一故障、独立 `trajectory_hold` 事件、执行日志与最新七轴使能/控制器反馈；恢复沿现有显式流程，不自动使能或回零。当前右侧保持尚未验证，不能仅因算法相同就认为右侧/双臂具备该能力。

## 接近、七轴回零与反馈间隔

错误 `feedback interval ... since last committed command state` 表示当前反馈时间与上次已发送、已提交轨迹状态的差，不一定是相邻 CAN 帧间隔。重复反馈、超期候选丢弃、发送失败或几何认证耗时都可能让它增长。相邻反馈间隔与轨迹状态间隔分别检查。

初始接近按 20 ms 固定期限运行，超期跳过错过的时刻，未发送候选不提交轨迹状态；读取已验证来源的非阻塞快照。几何认证只复用完整请求被单个有效区间盒覆盖的证书；上下文变化或越界必须拒绝/重规划，不能拼接证书或扩大证明范围。

当前标定后先移动 J1–J5、保持 J6/J7，再保持 J1–J5、移动 J6/J7 到同一次标定目标。两段均保持手指并使用仅臂/腕碰撞配置；七轴受检回零另有完整几何检查。`planning` 阶段失败意味着规划未通过，不说明实际发生碰撞。模型间隙和认证预算需用相同实测姿态离线复现。

## J6/J7、手指首次接管与恢复

新增有界恢复的能力开关默认关闭。先看 `input_recovery_enabled`，不能因代码已经同步就认为自动重新接管已启用。完整状态与阈值见 [计算逻辑](07_CODE_AND_COMPUTATION.md#输入恢复协议)。

分析新日志时对照 `input_recovery.check` 内的固定 anchor、候选 source、有效性判定与原始时间戳；不要把命令滤波追赶或实测反馈误差当作源目标跳变。`HOLD_LATCHED` 表示输入故障已由机器人端接管保持，恢复检查失败不再自动跟随；反馈中断时不能宣称保持有效。

腕部限速分析：在现有项目 PYTHONPATH 下运行 `python laptop_teleop/tests/tools/summarize_wrist_limits.py <session.jsonl>`。报告 J5–J7 的限制占比及采样误差/速度统计。没有 `motion_diagnostics` 的历史日志不能生成有效提速依据。先完成左右臂各自的受控保持、低幅跟随验收，再决定是否修改速度；不要同时放宽速度、领先量与力矩保护。

`initial wrist handover exceeded 10 s` 属于旧版 ACTIVE 内追赶实时腕部目标的首次接管路径。新版 `entry_protocol=7` 把 J6/J7 放在第二段固定标定目标接近中；两段完成才进入 ACTIVE。如果仍出现旧报错，核对本机和网关运行版本，不能只更新磁盘文件。新阶段执行失败在日志中区分 `approaching_arm` 和 `approaching_wrist`。

J6/J7 首次接管从实测位置限速过渡；已跟随后输入中断恢复仍检查目标连续性。日志 `J7 target delta ... > 5deg` 是恢复目标与上次有效目标的差，不能自动把它当作首次接管。

`initial hand handover exceeded 10 s` 表示固定首次手目标未在期限内完成反馈核验。`goal` 是首次到位目标，`command` 是本机限步候选，`feedback` 是实测手状态。三个值应分别比较：

- 命令迟迟达不到 goal：检查步长、速度、领先量、反向等待及输入保持。
- 已发布命令达标但实测反馈落后：检查 ROS/UDP/CAN、电机或机构响应及该侧反馈标定。
- `feedback diverged from command`：对应物理索引的反馈与已提交命令偏差超过保护，不能以放大容差代替原因分析。

L10 物理索引：0 拇指屈曲，1 拇指侧摆，2–5 食/中/无名/小指屈曲，6–8 侧摆，9 拇指旋转。左手使用十轴 URDF 拟合和独立命令映射，右手保留原增益；过渡保护共用、启用轴分侧，反馈标定及右拇指硬止点仍分侧。历史右拇指出现过反馈基本不动；这类硬件响应不能归因于 PICO 新帧转发，也不能据此忽略其他关节的拒绝。

网页区分首次目标、实时输入、本机命令、网关发布命令和实测反馈。L10 启动速度寄存器设为厂商接口最大值 255，软件仍限制 400 单位/s、单步 18 及反馈领先量；255 不是实测关节速度。

## 控制频率不足与输入保持

本机上限 50 Hz，配置要求 40 Hz、允许 5 Hz 容差，两秒窗口低于 35 Hz 会停止；有效目标间隔上限 80 ms。ACTIVE 输入间隙进入短时保持，最多 500 ms；持续缺失或无法满足连续 3 帧恢复条件会 FAULT。网关独立检查目标 lease 和机器人反馈。

`waiting for next complete source frame` 表示所需组件尚未全部产生未消费的新鲜样本，即使某个组件仍在更新也可能等待。旧记录中的 `last recovery check: J5 target delta ... > 5deg` 曾表示源变化超过普通恢复门限，该普通恢复门限现已取消。当前若出现类似信息，需核对是否为首次 ACTIVE 输入阶段或仍运行旧进程；首次准入拒绝不会重置 500 ms 保持期限。平均采集频率正常不能排除短时空档，应对照 SDK 真实输入年龄、序号和通知/转发记录。

急停后七轴可能仍报告使能，不能凭急停状态推断已失能。网页受检恢复不会自动失能：控制器急停且七轴仍使能时，须先确认机械支撑并使用独立失能入口，取得新鲜七轴失能反馈后才提交恢复。健康的使能锁存保持则走保持承重回零路径，到位后才失能。初始化 CAN 和重复点击回零均不能替代状态核验。

控制频率指成功完整目标发送频率，不是网页轮询频率。用独立工具区分 SDK 空档、Hand 无效、UDP 积压、Body/Hand 配对、IK 和发送；不能靠重复发送旧样本提高有效频率。

## 独立测试与耗时测量

正式运行不加载性能装饰器、逐周期耗时或分位数统计。测量工具在 `tests` 中运行时包装函数，结束恢复；记录进入有界异步队列。先停止正式计算，真实采集测试还须确认新鲜七轴失能反馈和唯一 SDK 订阅；不占用他人的 UDP 端口。

```bash
export PYTHONPATH="$PWD/laptop_teleop/src:$PWD/laptop_teleop/tests:$PWD/robot_link:$PWD/teleoperation/src:$PWD/teleoperation:$PWD/teleoperation/tests"
PY=/home/ddc/miniforge3/envs/esrobo_laptop/bin/python
"$PY" laptop_teleop/tests/tools/profile_runtime.py preview --synthetic --seconds 60 --output /tmp/esrobo-preview.jsonl
```

真实采集测量需要停止普通采集，并加载 SDK 库路径：

```bash
export LD_LIBRARY_PATH="$PWD/laptop_teleop/external/XRoboToolkit-PC-Service-Pybind/lib:/home/ddc/miniforge3/envs/esrobo_laptop/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$PY" laptop_teleop/tests/tools/profile_runtime.py acquisition --seconds 60 --output /tmp/esrobo-acquisition.jsonl
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

开机完全无 CAN 反馈时，普通“受检回零”不会盲目使能。若厂家上位机暂不能开启推送，独立维护操作 `remote_wake_left_zero` 仅在现场明确确认左臂已托稳、仍保持项目回零姿态后使用；先停止计算和网关，取得独占硬件锁，以项目关节偏置作为保持目标、5% 控制器速度、3 秒唤醒期限调用 SDK。此操作可能使能及运动，不能用于未知姿态。反馈恢复后核验七轴位置，执行有界失能并核验结果；无反馈不得报告失能成功。它不会自动开始遥操作或合并进普通回零按钮。

若页面提示“回零尚未启动”，且七轴仍使能、控制器 `EMERGENCY_STOP` 或有力矩故障，按钮会在发送 `r` 前拒绝。健康 `HOLD_LATCHED` 才可连续使能回零。故障状态需现场可靠托稳、排除接触或卡阻后，点击对应臂“单独失能”，确认七轴全部失能，再受检恢复。页面显示具体故障来源；使能位不能证明仍在承重。

若控制器复位及手部归零成功，随后 `planning` 的几何诊断为 `certification_budget_exhausted`，表示有限计算预算内未完成证明，不等同于已确认碰撞。当前认证会有界细分原路径边，保留全部间隙、手部及制动区域检查；执行使用同一路径上的细分路点和冻结证明。仍无法认证则拒绝运动。规划/预检失败后，只有新鲜反馈确认七轴失能且控制器、驱动均健康，才避免再次发送急停；网关保留 FAULT 和失败原因，不能假报 IDLE。

若机械臂已在空中 `HOLD_LATCHED`，点击回零后出现 `Return phase braking lost fresh joint feedback`、`controlled braking failed`，需区别于已失能的急停恢复：旧版保持回零在制动前执行完整几何计算并清空实时轨迹，随后严格反馈读数拒绝计算空档，网关将失败升级为电子急停。当前先沿用原位置/速度状态完成实时制动并确认静止，再执行完整几何规划；成功回零后才失能。控制器或反馈真正异常时仍不能保证保持承重；不要将使能位等同于位置保持有效。

若 `controller_recovery_before_hand` 已成功，但 `hand_open_before_arm` 报 `arm changed or feedback invalid during hand open`，旧版可能在手部几何计算后尚未重取完整反馈便直接进入发送检查；失败收尾又因反馈不完整重发急停。当前在第一条手指命令前有界等待新鲜完整反馈，锚点和 0.2° 漂移限制不变；运动中的反馈失败仍立即停止手部输出。具体拒绝原因和 `hand_return_feedback` 区分反馈缺失、控制器异常及某轴漂移。只在最终反馈确认健康、七轴失能时避免重复急停，不能将未完成的回零报告为 IDLE。

若旧版在 `controller_recovery_before_hand` 报 `current pose failed disabled recovery geometry/limits check`，可能是将静止复位与运动几何准入耦合。当前显式恢复先独立核验七轴失能、新鲜位置、关节范围和孤立 CAN 模式电子急停；无其他驱动故障才复位，并验证三帧健康失能反馈。此步骤不使能或运动。后续手部扫掠或机械臂路径仍必须通过原检查；几何拒绝会保留回零失败，不能报告已完成回零或跳过碰撞检查。启动时的自动故障识别不因此放宽。

若 `controller_recovery_before_hand` 已报告 `reset_sent=True`、三帧核验成功，但随后在 `enabling` 报 `fresh measured-pose preload was rejected`，应检查使能前几何计算是否耗尽了位置反馈时效。当前实现先认证实测位置周围不超过 ±0.2° 的关节区域（裁剪到关节范围），在几何与模式配置后重新取样，确认仍在已认证区域，并再次核验控制器、手部几何及发送时效；不通过刷新时间戳或放宽 100 ms 门限解决。

七轴已确认失能、控制器仅有电子急停且无其他硬件故障时，允许重新接纳长间隔后的两帧有效位置反馈，使显式恢复入口可用；该只读反馈恢复不会清除急停、软件故障或自动使能。真正的 CAN 断流、驱动故障、混合使能和无效位置仍拒绝恢复。“关闭服务并初始化 CAN”只恢复通信，不执行控制器复位。

### 力矩偏差超限后的数据

出现 `torque deviation exceeded` 时，先定位对应 `gateway_<timestamp>.jsonl` 的 `first_fault` 及同会话 `torque_<timestamp>.jsonl`。确认 `torque_diagnostics.logging` 无写入错误，检查序号缺口，再按侧别、阶段、基准编号分析；不要把接近、实时跟随、回零的不同基准混为同一统计。只有 J4 故障触发值不能推导其余关节需要增加多少门限。当前记录版本不调高门限，实机复测仍由现场执行，保护停止后保留记录，不自动重启跟随。统计命令见 [七轴力矩记录](07_CODE_AND_COMPUTATION.md#七轴力矩记录)。

### 近零位张手提示拇指包络与腰部重叠

`hand open sweep not certified safe` 表示模型扫掠范围未满足间隙，不等于已确认实物接触。当前单臂带手回零在七轴均处于 1.5°零位容差内时省略手指包络，保留手掌/臂/腕几何和反馈保护；超出容差仍检查完整扫掠。修改需机器人网关加载新源码后生效。非零位回零同时使用独立几何进程和持续反馈监控，失败报文大小另受编码检查保护。

### 回零失败后网关退出的诊断

旧版可能因同步手部几何计算让使能状态的反馈读取超过 300 ms，随后完整反馈重复嵌入失败结果，导致 UDP 状态包超出 8192 字节并关闭网关。当前独立几何进程期间持续读取反馈，不放宽时效门限。几何超时或拒绝本身不代表真实控制器故障；真实反馈/力矩/控制器异常仍执行原停止保护。查看 `return_result` 的阶段、完整反馈和几何诊断，以及 `network_state_error` 的 `diagnostics_compacted` 或 `core_state_rejected`。前者只是诊断精简；后者表示必要状态无法安全发送，必须处理故障。页面仅显示简短原因和日志提示。

左手映射出现 version/model mismatch 时，核对本机配置与模型并重启 PICO 转发和计算进程。计算超时／进程退出不会继续复用旧结果。诊断及回放见 [左手十轴重定向](07_CODE_AND_COMPUTATION.md#左手-l20lite-十轴重定向)。
## 运行一段时间后暂停、抓取时手指受阻（2026-10-16）

现场三次 `input reacquisition exceeded 3 s` 都发生在 PICO 短暂缺帧后的恢复期间，并非总运行时长限制。旧逻辑追逐每帧移动的源目标，要求机械臂误差 ≤1.5°、手指误差 ≤2 单位；记录中恢复阶段机械臂最大轴误差中位数为 6.35°、2.16°、9.20°，手指误差中位数为 26、16、11 单位，因而无法结束恢复。现改为核验连续有效输入并保持机器人轨迹保护，不以源目标静止到位为恢复条件。

另一次为 `physical[3] goal=182 command=166 feedback=190 budget=8 lead=15`：24 单位反馈偏差使两组允许区间不相交。随后按现场要求将 `live_feedback_lead` 设为 `null`：ACTIVE 和输入恢复不再根据手指指令/反馈差值限幅或拒绝，原 30/60 单位门限均不生效；目标只按原速度、单步和行程限制趋近人手目标，允许反向张手。受阻不动时指令仍可继续变化。首次接管仍严格到位，反馈过期、输入跳变及碰撞/力矩故障继续阻止运动。此次没有放宽启动阶段的 `entry_certificate_miss` 几何认证；日志中的两次接近阶段失败属于另一类保护。

离线记录位于 `laptop_teleop/log/retained/stop_contact_20261016/`；这些是位置反馈，只能确认指令/反馈滞后，不能量化抓持力。

## 回归测试历史失败清理

2026-10-19 对此前 37 项失败的归因、保留／删除依据和验证结果，见 [修改日志中的回归测试清理记录](08_CHANGELOG.md#2026-10-19-回归测试清理)。历史测试日志中的失败数量不代表当前回归状态，也不代表实机已完成全部验证。

## 日志与测试目录维护

`laptop_teleop/log/` 根目录由运行程序自动写入，不手工重命名仍被进程打开的文件；`dashboard_teleop.json` 等是状态文件，不按普通日志删除。自动保留器保护打开文件、近期变化文件及带 `.keep` 的目录。

`log/retained/` 保存需要复核的故障、现场回放输入和验收汇总。人工清理时：

- 现场数据和有效故障证据保留；关闭的 PICO 原始采集／传输文件可无损压缩到 `retained/sessions/<会话>/`，压缩后校验解压 SHA256 再移除原文件。
- 当前 J6/J7 回放所需 `motion`、标定和执行记录仍保留原 JSONL 路径。完全相同的副本只保留一份，删除前比较内容哈希。
- `compute.jsonl`／`pico.jsonl`／`execution.jsonl` 若来自明确的模拟性能实验，可清理明细；保留对应 `result.json`、性能汇总、输入生成工具、最终验收和失败记录。不按文件扩展名批量删除现场 JSONL。
- `retained/maintenance/` 保存整理清单，记录删除理由、归档路径与 SHA256；源码和文档整理前的备份放在项目 `.deploy-backups/`。

读取压缩现场记录时先解压到临时目录供现有 JSONL 工具使用，例如 `gzip -dc <archive.jsonl.gz> > /tmp/field-session.jsonl`。归档文件名保留原始会话编号，不能把文件修改时间当作采样时间。容量目标不是硬上限；活动采集仍会持续占用空间。

`tests/` 只存有效回归、固定 `fixtures/` 与独立 `tools/`。完整日志和一次性输出不放进测试目录。当前结构及运行入口见 [测试索引](../tests/README.md)；可清理字节码缓存，不能依据测试年限或文件数量删除有效用例。
