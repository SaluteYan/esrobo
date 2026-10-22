# 本机遥操作测试

- 根目录 `test_*.py`：有效的自动化回归。保留按功能划分的文件，避免为了文件数量合并不相关的安全检查。
- `fixtures/`：回归必需的固定样本及原生代码。`moving_recovery/` 从已保存现场会话提取 REACQUIRING 记录，来源与 SHA256 见 `provenance.json`；测试不再依赖可清理的运行日志。
- `tools/`：独立分析、回放、性能测量工具；不由生产入口导入，不在每次 pytest 中自动运行。工具之间通过包名引用，原根目录工具路径已迁移。
- `__pycache__/`：可再生缓存，可在测试结束后清理。

2026-10-21 全仓库整理删除了测试字节码、pytest缓存和已确认的模拟运行输出，保留三个项目测试目录中的有效源码、fixtures及分析工具；本次没有按文件年龄或用例数量削减保护覆盖。文档／缓存／归档整理只做清单、哈希、语法与链接核验，不触发完整运动回归。历史工具需要的压缩记录按 [归档说明](../docs/04_TROUBLESHOOTING.md#日志与测试目录维护)按需解压；`fixtures/` 中JSONL是固定回归样本，不是待删除运行日志。

项目根目录执行：

```bash
bash laptop_teleop/scripts/test.sh
```

无参数只运行42项快速冒烟，不代表完整回归。小改动优先直接传入测试文件或 `文件::用例`，不会额外运行整套目录：

```bash
bash laptop_teleop/scripts/test.sh laptop_teleop/tests/test_hand_handover.py
bash laptop_teleop/scripts/test.sh --hand
bash laptop_teleop/scripts/test.sh --full
bash laptop_teleop/scripts/test.sh --performance
```

还可选择 `--handover`、`--transport`、`--web`、`--arm`，使用 `--help` 查看入口。`--full` 覆盖本机、robot_link 和分区 IK 的现行功能用例；原来长期排除的两项旧轨迹模拟已删除，不再靠 `-k` 隐藏。10项机器负载敏感的采集时序用例标记为 `performance`，原门槛不变，单独运行并报告。它们不属于被废弃的测试；改变采集调度、并发、日志开销或计算速度时仍需验收。成功不代表仓库全部硬件相关测试通过，也不代表实机已就绪。

入口会先构建左手 C++17 数值内核，依赖见安装环境脚本的 `test,build` extras。直接调用 pytest 或下方回放工具前，先执行 `bash laptop_teleop/scripts/build_hand_native.sh`。几何与接触测试同时运行原生实现和 Python 参考实现，并逐值比较 FK／雅可比与求解结果；缓存二进制不纳入仓库。

映射回归按左右侧分开验收：左手 `urdf_10` 直接按控制标定转换十轴 URDF 角度，不重复施加旧增益；右手继续验证旧增益与止点。SDK 生命周期测试使用模拟映射器和内存 UDP socket，临时目录仅承载日志；真实配置加载另行验证，测试不得向运行中的采集端口注入模拟帧。

独立工具通常使用以下环境（替换 Python 为项目环境解释器）：

```bash
export PYTHONPATH="$PWD/laptop_teleop/src:$PWD/laptop_teleop/tests:$PWD/robot_link:$PWD/teleoperation/src:$PWD/teleoperation:$PWD/teleoperation/tests"
python laptop_teleop/tests/tools/audit_wrist_filter_recovery.py --output /tmp/wrist-recovery.json
python laptop_teleop/tests/tools/benchmark_motion_trace.py --enabled 1 --seconds 60 --output /tmp/motion-benchmark-new
python laptop_teleop/tests/tools/summarize_torque.py /path/to/torque.jsonl
```

`benchmark_*` 是离线模拟或几何计算；`audit_transfers.py` 可读取真实服务状态，`profile_runtime.py acquisition` 会使用实际采集，运行前按 [排障文档](../docs/04_TROUBLESHOOTING.md#独立测试与耗时测量)确认使用条件。不要把所有工具批量当作离线测试启动。

新增测试应覆盖尚未覆盖的行为、边界或真实回归，优先扩展既有参数化用例，不为每次常量、文案或文档修改机械新增文件。若功能明确废弃，删除对应断言并在统一 [修改日志](../docs/08_CHANGELOG.md)记录依据。保留有效的保护验证，不为减少数量删除独立故障场景。按修改范围运行一次足够的验证，只有新改动或未解决失败才重复；完整回归用于公共接口、跨模块变更及发布验收。详细选择依据见 [按修改范围验证](../docs/04_TROUBLESHOOTING.md#按修改范围验证)。

2026-10-22实验分支整理仅移除可再生缓存及归档目录内已结束的一次性试验入口；现行v13和wrist5均仍可选，相关回归不视作废弃。维护向量参数时优先使用 `scripts/replay_hand_vector.py` 和 `scripts/compare_wrist_geometry.py`，不要再次复制参数扫描程序到测试目录。本次未新增用例或扩大默认测试范围；向量专项40项和发布提交专项5项通过，未重复完整回归。未跟踪的测试按仓库现有 `tests/` 忽略规则留在本机，不能将本机通过数量理解为远程检出后自动具备同一套测试。

左手接触重定向可重复验收：

长时间验收前，先把所需输入快照保存在 `log/retained/<本次验收>/`，按现有规则添加 `.keep`；不要依赖普通运行日志在整个测试期间仍被保留。活动日志只做窗口副本，不移动或截断原文件。

```bash
python laptop_teleop/tests/tools/replay_hand_contact.py --raw hand_raw.jsonl --session session.jsonl --execution hand_execution.jsonl --baseline /path/to/preserved/hand_retarget.py --output /tmp/contact-review --reference-stride 5
python laptop_teleop/tests/tools/benchmark_hand_contact.py --pico pico_motion.jsonl --raw hand_raw.jsonl --session session.jsonl --baseline /path/to/preserved/hand_retarget.py --output /tmp/contact-timing.json --repeats 3
```

两工具都保留 ACTIVE 前的原始历史；前者每 5 帧抽取一个高预算多初值参考解，并明确报告抽样数量及未找到可行解的样本。后者在隔离进程中回放原始 Body/双手到达节奏，复用实际掌面与右手计算，本地临时 UDP 端口仅验证发布延迟，不连接运行服务，也不模拟机器人动力学。自然张开候选是几何筛选，不能视为人工静止/自然张开标定。`fixtures/hand_contact/middle_boundary.json` 保存导致协调步长错误截断的最小数值回归样本，其预置状态是离线求解状态，不冒充实测机器人姿态。

实验分支的 `test_hand_vector_experiment.py` 使用隔离向量环境运行，验证真实 FK／mimic 雅可比、可达目标、限位及候选提交；普通环境缺少可选依赖时跳过。本次从原实验归档恢复既有用例，没有将它加入默认冒烟：

```bash
PYTHONPATH="$PWD/laptop_teleop/src:$PWD/teleoperation/src:$PWD/robot_link" \
  laptop_teleop/external/dex-vector-venv/bin/python -m pytest laptop_teleop/tests/test_hand_vector_experiment.py -q
```
