# 本机遥操作测试

- 根目录 `test_*.py`：有效的自动化回归。保留按功能划分的文件，避免为了文件数量合并不相关的安全检查。
- `fixtures/`：回归必需的固定样本及原生代码。`moving_recovery/` 从已保存现场会话提取 REACQUIRING 记录，来源与 SHA256 见 `provenance.json`；测试不再依赖可清理的运行日志。
- `tools/`：独立分析、回放、性能测量工具；不由生产入口导入，不在每次 pytest 中自动运行。工具之间通过包名引用，原根目录工具路径已迁移。
- `__pycache__/`：可再生缓存，可在测试结束后清理。

项目根目录执行：

```bash
bash laptop_teleop/scripts/test.sh
```

该入口覆盖本机、robot_link 和分区 IK 测试，仍保留已有两个 `-k` 排除项。成功不代表仓库全部硬件相关测试通过，也不代表实机已就绪。

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

新增测试应覆盖当前需求；若功能明确废弃，删除对应断言并在统一 [修改日志](../docs/08_CHANGELOG.md)记录依据。不得为了通过测试而删除仍有效的保护验证。

左手接触重定向可重复验收：

长时间验收前，先把所需输入快照保存在 `log/retained/<本次验收>/`，按现有规则添加 `.keep`；不要依赖普通运行日志在整个测试期间仍被保留。活动日志只做窗口副本，不移动或截断原文件。

```bash
python laptop_teleop/tests/tools/replay_hand_contact.py --raw hand_raw.jsonl --session session.jsonl --execution hand_execution.jsonl --baseline /path/to/preserved/hand_retarget.py --output /tmp/contact-review --reference-stride 5
python laptop_teleop/tests/tools/benchmark_hand_contact.py --pico pico_motion.jsonl --raw hand_raw.jsonl --session session.jsonl --baseline /path/to/preserved/hand_retarget.py --output /tmp/contact-timing.json --repeats 3
```

两工具都保留 ACTIVE 前的原始历史；前者每 5 帧抽取一个高预算多初值参考解，并明确报告抽样数量及未找到可行解的样本。后者在隔离进程中回放原始 Body/双手到达节奏，复用实际掌面与右手计算，本地临时 UDP 端口仅验证发布延迟，不连接运行服务，也不模拟机器人动力学。自然张开候选是几何筛选，不能视为人工静止/自然张开标定。`fixtures/hand_contact/middle_boundary.json` 保存导致协调步长错误截断的最小数值回归样本，其预置状态是离线求解状态，不冒充实测机器人姿态。
