# 分布式遥操作开发与验收记录

## 2026-09-22：补充机器人接收目标后的执行说明

- 新增 [ROBOT_EXECUTION.md](ROBOT_EXECUTION.md)，按当前代码说明收包校验、最新目标缓存、状态门禁、
  URDF/电机坐标转换、反馈约束轨迹、MOVE_J/CAN、手部本机 ROS 桥、双侧协调及故障处理。
- 明确网络目标、实际下发中间目标、实测反馈的区别；`accepted_seq` 不代表下发/到位，
  当前 state/网关日志未完整导出每周期中间命令。列明当前限速、碰撞开关和各类超时。
- README、PROTOCOL、运行逻辑及手册增加入口。本次仅修改文档，核对相关函数与 YAML，
  检查本地文档链接和格式；没有修改控制代码、运动参数或操作真机。


## 2026-09-22：增加双臂＋双手联合选项

在同一 UDP 通信框架新增 `--side both`（包含双手），保持单臂入口、原运动参数与协议 v1 单侧格式。
新增内容：

- 双侧合同、四部件整包目标校验、`RobotClient.send_dual_target`、双侧新鲜度/使能保持接入示例。
- 一个控制会话和执行器统一协调两臂；任何一侧失败停止两臂并冻结双手，停止/失能不会因第一侧异常跳过第二侧。
- 两侧首目标和反馈先预检查，再依次使能；第二臂使能期间检查先使能臂保持状态，双手最后开放发送。
- 两臂共享一个双手反馈接收器；新增手部批量物理目标入口，一次限频后整包发送双手，避免右手被共享限频器跳过。
- 双侧反馈分组返回，先采样侧累计等待时间；拒绝左右臂配置同一 CAN 通道。
- 双侧 `z` 明确拒绝，不调用单臂回零：当前回零几何没有完整对侧臂/手扫掠检查。单侧 `z` 不变。

验证：沿用下文完整测试命令，**102 passed, 26 subtests passed**。新增覆盖双侧真实本机 UDP 收发、
四部件模拟运动、单侧无效包整包拒绝、部分使能失败、先使能臂保持故障、第二侧反馈失效、单侧停止发送失败、
双手合并发送与限频、任一路人体输入过期、共享接收器和 CAN 配置冲突。未启动硬件、未实机运动。

边界：两臂硬件命令依次发送，不承诺硬件同步或稳定 50 Hz；不新增双臂互撞保护；故障排障后需重新启动双侧网关。
笔记本完整采集/IK/网页入口及双机实机验收仍待后续完成。以下“首版”记录保留为历史基线，单侧范围以本次更新为准。

## 2026-09-22：通信与机器人执行边界首版

### 需求和实现

机器人主机退出人体采集、IK、网页渲染工作，笔记本（Ubuntu/Linux）接管这些任务。
本次提供通信基础及真实驱动适配；保留旧入口供现有模式使用，不在未接入笔记本应用前替换运行方式。

| 文件 | 本次工作 |
| --- | --- |
| `esrobo_link/protocol.py` | 有界 JSON/HMAC、版本/数值/侧别/限位校验 |
| `esrobo_link/session.py` | 独占会话、递增序号、机器人令牌有效期、拒绝过期/重复包 |
| `esrobo_link/gateway.py` | UDP 状态通道、本机串行执行、闭锁/取消/显式恢复、JSONL 诊断 |
| `esrobo_link/backends.py` | 默认无硬件模拟及单臂＋可选同侧手适配；复用原有参数与保护 |
| `esrobo_link/client.py` / `monitor.py` | 笔记本 SDK 和只读通信检查 |
| `examples/laptop_integration.py` | 新求解帧、新鲜度、使能保持目标的接入示例 |
| `esrobo_link/camera.py` | ROS JPEG 原样转发或 RealSense 直采，独立最新帧 HTTP 流 |
| 原 `nero_driver.py` | 使能和力矩基线新增可选取消回调；原调用兼容 |
| 原 `linker_hand_driver.py` | 只读反馈及已映射物理十轴目标入口，继续应用本地安全限制 |

关键修正：每次下发前重新获取关节反馈，防止使能后复用旧控制周期；过期使能状态返回未知；
旧目标流到期必须先闭锁，再接受后来包；使能部分完成后失败也请求电子停止。

### 离线验证

通信独立测试（系统 Python，无机器人依赖）：

```bash
PYTHONPATH=robot_link python3 -m unittest discover -s robot_link/tests -v
```

17 项通过，含本机真实 UDP socket/SDK/网关收发、超时闭锁、状态日志及真实 HTTP multipart 响应。

适配与既有驱动回归（测试用假反馈/拦截 CAN 发送，不连接设备）：

```bash
PYTHONPATH=teleoperation:teleoperation/src:teleoperation/tests:robot_link \
  /home/esrobo/miniconda3/envs/teleop_esrobo/bin/python -m pytest -q \
  robot_link/tests \
  teleoperation/tests/test_robot_link_adapter.py \
  teleoperation/tests/test_nero_motion_stream.py \
  teleoperation/tests/test_runtime_feedback.py \
  teleoperation/tests/test_safety.py::NeroSingleArmTests \
  teleoperation/tests/test_safety.py::HandMappingTests
```

结果：**82 passed, 22 subtests passed**。覆盖新鲜周期反馈、过期使能状态、使能取消、手部活动轴掩码/
限速/反馈失效以及现有左右臂发送模式回归。初次测试发现测试配置未等待手部发布周期，修正假时序后通过；
没有修改实际限速去迁就测试。CLI `--help` 及 `git diff --check` 通过。

### 尚待现场和后续开发验证

- 笔记本克隆仓库后，将真实 PICO/SenseGlove 标定、重定向、IK 和网页接入 SDK；手目标需完成与原映射一致的物理十轴转换。
- 两台机器实际网络的延迟、丢包及负载表现；本机 socket 测试不代表跨机链路已验收。
- 核对 URDF/配置合同、手部反馈零位与几何标定，验证实机反馈，再做有人托扶的小幅运动及断流停止检查。
- 真相机采集、双机观看、CPU/带宽及长时间稳定性。本轮未启用相机或机器人硬件。
- 同时双臂、单手独立网关、深度/内参传输暂不在首版实现范围；当前网关整机互斥。
- 操作系统崩溃根因仍未证明；卸载上层计算不等同于修复硬件/内核问题。

运行产生的 `log/gateway_*.jsonl` 留在本机，不提交仓库。协议变更更新 PROTOCOL，运行方式更新 README，
每次验收在本文件追加时间、代码版本、所用后端、网络条件、结果及未通过项；不要把模拟结果写成真机通过。
