# 当前遥操作文档

本目录描述当前源码中的 PICO 遥操作流程，不把历史现场状态或某次测试结果作为设备已就绪的证明。首次环境安装、依赖和配置入口集中在 [`../README.md`](../README.md#环境安装与配置)。

| 文档 | 用途 |
| --- | --- |
| [02_ROBOT_CONNECTION.md](02_ROBOT_CONNECTION.md) | 两机网络、SSH、认证密钥、同步与机器人服务启动。 |
| [03_OPERATION_RUNBOOK.md](03_OPERATION_RUNBOOK.md) | 每次运行的检查、预览、准备、标定、跟随和结束顺序；命令行替代入口。 |
| [04_TROUBLESHOOTING.md](04_TROUBLESHOOTING.md) | 按故障阶段定位输入、计算、反馈、手部和控制器问题；独立测量工具。 |
| [05_WEB_CONSOLE.md](05_WEB_CONSOLE.md) | 网页按钮、模式、骨架层、状态和恢复操作。 |
| [06_PICO_HAND_INPUT.md](06_PICO_HAND_INPUT.md) | Body/Hand 来源、新帧通知、时间戳、同步和转发语义。 |
| [08_CODE_AND_COMPUTATION.md](08_CODE_AND_COMPUTATION.md) | 当前源码职责、数据流、映射、IK、接近、首次接管和回零的实现。 |

网页日常使用先读 05，再按 03 检查；出错查 04。修改采集查 06，修改计算或机器人执行查 08。文档编号保留，便于现有链接继续使用。

旧环境文档已合入上级 README；旧右手 SDK/CAN 诊断中的通用结论合入 04，设计稿中的已实现流程合入 08。历史运行日志、现场照片、临时报告和外部 SDK 构建不放入本目录或 Git 提交。旧 SenseGlove 维护入口仍在脚本和原 [`teleoperation`](../../teleoperation/README.md) 中，当前 PICO 操作不需要启动它们。
