# robot_link 通信协议 v1

当前 v1 增加 `side="both"` 的可选双侧合同/目标格式，保留原单侧格式；旧客户端不能控制双侧端点。

实现依据：[protocol.py](esrobo_link/protocol.py)、[session.py](esrobo_link/session.py)、[gateway.py](esrobo_link/gateway.py)。本文件随协议实现更新；不兼容修改必须提升 `v`。

协议字段通过检查后如何生成本周期运动指令、如何下发和返回反馈，见
[机器人端执行流程](ROBOT_EXECUTION.md)。该文区分网络目标、限速后命令和真实反馈，也说明双侧失败的处理。

## 1. 通道与数据流

| 通道 | 地址/频率 | 数据 |
| --- | --- | --- |
| 关节控制 | 机器人 UDP 16000；笔记本临时 UDP 端口 | hello、target、hold、stop |
| 机器人反馈 | 回复已认证客户端源 IP/端口，目标约 50 Hz | state、合同、实测关节、故障 |
| 本机灵巧手桥 | loopback UDP 15051/配置反馈端口 | 现有 ROS/CAN 桥，不是笔记本接口 |
| 图像 | 机器人 loopback TCP 18080，经 SSH 转发 | HTTP multipart JPEG，默认采集 15 fps |

控制循环目标 50 Hz；机器人操作耗时会降低实际频率，不作硬实时承诺。网络收发线程独立于串行硬件执行线程。只保留最近有效目标，禁止按队列播放过时目标。

## 2. 包封装

UTF-8 JSON，整个 UDP datagram 不超过 **8192 bytes**。每包为：

```json
{"body":"<内部 JSON 字符串>","mac":"<64 位小写十六进制 HMAC-SHA256>"}
```

`mac = HMAC_SHA256(shared_key, body.encode('utf-8'))`。内部 JSON 按键排序、无多余空格、禁止 NaN/Infinity；内部固定 `v:1`。认证的是 body 的原始 UTF-8 字节；不可解析后重排再校验。密钥读取去掉前后空白，至少 32 bytes；推荐生成 32 随机字节的 hex 文本，两端保持相同文本。HMAC 不提供保密性。

## 3. 握手与配置合同

笔记本首次发送：

```json
{"v":1,"type":"hello","client_nonce":"<每次新客户端生成的32字符随机串>"}
```

机器人选定唯一源地址/端口、生成新 `session` 并返回 `state`。相同 hello 幂等，不续运动有效期。ACTIVE/ARMING/RETURNING 时拒绝新客户端；空闲或故障状态也要旧客户端至少 1 秒未发送有效指令才允许替换。只读 monitor 使用相同会话机制，不支持旁路多观察者。

`contract` 包含：`id`、`side`、`with_hand`、`arm_unit`、`hand_unit`、`arm_order`、`hand_order`、`lower_rad`、`upper_rad`、`physical_direction`、`physical_offset_rad`、`velocity_rad_s`、`acceleration_rad_s2`。模拟合同只含测试必需字段。
真实 `id` 为配置及 URDF 内容 SHA256 派生摘要，覆盖 robot/hand/IK/末端速度/侧别；不是现场几何或固件校验认证，外部 meshes 和标定现场真实性仍需独立核对。

笔记本应固定经核对的 `expected_contract_id`，不得只收到新合同就认为适配已完成。物理映射在机器人执行，网络始终使用 URDF 弧度。

## 4. 目标 target

```json
{
  "v":1,"type":"target","session":"<机器人会话>","seq":1,
  "lease":"<最近 state 的 lease>","contract_id":"<已核对合同摘要>",
  "side":"left",
  "arm_urdf_rad":[0,0,0,0,0,0,0],
  "hand_unit":[255,255,255,255,255,255,128,128,128,128]
}
```

此为字段示例，不是可直接执行的启动姿态。首目标必须来自实测反馈。

- `seq`：整数、非布尔，`0 <= seq < 2^53`，本会话严格递增。重复、乱序拒绝，不续有效期。
- `arm_urdf_rad`：7 个有限普通数字，选定侧关节顺序，必须位于合同限位。
- `hand_unit`：带手时 10 个 0–255 整数；不带手时为 null 或省略。
- 手部物理轴顺序：`thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch, index_mcp_roll, ring_mcp_roll, pinky_mcp_roll, thumb_cmc_roll`。
- 两种数据同包接收，但执行为机械臂后灵巧手的串行调用，**不是硬件同步或事务提交**。一侧发送失败会闭锁，已发出的另一侧指令无法撤销。
- 数组长度、数值、限位、侧别、合同、认证或有效期任一不符时拒绝整包；不会把错误字段默认为零。

### 有效期为什么不用笔记本时间戳

机器人每次 `state` 生成随机 `lease`，记录其本机单调时钟 `issued`，保留最近 32 个。收到目标要求 `0 <= now-issued < 0.2 s`；执行截止时间仍是 **issued+0.2**，不是收到目标后再给 0.2 秒。这同时覆盖反馈去程和指令回程延迟，不要求两台机器墙上时钟同步。

ACTIVE/ARMING 在旧流过期后先故障闭锁，再处理后来包，禁止迟到包恢复运动。客户端必须每个新求解帧读取最新 state 并发一次目标，不能自己生成 lease。该机制只能证明网络往返新鲜度，**不能证明人体传感器新鲜**；笔记本仍需检查 PICO/手套采样及求解输入年龄。

## 5. 受控保持 hold 与停止 stop

单臂纯机械臂模式在已进入 `ACTIVE` 后，若 IK 确认是可恢复的关节位置限位，可发送 `hold`：

```json
{"v":1,"type":"hold","session":"<会话>","seq":2,"lease":"<有效令牌>","contract_id":"<已核对合同摘要>","side":"right"}
```

`hold` 不含关节角或手部目标，网关调用机械臂驱动的受检减速保持。它沿用递增序号、独占会话和 200 ms 令牌有效期；本机只在每个全新、完整且未过期的 PICO 帧计算后发送，输入丢失仍会触发故障。双臂、带灵巧手和仅灵巧手模式拒绝 `hold`。回到可达范围并连续满足几何误差阈值后，客户端重新发送 `target`。

```json
{"v":1,"type":"stop","session":"<会话>","seq":2,"lease":"<有效令牌>"}
```

stop 与 target/hold 使用同一序号空间和认证。没有网络 enable/disable/return 指令，避免计算程序重连后自动改变执行状态。远程 stop 丢失或无法发送时，跟随有效期耗尽仍会触发停止；停止请求不等于物理停止证明。

## 6. 状态 state

| 字段 | 含义 |
| --- | --- |
| `session`, `client_nonce`, `state_seq` | 会话、客户端随机串、机器人状态递增序号 |
| `lease`, `lease_ttl_s` | 最近令牌与有效期（默认 0.2 秒） |
| `accepted_seq` | 最后通过协议校验的指令序号，不是执行/到位确认 |
| `target_remaining_s` | 发 state 时当前目标剩余有效期 |
| `contract` | 当前模型与配置合同 |
| `mode`, `reason` | IDLE / ARMING / ACTIVE / FAULT / RETURNING 及说明 |
| `feedback.arm_urdf_rad` | 七轴实测 URDF 弧度；不可用为 null |
| `feedback.arm_feedback_age_s` | 采样时关节数据年龄 |
| `feedback.enable_states` | 七轴新鲜使能反馈；未知为 null |
| `feedback.controller_fault`, `feedback.driver_fault` | 控制器检查结果及本机驱动闭锁原因 |
| `feedback.hand.position_unit`, `.age_s` | 手部十轴实测值和年龄；未启用手则 hand=null |
| `feedback_cache_age_s` | 状态采样后缓存时间；长时间使能/回零期间会增长 |
| `stop_send_returned` | null/true/false；仅停止调用是否返回，非失能证明 |
| `rejected_packets`, `last_rejection` | 协议拒绝累计数及最近原因 |
| `last_return` | 最近回零结构化结果：returned、disabled、stage、reason 等 |

判断反馈年龄需相加“采样年龄 + 缓存年龄”，再考虑网络延迟，不能把新收到的 state 当新关节反馈。
SDK 会丢弃乱序 state，不自动重放目标；机器人重启或会话变化需要新建客户端和现场重新使能。

## 7. 执行状态与恢复

IDLE 接收目标但不运动 → 机器人终端 `e` → ARMING 本机预检查、保持实测姿态使能、力矩基线 → ACTIVE 原有限速跟随。
任何执行错误/目标流过期 → FAULT，取消运动并请求电子停止，不自动回零、不自动去除电机支撑。
人工排障后 `z` → RETURNING，沿用原避碰回零，成功且确认失能后 IDLE；失败仍 FAULT。
回零是本机独立流程，不依赖笔记本持续目标流，但本机 `x/q/d` 仍可取消。

硬件调用串行；网络线程只设置取消事件。底层阻塞 CAN/SDK 调用返回前，不能保证电子停止已经发出；进程/内核卡死不受此软件看门狗保护。机器人端没有图形前端，但仍承担原闭环保护检查。

## 8. 图像帧

`GET /stream.mjpg`，`multipart/x-mixed-replace; boundary=frame`。每帧头：

- `Content-Type: image/jpeg`，`Content-Length`；单帧上限 4 MiB。
- `X-Frame-Sequence`：当前相机进程递增帧号。
- `X-Capture-Unix-Ns`：直采为机器人主机接收帧时间；ROS 为原消息时间（可能是仿真时钟）。
- `X-Frame-Id`：净化换行后的坐标帧名；直采为 `color_host_arrival`。

只保留最新 JPEG；慢观看端跳帧，无全历史队列。旧帧超过 1 秒拒绝发送、无新帧 2 秒结束连接；客户端必须自行显示“图像过期”，不能用最后静止画面判断机器人停止。

## 9. 双臂＋双手扩展（2026-09-22）

### 合同和目标

双侧 `contract.side="both"`、`with_hand=true`、`return_supported=false`，`contract.sides.left/right`
分别保留完整单侧合同。总 `id` 为两个单侧合同的 SHA256 派生摘要。不要取某一侧 id 作为整包合同 id。

```json
{
  "v":1,"type":"target","session":"<会话>","seq":1,
  "lease":"<令牌>","side":"both","contract_id":"<双侧合同摘要>",
  "targets":{
    "left":{"arm_urdf_rad":[0,0,0,0,0,0,0],"hand_unit":[128,128,128,128,128,128,128,128,128,128]},
    "right":{"arm_urdf_rad":[0,0,0,0,0,0,0],"hand_unit":[128,128,128,128,128,128,128,128,128,128]}
  }
}
```

示例数值不是启动指令。实测保持姿态才是首目标。两组臂/手必须同时出现，各自按本侧限位检查，任何一项错误拒绝整包；
不更新序号、目标或截止时间。单侧不能发到双侧端点；双侧不能发到单侧端点。版本、认证、8192-byte 上限、
200 ms 令牌有效期及独占会话规则不变。

### 双侧状态与执行

`feedback.sides.left/right` 各含单侧反馈字段；`feedback.last_operation` 记录最近 stop/disable 的各侧结果。
先采样的一侧额外累计等待另一侧采样的时间，客户端还须叠加外层 `feedback_cache_age_s`。
`accepted_seq` 是整包接收序号，不意味着四个部件全部执行成功。

一个执行器先验证两侧反馈，然后串行发两臂命令，最后通过共享手驱动整包发送双手命令；不存在网络层部分提交，
但硬件层无法原子提交。若左臂已发送而右臂失败，不能撤销已发左臂目标，只能立即请求停止两臂、冻结两手并闭锁。
停止或失能时，不因第一侧 CAN 异常跳过第二侧；`stop_send_returned=true` 仍不表示硬件已确认停止。

两臂使能依次进行，先使能侧在另一侧准备期间继续检查反馈/控制器/力矩；双手只在两臂都就绪后开放。
所有四路人体输入的新鲜度必须由笔记本独立检查，任一过期停止整包发送，不能用新左侧帧给旧右侧目标续期。

### 双侧回零边界

现有单臂回零碰撞检查未覆盖对侧机械臂和手，双侧网关因此在接收本机 `z` 时立即拒绝，状态不进入 RETURNING，
不发送回零目标；不将两条单臂避碰路径组合后冒充双臂安全回零。故障不因重连解锁；须现场排障后重启。
双侧遥操作本身不新增双臂之间实时碰撞检查，保留原有限位、反馈、力矩和躯干检查策略。
