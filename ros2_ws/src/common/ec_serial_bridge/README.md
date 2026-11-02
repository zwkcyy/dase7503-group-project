

# ec_serial_bridge

树莓派侧 ROS 2 节点，把 EC 固件（ESP32-S3，`main@6985c8a`）的二进制 UART 协议与 NAV
`ec_adapter` 使用的话题连接起来。协议依据是 EC 仓 `docs/communication.md` 与
`src/communication/serial_link.cpp`、`serial_protocol.cpp`、`serial_telemetry.cpp`，
不是 README 的转述。

## 边界

本包只做编解码与转发，不做任何语义改造：

- 不产生保活指令。EC 侧 500 ms 命令看门狗是安全链的一环，桥不替 NAV 续命。
  唯一例外是 EC 明确要求的 **02 回执重发**：只有回执逾期才发、收到 `F0` 即停、
  到达 `lift_retry_max` 也停，因此它不是周期保活（见"02 是事件命令"一节）。
- 不自动发 `03 arm`、`04 estop`、`05 stop`。三者只通过服务显式调用。
- 不发未知文本映射。`lift_cmd` 收到无法识别的字符串时只计数并丢弃，不猜方向。
- 上行静默超过 `telemetry_timeout_s` 后停止发布，让 `ec_adapter` 判定数据过期，
  而不是把同一个旧值反复重发成"新鲜数据"。

## 文件职责

| 文件 | 职责 | 依赖 |
| --- | --- | --- |
| `ec_serial_bridge/framing.py` | CRC-16/Modbus、组帧、切帧、半包/粘包/坏帧重同步、超时丢弃残片 | 仅标准库 |
| `ec_serial_bridge/protocol.py` | 下行 01–07 编码，上行 81/82/83/84/F0 解码与长度/有限性校验 | 仅标准库 |
| `ec_serial_bridge/lift_retry.py` | 02 的 F0 回执重发状态机（纯逻辑，注入时间） | 仅标准库 |
| `ec_serial_bridge/transport.py` | pyserial 打开、重连、读、写；不解释任何字节 | pyserial |
| `ec_serial_bridge/node.py` | ROS 话题、服务、时间戳、超时、诊断状态 | rclpy + 上述三者 |
| `config/ec_bridge_20261102.yaml` | 默认参数与开项开关 | — |
| `launch/ec_serial_bridge.launch.py` | 单独启动桥（不含 `ec_adapter`） | — |
| `../tests/test_ec_serial_bridge.py` | 离线检查，覆盖切帧、编解码与 02 的 F0 重发状态机 | 仅标准库 |

分层是为了让 `scripts/check_nav.sh offline` 能在没有 pyserial 的云端
（CI 只装 `python3-opencv python3-numpy python3-yaml`）覆盖切帧逻辑。
编解码检查不能替代 ROS 节点联调、实板回执、升降完成和断线恢复检查。

**NAV 接入修复与真机操作步骤：** [EC 真机联调步骤](../../../../docs/NAV_EC_真机联调_20261102.md)。

## 话题与服务

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/ec/cmd_vel` | `geometry_msgs/Twist` | 转发为 01；`linear.y` 非零时 EC 会拒绝该帧 |
| 订阅 | `/ec/lift_cmd` | `std_msgs/String`（`lift_cmd_mode=text`）或 `UInt8` | 映射为 02 / 07 |
| 发布 | `/ec/wheel/odom` | `nav_msgs/Odometry` | 81 → odom→base_link，时间戳取 Pi 时钟 |
| 发布 | `/ec/imu/data_raw` | `sensor_msgs/Imu` | 82 原样转发，`orientation_covariance[0] = -1` |
| 发布 | `/ec/lift_state` | `std_msgs/UInt8` | 83，行为由 `lift_state_mode` 决定 |
| 发布 | `/ec_bridge/status` | `std_msgs/String` | JSON：链路、计数、CRC 错误、最后回执、时延 |
| 发布 | `/ec_bridge/lift_event` | `std_msgs/String` | 本地 session/request_id、方向、ROS 时间戳及 submitted/sent/accepted/cancelled/failed 事件；不改变串口协议 |
| 订阅 | `/ec_bridge/cancel_lift` | `std_msgs/Empty` | 取消待重发指令；不能停止已经开始的机械动作 |
| 服务 | `/ec_bridge/set_arm` | `std_srvs/SetBool` | 手动 arming |
| 服务 | `/ec_bridge/estop` | `std_srvs/SetBool` | 手动急停 |
| 服务 | `/ec_bridge/stop` | `std_srvs/Trigger` | 05 |
| 服务 | `/ec_bridge/lift_stop` | `std_srvs/Trigger` | 07；`enable_lift_stop=false` 时返回 `success=false`，不发帧 |
| 服务 | `/ec_bridge/request_status` | `std_srvs/Trigger` | 06 |

主启动入口同时更新 `ec_adapter`，默认加载 `ec_20261102.yaml` 的 `ec_state` 模式：
先取得新鲜的原端点，再发一次命令；只有本次请求收到 F0 接受事件，并且 83 状态由原端点
变到目标端点，才发布 NAV 的 `/lift_state` 完成值。旧 `ec_20261025.yaml` 保留人工确认模式。
EC 的端点状态来自固件计时，不是物理到位传感器。主启动入口禁止 `target_mirror` 和 `uint8`
搭配现有 String 适配器，以免产生假完成或消息类型不匹配。

## 参数

| 参数 | 默认 | 备注 |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | ESP32-S3 板载 USB CDC 一般是 ACM，4-pin 排针接 USB-UART 则是 USB0 |
| `baudrate` | `115200` | 与固件一致 |
| `resync_timeout_s` | `0.1` | 与固件 100 ms 残片超时对齐 |
| `telemetry_timeout_s` | `1.0` | 超过后停发上行 |
| `lift_cmd_mode` | `text` | `text`/`uint8`，对应 B3 两个方案 |
| `lift_ack_timeout_s` | `0.2` | 02 发出后等 `F0` 回执的窗口；超时即重发（EC 建议 200 ms） |
| `lift_retry_max` | `5` | 02 重发次数上限；`-1` 只取消次数上限，仍受总有效期限制；`0` 单发 |
| `lift_command_max_age_s` | `2.0` | 从首帧起计算的绝对有效期，过期不重发、不接受迟到回执 |
| `enable_lift_stop` | `false` | 服务、文本和枚举三个入口统一禁止 07；仍取消待重发指令 |
| `lift_state_mode` | `passthrough` | B2 已确认，生产固定 `passthrough`；另两个值仅诊断用 |
| `imu_frame_id` | `base_link` | 82 是芯片轴，未做任何旋转 |
| `pose_covariance_diagonal` | 全 0 | 未标定；0 表示未建模，不表示完美测量 |

## 测试

```bash
python3 -m unittest discover -s ros2_ws/src/common/tests -p 'test_ec_serial_bridge.py' -v
```

覆盖：文档示例帧 CRC（`AA 55 02 01 01 10 50 0D 0A`）往返、逐字节喂包、一次喂两帧、
前导噪声、CRC 错与尾错后恢复、超长 length、半包等待、残片超时丢弃并从其后恢复完整帧、
81/82 浮点往返与 NaN/Inf 拒绝、83 越界值拒绝、84 标志位、F0 接受/拒绝、长度不符拒绝，
以及 02 的 F0 回执重发状态机（窗口内不重发、超时重发、回执清除、拒绝终止、重发上限、
`0` 退回单发、`-1` 不限次数、非法参数拒绝）。原 PR #18 记录为 36 项通过。
该记录属于原实现；本次接入修复未新增或运行测试，不能沿用为当前修改的测试结果。

## 83 的语义（B2 已由 EC 确认）

EC 负责人 2026-11-02 明确：

- 状态帧按 10 Hz **持续发送**：机构在底端就一直发 0，在顶端就一直发 1，不带附加参数。
- 3000 ms 由 EC 自己计时（`config::liftTravelTimeMs`），树莓派不下发任何计时指令。
- **没有"未完成"编码**。收到 02 后 EC 先驱动舵机，等满 3000 ms 才把 0/1 翻转。

由此推出两条实现约束：

1. "动作完成"的唯一判据是**状态值翻转到目标值**，不存在额外完成位，也不是靠计时器
   在树莓派侧推算。桥用 `passthrough` 原样转发即可，`target_mirror` 与 `off` 仅供
   诊断，正常运行不要开。
2. 收到指令后的 3 s 内，状态**保持旧值**是正常现象，不是指令未生效。下游若以
   "目标序列号 + 短时间窗"判新鲜度，会与这套语义冲突。

## 05 / 07 语义变更（EC 2026-11-02 口头确认，固件尚未落地）

EC 负责人明确：

- **07（只停升降 PWM）取消。** 舵机类执行器没有中途停止的控制手段，固件不提供该能力。
- **05 语义收窄为"只停底盘"**，不再同时停升降 PWM。
- 收到 02 后 EC 一定把该动作走完，3000 ms 后更新状态交给导航。

因此本包当前实现与目标协议存在差异，等 EC 出新 commit 并更新 `docs/communication.md`
后一次改到位：

1. `/ec_bridge/lift_stop` 已加 `enable_lift_stop` 开关（默认 `false`）：关闭时服务返回
   `success=false` 并说明原因，不写 07 帧；文本/枚举入口使用同一个检查。
   `/ec_adapter/stop_lift` 转发桥的服务结果，最多等待 1 s，不把“发布请求”当成成功。
   `protocol.encode_lift_stop()` 保留，供固件
   恢复 07 时打开开关复用。
2. `encode_stop()` 明确记录升降 PWM 行为取决于固件版本；字节不变，不能据此保证升降会停。

在此之前，`6985c8a` 的 `communication.md` 仍写着 05 停两路、07 存在，本包按该版本
文档实现；两处差异以固件新版本为准。

### 02 是事件命令：单帧 + F0 回执重发（EC 2026-11-02 补充）

EC 负责人第二轮的完整口径：

- **01（底盘速度）保持周期发送**，建议 20–50 Hz，确保 EC 一直收到最新底盘控制目标。
- **02（升降）不要一直发**：等小车运动到指定位置后再发一帧。
- 发完 02 后设一个检查周期：**200 ms 内没收到 EC 回的 `F0`（确认帧）就重发这条 02**。
  周期可由我方自定义，200 ms 是 EC 给的参考值。

据此本包实现如下（原"收到即发一帧、不重发"的写法已废止）：

| 项 | 实现 |
| --- | --- |
| `/ec/lift_cmd` 到达 | 发一帧 02，登记为"待确认"，起算 `lift_ack_timeout_s`（默认 0.2 s） |
| 收到 `F0` 且 `command_type=0x02`、`result=0` | 清除待确认状态，不重发 |
| 收到 `F0` 但 `result≠0`（EC 拒绝） | 立即清除，不重发，记 `rejected_command` 与 error 日志 |
| 超时未收到回执 | 重发同一帧，重置窗口，计 `lift_retry`；重发次数上限 `lift_retry_max`（默认 5，`-1` 不限，`0` 退回单发） |
| 次数耗尽仍无回执 | 计 `lift_ack_lost`，打 error 日志，停止重发（不无限刷帧，避免掩盖链路故障） |
| 超过总有效期 | 同样停止重发并向适配器报告失败，即使 `lift_retry_max=-1` |
| 串口断线/换会话、有效上行超时、停止/急停/解除使能、任务结束或取消 | 清除旧请求和待重发状态；恢复连接只请求状态，必须重新发起指令 |

`/ec_bridge/status` 里可看到 `lift_ack_pending` 与 `lift_frames_sent`，实车调"到底有没有收到
回执"直接看这两个字段。

三点注意：

1. **重发只作用于 02**。01 由 NAV 按 20–50 Hz 持续发布，桥逐帧转发，不做去重、不做重发；
   05 / 04 / 03 仍是一次性服务调用。
2. **02 的"何时发"属于 NAV 责任**：等导航判定到达目标位姿后再发，桥不判断"到了没有"。
   如果 `ec_adapter` 仍按固定频率重发同一方向，桥会按事件命令逐条处理并各起一个确认窗口，
   所以上游仍应改成"到点发一次"。
3. **F0 的确认语义沿用 `6985c8a` 的 `docs/communication.md`**（`command_type` + `result`，
   2 字节）。若新固件把 02 的回执改成别的形式，必须同步改 `protocol.decode()` 与本节的判定。

## 尚未验证（需要硬件或 EC 答复）

1. 端口名与波特率在实板上的实际取值（ACM 还是 USB0）。
2. 82 的芯片轴与 `base_link` 的关系；`ec_adapter` 的 `imu_to_base_rpy_rad` 仍是
   `[0,0,0]`，属于暂定假设。
3. `lift_cmd` 的最终形态（文本还是枚举）与 B3 结论；当前默认沿用文本。
4. **04 急停是否同时切升降动力**（EC 两轮均未正面回答）。若 04 也只停底盘，升降启动后
   的 3 s 行程内没有任何软件中断手段，只能靠限流/超时/机械限位兜底，必须在实车上
   验证"升降中途拍急停"时执行器的真实反应。
5. 协方差未标定，`ekf_ec_10hz.yaml` 只用 twist 的 vx/vy/vyaw 与 IMU 的 yaw rate。
6. 02 在实板上是否真的回 `F0`（`command_type=0x02`）。若固件只对部分命令回执，桥会按
   `lift_retry_max` 重发到上限后报 `lift_ack_lost`——第一次上电必须看这个计数是否为 0。

