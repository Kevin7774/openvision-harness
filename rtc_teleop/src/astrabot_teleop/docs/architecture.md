# 设计边界与运行数据流

## 组件边界

`astrabot_rtc` 拥有 PeerConnection、Media Track、DataChannel 和 SDP/ICE；`astrabot_teleop` 只解释 label 为
`astrabot.teleop` 的已授权二进制 payload。两者通过 ROS typed message/service 连接，Teleop 不包含任何 WebRTC SDK
头文件。

## 授权链

```text
RTC DataChannel open
  → /astrabot/teleop/authorize_channel
  → 校验 purpose/resource/channel
  → base64url token 分段
  → 按 payload 原始 bytes 验 Ed25519
  → 校验 session/run/device/resource/expires_at
  → 原子消费 nonce
  → 注册唯一 writer
  → Authorized → Connected
```

任一步失败均返回 `allowed=false`。原始 token、payload JSON、signature、user id 和 nonce 不进入日志或状态 topic。
nonce 在 verifier 中于进程生命周期内只消费一次，不因 grant 到期而清理；缓存达到固定上限时 fail closed。当前活动
writer 对完全相同 token 的 service retry 通过 token SHA-256 指纹识别，不再次消费 nonce。writer 关闭后不会保留该幂等
窗口，旧 token 或使用相同 nonce 重新签发的 token 都必须拒绝，重连必须取得新 session/grant。

## 高频控制链

```text
RtcDataPacket callback
  → session/peer/channel 快速匹配
  → capacity-1 mailbox（latest wins）
  → 2 ms timer
  → protobuf + CRC
  → sequence/timestamp/replay
  → finite/quaternion/axis/skeleton
  → workspace + step + steady-time velocity/acceleration limit
  → deadman/mapping
  ├─ shadow → TeleopCommand(owner_epoch=0, shadow_only=true)
  └─ cpp
       → READY：仅保留 grant、RTC channel 与 Connected
       → ACTIVE DataCollectionRunContext
       → async Acquire owner（单 pending，40 ms 默认超时）
       → Connected → OwnerAcquired → Armed → Controlling
       → TeleopCommand(owner_epoch>0, shadow_only=false)
       → 50 ms 默认周期 Renew
```

只有完整通过的帧才刷新 watchdog。无效帧洪泛不会保持控制权；最后有效帧消失后 100～150 ms 内进入 Closed 并发布
stop shadow command，或在 cpp backend 主动 Release owner，由 arbitration 生成最终安全 stop。RTC connected 本身不会
触发 Acquire，更不会导致运动。

## Owner 异步生命周期

- 同一时刻最多一个 Acquire/Renew/Release pending request。
- operation id 与 generation 同时匹配才接受 callback；timeout、cancel、stop 或新 session 后的旧 callback 不能复活
  owner epoch。
- Acquire 成功前状态保持 `Connected`，生产 topic 无输出。
- Release 开始即撤销本地 epoch；即使服务不可用、响应丢失或超时，后续帧也不能继续携带旧 epoch。
- deadman release、watchdog、run complete/cancel/fail、peer close 和进程 stop 都进入 release/close 收尾。
- 服务不可用、Acquire/Renew 失败或 ACTIVE run 不匹配均 fail closed。READY 不会 Acquire/Renew，也不会接受生产 command。
- 同一 run 的 `ACTIVE → READY` 会清空待处理控制帧、重置 watchdog/映射、撤销本地 epoch 并请求 Release owner，但保留
  RTC peer、DataChannel 与已授权 session；再次进入 ACTIVE 后必须 Acquire 新 epoch。
- READY context 更新与 owner acquire/renew/expiry/timeout 失败使用同一控制转换锁。READY 已生效且完整 binding 仍匹配
  时，晚到 callback 只能幂等 pause/release；已有 Release pending 不会被重复 pause 取消，也不会调用 RTC close。
- `IDLE/COMPLETED/CANCELED/FAILED`、run/resource 变化或 publisher generation 变化才终止并关闭旧 session。

## Data Collection 状态上报链

```text
Teleop 内部状态投影为 connected/disconnected，或 binding 变化
  → 稳定 ReportTeleopStatus request 快照
  → 单 ROS pending + capacity-1 latest-wins
  → /astrabot/data_collection/report_teleop_status
  → Data Collection 校验当前 run 并分配 event_sequence
```

- `Connected/Armed/Controlling` 合并为 `connected`，`Stopping/Closed/Fault` 合并为 `disconnected`；
  `Idle/Authorized` 不扩散到平台。
- `last_sequence`、`sequence_gap_count`、reason 或同一连接组内的内部状态变化不生成新事件；投影状态或完整 binding
  变化才生成请求。
- timeout/reject 重试复用原始 `request_id`、`device_ts` 和完整请求体，满足 Data Collection 严格幂等契约。
- pending `connected` 可被 `disconnected` 抢占；重复 disconnect 会被合并。
- 服务不可用、拒绝、超时或晚到 callback 不会改变 Teleop 状态、owner 或命令发布；队列始终有界。
- shutdown 必须先 quiesce callback group 再 stop。强制进程退出可能取消尚未完成的 terminal 上报，因此安全停止只以
  arbitration/TTL/watchdog 为准，不能依赖平台状态事件。

## 时间域

- `TeleopFrame.header.timestamp_ms`：Quest wall clock，用于 stale/future 检查。源帧年龄默认最多 100 ms，且不得超过
  command TTL；生产验收还必须测量 Quest/机器人时钟偏差和应用层排队，不能仅靠该 wall-clock 检查推导 150 ms
  安全停止结论。
- `RtcDataPacket.receive_steady_time_ns`：RTC 在机器人本机接包时记录。
- `AuthorizeDataChannel.Response.expires_at`：Teleop 将 grant 的 Unix epoch 秒转换成剩余寿命后，返回机器人本机
  steady clock 绝对纳秒；RTC 直接按 steady clock 比较。转换后系统时钟跳变不改变已授权通道寿命。
- `TeleopCommand.valid_until_ns`：同一机器人 steady clock 域。
- `TeleopCommand.owner_epoch`：shadow 固定为 0；生产控制必须携带 arbitration owner epoch，隔离重新 acquire 前的旧帧。
- `right_gripper_valid/left_gripper_valid`：分别授权对应夹爪字段；生产模式下与该侧 SafetyFlags、tracking、re-grab
  和 deadman 共同 fail closed，不能仅凭 gripper 数值存在就执行。
- owner lease expiry：arbitration 响应中的机器人本机 steady clock 绝对纳秒；Teleop 不使用 wall clock 续租。
- watchdog：只使用 `std::chrono::steady_clock`，不受 NTP 或系统时钟跳变影响。

## 生产切换前的强制条件

1. Quest 使用 v2 unordered/20 ms channel，并通过丢包/乱序测试。
2. tracking-space 到机器人 base frame 的标定映射完成差分和 HIL 验收。
3. arbitration 实现 owner、TTL、run/session/resource 和 deadman 的第二道校验，并在 release/TTL/watchdog 时发布最终
   executed stop。
4. `shadow_only=false` 只能由 `cpp` backend 在有效 owner epoch 下产生；`cpp` 与 legacy writer 必须部署互斥。
5. ROS fake-service 故障注入、仿真、HIL、延迟测试、8 小时 soak 和 canary 全部通过；在此之前 robot_start 保持拒绝
   `cpp`。
6. grant claim 必须完成不可伪造的 peer identity 绑定；平台 grant TTL/续签必须覆盖预期的连续 Teleop session。
7. Quest 与机器人 wall clock 偏差、Quest 发送队列和 v2 DataChannel 20 ms 生命周期必须联合通过 HIL；100 ms
   `max_frame_age_ms` 只负责拒绝明显旧帧，不能单独证明“Quest 最后采样到安全停止小于 150 ms”。
