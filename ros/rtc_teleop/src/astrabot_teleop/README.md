# astrabot_teleop

`astrabot_teleop` 是端侧遥操应用组件。它消费公共组件 `astrabot_rtc` 输出的通用 peer/DataChannel 数据，完成
Teleop grant、`TeleopFrame`、session、deadman、watchdog、READY/ACTIVE run context 和 arbitration owner lease 校验，
按 `disabled | shadow | legacy_mpc | cpp` backend 输出命令。

它不是 WebRTC 组件，也不是执行器驱动。RTC、视频编码、SDP/ICE、AI/Dora、Recorder 和最终运动仲裁都不属于本仓库。

## 依赖方向

```text
Quest
  ├── video Media Track ───────────────> astrabot_rtc
  └── astrabot.teleop DataChannel
                              │
                              ▼
                        astrabot_rtc
                              │ RtcDataPacket / RtcPeerEvent
                              ▼
                      astrabot_teleop
                              │ TeleopCommand
                              ▼
                  astrabot_arbitration typed ingress
```

默认、`shadow` 和 `cpp` backend 不发布旧 `/reference/pose` 或执行器 topic。`legacy_mpc` 是明确的迁移期例外：
它向旧仲裁入口发布双臂 JSON，并向现有普通夹爪 actuator topic 发布 `Float64`。`cpp` backend 通过有界异步服务获取、
续租和释放 arbitration owner；最终限幅、owner 二次校验和执行仍由 `astrabot_arbitration` 完成。

## 已实现能力

- 原样引入现有 `TeleopFrame.proto`，构建时用系统 `protoc` 生成 C++ binding。
- 固定唯一 `astrabot.teleop` 契约：`ordered=false`、`max_packet_lifetime_ms=20`、
  `max_payload_bytes=16384`。其他 label 或可靠性参数全部拒绝，不提供版本选择或兼容回退。
- Ed25519 grant 验签、key rotation、字段绑定和有界 nonce replay cache；每个 nonce 在进程生命周期内只允许由 verifier
  原子消费一次，不因旧 grant 到期而删除，缓存满时 fail closed。
  仅当前 writer binding 仍活动时，RTC 对完全相同 token 的授权 service 重试可通过 SHA-256 指纹幂等返回；session 关闭后
  相同 token/nonce 再次出现一律视为 replay，重连必须取得新 grant。
- payload 大小、protobuf、未知字段、CRC、sequence、timestamp、NaN/Inf、四元数和 OpenXR 关节数校验。源帧年龄
  默认上限为 100 ms，且配置必须不大于 command TTL，避免应用层旧帧重新喂活 watchdog。
- `shadow` backend 可用于观测历史帧；产生真实控制的 `legacy_mpc` 与 `cpp` backend 要求 `safety_present`，并分别使用
  `is_safe_right/is_safe_left/is_active_head` 门控对应位姿与 deadman。标志缺失或为 false 时对应控制自由度 fail closed。
- `TeleopCommand` 使用独立的 `right_gripper_valid/left_gripper_valid`。生产链路中某侧 tracking、re-grab 或
  deadman 失效时，该侧机械臂与夹爪都会失效；无效侧也不会推进夹爪 toggle 锁存状态。
- 基于 RTC steady receive timestamp 的 workspace、单帧步长、位置速度和位置加速度限制；任一手臂/头部失败时整帧不提交新基线。
- 容量为 1 的 latest-wins mailbox；过载覆盖旧控制帧，不重放历史动作。
- 显式 `Idle → Authorized → Connected → Armed → Controlling` 状态机。Grant 通过只进入 `Authorized`；只有收到与
  session/peer/run/resource/channel 全绑定一致的 `state=CONNECTED、reason_code=data_channel_open` 事件后才进入
  `Connected`。普通 PeerConnection connected 事件不能打开控制链路。
- 100～150 ms 可配置 watchdog；它从真实 DataChannel open 进入 `Connected` 时开始计时，授权后但尚未 open 的 SDP/ICE/
  DTLS/SCTP 建连阶段不会被误判为控制帧超时。DataChannel close 或 peer 断开会立即停止活动 session。
- run/session/resource/peer/channel 完整绑定；同一进程最多一个 writer。
- `cpp` backend 只接受 `astrabot.teleop`，订阅 transient-local READY/ACTIVE run context。READY 只允许 grant、RTC
  DataChannel 和 `Connected` 上下文存在；只有 ACTIVE 才允许 Acquire/Renew owner、进入 `Armed/Controlling` 或发布生产
  命令。`ACTIVE → READY` 会立即撤销本地控制、重置 watchdog/映射并请求释放 owner，但保留 RTC peer 与授权 session。
- run context 只在相同非空 `publisher_generation` 内校验 `context_version` 单调；发布代次变化会主动关闭旧 Teleop
  session、释放 owner，并以新 generation 的版本重新建立上下文。最近 32 个已退休 generation 使用 FIFO 有界保存；旧
  publisher 的迟到或 transient-local 样本再次出现时按双 publisher 故障处理，不切回旧代次，并等待当前 generation 的
  更高版本恢复。相同 generation 的版本回退或同版本冲突也会保持 fail closed，绝不根据“当前无 session/owner”猜测
  Data Collection 已重启。新 publisher 启动时允许 `STATE_IDLE` 携带空 `run_id`，该样本也会立即完成代次切换并停止旧
  session；除 `STATE_IDLE` 外的状态仍必须携带非空 `run_id`。
- owner TTL 默认 150 ms、renew 周期 50 ms、服务超时 40 ms；单 pending、旧 callback generation 隔离，release
  开始即本地撤销 epoch。Acquire/Renew/Release 不可用、超时或失败时 fail closed。
- READY 切换与 acquire/renew callback、owner TTL、pending timeout 通过同一控制转换锁串行化；完整授权 binding 仍处于
  READY 时，这些晚到失败只会 pause/release 并保持 `Connected`，不会进入 Fault 或请求关闭 RTC。
- 通过 `/astrabot/data_collection/report_teleop_status` 非阻塞上报低频状态事实，由 Data Collection 分配平台
  `event_sequence`；服务故障、拒绝或超时只产生有界重试与告警，不改变控制状态。

## 历史 CRC 兼容

Proto 注释规定 CRC 覆盖序列化 `DataBody bytes`，但当前 Quest 的 `FrameManager.CalculateCRC32()` 实际覆盖
“只包含 `data_body` 的 `TeleopFrame` envelope”。Codec 同时接受两者，并通过 `CrcVariant` 区分；除此之外的 CRC
一律拒绝。Quest 修正并完成 golden 验收后可移除 legacy 分支。

## ROS 接口

输入：

```text
/astrabot/rtc/data_channel/received  astrabot_rtc/msg/RtcDataPacket  BestEffort KeepLast(1)
/astrabot/rtc/peer_event             astrabot_rtc/msg/RtcPeerEvent   Reliable KeepLast(32)
/astrabot/data_collection/run_context astrabot_data_interfaces/msg/DataCollectionRunContext
                                     Reliable TransientLocal KeepLast(1)（仅 cpp）
```

服务：

```text
/astrabot/teleop/authorize_channel   astrabot_rtc/srv/AuthorizeDataChannel
/astrabot/rtc/close_peer             astrabot_rtc/srv/CloseRtcPeer（client）
/astrabot/arbitration/acquire_owner  astrabot_teleop/srv/AcquireControlOwner（client，仅 cpp）
/astrabot/arbitration/renew_owner    astrabot_teleop/srv/RenewControlOwner（client，仅 cpp）
/astrabot/arbitration/release_owner  astrabot_teleop/srv/ReleaseControlOwner（client，仅 cpp）
/astrabot/data_collection/report_teleop_status
                                     astrabot_data_interfaces/srv/ReportTeleopStatus（client）
```

输出：

```text
/astrabot/teleop/shadow_command      astrabot_teleop/msg/TeleopCommand BestEffort KeepLast(1)
/astrabot/teleop/command             astrabot_teleop/msg/TeleopCommand BestEffort KeepLast(1)（仅 cpp）
/astrabot/teleop/session_status      astrabot_teleop/msg/TeleopSessionStatus Reliable KeepLast(32)
```

`TeleopCommand.valid_until_ns` 和 `receive_steady_time_ns` 均使用机器人本机 steady clock；不得与 wall clock 比较。
`TeleopCommand.owner_epoch` 在 shadow backend 固定为 `0`；生产 cpp backend 必须填入 arbitration 返回的真实 epoch，
并设置 `shadow_only=false`，防止同一 binding 重新 acquire 后的旧帧跨 epoch replay。cpp 无有效 epoch 时不发布。

`AuthorizeDataChannel.Response.expires_at` 同样固定为机器人本机 `steady_clock` 绝对纳秒。grant wire
中的 `expires_at` 是 Unix epoch 秒，Teleop 只在授权边界按剩余寿命换算一次；RTC 不得把响应值当 Unix 时间。

控制权 schema 位于本包的 `AcquireControlOwner.srv`、`RenewControlOwner.srv` 和 `ReleaseControlOwner.srv`，由
arbitration 实现服务、Teleop 作为 client。owner 的过期点使用机器人本机 steady clock；same-binding
acquire/release 重试必须幂等，unknown、expired 或旧 epoch 必须 fail closed。deadman release、watchdog、run end、peer
close 和 stop 都会主动释放 owner；真正的安全 stop 由 arbitration 在 release/TTL/watchdog 路径生成。

状态上报把内部状态投影为平台契约的两种连接事实：`Connected/Armed/Controlling → connected`，
`Stopping/Closed/Fault → disconnected`；`Idle/Authorized` 不上报。只有投影状态或完整
`session/run/resource/peer/channel` binding 变化才生成新请求，reason、`last_sequence` 与 `sequence_gap_count` 单独变化
不会制造平台事件。同一逻辑请求重试时复用完整 request 快照。内部只有一个 ROS pending request 与一个容量为 1 的
latest-wins 后备状态；disconnect 可抢占尚未完成的 connect，timeout 后晚到 callback 由 operation generation 和 weak
ownership 拒绝。

## 配置

样例位于 `config/teleop.yaml`。必须配置：

```yaml
device_id: robot-001
grant_key_ids: [teleop-key-2026-01]
grant_public_keys: [BASE64_ENCODED_32_BYTE_ED25519_PUBLIC_KEY]
```

公钥支持 32-byte Ed25519 raw key 的标准 Base64（可带 `base64:` 前缀）或 PKCS#8 public PEM。私钥不得部署到机器人。
`grant_key_ids` 和 `grant_public_keys` 按数组索引配对，重复 key id 会导致启动失败。

`backend` 允许：

- `disabled`：保留节点和状态接口，授权始终返回 `teleop_disabled`。
- `shadow`：执行完整安全链，只发影子命令。
- `cpp`：READY 或 ACTIVE run context 均可建立 Teleop v2 授权 session；只有 ACTIVE 加有效 owner lease 才向
  `/astrabot/teleop/command` 发布生产 typed command。episode 的 `ACTIVE → READY` 会 stop/release owner，但不会关闭
  run-level RTC/Teleop session；重新进入 ACTIVE 后必须获取新的 owner epoch。
- `legacy_mpc`：通过 `wbmpc_remote_ctrl` 接入现有 `/reference/pose → astrabot_arbitration → /reference/cmd`
  链路，只控制双臂和普通夹爪。它不输出底盘、头部或灵巧手，也不使用 typed owner 服务；必须与旧 remote writer
  互斥，并依赖 `astrabot_arbitration` 的 120 ms remote watchdog 完成掉线 hold。

`legacy_mpc` 默认输出：

```text
/reference/pose                                  std_msgs/msg/String
/rm_left/rm_driver/teleop_gripper_float          std_msgs/msg/Float64
/rm_right/rm_driver/teleop_gripper_float         std_msgs/msg/Float64
```

deadman 释放、watchdog、RTC 断开、session 关闭或节点 stop 时，本节点会使用新鲜 `/ee/pose` 发布当前双臂 hold；若本地
位姿不可用，则保持 fail closed，并由旧 arbitration 的独立 watchdog 做第二道保护。该 backend 只用于最小迁移和回滚，
同一机器人上不得同时运行另一个 `/reference/pose` remote writer。

状态上报默认配置为：

```yaml
report_teleop_status_service: /astrabot/data_collection/report_teleop_status
status_report_service_timeout_ms: 100
status_report_retry_period_ms: 1000
status_report_poll_period_ms: 10
```

`retry_period` 不得短于 service timeout，poll 周期不得长于 service timeout。Data Collection 不在线时控制链继续运行，
后备状态按 latest-wins 合并，避免无界排队。

位姿运动边界默认为 `max_position_step_m=0.25`、`max_position_velocity_mps=1.0`和
`max_position_acceleration_mps2=5.0`。速度/加速度使用 `RtcDataPacket.receive_steady_time_ns` 计算，不信任 Quest wall
clock；进入 HIL 前必须根据标定后的 robot base frame 和执行器能力收紧这些默认值。

虽然二进制已实现 `cpp`，当前 `robot_start` 部署 Gate 仍只开放 `disabled | shadow`。完成仿真、HIL 和 canary 前，
不得在量产启动模板中启用 `cpp`。

## 构建与测试

仓库规定正式验证必须在 Docker 内执行：

```bash
./scripts/docker_build_and_test.sh
```

构建产物默认写入机器人工作区根目录下的 `.colcon/astrabot_teleop/`，不污染本包源码树。可通过
`ASTRABOT_TELEOP_NATIVE_BUILD_ROOT` 与 `ASTRABOT_TELEOP_CROSS_BUILD_ROOT` 覆盖。

该脚本挂载同一 robot workspace 下的 `astrabot_rtc`，完成 Jazzy native 构建、单测、格式检查、no-exceptions
检查，并在 `/opt/astrabot_sdk_2404` 可用时尝试 THOR ARM64 交叉构建。

可通过 `ASTRABOT_TELEOP_RUN_ARM64=0|1|auto` 明确控制交叉 Gate：`1` 在 SDK 不存在时直接失败，`0` 只跑 native，
`auto` 保持本地开发时“有 SDK 就交叉”的行为。包内 `.gitlab-ci.yml` 将 native 与 ARM64 job 分开；设置
`ASTRABOT_TELEOP_RUN_ARM64_GATE=1` 后 ARM64 job 成为强制 Gate。

native 测试还包含 ROS 进程内边界回归：`disabled` backend 的授权 service 必须返回 `teleop_disabled`，伪造 RTC packet
不能产生 shadow/production 命令，重复 `start()/stop()` 必须保持 one-shot fail-closed 语义。该测试不模拟真实 owner 成功，
因此不会产生任何运动输出。

只在已进入项目 Docker 镜像时，可分别运行：

```bash
./scripts/build_native.sh
./scripts/build_cross_arm64.sh
```

## 运行

```bash
ros2 launch astrabot_teleop teleop.launch.py \
  config_file:=/etc/astrabot/teleop.yaml
```

建议启动顺序为 Gateway、RTC、Teleop；Teleop 服务不可用时 RTC 对控制 DataChannel 必须 fail closed。普通视频 viewer
不依赖本节点。

`TeleopNode` 按进程 one-shot 使用：运行期间重复 `start()` 与停止后重复 `stop()` 幂等；一旦完成 `stop()`，同一对象
再次 `start()` 会返回 `FailedPrecondition`。standalone 可执行程序在 `rclcpp::spin()` 退出后再 stop，保证 callback
已静默；若后续改为
`MultiThreadedExecutor`/component 组合，组合容器必须先 quiesce 对应 callback group，再调用 stop。

## 已知限制

- 当前输出仍是 Quest tracking-space shadow 位姿，尚未完成机器人 base frame 标定映射；绝不能直接接执行器。
- 状态服务只上报 `connected/disconnected`，不读取也不自行生成平台 `event_sequence`；该序列由 Data Collection plugin
  独占分配。
- 进程正常运行期间会尽力补报 `disconnected`；进程强制退出或 executor 未先 quiesce 时，尚未完成的异步报告
  仍可能被取消，不能把状态上报当作安全停止确认链。
- grant v1 尚未把 `peer_id` 纳入签名 claims；当前 peer identity 依赖平台/Gateway/RTC signaling route 绑定。生产协议升级前
  必须把 peer identity 纳入签名或提供等价的不可伪造绑定。
- grant 到期后端侧会 fail closed；平台若只续 writer lease 而不续 grant，连续 Teleop 时长将受 grant TTL 限制。
- 目前自动化覆盖 owner lease 纯 C++ 生命周期、接口契约和 disabled backend 的 ROS 进程内 fail-closed 边界；cpp backend
  对 Acquire/Renew/Release 的真实 ROS fake-service 超时/晚响应注入、仿真、HIL 与 8 小时 soak 仍属于上线 Gate，不能用
  单元测试结果替代。
- 本仓库不实现 WebRTC、视频、Quest signaling、LiveKit 删除、Dora 或云端推理传输。
