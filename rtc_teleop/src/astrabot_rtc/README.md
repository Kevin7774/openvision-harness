# astrabot_rtc

`astrabot_rtc` 是端侧公共实时通信组件。它只负责 Gateway 信令、PeerConnection、媒体 track 和通用
DataChannel 的 transport 边界，不理解遥操、AI、机械臂、deadman 或 arbitration 语义。

当前提交完成 ROS 接口、配置、资源边界、会话注册表、授权后转发、FFmpeg H.264 producer、每 track latest-wins worker，
以及可选的 `libdatachannel` 真实 transport。部署配置仍默认使用 `disabled` 且 `media.enabled=false`，不会在 Quest、TURN、
THOR 硬编和 HIL 尚未验收时自动开放 RTC。

## 模块边界

```text
astrabot_gateway -- std_msgs/String signaling --> astrabot_rtc
camera Image/CameraInfo -----------------------> astrabot_rtc media input
astrabot_rtc -- generic RtcDataPacket --------> astrabot_teleop
astrabot_teleop -- typed command -------------> arbitration
```

RTC 不直接依赖 Teleop、Recorder、Dora、LiveKit 或 WebRTC SDK 类型。`IWebRtcTransport` 是真实 SDK 的唯一接入 seam。

## 当前能力

- Teleop peer 使用 `session_id + peer_id`，平台 video-only viewer 可使用空 session + `peer_id`；默认最多 4 个 peer、2 个
  media peer。真实 transport 在创建 PeerConnection 前进一步固定为最多 1 个 Teleop peer 加 1 个 video peer；purpose 配额
  在媒体关闭时仍然生效，不能用纯 DataChannel peer 绕过。`media.max_encoded_subscribers=2` 是总媒体 peer admission 上限，
  第二个 Quest、第二个 WebOps 或第三个媒体 peer 都会被拒绝。
- `RtcDataPacket` payload 硬上限 16384 bytes。
- DataChannel 在授权 service 返回 `allowed=true` 之前不会转发任何 packet。
- DataChannel 的“申请授权”和“实际可用”是两个事件：创建本地 channel 后可提前发起授权，但只有 SDK `open`
  callback 与授权结果都已成功时，RTC 才发布 `state=CONNECTED、reason_code=data_channel_open` 的
  `RtcPeerEvent`。普通 PeerConnection `peer_connected` 不代表控制 channel 已可用；两事件无论谁先到都只通知一次。
- 每个授权 channel 只保留最新一包；consumer 变慢时覆盖旧包，不形成历史控制队列。
- DataChannel packet 在入队和 ROS dispatch 出队时都检查 steady deadline；若等待 dispatch 期间授权过期，RTC 删除该
  channel、丢弃缓存并增加 `data_expired_queued`，不会把“入队时合法、出队时过期”的控制包发布给 Teleop。
- Peer 进入 Disconnected/Failed/Closed，或应用通过 `CloseRtcPeer` 关闭整 peer/单 channel 时，RTC 会同步撤销对应 active
  grant 和 pending 授权请求；晚到的 service response 不能重新授权已关闭 route，请求自身超过本地 timeout 后也不会被接受。
- transport 收到 Disconnected/Failed/Closed 后会立即从 active peer 索引退役该 PeerConnection，并由单个有界 cleanup worker
  在所有已进入 SDK callback 退出后销毁资源。退役中 peer 仍计入 peer/purpose 容量，避免快速断连重建形成隐藏资源峰值；清理
  完成后同一 Quest/WebOps identity 可重新 `peer_joined`，Teleop 必须随新 session 获取新 grant，不会沿用旧授权。
- 远端主动关闭 DataChannel 时，transport 会发布 `reason_code=data_channel_closed` 且移除对应 channel label；runtime 根据
  channel 差异立即撤销该 route，PeerConnection 和视频状态仍保持独立，不会把“控制通道关闭”伪装成“RTC 断线”。
- 每个 track 独立使用容量 1 的 latest-wins mailbox；encoder 变慢时覆盖旧原始帧，不积压历史画面。
- FFmpeg `libswscale` 支持 `rgb8`、`bgr8`、`rgba8`、`bgra8`、`mono8`、`yuv422_yuy2`/`yuyv`、`uyvy` 和
  `nv12` 输入，统一缩放/转换为 NV12，再由显式选择的 FFmpeg H.264 encoder 生成 Annex-B access unit。
- 编码帧直接以同一 `shared_ptr` 调用一次 `IWebRtcTransport::sendEncodedFrame()`；libdatachannel transport 再将同一 payload
  扇出给所有订阅该 track 的独立 PeerConnection，不为 Quest/WebOps 重复编码。
- production 默认只允许 `h264_nvenc` 且要求 FFmpeg 标记 `AV_CODEC_CAP_HARDWARE`；encoder 不存在、不是硬件实现或无法打开时
  阻止节点启动，绝不会悄悄切换到 `libx264`。
- 默认先以 `640x480@60fps` 打开同一 encoder；打开失败时只允许用同一 encoder 尝试 `30fps`。运行中负载过高不会自动改用
  软件 encoder，也不会隐式改变分辨率。
- B-frame 固定为 0，lookahead/delay 尽可能通过 encoder private option 固定为 0；NVENC surface 默认最多 2 个，单帧编码输出
  默认不超过 2 MiB。
- `start()`/`stop()` 幂等；停止时清理 pending ROS service request、授权、peer 和缓存。
- diagnostics 明确发布 backend 真实能力、peer/viewer/channel 数量及丢弃计数。
- `reconnect_count` 只统计成功重连：同一 binding 首次 Connected 不计数；已经 Connected 的 peer 在
  Disconnected/Failed 后再次 Connected，或失联后销毁并以同一 binding 新建成功时加一。待重连 binding 历史有界。
- diagnostics 使用相邻采样窗口计算应用层 `media_encoded_*_window` 与 `media_peer_send_*_window`。后者按
  peer/track 成功发送计数，同一编码帧扇出给 Quest 与 WebOps 会计为两次发送；bitrate 只包含 H.264 payload bytes，
  不等于 RTP/UDP/IP 线上码率。
- 可选 `libdatachannel` backend 已实现 device offer、远端 answer/offer、双向 trickle ICE、独立 PeerConnection、
  v1/v2 DataChannel 生命周期和双向 payload。
- H264 sender 接受 Annex-B `EncodedVideoFrame`，同一编码 payload 可扇出到多个 peer track；发送前执行 track/open 检查。
  RTP timestamp 使用本机 `steady_clock` 生成，不依赖可能为 0、回跳或使用系统墙钟的 ROS image header；原始
  `capture_time_ns` 仍保留在编码帧中用于曝光到显示延迟观测。
  媒体发送按 Teleop peer 优先、普通 video viewer 在后；每个 Track 独立检查 `bufferedAmount`，超过
  `max_media_buffered_amount_bytes` 时直接丢弃该 peer 的当前帧，不积压历史画面。WebOps 拥塞只增加 viewer drop 指标，
  WebOps 正在关闭导致 sender 锁忙时也直接丢弃该 viewer 当前帧，不等待它释放锁，因此不会反向阻塞 Quest 的下一帧。
  DataChannel 发送另有独立的 `max_buffered_amount_bytes` 上限，二者不能混为一谈。
- 平台 `purpose=video` viewer 可以不携带 application `session_id`，此时 RTC 使用空 session + `peer_id` 管理连接，并按
  `peer_id` 路由 answer/candidate；video viewer 严禁携带 run/resource/token/DataChannel 控制字段。
- Teleop v2 的 `data_channel` 必须精确包含 `max_payload_bytes=16384`；v1 严格冻结为
  `label/ordered/max_packet_lifetime_ms` 三字段，额外携带 `max_payload_bytes` 也会被拒绝。`max_retransmits`、未知字段和任何
  被篡改的可靠性/大小参数都会被拒绝；v1 内部 payload 防御上限仍固定为 16384 bytes。

## ROS 契约

默认 endpoint：

| 方向 | Endpoint | 类型 |
| --- | --- | --- |
| Gateway → RTC | `/astrabot_gateway/webrtc_signal/cmd` | `std_msgs/msg/String` |
| RTC → Gateway | `/astrabot_gateway/webrtc_signal/report` | `std_msgs/msg/String` |
| RTC → 应用 | `/astrabot/rtc/peer_event` | `astrabot_rtc/msg/RtcPeerEvent` |
| RTC → 应用 | `/astrabot/rtc/data_channel/received` | `astrabot_rtc/msg/RtcDataPacket` |
| RTC → Teleop | `/astrabot/teleop/authorize_channel` | `astrabot_rtc/srv/AuthorizeDataChannel` |
| Teleop → RTC | `/astrabot/rtc/close_peer` | `astrabot_rtc/srv/CloseRtcPeer` |

`authorization_token` 只进入授权 service request，不进入普通 topic、diagnostics、日志或 RTC 持久状态。授权 service
不可用、超时、拒绝、返回 0 或返回过期时间时一律 fail closed。`expires_at` 的语义固定为机器人本机
`steady_clock` 纳秒；Teleop 必须将 grant 的墙钟剩余有效期换算成 steady deadline，避免系统时钟跳变改变已授权控制
TTL。

网络 diagnostics 中 `network_stats_available=false`，`network_rtt_ms`、`network_packet_loss_percent` 和
`network_jitter_ms` 明确标记为 `unsupported_libdatachannel_c_api`。当前使用的 libdatachannel 0.24.2 C API 没有通用
`getStats` 接口，只有 selected candidate 查询和 bitrate request，不能把“拿不到”伪装成 0。若未来切换到能提供稳定
stats contract 的 SDK，再新增真实 RTT/loss/jitter 采样。

## 配置与运行

配置样例为 `config/rtc.yaml`。安全默认值：

```yaml
transport:
  backend: disabled
  max_buffered_amount_bytes: 65536
  max_media_buffered_amount_bytes: 2097152

media:
  enabled: false
  encoder_name: h264_nvenc
  require_hardware: true
  output_width: 640
  output_height: 480
  frame_rate: 60
  fallback_frame_rate: 30
```

仅当二进制使用 `-DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=ON` 构建后，才可显式改为：

```yaml
transport:
  backend: libdatachannel
  max_buffered_amount_bytes: 65536
  max_media_buffered_amount_bytes: 2097152
media:
  enabled: true
```

请求未编译进二进制的 backend 会阻止节点启动，不会回退到 stub。DataChannel 由 device-offer 一侧创建；远端重复创建
同 label channel 会被拒绝。平台 `peer_joined.media_tracks` 可显式选择 track；未提供时 `purpose=video` 默认选择
`right_eye`，Teleop 使用配置中的全部 track。`media.enabled=true` 还要求 transport 具备真实 media track 能力，并要求每路
encoder 全部成功启动；disabled transport 下不能只开相机订阅假装视频链路可用。

`media.enabled=false` 时真实 transport 不会公开媒体 track，`purpose=video` viewer 会被拒绝；Teleop 仍可按显式平台契约使用
纯 DataChannel，但单 Teleop peer 配额仍保持为 1。

测试中如需显式验证软件编码，可使用 `encoder_name=libx264`、`require_hardware=false`、`preset=ultrafast`、
`tune=zerolatency`。这是测试/诊断选择，不是 production fallback。

运行：

```bash
ros2 launch astrabot_rtc rtc.launch.py
```

显式配置路径：

```bash
ASTRABOT_RTC_CONFIG=/etc/astrabot/rtc.yaml ros2 launch astrabot_rtc rtc.launch.py
```

显式路径不可读、未知 YAML key、topic 非绝对路径或资源上限越界都会阻止启动，不会静默回退。

## 构建与测试

机器人仓库规定正式验证必须使用 Docker：

```bash
./scripts/docker_build_and_test.sh
```

编码规范检查：

```bash
/root/.codex/skills/cpp-coding-standard/scripts/check_cpp_style.sh .
```

包内 `.gitlab-ci.yml` 将 disabled backend、真实 libdatachannel native backend 和 ARM64 runtime package Gate 分开。
共享 runner 默认没有本地 `webrtc-device:latest` 镜像，因此 libdatachannel native job 默认是可选手动 Gate；配置
`ASTRABOT_RTC_RUN_LIBDATACHANNEL_GATE=1` 且 runner 已预载受控镜像后才自动执行。disabled backend 与格式检查仍是必跑项。
ARM64 job 默认是手工 Gate；设置 `ASTRABOT_RTC_RUN_ARM64_GATE=1` 时变为强制执行，runner 必须挂载
`/opt/astrabot_sdk_2404`，不能静默退回 x86 或 disabled transport。

## libdatachannel 技术 Gate

已审查 `/root/workspace/webrtc/device` prototype。可复用结论包括：

- `LibDataChannel::LibDataChannel` CMake package 可用；
- H264 send-only track 使用 `H264RtpPacketizer` 和 Annex-B NAL；
- 低延迟 DataChannel 可设置 `unordered=true` 与 `maxPacketLifeTime=20ms`；
- 每个 viewer 必须拥有独立 `PeerConnection`，本地 SDP/ICE 回调需要携带目标 peer。

prototype 中的 MQTT、硬编码调试文件、单 peer owner、隐式 callback 生命周期和手写异常边界没有复制进本包。

执行 SDK API Gate：

```bash
./scripts/docker_libdatachannel_tech_gate.sh
```

如果 ROS SDK image 已安装 `LibDataChannelConfig.cmake`，也可在包构建时启用：

```bash
colcon build --packages-select astrabot_rtc --cmake-args \
  -DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_TECH_GATE=ON
```

该 Gate 只证明 SDK、H264 packetizer 和 v2 DataChannel API 可编译/初始化。

真实 backend 的 Docker 构建和本机双端互操作测试：

```bash
./scripts/docker_libdatachannel_backend_test.sh
```

测试会使用 `webrtc-device:latest` 中已验证的 libdatachannel 0.24.2 SDK，并在 Jazzy SDK image 中覆盖：

- device offer / remote answer；
- 双向 trickle ICE；
- `astrabot.teleop` 的 unordered + 20 ms lifetime；
- DataChannel 双向二进制 payload；
- 双 H264 send-only track；
- 重复 create/stop 和 stop 后 callback 屏障。

实现使用 libdatachannel C API，所有 SDK 失败通过返回码转换成 `Status`，不会让第三方 C++ 异常穿透 runtime。

THOR AArch64 的 FFmpeg/ROS 交叉构建：

```bash
./scripts/docker_cross_build_arm64.sh
```

脚本显式将 pkg-config 锁定到 `/opt/astrabot_sdk_2404/usr/lib/aarch64-linux-gnu/pkgconfig`，避免误链接 x86 FFmpeg。默认只构建
RTC core、FFmpeg producer 和 disabled transport。

从固定源码生成 ARM64 libdatachannel SDK，并立即回归 RTC backend 交叉链接 Gate：

```bash
sdk_output="$(mktemp -d /tmp/astrabot-libdatachannel-arm64.XXXXXX)"
./scripts/docker_build_libdatachannel_arm64_sdk.sh "${sdk_output}"
```

脚本只接受调用方显式创建的空目录，目录非空、是符号链接或已含任何文件时都会拒绝，且不会删除或覆盖已有内容。构建固定：

- Docker image digest：
  `nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e`；
- libdatachannel `v0.24.2` commit：`4e4f4892dccb2a57fe3a490d0c9d958de4244e74`；
- `json/libjuice/libsrtp/plog/usrsctp` 均校验固定 gitlink commit；
- target sysroot：`/opt/astrabot_sdk_2404`，OpenSSL 使用 target `libssl.so.3/libcrypto.so.3`；
- WebSocket API、examples 和 upstream tests 关闭，媒体与 C API 保留；子依赖静态进入 `libdatachannel.so`。

输出会校验版本头、AArch64 ELF、`libdatachannel.so.0.24` SONAME、无 RPATH/RUNPATH、CMake package、固定运行时依赖，
并携带主库与五个 vendored dependency 的许可证及 `source-manifest.txt`。SDK 自检通过后，脚本会调用
`docker_cross_build_arm64.sh`，确认 `libastrabot_rtc_webrtc.so` 精确依赖 `libdatachannel.so.0.24`。

已有同等 SDK 时也可单独执行严格导入 Gate：

```bash
ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL=1 \
ASTRABOT_RTC_ARM64_LIBDATACHANNEL_PREFIX=/path/to/libdatachannel-aarch64 \
./scripts/docker_cross_build_arm64.sh
```

导入目录必须同时包含 0.24.2 headers、AArch64 shared library、CMake package、source manifest 和完整许可证。脚本会在
configure 前检查 ELF 架构、SONAME、RPATH、精确 `NEEDED` 集及 THOR sysroot 是否提供其系统依赖；x86 SDK、额外动态依赖或
不完整目录会立即失败。

把已验证 SDK 收敛为最小运行时发布树：

```bash
runtime_output="$(mktemp -d /tmp/astrabot-libdatachannel-runtime.XXXXXX)"
./scripts/docker_stage_libdatachannel_runtime.sh /path/to/validated-sdk "${runtime_output}"
```

该脚本只能在受控 Docker image 内执行，输入、输出都必须是显式非符号链接目录，且输出必须为空。它会再次校验固定 commit、
AArch64 ELF、`libdatachannel.so.0.24` SONAME、无 RPATH/RUNPATH、精确动态依赖和六份许可证，只暂存：

```text
lib/astrabot_rtc/third_party/libdatachannel.so.0.24.2
lib/astrabot_rtc/third_party/libdatachannel.so.0.24
share/astrabot_rtc/third_party/libdatachannel/source-manifest.txt
share/astrabot_rtc/third_party/libdatachannel/licenses/...
share/astrabot_rtc/third_party/libdatachannel/runtime-dependencies.txt
share/astrabot_rtc/third_party/libdatachannel/runtime-package.sha256
```

不携带 headers、CMake package 或无版本开发链接，也不会写宿主机、调用 `ldconfig` 或修改全局 loader 配置。发布系统应把该树
合入 `/opt/ros/astrabot`；本包 systemd unit 只为 RTC 进程显式设置
`LD_LIBRARY_PATH=/opt/ros/astrabot/lib/astrabot_rtc/third_party`。最终 target image 仍必须执行 checksum、`ldd/readelf` 和
启动 smoke test，运行时暂存通过不等同于真机发布通过。

使用 `ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=ON` 构建安装时，还会生成
`share/astrabot_rtc/capabilities/libdatachannel.enabled`。`robot_start` 将该文件作为发布能力标记；disabled 构建不会生成它，
因此仅复制 runtime 或修改 YAML 不能误开真实 backend。

## 已知限制与后续 Gate

- FFmpeg producer 与 transport 接线已经完成，并在 Docker 中用显式 `libx264` 验证 Annex-B 输出；这仍不等于
  `h264_nvenc` 已在 THOR 真机可用。交叉容器没有 `libcuda.so.1`/NVIDIA driver，硬编打开和吞吐必须在真实 THOR 上验证。
- `60 → 30fps` 降级只发生在同一 encoder 的启动阶段；当前没有基于温度、encoder backlog 或网络拥塞的运行期自适应。
- encoder 当前在 `media.enabled=true` 后持续工作，即使暂时没有 viewer；后续可在不改变包边界的前提下增加 viewer-aware
  suspend/resume，但必须保持首个 viewer 加入时的 IDR/SPS/PPS 行为可验证。
- 运行时没有独立 `EncodedFrameHub`：encoder worker 直接把同一 `shared_ptr<const EncodedVideoFrame>` 交给 transport，
  transport 同时承担媒体 peer admission 和各 PeerConnection fanout。`media.max_encoded_subscribers` 保留原 YAML key，
  但它现在真实约束 transport，而不是构造一个未消费的占位对象。
- THOR 基础 sysroot 仍未预装 libdatachannel runtime；当前已经能生成带 checksum/许可证的受控最小运行时树，但把该树纳入
  机器人正式 release image、在目标机验证动态链接并走发布审批仍是部署 Gate。
- 当前自动测试使用本机 host candidate；真实 TURN credential、NAT、防火墙和 ICE restart 仍需平台环境验证。
- 必须完成 Quest + WebOps 真机解码、THOR `h264_nvenc`、100 次真实连接关闭、8 小时 soak 和 glass-to-glass 延迟测试后才能
  用于实机遥操。
