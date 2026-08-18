# VISTA 风格单 ZED 机器人实施文档

> 适用设备：Astrabot XR1 / NVIDIA Thor / ROS 2 Jazzy  
> 实施基线：2026-08-14 现场状态  
> 唯一可用视觉设备：ZED 2i  
> 目标：把通用多模态模型接成一个“看、记、想、调用受限技能”的机器人上层代理

## 0. 一句话结论

VISTA 不是新的运动控制器，也不是需要先训练的机器人策略。最快落地方式是：

```text
通用多模态模型
  → 读取 ZED 当前观测和历史观测
  → 用自然语言维护可修改的环境认知
  → 从白名单中选择一个机器人技能
  → 独立安全层校验
  → 复用现有 XR1 / MPC / Controller 执行
  → ZED 拍摄动作结果并写入长期记忆
```

第一版不训练模型、不写新的运动规划器、不允许模型执行任意 shell 或发布任意 ROS 话题。

## 1. 现场事实与约束

### 1.1 只允许使用 ZED

现场确认只有 ZED 相机可用。即使 ROS 图中还能看到腕部相机、底盘相机等话题，也必须视为残留、占位或跨机 DDS 数据，不能作为本项目输入、健康判据或成功证据。

唯一允许的视觉命名空间：

```text
/zed/zed_node/*
```

主要输入：

| 数据 | ROS 话题 | 用途 |
|---|---|---|
| 主彩色图 | `/zed/zed_node/rgb/color/rect/image` | VLM 当前视觉输入；保存无损 PNG |
| 相机内参 | `/zed/zed_node/rgb/color/rect/camera_info` | 像素反投影 |
| 注册深度 | `/zed/zed_node/depth/depth_registered` | 像素距离、三维点、桌面平面 |
| 注册点云 | `/zed/zed_node/point_cloud/cloud_registered` | 环境几何、障碍物 |
| 处理点云 | `/zed/zed_node/point_cloud/cloud_processed` | 已有点云预处理结果 |
| 左右目 | `/zed/zed_node/left\|right/color/rect/image` | 双目原始证据；第一版不直接使用 |
| IMU | `/zed/zed_node/imu/data` | 重力方向与相机运动 |
| 里程计 | `/zed/zed_node/odom` | ZED 视觉里程计 |
| 位姿 | `/zed/zed_node/pose` | 相机位姿 |
| 健康状态 | `/zed/zed_node/status/health` | 相机健康检查 |
| 心跳 | `/zed/zed_node/status/heartbeat` | 相机存活检查 |

ZED 被 `Astrabot_ZED.service` 的 `zed_wrapper` 独占。禁止再用 `pyzed` 或其它进程直接打开 USB 相机。

### 1.2 ZED 现场输入契约

2026-08-14 从正在运行的 ROS 图只读采样得到：

| 输入 | ROS 类型 | 实测格式 | frame_id | 实测频率 |
|---|---|---|---|---:|
| RGB | `sensor_msgs/msg/Image` | `960×540`、`bgra8`、step `3840` | `zed_left_camera_frame_optical` | `14.7 Hz` |
| 相机内参 | `sensor_msgs/msg/CameraInfo` | `960×540`、`rational_polynomial` | `zed_left_camera_frame_optical` | 随图像发布 |
| 注册深度 | `sensor_msgs/msg/Image` | `960×540`、`32FC1`、单位米 | `zed_left_camera_frame_optical` | `15.0 Hz` |
| 注册点云 | `sensor_msgs/msg/PointCloud2` | `448×256`、`x/y/z/rgb` 均为 float32 | `zed_left_camera_frame` | `10.1 Hz` |
| IMU | `sensor_msgs/msg/Imu` | 四元数、角速度、线加速度 | `zed_imu_link` | `85.4 Hz` |

四个主要 ZED publisher 均为 `RELIABLE + VOLATILE`。ZED 参数实测为：

```text
camera_model: zed2i
grab_resolution: HD1080
grab_frame_rate: 15
depth_mode: NEURAL_LIGHT
point_cloud_freq: 10.0
publish_tf: false
```

上述频率来自同一个 ROS 进程连续采样 4 秒；命令行逐话题短测会因为 DDS 发现和解码开销显著低估频率。配置频率不等于消费端一定收到的频率，harness 仍必须记录实际接收频率和丢帧。

同一次探针结果还确认：RGB 与注册深度最佳 header 时间差为 `0 ms`；深度有效率为 `97.24%`，有效值范围约 `0.540–2.340 m`，中位数 `1.307 m`。这些数值只描述当时场景，不应成为固定阈值。

首次探针发现 RGB、深度、点云和 IMU 的 header 时间戳比系统接收时间早约 `1.9475×10^7 s`，而 `/joint_states` 时间正确。原因是 ZED 服务在系统时钟校准前启动。2026-08-14 13:04，经人工授权并确认 Chrony 已同步后重启 `Astrabot_ZED.service`，问题已消失：RGB/深度/点云偏移约 `0.09 s`，IMU 约 `0.003 s`，探针返回 `ok: true`。

现场已确认 `base_link ← zed_left_camera_frame_optical` 和 `base_link ← zed_left_camera_frame` TF 链存在并持续更新。P0 的输入格式、同步和时钟门禁当前已经通过。

处理规则：

- 每帧同时保存 `sensor_stamp_ns` 和 `received_at_ns`。
- RGB/深度同步使用 `sensor_stamp_ns`。
- 新鲜度判断使用 `received_at_ns`。
- 时钟偏移绝对值大于 `2 s` 时，禁止把像素转换到 `base_link`，禁止据此执行动作。
- 由人工确认系统时间已同步后重启 ZED 服务，再重新运行契约探针；不得由模型自动重启服务。
- 如果重启后偏移仍存在，先修复 ZED 时间戳来源，不允许用“最新 TF”静默冒充同一时刻 TF。

可重复的只读检查入口：

```text
scripts/zed_observation_probe.py
```

### 1.3 现有控制栈继续使用

现场已运行：

```text
Astrabot_Controller.service
Astrabot_Mpc.service
Astrabot_ZED.service
Astrabot_ZED_Points.service
```

VISTA 层只负责决定“调用哪个技能”，不替代这些服务。

### 1.4 模型不能直接操作 ROS

禁止向模型开放：

```text
shell(command)
ros2 topic pub ...
任意关节角数组
任意 systemctl start/stop/restart
任意文件删除或配置修改
```

模型只能调用固定 JSON 工具。参数由独立安全层验证，不能把自然语言直接拼进 shell。

### 1.5 MPC 与直接位置控制只能选一路

MPC 和双臂 forward position controller 可以同时作为进程存在，但同一时刻不能同时向机械臂持续输出。

VISTA 执行动作前必须确认控制权；发现 MPC 或其它发布者正在输出时拒绝动作，不能绕过现有 `astra_arm` busy-channel guard。

## 2. 目标架构

```text
┌──────────────────────────────────────────────┐
│ VLM Agent（Mac、云端或独立进程）             │
│ observe → reason → expected_outcome → action │
└──────────────────────┬───────────────────────┘
                       │ 严格 JSON 工具调用
┌──────────────────────▼───────────────────────┐
│ vista_robot.py                               │
│ 1. ZED 观测                                 │
│ 2. 无损视觉记忆                             │
│ 3. GUIDE.md / WORKING.md                    │
│ 4. 技能白名单                               │
│ 5. 参数与安全校验                           │
│ 6. 动作结果验证                             │
└───────┬───────────────────────────┬──────────┘
        │ 只读                         │ 受限动作
┌───────▼────────┐          ┌────────▼─────────┐
│ ZED ROS topics │          │ xr1.py / 现有技能 │
│ joint_states   │          │ arbitration/MPC   │
│ TF             │          │ controller        │
└────────────────┘          └──────────────────┘
```

这就是本机的 VISTA harness。它不是模型，也不是控制器，而是一层固定接口：

| VISTA 原设计 | Thor 单 ZED 对应实现 |
|---|---|
| 原始 PNG 观察 | 原始 `bgra8` ROS 图像无损转为 RGB PNG |
| 所有返回帧进入视觉记忆 | 所有真正返回给模型的决策帧、动作中间帧和结果帧永久按 frame_id 保存 |
| `inspect` | 回看任意历史原图、多个历史帧或局部裁剪 |
| `read_pixels` | 读取小区域的精确像素值；不把整图文本化 |
| `GUIDE.md` | 紧凑、可修订的长期环境规律 |
| `WORKING.md` | 当前任务草稿和待验证假设 |
| `play` | 只能调用白名单机器人技能，并经过独立安全层 |
| 先预测再动作 | 动作前写 `expected_outcome`，动作后列出所有可见变化，包括意外变化 |

点云、深度数组和 ROS 消息不会直接塞入 VLM 上下文。模型默认只看 PNG 和精简状态；需要几何时，通过 `measure_depth` 等工具按需查询。这保留原始信息，同时避免高带宽传感器数据淹没上下文。

建议目录：

```text
/home/astrabot/workspace/zed_ws/
├── docs/11_vista_single_zed_implementation.md
├── scripts/vista_robot.py          # 后续新增；单进程 ROS bridge
├── scripts/test_vista_robot.py     # 后续新增；一个离线自检
└── vista_runs/
    └── <run_id>/
        ├── GUIDE.md
        ├── WORKING.md
        ├── events.jsonl
        └── observations/
            └── <frame_id>/
                ├── rgb.png
                ├── depth.npy
                └── state.json
```

## 3. 阶段 0：只读环境验收

### 3.1 设置 ROS 环境

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/astrabot/setup.bash
export ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

验收：

```bash
test "$ROS_DOMAIN_ID" = 12
ros2 node list | grep -Fx /zed/zed_node
```

注意：DDS 第一次发现可能返回假阴性，等待一秒后重试一次。

### 3.2 固化输入契约

```bash
cd /home/astrabot/workspace/zed_ws/scripts
python3 zed_observation_probe.py --seconds 5 \
  --out /tmp/zed_observation_contract.json
probe_status=$?
python3 -m json.tool /tmp/zed_observation_contract.json
test "$probe_status" -eq 0
```

探针只订阅，不创建 publisher，不打开相机设备，不移动机器人。它检查：

- ROS 类型、尺寸、encoding、step 和 data 长度。
- 相机内参与 RGB 尺寸是否一致。
- 深度单位、有效率和数值范围。
- 点云字段和 frame_id。
- RGB/深度最小时间差、各输入实际接收频率。
- ZED header 与系统接收时间的偏移。
- publisher QoS。

2026-08-14 13:05 的重启后验收结果为 `ok: true`、状态码 `0`、RGB/深度最佳时间差 `0 ms`，当前 P0 契约门禁通过。这个检查仍须在每次 ZED 服务异常重启、系统时间跳变或配置更新后重新运行。

### 3.3 验收四个现有服务

```bash
systemctl is-active \
  Astrabot_Controller.service \
  Astrabot_Mpc.service \
  Astrabot_ZED.service \
  Astrabot_ZED_Points.service
```

通过条件：四行均为 `active`。

失败措施：只记录失败服务和日志，不自动重启。

```bash
systemctl --no-pager --full status <失败服务>
journalctl -u <失败服务> --no-pager -n 100
```

### 3.4 验收 ZED 数据

```bash
ros2 topic info /zed/zed_node/rgb/color/rect/image
ros2 topic info /zed/zed_node/depth/depth_registered
ros2 topic info /zed/zed_node/point_cloud/cloud_registered
ros2 topic info /zed/zed_node/imu/data
```

通过条件：每个话题至少一个 publisher。

获取一帧彩色图的现有入口：

```bash
cd /home/astrabot/workspace/zed_ws/scripts
python3 xr1.py snap --which zed --out /tmp/vista-zed
test -s /tmp/vista-zed/zed.jpg
```

这条命令走压缩 JPEG，只用于确认链路能出图。VISTA 视觉记忆不得使用它；正式 harness 必须订阅原始 `bgra8` 并无损保存 PNG。

### 3.5 验收 TF

```bash
timeout 8 ros2 run tf2_ros tf2_echo \
  base_link zed_left_camera_frame_optical
```

通过条件：得到非零 translation 和合法 quaternion。

### 3.6 阶段门槛

满足以下全部条件才进入阶段 1：

- ZED RGB 有发布者且能保存一帧。
- ZED depth 有发布者。
- 契约探针 `ok: true`；RGB 为 `bgra8`，深度为 `32FC1` 米。
- RGB/深度最佳时间差不超过 `50 ms`。
- ZED header 与接收时钟偏移绝对值不超过 `2 s`。
- `base_link ← zed_left_camera_frame_optical` TF 可查询。
- `/joint_states` 有发布者。
- 本阶段没有移动机器人。

## 4. 阶段 1：实现无动作的视觉记忆

这一阶段只做 VISTA 最核心的三件事：当前观察、历史回看、精确局部查询。

### 4.1 `observe()`

实现状态：第一版只读命令已经落地。

```bash
cd /home/astrabot/workspace/zed_ws/scripts
python3 vista_observe.py --run-id shared --timeout 20
```

脚本路径：

```text
/home/astrabot/workspace/zed_ws/scripts/vista_observe.py
```

其他 agent 的发现入口：

```text
/home/astrabot/.codex/skills/thor-observe/SKILL.md
/home/astrabot/.agents/skills/thor-observe/SKILL.md
/home/astrabot/workspace/zed_ws/AGENTS.md
```

2026-08-14 smoke observation 已通过：RGB/深度时间差 `0 ms`、精确图像时刻 TF 可用、PNG 为 `960×540` 三通道、深度数组为 `540×960 float32`。这只证明单次调用正确；阶段 1 的连续 100 次验收仍未完成。

一次 `observe()` 同步保存：

```text
RGB 原图 → rgb.png
注册深度 → depth.npy
相机内参 → camera_info.json
关节、夹爪、ZED pose、时间戳 → state.json
事件索引 → events.jsonl
```

RGB 必须订阅未压缩的 `sensor_msgs/Image`。当前源格式为 `bgra8`；去掉 alpha 并进行 BGRA→RGB 通道转换后保存 PNG，这个转换不损失颜色信息。不能把 JPEG 再命名成 PNG。

当前深度契约是 `32FC1` 米。保存时仍必须把原始 encoding、单位、step 和无效像素统计写入 `state.json`；如果运行时变成其它 encoding，应拒绝该帧并要求重新确认契约，不能猜单位。

返回给模型的最小结果：

```json
{
  "frame_id": "000042",
  "rgb_path": "observations/000042/rgb.png",
  "depth_path": "observations/000042/depth.npy",
  "sensor_stamp_ns": 1786683920000000000,
  "received_at_ns": 1786683920090000000,
  "rgb_depth_delta_ms": 0.0,
  "clock_offset_ms": 90.0,
  "clock_ok": true,
  "joint_state_age_ms": 12,
  "tf_ok": true,
  "geometry_status": "ready"
}
```

只有 `clock_ok` 和 `tf_ok` 同时为真时，`measure_depth()` 才能返回 `point_base_m`；否则只允许返回光学坐标和明确错误原因。

### 4.2 `inspect()`

输入：

```json
{"frame_id":"000042","crop":[420,180,760,520]}
```

行为：从原始 `rgb.png` 裁剪指定区域，返回裁剪图；不重新请求相机。

边界校验：

- `frame_id` 必须存在于当前 run。
- crop 坐标必须为整数且落在原图范围内。
- 宽高必须大于零。

### 4.3 `read_pixels()`

输入一个小区域，返回原始 RGB 数值统计。用途是确认小色块、边界和变化，不把整张图转成文本矩阵。

### 4.4 `measure_depth()`

输入：

```json
{"frame_id":"000042","pixel":[691,290],"radius":3}
```

输出：

```json
{
  "depth_median_m": 0.82,
  "valid_count": 31,
  "point_optical_m": [0.12, -0.08, 0.82],
  "point_base_m": [0.52, -0.16, 0.83]
}
```

深度用邻域中位数，不能信单个像素；反投影使用同一帧 `camera_info`；到 `base_link` 的转换使用同一时间戳 TF。

### 4.5 第一版存储策略

保存“harness 实际返回给模型的每一帧”：决策帧、模型请求回看的帧，以及动作期间 harness 明确采集并返回的有限中间帧和最终帧。不连续保存整条 15 Hz 原始流。

这是 VISTA 原则在连续物理世界中的对应实现：不是保存相机产生的每一帧，而是无损保存环境接口返回给 agent 的每一帧。只有确认短暂事件在两次观察之间丢失时，才增加有时长上限的环形缓存。

### 4.6 阶段门槛

- 连续执行 100 次 `observe()` 无崩溃。
- 任意历史 `frame_id` 能恢复相同 RGB 尺寸和像素。
- `inspect()` 不修改原图。
- `measure_depth()` 对无效深度明确返回错误，不能返回 0 当作有效距离。
- 全程没有 ROS publisher 和机器人动作。

## 5. 阶段 2：定义模型工具协议

初始工具白名单：

| 工具 | 初始状态 | 说明 |
|---|---|---|
| `observe` | 开启 | 获取当前 ZED、深度和状态 |
| `inspect` | 开启 | 回看历史原图或局部 |
| `read_pixels` | 开启 | 精确颜色查询 |
| `measure_depth` | 开启 | 像素深度与三维点 |
| `read_guide` | 开启 | 读取长期认知 |
| `write_guide` | 开启 | 更新长期认知；只限当前 run 文件 |
| `read_working` | 开启 | 读取当前任务草稿 |
| `write_working` | 开启 | 更新当前任务草稿 |
| `get_robot_state` | 开启 | 只读关节、夹爪、控制占用 |
| `execute_skill` | 关闭 | 阶段 4 后逐项开放 |
| `stop` | 开启 | 停止 VISTA 自己发起的动作 |

工具调用统一使用 JSON，不接受自由文本参数。

每次动作前，模型必须给出：

```json
{
  "goal": "让 ZED 看清桌面中央",
  "expected_outcome": "下一帧中桌面区域面积增加，头部 pitch 接近 35 度",
  "skill": "look",
  "params": {"pitch_deg":35,"yaw_deg":0}
}
```

`expected_outcome` 只用于记录和动作后验证，不作为绕过安全层的理由。

## 6. 阶段 3：影子模式

影子模式中，模型可以完整地看、回看、测深度、写笔记、提出动作，但 `execute_skill` 永远返回：

```json
{"accepted":false,"reason":"shadow_mode"}
```

### 6.1 测试任务

依次让模型完成：

1. 描述桌面中可见物体。
2. 找到黄色目标并用 crop 放大。
3. 查询目标中心深度。
4. 将目标像素转换到 `base_link`。
5. 比较动作前后两帧中同一区域的变化。
6. 把确认过的环境规律写入 `GUIDE.md`。

### 6.2 验收

- 连续完成 20 个观察—推理回合。
- 模型引用的 `frame_id` 都真实存在。
- 不存在越界 crop、无效深度当有效值、错误 TF frame。
- 模型提出的动作全部被记录，但机器人没有移动。
- 人工检查 `GUIDE.md`，没有把假设写成已确认事实。

## 7. 阶段 4：逐个开放低风险技能

技能按风险从低到高逐个开放，不一次性开放全部能力。

### 7.1 `look`

复用现有 `XR1.look_at()` 或 `xr1.py look` 的斜坡和限位，不重复实现颈部控制。

参数：

```json
{"pitch_deg":35,"yaw_deg":0}
```

校验：

- pitch/yaw 均限制在 `[-40, 40]` 度。
- 一次只执行一个 look。
- 动作前 `/joint_states` 必须新鲜。
- 动作后必须 `observe()`，比较预期与实际。

验收：5 次小角度动作和 5 次返回 0 度均成功，再开放下一项。

### 7.2 `grip`

复用 `XR1.grip()`，参数只允许：

```text
side: left | right
value: open | close | 0.0..1.0
```

校验：

- 夹爪反馈话题存在且新鲜。
- 一次只动一只夹爪。
- 无反馈时拒绝，不允许“发了就算成功”。

### 7.3 `home`

复用 `XR1.home()` 的 measured-pose ramp、速度限制和 URDF 限位。

校验：

- 人工确认工作区无人且无遮挡。
- MPC/其它控制发布者未占用手臂通道。
- 当前 `/joint_states` 新鲜。
- 失败时保持当前位置，不自动无限重试。

### 7.4 阶段门槛

- 每个技能单独通过后才加入白名单。
- 动作都能在 `events.jsonl` 找到请求、校验、开始、结束、验证五段记录。
- 任何动作失败后，下一步只能是 `observe`、`stop` 或人工处理。

## 8. 阶段 5：接入现有高级技能

VISTA 不生成新的抓取轨迹，只调用已经由人验证过的高层技能。

第一版接口：

```json
{
  "skill":"pick_yellow_block",
  "params":{"side":"right"},
  "expected_outcome":"黄色积木离开原位置并随夹爪抬升"
}
```

接入前要求：

- 对应脚本在当前硬件状态下由人工独立验证成功。
- 脚本内部已有人员检测、工作区、路径、关节速度、桌面高度等拒动条件。
- 脚本能返回结构化结果，而不是只依赖终端文字。
- 技能执行期间模型不能插入第二个动作。

如果当前没有一条可靠抓取技能，就保持 `pick_yellow_block` 关闭。VISTA 不能把未验证脚本变成可靠技能。

## 9. 动作结果验证

每个动作结束后立即保存新观察，并比较：

```text
expected_outcome
ZED 动作前 RGB/depth
ZED 动作后 RGB/depth
关节前后状态
夹爪前后状态
技能返回码
```

结果状态只允许：

```text
succeeded
failed
uncertain
refused
crashed
```

禁止把“命令无报错”自动写成 `succeeded`。

抓取结果至少使用两类独立证据：

- 夹爪没有完全闭合或完全张开。
- ZED 中目标原位置明显变化。
- 抬升后 ZED 能看到目标随末端移动。

只有一类证据时写 `uncertain`。

## 10. VISTA 风格记忆规则

### 10.1 `GUIDE.md`

保存跨任务仍可能有用的、经过证据确认的规律：

```text
- ZED 头部在 pitch=35°、yaw=0° 时覆盖主要桌面工作区。
- 黄色目标在某类光照下的稳定颜色范围。
- 哪些 base_link 区域属于右臂可靠可达区。
```

每条规律必须附证据 frame_id 或事件 id。

### 10.2 `WORKING.md`

只保存当前任务的短期状态：

```text
- 当前目标是什么。
- 已经尝试了什么。
- 哪些假设尚未确认。
- 下一步为什么这样选。
```

新任务开始时重新创建，不能把上一个任务的物体位置当作当前事实。

### 10.3 上下文压缩

模型上下文接近上限时：

1. 把长期规律写入 `GUIDE.md`。
2. 把当前进度写入 `WORKING.md`。
3. 启动新模型上下文。
4. 新上下文先读两份笔记和最近事件。
5. 历史原图仍通过 `inspect(frame_id)` 无损取回。

## 11. 安全边界

VISTA 层不能拥有高于现有机器人安全层的权限。

强制规则：

- 模型不能修改安全阈值。
- 模型不能关闭人员检测或碰撞检查。
- 模型不能调用 `systemctl`。
- 模型不能使用 `pkill`、`kill -9`、设备重枚举或 USB reset。
- 模型不能自行停止 ZED 服务后改用 pyzed。
- 每次只允许一个动作在执行。
- 所有动作必须有超时，但不能只依赖 shell `timeout`；执行循环内部也必须检查截止时间。
- `stop` 必须不依赖 VLM 正常响应。
- 动作执行与日志记录失败时，以停止动作为优先。

## 12. 失败处理与回滚

### 12.1 ZED 无图

```bash
systemctl --no-pager --full status Astrabot_ZED.service
journalctl -u Astrabot_ZED.service --no-pager -n 100
ros2 topic info /zed/zed_node/status/heartbeat
```

只诊断，不自动重启；重启相机会影响其它 ROS 消费者。

### 12.2 深度无效

- 返回结构化 `invalid_depth`。
- 不用 RGB 猜距离。
- 不执行依赖三维位置的技能。
- 允许模型换一个像素邻域重新查询，但限制重试次数。

### 12.3 TF 不可用

- 保留像素和 optical frame 三维点。
- 禁止输出或执行 base_link 目标。
- 不使用最后一次旧 TF 冒充当前 TF。

### 12.4 控制通道忙

- 返回 `control_channel_busy`。
- 不绕过仲裁器。
- 不自动停止 MPC 或其它控制进程。

### 12.5 关闭 VISTA

VISTA 第一版不是 systemd 服务，直接停止 `vista_robot.py` 即可。现有 Controller、MPC、ZED 和其它机器人服务保持原状，因此回滚不需要修改系统配置。

## 13. 开发顺序与交付物

| 顺序 | 交付物 | 是否动机器人 | 通过标准 |
|---|---|---:|---|
| P0 | 本文档 + 环境检查 | 否 | ZED、TF、状态事实确认 |
| P1 | `observe/inspect/read_pixels/measure_depth` | 否 | 100 次观察自检 |
| P2 | `GUIDE.md/WORKING.md/events.jsonl` | 否 | 任意历史帧可恢复 |
| P3 | VLM 影子模式 | 否 | 20 回合零动作、零越界 |
| P4 | `look` | 是 | 10 次小动作成功 |
| P5 | `grip/home` | 是 | 单技能人工验收 |
| P6 | 一个已验证高层技能 | 是 | 固定场景成功后再扩展 |
| P7 | 长任务组合 | 是 | 每一步都有物理结果验证 |

## 14. 第一版明确不做

- 不在 Thor 上训练 VLM。
- 不接入腕部、胸前或底盘相机。
- 不连续录制全部 ZED 帧。
- 不让 VLM 输出关节轨迹。
- 不新建运动规划器。
- 不替换 MPC 或 ros2_control。
- 不做多代理框架、消息队列、数据库服务或 Web 控制台。
- 不把 VISTA 注册为开机自动启动服务。

当 P1–P6 已稳定、确实出现吞吐或运维需求时，再增加常驻服务、远程 API 或数据库。

## 15. 最终验收定义

单 ZED VISTA 最小版本完成，需要同时满足：

1. 模型可以观察当前 ZED 原图。
2. 模型可以按 frame_id 无损回看历史原图和局部区域。
3. 模型可以查询深度和 `base_link` 三维点。
4. 模型能维护有证据引用的 `GUIDE.md` 和当前任务 `WORKING.md`。
5. 模型只能调用明确白名单技能。
6. 安全层能拒绝越界、旧状态、无 TF、无深度和控制通道占用。
7. 每个动作都有预期结果、前后观察和物理结果判定。
8. 停止 VISTA 后，机器人原有 ROS、MPC、Controller、ZED 服务不受影响。

## 16. 参考

- VISTA 项目页：<https://vista-research.github.io/>
- 本机统一控制入口：`scripts/xr1.py`
- 本机全栈自检：`scripts/xr1_verify.py`
- 本机感知说明：`docs/07_perception_scheme.md`
- 本机抓取流程：`docs/10_grasp_pipeline.md`
- 本机踩坑与安全边界：`PITFALLS.md`
