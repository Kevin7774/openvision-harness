# 02 · 机器人本体信息 & 执行条件自检

> 🔴 **桌高现值 = `0.8108 m`（2026-08-11 摇操真值）。本文出现的 0.75 / 0.750 / 0.7415 / 0.702 全部作废。**
> 依据：人摇到一个能**夹住**黄积木的位姿，两指中点 FK = (+0.4749, −0.3591, **+0.8238**)，减 13 mm 夹持高度。
> 积木坐在桌上、被夹在两指之间 ⇒ 不经相机、不经外参、不经人的瞄准，**自证**。详见 [`PITFALLS.md` §45](../PITFALLS.md)。
> 代码里唯一权威是 `grasp_block.py` 的 `TABLE_TOP`；**别在文档或脚本里再硬编码桌高**。


> 全部数据来自本机实测（`ros2 node/topic list`、URDF 解析、`lsusb`/`v4l2-ctl`、启动日志），非推测。
> 采集时间：2026-08-07 10:30–10:36 CST

---

## 1. 计算与系统

| 项 | 值 | 来源 |
|---|---|---|
| 主控 | NVIDIA **Jetson Thor**（GPU UUID `GPU-a7c66ad2-…`），aarch64 | `nvidia-smi -L`, `uname -m` |
| OS | Ubuntu 24.04.3 LTS (Noble)，内核 `6.8.12-rt-tegra`（**RT 实时内核**） | `/etc/os-release` |
| ROS | ROS 2 **Jazzy** + 自研 overlay `/opt/ros/astrabot` | `/opt/ros/` |
| 机器人软件版本 | `astrabot-ros-thor-2.3.10` | `/opt/ros/astrabot/astrabot-ros-thor-2.3.10-version.info` |
| DDS | `rmw_fastrtps_cpp`，**`ROS_DOMAIN_ID=12`** | `/opt/ros/start_up/config/ros_config.sh` |
| 第二计算单元 | **ECU**（节点命名空间 `/astrabot/ecu/*`，与 `/astrabot/thor/*` 并存） | `ros2 node list` |
| ZED SDK | **5.2.3**，含 ZED2i firmware | `/usr/local/zed/zed-config-version.cmake` |
| 数采环境 | `~/deploy/.venv`（Python **3.10**，`dora` + `astra` CLI）—— 注意与 ROS 侧 Python 3.12 不同 | `~/deploy/.venv/pyvenv.cfg` |

> ⚠️ **踩坑点**：任何 `ros2` 命令必须先 `export ROS_DOMAIN_ID=12`，否则只看得到 `/rosout`、`/parameter_events` 两个话题，会误判"机器人没起来"。

---

## 2. 机械本体：XR1 / EVT2（配置里 `robot_type: "evt11"`）

URDF：`/opt/ros/astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_description.urdf`

### 2.1 自由度总表

| 部位 | DOF | 关节名 |
|---|---|---|
| 左臂 | 7 | `left_arm_1_joint` … `left_arm_7_joint` |
| 右臂 | 7 | `right_arm_1_joint` … `right_arm_7_joint` |
| 左夹爪 | 1（prismatic） | `left_gripper_joint` |
| 右夹爪 | 1（prismatic） | `right_gripper_joint` |
| 头部 | 2 | `head_yaw_joint`, `head_pitch_joint` |
| 升降/躯干 | — | `platform_joint`（URDF 里是 **fixed**，z=0.4416）；躯干另有 `/astrabot_body/{hip,knee,ankle}_joint/command` + `/astrabot/height` |
| 底盘 | 轮式 | `/cmd_vel`（无关节） |

**可控总计：7+7+1+1+2 = 18 DOF（上体）+ 轮式底盘 + 躯干升降。**

### 2.2 关节限位（URDF 实测）

| 关节 | 类型 | lower | upper |
|---|---|---|---|
| `left_arm_1` / `right_arm_1` | revolute | -3.1 | 3.1 |
| `left_arm_2` | revolute | **-0.174** | **3.05** |
| `right_arm_2` | revolute | **-3.05** | **0.174** |
| `left_arm_3` / `right_arm_3` | revolute | -3.1 | 3.1 |
| `left_arm_4` | revolute | **-0.139** | **2.355** |
| `right_arm_4` | revolute | **-2.30** | **0.0** |
| `left_arm_5` / `right_arm_5` | revolute | -3.1 | 3.1 |
| `left_arm_6` | revolute | -1.57 | 1.57 |
| `right_arm_6` | revolute | -1.50 | 1.50 |
| `left_arm_7` / `right_arm_7` | revolute | -3.1 | 3.1 |
| `left_gripper` / `right_gripper` | **prismatic** | **0** | **0.045 m** |
| `head_yaw` / `head_pitch` | revolute | ~~-3.1~~ → **-0.698132** | ~~3.1~~ → **0.698132** |

> 🔴 **头部那一行是我改的，不是厂商值 —— 这条必须记住。**
> 出厂 URDF 给两个头部关节写的是 ±3.1 rad 占位符（`effort` / `velocity` 也是 3.1，见下面的警告），
> 真实行程是 **±0.698132 rad = ±40°**，来自 `astrabot_fd_sdk/AstrabotFdSm45bl.hpp` 的 `GEAR_ANGLE_MAX`。
> `astra_arm` / `xr1.py` 从**活的 `/robot_description`** 读限位，所以这份 URDF 是**唯一**的软限位来源:
> 厂商包一重装就静默回到 ±3.1，头部会接受 **4.4 倍**真实行程且**没有任何报错**。
> 已登记在 `scripts/xr1_verify.py` 的 `GUARDED_CONFIGS`（section 10 每次会话都查），
> 详见 `../PITFALLS.md` §22 / §23（一共 4 个装机目录文件被改过，不是 1 个）。

> 🚨 **重要警告：URDF 里所有关节的 `effort` 和 `velocity` 都是 `3.1`。**
> 这是**占位值**，不是真实力矩/速度上限（14 个关节 + 夹爪 + 头部全部相同，物理上不可能）。
> **→ 任何关于"力度"的推导都不能用 URDF。** 力控上限必须从 `astrabot_actuator_sdk` 或厂商手册取，
> 或用阶段三的实测流程标定（见 `05_experiment_protocol.md` 假设 H2）。

### 2.3 几何 / 工作空间（URDF 链节实测）

肩关节（`*_arm_1_joint`）在 `base_link` 下的位置（忽略旋转的粗算）：

```
left_arm_1  ≈ (x=-0.040, y=+0.125, z=1.014)   [m]
right_arm_1 ≈ (x=-0.040, y=-0.125, z=1.014)   [m]
```

肩到 TCP 的链节位移逐段（单臂相同）：

```
arm_1→arm_2  0.0855      arm_5→arm_6  0.0530
arm_2→arm_3  0.1405      arm_6→arm_7  0.1166
arm_3→arm_4  0.1695      arm_7→TCP    0.0728
arm_4→arm_5  0.1330      ──────────────────
                         肩→TCP 链节和 ≈ 0.771 m（伸直上界）
```

- **粗略可达半径**：肩心为球心，**R_max ≈ 0.75 m**（完全伸直，实际因 arm_2/arm_4 限位收窄）
- **实用可达半径**：建议按 **0.30 ~ 0.60 m** 规划，肘部留姿态余量
- **肩高 1.014 m**：若桌面高 0.72–0.75 m，则肩在桌面上方约 **0.26–0.29 m** —— 桌面操作的姿态是"俯身向下抓"，是合理构型
- **双肩间距 0.25 m**：双臂共同可达区在机身正前方一个窄带，双臂协同抓同一块积木需要 y ≈ 0

> ⚠️ 以上是**链节位移求和**，不是 FK。真正的可达空间必须跑一遍正运动学采样（阶段三 H1 会做，见 `05_experiment_protocol.md`）。

### 2.4 夹爪

- 类型：**平行两指夹爪**（`*_gripper_base_link` → `*_gripper_link`，单个 prismatic 关节）
- 行程：**0 → 0.045 m**（单关节值；实际张口 mm 数需实测，可能是单指行程也可能是总开口）
- 驱动话题：`/rm_left/rm_driver/teleop_gripper_float`、`/rm_right/rm_driver/teleop_gripper_float`
  （`rm_` 前缀 → **RealMan 系列驱动**）
- 状态话题：`/astrabot/gripper_left_state`、`/astrabot/gripper_right_state`
- 技能库里夹爪的语义描述是 **"red two finger gripper"**（红色两指夹爪）

---

## 3. 传感器

### 3.1 相机（`lsusb` + `v4l2-ctl` 实测）

| 逻辑名 | 物理设备 | `/dev/video*` | USB | 状态 |
|---|---|---|---|---|
| 头部主视觉 | **STEREOLABS ZED 2i**（USB3 `2b03:f880` + HID `2b03:f881`） | `video0`, `video1` | Bus002 Dev002 | 🔴 **ROS 节点已死** |
| 胸部（鱼眼） | Realtek `0bda:5856` "USB Cam" | `video2`, `video3` | usb-3.3 | 🟢 **实拍成功**；软链缺失 |
| 腕部 A（左） | Sunplus `1bcf:2cd1` "DECXIN CAMERA" | `video4`, `video5` | usb-4.3 | 🟢 **实拍成功**（曾因镜头盖未开出纯噪声，已解决）；软链缺失 |
| 腕部 B（右） | Sunplus `1bcf:2cd1` "DECXIN CAMERA" | `video6`, `video7` | usb-4.4 | 🟢 **实拍成功**（同上）；软链缺失 |
| 底盘左 | Orbbec 深度相机 `2bc5:069f` | — | — | 🟢 `/chassis_left_camera/depth/image_raw` 在发 |
| 底盘右 | Orbbec 深度相机 | — | — | 🟢 `/chassis_right_camera/depth/image_raw` 在发 |

> 两个腕部相机是**同型号同 VID:PID**（`1bcf:2cd1`），靠 USB 拓扑（4.3 / 4.4）区分。
> 写 udev 规则时必须用 `KERNELS=="...usb-4.3..."` 之类的路径匹配，**不能只用 idVendor/idProduct**，否则左右腕会随机互换 —— 这会静默污染数据集。
>
> 📌 **实测经验**：三路 webcam 都必须用 **MJPG** 打开（`ffmpeg -input_format mjpeg`）；
> `video3`/`video5`/`video7` 是 **metadata 节点**（`VIDIOC_G_INPUT: Inappropriate ioctl for device`），不是图像节点。
>
> 📌 **已解决的坑**：两路腕部相机初测输出纯传感器噪声，原因是**镜头保护盖未打开**（2026-08-07 10:45 打开后恢复正常）。
> 左腕画面里能直接看到自己那只两指夹爪，与技能库 `gripper: red two finger gripper` 对上。
> ⚠️ 但**实测夹垫是橙色不是红色**（HSV hue 落在 5~25，检测器用 `(5,120,80)-(25,255,255)`）——照"红色"调阈值会漏检。技能库那个字符串是标签，不是测量值。

### 3.2 其他传感器

| 传感器 | 话题 | 状态 |
|---|---|---|
| 激光雷达（bluesea） | `/scan` | 🟢 |
| 超声 ×3（KS114 左/中/右） | `/ultrasonic/{left,middle,right}/range` | 🟢 |
| IMU（底盘） | `imu_chassis_node` | 🟢 |
| 里程计 | `/odom`, `/wodom`, `FusionOdom` | 🟢 |
| 关节状态 | `/joint_states`（14 臂关节，`frame_id: base_link`） | 🟢 有数据 |
| EE 位姿 | `/ee/pose` | 🟢 |

---

## 4. 控制接口（可用于"自主"的入口）

| 层级 | 接口 | 说明 |
|---|---|---|
| 关节位置（底层） | `/astrabot_arm_forward_position_controller/commands` | `forward_command_controller/ForwardCommandController`，**active** |
| 关节位置（管理层） | `/actuator_manager/joint_position_commands` | |
| **笛卡尔 EE** | **`/ee/pose_cmd`** | ✅ 积木 pick&place 最合适的入口 |
| 单臂命令 | `/astrabot/left_cmd`, `/astrabot/right_cmd` | |
| **全身 MPC** | `/mobile_manipulator_{left,right}_tcp_link_mpc_target` | `astrabot_mpc` 节点在跑，臂+底盘协同 |
| 躯干 | `/astrabot/trunk_cmd`, `/astrabot/height`, `/astrabot_body/*/command` | |
| 底盘 | `/cmd_vel`（Nav2 全栈 + `collision_monitor` 在跑） | |
| 控制模式 | `/astrabot/ctrlmode` | 切换自主/遥操的关键，**需确认取值** |
| 状态/故障 | `/astrabot_status/{summary,detailed,faults,exception}` | 自主回路必须订阅这个做安全监控 |
| 仿真镜像 | `/astrabot/joints_mujoco` + `skill_library/sim_astrabot/` | ✅ **可先在 MuJoCo 里预演，别拿真机试错** |

**已加载控制器**（`ros2 control list_controllers`）：

```
joint_state_broadcaster                   JointStateBroadcaster   active
astrabot_arm_forward_position_controller  ForwardCommandController active
```

`controller_manager` 目标频率 **200 Hz**。

---

## 5. 数据采集链路（代码仓理解）

```
astra run ~/deploy/data_collection.yaml
   └─ dora flow: astra_data_collection
      ├─ add_astrabot          ROS 控制节点 + ZED + 腕部/胸部 webcam + robotd 云桥
      ├─ add_virtual_keyboard  esc = 全图停止
      ├─ add_teleop            解 LiveKit 字节流 → Action_* / Button_*
      ├─ add_simple_planning   skill 序列，Button_X 推进 / Button_B 复位
      ├─ add_data_buffer       组装 observation dict
      └─ add_recorder          落 LeRobot dataset，Button_A 开/停录制
```

- 入口脚本：`/opt/ros/start_up/run/start-data-collection.sh`
- Flow 源码：`~/deploy/.venv/lib/python3.10/site-packages/robot_data_collection/dataflows/flows/astra_data_collection.py`
- 技能库：`.../robot_planning/skill/skill_library/astrabot/`（现有 skill：`pick_dish_box_from_machine`、`place_dish_box_on_table`、`add_ice`、`pour_liquid`、`seal`、`scan_QR_code` …）
- Skill YAML 格式（可直接照抄写积木 skill）：
  ```yaml
  skill_id: astrabot_xr1.place_dish_box_on_table_897b8062
  instruction: place the <dish_box> on the <table>
  arguments:
    objects_grounding:
      dish_box: stainless steel dish box with handles on both ends
      gripper: red two finger gripper
      table: wooden kitchen table
  initial_condition: (there is <dish_box> on the <gripper>) AND (there is no <dish_box> on the <table>)
  terminal_conditions_set: (<gripper> is empty) AND (there is <dish_box> on the <table>)
  grounded: true
  ```

### 5.1 数据集 Schema（`robot_planning/config/astrabot/astrabot_features.yaml`）

| feature | dtype | shape | hz |
|---|---|---|---|
| `observation_images_head` | video | 480×640×3 | 30 |
| `observation_images_left_wrist` | video | 480×640×3 | 30 |
| `observation_images_right_wrist` | video | 480×640×3 | 30 |
| `observation_images_chest` | video | 480×640×3 | 30 |
| `observation_states_ee_pose_left` / `_right` | float32 | [9] | 100 |
| `observation_states_joint_angle_left` / `_right` | float32 | [7] | 100 |
| `observation_states_gripper_left` / `_right` | float32 | [1] | 100 |
| `observation_states_chassis_obs_vel` | float32 | [3] | 100 |
| `action_left` / `action_right` | float32 | [10] | 200 |
| `action_left_filter` / `action_right_filter` | float32 | [10] | 200 |
| `action_chassis_cmd_vel` | float32 | [3] | 100 |
| `timestamp` / `skill_id` / `task_id` | float32 / string / string | [1] | — |

录制 fps 30，`use_videos: true`，`video_encode_method: cv2`，上传 lakeFS bucket `bj-data`。

> ✅ **好消息**：schema 已经完备，`skill_id` 是 per-frame 字段 —— 阶段三不同 sub agent 的不同策略可以**直接靠 `skill_id` 区分**，不需要改 schema。
> ⚠️ `action_*` 是 **[10] 维**（≈ 7 关节 + 夹爪 + 2？），确切语义需读 `hardware/dataflows/modules/robots/astrabot.py` 确认。

---

## 6. 🔍 执行条件自检（逐项）

### 🔴 硬阻塞

#### B1 — ZED 2i 头部相机节点已崩溃退出

```
[zed.zed_node] Error opening camera: INVALID FUNCTION CALL
[ZED][ERROR] INVALID FUNCTION CALL in sl::ERROR_CODE sl::Camera::open(sl::InitParameters)
[zed.zed_node] Camera detection timeout
[ERROR] process has died [pid 84188, exit code 1, cmd '.../component_container_isolated ... -r __node:=zed_container']
```

- 相机**物理在线**：`lsusb` 见 `Bus002 Dev002 2b03:f880 STEREOLABS ZED 2i`（USB3）+ `Bus001 Dev004 …f881 HID`，`v4l2-ctl` 见 `video0/1` card type = "ZED 2i"
- `~/ZED_Diagnostic_Results.json` 大小 **0 字节** → 诊断也没跑完
- 残留的 `/zed/zed_node/point_cloud/*` 话题是**节点死前注册的僵尸话题**，不要误判为"ZED 正常"

**复检更新（2026-08-07，用户修了一些东西之后）**：节点这次**没有崩**，但状态仍不可用 ——

```
仅有 3 个话题：
  /zed/zed_description
  /zed/zed_node/point_cloud/cloud_processed
  /zed/zed_node/point_cloud/cloud_registered
→ 没有任何 rgb / depth / left / right 图像话题
→ 三个话题 `ros2 topic hz` 10 s 内均无数据
```

即从「节点崩了」变成「节点起来了但不出数据」。**这不是进展的假象也不是退步，是换了一个故障面** ——
之前要查 `Camera::open` 失败，现在要查为什么话题不发布、以及为什么图像话题根本没注册
（怀疑是 launch 里的 `general.pub_*` / 分辨率配置，或下面那条共享内存错误）。

#### ❌ B1 复检更新之二（2026-08-07，两个假设都被证伪）

> ## ⚠️ 2026-08-10 再次更正：下面「证伪 1」本身是错的
>
> 「这台机器的 ZED 真正的通路是 dora，不是 ROS」**不成立**。`Astrabot_ZED.service`
> 跑的是 `zed_wrapper`，它一直把 RGB / `depth/depth_registered` / `point_cloud/cloud_registered`
> / `imu/data` / `camera_info` 和**完整 TF** 发在 ROS 图上。实测（08-10）：
> rgb 11.0 Hz、depth 8.5 Hz、imu 99 Hz、`base_link ← zed_camera_link` = (+0.0492, −0.0015, **+1.3430**)。
> 我当时看不到它，原因是 **`ROS_DOMAIN_ID` 不是 12**（见 `../PITFALLS.md` §1），
> 不是「wrapper 死了」。
>
> 连带更正:①「全部 3D 感知不可用」不成立 —— 深度是米制的（baseline 119.86 mm）；
> 真实限制是**逐像素深度不能当绝对读数用**，聚合平面可用、单点读数不可用。
> （原写"桌布重复纹理"是**不完整的归因**：08-11 换白桌面后覆盖率 70.7%→90.9%，
> 但桌高偏差从 −48 mm 变成 **+63 mm**，**反号** ⇒ 纹理只影响覆盖率，桌高那一项是
> 偏差、跟纹理无关。见 `../PITFALLS.md` §41。）
> ②不需要打开 dora 的 `SEND_DEPTH=1`，也不需要 pyzed —— 而且 wrapper **独占相机**，
> 所以 pyzed 直开**必然**失败（`../PITFALLS.md` §5）。用 `scripts/xr1.py snap`。
>
> 下面的原文保留，因为「我是怎么把一个环境变量问题误判成通路问题的」值得记着。

**证伪 1 —— 我在错的地方找 ZED。** ROS 侧的 ZED 话题现已**全部消失**（之前那 3 个来自一个
已经死掉的 ROS wrapper）。**这台机器的 ZED 真正的通路是 dora，不是 ROS**：
`python -m hardware.devices.sensors.camera.single_zed_node`（PID 973488）活在
`dora run` 之下、`astra run /home/astrabot/config/data_collection_xr1_evt2.yaml`（PID 972795）之内，
14:29:02 启动。**数采链路是在跑的。** 所以"修 ROS 侧 ZED wrapper"这个方向本身是错的。

**ZED 的真实状态：图像有，深度没有。** 两道独立的门：
1. `~/deploy/.venv/.../single_zed_node.py:152` —— `SEND_DEPTH = bool(int(os.getenv("SEND_DEPTH","0")))`。
   主循环只取 `sl.VIEW.LEFT/RIGHT/LEFT_UNRECTIFIED/RIGHT_UNRECTIFIED` 经 pyarrow 发 rgb8，
   **完全不输出深度**。
2. 这台相机开机自校准失败（`POTENTIAL_CALIBRATION_ISSUE`），靠 `camera_disable_self_calib=True`
   才开起来。`~/config/zed_dora_selfcalib.sh` 原话记着代价：
   *"the SDK falls back to a factory calibration it flags as suspect, so depth is NOT metric
   (2.4x-8.9x error, growing with range)."* —— **让它"能看到"的修复，正好弄坏了深度。**
   ⚠️ 该数字是 **08-06 13:04** 量的，而 `/usr/local/zed/settings/SN38516750.conf` 是
   **08-07 10:12** 的、内容正常（`Baseline=119.859` mm ≈ ZED 2i 标称 120 mm）→ **需重测**
   （要独占相机，即短暂停数采 dataflow；注意 `docs/06` §2.4 的连带停 LiveKit）。

**证伪 2 —— Fast-RTPS 共享内存不是根因。** 我曾猜 B1/B3/B6 都表现为「话题注册了但没数据」，
共享 `RTPS_TRANSPORT_SHM ... fastrtps_port7002: open_and_lock_file failed` 这一个根因。
**用 root 查过，不是**：`/dev/shm` 62 G 只用了 332 M（1%）；197 个 `fastrtps_*` 段属主
**全是 `astrabot`**（无 root 遗留死段）；**`7002` 文件在 `/dev/shm`、`/tmp`、`/var/tmp` 全都不存在**；
近 2 h journal 里无该报错；现在起新节点 stderr 干净、能看到 **109 个话题**。
那是一次性的陈旧段，已自行消失。**三项的原因各自独立。**

- ~~影响：`observation_images_head` 可用（走 dora），**但全部 3D 感知不可用**~~
  → **已推翻（08-10）**：ROS 图上 RGB + 深度 + 点云 + IMU + TF 全在，3D 感知可用。
- ~~**下一步动作**：不是查 ROS wrapper，而是重测深度是否米制。若可用 → 打开 `SEND_DEPTH=1`~~
  → 深度已重测（米制），**且完全不用碰 `SEND_DEPTH`**：走 `zed_wrapper` 的 ROS 话题即可。
  桌面平面能自动拟合，但**绝对高度仍差 ~5 cm**（卷尺才是基准），所以 `docs/05` E1.2 不能删。

#### B2 — 3 路 webcam 的 udev 软链不存在

`data_collection.yaml` 要 `/dev/l_arm_cam`、`/dev/r_arm_cam`、`/dev/f_chest_cam`，当时实测**三个都不存在**（`ls: No such file or directory`）。

> ⚠️ **2026-08-11 更新**：前两个已由 udev 规则建好，但 **`r_arm_cam` 现在永远不会出现** —— 右腕单目已拆，换成 DaBai DW2（libusb 厂商类，无 `/dev/video*`）。dora 配置里那个 `right_wrist` 节点**打不开设备了**，要删掉或改指向 DW2 的 ROS 话题。见 `../PITFALLS.md` §48。
→ 数采 flow 起不来。需要写 udev 规则（注意 §3.1 的左右腕同型号陷阱）。

#### B3 — 左右夹爪反馈恒为 0

`Astrabot_Controller.log` 每 30 s 一次，持续 30+ 分钟：

```
=== Gripper Message Reception Summary ===
Left  gripper: Total received=0, processed=0, errors=0
Right gripper: Total received=0, processed=0, errors=0
```

`errors=0` 但 `received=0` → 不是通信报错，而是**根本没有消息进来**（链路没接通 / 上游没发 / 夹爪未上电）。
→ **抓取的成败无法在软件侧确认**，S2/S3 判据没法自动判。这是"抓积木"最致命的一项。

**2026-08-07 复检**：`/astrabot/gripper_{left,right}_state` 现在**有帧了**（Publisher count 1，
`ros2 topic echo --once` 能返回），但内容恒为：

```
data: [0, 0, 0, 0]
```

即驱动照常发布**零填充默认值**。

> ⚠️ **2026-08-07/08-10 更正：** 原文写"根因是两个 NiMotion 夹爪驱动器在 CAN 总线上不发任何帧，
> 属供电/接线，修法在物理层"。**这是错的，而且错得有代价** —— 它把结论推向"只能等人到现场接线"。
> 真相:这两个恒零话题属于**配错厂商**的那份厂家 SDK（NiMotion + CAN 节点 101/102）；
> 本机真夹爪是 **UFactory G2 走 Modbus RTU**（左 `/dev/ttyAMA5`、右 `/dev/ttyUSB0`，@2000000 slave 8），
> 从未被使能。纯软件解决，两侧现已闭环（状态 20.5 Hz，`xr1.py grip` 实测 838→13 / 2→813 mm）。
> "CAN 上没有帧"是真的，它证明的是**夹爪不在 CAN 上**，不是夹爪坏了。
> 见 `08_gripper_g2_driver.md` 与 `../PITFALLS.md`。**卡住自动评分的真缺口是没有力反馈**（B8）。

#### B4 — `data_collection.yaml` 6 处占位符未填

`<YOUR_ZED_SERIAL>`、`<YOUR_ROBOTD_SERVER_IP>`、`<YOUR_SKILL_CONFIG_PATH>`、`<YOUR_SKILL_LIBRARY_PATH>`、`<YOUR_NAME>`、lakeFS 三件套（endpoint/access key/secret）。
另：`task_id: "astrabot:my_task"` 也是默认值，`live_upload: true` 在凭证没填的情况下会在录制中途失败。

#### B5 — 现有链路是「遥操」，不是「自主」

`astra_data_collection` 的录制由**人按手柄**触发：`Button_A` 开停录制、`Button_X` 推进 skill、`Button_B` 复位、右摇杆打 episode 质量标签，动作来自 `add_teleop` 解 LiveKit 字节流。
代码仓里**没有**"自主执行 skill 并自动录制"的入口，`skill_library/astrabot/` 里**没有任何积木/blocks/stack 相关 skill**（现有的是餐厅场景：dish_box / 加冰 / 倒液体 / 封口）。

→ 「自主」这一步需要**新写**：目标检测 → 抓取位姿 → `/ee/pose_cmd` 或 MPC target → 夹爪 → 自动触发 recorder（绕过 `Button_A`）。
→ 也需要**新写**积木 skill YAML（格式见 §5，照抄即可）。

#### B6 — `/ee/pose` 没有数据（原为「没有发布者」）

订阅 `/ee/pose` 与 `/astrabot/height` 均超时无数据（话题**注册了但没人发**）。

**复检更新（2026-08-07）**：`/ee/pose` 现在 `Publisher count: 1`，类型
`geometry_msgs/msg/PoseArray`（之前是 0 个发布者）—— 但 `ros2 topic hz` 10 s 仍无数据。
所以发布者进程起来了、话题也建了，**数据没流出来**。与 B1 同一症状，见上面的
Fast-RTPS 共享内存线索。

**FK 绕法已经不只是「绕法」了**：本仓的 FK 已用真机 `/tf` 验证 ——
q=0 时 `base_link→left_tcp_link` 实测 `[-0.072, 0.461, 0.402]`、
`right_tcp_link` `[-0.072, -0.449, 0.398]`，本仓计算吻合到 **0.5 mm**（TF 打印精度上限）。
即 FK 路线精度足够，B6 可以**降为非阻塞**，不必挡住阶段三。

- 影响 1：数据集必填字段 `observation_states_ee_pose_left/right`（各 [9] 维）**采不到**
- 影响 2：笛卡尔入口 `/ee/pose_cmd` **没有位姿反馈可用于闭环收敛判断** —— 只能开环发指令，无法判断"到位了没"
- 对比：`/joint_states` 是好的（14 关节，实测全在零位，量级 1e-6 rad，速度 0）
- → 短期绕法：自己用 URDF 跑 FK 从 `/joint_states` 算 EE 位姿；长期要查为什么发布者没起来

#### B7 — ~~腕部相机纯噪声~~ ✅ **已解决**

原因：**镜头保护盖未打开**。2026-08-07 10:45 打开后两路复测正常成像。保留在此作为排查留档。

### 🟡 需注意

| # | 项 | 说明 |
|---|---|---|
| W1 | `controller_manager` 200 Hz 偶发 overrun（`loop took 6.909 ms, missed cycles: 2`） | 目前是零星告警，但自主回路加上视觉推理负载后可能恶化 → 必须监控 |
| W2 | `wait-controller-node.sh`、`wait-livekit-ready.sh` 仍在轮询 | 说明启动链有环节未就绪 |
| W3 | URDF 的 effort/velocity 全 = 3.1（占位值） | **不能用于力控/力度推导**，见 §2.2 警告 |
| W4 | `platform_joint` 在 URDF 里是 fixed | 但躯干有 `/astrabot/height` 等命令话题 → 真实升降能力需实测，会直接影响桌面可达性 |
| W5 | Python 版本割裂：ROS 侧 3.12 / 数采 venv 3.10 | 写桥接代码时别混用 |
| W6 | `ROS_DOMAIN_ID=12` 必须显式 export | 否则误判"机器人没起来" |
| W7 | 机器人自诊断 `worst_level = WARN`，唯一 active fault：`ABT-THOR-SYS-GPU-UNAVAILABLE`（dtc `0x11101005`）"gpu metrics unavailable" | `nvidia-smi -L` 能列出 Thor GPU，说明 GPU 本身在，是**遥测采集**失败（Thor 上常见 tegrastats/nvidia-smi 差异）。不直接阻塞任务，但会让"算力是否吃紧"没法监控 —— 视觉推理上线后需要这个指标 |
| W8 | 场景里有**另一台机器人**在同一房间（见 `03_scene_physical.md`） | 需确认其 `ROS_DOMAIN_ID` 不是 12，否则 DDS 会串话题 |

### 🟢 已具备（可直接用）

- 双 7-DOF 臂 + 平行夹爪，`joint_state_broadcaster` + `forward_position_controller` **active**，`/joint_states` 有真实数据
- **~~3 路~~ 2 路 webcam 实拍成功**（左腕 / 胸部鱼眼）—— 右腕那路已于 2026-08-11 拆除，换成 DaBai DW2 深度相机
- 笛卡尔入口 `/ee/pose_cmd` + 全身 MPC target 均在线（但注意 B6：无位姿反馈）
- 底盘/Nav2/雷达/超声/IMU/里程计 + `collision_monitor` 全部在线（安全兜底可用）
- 底盘双 Orbbec 深度相机在发数据 —— **ZED 修不好时的备选深度源**（但视角是底盘，看桌面不理想）
- LeRobot dataset schema 完备，`skill_id` per-frame 可区分策略
- `dora` + `astra` CLI 装好可用
- **MuJoCo 仿真镜像存在**（`/astrabot/joints_mujoco` + `skill_library/sim_astrabot/`）→ 阶段三先在仿真里跑，别拿真机试错
- RT 实时内核 + Jetson Thor 算力充足

---

## 7. 结论

> **不具备**立即开始「自主搭建积木 + 采集数据」的条件。剩余 **5 项**硬阻塞（B7 已解决）。
>
> 解锁顺序（**2026-08-07 二次修订：Fast-RTPS 共享内存已从榜首删除 —— 它被证伪了**）：
> **① B3 夹爪** → **② ZED 深度重测** → **B2 udev（胸部）** → **B4 配置** →
> **B5 自主入口 + 积木 skill**。
> **B6 降为非阻塞**（FK 路线已用真机 TF 验证到 0.5 mm）。
>
> - ~~**① Fast-RTPS 共享内存**（疑似 B1/B3/B6 共同根因）~~ → ❌ **假设已证伪**，见上面 B1 复检更新之二。
>   `/dev/shm` 空得很、无 root 遗留段、`7002` 文件不存在、journal 无报错。**三项原因各自独立。**
>   这条教训值得留着：**"多个症状看起来一样"不构成共同根因的证据。**
> - ~~**B3 排最前**：夹爪不通……现在知道它是**硬件**（CAN 上无帧 → 供电/接线），软件侧无从下手，需要人动手查线。~~
>   → **整个诊断方向是错的，B3 已关闭。** 夹爪根本不在 CAN 上：它们是 **UFactory G2 走 Modbus RTU**
>   （左 `/dev/ttyAMA5`、右 `/dev/ttyUSB0`，@2000000 slave 8），只是**从未使能**。
>   现在两侧都是闭环可用的（状态 20.5 Hz，`xr1.py grip` 实测 838→13 / 2→813 mm）。
>   "CAN 上没有帧" 是真的，但它证明的是"夹爪不在 CAN 上"，**不是**"夹爪坏了" ——
>   见 `../PITFALLS.md` 元教训 5、`08_gripper_g2_driver.md`。
> - 剩下的真问题是**"抓到没抓到"仍无法用力反馈判定**（`effort` 全 `.nan`，B8），
>   唯一代理是 `gripper_cmd.py --ramp` 的 `pos_mm` 卡滞。
> - ~~**B1 排第二，但形态变了**：不是"修 ROS wrapper"（那是找错了地方，真通路在 dora），
>   而是**重测深度是否米制**。~~ → **B1 已关闭（08-10）：通路就是 ROS，深度是米制的。**
>   "真通路在 dora" 这句是当时 domain 设错导致的误判。ZED 是唯一装在能看到桌面的位置上的深度源
>   （腕/胸无深度硬件；底盘 Orbbec 有真米制深度但离地 7 cm 朝侧面，前方桌面区零个点）。
> - **B6 有短期绕法**（用 URDF 跑 FK 从 `/joint_states` 算 EE 位姿），不必卡住。
> - **ArUco 视觉真值优先级下调**（原先是"并行主线、今天就能开工"）：
>   FK 已验证 0.5 mm，积木在爪子里时 FK 就是积木位置 → **抓不需要标定，放/叠才需要**。
>   而且本机腕部/胸部相机**无任何现成内参文件**，胸部连 udev 规则都没有。
>   见 `docs/05_experiment_protocol.md` §3 E1.0–E1.2。
> - B1 / B2 / B6 可与 B3 **并行**推进（不同人 / 不同 sub agent）。
>
> 已消耗的排查成本可参考：B7（腕部相机）从"看起来是模组损坏"到定位为"镜头盖没打开"，
> 说明**先做最便宜的物理检查**比先怀疑软件更划算 —— 这条经验直接适用于 B1 和 B3。
