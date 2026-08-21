# 硬件与配置权威地图

> ⚠️ **文中出现的 `scripts/xr1_verify.py`、`scripts/arm_unit_probe.py`、
> `scripts/gripper_cmd.py` 都已不在磁盘上**
> ([ADR 0003](../decisions/0003-lost-python-pipeline.md))。所有「去跑它」的地方
> 已改写成现在真实存在的检查:`py/xr1.py pose`(关节/夹爪反馈)、`bin/tf-frames`
> (TF 总数 52 / zed 6)、`py/xr1_cam.py doctor`(录制器)。剩下的脚本名只作为
> **某个数字是怎么测出来的**的标签保留。**硬件数字本身仍然有效。**
>
> **这份文档的每一行都来自本机实测**，不是抄手册、不是推测。
> 采集时间：2026-08-07 15:50–16:40 CST · 主机 `tegra-ubuntu` (192.168.123.102)
> 采集手段：`lsusb -t` / `udevadm info` / `/sys/class/tty` / `ip -details link` /
> `ros2 node|topic list` / `ros2 control list_*` / 只读 Modbus 0x03 扫描 / `/proc/<pid>/environ`
>
> 凡是**没有实测到**的，本文一律写「未验证」而不是猜一个值。
> 配套文档：[`08_gripper_g2_driver.md`](gripper-g2.md)（夹爪专题）

---

## 0. 先读这一条：`ROS_DOMAIN_ID=12`

**这是这台机器上最容易踩、后果最隐蔽的一个坑。**

整台机器人的 ROS 2 图跑在 **domain 12**。而你手工开一个终端 `source /opt/ros/jazzy/setup.bash`
之后 `ROS_DOMAIN_ID` 是**未设置的（=0）**。两个域在 DDS 层面完全隔离：

- `ros2 topic list` 只看得到你自己起的节点，看起来「机器人什么都没跑」
- `ros2 control list_controllers` 卡在 `waiting for service ... to become available`
- 你起的节点发的命令，机器人**收不到**；机器人发的话题，你**看不到**
- 而且**不会报任何错** —— 只是安静地什么都不发生

2026-08-07 实测对比：

| | domain 0（手工终端默认） | domain 12（机器人真实所在） |
|---|---|---|
| 节点数 | 1（只有我自己起的） | **44** |
| 话题数 | 6 | **109** |
| `/joint_states` | 不存在 | 存在，16 关节 |
| `/controller_manager` | 不存在 | 存在 |

**每个手工终端、每个脚本，都必须先 export：**

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/astrabot/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### 这个 12 是从哪来的

不是环境变量、不是 `/etc/environment`、**也不在 `.bashrc` 里**。是启动框架写死的兜底值：

```
/opt/ros/start_up/function/environment.sh:31   ROS_DOMAIN_ID_FROM_BASHRC=$(grep -E '^export[[:space:]]+ROS_DOMAIN_ID=' /home/astrabot/.bashrc | tail -n1 | cut -d= -f2- || true)
/opt/ros/start_up/function/environment.sh:35   export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_FROM_BASHRC:-12}
```

逻辑是「先从 `/home/astrabot/.bashrc` 里抓 `export ROS_DOMAIN_ID=`，抓不到就用 12」。
本机 `.bashrc` 里**没有**这一行（已 grep 确认），所以取兜底的 **12**。

已用 `/proc/864478/environ` 直接验证：`ros2_control_node` 的环境里确实是 `ROS_DOMAIN_ID=12`。

> **可选的一劳永逸做法**：往 `/home/astrabot/.bashrc` 追加 `export ROS_DOMAIN_ID=12`。
> 这样交互终端和启动框架就永远一致。但注意 `/opt/ros/start_up/function/execute.sh:133-138`
> 有一段会 `sed -i` 改写这一行，所以别指望它不被工具覆盖。

### 另一个同源的坑：ZED / webcam 起在了错误的域

2026-08-07 期间用 `setsid nohup` 手工拉起的 ZED 和 webcam 节点，因为没 export domain，
全都落在 domain 0 —— 也就是**机器人自己看不见自己的相机**。
`domain 12` 里 grep `zed` 一个话题都没有。手工拉任何相机节点时务必带上 domain。

---

## 1. 平台

| 项 | 实测值 |
|---|---|
| 板卡 | `NVIDIA Jetson AGX Thor Developer Kit`（`/proc/device-tree/model`） |
| L4T | **R38 (release), REVISION 2.1**, GCID 42061081, 2025-09-10 |
| `nvidia-l4t-core` | `38.2.2-20250925153837` |
| 内核 | `Linux 6.8.12-rt-tegra aarch64`（**PREEMPT_RT**，`KERNEL_VARIANT: oot`） |
| 发行版 | Ubuntu 24.04.3 LTS |
| 主机名 | `tegra-ubuntu` |
| CPU | 14 核 aarch64 |
| 内存 | 122 GiB 总 / 约 10 GiB 已用 / 约 111 GiB 可用 |
| 负载 | `31.25 30.76 32.31`（14 核上跑到 31，**长期过载 ~2.2×**，见 §11） |
| 根分区 | 467 G，已用 10%，剩 401 G |

## 2. 网络

| 接口 | 状态 | 地址 | 说明 |
|---|---|---|---|
| `enP1p1s0` | UP | **192.168.123.102/24** | 机器人主网，Mac 上位机在 192.168.123.138（用户 `apple`，免密 key `~/.ssh/id_xr1rec`，挂着外置录制摄像头）|
| `l4tbr0` | DOWN | 192.168.55.1/24 | Jetson USB-device 模式的自带桥，未用 |
| `docker0` | DOWN | 172.17.0.1/16 | 未用 |

> 📌 **两台机器的 IP / 账号 / 密码 / 摄像头清单在仓库根目录的 [`AGENTS.md`](../../AGENTS.md)**
> （含免密 SSH 与 `ROS_DOMAIN_ID=12` 互通说明）。本文档只覆盖机器人这一侧的硬件。

`/etc/hosts` 里**没有** `tegra-ubuntu` 条目 —— 所以每次 `sudo` 都会先甩一句
`sudo: unable to resolve host tegra-ubuntu`。不影响功能，但会污染所有脚本输出，
建议补一行 `127.0.1.1 tegra-ubuntu`。

## 3. ROS 2 栈的三层布局

| 路径 | 是什么 |
|---|---|
| `/opt/ros/jazzy` | 上游 ROS 2 Jazzy |
| `/opt/ros/astrabot` | 厂家 overlay（**只有 install 空间，没有源码**，见 §10 警告） |
| `/opt/ros/start_up` | 厂家自研的启动框架，**不是 systemd unit** |
| `/home/astrabot/gripper_ws` | 夹爪驱动工作区（本次新建，见 `08_*.md`） |
| `/home/astrabot/deploy` | 数采 / 遥操作部署目录，自带 `.venv` |

`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（在 `/opt/ros/start_up/config/ros_config.sh` 里 export，
注意那个文件**只** export 了 RMW，没有 export domain）。

### 启动框架怎么用

单元定义在 `/opt/ros/start_up/auto_start_script/*.start_script`，共 18 个：

```
Astrabot_Backend      Astrabot_Controller   Astrabot_Diagnostics  Astrabot_File_Transfer
Astrabot_Frontend     Astrabot_Gateway      Astrabot_Log_Agent    Astrabot_Log_Hub
Astrabot_Mapping      Astrabot_Mpc          Astrabot_Nav2         Astrabot_Poi
Astrabot_Remote_Shell Astrabot_Ros_Monitor  Astrabot_Shutdown     Astrabot_Task
Astrabot_ZED          Astrabot_ZED_Points
```

对应的启动脚本在 `/opt/ros/start_up/run/*.sh`，日志在
`/opt/ros/start_up/log/<单元名>/<MMDD>/<HH.MM.SS>/`。

**实测正在跑的 launch（domain 12）**：

```
astrabot_controller_bringup  astrabot_xr1_evt2_arm.launch.py
astrabot_hwcontrol           astrabot_xr1_evt2_arm_head_forward_ros2_control.launch.py
astrabot_diagnostics         diagnostics.launch.py
astrabot_nav_bringup         diff_drive_astrabot_mppi.launch.py
astrabot_slam_bringup        localization_launch.py
astrabot_poi_manager         poi.launch.py
nav2_task_orchestrator       task_orchestrator_launch.py
astrabot_remote_shell        remote_shell.launch.py
astrabot_shutdown            shutdown_node.launch.py
```

~~**`Astrabot_ZED` 当前没在跑**（`ps` 里没有任何 zed 进程）~~
→ **2026-08-10 复核：在跑。** 这是 08-07 的瞬时状态，不要当结论读。
它当时处于 failed 状态，恢复自启用
`systemctl reset-failed Astrabot_ZED && systemctl start Astrabot_ZED`（这条仍然有效）。
**判断它现在活不活：`py/xr1.py pose` ＋ `bin/tf-frames`。**

> 注意 `Astrabot_Controller.sh` 于 2026-08-07 被改过：原来启的是 arm-only 的
> ros2_control launch，改成了 `..._arm_head_forward_ros2_control.launch.py` 以便把颈部
> 一起纳入 ros2_control。备份 `Astrabot_Controller.sh.bak-20260807-114240`。
> 效果已验证：`AstraNeckHW` 现在是 `active`，`head_yaw/pitch_joint` 出现在 `/joint_states` 里。

## 4. CAN —— 手臂驱动器

| 接口 | 状态 | 比特率 | 挂了什么 |
|---|---|---|---|
| `can0` | **DOWN** | 1 Mbps | 未使用 |
| `can1` | UP | 1 Mbps | **左臂** 7 个驱动器，node id 11–17 |
| `can2` | UP | 1 Mbps | **右臂** 7 个驱动器，node id 21–27 |
| `can3` | **DOWN** | 1 Mbps | 未使用 |

配置来源 `/opt/ros/astrabot/share/astrabot_actuator_sdk/config/astrabot_arm_actuator_config.yaml`，
厂家 vendor 字符串一律 `"eyou"`（亿佑），减速比：

| node id | 总线 | 减速比 | 关节 |
|---|---|---|---|
| 11 | can1 | 81 | `left_arm_1_joint` |
| 12 | can1 | 101 | `left_arm_2_joint` |
| 13 | can1 | 51 | `left_arm_3_joint` |
| 14 | can1 | 51 | `left_arm_4_joint` |
| 15 | can1 | 51 | `left_arm_5_joint` |
| 16 | can1 | 101 | `left_arm_6_joint` |
| 17 | can1 | 101 | `left_arm_7_joint` |
| 21–27 | can2 | 81/101/51/51/51/101/101 | `right_arm_1..7_joint` |

其它同文件里的关键参数：`update_rate: 200.0`、`thread_priority: 50`、
`tolerance_check_enable: true`、`tolerance_pos_list: [0.8,0.8,1.4,1.4,1.4,1.0,1.0] ×2`、
`actuator_direction_list` 全 1、`gohome_enable: false`、`publish_joint_states: false`、
`can_delta_enable: false`、`tpdo2_enable: false`。

> **夹爪不在 CAN 上。** 对 can1+can2 全部 127 个 node id 做过完整扫描，只有上表这 14 个
> 驱动器应答。详见 [`gripper-g2.md`](gripper-g2.md)。

## 5. 串口全表

**Thor 的原生 UART 是 ARM PL011，设备节点是 `/dev/ttyAMA*`。**
不是老 Jetson 那套 `/dev/ttyTHS*` —— 本机 `/dev/ttyTHS*` 根本不存在。这一点找错过一次。

| 设备节点 | 物理层 | 驱动 | 用途（实测） | 当前占用者 |
|---|---|---|---|---|
| `/dev/ttyUSB0` | CP2102N，USB `1-3.1` | `cp210x` | **右手夹爪** G2 | `g2_gripper_node` |
| `/dev/ttyAMA5` | PL011 `810c510000.serial` | `port` | **左手夹爪** G2 | `g2_gripper_node`（同一个进程同时持 `ttyAMA5` + `ttyUSB0`，实测 PID 34011）|
| `/dev/ttyAMA4` | PL011 `810c500000.serial` | `port` | **不明**。6 种波特率 × 5 个 slave id 全静默 | 空闲 |
| `/dev/ttyAMA10` | PL011 `810c540000.serial` | `port` | **头颈 SM45BL 舵机** | `ros2_control_node` (PID 864478) |

- 权限一律 `crw-rw---- root:dialout`，`astrabot` 已在 `dialout` 组，不需要 sudo。
- `/dev/ttyUSB0` 的稳定标识：`ID_SERIAL_SHORT=d60f7389ced5ef118820724b49d2c684`，
  `ID_PATH=platform-a80aa10000.usb-usb-0:3.1:1.0`。
  **它是本机唯一的 USB 串口转换器**，所以枚举顺序不会变，但如果以后再插一个，
  建议按上面这个 `ID_SERIAL_SHORT` 写 udev 规则固定住。
- ⚠️ **`/dev/ttyAMA10` 绝对不要碰。** 它被 `AstraNeckHW` 独占，打开它会抢掉头部控制。

## 6. USB 拓扑全表

Bus 001 = USB 2.0（`tegra-xusb`，480M），Bus 002 = USB 3.2（20000M/x2）。

```
Bus 001 root_hub (tegra-xusb/4p, 480M)
├── 1-2      Microchip 0424:2512  2-port hub
│   └── 1-2.2   2b03:f881  STEREOLABS ZED-2i HID INTERFACE   Driver=[none]  ⚠
├── 1-3      Genesys  05e3:0610  4-port hub
│   ├── 1-3.1   10c4:ea60  Silicon Labs CP210x               cp210x   → /dev/ttyUSB0 右手夹爪
│   ├── 1-3.2   2bc5:069f  ORBBEC Depth Sensor (DaBai DW2)   Driver=[none] ← 正常，libusb 设备
│   ├── 1-3.3   0bda:5856  Realtek USB Cam                   uvcvideo  ← 胸部相机
│   └── 1-3.4   14cd:8601  Super Top 4-port hub
│       ├── 1-3.4.2  0d8c:0014  C-Media Audio (Unitek Y-247A) snd-usb-audio + usbhid
│       └── 1-3.4.3  1a86:7523  QinHeng CH340                 Driver=[none]  ⚠ 见 §11
└── 1-4      Genesys  05e3:0610  4-port hub
    └── downstream Realtek hub
        ├── 8086:0b5b  Intel RealSense D405                  uvcvideo  ← 右手近场深度，当前仅 480M
        └── 1a86:7523  QinHeng CH340                         Driver=[none]  ← 触觉候选，协议/映射待验

Bus 002 root_hub (tegra-xusb/4p, 20000M/x2)
├── 2-1      2b03:f880  STEREOLABS ZED 2i                    uvcvideo → video0/1  (5000M)
├── 2-2      05e3:0620  Genesys GL3523 hub                   （空）
└── 2-3      05e3:0620  Genesys GL3523 hub                   （空）
```

2026-08-18 真机复核确认 D405 序列号为 `262422270599`；当时它在 480M USB 2.0
链路，流曾出现 protocol error、断开和重新枚举。2026-08-21 复核时设备已在 5000M
USB 3.x 链路，`848x480@15` 连续 20 帧实测 16.25 Hz。软件仍只在每次动作前取得连续
新鲜 RGB/深度帧并通过帧间隔、有效深度检查时，临时授权一个有界近场事务。

同次复核看到两颗 `1a86:7523` CH340。它们可能是操作者说明的触觉垫片聚合器，但当前
内核没有 `ch341` 驱动，因此没有 tty 节点，115200 协议也无法取帧。`/dev/ttyUSB0`
仍是右 G2 夹爪的 CP2102N/2 Mbaud 端口，绝不能按触觉 115200 打开。

**关键：两个腕部相机型号完全相同（`1bcf:2cd1`，名字都是 `DECXIN  CAMERA`，
连 `ID_SERIAL` 也一样 —— `DECXIN_CAMERA_DECXIN_CAMERA_01.00.00`，固件写死的常量）**，
所以 udev 只能靠**物理端口号**区分，规则必须匹配 `ID_PATH`。

> 🔴 **但由此推出的「哪个口是哪一侧」不是硬结论，而是一句关于插线的断言。**
> 2026-08-11 17:15 的单变量拔线实验证明当时 **4.4 = 左腕**（与厂商规则相反）；
> 17:29 操作者改插到 4.3，规则又对了。**重插一次侧别就变，且零痕迹。**
> 判断当下只有两招：让人拔一根线看内核掉哪个口，或抓一帧看画面里有没有同侧橙色夹垫。
> 完整叙述见 `../operations/pitfalls.md` §48。

## 7. 相机 / video 全表

| 节点 | v4l2 name | USB 物理口 | `ID_PATH` | 角色 | 稳定符号链接 |
|---|---|---|---|---|---|
| v4l2 name | USB 物理口 | `ID_PATH` | 角色 | 稳定符号链接 |
|---|---|---|---|---|
| `ZED 2i: ZED 2i` | `2-1` | `...usb-0:1:1.0` | 头部立体相机 | （由 zed_wrapper 按序列号打开，不要直接开） |
| `USB Cam: USB Cam` | `1-3.3` | `...usb-0:3.3:1.0` | **胸部**广角 | **`/dev/f_chest_cam`** |
| `DECXIN  CAMERA` | `1-4.3`（当前） | `...usb-0:4.3:1.0` | **左腕**单目 | **`/dev/l_arm_cam`** |
| `ORBBEC Depth Sensor` | `1-3.2` | — | **DaBai DW2** 深度（操作者称装在右手） | **没有 `/dev/video*`**，只能走 ROS 话题 |

**上表故意不写 `video<N>` 编号 —— 每次插拔都变**（2026-08-11 当天：胸部 video2→video3、
腕部 video6→video2）。写进文档的编号一定会骗下一个读者。

每台 UVC 相机占两个 node，分辨图像节点的**唯一**判据是 **`ATTR{index}=="0"`**，
udev 规则里必须带它，否则符号链接会随机指到 metadata node 上。

> ⚠️ ~~偶数号是出图的、奇数号是 metadata~~ —— **这句原来写在这里，是错的，2026-08-11 实测推翻。**
> 当前实测：胸部 **video3 = 图像（index=0）/ video8 = metadata（index=1）**，
> 腕部 **video2 = 图像 / video5 = metadata**。编号奇偶不含任何信息。

### ~~左右腕已三重确认，不需要再查~~ → **2026-08-11 推翻。那"三重"是零重。**

原文列的三条"独立证据"，实际上**一条都不独立**：

| 原"证据" | 为什么不算 |
|---|---|
| ① 厂商 udev 规则写 `4.3→l_arm_cam` | 这是**待验命题本身**，不是证据 |
| ② 两个 `webcam_node` 分别持有 `l_arm_cam` / `r_arm_cam` | 这两个符号链接**就是规则①生成的** —— 循环引用 |
| ③ 操作者口述「usb2 对应右臂，usb1 对应左臂」 | `usb1/usb2` 与 `4.3/4.4` 之间**从来没有定义过对应关系**，接不上 |

**教训比结论重要**：三条证据都来自同一个源头时，"三重确认"给的是三倍的信心和零倍的信息。
真正能定案的判据只有两个，都必须**绕开 udev 规则**：

1. **单变量拔线实验** —— 让人物理拔掉某一侧，看内核掉的是哪个口（`dmesg | grep 'usb 1-4'`）。
   ⚠️ **不能用插拔史反推** —— 插拔史只告诉你哪个口在动，不告诉你哪个口是哪一侧（我在这上面错过一次）。
2. **抓一帧看画面里有没有同侧橙色夹垫** —— 真装在某侧腕上的相机必然看得见同侧夹垫。
   检测器：HSV `(5,120,80)-(25,255,255)` + `MORPH_OPEN`，脚本 `data/experiments/wristcam/grab.py`。

> 原文那句"我一度打算再写一份自己的腕部 udev 规则，已丢弃 —— 厂家那份已经对了"
> **也作废了**：厂商那份当时是反的，现在已装 `/etc/udev/rules.d/99-astrabot-wrist-camera.rules`
> 覆盖它（**同名文件整体覆盖** `/usr/lib` 那份，所以不会"两份打架"，也抗厂商重装）。

### udev 规则清单

| 文件 | 来源 | 内容 |
|---|---|---|
| `/usr/lib/udev/rules.d/99-astrabot-wrist-camera.rules` | 厂家自带（`dpkg -S` **查不到归属**，手工装的，无人记录谁确认了接线） | `4.3→l_arm_cam` + `4.4→r_arm_cam`，**左右取决于当时接线** |
| `/etc/udev/rules.d/99-astrabot-wrist-camera.rules` | **2026-08-11 新增，覆盖上面那份**（同名整体覆盖，抗厂商重装） | **两个口都 → `l_arm_cam`**；不再定义 `r_arm_cam` |
| `/etc/udev/rules.d/99-astrabot-chest-camera.rules` | 2026-08-07 新增 | `f_chest_cam` |

⚠️ 覆盖版之所以让**两个口都映射到 `l_arm_cam`**：现在全机只剩一台单目腕相机且是左边的，
插哪个口都认（17:29 操作者改口正是靠这一点才没出问题）。
**哪天右腕又装回单目，这两行必须改回按口区分**，否则两台抢同一个符号链接。

新增胸部规则的原因：`/home/astrabot/deploy/.astra/astrabot_data_collection.yml:129` 要求
`/dev/f_chest_cam`，但厂家的腕部规则文件里**没有**胸部条目，导致这个链接一直不存在，
数采配置指向一个空路径。现在三个链接齐了：

现在只有**两个** UVC 符号链接（编号会变，用 `ls -l /dev/*_cam*` 现查，别抄这里）：

```
/dev/f_chest_cam    胸部广角
/dev/l_arm_cam      左腕单目
```

**没有 `/dev/r_arm_cam`** —— 右腕原来那台 DECXIN 单目已拆（为装 DaBai DW2），
DW2 是 libusb 厂商类设备，**不出 `/dev/video*`**，只能走 ROS 话题。

DW2 **没有 RGB**，这是器件本身的硬结论，不是驱动没配好：OrbbecSDK 自己在
`OpenNISensorIO.cpp:65` 打印 `Image endpoint is not supported...`（2026-08-11
18:14:10 的 SDK 日志），USB 侧也只列出一路 Depth interface。所以
`enable_color:=true` 传给 launch 会被静默丢掉，不会报错。它已由右腕 D455 替代
（见第 9 节），本条留着是因为拆下来的器件还在。

改完 udev 后生效：`sudo udevadm control --reload && sudo udevadm trigger`。

## 8. ros2_control 现状

`ros2_control_node`（PID 864478，domain 12）起在：

```
--params-file astrabot_xr1_controller.yaml
--params-file astrabot_arm_actuator_config.yaml     ← 手臂 + （原）夹爪配置都在这
--params-file astrabot_motion_list.yaml
-r ~/robot_description:=/robot_description
```

**硬件组件**（`ros2 control list_hardware_components`）：

| 名称 | 类型 | 插件 | 状态 | 速率 |
|---|---|---|---|---|
| `AstraNeckHW` | actuator | `astrabot_neck_v1/AstrabotNeckHW` | **active** | 200 Hz |
| `AstraBodyHW` | system | `astrabot_body_v1/AstrabotBodyHW` | **active** | 200 Hz |

**控制器**（`ros2 control list_controllers`）：

| 名称 | 类型 | 状态 |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `astrabot_arm_forward_position_controller` | `forward_command_controller/ForwardCommandController` | active |
| `astrabot_neck_forward_controller` | `forward_command_controller/ForwardCommandController` | active |

两个手臂控制器都是 **ForwardCommandController + position 接口** —— 也就是说
**没有轨迹插值，你发什么位置它就直接下什么位置**。上层必须自己做限速/平滑，
否则一条大跨度指令会变成一次全速冲击。

三条使用它之前必须知道的事：

1. **一次必须给全 14 个关节**（左 7 后右 7 一个数组）。想动一只手臂，就得同时给另一只
   下「保持」—— 而「保持」值**不能**从 `/joint_states` 抄，单位不一样，见 §9 的红框。
2. **`/astrabot_arm_forward_position_controller/commands` 上还有另外两个发布者**：
   `astrabot_mrt` 和 `astrabot_actuator_sdk`。静止时它们不发，但**有能力抢占**。
   下指令前先 `ros2 topic hz` 看一眼有没有别人在发。
3. **没有任何力/力矩/接触反馈**：`/joint_states` 的 `effort` 在**全部 16 个关节上都是 `.nan`**，
   `velocity` 手臂恒为 `0.0`、头部为 `.nan`。后果见 §11 D10。

## 9. 关节与 URDF

`/joint_states`（domain 12）实测 **16 个关节**，顺序如下：

```
head_pitch_joint  head_yaw_joint
left_arm_1_joint  … left_arm_7_joint
right_arm_1_joint … right_arm_7_joint
```

2026-08-07 16:36 实测姿态：**14 个手臂关节全部精确读到 0.0000**（=零编码器计数，
即回零位，垂在身侧），`head_yaw_joint` 0.0000，`head_pitch_joint` **+0.6632 rad = +38.00°**。

> ### 🔴 手臂反馈在「控制器收到第一条指令之前」是无效值
>
> **这是本机最危险的一个坑，比 domain 12 更容易出事**（domain 错了只是没反应，
> 这个错了会真的把关节甩过限位）。
>
> ⚠️ **本框先前的标题是「单位不是弧度」，那个结论已被实测证伪** ——
> `scripts/arm_unit_probe.py`：指令 `+0.174533 rad`（+10.0°）→ 反馈增量 `+0.174526`，
> **系数 = 1.0000，就是弧度**；5 个分步点全部精确跟随，回零后 `max|q| = 0.000008`。
> **读写单位一致，`/joint_states` 可以直接喂 FK/IK。**
>
> 危险是真的，但机制是**时序**不是单位。下面的观测数据全部有效，
> 只是解释变了 —— 那次读数发生在 `astrabot_arm_forward_position_controller`
> **收到第一条指令之前**，反馈还是未锁存的原始执行器状态：
>
> | 关节 | 报出值 | URDF 限位 (rad) | 减速比 | `值 × 减速比 / 95.8738` |
> |---|---|---|---|---|
> | `right_arm_1_joint` | 1.1836271 | ±3.100 | 81 | 1 |
> | `right_arm_2_joint` | 2.8477366 | −3.050…0.174 | 101 | 3 |
> | `right_arm_4_joint` | 3.7597568 | −2.300…0.000 | 51 | 2 |
> | `right_arm_7_joint` | 9.4924553 | ±3.100 | 101 | 10 |
> | `left_arm_1_joint` | 5.9181357 | ±3.100 | 81 | 5 |
> | `left_arm_3_joint` | 7.5195136 | ±3.100 | 51 | 4 |
>
> `值 × 减速比` 对每个关节都是 **95.8738** 的整数倍（GCD = 95.8738，n = 1…10），
> 减速比之比吻合到 5 位有效数字（`101/81 = 1.246914` vs 实测 `1.246960`）。
> 这套算术本身是真的，但它只能说明这些值是**未锁存的原始编码器量**，
> **不能推出稳态下的单位也是它** —— 这就是我当时错的地方。
>
> **发过第一条指令之后**，14 路全部落到 `±0.000006` 并稳住
> （8 秒 1530 帧，极差 3 µrad，两个发布者 `astrabot_mrt` + `joint_state_broadcaster` 数值一致）。
>
> （参考：同一条消息里的 `head_pitch_joint` 从一开始就是正确的弧度（0.6632 = 38°），
> 因为颈部是另一个插件（`AstraNeckHW` / SM45BL），它不经过手臂那条锁存路径。）
>
> **指令侧也是弧度**：`command_interface min/max = ±3.14`，xacro 里注释写着 `<!-- -180 ~ 180 -->`。
>
> **为什么这是危险而不只是别扭**：`astrabot_arm_forward_position_controller`
> 一次吃**全部 14 个关节的一个数组**（左 7 后右 7）。想动右臂就必须同时给左臂下
> 「保持当前位姿」—— 而把读回来的数当弧度回灌，会把 `left_arm_6` 送到
> 9.4925 rad = 544°，而它的限位是 ±1.57 rad。
>
> ✅ **留下来的操作规则（这才是真正的教训）：先发一条指令，再信反馈。**
> 给 14 个关节全下常量 **0.0** 作为第一条指令：0 在每个关节限位内，
> 且 FK 已证 `q=0` 就是“手臂垂在身侧”（实拍一致），所以那是个小动作。
> **绝不要在控制器刚起来时用 `/joint_states` 去构造“保持当前位姿”的指令。**
> 实测脚本：`../scripts/arm_unit_probe.py`。

**注意 URDF 有两套同名的包，用错会静默拿到错模型：**

| 包 | 说明 |
|---|---|
| `astrabot_xr1_evt2_description` | **本机是 EVT2，用这个** |
| `astrabot_xr1_description` | 非 EVT2 版本，结构相同但尺寸不同 |

EVT2 包里的文件：

```
urdf/astrabot_xr1_evt2_description.urdf              整机
urdf/astrabot_xr1_evt2_arm_description.urdf          仅双臂
urdf/astrabot_hwcontrol_xr1_evt2_arm_head.urdf.xacro ros2_control 用（当前生效）
urdf/astrabot_hwcontrol_xr1_evt2_arm.urdf.xacro
urdf/astrabot_hwcontrol_xr1_evt2_head.urdf.xacro
ros2_control/astrabot_hwcontrol_xr1_arm_head.ros2_control.xacro
```

`/robot_description` 话题在 domain 12 里有效，`robot_state_publisher` 在跑，`/tf` 可用。
本工作区的 FK 已用真机 `/tf` 交叉验证过（q=0 时 `base_link→*_tcp_link` 吻合 0.5 mm，
见 `kinematics.md`「Search lessons」）—— 也就是说 **URDF 与真机一致，可以放心用来算 IK**。

## 10. ZED 2i 状态与已做的配置改动

硬件在位：`2b03:f880 STEREOLABS ZED 2i` 挂在 Bus 002 Port 001，`uvcvideo`，5000M，
出 `video0/1`。

> **2026-08-10 复核：`Astrabot_ZED` 正在跑。** 本节原先写着"当前没有进程在跑"，
> 那是 08-07 那次抓取时的瞬时状态，已经过期。现在 `/opt/ros/start_up/run/Astrabot_ZED.sh`
> 和 `Astrabot_ZED_Points.sh` 都在，`zed_container` / `zed_points_preprocessing_container`
> 在位，`/zed/zed_node/**` 话题正常。
> **要判断它现在活不活，跑 `py/xr1.py pose` ＋ `bin/tf-frames`（第 6 节），别读这一行。**
> 文档里凡是"当前 X 没在跑"这种瞬时状态，都只能当历史记录看。

配置文件（`/opt/ros/astrabot/share/zed_wrapper/config/`）当前值：

| 参数 | 文件 | 当前值 | 备注 |
|---|---|---|---|
| `camera_model` | `zed2i.yaml` | `zed2i` | |
| `grab_resolution` | `zed2i.yaml` | `HD1080` | |
| `grab_frame_rate` | `zed2i.yaml` | **15** | **本次从 30 改小**，见下 |
| `depth.min_depth` | `zed2i.yaml` | 0.01 m | |
| `depth.max_depth` | `zed2i.yaml` | 15.0 m | |
| `depth_mode` | `common_stereo.yaml` | `NEURAL_LIGHT` | |
| `self_calib` | `common_stereo.yaml` | **false** | **本次从 true 改掉** |
| `publish_left_right` | `common_stereo.yaml` | **true** | **本次从 false 改开**，双目原图才会发 |
| `pub_resolution` | `common_stereo.yaml` | `CUSTOM` + `pub_downscale_factor: 2.0` | 发布时降一半带宽 |
| `pub_frame_rate` | `common_stereo.yaml` | 0.0 | =不限，跟随 grab |
| `point_cloud_freq` | `common_stereo.yaml` | 10.0 Hz | `point_cloud_res: COMPACT` |

改动理由：
- `grab_frame_rate 30 → 15`：**原因不是"跑不满 30 Hz"（这一行以前写错了）。**
  真实原因是 **USB 带宽/功耗余量**：HD1080@30 ≈ 249 MB/s，相机会在开流 ~10 s 后
  从 USB 总线上掉下去 —— USB3 视频端点和 USB2 HID 端点**一起**断开再重新枚举，
  日志里表现为 `CAMERA REBOOTING`。实测 ≤ ~125 MB/s 的组合全部长时间稳定。
  这是**带宽/功耗问题，不是相机坏**。若确实要 30 Hz，用 `HD720` + 30（也在这个预算内）。
- `self_calib true → false`：开机自标定会在启动时引入额外耗时和抖动。
- `publish_left_right false → true`：这是「要双摄像头原图」的开关，不开只有深度和点云。

备份：`zed2i.yaml.bak.20260807`、`common_stereo.yaml.bak.20260807`。

> ### ⚠️ 严重警告：这些改动在 install 空间里，会被覆盖
>
> `/opt/ros/astrabot/` 是**纯 install 空间，本机没有对应源码**。所有改动都会在
> **重装/升级厂家包时被无声还原**。
>
> **完整清单是 6 个文件**（2026-08-10 用"在整个 `/opt/ros` 里找 `.bak`"重新清点得到，
> 比原先这一行多出 2 个）：
>
> | # | 文件 | 改了什么 | 回退后的症状 |
> |---|---|---|---|
> | ① | `astrabot/share/astrabot_actuator_sdk/config/astrabot_arm_actuator_config.yaml` | `gripper_list` 保留条目、topic 改成 inert 名 | 每次开机 `ros2_control_node` SIGABRT |
> | ② | `astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_description.urdf` | 头部限位 ±0.698132、底盘相机 link 改名、腕部相机 origin | 头部接受 ±3.1 rad（真实行程的 4.4 倍），无任何报错 |
> | ③ | `astrabot/share/astrabot_xr1_evt2_description/urdf/**astrabot_xr1_evt2_arm_description.urdf**` | 同 ② 的相机改名 + 腕部相机 origin | **手臂那条 launch 加载的是这个文件**，只改 ② 会让两边几何不一致 |
> | ④ | `astrabot/share/zed_wrapper/config/zed2i.yaml` | `grab_frame_rate: 15` | 开流 ~10 s 后 `CAMERA REBOOTING` |
> | ⑤ | `astrabot/share/zed_wrapper/config/common_stereo.yaml` | `self_calib: false`、`publish_left_right: true` | 外参每次开机漂；拿不到左右原图 |
> | ⑥ | **`/opt/ros/start_up/run/Astrabot_Controller.sh`** | 启动 **arm_head** 而非 arm-only 的 ros2_control launch | 不加载 neck controller → `head_pitch/head_yaw` **没有命令接口**，话题直接不存在 |
>
> ⑥ 不在 `/opt/ros/astrabot/` 下，在 `/opt/ros/start_up/` 下 —— 任何"只扫 astrabot 包树"
> 的清点都会漏掉它。
>
> 因此：
> 1. 每次改都留 `.bak-<时间戳>`（已做，6 个文件都有）
> 2. **不要靠本文档做校验** —— 跑 `py/xr1.py pose` ＋ `bin/tf-frames`，
>    6 条 guard 会逐条告诉你哪个被回退了、为什么要紧、怎么改回去
> 3. 每条 guard 都验证过"在 live 上通过、在厂商 `.bak` 上失败"。**新加改动时也必须
>    这样双向验证**，否则会写出一条永远绿的假 guard（`Astrabot_Controller.sh` 就是：
>    改动只是挪了一个 `#`，两个 launch 名改前改后都在文件里）
> 4. `.bak` 是 CRLF、改过的文件是 LF，比对时用 `diff --strip-trailing-cr`，
>    否则会报"2000 多行全变了"

## 11. 已知缺陷清单（实测发现）

> 状态截至 **2026-08-10 全量复验**，**2026-08-11 新增 D13~D15**。已关闭的条目保留原文
> 并标 **已证伪 / 已修复**，因为「当初为什么会那样判断」本身就是要记住的东西（D3 / D7 / D10）。
> 自动复查:`bin/tf-frames`（当年是 `xr1_verify.py` 里的一段 **TF 检查**：frame 总数 /
> 根数 / **zed frame 数**，`<5` 直接 FAIL —— 见 D13）。

| # | 现象 | 实测证据 | 影响 / 建议 |
|---|---|---|---|
| D1 | **CH340 没有内核 tty 驱动** | `1a86:7523` @ `1-3.4.3` 枚举成功但 `Driver=[none]`，无 tty 节点。`modinfo ch341` → `Module ch341 not found`；`/lib/modules/6.8.12-rt-tegra/kernel/drivers/usb/serial/` 里没有 `ch341.ko`。08-19 操作者确认夹爪内两片压力传感器已接线且曾测到数据 | 无 tty 不等于设备不能访问；`py/tactile_adapter.py` 已提供显式 USB 路径的用户态 PyUSB/CH340 边界。仍须实测准确端口、帧字段和两贴片映射，且 `/dev/ttyUSB0` 是夹爪，绝不能误用 |
| D2 | ~~**Orbbec 深度相机没有驱动**~~ → **2026-08-11 部分证伪。`Driver=[none]` 对这颗是正常的** | `2bc5:069f ORBBEC Depth Sensor` @ `1-3.2`（serial `CH7PB4200C8`）确实 `Driver=[none]`、确实**没有 video 节点** —— 但那是因为它是 **vendor class（`bInterfaceClass 0xFF`）走 OpenNI/libusb 的设备**，本来就不该有内核驱动或 `/dev/video*`。实测已取到深度流：**640×400@15Hz**，`fx=fy=306.44, cx=317.28, cy=195.87`，FOV 92.5°×66.3°，**只有 depth+IR 没有 RGB** | 🟠 **这条是 D3 那个推理错误的重犯**（「`Driver=[none]` ⇒ 功能不可用」）—— 同一份清单里连着两条栽在同一个坑，所以那条元教训值钱：**测你关心的量（有没有帧），不要测它的代理（驱动绑定 / video 节点）**。起节点必须显式 `depth_height:=400`（厂商 launch 默认 480 是**无效值**）+ `publish_tf:=false`；udev 规则已装，免 root。✅ **08-11 定案：就是 08-10 那颗，被挪到了右腕**（操作者口述两次：「同一颗，我把它挪到右手了」「右手算了就不连接了以后也不用，右手已经有双目了」）。所以「`1-3.2` 从 08-10 就被占着」和「右手新加了一个」不矛盾 —— **换的是安装位置，不是设备**；「双目」是它两颗 IR 镜头的外观。⚠️ 这是**操作者证词，不是仪器测量**（我自己那两个判据都是坏的，见 `../operations/pitfalls.md` §48）。右腕的单目 UVC 为装它而拆除且**永久不再装回**。`/chassis_{left,right}_camera` 深度话题来自 ECU，不是这颗 |
| D3 | ~~ZED HID 接口无驱动 → IMU 拿不到~~ → **已证伪** | `2b03:f881 ZED-2i HID INTERFACE` @ `1-2.2` 确实 `Driver=[none]`（内核层面是真的），但 **`/zed/zed_node/imu/data` 实测 ~99 Hz**（2026-08-10）| ✅ **IMU 是可用的。** ZED SDK 自己拿 `hidraw`，不需要内核 usbhid 驱动。「`Driver=[none]` ⇒ 功能不可用」这个推理在这里是错的 —— 这正是 `../operations/pitfalls.md` 元教训 5「测你关心的量，不要测它的代理」的一个实例：该测话题频率，不是测驱动绑定 |
| D4 | 系统长期过载 | `loadavg 31.25 30.76 32.31` / 14 核 ≈ **2.2×** | 这是 ZED 降到 15 Hz 的根因之一。任何时序敏感的实验前先 `top` 看一眼谁在吃 CPU |
| D5 | `sudo` 每次报 host 解析失败 | `sudo: unable to resolve host tegra-ubuntu` | 补 `/etc/hosts`：`127.0.1.1 tegra-ubuntu` |
| D6 | 数采配置有未填占位符 | `/home/astrabot/deploy/data_collection.yaml` 里 `<YOUR_ZED_SERIAL>` `<YOUR_ROBOTD_SERVER_IP>` `<YOUR_SKILL_CONFIG_PATH>` `<YOUR_LAKEFS_ENDPOINT_URL>` `<YOUR_NAME>` | **需要人来填**，我没法猜。不填数采跑不起来 |
| D7 | ~~`Astrabot_ZED` 处于 failed 且当前没跑~~ → **已修复** | 2026-08-10 实测 `systemctl is-active` = `active`，`rgb .../compressed` 11.0 Hz、`depth_registered` 8.5 Hz、`imu/data` 99 Hz | ✅ 正常。修法就是 `systemctl reset-failed Astrabot_ZED && systemctl start Astrabot_ZED`，**别用手工 `setsid nohup`**（会落在 domain 0，见 §0）。⚠️ 它 **独占相机**，所以直接 `sl.Camera().open()` 必然失败 —— 那是设计如此，不是相机坏，见 `../operations/pitfalls.md` §5 |
| D8 | 控制器日志有 getActuatorIndex 异常 | `[FATAL] mainThread: id 82/92/109, getActuatorIndex -1 exception` | id 82/92/109 不在 §4 那 14 个里，也不是夹爪的 101/102。**尚未查清是谁在问这些 id**，待排 |
| D9 | `/dev/ttyAMA4` 用途不明 | 6 波特率 × 5 slave id 只读扫描全静默 | 可能是预留口，也可能对面设备没上电。**不要假设它是空的**就拿去接别的东西 |
| D10 | ~~手臂 `/joint_states` 单位不是弧度~~ → **已证伪。真正的缺陷是：控制器收到第一条指令之前，手臂反馈是未锁存的无效值** | `scripts/arm_unit_probe.py` 实测：指令 +0.174533 rad → 反馈增量 +0.174526，**系数 1.0000 = 弧度**；发过第一条指令后 14 路全部落到 ±0.000006 并稳住（1530 帧，极差 3 µrad） | 🟠 **仍然危险，但危险点变了**：`报出值 × 减速比` 恒为 95.8738 整数倍那套算术是**冷启动**现象，不是单位问题。**规则：先发一条指令，再信反馈。** 绝不要在控制器刚起来时用 `/joint_states` 构造「保持当前位姿」—— 那正是把 `left_arm_6` 发到 544°（限位 ±1.57 rad）的方式。安全起点是全 14 路发常量 `0.0`。详见 ADR 0003 所述的已删文档 |
| D11 | ~~**全机没有力 / 力矩 / 接触反馈**~~ → **08-19 部分证伪** | `effort` 在 16 个关节上仍全是 `.nan`，G2 状态也仍只有 `pos_mm` 可信；但操作者确认夹爪内部两片压力贴片已接入且曾读到数据 | 手臂关节力反馈确实不存在，不能靠手臂触碰探测桌高。夹爪接触不再只剩开口阻塞代理：两贴片软件边界和闭环决策已补上，但协议、映射、阈值与真机闭环记录未完成前继续 fail closed |
| D12 | 头部看不到自己的工作区 | `head_pitch` 必须压到 **+40° 限位**，ZED 视野才和手臂可达区有交集；居中所需的角度**超出限位** | 桌面精细判定只能靠**腕部相机**，不能指望头部 ZED。这条反向约束了判定器设计，见 `09_*.md` §1 |
| **D13** | **ZED 自己那棵 TF 会整棵消失，而图像话题照发**（2026-08-11 新增） | 08-10 16:22 起持续一整夜：机体 TF 完好、`/zed/.../image/compressed` 正常在发，但 **6 个 zed frame 一个都没有**，感知报拿不到 `base_link ← zed_camera_link`。哑掉的是 `zed_state_publisher` | 🔴 **判据是「数 zed frame 个数」（正常 6，`<5` 即故障），不是看图像 Hz。** `tf2_echo base_link zed_camera_link` 会**误报正常**。修法：`systemctl restart Astrabot_ZED.service`（~45 s 后复验）。这条故障期间 `xr1.py status` 一直全绿 —— 因为它只查图像话题。现在数 frame 的是 `bin/tf-frames`（`../operations/pitfalls.md` §38）|
| **D14** | **串口设备节点会被重新枚举，而持有它的驱动进程照样活着**（2026-08-11 新增） | 08-11 **00:04** `/dev/ttyAMA5`(204,69) 与 `/dev/ttyUSB0`(188,0) 被重新创建；`g2_gripper_node` (pid 487809) 启动于此之前，`fuser -v` 显示它仍占着这两个节点，但读回全静默。**`SIGTERM` 无效**（阻塞在串口 read 上） | 🔴 **只有 `kill -9`。** 判据是**比时间戳**：`ls -l /dev/tty*` 的创建时间 vs `ps -o lstart= -p <pid>` —— **设备比进程新 ⇒ 废 fd**。⚠️ `xr1.py bringup` 在这个状态下**修不了**：它的存在判据是 `pgrep`，进程在就不重拉（它会如实打印 `STILL SILENT`，但那容易被读成"再等等"）。元教训：**「进程在」≠「设备可用」**（`../operations/pitfalls.md` §39）|
| **D15** | **两个 URDF 里腕相机 `origin` 左右完全相同，而关节轴是镜像的**（2026-08-11 新增） | 左右都是 `xyz="0 -0.0768 0.0995"`（rpy 也一样），而 `axis` 一侧 `0 1 0`、另一侧 `0 -1 0`。左右对称机械上 y 应当反号 ⇒ **至少一侧 y 符号错**，量级 ~2×76.8 = **154 mm**。而且全机搜不到这个数的出处（§28：谁量的没人记录） | 🔴 **不要在它上面建手眼标定。** 腕相机是唯一绕开 `head_yaw` 半米误差链的通道，所以这条很要紧。`scripts/teleop_truth.py` 顺带存了腕相机 TF + 两路腕相机图，可以把它**拟合**出来再用（`../operations/pitfalls.md` §28/§44）|

## 12. 复查命令速查

```bash
# 0) 永远先做这一步，否则下面全是空的
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=12 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 机器人图（2026-08-10 实测 52~62 节点 / ~207 话题；08-07 首测 44/109。数目随 MPC/ZED/夹爪起停波动，
#  且 `ros2 node list` 会提示「有同名节点」——那是厂商栈里本来就有重名，不是故障）
ros2 node list | wc -l && ros2 topic list | wc -l
ros2 control list_controllers
ros2 control list_hardware_components
ros2 topic echo /joint_states --once

# 硬件
ip -brief link show type can            # can1/can2 应为 UP
ls -l /dev/ttyUSB* /dev/ttyAMA*
ls -l /dev/*_cam*                       # 应有 f_chest_cam / l_arm_cam（没有 r_arm_cam，右腕已换 DW2）
lsusb -t                                # 找 Driver=[none]
cat /proc/loadavg

# 谁占着某个设备（sudo 需要 askpass，见下）
export SUDO_ASKPASS=/tmp/.ap1; printf '#!/bin/sh\necho 1\n' > $SUDO_ASKPASS && chmod +x $SUDO_ASKPASS
sudo -A fuser -v /dev/ttyAMA5 /dev/video0

# 某个进程到底在哪个 domain（排查"看不见话题"的第一招）
sudo -A tr '\0' '\n' < /proc/<PID>/environ | grep ROS_DOMAIN_ID
```

> `sudo` 在无 TTY 的场景（脚本 / agent）下必须用 `SUDO_ASKPASS`，
> 而且 `SUDO_ASKPASS` **不跨 shell 保留**，每次调用都要重新 export。

---

## 附：本文档没有覆盖的部分（明确声明，避免被误当成全集）

- **底盘 / 轮子 / IMU / 激光雷达**：`xr1_chassis`、`bluesea_node`、`imu_chassis_node`、
  三个 `ks114_*_driver` 超声都在 domain 12 里跑着，但本次**没有逐一验证**它们的硬件接口
- **ECU 那台机器**：`/astrabot/ecu/*` 命名空间说明还有第二个计算单元，本文只写了 Thor
- **夹爪的力控与标定**：`force_cmd` 只做过 30/50 两档定性测试，**没有做力标定**
- **`Astrabot_Backend` / `Frontend` / `Gateway` / `Log_Hub` / `Task` / `Mpc` / `Mapping`**
  这些单元的内部行为
- **各 `.start_script` 的 `Enable=` 字段**：文件里没有该字段（grep 为空），
  启用与否的判定逻辑在 `execute.sh` 里，**未细读**
