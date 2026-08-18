# 08 · 夹爪：UFactory Gripper G2 / Modbus RTU 驱动

> **全部结论来自本机实测**（只读 Modbus 0x03 扫描 → 人眼确认动作 → 端到端闭环验证 →
> 丢帧率定量测量）。这份文档同时是 2026-08-07 那次「夹爪到底怎么了」的**完整定案记录**，
> 包括**三个被实测推翻的错误假设**。
> 采集时间：2026-08-07 14:00–16:40 CST · 主机 `tegra-ubuntu`
> 硬件底图见 [`07_hardware_map.md`](07_hardware_map.md)

---

## 1. 一句话结论

本机夹爪是 **UFactory Gripper G2**，走 **RS485 / Modbus RTU**，
**slave id 8 @ 2000000 baud**，左手 `/dev/ttyAMA5`、右手 `/dev/ttyUSB0`。
两只都**电气完好**；之前不动的唯一原因是 **`Fn100` 使能寄存器一直是 0，从来没人使能过** ——
因为厂家 SDK 里配的是**另一家的夹爪挂在另一条总线上**。

现在两只都已验证可用，驱动是 `/home/astrabot/gripper_ws` 里的 `g2_gripper_pc`。

## 2. 三个被实测推翻的假设（过程本身是结论）

| # | 曾经的假设 | 怎么被推翻的 |
|---|---|---|
| A1 | 「夹爪在 CAN 上，是供电或接线问题」 | 对 `can1`+`can2` 全部 **127 个 node id** 做完整扫描，只有 14 个手臂驱动器应答。**夹爪从来不在 CAN 上。** 这个假设浪费了最多时间 |
| A2 | 「是 xArm 夹爪，走网口」（tar 文件名 `xarm_gripper.tar` 误导） | 解包后 `package.xml` 写的是 `UFactory Gripper G2 (RS485 / Modbus RTU) ROS 2 driver`，依赖 `python3-serial`，代码里全是 CRC16 + FC 0x03/0x06/0x10。**和 xArm SDK、和网口都没关系。** 文件名不可信，看代码 |
| A3 | 「`ttyAMA5` 读失败是超时太短」 | 把 timeout 从 0.05 扫到 0.30 s，失败率 **0.3% / 1.3% / 1.0% —— 在噪声内持平**，而成功的应答中位数只有 **0.8 ms**（最坏 4.2 ms）。**等更久救不回来**，因为帧是被整体丢掉的（每次失败都返回 **0 字节**，不是短帧也不是 CRC 错）。见 §7 |

**教训：`Fn702` 静态读数不能用来判断左右手。** 我一度把一只夹爪停在 840（全开）当视觉标记，
结果用户回「两只手都是张开的」—— 因为没动的那只静止在 **548/840 = 65% 开**，
肉眼和 100% 开根本分不出来。**要判断是哪只手，必须用动作，不能用开口大小。**

## 3. 硬件与总线

| | 左手 | 右手 |
|---|---|---|
| 设备节点 | **`/dev/ttyAMA5`** | **`/dev/ttyUSB0`** |
| 物理层 | Thor 原生 PL011 UART `810c510000.serial` | CP2102N USB 转串口，USB `1-3.1` |
| 驱动 | 内核 `port` | `cp210x` |
| 稳定标识 | `/sys/class/tty/ttyAMA5` 固定 | `ID_SERIAL_SHORT=d60f7389ced5ef118820724b49d2c684` |
| slave id | 8 | 8 |
| 波特率 | 2000000 | 2000000 |
| 首次扫到的 `Fn702` | 548 | 560 |
| 首次扫到的 `Fn100` | **0（未使能）** | **0（未使能）** |
| Modbus 丢帧率 | **~1–2%**（见 §7） | **0%** |

> Thor 的原生 UART 是 **PL011 → `/dev/ttyAMA*`**，不是老 Jetson 的 `/dev/ttyTHS*`
> （本机 `ttyTHS*` 不存在）。这一点找错过一次。

**两只是不同的物理设备，不是一个口的别名** —— 依据是首次扫描时两个 `Fn702` 读数不同（548 vs 560）。

### 左右手是怎么确认的

- **右手**：只跑 `ttyUSB0` 一路，10 个开合循环（293 ↔ 840），人眼观察 → **右手在动**
- **左手**：只给 `ttyAMA5` 发指令，位置 541 → 839，同时右手保持 839 不变 → **左手是 ttyAMA5**

### `/dev/ttyAMA4` 不是夹爪

6 种波特率（2M / 921600 / 115200 / 57600 / 19200 / 9600）× 5 个 slave id（8/1/2/9/10）
全部静默。用途不明，**不要假设它空着就拿去接别的东西**。

### `/dev/ttyAMA10` 绝对不要碰

头颈 SM45BL 舵机，被 `ros2_control_node` 的 `AstraNeckHW` 独占（`fuser` 已确认）。
打开它会抢掉头部控制。

## 4. Modbus 寄存器映射（源码核对过）

来自 `g2_gripper_pc/g2_gripper_node.py` 的 `G2Gripper` 类常量：

| 常量 | 地址 | 手册名 | 长度 | 用法 |
|---|---|---|---|---|
| `REG_ENABLE` | `0x0100` | `Fn100` | 1 | 写 1 使能。**开机默认是 0** |
| `REG_FNC_BASE` | `0x0C00` | `FnC00..FnC04` | 5 | 位置指令块 |
| `REG_FDBK_POS` | `0x0702` | `Fn702..Fn703` | 2 | 位置反馈，`(reg0<<16) | reg1` |

**位置指令块（FC 0x10 写 5 个寄存器）：**

```
[ 1, speed_cmd, force_cmd, pos_hi, pos_lo ]
  │        │          │        └────┴── 目标开口 mm，32 位拆高低字
  │        │          └── force_cmd，配置默认 50
  │        └── speed_cmd，配置默认 3000
  └── 固定 1
```

**开口换算（teleop 约定：命令 0 = 全开，1 = 全闭）：**

```
open01 = 1 - close01
pos_mm = round(open01 * max_position_mm)        # max_position_mm = 840
```

Modbus 细节：CRC16 多项式 **0xA001**（标准 Modbus），功能码用到 `0x03` 读保持寄存器、
`0x06` 写单个、`0x10` 写多个。帧间隔要求 3.5 字符，在 2 Mbaud 下约 **20 µs**。

**端到端闭环实测**（命令 → Modbus → 动作 → 位置反馈）：

| 命令 `close01` | 理论 `(1-cmd)*840` | 左手实测 | 右手实测 |
|---|---|---|---|
| 0.65 | 294 | 293 | 291 |
| 0.00 | 840 | 838 | 840 |
| 0.35 | 546 | 545 | 544 |
| 0.00 | 840 | 838 | 839 |

误差都在编码器噪声量级 —— **两只手都完全可用**。

## 5. 驱动包 `g2_gripper_pc`

来源：用户提供的 `xarm_gripper.tar`（**文件名误导**，里面是 `gripper_ws/` + 包 `g2_gripper_pc`）。
原包是给 **Ubuntu 22.04 / python3.10** 编的，本机是 **Jazzy / python3.12**，所以必须重编。

```
/home/astrabot/gripper_ws/
├── src/g2_gripper_pc/
│   ├── package.xml                 ament_python; 依赖 rclpy / std_msgs / python3-serial
│   ├── config/g2_gripper_config.yaml   ← 端口在这里改
│   ├── launch/g2_gripper_pc.launch.py
│   ├── 99-usb-grippers-g2.rules     ← 原机器的规则，本机不适用，见下
│   └── g2_gripper_pc/g2_gripper_node.py   ← 注意是双层同名目录
├── build/  install/  log/
```

> **解包时的两个坑**
> 1. tar 里 `socket_teleop_bridge.py` 和 `lan_teleop_relay.py` 在 `src/` 下**只有
>    `__pycache__`，没有 `.py` 源码**。已从 `build/g2_gripper_pc/build/lib/g2_gripper_pc/`
>    里恢复回 `src/`。
> 2. 源码路径是 `src/g2_gripper_pc/g2_gripper_pc/g2_gripper_node.py` —— **双层同名目录**，
>    少一层会找不到文件。
>
> tar 已做过安全检查：无绝对路径、无 `../`。

### 随包的 udev 规则**不适用于本机**，别装

`99-usb-grippers-g2.rules` 里写的是两个 CP2102N 的序列号
`88dfb2f1526eef11858adcc2c169b110` 和 `fe1a852a556eef118f06e3c2c169b110`。
本机唯一的 CP2102N 序列号是 `d60f7389ced5ef118820724b49d2c684` —— **两个都不匹配**。

这反过来证明：**原机器有两个 USB 转串口 dongle（ttyUSB3 + ttyUSB2），本机是「一个
dongle + 一个原生 UART」**。所以 tar 里默认的 `ttyUSB3`/`ttyUSB2` 在本机根本不存在，
必须改配置（已改，见 §6）。

## 6. 配置

`src/g2_gripper_pc/config/g2_gripper_config.yaml` 当前值：

| 参数 | 值 | 说明 |
|---|---|---|
| `rs485_dev_name_left` | `/dev/ttyAMA5` | **本机改过**（原 `ttyUSB3`） |
| `rs485_dev_name_right` | `/dev/ttyUSB0` | **本机改过**（原 `ttyUSB2`） |
| `gripper_left_enable` / `_right_enable` | `true` / `true` | |
| `baudrate` | 2000000 | |
| `slave_id_left` / `_right` | 8 / 8 | |
| `max_position_mm` | 840.0 | |
| `speed_cmd` | 3000 | 首次试动时临时降到 1000 |
| `force_cmd` | 50 | 首次试动时临时降到 30。**没做过力标定** |
| `invert` | false | |
| `publish_frequency` | 20.0 Hz | 位置反馈发布频率 |
| `write_min_period_ms` | 100 | 写指令最小间隔 |
| `modbus_retries` | 2 | **本次新增**，见 §7 |
| `diagnostics_rss_log_interval_s` | 15.0 | 顺带打印总线健康计数，见 §7 |
| `diagnostics_rss_warn_mb` | 512.0 | |

### 话题

| 方向 | 话题 | 类型 | 约定 |
|---|---|---|---|
| 订阅（指令） | `/rm_left/rm_driver/teleop_gripper_float` | `std_msgs/Float64` | **0 = 全开，1 = 全闭** |
| 订阅（指令） | `/rm_right/rm_driver/teleop_gripper_float` | `std_msgs/Float64` | 同上 |
| 发布（状态） | `/qg_robot/gripper_left_state` | `std_msgs/UInt32MultiArray` | `data[0]` = 开口 mm |
| 发布（状态） | `/qg_robot/gripper_right_state` | `std_msgs/UInt32MultiArray` | 同上 |

> ⚠️ 指令话题名和厂家 SDK **完全相同**，所以 `g2_gripper_pc` 和 SDK 的夹爪逻辑
> **绝对不能同时跑**，否则两边抢同一条命令流。这就是 §9 要关掉 SDK 那份的原因。

**不要把 `/astrabot/gripper_{left,right}_state` 当夹爪状态用。** 那两个话题属于
**厂家那份错的 SDK**，内容永远是 `[0,0,0,0]`。真驱动发的是 `/qg_robot/gripper_*_state`。
（这两组话题在 domain 12 里同时存在，很容易抓错。）

**状态数组里只有第一个数是真的**：`[pos_mm, running, temp, error]`，
`running / temp / error` 在 `g2_gripper_node.py` 里**硬编码为 0**，不是传感器读数。

### 唯一可用的「抓到了没有」信号

全机没有任何力/力矩反馈（`effort` 在 16 个关节上全是 `.nan`），夹爪的 `force_cmd`
也是开环丢给固件的。所以判断有没有抓到东西**只有一招**：

> **比较命令开口和实际 `pos_mm`。夹爪卡在比命令值更宽的位置 = 指间有东西。**

已实现在 `../scripts/gripper_cmd.py --ramp`。这个信号在自进化循环里是判定器的第二条
独立通道，重要性见 [`09_self_evolution_bottlenecks.md`](09_self_evolution_bottlenecks.md) §1。

### 三条运行期铁律

1. **每个串口只能有一个驱动进程。** 第二个实例会和第一个抢 RS485 时隙，症状是
   `read failed: ... multiple access on port?`，并且 `ttyUSB0` 的重试率从 0 飙到 2.1%。
   报错文字看起来像「设备掉线」，其实是自己打自己。**排查「夹爪坏了」的第一步是
   `fuser -v /dev/ttyUSB0 /dev/ttyAMA5` 看有几个进程。**
2. **必须带 `ROS_DOMAIN_ID=12` 启动。** 漏了会落在 domain 0：夹爪**照样被使能、
   照样发状态**，只是发在自己的域里。从 domain 12 看过去，话题「存在但没数据」——
   这个表象和硬件坏掉**一模一样**。用
   `tr '\0' '\n' < /proc/<pid>/environ | grep ROS_DOMAIN_ID` 确认。
3. 🔴 **「进程在」不等于「设备可用」**（2026-08-11 新增，`../PITFALLS.md` §39）。
   串口设备节点会被**重新枚举**（08-11 **00:04** `/dev/ttyAMA5` 与 `/dev/ttyUSB0`
   双双被重新创建），而早于它启动的 `g2_gripper_node` 仍然活着、`fuser -v` 里仍然
   占着这两个节点，但**攥的是已经失效的 fd**，读回全静默。而且它**吃掉 SIGTERM**
   （阻塞在串口 read 上）——**只有 `kill -9`**。

   判据是**比时间戳**，不是看进程在不在：

   ```bash
   ls -l /dev/ttyAMA5 /dev/ttyUSB0                       # 设备节点创建时间
   ps -o lstart= -p "$(pgrep -f 'g2_gripper[_]node')"    # 驱动进程启动时间
   # 设备比进程新  ⇒  废 fd  ⇒  kill -9 然后 xr1.py bringup
   ```

   ⚠️ **`xr1.py bringup` 在这个状态下修不了它**：它的存在判据是 `pgrep`，
   进程在就打印 `already running` 不重拉。它随后会如实打印
   `left gripper readback: STILL SILENT` —— 那行字是真的，但很容易被读成
   「再等等就好」。**判断"修好没有"的工具本身不能假阴性**，这是元教训 30。

### 启动

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/astrabot/setup.bash
source /home/astrabot/gripper_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=12                      # ← 漏了这行，机器人收不到，且不报错
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch g2_gripper_pc g2_gripper_pc.launch.py
```

只跑一只（调试用）：

```bash
ros2 run g2_gripper_pc g2_gripper_node --ros-args -r __node:=g2_gripper_pc \
  --params-file /home/astrabot/gripper_ws/install/g2_gripper_pc/share/g2_gripper_pc/config/g2_gripper_config.yaml \
  -p gripper_left_enable:=false
```

重编：

```bash
cd /home/astrabot/gripper_ws && colcon build --packages-select g2_gripper_pc
```

**尚未加入开机自启** —— 要自启需在 `/opt/ros/start_up/auto_start_script/` 加单元。

## 7. `ttyAMA5` 丢帧与重试修复

### 现象

`/dev/ttyAMA5`（左手，PL011 原生 UART）**静默丢掉约 1–2% 的 Modbus 事务**；
同一次运行里 `/dev/ttyUSB0`（CP2102N）**一帧不丢**。

### 定量测量（`/tmp/g2_timeout_soak.py`，每档 300 次采样）

| 端口 | timeout | 失败率 | 失败形态 | 成功延迟 |
|---|---|---|---|---|
| `ttyAMA5` | 0.05 s | 0.3% | **全部 0 字节** | p50 0.8 ms / max 4.2 ms |
| `ttyAMA5` | 0.15 s | 1.3% | 全部 0 字节 | 同量级 |
| `ttyAMA5` | 0.30 s | 1.0% | 全部 0 字节 | 同量级 |
| `ttyUSB0` | 全部档位 | 0% | — | — |

**加大 timeout 完全无效**（0.3 / 1.3 / 1.0% 在噪声内持平），而正常应答只要 0.8 ms。
失败形态是**整帧消失**（0 字节），不是短帧、不是 CRC 错。
结论：**是 PL011 RS485 半双工换向时把整帧丢了，不是超时。等更久救不回来，重问才行。**

### 修复：幂等重试

`G2ModbusClient._txrx()` 加了 `DEFAULT_RETRIES = 2` + `RETRY_GAP_S = 0.002`
（2 ms ≫ 2 Mbaud 下 3.5 字符的 ~20 µs 帧间隔，确保迟到的半截帧先排空，
不会让重试撞上上一帧的尾巴）。

**重试对本驱动用到的每个功能码都是安全的**：`0x03` 是纯读；`0x06`/`0x10` 写的是
**幂等的绝对量**（使能 = 1，或一个绝对目标位置），所以丢了应答后重发同一帧不会累加。

### 效果

| | 修复前 | 修复后 |
|---|---|---|
| 事务总数 | — | 3902 |
| 重试 | — | 79（2.02%） |
| **最终失败** | ~1–2% | **0** |
| 用户可见告警 | 30 s 内 38 条 | **110 s 内 0 条** |

### 配套的总线健康计数器

光加重试会**把总线劣化藏起来** —— 1% 丢帧和 30% 丢帧在用户看来一样安静。
所以 `G2ModbusClient` 加了 `tx_total` / `tx_retried` / `tx_failed` 三个计数器，
搭在原有的 RSS 诊断 tick 上每 15 s 打一行：

```
[bus] left /dev/ttyAMA5 tx=3902 retried=79 (2.02%) failed=0 (0.00%)
```

**运维要点：`retried%` 就是总线健康度。** 平时左手约 2%、右手 0%；
如果 `retried%` 明显爬升，说明接线/屏蔽/地线在退化，去查硬件，**不要去调 timeout**。

> 实现细节：`_start_rss_monitor()` 在 `self.grippers` 赋值**之前**就被调用了，
> 所以计数器循环用的是 `getattr(self, "grippers", {})`。15 s 的首次等待让真正撞上
> 的概率很低，但不是零。

## 8. 安全注意

- **使能（`Fn100 = 1`）在本机这两只上不会触发回零扫动** —— 实测位置保持不变，
  这和 G2 手册的暗示不一样。但**动手前还是先清场**。
- **`Fn100` 在驱动器里是掉电前锁存的**：节点退出后它仍然是 1。
- 首次试动一律先发 `0.0`（全开），这样夹爪只会张开，不可能夹到东西。
- 做识别动作时闭合侧封在 **0.65**（仍留 35% 开口），夹不住任何误入的东西。
- 本次全部**发现阶段**只用只读 `0x03`，绝不用 `0x06`/`0x10`；确认要动之前先问人。

## 9. 已关掉的错误配置（2026-08-07 16:35 执行）

### 错在哪

厂家 `astrabot_actuator_sdk` 里配的夹爪是：

```yaml
gripper_list:
  - "{id: 101, can_port: 1, force: 100, topic: \"/rm_left/rm_driver/teleop_gripper_float\"}"
  - "{id: 102, can_port: 2, force: 100, topic: \"/rm_right/rm_driver/teleop_gripper_float\"}"
can_gripper_enable: true
gripper_hw_ver: 5     # 5 = NiMotion 4 代单指夹爪
```

**厂家和总线都不对**：本机装的是 UFactory G2 走 RS485，这份配置写的是 NiMotion 挂 CAN
node 101/102。这份配置里**根本没有串口的概念**，所以它只会永远打印
`AstrabotNiMotion::setPosition(): Operation not enabled yet!`，
而且它抢的正是 `g2_gripper_pc` 要用的两个话题。

### 两个文件，只有一个是活的 —— 别改错

| 文件 | 状态 | 依据 |
|---|---|---|
| `astrabot_actuator_config.yaml` | **死文件** | 在 `/opt/ros/astrabot/`、`/opt/ros/start_up/`、`/home/astrabot/deploy/` 里精确 grep 这个文件名，**零命中**。唯一提到它的是 SDK 二进制里一句报错字符串 `"actuator_list is empty, loadActuatorConfig from astrabot_actuator_config.yaml failed"` |
| **`astrabot_arm_actuator_config.yaml`** | **活的** | `ros2_control_node` PID 864478 的命令行里明确带 `--params-file .../astrabot_arm_actuator_config.yaml` |

> 这里我自己犯过一个错并纠正了：一开始 `grep -rl` 命中 launch 文件，让我以为
> `astrabot_actuator_config.yaml` 是活的。实际上 launch 里那是个**变量名**
> （`astrabot_actuator_config`），它的**值**指向的是 `astrabot_arm_actuator_config.yaml`。
> `astrabot_hwcontrol/launch/astrabot_xr1_evt2_arm_head_forward_ros2_control.launch.py`
> 和 `astrabot_actuator_sdk/launch/astrabot_actuator_node.launch.py` 都是这个套路。
> **grep 变量名 ≠ grep 文件名。**

### 实际改动

1. `astrabot_arm_actuator_config.yaml`：删掉 `gripper_list` 两条，`can_gripper_enable: true → false`，
   并在原位留了一段说明注释（为什么删、真驱动在哪、备份在哪）。
   - 安全性依据：SDK 二进制里有 `"gripper_list size is zero"` 这个字符串，
     说明**空 / 缺失的 `gripper_list` 是它支持的正常分支**，不会崩。
   - 备份：`astrabot_arm_actuator_config.yaml.bak-20260807-163540`（同目录）
   - **验证过 14 个手臂驱动器一个都没动**：用 `yaml.safe_load` 逐键 diff 新旧文件，
     只有 `can_gripper_enable` 和 `gripper_list` 两项变化，
     `actuator_list` / `actuator_direction_list` / `tolerance_pos_list` 完全相同。
2. 删除死文件 `astrabot_actuator_config.yaml`。
   - 备份：`/home/astrabot/astrabot_actuator_config.yaml.dead-20260807-163540`

### ⚠️ 生效需要重启控制器

错的夹爪逻辑**跑在 `ros2_control_node` 进程内部**（日志证据：
`/opt/ros/start_up/log/Astrabot_Controller/0807/13.52.32/` 里每 5 秒一对
`[astrabot_actuator_sdk]: Left/Right gripper: No messages received for NNNN seconds`）。
配置改在磁盘上，**下次 `Astrabot_Controller` 启动才生效**，当前进程仍带着旧参数在跑。

> 我一度告诉用户「那个错的节点根本没在跑，不用重启」—— **这句是错的**，
> 依据是当时 `ros2 topic info -v` 只列出 `g2_gripper_pc` 一个订阅者。
> 真正原因是我那条命令跑在 **domain 0**，看不见 domain 12 里的订阅者（见
> `07_hardware_map.md` §0）。SDK 的夹爪逻辑一直在跑。

重启风险评估（2026-08-07 16:36 实测）：**14 个手臂关节全部精确在 0.0000 rad**
（零位，垂在身侧，已在行程底部），`head_pitch` 在 +38°。所以重启对手臂本身风险很低，
但**会中断手臂和头部控制**，要在确认无人操作时做。

## 10. 待办

| # | 事项 |
|---|---|
| T1 | 重启 `Astrabot_Controller` 让 §9 的改动生效（需人确认时机） |
| T2 | 把 `g2_gripper_pc` 加进 `/opt/ros/start_up/auto_start_script/` 实现开机自启（含 `ROS_DOMAIN_ID=12`） |
| T3 | 做一次夹爪**力标定** —— `force_cmd` 目前只有 30/50 两档定性经验，没有力—数值对应关系 |
| T4 | 给 `/dev/ttyUSB0` 写一条按 `ID_SERIAL_SHORT=d60f7389ced5ef118820724b49d2c684` 匹配的 udev 规则，防止以后再插 USB 串口时编号漂移 |
| T5 | 查清 `/dev/ttyAMA4` 到底接了什么（`07_hardware_map.md` D9） |
| T6 | `g2_gripper_pc` 的改动（端口 / 重试 / 计数器）目前只在本机，应该同步回上游包 |

## 11. 调试脚本

本次用到的脚本，**已从 `/tmp` 落盘到 `../scripts/`**（`/tmp` 重启会清）：

| 文件 | 用途 |
|---|---|
| `../scripts/g2_probe.py` | 只读 Modbus 扫描：所有串口 × 6 波特率 × 5 slave id，读 `Fn702`/`Fn100`。会报告驱动**实际设定**的波特率，免得把静默的降频钳制误判成「没设备」。**纯 `0x03`，不写任何寄存器，可以放心跑** |
| `../scripts/g2_wiggle.py` | 只动一只夹爪做左右识别。闭合侧封 0.65（仍留 35% 开口），结束停在全开；发布前先等 `get_subscription_count() > 0`，否则第一帧发进虚空、手不动 |
| `../scripts/g2_timeout_soak.py` | timeout × 端口扫描，把失败分类成 zero / short / bad-CRC 并给 p50/p99/max 延迟。**就是这个脚本推翻了超时假说**（§2 A3） |

`g2_wiggle.py` 里的话题名硬编码成 **left**（`/rm_left/...` + `/qg_robot/gripper_left_state`），
要测右手需要改这两个常量。跑它之前 `g2_gripper_node` 必须已经在**同一个 domain**里跑着。

---

## 更正 2026-08-07 19:5x —— 不要清空 `gripper_list`

本文档前面建议把 `astrabot_arm_actuator_config.yaml` 的 `gripper_list` 留空、
`can_gripper_enable: false`，并称那是 SDK 支持的正常分支。**这个说法是错的，
会让 `ros2_control_node` 每次启动都 SIGABRT（exit -6）：**

```
gripper_enable is false, skip gripper node
ActuatorManager::mainThread() -> AstrabotGripper::parseGripperState(CanMessage const&)
terminate called after throwing an instance of 'std::out_of_range'
what():  vector::_M_range_check: __n (which is 0) >= this->size() (which is 0)
```

发送路径确实检查空表，但 **CAN 接收路径不检查** —— `parseGripperState` 无条件
访问 `gripper_[0]`。后果链：无 `/controller_manager` → spawner 永远等
`list_controllers` → 无 `joint_state_broadcaster` → 无 `/joint_states` →
`astra_arm.Robot()` 直接 `MotionRefused`，双臂完全不能动。

**正确做法：保留两条 `gripper_list` 条目和 `can_gripper_enable: true`
（保证向量非空），只把它们的 `topic:` 改成惰性名字：**

```yaml
    gripper_list:
      - "{id: 101, can_port: 1, force: 100, topic: \"/vendor_can_gripper_unused_left\"}"
      - "{id: 102, can_port: 2, force: 100, topic: \"/vendor_can_gripper_unused_right\"}"
    can_gripper_enable: true
```

幽灵 NiMotion 夹爪于是只会无害地打印 `Operation not enabled yet!`，而
`/rm_{left,right}/rm_driver/teleop_gripper_float` 留给真正的 g2_gripper_pc。

另外两点（同日实测）：

* `astrabot_fd_sdk` **不是手臂 SDK，是颈部的 RS485/Modbus SDK**。

  > ⚠️ **本条曾经写错，2026-08-07 由实验推翻。** 原文声称颈部与 G2 夹爪共用
  > `/dev/ttyAMA5` + `/dev/ttyUSB0`，并要求"kill 夹爪 → 重启控制器 → 再起夹爪"
  > 的启动顺序。**两条都是错的。**
  >
  > 实测：让 `g2_gripper_node` **保持占着**两个夹爪口去重启
  > `Astrabot_Controller.service`，`/joint_states` 照样 **202.97 Hz**，控制器全活。
  >
  > 真实接线：颈部 2 个伺服在 **`/dev/ttyAMA10`**（由 `ros2_control_node` 独占；
  > 设备名不在任何配置文件、也不在 `.so` 字符串里，是编译进去的）；夹爪在
  > `/dev/ttyAMA5`（左）和 `/dev/ttyUSB0`（右）。**不同口，无冲突，启动顺序无所谓。**
  >
  > `[FATAL] astrabot_fd_sdk: device-3 report slave failed` 是**无害的**：fd_sdk
  > 探测 device-0..3，而颈部只有 2 个伺服，这条后面紧跟 `goto home success`。它在
  > 健康情况下同样出现，所以**它永远不可能是崩溃的原因**。
  >
  > 教训：拿一条 FATAL 日志去解释崩溃之前，先确认它在正常情况下不出现。本次唯一
  > 真因就是上面那条 `gripper_list` 越界。
* `pkill -f g2_gripper_node` 会把自己的 shell 一起杀掉，用
  `pkill -f 'g2_gripper[_]node'`。

验证结果：`/joint_states` 200.8 Hz，`joint_state_broadcaster` /
`astrabot_arm_forward_position_controller` / `astrabot_neck_forward_controller`
全部 active，双臂 + 双爪同时工作。
