# XR1 踩坑指南

> 本机(AstraBot XR1 / Jetson Thor / ROS 2 Jazzy)上真实踩过的每一个坑。
> 每条按 **症状 → 我当时的错误判断 → 真实原因 → 正确做法** 写,因为下次重犯的方式
> 一定是"症状看起来像另一件事"。
>
> 校验时间:2026-08-07 起,最后一次新增 **2026-08-18**。各条最后一次全量复验 2026-08-10。
>
> ⚠️ **脚本名是历史记录,不是入口。** 这份文件写成时,工作区里有大约 95 个 Python 脚本
> (`xr1_verify.py` `grasp_block.py` `servo.py` `agent_loop.py` `zed_perception.py` …)。
> 那些文件**已经不在磁盘上了**,也不在任何 git 历史里(2026-08-18 的重构开始前这里没有
> 版本控制),无法恢复。每条坑记的**症状、机制、判据和修法仍然有效** —— 只有命令行需要
> 翻译到现在真实存在的两个入口:
>
> | 文中的写法 | 现在 |
> |---|---|
> | `scripts/xr1.py …` | `py/xr1.py …`(同一个文件,同一套命令) |
> | `scripts/xr1_verify.py` | **不存在了**。等价检查:`py/xr1.py pose` ＋ `bin/tf-frames` ＋ `py/xr1_cam.py doctor` |
> | `scripts/vista_observe.py` | `py/vista_observe.py` / `xr1-vision observe` |
> | `grasp_block` / `zed_perception` 的感知与 IK | `xr1-vision plan`(`crates/xr1-vision/`) |
> | `pad_offset.py` 的手眼测量 | `py/pad_offset_measure.py` |
> | `servo.py` / `plan_descent.py` | **不存在了**,见 [ADR 0003](../decisions/0003-lost-python-pipeline.md) |
>
> **今天卡在哪、下一步做什么** → [`status.md`](./status.md)。

---

## 0. 一句话版本(如果只看一行)

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**没有它,ROS 图看起来几乎是空的,而且不报任何错。** 本文档里至少三个坑的第一层
误判都源自这一条。

---

## 1. `ROS_DOMAIN_ID` 不是 12 → 整个机器人"看起来没启动"

| | |
|---|---|
| **症状** | `ros2 topic list` 只有 `/parameter_events` `/rosout`;`ros2 node list` 几乎空;`/joint_states` 不存在 |
| **错误判断** | "驱动没起来 / 控制器崩了 / 需要重启服务" |
| **真实原因** | 新开的 shell 默认 `ROS_DOMAIN_ID=0`,机器人的图在 **12**。跨域完全不可见,**DDS 不会报错**,因为"没有发现任何节点"在 DDS 语义里是合法状态 |
| **正确做法** | 每个 shell 先 `export ROS_DOMAIN_ID=12`。怀疑任何 ROS 问题时,**第一件事**是确认它,而不是重启服务 |

**为什么值得单独列第一条**:这是一个"沉默的错误"。所有正常的排查直觉(看话题、看节点)
在这里都会给出**一致且完全错误**的结论,而且不会有任何线索指向环境变量。

---

## 2. 空 `gripper_list` → `ros2_control_node` 每次启动必 SIGABRT(**我自己造的**)

| | |
|---|---|
| **症状** | `/joint_states` 出现约 5 秒后消失;`ros2 control list_controllers` 超时;所有关节读到 `nan`;`astra_arm.Robot()` 直接 `MotionRefused`;双臂完全不能动 |
| **错误判断** | 先怀疑是串口冲突(见 §3),再怀疑硬件掉线。**都错了** |
| **真实原因** | 我之前把 `astrabot_arm_actuator_config.yaml` 的 `gripper_list` 清空并设 `can_gripper_enable: false`,还在注释里写了"留空是 SDK 支持的正常分支"——**那句话是我编的,是假的**。SDK 的**发送路径**检查空表,但 **CAN 接收路径不检查**:`AstrabotGripper::parseGripperState()` 无条件访问 `gripper_[0]` |

```
gripper_enable is false, skip gripper node          <- 厂商自己的提示,误导性极强
ActuatorManager::mainThread() -> AstrabotGripper::parseGripperState(CanMessage)
terminate called after throwing 'std::out_of_range'
what(): vector::_M_range_check: __n (which is 0) >= this->size() (which is 0)
```

**正确做法**:保留两条 `gripper_list` 条目 + `can_gripper_enable: true`(保证向量非空),
只把 `topic:` 改成惰性名字,让幽灵 CAN 夹爪去发一个没人订阅的话题:

```yaml
gripper_list:
  - "{id: 101, can_port: 1, force: 100, topic: \"/vendor_can_gripper_unused_left\"}"
  - "{id: 102, can_port: 2, force: 100, topic: \"/vendor_can_gripper_unused_right\"}"
can_gripper_enable: true
```

**两个衍生教训**:

1. 厂商打印 `skip gripper node` **不代表**那条分支被测试过。日志里的"我支持"要用崩溃
   栈去验证,不能信。
2. 我在注释里写下一个未经验证的断言("留空是支持的"),它后来变成了**下一次排查的
   错误前提**。**不确定的事不要写成肯定句**,尤其不要写进会被未来的自己当作事实读的地方。

**配置在 install space** (`/opt/ros/astrabot/share/...`),厂商包重装会静默还原。
备份:`.bak-20260807-163540`(原始)、`.bak-emptylist-crash`(崩溃版)。
`xr1_verify.py` section 10 专门检查这个补丁还在不在。

---

## 3. 串口冲突 —— 一个**我自己造出来又推翻**的假因果

| | |
|---|---|
| **症状** | 控制器日志里有 `[FATAL] astrabot_fd_sdk: device-3 report slave failed` |
| **错误判断** | "`astrabot_fd_sdk` 驱动颈部,它的 dev-0/dev-1 和 G2 夹爪共用 `/dev/ttyAMA5` + `/dev/ttyUSB0`,夹爪节点抢了口,所以颈部初始化失败,所以控制器崩了。必须按 kill 夹爪 → 重启控制器 → 再起夹爪 的顺序启动。" |
| **真实原因** | **完全没有冲突。** 颈部 2 个伺服在 **`/dev/ttyAMA10`**(由 `ros2_control_node` 独占;设备名不在任何配置文件、也不在 `.so` 字符串里,是编译进去的)。夹爪在 `/dev/ttyAMA5`(左)+ `/dev/ttyUSB0`(右)。fd_sdk 探测 device-0..3,而颈部只有 2 个伺服,所以 **device-3 那条 FATAL 是无害的**,后面紧跟 `goto home success` |
| **判决实验** | 让 `g2_gripper_node` **保持占着**两个夹爪口,直接重启 `Astrabot_Controller.service` → `/joint_states` **202.97 Hz**,控制器全活,而且 **device-3 FATAL 照样出现**。假设被彻底推翻 |

**最重要的元教训**:

> 在用一条 FATAL 日志解释崩溃之前,**先确认它在正常情况下不出现**。
> 一个在健康状态下也会打印的错误,**永远不可能**是崩溃的原因。

我在这条假设上浪费了很长时间,而验证它只需要一次"带着所谓的冲突去重启"的实验。
**能一次实验推翻的假设,不要先写进文档。** 当时我已经把它写进了 memory 和 `docs/08`,
两处都得回头改——错误结论的传播成本远高于验证成本。

**结论**:颈部和夹爪是独立子系统,**任意顺序启动都可以**。

---

## 4. `pkill -f g2_gripper_node` 会杀掉自己的 shell

| | |
|---|---|
| **症状** | 命令返回 exit code 144,shell 直接没了,后续命令全部失败 |
| **真实原因** | `pkill -f` 匹配**完整命令行**,而我自己那条 bash 的命令行里就包含 `g2_gripper_node` 这个字符串,于是它匹配到了自己 |
| **正确做法** | `pkill -f 'g2_gripper[_]node'` —— 方括号让正则匹配到目标,但**字面量不等于**自己的命令行 |

同类风险适用于任何 `pkill -f <你刚打出来的字符串>`。

---

## 5. 直接 open pyzed **必然**失败 —— 而且那不是相机坏了

| | |
|---|---|
| **症状** | `CAMERA STREAM FAILED TO START`,或 grab 卡死 / `can't claim interface -6` |
| **错误判断** | "ZED 坏了 / USB 掉线 / 需要 reset 相机" |
| **真实原因** | `Astrabot_ZED.service` 跑的是 **`zed_wrapper`**,它**独占**相机。ZED 只有一路 USB,同时只能被一个进程 open |
| **正确做法** | **别 open,去订阅。** wrapper 已经把一切发布到 ROS 图上了 |

```
/zed/zed_node/rgb/color/rect/image[/compressed]   ~7.8 Hz
/zed/zed_node/depth/depth_registered              ~7.2 Hz
/zed/zed_node/point_cloud/cloud_registered        ~5.3 Hz
/zed/zed_node/imu/data, .../camera_info
/chassis_{left,right}_camera/depth/image_raw      ~9.7 Hz  <- 另外两个深度相机
```

**而且 TF 是通的**:`base_link <- zed_camera_link` = (+0.0492, −0.0015, **+1.3430**)。
所以**不需要手标外参**。光学系名字是 **`zed_left_camera_frame_optical`**
(**不是** `..._optical_frame`,查过 TF 树才知道)。

**这个坑的代价最大**:因为 pyzed 只在 deploy venv(py3.10)里有,而 `rclpy`/`tf2` 只在
ROS 解释器(py3.12)里有,我为了绕开"相机被占"搭了一整套
**停服务 → pyzed 出 JSON → 换解释器 → tf2 转 base_link** 的流水线。
**这整套东西本来就不需要存在**——ROS 侧一个进程就能拿到 RGB + 深度 + 点云 + TF。

> 元教训:遇到"资源被占用"时,先问**占用者是不是已经把我要的东西暴露出来了**,
> 再去考虑抢占。抢占式方案会连带引入解释器隔离、进程间交接、外参标定三个新问题。

用 `python3 scripts/xr1.py snap` / `XR1().snap_zed()`。
pyzed 原生 API 只在确实需要时用,那时才 `systemctl stop`(记得起回来)。

---

## 6. 两个 Python 解释器,不要混

| | |
|---|---|
| **症状** | `ImportError: No module named pyzed` 或 `No module named rclpy`,取决于用了哪个 |
| **真实原因** | ROS 侧是 `/usr/bin/python3`(3.12,有 `rclpy` `numpy` `cv2` `yaml` `serial`);`pyzed` **只**存在于 `/home/astrabot/deploy/.venv/bin/python`(3.10,有 `pyzed.sl` `numpy` `cv2` `torch` `yaml`) |
| **正确做法** | **优先留在 ROS 解释器里**(见 §5:ZED 在那边也能拿到)。真要 pyzed 时用绝对路径调 venv,别指望 `source activate` 后还能 import rclpy |

顺带:`transformers` 不在,`torch` 是 CPU-only,OWLv2 权重也没了 → **本机没有可用的
开放词表检测器**,需要检测就把帧发出去。

---

## 7. `python -c` 传多行脚本会被引号毁掉

| | |
|---|---|
| **症状** | `xr1_verify.py` 每次都报 "ZED open failed",但手动跑同样的代码是好的 |
| **真实原因** | 我用 `sh(f"{DEPLOY_PY} -c {json.dumps(code)}")` 传一段含嵌套引号的脚本,经过 shell 一层后引号被吃掉,子进程执行的是残缺代码。**报错信息完全指向错误的方向**(看起来像相机问题,实际是引号问题) |
| **正确做法** | 写临时文件再执行:`open("/tmp/probe.py","w").write(code)` → `sh(f"{PY} /tmp/probe.py")`。或用 `subprocess.run([PY,"-c",code])` **列表形式**(不过 shell)。绝不要把多行代码塞进 f-string 拼出来的 shell 命令 |

---

## 8. TF 监听器会把 `/joint_states` 饿死(单线程 executor)

| | |
|---|---|
| **症状** | `astra_arm` 抛 `MotionRefused: /joint_states is 0.31s stale`,阈值是 0.30s——**只差一点**,而且时好时坏 |
| **错误判断** | "`/joint_states` 掉频了 / 控制器有问题"。实测它是 **200 Hz**,健康的 |
| **真实原因** | 我在同一个节点上加了 `TransformListener`。`/tf` 也是 200 Hz,单线程 executor 下 `spin_once` 每次只处理一个回调,TF 的流量把 `/joint_states` 回调挤到了阈值之外 |
| **正确做法** | **TF 改成惰性创建**——只有真正要查 `tcp_z()` 时才建。热路径(手臂/夹爪)不为它付代价。另外给"临界超时"加一次**诚实的重试**:多 spin 0.4s 后**重新读真实数据并重新做同样的检查**(不是跳过检查) |

> 注意区别:**重试 ≠ 绕过安全检查**。真的 `/joint_states` 死了,重试后依然会抛。

---

## 9. 手臂命令话题上有 2 个发布者 —— 这是**正常**的

| | |
|---|---|
| **症状** | `count_publishers()` 在 `/astrabot_arm_forward_position_controller/commands` 上返回 2 |
| **错误判断** | "有别人在抢控制权,astra_arm 会拒绝移动" |
| **真实原因** | `astrabot_actuator_sdk` 和 `astrabot_mrt` **常驻**持有发布者,但绝大多数时候**一个字节都不发**。MPC 栈(`astrabot_mpc` / `astrabot_mrt` / `astrabot_arbitration`)在跑,但只有 `/reference/cmd` 被喂东西时才会真的开始流 |
| **正确做法** | **测流量,不要数发布者。**订阅 1 秒,`< 0.5 Hz` 就是空闲。`xr1_verify.py` 现在这么做,并把发布者名字打出来 |

**真正的危险点**:如果 MPC 栈开始流,它会和你的直接命令**打架**。`astra_arm` 检测到
流量时会拒绝移动——**那个拒绝是对的,不要绕过它**。

---

## 10. `FollowJointTrajectory` 有 3 个客户端、**0 个服务端** → 用不了

| | |
|---|---|
| **症状** | 想用标准轨迹接口,action 一直不返回 |
| **真实原因** | `/astrabot_eyou_controller/follow_joint_trajectory` 的 `astrabot_eyou_controller` **没有被 spawn**。`astrabot_mrt` 是它的**客户端**(×3),一直在等一个不存在的服务端 |
| **正确做法** | 唯一可用的手臂通道是 `forward_command_controller/ForwardCommandController`,它**没有插值**。所以**客户端插值是强制的**——必须自己从**实测**位姿开始打点、限速。`astra_arm.Robot.move()` 已经这么做了 |

同理:厂商动作库(`astrabot_motion_list.yaml` 里的 `hello` / `byebye` / `cheer` /
`welcome_R` / `welcome_L`,以及 `astrabot_action_play/action_file/action_*.txt`)
**当前没有活的触发入口**(`MotionControl.action` 无服务端),所以那些现成动作调不起来。

---

## 11. `ForwardCommandController` 没有插值 —— 未斜坡的目标 = 全速冲

| | |
|---|---|
| **症状** | 直接往命令话题发一个目标位置,手臂猛冲 |
| **真实原因** | 这个控制器把收到的数就是当前指令,不做任何时间插值。发一个远处的值就是让它以最大能力冲过去 |
| **正确做法** | 永远从**实测**关节位置开始,按时间打点、限速下发。别从"上一次的目标"开始(实际可能没到位)。`astra_arm` 的限速/夹限/保持未命令关节的逻辑不要绕 |

---

## 12. `effort` 全是 `.nan` → **接触力无法感知**

| | |
|---|---|
| **真实原因** | `/joint_states` 的 `effort` 字段在所有 16 个关节上都是 `.nan` |
| **后果** | **拿不到任何力/接触反馈**。不能靠"感觉到阻力"判断抓到了、碰到了、到边界了。所有边界只能是**几何**的 |
| **验证手段** | 想确认动作真的发生了,只能看 **TF**(如 `tcp_z()` 的高度变化)或**夹爪 `pos_mm` 读回**,不能看 effort |

夹爪读回是真实的:`839 → 13 → 840 mm` 这种变化证明了真的动了。

---

## 13. 冷启动时的关节反馈是无效的

| | |
|---|---|
| **症状** | 刚启动时关节值看起来像"编码器 ticks",算出一个 ×95.8738 的比例因子 |
| **错误判断** | "`/joint_states` 的单位不是弧度,要乘一个系数" |
| **真实原因** | 那是**冷启动伪影**。发出第一条命令之前反馈无效。实测单位就是**弧度,系数 1.0000** |
| **正确做法** | 先发一条命令,再信反馈 |

---

## 14. 头看不到自己的工作区

| | |
|---|---|
| **真实原因** | `head_pitch` 限位是 **±0.698 rad(±40°)**。要让 ZED 视野和手臂可达范围**有交集**,pitch 必须压到 **+40° 的极限**;想"居中看桌面"所需的角度**超出限位** |
| **数字** | pitch=0 时桌子从 x>0.826 m 才开始,而手最远只能到 x≈0.49 m ——**完全没有交集** |
| **正确做法** | 感知前 `xr1.py look 40 0`。实际到位 **+39.0°**(URDF 限位是 0.681 rad,不是标称的 40°) |

积木在 base_link x≈0.55 m,超出可达带 x=0.22~0.46 → **底盘挪近是抓取的前置条件**,
不是可选项。

---

## 15. 左右臂符号是**镜像**的,而且左臂挥手会被限位截断

| | |
|---|---|
| **真实原因** | 抬手关节是 `arm_2`,但**左臂 + 抬、右臂 − 抬**。用 TF 的 tcp dz 实测:right_arm_2 符号 −1(dz +0.0609 m),left_arm_2 符号 +1(dz +0.0625 m) |
| **附加坑** | `left_arm_4` 下限只有 **−0.139 rad**,所以对称的 ±0.40 挥手在左边**会被截断**——左右挥手**不是镜像对称的**。这是限位,不是 bug |
| **正确做法** | 别猜符号。`XR1.raise_arm(side, amount)` 已经把它封进去了 |

限位速查:`left_arm_2 [-0.174, +3.050]` / `right_arm_2 [-3.050, +0.174]`;
`left_arm_4 [-0.139, +2.355]` / `right_arm_4 [-2.300, 0.000]`。

---

## 16. 夹爪:0 是开,1 是关,而且**不是** `astra_arm.gripper()`

| | |
|---|---|
| **错误判断** | 用 `astra_arm.gripper()`——它发的是厂商 NiMotion/CAN 话题,**那条链路上没有真硬件** |
| **真实硬件** | UFactory **G2**,Modbus RTU **slave 8 @ 2000000 baud**,左 `/dev/ttyAMA5`、右 `/dev/ttyUSB0`,由 `g2_gripper_pc` 驱动 |
| **命令** | `/rm_{left,right}/rm_driver/teleop_gripper_float`,`Float64`,**0.0 = 全开,1.0 = 全闭** |
| **读回** | `/qg_robot/gripper_{left,right}_state`,`UInt32MultiArray` = `[pos_mm, running, temp, error]`,**840 mm = 开**,~5 mm = 闭,20 Hz |

**顺带优化**:原来我用固定 1.2 s sleep 等夹爪到位;改成**轮询 `pos_mm` 到位就返回**后,
每次 **1.2 s → 0.58 s**。固定 sleep 是"测出来的动作时间"里的大头,它测的其实是我自己
的 sleep。

---

## 17. UVC 相机必须以 MJPG 打开,否则 capture 卡死;图像节点是 `ATTR{index}=="0"` 那个

| | |
|---|---|
| **真实原因** | UVC 相机同时支持 `MJPG` 和 `YUYV`,但用 YUYV 拿全分辨率时带宽不够,`read()` 直接挂住 |
| **一个 UVC 设备出两个节点** | 一个图像、一个 metadata。分辨它们的**唯一**判据是 `ATTR{index}=="0"`(图像) |
| ~~奇数 video 节点是 metadata~~ | **这句是错的,2026-08-11 推翻**:胸前相机现在是 **video3=图像 / video8=metadata**,恰好和"奇数是 metadata"相反。编号奇偶不含任何信息 |
| **正确做法** | 文档和代码里**一律用符号链接,不写 `video<N>`** —— 每次插拔都变(当天胸前 video2→video3、腕部 video6→video2)。打开后立刻 `set(CAP_PROP_FOURCC, MJPG)`,并**丢掉前 ~8 帧**等自动曝光收敛 |
| **现有几路** | `/dev/f_chest_cam`(胸前)、`/dev/l_arm_cam`(左腕单目)。**没有 `/dev/r_arm_cam`** —— 右腕单目已拆,换成 DaBai DW2(libusb 厂商类,**不出 `/dev/video*`**,只能走 ROS 话题)。左右为什么曾经是反的见 §48 |

---

## 18. DDS 发现不是瞬时的 —— `get_node_names()` 会给出假的 ABSENT

| | |
|---|---|
| **症状** | `xr1_verify.py` 报 "nodes visible 1",并把 `controller_manager` / `robot_state_publisher` / `g2_gripper_pc` 全标成 ABSENT,而它们**都在跑** |
| **真实原因** | `Node()` 刚建好就调 `get_node_names()`,DDS 发现还没收敛 |
| **正确做法** | 先 spin 到节点数**稳定**(或超时)再相信图。实测收敛需 **0.7~0.9 s**。这类"偶发假阴性"很危险,因为它会让你去修一个根本没坏的东西 |

同一个原因也会让频率统计失真(曾经量到 `/joint_states` 只有 4.5 Hz,实际 200 Hz)。

---

## 19. 一个动作拆成 N 个 shell 调用 = 慢 3 个数量级

| | |
|---|---|
| **症状** | "这么简单一个动作为什么这么慢" |
| **真实原因** | 三个时钟被混为一谈:①机器人控制环 **200 Hz(5 ms)**;②**每进程** ROS 启动开销 **~3.3 s**(import rclpy/tf2、建 DDS participant、发现、等 `/joint_states`、取 URDF);③**每个工具调用一次 LLM 前向**。把 20 阶段动作拆成 20 次 `ros2 topic pub`,就是 20×②+20×③ 去做 ~15 s 的动作 |
| **实测数字** | `source` 两个 setup.bash 0.391 s;python+rclpy/tf2 import 0.35~1.38 s;DDS participant+发现 1.52~1.97 s;一次裸 `ros2 param set` 往返 1.469 s。单进程跑完整 20 阶段 demo:**总 77.7 s,其中固定开销只有 3.35 s**;拆开则光启动就 ~90 s,外加 20 次 LLM 往返 |
| **正确做法** | **把整个任务(包括标定探测)放进一个进程。** 这就是 `xr1.py` 存在的理由 |

---

## 20. 主机层面的两个长期噪音

| | |
|---|---|
| **RTC 是死的** | 系统时间从 **1970-01-01** 开始。文件时间戳、日志时间都不可靠;**只能用 `/proc/uptime`** 判断有没有重启过 |
| **hostname 不解析** | `tegra-ubuntu` 无法解析,于是**每次 `sudo` 都打印一条无害的 "unable to resolve host"**。它不是错误,不要去追 |

另外 `sudo` 在无 TTY 环境下需要 `SUDO_ASKPASS`。
`Astrabot_File_Transfer` / `Astrabot_Log_Agent` 常处于 `activating/auto-restart`,
`Astrabot_Log_Hub` 常 `inactive/dead` —— 与手臂/夹爪/相机功能无关。

---

## 21. G2 夹爪驱动**不是 systemd 服务** → 每次重启后夹爪静默消失

| | |
|---|---|
| **症状** | 突然 `/qg_robot/gripper_*_state` 完全静默、`/joint_states` 只有 25 Hz、`controller_manager` 报 ABSENT。上一次跑还全好 |
| **错误判断** | "我把什么东西改坏了" —— 于是开始回滚自己的改动,越查越乱 |
| **真实原因** | **机器发生过重启**。`/proc/uptime` = 6 min、load 9.04:所有 Astrabot systemd 服务自己回来了,但 `g2_gripper_pc` 是我手工 `ros2 launch` 起来的,**没有 unit 文件**,重启后就没了。而 §20 的死 RTC 让重启完全不可见——日志时间戳都是 1970 |
| **正确做法** | 任何"整套东西突然退化"先看 `cat /proc/uptime`。<3 min 时频率和 DDS 发现还在收敛,别急着下结论。夹爪拉起来:**`python3 scripts/xr1.py bringup`**(幂等:已在跑就跳过,并回读两侧开口 mm 验证) |
| **仍未做的** | 给 `g2_gripper_pc` 写一个 systemd unit。没做是因为它和颈部共享 RS485,启动次序还需要按 §3 的结论再实测一遍 |

---

## 22. 头部 ±40° 限位是**我自己加的**,厂商 URDF 里是 ±3.1 rad 占位值

| | |
|---|---|
| **症状** | 无。**这正是危险之处**:厂商包一次重装,限位静默回到 ±3.1 rad,头部会接受 **4.4 倍**真实行程,没有任何报错 |
| **真实原因** | 出厂 URDF 对 `head_pitch_joint` / `head_yaw_joint` 写的是 `lower="-3.1" upper="3.1"`(effort、velocity 也都写 3.1,明显是占位符)。真实行程 ±0.698132 rad(40°),来自 `astrabot_fd_sdk/AstrabotFdSm45bl.hpp` 的 `GEAR_ANGLE_MAX`。`astra_arm` 从**活的 `/robot_description`** 读限位,所以这个 URDF 是**唯一的**软限位来源 |
| **正确做法** | `xr1_verify.py` §10 的 `GUARDED_CONFIGS` 已经守住它。每次会话先跑 verifier;报 drift 就照 `fix` 字段修 |
| **顺带的坑** | 改这个 URDF 时**注释里不能出现"冒号+空格"**:`launch_ros` 对 `robot_description` 跑 `yaml.safe_load`,一个 `: ` 会让 YAML 把整个 URDF 当成 mapping 解析 → 启动期 `ScannerError`。报错指向 YAML,原因在 XML 注释里 |

---

## 23. 我改过的**不是 1 个**装机目录文件,而是 6 个

| | |
|---|---|
| **症状** | "我只改过夹爪那个 yaml" —— 错。重装任何一个厂商包都会静默回退我依赖的行为 |
| **真实清单** | ① `astrabot/share/astrabot_actuator_sdk/config/astrabot_arm_actuator_config.yaml`(空 `gripper_list` → SIGABRT,§2)<br>② `astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_description.urdf`(头部限位 + 底盘相机 link 改名 + 腕部相机 origin,§22)<br>③ `astrabot/share/astrabot_xr1_evt2_description/urdf/**astrabot_xr1_evt2_arm_description.urdf**`(同样的相机改名和腕部相机 origin)<br>④ `astrabot/share/zed_wrapper/config/zed2i.yaml`(`grab_frame_rate: 15`;HD1080@30 ≈ 249 MB/s 会把相机从 USB 总线上打掉 → `CAMERA REBOOTING`,是带宽/功耗余量问题**不是**相机坏)<br>⑤ `astrabot/share/zed_wrapper/config/common_stereo.yaml`(`self_calib: false` 保住我们依赖的外参;`publish_left_right: true`)<br>⑥ **`/opt/ros/start_up/run/Astrabot_Controller.sh`** —— 把启动的 ros2_control launch 从 arm-only 换成 **arm_head**,这是头颈**有没有命令接口**的开关 |
| **⑥ 为什么最危险** | 厂商默认是 arm-only,不加载 `astrabot_neck_forward_controller`,于是 `head_pitch/head_yaw` **根本没有命令接口** —— 而且不报错,只是话题不存在。所有跟头有关的能力(包括"把 ZED 转到工作区",要 head_pitch 打到 +40° 极限)全挂在这一行上 |
| **⑥ 为什么之前漏了** | 准确地说:它**被记录了但没被 guard**(`../architecture/hardware-map.md` 早就写了这次改动)。漏的是 guard —— 因为它**不在 `/opt/ros/astrabot/` 下**,而在 `/opt/ros/start_up/` 下,我按"astrabot 包树下改过的文件"去建 guard 清单,它天然不在范围内。2026-08-10 改成在**整个 `/opt/ros` 里找 `.bak` 文件**才捞出来。**"文档里写过"和"回退时会被发现"是两件事** |
| **③ 为什么最容易漏** | 它和 ② 名字只差一个 `_arm`,而**手臂那条 launch 加载的是 ③ 不是 ②**(`astrabot_controller_bringup/launch/astrabot_xr1_evt2_arm.launch.py`)。只补 ② 的话,整机描述是对的、手臂栈跑的却是厂商几何——两边不一致而且没人报错 |
| **③ 的一个诚实说明** | 那两个 `{left,right}_arm_camera_joint` 的 origin(厂商 `±0.007 -0.0615 0.0161` → 现在 `0 -0.0768 0.0995`)**没有任何来源记录**,谁量的、怎么量的都查不到(§28)。guard 的作用是"别再静默变化",**不是**断言这个数是对的 |
| **diff 时的坑** | `.bak` 是 CRLF、我改过的文件是 LF,`diff` 会报"2084 行全变了"。必须 `diff --strip-trailing-cr`,真实差异只有 7 行 |
| **正确做法** | 全部登记在 `xr1_verify.py` 的 `GUARDED_CONFIGS` 里,每条带 `must_contain` / `must_not_contain` / `why` / `fix`。**改装机目录 = 必须同时加一条 guard**,否则下次重装就是一次静默回归 |
| **写 guard 的坑 (一):误报** | `lower="-3.1"` 在文件里合法出现 **24 次**(手臂关节和 `zed_camera_joint` 的占位符),而 `left_chassis_camera_link` 作为**网格文件名**(`meshes/left_chassis_camera_link.STL` 是真实文件)正确地保留着——被改名的只是 **link**。所以 guard 必须匹配 `link="..."` 这种唯一形式,不能匹配裸字符串 |
| **写 guard 的坑 (二):漏报** | ⑥ 的第一版 guard **完全无效**:改动只是移动了一个 `#`,两个 launch 名在改前改后**都在文件里**,所以"包含 arm_head 那行"在厂商版上也成立。必须去 guard **注释符**本身(`must_not_contain: "#'ros2 launch ...arm_head..."`,`must_contain: "\n    'ros2 launch ...arm_head..."`) |
| **怎么知道 guard 真的有效** | 拿它去跑 `.bak`。guard 必须在 live 上 PASS **且**在厂商备份上 FAIL —— 只测 live 通过,等于什么都没测。这一步当场揪出了 ⑥ 的漏报。6 条 guard 现在全部通过这个双向测试 |

---

## 24. 检查脚本自己会说谎:0 failure ≠ 机器人可用

| | |
|---|---|
| **症状** | verifier 打出 `0 failure(s), 10 warning(s)` 然后紧接一行 `ALL REQUIRED CHECKS PASSED -- arms, neck, grippers and cameras are commandable.`——**而当时两侧夹爪状态话题完全静默、`/joint_states` 只有 25 Hz**(正是 §21 那次重启)|
| **真实原因** | 结论行的条件写成了 `if not FAILURES`。夹爪静默、频率过低都被归类为 warning,于是横幅在机器人明显残废时依然打绿字。**结论必须由实测量推出,不能由"没有硬失败"推出** |
| **正确做法** | 现在逐子系统给结论:arms / neck / grippers / ZED / UVC cams 各自 READY / NOT READY 并附上支撑它的实测数(`/joint_states 206 Hz`、`state 20.5/20.5 Hz`、`rgb 11.0 Hz`),总评是 `BROKEN` / `PARTIAL(列出哪个没好)` / `ALL PASSED` 三态 |
| **同一类的另两条** | ①verifier 每次都报 "ZED open failed" —— 那是它自己去 `sl.Camera().open()`,而 wrapper 独占相机(§5),**按设计必然失败**的检查产出的告警是纯噪音,已改成测 ROS 话题频率。②`RMW_IMPLEMENTATION=None` 每次告警 —— 而 `rmw_fastrtps_cpp` 本来就是 Jazzy 默认(`ros2 doctor --report` 可证),不设置是对的,只有**设成别的值**才致命 |

---

## 25. ffmpeg 6.1.1 的 `drawtext` 会把中文字幕**截断在字符中间**

| | |
|---|---|
| **症状** | 标题卡上 `XR1 实验记录 20260810-121516_离线自检` 只画出 `XR1 实验记录 20260810-121`;`离线自检` 画成 `离` + 一个豆腐块。纯英文行**完全正常**。而且合成流水线的 13 项自检**全绿** —— 帧数、分辨率、时长、报告内容全部通过 |
| **真实原因** | drawtext 把文本截断到「前 N 个**字节**」,其中 N 是**字符数**。ASCII 恰好 `bytes == chars` 所以无害;任何中文都会被砍掉尾巴,而且切点落在多字节字符中间,于是尾部多出一个豆腐块 |
| **怎么确认的** | `-loglevel debug` 会打 `Line: 0 -- glyphs count: N`。20 个汉字(60 字节)→ `glyphs count: 8` = 前 20 字节里的 6 个完整汉字 + 2 个残字节。构造 11 组不同长度/中英混排的字符串,**每一组的 glyph 数都精确等于「前 chars 个字节能解出多少码点」**,无一例外 |
| **在 drawtext 里无解** | `text_shaping=0`、`text=` 换 `textfile=`、`fontfile=` 换 `font=` 按 fontconfig 名选、换 `.ttc`/`.otf` —— 四种组合渲染结果**完全一致**。截断发生在取文本那一层,不在 harfbuzz/fribidi 整形层 |
| **正确做法** | 字幕改走 **libass**(`ass=文件.ass`),它自己解 UTF-8,完整。顺带白拿两样东西:自动换行(过长的 run_id 不再溢出画面)、`BorderStyle: 3` 的底色块(等价于 `box=1:boxcolor=black@0.55`)。实现见 `scripts/xr1_experiment.py` 的 `_ass()`,注释里写明了「别改回 drawtext」 |
| **元教训** | **自检全绿没有发现它。** 断言查的是「有没有帧、尺寸对不对、报告里有没有这个词」,而缺陷在**画面内容**里。输出是给人看的图像时,必须真的把帧抽出来看:`ffmpeg -ss T -i movie.mp4 -frames:v 1 f.png` |

---

## 26. macOS 相机授权(TCC)有 4 个互相独立的坎,每个都会让远程录制静默失败

| | |
|---|---|
| **症状** | `auth: "denied"`,**没有弹窗**,系统设置 > 隐私与安全性 > 摄像头 里**连这个 app 都不列出**(即没有任何 TCC 记录) |
| **坎 ①** | **hardened runtime 会直接拒绝相机**。`codesign --options runtime` 而二进制没有 `com.apple.security.device.camera` entitlement(ad-hoc 签名也无法背书)→ `authorizationStatus` 直接是 `denied`,不弹窗、不留记录。去掉 `--options runtime` 即恢复 |
| **坎 ②** | **授权绑在 .app 的 cdhash 上** → **重新编译就等于把授权扔了**,得再找人去点一次 Allow。所以 `install`(编译)和 `relaunch`(只重启)必须是两条命令:`xr1_cam.py relaunch` 走 `XR1REC_NOBUILD=1`,不碰二进制 |
| **坎 ③** | **同意弹窗只存在于 Mac 的 GUI(Aqua)会话里**。sshd 拉起来的进程永远等不到弹窗,所以最后一步必须是 `open -a`(把启动交给 Aqua 的 launchd),不能直接 exec。**远程无法代点**:改 TCC.db 要 FDA、用 System Events 点按钮要 Accessibility,两者本身都需要先在 GUI 上点一次 —— 死循环,没有绕过的办法 |
| **坎 ④** | `Info.plist` 里**必须**有 `NSCameraUsageDescription`,否则进程一碰相机就被系统杀掉(不是报错,是 kill) |
| **别设等待超时** | 第一版等 900 s 就写 `state: fatal` 退出。结果没人在 Mac 旁边 → daemon 死了 → 弹窗**跟着宿主一起消失**,后来走过去的人什么都看不到,还得先重启才能授权。现在无限等待,并且**同时轮询 `authorizationStatus`** ——弹窗被顺手关掉、之后在系统设置里打开的这条路,`requestAccess` 的回调永远不会触发,只有状态会变 |
| **怎么看进展** | daemon 在请求授权**之前**就把 `state: "awaiting_permission"` + `waited_s` 写进 `state.json`,所以「在等人点」和「进程死了」在远端是可区分的两种状态 |

---

## 27. UVC 相机的「30 fps」其实是 **30.00003**,拿 30 去比会全线失配

| | |
|---|---|
| **症状 ①** | 明明有 `1920x1080@30` 的格式,按 `minFrameRate <= 30 && 30 <= maxFrameRate` 去筛却一个都选不出来,`best_at_30` 报 `none` |
| **症状 ②** | `device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 30)` 抛 `NSInvalidArgumentException`(值超出该格式支持的区间)。**这是 ObjC 异常,Swift 的 `do/catch` 抓不到** —— 进程直接没了,只在 stderr 留一行 |
| **真实原因** | UVC 描述符里帧间隔的单位是 **100 ns**,30 fps 存成 `333333`,反算回来是 `1e7/333333 = 30.000030000030001`。所以设备报的 `maxFrameRate` 略**大于** 30,而 `minFrameRate` 也略大于 30 |
| **正确做法** | 比较一律带容差(`eps = 0.2`);设置帧时长用 `CMTimeClampToRange(_, range: min...max)` 夹进设备真实区间,而不是直接塞 `CMTime(1, 30)`。`lockForConfiguration` 要用真的 `do/catch`,别 `try?` 之后照样往下写属性 |

---

## 28. 尚未查清的(诚实记录,不要假装知道)

- `astrabot_fd_sdk` 从哪里读到 `/dev/ttyAMA10` 这个设备名:**不在任何配置文件里,
  也不在 `.so` 的字符串表里**。推断是编译进去的,但没有证据。
- `/astrabot_fd_sdk` 的参数服务**超时**,拿不到它的参数。
- 厂商动作库(§10)没有活的触发入口,所以那 6 个现成动作**当前调不起来**。
- `Astrabot_Backend.service` 处于 `activating/auto-restart`,影响未知。
- **腕部相机 origin `xyz="0 -0.0768 0.0995"` 是谁量出来的**:两个 URDF 里
  `{left,right}_arm_camera_joint`(父 `{side}_gripper_base_link`,
  `rpy="1.5708 -1.5708 0"`)都被从厂商的 `±0.007 -0.0615 0.0161` 改成了这个值,
  改动时间 2026-08-07 11:03。**`docs/`、任何 `.md`、`scripts/`、`~/config`、
  `~/tools` 里都搜不到一句相关记录**。`~/config/` 下只有 4 个文件
  (`data_collection_xr1_evt2.yaml`、`run_teleop.sh`、`stop_teleop.sh`、
  `zed_dora_selfcalib.sh`),没有任何与腕部相机标定有关的脚本。
  两侧从不对称变成完全对称这一点像是刻意
  为之(对称支架),但这只是推断。<br>后果:任何依赖腕部相机外参的东西(手眼标定、
  用腕部相机做抓取)都建立在一个**未验证的数**上。要用之前先自己量一次;
  在那之前 `xr1_verify.py` 只保证它不会被厂商重装静默改掉。

---

## 29. 录制器是**独占**的:任何后台脚本都能抢走它,并且会挤进你正开着的 run

| | |
|---|---|
| **症状** | 授权刚到手,`rec new` 之后第一个 `--record` 动作直接抛<br>`RecorderError: 开录失败: start '...__01_look' ignored: already recording '...__01_dryrun-video'`。那个 `dryrun-video` 我**没有手动跑过** |
| **原因** | 我早先埋了一个后台观察者:「授权一变成 authorized 就跑 `grasp_block.py --dry-run --require-video`」。它触发了,而 `grasp_block` 走 `exp_log.Experiment` → `xr1_experiment.Step`,`Step` 调的是 `current_run()` —— **复用当前打开的 run**。于是那次 dry-run 变成了我这个 run 的第 1 步(`01_dryrun-video`),同时把 Mac 侧的录制通道占满 120 s |
| **两件事叠在一起** | ① Mac 上的 daemon **一次只录一路**,第二个 `start` 不是排队而是**被拒**(这是对的:静默排队会让"先开录再动"的保证失效);② `current_run()` 的复用语义是**故意**的(同一轮实验的多个脚本要落进同一个 run),但它也意味着**任何**带录像的脚本都会加入你的 run,而不是另开一个 |
| **不是 bug 的部分** | 报错信息把占用者的 clip 名整个打出来了,所以 3 秒就能定位到是谁。`ps -eo pid,etimes,cmd \| grep grasp_block` 直接看到那个进程和它已经跑了多久 |
| **怎么办** | 动手之前先 `xr1_cam.py status \| grep -E '"(state\|clip)"'`:`state=recording` 就说明有人在录,`clip` 名的前半段是它挂在哪个 run 上。要独占就等它结束(`until ! pgrep -f 'grasp_block[.]py'; do sleep 3; done`),别去 `xr1_cam.py stop` —— 那会把别人那段素材截断,而它的 `require_video=True` 会让整次试验作废 |
| **元教训** | **自己埋的后台任务是环境的一部分。** 我把这次"意外的第 1 步"当成了故障去查,其实它是我自己两小时前安排的。埋观察者时就该想到:它触发的时刻,我大概正在用同一批硬件 |

## 30. 报告里的"证据列"填错字段,等于把一个会自相矛盾的数做成了不可编辑的证据

| | |
|---|---|
| **症状** | 真机第一份 `REPORT.md` 和烧进 `movie.mp4` 的字幕,「先开录」都写着 `0.32s / 0.35s / 0.36s`;而同一批 `steps/*.json` 里 `rec_confirm_ms` 是 `1309 / 1343 / 1335` ms。**两个数差了 4 倍** |
| **原因** | 两个字段都存在、含义不同,我在生成报告和字幕时取了**错的那个**:`lead_s` = 机器人开始动 − **Mac 写的**开录时间戳,里面含两机时钟差(当时 `doctor` 报 −1s);`rec_confirm_ms` = 开录确认返回 → 动作开始,**纯本机时钟** |
| **为什么危险** | 字幕是**烧进画面**的,事后改不了。钟差再大一点 `lead_s` 会直接变成**负数**,那时影片上会写着"先开录 −0.6s",看起来正好是它要否证的那件事 |
| **修法** | 报告列拆成两列(本机钟 / 含钟差)并写明该采信哪一列;字幕只烧本机钟那个(`_lead_short()`)。两个都留着,是因为它们的**差**就是钟差,能自己交叉验算 |
| **元教训** | **同一件事有两个近似字段时,"取哪个"是设计决定,不是实现细节。** 我在 README 和 SKILL.md 里已经写清了该采信 `rec_confirm_ms`,代码里却仍在用 `lead_s` —— **文档写对了不等于代码用对了**,这和 §23 ⑥「被记录了但没被 guard」是同一种病 |

## 31. 判「这一步动没动」不能用 `before`/`after` 之差,而采样又不能 `spin`

| | |
|---|---|
| **症状** | 用户要求「影片只留动的部分」。第一份成片 143.9s 里 120.8s 是一动不动的 dry-run —— 84% 是废画面。我的第一个方案是拿每步的 `before`/`after` 快照求差,差不多就当没动 |
| **为什么这个方案是错的** | 挥手、抓取、`look` 回原位这些动作**结束时手臂回到起始位姿**。实测一次 12.5s 的挥手 `max|Δq|` = **1.9e-6 rad**,一次抬手再放回(settle 之后)是 **9.4e-6 rad** —— 都会被判成"全程静止",于是**恰好是有动作的那些步骤会被整段剪掉**。快照之差测的是"位姿变了没有",而问题问的是"动过没有",这是两件不同的事 |
| **只能在动作过程中采** | 所以 `Step` 在 `__enter__` 起一个 10 Hz 的采样线程,边采边归约成 `motion_spans`(相对步开始的秒区间),不存整条轨迹(会把 step json 撑大几十倍) |
| **第二个坑:采样线程不能 spin** | `XR1.pose()` 默认 `fresh=True` → `astra_arm.joints(fresh=True)` → `_spin(0.1)` → `rclpy.spin_once`。在后台线程里这么读,就是和**正在做动作的主线程抢同一个 node 的 spin**:轻则采到的点乱序,重则主线程的 `/joint_states` 新鲜度检查扑空、动作中途被 `MotionRefused` 打断。为了一个"剪掉静止"的功能去冒中断动作的风险,完全不值得 |
| **修法** | 给 `pose()` 加 `fresh` 参数,采样只走 `pose(fresh=False)`(纯读缓存,不 spin);`Step.__enter__` 先探一次这个能力,**探不到就干脆不采**(影片退化成整段收录),绝不退化成 `fresh=True` |
| **第三个坑:夹爪那一路靠不住** | `grip_state()` 读的也是缓存,而缓存只在**有人 spin 这个 node**时才刷新 —— 采样线程按设计不 spin,主线程只在做手臂动作时 spin。真机两次实测:一次 `grip close` 命令返回时反馈还停在 850mm(真正合到 9mm 是更晚的事,采样窗口里一次没变);一次 3.05s 的 `grip close` 只采到 **0.10s** 在动,按它裁只剩 1.5s,一半可见动作被剪掉。所以 `MOTION_BLIND_ACTIONS`(目前 = `grip`)**一律整段收录**,连"采到了一点"都不信 |
| **空列表和 None 不是一回事** | `motion_spans = []` 意思是"采到了、全程静止"(→ 剪掉),`None` 意思是"没采"(→ 整段收录)。采样线程起不来或中途异常时如果留下 `[]`,就会把有动作的一步剪掉 —— 所以落盘时的判据是"采样线程真的起来了**且**没报错",不是"robot 不是 None" |
| **元教训** | 见下面第 19 条 |

## 32. 一次堵转能把 `ros2_control_node` 打成 SIGSEGV,整台机器的 `/joint_states` 一起没

| | |
|---|---|
| **症状** | 一个 `wave right` 之后,再开一个进程就是 `MotionRefused: no /joint_states within 1.5 s`,顺带 `[warn] using built-in joint limits (robot_state_publisher not answering)`。`systemctl is-active Astrabot_Controller.service` 却是 **active** |
| **真正的状态** | 服务活着、launch 进程活着(`ros2 launch astrabot_hwcontrol ...arm_head_forward_ros2_control.launch.py` 还在),但它的子进程 **`ros2_control_node` 没了**。`pgrep -af 'ros2_contro[l]'` 只剩 launch,`ros2 node list` 里一个 controller 都没有 |
| **原因(日志里写得很清楚)** | `/home/astrabot/start_up_log/Astrabot_Controller.log` 末尾:几百行 `[FATAL] astrabot_actuator_sdk: updateErrorFlag: ID_24, error 1, 0x7121, stalling` / `error_flag: 0x00000400`,紧跟着 `[ERROR] process has died [pid ..., exit code -11]`。**exit code −11 = SIGSEGV**。`0x7121` 是堵转,`ID_24` = **right_arm_4**(右臂 21–27 在 can2,所以 24 是第 4 关节),正是 `wave` 摆动的那个关节 |
| **为什么会堵转** | `wave()` 在 `right_arm_4` 上摆到 **−1.884 rad**。URDF 限位是 `[-2.300, 0.0]`,所以软限位放它过去,但那个位姿真机**够不到**(负载/碰撞),电机堵在 −1.443 rad。`effort` 全是 `.nan`(§12),所以**堵转在 ROS 侧完全无法感知** —— 唯一的早期信号是 `astra_arm` 那句 `[warn] N joint(s) did not reach target within 0.05 rad`,而它长得像一句无害的警告 |
| **注意堵转本身不必然崩** | 同一份日志里 11:43:55 启动时 `ID_23` 也刷过一轮 `0x7121` 却没崩。崩的是错误标志洪泛路径上的某个段错误,所以**"上次堵转没事"不能作为这次安全的依据** |
| **恢复** | ① `kill` 掉 `g2_gripper_pc` 的两个进程(颈部与夹爪共用 RS485,启动时别让它占着口);② `sudo -A systemctl restart Astrabot_Controller.service`;③ `until timeout 3 ros2 topic echo /joint_states --once; do sleep 3; done` 等它回来(先到 80–110 Hz,再爬到 200);④ `python3 xr1.py bringup` 把夹爪驱动拉回来;⑤ `python3 xr1.py status` |
| **怎么避免** | 把 `did not reach target` 当**硬故障**处理:立刻停手,不要接着往同方向发更大的目标(我就是紧接着 `wave` 把 −1.443 又往 −1.884 推,然后就崩了)。写自检/验证脚本时别用 `wave()` 做"随便动一下"的动作 —— 它的幅度是按理论限位来的。要小幅度就 `r.move({j: base ± 0.25})` |
| **元教训** | 见下面第 20 条 |

---

## 33. 头一转 40°,同一块**没动过**的积木就被算到 59cm 之外

| | |
|---|---|
| **症状** | 定住头连采 8 帧,积木算在 base 系 `(0.3066, -0.3960)`。命令 `head_yaw = -40°`,**积木一动没动**,再采 8 帧,算出来变成 `(0.2900, +0.1960)` —— `dx = -16.6mm`,`dy = +591.9mm`,合计 **592.0mm**。y 甚至换了号 |
| **证据** | `experiments/20260810-22`(`cross_yaw_consistency`,591.9mm)和 `20260810-23`(`assoc_check`,584.2mm)两次**独立复现**,彼此只差 8mm。`20260810-23` 还记到:转到 `yaw=+40°` 后**连续 8 帧完全认不出**那块积木(`same=false`) |
| **先排除噪声** | 同一次里定点采样的 `u_std = 0.0 px`(整整 8 帧像素坐标一模一样),折算到桌面 `x_std = 0.019~0.060mm`。**随机误差比这个错小四个数量级**,所以这是系统性的,滤波/多帧平均/加采样数一点用都没有 |
| **量级对不上任何一个简单解释** | 把误差当成"绕颈部竖轴多转/少转了一个角度"来算(`head_base_link` 在 base 系 `x=-0.0265, y=0, z=1.2867`,pitch 0~40° 取范围):<br>· **yaw 符号取反/被加了两次**(等效转 80°)→ 预测位移 **665~908mm**<br>· **yaw 完全没进 TF**(等效转 40°)→ 预测位移 **354~479mm**<br>实测 592mm **夹在两者之间**,哪个都不严丝合缝 |
| **但反解转轴能分出高下** | 反过来问"绕哪根竖轴转多少度能把 A 点搬到 B 点":按 **80°** 解,两次实验各自解出的轴心是 `(-0.054,-0.110)` 和 `(-0.046,-0.107)` —— 两次一致到 8mm,且离真正的颈部轴 `(-0.0265, 0)` 只有几厘米;按 **40°** 解,轴心落在 `(-0.515,-0.123)` 或 `(+1.111,-0.077)`,**机器上那儿什么都没有**。所以"角度被放大成两倍"(符号取反或重复施加)比"yaw 根本没进 TF"更像 |
| **诚实的结论** | 方向清楚:**误差随 head_yaw 走,而且量级接近"转了两倍"**。机制没定死 —— 残下的 100 多毫米说明不是"纯粹一个符号写反",很可能同时还有一项:比如相机相对颈部的平移没跟着转,或者链路里用的 pitch 不是实际 pitch。**在查清之前别写"已修"** |
| **判别实验(还没跑)** | 跑一次 `head_yaw = -20°` 的同样测量。加倍/取反预测 **354~479mm**,没进 TF 预测 **180~237mm** —— 两者**差两倍且不重叠**,一次试验就能定下来。`perc_repeat.py` 现在**没有 argparse**,yaw 是按积木方位角自己算再夹到 ±40 的(第 84~85 行),所以得先给它加一个"指定 yaw"的入口,否则跑出来还是 40° |
| **日志里已经有一行 15° 的数据,但还不能拿来定案** | `experiments/20260810-28`(tag `转头不变量-射线原点AB`,2026-08-10 17:08)转 `yaw=-15°`(实际到 -14°),同一块积木从 `(0.5004, 0.0539)` 变到 `(0.5037, 0.1138)` —— 只差 **59.9mm**。而按上面的模型,15° 时"加倍"预测约 **274mm**、"没进 TF"预测约 **138mm**,**两个都对不上,实测比两个都小得多**。但那次试验**只写了 2 行事件就停了**(没有 meta.json、没有裁定),而且脚本名是 `-`(heredoc),从记录里看不出它用的重建方式和 `-22/-23` 是不是同一套 —— 光凭这一行既不能说"已经修好了",也不能说"假设都错了"。**要么把那次跑完,要么在同一份代码上重跑一次 40° 做对照** |
| **在那之前怎么办** | **把 head_yaw 钉死在 0**,只靠 pitch(必须 +40° 限位,见 `docs/10 §5.1`)。目标不在视野里就**挪底盘**,不要转头 —— 转头一次就引入半米误差,比要修的抓取误差(§`docs/10 §3.1` 的 22.5/48.5mm)大**一个数量级**。这也是 `docs/05 §E1.3` 说"头部姿态必须钉死"的真正原因:它不是为了整齐,是因为一变就全错 |
| **元教训** | 见下面第 21 条 |

---

## 34. 定点迭代修指尖误差,越迭代越不收敛 —— 因为被迭代的映射每轮换了一个解支

| | |
|---|---|
| **背景** | `tcp_link` 是**单根手指**,两指中点要在 tcp 系里偏 (−22.5, 0, +48.5)mm(§`docs/10 §3.1`)。IK 只会把 **tcp** 送到给定点,所以要让**指尖中点**落在积木上,只能:解一次 → 用 FK 量出指尖中点的偏差 → 把 IK 的目标点朝反方向挪一挪 → 再解。这就是 `ik_center()` |
| **症状** | 加了阻尼(步长 0.5 起、不降就减半)之后,期望 0° 那一格 **6 轮后仍有 35.9mm**,而且残差在轮与轮之间**上下跳**,不是单调下降 |
| **错的诊断** | "步长不对/迭代轮数不够"。于是继续调阻尼系数和轮数 —— 全无效果。**残差上下跳这个现象本身就否掉了"步长"假设**:步长只会让收敛快或慢,不会让它非单调 |
| **真因** | `ik_near(target, q_ref)` 是**随机多起点全局搜索**,只按"离 `q_ref` 最近"从一堆解里挑一个。我每轮都把 `q_ref` 固定成备位姿态,于是第 2 轮完全可能挑到**另一个手腕解支**上去。那么 `目标点 -> 指尖中点` 这个映射是**不连续**的,而定点迭代和阻尼**只对连续映射成立**。非单调的残差就是换支的指纹 |
| **修法(第一层,不够)** | 每轮把 `q_ref` 换成**上一轮的解**(锁支),外加"一次没解出来就用 `tries=10, starts=120` 再搜一次"(`ik_near_hard`)。前者让映射连续,后者防止随机搜索偶然返回 `None` 把迭代预算白白耗光 |
| **锁支只是"偏好",还得会拒绝** | `ik_near` 的"挑离 `q_ref` 最近的"是从**这次随机抽到的解集里**挑 —— 近支这轮没被抽到,它就老老实实交一个远支的。所以锁支必须配一个**判罚**:关节跳幅超过 25° 就判定换了支,这一轮**留作候选但迭代不跟过去**,并且阻尼要从"本支上目前最好的那个目标点"重新出发,不能从全局 best 出发(全局 best 可能在另一支上,从它出发等于每次都跳回去) |
| **这样仍然只到 8~14mm** | 12 个朝向里 90° 停在 8.55mm、120° 停在 8.95mm、15° 停在 29.4mm、165° 停在 42.0mm。40mm 的积木上 8.5mm 已经是 21% 块宽 —— 而这一整轮工作的起因就是修那个 22.5mm 的偏差,拿 8.5mm 交差等于把 P1 白做 |
| **真正的修法:换目标函数** | 别再"挪目标点去喂一个随机全局求解器"。`tip_center(q)` 对 `q` 是**光滑**的,所以直接对**关节**做局部最小化:`ik_polish()` 以全局解为起点、L-BFGS-B、`bounds` 锁在起点 ±0.35 rad 内,代价函数就是 \|tip_center(q) − goal\|²(米²,1mm→1e-6)。收敛后三条硬判据(接近角 ≤45°、开合偏差 ≤20°、不碰撞)**原样复检**,一条不过就返回 `None` 沿用未精修解 —— 精修必须是净改进,不许把可行解变成不可行解 |
| **判罚要做成 hinge** | 容差带**内**免费、只在越界后才加惩罚。带内也罚等于拿真实的毫米去换物理上无所谓的度数。开合偏差用 `sin` 不用 `cos`(mod-180 对称) |
| **数字** | 53.5mm → 20.6mm(锁支) → 8.5~42mm(加拒绝跟随) → **12/12 全部 0.00mm**(精修),关节只挪 0.56°~1.16°。代价:实际开合方向偏 0.0~3.0°(想要 15° 时给 12.0°),在 20° 容差带内,40mm 积木上无关紧要 |
| **顺带一个真 bug** | `ik_near` 从第 2 轮起返回的 `d` 是"离**上一轮解**多远",而调用方拿 `d` 判断"这一步动多大、要不要拒动"。锁支之后这个语义漂移才显形 —— 必须按原始 `q_ref` 重算一遍再交出去 |
| **元教训** | 见下面第 22 条 |

---

## 35. `timeout 560 python3 xxx.py` **不是**硬边界:rclpy 脚本会把 SIGTERM 一起无视掉

| | |
|---|---|
| **症状** | `timeout 560 python3 yaw_invariance.py` 在 **2 小时 25 分**后仍在跑,占 94.7% CPU。同期另一个 `teleop_record.py` 已经对着**一动不动**的手臂 200Hz 记了 87 分钟,`record.jsonl` 涨到 233MB(相邻样本的 `q` 完全逐位相同 —— 它记录的是"什么都没发生") |
| **机制** | `timeout` 到点只发 **SIGTERM**。这些脚本是多线程 rclpy(executor 在自己的线程里 spin),Python 的信号处理只在主线程的字节码间隙执行,而主线程卡在 C 层的 spin 里 —— 于是 **SIGINT 和 SIGTERM 都被无视**,只有 SIGKILL 收得掉(实测:INT 无效 → TERM 无效 → KILL 立刻退) |
| **为什么危险** | 我一直把 `timeout N` 当成"跑飞了也有兜底"的安全绳。**它不是。** 一个会发臂部指令的脚本如果卡在循环里,`timeout` 拦不住它,而这台机器上 `effort` 是 `.nan`、控制器无插值 —— 没有第二道防线 |
| **怎么办** | ① 兜底要写在**脚本内部**(循环里查自己的挂钟,超时就 `break` 并回到安全位),别指望外部信号;② 外部收尾用 `kill -9`,别停在 TERM 就以为收掉了;③ 收工前 `pgrep -af 'python3 .*workspace/py/'` 点一遍 —— 自己埋的进程是环境的一部分(§18),而且这次它们合计吃掉约 170% CPU,正好压在我要跑 CPU 推理的时候 |
| **元教训** | 见下面第 23 条 |

---

## 37. `no /joint_states within 1.5 s` **不等于**控制器死了 —— 那是 DDS 发现慢

| | |
|---|---|
| **症状** | `astra_arm.MotionRefused: no /joint_states within 1.5 s -- is Astrabot_Controller.service up?`,连着两次。而 20 分钟前同一个脚本 `Robot() ready (4.59s)` 好得很 |
| **错的诊断** | 报错自己给的那个:"控制器挂了,去重启 `Astrabot_Controller.service`"。**重启是个不可逆且外溢的动作** —— 当时另一个会话正在跑 `teleop_record.py --side right`,重启会作废别人的试验 |
| **怎么排除的** | `ros2 topic hz /joint_states` → **137~169 Hz**,发布者 2 个。话题是活的。再写一个裸 rclpy 订阅计时:**第一条消息 2.56s 才到**,而 `astra_arm` 构造时只 `_spin(1.5)` |
| **真因** | DDS 发现延迟。这台机器的**常态** load average 就是 42~45(14 核):`zed_node` 82%、nav2 79%、`astrabot_mpc` 92%、`astrabot_mrt` 69%,加起来约 4~5 核,再叠上别人的脚本。1.5s 在这个负载下不够握手 |
| **修法** | `astra_arm.py` 的初始等待改成 `DISCOVERY_WAIT_S = 8.0` 的分段重试,报错文案改成"先查 `ROS_DOMAIN_ID`,再查话题有没有流量"。**注意这削弱不了任何安全性质**:每次移动前的 `STATE_STALE_S = 0.3` 陈旧检查一个字没改 —— 等的是**发现**,不是**新鲜度**,两件事不能混 |
| **别踩的连带坑** | memory 里那条"`/joint_states` 假死 → 只能重启控制器"讲的是**完全没有流量**的情形。区分它和本条的唯一办法就是 `ros2 topic hz`,而不是看 `MotionRefused` 的文案 |

---

## 38. ZED 图像 12 Hz、机体 TF 完好、体检报 READY —— 感知却一行都跑不了

| | |
|---|---|
| **症状** | `grasp_block.py` 第一步就抛 `RuntimeError: 拿不到 base_link <- zed_left_camera_frame_optical 的 TF`。而同一时刻:ZED 压缩图 12 Hz、`xr1.py status` **ZED 判 READY、0 failures**、`ros2 run tf2_ros tf2_echo base_link zed_camera_link` 秒出 `(0.118, −0.001, 1.255)` |
| **错的诊断** | 往 §20(机体 rsp 的 participant 没进图)上靠,准备再补一个 `robot_state_publisher`。两条症状文案几乎一样,但**修法互斥**:补 rsp 对本条完全无效 |
| **怎么排除的** | 用 `tf2_ros.Buffer` 等 7s 再 `all_frames_as_yaml()`:**37 帧、单根 `odom`、机体链齐全** → 机体树没断,§20 排除。再数带 `zed` 的 frame:**只有 1 个**,就是 URDF 给的那个 `zed_camera_link` |
| **真因** | ZED launch 自己拉的 `zed_state_publisher`(`-r __ns:=/`)哑了,`zed_camera_center / zed_left_camera_frame / *_optical / zed_right_camera_frame / *_optical` **五个 frame 一个都不发**。它和图像发布是**两条独立的路**,所以图像照常、TF 缺一半 |
| **为什么 `tf2_echo` 会骗人** | `zed_camera_link` 是**URDF 里的**,机体 rsp 在发,所以永远查得到。而射线投射需要的是**光学系**,那个才是缺的。拿 `zed_camera_link` 判「TF 没问题」= 拿错的那一半当证据 |
| **判据(唯一可靠)** | 数 frame:**总数 ≥10 且单根** → 机体树好;**带 `zed` 的 ≥5**(正常 6 个)→ ZED 系好。两个条件分别对应两种修法,不能混 |
| **修法** | `sudo -A systemctl restart Astrabot_ZED.service`,45s 后重查 → 42 帧 / 6 个 zed frame,`base_link ← zed_left_camera_frame_optical` = `(0.122, 0.059, 1.273)` |
| **代价** | 故障从**前一天 16:22** 那次重启就在了。那之后所有依赖 ZED 光学系的感知都不可能跑通,而体检一直全绿 —— 中间那段时间恰好在跑别的脚本,所以没暴露 |
| **已修** | `xr1_verify.py` 新增 TF 段:dump frame 数、根数、zed frame 数,zed frame <5 直接 **FAIL** 并打出重启命令;根数 ≠1 则指向 §20 的补 rsp。ZED 子系统的 READY 判据从「rgb hz>3」改成「rgb hz>3 **且** zed TF frame ≥5」 |

---

## 39. 夹爪读数静默,可是 `g2_gripper_node` **活着** —— 设备节点半夜被重新枚举,旧 fd 已废

| | |
|---|---|
| **症状** | 夹爪状态话题不出数,`grip_state()` 返回 `None`。而 `pgrep -f 'g2_gripper[_]node'` 有进程(487809)、`fuser -v /dev/ttyUSB0 /dev/ttyAMA5` 显示**就是它**在持有两个端口。`xr1.py bringup` 打印 `g2_gripper_node already running` + `readback STILL SILENT`,然后**什么也不做** |
| **错的诊断** | "端口被占着、进程也在,所以是夹爪硬件/波特率/Modbus 的问题",于是往 `docs/08` 那条串口线上查。也想过"重启 `Astrabot_Controller`" —— 和夹爪完全无关,纯浪费 |
| **真因** | `ls -l` 那两个设备节点:**`8月 11 00:04`** 重新创建过(`/dev/ttyAMA5` 204,69;`/dev/ttyUSB0` 188,0)。设备被重新枚举,旧节点的 inode 换了,而 `g2_gripper_node` 从**前一天**就在跑,手里那两个 fd 指向的是已经消失的设备。写不报错,读永远超时 |
| **判据(一条命令)** | `ls -l /dev/ttyAMA5 /dev/ttyUSB0` 的时间戳 vs `ps -o lstart= -p $(pgrep -f 'g2_gripper[_]node')`。**设备比进程新 → 就是这条**,不用再查协议层 |
| **`kill` 无效** | SIGTERM 打不掉它(卡在阻塞的串口 read 里),`kill 487809` 之后进程照旧。**只有 `kill -9`**。这和 §35 是同一件事的另一面:信号只是建议 |
| **修法** | `kill -9 <pid>` → `python3 xr1.py bringup`(这次会打 `MISSING -> launching`)→ 左 816 mm / 右 829 mm |
| **为什么 `bringup` 自己修不了** | 它的启动判据是 `pgrep`(§21 的场景是"进程没了"),所以对"进程在、fd 废了"这种状态它只会**如实报 `STILL SILENT` 然后放过**。读数静默而进程存在时,`bringup` 跑第二遍也没用 —— 先 `kill -9` |
| **元教训** | 见下面第 29 条 |

---

## 40. 三种「没有 `/joint_states`」文案一样,修法互斥 —— 而重启之后的**第一次探针必然骗你**

| | |
|---|---|
| **症状** | 脚本报没有 `/joint_states`。这个症状今早出现一次,过去两天各出现过一次,**三次是三个不同的故障** |
| **A:真的没有流量** | 发布者数 ≥1,但 `ros2 topic hz` 20 s 一条不出,`/dynamic_joint_states` 一起停 → `joint_state_broadcaster` / `astrabot_mrt` 哑了。**只能重启 `Astrabot_Controller.service`**。今早重启后 **199.885 Hz** |
| **B:有流量,只是发现慢** | `ros2 topic hz` 是 137~200 Hz,而脚本等 1.5 s 就放弃 → DDS 发现延迟(§37),负载 25~45 时首条消息要 2.5 s+。**不要重启任何东西** |
| **C:域不对** | 话题压根不存在(§1)。先查 `ROS_DOMAIN_ID` |
| **判据顺序** | ① `echo $ROS_DOMAIN_ID` ② `ros2 topic hz /joint_states` 等满 20 s ③ 裸 rclpy 订阅计时看首条延迟。**只有②能分开 A 和 B**,而 `MotionRefused` 的文案分不开(§37 已把文案改成"接下来查什么") |
| **今早真正差点踩的坑** | 重启控制器 → 等 30 s → 自己的探针报 **`STILL SILENT`**。差一步就得出"重启没用"并去重启第二遍。而同一时刻 `ros2 topic hz` 是 **199.885 Hz** —— **重启完成之后,新参与者的 DDS 发现还没完成**,此时任何短探针都是 B 类假阴性 |
| **正确做法** | 重启后的验收必须用**等得够久**的测量(`ros2 topic hz` 满 20 s,或分段重试到 8 s),不能用一次性短探针。**判"修好了没有"的工具,不能是本身会假阴性的那个** |
| **元教训** | 见下面第 30 条 |

---

## 41. ~~白桌面:深度自洽但有偏、符号会翻~~ → **是基准错了,ZED 只差 2.0 mm**

> **2026-08-11 傍晚结论翻转。** 本节原来的标题和"未定案"结论都是错的:深度没有 +63 mm 偏,
> 是**卷尺 0.750 已经过期**。真桌面 **0.8108**(摇操真值反推),ZED 的 0.8128 差 **2.0 mm**。
> L1 覆盖率不是问题、L3 外参是主项这两条仍然成立;**只有"绝对高度不可用"这条被推翻**。

| | |
|---|---|
| **背景** | 桌布已经撤了,现在是**白色桌面**。此前 已删的旧感知文档(见 ADR 0003) 把逐像素深度不可信归因为"粉色重复格子桌布" |
| **两次预测,两次被实测推翻** | ① "重复纹理是真因 → 换掉桌布" —— 桌布本来就没了,建议无效。② 改口"白色无纹理 → 被动立体会**没有**深度" —— 实测 **89.7~90.93% 有效像素**,深度不但有,还很平 |
| **实测(08-11,`depth_now.py` / `plane_now.py`)** | 有效 **90.93%**(NaN 9.07%),960×540,`frame_id=zed_left_camera_frame_optical`;平面残差 **std 10.66 mm**(p95 20.45 mm);法向倾角 **0.26°**;拟合桌高 **0.8128 m** vs 卷尺 0.750 → **+62.8 mm** |
| **对比(08-07,粉桌布)** | 有效 70.7%;拟合桌高 0.702 → **−48 mm** |
| **真结论** | 两次相差 **11 cm 且符号相反** ⇒ 这**不是**一个可标定的常量偏置。分三层看:**L1 覆盖率**不是问题(89.7~90.9%);**L2 精度**是真问题,但残差 p95 20 mm 仍远小于积木 40 mm 高,所以"平面 + 颜色 + 射线投射"的架构**两种情况下都站得住**;**L3 外参/TF** 才是主项 —— §33 那个 **590 mm** 在这里 |
| **所以"加一个双目相机"解决不了** | 加传感器降的是**方差**,不是**偏差**;而本机误差预算是偏差主导,量级差约 4 个数量级。ZED 2i 本身就是双目(基线 119.86 mm),再加一路被动立体是**同一个失效机制**,还多一条没标定的外参链 |
| **✅ 已定案(2026-08-11 傍晚,摇操真值)** | **是卷尺过期,不是深度有偏。** 人把右臂摇到一个**能夹住**黄积木的位姿,两指中点 FK = (+0.4749, −0.3591, **+0.8238**) ⇒ 桌面 **0.8108**。于是:ZED 的 0.8128 **只差 2.0 mm**(在真值带内),而卷尺 0.750 低出 **44~64 mm**。上面那个"+62.8 mm 偏差"整个是**拿过期基准去比**造出来的 |
| **那"符号会翻"怎么解释** | 08-07 的 −48 mm 是**粉桌布**那次,覆盖率只有 70.7%、倾角 1.3°,拟合本身就差;而且那次卷尺和底盘位置都不同。**反号不是一个需要解释的物理现象,是两次都在跟一把错尺子比。** ZED 在白桌面上的绝对高度实际是可用的 |
| **代价** | 因为这条挂了"未定案",桌高一直用 0.7415,于是 **38 次抓取全部瞄在桌面以下 56.5 mm**(§45)。这条坑的排查成本不在它本身,在它下游 |
| **判别方式变了** | 原先想的是"再拿卷尺量一次桌高"。**现在的判断:抓取不需要绝对真值,需要的是「相机观测 → 关节指令」这个映射**,而摇操示范一次就能给出带标签的样本对(见 §43 与 `scripts/teleop_truth.py`),外参/桌高/`base_link` 在不在地面全被吸收进映射里。卷尺仍然有用,但它不再是关键路径 |
| **元教训** | 见下面第 31 条 |

---

## 42. `os._exit()` 不 flush stdout → 脚本"退出码 0、一个字都没输出"

| | |
|---|---|
| **症状** | `depth_now.py > /tmp/out.txt` 退出码 **0**,文件**空的**。直接在终端跑却有输出 |
| **错的诊断** | "脚本挂在采集上、什么都没算出来" |
| **真因** | 内部看门狗用 `os._exit(0)` 硬退(必须这样,见 §35:rclpy 脚本无视 SIGTERM/SIGINT)。而 `os._exit` **绕过 atexit 和 stdio 缓冲区** —— 重定向时 stdout 是**块缓冲**,几 KB 输出还在缓冲区里就被整个丢掉。终端下是行缓冲,所以看不出来 |
| **修法** | `sys.stdout.flush()` 之后再 `os._exit(...)`。凡是用 `os._exit` 的地方,**每一处**都要配一次 flush |
| **配套关系** | §35(外部 timeout 不是边界,兜底必须写在进程内)迫使我们用 `os._exit`;`os._exit` 又带来本条。两条必须一起记,只记一条就会在另一头翻车 |

---

## 43. 摇操把手臂丢在**哪个位形**,决定下一个自主脚本能不能规划出路径

| | |
|---|---|
| **症状** | `grasp_block.py` 第一次带录像的真尝试落 `skipped`:感知干净(黄块 x=+0.476 y=−0.155)、**pre 和 grasp 两个 IK 都有解**,但 `plan_to` 判**直达和经中转位姿都碰撞**,机器人正确地一动没动。录像有,是 **6.9 分钟的静止画面** |
| **错的诊断** | 往"IK 不可达 / 目标点在可达带外"上想,准备去改目标点或放宽倾角 |
| **真因** | 上一轮摇操结束时右手停在 **tcp z = 0.547 m**,比桌面(0.75)低 **20 cm**。关节空间直线插值从这个位形去桌面上方,中途一定扫过桌面 ⇒ 每个路点都判碰撞。**IK 有解 ≠ 有路可走**,而"可达性"是相对**当前位形**的性质,不是位置的性质 |
| **修法** | 任何自主脚本开工前先归到固定起点:`python3 xr1.py home`(两臂零位、爪张开)。今早归零后同一目标就有路了 |
| **这解释了一条老账** | `docs/10` §9 #7「可达性不可复现」—— 机制就是这个:两次探测之间手臂停的地方不一样,于是同一个点一次可达一次不可达。**先归零再探测**,结论才可比 |
| **已修的工具洞** | `plan_failed` 现在落盘 `q_pre`/`q_grasp` 和归因诊断(第几个路点、撞桌面还是贴躯干、深多少毫米)。之前那 6.5 分钟算出来的 IK 结果**算完就扔**,失败记录里只有一句"规划失败",事后无法复盘 |

---

## 44. 左右腕相机在 URDF 里 origin **完全相同**,而同一个文件里关节轴是镜像的

| | |
|---|---|
| **事实** | 两个 URDF(`astrabot_xr1_evt2_description.urdf` 与 `..._arm_description.urdf`)里,左右腕相机的 `origin` 是**同一串** `xyz="0 -0.0768 0.0995"`、`rpy` 也一样;而同一个文件里对应关节的 `axis` 是镜像的(`0 1 0` vs `0 -1 0`) |
| **推论** | 作者镜像了轴、**没镜像平移**。左右手是镜像结构,y 方向的偏置不可能同号 ⇒ **至少有一侧的 y 号是错的**。§28 早就记着这个 origin"来源不明"(机器上没有任何东西记录是谁量的),现在多了一条内部不一致的证据 |
| **别做什么** | 不要在这个外参上建腕部相机的手眼标定,也不要用它去解释腕相机的偏差 —— 会把一个可能是符号错误的量当成基准 |
| **怎么定案(不用猜)** | `scripts/teleop_truth.py`:摇操夹住积木时两指中点**就是**积木真实位置,配上同一时刻的腕相机像素,这个外参可以被直接拟合出来(顺带给出 y 的符号) |
| **为什么现在才捞出来** | 一直没人用过腕相机。它们跟着手臂动,**天然绕开 `head_yaw` 那条 590 mm 误差链**(§33),而且离积木近 —— 同样的角度误差换算成位置误差小一个数量级。**从没被用过是漏项,不是结论** |

---

## 45. 38 次抓取全部瞄在**桌面以下 56.5 mm** —— 一个过期的人工测量,下游全废

| | |
|---|---|
| **症状** | 38 次成形试验,`grasped` **0**。而且**每次失败原因都不一样**:IK 无解 / 路径判碰撞 / 空爪 / 守卫拒动。感知看着是准的(黄块 x 报 +0.476),IK 也有解 |
| **真因** | `TABLE_TOP = 0.7415`,真桌面 **0.8108**。代码命令指尖去 `GRASP_TIP_Z = 0.7545`,也就是**桌板内部 56.5 mm**。"IK 有解但规划全拒"不是规划器保守 —— **目标点在实体里,判碰撞是对的** |
| **0.7415 是怎么来的** | `TCP_Z_CONTACT(0.790) − TCP_TO_TIP`,而 0.790 来自 08-10 的触桌探针:它看「指令角 vs 实测角」何时分离,把 **1.03e-2 的偏差出现**读成**接触发生**。实际上指尖到不了桌下 70 mm,那个偏差是手臂**顶在桌面上**的伺服误差。顶住之后偏差只会继续涨 ⇒ **单调探针必须配一个独立量才能定标,否则它只会告诉你"伺服跟不上了",不告诉你"碰到了"** |
| **上游还有一层** | 0.790 之前用的是卷尺 0.750(08-07)。那次量的时候**桌布还在、底盘位置也不同**,现在已过期 44~64 mm。**一次性的人工测量会过期,而且不会告诉你它过期了** |
| **怎么定案的(不用卷尺)** | 让人摇操摆一个**能夹住积木**的位姿。积木坐在桌上、被夹在两指之间 ⇒ 两指中点 FK **就是**积木真实位置,`tip_z − 13 mm` 就是桌面。链里没有相机、没有外参、没有人的瞄准 ⇒ **自证**。实测 (+0.4749, −0.3591, **+0.8238**),抖动 0.00 mrad,`verify_fk.py` 用 rsp/KDL 复算差 0.00 mm |
| **判据的选择很关键** | 旧记录器用"停稳 2 秒"当判据,那把**人的目测瞄准误差**焊进了真值。"夹住了"没有这个问题 —— 夹住是物理事实,不是人的判断 |
| **残余** | 我采的第一条样本夹爪读数 **839/840 = 全开**,所以它是"人认为能夹住"而非已验证的夹住,桌高还带 ±10 mm(不知道在 4 cm 积木上夹的高度)。**真夹合上的样本才能同时消掉这两项。** 而且**一个样本只锚一个标量** —— 要解出相机→base 的旋转必须在 x 和 y 两个方向都铺散开 |

---

## 46. `tcp_z()` 丢旋转,所以它**不能**当碰撞地板 —— 按真值重算反而会拦死正确位姿

| | |
|---|---|
| **事实** | `tcp_z()` 走 TF 只取平移。而 `TIP_CENTER = (−0.0225, 0, +0.0485)` 有 **−22.5 mm 横向分量**,夹爪一歪它就**转进 z**。两者的差不是常量 |
| **实测** | 那个已知能夹住的位姿:`tcp_z = 0.8417`,指尖中点 **0.8238** —— 差 **17.8 mm,不是 48.5 mm** |
| **危险在哪** | 拿到新桌高之后,顺手把 `TCP_Z_FLOOR` 按真值重算成 `TABLE_TOP + TCP_TO_TIP = 0.8593`。这个"修好了"的地板会把**唯一一个已知正确的位姿**(tcp_z 0.8417)判成压桌拒动。**用一个正确的常量喂一个错误的公式,失败会更隐蔽** |
| **修法** | 判据读**指尖中点**:`XR1.tip_center(side, offset)`(取完整 TF 的 R 和 t,`t + R @ offset`),阈值 `TIP_Z_FLOOR = TABLE_TOP + 余量`。横向和竖直修正必须由**同一个旋转矩阵**合成,分成两步算会在歪斜时互相打架(`docs/10` §3.1.1) |

---

## 47. 摇操夹爪扳机是**连续量**,于是「夹住了」判据在**同一天连漏两次**

| | |
|---|---|
| **事实** | `/rm_{side}/rm_driver/teleop_gripper_float` 不是开关。08-11 实测到的指令值:**0.0 / 0.129 / 0.6266 / 1.0**。人扣扳机是一个连续行程 |
| **漏放 1(16:27)** | 判据 `closing and settled and pos >= 20`。按下扳机那一瞬夹爪还停在**全开 839**,`839>=20` 成立、读数也"稳"(它**还没开始动**)⇒ 落盘一条 `读数 839` 的假真值,值 `(−0.0499, −0.4656, 0.3527)` 正是**归零位** |
| **漏放 2(16:35)** | 补上上界 `pos <= 400` 之后又落了一条 `读数 348、指令 0.6266`。扳机只扣一半,夹爪正走在 `523 → 348` 的行程中间,0.5 s 窗口里恰好走得慢于 3.0 ⇒ 判成"夹住"。**8 秒后夹爪回到全开 840,根本没夹住** |
| **同一个失效族的两端** | 漏放 1 是"**还没开始动**",漏放 2 是"**正在动但一瞬间看着不动**"。两者和"停在物体上"在读数上**完全一样**。凡是拿「量不变」当稳定判据,都要另外证明它 (a) 已经开始变过、(b) 不是还在变 |
| **修法** | 四条一起:`cmd >= 0.9`(**指令必须打满**,扣一半不算)、`20 <= pos <= 400`(双侧夹逼)、`settle_s` 0.5 → **1.5 s**、落盘 `hold_s` 让样本事后能自证 |
| **代价是零** | 真夹住的那条在 **124~125 上稳了 365 秒**;两条假的分别稳了 <1 秒。把窗口从 0.5 提到 1.5 s 不会漏掉任何真样本 |
| **顺带拿到的标尺** | 同一只右爪三个读数钉死了阈值:**空爪合到底 6~7 · 夹住 40 mm 黄积木 124~125 · 全开 838~840**。这三个数比任何"取个中间值"的猜测都硬 |
| **为什么值得单独记** | 这是**真值录制器**。它错不会报错,只会产出一条格式完美、数值错误的真值 —— 而真值是用来校准其它一切的,污染顺着整条链往下走。同一天连漏两次说明:**这类判据要一次把失效空间的两端都堵住,补一端等于没补** |

---

## 48. 腕相机的"左/右"是**插线的属性,不是规则的属性** —— 重插一次就变,而且没有任何东西能发现

> 本节 2026-08-11 17:0x 首次写成《左腕相机掉下 USB 总线》,**前提是反的,已整节改写。**
> 掉下总线的是 **4.3 口**,而 4.3 是**右腕**那台(为装 DaBai DW2 而拆)。
> 死 fd 那套机制(下半节)是对的,当时错的只有"哪一侧"。

| | |
|---|---|
| **为什么会错** | 两个腕相机**同型号同 VID:PID**(DECXIN `1bcf:2cd1`),而且 `ID_SERIAL` **两台一模一样**(`DECXIN_CAMERA_DECXIN_CAMERA_01.00.00`,固件写死的常量)。udev 唯一能用的区分是**物理 hub 口号**,所以"左腕/右腕"从来是**一句关于插线的断言,不是量出来的** |
| **厂商怎么写的** | `/usr/lib/udev/rules.d/99-astrabot-wrist-camera.rules`:`usb-0:4.3` → `l_arm_cam`,`usb-0:4.4` → `r_arm_cam`。该文件 `dpkg -S` **查不到归属**,不属于任何包,手工装的,**没有任何记录说明当初是谁确认了哪根线插哪个口** |
| **实测结论(17:15 那一刻)** | 当时 **4.4 = 左腕**,而厂商规则叫它 `r_arm_cam` —— 也就是说**当时的接线与规则相反**。三条独立证据一致:① 单变量拔线实验 —— 操作者物理拔掉左腕单目相机,内核只掉一条 `usb 1-4.4: USB disconnect`;② 画面内容 —— 抓一帧,框上沿两片橙色 blob(13289 / 10679 px)就是同侧夹垫,确实装在腕上;③ 高度自洽 —— 画面是地毯和从低处仰看的桌腿,只有 `left_arm_camera_link` z=**0.386** 看得到,`right_arm_camera_link` z=**0.914** 在 0.8108 桌面之上,该看到桌面 |
| **⚠️ 然后接线又变了(17:29)** | 操作者把左腕相机改插到 **4.3** 口,于是厂商规则**又对了**。现在:**4.3 = 左腕**,4.4 空。`/dev/l_arm_cam → video2`,speed 480,枚举干净。**这正是本节的要点** —— "规则是对的还是反的"根本不是一个可以长期成立的说法,它取决于此刻线插在哪。要判断当下,只有两个办法:让人拔一根线看内核掉哪个口,或者抓一帧看画面里有没有同侧橙色夹垫 |
| **可复现性(实测)** | 拔插一整轮之后再抓帧,两个橙色 blob 的面积和质心几乎逐像素重合(13289→13364 px @ (1049.8,60.8)→(1049.6,61.2);10679→10701 px @ (381.8,54.5)→(381.7,54.5))⇒ **装回去的位姿是可复现的**,这次拔插不会作废已标的腕部外参 |
| **后果(比"死流"严重得多)** | dora 采集图里 `id: left_wrist` 开 `/dev/l_arm_cam`、`id: right_wrist` 开 `/dev/r_arm_cam`,所以**修之前两个腕部通道的左右一直是对调的** —— 厂商采集栈录下的每条 episode,腕部通道标签全是错的。这也解释了 §28/§44 那个悬案(两份 URDF 给左右腕相机写了**完全相同的 origin**、"至少一个 y 符号是错的"):在一个左右通道本身就对调的系统里标外参,必然标出自相矛盾的结果。**修好之前拟合的腕部外参全是废的** |
| **修法(已装)** | `/etc/udev/rules.d/99-astrabot-wrist-camera.rules` —— **同名文件整体覆盖** `/usr/lib` 那份,抗厂商重装(比改 `/usr/lib` 强,见 §12 那六个装机空间补丁的教训)。因为**全机只剩一台单目腕相机且是左边的**,规则让 **4.3 和 4.4 两个口都映射到 `l_arm_cam`**,**插哪个口都认** —— 17:29 那次改口正是靠这一点才没出问题。不再定义 `r_arm_cam`。⚠️ 哪天右腕又装回单目,这两行**必须改回按口区分**,否则两台抢同一个符号链接。`rm` 该文件 + `udevadm control --reload-rules` 即可还原 |
| **为什么改软件不改线** | "软件写错了还是硬件接反了"在这台机器上**不可判定**:序列号相同、规则无归属、hub 口无标签。改软件可逆、且不必再去应力那个已经报过 `error -32` 的接头 |
| **我在这上面栽的坑** | 拿 `dmesg` 插拔史反推左右,**反推错了**。看到"4.3 口当天进出四次、4.4 口开机 24.8 h 没断过",我断言"左腕那台插的就是对的口",并宣布对调故事被推翻。4.3 那四次进出是**右腕**那台在装 DW2 时反复拆装。教训:**插拔史只告诉你哪个口在动,不告诉你哪个口是哪一侧** |
| **顺带查清:4.3 口那 4 次掉线不是接触不良** | 把 `dmesg` 按分钟排开,每次 disconnect 后面都隔 **30~43 分钟**才重新枚举 —— 这是**手在拔插**的节奏(装 DW2 时反复拆装),不是坏接头。接触不良的特征是**秒级反复抖 + `uvcvideo` 流错误**,而 4.3 全程**零条 `uvcvideo` 流错误**。唯一那次 `device descriptor read/64, error -32` + 误识 low-speed 是**一次插入过程中**插到一半的瞬态,自己就升到 high-speed 了。⇒ **别拿掉线次数当线材受损的证据,要看间隔和有无流错误** |

### 48b. 同一次事故的另一半:设备节点消失了,攥着它 fd 的进程不报错也不退出

| | |
|---|---|
| **事实** | 16:38 端口 4.3 的相机掉线后,dora 的 `webcam_node` (PID 2748885) 继续攥着 **`/dev/video4 (deleted)`** —— 设备节点早没了,fd 还开着,进程全绿 |
| **它顺手杀了什么** | 我的只读真值录制器。16:27 那次 run 开着 `chest` + 一路腕相机(开的是 `/dev/l_arm_cam`,也就是 **4.3 那台**);掉线后它攥着死 fd,16:47 强制落盘时 `retrieve()` 拿到空缓冲区 → cv2 在 `imdecode_` 里**抛断言**(不是返回 False)→ 整个录制器 crashed。**不是随机抖动,是有确定原因的** |
| **和 §39 同族** | 那条是串口被重新枚举、驱动攥着废 fd。**同一个失效族,这次发生在相机上**:设备节点消失 ≠ 进程报错,fd 还在,读出来是空的 |
| **判据** | 扫 `/proc/*/fd` 找 **`(deleted)`** —— 这三个字比时间戳对比更直接,一眼定案。**看不出来的做法**:`lsusb \| grep -c DECXIN` 只返回个数不报错;进程列表全绿 |
| **插回去也不会自己好** | 必须重启 dora 图。而重启 dora 会掐掉别人正在跑的摇操采集,是**对外动作** —— 先看 `status.md` 和 `pgrep -af`。另注:dora 起的是**两个** `webcam_node`,右腕单目已被 DW2 取代后,**总有一个节点必然是死的**,光插线不改 dora 配置修不干净 |
| **顺带纠正一个框架错误** | 我一度把"dora 独占腕相机"写成障碍。**那是摇操能工作的原因**(人要在头显里看图),而且摇操期间真值只需要关节角、腕相机闭环只在**自主运行**时用(那时 dora 是关的)—— 两者时间上根本不重叠,不存在冲突 |

---

## 附:反复出现的元教训

把这几条单独拎出来,因为它们**不是硬件知识**,而是我这次犯错的**方式**:

1. **一个在健康状态下也出现的症状,永远不能用来解释故障。**(§3)
   先问"正常的时候有没有这条日志",再建立因果。
2. **不确定的话不要写成肯定句。**(§2)
   我写下的未验证断言会变成未来自己的错误前提。文档里的每句话都会被当作事实读。
3. **能一次实验推翻的假设,先做实验再写文档。**(§3)
   错误结论一旦写进 memory 和 docs,清理成本远高于当初验证的成本。
4. **"资源被占用"时,先看占用者有没有已经把你要的东西暴露出来。**(§5)
   抢占式方案会连带引入解释器隔离、进程间交接、手标外参三个新问题——而它们本来
   都不必存在。
5. **测你关心的量,不要测它的代理。**(§9 数发布者 vs 测流量;§12 用 TF/读回 vs 用 effort)
6. **报错信息经常指向错误的层。**(§7 引号问题伪装成相机问题;§1 环境变量伪装成驱动没起)
7. **"测出来的动作耗时"里可能大部分是我自己的 sleep。**(§16)
8. **在怀疑自己改坏之前,先确认环境有没有在我背后变过。**(§21)
   死 RTC 让一次重启完全隐形;`/proc/uptime` 三秒就能排除掉一整条错误的调查方向。
9. **改了装机目录就必须同时写一条 guard。**(§22 §23)
   静默回退的配置比崩溃的配置危险得多 —— 崩溃会告诉你,回退不会。
10. **自查脚本的结论行必须由测量推出。**(§24)
    "没有硬失败" 不是 "可用"。一个会说谎的绿字比没有检查更糟。
11. **断言全绿 ≠ 输出正确 —— 输出是给人看的东西时,必须自己看一眼。**(§25)
    13 项自检全过,而画面上的中文全被截断。断言只覆盖了「有没有帧」,没覆盖
    「画面上写了什么」。抽帧看图的成本是十秒。
12. **"30" 这种整数在硬件里往往不是整数。**(§27)
    UVC 的 30 fps 是 30.00003。所有跟硬件报数做的**相等/边界比较**都要带容差,
    否则失败方式是"明明支持却选不出来",最难往这个方向想。
13. **等待人类的超时,几乎总是应该设成无限。**(§26)
    带超时的等待死掉时,会把人类唯一的操作入口(弹窗)一起带走,于是从"等一下"
    变成"必须先修复才能操作"。
14. **一条只在"好状态"上验证过的检查,等于没有检查。**(§23)
    guard 必须在**坏状态**(厂商 `.bak`)上真的报错。`Astrabot_Controller.sh` 的
    guard 在 live 上是绿的,在厂商版上**也是绿的** —— 因为改动只是挪了一个 `#`。
    每写一条断言,顺手拿反例跑一遍,成本几秒。
15. **"我清点过我改了什么"的清点范围本身会出错。**(§23)
    我按"`/opt/ros/astrabot/` 下改过的文件"去清点,而第 6 个文件在
    `/opt/ros/start_up/` 下 —— 清点方法决定了它永远不会出现。换成"整个 `/opt/ros`
    里找 `.bak`"才捞到。**用文件系统的痕迹去清点,不要用记忆。**
16. **"写进文档了"不等于"回退时会被发现"。**(§23)
    第 6 个文件在 `../architecture/hardware-map.md` 里老早就有记录,却照样一年半载没人会去比对。
    人读文档,机器读 guard。**改动的记录归文档,改动的守卫归自检脚本,两件事都要做。**
17. **文档写对了不等于代码用对了。**(§30)
    README 和 SKILL.md 里早写明「两个 lead 字段该采信 `rec_confirm_ms`」,而生成报告
    和烧字幕的代码取的一直是另一个。同一条知识要在**每一个消费它的地方**分别落实;
    §23 ⑥ 是文档有、guard 没有,这条是文档有、实现没有,同一种病的两个面。
18. **自己埋的后台任务是环境的一部分。**(§29)
    定时/条件触发的观察者会在我正用同一批硬件时醒来,表现成"凭空出现的故障"。
    埋的时候就要想清楚它触发时会跟谁抢资源;查故障时先问一句"这是不是我自己安排的"。
19. **"状态变了"和"发生过事件"是两个不同的问题,别拿前者的测量去回答后者。**(§31)
    挥手前后的位姿差是 2e-6 rad,而它明明动了 12.5 秒。**周期性的动作会把自己的证据
    抹掉** —— 要判断"发生过没有",就必须在过程中采样,事后快照永远答不了。
    同族的错还有:拿"文件存在"证明"写成功过"、拿"进程活着"证明"一直在发消息"(§见
    `/joint_states` 假死)。
20. **一个"警告"级别的输出,可能是唯一一次预警。**(§32)
    `did not reach target within 0.05 rad` 印出来的时候,电机已经在堵转,而
    `effort` 是 `.nan` 所以没有别的通道能告诉我。我把它当成噪音,接着往同方向发了更
    大的目标,30 秒后 `ros2_control_node` 段错误,整台机器的 `/joint_states` 一起没。
    **在一个连力都测不到的系统里,"没到位"就是最高级别的报警。**
21. **两个数据点能定量级,定不了机制 —— 要定机制就设计一个"两个假设预测相差两倍"
    的实验。**(§33)
    转头 40° 差 592mm 复现了两次,一致到 8mm,可是"符号取反"预测 665~908mm、
    "yaw 没进 TF"预测 354~479mm,实测正好夹在中间,两个都能勉强讲得通。
    再多跑几次 40° 也分不出来 —— **换个角度跑一次**才行(20° 时两个预测差两倍且
    不重叠)。复现是在确认"这事是真的",不是在解释"为什么"。
22. **误差不收敛时,先问"我在迭代的这个映射连续吗",再问"步长对不对"。**(§34)
    残差**非单调**(上下跳)就已经排除了步长假设:调参永远修不好一个每轮换解支的
    映射。同族的坑:任何"随机多起点/重新全局搜索"的求解器放进迭代里,都要先锁支
    (把上一轮的解当这一轮的起点),否则它每轮给的是另一个函数。
23. **外部超时不是安全绳,它只是一封建议信。**(§35)
    `timeout` 发的是 SIGTERM,而多线程 rclpy 脚本连 SIGINT 都无视 —— 实测超时
    2 小时 25 分仍在满载运行。**兜底必须写在会失控的那个循环内部**;外部只有
    SIGKILL 是真的。推论:凡是"跑飞了还有 timeout 拦着"的安心感,都要重新评估。
24. **迭代"输入"迭代不动的时候,试试直接优化"输出"。**(§34)
    我要的是"指尖中点落在积木上",手上的工具是"把 tcp 送到给定点的 IK"。于是本能
    地去**挪 tcp 目标点**,把工具当黑盒。可挪目标点这个映射经过一个随机全局求解器,
    不连续、还带 8mm 的地板。而 `tip_center(q)` 对关节是光滑的 —— 换成**直接对关节
    做局部最小化**,残差从 8.5mm 掉到 0.00mm。判据:如果被你迭代的量要经过一个你不
    信的黑盒才变成你关心的量,就找一条绕过黑盒的路,而不是在黑盒外面调参。
25. **精修必须是净改进,而且不能偷偷改口径。**(§34)
    局部精修在硬判据上**原样复检**(不放宽哪怕 1e-6),不过就退回未精修解;返回的
    `pos_err_mm` 填**真实残差**而不是 0.0 —— 填 0 会被下游读成"IK 完美命中",读到
    它的人只会低估精度、不会高估。判罚做成 hinge(容差带内免费),否则带内的度数
    会去和带外的毫米抢权重。
26. **一个"听起来合理"的解释,先拿量级去卡它。**(§36)
    "CPU 推理慢" 解释得了 2 倍、解释不了 125 倍。凡是解释和实测差一个数量级,就说明
    真因还没找到 —— 而这种解释比"完全没有解释"更危险,因为它会让你停止排查。
    配套的取证手段极便宜:`ls -la` 一下工作目录,看**文件的时间戳和大小**在干什么。
    进程"慢"的时候它到底在算还是在写 I/O,目录会告诉你。
27. **第三方库按 CWD 解析权重路径,所以"我已经下好了"要按目录验证,不是按存在验证。**(§36)
    同一份权重,`cd models && python x.py` 成功、`cd scripts && python ../scripts/x.py`
    失败,而两次的报错都不提路径。凡是子进程跑第三方推理框架,要么 chdir 到权重目录,
    要么显式检查文件在不在 —— 别让它有机会"静默地自己去下"。
28. **报错文案会自信地指错方向,尤其是我自己写的那些。**(§37)
    `is Astrabot_Controller.service up?` 是我当初随手写的猜测,后来它变成了下一次
    排查的**错误前提** —— 和 §2/§3 完全同一个族:不确定的事写成肯定句,未来的自己会
    当事实读。超时类的报错要写成"接下来查什么",而不是"原因是什么"。
    推论:凡是报错建议一个**不可逆或会影响别人**的动作(重启服务、抢占设备),先花
    一条命令去证伪它。
29. **"进程在"证明不了"设备可用" —— 幂等的 bringup 只保证前者。**(§39)
    设备节点被重新枚举之后,旧进程活着、还持有着端口,但那两个 fd 已经指向不存在的
    设备。所有"检查进程是否存在"的守卫在这种状态下都会给出绿灯。**判据要落在你真正
    需要的那个量上**(读得回来数吗),而不是它的代理(进程在吗)—— 这是元教训 5 在
    进程层的复现。顺手可得的取证:`ls -l /dev/...` 的时间戳比进程还新。
30. **判"修好了没有"的工具,不能是本身会假阴性的那个。**(§40)
    重启服务之后新参与者的 DDS 发现还没完成,此时任何短探针都会报"还是没有",于是
    把一次**成功的**修复读成失败,并诱导你再来一遍(而重启是外溢动作,可能作废别人
    正在跑的试验)。验收要用等得够久的测量,而且**验收标准要在动手之前就定好**,
    不能拿动手时顺手的那条命令当验收。
31. **加传感器降的是方差,不是偏差。**(§33)
    ZED 的深度自洽得很漂亮(覆盖率 90.9%、残差 10.66 mm、倾角 0.26°),而 `head_yaw`
    转 40° 就能把同一块没动过的积木算到 59 cm 之外,两次复现一致到 8 mm ——
    **一致到 8 mm 的错误答案。自洽 ≠ 正确。** 再加一路被动立体只会让自洽更漂亮,
    还多一条没标定的外参链。先做误差预算:偏差主导的时候买硬件买不到精度。
    ⚠️ 这条原来引的证据是"桌高偏 62.8 mm、换桌面符号会翻",**那个证据已被推翻**
    (§41:是卷尺过期,ZED 只差 2.0 mm)。结论对、原来的例子错 —— 已换成 §33。
32. **同一句错误文案背后可以是互斥的几个故障,要先列判据树再动手。**(§40 §38 §20)
    "没有 `/joint_states`" 有三个真因、"拿不到 TF" 有两个真因,而它们的修法互相无效
    (重启控制器 / 什么都别做 / 改环境变量;重启 ZED / 补 rsp)。**把判据写成一棵树
    (先测哪个量、什么阈值分开哪两支)**,比记住任何单条修法都值钱。
33. **两个各自自洽的读数互相矛盾时,先怀疑基准,别忙着怀疑仪器。**(§45 §41)
    ZED 每一帧都重测、残差 10.66 mm;卷尺是**一次性的、人做的、而且量完桌子还挪过**。
    我却花了几天把 ZED 当嫌疑人,写下"深度偏 +62.8 mm"甚至"符号会翻"这种需要物理机制
    才能解释的结论 —— 而真相是**尺子过期了,ZED 只差 2.0 mm**。
    可操作的版本:**先按「这个数被测过几次、由谁测、之后现场变过没有」排序,再排查。**
    一次性人工测量是最脆的一环,而且它过期时**不会报错**。
34. **找一个自证的判据,比提高测量精度有用得多。**(§45 §44)
    "夹住了"这件事让指尖中点**就是**积木位置 —— 不经相机、不经外参、不经人的瞄准,
    所以它不需要被信任,它自己成立。相比之下"停稳 2 秒"把人的目测精度焊进了真值,
    而"卷尺量桌高"把一次性人工操作焊进了整条链。**先问「有没有一个物理事实能替我
    做这个判断」**,再考虑怎么把测量做准。

---

## §49. DW2 在发真深度,但它的 frame 不在 TF 树里 —— 不是"外参不准",是图里没这条边

2026-08-11 17:50 实测(只读,摇操运行中):

| 量 | 值 |
|---|---|
| 话题 | `/camera/depth/image_raw`(还有 `/camera/ir/*`),节点 `/camera/camera` |
| 帧率 | **14.9 Hz**(IR 14.8) |
| 分辨率 / 编码 | **640×400**,`16UC1`(单位 mm) |
| 有效像素 | **90.3%**(231296/256000) |
| 深度范围 | **834 ~ 2613 mm**,中位 **1264 mm**;画面中心 40×40 全有效、中位 1298 mm |
| **frame_id** | **`camera_depth_optical_frame`** |

**`camera_depth_optical_frame` 不在 TF 的 39 个 frame 里。**

```
tf2_echo base_link camera_depth_optical_frame
  → Invalid frame ID "camera_depth_optical_frame" — frame does not exist
```

⚠️ **`grep camera_depth_optical_frame` 会骗你**:它命中的是
`chassis_left_camera_depth_optical_frame`(底盘那台,9.4 Hz,**这台的 frame 是齐的**)。
判据必须是全等匹配或 `tf2_echo`,不能用子串。

**为什么这比"外参标错"更严重**:标错了至少还有一条边可以修正。这里**根本没有边** ——
640×400 的深度值相对 `base_link` 在几何上没有意义,任何像素都变不成机器人能去的坐标。
补它要 6 个数(安装位姿),**一个都没量过**;而且**它装在左腕还是右腕目前只有操作者口述**
(§48 那次 300 s 相关性实验期间手臂没动过,判定不了)。

**连带的一条风险(未测)**:整帧最近像素是 **834 mm**,而抓取时积木离腕部只有
100~300 mm。若 DW2 最小工作距离≈800 mm,它在**抓取瞬间是瞎的**,只能远距引导。
测法:把手伸到积木上方 15 cm,看有没有有效像素。**2 分钟,但需要摇操停。**

**同时记下的旁证**:全机 **0 个 `PointCloud2` 话题** —— 点云只来自
`Astrabot_ZED_Points.service`,它现在 `failed`(为摇操让路),zed frame 只剩 **1 个**(正常 6)。
所以"头部 ZED 带点云"这句话在摇操运行期间**是不成立的**。

### 反过来,这一天最有用的发现:积木已经在手里 = 标定可自动化

手里夹着的积木,位置由 FK 已知(指尖中点),而 ZED 和 DW2 **都能看到同一个刚体**。
让手臂自己摆 5~6 个位姿,每个位姿同时记
`(FK 指尖中点, ZED 像素+深度, DW2 深度)` ⇒ **一次同时解出两台相机的外参**。
这把手眼标定从"需要人反复瞄准"降级成"让机器人自己摆姿势",**不再需要人在环**。

---

## §51. 重启厂商 dora 采集图要**三个**前置条件,而且每次报错都指向错误的方向

2026-08-11 18:00 重启 `astrabot_data_collection_xr1_evt2.yml` 花了**四次**才成功。
三个条件缺一不可,报错却没有一次指向真凶:

| # | 前置条件 | 缺了的报错 | 为什么误导 |
|---|---|---|---|
| 1 | `export PATH=/home/astrabot/deploy/.venv/bin:$PATH` | 每个 python 节点 `ModuleNotFoundError: No module named 'hardware'` | yml 写的是 `path: python` —— **裸名,从 PATH 解析**。直接调 `./.venv/bin/dora` 二进制时它解析成 `/usr/bin/python`。报错像"包没装",其实是**解释器选错** |
| 2 | `export ROS_DOMAIN_ID=12` | `control_astrabot` 抛 `[astrabot_robot_controller]: failed to connect TF`(前面刷一堆 SHM 错,10 s 后抛) | 看着像 TF 挂了(§38 那个故障)。实测 `/tf`+`/tf_static` 都在、`rsp` 活着 —— 它只是在 **domain 0** 上看了个空图。**那几行 `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` 是噪音**,我照着它去查 `/dev/shm` 白花了一轮 |
| 3 | 先 `sudo -A systemctl stop Astrabot_ZED.service Astrabot_ZED_Points.service` | `eye_zed` 抛 `Failed to open ZED camera: CAMERA NOT DETECTED` | 听着像相机掉了 —— 但 `lsusb` 里它明明在。见下 |

**我最初判断"dora 侧不走 ROS,所以不用设 domain"的依据是错的:**
我 grep 了 `robot_runtime/` 没找到 `rclpy` 就下了结论。但 `control_astrabot` 来自
**另一个包、另一个 venv**(`/opt/astrabot/venv312` 的 `hardware.robots.astrabot`),
它 `import rclpy` + `rclpy.spin` + `_wait_for_tf()`,代码里不写死 domain,**全靠环境继承**。
**教训:"这个栈用不用 ROS"不能只查一个包。**

### 🔴 `Astrabot_ZED.service` 是 `Restart=always` + `RestartUSec=100ms`

**dora 一松手,systemd 在 100 ms 内就把 ZED 抢回去。** 实测:我 17:56 拆完图,
服务的 `ActiveEnterTimestamp` 就是 **17:57:42**,journal 显示是自动重启不是人为。
所以「dora 抢得比服务快」是**不可能成立**的,必须**显式 `systemctl stop`**
(显式 stop 不触发 `Restart=always`)。

原来 SKILL trap 6/24 只写了反方向那半句(「dora 跑着时别 `systemctl start` ZED」),
**漏了这半句** —— 于是拆图之后的窗口里服务自动回来,下一次启动必然失败。
⚠️ 停 ZED 服务是**对外动作**:整张 ROS 图上 `/zed/...` 全消失,别人的感知脚本会瞎。
(`Astrabot_ZED_Points` 本来就是 `failed`,不是停 ZED 造成的。)

**正确的启动命令:**

```bash
sudo -A systemctl stop Astrabot_ZED.service Astrabot_ZED_Points.service
cd /home/astrabot/deploy
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export VIRTUAL_ENV=/home/astrabot/deploy/.venv PATH=/home/astrabot/deploy/.venv/bin:$PATH
.venv/bin/dora run .astra/astrabot_data_collection_xr1_evt2.yml
```

### 拆图:按名字 `pgrep` 一定漏,而 `pkill -f` 会杀掉你自己

`dora run` 被 SIGTERM 收掉后,**10 个子节点全部孤儿化(ppid→1)继续跑**,
而它们的模块名各不相同。我按模式列表杀了**三轮**才干净(每轮都以为干净了),
漏掉的旧 `control_astrabot` 实例还让我把新实例的 `failed to connect TF`
**误诊成"旧实例占着端口"** —— 一个完全错误的因果。

可靠的枚举是**按启动时刻的 PID 段**(同一次 `dora run` 的子进程 PID 连号):

```bash
ps -eo pid=,lstart=,cmd= | awk '$1>=2748600 && $1<=2749000'
```

而 `pkill -f <模式>` / `pgrep -f <模式>` **会匹配到你自己这条命令行**并把 shell 杀掉。
SKILL trap 5 只对 `g2_gripper_node` 写了 `g2_gripper[_]node` 的括号技巧,
但这是**通用问题**:我在同一个会话里**栽了两次**(第二次是
`'robotd_bin/robot_daemon'`、`'robot_control_node_odom'` 这几个我忘了加括号的模式)。
稳妥写法是**显式排除自己**,别靠记得加 `[ ]`:

```bash
me=$$; ps -eo pid=,cmd= | grep -E "$pats" | grep -v ' grep ' | awk -v m=$me '$1!=m{print $1}'
```

### 验证必须用正面判据 —— 「没有报错行」和「压根没起来」输出一模一样

`log_left_wrist.txt` 里那条 `[webcam] ... driver negotiated WxH@F` **只在协商失败时**打印。
所以"日志里没有 `[webcam]` 行"既可能是"正常打开",也可能是"节点没起来日志是空的"。
**我拿它当成功判据误判过一次**(§16 同一类错)。能用的正面判据:

- `grep 'Camera successfully opened' log_eye_zed.txt`
- `grep 'Setup completed' log_control_astrabot.txt` ← TF 真接上了
- `ls -l /proc/<webcam pid>/fd/3` 指向 `/dev/videoN` 且**不带 `(deleted)`**(§39/§48b)
- **别用 `/proc/<pid>/fdinfo/3` 的 `pos:`** —— V4L2 字符设备偏移**恒为 0**,零分辨力。
  用 **CPU 时间增长**:实测 `left_wrist` 6 s 涨 127 ticks(≈21% 核,在解 MJPEG),
  而黑帧的 `right_wrist` 也涨 71 ticks(≈12% 核)—— 后者说明**失败的那一路不是空转**。

### 附带定案:右腕永久只有 DW2,`right_wrist` 通道从此是黑帧

操作者 2026-08-11 定案「右手算了就不连接了以后也不用,右手已经有双目了」。
所以 §48 那条"两个口都映射到 `l_arm_cam`"的 udev 规则**是永久的**,
中途因"相机还在腕上只是线拔了"打算改回按口区分,被这句推翻,**没有改**。
而 `webcam_node.py` 打不开设备**不退出**,填黑图照样 `send_output` ⇒
**黑帧会被当正常图像录进数据集**。恒黑通道**不像左右对调那样有毒**(只是白占容量),
但它会**焊进数据集 schema**:先录 N 条再删节点,前后两批不兼容 —— 要删就趁早。

---

## §50. `Astrabot_ZED.service` failed 的根因可能在 **USB 接口的驱动绑定**上,而不在 zed_wrapper

2026-08-11 18:03 实测。17:57 我刚把 ZED 起好(6 个 zed frame、2 个点云话题,
背景等待器 8 s 就报就绪),18:06 再用时**两个 ZED 服务都是 `failed`**,
而 `ros2 topic list` 里 3 个 zed 话题**还在**(陈旧发现,`topic hz` 一个都不出数)。

### 判据链(照这个顺序问,4 步定位)

```bash
systemctl status Astrabot_ZED.service --no-pager -n 25     # 1) 看是哪个 ExecStartPre 退非零
lsusb | grep -i 2b03                                       # 2) 相机在不在 USB 上(f880 视频 + f881 HID)
echo 1 | sudo -S /usr/local/bin/prepare-zed-usb.sh         # 3) 让它自己说话
# 4) 数 f881 那个接口的 driver
for d in /sys/bus/usb/devices/*; do [ -f "$d/idVendor" ] || continue
  [ "$(cat $d/idVendor)" = 2b03 ] || continue
  for i in "$d":*; do [ -d "$i" ] &&
    echo "$(basename $i) class=$(cat $i/bInterfaceClass) driver=$(basename $(readlink -f $i/driver))"; done; done
```

本次的读数:

| 项 | 值 | 含义 |
|---|---|---|
| `ExecStartPre=/usr/local/bin/prepare-zed-usb.sh` | **status=1/FAILURE** | 相机还没被打开就挂了 |
| `restart counter is at 5` → `Start request repeated too quickly` | — | 所以最终态是 failed 而不是 restarting |
| `lsusb` | `2b03:f880` + `2b03:f881` **都在** | **不是掉线,不是线松** |
| 脚本自述 | `ZED HID interface is already owned by another process: 1-2.2:1.0` | 就是它 |
| `1-2.2:1.0` | class=03 **driver=usbfs** | 有进程用 **libusb** 绑走了 HID 接口 |
| `2-1:1.0/1.1` | class=0e driver=uvcvideo | 视频接口反而是好的 |

**`driver=usbfs` = 某个 libusb 进程持有它**;正常态是 `driver=usbhid`。

> ⚠️ **修正(另一会话 18:19 实测):`driver=usbfs` 本身不是故障判据 —— 它也是
> ZED 服务正常运行时的常态。** 我在 dora 全停、服务 `active`、**6 个 zed frame 都在**
> 的状态下量到 `1-2.2:1.0 class=03 driver=usbfs`,持有者是
> `component_container_isolated`(pid 14171)—— **zed_wrapper 自己**。
> 也就是说 `usbhid` 只是 **`prepare-zed-usb.sh` 跑完、相机还没被打开** 那个瞬间的状态;
> 一旦 wrapper 打开相机,它自己就用 libusb 把 HID 接口绑成 usbfs。
> 所以 **必须解析持有者是谁**(下面那段 `/proc/*/fd` 扫描),
> 不能看到 usbfs 就断定"被抢了" —— 否则会在健康的系统上"修"出故障来。
> 这正是本文件元教训 5 的又一例:**测你关心的量(谁持有),不要测它的代理(绑到哪个驱动)**。
>
> 好消息是**恢复不需要人工**:持有者一死,接口的 driver 变成 `""`,
> `prepare-zed-usb.sh` 会自己 bind 回 usbhid。所以让路之后只要
> `systemctl reset-failed Astrabot_ZED.service && systemctl start Astrabot_ZED.service`
> 就够了(18:13 实测:20 s 后 active,30 s 后 6 个 zed frame 齐)。
`prepare-zed-usb.sh` 的逻辑就是:`""` → 重新 bind 回 usbhid;**`usbfs` → 直接 `return 1` 放弃**
(它刻意不抢,因为抢了会把对方搞崩)。所以这个失败是**设计出来的让路**,不是 bug。

### 找持有者

```bash
for f in /proc/[0-9]*/fd/*; do t=$(readlink "$f" 2>/dev/null) || continue
  case "$t" in */bus/usb/001/004*) p=${f#/proc/}; ps -o pid=,lstart=,cmd= -p "${p%%/*}";; esac; done
```
(`001/004` 从 `lsusb` 那行的 Bus/Device 号来。)本次抓到
`python -m hardware.devices.sensors.camera.single_zed_node`,启动时刻 18:02:28 ——
**厂商 dora 采集栈的 ZED 节点**,由**另一个 Claude 会话**(pts/5)在 18:02:25 拉起。

### 为什么这条值得单独记:它和 §38 长得不一样,修法相反

| | §38(zed frame 集体消失) | §50(本条) |
|---|---|---|
| 服务状态 | **active** | **failed** |
| 图像话题 | 照常在发 | `topic hz` 出不来数 |
| zed frame 数 | 1(正常 6) | 1(正常 6) |
| 根因 | zed_state_publisher 哑了 | HID 接口被 libusb 抢走 |
| 修法 | `systemctl restart Astrabot_ZED.service` | **restart 一万次也没用**,必须先让持有者松手 |

**"zed frame 只剩 1 个"这一个症状对应两种完全不同的病**,
所以下手前必须先看 `systemctl is-active` —— active 走 §38,failed 走本条。

### 同机多会话:这是**别人的进程**,不能直接 kill

`ListAgents` 显示本机还有 2 个 Claude 会话在跑。抢占前用 `SendMessage` 问一句,
理由和 §29(外置录制器)完全同类:**独占资源上,静默抢占会让对方的试验作废且毫无痕迹**。
反向也成立 —— 我 17:57 起 ZED 的时候如果对方正要起 dora,我就是那个抢的人。

**祖先链能定位是谁干的**,比猜快得多:
```bash
pid=<PID>; while [ "$pid" != 1 ]; do ps -o pid=,ppid=,lstart=,cmd= -p $pid; \
  pid=$(ps -o ppid= -p $pid|tr -d ' '); done      # 一直走到 sshd,就知道是哪个 pts
```

### 附带确认:积木还夹着

`/qg_robot/gripper_right_state` = `[124, 0, 0, 0]` —— dora 起来了、ZED 被抢了,
但**右爪没松**,§49 末尾那个"积木在手里 ⇒ 标定可自动化"的基准仍然有效。
唯一能毁掉它的动作是**开右夹爪**(给 `teleop_gripper_float` 0.0);
手臂随便动都不要紧,FK 跟着走。已就此给对方会话发了明确提醒。

## §52. 腕部相机看的是爪子**前方**,不是爪子**下方** —— 别拿 `wrist_scan` 的"0 px 黄色"判"积木不在"

2026-08-12 16:0x,我自己刚踩进去又爬出来的一个:差一步就把整个像素闭环方案否掉。

`experiments/wrist_scan/20260812-151749`(扫 y)和 `20260812-152310`(扫 x)
两组扫描,在 ZED 报的目标位附近**每一行都是 0 px 黄色**,而少数出数的行
`Z_mm` 是 **10975 / 11853** —— 十一米。当时的推论是"积木根本不在那儿 / 已被推出可达带",
按这个推论下一步就该去重标外参而不是写闭环。**推论是错的,数据没错。**

### 判据:准心在哪,一张图就够

`experiments/20260812-07/hold_wrist.jpg`(抓取位、爪全开、爪下空桌面)做 orange 连通域:
两片夹垫质心 **(110.0, 405.4)** 和 **(460.7, 406.3)**,中点 **(285.4, 405.9)**。
画面 640×480 ⇒ 两指中点在**画面下缘**,离画面中心竖直差 **166 px**。
夹垫上方那整幅画面拍的是夹爪**前方**一路铺开去的桌子。

于是那两组扫描的读数全都自洽:
- 爪下那块(准心附近)只剩 74 px 的余量,积木稍微偏向机器人这侧就直接出画 ⇒ **0 px**;
- 画面主体是远处 ⇒ 偶尔出数的"黄色"是**远墙**,所以 Z=11~12 m。

| | 我当时的读法 | 实际 |
|---|---|---|
| 0 px 黄色 | 目标位没有积木 | 目标在准心那一侧的**画外** |
| Z=11.9 m 的黄色 | 深度乱了 | 远墙,深度是对的 |
| 该做什么 | 重标手眼 | 把准心当基准写闭环 |

### 连带修掉的一个**闸门 bug**(比误判本身值钱)

`--wrist-off-max 0.5` 原先比的是**离画面中心**的偏移。一块**正对准两指中点**的积木
读出来是 `off_center=(-0.109, +0.692)` —— 0.692 > 0.5,**每一次瞄准正确的抓取都会被下降闸拒掉**。
14:14 那次数据碰巧没暴露它(黄色只有 71 px,竖直偏心恰好 0.002,看着"居中",
其实离准心 166 px)。现在闸门判 `off_aim`(离准心),`off_center` 只留着和历史日志可比。
`scripts/test_wrist_aim.py` 把"正对准 ⇒ off_center>0.5"写成断言,防止有人把常数改回画面中心。

### 元教训

**"某个传感器没看到"只有在你知道它看的是哪儿之后才是证据。**
外参没标出来(`wrist_extrinsics/20260812-150406-d455` 那 9 帧 `det` 全 null)
不等于没有可用基准 —— 夹垫是相机自己画面里的刚性参照物,量一次一直有效,
比标一次外参便宜得多。和 §33 同类:先确认视场,再解释视场里的空白。

## §53. 手眼「修正」本身就是误差源 —— 一个自洽的 LOO 11.24mm 掩着 230mm 的反号偏移

2026-08-12 17:03。上一节刚修好准心,接下来两次抓取分别被下降闸(§52 的正确闸门)
和腕部闭环拒掉,理由都是"腕相机在抓取位一个黄色像素都看不到"。这一次**闸门没冤枉谁**:
同一时刻的 ZED 照片里,**夹爪根本不在取景框内**,而积木清清楚楚在画面中央。

链条上每一环都自称正确:检测像素 `px [764,433]` 落在积木上(圈画确认)、
IK 残差 0.0mm、地板检查余隙 13mm、FK 反算指尖中点和目标残差 0.0mm。
**两个"同一个坐标"不是同一个物理位置。**

### 判据:让夹爪自己当标记物,不做任何拟合

`scripts/zed_hand_probe.py`:把指尖停在桌面上方 90mm 的 5 个 **FK 已知**位姿,
每个位姿拍一张 ZED,直接比「相机模型预测的夹垫像素」vs「夹垫实际出现的像素」。
没有外参、没有拟合、没有深度,只有一个纯观测量。

| | Δu 均值 (px) | 备注 |
|---|---|---|
| 套 `handeye/correction.json` | **+228.5** (std 47.5) | 5 个位姿**同号** |
| 不套(原始相机模型) | **−9.6** (std 11.3) | 两片夹垫都可见的那两个位姿 +2.8 / −33.7 px ≈ 5~57mm |
| 取反的修正 | −279.4 | 更差 |
| 只用 M / 只用 t | +141.9 / +136.4 | 都更差 |

折到目标上:黄积木 px(764,433) 修正后 (0.396,−0.435),**不修正 (0.382,−0.205)** ——
差 **230mm**,正好是"手臂扑空、且连 ZED 取景框都没进"的那个量。
`correction.json` 自称 `no_correction_loo_mm = 218.85`,**和实测反号**。

### 为什么 LOO 11.24mm 骗得过去

那 15 个样本全是**积木被举在空中**取的(z 0.857~1.023),而桌面抓取高度 0.8238 ——
每一个真实目标在 z 上都是盒外外推。刚体拟合把"光心挪 [54.9,−133.6,−20.8]mm"
拟得内部极其自洽:LOO 是**盒内自洽度,不是对世界的正确性**。
留着同一批样本重新拟合、交叉验证、换个正则,永远看不出这件事。

### 元教训

**要推翻一个标定,不能靠重新拟合它自己的样本,只能靠一个链外的独立观测。**
夹爪自己就是最便宜的那个观测:它的位姿 FK 已知、它在相机画面里显眼(橙垫)、
它想停哪儿停哪儿。`USE_HANDEYE = False` 现在是默认值,理由写在
`scripts/grasp_block.py` 那个开关上面;`experiments/handeye/correction.json` 作废但不删。

⚠️ `zed_hand_probe.py` 的 `pads_px` 有个已知弱点:只看到一片夹垫时它会把桌上那个
恒定的橙色果篮团(173.5, 202.2)拉进来平均,污染了 5 个位姿里的 0/1/3。
**只有两片夹垫都可见的位姿(4、5)是无偏的** —— 上表的结论靠的是这两个 + 全体同号。

## §54. 「中转点抬得够高就能绕过桌子」是错的 —— 通不通取决于 IK 解支,不是高度

2026-08-12 18:0x。零位出发的关节直线必扫桌板(§43),已经写好的三条备用路线
(`high_over_target` / `lateral_high→high_over_target` / `lateral_high`)**三条全废**。
直觉上的下一步是"把中转点抬更高",于是离线(`/tmp/route_probe.py`,只用 URDF+IK+
碰撞检查,不连机器人不跑感知,几秒一次)把 x=0.15 这根竖线上四个高度全试了一遍。

目标 (0.360,−0.183),`gz=0.8238`,开合方向 45°,起点零位(指尖 z=0.353,**在桌面
0.8108 以下**):

| 中转点 | 高度 | 零位→中转 | 中转→预抓取 |
|---|---|---|---|
| `lateral_mid`    | gz+0.15 (0.974) | ✅ 通 | ✅ 通 |
| `lateral_high`   | gz+0.30 (1.124) | ❌ `arm_5_link` 插进桌面 **273.8mm** @ 路点 3/21, x=+0.204 | ✅ 通 |
| `lateral_higher` | gz+0.45 (1.274) | ✅ 通 | ✅ 通 |
| `high_over_target` | gz+0.30, x=0.36 | ❌ `arm_6_link` **328.5mm** @ 路点 3/20 | ✅ 通 |
| (直达) | — | ❌ `arm_7_link` **347.4mm** @ 路点 3/16 | — |

**更低的 gz+0.15 通,中间的 gz+0.30 撞,更高的 gz+0.45 又通。** 不是单调的,所以
"高度不够"解释不了它。撞的是那个高度上 `ik_center` 交出来的**解支**:同一个笛卡尔
中转点有多个手腕/肘部构型,零位→它的关节直线扫过哪里由构型决定,和中转点自己的 z
基本无关。四条失败的腿全部死在**路点 3/16~21(前 15%)、x≈0.20~0.26**,也就是刚
迈过 `TABLE_X_MIN=0.2`、手臂还挂在身侧最低的那一瞬 —— 失败由**起点位形**主导,
不由目标或中转点主导。

### 修法(已落进 `grasp_block.py`)

中转点列表多给两个同 x、不同高度的候选 `lateral_mid`(gz+0.15)和
`lateral_higher`(gz+0.45),并在三条老路线之后各追加一条单中转点兜底路线。
代价是每次多两次 `ik_center`(每次分钟级),换来的是不再整轮拒动。

**不许做的两件事**(它们都会把拒动条件本身拆掉,而拒动这次是对的):
调大 `plan_path` 的 `step_max`(只会让碰撞检查变稀,§43),或放宽
`path_floor_check` / `tip_clear_mm`。

### 元教训

路线是**离线可判定**的:URDF + IK + 碰撞检查就够,不需要机器人也不需要感知。
一次真机跑要等 20 分钟感知+IK 才知道路线不通,离线几分钟能把一整条竖线扫完。
凡是"只跟运动学有关"的失败,先离线穷举再上机器人。

## §55. 「抓取解贴着预抓取解找」只是注释,不是约束 —— 12cm 下降变成了整臂重构

2026-08-12 18:31。`20260812-16` 那次 dry-run 在 `plan` 阶段自称一切正确:
`ik_err_mm 0.0`、`tip_resid_mm 0.00036`、`tip_above_table_mm 13.0`、`clearance_mm 10.55`,
然后被静态地板检查以 **7.4mm < 8mm** 否掉整轮。只差 0.6mm,看着像闸门太严。

**不是闸门的问题。**把记下来的 `q_pre`/`q_grasp` 拿出来看(`/tmp/floor_where.py`,
纯 FK+碰撞,秒级):

```
q_pre   [ 1.632 -0.167 -1.144 -1.006 -0.509  1.243 -1.930]
q_grasp [-1.357 -2.784  0.603 -1.089 -1.721 -0.074 -2.618]
逐关节 |Δ| [2.989 2.617 1.748 0.084 1.212 1.317 0.688]  max 2.99 rad
```

两端指尖分别是 (0.5397,−0.1584,**0.9438**) 和 (0.5397,−0.1584,**0.8238**) —— 一个
**竖直 12cm 的下降**。中间那 20 段却让指尖 `y∈[−0.813,−0.158] z∈[0.810,1.241]`:
先甩到积木右侧 65cm、升到桌面上方 43cm,再擦着桌面 **−1.8mm** 回来。
调用处的注释写着"抓取解要贴着预抓取解找,保证下降是一小步而不是重构位形",
可实现只是把 `pre["q"]` 当**种子**传进 `ik_center` —— 种子是偏好,不是约束。

### 为什么每一道检查都放行了

* `ik_center` 按**残差**挑 best。两个解支的残差都是 0.0mm,谁小谁上,和"能不能走过去"无关。
* `plan_path`/`collide()` 查的是**连杆** vs 桌面半平面。指尖中点不是连杆
  (`tcp_link` 是单根手指,§tcp-link),所以擦桌面这件事它看不见。
* `path_floor_check` 是唯一看指尖中点的,它逮住了 —— 但它在 `plan_to` 选完路线**之后**
  才跑,而且下降段 `path_gr` 不随路线变,所以它只能一票否决整轮,没法换个方案。

三道判据各看一部分,拼起来才是"这条路能不能走",而它们是串行的、后面那道只有否决权。

### 修法(三处,全部是**收紧**,没有放宽任何现有判据)

1. `ik_center(..., max_dev=)`:超出 `max_dev` 的解**不许当候选**(不是事后再否),
   精修也不许把解推出去。`DESCEND_MAX_DEV = 0.6` rad —— 12cm 笛卡尔位移落在 ~0.5m
   臂上约 0.24 rad/关节,留 2.5 倍余量。抓取解这一步用它。
2. 调用处再核一遍 `dg`(它本来就是 `ik_center` 返回的 max|q−q_ref|,以前没人看)。
3. `plan_to(..., accept=)`:路线拼好后过一次指尖地板检查,不通过就**试下一条**,
   而不是等选完了一票否决整轮。自检 `scripts/test_plan_to_accept.py`。

### 元教训

**注释里的"保证"要么是代码,要么是假的。** 这一条注释写对了意图、写对了理由,
然后传了个种子就完事了 —— 而种子在多起点随机搜索里只是"抽中了就偏好一下"。
凡是"保证 X"的注释,要么有一个判据能让它失败,要么就把它删掉别骗后面的人。

⚠️ 还没定的一件事:**这个目标(x=0.54,可达带 0.18~0.56 的边缘)在同一解支下到底有没有
抓取解**。如果没有,那 12cm 就只能靠换支到达,下一步是挪底盘/把积木推近,而不是改判据。
