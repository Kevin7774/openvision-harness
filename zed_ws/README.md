# zed_ws — 「学会放积木」工作区

Astrabot XR1（Jetson Thor / ROS 2 Jazzy）上的桌面积木整理任务工作区。
按你给的三阶段结构组织：**Goal + 自检 → 策略清单 → sub agent 执行协议**。

创建于 2026-08-07，最后更新 **2026-08-11**。所有结论均来自本机实测（见各文档的数据来源说明）。

---

> ## ⚠️ 动手之前先看这一条：`ROS_DOMAIN_ID=12`
>
> 整台机器人跑在 **domain 12**。手工终端默认是 **domain 0**，两个域在 DDS 层完全隔离，
> 而且**不报任何错** —— 只是 `ros2 topic list` 空、`ros2 control` 卡住、你发的指令机器人收不到。
> 实测对比：domain 0 → 1 节点 / 6 话题；domain 12 → **52~62 节点**（2026-08-10 复测；
> 随 MPC / ZED / 夹爪驱动起停波动。08-07 首测是 44 节点 / 109 话题）。
> ⚠️ 而且 **DDS 发现不是瞬时的**：`Node()` 之后立刻 `get_node_names()` 会返回 1 个节点、
> 把活着的 `controller_manager` 报成 ABSENT。要自旋到数目稳定（实测 0.7~0.9 s）再采信。
>
> ```bash
> source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=12 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
> ```
>
> 完整原理与其它 9 条已知缺陷见 [`docs/07_hardware_map.md`](docs/07_hardware_map.md)。

---

> ## 🟢 「现在卡在哪、下一步做什么」→ [`STATUS.md`](STATUS.md)
>
> 本 README 记录的是**方法与结论**(哪些假设被推翻、为什么这样设计),它更新得慢,
> 而且下面「当前状态速览」那一节是 **08-07 的快照**,已经不代表今天。
> 机器当下的子系统状态、今天修了哪些故障、试验账本、按杠杆排序的卡点,都在
> [`STATUS.md`](STATUS.md)(最后更新 **2026-08-11 10:45**)。硬结论仍看 [`PITFALLS.md`](PITFALLS.md)。

---

## 文档

| 文件 | 阶段 | 内容 |
|---|---|---|
| [**`STATUS.md`**](STATUS.md) | — | 🟢 **现状快照(会过期的那一份)**:子系统逐项状态 + 支撑它的实测数 / 今天修掉的四个故障 / **38 次成形试验 `grasped` 0** 的账本 / 按杠杆从高到低排的 6 个卡点 / 下一步 / **动手前必须知道的三件事**(摇操在跑、录制器独占、可能有别的会话在动这台机器) |
| [**`PITFALLS.md`**](PITFALLS.md) | — | 🔴 **踩坑指南(先看这个)**:本机踩过的每个坑,按 **症状 → 错误判断 → 真实原因 → 正确做法** 排。**44 条坑 + 1 条「尚未查清」+ 32 条元教训**(§39~§44 是 2026-08-11 上午当天踩到并修掉的:**设备节点被重新枚举后驱动进程还活着但攥着废 fd,SIGTERM 无效只有 `kill -9`**、**「没有 `/joint_states`」有三个互斥真因所以要按判据树查**、**白桌面深度偏了 +63 mm —— 和粉桌布那次的 −48 mm 反号**、**`os._exit()` 不 flush 导致 exit 0 却输出空文件**、**上一轮摇操把手臂停在桌面以下会让所有路径规划判碰撞**、**腕相机 URDF origin 左右完全相同而关节轴是镜像的**)。含 `ROS_DOMAIN_ID=12` 沉默陷阱、空 `gripper_list` 导致的 SIGABRT、被实验推翻的串口冲突假设、pyzed 必然失败的真因、单进程 vs N 次 shell 调用的 3 个数量级差、**夹爪驱动不是 systemd 服务所以每次重启后静默消失**、**头部 ±40° 限位其实是我加的(厂商是 ±3.1 占位值)**、**装机目录被我改过的是 6 个文件不是 1 个,其中 1 个在 `/opt/ros/start_up/` 下、决定头颈有没有命令接口**、**自检脚本 0 failure 却打绿字说一切可用**、**只在好状态上验证过的 guard 等于没写**、**录制器是独占的:自己埋的后台任务会抢走它并挤进当前 run**、**报告的"证据列"取了带钟差的那个字段,还烧进了改不了的字幕**、**判「动没动」不能用动作前后的快照之差(挥手前后差 2e-6 rad),而采样线程又不能 spin**、**一次关节堵转把 `ros2_control_node` 打成 SIGSEGV,服务还是 active 但 `/joint_states` 整个没了** |
| [`docs/01_goal.md`](docs/01_goal.md) | 一 | Goal「**让「学会放积木」自己转起来**」+ **S 层（单条 episode，S1–S6）与 L 层（循环，L1–L4）双层判据** + 自检摘要 B1–B10 |
| [`docs/02_robot_body.md`](docs/02_robot_body.md) | 一 | 机器人本体信息 + **是否具备执行条件**（逐项自检 B1–B7 / W1–W8） |
| [`docs/03_scene_physical.md`](docs/03_scene_physical.md) | 一 | 场景物理条件：场景里有什么（**基于实拍**） |
| [`docs/04_strategies.md`](docs/04_strategies.md) | 二 | 一系列可选策略（A–E 五组，30 条，带 `skill_id`） |
| [`docs/05_experiment_protocol.md`](docs/05_experiment_protocol.md) | 三 | 实验设计 E0–E4 + 7 个 sub agent 分工 + 假设 H1（物理空间）/ H2（力度）|
| [`docs/06_teleop_bringup_sop.md`](docs/06_teleop_bringup_sop.md) | — | **VR 遥操作启动 SOP + 失败经验**：Quest→LiveKit→dora 链路怎么稳定拉起，2026-08-07 打通那半天踩的全部坑（crash-loop 服务 / PATH bug / DuplicateIdentity）|
| [**`docs/07_self_improving_loop.md`**](docs/07_self_improving_loop.md) | 二 | 🔴 **自进化循环的施工文档**：循环状态机（重置 = 一条 episode）/ 散开采样器（重置器 = 域随机化器 = 课程表）/ **物理清单 P1–P6**（挪底盘、贴胶带、装挡边）/ 要写的 3 个脚本 / **MTBH ≥ 8 h 验收门槛** |
| [`docs/07_perception_scheme.md`](docs/07_perception_scheme.md) | — | **感知方案**（已端到端实测）：ZED 聚合深度拟合桌面平面 + HSV 颜色 blob + 射线投射 → base_link。**逐像素深度不可直接读**（粉桌布上同平面三块差 25 cm）；换成白桌面后覆盖率 **90.9%**、平面残差 σ=**10.7 mm**、拟合桌高 **0.8128 m**——**与摇操真值 0.8108 只差 2.0 mm，绝对高度是准的**（原先说的"+62.8 mm 偏差、符号会翻"是拿过期卷尺当基准造出来的，已推翻，`PITFALLS.md` §41）|
| [`docs/07_hardware_map.md`](docs/07_hardware_map.md) | — | **本机硬件与配置权威地图**：`ROS_DOMAIN_ID=12` 陷阱 / 平台与网络 / CAN 手臂映射 / 串口全表 / USB 拓扑 / 相机与 udev / ros2_control 现状 / URDF 用哪个包 / ZED 配置改动 / **12 条已知缺陷** / 复查命令速查 |
| [`docs/08_gripper_g2_driver.md`](docs/08_gripper_g2_driver.md) | — | **夹爪专题**：UFactory G2 走 Modbus RTU，左 `ttyAMA5` 右 `ttyUSB0` @2M slave 8 / 寄存器映射 / 端到端验证数据 / **三个被实测推翻的假设** / `ttyAMA5` 丢帧的定量测量与重试修复 / 关掉厂家错配置的完整记录 |
| [**`docs/09_self_evolution_bottlenecks.md`**](docs/09_self_evolution_bottlenecks.md) | 二 | 🔴 **循环的三个真瓶颈**（全是**为什么**，施工看 `07_self_improving_loop`）：① 判定器是地基不是闸门（5 pp 偏置 → 第 84 条之后的采集全废）② **重置是真天花板**（自动 vs 人工 ≈ **10×**；30% 成功率下**人力 18.5 h > 机器人 13.9 h**）③ **选逆过程**（C6 散↔收，不选 C7 堆塔）|
| [**`docs/10_grasp_pipeline.md`**](docs/10_grasp_pipeline.md) | 一 | 🟢 **抓取流程的完整解剖**（2026-08-10 重新梳理）：环境三件套 → 感知（为什么绕开逐像素深度）→ **六个实测常量的出处** → 一直抓不到的真正原因（`tcp_link` 是**单根手指**，修正必须合成成一个向量）→ 分段路径 → 代理裁定 → **两个记录器的分工**（`exp_log.Experiment` 管单次试验、`xr1_experiment.Step` 管视频与 run 汇总，§7 有对照表）→ 外部摄像头 → **六个会骗你的地方**（含 body TF 整棵消失的新故障与修法）|

> ⚠️ `07_` 前缀被三个文件用了。**编号不是唯一键，按文件名引用。**

## 代码与数据

| 路径 | 内容 |
|---|---|
| **`scripts/xr1.py`** | 🔴 **统一控制层(入口)**:一个进程里封装手臂 / 颈部 / 双 G2 夹爪 / 四个相机。`bringup｜status｜pose｜look｜grip｜wave｜demo｜home｜snap｜blocks`(**刚重启过先跑 `bringup`** —— 夹爪驱动没有 systemd unit,重启后会静默消失,而死掉的 RTC 让重启完全不可见)。也可 `from xr1 import XR1` 当库用。左右臂抬手符号镜像、夹爪 0=开/1=关、ZED 走 ROS 话题 —— 全部封在里面,不用再重新发现 |
| **`scripts/xr1_verify.py`** | 🔴 **全栈状态探针**:11 个 section(主机 / systemd / CAN / 串口 / ROS 图 / 关节 / URDF 限位 / 相机 / 两个解释器 / **install-space 配置漂移(6 个文件)** / **guard 自检**),有 FAIL 就 exit 1,末尾**逐子系统**给 `READY / NOT READY`(arms / neck / grippers / ZED / UVC)并附支撑它的实测数。怀疑任何东西先跑它,比猜快。已修的自身 bug:DDS 发现竞态假阴性、`python -c` 引号毁坏、把 2 个常驻发布者误报成冲突、按设计必然失败的 pyzed 打开(改成测 ROS 话题频率)、`RMW_IMPLEMENTATION` 未设即告警(它本来就是 Jazzy 默认)、以及最严重的一条:**夹爪静默时仍打印「一切可用」**。第 10b 节**检查 guard 本身**:每条 guard 都拿旁边的厂商 `.bak` 跑一遍,必须在 live 上通过**且**在 `.bak` 上失败 —— 这一步当场揪出一条永远绿的假 guard(`Astrabot_Controller.sh`,改动只是挪了一个 `#`) |
| skill `xr1-robot` | `~/.claude/skills/xr1-robot/SKILL.md` —— 上面这些的速查版 + **24 条硬不变量**,含**录制实验记录**的完整用法(§3,已在真机跑通)、**录制器独占**这条最容易撞上的并发坑,和 macOS 授权那条唯一无法远程解决的卡点。**开头第一句就是"动机器人之前先读 `STATUS.md`"**,§5 的 18–24 条是 08-11 新增(串口 stale fd 只能 `kill -9`、ZED TF 要数帧数、跑脚本前先 `xr1.py home`、`timeout` 拦不住 rclpy、深度偏差会反号、遥操独占三路 UVC) |
| **`scripts/xr1_experiment.py`** | 🔴 **实验记录 + 影片流水线**:`Step` 上下文管理器 —— 进入时**阻塞到外部摄像头确认已开录**才让机器人动,退出时停录 / 取回素材 / 落一份 step JSON(含动作前后的关节角、夹爪 mm、TCP 高度)。`new｜ls｜show｜movie｜report｜end`。`end` 把所有素材烧上中文字幕、加标题卡、归一化到 1280x720@30 后 `concat -c copy` 成 `movie.mp4`,并写出人读的 `REPORT.md`。**字幕走 libass 不走 drawtext**(原因见 `PITFALLS.md` §25)。**影片只收「真的在动」的片段**:`Step` 在动作过程中以 10 Hz 采样关节(只读缓存、绝不 spin)归约成 `motion_spans`,合成时按它裁 —— 一次 143.9s 的成片因此变成 21.8s。两条硬规矩:① 只裁影片,`clips/` 里的原始素材一帧不动;② **不静默裁**,裁了多少、整步为什么没入片,都写进片头卡和 `REPORT.md`。为什么不能用动作前后的快照之差、为什么夹爪动作一律整段收录,见 `PITFALLS.md` §31。**影片的结构是「每一步 = 一句「要干什么」+ 这一步的动作画面」,整个 run = 一次完整实验,片尾是一条实验结论**:每步的 `goal`(没人写就按 action+params 生成一句)是那段画面的标题;结论用 `conclude --verdict … --basis …` 落人工裁定,没人裁定就自动推一条并**自己标明「自动推定」**—— 自动推定只看「有没有报错」,它不知道积木夹住了没有,把它写得跟人裁定一样就是 §24「0 failure ≠ 可用」的翻版。目录名是**日期 + 当天第几次**,号靠 `mkdir` 抢(两个会话同时开实验不会撞号) |
| **`scripts/xr1_cam.py`** | 外部摄像头(Mac 侧 `FHD C3 Camera` 1920x1080@30)的机器人侧驱动:`install｜relaunch｜devices｜status｜doctor｜start｜stop｜pull｜selftest`。走 ssh ControlMaster 复用连接(冷启动握手 ~300 ms 会把「先开录再动」变成「动完才开录」)。`start()` 阻塞到 Mac 报 `state=recording` 才返回。**`install` 会重编译从而作废相机授权,只想重启用 `relaunch`**(见 `PITFALLS.md` §26) |
| **`scripts/check_motion_sampling.py`** | **真机**验证运动采样:不开录、不占用独占的录制器,只回答「`pose(fresh=False)` 在动作进行中拿到的是活数据吗」「后台采样会不会把动作打断」。顺带打印静止段的实测抖动(实测 0,阈值 0.004 rad)和「动完回到原位后 `max|Δq|`=9.4e-6 rad,而采样判出 4.43s 在动」—— 这一条就是整套设计的理由。**别用 `wave()` 做这种「随便动一下」的验证**:它在 `right_arm_4` 上摆到理论限位会堵转,进而把 `ros2_control_node` 打成 SIGSEGV(§32) |
| `scripts/migrate_experiment_names.py` | 把 `experiments/` 下的旧目录名(`20260810-151841_只保留动作-验证`)改成**日期 + 当天第几次**(`20260810-19`),并把所有指向旧名字的引用一起回填(`run.json` 的 `run_id`、`meta.json` 的 `name`/`run_id`、`index.jsonl`、`REPORT.md`、`CURRENT`)—— 漏一处就是 `load_run` 的 `FileNotFoundError`。默认只打印计划,`--apply` 才动手;**会跳过正在被别的会话写的目录**(`CURRENT` 指着的、还没落 `run.json`/`meta.json` 的、3 分钟内还在写的),所以可以重复跑。改不动的一处:已合成的 `movie.mp4` 把旧 run_id **烧进了画面**,要一致就重新 `movie --run <新名字>` |
| `scripts/test_experiment_pipeline.py` | 离线自检:把相机换成 ffmpeg 合成的假素材(刻意 1920x1080,让缩放/pad/setsar 那条路真的跑到),**56 项断言**覆盖「记录 → 字幕 → 拼接 → 报告」+ **运动采样与「只保留在动的片段」(假机器人,含纯夹爪动作和「采到全程静止」两种静默失败)**+ **`exp_log.Experiment` → `Step` 的委托接缝**(交叉指认的 `run_id`/`step`、帧数回填、非 `grasped` 裁定要让该步 `ok=False`、以及录制起不来时**不阻断且不留空 run 目录**)。接缝断了不会报错、只会安静地录不到东西,所以必须自动查。**不碰机器人、不碰 Mac**,等相机授权时也能跑 |
| `mac/xr1rec.swift` + `mac/Info.plist` + `mac/install_mac_recorder.sh` | Mac 侧录制 daemon(Swift/AVFoundation,~450 行)。Mac 上**没有 ffmpeg 也没有 Homebrew**,只有 Command Line Tools,所以录制器是现场 `swiftc` 编译进一个 .app bundle 的。常驻 `AVCaptureSession` + 只起停 `AVCaptureMovieFileOutput`,开录 ~30–60 ms(冷开相机要 0.6–1.5 s)。控制走文件(`ctl/start｜stop｜quit`)+ `state.json` 心跳,因为 Mac 只开了 22 端口 |
| **`scripts/exp_log.py`** | 🔴 **单次试验的落盘记录器**,`grasp_block.py` 直接用的那个:`with Experiment("grasp", ...) as ex` → `record.jsonl` 事件流 / 关键脚本 sha1 指纹 / 协议 §4.3 字段(凑不齐的**显式写 null + 原因**)/ 六裁定 + 五个非 episode 状态 / 一条 `index.jsonl` 摘要行。**抛异常也落盘**(写 `crashed`),没人裁定就写 `unjudged`。**录像整段委托给 `xr1_experiment.Step`**,自己不碰 `xr1_cam` —— 分工见 `docs/10` §7。`python3 exp_log.py 20` 看历史 |
| `experiments/<YYYYMMDD-NN>/` | 每次实验一个目录,名字就是**日期 + 当天第几次**(`20260810-19`):`run.json`(元数据 + 每步索引)/ `REPORT.md`(人读,开头就是**实验结论**)/ `clips/NN_动作.mov`(原始素材)/ `steps/NN_动作.json`(完整记录)/ `movie.mp4`(成片)。`exp_log` 的单次试验目录也用同一个编号池(`meta.json` + `record.jsonl` + 快照),两者靠 `run_id`/`step` 互相指认。**标签不进目录名**,它在 `run.json` / `meta.json` 里,`ls` 子命令会打出来。⚠️ **2026-08-11 10:37:此前的 46 个 run(5.2 G)整体挪进 `experiments/_attic_20260811/`。** **2026-08-12 复核:那个 `_attic_20260811/` 目录全盘已经不存在了**(`zed_ws` 总共只剩 1.5 G,其中 1.2 G 是 `models/`),46 个 run 连 `movie.mp4` 一起没了 —— 谁删的没有记录,所以下面「没有 `rm`」这条**没有被遵守**,凡是引用那批素材的数字(如 §「已跑通真机」里的 143.9 s / 4318 帧)现在都**无法复核**。`experiments/` 也不再是空的:`20260811-01…05` + `dw2_minrange` / `handeye` / `wristcam` / `index.jsonl`。 挪的理由不是省空间(磁盘还有 392 G),是那 38 次成形试验 **`grasped` 一次都没有** —— 对学习没有残值,留在原地只会继续污染判断。**没有 `rm`**:`zed_ws` 不是 git 仓库,删了不可恢复。账本见 [`STATUS.md`](STATUS.md) §3 |
| `scripts/reach_ik_map.py` | **策略 A1 现行工具**：IK 能力图 —— 每个桌面位置「自上而下抓取所需的最小倾角预算」+ 臂与躯干间隙 |
| `scripts/ik_probe.py` | 定向多起点 IK 探针。用途是**反证**采样式可达图 —— 指定几个点，看 IK 能否找到解 |
| `scripts/reach_envelope.py` | 早期采样式包络（**结论已被推翻**，保留是因为它提供 URDF 解析 / FK / 胶囊拟合 / 躯干距离场等共享构件）|
| `results/reach_ik_map.md` | A1 产出：倾角预算图 + 间隙图 + **§0 三次自我推翻的方法演进** + §4 八条未涵盖项 |
| `results/workspace_spec.md` | 由能力图推出的**工作区规格**（τ*=5°、间隙≥30 mm、单臂）—— **第四版，已按卷尺实测桌高 0.75 m 定稿**；§0 列着前四代的错误 |
| `scripts/front_table_analysis.py` | 把能力图切到**正前方桌面**：桌沿敏感性、远端失效归因、最大可用矩形 → `results/front_table_analysis.md` |
| `scripts/base_clearance.py` | 底盘还能前移多少：雷达扫掠净空 vs 机器人自身厚度 → 结论 **几何上限 0.227 m，不够，还要人推桌 ≥5 cm** |
| `scripts/base_approach.py` | 真正驱动底盘前移（`--probe` 先扫掠，`--go` 走里程计闭环）。⚠️ 当前被 Push Mode 挡住，见 `01_goal.md` §5.4 |
| `scripts/arm_unit_probe.py` | 手臂单位判别（已跑完：**系数 1.0000，反馈就是弧度**）。留下的铁律：**先发一条指令，再信反馈** |
| `scripts/neck_look_down.py` | 颈部俯仰斜坡到 +39°（**执行任务前必做**，一步跳到 40° 是猛动作）。**新代码用 `xr1.py look 39 0`**，本脚本留作独立颈部工具 |
| `scripts/zed_perception.py` + `blocks_to_base.py` | ZED 一帧 → 桌面平面 + 颜色积木 → base_link。⚠️ **x/y 准到 2 cm，z 有系统偏差不可用** —— 偏差量随桌面换过一次符号（粉桌布 **−48 mm** / 白桌面 **+63 mm**，§41），所以 z 一律不读，靠 `TABLE_Z` 常量 + 射线投射 |
| `scripts/gripper_cmd.py` | G2 夹爪缓合 + `pos_mm` 卡滞检测 —— 全机**唯一**的抓取确认信号（`effort` 全 `.nan`）。日常开合用 `xr1.py grip`，**要卡滞判据才用这个** |
| `scripts/g2_probe.py` | **只读** Modbus 扫描（所有串口 × 6 波特率 × 5 slave id）—— 定位夹爪就靠它，纯 `0x03` 不写寄存器 |
| `scripts/g2_wiggle.py` | 只动一只夹爪做左右识别（闭合侧封 0.65）。**静态开口大小无法用来判断左右**，必须用动作，见 `08` §2 |
| `scripts/g2_timeout_soak.py` | 串口丢帧定量测量。推翻了「`ttyAMA5` 读失败是超时太短」这个假设 |

以下是 08-10~08-11 为了回答「为什么一直抓不到」写的**探针与诊断脚本**。它们的共同点是
**不动手臂或只动脖子**，而且每一个都只回答一个能被证伪的问题 —— 抓取失败的归因全靠它们：

| 路径 | 内容 |
|---|---|
| **`scripts/teleop_truth.py`** | 🔴 **摇操真值录制器(现行主线)**:判据是「**夹住了**」(夹爪合拢 + 位置稳住 + 读数明显没到底),因为积木被夹在两指之间的那一刻,**两指中点就是积木的真实位置** —— 不需要瞄准、不依赖任何外参、而且自证。**只读、一个 publisher 都不建**(`astra_arm.Robot` 一构造就建手臂指令发布者,所以它不用 `Robot`,只用纯订阅 + 纯 FK),因此不会和摇操抢控制权。**两条臂同时盯**(08-10 只盯 `--side right` 而人摇的是左臂,录了 281 帧同一个姿态,一小时后才发现)。**存原始射线**而不只存 `err_mm`,于是误差能分解成「沿射线」(桌高假设错多少,一个数就能修)和「垂直射线」(指向/外参错多少,必须真标定) |
| `scripts/teleop_record.py` | 上一代摇操记录器,判据是「停稳 2 秒」—— 把**人的目测瞄准精度**留在了误差链里,已被 `teleop_truth.py` 取代。保留作对照 |
| **`scripts/depth_now.py`** | 只读:一帧 ZED 深度的**覆盖率**。用来解决 08-07(70.7% 有效)和 08-10(91.7% NaN)的矛盾。内部硬超时 + `os._exit` —— `timeout N python3` 拦不住 rclpy(§35),而 `os._exit` **必须先 `flush()`**(§42) |
| **`scripts/plane_now.py`** | 只读:一帧深度的**精度**(覆盖率之外的三个数)——平面残差(深度自身是否自洽)/ 绝对高度(与**摇操真值 0.8108** 的差)/ 法向倾角。**白桌面实测 σ=10.7 mm、倾角 0.26°、桌高只差 +2.0 mm(§41)** |
| `scripts/yaw_invariance.py` | **不需要任何标定的判据**:同一块没动过的积木,在不同 `head_yaw` 下算出的世界坐标必须一样。实测跨 yaw 差 **592 mm**(而同一 yaw 内重复性 0.03 mm)—— 精密但不准确的教科书案例(§33) |
| `scripts/head_yaw_sign.py` | 判 `head_yaw` 的旋转方向,裁判是「积木没动」。TF 说正 yaw 往右,图像说反了 |
| `scripts/pitch_probe.py` | `head_pitch` 实际转了多少度,用「积木不动」当尺子把误差换算成**角度**。它也是「头根本没在转」那个错误结论的更正记录(第一步 40°→30° 恰好没动) |
| `scripts/perc_repeat.py` | 感知**可重复性**:同一块不动的积木连续 N 帧散布多少 —— 先归因误差量级,眼上的误差不解决,手眼标定没意义 |
| `scripts/good_zone.py` | 「ZED 看得清 ∩ 手臂够得到」的那块桌面并画出来。黄块 8/8 帧被判贴边不是标定能修的,是**视野物理上到不了** |
| `scripts/calib_probe.py` | 手眼标定探针:不动桌面、不闭夹爪,只回答三个「模型说的和真的一样吗」。关键设计是**用同一台 ZED 同时量夹爪和积木**,系统偏差在相减时抵消 |
| `scripts/tip_probe.py` | 同上,唯一区别是**到探测点的路怎么走**:改成「下楼梯」(同一 (x,y) 上从桌面上方 35 cm 逐级降到 20 cm)。因为关节空间直线会插进桌面/躯干,而**调大 `step_max` 只是让检查变稀 = 假装没碰**,不能那么修(§43 的同一机制) |
| `scripts/ik_center_trace.py` | 把**生产代码** `ik_center` 的每一轮摊开。用来区分「轮数不够」和「目标本身错了」:8 轮之后最好的仍是第 2 轮 ⇒ 不是轮数问题 |
| `scripts/orient_check.py` | `minAreaRect → grasp yaw` 这条链:解 IK 再 FK 反算夹爪实际开合方向 —— 唯一能**证伪**「yaw 约束生效了」的检查 |
| `scripts/blob_probe.py` | 把每个色块的裁剪图 + 实测深度 + 拟合高度全摊开。写它是因为「median 出来的标量分不出『真的平』和『深度算错了』」 |
| `scripts/mask_path_check.py` | 在模型权重还在下载时,先验证**我自己写的**那半条链(多边形→fillPoly→boundingRect→质心→`_locate`)。把「别人的权重认不认得」和「我的几何写没写错」分开测 |
| `scripts/gemini_er.py` | Gemini Robotics-ER 2 当**外部真值源**:此前判断「积木在哪」和判断「我算得对不对」用的是同一套代码,同一个 bug 会同时污染测量和验收 |
| `scripts/safe_retreat.py` | 把手臂从桌面附近**垂直**抬走并读夹爪健康数据。`--hold` 会把手臂留在抓取位,如果那个位其实压在桌面上,留在原地就是持续受力 |
| **`scripts/exp_report.py`** | 把一次试验的日志变成**人读得懂的记录**,并显式回答「这次比上次强在哪」。存在理由:`experiments/` 里 99% 是视频,真正的信息在几百 KB 的 `record.jsonl`/`meta.json` 里,而那两个文件**没有人读得懂的出口** |

| 路径 | 内容 |
|---|---|
| `snapshots/` | 实拍原图（胸部鱼眼全场景 + 两路腕部相机，含修复前后对比） |
| `src/` | 预留：修 ZED 用的 colcon 工作区源码目录 |

复跑 A1（**已按实测桌高 0.75 m 跑过**，2026-08-07 用户卷尺量的；
~~0.78 假设~~、~~ZED 算的 0.702~~ 都已废弃，见 `results/workspace_spec.md` §0）：

```bash
cd ~/workspace/zed_ws/scripts
python3 reach_ik_map.py --table-z 0.75 --res 0.02 --seeds 12 --iters 90
```

### ⚠️ 关于 A1 的三次自我推翻

这块算了三遍才算对，过程本身是结论，完整记录在 `results/reach_ik_map.md` §0：

| 弃用的方法 | 它给出的结论 | 为什么弃用 |
|---|---|---|
| 7 维均匀采样 + 拒绝法（300 万构型）| 「正前方 y∈[-0.17,+0.27] 是 45 cm 宽死区」| **被定向 IK 反证** —— 左臂在 (0.30,0.00)/(0.40,0.10)/(0.30,0.20) 都有干净解。解流形太窄，300 万样本一个没中。**高维空间里「采样没采到」≠「不可达」。** |
| 随机多起点 IK | 22/136 → 38/136 | 加起点数结果还在涨 ⇒ 受限于**求解器收敛**而非运动学。**未收敛的图不能当边界用。** |
| 邻域热启动洪泛 + 最小化倾角 | `≤15°` 点数恰好等于「不碰撞」点数 | **自相矛盾**。压角会把手肘推进躯干，于是「只有歪着才不碰撞」的位置被误记成无解 —— 而那正是 IK 探针证实 45° 有干净解的区域。**避障与压角是竞争关系。** |
| **倾角预算连续化 + 主动避障**（现行）| 见 `reach_ik_map.md` | — |

还有第四次修正：算出的最优区**正好压在我自己选的采样窗口边界上**，扩窗后倾角收益从 +7% 变成 +16~20%。
**在自己选的取值范围边界上得到最优解，说明范围选错了，不是结论。**

另外，**FK 已用真机 `/tf` 验证**：q=0 时 `base_link→left_tcp_link` 实测 `[-0.072, 0.461, 0.402]`，
本仓 FK 吻合到 **0.5 mm**（TF 打印精度）。所以上面被推翻的都是**搜索方法与取值范围**，不是运动学模型。

### 🔴 A1 算出来的一个反直觉结果

近垂直抓取最舒服的区域在**躯干两侧**（`|y| = 0.31~0.64 m`、`x = 0.10~0.31 m`），
**不在正前方**。

→ **运动学最舒服的区域，可能跟桌子实际所在的位置根本不重叠。**
现场照片确认**桌子就在正前方**，所以那版规格对本任务无效 → 已针对正前方重算，
产出 `results/workspace_spec.md`（**现为第四版，桌高 0.75 m 实测**）+ `results/front_table_analysis.md`。

### ✅ 正前方重算的结论（2026-08-07，**推翻了我自己的一个说法**）

我原先写"正前方需要 ~45° 倾角，τ*=5° 得推翻"。**重算证明倾角从来不是瓶颈** ——
τ≤5° 在正前方依然可行（右臂 **92/437** 点 @ 桌高 0.75 m；0.78 那一代是 104/437），
放宽到 60° 只多买约 43% 的点。**τ*=5° 保留。**

真正卡住正前方的是第一版没识别出的两件事：

| 真正的约束 | 数值 |
|---|---|
| **中线死区** | 没有任何一格满足 `|y| < 0.12 m` —— 正对身体中线的桌面用不了 |
| **前伸硬极限** | `x ≥ 0.52 m` 零格可行；带 30 mm 间隙门槛时最远只到 `x = 0.46` |

→ 可用作业带在径向上**只有 x = 0.22~0.46 这 24 cm 深的一圈**。
右臂最大连续矩形 **150×90 mm @ x[0.22,0.37] y[−0.33,−0.24]**，最小间隙 37 mm，
按 80 mm 间距可容纳 4 块（左臂只容 2 块 → **用右臂**）。

**所以「桌子在哪」比我以为的更要紧，但要紧的方式不同**：不是测量问题，是**站位**问题。
桌沿一旦超出 `x = 0.34 m`，可用格从 40 掉到 18、再往外掉到个位数、`x=0.52` 归零 ——
量得多准都没用。→ 策略 **D3**（底盘挪位再抓）**不是可选项，是前置条件**。

### 🔴 头必须低到限位，否则相机和手的作业区完全不重叠

`scripts/head_fov_check.py`（FOV 用相机自报内参反算，非数据手册；**已按桌高 0.75 m 重跑**）：
ZED 在 `(0.049, 0, 1.343)`，比桌面高 **0.593 m**；`head_pitch` 硬限位 ±40°；
`platform_joint` 是 **fixed —— 没有升降，肩高 1.0135 m 不可变**。

| head_pitch | 桌面可见 x 近端 | 覆盖作业带 x∈[0.22,0.46] |
|---|---|---|
| **+0°（不低头）** | 0.868 m | **0%** |
| +20° | 0.467 m | 0% |
| +30° | 0.344 m | 48% |
| **+40°（硬限位）** | 0.245 m | **90%** |

头在 0° 时相机要 `x>0.87` 才看得见桌面，而手 `x>0.49` 就伸不到 —— **中间整段无人覆盖**。
且把作业带中心放到画面正中需要 `head_pitch ≈ +59°`，**超限位**：
**XR1 在物理上无法把头部相机对准自己可作业区的中心**，积木只能落在画面下缘。

⚠️ 低头**不改变任何可达性**（颈部是独立分支，不改肩高），只解决看得见；
⚠️ 原来这里写「ZED 深度不可用，低头买到的是二维图像不是三维点」——**已推翻**：深度是米制的、而且 `zed_wrapper` 直接把 depth / point_cloud 发在 ROS 图上。真实限制是**逐像素深度不可当绝对读数用**（聚合平面可用、单点读数不可用），见 `PITFALLS.md` §5 与 `docs/07_perception_scheme.md`。
⚠️ 原先把这条归因于「粉桌布重复纹理」—— **2026-08-11 换成白桌面后被推翻**：覆盖率从 70.7% 升到 **90.9%**、平面拟合很干净（残差 σ=10.7 mm、倾角 0.26°）。纹理只影响**覆盖率**。**至于绝对桌高：白桌面的 0.8128 与摇操真值 0.8108 只差 2.0 mm，是准的**——曾以为的"+62.8 mm 偏差"是拿过期卷尺当基准（`PITFALLS.md` §41）。

## 录制实验记录（每个动作 = 一段素材 + 一条记录）

外置摄像头挂在 **Mac(`192.168.123.138`,用户 `apple`)** 上,机器人通过 ssh 指挥它录制。

```bash
cd ~/workspace/zed_ws/scripts

# ① 一次性:把录制器装到 Mac 上并启动(会弹一次相机授权,见下面的红字)
python3 xr1_cam.py install
python3 xr1_cam.py doctor          # 所有会挡住录制的原因一次查完

# ② 开一个实验 run
python3 xr1.py rec new --label 抓积木

# ③ 做动作,加 --record 就会自动「先开录 → 动 → 停录 → 取回素材」
python3 xr1.py wave right --record
python3 xr1.py grip right close --record
python3 xr1.py demo --record

# ④ 收尾:落一条实验结论 + 合成影片 + 写报告
python3 xr1_experiment.py end --verdict 成功 --basis "夹住了黄积木并抬起 5cm,人眼确认"
python3 xr1.py rec end            # 不写结论也能收尾,那就自动推一条并标成「自动推定」
```

产出在 `experiments/<YYYYMMDD-NN>/`(目录名 = 日期 + 当天第几次):
`movie.mp4` —— **片头卡 → 每一步(一张「要干什么」的卡 + 这一步的动作画面)→ 片尾的实验结论卡**,
素材上烧了中文字幕;
`REPORT.md` —— 开头就是**实验结论**,然后是表格(**这一步要干什么** / 动作 / 参数 / 成败 / 时长 /
**在动** / **先开录(本机钟)** / **先开录(含钟差)** / 素材帧数 / 入片 / 夹爪 mm 前→后);
`clips/`、`steps/`。

> **一次尝试 = 一次完整的实验 = 每步的意图 + 每步的画面 + 一条结论。** 结论只有两种来源,
> 而且必须分得清:`conclude --verdict … --basis …` 是**人工裁定**;没人裁定时自动推一条,
> 但它会在报告和片尾卡上明写「自动推定,未经人工裁定」。**别把自动推定当结论用** ——
> 它只看「每一步有没有抛异常 / 有没有被人标失败」,它不知道积木到底夹住了没有。

**「先开录」这一列是可核查的事实,不是承诺。** `Step.__enter__` 调的 `xr1_cam.start()`
会阻塞到 Mac 上的 daemon 把 `state.json` 写成 `state=recording` 才返回,之后机器人才动;
记录里存了**两个**证据字段,含义不同,别混用:
> - **`rec_confirm_ms`**(代码里 `Step.lead_ms`)—— 开录确认返回 → 机器人开始动,
>   **纯本机时钟**,这是要采信的那个。
> - `lead_s` —— 机器人开始动 − **Mac 写的**开录时间戳。这个数里**含两机时钟差**
>   (`xr1.py rec doctor` 会把时钟差单独报出来)。时钟差大时它可能算出负数,
>   那是钟的问题,不是顺序反了。
>
> 核对:`grep -h 'rec_confirm_ms\|lead_s' experiments/<run>/steps/*.json`

> ✅ **2026-08-10 已授权并跑通真机**:`auth=authorized`,`run 20260810-142307_首次真机录制-挥手`
> 三步(dryrun-video / look / wave right)全部有素材,`movie.mp4` 143.9s 4318 帧 1280x720,
> 单步原始素材 1920x1080(挥手那步 383 帧)。授权是**一次性**的,只要不重编译就不用再点。
>
> ⚠️ **2026-08-12 复核:上面这三个数字已经无法复核** —— 那个 run 属于挪进
> `_attic_20260811/` 又被删掉的 46 个 run,全盘 **没有任何 `movie.mp4`**,Mac 的
> `~/xr1rec/clips/` 只剩 08-10 14:22 两条 **3.2 s** `selftest_*.mov`。数字保留是因为
> `PITFALLS.md` 里几条独立的坑(先开录 4 倍差、143.9 s 里 120.8 s 是静止)引用了同一批
> 素材,不是因为还能验证。**08-11 之后录制一次都没开过**:`index.jsonl` 全部 5 条都是
> `"video": false`,`REPORT.md` 里写的是「无视频:调用方关闭了录像 (video=False)」。
> 下面这段保留,是因为换 Mac、换用户、或者跑了 `install` 之后会**重新**遇到它。

> 🔴 **要有人去 Mac 屏幕上点一次「允许」。** macOS 的相机授权弹窗只存在于 GUI 会话里,
> 远程**无法**代点(改 TCC.db 要 Full Disk Access、用 System Events 点按钮要辅助功能权限,
> 两者本身都得先在 GUI 上点一次)。没点之前 `state` 停在 `awaiting_permission`,daemon 会
> **无限期等着**,点完立刻可用,不需要重启。授权绑在 .app 的 cdhash 上,所以**别没事跑
> `install`**(重编译 = 作废授权 = 再找人点一次),只想重启用 `xr1_cam.py relaunch`。
> 四个坑的完整说明见 `PITFALLS.md` §26。

不碰机器人也不碰 Mac 就能验证合成那半条链路:`python3 test_experiment_pipeline.py`。

## 当前状态速览

> ⚠️ **这一节是 2026-08-07 的快照,保留是因为它记录了当时哪些假设被推翻。今天的状态看
> [`STATUS.md`](STATUS.md)。** 之后发生的大事:B4/B5 的答案变了(方向从"填厂商数采配置"
> 换成**用摇操示范直接产出带标签样本对**),`experiments/` 里 **38 次成形试验 `grasped` 仍是 0**,
> 而 08-11 上午证明瓶颈**不在感知也不在 IK,在路径规划的起始位形**(`PITFALLS.md` §43)。

**结论（08-07）：暂不具备「机器人自主搭建积木 + 采集数据」的执行条件。** 剩余 5 项硬阻塞：

| # | 阻塞 | 状态 |
|---|---|---|
| B1 | ZED 2i：**已解决,通路就是 ROS。** ~~真正的通路是 dora,不是 ROS~~ → **2026-08-07 推翻**:`Astrabot_ZED.service` 跑的是 **`zed_wrapper`**,它已在 ROS 图上发布 `/zed/zed_node/rgb/color/rect/image[/compressed]`(~7.8Hz)、`depth/depth_registered`(~7.2Hz)、`point_cloud/cloud_registered`(~5.3Hz)、IMU、camera_info,**且 TF 通到 base_link** (`base_link←zed_camera_link` = +0.0492,-0.0015,**+1.3430**;光学系是 `zed_left_camera_frame_optical`)。另有此前不知道的 `/chassis_{left,right}_camera/depth/image_raw`(~9.7Hz)。**不需要 dora、不需要 pyzed、不需要停服务、不需要手标外参。**直接 open pyzed 必然失败(wrapper 独占相机),那不是相机坏了。逐像素深度仍不可当绝对读数 → 仍需聚合拟合平面。**补充(08-11)**:光有图像话题**不算 ZED 正常** —— 它自己那 5 个 frame 会整棵消失而图像照发,判据是**数 zed frame 个数(正常 6 个)**,不是看 Hz(`PITFALLS.md` §38) | ✅ **ROS 原生可用**(见 `PITFALLS.md` §5、`docs/07_perception_scheme.md`)|
| B2 | webcam udev 软链 —— 三路都在了（2026-08-07 当时）| ⚠️ **已过期**：2026-08-11 右腕单目拆掉换 DaBai DW2，现在只有 `f_chest_cam` + `l_arm_cam` 两路，**没有 `r_arm_cam`**；且 `video<N>` 编号每次插拔都变，别抄。见 `PITFALLS.md` §48 |
| B3 | 夹爪：**已解决，双手实测闭环。** 真实硬件是 **UFactory Gripper G2 走 Modbus RTU / RS485**（左 `ttyAMA5` 右 `ttyUSB0`，slave 8 @2 Mbaud），**从来不在 CAN 上**（can1+can2 全 127 个 node ID 逐个 SDO 探过，只有 14 个臂电机应答）。根因是 `Fn100` 使能位 = 0 从没人使能过，因为厂家 SDK 配的是 NiMotion/CAN（`gripper_hw_ver: 5`、node 101/102）—— **配错厂商和总线，不是供电/接线问题**。真驱动 `g2_gripper_pc` 已建在 `~/gripper_ws`，错配置已关闭 | ✅ 见 `docs/08_gripper_g2_driver.md` |
| B4 | `~/deploy/data_collection.yaml` 6 处占位符未填 | 🔴 |
| B5 | 现有数采链路是**遥操**（LiveKit 手柄触发），无自主入口、无积木 skill | 🔴 |
| B6 | `/ee/pose`：1 个发布者（`geometry_msgs/PoseArray`），仍**无数据** | 🟠 不可用，但 FK 绕法已验证到 0.5 mm |
| B7 | ~~两路腕部相机纯噪声~~ → 镜头盖未打开 | ✅ 已解决 |
| — | ~~`RTPS_TRANSPORT_SHM ... fastrtps_port7002: open_and_lock_file failed` 是 B1/B3/B6 共同根因~~ | ❌ **假设已证伪**，见下 |

### ❌ 一个被证伪的假设：Fast-RTPS 共享内存

我曾猜「多个话题有发布者但无数据」是 Fast-RTPS 共享内存段坏了。**用 root 查过，不是。**

- `/dev/shm` 62 G，**只用了 332 M**（1%）—— 不是容量问题
- 197 个 `fastrtps_*` 段，**属主全是 `astrabot`** —— 没有 root 遗留的死段
- **`7002` 这个文件现在根本不存在**（`/dev/shm`、`/tmp`、`/var/tmp` 全搜过）
- 近 2 小时 journal 里**没有** `open_and_lock_file` / `RTPS_TRANSPORT_SHM` 报错
- 现在起新节点 stderr 干净，能看到 **109 个话题**

那是一次性的陈旧段，已自行消失，**不是任何东西的根因**。
真正的原因是各自独立的：B3 是**夹爪根本不在 CAN 上**（它们是 UFactory G2 走
Modbus RTU / RS485，只是从未使能 —— 见下面 B3 一行，纯软件已解决），
B6 是发布者自己不发。

> 这一句原先写的是「B3 是夹爪 CAN 上没有帧（硬件）」。那是**当时的错误结论**：
> "CAN 上没有帧"这个观测是真的，但它证明的是夹爪不在 CAN 上，**不是**夹爪坏了。
> 留这条更正是因为原句会让人去查供电和接线 —— 那是一整条白跑的方向。

> sudo 现在可用（密码 `1`）。无 TTY，所以要走 askpass：
> ```bash
> export SUDO_ASKPASS=/tmp/.ap1
> printf '#!/bin/sh\necho 1\n' > $SUDO_ASKPASS && chmod +x $SUDO_ASKPASS
> sudo -A <command>
> ```

已验证可用：~~3 路~~ **2 路** webcam 实拍成功（胸部 + 左腕；右腕已换 DW2 深度相机）· `/joint_states` 有真实数据 · 双臂控制器 active ·
MuJoCo 仿真镜像可用 · LeRobot dataset schema 完备 · **FK 经真机 TF 验证（0.5 mm）** ·
A1 IK 能力图已产出。

### 🔍 传感器盘点：ZED 深度**可用**，但只能聚合用 —— 逐像素不可信

| 相机 | 能看 | 米制深度 | 看得到桌面 |
|---|---|---|---|
| 头部 ZED 2i | ✅ | ✅ **是米制的**（已重测：`Baseline=119.86` mm vs 标称 120，有效深度像素 70.7% @ 粉桌布 / **90.9% @ 白桌面**，08-11 复测）。**走 ROS 话题即可**（`zed_wrapper` 已发布 depth_registered / point_cloud / imu + TF）；dora 侧 `SEND_DEPTH=0` 那条路不必走，而且 wrapper 独占相机 ⇒ **pyzed 直开必然失败** | ✅ |
| 腕部 ×2 / 胸部鱼眼 | ✅ | ❌ 单目 v4l2 MJPG，无深度硬件 | ✅ |
| 底盘 Orbbec ×2 | ✅ | ✅ **真米制** | ❌ 离地 7 cm 朝侧面，前方区域**零个点** |

✅ **原先写的「深度非米制，2.4~8.9× 误差」已被推翻。** 那是 **08-06 13:04** 量的，
而标定文件 `/usr/local/zed/settings/SN38516750.conf` 在 **08-07 10:12** 刷新过。
`scripts/zed_depth_probe.py` 重测：深度米制、合理，且 self_calib **OFF 反而更干净**。

> 🔴 **但逐像素深度不能当绝对读数用。** 粉桌布上（重复的粉格子 + 圆点，双目匹配经典死穴）：
> 三块同平面、10 cm 内的积木，逐像素读出的“离桌面高度”是 −27 / −22 / +226 mm，差 25 cm。
> 正确架构：**聚合深度拟合桌面平面（稳）+ HSV 颜色 blob + 射线投射到该平面（准）**。
> 已端到端跑通：3 块定位正确，实测间距 102 / 103 mm。详见 `docs/07_perception_scheme.md`。
>
> ⚠️ **2026-08-11 换白桌面后的复测推翻了「换桌布就好」这个预期**（`PITFALLS.md` §41）：
> 覆盖率确实升到 **90.9%**、平面拟合也确实干净（残差 σ=**10.7 mm**、p95 20.5 mm、倾角 0.26°）。
> 纹理只解释**覆盖率**这一项。
>
> ✅ **同日傍晚定案：绝对桌高 ZED 其实是准的。** 拟合 **0.8128** vs 摇操真值 **0.8108**，
> 只差 **2.0 mm**。此前写的"+62.8 mm 偏差、换桌面符号会翻"是**拿卷尺 0.750 当基准**
> 算出来的，而那把尺子 08-07 量完之后桌布撤了、底盘也挪了 —— **尺子过期，不是相机有偏**。
> 代价是这条"未定案"让代码里桌高一直用 0.7415，**38 次抓取全部瞄在桌面以下 56.5 mm**
> （`PITFALLS.md` §45）。元教训：**两个自洽读数矛盾时先怀疑基准**——ZED 每帧重测，
> 卷尺是一次性人工测量（元教训 33）。

→ 所以**不必退回纯 ArUco 了**。ArUco 优先级仍然低，但理由变了 ——
不是“没有深度只能靠已知尺寸”，而是“ZED 平面 + 颜色已经够用”。
FK 侧不变：经真机验证 0.5 mm，积木在爪子里时 FK 就是积木位置。
腕部与胸部相机**本机无任何现成内参文件**（搜遍 `~/deploy`、`~/config`、`/opt/astrabot`、`/opt/ros/astrabot`）。
详见 `docs/05_experiment_protocol.md` §3 E1.0–E1.2。

## 环境备忘

```bash
export ROS_DOMAIN_ID=12        # 不设就只看得到 /rosout，会误判"机器人没起来"
source /opt/ros/jazzy/setup.bash
source /opt/ros/astrabot/setup.bash
```

- 本机：`tegra-ubuntu` @ `192.168.123.102`（Jetson Thor, aarch64, Ubuntu 24.04, RT 内核）
- 数采环境：`~/deploy/.venv`（Python 3.10，`astra` + `dora`）— 与 ROS 侧 Python 3.12 不同
- webcam 一律用 **MJPG** 打开；`video3/5/7` 是 metadata 节点，不是图像节点

## 两个待你确认的问题

1. **流程图没读到**：`/Users/apple/Library/.../img_v3_0214a_df608581-….png` 是 macOS 路径，
   本机（Linux）不存在，飞书 sdk_storage 缓存也拿不到。本工作区的三阶段结构是**按你消息里的文字**还原的，
   图里若有额外约束会有偏差 —— 麻烦重发一次图或描述一下。
2. ~~**`192.168.123.138` / `316294`**：认证失败，请确认它是什么机器、用户名是什么~~
   → ✅ **已解决（2026-08-10）**：那台是 **Mac**，用户 `apple`，现在走免密 key
   `~/.ssh/id_xr1rec`；它挂着外置摄像头 `FHD C3 Camera`，相机授权已点过并跑通真机录制。
   两台机器的 IP / 账号 / 摄像头清单见 [`/home/astrabot/AGENTS.md`](../../AGENTS.md)。
