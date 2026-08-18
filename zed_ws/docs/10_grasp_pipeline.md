# 10 抓取流程（重新梳理，2026-08-10）

这份文档是 `scripts/grasp_block.py` 一次运行的完整解剖：**每一步在干什么、每个数字是
怎么量出来的、以及哪一步会骗你**。写它的原因是前面几次失败都不是"算法不行"，而是
某个常量取了"看起来合理"的值 —— 那类错误只有把出处写下来才不会重犯。

配套文件：

| 文件 | 角色 |
|---|---|
| `scripts/grasp_block.py` | 主流程（感知 → 筛选 → IK → 分段路径 → 执行 → 裁定） |
| `scripts/xr1.py` | 机器人门面：`Robot()`/关节/夹爪/颈部/相机 |
| `scripts/ik_probe.py` | IK + FK + 胶囊碰撞检查 |
| `scripts/exp_log.py` | 每次试验落盘（§7） |
| `scripts/xr1_cam.py` | 外部摄像头录制（Mac 侧驱动，§8） |

---

## 0. 跑之前：环境三件套

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=/home/astrabot/tools:/home/astrabot/workspace/zed_ws/scripts:$PYTHONPATH
```

三个都不能省：

- `ROS_DOMAIN_ID=12` —— 默认 0 的 shell 看到的是一个几乎空的图，会让你得出完全错误的结论。
- `astra_arm` 在 `/home/astrabot/tools/`，不在默认 PYTHONPATH 上。
- 不 source ROS 就没有 `rclpy`，`XR1()` 在构造函数里就炸。

常用调用：

```bash
python3 grasp_block.py --dry-run            # 只规划不动
python3 grasp_block.py --hold               # 下到抓取位停住拍照（人工核对最有用的一档）
python3 grasp_block.py --no-video           # 不录外部视频
python3 exp_log.py 20                       # 看最近 20 次试验
```

---

## 1. 感知：为什么绕开逐像素深度

`perceive()` 的输入是 ZED 的**彩色图 + camera_info + TF**，深度只用来投否决票。

```
/zed/zed_node/rgb/color/rect/image/compressed   → HSV 阈值分割 → 色块质心 (u,v)
/zed/zed_node/rgb/color/rect/camera_info        → fx,fy,cx,cy 反投影成视线
TF base_link ← zed_camera_link                  → 视线变换到 base_link
                                                 ↓
                      视线 与 z = 桌面高度 的平面求交 → 积木 (x, y)
```

**为什么不用逐像素深度**：2026-08-10 实测，这台机器 ZED 深度图全图 **91.7% 是 NaN**，
有效深度只到 0.69 m，而桌面在 ~1 m 外 —— 积木上的有效深度率是 **0%**。还有 ~5 cm 的
系统性 z 偏差。所以深度只能当"有值时的否决票"（`rise < MIN_RISE` 判为桌布印花），
**不能当准入条件**，否则永远抓不到东西。这条已经在 `why_reject()` 里写死了。

**平面求交的代价**：x/y 精度直接取决于假定的平面高度。桌面高度错 1 cm，在这个视角下
x 会错约 1 cm。所以 `TABLE_Z` 必须是量出来的（§3）。

### 1.1 预筛（`why_reject`）

| 条件 | 值 | 为什么 |
|---|---|---|
| `clipped` | 外接框离画面边缘 ≥ `BORDER_PAD`=6 px | **被裁掉的斑块质心是偏的**，只算到了留在画面里的那部分；真实范围还延伸到视野外，无从判断能不能抓 |
| `BAND_X` | 0.18 ~ 0.56 m | x > 0.6 的色块是背景/另一张桌子，不是作业区 |
| `BAND_Y_MIN` | \|y\| ≥ 0.04 m | 中线附近解流形极窄（不是必然无解，只是先跳过） |
| `rise` | ≥ `MIN_RISE`，**仅当有深度** | 深度不可用是常态，见上 |

**贴边这条必须排在 x/y 之前**：贴边斑块的 x/y 本身不可信，拿它去判"在不在带内"
就是用坏数据做决策，而**恰好落在带内比落在带外更危险**。
2026-08-10 实测：白桌全空时，画面下沿一个被裁掉一半的红色物体被反投影成
`(0.29, −0.09)` —— 正好在带内，于是整条流程一路算到 IK 才停下，白烧 100 s，
而那个位置根本没有积木。

预筛只排除明显无望的点。真正的可达性判定交给带碰撞检查的 IK —— 固定工作带对窄解
流形偏悲观，会误杀边缘上真正可解的点。

## 2. 两个 Python 解释器：不要混用

| 解释器 | 版本 | 有什么 | 谁用 |
|---|---|---|---|
| `/usr/bin/python3` | 3.12 | rclpy, cv2, tensorrt 10.13.3.9 | ROS 侧全部脚本 |
| `/home/astrabot/deploy/.venv/bin/python` | 3.10 | pyzed | 只有直开 ZED 才用，抓取链不用 |

两边的 site-packages 不能互相 import，混用会炸在 C 扩展的 ABI 上。所以

---

## 3. 那几个不许乱改的常量

全在 `grasp_block.py` 开头。每一个都是量出来或从 URDF 推出来的：

| 常量 | 值 | 出处 |
|---|---|---|
| `GRASP_TIP_Z` | **0.8238 m** | **摇操真值,2026-08-11**：人把右臂摇到一个能夹住黄积木的位姿,两指中点 FK = (+0.4749, −0.3591, **+0.8238**),抖动 0.00 mrad。链里没有相机、没有外参、没有人的瞄准 ⇒ 自证。`verify_fk.py` 用 rsp/KDL 独立复算差 0.00 mm |
| `TABLE_TOP` | **0.8108 m** | `GRASP_TIP_Z − 13 mm`(指尖卡在 4 cm 积木下半部)。真值带 [0.794, 0.814] |
| `TABLE_Z` | **= `TABLE_TOP`** | 感知与运动**必须同一个桌高**。旧代码里两者差 8.5 mm,已消除 |
| `TCP_TO_TIP` | 0.0485 m | URDF：`right_gripper_base_link` 网格沿 +z 伸到 0.1105，`right_tcp_link` 在 0.062 |
| `TCP_TO_CENTER` | (−0.0225, 0, 0) | URDF：`tcp_link` 在 gripper_base 的 x=+0.0313，另一指 `gripper_link` 在 x=−0.0137，中点 +0.0088 |
| `TIP_Z_FLOOR` | **0.8188 m** | 运行时地板。判据读**指尖中点**(`XR1.tip_center`),**不是** `tcp_z` —— 见 §3.2 |

> **已作废,不要再用**：卷尺 0.750(08-07,量的时候桌布还在、底盘位置也不同,低出真值带 44~64 mm)、
> 触桌反推 0.7415 / `TCP_Z_CONTACT=0.790`(把「关节偏差出现」读成「接触发生」,低 70 mm)、
> ZED 早期拟合 0.702。**ZED 08-11 的 0.8128 只差 2.0 mm,是可信的那一个。**

### 3.1 38 次 0 成功的真正原因(2026-08-11 定案)

**桌高错了 69 mm,每一次抓取都在瞄桌面以下 56.5 mm。** 代码命令指尖去
`GRASP_TIP_Z = 0.7545`,而桌面在 **0.8108**。所谓"IK 有解但路径规划全拒"根本不是
规划器保守 —— **目标点在桌板内部**,规划器判碰撞是对的,机器人一动不动是对的。

这一条盖过下面所有其它因素。下面两项(`tcp_link` 是单指、`head_yaw` 转角误差)都是
真的,但在目标点埋在桌子里的前提下,它们修与不修都抓不到。

> 元教训:38 次 0 成功、而**每次失败原因都不一样**,这个模式本身就在说缺的是
> **一次可信的标定**,不是**更多的尝试**。真正锚死它的是一个摇操位姿,不是卷尺。

#### 3.1.1 次要项一：`tcp_link` 是单根手指

`right_tcp_link` 是**单根手指**的坐标系，不是两指中心。之前 IK 把**一根手指**对准了
积木中心 —— 于是积木总是被推开或从两指之间漏掉。

修正**必须合成成一个向量**：

```python
TIP_CENTER = TCP_TO_CENTER + [0, 0, TCP_TO_TIP]     # 横向 −22.5mm + 沿接近轴 +48.5mm
```

**不能**分成"横向修正"和"竖直修正"两步算。两个 link 的 rpy 都是 0，所以这是纯平移，
合成后横向和竖直修正由**同一个旋转矩阵**算出来，自洽。分开算会在夹爪歪斜时互相打架
—— 实测 45° 歪斜下分开算把指尖抬到 0.784，比积木顶面还高，必然空夹。

**但这不是最大的那一项。** 2026-08-10 测出来：头一转 40°，同一块**没动过**的积木就被
算到 **59cm** 之外（`experiments/20260810-22` 591.9mm / `-23` 584.2mm，两次独立复现，
一致到 8mm）。那是这里说的 22.5/48.5mm 的**一个数量级以上**。所以修 `TIP_CENTER` 是
对的，但只在 `head_yaw = 0` 时才有意义 —— 转过头的那一帧，指尖修正多少毫米都无关紧要。
详见 `PITFALLS.md §33`（含量级推算和还没跑的判别实验）。

### 3.2 IK 求解的是指尖中点，不是 tcp

`ik_center()` 做的是不动点迭代：先按积木中心求一次解，用该解的旋转把 `TIP_CENTER`
转回去得到 tcp 目标，再解一次（默认 2 轮）。

```python
tcp_target = block_center − R_tcp(q) @ TIP_CENTER
```

因为偏置在 tcp 系里，而 tcp 的朝向本身是 IK 的输出，所以只能迭代。收敛很快（2 轮足够）。

解完一定用 FK 反算指尖中点核对 —— **这是唯一能证伪"修正算对了"的检查**：

```
tcp 去 (+0.290,-0.091,0.803) -> 指尖中点 (+0.290,-0.091,0.755),
对目标残差 0.3mm; 桌面 0.742, 指尖高出 13mm
```

---

## 4. 运动：为什么要拆段

`astra_arm.Robot.move()` 拒绝任何单关节 |Δq| > 1.8 rad 的指令。而从当前位形到积木
上方经常需要 2.2 rad 以上。`plan_path()` 把直线插值成 ≤1.2 rad 的段，**并对每个中间
位形做胶囊碰撞检查**（终点已由 IK 的 collide 验证过）。

顺序：

```
张开夹爪 → path_pre（到积木上方 GRASP_TIP_Z + --clear）
         → path_gr（下降到 GRASP_TIP_Z）
         → [--hold: 停住拍照，结束]
         → 闭合夹爪 → 沿 path_gr 倒序提起
```

提起走原路倒序，不重新规划：下降路径刚刚验证过无碰撞，反着走一定也无碰撞。

### 4.1 直达不行就经中转位姿（`plan_to`）

`plan_path` 走的是**关节空间直线**，能不能过完全取决于**起点** —— 而起点是整个任务里
最不受控的量（上次试验停在哪、有没有人手动动过、有没有别的程序正在动它）。

2026-08-10 14:23 实测：同一个目标、同一个 IK 解（误差 0.0 mm），从零位出发 2 段全
干净；从挥手中途的位形出发**必碰**，整次试验白跑。所以不能让起点决定成败。

`plan_to()` 先试直达，失败就走 `q_now → STAGING_Q → q_goal` 两条腿，
`STAGING_Q` 是**手臂垂下的零位**（离桌面最远、不贴躯干，且 URDF 零位不随标定漂移，
比自己编一个"安全姿态"可靠）。**两条腿各自逐点碰撞检查**，中转点不是免检通道；
两条都干净才算成功，连"退回中转位姿"都不安全时直接放弃，不硬闯。

实测 40 个随机起始位姿：直达就行 27 个，**经中转救回 5 个**，两种都不行 8 个
（那 8 个要真正的运动规划器，超出本脚本范围）。零位起点行为不变（仍是 2 段、不绕）。
绕了会在 `record.jsonl` 里留一条 `plan_via_staging`。

> 🔴 **2026-08-11：那 8/40 不是理论问题，它就是现在的头号卡点。**(`PITFALLS.md` §43)
> `20260811-03` 是今早修完四个故障后的第一次真尝试:感知干净(黄块 x=+0.476,
> y=−0.155)、pre 和 grasp **两个 IK 都有解**,但 `plan_to` **直达和经中转都被拒**,
> 裁定 `skipped` —— 机器人**正确地**一动没动。
>
> 真因不在规划器,在**起点**:上一轮摇操把右手停在 tcp **z=0.547**,比 0.75 的桌面
> 低 20 cm。从桌面**以下**去桌面**上方**,关节空间直线中途必然扫过桌面,于是每一段
> 都判碰撞;而 `STAGING_Q`(手臂垂下的零位)同样在桌面以下,所以中转也救不回来。
>
> **修法(必须):自主脚本开跑前先 `python3 xr1.py home`。** 不是"建议",因为
> 「可达性」是**当前位形的性质**而不是目标点的性质 —— 不归零,两次试验的结果就
> 没法比(这也正是下面 §9 #7「可达性判定不可复现」的机制)。
>
> **别用调大 `step_max` 来"修"它**:那只是让碰撞检查变稀 = 假装没碰
> (`tip_probe.py` 的注释里记着同一条教训)。
>
> 另外 `plan_failed` 现在**落盘诊断**:`q_pre`/`q_grasp` 两个解 + 归因(卡在第几个
> 路点、撞的是桌面还是躯干、深多少毫米)。之前算了 6.5 分钟的 IK **算完就扔**,
> 于是只知道"规划失败",不知道失败在哪 —— 那等于这次试验白跑。

---

## 5. 已知会骗你的地方

按坑人程度排序。

### 5.1 头必须俯到 +40°，否则相机和手臂的可达区完全不重叠

`head_pitch` 的上限就是 +40°，而"把工作区摆到画面中心"需要的角度超过这个上限。
所以 `look_at(pitch_deg=40)` 拿到的是**边缘视角**，积木在画面下缘。这不是 bug，
是这台机器的硬约束。

### 5.2 `/joint_states` 会假死；发布者是 `astrabot_mrt`

不是 `ros2_control_node`。进程活着也可能停发。没有它 `astra_arm.Robot()` **在构造
函数里**就抛 `MotionRefused`，所以"先把手臂抬起来"这种应急操作是做不到的 —— 只能先
重启 `Astrabot_Controller.service`（需要 `SUDO_ASKPASS`，没有 TTY）。

### 5.3 body TF 会整棵消失（2026-08-10 新发现）

症状：`perceive()` 报 `拿不到 base_link <- zed_camera_link 的 TF`，
`tf2` 说 `two or more unconnected trees`。

原因：`robot_state_publisher` 进程还活着（`ps` 能看到），但它的 DDS participant 没
进图 —— `ros2 node list` 里没有它，`/tf` 上抓不到任何机体 frame，只剩
`odom → base_link` 和 ZED 自己的内部链。伴随
`RTPS_TRANSPORT_SHM Failed init_port fastrtps_port7014: open_and_lock_file failed`。

**先别急着重启控制器**（会让手臂短暂失力）。补一个 rsp 就行，用同一份 URDF 参数，
可随时 kill 掉：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
nohup /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher \
  --ros-args -r __node:=xr1_rsp_helper \
  --params-file /tmp/launch_params_aai4mw6j >/tmp/rsp_helper.log 2>&1 &
```

参数文件名每次 launch 都会变，用
`ps aux | grep robot_state_publisher` 找当前那个（**不是** `-r __ns:=/zed` 那个，
那是 ZED 自己的）。

**验证要看 TF，不要看 `ros2 node list`。** 这台机器上 helper rsp 起来之后，
`ros2 node list` 里**照样看不到** `xr1_rsp_helper`，但 `/tf` 上机体链是好的 ——
节点发现和话题发现在这里不同步，拿 node list 判断会得出"还是坏的"这个错误结论：

```bash
ros2 run tf2_ros tf2_echo base_link zed_camera_link      # 唯一可信的检查
# 头部 ~40° 时应约为 Translation [0.118, -0.001, 1.255]
```

同一个原因还会让 `grasp_block.py` 打印
`[warn] using built-in joint limits (robot_state_publisher not answering)` ——
`/robot_description` 的参数服务要靠节点名找，找不到就退回内置限位。
**这条 warn 不影响抓取**（内置限位就是从同一份 URDF 抄来的），但看到它就说明
现在跑在 helper 上。

### 5.4 `effort` 全是 `.nan`

夹爪和关节都感知不到接触力。所以**没有任何力学判据**可用，一切都是几何 + 夹爪行程。
"抓到没抓到"只能靠夹爪停在中间行程推断（§6），那是代理判据不是真值。

### 5.6 底盘会转，桌面高度会变

同一组常量只在底盘没动过的前提下成立。视角变了但 `head_pitch/yaw` 没变，就说明底盘
转了 —— 此时 `TABLE_Z` 和整个工作带都要重新确认。

---

## 6. 裁定：`grasped` / `empty` 是代理判据

闭合夹爪后读 `grip_state()`：夹住物体时夹爪停在中间行程，走到底（~5 mm）说明夹空了。
门限取 30 mm。

这是**代理判据，不是真值**。所以 `outcome()` 里把判据本身一起记下来
（`basis`、`grip_mm`、`auto=true`、`needs_video_review=true`），将来
`adjudicate.py`（docs/05 §E1.3）重跑日志得出不同结论时，能看出分歧在哪。

裁定词表（前六个来自 docs/05 §E1.3）：

| verdict | 含义 |
|---|---|
| `grasped` / `empty` / `dropped` | 成功 / 夹空 / 掉落 |
| `pose_error` / `collision_abort` / `timeout` | 位置偏差 / 碰撞中止 / 超时 |
| `refused` | 安全前置条件不满足（如工作区有人），一步都没动 |
| `skipped` | 没有可抓目标 / `--dry-run` / 无 IK 解 |
| `held` | `--hold` 停在抓取位没闭合，成败由人看照片定 |
| `crashed` / `unjudged` | 抛异常 / 跑完了却没写裁定（后者是流程漏洞的信号） |

`--hold` **不会**自动写成功 —— 没闭合夹爪就没有成败可言。

---

## 7. 每次试验都落盘（`exp_log.py` + `xr1_experiment.py`）

**这里有两个记录器，它们不是重复的，是分工的。** 先说清楚，否则很容易以为其中一个
该被删掉：

| | `xr1_experiment.Step` | `exp_log.Experiment` |
|---|---|---|
| 粒度 | 一个 **run** 里的一个动作步骤 | 一次**试验**的内部过程 |
| 负责 | 视频：先开录再动、`rec_confirm_ms`/`lead_s` 证据、clip 收进 run 的 `clips/`、ffprobe 帧数、烧字幕拼影片 + `REPORT.md` | 事件流 `record.jsonl`、代码指纹、协议 §4.3 字段、`index.jsonl` |
| 谁调谁 | 被调 | `Experiment._rec_start()` 里**整段委托**给 `Step` |

`Experiment` **不自己碰 `xr1_cam`**。理由：`Step` 已经把录像这件事做对了（阻塞到
Mac 确认在录、把 clip 按 seq 收好、用 ffprobe 数真实帧数、能把整个 run 烧成一条
影片）。再写第二条录制路径，只会分叉出两套 `experiments/` 布局和两份都不完整的
记录。反过来 `Step` 没有事件流、没有代码指纹、没有 `index.jsonl`，所以也不能反过来
只留 `Step`。两边靠 `run_id` / `step` 互相指认：`meta.json` 里能查到影片，
`run.json` 里能找回这次试验。

落盘长这样：

```
experiments/
  index.jsonl                       # 每行一次试验的摘要，唯一需要顺序扫的文件
  CURRENT                           # 指向当前 run（让多次 CLI 调用并进同一个 run）
  20260810-124258_smoke/            # ← Experiment：单次试验
    meta.json                       # 环境、参数、代码指纹、裁定、录像状态、run_id/step
    record.jsonl                    # 时间轴事件流
    view.jpg  hold_*.jpg  after_lift.jpg
  20260810-1252_grasp/              # ← Step：run 级
    run.json                        # 步骤表
    steps/01_grasp.json             # 每步的状态快照 + video.lead_s + probe.frames
    clips/01_grasp.mov              # 外部摄像头素材
    movie.mp4  REPORT.md            # `xr1_experiment.py end` 生成
```

`record.jsonl` 的事件：`begin` → `record_start` → `snap` → `perceive` →
`candidates` → `plan` → `joints`(每段前后) → `grip`
→ `at_pregrasp` / `at_grasp` → `outcome` → `record_stop`。
`plan` 那条带齐了协议 §4.3 要的规划信息：目标点、tcp 目标、两组关节角、IK 误差、
**接近角**、余隙、FK 反算的指尖中点和残差。
`record_stop` 那条带回 `frames` —— **帧数才是"真的录到了"的证据**，`ok=true` 只说明
命令发出去了。

四个设计决定：

- **记关键脚本的 sha1 + mtime**。自演化的前提是"这次的结果能对应到当时那份代码"。
  workspace 不是 git 仓库，所以退化成指纹。改了常量忘了记，就是在制造无法复现的实验。
- **凑不齐的字段显式写 `null` + 原因**，不省掉。`ground_truth_pose: null` +
  `ground_truth_reason: "ArUco 真值通道未搭"` —— 省掉会让后来的人以为那次没测。
- **抛异常也要落盘**。`__exit__` 里记异常并写 `crashed`，**失败的试验才是要研究的
  那些**。跑完却没人调 `outcome()` 写 `unjudged`，别默默当成功。
- **先问 Mac 再建 `Step`**。`Step.__init__` 会立刻占一个 run 目录，所以
  `_rec_start()` 先 `xr1_cam.require_ready()`（ControlMaster 下 ~20 ms）再构造；
  否则录制起不来的每一次都会留下一个空 run 目录。

裁定与 run 的 ok 位是对齐的：不是 `grasped` / `held` 就调 `Step.fail(verdict)`，
所以 `REPORT.md` 不会把夹空写成成功。

看历史：`python3 exp_log.py 20`（试验维度）、`python3 xr1_experiment.py ls`（run 维度）。

这条委托接缝有自动检查：`python3 test_experiment_pipeline.py`（把相机换成 ffmpeg
假素材，23 项断言，不碰机器人也不碰 Mac）。**接缝断了不会抛异常，只会安静地录不到
东西**，所以改动 `exp_log` 或 `xr1_experiment` 之后一定跑一遍。

---

## 8. 外部摄像头（`xr1_cam.py` + `mac/xr1rec.swift`）

机器人自己的相机看不全动作（§5.1 的视角问题），所以第三视角录像是判定"到底发生了
什么"的唯一可靠证据。录制端是 Mac（192.168.123.138，用户 `apple`）上的
`XR1Rec.app`，机器人侧通过 SSH 写控制文件驱动。

```bash
python3 xr1_cam.py install    # 推源码+编译+启动（会重置 TCC 授权，别放进日常流程）
python3 xr1_cam.py relaunch   # 只重启不重编（XR1REC_NOBUILD=1，保住 TCC 授权）
python3 xr1_cam.py doctor     # 一次把所有会挡住录制的原因查完
python3 xr1_cam.py selftest   # 录 3 秒再取回，验证整条链路
```

**别在抓取脚本里直接调这些。** 调用链是
`grasp_block.py` → `exp_log.Experiment` → `xr1_experiment.Step` → `xr1_cam`（§7 的
分工表）。`Step` 负责把 clip 命名成 `NN_action.mov` 收进 run 的 `clips/`，绕过它就
拿不到 `lead_s` 和帧数这两样证据。

五个必须知道的事：

- **`start()` 阻塞到 `state == recording`**，不是 touch 完就返回。
  `startRecording()` 是异步的，touch 完立刻动手臂会丢掉前几十帧 —— 而动作起始瞬间
  恰恰是回放时最需要的一段。`Step` 把这段等待量下来记进 step JSON：这是"先开录后
  动作"的**证据**，不是承诺。两个数别混：`rec_confirm_ms` 是纯本机时钟（从发起
  到确认在录），`lead_s` 用了 Mac 写的时间戳因而**含两机时钟差** —— 要严格论证时序
  就看 `rec_confirm_ms`。
- **`install` 和 `relaunch` 必须是两个命令**。授权绑在 .app 的 cdhash 上，
  `install` 会重新编译因而丢掉授权；日常重启一律用 `relaunch`。
- **相机授权（TCC）只能在 Mac 的 GUI 会话里点 Allow**，远程代点不了。`state` 是
  `awaiting_permission` 时任何远程操作都推不动 —— 这就是目前 §9 第 2 条卡住的原因。
- **UVC 的帧间隔以 100ns 为单位**，所以标称 30 fps 读回来是 30.00003。
  `minFrameRate <= 30` 这种精确比较会**拒掉全部 21 个格式**，导致 `activeFormat`
  根本没被设置过。`bestFormat()` 里留了 `eps = 0.2` 的容差。
- **录像失败默认不阻断试验**（记 `recorder.ok=false` + 原因继续跑）。为一段视频废掉
  一次真机试验不值得。要严格用 `--require-video`。

SSH 用的是专用密钥 `~/.ssh/id_xr1rec`（公钥已装进 Mac 的
`~/.ssh/authorized_keys`），不在仓库里存密码。不想要了就把该行从 Mac 的
`authorized_keys` 删掉。

---

## 9. 目前卡在哪

> ## 🔴 2026-08-11 更新:方向变了,而且这一节的实时版本搬到了 [`STATUS.md`](../STATUS.md)
>
> **账本先说清**:到 08-11 上午为止 **38 次成形试验,`grasped` 仍然是 0**
> (held 13 / skipped 13 / crashed 6 / refused 5 / empty 1)。46 个 run 目录已整体
> 挪进 `experiments/_attic_20260811/`(没有 `rm`),因为**一次成功都没有的数据对学习
> 没有残值**,留着只会继续污染判断。
>
> **今早那次真尝试证明了瓶颈在哪**:不在感知(黄块认得准)、也不在 IK(两个解都有),
> **在路径规划的起始位形**(§4.1 的红框、`PITFALLS.md` §43)。
>
> **于是方向换了一次:不再往系统里注入绝对真值,改用摇操示范直接产出带标签样本对。**
> 判据从「停稳 2 秒」换成「**夹住了**」—— 积木被夹在两指之间的那一刻,两指中点
> **就是**积木的真实位置:不需要人瞄得准、不依赖任何外参、而且自证。这一步同时
> 绕开了下面 #9(手眼误差一次没测到)、#8(桌高)、和 §5 那条 `head_yaw` 半米误差,
> 因为它们的偏差**全都被吸收进那个映射里**,不需要拆开分别标定。
> 工具是 `scripts/teleop_truth.py`(只读、一个 publisher 都不建)。
>
> 下面这一节保留 08-10 的原文,它记录的是**当时**的归因。

状态截至 2026-08-10 17:10（**改写过一次**：13:00 那版写着"剩下的卡点全在硬件和现场，
不在代码里"，当天下午的 26 次试验证明这句是错的 —— 最大的那个卡点在 TF/标定里）。

软件侧整条链路（感知 → IK → 分段 → 记录）已经 `--dry-run` 跑通，也真的动手
抓过一次（`20260810-14`，阶梯 9 关全过）。**但至今零次成功抓取**，`-14` 那次夹爪合到
只剩 6mm、三路举证 0/3 —— 指间是空的。

每次试验的记录见 `experiments/<name>/REPORT.md`，纵向的进步链和已经躺在日志里的
硬发现见 `experiments/README.md`（都由 `scripts/exp_report.py` 从日志生成）。

### 首要卡点：head_yaw 一转就带来半米误差

转头 40° 后，同一块**没动过**的积木被算到 **59cm** 之外（`-22` 591.9mm / `-23`
584.2mm，两次独立复现，差 8mm）。而同一次里定点采样的 `u_std = 0.0 px`、
`x_std = 0.019~0.060mm` —— **随机误差比这个错小四个数量级**，所以这是系统性的：
该修标定和 TF，滤波和多帧平均是白费力气。量级推算、两个候选机制、以及还没跑的
判别实验（`head_yaw = -20°`，两个假设预测差两倍）全在 `PITFALLS.md §33`。

**在这条查清之前：把 head_yaw 钉死在 0，目标不在视野里就挪底盘，不要转头。**

### 其余卡点

| # | 卡点 | 需要谁 |
|---|---|---|
| 1 | 白桌上没有积木，底盘转过 —— 现在检到的橙色目标是桌上杂物，不是积木 | 人：把积木放回 `x∈[0.18,0.56]`、`\|y\|≥0.04` |
| 2 | ~~Mac 相机授权~~ **已解决**（2026-08-10 14:25，真机录到 3621 帧 / 1920×1080 / 29.93 fps，`lead_s=0.315 s`） | — |
| 2b | ~~`plan_failed which=pre` 依赖起始位姿~~ **已修**：`plan_to()` 直达失败就经中转位姿（§4.1，实测救回 5/40） | — |
| 4 | 深度不可用 → `rise` 只能当否决票，无法确认"这是真积木" | 见 docs/07_perception_scheme |
| 5 | ArUco 真值通道未搭 → 裁定只有代理判据（§6）。**2026-08-10 17:03 有了一个替代品**：`20260810-27` 用外部模型（`gemini-robotics-er-2-preview`）在照片上点积木像素，反投影后与自己的感知只差 dx −1.7mm / dy −0.35mm。但只有一次、只有 `yaw=0`、模型是黑盒 —— **一致不等于正确** | docs/05 §E1.2；要当真值用，先在已知位置上量一遍它自己的误差 |
| 6 | 机体 TF 现在靠 §5.3 的 helper rsp 撑着 | 下次重启控制器时会自然恢复；`ros2 node list` 看不到它是正常的 |
| 7 | **可达性判定不可复现**：同一份代码（`ik_probe.py` 指纹相同）、同一块积木，`20260810-25` 四级高度全通、`-26` 第一级就判碰撞。差别只在手臂当下停在哪（通过那次第一级用了 17 段路径绕过去，后三级各 1 段） | 代码：每次探测前先归到**固定起点**，否则两次结果没法比。→ **08-11 机制已查清**：起点在桌面**以下**时关节空间直线必扫桌面，`STAGING_Q` 也在桌面以下所以中转救不回来（§4.1 红框 / `PITFALLS.md` §43）。修法就是 `xr1.py home` |
| 8 | **拟合桌面高度有 146mm 的离群**：`-24/-25/-26` 聚在 0.7684~0.7696 m，`-21` 却是 0.9146 m，桌子没动。最可能是它的"白色大区域"拟合抓到的不是桌面 | 代码：给平面拟合加一条"高度偏离常量 `TABLE_TOP` 超过 xx mm 就拒绝"的护栏。→ **08-11 复测把这条降级了**：白桌面平面拟合本身很干净（残差 σ=10.7 mm、倾角 0.26°），但**绝对高度整体偏 +62.8 mm**，而粉桌布那次偏 **−48 mm** —— **反号**⇒ 是偏差不是离群，且分不清"深度偏"还是"`TABLE_Z=0.75` 过期"。护栏只能挡离群、挡不了偏差（`PITFALLS.md` §41） |
| 9 | 手眼误差**至今一次都没测到**：`calib_probe`/`tip_probe` 四次试验全在探测点被判碰撞或停在 `--plan-only`，`probe_pose → gripper_measured → handeye_error_mm` 这条链一次没走完 | 先解决 #7，再让探针真的走过去。→ **08-11 起改走摇操**：让**人**把夹爪摇过去并**夹住**，绕开"探针自己走不过去"这个前提（`teleop_truth.py`）。这是整条链上**唯一从未被测量**的量，其余卡点都只能等它 |
| 11 | **腕相机 URDF `origin` 左右完全相同**（`xyz="0 -0.0768 0.0995"`，rpy 也一样），而两条臂的关节轴是**镜像**的 ⇒ 至少一侧 y 符号错，量级 ~154 mm；而且这个数**没人记录是谁量的**（`PITFALLS.md` §28/§44）。腕相机是唯一绕开 `head_yaw` 误差链的通道，却建在这上面 | 别在它上面建手眼标定。`teleop_truth.py` 顺带存了腕相机 TF + 两路腕相机图，可以把它**拟合**出来 |
| 12 | **`ZED` 好不好不能看图像 Hz**：它自己那 6 个 TF frame 会整棵消失而图像照发，`xr1.py status` 因此从 08-10 16:22 一直误报全绿到第二天早上 | 已修进 `xr1_verify.py`（zed frame `<5` 直接 FAIL）。判据是**数 frame**，修法是重启 `Astrabot_ZED.service`（`PITFALLS.md` §38） |
| 10 | 规划会把目标改投到可达区（`block_replanned`：`-21` 155mm，`-24/-25/-26` 17~18mm）。**不是规划器乱搬** —— 目标越出可达带越远，改投越多 | 人：把底盘挪近，或只选 `x∈[0.18,0.56]` 带内的目标 |
