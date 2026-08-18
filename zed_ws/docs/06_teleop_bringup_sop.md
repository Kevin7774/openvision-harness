# 06 · VR 遥操作启动 SOP + 失败经验

> 🔴 **桌高现值 = `0.8108 m`（2026-08-11 摇操真值）。本文出现的 0.75 / 0.750 / 0.7415 / 0.702 全部作废。**
> 依据：人摇到一个能**夹住**黄积木的位姿，两指中点 FK = (+0.4749, −0.3591, **+0.8238**)，减 13 mm 夹持高度。
> 积木坐在桌上、被夹在两指之间 ⇒ 不经相机、不经外参、不经人的瞄准，**自证**。详见 [`PITFALLS.md` §45](../PITFALLS.md)。
> 代码里唯一权威是 `grasp_block.py` 的 `TABLE_TOP`；**别在文档或脚本里再硬编码桌高**。


Quest 3 → LiveKit → dora → 机器人 的遥操作链路，如何**稳定拉起**、以及
2026-08-07 那次花了大半天才打通的**全部失败经验**。

> 数据来源：2026-08-07 本机实测（Jetson Thor `tegra-ubuntu`，`ROS_DOMAIN_ID=12`）。
> 所有 pid / 时间戳 / 日志行都摘自当天 `/tmp/teleop-claude-*.log` 与
> `journalctl -u Astrabot_LiveKit*` / `-u Astrabot_Data_Collection`。

---

## 0. TL;DR — 打通那一刻的正确配方

链路第一次真正通（画面出来 + 可控），靠的是三件事同时成立：

1. **停掉 crash-loop 的 `Astrabot_Data_Collection.service`**（它每 5 秒重启一次，
   抢 token server / 相机 / dora，是所有诡异现象的系统性根因）。
2. **LiveKit 媒体(:7880) + token(:5000) 两个服务都 active**，且 daemon 要连的
   `192.168.123.102:5000` POST 返回 200。
3. **Quest 端的 identity 不能和机器人的 `ASTRABOT-4` 相同**，否则谁后进谁把对方
   踢下线（`Room disconnected: DuplicateIdentity`）。

三者缺一，表现都是「Quest 连上了但看不到画面 / 机器人不动」。

---

## 1. 标准启动 SOP

> ⚠️ **安全**：图一旦被 Quest 取得控制权（`request_control`）就会驱动手臂。
> **全程手放急停**。机器人能动，这不是演习。

### 前置（一次性/每次开机确认）

| 项 | 期望 | 查法 |
|---|---|---|
| crash-loop 服务已停 | `Astrabot_Data_Collection` = inactive | `systemctl is-active Astrabot_Data_Collection` |
| Controller 已就绪 | TF `base_link→left/right_tcp_link` 可解析 | `~/tools/rosq tf base_link left_tcp_link` |
| pandas 修复在位（D3） | `venv312` import pandas 不报 dtype | 见 [`../../..`] D3 备注 |

### 步骤

```bash
# ---- 1. 止血:确保 crash-loop 服务停掉(否则它抢一切) ----
export SUDO_ASKPASS=/tmp/.ap_claude          # echo "1"; sudo -A
sudo -A systemctl stop Astrabot_Data_Collection.service

# ---- 2. 拉起 LiveKit 媒体 + token(注意:上一步 stop 会连带停掉它们!见 §2.4) ----
sudo -A systemctl start Astrabot_LiveKit.service
sudo -A systemctl start Astrabot_LiveKit_Srv.service
LIVEKIT_READY_HOST=127.0.0.1 LIVEKIT_TOKEN_URL=http://127.0.0.1:5000/token \
  /opt/ros/start_up/run/wait-livekit-ready.sh
# 关键:daemon 连的是 .102 不是 127.0.0.1,单独验一次:
curl -s -o /dev/null -w '%{http_code}\n' -X POST \
  -H 'Content-Type: application/json' -d '{}' http://192.168.123.102:5000/token
# 期望 200。000/refused = token server 没在;405 也算活着(GET 才 405,POST 应 200)

# ---- 3. 释放相机(dora 要独占 ZED + 双腕相机) ----
for u in Astrabot_ZED Astrabot_ZED_Points Astrabot_Wrist_Camera; do
  sudo -A systemctl stop $u.service
done
# 确认 /dev/video0(ZED) /dev/l_arm_cam 全 free   ← 没有 r_arm_cam 了，右腕已换 DW2

# ---- 4. 起遥操图(务必让 deploy venv 的 bin 在 PATH 最前,见 §2.1) ----
cd /home/astrabot/deploy
source /opt/ros/start_up/run/environment.sh        # ROS_DOMAIN_ID=12
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ASTRABOT_CONTROL_LAUNCHER=/opt/astrabot/run_control_node.sh
export PATH="/home/astrabot/deploy/.venv/bin:$PATH" # ← 没这行 astra 找不到 dora
LOG=/tmp/teleop-$(date +%H%M%S).log
setsid nohup astra run /home/astrabot/config/data_collection_xr1_evt2.yaml \
  >"$LOG" 2>&1 < /dev/null &
```

> ⚠️ **第 3 步的 `stop` 不是"停一次就完事"：`Astrabot_ZED.service` 是 `Restart=always`
> + `RestartUSec=100ms`。** 只要 dora 松手（拆图、节点崩、你 kill 了它），systemd 在
> **100 ms 内**就把 ZED 抢回去。所以"让 dora 抢得比服务快"这条路根本不存在，必须显式
> `systemctl stop`（显式 stop 不触发 `Restart`）。**上一次拆图之后的窗口里服务已经自动
> 回来了**，于是下一次启动必然撞 `Failed to open ZED camera: CAMERA NOT DETECTED` ——
> 那句话听起来像相机掉了，`lsusb` 里明明还在。2026-08-11 17:57:42 实测。
>
> 🔧 **绕过 `astra` 直接起（2026-08-11 18:02 实测跑通的就是这条）**：上面用的是
> `astra run` + `/home/astrabot/config/data_collection_xr1_evt2.yaml`（08-07 那份，3.2 KB）；
> 我今天验证的是 **`dora run` + `.astra/astrabot_data_collection_xr1_evt2.yml`**
> （08-11 改过，6.6 KB，10 个节点）。两份 yml 都在，**不是同一份**，别混着引用。
>
> ```bash
> cd /home/astrabot/deploy
> export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> export VIRTUAL_ENV=/home/astrabot/deploy/.venv PATH=/home/astrabot/deploy/.venv/bin:$PATH
> .venv/bin/dora run .astra/astrabot_data_collection_xr1_evt2.yml
> ```
>
> PATH 那行为什么是死规定：yml 里每个节点写的是 `path: python` —— **裸名，从 PATH 解析**。
> 直接调 `.venv/bin/dora` 二进制时它解析成 `/usr/bin/python`，于是每个节点
> `ModuleNotFoundError: No module named 'hardware'`，**报错像"包没装"，其实是解释器选错**。
> 三个前置条件的完整清单和每一条的误导性报错见 `../PITFALLS.md` §51。

### 拆图：按 PID 段，不要按名字

`dora run` 的父进程被 SIGTERM 收掉后，**10 个子节点会孤儿化到 `ppid=1` 继续跑**，
而它们的模块名各不相同 —— 按名字建 `pgrep` 清单**一定漏**（我杀了三轮才干净，
漏掉的旧 `control_astrabot` 还让我把新实例的 `failed to connect TF` 误诊成端口冲突）。
同一次 `dora run` 的子进程 **PID 连号**，所以：

```bash
P=<dora run 的 pid>
kill -TERM $P
me=$$; pids=$(ps -eo pid= | awk -v a=$((P-100)) -v b=$((P+400)) '$1>=a && $1<=b' \
  | awk -v m=$me '$1!=m{print $1}')
kill -9 $pids            # 孤儿节点无视 SIGTERM 的居多
```

⚠️ **不要用 `pkill -f <模式>`：模式字符串本身就在你这条命令行里，会杀掉自己的 shell。**
我在同一个会话里栽了两次。稳妥写法是像上面那样**显式排除自己的 PID**，
而不是靠记得写 `[ ]` 括号技巧。

拆完记得把 ZED 还回去：`sudo -A systemctl start Astrabot_ZED.service`
（`Astrabot_ZED_Points` 本来就是 `failed`，不是你弄的）。复验**要数 zed frame 个数
（正常 6）**，不是看图像 Hz —— 见 §D13 / `../PITFALLS.md` §38。

### 验收(在日志里必须看到，缺一即失败)

```bash
grep -E 'connected to room|Room disconnected|initialized successfully|published track' "$LOG"
```

- `Successfully connected to room: astrabot-room` ✅
- `Successfully registered LiveKit RPC methods: 'request_control'...` ✅
- `✅ Robot Daemon initialized successfully` ✅
- 4 路 `Successfully published track`（左右眼 + 左右腕）✅
- **不能**出现 `Room disconnected` / `Connection refused` / `DuplicateIdentity`

走 `dora run` 那条路时，日志落在
**`/home/astrabot/deploy/.astra/out/<run-uuid>/log_<node>.txt`**（注意是 `.astra/out/`，
`deploy/out/` 不存在），判据换成这四条（**都是正面判据，缺一即失败**）：

```bash
D=$(ls -dt /home/astrabot/deploy/.astra/out/*/ | head -1)   # 最近一次 run
grep 'Camera successfully opened' $D/log_eye_zed.txt          # ZED 拿到了
grep 'Setup completed'  $D/log_control_astrabot.txt           # TF 真接上了(不是 domain 0 的空图)
ls -l /proc/<left_wrist pid>/fd/3                             # → /dev/videoN 且【不带 (deleted)】
awk '{print $14+$15}' /proc/<left_wrist pid>/stat             # 隔 6 s 再读一次，必须在涨
```

⚠️ **「日志里没有报错行」和「节点压根没起来」输出完全一样** —— `webcam_node` 那条
`[webcam] ... driver negotiated` **只在协商失败时**才打印，所以健康和没起来都是空的。
我拿"无报错行"当成功误判过一次。
⚠️ **别用 `/proc/<pid>/fdinfo/N` 的 `pos:`** —— V4L2 字符设备的偏移**恒为 0**，
对"在不在读帧"零分辨力。用上面那个 CPU tick 增长：实测活着的 `left_wrist`
6 s 涨 **127** ticks（≈21% 核，在解 MJPEG）；而**打不开设备的 `right_wrist` 也在涨
71 ticks** —— 所以"有 CPU 消耗"只证明进程活着，**证明不了它拿到了图**，
那两条必须**一起**看（fd 不带 `deleted` + tick 在涨）。

### Quest 端连接参数

| 字段 | 值 |
|---|---|
| IP / 服务器 | `192.168.123.102` |
| 房间 room | `astrabot-room` |
| **identity / 用户名** | **任意，但绝不能是 `ASTRABOT-4`**（那是机器人的）。用 `OPERATOR` / `quest1` 等 |

连上 → 看到 4 路画面 → 手放急停 → `request_control` → 移动手柄，手臂跟随。

### 收尾

遥操结束后恢复相机服务，Nav2 才重新有 3D 避障输入：

```bash
for u in Astrabot_ZED Astrabot_ZED_Points Astrabot_Wrist_Camera; do
  sudo -A systemctl start $u.service
done
```

> 🔴 **验收不能只看图像 Hz**（2026-08-11 踩到，`PITFALLS.md` §38）。ZED 起来后
> **等 ~45 s 再数它自己的 TF frame**，正常是 **6 个**（`<5` 即故障）：
>
> ```bash
> ros2 topic echo /tf_static --once >/dev/null   # 先让 tf 缓存填上
> python3 ../scripts/xr1_verify.py 2>&1 | grep -i 'zed frame'
> ```
>
> 图像话题照发而这 6 个 frame **整棵消失**是真实发生过的故障，而且
> `tf2_echo base_link zed_camera_link` 会**误报正常**。这条故障从 08-10 16:22
> 起存在了一整夜，期间 `xr1.py status` 一直全绿 —— 因为它只查图像话题。
>
> `Astrabot_ZED_Points` 显示 **`failed (Result: signal)`** 是**正常的**：
> 拉遥操图时它被 `kill -9` 掉了，`start` 一下就回来，不是新故障。

> 🔴 **收尾还有一件事:把手臂归零。**（`PITFALLS.md` §43）遥操结束时手臂停在哪
> **决定下一个自主脚本能不能跑**。`20260811-03` 就是这么废掉的:上一轮摇操把右手
> 留在 tcp **z=0.547**（比 0.75 的桌面低 20 cm），于是所有从那里去桌面上方的
> 关节空间直线都扫过桌面，`plan_to` 直达和经中转**全判碰撞**，感知和 IK 都没问题
> 却一动没动。
>
> ```bash
> python3 ../scripts/xr1.py home      # 遥操收尾的最后一步,不是可选项
> ```
>
> 「可达性」是**当前位形的性质**，不是目标点的性质 —— 不归零，两次试验的结果
> 根本没法比（这也是 `docs/10` §9 #7「可达性判定不可复现」的机制）。

---

## 2. 失败经验(这半天到底踩了什么)

按「现象 → 真因 → 修法」记录。每一条当时都表现为**「Quest 连上但机器人不动」**，
迷惑性极强，因为它们叠加发生。

### 2.1 元凶：`Astrabot_Data_Collection.service` 从部署起就 crash-loop

**现象**：手动起的遥操图总在入房后不久掉线；token server 莫名被重启；相机被抢；
`astra run` 的 pid 每隔几秒冒出一个陌生的、跑 `deploy/data_collection.yaml`（**不是**
我们修好的 `~/config/data_collection_xr1_evt2.yaml`）的进程。

**真因**：systemd unit `Astrabot_Data_Collection.service`（`Restart=on-failure`,
`RestartSec=5`）在跑 `/opt/ros/start_up/run/start-data-collection.sh`，而那个脚本
**只用绝对路径 `exec "$VENV_DIR/bin/astra" run ...`，没有把 venv 的 bin 加进 PATH**。
`astra` 会 shell-out 调 `dora`，PATH 里没有 dora →

```
astra: `dora` not found on PATH — is the project env active? (needs dora-rs-cli)
```

→ 立刻退出 → systemd 5 秒后重启 → **循环了 464 次**（`NRestarts=464`）。
每次重启都 `wait-livekit-ready` + 抢 token server + 抢相机 + 在同一个 dora
coordinator 上抢注册，把我们手动起的图反复踩死。

**这个服务从部署那天起一次都没成功跑起来过**，纯粹是部署脚本的 PATH bug。

**修法（当时的止血）**：
```bash
sudo -A systemctl stop Astrabot_Data_Collection.service   # 它本来就是 disabled,不会开机自启
```

**根治（TODO，尚未做）**：给 `start-data-collection.sh` 在 `exec` 前加一行
`export PATH="$VENV_DIR/bin:$PATH"`（与 `run_teleop.sh:93` 同款）。改完这个服务
本身就能当正牌遥操入口，就不必手动 `astra run`。⚠️ 但它跑的 `deploy/data_collection.yaml`
是厂商默认（很可能 `use_byte:false`、URDF 没修），要先把它指向我们的 config
（`DATA_COLLECTION_CONFIG` 环境变量可覆盖）。

### 2.2 手动起图必须自己把 venv bin 放进 PATH

同一个 PATH 教训的另一面：手动 `astra run` 时，**先** `export PATH="$DEPLOY_DIR/.venv/bin:$PATH"`
再 exec，否则和 §2.1 一样 `dora not found`。`run_teleop.sh` 是对的（第 93 行），
`start-data-collection.sh` 是错的。这就是为什么手动能跑、服务不能跑。

### 2.3 token server 连接空窗：`Connection refused (os error 111)`

**现象**：daemon 日志
```
Error: Failed to initialize robot daemon
  0: Failed to send token request
  4: Connection refused (os error 111)
```
但我前一秒还验过 `.102:5000` 返回 200。

**真因**：§2.1 那个 crash-loop 服务每次重启会**连带重启 token server**（它的 unit
`Requires=Astrabot_LiveKit_Srv`）。daemon 恰好在 token server 的重启空窗里发请求 →
refused。dora 里**任何一个节点退出会拖垮整张图**，所以 robot_daemon 一 refused，
全图就塌。

**修法**：停掉 crash-loop 服务后空窗消失。启动前用 `curl POST .102:5000` 连测 5 次
确认稳定（当时首次 000、后 4 次 200，就是还在 flap 的信号）。

### 2.4 stop crash-loop 服务会**连带停掉 LiveKit**（反直觉）

**现象**：停掉 `Astrabot_Data_Collection` 后再起图，daemon 又 `Connection refused`；
一查 `Astrabot_LiveKit` 和 `Astrabot_LiveKit_Srv` 双双变成 **inactive**，:7880/:5000
都没监听了。

**真因**：`Astrabot_Data_Collection.service` 里有
`Requires=Astrabot_LiveKit.service Astrabot_LiveKit_Srv.service`。当没有别的单元
require 这俩时，systemd 在停止 Data_Collection 时会把它的依赖**一起停掉**。

**修法**：停完 crash-loop 服务后，**必须显式** `systemctl start Astrabot_LiveKit
Astrabot_LiveKit_Srv` 再验就绪（就是 SOP 步骤 2 的顺序）。

### 2.5 最后一公里：`Room disconnected: DuplicateIdentity`

**现象**：机器人成功入房、4 路 track 都 published、稳定无掉线；然后我戴 Quest 一连，
**画面没有、机器人不动**。查 daemon 日志：
```
WARN Room disconnected: DuplicateIdentity
```
而 LiveKit 媒体服务端显示房间里唯一稳定 ping 的参与者是 `ASTRABOT-4`
（participantID `PA_...`，RTT 3~111ms）——**那其实是 Quest**。

**真因**：Quest App 里的 **identity 被填成了 `ASTRABOT-4`**，和机器人 daemon
（config `identity: "ASTRABOT-4"`）撞车。LiveKit 不允许同一 room 里两个相同 identity，
**后进者把先到者踢掉**。Quest 后进 → 机器人被踢下线 → 房里只剩 Quest → 没有机器人
推的 track → 黑屏、不可控。

**修法**：Quest 端 identity 改成任意**别的**名字（`OPERATOR` 等）。两端靠**同一个
room 名**（`astrabot-room`）汇合，**不是**靠同一个 identity。改完机器人稳定在房、
Quest 作为第二参与者进来 → 画面出现 → `request_control` 可控。**这一步是当天真正的
最后一公里。**

---

## 3. 关键设施速查

### 3.1 为什么 ZED 不能和遥操并存(不是缺陷,是取舍)

ZED 2i 是一路 USB 相机 `/dev/video0`，V4L2 同一时刻只能被一个进程独占 open。两个互斥消费者：

| 谁 | 用途 |
|---|---|
| ROS 侧 `Astrabot_ZED` / `Astrabot_ZED_Points` | 出点云 → Nav2 stvl_layer **3D 避障** |
| dora 侧遥操图的 `eye_zed` | 抓画面 → 编码推 Quest（第一视角）|

只能二选一。`run_teleop.sh` 的设计意图就是：遥操时 stop ZED 服务让 dora 独占，
遥操结束再 start 恢复 Nav2 输入。代价是遥操期间 Nav2 少一路 3D 障碍点云——遥操时
人在直接控臂，本就不依赖自主避障，取舍合理。要真「并存」得加硬件或用 ZED SDK 多客户端层，
EVT2 没上。

**2026-08-10 补充：「二选一」可以不选 —— 直接省掉 `eye_zed` 整块。**
`AstrabotDeployment.eye_zed` 的类型是 `ZedPairDeployment | None = None`，**是可选的**
（`hardware/dataflows/modules/robots/astrabot.py:100`）。config 里不写这一块，dora 就
根本不去 open ZED，`Astrabot_ZED.service` 可以**全程不停**。

代价和收益要算清楚：

| | 有 `eye_zed` | 省掉 `eye_zed` |
|---|---|---|
| Quest 第一视角画面 | 有（左右眼）| **没有** |
| 腕/胸 3 路画面 | 有 | **有**（`/dev/{l,r}_arm_cam`、`/dev/f_chest_cam` 是独立 UVC 设备，和 ZED 无关）|
| ROS 侧 ZED 话题 | 断（感知链、Nav2 点云全瞎）| **活** |

所以当遥操的目的是**给感知链采真值**（人把夹爪摇到目标上方，用 FK 反推真实坐标去
和感知算出的坐标对账）时，**必须**省掉 `eye_zed` —— 否则 ZED 一被抢，要对账的那一侧
就没了，这趟遥操也就白摇。摇操手靠腕部+胸部 3 路画面操作，实测够用。

### 3.2 关键端口 / 服务 / 文件

| 项 | 值 |
|---|---|
| LiveKit 媒体 | `192.168.123.102:7880`，`Astrabot_LiveKit.service`，`--dev` 模式 |
| LiveKit token | `192.168.123.102:5000`（bind `0.0.0.0`），`Astrabot_LiveKit_Srv.service`，flask |
| 机器人 identity | `ASTRABOT-4`（config 固定）|
| room | `astrabot-room` |
| 遥操 config | `~/config/data_collection_xr1_evt2.yaml`（`use_byte:true` 必须）|
| 手动启动脚本 | `~/config/run_teleop.sh`（step-1 `set -e` 在 LiveKit flap 时会误退出，SOP 里绕过它自己按序做）|
| 就绪等待 | `/opt/ros/start_up/run/wait-livekit-ready.sh`（env: `LIVEKIT_READY_HOST` / `LIVEKIT_TOKEN_URL`）|
| crash-loop 元凶 | `Astrabot_Data_Collection.service` + `start-data-collection.sh`（PATH bug）|
| sudo | `SUDO_ASKPASS=/tmp/.ap_claude`（echo "1"）+ `sudo -A`；「unable to resolve host」告警无害 |

### 3.3 遥操动作出口话题(验证「动作真进机器人」时盯这些)

| 话题 | 含义 |
|---|---|
| `/astrabot/left_ee_vr/pose`、`/astrabot/right_ee_vr/pose` | VR 手柄目标位姿（Quest 取控后应有数据）|
| `/astrabot/left_ee_fiter/pose` 等 | 滤波后 |
| `/actuator_manager/joint_position_commands` | 最终发给臂的关节指令 |

`/reference/pose` 有发布者依赖 D3（venv312 pandas 修复）——若 pandas dtype 报错，
`robot_control_node_odom.py` 的 `pa.array()` 会抛异常，位姿永不发布。

---

## 4. 一键排障清单(下次「连上但不动」先跑这个)

```bash
# 1. crash-loop 元凶是否又活了?
systemctl is-active Astrabot_Data_Collection        # 应 inactive

# 2. LiveKit 双服务 + 端口 + daemon 实际连的 .102:5000
systemctl is-active Astrabot_LiveKit Astrabot_LiveKit_Srv   # 应都 active
ss -ltn | grep -E ':7880 |:5000 '                    # 都应在听(:5000 应 0.0.0.0)
curl -s -o /dev/null -w '%{http_code}\n' -X POST -H 'Content-Type: application/json' \
  -d '{}' http://192.168.123.102:5000/token          # 应 200

# 3. daemon 入房 / 掉线 / 身份冲突
grep -E 'connected to room|Room disconnected|DuplicateIdentity|Connection refused' /tmp/teleop-*.log | tail

# 4. 相机是否被别人占(dora 要独占)
fuser /dev/video0 /dev/l_arm_cam        # r_arm_cam 已不存在（右腕换 DW2）

# 5. LiveKit 服务端:房里到底有谁(排查 DuplicateIdentity)
sudo -A journalctl -u Astrabot_LiveKit.service --since '2 min ago' | grep -iE 'participant|identity'
```

```bash
# 6. 手臂能动但**夹爪不动**?看订阅者数(2026-08-10, §5.4)
ros2 topic info /rm_right/rm_driver/teleop_gripper_float   # Subscription count 必须 >= 1
pgrep -af g2_gripper                                       # 空 = 驱动没跑 -> xr1.py bringup

# 7. 重启图之前:上一轮的相机节点是否还attach着设备(§5.3)
pgrep -af 'fisheyecam_node|webcam_node'                    # 必须为空
fuser /dev/f_chest_cam /dev/l_arm_cam                       # 必须空闲（无 r_arm_cam）
```

对号入座：
- 现象「daemon 反复掉线 + 陌生 astra 进程」→ §2.1（停 Data_Collection）
- 现象「Connection refused」→ §2.3 / §2.4（LiveKit 没起或在 flap）
- 现象「机器人入房又立刻 DuplicateIdentity 掉线」→ §2.5（Quest 改 identity）
- 现象「dora 起不来 `dora not found`」→ §2.2（PATH 缺 venv/bin）
- 现象「画面正常、控制权也拿到了，手臂就是不动」→ **§5.1（`use_byte` 名字对不上）**
- 现象「`No module named 'rclpy._rclpy_pybind11'`」→ **§5.2（解释器 3.10/3.12 混用）**
- 现象「`camera initialization failed` + `fps: -1.0`，整图级联崩」→ **§5.3（旧相机节点没死）**
- 现象「手臂能动，夹爪不动」→ **§5.4（G2 驱动没跑，没人订阅）**

---

## 5. 2026-08-10 实测新增的坑（5.1-5.4 遥操栈本身，5.6 读数据那一侧）

这天用的是**厂商自带**的 `robot_teleop/dataflows/devs/astrabot_teleop.yaml`（不是 §1 里
那个已修好的 `data_collection_xr1_evt2.yaml`），所以又踩到一批只存在于 devs 配置里的坑。
最终能动的配置存在 `/home/astrabot/deploy/teleop_local.yaml`（我的副本，厂商文件没动）。

### 5.1 厂商 devs 配置的 `use_byte: false` 是错的 —— 数据到了机器人就掉地上

**现象**（最会骗人的一个）：Quest 连上、**画面正常**、房间里能看到机器人、
`request_control` 也**成功拿到租约**（`owner=AS`）—— 手臂就是一动不动。

**真因**：`use_byte` 只决定 dora 图里那条数据通道**叫什么名字**
（`robot_runtime/dataflows/modules/robotd.py:67`）：

```python
topic = f"teleop_byte_{identity}" if use_byte else f"teleop_{identity}"
```

而 `robot_daemon` 那个**二进制**是固定发在 `teleop_byte_<identity>` 上的（它自己的
lease 日志就写着 `topic=teleop_byte_ASTRABOT`）。`use_byte: false` 时图里连的却是
`data/teleop_ASTRABOT`：

```
robot_daemon outputs: ['data/teleop_ASTRABOT']        # 图声明的
Lease checker: ... topic=teleop_byte_ASTRABOT         # 二进制实际用的
```

两边名字不一样 → 你的手部数据到了 robot_daemon 就**没有任何节点在听**，
`teleop` 节点收不到输入，解不出 `Action_L`/`Action_R`，手臂当然不动。
注意 `RobotdDeployment.use_byte` 的**默认值本来就是 `True`**，厂商 devs yaml 是**显式写错**的。

**唯一的两个可辨识线索**（其它一切都正常，所以必须盯这两个）：
1. `Lease checker: owner=AS, topic=teleop_byte_ASTRABOT, elapsed=...` 里的 **`elapsed`
   一直往上涨**（886ms → 8.8s → 38.9s）。`elapsed` 是「距上次收到数据多久」，
   一直涨 = **从来没收到过**。名字接对之后它稳定在 ~325ms 不再增长。
2. `teleop` 节点从启动到死只有一句 `[Teleop] Teleop node spinning.`，**没有任何解码日志**。

**修法**：config 里 `use_byte: true`。改完两侧对齐：

```
robot_daemon outputs: ['data/teleop_byte_ASTRABOT']
teleop 订阅       : robot_daemon/data/teleop_byte_ASTRABOT
```

**附带的一个有用副作用**：名字接通之后，若对端发的不是 protobuf，`teleop` 会打印
探针行，比如某次收到一个测试包：

```
[TeleopFrameParser] Error parsing protobuf: Error parsing message with type 'TeleopFrame'
[TeleopFrameParser][PROBE] len=9 first_byte=0x68 head_hex=68656c6c6f20776f72 head_repr=b'hello wor'
```

**能看到 PROBE 行，说明通道是通的、只是载荷不对**（对端不是真的摇操 App，或还没进
摇操模式）。这条能把「通道断了」和「载荷不对」这两种完全不同的故障一刀切开 ——
`use_byte` 错的时候连 PROBE 都不会有。

#### 5.1.1 ⚠️ 修完 §5.1/§5.4 也可能照样不动 —— 三层判据（2026-08-12 15:00~15:56 实测）

那天机器人侧**两条静默故障都预先堵住了**（`use_byte` 两侧通道名对上、两个
`teleop_gripper_float` 订阅者数各 1、9/9 节点 ready、零错误），**手臂全程没动过**。
所以这两条是**必要不充分**，别修完就宣布通了。要按层次逐层证伪：

| 层 | 观测量 | 通了的样子 | ⚠️ 会骗你的地方 |
|---|---|---|---|
| 1 房间 | `journalctl -u Astrabot_LiveKit \| grep 'updating participant state'` | 有 `data-quest` 且没有随后的 disconnect | **头显 UI 说「已连接」不等于在房间里**：15:49:42 服务端记录 4 条 track 全 `close downtrack`、两个 peer connection `state=closed`、`participant disconnected`，操作者界面仍说连着。**服务端日志是权威** |
| 2 载荷 | `teleop` 节点的 `[TeleopFrameParser][PROBE]` 行 | 没有 PROBE 且有解码输出 | 有 PROBE = 通道通、载荷不对（见上） |
| 3 租约 | `Lease checker ... elapsed=` | **`elapsed < 50ms`** | **「不涨」不等于成功**：15:55 那段 elapsed 稳在 0.85~1.9 s、PROBE 也停了，看着像通了，手臂仍一动不动 —— tick 是 **5 ms**，0.85 s 只有 ~1 Hz，那是个能解析的心跳而不是姿态流 |
| 4 真·成功 | **指尖 FK 离开零位** `(-0.050, -0.466, +0.353)` | 数变了 | track 数、通道名、elapsed 三个都不能当成功判据 |

**为什么第 2 层那 9 个字节不可能是 TeleopFrame**（这一步把「App 版本不匹配」和
「App 根本没进摇操流」区分开）：`robot_teleop.protos.TeleopFrame_pb2` 里该 message
**只有两个字段** —— `#1 header` / `#2 data_body`，所以合法首字节只能是 `0x0a`
（field 1，wire type 2）或 `0x12`（field 2）。08-12 收到的 `first_byte=0x68` 是
**field #13**，根本不存在；`head_repr=b'hello wor'` 就是字面意义的 `hello world`
心跳，间隔 4~20 秒。⇒ App 连上了 LiveKit 但没进摇操流，**这一层机器人侧无法修**。

```bash
# 复现这个字段清单（不用起图）
/home/astrabot/deploy/.venv/bin/python -c "
from robot_teleop.protos import TeleopFrame_pb2 as m
print([(f.number, f.name) for f in m.TeleopFrame.DESCRIPTOR.fields])"
```
（模块名是 `TeleopFrame_pb2`，不是 `teleop_pb2`；`FieldDescriptor.label` 在
`google._upb._message` 下会 AttributeError，别取它。）

❌ **一个无效判据，别再用**：`ros2 topic hz /rm_right/rm_driver/movej_canfd_cmd`。
**那个话题不存在** —— 08-12 实测 `/rm_right/rm_driver/` 下只有 `teleop_gripper_float`
一个，摇操驱动手臂不走这个 ROS 通道。拿它测出的「无消息」是**判据无效**，不是证据。

**采真值的摇操图请把 `eye_zed` 节点从 yml 里删掉**（schema 里它是可选的），这样 ROS 侧
ZED 全程活着，不必执行 §1 里的 `systemctl stop Astrabot_ZED`。08-12 照抄了停 ZED 那一步，
等于白白让别的会话瞎了半小时，而那次采集需要的只有腕部 D455 + `/joint_states`。

### 5.2 控制节点必须走 py3.12 wrapper，否则 rclpy 的 C 扩展加载不了

**现象**：`control_astrabot` 秒退，其余 5 个节点全部级联失败
（`Node control_astrabot exited before initializing dora`）：

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/
  _rclpy_pybind11.cpython-310-aarch64-linux-gnu.so' isn't present
```

**真因**：就是 PITFALLS「两个解释器不要混用」那条。dora 默认 `path: python` = deploy
venv 的 **3.10**，而 ROS Jazzy 的 rclpy 是 **3.12** 编的 —— 它去找 `cpython-310` 的 `.so`，
只有 `cpython-312` 的。看那行报错里的 `python3.12/site-packages/...cpython-310...`
自相矛盾，就是这个坑的指纹。

**修法**：`export ASTRABOT_CONTROL_LAUNCHER=/opt/astrabot/run_control_node.sh`。
这是 `add_astrabot` 官方支持的钩子（`astrabot.py:155`），设了它 `control_astrabot`
的 `path` 就换成那个 wrapper（内部 `source /opt/ros/jazzy/setup.bash` + `venv312/bin/python`）。
**§1 的 SOP 里已经有这一行 export，别删。** `venv312` 里 `rclpy`/`dora`/`hardware` 都在，
只有 `robot_teleop` 没有 —— 不影响，`teleop` 节点不需要 ROS，留在 3.10 跑。

### 5.3 重启遥操图前必须先杀干净相机节点，否则整图级联崩

**现象**：改完 config 重启，`chest` 节点秒退，**整张图**跟着塌：

```
[astrabot_camera_fisheye_node] camera ID: /dev/f_chest_cam, fps: -1.0, resolution: -1.0 x -1.0
[astrabot_camera_fisheye_node] camera initialization failed
...
marking `robot_daemon` as cascading error caused by `chest`
```

**真因**：上一轮的三个相机节点**没死**，还 attach 着设备。V4L2 独占，新的打不开，
读出来的 fps/分辨率就是 `-1.0`。而 dora 里**任何一个节点退出会拖垮整张图**，
所以一个相机没让开 = 全图起不来。

坑在于 kill 的时候容易漏：这些节点的命令行是
`python -m hardware.devices.sensors.camera.{fisheyecam_node,webcam_node}`，
用 `robot_control_node_odom|robotd_bin` 之类的模式去 pkill **匹配不到它们**。

**修法**：停图之后按这个清单确认全空，再重启：

```bash
pkill -f 'fisheyecam_node|webcam_node'
pkill -f 'robot_control_node_odom'; pkill -f 'robotd_bin/robot_daemon'
sleep 3   # 给它们时间释放 fd;还不死再 kill -9
pgrep -af 'fisheyecam_node|webcam_node|robot_control_node_odom|robotd_bin'   # 必须为空
fuser /dev/f_chest_cam /dev/l_arm_cam                                       # 必须空闲（无 r_arm_cam）
```

顺带一个正面结论：这么停图，`Astrabot_ZED`、`ros2_control_node` 和另开的只读记录进程
**都不受影响**（实测三者全程存活）。

### 5.4 手臂能动但夹爪不动 —— G2 驱动没跑，指令没人接

**现象**：摇操手臂跟手很好，**扳机怎么按夹爪都不动**，且没有任何报错。

**真因**：控制节点其实**发对了**话题，但没有订阅者：

```
/rm_left/rm_driver/teleop_gripper_float   Publisher: astrabot_robot_node   Subscription count: 0
/rm_right/rm_driver/teleop_gripper_float  Publisher: astrabot_robot_node   Subscription count: 0
```

真正驱动夹爪的是 `g2_gripper_pc`（UFactory **G2, Modbus RTU** slave 8 @2000000，
左 `/dev/ttyAMA5` 右 `/dev/ttyUSB0`），而**它不是 systemd 服务** —— 任何一次重启之后
它都不会自己回来，于是发布者在喊、没人听，静默失败。

**修法**：

```bash
python3 xr1.py bringup     # 幂等:没跑才拉起
# 期望 left/right gripper readback: 8xx mm OK
```

拉起后复查两个话题的 `Subscription count` 都变成 1，扳机即可用（实测左 815mm /
右 829mm 回读正常）。

**顺序有讲究**：`g2_gripper_node` 不能在 `Astrabot_Controller` 启动**之前**占着
`ttyAMA5`/`ttyUSB0`（脖子和夹爪共用 RS485）；控制器起来之后再拉夹爪驱动就没事。
本次就是控制器早已 active、事后补 bringup，一次成功。

### 5.5 这天的连接参数（`--dev` 模式，注意安全边界）

| 项 | 值 |
|---|---|
| LiveKit | `ws://192.168.123.102:7880` |
| room | `astrabot-room` |
| 机器人 identity | `ASTRABOT`（devs 配置里是这个，不是 §2.5 的 `ASTRABOT-4`）|
| 操作端 identity | 任意其它名字（`Quest` 等），**不能同名**，见 §2.5 |
| token | `POST http://192.168.123.102:5000/token`，body `{"identity":"Quest","name":"Quest","ttl":12,"room":"astrabot-room"}` |
| key / secret | `devkey` / `secret` |

厂商 devs 配置里 `robotd.server_ip: "192.168.10.17"` 是他们内网的 LiveKit，
本机在 `192.168.123.0/24`，**实测 ping 不通**，必须改成本机（`127.0.0.1`）。

⚠️ livekit-server 跑在 `--dev` 模式：用的是**公开默认凭据** `devkey`/`secret` 且
bind `0.0.0.0`。实验室内网够用，但这台机器一旦接到不可信网络，任何人都能进这个房间
**控制手臂**。要长期用必须换掉密钥。

### 5.6 采真值那一侧的四个坑（同一天，都是"数据看着有、其实没用"）

这四个不在遥操栈里，在**读遥操**的那一侧。共同点：程序不报错、有输出、
数字看着挺像回事，但结论是错的。

**(a) 记录器只盯一条臂，录了 281 帧全是同一个姿态**
`teleop_record.py --side right` 只看右臂。那一小时右臂一动没动（真正被摇的是
头部跟随），可脚本照样每 4 s 存一帧，一直存到最后汇总才发现 281 个
`tip_center` 完全相同、连小数位都一样。修法：加了心跳，每 10 s 打印**两条臂**
各自的 10 s 极差，谁在动一眼可见：
```
  [心跳     50s] left 极差0.0000 | right 极差0.0000
```
判据是 `极差 > 0.01 rad`。两边都 0.0000 就说明这段时间在白录，立刻停。

**(b) `dx/dy` 不是感知误差 —— 先确认指尖在画面里**
关键帧里那行 `感知-真值 dx=+414.9 dy=+458.2 mm` 只有当两指中点**真的停在积木
正上方**时才是感知误差；否则它就是"手当时离积木多远"。识别方法用同一行的
`指尖按FK应出现在像素(1203,894)` —— 图像是 **960×540**，1203 > 960，
指尖根本在画面外。**先看这个像素在不在画面内，再看 dx/dy**，否则会把
40 多厘米的手-物距离当成标定误差去追。
（注意真值本身**不需要**指尖可见：FK 只用关节角。可见性只是个交叉验证。）

**(c) 用默认 QoS 订阅 `/tf_static` 会让你以为 TF 断成了两棵树**
`/tf_static` 是 **TRANSIENT_LOCAL(latched)** 发布的。用 `create_subscription`
默认的 VOLATILE 去订，**收不到早先发过的那一批**，只能收到订阅窗口内恰好重发的。
于是自己拼树会看到 `platform_link`、`chassis_*_camera_link` 各自成根、ZED 帧
一个都没有 —— 看着就像本体 TF 整棵掉了（那确实是个真实存在的故障，见
`xr1-body-tf-disappears-rsp-participant`），**但这次是测量方法的假象**。
正确做法是用 `tf2_ros.Buffer` + `TransformListener`（它自己用对了 QoS），
然后 `buf.all_frames_as_yaml()`。实测同一时刻：错方法 22 边 / 4 个根，
对方法 **45 帧 / 1 个根**，ZED 6 个帧都在。
另外 Buffer 需要**几秒**才填满 static，刚 spin 起来就 `lookup_transform`
会抛 `ConnectivityException`，那不是断树，是没等。

**(d) `pkill -f` / `pgrep -f` 会杀掉自己**
和 §5 里 `g2_gripper_node` 同一个坑，但这次是自己踩的：一条命令里同时写了
`pkill -f 'teleop_record[.]py'` 和 `exec python3 -u teleop_record.py ...`，
后半句让**本 shell 自己的命令行**含有 `teleop_record.py`，正好匹配前半句的正则，
于是 shell 先自杀（**退出码 144**），exec 那行根本没执行。
两次都栽在这：`pgrep -f "record.py --side"` 同理。
规则：括号写法只有在**整条命令里再没出现过那个字面量**时才有效。稳妥做法是
**杀和起分成两次调用**，探测用 `pgrep -af 'teleop_reco[r]d'`。

---

## 6. 2026-08-11 上午：一次干净的拉起，以及它之前必须先修的四个故障

这次遥操**一次成功**（10:39 起，4 路 track 全推、`DuplicateIdentity` / 掉线计数 **0**），
因为 §5.1 和 §5.4 两个静默故障都**预先**堵住了：`use_byte` 让通道名字对得上、
两个 `teleop_gripper_float` 的订阅者数各为 1。**这两条都不会报错，所以必须主动查。**

但在此之前先修了四个**彼此独立**的遗留故障，四条修法互不相通 —— 值得记下来的是
它们全都表现为"某个东西静默地不工作，而进程还活着"：

| 症状 | 真因 | 修法 |
|---|---|---|
| `no /joint_states within 8s` | `joint_state_broadcaster` 哑了（`/dynamic_joint_states` 一起停） | 重启 `Astrabot_Controller` → **199.885 Hz**。⚠️ **重启后第一次短探针必然假阴性**（DDS 发现要 0.7~0.9 s），当天差点因此误判"还是没修好"（`PITFALLS.md` §40） |
| 夹爪读数静默，而 `g2_gripper_node` **在跑** | 设备节点 **00:04 被重新枚举**（`/dev/ttyAMA5` 204,69；`/dev/ttyUSB0` 188,0），旧驱动进程攥着**已失效的 fd**，而且 **SIGTERM 无效**（阻塞在串口读上） | **只有 `kill -9`**，然后 `bringup`（`PITFALLS.md` §39） |
| 感知报拿不到 `base_link ← zed_camera_link` | **ZED 自己的 6 个 frame 一个都没发**，而机体 TF 完好 | 重启 `Astrabot_ZED` → 42 帧 / 6 个 zed frame（§38、本文 §1「收尾」的红框） |
| 三路 UVC 打不开 | **前一晚的遥操栈没收掉**，V4L2 独占 | 按本文 §1 停图（`pkill -f` 自杀坑又踩了一次，退出码 144 —— 见 §5.6(d)） |

### 6.1 `bringup` 在这个状态下**修不了**夹爪，而且它的输出会误导你

`xr1.py bringup` 判断"驱动在不在"用的是 `pgrep -f 'g2_gripper[_]node'`。
进程**在**（只是攥着废 fd），于是它打印 `g2_gripper_node already running` 然后
什么也不做。它接着如实地读回并打印：

```
  left gripper readback: STILL SILENT
```

**这行字是真的**，但很容易被读成"再等等就好"。判据是**比时间戳**：

```bash
ls -l /dev/ttyAMA5 /dev/ttyUSB0          # 设备节点的创建时间
ps -o lstart= -p "$(pgrep -f 'g2_gripper[_]node')"   # 驱动进程的启动时间
# 设备 比 进程 新  ⇒  进程攥的是废 fd  ⇒  kill -9
fuser -v /dev/ttyAMA5 /dev/ttyUSB0       # 确认是谁占着
```

**元教训**：「进程在」不等于「设备可用」，而且**判断"修好没有"的那个工具本身
不能假阴性**。这两条都写进了 `PITFALLS.md` 的元教训 29 / 30。

### 6.2 并发：现在这台机器上可能有别的会话

写这一节时有两个会话同时在动机器人（一个跑遥操、一个写记录器）。
**重启服务、抢设备、`pkill` 之前先 `pgrep -af` 看一眼是不是别人的进程** ——
重启是外溢动作。另外外置录制器是**独占**的：开录前
`python3 xr1_cam.py status | grep -E '"(state|clip)"'`，别人占着就等，
**不要 `xr1_cam.py stop`**（会静默作废别人整个试验，`PITFALLS.md` §29）。
