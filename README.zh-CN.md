# OpenVision Harness — XR1 工作区

[English](README.md) | **简体中文**

这是运行在 Jetson AGX Thor（`tegra-ubuntu`、Ubuntu 24.04、PREEMPT_RT、
ROS 2 Jazzy）上的 **AstraBot XR1** 人形机器人抓取栈。

机器人原生控制软件是 `/opt/ros/astrabot` 下的**仅二进制厂商覆盖层**，由约 20 个
`Astrabot_*.service` 服务监管，服务退出后约 100 ms 会被重新拉起。本工作区负责其外围：
感知与运动学决定抓取位置，薄 Python 层执行手臂/夹爪命令，证据账本记录动作是否成功。

## 当前状态 — 2026-08-21

Harness 已具备一条经过软件测试的完整路径：从硬件无关契约、注册式任务包，到有界视觉/
触觉执行，再到策略晋升。机器人访问和 D405 数据流已通过真机验证，但这**不代表机器人已经
自主运行或能够自我进化**：D405 目标/Jacobian、压力传感器标定、真实评测数据以及自动复位
仍未完成。

| 范围 | 当前状态 |
|---|---|
| 机器人无关契约 | `harness-contracts` 定义五个硬件无关端口，以及带版本的 `RobotProfile` 和 `CalibrationManifest` |
| 任务封装 | 黄块抓取/放置已迁入 `task-packs/yellow-block-pick-place`，由任务注册表选择，不再硬编码在核心中 |
| 运行边界 | 参数解析、适配器协议、证据处理和动作锁位于 `xr1-vision/src/support`；物理动作继续保持有界、串行和失败关闭 |
| 近场/接触抓取 | D405 已在机器人上持续采集 20 帧、达到 11.03 Hz；双压力贴片评估和有界闭合/保持/单次释放闭环已实现，但压力 USB 映射及当前目标/Jacobian 标定仍缺失 |
| 评测与晋升 | `harness-evaluation` 实现不可变 Episode、带弃权的双通道 Judge、冻结黄金集、策略血缘、基线/挑战者 Gate、Shadow、Canary、晋升和无条件回滚 |
| 真机可用性 | SSH、控制器、ZED、关节反馈和 D405 采集可用；压力输入及当前目标/Jacobian 标定仍失败关闭，软件测试通过不等于允许使用 `--go` |

当前**没有训练器、没有真实机器人 Episode 语料、没有真实黄金集或实测 Judge 偏差，也
没有自动复位**。仓库实现并测试的是“判断外部产生的挑战策略能否晋升”的路径，不是
“机器人已经自我进化”。详见
[`docs/assessment/harness-step-5-evaluation.zh.md`](docs/assessment/harness-step-5-evaluation.zh.md)
和真机约束 [`docs/operations/status.md`](docs/operations/status.md)。

最近一次本地验证结果：**159 个 Rust 测试通过**，**运行 29 个 Python 测试（其中 2 个
硬件相关测试跳过）**，Clippy 在禁止警告模式下通过，所有文档链接有效。这些仅是离线/
软件检查。

## 架构与依赖方向

生产依赖严格单向：底层不能导入或调用上层。

```text
profiles/*.json / 标定数据                 task-packs/yellow-block-pick-place
                  │                                     │
                  └──────────┐               ┌──────────┘
                             ▼               ▼
                     harness-contracts（端口）
                         │              │
                         ▼              ▼
                harness-evaluation   xr1-vision
                         └──────────────►│
                                        │ 经过校验的 JSON/子进程边界
                     ┌──────────────────┼──────────────────┐
                     ▼                  ▼                  ▼
              py/ ROS 适配器    xr1_moveit_bridge    data/ 证据
                     │
                     ▼
                 厂商 ROS 2 节点与物理硬件
```

`xr1-vision` 链接 Rust crates 和任务包，但不直接链接 `rclpy` 或厂商 SDK。它通过显式且
经过校验的进程协议调用窄接口 Python 适配器与 MoveIt 校验器。`astrabot_rtc` 和
`astrabot_teleop` 与抓取栈共享 ROS 图和部署环境，但属于独立的 C++ 遥操作路径；Rust
抓取规划器不会导入它们。

## 仓库目录与依赖详解

下文只描述 Git 跟踪的源码和证据。`target/`、Python 缓存目录以及
`ros/rtc_teleop/{build,install,log}/` 都是可再生且被忽略的构建产物，不是架构输入。

### 根目录文件

| 路径 | 职责 | 依赖/使用者 |
|---|---|---|
| `Cargo.toml` | Rust 工作区定义，纳入 `crates/*` 与 `task-packs/*` | Cargo 1.75+；控制全部 Rust 构建 |
| `Cargo.lock` | 固定第三方 Rust 依赖解析结果 | 由 Cargo 生成和读取；依赖变更时必须同步提交 |
| `AGENTS.md` | 安全、语言、证据和仓库工作流契约 | 约束所有贡献者及自动化会话 |
| `.gitignore` | 忽略可再生构建产物，同时明确保留测量数据 | 由 Git 使用；删除证据前必须阅读其中说明 |
| `README.md` / `README.zh-CN.md` | 英文和中文入口 | 链接到后续权威架构与运维文档 |

### `crates/` — Rust 核心工作区

| 二级目录 | 职责 | 直接依赖 | 下游使用者 |
|---|---|---|---|
| `crates/harness-contracts/` | 硬件无关 Trait，以及带版本的机器人身份、几何和标定契约 | 仅 `serde`、`serde_json`；刻意不依赖 ROS、感知或机器人实现 | `harness-evaluation`、`xr1-vision`、所有任务包及 Profile/标定工具 |
| `crates/harness-evaluation/` | 不可变 Episode、Fleet Scope、带弃权的分层 Judge、黄金集、策略注册表和发布 Gate | `harness-contracts`、`serde`、`serde_json` | `xr1-vision` CLI 和未来离线评测/晋升任务；永不驱动硬件 |
| `crates/xr1-vision/` | Rust 主库与 CLI：观测、语义提案、感知、运动学、规划、安全、有界伺服/抓取闭环和证据 | 上述两个 crate、`yellow-block-pick-place`、`image`、`nalgebra`、`roxmltree`、`serde`、`serde_json` | 操作者/Agent；调用 `py/` 适配器及可选 `xr1_moveit_validator` |

各 crate 内部目录：

| 目录 | 职责 | 依赖边界 |
|---|---|---|
| `harness-contracts/src/` | `ports.rs` 定义五个可替换边界；`profile.rs` 和 `calibration.rs` 将测量绑定到特定机器人/工位/URDF | Rust 最底层；其他模块可以依赖它，它不能反向依赖其他模块 |
| `harness-contracts/tests/` | 加载 `profiles/examples/`，验证示例 Schema 和失败关闭的占位标定 | 依赖示例 JSON，不依赖真机 |
| `harness-evaluation/src/` | `episode`、`judge`、`golden`、`policy`、`promotion`、`lifecycle` 模块 | 只依赖契约和序列化证据，不含运行时硬件适配器 |
| `harness-evaluation/tests/` | 合成数据上的 Episode → Judge → Gate → Shadow/Canary/晋升/回滚端到端测试 | 使用合成数据，不能视为真机结果 |
| `xr1-vision/src/kinematics/` | URDF 模型、FK/IK、抓取几何、关节与地面余量 | 使用 `nalgebra`、`roxmltree` 和有效 Profile 常量；向规划与安全层供数 |
| `xr1-vision/src/perception/` | ZED/D405 深度、几何、近场和视觉伺服信号 | 使用 `image`；黄块检测从已注册任务包重新导出 |
| `xr1-vision/src/planning/` | 候选搜索/排序和可选批量 MoveIt 碰撞验证 | 消费感知与运动学；配置后调用 `xr1_moveit_validator` |
| `xr1-vision/src/support/` | 统一参数解析、JSON 适配器协议、证据 I/O 和独占动作循环锁 | CLI、伺服与抓取循环共用，避免边界实现分叉 |
| `xr1-vision/src/task/` | 类型化任务事件和确定性回放执行器 | 消费 Proposal、已 Ground 的技能 ID 和物理证据；当前以回放为主 |
| 其他 `xr1-vision/src/*.rs` | CLI 路由、观测、硬件能力、安全 Envelope、伺服/抓取编排与任务包注册表 | Rust 负责决策；只有适配器可以跨入物理执行层 |

### `task-packs/` — 任务专用能力

| 二级目录 | 职责 | 直接依赖 | 下游使用者 |
|---|---|---|---|
| `task-packs/yellow-block-pick-place/` | 第一个注册能力：实测黄块检测器和 `yellow_block.pick_place` 技能实现 | `harness-contracts` 和 `image` 的 PNG 支持 | 由 `xr1-vision/src/taskpack.rs` 注册；检测器由感知层使用 |
| `yellow-block-pick-place/src/` | `detector.rs` 独占实测颜色阈值；`lib.rs` 独占稳定对象/技能 ID 和 Grounding 规则 | 禁止导入 XR1 核心；新增对象应新增任务包，而不是修改核心判断 |

### `py/` — ROS 2 与硬件边界

该目录刻意不再拆 Python 子包：每个文件都是窄接口可执行边界，业务决策留在 Rust。

| 文件组 | 职责 | 依赖/调用者 |
|---|---|---|
| `astra_arm.py`、`xr1.py` | 关节反馈、限速手臂命令、URDF Clamp、G2 夹爪 Bring-up 和操作者命令 | ROS 2 Jazzy `rclpy`、厂商 Topic/SDK；供操作者与动作适配器调用 |
| `vista_observe.py` | 同步采集 ZED RGB/深度、内参、关节状态和图像时刻 TF | `rclpy`、`sensor_msgs`、`tf2_ros`、OpenCV、NumPy；由 `bin/xr1 observe` 路径调用 |
| `d405_observe.py` | 有界、对齐的 D405 RGB/深度采集，并检查流持续性和新鲜度 | `pyrealsense2`、NumPy、`rclpy`；由 D405 与抓取循环命令调用 |
| `tactile_adapter.py` | 双压力贴片采集，支持串口或用户态 CH340/PyUSB，输出 Median/MAD 证据 | Python 标准库与可选 PyUSB；由触觉和抓取循环调用 |
| `motion_adapter.py`、`servo_adapter.py`、`grip_adapter.py` | 只执行一次 Rust 批准的动作、微步或夹爪增量，并返回绑定后的 JSON 报告 | `astra_arm.py` 或 ROS 夹爪 Topic；必须先通过 Rust 安全 Gate，默认 Dry-run |
| `pad_offset_measure.py` | 离线多姿态夹爪贴片/工具偏移测量 | NumPy 和 `bin/xr1 fk`；生成标定证据 |
| `xr1_cam.py` | 通过 SSH/SCP 控制 Mac 外置相机录像器 | 依赖 `mac/` 安装与 Daemon；实验录像路径调用 |
| `test_*.py` | 离线适配器契约与拒绝行为测试 | Python `unittest`；硬件路径通过注入替代或跳过 |

### `ros/` — C++ ROS 2 工作区

| 二级目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `ros/rtc_teleop/` | Colcon 工作区，包含机器人启动集成、RTC Transport、遥操作和 MoveIt 校验 | ROS 2 Jazzy/ament、厂商覆盖层和各包系统库 |
| `ros/rtc_teleop/robot_start/` | 从机器人部署路径保存的启动材料 | systemd、Shell、ROS 环境；修改会影响共享机器人服务，必须单独做运维评审 |
| `ros/rtc_teleop/src/` | 下述三个源码包 | 由 `colcon build` 构建；输出进入被忽略的 build/install/log 目录 |

`robot_start/start_up/` 的二级内容：

| 目录 | 职责 | 依赖边界 |
|---|---|---|
| `chrony_time_syn/` | 机器人时钟同步辅助脚本 | Chrony 与机器人网络配置 |
| `config/` | Launch/服务脚本读取的启动配置 | 厂商安装目录与 ROS Endpoint |
| `documents/` | 部署说明和辅助材料 | 仅作为运维参考 |
| `environment/` | 服务启动前 Source 的环境配置 | ROS 发行版、厂商 Overlay 与运行时库路径 |
| `run_script/` | Thor Base/Supplement 启动脚本与 systemd Unit 模板 | 调用已安装 ROS 可执行文件，不能当成源码包 |
| `test/` | Controller、RTC、Teleop、腕部相机、日志与 ZED 服务定义的 Shell 契约测试 | 读取启动文件，不证明真机功能已经正确 |

`ros/rtc_teleop/src/` 下的包：

| 包 | 职责 | 直接依赖 | 与 Harness 的关系 |
|---|---|---|---|
| `astrabot_rtc/` | 通用信令、Peer/媒体 Transport 和授权 DataChannel 路由；不理解 Teleop 语义 | `rclcpp`、ROS 消息/服务、FFmpeg、JSON；可选固定版本 `libdatachannel` | 输出供 `astrabot_teleop` 使用的 `RtcDataPacket`/Peer Event；Rust 规划不依赖它 |
| `astrabot_teleop/` | Grant 验证、帧校验、Deadman/Watchdog、Owner Lease 和 Typed/Shadow 命令 | `astrabot_rtc`、`astrabot_data_interfaces`、ROS 消息、OpenSSL、Protobuf、JSON | 独立进入 Arbitration 的遥操作路径，不能绕过抓取安全路径 |
| `xr1_moveit_bridge/` | 对 Rust 抓取候选执行批量 MoveIt 2 碰撞校验 | MoveIt Core/消息、URDF/SRDF、OctoMap 1.9 ABI、Geometry/Shape 消息、JSON | 请求 MoveIt 验证时由 `xr1-vision` 规划模块调用其可执行文件 |

ROS 包内部目录：

| 目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `cmake/` | 可复用 CMake 检查/工具链片段，包括 No-exceptions 检查 | 包级 `CMakeLists.txt` 和交叉构建脚本 Include |
| `config/` | 运行时 YAML 与 XR1 SRDF 配置 | 包启动时解析或随 MoveIt 安装；无效值失败关闭 |
| `docker/` | RTC 包的可复现 Native/交叉构建环境 | Docker 和固定 ROS/SDK Image；运行节点不依赖 |
| `docs/` | 包内架构与复盘记录 | 包维护者使用；代码和顶层 ADR 优先级更高 |
| `include/` | 按 Config/Media/Protocol/Runtime/Session/Safety/Transport 分类的公共 C++ 契约 | 由同包 `src/` 实现；测试和链接目标消费 |
| `launch/` | ROS 2 Launch 入口 | `launch`、`launch_ros`、安装后的包配置和环境变量 |
| `msg/` 与 `srv/` | 生成 RTC/Teleop ROS 消息和服务契约 | `rosidl_default_generators`；RTC、Teleop、Arbitration、Data Collection 跨包使用 |
| `proto/` | 固定的 Quest `TeleopFrame` Wire Schema（仅 `astrabot_teleop`） | Protobuf 编译器/运行时；Teleop Frame Codec 消费 |
| `scripts/` | Native、Docker、ARM64 构建/格式/运行时 Staging Gate | CMake/colcon、Docker、固定 SDK；只用于开发与部署 |
| `src/` | C++ 实现和节点入口 | 公共 Header 与 `package.xml` 声明的 ROS/系统依赖 |
| `systemd/` | RTC/Teleop 安装服务 Unit 模板 | `robot_start` 部署和已安装 ROS 可执行文件 |
| `test/` | 单元、接口、集成和安全契约测试 | 包库与 GTest/Shell；不能替代 HIL 或 Soak 测试 |

`xr1_moveit_bridge` 只需要 `config/include/src/test`，RTC 两个包使用上面的扩展结构。
修改部署行为前必须阅读各包自己的 README。

### `profiles/` 与 `examples/` — 配置契约

| 目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `profiles/examples/` | XR1 Thor 的 `RobotProfile` 和 `CalibrationManifest` 示例 | 由 `harness-contracts` 解析验证；占位标定刻意拒绝放行动作 |
| `examples/` | TaskProposal、视觉伺服请求、触觉配置/标定、D405 Target 和任务回放事件示例 | `xr1-vision` CLI 和 Python 适配器输入；示例是 Schema，不是当前真机标定 |

### `data/` — 只追加的实测证据

`data/` 连同图片和视频都由 Git 跟踪。生产者必须新增带日期的记录；消费者不能静默重写
旧证据。

| 二级目录 | 职责 | 生产者/消费者 |
|---|---|---|
| `data/benchmarks/` | 带日期的 IK 与语义规划器延迟测量 | Benchmark 运行生成；用于性能基线，不用于动作授权 |
| `data/experiments/` | 操作者实验、日志、动作前后帧、手眼和伺服测量 | `xr1.py`、标定工具和操作者生成；架构文档与 ADR 引用 |
| `data/snapshots/` | 小型带日期诊断快照，包括当前机器人主机权限故障证据 | 只读诊断命令生成；`docs/operations/status.md` 引用 |
| `data/vista_runs/` | 带 RGB/深度/状态/TF 的自描述观测 Run | `vista_observe.py` 与观测命令生成；感知回归和审计消费 |

`data/experiments/` 内部二级实验组：

| 目录 | 保存的证据 | 依赖/使用者 |
|---|---|---|
| `20260817-01/`、`20260817-02/` | 结构化 Run 元数据、事件、报告和外置相机 Clip | 实验 Runner 与 Mac 录像器；用于重建这两次运行 |
| `d455_which_arm/` | 腕部相机与手臂身份监看记录 | 拍摄时 D455/USB 拓扑；用于硬件映射诊断 |
| `handeye/` | 多姿态样本、放置真值、拟合结果和 ZED 标注图 | `pad_offset_measure.py`、FK 和人工标注；用于 Tool Frame 分析 |
| `loops/` | 迭代观测/规划日志和标记帧 | 历史循环 Runner；仅作证据，不是当前运行状态 |
| `pad_sideon/` | 夹爪贴片侧视图、Camera Info 和关节状态 | ZED 与命名机器人姿态；用于贴片几何测量 |
| `servo/` | 正负关节扰动和微步前后观测 | 视觉伺服测量会话；用于 Jacobian 与对账分析 |
| `teleop_truth/` | 遥操作成功抓取真值姿态 | Teleop 与机器人状态；用于感知/定位验证 |
| `wrist_extrinsics/`、`wrist_scan/`、`wristcam/` | 腕部相机身份、扫描和外参观测 | 腕部相机拓扑和采集工具；用于 Camera Map 标定 |
| `zed_hand_probe/` | ZED 与手部视野重叠探测 | ZED 图像和机器人状态；用于可达性/可见性诊断 |

`data/vista_runs/` 内部二级 Run：

| 目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `harness-upgrade-20260819/` | 验证 Harness 升级和实时能力时采集的观测 | ZED/ROS 采集；状态文档和升级评估 |
| `servo_ik_audit/` | 审计视觉伺服感知与 IK 所用的帧和状态 | Observation Bundle、FK/IK 代码和审计分析 |
| `yellow-block-harness/` | 黄块与夹爪贴片的标准观测语料 | 感知回归、规划证据和命名帧结论 |

每个 Vista Run 下都有 `observations/` 账本。跨 Run 使用帧时必须携带 Frame ID、时间戳、
变换和机器人状态。所有实验组都依赖记录时的硬件配置；目录名与时间戳是证据身份的一部分。

### `docs/` — 权威书面上下文

| 二级目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `docs/architecture/` | 当前硬件映射、感知、运动学、Proposal 和夹爪设计 | 必须与代码和带日期证据一致；结构调整前阅读 |
| `docs/assessment/` | 中英双语的 Harness 分步评估与实现报告 | 汇总契约、任务包拆分、编排拆分和策略晋升路径 |
| `docs/decisions/` | 解释不可逆或安全相关选择的编号 ADR | 出现矛盾时用后续 ADR 取代，禁止静默改写历史 |
| `docs/development/` | 构建 Gate 和视觉伺服实现指南 | 供开发者和本地 CI 式检查使用 |
| `docs/operations/` | 真机状态、Runbook 和实测故障模式 | 硬件动作前必须阅读；依赖最新带日期观测 |

### `bin/` 与 `mac/` — 运维辅助

| 目录 | 职责 | 依赖/使用者 |
|---|---|---|
| `bin/` | 唯一生产入口 `xr1`、`audit-deps`、`check-doc-links`、自动设置 ROS Domain 的 `home`、TF Frame 健康检查 | Shell、Cargo 元数据和已 Source 的 ROS 环境；开发/运维检查调用 |
| `mac/` | AVFoundation 录像器、Launch 配置和安装脚本 | macOS Swift/AVFoundation 与相机权限；由 `py/xr1_cam.py` 远程控制 |

## 运行方式

所有 ROS 命令都必须使用正确 Domain，否则会静默接入几乎为空的 ROS 图，进而得出错误结论：

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
```

```bash
python3 py/xr1.py pose
python3 py/xr1.py bringup
bin/tf-frames
cargo build --release
bin/xr1 observe
bin/xr1 bundle
bin/xr1 validate-proposal --proposal examples/pick_place_proposal.json
bin/xr1 plan --proposal examples/grasp_proposal.json
bin/xr1 replay --proposal examples/pick_place_proposal.json \
  --events examples/task_events.jsonl
bin/xr1 sensor-status
bin/xr1 d405-observe
bin/xr1 tactile-observe --config TACTILE_CONFIG
bin/xr1 tactile-assess --mode closure \
  --config TACTILE_CONFIG --calibration TACTILE_CALIBRATION
bin/xr1 servo-loop --calibration CALIBRATION_JSON
bin/xr1 grasp-loop --tactile-config TACTILE_CONFIG \
  --tactile-calibration TACTILE_CALIBRATION --d405-target D405_TARGET
# Dry-run 通过且标定有效后才能添加 --go；每次关节/夹爪增量最大 0.05。
```

生产真机只允许 `bin/xr1`，其内部固定使用 Release。命令只返回短 receipt，完整规划按机器人状态写入
`data/attempts/attempt_*/`；同一状态的后续请求只读取该 attempt。执行器只接受 receipt 中的
`attempt_path`，不接受任意 `plan.json`。从其他机器操作时使用
`bin/xr1 --host astrabot@192.168.123.102 COMMAND ...`，不要保持交互式 SSH Shell。

移动机器人前先阅读 [`docs/operations/status.md`](docs/operations/status.md)，相信任何单次读数
前先阅读 [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md)。多个会话共享同一台机器。

## 已验证能力

| 结论 | 证据 |
|---|---|
| ZED 深度足以定位黄块 | 与遥操作成功抓取位姿独立比对，误差 2.0 mm |
| 颜色 Mask 能区分黄块、绿方块和橙色夹爪贴片 | `crates/xr1-vision/src/perception/` 与任务包中的实测帧回归测试 |
| 指定视觉伺服 Proposal 无法绕过新鲜度、URDF 余量、指尖地面和必需传感器 Gate | `visual_servo.rs`、`kinematics/`、`safety.rs` 单元测试 |
| 视觉伺服信号使用物理橙色贴片而非较大的橙色水果 | 感知回归中的命名帧 `20260818-120701-385142786-132823` |
| 微步之后必须出现不同的新观测；方向翻转或连续三次改善低于 10% 时停止 | `visual_servo.rs` 对账测试 |
| 实时视觉循环在两次观测间最多执行一个批准微步，并限制步数/时间/并发且不管理服务 | `servo-loop`、Rust 安全 Envelope、`servo_adapter.py` |
| 近场/接触循环不会用同一压力样本闭合两次；贴片不平衡停止；超压后只允许一次释放 | `grasp-loop`、`grasp_feedback.rs`、`grip_adapter.py` 及离线测试 |
| Task Event 不能跳过观测、Grounding、几何、验证或物理验证阶段 | `task/executive.rs` 状态转换测试 |
| 曾成功抓取一次 | 2026-08-18：夹住物体读数 149、空夹闭合 14，抬起后仍为 148 |
| 动作受限速且 Clamp 到实时 URDF | `py/astra_arm.py`；关节状态过期或命令通道忙时拒绝 |

## 尚未验证或尚未完成

- **抓取与物体朝向相关。** 唯一一次成功来自有利的方块 Yaw，另一个朝向仍会失败。
- **没有手臂关节力反馈。** 所有关节 `effort` 都是 `.nan`。夹爪内部两个压力贴片有实测
  数据，软件采集与确定性接触策略已实现，但它们不等同于关节力矩反馈。
- **近场硬件闭环尚未真机验收。** D405 和压力采集、抓取循环已实现；仍需确认精确 USB
  路径、Frame 字段、贴片映射、压力阈值、D405 Target/Jacobian，之后 `--go` 才能放行。
- **视觉伺服编排已实现，但当前 3×3 Jacobian 尚未在真机重新测量并验证。**
- **TaskProposal v2 与任务执行器目前接入验证和回放，并未接入完全自主的实时执行。**
- **策略晋升路径不是训练流水线。** 仍缺训练器、真实 Episode、黄金集、实测 Judge 偏差
  和自动复位。

## 文档索引

| 路径 | 内容 |
|---|---|
| [`docs/architecture/overview.md`](docs/architecture/overview.md) | 机器人归属、系统分层与组件位置 |
| [`docs/architecture/hardware-map.md`](docs/architecture/hardware-map.md) | 总线、设备节点、相机和关节 ID |
| [`docs/architecture/gripper-g2.md`](docs/architecture/gripper-g2.md) | G2 夹爪、Modbus 寄存器、抓取信号和总线健康 |
| [`docs/architecture/perception.md`](docs/architecture/perception.md) | 图像到方块位姿以及阈值来源 |
| [`docs/architecture/kinematics.md`](docs/architecture/kinematics.md) | Tool Frame、IK 与抓取 Gate |
| [`docs/architecture/proposals.md`](docs/architecture/proposals.md) | 语义 Proposal 和类型化抓取候选契约 |
| [`docs/operations/status.md`](docs/operations/status.md) | **首先阅读**：当前常量、可用能力和阻塞项 |
| [`docs/operations/runbook.md`](docs/operations/runbook.md) | 观测、规划、实验、记录与遥操作前置条件 |
| [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md) | 带判别证据的已知失败模式 |
| [`docs/development/building.md`](docs/development/building.md) | 工具链、检查 Gate 与测试目标 |
| [`docs/development/visual-servo.md`](docs/development/visual-servo.md) | 视觉伺服边界与剩余真机工作 |
| [`docs/decisions/`](docs/decisions/) | 架构为什么采用当前形态 |

`bin/check-doc-links` 会在任一文档链接失效时返回失败。
