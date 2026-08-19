# Vision Harness — 第一步结论

[English](harness-step-1-verdict.md) | **中文**

用一个验收标准来评估当前的 `thor-workspace-live` harness：*每台机器人下载后直接使
用、快速完成任意任务并自进化*。这是"先下结论"这一步。格式仿照
[`deepseek-harness`](https://github.com/deepseek-ai/deepseek-harness)：结论在前，
证据用表格呈现，对未完成的部分保持诚实。

> [!WARNING]
> **开发者预览。** 这是一份评估，不是发布。下面的结论刻意苛刻。每一个评分都是关于
> *当前仓库里由代码和实测证据所能证明的东西* 的断言 —— 不是关于意图、设计笔记，或
> 某次未来重构可能达成的目标。本文档会随 harness 重构而变化。引用任何评分前，请重
> 新阅读被引用的文件。

---

## 结论

按"下载即用（任意机器人）、可自进化"的标准，**当前这套 Harness 不合格。**

它现在更准确的定义是：

> 针对一台特定 XR1、特定 ROS 环境、特定桌面、特定相机和黄色方块实验构建的
> **确定性实验执行内核**。

它不是跨机器人产品，也还不是自进化系统。直接复制到第二台机器人，**即使同型号**，也
可能出现最危险的失败：**软件正常输出、物理结果错误** —— 因为固化在内核里的数值是在
*第一台*机器人的硬件和工作站上测出来的。

这里对应 `dsh` 的是 `xr1-vision` 二进制。它能运行，能记录、回放、规划，并把运动挡在
真实的安全包络之后。这些是真实且经过测试的。它们也就是全部被证明的东西。

---

## 苛刻评分

评分先针对**单台 XR1 的实验价值**，再针对验收标准里的两个断言（可迁移、自进化）。

| 维度 | 当前评分 | 主要原因 |
|---|---|---|
| 单台 XR1 实验价值 | **5 / 10** | 能记录、回放、规划，但关键硬件链路（D405、触觉、实时 servo Jacobian）只是实现了，尚未在活体上验证。 |
| 代码模块化 | **5 / 10** | 有目录分层，但没有硬件端口抽象；编排文件正在变成新的大文件中心。 |
| 跨机器人可迁移 | **1 / 10** | 单机常量位于*核心*中。没有 `RobotProfile`、没有 `CalibrationManifest`、没有失效绑定。 |
| 自进化 | **0.5 / 10** | 只有理念和证据记录。没有 episode 标准、judge 注册表、训练、晋升或回滚的实现。 |

**65 个 Rust 测试全部通过。** 这只能证明这些已编码规则没有被当前测试打破。**不能**证
明可迁移、可安装或可以自进化。（验证：`grep -rc "#\[test\]" crates/xr1-vision/src`。）

---

## 五个最严重的问题

### 1. "通用接口"目前主要是命名，不是真正可替换的边界

Rust 源码里**没有真正的硬件端口 trait**。所有模块还从
[`crates/xr1-vision/src/lib.rs`](../../crates/xr1-vision/src/lib.rs) 直接扁平公开：

```rust
pub mod cli;
pub mod grasp_loop;
pub mod hardware;
pub mod kinematics;
pub mod perception;
pub mod planning;
pub mod proposal;
pub mod safety;
pub mod servo_loop;
pub mod task;
pub mod visual_servo;
```

真正需要按机器人替换的边界从未被抽出来：

```rust
trait ObservationSource      // 帧/状态从哪里来
trait MotionExecutor         // 谁在什么包络下移动手臂
trait KinematicsValidator    // 针对该 URDF 的 IK 与可达性
trait OutcomeJudge           // 物理结果是否匹配谓词
trait TaskSkill              // 一个可插拔能力
```

现在主要通过 **Python 文件名、环境变量和 JSON 子进程边界**（`py/`、
`servo_adapter.py`、`grip_adapter.py`）形成隐式接口。它能运行，但无法可靠替换为另一
台机器人的实现。

### 2. 单机绑定非常严重，而且就在核心里

[`crates/xr1-vision/src/kinematics/types.rs`](../../crates/xr1-vision/src/kinematics/types.rs)
把当前夹爪、指尖和桌面几何固定成编译期常量：

```rust
pub const TIP_CENTER_M: [f64; 3] = [-0.0225, 0.0, 0.0485];
pub const OPEN_JAW_GAP_M: f64 = 0.0465;
pub const PLANNING_MIN_TIP_Z_M: f64 = 0.785;   // <- 这是一个桌面高度
```

这些不是软件默认值。它们是**某台机器人、某个工作站的测量事实。**
`PLANNING_MIN_TIP_Z_M = 0.785` 是*那张桌子*的地板。把它发到另一张桌子上的机器人，几
何门就会悄悄撒谎。它们都不应位于通用核心中，而应属于按机器人的 profile 和标定清单。

### 3. 任务接口看似语义化，实际能力非常窄

[`crates/xr1-vision/src/proposal.rs`](../../crates/xr1-vision/src/proposal.rs) 只暴
露了：

```rust
pub enum Task     { Grasp, PickPlace }
pub enum GraspIntent { TopDown }
```

而且 `object_id` 是**标签，不是可插拔的 grounding**。无论填什么字符串，
`grasp_request()` 都会拒绝除一个硬编码值以外的一切（`proposal.rs:208`）：

```rust
if object_id != "yellow_block" {
    return Err(format!(
        "object_id {object_id:?} is not supported by the current measured detector"
    ));
}
```

底层的 [`perception/mod.rs`](../../crates/xr1-vision/src/perception/mod.rs) 无论如何
都进入黄色重建，而
[`perception/yellow.rs`](../../crates/xr1-vision/src/perception/yellow.rs) 使用的是
**在特定实验帧上测出的**颜色阈值（`yellow.rs:59`）：

```rust
// Thresholds measured on frames 20260818-112803 and 20260818-170043.
// The two-sided R/G window rejects both the green cube and orange pads.
if (20..=5000).contains(&area)
    && sum_red >= 0.85 * sum_green
    && sum_red <= 1.15 * sum_green
    && mean_chroma >= 10.0
```

所以"语义化"的对象查询最终落到一个只针对一种光照调好的探测器上。它是诚实的——遇到
别的东西会失败关闭——但它不是通用任务接口。

### 4. 编排模块已经开始变成新的"大文件中心"

| 文件 | 行数 |
|---|---|
| [`servo_loop.rs`](../../crates/xr1-vision/src/servo_loop.rs) | 990 |
| [`visual_servo.rs`](../../crates/xr1-vision/src/visual_servo.rs) | 936 |
| [`grasp_loop.rs`](../../crates/xr1-vision/src/grasp_loop.rs) | 871 |
| [`cli.rs`](../../crates/xr1-vision/src/cli.rs) | 555 |

其中 **CLI 参数解析、JSON 解析、证据存储、锁和适配器子进程协议没有分开。** 一个把传
输、策略和载荷混在一起的边界，不可能成为另一台机器人的稳定端口。

### 5. "自进化"目前只是设计概念

[`docs/decisions/0005-automatic-reset-is-the-ceiling.md`](../decisions/0005-automatic-reset-is-the-ceiling.md)
认真讨论了 judge、黄金集和自动复位——包括"judge 必须比 policy 好一个数量级"的算术推
导。这些思考是真实的。但**代码里没有以下任何一项：**

- [ ] Episode 数据标准（不可变 observation / action / outcome）
- [ ] Outcome Judge 注册表（带 `abstain`）
- [ ] 训练流水线
- [ ] Policy / 模型注册表
- [ ] baseline / challenger 对比
- [ ] Shadow evaluation
- [ ] 自动晋升条件
- [ ] Canary 部署
- [ ] 回滚
- [ ] 跨机器人数据隔离
- [ ] 防止错误标签形成正反馈的实现

因此目前不能叫"自进化"，最多叫*"为未来训练保存了部分证据"*。

---

## "下载即用"应该怎么定义

不应理解成"跳过标定直接运动"，而应定义成一个**引导式状态机**，在机器人被识别并验证
之前拒绝运动：

```
下载 / 安装
      ↓
自动发现硬件和 ROS 能力    （只读）
      ↓
识别机器人型号与软件兼容性
      ↓
加载该机器人 Profile
      ↓
完成引导式标定
      ↓
运行验收测试
      ↓
激活任务执行
```

即使是同型号 XR1，不同的也只是 profile 和标定——而不是源码。

---

## 目标架构

```
                    Task Executive（语义 → TaskSpec → skill）
                              │
                              ▼
                    Robot Platform Adapter
        observation / motion / kinematics / gripper / sensors
                              │
                              ▼
                    RobotProfile + Calibration
                              │
                              ▼
                    ROS / SDK / Hardware
```

建议最终仓库拆成（当前单 crate 拆成契约优先的 workspace，平台/任务/profile 数据离开
核心）：

```
harness/
├── crates/
│   ├── harness-contracts/     # 五个 trait + schema，无硬件
│   ├── harness-core/          # 规划、几何、安全包络
│   ├── harness-executive/     # 语义 → TaskSpec → skill 分派
│   ├── harness-evaluation/    # episode、judge、注册表、晋升
│   └── harness-cli/           # doctor / commission / verify / task
├── platforms/
│   └── xr1-thor/
│       ├── ros-adapters/
│       ├── moveit-bridge/
│       └── platform.toml
├── task-packs/
│   └── yellow-block-pick-place/   # <- 黄色探测器 + 方块语义搬到这里
├── profiles/
│   └── examples/
├── schemas/
├── packaging/                 # ARM64 .deb 包
└── tests/
    ├── contracts/
    ├── replay/
    └── commissioning/
```

当前 yellow detector、黄色方块语义、桌面参数应该移入 task pack 或 RobotProfile，不
能继续存在于核心能力中。

---

## Profile + 标定模型

`platform.toml` 描述机器人；标定是单独的、绑定失效信息的清单。

```toml
# platform.toml（示意）
[robot]
model = "xr1"
tool  = "right_tool"

[arms.right]
planning_group = "right_arm"
urdf_hash      = "..."
moveit_backend = "xr1_moveit"

[sensors.zed]
adapter = "ros_image"
serial  = "..."

[sensors.d405]
adapter = "librealsense"
serial  = "262422270599"

[calibration]
tool           = "calibrations/tool.json"
zed_extrinsics = "calibrations/zed.json"
d405_extrinsics= "calibrations/d405.json"
tactile        = "calibrations/tactile.json"
servo          = "calibrations/servo.json"
station        = "calibrations/station.json"
```

每份 calibration **必须**绑定：

`robot_id` · `sensor serial` · `tool serial` · `URDF hash` · `station_id` ·
`measurement time` · 适用姿态范围 · 样本数 · 误差指标。

否则一份旧标定被复制到另一台机器后，系统无法知道它已经失效——这正是上面第 2 个失败
模式，只是长了脚。

---

## 安装方案

在 XR1 / Thor 上，首选不是把整个运行时塞进 Docker。USB、DDS、GPU、RealSense、ROS
overlay 和 vendor SDK 会让容器*更*复杂，而不是更简单。更合理的是发布**两个 ARM64
Debian 包**（一个基础包，一个平台包）。

CLI 表面，对应引导式状态机：

```bash
harness doctor                                        # 只读发现能力和版本
harness commission --platform xr1-thor --robot-id xr1-004   # 绑定设备并引导标定
harness verify                                        # 回放 + IK + 传感器 + 坐标 + dry-run 验收
harness task "把黄色方块放进绿色托盘"                    # 自然语言 → TaskSpec → task pack 执行
```

- `doctor`：只读发现能力和版本。
- `commission`：绑定设备并引导标定。
- `verify`：执行回放、IK、传感器、坐标和 dry-run 验收。
- `task`：自然语言转换为 `TaskSpec`，再由 task pack 执行。

如果没有 root 权限，就不能承诺自动安装 udev、ROS 包和系统服务；最多提供用户目录中的
可执行 bundle。

---

## 自进化应该怎样实现

不要让 Agent 在机器人上直接改源码。那不是自进化，是不可审计的在线开发。合理闭环是离
线训练、逐级放行、可回滚的：

```
执行 Episode
      ↓
不可变 Observation / Action / Outcome
      ↓
Judge + abstain
      ↓
候选策略离线训练
      ↓
冻结数据集回放
      ↓
黄金集评测
      ↓
Shadow 模式
      ↓
少量机器人 Canary
      ↓
达到晋升阈值
      ↓
发布新 Policy Artifact（带回滚 + 发布规则）
```

---

## 最小、非霰弹式改造顺序

按顺序做。不要一次性动所有边界。

1. **第一版只支持"相同 XR1/Thor 平台"。** 不要宣称任意机器人。
2. 引入 `RobotProfile` 和 `CalibrationManifest`，逐项迁出硬编码。
3. 只在真正可替换边界增加约五个 trait，不为每个函数制造接口。
4. 把黄色方块逻辑移成第一个 **task pack**。
5. 拆分两个大循环中的配置解析 / 证据存储 / 锁 / 适配器协议。
6. 添加 `doctor → commission → verify` 状态机。
7. 制作可重复的 ARM64 `.deb` 和根级 CI。
8. 接通**实时** task executive，而不是只 replay。
9. 实现 episode schema、judge、policy registry、shadow / canary / rollback。
10. 最后用**至少三台相同型号机器人**验证"只换 Profile，不改源码"。

---

## 必须达到的验收标准

在声称"可下载即用"之前，至少要证明：

- 全新厂家基线可以**一条安装命令**部署。
- `doctor` 在不移动任何东西的前提下发现硬件/ROS，并报告版本兼容性。
- `commission` 绑定设备并产出绑定失效信息的 `CalibrationManifest`。
- `verify` 在该机器人上通过回放、IK、传感器、坐标和 dry-run 验收。
- 从另一台机器人复制来的标定被**识别为失效**并阻止运动。
- 相同源码、只换 Profile，在**三台相同型号机器人**上运行。
- 一个任务通过实时 executive 端到端完成（不是 replay）。
- 自进化闭环产出一个可**回滚**的、被晋升的 policy artifact。

---

## 本文档的范围

这是**第一步**：结论。它固定了对现状的诚实定义、评分、五个失败点和改造顺序。它不实现
其中任何一项。第二步及之后按上面的改造顺序执行，一次一个边界，各自带证据。
