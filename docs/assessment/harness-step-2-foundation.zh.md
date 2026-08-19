# Vision Harness — 第二步 基础

[English](harness-step-2-foundation.md) | **中文**

第一步（[结论](harness-step-1-verdict.zh.md)）固定了诚实的定义和改造顺序。第二步执行
其中**软件可证明**的部分——问题 #1 和 #2，以及无需硬件即可证明的验收标准。它是增量
的、可回退的，不改变任何现有行为。

> [!NOTE]
> **第二步不是什么。** 它不碰硬件、ROS 或运动。需要机器人、机群或实时 Jacobian 的步
> 骤（`.deb` 打包、实时 task executive、三机验证）仍然开放，列在
> [仍受硬件阻塞](#仍受硬件阻塞) 一节。下面没有任何一项被声称"在机器人上可用"；只声称
> "能编译、有测试、作为软件是正确的"。

---

## 建了什么

新增一个下层 crate `harness-contracts`，除 `serde` 外不依赖任何东西——无硬件、无
ROS、无感知数学。`xr1-vision` 现在依赖它。

```
crates/
├── harness-contracts/         # 新增 —— 边界 + 按机器人的事实
│   ├── src/ports.rs           #   五个（且只有五个）端口 trait
│   ├── src/profile.rs         #   RobotProfile：几何 + 规划限制
│   ├── src/calibration.rs     #   CalibrationManifest + 失效绑定
│   └── tests/examples.rs      #   随附示例文件可加载且行为正确
└── xr1-vision/                # 行为不变；现在依赖 contracts
profiles/
└── examples/
    ├── xr1-thor.profile.json      # 精确复现编译期常量
    └── xr1-thor.calibration.json  # 拒绝为运动放行的占位符
```

### 五个端口（问题 #1）

`harness-contracts/src/ports.rs` 恰好定义了第一步说缺失的那些边界——用 trait 表达，
而不是用 Python 文件名和 JSON 管道：

| Trait | 它让什么变得可替换 |
|---|---|
| `ObservationSource` | 帧从哪里来（ROS / RealSense / 回放 / 仿真） |
| `MotionExecutor` | 谁在声明的有界包络下移动手臂 |
| `KinematicsValidator` | 对*这台* URDF 和这张桌子，该姿态是否可达且安全 |
| `OutcomeJudge` | 物理结果是否匹配——带强制的 `Abstain` |
| `TaskSkill` | 一个可插拔能力，按 id 分派 |

`Judgement::Abstain` 是刻意的：第一步（和 ADR 0005）要求 judge 必须被允许说*"我不知
道"*，而不是给出一个自信的错误标签，从而形成正反馈闭环。

### RobotProfile（问题 #2）

`kinematics/types.rs` 仍然编译相同的常量——但它们现在在核心之外有了家，还有一个绊线
保持两者一致：

```rust
// crates/xr1-vision/src/kinematics/types.rs  (test)
#[test]
fn profile_equivalence() {
    let profile = RobotProfile::xr1_thor_reference();
    assert_eq!(profile.planning.min_tip_z_m, PLANNING_MIN_TIP_Z_M); // 0.785，那张桌子
    // ... 每个 tool + planning 值都断言相等 ...
}
```

改了常量却没改 profile（或反过来），这个测试就失败。"核心里的桌面高度"现在是*profile
里的一个值*，换一个工作站只需改 JSON，不改源码
（`a_table_height_change_is_expressible_without_touching_source`）。

### CalibrationManifest（问题 #2，长了脚的版本）

那个危险场景——把标定复制到另一台机器人——现在可以被检测。`CalibrationBinding::check`
返回一个带类型的决定，只有 `Valid` 才能为运动放行：

```rust
pub enum CalibrationStatus {
    Valid,
    Mismatch    { reason },            // 不同机器人 / 工位 / URDF / 序列号
    Stale       { age_ns, valid_for_ns },
    Insufficient{ reason },            // 样本太少、误差指标非法
}
```

顺序是保证，不是巧合：身份在时效*之前*检查，所以一份"别的机器人的、但很新的"标定读
作 `Mismatch`，绝不会被当成仅仅 `Stale`
（`foreign_recent_calibration_reads_as_mismatch_not_stale`）。

---

## 关闭了第一步的哪些条目

| 第一步条目 | 第二步后状态 | 证据 |
|---|---|---|
| #1 "通用接口只是命名，不是边界" | **关闭（软件层）** | `ports.rs`：五个 trait，无硬件 crate |
| #2 单机常量在核心里 | **关闭（软件层）** | `RobotProfile` + `profile_equivalence` 绊线 |
| #2 旧/外来标定无法检测 | **关闭（软件层）** | `CalibrationStatus` + 9 个绑定测试 |
| 改造第 2 步"引入 RobotProfile + CalibrationManifest" | **完成** | 本 crate |
| 改造第 3 步"在真正边界加约五个 trait" | **完成** | `ports.rs` |

第一步的验收标准中，现在可作为**软件**证明的（其余仍受硬件阻塞）：

- [x] "五个可替换边界作为契约存在"
- [x] "相同源码、只换 Profile" —— `example_profile_loads_and_matches_the_reference`
- [x] "从另一台机器人复制来的标定被识别为失效并阻止运动" —— `a_calibration_copied_from_another_robot_is_a_mismatch_and_blocks`
- [x] "示例绝不能移动机器人" —— `example_calibration_is_a_placeholder_that_refuses_to_gate_motion`

---

## 证据

```
cargo test --workspace
  harness-contracts (unit):        14 通过
  harness-contracts (examples):     2 通过
  xr1-vision:                       66 通过   (原 65；+1 profile_equivalence)
  ────────────────────────────────────────────
  合计:                             82 通过, 0 失败
cargo clippy --workspace:          干净
```

原有 65 个 xr1-vision 测试**未被触碰、仍然全绿**——第二步在它们下面加了一层，没有改
变它们所守护的东西。

---

## 仍受硬件阻塞

第二步恰好停在软件诚实所能到达的地方。以下需要机器人，不声称完成：

- [ ] `harness-core` / `harness-executive` 拆分（机械性工作，但推迟到端口有真实实现者之后，让接缝由使用来划定，而不是猜）
- [ ] 把 yellow detector 移入 `task-packs/`（问题 #3）——一次无需硬件的代码搬迁，但它属于 executive 接线，紧随其后
- [ ] 拆分四个大编排文件（问题 #4）
- [ ] `doctor → commission → verify` 状态机（需要实时发现）
- [ ] ARM64 `.deb` 打包 + 根级 CI（需要目标机）
- [ ] 用实时 task executive 取代 replay
- [ ] Episode schema、judge 注册表、shadow / canary / rollback（问题 #5）
- [ ] 三台相同机器人"只换 Profile"验证

---

## 为什么是这一片，为什么在这里停

第一步自己的规则 #1–#3 说：第一版只支持一个平台、逐项迁出常量、只在真正边界加约五个
trait——**不要霰弹式改造。** 今天做一次五 crate 的大拆解，会拿 82 个绿测试去冒险，却
换不来*可验证*的收益，因为能证明可迁移性的那部分（第二台机器人）不在场。

所以第二步完成的是这样一片最大的切片：(a) 对问题 #1 和 #2 是真正的根因修复，(b) 无需
硬件即可完整完成，(c) 可回退——删掉一个 crate 和一行依赖即可还原。受硬件阻塞的其余
部分被明确地留作开放，而不是伪造，这与第一步结论所秉持的是同一个标准。
