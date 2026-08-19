# Vision Harness — 第三步 Task Pack 拆分 + Executive 接线

[English](harness-step-3-taskpack.md) | **中文**

第二步（[基础](harness-step-2-foundation.zh.md)）建了五个端口，并把单机常量迁出核心。
第三步关闭**问题 #3**：黄色方块探测器及其 `object_id` 门离开核心，成为一个 task
pack，通过注册表分派，于是新增第二个对象无需改核心。

> [!NOTE]
> 仍无硬件。探测器数学逐字节不变，其实测回归测试随之搬迁——这是一次*重定位与间接
> 化*改动，不是感知改动。这里没有任何一项被声称在机器人上运行；只声称作为软件能编译、
> 有测试、可扩展。

---

## 搬了什么

```
task-packs/
└── yellow-block-pick-place/          # 新 crate
    ├── src/detector.rs               #  实测探测器（原 perception/yellow.rs）
    └── src/lib.rs                    #  impl TaskSkill：grounding "yellow_block"
crates/xr1-vision/
├── src/perception/yellow.rs          # 现在是对 pack 的一行 re-export
├── src/taskpack.rs                   # 新增：TaskPackRegistry
├── src/proposal.rs                   # 门被 registry.can_ground() 取代
├── src/task/executive.rs            # TargetLocked 通过注册表 grounding
└── src/cli.rs                       # 新增 `packs` 命令报告注册表
```

### 探测器的规范归属现在是 pack

`perception/yellow.rs` 过去持有颜色阈值。它现在是：

```rust
pub(super) use yellow_block_pick_place::detector::{component_mask, components};
```

核心里的三个调用点（`observe_object`、servo 目标掩膜、D405 近场目标）保持相同的
`yellow::…` 路径和完全相同的行为。三个实测回归测试搬入了 pack——没有丢失任何测试，它
们现在在 `yellow-block-pick-place` 里运行。

### 硬编码的门没了

之前（`proposal.rs`，第一步指出的问题，逐字）：

```rust
if object_id != "yellow_block" {
    return Err(format!(
        "object_id {object_id:?} is not supported by the current measured detector"
    ));
}
```

之后——grounding 是一次注册表查询，而这个文件不再点名任何对象：

```rust
if !registry.can_ground(&descriptor) {
    return Err(format!("no task pack can ground object_id {object_id:?} for this task"));
}
```

`TaskPackRegistry` 持有 `TaskSkill` 端口。`with_default_packs()` 恰好装载那一个真实
的 pack；`empty()` 什么都不 ground——证明核心自己不再认识任何对象
（`empty_core_knows_no_objects_on_its_own`）。

### Executive 通过注册表 grounding

`TaskExecutive::new` 现在会构造一个注册表；在 `TargetLocked` 时通过它 ground 被锁定
的对象，于是一个没有 pack 能处理的对象在*锁定时*就被拒绝，而不是稍后作为几何失败浮
现。`new_with_registry` 允许调用者（或测试）注入不同的 pack 集合。

---

## 可扩展性，由测试证明

第一步说缺失的那个性质——*不改核心就能新增对象*——现在是一个通过的测试
（`a_registered_pack_grounds_its_object_without_a_core_edit`）：

```rust
struct BlueCupPack;                       // 在调用点定义的第二个 pack
impl TaskSkill for BlueCupPack { /* grounds "blue_cup" */ }

let mut registry = TaskPackRegistry::with_default_packs();
registry.register(BlueCupPack);

// 同一个 proposal，两个注册表：
assert!(proposal.grasp_request().is_err());              // 默认 pack：不认识 blue_cup
assert!(proposal.grasp_request_with(&registry).is_ok()); // 加了新 pack：可 ground
```

没有改 `proposal.rs` 的 grounding 逻辑、`taskpack.rs` 或 executive 的任何一行来让
`blue_cup` 工作——只加了一次 `register` 调用。

而且在运行时可观测，不只是测试里：

```
$ xr1-vision packs
["yellow_block.pick_place"]
```

---

## 关闭了第一步的哪些条目

| 第一步条目 | 状态 | 证据 |
|---|---|---|
| #3 `object_id` 只是标签，不是可插拔 grounding | **关闭** | 注册表查询取代硬编码字符串 |
| #3 黄色探测器焊死在核心里 | **关闭** | 探测器位于 `task-packs/yellow-block-pick-place` |
| 改造第 4 步"把黄色方块逻辑移成第一个 task pack" | **完成** | 本 crate |

---

## 证据

```
cargo test --workspace
  harness-contracts:               16 通过  (14 单元 + 2 示例)
  xr1-vision:                      69 通过  (63 核心 + 4 注册表 + 2 grounding)
  yellow-block-pick-place:          5 通过  (3 探测器回归 + 2 skill)
  ───────────────────────────────────────────
  合计:                            90 通过, 0 失败
cargo clippy --workspace:          干净
xr1-vision packs:                  ["yellow_block.pick_place"]
```

探测器的实测行为被精确保留——同样的三个回归帧（`20260818-…`）仍然守着它，只是现在从
pack 内部。

---

## 仍受硬件阻塞 / 仍开放

相比第二步不变，减去问题 #3：

- [ ] 拆分四个大编排文件（问题 #4）—— 纯软件，下一个候选
- [ ] `harness-core` / `harness-executive` crate 拆分（现在端口有了真实实现者，接缝由使用划定）
- [ ] `doctor → commission → verify` 状态机（需要实时发现）
- [ ] ARM64 `.deb` 打包 + 根级 CI（需要目标机）
- [ ] 用实时 task executive 取代 replay（需要机器人）
- [ ] Episode schema、judge 注册表、shadow / canary / rollback（问题 #5）
- [ ] 三台相同机器人"只换 Profile"验证

---

## 为什么是这个形状

探测器搬迁是*重定位*，不是重写：数学和它的实测测试一起搬，所以感知行为可证明地不变，
而它的归属搬到了第一步所说它该在的地方。注册表间接化是移除核心里硬编码对象名的最小改
动——一次查询、一个 boxed 端口，任何地方都没有新增对象专属的 match 分支。新增下一个对
象现在是一个 pack，恰如改造顺序所要求。
