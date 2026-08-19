# Vision Harness — 第五步 评测与发布

[English](harness-step-5-evaluation.md) | **中文**

第四步（[编排拆分](harness-step-4-orchestration.zh.md)）关闭了最后一个结构性问题。第五
步处理**问题 #5**，也就是第一步给出 **0.5/10** 的那一项：自进化只作为
[ADR 0005](../decisions/0005-automatic-reset-is-the-ceiling.md) 里的推理存在，代码里什
么都没有。

> [!IMPORTANT]
> **这并没有让 harness 变成自进化，评分也不会变成 10。** 现在存在的是*晋升路径*：
> episode 标准、judge、黄金集、gate，以及 shadow/canary/rollback 生命周期。仍然**没有
> 训练器**，也没有来自真实机器人的 episode。诚实的说法是"决定一个新策略是否可以服务的
> 机制已实现并有测试"——而不是"机器人在自我改进"。

---

## 为什么这不是一条通用 MLOps 流水线

ADR 0005 异常具体，所以实现遵循它而不是套模板。它的四条陈述变成了可执行代码：

**1. "judge 必须比 policy 好一个数量级——这句话是算术，不是修辞。"** 在成功率 `p` 下，
系统性偏差为 `b` 的 judge 在 `n* = p(1-p)/b²` 之后不再提供信息。这就是
`information_ceiling`，测试直接断言 ADR 自己发布的那张表：

| judge 偏差 | ADR 给出 | 计算得到 |
|---|--:|--:|
| 5 pp | 84 | 84 |
| 2 pp | 525 | 525 |
| 1 pp | 2 100 | 2 100 |
| 0.5 pp | 8 400 | 8 400 |

它可运行，所以当操作者问"该不该再多收集 episode？"时能得到真实答案：

```
$ xr1-vision judge-ceiling --rate 0.30 --bias 0.05
{"ok":true,"episode_ceiling":84.0,"standard_error_at_ceiling":0.05, ...}
```

在 ceiling 处标准误恰好等于偏差——这正是定义本身，也是一个有用的自检。

**2. "两个通道加弃权：当它们不一致时，标为 `uncertain` 并排除在训练集之外。"**
`LayeredJudge` 把头部 ZED（场景）与腕部相机加 `pos_mm` 堵转（抓取瞬间）结合，不一致时
弃权。当某个通道*不可用*时同样弃权，因为一台坏掉的腕部相机绝不能被读成"没抓到"。

**3. "把弃权率作为一等指标追踪……突然下降通常意味着 judge 学会了自信地犯错。"** 这就是
错误标签正反馈的防护，也是唯一一处*看起来更好*的结果被当作取消资格的地方。
`AbstainMonitor` 返回 `SuspiciousDrop`，gate 把它变成 `Reject`——不是 `Hold`。

**4. "闭环的验收门是人工干预间隔时间，而不是成功率。目标是 MTBH ≥ 8 小时。"** 因此一个
成功率更高但 MTBH 只有 1.5 小时的挑战者会被 hold，理由写明
`success rate is not the gate`。

另外两条 ADR 约束塑造了 episode 记录本身：所有判定必须来自**同一个固定头部姿态**（否则
judge 偏差会漂移），而移动工位会**静默地**让标定失效。两者都成为 scope 检查，拒绝不可
比的 episode，而不是把它们平均进去。

---

## 结构

```
crates/harness-evaluation/
├── src/episode.rs     # 不可变 Episode + 只追加 EpisodeLog + FleetScope
├── src/judge.rs       # LayeredJudge（impl OutcomeJudge）、偏差算术、AbstainMonitor
├── src/golden.rs      # 冻结黄金集、泄漏检查、judge 打分
├── src/policy.rs      # PolicyArtifact + 带血缘的注册表
├── src/promotion.rs   # gate：baseline 对 challenger
├── src/lifecycle.rs   # Registered → Shadow → Canary → Promoted / RolledBack / Rejected
└── tests/loop_end_to_end.rs
```

`LayeredJudge` 实现了第二步引入的 `OutcomeJudge` 端口，所以 judge 接入的是契约，而不是
自造接口。

### gate 会拒绝什么

`evaluate()` 区分"还不到时候"和"这个比较无效"：

| 情况 | 结论 |
|---|---|
| 每臂少于 200 个已判定 episode | `Hold` |
| 边际 ≤ 2× judge 偏差 | `Hold`，点名 judge 及其 ceiling |
| 边际 ≤ 1.96σ 采样噪声 | `Hold` |
| MTBH 低于 8 小时 | `Hold` |
| 弃权率高于可用上限 | `Hold` |
| 弃权率跌破基线 | **`Reject`** |
| 挑战者并不更好 | `Reject` |
| 黄金集冻结于挑战者*之后* | `Reject` |
| 挑战者在黄金集上训练过 | `Reject` |

边际对偏差这一条值得特别指出：过了 `n*` 之后再多 episode 也没用，所以理由说的是
**judge** 必须改进，而不是要求更大的样本。这正是 ADR 0005 称为"这里最昂贵的失败模式"
的那种情况——闭环烧着机器人小时数，同时画出一条看起来完全正常的曲线。

这里刻意**没有 override 参数**。需要 override 的晋升是人的决定，在这个类型之外做出。

### 生命周期会拒绝什么

```
Registered --shadow--> Shadow --gate--> Canary --gate--> Promoted
                          |               |                |
                          +--- Rejected / RolledBack ------+
```

- Shadow 不能跳过；没有通过 shadow gate 的 canary 会被拒绝。
- 通过 shadow gate **不部署任何东西**——它换来的是一次 canary。两次 shadow 通过仍然是
  在位策略在服务。
- Canary 份额上限 25%：大到足以影响机群的 canary 就不是 canary。
- **回滚在任何存活阶段都可用，且从不设门。** 需要为自己辩护的回滚就是发生得太晚的回滚。
- `serving_policy_id()` 在晋升那一刻之前始终返回在位策略，所以"当前哪个策略在服务"永远
  不靠阶段名去推断。

---

## 证据

```
cargo test --workspace
  harness-contracts:              16 通过
  harness-evaluation:             52 通过   (48 单元 + 4 端到端)
  xr1-vision:                     82 通过
  yellow-block-pick-place:         5 通过
  ──────────────────────────────────────────
  合计:                          155 通过, 0 失败
cargo clippy --workspace --all-targets:   干净
```

端到端测试驱动的是整条路径而不是单个部件：每臂记录 1 200 个 episode，**通过
`OutcomeJudge` 端口**打标签，由 ledger 统计，由 gate 比较，再走完生命周期。其中三个断言
的是拒绝行为：

- 分数变好是**因为 judge 停止弃权**的挑战者被拒绝。
- canary 回归时无需 gate 即回滚到在位策略。
- 来自第二台机器人——或同一台机器人换到另一张工作台——的 episode 无法进入第一台机器人
  的评测。

---

## 关闭了第一步的哪些条目

| 第一步缺口 | 状态 | 位置 |
|---|---|---|
| Episode 数据标准 | **关闭** | `episode.rs` |
| 带 abstain 的 Outcome Judge 注册表 | **关闭** | `judge.rs`，基于 `OutcomeJudge` 端口 |
| Policy / 模型注册表 | **关闭** | `policy.rs` |
| baseline / challenger 对比 | **关闭** | `promotion.rs` |
| Shadow evaluation | **关闭** | `lifecycle.rs` |
| 自动晋升条件 | **关闭** | `PromotionCriteria` |
| Canary 部署 | **关闭** | `lifecycle.rs`，上限 25% |
| 回滚 | **关闭** | `lifecycle.rs`，无门 |
| 跨机器人数据隔离 | **关闭** | `FleetScope` |
| 防止错误标签正反馈 | **关闭** | `AbstainMonitor` + `GoldenSet` |
| **训练流水线** | **开放 —— 需要机器人** | 见下 |

---

## 仍然开放，且如实说明

- [ ] **训练器。** 这里没有模型、优化器或梯度。挑战者是某个离线流程产出的 artifact，本
      crate 只决定它能否服务。构建训练器需要只有机器人才能产生的 episode。
- [ ] **真实 episode。** 这些测试里的每个数字都是合成的。gate 从未见过真实机器人的数
      据，其阈值（每臂 200、2× 偏差、1.96σ）是可辩护的默认值，不是调过的值。
- [ ] **实测的 judge 偏差。** `JudgeQuality::bias` 必须来自对照人工真值打分的黄金集。打
      分机制已存在（`GoldenSet::score`），黄金集本身不存在。
- [ ] **自动复位**，也就是 ADR 0005 指出的真正天花板：没有它，每周约 3 100 个 episode、
      43% 占空比，而不是约 32 000 个、100%，并且每次尝试都要花掉一个人工分钟。gate 的每
      臂 200 个下限在自动复位下很便宜，没有它就很贵。
- [ ] `doctor → commission → verify` 状态机、ARM64 `.deb`、实时 executive、三机
      Profile 互换验证（与第四步相同）。

问题 #1–#5 现在都已在**软件层**关闭。剩下的不是架构问题：是一台机器人、一道围栏、地上
一条胶带标记，以及让这个闭环负担得起的自动复位。
