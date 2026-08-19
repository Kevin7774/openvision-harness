# Vision Harness — 第四步 编排拆分

[English](harness-step-4-orchestration.md) | **中文**

第三步（[task pack](harness-step-3-taskpack.zh.md)）把黄色方块逻辑移出核心。第四步关闭
**问题 #4**：编排文件已变成新的大文件中心，其中"CLI 参数解析、JSON 解析、证据存储、锁
和适配器子进程协议"没有分开。

> [!NOTE]
> 仍无硬件。这一步搬迁并去重代码；没有改动任何循环逻辑、门或边界。所有现有测试仍然通
> 过，CLI 可观测的错误信息逐字节相同。

---

## 拆分实际发现了什么

问题比"文件太长"更严重。同一批 helper 被**复制进三个文件**，其中两份副本已经悄悄分岔：

| Helper | `cli.rs` | `servo_loop.rs` | `grasp_loop.rs` | 副本一致？ |
|---|:--:|:--:|:--:|---|
| `option` / `optional_option` / `flag` | ✔ | ✔ | ✔ | 是，逐字节 |
| `json_string` | ✔ | ✔ | ✔ | 是 |
| `parse_last_json` | ✔ | ✔ | ✔ | 是（仅差一个 turbofish） |
| `validate_command_args` | ✔ | ✔ | ✔ | 仅错误标签不同 |
| `read_json` / `read_json_file` | ✔ | — | ✔ | 是 |
| `write_json` | — | ✔ | ✔ | **否 —— 语义不同** |
| `try_lock_exclusive` + 锁类型 | — | ✔ | ✔ | **否 —— 已分岔** |

两个必须明说的发现：

**1. `write_json` 是同名的两个不同函数。** servo 那份覆盖写（`fs::write`）；grasp 那份
用 `create_new` 加 `sync_data`，也就是*拒绝*覆盖并 fsync。把它们合并会悄悄改变持久性
语义。现在它们是 [`evidence::write_json`]（本就该移动的指针）和
[`evidence::create_new_json`]（绝不可被静默替换的记录）——用不同的名字说明你得到哪种
保证。

**2. 两个循环通过两份实现共用同一个锁文件。** 两者都打开
`xr1-robot-action-loop.lock`——这是刻意的，因为同一时刻只能有一个循环指挥机器人。但
`ServoLoopLock` 会报告持有者并保留 OS 错误，而 `GraspLoopLock` 两者都丢弃。这是一个安
全关键的互斥原语，却在两个地方各自维护。现在它是一个类型
`RobotActionLoopLock`，保留两者中信息更充分的行为。

---

## 结构

```
crates/xr1-vision/src/support/
├── args.rs        # option / flag / 有界数值选项 / 参数校验
├── adapter.rs     # Python 适配器的 stdout-JSON 协议
├── evidence.rs    # 写 / 创建新文件 / 追加 JSONL / 读
└── runlock.rs     # 唯一的 robot-action-loop 锁
```

每个消费者现在导入它们，而不再各自携带副本：

```rust
// servo_loop.rs
use crate::support::adapter::{json_string, parse_last_json};
use crate::support::args::{f64_option, flag, option, optional_option, usize_option};
use crate::support::evidence::{append_json_line, write_json};
use crate::support::runlock::RobotActionLoopLock;
```

每个循环的错误标签只存在于各文件一个薄包装里，所以
`unsupported servo-loop argument "--oops"` 与
`unsupported grasp-loop argument "--oops"` 被精确保留。

---

## 数字

| 文件 | 之前 | 之后 | Δ |
|---|--:|--:|--:|
| `servo_loop.rs` | 990 | 827 | −163 |
| `grasp_loop.rs` | 871 | 748 | −123 |
| `cli.rs` | 564 | 501 | −63 |
| `visual_servo.rs` | 936 | 936 | 0 —— 见下 |
| `support/`（新增） | — | 496 | +496 |

三个消费者的 `git diff --stat`：**46 行新增，386 行删除。**

### 为什么没动 `visual_servo.rs`

第一步把它列为 936 行。读过之后：它是**599 行内聚的 servo 领域逻辑**
（`measure_jacobian`、`reconcile`、`propose` 及其校验器）加上**337 行测试**。它不含 CLI
解析、不含锁、不含证据存储、不含适配器协议——问题 #4 点名的五个关注点它一个都没有。为
了凑行数指标去拆它只是制造 churn，所以没拆。行数是另外三个文件的*症状*，混合关注点才
是病。

---

## 证据

```
cargo test --workspace
  harness-contracts:              16 通过
  xr1-vision:                     82 通过   (69 + 14 support − 1 个搬走的重复)
  yellow-block-pick-place:         5 通过
  ──────────────────────────────────────────
  合计:                          103 通过, 0 失败
cargo clippy --workspace --all-targets:   干净
```

support 模块按自己的标准被测试，包括旧副本未被守护的那些行为：

- `write_json` 覆盖，`create_new_json` 拒绝覆盖——在同一个测试里断言。
- `RobotActionLoopLock` 拒绝第二次获取**并指出持有者**；释放后下一个循环可以进入。这是
  真实的争用测试，不是 mock。
- 真值但非布尔的 `"ok"`（`1`、`"true"`）被 `require_ok` 拒绝。
- 形如选项的选项值（`--calibration --go`）被拒绝。

针对已构建二进制验证的可观测 CLI 行为：

```
$ xr1-vision servo-loop --oops
ERROR: unsupported servo-loop argument "--oops"
$ xr1-vision grasp-loop --oops
ERROR: unsupported grasp-loop argument "--oops"
$ xr1-vision d405-observe --oops
ERROR: unsupported argument "--oops"
```

---

## 关闭了第一步的哪些条目

| 第一步条目 | 状态 | 证据 |
|---|---|---|
| #4 CLI 解析未分离 | **关闭** | `support/args.rs`，删除三份副本 |
| #4 JSON / 适配器协议未分离 | **关闭** | `support/adapter.rs` |
| #4 证据存储未分离 | **关闭** | `support/evidence.rs`，两种语义分开命名 |
| #4 锁未分离 | **关闭** | `support/runlock.rs`，一个锁类型 |
| 改造第 5 步"拆分配置解析 / 证据存储 / 锁 / 适配器协议" | **完成** | 本模块 |

---

## 仍受硬件阻塞 / 仍开放

- [ ] `harness-core` / `harness-executive` crate 拆分（机械性；端口现在有真实实现者可据以划定接缝）
- [ ] `doctor → commission → verify` 状态机（需要实时发现）
- [ ] ARM64 `.deb` 打包 + 根级 CI（需要目标机）
- [ ] 用实时 task executive 取代 replay（需要机器人）
- [ ] Episode schema、judge 注册表、shadow / canary / rollback（问题 #5）
- [ ] 三台相同机器人"只换 Profile"验证

问题 #1–#4 现在已在软件层关闭。**问题 #5（自进化）是剩下唯一以软件为主的一项**——
episode schema、judge 注册表和晋升/回滚状态机可以在没有机器人的情况下构建并测试，尽管
*填充*它们需要机器人。
