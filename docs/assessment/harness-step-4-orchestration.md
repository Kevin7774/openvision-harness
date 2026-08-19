# Vision Harness — Step 4 Orchestration Split

**English** | [中文](harness-step-4-orchestration.zh.md)

Step 3 ([task pack](harness-step-3-taskpack.md)) moved the yellow-block logic out
of the core. Step 4 closes **finding #4**: the orchestration files had become the
new monoliths, with "CLI argument parsing, JSON parsing, evidence storage,
locking, and the adapter subprocess protocol" not separated.

> [!NOTE]
> Still no hardware. This step moves and de-duplicates code; no loop logic, gate,
> or bound was changed. Every existing test still passes, and the CLI's observable
> error messages are byte-identical.

---

## What the split actually found

The problem was worse than "files are long." The same helpers were **copied into
three files**, and two of the copies had silently diverged:

| Helper | `cli.rs` | `servo_loop.rs` | `grasp_loop.rs` | Copies identical? |
|---|:--:|:--:|:--:|---|
| `option` / `optional_option` / `flag` | ✔ | ✔ | ✔ | yes, byte-for-byte |
| `json_string` | ✔ | ✔ | ✔ | yes |
| `parse_last_json` | ✔ | ✔ | ✔ | yes (modulo a turbofish) |
| `validate_command_args` | ✔ | ✔ | ✔ | only the error label differed |
| `read_json` / `read_json_file` | ✔ | — | ✔ | yes |
| `write_json` | — | ✔ | ✔ | **no — different semantics** |
| `try_lock_exclusive` + lock type | — | ✔ | ✔ | **no — divergent** |

Two findings worth stating plainly:

**1. `write_json` was two different functions with one name.** The servo copy
overwrote (`fs::write`); the grasp copy used `create_new` plus `sync_data`, so it
*refused* to overwrite and fsynced. Merging them would have silently changed
durability. They are now [`evidence::write_json`] (a pointer that is meant to
move) and [`evidence::create_new_json`] (a record that must never be silently
replaced) — separate names that say which guarantee you get.

**2. The two loops shared one lock file through two implementations.** Both opened
`xr1-robot-action-loop.lock` — deliberately, so only one loop may command the
robot at a time. But `ServoLoopLock` reported the holder and preserved the OS
error, while `GraspLoopLock` discarded both. That is a safety-critical mutual
exclusion primitive maintained in two places. It is now one type,
`RobotActionLoopLock`, keeping the more informative behaviour of the two.

---

## Structure

```
crates/xr1-vision/src/support/
├── args.rs        # option / flag / bounded numeric options / arg validation
├── adapter.rs     # the Python-adapter stdout-JSON protocol
├── evidence.rs    # write / create-new / append-JSONL / read
└── runlock.rs     # the one robot-action-loop lock
```

Each consumer now imports these instead of carrying its own copy:

```rust
// servo_loop.rs
use crate::support::adapter::{json_string, parse_last_json};
use crate::support::args::{f64_option, flag, option, optional_option, usize_option};
use crate::support::evidence::{append_json_line, write_json};
use crate::support::runlock::RobotActionLoopLock;
```

The per-loop error label lives in one thin wrapper per file, so
`unsupported servo-loop argument "--oops"` and
`unsupported grasp-loop argument "--oops"` are preserved exactly.

---

## Numbers

| File | Before | After | Δ |
|---|--:|--:|--:|
| `servo_loop.rs` | 990 | 827 | −163 |
| `grasp_loop.rs` | 871 | 748 | −123 |
| `cli.rs` | 564 | 501 | −63 |
| `visual_servo.rs` | 936 | 936 | 0 — see below |
| `support/` (new) | — | 496 | +496 |

`git diff --stat` over the three consumers: **46 insertions, 386 deletions.**

### Why `visual_servo.rs` was left alone

Step 1 listed it at 936 lines. Read, it is **599 lines of cohesive servo domain
logic** (`measure_jacobian`, `reconcile`, `propose`, and their validators) plus
**337 lines of tests**. It contains no CLI parsing, no locking, no evidence
storage, and no adapter protocol — none of the five concerns finding #4 named.
Splitting it would be churn to hit a line-count target, so it was not split. Line
count was the symptom in the other three files; mixed concerns were the disease.

---

## Proof

```
cargo test --workspace
  harness-contracts:              16 passed
  xr1-vision:                     82 passed   (69 + 14 support − 1 duplicate moved)
  yellow-block-pick-place:         5 passed
  ──────────────────────────────────────────
  total:                         103 passed, 0 failed
cargo clippy --workspace --all-targets:   clean
```

The support modules are tested on their own terms, including the behaviours the
old duplicates left unguarded:

- `write_json` overwrites, `create_new_json` refuses to — asserted in one test.
- `RobotActionLoopLock` refuses a second acquisition **and names the holder**;
  releasing it lets the next loop in. This is a real contention test, not a mock.
- A truthy-but-non-boolean `"ok"` (`1`, `"true"`) is rejected by `require_ok`.
- An option value that looks like an option (`--calibration --go`) is rejected.

Observable CLI behaviour, verified against the built binary:

```
$ xr1-vision servo-loop --oops
ERROR: unsupported servo-loop argument "--oops"
$ xr1-vision grasp-loop --oops
ERROR: unsupported grasp-loop argument "--oops"
$ xr1-vision d405-observe --oops
ERROR: unsupported argument "--oops"
```

---

## Which Step 1 items this closes

| Step 1 item | Status | Evidence |
|---|---|---|
| #4 CLI parsing not separated | **Closed** | `support/args.rs`, three copies removed |
| #4 JSON / adapter protocol not separated | **Closed** | `support/adapter.rs` |
| #4 evidence storage not separated | **Closed** | `support/evidence.rs`, two semantics named apart |
| #4 locking not separated | **Closed** | `support/runlock.rs`, one lock type |
| Rebuild step 5 "split config parsing / evidence / locking / adapter protocol" | **Done** | this module |

---

## Still hardware-gated / still open

- [ ] `harness-core` / `harness-executive` crate split (mechanical; the ports now have a real implementor to draw seams from)
- [ ] `doctor → commission → verify` state machine (needs live discovery)
- [ ] ARM64 `.deb` packaging + root-level CI (needs the target)
- [ ] Live task executive instead of replay (needs the robot)
- [ ] Episode schema, judge registry, shadow / canary / rollback (finding #5)
- [ ] Three-identical-robot "swap only the Profile" validation

Findings #1–#4 are now closed in software. **Finding #5 (self-evolution) is the
only one left that is mostly software** — episode schema, judge registry, and the
promotion/rollback state machine can be built and tested without a robot, even
though *populating* them requires one.
