# Vision Harness — Step 1 Verdict

**English** | [中文](harness-step-1-verdict.zh.md)

A grounded assessment of the current `thor-workspace-live` harness against one
acceptance standard: *download onto any robot, immediately use it, complete
arbitrary tasks, and self-improve.* This is the "conclusion first" step. It is
modelled on the [`deepseek-harness`](https://github.com/deepseek-ai/deepseek-harness)
documentation style — verdict up front, evidence in tables, honest about what is
not done.

> [!WARNING]
> **DEVELOPER PREVIEW.** This is an assessment, not a release. The verdict below
> is deliberately harsh. Every score is a claim about *what is proven by code and
> measured evidence in this repository today* — not about intent, design notes,
> or what a future refactor could achieve. THIS DOCUMENT WILL CHANGE as the
> harness is restructured. Do not cite a score without re-reading the cited file.

---

## Verdict

Against the "download-and-use on any robot, self-improving" standard, **this
harness does not pass.**

A more accurate definition of what exists today:

> A **deterministic experiment-execution kernel** built for one specific XR1, one
> specific ROS environment, one specific tabletop, one specific pair of cameras,
> and one yellow-block experiment.

It is not a cross-robot product, and it is not yet a self-evolving system.
Copying it onto a second robot — **even an identical XR1** — can produce the most
dangerous failure mode there is: **software emits correct-looking output while the
physical result is wrong**, because the numbers baked into the core were measured
on the *first* robot's hardware and workstation.

The `dsh`-equivalent here is the `xr1-vision` binary. It runs. It records, replays,
plans, and gates motion behind real safety envelopes. That is genuine and it is
tested. It is also the whole of what is proven.

---

## Scores

Scoring is against **single-XR1 experimental value**, then against the two claims
in the acceptance standard (portability, self-evolution).

| Dimension | Score | Primary reason |
|---|---|---|
| Single-XR1 experimental value | **5 / 10** | Records, replays, and plans, but the critical hardware chain (D405, tactile, live servo Jacobian) is implemented, not yet live-verified. |
| Code modularity | **5 / 10** | Directory layering exists, but there is no hardware-port abstraction; orchestration files are turning into new monoliths. |
| Portability across robots | **1 / 10** | Single-machine constants sit in the *core*. No `RobotProfile`, no `CalibrationManifest`, no staleness binding. |
| Self-evolution | **0.5 / 10** | Concept and evidence-keeping only. No episode schema, judge registry, training, promotion, or rollback in code. |

**65 Rust tests pass.** That proves only that the encoded rules have not been
broken by the current tests. It does **not** prove the harness is portable,
installable, or capable of self-evolution. (Verify:
`grep -rc "#\[test\]" crates/xr1-vision/src`.)

---

## The five most serious problems

### 1. The "generic interface" is naming, not a replaceable boundary

There is **no hardware-port trait** in the Rust source. Every module is exposed
flat from [`crates/xr1-vision/src/lib.rs`](../../crates/xr1-vision/src/lib.rs):

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

The boundaries that must actually be swappable per robot were never extracted:

```rust
trait ObservationSource      // where do frames/state come from
trait MotionExecutor         // who moves the arm, under what envelope
trait KinematicsValidator    // IK + reachability for this URDF
trait OutcomeJudge           // did the physical result match the predicate
trait TaskSkill              // one pluggable capability
```

Today the "interface" is expressed through **Python filenames, environment
variables, and JSON passed over subprocess boundaries** (`py/`, `servo_adapter.py`,
`grip_adapter.py`). That runs. It cannot be reliably substituted for a different
robot's implementation.

### 2. Single-machine binding is severe, and it lives in the core

[`crates/xr1-vision/src/kinematics/types.rs`](../../crates/xr1-vision/src/kinematics/types.rs)
pins the current gripper, fingertip, and table geometry as compile-time constants:

```rust
pub const TIP_CENTER_M: [f64; 3] = [-0.0225, 0.0, 0.0485];
pub const OPEN_JAW_GAP_M: f64 = 0.0465;
pub const PLANNING_MIN_TIP_Z_M: f64 = 0.785;   // <- this is a table height
```

These are not software defaults. They are **measured facts about one robot and
one workstation.** `PLANNING_MIN_TIP_Z_M = 0.785` is a floor for *that table*.
Ship it to a robot at a different table and the geometry gate silently lies. None
of these belong in a generic core; they belong in a per-robot profile and
calibration manifest.

### 3. The task interface looks semantic but is very narrow

[`crates/xr1-vision/src/proposal.rs`](../../crates/xr1-vision/src/proposal.rs)
exposes exactly:

```rust
pub enum Task     { Grasp, PickPlace }
pub enum GraspIntent { TopDown }
```

And `object_id` is a **label, not pluggable grounding**. Whatever string you put
in, `grasp_request()` rejects everything except one hard-coded value
(`proposal.rs:208`):

```rust
if object_id != "yellow_block" {
    return Err(format!(
        "object_id {object_id:?} is not supported by the current measured detector"
    ));
}
```

Underneath, [`perception/mod.rs`](../../crates/xr1-vision/src/perception/mod.rs)
always runs the yellow reconstruction, and
[`perception/yellow.rs`](../../crates/xr1-vision/src/perception/yellow.rs) uses
colour thresholds **measured on specific experiment frames** (`yellow.rs:59`):

```rust
// Thresholds measured on frames 20260818-112803 and 20260818-170043.
// The two-sided R/G window rejects both the green cube and orange pads.
if (20..=5000).contains(&area)
    && sum_red >= 0.85 * sum_green
    && sum_red <= 1.15 * sum_green
    && mean_chroma >= 10.0
```

So the "semantic" object query resolves to one detector tuned to one lighting
condition. It is honest — it fails closed on anything else — but it is not a
generic task interface.

### 4. Orchestration files are becoming the new monoliths

| File | Lines |
|---|---|
| [`servo_loop.rs`](../../crates/xr1-vision/src/servo_loop.rs) | 990 |
| [`visual_servo.rs`](../../crates/xr1-vision/src/visual_servo.rs) | 936 |
| [`grasp_loop.rs`](../../crates/xr1-vision/src/grasp_loop.rs) | 871 |
| [`cli.rs`](../../crates/xr1-vision/src/cli.rs) | 555 |

Inside each, **CLI argument parsing, JSON parsing, evidence storage, locking, and
the adapter subprocess protocol are not separated.** A boundary that mixes
transport, policy, and payload cannot be a stable port for another robot.

### 5. "Self-evolution" is a design concept, not code

[`docs/decisions/0005-automatic-reset-is-the-ceiling.md`](../decisions/0005-automatic-reset-is-the-ceiling.md)
reasons carefully about judges, golden sets, and automatic reset — including the
arithmetic that the judge must be an order of magnitude better than the policy.
That thinking is real. **None of the following exists in code:**

- [ ] Episode data standard (immutable observation / action / outcome)
- [ ] Outcome-judge registry (with `abstain`)
- [ ] Training pipeline
- [ ] Policy / model registry
- [ ] baseline / challenger comparison
- [ ] Shadow evaluation
- [ ] Automatic promotion criteria
- [ ] Canary deployment
- [ ] Rollback
- [ ] Cross-robot data isolation
- [ ] Guard against mislabels forming a positive feedback loop

Therefore it cannot be called "self-evolving." At most it is *"some evidence has
been preserved for future training."*

---

## What "download and use" should actually mean

Not "skip calibration and move." It should mean a **commissioning state machine**
that refuses to move until the robot is known and verified:

```
download / install
      ↓
auto-discover hardware + ROS capabilities   (read-only)
      ↓
identify robot model + software compatibility
      ↓
load that robot's Profile
      ↓
guided calibration
      ↓
run acceptance tests
      ↓
activate task execution
```

Even for an identical XR1, the profile and calibration are what differ — not the
source.

---

## Target architecture

```
                    Task Executive (semantic → TaskSpec → skill)
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

Proposed final repository shape (the current single crate splits into a
contracts-first workspace, and platform/task/profile data leave the core):

```
harness/
├── crates/
│   ├── harness-contracts/     # the five traits + schemas, no hardware
│   ├── harness-core/          # planning, geometry, safety envelopes
│   ├── harness-executive/     # semantic → TaskSpec → skill dispatch
│   ├── harness-evaluation/    # episode, judge, registry, promotion
│   └── harness-cli/           # doctor / commission / verify / task
├── platforms/
│   └── xr1-thor/
│       ├── ros-adapters/
│       ├── moveit-bridge/
│       └── platform.toml
├── task-packs/
│   └── yellow-block-pick-place/   # <- yellow detector + block semantics move HERE
├── profiles/
│   └── examples/
├── schemas/
├── packaging/                 # ARM64 .deb bundles
└── tests/
    ├── contracts/
    ├── replay/
    └── commissioning/
```

The yellow detector, yellow-block semantics, and table constants must move into a
**task pack** or a **RobotProfile**. They must not remain in the core capability.

---

## Profile + calibration model

A `platform.toml` describes the robot; calibration is a separate, staleness-bound
manifest.

```toml
# platform.toml (sketch)
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

Every calibration file **must** bind:

`robot_id` · `sensor serial` · `tool serial` · `URDF hash` · `station_id` ·
`measurement time` · valid pose range · sample count · error metric.

Without this, an old calibration copied to another machine cannot be detected as
already-invalid — which is exactly failure mode #2 above, made mobile.

---

## Installation

On XR1 / Thor, do **not** default to shipping the whole runtime in Docker. USB,
DDS, GPU, RealSense, ROS overlay, and vendor SDK make containers *harder*, not
easier. Prefer publishing **two ARM64 Debian packages** (a base bundle and a
platform bundle).

CLI surface, mapped to the commissioning state machine:

```bash
harness doctor                                        # read-only discovery + versions
harness commission --platform xr1-thor --robot-id xr1-004   # bind devices, guide calibration
harness verify                                        # replay + IK + sensors + frames + dry-run acceptance
harness task "把黄色方块放进绿色托盘"                    # NL → TaskSpec → task pack executes
```

- `doctor` — read-only capability and version discovery.
- `commission` — bind devices and guide calibration.
- `verify` — replay, IK, sensor, coordinate, and dry-run acceptance.
- `task` — natural language → `TaskSpec` → task pack.

Without root, do not promise automatic install of udev rules, ROS packages, or
system services — offer at most a user-directory executable bundle.

---

## How self-evolution should actually work

Do **not** let an agent edit source on the robot. That is not self-evolution; it
is unauditable online development. The correct closed loop is offline-trained,
gated, and reversible:

```
execute Episode
      ↓
immutable Observation / Action / Outcome
      ↓
Judge + abstain
      ↓
candidate policy trained OFFLINE
      ↓
frozen-dataset replay
      ↓
golden-set evaluation
      ↓
Shadow mode
      ↓
small-fleet Canary
      ↓
promotion threshold met
      ↓
publish new Policy Artifact  (with rollback + release rules)
```

---

## Minimal, non-shotgun rebuild order

Do these in sequence. Do not attempt all boundaries at once.

1. **Scope v1 to "same XR1/Thor platform only."** Stop claiming arbitrary robots.
2. Introduce `RobotProfile` and `CalibrationManifest`; migrate hardcoded constants out one at a time.
3. Add ~five traits **only at genuinely replaceable boundaries** — not one interface per function.
4. Move the yellow-block logic into the first **task pack**.
5. Split the big loops into config-parsing / evidence-store / locking / adapter-protocol layers.
6. Add the `doctor → commission → verify` state machine.
7. Produce a reproducible ARM64 `.deb` and root-level CI.
8. Connect the **live** task executive, not just replay.
9. Implement episode schema, judge, policy registry, shadow / canary / rollback.
10. Finally, validate on **at least three identical robots**: swap only the Profile, change no source.

---

## Acceptance criteria before claiming "download-and-use"

At minimum, prove:

- A fresh factory baseline deploys with **one install command**.
- `doctor` discovers hardware/ROS without moving anything and reports version compatibility.
- `commission` binds devices and produces a staleness-bound `CalibrationManifest`.
- `verify` passes replay, IK, sensor, coordinate, and dry-run acceptance on that robot.
- A calibration copied from another robot is **detected as invalid** and blocks motion.
- The same source, with only a swapped Profile, runs on **three identical robots**.
- One task completes end-to-end through the live executive (not replay).
- The self-evolution loop produces a promoted policy artifact that can be **rolled back**.

---

## Scope of this document

This is **Step 1**: the conclusion. It fixes the honest definition of what exists,
the scores, the five failures, and the rebuild order. It does not implement any of
it. Steps 2+ execute the rebuild order above, one boundary at a time, each with
its own evidence.
