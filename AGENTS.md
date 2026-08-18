# Workspace contract

Read this before changing anything. It is the rule set, not a tutorial ---
the tutorial is [`README.md`](README.md).

## 1. Purpose

Make an XR1 humanoid pick up a block and place it, reliably enough that the
success rate is a number rather than an anecdote. Everything here exists to
serve that, or to prove whether a change helped.

## 2. Architecture

Strict one-way layering. A layer may call downward, never upward:

```
Agent      you, or an LLM        decides what to try next; writes a prediction first
  |
Planner    xr1-vision plan       perception -> footprint -> grasp candidates -> IK
  |
Executor   py/xr1.py             one named motion at a time, with a return code
  |
Safety     py/astra_arm.py       rate limit, URDF clamp, staleness, channel idle
  |
Device     /opt/ros/astrabot     vendor binaries + ros2_control (not ours)
```

**Safety is never an Agent decision.** Every limit lives in `astra_arm.py` or in
the IK/collision gates, is deterministic, and depends on no model, prompt, or
vision output. An agent may propose a pose; it may not widen a gate. Relaxing a
gate requires a human saying so in the transcript, and an ADR.

## 3. Language policy

Rust is the main line. Business logic --- perception, geometry, kinematics,
planning, the experiment journal --- goes in `crates/`.

Python stays, permanently, in exactly one role: the **ROS 2 / hardware
boundary**. `rclpy`, the vendor `astra_arm` SDK and `pyzed` have no usable Rust
bindings on this platform, and re-implementing the DDS + URDF + trajectory path
would move risk, not remove it. `py/` is thin, is not where decisions live, and
new logic does not go there.
See [ADR 0002](docs/decisions/0002-python-at-the-ros-boundary.md).

C++ stays in `ros/rtc_teleop/` because those are ROS 2 nodes built by colcon.

Swift stays in `mac/xr1rec.swift` because it is an AVFoundation recorder that
runs on the Mac, not on the robot: the camera and its TCC permission prompt are
macOS APIs. It is driven from here over ssh by `py/xr1_cam.py`.

## 4. Rust rules

- One capability, one implementation, one place. If you are about to write a
  second definition of "yellow" or a second forward-kinematics chain, delete the
  first one instead.
- No `foo_v2.rs`, no commented-out code, no module that exists so something can
  be plugged into it later.
- A constant that came from a tape measure or a camera frame carries the
  measurement in a comment next to it. A constant with no provenance is a bug
  waiting to be argued about.
- `unwrap`/`expect`/`panic!` are not allowed on any path that can be reached
  with the robot powered on. Return `Result<_, String>` and let the CLI print it.

## 5. Testing

`cargo test --workspace` must pass, and every non-trivial rule must have one
test that fails if the rule breaks. The tests here are unusual on purpose: they
assert against **numbers measured off named hardware frames**, so a threshold
someone "cleans up" fails immediately. Do not replace them with synthetic
fixtures.

There is no way to unit-test the robot. Hardware claims are backed by an entry
in `data/` with a timestamp, not by a test.

## 6. Evidence

`data/` is append-only ground truth: `vista_runs/` observations, `experiments/`
reports, `snapshots/`. It is **tracked in git**, binary frames included --- see
the comment in `.gitignore` for why, and for the one way to prune it.

Never rewrite an old record to match a new belief: add a new record and, if the
old one was wrong, say so in the new one. Mechanically repointing stored absolute
paths after a directory move is the one allowed edit, and it belongs in the commit
message. A conclusion with no dated observation behind it is a guess, and must be
written as one.

Cross-session coordinates **must carry a timestamp**. Two sessions once spent an
afternoon reconciling "contradictory" IK results that were simply measured
before and after somebody nudged the block 67 mm.

## 7. Docs

Four directories, and that is all:

| Path | Holds |
|---|---|
| `docs/architecture/` | how the system is built, as it is now |
| `docs/operations/` | how to run it, and how it fails |
| `docs/development/` | how to build, test and lint it |
| `docs/decisions/` | why it is this way (ADRs, numbered, immutable) |

Docs explain code; code is the truth. A doc that disagrees with the code gets
rewritten or deleted --- never annotated with "note: actually". No history
narratives, no changelog prose: that is what `git log` is.

## 8. Deletion

Delete wrong, duplicate, unreferenced and superseded things, and say so in the
commit message. Keeping something "just in case" is how this workspace ended up
with ~9,600 lines of documentation describing ~95 scripts that did not exist.

Two things you may not delete: a **capability** that still works, and a
**measurement** that cannot be retaken cheaply. If you must remove code that
holds either, move the knowledge into `docs/decisions/` in the same commit.

## 9. Naming

Crates `xr1-*`. Files and modules `snake_case`, named for what they *are*
(`perception.rs`), never for their status (`new`, `fixed`, `old`, `v2`).
Commands are verbs: `observe`, `plan`, `grip`, `home`.

## 10. Agent workflow

One action at a time, and observe again afterwards --- the world moved, possibly
because a different session moved it.

1. Observe. Look at the actual image before reading any aggregate number; an
   aggregate can only confirm a hypothesis you already got right.
2. Write the prediction down *before* acting, as a number.
3. Execute exactly one action.
4. Observe again and reconcile prediction against reality.
5. If they disagree, the model is wrong --- fix the model, not the gate.

Before touching hardware: `export ROS_DOMAIN_ID=12`, read
`docs/operations/status.md`, and `pgrep -af` before any `pkill`. Restarting a
service and claiming a device are visible to every other session on this
machine.
