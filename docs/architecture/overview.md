# Architecture

## Who owns the robot

Almost nothing here talks to a motor. The XR1's actual control stack is a
**binary-only vendor overlay** at `/opt/ros/astrabot` (33 ELF binaries, 0 Python
files) driven by ~20 `Astrabot_*.service` units. Those units self-heal in ~100 ms,
so `pkill` is not "stop" --- it is "restart".

Two consequences shape everything else:

- **The effective robot model is not on disk.** Editing a `.urdf` file changes
  nothing at runtime. Read the live one:
  `ros2 param get /robot_state_publisher robot_description`.
- **We are a guest.** Claiming a serial port, restarting a service or moving an
  arm is visible to every other session on this machine.

## Layers

```
Agent      you / an LLM          emit a semantic TaskProposal v2
  |
Executive  Rust task module      evidence-driven stage transitions and replay
  |
Planner    bin/xr1 plan          image -> object pose -> grasp candidates -> IK
  |
Guard      xr1-vision safety     freshness, step, URDF path, required sensors
  |
Executor   py/xr1.py             one named motion, one return code
  |
Safety     py/astra_arm.py       live staleness, rate, clamp, channel idle
  |
Device     /opt/ros/astrabot     ros2_control + vendor binaries
```

Calls only ever go downward. The Planner never publishes to a command topic; the
Executor never decides *where* to reach; the Safety layer never asks anything
above it for permission.

## The Rust side: `crates/xr1-vision`

One reusable library with a thin binary entry point. Module dependencies point
from orchestration toward domain modules; perception does not call planning and
planning does not publish to hardware.

| Module | Responsibility |
|---|---|
| `proposal.rs` | natural-language task, target/destination relations and success predicates; no Cartesian or joint pose fields |
| `task/` | deterministic observe/plan/act/verify/diagnose state transitions and replay |
| `perception/` | read one observation and produce object geometry plus physical pad/target servo signals |
| `plan_cache.rs` | bind production plans to exact inputs and publish one immutable attempt per state |
| `planning/` | full-circle roll search, typed `GraspCandidate` ranking and MoveIt collision validation |
| `kinematics/model.rs` | URDF chain, FK, gripper-pad geometry and shared motion envelope |
| `kinematics/ik.rs` | multi-seed numerical IK and duplicate-branch removal |
| `kinematics/grasp.rs` | contact and closing-axis feasibility |
| `visual_servo.rs` | fit a local 3×3 image Jacobian, propose one bounded step and reconcile its observed result |
| `servo_loop.rs` | bounded observe/one-step/reobserve orchestration, session lock and append-only evidence |
| `grasp_feedback.rs` | calibrated two-pad baseline, contact, balance, pressure-ceiling and retention decisions |
| `grasp_loop.rs` | D405 alignment plus one jaw increment per fresh pressure observation |
| `safety.rs` | bind a proposal to fresh evidence and apply deterministic step, URDF margin, floor-path and sensor-capability gates |
| `observation.rs` | unified ZED artifact, joint, TF, gripper and sensor-capability bundle |
| `hardware.rs` | read-only D405 and tactile capability discovery; no task policy |
| `cli.rs` | command dispatch and stable JSON boundaries |
| `runtime.rs` | workspace paths and the Python process boundary; `bin/xr1` sources ROS base once, then adapters source only the robot overlay with FastDDS UDPv4 |
| `experiment.rs` | experiment lifecycle and report persistence |
| `main.rs` | process exit code only |

Commands:

```
bin/xr1 preflight                       py/xr1.py pose
bin/xr1 observe                         py/vista_observe.py -> data/vista_runs/<run>/latest.json
bin/xr1 bundle [--latest FILE]          unified immutable observation contract
bin/xr1 validate-proposal --proposal P  validate/upgrade TaskProposal v2
bin/xr1 plan [--proposal P] [--latest L]
                                              saved observation -> geometry -> candidates
bin/xr1 replay --proposal P --events E  deterministic task-state replay; no motion
bin/xr1 fk J1 .. JN                     pad-inner points, midpoint and tool rotation
bin/xr1 sensor-status                   read-only D405 / tactile capability state
bin/xr1 d405-observe                    real bounded D405 frame + Rust near-field signal
bin/xr1 tactile-observe --config C      real two-pad pressure sample; no motion
bin/xr1 tactile-assess ...              capture/read two pads + deterministic assessment
bin/xr1 servo-pads --frame DIR          shared Rust physical-pad detector
bin/xr1 servo-observe [--latest L]      physical pad + pinned target signal
bin/xr1 servo-calibrate --input I       +/- observations -> local 3x3 Jacobian
bin/xr1 servo-propose --input I --state S
                                              bounded proposal + deterministic Rust gate
bin/xr1 servo-step --proposal P [--go]  dry-run or exactly one approved microstep
bin/xr1 servo-reconcile --input I       prediction vs distinct newer observation
bin/xr1 servo-loop --calibration C [--go]
                                              bounded observe/action/reconcile orchestration
bin/xr1 grasp-loop ... [--go]           D405/tactile bounded jaw closure
bin/xr1 begin --purpose TEXT            open a numbered experiment
bin/xr1 note --section NAME --text TEXT  append to its report
bin/xr1 grip --side S --state open|close
bin/xr1 ready --side right               MOVE right arm to measured planning pose
bin/xr1 motion --attempt A --phase P [--go]
                                        dry-run/execute one immutable plan phase
bin/xr1 end --status SUCCESS|FAILED
bin/xr1 status
```

`observe`, `bundle` and `plan` are deliberately separate: `observe` touches
hardware and `plan` is pure computation over files. Production planning is
executed once per exact input state and published atomically under
`data/attempts/`; the command returns a short receipt and later inspection reads
that immutable attempt. A changed robot
state or model invalidates the key and requires a fresh observation and plan.
`plan` moves nothing; the saved plan is labelled `online_plan_dry_run`, and
execution is a separate, human decision through the attempt-only adapter.
`servo-propose` is also non-executing: `ready_for_execution_adapter` only means
the Rust checks passed. `servo-step` requires a separate `--go`, then the Python
boundary re-checks envelope age, live joint freshness, start drift and channel
idleness. It executes at most one microstep and requires a newer observation
before `servo-reconcile` can authorize continued reasoning.
`servo-loop` composes those same transactions with one process lock, a six-step
default ceiling and an overall deadline. It uses existing ROS publishers only;
it contains no service start, stop or restart path. Dry-run is the default.

The semantic proposal and typed candidate schemas are specified in
[`proposals.md`](./proposals.md).

## The Python side: `py/`

| File | Role |
|---|---|
| `xr1.py` | the robot API: `bringup`, `pose`, `look`, `grip`, `home`, `ready`, `wave`, `demo`, `rec`, `snap` |
| `astra_arm.py` | the safety layer wrapping the vendor SDK (see below) |
| `vista_observe.py` | read-only ZED snapshot: RGB, aligned depth, intrinsics, image-time TF, joints and optional gripper readings |
| `xr1_cam.py` | manually drive the optional external recorder on the Mac at 192.168.123.138; never a harness gate |
| `pad_offset_measure.py` | measure the gripper-pad pixel offset against `bin/xr1 fk` |
| `motion_adapter.py` | accept one immutable Rust attempt, select a fully feasible candidate and execute one phase |
| `servo_adapter.py` | consume one approved Rust envelope and publish at most one joint microstep |
| `mac/` | the recorder itself (Swift), installed on the Mac |

`astra_arm.py` is the only thing standing between a bad number and the hardware.
It ramps from the *measured* pose rather than the last commanded one, caps
per-joint velocity, clamps against the live URDF limits, holds joints nobody
asked to move, refuses when `/joint_states` is stale or when another publisher is
already driving the command topic, and raises `MotionRefused` instead of
clipping silently. It is Python because `rclpy` and the vendor SDK are ---
see [ADR 0002](../decisions/0002-python-at-the-ros-boundary.md).

## The C++ side: `ros/rtc_teleop/`

Source for `astrabot_teleop` and `astrabot_rtc`, with gtest suites. Note that
the *running* nodes are the vendor binaries in `/opt/ros/astrabot`, not these ---
this tree is how the teleop path is understood and modified, not what is live.

## Data

`data/` is the append-only evidence ledger and is tracked deliberately. Raw
frames are kept because a hardware claim without its named source frame cannot
be reproduced or audited.

| Path | Contents |
|---|---|
| `vista_runs/<run>/` | observations: `rgb.png`, `depth.npy`, `camera_info.json`, `state.json`, `latest.json` |
| `attempts/attempt_*/` | immutable production snapshot, proposal, plan, diagnostics and timings |
| `experiments/<id>/` | one experiment: `REPORT.md` plus frames |
| `snapshots/` | raw captures |
| `benchmarks/` | dated, non-motion timing and resource measurements |
