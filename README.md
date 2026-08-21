# OpenVision Harness — XR1 workspace

**English** | [简体中文](README.zh-CN.md)

Grasping stack for an **AstraBot XR1** humanoid on a Jetson AGX Thor
(`tegra-ubuntu`, Ubuntu 24.04, PREEMPT_RT kernel, ROS 2 Jazzy).

The robot's own control software is a **binary-only vendor overlay** in
`/opt/ros/astrabot`, supervised by ~20 `Astrabot_*.service` units that restart
themselves within ~100 ms. This workspace is everything *around* that: the
perception and kinematics that decide where to reach, the thin Python layer that
actually commands the arms, and the evidence ledger that says whether it worked.

## Current status — 2026-08-21

The harness now has a tested software path from typed hardware contracts and a
registered task pack through bounded visual/tactile execution and policy
promotion. Robot access and the D405 stream are live-validated. That does **not**
mean the robot is autonomous or self-improving yet: D405 target/Jacobian and
pressure calibration, real evaluation data and automatic reset are still
missing.

| Area | Current state |
|---|---|
| Robot-independent contracts | `harness-contracts` defines five hardware-independent ports plus versioned `RobotProfile` and `CalibrationManifest` contracts |
| Task packaging | The yellow-block pick/place behavior lives in `task-packs/yellow-block-pick-place` and is selected through the task registry instead of being hard-coded into the core |
| Runtime boundaries | Argument parsing, adapter protocol, evidence handling and action locking are isolated under `xr1-vision/src/support`; physical actions remain bounded, serialized and fail-closed |
| Near-field/contact grasping | The D405 sustained 20 aligned frames at 11.03 Hz on the robot; two-pad pressure assessment and the bounded close/hold/single-release loop are implemented, but pressure USB mapping and live target/Jacobian calibration are still missing |
| Evaluation and promotion | `harness-evaluation` implements immutable episodes, a two-channel judge with abstention, frozen golden sets, policy lineage, baseline/challenger gates, shadow, canary, promotion and unconditional rollback |
| Live readiness | SSH, controller, ZED, joint feedback and D405 capture work; pressure input and the current target/Jacobian calibration remain fail-closed, so passing software tests is not permission to use `--go` |

There is currently **no trainer, no real-robot episode corpus, no real golden
set or measured judge bias, and no automatic reset**. The repository implements
and tests the path that decides whether a separately produced challenger may be
promoted; it does not yet implement "self-evolution." See
[`docs/assessment/harness-step-5-evaluation.md`](docs/assessment/harness-step-5-evaluation.md)
and the live constraints in
[`docs/operations/status.md`](docs/operations/status.md).

Latest local verification: **159 Rust tests passed**, **29 Python tests ran
(2 hardware-dependent tests skipped)**, Clippy passed with warnings denied, and
all documentation links resolved. These are offline/software checks only.

## Architecture and dependency direction

The production dependency direction is intentionally one-way. A lower layer
never imports or calls a higher one:

```text
profiles/*.json / calibration data        task-packs/yellow-block-pick-place
                  │                                     │
                  └──────────┐               ┌──────────┘
                             ▼               ▼
                     harness-contracts (ports)
                         │              │
                         ▼              ▼
                harness-evaluation   xr1-vision
                         └──────────────►│
                                        │ validated JSON/subprocess boundary
                     ┌──────────────────┼──────────────────┐
                     ▼                  ▼                  ▼
              py/ ROS adapters   xr1_moveit_bridge   data/ evidence
                     │
                     ▼
           vendor ROS 2 nodes and physical hardware
```

`xr1-vision` links the Rust crates and task pack, but it does not link `rclpy`
or the vendor SDK. It invokes narrow Python adapters and the MoveIt validator
through explicit, validated process contracts. `astrabot_rtc` and
`astrabot_teleop` share the ROS graph and deployment environment but are a
separate C++ teleoperation path; neither is imported by the Rust grasp planner.

## Repository map and dependencies

Only source and evidence tracked by Git are described below. `target/`, Python
cache directories and `ros/rtc_teleop/{build,install,log}/` are generated,
ignored products and are not architectural inputs.

### Root files

| Path | Responsibility | Depends on / consumed by |
|---|---|---|
| `Cargo.toml` | Rust workspace definition; includes every crate under `crates/*` and `task-packs/*` | Cargo 1.75+; controls all Rust builds |
| `Cargo.lock` | Reproducible third-party Rust dependency resolution | Generated and consumed by Cargo; must be committed with dependency changes |
| `AGENTS.md` | Safety, language, evidence and repository workflow contract | Applies to every contributor and automation session |
| `.gitignore` | Excludes reproducible build products while deliberately keeping measured data tracked | Git; see its comment before pruning evidence |
| `README.md` / `README.zh-CN.md` | English and Chinese entry points | Link to authoritative architecture and operations documents below |

### `crates/` — Rust core workspace

| Second-level directory | Responsibility | Direct dependencies | Used by |
|---|---|---|---|
| `crates/harness-contracts/` | Hardware-neutral traits plus versioned robot identity, geometry and calibration contracts | `serde`, `serde_json` only; deliberately no ROS, perception or robot dependency | `harness-evaluation`, `xr1-vision`, every task pack and profile/calibration tooling |
| `crates/harness-evaluation/` | Immutable episodes, fleet scope, layered outcome judge with abstention, golden sets, policy registry and rollout gates | `harness-contracts`, `serde`, `serde_json` | `xr1-vision` CLI and future offline evaluation/promotion jobs; never moves hardware |
| `crates/xr1-vision/` | Main Rust library and CLI: observations, semantic proposals, perception, kinematics, planning, safety, bounded servo/grasp loops and evidence | Both crates above, `yellow-block-pick-place`, `image`, `nalgebra`, `roxmltree`, `serde`, `serde_json` | Operators/agents; calls `py/` adapters and optionally `xr1_moveit_validator` |

Inside each crate:

| Directory | Responsibility | Dependency boundary |
|---|---|---|
| `harness-contracts/src/` | `ports.rs` defines the five replaceable seams; `profile.rs` and `calibration.rs` bind measurements to one robot/station/URDF | Lowest Rust layer; consumers may depend on it, it may not depend on them |
| `harness-contracts/tests/` | Loads `profiles/examples/` and verifies example schemas and fail-closed placeholder calibration | Depends on example JSON, not live hardware |
| `harness-evaluation/src/` | `episode`, `judge`, `golden`, `policy`, `promotion` and `lifecycle` modules | Depends only on contracts and serialized evidence; no runtime adapter |
| `harness-evaluation/tests/` | End-to-end synthetic episode → judge → gate → shadow/canary/promotion/rollback tests | Uses synthetic data; it is not a real-robot result |
| `xr1-vision/src/kinematics/` | URDF model, FK/IK, grasp geometry and joint/floor margins | Uses `nalgebra`, `roxmltree`, active profile constants; feeds planning and safety |
| `xr1-vision/src/perception/` | ZED/D405 depth, geometry, near-field and visual-servo signals | Uses `image`; yellow detection is re-exported from the registered task pack |
| `xr1-vision/src/planning/` | Candidate search/ranking and optional batch MoveIt collision validation | Consumes perception + kinematics; invokes `xr1_moveit_validator` when configured |
| `xr1-vision/src/support/` | Shared argument parsing, JSON adapter protocol, evidence I/O and exclusive action-loop lock | Used by CLI, servo loop and grasp loop so they cannot implement divergent boundaries |
| `xr1-vision/src/task/` | Typed task events and deterministic replay executive | Consumes proposals, grounded task-skill ids and physical evidence; currently replay-oriented |
| Other `xr1-vision/src/*.rs` | CLI routing, observations, hardware capability status, safety envelopes, servo/grasp orchestration and task-pack registry | Rust owns decisions; physical execution crosses only through adapters |

### `task-packs/` — task-specific behavior

| Second-level directory | Responsibility | Direct dependencies | Used by |
|---|---|---|---|
| `task-packs/yellow-block-pick-place/` | First registered capability: measured yellow detector and `yellow_block.pick_place` task-skill implementation | `harness-contracts` and PNG support from `image` | Registered by `xr1-vision/src/taskpack.rs`; detector is consumed by perception |
| `yellow-block-pick-place/src/` | `detector.rs` owns the measured colour thresholds; `lib.rs` owns stable object/skill ids and grounding rules | Must not import XR1 core; a new object should be a new pack rather than a core edit |

### `py/` — ROS 2 and hardware boundary

This directory has no subpackages by design: every file is a narrow executable
boundary. Business decisions stay in Rust.

| File group | Responsibility | Depends on / called by |
|---|---|---|
| `astra_arm.py`, `xr1.py` | Joint feedback, rate-limited arm commands, URDF clamps, G2 gripper bring-up and operator commands | ROS 2 Jazzy `rclpy`, vendor topics/SDK; used directly and by motion adapters |
| `vista_observe.py` | Synchronized ZED RGB/depth, intrinsics, joint state and image-time TF capture | `rclpy`, `sensor_msgs`, `tf2_ros`, OpenCV, NumPy; called by `bin/xr1 observe` paths |
| `d405_observe.py` | Bounded aligned D405 RGB/depth capture with stream/freshness checks | `pyrealsense2`, NumPy, `rclpy`; called by D405 and grasp-loop commands |
| `tactile_adapter.py` | Two-pad pressure capture, serial or user-space CH340/PyUSB transport, median/MAD evidence | Python stdlib and optional PyUSB; called by tactile and grasp-loop commands |
| `motion_adapter.py`, `servo_adapter.py`, `grip_adapter.py` | Execute exactly one Rust-approved motion, microstep or jaw increment and return a bound JSON report | `astra_arm.py` or ROS gripper topics; called only after Rust safety approval; dry-run by default |
| `pad_offset_measure.py` | Offline multi-pose pad/tool-offset measurement | NumPy and `bin/xr1 fk`; writes calibration evidence |
| `xr1_cam.py` | Controls the external Mac recorder over SSH/SCP | `mac/` installation and daemon; used by experiment recording paths |
| `test_*.py` | Offline adapter contract and refusal tests | Python `unittest`; hardware-dependent paths are injected or skipped |

### `ros/` — C++ ROS 2 workspace

| Second-level directory | Responsibility | Depends on / used by |
|---|---|---|
| `ros/rtc_teleop/` | Colcon workspace containing robot startup integration, RTC transport, teleoperation and MoveIt validation | ROS 2 Jazzy/ament, vendor overlay and package-specific system libraries |
| `ros/rtc_teleop/robot_start/` | Deployment/startup material copied from the robot path | systemd, shell, ROS environment; deploy changes affect shared robot services and require explicit operational review |
| `ros/rtc_teleop/src/` | Three source packages described below | `colcon build`; outputs are generated in ignored build/install/log directories |

`robot_start/start_up/` is split into:

| Directory | Responsibility | Dependency boundary |
|---|---|---|
| `chrony_time_syn/` | Robot time-synchronization helpers | Chrony and robot network configuration |
| `config/` | Startup configuration consumed by launch/service scripts | Vendor installation layout and ROS endpoints |
| `documents/` | Deployment notes and supporting material | Operational reference only |
| `environment/` | Environment setup sourced before services start | ROS distribution, vendor overlay and runtime library paths |
| `run_script/` | Thor base/supplement launch scripts and systemd unit templates | Calls installed ROS executables; must not be confused with source packages |
| `test/` | Shell contract tests for controller, RTC, teleop, wrist camera, logging and ZED service definitions | Reads startup files; does not establish live-hardware correctness |

Packages under `ros/rtc_teleop/src/`:

| Package | Responsibility | Direct dependencies | Relationship to harness |
|---|---|---|---|
| `astrabot_rtc/` | Generic signaling, peer/media transport and authorized DataChannel routing; deliberately knows no teleop semantics | `rclcpp`, ROS messages/services, FFmpeg, JSON; optional pinned `libdatachannel` backend | Produces `RtcDataPacket`/peer events consumed by `astrabot_teleop`; not used by Rust planning |
| `astrabot_teleop/` | Grant verification, frame validation, deadman/watchdog, owner lease and typed/shadow command production | `astrabot_rtc`, `astrabot_data_interfaces`, ROS messages, OpenSSL, Protobuf, JSON | Independent teleoperation path into arbitration; does not bypass the grasp safety path |
| `xr1_moveit_bridge/` | Batch MoveIt 2 collision validation for Rust grasp candidates | MoveIt Core/messages, URDF/SRDF, OctoMap 1.9 ABI, geometry/shape messages, JSON | Executable is invoked by `xr1-vision` planning when MoveIt validation is requested |

Internal directories of the ROS packages:

| Directory | Responsibility | Depends on / consumed by |
|---|---|---|
| `cmake/` | Reusable CMake checks/toolchain fragments, including no-exceptions enforcement | Included by package `CMakeLists.txt` and cross-build scripts |
| `config/` | Runtime YAML and XR1 SRDF configuration | Parsed by the package at startup or installed for MoveIt; invalid values fail closed |
| `docker/` | Reproducible native/cross-build environments for the RTC packages | Docker plus the pinned ROS/SDK images; not used by running nodes |
| `docs/` | Package-specific architecture and retrospective notes | Maintainers; subordinate to code and top-level ADRs |
| `include/` | Public C++ contracts grouped by config/media/protocol/runtime/session/safety/transport concerns | Implemented by the same package's `src/`; consumed by its tests and linked targets |
| `launch/` | ROS 2 launch entry points | `launch`, `launch_ros`, installed package config and environment variables |
| `msg/` and `srv/` | Generated RTC/Teleop ROS message and service contracts | `rosidl_default_generators`; consumed across `astrabot_rtc`, `astrabot_teleop`, arbitration and data collection |
| `proto/` | Frozen Quest `TeleopFrame` wire schema (`astrabot_teleop` only) | Protobuf compiler/runtime; consumed by the teleop frame codec |
| `scripts/` | Native, Docker and ARM64 build/format/runtime staging gates | CMake/colcon, Docker and pinned SDKs; development/deployment only |
| `src/` | C++ implementations and node entry points | Public headers plus ROS/system dependencies declared in `package.xml` |
| `systemd/` | Installed service unit templates for RTC/Teleop | `robot_start` deployment and installed ROS executables |
| `test/` | Unit, interface, integration and safety-contract tests | Package libraries plus GTest/shell; does not substitute for HIL or soak tests |

`xr1_moveit_bridge` needs only `config/include/src/test`; the RTC packages use
the broader layout above. Read each package's own README before changing its
deployment behavior.

### `profiles/` and `examples/` — configuration contracts

| Directory | Responsibility | Depends on / used by |
|---|---|---|
| `profiles/examples/` | Example `RobotProfile` and `CalibrationManifest` for the XR1 Thor | Parsed and validated by `harness-contracts`; placeholder calibration intentionally blocks motion |
| `examples/` | TaskProposal, visual-servo request, tactile config/calibration, D405 target and replay event examples | Inputs to `xr1-vision` CLI and Python adapters; examples are schemas, not current live calibration |

### `data/` — append-only measured evidence

`data/` is intentionally tracked, including images and video. Producers must
write a new dated record; consumers must not silently rewrite old evidence.

| Second-level directory | Responsibility | Produced by / consumed by |
|---|---|---|
| `data/benchmarks/` | Dated IK and semantic-planner latency measurements | Benchmark runs; used for performance baselines, not safety authorization |
| `data/experiments/` | Operator experiments, journals, before/after frames, hand-eye and servo measurements | `xr1.py`, calibration helpers and operators; referenced by architecture/ADR conclusions |
| `data/snapshots/` | Small dated diagnostic snapshots, including current robot-host permission failure evidence | Read-only diagnostic commands; referenced by `docs/operations/status.md` |
| `data/vista_runs/` | Self-describing observation runs with RGB/depth/state/TF bundles | `vista_observe.py` and observation commands; consumed by perception regressions and audits |

Second-level groups inside `data/experiments/`:

| Directory | Evidence held | Depends on / consumed by |
|---|---|---|
| `20260817-01/`, `20260817-02/` | Structured run metadata, event records, reports and an external-camera clip | Experiment runner and Mac recorder; used to reconstruct those two runs |
| `d455_which_arm/` | Wrist-camera arm-identity watch records | D455/USB topology at capture time; hardware-map diagnosis |
| `handeye/` | Multi-pose samples, placed truth, fit output and annotated ZED image | `pad_offset_measure.py`, FK and operator annotations; tool-frame analysis |
| `loops/` | Iterative observation/plan journals and marked frames | Historical loop runner; consumed as evidence, not as current executable state |
| `pad_sideon/` | Side-on gripper-pad images, camera info and joint state | ZED plus named robot pose; pad geometry measurement |
| `servo/` | Plus/minus joint perturbations and before/after microstep captures | Visual-servo measurement sessions; Jacobian and reconciliation analysis |
| `teleop_truth/` | Teleoperated successful grasp truth poses | Teleop and robot state; perception/localisation validation |
| `wrist_extrinsics/`, `wrist_scan/`, `wristcam/` | Wrist-camera identity, scan and extrinsic observations | Wrist camera topology and capture tools; camera-map calibration |
| `zed_hand_probe/` | ZED/hand overlap probe | ZED image plus robot state; reach/visibility diagnosis |

Second-level runs inside `data/vista_runs/`:

| Directory | Responsibility | Depends on / consumed by |
|---|---|---|
| `harness-upgrade-20260819/` | Observations captured while validating the harness upgrade and live capabilities | ZED/ROS capture; status and upgrade assessment |
| `servo_ik_audit/` | Frames and state used to audit servo perception against IK | Observation bundle, FK/IK code and audit analysis |
| `yellow-block-harness/` | Canonical yellow-block and gripper-pad observation corpus | Perception regression tests, planning evidence and named-frame claims |

Each Vista run contains an `observations/` ledger. Do not mix frames across runs
without carrying the frame id, timestamp, transform and robot state. Every
experiment group depends on the hardware configuration at its recorded time;
directory names and timestamps are part of the evidence identity.

### `docs/` — authoritative written context

| Second-level directory | Responsibility | Depends on / used by |
|---|---|---|
| `docs/architecture/` | Current hardware map, perception, kinematics, proposals and gripper design | Must agree with code and dated evidence; read before structural changes |
| `docs/assessment/` | Bilingual step-by-step harness assessment and implementation reports | Summarizes contracts, task-pack split, orchestration split and promotion path |
| `docs/decisions/` | Numbered ADRs explaining irreversible or safety-relevant choices | New contradictions require a superseding ADR, not silent history edits |
| `docs/development/` | Build gates and visual-servo implementation guidance | Used by contributors and CI-style local checks |
| `docs/operations/` | Live status, runbook and measured failure modes | Mandatory before hardware actions; depends on the newest dated observations |

### `bin/` and `mac/` — operations support

| Directory | Responsibility | Depends on / used by |
|---|---|---|
| `bin/` | `audit-deps`, `check-doc-links`, ROS-domain-aware `home`, and TF frame health check | Shell, Cargo metadata and sourced ROS environment; used by development/operations checks |
| `mac/` | AVFoundation recorder, launch configuration and installer | macOS Swift/AVFoundation and camera permission; controlled remotely by `py/xr1_cam.py` |

## Run it

Every ROS command needs the domain, or you silently attach to an almost-empty
graph and draw wrong conclusions:

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
```

```bash
python3 py/xr1.py pose            # joints, grippers, tcp
python3 py/xr1.py bringup         # the G2 gripper driver is not a systemd unit
bin/tf-frames                     # 52 frames, 6 of them zed_*, or something is dead
cargo build --release             # -> target/release/xr1-vision
bin/xr1 observe                    # ZED snapshot + intrinsics + image-time TF
bin/xr1 bundle                     # one typed ZED/robot/capability observation
bin/xr1 validate-proposal --proposal examples/pick_place_proposal.json
bin/xr1 plan                       # immutable Release attempt receipt
bin/xr1 plan --proposal examples/grasp_proposal.json
bin/xr1 plan --proposal examples/grasp_proposal.json --latest SAVED_LATEST_JSON
bin/xr1 replay --proposal examples/pick_place_proposal.json \
  --events examples/task_events.jsonl
bin/xr1 fk J1 .. J7               # fingertip-pad FK, for hand-eye work
bin/xr1 sensor-status             # D405 / tactile capability report
bin/xr1 d405-observe              # fresh aligned RGB/depth + near-field target signal
bin/xr1 tactile-observe --config TACTILE_CONFIG
bin/xr1 tactile-assess --mode closure --config TACTILE_CONFIG --calibration TACTILE_CALIBRATION
bin/xr1 servo-pads --frame FRAME_DIR
bin/xr1 servo-observe --latest LATEST_JSON
bin/xr1 servo-calibrate --input PLUS_MINUS_SAMPLES_JSON
bin/xr1 servo-propose --input examples/servo_request.json --state STATE_JSON
bin/xr1 servo-propose --request CALIBRATED_REQUEST_JSON --state STATE_JSON \
  > /tmp/servo-proposal.json
bin/xr1 servo-step --proposal /tmp/servo-proposal.json       # dry-run
bin/xr1 servo-step --proposal /tmp/servo-proposal.json --go  # exactly one microstep
bin/xr1 servo-reconcile --input RECONCILIATION_JSON
bin/xr1 servo-loop --calibration CALIBRATION_JSON            # fresh dry-run
bin/xr1 servo-loop --calibration CALIBRATION_JSON --go       # bounded live loop
bin/xr1 grasp-loop --tactile-config TACTILE_CONFIG \
  --tactile-calibration TACTILE_CALIBRATION --d405-target D405_TARGET
# Add --go only after the dry run passes; each jaw step is at most 0.05.
```

From another machine, use one non-interactive SSH command and let the wrapper
reuse the connection: `bin/xr1 --host astrabot@192.168.123.102 status`.

Production robot operation accepts only `bin/xr1`, which pins Release. `plan`
returns a short receipt and stores the full immutable attempt under
`data/attempts/attempt_*/`; repeat inspection reads that attempt without
replanning. The executor accepts only its `attempt_path`, never an arbitrary
`plan.json`.

Read [`docs/operations/status.md`](docs/operations/status.md) before moving the
robot, and [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md) before
believing any single reading. Several sessions share this one machine.

## What is verified

| Claim | Evidence |
|---|---|
| ZED depth is trustworthy enough to localise the block | independently validated against the teleoperated grasp pose to 2.0 mm |
| The colour mask separates the block from the green cube *and* from the orange gripper pads | measured regression tests in `crates/xr1-vision/src/perception/` |
| A named visual-servo proposal cannot bypass the Rust step, freshness, URDF-margin, fingertip-floor or required-sensor gates | unit tests in `visual_servo.rs`, `kinematics/` and `safety.rs` |
| The servo signal uses the two physical orange pads near FK, not the larger orange fruit | named frame `20260818-120701-385142786-132823` in the Rust perception regression |
| A microstep must be followed by a distinct newer observation and stops on a sign flip or three reductions below 10% | reconciliation tests in `visual_servo.rs` |
| The live loop performs at most one approved microstep between observations, has step/time/concurrency bounds, and never starts, stops or restarts a service | `servo-loop`, the Rust safety envelope and `servo_adapter.py` |
| The near-field/contact loop cannot close twice from one pressure sample, stops on pad imbalance, and permits only one release increment after overpressure | `grasp-loop`, `grasp_feedback.rs`, `grip_adapter.py` and their offline tests |
| Task events cannot skip observation, grounding, geometry, validation or physical verification stages | state-transition tests in `task/executive.rs` |
| One grasp succeeded | 2026-08-18, gripper reads 149 closed on the object vs 14 closed on air, and it stayed at 148 after lifting |
| Motion is rate-limited and clamped to the live URDF | `py/astra_arm.py`, refuses on stale `/joint_states` or a busy command channel |

## What is not

- **Grasping is orientation-dependent.** The one success came from a block yaw
  that put the tool-frame error across the closing axis. Nothing was fixed; a
  different yaw still fails.
- **No joint/arm force feedback.** `effort` is `.nan` on every joint, so a
  request that requires that channel still fails closed. Two pressure patches
  do exist inside the gripper; their software boundary and deterministic
  contact policy are implemented separately from joint effort.
- **Near-field hardware is not yet live-validated.** D405 capture, CH340/PyUSB
  pressure capture and the bounded grasp loop are implemented, but the current
  robot still needs its exact USB paths, frame fields, pad mapping, pressure
  thresholds and D405 target/Jacobian measured before `--go` can pass.
- **The bounded visual-servo orchestration is implemented, but the current 3×3
  Jacobian has not yet been re-measured and validated on live hardware.**
  `servo-loop` makes capture and reconciliation mandatory before another step,
  but it cannot make an old or absent calibration valid. The old Python loop
  remains historical evidence in
  [ADR 0003](docs/decisions/0003-lost-python-pipeline.md).
- **TaskProposal v2 and the task executive are connected for validation and
  replay, not live autonomous execution yet.** `replay` consumes evidence; it
  does not fabricate it or publish motion.

## Docs

| | |
|---|---|
| [`docs/architecture/overview.md`](docs/architecture/overview.md) | who owns the robot, the layers, what lives where |
| [`docs/architecture/hardware-map.md`](docs/architecture/hardware-map.md) | buses, device nodes, cameras, joint ids --- all measured |
| [`docs/architecture/gripper-g2.md`](docs/architecture/gripper-g2.md) | G2 grippers: Modbus registers, the only grasp signal, bus health |
| [`docs/architecture/perception.md`](docs/architecture/perception.md) | image → block pose, and why each threshold is that number |
| [`docs/architecture/kinematics.md`](docs/architecture/kinematics.md) | the tool frame, the IK, the grasp gates |
| [`docs/architecture/proposals.md`](docs/architecture/proposals.md) | semantic proposal and typed grasp-candidate contracts |
| [`docs/operations/status.md`](docs/operations/status.md) | **read first**: current constants, what works, ranked blockers |
| [`docs/operations/runbook.md`](docs/operations/runbook.md) | observe / plan / experiment / record / teleop, and their preconditions |
| [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md) | 55 failures with their disambiguating evidence. Long on purpose |
| [`docs/development/building.md`](docs/development/building.md) | toolchain, the four gates, what the tests guard |
| [`docs/decisions/`](docs/decisions/) | why the architecture is this shape, including what was deleted |

See [`docs/development/visual-servo.md`](docs/development/visual-servo.md) for
the implemented proposal boundary and the remaining live-loop work.

`bin/check-doc-links` fails if any of those links rot.
