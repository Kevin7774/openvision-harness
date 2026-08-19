# XR1 workspace

Grasping stack for an **AstraBot XR1** humanoid on a Jetson AGX Thor
(`tegra-ubuntu`, Ubuntu 24.04, PREEMPT_RT kernel, ROS 2 Jazzy).

The robot's own control software is a **binary-only vendor overlay** in
`/opt/ros/astrabot`, supervised by ~20 `Astrabot_*.service` units that restart
themselves within ~100 ms. This workspace is everything *around* that: the
perception and kinematics that decide where to reach, the thin Python layer that
actually commands the arms, and the evidence ledger that says whether it worked.

```
crates/xr1-vision/   Rust library + thin CLI: task executive, observation,
                     perception, planning, kinematics and safety        <- main line
py/                  Python: the rclpy / hardware boundary              <- see AGENTS.md §language
ros/rtc_teleop/      C++ ROS 2 nodes (source for the vendor teleop path)
mac/                 Swift: the external-camera recorder that runs on the Mac
bin/home             one-line wrapper that exports ROS_DOMAIN_ID before homing
data/                measured evidence: vista_runs/ (observations), experiments/,
                     snapshots/ --- append-only, every record dated
docs/                architecture / operations / development / decisions
```

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
export PATH="$PWD/target/release:$PATH"   # the commands below are that binary
xr1-vision observe                # ZED snapshot + intrinsics + image-time TF
xr1-vision bundle                 # one typed ZED/robot/capability observation
xr1-vision validate-proposal --proposal examples/pick_place_proposal.json
xr1-vision plan                   # default yellow-block semantic proposal -> grasp candidates
xr1-vision plan --proposal examples/grasp_proposal.json
xr1-vision plan --proposal examples/grasp_proposal.json --latest SAVED_LATEST_JSON
xr1-vision replay --proposal examples/pick_place_proposal.json \
  --events examples/task_events.jsonl
xr1-vision fk J1 .. J7            # fingertip-pad FK, for hand-eye work
xr1-vision sensor-status          # D405 / tactile capability report
xr1-vision d405-observe           # fresh aligned RGB/depth + near-field target signal
xr1-vision tactile-observe --config TACTILE_CONFIG
xr1-vision tactile-assess --mode closure --config TACTILE_CONFIG --calibration TACTILE_CALIBRATION
xr1-vision servo-pads --frame FRAME_DIR
xr1-vision servo-observe --latest LATEST_JSON
xr1-vision servo-calibrate --input PLUS_MINUS_SAMPLES_JSON
xr1-vision servo-propose --input examples/servo_request.json --state STATE_JSON
xr1-vision servo-propose --request CALIBRATED_REQUEST_JSON --state STATE_JSON \
  > /tmp/servo-proposal.json
xr1-vision servo-step --proposal /tmp/servo-proposal.json       # dry-run
xr1-vision servo-step --proposal /tmp/servo-proposal.json --go  # exactly one microstep
xr1-vision servo-reconcile --input RECONCILIATION_JSON
xr1-vision servo-loop --calibration CALIBRATION_JSON             # fresh dry-run
xr1-vision servo-loop --calibration CALIBRATION_JSON --go        # bounded live loop
xr1-vision grasp-loop --tactile-config TACTILE_CONFIG \
  --tactile-calibration TACTILE_CALIBRATION --d405-target D405_TARGET
# Add --go only after the dry run passes; each jaw step is at most 0.05.
```

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
