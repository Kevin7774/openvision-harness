# XR1 workspace

Grasping stack for an **AstraBot XR1** humanoid on a Jetson AGX Thor
(`tegra-ubuntu`, Ubuntu 24.04, PREEMPT_RT kernel, ROS 2 Jazzy).

The robot's own control software is a **binary-only vendor overlay** in
`/opt/ros/astrabot`, supervised by ~20 `Astrabot_*.service` units that restart
themselves within ~100 ms. This workspace is everything *around* that: the
perception and kinematics that decide where to reach, the thin Python layer that
actually commands the arms, and the evidence ledger that says whether it worked.

```
crates/xr1-vision/   Rust: perception, kinematics, experiment journal   <- main line
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
xr1-vision plan                   # yellow block -> footprint -> grasp IK
xr1-vision fk J1 .. J7            # fingertip-pad FK, for hand-eye work
xr1-vision sensor-status          # D405 / tactile capability report
xr1-vision servo-propose --input examples/servo_request.json --state STATE_JSON
```

Read [`docs/operations/status.md`](docs/operations/status.md) before moving the
robot, and [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md) before
believing any single reading. Several sessions share this one machine.

## What is verified

| Claim | Evidence |
|---|---|
| ZED depth is trustworthy enough to localise the block | independently validated against the teleoperated grasp pose to 2.0 mm |
| The colour mask separates the block from the green cube *and* from the orange gripper pads | 3 tests in `crates/xr1-vision/src/perception.rs`, thresholds measured off named frames |
| A named visual-servo proposal cannot bypass the Rust step, freshness, URDF-margin, fingertip-floor or required-sensor gates | unit tests in `visual_servo.rs`, `kinematics.rs` and `safety.rs` |
| One grasp succeeded | 2026-08-18, gripper reads 149 closed on the object vs 14 closed on air, and it stayed at 148 after lifting |
| Motion is rate-limited and clamped to the live URDF | `py/astra_arm.py`, refuses on stale `/joint_states` or a busy command channel |

## What is not

- **Grasping is orientation-dependent.** The one success came from a block yaw
  that put the tool-frame error across the closing axis. Nothing was fixed; a
  different yaw still fails.
- **No force sensing.** `effort` is `.nan` on every joint, so contact can only
  ever be inferred geometrically.
- **Near-field sensing is not execution-ready.** The D405 is present but on a
  degraded 480 Mbit/s path; the two tactile CH340 devices have no tty driver or
  verified frame contract.
- **The Rust visual-servo proposal and deterministic gate exist, but the live
  measure/execute/verify loop does not.** The old Python loop remains historical
  evidence in [ADR 0003](docs/decisions/0003-lost-python-pipeline.md).

## Docs

| | |
|---|---|
| [`docs/architecture/overview.md`](docs/architecture/overview.md) | who owns the robot, the layers, what lives where |
| [`docs/architecture/hardware-map.md`](docs/architecture/hardware-map.md) | buses, device nodes, cameras, joint ids --- all measured |
| [`docs/architecture/gripper-g2.md`](docs/architecture/gripper-g2.md) | G2 grippers: Modbus registers, the only grasp signal, bus health |
| [`docs/architecture/perception.md`](docs/architecture/perception.md) | image → block pose, and why each threshold is that number |
| [`docs/architecture/kinematics.md`](docs/architecture/kinematics.md) | the tool frame, the IK, the grasp gates |
| [`docs/operations/status.md`](docs/operations/status.md) | **read first**: current constants, what works, ranked blockers |
| [`docs/operations/runbook.md`](docs/operations/runbook.md) | observe / plan / experiment / record / teleop, and their preconditions |
| [`docs/operations/pitfalls.md`](docs/operations/pitfalls.md) | 55 failures with their disambiguating evidence. Long on purpose |
| [`docs/development/building.md`](docs/development/building.md) | toolchain, the four gates, what the tests guard |
| [`docs/decisions/`](docs/decisions/) | why the architecture is this shape, including what was deleted |

See [`docs/development/visual-servo.md`](docs/development/visual-servo.md) for
the implemented proposal boundary and the remaining live-loop work.

`bin/check-doc-links` fails if any of those links rot.
