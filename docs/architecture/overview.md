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
Agent      you / an LLM          pick the next experiment, write the prediction
  |
Planner    xr1-vision plan       image -> object pose -> grasp candidates -> IK
  |
Executor   py/xr1.py             one named motion, one return code
  |
Safety     py/astra_arm.py       rate limit, URDF clamp, staleness, channel idle
  |
Device     /opt/ros/astrabot     ros2_control + vendor binaries
```

Calls only ever go downward. The Planner never publishes to a command topic; the
Executor never decides *where* to reach; the Safety layer never asks anything
above it for permission.

## The Rust side: `crates/xr1-vision`

One binary, three files, no framework.

| Module | Responsibility |
|---|---|
| `perception.rs` | read one observation, find the block, produce ranked grasp candidates as JSON |
| `kinematics.rs` | parse the live URDF chain, forward kinematics to the gripper pads, position IK, grasp-geometry feasibility |
| `main.rs` | argument parsing, shelling out to the two Python entry points, and the experiment journal |

Commands:

```
xr1-vision preflight                       py/xr1.py pose + py/xr1_cam.py doctor
xr1-vision observe                         py/vista_observe.py -> data/vista_runs/<run>/latest.json
xr1-vision plan                            perception::plan on the newest observation
xr1-vision fk J1 .. JN                     pad-inner points, midpoint and tool rotation
xr1-vision begin --purpose TEXT            open a numbered experiment
xr1-vision note --section NAME --text TEXT  append to its report
xr1-vision grip --side S --state open|close
xr1-vision end --status SUCCESS|FAILED
xr1-vision status
```

`observe` and `plan` are deliberately separate processes: `observe` touches
hardware and `plan` is pure computation over files, so a plan can be re-run and
argued with long after the frame was taken. `plan` moves nothing --- its output
is labelled `online_plan_dry_run` and executing it is a separate, human decision.

## The Python side: `py/`

| File | Role |
|---|---|
| `xr1.py` | the robot API: `bringup`, `pose`, `look`, `grip`, `home`, `wave`, `demo`, `rec`, `snap` |
| `astra_arm.py` | the safety layer wrapping the vendor SDK (see below) |
| `vista_observe.py` | read-only ZED snapshot: RGB, aligned depth, intrinsics, image-time TF |
| `xr1_cam.py` | drive the external recorder on the Mac at 192.168.123.138 |
| `pad_offset_measure.py` | measure the gripper-pad pixel offset against `xr1-vision fk` |
| `motion_adapter.py` | joint-name / sign mapping for the vendor motion interface |
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

`data/` is the evidence ledger and is gitignored (180 MB and growing):

| Path | Contents |
|---|---|
| `vista_runs/<run>/` | observations: `rgb.png`, `depth.npy`, `camera_info.json`, `state.json`, `latest.json` |
| `experiments/<id>/` | one experiment: `REPORT.md` plus frames |
| `snapshots/` | raw captures |
