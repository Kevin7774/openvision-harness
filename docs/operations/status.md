# Status

Read this before moving the robot. **Several sessions share this machine** ---
the arm is wherever the last one left it, and restarting a service or claiming a
serial port is visible to all of them. `pgrep -af` before any `pkill`.

**A coordinate without a timestamp is meaningless here.** Two sessions once
produced flatly contradictory reachability results for "the current grasp point";
both were right, because the block had been moved 67 mm between them --- same
perception path, same head pose, `depth_frac` 1.0 in both, and the pixel centroid
went (693, 265) → (703, 307) with the area 738 → 867. The object moved. Re-observe
at the start of every session and never inherit an absolute coordinate.

Every line here carries the date it was measured. Nothing in this file is a
prediction. Replace a line when you re-measure it; do not append a new dated
section --- history lives in `git log`.

## Verify before trusting

```bash
export ROS_DOMAIN_ID=12 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash
python3 py/xr1.py pose        # joints, grippers, tcp -- and proves /joint_states is live
python3 py/xr1.py bringup     # after any reboot: the G2 driver is not a systemd unit
bin/tf-frames                 # 52 frames total, 6 zed_*; exits non-zero otherwise
python3 py/xr1_cam.py doctor  # the Mac recorder is EXCLUSIVE; state=recording means someone else is live

`xr1-vision preflight` runs the first and last of those together. There is no
single READY/NOT-READY probe any more --- the script that did that is gone
([ADR 0003](../decisions/0003-lost-python-pipeline.md)), and the checks above are
what it actually looked at.
```

## Authoritative constants

| Constant | Value | Provenance |
|---|---|---|
| `TIP_CENTER_M` | `[-0.0225, 0, 0.0485]` m | **this is the one the code uses** --- `crates/xr1-vision/src/kinematics/types.rs`, tcp frame, 53.5 mm long |
| `TCP_TO_TIP` | 0.168 m | operator tape measure (flange to pad centre 230 mm, minus the 62 mm the tcp sits outside the flange, 08-14). ⚠️ **Not in force anywhere.** It disagrees with `TIP_CENTER_M` by 119 mm and it lived in a script that no longer exists. An 8-pose camera measurement puts the residual at tens of mm, not 119 --- so do not paste this number into the code. [ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md) |
| `TABLE_TOP` | 0.790 m | upper edge of the band implied by the teleoperated grasp truth (08-14). **0.8108 is void** |
| `GRASP_TIP_Z` | 0.8001 m | truth sample with `grip=124`, the only one inside the holding band (08-14) |
| image scale | 1.84 px/mm | `arm_2` 8° moves the tcp 59.0 mm and the pads 777 px/rad (08-14, ZED view) |

The white table **has castors**. Its position is not a constant --- re-measure
where the block is every session. Table *height* does not change when it rolls;
table *position* does.

## Subsystems

| Subsystem | State | Measured |
|---|---|---|
| Robot host access | **BLOCKED.** The existing `astrabot` shell is alive, but `/bin/ls`, `/usr/bin/id`, Python, systemd and workspace traversal all return `Permission denied`; new SSH sessions cannot start `/bin/bash`. Repository sync and ROS validation require local-console or administrator repair of the exact directory modes/ACLs/mounts. Do not use recursive `chmod` and do not reboot away the only diagnostic shell. Evidence: `data/snapshots/20260819-131956-host-permission-failure.json` | 08-19 |
| Arms | 7 DoF each, CAN: left node ids 11-17 on `can1`, right 21-27 on `can2` | 08-07 |
| Joint feedback | `/joint_states` is in **radians** (factor 1.0000). Invalid until you publish one command --- publish first, then trust it | 08-13 |
| Force/contact sensing | Joint `effort` remains `.nan`, so there is no arm force feedback. Two pressure patches are physically installed inside the gripper and have produced operator-tested data; software capture/decision adapters now exist, but live protocol/mapping/threshold calibration is still required | 08-19 |
| Grippers | UFactory G2 over Modbus RTU: right on `/dev/ttyUSB0`, left on `/dev/ttyAMA5`, 2 Mbaud slave 8, driver `g2_gripper_pc`. 0 = open, 1 = close, 840 mm travel | 08-10 |
| Head | pitch must be at the **+40° limit** (reads 39) or the ZED and the arm's reach do not overlap. yaw **pinned to 0** --- 40° of yaw is a half-metre localisation error | 08-11 |
| ZED 2i | owned by `Astrabot_ZED.service`; never open `pyzed` directly. After the 10:43 timeout and an explicitly authorized ZED service restart, read-only captures at 11:45 and 12:28 succeeded. The latest has head pitch 0.681 rad, RGB/depth delta 0 ms and depth valid ratio 0.879. Evidence: `data/vista_runs/yellow-block-harness/observations/20260819-122807-895062316-1623112/` | 08-19 |
| Right-hand near field | RealSense D405 serial `262422270599` is physically installed. A bounded `848x480@10` capture adapter now requires a sustained fresh aligned stream; the 480 Mbit/s link remains unvalidated for repeatable operation | 08-19 |
| Gripper pressure | Two internal pressure patches are physically installed. The adapter supports explicit tty or user-space CH340/PyUSB access and two-pad median/MAD sampling; exact USB path, frame fields, pad mapping and thresholds remain unverified on the current runtime | 08-19 |
| Wrist cameras | two DECXIN monocular units, distinguished **only** by hub port 4.3 / 4.4. Swapping the cables silently swaps left and right | 08-11 |
| Recorder | Mac at 192.168.123.138, `py/xr1_cam.py`. **Exclusive** --- `stop` from another session silently voids the running experiment | 08-11 |

## What works

- **Task proposal → perception → grasp plan**: `xr1-vision observe` then
  `xr1-vision plan`. `TaskProposal` schema v2 carries the natural-language
  command, target/destination relations and success predicates; schema v1 files
  are upgraded at the boundary. The deterministic task executive covers
  observe through verify/place/diagnose as a replayable event state machine.
  The colour mask separates the block from the green cube *and* from the
  gripper's own orange pads (3 tests, thresholds measured off named frames).
  `plan --proposal FILE` accepts the typed semantic task; Rust
  retains ownership of closing axis, full-circle roll search, IK and ranking.
- **Rate-limited motion**: `py/astra_arm.py` ramps from the measured pose, caps
  velocity, clamps to the live URDF, and refuses on stale feedback or a busy
  command channel.
- **Visual-servo transaction components**: Rust extracts the physical pad and
  pinned target signal, fits a named 3×3 Jacobian from +/- samples, caps each
  proposal at 0.05 rad, and reuses the URDF limit/margin and 21-sample
  fingertip-floor envelope. The Python ROS boundary can execute exactly one
  approved microstep; Rust then reconciles predicted versus actual signal and
  stops on a direction reversal or three reductions below 10%. Required D405
  or tactile capability still fails closed unless hardware is `HEALTHY`.
- **Bounded visual-servo orchestration**: `servo-loop` now composes fresh
  observation, deterministic proposal/gates, at most one microstep, a distinct
  post-action observation and reconciliation. It defaults to dry-run, enforces
  one active loop plus step/time bounds, checks endpoint error and never manages
  services. The current Jacobian is still not hardware-validated.
- **Bounded near-field/contact orchestration**: `d405-observe` accepts only a
  fresh real D405 frame; `tactile-assess` evaluates two named pressure patches;
  `grasp-loop` checks D405 alignment and the open baseline, moves the jaw by at
  most 0.05, then requires a new pressure sample. Balanced contact holds,
  imbalance stops and overpressure authorizes only one release increment. It
  defaults to dry-run and never manages a service.
- **One successful grasp**, 2026-08-18. Criterion was a *static* reading plus a
  lift: 149 closed on the object vs 14 closed on air, still 148 after lifting.
  "The gripper closed" is not a criterion.

## Blockers, highest leverage first

1. **Grasp success is orientation-dependent.** Nothing was fixed to get the
   08-18 success; the block's yaw changed, which put the tool-frame error
   across the closing axis. The next experiment is to place the offset *parallel*
   to the closing axis and try to falsify this.
2. **The tool-frame error is partly a rotation.** Measured against the camera over
   8 poses (`py/pad_offset_measure.py solve`): **+38.7 mm ± 17.0 along the tool's
   long axis, sign-stable in all 8**, and x/y that flip sign at the same
   magnitude. A constant cannot produce that, so no single number fixes it ---
   it needs a multi-pose hand-eye solve. Every gate compares FK to FK and
   therefore cannot see any of it.
3. **The live visual-servo transaction is not hardware-validated yet.** Signal
   extraction, Jacobian fitting, one-step execution and prediction
   reconciliation now have connected commands, but no current six-perturbation
   Jacobian or executed/re-observed microstep has been recorded on the robot.
4. **Reach boundary.** At tol=70, z=0.8001, y=-0.157: x = 0.592 and 0.560 have no
   solution; 0.530, 0.500 and 0.470 solve at 0.00 mm residual. The wall is at
   x ≈ 0.54, so pushing the block ~10 cm to x ≈ 0.49 is the cleanest fix
   (13 waypoints). **`py/xr1.py` has no base control**, and the base will not
   drive itself: `Push Mode State = PRESSED` is a physical DIO switch (pin 27)
   that cuts torque to the wheel motors, so `/ecu/cmd_vel_smoothed` zeroes every
   frame and `/odom` does not move. That is the robot correctly refusing to drive
   while someone might be pushing it. It also means **pushing it by hand is the
   zero-risk path** --- it is already in the mode designed for that. Tape-mark the
   station afterwards ([ADR 0005](../decisions/0005-automatic-reset-is-the-ceiling.md)).
   (08-17)
5. **Self-evolution throughput is capped by reset**, not by the policy: automatic
   reset versus manual is ~10x, and at a 30% success rate the human labour
   exceeds the robot time. What that forces on the task, the blocks, the fence
   and the base station is
   [ADR 0005](../decisions/0005-automatic-reset-is-the-ceiling.md).

## Known gaps

Not blockers --- things the code does not do, written down so the absence is not
mistaken for coverage.

| Gap | Consequence |
|---|---|
| Gripper body has no collision model; only the fingertip pose is floor-checked | a plan clearing the table by a few mm is unverified, not safe ([kinematics](../architecture/kinematics.md)) |
| The tool-frame error is open and partly rotational | grasping is orientation-dependent; the 08-18 success is not repeatable at another block yaw ([ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md)) |
| Visual servo has no current hardware-validated Jacobian/step record | bounded orchestration exists, but physical convergence is unproven until a +/- measurement and execute/re-observe/reconcile record are captured; [implementation status](../development/visual-servo.md) |
| Task executive is replay-only | live orchestration still has to turn each real observation/action/evidence result into the same ordered event contract |
| D405 stream plus pressure USB path/protocol/mapping/thresholds lack a current live calibration record | near-field and contact-dependent execution remains fail-closed even though the software path exists |
| No `cargo deny` / `nextest`; `cargo audit` cannot run on rustc 1.75 | licence drift is unchecked; advisories are covered instead by `bin/audit-deps` (0 of 41 crates vulnerable) ([building](../development/building.md)) |

## Recovery

| Symptom | Fix |
|---|---|
| Almost-empty ROS graph | `export ROS_DOMAIN_ID=12`. Not a robot fault |
| `/joint_states` silent, publisher alive | restart `Astrabot_Controller`. Raising the arm first is impossible --- `astra_arm` will not even construct |
| Body TF gone, perception cannot get `base_link` ← `zed_camera_link` | `robot_state_publisher` is alive but its DDS participant never joined. Start a second `rsp`; do **not** restart the controller |
| Fewer than 6 `zed_*` TF frames (`bin/tf-frames`) | restart `Astrabot_ZED.service`. `tf2_echo base_link zed_camera_link` will tell you everything is fine |
| Gripper reads silent, driver process alive | the device node was re-enumerated and the driver holds a stale fd. `ls -l /dev/tty*` newer than `ps -o lstart=` proves it. Only `kill -9` works; `bringup` cannot fix it and its `STILL SILENT` message misleads |
| Gripper command "ignored" right after start | DDS discovery had not completed, so it was dropped. `xr1.py grip()` waits; a log line `before=None` is the fingerprint |
| Arm below the table, every plan refused | `py/xr1.py home` --- but note `home` itself does **not** pass collision checks, and the right arm at zero is 3.0 mm from the torso |
| rclpy script will not die | `timeout N python3` is **not** a bound (one ignored it for 2 h 25 m). Only `kill -9`. Put the deadline inside the loop |

More failure modes, with the reasoning that identified each one:
[`pitfalls.md`](./pitfalls.md).
