# ADR 0003 --- The old Python pipeline is gone, and the docs described it

- Date: 2026-08-18
- Status: Accepted (a record, not a choice)

## Context

The markdown in this workspace (11,397 lines before consolidation) documented
roughly **95 Python scripts**. Five exist.

A whole-disk search of `/home/astrabot` and `/opt/ros` found no copy of
`grasp_block.py`, `servo.py`, `agent_loop.py`, `agent_runner.py`,
`zed_perception.py`, `teleop_truth.py`, `xr1_verify.py`, `plan_descent.py` or
`home_safely.py`. They are not in `_attic_20260811/` (that directory is itself
gone), not in any backup, and there was **no version control in this workspace
before 2026-08-18**. They are unrecoverable.

This is not a small documentation drift. `CLAUDE.md` opened with "always use
this 15-step loop, driven by `scripts/agent_loop.py`" --- an entry point that
does not exist. Every runbook, every pitfall entry and every worked example
named a file that is not on the disk. An agent following those docs spends its
session discovering that, one `No such file` at a time.

The single worst loss is the **pixel servo**: the only closed loop this project
ever verified end to end (pad↔block pixel error 106.9 → 9.1 px ≈ 4.9 mm,
predicted/actual gain ratio 0.94--0.96, three calibration constants bypassed
entirely). Its *measurement* half survives as `py/pad_offset_measure.py`. Its
control half does not.

## Decision

1. Do not attempt to reconstruct the scripts from their documentation. The docs
   describe behaviour at the level of "and then it descends"; the parts that
   mattered were the gains, the seed sets and the rejection thresholds, and none
   of those are written down anywhere.
2. Rewrite the docs from **surviving code plus durable measured facts**, not by
   correcting the old text in place. Anything not backed by code that runs or a
   measurement with a date is deleted, not softened.
3. Keep the measurements. A number like "table top = 0.790 m, from the
   teleoperated pose that actually gripped" outlives the script that printed it,
   and is carried into `docs/operations/status.md`.
4. Git exists now, from `5999a05`.

## Consequences

- The workspace can localise the block and solve a grasp, and cannot yet close a
  visual loop on it. That is a real regression against the ledger, and it is
  stated in the README rather than hidden.
- Re-implementing the servo is the highest-value work available, in Rust, with
  the spec that survives in the ledger: solve `du, dv, dz` as one square system
  (a 2×3 Jacobian is degenerate in z, and `pinv` on it *lifts the arm* --- seven
  steps once undid 70 mm of descent), and pin the target pixel rather than
  re-detecting it, because the visible centroid drifts ~3 px per step upward as
  the pads occlude the block from above.
- Historical script names appear all over `docs/operations/pitfalls.md`. They are
  kept as *evidence labels*, with a translation table at the top of that file,
  because the lesson attached to each one is still true.

## Addendum (2026-08-18) --- what survived, outside the workspace

An older, *different* Python stack lives in `/home/astrabot/tools/`, outside
this workspace and outside git. It is not the lost pipeline above; it predates
it (2026-08-05/06) and it is the layer the vendor stack does not provide:

| file | capability | in this workspace? |
|---|---|---|
| `astra_arm.py` | arm/gripper command + rate limiting + channel-idle refusal | yes --- copied to `py/astra_arm.py`, imported by `py/xr1.py` |
| `astra.py` | skill-SDK facade over the rest (`bot.find()`, `bot.move()`) | no |
| `astra_cams.py` | publishes the three V4L2 cameras onto the `/astrabot/data_sources/image/*` topics nothing else publishes | no |
| `astra_detect.py` + `fetch_owlv2.py` | open-vocabulary detection (OWLv2, weights fetched from ModelScope because huggingface.co does not resolve here) | no --- `xr1-vision` detects by colour + plane only |
| `astra_neck_raw.py` | reads the neck servos over Modbus RTU, bypassing ROS entirely (answers "is torque on?" before bring-up) | no |
| `astra_task_server.py` | serves `execute_task`, the vendor behaviour-tree skill socket | no |
| `astra_snap.py`, `rosq.py` | one-frame grab; structured ROS health queries | partly --- `py/xr1.py snap`, `bin/tf-frames` |

These are **not** part of the architecture: none of them is referenced from
anything in this workspace, and none has been re-verified on hardware in this
cleanup. They are left where they were found, unpromoted, because each row with
"no" is a capability the Rust line does not have yet.

`~/tools` was found *missing from disk* on 2026-08-18 at 21:20 --- something
outside this session removed it after 20:08, and several sessions share this
machine. It was restored from `.before-cleanup-outside-workspace.tar.gz`, and
that tarball was then deleted from the workspace and purged from git history:
it carried `deploy/livekit.env` (LiveKit API key and secret) into the repo, and
every other non-backup file in it was byte-identical to the live original under
`~/config`, `~/deploy`, `~/gripper_ws` (checked with `cmp`, 23/23 identical).
`go_zero.py` / `go_zero_head.py` were deliberately *not* restored --- `py/xr1.py
home` and `look` ramp the same joints with limit clamping and the channel-idle
check, so they were duplicates of in-workspace capability.
