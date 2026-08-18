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
