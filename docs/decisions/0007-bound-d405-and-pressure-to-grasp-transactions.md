# ADR 0007: bind D405 and gripper pressure to bounded grasp transactions

## Status

Accepted, 2026-08-19. Extends ADR 0006 and supersedes its rule that only a USB
3.x link can become healthy: a newly validated sustained frame may authorize
one bounded transaction even when the persistent USB 2.0 capability is degraded.

## Context

The D405 and two pressure patches are physically installed, but hardware
presence is not a control signal. A reliable grasp needs a fresh image, an
explicit pressure-channel mapping and one new observation after every action.
Joint `effort` remains unavailable and must not be confused with the gripper's
two local pressure channels.

## Decision

- Python owns only librealsense, CH340/tty and G2 ROS boundaries.
- Rust owns timestamps, calibration, contact/balance/pressure decisions and
  authorization of the next jaw increment.
- A D405 sample is accepted only from serial `262422270599`, aligned RGB/depth
  at the exercised `848x480@10` profile, with a sustained-stream and depth-valid
  check plus fresh joint state.
- D405 observations use `d405-*` frame IDs. The final servo refuses a target or
  Jacobian that was not measured from those named frames, and the right-wrist
  camera may authorize only right-arm/right-gripper transactions.
- Pressure capture names exactly two pads through explicit configuration and
  reports median, median absolute deviation and sample count. Reserved gripper
  and neck serial ports are rejected.
- The gripper starts open. Each authorized close changes `close01` by at most
  0.05 and is followed by a new pressure observation. No contact permits one
  further close; balanced two-pad contact holds; imbalance stops; overpressure
  permits exactly one release increment.
- D405 alignment is checked before closure and again before another close.
- Lift and retention verification remain a separate action/observation pair.
- Missing/stale samples, unknown mappings, adapter errors and absent DDS
  subscribers fail closed. No service is started, stopped or restarted.
- Arm servo and gripper-feedback loops share one process-wide action lock, so
  separate sessions cannot interleave arm and jaw transactions.

## Consequences

The software path is complete enough to calibrate and exercise without
inventing signals. It is not live-validated until append-only evidence records
the current USB paths, pressure frame fields, pad mapping, thresholds, D405
target/Jacobian and a dry-run followed by bounded execute/re-observe results.
