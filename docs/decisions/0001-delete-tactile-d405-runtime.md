# ADR 0001 --- Delete the tactile, D405 and multi-sensor-fusion code

- Date: 2026-08-18
- Status: Accepted

## Context

The vision crate contained `tactile.rs`, `d405.rs`, `fusion.rs` and a
`runtime.rs` that wired them into a three-sensor pipeline (ZED + wrist depth +
tactile pads), plus `policy/` and `local_model.rs` for a learned grasp policy.

None of it can run on this robot:

- **Tactile: there is no sensor.** `effort` is `.nan` on every joint of both
  arms, and the G2 grippers report position only. There is no channel a tactile
  reading could arrive on.
- **D405: not installed.** The right wrist carries a **D455**. Its minimum depth
  is ~250 mm, which is exactly the range the fusion code was meant to cover, and
  it publishes **0 TF frames**, so its geometric contribution is zero even when
  its topics are alive. A D405 would fix the range; nobody has bought one.
- **Fusion of one source is not fusion.**
- The policy path had no weights on disk and no training data (0 recorded
  episodes).

Together they were ~2000 lines that compiled, read as the architecture, and had
never executed against hardware. A new agent reading the crate would conclude
the robot has touch sensing.

## Decision

Delete all of it. `crates/xr1-vision` keeps exactly two modules, `perception`
and `kinematics`, both of which run.

Contact is inferred **geometrically** --- `pads_bracket_object` plus the
fingertip floor check --- and that limitation is stated in the README rather
than papered over by code for a sensor that isn't there.

## Consequences

- The last ~25 cm of a descent is open loop, and now says so.
- Reinstating tactile means buying hardware first; reinstating close-range depth
  means a D405, and publishing wrist-camera TF frames.
- The deleted code is in git at `5999a05` if anyone wants the shape of it. It is
  not a starting point: it was written against sensors that were never measured.
