# Visual servo --- specification for the highest-value missing piece

**Status: the one-action transaction is implemented; live hardware validation
is incomplete.** The deleted Python loop remains historical context in
[ADR 0003](../decisions/0003-lost-python-pipeline.md). Rust now owns physical
pad/target signal extraction, central-difference 3×3 fitting, conditioning,
the 0.05 rad hard step ceiling, observation binding, freshness, URDF
limit/margin, fingertip-floor path and predicted/actual reconciliation. Python
only publishes the one approved joint microstep.

Inspect a saved observation, fit the local gain, propose one step, and keep the
target signal from the first unobstructed frame pinned:

```bash
xr1-vision servo-observe --latest data/vista_runs/<run>/latest.json \
  > /tmp/servo-signal.json
xr1-vision servo-calibrate --input /tmp/plus-minus-samples.json \
  > /tmp/servo-calibration.json
xr1-vision servo-propose \
  --request /tmp/calibrated-servo-request.json \
  --state data/vista_runs/<run>/observations/<frame>/state.json
```

`servo-propose` never publishes and keeps `execution_authorized=false`. A true
`ready_for_execution_adapter` means the Rust checks passed. Save that exact JSON
and dry-run the hardware boundary before the explicit action:

```bash
xr1-vision servo-step --proposal /tmp/servo-proposal.json
xr1-vision servo-step --proposal /tmp/servo-proposal.json --go
# capture one new frame; never execute a second step first
xr1-vision observe
xr1-vision servo-observe > /tmp/servo-after.json
xr1-vision servo-reconcile --input /tmp/reconciliation.json
```

The adapter re-checks envelope age, live `/joint_states`, start drift and
command-channel idleness. A real action returns `requires_reobservation=true`.
Self-collision and gripper-body collision are still unmodelled by this servo
gate and remain explicit limitations.

## Why this and not more calibration

Every gate in this workspace compares FK to FK, so all of them agree with each
other and none of them sees the tool-frame error
([ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md)). A pixel-space
loop closes on what the camera sees, and **differencing cancels any constant
extrinsic error** --- it does not need the constant to be right. When it ran, it
took pad↔block error from 106.9 px to 9.1 px (≈ 4.9 mm) in four steps, with a
predicted/actual gain ratio of 0.94--0.96, using **none** of the three
calibration constants.

## Measured inputs that survived

**One column of the image Jacobian has been measured.** Rotating `arm_6` by −30°
alone moved the pads 252.2 px and `tcp` 140.7 px --- ratio 1.79. That measurement
was originally a *diagnosis* (a ratio ≠ 1.0 means the offset hangs rigidly off
the wrist, so `TIP_CENTER` is wrong). The same procedure over three joints is a
3×3 gain matrix. `py/pad_offset_measure.py` performs the swing.

**Pad detection is now one Rust implementation.** `servo-pads` and
`servo-observe` use the same measured RGB threshold, FK-local search and
35--95 mm open-pad separation gate. The named regression frame
`20260818-120701-385142786-132823` finds midpoint `(726.0, 410.9)` and rejects
the larger orange fruit. `py/pad_offset_measure.py` consumes this Rust result;
it no longer carries a second colour detector.

## `servo-calibrate`: fit the gain in place

Perturb three joints by ±a small angle around the current pose, one frame each,
and difference:

```
s = [u_block, v_block, depth_at_block]     # signal
q = [three joint angles]                   # control
J = ds/dq                                  # per joint, average both directions
```

Valid only near the pose family it was measured at --- re-measure after moving.
Six small motions plus the centre frame, tens of seconds; three orders of
magnitude cheaper than a global calibration. The command requires exactly one
unique +/- frame pair per named joint, checks that the centre is bracketed,
reports unintended background-joint drift, and rejects a singular/ill-conditioned
matrix before it can reach `servo-propose`.

## `servo-step`: one iteration

```
1. observe
2. detect pads + block; e = s_target - s_now
3. dq = 0.5 * J^-1 * e                     # damping 0.5; do not take the whole step
4. propose to the existing deterministic gates -- freshness, step, URDF margin,
   fingertip floor path and required sensor capability.
   The servo proposes; the gates dispose.
5. execute exactly one step, re-observe, and run `servo-reconcile`
6. converged when |e_px| < threshold and depth is in the grip band -> suggest close
```

Feed that reconciliation into the task executive as a
`servo_step_reconciled` event. Its `before_frame_id` must equal the executive's
current observation and its `after_frame_id` must be new. A valid unfinished
step stays in `SERVO`; convergence enters `GRASP`; prediction mismatch or stall
enters `DIAGNOSE`. This makes observe -> act once -> observe -> reconcile the
same transaction in both the servo contract and the task state machine.

Three things that are not optional:

- **`J` must be square.** A 2×3 Jacobian is degenerate in z and `pinv` on it
  *lifts the arm*: seven steps once undid 70 mm of hard-won descent, then put it
  back. Put `dz` in the error vector; the z row comes from numerical
  differentiation of the existing FK, so it introduces no new constant.
- **Pin the target pixel.** The pads occlude the block from above, so the visible
  centroid creeps ~3 px per step until the target is lost. Judge "did it move"
  from the top edge, which is never occluded, not from the centroid.
- **Stop on stall**: three consecutive steps with < 10 % reduction in `|e|` ⇒
  stop and report. The timeout goes *inside* the loop --- `timeout N python3` does
  not bound an rclpy process (it once ran 2 h 25 m past its deadline and ignored
  both SIGINT and SIGTERM).

Keep the gripper **open** throughout; `close` is the last action. Two blobs are
what makes the midpoint computable.

`damp = 0.5` halving each of the three error components *exactly* is the
fingerprint of an exact solve --- it is the cheapest acceptance check that the
square system is wired up correctly.

Two approaches that look reasonable and are not:

- **Alternate descend and servo.** With a 2×3 `J` this is a staircase that walks
  back down every step it climbs. Descent and pixel correction are one action.
- **Pre-compensate the parallax with a bias, then descend blind.** The bias
  vector *is* the image signature of the z change, so the servo obediently undoes
  the descent. It is self-referential; it was tried and it failed.

## How it will fail

| Risk | Symptom | Response |
|---|---|---|
| ZED views the workspace obliquely at pitch +40°, low pixel SNR | ill-conditioned `J`, non-monotonic `\|e\|` | use ZED for coarse alignment only; do not expect mm convergence from it |
| pads merge into one blob | separation outside 35--95 mm | stay open until the final close |
| `J` is only locally valid | predicted and actual signs disagree | damping 0.5 plus a per-step angle cap; on a sign flip, stop and re-measure `J` |
| the servo asks for a pose that is geometrically unreachable | a gate refuses | that refusal is real information --- report it and ask for the block to be moved |
| IK is minutes per call | --- | measure single-step cost first. If > 30 s/step, add `dq` in joint space directly and use the gates only to verify |

## Offline self-check

The test suite runs the detector over the named 2026-08-18 frame, verifies that
the unrelated orange fruit is rejected, reconstructs a known 3×3 matrix from
six central-difference samples, checks that `e = 0` yields `dq = 0`, and tests
sign-flip and three-step stall termination. The Python adapter tests prove a
dry run cannot silently become a physical action. Seconds, not minutes.

## Then: stacking is easier than grasping

"Put A on B" needs two world coordinates in the geometry chain --- two extrinsic
errors. In pixel space it is a single alignment: drive A's centroid above B's,
match depth, release. Both blocks in one frame means the shared camera error
cancels. Same `step`, different target signal.
