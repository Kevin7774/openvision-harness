# ADR 0004 --- The tool-frame error is open, part rotation, and gates do not detect it

- Date: 2026-08-18
- Status: Open

## Context

`tcp_link` in the vendor URDF is **one finger**, not the midpoint between the
two. `kinematics.rs` corrects for that with a measured offset
(`TIP_CENTER_M = [-0.0225, 0.0, 0.0485]`) and derives the two pad-inner points
from it.

The history of that correction:

- 08-14: aiming was accurate to 2.2 mm by FK and the gripper still closed on
  air. FK claimed a two-finger midpoint ~190 mm from the real fingers.
- 08-18: with `TCP_TO_TIP` corrected to 168 mm, **~21 mm of error remains** ---
  an FK-side estimate from the Python that no longer exists, superseded by the
  camera measurement below.
- 08-18: one grasp succeeded --- because the block's yaw happened to put that
  21 mm *across* the closing axis, where a 46.5 mm jaw gap absorbs it. Nothing
  was fixed. A different yaw fails again.

The reason it survived 38 attempts is structural: **every gate compares FK to
FK.** `pads_bracket_object`, the floor check, the clearance test --- all of them
ask the kinematic model where the pads are, and the model is the thing that is
wrong. They agree with each other perfectly and with the robot not at all.

It becomes visible the moment you compare FK to the *camera*, and it is
observable **before the gripper closes**. `py/pad_offset_measure.py` does this:
it projects the FK pad points into the frame and finds the physical pads by their
orange colour. Do not use "the two largest orange blobs" --- the fake fruit on
the table is bigger than a pad, and picking it reports a 216 mm offset.

## Measurement (2026-08-18, 8 poses, offline)

`pad_offset_measure.py solve` was run over every observation on disk that has
both pads in view, the image-time TF and valid depth --- 8 of 14 candidate poses.
Per pose it unprojects the physical pad midpoint at its own measured depth and
expresses the FK error in the **tool frame**, where a constant model offset must
come out the same every time:

| axis | mean | std | per-pose range |
|---|---|---|---|
| x | −6.2 mm | 22.1 mm | −40.8 … +26.3 |
| y | +49.4 mm | 36.7 mm | −13.1 … +102.4 |
| z (tool long axis) | **+38.7 mm** | 17.0 mm | +14.4 … +70.7 |

Read it as two separate results:

1. **z is sign-stable across all 8 poses.** The modelled pads sit consistently
   ~39 mm short along the tool's long axis. This is the part that behaves like a
   translation.
2. **x and y flip sign.** A constant tool-frame offset cannot do that. There is a
   rotation component, and it is the same size as the translation --- so adding a
   constant to `TIP_CENTER_M` cannot fix this, which is what the decision below
   already said on one pose's evidence and is now measured on eight.

Magnitudes are soft in one specific direction: the depth sample is the pad's
**front surface**, and the midpoint of the two blobs leans toward the near jaw
(810 px versus 337 px at a typical angle). Both biases inflate the numbers.
The signs, the sign-stability of z, and the spread structure are the result.

Reproduce with `python3 py/pad_offset_measure.py solve data/vista_runs/*/observations/*/`
(frames are gitignored evidence; `--check` runs the offline self-check).

## Decision

Do not tune the constant. One pose cannot separate a constant tool-frame offset
from a rotation error, and eight poses now show both are present at the same
magnitude: sign-stable in tool z, sign-flipping in x and y.

A second, separate thing to leave alone: `status.md` used to list
`TCP_TO_TIP = 0.168 m` as an authoritative constant. It is a tape measurement
from 08-14 that lived in a script which no longer exists, it disagrees with
`TIP_CENTER_M` by 119 mm, and the camera puts the residual at tens of mm rather
than 119. **Do not paste it into the code.** It is now labelled as not-in-force
rather than deleted, because the tape measurement itself may still be right about
something --- the flange geometry --- and the disagreement is the open question.

The next experiment is a **falsification**, not a fix: place the block at a yaw
that puts the remaining offset *parallel* to the closing axis. If the grasp fails
there and succeeds at the 08-18 yaw, the error is confirmed as a fixed
tool-frame vector and can be solved for from several poses. If it fails at both,
it is rotational.

Until then: **grasp success is orientation-dependent**, and no gate in this
workspace will warn you.

## Consequences

- Reported grasp success rates are meaningless without the block yaw recorded
  alongside. `xr1-vision plan` prints the footprint orientation; write it down.
- The only honest cross-check available is FK-versus-camera. Any new gate that
  compares two FK-derived quantities adds confidence without adding information.
- The 8-pose numbers above are the baseline. Any change to the tool model must
  re-run `pad_offset_measure.py solve` on the same frames and reduce them ---
  a change that only improves one pose has not been tested.
- Fixing this properly probably needs the multi-pose solve, i.e. a hand-eye
  calibration over ≥ 8 poses. The tooling for that is
  `py/pad_offset_measure.py` plus `xr1-vision fk`.
