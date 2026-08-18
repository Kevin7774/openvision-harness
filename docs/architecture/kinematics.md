# Kinematics and grasp geometry

`crates/xr1-vision/src/kinematics.rs`. Pure computation over the URDF; it never
publishes anything.

## The chain

`Chain::from_urdf(path, "right_tcp_link")` walks parent links from the tip back
to the root and keeps the joints in order, with their origin transform, axis and
limits. Fixed joints contribute their origin; movable joints contribute an axis
rotation.

The URDF read is the **vendor's installed** description:

```
/opt/ros/astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_arm_description.urdf
```

⚠️ That file and the description actually *running* can diverge. When geometry is
what you are arguing about, compare against the live one:

```bash
ros2 param get /robot_state_publisher robot_description
```

Joint limits therefore come from the robot's own model, not from a table
maintained here --- which is the only reason the clamp can be trusted.

## Tool geometry: `tcp_link` is one finger, not the centre

Four constants, all in `right_tcp_link` coordinates:

| Constant | Value (m) | What it is |
|---|---|---|
| `TIP_CENTER_M` | `[-0.0225, 0, 0.0485]` | the point the IK actually aims, midway between the jaws |
| `FIXED_PAD_INNER_M` | `[0.0015, 0, 0.0485]` | inner face of the fixed pad |
| `MOVING_PAD_INNER_OPEN_M` | `[-0.0450, 0, 0.0485]` | inner face of the moving pad, jaws open |
| `OPEN_JAW_GAP_M` | `0.0465` | usable gap between them |

`tcp_link` sits on **one finger**. The lateral -22.5 mm and the vertical +48.5 mm
are one vector and must be applied together --- applied separately they fight
each other whenever the wrist is tilted. `contact_pose()` is the only pose the
solver aims at, so this offset is applied once, in one place.

There is a **measured residual error** between where this model says the pads are
and where the camera sees them, and it is visible *before* the jaws close
(`py/pad_offset_measure.py solve`, 8 poses): the modelled pads are ~39 mm short
along the tool's long axis in every pose, with an x/y component of the same size
that flips sign --- i.e. part translation, part rotation. All the gates below
compare FK against FK, so they cannot see any of it; only the camera can.
See [ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md).

## Position IK

`solve_position_with_reference(target, seed, reference, orientation_offset)`.

The target *rotation* is expressed as an offset from a reference pose rather than
as an absolute orientation. That is deliberate: the useful question is never
"reach this quaternion", it is "keep roughly this wrist attitude but line the
closing axis up with the object", and stating it as a delta from a pose that is
already known to be reachable keeps the solver in a sane basin.

Per attempt: damped-least-squares over a numerically differentiated 6-DoF
Jacobian, steps clamped to 0.08 rad, joints clamped to URDF limits every
iteration, converged at 4 mm and 0.04 rad, up to 160 iterations. It runs from
`3 + 2N` seeds (current pose, joint-range midpoint, a 0.7/0.3 blend, and that
blend perturbed ±0.45 rad on each joint in turn) and keeps the best-scoring
result:

```
score = 1000*residual + 20*orientation_residual + 4*max_joint_delta
        + limit_penalty(margin < 0.12 rad) + 1000*(not floor_clear)
```

Two things matter about that score. `max_joint_delta` prices in "how far the arm
has to travel", so a mathematically equal solution that reconfigures the whole
arm loses. `floor_clear` is checked by **sampling 21 points along the joint-space
path** from the current pose and requiring the contact point to stay above
0.785 m --- not by checking the endpoint. IK having a solution and the arm being
able to *get* there are different questions, and the second one is the one that
was failing.

`floor_clear` is a heavy penalty rather than a hard rejection so that a rejected
plan still reports *why*; the caller decides.

### Ask each height separately, and search the pair

Two measured lessons that shaped this, both from 08-17:

**Reachability is per-layer.** At the pregrasp height (table + 133 mm) a point
solves in seconds; the *same* x,y at the grasp height (table + 13 mm) can be
genuinely unsolvable --- 331 s over 8 rounds of free global search, no solution.
The lateral bound at grasp height is tighter than at pregrasp height. "Is it
reachable" is not a property of an x,y; ask at the height you will actually be at.

**The pair is the problem, not the endpoint.** At x = 0.522 a free probe solved
*both* layers at 0.00 mm residual in 22 s, while asking for a descent from the
already-chosen pregrasp solution, constrained to the same branch within
`max_dev = 0.6 rad`, returned no solution after 406 s. The endpoint was reachable
the whole time; what failed was choosing a pregrasp branch that a legal descent
could continue from. So the search must be over *compatible pairs*
`(q_pregrasp, q_grasp)` --- or solve at grasp height and work upward --- and never
by deleting the 0.6 rad, floor or collision gates, which is the reading that
mistake invites.

This is why `plan()` tries several clearances rather than one, and why the
candidate that matters is a whole approach, not a pose.

An approach-angle tolerance of 45° also has to go: the one teleoperated pose that
*actually gripped* the block had an approach angle of **59.5°**, so a 45° gate
calls the ground truth infeasible. Some fraction of the 38 recorded "no solution"
results were the gate being stricter than reality. Widening it to 70° is correct
on its own evidence --- and it did **not** rescue x = 0.589, which is genuinely
out of reach at grasp height. Both things are true; do not use one to argue the
other.

## Grasp feasibility

`grasp_metrics()` answers "if the arm went here and closed, would it get the
object?" using only geometry:

| Check | Threshold |
|---|---|
| pad midpoint to object centre | <= 8 mm |
| closing axis vs the nearest horizontal object axis | <= 0.35 rad (20°) |
| object width along that axis | >= 5 mm |
| jaw clearance (`0.0465 - width`) | >= 4 mm |
| pads bracket the object | signed distances straddle ±half-width |

`pads_bracket_object` is the check that catches the failure the others miss: the
midpoint can be within 8 mm and the axis within 20° while both pads sit on the
*same side* of the block, which closes on air. It is the geometric stand-in for
force feedback, and it is all we have --- `effort` is `.nan` on every joint of
this robot, so contact is never sensed, only predicted.

## Search lessons, paid for four times

An earlier effort built a full reachability grid (30 mm cells, both arms, five
tilt budgets). **Its numbers are gone and should not be reconstructed** --- it was
computed on a table plane of 0.750 m, which is wrong by 40 mm, and every feasible-cell
count and clearance in it is therefore void. What survives is why three earlier
solvers were thrown away, because each reason is a standing constraint on any IK
search written here:

| Method | What it concluded | Why it was wrong |
|---|---|---|
| Uniform 7-DoF sampling with rejection | "a 45 cm dead zone straight ahead" | directed IK found clean solutions inside the "dead zone". The solution manifold is thin enough that 3 million samples missed it entirely. **Not sampled is not unreachable.** |
| Random multi-start IK | 22/136 reachable, then 38/136 | the count was still climbing with more starts, so it measured solver convergence, not kinematics. **A grid that has not converged cannot be used as a boundary.** |
| Warm-start flood, minimising tilt | points with tilt <= 15 deg exactly equalled the non-colliding points | self-contradictory. Pressing the wrist toward vertical drives the elbow into the torso, so positions that are only reachable *tilted* get recorded as unreachable --- and those are exactly the cells a directed probe solves cleanly at 45 deg. **Collision avoidance and tilt minimisation compete; "minimise tilt, then check collision" is the wrong objective.** |

The objective that replaced them keeps tilt and clearance as hinges that cost
nothing while inside budget --- `20*(p - p_target)`, `0.3*max(0, tilt - budget)`,
`2.0*max(0, 40mm - clearance)` --- which is the same shape as the score above:
position is a hard requirement, everything else is a penalty that only switches on
when it is violated.

One measurement from that work is still good: at `q = 0` the FK chain
`base_link -> *_tcp_link` agrees with the robot's own `/tf` to **0.5 mm**. The
forward kinematics are not the problem; see
[ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md) for what is.

## What is *not* checked here

The floor test samples the **fingertip contact pose** only. The gripper body ---
the housing behind the pads --- has no model, so a pose whose fingertips clear the
table by +5 mm may have body geometry 29 mm *below* it. That is not hypothetical:
the deleted servo carried its own gripper-body gate and vetoed a descent that this
floor check passes, and the two walls are independent (a plan can fail either one
alone). Until the body is modelled, a plan that clears the floor by only a few
millimetres is unverified rather than safe. See the debt list in
[../operations/status.md](../operations/status.md).
