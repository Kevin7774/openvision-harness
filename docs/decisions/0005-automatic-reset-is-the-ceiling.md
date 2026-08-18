# ADR 0005 --- Automatic reset is the ceiling, and it constrains everything upstream

- Date: 2026-08-18 (analysis 2026-08-11)
- Status: Accepted

## Context

The goal is a self-improving loop: the robot attempts a task, a judge labels the
attempt, and the data trains the next policy. Two things determine whether that
loop is a data engine or an expensive random search, and neither of them is the
policy.

### The judge has to be an order of magnitude better than the policy

That phrase is arithmetic, not rhetoric. At success rate `p`, the standard error
of the estimate over `n` episodes is `SE = sqrt(p(1-p)/n)`. A judge with a
*systematic* bias `b` stops adding information once noise falls to `b`:

```
n* = p(1-p) / b²
```

At `p = 0.30`:

| judge bias `b` | `n*` |
|---|---|
| 5 pp | **84** |
| 2 pp | 525 |
| 1 pp | 2 100 |
| 0.5 pp | 8 400 |

So a 5 pp-biased judge makes every episode after the 84th worthless for deciding
whether anything improved --- while the loop keeps running, keeps burning robot
hours, and keeps producing a curve that looks perfectly normal. That is the most
expensive failure mode available here.

### Manual reset caps the sampling rate at the speed of a human hand

Calibrating against the observed 60--100 episodes/hour under manual reset gives
robot time `T_r ≈ 15 s` and human reset `T_h ≈ 20 s` per attempt.

| | ep/hour | hours/week | ep/week | duty cycle |
|---|---|---|---|---|
| manual reset | ~103 | 30 (6 h × 5 d; unattended time is *zero*) | ~3 100 | 43 % |
| automatic reset | ~240 | 134 (24 × 7 × 0.8 availability) | ~32 000 | 100 % |

The two gains **multiply**: 4.5× from duty cycle (unattended hours go from
worthless to productive) times 2.3× from rate (reset stops being overhead and
becomes another episode). ~10×, or ~8× being conservative. That is the difference
between a research demo and a data engine.

Worse, at `p = 0.30`, reaching 1000 *successful* episodes needs 3333 attempts:
13.9 h of robot time and **18.5 h of human time**. Successes need resetting too.
The entire value proposition of self-evolution is that it can afford attempts that
are doomed; that proposition is void the moment the marginal cost of an attempt
is a human-minute.

## Decision

**Design for automatic reset first.** Concretely, and none of these are free
choices once that is fixed:

| Constrained | Constraint |
|---|---|
| **Task** | pick one whose **inverse is itself a legal episode** --- scatter ↔ line. Otherwise reset always needs a human. The scatter must be *placed*, not shoved: a push has no reachable inverse |
| **Blocks** | cubic/symmetric, so reset restores position only and not orientation (there is no orientation feedback); heavy and grippy enough not to skitter; sized with margin against the 840-unit jaw |
| **Table** | 🔴 **a physical fence.** A 15 mm lip turns "on the floor, needs a human" into "still on the table, the robot can recover". With no contact feedback anywhere, the robot cannot notice it pushed something off, so the boundary must be **geometric, not perceptual.** Cheapest, highest-leverage hardware change available |
| **Base position** | move it once, then **tape-mark it on the floor** and never move it again. There is no reliable base↔table registration, so if the station moves, the workspace spec and the judge's extrinsics both silently expire --- no sensor reports it. Moving must happen *before* calibration, never after |
| **Head pose** | `head_pitch` pinned at the +40° limit for the entire loop. Every judgement must be made at the same head pose or the judge's bias drifts with head motion --- and per the table above, a few tens of episodes of drift is enough to void the data stream |

**The judge is layered, because the sensors force it to be:**

- Scene level ("are five blocks in a line", "is each within radius r") → **head
  ZED at one fixed inspection pose**. Centimetre-scale global geometry, which is
  exactly the capability that is already verified.
- Grasp instant ("did it grab it") → **wrist camera plus `pos_mm` stall**. ZED
  suffers parallax and occlusion at this scale, and the gripper hides the block.

`pos_mm` stall is the **only non-visual independent channel on the whole robot**,
and it happens to answer the one bit most in need of cross-checking. Do not waste
it. Everything else --- block placed, block dropped, gripper hit the table --- is
visual inference only.

Two channels plus **abstention**: when they disagree, label `uncertain` and keep
it out of the training set. Track the abstention *rate* as a first-class metric
rather than trying to drive it to zero --- a sudden drop usually means the judge
learned to be confidently wrong. Keep a frozen golden set that never enters
training and re-run it on every judge change; without it, judge drift is
invisible, and in a positive feedback loop an invisible drift is fatal. Human
spot-checks go near the decision boundary, not uniformly.

## Consequences

- The acceptance gate for the loop is **mean time between human interventions**,
  not success rate. The target is MTBH ≥ 8 h.
- Physical work (fence, base station, block selection) comes **before**
  calibration, and calibration before any loop metric means anything.
- The tape mark is a hidden state variable, not a setup step. It was skipped once
  and cost a full session.
- What this analysis does not answer: how the judge itself gets trained or
  validated beyond the golden set, and how the curriculum knob (scatter
  difficulty) should be scheduled.
