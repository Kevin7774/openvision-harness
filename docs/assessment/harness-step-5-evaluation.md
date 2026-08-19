# Vision Harness — Step 5 Evaluation and Rollout

**English** | [中文](harness-step-5-evaluation.zh.md)

Step 4 ([orchestration](harness-step-4-orchestration.md)) closed the last
structural finding. Step 5 addresses **finding #5**, the one Step 1 scored
**0.5/10**: self-evolution existed as reasoning in
[ADR 0005](../decisions/0005-automatic-reset-is-the-ceiling.md) and as nothing in
code.

> [!IMPORTANT]
> **This does not make the harness self-evolving, and the score does not go to
> 10.** What exists now is the *promotion path*: the episode standard, the judge,
> the golden set, the gate, and the shadow/canary/rollback lifecycle. There is
> still **no trainer**, and no episodes from a real robot. The honest claim is
> "the machinery that decides whether a new policy may serve is implemented and
> tested" — not "the robot improves itself."

---

## Why this is not a generic MLOps pipeline

ADR 0005 is unusually specific, so the implementation follows it rather than a
template. Four of its statements became executable code:

**1. "The judge has to be an order of magnitude better than the policy — that
phrase is arithmetic, not rhetoric."** At success rate `p`, a judge with
systematic bias `b` stops adding information at `n* = p(1-p)/b²`. That is
`information_ceiling`, and the test asserts the ADR's own published table:

| judge bias | ADR says | computed |
|---|--:|--:|
| 5 pp | 84 | 84 |
| 2 pp | 525 | 525 |
| 1 pp | 2 100 | 2 100 |
| 0.5 pp | 8 400 | 8 400 |

It is runnable, so an operator asking "should we collect more episodes?" gets the
real answer:

```
$ xr1-vision judge-ceiling --rate 0.30 --bias 0.05
{"ok":true,"episode_ceiling":84.0,"standard_error_at_ceiling":0.05, ...}
```

At the ceiling the standard error equals the bias — which is the definition, and a
useful self-check.

**2. "Two channels plus abstention: when they disagree, label `uncertain` and keep
it out of the training set."** `LayeredJudge` combines the head ZED (scene) with
the wrist camera plus `pos_mm` stall (grasp instant) and abstains on disagreement.
It also abstains when a channel is *unavailable*, because a dead wrist camera must
not read as "did not grasp".

**3. "Track the abstention rate as a first-class metric… a sudden drop usually
means the judge learned to be confidently wrong."** This is the mislabel
positive-feedback guard, and it is the one place where a *better-looking* result is
treated as disqualifying. `AbstainMonitor` returns `SuspiciousDrop`, and the gate
turns that into `Reject` — not `Hold`.

**4. "The acceptance gate for the loop is mean time between human interventions,
not success rate. The target is MTBH ≥ 8 h."** So a challenger with a better
success rate and MTBH of 1.5 h is held, and the reason says
`success rate is not the gate`.

Two further ADR constraints shape the episode record itself: judgements must all
come from **one pinned head pose** (or the judge's bias drifts), and moving the
station **silently expires** calibration. Both become scope checks that refuse
non-comparable episodes rather than averaging them in.

---

## Structure

```
crates/harness-evaluation/
├── src/episode.rs     # immutable Episode + append-only EpisodeLog + FleetScope
├── src/judge.rs       # LayeredJudge (impl OutcomeJudge), bias arithmetic, AbstainMonitor
├── src/golden.rs      # frozen golden set, leak check, judge scoring
├── src/policy.rs      # PolicyArtifact + registry with lineage
├── src/promotion.rs   # the gate: baseline vs challenger
├── src/lifecycle.rs   # Registered → Shadow → Canary → Promoted / RolledBack / Rejected
└── tests/loop_end_to_end.rs
```

`LayeredJudge` implements the `OutcomeJudge` port added in Step 2, so the judge
plugs into the contract rather than a bespoke interface.

### What the gate refuses

`evaluate()` distinguishes "not yet" from "this comparison is invalid":

| Situation | Verdict |
|---|---|
| Fewer than 200 decided episodes per arm | `Hold` |
| Margin ≤ 2× the judge's bias | `Hold`, naming the judge and its ceiling |
| Margin ≤ 1.96σ of sampling noise | `Hold` |
| MTBH below 8 h | `Hold` |
| Abstention above the usable ceiling | `Hold` |
| Abstention collapsed below baseline | **`Reject`** |
| Challenger no better | `Reject` |
| Golden set frozen *after* the challenger | `Reject` |
| Challenger trained on the golden set | `Reject` |

The margin-vs-bias case is worth calling out: past `n*` more episodes cannot help,
so the reason says the **judge** must improve rather than asking for a bigger
sample. That is the failure ADR 0005 calls "the most expensive failure mode
available here" — a loop that burns robot hours while drawing a normal-looking
curve.

There is deliberately **no override parameter**. A promotion that needs one is a
human decision, made outside this type.

### What the lifecycle refuses

```
Registered --shadow--> Shadow --gate--> Canary --gate--> Promoted
                          |               |                |
                          +--- Rejected / RolledBack ------+
```

- Shadow cannot be skipped; a canary without a passing shadow gate is refused.
- A passing shadow gate **does not deploy** — it earns a canary. Two shadow passes
  still leave the incumbent serving.
- Canary share is capped at 25%: a canary large enough to matter to the fleet is
  not a canary.
- **Rollback is available from any live stage and is never gated.** A rollback that
  has to argue its case is a rollback that happens too late.
- `serving_policy_id()` returns the incumbent until the moment of promotion, so
  "which policy is live" is never inferred from the stage name.

---

## Proof

```
cargo test --workspace
  harness-contracts:              16 passed
  harness-evaluation:             52 passed   (48 unit + 4 end-to-end)
  xr1-vision:                     82 passed
  yellow-block-pick-place:         5 passed
  ──────────────────────────────────────────
  total:                         155 passed, 0 failed
cargo clippy --workspace --all-targets:   clean
```

The end-to-end tests drive the whole path rather than each piece alone: 1 200
episodes per arm are recorded, labelled **through the `OutcomeJudge` port**, tallied
by the ledger, compared by the gate, and walked through the lifecycle. Three of
them assert refusals:

- A challenger whose score improved **because the judge stopped abstaining** is
  rejected.
- A canary regression rolls back to the incumbent with no gate.
- Episodes from a second robot — or the same robot at another bench — cannot enter
  the first robot's evaluation.

---

## Which Step 1 items this closes

| Step 1 gap | Status | Where |
|---|---|---|
| Episode data standard | **Closed** | `episode.rs` |
| Outcome judge registry with abstain | **Closed** | `judge.rs`, over the `OutcomeJudge` port |
| Policy / model registry | **Closed** | `policy.rs` |
| baseline / challenger comparison | **Closed** | `promotion.rs` |
| Shadow evaluation | **Closed** | `lifecycle.rs` |
| Automatic promotion criteria | **Closed** | `PromotionCriteria` |
| Canary deployment | **Closed** | `lifecycle.rs`, capped at 25% |
| Rollback | **Closed** | `lifecycle.rs`, ungated |
| Cross-robot data isolation | **Closed** | `FleetScope` |
| Mislabel positive-feedback guard | **Closed** | `AbstainMonitor` + `GoldenSet` |
| **Training pipeline** | **Open — needs a robot** | see below |

---

## Still open, and honestly so

- [ ] **The trainer.** No model, optimiser, or gradient exists here. A challenger
      arrives as an artifact some offline process produced; this crate only decides
      whether it may serve. Building the trainer needs episodes only a robot can
      produce.
- [ ] **Real episodes.** Every number in these tests is synthetic. The gate has
      never seen a real robot's data, and its thresholds (200 episodes/arm, 2× bias,
      1.96σ) are defensible defaults, not tuned values.
- [ ] **A measured judge bias.** `JudgeQuality::bias` must come from a golden set
      scored against human truth. The machinery to score it exists
      (`GoldenSet::score`); the golden set itself does not.
- [ ] **Automatic reset**, which ADR 0005 identifies as the actual ceiling: without
      it, ~3 100 episodes/week at 43% duty cycle instead of ~32 000 at 100%, and
      every attempt costs a human-minute. The gate's 200-episodes-per-arm minimum is
      cheap under automatic reset and expensive without it.
- [ ] `doctor → commission → verify` state machine, ARM64 `.deb`, live executive,
      three-robot Profile-swap validation (unchanged from Step 4).

Findings #1–#5 are now closed **in software**. What remains is not architecture: it
is a robot, a fence, a tape mark on the floor, and the automatic reset that makes
the loop affordable.
