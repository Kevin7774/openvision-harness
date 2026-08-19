# Vision Harness — Step 2 Foundation

**English** | [中文](harness-step-2-foundation.zh.md)

Step 1 ([verdict](harness-step-1-verdict.md)) fixed the honest definition and the
rebuild order. Step 2 executes the **software-provable** part of that order —
findings #1 and #2, and the acceptance criteria that need no hardware to prove.
It is additive, reversible, and breaks no existing behaviour.

> [!NOTE]
> **What Step 2 is not.** It does not touch hardware, ROS, or motion. The steps
> that require a robot, a fleet, or a live Jacobian (`.deb` packaging, live task
> executive, three-robot validation) remain open and are listed under
> [Still hardware-gated](#still-hardware-gated). Nothing below is claimed as
> "works on the robot"; it is claimed as "compiles, is tested, and is correct as
> software."

---

## What was built

A new lower-layer crate, `harness-contracts`, depending on nothing but `serde` —
no hardware, no ROS, no perception maths. `xr1-vision` now depends on it.

```
crates/
├── harness-contracts/         # NEW — the boundaries + per-robot facts
│   ├── src/ports.rs           #   the five (and only five) port traits
│   ├── src/profile.rs         #   RobotProfile: geometry + planning limits
│   ├── src/calibration.rs     #   CalibrationManifest + staleness binding
│   └── tests/examples.rs      #   shipped example files load & behave
└── xr1-vision/                # unchanged behaviour; now depends on contracts
profiles/
└── examples/
    ├── xr1-thor.profile.json      # reproduces the compiled constants exactly
    └── xr1-thor.calibration.json  # placeholder that refuses to gate motion
```

### The five ports (finding #1)

`harness-contracts/src/ports.rs` defines exactly the boundaries Step 1 said were
missing — expressed as traits, not as Python filenames and JSON pipes:

| Trait | Boundary it makes replaceable |
|---|---|
| `ObservationSource` | where a frame comes from (ROS / RealSense / replay / sim) |
| `MotionExecutor` | who moves the arm, under a declared bounded envelope |
| `KinematicsValidator` | is this pose reachable + safe for *this* URDF and table |
| `OutcomeJudge` | did the physical result match — with a mandatory `Abstain` |
| `TaskSkill` | one pluggable capability, dispatched by id |

`Judgement::Abstain` is deliberate: Step 1 (and ADR 0005) require the judge to be
allowed to say *"I don't know"* rather than emit a confident wrong label that
would form a positive-feedback loop.

### RobotProfile (finding #2)

`kinematics/types.rs` still compiles the same constants — but they now have a home
outside the core, and a tripwire keeps the two honest:

```rust
// crates/xr1-vision/src/kinematics/types.rs  (test)
#[test]
fn profile_equivalence() {
    let profile = RobotProfile::xr1_thor_reference();
    assert_eq!(profile.planning.min_tip_z_m, PLANNING_MIN_TIP_Z_M); // 0.785, the table
    // ... every tool + planning value asserted equal ...
}
```

Edit the constant without the profile (or vice-versa) and this test fails. The
"table height in the core" is now a *value in a profile*, and a different
workstation is expressible by editing JSON, not source
(`a_table_height_change_is_expressible_without_touching_source`).

### CalibrationManifest (finding #2, made mobile)

The dangerous case — a calibration copied to another robot — is now detectable.
`CalibrationBinding::check` returns a typed decision, and only `Valid` may gate
motion:

```rust
pub enum CalibrationStatus {
    Valid,
    Mismatch    { reason },            // different robot / station / URDF / serial
    Stale       { age_ns, valid_for_ns },
    Insufficient{ reason },            // too few samples, bad error metric
}
```

Ordering is a guarantee, not an accident: identity is checked **before** age, so a
foreign-but-recent calibration reads as `Mismatch`, never as merely `Stale`
(`foreign_recent_calibration_reads_as_mismatch_not_stale`).

---

## Which Step 1 items this closes

| Step 1 item | Status after Step 2 | Evidence |
|---|---|---|
| #1 "generic interface is naming, not a boundary" | **Closed (software)** | `ports.rs`: five traits, hardware-free crate |
| #2 single-machine constants in the core | **Closed (software)** | `RobotProfile` + `profile_equivalence` tripwire |
| #2 stale/foreign calibration undetectable | **Closed (software)** | `CalibrationStatus` + 9 binding tests |
| Rebuild step 2 "introduce RobotProfile + CalibrationManifest" | **Done** | this crate |
| Rebuild step 3 "add ~five traits at real boundaries" | **Done** | `ports.rs` |

Acceptance criteria from Step 1 now provable **as software** (the rest remain
hardware-gated):

- [x] "the five replaceable boundaries exist as contracts"
- [x] "same source, swap only the Profile" — `example_profile_loads_and_matches_the_reference`
- [x] "a calibration copied from another robot is detected as invalid and blocks motion" — `a_calibration_copied_from_another_robot_is_a_mismatch_and_blocks`
- [x] "an example must never be able to move a robot" — `example_calibration_is_a_placeholder_that_refuses_to_gate_motion`

---

## Proof

```
cargo test --workspace
  harness-contracts (unit):        14 passed
  harness-contracts (examples):     2 passed
  xr1-vision:                       66 passed   (was 65; +1 profile_equivalence)
  ────────────────────────────────────────────
  total:                            82 passed, 0 failed
cargo clippy --workspace:           clean
```

The prior 65 xr1-vision tests are **untouched and still green** — Step 2 added a
layer beneath them without changing what they guard.

---

## Still hardware-gated

Step 2 stops exactly where software honesty stops. These need a robot and are not
claimed as done:

- [ ] `harness-core` / `harness-executive` split (mechanical, but deferred until the ports have real implementors so the seams are drawn from use, not guessed)
- [ ] Move the yellow detector into a `task-packs/` pack (finding #3) — a code move with no hardware need, but it belongs with the executive wiring, next
- [ ] Split the four large orchestration files (finding #4)
- [ ] `doctor → commission → verify` state machine (needs live discovery)
- [ ] ARM64 `.deb` packaging + root-level CI (needs the target)
- [ ] Live task executive instead of replay
- [ ] Episode schema, judge registry, shadow / canary / rollback (finding #5)
- [ ] Three-identical-robot "swap only the Profile" validation

---

## Why this slice, and why stop here

Step 1's own rules #1–#3 say: scope v1 to one platform, migrate constants out one
at a time, add ~five traits only at real boundaries — **do not shotgun.** A full
five-crate teardown today would risk the 82 green tests for zero *verifiable*
gain, because the parts that would prove portability (a second robot) are absent.

So Step 2 completes the largest slice that is (a) a genuine root-cause fix for
findings #1 and #2, (b) fully finishable without hardware, and (c) reversible —
delete one crate and one dependency line to revert. The hardware-gated remainder
is left visibly open rather than faked, which is the same standard the Step 1
verdict was written to.
