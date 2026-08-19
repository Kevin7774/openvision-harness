# Building and checking

## Toolchain

There is **no rustup** on this machine. Rust is the Ubuntu 24.04 distro
toolchain, and the lint tools must match its version exactly or they refuse to
run:

```
rustc 1.75.0    /usr/bin/rustc      (apt: rustc)
cargo 1.75.0    /usr/bin/cargo      (apt: cargo)
rustfmt 1.7.0   /usr/bin/rustfmt    (apt: rustfmt)
clippy 0.1.75   /usr/bin/cargo-clippy (apt: rust-clippy)
```

`edition = "2021"` and `rust-version = "1.75"` in `[workspace.package]` are the
real ceiling, not a preference. Anything needing a newer compiler needs a
toolchain decision first, and an ADR.

Offline: `~/.cargo/registry` already holds these four and their transitive tail
(`image`, `nalgebra`, `roxmltree`, `serde`/`serde_json`). Adding a fifth needs
network, which this machine may not have.

## The four gates

All four must pass. In order of how fast they fail:

```bash
cargo fmt --all -- --check
cargo check --workspace
cargo clippy --workspace --all-targets --all-features
cargo test --workspace
```

Clippy is deny-level in practice: treat every warning as a failure. Two useful
ones it already caught here --- `approx_constant` (a hand-written `1.5708` that
should have been `FRAC_PI_2`, in a list that turned out to be better generated
than written) and `items_after_test_module`.

```bash
cargo build --release       # -> target/release/xr1-vision, 0 warnings
```

The release profile is `strip`, `lto`, `codegen-units = 1`. Debug builds are
fine for tests; use release on the robot, since IK runs a multi-seed
damped-least-squares solve and the debug build is minutes slower per call.

## Tests

```bash
cargo test --workspace
python3 py/test_motion_adapter.py
python3 py/test_servo_adapter.py
```

The suite contains measured perception regressions plus pure contract tests for
proposal validation, URDF parsing, IK branch deduplication, roll refinement,
visual-servo algebra, safety reports and execution-plan selection. None of them
pretends to unit-test hardware.

| Test | What breaks it |
|---|---|
| `mask_keeps_the_yellow_block_and_drops_the_green_cube` | widening the R/G window until the green cube is a candidate |
| `orange_pads_are_not_yellow_but_the_block_still_is` | the gripper's own pads becoming grasp targets (this happened) |
| `footprint_axes_stay_horizontal_for_a_block_standing_on_end` | letting the oriented box tilt, which makes the closing-axis solve meaningless |
| `named_frame_extracts_both_pads_and_rejects_the_fruit` | duplicating or weakening the measured pad detector until the orange fruit is selected |
| `central_difference_recovers_the_measured_jacobian` | permuting signal rows/joint columns in the +/- fit |
| `three_low_improvement_steps_stop_the_loop` | allowing a stalled servo to continue moving indefinitely |

The final system check remains `xr1-vision plan` against a named saved frame. It
exercises the installed URDF and full candidate pipeline without moving hardware;
record its wall time and candidate counts when changing the search.

## Dependency audit

`bin/audit-deps` checks all 41 locked crates against the RustSec advisory
database in `~/.cargo/advisory-db`, and `bin/audit-deps --check` self-tests its
version comparison without needing the database at all. Today's result:

```
ok         image 0.24.9     RUSTSEC-2019-0014  patched >= 0.21.3
ok         image 0.24.9     RUSTSEC-2020-0073  patched >= 0.23.12  [unsound]
ok         nalgebra 0.32.6  RUSTSEC-2021-0070  patched >= 0.27.1
unpatched  paste 1.0.15     RUSTSEC-2024-0436  [unmaintained]
41 locked crates checked, 0 vulnerable
```

`paste` is a transitive dependency of `nalgebra`'s macros; the advisory is
informational (the crate is unmaintained, not vulnerable) and has no patched
version to move to.

This exists because `cargo audit` cannot run here, and the failure is worth
recording so nobody spends the afternoon again:

| Version | What happens |
|---|---|
| `cargo-audit@0.21.2` | won't build: "requires rustc 1.81.0 or newer" |
| `cargo-audit@0.21.1` | installs, then dies loading the DB: "unsupported CVSS version: 4.0" |
| `cargo-audit@0.18.3` | same CVSS 4.0 parse failure |
| `cargo-machete` | won't build: needs the `edition2024` Cargo feature |

0.21.1 is what is installed in `~/.cargo/bin` — it is not usable, because
advisories added in 2026 (RUSTSEC-2026-0245 among them) carry `CVSS:4.0`
vectors and its parser only knows 3.x. Nothing older parses them either.
Re-try `cargo audit` when the toolchain moves past 1.75; until then
`bin/audit-deps` is the standing substitute.

`cargo-deny` and `cargo-nextest` are still not installed: same toolchain
ceiling, and with one crate and four direct dependencies neither would tell us
something `cargo tree`, the gate above, and this audit do not.
