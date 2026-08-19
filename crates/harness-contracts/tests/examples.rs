//! Proves the shipped example files load through the real loaders and behave as
//! the acceptance criteria require. These are the files an operator would copy
//! and edit per robot, so they must stay honest.

use harness_contracts::{CalibrationManifest, RobotIdentity, RobotProfile};
use std::path::PathBuf;

fn examples_dir() -> PathBuf {
    // crates/harness-contracts -> repo root -> profiles/examples
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../profiles/examples")
}

#[test]
fn example_profile_loads_and_matches_the_reference() {
    let profile = RobotProfile::read(&examples_dir().join("xr1-thor.profile.json"))
        .expect("example profile should load");
    // Swapping in the shipped profile reproduces the compiled defaults exactly —
    // the "change only the profile, not the source" path is real.
    assert_eq!(profile, RobotProfile::xr1_thor_reference());
}

#[test]
fn example_calibration_is_a_placeholder_that_refuses_to_gate_motion() {
    let manifest = CalibrationManifest::read(&examples_dir().join("xr1-thor.calibration.json"))
        .expect("example calibration should load");
    let identity = RobotIdentity {
        robot_id: "xr1-thor-reference".into(),
        station_id: "astrabot-bench".into(),
        urdf_hash: "REPLACE_WITH_sha256_of_astrabot_xr1_evt2_arm_description.urdf".into(),
        serials: vec![
            ("d405".into(), "262422270599".into()),
            ("tool".into(), "d60f7389ced5ef118820724b49d2c684".into()),
        ],
        now_ns: 1_000_000_000_000_000,
    };
    // Even with a perfectly matching identity, the placeholder (sample_count=0)
    // must NOT gate motion. An example that could move a robot would be a trap.
    assert!(manifest.gate_motion(&identity).is_err());
}
