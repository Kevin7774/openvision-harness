use std::env;
use std::path::PathBuf;
use std::process::Command;

const PLANNER_INPUTS: &[&str] = &[
    "../../Cargo.lock",
    "../harness-contracts/src/profile.rs",
    "../../task-packs/yellow-block-pick-place/src/detector.rs",
    "../../task-packs/yellow-block-pick-place/src/lib.rs",
    "src/kinematics/grasp.rs",
    "src/kinematics/ik.rs",
    "src/kinematics/mod.rs",
    "src/kinematics/model.rs",
    "src/kinematics/types.rs",
    "src/perception/depth.rs",
    "src/perception/geometry.rs",
    "src/perception/mod.rs",
    "src/planning/mod.rs",
    "src/planning/moveit.rs",
    "src/planning/search.rs",
    "src/planning/types.rs",
    "src/proposal.rs",
    "src/taskpack.rs",
];

fn main() {
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest directory"));
    let mut revision = String::new();
    for relative in PLANNER_INPUTS {
        println!("cargo:rerun-if-changed={relative}");
        let path = root.join(relative);
        let output = Command::new("openssl")
            .args(["dgst", "-sha256", "-r"])
            .arg(&path)
            .output()
            .unwrap_or_else(|error| panic!("failed to hash {}: {error}", path.display()));
        assert!(output.status.success(), "failed to hash {}", path.display());
        revision.push_str(
            std::str::from_utf8(&output.stdout)
                .expect("openssl output")
                .split_whitespace()
                .next()
                .expect("openssl digest"),
        );
    }
    println!("cargo:rustc-env=XR1_PLANNER_REVISION={revision}");
}
