use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

const DEFAULT_WORKSPACE_ROOT: &str = "/home/astrabot/workspace";
const DEFAULT_ARM_URDF: &str = "/opt/ros/astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_arm_description.urdf";
const DEFAULT_MOVEIT_VALIDATOR: &str =
    "ros/rtc_teleop/install/xr1_moveit_bridge/lib/xr1_moveit_bridge/xr1_moveit_validator";
const DEFAULT_MOVEIT_SRDF: &str = "ros/rtc_teleop/src/xr1_moveit_bridge/config/xr1_arms.srdf";

#[derive(Clone, Debug)]
pub struct RuntimePaths {
    workspace_root: PathBuf,
    arm_urdf: PathBuf,
    moveit_validator: PathBuf,
    moveit_srdf: PathBuf,
}

impl RuntimePaths {
    pub fn discover() -> Self {
        let workspace_root = std::env::var_os("XR1_WORKSPACE_ROOT")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_WORKSPACE_ROOT));
        let arm_urdf = std::env::var_os("XR1_ARM_URDF")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_ARM_URDF));
        let moveit_validator = std::env::var_os("XR1_MOVEIT_VALIDATOR")
            .map(PathBuf::from)
            .unwrap_or_else(|| workspace_root.join(DEFAULT_MOVEIT_VALIDATOR));
        let moveit_srdf = std::env::var_os("XR1_MOVEIT_SRDF")
            .map(PathBuf::from)
            .unwrap_or_else(|| workspace_root.join(DEFAULT_MOVEIT_SRDF));
        Self {
            workspace_root,
            arm_urdf,
            moveit_validator,
            moveit_srdf,
        }
    }

    pub fn workspace_root(&self) -> &Path {
        &self.workspace_root
    }

    pub fn data_root(&self) -> PathBuf {
        self.workspace_root.join("data")
    }

    pub fn scripts_root(&self) -> PathBuf {
        self.workspace_root.join("py")
    }

    pub fn arm_urdf(&self) -> &Path {
        &self.arm_urdf
    }

    pub fn moveit_validator(&self) -> &Path {
        &self.moveit_validator
    }

    pub fn moveit_srdf(&self) -> &Path {
        &self.moveit_srdf
    }

    pub fn run_python(&self, script: &str, args: &[&str]) -> Result<(), String> {
        let script_path = self.scripts_root().join(script);
        let mut command = format!(
            "source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash && exec python3 {}",
            shell_quote(&script_path.to_string_lossy())
        );
        for arg in args {
            command.push(' ');
            command.push_str(&shell_quote(arg));
        }
        let status = Command::new("/bin/bash")
            .args(["-lc", &command])
            .current_dir(self.scripts_root())
            .env("ROS_DOMAIN_ID", "12")
            .env("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            .stdin(Stdio::null())
            .status()
            .map_err(|error| format!("failed to start {script}: {error}"))?;
        if status.success() {
            Ok(())
        } else {
            Err(format!("{script} exited with {status}"))
        }
    }
}

fn shell_quote(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\\''"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runtime_paths_are_derived_from_one_root() {
        let paths = RuntimePaths {
            workspace_root: PathBuf::from("/tmp/xr1"),
            arm_urdf: PathBuf::from("/tmp/robot.urdf"),
            moveit_validator: PathBuf::from("/tmp/validator"),
            moveit_srdf: PathBuf::from("/tmp/robot.srdf"),
        };
        assert_eq!(paths.data_root(), PathBuf::from("/tmp/xr1/data"));
        assert_eq!(paths.scripts_root(), PathBuf::from("/tmp/xr1/py"));
        assert_eq!(paths.arm_urdf(), Path::new("/tmp/robot.urdf"));
        assert_eq!(paths.moveit_validator(), Path::new("/tmp/validator"));
        assert_eq!(paths.moveit_srdf(), Path::new("/tmp/robot.srdf"));
    }
}
