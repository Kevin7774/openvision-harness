use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

const DEFAULT_WORKSPACE_ROOT: &str = "/home/astrabot/workspace";
const DEFAULT_ARM_URDF: &str = "/opt/ros/astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_arm_description.urdf";

#[derive(Clone, Debug)]
pub struct RuntimePaths {
    workspace_root: PathBuf,
    arm_urdf: PathBuf,
}

impl RuntimePaths {
    pub fn discover() -> Self {
        let workspace_root = std::env::var_os("XR1_WORKSPACE_ROOT")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_WORKSPACE_ROOT));
        let arm_urdf = std::env::var_os("XR1_ARM_URDF")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_ARM_URDF));
        Self {
            workspace_root,
            arm_urdf,
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
        };
        assert_eq!(paths.data_root(), PathBuf::from("/tmp/xr1/data"));
        assert_eq!(paths.scripts_root(), PathBuf::from("/tmp/xr1/py"));
        assert_eq!(paths.arm_urdf(), Path::new("/tmp/robot.urdf"));
    }
}
