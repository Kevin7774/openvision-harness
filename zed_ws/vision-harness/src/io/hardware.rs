use serde::Serialize;
use std::path::Path;
use std::process::Command;

#[derive(Serialize)]
struct HardwareStatus {
    d405_usb: bool,
    d405_ros_driver: bool,
    tactile_serial: Option<&'static str>,
    tactile_baud: u32,
}

pub fn print_status() -> Result<(), String> {
    let d405_usb = Command::new("/opt/ros/jazzy/bin/rs-enumerate-devices")
        .arg("-s").output().map(|out| String::from_utf8_lossy(&out.stdout).contains("D405"))
        .unwrap_or(false);
    let status = HardwareStatus {
        d405_usb,
        d405_ros_driver: Path::new("/opt/ros/jazzy/lib/realsense2_camera/realsense2_camera_node").exists(),
        tactile_serial: Path::new("/dev/ttyUSB0").exists().then_some("/dev/ttyUSB0"),
        tactile_baud: 115_200,
    };
    println!("{}", serde_json::to_string(&status).map_err(|e| e.to_string())?);
    Ok(())
}
