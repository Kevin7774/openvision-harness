use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Health {
    Healthy,
    Degraded,
    Unavailable,
}

impl Health {
    pub fn is_healthy(self) -> bool {
        self == Self::Healthy
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct D405Status {
    pub present: bool,
    pub serial: Option<String>,
    pub usb_speed_mbps: Option<u32>,
    #[serde(default)]
    pub sustained_stream_verified: bool,
    pub health: Health,
    pub reason: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SerialCandidate {
    pub usb_path: String,
    pub vendor_product: String,
    pub tty: Option<String>,
    pub health: Health,
    pub role: String,
    pub reason: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SensorStatus {
    pub d405: D405Status,
    pub tactile_candidates: Vec<SerialCandidate>,
    pub right_gripper_serial: String,
}

pub fn print_status() -> Result<(), String> {
    let report = inspect()?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|e| e.to_string())?
    );
    Ok(())
}

pub fn inspect() -> Result<SensorStatus, String> {
    let d405_device = find_usb_device("8086", "0b5b")?;
    let enumeration = Command::new("/opt/ros/jazzy/bin/rs-enumerate-devices")
        .arg("-s")
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).into_owned());
    let serial = enumeration.as_deref().and_then(d405_serial);
    let speed = d405_device.as_ref().and_then(|path| read_speed(path).ok());
    // No D405 frame adapter or dated sustained-stream verification is connected
    // to this process yet. Enumeration and link speed alone cannot prove frames.
    let sustained_stream_verified = false;
    let (health, reason) = classify_d405(
        d405_device.is_some() && serial.is_some(),
        speed,
        sustained_stream_verified,
    );
    let tactile_candidates = find_usb_devices("1a86", "7523")?
        .into_iter()
        .map(|path| serial_candidate(&path))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(SensorStatus {
        d405: D405Status {
            present: d405_device.is_some(),
            serial,
            usb_speed_mbps: speed,
            sustained_stream_verified,
            health,
            reason,
        },
        tactile_candidates,
        right_gripper_serial: "/dev/ttyUSB0 (CP2102N, reserved for G2 at 2 Mbaud)".into(),
    })
}

fn classify_d405(
    enumerated: bool,
    speed: Option<u32>,
    sustained_stream_verified: bool,
) -> (Health, String) {
    if !enumerated {
        return (
            Health::Unavailable,
            "USB device or librealsense enumeration missing".into(),
        );
    }
    match speed {
        Some(value) if value >= 5_000 && sustained_stream_verified => (Health::Healthy, format!("librealsense sustained stream verified on {value} Mbit/s USB path")),
        Some(value) if value >= 5_000 => (Health::Degraded, format!("librealsense enumerated on {value} Mbit/s USB path, but sustained streaming is not verified")),
        Some(value) => (Health::Degraded, format!("librealsense enumerated, but {value} Mbit/s USB path is below USB 3.x and has disconnected under streaming load")),
        None => (Health::Degraded, "librealsense enumerated, USB link speed unknown".into()),
    }
}

fn d405_serial(output: &str) -> Option<String> {
    output
        .lines()
        .find(|line| line.contains("Intel RealSense D405"))
        .and_then(|line| line.split_whitespace().rev().nth(1))
        .map(str::to_string)
}

fn serial_candidate(path: &Path) -> Result<SerialCandidate, String> {
    let tty = find_tty(path)?;
    let health = if tty.is_some() {
        Health::Degraded
    } else {
        Health::Unavailable
    };
    let reason = if tty.is_some() {
        "serial node exists, but the 115200 query/response frame and left/right mapping are unverified"
    } else {
        "USB UART is present but this kernel exposes no tty driver node"
    };
    Ok(SerialCandidate {
        usb_path: path
            .file_name()
            .and_then(|v| v.to_str())
            .unwrap_or("unknown")
            .into(),
        vendor_product: "1a86:7523 CH340".into(),
        tty,
        health,
        role: "tactile_candidate_unverified".into(),
        reason: reason.into(),
    })
}

fn find_usb_device(vendor: &str, product: &str) -> Result<Option<PathBuf>, String> {
    Ok(find_usb_devices(vendor, product)?.into_iter().next())
}

fn find_usb_devices(vendor: &str, product: &str) -> Result<Vec<PathBuf>, String> {
    let mut matches = Vec::new();
    for entry in fs::read_dir("/sys/bus/usb/devices").map_err(|e| e.to_string())? {
        let path = entry.map_err(|e| e.to_string())?.path();
        if read_trimmed(path.join("idVendor")).as_deref() == Ok(vendor)
            && read_trimmed(path.join("idProduct")).as_deref() == Ok(product)
        {
            matches.push(path);
        }
    }
    matches.sort();
    Ok(matches)
}

fn read_speed(path: &Path) -> Result<u32, String> {
    let value = read_trimmed(path.join("speed"))?;
    value
        .parse::<f64>()
        .map(|speed| speed.round() as u32)
        .map_err(|e| e.to_string())
}

fn find_tty(path: &Path) -> Result<Option<String>, String> {
    for entry in walk(path, 3)? {
        if let Some(name) = entry.file_name().and_then(|v| v.to_str()) {
            if name.starts_with("ttyUSB") || name.starts_with("ttyACM") {
                return Ok(Some(format!("/dev/{name}")));
            }
        }
    }
    Ok(None)
}

fn walk(root: &Path, depth: usize) -> Result<Vec<PathBuf>, String> {
    if depth == 0 {
        return Ok(Vec::new());
    }
    let mut paths = Vec::new();
    for entry in fs::read_dir(root).map_err(|e| e.to_string())? {
        let path = entry.map_err(|e| e.to_string())?.path();
        paths.push(path.clone());
        if path.is_dir() {
            paths.extend(walk(&path, depth - 1)?);
        }
    }
    Ok(paths)
}

fn read_trimmed(path: impl AsRef<Path>) -> Result<String, String> {
    fs::read_to_string(path)
        .map(|value| value.trim().to_string())
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn usb2_d405_is_degraded_not_absent() {
        assert_eq!(classify_d405(true, Some(480), false).0, Health::Degraded);
    }
    #[test]
    fn usb3_without_sustained_stream_is_still_degraded() {
        assert_eq!(classify_d405(true, Some(5_000), false).0, Health::Degraded);
    }
    #[test]
    fn usb3_with_sustained_stream_is_healthy() {
        assert_eq!(classify_d405(true, Some(5_000), true).0, Health::Healthy);
    }
}
