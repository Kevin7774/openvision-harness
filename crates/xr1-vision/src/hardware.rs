use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

const D405_FRAME_EVIDENCE_MAX_AGE_MS: f64 = 3_000.0;
const D405_MIN_DEPTH_VALID_RATIO: f64 = 0.50;
const TACTILE_EVIDENCE_MAX_AGE_MS: f64 = 1_000.0;

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
    let serial = enumeration
        .as_deref()
        .and_then(d405_serial)
        .or_else(|| {
            d405_device
                .as_ref()
                .and_then(|path| read_trimmed(path.join("serial")).ok())
                .filter(|value| !value.is_empty())
        })
        .or_else(python_d405_serial);
    let speed = d405_device.as_ref().and_then(|path| read_speed(path).ok());
    let sustained_stream_verified = serial
        .as_deref()
        .is_some_and(|serial| fresh_d405_frame_verified(serial).unwrap_or(false));
    let (health, reason) = classify_d405(
        d405_device.is_some() && serial.is_some(),
        speed,
        sustained_stream_verified,
    );
    let tactile_stream_verified = fresh_tactile_sample_verified().unwrap_or_default();
    let tactile_candidates = find_usb_devices("1a86", "7523")?
        .into_iter()
        .map(|path| serial_candidate(&path, &tactile_stream_verified))
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
        Some(value) if sustained_stream_verified => (Health::Healthy, format!("fresh bounded librealsense RGB/depth stream verified on {value} Mbit/s USB path")),
        Some(value) if value >= 5_000 => (Health::Degraded, format!("librealsense enumerated on {value} Mbit/s USB path, but sustained streaming is not verified")),
        Some(value) => (Health::Degraded, format!("librealsense enumerated, but {value} Mbit/s USB path is below USB 3.x and has disconnected under streaming load")),
        None => (Health::Degraded, "librealsense enumerated, USB link speed unknown".into()),
    }
}

#[derive(Deserialize)]
struct D405FrameEvidence {
    ok: bool,
    mode: String,
    serial: String,
    received_at_ns: u64,
    depth_valid_ratio: f64,
    sustained_stream_verified: bool,
}

fn fresh_d405_frame_verified(expected_serial: &str) -> Result<bool, String> {
    let root = std::env::var_os("XR1_WORKSPACE_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/home/astrabot/workspace"));
    let path = root.join("data/sensors/d405/latest.json");
    let report: D405FrameEvidence = serde_json::from_slice(
        &fs::read(&path).map_err(|error| format!("{}: {error}", path.display()))?,
    )
    .map_err(|error| format!("invalid D405 evidence {}: {error}", path.display()))?;
    if !report.ok
        || report.mode != "d405_near_field_observation"
        || report.serial != expected_serial
        || !report.sustained_stream_verified
        || !report.depth_valid_ratio.is_finite()
        || report.depth_valid_ratio < D405_MIN_DEPTH_VALID_RATIO
    {
        return Ok(false);
    }
    let now_ns: u64 = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| error.to_string())?
        .as_nanos()
        .try_into()
        .map_err(|_| "system timestamp does not fit u64".to_string())?;
    if now_ns < report.received_at_ns {
        return Ok(false);
    }
    let age_ms = (now_ns - report.received_at_ns) as f64 / 1_000_000.0;
    Ok(age_ms <= D405_FRAME_EVIDENCE_MAX_AGE_MS)
}

fn d405_serial(output: &str) -> Option<String> {
    let trimmed = output.trim();
    if !trimmed.is_empty() && trimmed.bytes().all(|byte| byte.is_ascii_digit()) {
        return Some(trimmed.to_string());
    }
    output
        .lines()
        .find(|line| line.contains("Intel RealSense D405"))
        .and_then(|line| line.split_whitespace().rev().nth(1))
        .map(str::to_string)
}

fn python_d405_serial() -> Option<String> {
    let output = Command::new("/usr/bin/python3")
        .args([
            "-c",
            "import pyrealsense2 as r; d=[x for x in r.context().query_devices() if 'D405' in x.get_info(r.camera_info.name)]; print(d[0].get_info(r.camera_info.serial_number) if len(d)==1 else '')",
        ])
        .output()
        .ok()
        .filter(|output| output.status.success())?;
    d405_serial(&String::from_utf8_lossy(&output.stdout))
}

fn serial_candidate(
    path: &Path,
    verified_endpoints: &BTreeSet<String>,
) -> Result<SerialCandidate, String> {
    let tty = find_tty(path)?;
    let usb_path = path
        .file_name()
        .and_then(|value| value.to_str())
        .unwrap_or("unknown")
        .to_string();
    let tactile_stream_verified = verified_endpoints.contains(&usb_path)
        || tty
            .as_ref()
            .is_some_and(|device| verified_endpoints.contains(device));
    let health = if tactile_stream_verified {
        Health::Healthy
    } else if tty.is_some() {
        Health::Degraded
    } else {
        Health::Unavailable
    };
    let reason = if tactile_stream_verified {
        "fresh two-pad pressure samples are available through the configured hardware boundary"
    } else if tty.is_some() {
        "serial node exists, but the 115200 query/response frame and left/right mapping are unverified"
    } else {
        "USB UART is present but this kernel exposes no tty driver node"
    };
    Ok(SerialCandidate {
        usb_path,
        vendor_product: "1a86:7523 CH340".into(),
        tty,
        health,
        role: if tactile_stream_verified {
            "tactile_pressure_pad".into()
        } else {
            "tactile_pressure_pad_unverified".into()
        },
        reason: reason.into(),
    })
}

#[derive(Deserialize)]
struct TactileFrameEvidence {
    ok: bool,
    mode: String,
    sensor_stamp_ns: u64,
    received_at_ns: u64,
    sources: Vec<TactileSourceEvidence>,
    pads: Vec<TactilePadEvidence>,
}

#[derive(Deserialize)]
struct TactileSourceEvidence {
    endpoint: String,
}

#[derive(Deserialize)]
struct TactilePadEvidence {
    id: String,
    raw: f64,
    median_abs_deviation: f64,
    sample_count: usize,
}

fn fresh_tactile_sample_verified() -> Result<BTreeSet<String>, String> {
    let root = std::env::var_os("XR1_WORKSPACE_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/home/astrabot/workspace"));
    let path = root.join("data/sensors/tactile/latest.json");
    let report: TactileFrameEvidence = serde_json::from_slice(
        &fs::read(&path).map_err(|error| format!("{}: {error}", path.display()))?,
    )
    .map_err(|error| format!("invalid tactile evidence {}: {error}", path.display()))?;
    let mut ids = report
        .pads
        .iter()
        .map(|pad| pad.id.as_str())
        .collect::<Vec<_>>();
    ids.sort_unstable();
    ids.dedup();
    if !report.ok
        || report.mode != "tactile_observation"
        || report.pads.len() != 2
        || ids.len() != 2
        || report.pads.iter().any(|pad| {
            !pad.raw.is_finite()
                || !pad.median_abs_deviation.is_finite()
                || pad.sample_count < crate::grasp_feedback::MIN_PAD_SAMPLES
        })
    {
        return Ok(BTreeSet::new());
    }
    let now_ns: u64 = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| error.to_string())?
        .as_nanos()
        .try_into()
        .map_err(|_| "system timestamp does not fit u64".to_string())?;
    if now_ns < report.received_at_ns
        || now_ns < report.sensor_stamp_ns
        || report.sensor_stamp_ns == 0
        || report.sensor_stamp_ns > report.received_at_ns
    {
        return Ok(BTreeSet::new());
    }
    let age_ms = ((now_ns - report.received_at_ns).max(now_ns - report.sensor_stamp_ns)) as f64
        / 1_000_000.0;
    if age_ms > TACTILE_EVIDENCE_MAX_AGE_MS {
        return Ok(BTreeSet::new());
    }
    Ok(report
        .sources
        .into_iter()
        .map(|source| source.endpoint)
        .collect())
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
    fn plain_python_d405_serial_is_accepted() {
        assert_eq!(
            d405_serial("262422270599\n").as_deref(),
            Some("262422270599")
        );
    }

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

    #[test]
    fn fresh_bounded_stream_can_gate_one_step_even_on_usb2() {
        assert_eq!(classify_d405(true, Some(480), true).0, Health::Healthy);
    }
}
