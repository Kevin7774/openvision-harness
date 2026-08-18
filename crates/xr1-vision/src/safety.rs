use serde::Serialize;
use std::collections::BTreeMap;

use crate::hardware::SensorStatus;
use crate::kinematics::{Chain, PLANNING_MIN_LIMIT_MARGIN_RAD, PLANNING_MIN_TIP_Z_M};
use crate::observation::ObservationState;
use crate::visual_servo::{SensorRequirements, ServoProposal};

pub const MAX_SERVO_STEP_RAD: f64 = 0.05;
pub const MAX_PROPOSAL_OBSERVATION_AGE_MS: f64 = 3_000.0;
pub const MAX_PROPOSAL_JOINT_STATE_AGE_MS: f64 = 3_000.0;
const PATH_SAMPLES: usize = 20;

#[derive(Debug, Serialize)]
pub struct SafetyCheck {
    pub name: String,
    pub passed: bool,
    pub detail: String,
}

#[derive(Debug, Serialize)]
pub struct ServoSafetyReport {
    pub approved: bool,
    pub checks: Vec<SafetyCheck>,
    pub current_joints_rad: BTreeMap<String, f64>,
    pub target_joints_rad: BTreeMap<String, f64>,
    pub observation_age_ms: f64,
    pub joint_state_age_ms: f64,
    pub max_joint_delta_rad: f64,
    pub min_joint_limit_margin_rad: f64,
    pub min_tip_z_m: f64,
    pub limitations: Vec<String>,
}

pub fn evaluate_servo(
    chain: &Chain,
    state: &ObservationState,
    expected_frame_id: &str,
    proposal: &ServoProposal,
    requirements: &SensorRequirements,
    sensors: &SensorStatus,
    now_ns: u64,
) -> Result<ServoSafetyReport, String> {
    let names = chain.names();
    let mut current_joints_rad = BTreeMap::new();
    let mut current = Vec::with_capacity(names.len());
    for name in &names {
        let value = state
            .joint_state
            .positions_rad
            .get(name)
            .and_then(|value| *value)
            .ok_or_else(|| format!("missing live joint {name}"))?;
        if !value.is_finite() {
            return Err(format!("live joint {name} is non-finite"));
        }
        current_joints_rad.insert(name.clone(), value);
        current.push(value);
    }

    if proposal.joint_delta_rad.len() != proposal.controlled_joints.len() {
        return Err("visual-servo proposal has inconsistent named deltas".into());
    }
    let mut target = current.clone();
    for name in &proposal.controlled_joints {
        let Some(index) = names.iter().position(|candidate| candidate == name) else {
            return Err(format!(
                "controlled joint {name} does not belong to this arm"
            ));
        };
        let delta = proposal
            .joint_delta_rad
            .get(name)
            .copied()
            .ok_or_else(|| format!("proposal is missing delta for {name}"))?;
        if !delta.is_finite() {
            return Err(format!("proposal delta for {name} is non-finite"));
        }
        target[index] += delta;
    }
    let target_joints_rad = names
        .iter()
        .cloned()
        .zip(target.iter().copied())
        .collect::<BTreeMap<_, _>>();
    let envelope = chain.motion_envelope(&current, &target, PATH_SAMPLES)?;
    let observation_age_ms = age_ms(now_ns, state.received_at_ns);
    let joint_state_age_ms = age_ms(now_ns, state.joint_state.received_at_ns);
    let tactile_healthy = sensors
        .tactile_candidates
        .iter()
        .any(|candidate| candidate.health.is_healthy());

    let mut checks = Vec::new();
    push_check(
        &mut checks,
        "observation_binding",
        state.frame_id == expected_frame_id,
        format!(
            "request={} observation={}",
            expected_frame_id, state.frame_id
        ),
    );
    push_check(
        &mut checks,
        "observation_freshness",
        observation_age_ms <= MAX_PROPOSAL_OBSERVATION_AGE_MS,
        format!("age_ms={observation_age_ms:.3} ceiling_ms={MAX_PROPOSAL_OBSERVATION_AGE_MS:.0}"),
    );
    push_check(
        &mut checks,
        "joint_state_freshness",
        joint_state_age_ms <= MAX_PROPOSAL_JOINT_STATE_AGE_MS,
        format!("age_ms={joint_state_age_ms:.3} ceiling_ms={MAX_PROPOSAL_JOINT_STATE_AGE_MS:.0}"),
    );
    push_check(
        &mut checks,
        "joint_step_bound",
        envelope.max_joint_delta_rad <= MAX_SERVO_STEP_RAD + 1e-12,
        format!(
            "max_delta_rad={:.6} ceiling_rad={MAX_SERVO_STEP_RAD:.6}",
            envelope.max_joint_delta_rad
        ),
    );
    push_check(
        &mut checks,
        "urdf_joint_limits",
        envelope.joint_limits_ok,
        format!(
            "target_min_margin_rad={:.6}",
            envelope.min_joint_limit_margin_rad
        ),
    );
    push_check(
        &mut checks,
        "joint_limit_margin",
        envelope.min_joint_limit_margin_rad >= PLANNING_MIN_LIMIT_MARGIN_RAD,
        format!(
            "margin_rad={:.6} floor_rad={PLANNING_MIN_LIMIT_MARGIN_RAD:.6}",
            envelope.min_joint_limit_margin_rad
        ),
    );
    push_check(
        &mut checks,
        "fingertip_floor_path",
        envelope.min_tip_z_m > PLANNING_MIN_TIP_Z_M,
        format!(
            "min_tip_z_m={:.6} floor_m={PLANNING_MIN_TIP_Z_M:.6}",
            envelope.min_tip_z_m
        ),
    );
    push_check(
        &mut checks,
        "required_d405_capability",
        !requirements.d405 || sensors.d405.health.is_healthy(),
        format!(
            "required={} health={:?}",
            requirements.d405, sensors.d405.health
        ),
    );
    push_check(
        &mut checks,
        "required_tactile_capability",
        !requirements.tactile || tactile_healthy,
        format!(
            "required={} healthy_candidate={tactile_healthy}",
            requirements.tactile
        ),
    );

    let approved = checks.iter().all(|check| check.passed);
    Ok(ServoSafetyReport {
        approved,
        checks,
        current_joints_rad,
        target_joints_rad,
        observation_age_ms,
        joint_state_age_ms,
        max_joint_delta_rad: envelope.max_joint_delta_rad,
        min_joint_limit_margin_rad: envelope.min_joint_limit_margin_rad,
        min_tip_z_m: envelope.min_tip_z_m,
        limitations: vec![
            "self-collision and gripper-body collision are not modelled by this Rust gate".into(),
            "the hardware adapter must re-check live joint freshness and command-channel idleness"
                .into(),
        ],
    })
}

fn age_ms(now_ns: u64, received_at_ns: u64) -> f64 {
    if now_ns < received_at_ns {
        f64::MAX
    } else {
        (now_ns - received_at_ns) as f64 / 1_000_000.0
    }
}

fn push_check(checks: &mut Vec<SafetyCheck>, name: &str, passed: bool, detail: String) {
    checks.push(SafetyCheck {
        name: name.into(),
        passed,
        detail,
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hardware::{D405Status, Health, SerialCandidate};
    use crate::observation::{JointState, Transform};
    use std::collections::{BTreeMap, HashMap};

    const THREE_JOINT_URDF: &str = r#"
        <robot name="test">
          <link name="base_link"/><link name="l1"/><link name="l2"/><link name="right_tcp_link"/>
          <joint name="right_arm_2_joint" type="revolute">
            <parent link="base_link"/><child link="l1"/><origin xyz="0 0 1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/><limit lower="-1" upper="1"/>
          </joint>
          <joint name="right_arm_4_joint" type="revolute">
            <parent link="l1"/><child link="l2"/><origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/><limit lower="-1" upper="1"/>
          </joint>
          <joint name="right_arm_6_joint" type="revolute">
            <parent link="l2"/><child link="right_tcp_link"/><origin xyz="0 0 0" rpy="0 0 0"/>
            <axis xyz="1 0 0"/><limit lower="-1" upper="1"/>
          </joint>
        </robot>
    "#;

    fn state() -> ObservationState {
        let positions_rad = [
            ("right_arm_2_joint".into(), Some(0.0)),
            ("right_arm_4_joint".into(), Some(0.0)),
            ("right_arm_6_joint".into(), Some(0.0)),
        ]
        .into_iter()
        .collect::<HashMap<_, _>>();
        ObservationState {
            frame_id: "frame-1".into(),
            sensor_stamp_ns: 900_000_000,
            received_at_ns: 1_000_000_000,
            tf: Transform {
                target_frame: "base_link".into(),
                source_frame: "camera".into(),
                translation_m: vec![0.0; 3],
                rotation_xyzw: vec![0.0, 0.0, 0.0, 1.0],
            },
            joint_state: JointState {
                received_at_ns: 1_000_000_000,
                positions_rad,
            },
        }
    }

    fn proposal() -> ServoProposal {
        ServoProposal {
            controlled_joints: vec![
                "right_arm_2_joint".into(),
                "right_arm_4_joint".into(),
                "right_arm_6_joint".into(),
            ],
            joint_delta_rad: [
                ("right_arm_2_joint".into(), 0.01),
                ("right_arm_4_joint".into(), 0.01),
                ("right_arm_6_joint".into(), 0.01),
            ]
            .into_iter()
            .collect::<BTreeMap<_, _>>(),
            predicted_signal_delta: [0.0; 3],
            predicted_remaining_error: [0.0; 3],
            condition_number: 1.0,
            applied_scale: 1.0,
            max_joint_step_rad: MAX_SERVO_STEP_RAD,
        }
    }

    fn sensors(d405: Health) -> SensorStatus {
        SensorStatus {
            d405: D405Status {
                present: true,
                serial: Some("test".into()),
                usb_speed_mbps: Some(480),
                health: d405,
                reason: "test".into(),
            },
            tactile_candidates: vec![SerialCandidate {
                usb_path: "1-1".into(),
                vendor_product: "test".into(),
                tty: None,
                health: Health::Unavailable,
                role: "test".into(),
                reason: "test".into(),
            }],
            right_gripper_serial: "test".into(),
        }
    }

    #[test]
    fn healthy_fresh_bounded_proposal_passes_existing_envelope() {
        let chain = Chain::from_urdf_xml(THREE_JOINT_URDF, "right_tcp_link");
        assert!(chain.is_ok());
        let Some(chain) = chain.ok() else { return };
        let report = evaluate_servo(
            &chain,
            &state(),
            "frame-1",
            &proposal(),
            &SensorRequirements::default(),
            &sensors(Health::Degraded),
            1_100_000_000,
        );
        assert!(report.is_ok());
        assert!(report.ok().map(|value| value.approved).unwrap_or(false));
    }

    #[test]
    fn unavailable_required_sensor_fails_closed() {
        let chain = Chain::from_urdf_xml(THREE_JOINT_URDF, "right_tcp_link");
        assert!(chain.is_ok());
        let Some(chain) = chain.ok() else { return };
        let report = evaluate_servo(
            &chain,
            &state(),
            "frame-1",
            &proposal(),
            &SensorRequirements {
                d405: true,
                tactile: false,
            },
            &sensors(Health::Degraded),
            1_100_000_000,
        );
        assert!(report.is_ok());
        assert!(!report.ok().map(|value| value.approved).unwrap_or(true));
    }

    #[test]
    fn future_or_stale_timestamp_fails_closed() {
        assert_eq!(age_ms(1, 2), f64::MAX);
        assert!(age_ms(4_000_000_001, 1) > MAX_PROPOSAL_OBSERVATION_AGE_MS);
    }
}
