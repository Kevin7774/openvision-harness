use serde::{Deserialize, Serialize};

const NS_PER_MS: u64 = 1_000_000;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Pose3 {
    pub position_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TargetObservation {
    pub stamp_ns: u64,
    pub pose: Pose3,
    pub confidence: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TactilePad {
    pub stamp_ns: u64,
    pub contact: bool,
    pub normal_force_n: f64,
    pub shear_force_n: [f64; 2],
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RobotState {
    pub stamp_ns: u64,
    pub joint_limit_margin_rad: f64,
    pub collision_free: bool,
    pub gripper_opening_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SensorFrame {
    pub frame_id: String,
    pub received_at_ns: u64,
    pub stereo_target: Option<TargetObservation>,
    pub d405_target: Option<TargetObservation>,
    pub tactile_left: TactilePad,
    pub tactile_right: TactilePad,
    pub robot: RobotState,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SkillStage {
    Search,
    Align,
    Approach,
    Contact,
    Grasped,
    Transport,
    Place,
    Verify,
    Stopped,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RuntimeState {
    pub stage: SkillStage,
    pub cycle: u64,
}

impl Default for RuntimeState {
    fn default() -> Self {
        Self {
            stage: SkillStage::Search,
            cycle: 0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ModelProposal {
    pub confidence: f64,
    pub end_effector_delta_m: [f64; 3],
    pub yaw_delta_rad: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RuntimeAction {
    Observe,
    Servo {
        end_effector_delta_m: [f64; 3],
        yaw_delta_rad: f64,
    },
    CloseGripper,
    Hold,
    Lift,
    OpenGripper,
    Stop { reason: String },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FusedTarget {
    pub pose: Pose3,
    pub confidence: f64,
    pub source: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RuntimeDecision {
    pub ok: bool,
    pub frame_id: String,
    pub stage_before: SkillStage,
    pub stage_after: SkillStage,
    pub target: Option<FusedTarget>,
    pub action: RuntimeAction,
}

#[derive(Clone, Debug)]
pub struct RuntimeConfig {
    pub max_sensor_age_ms: u64,
    pub max_camera_skew_ms: u64,
    pub min_joint_margin_rad: f64,
    pub min_model_confidence: f64,
    pub max_servo_step_m: f64,
    pub contact_force_n: f64,
    pub hold_force_n: f64,
    pub max_shear_force_n: f64,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            max_sensor_age_ms: 100,
            max_camera_skew_ms: 40,
            min_joint_margin_rad: 0.08,
            min_model_confidence: 0.70,
            max_servo_step_m: 0.015,
            contact_force_n: 0.20,
            hold_force_n: 0.60,
            max_shear_force_n: 0.80,
        }
    }
}

pub struct HarnessRuntime {
    config: RuntimeConfig,
    pub state: RuntimeState,
}

impl HarnessRuntime {
    pub fn new(config: RuntimeConfig, state: RuntimeState) -> Self {
        Self { config, state }
    }

    pub fn step(&mut self, frame: &SensorFrame, proposal: Option<ModelProposal>) -> RuntimeDecision {
        let before = self.state.stage;
        if let Some(reason) = crate::safety::gate::validate(frame, &self.config) {
            self.state.stage = SkillStage::Stopped;
            return self.decision(frame, before, None, RuntimeAction::Stop { reason });
        }

        let target = crate::perception::fusion::fuse_target(frame, &self.config);
        let action = match self.state.stage {
            SkillStage::Search => {
                if target.is_some() {
                    self.state.stage = SkillStage::Align;
                }
                RuntimeAction::Observe
            }
            SkillStage::Align | SkillStage::Approach => {
                let Some(model) = proposal else {
                    return self.decision(frame, before, target, RuntimeAction::Observe);
                };
                if target.is_none() || model.confidence < self.config.min_model_confidence {
                    RuntimeAction::Observe
                } else if !valid_servo_step(&model, self.config.max_servo_step_m) {
                    self.state.stage = SkillStage::Stopped;
                    RuntimeAction::Stop {
                        reason: "local model servo step exceeds safety envelope".into(),
                    }
                } else {
                    self.state.stage = SkillStage::Approach;
                    RuntimeAction::Servo {
                        end_effector_delta_m: model.end_effector_delta_m,
                        yaw_delta_rad: model.yaw_delta_rad,
                    }
                }
            }
            SkillStage::Contact => RuntimeAction::CloseGripper,
            SkillStage::Grasped => RuntimeAction::Lift,
            SkillStage::Transport => RuntimeAction::Hold,
            SkillStage::Place => RuntimeAction::OpenGripper,
            SkillStage::Verify => {
                self.state.stage = SkillStage::Search;
                self.state.cycle += 1;
                RuntimeAction::Observe
            }
            SkillStage::Stopped => RuntimeAction::Stop {
                reason: "runtime is stopped and requires explicit reset".into(),
            },
        };

        crate::perception::tactile::apply_transition(&mut self.state, frame, &self.config);
        self.decision(frame, before, target, action)
    }

    fn decision(
        &self,
        frame: &SensorFrame,
        before: SkillStage,
        target: Option<FusedTarget>,
        action: RuntimeAction,
    ) -> RuntimeDecision {
        RuntimeDecision {
            ok: !matches!(action, RuntimeAction::Stop { .. }),
            frame_id: frame.frame_id.clone(),
            stage_before: before,
            stage_after: self.state.stage,
            target,
            action,
        }
    }

}

fn valid_servo_step(proposal: &ModelProposal, max_step: f64) -> bool {
    proposal.confidence.is_finite()
        && proposal.yaw_delta_rad.is_finite()
        && proposal.end_effector_delta_m.iter().all(|value| value.is_finite())
        && proposal
            .end_effector_delta_m
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt()
            <= max_step
        && proposal.yaw_delta_rad.abs() <= 0.15
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> SensorFrame {
        let stamp = 1_000_000_000;
        let target = TargetObservation {
            stamp_ns: stamp,
            pose: Pose3 {
                position_m: [0.5, -0.1, 0.82],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            confidence: 0.9,
        };
        SensorFrame {
            frame_id: "test".into(),
            received_at_ns: stamp + 10 * NS_PER_MS,
            stereo_target: Some(target.clone()),
            d405_target: Some(target),
            tactile_left: TactilePad {
                stamp_ns: stamp,
                contact: false,
                normal_force_n: 0.0,
                shear_force_n: [0.0, 0.0],
            },
            tactile_right: TactilePad {
                stamp_ns: stamp,
                contact: false,
                normal_force_n: 0.0,
                shear_force_n: [0.0, 0.0],
            },
            robot: RobotState {
                stamp_ns: stamp,
                joint_limit_margin_rad: 0.3,
                collision_free: true,
                gripper_opening_m: 0.046,
            },
        }
    }

    #[test]
    fn fuses_synchronized_cameras() {
        let mut runtime = HarnessRuntime::new(RuntimeConfig::default(), RuntimeState::default());
        let decision = runtime.step(&frame(), None);
        assert_eq!(decision.stage_after, SkillStage::Align);
        assert_eq!(decision.target.unwrap().source, "stereo+d405");
    }

    #[test]
    fn rejects_stale_sensor_data() {
        let mut input = frame();
        input.received_at_ns += 200 * NS_PER_MS;
        let mut runtime = HarnessRuntime::new(RuntimeConfig::default(), RuntimeState::default());
        let decision = runtime.step(&input, None);
        assert!(!decision.ok);
        assert_eq!(decision.stage_after, SkillStage::Stopped);
    }

    #[test]
    fn rejects_oversized_model_step() {
        let mut runtime = HarnessRuntime::new(
            RuntimeConfig::default(),
            RuntimeState { stage: SkillStage::Align, cycle: 0 },
        );
        let decision = runtime.step(
            &frame(),
            Some(ModelProposal {
                confidence: 0.95,
                end_effector_delta_m: [0.03, 0.0, 0.0],
                yaw_delta_rad: 0.0,
            }),
        );
        assert!(!decision.ok);
    }

    #[test]
    fn bilateral_contact_advances_stage() {
        let mut input = frame();
        for pad in [&mut input.tactile_left, &mut input.tactile_right] {
            pad.contact = true;
            pad.normal_force_n = 0.3;
        }
        let mut runtime = HarnessRuntime::new(
            RuntimeConfig::default(),
            RuntimeState { stage: SkillStage::Approach, cycle: 0 },
        );
        let decision = runtime.step(
            &input,
            Some(ModelProposal {
                confidence: 0.9,
                end_effector_delta_m: [0.0, 0.0, -0.002],
                yaw_delta_rad: 0.0,
            }),
        );
        assert_eq!(decision.stage_after, SkillStage::Contact);
    }
}
