use crate::runtime::{FusedTarget, ModelProposal, RuntimeState, SensorFrame};

pub trait LocalPolicy {
    fn infer(&mut self, frame: &SensorFrame, target: &FusedTarget, state: &RuntimeState)
        -> Result<ModelProposal, String>;
}

pub struct NoModel;

impl LocalPolicy for NoModel {
    fn infer(&mut self, _frame: &SensorFrame, _target: &FusedTarget, _state: &RuntimeState)
        -> Result<ModelProposal, String> {
        Err("no local model backend configured".into())
    }
}
