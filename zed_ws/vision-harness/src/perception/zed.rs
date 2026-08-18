use crate::runtime::TargetObservation;

#[derive(Clone, Debug)]
pub struct ZedObservation {
    pub target: TargetObservation,
    pub frame_id: String,
}
