use crate::runtime::TargetObservation;

#[derive(Clone, Debug)]
pub struct D405Observation {
    pub target: TargetObservation,
    pub frame_id: String,
}
