#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize)]
pub struct EpisodicMemory { pub successful_grasps: u64, pub failed_grasps: u64 }
