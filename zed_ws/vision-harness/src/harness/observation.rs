#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct ObservationRecord { pub frame_id: String, pub received_at_ns: u64 }
