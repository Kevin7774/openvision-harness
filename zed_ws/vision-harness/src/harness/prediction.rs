#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct Prediction { pub outcome: String, pub confidence: f64 }
