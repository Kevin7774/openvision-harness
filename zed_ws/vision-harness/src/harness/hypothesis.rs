#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct Hypothesis { pub statement: String, pub confidence: f64 }
