use super::bbox::BoundingBox3;
#[derive(Clone, Debug)]
pub struct ObjectGeometry { pub bounds: BoundingBox3, pub confidence: f64 }
