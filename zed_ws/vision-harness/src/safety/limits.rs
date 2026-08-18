#[derive(Clone, Copy, Debug)]
pub struct MotionEnvelope {
    pub max_translation_m: f64,
    pub max_yaw_rad: f64,
}
