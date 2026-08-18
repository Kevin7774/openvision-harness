use super::contact::ContactPair;
#[derive(Clone, Debug)]
pub struct GraspCandidate { pub contacts: ContactPair, pub approach: [f64; 3], pub score: f64 }
