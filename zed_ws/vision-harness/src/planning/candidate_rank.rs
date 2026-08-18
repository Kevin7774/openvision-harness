use crate::geometry::grasp::GraspCandidate;
pub fn best(mut values: Vec<GraspCandidate>) -> Option<GraspCandidate> {
    values.retain(|v| v.score.is_finite());
    values.into_iter().max_by(|a, b| a.score.total_cmp(&b.score))
}
