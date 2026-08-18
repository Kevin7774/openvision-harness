use super::model::Chain;
use super::types::{Solution, PLANNING_MIN_LIMIT_MARGIN_RAD, PLANNING_MIN_TIP_Z_M};
use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};

impl Chain {
    /// Return every distinct safe IK branch found by the multi-seed search,
    /// ordered by score.
    pub fn solve_position_candidates_with_reference(
        &self,
        target: [f64; 3],
        seed: &[f64],
        reference: &[f64],
        orientation_offset: [f64; 3],
    ) -> Vec<Solution> {
        let target_rotation = self.fk(reference).rotation
            * UnitQuaternion::from_euler_angles(
                orientation_offset[0],
                orientation_offset[1],
                orientation_offset[2],
            );
        self.solve_pose_candidates(
            Vector3::new(target[0], target[1], target[2]),
            target_rotation,
            seed,
            orientation_offset,
        )
    }

    fn solve_pose_candidates(
        &self,
        target: Vector3<f64>,
        target_rotation: UnitQuaternion<f64>,
        current: &[f64],
        orientation_offset: [f64; 3],
    ) -> Vec<Solution> {
        let midpoint = self
            .active
            .iter()
            .map(|&index| {
                let joint = &self.joints[index];
                (joint.lower + joint.upper) * 0.5
            })
            .collect::<Vec<_>>();
        let blended = current
            .iter()
            .zip(&midpoint)
            .map(|(current, middle)| 0.7 * current + 0.3 * middle)
            .collect::<Vec<_>>();
        let mut seeds = vec![current.to_vec(), midpoint, blended.clone()];
        for index in 0..current.len() {
            for delta in [-0.45, 0.45] {
                let mut seed = blended.clone();
                seed[index] += delta;
                self.clamp(&mut seed);
                seeds.push(seed);
            }
        }

        let mut accepted = Vec::new();
        let mut best_rejected = None;
        for mut joints in seeds {
            self.clamp(&mut joints);
            for _ in 0..160 {
                let tcp_pose = self.fk(&joints);
                let contact_pose = self.contact_pose(&joints);
                let position_error = target - contact_pose.translation.vector;
                let orientation_error =
                    (target_rotation * tcp_pose.rotation.inverse()).scaled_axis();
                if position_error.norm() < 0.004 && orientation_error.norm() < 0.04 {
                    break;
                }
                let mut jacobian = DMatrix::zeros(6, joints.len());
                let epsilon = 1e-4;
                for column in 0..joints.len() {
                    let mut shifted = joints.clone();
                    shifted[column] += epsilon;
                    let shifted_tcp = self.fk(&shifted);
                    let shifted_contact = self.contact_pose(&shifted);
                    let position_delta = (shifted_contact.translation.vector
                        - contact_pose.translation.vector)
                        / epsilon;
                    let rotation_delta = (shifted_tcp.rotation * tcp_pose.rotation.inverse())
                        .scaled_axis()
                        / epsilon;
                    for row in 0..3 {
                        jacobian[(row, column)] = position_delta[row];
                        jacobian[(row + 3, column)] = rotation_delta[row];
                    }
                }
                let error = DVector::from_column_slice(&[
                    position_error[0],
                    position_error[1],
                    position_error[2],
                    orientation_error[0],
                    orientation_error[1],
                    orientation_error[2],
                ]);
                let lhs = &jacobian * jacobian.transpose() + DMatrix::identity(6, 6) * 1e-4;
                let Some(step6) = lhs.lu().solve(&error) else {
                    break;
                };
                let step = jacobian.transpose() * step6;
                for (value, delta) in joints.iter_mut().zip(step.iter()) {
                    *value += delta.clamp(-0.08, 0.08);
                }
                self.clamp(&mut joints);
            }

            let final_tcp = self.fk(&joints);
            let final_contact = self.contact_pose(&joints);
            let residual = (target - final_contact.translation.vector).norm();
            let orientation_residual = (target_rotation * final_tcp.rotation.inverse())
                .scaled_axis()
                .norm();
            let Ok(envelope) = self.motion_envelope(current, &joints, 20) else {
                continue;
            };
            let floor_clear = envelope.min_tip_z_m > PLANNING_MIN_TIP_Z_M;
            let limit_penalty = if envelope.min_joint_limit_margin_rad < 0.12 {
                (0.12 - envelope.min_joint_limit_margin_rad) * 50.0
            } else {
                0.0
            };
            let score = residual * 1000.0
                + orientation_residual * 20.0
                + envelope.max_joint_delta_rad * 4.0
                + limit_penalty
                + if floor_clear { 0.0 } else { 1000.0 };
            let solution = Solution {
                joints: self.names().into_iter().zip(joints).collect(),
                residual_m: residual,
                orientation_residual_rad: orientation_residual,
                max_delta_rad: envelope.max_joint_delta_rad,
                score,
                floor_clear,
                orientation_offset_rpy_rad: orientation_offset,
                min_limit_margin_rad: envelope.min_joint_limit_margin_rad,
                min_tip_z_m: envelope.min_tip_z_m,
            };
            if solution.residual_m <= 0.015
                && solution.orientation_residual_rad <= 0.10
                && solution.floor_clear
                && solution.min_limit_margin_rad >= PLANNING_MIN_LIMIT_MARGIN_RAD
            {
                accepted.push(solution);
            } else if best_rejected
                .as_ref()
                .map(|candidate: &Solution| solution.score < candidate.score)
                .unwrap_or(true)
            {
                best_rejected = Some(solution);
            }
        }
        let mut accepted = distinct_solutions(accepted);
        accepted.sort_by(|left, right| left.score.total_cmp(&right.score));
        if accepted.is_empty() {
            if let Some(solution) = &best_rejected {
                eprintln!(
                    "IK_REJECT target={:.4},{:.4},{:.4} offset={:.3},{:.3},{:.3} residual_m={:.5} orientation_rad={:.5} floor_clear={} limit_margin_rad={:.5}",
                    target[0], target[1], target[2], orientation_offset[0],
                    orientation_offset[1], orientation_offset[2], solution.residual_m,
                    solution.orientation_residual_rad, solution.floor_clear,
                    solution.min_limit_margin_rad
                );
            }
        }
        accepted
    }
}

fn distinct_solutions(mut solutions: Vec<Solution>) -> Vec<Solution> {
    solutions.sort_by(|left, right| left.score.total_cmp(&right.score));
    let mut unique = Vec::new();
    for solution in solutions {
        let duplicate = unique.iter().any(|existing: &Solution| {
            existing.joints.iter().zip(&solution.joints).all(
                |((left_name, left), (right_name, right))| {
                    left_name == right_name && (left - right).abs() < 1e-3
                },
            )
        });
        if !duplicate {
            unique.push(solution);
        }
    }
    unique
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solution(joints: [f64; 2], score: f64) -> Solution {
        Solution {
            joints: vec![("j1".into(), joints[0]), ("j2".into(), joints[1])],
            residual_m: 0.0,
            orientation_residual_rad: 0.0,
            max_delta_rad: 0.0,
            score,
            floor_clear: true,
            orientation_offset_rpy_rad: [0.0; 3],
            min_limit_margin_rad: 1.0,
            min_tip_z_m: 1.0,
        }
    }

    #[test]
    fn duplicate_ik_branches_keep_the_best_score() {
        let unique = distinct_solutions(vec![
            solution([0.0, 0.0], 2.0),
            solution([0.0005, 0.0005], 1.0),
            solution([0.1, 0.0], 3.0),
        ]);
        assert_eq!(unique.len(), 2);
        assert_eq!(unique[0].score, 1.0);
    }
}
