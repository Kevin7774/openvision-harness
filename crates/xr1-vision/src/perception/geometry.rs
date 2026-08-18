use nalgebra::{Matrix2, Vector2, Vector3};
use std::cmp::Ordering;

/// Build a top-down object frame from the horizontal footprint. A full 3D PCA
/// on a single oblique view produces tilted axes for an upright block.
pub(super) fn object_obb(points: &[[f64; 3]], center: [f64; 3]) -> ([[f64; 3]; 3], [f64; 3]) {
    let center = Vector3::new(center[0], center[1], center[2]);
    let mut covariance = Matrix2::zeros();
    for point in points {
        let delta = Vector2::new(point[0] - center[0], point[1] - center[1]);
        covariance += delta * delta.transpose();
    }
    covariance /= points.len() as f64;
    let eigen = covariance.symmetric_eigen();
    let major = usize::from(eigen.eigenvalues[1] > eigen.eigenvalues[0]);
    let column = eigen.eigenvectors.column(major);
    let long = Vector3::new(column[0], column[1], 0.0).normalize();
    let short = Vector3::new(-long.y, long.x, 0.0);
    let frame = [long, short, Vector3::z()];
    let mut axes = [[0.0; 3]; 3];
    let mut extents = [0.0; 3];
    for (index, axis) in frame.iter().enumerate() {
        axes[index] = (*axis).into();
        let mut projections = points
            .iter()
            .map(|point| axis.dot(&(Vector3::from(*point) - center)))
            .collect::<Vec<_>>();
        projections.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
        let low = projections[(projections.len() as f64 * 0.05) as usize];
        let high =
            projections[((projections.len() as f64 * 0.95) as usize).min(projections.len() - 1)];
        extents[index] = high - low;
    }
    (axes, extents)
}

pub(super) fn robust_object_points(points: &[[f64; 3]], center: [f64; 3]) -> Vec<[f64; 3]> {
    let mut deviations = [Vec::new(), Vec::new(), Vec::new()];
    for point in points {
        for axis in 0..3 {
            deviations[axis].push((point[axis] - center[axis]).abs());
        }
    }
    let limits = deviations.map(|values| (median(values) * 4.0).clamp(0.012, 0.080));
    points
        .iter()
        .copied()
        .filter(|point| (0..3).all(|axis| (point[axis] - center[axis]).abs() <= limits[axis]))
        .collect()
}

pub(super) fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    values[values.len() / 2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn footprint_axes_stay_horizontal_for_a_block_standing_on_end() {
        let (half_x, half_y, height) = (0.015, 0.009, 0.050);
        let mut points = Vec::new();
        for i in 0..20 {
            let x_fraction = i as f64 / 19.0;
            for j in 0..20 {
                let y_fraction = j as f64 / 19.0;
                points.push([
                    -half_x + 2.0 * half_x * x_fraction,
                    -half_y + 2.0 * half_y * y_fraction,
                    0.79 + height,
                ]);
                points.push([
                    -half_x + 2.0 * half_x * x_fraction,
                    half_y,
                    0.79 + height * y_fraction,
                ]);
                points.push([
                    half_x,
                    -half_y + 2.0 * half_y * x_fraction,
                    0.79 + height * y_fraction,
                ]);
            }
        }
        let center =
            std::array::from_fn(|axis| median(points.iter().map(|point| point[axis]).collect()));
        let (axes, extents) = object_obb(&points, center);
        assert!(axes[0][2].abs() < 1e-12);
        assert!(axes[1][2].abs() < 1e-12);
        assert!((axes[2][2] - 1.0).abs() < 1e-12);
        assert!(axes[0][0].abs() > 0.99);
        assert!((0.024..=0.030).contains(&extents[0]));
        assert!((0.014..=0.018).contains(&extents[1]));
        assert!((0.040..=0.050).contains(&extents[2]));
    }
}
