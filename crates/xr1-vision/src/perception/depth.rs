use crate::observation::Transform;
use std::cmp::Ordering;
use std::fs;
use std::path::Path;

pub(super) fn at_or_neighbor(
    depth: &[f32],
    width: usize,
    height: usize,
    u: usize,
    v: usize,
) -> Option<f64> {
    let direct = f64::from(depth[v * width + u]);
    if direct.is_finite() && (0.20..=2.0).contains(&direct) {
        return Some(direct);
    }
    let mut nearby = Vec::new();
    for y in v.saturating_sub(2)..=(v + 2).min(height - 1) {
        for x in u.saturating_sub(2)..=(u + 2).min(width - 1) {
            let value = f64::from(depth[y * width + x]);
            if value.is_finite() && (0.20..=2.0).contains(&value) {
                nearby.push(value);
            }
        }
    }
    if nearby.len() < 3 {
        return None;
    }
    nearby.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    if nearby[nearby.len() - 1] - nearby[0] > 0.040 {
        return None;
    }
    Some(nearby[nearby.len() / 2])
}

pub(super) fn in_manipulation_workspace(point: [f64; 3]) -> bool {
    (0.15..=0.80).contains(&point[0])
        && (-0.60..=0.60).contains(&point[1])
        && (0.76..=0.86).contains(&point[2])
}

pub(super) fn transform_point(tf: &Transform, point: [f64; 3]) -> Result<[f64; 3], String> {
    let rotation = rotation_matrix(tf)?;
    let rotated: [f64; 3] = std::array::from_fn(|row| {
        (0..3)
            .map(|column| rotation[row][column] * point[column])
            .sum::<f64>()
    });
    Ok(std::array::from_fn(|index| {
        rotated[index] + tf.translation_m[index]
    }))
}

pub(super) fn inverse_transform_point(tf: &Transform, point: [f64; 3]) -> Result<[f64; 3], String> {
    let rotation = rotation_matrix(tf)?;
    let delta = std::array::from_fn::<_, 3, _>(|index| point[index] - tf.translation_m[index]);
    Ok(std::array::from_fn(|column| {
        (0..3)
            .map(|row| rotation[row][column] * delta[row])
            .sum::<f64>()
    }))
}

fn rotation_matrix(tf: &Transform) -> Result<[[f64; 3]; 3], String> {
    if tf.translation_m.len() != 3 || tf.rotation_xyzw.len() != 4 {
        return Err("invalid TF dimensions".into());
    }
    let [x, y, z, w] = [
        tf.rotation_xyzw[0],
        tf.rotation_xyzw[1],
        tf.rotation_xyzw[2],
        tf.rotation_xyzw[3],
    ];
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if !norm.is_finite() || norm < 1e-9 {
        return Err("invalid TF quaternion".into());
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    Ok([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])
}

pub(super) fn read_npy_f32(path: &Path, expected: usize) -> Result<Vec<f32>, String> {
    let data = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    if data.len() < 12 || &data[..6] != b"\x93NUMPY" {
        return Err("invalid NPY file".into());
    }
    let major = data[6];
    let (header_len, start) = match major {
        1 => (u16::from_le_bytes([data[8], data[9]]) as usize, 10),
        2 | 3 => (
            u32::from_le_bytes([data[8], data[9], data[10], data[11]]) as usize,
            12,
        ),
        _ => return Err(format!("unsupported NPY version {major}")),
    };
    let payload = start + header_len;
    if payload > data.len() || (data.len() - payload) / 4 != expected {
        return Err("NPY shape or payload size mismatch".into());
    }
    let header = std::str::from_utf8(&data[start..payload])
        .map_err(|_| "NPY header is not UTF-8".to_string())?;
    if !header.contains("'<f4'") && !header.contains("\"<f4\"") {
        return Err("NPY depth must use little-endian float32".into());
    }
    if header.contains("True") {
        return Err("Fortran-ordered NPY depth is unsupported".into());
    }
    Ok(data[payload..]
        .chunks_exact(4)
        .map(|value| f32::from_le_bytes([value[0], value[1], value[2], value[3]]))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_transform_preserves_a_point() {
        let transform = Transform {
            target_frame: "base".into(),
            source_frame: "camera".into(),
            translation_m: vec![0.0; 3],
            rotation_xyzw: vec![0.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(
            transform_point(&transform, [1.0, 2.0, 3.0]),
            Ok([1.0, 2.0, 3.0])
        );
    }

    #[test]
    fn inverse_transform_undoes_a_rotated_transform() {
        let half = std::f64::consts::FRAC_PI_4.sin();
        let transform = Transform {
            target_frame: "base".into(),
            source_frame: "camera".into(),
            translation_m: vec![0.5, -0.2, 1.0],
            rotation_xyzw: vec![0.0, 0.0, half, half],
        };
        let camera = [0.1, -0.3, 0.8];
        let base = transform_point(&transform, camera);
        assert!(base.is_ok());
        let Some(base) = base.ok() else { return };
        let recovered = inverse_transform_point(&transform, base);
        assert!(recovered.is_ok());
        let Some(recovered) = recovered.ok() else {
            return;
        };
        for (actual, expected) in recovered.iter().zip(camera) {
            assert!((actual - expected).abs() < 1e-12);
        }
    }
}
