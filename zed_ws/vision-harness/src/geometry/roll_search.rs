pub fn symmetric_rolls(step_rad: f64, count: usize) -> Vec<f64> {
    let mut values = Vec::with_capacity(count * 2 + 1);
    values.push(0.0);
    for i in 1..=count { values.push(i as f64 * step_rad); values.push(-(i as f64) * step_rad); }
    values
}
