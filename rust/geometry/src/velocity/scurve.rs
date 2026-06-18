pub(super) fn max_reachable_velocity(v_in: f64, length: f64, accel: f64, jerk: f64) -> f64 {
    let triangular_distance = (2.0 * accel / jerk) * (v_in + accel * accel / (2.0 * jerk));
    let delta = if length <= triangular_distance {
        let p = 2.0 * v_in / jerk;
        let q = -length / jerk;
        let disc = (q * q / 4.0 + p * p * p / 27.0).sqrt();
        let u = (-q / 2.0 + disc).cbrt() + (-q / 2.0 - disc).cbrt();
        jerk * u * u
    } else {
        let a = 1.0 / (2.0 * accel);
        let b = v_in / accel + accel / (2.0 * jerk);
        let c = accel * v_in / jerk - length;
        (-b + (b * b - 4.0 * a * c).sqrt()) / (2.0 * a)
    };
    v_in + delta
}

#[cfg(test)]
mod tests;
