const BISECTION_STEPS: u32 = 64;

pub(super) fn velocity_change_distance(v_in: f64, v_out: f64, accel: f64, jerk: f64) -> f64 {
    let delta = (v_out - v_in).abs();
    let time = if delta <= accel * accel / jerk {
        2.0 * (delta / jerk).sqrt()
    } else {
        delta / accel + accel / jerk
    };
    time * (v_in + v_out) * 0.5
}

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

pub(super) fn peak_velocity(
    v_in: f64,
    v_out: f64,
    length: f64,
    accel: f64,
    jerk: f64,
    ceiling: f64,
) -> f64 {
    let combined = |peak: f64| {
        velocity_change_distance(v_in, peak, accel, jerk)
            + velocity_change_distance(peak, v_out, accel, jerk)
    };
    let mut lo = v_in.max(v_out);
    debug_assert!(
        lo <= ceiling,
        "sweep node speed exceeded its ceiling before apex"
    );
    if combined(ceiling) <= length {
        return ceiling;
    }
    if combined(lo) >= length {
        return lo;
    }
    let mut hi = ceiling;
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        if combined(mid) <= length {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

#[cfg(test)]
mod tests;
