use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub(super) enum ReachError {
    #[error("invalid input")]
    InvalidInput,
    #[error("infeasible reach")]
    InfeasibleReach,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct SevenSeg {
    pub(super) v0: f64,
    pub(super) a0: f64,
    pub(super) s_jup_end: f64,
    pub(super) s_hold_end: f64,
    pub(super) ds: f64,
    pub(super) accel_max: f64,
    pub(super) jerk_max: f64,
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

fn validate_inputs(
    v0: f64,
    a0: f64,
    ds: f64,
    accel_max: f64,
    jerk_max: f64,
) -> Result<(), ReachError> {
    if !v0.is_finite()
        || !a0.is_finite()
        || !ds.is_finite()
        || !accel_max.is_finite()
        || v0 < 0.0
        || accel_max <= 0.0
        || ds < 0.0
        || a0 > accel_max
        || a0 < -accel_max
    {
        return Err(ReachError::InvalidInput);
    }
    if jerk_max != f64::INFINITY && (jerk_max <= 0.0 || !jerk_max.is_finite()) {
        return Err(ReachError::InvalidInput);
    }
    Ok(())
}

fn jerkup_displacement(v0: f64, a0: f64, t: f64, jerk_max: f64) -> f64 {
    v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * jerk_max * t * t * t
}

fn jerkup_velocity(v0: f64, a0: f64, t: f64, jerk_max: f64) -> f64 {
    v0 + a0 * t + 0.5 * jerk_max * t * t
}

fn solve_jerkup_cubic(v0: f64, a0: f64, jerk_max: f64, ds: f64, t_max: f64) -> f64 {
    let f = |t: f64| jerkup_displacement(v0, a0, t, jerk_max) - ds;

    let f_lo = f(0.0);
    let f_hi = f(t_max);

    debug_assert!(f_lo <= 0.0 && f_hi >= 0.0);

    let mut lo = 0.0_f64;
    let mut hi = t_max;
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        if mid == lo || mid == hi {
            break;
        }
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

enum JerkUpOutcome {
    PartialJerkUp { t: f64 },
    FullJerkUp { s_jup: f64 },
}

fn jerkup_phase(v0: f64, a0: f64, ds: f64, accel_max: f64, jerk_max: f64) -> JerkUpOutcome {
    if jerk_max == f64::INFINITY || a0 == accel_max {
        return JerkUpOutcome::FullJerkUp { s_jup: 0.0 };
    }
    let t_jup = (accel_max - a0) / jerk_max;
    let s_jup = jerkup_displacement(v0, a0, t_jup, jerk_max);
    if s_jup >= ds {
        let t = solve_jerkup_cubic(v0, a0, jerk_max, ds, t_jup);
        JerkUpOutcome::PartialJerkUp { t }
    } else {
        JerkUpOutcome::FullJerkUp { s_jup }
    }
}

pub(super) fn reach_velocity_with_accel(
    v0: f64,
    a0: f64,
    ds: f64,
    accel_max: f64,
    jerk_max: f64,
) -> Result<(f64, f64), ReachError> {
    validate_inputs(v0, a0, ds, accel_max, jerk_max)?;

    if ds == 0.0 {
        return Ok((v0, a0));
    }

    match jerkup_phase(v0, a0, ds, accel_max, jerk_max) {
        JerkUpOutcome::PartialJerkUp { t } => {
            let v1 = jerkup_velocity(v0, a0, t, jerk_max);
            let a1 = a0 + jerk_max * t;
            Ok((v1, a1))
        }
        JerkUpOutcome::FullJerkUp { s_jup } => {
            let (v_j, a1) = if jerk_max == f64::INFINITY || a0 == accel_max {
                (v0, accel_max)
            } else {
                let t_jup = (accel_max - a0) / jerk_max;
                (jerkup_velocity(v0, a0, t_jup, jerk_max), accel_max)
            };
            let d_hold = ds - s_jup;
            let v1 = (v_j * v_j + 2.0 * accel_max * d_hold).sqrt();
            Ok((v1, a1))
        }
    }
}

pub(super) fn breakpoints(
    v0: f64,
    a0: f64,
    ds: f64,
    accel_max: f64,
    jerk_max: f64,
) -> Result<SevenSeg, ReachError> {
    validate_inputs(v0, a0, ds, accel_max, jerk_max)?;

    let (s_jup_end, s_hold_end) = match jerkup_phase(v0, a0, ds, accel_max, jerk_max) {
        JerkUpOutcome::PartialJerkUp { .. } => (ds, ds),
        JerkUpOutcome::FullJerkUp { s_jup } => (s_jup, ds),
    };

    Ok(SevenSeg {
        v0,
        a0,
        s_jup_end,
        s_hold_end,
        ds,
        accel_max,
        jerk_max,
    })
}

pub(super) fn accel_at(seg: &SevenSeg, s: f64) -> f64 {
    let s = s.clamp(0.0, seg.ds);

    if s <= seg.s_jup_end {
        if seg.jerk_max == f64::INFINITY {
            seg.accel_max
        } else {
            let t = time_in_jerkup(seg, s);
            seg.a0 + seg.jerk_max * t
        }
    } else {
        seg.accel_max
    }
}

pub(super) fn velocity_at(seg: &SevenSeg, s: f64) -> f64 {
    let s = s.clamp(0.0, seg.ds);

    if s <= seg.s_jup_end {
        if seg.jerk_max == f64::INFINITY {
            (seg.v0 * seg.v0 + 2.0 * seg.accel_max * s).sqrt()
        } else {
            let t = time_in_jerkup(seg, s);
            jerkup_velocity(seg.v0, seg.a0, t, seg.jerk_max)
        }
    } else {
        let (v_j, a_hold) = jerkup_exit_state(seg);
        let d = s - seg.s_jup_end;
        (v_j * v_j + 2.0 * a_hold * d).sqrt()
    }
}

fn time_in_jerkup(seg: &SevenSeg, s: f64) -> f64 {
    if s == 0.0 {
        return 0.0;
    }
    let t_jup = if seg.jerk_max == f64::INFINITY || seg.a0 == seg.accel_max {
        0.0
    } else {
        (seg.accel_max - seg.a0) / seg.jerk_max
    };
    solve_jerkup_cubic(seg.v0, seg.a0, seg.jerk_max, s, t_jup)
}

fn jerkup_exit_state(seg: &SevenSeg) -> (f64, f64) {
    if seg.jerk_max == f64::INFINITY || seg.a0 == seg.accel_max {
        (seg.v0, seg.accel_max)
    } else {
        let t_jup = (seg.accel_max - seg.a0) / seg.jerk_max;
        (
            jerkup_velocity(seg.v0, seg.a0, t_jup, seg.jerk_max),
            seg.accel_max,
        )
    }
}

#[cfg(test)]
mod tests;
