use super::EVENT_BISECT_ITERS;

#[derive(Clone, Copy, Debug)]
pub(in crate::velocity) struct State {
    pub t: f64,
    pub s: f64,
    pub v: f64,
    pub a: f64,
}

pub(in crate::velocity) fn advance(st: State, j: f64, dt: f64) -> State {
    State {
        t: st.t + dt,
        s: st.s + st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0,
        v: (st.v + st.a * dt + 0.5 * j * dt * dt).max(0.0),
        a: st.a + j * dt,
    }
}

/// Time for the constant-jerk motion from `st` to advance `ds`, or `None` if
/// it stalls (speed reaches zero) first. The bisection bracket is capped at
/// the stall, where the position curve folds — a bracket extending past it
/// would converge onto the fold instead of the crossing.
pub(in crate::velocity) fn time_to_cross(st: State, j: f64, ds: f64) -> Option<f64> {
    if ds <= 0.0 {
        return Some(0.0);
    }
    let pos = |dt: f64| st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0;
    let stall = next_stall(st, j);
    let mut hi = {
        let by_v = ds / st.v.max(1e-9);
        let by_a = (2.0 * ds / st.a.abs().max(1e-9)).sqrt();
        let mut hi = by_v.min(by_a);
        // The jerk-term seed only matters starting near rest; cbrt is too
        // expensive to pay on every crossing solve.
        if hi > 1.0 {
            hi = hi.min(libm::cbrt(6.0 * ds / j.abs().max(1e-9)));
        }
        hi.max(1e-12)
    };
    let mut guard = 0;
    loop {
        if let Some(ts) = stall {
            if hi >= ts {
                if pos(ts) < ds {
                    return None;
                }
                hi = ts;
                break;
            }
        }
        if pos(hi) >= ds {
            break;
        }
        hi *= 2.0;
        guard += 1;
        if guard > 200 {
            return None;
        }
    }
    let vel = |dt: f64| st.v + st.a * dt + 0.5 * j * dt * dt;
    let mut lo = 0.0;
    let mut t = hi;
    for _ in 0..EVENT_BISECT_ITERS {
        let f = pos(t) - ds;
        if f >= 0.0 {
            hi = t;
        } else {
            lo = t;
        }
        let dv = vel(t);
        let newton = if dv > 0.0 { t - f / dv } else { f64::NAN };
        let next = if newton > lo && newton < hi {
            newton
        } else {
            0.5 * (lo + hi)
        };
        if (next - t).abs() <= 1e-15 * (1.0 + t) {
            return Some(next);
        }
        t = next;
    }
    Some(t)
}

/// Step duration: the aim `dt_aim`, clipped to the crossing of `ds` ahead.
/// The crossing solve is skipped when the aim provably stays inside — the
/// position cubic is monotone up to the stall, so one eval decides.
pub(super) fn step_within(st: State, j: f64, dt_aim: f64, ds: f64) -> f64 {
    let stalls_first = next_stall(st, j).is_some_and(|ts| ts < dt_aim);
    if !stalls_first && dt_aim.is_finite() {
        let pos = st.v * dt_aim + 0.5 * st.a * dt_aim * dt_aim + j * dt_aim * dt_aim * dt_aim / 6.0;
        if pos <= ds {
            return dt_aim;
        }
    }
    match time_to_cross(st, j, ds) {
        Some(t) => t.min(dt_aim),
        None => dt_aim,
    }
}

/// A feasibility-march step: like [`step_within`] but the boundary crossing
/// is a one-shot trapezoid estimate, not a solve. The march re-reads its cell
/// after every advance and its verdict comes from state checks, so landing a
/// cell-fraction past (or short of) the boundary costs nothing — cell-exact
/// crossing times are wasted precision here. Near a stall the estimate is
/// unsafe (the position cubic folds), so that case falls through to the
/// exact solve.
pub(super) fn march_step(st: State, j: f64, dt_aim: f64, ds: f64) -> f64 {
    let stall = next_stall(st, j);
    if stall.is_none_or(|ts| ts >= dt_aim) {
        let pos = st.v * dt_aim + 0.5 * st.a * dt_aim * dt_aim + j * dt_aim * dt_aim * dt_aim / 6.0;
        if pos <= ds {
            return dt_aim;
        }
        if st.v > 1e-6 {
            let dt0 = (ds / st.v).min(dt_aim);
            let v1 = (st.v + st.a * dt0 + 0.5 * j * dt0 * dt0).max(0.0);
            if v1 > 1e-6 {
                let dt = (2.0 * ds / (st.v + v1)).min(dt_aim);
                if stall.is_none_or(|ts| ts >= dt) {
                    return dt;
                }
            }
        }
    }
    match time_to_cross(st, j, ds) {
        Some(t) => t.min(dt_aim),
        None => dt_aim,
    }
}

/// Smallest non-negative time at which the speed reaches zero from above, in
/// closed form. A root the motion departs from positively (rest with
/// positive jerk or acceleration behind it) is not a stall.
pub(super) fn next_stall(st: State, j: f64) -> Option<f64> {
    let arriving = |t: f64| st.a + j * t < 0.0 || (st.a + j * t == 0.0 && j <= 0.0 && st.v <= 0.0);
    if j.abs() < 1e-300 {
        if st.a >= 0.0 {
            return (st.v <= 0.0 && st.a == 0.0).then_some(0.0);
        }
        return Some((st.v / -st.a).max(0.0));
    }
    let disc = st.a * st.a - 2.0 * j * st.v;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let (r1, r2) = ((-st.a - sq) / j, (-st.a + sq) / j);
    [r1.min(r2), r1.max(r2)]
        .into_iter()
        .find(|&t| t >= 0.0 && arriving(t))
}
