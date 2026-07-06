use super::EVENT_BISECT_ITERS;

#[derive(Clone, Copy, Debug)]
pub(super) struct State {
    pub t: f64,
    pub s: f64,
    pub v: f64,
    pub a: f64,
}

pub(super) fn advance(st: State, j: f64, dt: f64) -> State {
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
pub(super) fn time_to_cross(st: State, j: f64, ds: f64) -> Option<f64> {
    if ds <= 0.0 {
        return Some(0.0);
    }
    let pos = |dt: f64| st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0;
    let stall = next_stall(st, j);
    let mut hi = {
        let by_v = ds / st.v.max(1e-9);
        let by_a = (2.0 * ds / st.a.abs().max(1e-9)).sqrt();
        let by_j = libm::cbrt(6.0 * ds / j.abs().max(1e-9));
        by_v.min(by_a).min(by_j).max(1e-12)
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
    let mut lo = 0.0;
    for _ in 0..EVENT_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        if pos(mid) < ds {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(0.5 * (lo + hi))
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
