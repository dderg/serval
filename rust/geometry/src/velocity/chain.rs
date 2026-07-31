//! Reading a constant-jerk phase chain: the exact `(v, a)` a chain holds at an
//! arc-length, and whether its joints chain as one continuous state.

use super::profile::StraightPhase;

const POS_EPS_MM: f64 = 1e-12;
const CROSSING_BISECT_ITERS: u32 = 48;

#[derive(Clone, Copy, Debug)]
struct State {
    s: f64,
    v: f64,
    a: f64,
}

/// Constant-jerk step. A step that stalls (speed reaches zero while still
/// decelerating) ends at the stall — the position cubic folds there, and
/// integrating past the fold walks backwards along the chain.
fn advance(st: State, j: f64, dt: f64) -> State {
    let dt = if speed_dips_to_zero(st, j, dt) {
        next_stall(st, j).map_or(dt, |ts| ts.min(dt))
    } else {
        dt
    };
    State {
        s: st.s + st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0,
        v: (st.v + st.a * dt + 0.5 * j * dt * dt).max(0.0),
        a: st.a + j * dt,
    }
}

fn speed_dips_to_zero(st: State, j: f64, dt: f64) -> bool {
    let v_end = st.v + st.a * dt + 0.5 * j * dt * dt;
    if v_end <= 0.0 {
        return true;
    }
    if j <= 0.0 || st.a >= 0.0 {
        return false;
    }
    -st.a / j < dt && st.v - st.a * st.a / (2.0 * j) <= 0.0
}

/// Smallest non-negative time at which the speed reaches zero from above, in
/// closed form. A root the motion departs from positively (rest with
/// positive jerk or acceleration behind it) is not a stall.
fn next_stall(st: State, j: f64) -> Option<f64> {
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

/// Time for the constant-jerk motion from `st` to advance `ds`, or `None` if
/// it stalls (speed reaches zero) first. The bisection bracket is capped at
/// the stall, where the position curve folds — a bracket extending past it
/// would converge onto the fold instead of the crossing.
fn time_to_cross(st: State, j: f64, ds: f64) -> Option<f64> {
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
    for _ in 0..CROSSING_BISECT_ITERS {
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

fn start_state(p: &StraightPhase) -> State {
    State {
        s: p.s0,
        v: p.v0,
        a: p.a0,
    }
}

pub(super) fn phase_end_s(p: &StraightPhase) -> f64 {
    p.s0 + p.v0 * p.dt + 0.5 * p.a0 * p.dt * p.dt + p.j * p.dt * p.dt * p.dt / 6.0
}

/// Whether every joint in the chain is state-continuous — each phase's end
/// state is the next phase's start state. Under a jerk limit acceleration is
/// part of the continuous state; with jerk unlimited the profile steps its
/// acceleration at phase joints by design, so only position and velocity
/// gate (`require_accel = false`).
pub(super) fn chain_is_continuous(chain: &[StraightPhase], require_accel: bool) -> bool {
    chain.windows(2).all(|w| {
        let (p, q) = (&w[0], &w[1]);
        let e = advance(start_state(p), p.j, p.dt);
        (e.s - q.s0).abs() <= 1e-9 * (1.0 + q.s0.abs())
            && (e.v - q.v0).abs() <= 1e-8 * (1.0 + q.v0)
            && (!require_accel || (e.a - q.a0).abs() <= 1e-4 * (1.0 + q.a0.abs()))
    })
}

/// Exact `(v, a)` at each arc-length in ascending `s`, read off the same
/// closed-form phases the lowering executes.
pub(super) fn chain_states(chain: &[StraightPhase], s: &[f64]) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(s.len());
    let mut idx = 0usize;
    for &x in s {
        while idx + 1 < chain.len() && phase_end_s(&chain[idx]) < x - POS_EPS_MM {
            idx += 1;
        }
        let p = &chain[idx];
        let st = start_state(p);
        let end = phase_end_s(p);
        let state = if x <= p.s0 + POS_EPS_MM {
            (p.v0, p.a0)
        } else if x >= end - POS_EPS_MM {
            let e = advance(st, p.j, p.dt);
            (e.v, e.a)
        } else {
            match time_to_cross(st, p.j, x - p.s0) {
                Some(tau) => {
                    let e = advance(st, p.j, tau);
                    (e.v, e.a)
                }
                None => (p.v0, p.a0),
            }
        };
        out.push(state);
    }
    out
}

#[cfg(test)]
mod tests;
