use geometry::StraightPhase;

use super::LoweringError;

/// Which window a query time on a shared knot resolves to. A phase-built
/// profile steps its acceleration exactly at knots, so a fitted span must read
/// its own side of the joint: the span beginning at the knot reads `Begin`
/// (the right window's start state), the span ending there reads `End` (the
/// left window's end state). Interior times resolve identically either way.
#[derive(Clone, Copy)]
pub(super) enum KnotSide {
    Begin,
    End,
}

const PROFILE_OVERSHOOT_EPS: f64 = 1e-6;
const PROFILE_VELOCITY_FLOOR: f64 = 1e-9;

/// One window's `s(τ)` polynomial in local time. Adjacent windows share their
/// knot state, so the whole move's `s(t)` is C2 by construction.
pub(super) struct QuinticWindow {
    pub(super) dt: f64,
    pub(super) coeffs: [f64; 6],
    pub(super) s0: f64,
    pub(super) s1: f64,
}

impl QuinticWindow {
    fn state_at(&self, tau: f64) -> (f64, f64, f64) {
        let c = &self.coeffs;
        let s = (((((c[5] * tau) + c[4]) * tau + c[3]) * tau + c[2]) * tau + c[1]) * tau + c[0];
        let v = ((((5.0 * c[5] * tau) + 4.0 * c[4]) * tau + 3.0 * c[3]) * tau + 2.0 * c[2]) * tau
            + c[1];
        let a = ((20.0 * c[5] * tau) + 12.0 * c[4]) * tau * tau + 6.0 * c[3] * tau + 2.0 * c[2];
        (s, v, a)
    }
}

/// C2 scalar arc-length profile `s(t)`: one polynomial window per plan phase.
pub(super) struct ScalarProfile {
    pub(super) windows: Vec<QuinticWindow>,
    pub(super) knot_t: Vec<f64>,
}

impl ScalarProfile {
    fn locate(&self, t: f64, side: KnotSide) -> (usize, f64) {
        let count = match side {
            KnotSide::Begin => self.knot_t.partition_point(|&kt| kt <= t),
            KnotSide::End => self.knot_t.partition_point(|&kt| kt < t),
        };
        let idx = count.saturating_sub(1).min(self.windows.len() - 1);
        let tau = (t - self.knot_t[idx]).clamp(0.0, self.windows[idx].dt);
        (idx, tau)
    }

    #[cfg(test)]
    pub(super) fn state_at(&self, t: f64) -> (f64, f64, f64) {
        self.state_at_side(t, KnotSide::Begin)
    }

    pub(super) fn state_at_side(&self, t: f64, side: KnotSide) -> (f64, f64, f64) {
        let (idx, tau) = self.locate(t, side);
        let w = &self.windows[idx];
        let (s, v, a) = w.state_at(tau);
        let margin = PROFILE_OVERSHOOT_EPS * (1.0 + (w.s1 - w.s0).abs());
        debug_assert!(
            s >= w.s0 - margin && s <= w.s1 + margin,
            "quintic profile position {s} escaped window [{}, {}]",
            w.s0,
            w.s1
        );
        debug_assert!(
            v >= -PROFILE_VELOCITY_FLOOR,
            "quintic profile velocity {v} below zero"
        );
        (s, v, a)
    }
}

/// Exact scalar profile from a move's closed-form phases: one polynomial
/// window per phase, coefficients read straight off the constant-jerk
/// kinematics. Nothing is interpolated — the windows are the plan itself, so
/// acceleration steps at phase joints stay exactly at knots.
pub(super) fn profile_from_phases(
    phases: &[StraightPhase],
) -> Result<(ScalarProfile, f64), LoweringError> {
    if phases.is_empty() {
        return Err(LoweringError::EmptyProfile);
    }
    let mut windows = Vec::with_capacity(phases.len());
    let mut knot_t = Vec::with_capacity(phases.len() + 1);
    knot_t.push(0.0);
    let mut t_acc = 0.0;
    for p in phases {
        let finite = p.dt.is_finite() && p.s0.is_finite() && p.v0.is_finite() && p.a0.is_finite();
        if !(finite && p.j.is_finite() && p.dt > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        let s1 = p.s0 + p.dt * (p.v0 + p.dt * (0.5 * p.a0 + p.dt * p.j / 6.0));
        windows.push(QuinticWindow {
            dt: p.dt,
            coeffs: [p.s0, p.v0, 0.5 * p.a0, p.j / 6.0, 0.0, 0.0],
            s0: p.s0,
            s1,
        });
        t_acc += p.dt;
        knot_t.push(t_acc);
    }
    Ok((ScalarProfile { windows, knot_t }, t_acc))
}
