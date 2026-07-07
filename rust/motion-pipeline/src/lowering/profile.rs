use geometry::VelSample;

use super::LoweringError;
use super::ladder::quintic_hermite_coeffs;

const PROFILE_OVERSHOOT_EPS: f64 = 1e-6;
const PROFILE_VELOCITY_FLOOR: f64 = 1e-9;

/// One sample window's duration: the trapezoid estimate `2·ds/(v0+v1)`,
/// Newton-refined against the constant-jerk kinematics through the window's
/// own endpoint states. The trapezoid value alone is off by O(ds³), and the
/// quintic window pinned to six endpoint constraints swallows that duration
/// error as interior wiggle whose acceleration amplitude grows as 1/h² — on
/// short low-speed windows it exceeds the fit budget and the piece fit chases
/// phantom jerk. The refined duration is consistent with the endpoint states,
/// so the interior stays as smooth as the plan itself.
fn window_duration(ds: f64, v0: f64, v1: f64, a0: f64, a1: f64) -> f64 {
    let trapezoid = 2.0 * ds / (v0 + v1);
    let mut dt = trapezoid;
    for _ in 0..3 {
        let j = (a1 - a0) / dt;
        let residual = dt * (v0 + dt * (0.5 * a0 + dt * j / 6.0)) - ds;
        let slope = v0 + dt * (a0 + dt * 0.5 * j);
        let next = dt - residual / slope;
        if !(next.is_finite() && next > 0.0) {
            return trapezoid;
        }
        dt = next;
    }
    if (dt - trapezoid).abs() > 0.5 * trapezoid {
        return trapezoid;
    }
    dt
}

/// Quintic Hermite piece for `s(t)` over one sample window, in local time `τ`. It
/// matches `(s, v, a)` at both knots, so adjacent windows — which share those knot
/// values — join C2, and the whole move's `s(t)` is C2 by construction.
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

/// C2 scalar arc-length profile `s(t)` for a curved move, one quintic Hermite per
/// sample window over the dense `(s, v, a)` grid.
pub(super) struct ScalarProfile {
    pub(super) windows: Vec<QuinticWindow>,
    pub(super) knot_t: Vec<f64>,
}

impl ScalarProfile {
    fn locate(&self, t: f64) -> (usize, f64) {
        let count = self.knot_t.partition_point(|&kt| kt <= t);
        let idx = count.saturating_sub(1).min(self.windows.len() - 1);
        let tau = (t - self.knot_t[idx]).clamp(0.0, self.windows[idx].dt);
        (idx, tau)
    }

    pub(super) fn state_at(&self, t: f64) -> (f64, f64, f64) {
        let (idx, tau) = self.locate(t);
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

pub(super) fn build_profile(samples: &[VelSample]) -> Result<(ScalarProfile, f64), LoweringError> {
    if samples.len() < 2 {
        return Err(LoweringError::EmptyProfile);
    }
    let mut windows = Vec::with_capacity(samples.len() - 1);
    let mut knot_t = Vec::with_capacity(samples.len());
    knot_t.push(0.0);
    let mut t_acc = 0.0;
    for w in samples.windows(2) {
        let (s0, v0, a0) = (w[0].s, w[0].v, w[0].a);
        let (s1, v1, a1) = (w[1].s, w[1].v, w[1].a);
        let ds = s1 - s0;
        let v_sum = v0 + v1;
        let finite = v0.is_finite() && v1.is_finite() && a0.is_finite() && a1.is_finite();
        if !(ds.is_finite() && ds > 0.0 && v_sum > 0.0 && finite) {
            return Err(LoweringError::DegeneratePhase);
        }
        let dt = window_duration(ds, v0, v1, a0, a1);
        if !(dt.is_finite() && dt > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        windows.push(QuinticWindow {
            dt,
            coeffs: quintic_hermite_coeffs(s0, v0, a0, s1, v1, a1, dt),
            s0,
            s1,
        });
        t_acc += dt;
        knot_t.push(t_acc);
    }
    Ok((ScalarProfile { windows, knot_t }, t_acc))
}
