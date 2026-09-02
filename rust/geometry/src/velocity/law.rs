//! Exact scalar time laws.
//!
//! Under unlimited jerk the time-optimal scalar profile is piecewise exactly
//! one of two laws: constant tangential acceleration (cruise when zero, the
//! straight-line rail otherwise), or the curved disk rail
//! `dv/dt = ±sqrt(A² − κ(s)²·v⁴)` with `κ(s) = κ0 + σ·(s − s0)` — total
//! acceleration pinned to the budget while the curvature turns it. There is
//! no jerk parameter: a constant-jerk cubic is not a law any axis executes,
//! it was only ever a grid-cell approximation of the rail.
//!
//! `ConstAccel` is closed-form. `DiskRail` is evaluated through a dense
//! Runge–Kutta solution built deterministically from the law parameters and
//! cached inside the segment; between knots velocity and arc interpolate at
//! O(h⁴) float accuracy while the acceleration is recomputed *from the law*
//! at the interpolated state, so every sample lies exactly on the disk.

use std::sync::{Arc, OnceLock};

/// Knot-count clamp for the dense rail solution.
const RAIL_KNOTS_MIN: usize = 16;
const RAIL_KNOTS_MAX: usize = 32768;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScalarLaw {
    /// Constant tangential acceleration; cruise when `a0 == 0`.
    ConstAccel { a0: f64 },
    /// The curved disk rail at total-acceleration budget `accel`, with the
    /// path curvature `κ(s) = kappa0 + sigma·(s − segment.s0)` (both signed
    /// magnitudes; only |κ| enters the law). `brake` selects the descending
    /// branch.
    DiskRail {
        accel: f64,
        kappa0: f64,
        sigma: f64,
        brake: bool,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct RailKnot {
    t: f64,
    s: f64,
    v: f64,
}

/// One law over one time interval, in chain-local time and run-frame arc.
#[derive(Clone, Debug)]
pub struct LawSegment {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub law: ScalarLaw,
    dense: OnceLock<Arc<[RailKnot]>>,
}

impl PartialEq for LawSegment {
    fn eq(&self, other: &Self) -> bool {
        self.t0 == other.t0
            && self.dt == other.dt
            && self.s0 == other.s0
            && self.v0 == other.v0
            && self.law == other.law
    }
}

fn rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
}

impl ScalarLaw {
    /// Tangential acceleration commanded by the law at state `(ds, v)`,
    /// where `ds` is arc travelled since the segment start.
    fn accel_at(&self, ds: f64, v: f64) -> f64 {
        match *self {
            ScalarLaw::ConstAccel { a0 } => a0,
            ScalarLaw::DiskRail {
                accel,
                kappa0,
                sigma,
                brake,
            } => {
                let magnitude = rail_accel(accel, (kappa0 + sigma * ds).abs(), v);
                if brake { -magnitude } else { magnitude }
            }
        }
    }

    /// Time derivative of `accel_at` along the law's own trajectory, used to
    /// interpolate a velocity whose derivative is the law acceleration.
    fn accel_rate_at(&self, ds: f64, v: f64, a: f64) -> f64 {
        match *self {
            ScalarLaw::ConstAccel { .. } => 0.0,
            ScalarLaw::DiskRail {
                accel,
                kappa0,
                sigma,
                ..
            } => {
                let kappa = kappa0 + sigma * ds;
                let q = kappa.abs() * v * v;
                let root = a.abs();
                if root <= 0.02 * accel {
                    return 0.0;
                }
                let q_rate = kappa.signum() * sigma * v * v * v + 2.0 * kappa.abs() * v * a;
                -a.signum() * q * q_rate / root
            }
        }
    }

    /// Where a fixed-step solution that crossed the curvature cap `A/|κ|`
    /// between `from` and `to` actually is. The rail's slope vanishes as a
    /// square root at the cap, so a cap holding or rising ahead of the state
    /// draws it on with unbounded strength and the true solution settles on
    /// the `v²` whose remaining tangential budget is exactly the cap's own
    /// slope `A·|σ|/κ²`: the step overshot, the state did not cross. A cap
    /// that falls toward the state, or one rising faster than any rail can
    /// climb (`|σ| ≥ 2κ²`), has no such curve; the step stands as a
    /// reachability bound.
    fn settle_crossed_cap(&self, from: f64, to: f64, w_next: f64) -> f64 {
        let ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            ..
        } = *self
        else {
            return w_next;
        };
        let kappa = (kappa0 + sigma * to).abs();
        let cap = if kappa > 0.0 {
            accel / kappa
        } else {
            f64::INFINITY
        };
        let falling = kappa > (kappa0 + sigma * from).abs();
        let follow = 0.5 * sigma / (kappa * kappa);
        if w_next <= cap || falling || follow.abs() >= 1.0 {
            return w_next;
        }
        cap * (1.0 - follow * follow).sqrt()
    }
}

impl LawSegment {
    pub fn new(t0: f64, dt: f64, s0: f64, v0: f64, law: ScalarLaw) -> Self {
        assert!(
            matches!(law, ScalarLaw::ConstAccel { .. }),
            "a DiskRail segment is arc-bounded; construct it with until_arc"
        );
        LawSegment {
            t0,
            dt,
            s0,
            v0,
            law,
            dense: OnceLock::new(),
        }
    }

    pub fn end_time(&self) -> f64 {
        self.t0 + self.dt
    }

    /// State `(s, v, a)` at chain-local time `t`, clamped to the segment.
    pub fn state_at(&self, t: f64) -> (f64, f64, f64) {
        let tau = (t - self.t0).clamp(0.0, self.dt);
        match self.law {
            ScalarLaw::ConstAccel { a0 } => (
                self.s0 + self.v0 * tau + 0.5 * a0 * tau * tau,
                self.v0 + a0 * tau,
                a0,
            ),
            ScalarLaw::DiskRail { .. } => {
                let knots = self.rail_knots();
                let idx = knots
                    .partition_point(|k| k.t <= tau)
                    .saturating_sub(1)
                    .min(knots.len() - 2);
                let (lo, hi) = (&knots[idx], &knots[idx + 1]);
                let h = hi.t - lo.t;
                if h <= 0.0 {
                    let ds = lo.s;
                    return (self.s0 + ds, lo.v, self.law.accel_at(ds, lo.v));
                }
                let u = ((tau - lo.t) / h).clamp(0.0, 1.0);
                let a_lo = self.law.accel_at(lo.s, lo.v);
                let a_hi = self.law.accel_at(hi.s, hi.v);
                let r_lo = self.law.accel_rate_at(lo.s, lo.v, a_lo);
                let r_hi = self.law.accel_rate_at(hi.s, hi.v, a_hi);
                let v = quintic(
                    lo.v,
                    a_lo * h,
                    r_lo * h * h,
                    hi.v,
                    a_hi * h,
                    r_hi * h * h,
                    u,
                );
                let a = quintic_deriv(
                    lo.v,
                    a_lo * h,
                    r_lo * h * h,
                    hi.v,
                    a_hi * h,
                    r_hi * h * h,
                    u,
                ) / h;
                let mass = quintic_integral(
                    lo.v,
                    a_lo * h,
                    r_lo * h * h,
                    hi.v,
                    a_hi * h,
                    r_hi * h * h,
                    u,
                );
                let full_mass = quintic_integral(
                    lo.v,
                    a_lo * h,
                    r_lo * h * h,
                    hi.v,
                    a_hi * h,
                    r_hi * h * h,
                    1.0,
                );
                let ds = lo.s + h * mass + (hi.s - lo.s - h * full_mass) * u;
                (self.s0 + ds, v.max(0.0), a)
            }
        }
    }

    pub fn end_state(&self) -> (f64, f64, f64) {
        self.state_at(self.t0 + self.dt)
    }

    pub fn end_distance(&self) -> f64 {
        self.end_state().0
    }

    /// Chain-local time at which the segment reaches run arc `s`, or `None`
    /// when the arc is outside the segment's span.
    pub fn time_at_distance(&self, s: f64) -> Option<f64> {
        let ds = s - self.s0;
        if ds < -1e-9 {
            return None;
        }
        if ds <= 0.0 {
            return Some(self.t0);
        }
        match self.law {
            ScalarLaw::ConstAccel { a0 } => {
                let end = self.end_distance() - self.s0;
                if ds > end + 1e-9 {
                    return None;
                }
                // At a rest end the quadratic has a double root and the
                // closed form loses half its digits; the end arc maps to the
                // segment's own duration exactly.
                if ds >= end - 1e-9 * (1.0 + end.abs()) {
                    return Some(self.t0 + self.dt);
                }
                let tau = positive_quad_root(0.5 * a0, self.v0, ds)?;
                Some(self.t0 + tau.min(self.dt))
            }
            ScalarLaw::DiskRail { .. } => {
                let knots = self.rail_knots();
                let end = knots[knots.len() - 1].s;
                if ds > end + 1e-9 {
                    return None;
                }
                if ds >= end - 1e-9 * (1.0 + end.abs()) {
                    return Some(self.t0 + self.dt);
                }
                let idx = knots
                    .partition_point(|k| k.s <= ds)
                    .saturating_sub(1)
                    .min(knots.len() - 2);
                let (lo, hi) = (&knots[idx], &knots[idx + 1]);
                let h = hi.t - lo.t;
                if h <= 0.0 || hi.s - lo.s <= 0.0 {
                    return Some(self.t0 + lo.t);
                }
                let mut u = ((ds - lo.s) / (hi.s - lo.s)).clamp(0.0, 1.0);
                for _ in 0..8 {
                    let s_u = hermite(lo.s, lo.v * h, hi.s, hi.v * h, u);
                    let v_u = hermite(
                        lo.v,
                        self.law.accel_at(lo.s, lo.v) * h,
                        hi.v,
                        self.law.accel_at(hi.s, hi.v) * h,
                        u,
                    );
                    let slope = (v_u * h).max(1e-12);
                    u = (u - (s_u - ds) / slope).clamp(0.0, 1.0);
                }
                Some(self.t0 + (lo.t + u * h).min(self.dt))
            }
        }
    }

    /// The lowest velocity anywhere in the segment.
    pub fn min_velocity(&self) -> f64 {
        match self.law {
            ScalarLaw::ConstAccel { a0 } => {
                let v_end = self.v0 + a0 * self.dt;
                self.v0.min(v_end)
            }
            ScalarLaw::DiskRail { brake, .. } => {
                if brake {
                    self.end_state().1
                } else {
                    self.v0
                }
            }
        }
    }

    /// Construct a segment covering exactly `ds` of arc from `(s0, v0)`; the
    /// duration falls out of the law. `None` when the law stalls (reaches
    /// rest) strictly before covering the arc — a rest landing must target
    /// the stall arc exactly, not overshoot it.
    pub fn until_arc(t0: f64, s0: f64, v0: f64, law: ScalarLaw, ds: f64) -> Option<LawSegment> {
        match law {
            ScalarLaw::ConstAccel { a0 } => {
                let dt = positive_quad_root(0.5 * a0, v0, ds)?;
                let v_end = v0 + a0 * dt;
                if v_end < -1e-9 {
                    return None;
                }
                Some(LawSegment::new(t0, dt, s0, v0, law))
            }
            ScalarLaw::DiskRail { .. } => {
                let knots = integrate_rail_arc(&law, v0, ds)?;
                let dt = knots[knots.len() - 1].t;
                let seg = LawSegment {
                    t0,
                    dt,
                    s0,
                    v0,
                    law,
                    dense: OnceLock::new(),
                };
                let _ = seg.dense.set(knots);
                Some(seg)
            }
        }
    }

    /// Speed after covering `ds` of arc under `law` from `(0, v0)`, or `None`
    /// on a stall before the end.
    pub fn reach_over(law: ScalarLaw, v0: f64, ds: f64) -> Option<f64> {
        match law {
            ScalarLaw::ConstAccel { a0 } => {
                let w = v0 * v0 + 2.0 * a0 * ds;
                (w >= -1e-12).then(|| w.max(0.0).sqrt())
            }
            ScalarLaw::DiskRail { .. } => {
                let knots = integrate_rail_arc(&law, v0, ds)?;
                Some(knots[knots.len() - 1].v)
            }
        }
    }

    /// Construct the braking segment that covers `ds` of arc from `s0` and
    /// lands at exactly `v_end` (which may be rest): the law is integrated in
    /// the reversed frame from the anchor — where the landing is exact by
    /// construction — and flipped into forward time. Returns the segment and
    /// its entry speed.
    pub fn brake_to(
        t0: f64,
        s0: f64,
        law: ScalarLaw,
        ds: f64,
        v_end: f64,
    ) -> Option<(LawSegment, f64)> {
        match law {
            ScalarLaw::ConstAccel { a0 } => {
                if a0 >= 0.0 {
                    return None;
                }
                let v0 = (v_end * v_end - 2.0 * a0 * ds).sqrt();
                let dt = (v_end - v0) / a0;
                Some((LawSegment::new(t0, dt, s0, v0, law), v0))
            }
            ScalarLaw::DiskRail {
                accel,
                kappa0,
                sigma,
                brake,
            } => {
                if !brake {
                    return None;
                }
                let reversed = ScalarLaw::DiskRail {
                    accel,
                    kappa0: kappa0 + sigma * ds,
                    sigma: -sigma,
                    brake: false,
                };
                let rev = integrate_rail_arc(&reversed, v_end, ds)?;
                let n = rev.len();
                let total_t = rev[n - 1].t;
                let v0 = rev[n - 1].v;
                let knots: Vec<RailKnot> = (0..n)
                    .map(|i| {
                        let k = &rev[n - 1 - i];
                        RailKnot {
                            t: total_t - k.t,
                            s: ds - k.s,
                            v: k.v,
                        }
                    })
                    .collect();
                let seg = LawSegment {
                    t0,
                    dt: total_t,
                    s0,
                    v0,
                    law,
                    dense: OnceLock::new(),
                };
                let _ = seg.dense.set(Arc::from(knots));
                Some((seg, v0))
            }
        }
    }

    fn rail_knots(&self) -> &Arc<[RailKnot]> {
        self.dense
            .get()
            .expect("a DiskRail segment always carries its dense solution")
    }
}

fn quintic(p0: f64, m0: f64, c0: f64, p1: f64, m1: f64, c1: f64, u: f64) -> f64 {
    let (u2, u3) = (u * u, u * u * u);
    let (u4, u5) = (u3 * u, u3 * u * u);
    p0 * (1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5)
        + m0 * (u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5)
        + c0 * (0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5)
        + p1 * (10.0 * u3 - 15.0 * u4 + 6.0 * u5)
        + m1 * (-4.0 * u3 + 7.0 * u4 - 3.0 * u5)
        + c1 * (0.5 * u3 - u4 + 0.5 * u5)
}

fn quintic_deriv(p0: f64, m0: f64, c0: f64, p1: f64, m1: f64, c1: f64, u: f64) -> f64 {
    let (u2, u3, u4) = (u * u, u * u * u, u * u * u * u);
    p0 * (-30.0 * u2 + 60.0 * u3 - 30.0 * u4)
        + m0 * (1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4)
        + c0 * (u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4)
        + p1 * (30.0 * u2 - 60.0 * u3 + 30.0 * u4)
        + m1 * (-12.0 * u2 + 28.0 * u3 - 15.0 * u4)
        + c1 * (1.5 * u2 - 4.0 * u3 + 2.5 * u4)
}

fn quintic_integral(p0: f64, m0: f64, c0: f64, p1: f64, m1: f64, c1: f64, u: f64) -> f64 {
    let (u2, u3) = (u * u, u * u * u);
    let (u4, u5, u6) = (u3 * u, u3 * u2, u3 * u3);
    p0 * (u - 2.5 * u4 + 3.0 * u5 - u6)
        + m0 * (0.5 * u2 - 1.5 * u4 + 1.6 * u5 - 0.5 * u6)
        + c0 * (u3 / 6.0 - 0.375 * u4 + 0.3 * u5 - u6 / 12.0)
        + p1 * (2.5 * u4 - 3.0 * u5 + u6)
        + m1 * (-u4 + 1.4 * u5 - 0.5 * u6)
        + c1 * (0.125 * u4 - 0.2 * u5 + u6 / 12.0)
}

fn hermite(p0: f64, m0: f64, p1: f64, m1: f64, u: f64) -> f64 {
    let u2 = u * u;
    let u3 = u2 * u;
    (2.0 * u3 - 3.0 * u2 + 1.0) * p0
        + (u3 - 2.0 * u2 + u) * m0
        + (-2.0 * u3 + 3.0 * u2) * p1
        + (u3 - u2) * m1
}

/// Target dense-knot spacing along the arc, in millimetres.
const RAIL_KNOT_DS: f64 = 2.5e-4;

/// Dense arc-uniform solution of the rail ODE `d(v²)/ds = 2·a(s, v)` from
/// `(0, v0)` over `ds`. Each knot pair's duration is the one under which the
/// quintic state interpolant [`LawSegment::state_at`] reads between them
/// covers exactly the pair's arc, so the knot times, the interpolated
/// velocity and the interpolated arc describe one motion. `None` when the
/// speed folds to rest strictly inside the span.
fn integrate_rail_arc(law: &ScalarLaw, v0: f64, ds: f64) -> Option<Arc<[RailKnot]>> {
    if ds <= 0.0 {
        let knot = RailKnot {
            t: 0.0,
            s: 0.0,
            v: v0,
        };
        return Some(Arc::from(vec![knot, knot]));
    }
    let n = ((ds / RAIL_KNOT_DS).ceil() as usize).clamp(RAIL_KNOTS_MIN, RAIL_KNOTS_MAX);
    // Near an apex the rail acceleration vanishes and w(s) leaves the cap as
    // s^(3/2); quadratic grading toward that end restores the integrator's
    // order there.
    let apex_start = {
        let a0 = law.accel_at(0.0, v0).abs();
        let budget = match law {
            ScalarLaw::DiskRail { accel, .. } => *accel,
            ScalarLaw::ConstAccel { a0 } => a0.abs().max(1.0),
        };
        a0 < 0.1 * budget
    };
    let arc_at = |k: usize| -> f64 {
        let u = k as f64 / n as f64;
        if apex_start { ds * u * u } else { ds * u }
    };
    let mut knots = Vec::with_capacity(n + 1);
    let mut w = v0 * v0;
    let mut t = 0.0_f64;
    knots.push(RailKnot { t, s: 0.0, v: v0 });
    let f = |s: f64, w: f64| 2.0 * law.accel_at(s, w.max(0.0).sqrt());
    for i in 0..n {
        let s = arc_at(i);
        let h = arc_at(i + 1) - s;
        let k1 = f(s, w);
        let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
        let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
        let k4 = f(s + h, w + h * k3);
        let w_next =
            law.settle_crossed_cap(s, s + h, w + h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0);
        let last = i == n - 1;
        if w_next <= 0.0 && !last {
            return None;
        }
        let v_lo = w.max(0.0).sqrt();
        let v_hi = w_next.max(0.0).sqrt();
        if v_lo + v_hi <= 0.0 {
            return None;
        }
        let a_lo = law.accel_at(s, v_lo);
        let a_hi = law.accel_at(s + h, v_hi);
        let r_lo = law.accel_rate_at(s, v_lo, a_lo);
        let r_hi = law.accel_rate_at(s + h, v_hi, a_hi);
        t += knot_duration(h, (v_lo, a_lo, r_lo), (v_hi, a_hi, r_hi));
        w = w_next.max(0.0);
        knots.push(RailKnot {
            t,
            s: arc_at(i + 1),
            v: w.sqrt(),
        });
    }
    Some(Arc::from(knots))
}

/// The duration `τ` under which the quintic Hermite through `(v, a, ȧ)` at
/// both ends integrates to exactly `ds`:
/// `ds = τ·(v_lo+v_hi)/2 + τ²·(a_lo−a_hi)/10 + τ³·(ȧ_lo+ȧ_hi)/120`.
/// The leading term alone is the trapezoid, exact for constant acceleration
/// at any speed ratio — including a rest end, where a quadrature in `1/v`
/// diverges; the corrections carry the acceleration's change across the pair.
fn knot_duration(
    ds: f64,
    (v_lo, a_lo, r_lo): (f64, f64, f64),
    (v_hi, a_hi, r_hi): (f64, f64, f64),
) -> f64 {
    let c1 = 0.5 * (v_lo + v_hi);
    let c2 = (a_lo - a_hi) / 10.0;
    let c3 = (r_lo + r_hi) / 120.0;
    let mut tau = ds / c1;
    for _ in 0..8 {
        let residual = ((c3 * tau + c2) * tau + c1) * tau - ds;
        let slope = (3.0 * c3 * tau + 2.0 * c2) * tau + c1;
        let next = tau - residual / slope;
        if !(next > 0.0) || (next - tau).abs() <= 4.0 * f64::EPSILON * tau {
            break;
        }
        tau = next;
    }
    assert!(
        tau.is_finite() && tau > 0.0,
        "rail knot over {ds} of arc from v={v_lo} to v={v_hi} has no positive duration"
    );
    tau
}

/// Positive root of `c2·τ² + c1·τ − ds = 0`, cancellation-stable.
fn positive_quad_root(c2: f64, c1: f64, ds: f64) -> Option<f64> {
    if c2.abs() <= 1e-18 {
        return (c1 > 0.0).then(|| ds / c1);
    }
    let disc = c1 * c1 + 4.0 * c2 * ds;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (c1 + c1.signum() * sq);
    let (r1, r2) = (q / c2, -ds / q);
    [r1, r2]
        .into_iter()
        .filter(|r| r.is_finite() && *r >= 0.0)
        .fold(None, |acc: Option<f64>, r| {
            Some(acc.map_or(r, |a| a.min(r)))
        })
}

#[cfg(test)]
mod tests;
