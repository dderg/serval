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
//! cached inside the segment; between knots a C2 quintic interpolates
//! velocity, arc is its anchored integral, and acceleration is its
//! derivative taken across each knot pair's speed rise as the integrator
//! produced it, not as the difference of two rounded knot speeds. A pair
//! that leaves the curvature cap from one end interpolates in the square
//! root of the time from that end instead: see [`CapDeparture`].

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
    rise: f64,
    /// How the pair ending at this knot interpolates between its ends.
    departure: CapDeparture,
}

/// Whether a knot pair leaves the curvature cap from one of its ends. A rail
/// starting on its cap where the cap rises leaves it with `v − v₀ ∝ τ^(3/2)`
/// — the acceleration rises as `√τ` at an unbounded rate, which a polynomial
/// in time misses by a fixed fraction of the pair's acceleration change
/// however the knots are placed. Such a pair interpolates velocity as
/// `v_end + Σ cₖ·zᵏ` (k = 2..=5) in `z = √(time from that end / h)`: the
/// `z²` term carries the end's own acceleration, the odd terms the square
/// root, and the far end's velocity, acceleration and rate fix the rest.
#[derive(Clone, Copy, Debug, PartialEq)]
enum CapDeparture {
    None,
    FromLo,
    FromHi,
}

impl CapDeparture {
    fn mirrored(self) -> Self {
        match self {
            Self::None => Self::None,
            Self::FromLo => Self::FromHi,
            Self::FromHi => Self::FromLo,
        }
    }
}

/// One knot pair's velocity in its departure frame — time running away
/// from the departure end, `z = √(that time / h)` — as the rise over the
/// end's speed, `Σ cₖ·zᵏ` for k = 2..=5.
struct DepartureInterpolant {
    c: [f64; 4],
}

impl DepartureInterpolant {
    /// The interpolant leaving an end at acceleration `a_end` and reaching
    /// the far end `rise` faster at acceleration `a_far` and rate `r_far`,
    /// all in the departure frame.
    fn new(h: f64, a_end: f64, rise: f64, a_far: f64, r_far: f64) -> Self {
        let c2 = h * a_end;
        let r = rise - c2;
        let p = 2.0 * h * a_far - 2.0 * c2;
        let q = 4.0 * h * h * r_far;
        Self {
            c: [
                c2,
                10.0 * r - 3.5 * p + 0.5 * q,
                6.0 * p - 15.0 * r - q,
                0.5 * q - 2.5 * p + 6.0 * r,
            ],
        }
    }

    fn rise(&self, z: f64) -> f64 {
        let [c2, c3, c4, c5] = self.c;
        z * z * (c2 + z * (c3 + z * (c4 + z * c5)))
    }

    /// Acceleration times `h`.
    fn scaled_accel(&self, z: f64) -> f64 {
        let [c2, c3, c4, c5] = self.c;
        c2 + z * (1.5 * c3 + z * (2.0 * c4 + z * 2.5 * c5))
    }

    /// `∫ (v − v_end) du` from the departure end out to `z²`.
    fn rise_mass(&self, z: f64) -> f64 {
        let [c2, c3, c4, c5] = self.c;
        let z4 = z * z * z * z;
        z4 * (0.5 * c2 + z * (0.4 * c3 + z * (c4 / 3.0 + z * 2.0 * c5 / 7.0)))
    }
}

/// One knot pair of a disk rail: segment-local times, arc travelled since
/// the segment start and speed at both knots, and how velocity
/// interpolates between them. With `h = t_hi − t_lo` and
/// `u = (τ − t_lo)/h`, [`LawSegment::state_at`] reads speed, `h·a` and
/// `∫₀ᵘ v du` off the shape and the arc as
/// `s_lo + h·∫₀ᵘ v + (s_hi − s_lo − h·∫₀¹ v)·u`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RailPair {
    pub t_lo: f64,
    pub t_hi: f64,
    pub s_lo: f64,
    pub s_hi: f64,
    pub v_lo: f64,
    pub v_hi: f64,
    pub shape: RailPairShape,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RailPairShape {
    /// The C2 quintic Hermite through both knots' speeds with end slopes
    /// and curvatures `slopes = [h·a_lo, h²·ȧ_lo, h·a_hi, h²·ȧ_hi]`; `h·a`
    /// is its derivative across the integrator's own speed `rise`.
    Quintic { rise: f64, slopes: [f64; 4] },
    /// Leaving the curvature cap from one end: with `z` the square root of
    /// the time from that end over `h`, speed is that end's plus
    /// `z²·(c₂ + z·(c₃ + z·(c₄ + z·c₅)))`, `coefficients = [c₂, c₃, c₄, c₅]`,
    /// and `h·a` is `±(c₂ + z·(1.5c₃ + z·(2c₄ + z·2.5c₅)))`, negative
    /// leaving from the high end.
    Departure {
        from_lo: bool,
        coefficients: [f64; 4],
    },
}

/// One law over one time interval, in chain-local time and run-frame arc.
#[derive(Clone, Debug)]
pub struct LawSegment {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub law: ScalarLaw,
    dense: OnceLock<Arc<Vec<RailKnot>>>,
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

/// Below this fraction of its budget a disk rail's tangential acceleration
/// is in contact with the curvature cap: the acceleration's shape changes
/// where a rail enters or leaves it, and a rail starting in contact departs
/// it as a square root.
const CAP_CONTACT_FRACTION: f64 = 0.02;

/// Below this fraction of its budget a disk rail's tangential acceleration
/// is the rounding of a held cap — `√(A² − q²)` with `q` a few ulps from `A`
/// reads under `1e-7·A` — and its rate is a held cap's, zero. Above it the
/// law's own rate holds, departing or approaching the cap alike.
const CAP_HOLD_RESOLUTION: f64 = 1e-6;

fn rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
}

impl ScalarLaw {
    /// Whether tangential acceleration `a` puts the rail in contact with its
    /// curvature cap.
    fn in_cap_contact(&self, a: f64) -> bool {
        match *self {
            ScalarLaw::ConstAccel { .. } => false,
            ScalarLaw::DiskRail { accel, .. } => a.abs() <= CAP_CONTACT_FRACTION * accel,
        }
    }

    /// Equal to the bit, so integrating either from the same state yields the
    /// same knots; `==` would also identify `-0.0` with `0.0`.
    fn is_bitwise(&self, other: &ScalarLaw) -> bool {
        match (*self, *other) {
            (ScalarLaw::ConstAccel { a0 }, ScalarLaw::ConstAccel { a0: other_a0 }) => {
                a0.to_bits() == other_a0.to_bits()
            }
            (
                ScalarLaw::DiskRail {
                    accel,
                    kappa0,
                    sigma,
                    brake,
                },
                ScalarLaw::DiskRail {
                    accel: other_accel,
                    kappa0: other_kappa0,
                    sigma: other_sigma,
                    brake: other_brake,
                },
            ) => {
                accel.to_bits() == other_accel.to_bits()
                    && kappa0.to_bits() == other_kappa0.to_bits()
                    && sigma.to_bits() == other_sigma.to_bits()
                    && brake == other_brake
            }
            _ => false,
        }
    }

    /// Whether the law reads the arc nowhere along a rail: constant
    /// acceleration, or a disk rail of constant curvature. With `σ = ±0`,
    /// `σ·ds` is a zero of `σ`'s sign for every finite `ds ≥ 0`, so
    /// `κ0 + σ·ds` is one float and [`Self::accel_at`],
    /// [`Self::accel_rate_at`] and [`Self::settle_crossed_cap`] give the same
    /// bits at every arc.
    fn is_autonomous(&self) -> bool {
        match *self {
            ScalarLaw::ConstAccel { .. } => true,
            ScalarLaw::DiskRail { sigma, .. } => sigma == 0.0,
        }
    }

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

    /// The `v²` at which the curvature at arc `ds` takes the whole budget,
    /// `A/|κ|`; `None` for constant acceleration or a straight.
    fn cap_speed_squared(&self, ds: f64) -> Option<f64> {
        match *self {
            ScalarLaw::ConstAccel { .. } => None,
            ScalarLaw::DiskRail {
                accel,
                kappa0,
                sigma,
                ..
            } => {
                let kappa = (kappa0 + sigma * ds).abs();
                (kappa > 0.0).then(|| accel / kappa)
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
                if root <= CAP_HOLD_RESOLUTION * accel {
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
    fn settle_crossed_cap(&self, from: f64, to: f64, w_next: f64) -> Option<f64> {
        let ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            ..
        } = *self
        else {
            return None;
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
            return None;
        }
        Some(cap * (1.0 - follow * follow).sqrt())
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
                let (v, a, mass, full_mass) = match hi.departure {
                    CapDeparture::None => {
                        let [m0, c0, m1, c1] = self.pair_slopes(lo, hi, h);
                        (
                            quintic(lo.v, m0, c0, hi.v, m1, c1, u),
                            quintic_deriv(0.0, m0, c0, hi.rise, m1, c1, u) / h,
                            quintic_integral(lo.v, m0, c0, hi.v, m1, c1, u),
                            quintic_integral(lo.v, m0, c0, hi.v, m1, c1, 1.0),
                        )
                    }
                    CapDeparture::FromLo => {
                        let departing = self.departure_interpolant(lo, hi, h);
                        let z = u.sqrt();
                        (
                            lo.v + departing.rise(z),
                            departing.scaled_accel(z) / h,
                            lo.v * u + departing.rise_mass(z),
                            lo.v + departing.rise_mass(1.0),
                        )
                    }
                    CapDeparture::FromHi => {
                        let departing = self.departure_interpolant(lo, hi, h);
                        let z = (1.0 - u).sqrt();
                        let full_mass = hi.v + departing.rise_mass(1.0);
                        (
                            hi.v + departing.rise(z),
                            -departing.scaled_accel(z) / h,
                            full_mass - hi.v * (1.0 - u) - departing.rise_mass(z),
                            full_mass,
                        )
                    }
                };
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
                // The cubic above seeds Newton on the arc `state_at` itself
                // reports, so the time found is the one whose interpolated
                // state sits at `s`.
                let mut tau = lo.t + u * h;
                for _ in 0..3 {
                    let (s_at, v_at, _) = self.state_at(self.t0 + tau);
                    let slope = v_at.max(1e-12);
                    tau = (tau - (s_at - self.s0 - ds) / slope).clamp(lo.t, hi.t);
                }
                Some(self.t0 + tau.min(self.dt))
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

    /// The speed at the far end of a rail built by [`Self::until_arc`],
    /// exactly the speed [`Self::reach_over`] reports for the same span.
    pub(crate) fn rail_end_speed(&self) -> f64 {
        let knots = self.rail_knots();
        knots[knots.len() - 1].v
    }

    /// Whether this rail, built by [`Self::until_arc`], is the one
    /// `until_arc(0.0, 0.0, v0, law, ds)` builds: the same inputs to the bit,
    /// so the same knots.
    pub(crate) fn is_origin_rail(&self, v0: f64, law: ScalarLaw, ds: f64) -> bool {
        let Some(knots) = self.dense.get() else {
            return false;
        };
        self.t0.to_bits() == 0.0_f64.to_bits()
            && self.s0.to_bits() == 0.0_f64.to_bits()
            && self.v0.to_bits() == v0.to_bits()
            && self.law.is_bitwise(&law)
            && knots[knots.len() - 1].s.to_bits() == ds.to_bits()
    }

    /// This rail cut at the first time its speed reaches `target`, on its
    /// own interpolant, so the cut ends at `target` exactly while its interior
    /// stays the motion this segment describes. `None` when the speed never
    /// gets there.
    pub fn cut_at_speed(&self, target: f64) -> Option<LawSegment> {
        match self.law {
            ScalarLaw::ConstAccel { a0 } => {
                if self.v0 >= target {
                    return Some(LawSegment::new(self.t0, 0.0, self.s0, self.v0, self.law));
                }
                let dt = (target - self.v0) / a0;
                (a0 > 0.0 && dt <= self.dt)
                    .then(|| LawSegment::new(self.t0, dt, self.s0, self.v0, self.law))
            }
            ScalarLaw::DiskRail { .. } => {
                let knots = self.rail_knots();
                let reached = knots.iter().position(|knot| knot.v >= target)?;
                if reached == 0 {
                    return Some(self.cut_at_time(0.0));
                }
                let tau = self.solve_in_pair(reached, |state| state.1 >= target);
                Some(self.cut_at_time(tau))
            }
        }
    }

    /// This rail cut where it has covered `ds` of arc, on its own
    /// interpolant.
    pub fn cut_at_arc(&self, ds: f64) -> Option<LawSegment> {
        match self.law {
            ScalarLaw::ConstAccel { .. } => {
                let t = self.time_at_distance(self.s0 + ds)?;
                Some(LawSegment::new(
                    self.t0,
                    t - self.t0,
                    self.s0,
                    self.v0,
                    self.law,
                ))
            }
            ScalarLaw::DiskRail { .. } => {
                let knots = self.rail_knots();
                let reached = knots.iter().position(|knot| knot.s >= ds)?;
                if reached == 0 {
                    return Some(self.cut_at_time(0.0));
                }
                let tau = self.solve_in_pair(reached, |state| state.0 >= ds);
                Some(self.cut_at_time(tau))
            }
        }
    }

    /// This rail, integrated in the reversed frame from a landing, cut where
    /// it has covered `ds` of reversed arc and flipped into forward time from
    /// `(t0, s0)` under `law`: the braking segment that lands where this one
    /// started. Returns the segment and its entry speed.
    pub fn flipped_cut(
        &self,
        t0: f64,
        s0: f64,
        law: ScalarLaw,
        ds: f64,
    ) -> Option<(LawSegment, f64)> {
        if let ScalarLaw::ConstAccel { .. } = self.law {
            return Self::brake_to(t0, s0, law, ds, self.v0);
        }
        let reversed = self.cut_at_arc(ds)?;
        Some(flipped_rail(reversed.rail_knots(), t0, s0, law, ds))
    }

    /// The segment-local time inside knot pair `(reached - 1, reached)` at
    /// which `reached` first holds on the interpolated `(s, v, a)`; the pair's
    /// far knot already satisfies it.
    fn solve_in_pair(&self, reached: usize, holds: impl Fn((f64, f64, f64)) -> bool) -> f64 {
        let knots = self.rail_knots();
        let (mut t_lo, mut t_hi) = (knots[reached - 1].t, knots[reached].t);
        for _ in 0..64 {
            let mid = 0.5 * (t_lo + t_hi);
            let mut state = self.state_at(self.t0 + mid);
            state.0 -= self.s0;
            if holds(state) {
                t_hi = mid;
            } else {
                t_lo = mid;
            }
        }
        t_hi
    }

    /// This rail truncated at segment-local time `tau`: the knots before it
    /// and one knot at the interpolated state there.
    fn cut_at_time(&self, tau: f64) -> LawSegment {
        let knots = self.rail_knots();
        let (s, v, _) = self.state_at(self.t0 + tau);
        let kept = knots
            .iter()
            .position(|knot| !(knot.t < tau))
            .unwrap_or(knots.len());
        let mut cut = Vec::with_capacity(kept + 2);
        cut.extend_from_slice(&knots[..kept]);
        let (rise, departure) = match cut.last() {
            Some(lo) => {
                let hi = &knots[cut.len()];
                let h = hi.t - lo.t;
                let u = (tau - lo.t) / h;
                match hi.departure {
                    CapDeparture::None => {
                        let [m0, c0, m1, c1] = self.pair_slopes(lo, hi, h);
                        (quintic(0.0, m0, c0, hi.rise, m1, c1, u), CapDeparture::None)
                    }
                    CapDeparture::FromLo => (
                        self.departure_interpolant(lo, hi, h).rise(u.sqrt()),
                        CapDeparture::FromLo,
                    ),
                    CapDeparture::FromHi => (
                        hi.rise + self.departure_interpolant(lo, hi, h).rise((1.0 - u).sqrt()),
                        CapDeparture::None,
                    ),
                }
            }
            None => (0.0, CapDeparture::None),
        };
        cut.push(RailKnot {
            t: tau,
            s: s - self.s0,
            v,
            rise,
            departure,
        });
        if cut.len() == 1 {
            cut.insert(0, cut[0]);
        }
        let seg = LawSegment {
            t0: self.t0,
            dt: tau,
            s0: self.s0,
            v0: self.v0,
            law: self.law,
            dense: OnceLock::new(),
        };
        let _ = seg.dense.set(Arc::new(cut));
        seg
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
                Some(flipped_rail(
                    &integrate_rail_arc(&reversed, v_end, ds)?,
                    t0,
                    s0,
                    law,
                    ds,
                ))
            }
        }
    }

    /// Chain-local times at which a disk rail enters or leaves contact with
    /// its curvature cap, including either end of the segment when it starts
    /// or finishes in contact. The acceleration is not analytic there — its
    /// rate is unbounded or jumps where the rail leaves or reaches the cap —
    /// so a polynomial fit straddling one cannot hold a tolerance.
    #[must_use]
    pub fn cap_contact_times(&self) -> Vec<f64> {
        if !matches!(self.law, ScalarLaw::DiskRail { .. }) {
            return Vec::new();
        }
        let knots = self.rail_knots();
        let in_contact =
            |knot: &RailKnot| self.law.in_cap_contact(self.law.accel_at(knot.s, knot.v));
        let mut times = Vec::new();
        if knots.first().is_some_and(in_contact) {
            times.push(self.t0);
        }
        for pair in knots.windows(2) {
            if in_contact(&pair[0]) != in_contact(&pair[1]) {
                let entering = in_contact(&pair[1]);
                times.push(self.t0 + if entering { pair[1].t } else { pair[0].t });
            }
        }
        if knots.last().is_some_and(in_contact) {
            times.push(self.end_time());
        }
        times
    }

    fn rail_knots(&self) -> &Arc<Vec<RailKnot>> {
        self.dense
            .get()
            .expect("a DiskRail segment always carries its dense solution")
    }

    /// Hands a rail built by [`Self::until_arc`] back to this thread's spare
    /// knot buffers when nothing else shares its knots, so the next
    /// full-length rail integrated here writes into memory already mapped
    /// rather than faulting in a fresh mebibyte.
    pub(crate) fn recycle(self) {
        let Some(Ok(mut knots)) = self.dense.into_inner().map(Arc::try_unwrap) else {
            return;
        };
        if knots.capacity() < FULL_RAIL_KNOTS {
            return;
        }
        knots.clear();
        SPARE_FULL_RAILS.with_borrow_mut(|spare| {
            if spare.buffers.len() < SPARE_FULL_RAILS_MAX {
                spare.buffers.push(knots);
            }
        });
    }

    fn pair_slopes(&self, lo: &RailKnot, hi: &RailKnot, h: f64) -> [f64; 4] {
        let a_lo = self.law.accel_at(lo.s, lo.v);
        let a_hi = self.law.accel_at(hi.s, hi.v);
        [
            a_lo * h,
            self.law.accel_rate_at(lo.s, lo.v, a_lo) * h * h,
            a_hi * h,
            self.law.accel_rate_at(hi.s, hi.v, a_hi) * h * h,
        ]
    }

    /// The square-root interpolant of a pair that leaves the curvature cap
    /// from one end, in that end's departure frame.
    fn departure_interpolant(&self, lo: &RailKnot, hi: &RailKnot, h: f64) -> DepartureInterpolant {
        let a_lo = self.law.accel_at(lo.s, lo.v);
        let a_hi = self.law.accel_at(hi.s, hi.v);
        match hi.departure {
            CapDeparture::FromLo => DepartureInterpolant::new(
                h,
                a_lo,
                hi.rise,
                a_hi,
                self.law.accel_rate_at(hi.s, hi.v, a_hi),
            ),
            CapDeparture::FromHi => DepartureInterpolant::new(
                h,
                -a_hi,
                -hi.rise,
                -a_lo,
                self.law.accel_rate_at(lo.s, lo.v, a_lo),
            ),
            CapDeparture::None => panic!("a pair that leaves no cap has no departure interpolant"),
        }
    }

    /// The knot pairs of a disk rail that reach into segment-local time
    /// `(tau_lo, tau_hi)`, each with the formula [`Self::state_at`]
    /// evaluates between its knots; empty for a constant-acceleration
    /// segment. Pairs of zero duration hold no time and are skipped.
    pub fn rail_pairs(&self, tau_lo: f64, tau_hi: f64) -> impl Iterator<Item = RailPair> + '_ {
        let knots: &[RailKnot] = match self.law {
            ScalarLaw::ConstAccel { .. } => &[],
            ScalarLaw::DiskRail { .. } => self.rail_knots(),
        };
        let first = knots
            .partition_point(|knot| knot.t <= tau_lo)
            .saturating_sub(1);
        knots
            .windows(2)
            .skip(first)
            .take_while(move |pair| pair[0].t < tau_hi)
            .filter(|pair| pair[1].t > pair[0].t)
            .map(|pair| {
                let (lo, hi) = (&pair[0], &pair[1]);
                let h = hi.t - lo.t;
                let shape = match hi.departure {
                    CapDeparture::None => RailPairShape::Quintic {
                        rise: hi.rise,
                        slopes: self.pair_slopes(lo, hi, h),
                    },
                    departure => RailPairShape::Departure {
                        from_lo: departure == CapDeparture::FromLo,
                        coefficients: self.departure_interpolant(lo, hi, h).c,
                    },
                };
                RailPair {
                    t_lo: lo.t,
                    t_hi: hi.t,
                    s_lo: lo.s,
                    s_hi: hi.s,
                    v_lo: lo.v,
                    v_hi: hi.v,
                    shape,
                }
            })
    }
}

fn flipped_rail(rev: &[RailKnot], t0: f64, s0: f64, law: ScalarLaw, ds: f64) -> (LawSegment, f64) {
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
                rise: if i == 0 { 0.0 } else { -rev[n - i].rise },
                departure: if i == 0 {
                    CapDeparture::None
                } else {
                    rev[n - i].departure.mirrored()
                },
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
    let _ = seg.dense.set(Arc::new(knots));
    (seg, v0)
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

/// Knots of a rail the knot-count clamp caps: one mebibyte each.
const FULL_RAIL_KNOTS: usize = RAIL_KNOTS_MAX + 1;

/// Full-length knot buffers a thread keeps for reuse, at most. The reverse
/// pass recycles each member's rails as soon as its profile is built, so the
/// spares only have to bridge the members in flight, not a whole window.
const SPARE_FULL_RAILS_MAX: usize = 4;

/// A thread's recycled full-length knot buffers, and how many full-length
/// buffers it has asked for since the last [`trim_spare_full_rails`].
struct SpareFullRails {
    buffers: Vec<Vec<RailKnot>>,
    requested: usize,
}

thread_local! {
    static SPARE_FULL_RAILS: std::cell::RefCell<SpareFullRails> =
        const {
            std::cell::RefCell::new(SpareFullRails {
                buffers: Vec::new(),
                requested: 0,
            })
        };
}

/// An empty knot vector with room for `len` knots, reusing a recycled
/// full-length buffer when one is spare.
fn knot_buffer(len: usize) -> Vec<RailKnot> {
    let spare = if len == FULL_RAIL_KNOTS {
        SPARE_FULL_RAILS.with_borrow_mut(|spare| {
            spare.requested += 1;
            spare.buffers.pop()
        })
    } else {
        None
    };
    spare.unwrap_or_else(|| Vec::with_capacity(len))
}

/// Keeps no more of this thread's spare full-length buffers than the plan it
/// just finished asked for, so a plan without full-length rails leaves none
/// pinned for the next.
pub(crate) fn trim_spare_full_rails() {
    SPARE_FULL_RAILS.with_borrow_mut(|spare| {
        spare.buffers.truncate(spare.requested);
        spare.requested = 0;
    });
}

/// Dense arc-uniform solution of the rail ODE `d(v²)/ds = 2·a(s, v)` from
/// `(0, v0)` over `ds`. Each knot pair's duration is the one under which the
/// state interpolant [`LawSegment::state_at`] reads between them covers
/// exactly the pair's arc, so the knot times, the interpolated velocity and
/// the interpolated arc describe one motion. A start on the curvature cap
/// departs it across the first two steps of the graded grid: the pair after
/// it then spans at most 9/4 of its start's time from the apex, which keeps
/// the square root within a quintic's reach. `None` when the speed folds to
/// rest strictly inside the span.
fn integrate_rail_arc(law: &ScalarLaw, v0: f64, ds: f64) -> Option<Arc<Vec<RailKnot>>> {
    if ds <= 0.0 {
        let knot = RailKnot {
            t: 0.0,
            s: 0.0,
            v: v0,
            rise: 0.0,
            departure: CapDeparture::None,
        };
        return Some(Arc::new(vec![knot, knot]));
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
    let mut layout = RailLayout {
        knots: knot_buffer(n + 1),
        t: 0.0,
        w: v0 * v0,
        carried: None,
    };
    layout.knots.push(RailKnot {
        t: 0.0,
        s: 0.0,
        v: v0,
        rise: 0.0,
        departure: CapDeparture::None,
    });
    let mut first_regular = 0;
    if law.in_cap_contact(law.accel_at(0.0, v0)) {
        let h = arc_at(2);
        let w = layout.w;
        let lo = rail_state(law, 0.0, w);
        let increment = departure_increment(law, 0.0, h, w);
        let step = settle_rail_step(law, &lo, h, w, increment, n == 2, departure_knot_duration)?;
        layout.lay(h, h, step, CapDeparture::FromLo);
        first_regular = 2;
    }
    let f = |s: f64, w: f64| 2.0 * law.accel_at(s, w.max(0.0).sqrt());
    let mut memo = law.is_autonomous().then(StepMemo::new);
    for i in first_regular..n {
        let s = arc_at(i);
        let s_next = arc_at(i + 1);
        let h = s_next - s;
        let w = layout.w;
        let lo = layout
            .carried
            .take()
            .filter(|end| end.s == s)
            .unwrap_or_else(|| rail_state(law, s, w));
        let last = i == n - 1;
        let step = match memo.as_ref().and_then(|memo| memo.get(w, h)) {
            Some(step) => {
                if step.w_next <= 0.0 && !last {
                    return None;
                }
                if lo.v + step.hi.v <= 0.0 {
                    return None;
                }
                step
            }
            None => {
                let k1 = 2.0 * lo.a;
                let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
                let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
                let k4 = f(s + h, w + h * k3);
                let increment = h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
                let step = settle_rail_step(law, &lo, h, w, increment, last, knot_duration)?;
                if let Some(memo) = memo.as_mut() {
                    memo.put(w, h, step);
                }
                step
            }
        };
        if !law.in_cap_contact(lo.a) && law.in_cap_contact(step.hi.a) {
            lay_approach(law, &mut layout, &lo, (s, s_next, h, w), step, last)?;
        } else if law.in_cap_contact(lo.a) && !law.in_cap_contact(step.hi.a) {
            let increment = departure_increment(law, s, h, w);
            let departure =
                settle_rail_step(law, &lo, h, w, increment, last, departure_knot_duration)?;
            layout.lay(s_next, s + h, departure, CapDeparture::FromLo);
        } else {
            layout.lay(s_next, s + h, step, CapDeparture::None);
        }
    }
    Some(Arc::new(layout.knots))
}

/// Lays a step from `lo`, off the cap, whose end `step` reached reads in
/// contact with it. Where the solution reaches the cap inside the step the
/// step splits there: the approach onto the cap, whose acceleration falls
/// to zero as the square root of the time left — the mirror of a departure
/// — then the departure from the contact over the rest of the step. A step
/// ending in the contact band short of the cap is the approach alone.
fn lay_approach(
    law: &ScalarLaw,
    layout: &mut RailLayout,
    lo: &RailState,
    (s, s_next, h, w): (f64, f64, f64, f64),
    step: RailStep,
    last: bool,
) -> Option<()> {
    let approach_duration = |hi: &RailState, ds: f64| {
        departure_knot_duration(ds, (hi.v, -hi.a, hi.rate), (lo.v, -lo.a, lo.rate))
    };
    let Some((reach, w_contact)) = cap_contact_in_step(law, s, w, h) else {
        let duration = approach_duration(&step.hi, h);
        layout.lay(
            s_next,
            s + h,
            RailStep { duration, ..step },
            CapDeparture::FromHi,
        );
        return Some(());
    };
    let s_contact = s + reach;
    let rest = h - reach;
    let contact = rail_state(law, s_contact, w_contact);
    let approach = RailStep {
        w_next: w_contact,
        hi: contact,
        rise: contact.v - lo.v,
        duration: approach_duration(&contact, reach),
    };
    if rest <= 0.0 {
        layout.lay(s_next, s + h, approach, CapDeparture::FromHi);
        return Some(());
    }
    layout.lay(s_contact, s_contact, approach, CapDeparture::FromHi);
    let increment = departure_increment(law, s_contact, rest, w_contact);
    let departure = settle_rail_step(
        law,
        &contact,
        rest,
        w_contact,
        increment,
        last,
        departure_knot_duration,
    )?;
    layout.lay(s_next, s_contact + rest, departure, CapDeparture::FromLo);
    Some(())
}

/// Bisection steps locating a contact inside one grid step.
const CONTACT_BISECTIONS: usize = 64;

/// Where inside `(s, s + h]` the solution from `v² = w` at `s` reaches the
/// cap `v² = A/|κ|`, as the arc past `s` and the cap's `v²` there: the
/// contact whose approach, integrated back from the cap, lands on `w` at
/// `s`. `None` when the step does not reach the cap.
fn cap_contact_in_step(law: &ScalarLaw, s: f64, w: f64, h: f64) -> Option<(f64, f64)> {
    let landing = |reach: f64| -> Option<f64> {
        let cap = law.cap_speed_squared(s + reach)?;
        Some(cap + approach_increment(law, s + reach, reach, cap) - w)
    };
    let (mut short, mut short_landing) = (0.0, landing(0.0)?);
    let (mut reaches, mut reaches_landing) = (h, landing(h)?);
    if reaches_landing > 0.0 || short_landing <= 0.0 {
        return None;
    }
    let mut kept_side = 0_i8;
    for _ in 0..CONTACT_BISECTIONS {
        let secant =
            reaches - reaches_landing * (reaches - short) / (reaches_landing - short_landing);
        let middle = if secant > short && secant < reaches {
            secant
        } else {
            0.5 * (short + reaches)
        };
        if middle <= short || middle >= reaches {
            break;
        }
        let middle_landing = landing(middle)?;
        if middle_landing > 0.0 {
            (short, short_landing) = (middle, middle_landing);
            if kept_side == 1 {
                reaches_landing *= 0.5;
            }
            kept_side = 1;
        } else {
            (reaches, reaches_landing) = (middle, middle_landing);
            if kept_side == -1 {
                short_landing *= 0.5;
            }
            kept_side = -1;
            if middle_landing == 0.0 {
                break;
            }
        }
        if reaches - short <= CONTACT_ARC_RESOLUTION * (s.abs() + h) {
            break;
        }
    }
    Some((reaches, law.cap_speed_squared(s + reaches)?))
}

/// How finely a contact's arc is located, relative to the arc it sits at:
/// far below any step the rail is integrated in, and above the ulp noise
/// the landing reads with.
const CONTACT_ARC_RESOLUTION: f64 = 1e-13;

/// The change in `v²` from the cap at `s` back over `h` of arc, the mirror
/// of [`departure_increment`]: the approach reaches the cap as
/// `(s − s')^(3/2)`, smooth in `x = √((s − s')/h)`.
fn approach_increment(law: &ScalarLaw, s: f64, h: f64, w: f64) -> f64 {
    let slope = |x: f64, w: f64| -4.0 * h * x * law.accel_at(s - h * x * x, w.max(0.0).sqrt());
    let k1 = slope(0.0, w);
    let k2 = slope(0.5, w + 0.5 * k1);
    let k3 = slope(0.5, w + 0.5 * k2);
    let k4 = slope(1.0, w + k3);
    (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
}

/// The rail's knots as [`integrate_rail_arc`] lays them, with the state the
/// next step starts from.
struct RailLayout {
    knots: Vec<RailKnot>,
    t: f64,
    w: f64,
    carried: Option<RailState>,
}

impl RailLayout {
    /// Lays the knot at grid arc `s_next` that `step` reached at `s_end`,
    /// the arc its own sum lands on.
    fn lay(&mut self, s_next: f64, s_end: f64, step: RailStep, departure: CapDeparture) {
        self.t += step.duration;
        self.w = step.w_next.max(0.0);
        self.knots.push(RailKnot {
            t: self.t,
            s: s_next,
            v: step.hi.v,
            rise: step.rise,
            departure,
        });
        self.carried = Some(RailState {
            s: s_end,
            ..step.hi
        });
    }
}

fn rail_state(law: &ScalarLaw, s: f64, w: f64) -> RailState {
    let v = w.max(0.0).sqrt();
    let a = law.accel_at(s, v);
    RailState {
        s,
        v,
        a,
        rate: law.accel_rate_at(s, v, a),
    }
}

/// The step from `lo`, where `v² = w`, that raises `v²` by `increment` over
/// `h` of arc, settled onto the cap where it crosses it, and lasts the
/// `duration` its pair's interpolant takes. `None` when the speed folds to
/// rest before the rail's `last` step.
fn settle_rail_step(
    law: &ScalarLaw,
    lo: &RailState,
    h: f64,
    w: f64,
    increment: f64,
    last: bool,
    duration: impl Fn(f64, (f64, f64, f64), (f64, f64, f64)) -> f64,
) -> Option<RailStep> {
    let s = lo.s;
    let stepped = w + increment;
    let settled = law.settle_crossed_cap(s, s + h, stepped);
    let w_next = settled.unwrap_or(stepped);
    if w_next <= 0.0 && !last {
        return None;
    }
    let hi = rail_state(law, s + h, w_next);
    if lo.v + hi.v <= 0.0 {
        return None;
    }
    let rise = match settled {
        None if w_next > 0.0 => increment / (lo.v + hi.v),
        _ => hi.v - lo.v,
    };
    Some(RailStep {
        w_next,
        hi,
        rise,
        duration: duration(h, (lo.v, lo.a, lo.rate), (hi.v, hi.a, hi.rate)),
    })
}

/// One completed integrator step: the state it reached and the knot it laid.
#[derive(Clone, Copy)]
struct RailStep {
    w_next: f64,
    hi: RailState,
    rise: f64,
    duration: f64,
}

const STEP_MEMO_SLOTS: usize = 64;

/// Completed regular steps of an autonomous rail by the `(v², h)` they
/// started from. Such a law reads the arc nowhere, so a regular step is a
/// function of its start speed and size alone, and a rail riding its cap
/// repeats a handful of `(v², h)` pairs — the grid's step sizes differ only
/// in rounding — for thousands of steps. Direct-mapped on the step size; a
/// collision only costs a recomputation. A rail's departure from the cap is
/// a different step from the same `(v², h)`, so it neither reads nor fills
/// the memo.
struct StepMemo {
    slots: [Option<(u64, u64, RailStep)>; STEP_MEMO_SLOTS],
}

impl StepMemo {
    fn new() -> Self {
        StepMemo {
            slots: [None; STEP_MEMO_SLOTS],
        }
    }

    fn slot(h: f64) -> usize {
        (h.to_bits().wrapping_mul(0x9E37_79B9_7F4A_7C15) >> (64 - STEP_MEMO_SLOTS.trailing_zeros()))
            as usize
    }

    fn get(&self, w: f64, h: f64) -> Option<RailStep> {
        match self.slots[Self::slot(h)] {
            Some((w_bits, h_bits, step)) if w_bits == w.to_bits() && h_bits == h.to_bits() => {
                Some(step)
            }
            _ => None,
        }
    }

    fn put(&mut self, w: f64, h: f64, step: RailStep) {
        self.slots[Self::slot(h)] = Some((w.to_bits(), h.to_bits(), step));
    }
}

/// The rise in `v²` over `[s, s + h]` departing the cap at `s`. There `v²`
/// leaves as `(s'−s)^(3/2)`, whose unbounded curvature a step in arc
/// misses by a fixed fraction however short; one Runge–Kutta step in
/// `x = √((s'−s)/h)` instead, where the slope `d(v²)/dx = 4h·x·a` is smooth,
/// holds the launch and landing tests to the law as closely as eight do.
fn departure_increment(law: &ScalarLaw, s: f64, h: f64, w: f64) -> f64 {
    let slope = |x: f64, w: f64| 4.0 * h * x * law.accel_at(s + h * x * x, w.max(0.0).sqrt());
    let k1 = slope(0.0, w);
    let k2 = slope(0.5, w + 0.5 * k1);
    let k3 = slope(0.5, w + 0.5 * k2);
    let k4 = slope(1.0, w + k3);
    (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
}

/// The rail ODE's state at one arc position, with the law's acceleration
/// and its rate there.
#[derive(Clone, Copy)]
struct RailState {
    s: f64,
    v: f64,
    a: f64,
    rate: f64,
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
    pair_duration(
        ds,
        (
            0.5 * (v_lo + v_hi),
            (a_lo - a_hi) / 10.0,
            (r_lo + r_hi) / 120.0,
        ),
        (v_lo, v_hi),
    )
}

/// The duration `τ` under which the square-root interpolant departing the
/// cap from `lo`, whose rate the square root leaves unbounded, integrates to
/// exactly `ds`: `ds = τ·(2v_lo+5v_hi)/7 + τ²·(a_lo/70 − 8a_hi/35) + τ³·4ȧ_hi/105`.
fn departure_knot_duration(
    ds: f64,
    (v_lo, a_lo, _): (f64, f64, f64),
    (v_hi, a_hi, r_hi): (f64, f64, f64),
) -> f64 {
    pair_duration(
        ds,
        (
            (2.0 * v_lo + 5.0 * v_hi) / 7.0,
            a_lo / 70.0 - 8.0 * a_hi / 35.0,
            4.0 * r_hi / 105.0,
        ),
        (v_lo, v_hi),
    )
}

/// The positive root of `c3·τ³ + c2·τ² + c1·τ = ds`, by Newton's method from
/// the leading term's root.
fn pair_duration(ds: f64, (c1, c2, c3): (f64, f64, f64), (v_lo, v_hi): (f64, f64)) -> f64 {
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
