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
//! `ConstAccel` is closed-form. `DiskRail` is evaluated through a
//! Runge–Kutta solution built deterministically from the law parameters and
//! cached inside the segment; between knots a C2 quintic interpolates
//! velocity, arc is its anchored integral, and acceleration is its
//! derivative taken across each knot pair's speed rise as the integrator
//! produced it, not as the difference of two rounded knot speeds. A pair
//! that leaves the curvature cap from one end interpolates in the square
//! root of the time from that end instead: see [`CapDeparture`].
//!
//! # Accuracy contract
//!
//! The rail is the motion its interpolant describes, and that motion is
//! what every later stage plans, shapes and fits against. The integrator
//! places its knots so each pair provably holds two things — bounded by
//! Taylor models of its interpolant: its acceleration stays within
//! [`RAIL_ACCELERATION_DEFECT`] of the law `a(s, v)` read at its own
//! interpolated state — so its acceleration budget and its disk hold to
//! that — and its arc stays within [`RAIL_ARC_DEFECT`] of the integral of
//! its own speed, so arc, speed and time describe one consistent motion.
//! Both are a tenth of the default fit tolerances. Nothing ties the rail
//! to the exact solution of the ODE: that solution is no physical
//! reference, only one more motion along the same path, and the rail's
//! timing may drift from it by the defect integrated over the rail's
//! duration while the path, the limits and the fit tolerances all hold.

use std::sync::{Arc, OnceLock};

mod defect;
mod variation;

pub use defect::{PairDefect, RAIL_ACCELERATION_DEFECT, RAIL_ARC_DEFECT, RAIL_SPEED_SLACK};

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
    slopes: KnotSlopes,
}

/// Where a knot's acceleration and acceleration rate come from.
#[derive(Clone, Copy, Debug, PartialEq)]
enum KnotSlopes {
    /// The law at the knot's state: every knot the integrator lays.
    Law,
    /// The interpolant of the pair a cut ended inside. The cut keeps that
    /// pair's motion, which leaves the law where the pair straddles the
    /// curvature cap: re-anchored to the law at the cut, the shortened
    /// pair's rise and slopes disagree and its quintic swings between them.
    Interpolant { accel: f64, rate: f64 },
}

impl KnotSlopes {
    fn mirrored(self) -> Self {
        match self {
            Self::Law => Self::Law,
            Self::Interpolant { accel, rate } => Self::Interpolant {
                accel: -accel,
                rate,
            },
        }
    }
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
    Clipped(ClippedDeparture),
}

impl CapDeparture {
    fn mirrored(self) -> Self {
        match self {
            Self::None => Self::None,
            Self::FromLo => Self::FromHi,
            Self::FromHi => Self::FromLo,
            Self::Clipped(clipped) => Self::Clipped(ClippedDeparture {
                theta_lo: clipped.theta_hi,
                theta_hi: clipped.theta_lo,
                ..clipped
            }),
        }
    }
}

/// The part of a departure pair a cut kept clear of its departure end: the
/// parent pair's square-root interpolant `v_end + Σ cₖ·zᵏ`,
/// `z = √(θ/h)`, over the departure-frame times `theta_lo..theta_hi` the
/// kept pair's ends sit at. Both stay off the departure end, so the kept
/// pair is analytic in its own time.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ClippedDeparture {
    coefficients: [f64; 4],
    h: f64,
    v_end: f64,
    theta_lo: f64,
    theta_hi: f64,
}

impl ClippedDeparture {
    fn interpolant(&self) -> DepartureInterpolant {
        DepartureInterpolant {
            c: self.coefficients,
        }
    }

    /// `+1` where the departure-frame time runs with the pair's own.
    fn direction(&self) -> f64 {
        (self.theta_hi - self.theta_lo).signum()
    }

    fn theta_at(&self, u: f64) -> f64 {
        self.theta_lo + (self.theta_hi - self.theta_lo) * u
    }

    fn z_at(&self, theta: f64) -> f64 {
        (theta / self.h).sqrt()
    }

    /// `∫ v dθ` from the departure end out to `theta`.
    fn mass_from_end(&self, theta: f64) -> f64 {
        self.v_end * theta + self.h * self.interpolant().rise_mass(self.z_at(theta))
    }

    /// Speed, acceleration and `∫₀ᵘ v du` at the pair's time share `u`.
    fn state(&self, u: f64) -> (f64, f64, f64) {
        let theta = self.theta_at(u);
        let z = self.z_at(theta);
        let interpolant = self.interpolant();
        (
            self.v_end + interpolant.rise(z),
            self.direction() * interpolant.scaled_accel(z) / self.h,
            (self.mass_from_end(theta) - self.mass_from_end(self.theta_lo))
                / (self.theta_hi - self.theta_lo),
        )
    }

    /// This clipped pair cut at its time share `u`.
    fn cut(&self, u: f64) -> Self {
        Self {
            theta_hi: self.theta_at(u),
            ..*self
        }
    }
}

/// The cut knot ending `clipped` after `lo`, at speed `v`: its rise, the
/// pair's formula and the interpolant's slopes there.
fn clipped_cut(
    clipped: &ClippedDeparture,
    lo: &RailKnot,
    v: f64,
) -> (f64, CapDeparture, KnotSlopes) {
    let z = clipped.z_at(clipped.theta_hi);
    let interpolant = clipped.interpolant();
    (
        v - lo.v,
        CapDeparture::Clipped(*clipped),
        KnotSlopes::Interpolant {
            accel: clipped.direction() * interpolant.scaled_accel(z) / clipped.h,
            rate: interpolant.scaled_rate(z) / (clipped.h * clipped.h),
        },
    )
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

    /// Acceleration rate times `h²`, away from the departure end: unbounded
    /// at it.
    fn scaled_rate(&self, z: f64) -> f64 {
        let [_, c3, c4, c5] = self.c;
        (1.5 * c3 + z * (4.0 * c4 + z * 7.5 * c5)) / (2.0 * z)
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
    /// The part of a departure pair a cut kept clear of its departure end:
    /// at departure-frame time `θ`, running linearly from `theta_lo` at
    /// `t_lo` to `theta_hi` at `t_hi`, speed is `v_end` plus the
    /// departure's `z²·(c₂ + …)` at `z = √(θ/h)`, and `h·a` its
    /// `c₂ + z·(1.5c₃ + …)` signed by the direction `θ` runs in.
    Clipped {
        coefficients: [f64; 4],
        h: f64,
        v_end: f64,
        theta_lo: f64,
        theta_hi: f64,
    },
}

/// One law over one time interval, in chain-local time and run-frame arc.
#[derive(Clone)]
pub struct LawSegment {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub law: ScalarLaw,
    dense: OnceLock<Arc<Vec<RailKnot>>>,
    /// Each dense knot's acceleration and its rate under the law, read
    /// once for every pair [`Self::state_at`] interpolates.
    knot_slopes: OnceLock<Arc<[(f64, f64)]>>,
}

impl std::fmt::Debug for LawSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LawSegment")
            .field("t0", &self.t0)
            .field("dt", &self.dt)
            .field("s0", &self.s0)
            .field("v0", &self.v0)
            .field("law", &self.law)
            .field("dense", &self.dense)
            .finish()
    }
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

    /// Whether tangential acceleration `a` is a held cap's: the rail rides
    /// its curvature cap.
    fn holds_cap(&self, a: f64) -> bool {
        match *self {
            ScalarLaw::ConstAccel { .. } => false,
            ScalarLaw::DiskRail { accel, .. } => a.abs() <= CAP_HOLD_RESOLUTION * accel,
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

    /// The `v²` of the curve [`Self::settle_crossed_cap`] settles a rail on
    /// at arc `ds`, where its cap rises; `None` elsewhere.
    fn settled_speed_squared(&self, ds: f64) -> Option<f64> {
        let (settled, _) = self.settled_slopes(ds, 0.0)?;
        let ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            ..
        } = *self
        else {
            return None;
        };
        let kappa = (kappa0 + sigma * ds).abs();
        let follow = 0.5 * sigma / (kappa * kappa);
        (settled > 0.0).then(|| accel / kappa * (1.0 - follow * follow).sqrt())
    }

    /// The acceleration and its rate along the curve
    /// [`Self::settle_crossed_cap`] settles a rail on at arc `ds`, speed `v`:
    /// `w(s) = A·√(k⁻² − c·k⁻⁶)`, `k = |κ(s)|`, `c = σ²/4`, read as
    /// `a = ½·w'` and `ȧ = v·½·w''`; `None` where no such curve exists — a
    /// falling cap, a straight, or one rising faster than a rail can climb.
    fn settled_slopes(&self, ds: f64, v: f64) -> Option<(f64, f64)> {
        let ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            ..
        } = *self
        else {
            return None;
        };
        let signed = kappa0 + sigma * ds;
        let k = signed.abs();
        let dk = sigma * signed.signum();
        let c = 0.25 * sigma * sigma;
        if k == 0.0 || dk > 0.0 || c >= k.powi(4) {
            return None;
        }
        let u = k.powi(-2) - c * k.powi(-6);
        let du = -2.0 * k.powi(-3) + 6.0 * c * k.powi(-7);
        let ddu = 6.0 * k.powi(-4) - 42.0 * c * k.powi(-8);
        let root = u.sqrt();
        let dw = accel * du / (2.0 * root) * dk;
        let ddw = accel * (ddu / (2.0 * root) - du * du / (4.0 * u * root)) * dk * dk;
        Some((0.5 * dw, v * 0.5 * ddw))
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
            knot_slopes: OnceLock::new(),
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
                        let [m0, c0, m1, c1] = self.cached_pair_slopes(idx, h);
                        (
                            quintic(lo.v, m0, c0, hi.v, m1, c1, u),
                            quintic_deriv(0.0, m0, c0, hi.rise, m1, c1, u) / h,
                            quintic_integral(lo.v, m0, c0, hi.v, m1, c1, u),
                            quintic_integral(lo.v, m0, c0, hi.v, m1, c1, 1.0),
                        )
                    }
                    CapDeparture::FromLo => {
                        let departing = self.law.departure_interpolant(lo, hi, h);
                        let z = u.sqrt();
                        (
                            lo.v + departing.rise(z),
                            departing.scaled_accel(z) / h,
                            lo.v * u + departing.rise_mass(z),
                            lo.v + departing.rise_mass(1.0),
                        )
                    }
                    CapDeparture::FromHi => {
                        let departing = self.law.departure_interpolant(lo, hi, h);
                        let z = (1.0 - u).sqrt();
                        let full_mass = hi.v + departing.rise_mass(1.0);
                        (
                            hi.v + departing.rise(z),
                            -departing.scaled_accel(z) / h,
                            full_mass - hi.v * (1.0 - u) - departing.rise_mass(z),
                            full_mass,
                        )
                    }
                    CapDeparture::Clipped(clipped) => {
                        let (v, a, mass) = clipped.state(u);
                        (v, a, mass, clipped.state(1.0).2)
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
                if ds >= end {
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
                const ARC_INVERSION_STEPS: usize = 16;
                let mut tau = lo.t + u * h;
                for _ in 0..ARC_INVERSION_STEPS {
                    let (s_at, v_at, _) = self.state_at(self.t0 + tau);
                    let slope = v_at.max(1e-12);
                    let next = (tau - (s_at - self.s0 - ds) / slope).clamp(lo.t, hi.t);
                    let settled = (next - tau).abs() <= 4.0 * f64::EPSILON * tau.abs();
                    tau = next;
                    if settled {
                        break;
                    }
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
                    knot_slopes: OnceLock::new(),
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
    /// and one knot at the interpolated state there, carrying the
    /// interpolant's acceleration and rate, so the stretch it keeps is the
    /// motion this rail describes there.
    fn cut_at_time(&self, tau: f64) -> LawSegment {
        let knots = self.rail_knots();
        let (s, v, _) = self.state_at(self.t0 + tau);
        let kept = knots
            .iter()
            .position(|knot| !(knot.t < tau))
            .unwrap_or(knots.len());
        let mut cut = Vec::with_capacity(kept + 2);
        cut.extend_from_slice(&knots[..kept]);
        let (rise, departure, slopes) = match cut.last() {
            Some(lo) => {
                let hi = &knots[cut.len()];
                let h = hi.t - lo.t;
                let u = (tau - lo.t) / h;
                match hi.departure {
                    CapDeparture::None => {
                        let [m0, c0, m1, c1] = self.law.pair_slopes(lo, hi, h);
                        (
                            quintic(0.0, m0, c0, hi.rise, m1, c1, u),
                            CapDeparture::None,
                            KnotSlopes::Interpolant {
                                accel: quintic_deriv(0.0, m0, c0, hi.rise, m1, c1, u) / h,
                                rate: quintic_second_deriv(0.0, m0, c0, hi.rise, m1, c1, u)
                                    / (h * h),
                            },
                        )
                    }
                    CapDeparture::FromLo => {
                        let departing = self.law.departure_interpolant(lo, hi, h);
                        let z = u.sqrt();
                        (
                            departing.rise(z),
                            CapDeparture::FromLo,
                            KnotSlopes::Interpolant {
                                accel: departing.scaled_accel(z) / h,
                                rate: departing.scaled_rate(z) / (h * h),
                            },
                        )
                    }
                    CapDeparture::FromHi => {
                        let departing = self.law.departure_interpolant(lo, hi, h);
                        let clipped = ClippedDeparture {
                            coefficients: departing.c,
                            h,
                            v_end: hi.v,
                            theta_lo: h,
                            theta_hi: h * (1.0 - u),
                        };
                        clipped_cut(&clipped, lo, v)
                    }
                    CapDeparture::Clipped(parent) => clipped_cut(&parent.cut(u), lo, v),
                }
            }
            None => (0.0, CapDeparture::None, KnotSlopes::Law),
        };
        cut.push(RailKnot {
            t: tau,
            s: s - self.s0,
            v,
            rise,
            departure,
            slopes,
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
            knot_slopes: OnceLock::new(),
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

    /// [`ScalarLaw::pair_slopes`] of the dense pair from knot `lo` to the
    /// next, from the knots' slopes read once per segment.
    fn cached_pair_slopes(&self, lo: usize, h: f64) -> [f64; 4] {
        let slopes = self.knot_slopes.get_or_init(|| {
            self.rail_knots()
                .iter()
                .map(|knot| self.law.knot_slopes(knot))
                .collect()
        });
        let ((a_lo, rate_lo), (a_hi, rate_hi)) = (slopes[lo], slopes[lo + 1]);
        [a_lo * h, rate_lo * h * h, a_hi * h, rate_hi * h * h]
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
            .map(|pair| self.law.rail_pair(&pair[0], &pair[1]))
    }

    /// An upper bound on `|a|` of the rail's interpolated motion across
    /// every knot pair reaching into segment-local time `(tau_lo, tau_hi)`:
    /// the motion every later stage reads, which strays from the law's
    /// own acceleration by up to its proven defect; zero for a
    /// constant-acceleration segment.
    pub fn interpolated_acceleration_bound(&self, tau_lo: f64, tau_hi: f64) -> f64 {
        self.rail_pairs(tau_lo, tau_hi)
            .map(|pair| defect::acceleration_bound(&pair))
            .fold(0.0, f64::max)
    }

    /// The proven defect of every knot pair of this rail against its law.
    pub fn rail_defects(&self) -> impl Iterator<Item = PairDefect> + '_ {
        self.rail_pairs(0.0, self.dt)
            .map(|pair| defect::pair_defect(&self.law, &pair))
    }
}

impl ScalarLaw {
    /// The knot's acceleration and its rate.
    fn knot_slopes(&self, knot: &RailKnot) -> (f64, f64) {
        match knot.slopes {
            KnotSlopes::Law => {
                let accel = self.accel_at(knot.s, knot.v);
                (accel, self.accel_rate_at(knot.s, knot.v, accel))
            }
            KnotSlopes::Interpolant { accel, rate } => (accel, rate),
        }
    }

    fn pair_slopes(&self, lo: &RailKnot, hi: &RailKnot, h: f64) -> [f64; 4] {
        let (a_lo, rate_lo) = self.knot_slopes(lo);
        let (a_hi, rate_hi) = self.knot_slopes(hi);
        [a_lo * h, rate_lo * h * h, a_hi * h, rate_hi * h * h]
    }

    /// The square-root interpolant of a pair that leaves the curvature cap
    /// from one end, in that end's departure frame.
    fn departure_interpolant(&self, lo: &RailKnot, hi: &RailKnot, h: f64) -> DepartureInterpolant {
        let (a_lo, rate_lo) = self.knot_slopes(lo);
        let (a_hi, rate_hi) = self.knot_slopes(hi);
        match hi.departure {
            CapDeparture::FromLo => DepartureInterpolant::new(h, a_lo, hi.rise, a_hi, rate_hi),
            CapDeparture::FromHi => DepartureInterpolant::new(h, -a_hi, -hi.rise, -a_lo, rate_lo),
            CapDeparture::None => panic!("a pair that leaves no cap has no departure interpolant"),
            CapDeparture::Clipped(_) => {
                panic!("a clipped pair carries its parent's departure interpolant")
            }
        }
    }

    /// The formula [`LawSegment::state_at`] evaluates between `lo` and `hi`.
    fn rail_pair(&self, lo: &RailKnot, hi: &RailKnot) -> RailPair {
        let h = hi.t - lo.t;
        let shape = match hi.departure {
            CapDeparture::None => RailPairShape::Quintic {
                rise: hi.rise,
                slopes: self.pair_slopes(lo, hi, h),
            },
            CapDeparture::Clipped(clipped) => RailPairShape::Clipped {
                coefficients: clipped.coefficients,
                h: clipped.h,
                v_end: clipped.v_end,
                theta_lo: clipped.theta_lo,
                theta_hi: clipped.theta_hi,
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
                slopes: k.slopes.mirrored(),
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
        knot_slopes: OnceLock::new(),
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

fn quintic_second_deriv(p0: f64, m0: f64, c0: f64, p1: f64, m1: f64, c1: f64, u: f64) -> f64 {
    let (u2, u3) = (u * u, u * u * u);
    (p1 - p0) * (60.0 * u - 180.0 * u2 + 120.0 * u3)
        + m0 * (-36.0 * u + 96.0 * u2 - 60.0 * u3)
        + c0 * (1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3)
        + m1 * (-24.0 * u + 84.0 * u2 - 60.0 * u3)
        + c1 * (3.0 * u - 12.0 * u2 + 10.0 * u3)
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

/// The share of a rail's arc its first step tries.
const RAIL_FIRST_STEP_SHARE: f64 = 1.0 / 16.0;

/// How far one step's size may grow toward the next, and shrink on a retry.
const RAIL_STEP_GROWTH_MAX: f64 = 4.0;
const RAIL_STEP_SHRINK_MAX: f64 = 0.125;

/// The share of its allowance a step's size aims the worse defect at.
const RAIL_STEP_DEFECT_TARGET: f64 = 0.5;

/// The order in the step size of a knot pair's acceleration defect.
const RAIL_DEFECT_ORDER: f64 = 5.0;

/// The shortest step, as a share of the rail's arc, a step is shrunk to.
const RAIL_STEP_SHARE_MIN: f64 = 1e-12;

/// Knots a rail may need before it fails loudly.
const RAIL_KNOTS_MAX: usize = 1 << 16;

/// How far above its curvature cap a rail may start, in `v²`: rounding of
/// a speed read off the cap.
pub(super) const CAP_START_SLACK: f64 = 1e-9;

/// The size the next step tries after one of `size` whose worse defect was
/// `excess` times its allowance.
fn next_step_size(size: f64, excess: f64) -> f64 {
    let factor = libm::pow(RAIL_STEP_DEFECT_TARGET / excess, 1.0 / RAIL_DEFECT_ORDER);
    size * factor.clamp(RAIL_STEP_SHRINK_MAX, RAIL_STEP_GROWTH_MAX)
}

/// Solution of the rail ODE `d(v²)/ds = 2·a(s, v)` from `(0, v0)` over
/// `ds`, its steps as long as their knot pairs' proven defects against the
/// law ([`defect`]) stay within [`RAIL_ACCELERATION_DEFECT`] and
/// [`RAIL_ARC_DEFECT`]: each step retried shorter while its pairs miss,
/// the next sized from the last's defect. Each knot pair's duration is the
/// one under which the state interpolant [`LawSegment::state_at`] reads
/// between them covers exactly the pair's arc, so the knot times, the
/// interpolated velocity and the interpolated arc describe one motion. A
/// start on the curvature cap departs it in the first step. `None` when
/// the speed folds to rest strictly inside the span.
fn integrate_rail_arc(law: &ScalarLaw, v0: f64, ds: f64) -> Option<Arc<Vec<RailKnot>>> {
    let ScalarLaw::DiskRail {
        accel,
        kappa0,
        sigma,
        brake,
    } = *law
    else {
        unreachable!("only a disk rail is integrated knot by knot")
    };
    let key = [
        v0.to_bits(),
        ds.to_bits(),
        accel.to_bits(),
        kappa0.to_bits(),
        sigma.to_bits(),
        u64::from(brake),
    ];
    let recent = || RECENT_RAILS.lock().expect("a rail integration panicked");
    if let Some(Rail(knots)) = recent().read(&key) {
        return knots;
    }
    let knots = integrate_rail_arc_afresh(law, v0, ds);
    recent().keep(key, Rail(knots.clone()));
    knots
}

/// The rails integrated lately, by their inputs to the bit: a velocity plan
/// settles the same members from the same speeds again as moves join the
/// window it plans, and the shaper's workers read the same rails again, so
/// every thread reads the one cache. The integration is a pure function of
/// the key and runs outside the lock: two threads integrating one key at
/// once keep the same rail. Two generations of [`RECENT_RAILS_GENERATION`]
/// entries each: the older is dropped when the newer fills.
#[derive(Default)]
struct RecentRails {
    newer: std::collections::HashMap<[u64; 6], Rail>,
    older: std::collections::HashMap<[u64; 6], Rail>,
}

/// One integrated rail: its knots, or `None` where its speed folds to rest.
#[derive(Clone)]
struct Rail(Option<Arc<Vec<RailKnot>>>);

const RECENT_RAILS_GENERATION: usize = 4096;

impl RecentRails {
    fn read(&self, key: &[u64; 6]) -> Option<Rail> {
        self.newer.get(key).or_else(|| self.older.get(key)).cloned()
    }

    fn keep(&mut self, key: [u64; 6], knots: Rail) {
        if self.newer.len() == RECENT_RAILS_GENERATION {
            self.older = std::mem::take(&mut self.newer);
        }
        self.newer.insert(key, knots);
    }
}

static RECENT_RAILS: std::sync::LazyLock<std::sync::Mutex<RecentRails>> =
    std::sync::LazyLock::new(std::sync::Mutex::default);

fn integrate_rail_arc_afresh(law: &ScalarLaw, v0: f64, ds: f64) -> Option<Arc<Vec<RailKnot>>> {
    let start = RailKnot {
        t: 0.0,
        s: 0.0,
        v: v0,
        rise: 0.0,
        departure: CapDeparture::None,
        slopes: KnotSlopes::Law,
    };
    if ds <= 0.0 {
        return Some(Arc::new(vec![start, start]));
    }
    if let Some(cap) = law.cap_speed_squared(0.0) {
        assert!(
            v0 * v0 <= cap * (1.0 + CAP_START_SLACK),
            "rail {law:?} starts at v={v0}, above its curvature cap v={}",
            cap.sqrt()
        );
    }
    let mut layout = RailLayout {
        knots: vec![start],
        t: 0.0,
        w: v0 * v0,
        carried: None,
    };
    let mut s = 0.0;
    let mut size = ds * RAIL_FIRST_STEP_SHARE;
    while s < ds {
        let remaining = ds - s;
        let last = size >= remaining;
        let h = if last {
            remaining
        } else if 2.0 * size > remaining {
            0.5 * remaining
        } else {
            size
        };
        let s_next = if last { ds } else { s + h };
        let w = layout.w;
        let lo = layout
            .carried
            .filter(|end| end.s == s)
            .unwrap_or_else(|| rail_state(law, s, w));
        let shortest = h <= ds * RAIL_STEP_SHARE_MIN;
        let Some((first, second)) = rail_step(law, &lo, (s, s_next, h, w), last) else {
            if shortest {
                return None;
            }
            size = h * RAIL_STEP_SHRINK_MAX;
            continue;
        };
        let laid: &[Laid] = match &second {
            Some(second) => &[first, *second],
            None => &[first],
        };
        let excess = layout.excess(law, laid);
        size = next_step_size(h, excess);
        if excess > 1.0 {
            assert!(
                !shortest,
                "rail {law:?} from v={v0} over {ds} mm: a step of {h} mm at {s} mm still misses its law by {excess}× its allowance"
            );
            continue;
        }
        for laid in laid {
            layout.lay(laid);
        }
        assert!(
            layout.knots.len() <= RAIL_KNOTS_MAX,
            "rail {law:?} from v={v0} over {ds} mm needs more than {RAIL_KNOTS_MAX} knots"
        );
        s = s_next;
    }
    Some(Arc::new(layout.knots))
}

/// The knots one step of `h` from `lo` at arc `s`, where `v² = w`, lays up
/// to grid arc `s_next`: a start on the cap departs it, a step running onto
/// the cap approaches it ([`approach`]), one leaving it departs, any other
/// is one regular Runge–Kutta step. `None` when the speed folds to rest
/// before the rail's `last` step.
fn rail_step(
    law: &ScalarLaw,
    lo: &RailState,
    (s, s_next, h, w): (f64, f64, f64, f64),
    last: bool,
) -> Option<(Laid, Option<Laid>)> {
    let laid = |step, departure| Laid {
        s_next,
        s_end: s + h,
        step,
        departure,
    };
    let departure = || {
        let increment = departure_increment(law, s, h, w);
        settle_rail_step(law, lo, h, w, increment, last, departure_knot_duration)
    };
    if s == 0.0 && law.in_cap_contact(lo.a) {
        return Some((laid(departure()?, CapDeparture::FromLo), None));
    }
    if let Some(step) = settled_step(law, lo, h) {
        return Some((laid(step, CapDeparture::None), None));
    }
    let f = |s: f64, w: f64| 2.0 * law.accel_at(s, w.max(0.0).sqrt());
    let k1 = 2.0 * lo.a;
    let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
    let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
    let k4 = f(s + h, w + h * k3);
    let increment = h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    let step = settle_rail_step(law, lo, h, w, increment, last, knot_duration)?;
    let approaching =
        law.in_cap_contact(step.hi.a) && !law.holds_cap(lo.a) && step.hi.a.abs() < lo.a.abs();
    if approaching {
        approach(law, lo, (s, s_next, h, w), step, last)
    } else if law.holds_cap(lo.a) && !law.holds_cap(step.hi.a) {
        Some((laid(departure()?, CapDeparture::FromLo), None))
    } else {
        Some((laid(step, CapDeparture::None), None))
    }
}

/// The share of [`RAIL_ACCELERATION_DEFECT`] within which the curve a
/// rising cap draws must follow the law for a rail to ride it: the curve
/// is the law's own to `O(f²)`, `f = σ/2κ²`, which only a stiff cap makes
/// this close, and a gentle one the integrator steps across stably anyway.
const SETTLED_CURVE_SHARE: f64 = 1.0 / 64.0;

/// The step from `lo` along the curve a rising cap draws the rail on, where
/// `lo` rides it and the curve follows the law: at both ends the law's
/// acceleration within [`SETTLED_CURVE_SHARE`] of the curve's, and `lo`'s
/// speed close enough that closing the gap over the step adds at most half
/// [`RAIL_ACCELERATION_DEFECT`]. The solution is pulled onto the curve within
/// `1/λ` of arc, `λ = ∂(2a)/∂(v²)` its stiffness there, far below any step
/// an explicit integrator is stable at across it, so the rail rides the
/// curve itself and the defect check holds its knots to the law. `None`
/// off such a curve.
fn settled_step(law: &ScalarLaw, lo: &RailState, h: f64) -> Option<RailStep> {
    let follows = |s: f64, v: f64, settled: f64| {
        settled > 0.0
            && (law.accel_at(s, v) - settled).abs()
                <= SETTLED_CURVE_SHARE * RAIL_ACCELERATION_DEFECT
    };
    let (settled, _) = law.settled_slopes(lo.s, lo.v)?;
    let on_curve = law.settled_speed_squared(lo.s)?.sqrt();
    let gap = (lo.v - on_curve).abs();
    if !(follows(lo.s, on_curve, settled)
        && gap * (lo.v + on_curve) <= RAIL_ACCELERATION_DEFECT * h)
    {
        return None;
    }
    let w_next = law.settled_speed_squared(lo.s + h)?;
    let v = w_next.sqrt();
    let (settled, rate) = law.settled_slopes(lo.s + h, v)?;
    if !follows(lo.s + h, v, settled) {
        return None;
    }
    let a = law.accel_at(lo.s + h, v);
    let hi = RailState {
        s: lo.s + h,
        v,
        a,
        rate,
        slopes: KnotSlopes::Interpolant { accel: a, rate },
    };
    Some(RailStep {
        w_next,
        hi,
        rise: v - lo.v,
        duration: knot_duration(h, (lo.v, lo.a, lo.rate), (hi.v, hi.a, hi.rate)),
    })
}

/// The knots of a step from `lo`, short of the cap, whose end `step`
/// reached reads in contact with it. Where the solution reaches the cap
/// inside the step the step splits there: the approach onto the cap, whose
/// acceleration falls to zero as the square root of the time left — the
/// mirror of a departure — then the departure from the contact over the
/// rest of the step. A step ending short of the cap is a regular one: the
/// steps shorten toward the contact until one reaches it.
fn approach(
    law: &ScalarLaw,
    lo: &RailState,
    (s, s_next, h, w): (f64, f64, f64, f64),
    step: RailStep,
    last: bool,
) -> Option<(Laid, Option<Laid>)> {
    let approach_duration = |hi: &RailState, ds: f64| {
        departure_knot_duration(ds, (hi.v, -hi.a, hi.rate), (lo.v, -lo.a, lo.rate))
    };
    let Some((reach, w_contact)) = cap_contact_in_step(law, s, w, h) else {
        let laid = Laid {
            s_next,
            s_end: s + h,
            step,
            departure: CapDeparture::None,
        };
        return Some((laid, None));
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
    if approach.duration <= speed_noise_duration(lo.v) {
        let increment = departure_increment(law, s, h, w);
        let departure = settle_rail_step(law, lo, h, w, increment, last, departure_knot_duration)?;
        let laid = Laid {
            s_next,
            s_end: s + h,
            step: departure,
            departure: CapDeparture::FromLo,
        };
        return Some((laid, None));
    }
    if rest <= 0.0 {
        let laid = Laid {
            s_next,
            s_end: s + h,
            step: approach,
            departure: CapDeparture::FromHi,
        };
        return Some((laid, None));
    }
    let onto = Laid {
        s_next: s_contact,
        s_end: s_contact,
        step: approach,
        departure: CapDeparture::FromHi,
    };
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
    let off = Laid {
        s_next,
        s_end: s_contact + rest,
        step: departure,
        departure: CapDeparture::FromLo,
    };
    Some((onto, Some(off)))
}

/// The shortest pair from speed `v` whose acceleration its speeds' rounding
/// leaves within [`RAIL_ACCELERATION_DEFECT`]: a contact closer than this
/// is the step's own start.
fn speed_noise_duration(v: f64) -> f64 {
    defect::SPEED_NOISE_ULPS * f64::EPSILON * v.abs() / RAIL_ACCELERATION_DEFECT
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

/// A knot a step reached: at arc `s_next`, by `step` landing on `s_end`,
/// the arc its own sum lands on.
#[derive(Clone, Copy)]
struct Laid {
    s_next: f64,
    s_end: f64,
    step: RailStep,
    departure: CapDeparture,
}

impl Laid {
    fn knot(&self, t_lo: f64) -> RailKnot {
        RailKnot {
            t: t_lo + self.step.duration,
            s: self.s_next,
            v: self.step.hi.v,
            rise: self.step.rise,
            departure: self.departure,
            slopes: self.step.hi.slopes,
        }
    }
}

impl RailLayout {
    fn lay(&mut self, laid: &Laid) {
        let knot = laid.knot(self.t);
        self.t = knot.t;
        self.w = laid.step.w_next.max(0.0);
        self.knots.push(knot);
        self.carried = Some(RailState {
            s: laid.s_end,
            ..laid.step.hi
        });
    }

    /// How many times over its allowance the worse defect of the pairs
    /// `laid` would add is.
    fn excess(&self, law: &ScalarLaw, laid: &[Laid]) -> f64 {
        let mut lo = *self
            .knots
            .last()
            .expect("a rail layout starts with its first knot");
        let mut worst = 0.0_f64;
        for laid in laid {
            let hi = laid.knot(lo.t);
            if hi.t > lo.t {
                worst = worst.max(defect::pair_defect(law, &law.rail_pair(&lo, &hi)).excess());
            }
            lo = hi;
        }
        worst
    }
}

/// The rail's state at arc `s` where `v² = w`: the law's acceleration, and
/// its rate — the settled curve's where the state rides a rising cap's
/// curve within [`RAIL_ACCELERATION_DEFECT`]: the curve draws the rail
/// along it, and the law's own rate there is its stiffness pulling an
/// ulp's departure back, not the motion's. A state holding a held cap
/// rides it flat: the law's acceleration there is the rounding of its
/// speed against the cap, which a rail holding the cap does not follow.
fn rail_state(law: &ScalarLaw, s: f64, w: f64) -> RailState {
    let v = w.max(0.0).sqrt();
    let a = law.accel_at(s, v);
    match law.settled_slopes(s, v) {
        Some((settled, rate)) if settled == 0.0 && rate == 0.0 && law.holds_cap(a) => RailState {
            s,
            v,
            a: 0.0,
            rate: 0.0,
            slopes: KnotSlopes::Interpolant {
                accel: 0.0,
                rate: 0.0,
            },
        },
        Some((settled, rate))
            if settled > 0.0 && (a - settled).abs() <= RAIL_ACCELERATION_DEFECT =>
        {
            RailState {
                s,
                v,
                a,
                rate,
                slopes: KnotSlopes::Interpolant { accel: a, rate },
            }
        }
        _ => RailState {
            s,
            v,
            a,
            rate: law.accel_rate_at(s, v, a),
            slopes: KnotSlopes::Law,
        },
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
    slopes: KnotSlopes,
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
