//! Enclosures of the continuous axes the shaper fits directly: an analytic
//! move's velocity law carried through its path, the polynomial tracks and
//! holds.
//!
//! Every order is enclosed from the formula `ContinuousAxis::eval_pva`
//! evaluates: the law's `(s, v, a)` between its phase seams and rail knots
//! — a polynomial in time, or in the square root of the time from a
//! curvature-cap departure — then the path's point, heading and turn at `s`.
//! The bounds hold the formulas in exact arithmetic on the evaluator's own
//! inputs, as an exact polynomial holds the convolution; the evaluator's
//! rounding of them is not in the bound. A clothoid's point is the one
//! formula that is itself an approximation (Cephes' Fresnel integrals): its
//! enclosure follows the clothoid from the evaluator's reading at the
//! window centre.

use std::ops::{Add, Mul, Neg};

use geometry::path::lowering::PositionProfile;
use geometry::path::{Arc as PathArc, Clothoid, Segment};
use geometry::{FollowerDemand, RailPair, RailPairShape, ScalarLaw};
use nurbs::bezier::extract_bezier_pieces;
use trajectory::{AnalyticMoveSpan, ContinuousAxis, RelativeSplinePiece, SurfaceMode};

use super::ExactPiece;
use super::taylor::{self, Ball, TaylorModel, Window, derivative_of, polynomial_of_balls};

/// Whether `axis` of `span` reads an analytic law its enclosure models.
pub(crate) fn encloses(span: &AnalyticMoveSpan, axis: usize) -> bool {
    AxisLaw::of(span, axis).is_some()
}

pub(crate) fn continuous_axis_enclosure(
    axis: &ContinuousAxis,
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
) -> Option<Vec<ExactPiece>> {
    let mut out = Vec::new();
    match axis {
        ContinuousAxis::Analytic { span, axis } => {
            out = analytic_axes_enclosure(span, &[*axis], (lo, hi), accuracy)?
                .pop()
                .expect("one axis asked, one enclosure built");
        }
        ContinuousAxis::Spline(curve) => spline_enclosure(curve, 0.0, (lo, hi), accuracy, &mut out),
        ContinuousAxis::RelativeSpline {
            base_position,
            curve,
        } => spline_enclosure(curve, *base_position, (lo, hi), accuracy, &mut out),
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => {
            relative_pieces_enclosure(pieces, (lo, hi), accuracy, &mut out);
        }
        ContinuousAxis::Hold { position, .. } => {
            let held = [
                TaylorModel::constant(Ball::exact(*position)),
                TaylorModel::constant(Ball::exact(0.0)),
                TaylorModel::constant(Ball::exact(0.0)),
            ];
            taylor::enclose((lo, hi), accuracy, &|_| held, &mut out);
        }
        ContinuousAxis::Nudge(_) | ContinuousAxis::Buzz { .. } => return None,
    }
    Some(out)
}

fn spline_enclosure(
    curve: &nurbs::ScalarNurbs,
    base: f64,
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    out: &mut Vec<ExactPiece>,
) {
    for piece in extract_bezier_pieces(curve) {
        let (from, to) = (lo.max(piece.u_start), hi.min(piece.u_end));
        if from >= to {
            continue;
        }
        taylor::enclose(
            (from, to),
            accuracy,
            &|window: Window| {
                let tau = window.time_since(piece.u_start);
                [0, 1, 2].map(|order| {
                    let model = derivative_of(&piece.coeffs, order, &tau);
                    if order == 0 {
                        model.add_ball(Ball::exact(base))
                    } else {
                        model
                    }
                })
            },
            out,
        );
    }
}

fn relative_pieces_enclosure(
    pieces: &[RelativeSplinePiece],
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    out: &mut Vec<ExactPiece>,
) {
    for piece in pieces {
        let (from, to) = (lo.max(piece.t_start), hi.min(piece.t_end));
        if from < to {
            spline_enclosure(&piece.curve, piece.base_position, (from, to), accuracy, out);
        }
    }
}

/// The law's state on a window: arc from the source's distance origin,
/// speed and tangential acceleration.
#[derive(Clone, Copy)]
struct Tangential {
    s: TaylorModel,
    v: TaylorModel,
    a: TaylorModel,
}

/// How an analytic axis reads the law's state.
enum AxisLaw<'a> {
    Constant(f64),
    Line {
        start: f64,
        extent: f64,
        length: f64,
    },
    Arc(&'a PathArc, usize),
    Clothoid(&'a Clothoid, usize),
    Follower {
        start: f64,
        demand: FollowerDemand,
        length: f64,
    },
}

impl AxisLaw<'_> {
    /// The axis's law and the constant surface offset its position adds.
    fn of(span: &AnalyticMoveSpan, axis: usize) -> Option<(AxisLaw<'_>, f64)> {
        if axis < 3 {
            let segment = span.source.segment.spatial.as_ref()?;
            let offset = match (&span.surface, axis) {
                (SurfaceMode::Variable(_), 2) => return None,
                (SurfaceMode::Constant(offset), 2) => *offset,
                _ => 0.0,
            };
            if let Some(coordinate) = segment.held_coordinate(axis) {
                return Some((AxisLaw::Constant(coordinate), offset));
            }
            let law = match segment {
                Segment::Line(line) => AxisLaw::Line {
                    start: line.start[axis],
                    extent: line.end[axis] - line.start[axis],
                    length: line.length(),
                },
                Segment::Arc(arc) => AxisLaw::Arc(arc, axis),
                Segment::Clothoid(clothoid) => AxisLaw::Clothoid(clothoid, axis),
            };
            return Some((law, offset));
        }
        let demand = if span.source.segment.spatial.is_some() {
            span.source.segment.followers.get(axis - 3)
        } else {
            span.source
                .segment
                .followers
                .iter()
                .find(|demand| demand.axis_index == axis)
        };
        let law = match demand {
            Some(demand) => AxisLaw::Follower {
                start: *span.axis_start_positions.get(demand.axis_index)?,
                demand: *demand,
                length: span.source.segment.s_len(),
            },
            None => AxisLaw::Constant(*span.axis_start_positions.get(axis)?),
        };
        Some((law, 0.0))
    }

    fn models(&self, window: &WindowReading) -> [TaylorModel; 3] {
        let Tangential { s, v, a } = &window.state;
        match *self {
            AxisLaw::Constant(position) => [
                TaylorModel::constant(Ball::exact(position)),
                TaylorModel::constant(Ball::exact(0.0)),
                TaylorModel::constant(Ball::exact(0.0)),
            ],
            AxisLaw::Line {
                start,
                extent,
                length,
            } => {
                let heading = Ball::rounded(extent / length, 1);
                [
                    s.scale(heading).add_ball(Ball::exact(start)),
                    v.scale(heading),
                    a.scale(heading),
                ]
            }
            AxisLaw::Arc(arc, axis) => {
                let sign = arc.sweep.signum();
                let (sine, cosine) = window.trig.get_or_init(|| {
                    s.scale(Ball::rounded(sign / arc.radius, 1))
                        .add_ball(Ball::exact(arc.start_angle))
                        .sin_cos()
                });
                let (u, w) = (arc.u[axis], arc.v[axis]);
                let position = cosine
                    .scale(Ball::rounded(arc.radius * u, 1))
                    .add(&sine.scale(Ball::rounded(arc.radius * w, 1)))
                    .add_ball(Ball::exact(arc.origin[axis]));
                let heading = sine
                    .scale(Ball::exact(-sign * u))
                    .add(&cosine.scale(Ball::exact(sign * w)));
                let turn = cosine
                    .scale(Ball::rounded(-u / arc.radius, 1))
                    .add(&sine.scale(Ball::rounded(-w / arc.radius, 1)));
                [
                    position,
                    v.mul(&heading),
                    a.mul(&heading).add(&v.mul(v).mul(&turn)),
                ]
            }
            AxisLaw::Clothoid(clothoid, axis) => {
                let (sine, cosine) = window
                    .trig
                    .get_or_init(|| clothoid_angle(clothoid, s).sin_cos());
                let (u, w) = (clothoid.u[axis], clothoid.v[axis]);
                let heading = cosine
                    .scale(Ball::exact(u))
                    .add(&sine.scale(Ball::exact(w)));
                let curvature = s
                    .scale(Ball::exact(clothoid.sigma))
                    .add_ball(Ball::exact(clothoid.kappa_0));
                let turn = curvature.mul(
                    &sine
                        .scale(Ball::exact(-u))
                        .add(&cosine.scale(Ball::exact(w))),
                );
                [
                    clothoid_position(clothoid, axis, s, &window.spread_trig),
                    v.mul(&heading),
                    a.mul(&heading).add(&v.mul(v).mul(&turn)),
                ]
            }
            AxisLaw::Follower {
                start,
                demand,
                length,
            } => {
                let (ratio, slope, offset) = if demand.ratio_end == demand.ratio {
                    (
                        TaylorModel::constant(Ball::exact(demand.ratio)),
                        0.0,
                        s.scale(Ball::exact(demand.ratio)),
                    )
                } else {
                    let ramp = Ball::rounded((demand.ratio_end - demand.ratio) / length, 2);
                    let ratio = s.scale(ramp).add_ball(Ball::exact(demand.ratio));
                    let offset = s
                        .scale(Ball::exact(demand.ratio))
                        .add(&s.mul(s).scale(ramp.mul(Ball::exact(0.5))));
                    (ratio, demand.ratio_slope(length), offset)
                };
                let slope = Ball::rounded(slope, 2);
                [
                    offset.add_ball(Ball::exact(start)),
                    ratio.mul(v),
                    ratio.mul(a).add(&v.mul(v).scale(slope)),
                ]
            }
        }
    }
}

fn clothoid_angle(clothoid: &Clothoid, s: &TaylorModel) -> TaylorModel {
    s.scale(Ball::exact(clothoid.kappa_0))
        .add(&s.mul(s).scale(Ball::exact(0.5 * clothoid.sigma)))
}

/// The clothoid's coordinate at `s`: its reading at the model's centre arc
/// `s_c` plus `∫_{s_c}^{s}` of its heading, the heading's model built over
/// the arc's whole spread and integrated there; `spread_trig` the sine and
/// cosine of its angle over that spread, shared by every axis.
fn clothoid_position(
    clothoid: &Clothoid,
    axis: usize,
    s: &TaylorModel,
    spread_trig: &std::cell::OnceCell<(TaylorModel, TaylorModel)>,
) -> TaylorModel {
    let (centre, spread) = s.spread();
    let anchor = Ball::exact(clothoid.point_at(centre)[axis]);
    if spread == 0.0 {
        return TaylorModel::constant(anchor);
    }
    let reach = taylor::upper(spread * (1.0 + CLOTHOID_REACH_MARGIN));
    let (sine, cosine) = spread_trig.get_or_init(|| {
        clothoid_angle(clothoid, &TaylorModel::affine(Ball::exact(centre), reach)).sin_cos()
    });
    let heading = cosine
        .scale(Ball::exact(clothoid.u[axis]))
        .add(&sine.scale(Ball::exact(clothoid.v[axis])));
    let travel = heading.integral().scale(Ball::exact(reach));
    let along = s
        .add_ball(Ball::exact(-centre))
        .scale(Ball::rounded(1.0 / reach, 1));
    along
        .compose_model(&travel)
        .map_or(TaylorModel::UNBOUNDED, |travel| travel.add_ball(anchor))
}

/// How far past the arc's spread its heading is modelled, so the arc's
/// normalized deviation keeps inside `[−1, 1]` through its rounding.
const CLOTHOID_REACH_MARGIN: f64 = 1.0 / (1_u64 << 30) as f64;

/// An axis's law with the constant surface offset its position adds.
struct AxisReading<'a> {
    law: AxisLaw<'a>,
    offset: f64,
}

impl AxisReading<'_> {
    fn models(&self, window: &WindowReading) -> [TaylorModel; 3] {
        let [position, velocity, acceleration] = self.law.models(window);
        [
            position.add_ball(Ball::exact(self.offset)),
            velocity,
            acceleration,
        ]
    }
}

/// One window's tangential state, and the path trigonometry every axis of
/// a curved segment reads off it, modelled once for all of them.
struct WindowReading {
    state: Tangential,
    trig: std::cell::OnceCell<(TaylorModel, TaylorModel)>,
    spread_trig: std::cell::OnceCell<(TaylorModel, TaylorModel)>,
}

/// The enclosures of the analytic `axes` of `span` over `(lo, hi)`, each
/// window's state and path trigonometry modelled once for every axis;
/// `None` where an axis has no analytic law.
pub(crate) fn analytic_axes_enclosure(
    span: &AnalyticMoveSpan,
    axes: &[usize],
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
) -> Option<Vec<Vec<ExactPiece>>> {
    let readings: Vec<AxisReading> = axes
        .iter()
        .map(|&axis| AxisLaw::of(span, axis).map(|(law, offset)| AxisReading { law, offset }))
        .collect::<Option<_>>()?;
    let mut outs = vec![Vec::new(); axes.len()];
    let (lo, hi) = (lo.max(span.t_start), hi.min(span.t_end));
    let enclose = |range: (f64, f64),
                   state: &dyn Fn(Window) -> Option<Tangential>,
                   outs: &mut Vec<Vec<ExactPiece>>| {
        taylor::enclose_axes(
            range,
            accuracy,
            &|window: Window, axis: usize, shared: &mut Option<Option<WindowReading>>| {
                let reading = shared.get_or_insert_with(|| {
                    state(window).map(|state| WindowReading {
                        state,
                        trig: std::cell::OnceCell::new(),
                        spread_trig: std::cell::OnceCell::new(),
                    })
                });
                reading
                    .as_ref()
                    .map_or([TaylorModel::UNBOUNDED; 3], |reading| {
                        readings[axis].models(reading)
                    })
            },
            outs,
        );
    };
    for phase in span.phases.iter() {
        let origin = span.t_start + phase.t0;
        let (from, to) = (lo.max(origin), hi.min(span.t_start + phase.end_time()));
        if from >= to {
            continue;
        }
        match phase.law {
            ScalarLaw::ConstAccel { a0 } => {
                let arc_origin = Ball::rounded(phase.s0 - span.source_distance_origin, 1);
                enclose(
                    (from, to),
                    &|window: Window| {
                        let tau = window.time_since(origin);
                        Some(Tangential {
                            s: polynomial_of_balls(
                                [arc_origin, Ball::exact(phase.v0), Ball::exact(0.5 * a0)]
                                    .into_iter(),
                                &tau,
                            ),
                            v: polynomial_of_balls(
                                [Ball::exact(phase.v0), Ball::exact(a0)].into_iter(),
                                &tau,
                            ),
                            a: TaylorModel::constant(Ball::exact(a0)),
                        })
                    },
                    &mut outs,
                );
            }
            ScalarLaw::DiskRail { .. } => {
                let arc_base = phase.s0 - span.source_distance_origin;
                for pair in phase.rail_pairs(from - origin, to - origin) {
                    let range = (from.max(origin + pair.t_lo), to.min(origin + pair.t_hi));
                    if range.0 >= range.1 {
                        continue;
                    }
                    let interpolant = PairInterpolant::new(&pair, arc_base);
                    enclose(
                        range,
                        &|window: Window| interpolant.state(&window.time_since(origin)),
                        &mut outs,
                    );
                }
            }
        }
    }
    Some(outs)
}

/// The variable a rail pair's formula is a polynomial in.
#[derive(Clone, Copy, Debug, PartialEq)]
enum PairVariable {
    /// Its time share `u`.
    Time,
    /// `z = √u`, leaving the cap from its low end.
    FromLo,
    /// `z = √(1 − u)`, leaving the cap from its high end.
    FromHi,
    /// `z = √(θ/parent_h)` at the departure-frame time `θ` running from
    /// `theta_lo` to `theta_hi` across the pair, clear of the departure.
    Clipped {
        parent_h: f64,
        theta_lo: f64,
        theta_hi: f64,
    },
}

/// A rail knot pair's formula as monomial coefficients: speed, `h·a` and
/// `∫₀ v` in the pair's variable — its time `u` for a quintic, the square
/// root `z` of the time from the departure end for a cap departure — and
/// the arc's linear correction.
struct PairInterpolant {
    t_lo: f64,
    inverse_h: Ball,
    h: f64,
    variable: PairVariable,
    speed: [Ball; 8],
    scaled_acceleration: [Ball; 8],
    mass: [Ball; 8],
    arc_start: Ball,
    arc_correction: Ball,
    full_mass: Ball,
}

fn linear(terms: &[(f64, Ball)]) -> Ball {
    terms
        .iter()
        .fold(Ball::exact(0.0), |sum, &(weight, value)| {
            sum.add(Ball::exact(weight).mul(value))
        })
}

impl PairInterpolant {
    fn new(pair: &RailPair, arc_base: f64) -> Self {
        let h = pair.t_hi - pair.t_lo;
        let zero = Ball::exact(0.0);
        let (speed, scaled_acceleration, mass, full_mass, variable) = match pair.shape {
            RailPairShape::Quintic { rise, slopes } => {
                let [m0, c0, m1, c1] = slopes.map(Ball::exact);
                let cubic_terms = |delta: Ball| {
                    [
                        linear(&[(10.0, delta), (-6.0, m0), (-1.5, c0), (-4.0, m1), (0.5, c1)]),
                        linear(&[(-15.0, delta), (8.0, m0), (1.5, c0), (7.0, m1), (-1.0, c1)]),
                        linear(&[(6.0, delta), (-3.0, m0), (-0.5, c0), (-3.0, m1), (0.5, c1)]),
                    ]
                };
                let delta = Ball::exact(pair.v_hi).add(Ball::exact(-pair.v_lo));
                let [v3, v4, v5] = cubic_terms(delta);
                let speed = [
                    Ball::exact(pair.v_lo),
                    m0,
                    c0.mul(Ball::exact(0.5)),
                    v3,
                    v4,
                    v5,
                    zero,
                    zero,
                ];
                let [r3, r4, r5] = cubic_terms(Ball::exact(rise));
                let scaled_acceleration = [
                    m0,
                    c0,
                    r3.mul(Ball::exact(3.0)),
                    r4.mul(Ball::exact(4.0)),
                    r5.mul(Ball::exact(5.0)),
                    zero,
                    zero,
                    zero,
                ];
                let mut mass = [zero; 8];
                for power in 0..6 {
                    mass[power + 1] = speed[power].mul(Ball::rounded(1.0 / (power + 1) as f64, 1));
                }
                let full_mass = mass.iter().fold(zero, |sum, &term| sum.add(term));
                (
                    speed,
                    scaled_acceleration,
                    mass,
                    full_mass,
                    PairVariable::Time,
                )
            }
            RailPairShape::Departure {
                from_lo,
                coefficients,
            } => {
                let [c2, c3, c4, c5] = coefficients.map(Ball::exact);
                let end_speed = Ball::exact(if from_lo { pair.v_lo } else { pair.v_hi });
                let speed = [end_speed, zero, c2, c3, c4, c5, zero, zero];
                let scaled_acceleration = [
                    c2,
                    c3.mul(Ball::exact(1.5)),
                    c4.mul(Ball::exact(2.0)),
                    c5.mul(Ball::exact(2.5)),
                    zero,
                    zero,
                    zero,
                    zero,
                ];
                let rise_mass = [
                    zero,
                    zero,
                    zero,
                    zero,
                    c2.mul(Ball::exact(0.5)),
                    c3.mul(Ball::rounded(0.4, 1)),
                    c4.mul(Ball::rounded(1.0 / 3.0, 1)),
                    c5.mul(Ball::rounded(2.0 / 7.0, 1)),
                ];
                let full_mass = rise_mass.iter().fold(end_speed, |sum, &term| sum.add(term));
                let variable = if from_lo {
                    PairVariable::FromLo
                } else {
                    PairVariable::FromHi
                };
                (speed, scaled_acceleration, rise_mass, full_mass, variable)
            }
            RailPairShape::Clipped {
                coefficients,
                h: parent_h,
                v_end,
                theta_lo,
                theta_hi,
            } => {
                let [c2, c3, c4, c5] = coefficients.map(Ball::exact);
                let end_speed = Ball::exact(v_end);
                let speed = [end_speed, zero, c2, c3, c4, c5, zero, zero];
                let acceleration_scale =
                    Ball::rounded((theta_hi - theta_lo).signum() * h / parent_h, 1);
                let scaled_acceleration = [
                    c2,
                    c3.mul(Ball::exact(1.5)),
                    c4.mul(Ball::exact(2.0)),
                    c5.mul(Ball::exact(2.5)),
                    zero,
                    zero,
                    zero,
                    zero,
                ]
                .map(|c| c.mul(acceleration_scale));
                let mass_scale = Ball::rounded(parent_h / (theta_hi - theta_lo), 1);
                let rise_mass = [
                    zero,
                    zero,
                    zero,
                    zero,
                    c2.mul(Ball::exact(0.5)),
                    c3.mul(Ball::rounded(0.4, 1)),
                    c4.mul(Ball::rounded(1.0 / 3.0, 1)),
                    c5.mul(Ball::rounded(2.0 / 7.0, 1)),
                ]
                .map(|c| c.mul(mass_scale));
                let rise_mass_at = |theta: f64| {
                    let z = Ball::rounded((theta / parent_h).sqrt(), 2);
                    rise_mass
                        .iter()
                        .rev()
                        .fold(zero, |acc, &c| acc.mul(z).add(c))
                };
                let mut mass = rise_mass;
                mass[0] = rise_mass_at(theta_lo).neg();
                let full_mass = end_speed.add(rise_mass_at(theta_hi)).add(mass[0]);
                (
                    speed,
                    scaled_acceleration,
                    mass,
                    full_mass,
                    PairVariable::Clipped {
                        parent_h,
                        theta_lo,
                        theta_hi,
                    },
                )
            }
        };
        Self {
            t_lo: pair.t_lo,
            inverse_h: Ball::rounded(1.0 / h, 1),
            h,
            variable,
            speed,
            scaled_acceleration,
            mass,
            arc_start: Ball::rounded(arc_base, 1).add(Ball::exact(pair.s_lo)),
            arc_correction: Ball::exact(pair.s_hi)
                .add(Ball::exact(-pair.s_lo))
                .add(Ball::exact(h).mul(full_mass).neg()),
            full_mass,
        }
    }

    /// The pair's `(s, v, a)` for the model `tau` of segment-local time, or
    /// `None` where a departure's square root reaches its singular end.
    fn state(&self, tau: &TaylorModel) -> Option<Tangential> {
        let u = tau.add_ball(Ball::exact(-self.t_lo)).scale(self.inverse_h);
        let poly = |coefficients: &[Ball; 8], at: &TaylorModel| {
            polynomial_of_balls(coefficients.iter().copied(), at)
        };
        let h = Ball::exact(self.h);
        let (v, a, mass) = match self.variable {
            PairVariable::Time => (
                poly(&self.speed, &u),
                poly(&self.scaled_acceleration, &u),
                poly(&self.mass, &u),
            ),
            PairVariable::FromLo => {
                let z = u.sqrt()?;
                let mass = poly(&self.mass, &z).add(&u.scale(self.speed[0]));
                (
                    poly(&self.speed, &z),
                    poly(&self.scaled_acceleration, &z),
                    mass,
                )
            }
            PairVariable::FromHi => {
                let remaining = u.scale(Ball::exact(-1.0)).add_ball(Ball::exact(1.0));
                let z = remaining.sqrt()?;
                let mass = poly(&self.mass, &z)
                    .add(&remaining.scale(self.speed[0]))
                    .scale(Ball::exact(-1.0))
                    .add_ball(self.full_mass);
                (
                    poly(&self.speed, &z),
                    poly(&self.scaled_acceleration, &z).scale(Ball::exact(-1.0)),
                    mass,
                )
            }
            PairVariable::Clipped {
                parent_h,
                theta_lo,
                theta_hi,
            } => {
                let theta = u
                    .scale(Ball::rounded(theta_hi - theta_lo, 1))
                    .add_ball(Ball::exact(theta_lo));
                let z = theta.scale(Ball::rounded(1.0 / parent_h, 1)).sqrt()?;
                let mass = poly(&self.mass, &z).add(&u.scale(self.speed[0]));
                (
                    poly(&self.speed, &z),
                    poly(&self.scaled_acceleration, &z),
                    mass,
                )
            }
        };
        let mut v = v;
        let (centre, spread) = v.spread();
        let lowest = centre - spread;
        if lowest < 0.0 {
            v.widen(-lowest);
        }
        Some(Tangential {
            s: mass
                .scale(h)
                .add(&u.scale(self.arc_correction))
                .add_ball(self.arc_start),
            v,
            a: a.scale(self.inverse_h),
        })
    }
}

#[cfg(test)]
#[path = "shaper_axis_enclosure_tests.rs"]
mod tests;
