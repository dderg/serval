//! Enclosures of the continuous axes the shaper fits directly: an analytic
//! move's velocity law carried through its path, the polynomial tracks,
//! holds, nudges and buzzes.
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
use geometry::{
    FollowerDemand, MeshPatch, PatchAxis, RailPair, RailPairShape, ScalarLaw, SurfaceTransform,
};
use nurbs::bezier::BezierPiece;
use nurbs::bezier::extract_bezier_pieces;
use trajectory::{
    AnalyticMoveSpan, BuzzProfile, ContinuousAxis, NudgeProfile, RelativeSplinePiece, SurfaceMode,
};

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
        ContinuousAxis::Nudge(profile) => {
            pieces_enclosure(nudge_pieces(profile), 0.0, (lo, hi), accuracy, &mut out);
        }
        ContinuousAxis::Buzz {
            base_position,
            sign,
            profile,
        } => {
            for (start, end, envelope) in profile.envelope_pieces() {
                let (from, to) = (lo.max(start), hi.min(end));
                if from >= to {
                    continue;
                }
                taylor::enclose(
                    (from, to),
                    accuracy,
                    &|window: Window| {
                        let [p, v, a, _] = buzz_models(profile, envelope, window, false);
                        [
                            p.scale(Ball::exact(*sign))
                                .add_ball(Ball::exact(*base_position)),
                            v.scale(Ball::exact(*sign)),
                            a.scale(Ball::exact(*sign)),
                        ]
                    },
                    &mut out,
                );
            }
        }
    }
    Some(out)
}

/// A nudge's phases as polynomial pieces in the time since each starts.
fn nudge_pieces(profile: &NudgeProfile) -> Vec<BezierPiece> {
    profile
        .phase_polynomials()
        .into_iter()
        .map(|(u_start, u_end, coeffs)| BezierPiece {
            u_start,
            u_end,
            coeffs: coeffs.to_vec(),
        })
        .collect()
}

/// A buzz carrier's `[p, v, a, j]` on `window` inside the envelope stretch
/// reading `envelope` (`e₀ + e₁·τ`, `τ` since the profile's start), before
/// its sign and base — the formulas `BuzzProfile::eval` and `jerk` read:
/// with `b = envelope·amplitude`, `amplitude = A·ω₀/ω` (or `A` without a
/// sweep) and the phase `(ω₀ + ½ṡ·τ)·τ`, `p = b·sin`, `v = ḃ·sin + bω·cos`,
/// `a = (b̈ − bω²)·sin + (2ḃω + bṡ)·cos`, and `j` likewise; `j` only
/// `with_jerk`, else unbounded.
fn buzz_models(
    profile: &BuzzProfile,
    [e0, e1]: [f64; 2],
    window: Window,
    with_jerk: bool,
) -> [TaylorModel; 4] {
    let tau = window.time_since(profile.t_start());
    let sweep = profile.sweep_rate();
    let omega = tau
        .scale(Ball::exact(sweep))
        .add_ball(Ball::exact(profile.omega_start()));
    let zero = TaylorModel::constant(Ball::exact(0.0));
    let [amplitude, amplitude_rate, amplitude_accel, amplitude_jerk] = if sweep == 0.0 {
        [
            TaylorModel::constant(Ball::exact(profile.amplitude_mm())),
            zero,
            zero,
            zero,
        ]
    } else {
        let inverse = omega
            .reciprocal()
            .expect("a sweeping buzz's angular frequency stays off zero");
        let base = Ball::rounded(profile.amplitude_mm() * profile.omega_start(), 1);
        let sweep = Ball::exact(sweep);
        let inverse_squared = inverse.mul(&inverse);
        let inverse_cubed = inverse_squared.mul(&inverse);
        [
            inverse.scale(base),
            inverse_squared.scale(-(base * sweep)),
            inverse_cubed.scale(Ball::exact(2.0) * base * sweep * sweep),
            inverse_cubed
                .mul(&inverse)
                .scale(-(Ball::exact(6.0) * base * sweep * sweep * sweep)),
        ]
    };
    let envelope = tau.scale(Ball::exact(e1)).add_ball(Ball::exact(e0));
    let envelope_rate = Ball::exact(e1);
    let b = envelope.mul(&amplitude);
    let b_rate = amplitude
        .scale(envelope_rate)
        .add(&envelope.mul(&amplitude_rate));
    let b_accel = amplitude_rate
        .scale(Ball::exact(2.0) * envelope_rate)
        .add(&envelope.mul(&amplitude_accel));
    let phase = polynomial_of_balls(
        [
            Ball::exact(0.0),
            Ball::exact(profile.omega_start()),
            Ball::exact(0.5 * sweep),
        ]
        .into_iter(),
        &tau,
    );
    let (sine, cosine) = phase.sin_cos();
    let sweep = Ball::exact(sweep);
    let omega_squared = omega.mul(&omega);
    let position = b.mul(&sine);
    let velocity = b_rate.mul(&sine).add(&b.mul(&omega).mul(&cosine));
    let acceleration = b_accel.sub(&b.mul(&omega_squared)).mul(&sine).add(
        &b_rate
            .mul(&omega)
            .scale(Ball::exact(2.0))
            .add(&b.scale(sweep))
            .mul(&cosine),
    );
    let jerk = if with_jerk {
        let b_jerk = amplitude_accel
            .scale(Ball::exact(3.0) * envelope_rate)
            .add(&envelope.mul(&amplitude_jerk));
        let sine_coefficient = b_jerk
            .sub(&b_rate.mul(&omega_squared).scale(Ball::exact(3.0)))
            .sub(&b.mul(&omega).scale(Ball::exact(3.0) * sweep));
        let cosine_coefficient = b_accel
            .mul(&omega)
            .scale(Ball::exact(3.0))
            .sub(&b.mul(&omega_squared).mul(&omega))
            .add(&b_rate.scale(Ball::exact(3.0) * sweep));
        sine_coefficient
            .mul(&sine)
            .add(&cosine_coefficient.mul(&cosine))
    } else {
        TaylorModel::UNBOUNDED
    };
    [position, velocity, acceleration, jerk]
}

fn spline_enclosure(
    curve: &nurbs::ScalarNurbs,
    base: f64,
    span: (f64, f64),
    accuracy: [f64; 3],
    out: &mut Vec<ExactPiece>,
) {
    pieces_enclosure(extract_bezier_pieces(curve), base, span, accuracy, out);
}

fn pieces_enclosure(
    pieces: Vec<BezierPiece>,
    base: f64,
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    out: &mut Vec<ExactPiece>,
) {
    for piece in pieces {
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
    /// The path's z carried onto a variable bed surface: `z + w(x, y, z)`,
    /// the path's coordinates read by their own laws.
    Warped {
        surface: &'a SurfaceTransform,
        path: Box<[AxisLaw<'a>; 3]>,
    },
}

impl<'a> AxisLaw<'a> {
    /// The axis's law and the constant surface offset its position adds.
    fn of(span: &'a AnalyticMoveSpan, axis: usize) -> Option<(AxisLaw<'a>, f64)> {
        if axis < 3 {
            let segment = span.source.segment.spatial.as_ref()?;
            return Some(match (&span.surface, axis) {
                (SurfaceMode::Variable(surface), 2) => (
                    AxisLaw::Warped {
                        surface,
                        path: Box::new(std::array::from_fn(|axis| path_law(segment, axis))),
                    },
                    0.0,
                ),
                (SurfaceMode::Constant(offset), 2) => (path_law(segment, axis), *offset),
                _ => (path_law(segment, axis), 0.0),
            });
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
        match self {
            AxisLaw::Warped { surface, path } => {
                warped_models(surface, path.each_ref().map(|law| law.models(window)))
            }
            _ => self.flat_models(window, s, v, a),
        }
    }

    fn flat_models(
        &self,
        window: &WindowReading,
        s: &TaylorModel,
        v: &TaylorModel,
        a: &TaylorModel,
    ) -> [TaylorModel; 3] {
        match *self {
            AxisLaw::Warped { .. } => unreachable!("a warped law reads its path's flat laws"),
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

    /// The axis's jerk on the window, the tangential jerk `jerk` given:
    /// `j·h + 3·a·v·h' + v³·h''`, `h` the axis's heading along the path and
    /// `'` its arc derivatives; `None` on a variable surface, whose z the
    /// evaluator reads no jerk of.
    fn jerk(&self, window: &WindowReading, jerk: &TaylorModel) -> Option<TaylorModel> {
        let Tangential { s, v, a } = &window.state;
        let zero = TaylorModel::constant(Ball::exact(0.0));
        let (heading, turn, bend) = match *self {
            AxisLaw::Warped { .. } => return None,
            AxisLaw::Constant(_) => return Some(zero),
            AxisLaw::Line { extent, length, .. } => {
                return Some(jerk.scale(Ball::rounded(extent / length, 1)));
            }
            AxisLaw::Arc(arc, axis) => {
                let sign = arc.sweep.signum();
                let (sine, cosine) = window.trig.get_or_init(|| {
                    s.scale(Ball::rounded(sign / arc.radius, 1))
                        .add_ball(Ball::exact(arc.start_angle))
                        .sin_cos()
                });
                let (u, w) = (arc.u[axis], arc.v[axis]);
                let heading = sine
                    .scale(Ball::exact(-sign * u))
                    .add(&cosine.scale(Ball::exact(sign * w)));
                let turn = cosine
                    .scale(Ball::rounded(-u / arc.radius, 1))
                    .add(&sine.scale(Ball::rounded(-w / arc.radius, 1)));
                let bend = heading.scale(Ball::rounded(-1.0 / (arc.radius * arc.radius), 2));
                (heading, turn, bend)
            }
            AxisLaw::Clothoid(clothoid, axis) => {
                let (sine, cosine) = window
                    .trig
                    .get_or_init(|| clothoid_angle(clothoid, s).sin_cos());
                let (u, w) = (clothoid.u[axis], clothoid.v[axis]);
                let heading = cosine
                    .scale(Ball::exact(u))
                    .add(&sine.scale(Ball::exact(w)));
                let normal = sine
                    .scale(Ball::exact(-u))
                    .add(&cosine.scale(Ball::exact(w)));
                let curvature = s
                    .scale(Ball::exact(clothoid.sigma))
                    .add_ball(Ball::exact(clothoid.kappa_0));
                let bend = normal
                    .scale(Ball::exact(clothoid.sigma))
                    .sub(&curvature.mul(&curvature).mul(&heading));
                (heading, curvature.mul(&normal), bend)
            }
            AxisLaw::Follower { demand, length, .. } => {
                let (ratio, slope) = if demand.ratio_end == demand.ratio {
                    (TaylorModel::constant(Ball::exact(demand.ratio)), 0.0)
                } else {
                    let ramp = Ball::rounded((demand.ratio_end - demand.ratio) / length, 2);
                    (
                        s.scale(ramp).add_ball(Ball::exact(demand.ratio)),
                        demand.ratio_slope(length),
                    )
                };
                return Some(
                    ratio
                        .mul(jerk)
                        .add(&a.mul(v).scale(Ball::rounded(3.0 * slope, 2))),
                );
            }
        };
        Some(
            jerk.mul(&heading)
                .add(&a.mul(v).mul(&turn).scale(Ball::exact(3.0)))
                .add(&v.mul(v).mul(v).mul(&bend)),
        )
    }
}

/// The law `segment`'s point reads along `axis`.
fn path_law(segment: &Segment, axis: usize) -> AxisLaw<'_> {
    if let Some(coordinate) = segment.held_coordinate(axis) {
        return AxisLaw::Constant(coordinate);
    }
    match segment {
        Segment::Line(line) => AxisLaw::Line {
            start: line.start[axis],
            extent: line.end[axis] - line.start[axis],
            length: line.length(),
        },
        Segment::Arc(arc) => AxisLaw::Arc(arc, axis),
        Segment::Clothoid(clothoid) => AxisLaw::Clothoid(clothoid, axis),
    }
}

/// `[p, v, a]` of the path's z on `surface`, `[x, y, z]` the path's own
/// `[p, v, a]`: the chain rule `eval_warped_z` evaluates, on the mesh patch
/// and fade stretch the models' centre reads.
fn warped_models(surface: &SurfaceTransform, [x, y, z]: [[TaylorModel; 3]; 3]) -> [TaylorModel; 3] {
    let centre = |model: &TaylorModel| model.coefficients()[0];
    let patch = surface.mesh().patch_at(centre(&x[0]), centre(&y[0]));
    let (u, du) = patch_parameter(patch.x, &x[0]);
    let (v, dv) = patch_parameter(patch.y, &y[0]);
    let mesh = PatchReading::of(&patch, &u, &v);
    let fade = surface.fade();
    let target = Ball::exact(fade.target);
    let (factor, factor_slope) = if fade.is_disabled() {
        (TaylorModel::constant(Ball::exact(1.0)), Ball::exact(0.0))
    } else {
        let (start, end) = fade.band();
        let height = centre(&z[0]);
        if height >= end {
            (TaylorModel::constant(Ball::exact(0.0)), Ball::exact(0.0))
        } else if height >= start {
            let inverse_band = Ball::rounded(1.0 / (end - start), 2);
            (
                z[0].scale(Ball::exact(-1.0))
                    .add_ball(Ball::exact(end))
                    .scale(inverse_band),
                -inverse_band,
            )
        } else {
            (TaylorModel::constant(Ball::exact(1.0)), Ball::exact(0.0))
        }
    };
    let deviation = mesh.value.add_ball(-target);
    let mesh_x = mesh.du.scale(du);
    let mesh_y = mesh.dv.scale(dv);
    let w = factor.mul(&deviation).add_ball(target);
    let wx = factor.mul(&mesh_x);
    let wy = factor.mul(&mesh_y);
    let wz = deviation.scale(factor_slope);
    let wxx = factor.mul(&mesh.duu.scale(du * du));
    let wxy = factor.mul(&mesh.duv.scale(du * dv));
    let wyy = factor.mul(&mesh.dvv.scale(dv * dv));
    let wxz = mesh_x.scale(factor_slope);
    let wyz = mesh_y.scale(factor_slope);
    let two = Ball::exact(2.0);
    let [_, xv, xa] = &x;
    let [_, yv, ya] = &y;
    let [zp, zv, za] = &z;
    [
        zp.add(&w),
        zv.add(&wx.mul(xv)).add(&wy.mul(yv)).add(&wz.mul(zv)),
        za.add(&wx.mul(xa))
            .add(&wy.mul(ya))
            .add(&wz.mul(za))
            .add(&wxx.mul(xv).mul(xv))
            .add(&wxy.mul(xv).mul(yv).scale(two))
            .add(&wyy.mul(yv).mul(yv))
            .add(&wxz.mul(xv).mul(zv).scale(two))
            .add(&wyz.mul(yv).mul(zv).scale(two)),
    ]
}

/// A patch parameter's model along `coordinate` and its rate in it: the
/// grid's inverse spacing inside, held flat past its edge.
fn patch_parameter(axis: PatchAxis, coordinate: &TaylorModel) -> (TaylorModel, Ball) {
    match axis {
        PatchAxis::Inside { min, spacing, cell } => {
            let rate = Ball::rounded(1.0 / spacing, 1);
            (
                coordinate
                    .add_ball(Ball::exact(-min))
                    .scale(rate)
                    .add_ball(Ball::exact(-cell)),
                rate,
            )
        }
        PatchAxis::Held(parameter) => (
            TaylorModel::constant(Ball::exact(parameter)),
            Ball::exact(0.0),
        ),
    }
}

/// A mesh patch's height and its parameter derivatives on a window.
struct PatchReading {
    value: TaylorModel,
    du: TaylorModel,
    dv: TaylorModel,
    duu: TaylorModel,
    duv: TaylorModel,
    dvv: TaylorModel,
}

impl PatchReading {
    fn of(patch: &MeshPatch, u: &TaylorModel, v: &TaylorModel) -> Self {
        let along_u = cardinal_weights(patch.tension, u);
        let along_v = cardinal_weights(patch.tension, v);
        let rows: [[TaylorModel; 3]; 4] = std::array::from_fn(|k| {
            std::array::from_fn(|order| {
                patch.points[k].iter().zip(&along_u).fold(
                    TaylorModel::constant(Ball::exact(0.0)),
                    |sum, (&point, weight)| sum.add(&weight[order].scale(Ball::exact(point))),
                )
            })
        });
        let combine = |row_order: usize, column_order: usize| {
            rows.iter().zip(&along_v).fold(
                TaylorModel::constant(Ball::exact(0.0)),
                |sum, (row, weight)| sum.add(&weight[column_order].mul(&row[row_order])),
            )
        };
        Self {
            value: combine(0, 0),
            du: combine(1, 0),
            dv: combine(0, 1),
            duu: combine(2, 0),
            duv: combine(1, 1),
            dvv: combine(0, 2),
        }
    }
}

/// The cardinal spline's four weights at the parameter `t` with their
/// first two derivatives in it: the cubic Hermite basis `h₀₀ = 1 − 3t² +
/// 2t³`, `h₁₀ = t − 2t² + t³`, `h₀₁ = 3t² − 2t³`, `h₁₁ = t³ − t²`
/// combined as [`MeshPatch`] reads them.
fn cardinal_weights(tension: f64, t: &TaylorModel) -> [[TaylorModel; 3]; 4] {
    let tension = Ball::exact(tension);
    let basis = |coeffs: [f64; 4]| coeffs.map(Ball::exact);
    let h00 = basis([1.0, 0.0, -3.0, 2.0]);
    let h10 = basis([0.0, 1.0, -2.0, 1.0]);
    let h01 = basis([0.0, 0.0, 3.0, -2.0]);
    let h11 = basis([0.0, 0.0, -1.0, 1.0]);
    let weights: [[Ball; 4]; 4] = [
        h10.map(|c| -(tension * c)),
        std::array::from_fn(|n| h00[n].add(-(tension * h11[n]))),
        std::array::from_fn(|n| h01[n].add(tension * h10[n])),
        h11.map(|c| tension * c),
    ];
    weights.map(|[c0, c1, c2, c3]| {
        let two = Ball::exact(2.0);
        let three = Ball::exact(3.0);
        [
            polynomial_of_balls([c0, c1, c2, c3].into_iter(), t),
            polynomial_of_balls([c1, two * c2, three * c3].into_iter(), t),
            polynomial_of_balls([two * c2, Ball::exact(6.0) * c3].into_iter(), t),
        ]
    })
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
pub(crate) struct AxisReading<'a> {
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
    let surface_seams = if readings
        .iter()
        .any(|reading| matches!(reading.law, AxisLaw::Warped { .. }))
    {
        span.surface_seam_times()?
    } else {
        Vec::new()
    };
    let mut outs = vec![Vec::new(); axes.len()];
    let (lo, hi) = (lo.max(span.t_start), hi.min(span.t_end));
    let enclose = |(from, to): (f64, f64),
                   state: &dyn Fn(Window) -> Tangential,
                   outs: &mut Vec<Vec<ExactPiece>>| {
        let mut edges = vec![from];
        edges.extend(
            surface_seams
                .iter()
                .copied()
                .filter(|&seam| seam > from && seam < to),
        );
        edges.push(to);
        edges.sort_by(f64::total_cmp);
        for stretch in edges.windows(2) {
            taylor::enclose_axes(
                (stretch[0], stretch[1]),
                accuracy,
                &|window: Window, axis: usize, shared: &mut Option<WindowReading>| {
                    let reading = shared.get_or_insert_with(|| WindowReading {
                        state: state(window),
                        trig: std::cell::OnceCell::new(),
                        spread_trig: std::cell::OnceCell::new(),
                    });
                    readings[axis].models(reading)
                },
                outs,
            );
        }
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
                        Tangential {
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
                        }
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

/// A rail leaving its curvature cap: from `at` toward `far`, the pair's
/// other end, its tangential acceleration `a = (c₂ + 1.5c₃·z + …)/h` rises
/// in `z = √(τ/h)`, `τ` the time from `at`, so its jerk is
/// `jerk_coefficient/√τ` past terms bounded on the pair:
/// `0.75·|c₃|/h^{3/2}`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SquareRootOnset {
    pub at: f64,
    pub far: f64,
    pub jerk_coefficient: f64,
}

impl SquareRootOnset {
    fn of(pair: &RailPair, origin: f64) -> Option<Self> {
        let RailPairShape::Departure {
            from_lo,
            coefficients: [_, c3, _, _],
        } = pair.shape
        else {
            return None;
        };
        let h = pair.t_hi - pair.t_lo;
        let (at, far) = if from_lo {
            (pair.t_lo, pair.t_hi)
        } else {
            (pair.t_hi, pair.t_lo)
        };
        (c3 != 0.0).then(|| Self {
            at: origin + at,
            far: origin + far,
            jerk_coefficient: 0.75 * c3.abs() / (h * h.sqrt()),
        })
    }
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

    /// The pair's `(s, v, a)` for the model `tau` of segment-local time. A
    /// departure window reaching its singular end, where the square root
    /// has no model, is enclosed by the formula's range over the window.
    fn state(&self, tau: &TaylorModel) -> Tangential {
        let u = tau.add_ball(Ball::exact(-self.t_lo)).scale(self.inverse_h);
        let poly = |coefficients: &[Ball; 8], at: &TaylorModel| {
            polynomial_of_balls(coefficients.iter().copied(), at)
        };
        let h = Ball::exact(self.h);
        let in_root = |z: (TaylorModel, bool), model: TaylorModel| {
            if z.1 { collapsed(&model) } else { model }
        };
        let (v, a, mass) = match self.variable {
            PairVariable::Time => (
                poly(&self.speed, &u),
                poly(&self.scaled_acceleration, &u),
                poly(&self.mass, &u),
            ),
            PairVariable::FromLo => {
                let z = root_of(&u);
                let mass = in_root(z, poly(&self.mass, &z.0)).add(&u.scale(self.speed[0]));
                (
                    in_root(z, poly(&self.speed, &z.0)),
                    in_root(z, poly(&self.scaled_acceleration, &z.0)),
                    mass,
                )
            }
            PairVariable::FromHi => {
                let remaining = u.scale(Ball::exact(-1.0)).add_ball(Ball::exact(1.0));
                let z = root_of(&remaining);
                let mass = in_root(z, poly(&self.mass, &z.0))
                    .add(&remaining.scale(self.speed[0]))
                    .scale(Ball::exact(-1.0))
                    .add_ball(self.full_mass);
                (
                    in_root(z, poly(&self.speed, &z.0)),
                    in_root(z, poly(&self.scaled_acceleration, &z.0)).scale(Ball::exact(-1.0)),
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
                let z = root_of(&theta.scale(Ball::rounded(1.0 / parent_h, 1)));
                let mass = in_root(z, poly(&self.mass, &z.0)).add(&u.scale(self.speed[0]));
                (
                    in_root(z, poly(&self.speed, &z.0)),
                    in_root(z, poly(&self.scaled_acceleration, &z.0)),
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
        Tangential {
            s: mass
                .scale(h)
                .add(&u.scale(self.arc_correction))
                .add_ball(self.arc_start),
            v,
            a: a.scale(self.inverse_h),
        }
    }
}

/// Below this the evaluator reads a rail's jerk as zero: an acceleration
/// that small is a held cap's rounding.
const RAIL_JERK_ACCELERATION_FLOOR: f64 = 1e-15;

/// The jerk the rail's law reads at the interpolated state as the
/// evaluator computes it: the time derivative of `√(A² − κ²v⁴)` along it,
/// `−κv³·(σv² + 2κa)/a`, and zero below [`RAIL_JERK_ACCELERATION_FLOOR`].
/// On constant curvature that is `−2κ²v³` off the floor, enclosed with
/// zero where the acceleration's range reaches the floor; on a turning one
/// `None` there, where it is unbounded.
fn rail_law_jerk(kappa: &TaylorModel, sigma: f64, state: &Tangential) -> Option<TaylorModel> {
    let Tangential { v, a, .. } = state;
    let speed_squared = v.mul(v);
    let (centre, spread) = a.spread();
    let reaches_floor = centre.abs() - spread < RAIL_JERK_ACCELERATION_FLOOR;
    if sigma == 0.0 {
        let mut jerk = kappa
            .mul(kappa)
            .mul(&speed_squared)
            .mul(v)
            .scale(Ball::exact(-2.0));
        if reaches_floor {
            let reach = jerk.bound();
            jerk = TaylorModel::constant(Ball::exact(0.0));
            jerk.widen(reach);
        }
        return Some(jerk);
    }
    if reaches_floor {
        return None;
    }
    let bracket = speed_squared
        .scale(Ball::exact(sigma))
        .add(&kappa.mul(a).scale(Ball::exact(2.0)));
    Some(
        kappa
            .mul(&speed_squared)
            .mul(v)
            .mul(&bracket)
            .mul(&a.reciprocal()?)
            .scale(Ball::exact(-1.0)),
    )
}

/// A model of `√argument`, or — where the argument's range reaches zero,
/// or the root's model is wider than that range — an affine model spanning
/// the root's range in its own variable, flagged so what is built on it
/// collapses to its range.
fn root_of(argument: &TaylorModel) -> (TaylorModel, bool) {
    let (centre, spread) = argument.spread();
    let lo = (centre - spread).max(0.0).sqrt();
    let hi = taylor::upper(taylor::upper(centre + spread).max(0.0).sqrt());
    let half_range = taylor::upper(0.5 * (hi - lo));
    match argument.sqrt() {
        Some(root) if root.rad() <= half_range => (root, false),
        _ => (
            TaylorModel::affine(Ball::rounded(0.5 * (lo + hi), 1), half_range),
            true,
        ),
    }
}

/// A constant model of `model`'s range.
fn collapsed(model: &TaylorModel) -> TaylorModel {
    let (centre, spread) = model.spread();
    TaylorModel::constant(Ball {
        mid: centre,
        rad: spread,
    })
}

/// One axis read window by window, its formulas gathered once: the
/// models of a window one formula covers, and the times its formula
/// changes at.
pub(crate) enum AxisWindows<'a> {
    /// Polynomial pieces, each with the base position it is relative to.
    Pieces(Vec<(f64, nurbs::bezier::BezierPiece)>),
    Analytic {
        span: &'a AnalyticMoveSpan,
        reading: AxisReading<'a>,
        /// Where a variable surface changes its formula along the path.
        surface_seams: Vec<f64>,
    },
    Hold(f64),
    /// A buzz's carrier, with its envelope's stretches.
    Buzz {
        base: f64,
        sign: f64,
        profile: &'a BuzzProfile,
        envelopes: Vec<(f64, f64, [f64; 2])>,
    },
}

impl<'a> AxisWindows<'a> {
    /// `None` for an axis with no formula the windows can read: an analytic
    /// axis without a law.
    pub(crate) fn of(axis: &'a ContinuousAxis) -> Option<Self> {
        Some(match axis {
            ContinuousAxis::Spline(curve) => Self::Pieces(
                extract_bezier_pieces(curve)
                    .into_iter()
                    .map(|piece| (0.0, piece))
                    .collect(),
            ),
            ContinuousAxis::RelativeSpline {
                base_position,
                curve,
            } => Self::Pieces(
                extract_bezier_pieces(curve)
                    .into_iter()
                    .map(|piece| (*base_position, piece))
                    .collect(),
            ),
            ContinuousAxis::PiecewiseRelativeSpline(pieces) => Self::Pieces(
                pieces
                    .iter()
                    .flat_map(|relative| {
                        extract_bezier_pieces(&relative.curve)
                            .into_iter()
                            .map(|piece| (relative.base_position, piece))
                    })
                    .collect(),
            ),
            ContinuousAxis::Analytic { span, axis } => match AxisLaw::of(span, *axis)? {
                (AxisLaw::Constant(position), offset) => Self::Hold(position + offset),
                (law, offset) => Self::Analytic {
                    surface_seams: match law {
                        AxisLaw::Warped { .. } => {
                            let mut seams = span.surface_seam_times()?;
                            seams.sort_by(f64::total_cmp);
                            seams
                        }
                        _ => Vec::new(),
                    },
                    span,
                    reading: AxisReading { law, offset },
                },
            },
            ContinuousAxis::Hold { position, .. } => Self::Hold(*position),
            ContinuousAxis::Nudge(profile) => Self::Pieces(
                nudge_pieces(profile)
                    .into_iter()
                    .map(|piece| (0.0, piece))
                    .collect(),
            ),
            ContinuousAxis::Buzz {
                base_position,
                sign,
                profile,
            } => Self::Buzz {
                base: *base_position,
                sign: *sign,
                profile,
                envelopes: profile.envelope_pieces(),
            },
        })
    }

    /// `[p, v, a, j]` on `window` where one formula covers it — `j` only
    /// `with_jerk`, else unbounded —; `None` past a formula's end, or for a
    /// jerk where the law has none, at a cap departure's square root.
    pub(crate) fn models(&self, window: Window, with_jerk: bool) -> Option<[TaylorModel; 4]> {
        self.models_from(window, with_jerk, 0)
    }

    /// `[v, a, j]` on `window`, as [`Self::models`] reads them, the
    /// position left unmodelled where the formula allows.
    pub(crate) fn motion_models(
        &self,
        window: Window,
        with_jerk: bool,
    ) -> Option<[TaylorModel; 3]> {
        let [_, v, a, j] = self.models_from(window, with_jerk, 1)?;
        Some([v, a, j])
    }

    /// [`Self::models`] with the polynomial pieces' orders below
    /// `lowest_order` left unbounded.
    fn models_from(
        &self,
        window: Window,
        with_jerk: bool,
        lowest_order: usize,
    ) -> Option<[TaylorModel; 4]> {
        let covers = |lo: f64, hi: f64| {
            let slack = 1e-9 * (hi - lo).max(1.0);
            window.lo >= lo - slack && window.hi <= hi + slack
        };
        match self {
            Self::Pieces(pieces) => {
                let index = pieces
                    .partition_point(|(_, piece)| piece.u_end < window.center)
                    .min(pieces.len().checked_sub(1)?);
                let (base, piece) = &pieces[index];
                if !covers(piece.u_start, piece.u_end) {
                    return None;
                }
                let tau = window.time_since(piece.u_start);
                let [p, v, a, j] = [0, 1, 2, 3].map(|order| {
                    if order < lowest_order {
                        TaylorModel::UNBOUNDED
                    } else {
                        derivative_of(&piece.coeffs, order, &tau)
                    }
                });
                Some([p.add_ball(Ball::exact(*base)), v, a, j])
            }
            Self::Analytic {
                span,
                reading,
                surface_seams,
            } => {
                let next_seam = surface_seams.partition_point(|&seam| seam <= window.center);
                let seam_before = next_seam
                    .checked_sub(1)
                    .map_or(span.t_start, |index| surface_seams[index]);
                let seam_after = surface_seams.get(next_seam).copied().unwrap_or(span.t_end);
                if !covers(seam_before, seam_after) {
                    return None;
                }
                let phase = span
                    .phases
                    .iter()
                    .find(|phase| window.center <= span.t_start + phase.end_time())
                    .or_else(|| span.phases.last())?;
                let origin = span.t_start + phase.t0;
                if !covers(origin, span.t_start + phase.end_time()) {
                    return None;
                }
                let tau = window.time_since(origin);
                let (state, tangential_jerk) = match phase.law {
                    ScalarLaw::ConstAccel { a0 } => (
                        Tangential {
                            s: polynomial_of_balls(
                                [
                                    Ball::rounded(phase.s0 - span.source_distance_origin, 1),
                                    Ball::exact(phase.v0),
                                    Ball::exact(0.5 * a0),
                                ]
                                .into_iter(),
                                &tau,
                            ),
                            v: polynomial_of_balls(
                                [Ball::exact(phase.v0), Ball::exact(a0)].into_iter(),
                                &tau,
                            ),
                            a: TaylorModel::constant(Ball::exact(a0)),
                        },
                        TaylorModel::constant(Ball::exact(0.0)),
                    ),
                    ScalarLaw::DiskRail { kappa0, sigma, .. } => {
                        let centre = window.center - origin;
                        let pair = phase.rail_pairs(centre, centre).next()?;
                        if !covers(origin + pair.t_lo, origin + pair.t_hi) {
                            return None;
                        }
                        let state =
                            PairInterpolant::new(&pair, phase.s0 - span.source_distance_origin)
                                .state(&tau);
                        let jerk = if with_jerk {
                            let kappa = state
                                .s
                                .add_ball(Ball::rounded(span.source_distance_origin - phase.s0, 1))
                                .scale(Ball::exact(sigma))
                                .add_ball(Ball::exact(kappa0));
                            rail_law_jerk(&kappa, sigma, &state)?
                        } else {
                            TaylorModel::UNBOUNDED
                        };
                        (state, jerk)
                    }
                };
                let reading_window = WindowReading {
                    state,
                    trig: std::cell::OnceCell::new(),
                    spread_trig: std::cell::OnceCell::new(),
                };
                let [p, v, a] = reading.models(&reading_window);
                let j = if with_jerk {
                    reading.law.jerk(&reading_window, &tangential_jerk)?
                } else {
                    TaylorModel::UNBOUNDED
                };
                Some([p, v, a, j])
            }
            Self::Hold(position) => Some([
                TaylorModel::constant(Ball::exact(*position)),
                TaylorModel::constant(Ball::exact(0.0)),
                TaylorModel::constant(Ball::exact(0.0)),
                TaylorModel::constant(Ball::exact(0.0)),
            ]),
            Self::Buzz {
                base,
                sign,
                profile,
                envelopes,
            } => {
                let &(_, _, envelope) = envelopes
                    .iter()
                    .find(|&&(start, end, _)| covers(start, end))?;
                let [p, v, a, j] = buzz_models(profile, envelope, window, with_jerk);
                let sign = Ball::exact(*sign);
                Some([
                    p.scale(sign).add_ball(Ball::exact(*base)),
                    v.scale(sign),
                    a.scale(sign),
                    j.scale(sign),
                ])
            }
        }
    }

    /// The curvature-cap departures of an analytic axis's rails reaching
    /// into `(lo, hi)`, where its tangential acceleration rises as a square
    /// root of the time.
    pub(crate) fn square_root_onsets(&self, lo: f64, hi: f64) -> Vec<SquareRootOnset> {
        let Self::Analytic { span, .. } = self else {
            return Vec::new();
        };
        span.phases
            .iter()
            .filter(|phase| matches!(phase.law, ScalarLaw::DiskRail { .. }))
            .flat_map(|phase| {
                let origin = span.t_start + phase.t0;
                phase
                    .rail_pairs(lo - origin, hi - origin)
                    .filter_map(move |pair| SquareRootOnset::of(&pair, origin))
            })
            .collect()
    }

    /// The times inside `(lo, hi)` at which the axis's formula changes.
    pub(crate) fn breaks(&self, lo: f64, hi: f64) -> Vec<f64> {
        let inside = |t: &f64| *t > lo && *t < hi;
        match self {
            Self::Pieces(pieces) => pieces
                .iter()
                .map(|(_, piece)| piece.u_end)
                .filter(inside)
                .collect(),
            Self::Analytic {
                span,
                surface_seams,
                ..
            } => span
                .phases
                .iter()
                .flat_map(|phase| {
                    let origin = span.t_start + phase.t0;
                    std::iter::once(span.t_start + phase.end_time()).chain(
                        phase
                            .rail_pairs(lo - origin, hi - origin)
                            .map(move |pair| origin + pair.t_hi),
                    )
                })
                .chain(surface_seams.iter().copied())
                .filter(inside)
                .collect(),
            Self::Hold(_) => Vec::new(),
            Self::Buzz { envelopes, .. } => envelopes
                .iter()
                .flat_map(|&(start, end, _)| [start, end])
                .filter(inside)
                .collect(),
        }
    }
}

#[cfg(test)]
#[path = "shaper_axis_enclosure_tests.rs"]
mod tests;
