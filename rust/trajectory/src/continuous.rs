use std::f64::consts::PI;
use std::sync::Arc;

use geometry::path::lowering::PositionProfile;
use geometry::path::Segment;
use geometry::{FollowerDemand, LawSegment, Move, ScalarLaw, SurfaceTransform};
use nurbs::ScalarNurbs;
use thiserror::Error;

mod profiles;
pub use profiles::{BuzzProfile, NudgeProfile, ProfileError, ProfileSample};

#[cfg(test)]
mod tests;

pub const MAX_SPAN_SECS: f64 = 0.025;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pva {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pvaj {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub jerk: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PvaBounds {
    pub velocity_min: f64,
    pub velocity_max: f64,
    pub acceleration_abs_max: f64,
    /// Velocity has no jump inside the interval, so `acceleration_abs_max`
    /// bounds how far it can drift from either endpoint's velocity.
    pub velocity_continuous: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceMode {
    None,
    Constant(f64),
    Variable(Arc<SurfaceTransform>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct AnalyticMoveSpan {
    pub source: Move,
    pub phases: Arc<[LawSegment]>,
    pub source_distance_origin: f64,
    pub t_start: f64,
    pub t_end: f64,
    pub axis_start_positions: Arc<[f64]>,
    pub surface: SurfaceMode,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RelativeSplinePiece {
    pub base_position: f64,
    pub curve: Arc<ScalarNurbs>,
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ContinuousAxis {
    Analytic {
        span: Arc<AnalyticMoveSpan>,
        axis: usize,
    },
    Spline(Arc<ScalarNurbs>),
    RelativeSpline {
        base_position: f64,
        curve: Arc<ScalarNurbs>,
    },
    PiecewiseRelativeSpline(Arc<[RelativeSplinePiece]>),
    Hold {
        position: f64,
        t_start: f64,
        t_end: f64,
    },
    Nudge(NudgeProfile),
    Buzz {
        base_position: f64,
        sign: f64,
        profile: Arc<BuzzProfile>,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContinuousSegment {
    pub axes: Arc<[ContinuousAxis]>,
    pub followers: Arc<[FollowerDemand]>,
    pub spatial_path: bool,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    pub source_line: u32,
    pub rest_at_end: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MotorTerm {
    pub source_axis: usize,
    pub axis: ContinuousAxis,
    pub scale: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum MotorGroup {
    Analytic {
        span: Arc<AnalyticMoveSpan>,
        terms: Arc<[MotorTerm]>,
    },
    Spline {
        curve: Arc<ScalarNurbs>,
        summed_scale: f64,
    },
    RelativeSpline {
        curve: Arc<ScalarNurbs>,
        base_position: f64,
        summed_scale: f64,
    },
    Independent(MotorTerm),
}

#[derive(Debug, Clone, PartialEq)]
pub struct MotorSpan {
    pub groups: Arc<[MotorGroup]>,
    pub breakpoints: Arc<[f64]>,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    pub source_line: u32,
    pub is_explicit_hold: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ClockedMotorSpan {
    pub signal: Arc<MotorSpan>,
    pub stream_t_start: f64,
    pub stream_t_end: f64,
    pub start_host: f64,
    pub end_host: f64,
    pub start_clock_exact: f64,
    pub start_clock: u64,
    pub end_clock: u64,
    pub clock_freq_hz: f64,
}

#[derive(Debug, Clone, Copy, Error, PartialEq)]
pub enum ContinuousError {
    #[error("non-finite continuous evaluation at stream time {t}")]
    NonFinite { t: f64 },
    #[error("non-finite continuous evaluation of source axis {source_axis} at stream time {t}")]
    NonFiniteEvaluation { source_axis: usize, t: f64 },
    #[error("stream time {t} is outside [{t_start}, {t_end}]")]
    TimeOutsideSpan { t: f64, t_start: f64, t_end: f64 },
    #[error("source axis {axis} is not present")]
    AxisOutsideMove { axis: usize },
    #[error("a variable surface must be replaced by a spline before dispatch")]
    VariableSurfaceBeforeDispatch,
    #[error("clock {clock} is outside [{start_clock}, {end_clock}]")]
    ClockOutsideSpan {
        clock: u64,
        start_clock: u64,
        end_clock: u64,
    },
    #[error("invalid continuous span: {reason}")]
    InvalidSpan { reason: &'static str },
    #[error("analytic phase gap from {previous_end} to {next_start}")]
    PhaseGap { previous_end: f64, next_start: f64 },
    #[error("analytic phase overlap from {next_start} to {previous_end}")]
    PhaseOverlap { previous_end: f64, next_start: f64 },
    #[error("analytic phase endpoint mismatch: expected {expected}, got {actual}")]
    PhaseEndpointMismatch { expected: f64, actual: f64 },
    #[error("analytic phase has negative velocity {velocity}")]
    NegativeVelocity { velocity: f64 },
    #[error("relative spline piece gap from {previous_end} to {next_start}")]
    PieceGap { previous_end: f64, next_start: f64 },
    #[error("relative spline piece overlap from {next_start} to {previous_end}")]
    PieceOverlap { previous_end: f64, next_start: f64 },
}

impl AnalyticMoveSpan {
    pub fn try_new(
        source: Move,
        phases: Arc<[LawSegment]>,
        source_distance_origin: f64,
        t_start: f64,
        t_end: f64,
        axis_start_positions: Arc<[f64]>,
        surface: SurfaceMode,
    ) -> Result<Self, ContinuousError> {
        if !t_start.is_finite() || !t_end.is_finite() || t_end <= t_start {
            return Err(ContinuousError::InvalidSpan {
                reason: "analytic time range must be finite and positive",
            });
        }
        if phases.is_empty()
            || !source_distance_origin.is_finite()
            || axis_start_positions.iter().any(|value| !value.is_finite())
        {
            return Err(ContinuousError::InvalidSpan {
                reason: "analytic span requires finite axis starts, distance origin, and phases",
            });
        }
        if let SurfaceMode::Constant(offset) = &surface {
            if !offset.is_finite() {
                return Err(ContinuousError::InvalidSpan {
                    reason: "constant surface offset must be finite",
                });
            }
        }
        if phases.iter().any(|segment| {
            ![segment.t0, segment.dt, segment.s0, segment.v0]
                .into_iter()
                .all(f64::is_finite)
                || !match segment.law {
                    ScalarLaw::ConstAccel { a0 } => a0.is_finite(),
                    ScalarLaw::DiskRail {
                        accel,
                        kappa0,
                        sigma,
                        ..
                    } => accel.is_finite() && kappa0.is_finite() && sigma.is_finite(),
                }
                || segment.dt <= 0.0
        }) {
            return Err(ContinuousError::InvalidSpan {
                reason: "analytic phases must be finite and positive-duration",
            });
        }
        let duration = t_end - t_start;
        let time_scale = phases.iter().fold(duration.abs(), |scale, phase| {
            scale.max(phase.t0.abs()).max(phase.end_time().abs())
        });
        let time_slack = scale_aware_slack(time_scale);
        validate_ordered_coverage(
            0.0,
            duration,
            phases
                .iter()
                .map(|phase| (phase.t0, phase.end_time(), time_slack)),
            time_slack,
        )?;
        let length = source.segment.s_len();
        let source_distance_end = source_distance_origin + length;
        let distance_scale = phases.iter().fold(
            source_distance_origin.abs().max(source_distance_end.abs()),
            |scale, phase| scale.max(phase.s0.abs()).max(phase.end_distance().abs()),
        );
        let distance_slack =
            phase_distance_solver_slack(&phases, distance_scale, t_start.abs().max(t_end.abs()));
        validate_ordered_coverage(
            source_distance_origin,
            source_distance_end,
            phases.iter().enumerate().map(|(index, phase)| {
                let joint_slack = match index.checked_sub(1).map(|previous| &phases[previous]) {
                    Some(previous) => phase_joint_distance_slack(previous, phase, distance_slack),
                    None => distance_slack,
                };
                (phase.s0, phase.end_distance(), joint_slack)
            }),
            distance_slack,
        )?;
        for segment in phases.iter() {
            let minimum_velocity = segment.min_velocity();
            let velocity_slack = phase_velocity_solver_slack(segment, distance_slack);
            if minimum_velocity < -velocity_slack {
                return Err(ContinuousError::NegativeVelocity {
                    velocity: minimum_velocity,
                });
            }
        }
        Ok(Self {
            source,
            phases,
            source_distance_origin,
            t_start,
            t_end,
            axis_start_positions,
            surface,
        })
    }

    pub fn eval_axis(&self, axis: usize, t: f64) -> Result<Pva, ContinuousError> {
        if axis == 2 {
            if let SurfaceMode::Variable(surface) = &self.surface {
                return self.eval_warped_z(surface, t);
            }
        }
        let exact = self.eval_axis_pvaj(axis, t)?;
        Ok(Pva {
            position: exact.position,
            velocity: exact.velocity,
            acceleration: exact.acceleration,
        })
    }

    pub fn eval_axis_pvaj(&self, axis: usize, t: f64) -> Result<Pvaj, ContinuousError> {
        check_time(t, self.t_start, self.t_end)?;
        if axis == 2 && matches!(&self.surface, SurfaceMode::Variable(_)) {
            return Err(ContinuousError::VariableSurfaceBeforeDispatch);
        }
        let (s, velocity, acceleration, jerk) = self.tangential_state(t);
        let length = self.source.segment.s_len();
        let result = if axis < 3 {
            let segment = self
                .source
                .segment
                .spatial
                .as_ref()
                .ok_or(ContinuousError::AxisOutsideMove { axis })?;
            let heading = segment.heading_at(s);
            let dheading = segment.dheading_ds(s);
            let d2heading = segment.d2heading_ds2(s);
            let mut result = Pvaj {
                position: segment.point_at(s)[axis],
                velocity: velocity * heading[axis],
                acceleration: acceleration * heading[axis] + velocity * velocity * dheading[axis],
                jerk: jerk * heading[axis]
                    + 3.0 * velocity * acceleration * dheading[axis]
                    + velocity * velocity * velocity * d2heading[axis],
            };
            if axis == 2 {
                if let SurfaceMode::Constant(offset) = &self.surface {
                    result.position += offset;
                }
            }
            result
        } else {
            match analytic_follower(&self.source, axis) {
                Some(follower) => {
                    let start = *self
                        .axis_start_positions
                        .get(follower.axis_index)
                        .ok_or(ContinuousError::AxisOutsideMove { axis })?;
                    let ratio = follower.ratio_at(s, length);
                    let slope = follower.ratio_slope(length);
                    Pvaj {
                        position: start + follower.offset_at(s, length),
                        velocity: ratio * velocity,
                        acceleration: ratio * acceleration + slope * velocity * velocity,
                        jerk: ratio * jerk + 3.0 * slope * velocity * acceleration,
                    }
                }
                None => Pvaj {
                    position: *self
                        .axis_start_positions
                        .get(axis)
                        .ok_or(ContinuousError::AxisOutsideMove { axis })?,
                    velocity: 0.0,
                    acceleration: 0.0,
                    jerk: 0.0,
                },
            }
        };
        finite_pvaj(result, t)
    }

    fn tangential_state(&self, t: f64) -> (f64, f64, f64, f64) {
        let local_t = (t - self.t_start).clamp(0.0, self.t_end - self.t_start);
        let segment = active_phase(&self.phases, local_t);
        let (phase_s, velocity, acceleration) = segment.state_at(local_t);
        let jerk = match segment.law {
            ScalarLaw::ConstAccel { .. } => 0.0,
            ScalarLaw::DiskRail { kappa0, sigma, .. } => {
                if acceleration.abs() < 1e-15 {
                    0.0
                } else {
                    let ds = phase_s - segment.s0;
                    let kappa = kappa0 + sigma * ds;
                    -kappa
                        * velocity.powi(3)
                        * (sigma * velocity * velocity + 2.0 * kappa * acceleration)
                        / acceleration
                }
            }
        };
        (
            phase_s - self.source_distance_origin,
            velocity,
            acceleration,
            jerk,
        )
    }

    fn eval_warped_z(&self, surface: &SurfaceTransform, t: f64) -> Result<Pva, ContinuousError> {
        check_time(t, self.t_start, self.t_end)?;
        let (s, velocity, acceleration, _) = self.tangential_state(t);
        let segment = self
            .source
            .segment
            .spatial
            .as_ref()
            .ok_or(ContinuousError::AxisOutsideMove { axis: 2 })?;
        let point = segment.point_at(s);
        let heading = segment.heading_at(s);
        let dheading = segment.dheading_ds(s);
        let warp = surface.warp(point[0], point[1], point[2]);
        let path_velocity = heading.map(|component| velocity * component);
        let path_acceleration: [f64; 3] = std::array::from_fn(|component| {
            acceleration * heading[component] + velocity * velocity * dheading[component]
        });
        let result = Pva {
            position: point[2] + warp.w,
            velocity: velocity * heading[2]
                + warp.wx * path_velocity[0]
                + warp.wy * path_velocity[1]
                + warp.wz * path_velocity[2],
            acceleration: acceleration * heading[2]
                + velocity * velocity * dheading[2]
                + warp.wx * path_acceleration[0]
                + warp.wy * path_acceleration[1]
                + warp.wz * path_acceleration[2]
                + warp.wxx * path_velocity[0] * path_velocity[0]
                + 2.0 * warp.wxy * path_velocity[0] * path_velocity[1]
                + warp.wyy * path_velocity[1] * path_velocity[1]
                + 2.0 * warp.wxz * path_velocity[0] * path_velocity[2]
                + 2.0 * warp.wyz * path_velocity[1] * path_velocity[2],
        };
        finite_pva(result, t)
    }
}

impl ContinuousAxis {
    pub fn try_piecewise_relative_spline(
        pieces: Arc<[RelativeSplinePiece]>,
    ) -> Result<Self, ContinuousError> {
        validate_relative_pieces(&pieces)?;
        Ok(Self::PiecewiseRelativeSpline(pieces))
    }

    pub fn eval_pva(&self, t: f64) -> Result<Pva, ContinuousError> {
        if !t.is_finite() {
            return Err(ContinuousError::NonFinite { t });
        }
        let result = match self {
            Self::Analytic { span, axis } => span.eval_axis(*axis, t)?,
            Self::Spline(curve) => spline_pva(curve, t)?,
            Self::RelativeSpline {
                base_position,
                curve,
            } => {
                let mut value = spline_pva(curve, t)?;
                value.position += base_position;
                value
            }
            Self::PiecewiseRelativeSpline(pieces) => {
                let piece = owning_piece(pieces, t)?;
                let mut value = spline_pva(&piece.curve, t)?;
                value.position += piece.base_position;
                value
            }
            Self::Hold {
                position,
                t_start,
                t_end,
            } => {
                checked_clamped_time(t, *t_start, *t_end)?;
                Pva {
                    position: *position,
                    velocity: 0.0,
                    acceleration: 0.0,
                }
            }
            Self::Nudge(profile) => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                let value = profile.eval(t);
                Pva {
                    position: value.position,
                    velocity: value.velocity,
                    acceleration: value.acceleration,
                }
            }
            Self::Buzz {
                base_position,
                sign,
                profile,
            } => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                let relative = profile.eval(t);
                Pva {
                    position: base_position + sign * relative.position,
                    velocity: sign * relative.velocity,
                    acceleration: sign * relative.acceleration,
                }
            }
        };
        finite_pva(result, t)
    }

    pub fn eval_pvaj(&self, t: f64) -> Result<Pvaj, ContinuousError> {
        if !t.is_finite() {
            return Err(ContinuousError::NonFinite { t });
        }
        let result = match self {
            Self::Analytic { span, axis } => span.eval_axis_pvaj(*axis, t)?,
            Self::Spline(curve) => spline_pvaj(curve, t)?,
            Self::RelativeSpline {
                base_position,
                curve,
            } => {
                let mut value = spline_pvaj(curve, t)?;
                value.position += base_position;
                value
            }
            Self::PiecewiseRelativeSpline(pieces) => {
                let piece = owning_piece(pieces, t)?;
                let mut value = spline_pvaj(&piece.curve, t)?;
                value.position += piece.base_position;
                value
            }
            Self::Hold {
                position,
                t_start,
                t_end,
            } => {
                checked_clamped_time(t, *t_start, *t_end)?;
                Pvaj {
                    position: *position,
                    velocity: 0.0,
                    acceleration: 0.0,
                    jerk: 0.0,
                }
            }
            Self::Nudge(profile) => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                let value = profile.eval(t);
                Pvaj {
                    position: value.position,
                    velocity: value.velocity,
                    acceleration: value.acceleration,
                    jerk: profile.jerk(t),
                }
            }
            Self::Buzz {
                base_position,
                sign,
                profile,
            } => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                let relative = profile.eval(t);
                Pvaj {
                    position: base_position + sign * relative.position,
                    velocity: sign * relative.velocity,
                    acceleration: sign * relative.acceleration,
                    jerk: sign * profile.jerk(t),
                }
            }
        };
        finite_pvaj(result, t)
    }

    pub fn position(&self, t: f64) -> Result<f64, ContinuousError> {
        if !t.is_finite() {
            return Err(ContinuousError::NonFinite { t });
        }
        let result = match self {
            Self::Analytic { span, axis } => span.eval_axis(*axis, t)?.position,
            Self::Spline(curve) => {
                let t = spline_evaluation_time(curve, t)?;
                nurbs::eval::eval(&curve.as_view(), t)
            }
            Self::RelativeSpline {
                base_position,
                curve,
            } => {
                let t = spline_evaluation_time(curve, t)?;
                base_position + nurbs::eval::eval(&curve.as_view(), t)
            }
            Self::PiecewiseRelativeSpline(pieces) => {
                let piece = owning_piece(pieces, t)?;
                let t = spline_evaluation_time(&piece.curve, t)?;
                piece.base_position + nurbs::eval::eval(&piece.curve.as_view(), t)
            }
            Self::Hold {
                position,
                t_start,
                t_end,
            } => {
                checked_clamped_time(t, *t_start, *t_end)?;
                *position
            }
            Self::Nudge(profile) => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                profile.eval(t).position
            }
            Self::Buzz {
                base_position,
                sign,
                profile,
            } => {
                let t = checked_clamped_time(t, profile.t_start(), profile.t_end())?;
                base_position + sign * profile.eval(t).position
            }
        };
        if result.is_finite() {
            Ok(result)
        } else {
            Err(ContinuousError::NonFinite { t })
        }
    }

    pub fn pva_bounds(&self, t0: f64, t1: f64) -> Result<PvaBounds, ContinuousError> {
        check_interval(self, t0, t1)?;
        match self {
            Self::Analytic { span, axis } => {
                analytic_group_bounds(span, std::iter::once((*axis, 1.0)), t0, t1)
            }
            Self::Spline(curve) => spline_bounds(curve, 1.0, t0, t1),
            Self::RelativeSpline { curve, .. } => spline_bounds(curve, 1.0, t0, t1),
            Self::PiecewiseRelativeSpline(pieces) => piecewise_relative_bounds(pieces, t0, t1),
            Self::Hold { .. } => Ok(zero_bounds()),
            Self::Nudge(profile) => Ok(profile_bounds(
                profile.velocity_bounds(),
                profile.acceleration_bounds(),
                !profile.velocity_step_inside(t0, t1),
            )),
            Self::Buzz { sign, profile, .. } => scale_bounds(
                profile_bounds(
                    profile.velocity_bounds(),
                    profile.acceleration_bounds(),
                    !profile.velocity_step_inside(t0, t1),
                ),
                *sign,
            ),
        }
    }

    pub fn domain(&self) -> (f64, f64) {
        match self {
            Self::Analytic { span, .. } => (span.t_start, span.t_end),
            Self::Spline(curve) => spline_domain(curve),
            Self::RelativeSpline { curve, .. } => spline_domain(curve),
            Self::PiecewiseRelativeSpline(pieces) => piecewise_relative_domain(pieces),
            Self::Hold { t_start, t_end, .. } => (*t_start, *t_end),
            Self::Nudge(profile) => (profile.t_start(), profile.t_end()),
            Self::Buzz { profile, .. } => (profile.t_start(), profile.t_end()),
        }
    }

    pub fn breakpoints(&self) -> Vec<f64> {
        let mut output = Vec::new();
        self.append_breakpoints(&mut output);
        output
    }

    pub fn append_breakpoints(&self, output: &mut Vec<f64>) {
        match self {
            Self::Analytic { span, .. } => {
                output.extend(span.phases.iter().map(|phase| span.t_start + phase.t0));
                output.extend(
                    span.phases
                        .iter()
                        .map(|phase| span.t_start + phase.end_time()),
                );
            }
            Self::Spline(curve) => output.extend_from_slice(curve.knots()),
            Self::RelativeSpline { curve, .. } => output.extend_from_slice(curve.knots()),
            Self::PiecewiseRelativeSpline(pieces) => {
                for piece in pieces.iter() {
                    output.extend([piece.t_start, piece.t_end]);
                    output.extend(
                        piece
                            .curve
                            .knots()
                            .iter()
                            .copied()
                            .filter(|knot| *knot > piece.t_start && *knot < piece.t_end),
                    );
                }
            }
            Self::Hold { t_start, t_end, .. } => output.extend([*t_start, *t_end]),
            Self::Nudge(profile) => output.extend_from_slice(profile.breakpoints()),
            Self::Buzz { profile, .. } => output.extend_from_slice(profile.breakpoints()),
        }
    }
}

impl ContinuousSegment {
    pub fn eval_axis(&self, axis: usize, t: f64) -> Result<Pva, ContinuousError> {
        let t = checked_clamped_time(t, self.t_start, self.t_end)
            .map_err(|error| with_source_axis(error, axis, t))?;
        let source = self
            .axes
            .get(axis)
            .ok_or(ContinuousError::AxisOutsideMove { axis })?;
        source
            .eval_pva(t)
            .map_err(|error| with_source_axis(error, axis, t))
    }

    pub fn eval_axis_pvaj(&self, axis: usize, t: f64) -> Result<Pvaj, ContinuousError> {
        let t = checked_clamped_time(t, self.t_start, self.t_end)
            .map_err(|error| with_source_axis(error, axis, t))?;
        let source = self
            .axes
            .get(axis)
            .ok_or(ContinuousError::AxisOutsideMove { axis })?;
        source
            .eval_pvaj(t)
            .map_err(|error| with_source_axis(error, axis, t))
    }

    pub fn breakpoints(&self) -> Vec<f64> {
        let mut output = vec![self.t_start, self.t_end];
        for axis in self.axes.iter() {
            axis.append_breakpoints(&mut output);
        }
        output.retain(|value| *value >= self.t_start && *value <= self.t_end);
        output.sort_by(f64::total_cmp);
        output.dedup();
        output
    }
}

impl MotorSpan {
    pub fn try_new(
        groups: Arc<[MotorGroup]>,
        t_start: f64,
        t_end: f64,
        motor_mask: u8,
        source_line: u32,
        is_explicit_hold: bool,
    ) -> Result<Self, ContinuousError> {
        if !t_start.is_finite() || !t_end.is_finite() || t_end <= t_start {
            return Err(ContinuousError::InvalidSpan {
                reason: "motor time range must be finite and positive",
            });
        }
        if groups.is_empty() {
            return Err(ContinuousError::InvalidSpan {
                reason: "motor span requires at least one group",
            });
        }
        for group in groups.iter() {
            group.validate_for_dispatch(t_start, t_end)?;
        }
        let mut breakpoints = Vec::new();
        breakpoints.extend([t_start, t_end]);
        for group in groups.iter() {
            group.append_breakpoints(&mut breakpoints);
        }
        breakpoints.retain(|value| value.is_finite() && *value >= t_start && *value <= t_end);
        breakpoints.sort_by(f64::total_cmp);
        breakpoints.dedup_by(|left, right| *left == *right);
        Ok(Self {
            groups,
            breakpoints: breakpoints.into(),
            t_start,
            t_end,
            motor_mask,
            source_line,
            is_explicit_hold,
        })
    }

    pub fn first_source_axis(&self) -> usize {
        self.groups
            .iter()
            .find_map(MotorGroup::first_source_axis)
            .unwrap_or(0)
    }

    pub fn eval_pva(&self, t: f64) -> Result<Pva, ContinuousError> {
        let source_axis = self.first_source_axis();
        if !t.is_finite() {
            return Err(ContinuousError::NonFiniteEvaluation { source_axis, t });
        }
        let t = checked_clamped_time(t, self.t_start, self.t_end)?;
        let mut result = Pva {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        };
        for group in self.groups.iter() {
            let value = group
                .eval_pva(t)
                .map_err(|(error, source_axis)| with_source_axis(error, source_axis, t))?;
            result.position += value.position;
            result.velocity += value.velocity;
            result.acceleration += value.acceleration;
        }
        if [result.position, result.velocity, result.acceleration]
            .into_iter()
            .all(f64::is_finite)
        {
            Ok(result)
        } else {
            Err(ContinuousError::NonFiniteEvaluation { source_axis, t })
        }
    }

    pub fn position(&self, t: f64) -> Result<f64, ContinuousError> {
        let source_axis = self.first_source_axis();
        if !t.is_finite() {
            return Err(ContinuousError::NonFiniteEvaluation { source_axis, t });
        }
        let t = checked_clamped_time(t, self.t_start, self.t_end)?;
        let mut result = 0.0;
        for group in self.groups.iter() {
            result += group
                .eval_position(t)
                .map_err(|(error, source_axis)| with_source_axis(error, source_axis, t))?;
        }
        if result.is_finite() {
            Ok(result)
        } else {
            Err(ContinuousError::NonFiniteEvaluation { source_axis, t })
        }
    }

    pub fn pva_bounds(&self, t0: f64, t1: f64) -> Result<PvaBounds, ContinuousError> {
        let t0 = checked_clamped_time(t0, self.t_start, self.t_end)?;
        let t1 = checked_clamped_time(t1, self.t_start, self.t_end)?;
        if t1 < t0 {
            return Err(ContinuousError::InvalidSpan {
                reason: "bounds interval is reversed",
            });
        }
        let mut result = zero_bounds();
        for group in self.groups.iter() {
            let bounds = group.pva_bounds(t0, t1)?;
            result.velocity_min += bounds.velocity_min;
            result.velocity_max += bounds.velocity_max;
            result.acceleration_abs_max += bounds.acceleration_abs_max;
            result.velocity_continuous &= bounds.velocity_continuous;
        }
        if result.velocity_continuous {
            let radius = result.acceleration_abs_max * (t1 - t0);
            let start_velocity = self.eval_pva(next_toward(t0, t1))?.velocity;
            let end_velocity = self.eval_pva(next_toward(t1, t0))?.velocity;
            result.velocity_min = result
                .velocity_min
                .max(start_velocity - radius)
                .max(end_velocity - radius);
            result.velocity_max = result
                .velocity_max
                .min(start_velocity + radius)
                .min(end_velocity + radius);
        }
        Ok(result)
    }
}

impl MotorGroup {
    fn eval_pva(&self, t: f64) -> Result<Pva, (ContinuousError, usize)> {
        match self {
            Self::Analytic { span, terms } => analytic_group_pva(span, terms, t),
            Self::Spline {
                curve,
                summed_scale,
            } => {
                let axis = ContinuousAxis::Spline(Arc::clone(curve));
                axis.eval_pva(t)
                    .map(|value| scale_pva(value, *summed_scale))
                    .map_err(|error| (error, 0))
            }
            Self::RelativeSpline {
                curve,
                base_position,
                summed_scale,
            } => {
                let axis = ContinuousAxis::RelativeSpline {
                    base_position: *base_position,
                    curve: Arc::clone(curve),
                };
                axis.eval_pva(t)
                    .map(|value| scale_pva(value, *summed_scale))
                    .map_err(|error| (error, 0))
            }
            Self::Independent(term) => term
                .axis
                .eval_pva(t)
                .map(|value| scale_pva(value, term.scale))
                .map_err(|error| (error, term.source_axis)),
        }
    }

    fn eval_position(&self, t: f64) -> Result<f64, (ContinuousError, usize)> {
        match self {
            Self::Analytic { span, terms } => {
                analytic_group_pva(span, terms, t).map(|value| value.position)
            }
            Self::Spline {
                curve,
                summed_scale,
            } => {
                let t = spline_evaluation_time(curve, t).map_err(|error| (error, 0))?;
                Ok(summed_scale * nurbs::eval::eval(&curve.as_view(), t))
            }
            Self::RelativeSpline {
                curve,
                base_position,
                summed_scale,
            } => {
                let t = spline_evaluation_time(curve, t).map_err(|error| (error, 0))?;
                Ok(summed_scale * (base_position + nurbs::eval::eval(&curve.as_view(), t)))
            }
            Self::Independent(term) => term
                .axis
                .position(t)
                .map(|value| value * term.scale)
                .map_err(|error| (error, term.source_axis)),
        }
    }

    fn pva_bounds(&self, t0: f64, t1: f64) -> Result<PvaBounds, ContinuousError> {
        match self {
            Self::Analytic { span, terms } => analytic_group_bounds(
                span,
                terms.iter().map(|term| (term.source_axis, term.scale)),
                t0,
                t1,
            ),
            Self::Spline {
                curve,
                summed_scale,
            } => spline_bounds(curve, *summed_scale, t0, t1),
            Self::RelativeSpline {
                curve,
                summed_scale,
                ..
            } => spline_bounds(curve, *summed_scale, t0, t1),
            Self::Independent(term) => scale_bounds(term.axis.pva_bounds(t0, t1)?, term.scale),
        }
    }

    fn append_breakpoints(&self, output: &mut Vec<f64>) {
        match self {
            Self::Analytic { span, terms } => {
                output.extend(
                    span.phases.iter().flat_map(|phase| {
                        [span.t_start + phase.t0, span.t_start + phase.end_time()]
                    }),
                );
                for term in terms.iter() {
                    term.axis.append_breakpoints(output);
                }
            }
            Self::Spline { curve, .. } => output.extend_from_slice(curve.knots()),
            Self::RelativeSpline { curve, .. } => output.extend_from_slice(curve.knots()),
            Self::Independent(term) => term.axis.append_breakpoints(output),
        }
    }

    fn first_source_axis(&self) -> Option<usize> {
        match self {
            Self::Analytic { terms, .. } => terms.first().map(|term| term.source_axis),
            Self::Spline { .. } => None,
            Self::RelativeSpline { .. } => None,
            Self::Independent(term) => Some(term.source_axis),
        }
    }
    fn validate_for_dispatch(&self, t_start: f64, t_end: f64) -> Result<(), ContinuousError> {
        match self {
            Self::Analytic { span, terms } => {
                if terms.is_empty() {
                    return Err(ContinuousError::InvalidSpan {
                        reason: "analytic motor group requires terms",
                    });
                }
                if matches!(&span.surface, SurfaceMode::Variable(_)) {
                    return Err(ContinuousError::VariableSurfaceBeforeDispatch);
                }
                checked_clamped_time(t_start, span.t_start, span.t_end)?;
                checked_clamped_time(t_end, span.t_start, span.t_end)?;
                for term in terms.iter() {
                    if !term.scale.is_finite() {
                        return Err(ContinuousError::InvalidSpan {
                            reason: "motor scale must be finite",
                        });
                    }
                    match &term.axis {
                        ContinuousAxis::Analytic {
                            span: term_span,
                            axis,
                        } if Arc::ptr_eq(span, term_span) && *axis == term.source_axis => {}
                        _ => {
                            return Err(ContinuousError::InvalidSpan {
                                reason: "analytic group terms must share its span",
                            })
                        }
                    }
                }
            }
            Self::Spline {
                curve,
                summed_scale,
            } => {
                if !summed_scale.is_finite() {
                    return Err(ContinuousError::InvalidSpan {
                        reason: "summed spline scale must be finite",
                    });
                }
                validate_spline_control_points(curve)?;
                let (start, end) = spline_domain(curve);
                checked_clamped_time(t_start, start, end)?;
                checked_clamped_time(t_end, start, end)?;
            }
            Self::RelativeSpline {
                curve,
                base_position,
                summed_scale,
            } => {
                if !summed_scale.is_finite() || !base_position.is_finite() {
                    return Err(ContinuousError::InvalidSpan {
                        reason: "relative spline base and summed scale must be finite",
                    });
                }
                validate_spline_control_points(curve)?;
                let (start, end) = spline_domain(curve);
                checked_clamped_time(t_start, start, end)?;
                checked_clamped_time(t_end, start, end)?;
            }
            Self::Independent(term) => {
                if !term.scale.is_finite() {
                    return Err(ContinuousError::InvalidSpan {
                        reason: "motor scale must be finite",
                    });
                }
                if let ContinuousAxis::Analytic { span, .. } = &term.axis {
                    if matches!(&span.surface, SurfaceMode::Variable(_)) {
                        return Err(ContinuousError::VariableSurfaceBeforeDispatch);
                    }
                }
                validate_axis_for_dispatch(&term.axis)?;
                let (start, end) = term.axis.domain();
                checked_clamped_time(t_start, start, end)?;
                checked_clamped_time(t_end, start, end)?;
            }
        }
        Ok(())
    }
}
fn validate_axis_for_dispatch(axis: &ContinuousAxis) -> Result<(), ContinuousError> {
    match axis {
        ContinuousAxis::Analytic { span, .. } => {
            if matches!(&span.surface, SurfaceMode::Variable(_)) {
                return Err(ContinuousError::VariableSurfaceBeforeDispatch);
            }
        }
        ContinuousAxis::Spline(curve) => {
            if curve
                .control_points()
                .iter()
                .any(|value| !value.is_finite())
            {
                return Err(ContinuousError::InvalidSpan {
                    reason: "spline control points must be finite",
                });
            }
        }
        ContinuousAxis::RelativeSpline {
            base_position,
            curve,
        } => {
            if !base_position.is_finite()
                || curve
                    .control_points()
                    .iter()
                    .any(|value| !value.is_finite())
            {
                return Err(ContinuousError::InvalidSpan {
                    reason: "relative spline base and control points must be finite",
                });
            }
        }
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => validate_relative_pieces(pieces)?,
        ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        } => {
            if !position.is_finite() {
                return Err(ContinuousError::InvalidSpan {
                    reason: "hold position must be finite",
                });
            }
            if !t_start.is_finite() || !t_end.is_finite() || t_end <= t_start {
                return Err(ContinuousError::InvalidSpan {
                    reason: "hold time range must be finite and positive",
                });
            }
        }
        ContinuousAxis::Nudge(_) => {}
        ContinuousAxis::Buzz {
            base_position,
            sign,
            ..
        } => {
            if !base_position.is_finite() || (*sign != -1.0 && *sign != 1.0) {
                return Err(ContinuousError::InvalidSpan {
                    reason: "buzz base must be finite and sign must be -1 or 1",
                });
            }
        }
    }
    Ok(())
}

impl ClockedMotorSpan {
    pub fn try_new(
        signal: Arc<MotorSpan>,
        stream_t_start: f64,
        stream_t_end: f64,
        start_host: f64,
        end_host: f64,
        start_clock_exact: f64,
        clock_freq_hz: f64,
    ) -> Result<Self, ContinuousError> {
        if ![
            stream_t_start,
            stream_t_end,
            start_host,
            end_host,
            start_clock_exact,
            clock_freq_hz,
        ]
        .into_iter()
        .all(f64::is_finite)
            || stream_t_end <= stream_t_start
            || clock_freq_hz <= 0.0
        {
            tracing::error!(
                subsystem = "motion",
                event = "clocked_view_rejected",
                stream_t_start,
                stream_t_end,
                start_host,
                end_host,
                start_clock_exact,
                clock_freq_hz,
                "clocked view rejected"
            );
            return Err(ContinuousError::InvalidSpan {
                reason: "clocked view requires finite positive ranges and frequency",
            });
        }
        checked_clamped_time(stream_t_start, signal.t_start, signal.t_end)?;
        checked_clamped_time(stream_t_end, signal.t_start, signal.t_end)?;
        let start_clock = rounded_clock(start_clock_exact)?;
        let end_clock =
            rounded_clock(start_clock_exact + (stream_t_end - stream_t_start) * clock_freq_hz)?;
        if end_clock <= start_clock {
            return Err(ContinuousError::InvalidSpan {
                reason: "positive-duration clocked view must span at least one clock",
            });
        }
        Ok(Self {
            signal,
            stream_t_start,
            stream_t_end,
            start_host,
            end_host,
            start_clock_exact,
            start_clock,
            end_clock,
            clock_freq_hz,
        })
    }

    pub fn clock_at_stream_time(&self, t: f64) -> Result<u64, ContinuousError> {
        let t = checked_clamped_time(t, self.stream_t_start, self.stream_t_end)?;
        rounded_clock(self.start_clock_exact + (t - self.stream_t_start) * self.clock_freq_hz)
    }

    pub fn stream_time_at_clock(&self, clock: u64) -> Result<f64, ContinuousError> {
        if clock < self.start_clock || clock > self.end_clock {
            return Err(ContinuousError::ClockOutsideSpan {
                clock,
                start_clock: self.start_clock,
                end_clock: self.end_clock,
            });
        }
        let t = self.stream_t_start + (clock as f64 - self.start_clock_exact) / self.clock_freq_hz;
        if clock == self.start_clock && t < self.stream_t_start {
            Ok(self.stream_t_start)
        } else if clock == self.end_clock && t > self.stream_t_end {
            Ok(self.stream_t_end)
        } else {
            checked_clamped_time(t, self.stream_t_start, self.stream_t_end)
        }
    }

    pub fn eval_at_clock(&self, clock: u64) -> Result<Pva, ContinuousError> {
        self.signal.eval_pva(self.stream_time_at_clock(clock)?)
    }

    pub fn position_at_clock(&self, clock: u64) -> Result<f64, ContinuousError> {
        self.signal.position(self.stream_time_at_clock(clock)?)
    }

    pub fn split_max_duration(&self) -> Result<Vec<Self>, ContinuousError> {
        let mut views: Vec<Self> = Vec::new();
        let duration = self.stream_t_end - self.stream_t_start;
        let count = (duration / MAX_SPAN_SECS).ceil() as usize;
        let host_rate = (self.end_host - self.start_host) / duration;
        for index in 0..count {
            let offset_start = (index as f64 * MAX_SPAN_SECS).min(duration);
            let offset_end = ((index + 1) as f64 * MAX_SPAN_SECS).min(duration);
            let t_lo = self.stream_t_start + offset_start;
            let (t_hi, host_hi) = if index + 1 == count {
                (self.stream_t_end, self.end_host)
            } else {
                (
                    self.stream_t_start + offset_end,
                    self.start_host + offset_end * host_rate,
                )
            };
            if t_hi <= t_lo {
                debug_assert!(
                    index + 1 == count,
                    "only the fp-edge tail chunk may collapse below one ulp"
                );
                let last = views.last_mut().expect("original clocked span has a clock");
                last.stream_t_end = self.stream_t_end;
                last.end_host = self.end_host;
                last.end_clock = self.end_clock;
                continue;
            }
            let exact = self.start_clock_exact + offset_start * self.clock_freq_hz;
            match Self::try_new(
                Arc::clone(&self.signal),
                t_lo,
                t_hi,
                self.start_host + offset_start * host_rate,
                host_hi,
                exact,
                self.clock_freq_hz,
            ) {
                Ok(view) => views.push(view),
                Err(ContinuousError::InvalidSpan {
                    reason: "positive-duration clocked view must span at least one clock",
                }) if index + 1 == count => {
                    let last = views.last_mut().expect("original clocked span has a clock");

                    last.stream_t_end = self.stream_t_end;
                    last.end_host = self.end_host;
                    last.end_clock = self.end_clock;
                }
                Err(error) => return Err(error),
            }
        }
        Ok(views)
    }
}
fn next_toward(value: f64, toward: f64) -> f64 {
    if value == toward {
        return value;
    }
    if (toward > value) == (value >= 0.0) {
        f64::from_bits(value.to_bits() + 1)
    } else {
        f64::from_bits(value.to_bits() - 1)
    }
}

fn scale_aware_slack(scale: f64) -> f64 {
    1e-10_f64.max(64.0 * f64::EPSILON * scale.abs().max(1.0))
}

const PHASE_DISTANCE_ROOT_ABS_EPS_MM: f64 = 1e-10;
const PHASE_DISTANCE_ACCUMULATION_ULPS: f64 = 64.0;
const PHASE_DURATION_ACCUMULATION_ULPS: f64 = 8.0;

fn phase_distance_solver_slack(phases: &[LawSegment], distance_scale: f64, time_scale: f64) -> f64 {
    let velocity_scale = phases.iter().fold(0.0_f64, |scale, segment| {
        let (_, end_v, _) = segment.end_state();
        scale.max(segment.v0.abs()).max(end_v.abs())
    });
    let duration_accumulation =
        PHASE_DURATION_ACCUMULATION_ULPS * f64::EPSILON * time_scale.abs().max(1.0);
    PHASE_DISTANCE_ROOT_ABS_EPS_MM * (1.0 + distance_scale.abs())
        + PHASE_DISTANCE_ACCUMULATION_ULPS * f64::EPSILON * distance_scale.abs().max(1.0)
        + velocity_scale * duration_accumulation
}

const PHASE_JOINT_INVERSION_REL_TOL: f64 = 1e-9;

fn phase_reconstructed_arc(segment: &LawSegment) -> f64 {
    let (_, end_v, _) = segment.end_state();
    segment.v0.abs().max(end_v.abs()) * segment.dt
}

fn phase_joint_distance_slack(
    previous: &LawSegment,
    next: &LawSegment,
    distance_slack: f64,
) -> f64 {
    distance_slack
        + PHASE_JOINT_INVERSION_REL_TOL
            * phase_reconstructed_arc(previous).max(phase_reconstructed_arc(next))
}

fn phase_velocity_solver_slack(segment: &LawSegment, distance_slack: f64) -> f64 {
    let acceleration_scale = match segment.law {
        ScalarLaw::ConstAccel { a0 } => a0.abs(),
        ScalarLaw::DiskRail { accel, .. } => accel,
    };
    distance_slack / segment.dt + (2.0 * acceleration_scale * distance_slack).sqrt()
}

fn validate_ordered_coverage(
    expected_start: f64,
    expected_end: f64,
    intervals: impl IntoIterator<Item = (f64, f64, f64)>,
    endpoint_slack: f64,
) -> Result<(), ContinuousError> {
    let mut intervals = intervals.into_iter();
    let Some((first_start, first_end, _)) = intervals.next() else {
        return Err(ContinuousError::InvalidSpan {
            reason: "analytic span requires phases",
        });
    };
    if (first_start - expected_start).abs() > endpoint_slack {
        return Err(ContinuousError::PhaseEndpointMismatch {
            expected: expected_start,
            actual: first_start,
        });
    }
    let mut previous_end = first_end;
    for (next_start, next_end, joint_slack) in intervals {
        if next_start > previous_end + joint_slack {
            return Err(ContinuousError::PhaseGap {
                previous_end,
                next_start,
            });
        }
        if next_start < previous_end - joint_slack {
            return Err(ContinuousError::PhaseOverlap {
                previous_end,
                next_start,
            });
        }
        previous_end = next_end;
    }
    if (previous_end - expected_end).abs() > endpoint_slack {
        return Err(ContinuousError::PhaseEndpointMismatch {
            expected: expected_end,
            actual: previous_end,
        });
    }
    Ok(())
}

fn active_phase(phases: &[LawSegment], local_t: f64) -> &LawSegment {
    let index = phases.partition_point(|segment| segment.end_time() < local_t);
    &phases[index.min(phases.len() - 1)]
}

fn check_interval(axis: &ContinuousAxis, t0: f64, t1: f64) -> Result<(), ContinuousError> {
    let (start, end) = axis.domain();
    checked_clamped_time(t0, start, end)?;
    checked_clamped_time(t1, start, end)?;
    if t1 < t0 {
        Err(ContinuousError::InvalidSpan {
            reason: "bounds interval is reversed",
        })
    } else {
        Ok(())
    }
}

fn check_time(t: f64, start: f64, end: f64) -> Result<(), ContinuousError> {
    checked_clamped_time(t, start, end).map(|_| ())
}

fn checked_clamped_time(t: f64, start: f64, end: f64) -> Result<f64, ContinuousError> {
    if !t.is_finite() {
        return Err(ContinuousError::NonFinite { t });
    }
    if !start.is_finite() || !end.is_finite() || end < start {
        return Err(ContinuousError::InvalidSpan {
            reason: "span time range must be finite and ordered",
        });
    }
    let magnitude = t.abs().max(start.abs()).max(end.abs());
    let slack = 1e-12_f64.max(8.0 * f64::EPSILON * magnitude);
    if t < start - slack || t > end + slack {
        Err(ContinuousError::TimeOutsideSpan {
            t,
            t_start: start,
            t_end: end,
        })
    } else {
        Ok(t.clamp(start, end))
    }
}

fn finite_pva(value: Pva, t: f64) -> Result<Pva, ContinuousError> {
    if [value.position, value.velocity, value.acceleration]
        .into_iter()
        .all(f64::is_finite)
    {
        Ok(value)
    } else {
        Err(ContinuousError::NonFinite { t })
    }
}

fn finite_pvaj(value: Pvaj, t: f64) -> Result<Pvaj, ContinuousError> {
    if [
        value.position,
        value.velocity,
        value.acceleration,
        value.jerk,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        Ok(value)
    } else {
        Err(ContinuousError::NonFinite { t })
    }
}

fn with_source_axis(error: ContinuousError, source_axis: usize, t: f64) -> ContinuousError {
    match error {
        ContinuousError::NonFinite { .. } => {
            ContinuousError::NonFiniteEvaluation { source_axis, t }
        }
        other => other,
    }
}

fn analytic_follower(source: &Move, axis: usize) -> Option<&FollowerDemand> {
    if source.segment.spatial.is_some() {
        axis.checked_sub(3)
            .and_then(|index| source.segment.followers.get(index))
    } else {
        source
            .segment
            .followers
            .iter()
            .find(|demand| demand.axis_index == axis)
    }
}

fn analytic_group_pva(
    span: &AnalyticMoveSpan,
    terms: &[MotorTerm],
    t: f64,
) -> Result<Pva, (ContinuousError, usize)> {
    let source_axis = terms.first().map_or(0, |term| term.source_axis);
    let scale = terms
        .iter()
        .map(|term| term.scale.abs())
        .fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return Ok(Pva {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        });
    }
    let local_t = checked_clamped_time(t, span.t_start, span.t_end)
        .map_err(|error| (error, source_axis))?
        - span.t_start;
    let (phase_s, velocity, acceleration) = active_phase(&span.phases, local_t).state_at(local_t);
    let s = phase_s - span.source_distance_origin;
    let length = span.source.segment.s_len();
    let mut spatial = [0.0; 3];
    let mut follower_position = 0.0;
    let mut follower_ratio = 0.0;
    let mut follower_slope = 0.0;
    for term in terms {
        let coefficient = term.scale / scale;
        if term.source_axis < 3 {
            spatial[term.source_axis] += coefficient;
        } else {
            match analytic_follower(&span.source, term.source_axis) {
                Some(demand) => {
                    let start = *span.axis_start_positions.get(demand.axis_index).ok_or((
                        ContinuousError::AxisOutsideMove {
                            axis: term.source_axis,
                        },
                        term.source_axis,
                    ))?;
                    follower_position +=
                        coefficient * start + coefficient * demand.offset_at(s, length);
                    follower_ratio += coefficient * demand.ratio_at(s, length);
                    follower_slope += coefficient * demand.ratio_slope(length);
                }
                None => {
                    let start = *span.axis_start_positions.get(term.source_axis).ok_or((
                        ContinuousError::AxisOutsideMove {
                            axis: term.source_axis,
                        },
                        term.source_axis,
                    ))?;
                    follower_position += coefficient * start;
                }
            }
        }
    }
    let segment = span.source.segment.spatial.as_ref();
    if segment.is_none() && spatial != [0.0; 3] {
        return Err((
            ContinuousError::AxisOutsideMove {
                axis: spatial
                    .iter()
                    .position(|coefficient| *coefficient != 0.0)
                    .unwrap_or(0),
            },
            source_axis,
        ));
    }
    let spatial_position = segment.map_or(0.0, |segment| dot(spatial, segment.point_at(s)));
    let surface_position = if spatial[2] == 0.0 {
        0.0
    } else {
        match &span.surface {
            SurfaceMode::None => 0.0,
            SurfaceMode::Constant(offset) => spatial[2] * offset,
            SurfaceMode::Variable(_) => {
                return Err((ContinuousError::VariableSurfaceBeforeDispatch, source_axis));
            }
        }
    };
    let projection = segment.map_or(follower_ratio, |segment| {
        dot(spatial, segment.heading_at(s)) + follower_ratio
    });
    let projection_slope = segment.map_or(follower_slope, |segment| {
        dot(spatial, segment.dheading_ds(s)) + follower_slope
    });
    let normalized = Pva {
        position: spatial_position + surface_position + follower_position,
        velocity: velocity * projection,
        acceleration: acceleration * projection + velocity * velocity * projection_slope,
    };
    finite_pva(scale_pva(normalized, scale), t).map_err(|error| (error, source_axis))
}

fn scale_pva(value: Pva, scale: f64) -> Pva {
    Pva {
        position: scale * value.position,
        velocity: scale * value.velocity,
        acceleration: scale * value.acceleration,
    }
}

fn zero_bounds() -> PvaBounds {
    PvaBounds {
        velocity_min: 0.0,
        velocity_max: 0.0,
        acceleration_abs_max: 0.0,
        velocity_continuous: true,
    }
}
fn profile_bounds(
    velocity: (f64, f64),
    acceleration: (f64, f64),
    velocity_continuous: bool,
) -> PvaBounds {
    PvaBounds {
        velocity_min: velocity.0,
        velocity_max: velocity.1,
        acceleration_abs_max: acceleration.0.abs().max(acceleration.1.abs()),
        velocity_continuous,
    }
}

fn scale_bounds(bounds: PvaBounds, scale: f64) -> Result<PvaBounds, ContinuousError> {
    if !scale.is_finite() {
        return Err(ContinuousError::InvalidSpan {
            reason: "motor scale must be finite",
        });
    }
    if scale == 0.0 {
        return Ok(zero_bounds());
    }
    let a = scale * bounds.velocity_min;
    let b = scale * bounds.velocity_max;
    Ok(PvaBounds {
        velocity_min: a.min(b),
        velocity_max: a.max(b),
        acceleration_abs_max: scale.abs() * bounds.acceleration_abs_max,
        velocity_continuous: bounds.velocity_continuous,
    })
}

fn validate_spline_control_points(curve: &ScalarNurbs) -> Result<(), ContinuousError> {
    if curve
        .control_points()
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(ContinuousError::InvalidSpan {
            reason: "spline control points must be finite",
        });
    }
    Ok(())
}

fn spline_domain(curve: &ScalarNurbs) -> (f64, f64) {
    let degree = curve.degree() as usize;
    (
        curve.knots()[degree],
        curve.knots()[curve.knots().len() - degree - 1],
    )
}

fn spline_derivatives<const ORDERS: usize>(curve: &ScalarNurbs, t: f64) -> [f64; ORDERS] {
    let mut out = [0.0; ORDERS];
    nurbs::eval::eval_derivatives(
        curve.control_points(),
        curve.knots(),
        curve.degree(),
        t,
        ORDERS - 1,
        &mut out,
    );
    out
}

fn spline_pva(curve: &ScalarNurbs, t: f64) -> Result<Pva, ContinuousError> {
    let t = spline_evaluation_time(curve, t)?;
    let [position, velocity, acceleration] = spline_derivatives(curve, t);
    Ok(Pva {
        position,
        velocity,
        acceleration,
    })
}

fn spline_pvaj(curve: &ScalarNurbs, t: f64) -> Result<Pvaj, ContinuousError> {
    let t = spline_evaluation_time(curve, t)?;
    let [position, velocity, acceleration, jerk] = spline_derivatives(curve, t);
    Ok(Pvaj {
        position,
        velocity,
        acceleration,
        jerk,
    })
}

fn spline_evaluation_time(curve: &ScalarNurbs, t: f64) -> Result<f64, ContinuousError> {
    let (t_start, t_end) = spline_domain(curve);
    Ok(spline_owned_time(
        curve,
        checked_clamped_time(t, t_start, t_end)?,
    ))
}

const DEGENERATE_PIECE_WIDTH_ULPS: f64 = 4.0;
const DEGENERATE_PIECE_SLACK_S: f64 = 1e-12;
const PIECE_SEAM_ROUNDOFF_ULPS: f64 = (1_u64 << 20) as f64;

fn degenerate_knot_span(knots: &[f64], span: usize) -> bool {
    let (low, high) = (knots[span], knots[span + 1]);
    high - low <= DEGENERATE_PIECE_WIDTH_ULPS * f64::EPSILON * low.abs().max(high.abs())
}

fn spline_pv(curve: &ScalarNurbs, t: f64) -> (f64, f64) {
    let [position, velocity] = spline_derivatives(curve, t);
    (position, velocity)
}

fn spline_pv_continuous(curve: &ScalarNurbs, left: f64, right: f64) -> bool {
    let (left_position, left_velocity) = spline_pv(curve, left);
    let (right_position, right_velocity) = spline_pv(curve, right);
    let position_scale = left_position.abs().max(right_position.abs()).max(1.0);
    let velocity_scale = left_velocity.abs().max(right_velocity.abs()).max(1.0);
    (left_position - right_position).abs()
        <= PIECE_SEAM_ROUNDOFF_ULPS * f64::EPSILON * position_scale
        && (left_velocity - right_velocity).abs()
            <= PIECE_SEAM_ROUNDOFF_ULPS * f64::EPSILON * velocity_scale
}

/// The largest representable time strictly below `value`, for evaluating
/// infinitesimally inside the carrier interval that *ends* at `value`.
pub fn interior_time_below(value: f64) -> f64 {
    if value > 0.0 {
        f64::from_bits(value.to_bits() - 1)
    } else if value < 0.0 {
        f64::from_bits(value.to_bits() + 1)
    } else {
        -f64::from_bits(1)
    }
}

/// The smallest representable time strictly above `value`, for evaluating
/// infinitesimally inside the carrier interval that *starts* at `value`.
pub fn interior_time_above(value: f64) -> f64 {
    if value < 0.0 {
        f64::from_bits(value.to_bits() - 1)
    } else {
        f64::from_bits(value.to_bits() + 1)
    }
}

fn spline_owned_time(curve: &ScalarNurbs, t: f64) -> f64 {
    let degree = curve.degree() as usize;
    let knots = curve.knots();
    let count = curve.control_points().len();
    let span = nurbs::knot::find_knot_span(knots, degree, count, t);
    if !degenerate_knot_span(knots, span) {
        return t;
    }
    let mut right = span;
    while right + 1 < count && degenerate_knot_span(knots, right) {
        right += 1;
    }
    let mut left = span;
    while left > degree && degenerate_knot_span(knots, left) {
        left -= 1;
    }
    let right_time = (!degenerate_knot_span(knots, right)).then(|| knots[right]);
    let left_time =
        (!degenerate_knot_span(knots, left)).then(|| interior_time_below(knots[left + 1]));
    match (left_time, right_time) {
        (Some(left_time), Some(right_time)) => {
            if knots[right] - knots[left + 1] <= DEGENERATE_PIECE_SLACK_S
                && spline_pv_continuous(curve, left_time, right_time)
            {
                right_time
            } else {
                t
            }
        }
        (None, Some(right_time)) if knots[right] - t <= DEGENERATE_PIECE_SLACK_S => right_time,
        (Some(left_time), None) if t - knots[left + 1] <= DEGENERATE_PIECE_SLACK_S => left_time,
        _ => t,
    }
}

/// The signal on `[t0, valid_until)` as one Taylor polynomial about `t0`:
/// exact for spline and hold carriers between two of their breakpoints, so a
/// root search can predict crossings without touching the spline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPolynomial {
    t0: f64,
    valid_until: f64,
    degree: usize,
    coefficients: [f64; nurbs::WORKSPACE_SIZE],
    /// The largest operand the carrier's own evaluation handles - its local
    /// control points and base - which its rounding scales with.
    operand_scale: f64,
}

impl LocalPolynomial {
    pub fn t0(&self) -> f64 {
        self.t0
    }

    pub fn valid_until(&self) -> f64 {
        self.valid_until
    }

    /// An interval certainly containing every position on `[t_from, t_to]`:
    /// the Bernstein hull on that interval, widened by its own roundoff.
    pub fn position_range(&self, t_from: f64, t_to: f64) -> (f64, f64) {
        bernstein_hull(
            &self.coefficients[..=self.degree],
            t_from - self.t0,
            t_to - self.t0,
        )
    }

    /// An interval certainly containing every velocity on `[t_from, t_to]`.
    pub fn velocity_range(&self, t_from: f64, t_to: f64) -> (f64, f64) {
        if self.degree == 0 {
            return (0.0, 0.0);
        }
        let mut derivative = [0.0; nurbs::WORKSPACE_SIZE];
        for order in 1..=self.degree {
            derivative[order - 1] = self.coefficients[order] * order as f64;
        }
        bernstein_hull(&derivative[..self.degree], t_from - self.t0, t_to - self.t0)
    }

    /// How far this polynomial and the carrier it expands can disagree at
    /// `t` once both have been rounded: the carrier's evaluation rounds
    /// against its control points, the expansion against its Taylor terms,
    /// each through about `degree` operations; sixty-four times that.
    pub fn noise_band(&self, t: f64) -> f64 {
        let local = (t - self.t0).abs();
        let mut magnitude = self.operand_scale;
        let mut power = 1.0;
        for &coefficient in &self.coefficients[..=self.degree] {
            magnitude = magnitude.max(coefficient.abs() * power);
            power *= local;
        }
        64.0 * (self.degree + 1) as f64 * f64::EPSILON * magnitude
    }

    pub fn position(&self, t: f64) -> f64 {
        let local = t - self.t0;
        let mut acc = self.coefficients[self.degree];
        for &coefficient in self.coefficients[..self.degree].iter().rev() {
            acc = acc * local + coefficient;
        }
        acc
    }

    pub fn velocity(&self, t: f64) -> f64 {
        if self.degree == 0 {
            return 0.0;
        }
        let local = t - self.t0;
        let mut acc = self.coefficients[self.degree] * self.degree as f64;
        for order in (1..self.degree).rev() {
            acc = acc * local + self.coefficients[order] * order as f64;
        }
        acc
    }

    fn constant(t0: f64, valid_until: f64, value: f64) -> Self {
        let mut coefficients = [0.0; nurbs::WORKSPACE_SIZE];
        coefficients[0] = value;
        Self {
            t0,
            valid_until,
            degree: 0,
            coefficients,
            operand_scale: value.abs(),
        }
    }

    fn add_scaled(&mut self, other: &Self, scale: f64) {
        for order in 0..=other.degree {
            self.coefficients[order] += scale * other.coefficients[order];
        }
        self.degree = self.degree.max(other.degree);
        self.valid_until = self.valid_until.min(other.valid_until);
        self.operand_scale = self.operand_scale.max(scale.abs() * other.operand_scale);
    }

    fn scaled(self, scale: f64) -> Self {
        let mut result = Self::constant(self.t0, self.valid_until, 0.0);
        result.add_scaled(&self, scale);
        result
    }
}

/// Every value of the power-basis polynomial `coefficients` on `[a, b]` lies
/// between the least and greatest of its Bernstein coefficients on that
/// interval. Every intermediate of the shift to `a`, the scale by `b - a`
/// and the basis change is bounded by `Σ|c_k|·(2·max(|a|,|b|))^k`, so the
/// hull is widened by that many roundings of it.
fn bernstein_hull(coefficients: &[f64], a: f64, b: f64) -> (f64, f64) {
    let degree = coefficients.len() - 1;
    let width = b - a;
    let reach = 2.0 * a.abs().max(b.abs());
    let mut shifted = [0.0; nurbs::WORKSPACE_SIZE];
    shifted[..=degree].copy_from_slice(coefficients);
    let mut magnitude = 0.0;
    let mut reach_power = 1.0;
    for &coefficient in coefficients {
        magnitude += coefficient.abs() * reach_power;
        reach_power *= reach;
    }
    for level in 0..degree {
        for index in (level..degree).rev() {
            shifted[index] += a * shifted[index + 1];
        }
    }
    let mut width_power = 1.0;
    for coefficient in shifted[..=degree].iter_mut() {
        *coefficient *= width_power;
        width_power *= width;
    }
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for j in 0..=degree {
        let mut value = shifted[0];
        let mut weight = 1.0;
        for k in 1..=j {
            weight *= (j + 1 - k) as f64 / (degree + 1 - k) as f64;
            value += weight * shifted[k];
        }
        low = low.min(value);
        high = high.max(value);
    }
    let operations = ((degree + 1) * (degree + 1)) as f64;
    let margin = 4.0 * operations * f64::EPSILON * magnitude;
    (low - margin, high + margin)
}

fn spline_local_polynomial(curve: &ScalarNurbs, base: f64, t0: f64) -> LocalPolynomial {
    let degree = curve.degree() as usize;
    let knots = curve.knots();
    let count = curve.control_points().len();
    let span = nurbs::knot::find_knot_span(knots, degree, count, t0);
    let valid_until = knots[span + 1..=count]
        .iter()
        .copied()
        .find(|&knot| knot > t0)
        .unwrap_or(knots[count]);
    let mut coefficients = [0.0; nurbs::WORKSPACE_SIZE];
    nurbs::eval::eval_derivatives(
        curve.control_points(),
        knots,
        curve.degree(),
        t0,
        degree,
        &mut coefficients,
    );
    let mut factorial = 1.0;
    for (order, coefficient) in coefficients.iter_mut().enumerate().take(degree + 1).skip(1) {
        factorial *= order as f64;
        *coefficient /= factorial;
    }
    coefficients[0] += base;
    let operand_scale = curve.control_points()[span - degree..=span]
        .iter()
        .fold(base.abs(), |scale, point| scale.max(point.abs()));
    LocalPolynomial {
        t0,
        valid_until,
        degree,
        coefficients,
        operand_scale,
    }
}

impl ContinuousAxis {
    fn local_polynomial(&self, t0: f64) -> Option<LocalPolynomial> {
        match self {
            Self::Spline(curve) => Some(spline_local_polynomial(curve, 0.0, t0)),
            Self::RelativeSpline {
                base_position,
                curve,
            } => Some(spline_local_polynomial(curve, *base_position, t0)),
            Self::PiecewiseRelativeSpline(pieces) => {
                let piece = owning_piece(pieces, t0).ok()?;
                let mut polynomial = spline_local_polynomial(&piece.curve, piece.base_position, t0);
                polynomial.valid_until = polynomial.valid_until.min(piece.t_end);
                Some(polynomial)
            }
            Self::Hold {
                position, t_end, ..
            } => Some(LocalPolynomial::constant(t0, *t_end, *position)),
            Self::Analytic { .. } | Self::Nudge(_) | Self::Buzz { .. } => None,
        }
    }
}

impl MotorGroup {
    fn local_polynomial(&self, t0: f64) -> Option<LocalPolynomial> {
        match self {
            Self::Spline {
                curve,
                summed_scale,
            } => Some(spline_local_polynomial(curve, 0.0, t0).scaled(*summed_scale)),
            Self::RelativeSpline {
                curve,
                base_position,
                summed_scale,
            } => Some(spline_local_polynomial(curve, *base_position, t0).scaled(*summed_scale)),
            Self::Independent(term) => Some(term.axis.local_polynomial(t0)?.scaled(term.scale)),
            Self::Analytic { .. } => None,
        }
    }
}

impl MotorSpan {
    /// The signal from `t0` to the nearest breakpoint after it as one exact
    /// polynomial, when every carrier is polynomial there. Analytic and
    /// oscillating carriers have none.
    pub fn local_polynomial(&self, t0: f64) -> Option<LocalPolynomial> {
        let t0 = checked_clamped_time(t0, self.t_start, self.t_end).ok()?;
        let mut total = LocalPolynomial::constant(t0, self.t_end, 0.0);
        for group in self.groups.iter() {
            total.add_scaled(&group.local_polynomial(t0)?, 1.0);
        }
        (total.valid_until > t0).then_some(total)
    }
}

/// The `order`-th hodograph control point `Q^(order)_index`, from the standard
/// B-spline derivative recurrence `Q^k_i = (p−k+1)·(Q^{k−1}_{i+1} − Q^{k−1}_i) /
/// (u_{i+p+1} − u_{i+k})`.
fn derivative_control(
    cps: &[f64],
    knots: &[f64],
    degree: usize,
    order: usize,
    index: usize,
) -> f64 {
    if order == 0 {
        return cps[index];
    }
    let denominator = knots[index + degree + 1] - knots[index + order];
    if denominator <= 0.0 {
        return 0.0;
    }
    let low = derivative_control(cps, knots, degree, order - 1, index);
    let high = derivative_control(cps, knots, degree, order - 1, index + 1);
    (degree - order + 1) as f64 * (high - low) / denominator
}

fn spline_bounds(
    curve: &ScalarNurbs,
    scale: f64,
    t0: f64,
    t1: f64,
) -> Result<PvaBounds, ContinuousError> {
    if !scale.is_finite() {
        return Err(ContinuousError::InvalidSpan {
            reason: "spline scale must be finite",
        });
    }
    let (domain_start, domain_end) = spline_domain(curve);
    let t0 = checked_clamped_time(t0, domain_start, domain_end)?;
    let t1 = checked_clamped_time(t1, domain_start, domain_end)?;
    if t1 < t0 {
        return Err(ContinuousError::InvalidSpan {
            reason: "bounds interval is reversed",
        });
    }
    if scale == 0.0 {
        return Ok(zero_bounds());
    }
    let degree = curve.degree() as usize;
    let cps = curve.control_points();
    let knots = curve.knots();
    let mut velocity_min = f64::INFINITY;
    let mut velocity_max = f64::NEG_INFINITY;
    let mut acceleration_abs_max = 0.0_f64;
    if degree > 0 {
        for index in 0..cps.len() - 1 {
            let support_start = knots[index + 1];
            let support_end = knots[index + degree + 1];
            if if t1 > t0 {
                support_end <= t0 || support_start >= t1
            } else {
                support_end < t0 || support_start > t1
            } {
                continue;
            }
            let velocity = derivative_control(cps, knots, degree, 1, index);
            velocity_min = velocity_min.min(velocity);
            velocity_max = velocity_max.max(velocity);
        }
        if !velocity_min.is_finite() {
            let velocity = spline_pv(curve, t0).1;
            velocity_min = velocity;
            velocity_max = velocity;
        }
        if degree > 1 {
            for index in 0..cps.len() - 2 {
                let support_start = knots[index + 2];
                let support_end = knots[index + degree + 1];
                if if t1 > t0 {
                    support_end <= t0 || support_start >= t1
                } else {
                    support_end < t0 || support_start > t1
                } {
                    continue;
                }
                let acceleration = derivative_control(cps, knots, degree, 2, index);
                acceleration_abs_max = acceleration_abs_max.max(acceleration.abs());
            }
        }
    } else {
        velocity_min = 0.0;
        velocity_max = 0.0;
    }
    let velocity_continuous = velocity_continuous_within(knots, degree, t0, t1);
    if degree > 0 && velocity_continuous {
        let radius = acceleration_abs_max * (t1 - t0);
        let start_velocity = spline_pv(curve, next_toward(t0, t1)).1;
        let end_velocity = spline_pv(curve, next_toward(t1, t0)).1;
        velocity_min = velocity_min
            .max(start_velocity - radius)
            .max(end_velocity - radius);
        velocity_max = velocity_max
            .min(start_velocity + radius)
            .min(end_velocity + radius);
    }
    scale_bounds(
        PvaBounds {
            velocity_min,
            velocity_max,
            acceleration_abs_max,
            velocity_continuous,
        },
        scale,
    )
}

/// An interior knot of multiplicity `degree` is a C0 joint: velocity jumps there.
fn velocity_continuous_within(knots: &[f64], degree: usize, t0: f64, t1: f64) -> bool {
    let start = knots.partition_point(|&t| t <= t0);
    let end = knots.partition_point(|&t| t < t1).max(start);
    let interior = &knots[start..end];
    interior
        .chunk_by(|a, b| a == b)
        .all(|run| run.len() < degree)
}

const EMPTY_PIECES: ContinuousError = ContinuousError::InvalidSpan {
    reason: "piecewise relative spline requires at least one piece",
};

fn validate_relative_pieces(pieces: &[RelativeSplinePiece]) -> Result<(), ContinuousError> {
    let Some(first) = pieces.first() else {
        return Err(EMPTY_PIECES);
    };
    let mut previous_end = first.t_start;
    for piece in pieces {
        if ![piece.base_position, piece.t_start, piece.t_end]
            .into_iter()
            .all(f64::is_finite)
            || piece.t_end <= piece.t_start
        {
            return Err(ContinuousError::InvalidSpan {
                reason: "relative spline piece needs a finite base and a positive time window",
            });
        }
        if piece
            .curve
            .control_points()
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(ContinuousError::InvalidSpan {
                reason: "spline control points must be finite",
            });
        }
        let seam_slack = scale_aware_slack(previous_end.abs().max(piece.t_start.abs()));
        if piece.t_start > previous_end + seam_slack {
            return Err(ContinuousError::PieceGap {
                previous_end,
                next_start: piece.t_start,
            });
        }
        if piece.t_start < previous_end - seam_slack {
            return Err(ContinuousError::PieceOverlap {
                previous_end,
                next_start: piece.t_start,
            });
        }
        let (curve_start, curve_end) = spline_domain(&piece.curve);
        checked_clamped_time(piece.t_start, curve_start, curve_end)?;
        checked_clamped_time(piece.t_end, curve_start, curve_end)?;
        previous_end = piece.t_end;
    }
    Ok(())
}

fn piecewise_relative_domain(pieces: &[RelativeSplinePiece]) -> (f64, f64) {
    match (pieces.first(), pieces.last()) {
        (Some(first), Some(last)) => (first.t_start, last.t_end),
        _ => (f64::NAN, f64::NAN),
    }
}

fn owning_piece_index(pieces: &[RelativeSplinePiece], t: f64) -> usize {
    pieces
        .partition_point(|piece| piece.t_end <= t)
        .min(pieces.len() - 1)
}

fn owning_piece(
    pieces: &[RelativeSplinePiece],
    t: f64,
) -> Result<&RelativeSplinePiece, ContinuousError> {
    let (t_start, t_end) = piecewise_relative_domain(pieces);
    if pieces.is_empty() {
        return Err(EMPTY_PIECES);
    }
    let t = checked_clamped_time(t, t_start, t_end)?;
    Ok(&pieces[owning_piece_index(pieces, t)])
}

fn piecewise_relative_bounds(
    pieces: &[RelativeSplinePiece],
    t0: f64,
    t1: f64,
) -> Result<PvaBounds, ContinuousError> {
    let (t_start, t_end) = piecewise_relative_domain(pieces);
    if pieces.is_empty() {
        return Err(EMPTY_PIECES);
    }
    let t0 = checked_clamped_time(t0, t_start, t_end)?;
    let t1 = checked_clamped_time(t1, t_start, t_end)?;
    if t1 < t0 {
        return Err(ContinuousError::InvalidSpan {
            reason: "bounds interval is reversed",
        });
    }
    let first = owning_piece_index(pieces, t0);
    let last = owning_piece_index(pieces, t1);
    let mut result = PvaBounds {
        velocity_min: f64::INFINITY,
        velocity_max: f64::NEG_INFINITY,
        acceleration_abs_max: 0.0,
        velocity_continuous: first == last,
    };
    for piece in &pieces[first..=last] {
        let lo = t0.max(piece.t_start);
        let hi = t1.min(piece.t_end);
        if hi < lo {
            continue;
        }
        let bounds = spline_bounds(&piece.curve, 1.0, lo, hi)?;
        result.velocity_min = result.velocity_min.min(bounds.velocity_min);
        result.velocity_max = result.velocity_max.max(bounds.velocity_max);
        result.acceleration_abs_max = result.acceleration_abs_max.max(bounds.acceleration_abs_max);
        result.velocity_continuous &= bounds.velocity_continuous;
    }
    Ok(result)
}

fn analytic_group_bounds<I>(
    span: &AnalyticMoveSpan,
    axes: I,
    t0: f64,
    t1: f64,
) -> Result<PvaBounds, ContinuousError>
where
    I: Iterator<Item = (usize, f64)> + Clone,
{
    if matches!(&span.surface, SurfaceMode::Variable(_)) {
        return Err(ContinuousError::VariableSurfaceBeforeDispatch);
    }
    let local0 = checked_clamped_time(t0, span.t_start, span.t_end)? - span.t_start;
    let local1 = checked_clamped_time(t1, span.t_start, span.t_end)? - span.t_start;
    if local1 < local0 {
        return Err(ContinuousError::InvalidSpan {
            reason: "bounds interval is reversed",
        });
    }
    if axes.clone().any(|(_, scale)| !scale.is_finite()) {
        return Err(ContinuousError::InvalidSpan {
            reason: "motor scale must be finite",
        });
    }
    let scale = axes
        .clone()
        .map(|(_, scale)| scale.abs())
        .fold(0.0_f64, f64::max);
    if scale == 0.0 {
        return Ok(zero_bounds());
    }
    let mut spatial = [0.0; 3];
    let mut follower_start = 0.0;
    let mut follower_end = 0.0;
    let mut follower_slope = 0.0;
    let length = span.source.segment.s_len();
    let (phase_s0, _, _) = active_phase(&span.phases, local0).state_at(local0);
    let (phase_s1, _, _) = active_phase(&span.phases, local1).state_at(local1);
    let s0 = phase_s0 - span.source_distance_origin;
    let s1 = phase_s1 - span.source_distance_origin;
    let s_lo = s0.min(s1);
    let s_hi = s0.max(s1);
    for (axis, raw_coefficient) in axes {
        let coefficient = raw_coefficient / scale;
        if axis < 3 {
            spatial[axis] += coefficient;
        } else if let Some(demand) = analytic_follower(&span.source, axis) {
            follower_start += coefficient * demand.ratio_at(s_lo, length);
            follower_end += coefficient * demand.ratio_at(s_hi, length);
            follower_slope += coefficient * demand.ratio_slope(length);
        } else if axis >= span.axis_start_positions.len() {
            return Err(ContinuousError::AxisOutsideMove { axis });
        }
    }
    let (q_min, q_max, q_prime_abs) = projection_bounds(
        span.source.segment.spatial.as_ref(),
        spatial,
        follower_start,
        follower_end,
        follower_slope,
        s_lo,
        s_hi,
    )?;
    if q_min == 0.0 && q_max == 0.0 && q_prime_abs == 0.0 {
        return Ok(zero_bounds());
    }
    let (v_min, v_max, a_abs) = scalar_phase_bounds(&span.phases, local0, local1);
    let products = [v_min * q_min, v_min * q_max, v_max * q_min, v_max * q_max];
    let velocity_min = products.into_iter().fold(f64::INFINITY, f64::min);
    let velocity_max = products.into_iter().fold(f64::NEG_INFINITY, f64::max);
    let q_abs = q_min.abs().max(q_max.abs());
    let acceleration_abs_max = a_abs * q_abs + v_min.abs().max(v_max.abs()).powi(2) * q_prime_abs;
    let bounds = scale_bounds(
        PvaBounds {
            velocity_min,
            velocity_max,
            acceleration_abs_max,
            velocity_continuous: true,
        },
        scale,
    )?;
    if [
        bounds.velocity_min,
        bounds.velocity_max,
        bounds.acceleration_abs_max,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        Ok(bounds)
    } else {
        Err(ContinuousError::NonFinite { t: t0 })
    }
}

fn scalar_phase_bounds(phases: &[LawSegment], t0: f64, t1: f64) -> (f64, f64, f64) {
    let (_, velocity0, acceleration0) = active_phase(phases, t0).state_at(t0);
    let (_, velocity1, acceleration1) = active_phase(phases, t1).state_at(t1);
    let mut velocity_min = velocity0.min(velocity1);
    let mut velocity_max = velocity0.max(velocity1);
    let mut acceleration_abs_max = acceleration0.abs().max(acceleration1.abs());
    for segment in phases {
        let lo = t0.max(segment.t0);
        let hi = t1.min(segment.end_time());
        if lo > hi {
            continue;
        }
        let mut arc = [0.0_f64; 2];
        let mut speed = [0.0_f64; 2];
        for (slot, time) in [lo, hi].into_iter().enumerate() {
            let (distance, velocity, acceleration) = segment.state_at(time);
            velocity_min = velocity_min.min(velocity);
            velocity_max = velocity_max.max(velocity);
            acceleration_abs_max = acceleration_abs_max.max(acceleration.abs());
            arc[slot] = distance - segment.s0;
            speed[slot] = velocity;
        }
        acceleration_abs_max =
            acceleration_abs_max.max(rail_acceleration_ceiling(&segment.law, arc, speed));
    }
    (velocity_min, velocity_max, acceleration_abs_max)
}

/// The rail spends its budget as `|a_t| = sqrt(A² − (κ·v²)²)`, which grows as
/// the normal load `|κ|·v²` falls: the tangential peak sits strictly inside the
/// interval whenever the curvature crosses zero there, so both endpoints
/// report the smallest accelerations of the phase.
fn rail_acceleration_ceiling(law: &ScalarLaw, arc: [f64; 2], speed: [f64; 2]) -> f64 {
    let ScalarLaw::DiskRail {
        accel,
        kappa0,
        sigma,
        ..
    } = *law
    else {
        return 0.0;
    };
    let curvature = arc.map(|distance| kappa0 + sigma * distance);
    let curvature_abs_min = if curvature[0] * curvature[1] <= 0.0 {
        0.0
    } else {
        curvature[0].abs().min(curvature[1].abs())
    };
    let normal_load = curvature_abs_min * speed[0].min(speed[1]).powi(2);
    (accel * accel - normal_load * normal_load).max(0.0).sqrt()
}

fn projection_bounds(
    segment: Option<&Segment>,
    spatial: [f64; 3],
    follower_start: f64,
    follower_end: f64,
    follower_slope: f64,
    s_lo: f64,
    s_hi: f64,
) -> Result<(f64, f64, f64), ContinuousError> {
    let follower_min = follower_start.min(follower_end);
    let follower_max = follower_start.max(follower_end);
    let Some(segment) = segment else {
        return Ok((follower_min, follower_max, follower_slope.abs()));
    };
    let mut candidates = vec![s_lo, s_hi];
    append_projection_extrema(
        segment,
        spatial,
        follower_slope,
        s_lo,
        s_hi,
        &mut candidates,
    )?;
    candidates.sort_by(f64::total_cmp);
    candidates.dedup_by(|left, right| *left == *right);
    let kappa_max = match segment {
        Segment::Line(_) => 0.0,
        Segment::Arc(arc) => 1.0 / arc.radius,
        Segment::Clothoid(clothoid) => (clothoid.kappa_0 + clothoid.sigma * s_lo)
            .abs()
            .max((clothoid.kappa_0 + clothoid.sigma * s_hi).abs()),
    };
    let planar_norm = match segment {
        Segment::Line(_) => spatial
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt(),
        Segment::Arc(arc) => libm::hypot(dot(spatial, arc.u), dot(spatial, arc.v)),
        Segment::Clothoid(clothoid) => {
            libm::hypot(dot(spatial, clothoid.u), dot(spatial, clothoid.v))
        }
    };
    if matches!(segment, Segment::Clothoid(_)) && follower_slope != 0.0 {
        let mut combined_min = f64::INFINITY;
        let mut combined_max = f64::NEG_INFINITY;
        for partition in candidates.windows(2) {
            let spatial0 = dot(spatial, segment.heading_at(partition[0]));
            let spatial1 = dot(spatial, segment.heading_at(partition[1]));
            let follower0 = follower_start + follower_slope * (partition[0] - s_lo);
            let follower1 = follower_start + follower_slope * (partition[1] - s_lo);
            combined_min = combined_min.min(spatial0.min(spatial1) + follower0.min(follower1));
            combined_max = combined_max.max(spatial0.max(spatial1) + follower0.max(follower1));
        }
        return Ok((
            combined_min,
            combined_max,
            planar_norm * kappa_max + follower_slope.abs(),
        ));
    }
    let mut combined_min = f64::INFINITY;
    let mut combined_max = f64::NEG_INFINITY;
    for &s in &candidates {
        let value =
            dot(spatial, segment.heading_at(s)) + follower_start + follower_slope * (s - s_lo);
        combined_min = combined_min.min(value);
        combined_max = combined_max.max(value);
    }
    Ok((
        combined_min,
        combined_max,
        planar_norm * kappa_max + follower_slope.abs(),
    ))
}

fn append_projection_extrema(
    segment: &Segment,
    spatial: [f64; 3],
    follower_slope: f64,
    s_lo: f64,
    s_hi: f64,
    output: &mut Vec<f64>,
) -> Result<(), ContinuousError> {
    match segment {
        Segment::Line(_) => {}
        Segment::Arc(arc) => {
            let a = dot(spatial, arc.u);
            let b = dot(spatial, arc.v);
            let magnitude = libm::hypot(a, b);
            if magnitude == 0.0 {
                return Ok(());
            }
            let normalized = follower_slope * arc.radius / magnitude;
            if normalized.abs() > 1.0 {
                return Ok(());
            }
            let center = libm::atan2(b, a);
            let offset = libm::acos(normalized);
            let sign = arc.sweep.signum();
            let theta_lo = arc.start_angle + sign * s_lo / arc.radius;
            let theta_hi = arc.start_angle + sign * s_hi / arc.radius;
            for base in [center - offset, center + offset] {
                append_angle_roots(
                    base,
                    2.0 * PI,
                    theta_lo.min(theta_hi),
                    theta_lo.max(theta_hi),
                    |theta| (theta - arc.start_angle) * arc.radius / sign,
                    output,
                )?;
            }
        }
        Segment::Clothoid(clothoid) => {
            let a = dot(spatial, clothoid.u);
            let b = dot(spatial, clothoid.v);
            if a == 0.0 && b == 0.0 {
                return Ok(());
            }
            if clothoid.sigma != 0.0 {
                let zero = -clothoid.kappa_0 / clothoid.sigma;
                push_candidate(zero, s_lo, s_hi, output);
            }
            let theta0 = clothoid.kappa_0 * s_lo + 0.5 * clothoid.sigma * s_lo * s_lo;
            let theta1 = clothoid.kappa_0 * s_hi + 0.5 * clothoid.sigma * s_hi * s_hi;
            let turning = if clothoid.sigma != 0.0 {
                -clothoid.kappa_0 / clothoid.sigma
            } else {
                f64::NAN
            };
            let theta_turn = clothoid.kappa_0 * turning + 0.5 * clothoid.sigma * turning * turning;
            let theta_min = if turning > s_lo && turning < s_hi {
                theta0.min(theta1).min(theta_turn)
            } else {
                theta0.min(theta1)
            };
            let theta_max = if turning > s_lo && turning < s_hi {
                theta0.max(theta1).max(theta_turn)
            } else {
                theta0.max(theta1)
            };
            let base = libm::atan2(b, a);
            let (first, last) = periodic_index_range(base, PI, theta_min, theta_max)?;
            reserve_periodic_roots(output, first, last)?;
            for index in first..=last {
                let target = base + index as f64 * PI;
                for root in quadratic_roots(0.5 * clothoid.sigma, clothoid.kappa_0, -target)
                    .into_iter()
                    .flatten()
                {
                    push_candidate(root, s_lo, s_hi, output);
                }
            }
        }
    }
    Ok(())
}

fn append_angle_roots<F: Fn(f64) -> f64>(
    base: f64,
    period: f64,
    lo: f64,
    hi: f64,
    map: F,
    output: &mut Vec<f64>,
) -> Result<(), ContinuousError> {
    let (first, last) = periodic_index_range(base, period, lo, hi)?;
    reserve_periodic_roots(output, first, last)?;
    for index in first..=last {
        output.push(map(base + index as f64 * period));
    }
    Ok(())
}

fn periodic_index_range(
    base: f64,
    period: f64,
    lo: f64,
    hi: f64,
) -> Result<(i64, i64), ContinuousError> {
    let first = ((lo - base) / period).ceil();
    let last = ((hi - base) / period).floor();
    if !first.is_finite()
        || !last.is_finite()
        || first < i64::MIN as f64
        || first >= i64::MAX as f64
        || last < i64::MIN as f64
        || last >= i64::MAX as f64
    {
        return Err(ContinuousError::InvalidSpan {
            reason: "projection extrema family is not representable",
        });
    }
    Ok((first as i64, last as i64))
}

fn reserve_periodic_roots(
    output: &mut Vec<f64>,
    first: i64,
    last: i64,
) -> Result<(), ContinuousError> {
    if last < first {
        return Ok(());
    }
    let count = u64::try_from(i128::from(last) - i128::from(first))
        .ok()
        .and_then(|count| count.checked_add(1))
        .and_then(|count| usize::try_from(count).ok())
        .ok_or(ContinuousError::InvalidSpan {
            reason: "projection extrema family is too large",
        })?;
    output
        .try_reserve(count)
        .map_err(|_| ContinuousError::InvalidSpan {
            reason: "projection extrema family is too large",
        })
}

fn quadratic_roots(a: f64, b: f64, c: f64) -> [Option<f64>; 2] {
    if a == 0.0 {
        return if b == 0.0 {
            [None, None]
        } else {
            [Some(-c / b), None]
        };
    }
    let coefficient_scale = a.abs().max(b.abs()).max(c.abs());
    let a = a / coefficient_scale;
    let b = b / coefficient_scale;
    let c = c / coefficient_scale;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return [None, None];
    }
    let root = libm::sqrt(discriminant);
    let q = -0.5 * (b + if b >= 0.0 { root } else { -root });
    if q == 0.0 {
        [Some(-b / (2.0 * a)), None]
    } else {
        [Some(q / a), Some(c / q)]
    }
}

fn push_candidate(value: f64, lo: f64, hi: f64, output: &mut Vec<f64>) {
    if value.is_finite() && value >= lo && value <= hi {
        output.push(value);
    }
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn rounded_clock(value: f64) -> Result<u64, ContinuousError> {
    if !value.is_finite() || value < 0.0 || value >= u64::MAX as f64 {
        Err(ContinuousError::InvalidSpan {
            reason: "clock mapping is not representable",
        })
    } else {
        Ok(libm::round(value) as u64)
    }
}
