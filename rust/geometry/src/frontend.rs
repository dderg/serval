pub mod arc;

use crate::GeometryError;
use crate::path::{CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::{FollowerDemand, SourceRange};

const DISPLACEMENT_EPSILON: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityLimits {
    pub max_velocity_mm_s: f64,
    pub accel_mm_s2: f64,
    pub square_corner_velocity_mm_s: f64,
    pub max_jerk_mm_s3: f64,
}

impl VelocityLimits {
    pub fn try_new(
        max_velocity_mm_s: f64,
        accel_mm_s2: f64,
        square_corner_velocity_mm_s: f64,
        max_jerk_mm_s3: f64,
    ) -> Result<Self, &'static str> {
        let limits = Self {
            max_velocity_mm_s,
            accel_mm_s2,
            square_corner_velocity_mm_s,
            max_jerk_mm_s3,
        };
        limits.check()?;
        Ok(limits)
    }

    fn check(&self) -> Result<(), &'static str> {
        if !(self.max_velocity_mm_s.is_finite() && self.max_velocity_mm_s > 0.0) {
            return Err("max_velocity must be finite and positive");
        }
        if !(self.accel_mm_s2.is_finite() && self.accel_mm_s2 > 0.0) {
            return Err("accel must be finite and positive");
        }
        if !(self.square_corner_velocity_mm_s.is_finite()
            && self.square_corner_velocity_mm_s >= 0.0)
        {
            return Err("square_corner_velocity must be finite and non-negative");
        }
        if !(self.max_jerk_mm_s3.is_finite() && self.max_jerk_mm_s3 > 0.0) {
            return Err("max_jerk must be finite and positive");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveContext {
    pub extruder_axis: usize,
    pub feedrate_mm_s: f64,
    pub limits: VelocityLimits,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Move {
    pub segment: PathSegment,
    pub feedrate_mm_s: f64,
    pub limits: VelocityLimits,
    pub source: SourceRange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrontendError {
    ZeroMotion {
        line_no: u32,
    },
    NonFiniteInput {
        line_no: u32,
    },
    HelicalArc {
        line_no: u32,
    },
    ArcRadiusMismatch {
        line_no: u32,
        radius: f64,
        end_radius: f64,
    },
    InvalidFeedrate {
        line_no: u32,
    },
    InvalidLimits {
        line_no: u32,
        reason: &'static str,
    },
    Segment {
        line_no: u32,
        source: GeometryError,
    },
}

pub fn line_move(
    start: [f64; 3],
    end: [f64; 3],
    e_delta: f64,
    ctx: MoveContext,
) -> Result<Move, FrontendError> {
    ctx.validate()?;
    let line_no = ctx.source.start_line;
    if !(coords_finite(&[start, end]) && e_delta.is_finite()) {
        return Err(FrontendError::NonFiniteInput { line_no });
    }

    let spatial_distance = euclidean_distance(start, end);
    let has_spatial = spatial_distance > DISPLACEMENT_EPSILON;
    let has_extrusion = e_delta.abs() > DISPLACEMENT_EPSILON;

    if has_spatial {
        let line = Line::try_new(start, end).map_err(segment_err(line_no))?;
        let followers = if has_extrusion {
            vec![extruder_follower(
                ctx.extruder_axis,
                e_delta / spatial_distance,
            )]
        } else {
            Vec::new()
        };
        let segment =
            PathSegment::try_new(Segment::Line(line), followers).map_err(segment_err(line_no))?;
        return Ok(ctx.into_move(segment));
    }

    if has_extrusion {
        let virtual_path_mm = e_delta.abs();
        let followers = vec![extruder_follower(
            ctx.extruder_axis,
            e_delta / virtual_path_mm,
        )];
        let segment = PathSegment::try_new_virtual(followers, virtual_path_mm)
            .map_err(segment_err(line_no))?;
        return Ok(ctx.into_move(segment));
    }

    Err(FrontendError::ZeroMotion { line_no })
}

pub fn arc_move(
    start: [f64; 3],
    end: [f64; 3],
    i: f64,
    j: f64,
    ccw: bool,
    e_delta: f64,
    ctx: MoveContext,
) -> Result<Move, FrontendError> {
    ctx.validate()?;
    let line_no = ctx.source.start_line;
    if !(coords_finite(&[start, end]) && i.is_finite() && j.is_finite() && e_delta.is_finite()) {
        return Err(FrontendError::NonFiniteInput { line_no });
    }

    let arc = arc::build_arc(start, end, i, j, ccw, line_no)?;
    let arc_length = arc.s_len();
    let followers = if e_delta.abs() > DISPLACEMENT_EPSILON {
        vec![extruder_follower(ctx.extruder_axis, e_delta / arc_length)]
    } else {
        Vec::new()
    };
    let segment =
        PathSegment::try_new(Segment::Arc(arc), followers).map_err(segment_err(line_no))?;
    Ok(ctx.into_move(segment))
}

impl MoveContext {
    fn validate(&self) -> Result<(), FrontendError> {
        let line_no = self.source.start_line;
        if !(self.feedrate_mm_s.is_finite() && self.feedrate_mm_s > 0.0) {
            return Err(FrontendError::InvalidFeedrate { line_no });
        }
        self.limits
            .check()
            .map_err(|reason| FrontendError::InvalidLimits { line_no, reason })
    }

    fn into_move(self, segment: PathSegment) -> Move {
        Move {
            segment,
            feedrate_mm_s: self.feedrate_mm_s,
            limits: self.limits,
            source: self.source,
        }
    }
}

fn extruder_follower(axis_index: usize, ratio: f64) -> FollowerDemand {
    FollowerDemand { axis_index, ratio }
}

fn segment_err(line_no: u32) -> impl Fn(GeometryError) -> FrontendError {
    move |source| FrontendError::Segment { line_no, source }
}

fn coords_finite(points: &[[f64; 3]]) -> bool {
    points.iter().flatten().all(|c| c.is_finite())
}

fn euclidean_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt()
}

#[cfg(test)]
mod tests;
