use crate::GeometryError;
use crate::path::{Line, PathSegment, Segment};
use crate::segment::{FollowerDemand, SourceRange};
use crate::velocity::law::RAIL_ACCELERATION_DEFECT;

mod leader_spill;
pub use leader_spill::{LeaderSpill, LeaderSpillId, SPILL_CELLS};
mod rest_settle;
pub use rest_settle::{
    RestSettle, RestSettleId, SETTLE_CELLS, SETTLE_SHARE_OF_PATH_DEVIATION, SettleDemand,
    SettleSide,
};

const DISPLACEMENT_EPSILON: f64 = 1e-9;

// TODO: a DerivativeGains chain stage with k2 != 0 demands extra motor
// acceleration proportional to jerk; fold that demand into these planning
// limits once the mode-inverse post-processor lands.
pub const CORNER_DEVIATION_SCV_FACTOR: f64 = std::f64::consts::SQRT_2 - 1.0;

#[must_use]
pub fn corner_deviation_from_scv(scv_mm_s: f64, accel_mm_s2: f64) -> f64 {
    scv_mm_s * scv_mm_s * CORNER_DEVIATION_SCV_FACTOR / accel_mm_s2
}

#[must_use]
pub fn scv_from_corner_deviation(corner_deviation_mm: f64, accel_mm_s2: f64) -> f64 {
    (corner_deviation_mm * accel_mm_s2 / CORNER_DEVIATION_SCV_FACTOR).sqrt()
}

/// The corner budget a move is planned under.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CornerBudget {
    /// `corner_deviation`: the shaped path misses every corner's vertex by at most this many mm.
    Shaped(f64),
    /// Neither corner option set: the shaper's own rounding of the corner at its planned
    /// acceleration, plus this many mm.
    OverFloor(f64),
    /// `square_corner_velocity` (mm/s): that rounding plus what a sharp 90° corner taken at
    /// this speed adds to it.
    ScvEquivalent(f64),
}

/// The allowance over the shaper's floor a `[printer]` that sets neither corner option gets,
/// in mm. The same at every `max_accel`: see `docs/Config_Reference_Motion.md` for the
/// profile study behind it.
pub const DEFAULT_CORNER_ALLOWANCE_MM: f64 = 0.04;

impl CornerBudget {
    /// The raw cut the fitter spends while no shaper prediction applies.
    #[must_use]
    pub fn unshaped_cut_mm(self, accel_mm_s2: f64) -> f64 {
        match self {
            Self::Shaped(mm) | Self::OverFloor(mm) => mm,
            Self::ScvEquivalent(scv) => corner_deviation_from_scv(scv, accel_mm_s2),
        }
    }

    /// The budget holding both `self` and `other`, for a segment that inherits from two
    /// moves: the smaller payload of a shared kind, else the one cutting less at `accel_mm_s2`.
    #[must_use]
    pub fn stricter(self, other: Self, accel_mm_s2: f64) -> Self {
        match (self, other) {
            (Self::Shaped(a), Self::Shaped(b)) => Self::Shaped(a.min(b)),
            (Self::OverFloor(a), Self::OverFloor(b)) => Self::OverFloor(a.min(b)),
            (Self::ScvEquivalent(a), Self::ScvEquivalent(b)) => Self::ScvEquivalent(a.min(b)),
            _ if other.unshaped_cut_mm(accel_mm_s2) < self.unshaped_cut_mm(accel_mm_s2) => other,
            _ => self,
        }
    }

    pub fn validate(self) -> Result<(), &'static str> {
        let (Self::Shaped(value) | Self::OverFloor(value) | Self::ScvEquivalent(value)) = self;
        if value.is_finite() && value >= 0.0 {
            Ok(())
        } else {
            Err("the corner budget must be finite and non-negative")
        }
    }
}

/// A move's motion caps. The first four bound the toolhead path; the z caps
/// bound the z axis alone, so the planner applies them to whatever share of
/// the path's motion each fitted segment puts on z. `try_new` leaves z
/// uncapped beyond the path caps; `with_z_caps` sets them. The extruder caps
/// bound the motor of the follower axis the move extrudes on, see
/// [`ExtruderCaps`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityLimits {
    pub max_velocity_mm_s: f64,
    pub accel_mm_s2: f64,
    pub corner: CornerBudget,
    pub max_jerk_mm_s3: f64,
    pub max_z_velocity_mm_s: f64,
    pub max_z_accel_mm_s2: f64,
    pub extruder: Option<ExtruderCaps>,
}

/// What the extruder's motor can do and how its command follows the path.
///
/// A follower extrudes `r` per millimetre of the shaped path, so it runs at
/// `r·σ̇` and its advance `a(ė)` adds `a′·r·σ̈` to that velocity and
/// `a′·r·σ⃛ + a″·(r·σ̈)²` to its acceleration, where `σ` is the shaped
/// path's arc length. `σ̈` never exceeds the planned acceleration `A` (a
/// unit-mass kernel averages it); `σ⃛` is bounded per unit of `A` by the
/// smoothing kernels the jerk passes through (see
/// [`ExtruderCaps::path_jerk_per_accel_hz`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtruderCaps {
    pub axis_index: usize,
    pub max_velocity_mm_s: f64,
    pub max_accel_mm_s2: f64,
    /// The advance's largest slope `sup da/dv`: `k` of a linear advance, the
    /// small-signal slope of a nonlinear one, 0 without an advance.
    pub advance_slope_s: f64,
    /// The advance's largest `|d²a/dv²|`, 0 for a linear advance.
    pub advance_curvature_s2_per_mm: f64,
    /// By the set of spatial axes the path moves on around a segment (see
    /// [`ExtruderCaps::path_jerk_gain`]), the shaped path's tangential jerk
    /// per unit of planned acceleration through the kernels of the shaped
    /// axes the follower follows among them: a planned acceleration within
    /// the disk anywhere in the kernels' reach leaves at most
    /// `A·max_d ∫ |(dᵢ·wᵢ′(u))ᵢ| du` along the shaped tangent `d`
    /// ([`nurbs::algebra::PiecewisePolynomialKernel::directional_variation`]),
    /// every lobe of a multimodal kernel taking a fresh swing of it. 0 for a
    /// set without a shaped followed axis.
    pub path_jerk_per_accel_hz: [f64; SPATIAL_AXIS_SETS],
    /// By the same sets, the shaped path's jerk per unit of planned
    /// acceleration a *switch* the kernels meet alone leaves: a switch
    /// steps the acceleration by up to twice the budget (decelerating into
    /// a seam and accelerating out of it, a triangle's apex), and a step
    /// `Δa` leaves at most `|Δa|·maxᵢ max |wᵢ|` along any direction
    /// ([`nurbs::algebra::PiecewisePolynomialKernel::directional_peak`]).
    /// The variation of a unimodal kernel, less for a multimodal one. 0 for
    /// a set without a shaped followed axis.
    pub path_jerk_per_accel_switch_hz: [f64; SPATIAL_AXIS_SETS],
    /// The followed spatial axes no kernel shapes: their planned
    /// acceleration steps reach the advance unsmoothed.
    pub unshaped_leaders: [bool; 3],
    /// The same bound through the follower's own kernel, which differentiates
    /// the advance's input `σ̈ ≤ A` itself: the kernel's total variation,
    /// infinite without one.
    pub own_jerk_per_accel_hz: f64,
    /// The velocity step the motor takes at once where no kernel smooths
    /// the advance: an unshaped followed axis steps the shaped path's
    /// acceleration, and the advance turns each step into one.
    pub instantaneous_corner_velocity_mm_s: f64,
    /// What the kernels of the spatial axes the follower follows make of
    /// the planned speed (see [`LeaderSpill`]): the planner bounds the
    /// shaped speed at a ratio step with it.
    pub leader_spill: LeaderSpillId,
    /// How far either side of an instant the kernels of the leaders the
    /// follower rides reach: the shaped path there averages the planned
    /// path within it.
    pub leader_kernel_reach_s: f64,
    /// How the shaped path settles onto a point the planned path rests at
    /// (see [`RestSettle`]): an extrude-only move holds the toolhead for it.
    pub rest_settle: RestSettleId,
}

impl ExtruderCaps {
    /// The caps holding both `a` and `b`, for a segment that inherits from
    /// two moves: the lower limits, the steeper advance, the larger gains.
    #[must_use]
    pub fn stricter(a: Option<Self>, b: Option<Self>) -> Option<Self> {
        let (Some(a), Some(b)) = (a, b) else {
            return a.or(b);
        };
        assert_eq!(
            a.axis_index, b.axis_index,
            "one fitted segment cannot extrude on two follower axes"
        );
        let larger = |a: [f64; SPATIAL_AXIS_SETS], b: [f64; SPATIAL_AXIS_SETS]| {
            std::array::from_fn(|set| a[set].max(b[set]))
        };
        let unshaped_leaders =
            std::array::from_fn(|axis| a.unshaped_leaders[axis] || b.unshaped_leaders[axis]);
        Some(Self {
            axis_index: a.axis_index,
            max_velocity_mm_s: a.max_velocity_mm_s.min(b.max_velocity_mm_s),
            max_accel_mm_s2: a.max_accel_mm_s2.min(b.max_accel_mm_s2),
            advance_slope_s: a.advance_slope_s.max(b.advance_slope_s),
            advance_curvature_s2_per_mm: a
                .advance_curvature_s2_per_mm
                .max(b.advance_curvature_s2_per_mm),
            path_jerk_per_accel_hz: larger(a.path_jerk_per_accel_hz, b.path_jerk_per_accel_hz),
            path_jerk_per_accel_switch_hz: larger(
                a.path_jerk_per_accel_switch_hz,
                b.path_jerk_per_accel_switch_hz,
            ),
            unshaped_leaders,
            own_jerk_per_accel_hz: a.own_jerk_per_accel_hz.max(b.own_jerk_per_accel_hz),
            instantaneous_corner_velocity_mm_s: a
                .instantaneous_corner_velocity_mm_s
                .min(b.instantaneous_corner_velocity_mm_s),
            leader_spill: a.leader_spill.wider(b.leader_spill),
            leader_kernel_reach_s: a.leader_kernel_reach_s.max(b.leader_kernel_reach_s),
            rest_settle: a.rest_settle.wider(b.rest_settle),
        })
    }

    /// The jerk gain of the path around a segment that moves on `axes`.
    #[must_use]
    pub fn path_jerk_gain(&self, axes: [bool; 3]) -> f64 {
        self.path_jerk_per_accel_hz[axis_set(axes)]
    }

    /// The jerk gain the planner budgets the shaped path around a segment
    /// that moves on `axes` at. Without a follower kernel, the gain of a
    /// switch the kernels meet alone
    /// ([`Self::path_jerk_per_accel_switch_hz`]): switches closer together
    /// than the kernels' reach ask more, which the turning guard holds
    /// within the extruder's limits. With one, the total variation
    /// ([`Self::path_jerk_gain`]), which nothing behind the planner checks.
    #[must_use]
    pub fn planned_path_jerk_gain(&self, axes: [bool; 3]) -> f64 {
        if self.own_jerk_per_accel_hz.is_infinite() {
            self.path_jerk_per_accel_switch_hz[axis_set(axes)]
        } else {
            self.path_jerk_gain(axes)
        }
    }

    /// The rule the follower's ratio ramp derives each step's ramp time by.
    #[must_use]
    pub fn ratio_ramp_rule(&self) -> crate::RatioRampRule {
        crate::RatioRampRule {
            advance_slope_s: self.advance_slope_s,
            max_velocity_mm_s: self.max_velocity_mm_s,
            max_accel_mm_s2: self.max_accel_mm_s2,
            kernel_jerk_per_accel_hz: self.own_jerk_per_accel_hz,
        }
    }

    fn check(&self) -> Result<(), &'static str> {
        if !(self.max_velocity_mm_s > 0.0 && self.max_accel_mm_s2 > 0.0) {
            return Err("extruder caps must be positive");
        }
        if !(self.instantaneous_corner_velocity_mm_s.is_finite()
            && self.instantaneous_corner_velocity_mm_s > 0.0)
        {
            return Err("the extruder's instantaneous corner velocity must be finite and positive");
        }
        if !(self.advance_slope_s.is_finite()
            && self.advance_slope_s >= 0.0
            && self.advance_curvature_s2_per_mm.is_finite()
            && self.advance_curvature_s2_per_mm >= 0.0)
        {
            return Err("the advance's slope and curvature must be finite and non-negative");
        }
        if !self
            .path_jerk_per_accel_hz
            .iter()
            .chain(&self.path_jerk_per_accel_switch_hz)
            .all(|gain| gain.is_finite() && *gain >= 0.0)
        {
            return Err("path jerk gains must be finite and non-negative");
        }
        if !(self.own_jerk_per_accel_hz >= 0.0) {
            return Err("the follower kernel's jerk gain must be non-negative");
        }
        self.leader_spill.spill().check()?;
        if !(self.leader_kernel_reach_s.is_finite() && self.leader_kernel_reach_s >= 0.0) {
            return Err("the leaders' kernel reach must be finite and non-negative");
        }
        self.rest_settle.settle().check()?;
        Ok(())
    }
}

/// How many sets of spatial axes [`ExtruderCaps::path_jerk_per_accel_hz`]
/// holds a gain for: bit `i` of a set's index stands for axis `i`.
pub const SPATIAL_AXIS_SETS: usize = 8;

/// The index of the set of the spatial axes `axes` marks.
#[must_use]
pub fn axis_set(axes: [bool; 3]) -> usize {
    axes.iter()
        .enumerate()
        .filter(|(_, moves)| **moves)
        .map(|(axis, _)| 1 << axis)
        .sum()
}

impl VelocityLimits {
    pub fn try_new(
        max_velocity_mm_s: f64,
        accel_mm_s2: f64,
        corner: CornerBudget,
        max_jerk_mm_s3: f64,
    ) -> Result<Self, &'static str> {
        let limits = Self {
            max_velocity_mm_s,
            accel_mm_s2,
            corner,
            max_jerk_mm_s3,
            max_z_velocity_mm_s: f64::INFINITY,
            max_z_accel_mm_s2: f64::INFINITY,
            extruder: None,
        };
        limits.check()?;
        Ok(limits)
    }

    pub fn with_z_caps(
        self,
        max_z_velocity_mm_s: f64,
        max_z_accel_mm_s2: f64,
    ) -> Result<Self, &'static str> {
        let limits = Self {
            max_z_velocity_mm_s,
            max_z_accel_mm_s2,
            ..self
        };
        limits.check()?;
        Ok(limits)
    }

    pub fn with_extruder_caps(self, extruder: Option<ExtruderCaps>) -> Result<Self, &'static str> {
        let limits = Self { extruder, ..self };
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
        self.corner.validate()?;
        if !(self.max_jerk_mm_s3 > 0.0) {
            return Err("max_jerk must be positive (infinity disables jerk limiting)");
        }
        if !(self.max_z_velocity_mm_s > 0.0 && self.max_z_accel_mm_s2 > 0.0) {
            return Err("z caps must be positive (infinity leaves z to the path caps)");
        }
        if !(self.max_z_accel_mm_s2 > RAIL_ACCELERATION_DEFECT) {
            return Err("max_z_accel must exceed the disk rail's acceleration defect");
        }
        self.extruder.as_ref().map_or(Ok(()), ExtruderCaps::check)
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
    ZeroMotion { line_no: u32 },
    NonFiniteInput { line_no: u32 },
    InvalidFeedrate { line_no: u32 },
    InvalidLimits { line_no: u32, reason: &'static str },
    Segment { line_no: u32, source: GeometryError },
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

    let spatial_distance = crate::vec3::dist(start, end);
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
    FollowerDemand::constant(axis_index, ratio)
}

fn segment_err(line_no: u32) -> impl Fn(GeometryError) -> FrontendError {
    move |source| FrontendError::Segment { line_no, source }
}

fn coords_finite(points: &[[f64; 3]]) -> bool {
    points.iter().flatten().all(|c| c.is_finite())
}

#[cfg(test)]
mod tests;
