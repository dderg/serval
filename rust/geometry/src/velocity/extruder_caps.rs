use super::extrusion_reach::{ExtrusionBudget, PathInReach};
use crate::frontend::ExtruderCaps;
use crate::path::{CurvatureProfile, Segment};
use crate::ratio_ramp::ADVANCE_VALLEY_SPEED_MM_S;

use super::rate_cap::LinearRate;

const SPATIAL_AXES: usize = 3;
const BISECTION_STEPS: u32 = 64;
/// Extrusion ratios are budgeted at the level above them on a geometric
/// grid of this many levels per octave, at most 19% stricter: neighbouring
/// extrusions a slicer lays at nearly the same ratio then share one
/// acceleration budget, and the planned acceleration steps at few of their
/// joins rather than at nearly every one.
const RATIO_LEVELS_PER_OCTAVE: f64 = 4.0;

/// `ratio` rounded up to its level, 0 for travel and retraction.
fn budgeted_ratio(ratio: f64) -> f64 {
    if ratio <= 0.0 {
        return 0.0;
    }
    let level = libm::ceil(libm::log2(ratio) * RATIO_LEVELS_PER_OCTAVE);
    libm::exp2(level / RATIO_LEVELS_PER_OCTAVE)
}

/// One extruding segment's demand on the extruder motor under a speed
/// ceiling `v` and an acceleration budget `A`, from the bounds
/// [`ExtruderCaps`] documents: velocity `r·v + k·r·A`, acceleration
/// `r·A + k·r·J + c·(r·A)²` with `k`, `c` the advance's slope and
/// curvature and `J` the bound on the shaped path's `σ⃛`.
///
/// `σ⃛ = j·t̂ + a_⊥²/|v|` on the shaped path: the jerk's tangential part,
/// at most `G·A` with `G` the jerk gain of the shaped axes the path moves on
/// within the leaders' spill of the segment, plus the turning of the tangent
/// under the normal acceleration (see [`Demand::turning`]). A follower
/// kernel `w_e` bounds the same term by `G_e·A` on its own, differentiating
/// `σ̈ ≤ A` rather than `σ⃛`.
///
/// An unshaped leader steps `σ̈` itself wherever its planned acceleration
/// steps, by at most `2A·U` with `U` the share of the segment's plane on
/// the unshaped axes; with no follower kernel the advance turns that into a
/// velocity step `k·r·2A·U`, held within the extruder's instantaneous
/// corner velocity instead of its acceleration.
struct Demand {
    ratio: f64,
    slope: f64,
    curvature: f64,
    path_gain: f64,
    own_gain: f64,
    path: PathInReach,
    accel_mass: f64,
    unshaped_share: f64,
    ratio_in_reach: f64,
}

impl Demand {
    fn velocity(&self, v: f64, accel: f64) -> f64 {
        self.ratio * (v + self.slope * accel)
    }

    fn velocity_rate(&self) -> LinearRate {
        LinearRate {
            per_velocity: self.ratio,
            per_accel: self.ratio * self.slope,
        }
    }

    /// The turning of the shaped tangent, `a_⊥²/|v|`, around the segment
    /// under the budget `A` at speeds up to `v`. The shaped path averages the
    /// planned path within the spill, whose headings lie within `acos c` of
    /// their mean (`c` the path's speed share): its normal acceleration takes
    /// the planned normal acceleration, at most `min(A, v²κ)`, and the
    /// planned tangential acceleration turned by up to `2·acos c` off the
    /// shaped tangent, never more than the kernels' mass `m` times `A`; its
    /// speed keeps the share `c` of the slowest the path is driven there, at
    /// most `√(A/κ)` on its peak curvature `κ` and its slowest ceiling. Below
    /// [`ADVANCE_VALLEY_SPEED_MM_S`] the advance lays a valley instead.
    fn turning(&self, v: f64, accel: f64) -> f64 {
        let path = &self.path;
        if path.speed_share >= 1.0 && path.kappa_peak_per_mm == 0.0 {
            return 0.0;
        }
        let normal = accel.min(v * v * path.kappa_peak_per_mm);
        let turn_off_tangent = 2.0 * libm::acos(path.speed_share.clamp(0.0, 1.0));
        let swing = libm::sin(turn_off_tangent.min(std::f64::consts::FRAC_PI_2));
        let shaped_normal = (self.accel_mass * accel).min(accel * swing + normal);
        let slowest = v
            .min(path.slowest_ceiling_mm_s)
            .min((accel / path.kappa_peak_per_mm).sqrt());
        let shaped_speed = (slowest * path.speed_share).max(ADVANCE_VALLEY_SPEED_MM_S);
        shaped_normal * shaped_normal / shaped_speed
    }

    fn accel(&self, v: f64, accel: f64) -> f64 {
        let shaped_jerk = self.path_gain * accel + self.turning(v, accel);
        let own_jerk = self.own_gain * accel;
        let path_jerk = if self.own_gain.is_infinite() {
            shaped_jerk
        } else if self.unshaped_share > 0.0 {
            own_jerk
        } else {
            shaped_jerk.min(own_jerk)
        };
        let base = self.ratio * accel;
        base + self.slope * self.ratio * path_jerk + self.curvature * base * base
    }

    /// The largest budget whose advance velocity steps, `k·r·2A·U` at the
    /// ratio in reach, stay within `instantaneous_corner_velocity`.
    fn velocity_step_accel_cap(&self, instantaneous_corner_velocity: f64) -> f64 {
        let step_per_accel = 2.0 * self.slope * self.ratio_in_reach * self.unshaped_share;
        if self.own_gain.is_finite() || step_per_accel <= 0.0 {
            return f64::INFINITY;
        }
        instantaneous_corner_velocity / step_per_accel
    }
}

/// The speed ceiling and acceleration budget under which `segment`, with
/// `budget` what its extruder demand can meet around it (see
/// [`super::extrusion_reach::ExtrusionReach`]), asks the extruder for no more than
/// its caps. The velocity steps an unshaped leader makes cap the budget
/// first, then the acceleration cap takes the largest budget whose demand
/// fits; when the velocity cap still binds, the ceiling or the budget gives
/// way, whichever lets a profile from rest cross the segment sooner.
pub(super) fn extruder_capped(
    segment: &Segment,
    caps: &ExtruderCaps,
    budget: &ExtrusionBudget,
    kappa_peak: f64,
    (ceiling, accel): (f64, f64),
) -> (f64, f64) {
    let ratio_in_reach = budget.ratio_in_reach;
    let unshaped_plane_sq: f64 = (0..SPATIAL_AXES)
        .filter(|&axis| caps.unshaped_leaders[axis])
        .map(|axis| segment.lean_on(axis).plane.powi(2))
        .sum();
    let demand = Demand {
        ratio: budgeted_ratio(ratio_in_reach),
        slope: caps.advance_slope_s,
        curvature: caps.advance_curvature_s2_per_mm,
        path_gain: caps.path_jerk_gain(budget.path_in_reach.axes),
        own_gain: caps.own_jerk_per_accel_hz,
        path: budget.path_in_reach,
        accel_mass: caps.leader_spill.mass,
        unshaped_share: unshaped_plane_sq.sqrt().min(1.0),
        ratio_in_reach: ratio_in_reach.max(0.0),
    };
    let accel = accel.min(demand.velocity_step_accel_cap(caps.instantaneous_corner_velocity_mm_s));
    if demand.ratio == 0.0 {
        return (ceiling, accel);
    }
    let accel = largest_fitting_accel(&demand, ceiling, accel, caps.max_accel_mm_s2);
    if demand.velocity(ceiling, accel) <= caps.max_velocity_mm_s {
        return (ceiling, accel);
    }
    demand.velocity_rate().capped(
        caps.max_velocity_mm_s,
        segment.s_len(),
        kappa_peak,
        (ceiling, accel),
    )
}

/// The extruder motor's velocity, `r·v + k·r·A`, at the ratio a segment
/// with `ratio_in_reach` is budgeted at.
pub(super) fn velocity_rate(caps: &ExtruderCaps, ratio_in_reach: f64) -> LinearRate {
    let ratio = budgeted_ratio(ratio_in_reach);
    LinearRate {
        per_velocity: ratio,
        per_accel: ratio * caps.advance_slope_s,
    }
}

/// The largest budget up to `accel` whose acceleration demand at `ceiling`
/// fits `max_accel`; the demand grows with the budget, so bisection finds it.
fn largest_fitting_accel(demand: &Demand, ceiling: f64, accel: f64, max_accel: f64) -> f64 {
    if demand.accel(ceiling, accel) <= max_accel {
        return accel;
    }
    let (mut fits, mut exceeds) = (0.0, accel);
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (fits + exceeds);
        if demand.accel(ceiling, mid) <= max_accel {
            fits = mid;
        } else {
            exceeds = mid;
        }
    }
    fits
}

/// The ratio a segment lays on `axis` at its start and end, 0 where it
/// lays none.
fn ratios_on(segment: &crate::path::PathSegment, axis: usize) -> (f64, f64) {
    segment
        .followers
        .iter()
        .find(|follower| follower.axis_index == axis)
        .map_or((0.0, 0.0), |follower| (follower.ratio, follower.ratio_end))
}

/// The speed ceiling at the join of `up` into `dn` where the extrusion
/// ratio moves: the follower's ratio ramp spreads the step there, with the
/// change along either segment (a blend's ratio slides from one move's to
/// the next's over a path a window gathers whole), over at most the longest
/// ramp time, so the leaders' shaped speed there must stay where that ramp
/// keeps the advanced follower within its limits under the budget `accel`
/// around the join ([`crate::RatioRampRule::step_speed_cap`]; the path's
/// jerk is the budget's own to hold, no slower crossing lowers it), and the
/// planned speed lower still by what the leaders' kernels spill into it.
/// Infinite where the ratio does not move, or no path carries it.
pub(super) fn ratio_step_ceiling(up: &crate::Move, dn: &crate::Move, accel: f64) -> f64 {
    let Some(caps) = ExtruderCaps::stricter(up.limits.extruder, dn.limits.extruder) else {
        return f64::INFINITY;
    };
    if up.segment.spatial.is_none() || dn.segment.spatial.is_none() {
        return f64::INFINITY;
    }
    let (up_start, up_end) = ratios_on(&up.segment, caps.axis_index);
    let (dn_start, dn_end) = ratios_on(&dn.segment, caps.axis_index);
    let moved = (up_end - up_start).abs() + (dn_start - up_end).abs() + (dn_end - dn_start).abs();
    let peak = [up_start, up_end, dn_start, dn_end]
        .iter()
        .fold(0.0_f64, |peak, ratio| peak.max(ratio.abs()));
    let shaped = caps
        .ratio_ramp_rule()
        .step_speed_cap(moved, peak, accel, 0.0);
    if shaped.is_infinite() {
        return shaped;
    }
    caps.leader_spill.planned_speed(shaped, accel)
}

#[cfg(test)]
mod tests;
