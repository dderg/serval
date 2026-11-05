use super::extrusion_reach::ExtrusionBudget;
use crate::frontend::ExtruderCaps;
use crate::path::{CurvatureProfile, Segment};
use crate::ratio_ramp::StepCrossing;

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
/// under the normal acceleration, `v³κ²` at speed `v` on curvature `κ`,
/// where the disk keeps `v ≤ √(A/κ)`. A follower kernel `w_e` bounds the
/// same term by `G_e·A` on its own, differentiating `σ̈ ≤ A` rather than
/// `σ⃛`.
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
    kappa_peak: f64,
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

    fn accel(&self, v: f64, accel: f64) -> f64 {
        let turning_speed = v.min((accel / self.kappa_peak).sqrt());
        let turning = turning_speed.powi(3) * self.kappa_peak * self.kappa_peak;
        let shaped_jerk = self.path_gain * accel + turning;
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
        path_gain: caps.path_jerk_gain(budget.axes_in_reach),
        own_gain: caps.own_jerk_per_accel_hz,
        kappa_peak,
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

/// A move that retracts while the toolhead travels, a wipe, is a retract:
/// no advance leads it, and as in Klipper its filament motion `|r|·v` keeps
/// the extrude-only caps, `|r|` its steepest retracting ratio.
pub(super) fn moving_retract_capped(
    segment: &crate::path::PathSegment,
    (velocity_cap, accel_cap): (f64, f64),
    (ceiling, accel): (f64, f64),
) -> (f64, f64) {
    let retract = segment
        .followers
        .iter()
        .map(|follower| -follower.ratio.min(follower.ratio_end))
        .fold(0.0, f64::max);
    if retract <= 0.0 {
        return (ceiling, accel);
    }
    (
        ceiling.min(velocity_cap / retract),
        accel.min(accel_cap / retract),
    )
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
/// ramp time, and over less where the leaders rest within the room `dn`'s
/// budget gives, so the leaders' shaped speed there must stay where that
/// ramp keeps the advanced follower within its limits under the budget
/// `accel` around the join ([`crate::RatioRampRule::step_speed_cap`]; the
/// path's jerk is the budget's own to hold, no slower crossing lowers it).
/// The leaders' own advanced flow there is held by the moves' own caps
/// ([`own_flow_bound`]), and the shaped speed by the fastest planned
/// ceiling within either budget's reach, whose [`crate::LeaderSpill`]
/// gives the planned speed that keeps the shaped one within the cap.
/// Infinite where the ratio does not move, no path carries it, or the
/// leaders cross it within its limits at any speed they reach.
pub(super) fn ratio_step_ceiling(
    up: &crate::Move,
    dn: &crate::Move,
    (up_budget, dn_budget): (&ExtrusionBudget, &ExtrusionBudget),
    accel: f64,
) -> f64 {
    let Some(caps) = ExtruderCaps::stricter(up.limits.extruder, dn.limits.extruder) else {
        return f64::INFINITY;
    };
    if up.segment.spatial.is_none() || dn.segment.spatial.is_none() {
        return f64::INFINITY;
    }
    let (up_start, up_end) = ratios_on(&up.segment, caps.axis_index);
    let (dn_start, dn_end) = ratios_on(&dn.segment, caps.axis_index);
    let ratios = [up_start, up_end, dn_start, dn_end];
    let moved = (up_end - up_start).abs() + (dn_start - up_end).abs() + (dn_end - dn_start).abs();
    if moved == 0.0 {
        return f64::INFINITY;
    }
    let peak = ratios
        .iter()
        .fold(0.0_f64, |peak, ratio| peak.max(ratio.abs()));
    let fastest = up_budget
        .fastest_ceiling_mm_s
        .min(dn_budget.fastest_ceiling_mm_s);
    assert!(
        fastest.is_finite(),
        "the join into line {} extrudes with no extruder budget reaching it",
        dn.source.start_line
    );
    let shaped = caps.ratio_ramp_rule().step_speed_cap(
        &StepCrossing {
            ratio_step: moved,
            peak_ratio: peak,
            accel_mm_s2: accel,
            jerk_mm_s3: 0.0,
            rest_room_mm: dn_budget.rest_room_mm,
            own_flow_bound_mm_s: own_flow_bound(&caps, ratios),
        },
        caps.leader_spill.mass * fastest,
    );
    if shaped.is_infinite() {
        return shaped;
    }
    caps.leader_spill.planned_speed(shaped, fastest, accel)
}

/// The most the leaders' own advanced flow `r·(σ̇ + k·σ̈)` reaches across
/// a join extruding `ratios` either side of it. The kernels shape it
/// linearly from the planned `v + k·a` of the moves within their reach of
/// the join, so it is at most `r·mass·max (cⱼ + k·Aⱼ)` over those moves'
/// ceilings and budgets; each of them budgets at least the join's peak
/// ratio `r`, at its level `r_b` or above, and holds `r_b·(cⱼ + k·Aⱼ)`
/// within the extruder's velocity cap ([`extruder_capped`]). A retraction
/// is held by the extrude-only caps instead: unbounded here.
fn own_flow_bound(caps: &ExtruderCaps, ratios: [f64; 4]) -> f64 {
    if ratios.iter().any(|ratio| *ratio < 0.0) {
        return f64::INFINITY;
    }
    let peak = ratios.iter().fold(0.0_f64, |peak, ratio| peak.max(*ratio));
    caps.leader_spill.mass * caps.max_velocity_mm_s * peak / budgeted_ratio(peak)
}

#[cfg(test)]
mod noise_step_tests;
#[cfg(test)]
mod tests;
