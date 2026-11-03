use crate::frontend::ExtruderCaps;
use crate::path::{CurvatureProfile, Segment};

use super::z_caps::crossing_time;

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
    let level = (ratio.log2() * RATIO_LEVELS_PER_OCTAVE).ceil();
    (level / RATIO_LEVELS_PER_OCTAVE).exp2()
}

/// What a segment extrudes per path millimetre: `own` along itself, and
/// `at_joins` the largest of it and its neighbours', which a ratio ramp
/// carries across the joins where the planned acceleration steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ExtrudingRatios {
    pub own: f64,
    pub at_joins: f64,
}

/// One extruding segment's demand on the extruder motor under a speed
/// ceiling `v` and an acceleration budget `A`, from the bounds
/// [`ExtruderCaps`] documents: velocity `r·v + k·r·A`, acceleration
/// `r·A + k·r·J + c·(r·A)²` with `k`, `c` the advance's slope and
/// curvature and `J` the bound on the shaped path's `σ⃛`.
///
/// `σ⃛ = j·t̂ + a_⊥²/|v|` on the shaped path: the jerk's tangential part,
/// at most `G·A` with `G` the largest jerk gain of a shaped axis the segment
/// moves, plus the turning of the tangent under the normal acceleration,
/// `v³κ²` at speed `v` on curvature `κ`, where the disk keeps `v ≤ √(A/κ)`.
/// A follower kernel `w_e` bounds the same term by `G_e·A` on its own,
/// differentiating `σ̈ ≤ A` rather than `σ⃛`.
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
    ratio_at_joins: f64,
}

impl Demand {
    fn velocity(&self, v: f64, accel: f64) -> f64 {
        self.ratio * (v + self.slope * accel)
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
    /// joins' ratio, stay within `instantaneous_corner_velocity`.
    fn velocity_step_accel_cap(&self, instantaneous_corner_velocity: f64) -> f64 {
        let step_per_accel = 2.0 * self.slope * self.ratio_at_joins * self.unshaped_share;
        if self.own_gain.is_finite() || step_per_accel <= 0.0 {
            return f64::INFINITY;
        }
        instantaneous_corner_velocity / step_per_accel
    }
}

/// The speed ceiling and acceleration budget under which `segment`,
/// extruding `ratios` per path millimetre, asks the extruder for no more than
/// its caps. The velocity steps an unshaped leader makes cap the budget
/// first, then the acceleration cap takes the largest budget whose demand
/// fits; when the velocity cap still binds, the ceiling or the budget gives
/// way, whichever lets a profile from rest cross the segment sooner.
pub(super) fn extruder_capped(
    segment: &Segment,
    caps: &ExtruderCaps,
    ratios: ExtrudingRatios,
    kappa_peak: f64,
    (ceiling, accel): (f64, f64),
) -> (f64, f64) {
    let mut path_gain: f64 = 0.0;
    let mut unshaped_plane_sq = 0.0;
    for axis in 0..SPATIAL_AXES {
        let plane = segment.lean_on(axis).plane;
        if plane == 0.0 {
            continue;
        }
        let gain = caps.path_jerk_per_accel_hz[axis];
        if gain.is_finite() {
            path_gain = path_gain.max(gain);
        } else {
            unshaped_plane_sq += plane * plane;
        }
    }
    let demand = Demand {
        ratio: budgeted_ratio(ratios.own),
        slope: caps.advance_slope_s,
        curvature: caps.advance_curvature_s2_per_mm,
        path_gain,
        own_gain: caps.own_jerk_per_accel_hz,
        kappa_peak,
        unshaped_share: unshaped_plane_sq.sqrt().min(1.0),
        ratio_at_joins: ratios.at_joins.max(0.0),
    };
    let accel = accel.min(demand.velocity_step_accel_cap(caps.instantaneous_corner_velocity_mm_s));
    if demand.ratio == 0.0 {
        return (ceiling, accel);
    }
    let accel = largest_fitting_accel(&demand, ceiling, accel, caps.max_accel_mm_s2);
    if demand.velocity(ceiling, accel) <= caps.max_velocity_mm_s {
        return (ceiling, accel);
    }
    velocity_capped(
        &demand,
        caps.max_velocity_mm_s,
        segment.s_len(),
        kappa_peak,
        (ceiling, accel),
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

/// `r·v + k·r·A ≤ V_e`: keep the ceiling and cut the budget, or keep the
/// budget and cut the ceiling, whichever crosses the segment sooner; when
/// neither alone fits, each gets half of `V_e`.
fn velocity_capped(
    demand: &Demand,
    max_velocity: f64,
    length: f64,
    kappa_peak: f64,
    (ceiling, accel): (f64, f64),
) -> (f64, f64) {
    let (ratio, slope) = (demand.ratio, demand.slope);
    let cut_ceiling = (max_velocity - ratio * slope * accel) / ratio;
    if slope == 0.0 {
        return (cut_ceiling, accel);
    }
    let cut_accel = (max_velocity - ratio * ceiling) / (ratio * slope);
    match (cut_ceiling > 0.0, cut_accel > 0.0) {
        (true, true) => {
            let keep_ceiling = (ceiling, cut_accel);
            let keep_accel = (cut_ceiling, accel);
            if crossing_time(length, kappa_peak, keep_ceiling)
                <= crossing_time(length, kappa_peak, keep_accel)
            {
                keep_ceiling
            } else {
                keep_accel
            }
        }
        (true, false) => (cut_ceiling, accel),
        (false, true) => (ceiling, cut_accel),
        (false, false) => (
            0.5 * max_velocity / ratio,
            accel.min(0.5 * max_velocity / (ratio * slope)),
        ),
    }
}

#[cfg(test)]
mod tests;
