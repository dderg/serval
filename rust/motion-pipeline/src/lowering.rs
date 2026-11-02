mod ladder;

pub(crate) use ladder::{
    AccelerationTrend, LADDER_PROBES_U, LadderBudget, LadderFailure, LadderPolicy, LadderPva,
    LadderSpan, LadderTruth, VelocitySign, exact_piece, ladder_fit, quintic_in_u,
};
#[cfg(feature = "fit-audit")]
pub(crate) use ladder::{eval_mono, eval_mono_dd};

use geometry::FollowerDemand;

#[derive(Debug, Clone, Copy)]
pub struct FitTol {
    pub pos_mm: f64,
    pub accel_mm_s2: f64,
}

const FOLLOWER_TOL_SCALE_MIN: f64 = 1e-2;

/// Acceleration error allowed in proportion to the acceleration being
/// commanded, on top of the absolute `accel_mm_s2` floor.
pub(crate) const ACCEL_RELATIVE_TOL: f64 = 1e-3;

impl FitTol {
    /// Acceleration error allowed where the exact signal commands
    /// `acceleration`.
    pub(crate) fn accel_budget_at(self, acceleration: f64) -> f64 {
        self.accel_mm_s2
            .max(ACCEL_RELATIVE_TOL * acceleration.abs())
    }

    /// Acceleration error allowed across an interval whose exact
    /// acceleration moves monotonically between `left` and `right`.
    pub(crate) fn accel_budget_between(self, left: f64, right: f64) -> f64 {
        if left * right <= 0.0 {
            self.accel_mm_s2
        } else {
            self.accel_budget_at(left.abs().min(right.abs()))
        }
    }

    #[cfg(test)]
    pub(crate) fn scaled(self, factor: f64) -> Self {
        Self {
            pos_mm: self.pos_mm * factor,
            accel_mm_s2: self.accel_mm_s2 * factor,
        }
    }
}

pub(crate) fn follower_tol_scale(followers: &[FollowerDemand], axis: usize) -> f64 {
    followers
        .iter()
        .find(|f| f.axis_index == axis)
        .map_or(1.0, |f| {
            f.max_abs_ratio().clamp(FOLLOWER_TOL_SCALE_MIN, 1.0)
        })
}
