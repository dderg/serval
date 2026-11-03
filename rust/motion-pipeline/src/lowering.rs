mod ladder;

#[cfg(test)]
pub(crate) use ladder::ladder_candidate;
pub(crate) use ladder::{
    AccelerationTrend, HeldSample, LADDER_MAX_DEGREE, LADDER_PROBES_U, LadderBudget, LadderFailure,
    LadderMiss, LadderPolicy, LadderPva, LadderSpan, LadderTruth, VelocitySign, eval_mono,
    eval_mono_d, eval_mono_dd, exact_piece, held_miss, ladder_fit, quintic_in_u,
};

use geometry::FollowerDemand;

#[derive(Debug, Clone, Copy)]
pub struct FitTol {
    pub pos_mm: f64,
    pub accel_mm_s2: f64,
}

const FOLLOWER_TOL_SCALE_MIN: f64 = 1e-2;

impl FitTol {
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
