mod ladder;

#[cfg(feature = "fit-audit")]
pub(crate) use ladder::eval_mono;
#[cfg(any(test, feature = "fit-audit"))]
pub(crate) use ladder::eval_mono_dd;
#[cfg(test)]
pub(crate) use ladder::ladder_candidate;
pub(crate) use ladder::{
    AccelerationExemption, AccelerationTrend, HeldSample, LADDER_MAX_DEGREE, LADDER_PROBES_U,
    LadderBudget, LadderFailure, LadderMiss, LadderPolicy, LadderPva, LadderSpan, LadderTruth,
    SeamContinuity, VelocitySign, acceleration_bound, exact_piece, held_miss, ladder_accept,
    ladder_fit, quintic_in_u,
};

use geometry::FollowerDemand;

#[derive(Debug, Clone, Copy)]
pub struct FitTol {
    pub pos_mm: f64,
    pub accel_mm_s2: f64,
    pub accel_budget: AccelBudget,
}

/// How a fit holds a piece's acceleration error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccelBudget {
    /// `accel_mm_s2` on every piece. A toolhead's acceleration error is
    /// force the frame feels, whatever it does to position.
    Absolute,
    /// On a piece of duration `h`, the looser of `accel_mm_s2` and
    /// `8·pos_mm/h²`. Every accepted piece meets the signal's position at
    /// both ends, and an error pinned at both ends with
    /// `|δ''| ≤ 8·pos_mm/h²` stays within `pos_mm` between them
    /// (`|δ(t)| ≤ max|δ''|·t·(h − t)/2`): the acceleration
    /// budget spends the position budget and no more. A follower's
    /// acceleration moves nothing but material, and the nozzle's melt lag
    /// `K` (`e_cmd = e_out + K·ė_out`) turns a command error held within
    /// `pos_mm` into a nozzle-flow error of at most `2·pos_mm/K` whatever
    /// its acceleration; the acceleration error bounds the command's own
    /// feed-rate error, which that lag filters away. Its motor still holds
    /// `accel_limit_mm_s2`: a piece the looser budget would carry past it
    /// by more than `accel_mm_s2` is held to `accel_mm_s2`, as the
    /// absolute budget holds a signal within its limit to.
    PositionEquivalent { accel_limit_mm_s2: f64 },
}

const FOLLOWER_TOL_SCALE_MIN: f64 = 1e-2;

impl FitTol {
    pub(crate) const fn absolute(pos_mm: f64, accel_mm_s2: f64) -> Self {
        Self {
            pos_mm,
            accel_mm_s2,
            accel_budget: AccelBudget::Absolute,
        }
    }

    /// The tolerance one piece of duration `h` is held to, its acceleration
    /// budget resolved for that duration.
    pub(crate) fn for_piece(self, h: f64) -> Self {
        let accel_mm_s2 = match self.accel_budget {
            AccelBudget::Absolute => self.accel_mm_s2,
            AccelBudget::PositionEquivalent { .. } => {
                self.accel_mm_s2.max(8.0 * self.pos_mm / (h * h))
            }
        };
        Self::absolute(self.pos_mm, accel_mm_s2)
    }

    /// The acceleration no piece may reach where its budget is looser than
    /// `accel_mm_s2`: the follower's limit plus `accel_mm_s2`.
    pub(crate) fn accel_ceiling(self) -> f64 {
        match self.accel_budget {
            AccelBudget::Absolute => f64::INFINITY,
            AccelBudget::PositionEquivalent { accel_limit_mm_s2 } => {
                accel_limit_mm_s2 + self.accel_mm_s2
            }
        }
    }

    /// This tolerance under the absolute budget.
    pub(crate) const fn absolute_budget(self) -> Self {
        Self::absolute(self.pos_mm, self.accel_mm_s2)
    }

    #[cfg(test)]
    pub(crate) fn scaled(self, factor: f64) -> Self {
        Self {
            pos_mm: self.pos_mm * factor,
            accel_mm_s2: self.accel_mm_s2 * factor,
            ..self
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

/// The tolerance scale of a follower whose segment demands `followers` yet
/// rides the shaped path at ratios up to `ridden_ratio`: the tighter of
/// the two. A segment whose raw track lays no extrusion — a travel, the
/// rest a stream opens or closes on — is held to the budget of any
/// extrusion its shaped support spills into.
pub(crate) fn riding_tol_scale(
    followers: &[FollowerDemand],
    axis: usize,
    ridden_ratio: f64,
) -> f64 {
    let own = follower_tol_scale(followers, axis);
    if ridden_ratio > 0.0 {
        own.min(ridden_ratio.clamp(FOLLOWER_TOL_SCALE_MIN, 1.0))
    } else {
        own
    }
}

#[cfg(test)]
#[path = "lowering_tests.rs"]
mod tests;
