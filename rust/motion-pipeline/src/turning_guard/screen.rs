use super::shadow::{AdvanceDemand, FollowerKernels, norm};
use geometry::ADVANCE_VALLEY_SPEED_MM_S;

/// What the screen reads of one planned move: when it runs, how fast, the
/// headings it takes, how hard it accelerates on each axis, and the
/// extrusion ratio its budget met within its reach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct MoveSummary {
    pub t0: f64,
    pub t1: f64,
    pub speed: (f64, f64),
    /// The least and largest component of the unit heading on each axis;
    /// `None` where the leaders rest.
    pub heading: Option<[(f64, f64); 3]>,
    /// A bound on each axis's planned acceleration.
    pub accel: [f64; 3],
    pub accel_bound: f64,
    pub moves_on: [bool; 3],
    /// The follower axis and ratio in reach a moving, extruder-budgeted
    /// move bounds the laid ratio by wherever its acceleration reaches the
    /// shaped path (see [`geometry::ExtrusionBudget`]).
    pub ratio: Option<(usize, f64)>,
}

impl MoveSummary {
    pub fn rest(t0: f64, t1: f64) -> Self {
        Self {
            t0,
            t1,
            speed: (0.0, 0.0),
            heading: None,
            accel: [0.0; 3],
            accel_bound: 0.0,
            moves_on: [false; 3],
            ratio: None,
        }
    }
}

/// What the planned moves one shaped instant's kernels read bound there:
/// the laid ratio (each move's ratio in reach does, its acceleration
/// acting on the instant), their largest planned acceleration and the
/// axes they move on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Read {
    pub ratio: f64,
    pub accel: f64,
    pub axes: [bool; 3],
}

impl Read {
    pub fn over<'a>(axis: usize, summaries: impl Iterator<Item = &'a MoveSummary>) -> Self {
        summaries.fold(
            Self {
                ratio: f64::INFINITY,
                accel: 0.0,
                axes: [false; 3],
            },
            |read, summary| Self {
                ratio: match summary.ratio {
                    Some((on, ratio)) if on == axis => read.ratio.min(ratio),
                    _ => read.ratio,
                },
                accel: read.accel.max(summary.accel_bound),
                axes: std::array::from_fn(|k| read.axes[k] || summary.moves_on[k]),
            },
        )
    }
}

/// Everything a move's own stretch of the shaped path asks of the
/// extruder's advance at most, from the planned moves within the kernels'
/// reach of it alone, without convolving them: the pre-filter.
///
/// **Bound.** Over the planned time `W` the kernels read while the shaped
/// path crosses the move, each axis's planned velocity `v_k` lies in a box
/// `[lo_k, hi_k]` (speed range times heading-component range, per move,
/// joined), and its planned acceleration within `±α_k`. Splitting a kernel
/// into its positive and negative parts, `∫w·v_k` lies in
/// `[W⁺·lo_k − W⁻·hi_k, W⁺·hi_k − W⁻·lo_k]` and `|∫w·a_k| ≤ (W⁺ + W⁻)·α_k`,
/// so the shaped velocity `v` lies in a box whose distance from the origin
/// bounds `|v|` below, and the shaped acceleration `a` in one bounding it
/// above, along with `|a × v|`; the shaped jerk stays within the kernels'
/// directional variation `G` times the largest planned acceleration. The
/// advance's demand `r·(σ̈ + k·σ⃛) + c·(r·σ̈)²` then stays within
/// `r·(|a| + k·(G·A + a⊥²/|v|)) + c·(r·|a|)²`, with `a⊥²/|v| = |a × v|²/|v|³
/// ≤ |a|²/|v|`, wherever the shaped speed is at least
/// [`ADVANCE_VALLEY_SPEED_MM_S`] (slower, the advance lays a valley).
///
/// **Conservative.** A move the screen passes asks no more than `max_accel`
/// anywhere along its own stretch: every term above bounds its counterpart
/// at every instant of that stretch, whatever the plan does within the
/// boxes. It widens the predicate the slow-corner study started from
/// (headings spanning half a turn within the spill, no stop in it, a turn
/// narrower than the kernel's reach), none of whose conditions bounds the
/// turning term when it fails: headings within half a turn keep the shaped
/// speed above their speed share of the slowest planned speed only, and
/// nothing keeps that from zero where the plan slows, which a stop is the
/// extreme of; and a turn wider than the kernel's reach still folds the
/// shaped speed where the axes' kernels differ. The box needs none of
/// them: it passes a move exactly when its own bound fits, and the shadow
/// convolution decides every move it does not.
pub(super) fn passes(
    kernels: &FollowerKernels,
    window: impl Iterator<Item = MoveSummary>,
    demand: &AdvanceDemand,
    jerk_gain: impl Fn([bool; 3]) -> f64,
    max_accel: f64,
) -> bool {
    let shaped = ShapedBox::over(kernels, window);
    let speed = shaped.speed_floor.max(ADVANCE_VALLEY_SPEED_MM_S);
    let accel_ceiling = norm(shaped.strongest);
    let cross = shaped.cross_ceiling();
    let turning =
        (cross * cross / (speed * speed * speed)).min(accel_ceiling * accel_ceiling / speed);
    demand.within(
        accel_ceiling,
        jerk_gain(shaped.moves_on) * shaped.accel_bound,
        turning,
    ) <= max_accel
}

/// The box the shaped motion stays in while the shaped path crosses a
/// move, from the planned moves within the kernels' reach of it (see
/// [`passes`]): `|v|` at least `speed_floor`, each axis's `|v_k|` within
/// `fastest` and `|a_k|` within `strongest`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ShapedBox {
    pub speed_floor: f64,
    pub fastest: [f64; 3],
    pub strongest: [f64; 3],
    pub accel_bound: f64,
    pub moves_on: [bool; 3],
}

impl ShapedBox {
    pub fn over(kernels: &FollowerKernels, window: impl Iterator<Item = MoveSummary>) -> Self {
        let mut velocity = [(f64::INFINITY, f64::NEG_INFINITY); 3];
        let mut accel = [0.0_f64; 3];
        let mut accel_bound = 0.0_f64;
        let mut moves_on = [false; 3];
        for summary in window {
            accel_bound = accel_bound.max(summary.accel_bound);
            for axis in 0..3 {
                accel[axis] = accel[axis].max(summary.accel[axis]);
                moves_on[axis] |= summary.moves_on[axis];
                let (lo, hi) = match summary.heading {
                    None => (0.0, 0.0),
                    Some(heading) => {
                        let (slow, fast) = summary.speed;
                        let (least, most) = heading[axis];
                        (
                            (slow * least).min(fast * least),
                            (slow * most).max(fast * most),
                        )
                    }
                };
                velocity[axis] = (velocity[axis].0.min(lo), velocity[axis].1.max(hi));
            }
        }
        let mut gap_sq = 0.0;
        let mut fastest = [0.0; 3];
        let mut strongest = [0.0; 3];
        for axis in 0..3 {
            let (positive, negative) = kernels.masses(axis);
            let (lo, hi) = velocity[axis];
            if !(lo <= hi) {
                continue;
            }
            let (least, most) = (positive * lo - negative * hi, positive * hi - negative * lo);
            let gap = least.max(-most).max(0.0);
            gap_sq += gap * gap;
            fastest[axis] = least.abs().max(most.abs());
            strongest[axis] = (positive + negative) * accel[axis];
        }
        Self {
            speed_floor: gap_sq.sqrt(),
            fastest,
            strongest,
            accel_bound,
            moves_on,
        }
    }

    /// `|a × v|` at most: each component `a_i·v_j − a_j·v_i` within
    /// `strongest_i·fastest_j + strongest_j·fastest_i`.
    pub fn cross_ceiling(&self) -> f64 {
        let cross_sq: f64 = [(0, 1), (0, 2), (1, 2)]
            .iter()
            .map(|&(i, j)| {
                let term =
                    self.strongest[i] * self.fastest[j] + self.strongest[j] * self.fastest[i];
                term * term
            })
            .sum();
        cross_sq.sqrt()
    }
}

#[cfg(test)]
mod tests;
