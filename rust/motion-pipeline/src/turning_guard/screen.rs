use std::collections::VecDeque;

use super::kernel_cells::{KernelCells, Of};
use super::shadow::{AdvanceDemand, FollowerKernels, Leader, norm};
use geometry::ADVANCE_VALLEY_SPEED_MM_S;

/// How many acceleration steps inside one move a summary places in time;
/// the steps past them join the move's variation.
pub(super) const PLACED_STEPS: usize = 4;

/// What the screen reads of one planned move: when it runs, how fast, the
/// headings it takes, how hard it accelerates on each axis, how its
/// acceleration changes, and the extrusion ratio its budget met within its
/// reach.
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
    pub changes: AccelChanges,
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
            changes: AccelChanges::STILL,
            ratio: None,
        }
    }
}

/// How a move's planned acceleration changes, which is what the leaders'
/// kernels turn into shaped jerk: its velocity and acceleration at either
/// end, where the seams into its neighbours step them, the steps inside it
/// at the instants they take, and bounds on what else its acceleration
/// and velocity travel on each axis inside it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct AccelChanges {
    pub velocity_ends: [[f64; 3]; 2],
    pub accel_ends: [[f64; 3]; 2],
    pub steps: [(f64, [f64; 3]); PLACED_STEPS],
    pub step_count: usize,
    /// The total variation of each axis's planned acceleration inside the
    /// move, the placed steps left out.
    pub variation: [f64; 3],
    /// The steps of each axis's planned velocity inside the move.
    pub kinks: [f64; 3],
}

impl AccelChanges {
    pub const STILL: Self = Self {
        velocity_ends: [[0.0; 3]; 2],
        accel_ends: [[0.0; 3]; 2],
        steps: [(0.0, [0.0; 3]); PLACED_STEPS],
        step_count: 0,
        variation: [0.0; 3],
        kinks: [0.0; 3],
    };

    /// Places the step `step` of the acceleration at `at`, or adds it to
    /// the variation once every place is taken.
    pub fn step(&mut self, at: f64, step: [f64; 3]) {
        if self.step_count < PLACED_STEPS {
            self.steps[self.step_count] = (at, step);
            self.step_count += 1;
        } else {
            for (variation, step) in self.variation.iter_mut().zip(step) {
                *variation += step.abs();
            }
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
/// above, along with `|a × v|`. The shaped jerk on axis `k` is
/// `j_k(t) = ∫ w_k(t − τ) da_k(τ) + Σ Δv_k·w_k′(t − τ_Δ)`, the planned
/// acceleration's changes `da_k` and velocity steps `Δv_k` read through the
/// kernel; bounding `|w_k(t − τ)|` (and `|w_k′|`) by its largest over the
/// stretch's instants `t` and the time `τ` each change takes
/// ([`JerkBound`]) bounds `|j_k|` by `J_k`, and `|j·v̂|` by `|J|` and by
/// the kernels' directional variation `G` times the largest planned
/// acceleration. The advance's demand `r·(σ̈ + k·σ⃛) + c·(r·σ̈)²` then stays
/// within `r·(|a| + k·(min(|J|, G·A) + a⊥²/|v|)) + c·(r·|a|)²`, with
/// `a⊥²/|v| = |a × v|²/|v|³ ≤ |a|²/|v|`, wherever the shaped speed is at
/// least [`ADVANCE_VALLEY_SPEED_MM_S`] (slower, the advance lays a valley).
///
/// **Conservative.** A move the screen passes asks no more than `max_accel`
/// anywhere along its own stretch: every term above bounds its counterpart
/// at every instant of that stretch, whatever the plan does within the
/// boxes. For the jerk: `a_k` is of bounded variation, so `da_k` is its
/// steps, placed at their instants, plus a continuous part whose mass over
/// a move is at most the move's [`AccelChanges::variation`]; the seams'
/// steps are the difference of the neighbours' end values; and
/// `|∫ w da_k| ≤ ∫ |w| d|da_k|`, with `|w(t − τ)|` at most its cell bound
/// over the `u = t − τ` the stretch and each change's time span reach.
/// Changes before the window or after it lie past the kernels' reach of
/// every instant of the stretch. It widens the predicate the slow-corner
/// study started from (headings spanning half a turn within the spill, no
/// stop in it, a turn narrower than the kernel's reach), none of whose
/// conditions bounds the turning term when it fails: headings within half
/// a turn keep the shaped speed above their speed share of the slowest
/// planned speed only, and nothing keeps that from zero where the plan
/// slows, which a stop is the extreme of; and a turn wider than the
/// kernel's reach still folds the shaped speed where the axes' kernels
/// differ. The box needs none of them: it passes a move exactly when its
/// own bound fits, and the shadow convolution decides every move it does
/// not.
pub(super) fn passes(
    kernels: &FollowerKernels,
    window: impl Iterator<Item = MoveSummary>,
    stretch: (f64, f64),
    demand: &AdvanceDemand,
    jerk_gain: impl Fn([bool; 3]) -> f64,
    max_accel: f64,
) -> bool {
    let mut velocity = [(f64::INFINITY, f64::NEG_INFINITY); 3];
    let mut accel = [0.0_f64; 3];
    let mut accel_bound = 0.0_f64;
    let mut moves_on = [false; 3];
    let mut jerk = JerkBound::new(kernels, stretch);
    for summary in window {
        jerk.add(&summary);
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
    let speed = gap_sq.sqrt().max(ADVANCE_VALLEY_SPEED_MM_S);
    let accel_ceiling = (strongest.iter().map(|a| a * a).sum::<f64>()).sqrt();
    let cross_sq: f64 = [(0, 1), (0, 2), (1, 2)]
        .iter()
        .map(|&(i, j)| {
            let term = strongest[i] * fastest[j] + strongest[j] * fastest[i];
            term * term
        })
        .sum();
    let turning = (cross_sq / (speed * speed * speed)).min(accel_ceiling * accel_ceiling / speed);
    let tangential_jerk = jerk
        .finish()
        .tangential
        .min(jerk_gain(moves_on) * accel_bound);
    demand.within(accel_ceiling, tangential_jerk, turning) <= max_accel
}

/// The bound on the shaped jerk at every instant of a stretch from the
/// changes of the planned motion the window's summaries hold, in time
/// order. The stretch is cut into blocks of about
/// [`BLOCKS_PER_KERNEL`]-th of the widest kernel's support: on each, `J_k`
/// bounds axis `k`'s jerk by the changes within the kernel's reach of it,
/// each through the largest `|w_k|` the block's instants read it at, and
/// `|J|` the jerk's magnitude; the largest over the blocks bounds the
/// stretch, and changes far apart in time meet on no block.
///
/// A planned velocity step `Δv_k` reads through the kernel's regular slope
/// as `Δv_k·w_k′`; where it meets one of the kernel's own steps it steps
/// the shaped acceleration, and so the advance's velocity, by `Δv_k·J_b`,
/// as raw and unfollowed axes step it, which the planner holds within the
/// extruder's instantaneous corner velocity instead.
pub(super) struct JerkBound<'a> {
    kernels: &'a FollowerKernels,
    stretch: (f64, f64),
    /// The kernels' support, `u ∈ [reach.0, reach.1]` across the axes.
    reach: (f64, f64),
    block: f64,
    count: usize,
    /// The open blocks from `first` on, `J_k` each.
    open: VecDeque<[f64; 3]>,
    first: usize,
    peaks: JerkPeaks,
    before: Option<AccelChanges>,
}

/// How many blocks across the widest kernel's support [`JerkBound`] cuts a
/// stretch into.
const BLOCKS_PER_KERNEL: usize = 8;

/// The largest `J_k` over a stretch's blocks and the largest `|J|`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct JerkPeaks {
    pub per_axis: [f64; 3],
    pub tangential: f64,
}

impl<'a> JerkBound<'a> {
    pub fn new(kernels: &'a FollowerKernels, stretch: (f64, f64)) -> Self {
        let h = kernels.h;
        let (mut reach, mut widest) = ((f64::INFINITY, f64::NEG_INFINITY), 0);
        for leader in &kernels.axes {
            if let Leader::Shaped(cells) = leader {
                let span = cells.span();
                reach = (
                    reach.0.min(span.0 as f64 * h),
                    reach.1.max(span.1 as f64 * h),
                );
                widest = widest.max(span.1 - span.0);
            }
        }
        let block = h * ((widest as usize).div_ceil(BLOCKS_PER_KERNEL)).max(1) as f64;
        Self {
            kernels,
            stretch,
            reach,
            block,
            count: (((stretch.1 - stretch.0) / block).ceil() as usize).max(1),
            open: VecDeque::new(),
            first: 0,
            peaks: JerkPeaks {
                per_axis: [0.0; 3],
                tangential: 0.0,
            },
            before: None,
        }
    }

    /// Adds the changes `summary` holds, and those of the seam into it.
    /// The window's first summary must start out of the kernels' reach of
    /// the stretch, so the seam into it reaches no instant of it.
    ///
    /// Inside a move whose acceleration also varies between its steps, the
    /// changes `da` over its interior `(τ_a, τ_b)` read through the kernel
    /// as `∫ w(t − τ) da(τ)`, bounded both by the steps and the variation
    /// each through their largest `|w|`, and, by parts, by
    /// `|a(τ_b) − a(τ_a)|·|w(t − τ_b)| + ε·TV(w over [t − τ_b, t − τ_a])`
    /// with `ε` the furthest `a` strays from `a(τ_a)`, at most the
    /// variation and twice its bound: each block takes the smaller.
    pub fn add(&mut self, summary: &MoveSummary) {
        let (back, _) = self.kernels.sample_span();
        assert!(
            self.before.is_some() || summary.t0 <= self.stretch.0 - back as f64 * self.kernels.h,
            "turning guard: the screen's window starts at t={} inside the kernels' reach of the \
             stretch from t={}",
            summary.t0,
            self.stretch.0
        );
        self.close_before(summary.t0);
        let changes = summary.changes;
        let h = self.kernels.h;
        if let Some(before) = self.before {
            let at = summary.t0;
            let accel: [f64; 3] =
                std::array::from_fn(|k| (changes.accel_ends[0][k] - before.accel_ends[1][k]).abs());
            let kink: [f64; 3] = std::array::from_fn(|k| {
                (changes.velocity_ends[0][k] - before.velocity_ends[1][k]).abs()
            });
            if accel.iter().chain(&kink).any(|size| *size > 0.0) {
                self.spread((at, at), |k, cells, (t0, t1)| {
                    let reach = (t0 - at, t1 - at);
                    accel[k] * cells.peak_over(Of::Value, reach, h)
                        + kink[k] * cells.peak_over(Of::Slope, reach, h)
                });
            }
        }
        let steps = &changes.steps[..changes.step_count];
        let span = (summary.t0, summary.t1);
        if changes.variation.iter().all(|v| *v == 0.0) {
            for &(at, step) in steps {
                self.spread((at, at), |k, cells, (t0, t1)| {
                    step[k].abs() * cells.peak_over(Of::Value, (t0 - at, t1 - at), h)
                });
            }
        } else {
            let stray: [f64; 3] = std::array::from_fn(|k| {
                let travelled =
                    changes.variation[k] + steps.iter().map(|(_, s)| s[k].abs()).sum::<f64>();
                travelled.min(2.0 * summary.accel[k])
            });
            let duration = span.1 - span.0;
            self.spread(span, |k, cells, (t0, t1)| {
                let over_span = (t0 - span.1, t1 - span.0);
                let by_steps = steps
                    .iter()
                    .map(|&(at, step)| {
                        step[k].abs() * cells.peak_over(Of::Value, (t0 - at, t1 - at), h)
                    })
                    .sum::<f64>()
                    + changes.variation[k] * cells.peak_over(Of::Value, over_span, h);
                let net = (changes.accel_ends[1][k] - changes.accel_ends[0][k]).abs();
                let kernel_travel = (cells.peak_over(Of::Slope, over_span, h) * duration
                    + cells.jumps)
                    .min(cells.variation);
                let by_parts = net * cells.peak_over(Of::Value, (t0 - span.1, t1 - span.1), h)
                    + stray[k] * kernel_travel;
                by_steps.min(by_parts)
            });
        }
        if changes.kinks.iter().any(|kink| *kink > 0.0) {
            self.spread(span, |k, cells, (t0, t1)| {
                changes.kinks[k] * cells.peak_over(Of::Slope, (t0 - span.1, t1 - span.0), h)
            });
        }
        self.before = Some(changes);
    }

    /// Adds `bound(k, kernel, block)` of a change somewhen in `span` to
    /// every block it reaches on every shaped axis.
    fn spread(&mut self, span: (f64, f64), bound: impl Fn(usize, &KernelCells, (f64, f64)) -> f64) {
        let reached = (span.0 + self.reach.0, span.1 + self.reach.1);
        if reached.1 < self.stretch.0 || reached.0 > self.stretch.1 {
            return;
        }
        let block_of = |t: f64| (((t - self.stretch.0) / self.block).floor().max(0.0) as usize);
        let (from, to) = (block_of(reached.0), block_of(reached.1).min(self.count - 1));
        while self.first + self.open.len() <= to {
            self.open.push_back([0.0; 3]);
        }
        assert!(
            from >= self.first,
            "turning guard: a change from t={} reaches the screen's block {from}, closed before \
             block {}",
            span.0,
            self.first
        );
        for block in from..=to {
            let instants = (
                self.stretch.0 + block as f64 * self.block,
                (self.stretch.0 + (block + 1) as f64 * self.block).min(self.stretch.1),
            );
            let sums = &mut self.open[block - self.first];
            for (axis, leader) in self.kernels.axes.iter().enumerate() {
                if let Leader::Shaped(cells) = leader {
                    sums[axis] += bound(axis, cells, instants);
                }
            }
        }
    }

    /// Closes the blocks no change from `t` on reaches.
    fn close_before(&mut self, t: f64) {
        let reached = t + self.reach.0;
        while let Some(sums) = self.open.front().copied() {
            let ends = self.stretch.0 + (self.first + 1) as f64 * self.block;
            if ends >= reached {
                return;
            }
            self.close(sums);
            self.open.pop_front();
            self.first += 1;
        }
    }

    fn close(&mut self, sums: [f64; 3]) {
        for (peak, sum) in self.peaks.per_axis.iter_mut().zip(sums) {
            *peak = peak.max(sum);
        }
        self.peaks.tangential = self.peaks.tangential.max(norm(sums));
    }

    pub fn finish(mut self) -> JerkPeaks {
        while let Some(sums) = self.open.pop_front() {
            self.close(sums);
        }
        self.peaks
    }
}

#[cfg(test)]
mod tests;
