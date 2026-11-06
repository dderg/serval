use std::collections::VecDeque;

use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{
    BrakeMove, ExtrusionBudget, Move, MoveVelocity, ScalarLaw, VelocityError, VelocityPlanParams,
    brake_horizon, brake_move, replan_span, seam_requires_stop,
};
use trajectory::AxisChainSet;

use crate::types::{Control, PlannedItem, PlannedMove, StreamConfig};

mod kernel_cells;
mod screen;
mod shadow;

use screen::{MoveSummary, Read};
use shadow::{
    AdvanceDemand, CellMotion, FollowerKernels, GridSample, PlannedGrid, Ride, shaped_at,
};

/// The guard's coarsest grid step: the planned motion is sampled, and the
/// shaped motion bounded, every this many seconds or finer.
const GRID_S: f64 = 2.5e-4;
/// The grid cells across the narrowest leader kernel at least: the bounds
/// between grid instants shrink with the square of the cell over the
/// kernel's width.
const GRID_CELLS_PER_KERNEL: f64 = 48.0;
/// How many times finer the grid a binding cell is checked again on is.
const GRID_ZOOM: i64 = 8;
/// The gentlest and the sharpest a back-off scales a binding window's
/// acceleration budgets by in one pass.
const BACKOFF_STEP_MAX: f64 = 0.95;
const BACKOFF_STEP_MIN: f64 = 0.5;
/// The demand of the turning term grows as `A^1.5` with the budget `A`
/// where the shaped speed rides the curvature cap `√(A/κ)`: a back-off
/// takes the binding ratio to this power.
const BACKOFF_EXPONENT: f64 = 2.0 / 3.0;
/// A back-off aims this far under the limit, so the pass after it settles.
const BACKOFF_AIM: f64 = 0.98;
/// The least share of its budget a back-off may leave a move: a move asked
/// to drop under it fails loudly ([`Stuck`]), and the guard emits only what
/// stands even with every held move at it ([`TurningGuard::emittable`]).
const SCALE_FLOOR: f64 = 1.0 / 32.0;

/// Holds the advance's turning demand within the extruder's acceleration
/// limit on kernel-free followers, slowing only where it binds.
///
/// The planner bounds an extruding move's demand `r·(σ̈ + k·σ⃛)` with the
/// planned path's own turning; on the shaped path the turning term
/// `a⊥²/|v|` reads the *shaped* speed, which the kernels fold well under
/// the planned one at tight turns closer together than their reach. This
/// stage sits between the planner and the lowerer, holds committed moves
/// until every instant whose kernels read them is proven, and proves each
/// move's own stretch of the shaped path in two steps: a screen bounding
/// the demand from the planned moves in reach alone
/// ([`screen::passes`]), and, where that does not fit, a shadow
/// convolution of the planned motion with the leaders' kernels bounding
/// the shaped motion rigorously cell by grid cell ([`shadow::shaped_at`],
/// [`CellMotion::between`]). Where a cell's bound exceeds the limit it
/// scales the acceleration budgets of the moves that cell reads, re-plans
/// the span between two seams the change leaves standing
/// ([`geometry::replan_span`]), never raising a seam's speed, and checks
/// again: each pass lowers budgets only, and the demand falls with them,
/// so it settles; a move asked to drop under [`SCALE_FLOOR`] of its budget
/// fails loudly.
///
/// Its output reads its input alone: a move is checked once the planned
/// motion its stretch reads has arrived, a re-plan waits for the seams it
/// needs, and a move leaves only once no later back-off can reach it or
/// the motion its kernels read ([`Self::emittable`]), so the stream's
/// batching never changes what it emits.
pub(crate) struct TurningGuard {
    config: StreamConfig,
    followers: Vec<(usize, Follower)>,
    span: (i64, i64),
    entries: Entries,
    history: VecDeque<MoveSummary>,
    grid: PlannedGrid,
    carried_position: Option<[f64; 3]>,
    /// The seam the last emission stood on: no back-off scales or re-plans
    /// a move before it, so the motion emitted and what its proofs read
    /// never change.
    frozen: usize,
    /// The last standing seam found and the reach it was found for: it
    /// stands for any reach at or past that one.
    standing: Option<(usize, usize)>,
}

/// The moves the guard holds, one column per field.
#[derive(Default)]
struct Entries {
    moves: Vec<Move>,
    velocities: Vec<MoveVelocity>,
    budgets: Vec<ExtrusionBudget>,
    stops: Vec<bool>,
    /// Each move as the emission horizon reads it, at [`SCALE_FLOOR`].
    floor_brakes: Vec<BrakeMove>,
    scales: Vec<f64>,
    backoffs: Vec<u32>,
    /// The first grid cell of a move whose check is still to prove, `None`
    /// once every cell is.
    unproven: Vec<Option<i64>>,
    timing: Vec<Timing>,
    summaries: Vec<MoveSummary>,
    /// Whether the segment the moves belong to ends here, at rest.
    closed: bool,
}

/// Where a held move lies along its segment: when it starts and how long
/// it runs, how far along the path, and where the toolhead is at its ends.
#[derive(Debug, Clone, Copy)]
struct Timing {
    t0: f64,
    duration: f64,
    arc0: f64,
    length: f64,
    start: Option<[f64; 3]>,
    end: Option<[f64; 3]>,
    accel: [f64; 3],
}

impl Timing {
    fn t1(&self) -> f64 {
        self.t0 + self.duration
    }

    fn arc1(&self) -> f64 {
        self.arc0 + self.length
    }
}

/// A follower's leader kernels on the guard's grid, and on the grid
/// [`GRID_ZOOM`] times finer a binding cell is checked again on.
struct Follower {
    coarse: FollowerKernels,
    fine: FollowerKernels,
}

/// What a move's cells are checked against: its advance, its extruder's
/// caps, and the planned moves its stretch reads.
struct CellCheck<'a> {
    demand: AdvanceDemand,
    caps: &'a geometry::ExtruderCaps,
    nearby: &'a [MoveSummary],
}

impl CellCheck<'_> {
    /// The binding of the grid cell `n` between the samples `from` and
    /// `to`, `None` where its demand fits.
    fn asks(
        &self,
        kernels: &FollowerKernels,
        from: &shadow::ShapedSample,
        to: &shadow::ShapedSample,
        n: i64,
    ) -> Option<Check> {
        let h = kernels.h;
        let (back, ahead) = kernels.sample_span();
        let motion = CellMotion::between(kernels, from, to);
        let read = ((n - back) as f64 * h, (n + 1 + ahead) as f64 * h);
        let reach = Read::over(
            self.caps.axis_index,
            self.nearby
                .iter()
                .filter(|s| s.t1 > read.0 && s.t0 < read.1),
        );
        let jerk_gain = self.caps.path_jerk_gain(reach.axes) * reach.accel;
        let asked = AdvanceDemand {
            ratio: self.demand.ratio.min(reach.ratio),
            ..self.demand
        }
        .of(
            motion.tangential_accel,
            (
                motion.tangential_jerk.0.max(-jerk_gain),
                motion.tangential_jerk.1.min(jerk_gain),
            ),
            motion.turning_ceiling(),
        );
        (asked > self.caps.max_accel_mm_s2).then_some(Check::Binds(Binding {
            cell: n as f64 * h,
            from: read.0,
            to: read.1,
            demand: asked,
            max_accel: self.caps.max_accel_mm_s2,
        }))
    }
}

fn expect_sample(kernels: &FollowerKernels, grid: &PlannedGrid, n: i64) -> shadow::ShapedSample {
    shaped_at(kernels, grid, n)
        .unwrap_or_else(|| panic!("turning guard: the grid misses t={}", n as f64 * kernels.h))
}

enum Check {
    Passed,
    NotReady,
    Binds(Binding),
}

/// A grid cell starting at `cell` whose shaped motion asks `demand` of the
/// extruder against `max_accel`, reading the planned time `[from, to]`.
#[derive(Debug, Clone, Copy)]
struct Binding {
    cell: f64,
    from: f64,
    to: f64,
    demand: f64,
    max_accel: f64,
}

enum Resettle {
    Done,
    NeedsMore,
}

impl TurningGuard {
    pub fn new(config: StreamConfig, chains: &AxisChainSet) -> Self {
        let followers = guarded_followers(chains);
        let span = followers
            .iter()
            .map(|(_, follower)| follower.coarse.sample_span())
            .fold((1, 0), |(back, ahead), (b, a)| (back.max(b), ahead.max(a)));
        Self {
            config,
            followers,
            span,
            entries: Entries::default(),
            history: VecDeque::new(),
            grid: PlannedGrid::default(),
            carried_position: None,
            frozen: 0,
            standing: None,
        }
    }

    pub fn feed(
        &mut self,
        item: PlannedItem,
        output: &mut impl FnMut(PlannedItem) -> bool,
    ) -> bool {
        if self.followers.is_empty() {
            return output(item);
        }
        match item {
            PlannedItem::Move(planned) => {
                self.admit(planned);
                self.settle();
                self.emit_settled(output)
            }
            PlannedItem::Drain => {
                self.entries.closed = true;
                self.settle();
                assert!(
                    self.entries.unproven.iter().all(Option::is_none),
                    "turning guard: a drained segment left a move unchecked"
                );
                let count = self.entries.moves.len();
                self.emit(count, output) && {
                    self.end_segment();
                    output(PlannedItem::Drain)
                }
            }
            PlannedItem::Control(control) => {
                match &control {
                    Control::Reset { .. } => {
                        self.entries = Entries::default();
                        self.end_segment();
                        self.carried_position = None;
                    }
                    Control::SetAxisChains(chains) => {
                        assert!(
                            self.entries.moves.is_empty(),
                            "turning guard: chains changed under held moves"
                        );
                        *self = Self::new(self.config.clone(), chains);
                    }
                    Control::Dwell { .. } | Control::SetMesh { .. } | Control::Dispatch(_) => {
                        assert!(
                            self.entries.moves.is_empty(),
                            "turning guard: a control token arrived with held moves"
                        );
                    }
                }
                output(PlannedItem::Control(control))
            }
        }
    }

    fn end_segment(&mut self) {
        self.entries = Entries::default();
        self.history.clear();
        self.grid.clear();
        self.frozen = 0;
        self.standing = None;
    }

    fn h(&self) -> f64 {
        self.followers
            .first()
            .map_or(GRID_S, |(_, follower)| follower.coarse.h)
    }

    fn admit(&mut self, planned: PlannedMove) {
        let PlannedMove {
            geometry,
            velocity,
            budget,
        } = planned;
        let held = &self.entries;
        let stop = held
            .moves
            .last()
            .is_some_and(|before| seam_requires_stop(before, &geometry, self.config.corner));
        let floor_brake = brake_move(
            &geometry,
            &budget,
            stop,
            held.moves
                .last()
                .zip(held.budgets.last())
                .zip(held.floor_brakes.last())
                .map(|((m, budget), brake)| (m, budget, brake)),
            SCALE_FLOOR,
            self.plan_params(),
        )
        .unwrap_or_else(|error| panic!("turning guard: brake caps failed: {error:?}"));
        let entries = &mut self.entries;
        let previous = entries.timing.last().copied();
        let timing = timing_of(&geometry, &velocity, previous, self.carried_position);
        if let Some(start) = timing.start {
            for earlier in entries.timing.iter_mut().rev() {
                if earlier.end.is_some() {
                    break;
                }
                earlier.start = Some(start);
                earlier.end = Some(start);
            }
        }
        self.carried_position = timing.end.or(self.carried_position);
        entries
            .summaries
            .push(summary_of(&geometry, &velocity, &budget, &timing));
        entries.moves.push(geometry);
        entries.velocities.push(velocity);
        entries.budgets.push(budget);
        entries.stops.push(stop);
        entries.floor_brakes.push(floor_brake);
        entries.scales.push(1.0);
        entries.backoffs.push(0);
        entries.unproven.push(Some(i64::MIN));
        entries.timing.push(timing);
    }

    /// Checks held moves in order until one waits for motion still to come.
    fn settle(&mut self) {
        loop {
            self.extend_grid();
            let Some(index) = self.entries.unproven.iter().position(Option::is_some) else {
                return;
            };
            match self.check(index) {
                Check::Passed => self.entries.unproven[index] = None,
                Check::NotReady => return,
                Check::Binds(binding) => {
                    if let Resettle::NeedsMore = self.back_off(binding) {
                        return;
                    }
                }
            }
        }
    }

    fn segment_end(&self) -> f64 {
        self.entries.timing.last().map_or(0.0, Timing::t1)
    }

    /// Whether the planned motion up to `t` has all arrived.
    fn arrived(&self, t: f64) -> bool {
        self.entries.closed || t <= self.segment_end()
    }

    fn check(&self, index: usize) -> Check {
        let h = self.h();
        let (back, ahead) = self.span;
        let timing = self.entries.timing[index];
        if !self.arrived(timing.t1() + (ahead + 2) as f64 * h) {
            return Check::NotReady;
        }
        let m = &self.entries.moves[index];
        let Some((follower, demand, caps)) = self.demand_of(index) else {
            return Check::Passed;
        };
        if m.segment.spatial.is_none() {
            return Check::Passed;
        }
        let window = (timing.t0 - back as f64 * h, timing.t1() + ahead as f64 * h);
        let nearby: Vec<MoveSummary> = self.summaries_over(window).collect();
        if screen::passes(
            &follower.coarse,
            nearby.iter().copied(),
            &demand,
            |axes| caps.path_jerk_gain(axes),
            caps.max_accel_mm_s2,
        ) {
            return Check::Passed;
        }
        let cell = CellCheck {
            demand,
            caps: &caps,
            nearby: &nearby,
        };
        let last = (timing.t1() / h).ceil() as i64;
        let first = ((timing.t0 / h).floor() as i64)
            .max(self.entries.unproven[index].expect("turning guard: checking a proven move"))
            .min(last);
        let mut from = expect_sample(&follower.coarse, &self.grid, first);
        for n in first..last {
            let to = expect_sample(&follower.coarse, &self.grid, n + 1);
            if cell.asks(&follower.coarse, &from, &to, n).is_some() {
                if let Some(binds) = self.zoom(follower, &cell, n) {
                    return binds;
                }
            }
            from = to;
        }
        Check::Passed
    }

    /// Checks the coarse cell `n` again on the fine grid, which bounds the
    /// shaped motion [`GRID_ZOOM`] times closer: the cell binds only where a
    /// fine cell inside it does.
    fn zoom(&self, follower: &Follower, cell: &CellCheck, n: i64) -> Option<Check> {
        let fine = &follower.fine;
        let (back, ahead) = fine.sample_span();
        let mut grid = PlannedGrid::default();
        for k in n * GRID_ZOOM - back - 1..=(n + 1) * GRID_ZOOM + ahead + 1 {
            let sample = self.sample(k, fine.h).unwrap_or_else(|| {
                panic!(
                    "turning guard: the planned motion misses t={}",
                    k as f64 * fine.h
                )
            });
            grid.push(k, sample);
        }
        let mut from = expect_sample(fine, &grid, n * GRID_ZOOM);
        for m in n * GRID_ZOOM..(n + 1) * GRID_ZOOM {
            let to = expect_sample(fine, &grid, m + 1);
            if let Some(binds) = cell.asks(fine, &from, &to, m) {
                return Some(binds);
            }
            from = to;
        }
        None
    }

    /// The kernels and advance a move's extrusion reads, `None` where the
    /// planner's own bound already holds the turning term: no extrusion in
    /// reach, no advance, or the follower's own kernel bounding the
    /// advance's jerk.
    fn demand_of(
        &self,
        index: usize,
    ) -> Option<(&Follower, AdvanceDemand, geometry::ExtruderCaps)> {
        let caps = self.entries.moves[index].limits.extruder?;
        let ratio = self.entries.budgets[index].ratio_in_reach;
        if ratio <= 0.0 || caps.advance_slope_s == 0.0 || caps.own_jerk_per_accel_hz.is_finite() {
            return None;
        }
        let follower = self
            .followers
            .iter()
            .find(|(axis, _)| *axis == caps.axis_index)
            .map(|(_, follower)| follower)?;
        Some((
            follower,
            AdvanceDemand {
                ratio,
                slope_s: caps.advance_slope_s,
                curvature_s2_per_mm: caps.advance_curvature_s2_per_mm,
            },
            caps,
        ))
    }

    /// The summaries of the planned motion over `[from, to]`, rest where it
    /// runs past either end of the segment.
    fn summaries_over(&self, (from, to): (f64, f64)) -> impl Iterator<Item = MoveSummary> + '_ {
        let end = self.segment_end();
        let before = (from < 0.0).then(|| MoveSummary::rest(from, 0.0));
        let after = (to > end).then(|| MoveSummary::rest(end, to));
        self.history
            .iter()
            .chain(self.entries.summaries.iter())
            .filter(move |summary| summary.t1 > from && summary.t0 < to)
            .copied()
            .chain(before)
            .chain(after)
    }

    /// Scales the budgets of the moves the planned time a binding cell
    /// reads holds, from the frozen seam on, and re-plans around them; the
    /// moves whose stretch reads what moved are checked again.
    fn back_off(&mut self, binding: Binding) -> Resettle {
        let Binding {
            from,
            to,
            demand,
            max_accel,
            ..
        } = binding;
        let step = (libm::pow(max_accel / demand, BACKOFF_EXPONENT) * BACKOFF_AIM)
            .clamp(BACKOFF_STEP_MIN, BACKOFF_STEP_MAX);
        let timing = &self.entries.timing;
        let first = timing
            .partition_point(|timing| timing.t1() <= from)
            .max(self.frozen);
        let last = timing.partition_point(|timing| timing.t0 < to);
        assert!(
            first < last,
            "turning guard: the binding window [{from}, {to}] holds no move past the frozen seam"
        );
        let saved = (
            self.entries.scales[first..last].to_vec(),
            self.entries.backoffs[first..last].to_vec(),
        );
        for index in first..last {
            self.entries.scales[index] *= step;
            self.entries.backoffs[index] += 1;
            if self.entries.scales[index] < SCALE_FLOOR {
                self.stuck(index, binding).fail();
            }
        }
        match self.resettle(first, last) {
            Resettle::Done => Resettle::Done,
            Resettle::NeedsMore => {
                self.entries.scales[first..last].copy_from_slice(&saved.0);
                self.entries.backoffs[first..last].copy_from_slice(&saved.1);
                Resettle::NeedsMore
            }
        }
    }

    fn stuck(&self, index: usize, binding: Binding) -> Stuck {
        let entries = &self.entries;
        Stuck {
            binding,
            line: entries.moves[index].source.start_line,
            backoffs: entries.backoffs[index],
            frozen: (
                entries.timing[self.frozen].t0,
                entries.moves[self.frozen].source.start_line,
            ),
        }
    }

    /// Re-plans the span around the moves `[first, last)` between two seams
    /// whose speeds it leaves standing, no seam over the speed it has:
    /// back from just before them until the entry seam's speed can still
    /// brake into the scaled budgets, never past the frozen seam, on until
    /// the exit seam's speed is reached again. The moves from the first one
    /// whose plan changed on are re-timed.
    fn resettle(&mut self, first: usize, last: usize) -> Resettle {
        let count = self.entries.moves.len();
        let mut lo = first.saturating_sub(1).max(self.frozen);
        let mut hi = (last + 1).min(count);
        let profile = loop {
            let exit_v = self.entries.velocities[hi - 1].exit_v;
            let ceilings: Vec<f64> = self.entries.velocities[lo..hi]
                .iter()
                .map(|velocity| velocity.entry_v)
                .chain(std::iter::once(exit_v))
                .collect();
            let params = VelocityPlanParams {
                entry_v: self.entries.velocities[lo].entry_v,
                ..self.plan_params()
            };
            match replan_span(
                &self.entries.moves[lo..hi],
                &self.entries.stops[lo..hi],
                &self.entries.budgets[lo..hi],
                &self.entries.scales[lo..hi],
                &ceilings,
                params,
                exit_v,
            ) {
                Err(VelocityError::OverCommitted { .. }) => lo = self.reach_back(lo, first),
                Err(VelocityError::Infeasible {
                    member: 0, line_no, ..
                }) if line_no == self.entries.moves[lo].source.start_line => {
                    lo = self.reach_back(lo, first);
                }
                Err(error) => panic!("turning guard: re-plan failed: {error:?}"),
                Ok(profile) => {
                    let reached = profile.boundary_speeds[hi - lo];
                    if reached + self.config.integration_tol * (1.0 + exit_v) >= exit_v {
                        break profile;
                    }
                    if hi == count {
                        assert!(
                            !self.entries.closed,
                            "turning guard: a drained segment's end at rest is out of reach"
                        );
                        return Resettle::NeedsMore;
                    }
                    hi += 1;
                }
            }
        };
        let changed = profile
            .moves
            .iter()
            .zip(&self.entries.velocities[lo..hi])
            .position(|(planned, held)| planned != held)
            .map(|offset| lo + offset);
        for (offset, velocity) in profile.moves.into_iter().enumerate() {
            self.entries.velocities[lo + offset] = velocity;
        }
        if let Some(changed) = changed {
            self.retime_from(changed);
        }
        Resettle::Done
    }

    /// The seam before `lo` a re-plan of the back-off at `first` reaches
    /// back to, failing loudly at the frozen seam.
    fn reach_back(&self, lo: usize, first: usize) -> usize {
        assert!(
            lo > self.frozen,
            "turning guard: a back-off at line {} re-plans past the frozen seam at t={} (line {})",
            self.entries.moves[first].source.start_line,
            self.entries.timing[self.frozen].t0,
            self.entries.moves[self.frozen].source.start_line,
        );
        lo - 1
    }

    /// The planner bakes each move's corner hold into its limits before it
    /// arrives ([`geometry::MoveVelocity::corner_held`]), so a re-plan here
    /// holds no corner again.
    fn plan_params(&self) -> VelocityPlanParams<'_> {
        VelocityPlanParams {
            integration_tol: self.config.integration_tol,
            max_extrude_only_velocity_mm_s: self.config.max_extrude_only_velocity_mm_s,
            max_extrude_only_accel_mm_s2: self.config.max_extrude_only_accel_mm_s2,
            entry_v: 0.0,
            step_capacity: self.config.step_capacity.as_deref(),
            shaped_corners: None,
        }
    }

    /// Re-times the moves from `index` on after their velocities changed,
    /// forgets the grid past the change and re-opens every cell that reads
    /// it: a cell reads the grid up to `ahead + 2` samples on, and the
    /// sample the change straddles carries the acceleration over it.
    fn retime_from(&mut self, index: usize) {
        let h = self.h();
        let (back, ahead) = self.span;
        let entries = &mut self.entries;
        let changed_at = entries.timing[index].t0;
        let held_from = entries.timing[0].t0;
        assert!(
            held_from == 0.0 || changed_at - (back + ahead + 4) as f64 * h >= held_from,
            "turning guard: a re-plan from t={changed_at} changes motion the moves emitted up to \
             t={held_from} were proven against"
        );
        for at in index..entries.moves.len() {
            let previous = (at > 0).then(|| entries.timing[at - 1]);
            let mut timing = timing_of(&entries.moves[at], &entries.velocities[at], previous, None);
            timing.start = timing.start.or(entries.timing[at].start);
            timing.end = timing.end.or(entries.timing[at].end);
            if at == index {
                timing.t0 = entries.timing[at].t0;
                timing.arc0 = entries.timing[at].arc0;
            }
            entries.timing[at] = timing;
            entries.summaries[at] = summary_of(
                &entries.moves[at],
                &entries.velocities[at],
                &entries.budgets[at],
                &timing,
            );
        }
        let straddled = (changed_at / h).floor() as i64;
        let reopened_from = straddled - ahead - 2;
        for at in 0..entries.moves.len() {
            if (entries.timing[at].t1() / h).ceil() as i64 > reopened_from {
                let unproven = &mut entries.unproven[at];
                *unproven = Some(unproven.map_or(reopened_from, |from| from.min(reopened_from)));
            }
        }
        self.grid.truncate_from(straddled + 1);
        let cell_accel = self.cell_accel(straddled as f64 * h, (straddled + 1) as f64 * h);
        self.grid.set_cell_accel(straddled, cell_accel);
    }

    /// Samples the planned motion on the grid as far as it has arrived,
    /// and forgets what no held move's check reads any more.
    fn extend_grid(&mut self) {
        let h = self.h();
        let (back, ahead) = self.span;
        let Some(first) = self.entries.timing.first().copied() else {
            return;
        };
        let end = self.segment_end();
        let reach_end = if self.entries.closed {
            (end / h).ceil() as i64 + ahead + 4
        } else {
            (end / h).floor() as i64
        };
        let mut k = if self.grid.is_empty() {
            assert!(
                first.t0 == 0.0,
                "turning guard: the planned grid emptied mid-segment, at t={}",
                first.t0
            );
            -back - 2
        } else {
            self.grid.end()
        };
        while k < reach_end {
            let Some(sample) = self.sample(k, h) else {
                break;
            };
            self.grid.push(k, sample);
            k += 1;
        }
        let keep_from = (first.t0 / h).floor() as i64 - back - 4;
        self.grid.trim_before(keep_from);
        let forget = keep_from as f64 * h - h;
        while self
            .history
            .front()
            .is_some_and(|summary| summary.t1 < forget)
        {
            self.history.pop_front();
        }
    }

    /// The planned motion at grid index `k`, `None` while a move it needs
    /// is still to come or its position unknown.
    fn sample(&self, k: i64, h: f64) -> Option<GridSample> {
        let (tau, next) = (k as f64 * h, (k + 1) as f64 * h);
        let timing = &self.entries.timing;
        let first = timing.first()?;
        let last = timing.last()?;
        if next > last.t1() && !self.entries.closed {
            return None;
        }
        let cell_accel = self.cell_accel(tau, next);
        let at_rest = |position: [f64; 3], arc: f64| GridSample {
            cell_accel,
            ..GridSample::at_rest(position, arc)
        };
        if tau < first.t0 {
            assert!(
                first.t0 == 0.0,
                "turning guard: sampling t={tau}, before the held moves from t={}",
                first.t0
            );
            return Some(at_rest(first.start?, first.arc0));
        }
        let index = timing.partition_point(|timing| timing.t1() <= tau);
        let Some(at) = timing.get(index) else {
            return Some(at_rest(last.end?, last.arc1()));
        };
        let (position, heading, s, speed) = match &self.entries.moves[index].segment.spatial {
            None => (at.start?, [0.0; 3], 0.0, 0.0),
            Some(segment) => {
                let (s, speed) = state_at(
                    &self.entries.velocities[index],
                    tau - at.t0,
                    segment.s_len(),
                );
                (segment.point_at(s), segment.heading_at(s), s, speed)
            }
        };
        Some(GridSample {
            position,
            velocity: heading.map(|component| component * speed),
            arc: at.arc0 + s,
            cell_accel,
        })
    }

    /// Each axis's planned acceleration bound over the moves, held or
    /// emitted, that `[from, to]` touches.
    fn cell_accel(&self, from: f64, to: f64) -> [f64; 3] {
        let timing = &self.entries.timing;
        let lo = timing.partition_point(|timing| timing.t1() <= from);
        let held = timing[lo..]
            .iter()
            .take_while(|timing| timing.t0 < to)
            .map(|timing| timing.accel);
        let emitted = self
            .history
            .iter()
            .rev()
            .take_while(|summary| summary.t1 > from)
            .filter(|summary| summary.t0 < to)
            .map(|summary| summary.accel);
        held.chain(emitted).fold([0.0; 3], |acc, accel| {
            std::array::from_fn(|axis| acc[axis].max(accel[axis]))
        })
    }

    /// Emits the held moves no later check can reach, and freezes the seam
    /// that holds them standing.
    fn emit_settled(&mut self, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
        let (count, standing) = self.emittable();
        if count > 0 {
            self.frozen = standing;
        }
        self.emit(count, output)
    }

    /// How many held moves no later back-off can change, nor the motion
    /// their own stretch reads, and the seam that stands for them.
    ///
    /// A back-off scales moves the planned time a pending check reads,
    /// which starts no earlier than `u` (the kernels' backward reach)
    /// before the first move still unchecked, or the end of what has
    /// arrived; the first move overlapping it is the earliest it can touch.
    /// Its re-plan re-opens the cells that read what it changed, whose own
    /// back-offs reach further back, as far as the frozen seam: any move
    /// held may fall to [`SCALE_FLOOR`] of its budget, and no re-plan raises
    /// a seam. The seam a rest at that earliest move leaves standing with
    /// every move at the floor ([`geometry::brake_horizon`]) therefore
    /// stands whatever the pending checks do. A move leaves only once its
    /// stretch's kernels read nothing past it.
    fn emittable(&mut self) -> (usize, usize) {
        let h = self.h();
        let (back, ahead) = self.span;
        let timing = &self.entries.timing;
        let earliest = self
            .entries
            .unproven
            .iter()
            .position(Option::is_some)
            .map_or(self.segment_end(), |index| timing[index].t0)
            - (back + 2) as f64 * h;
        let reach = timing.partition_point(|timing| timing.t1() <= earliest);
        let Some(standing) = self.horizon(reach) else {
            return (0, self.frozen);
        };
        self.standing = Some((standing, reach));
        let timing = &self.entries.timing;
        let standing_t = timing
            .get(standing)
            .map_or(self.segment_end(), |timing| timing.t0);
        let count = timing[..standing]
            .partition_point(|timing| timing.t1() + (back + ahead + 4) as f64 * h <= standing_t);
        (count, standing)
    }

    /// The last seam at or before `reach`, and at or past the frozen one, a
    /// rest at seam `reach` leaves standing with every move at
    /// [`SCALE_FLOOR`] of its budget. A seam found for an earlier reach
    /// still stands: the brake envelope only rises as the rest moves on, and
    /// the seams' speeds only fall.
    fn horizon(&self, reach: usize) -> Option<usize> {
        let since = match self.standing {
            Some((standing, found_for)) if found_for <= reach => standing,
            _ => self.frozen,
        };
        if reach <= since {
            return None;
        }
        let entries = &self.entries;
        brake_horizon(
            &entries.floor_brakes[since..reach],
            entries.velocities[since..reach]
                .iter()
                .map(|velocity| velocity.entry_v),
        )
        .map(|seam| since + seam)
    }

    fn emit(&mut self, count: usize, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
        if count == 0 {
            return true;
        }
        self.frozen = self.frozen.saturating_sub(count);
        self.standing = self.standing.and_then(|(standing, found_for)| {
            Some((standing.checked_sub(count)?, found_for - count))
        });
        let entries = &mut self.entries;
        self.history.extend(entries.summaries.drain(..count));
        entries.stops.drain(..count);
        entries.floor_brakes.drain(..count);
        entries.scales.drain(..count);
        entries.backoffs.drain(..count);
        entries.unproven.drain(..count);
        entries.timing.drain(..count);
        for ((geometry, velocity), budget) in entries
            .moves
            .drain(..count)
            .zip(entries.velocities.drain(..count))
            .zip(entries.budgets.drain(..count))
        {
            if !output(PlannedItem::Move(PlannedMove {
                geometry,
                velocity,
                budget,
            })) {
                return false;
            }
        }
        true
    }
}

/// A binding cell whose back-offs would take line `line`'s budget under
/// [`SCALE_FLOOR`] after `backoffs` of them; `frozen` is the time and line
/// of the frozen seam the window may not reach past.
#[derive(Debug)]
struct Stuck {
    binding: Binding,
    line: u32,
    backoffs: u32,
    frozen: (f64, u32),
}

impl Stuck {
    #[cold]
    #[inline(never)]
    fn fail(&self) -> ! {
        let Binding {
            cell,
            from,
            to,
            demand,
            max_accel,
        } = self.binding;
        panic!(
            "turning guard: the cell at t={cell:.6}s still asks {demand:.0} mm/s² of the \
             extruder (limit {max_accel:.0} mm/s²) after {} back-offs of line {}, the next \
             taking it under {SCALE_FLOOR} of its budget; the cell reads [{from:.6}, {to:.6}]s, \
             the frozen seam is at t={:.6}s (line {})",
            self.backoffs, self.line, self.frozen.0, self.frozen.1
        );
    }
}

/// The kernel-free followers with an advance, each with the kernels of the
/// spatial axes it rides on a grid fine enough for the narrowest of them.
fn guarded_followers(chains: &AxisChainSet) -> Vec<(usize, Follower)> {
    let guarded: Vec<(usize, [Ride<'_>; 3])> = chains
        .projected_followers()
        .filter(|(axis, _)| {
            let chain = &chains.chains[*axis];
            chain.kernel().is_none() && chain.transform().is_some()
        })
        .map(|(axis, leaders)| {
            let rides = |spatial: usize| {
                if !leaders.contains(&spatial) {
                    return Ride::Not;
                }
                chains.chains[spatial]
                    .kernel()
                    .map_or(Ride::Raw, Ride::Through)
            };
            (axis, [rides(0), rides(1), rides(2)])
        })
        .collect();
    let narrowest = guarded
        .iter()
        .flat_map(|(_, axes)| axes.iter().filter_map(|ride| ride.kernel()))
        .map(|kernel| {
            let (lo, hi) = kernel.support();
            hi - lo
        })
        .fold(f64::INFINITY, f64::min);
    let h = GRID_S.min(narrowest / GRID_CELLS_PER_KERNEL);
    guarded
        .into_iter()
        .map(|(axis, axes)| {
            (
                axis,
                Follower {
                    coarse: FollowerKernels::new(axes, h),
                    fine: FollowerKernels::new(axes, h / GRID_ZOOM as f64),
                },
            )
        })
        .collect()
}

/// Where a move lies along its segment after `previous`, and how hard it
/// accelerates on each axis.
fn timing_of(
    m: &Move,
    velocity: &MoveVelocity,
    previous: Option<Timing>,
    carried: Option<[f64; 3]>,
) -> Timing {
    let (t0, arc0, before) = previous.map_or((0.0, 0.0, carried), |previous| {
        (previous.t1(), previous.arc1(), previous.end)
    });
    let duration = velocity.phases.iter().map(|phase| phase.dt).sum();
    let (start, end, accel, length) = match &m.segment.spatial {
        None => (before, before, [0.0; 3], 0.0),
        Some(segment) => {
            let bound = accel_bound(segment, velocity);
            (
                Some(segment.point_at(0.0)),
                Some(segment.point_at(segment.s_len())),
                std::array::from_fn(|axis| bound * segment.lean_on(axis).plane),
                segment.s_len(),
            )
        }
    };
    Timing {
        t0,
        duration,
        arc0,
        length,
        start,
        end,
        accel,
    }
}

/// A bound on the planned acceleration's magnitude along a move: the
/// largest tangential acceleration its phases take, their interpolated
/// rails included, beside the normal acceleration its peak speed takes on
/// its peak curvature, which the disk keeps within the budget.
fn accel_bound(segment: &Segment, velocity: &MoveVelocity) -> f64 {
    let tangential = velocity
        .phases
        .iter()
        .map(|phase| match phase.law {
            ScalarLaw::ConstAccel { a0 } => a0.abs(),
            ScalarLaw::DiskRail { .. } => phase.interpolated_acceleration_bound(0.0, phase.dt),
        })
        .fold(0.0, f64::max);
    let (_, kappa_peak) = segment.kappa_peak();
    let normal = (velocity.peak_v * velocity.peak_v * kappa_peak.abs()).min(velocity.accel);
    velocity
        .accel
        .max((tangential * tangential + normal * normal).sqrt())
}

/// The arc into the move and the speed at move-local time `t`.
fn state_at(velocity: &MoveVelocity, t: f64, length: f64) -> (f64, f64) {
    let origin = velocity.phases.first().map_or(0.0, |phase| phase.s0);
    let mut elapsed = 0.0;
    for phase in &velocity.phases {
        if t < elapsed + phase.dt {
            let (s, v, _) = phase.state_at(phase.t0 + (t - elapsed));
            return ((s - origin).clamp(0.0, length), v);
        }
        elapsed += phase.dt;
    }
    let last = velocity.phases.last().expect("a planned move has phases");
    let (s, v, _) = last.end_state();
    ((s - origin).clamp(0.0, length), v)
}

/// What the screen reads of a move. Each phase runs monotone in speed (a
/// constant acceleration or one branch of a disk rail), so its ends bound
/// its speeds.
fn summary_of(
    m: &Move,
    velocity: &MoveVelocity,
    budget: &ExtrusionBudget,
    timing: &Timing,
) -> MoveSummary {
    let Some(segment) = &m.segment.spatial else {
        return MoveSummary::rest(timing.t0, timing.t1());
    };
    let (slowest, fastest) =
        velocity
            .phases
            .iter()
            .fold((f64::INFINITY, 0.0_f64), |(slowest, fastest), phase| {
                let (_, end, _) = phase.end_state();
                (
                    slowest.min(phase.v0).min(end),
                    fastest.max(phase.v0).max(end),
                )
            });
    MoveSummary {
        t0: timing.t0,
        t1: timing.t1(),
        speed: (slowest.max(0.0), fastest),
        heading: Some(std::array::from_fn(|axis| segment.heading_span_on(axis))),
        accel: timing.accel,
        accel_bound: accel_bound(segment, velocity),
        moves_on: std::array::from_fn(|axis| segment.lean_on(axis).plane != 0.0),
        ratio: m
            .limits
            .extruder
            .map(|caps| (caps.axis_index, budget.ratio_in_reach)),
    }
}

#[cfg(test)]
mod backoff_stream_tests;
#[cfg(test)]
mod test_motion;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod valley_stream_tests;
