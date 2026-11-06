use std::collections::VecDeque;

use geometry::ADVANCE_VALLEY_SPEED_MM_S;
use nurbs::algebra::PiecewisePolynomialKernel;

use super::kernel_cells::KernelCells;

/// The planned motion at one instant `τ_k = k·h` of the guard's grid, and
/// a bound on each axis's planned acceleration over the cell `[τ_k, τ_k+1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GridSample {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// The path length travelled since the segment began.
    pub arc: f64,
    pub cell_accel: [f64; 3],
}

impl GridSample {
    pub fn at_rest(position: [f64; 3], arc: f64) -> Self {
        Self {
            position,
            velocity: [0.0; 3],
            arc,
            cell_accel: [0.0; 3],
        }
    }
}

/// The planned motion sampled on the grid, from grid index `base` on.
#[derive(Debug, Default)]
pub(super) struct PlannedGrid {
    base: i64,
    samples: VecDeque<GridSample>,
}

impl PlannedGrid {
    pub fn get(&self, k: i64) -> Option<&GridSample> {
        usize::try_from(k - self.base)
            .ok()
            .and_then(|index| self.samples.get(index))
    }

    /// One past the last sampled index.
    pub fn end(&self) -> i64 {
        self.base + self.samples.len() as i64
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Resets the planned acceleration bound of the cell from index `k`,
    /// where one is sampled.
    pub fn set_cell_accel(&mut self, k: i64, cell_accel: [f64; 3]) {
        if let Some(sample) = usize::try_from(k - self.base)
            .ok()
            .and_then(|index| self.samples.get_mut(index))
        {
            sample.cell_accel = cell_accel;
        }
    }

    pub fn push(&mut self, k: i64, sample: GridSample) {
        if self.samples.is_empty() {
            self.base = k;
        }
        assert_eq!(k, self.end(), "the planned grid is sampled in order");
        self.samples.push_back(sample);
    }

    /// Forgets the samples from index `k` on, which a re-plan changed.
    pub fn truncate_from(&mut self, k: i64) {
        let keep = usize::try_from(k - self.base).unwrap_or(0);
        self.samples.truncate(keep);
    }

    /// Forgets the samples before index `k`, which no check reads again.
    pub fn trim_before(&mut self, k: i64) {
        while self.base < k && !self.samples.is_empty() {
            self.samples.pop_front();
            self.base += 1;
        }
    }

    pub fn clear(&mut self) {
        self.samples.clear();
    }
}

/// How a follower reads one spatial axis: through a leader kernel, raw, or
/// not at all.
#[derive(Debug, Clone)]
pub(super) enum Leader {
    Shaped(KernelCells),
    Unshaped,
    Unfollowed,
}

/// How a follower rides one spatial axis, before the guard grids it.
#[derive(Debug, Clone, Copy)]
pub(super) enum Ride<'a> {
    Through(&'a PiecewisePolynomialKernel),
    Raw,
    Not,
}

impl<'a> Ride<'a> {
    pub fn kernel(self) -> Option<&'a PiecewisePolynomialKernel> {
        match self {
            Self::Through(kernel) => Some(kernel),
            Self::Raw | Self::Not => None,
        }
    }
}

/// The kernels of the spatial axes a follower rides, on the guard's grid.
#[derive(Debug, Clone)]
pub(super) struct FollowerKernels {
    pub h: f64,
    pub axes: [Leader; 3],
}

impl FollowerKernels {
    pub fn new(axes: [Ride<'_>; 3], h: f64) -> Self {
        Self {
            h,
            axes: axes.map(|axis| match axis {
                Ride::Not => Leader::Unfollowed,
                Ride::Raw => Leader::Unshaped,
                Ride::Through(kernel) => Leader::Shaped(KernelCells::new(kernel, h)),
            }),
        }
    }

    /// How many grid samples before and after `t_n` the shaped motion
    /// there reads: kernel cell `j` reads samples `n − j − 1` and `n − j`.
    pub fn sample_span(&self) -> (i64, i64) {
        self.axes
            .iter()
            .fold((1, 0), |(back, ahead), axis| match axis {
                Leader::Shaped(cells) => (
                    back.max(cells.first + cells.value.len() as i64),
                    ahead.max(-cells.first),
                ),
                Leader::Unshaped | Leader::Unfollowed => (back, ahead),
            })
    }

    /// Upper bounds on the leaders' kernel masses, `(∫w⁺, ∫w⁻)` per axis.
    pub fn masses(&self, axis: usize) -> (f64, f64) {
        match &self.axes[axis] {
            Leader::Shaped(cells) => (cells.positive_mass, cells.negative_mass),
            Leader::Unshaped => (1.0, 0.0),
            Leader::Unfollowed => (0.0, 0.0),
        }
    }
}

/// The shaped motion at a grid instant: velocity, acceleration and jerk
/// per axis, each within its radius of the truth, and the largest planned
/// acceleration bound per axis over the cells it read.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(super) struct ShapedSample {
    pub velocity: [f64; 3],
    pub accel: [f64; 3],
    pub jerk: [f64; 3],
    pub velocity_miss: [f64; 3],
    pub accel_miss: [f64; 3],
    pub jerk_miss: [f64; 3],
    pub window_accel: [f64; 3],
}

/// The shaped motion at `t_n = n·h`, `None` where the grid lacks a cell it
/// reads.
///
/// Each kernel cell `j` reads the planned cell `m = n − j − 1`, where
/// `w(t_n − τ) = α − β·(τ − τ_mid) + e` with `|e| ≤ ρ`:
/// - `∫ w·v = α·ΔP − β·∫(τ − τ_mid)·v + ∫e·v`; the middle term is at most
///   `A·h³/12` (`v` is `A`-Lipschitz), the last `ρ·Δs`;
/// - `∫ w·a = α·Δv − β·T + ∫e·a` with `T = ∫(τ − τ_mid)·a
///   = h/2·(v_m + v_m+1) − ΔP` exactly, the last at most `ρ·A·h`;
/// - the jerk, `∫ w′·a` over the regular slope plus `Σ J_b·a(t − u_b)` over
///   the kernel's steps, the same way with `w′`'s cells and `|J_b|·A`.
///
/// A raw axis reads its planned velocity and nothing of its acceleration
/// but the bound: its planned acceleration steps reach the advance as
/// velocity steps, which the planner holds within the extruder's
/// instantaneous corner velocity instead.
pub(super) fn shaped_at(
    kernels: &FollowerKernels,
    grid: &PlannedGrid,
    n: i64,
) -> Option<ShapedSample> {
    let h = kernels.h;
    let mut out = ShapedSample::default();
    for (axis, leader) in kernels.axes.iter().enumerate() {
        match leader {
            Leader::Unfollowed => {}
            Leader::Unshaped => {
                let here = grid.get(n)?;
                let before = grid.get(n - 1)?;
                let accel = here.cell_accel[axis].max(before.cell_accel[axis]);
                out.velocity[axis] = here.velocity[axis];
                out.accel_miss[axis] = accel;
                out.window_accel[axis] = accel;
            }
            Leader::Shaped(cells) => {
                let mut sums = [0.0; 6];
                let mut window = 0.0_f64;
                for (offset, (value, slope)) in cells.value.iter().zip(&cells.slope).enumerate() {
                    let m = n - (cells.first + offset as i64) - 1;
                    let lo = grid.get(m)?;
                    let hi = grid.get(m + 1)?;
                    let accel = lo.cell_accel[axis];
                    window = window.max(accel);
                    let moved = hi.position[axis] - lo.position[axis];
                    let turned = hi.velocity[axis] - lo.velocity[axis];
                    let defect = 0.5 * h * (hi.velocity[axis] + lo.velocity[axis]) - moved;
                    let travelled = hi.arc - lo.arc;
                    sums[0] += value.at * moved;
                    sums[1] +=
                        value.slope.abs() * accel * h * h * h / 12.0 + value.miss * travelled;
                    sums[2] += value.at * turned - value.slope * defect;
                    sums[3] += value.miss * accel * h;
                    sums[4] += slope.at * turned - slope.slope * defect;
                    sums[5] += slope.miss * accel * h;
                }
                out.velocity[axis] = sums[0];
                out.velocity_miss[axis] = sums[1];
                out.accel[axis] = sums[2];
                out.accel_miss[axis] = sums[3];
                out.jerk[axis] = sums[4];
                out.jerk_miss[axis] = sums[5] + cells.jumps * window;
                out.window_accel[axis] = window;
            }
        }
    }
    Some(out)
}

/// Bounds on the shaped motion across the grid cell between two samples
/// `h` apart: the least and the largest speed and acceleration, `|a × v|`
/// and jerk, and the range of the acceleration and jerk along the velocity
/// wherever the speed is at least [`ADVANCE_VALLEY_SPEED_MM_S`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CellMotion {
    pub speed_floor: f64,
    pub speed_ceiling: f64,
    pub accel_floor: f64,
    pub accel_ceiling: f64,
    pub cross_ceiling: f64,
    pub jerk_ceiling: f64,
    pub tangential_accel: (f64, f64),
    pub tangential_jerk: (f64, f64),
}

impl CellMotion {
    /// Between the samples each quantity strays from the chord through
    /// them by at most `f″·h²/8` where `f′` is Lipschitz, by `L·h/2` where
    /// `f` itself is `L`-Lipschitz, and by `K·h/4` more where `f′` steps by
    /// `K`: the velocity's slope is the acceleration, bounded by the jerk;
    /// the acceleration's slope is the jerk, Lipschitz at the kernel's rate
    /// times the planned acceleration, stepping by `2·|J_b|·A` where a
    /// kernel step meets a planned acceleration step. A product of two
    /// chords is a quadratic, within a quarter of its leading coefficient
    /// of the chord through its ends.
    pub fn between(kernels: &FollowerKernels, from: &ShapedSample, to: &ShapedSample) -> Self {
        let h = kernels.h;
        let mut velocity_radius = [0.0; 3];
        let mut accel_radius = [0.0; 3];
        let mut jerk_radius = [0.0; 3];
        let mut jerk = [0.0; 3];
        for (axis, leader) in kernels.axes.iter().enumerate() {
            let window = from.window_accel[axis].max(to.window_accel[axis]);
            match leader {
                Leader::Unfollowed => {}
                Leader::Unshaped => {
                    velocity_radius[axis] = 0.5 * window * h;
                    accel_radius[axis] = window;
                }
                Leader::Shaped(cells) => {
                    let rate = cells.jerk_rate_per_accel * window;
                    let step = 2.0 * cells.jumps * window;
                    jerk_radius[axis] =
                        from.jerk_miss[axis].max(to.jerk_miss[axis]) + 0.5 * rate * h + step;
                    jerk[axis] = from.jerk[axis].abs().max(to.jerk[axis].abs()) + jerk_radius[axis];
                    velocity_radius[axis] = from.velocity_miss[axis].max(to.velocity_miss[axis])
                        + jerk[axis] * h * h / 8.0;
                    accel_radius[axis] = from.accel_miss[axis].max(to.accel_miss[axis])
                        + rate * h * h / 8.0
                        + step * h / 4.0;
                }
            }
        }
        let velocity_radius = norm(velocity_radius);
        let accel_radius = norm(accel_radius);
        let jerk_radius = norm(jerk_radius);
        let fastest = norm(from.velocity).max(norm(to.velocity));
        let strongest = norm(from.accel).max(norm(to.accel));
        let accel_step = sub(to.accel, from.accel);
        let velocity_step = sub(to.velocity, from.velocity);
        let chord_cross = norm(cross(from.accel, from.velocity))
            .max(norm(cross(to.accel, to.velocity)))
            + 0.25 * norm(cross(accel_step, velocity_step));
        let speed_floor = distance_to_segment(from.velocity, to.velocity) - velocity_radius;
        let accel_ceiling = strongest + accel_radius;
        let jerk_ceiling = norm(jerk);
        let (slowest, fastest_bound) = (
            speed_floor.max(ADVANCE_VALLEY_SPEED_MM_S),
            fastest + velocity_radius,
        );
        let along = |(p0, p1, radius): ([f64; 3], [f64; 3], f64), ceiling: f64| {
            let ends = (dot(p0, from.velocity), dot(p1, to.velocity));
            let spread = 0.25 * dot(sub(p1, p0), velocity_step).abs()
                + norm(p0).max(norm(p1)) * velocity_radius
                + fastest * radius
                + radius * velocity_radius;
            let (least, most) = (ends.0.min(ends.1) - spread, ends.0.max(ends.1) + spread);
            let over = |product: f64| {
                let speeds = (product / slowest, product / fastest_bound);
                (speeds.0.min(speeds.1), speeds.0.max(speeds.1))
            };
            (over(least).0.max(-ceiling), over(most).1.min(ceiling))
        };
        Self {
            speed_floor,
            speed_ceiling: fastest_bound,
            accel_floor: (distance_to_segment(from.accel, to.accel) - accel_radius).max(0.0),
            accel_ceiling,
            cross_ceiling: chord_cross
                + strongest * velocity_radius
                + fastest * accel_radius
                + accel_radius * velocity_radius,
            jerk_ceiling,
            tangential_accel: along((from.accel, to.accel, accel_radius), accel_ceiling),
            tangential_jerk: along((from.jerk, to.jerk, jerk_radius), jerk_ceiling),
        }
    }

    /// The largest `a⊥²/|v| = |a × v|²/|v|³` across the cell wherever the
    /// shaped speed is at least [`ADVANCE_VALLEY_SPEED_MM_S`]; slower, the
    /// advance lays a valley instead of reading the path speed through.
    pub fn turning_ceiling(&self) -> f64 {
        let speed = self.speed_floor.max(ADVANCE_VALLEY_SPEED_MM_S);
        (self.cross_ceiling * self.cross_ceiling / (speed * speed * speed))
            .min(self.accel_ceiling * self.accel_ceiling / speed)
    }
}

/// What the advance asks of the extruder's motor at most: `r·(σ̈ + k·σ⃛)`
/// plus the advance's curvature `c·(r·σ̈)²`, with `σ̈ = a·v̂` and
/// `σ⃛ = j·v̂ + a⊥²/|v|`, the turning term never negative; inside advance
/// valleys of weight `Φ`, `r·(σ̈ + k·(Φ·σ⃛ + 3Φ′·σ̇·σ̈ + Φ″·σ̇³))`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct AdvanceDemand {
    pub ratio: f64,
    pub slope_s: f64,
    pub curvature_s2_per_mm: f64,
}

/// What the advance valleys around a cell weigh the advance by: their
/// weights' product `Φ` within `[weight_floor, 1]`, `|Φ′·σ̇|` within
/// `rebuild` and `Φ″·σ̇³` within `[−fold, bend]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ValleyRead {
    pub weight_floor: f64,
    pub rebuild: f64,
    pub fold: f64,
    pub bend: f64,
}

impl ValleyRead {
    pub const NONE: Self = Self {
        weight_floor: 1.0,
        rebuild: 0.0,
        fold: 0.0,
        bend: 0.0,
    };
}

impl AdvanceDemand {
    /// The demand with `σ̈` within `accel`, `j·v̂` within `jerk` and the
    /// turning term within `[0, turning]`.
    pub fn of(&self, accel: (f64, f64), jerk: (f64, f64), turning: f64) -> f64 {
        self.weighted(accel, jerk, turning, &ValleyRead::NONE)
    }

    /// [`Self::of`] read through the advance valleys `valley`.
    pub fn weighted(
        &self,
        accel: (f64, f64),
        jerk: (f64, f64),
        turning: f64,
        valley: &ValleyRead,
    ) -> f64 {
        let jerk_most = jerk.1 + turning;
        let weighted = (
            jerk.0.min(valley.weight_floor * jerk.0),
            jerk_most.max(valley.weight_floor * jerk_most),
        );
        let steepest = accel.0.abs().max(accel.1.abs());
        let rebuild = 3.0 * valley.rebuild * steepest;
        let least = accel.0 + self.slope_s * (weighted.0 - rebuild - valley.fold);
        let most = accel.1 + self.slope_s * (weighted.1 + rebuild + valley.bend);
        let base = self.ratio * steepest;
        self.ratio * least.abs().max(most.abs()) + self.curvature_s2_per_mm * base * base
    }

    /// The demand with `|σ̈| ≤ accel`, `|j·v̂| ≤ jerk` and the turning term
    /// within `[0, turning]`.
    pub fn within(&self, accel: f64, jerk: f64, turning: f64) -> f64 {
        self.of((-accel, accel), (-jerk, jerk), turning)
    }
}

pub(super) fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The distance from the origin to the segment from `a` to `b`.
fn distance_to_segment(a: [f64; 3], b: [f64; 3]) -> f64 {
    let step = sub(b, a);
    let length_sq = step[0] * step[0] + step[1] * step[1] + step[2] * step[2];
    if length_sq == 0.0 {
        return norm(a);
    }
    let along = -(a[0] * step[0] + a[1] * step[1] + a[2] * step[2]) / length_sq;
    let t = along.clamp(0.0, 1.0);
    norm([a[0] + t * step[0], a[1] + t * step[1], a[2] + t * step[2]])
}

#[cfg(test)]
mod tests;
