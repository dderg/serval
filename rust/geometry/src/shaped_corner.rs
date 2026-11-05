//! How far the input shaper's X and Y kernels pull the toolhead off a
//! corner's vertex, so the corner budget can bound the shaped path rather
//! than the planned one.
//!
//! A corner is planned as a symmetric clothoid blend of cut `c` (the apex's
//! distance from the vertex), crossed at apex speed `v` under the
//! acceleration disk `|a| ≤ A`. Lengths scale with `L = A·T²` and speeds
//! with `A·T`, `T` the wider kernel's standard deviation, so the miss
//! `M = L·m(θ, c/L, v/v_cap)` reads off one table per kernel pair: the
//! convolution of that canonical motion, worst over the corner's
//! orientation against the axes. `v_cap = √(A/κ_peak)` is the blend's
//! curvature ceiling. A stop at a sharp corner is the table's `c = 0` row:
//! the shaper's floor, which no apex speed undercuts.

use std::sync::{Mutex, OnceLock};

use nurbs::algebra::PiecewisePolynomialKernel;

use crate::frontend::{CornerBudget, Move};
use crate::path::lowering::PositionProfile;
use crate::path::{Clothoid, CurvatureProfile, Segment};
use crate::vec3::{dot, norm, sub};

const THETA_STEP_DEG: f64 = 5.0;
const THETA_NODES: usize = 37;
/// Where a blend row sits at a reversal: a blend never turns this far (the
/// fitter stops at near-reversals), and the canonical pair degenerates at
/// 180°.
const BLEND_THETA_MAX_DEG: f64 = 179.0;
const CUT_OCTAVE_LOW: i32 = -8;
const CUT_NODES: usize = 13;
const RATIO_NODES: usize = 9;
const ORIENTATIONS: usize = 12;
const STEPS_PER_SIGMA: f64 = 16.0;
const SEARCH_SIGMAS: f64 = 2.0;
const RK_SUBSTEPS: usize = 4;
const BLEND_SAMPLES: usize = 256;
const SPEED_OCTAVE_LOW: i32 = -10;
const SPEED_NODES: usize = 19;
const CIRCLE_NODES: usize = 64;
const CIRCLE_OMEGA_MAX: f64 = 3.0;
const CIRCLE_PHASES: usize = 64;
const CIRCLE_RATE_FLOOR: f64 = 1e-3;
const SQUARE_CORNER_RAD: f64 = std::f64::consts::FRAC_PI_2;
const ACCEL_BISECTIONS: u32 = 48;
const RATIO_BISECTIONS: u32 = 40;
const CUT_CANDIDATES: i32 = 16;
const ROOM_ACCEL_OCTAVES: i32 = 10;
const TURN_EPS_RAD: f64 = 1e-6;
/// How far before and after a slowed corner its slower acceleration reaches,
/// in `σ²·A`: the distance a stop at `A` covers in four of the wider
/// kernel's standard deviations, past the reach of its weight.
const APPROACH_SIGMAS_SQUARED: f64 = 16.0;
/// What the prediction adds to the shaper's share of a miss for the
/// approximations between the table and the planned motion: a corner whose
/// approach or exit is slower than the disk's, an asymmetric blend, the
/// orientations between the sampled ones.
pub const PREDICTION_MARGIN: f64 = 1.05;
/// A corner a stop at its planned acceleration rounds past its budget is
/// slowed to the acceleration at which a stop takes this share of the
/// budget's room beyond the cut, leaving the rest to the apex speed.
pub const STOP_SHARE: f64 = 0.5;
/// The share of a corner budget an arc or a merged run of facets may spend
/// on its fit, the rest left to the shaper's pull on the curve.
pub const ARC_FIT_SHARE: f64 = 0.5;

/// A kernel sampled every `1/STEPS_PER_SIGMA` of the pair's `T`, as
/// weights of unit sum from offset `first` on.
#[derive(Debug, Clone)]
struct Taps {
    first: i64,
    weights: Vec<f64>,
}

impl Taps {
    fn identity() -> Self {
        Self {
            first: 0,
            weights: vec![1.0],
        }
    }

    fn of(kernel: &PiecewisePolynomialKernel, sigma_s: f64) -> Self {
        let step_s = sigma_s / STEPS_PER_SIGMA;
        let (lo, hi) = kernel.support();
        let first = (lo / step_s).floor() as i64;
        let last = (hi / step_s).ceil() as i64;
        let mut weights: Vec<f64> = (first..=last)
            .map(|j| kernel_value(kernel, j as f64 * step_s))
            .collect();
        let mass: f64 = weights.iter().sum();
        assert!(
            mass.is_finite() && mass > 0.0,
            "a shaper kernel sampled every {step_s} s has mass {mass}"
        );
        for weight in &mut weights {
            *weight /= mass;
        }
        Self { first, weights }
    }

    fn last(&self) -> i64 {
        self.first + self.weights.len() as i64 - 1
    }

    fn reach(&self) -> i64 {
        self.first.abs().max(self.last().abs())
    }

    /// `Σ_j w_j·p(m − j)` for the samples `p` indexed from `-offset`.
    fn apply(&self, samples: &[f64], offset: i64, m: i64) -> f64 {
        self.weights
            .iter()
            .enumerate()
            .map(|(index, weight)| {
                let j = self.first + index as i64;
                weight * samples[(m - j + offset) as usize]
            })
            .sum()
    }

    fn gain(&self, omega: f64) -> (f64, f64) {
        self.weights
            .iter()
            .enumerate()
            .fold((0.0, 0.0), |(re, im), (index, weight)| {
                let phase = -omega * (self.first + index as i64) as f64 / STEPS_PER_SIGMA;
                (re + weight * phase.cos(), im + weight * phase.sin())
            })
    }
}

fn kernel_value(kernel: &PiecewisePolynomialKernel, u: f64) -> f64 {
    kernel
        .pieces
        .iter()
        .find(|piece| u >= piece.u_start && u < piece.u_end)
        .map_or(0.0, |piece| piece.evaluate(u))
}

/// The canonical blend of one turn: its halves sampled from the vertex,
/// cut 1 at the apex.
struct Blend {
    theta: f64,
    half1: Vec<[f64; 2]>,
    half2: Vec<[f64; 2]>,
    /// The half length per unit cut.
    length: f64,
}

impl Blend {
    fn of(theta: f64) -> Self {
        let x = [1.0, 0.0, 0.0];
        let y = [0.0, 1.0, 0.0];
        let half1 = Clothoid::try_new([0.0; 3], x, y, 0.0, theta, 1.0)
            .expect("the canonical half turns a positive angle over unit length");
        let apex = half1.point_at(1.0);
        let u_apex = half1.heading_at(1.0);
        let v_apex = [-u_apex[1], u_apex[0], 0.0];
        let half2 = Clothoid::try_new(apex, u_apex, v_apex, theta, -theta, 1.0)
            .expect("the canonical second half mirrors the first");
        let end = half2.point_at(1.0);
        let end_heading = half2.heading_at(1.0);
        let vertex_x = end[0] - end[1] / end_heading[1] * end_heading[0];
        let cut = ((apex[0] - vertex_x).powi(2) + apex[1].powi(2)).sqrt();
        let sample = |half: &Clothoid| -> Vec<[f64; 2]> {
            (0..=BLEND_SAMPLES)
                .map(|i| {
                    let p = half.point_at(i as f64 / BLEND_SAMPLES as f64);
                    [(p[0] - vertex_x) / cut, p[1] / cut]
                })
                .collect()
        };
        Self {
            theta,
            half1: sample(&half1),
            half2: sample(&half2),
            length: 1.0 / cut,
        }
    }

    /// `κ_peak·c`: the blend's peak curvature times its cut.
    fn peak_curvature_cut(&self) -> f64 {
        self.theta / self.length
    }

    fn sample(half: &[[f64; 2]], u: f64) -> [f64; 2] {
        let at = u.clamp(0.0, 1.0) * BLEND_SAMPLES as f64;
        let i = (at.floor() as usize).min(BLEND_SAMPLES - 1);
        let f = at - i as f64;
        [
            half[i][0] + f * (half[i + 1][0] - half[i][0]),
            half[i][1] + f * (half[i + 1][1] - half[i][1]),
        ]
    }

    /// Position at arc `s` from the apex of the blend of cut `cut`.
    fn point(&self, cut: f64, s: f64) -> [f64; 2] {
        let half = self.length * cut;
        let exit = [self.theta.cos(), self.theta.sin()];
        if s < -half {
            let start = Self::sample(&self.half1, 0.0);
            [cut * start[0] + (s + half), cut * start[1]]
        } else if s <= 0.0 {
            let p = Self::sample(&self.half1, 1.0 + s / half);
            [cut * p[0], cut * p[1]]
        } else if s <= half {
            let p = Self::sample(&self.half2, s / half);
            [cut * p[0], cut * p[1]]
        } else {
            let end = Self::sample(&self.half2, 1.0);
            [
                cut * end[0] + (s - half) * exit[0],
                cut * end[1] + (s - half) * exit[1],
            ]
        }
    }

    fn curvature(&self, cut: f64, s: f64) -> f64 {
        let half = self.length * cut;
        if s.abs() >= half {
            0.0
        } else {
            self.theta / half * (1.0 - s.abs() / half)
        }
    }
}

/// The arc from the apex at each grid instant `k/STEPS_PER_SIGMA`, `k` from
/// 0 on, of a motion leaving the apex at `v0` under the unit disk.
fn arc_from_apex(curvature: impl Fn(f64) -> f64, v0: f64, count: usize) -> Vec<f64> {
    let h = 1.0 / (STEPS_PER_SIGMA * RK_SUBSTEPS as f64);
    let accel = |s: f64, v: f64| (1.0 - (v * v * curvature(s)).powi(2)).max(0.0).sqrt();
    let mut arcs = Vec::with_capacity(count);
    let (mut s, mut v) = (0.0_f64, v0);
    arcs.push(s);
    for _ in 1..count {
        for _ in 0..RK_SUBSTEPS {
            let (s1, v1) = (v, accel(s, v));
            let (s2, v2) = (v + 0.5 * h * v1, accel(s + 0.5 * h * s1, v + 0.5 * h * v1));
            let (s3, v3) = (v + 0.5 * h * v2, accel(s + 0.5 * h * s2, v + 0.5 * h * v2));
            let (s4, v4) = (v + h * v3, accel(s + h * s3, v + h * v3));
            s += h / 6.0 * (s1 + 2.0 * s2 + 2.0 * s3 + s4);
            v += h / 6.0 * (v1 + 2.0 * v2 + 2.0 * v3 + v4);
        }
        arcs.push(s);
    }
    arcs
}

/// The worst closest approach to the vertex over the corner's orientations
/// of the motion `points`, sampled from index `-offset`, shaped by `taps`.
fn worst_miss(taps: &[Taps; 2], xs: &[f64], ys: &[f64], offset: i64, search: i64) -> f64 {
    let window = (2 * search + 1) as usize;
    let mut sums = [[0.0; 4]].repeat(window);
    for (index, m) in (-search..=search).enumerate() {
        sums[index] = [
            taps[0].apply(xs, offset, m),
            taps[0].apply(ys, offset, m),
            taps[1].apply(xs, offset, m),
            taps[1].apply(ys, offset, m),
        ];
    }
    (0..ORIENTATIONS)
        .map(|l| {
            let phi = std::f64::consts::PI * l as f64 / ORIENTATIONS as f64;
            let (sin, cos) = phi.sin_cos();
            let at = |s: &[f64; 4]| [cos * s[0] - sin * s[1], sin * s[2] + cos * s[3]];
            let (closest, index) = sums
                .windows(2)
                .enumerate()
                .map(|(index, pair)| (distance_to_chord(at(&pair[0]), at(&pair[1])), index))
                .fold((f64::INFINITY, 0), |best, next| {
                    if next.0 < best.0 { next } else { best }
                });
            assert!(
                index > 0 && index + 2 < window,
                "the shaped corner's closest approach lies at the edge of its search window"
            );
            closest
        })
        .fold(0.0, f64::max)
}

fn distance_to_chord(a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let length_squared = d[0] * d[0] + d[1] * d[1];
    let along = if length_squared > 0.0 {
        (-(a[0] * d[0] + a[1] * d[1]) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a[0] + along * d[0]).hypot(a[1] + along * d[1])
}

fn theta_node(index: usize) -> f64 {
    (index as f64 * THETA_STEP_DEG).to_radians()
}

fn cut_node(index: usize) -> f64 {
    2.0_f64.powi(CUT_OCTAVE_LOW + index as i32)
}

fn ratio_node(index: usize) -> f64 {
    index as f64 / (RATIO_NODES - 1) as f64
}

fn speed_node(index: usize) -> f64 {
    2.0_f64.powi(SPEED_OCTAVE_LOW + index as i32)
}

fn circle_node(index: usize) -> f64 {
    (CIRCLE_OMEGA_MAX * index as f64 / (CIRCLE_NODES - 1) as f64).max(CIRCLE_RATE_FLOOR)
}

/// Where `x` falls between nodes `0..count` spaced by `node(i)`: the lower
/// node and the fraction towards the next.
fn bracket(position: f64, count: usize) -> (usize, f64) {
    let clamped = position.clamp(0.0, (count - 1) as f64);
    let index = (clamped.floor() as usize).min(count - 2);
    (index, clamped - index as f64)
}

/// The shaper's rounding of corners and arcs for one pair of X and Y
/// kernels.
#[derive(Debug, Clone, PartialEq)]
pub struct ShapedCorners {
    sigma_s: f64,
    peak_curvature_cut: [f64; THETA_NODES],
    floor: [f64; THETA_NODES],
    miss: Vec<f64>,
    square_at_speed: [f64; SPEED_NODES],
    /// `(1 − ρ)/ω̂²`, `ρ` the least radius of a unit circle crossed at `ω̂`
    /// radians per `T`, node 0 at a vanishing rate.
    circle: [f64; CIRCLE_NODES],
}

impl ShapedCorners {
    /// `None` where neither X nor Y is shaped: the planned path is the
    /// toolhead's.
    #[must_use]
    pub fn new(kernels: [Option<&PiecewisePolynomialKernel>; 2]) -> Option<Self> {
        let sigma_s = kernels
            .iter()
            .flatten()
            .map(|kernel| kernel.second_moment())
            .reduce(f64::max)?
            .sqrt();
        assert!(
            sigma_s.is_finite() && sigma_s > 0.0,
            "a shaper kernel has a non-positive second moment"
        );
        let taps =
            kernels.map(|kernel| kernel.map_or_else(Taps::identity, |k| Taps::of(k, sigma_s)));
        let search = (SEARCH_SIGMAS * STEPS_PER_SIGMA) as i64;
        let offset = search + taps[0].reach().max(taps[1].reach());
        let count = (offset + 1) as usize;
        let miss_of = |points: &dyn Fn(f64) -> [f64; 2], arcs: &[f64]| {
            let (xs, ys): (Vec<f64>, Vec<f64>) = (-offset..=offset)
                .map(|k| {
                    let s = arcs[k.unsigned_abs() as usize];
                    points(if k < 0 { -s } else { s })
                })
                .map(|p| (p[0], p[1]))
                .unzip();
            worst_miss(&taps, &xs, &ys, offset, search)
        };
        let sharp = |theta: f64, v0: f64| {
            let exit = [theta.cos(), theta.sin()];
            let arcs = arc_from_apex(|_| 0.0, v0, count);
            miss_of(
                &|s: f64| {
                    if s < 0.0 {
                        [s, 0.0]
                    } else {
                        [s * exit[0], s * exit[1]]
                    }
                },
                &arcs,
            )
        };
        let mut floor = [0.0; THETA_NODES];
        let mut peak_curvature_cut = [0.0; THETA_NODES];
        let mut miss = vec![0.0; THETA_NODES * CUT_NODES * RATIO_NODES];
        for t in 1..THETA_NODES {
            floor[t] = sharp(theta_node(t), 0.0);
            let blend = Blend::of(theta_node(t).min(BLEND_THETA_MAX_DEG.to_radians()));
            peak_curvature_cut[t] = blend.peak_curvature_cut();
            for c in 0..CUT_NODES {
                let cut = cut_node(c);
                let cap = (cut / blend.peak_curvature_cut()).sqrt();
                let mut envelope = 0.0_f64;
                for r in 0..RATIO_NODES {
                    let arcs =
                        arc_from_apex(|s| blend.curvature(cut, s), ratio_node(r) * cap, count);
                    envelope = envelope.max(miss_of(&|s| blend.point(cut, s), &arcs));
                    miss[(t * CUT_NODES + c) * RATIO_NODES + r] = envelope;
                }
            }
        }
        peak_curvature_cut[0] = peak_curvature_cut[1];
        let square_at_speed = std::array::from_fn(|i| sharp(SQUARE_CORNER_RAD, speed_node(i)));
        let circle = std::array::from_fn(|i| {
            let omega_hat = circle_node(i);
            (1.0 - circle_radius_ratio(&taps, omega_hat)) / (omega_hat * omega_hat)
        });
        Some(Self {
            sigma_s,
            peak_curvature_cut,
            floor,
            miss,
            square_at_speed,
            circle,
        })
    }

    /// The process's one table for this pair of kernels, built on first use
    /// and kept for the process's life: a printer holds a handful of kernel
    /// pairs, and every pipeline on them shares the table.
    #[must_use]
    pub fn interned(kernels: [Option<&PiecewisePolynomialKernel>; 2]) -> Option<&'static Self> {
        static TABLES: OnceLock<Mutex<Vec<(KernelKey, &'static ShapedCorners)>>> = OnceLock::new();
        let key = kernels.map(|kernel| kernel.map(kernel_key));
        if key.iter().all(Option::is_none) {
            return None;
        }
        let mut tables = TABLES
            .get_or_init(|| Mutex::new(Vec::new()))
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some((_, table)) = tables.iter().find(|(known, _)| *known == key) {
            return Some(table);
        }
        let table: &'static Self = Box::leak(Box::new(Self::new(kernels)?));
        tables.push((key, table));
        Some(table)
    }

    fn length_unit(&self, accel: f64) -> f64 {
        accel * self.sigma_s * self.sigma_s
    }

    fn by_theta(&self, theta: f64, row: impl Fn(usize) -> f64) -> f64 {
        let (t, f) = bracket(theta.abs().to_degrees() / THETA_STEP_DEG, THETA_NODES);
        row(t) * (1.0 - f) + row(t + 1) * f
    }

    /// `m` at turn node `t`, cut `ĉ` over the length unit, ratio node `r`.
    fn cut_row(&self, t: usize, cut_hat: f64, r: usize) -> f64 {
        let at = |c: usize| self.miss[(t * CUT_NODES + c) * RATIO_NODES + r];
        let lowest = cut_node(0);
        let highest = cut_node(CUT_NODES - 1);
        if cut_hat <= 0.0 {
            self.floor[t]
        } else if cut_hat < lowest {
            let f = cut_hat / lowest;
            self.floor[t] * (1.0 - f) + at(0) * f
        } else if cut_hat > highest {
            at(CUT_NODES - 1) - highest + cut_hat
        } else {
            let (c, f) = bracket(cut_hat.log2() - f64::from(CUT_OCTAVE_LOW), CUT_NODES);
            at(c) * (1.0 - f) + at(c + 1) * f
        }
    }

    fn at_ratio_node(&self, theta: f64, cut_mm: f64, accel: f64, r: usize) -> f64 {
        let unit = self.length_unit(accel);
        let cut_hat = cut_mm / unit;
        let raw = self.by_theta(theta, |t| self.cut_row(t, cut_hat, r));
        unit * (cut_hat + PREDICTION_MARGIN * (raw - cut_hat).max(0.0))
    }

    /// The shaped path's closest approach to the vertex of a corner turning
    /// `theta`, blended with apex `cut_mm` from the vertex and crossed at
    /// `speed_ratio` of the blend's curvature ceiling under `accel`.
    #[must_use]
    pub fn miss_mm(&self, theta: f64, cut_mm: f64, speed_ratio: f64, accel: f64) -> f64 {
        let (r, f) = bracket(speed_ratio * (RATIO_NODES - 1) as f64, RATIO_NODES);
        self.at_ratio_node(theta, cut_mm, accel, r) * (1.0 - f)
            + self.at_ratio_node(theta, cut_mm, accel, r + 1) * f
    }

    /// The miss of a stop at the sharp corner: what the shaper alone rounds
    /// it by at `accel`.
    #[must_use]
    pub fn floor_mm(&self, theta: f64, accel: f64) -> f64 {
        PREDICTION_MARGIN * self.length_unit(accel) * self.by_theta(theta, |t| self.floor[t])
    }

    /// What crossing a sharp 90° corner at `scv` adds to its floor at
    /// `accel`: the rounding `square_corner_velocity` grants beyond it,
    /// extrapolated linearly in speed past the table.
    #[must_use]
    pub fn scv_allowance_mm(&self, scv: f64, accel: f64) -> f64 {
        let speed_hat = scv / (accel * self.sigma_s);
        let square_floor = self.by_theta(SQUARE_CORNER_RAD, |t| self.floor[t]);
        let at_speed = if speed_hat <= speed_node(0) {
            let f = speed_hat / speed_node(0);
            square_floor * (1.0 - f) + self.square_at_speed[0] * f
        } else if speed_hat >= speed_node(SPEED_NODES - 1) {
            let (a, b) = (speed_node(SPEED_NODES - 2), speed_node(SPEED_NODES - 1));
            let slope = (self.square_at_speed[SPEED_NODES - 1]
                - self.square_at_speed[SPEED_NODES - 2])
                / (b - a);
            self.square_at_speed[SPEED_NODES - 1] + slope * (speed_hat - b)
        } else {
            let (i, f) = bracket(speed_hat.log2() - f64::from(SPEED_OCTAVE_LOW), SPEED_NODES);
            self.square_at_speed[i] * (1.0 - f) + self.square_at_speed[i + 1] * f
        };
        PREDICTION_MARGIN * self.length_unit(accel) * (at_speed - square_floor).max(0.0)
    }

    /// The shaped miss a corner turning `theta` at `accel` may take.
    #[must_use]
    pub fn budget_mm(&self, budget: CornerBudget, theta: f64, accel: f64) -> f64 {
        match budget {
            CornerBudget::Shaped(mm) => mm,
            CornerBudget::OverFloor(allowance) => self.floor_mm(theta, accel) + allowance,
            CornerBudget::ScvEquivalent(scv) => {
                self.floor_mm(theta, accel) + self.scv_allowance_mm(scv, accel)
            }
        }
    }

    /// `κ_peak` of the symmetric blend turning `theta` with apex `cut_mm`
    /// from the vertex.
    #[must_use]
    pub fn blend_peak_curvature(&self, theta: f64, cut_mm: f64) -> f64 {
        self.by_theta(theta, |t| self.peak_curvature_cut[t]) / cut_mm
    }

    /// The largest share of its curvature ceiling a blend may cross its apex
    /// at within `budget_mm`; `None` where even a stop there misses by more.
    #[must_use]
    pub fn apex_speed_ratio(
        &self,
        theta: f64,
        cut_mm: f64,
        accel: f64,
        budget_mm: f64,
    ) -> Option<f64> {
        let miss = |ratio: f64| self.miss_mm(theta, cut_mm, ratio, accel);
        if miss(0.0) > budget_mm {
            return None;
        }
        if miss(1.0) <= budget_mm {
            return Some(1.0);
        }
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..RATIO_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if miss(mid) <= budget_mm {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Some(lo)
    }

    /// The acceleration a corner whose stop at `accel` misses past
    /// `budget_mm` is slowed to: a stop there takes [`STOP_SHARE`] of the
    /// budget's room beyond the cut, or all of it at a sharp corner, which
    /// has no apex speed to spend the rest on. A cut the budget does not
    /// cover fails loudly.
    #[must_use]
    pub fn corner_accel(&self, theta: f64, cut_mm: f64, accel: f64, budget_mm: f64) -> f64 {
        let stopped = |a: f64| self.miss_mm(theta, cut_mm, 0.0, a);
        if stopped(accel) <= budget_mm {
            return accel;
        }
        assert!(
            cut_mm < budget_mm,
            "a corner turning {theta} rad is cut {cut_mm} mm, past its whole shaped budget {budget_mm} mm"
        );
        let target = if cut_mm > 0.0 {
            cut_mm + STOP_SHARE * (budget_mm - cut_mm)
        } else {
            budget_mm
        };
        let (mut lo, mut hi) = (0.0, accel);
        for _ in 0..ACCEL_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if stopped(mid) <= target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// The fitter's cut for a line-line corner turning `theta` under
    /// `accel`: the one whose blend crosses its apex fastest within the
    /// corner's shaped budget, at the acceleration [`Self::corner_accel`]
    /// leaves it; 0 where no blend beats a stop.
    #[must_use]
    pub fn blend_cut_mm(&self, budget: CornerBudget, theta: f64, accel: f64) -> f64 {
        let budget_mm = self.budget_mm(budget, theta, accel);
        let room = self.cut_room_mm(budget, accel);
        let accel = if self.floor_mm(theta, accel) <= budget_mm {
            accel
        } else {
            self.corner_accel(theta, 0.0, accel, STOP_SHARE * budget_mm)
        };
        (1..=CUT_CANDIDATES)
            .map(|j| room * 2.0_f64.powf(-0.5 * f64::from(j)))
            .filter_map(|cut| {
                let ratio = self.apex_speed_ratio(theta, cut, accel, budget_mm)?;
                Some((
                    ratio * (accel / self.blend_peak_curvature(theta, cut)).sqrt(),
                    cut,
                ))
            })
            .fold(
                (0.0, 0.0),
                |best, next| if next.0 > best.0 { next } else { best },
            )
            .1
    }

    /// The most a blend may cut under `budget` whatever acceleration at or
    /// under `accel` its corner is planned at: an implicit budget's
    /// allowance over the floor shrinks with the acceleration the planner
    /// leaves the corner, and the cut must stay within it there.
    fn cut_room_mm(&self, budget: CornerBudget, accel: f64) -> f64 {
        match budget {
            CornerBudget::Shaped(mm) | CornerBudget::OverFloor(mm) => mm,
            CornerBudget::ScvEquivalent(scv) => (0..=ROOM_ACCEL_OCTAVES)
                .map(|k| self.scv_allowance_mm(scv, accel * 2.0_f64.powi(-k)))
                .fold(f64::INFINITY, f64::min),
        }
    }

    /// How far the shaper pulls a circle of `radius_mm` crossed at
    /// `speed_mm_s` inward.
    #[must_use]
    pub fn arc_shrink_mm(&self, radius_mm: f64, speed_mm_s: f64) -> f64 {
        let omega_hat = speed_mm_s * self.sigma_s / radius_mm;
        let (i, f) = bracket(
            omega_hat / CIRCLE_OMEGA_MAX * (CIRCLE_NODES - 1) as f64,
            CIRCLE_NODES,
        );
        let per_square_rate = self.circle[i] * (1.0 - f) + self.circle[i + 1] * f;
        PREDICTION_MARGIN * radius_mm * (omega_hat * omega_hat * per_square_rate).clamp(0.0, 1.0)
    }

    /// The fastest an arc of `radius_mm` may be crossed with the shaper's
    /// pull within `room_mm`.
    #[must_use]
    pub fn arc_speed_cap(&self, radius_mm: f64, room_mm: f64) -> f64 {
        if room_mm <= 0.0 {
            return 0.0;
        }
        let speed_at = |i: usize| circle_node(i) * radius_mm / self.sigma_s;
        let over =
            (1..CIRCLE_NODES).find(|&i| self.arc_shrink_mm(radius_mm, speed_at(i)) > room_mm);
        let Some(i) = over else {
            return speed_at(CIRCLE_NODES - 1);
        };
        let (mut lo, mut hi) = (speed_at(i - 1), speed_at(i));
        for _ in 0..RATIO_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if self.arc_shrink_mm(radius_mm, mid) <= room_mm {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// Whether a stop at a corner turning `theta`, blended with apex
    /// `cut_mm` from the vertex, misses past the corner's budget at `accel`:
    /// the corner then slows the moves leading into and out of it.
    #[must_use]
    pub fn slows(&self, budget: CornerBudget, theta: f64, cut_mm: f64, accel: f64) -> bool {
        self.miss_mm(theta, cut_mm, 0.0, accel) > self.budget_mm(budget, theta, accel)
    }

    /// The length of path either side of a corner a slower acceleration
    /// needs to cover for the shaper to read no faster motion through it.
    #[must_use]
    pub fn approach_mm(&self, accel: f64) -> f64 {
        APPROACH_SIGMAS_SQUARED * self.length_unit(accel)
    }

    /// The tolerance an arc or a merged run of facets is fitted to under
    /// `budget` at `accel`.
    #[must_use]
    pub fn arc_fit_tolerance_mm(&self, budget: CornerBudget, accel: f64) -> f64 {
        ARC_FIT_SHARE
            * match budget {
                CornerBudget::Shaped(mm) | CornerBudget::OverFloor(mm) => mm,
                CornerBudget::ScvEquivalent(scv) => self.scv_allowance_mm(scv, accel),
            }
    }
}

/// The turn and the apex's distance from the vertex of a clothoid pair
/// whose curvature peaks at the seam between `up` and `down`: the vertex is
/// where the pair's entry and exit tangents meet.
pub fn blend_apex(up: &Move, down: &Move) -> Option<(f64, f64)> {
    let (Some(Segment::Clothoid(first)), Some(Segment::Clothoid(second))) =
        (&up.segment.spatial, &down.segment.spatial)
    else {
        return None;
    };
    let (up_start, up_end) = first.kappa_endpoints();
    let (down_start, down_end) = second.kappa_endpoints();
    let peaks =
        up_end.abs() >= up_start.abs() && down_start.abs() >= down_end.abs() && up_end.abs() > 0.0;
    if !peaks {
        return None;
    }
    let entry = first.point_at(0.0);
    let entry_heading = first.heading_at(0.0);
    let exit = second.point_at(second.s_len());
    let exit_heading = second.heading_at(second.s_len());
    let theta = dot(entry_heading, exit_heading).clamp(-1.0, 1.0).acos();
    if theta < TURN_EPS_RAD {
        return None;
    }
    let vertex = tangent_meet(entry, entry_heading, exit, exit_heading);
    Some((theta, norm(sub(first.point_at(first.s_len()), vertex))))
}

/// The point midway between the closest points of the lines through
/// `a` along `u` and through `b` along `w`.
fn tangent_meet(a: [f64; 3], u: [f64; 3], b: [f64; 3], w: [f64; 3]) -> [f64; 3] {
    let r = sub(a, b);
    let (uu, uw, ww) = (dot(u, u), dot(u, w), dot(w, w));
    let (ur, wr) = (dot(u, r), dot(w, r));
    let det = uu * ww - uw * uw;
    let s = (uw * wr - ww * ur) / det;
    let t = (uu * wr - uw * ur) / det;
    let p = [a[0] + s * u[0], a[1] + s * u[1], a[2] + s * u[2]];
    let q = [b[0] + t * w[0], b[1] + t * w[1], b[2] + t * w[2]];
    [
        0.5 * (p[0] + q[0]),
        0.5 * (p[1] + q[1]),
        0.5 * (p[2] + q[2]),
    ]
}

/// The turn a stop between two spatial moves makes.
pub fn stop_turn(before: &Move, after: &Move) -> Option<f64> {
    let (Some(incoming), Some(outgoing)) = (&before.segment.spatial, &after.segment.spatial) else {
        return None;
    };
    let theta = dot(
        incoming.heading_at(incoming.s_len()),
        outgoing.heading_at(0.0),
    )
    .clamp(-1.0, 1.0)
    .acos();
    (theta >= TURN_EPS_RAD).then_some(theta)
}

type KernelKey = [Option<Vec<u64>>; 2];

fn kernel_key(kernel: &PiecewisePolynomialKernel) -> Vec<u64> {
    kernel
        .pieces
        .iter()
        .flat_map(|piece| {
            [piece.u_start.to_bits(), piece.u_end.to_bits()]
                .into_iter()
                .chain(piece.coeffs.iter().map(|c| c.to_bits()))
        })
        .collect()
}

/// The least radius, over its phase, of a unit circle shaped by `taps` at
/// angular rate `omega_hat` per `T`.
fn circle_radius_ratio(taps: &[Taps; 2], omega_hat: f64) -> f64 {
    let (gx, gy) = (taps[0].gain(omega_hat), taps[1].gain(omega_hat));
    (0..CIRCLE_PHASES)
        .map(|k| {
            let u = std::f64::consts::TAU * k as f64 / CIRCLE_PHASES as f64;
            let (sin, cos) = u.sin_cos();
            let x = gx.0 * cos - gx.1 * sin;
            let y = gy.0 * sin + gy.1 * cos;
            x.hypot(y)
        })
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests;
