use nurbs::algebra::PiecewisePolynomialKernel;

use super::screen::MoveSummary;
use super::shadow::{GridSample, PlannedGrid};

/// Gauss–Legendre nodes and weights on `[-1, 1]`, exact for polynomials
/// up to degree 15.
const GAUSS: [(f64, f64); 8] = [
    (-0.960_289_856_497_536_3, 0.101_228_536_290_376_26),
    (-0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (-0.525_532_409_916_329_0, 0.313_706_645_877_887_3),
    (-0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329_0, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (0.960_289_856_497_536_3, 0.101_228_536_290_376_26),
];

/// A small deterministic generator for the tests' random cases.
pub(super) struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    pub fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn between(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

/// A planned motion of constant acceleration vectors: from `t = 0` the
/// pieces in turn, at constant velocity before and after.
pub(super) struct Motion {
    v0: [f64; 3],
    pieces: Vec<(f64, [f64; 3])>,
    starts: Vec<(f64, [f64; 3], [f64; 3])>,
}

impl Motion {
    pub fn new(v0: [f64; 3], pieces: Vec<(f64, [f64; 3])>) -> Self {
        let mut starts = Vec::with_capacity(pieces.len());
        let (mut t, mut p, mut v) = (0.0, [0.0; 3], v0);
        for &(dt, a) in &pieces {
            starts.push((t, p, v));
            for axis in 0..3 {
                p[axis] += v[axis] * dt + 0.5 * a[axis] * dt * dt;
                v[axis] += a[axis] * dt;
            }
            t += dt;
        }
        starts.push((t, p, v));
        Self { v0, pieces, starts }
    }

    pub fn end(&self) -> f64 {
        self.starts.last().map_or(0.0, |start| start.0)
    }

    /// The acceleration steps: the instants the acceleration jumps, with the
    /// jump.
    fn steps(&self) -> Vec<(f64, [f64; 3])> {
        let mut before = [0.0; 3];
        let mut steps = Vec::new();
        for (index, &(_, a)) in self.pieces.iter().enumerate() {
            steps.push((self.starts[index].0, sub(a, before)));
            before = a;
        }
        steps.push((self.end(), sub([0.0; 3], before)));
        steps
    }

    pub fn state(&self, t: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
        if t < 0.0 {
            return (std::array::from_fn(|axis| self.v0[axis] * t), self.v0, [0.0; 3]);
        }
        let index = self
            .starts
            .partition_point(|start| start.0 <= t)
            .saturating_sub(1);
        let (t0, p, v) = self.starts[index];
        let a = self.pieces.get(index).map_or([0.0; 3], |piece| piece.1);
        let d = t - t0;
        (
            std::array::from_fn(|axis| p[axis] + v[axis] * d + 0.5 * a[axis] * d * d),
            std::array::from_fn(|axis| v[axis] + a[axis] * d),
            a,
        )
    }

    fn breaks(&self) -> Vec<f64> {
        self.starts.iter().map(|start| start.0).collect()
    }

    /// The path length from `t = −1 s`, by Gauss–Legendre between the
    /// acceleration steps cut fine.
    pub fn arc(&self, t: f64) -> f64 {
        let mut cuts = vec![-1.0];
        cuts.extend(self.breaks().into_iter().filter(|&b| b > -1.0 && b < t));
        cuts.push(t);
        cuts.windows(2)
            .map(|span| {
                let pieces = 32;
                let step = (span[1] - span[0]) / pieces as f64;
                (0..pieces)
                    .map(|index| {
                        let a = span[0] + step * index as f64;
                        gauss(a, a + step, |tau| norm(self.state(tau).1))
                    })
                    .sum::<f64>()
            })
            .sum()
    }

    /// The motion on the grid `k·h` for `k` in `range`, with each cell's
    /// acceleration bound.
    pub fn grid(&self, h: f64, range: std::ops::Range<i64>) -> PlannedGrid {
        let mut grid = PlannedGrid::default();
        let mut arc = self.arc(range.start as f64 * h);
        for k in range {
            let (tau, next) = (k as f64 * h, (k + 1) as f64 * h);
            let (position, velocity, _) = self.state(tau);
            let cell_accel = self.cell_accel(tau, next);
            grid.push(
                k,
                GridSample {
                    position,
                    velocity,
                    arc,
                    cell_accel,
                },
            );
            arc += gauss(tau, next, |t| norm(self.state(t).1));
        }
        grid
    }

    fn cell_accel(&self, from: f64, to: f64) -> [f64; 3] {
        let mut cuts = vec![from];
        cuts.extend(self.breaks().into_iter().filter(|&b| b > from && b < to));
        cuts.push(to);
        cuts.windows(2).fold([0.0; 3], |acc, span| {
            let a = self.state(0.5 * (span[0] + span[1])).2;
            std::array::from_fn(|axis| acc[axis].max(a[axis].abs()))
        })
    }

    /// The exact shaped velocity, acceleration and jerk at `t` through the
    /// per-axis kernels, `None` leaving the axis out.
    pub fn shaped(
        &self,
        kernels: &[Option<&PiecewisePolynomialKernel>; 3],
        t: f64,
    ) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let mut out = ([0.0; 3], [0.0; 3], [0.0; 3]);
        let steps = self.steps();
        for (axis, kernel) in kernels.iter().enumerate() {
            let Some(kernel) = kernel else {
                continue;
            };
            let (lo, hi) = kernel.support();
            let mut cuts = vec![lo, hi];
            cuts.extend(kernel.pieces.iter().map(|piece| piece.u_start));
            cuts.extend(self.breaks().into_iter().map(|b| t - b));
            cuts.retain(|&u| u >= lo && u <= hi);
            cuts.sort_by(f64::total_cmp);
            cuts.dedup();
            for span in cuts.windows(2) {
                out.0[axis] += gauss(span[0], span[1], |u| {
                    value(kernel, u) * self.state(t - u).1[axis]
                });
                out.1[axis] += gauss(span[0], span[1], |u| {
                    value(kernel, u) * self.state(t - u).2[axis]
                });
            }
            out.2[axis] = steps
                .iter()
                .map(|&(at, jump)| jump[axis] * value(kernel, t - at))
                .sum();
        }
        out
    }

    /// One summary per piece, and the constant-velocity stretches either
    /// side, over `[from, to]`.
    pub fn summaries(&self, from: f64, to: f64) -> Vec<MoveSummary> {
        let mut spans = vec![(f64::NEG_INFINITY, 0.0)];
        spans.extend(self.starts.windows(2).map(|pair| (pair[0].0, pair[1].0)));
        spans.push((self.end(), f64::INFINITY));
        spans
            .into_iter()
            .filter(|&(t0, t1)| t1 > from && t0 < to)
            .map(|(t0, t1)| {
                let (a, b) = (t0.max(from), t1.min(to));
                let (v_a, v_b, accel) = (self.state(a).1, self.state(b).1, self.state(0.5 * (a + b)).2);
                let slowest = distance_to_segment(v_a, v_b);
                let fastest = norm(v_a).max(norm(v_b));
                MoveSummary {
                    t0: a,
                    t1: b,
                    speed: (slowest, fastest),
                    heading: Some(heading_spans(v_a, v_b)),
                    accel: accel.map(f64::abs),
                    accel_bound: norm(accel),
                    moves_on: std::array::from_fn(|axis| {
                        v_a[axis] != 0.0 || v_b[axis] != 0.0 || accel[axis] != 0.0
                    }),
                    ratio: None,
                }
            })
            .collect()
    }
}

fn value(kernel: &PiecewisePolynomialKernel, u: f64) -> f64 {
    kernel
        .pieces
        .iter()
        .find(|piece| piece.u_start <= u && u < piece.u_end)
        .map_or(0.0, |piece| piece.evaluate(u))
}

pub(super) fn gauss(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let (mid, half) = (0.5 * (a + b), 0.5 * (b - a));
    GAUSS
        .iter()
        .map(|&(x, w)| w * f(mid + half * x))
        .sum::<f64>()
        * half
}

pub(super) fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub(super) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(super) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn distance_to_segment(a: [f64; 3], b: [f64; 3]) -> f64 {
    let step = sub(b, a);
    let length_sq = step.iter().map(|x| x * x).sum::<f64>();
    if length_sq == 0.0 {
        return norm(a);
    }
    let along = (-(a[0] * step[0] + a[1] * step[1] + a[2] * step[2]) / length_sq).clamp(0.0, 1.0);
    norm(std::array::from_fn(|axis| a[axis] + along * step[axis]))
}

/// The range of each component of `v/|v|` along the chord from `a` to `b`:
/// the direction sweeps the arc of the great circle between their
/// directions, or every direction of their plane where the chord passes
/// the origin.
fn heading_spans(a: [f64; 3], b: [f64; 3]) -> [(f64, f64); 3] {
    if distance_to_segment(a, b) <= 1e-9 * (1.0 + norm(a).max(norm(b))) {
        return [(-1.0, 1.0); 3];
    }
    let (na, nb) = (norm(a), norm(b));
    let u = a.map(|x| x / na);
    let w = b.map(|x| x / nb);
    let cos = (u[0] * w[0] + u[1] * w[1] + u[2] * w[2]).clamp(-1.0, 1.0);
    let sweep = cos.acos();
    let normal = sub(w, u.map(|x| x * cos));
    let normal_len = norm(normal);
    std::array::from_fn(|axis| {
        if normal_len <= 1e-12 {
            return (u[axis], u[axis]);
        }
        let v = normal[axis] / normal_len;
        let plane = (u[axis] * u[axis] + v * v).sqrt();
        let alpha = v.atan2(u[axis]);
        let (from, to) = (-alpha, sweep - alpha);
        let ends = [from.cos(), to.cos()];
        let two_pi = 2.0 * std::f64::consts::PI;
        let crest = (from / two_pi).ceil() * two_pi;
        let trough = ((from - std::f64::consts::PI) / two_pi).ceil() * two_pi + std::f64::consts::PI;
        let top = if crest <= to { 1.0 } else { ends[0].max(ends[1]) };
        let bottom = if trough <= to { -1.0 } else { ends[0].min(ends[1]) };
        (plane * bottom, plane * top)
    })
}
