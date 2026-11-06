use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

/// Sub-intervals each kernel piece is cut into to bound its integrals.
const INTEGRAL_SPLITS: usize = 64;

/// A kernel function linearised about the middle of one grid cell:
/// `f(u) = at + slope·(u − u_mid) + e(u)` with `|e| ≤ miss` across the cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Linear {
    pub at: f64,
    pub slope: f64,
    pub miss: f64,
}

/// One leader axis's kernel `w` cut into the cells `[j·h, (j+1)·h]` of the
/// guard's grid, each with `w` and its regular slope `w′` linearised, plus
/// what reaches the shaped jerk outside those cells.
///
/// Every bound is rigorous: a polynomial on an interval is a convex
/// combination of its Bernstein coefficients there, so the largest of them
/// bounds it, and their signs bound its sign.
#[derive(Debug, Clone)]
pub(super) struct KernelCells {
    /// The index `j` of the first cell.
    pub first: i64,
    pub value: Vec<Linear>,
    pub slope: Vec<Linear>,
    /// `Σ|J_b|` over the kernel's steps, its support ends included: each
    /// adds `J_b·a(t − u_b)` to the shaped jerk.
    pub jumps: f64,
    /// `∫|w″| + Σ|Δw′|`: the shaped jerk's rate per unit of planned
    /// acceleration, away from the steps the kernel's own jumps take.
    pub jerk_rate_per_accel: f64,
    /// Upper bounds on `∫w⁺` and `∫w⁻`.
    pub positive_mass: f64,
    pub negative_mass: f64,
    /// A bound on the total variation of `w`, its steps included.
    pub variation: f64,
    /// Each cell's bound on `|w|`, and on `|w′|` off the kernel's steps.
    value_peaks: RangeMax,
    slope_peaks: RangeMax,
}

impl Linear {
    /// The largest `|f|` across its cell of width `h`.
    fn peak(&self, h: f64) -> f64 {
        self.at.abs() + 0.5 * h * self.slope.abs() + self.miss
    }
}

/// Whether a cell bound is read off `w` or off its regular slope `w′`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Of {
    Value,
    Slope,
}

/// The largest of a run of values over any range of them, in two lookups:
/// level `k` holds the largest of every `2ᵏ` consecutive values.
#[derive(Debug, Clone)]
struct RangeMax {
    levels: Vec<Vec<f64>>,
}

impl RangeMax {
    fn new(values: Vec<f64>) -> Self {
        let count = values.len();
        let mut levels = vec![values];
        let mut width = 1;
        while 2 * width <= count {
            let below = &levels[levels.len() - 1];
            let level = (0..=count - 2 * width)
                .map(|i| below[i].max(below[i + width]))
                .collect();
            levels.push(level);
            width *= 2;
        }
        Self { levels }
    }

    /// The largest of the values `lo..=hi`.
    fn over(&self, lo: usize, hi: usize) -> f64 {
        let level = (hi - lo + 1).ilog2() as usize;
        let row = &self.levels[level];
        row[lo].max(row[hi + 1 - (1 << level)])
    }
}

impl KernelCells {
    pub fn new(kernel: &PiecewisePolynomialKernel, h: f64) -> Self {
        assert!(h > 0.0, "a kernel grid needs a positive cell, got {h}");
        let (lo, hi) = kernel.support();
        let first = (lo / h).floor() as i64;
        let last = (hi / h).ceil() as i64;
        let slopes = differentiated(&kernel.pieces);
        let curvatures = differentiated(&slopes);
        let (value, slope): (Vec<Linear>, Vec<Linear>) = (first..last)
            .map(|j| {
                let (a, b) = (j as f64 * h, (j + 1) as f64 * h);
                (
                    linearise(&kernel.pieces, &slopes, a, b),
                    linearise(&slopes, &curvatures, a, b),
                )
            })
            .unzip();
        let (positive_mass, negative_mass) = signed_masses(&kernel.pieces);
        let peaks = |cells: &[Linear]| RangeMax::new(cells.iter().map(|c| c.peak(h)).collect());
        let jumps = jump_total(&kernel.pieces);
        Self {
            first,
            variation: slope.iter().map(|cell| cell.peak(h) * h).sum::<f64>() + jumps,
            value_peaks: peaks(&value),
            slope_peaks: peaks(&slope),
            value,
            slope,
            jumps,
            jerk_rate_per_accel: absolute_integral(&curvatures) + jump_total(&slopes),
            positive_mass,
            negative_mass,
        }
    }

    /// The cells' indices, `first..end`: they cover `u ∈ [first·h, end·h]`.
    pub fn span(&self) -> (i64, i64) {
        (self.first, self.first + self.value.len() as i64)
    }

    /// A bound on `|w|` (or on `|w′|` off the kernel's steps) over
    /// `u ∈ [lo, hi]`: the largest bound of the cells that interval meets,
    /// their ends included, 0 where it misses the kernel.
    pub fn peak_over(&self, of: Of, (lo, hi): (f64, f64), h: f64) -> f64 {
        let last = self.first + self.value.len() as i64 - 1;
        let from = ((lo / h).floor() as i64).max(self.first);
        let to = ((hi / h).ceil() as i64 - 1)
            .max((lo / h).floor() as i64)
            .min(last);
        if from > to {
            return 0.0;
        }
        let peaks = match of {
            Of::Value => &self.value_peaks,
            Of::Slope => &self.slope_peaks,
        };
        peaks.over((from - self.first) as usize, (to - self.first) as usize)
    }
}

fn differentiated(pieces: &[BezierPiece]) -> Vec<BezierPiece> {
    pieces.iter().map(BezierPiece::differentiate).collect()
}

fn piece_at(pieces: &[BezierPiece], u: f64) -> Option<&BezierPiece> {
    pieces
        .iter()
        .find(|piece| piece.u_start <= u && u < piece.u_end)
}

/// `f` (the pieces, zero outside them) linearised about the middle of
/// `[a, b]` with its slope `df` there.
fn linearise(f: &[BezierPiece], df: &[BezierPiece], a: f64, b: f64) -> Linear {
    let mid = 0.5 * (a + b);
    let at = piece_at(f, mid).map_or(0.0, |piece| piece.evaluate(mid));
    let slope = piece_at(df, mid).map_or(0.0, |piece| piece.evaluate(mid));
    let mut cuts = vec![a, b];
    cuts.extend(
        f.iter()
            .flat_map(|piece| [piece.u_start, piece.u_end])
            .filter(|&u| u > a && u < b),
    );
    cuts.sort_by(f64::total_cmp);
    cuts.dedup();
    let miss = cuts
        .windows(2)
        .map(|span| {
            let (origin, mut residual) = piece_at(f, 0.5 * (span[0] + span[1]))
                .map_or((span[0], vec![0.0, 0.0]), |piece| {
                    (piece.u_start, piece.coeffs.clone())
                });
            residual.resize(residual.len().max(2), 0.0);
            residual[0] -= at + slope * (origin - mid);
            residual[1] -= slope;
            largest_magnitude(&bernstein(&residual, origin, span[0], span[1]))
        })
        .fold(0.0, f64::max);
    Linear { at, slope, miss }
}

/// The Bernstein coefficients on `[a, b]` of `p(u) = Σ cᵢ·(u − origin)ⁱ`.
pub(super) fn bernstein(coeffs: &[f64], origin: f64, a: f64, b: f64) -> Vec<f64> {
    let n = coeffs.len();
    let shift = a - origin;
    let width = b - a;
    let mut shifted = vec![0.0; n];
    for (i, &c) in coeffs.iter().enumerate() {
        let mut power = 1.0;
        for k in (0..=i).rev() {
            shifted[k] += c * binomial(i, k) * power;
            power *= shift;
        }
    }
    let mut scale = 1.0;
    for value in &mut shifted {
        *value *= scale;
        scale *= width;
    }
    let degree = n - 1;
    (0..n)
        .map(|k| {
            (0..=k)
                .map(|i| binomial(k, i) / binomial(degree, i) * shifted[i])
                .sum()
        })
        .collect()
}

pub(super) fn largest_magnitude(values: &[f64]) -> f64 {
    values.iter().fold(0.0, |acc, value| acc.max(value.abs()))
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

/// `Σ|J_b|` over the steps of the pieces (zero outside them), their ends
/// included.
fn jump_total(pieces: &[BezierPiece]) -> f64 {
    let mut total = 0.0;
    let mut before = 0.0_f64;
    for (index, piece) in pieces.iter().enumerate() {
        if index > 0 && pieces[index - 1].u_end != piece.u_start {
            total += before.abs();
            before = 0.0;
        }
        total += (piece.evaluate(piece.u_start) - before).abs();
        before = piece.evaluate(piece.u_end);
    }
    total + before.abs()
}

/// The Bernstein coefficients of each of [`INTEGRAL_SPLITS`] cuts of each
/// piece, with the cut's width.
fn cuts(pieces: &[BezierPiece]) -> impl Iterator<Item = (f64, Vec<f64>)> + '_ {
    pieces.iter().flat_map(|piece| {
        let step = (piece.u_end - piece.u_start) / INTEGRAL_SPLITS as f64;
        (0..INTEGRAL_SPLITS).map(move |index| {
            let a = piece.u_start + step * index as f64;
            (step, bernstein(&piece.coeffs, piece.u_start, a, a + step))
        })
    })
}

fn absolute_integral(pieces: &[BezierPiece]) -> f64 {
    cuts(pieces)
        .map(|(width, coefficients)| width * largest_magnitude(&coefficients))
        .sum()
}

/// Upper bounds on `(∫w⁺, ∫w⁻)`: a cut whose Bernstein coefficients share a
/// sign counts on that side only, any other on both.
fn signed_masses(pieces: &[BezierPiece]) -> (f64, f64) {
    cuts(pieces).fold((0.0, 0.0), |(positive, negative), (width, coefficients)| {
        let bound = width * largest_magnitude(&coefficients);
        if coefficients.iter().all(|&c| c >= 0.0) {
            (positive + bound, negative)
        } else if coefficients.iter().all(|&c| c <= 0.0) {
            (positive, negative + bound)
        } else {
            (positive + bound, negative + bound)
        }
    })
}

#[cfg(test)]
mod tests;
