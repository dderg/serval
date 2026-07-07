use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

fn eval_clamped(curve: &ScalarNurbs, t: f64) -> f64 {
    let knots = curve.knots();
    let lo = knots[0];
    let hi = knots[knots.len() - 1];
    nurbs::eval::eval(curve, t.clamp(lo, hi))
}

fn eval_kernel(kernel: &PiecewisePolynomialKernel, z: f64) -> f64 {
    let (k_lo, k_hi) = kernel.support();
    if z < k_lo || z > k_hi {
        return 0.0;
    }
    for p in &kernel.pieces {
        if z >= p.u_start - 1e-15 && z <= p.u_end + 1e-15 {
            return p.evaluate(z);
        }
    }
    0.0
}

/// 8-point Gauss–Legendre on [−1, 1]: exact through degree 15, comfortably
/// above the degree-11 worst case (degree-7 input piece × degree-4 kernel).
const GAUSS_NODES: [f64; 8] = [
    -0.960_289_856_497_536_2,
    -0.796_666_477_413_626_7,
    -0.525_532_409_916_329,
    -0.183_434_642_495_649_8,
    0.183_434_642_495_649_8,
    0.525_532_409_916_329,
    0.796_666_477_413_626_7,
    0.960_289_856_497_536_2,
];
const GAUSS_WEIGHTS: [f64; 8] = [
    0.101_228_536_290_376_26,
    0.222_381_034_453_374_47,
    0.313_706_645_877_887_3,
    0.362_683_783_378_362,
    0.362_683_783_378_362,
    0.313_706_645_877_887_3,
    0.222_381_034_453_374_47,
    0.101_228_536_290_376_26,
];

const CUT_DEDUP_EPS_S: f64 = 1e-12;

/// The convolution `(input ∗ kernel)(t)`, evaluated exactly: both factors are
/// piecewise polynomials, so integrating between their breakpoints with a
/// Gauss rule of sufficient order carries no quadrature error at all. The
/// previous sampled rectangle-rule evaluator corrugated the result at the
/// sample wavelength — noise invisible in position but fatal to the refit's
/// second-difference acceleration probes, which chased it into subdividing
/// every span to the floor.
pub struct ShapedSignal<'a> {
    eval_input: Box<dyn Fn(f64) -> f64 + 'a>,
    /// Sorted times where the input signal changes polynomial (piece seams,
    /// segment boundaries, clamp edges). Between two consecutive cuts the
    /// integrand is one polynomial, which the Gauss rule integrates exactly.
    input_breaks: Vec<f64>,
    kernel: &'a PiecewisePolynomialKernel,
    /// `k′` and `k″` as piecewise polynomials over the same support, plus the
    /// jump of `k′` at each internal piece boundary (a delta in `k″` — the
    /// triangle kernel has them, the bell kernel does not). With these,
    /// `(f∗k)′ = f∗k′` and `(f∗k)″ = f∗k″ + Σ Δk′·f(t−τ)` are as exact as
    /// `eval` itself — no finite-difference stencil, no stencil noise.
    d_kernel: PiecewisePolynomialKernel,
    dd_kernel: PiecewisePolynomialKernel,
    d_kernel_jumps: Vec<(f64, f64)>,
    k_lo: f64,
    k_hi: f64,
}

impl<'a> ShapedSignal<'a> {
    pub fn new(padded: &'a ScalarNurbs, kernel: &'a PiecewisePolynomialKernel) -> Self {
        let mut breaks = padded.knots().to_vec();
        breaks.dedup_by(|a, b| (*a - *b).abs() <= CUT_DEDUP_EPS_S);
        Self::new_from_evaluator(kernel, |t| eval_clamped(padded, t), breaks)
    }

    pub fn new_from_evaluator<F>(
        kernel: &'a PiecewisePolynomialKernel,
        eval: F,
        mut input_breaks: Vec<f64>,
    ) -> Self
    where
        F: Fn(f64) -> f64 + 'a,
    {
        let (k_lo, k_hi) = kernel.support();
        assert!(
            (k_hi - k_lo).is_finite() && k_hi - k_lo > 0.0,
            "shaper kernel support width must be finite and positive"
        );
        input_breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        input_breaks.dedup_by(|a, b| (*a - *b).abs() <= CUT_DEDUP_EPS_S);
        let d_kernel = PiecewisePolynomialKernel {
            pieces: kernel.pieces.iter().map(|p| p.differentiate()).collect(),
        };
        let dd_kernel = PiecewisePolynomialKernel {
            pieces: d_kernel.pieces.iter().map(|p| p.differentiate()).collect(),
        };
        assert!(
            eval_kernel(kernel, k_lo).abs() <= 1e-9 && eval_kernel(kernel, k_hi).abs() <= 1e-9,
            "shaper kernel must vanish at its support edges — otherwise the \
             convolution differentiates into boundary deltas eval() cannot carry"
        );
        let mut d_kernel_jumps: Vec<(f64, f64)> = Vec::new();
        let edge_jumps = [
            (k_lo, eval_kernel(&d_kernel, k_lo)),
            (k_hi, -eval_kernel(&d_kernel, k_hi)),
        ];
        for (tau, jump) in edge_jumps {
            if jump.abs() > 0.0 {
                d_kernel_jumps.push((tau, jump));
            }
        }
        for w in kernel.pieces.windows(2) {
            let tau = w[1].u_start;
            let jump = w[1].differentiate().evaluate(tau) - w[0].differentiate().evaluate(tau);
            if jump.abs() > 0.0 {
                d_kernel_jumps.push((tau, jump));
            }
        }
        Self {
            eval_input: Box::new(eval),
            input_breaks,
            kernel,
            d_kernel,
            dd_kernel,
            d_kernel_jumps,
            k_lo,
            k_hi,
        }
    }

    pub fn eval(&self, t: f64) -> f64 {
        self.convolve(t, self.kernel)
    }

    /// `(f∗k)′(t) = (f∗k′)(t)` — exact because `k` is continuous and vanishes
    /// at its support edges.
    pub fn deriv(&self, t: f64) -> f64 {
        self.convolve(t, &self.d_kernel)
    }

    /// `(f∗k)″(t) = (f∗k″)(t) + Σ Δk′(τ)·f(t−τ)` — the sum carries the deltas
    /// a piecewise `k′` puts into `k″` at its jump points.
    pub fn second_deriv(&self, t: f64) -> f64 {
        let mut acc = self.convolve(t, &self.dd_kernel);
        for &(tau, jump) in &self.d_kernel_jumps {
            acc += jump * (self.eval_input)(t - tau);
        }
        acc
    }

    fn convolve(&self, t: f64, kernel: &PiecewisePolynomialKernel) -> f64 {
        let mut cuts: Vec<f64> = Vec::with_capacity(self.kernel.pieces.len() + 9);
        for p in &self.kernel.pieces {
            cuts.push(p.u_start);
        }
        cuts.push(self.k_hi);
        let b_lo = self
            .input_breaks
            .partition_point(|&b| b <= t - self.k_hi + CUT_DEDUP_EPS_S);
        let b_hi = self
            .input_breaks
            .partition_point(|&b| b < t - self.k_lo - CUT_DEDUP_EPS_S);
        for &b in &self.input_breaks[b_lo..b_hi] {
            cuts.push(t - b);
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        cuts.dedup_by(|a, b| (*a - *b).abs() <= CUT_DEDUP_EPS_S);

        let mut acc = 0.0_f64;
        for w in cuts.windows(2) {
            let (lo, hi) = (w[0], w[1]);
            let half = 0.5 * (hi - lo);
            if half <= 0.0 {
                continue;
            }
            let mid = 0.5 * (lo + hi);
            let mut sub = 0.0_f64;
            for (node, weight) in GAUSS_NODES.iter().zip(&GAUSS_WEIGHTS) {
                let tau = node.mul_add(half, mid);
                sub += weight * (self.eval_input)(t - tau) * eval_kernel(kernel, tau);
            }
            acc += sub * half;
        }
        acc
    }
}

#[cfg(test)]
mod fixtures;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod long_segment_stability;
