use nurbs::algebra::PiecewisePolynomialKernel;
#[cfg(test)]
use nurbs::ScalarNurbs;

#[cfg(test)]
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

const MAX_EXACT_PRODUCT_DEGREE: usize = 13;
const GAUSS_1_NODES: [f64; 1] = [0.0];
const GAUSS_1_WEIGHTS: [f64; 1] = [2.0];
const GAUSS_2_NODES: [f64; 2] = [-0.577_350_269_189_625_7, 0.577_350_269_189_625_7];
const GAUSS_2_WEIGHTS: [f64; 2] = [1.0, 1.0];
const GAUSS_3_NODES: [f64; 3] = [-0.774_596_669_241_483_4, 0.0, 0.774_596_669_241_483_4];
const GAUSS_3_WEIGHTS: [f64; 3] = [
    0.555_555_555_555_555_6,
    0.888_888_888_888_888_8,
    0.555_555_555_555_555_6,
];
const GAUSS_4_NODES: [f64; 4] = [
    -0.861_136_311_594_052_6,
    -0.339_981_043_584_856_3,
    0.339_981_043_584_856_3,
    0.861_136_311_594_052_6,
];
const GAUSS_4_WEIGHTS: [f64; 4] = [
    0.347_854_845_137_453_8,
    0.652_145_154_862_546_1,
    0.652_145_154_862_546_1,
    0.347_854_845_137_453_8,
];
const GAUSS_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
const GAUSS_5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];
const GAUSS_6_NODES: [f64; 6] = [
    -0.932_469_514_203_152,
    -0.661_209_386_466_264_5,
    -0.238_619_186_083_196_9,
    0.238_619_186_083_196_9,
    0.661_209_386_466_264_5,
    0.932_469_514_203_152,
];
const GAUSS_6_WEIGHTS: [f64; 6] = [
    0.171_324_492_379_170_4,
    0.360_761_573_048_138_6,
    0.467_913_934_572_691,
    0.467_913_934_572_691,
    0.360_761_573_048_138_6,
    0.171_324_492_379_170_4,
];
const GAUSS_7_NODES: [f64; 7] = [
    -0.949_107_912_342_758_5,
    -0.741_531_185_599_394_5,
    -0.405_845_151_377_397_2,
    0.0,
    0.405_845_151_377_397_2,
    0.741_531_185_599_394_5,
    0.949_107_912_342_758_5,
];
const GAUSS_7_WEIGHTS: [f64; 7] = [
    0.129_484_966_168_869_7,
    0.279_705_391_489_276_6,
    0.381_830_050_505_118_9,
    0.417_959_183_673_469_4,
    0.381_830_050_505_118_9,
    0.279_705_391_489_276_6,
    0.129_484_966_168_869_7,
];

fn quadrature_rule(product_degree: usize) -> (&'static [f64], &'static [f64]) {
    match product_degree {
        0..=1 => (&GAUSS_1_NODES, &GAUSS_1_WEIGHTS),
        2..=3 => (&GAUSS_2_NODES, &GAUSS_2_WEIGHTS),
        4..=5 => (&GAUSS_3_NODES, &GAUSS_3_WEIGHTS),
        6..=7 => (&GAUSS_4_NODES, &GAUSS_4_WEIGHTS),
        8..=9 => (&GAUSS_5_NODES, &GAUSS_5_WEIGHTS),
        10..=11 => (&GAUSS_6_NODES, &GAUSS_6_WEIGHTS),
        12..=13 => (&GAUSS_7_NODES, &GAUSS_7_WEIGHTS),
        _ => panic!("convolution product degree {product_degree} exceeds 13"),
    }
}

const CUT_DEDUP_EPS_S: f64 = 1e-12;

/// The convolution `(input ∗ kernel)(t)`, evaluated exactly: both factors are
/// piecewise polynomials, so integrating between their breakpoints with a
/// Gauss rule of sufficient order carries no quadrature error at all. The
/// previous sampled rectangle-rule evaluator corrugated the result at the
/// sample wavelength — noise invisible in position but fatal to the refit's
/// second-difference acceleration probes, which chased it into subdividing
/// every span to the floor.
pub struct ShapedSignal<'a, F = Box<dyn Fn(f64) -> f64 + 'a>> {
    eval_input: F,
    gauss_nodes: &'static [f64],
    gauss_weights: &'static [f64],
    /// Sorted times where the input signal changes polynomial (piece seams,
    /// segment boundaries, clamp edges). Between two consecutive cuts the
    /// integrand is one polynomial, which the Gauss rule integrates exactly.
    input_breaks: Vec<f64>,
    /// Reusable cut buffer for `convolve` — the merge of kernel-piece
    /// boundaries and in-window input breaks, rebuilt on every call.
    cuts: std::cell::RefCell<Vec<f64>>,
    /// Recent `(t, (p, v, a))` results: adjacent fit spans share boundary
    /// times (a segment's end is the next segment's start, and bisection
    /// re-evaluates its parent's endpoints), so a tiny exact-`t` memo removes
    /// whole convolution passes without changing any computed value.
    pva_memo: std::cell::RefCell<[Option<(u64, (f64, f64, f64))>; 4]>,
    pva_memo_next: std::cell::Cell<usize>,
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
    #[cfg(test)]
    pub fn new(padded: &'a ScalarNurbs, kernel: &'a PiecewisePolynomialKernel) -> Self {
        let mut breaks = padded.knots().to_vec();
        breaks.dedup_by(|a, b| (*a - *b).abs() <= CUT_DEDUP_EPS_S);
        Self::new_from_evaluator(
            kernel,
            Box::new(move |t| eval_clamped(padded, t)),
            breaks,
            padded.degree() as usize,
        )
    }
}

impl<'a, F> ShapedSignal<'a, F>
where
    F: Fn(f64) -> f64,
{
    pub fn new_from_evaluator(
        kernel: &'a PiecewisePolynomialKernel,
        eval: F,
        mut input_breaks: Vec<f64>,
        input_degree: usize,
    ) -> Self {
        let (k_lo, k_hi) = kernel.support();
        assert!(
            (k_hi - k_lo).is_finite() && k_hi - k_lo > 0.0,
            "shaper kernel support width must be finite and positive"
        );
        let kernel_degree = kernel
            .pieces
            .iter()
            .map(|piece| piece.degree())
            .max()
            .expect("shaper kernel has no pieces");
        assert!(
            input_degree + kernel_degree <= MAX_EXACT_PRODUCT_DEGREE,
            "convolution product degree {} exceeds exact quadrature degree {MAX_EXACT_PRODUCT_DEGREE}",
            input_degree + kernel_degree
        );
        let (gauss_nodes, gauss_weights) = quadrature_rule(input_degree + kernel_degree);
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
            eval_input: eval,
            gauss_nodes,
            gauss_weights,
            input_breaks,
            cuts: std::cell::RefCell::new(Vec::new()),
            pva_memo: std::cell::RefCell::new([None; 4]),
            pva_memo_next: std::cell::Cell::new(0),
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

    /// `(eval, deriv, second_deriv)` at `t` in one pass: the three kernels
    /// share piece boundaries (differentiation preserves them), so the cut
    /// merge and every `eval_input` sample — the expensive part on dense
    /// micro-segment windows — are computed once instead of three times.
    /// Accumulation mirrors `convolve` op for op, so each component is
    /// bit-identical to the separate `eval`/`deriv`/`second_deriv` calls.
    pub fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let key = t.to_bits();
        if let Some(hit) = self
            .pva_memo
            .borrow()
            .iter()
            .flatten()
            .find(|(bits, _)| *bits == key)
        {
            return hit.1;
        }
        let mut cuts = self.cuts.borrow_mut();
        self.merge_cuts(t, &mut cuts);

        let mut kernel_idx = 0usize;
        let (mut p, mut v, mut a) = (0.0_f64, 0.0_f64, 0.0_f64);
        for w in cuts.windows(2) {
            let (lo, hi) = (w[0], w[1]);
            let half = 0.5 * (hi - lo);
            if half <= 0.0 {
                continue;
            }
            let mid = 0.5 * (lo + hi);
            while kernel_idx + 1 < self.kernel.pieces.len()
                && self.kernel.pieces[kernel_idx].u_end <= mid
            {
                kernel_idx += 1;
            }
            let k = &self.kernel.pieces[kernel_idx];
            let kd = &self.d_kernel.pieces[kernel_idx];
            let kdd = &self.dd_kernel.pieces[kernel_idx];
            let (mut sp, mut sv, mut sa) = (0.0_f64, 0.0_f64, 0.0_f64);
            for (node, weight) in self.gauss_nodes.iter().zip(self.gauss_weights) {
                let tau = nurbs::fmadd(*node, half, mid);
                let f = weight * (self.eval_input)(t - tau);
                sp += f * k.evaluate(tau);
                sv += f * kd.evaluate(tau);
                sa += f * kdd.evaluate(tau);
            }
            p += sp * half;
            v += sv * half;
            a += sa * half;
        }
        for &(tau, jump) in &self.d_kernel_jumps {
            a += jump * (self.eval_input)(t - tau);
        }
        {
            let slot = self.pva_memo_next.get();
            self.pva_memo.borrow_mut()[slot] = Some((key, (p, v, a)));
            self.pva_memo_next.set((slot + 1) % 4);
        }
        (p, v, a)
    }

    /// Merge the kernel-piece boundaries (ascending by construction) with the
    /// in-window input breaks (`t - b` is ascending over `input_breaks`
    /// iterated in reverse), deduplicating on the fly — no per-call sort.
    fn merge_cuts(&self, t: f64, cuts: &mut Vec<f64>) {
        cuts.clear();
        let b_lo = self
            .input_breaks
            .partition_point(|&b| b <= t - self.k_hi + CUT_DEDUP_EPS_S);
        let b_hi = self
            .input_breaks
            .partition_point(|&b| b < t - self.k_lo - CUT_DEDUP_EPS_S);
        let mut breaks = self.input_breaks[b_lo..b_hi].iter().rev().peekable();
        let push = |v: f64, cuts: &mut Vec<f64>| {
            if cuts.last().is_none_or(|&last| v - last > CUT_DEDUP_EPS_S) {
                cuts.push(v);
            }
        };
        for boundary in self
            .kernel
            .pieces
            .iter()
            .map(|p| p.u_start)
            .chain(std::iter::once(self.k_hi))
        {
            while let Some(&&b) = breaks.peek() {
                if t - b >= boundary {
                    break;
                }
                push(t - b, cuts);
                breaks.next();
            }
            push(boundary, cuts);
        }
    }

    fn convolve(&self, t: f64, kernel: &PiecewisePolynomialKernel) -> f64 {
        let mut cuts = self.cuts.borrow_mut();
        self.merge_cuts(t, &mut cuts);

        let mut kernel_idx = 0usize;
        let mut acc = 0.0_f64;
        for w in cuts.windows(2) {
            let (lo, hi) = (w[0], w[1]);
            let half = 0.5 * (hi - lo);
            if half <= 0.0 {
                continue;
            }
            let mid = 0.5 * (lo + hi);
            // Every interval lies inside one kernel piece: the merge kept all
            // piece boundaries, so advancing past pieces ending at or before
            // `mid` lands on the covering piece without a per-node scan.
            while kernel_idx + 1 < kernel.pieces.len() && kernel.pieces[kernel_idx].u_end <= mid {
                kernel_idx += 1;
            }
            let piece = &kernel.pieces[kernel_idx];
            let mut sub = 0.0_f64;
            for (node, weight) in self.gauss_nodes.iter().zip(self.gauss_weights) {
                let tau = nurbs::fmadd(*node, half, mid);
                sub += weight * (self.eval_input)(t - tau) * piece.evaluate(tau);
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
