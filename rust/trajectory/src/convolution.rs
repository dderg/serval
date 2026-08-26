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

const MAX_EXACT_PRODUCT_DEGREE: usize = 19;
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

const GAUSS_8_NODES: [f64; 8] = [
    -0.9602898564975362,
    -0.7966664774136267,
    -0.525532409916329,
    -0.18343464249564984,
    0.18343464249564984,
    0.525532409916329,
    0.7966664774136267,
    0.9602898564975362,
];
const GAUSS_8_WEIGHTS: [f64; 8] = [
    0.10122853629037652,
    0.22238103445337445,
    0.31370664587788716,
    0.3626837833783618,
    0.3626837833783618,
    0.31370664587788716,
    0.22238103445337445,
    0.10122853629037652,
];
const GAUSS_9_NODES: [f64; 9] = [
    -0.9681602395076261,
    -0.8360311073266358,
    -0.6133714327005904,
    -0.3242534234038089,
    0.0,
    0.3242534234038089,
    0.6133714327005904,
    0.8360311073266358,
    0.9681602395076261,
];
const GAUSS_9_WEIGHTS: [f64; 9] = [
    0.08127438836157368,
    0.18064816069485742,
    0.26061069640293555,
    0.3123470770400032,
    0.3302393550012601,
    0.3123470770400032,
    0.26061069640293555,
    0.18064816069485742,
    0.08127438836157368,
];
const GAUSS_10_NODES: [f64; 10] = [
    -0.9739065285171717,
    -0.8650633666889845,
    -0.6794095682990244,
    -0.43339539412924716,
    -0.14887433898163122,
    0.14887433898163122,
    0.43339539412924716,
    0.6794095682990244,
    0.8650633666889845,
    0.9739065285171717,
];
const GAUSS_10_WEIGHTS: [f64; 10] = [
    0.06667134430868729,
    0.1494513491505805,
    0.21908636251598226,
    0.26926671930999685,
    0.29552422471475304,
    0.29552422471475304,
    0.26926671930999685,
    0.21908636251598226,
    0.1494513491505805,
    0.06667134430868729,
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
        14..=15 => (&GAUSS_8_NODES, &GAUSS_8_WEIGHTS),
        16..=17 => (&GAUSS_9_NODES, &GAUSS_9_WEIGHTS),
        18..=19 => (&GAUSS_10_NODES, &GAUSS_10_WEIGHTS),
        _ => panic!("convolution product degree {product_degree} exceeds 19"),
    }
}

type MomentEvaluator<'a> = dyn Fn(f64, f64, usize, f64, &mut [f64]) -> bool + 'a;

/// The convolution `(input ∗ kernel)(t)`, evaluated exactly: both factors are
/// piecewise polynomials, so integrating between their breakpoints with a
/// Gauss rule of sufficient order carries no quadrature error at all. The
/// previous sampled rectangle-rule evaluator corrugated the result at the
/// sample wavelength — noise invisible in position but fatal to the refit's
/// second-difference acceleration probes, which chased it into subdividing
const PVA_MEMO_SLOTS: usize = 256;

/// every span to the floor.
pub struct ShapedSignal<'a, F = Box<dyn Fn(f64) -> f64 + 'a>> {
    eval_input: F,
    moment_input: Option<Box<MomentEvaluator<'a>>>,
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
    pva_memo: std::cell::RefCell<[Option<(u64, (f64, f64, f64))>; PVA_MEMO_SLOTS]>,
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
        breaks.dedup();
        Self::new_from_evaluator(
            kernel,
            Box::new(move |t| eval_clamped(padded, t)),
            breaks,
            padded.degree() as usize,
        )
    }
}

fn ordered_f64_key(value: f64) -> u64 {
    let bits = value.to_bits();
    if bits >> 63 == 0 {
        bits | (1_u64 << 63)
    } else {
        !bits
    }
}

fn f64_from_ordered_key(key: u64) -> f64 {
    let bits = if key >> 63 == 0 {
        !key
    } else {
        key & !(1_u64 << 63)
    };
    f64::from_bits(bits)
}

fn first_time_satisfying(predicate: impl Fn(f64) -> bool) -> f64 {
    let mut lower = ordered_f64_key(-f64::MAX);
    let mut upper = ordered_f64_key(f64::MAX);
    assert!(!predicate(-f64::MAX));
    assert!(predicate(f64::MAX));
    while upper - lower > 1 {
        let middle = lower + (upper - lower) / 2;
        if predicate(f64_from_ordered_key(middle)) {
            upper = middle;
        } else {
            lower = middle;
        }
    }
    f64_from_ordered_key(upper)
}

impl ShapedSignal<'_> {
    /// The cut boundaries `merge_cuts` walks, in the order it walks them: every
    /// kernel piece start followed by the support's upper edge.
    pub fn kernel_cut_boundaries<'k>(
        kernel: &'k PiecewisePolynomialKernel,
    ) -> impl Iterator<Item = f64> + 'k {
        kernel
            .pieces
            .iter()
            .map(|piece| piece.u_start)
            .chain(std::iter::once(kernel.support().1))
    }

    /// Every output time at which the cut list `merge_cuts` builds changes
    /// shape because of this `(input_break, kernel_break)` pair, appended to
    /// `transitions`.
    ///
    /// `merge_cuts` decides ownership with `t - input_break >= kernel_break`
    /// and window membership with `input_break <= t - k_hi` /
    /// `input_break < t - k_lo`. Those three comparisons round differently from
    /// the algebraically equivalent `t >= input_break + kernel_break`, so the
    /// transition is found by bisecting the comparison itself over the ordered
    /// f64 key space — 64 steps, and the answer is the first `t` the evaluator
    /// actually treats as being on the far side. Each returned time is
    /// right-owned: the discontinuity belongs to the span starting there.
    ///
    /// At the support edges the ownership comparison and the window-membership
    /// comparison are the same branch of the evaluator — a break crossing
    /// `k_lo` either leaves the window or is placed below the first kernel
    /// piece, and the interval between the two roundings is narrower than one
    /// ulp, so it carries no integral. Both roundings therefore collapse to
    /// the later one, the first time every comparison agrees on the far side;
    /// emitting both would seed the refit with an ulp-wide span whose knots
    /// make de Boor divide by the span width.
    pub fn output_cut_transitions(
        kernel: &PiecewisePolynomialKernel,
        input_break: f64,
        kernel_break: f64,
        transitions: &mut Vec<f64>,
    ) {
        let (k_lo, k_hi) = kernel.support();
        let owned = first_time_satisfying(|t| t - input_break >= kernel_break);
        let window_edge = if kernel_break == k_lo {
            Some(first_time_satisfying(|t| input_break < t - k_lo))
        } else if kernel_break == k_hi {
            Some(first_time_satisfying(|t| input_break <= t - k_hi))
        } else {
            None
        };
        transitions.push(match window_edge {
            Some(edge) => owned.max(edge),
            None => owned,
        });
    }
}

impl<'a, F> ShapedSignal<'a, F>
where
    F: Fn(f64) -> f64,
{
    pub fn new_from_evaluator(
        kernel: &'a PiecewisePolynomialKernel,
        eval: F,
        input_breaks: Vec<f64>,
        input_degree: usize,
    ) -> Self {
        Self::new_with_moments(kernel, eval, input_breaks, input_degree, None)
    }

    pub fn new_from_polynomial_evaluator<M>(
        kernel: &'a PiecewisePolynomialKernel,
        eval: F,
        input_breaks: Vec<f64>,
        input_degree: usize,
        moments: M,
    ) -> Self
    where
        M: Fn(f64, f64, usize, f64, &mut [f64]) -> bool + 'a,
    {
        Self::new_with_moments(
            kernel,
            eval,
            input_breaks,
            input_degree,
            Some(Box::new(moments)),
        )
    }

    fn new_with_moments(
        kernel: &'a PiecewisePolynomialKernel,
        eval: F,
        mut input_breaks: Vec<f64>,
        input_degree: usize,
        moment_input: Option<Box<MomentEvaluator<'a>>>,
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
        input_breaks.sort_by(f64::total_cmp);
        input_breaks.dedup();
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
            moment_input,
            gauss_nodes,
            gauss_weights,
            input_breaks,
            cuts: std::cell::RefCell::new(Vec::new()),
            pva_memo: std::cell::RefCell::new([None; PVA_MEMO_SLOTS]),
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
        let value = self
            .convolve_pva_from_moments(t)
            .unwrap_or_else(|| self.convolve_pva_quadrature(t));
        let slot = self.pva_memo_next.get();
        self.pva_memo.borrow_mut()[slot] = Some((key, value));
        self.pva_memo_next.set((slot + 1) % PVA_MEMO_SLOTS);
        value
    }

    fn convolve_pva_from_moments(&self, t: f64) -> Option<(f64, f64, f64)> {
        let moment_input = self.moment_input.as_ref()?;
        let (mut p, mut v, mut a) = (0.0, 0.0, 0.0);
        for ((kernel, d_kernel), dd_kernel) in self
            .kernel
            .pieces
            .iter()
            .zip(&self.d_kernel.pieces)
            .zip(&self.dd_kernel.pieces)
        {
            let degree = kernel.degree();
            let mut moments = [0.0; MAX_EXACT_PRODUCT_DEGREE + 1];
            if !moment_input(
                t - kernel.u_end,
                t - kernel.u_start,
                degree,
                t,
                &mut moments[..=degree],
            ) {
                return None;
            }
            p += self.integrate_kernel_piece(kernel, &moments);
            v += self.integrate_kernel_piece(d_kernel, &moments);
            a += self.integrate_kernel_piece(dd_kernel, &moments);
        }
        for &(tau, jump) in &self.d_kernel_jumps {
            a += jump * (self.eval_input)(t - tau);
        }
        Some((p, v, a))
    }

    fn convolve_pva_quadrature(&self, t: f64) -> (f64, f64, f64) {
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
        (p, v, a)
    }

    fn integrate_kernel_piece(&self, kernel: &nurbs::bezier::BezierPiece, moments: &[f64]) -> f64 {
        let shifted_t = -kernel.u_start;
        let mut value = 0.0;
        for (power, coefficient) in kernel.coeffs.iter().copied().enumerate() {
            let mut choose = 1.0;
            let mut expanded = 0.0;
            for (moment_power, moment) in moments.iter().copied().enumerate().take(power + 1) {
                let sign = if moment_power % 2 == 0 { 1.0 } else { -1.0 };
                expanded += sign * choose * shifted_t.powi((power - moment_power) as i32) * moment;
                if moment_power < power {
                    choose *= (power - moment_power) as f64 / (moment_power + 1) as f64;
                }
            }
            value += coefficient * expanded;
        }
        value
    }

    /// Merge the kernel-piece boundaries (ascending by construction) with the
    /// in-window input breaks (`t - b` is ascending over `input_breaks`
    /// iterated in reverse), deduplicating on the fly — no per-call sort.
    fn merge_cuts(&self, t: f64, cuts: &mut Vec<f64>) {
        cuts.clear();
        let b_lo = self.input_breaks.partition_point(|&b| b <= t - self.k_hi);
        let b_hi = self.input_breaks.partition_point(|&b| b < t - self.k_lo);
        let mut breaks = self.input_breaks[b_lo..b_hi].iter().rev().peekable();
        let push = |v: f64, cuts: &mut Vec<f64>| {
            if cuts.last().is_none_or(|&last| v > last) {
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

    fn convolve_from_moments(&self, t: f64, kernel: &PiecewisePolynomialKernel) -> Option<f64> {
        let moment_input = self.moment_input.as_ref()?;
        let mut value = 0.0;
        for piece in &kernel.pieces {
            let degree = piece.degree();
            let mut moments = [0.0; MAX_EXACT_PRODUCT_DEGREE + 1];
            if !moment_input(
                t - piece.u_end,
                t - piece.u_start,
                degree,
                t,
                &mut moments[..=degree],
            ) {
                return None;
            }
            value += self.integrate_kernel_piece(piece, &moments);
        }
        Some(value)
    }
    fn convolve(&self, t: f64, kernel: &PiecewisePolynomialKernel) -> f64 {
        if let Some(value) = self.convolve_from_moments(t, kernel) {
            return value;
        }
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
