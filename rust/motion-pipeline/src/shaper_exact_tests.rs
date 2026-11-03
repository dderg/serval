use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use trajectory::{build_smooth_mzv_kernel, build_smooth_zv_kernel, next_toward};

use super::{
    CentredSum, EXACT_CAPACITY, ExactPiece, ExactRounding, ExactSpan, FIRST_ORDER_SHIFT_LIMIT,
    RealSum, Tracked,
};
use crate::shaper::{
    AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, SignalBounds, TrackSignal,
    piece_midpoint, taylor_shift,
};

/// A value carried as an unevaluated sum of two floats, about 106 bits.
#[derive(Clone, Copy, Debug)]
struct Wide {
    head: f64,
    tail: f64,
}

impl Wide {
    const ZERO: Self = Self {
        head: 0.0,
        tail: 0.0,
    };

    fn of(value: f64) -> Self {
        Self {
            head: value,
            tail: 0.0,
        }
    }

    fn normalized(head: f64, tail: f64) -> Self {
        let sum = RealSum::of(head, tail);
        Self {
            head: sum.head,
            tail: sum.tail,
        }
    }

    fn add(self, other: Self) -> Self {
        let sum = RealSum::of(self.head, other.head);
        Self::normalized(sum.head, sum.tail + self.tail + other.tail)
    }

    fn neg(self) -> Self {
        Self {
            head: -self.head,
            tail: -self.tail,
        }
    }

    fn sub(self, other: Self) -> Self {
        self.add(other.neg())
    }

    fn mul(self, other: Self) -> Self {
        let product = self.head * other.head;
        let error = self.head.mul_add(other.head, -product);
        Self::normalized(
            product,
            error + self.head * other.tail + self.tail * other.head,
        )
    }

    fn scale(self, factor: f64) -> Self {
        self.mul(Self::of(factor))
    }

    fn divided_by(self, divisor: f64) -> Self {
        let quotient = self.head / divisor;
        let remainder = self.sub(Self::of(quotient).scale(divisor));
        Self::normalized(quotient, remainder.head / divisor)
    }

    fn value(self) -> f64 {
        self.head + self.tail
    }
}

fn wide_poly_eval(coefficients: &[Wide], at: Wide) -> Wide {
    coefficients
        .iter()
        .rev()
        .fold(Wide::ZERO, |acc, &c| acc.mul(at).add(c))
}

fn wide_derivative(coefficients: &[Wide], order: usize) -> Vec<Wide> {
    let mut current = coefficients.to_vec();
    for _ in 0..order {
        if current.len() <= 1 {
            return Vec::new();
        }
        current = (1..current.len())
            .map(|power| current[power].scale(power as f64))
            .collect();
    }
    current
}

fn wide_taylor_shift(coefficients: &[Wide], shift: Wide) -> Vec<Wide> {
    let mut shifted = coefficients.to_vec();
    let len = shifted.len();
    for lowest in 0..len {
        for power in (lowest + 1..len).rev() {
            shifted[power - 1] = shifted[power - 1].add(shifted[power].mul(shift));
        }
    }
    shifted
}

/// `∫_a^b p(v) dv`.
fn wide_integral(coefficients: &[Wide], a: Wide, b: Wide) -> Wide {
    let antiderivative: Vec<Wide> = std::iter::once(Wide::ZERO)
        .chain(
            coefficients
                .iter()
                .enumerate()
                .map(|(power, c)| c.divided_by((power + 1) as f64)),
        )
        .collect();
    wide_poly_eval(&antiderivative, b).sub(wide_poly_eval(&antiderivative, a))
}

fn binomial(n: usize, k: usize) -> f64 {
    let numerator: u64 = ((n - k + 1)..=n).map(|i| i as u64).product();
    let denominator: u64 = (1..=k).map(|i| i as u64).product();
    (numerator / denominator) as f64
}

/// The shaped signal's `order`-th derivative at `t`, integrated in
/// double-double straight from the table's pieces, holds, jumps and
/// impulses and the kernel's pieces: an independent reference for the
/// exact polynomials.
fn reference(source: &ShapedSignalSource, t: f64, order: usize) -> f64 {
    let table = &source.table;
    let t_wide = Wide::of(t);
    let mut regions: Vec<(Wide, Wide, Vec<Wide>)> = Vec::new();
    let far = 1e3;
    regions.push((
        Wide::of(table.first_t - far),
        Wide::of(table.first_t),
        vec![Wide::of(table.held_before())],
    ));
    for piece in 0..table.coeffs.len() {
        let start = table.starts[piece].max(table.first_t);
        let end = table.ends[piece].min(table.last_t);
        if end <= start {
            continue;
        }
        regions.push((
            Wide::of(start),
            Wide::of(end),
            table.coeffs[piece].iter().map(|&c| Wide::of(c)).collect(),
        ));
    }
    regions.push((
        Wide::of(table.last_t),
        Wide::of(table.last_t + far),
        vec![Wide::of(table.held_after())],
    ));
    let piece_origin = |region: usize| -> Wide {
        if region == 0 || region == regions.len() - 1 {
            regions[region].0
        } else {
            let start = regions[region].0.head;
            let piece = table
                .starts
                .iter()
                .rposition(|&s| s <= start)
                .expect("a region starts inside a piece");
            Wide::of(table.origins[piece])
        }
    };
    let mut total = Wide::of(if order == 0 {
        source.shaped_offset
    } else {
        0.0
    });
    for kernel_piece in &source.kernel.pieces {
        let kernel: Vec<Wide> = kernel_piece.coeffs.iter().map(|&c| Wide::of(c)).collect();
        let a = Wide::of(kernel_piece.u_start);
        let b = Wide::of(kernel_piece.u_end);
        let window_lo = t_wide.sub(b);
        let window_hi = t_wide.sub(a);
        for (region, (start, end, coefficients)) in regions.iter().enumerate() {
            let lo = if start.sub(window_lo).value() > 0.0 {
                *start
            } else {
                window_lo
            };
            let hi = if end.sub(window_hi).value() < 0.0 {
                *end
            } else {
                window_hi
            };
            if hi.sub(lo).value() <= 0.0 {
                continue;
            }
            let input = wide_taylor_shift(
                &wide_derivative(coefficients, order),
                lo.sub(piece_origin(region)),
            );
            if input.is_empty() {
                continue;
            }
            let origin = lo;
            let c = t_wide.sub(origin).sub(a);
            let mut kernel_in_v = vec![Wide::ZERO; kernel.len()];
            for (l, &kl) in kernel.iter().enumerate() {
                let mut c_power = Wide::of(1.0);
                for m in (0..=l).rev() {
                    let sign = if m % 2 == 0 { 1.0 } else { -1.0 };
                    kernel_in_v[m] =
                        kernel_in_v[m].add(kl.mul(c_power).scale(sign * binomial(l, m)));
                    c_power = c_power.mul(c);
                }
            }
            let mut product = vec![Wide::ZERO; kernel.len() + input.len() - 1];
            for (i, &ki) in kernel_in_v.iter().enumerate() {
                for (j, &xj) in input.iter().enumerate() {
                    product[i + j] = product[i + j].add(ki.mul(xj));
                }
            }
            total = total.add(wide_integral(&product, Wide::ZERO, hi.sub(origin)));
        }
        let masses = table
            .input_jumps()
            .iter()
            .map(|&(at, position, slope)| {
                (
                    at,
                    [
                        [0.0; 4],
                        [position, 0.0, 0.0, 0.0],
                        [slope, position, 0.0, 0.0],
                    ],
                )
            })
            .chain(table.gain_impulses().iter().map(|impulse| {
                (
                    impulse.at,
                    [
                        [impulse.delta, impulse.doublet, 0.0, 0.0],
                        [0.0, impulse.delta, impulse.doublet, 0.0],
                        [0.0, 0.0, impulse.delta, impulse.doublet],
                    ],
                )
            }));
        for (at, weights) in masses {
            let tau = t_wide.sub(Wide::of(at));
            if tau.sub(a).value() <= 0.0 || tau.sub(b).value() >= 0.0 {
                continue;
            }
            for (derivative, &weight) in weights[order].iter().enumerate() {
                let kernel_derivative = wide_derivative(&kernel, derivative);
                if weight == 0.0 || kernel_derivative.is_empty() {
                    continue;
                }
                total = total.add(wide_poly_eval(&kernel_derivative, tau.sub(a)).scale(weight));
            }
        }
    }
    total.value()
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn between(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }
}

/// A random piecewise-polynomial input over `[start, end]` with jumps in
/// value and slope at every seam, degrees up to nine, and a few seams an
/// ulp apart.
fn random_table(rng: &mut Rng, start: f64, end: f64, gains: Option<(f64, f64)>) -> AxisSignalTable {
    let mut table = AxisSignalTable {
        starts: Vec::new(),
        ends: Vec::new(),
        origins: Vec::new(),
        coeffs: Vec::new(),
        first_t: start,
        last_t: end,
        boundary: FitBoundary::StreamBoundary,
        lookahead: Lookahead::Forced,
        moment_tree: None,
        input_jumps: Vec::new(),
        entering_step: 0.0,
        leaving_step: 0.0,
        gain_impulses: Vec::new(),
        boundaries: Vec::new(),
    };
    let mut at = start;
    let mut value = rng.between(-50.0, 50.0);
    while at < end {
        let width = if rng.next() < 0.08 {
            4.0 * (next_toward(at, f64::INFINITY) - at)
        } else {
            rng.between(2e-4, 3e-3)
        };
        let piece_end = (at + width).min(end);
        let degree = (rng.next() * 10.0) as usize;
        let mut piece = vec![value + rng.between(-0.01, 0.01)];
        let mut scale = 300.0;
        for _ in 0..degree {
            piece.push(rng.between(-scale, scale));
            scale *= 300.0;
        }
        let tau = piece_end - at;
        value = piece.iter().rev().fold(0.0, |acc, &c| acc * tau + c);
        let mut centered = vec![0.0; piece.len()];
        taylor_shift(&piece, piece_midpoint(at, piece_end) - at, &mut centered);
        table.push_piece(at, piece_end, centered);
        at = piece_end;
    }
    match gains {
        Some((k1, k2)) => table.with_derivative_gains(k1, k2),
        None => table,
    }
}

/// The production bell kernel, `c (h² − t²)²` over `[−h, h]`.
fn bell_kernel(smooth_time: f64) -> PiecewisePolynomialKernel {
    let h = smooth_time / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c],
        (-h, h),
    )
}

/// The production triangle kernel: two linear pieces whose slope jumps at
/// their seam and both edges.
fn triangle_kernel(smooth_time: f64) -> PiecewisePolynomialKernel {
    let half = smooth_time / 2.0;
    let curvature = 1.0 / (half * half);
    PiecewisePolynomialKernel::from_pieces(vec![
        BezierPiece {
            u_start: -half,
            u_end: 0.0,
            coeffs: vec![0.0, curvature],
        },
        BezierPiece {
            u_start: 0.0,
            u_end: half,
            coeffs: vec![1.0 / half, -curvature],
        },
    ])
    .expect("triangle pieces tile")
}

fn kernels() -> Vec<(&'static str, PiecewisePolynomialKernel)> {
    vec![
        ("smooth_zv", build_smooth_zv_kernel(80.0)),
        ("smooth_mzv", build_smooth_mzv_kernel(120.0)),
        ("bell", bell_kernel(0.006)),
        ("triangle", triangle_kernel(0.008)),
    ]
}

const SIGNAL_START_S: f64 = 3.0;
const SIGNAL_END_S: f64 = 3.06;

fn assert_piece_matches_reference(
    source: &ShapedSignalSource,
    piece: &ExactPiece,
    t: f64,
    worst: &mut [f64; 3],
) {
    let exact = piece.eval_pva(t);
    let exact = [exact.0, exact.1, exact.2];
    for order in 0..3 {
        let truth = reference(source, t, order);
        let miss = (exact[order] - truth).abs();
        let bound = piece.pva[order].rounding;
        assert!(
            miss <= bound,
            "order {order} at {t} (piece [{}, {}]): exact {} reference {truth} misses by {miss:e}, \
             past its rounding bound {bound:e}",
            piece.lo,
            piece.hi,
            exact[order]
        );
        worst[order] = worst[order].max(miss / bound);
    }
}

/// The exact polynomial of every sub-interval agrees with a double-double
/// convolution of the same table and kernel to within its own a-priori
/// rounding term, at random times, at both ends of every piece, and an ulp
/// either side of every break, across kernels with one and two pieces,
/// with and without derivative-gain impulses, through clamped history and
/// lookahead. Late in a long print an edge's rounding grows past the
/// first-order shift's limit against the narrower chunks, so both shifts
/// are held to the reference.
#[test]
fn exact_polynomials_match_a_double_double_convolution_within_their_rounding_bound() {
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    let mut worst = [0.0_f64; 3];
    for start in [SIGNAL_START_S, 3000.0, 86400.0] {
        let end = start + (SIGNAL_END_S - SIGNAL_START_S);
        for (name, kernel) in kernels() {
            for gains in [None, Some((0.02, 1.5e-4))] {
                let table = random_table(&mut rng, start, end, gains);
                let breaks = table.starts.clone();
                let source = ShapedSignalSource::new(&kernel, table, breaks, start + 0.01);
                assert_windows_match_reference(&mut rng, name, &source, (start, end), &mut worst);
            }
        }
    }
    eprintln!("worst miss over rounding bound: {worst:?}");
}

fn assert_windows_match_reference(
    rng: &mut Rng,
    name: &str,
    source: &ShapedSignalSource,
    (start, end): (f64, f64),
    worst: &mut [f64; 3],
) {
    let signal = source.signal();
    for window in 0..16 {
        let lo = if window % 2 == 0 {
            rng.between(start - 0.01, end)
        } else {
            let boundaries = source.table.boundaries();
            let cuts = &source.exact_kernel.cuts;
            let boundary = boundaries[(rng.next() * boundaries.len() as f64) as usize];
            boundary + cuts[(rng.next() * cuts.len() as f64) as usize]
        };
        let hi = (lo + rng.between(1e-5, 8e-3)).min(end + 0.01);
        let pieces = signal
            .exact_polynomial_on(lo, hi)
            .unwrap_or_else(|| panic!("{name}: a clamped table has every window"));
        assert!(!pieces.is_empty(), "{name}: [{lo}, {hi}] has no pieces");
        assert_eq!(
            pieces[0].lo, lo,
            "{name}: the first piece starts at the interval"
        );
        assert_eq!(
            pieces.last().expect("non-empty").hi,
            hi,
            "{name}: the last piece ends at the interval"
        );
        for pair in pieces.windows(2) {
            assert!(
                pair[1].lo <= next_toward(pair[0].hi, f64::INFINITY),
                "{name}: pieces [{}, {}] and [{}, {}] leave a gap",
                pair[0].lo,
                pair[0].hi,
                pair[1].lo,
                pair[1].hi
            );
        }
        for piece in &pieces {
            let first_inside = next_toward(piece.lo, f64::INFINITY);
            let last_inside = next_toward(piece.hi, f64::NEG_INFINITY);
            if first_inside > last_inside {
                continue;
            }
            let mut times = vec![first_inside, last_inside];
            times.extend((0..4).map(|_| rng.between(first_inside, last_inside)));
            for t in times {
                assert_piece_matches_reference(source, piece, t, worst);
            }
        }
    }
}

const EVALUATOR_BREAK_ULPS: u32 = 16;

/// A piece's polynomial is the evaluator's signal: `eval_pva` lands on it
/// to within the bound plus the evaluator's own rounding. Within a few ulps
/// of a break the evaluator decides which side it reads by float
/// comparisons, which can disagree with the real order of the break, so
/// only times clear of the ends are compared.
#[test]
fn exact_polynomials_agree_with_the_evaluator() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for (name, kernel) in kernels() {
        let table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, Some((0.01, 0.0)));
        let breaks = table.starts.clone();
        let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.02);
        let signal = source.signal();
        let (lo, hi) = (SIGNAL_START_S + 0.02, SIGNAL_START_S + 0.03);
        for piece in signal.exact_polynomial_on(lo, hi).expect("clamped") {
            let clear = |from: f64, toward: f64| {
                (0..EVALUATOR_BREAK_ULPS).fold(from, |t, _| next_toward(t, toward))
            };
            let first_inside = clear(piece.lo, f64::INFINITY);
            let last_inside = clear(piece.hi, f64::NEG_INFINITY);
            if first_inside > last_inside {
                continue;
            }
            for _ in 0..8 {
                let t = rng.between(first_inside, last_inside);
                let exact = piece.eval_pva(t);
                let evaluated = signal.eval_pva(t);
                for (order, (e, v)) in [
                    (exact.0, evaluated.0),
                    (exact.1, evaluated.1),
                    (exact.2, evaluated.2),
                ]
                .into_iter()
                .enumerate()
                {
                    let slack = piece.pva[order].rounding + 1e-9 * (1.0 + v.abs());
                    assert!(
                        (e - v).abs() <= slack,
                        "{name} order {order} at {t} (piece [{}, {}]): exact {e}, evaluated {v}, \
                         reference {}",
                        piece.lo,
                        piece.hi,
                        reference(&source, t, order)
                    );
                }
            }
        }
    }
}

/// Every break of the signal inside the interval is a piece boundary: no
/// image of an input boundary under a kernel cut lies strictly inside a
/// piece, an ulp in from its ends.
#[test]
fn exact_pieces_split_at_every_break() {
    let mut rng = Rng(0xD1B5_4A32_D192_ED03);
    let kernel = triangle_kernel(0.008);
    let table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, None);
    let breaks = table.starts.clone();
    let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.01);
    let (lo, hi) = (SIGNAL_START_S + 0.012, SIGNAL_START_S + 0.04);
    let pieces = source
        .signal()
        .exact_polynomial_on(lo, hi)
        .expect("clamped");
    for &boundary in source.table.boundaries() {
        for &cut in &source.exact_kernel.cuts {
            let image = RealSum::of(boundary, cut);
            for piece in &pieces {
                let inside_from = next_toward(piece.lo, f64::INFINITY);
                let inside_to = next_toward(piece.hi, f64::NEG_INFINITY);
                assert!(
                    !(image.order_of(inside_from) == std::cmp::Ordering::Less
                        && image.order_of(inside_to) == std::cmp::Ordering::Greater),
                    "the break {boundary} + {cut} lies inside the piece [{}, {}]",
                    piece.lo,
                    piece.hi
                );
            }
        }
    }
}

/// The degree-13 piece the source fit laid over 124 ms of the dense print's
/// Y axis under a bowden profile at `corner_deviation: 0.04`, as its track
/// stores it about its start: its values stay within 3.5 mm while those
/// monomial terms swing past 10⁵.
const DENSE_PRINT_DEGREE_13_PIECE: [f64; 14] = [
    -1.498277337561234,
    -80.34290769901087,
    133.29081300507906,
    29093.238628373114,
    -1376200.889930974,
    86358173.49058539,
    -3325204560.854538,
    80140902425.37823,
    -1273500722382.6865,
    13562912171021.732,
    -95908575980999.06,
    432506081340011.9,
    -1126976833794649.5,
    1292327102349403.8,
];
const DENSE_PRINT_PIECE_S: (f64, f64) = (1.7465212887583386, 1.8710579523872128);
const SHAPER_POSITION_TOLERANCE_MM: f64 = 0.005;

#[test]
fn a_high_degree_input_piece_keeps_its_exact_polynomials_rounding_far_inside_the_fit_tolerance() {
    let (start, end) = DENSE_PRINT_PIECE_S;
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: start,
        u_end: end,
        coeffs: DENSE_PRINT_DEGREE_13_PIECE.to_vec(),
    }]);
    let table = AxisSignalTable::from_tracks(
        std::iter::once(&track),
        SignalBounds {
            first_t: start,
            last_t: end,
            boundary: FitBoundary::StreamBoundary,
            lookahead: Lookahead::Forced,
        },
    );
    let breaks = table.starts.clone();
    let kernel = build_smooth_mzv_kernel(36.0);
    let source = ShapedSignalSource::new(&kernel, table, breaks, start);
    let pieces = source
        .signal()
        .exact_polynomial_on(start, end)
        .expect("clamped");
    let worst = pieces
        .iter()
        .map(|piece| piece.pva[0].rounding)
        .fold(0.0, f64::max);
    assert!(
        worst <= 0.01 * SHAPER_POSITION_TOLERANCE_MM,
        "an exact polynomial over the degree-13 piece bounds its rounding at {worst:e} mm"
    );
}

/// Where a kernel window reaches history an interior window does not hold,
/// the signal has no exact polynomial, as the evaluator has no value.
#[test]
fn a_window_reaching_unheld_history_has_no_exact_polynomial() {
    let mut rng = Rng(0x0123_4567_89AB_CDEF);
    let kernel = build_smooth_zv_kernel(80.0);
    let mut table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, None);
    table.boundary = FitBoundary::Interior;
    let breaks = table.starts.clone();
    let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.02);
    let signal = source.signal();
    assert!(
        signal
            .exact_polynomial_on(SIGNAL_START_S, SIGNAL_START_S + 0.004)
            .is_none()
    );
    assert!(!signal.eval_pva(SIGNAL_START_S + 0.002).0.is_finite());
    assert!(
        signal
            .exact_polynomial_on(SIGNAL_START_S + 0.02, SIGNAL_START_S + 0.03)
            .is_some()
    );
}

/// Two spans meeting inside a sub-interval build its polynomials on the
/// same chunks about the same centres, so they read one value, bit for
/// bit, at the seam they share.
#[test]
fn spans_meeting_between_breaks_read_one_state_at_their_seam() {
    let mut rng = Rng(0x5851_F42D_4C95_7F2D);
    for (name, kernel) in kernels() {
        let table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, Some((0.01, 0.0)));
        let breaks = table.starts.clone();
        let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.02);
        let signal = source.signal();
        for _ in 0..32 {
            let seam = rng.between(SIGNAL_START_S + 0.03, SIGNAL_END_S - 0.03);
            let (lo, hi) = (
                seam - rng.between(1e-4, 8e-3),
                seam + rng.between(1e-4, 8e-3),
            );
            let before = ExactSpan::new(signal.exact_polynomial_on(lo, seam).expect("clamped"));
            let after = ExactSpan::new(signal.exact_polynomial_on(seam, hi).expect("clamped"));
            let (left, right) = (before.pva(seam, lo), after.pva(seam, hi));
            assert!(
                left.0.to_bits() == right.0.to_bits()
                    || source.table.boundaries().iter().any(|&boundary| {
                        source
                            .exact_kernel
                            .cuts
                            .iter()
                            .any(|&cut| RealSum::of(boundary, cut).order_of(seam).is_eq())
                    }),
                "{name}: spans meeting at {seam} read {left:?} and {right:?}"
            );
        }
    }
}

/// A read takes the polynomial of the sub-interval holding its time, and
/// at a time two sub-intervals share, the one it faces.
#[test]
fn exact_reads_take_the_sub_interval_they_face() {
    let mut rng = Rng(0x2127_599B_F432_5C37);
    let kernel = triangle_kernel(0.008);
    let table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, None);
    let breaks = table.starts.clone();
    let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.01);
    let (lo, hi) = (SIGNAL_START_S + 0.012, SIGNAL_START_S + 0.04);
    let pieces = source
        .signal()
        .exact_polynomial_on(lo, hi)
        .expect("clamped");
    let span = ExactSpan::new(pieces.clone());
    let bits = |pva: (f64, f64, f64)| [pva.0.to_bits(), pva.1.to_bits(), pva.2.to_bits()];
    for pair in pieces.windows(2) {
        let [before, after] = pair else {
            unreachable!("windows of two")
        };
        if before.hi == after.lo {
            let shared = before.hi;
            assert_eq!(bits(span.pva(shared, lo)), bits(before.eval_pva(shared)));
            assert_eq!(bits(span.pva(shared, hi)), bits(after.eval_pva(shared)));
        } else {
            for toward in [lo, hi] {
                assert_eq!(
                    bits(span.pva(after.lo, toward)),
                    bits(before.eval_pva(after.lo))
                );
                assert_eq!(
                    bits(span.pva(before.hi, toward)),
                    bits(after.eval_pva(before.hi))
                );
            }
        }
    }
}

/// A random polynomial of `len` coefficients whose terms are of order one
/// on `|w| ≤ radius`, all positive when `positive`.
fn scaled_polynomial(rng: &mut Rng, len: usize, radius: f64, positive: bool) -> Vec<f64> {
    let mut scale = 1.0;
    (0..len)
        .map(|_| {
            let value = if positive {
                rng.between(0.5, 1.0)
            } else {
                rng.between(-1.0, 1.0)
            };
            let coefficient = value / scale;
            scale *= radius;
            coefficient
        })
        .collect()
}

fn wide_shifted_value(coefficients: &[f64], shift: f64, w: f64) -> Wide {
    let wide: Vec<Wide> = coefficients.iter().map(|&c| Wide::of(c)).collect();
    wide_poly_eval(&wide, Wide::of(w).add(Wide::of(shift)))
}

const SHIFT_SAMPLES: usize = 9;

fn shift_samples(rng: &mut Rng, radius: f64) -> Vec<f64> {
    let mut samples = vec![-radius, radius, 0.0];
    samples.extend((3..SHIFT_SAMPLES).map(|_| rng.between(-radius, radius)));
    samples
}

/// The first-order shift drops `Σⱼ pⱼ[(w+ε)ʲ − wʲ − jεwʲ⁻¹]`, which the
/// Taylor remainder bounds by `Σⱼ |pⱼ|·C(j,2)·ε²·(r+|ε|)^{j−2}` on
/// `|w| ≤ r`. In double-double the dropped part stays within the
/// truncation `add_shifted` reports, and for positive coefficients at
/// `w = r` it reaches at least half of it, the bound's own factor two.
#[test]
fn a_first_order_shift_drops_no_more_than_its_truncation_bound() {
    let mut rng = Rng(0x6A09_E667_F3BC_C909);
    for len in 3..=EXACT_CAPACITY {
        for ratio in [
            2f64.powi(-50),
            2f64.powi(-40),
            2f64.powi(-31),
            FIRST_ORDER_SHIFT_LIMIT,
        ] {
            for positive in [false, true] {
                let radius = libm::pow(10.0, rng.between(-7.0, -2.0));
                let sign = if positive || rng.next() < 0.5 {
                    1.0
                } else {
                    -1.0
                };
                let shift = sign * ratio * radius;
                let coefficients = scaled_polynomial(&mut rng, len, radius, positive);
                let mut sum = CentredSum::ZERO;
                sum.add_shifted(&Tracked::exact(&coefficients), shift, 1.0, radius);
                assert!(
                    sum.truncation > 0.0,
                    "a first-order shift reports no truncation"
                );
                let first_order: Vec<Wide> = (0..len)
                    .map(|power| {
                        let slope = coefficients.get(power + 1).map_or(Wide::ZERO, |&next| {
                            Wide::of(next).scale(shift).scale((power + 1) as f64)
                        });
                        Wide::of(coefficients[power]).add(slope)
                    })
                    .collect();
                for w in shift_samples(&mut rng, radius) {
                    let dropped = wide_shifted_value(&coefficients, shift, w)
                        .sub(wide_poly_eval(&first_order, Wide::of(w)))
                        .value()
                        .abs();
                    assert!(
                        dropped <= sum.truncation,
                        "len {len} ratio {ratio:e} at {w}: dropped {dropped:e} past the \
                         truncation bound {:e}",
                        sum.truncation
                    );
                    if positive && w == radius {
                        assert!(
                            dropped >= 0.49 * sum.truncation,
                            "len {len} ratio {ratio:e}: dropped {dropped:e} is far inside the \
                             truncation bound {:e}",
                            sum.truncation
                        );
                    }
                }
            }
        }
    }
}

/// Either side of the first-order limit, the shifted coefficients land on
/// the double-double shift to within their rounding and truncation, and
/// past the limit the full shift reports no truncation.
#[test]
fn a_shift_by_an_edge_rounding_matches_the_double_double_shift_within_its_bound() {
    let mut rng = Rng(0xBB67_AE85_84CA_A73B);
    let gamma = ExactRounding::gamma(ExactRounding::SHIFT + 2 * EXACT_CAPACITY);
    for len in 1..=EXACT_CAPACITY {
        for ratio in [
            2f64.powi(-40),
            FIRST_ORDER_SHIFT_LIMIT,
            2.0 * FIRST_ORDER_SHIFT_LIMIT,
            2f64.powi(-10),
            0.5,
        ] {
            let radius = libm::pow(10.0, rng.between(-7.0, -2.0));
            let sign = if rng.next() < 0.5 { 1.0 } else { -1.0 };
            let shift = sign * ratio * radius;
            let coefficients = scaled_polynomial(&mut rng, len, radius, false);
            let mut sum = CentredSum::ZERO;
            sum.add_shifted(&Tracked::exact(&coefficients), shift, -1.0, radius);
            if ratio > FIRST_ORDER_SHIFT_LIMIT {
                assert_eq!(sum.truncation, 0.0, "a full shift reports a truncation");
            }
            let bound = 2.0 * gamma * sum.magnitude + sum.truncation;
            for w in shift_samples(&mut rng, radius) {
                let computed = sum.values[..sum.len]
                    .iter()
                    .rev()
                    .fold(0.0, |acc, &c| nurbs::fmadd(acc, w, c));
                let truth = wide_shifted_value(&coefficients, shift, w).neg().value();
                let miss = (computed - truth).abs();
                assert!(
                    miss <= bound,
                    "len {len} ratio {ratio:e} at {w}: misses by {miss:e}, past {bound:e}"
                );
            }
        }
    }
}
