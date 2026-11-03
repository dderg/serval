use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use trajectory::{build_smooth_mzv_kernel, build_smooth_zv_kernel, next_toward};

use super::{ExactPiece, RealSum};
use crate::shaper::{AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, TrackSignal};

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
            Wide::of(table.starts[piece])
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
    let (mut starts, mut ends, mut coeffs) = (Vec::new(), Vec::new(), Vec::new());
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
        starts.push(at);
        ends.push(piece_end);
        coeffs.push(piece);
        at = piece_end;
    }
    let table = AxisSignalTable {
        starts,
        ends,
        coeffs,
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
/// lookahead.
#[test]
fn exact_polynomials_match_a_double_double_convolution_within_their_rounding_bound() {
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    let mut worst = [0.0_f64; 3];
    for (name, kernel) in kernels() {
        for gains in [None, Some((0.02, 1.5e-4))] {
            let table = random_table(&mut rng, SIGNAL_START_S, SIGNAL_END_S, gains);
            let breaks = table.starts.clone();
            let source = ShapedSignalSource::new(&kernel, table, breaks, SIGNAL_START_S + 0.01);
            let signal = source.signal();
            for window in 0..16 {
                let lo = if window % 2 == 0 {
                    rng.between(SIGNAL_START_S - 0.01, SIGNAL_END_S)
                } else {
                    let boundaries = source.table.boundaries();
                    let cuts = &source.exact_kernel.cuts;
                    let boundary = boundaries[(rng.next() * boundaries.len() as f64) as usize];
                    boundary + cuts[(rng.next() * cuts.len() as f64) as usize]
                };
                let hi = (lo + rng.between(1e-5, 8e-3)).min(SIGNAL_END_S + 0.01);
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
                        assert_piece_matches_reference(&source, piece, t, &mut worst);
                    }
                }
            }
        }
    }
    eprintln!("worst miss over rounding bound: {worst:?}");
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
