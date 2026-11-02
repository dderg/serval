use std::iter::once;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::build_smooth_zv_kernel;

use super::{
    HiddenBreak, HiddenBreakBound, JumpSeries, hidden_break_bounds, input_jumps_at, kernel_jumps_at,
};
use crate::lowering::FitTol;
use crate::shaper::{
    AxisFit, AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, SignalBounds,
    TrackSignal, fit_leader_target,
};

const FIT_TOL: FitTol = FitTol {
    pos_mm: 0.005,
    accel_mm_s2: 50.0,
};

fn table_over(track: &nurbs::ScalarNurbs, first_t: f64, last_t: f64) -> AxisSignalTable {
    AxisSignalTable::from_tracks(
        once(track),
        SignalBounds {
            first_t,
            last_t,
            boundary: FitBoundary::StreamBoundary,
            lookahead: Lookahead::Forced,
        },
    )
}

/// Rest until `BREAK_S`, then an acceleration step with a jerk on top.
fn rest_then_accelerate() -> nurbs::ScalarNurbs {
    bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: BREAK_S,
            coeffs: vec![0.0, 0.0, 0.0, 0.0],
        },
        BezierPiece {
            u_start: BREAK_S,
            u_end: 2.0 * BREAK_S,
            coeffs: vec![0.0, 0.0, 0.5 * 3_000.0, 2.0e5 / 6.0],
        },
    ])
}

const BREAK_S: f64 = 0.1;

fn second_derivative_at(series: &JumpSeries, reach: f64) -> f64 {
    let mut term = 1.0;
    let mut sum = 0.0;
    for power in 0..series.len.saturating_sub(2) {
        sum += series.orders[power + 2] * term;
        term *= reach / (power + 1) as f64;
    }
    sum
}

/// Before the first kernel image of an input break that ends a rest, the
/// shaped signal is the rest's; past it, until the next image, the shaped
/// acceleration is exactly the image's jump polynomial `D″`.
#[test]
fn past_an_image_the_shaped_acceleration_is_its_jump_polynomial() {
    let kernel = build_smooth_zv_kernel(44.0);
    let track = rest_then_accelerate();
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, 2.0 * BREAK_S),
        track.knots().to_vec(),
        0.0,
    );
    let signal = source.signal();
    let (k_lo, k_hi) = kernel.support();
    let image = input_jumps_at(&source.table, BREAK_S)
        .expect("the break is a seam of the table")
        .image_through(kernel_jumps_at(&kernel, k_lo));
    for reach in [1e-4, 1e-3, 4e-3, 0.5 * (k_hi - k_lo)] {
        let shaped = signal.eval_pva(BREAK_S + k_lo + reach).2;
        let predicted = second_derivative_at(&image, reach);
        assert!(
            (shaped - predicted).abs() <= 1e-6 * predicted.abs().max(1.0),
            "{reach} s past the image the shaped acceleration is {shaped}, its jump \
             polynomial {predicted}"
        );
        let bound = image.bound_over(reach);
        assert!(
            shaped.abs() <= bound.acceleration_mm_s2,
            "the bound {bound:?} misses the shaped acceleration {shaped} at reach {reach}"
        );
    }
}

/// An image whose bound fits the span's share of the tolerance is hidden,
/// and the span it lands in pays for it at its reach from the nearer edge.
#[test]
fn an_image_is_hidden_only_while_its_span_stays_within_its_share() {
    let kernel = build_smooth_zv_kernel(44.0);
    let track = rest_then_accelerate();
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, 2.0 * BREAK_S),
        track.knots().to_vec(),
        0.0,
    );
    let breaks = &source.shaped_breaks;
    let (k_lo, _) = kernel.support();
    let image_t = *breaks
        .times()
        .iter()
        .find(|&&t| (t - (BREAK_S + k_lo)).abs() < 1e-12)
        .expect("the break's first image is a shaped break");
    let close = breaks.select_seeds(
        vec![image_t - 1e-5, image_t, image_t + 1e-5],
        source.hidden_break_input(),
        FIT_TOL,
    );
    assert_eq!(close.kept, vec![image_t - 1e-5, image_t + 1e-5]);
    let [hidden] = close.hidden[..] else {
        panic!("one image is hidden: {:?}", close.hidden)
    };
    let [bound] = hidden_break_bounds(&close.kept, &close.hidden)[..] else {
        panic!("one span")
    };
    let reach = (image_t - close.kept[0]).min(close.kept[1] - image_t);
    assert_eq!(bound, hidden.jumps.bound_over(reach));
    let far = breaks.select_seeds(
        vec![image_t - 5e-2, image_t, image_t + 5e-2],
        source.hidden_break_input(),
        FIT_TOL,
    );
    assert_eq!(far.kept, vec![image_t - 5e-2, image_t, image_t + 5e-2]);
    assert!(far.hidden.is_empty());
}

#[test]
fn a_span_with_no_hidden_breaks_keeps_its_whole_tolerance() {
    let left = HiddenBreakBound::default().tolerance_left(FIT_TOL);
    assert_eq!(
        (left.pos_mm, left.accel_mm_s2),
        (FIT_TOL.pos_mm, FIT_TOL.accel_mm_s2)
    );
    let bound = HiddenBreakBound {
        position_mm: 1e-4,
        acceleration_mm_s2: 2.0,
    };
    let left = bound.tolerance_left(FIT_TOL);
    assert_eq!(
        (left.pos_mm, left.accel_mm_s2),
        (FIT_TOL.pos_mm - 2e-4, FIT_TOL.accel_mm_s2 - 4.0)
    );
}

#[test]
#[should_panic(expected = "leave no tolerance")]
fn hidden_breaks_that_would_spend_the_whole_tolerance_are_refused() {
    HiddenBreakBound {
        position_mm: 0.0,
        acceleration_mm_s2: 0.5 * FIT_TOL.accel_mm_s2,
    }
    .tolerance_left(FIT_TOL);
}

#[test]
#[should_panic(expected = "fall past the partition")]
fn a_hidden_break_outside_the_partition_is_refused() {
    let jumps = JumpSeries::derivatives_at(&[0.0, 1.0], 0.0);
    hidden_break_bounds(&[0.0, 1.0], &[HiddenBreak { t: 2.0, jumps }]);
}

const SINE_MM: f64 = 10.0;
const SINE_RAD_S: f64 = 2.0 * std::f64::consts::PI * 5.0;
const SINE_PIECE_S: f64 = 2e-3;
const SINE_PIECES: u32 = 100;

/// A 5 Hz sine stored as degree-7 Taylor pieces 2 ms long: every seam is a
/// break whose jumps are the Taylor remainder, far below any tolerance.
fn piecewise_sine() -> nurbs::ScalarNurbs {
    let pieces: Vec<BezierPiece> = (0..SINE_PIECES)
        .map(|index| {
            let start = f64::from(index) * SINE_PIECE_S;
            let mut factorial = 1.0;
            let coeffs = (0..8)
                .map(|order| {
                    if order > 0 {
                        factorial *= f64::from(order);
                    }
                    let phase = SINE_RAD_S * start + f64::from(order) * std::f64::consts::FRAC_PI_2;
                    SINE_MM * SINE_RAD_S.powi(order as i32) * libm::sin(phase) / factorial
                })
                .collect();
            BezierPiece {
                u_start: start,
                u_end: f64::from(index + 1) * SINE_PIECE_S,
                coeffs,
            }
        })
        .collect();
    bezier_pieces_to_nurbs(&pieces)
}

/// A leader fit over an input whose seams barely break it hides their
/// images, fits fewer pieces than it would seed, and every piece still holds
/// the tolerance against the shaped signal at dense interior points.
#[test]
fn a_leader_fit_hides_near_smooth_images_and_holds_the_tolerance() {
    let kernel = build_smooth_zv_kernel(44.0);
    let track = piecewise_sine();
    let last_t = f64::from(SINE_PIECES) * SINE_PIECE_S;
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, last_t),
        track.knots().to_vec(),
        0.0,
    );
    let signal = source.signal();
    let (t_start, t_end) = (0.05, 0.15);
    let fitted = fit_leader_target(
        AxisFit {
            axis: 0,
            t_start,
            t_end,
            seed_breakpoints: source.shaped_breaks.times(),
            fit_tol: FIT_TOL,
            fit_context: "hidden-break test",
            velocity_continuous: true,
        },
        &signal,
        &source.shaped_breaks,
        source.hidden_break_input(),
    )
    .expect("a near-smooth input fits");
    let pieces = extract_bezier_pieces(&fitted);
    let seeded = source
        .shaped_breaks
        .times()
        .iter()
        .filter(|&&t| t > t_start && t < t_end)
        .count();
    assert!(
        pieces.len() * 4 <= seeded,
        "{} pieces for {seeded} shaped breaks: the near-smooth images were not hidden",
        pieces.len()
    );
    const POINTS: u32 = 256;
    for piece in &pieces {
        let acceleration = piece.differentiate().differentiate();
        let h = piece.u_end - piece.u_start;
        for k in 0..POINTS {
            let t = piece.u_start + h * (0.01 + 0.98 * f64::from(k) / f64::from(POINTS - 1));
            let (position, _, truth) = signal.eval_pva(t);
            assert!(
                (piece.evaluate(t) - position).abs() <= FIT_TOL.pos_mm,
                "position misses by {} at {t}",
                (piece.evaluate(t) - position).abs()
            );
            assert!(
                (acceleration.evaluate(t) - truth).abs() <= FIT_TOL.accel_mm_s2,
                "acceleration misses by {} at {t}",
                (acceleration.evaluate(t) - truth).abs()
            );
        }
    }
}
