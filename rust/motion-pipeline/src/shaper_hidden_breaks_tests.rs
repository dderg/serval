use std::iter::once;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_discontinuous_nurbs, bezier_pieces_to_nurbs};
use trajectory::build_smooth_zv_kernel;

use super::{CruiseEnds, JumpSeries, input_jumps_at, kernel_jumps_at};
use crate::lowering::FitTol;
use crate::shaper::{
    AxisFit, AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, SignalBounds,
    TrackSignal, fit_leader_target,
};
use crate::types::PostProcessError;

const FIT_TOL: FitTol = FitTol::absolute(0.005, 50.0);

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
    }
}

/// An image without a velocity step is left to the fit while it lies
/// within the minimum span of the last seed, and seeded past it; the image
/// where the shaped signal leaves rest stays a seed however close.
#[test]
fn an_image_is_seeded_only_past_the_minimum_span_or_at_a_rest() {
    let kernel = build_smooth_zv_kernel(44.0);
    let track = rest_then_accelerate();
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, 2.0 * BREAK_S),
        track.knots().to_vec(),
        0.0,
    );
    let breaks = &source.shaped_breaks;
    let (k_lo, k_hi) = kernel.support();
    let image = |cut: f64| {
        *breaks
            .times()
            .iter()
            .find(|&&t| (t - (BREAK_S + cut)).abs() < 1e-12)
            .expect("the break's images are shaped breaks")
    };
    let select = |around: f64, by: f64| {
        breaks.select_seeds(
            vec![around - by, around, around + by],
            source.hidden_break_input(),
            FIT_TOL,
            CruiseEnds::LeftToTheFit,
        )
    };
    let moving = image(k_hi);
    let close = 0.5 * breaks.minimum_span;
    assert_eq!(select(moving, close), vec![moving - close, moving + close]);
    let far = 2.0 * breaks.minimum_span;
    assert_eq!(
        select(moving, far),
        vec![moving - far, moving, moving + far]
    );
    let leaving_rest = image(k_lo);
    assert_eq!(
        select(leaving_rest, close),
        vec![leaving_rest - close, leaving_rest, leaving_rest + close]
    );
}

const CRUISE_MM_S: f64 = 50.0;
const BRAKE_MM_S2: f64 = 3_000.0;
/// The sixth-power term a source fit leaves along a cruise, its
/// acceleration a few nanometres per second squared.
const CRUISE_FIT_RESIDUE: f64 = 1e-5;

/// A cruise until `BREAK_S`, then an acceleration of `accel` from it.
fn cruise_then(accel: f64) -> nurbs::ScalarNurbs {
    bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: BREAK_S,
            coeffs: vec![0.0, CRUISE_MM_S, 0.0, 0.0, 0.0, 0.0, CRUISE_FIT_RESIDUE],
        },
        BezierPiece {
            u_start: BREAK_S,
            u_end: 2.0 * BREAK_S,
            coeffs: vec![
                CRUISE_MM_S * BREAK_S + CRUISE_FIT_RESIDUE * BREAK_S.powi(6),
                CRUISE_MM_S + 6.0 * CRUISE_FIT_RESIDUE * BREAK_S.powi(5),
                0.5 * accel,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        },
    ])
}

/// The seeds kept of the image of `BREAK_S` under the kernel's first cut
/// and two seeds half the minimum span either side of it, beside those
/// three.
fn seeds_around_the_first_cut_image(
    track: &nurbs::ScalarNurbs,
    cruise_ends: CruiseEnds,
) -> (Vec<f64>, Vec<f64>) {
    let kernel = build_smooth_zv_kernel(44.0);
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(track, 0.0, 2.0 * BREAK_S),
        track.knots().to_vec(),
        0.0,
    );
    let breaks = &source.shaped_breaks;
    let (k_lo, _) = kernel.support();
    let image = *breaks
        .times()
        .iter()
        .find(|&&t| (t - (BREAK_S + k_lo)).abs() < 1e-12)
        .expect("the break's images are shaped breaks");
    let close = 0.5 * breaks.minimum_span;
    let offered = vec![image - close, image, image + close];
    (
        breaks.select_seeds(
            offered.clone(),
            source.hidden_break_input(),
            FIT_TOL,
            cruise_ends,
        ),
        offered,
    )
}

/// Where a fit anchors cruise ends, the image where the shaped signal
/// leaves a cruise stays a seed however close to the last one: a piece
/// across it would let the velocity wander where the plan holds it, past
/// the limit where the plan cruises at it.
#[test]
fn an_anchored_image_where_the_shaped_signal_leaves_a_cruise_stays_a_seed() {
    let (kept, offered) =
        seeds_around_the_first_cut_image(&cruise_then(-BRAKE_MM_S2), CruiseEnds::Anchored);
    assert_eq!(kept, offered);
}

/// A fit that leaves cruise ends to itself seeds that image only past the
/// minimum span.
#[test]
fn a_cruise_end_left_to_the_fit_is_seeded_only_past_the_minimum_span() {
    let (kept, offered) =
        seeds_around_the_first_cut_image(&cruise_then(-BRAKE_MM_S2), CruiseEnds::LeftToTheFit);
    assert_eq!(kept, vec![offered[0], offered[2]]);
}

/// A break the cruise runs through bounds no cruise, and its image is left
/// to the fit within the minimum span.
#[test]
fn the_image_of_a_break_inside_a_cruise_is_left_to_the_fit() {
    let (kept, offered) = seeds_around_the_first_cut_image(&cruise_then(0.0), CruiseEnds::Anchored);
    assert_eq!(kept, vec![offered[0], offered[2]]);
}

/// A box kernel turns an input position step into a velocity step of the
/// shaped signal at each image: a velocity break the plan must anchor a knot
/// at, so it stays a seed however little position it costs.
#[test]
fn an_image_carrying_a_resolvable_velocity_step_stays_a_seed() {
    let width = 0.02;
    let kernel = nurbs::algebra::PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![1.0 / width],
        (-0.5 * width, 0.5 * width),
    );
    let track = bezier_pieces_to_discontinuous_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: BREAK_S,
            coeffs: vec![0.0],
        },
        BezierPiece {
            u_start: BREAK_S,
            u_end: 2.0 * BREAK_S,
            coeffs: vec![1e-6],
        },
    ]);
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, 2.0 * BREAK_S),
        track.knots().to_vec(),
        0.0,
    );
    let image_t = *source
        .shaped_breaks
        .times()
        .iter()
        .find(|&&t| (t - (BREAK_S - 0.5 * width)).abs() < 1e-12)
        .expect("the step's first image is a shaped break");
    let seeds = vec![image_t - 1e-5, image_t, image_t + 1e-5];
    assert_eq!(
        source.shaped_breaks.select_seeds(
            seeds.clone(),
            source.hidden_break_input(),
            FIT_TOL,
            CruiseEnds::LeftToTheFit
        ),
        seeds
    );
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

/// A leader fit over an input whose seams barely break it seeds few of
/// their images, fits fewer pieces than it would seed, and every piece
/// still holds the tolerance against the shaped signal at dense interior
/// points.
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
    let pieces = fit_leader_target(
        AxisFit {
            axis: 0,
            t_start,
            t_end,
            seed_breakpoints: source.shaped_breaks.times(),
            fit_tol: FIT_TOL,
            fit_context: "hidden-break test",
            seam_continuity: crate::lowering::SeamContinuity::Velocity,
        },
        &signal,
        &source.shaped_breaks,
        source.hidden_break_input(),
    )
    .expect("a near-smooth input fits");
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

/// The shaped signal read through its evaluator alone, with no exact
/// polynomials to prove a fit against.
struct EvaluatedOnly<S>(S);

impl<S: TrackSignal> TrackSignal for EvaluatedOnly<S> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        self.0.eval_pva(t)
    }
}

/// A leader plan leaves images inside its spans for the hull to hold, so a
/// leader span the signal has no exact polynomials over is refused, not
/// estimated.
#[test]
fn a_leader_span_without_exact_polynomials_is_refused() {
    let kernel = build_smooth_zv_kernel(44.0);
    let track = piecewise_sine();
    let last_t = f64::from(SINE_PIECES) * SINE_PIECE_S;
    let source = ShapedSignalSource::new(
        &kernel,
        table_over(&track, 0.0, last_t),
        track.knots().to_vec(),
        0.0,
    );
    let refused = fit_leader_target(
        AxisFit {
            axis: 0,
            t_start: 0.05,
            t_end: 0.15,
            seed_breakpoints: source.shaped_breaks.times(),
            fit_tol: FIT_TOL,
            fit_context: "hidden-break test",
            seam_continuity: crate::lowering::SeamContinuity::Velocity,
        },
        &EvaluatedOnly(source.signal()),
        &source.shaped_breaks,
        source.hidden_break_input(),
    );
    assert!(
        matches!(refused, Err(PostProcessError::UnprovableSpan { .. })),
        "a span without exact polynomials was fitted: {refused:?}"
    );
}
