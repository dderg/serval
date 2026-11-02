use std::iter::once;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::build_smooth_mzv_kernel;

use super::{
    AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, SignalBounds, TrackSignal,
};

const ACCEL_MM_S2: f64 = 8_000.0;
const HALF_S: f64 = 0.05;
const FAR_MM: f64 = 1_000.0;

fn accelerate_then_brake(offset_mm: f64) -> nurbs::ScalarNurbs {
    let peak_velocity = ACCEL_MM_S2 * HALF_S;
    let half_travel = 0.5 * ACCEL_MM_S2 * HALF_S * HALF_S;
    bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: HALF_S,
            coeffs: vec![offset_mm, 0.0, 0.5 * ACCEL_MM_S2],
        },
        BezierPiece {
            u_start: HALF_S,
            u_end: 2.0 * HALF_S,
            coeffs: vec![offset_mm + half_travel, peak_velocity, -0.5 * ACCEL_MM_S2],
        },
    ])
}

const PROBE_STEP_S: f64 = 1e-5;

fn worst_travel_mismatch(offset_mm: f64) -> f64 {
    let kernel = build_smooth_mzv_kernel(50.0);
    let track = accelerate_then_brake(offset_mm);
    let input = AxisSignalTable::from_tracks(
        once(&track),
        SignalBounds {
            first_t: 0.0,
            last_t: 2.0 * HALF_S,
            boundary: FitBoundary::StreamBoundary,
            lookahead: Lookahead::Forced,
        },
    );
    let source = ShapedSignalSource::new(&kernel, input, track.knots().to_vec(), HALF_S);
    let signal = source.signal();
    (1..400)
        .map(|k| {
            let t = 0.15 * f64::from(k) / 400.0;
            let (before, after) = (t - PROBE_STEP_S, t + PROBE_STEP_S);
            let travel = signal.eval(after) - signal.eval(before);
            let simpson = PROBE_STEP_S / 3.0
                * (signal.deriv(before) + 4.0 * signal.deriv(t) + signal.deriv(after));
            (travel - simpson).abs()
        })
        .fold(0.0, f64::max)
}

#[test]
fn the_shaped_position_does_not_pick_up_noise_from_the_axis_offset() {
    let near = worst_travel_mismatch(0.0);
    let far = worst_travel_mismatch(FAR_MM);
    let rounding_at_far = 8.0 * f64::EPSILON * FAR_MM;
    assert!(
        far <= near + rounding_at_far,
        "travel disagrees with the velocity integral by {far:e} mm at a {FAR_MM} mm offset, \
         {near:e} mm at the origin"
    );
}
