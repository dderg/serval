use std::iter::once;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_discontinuous_nurbs};
use trajectory::build_smooth_zv_kernel;

use super::{
    AxisFit, AxisSignalTable, FitBoundary, Lookahead, ShapedSignalSource, SignalBounds,
    TrackSignal, fit_axis_from_signal,
};
use crate::lowering::FitTol;

const TOLERANCE: FitTol = FitTol::absolute(1e-4, 50.0);

const TAYLOR_DEGREE: usize = 9;
const PIECE_S: f64 = 5e-4;
const SIGNAL_S: f64 = 0.12;
const TARGET_START_S: f64 = 0.05;
const TARGET_S: f64 = 0.014;
const KERNEL_HZ: f64 = 250.0;
const AUDIT_POINTS: u32 = 4000;

/// The shaped counterpart of `WaveWithQuietKnot`: a quadratic trend under
/// a sine, stored as degree-9 Taylor pieces whose seams are quiet knots,
/// convolved with a smooth ZV kernel and fitted over 14 ms seeded by none
/// of its images, as a follower kernel fit is seeded by its semantic cuts
/// alone. Every image hides inside a span, where only what holds the
/// fit between the probes sees it. `peak_over_tolerance` is the shaped
/// wave's own acceleration amplitude over the fit tolerance.
struct ShapedWave {
    kernel: nurbs::algebra::PiecewisePolynomialKernel,
    track: nurbs::ScalarNurbs,
}

impl ShapedWave {
    fn new(cycles: f64, eighths_of_a_turn: f64, peak_over_tolerance: f64) -> Self {
        let kernel = build_smooth_zv_kernel(KERNEL_HZ);
        let angular_rate = std::f64::consts::TAU * cycles / TARGET_S;
        let amplitude = peak_over_tolerance * TOLERANCE.accel_mm_s2
            / (angular_rate * angular_rate * kernel_gain(&kernel, angular_rate));
        let phase = eighths_of_a_turn * std::f64::consts::TAU / 8.0;
        let piece_count = (SIGNAL_S / PIECE_S).round() as u32;
        let pieces = (0..piece_count)
            .map(|index| {
                let start = f64::from(index) * PIECE_S;
                let mut factorial = 1.0;
                let coeffs = (0..=TAYLOR_DEGREE)
                    .map(|order| {
                        if order > 0 {
                            factorial *= order as f64;
                        }
                        let wave_phase = angular_rate * (start - TARGET_START_S)
                            + phase
                            + order as f64 * std::f64::consts::FRAC_PI_2;
                        let wave =
                            amplitude * angular_rate.powi(order as i32) * libm::sin(wave_phase)
                                / factorial;
                        wave + match order {
                            0 => 10.0 * start + 100.0 * start * start,
                            1 => 10.0 + 200.0 * start,
                            2 => 100.0,
                            _ => 0.0,
                        }
                    })
                    .collect();
                BezierPiece {
                    u_start: start,
                    u_end: f64::from(index + 1) * PIECE_S,
                    coeffs,
                }
            })
            .collect::<Vec<_>>();
        Self {
            kernel,
            track: bezier_pieces_to_discontinuous_nurbs(&pieces),
        }
    }

    /// The worst position and acceleration misses of the fit over the
    /// target, each over its tolerance.
    fn worst_misses_of_its_fit(&self) -> (f64, f64) {
        let source = ShapedSignalSource::new(
            &self.kernel,
            AxisSignalTable::from_tracks(
                once(&self.track),
                SignalBounds {
                    first_t: 0.0,
                    last_t: SIGNAL_S,
                    boundary: FitBoundary::StreamBoundary,
                    lookahead: Lookahead::Forced,
                },
            ),
            self.track.knots().to_vec(),
            TARGET_START_S,
        );
        let signal = source.signal();
        let fitted = fit_axis_from_signal(
            AxisFit {
                axis: 0,
                t_start: TARGET_START_S,
                t_end: TARGET_START_S + TARGET_S,
                seed_breakpoints: &[],
                fit_tol: TOLERANCE,
                fit_context: "shaped wave",
                seam_continuity: crate::lowering::SeamContinuity::Velocity,
            },
            &signal,
        )
        .expect("a shaped wave fits");
        let acceleration = nurbs::eval::derivative(&nurbs::eval::derivative(&fitted));
        (0..AUDIT_POINTS)
            .map(|k| {
                let t = TARGET_START_S + TARGET_S * (f64::from(k) + 0.5) / f64::from(AUDIT_POINTS);
                let (position, _, truth) = signal.eval_pva(t);
                (
                    (nurbs::eval::eval(&fitted, t) - position).abs() / TOLERANCE.pos_mm,
                    (nurbs::eval::eval(&acceleration, t) - truth).abs() / TOLERANCE.accel_mm_s2,
                )
            })
            .fold((0.0, 0.0), |(p, a), (sp, sa)| {
                (f64::max(p, sp), f64::max(a, sa))
            })
    }
}

/// `|K̂(ω)|`, the kernel's gain at angular rate `ω`.
fn kernel_gain(kernel: &nurbs::algebra::PiecewisePolynomialKernel, angular_rate: f64) -> f64 {
    const STEPS: u32 = 4096;
    let (mut cosine, mut sine) = (0.0, 0.0);
    for piece in &kernel.pieces {
        let width = piece.u_end - piece.u_start;
        for k in 0..STEPS {
            let tau = piece.u_start + width * (f64::from(k) + 0.5) / f64::from(STEPS);
            let weight = piece.evaluate(tau) * width / f64::from(STEPS);
            cosine += weight * libm::cos(angular_rate * tau);
            sine += weight * libm::sin(angular_rate * tau);
        }
    }
    libm::sqrt(cosine * cosine + sine * sine)
}

/// Over 1.5–7.9 cycles of the span, every eighth of a turn of phase and
/// four wave amplitudes, the fit accepts nothing that leaves the position
/// or acceleration tolerance anywhere in the target. Measured before the
/// hull acceptance: 44 of these 1056 settings missed, by up to 1.38× the
/// acceleration tolerance.
#[test]
fn a_shaped_wave_past_its_unseeded_images_holds_the_tolerance_everywhere() {
    let mut misses = Vec::new();
    for tenths_of_a_cycle in (15..80).step_by(2) {
        for eighths in 0..8 {
            for peak in [1.0, 1.4, 2.0, 3.0] {
                let cycles = f64::from(tenths_of_a_cycle) / 10.0;
                let (position, acceleration) =
                    ShapedWave::new(cycles, f64::from(eighths), peak).worst_misses_of_its_fit();
                if position > 1.0 || acceleration > 1.0 {
                    misses.push(format!(
                        "{cycles} cycles, {eighths}/8 turn, peak {peak}: position {position:.3}×, \
                         acceleration {acceleration:.3}×"
                    ));
                }
            }
        }
    }
    assert!(
        misses.is_empty(),
        "{} settings accept pieces past the tolerance:\n{}",
        misses.len(),
        misses.join("\n")
    );
}
