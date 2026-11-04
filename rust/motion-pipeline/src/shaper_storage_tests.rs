use nurbs::bezier::BezierPiece;

use crate::lowering::FitTol;
use crate::shaper::{AxisFit, TrackSignal, apply_derivative_gains_to_track, fit_axis_from_signal};

const BENCH_TOL: FitTol = FitTol::absolute(0.005, 50.0);
const FAR_FROM_ORIGIN_MM: f64 = 300.0;

fn mode_inverse_gains(frequency_hz: f64, damping_ratio: f64) -> (f64, f64) {
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    (2.0 * damping_ratio / omega, 1.0 / (omega * omega))
}

fn translated(track: &nurbs::ScalarNurbs, offset: f64) -> nurbs::ScalarNurbs {
    nurbs::ScalarNurbs::try_new(
        track.degree(),
        track.knots().to_vec(),
        track.control_points().iter().map(|p| p - offset).collect(),
    )
    .expect("a translated track keeps its knots")
}

fn interior_samples(track: &nurbs::ScalarNurbs) -> Vec<f64> {
    track
        .knots()
        .windows(2)
        .filter(|span| span[1] > span[0])
        .flat_map(|span| [0.25, 0.5, 0.75].map(|f| span[0] + f * (span[1] - span[0])))
        .collect()
}

fn degree_13_track_at(offset: f64, piece_s: f64) -> nurbs::ScalarNurbs {
    let (v, a, j) = (300.0, 10_000.0, 1.0e6);
    let pieces: Vec<BezierPiece> = (0..8)
        .map(|i| {
            let t = piece_s * f64::from(i);
            let mut coeffs = vec![
                offset + v * t + a * t * t / 2.0 + j * t * t * t / 6.0,
                v + a * t + j * t * t / 2.0,
                (a + j * t) / 2.0,
                j / 6.0,
            ];
            coeffs.resize(14, 0.0);
            BezierPiece {
                u_start: t,
                u_end: piece_s * f64::from(i + 1),
                coeffs,
            }
        })
        .collect();
    nurbs::bezier::bezier_pieces_to_discontinuous_nurbs(&pieces)
}

#[test]
fn derivative_gains_commute_with_a_translation_far_from_the_origin() {
    let (k1, k2) = mode_inverse_gains(50.0, 0.1);
    let track = degree_13_track_at(FAR_FROM_ORIGIN_MM, 1.0e-6);
    let near_origin = translated(&track, FAR_FROM_ORIGIN_MM);
    let gained = apply_derivative_gains_to_track(&track, k1, k2);
    let gained_near_origin = apply_derivative_gains_to_track(&near_origin, k1, k2);
    let worst = interior_samples(&track)
        .into_iter()
        .map(|t| {
            let far = nurbs::eval::eval(&gained, t) - FAR_FROM_ORIGIN_MM;
            let near = nurbs::eval::eval(&gained_near_origin, t);
            ((far - near).abs(), t)
        })
        .fold((0.0, f64::NAN), |worst, sample| {
            if sample.0 > worst.0 { sample } else { worst }
        });
    assert!(
        worst.0 <= 1e-9,
        "the motor command at t={} moved by {:e} mm when the same motion was \
         commanded {FAR_FROM_ORIGIN_MM} mm from the origin",
        worst.1,
        worst.0
    );
}

struct OffsetSine {
    offset: f64,
    amplitude: f64,
    omega: f64,
}

impl TrackSignal for OffsetSine {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let sin = libm::sin(self.omega * t);
        (
            self.offset + self.amplitude * sin,
            self.amplitude * self.omega * libm::cos(self.omega * t),
            -self.amplitude * self.omega * self.omega * sin,
        )
    }
}

fn worst_case_stored_acceleration(degree: usize, position_mm: f64, span_s: f64) -> f64 {
    let alternating_half_ulp_second_difference = 2.0 * f64::EPSILON * position_mm.abs();
    (degree * (degree - 1)) as f64 * alternating_half_ulp_second_difference / (span_s * span_s)
}

fn shortest_span_storing_half_the_accel_budget(degree: usize, position_mm: f64) -> f64 {
    (worst_case_stored_acceleration(degree, position_mm, 1.0) / (0.5 * BENCH_TOL.accel_mm_s2))
        .sqrt()
}

#[test]
fn a_nanosecond_seed_cluster_far_from_the_origin_keeps_the_motor_command_in_budget() {
    let sig = OffsetSine {
        offset: FAR_FROM_ORIGIN_MM,
        amplitude: 1.0,
        omega: 1500.0,
    };
    let sub_floor_cluster_s = 0.8
        * shortest_span_storing_half_the_accel_budget(
            crate::lowering::LADDER_MAX_DEGREE,
            FAR_FROM_ORIGIN_MM,
        );
    let clusters = [0.004, 0.004 + 3.0e-9, 0.006, 0.006 + sub_floor_cluster_s];
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 0,
            t_start: 0.0,
            t_end: 0.01,
            seed_breakpoints: &clusters,
            fit_tol: BENCH_TOL,
            fit_context: "test fit",
            velocity_continuous: false,
        },
        &sig,
    )
    .expect("the offset sine must fit");
    let degree = track.degree() as usize;
    for span in track.knots().windows(2).filter(|span| span[1] > span[0]) {
        let position = sig.eval_pva(span[0]).0;
        let storage_noise = worst_case_stored_acceleration(degree, position, span[1] - span[0]);
        assert!(
            storage_noise <= 0.5 * BENCH_TOL.accel_mm_s2,
            "knot span [{}, {}] of a degree-{degree} track can store {storage_noise:e} mm/s² \
             of rounding acceleration",
            span[0],
            span[1]
        );
    }
    let (k1, k2) = mode_inverse_gains(50.0, 0.1);
    let motor = apply_derivative_gains_to_track(&track, k1, k2);
    let motor_budget = BENCH_TOL.pos_mm + k2 * BENCH_TOL.accel_mm_s2;
    for t in interior_samples(&track) {
        let (p, v, a) = sig.eval_pva(t);
        let mut stored = [0.0; 3];
        nurbs::eval::eval_derivatives(
            track.control_points(),
            track.knots(),
            track.degree(),
            t,
            2,
            &mut stored,
        );
        assert!(
            (stored[2] - a).abs() <= BENCH_TOL.accel_mm_s2,
            "the stored track reads {:e} mm/s² of acceleration error at t={t}",
            stored[2] - a
        );
        let commanded = nurbs::eval::eval(&motor, t);
        let wanted = p + k1 * v + k2 * a;
        assert!(
            (commanded - wanted).abs() <= motor_budget,
            "the mode-inverse motor command is off by {:e} mm at t={t}",
            commanded - wanted
        );
    }
}

/// The follower projection reads the leaders' jerk wherever an advance has
/// no kernel after it, and a sub-microsecond piece stored on absolute
/// control points, raised to its track's degree 13, rounds that jerk to
/// tens of millions of mm/s³: the dense trident_ramp print's y leader
/// carried 1.15e6 mm/s³ over a 1.013 µs piece at 149.47 mm and read
/// 4.8e7 off it, which the advance turned into 45 000 mm/s² of extruder
/// acceleration. The projection reads each piece at its own degree and
/// precision instead, the same motion as the motors run.
#[test]
fn the_projection_reads_a_sub_microsecond_leader_piece_at_its_own_precision() {
    let sliver_s = 1.013_006_574_623_887e-6;
    let (p0, v0, a0) = (
        149.471_624_299,
        27.677_913_381_254_74,
        -1_276.978_973_016_173_3,
    );
    let jerk = 1.153_942_962_494_199_6e6;
    let lead_s: f64 = 1.0e-3;
    let lead_start = (
        p0 - v0 * lead_s + a0 * lead_s * lead_s / 2.0 - jerk * lead_s.powi(3) / 6.0,
        v0 - a0 * lead_s + jerk * lead_s * lead_s / 2.0,
        a0 - jerk * lead_s,
    );
    let mut lead_coeffs = vec![lead_start.0, lead_start.1, lead_start.2 / 2.0, jerk / 6.0];
    lead_coeffs.resize(14, 0.0);
    let pieces = vec![
        BezierPiece {
            u_start: 0.0,
            u_end: lead_s,
            coeffs: lead_coeffs,
        },
        BezierPiece {
            u_start: lead_s,
            u_end: lead_s + sliver_s,
            coeffs: vec![p0, v0, a0 / 2.0, jerk / 6.0],
        },
    ];
    let track = crate::shaper::leader_track(1, pieces).expect("the leader pieces fit a track");
    let midpoint = lead_s + 0.5 * sliver_s;
    let read_axis = track
        .read
        .as_ref()
        .expect("a leader track is read on its own pieces");
    let read = read_axis
        .eval_pvaj(midpoint)
        .expect("the read track covers its pieces");
    assert!(
        (read.jerk - jerk).abs() <= 1.0e3,
        "the projection reads {:e} mm/s³ of jerk off a {jerk:e} mm/s³ leader piece",
        read.jerk
    );
    for t in [0.5 * lead_s, lead_s + 0.25 * sliver_s, midpoint] {
        let stored = nurbs::eval::eval(&track.stored, t);
        let read = read_axis
            .eval_pva(t)
            .expect("the read track covers its pieces");
        assert!(
            (read.position - stored).abs() <= 1.0e-12,
            "the read track is {:e} mm off the stored one at t={t}",
            read.position - stored
        );
    }
}
