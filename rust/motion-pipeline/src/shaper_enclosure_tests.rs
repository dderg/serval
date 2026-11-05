use super::{NonlinearAdvanceSignal, TrackSignal, apply_nonlinear_advance_to_track};
use crate::lowering::FitTol;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AdvanceModel, NonlinearAdvance};

const AUDIT_POINTS_PER_PIECE: u32 = 200;

/// How far the fit leaves the signal at dense points inside its pieces, in
/// units of the tolerance each piece is held to.
fn worst_over_tolerance<S: TrackSignal>(sig: &S, track: &nurbs::ScalarNurbs, tol: FitTol) -> f64 {
    let mut worst = 0.0_f64;
    for piece in extract_bezier_pieces(track) {
        let h = piece.u_end - piece.u_start;
        let piece_tol = tol.for_piece(h);
        let acceleration = piece.differentiate().differentiate();
        for k in 0..AUDIT_POINTS_PER_PIECE {
            let t = piece.u_start + h * (f64::from(k) + 0.5) / f64::from(AUDIT_POINTS_PER_PIECE);
            let (position, _, truth_acceleration) = sig.eval_pva(t);
            worst = worst
                .max((piece.evaluate(t) - position).abs() / piece_tol.pos_mm)
                .max((acceleration.evaluate(t) - truth_acceleration).abs() / piece_tol.accel_mm_s2);
        }
    }
    worst
}

#[derive(Clone, Copy, Debug)]
struct AdvancedQuartic {
    duration: f64,
    velocity: f64,
    acceleration: f64,
    jerk: f64,
    snap: f64,
    advance: NonlinearAdvance,
}

impl AdvancedQuartic {
    const START: f64 = 1.0;
    const TOLERANCE: FitTol = FitTol::absolute(1e-3, 50.0);

    fn worst_over_tolerance_of_its_fit(&self) -> f64 {
        let track = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: Self::START,
            u_end: Self::START + self.duration,
            coeffs: vec![
                0.0,
                self.velocity,
                0.5 * self.acceleration,
                self.jerk / 6.0,
                self.snap / 24.0,
            ],
        }]);
        let sig = NonlinearAdvanceSignal::new(&track, self.advance);
        let fit = apply_nonlinear_advance_to_track(3, &track, self.advance, Self::TOLERANCE)
            .expect("the advanced quartic fits");
        worst_over_tolerance(&sig, &fit, Self::TOLERANCE)
    }
}

fn assert_holds_its_tolerance(case: AdvancedQuartic) {
    let worst = case.worst_over_tolerance_of_its_fit();
    assert!(
        worst <= 1.0,
        "{case:?}: the fit leaves its tolerance by {worst:.3}x"
    );
}

/// The between-probe estimates accepted this tanh advance's rungs 1.44x
/// over the acceleration tolerance.
#[test]
fn a_tanh_advance_bending_between_the_probes_is_proven_within_tolerance() {
    assert_holds_its_tolerance(AdvancedQuartic {
        duration: 0.134_948_609_740_770_7,
        velocity: -1.987_524_223_308_276,
        acceleration: -45.661_820_201_334_24,
        jerk: 7_981.493_574_745_934,
        snap: -253_276.970_903_101_58,
        advance: NonlinearAdvance {
            model: AdvanceModel::Tanh,
            linear_advance: 0.033_513_005_146_516_284,
            nonlinear_offset: 0.054_678_705_079_350_85,
            linearization_velocity: 6.259_977_319_758_175,
        },
    });
}

/// The between-probe estimates accepted this slow tanh advance's rungs
/// 1.46x over the acceleration tolerance.
#[test]
fn a_slow_tanh_advance_is_proven_within_tolerance() {
    assert_holds_its_tolerance(AdvancedQuartic {
        duration: 0.195_789_333_098_341_48,
        velocity: 3.608_445_866_618_646_7,
        acceleration: -42.032_911_210_146_2,
        jerk: -422.794_153_066_367_1,
        snap: 29_359.155_107_513_634,
        advance: NonlinearAdvance {
            model: AdvanceModel::Tanh,
            linear_advance: 0.011_042_356_732_553_715,
            nonlinear_offset: 0.116_250_390_020_546_48,
            linearization_velocity: 2.263_898_656_129_082,
        },
    });
}

#[test]
fn advanced_quartics_of_either_model_are_proven_within_tolerance() {
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut uniform = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1_u64 << 53) as f64
    };
    for _ in 0..1000 {
        let duration = 0.02 + 0.2 * uniform();
        let linearization_velocity = 0.5 + 8.0 * uniform();
        let velocity = (uniform() - 0.5) * 8.0 * linearization_velocity;
        let acceleration = (uniform() - 0.5) * 20.0 * linearization_velocity / duration;
        let jerk = (uniform() - 0.5) * 60.0 * linearization_velocity / (duration * duration);
        let snap = (uniform() - 0.5) * 200.0 * linearization_velocity / duration.powi(3);
        let model = if uniform() < 0.5 {
            AdvanceModel::Tanh
        } else {
            AdvanceModel::Reciprocal
        };
        assert_holds_its_tolerance(AdvancedQuartic {
            duration,
            velocity,
            acceleration,
            jerk,
            snap,
            advance: NonlinearAdvance {
                model,
                linear_advance: 0.05 * uniform(),
                nonlinear_offset: 0.02 + 0.1 * uniform(),
                linearization_velocity,
            },
        });
    }
}
