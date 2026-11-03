use super::{AccelBudget, FitTol};

const POSITION_EQUIVALENT: FitTol = FitTol {
    pos_mm: 1e-4,
    accel_mm_s2: 50.0,
    accel_budget: AccelBudget::PositionEquivalent,
};

#[test]
fn an_absolute_budget_holds_every_piece_to_the_configured_acceleration() {
    for h in [1e-5, 1e-3, 0.1, 1.0] {
        let piece = FitTol::absolute(1e-4, 50.0).for_piece(h);
        assert_eq!((piece.pos_mm, piece.accel_mm_s2), (1e-4, 50.0), "h = {h}");
    }
}

#[test]
fn a_position_equivalent_budget_is_the_acceleration_a_pinned_error_spends_the_position_budget_on() {
    for h in [1e-4, 1e-3, 2e-3] {
        let piece = POSITION_EQUIVALENT.for_piece(h);
        assert_eq!(piece.accel_budget, AccelBudget::Absolute);
        assert_eq!(piece.pos_mm, POSITION_EQUIVALENT.pos_mm);
        let pinned_parabola_peak = piece.accel_mm_s2 * h * h / 8.0;
        assert!(
            (pinned_parabola_peak - POSITION_EQUIVALENT.pos_mm).abs()
                <= 1e-12 * POSITION_EQUIVALENT.pos_mm,
            "over {h} s a pinned error at {} mm/s² peaks at {pinned_parabola_peak} mm",
            piece.accel_mm_s2
        );
    }
}

#[test]
fn a_position_equivalent_budget_never_holds_a_long_piece_tighter_than_the_configured_acceleration()
{
    let crossover = (8.0 * POSITION_EQUIVALENT.pos_mm / POSITION_EQUIVALENT.accel_mm_s2).sqrt();
    for h in [1.01 * crossover, 2.0 * crossover, 1.0] {
        assert_eq!(
            POSITION_EQUIVALENT.for_piece(h).accel_mm_s2,
            POSITION_EQUIVALENT.accel_mm_s2,
            "h = {h}"
        );
    }
}
