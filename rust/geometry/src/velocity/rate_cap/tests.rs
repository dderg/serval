use super::*;
use crate::velocity::SeamLimit;

const MAX_RATE: f64 = 20.0;
const ACCEL: f64 = 12_000.0;
const RATE: LinearRate = LinearRate {
    per_velocity: 0.04,
    per_accel: 0.0015,
};

/// The exit half of a blend from a 20 mm/s apex into a fast line (#914):
/// the run through the line peaks far above the move.
fn blend_exit() -> ReachableSpeed {
    ReachableSpeed {
        peak_mm_s: 92.0,
        entry: SeamLimit {
            ceiling_mm_s: 20.0,
            kappa: 8.0,
        },
        exit: SeamLimit {
            ceiling_mm_s: 100.0,
            kappa: 0.0,
        },
        run_mm: 2.3,
        behind_mm: 0.0,
        ahead_mm: 2.13,
    }
}

/// The entry half of a blend braking from a fast line into a 20 mm/s
/// apex (#914): its reachable peak, read at the most acceleration it can
/// take, lies above what it reaches on a traded budget.
fn blend_entry() -> ReachableSpeed {
    ReachableSpeed {
        peak_mm_s: 96.0,
        entry: SeamLimit {
            ceiling_mm_s: 100.0,
            kappa: 0.0,
        },
        exit: SeamLimit {
            ceiling_mm_s: 20.0,
            kappa: 2.0,
        },
        run_mm: 1.99,
        behind_mm: 1.62,
        ahead_mm: 0.0,
    }
}

#[test]
fn the_trade_spends_no_budget_on_speed_the_move_cannot_reach() {
    for (move_name, reachable) in [("blend exit", blend_exit()), ("blend entry", blend_entry())] {
        assert!(
            RATE.at(reachable.peak_mm_s, ACCEL) > MAX_RATE,
            "{move_name}"
        );

        let (ceiling, budget) = RATE.capped(MAX_RATE, &reachable, (380.0, ACCEL));

        let reached = reachable.peak_under(budget);
        assert!(
            ceiling <= reached * (1.0 + 1e-9),
            "{move_name}: ceiling {ceiling} above the {reached} the move reaches on {budget}"
        );
        let spent = RATE.at(ceiling, budget);
        assert!(
            spent >= MAX_RATE * (1.0 - 1e-9),
            "{move_name}: the pair ({ceiling}, {budget}) leaves {} of the rate unspent",
            MAX_RATE - spent
        );
    }
}
