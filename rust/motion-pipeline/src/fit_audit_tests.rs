use super::{AcceptedPiece, CapContactScope, Exemption, check, take_violations};
use crate::lowering::{AccelBudget, FitTol, quintic_in_u};
use crate::shaper::TrackSignal;

struct SquareRootOnset {
    at: f64,
    coefficient: f64,
}

impl TrackSignal for SquareRootOnset {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = (t - self.at).max(0.0);
        let root = tau.sqrt();
        (
            150.0 + 300.0 * (t - self.at) + self.coefficient * 4.0 / 15.0 * tau * tau * root,
            300.0 + self.coefficient * 2.0 / 3.0 * tau * root,
            self.coefficient * root,
        )
    }
}

fn audit_floor_quintic_at_onset(contacts: &[f64]) -> Vec<super::FitViolation> {
    let sig = SquareRootOnset {
        at: 0.211_721_933,
        coefficient: 9.8e6,
    };
    let (t0, t1) = (sig.at, sig.at + 0.8e-6);
    let mono_u = quintic_in_u(sig.eval_pva(t0), sig.eval_pva(t1), t1 - t0);
    let _scope = CapContactScope::enter(contacts);
    take_violations();
    check(
        &sig,
        AcceptedPiece {
            fit_context: "materialize_source",
            axis: 1,
            fit_tol: FitTol::absolute(0.005, 50.0),
            t0,
            t1,
            mono_u: &mono_u,
            accepted_at_floor: true,
            cubic_floor: 0.62e-6,
            edge_insets: (0.0, 0.0),
            exact: None,
        },
    );
    take_violations()
}

#[test]
fn a_floor_piece_on_a_cap_contact_onset_is_exempt_within_the_minimax_bound() {
    let violations = audit_floor_quintic_at_onset(&[0.211_721_933]);
    let [violation] = violations.as_slice() else {
        panic!("the quintic across a square-root onset must break the tolerance: {violations:?}");
    };
    match violation.exemption {
        Some(Exemption::CapContactOnset {
            onset_coefficient,
            error_mm_s2,
            bound_mm_s2,
        }) => {
            assert!(
                (onset_coefficient - 9.8e6).abs() < 1e3,
                "measured onset coefficient {onset_coefficient}"
            );
            assert!(error_mm_s2 > 50.0 && error_mm_s2 <= bound_mm_s2);
        }
        None => panic!("an onset floor piece within the bound is not exempt: {violation:?}"),
    }
}

#[test]
fn a_floor_piece_away_from_any_cap_contact_stays_a_violation() {
    let violations = audit_floor_quintic_at_onset(&[0.2117]);
    assert!(
        violations
            .iter()
            .all(|violation| violation.exemption.is_none()),
        "{violations:?}"
    );
    assert_eq!(violations.len(), 1);
}

/// The signal a flat piece over `[0, h]` misses by an error pinned at both
/// ends, bending at a constant `bend_mm_s2`.
struct PinnedParabola {
    h: f64,
    bend_mm_s2: f64,
}

impl TrackSignal for PinnedParabola {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        (
            0.5 * self.bend_mm_s2 * t * (self.h - t),
            0.5 * self.bend_mm_s2 * (self.h - 2.0 * t),
            -self.bend_mm_s2,
        )
    }
}

const PINNED_SPAN_S: f64 = 1e-3;
const FOLLOWER_POSITION_TOL_MM: f64 = 1e-4;

fn audit_flat_piece_over_pinned_parabola(
    bend_mm_s2: f64,
    accel_budget: AccelBudget,
) -> Vec<super::FitViolation> {
    let sig = PinnedParabola {
        h: PINNED_SPAN_S,
        bend_mm_s2,
    };
    take_violations();
    check(
        &sig,
        AcceptedPiece {
            fit_context: "follower_source",
            axis: 3,
            fit_tol: FitTol {
                accel_budget,
                ..FitTol::absolute(FOLLOWER_POSITION_TOL_MM, 50.0)
            },
            t0: 0.0,
            t1: PINNED_SPAN_S,
            mono_u: &[0.0],
            accepted_at_floor: false,
            cubic_floor: 1e-9,
            edge_insets: (0.0, 0.0),
            exact: None,
        },
    );
    take_violations()
}

fn position_equivalent_acceleration() -> f64 {
    8.0 * FOLLOWER_POSITION_TOL_MM / (PINNED_SPAN_S * PINNED_SPAN_S)
}

#[test]
fn a_follower_piece_bending_within_its_position_budget_holds_a_position_equivalent_tolerance() {
    let bend = 0.5 * position_equivalent_acceleration();
    assert!(
        bend > 50.0,
        "the bend must break the absolute 50 mm/s² budget"
    );
    let violations = audit_flat_piece_over_pinned_parabola(bend, AccelBudget::PositionEquivalent);
    assert!(violations.is_empty(), "{violations:?}");
    let absolute = audit_flat_piece_over_pinned_parabola(bend, AccelBudget::Absolute);
    let [absolute] = absolute.as_slice() else {
        panic!("the same piece must break an absolute 50 mm/s² budget: {absolute:?}");
    };
    assert!((absolute.acceleration_over_tolerance - bend / 50.0).abs() < 1e-9 * bend);
}

#[test]
fn a_follower_piece_bending_past_its_position_budget_breaks_a_position_equivalent_tolerance() {
    let bend = 2.0 * position_equivalent_acceleration();
    let violations = audit_flat_piece_over_pinned_parabola(bend, AccelBudget::PositionEquivalent);
    let [violation] = violations.as_slice() else {
        panic!("a bend of twice the position-equivalent acceleration must be a violation");
    };
    assert!(
        (violation.acceleration_over_tolerance - 2.0).abs() < 1e-9,
        "{violation:?}"
    );
}
