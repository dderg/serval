use super::{AcceptedPiece, CapContactScope, Exemption, check, take_violations};
use crate::lowering::{FitTol, quintic_in_u};
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
            fit_tol: FitTol {
                pos_mm: 0.005,
                accel_mm_s2: 50.0,
            },
            t0,
            t1,
            mono_u: &mono_u,
            accepted_at_floor: true,
            cubic_floor: 0.62e-6,
            edge_insets: (0.0, 0.0),
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
