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

/// A straight line whose exact acceleration carries a Gaussian bump the
/// line itself does not follow.
struct AccelerationBump {
    t0: f64,
    h: f64,
    center_u: f64,
    width_u: f64,
    height_mm_s2: f64,
}

impl TrackSignal for AccelerationBump {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let from_center = ((t - self.t0) / self.h - self.center_u) / self.width_u;
        (
            150.0 + 300.0 * (t - self.t0),
            300.0,
            self.height_mm_s2 * libm::exp(-0.5 * from_center * from_center),
        )
    }
}

const BUMP_TOLERANCE: FitTol = FitTol {
    pos_mm: 0.005,
    accel_mm_s2: 50.0,
};

fn audit_line_against(
    bump: &AccelerationBump,
    edge_insets: (f64, f64),
) -> Vec<super::FitViolation> {
    let line_in_u = [150.0 + 150.0 * bump.h, 150.0 * bump.h];
    take_violations();
    check(
        bump,
        AcceptedPiece {
            fit_context: "materialize_source",
            axis: 0,
            fit_tol: BUMP_TOLERANCE,
            t0: bump.t0,
            t1: bump.t0 + bump.h,
            mono_u: &line_in_u,
            accepted_at_floor: false,
            cubic_floor: 0.62e-6,
            edge_insets,
        },
    );
    take_violations()
}

fn the_one_violation(violations: &[super::FitViolation]) -> &super::FitViolation {
    let [violation] = violations else {
        panic!("the bump breaks the tolerance in exactly one piece: {violations:?}");
    };
    violation
}

#[test]
fn a_miss_between_the_onset_check_points_of_the_fit_is_found_at_its_peak() {
    let bump = AccelerationBump {
        t0: 3.0,
        h: 2e-3,
        center_u: 1.0 / 64.0,
        width_u: 3e-3,
        height_mm_s2: 1.5 * BUMP_TOLERANCE.accel_mm_s2,
    };
    let violations = audit_line_against(&bump, (0.0, 0.0));
    let violation = the_one_violation(&violations);
    assert!(
        (violation.acceleration_over_tolerance - 1.5).abs() < 0.01
            && (violation.worst_u - bump.center_u).abs() < 1e-4,
        "{violation:?}"
    );
}

#[test]
fn a_miss_at_a_resolved_edge_is_a_violation() {
    let h = 2e-3;
    let inset = 0.01 * h;
    let bump = AccelerationBump {
        t0: 3.0,
        h,
        center_u: 0.01,
        width_u: 2e-3,
        height_mm_s2: 1.5 * BUMP_TOLERANCE.accel_mm_s2,
    };
    let violations = audit_line_against(&bump, (inset, inset));
    let violation = the_one_violation(&violations);
    assert!(violation.acceleration_over_tolerance > 1.4, "{violation:?}");
}
