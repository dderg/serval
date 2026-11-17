use nurbs::algebra::PiecewisePolynomialKernel;

use super::*;

fn bell(smooth_time: f64) -> PiecewisePolynomialKernel {
    let h = smooth_time / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c],
        (-h, h),
    )
}

fn corners() -> ShapedCorners {
    let (x, y) = (bell(0.02), bell(0.025));
    ShapedCorners::new([Some(&x), Some(&y)]).expect("both axes are shaped")
}

const ACCEL: f64 = 10_000.0;

#[test]
fn unshaped_axes_have_no_shaped_corners() {
    assert!(ShapedCorners::new([None, None]).is_none());
}

#[test]
fn the_floor_scales_with_the_acceleration_and_grows_with_the_turn() {
    let corners = corners();
    let at = |deg: f64, accel: f64| corners.floor_mm(deg.to_radians(), accel);
    assert!((at(90.0, 2.0 * ACCEL) / at(90.0, ACCEL) - 2.0).abs() < 1e-9);
    assert!(at(0.0, ACCEL) == 0.0);
    assert!(at(45.0, ACCEL) < at(90.0, ACCEL));
    assert!(at(90.0, ACCEL) < at(135.0, ACCEL));
}

#[test]
fn a_sharp_stop_misses_by_about_its_half_variance_pull_projected_on_the_turn() {
    let corners = corners();
    let variance = (0.025_f64 / 2.0).powi(2) / 7.0;
    let pull = 0.5 * variance * ACCEL;
    let floor = corners.floor_mm(90f64.to_radians(), ACCEL) / PREDICTION_MARGIN;
    assert!(
        floor > 0.5 * pull && floor < 1.2 * pull,
        "floor {floor} against the half-variance pull {pull}"
    );
}

#[test]
fn the_miss_never_falls_below_the_cut_and_rises_with_the_apex_speed() {
    let corners = corners();
    let theta = 90f64.to_radians();
    for cut in [0.001, 0.01, 0.05, 0.2, 2.0] {
        let mut previous = 0.0;
        for step in 0..=10 {
            let miss = corners.miss_mm(theta, cut, f64::from(step) / 10.0, ACCEL);
            assert!(miss >= cut, "cut {cut}: miss {miss}");
            assert!(miss >= previous, "cut {cut}: the miss falls with speed");
            previous = miss;
        }
    }
}

#[test]
fn the_apex_speed_ratio_keeps_the_miss_within_the_budget() {
    let corners = corners();
    let theta = 120f64.to_radians();
    let cut = 0.02;
    let budget = corners.miss_mm(theta, cut, 0.6, ACCEL);
    let ratio = corners
        .apex_speed_ratio(theta, cut, ACCEL, budget)
        .expect("a stop fits the budget");
    assert!((ratio - 0.6).abs() < 1e-3, "ratio {ratio}");
    assert!(corners.apex_speed_ratio(theta, cut, ACCEL, cut).is_none());
}

#[test]
fn a_corner_past_its_budget_at_rest_slows_until_its_stop_fits() {
    let corners = corners();
    let theta = 90f64.to_radians();
    let budget = 0.5 * corners.floor_mm(theta, ACCEL);
    let accel = corners.corner_accel(theta, 0.0, ACCEL, budget);
    assert!((corners.floor_mm(theta, accel) / budget - 1.0).abs() < 1e-6);
    let blended = corners.corner_accel(theta, 0.2 * budget, ACCEL, budget);
    assert!(corners.miss_mm(theta, 0.2 * budget, 0.0, blended) <= budget);
}

#[test]
fn the_fitted_cut_fits_its_budget_and_crosses_the_apex_faster_than_a_stop() {
    let corners = corners();
    for deg in [20.0, 45.0, 90.0, 135.0] {
        let theta = f64::to_radians(deg);
        for budget in [
            CornerBudget::Shaped(0.5),
            CornerBudget::OverFloor(0.04),
            CornerBudget::ScvEquivalent(5.0),
        ] {
            let cut = corners.blend_cut_mm(budget, theta, ACCEL);
            let budget_mm = corners.budget_mm(budget, theta, ACCEL);
            assert!(cut > 0.0 && cut < budget_mm, "{deg}° {budget:?}: cut {cut}");
            assert!(
                corners
                    .apex_speed_ratio(theta, cut, ACCEL, budget_mm)
                    .is_some(),
                "{deg}° {budget:?}"
            );
        }
    }
}

#[test]
fn an_explicit_budget_under_the_floor_still_gets_a_cut_at_the_slowed_acceleration() {
    let corners = corners();
    let theta = 90f64.to_radians();
    let budget = 0.5 * corners.floor_mm(theta, ACCEL);
    let cut = corners.blend_cut_mm(CornerBudget::Shaped(budget), theta, ACCEL);
    assert!(cut > 0.0 && cut < budget);
}

#[test]
fn a_slow_circle_shrinks_by_half_its_variance_times_its_normal_acceleration() {
    let corners = corners();
    let variance = (0.025_f64 / 2.0).powi(2) / 7.0;
    let (radius, speed) = (50.0, 50.0);
    let expected = PREDICTION_MARGIN * 0.5 * variance * speed * speed / radius;
    let shrink = corners.arc_shrink_mm(radius, speed);
    assert!(
        (shrink / expected - 1.0).abs() < 0.05,
        "{shrink} against {expected}"
    );
    let cap = corners.arc_speed_cap(radius, shrink);
    assert!((cap / speed - 1.0).abs() < 1e-3, "{cap}");
}

#[test]
fn the_scv_allowance_grows_with_the_square_corner_velocity() {
    let corners = corners();
    let slow = corners.scv_allowance_mm(2.0, ACCEL);
    let fast = corners.scv_allowance_mm(10.0, ACCEL);
    assert!(slow > 0.0 && fast > slow, "{slow} {fast}");
}
