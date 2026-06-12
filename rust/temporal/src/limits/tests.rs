use super::*;

fn set(axes: &[usize], v: f64, a: f64, j: f64) -> LimitSet {
    LimitSet {
        axes: AxisSet::from_indices(axes),
        v_max: v,
        a_max: a,
        j_max: j,
    }
}

#[test]
fn coverage_validation_rejects_uncovered_axis() {
    let err = NormLimits::try_new(&[set(&[0, 1], 300.0, 3000.0, 6000.0)]).unwrap_err();
    assert!(matches!(err, LimitsError::NoVelocityCoverage { axis: 2 }));
}

#[test]
fn coverage_is_per_derivative() {
    let err = NormLimits::try_new(&[
        set(&[0, 1, 2], 300.0, f64::INFINITY, f64::INFINITY),
        set(&[0, 1], f64::INFINITY, 3000.0, 6000.0),
    ])
    .unwrap_err();
    assert!(matches!(err, LimitsError::NoAccelCoverage { axis: 2 }));
}

#[test]
fn rejects_nonpositive_caps() {
    let err = NormLimits::try_new(&[set(&[0], 0.0, 100.0, 200.0)]).unwrap_err();
    assert!(matches!(err, LimitsError::BadCap { set: 0 }));
}

#[test]
fn mvc_b_is_min_over_sets() {
    let lim = NormLimits::try_new(&[
        set(&[0, 1], 60.0, 6000.0, 12000.0),
        set(&[1], 40.0, f64::INFINITY, f64::INFINITY),
        set(&[2], 15.0, 100.0, 200.0),
    ])
    .unwrap();
    let pure_y = [0.0, 1.0, 0.0];
    assert!((lim.mvc_b(&pure_y, 1e-9) - 1600.0).abs() < 1e-9);
    let diag = [
        std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];
    let expected = (40.0 / std::f64::consts::FRAC_1_SQRT_2).powi(2).min(3600.0);
    assert!((lim.mvc_b(&diag, 1e-9) - expected).abs() < 1e-6);
}

#[test]
fn kappa_set_is_orthogonal_component_of_restricted_second_derivative() {
    let c_prime = [1.0, 0.0, 0.0];
    let c_double_prime = [0.0, 2.0, 1.0];
    assert!(
        (kappa_set(
            &c_prime,
            &c_double_prime,
            AxisSet::from_indices(&[0, 1]),
            1e-12
        ) - 2.0)
            .abs()
            < 1e-12
    );
    assert!(
        (kappa_set(
            &c_prime,
            &c_double_prime,
            AxisSet::from_indices(&[0, 1, 2]),
            1e-12
        ) - 5.0_f64.sqrt())
        .abs()
            < 1e-12
    );
    let c_dp_tangential = [3.0, 0.0, 0.0];
    assert!(
        kappa_set(
            &c_prime,
            &c_dp_tangential,
            AxisSet::from_indices(&[0]),
            1e-12
        )
        .abs()
            < 1e-12
    );
}

#[test]
fn b_cent_cap_uses_per_set_kappa() {
    let lim = NormLimits::try_new(&[
        set(&[0, 1], 300.0, 1000.0, 2000.0),
        set(&[2], 15.0, 100.0, 200.0),
    ])
    .unwrap();
    let c_prime = [1.0, 0.0, 0.0];
    let c_double_prime = [0.0, 0.5, 0.0];
    assert!((lim.b_cent_cap(&c_prime, &c_double_prime, 1e-12) - 2000.0).abs() < 1e-9);
    let c_dp_z_only = [0.0, 0.0, 0.5];
    assert!((lim.b_cent_cap(&c_prime, &c_dp_z_only, 1e-12) - 200.0).abs() < 1e-9);
}
