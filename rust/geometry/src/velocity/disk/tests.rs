use super::*;

fn kin(length: f64, accel: f64, kappa0: f64, sigma: f64, ceiling: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk: f64::INFINITY,
        kappa0,
        sigma,
        flat_ceiling: ceiling,
    }
}

#[test]
fn arc_reach_matches_the_closed_form() {
    // On a constant-curvature arc the rail has the closed form
    // w(s) = w_lim * sin(2*kappa*s + asin(w0/w_lim)).
    let k = kin(4.0, 1000.0, 0.12, 0.0, f64::INFINITY);
    let v = disk_reach_v(&k, 5.0, 4.0, 1e-9).unwrap();
    let w_lim = 1000.0 / 0.12;
    let arg = 2.0 * 0.12 * 4.0 + libm::asin(25.0 / w_lim);
    let closed = if arg >= std::f64::consts::FRAC_PI_2 {
        w_lim
    } else {
        w_lim * libm::sin(arg)
    };
    assert!(
        (v * v - closed).abs() < 1e-4 * (1.0 + closed),
        "{} vs {closed}",
        v * v
    );
}

#[test]
fn line_reach_is_constant_accel() {
    let k = kin(10.0, 500.0, 0.0, 0.0, f64::INFINITY);
    let v = disk_reach_v(&k, 3.0, 10.0, 1e-9).unwrap();
    assert!((v - (9.0_f64 + 2.0 * 500.0 * 10.0).sqrt()).abs() < 1e-9);
}

#[test]
fn clothoid_reach_respects_the_curvature_ceiling() {
    let k = kin(4.0, 1000.0, 0.0, 0.05, 300.0);
    let v = disk_reach_v(&k, 5.0, 4.0, 1e-9).unwrap();
    let cap_end = limit_speed(k.kappa_abs(4.0), 1000.0);
    assert!(
        v <= cap_end * (1.0 + 1e-9),
        "reach {v} above end cap {cap_end}"
    );
}

#[test]
fn reverse_reach_mirrors_the_member() {
    let k = kin(4.0, 1000.0, 0.0, 0.05, 300.0);
    let fwd = disk_reach_v(&k.reversed(), 5.0, 4.0, 1e-9).unwrap();
    let rev = disk_reach_v_rev(&k, 5.0, 4.0, 1e-9).unwrap();
    assert_eq!(fwd, rev);
}

#[test]
fn limit_speed_is_infinite_for_a_line() {
    assert_eq!(limit_speed(0.0, 1000.0), f64::INFINITY);
    assert!((limit_speed(0.02, 2000.0) - (2000.0_f64 / 0.02).sqrt()).abs() < 1e-9);
}
