use super::PositionProfile;
use super::fresnel::clothoid_offset;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};

const UNIT_U: [f64; 3] = [1.0, 0.0, 0.0];
const UNIT_V: [f64; 3] = [0.0, 1.0, 0.0];
const ORIGIN: [f64; 3] = [0.0, 0.0, 0.0];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn numeric_kappa(profile: &impl PositionProfile, s: f64, h: f64) -> f64 {
    let p_plus = profile.point_at(s + h);
    let p = profile.point_at(s);
    let p_minus = profile.point_at(s - h);
    let d1 = [
        (p_plus[0] - p_minus[0]) / (2.0 * h),
        (p_plus[1] - p_minus[1]) / (2.0 * h),
        (p_plus[2] - p_minus[2]) / (2.0 * h),
    ];
    let d2 = [
        (p_plus[0] - 2.0 * p[0] + p_minus[0]) / (h * h),
        (p_plus[1] - 2.0 * p[1] + p_minus[1]) / (h * h),
        (p_plus[2] - 2.0 * p[2] + p_minus[2]) / (h * h),
    ];
    norm(cross(d1, d2)) / norm(d1).powi(3)
}

fn assert_seam_kappa<P>(geom: &P, tol: f64)
where
    P: PositionProfile + CurvatureProfile,
{
    let l = geom.s_len();
    let h = 1e-3 * l;
    for i in 1..20 {
        let s = l * (i as f64) / 20.0;
        let analytic = geom.kappa(s).abs();
        let numeric = numeric_kappa(geom, s, h);
        assert!(
            (analytic - numeric).abs() <= tol,
            "kappa mismatch at s={s}: analytic={analytic}, numeric={numeric}"
        );
    }
}

fn gauss_legendre_offset(kappa_0: f64, sigma: f64, s: f64) -> (f64, f64) {
    let a = (5.0 - 2.0 * (10.0 / 7.0_f64).sqrt()).sqrt() / 3.0;
    let b = (5.0 + 2.0 * (10.0 / 7.0_f64).sqrt()).sqrt() / 3.0;
    let nodes = [-b, -a, 0.0, a, b];
    let w_a = (322.0 + 13.0 * 70.0_f64.sqrt()) / 900.0;
    let w_b = (322.0 - 13.0 * 70.0_f64.sqrt()) / 900.0;
    let weights = [w_b, w_a, 128.0 / 225.0, w_a, w_b];

    let phi = |t: f64| kappa_0 * t + 0.5 * sigma * t * t;
    let m = 2000;
    let h = s / (m as f64);
    let half = h * 0.5;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for j in 0..m {
        let mid = (j as f64 + 0.5) * h;
        for k in 0..5 {
            let t = mid + half * nodes[k];
            let w = weights[k] * half;
            cx += w * libm::cos(phi(t));
            cy += w * libm::sin(phi(t));
        }
    }
    (cx, cy)
}

fn make_line() -> Line {
    Line::try_new([1.0, 2.0, 3.0], [11.0, 2.0, 3.0]).unwrap()
}

fn make_arc(radius: f64, sweep: f64) -> Arc {
    Arc::try_new(ORIGIN, UNIT_U, UNIT_V, radius, 0.3, sweep).unwrap()
}

fn make_clothoid(kappa_0: f64, sigma: f64, length: f64) -> Clothoid {
    Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, kappa_0, sigma, length).unwrap()
}

#[test]
fn ac_seam1_line_kappa_from_position_is_zero() {
    assert_seam_kappa(&make_line(), 1e-5);
}

#[test]
fn ac_seam1_arc_kappa_from_position_matches_analytic() {
    assert_seam_kappa(&make_arc(2.0, std::f64::consts::PI), 1e-5);
}

#[test]
fn ac_seam1_clothoid_kappa_from_position_matches_analytic() {
    assert_seam_kappa(&make_clothoid(0.1, 0.05, 3.0), 1e-4);
    assert_seam_kappa(&make_clothoid(0.0, 0.5, 2.0), 1e-4);
    assert_seam_kappa(&make_clothoid(-0.2, 0.3, 2.5), 1e-4);
}

fn numeric_signed_kappa_xy(profile: &impl PositionProfile, s: f64, h: f64) -> f64 {
    let p_plus = profile.point_at(s + h);
    let p = profile.point_at(s);
    let p_minus = profile.point_at(s - h);
    let d1 = [
        (p_plus[0] - p_minus[0]) / (2.0 * h),
        (p_plus[1] - p_minus[1]) / (2.0 * h),
    ];
    let d2 = [
        (p_plus[0] - 2.0 * p[0] + p_minus[0]) / (h * h),
        (p_plus[1] - 2.0 * p[1] + p_minus[1]) / (h * h),
    ];
    let speed = (d1[0] * d1[0] + d1[1] * d1[1]).sqrt();
    (d1[0] * d2[1] - d1[1] * d2[0]) / speed.powi(3)
}

#[test]
fn ac_seam1_clothoid_signed_curvature_matches_bend_direction() {
    for clothoid in [
        make_clothoid(0.1, 0.05, 3.0),
        make_clothoid(-0.2, 0.3, 2.5),
        make_clothoid(0.3, -0.4, 2.0),
    ] {
        let l = clothoid.s_len();
        let h = 1e-3 * l;
        for i in 1..20 {
            let s = l * (i as f64) / 20.0;
            let analytic = clothoid.kappa(s);
            let numeric = numeric_signed_kappa_xy(&clothoid, s, h);
            assert!(
                (analytic - numeric).abs() <= 1e-4,
                "signed kappa mismatch at s={s}: analytic={analytic}, numeric={numeric}"
            );
        }
    }
}

#[test]
fn ac_pos1_endpoints_match_anchors() {
    let line = make_line();
    assert_eq!(line.point_at(0.0), [1.0, 2.0, 3.0]);
    let end = line.point_at(line.s_len());
    for i in 0..3 {
        assert!((end[i] - line.end[i]).abs() < 1e-12);
    }

    let arc = make_arc(2.0, std::f64::consts::FRAC_PI_2);
    let start = arc.point_at(0.0);
    let expected_start = [2.0 * libm::cos(0.3_f64), 2.0 * libm::sin(0.3_f64), 0.0];
    for i in 0..3 {
        assert!((start[i] - expected_start[i]).abs() < 1e-12);
    }
    let theta_end = 0.3 + std::f64::consts::FRAC_PI_2;
    let expected_end = [2.0 * libm::cos(theta_end), 2.0 * libm::sin(theta_end), 0.0];
    let end = arc.point_at(arc.s_len());
    for i in 0..3 {
        assert!((end[i] - expected_end[i]).abs() < 1e-12);
    }

    let clothoid = make_clothoid(0.1, 0.05, 3.0);
    assert_eq!(clothoid.point_at(0.0), ORIGIN);
}

#[test]
fn ac_pos2_heading_is_unit_and_matches_position_derivative() {
    let geoms: Vec<Box<dyn PositionProfile>> = vec![
        Box::new(make_line()),
        Box::new(make_arc(3.0, -1.2)),
        Box::new(make_clothoid(0.1, 0.05, 3.0)),
        Box::new(make_clothoid(0.0, 0.5, 2.0)),
    ];
    let lengths = [make_line().s_len(), make_arc(3.0, -1.2).s_len(), 3.0, 2.0];
    for (geom, l) in geoms.iter().zip(lengths) {
        let h = 1e-4 * l;
        for i in 1..10 {
            let s = l * (i as f64) / 10.0;
            let heading = geom.heading_at(s);
            assert!((norm(heading) - 1.0).abs() < 1e-9, "heading not unit");
            let d1 = sub(geom.point_at(s + h), geom.point_at(s - h));
            let d1 = [d1[0] / norm(d1), d1[1] / norm(d1), d1[2] / norm(d1)];
            for j in 0..3 {
                assert!(
                    (heading[j] - d1[j]).abs() < 1e-6,
                    "heading vs derivative mismatch"
                );
            }
        }
    }
}

fn numeric_d2heading_ds2(profile: &dyn PositionProfile, s: f64, h: f64) -> [f64; 3] {
    let plus = profile.dheading_ds(s + h);
    let minus = profile.dheading_ds(s - h);
    [
        (plus[0] - minus[0]) / (2.0 * h),
        (plus[1] - minus[1]) / (2.0 * h),
        (plus[2] - minus[2]) / (2.0 * h),
    ]
}

#[test]
fn ac_pos3_d2heading_matches_numeric_derivative_of_dheading() {
    let cases: Vec<(Box<dyn PositionProfile>, f64)> = vec![
        (Box::new(make_line()), make_line().s_len()),
        (Box::new(make_arc(3.0, -1.2)), make_arc(3.0, -1.2).s_len()),
        (Box::new(make_arc(0.7, 2.4)), make_arc(0.7, 2.4).s_len()),
        (Box::new(make_clothoid(0.1, 0.05, 3.0)), 3.0),
        (Box::new(make_clothoid(0.0, 0.5, 2.0)), 2.0),
        (Box::new(make_clothoid(-0.6, -0.4, 2.5)), 2.5),
    ];
    for (geom, l) in cases.iter() {
        let h = 1e-4 * l;
        for i in 1..10 {
            let s = l * (i as f64) / 10.0;
            let analytic = geom.d2heading_ds2(s);
            let numeric = numeric_d2heading_ds2(geom.as_ref(), s, h);
            for j in 0..3 {
                assert!(
                    (analytic[j] - numeric[j]).abs() < 1e-6,
                    "d2heading_ds2 mismatch at s={s} axis {j}: analytic={} numeric={}",
                    analytic[j],
                    numeric[j]
                );
            }
        }
    }
}

#[test]
fn ac_pos3_line_d2heading_is_zero() {
    let line = make_line();
    for i in 0..5 {
        assert_eq!(
            line.d2heading_ds2(line.s_len() * i as f64 / 4.0),
            [0.0, 0.0, 0.0]
        );
    }
}

#[test]
fn ac_pos3_arc_d2heading_is_minus_kappa_squared_heading() {
    for sweep in [-1.2, 2.4] {
        let arc = make_arc(1.7, sweep);
        let kappa2 = 1.0 / (1.7 * 1.7);
        for i in 1..8 {
            let s = arc.s_len() * i as f64 / 8.0;
            let heading = arc.heading_at(s);
            let analytic = arc.d2heading_ds2(s);
            for j in 0..3 {
                assert!(
                    (analytic[j] + kappa2 * heading[j]).abs() < 1e-12,
                    "arc d2heading is not -kappa^2*heading at s={s} axis {j}"
                );
            }
        }
    }
}

#[test]
fn ac_pos3_clothoid_d2heading_splits_into_sigma_normal_and_kappa_squared_heading() {
    let (kappa_0, sigma, length) = (0.2, 0.35, 3.0);
    let clothoid = make_clothoid(kappa_0, sigma, length);
    for i in 0..=8 {
        let s = length * i as f64 / 8.0;
        let kappa = kappa_0 + sigma * s;
        let heading = clothoid.heading_at(s);
        let normal = [-heading[1], heading[0], 0.0];
        let analytic = clothoid.d2heading_ds2(s);
        for j in 0..3 {
            let expected = sigma * normal[j] - kappa * kappa * heading[j];
            assert!(
                (analytic[j] - expected).abs() < 1e-12,
                "clothoid d2heading mismatch at s={s} axis {j}: {} vs {expected}",
                analytic[j]
            );
        }
    }
}

#[test]
fn ac_fres1_clothoid_offset_matches_quadrature() {
    let cases = [
        (0.1, 0.05, 3.0),
        (0.0, 0.5, 2.0),
        (-0.2, 0.3, 2.5),
        (0.4, 0.0, 1.5),
        (0.0, 0.0, 2.0),
        (0.0, std::f64::consts::PI, 4.0),
        (1.0, 0.5, 5.0),
        (0.0, 2.0, 6.0),
    ];
    for (kappa_0, sigma, s) in cases {
        let (cx, cy) = clothoid_offset(kappa_0, sigma, s);
        let (rx, ry) = gauss_legendre_offset(kappa_0, sigma, s);
        assert!(
            (cx - rx).abs() < 1e-9 && (cy - ry).abs() < 1e-9,
            "offset mismatch k0={kappa_0} sigma={sigma}: ({cx},{cy}) vs ({rx},{ry})"
        );
    }
}

/// A clothoid whose curvature barely moves sits `κ₀/σ` away from the spiral
/// centre. Completing the square there loses the whole segment inside that
/// offset: at `σ = 5e-17` the Fresnel difference used to report `cx = 0.4905`
/// for a 0.3 mm segment whose true reach is 0.2998 mm.
#[test]
fn ac_fres1_near_arc_clothoid_offset_survives_the_spiral_offset() {
    let (kappa_0, s) = (0.2, 0.3);
    for sigma in [5e-17, -5e-17, 1e-14, -1e-12, 6.67e-9, -6.67e-7] {
        let (cx, cy) = clothoid_offset(kappa_0, sigma, s);
        let (rx, ry) = gauss_legendre_offset(kappa_0, sigma, s);
        assert!(
            (cx - rx).abs() < 1e-14 * s && (cy - ry).abs() < 1e-14 * s,
            "offset mismatch sigma={sigma}: ({cx},{cy}) vs ({rx},{ry})"
        );
    }
}

/// The offset is a smooth function of `σ`, so a curvature rate that is pure
/// float residue must land on the circular arc it is a residue of.
#[test]
fn ac_fres1_clothoid_offset_is_continuous_as_sigma_vanishes() {
    let (kappa_0, s) = (0.2, 0.3);
    let (arc_x, arc_y) = clothoid_offset(kappa_0, 0.0, s);
    for sigma in [1e-18, -1e-18, 1e-16, -1e-16, 1e-13, -1e-13] {
        let (cx, cy) = clothoid_offset(kappa_0, sigma, s);
        assert!(
            (cx - arc_x).abs() < 1e-12 * s && (cy - arc_y).abs() < 1e-12 * s,
            "sigma={sigma} left the arc: ({cx},{cy}) vs ({arc_x},{arc_y})"
        );
    }
}

/// The sagitta of a barely-turning arc is `κ₀·s²/2`; taking it through
/// `1 − cos` throws away every digit the cancellation eats.
#[test]
fn ac_fres1_constant_curvature_sagitta_survives_a_vanishing_turn() {
    let s = 1.0;
    for turn in [1e-6, 1e-7, 1e-8, 1e-9] {
        let kappa_0 = turn / s;
        let (_, cy) = clothoid_offset(kappa_0, 0.0, s);
        let sagitta = 0.5 * kappa_0 * s * s;
        assert!(
            (cy - sagitta).abs() < 1e-12 * sagitta,
            "turn={turn}: sagitta {cy} vs {sagitta}"
        );
    }
}
