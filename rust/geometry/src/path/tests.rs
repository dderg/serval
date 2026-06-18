use super::{Arc, Clothoid, CurvatureProfile, Line, PathSegment, Segment};
use crate::{FollowerDemand, GeometryError};

const UNIT_U: [f64; 3] = [1.0, 0.0, 0.0];
const UNIT_V: [f64; 3] = [0.0, 1.0, 0.0];
const ORIGIN: [f64; 3] = [0.0, 0.0, 0.0];

fn make_line() -> Line {
    Line::try_new([0.0, 0.0, 0.0], [3.0, 4.0, 0.0]).unwrap()
}

fn make_arc(radius: f64, sweep: f64) -> Arc {
    Arc::try_new(ORIGIN, UNIT_U, UNIT_V, radius, 0.0, sweep).unwrap()
}

fn make_clothoid(kappa_0: f64, sigma: f64, length: f64) -> Clothoid {
    Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, kappa_0, sigma, length).unwrap()
}

fn sample_kappas(seg: &impl CurvatureProfile, n: usize) -> Vec<f64> {
    let l = seg.s_len();
    (0..=n)
        .map(|i| seg.kappa(l * (i as f64) / (n as f64)))
        .collect()
}

#[test]
fn ac_cp1_line_s_len_positive() {
    let line = make_line();
    assert!(line.s_len() > 0.0);
}

#[test]
fn ac_cp1_arc_s_len_positive() {
    let arc = make_arc(2.0, std::f64::consts::PI);
    assert!(arc.s_len() > 0.0);
}

#[test]
fn ac_cp1_clothoid_s_len_positive() {
    let c = make_clothoid(0.1, 0.05, 3.0);
    assert!(c.s_len() > 0.0);
}

#[test]
fn ac_cp1_degenerate_line_zero_length_fails() {
    let result = Line::try_new([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]);
    assert_eq!(result, Err(GeometryError::ZeroMotion));
}

#[test]
fn ac_cp2_line_dkappa_ds_matches_central_difference() {
    let line = make_line();
    check_dkappa_ds_central_diff(&line, 20, 1e-6);
}

#[test]
fn ac_cp2_arc_dkappa_ds_matches_central_difference() {
    let arc = make_arc(5.0, std::f64::consts::FRAC_PI_2);
    check_dkappa_ds_central_diff(&arc, 20, 1e-6);
}

#[test]
fn ac_cp2_clothoid_dkappa_ds_matches_central_difference() {
    let c = make_clothoid(0.2, 0.1, 4.0);
    check_dkappa_ds_central_diff(&c, 20, 1e-6);
}

fn check_dkappa_ds_central_diff(seg: &impl CurvatureProfile, n: usize, tol: f64) {
    let l = seg.s_len();
    let h = l * 1e-6;
    for i in 1..(n - 1) {
        let s = l * (i as f64) / (n as f64);
        let s_lo = (s - h).max(0.0);
        let s_hi = (s + h).min(l);
        let central_diff = (seg.kappa(s_hi) - seg.kappa(s_lo)) / (s_hi - s_lo);
        let reported = seg.dkappa_ds(s);
        assert!(
            (reported - central_diff).abs() < tol,
            "dkappa_ds({s}) = {reported}, central diff = {central_diff}, diff = {}",
            (reported - central_diff).abs()
        );
    }
}

#[test]
fn ac_cp3_line_kappa_endpoints_match_kappa_at_boundaries() {
    let line = make_line();
    let (k0, kl) = line.kappa_endpoints();
    assert_eq!(k0, line.kappa(0.0));
    assert_eq!(kl, line.kappa(line.s_len()));
}

#[test]
fn ac_cp3_arc_kappa_endpoints_match_kappa_at_boundaries() {
    let arc = make_arc(3.0, 1.5);
    let (k0, kl) = arc.kappa_endpoints();
    assert_eq!(k0, arc.kappa(0.0));
    assert_eq!(kl, arc.kappa(arc.s_len()));
}

#[test]
fn ac_cp3_clothoid_kappa_endpoints_match_kappa_at_boundaries() {
    let c = make_clothoid(0.1, 0.05, 5.0);
    let (k0, kl) = c.kappa_endpoints();
    assert_eq!(k0, c.kappa(0.0));
    assert_eq!(kl, c.kappa(c.s_len()));
}

#[test]
fn ac_cp4_line_kappa_peak_dominates_interior() {
    let line = make_line();
    let (_, kpeak) = line.kappa_peak();
    for k in sample_kappas(&line, 20) {
        assert!(kpeak >= k.abs(), "peak {kpeak} < interior kappa {k}");
    }
}

#[test]
fn ac_cp4_arc_kappa_peak_dominates_interior() {
    let arc = make_arc(2.0, 2.0);
    let (_, kpeak) = arc.kappa_peak();
    for k in sample_kappas(&arc, 20) {
        assert!(kpeak >= k.abs(), "peak {kpeak} < interior kappa {k}");
    }
}

#[test]
fn ac_cp4_clothoid_kappa_peak_dominates_interior() {
    let c = make_clothoid(0.3, 0.1, 4.0);
    let (_, kpeak) = c.kappa_peak();
    for k in sample_kappas(&c, 50) {
        assert!(kpeak >= k.abs(), "peak {kpeak} < interior kappa {k}");
    }
}

#[test]
fn ac_cp4_clothoid_kappa_peak_equals_max_of_endpoints() {
    let kappa_0 = 0.3_f64;
    let sigma = 0.1_f64;
    let length = 4.0_f64;
    let c = make_clothoid(kappa_0, sigma, length);
    let (_, kpeak) = c.kappa_peak();
    let expected = kappa_0.abs().max((kappa_0 + sigma * length).abs());
    assert!((kpeak - expected).abs() < f64::EPSILON * 10.0);
}

#[test]
fn clothoid_sigma_zero_kappa_uniform() {
    let kappa_0 = 0.25_f64;
    let c = make_clothoid(kappa_0, 0.0, 5.0);
    let l = c.s_len();
    for i in 0..=20 {
        let s = l * (i as f64) / 20.0;
        let k = c.kappa(s);
        assert!(
            (k - kappa_0).abs() < f64::EPSILON * 10.0,
            "kappa({s}) = {k}, expected {kappa_0}"
        );
    }
}

#[test]
fn clothoid_sigma_zero_negative_kappa_uniform() {
    let kappa_0 = -0.4_f64;
    let c = make_clothoid(kappa_0, 0.0, 3.0);
    let l = c.s_len();
    for i in 0..=20 {
        let s = l * (i as f64) / 20.0;
        let k = c.kappa(s);
        assert!((k - kappa_0).abs() < f64::EPSILON * 10.0);
    }
}

#[test]
fn line_kappa_zero_everywhere() {
    let line = make_line();
    let l = line.s_len();
    for i in 0..=10 {
        let s = l * (i as f64) / 10.0;
        assert_eq!(line.kappa(s), 0.0);
        assert_eq!(line.dkappa_ds(s), 0.0);
    }
    assert_eq!(line.kappa_peak(), (0.0, 0.0));
}

#[test]
fn arc_kappa_constant() {
    let r = 4.0_f64;
    let arc = make_arc(r, 2.0);
    let l = arc.s_len();
    let expected_k = 1.0 / r;
    for i in 0..=10 {
        let s = l * (i as f64) / 10.0;
        assert!((arc.kappa(s) - expected_k).abs() < f64::EPSILON * 10.0);
        assert_eq!(arc.dkappa_ds(s), 0.0);
    }
    let (_, kpeak) = arc.kappa_peak();
    assert!((kpeak - expected_k).abs() < f64::EPSILON * 10.0);
}

#[test]
fn arc_s_len_equals_r_times_abs_sweep() {
    let r = 5.0_f64;
    let sweep = std::f64::consts::PI * 1.5;
    let arc = make_arc(r, sweep);
    assert!((arc.s_len() - r * sweep.abs()).abs() < 1e-12);
}

#[test]
fn line_s_len_equals_euclidean_distance() {
    let start = [1.0, 2.0, 3.0];
    let end = [4.0, 6.0, 3.0];
    let line = Line::try_new(start, end).unwrap();
    let expected = 5.0_f64;
    assert!((line.s_len() - expected).abs() < 1e-12);
}

#[test]
fn degenerate_arc_zero_radius_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, 0.0, 0.0, 1.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "radius must be positive and finite"
        })
    );
}

#[test]
fn degenerate_arc_negative_radius_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, -2.0, 0.0, 1.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "radius must be positive and finite"
        })
    );
}

#[test]
fn degenerate_arc_zero_sweep_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, 1.0, 0.0, 0.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "sweep must be nonzero and finite"
        })
    );
}

#[test]
fn degenerate_arc_subnormal_product_underflows_to_zero_length_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, 1e-180, 0.0, 1e-180);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "radius * |sweep| underflows to a zero-length arc"
        })
    );
}

#[test]
fn degenerate_arc_non_orthonormal_basis_fails() {
    let u_not_unit = [2.0, 0.0, 0.0];
    let result = Arc::try_new(ORIGIN, u_not_unit, UNIT_V, 1.0, 0.0, 1.0);
    assert_eq!(
        result,
        Err(GeometryError::NonPlanarBasis {
            reason: "u and v must be orthonormal unit vectors"
        })
    );
}

#[test]
fn degenerate_arc_non_orthogonal_basis_fails() {
    let v_not_orthog = [1.0, 0.0, 0.0];
    let result = Arc::try_new(ORIGIN, UNIT_U, v_not_orthog, 1.0, 0.0, 1.0);
    assert_eq!(
        result,
        Err(GeometryError::NonPlanarBasis {
            reason: "u and v must be orthonormal unit vectors"
        })
    );
}

#[test]
fn degenerate_clothoid_zero_length_fails() {
    let result = Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, 0.1, 0.05, 0.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateClothoid {
            reason: "length must be finite and positive"
        })
    );
}

#[test]
fn degenerate_clothoid_negative_length_fails() {
    let result = Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, 0.1, 0.05, -1.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateClothoid {
            reason: "length must be finite and positive"
        })
    );
}

#[test]
fn degenerate_clothoid_nan_kappa0_fails() {
    let result = Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, f64::NAN, 0.05, 3.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateClothoid {
            reason: "kappa_0 must be finite"
        })
    );
}

#[test]
fn degenerate_clothoid_inf_sigma_fails() {
    let result = Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, 0.1, f64::INFINITY, 3.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateClothoid {
            reason: "sigma must be finite"
        })
    );
}

#[test]
fn degenerate_clothoid_non_orthonormal_basis_fails() {
    let u_bad = [0.0, 0.0, 0.0];
    let result = Clothoid::try_new(ORIGIN, u_bad, UNIT_V, 0.1, 0.05, 3.0);
    assert_eq!(
        result,
        Err(GeometryError::NonPlanarBasis {
            reason: "u and v must be orthonormal unit vectors"
        })
    );
}

#[test]
fn virtual_move_valid_with_follower() {
    let followers = vec![FollowerDemand {
        axis_index: 3,
        ratio: 0.5,
    }];
    let path = PathSegment::try_new_virtual(followers, 10.0).unwrap();
    assert!(path.virtual_path_mm.is_some());
    assert_eq!(path.s_len(), 10.0);
    assert!(!path.followers.is_empty());
}

#[test]
fn virtual_move_empty_followers_fails_zero_motion() {
    let result = PathSegment::try_new_virtual(vec![], 10.0);
    assert_eq!(result, Err(GeometryError::ZeroMotion));
}

#[test]
fn virtual_move_zero_virtual_path_fails() {
    let followers = vec![FollowerDemand {
        axis_index: 3,
        ratio: 0.5,
    }];
    let result = PathSegment::try_new_virtual(followers, 0.0);
    assert_eq!(
        result,
        Err(GeometryError::FollowerInvariantViolation {
            reason: "virtual path length must be finite and positive"
        })
    );
}

#[test]
fn virtual_move_negative_virtual_path_fails() {
    let followers = vec![FollowerDemand {
        axis_index: 3,
        ratio: 0.5,
    }];
    let result = PathSegment::try_new_virtual(followers, -5.0);
    assert_eq!(
        result,
        Err(GeometryError::FollowerInvariantViolation {
            reason: "virtual path length must be finite and positive"
        })
    );
}

#[test]
fn path_segment_try_new_line_valid() {
    let line = Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
    let seg = PathSegment::try_new(Segment::Line(line), vec![]).unwrap();
    assert!(seg.virtual_path_mm.is_none());
    assert!((seg.s_len() - 1.0).abs() < 1e-12);
}

#[test]
fn path_segment_with_followers_validates_ratio() {
    let line = Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
    let bad_followers = vec![FollowerDemand {
        axis_index: 3,
        ratio: 0.0,
    }];
    let result = PathSegment::try_new(Segment::Line(line), bad_followers);
    assert_eq!(
        result,
        Err(GeometryError::FollowerInvariantViolation {
            reason: "follower ratio must be finite and nonzero"
        })
    );
}

#[test]
fn path_segment_with_duplicate_followers_fails() {
    let line = Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
    let dup_followers = vec![
        FollowerDemand {
            axis_index: 3,
            ratio: 0.5,
        },
        FollowerDemand {
            axis_index: 3,
            ratio: -0.5,
        },
    ];
    let result = PathSegment::try_new(Segment::Line(line), dup_followers);
    assert_eq!(
        result,
        Err(GeometryError::FollowerInvariantViolation {
            reason: "duplicate follower axis"
        })
    );
}

#[test]
fn segment_enum_delegates_curvature_profile_correctly() {
    let line = Segment::Line(make_line());
    assert_eq!(line.kappa(0.0), 0.0);
    assert_eq!(line.dkappa_ds(0.0), 0.0);

    let arc = Segment::Arc(make_arc(2.0, 1.0));
    let expected_k = 0.5_f64;
    assert!((arc.kappa(0.0) - expected_k).abs() < 1e-12);
    assert_eq!(arc.dkappa_ds(0.0), 0.0);

    let clothoid = Segment::Clothoid(make_clothoid(0.1, 0.05, 3.0));
    assert!((clothoid.kappa(0.0) - 0.1).abs() < 1e-12);
    assert!((clothoid.dkappa_ds(0.0) - 0.05).abs() < 1e-12);
}

#[test]
fn clothoid_kappa_linear_in_s() {
    let kappa_0 = 0.2_f64;
    let sigma = 0.1_f64;
    let length = 5.0_f64;
    let c = make_clothoid(kappa_0, sigma, length);
    let l = c.s_len();
    for i in 0..=10 {
        let s = l * (i as f64) / 10.0;
        let expected = kappa_0 + sigma * s;
        assert!(
            (c.kappa(s) - expected).abs() < f64::EPSILON * 10.0,
            "kappa({s}) = {}, expected {expected}",
            c.kappa(s)
        );
    }
}

#[test]
fn arc_negative_sweep_gives_positive_s_len() {
    let arc = make_arc(3.0, -std::f64::consts::PI);
    assert!(arc.s_len() > 0.0);
    assert!((arc.s_len() - 3.0 * std::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn clothoid_decreasing_kappa_peak_at_start() {
    let kappa_0 = 0.5_f64;
    let sigma = -0.1_f64;
    let length = 4.0_f64;
    let c = make_clothoid(kappa_0, sigma, length);
    let (s_peak, kpeak) = c.kappa_peak();
    let expected_peak = kappa_0.abs().max((kappa_0 + sigma * length).abs());
    assert!((kpeak - expected_peak).abs() < 1e-12);
    assert_eq!(s_peak, 0.0);
    for k in sample_kappas(&c, 50) {
        assert!(kpeak >= k.abs());
    }
}

#[test]
fn clothoid_peak_at_end_when_kappa_increasing() {
    let kappa_0 = 0.1_f64;
    let sigma = 0.3_f64;
    let length = 4.0_f64;
    let c = make_clothoid(kappa_0, sigma, length);
    let (s_peak, kpeak) = c.kappa_peak();
    assert_eq!(s_peak, length);
    let expected_peak = (kappa_0 + sigma * length).abs();
    assert!((kpeak - expected_peak).abs() < 1e-12);
}

#[test]
fn arc_inf_radius_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, f64::INFINITY, 0.0, 1.0);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "radius must be positive and finite"
        })
    );
}

#[test]
fn arc_inf_sweep_fails() {
    let result = Arc::try_new(ORIGIN, UNIT_U, UNIT_V, 1.0, 0.0, f64::INFINITY);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateArc {
            reason: "sweep must be nonzero and finite"
        })
    );
}

#[test]
fn clothoid_inf_length_fails() {
    let result = Clothoid::try_new(ORIGIN, UNIT_U, UNIT_V, 0.1, 0.05, f64::INFINITY);
    assert_eq!(
        result,
        Err(GeometryError::DegenerateClothoid {
            reason: "length must be finite and positive"
        })
    );
}
