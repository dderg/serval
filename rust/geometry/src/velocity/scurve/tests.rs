use super::*;

fn simulate_distance(v_in: f64, v_out: f64, accel: f64, jerk: f64) -> f64 {
    let delta = (v_out - v_in).abs();
    let sign = (v_out - v_in).signum();
    let (t1, tc) = if delta <= accel * accel / jerk {
        ((delta / jerk).sqrt(), 0.0)
    } else {
        (accel / jerk, delta / accel - accel / jerk)
    };
    let total = 2.0 * t1 + tc;
    let steps: u32 = 400_000;
    let dt = total / f64::from(steps);
    let mut v = v_in;
    let mut s = 0.0;
    for i in 0..steps {
        let t = (f64::from(i) + 0.5) * dt;
        let a_mag = if t < t1 {
            jerk * t
        } else if t < t1 + tc {
            jerk * t1
        } else {
            jerk * (total - t)
        };
        v += sign * a_mag * dt;
        s += v * dt;
    }
    s
}

fn analytic_terminal_state(v0: f64, a0: f64, ds: f64, accel_max: f64, jerk_max: f64) -> (f64, f64) {
    if jerk_max == f64::INFINITY || a0 == accel_max {
        let v1 = (v0 * v0 + 2.0 * accel_max * ds).sqrt();
        return (v1, accel_max);
    }

    let t_jup = (accel_max - a0) / jerk_max;
    let s_jup = v0 * t_jup + 0.5 * a0 * t_jup * t_jup + (1.0 / 6.0) * jerk_max * t_jup.powi(3);

    if s_jup >= ds {
        let t = solve_jerkup_cubic(v0, a0, jerk_max, ds, t_jup);
        let v1 = v0 + a0 * t + 0.5 * jerk_max * t * t;
        let a1 = a0 + jerk_max * t;
        (v1, a1)
    } else {
        let v_j = v0 + a0 * t_jup + 0.5 * jerk_max * t_jup * t_jup;
        let d_hold = ds - s_jup;
        let v1 = (v_j * v_j + 2.0 * accel_max * d_hold).sqrt();
        (v1, accel_max)
    }
}

struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    fn next_f64_range(&mut self, lo: f64, hi: f64) -> f64 {
        let bits = self.next_u64() >> 11;
        let unit = bits as f64 / (1u64 << 53) as f64;
        lo + unit * (hi - lo)
    }
}

#[test]
fn reach_inverts_distance_triangular() {
    let (v0, a, j) = (0.0, 1000.0, 100_000.0);
    let length = 0.05;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) <= a * a / j);
    assert!((simulate_distance(v0, v1, a, j) - length).abs() < 1e-3 * length);
}

#[test]
fn reach_inverts_distance_trapezoidal() {
    let (v0, a, j) = (30.0, 2000.0, 80_000.0);
    let length = 12.0;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) > a * a / j);
    assert!((simulate_distance(v0, v1, a, j) - length).abs() < 1e-3 * length);
}

#[test]
fn reach_reduces_to_constant_accel_when_jerk_infinite() {
    let (v0, a) = (40.0, 1500.0);
    for &length in &[0.01, 1.0, 50.0] {
        let v1 = max_reachable_velocity(v0, length, a, f64::INFINITY);
        let expected = (v0 * v0 + 2.0 * a * length).sqrt();
        assert!((v1 - expected).abs() < 1e-9 * expected);
    }
}

#[test]
fn reach_from_rest_is_finite_and_positive() {
    let v1 = max_reachable_velocity(0.0, 0.02, 1000.0, 50_000.0);
    assert!(v1 > 0.0 && v1.is_finite());
}

#[test]
fn reach_is_monotone_in_length_and_jerk() {
    let (v0, a) = (10.0, 1000.0);
    let longer = max_reachable_velocity(v0, 5.0, a, 100_000.0);
    let shorter = max_reachable_velocity(v0, 1.0, a, 100_000.0);
    assert!(longer > shorter);
    let stiffer = max_reachable_velocity(v0, 5.0, a, 1_000_000.0);
    assert!(stiffer >= longer - 1e-9);
}

#[test]
fn reach_with_accel_invalid_input_v0_negative() {
    assert_eq!(
        reach_velocity_with_accel(-1.0, 0.0, 1.0, 1000.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_a0_exceeds_accel_max() {
    assert_eq!(
        reach_velocity_with_accel(10.0, 1001.0, 1.0, 1000.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_a0_below_neg_accel_max() {
    assert_eq!(
        reach_velocity_with_accel(10.0, -1001.0, 1.0, 1000.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_ds_negative() {
    assert_eq!(
        reach_velocity_with_accel(10.0, 0.0, -1.0, 1000.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_accel_max_zero() {
    assert_eq!(
        reach_velocity_with_accel(10.0, 0.0, 1.0, 0.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_jerk_negative() {
    assert_eq!(
        reach_velocity_with_accel(10.0, 0.0, 1.0, 1000.0, -1.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_invalid_input_nonfinite_v0() {
    assert_eq!(
        reach_velocity_with_accel(f64::NAN, 0.0, 1.0, 1000.0, 50_000.0),
        Err(ReachError::InvalidInput)
    );
}

#[test]
fn reach_with_accel_ds_zero_returns_initial_state() {
    let (v1, a1) = reach_velocity_with_accel(30.0, 500.0, 0.0, 1000.0, 50_000.0).unwrap();
    assert_eq!(v1, 30.0);
    assert_eq!(a1, 500.0);
}

#[test]
fn reach_with_accel_at_accel_max_no_jerkup() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, 1000.0, 5.0, 1000.0, 50_000.0);
    let (v1, a1) = reach_velocity_with_accel(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert!((a1 - accel_max).abs() < 1e-9);
    let expected_v1 = (v0 * v0 + 2.0 * accel_max * ds).sqrt();
    assert!((v1 - expected_v1).abs() < 1e-9 * expected_v1);
}

#[test]
fn reach_with_accel_jerk_infinite_degrades_to_constant_accel() {
    let (v0, a0, ds, accel_max) = (20.0, 0.0, 3.0, 500.0);
    let (v1, a1) = reach_velocity_with_accel(v0, a0, ds, accel_max, f64::INFINITY).unwrap();
    let expected_v1 = (v0 * v0 + 2.0 * accel_max * ds).sqrt();
    assert!((v1 - expected_v1).abs() < 1e-9 * expected_v1);
    assert!((a1 - accel_max).abs() < 1e-9);
}

#[test]
fn reach_with_accel_jerk_infinite_negative_a0_degrades_to_constant_accel() {
    let (v0, a0, ds, accel_max) = (20.0, -500.0, 3.0, 500.0);
    let (v1, a1) = reach_velocity_with_accel(v0, a0, ds, accel_max, f64::INFINITY).unwrap();
    let expected_v1 = (v0 * v0 + 2.0 * accel_max * ds).sqrt();
    assert!((v1 - expected_v1).abs() < 1e-9 * expected_v1);
    assert!((a1 - accel_max).abs() < 1e-9);
}

#[test]
fn reach_with_accel_monotone_in_ds() {
    let (v0, a0, accel_max, jerk_max) = (10.0, 0.0, 1000.0, 50_000.0);
    let (v_short, _) = reach_velocity_with_accel(v0, a0, 0.5, accel_max, jerk_max).unwrap();
    let (v_long, _) = reach_velocity_with_accel(v0, a0, 5.0, accel_max, jerk_max).unwrap();
    assert!(v_long > v_short);
}

#[test]
fn reach_with_accel_monotone_in_ds_negative_a0() {
    let (v0, a0, accel_max, jerk_max) = (10.0, -500.0, 1000.0, 50_000.0);
    let (v_short, _) = reach_velocity_with_accel(v0, a0, 0.5, accel_max, jerk_max).unwrap();
    let (v_long, _) = reach_velocity_with_accel(v0, a0, 5.0, accel_max, jerk_max).unwrap();
    assert!(v_long > v_short);
}

fn assert_reach_matches_analytic(v0: f64, a0: f64, ds: f64, accel_max: f64, jerk_max: f64) {
    let (v1_fn, a1_fn) = reach_velocity_with_accel(v0, a0, ds, accel_max, jerk_max).unwrap();
    let (v1_ref, a1_ref) = analytic_terminal_state(v0, a0, ds, accel_max, jerk_max);

    let v_tol = 1e-9_f64.max(1e-9 * v1_ref.abs());
    let a_tol = 1e-9_f64.max(1e-9 * a1_ref.abs());

    assert!(
        (v1_fn - v1_ref).abs() <= v_tol,
        "v1 mismatch: v0={v0}, a0={a0}, ds={ds}: got {v1_fn}, ref {v1_ref}, diff {}",
        (v1_fn - v1_ref).abs()
    );
    assert!(
        (a1_fn - a1_ref).abs() <= a_tol,
        "a1 mismatch: v0={v0}, a0={a0}, ds={ds}: got {a1_fn}, ref {a1_ref}, diff {}",
        (a1_fn - a1_ref).abs()
    );
}

#[test]
fn ac_s1_reach_matches_analytic_a0_zero() {
    assert_reach_matches_analytic(20.0, 0.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_a0_positive_partial_jerkup() {
    assert_reach_matches_analytic(20.0, 300.0, 0.01, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_a0_positive_full_jerkup() {
    assert_reach_matches_analytic(20.0, 300.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_a0_at_accel_max() {
    assert_reach_matches_analytic(20.0, 1000.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_a0_negative() {
    assert_reach_matches_analytic(20.0, -500.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_a0_at_neg_accel_max() {
    assert_reach_matches_analytic(20.0, -1000.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s1_reach_matches_analytic_randomized() {
    let mut lcg = Lcg::new(0xdeadbeef_cafebabe);
    let accel_max = 1000.0;
    let jerk_max = 50_000.0;

    for _ in 0..50 {
        let v0 = lcg.next_f64_range(0.1, 200.0);
        let a0 = lcg.next_f64_range(-accel_max, accel_max);
        let ds = lcg.next_f64_range(0.001, 20.0);
        assert_reach_matches_analytic(v0, a0, ds, accel_max, jerk_max);
    }
}

fn assert_breakpoints_consistent(v0: f64, a0: f64, ds: f64, accel_max: f64, jerk_max: f64) {
    let (v1_fn, a1_fn) = reach_velocity_with_accel(v0, a0, ds, accel_max, jerk_max).unwrap();
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();

    let v1_seg = velocity_at(&seg, ds);
    let a1_seg = accel_at(&seg, ds);

    let v_tol = 1e-9_f64.max(1e-9 * v1_fn.abs());
    let a_tol = 1e-9_f64.max(1e-9 * a1_fn.abs());

    assert!(
        (v1_seg - v1_fn).abs() <= v_tol,
        "velocity_at(ds) mismatch: v0={v0}, a0={a0}, ds={ds}: seg={v1_seg}, reach={v1_fn}"
    );
    assert!(
        (a1_seg - a1_fn).abs() <= a_tol,
        "accel_at(ds) mismatch: v0={v0}, a0={a0}, ds={ds}: seg={a1_seg}, reach={a1_fn}"
    );
}

#[test]
fn ac_s2_breakpoints_consistent_a0_zero() {
    assert_breakpoints_consistent(20.0, 0.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s2_breakpoints_consistent_a0_positive_partial() {
    assert_breakpoints_consistent(20.0, 300.0, 0.01, 1000.0, 50_000.0);
}

#[test]
fn ac_s2_breakpoints_consistent_a0_at_accel_max() {
    assert_breakpoints_consistent(20.0, 1000.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s2_breakpoints_consistent_a0_negative() {
    assert_breakpoints_consistent(20.0, -500.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s2_breakpoints_consistent_randomized() {
    let mut lcg = Lcg::new(0x1234_5678_abcd_ef01);
    let accel_max = 1000.0;
    let jerk_max = 50_000.0;

    for _ in 0..30 {
        let v0 = lcg.next_f64_range(0.1, 200.0);
        let a0 = lcg.next_f64_range(-accel_max, accel_max);
        let ds = lcg.next_f64_range(0.001, 20.0);
        assert_breakpoints_consistent(v0, a0, ds, accel_max, jerk_max);
    }
}

fn assert_limits_satisfied(v0: f64, a0: f64, ds: f64, accel_max: f64, jerk_max: f64) {
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    let steps = 10_000_u32;

    for i in 0..=steps {
        let s = ds * f64::from(i) / f64::from(steps);
        let a = accel_at(&seg, s);
        assert!(
            a.abs() <= accel_max + 1e-9,
            "accel_at({s}) = {a} exceeds accel_max={accel_max}: v0={v0}, a0={a0}"
        );

        if jerk_max < f64::INFINITY && i > 0 {
            let s_prev = ds * f64::from(i - 1) / f64::from(steps);
            let a_prev = accel_at(&seg, s_prev);
            let ds_step = s - s_prev;
            let v_mid = velocity_at(&seg, 0.5 * (s + s_prev));
            if v_mid > 1e-10 {
                let dt = ds_step / v_mid;
                if dt > 0.0 {
                    let jerk_est = (a - a_prev) / dt;
                    assert!(
                        jerk_est.abs() <= jerk_max + 1e-6 * jerk_max,
                        "jerk estimate {jerk_est} exceeds jerk_max={jerk_max} at s={s}: v0={v0}, a0={a0}"
                    );
                }
            }
        }
    }
}

#[test]
fn ac_s3_limits_satisfied_a0_zero() {
    assert_limits_satisfied(20.0, 0.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s3_limits_satisfied_a0_negative() {
    assert_limits_satisfied(20.0, -500.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s3_limits_satisfied_a0_at_accel_max() {
    assert_limits_satisfied(20.0, 1000.0, 5.0, 1000.0, 50_000.0);
}

#[test]
fn ac_s3_limits_satisfied_partial_jerkup() {
    assert_limits_satisfied(20.0, 300.0, 0.01, 1000.0, 50_000.0);
}

#[test]
fn ac_s3_limits_satisfied_randomized() {
    let mut lcg = Lcg::new(0xfeed_face_dead_beef);
    let accel_max = 1000.0;
    let jerk_max = 50_000.0;

    for _ in 0..20 {
        let v0 = lcg.next_f64_range(0.1, 200.0);
        let a0 = lcg.next_f64_range(-accel_max, accel_max);
        let ds = lcg.next_f64_range(0.001, 20.0);
        assert_limits_satisfied(v0, a0, ds, accel_max, jerk_max);
    }
}

#[test]
fn ac_s4_branch_a0_zero() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, 0.0, 5.0, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert!(seg.s_jup_end > 0.0);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_branch_a0_positive_below_accel_max() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, 300.0, 5.0, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert!(seg.s_jup_end > 0.0);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_branch_a0_at_accel_max_zero_jerkup() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, 1000.0, 5.0, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert_eq!(seg.s_jup_end, 0.0);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_branch_a0_negative() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, -500.0, 5.0, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert!(seg.s_jup_end > 0.0);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_branch_a0_at_neg_accel_max() {
    let (v0, a0, ds, accel_max, jerk_max) = (10.0, -1000.0, 5.0, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert!(seg.s_jup_end > 0.0);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_branch_partial_jerkup() {
    let (v0, a0, ds, accel_max, jerk_max) = (100.0, 300.0, 0.001, 1000.0, 50_000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
    assert_eq!(seg.s_jup_end, ds);
    assert_eq!(seg.s_hold_end, ds);
}

#[test]
fn ac_s4_invalid_a0_above_accel_max_returns_err() {
    assert!(breakpoints(10.0, 1001.0, 5.0, 1000.0, 50_000.0).is_err());
}

#[test]
fn ac_s4_jerk_infinite_branch() {
    let (v0, a0, ds, accel_max) = (10.0, 0.0, 5.0, 1000.0);
    let seg = breakpoints(v0, a0, ds, accel_max, f64::INFINITY).unwrap();
    assert_eq!(seg.s_jup_end, 0.0);
    assert_eq!(seg.s_hold_end, ds);
    let v1 = velocity_at(&seg, ds);
    let expected = (v0 * v0 + 2.0 * accel_max * ds).sqrt();
    assert!((v1 - expected).abs() < 1e-9 * expected);
}

#[test]
fn velocity_at_boundary_matches_reach() {
    let cases = [
        (0.0_f64, 0.0_f64, 1.0_f64),
        (10.0, 500.0, 5.0),
        (10.0, -500.0, 5.0),
        (10.0, 1000.0, 5.0),
        (50.0, 0.0, 0.005),
    ];
    let (accel_max, jerk_max) = (1000.0, 50_000.0);

    for (v0, a0, ds) in cases {
        let (v1_reach, a1_reach) =
            reach_velocity_with_accel(v0, a0, ds, accel_max, jerk_max).unwrap();
        let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
        let v1_seg = velocity_at(&seg, ds);
        let a1_seg = accel_at(&seg, ds);

        assert!(
            (v1_seg - v1_reach).abs() < 1e-9 * (1.0 + v1_reach),
            "velocity mismatch v0={v0}, a0={a0}, ds={ds}: {v1_seg} vs {v1_reach}"
        );
        assert!(
            (a1_seg - a1_reach).abs() < 1e-9 * (1.0 + a1_reach),
            "accel mismatch v0={v0}, a0={a0}, ds={ds}: {a1_seg} vs {a1_reach}"
        );
    }
}

#[test]
fn velocity_at_origin_is_v0() {
    let cases = [(10.0, 0.0), (20.0, 500.0), (30.0, -300.0), (0.0, 0.0)];
    let (accel_max, jerk_max, ds) = (1000.0, 50_000.0, 5.0);

    for (v0, a0) in cases {
        let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
        let v_origin = velocity_at(&seg, 0.0);
        assert!(
            (v_origin - v0).abs() < 1e-12,
            "velocity_at(0) = {v_origin} != v0 = {v0}"
        );
    }
}

#[test]
fn accel_at_origin_is_a0() {
    let cases = [(10.0, 0.0), (20.0, 500.0), (30.0, -300.0)];
    let (accel_max, jerk_max, ds) = (1000.0, 50_000.0, 5.0);

    for (v0, a0) in cases {
        let seg = breakpoints(v0, a0, ds, accel_max, jerk_max).unwrap();
        let a_origin = accel_at(&seg, 0.0);
        assert!(
            (a_origin - a0).abs() < 1e-12,
            "accel_at(0) = {a_origin} != a0 = {a0}"
        );
    }
}
