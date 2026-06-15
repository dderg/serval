use super::*;
use temporal::Limits;

fn limits(v: f64, a: f64, j: f64) -> Limits {
    Limits::axis_boxes([v, v, v], [a, a, a], [j, j, j])
}

fn constant(value: f64, t_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![value, value]).unwrap()
}

/// Axis 0 moves at constant velocity `v`; axes 1,2 are still. The executed
/// trajectory rides velocity at `v / v_max` and nothing else.
#[test]
fn constant_velocity_reports_velocity_family() {
    let t_end = 0.1;
    let v = 100.0;
    let ax0 = ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, v * t_end]).unwrap();
    let axes = [ax0, constant(0.0, t_end), constant(0.0, t_end)];

    let u = segment_peak_utilization(&axes, &limits(300.0, 10_000.0, 50_000.0))
        .expect("a moving segment must report utilization");
    let w = u.worst().expect("a moving segment has a worst family");
    assert_eq!(w.family, UtilFamily::Velocity);
    assert!(
        (w.ratio - v / 300.0).abs() < 1e-6,
        "velocity utilization must be v / v_max; got {}",
        w.ratio,
    );
    // per-family detail: the raw peak velocity is v, accel/jerk are ~0.
    assert!(
        (u.vel_mag - v).abs() < 1e-3,
        "raw peak velocity ≈ v; got {}",
        u.vel_mag
    );
    assert!((u.vel_ratio - v / 300.0).abs() < 1e-6);
    // accel/jerk are zero up to finite-difference floating-point noise (the jerk
    // stencil divides by dt³ ≈ 1.5e-14, so a true-zero rounds to ~6e-5 of cap) —
    // negligible against the 0.33 velocity ratio.
    assert!(u.accel_ratio < 1e-3 && u.jerk_ratio < 1e-3);
}

/// Axis 0 under constant acceleration `a` (x = ½ a t²). With caps chosen so the
/// accel ratio dominates the velocity ratio, the peak family is Accel and the
/// ratio is exact (the second difference of a quadratic is exact).
#[test]
fn constant_accel_reports_accel_family_exactly() {
    let t_end = 0.1;
    let a = 5_000.0;
    let ax0 = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, t_end, t_end, t_end],
        vec![0.0, 0.0, 0.5 * a * t_end * t_end],
    )
    .unwrap();
    let axes = [ax0, constant(0.0, t_end), constant(0.0, t_end)];

    // peak velocity is a*t_end = 500; v_max = 600 keeps velocity ratio below the
    // accel ratio of a / a_max = 1.0.
    let u = segment_peak_utilization(&axes, &limits(600.0, a, 50_000.0))
        .expect("a moving segment must report utilization");
    let w = u.worst().expect("a moving segment has a worst family");
    assert_eq!(w.family, UtilFamily::Accel);
    assert!(
        (w.ratio - 1.0).abs() < 1e-6,
        "accel utilization must be a / a_max = 1.0; got {}",
        w.ratio,
    );
    // raw peak accel is exactly a (second difference of a quadratic is exact).
    assert!(
        (u.accel_mag - a).abs() < 1e-3,
        "raw peak accel ≈ a; got {}",
        u.accel_mag
    );
}

/// A segment exceeding its accel cap reports utilization above 1 — the executed
/// trajectory is over the limit, which a feasibility check must surface.
#[test]
fn over_limit_segment_reports_ratio_above_one() {
    let t_end = 0.1;
    let a = 5_000.0;
    let ax0 = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, t_end, t_end, t_end],
        vec![0.0, 0.0, 0.5 * a * t_end * t_end],
    )
    .unwrap();
    let axes = [ax0, constant(0.0, t_end), constant(0.0, t_end)];

    let u = segment_peak_utilization(&axes, &limits(600.0, 2_500.0, 50_000.0))
        .expect("a moving segment must report utilization");
    let w = u.worst().expect("a moving segment has a worst family");
    assert!(
        w.ratio > 1.0,
        "an over-limit segment must report utilization > 1; got {}",
        w.ratio,
    );
}

/// The requested feedrate is wired as a separate full-spatial path-speed set
/// (v_max = feed) by `per_segment_limits`. A move cruising at the feed must read
/// utilization 1.0 against that set — NOT feed/machine against the per-axis box.
/// This locks that utilization is measured against the effective limit, so a move
/// running exactly at its requested feed reports as riding its limit, not as
/// leaving (machine - feed) headroom.
#[test]
fn utilization_credits_the_feedrate_path_speed_set() {
    let t_end = 0.1;
    let feed = 150.0; // half the machine cap of 300
    let ax0 =
        ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, feed * t_end]).unwrap();
    let axes = [ax0, constant(0.0, t_end), constant(0.0, t_end)];

    let lim = limits(300.0, 10_000.0, 50_000.0).with_extra_sets(&[temporal::LimitSet {
        axes: temporal::AxisSet::spatial(),
        v_max: feed,
        a_max: f64::INFINITY,
        j_max: f64::INFINITY,
    }]);

    let u = segment_peak_utilization(&axes, &lim).expect("util");
    let w = u.worst().expect("a moving segment has a worst family");
    assert_eq!(w.family, UtilFamily::Velocity);
    assert!(
        (w.ratio - 1.0).abs() < 1e-6,
        "must credit the feed set (1.0), not feed/machine (0.5); got {}",
        w.ratio,
    );
}

/// A segment too short to carry a jerk stencil yields no utilization sample.
#[test]
fn segment_too_short_is_none() {
    let t_end = 1e-5; // < 4 * MCU_DT
    let axes = [
        ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, 1.0]).unwrap(),
        constant(0.0, t_end),
        constant(0.0, t_end),
    ];
    assert!(segment_peak_utilization(&axes, &limits(300.0, 10_000.0, 50_000.0)).is_none());
}

/// The window peak is the max over its segments.
#[test]
fn window_peak_takes_the_max_segment() {
    let t_end = 0.1;
    let slow =
        ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, 50.0 * t_end]).unwrap();
    let fast =
        ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, 200.0 * t_end]).unwrap();
    let lim = limits(300.0, 10_000.0, 50_000.0);
    let seg_slow = [slow, constant(0.0, t_end), constant(0.0, t_end)];
    let seg_fast = [fast, constant(0.0, t_end), constant(0.0, t_end)];

    let u = window_peak_utilization([(seg_slow.as_slice(), &lim), (seg_fast.as_slice(), &lim)])
        .expect("window must report utilization");
    let w = u.worst().expect("window has a worst family");
    assert!(
        (w.ratio - 200.0 / 300.0).abs() < 1e-6,
        "window peak must be the faster segment; got {}",
        w.ratio,
    );
    assert!(
        (u.vel_mag - 200.0).abs() < 1e-3,
        "window raw peak velocity is the faster segment"
    );
}
