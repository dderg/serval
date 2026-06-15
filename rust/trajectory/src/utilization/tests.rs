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
    assert_eq!(u.family, UtilFamily::Velocity);
    assert!(
        (u.ratio - v / 300.0).abs() < 1e-6,
        "velocity utilization must be v / v_max; got {}",
        u.ratio,
    );
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
    assert_eq!(u.family, UtilFamily::Accel);
    assert!(
        (u.ratio - 1.0).abs() < 1e-6,
        "accel utilization must be a / a_max = 1.0; got {}",
        u.ratio,
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
    assert!(
        u.ratio > 1.0,
        "an over-limit segment must report utilization > 1; got {}",
        u.ratio,
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
    assert!(
        (u.ratio - 200.0 / 300.0).abs() < 1e-6,
        "window peak must be the faster segment; got {}",
        u.ratio,
    );
}
