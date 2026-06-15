use super::*;
use temporal::Limits;

fn limits(v: f64, a: f64, j: f64) -> Limits {
    Limits::axis_boxes([v, v, v], [a, a, a], [j, j, j])
}

fn constant(value: f64, t_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![value, value]).unwrap()
}

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
    assert!(
        (u.vel_mag - v).abs() < 1e-3,
        "raw peak velocity ≈ v; got {}",
        u.vel_mag
    );
    assert!((u.vel_ratio - v / 300.0).abs() < 1e-6);
    assert!(u.accel_ratio < 1e-3 && u.jerk_ratio < 1e-3);
}

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

    let u = segment_peak_utilization(&axes, &limits(600.0, a, 50_000.0))
        .expect("a moving segment must report utilization");
    let w = u.worst().expect("a moving segment has a worst family");
    assert_eq!(w.family, UtilFamily::Accel);
    assert!(
        (w.ratio - 1.0).abs() < 1e-6,
        "accel utilization must be a / a_max = 1.0; got {}",
        w.ratio,
    );
    assert!(
        (u.accel_mag - a).abs() < 1e-3,
        "raw peak accel ≈ a; got {}",
        u.accel_mag
    );
}

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

#[test]
fn utilization_credits_the_feedrate_path_speed_set() {
    let t_end = 0.1;
    let feed = 150.0;
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

#[test]
fn segment_too_short_is_none() {
    let t_end = 1e-5;
    let axes = [
        ScalarNurbs::try_new(1, vec![0.0, 0.0, t_end, t_end], vec![0.0, 1.0]).unwrap(),
        constant(0.0, t_end),
        constant(0.0, t_end),
    ];
    assert!(segment_peak_utilization(&axes, &limits(300.0, 10_000.0, 50_000.0)).is_none());
}

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
