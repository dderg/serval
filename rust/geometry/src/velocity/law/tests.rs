use super::{LawSegment, ScalarLaw};

fn rail(v0: f64, accel: f64, kappa0: f64, sigma: f64, brake: bool, ds: f64) -> LawSegment {
    LawSegment::until_arc(
        0.0,
        0.0,
        v0,
        ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            brake,
        },
        ds,
    )
    .expect("test rail must not stall")
}

#[test]
fn const_accel_matches_closed_form() {
    let seg = LawSegment::new(1.0, 0.5, 3.0, 10.0, ScalarLaw::ConstAccel { a0: -4.0 });
    let (s, v, a) = seg.state_at(1.25);
    assert!((s - (3.0 + 10.0 * 0.25 - 2.0 * 0.25 * 0.25)).abs() < 1e-12);
    assert!((v - 9.0).abs() < 1e-12);
    assert_eq!(a, -4.0);
    let t = seg.time_at_distance(s).unwrap();
    assert!((t - 1.25).abs() < 1e-12);
}

#[test]
fn disk_rail_scalar_acceleration_is_exactly_the_budget() {
    // Braking into a tightening clothoid: kappa 0 -> 53.7 over 0.0292 mm at
    // A = 1000, entering at the disk-reach speed for the length.
    let seg = rail(7.3, 1000.0, 0.0, 1838.0, true, 0.0292);
    for i in 0..=1000 {
        let t = seg.dt * f64::from(i) / 1000.0;
        let (s, v, a) = seg.state_at(t);
        let kappa = (1838.0 * s).abs();
        let scalar = (a * a + (kappa * v * v) * (kappa * v * v)).sqrt();
        assert!(
            (scalar - 1000.0).abs() < 1e-5,
            "scalar accel {scalar} off the disk at t={t} (v={v}, s={s})"
        );
    }
}

#[test]
fn disk_rail_on_a_straight_degenerates_to_constant_accel() {
    let seg = rail(5.0, 1000.0, 0.0, 0.0, false, 0.1);
    let (s, v, a) = seg.state_at(seg.dt);
    assert!((a - 1000.0).abs() < 1e-9);
    assert!((s - 0.1).abs() < 1e-9);
    assert!((v - (25.0_f64 + 2.0 * 1000.0 * 0.1).sqrt()).abs() < 1e-7);
}

#[test]
fn disk_rail_time_at_distance_inverts_state() {
    let seg = rail(4.0, 1000.0, 40.0, -1300.0, false, 0.02);
    for i in 1..40 {
        let t = seg.dt * f64::from(i) / 40.0;
        let (s, _, _) = seg.state_at(t);
        let back = seg.time_at_distance(s).unwrap();
        assert!(
            (back - t).abs() < 1e-10,
            "inversion drifted: t={t} back={back}"
        );
    }
}

#[test]
fn disk_rail_matches_fine_reference_integration() {
    let seg = rail(6.0, 2000.0, 10.0, 900.0, true, 0.008);
    // Reference: RK4 at 100x the dense resolution.
    let f = |s: f64, v: f64| -> (f64, f64) {
        let kappa = (10.0_f64 + 900.0 * s).abs();
        let a_n = kappa * v * v;
        (v, -((2000.0_f64.powi(2) - a_n * a_n).max(0.0)).sqrt())
    };
    let n = 200_000;
    let h = seg.dt / n as f64;
    let (mut s, mut v) = (0.0_f64, 6.0_f64);
    for _ in 0..n {
        let (k1s, k1v) = f(s, v);
        let (k2s, k2v) = f(s + 0.5 * h * k1s, v + 0.5 * h * k1v);
        let (k3s, k3v) = f(s + 0.5 * h * k2s, v + 0.5 * h * k2v);
        let (k4s, k4v) = f(s + h * k3s, v + h * k3v);
        s += h * (k1s + 2.0 * k2s + 2.0 * k3s + k4s) / 6.0;
        v += h * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
    }
    let (s_seg, v_seg, _) = seg.state_at(seg.dt);
    assert!((s_seg - 0.008).abs() < 1e-12, "arc {s_seg} vs target 0.008");
    assert!(
        (s - 0.008).abs() < 2e-8,
        "reference arc {s} vs the segment's own duration"
    );
    assert!((v_seg - v).abs() < 1e-6, "speed {v_seg} vs reference {v}");
}

#[test]
fn min_velocity_reports_the_brake_end() {
    let seg = rail(7.0, 1000.0, 0.0, 1838.0, true, 0.024);
    let end_v = seg.end_state().1;
    assert!((seg.min_velocity() - end_v).abs() < 1e-12);
    let accel = rail(4.0, 1000.0, 50.0, -1700.0, false, 0.02);
    assert_eq!(accel.min_velocity(), 4.0);
}
