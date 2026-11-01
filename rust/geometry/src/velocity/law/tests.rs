use std::sync::Arc;

use super::{
    LawSegment, RAIL_KNOT_DS, RAIL_KNOTS_MAX, RAIL_KNOTS_MIN, RailKnot, ScalarLaw, knot_duration,
};

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
fn a_rail_entered_just_above_rest_keeps_its_time_and_its_arc() {
    let accel = 1000.0;
    let kappa = 10.0;
    let ds = 0.0405;
    let rest_start = rail(0.0, accel, kappa, 0.0, false, ds);
    let creeping_start = rail(0.1, accel, kappa, 0.0, false, ds);
    assert!(
        creeping_start.dt < rest_start.dt,
        "entering faster must not take longer: {} vs {}",
        creeping_start.dt,
        rest_start.dt
    );
    for seg in [&rest_start, &creeping_start] {
        let mut previous_s = seg.s0;
        for i in 0..=4096 {
            let t = seg.dt * f64::from(i) / 4096.0;
            let (s, v, a) = seg.state_at(t);
            assert!(
                s >= previous_s && s <= seg.s0 + ds,
                "arc {s} left [0, {ds}] at t={t}"
            );
            let load = (a * a + (kappa * v * v) * (kappa * v * v)).sqrt();
            assert!(
                (load - accel).abs() / accel < 1e-9,
                "load {load} off the disk at t={t} (v={v}, s={s})"
            );
            previous_s = s;
        }
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
fn a_fast_straight_rail_holds_its_budget_to_float_precision() {
    let accel = 264.199_103_200_885_3;
    let speed = 434.055_216_353_112_1;
    let ds = 0.311_185_974_435_605_3;
    let straight = |brake| ScalarLaw::DiskRail {
        accel,
        kappa0: 0.0,
        sigma: 0.0,
        brake,
    };
    let launch = LawSegment::until_arc(0.0, 0.0, speed, straight(false), ds).unwrap();
    let (landing, _) = LawSegment::brake_to(0.0, 0.0, straight(true), ds, speed).unwrap();
    let cut = launch.cut_at_speed(speed + 0.15).unwrap();
    for seg in [&launch, &landing, &cut] {
        for i in 0..=20_000 {
            let t = seg.dt * f64::from(i) / 20_000.0;
            let (_, _, a) = seg.state_at(t);
            assert!(
                (a.abs() - accel).abs() <= 1e-12 * accel,
                "acceleration {a} strays from the budget {accel} at t={t}"
            );
        }
    }
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

/// The rail integration as it was before each step reused the state its
/// predecessor ended on: the oracle the carried version must match bit for
/// bit.
fn integrate_rail_arc_recomputing_step_ends(
    law: &ScalarLaw,
    v0: f64,
    ds: f64,
) -> Option<Arc<[RailKnot]>> {
    if ds <= 0.0 {
        let knot = RailKnot {
            t: 0.0,
            s: 0.0,
            v: v0,
            rise: 0.0,
        };
        return Some(Arc::from(vec![knot, knot]));
    }
    let n = ((ds / RAIL_KNOT_DS).ceil() as usize).clamp(RAIL_KNOTS_MIN, RAIL_KNOTS_MAX);
    let apex_start = {
        let a0 = law.accel_at(0.0, v0).abs();
        let budget = match law {
            ScalarLaw::DiskRail { accel, .. } => *accel,
            ScalarLaw::ConstAccel { a0 } => a0.abs().max(1.0),
        };
        a0 < 0.1 * budget
    };
    let arc_at = |k: usize| -> f64 {
        let u = k as f64 / n as f64;
        if apex_start { ds * u * u } else { ds * u }
    };
    let mut knots = Vec::with_capacity(n + 1);
    let mut w = v0 * v0;
    let mut t = 0.0_f64;
    knots.push(RailKnot {
        t,
        s: 0.0,
        v: v0,
        rise: 0.0,
    });
    let f = |s: f64, w: f64| 2.0 * law.accel_at(s, w.max(0.0).sqrt());
    for i in 0..n {
        let s = arc_at(i);
        let h = arc_at(i + 1) - s;
        let k1 = f(s, w);
        let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
        let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
        let k4 = f(s + h, w + h * k3);
        let increment = h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        let stepped = w + increment;
        let settled = law.settle_crossed_cap(s, s + h, stepped);
        let w_next = settled.unwrap_or(stepped);
        let last = i == n - 1;
        if w_next <= 0.0 && !last {
            return None;
        }
        let v_lo = w.max(0.0).sqrt();
        let v_hi = w_next.max(0.0).sqrt();
        if v_lo + v_hi <= 0.0 {
            return None;
        }
        let rise = match settled {
            None if w_next > 0.0 => increment / (v_lo + v_hi),
            _ => v_hi - v_lo,
        };
        let a_lo = law.accel_at(s, v_lo);
        let a_hi = law.accel_at(s + h, v_hi);
        let r_lo = law.accel_rate_at(s, v_lo, a_lo);
        let r_hi = law.accel_rate_at(s + h, v_hi, a_hi);
        t += knot_duration(h, (v_lo, a_lo, r_lo), (v_hi, a_hi, r_hi));
        w = w_next.max(0.0);
        knots.push(RailKnot {
            t,
            s: arc_at(i + 1),
            v: w.sqrt(),
            rise,
        });
    }
    Some(Arc::from(knots))
}

fn rail_laws() -> Vec<ScalarLaw> {
    let mut laws = vec![
        ScalarLaw::ConstAccel { a0: 0.0 },
        ScalarLaw::ConstAccel { a0: 3000.0 },
        ScalarLaw::ConstAccel { a0: -2500.0 },
    ];
    for &accel in &[100.0, 5000.0, 50_000.0] {
        for &kappa0 in &[0.0, 0.05, 2.0, -1.5] {
            for &sigma in &[0.0, 0.3, -4.0, 120.0] {
                for brake in [false, true] {
                    laws.push(ScalarLaw::DiskRail {
                        accel,
                        kappa0,
                        sigma,
                        brake,
                    });
                }
            }
        }
    }
    laws
}

fn assert_carrying_matches_recomputing(law: &ScalarLaw, v0: f64, ds: f64) {
    let bits = |knots: &Option<Arc<[RailKnot]>>| {
        knots.as_ref().map(|knots| {
            knots
                .iter()
                .map(|k| {
                    [
                        k.t.to_bits(),
                        k.s.to_bits(),
                        k.v.to_bits(),
                        k.rise.to_bits(),
                    ]
                })
                .collect::<Vec<_>>()
        })
    };
    assert_eq!(
        bits(&super::integrate_rail_arc(law, v0, ds)),
        bits(&integrate_rail_arc_recomputing_step_ends(law, v0, ds)),
        "{law:?} from v0={v0} over ds={ds}"
    );
}

#[test]
fn carrying_each_step_end_into_the_next_step_changes_no_knot() {
    let mut compared = 0usize;
    for law in rail_laws() {
        for &v0 in &[0.0, 1e-6, 0.5, 30.0, 250.0, 1200.0] {
            for &ds in &[0.0, 1e-5, 3e-4, 0.01, 0.7, 12.0] {
                assert_carrying_matches_recomputing(&law, v0, ds);
                compared += 1;
            }
        }
    }
    assert!(compared > 1000);
}

/// Steps of the quadratically graded grid near an apex whose `s + h` lands
/// on the next knot, and those that miss it by an ulp, over one span.
fn apex_grid_landings(ds: f64) -> (usize, usize) {
    let n = ((ds / RAIL_KNOT_DS).ceil() as usize).clamp(RAIL_KNOTS_MIN, RAIL_KNOTS_MAX);
    let arc_at = |k: usize| {
        let u = k as f64 / n as f64;
        ds * u * u
    };
    (0..n).fold((0, 0), |(lands, misses), i| {
        let (s, s_next) = (arc_at(i), arc_at(i + 1));
        if s + (s_next - s) == s_next {
            (lands + 1, misses)
        } else {
            (lands, misses + 1)
        }
    })
}

#[test]
fn an_apex_graded_step_ending_off_its_knot_recomputes_the_next_start() {
    let (accel, kappa0, v0) = (5000.0, 2.0, 49.9);
    let apex_spans = [
        0.000_401_859_266_767_729_34,
        0.039_314_775_273_804_545,
        0.152_112_117_011_757_6,
    ];
    let (mut lands, mut misses) = (0, 0);
    for &ds in &apex_spans {
        let (span_lands, span_misses) = apex_grid_landings(ds);
        lands += span_lands;
        misses += span_misses;
    }
    assert!(
        lands > 0 && misses > 0,
        "the sweep must exercise both reused and recomputed step starts \
         ({lands} landing, {misses} missing)"
    );
    for &sigma in &[0.0, 0.3, -4.0] {
        for brake in [false, true] {
            let law = ScalarLaw::DiskRail {
                accel,
                kappa0,
                sigma,
                brake,
            };
            assert!(
                law.accel_at(0.0, v0).abs() < 0.1 * accel,
                "{law:?} must start near its apex to grade the grid quadratically"
            );
            for &ds in &apex_spans {
                assert_carrying_matches_recomputing(&law, v0, ds);
            }
        }
    }
}
