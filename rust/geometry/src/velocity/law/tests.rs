use std::sync::Arc;

use super::{
    CAP_CONTACT_FRACTION, CapDeparture, FULL_RAIL_KNOTS, KnotSlopes, LawSegment, RAIL_KNOT_DS,
    RAIL_KNOTS_MAX, RAIL_KNOTS_MIN, RailKnot, SPARE_FULL_RAILS, ScalarLaw, departure_increment,
    departure_knot_duration, knot_duration, trim_spare_full_rails,
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
fn origin_rail_identity_is_bitwise_in_every_input() {
    let law = |kappa0: f64| ScalarLaw::DiskRail {
        accel: 1000.0,
        kappa0,
        sigma: 0.05,
        brake: false,
    };
    let seg = rail(50.0, 1000.0, -0.0, 0.05, false, 2.0);
    assert!(seg.is_origin_rail(50.0, law(-0.0), 2.0));
    assert!(!seg.is_origin_rail(50.0, law(0.0), 2.0));
    assert!(!seg.is_origin_rail(50.000_000_000_001, law(-0.0), 2.0));
    assert!(!seg.is_origin_rail(50.0, law(-0.0), 2.000_000_000_001));
    let shifted = LawSegment::until_arc(0.0, 1.0, 50.0, law(-0.0), 2.0).unwrap();
    assert!(!shifted.is_origin_rail(50.0, law(-0.0), 2.0));
    let line = LawSegment::new(0.0, 0.1, 0.0, 50.0, ScalarLaw::ConstAccel { a0: 1000.0 });
    assert!(!line.is_origin_rail(50.0, ScalarLaw::ConstAccel { a0: 1000.0 }, 2.0));
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
fn a_straight_rail_never_touches_the_curvature_cap() {
    assert!(
        rail(5.0, 1000.0, 0.0, 0.0, false, 0.1)
            .cap_contact_times()
            .is_empty()
    );
}

#[test]
fn a_rail_leaving_the_curvature_cap_reports_its_start_and_where_it_leaves() {
    let accel = 5000.0;
    let seg = rail((accel / 2.0_f64).sqrt(), accel, 2.0, -40.0, false, 0.05);
    let in_contact = |t: f64| seg.state_at(t - seg.t0).2.abs() <= CAP_CONTACT_FRACTION * accel;
    assert!(
        in_contact(seg.t0) && !in_contact(seg.end_time()),
        "fixture must start on the cap and leave it"
    );
    let times = seg.cap_contact_times();
    assert_eq!(times.len(), 2, "{times:?}");
    assert_eq!(times[0], seg.t0);
    assert!(
        in_contact(times[1]) && times[1] < seg.end_time(),
        "the last contact {} must sit on the cap inside the segment",
        times[1]
    );
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

/// The worst gap, over the first `window` away from the apex, between the
/// rail's acceleration and the law integrated finely in time from the apex:
/// forward from a launch, backward from a landing.
fn worst_departure_from_the_law_at_the_apex(
    seg: &LawSegment,
    accel: f64,
    apex_kappa: f64,
    sigma_away: f64,
    window: f64,
) -> (f64, f64) {
    let ScalarLaw::DiskRail { brake, .. } = seg.law else {
        panic!("an apex belongs to a disk rail");
    };
    let away = |s: f64, v: f64| -> f64 {
        let a_n = (apex_kappa + sigma_away * s).abs() * v * v;
        (accel * accel - a_n * a_n).max(0.0).sqrt()
    };
    let steps = 400_000;
    let h = window / f64::from(steps);
    let (mut s, mut v) = (0.0_f64, (accel / apex_kappa.abs()).sqrt());
    let mut worst = (0.0_f64, 0.0_f64);
    for i in 1..=steps {
        let (k1s, k1v) = (v, away(s, v));
        let (k2s, k2v) = (
            v + 0.5 * h * k1v,
            away(s + 0.5 * h * k1s, v + 0.5 * h * k1v),
        );
        let (k3s, k3v) = (
            v + 0.5 * h * k2v,
            away(s + 0.5 * h * k2s, v + 0.5 * h * k2v),
        );
        let (k4s, k4v) = (v + h * k3v, away(s + h * k3s, v + h * k3v));
        s += h * (k1s + 2.0 * k2s + 2.0 * k3s + k4s) / 6.0;
        v += h * (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
        if i % 100 == 0 {
            let from_apex = h * f64::from(i);
            let (t, truth) = if brake {
                (seg.dt - from_apex, -away(s, v))
            } else {
                (from_apex, away(s, v))
            };
            let off = (seg.state_at(t).2 - truth).abs();
            if off > worst.0 {
                worst = (off, from_apex);
            }
        }
    }
    worst
}

/// A corner blend's two clothoids meet at its apex on the curvature cap,
/// whose rise there outruns any rail (`|σ| > 2κ²`): the tangential
/// acceleration leaves zero as `√τ` on both sides. Taken from the dense top
/// layers of the Voron print on the fast bench config.
#[test]
fn a_rail_follows_the_square_root_departure_from_the_cap_at_a_blend_apex() {
    let accel = 50_000.0_f64;
    let apex_kappa = -575.525_278_267_944_3_f64;
    let apex_speed = (accel / apex_kappa.abs()).sqrt();
    let launch_sigma = 821_979.862_028_279;
    let launch = rail(
        apex_speed,
        accel,
        apex_kappa,
        launch_sigma,
        false,
        0.000_700_169_559_930_343_1,
    );
    let (landing_kappa0, landing_sigma, landing_ds) = (
        0.306_152_858_833_640_6,
        -821_972.640_474_798_4,
        0.000_700_548_172_496_542_9,
    );
    let (landing, _) = LawSegment::brake_to(
        0.0,
        0.0,
        ScalarLaw::DiskRail {
            accel,
            kappa0: landing_kappa0,
            sigma: landing_sigma,
            brake: true,
        },
        landing_ds,
        (accel / (landing_kappa0 + landing_sigma * landing_ds).abs()).sqrt(),
    )
    .expect("the landing reaches its apex");
    for (name, seg, kappa, sigma_away) in [
        ("launch", &launch, apex_kappa, launch_sigma),
        (
            "landing",
            &landing,
            landing_kappa0 + landing_sigma * landing_ds,
            -landing_sigma,
        ),
    ] {
        let (off, from_apex) =
            worst_departure_from_the_law_at_the_apex(seg, accel, kappa, sigma_away, 4e-6);
        assert!(
            off <= 1e-4 * accel,
            "the {name}'s acceleration strays {off} mm/s² from the law {from_apex} s from \
             the apex"
        );
    }
}

/// A long brake landing on the curvature cap of a gentle clothoid, whose cap
/// falls toward the rail slower than it can follow (`|σ| < 2κ²`): seen from
/// the landing, the acceleration departs zero as `√τ` and stays inside the
/// cap-contact band for many dense steps before settling toward the cap's
/// own slope. Taken from the dense top layers of the Voron print on the
/// fast bench config.
#[test]
fn a_brake_landing_on_a_slowly_falling_cap_follows_the_law_through_its_contact_band() {
    let accel = 50_000.0_f64;
    let (kappa0, sigma, ds) = (
        0.465_069_159_612_022_44_f64,
        -65.027_205_501_112_82,
        0.152_383_992_972_882_74,
    );
    let landing_kappa = kappa0 + sigma * ds;
    let (landing, _) = LawSegment::brake_to(
        0.0,
        0.0,
        ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            brake: true,
        },
        ds,
        (accel / landing_kappa.abs()).sqrt(),
    )
    .expect("the brake reaches its landing");
    let (off, from_apex) =
        worst_departure_from_the_law_at_the_apex(&landing, accel, landing_kappa, -sigma, 2e-6);
    assert!(
        off <= 1e-4 * accel,
        "the landing's acceleration strays {off} mm/s² from the law {from_apex} s before it"
    );
}

/// A rail that runs into its curvature cap inside a knot pair — the reversed
/// rail of a brake whose onset sits just past a cap contact runs on into the
/// cap the forward rail left. Past the cap the law reads no tangential
/// budget, so the pair's quintic misses the square-root approach and its
/// acceleration leaves the law's by hundreds of mm/s² inside the pair.
fn rail_running_into_its_cap() -> (LawSegment, f64, f64) {
    let seg = rail(12.5556, 70_000.0, 422.616, 634_643.7, false, 1e-4);
    let knots = seg.rail_knots();
    let accel = |knot: &RailKnot| seg.law.accel_at(knot.s, knot.v);
    let crossing = knots
        .windows(2)
        .find(|pair| accel(&pair[0]) > 0.0 && accel(&pair[1]) == 0.0)
        .map(|pair| (pair[0].t, pair[1].t))
        .expect("fixture must run into its cap inside a knot pair");
    (seg, crossing.0, crossing.1)
}

/// The largest gaps in arc, speed and acceleration between two readings of
/// a motion over `times`.
fn worst_gap(
    times: impl Iterator<Item = f64>,
    reading: impl Fn(f64) -> ((f64, f64, f64), (f64, f64, f64)),
) -> [f64; 3] {
    times.fold([0.0; 3], |worst, t| {
        let ((s, v, a), (other_s, other_v, other_a)) = reading(t);
        [
            worst[0].max((s - other_s).abs()),
            worst[1].max((v - other_v).abs()),
            worst[2].max((a - other_a).abs()),
        ]
    })
}

fn kept_stretch(lo: f64, tau: f64) -> impl Iterator<Item = f64> {
    (0..=64).map(move |k| lo + (tau - lo) * f64::from(k) / 64.0)
}

#[test]
fn a_rail_cut_inside_a_knot_pair_keeps_the_motion_its_interpolant_describes() {
    let accel = 5000.0;
    let departing = rail((accel / 2.0_f64).sqrt(), accel, 2.0, -40.0, false, 0.05);
    let departure_pair = {
        let knots = departing.rail_knots();
        assert_eq!(
            knots[1].departure,
            CapDeparture::FromLo,
            "fixture must depart its cap"
        );
        (knots[0].t, knots[1].t)
    };
    for (seg, (lo, hi)) in [
        (departing.clone(), departure_pair),
        {
            let (seg, lo, hi) = rail_running_into_its_cap();
            (seg, (lo, hi))
        },
    ] {
        for share in [0.15, 0.5, 0.85] {
            let tau = lo + share * (hi - lo);
            let cut = seg.cut_at_time(tau);
            let gap = worst_gap(kept_stretch(lo, tau), |t| {
                (cut.state_at(t), seg.state_at(t))
            });
            assert!(
                gap[0] <= 1e-15 && gap[1] <= 1e-12 && gap[2] <= 1e-6,
                "cut {share} of the way into a pair, the kept stretch leaves the rail's motion \
                 by {gap:?} in arc, speed and acceleration"
            );
        }
    }
}

/// A brake is its reversed rail cut at the onset and flipped: the flip keeps
/// the cut's motion, mirrored in time.
#[test]
fn a_brake_flipped_from_a_rail_cut_inside_a_knot_pair_keeps_its_motion() {
    let (reversed, lo, hi) = rail_running_into_its_cap();
    let onset = reversed.state_at(lo + 0.15 * (hi - lo)).0;
    let brake_law = ScalarLaw::DiskRail {
        accel: 70_000.0,
        kappa0: 634_643.7_f64.mul_add(onset, 422.616),
        sigma: -634_643.7,
        brake: true,
    };
    let cut = reversed.cut_at_arc(onset).expect("the onset lies on the rail");
    let (brake, _) = reversed
        .flipped_cut(0.0, 0.0, brake_law, onset)
        .expect("the onset lies on the rail");
    let gap = worst_gap(kept_stretch(lo, cut.dt), |t| {
        let (s, v, a) = cut.state_at(t);
        (brake.state_at(cut.dt - t), (onset - s, v, -a))
    });
    assert!(
        gap[0] <= 1e-15 && gap[1] <= 1e-12 && gap[2] <= 1e-6,
        "the brake leaves its reversed cut's motion by {gap:?} in arc, speed and acceleration"
    );
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
/// predecessor ended on, or a memoized step of an autonomous law: every step
/// recomputed from its start, the departure from a cap by its own step. The
/// oracle the integrator must match bit for bit.
fn integrate_rail_arc_recomputing_step_ends(
    law: &ScalarLaw,
    v0: f64,
    ds: f64,
) -> Option<Arc<Vec<RailKnot>>> {
    if ds <= 0.0 {
        let knot = RailKnot {
            t: 0.0,
            s: 0.0,
            v: v0,
            rise: 0.0,
            departure: CapDeparture::None,
            slopes: KnotSlopes::Law,
        };
        return Some(Arc::new(vec![knot, knot]));
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
    let departs_cap = law.in_cap_contact(law.accel_at(0.0, v0));
    let steps = if departs_cap { n - 1 } else { n };
    let grid_at = |i: usize| arc_at(if departs_cap && i > 0 { i + 1 } else { i });
    let mut knots = Vec::with_capacity(steps + 1);
    let mut w = v0 * v0;
    let mut t = 0.0_f64;
    knots.push(RailKnot {
        t,
        s: 0.0,
        v: v0,
        rise: 0.0,
        departure: CapDeparture::None,
        slopes: KnotSlopes::Law,
    });
    let f = |s: f64, w: f64| 2.0 * law.accel_at(s, w.max(0.0).sqrt());
    for i in 0..steps {
        let s = grid_at(i);
        let h = grid_at(i + 1) - s;
        let departure = if departs_cap && i == 0 {
            CapDeparture::FromLo
        } else {
            CapDeparture::None
        };
        let increment = if departure == CapDeparture::FromLo {
            departure_increment(law, s, h, w)
        } else {
            let k1 = f(s, w);
            let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
            let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
            let k4 = f(s + h, w + h * k3);
            h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        };
        let stepped = w + increment;
        let settled = law.settle_crossed_cap(s, s + h, stepped);
        let w_next = settled.unwrap_or(stepped);
        let last = i == steps - 1;
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
        let duration = match departure {
            CapDeparture::FromLo => departure_knot_duration,
            _ => knot_duration,
        };
        t += duration(h, (v_lo, a_lo, r_lo), (v_hi, a_hi, r_hi));
        w = w_next.max(0.0);
        knots.push(RailKnot {
            t,
            s: grid_at(i + 1),
            v: w.sqrt(),
            rise,
            departure,
            slopes: KnotSlopes::Law,
        });
    }
    Some(Arc::new(knots))
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

fn knots_bits(knots: &Arc<Vec<RailKnot>>) -> Vec<[u64; 5]> {
    knots
        .iter()
        .map(|k| {
            [
                k.t.to_bits(),
                k.s.to_bits(),
                k.v.to_bits(),
                k.rise.to_bits(),
                k.departure as u64,
            ]
        })
        .collect()
}

fn assert_carrying_matches_recomputing(law: &ScalarLaw, v0: f64, ds: f64) {
    assert_eq!(
        super::integrate_rail_arc(law, v0, ds)
            .as_ref()
            .map(knots_bits),
        integrate_rail_arc_recomputing_step_ends(law, v0, ds)
            .as_ref()
            .map(knots_bits),
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

#[test]
fn memoized_steps_of_constant_curvature_rails_change_no_knot() {
    for sigma in [0.0, -0.0] {
        for &(accel, kappa0, v0, ds) in &[
            (
                50_000.0,
                0.305_837_469_599_224_14,
                0.0,
                20.293_902_419_340_885,
            ),
            (50_000.0, -0.305_837_469_599_224_14, 120.0, 9.75),
            (20_000.0, 1.25, 0.0, 3.3),
            (20_000.0, 1.25, 400.0, 3.3),
        ] {
            for brake in [false, true] {
                let law = ScalarLaw::DiskRail {
                    accel,
                    kappa0,
                    sigma,
                    brake,
                };
                assert_carrying_matches_recomputing(&law, v0, ds);
            }
        }
    }
}

#[test]
fn a_constant_curvature_rail_departs_its_cap_by_its_own_step_not_a_memoized_one() {
    let rails: [(f64, f64, f64); 3] = [
        (50_000.0, 0.305_837_469_599_224_14, 0.8),
        (20_000.0, -1.25, 0.2),
        (5000.0, 2.0, 0.01),
    ];
    for sigma in [0.0, -0.0] {
        for &(accel, kappa0, ds) in &rails {
            let cap_speed = (accel / kappa0.abs()).sqrt();
            for v0 in [cap_speed, cap_speed * (1.0 - 1e-6), cap_speed * 1.5] {
                for brake in [false, true] {
                    let law = ScalarLaw::DiskRail {
                        accel,
                        kappa0,
                        sigma,
                        brake,
                    };
                    let knots = super::integrate_rail_arc(&law, v0, ds).unwrap();
                    assert_eq!(knots[1].departure, CapDeparture::FromLo, "{law:?} v0={v0}");
                    assert_eq!(knots[2].departure, CapDeparture::None, "{law:?} v0={v0}");
                    assert_carrying_matches_recomputing(&law, v0, ds);
                }
            }
        }
    }
}

fn knot_bits(seg: &LawSegment) -> Vec<[u64; 4]> {
    seg.rail_knots()
        .iter()
        .map(|k| {
            [
                k.t.to_bits(),
                k.s.to_bits(),
                k.v.to_bits(),
                k.rise.to_bits(),
            ]
        })
        .collect()
}

#[test]
fn recycled_full_rail_buffers_carry_nothing_into_the_next_rail() {
    let spare = || SPARE_FULL_RAILS.with_borrow(|spare| spare.buffers.len());
    let ds = 1.5 * RAIL_KNOTS_MAX as f64 * RAIL_KNOT_DS;
    let first = rail(30.0, 5000.0, 0.05, 0.3, false, ds);
    assert_eq!(first.rail_knots().len(), FULL_RAIL_KNOTS);
    let first_bits = knot_bits(&first);
    let shared = first.clone();
    first.recycle();
    assert_eq!(spare(), 0, "a rail still shared is not recycled");
    assert_eq!(knot_bits(&shared), first_bits);
    shared.recycle();
    assert_eq!(spare(), 1);

    let second = rail(120.0, 20_000.0, 1.25, 0.0, false, ds);
    assert_eq!(spare(), 0, "the next full rail reuses the spare buffer");
    let fresh = std::thread::spawn(move || knot_bits(&rail(120.0, 20_000.0, 1.25, 0.0, false, ds)))
        .join()
        .unwrap();
    assert_eq!(knot_bits(&second), fresh);
}

#[test]
fn a_thread_keeps_no_more_spare_full_rails_than_its_last_plan_asked_for() {
    let spare = || SPARE_FULL_RAILS.with_borrow(|spare| spare.buffers.len());
    let ds = 1.5 * RAIL_KNOTS_MAX as f64 * RAIL_KNOT_DS;
    let rails: Vec<LawSegment> = (0..3)
        .map(|i| rail(30.0 + f64::from(i), 5000.0, 0.05, 0.3, false, ds))
        .collect();
    trim_spare_full_rails();
    for rail in rails {
        rail.recycle();
    }
    assert_eq!(spare(), 3);

    let _next = rail(60.0, 5000.0, 0.05, 0.3, false, ds);
    trim_spare_full_rails();
    assert_eq!(
        spare(),
        1,
        "a plan that asked for one full rail keeps one spare"
    );
    trim_spare_full_rails();
    assert_eq!(spare(), 0, "a plan that asked for none keeps none");
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
