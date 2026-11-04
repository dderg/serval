use std::sync::Arc;

use super::{
    CAP_CONTACT_FRACTION, CapDeparture, LawSegment, RAIL_ACCELERATION_DEFECT, RAIL_ARC_DEFECT,
    RAIL_SPEED_SLACK, RailKnot, ScalarLaw, defect::pair_defect,
};

/// How far acceleration `a` lies from the law read at arc `s` and any speed
/// within [`RAIL_SPEED_SLACK`] of `v` — the law the rail is held to.
fn law_miss(law: &ScalarLaw, s: f64, v: f64, a: f64) -> f64 {
    let slower = law.accel_at(s, (v - RAIL_SPEED_SLACK).max(0.0));
    let faster = law.accel_at(s, v + RAIL_SPEED_SLACK);
    let (low, high) = (slower.min(faster), slower.max(faster));
    (a - law.accel_at(s, v))
        .abs()
        .min((low - a).max(a - high).max(0.0))
}

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
fn disk_rail_scalar_acceleration_holds_the_budget_within_the_rail_defect() {
    // Braking into a tightening clothoid: kappa 0 -> 53.7 over 0.0292 mm at
    // A = 1000, entering at the disk-reach speed for the length.
    let seg = rail(7.3, 1000.0, 0.0, 1838.0, true, 0.0292);
    for i in 0..=1000 {
        let t = seg.dt * f64::from(i) / 1000.0;
        let (s, v, a) = seg.state_at(t);
        let kappa = (1838.0 * s).abs();
        let scalar = (a * a + (kappa * v * v) * (kappa * v * v)).sqrt();
        assert!(
            (scalar - 1000.0).abs() <= RAIL_ACCELERATION_DEFECT,
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
                (load - accel).abs() <= RAIL_ACCELERATION_DEFECT,
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
    // Reference: RK4 in time, finely. The rail strays from the law by at
    // most its defect, so from the reference by that over its duration.
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
    let speed_drift = RAIL_ACCELERATION_DEFECT * seg.dt;
    assert!(
        (s - 0.008).abs() <= 0.5 * speed_drift * seg.dt,
        "reference arc {s} vs the segment's own duration"
    );
    assert!(
        (v_seg - v).abs() <= speed_drift,
        "speed {v_seg} vs reference {v}"
    );
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

/// A rail that runs into its curvature cap inside a step — the reversed
/// rail of a brake whose onset sits just past a cap contact runs on into the
/// cap the forward rail left — and the times of the pair approaching it as
/// the square root of the time left.
fn rail_running_into_its_cap() -> (LawSegment, f64, f64) {
    let seg = rail(12.5556, 70_000.0, 422.616, 634_643.7, false, 1e-4);
    let knots = seg.rail_knots();
    let crossing = knots
        .windows(2)
        .find(|pair| pair[1].departure == CapDeparture::FromHi && pair[1].t > pair[0].t)
        .map(|pair| (pair[0].t, pair[1].t))
        .expect("fixture must approach its cap inside a knot pair");
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
    for (seg, (lo, hi)) in [(departing, departure_pair), {
        let (seg, lo, hi) = rail_running_into_its_cap();
        (seg, (lo, hi))
    }] {
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

#[test]
fn a_cut_approach_keeps_its_law_within_the_proven_allowance() {
    let (seg, lo, hi) = rail_running_into_its_cap();
    for share in [0.15, 0.5, 0.85] {
        let cut = seg.cut_at_time(lo + share * (hi - lo));
        let last = cut
            .rail_pairs(0.0, cut.dt)
            .last()
            .expect("the cut keeps a pair");
        assert!(
            matches!(last.shape, super::RailPairShape::Clipped { .. }),
            "a cut inside an approach keeps its square-root form, not {:?}",
            last.shape
        );
        let proven = pair_defect(&cut.law, &last);
        assert!(
            proven.acceleration <= proven.allowance(),
            "cut {share} into the approach proves {proven:?}"
        );
        for k in 0..=64 {
            let t = last.t_lo + (last.t_hi - last.t_lo) * f64::from(k) / 64.0;
            let (s, v, a) = cut.state_at(t);
            let miss = law_miss(&cut.law, s - cut.s0, v, a);
            assert!(
                miss <= proven.acceleration + 1e-9 * (1.0 + a.abs()),
                "cut {share}: the law is missed by {miss} at t={t}, proven {proven:?}"
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
    let cut = reversed
        .cut_at_arc(onset)
        .expect("the onset lies on the rail");
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

fn rail_laws() -> Vec<ScalarLaw> {
    let mut laws = Vec::new();
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

/// Every rail `rail_laws` integrates from a sweep of speeds at or below its
/// cap over a sweep of spans.
fn swept_rails() -> impl Iterator<Item = (ScalarLaw, f64, f64, Arc<Vec<RailKnot>>)> {
    rail_laws().into_iter().flat_map(|law| {
        [0.0, 1e-6, 0.5, 30.0, 250.0, 1200.0]
            .into_iter()
            .filter(move |v0| law.cap_speed_squared(0.0).is_none_or(|cap| v0 * v0 <= cap))
            .flat_map(move |v0| {
                [1e-5, 3e-4, 0.01, 0.7, 12.0]
                    .into_iter()
                    .filter_map(move |ds| {
                        super::integrate_rail_arc(&law, v0, ds).map(|knots| (law, v0, ds, knots))
                    })
            })
    })
}

#[test]
fn every_rail_knot_pair_holds_its_law_within_the_proven_allowance() {
    let mut pairs = 0usize;
    for (law, v0, ds, knots) in swept_rails() {
        for pair in knots.windows(2).filter(|pair| pair[1].t > pair[0].t) {
            let defect = pair_defect(&law, &law.rail_pair(&pair[0], &pair[1]));
            assert!(
                defect.acceleration <= defect.allowance() && defect.arc <= RAIL_ARC_DEFECT,
                "{law:?} from v0={v0} over {ds}: pair at t={} proves {defect:?}",
                pair[0].t
            );
            pairs += 1;
        }
    }
    assert!(pairs > 1000, "the sweep covers {pairs} pairs");
}

#[test]
fn no_sample_of_a_rail_pair_leaves_the_law_by_more_than_its_proven_defect() {
    for (law, v0, ds, knots) in swept_rails().step_by(7) {
        let seg = LawSegment {
            t0: 0.0,
            dt: knots[knots.len() - 1].t,
            s0: 0.0,
            v0,
            law,
            dense: std::sync::OnceLock::from(knots.clone()),
        };
        for pair in knots.windows(2).filter(|pair| pair[1].t > pair[0].t) {
            let proven = pair_defect(&law, &law.rail_pair(&pair[0], &pair[1])).acceleration;
            for i in 0..=64 {
                let t = pair[0].t + (pair[1].t - pair[0].t) * f64::from(i) / 64.0;
                let (s, v, a) = seg.state_at(t);
                let miss = law_miss(&law, s, v, a);
                let float_noise = 1e-9 * (1.0 + a.abs());
                assert!(
                    miss <= proven + float_noise,
                    "{law:?} from v0={v0} over {ds}: at t={t} the law is missed by {miss}, proven {proven}"
                );
            }
        }
    }
}

#[test]
fn a_smooth_rail_lays_knots_only_where_its_law_asks_for_them() {
    let seg = rail(30.0, 5000.0, 0.05, 0.3, false, 12.0);
    assert!(
        seg.rail_knots().len() < 200,
        "a 12 mm rail of slowly turning curvature lays {} knots",
        seg.rail_knots().len()
    );
}

/// The worst the rail's interpolated acceleration leaves the law's own
/// `a(s, v)` near the interpolated state, over dense times.
fn worst_law_miss(seg: &LawSegment) -> f64 {
    (0..=20_000)
        .map(|i| {
            let t = seg.dt * f64::from(i) / 20_000.0;
            let (s, v, a) = seg.state_at(seg.t0 + t);
            law_miss(&seg.law, s - seg.s0, v, a)
        })
        .fold(0.0_f64, f64::max)
}

/// A 70 000 mm/s² Trident corner's rail: it runs onto its curvature cap
/// inside one grid step, rides it, and leaves it again before its end.
fn rail_onto_and_off_its_cap() -> LawSegment {
    rail(12.45, 70_000.0, 400.0, 634_643.703_105_817_3, false, 0.0007)
}

const LAW_MISS: f64 = RAIL_ACCELERATION_DEFECT;

#[test]
fn a_rail_running_onto_its_cap_inside_a_step_follows_the_law_through_the_contact() {
    let seg = rail_onto_and_off_its_cap();
    assert_eq!(
        seg.cap_contact_times().len(),
        2,
        "the fixture must run onto its cap and leave it"
    );
    let worst = worst_law_miss(&seg);
    assert!(
        worst <= LAW_MISS,
        "the interpolant leaves the law by {worst} mm/s² across the cap contacts"
    );
}

#[test]
fn a_cut_inside_the_step_that_runs_onto_the_cap_follows_the_law() {
    let seg = rail_onto_and_off_its_cap();
    let contact = seg.cap_contact_times()[0] - seg.t0;
    let (contact_arc, _, _) = seg.state_at(seg.t0 + contact);
    for share in [0.5, 0.85, 0.9, 0.95] {
        let cut = seg
            .cut_at_arc(share * (contact_arc - seg.s0))
            .expect("the cut lies inside the rail");
        let worst = worst_law_miss(&cut);
        assert!(
            worst <= LAW_MISS,
            "cut at {share} of the contact arc: the interpolant leaves the law by {worst} mm/s²"
        );
    }
}

#[test]
fn a_rail_riding_a_stiff_rising_cap_lays_its_curve_in_few_knots() {
    let seg = rail(
        3.617_106_127_137_380_8,
        47_884.320_049_387_56,
        33.215_730_573_950_42,
        -0.417_370_919_017_468_3,
        false,
        3.574_240_610_783_556,
    );
    assert!(
        seg.rail_knots().len() < 2_000,
        "a 3.6 mm rail riding a stiff rising cap lays {} knots",
        seg.rail_knots().len()
    );
}

#[test]
fn a_rail_holding_a_held_cap_lays_it_in_few_knots() {
    let seg = rail(
        0.0,
        320_032.091_245_085,
        -33.613_536_838_661_12,
        0.0,
        false,
        2.893_100_978_587_263,
    );
    assert!(
        seg.rail_knots().len() < 2_000,
        "a 2.9 mm rail holding its cap lays {} knots",
        seg.rail_knots().len()
    );
}
