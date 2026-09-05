use super::super::disk::Kinematics;
use super::super::disk::RunMember;
use super::super::law::ScalarLaw;
use super::{ReconstructError, member_profile};

fn straight(length: f64, ceiling: f64, accel: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk: f64::INFINITY,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: ceiling,
    }
}

fn clothoid(length: f64, kappa0: f64, sigma: f64, ceiling: f64, accel: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk: f64::INFINITY,
        kappa0,
        sigma,
        flat_ceiling: ceiling,
    }
}

fn profile(kin: &Kinematics, entry: f64, exit: f64) -> Vec<super::super::law::LawSegment> {
    let m = RunMember { kin, exit_v: exit };
    member_profile(0, &m, entry, exit).unwrap()
}

fn assert_tiles(segments: &[super::super::law::LawSegment], length: f64) {
    let mut t = 0.0;
    let mut s = 0.0;
    let mut v = None::<f64>;
    for seg in segments {
        assert!((seg.t0 - t).abs() <= 1e-9 * (1.0 + t), "time gap at {t}");
        assert!((seg.s0 - s).abs() <= 1e-9 * (1.0 + s), "arc gap at {s}");
        if let Some(v_prev) = v {
            assert!(
                (seg.v0 - v_prev).abs() <= 1e-6 * (1.0 + v_prev),
                "velocity step {v_prev} -> {} at joint",
                seg.v0
            );
        }
        let (s_end, v_end, _) = seg.end_state();
        t = seg.end_time();
        s = s_end;
        v = Some(v_end);
    }
    assert!(
        (s - length).abs() <= 1e-9 * (1.0 + length),
        "profile covers {s} of {length}"
    );
}

#[test]
fn straight_rest_to_rest_is_a_trapezoid() {
    let kin = straight(100.0, 30.0, 1000.0);
    let segments = profile(&kin, 0.0, 0.0);
    assert_tiles(&segments, 100.0);
    assert_eq!(segments.len(), 3);
    assert!(matches!(segments[0].law, ScalarLaw::ConstAccel { a0 } if a0 == 1000.0));
    assert!(matches!(segments[1].law, ScalarLaw::ConstAccel { a0 } if a0 == 0.0));
    assert!(matches!(segments[2].law, ScalarLaw::ConstAccel { a0 } if a0 == -1000.0));
    assert!((segments[1].v0 - 30.0).abs() < 1e-9);
    let (_, v_end, _) = segments[2].end_state();
    assert!(v_end.abs() < 1e-9);
}

#[test]
fn short_straight_is_a_triangle() {
    let kin = straight(1.0, 300.0, 1000.0);
    let segments = profile(&kin, 0.0, 0.0);
    assert_tiles(&segments, 1.0);
    assert_eq!(segments.len(), 2);
    let peak = segments[1].v0;
    assert!((peak - (1000.0_f64).sqrt()).abs() < 1e-6, "peak {peak}");
}

#[test]
fn clothoid_brake_stays_on_the_disk() {
    // The printer 90-degree corner's decelerating half.
    let kin = clothoid(0.0292, 0.0, 1838.0, 300.0, 1000.0);
    let exit = (1000.0_f64 / (1838.0 * 0.0292)).sqrt();
    let entry = super::super::disk::disk_reach_v_rev(&kin, exit, kin.length, 1e-9).unwrap();
    let segments = profile(&kin, entry, exit);
    assert_tiles(&segments, kin.length);
    for seg in &segments {
        for i in 0..=200 {
            let t = seg.t0 + seg.dt * f64::from(i) / 200.0;
            let (s, v, a) = seg.state_at(t);
            let kappa = (1838.0 * s).abs();
            let scalar = (a * a + (kappa * v * v).powi(2)).sqrt();
            assert!(
                (scalar - 1000.0).abs() < 2e-5,
                "scalar accel {scalar} left the disk at t={t}"
            );
        }
    }
}

#[test]
fn brake_lands_rest_exactly() {
    let kin = straight(10.0, 100.0, 2000.0);
    let entry = super::super::disk::disk_reach_v_rev(&kin, 0.0, kin.length, 1e-9)
        .unwrap()
        .min(kin.flat_ceiling);
    let segments = profile(&kin, entry, 0.0);
    assert_tiles(&segments, 10.0);
    let (_, v_end, _) = segments.last().unwrap().end_state();
    assert_eq!(v_end, 0.0);
}

#[test]
fn arc_cruises_at_its_curvature_cap() {
    // Constant curvature: the rail accelerates asymptotically onto
    // sqrt(A/kappa) and holds it with zero tangential acceleration.
    let kin = clothoid(40.0, 0.05, 0.0, 300.0, 2000.0);
    let cap = (2000.0_f64 / 0.05).sqrt();
    let segments = profile(&kin, cap, cap);
    assert_tiles(&segments, 40.0);
    for seg in &segments {
        let (_, v, a) = seg.state_at(seg.t0 + 0.5 * seg.dt);
        assert!((v - cap).abs() < 1e-6 * cap);
        assert!(a.abs() < 1e-3);
    }
}

#[test]
fn infeasible_seam_pair_fails_loudly() {
    let kin = straight(1.0, 300.0, 1000.0);
    let m = RunMember {
        kin: &kin,
        exit_v: 200.0,
    };
    let err = member_profile(0, &m, 0.0, 200.0).unwrap_err();
    assert!(matches!(err, ReconstructError::Infeasible { .. }));
}

/// The onset is solved in arc but consumed as a speed seam, and `dv/ds = a/v`
/// converts one into the other. At a high budget and a low ceiling the brake
/// span is short enough that a fixed-iteration bracket left ~3e-8 mm of arc
/// open, which the conversion turned into 9.4e-5 mm/s at the seam — four times
/// the slack the brake check allows — and the member came back `Infeasible`.
#[test]
fn a_short_brake_span_at_a_high_budget_still_closes_its_seam() {
    let kin = clothoid(14.953550178205024, 1.0, 0.0, 18.93733259936017, 216193.6);
    let segments = profile(&kin, 0.0, 0.0);
    let mut arc = 0.0;
    for seg in &segments {
        assert!(
            (seg.s0 - arc).abs() <= 1e-9 * (1.0 + arc),
            "arc gap at {arc}"
        );
        arc = seg.end_distance();
    }
    assert!(
        (arc - kin.length).abs() <= 1e-9 * kin.length,
        "profile covers {arc} of {}",
        kin.length
    );
    let peak = segments
        .iter()
        .map(|seg| seg.end_state().1)
        .fold(0.0_f64, f64::max);
    assert!(
        (peak - kin.flat_ceiling).abs() < 1e-5 * kin.flat_ceiling,
        "profile peaked at {peak}, not the ceiling {}",
        kin.flat_ceiling
    );
}

/// The knots never exceed the cap; between the last knot below it and the
/// first on it the quintic in time cannot carry the square-root approach and
/// overshoots by a sixteenth of that final gap.
fn assert_never_over_the_cap(
    segments: &[super::super::law::LawSegment],
    kin: &Kinematics,
    samples: usize,
) {
    for seg in segments {
        for i in 0..=samples {
            let (s, v, _) = seg.state_at(seg.t0 + seg.dt * i as f64 / samples as f64);
            let cap = super::super::disk::limit_speed(kin.kappa_abs(s), kin.accel);
            assert!(
                v <= cap * (1.0 + 1e-5),
                "speed {v} over the cap {cap} at arc {s}"
            );
        }
    }
}

/// The feed sits between the two end caps, so the brake into the tighter end
/// hugs its cap for most of the member. The reversed rail integrating that
/// brake used to overshoot the cap by ~2e-5 with a fixed-step overshoot that
/// depended on the step count, so the brake's entry speed and the cruise it
/// had to meet disagreed by more than the seam slack.
#[test]
fn a_brake_hugging_the_cap_still_meets_the_cruise() {
    let kin = clothoid(
        7.245762259004547,
        0.0,
        6.6248300011163614,
        43.108293553355324,
        85464.7,
    );
    let segments = profile(&kin, 0.0, 0.0);
    assert_tiles(&segments, kin.length);
    assert_never_over_the_cap(&segments, &kin, 512);
    assert!(
        segments
            .iter()
            .any(|seg| matches!(seg.law, ScalarLaw::ConstAccel { a0 } if a0 == 0.0)),
        "the feed binds before the cap does, so the member cruises"
    );
}

/// A tight arc at a feed far above its constant cap: both rails settle onto
/// the cap, and the accelerate/brake seam sits on it.
#[test]
fn a_rest_to_rest_arc_above_its_cap_cruises_on_the_cap() {
    let kappa = 1.0 / 0.024823023551351353;
    let kin = clothoid(
        5.57026221417865 * 0.024823023551351353,
        kappa,
        0.0,
        10.0,
        100.0,
    );
    let segments = profile(&kin, 0.0, 0.0);
    assert_tiles(&segments, kin.length);
    assert_never_over_the_cap(&segments, &kin, 512);
    let cap = (100.0_f64 / kappa).sqrt();
    let peak = segments
        .iter()
        .map(|seg| seg.end_state().1)
        .fold(0.0_f64, f64::max);
    assert!(
        (peak - cap).abs() < 1e-6 * cap,
        "profile peaked at {peak}, not the cap {cap}"
    );
}

/// Entered on its cap with the cap rising away, the rail reaches the feed
/// ceiling while still hugging it. The contact used to be read off the
/// interpolated profile, whose quintic overshoots between knots there, so the
/// accelerating segment built over that arc landed above the ceiling by more
/// than the seam slack — and nothing checked the accelerate/cruise seam.
#[test]
fn the_accelerate_to_cruise_seam_closes_off_a_rising_cap() {
    let kin = clothoid(
        2.0650975858530343,
        37.3767719082361,
        -18.03351395912919,
        32.5462164415722,
        39073.12145416431,
    );
    let entry = (kin.accel / kin.kappa0).sqrt();
    let segments = profile(&kin, entry, 0.0);
    assert_tiles(&segments, kin.length);
    let cruise = segments
        .iter()
        .position(|seg| matches!(seg.law, ScalarLaw::ConstAccel { a0 } if a0 == 0.0))
        .expect("the ceiling binds, so the member cruises");
    let accelerate_end = segments[cruise - 1].end_state().1;
    assert!(
        (accelerate_end - kin.flat_ceiling).abs() <= 1e-6 * (1.0 + kin.flat_ceiling) + 1e-6,
        "the accelerating rail ends at {accelerate_end}, off the ceiling {}",
        kin.flat_ceiling
    );
}

/// Entered on its cap with the cap rising, the rail hugs the cap up to the
/// feed ceiling, and the fixed-step integration jitters there by more than
/// the seam slack from one cut arc to the next. The contact used to be
/// bisected over that jitter and the accelerating segment re-integrated to
/// the found arc, landing off the ceiling; it is now the rail cut where it
/// first reaches the ceiling.
#[test]
fn a_rail_hugging_its_cap_meets_the_ceiling_exactly() {
    let kin = clothoid(
        3.1277409790611603,
        -29.141597325558312,
        9.362797258954922,
        15.46691300072049,
        6366.620569989854,
    );
    let entry = (kin.accel / kin.kappa0.abs()).sqrt();
    let segments = profile(&kin, entry, 0.0);
    assert_tiles(&segments, kin.length);
    let cruise = segments
        .iter()
        .position(|seg| matches!(seg.law, ScalarLaw::ConstAccel { a0 } if a0 == 0.0))
        .expect("the member cruises at the feed");
    assert!(cruise > 0, "the rail accelerates onto the ceiling first");
    let landing = segments[cruise - 1].end_state().1;
    assert_eq!(
        landing, kin.flat_ceiling,
        "the accelerating rail ends on the ceiling the cruise starts at"
    );
}
