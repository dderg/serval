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
