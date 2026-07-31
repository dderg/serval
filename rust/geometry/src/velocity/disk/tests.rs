use super::*;
use crate::velocity::scurve;

fn kin(kappa0: f64, sigma: f64, length: f64, accel: f64, jerk: f64, flat: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk,
        kappa0,
        sigma,
        flat_ceiling: flat,
    }
}

fn numeric_const_kappa_w(w_in: f64, length: f64, accel: f64, kappa: f64) -> f64 {
    let f = |w: f64| 2.0 * (accel * accel - (kappa * kappa) * (w * w)).max(0.0).sqrt();
    let steps = 200_000u32;
    let h = length / f64::from(steps);
    let mut w = w_in;
    for _ in 0..steps {
        let k1 = f(w);
        let k2 = f(w + 0.5 * h * k1);
        let k3 = f(w + 0.5 * h * k2);
        let k4 = f(w + h * k3);
        w += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    w
}

#[test]
fn const_kappa_closed_form_matches_numeric_ode() {
    let (w_in, length, accel, kappa) = (100.0, 2.0, 2000.0, 0.02);
    let closed = const_kappa_reach_w(w_in, length, accel, kappa);
    let numeric = numeric_const_kappa_w(w_in, length, accel, kappa);
    let v = closed.sqrt();
    assert!((closed.sqrt() - numeric.sqrt()).abs() < 1e-6 * v);
}

#[test]
fn line_reach_is_constant_accel() {
    let k = kin(0.0, 0.0, 3.0, 1000.0, f64::INFINITY, 1e9);
    let v = disk_reach_v(&k, 40.0, k.length, 1e-7).unwrap();
    let expected = (40.0_f64 * 40.0 + 2.0 * 1000.0 * 3.0).sqrt();
    assert!((v - expected).abs() < 1e-9 * expected);
}

#[test]
fn infinite_jerk_reach_is_pure_disk() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, f64::INFINITY, 300.0);
    let pure = disk_reach_v(&k, 50.0, k.length, 1e-7).unwrap();
    let jerk = scurve::max_reachable_velocity(50.0, k.length, k.accel, k.jerk);
    let reached = pure.min(jerk);
    assert!((reached - pure).abs() < 1e-9 * pure.max(1.0));
}

#[test]
fn zero_curvature_reach_is_scurve() {
    let k = kin(0.0, 0.0, 4.0, 1000.0, 60_000.0, 300.0);
    let disk = disk_reach_v(&k, 20.0, k.length, 1e-7).unwrap();
    let jerk = scurve::max_reachable_velocity(20.0, k.length, k.accel, k.jerk);
    let reached = disk.min(jerk);
    assert!((reached - jerk).abs() < 1e-9 * jerk);
}

#[test]
fn clothoid_samples_respect_the_acceleration_disk() {
    let accel = 1000.0;
    let k = kin(0.0, 0.05, 4.0, accel, f64::INFINITY, 300.0);
    let samples = sample_profile(&k, 100.0, 70.0, 1e-8).unwrap();
    for &(s, v, _a) in &samples {
        let kappa = (k.kappa0 + k.sigma * s).abs();
        let a_c = v * v * kappa;
        assert!(a_c <= accel + 1e-3, "a_c={a_c} at s={s}");
    }
}

#[test]
fn clothoid_total_acceleration_is_within_the_disk() {
    // The emitted tangential accel `a` plus the centripetal `kappa v^2` must stay
    // inside the acceleration disk — the pass bounds `a` by the disk budget,
    // so feasibility holds without any post-hoc clamp masking it.
    let accel = 1000.0;
    let k = kin(0.0, 0.05, 4.0, accel, 80_000.0, 300.0);
    let samples = sample_profile(&k, 100.0, 70.0, 1e-8).unwrap();
    for &(s, v, a) in &samples {
        let kappa = (k.kappa0 + k.sigma * s).abs();
        let a_c = v * v * kappa;
        let total = (a * a + a_c * a_c).sqrt();
        assert!(
            total <= accel + 1.0,
            "total accel {total} exceeds a_max={accel} at s={s} (a_t={a}, a_c={a_c})"
        );
    }
}

#[test]
fn sample_profile_is_deterministic() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 80_000.0, 300.0);
    let a = sample_profile(&k, 100.0, 70.0, 1e-8);
    let b = sample_profile(&k, 100.0, 70.0, 1e-8);
    assert_eq!(a, b);
}

#[test]
fn sample_profile_endpoints_are_entry_and_exit() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 80_000.0, 300.0);
    let s = sample_profile(&k, 100.0, 70.0, 1e-8).unwrap();
    assert_eq!(s.first().unwrap().0, 0.0);
    assert_eq!(s.last().unwrap().0, k.length);
    assert_eq!(s.first().unwrap().1, 100.0);
    assert_eq!(s.last().unwrap().1, 70.0);
}

#[test]
fn limit_speed_is_infinite_for_a_line() {
    assert_eq!(limit_speed(0.0, 1000.0), f64::INFINITY);
    assert!((limit_speed(0.02, 2000.0) - (2000.0_f64 / 0.02).sqrt()).abs() < 1e-9);
}

/// Worst |jerk| between adjacent samples, time base from the trapezoid rule.
/// A true bang-bang profile sampled on a grid reads at most `j_max` plus the
/// smear of a phase switch landing inside a cell (a pointwise state next to
/// a chord slope), so bounds carry a 2× sampling allowance.
fn worst_jerk(samples: &[(f64, f64, f64)]) -> f64 {
    samples
        .windows(2)
        .map(|w| {
            let ds = w[1].0 - w[0].0;
            let dt = 2.0 * ds / (w[0].1 + w[1].1).max(1e-9);
            if dt > 1e-12 {
                ((w[1].2 - w[0].2) / dt).abs()
            } else {
                0.0
            }
        })
        .fold(0.0_f64, f64::max)
}

fn worst_accel(samples: &[(f64, f64, f64)]) -> f64 {
    samples.iter().map(|p| p.2.abs()).fold(0.0, f64::max)
}

/// Boundary states shaped like the ones the envelope settlement hands
/// `reconstruct_run`: a seam is capped by the ceilings on both of its sides,
/// never by the upstream member's alone.
fn run_members<'a>(kins: &[&'a Kinematics], exit_v: f64) -> Vec<RunMember<'a>> {
    let mut fwd = 0.0;
    let mut members: Vec<RunMember> = kins
        .iter()
        .enumerate()
        .map(|(i, k)| {
            let seam_ceiling = kins
                .get(i + 1)
                .map_or(k.flat_ceiling, |next| k.flat_ceiling.min(next.flat_ceiling));
            let m = RunMember {
                kin: k,
                exit_v: seam_ceiling,
                exit_a: 0.0,
                fwd_s: fwd,
            };
            fwd += k.length;
            m
        })
        .collect();
    members.last_mut().unwrap().exit_v = exit_v;
    members
}

fn flatten(per_member: &[Vec<(f64, f64, f64)>], members: &[RunMember]) -> Vec<(f64, f64, f64)> {
    let mut out: Vec<(f64, f64, f64)> = Vec::new();
    for (local, m) in per_member.iter().zip(members) {
        for &(s, v, a) in local {
            let s_run = m.fwd_s + s;
            if out.last().is_some_and(|p| (p.0 - s_run).abs() < 1e-12) {
                continue;
            }
            out.push((s_run, v, a));
        }
    }
    out
}

#[test]
fn straight_run_matches_the_closed_form_profile() {
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let k = kin(0.0, 0.0, 30.0, accel, jerk, flat);
    let members = run_members(&[&k], 0.0);
    let super::RunReconstruction {
        samples, phases, ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    let oracle = super::super::profile::plan(0.0, 0.0, k.length, flat, accel, jerk);
    for &(s, v, a) in &samples[0] {
        let (ov, oa) = oracle.at(s);
        assert!(
            (v - ov).abs() < 1e-6 * (1.0 + ov),
            "v={v} oracle={ov} at s={s}"
        );
        // Cap-ride samples carry the chord slope of their cell, half a cell
        // offset from the pointwise value: allow `j·dt_cell/2` of smear,
        // which grows unboundedly as the cell *time* does near rest.
        if v > 10.0 {
            assert!((a - oa).abs() < 15.0, "a={a} oracle={oa} at s={s}");
        }
    }
    assert!(!phases[0].is_empty(), "straight run must emit phases");
    let t_run: f64 = phases[0].iter().map(|p| p.dt).sum();
    let t_oracle = {
        let ramp_t = accel / jerk + flat / accel;
        let ramp_dist = 0.5 * flat * ramp_t;
        2.0 * ramp_t + (k.length - 2.0 * ramp_dist) / flat
    };
    assert!(
        (t_run - t_oracle).abs() < 1e-6 * t_oracle,
        "traversal {t_run} vs oracle {t_oracle}"
    );
}

#[test]
fn straight_run_jerk_is_bang_bang() {
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let k = kin(0.0, 0.0, 30.0, accel, jerk, flat);
    let members = run_members(&[&k], 0.0);
    let super::RunReconstruction { samples, .. } =
        reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    assert!(worst_accel(&samples[0]) <= accel + 1e-6);
    let wj = worst_jerk(&samples[0]);
    assert!(wj <= jerk * 2.0, "worst jerk {wj} (j_max {jerk})");
}

#[test]
fn apex_triangle_jerk_is_bang_bang() {
    // Short run from rest to rest: the acceleration arc must land tangent on
    // the brake envelope, not snap across it at the apex.
    let (accel, jerk, flat) = (1000.0, 1e5, 500.0);
    let k = kin(0.0, 0.0, 8.0, accel, jerk, flat);
    let members = run_members(&[&k], 0.0);
    let super::RunReconstruction {
        samples, phases, ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    assert!(worst_accel(&samples[0]) <= accel + 1e-6);
    let wj = worst_jerk(&samples[0]);
    assert!(
        wj <= jerk * 2.0,
        "worst jerk {wj} at the apex (j_max {jerk})"
    );
    assert!(!phases[0].is_empty(), "triangular straight run has phases");
}

#[test]
fn line_clothoid_line_from_rest_is_jerk_clean_on_the_straights() {
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let l1 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let c1 = kin(0.0, 0.1, 2.0, accel, jerk, flat);
    let c2 = kin(0.2, -0.1, 2.0, accel, jerk, flat);
    let l2 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let members = run_members(&[&l1, &c1, &c2, &l2], 0.0);
    let super::RunReconstruction {
        samples: per_member,
        ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    let flat_samples = flatten(&per_member, &members);
    assert!(worst_accel(&flat_samples) <= accel + 1e-6);
    // The rest-start ramp lives on the first line: it must be the same clean
    // bang-bang ramp a straight-only chain gets.
    let entry: Vec<_> = flat_samples
        .iter()
        .filter(|p| p.0 <= 5.0)
        .copied()
        .collect();
    let wj = worst_jerk(&entry);
    assert!(
        wj <= jerk * 2.0,
        "entry ramp worst jerk {wj} (j_max {jerk})"
    );
    // Velocity through the clothoid respects its curvature ceiling.
    for &(s, v, _) in &flat_samples {
        for (m, local) in members_at(&members, s) {
            let ceil = m
                .kin
                .flat_ceiling
                .min(limit_speed(m.kin.kappa_abs(local), m.kin.accel));
            assert!(
                v <= ceil + 1e-3 * (1.0 + ceil),
                "v={v} over ceiling {ceil} at s={s}"
            );
        }
    }
}

#[test]
fn mixed_run_straight_members_emit_exact_phases() {
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let l1 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let c1 = kin(0.0, 0.1, 2.0, accel, jerk, flat);
    let c2 = kin(0.2, -0.1, 2.0, accel, jerk, flat);
    let l2 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let members = run_members(&[&l1, &c1, &c2, &l2], 0.0);
    let super::RunReconstruction {
        samples: per_member,
        phases,
        ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    assert!(
        !phases[0].is_empty() && !phases[3].is_empty(),
        "straight members of a mixed run lower from exact phases"
    );
    assert!(
        !phases[1].is_empty() && !phases[2].is_empty(),
        "curved members lower from their envelope chains"
    );
    for idx in [0usize, 3] {
        let chain = &phases[idx];
        let first = &chain[0];
        assert!(
            first.t0.abs() < 1e-9 && first.s0.abs() < 1e-9,
            "member {idx} chain rebased: t0={} s0={}",
            first.t0,
            first.s0
        );
        let last = chain.last().unwrap();
        let end_s = last.s0
            + last.v0 * last.dt
            + 0.5 * last.a0 * last.dt * last.dt
            + last.j * last.dt.powi(3) / 6.0;
        assert!(
            (end_s - members[idx].kin.length).abs() < 1e-6,
            "member {idx} phases span {end_s} of {}",
            members[idx].kin.length
        );
        let s: Vec<f64> = per_member[idx].iter().map(|p| p.0).collect();
        for (&(sx, v, _), (cv, _)) in per_member[idx].iter().zip(ride::chain_states(chain, &s)) {
            assert!(
                (v - cv).abs() <= 1e-6 * (1.0 + v),
                "member {idx}: sample v={v} vs chain v={cv} at s={sx}"
            );
        }
    }
}

#[test]
fn mixed_run_entry_matches_straight_run_entry() {
    // The same rest-start on the same line must produce the same profile
    // whether or not a clothoid follows later in the chain.
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let line = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let clot = kin(0.0, 0.02, 2.0, accel, jerk, flat);
    let tail = kin(0.02 * 2.0, 0.0, 10.0, accel, jerk, flat);

    let straight_members = run_members(&[&line], 60.0);
    let super::RunReconstruction {
        samples: straight, ..
    } = reconstruct_run(&straight_members, 0.0, 0.0, 1e-8).unwrap();

    let mixed_members = run_members(&[&line, &clot, &tail], 0.0);
    let super::RunReconstruction { samples: mixed, .. } =
        reconstruct_run(&mixed_members, 0.0, 0.0, 1e-8).unwrap();

    // Compare over the entry ramp (well before either profile brakes).
    for (&(s0, v0, a0), &(s1, v1, a1)) in straight[0].iter().zip(&mixed[0]) {
        if s0 > 6.0 {
            break;
        }
        assert_eq!(s0, s1);
        assert!(
            (v0 - v1).abs() <= 1e-6 * (1.0 + v0),
            "v {v0} vs {v1} at s={s0}"
        );
        assert!((a0 - a1).abs() <= 1.0, "a {a0} vs {a1} at s={s0}");
    }
}

#[test]
fn descending_ceiling_kink_is_dipped_under_tangentially() {
    // Two flat ceilings with a step down: the profile must peel off the high
    // cruise early and land tangent under the lower ceiling, not carry a jerk
    // spike across the kink.
    let (accel, jerk) = (1000.0, 1e5);
    let fast = kin(0.0, 0.0, 20.0, accel, jerk, 100.0);
    let slow = kin(0.0, 0.0, 20.0, accel, jerk, 40.0);
    let members = run_members(&[&fast, &slow], 0.0);
    let super::RunReconstruction {
        samples: per_member,
        ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    let flat_samples = flatten(&per_member, &members);
    assert!(worst_accel(&flat_samples) <= accel + 1e-6);
    let wj = worst_jerk(&flat_samples);
    assert!(
        wj <= jerk * 2.0,
        "worst jerk {wj} across the ceiling step (j_max {jerk})"
    );
    let at_seam = flat_samples
        .iter()
        .find(|p| (p.0 - 20.0).abs() < 1e-9)
        .unwrap();
    assert!(at_seam.1 <= 40.0 + 1e-6, "seam speed {}", at_seam.1);
}

#[test]
fn infinite_jerk_curved_member_carries_smooth_phases() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, f64::INFINITY, 300.0);
    let members = run_members(&[&k], 70.0);
    let super::RunReconstruction { phases, .. } =
        reconstruct_run(&members, 100.0, 0.0, 1e-8).unwrap();
    assert!(
        !phases[0].is_empty(),
        "unlimited-jerk curved member must carry its chain"
    );
    assert!(phases[0][0].s0.abs() < 1e-9);
    let last = phases[0].last().unwrap();
    let end_s = last.s0 + last.dt * (last.v0 + last.dt * (0.5 * last.a0 + last.dt * last.j / 6.0));
    assert!(
        (end_s - k.length).abs() < 1e-6,
        "phases cover the member: end {end_s} vs length {}",
        k.length
    );
    // The varying-curvature rail must not dispatch as a per-cell staircase:
    // acceleration steps only at genuine regime corners, a handful per run,
    // not at every 0.01mm grid joint.
    let steps = phases[0]
        .windows(2)
        .filter(|w| {
            let end_a = w[0].a0 + w[0].j * w[0].dt;
            (end_a - w[1].a0).abs() > 1.0
        })
        .count();
    assert!(
        steps <= 4,
        "{steps} accel steps across {} phases — chord staircase leaked through",
        phases[0].len()
    );
}

#[test]
fn finite_jerk_curved_member_emits_its_envelope_chain() {
    let (accel, jerk, flat) = (1000.0, 1e5, 60.0);
    let l1 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let c1 = kin(0.0, 0.1, 2.0, accel, jerk, flat);
    let c2 = kin(0.2, -0.1, 2.0, accel, jerk, flat);
    let l2 = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let members = run_members(&[&l1, &c1, &c2, &l2], 0.0);
    let super::RunReconstruction {
        phases,
        envelope_chains,
        samples,
        unreachable,
        ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    assert!(unreachable.is_empty());
    for idx in [1usize, 2] {
        assert!(!phases[idx].is_empty());
        assert_eq!(phases[idx], envelope_chains[idx]);
        assert!(super::ride::chain_is_continuous(&phases[idx], true));
        assert_curved_samples_view_the_chain(&samples[idx], &phases[idx]);
    }
}

fn assert_curved_samples_view_the_chain(samples: &[(f64, f64, f64)], chain: &[StraightPhase]) {
    let arcs: Vec<f64> = samples.iter().map(|p| p.0).collect();
    for (p, (v, a)) in samples
        .iter()
        .zip(super::ride::chain_states(chain, &arcs))
        .skip(1)
        .take(arcs.len().saturating_sub(2))
    {
        assert!(
            (p.1 - v).abs() <= 1e-6 * (1.0 + v.abs()),
            "sample velocity {} is not a view of the emitted chain {v}",
            p.1
        );
        assert!((p.2 - a).abs() <= 1e-4 * (1.0 + a.abs()));
    }
}

/// Limits under which a rest-to-rest straight never reaches the flat ceiling:
/// the marcher used to leave these moves with samples and no phase chain.
const SUB_CRUISE_LIMITS: (f64, f64, f64) = (3000.0, 1e4, 500.0);

#[test]
fn sub_cruise_straight_lowers_from_the_closed_form_chain() {
    let (accel, jerk, flat) = SUB_CRUISE_LIMITS;
    for len in [10.0, 37.3] {
        let k = kin(0.0, 0.0, len, accel, jerk, flat);
        let members = run_members(&[&k], 0.0);
        let super::RunReconstruction {
            samples, phases, ..
        } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
        assert!(
            !phases[0].is_empty(),
            "len {len}: a sub-cruise straight must carry its phase chain"
        );
        let oracle = super::super::profile::plan(0.0, 0.0, len, flat, accel, jerk);
        for &(s, v, a) in &samples[0] {
            let (ov, oa) = oracle.at(s);
            assert!(
                (v - ov).abs() <= 1e-9 * (1.0 + ov),
                "len {len}: v={v} oracle={ov} at s={s}"
            );
            assert!(
                (a - oa).abs() <= 1e-6 * (1.0 + oa.abs()),
                "len {len}: a={a} oracle={oa} at s={s}"
            );
        }
        let (end_s, end_v, end_a) = phases[0].last().unwrap().end_state();
        assert!(
            (end_s - len).abs() <= 1e-9 * (1.0 + len),
            "len {len}: {end_s}"
        );
        assert!(end_v.abs() <= 1e-9, "len {len}: exits at {end_v}");
        assert!(end_a.abs() <= 1e-6, "len {len}: exits carrying {end_a}");
    }
}

#[test]
fn straight_member_samples_are_a_view_of_its_chain() {
    let (accel, jerk, flat) = SUB_CRUISE_LIMITS;
    let k = kin(0.0, 0.0, 37.3, accel, jerk, flat);
    let members = run_members(&[&k], 0.0);
    let super::RunReconstruction {
        samples, phases, ..
    } = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    let arcs: Vec<f64> = samples[0].iter().map(|p| p.0).collect();
    let interior = 1..arcs.len() - 1;
    for (i, (v, a)) in ride::chain_states(&phases[0], &arcs)
        .into_iter()
        .enumerate()
        .filter(|(i, _)| interior.contains(i))
    {
        let (s, sv, sa) = samples[0][i];
        assert_eq!((sv, sa), (v, a), "sample at s={s} drifted from the chain");
    }
}

#[test]
fn straight_chain_phases_stay_inside_the_acceleration_disk() {
    let (accel, jerk, flat) = SUB_CRUISE_LIMITS;
    let k = kin(0.0, 0.0, 10.0, accel, jerk, flat);
    let members = run_members(&[&k], 0.0);
    let super::RunReconstruction { phases, .. } =
        reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    for p in &phases[0] {
        assert!(
            super::super::certify::is_certified(&k, p.s0, p.v0, p.a0, p.j, p.dt),
            "phase {p:?} is not certified feasible"
        );
    }
}
