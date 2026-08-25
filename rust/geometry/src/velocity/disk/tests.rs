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

fn run_members<'a>(kins: &[&'a Kinematics], exit_v: f64) -> Vec<RunMember<'a>> {
    let mut fwd = 0.0;
    let mut members: Vec<RunMember> = kins
        .iter()
        .map(|k| {
            let m = RunMember {
                kin: k,
                exit_v: k.flat_ceiling,
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
    let (samples, _, phases) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
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
    let (samples, _, _) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
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
    let (samples, _, phases) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
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
    let (per_member, _, _) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
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
    let (per_member, _, phases) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
    assert!(
        !phases[0].is_empty() && !phases[3].is_empty(),
        "straight members of a mixed run lower from exact phases"
    );
    assert!(
        phases[1].is_empty() && phases[2].is_empty(),
        "curved members lower by fitting"
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
    let (straight, _, _) = reconstruct_run(&straight_members, 0.0, 0.0, 1e-8).unwrap();

    let mixed_members = run_members(&[&line, &clot, &tail], 0.0);
    let (mixed, _, _) = reconstruct_run(&mixed_members, 0.0, 0.0, 1e-8).unwrap();

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
    let (per_member, _, _) = reconstruct_run(&members, 0.0, 0.0, 1e-8).unwrap();
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
    let (_, _, phases) = reconstruct_run(&members, 100.0, 0.0, 1e-8).unwrap();
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
fn finite_jerk_curved_member_still_carries_no_phases() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 1e5, 300.0);
    let members = run_members(&[&k], 70.0);
    let (_, _, phases) = reconstruct_run(&members, 100.0, 0.0, 1e-8).unwrap();
    assert!(phases[0].is_empty());
}

fn grid_of(kins: &[&Kinematics], entry_rest: bool, exit_rest: bool) -> Vec<f64> {
    let mut fwd = 0.0;
    let members: Vec<RunMember> = kins
        .iter()
        .map(|k| {
            let m = RunMember {
                kin: k,
                exit_v: k.flat_ceiling,
                fwd_s: fwd,
            };
            fwd += k.length;
            m
        })
        .collect();
    seed_grid(&members, entry_rest, exit_rest)
}

fn straight_kin(length: f64, ceiling: f64) -> Kinematics {
    kin(0.0, 0.0, length, 20_000.0, 40_000.0, ceiling)
}

fn curved_kin(length: f64) -> Kinematics {
    kin(0.02, 0.0, length, 20_000.0, 40_000.0, 600.0)
}

fn grid_for(length: f64, entry_rest: bool, exit_rest: bool) -> Vec<f64> {
    let k = straight_kin(length, 600.0);
    grid_of(&[&k], entry_rest, exit_rest)
}

#[test]
fn integration_grid_keeps_the_fine_step_on_a_short_member() {
    let grid = grid_for(0.5, false, false);
    let widest = grid.windows(2).map(|w| w[1] - w[0]).fold(0.0_f64, f64::max);
    assert!(
        (widest - GRID_STEP_MM).abs() <= 1e-12,
        "a member short enough to want GRID_STEP_MM must still get it, widest {widest}"
    );
}

#[test]
fn integration_grid_caps_nodes_on_a_long_curved_member() {
    let k = curved_kin(40.0);
    let grid = grid_of(&[&k], false, false);
    // Uncapped, 40 mm at GRID_STEP_MM would be 4000 nodes, and every one of
    // them becomes a reconstruction window the lowering pays a piece for.
    assert!(
        grid.len() <= MEMBER_SEED_MAX_POINTS + 1,
        "40 mm curved member produced {} nodes, cap is {MEMBER_SEED_MAX_POINTS}",
        grid.len()
    );
}

#[test]
fn seed_grid_caps_nodes_on_a_long_straight_member() {
    // A straight member is not exempt from the seed cap. `repro_z14.gcode`
    // line 2710 is a 15.6 mm straight cruising at the step-rate ceiling:
    // seeded at the grid pitch it takes 1560 nodes, lowers to 2336 pieces for
    // one shaped segment, and costs 3.5 s of shaper convolution fits on a
    // Pi 4 — a whole pump lead spent on one segment, which lands the next
    // lane's piece in the MCU's past. Only a member the reconstruction
    // convicts of ringing buys those nodes back.
    let grid = grid_for(40.0, false, false);
    assert!(
        grid.len() <= MEMBER_SEED_MAX_POINTS + 1,
        "40 mm straight member produced {} nodes, cap is {MEMBER_SEED_MAX_POINTS}",
        grid.len()
    );
}

#[test]
fn rest_ladder_leaves_no_hole_up_to_the_uniform_step() {
    // The node cap widens a long curved member's uniform step past
    // GRID_STEP_MM. The rest ladder has to climb all the way to that step:
    // stopping at GRID_STEP_MM would leave the pass jumping from a 0.005 mm
    // rung straight to a 0.16 mm one, and the profile's first arc out of rest
    // steps its acceleration across the hole.
    let length = 40.0;
    let k = curved_kin(length);
    let grid = grid_of(&[&k], true, true);
    let step = length / MEMBER_SEED_MAX_POINTS as f64;
    assert!(
        step > GRID_STEP_MM,
        "fixture must exercise the widened step"
    );
    let gaps: Vec<f64> = grid.windows(2).map(|w| w[1] - w[0]).collect();
    for pair in gaps.windows(2) {
        assert!(
            // The ladder doubles, and its top rung meets the first uniform
            // node part-way, so one transition gap can be up to ~2.5x its
            // predecessor. Stopping the ladder at GRID_STEP_MM instead made
            // that ratio ~60x.
            pair[1] <= 2.5 * pair[0] + 1e-12 || pair[1] <= GRID_STEP_MM + 1e-12,
            "grid spacing jumped {} -> {} (ladder must stay geometric until it \
             meets the uniform step {step})",
            pair[0],
            pair[1]
        );
    }
}

#[test]
fn member_boundary_cell_holds_the_fine_step_under_the_node_cap() {
    // The ceiling is piecewise constant per member, so the cell straddling a
    // boundary is where the pass reads the ceiling *step*, as the chord
    // `Δ(v²)/2Δs`. A capped member's 0.16 mm cell shallows that chord into the
    // band the pass classifies as a followable descent it must commit a
    // whole-run brake to; at GRID_STEP_MM the chord stays the super-rail step
    // it is.
    let (a, b) = (curved_kin(40.0), curved_kin(40.0));
    let grid = grid_of(&[&a, &b], false, false);
    let boundary = a.length;
    let straddling = grid
        .windows(2)
        .find(|w| w[0] < boundary - 1e-12 && w[1] >= boundary - 1e-12)
        .expect("a cell must straddle the member boundary");
    assert!(
        straddling[1] - straddling[0] <= GRID_STEP_MM + 1e-12,
        "boundary cell is {} mm wide; the ceiling step it carries reads as a \
         shallow ramp instead of a step",
        straddling[1] - straddling[0]
    );
}

#[test]
fn stepped_ceilings_across_long_members_never_brake_toward_rest() {
    // The speed_ramp shape: collinear 40 mm members whose feedrate steps
    // 10 → 30 → 80 → 200 → 10 mm/s. Every step is a ceiling discontinuity at a
    // member boundary, and the run has to cruise each plateau. A boundary cell
    // wide enough to shallow one of those steps sends the profile pass into a
    // committed peel tens of millimetres early, which runs the state past the
    // accel rail and stalls the profile at rest — a sawtooth where the plan
    // holds a plateau.
    let floor = 10.0;
    let ceilings = [floor, 30.0, 80.0, 200.0, floor];
    let kins: Vec<Kinematics> = ceilings
        .iter()
        .map(|&c| kin(0.0, 0.0, 40.0, 3000.0, 1e5, c))
        .collect();
    let refs: Vec<&Kinematics> = kins.iter().collect();
    // Anchored at the floor on both ends, not at rest: every velocity in the
    // run is then bounded below by the floor as a matter of physics, with no
    // ramp out of an anchor to except.
    let members = run_members(&refs, floor);
    let (samples, _, phases) = reconstruct_run(&members, floor, 0.0, 1e-8).unwrap();
    assert!(
        phases.iter().any(|p| !p.is_empty()),
        "the straight run's phase chain came back empty — the pass stalled and \
         fell back to node samples"
    );
    for (i, member) in samples.iter().enumerate() {
        let dip = member
            .iter()
            .map(|&(_, v, _)| v)
            .fold(f64::INFINITY, f64::min);
        assert!(
            dip >= floor - 1e-6,
            "member {i} dipped to {dip} mm/s; the slowest ceiling in the run is \
             {floor} mm/s and both anchors sit at it"
        );
    }
}

/// The `repro_z14.gcode` line 3333 run, as the fitter hands it to the
/// planner: a 15.7 mm straight cruising at its F3600 ceiling with a clothoid
/// blend pair at each end, on the bench's 20000 mm/s² / 40000 mm/s³ limits.
fn ceiling_riding_straight(jerk: f64) -> (Vec<Kinematics>, Vec<f64>) {
    let (accel, ceiling) = (20_000.0, 60.0);
    let kins = vec![
        kin(
            0.0,
            73.74995326051372,
            0.13728400896857004,
            accel,
            jerk,
            ceiling,
        ),
        kin(
            10.124689244847987,
            -73.74995326051372,
            0.13728400896857004,
            accel,
            jerk,
            ceiling,
        ),
        kin(0.0, 0.0, 15.672504385558916, accel, jerk, ceiling),
        kin(
            0.0,
            83.07884272157982,
            0.13125886761095884,
            accel,
            jerk,
            ceiling,
        ),
        kin(
            10.904834818063517,
            -83.07884272157982,
            0.13125886761095884,
            accel,
            jerk,
            ceiling,
        ),
    ];
    let exits = vec![
        44.44512650163059,
        ceiling,
        ceiling,
        42.8257968079636,
        ceiling,
    ];
    (kins, exits)
}

fn blended_run<'a>(kins: &'a [Kinematics], exits: &[f64]) -> Vec<RunMember<'a>> {
    let mut fwd = 0.0;
    kins.iter()
        .zip(exits)
        .map(|(k, &exit_v)| {
            let m = RunMember {
                kin: k,
                exit_v,
                fwd_s: fwd,
            };
            fwd += k.length;
            m
        })
        .collect()
}

fn plateau(samples: &[(f64, f64, f64)], m: &RunMember) -> Vec<(f64, f64, f64)> {
    let (s0, s1) = (m.fwd_s + 3.0, m.fwd_s + m.kin.length - 3.0);
    samples
        .iter()
        .copied()
        .filter(|p| p.0 > s0 && p.0 < s1)
        .collect()
}

#[test]
fn the_seeded_grid_rings_on_a_ceiling_riding_straight() {
    // The regression this guards: without refinement the seeded 256 nodes put
    // the pass into a limit cycle across the cruise — it touches the ceiling,
    // peels, jerks back up and touches again, once per ~1.5 mm.
    let (kins, exits) = ceiling_riding_straight(40_000.0);
    let members = blended_run(&kins, &exits);
    let seeded: Vec<usize> = kins.iter().map(member_seed_steps).collect();
    let grid = grid_from_steps(&members, &seeded, false, false);
    let samples = reconstruct_flat_on(&members, &grid, 60.0, 0.0).unwrap().0;
    let straight = &members[2];
    let reversals = accel_reversals(
        &samples,
        straight.fwd_s,
        straight.fwd_s + straight.kin.length,
    );
    assert!(
        reversals > PROFILE_REVERSALS_MAX,
        "the fixture must ring at the seed, saw {reversals} reversals"
    );
    let cruise = plateau(&samples, straight);
    let ripple = cruise.iter().map(|p| p.2.abs()).fold(0.0_f64, f64::max);
    assert!(
        ripple > 200.0,
        "the fixture must ring hard enough to matter, peak |a| {ripple} mm/s²"
    );
}

#[test]
fn refinement_holds_the_ceiling_on_a_ringing_straight() {
    let (kins, exits) = ceiling_riding_straight(40_000.0);
    let members = blended_run(&kins, &exits);
    let samples = reconstruct_flat(&members, 60.0, 0.0).unwrap().0;
    let straight = &members[2];
    let reversals = accel_reversals(
        &samples,
        straight.fwd_s,
        straight.fwd_s + straight.kin.length,
    );
    assert!(
        reversals <= PROFILE_REVERSALS_MAX,
        "refined reconstruction still rings: {reversals} reversals"
    );
    let cruise = plateau(&samples, straight);
    let ripple = cruise.iter().map(|p| p.2.abs()).fold(0.0_f64, f64::max);
    assert!(
        ripple <= GRID_ACCEL_TOL_MM_S2,
        "cruise carries {ripple} mm/s² of acceleration the plan never asked \
         for (budget {GRID_ACCEL_TOL_MM_S2})"
    );
    let dip = 60.0 - cruise.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    assert!(dip <= 0.05, "cruise dips {dip} mm/s below its own ceiling");
}

#[test]
fn refined_reconstruction_tracks_a_dense_grid_reference() {
    // The dense reference is the same pass at ten times the grid pitch, which
    // is what the seeded grid is being judged against: a member whose profile
    // the grid resolves says the same thing at any finer spacing.
    let (kins, exits) = ceiling_riding_straight(40_000.0);
    let members = blended_run(&kins, &exits);
    let dense: Vec<usize> = kins
        .iter()
        .map(|k| ((k.length / (0.1 * GRID_STEP_MM)).ceil() as usize).max(GRID_MIN_STEPS))
        .collect();
    let reference = reconstruct_flat_on(
        &members,
        &grid_from_steps(&members, &dense, false, false),
        60.0,
        0.0,
    )
    .unwrap()
    .0;
    let refined = reconstruct_flat(&members, 60.0, 0.0).unwrap().0;
    let straight = &members[2];
    let (mut dv, mut da) = (0.0_f64, 0.0_f64);
    for &(s, v, a) in &plateau(&refined, straight) {
        let (rv, ra) = interp_flat(&reference, s).unwrap();
        dv = dv.max((v - rv).abs());
        da = da.max((a - ra).abs());
    }
    assert!(
        da <= GRID_ACCEL_TOL_MM_S2,
        "acceleration is {da} mm/s² off the dense reference (budget \
         {GRID_ACCEL_TOL_MM_S2})"
    );
    assert!(dv <= 0.05, "velocity is {dv} mm/s off the dense reference");
}

#[test]
fn reversals_the_grid_cannot_retire_keep_the_seeded_grid() {
    // At 200000 mm/s³ the same run hunts at every spacing down to a tenth of
    // the grid pitch — the reversals are the pass's, not the grid's. Spending
    // nodes on them would buy a denser copy of the same profile, so the
    // regrid is measured and rolled back.
    let (kins, exits) = ceiling_riding_straight(200_000.0);
    let members = blended_run(&kins, &exits);
    let seeded: Vec<usize> = kins.iter().map(member_seed_steps).collect();
    let seed_grid_nodes = grid_from_steps(&members, &seeded, false, false);
    let straight = &members[2];
    let within = |nodes: &[f64]| {
        nodes
            .iter()
            .filter(|&&x| x >= straight.fwd_s && x <= straight.fwd_s + straight.kin.length)
            .count()
    };
    let seed_nodes = within(&seed_grid_nodes);
    let samples = reconstruct_flat(&members, 60.0, 0.0).unwrap().0;
    assert!(
        accel_reversals(
            &samples,
            straight.fwd_s,
            straight.fwd_s + straight.kin.length
        ) > PROFILE_REVERSALS_MAX,
        "fixture must be one the grid cannot fix"
    );
    let sample_arcs: Vec<f64> = samples.iter().map(|p| p.0).collect();
    assert_eq!(
        within(&sample_arcs),
        seed_nodes,
        "a member the grid cannot help kept {} nodes instead of the seeded \
         {seed_nodes}",
        within(&sample_arcs)
    );
}

#[test]
fn sub_budget_acceleration_wobble_is_not_a_reversal() {
    let noise: Vec<(f64, f64, f64)> = (0..40)
        .map(|i| {
            let s = i as f64 * 0.01;
            (s, 60.0, if i % 2 == 0 { 12.0 } else { -12.0 })
        })
        .collect();
    assert_eq!(accel_reversals(&noise, 0.0, 0.39), 0);
    let executed: Vec<(f64, f64, f64)> = (0..40)
        .map(|i| {
            let s = i as f64 * 0.01;
            (s, 60.0, if i % 2 == 0 { 300.0 } else { -300.0 })
        })
        .collect();
    assert_eq!(accel_reversals(&executed, 0.0, 0.39), 39);
}

#[test]
fn a_straight_run_that_cruises_cleanly_is_never_regridded() {
    // Refinement must be paid for by evidence: a member the seed already
    // resolves keeps the seed's node count exactly.
    let k = straight_kin(40.0, 600.0);
    let members = run_members(&[&k], 600.0);
    let seed = grid_from_steps(&members, &[member_seed_steps(&k)], false, false);
    let samples = reconstruct_flat(&members, 600.0, 0.0).unwrap().0;
    assert_eq!(samples.len(), seed.len());
}

#[test]
fn refinement_runs_out_of_room_at_four_times_the_grid_pitch() {
    // The ceiling is what makes `GridBudget` a verdict rather than a loop:
    // past it there is no finer grid to try, so a member still ringing there
    // is rejected instead of quietly planned.
    let k = straight_kin(15.0, 60.0);
    let pitch = (k.length / GRID_STEP_MM).ceil() as usize;
    assert_eq!(refine_step_count(&k, member_seed_steps(&k)), pitch);
    let mut steps = pitch;
    for _ in 0..8 {
        steps = refine_step_count(&k, steps);
    }
    assert_eq!(steps, pitch * GRID_REFINE_GROWTH);
    assert_eq!(
        refine_step_count(&k, steps),
        steps,
        "the ceiling must stick"
    );
}

#[test]
fn chaining_a_phase_run_lands_on_its_true_end_state() {
    // The chained phase is what the lowering turns into one piece, so its end
    // arc and end velocity are the seam the next piece starts from: they must
    // be the run's own, not the leading phase's extrapolation.
    let residue = 4.0e-4;
    let chain: Vec<StraightPhase> = (0..64)
        .map(|i| StraightPhase {
            t0: i as f64 * 0.001,
            dt: 0.001,
            s0: 60.0 * i as f64 * 0.001,
            v0: 60.0,
            a0: if i % 2 == 0 { 0.0 } else { residue },
            j: 0.0,
        })
        .collect();
    let last = chain[chain.len() - 1];
    let s_end = last.s0 + last.dt * (last.v0 + 0.5 * last.a0 * last.dt);
    let v_end = last.v0 + last.a0 * last.dt;
    let merged = merge_constant_accel(chain);
    assert_eq!(
        merged.len(),
        1,
        "a run inside the residue must chain to one"
    );
    let one = merged[0];
    let got_s = one.s0 + one.dt * (one.v0 + one.dt * (0.5 * one.a0 + one.j * one.dt / 6.0));
    let got_v = one.v0 + one.dt * (one.a0 + 0.5 * one.j * one.dt);
    assert!((got_s - s_end).abs() < 1e-12, "end arc {got_s} vs {s_end}");
    assert!(
        (got_v - v_end).abs() < 1e-12,
        "end speed {got_v} vs {v_end}"
    );
}

#[test]
fn a_real_acceleration_change_is_never_chained_away() {
    // Flattening two genuinely different accelerations into one phase leaves
    // the difference as a step at the far joint. The spread gate is what
    // keeps that from happening.
    let step = 2.0 * CHAIN_MERGE_ACCEL_MM_S2;
    let chain = vec![
        StraightPhase {
            t0: 0.0,
            dt: 0.001,
            s0: 0.0,
            v0: 60.0,
            a0: 0.0,
            j: 0.0,
        },
        StraightPhase {
            t0: 0.001,
            dt: 0.001,
            s0: 0.06,
            v0: 60.0,
            a0: step,
            j: 0.0,
        },
    ];
    assert_eq!(merge_constant_accel(chain).len(), 2);
}
