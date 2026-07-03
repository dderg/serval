use super::*;

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

fn single_move_anchors(entry: f64, exit: f64) -> JerkAnchors {
    JerkAnchors {
        fwd_a: 0.0,
        fwd_v: entry,
        fwd_s: 0.0,
        bwd_v: exit,
        bwd_s: 0.0,
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
    let samples = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
    for &(s, v, _a) in &samples {
        let kappa = (k.kappa0 + k.sigma * s).abs();
        let a_c = v * v * kappa;
        assert!(a_c <= accel + 1e-3, "a_c={a_c} at s={s}");
    }
}

#[test]
fn clothoid_total_acceleration_is_within_the_disk() {
    // The emitted tangential accel `a` plus the centripetal `kappa v^2` must stay
    // inside the acceleration disk — the jerk-ride bounds `a` by the disk budget,
    // so feasibility holds without any post-hoc clamp masking it.
    let accel = 1000.0;
    let k = kin(0.0, 0.05, 4.0, accel, 80_000.0, 300.0);
    let samples = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
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
    let a = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8);
    let b = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8);
    assert_eq!(a, b);
}

#[test]
fn sample_profile_endpoints_are_entry_and_exit() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 80_000.0, 300.0);
    let s = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
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

fn dump_stats(name: &str, samples: &[(f64, f64, f64)], a_max: f64, j_max: f64) {
    let mut worst_a = 0.0_f64;
    let mut worst_j = 0.0_f64;
    let mut worst_j_s = 0.0_f64;
    for w in samples.windows(2) {
        let (s0, v0, a0) = w[0];
        let (s1, v1, a1) = w[1];
        let ds = s1 - s0;
        if ds <= 1e-12 {
            continue;
        }
        let dt = 2.0 * ds / (v0 + v1).max(1e-9);
        let j = (a1 - a0) / dt;
        if j.abs() > worst_j {
            worst_j = j.abs();
            worst_j_s = s0;
        }
        worst_a = worst_a.max(a0.abs()).max(a1.abs());
    }
    println!(
        "{name}: n={} worst_a={worst_a:.1} (a_max={a_max}) worst_j={worst_j:.0} at s={worst_j_s:.4} (j_max={j_max})",
        samples.len()
    );
}

#[test]
fn diag_straight_grid_vs_closed() {
    let k = kin(0.0, 0.0, 30.0, 1000.0, 1e5, 60.0);
    let member = RunMember {
        kin: &k,
        entry_v: 0.0,
        exit_v: 0.0,
        fwd_s: 0.0,
        bwd_s: 0.0,
    };
    let members = [member];
    let (closed, _) = reconstruct_straight(&members, 0.0, 60.0, 1000.0, 1e5);
    dump_stats("closed", &closed[0], 1000.0, 1e5);

    let ctxs = build_ctxs(&members, 0.0, 0.0).unwrap();
    let (grid, _) = reconstruct_flat(&ctxs, 0.0, 1e-8).unwrap();
    dump_stats("grid", &grid, 1000.0, 1e5);
    for &(s, v, a) in grid.iter().take(40) {
        println!("  s={s:.5} v={v:.4} a={a:.1}");
    }
    let vmax = grid.iter().fold(0.0_f64, |m, p| m.max(p.1));
    println!("grid vmax={vmax}");
}

#[test]
fn diag_line_clothoid_line_from_rest() {
    // 10mm line -> 2mm clothoid ramping to kappa=0.2 -> 10mm line back to zero curvature
    let l1 = kin(0.0, 0.0, 10.0, 1000.0, 1e5, 60.0);
    let c1 = kin(0.0, 0.1, 2.0, 1000.0, 1e5, 60.0);
    let c2 = kin(0.2, -0.1, 2.0, 1000.0, 1e5, 60.0);
    let l2 = kin(0.0, 0.0, 10.0, 1000.0, 1e5, 60.0);
    let kins = [&l1, &c1, &c2, &l2];
    let mut fwd = 0.0;
    let total: f64 = kins.iter().map(|k| k.length).sum();
    let mut members = Vec::new();
    for k in kins {
        members.push(RunMember {
            kin: k,
            entry_v: 60.0,
            exit_v: 60.0,
            fwd_s: fwd,
            bwd_s: total - fwd - k.length,
        });
        fwd += k.length;
    }
    members[0].entry_v = 0.0;
    members[3].exit_v = 0.0;
    let ctxs = build_ctxs(&members, 0.0, 0.0).unwrap();
    let (grid, _) = reconstruct_flat(&ctxs, 0.0, 1e-8).unwrap();
    dump_stats("line-clothoid-line", &grid, 1000.0, 1e5);
    for &(s, v, a) in grid.iter().take(30) {
        println!("  s={s:.5} v={v:.4} a={a:.1}");
    }
}

#[test]
fn diag_apex_triangle() {
    // Short run from rest to rest: accel meets brake, triangular peak (image 3 shape).
    let k = kin(0.0, 0.0, 8.0, 1000.0, 1e5, 500.0);
    let member = RunMember {
        kin: &k,
        entry_v: 0.0,
        exit_v: 0.0,
        fwd_s: 0.0,
        bwd_s: 0.0,
    };
    let members = [member];
    // force grid path by pretending curvature? no: call reconstruct_flat directly
    let ctxs = build_ctxs(&members, 0.0, 0.0).unwrap();
    let (grid, _) = reconstruct_flat(&ctxs, 0.0, 1e-8).unwrap();
    dump_stats("apex-grid", &grid, 1000.0, 1e5);
    let (closed, _) = reconstruct_straight(&members, 0.0, 500.0, 1000.0, 1e5);
    dump_stats("apex-closed", &closed[0], 1000.0, 1e5);
    // print around the apex
    let apex = grid
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.1.partial_cmp(&b.1.1).unwrap())
        .unwrap()
        .0;
    for &(s, v, a) in &grid[apex.saturating_sub(10)..(apex + 10).min(grid.len())] {
        println!("  s={s:.5} v={v:.4} a={a:.1}");
    }
}
