use std::time::Instant;

use super::certify::{CERTIFICATE_REL_TOL, certified_dwell, is_certified};
use super::disk::{Kinematics, const_kappa_reach_w};
use super::profile;

struct Phase {
    s0: f64,
    v0: f64,
    a0: f64,
    j: f64,
    dt: f64,
}

fn state_at(ph: &Phase, tau: f64) -> (f64, f64, f64) {
    (
        ph.s0 + ph.v0 * tau + 0.5 * ph.a0 * tau * tau + ph.j * tau * tau * tau / 6.0,
        ph.v0 + ph.a0 * tau + 0.5 * ph.j * tau * tau,
        ph.a0 + ph.j * tau,
    )
}

fn disk_residual(kin: &Kinematics, ph: &Phase, tau: f64) -> f64 {
    let (s, v, a) = state_at(ph, tau);
    let k = kin.kappa0 + kin.sigma * s;
    let v2 = v * v;
    kin.accel * kin.accel - a * a - k * k * v2 * v2
}

fn ball_residual(kin: &Kinematics, ph: &Phase, tau: f64) -> f64 {
    let (s, v, a) = state_at(ph, tau);
    let k = kin.kappa0 + kin.sigma * s;
    let v3 = v * v * v;
    let tangential = ph.j - k * k * v3;
    let normal = kin.sigma * v3 + 3.0 * k * v * a;
    kin.jerk * kin.jerk - tangential * tangential - normal * normal
}

fn dwell(kin: &Kinematics, ph: &Phase) -> f64 {
    certified_dwell(kin, ph.s0, ph.v0, ph.a0, ph.j, ph.dt)
}

/// First sampled `tau` at which any residual drops below the same slack the
/// certificate allows itself, or `None` if the scan stays feasible throughout.
fn scan_first_violation(kin: &Kinematics, ph: &Phase, samples: usize) -> Option<f64> {
    let disk_tol = CERTIFICATE_REL_TOL * kin.accel * kin.accel;
    let ball_tol = CERTIFICATE_REL_TOL * kin.jerk * kin.jerk;
    let speed_tol = CERTIFICATE_REL_TOL * kin.flat_ceiling;
    for i in 0..=samples {
        let tau = ph.dt * (i as f64) / (samples as f64);
        if disk_residual(kin, ph, tau) < -disk_tol
            || ball_residual(kin, ph, tau) < -ball_tol
            || state_at(ph, tau).1 < -speed_tol
        {
            return Some(tau);
        }
    }
    None
}

struct Lcg(u64);

impl Lcg {
    fn next_unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    fn in_range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_unit()
    }
}

#[test]
fn straight_phases_are_certified_for_their_full_duration() {
    let (v_max, a_max, j_max) = (300.0, 3000.0, 100_000.0);
    let cases = [
        (0.0, 0.0, 40.0),
        (0.0, 0.0, 0.5),
        (50.0, 300.0, 20.0),
        (300.0, 20.0, 12.0),
        (120.0, 120.0, 3.0),
        (5.0, 250.0, 0.8),
    ];
    let mut phases_seen = 0;
    for &(v0, v1, length) in &cases {
        let planned = profile::plan(v0, v1, length, v_max, a_max, j_max);
        let kin = Kinematics {
            length,
            accel: a_max,
            jerk: j_max,
            kappa0: 0.0,
            sigma: 0.0,
            flat_ceiling: v_max,
        };
        for sp in planned.phases() {
            let ph = Phase {
                s0: sp.s0,
                v0: sp.v0,
                a0: sp.a0,
                j: sp.j,
                dt: sp.dt,
            };
            assert_eq!(
                dwell(&kin, &ph),
                ph.dt,
                "straight phase shortened: v0={v0} v1={v1} length={length} phase=({}, {}, {}, {}, {})",
                sp.s0,
                sp.v0,
                sp.a0,
                sp.j,
                sp.dt
            );
            assert!(is_certified(&kin, sp.s0, sp.v0, sp.a0, sp.j, sp.dt));
            phases_seen += 1;
        }
    }
    assert!(
        phases_seen >= 20,
        "expected a real phase population, got {phases_seen}"
    );
}

#[test]
fn dwell_never_exceeds_a_brute_force_scan() {
    const SAMPLES: usize = 2000;
    let mut rng = Lcg(0x5EED_1234_ABCD_0001);
    let mut interior_pairs = 0;
    let mut worst_gap_rel: f64 = 0.0;
    let mut violations_found = 0;
    for _ in 0..4000 {
        let accel = rng.in_range(200.0, 5000.0);
        let jerk = rng.in_range(1.0e3, 1.0e6);
        let flat_ceiling = rng.in_range(50.0, 500.0);
        let length = rng.in_range(0.1, 50.0);
        let kin = Kinematics {
            length,
            accel,
            jerk,
            kappa0: rng.in_range(-0.2, 0.2),
            sigma: rng.in_range(-2.0, 2.0),
            flat_ceiling,
        };
        let ph = Phase {
            s0: rng.in_range(0.0, length),
            v0: rng.in_range(1.0, flat_ceiling),
            a0: rng.in_range(-accel, accel),
            j: rng.in_range(-jerk, jerk),
            dt: rng.in_range(1.0e-4, 5.0e-2),
        };
        let certified = dwell(&kin, &ph);
        assert!(
            (0.0..=ph.dt).contains(&certified),
            "dwell {certified} outside [0, {}]",
            ph.dt
        );
        let Some(violation) = scan_first_violation(&kin, &ph, SAMPLES) else {
            continue;
        };
        violations_found += 1;
        assert!(
            certified <= violation + 1.0e-9 * ph.dt,
            "certificate is optimistic: dwell {certified} > first violation {violation} \
             (dt={}, accel={accel}, jerk={jerk}, kappa0={}, sigma={}, s0={}, v0={}, a0={}, j={})",
            ph.dt,
            kin.kappa0,
            kin.sigma,
            ph.s0,
            ph.v0,
            ph.a0,
            ph.j
        );
        if certified > 0.0 && violation < ph.dt {
            interior_pairs += 1;
            let gap_rel = (violation - certified) / ph.dt;
            worst_gap_rel = worst_gap_rel.max(gap_rel);
        }
    }
    assert!(
        violations_found >= 200,
        "the random sweep barely exercised the infeasible side: {violations_found}"
    );
    assert!(
        interior_pairs >= 100,
        "not enough interior exits to judge tightness: {interior_pairs}"
    );
    assert!(
        worst_gap_rel <= 2.0 / (SAMPLES as f64) + 1.0e-6,
        "certificate lags the brute-force exit by {worst_gap_rel} of dt over {interior_pairs} cases"
    );
}

#[test]
fn an_interior_excursion_is_caught() {
    let kin = Kinematics {
        length: 100.0,
        accel: 1000.0,
        jerk: 1.0e9,
        kappa0: 0.01,
        sigma: 0.0,
        flat_ceiling: 1000.0,
    };
    let ph = Phase {
        s0: 0.0,
        v0: 300.0,
        a0: 200.0,
        j: -1000.0,
        dt: 0.4,
    };

    assert!(disk_residual(&kin, &ph, 0.0) > 0.0);
    assert!(disk_residual(&kin, &ph, ph.dt) > 0.0);
    assert!(ball_residual(&kin, &ph, 0.0) > 0.0);
    assert!(ball_residual(&kin, &ph, ph.dt) > 0.0);
    assert!(
        disk_residual(&kin, &ph, 0.5 * ph.dt) < 0.0,
        "the fixture must actually leave the disk in its interior"
    );

    let certified = dwell(&kin, &ph);
    assert!(
        certified < ph.dt,
        "an endpoint-only check would pass this phase; the certificate must not"
    );
    assert!(certified > 0.0, "the phase starts feasible: {certified}");
    assert!(!is_certified(&kin, ph.s0, ph.v0, ph.a0, ph.j, ph.dt));

    let violation = scan_first_violation(&kin, &ph, 200_000).expect("fixture violates in scan");
    assert!(certified <= violation);
    assert!(violation - certified <= 1.0e-4 * ph.dt);
}

#[test]
fn arc_agrees_with_the_closed_form_reach() {
    let accel = 2000.0_f64;
    let jerk = 1.0e9_f64;
    for &kappa in &[0.002_f64, 0.01, 0.05, 0.2] {
        let v_lim = (accel / kappa).sqrt();
        let v0 = 0.6 * v_lim;
        let w_in = v0 * v0;
        let length = 0.01 * w_in / accel;
        let kin = Kinematics {
            length,
            accel,
            jerk,
            kappa0: kappa,
            sigma: 0.0,
            flat_ceiling: 10.0 * v_lim,
        };
        let span = |a: f64| (-v0 + (v0 * v0 + 2.0 * a * length).sqrt()) / a;
        let mut lo = 0.0;
        let mut hi = accel;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if is_certified(&kin, 0.0, v0, mid, 0.0, span(mid)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let w_end = w_in + 2.0 * lo * length;
        let w_reach = const_kappa_reach_w(w_in, length, accel, kappa);
        assert!(
            w_end <= w_reach * (1.0 + 1.0e-9),
            "kappa={kappa}: certified reach {w_end} beats the disk-riding closed form {w_reach}"
        );
        assert!(
            w_reach - w_end <= 0.05 * (w_reach - w_in),
            "kappa={kappa}: certified reach {w_end} lags {w_reach} (entry {w_in})"
        );
    }
}

#[test]
fn zero_jerk_and_zero_curvature_certify_the_full_span() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: 100_000.0,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    let ph = Phase {
        s0: 1.0,
        v0: 120.0,
        a0: 1500.0,
        j: 0.0,
        dt: 0.02,
    };
    assert_eq!(dwell(&kin, &ph), ph.dt);
    assert!(is_certified(&kin, ph.s0, ph.v0, ph.a0, ph.j, ph.dt));
}

#[test]
fn zero_dt_certifies_trivially() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: 100_000.0,
        kappa0: 0.05,
        sigma: 0.3,
        flat_ceiling: 300.0,
    };
    assert_eq!(certified_dwell(&kin, 0.0, 100.0, 0.0, 0.0, 0.0), 0.0);
    assert!(is_certified(&kin, 0.0, 100.0, 0.0, 0.0, 0.0));
}

#[test]
fn velocity_at_the_flat_ceiling_is_certified_when_the_disk_allows_it() {
    let flat_ceiling = 200.0;
    let accel = 3000.0;
    let kappa0 = 0.5 * accel / (flat_ceiling * flat_ceiling);
    let kin = Kinematics {
        length: 10.0,
        accel,
        jerk: 500_000.0,
        kappa0,
        sigma: 0.0,
        flat_ceiling,
    };
    let ph = Phase {
        s0: 0.0,
        v0: flat_ceiling,
        a0: 0.0,
        j: 0.0,
        dt: 0.01,
    };
    assert!(disk_residual(&kin, &ph, 0.0) > 0.0);
    assert_eq!(dwell(&kin, &ph), ph.dt);
}

#[test]
fn acceleration_on_the_disk_edge_holds_but_cannot_grow() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: 200_000.0,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    let holding = Phase {
        s0: 0.0,
        v0: 100.0,
        a0: kin.accel,
        j: 0.0,
        dt: 0.01,
    };
    assert_eq!(dwell(&kin, &holding), holding.dt);

    let growing = Phase {
        s0: 0.0,
        v0: 100.0,
        a0: kin.accel,
        j: 1000.0,
        dt: 0.01,
    };
    let certified = dwell(&kin, &growing);
    assert!(
        certified <= 1.0e-6 * growing.dt,
        "leaving the disk edge upward must not certify: {certified}"
    );
}

#[test]
#[should_panic(expected = "certify: v0 must be finite")]
fn non_finite_state_fails_loudly() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: 100_000.0,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    certified_dwell(&kin, 0.0, f64::NAN, 0.0, 0.0, 0.01);
}

#[test]
#[should_panic(expected = "certify: dt must be nonnegative")]
fn negative_dt_fails_loudly() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: 100_000.0,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    certified_dwell(&kin, 0.0, 100.0, 0.0, 0.0, -1.0e-6);
}

#[test]
#[should_panic(expected = "kinematics.accel must be strictly positive")]
fn degenerate_kinematics_fail_loudly() {
    let kin = Kinematics {
        length: 10.0,
        accel: 0.0,
        jerk: 100_000.0,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    certified_dwell(&kin, 0.0, 100.0, 0.0, 0.0, 0.01);
}

#[test]
#[should_panic(expected = "kinematics.jerk must be finite")]
fn infinite_jerk_fails_loudly() {
    let kin = Kinematics {
        length: 10.0,
        accel: 3000.0,
        jerk: f64::INFINITY,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 300.0,
    };
    certified_dwell(&kin, 0.0, 100.0, 0.0, 0.0, 0.01);
}

#[test]
fn certified_dwell_cost_is_measured() {
    let mut rng = Lcg(0xC057_0001);
    let mut cases = Vec::new();
    for _ in 0..1000 {
        let accel = rng.in_range(500.0, 4000.0);
        let jerk = rng.in_range(1.0e4, 1.0e6);
        let kin = Kinematics {
            length: 20.0,
            accel,
            jerk,
            kappa0: rng.in_range(-0.1, 0.1),
            sigma: rng.in_range(-1.0, 1.0),
            flat_ceiling: 300.0,
        };
        let ph = Phase {
            s0: rng.in_range(0.0, 20.0),
            v0: rng.in_range(10.0, 300.0),
            a0: rng.in_range(-accel, accel),
            j: rng.in_range(-jerk, jerk),
            dt: rng.in_range(1.0e-4, 2.0e-2),
        };
        cases.push((kin, ph));
    }
    let mut full = 0usize;
    let mut sink = 0.0;
    for (kin, ph) in &cases {
        if dwell(kin, ph) >= ph.dt {
            full += 1;
        }
    }
    let started = Instant::now();
    for (kin, ph) in &cases {
        sink += dwell(kin, ph);
    }
    let per_call_us = started.elapsed().as_secs_f64() * 1.0e6 / (cases.len() as f64);
    eprintln!(
        "certified_dwell: {per_call_us:.3} us/call over {} cases ({full} certified in full, sink={sink})",
        cases.len()
    );
    assert!(
        per_call_us < 500.0,
        "certified_dwell cost blew past a sane ceiling: {per_call_us} us/call"
    );
}

#[test]
fn a_terminal_sliver_violation_is_not_forgiven() {
    let kin = Kinematics {
        length: 100.0,
        accel: 1000.0,
        jerk: 1.0e6,
        kappa0: 0.0,
        sigma: 0.0,
        flat_ceiling: 500.0,
    };
    let (dt, j) = (0.01, 1.0e5);
    let crosses_the_rail_at_the_very_end = Phase {
        s0: 0.0,
        v0: 100.0,
        a0: kin.accel - j * dt * (1.0 - 5.0e-10),
        j,
        dt,
    };
    let ph = &crosses_the_rail_at_the_very_end;
    let disk_tol = CERTIFICATE_REL_TOL * kin.accel * kin.accel;
    let end = disk_residual(&kin, ph, dt);
    assert!(
        end < -100.0 * disk_tol,
        "the fixture must genuinely violate the disk at dt: residual {end} against tol {disk_tol}"
    );
    assert!(
        disk_residual(&kin, ph, 0.0) > disk_tol,
        "the fixture must start feasible"
    );
    assert!(
        !is_certified(&kin, ph.s0, ph.v0, ph.a0, ph.j, ph.dt),
        "is_certified forgave a violation {}x beyond its own disk tolerance",
        end.abs() / disk_tol
    );
}
