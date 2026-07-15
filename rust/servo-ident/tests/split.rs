use servo_ident::capture::Capture;
use servo_ident::model::{coulomb_sign, PhysicalParams, Structure};
use servo_ident::split::{fit_pair_splits, SplitCapture};

const DT: f64 = 0.001;

fn frame() -> Vec<Vec<f64>> {
    vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ]
}

fn params() -> PhysicalParams {
    PhysicalParams {
        mass: vec![0.012, 0.011],
        viscous: vec![0.09, 0.11],
        coulomb: vec![160.0, 175.0],
    }
}

const SIGNS: [f64; 4] = [1.0, -1.0, -1.0, -1.0];

/// Deterministic trapezoid belt motion: identical segment lengths for every
/// belt so two belts stay sample-aligned, with a caller-chosen direction
/// pattern and amplitude so the belts and their positions are independent.
fn belt(dirs: &[f64], accel: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut pos = Vec::new();
    let mut v = 0.0;
    let mut p = 0.0;
    for (rep, &dir) in dirs.iter().enumerate() {
        let ramp = 40;
        let cruise = 30 + 6 * (rep % 4);
        for phase in 0..(2 * ramp + cruise) {
            let a = if phase < ramp {
                dir * accel
            } else if phase < ramp + cruise {
                0.0
            } else {
                -dir * accel
            };
            v += a * DT;
            p += v * DT;
            acc.push(a);
            vel.push(v);
            pos.push(p);
        }
    }
    (acc, vel, pos)
}

fn white(k: usize, salt: usize) -> f64 {
    let h = (k.wrapping_mul(2654435761) ^ salt.wrapping_mul(40503)) as u32;
    f64::from(h % 1000) / 1000.0 - 0.5
}

struct Slots {
    acc: Vec<Vec<f64>>,
    vel: Vec<Vec<f64>>,
    pos: Vec<Vec<f64>>,
}

/// Two independent belts driving a 4-slot AWD frame. Pair (0,1) has λ=−1 so
/// its drive-frame commands are antiparallel (`slot1 = −slot0`); pair (2,3)
/// has λ=+1 so its commands are parallel (`slot3 = slot2`).
fn four_slot_motion() -> Slots {
    let dirs_a = [
        1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0,
    ];
    let dirs_b = [
        1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0,
    ];
    let (aa, va, pa) = belt(&dirs_a, 5200.0);
    let (ab, vb, pb) = belt(&dirs_b, 4100.0);
    let neg = |x: &[f64]| x.iter().map(|v| -v).collect::<Vec<f64>>();
    Slots {
        acc: vec![aa.clone(), neg(&aa), ab.clone(), ab.clone()],
        vel: vec![va.clone(), neg(&va), vb.clone(), vb.clone()],
        pos: vec![pa.clone(), neg(&pa), pb.clone(), pb.clone()],
    }
}

fn belt_forces(
    frame: &[Vec<f64>],
    p: &PhysicalParams,
    slots: &Slots,
    k: usize,
    first: usize,
    second: usize,
    lambda: f64,
) -> [f64; 3] {
    let n_modes = frame.len();
    let n_slots = frame[0].len();
    let mut g = [0.0_f64; 3];
    for md in 0..n_modes {
        let mut a = 0.0;
        let mut v = 0.0;
        for s in 0..n_slots {
            a += frame[md][s] * slots.acc[s][k];
            v += frame[md][s] * slots.vel[s][k];
        }
        let w = frame[md][first];
        g[0] += w * p.mass[md] * a;
        g[1] += w * p.viscous[md] * v;
        g[2] += w * p.coulomb[md] * coulomb_sign(v);
    }
    let belt_sign = SIGNS[first] + lambda * SIGNS[second];
    [belt_sign * g[0], belt_sign * g[1], belt_sign * g[2]]
}

/// Base per-slot torque the mode model implies (cancels in every pair
/// differential because the drive signs zero `s_i − λ·s_j`).
fn base_torque(frame: &[Vec<f64>], p: &PhysicalParams, slots: &Slots, k: usize) -> Vec<f64> {
    let n_modes = frame.len();
    let n_slots = frame[0].len();
    let mut mode_force = vec![0.0; n_modes];
    for md in 0..n_modes {
        let mut a = 0.0;
        let mut v = 0.0;
        for s in 0..n_slots {
            a += frame[md][s] * slots.acc[s][k];
            v += frame[md][s] * slots.vel[s][k];
        }
        mode_force[md] = p.mass[md] * a + p.viscous[md] * v + p.coulomb[md] * coulomb_sign(v);
    }
    (0..n_slots)
        .map(|s| (0..n_modes).map(|md| frame[md][s] * mode_force[md]).sum())
        .collect()
}

struct Injection {
    w: [f64; 6],
    even_kappa: f64,
    offset: f64,
    noise: f64,
}

/// Synthesize a 4-slot capture whose per-pair torque differential carries the
/// injected odd load-share model (`inj[pair]`), plus base mode torque that must
/// cancel in the differential.
fn synth(
    slots: &Slots,
    inj: &[(usize, usize, f64, Injection)],
    salt: usize,
) -> (Capture, Vec<Vec<f64>>) {
    let frame = frame();
    let p = params();
    let n = slots.acc[0].len();
    let n_slots = frame[0].len();
    let mut torque = vec![vec![0.0; n]; n_slots];
    for k in 0..n {
        let base = base_torque(&frame, &p, slots, k);
        for s in 0..n_slots {
            torque[s][k] = base[s];
        }
        for (first, second, lambda, ij) in inj {
            let fb = belt_forces(&frame, &p, slots, k, *first, *second, *lambda);
            let pb = SIGNS[*first] * slots.pos[*first][k];
            let odd = (ij.w[0] + ij.w[1] * pb) * fb[0]
                + (ij.w[2] + ij.w[3] * pb) * fb[1]
                + (ij.w[4] + ij.w[5] * pb) * fb[2];
            let d = odd + ij.even_kappa * fb[0].abs() + ij.offset;
            torque[*first][k] += 0.5 * SIGNS[*first] * d;
            torque[*second][k] -= 0.5 * SIGNS[*second] * d;
        }
        if inj.iter().any(|(_, _, _, ij)| ij.noise > 0.0) {
            for (s, col) in torque.iter_mut().enumerate() {
                col[k] += inj[0].3.noise * white(k, salt + s);
            }
        }
    }
    let cap = Capture {
        t: (0..n).map(|k| k as f64 * DT).collect(),
        acc: slots.acc.clone(),
        vel: slots.vel.clone(),
        vel_act: slots.vel.clone(),
        torque: torque.clone(),
        pos: slots.pos.clone(),
    };
    (cap, torque)
}

fn keep_all(n: usize) -> Vec<bool> {
    vec![true; n]
}

#[test]
fn recovers_injected_split_exactly() {
    let slots = four_slot_motion();
    let w0 = [0.02, -0.0002, 0.05, 0.0, -0.01, 0.0001];
    let w1 = [0.03, 0.0003, 0.04, 0.0, -0.015, -0.0002];
    let inj = vec![
        (
            0,
            1,
            -1.0,
            Injection {
                w: w0,
                even_kappa: 0.0,
                offset: 2.5,
                noise: 0.0,
            },
        ),
        (
            2,
            3,
            1.0,
            Injection {
                w: w1,
                even_kappa: 0.0,
                offset: -1.75,
                noise: 0.0,
            },
        ),
    ];
    let (cap, torque_filt) = synth(&slots, &inj, 0);
    let n = cap.t.len();
    let keep = keep_all(n);
    let structure = Structure::new(frame());
    let scaps = vec![SplitCapture {
        cap: &cap,
        torque_filt: &torque_filt,
        keep: &keep,
    }];
    let reports = fit_pair_splits(&structure, &params(), &SIGNS, 0.0, &scaps);
    assert_eq!(reports.len(), 2);
    for (r, truth) in reports.iter().zip([w0, w1]) {
        for i in 0..6 {
            assert!(
                (r.split.w[i] - truth[i]).abs() < 1e-6,
                "pair {}/{} w[{i}] = {} vs {}",
                r.split.first,
                r.split.second,
                r.split.w[i],
                truth[i]
            );
        }
        assert!(r.even_contribution[0] < 1e-4 && r.even_contribution[1] < 1e-4);
        assert!(!r.role_dependent);
        assert!(r.rms_after < 1e-6 * r.rms_before.max(1.0) + 1e-6);
    }
}

#[test]
fn recovers_with_noise_and_reduces_residual() {
    let slots = four_slot_motion();
    let w0 = [0.02, -0.0002, 0.05, 0.0, -0.01, 0.0001];
    let inj = vec![
        (
            0,
            1,
            -1.0,
            Injection {
                w: w0,
                even_kappa: 0.0,
                offset: 2.5,
                noise: 0.03,
            },
        ),
        (
            2,
            3,
            1.0,
            Injection {
                w: [0.03, 0.0003, 0.04, 0.0, -0.015, -0.0002],
                even_kappa: 0.0,
                offset: -1.0,
                noise: 0.03,
            },
        ),
    ];
    let (cap, torque_filt) = synth(&slots, &inj, 7);
    let n = cap.t.len();
    let keep = keep_all(n);
    let structure = Structure::new(frame());
    let scaps = vec![SplitCapture {
        cap: &cap,
        torque_filt: &torque_filt,
        keep: &keep,
    }];
    let reports = fit_pair_splits(&structure, &params(), &SIGNS, 0.0, &scaps);
    let r = &reports[0];
    assert!((r.split.w[0] - w0[0]).abs() < 0.02 * w0[0].abs());
    assert!((r.split.w[2] - w0[2]).abs() < 0.02 * w0[2].abs());
    assert!((r.split.w[4] - w0[4]).abs() < 0.02 * w0[4].abs());
    assert!(
        r.rms_after < r.rms_before,
        "{} !< {}",
        r.rms_after,
        r.rms_before
    );
    for i in [0, 2, 4] {
        assert!(r.w_tvalue[i].abs() > 3.0, "w[{i}] t = {}", r.w_tvalue[i]);
    }
}

#[test]
fn even_term_diagnostic_fires_on_injected_abs_force() {
    let slots = four_slot_motion();
    let inj = vec![
        (
            0,
            1,
            -1.0,
            Injection {
                w: [0.02, -0.0002, 0.05, 0.0, -0.01, 0.0001],
                even_kappa: 0.06,
                offset: 1.0,
                noise: 0.0,
            },
        ),
        (
            2,
            3,
            1.0,
            Injection {
                w: [0.03, 0.0003, 0.04, 0.0, -0.015, -0.0002],
                even_kappa: 0.0,
                offset: 0.5,
                noise: 0.0,
            },
        ),
    ];
    let (cap, torque_filt) = synth(&slots, &inj, 0);
    let n = cap.t.len();
    let keep = keep_all(n);
    let structure = Structure::new(frame());
    let scaps = vec![SplitCapture {
        cap: &cap,
        torque_filt: &torque_filt,
        keep: &keep,
    }];
    let reports = fit_pair_splits(&structure, &params(), &SIGNS, 0.0, &scaps);
    assert!(
        reports[0].role_dependent,
        "pair 0 should flag the injected |F_I| term (contrib {:.3} vs {:.3})",
        reports[0].even_contribution[0], reports[0].max_odd_contribution
    );
    assert!((reports[0].even_coeff[0] - 0.06).abs() < 1e-6);
    assert!(!reports[1].role_dependent, "pair 1 carries no even term");
}

#[test]
fn pools_multiple_captures_with_per_capture_offset() {
    let slots = four_slot_motion();
    let w0 = [0.02, -0.0002, 0.05, 0.0, -0.01, 0.0001];
    let w1 = [0.03, 0.0003, 0.04, 0.0, -0.015, -0.0002];
    let make = |offset0: f64, offset1: f64, salt: usize| {
        let inj = vec![
            (
                0,
                1,
                -1.0,
                Injection {
                    w: w0,
                    even_kappa: 0.0,
                    offset: offset0,
                    noise: 0.0,
                },
            ),
            (
                2,
                3,
                1.0,
                Injection {
                    w: w1,
                    even_kappa: 0.0,
                    offset: offset1,
                    noise: 0.0,
                },
            ),
        ];
        synth(&slots, &inj, salt)
    };
    let (cap_a, tq_a) = make(5.0, -3.0, 1);
    let (cap_b, tq_b) = make(-4.0, 6.0, 2);
    let n = cap_a.t.len();
    let keep = keep_all(n);
    let structure = Structure::new(frame());
    let scaps = vec![
        SplitCapture {
            cap: &cap_a,
            torque_filt: &tq_a,
            keep: &keep,
        },
        SplitCapture {
            cap: &cap_b,
            torque_filt: &tq_b,
            keep: &keep,
        },
    ];
    let reports = fit_pair_splits(&structure, &params(), &SIGNS, 0.0, &scaps);
    let r = &reports[0];
    for i in 0..6 {
        assert!(
            (r.split.w[i] - w0[i]).abs() < 1e-6,
            "pooled w[{i}] = {} vs {}",
            r.split.w[i],
            w0[i]
        );
    }
    assert_eq!(r.intercepts.len(), 2);
    assert!((r.intercepts[0] - 5.0).abs() < 1e-6, "{:?}", r.intercepts);
    assert!(
        (r.intercepts[1] - (-4.0)).abs() < 1e-6,
        "{:?}",
        r.intercepts
    );
}
