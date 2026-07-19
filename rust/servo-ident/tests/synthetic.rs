use servo_ident::fit::{fit, FitError, FitInput, FitOptions};
use servo_ident::model::{coulomb_sign, Structure};

fn triangle(a: f64, t1: f64, dt: f64, reps: usize) -> (Vec<f64>, Vec<f64>) {
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut v = 0.0;
    for _ in 0..reps {
        for phase in [a, -a, -a, a] {
            let steps = (t1 / dt) as usize;
            for _ in 0..steps {
                acc.push(phase);
                v += phase * dt;
                vel.push(v);
            }
        }
    }
    (acc, vel)
}

fn noisy(x: f64, k: usize) -> f64 {
    let h = k.wrapping_mul(2654435761) as u32;
    x + (f64::from(h % 1000) / 1000.0 - 0.5)
}

/// Excite each mode in its own time window (so the modes are independently
/// identifiable), then synthesize the per-slot torque the frame + mode truth
/// implies.
fn synth(frame: &[Vec<f64>], mass: &[f64], viscous: &[f64], coulomb: &[f64]) -> FitInput {
    let n_modes = frame.len();
    let n_slots = frame[0].len();
    #[allow(clippy::cast_precision_loss)]
    let windows: Vec<(Vec<f64>, Vec<f64>)> = (0..n_modes)
        .map(|k| triangle(8000.0 + 500.0 * k as f64, 0.05, 0.001, 4))
        .collect();
    let lens: Vec<usize> = windows.iter().map(|(a, _)| a.len()).collect();
    let total: usize = lens.iter().sum();
    let mut acc_mode = vec![Vec::with_capacity(total); n_modes];
    let mut vel_mode = vec![Vec::with_capacity(total); n_modes];
    for k in 0..n_modes {
        for (j, (a, v)) in windows.iter().enumerate() {
            if j == k {
                acc_mode[k].extend_from_slice(a);
                vel_mode[k].extend_from_slice(v);
            } else {
                acc_mode[k].extend(std::iter::repeat(0.0).take(lens[j]));
                vel_mode[k].extend(std::iter::repeat(0.0).take(lens[j]));
            }
        }
    }
    let cs_mode: Vec<Vec<f64>> = vel_mode
        .iter()
        .map(|col| col.iter().map(|&v| coulomb_sign(v)).collect())
        .collect();
    let mut torque = vec![Vec::with_capacity(total); n_slots];
    for (s, col) in torque.iter_mut().enumerate() {
        for k in 0..total {
            let mut tau = 0.0;
            for m in 0..n_modes {
                tau += frame[m][s]
                    * (mass[m] * acc_mode[m][k]
                        + viscous[m] * vel_mode[m][k]
                        + coulomb[m] * coulomb_sign(vel_mode[m][k]));
            }
            col.push(noisy(tau, k + 13 * s).round());
        }
    }
    FitInput {
        structure: Structure::new(frame.to_vec()),
        acc_mode,
        vel_mode,
        cs_mode,
        snap_mode: vec![],
        torque,
        ferr_mode: vec![],
        extra: Vec::new(),
    }
}

fn assert_recovers(frame: &[Vec<f64>], mass: &[f64], viscous: &[f64], coulomb: &[f64]) {
    let input = synth(frame, mass, viscous, coulomb);
    let r = fit(&input, &FitOptions::default()).unwrap();
    for k in 0..frame.len() {
        assert!(
            (r.params.mass[k] - mass[k]).abs() < 0.1 * mass[k],
            "mass[{k}] = {} vs {}",
            r.params.mass[k],
            mass[k]
        );
        assert!(
            (r.params.viscous[k] - viscous[k]).abs() < 0.15 * viscous[k],
            "viscous[{k}] = {} vs {}",
            r.params.viscous[k],
            viscous[k]
        );
        assert!(
            (r.params.coulomb[k] - coulomb[k]).abs() < 0.1 * coulomb[k],
            "coulomb[{k}] = {} vs {}",
            r.params.coulomb[k],
            coulomb[k]
        );
    }
}

#[test]
fn recovers_corexy_mode_truth() {
    let frame = vec![vec![0.5, 0.5], vec![0.5, -0.5]];
    assert_recovers(&frame, &[0.030, 0.020], &[0.09, 0.11], &[160.0, 175.0]);
}

#[test]
fn recovers_awd_mode_truth() {
    let frame = vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ];
    assert_recovers(&frame, &[0.0123, 0.0119], &[0.09, 0.11], &[160.0, 175.0]);
}

#[test]
fn refuses_unexcited_mode() {
    let n = 2000;
    let input = FitInput {
        structure: Structure::new(vec![vec![1.0]]),
        acc_mode: vec![vec![0.0; n]],
        vel_mode: vec![vec![100.0; n]],
        cs_mode: vec![vec![1.0; n]],
        snap_mode: vec![],
        torque: vec![vec![1.0; n]],
        ferr_mode: vec![],
        extra: Vec::new(),
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::UnexcitedMode { mode: 0 })
    ));
}

#[test]
fn refuses_collinear_excitation() {
    let n = 2000;
    let input = FitInput {
        structure: Structure::new(vec![vec![1.0]]),
        acc_mode: vec![vec![500.0; n]],
        vel_mode: vec![vec![100.0; n]],
        cs_mode: vec![vec![1.0; n]],
        snap_mode: vec![],
        torque: vec![vec![1.0; n]],
        ferr_mode: vec![],
        extra: Vec::new(),
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::InsufficientExcitation { .. })
    ));
}

#[test]
fn refuses_saturated_torque() {
    let (acc, vel) = triangle(2000.0, 0.08, 0.001, 4);
    let n = acc.len();
    let cs: Vec<f64> = vel.iter().map(|&v| coulomb_sign(v)).collect();
    let mut torque = vec![100.0; n];
    for t in torque.iter_mut().take(n / 10) {
        *t = 3995.0;
    }
    let input = FitInput {
        structure: Structure::new(vec![vec![1.0]]),
        acc_mode: vec![acc],
        vel_mode: vec![vel],
        cs_mode: vec![cs],
        snap_mode: vec![],
        torque: vec![torque],
        ferr_mode: vec![],
        extra: Vec::new(),
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::SaturatedTorque { .. })
    ));
}

/// Two-mass compliance shows up as torque correlated with snap (d²a/dt²).
/// With the snap column enabled the fit recovers the rigid mass and the
/// compliance coefficient separately; without it the ω² term aliases into
/// mass and drags it with the excitation spectrum — the drift the column
/// exists to prevent.
#[test]
fn snap_column_separates_compliance_from_mass() {
    let frame = vec![vec![1.0]];
    let (mass_true, c_true) = (0.02, -2.0e-7);
    let dt = 0.001;
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut snap = Vec::new();
    for k in 0..8000 {
        let t = k as f64 * dt;
        let (f, amp) = if k < 4000 {
            (8.0, 20000.0)
        } else {
            (35.0, 20000.0)
        };
        let w = 2.0 * std::f64::consts::PI * f;
        acc.push(amp * libm::sin(w * t));
        vel.push(-amp / w * libm::cos(w * t) + amp / w);
        snap.push(-amp * w * w * libm::sin(w * t));
    }
    let cs: Vec<f64> = vel.iter().map(|&v| coulomb_sign(v)).collect();
    let torque: Vec<f64> = (0..acc.len())
        .map(|k| {
            noisy(
                mass_true * acc[k] + 0.004 * vel[k] + 1.0 * cs[k] + c_true * snap[k],
                k,
            )
        })
        .collect();
    let with_snap = FitInput {
        structure: Structure::new(frame.clone()),
        acc_mode: vec![acc.clone()],
        vel_mode: vec![vel.clone()],
        cs_mode: vec![cs.clone()],
        snap_mode: vec![snap.clone()],
        torque: vec![torque.clone()],
        ferr_mode: vec![],
        extra: Vec::new(),
    };
    let r = fit(&with_snap, &FitOptions::default()).unwrap();
    assert!(
        (r.params.mass[0] - mass_true).abs() < 0.02 * mass_true,
        "mass with snap column = {} vs {mass_true}",
        r.params.mass[0]
    );
    assert!(
        (r.snap_params[0] - c_true).abs() < 0.05 * c_true.abs(),
        "snap coefficient = {} vs {c_true}",
        r.snap_params[0]
    );

    let without = FitInput {
        snap_mode: vec![],
        ..with_snap
    };
    let r0 = fit(&without, &FitOptions::default()).unwrap();
    assert!(
        r0.params.mass[0] > 1.05 * mass_true,
        "without the snap column the ω² rise must alias into mass \
         (got {} vs true {mass_true})",
        r0.params.mass[0]
    );
    assert!(r0.snap_params.is_empty());
}
