use servo_ident::ferr_out::render_ferr_json;
use servo_ident::fit::{fit_ferr, FerrFitResult, FitInput, FitOptions};
use servo_ident::model::{coulomb_sign, PhysicalParams, Structure};
use servo_ident::prep::TransientRms;

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

fn small_noise(x: f64, k: usize, amplitude: f64) -> f64 {
    let h = k.wrapping_mul(2654435761) as u32;
    x + (f64::from(h % 1000) / 1000.0 - 0.5) * amplitude
}

/// Excite each mode in its own time window (so the modes are independently
/// identifiable), then synthesize the following error the mode truth
/// implies: `ferr = alpha*acc + gamma*sign(vel)`, plus small noise.
fn synth_ferr(frame: &[Vec<f64>], alpha: &[f64], gamma: &[f64]) -> FitInput {
    let n_modes = frame.len();
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
    let ferr_mode: Vec<Vec<f64>> = (0..n_modes)
        .map(|m| {
            (0..total)
                .map(|k| {
                    small_noise(
                        alpha[m] * acc_mode[m][k] + gamma[m] * cs_mode[m][k],
                        k + 13 * m,
                        0.02,
                    )
                })
                .collect()
        })
        .collect();
    FitInput {
        structure: Structure::new(frame.to_vec()),
        acc_mode,
        vel_mode,
        cs_mode,
        snap_mode: vec![],
        torque: vec![],
        ferr_mode,
        jerk_mode: vec![],
        extra: Vec::new(),
    }
}

fn jerk_from_acc(acc_mode: &[Vec<f64>], dt: f64) -> Vec<Vec<f64>> {
    acc_mode
        .iter()
        .map(|a| {
            let n = a.len();
            (0..n)
                .map(|k| {
                    if k == 0 || k + 1 == n {
                        0.0
                    } else {
                        (a[k + 1] - a[k - 1]) / (2.0 * dt)
                    }
                })
                .collect()
        })
        .collect()
}

/// Delay every mode's ferr response by `shift` samples relative to the
/// commanded kinematics: `ferr = alpha * acc(t - shift*dt)`.
fn synth_shifted_ferr(frame: &[Vec<f64>], alpha: &[f64], shift: usize) -> FitInput {
    let mut input = synth_ferr(frame, alpha, &vec![0.0; frame.len()]);
    for (m, col) in input.ferr_mode.iter_mut().enumerate() {
        let n = col.len();
        for k in (shift..n).rev() {
            let base = alpha[m] * input.acc_mode[m][k - shift];
            col[k] = small_noise(base, k + 13 * m, 0.02);
        }
        for v in col.iter_mut().take(shift) {
            *v = 0.0;
        }
    }
    input
}

#[test]
fn recovers_signed_ferr_coefficients_per_mode() {
    let frame = vec![vec![0.5, 0.5], vec![0.5, -0.5]];
    let alpha = [2.0e-5, -1.5e-5];
    let gamma = [0.08, -0.05];
    let input = synth_ferr(&frame, &alpha, &gamma);
    let r = fit_ferr(&input, &FitOptions::default()).unwrap();
    for k in 0..2 {
        assert!(
            (r.params.mass[k] - alpha[k]).abs() < 0.15 * alpha[k].abs(),
            "mass[{k}] = {} vs truth {}",
            r.params.mass[k],
            alpha[k]
        );
        assert_eq!(
            r.params.mass[k].signum(),
            alpha[k].signum(),
            "mass[{k}] sign flipped: {} vs truth {}",
            r.params.mass[k],
            alpha[k]
        );
        assert!(
            (r.params.coulomb[k] - gamma[k]).abs() < 0.15 * gamma[k].abs(),
            "coulomb[{k}] = {} vs truth {}",
            r.params.coulomb[k],
            gamma[k]
        );
        assert_eq!(
            r.params.coulomb[k].signum(),
            gamma[k].signum(),
            "coulomb[{k}] sign flipped: {} vs truth {}",
            r.params.coulomb[k],
            gamma[k]
        );
        let viscous_se = r.param_stderr[3 * k + 1];
        assert!(
            r.params.viscous[k].abs() < 4.0 * viscous_se,
            "viscous[{k}] = {} not statistically zero (se {viscous_se})",
            r.params.viscous[k]
        );
    }
}

#[test]
fn zero_ferr_capture_yields_coefficients_statistically_indistinguishable_from_zero() {
    let frame = vec![vec![0.5, 0.5], vec![0.5, -0.5]];
    let input = synth_ferr(&frame, &[0.0, 0.0], &[0.0, 0.0]);
    let r = fit_ferr(&input, &FitOptions::default()).unwrap();
    for k in 0..2 {
        for (name, coef, se) in [
            ("mass", r.params.mass[k], r.param_stderr[3 * k]),
            ("viscous", r.params.viscous[k], r.param_stderr[3 * k + 1]),
            ("coulomb", r.params.coulomb[k], r.param_stderr[3 * k + 2]),
        ] {
            assert!(
                coef.abs() < 4.0 * se,
                "mode {k} {name} = {coef} not statistically zero (se {se})"
            );
        }
    }
}

#[test]
fn render_ferr_json_matches_the_documented_contract_shape() {
    let structure = Structure::new(vec![vec![1.0, 0.0], vec![0.0, 1.0]]);
    let r = FerrFitResult {
        params: PhysicalParams {
            mass: vec![1.23e-8, -4.5e-9],
            viscous: vec![2.0e-4, -1.0e-4],
            coulomb: vec![0.01, -0.02],
        },
        param_stderr: vec![4.5e-10, 1.0e-5, 2.0e-3, 3.0e-10, 8.0e-6, 1.5e-3],
        jerk: vec![-2.0e-8, 1.0e-8],
        jerk_stderr: vec![1.0e-9, 1.5e-9],
        ferr_rms: vec![0.012, 0.009],
        condition: 12.3,
        samples: 4321,
    };
    let mass = vec![
        TransientRms {
            rms: Some(0.0021),
            sigma: Some(0.0003),
            windows: 5,
        },
        TransientRms {
            rms: Some(0.0018),
            sigma: None,
            windows: 1,
        },
    ];
    let viscous = vec![
        TransientRms {
            rms: Some(0.0011),
            sigma: Some(0.0002),
            windows: 4,
        },
        TransientRms {
            rms: Some(0.0009),
            sigma: Some(0.0001),
            windows: 3,
        },
    ];
    let coulomb = vec![
        TransientRms {
            rms: None,
            sigma: None,
            windows: 0,
        },
        TransientRms {
            rms: Some(0.0007),
            sigma: Some(0.00005),
            windows: 2,
        },
    ];
    let json = render_ferr_json(
        &structure,
        &["x", "y"],
        &r,
        &[0.0034, 0.0044],
        &[0.0012, -0.0007],
        42,
        &mass,
        &viscous,
        &coulomb,
    );
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(v["version"], 2);
    assert_eq!(v["modes"], serde_json::json!(["x", "y"]));
    assert_eq!(v["coef"]["mass"], serde_json::json!([1.23e-8, -4.5e-9]));
    assert_eq!(v["coef"]["viscous"], serde_json::json!([2.0e-4, -1.0e-4]));
    assert_eq!(v["coef"]["coulomb"], serde_json::json!([0.01, -0.02]));
    assert_eq!(v["stderr"]["mass"], serde_json::json!([4.5e-10, 3.0e-10]));
    assert_eq!(v["stderr"]["viscous"], serde_json::json!([1.0e-5, 8.0e-6]));
    assert_eq!(v["stderr"]["coulomb"], serde_json::json!([2.0e-3, 1.5e-3]));
    assert_eq!(v["jerk"], serde_json::json!([-2.0e-8, 1.0e-8]));
    assert_eq!(v["jerk_stderr"], serde_json::json!([1.0e-9, 1.5e-9]));
    assert_eq!(v["ferr_rms"], serde_json::json!([0.012, 0.009]));
    assert_eq!(v["ferr_rms_raw"], serde_json::json!([0.0034, 0.0044]));
    assert_eq!(v["onset_bias"], serde_json::json!([0.0012, -0.0007]));
    assert_eq!(v["onset_windows"], 42);
    assert_eq!(v["samples"], 4321);
    assert_eq!(
        v["ferr_rms_ff"]["mass"]["rms"],
        serde_json::json!([0.0021, 0.0018])
    );
    assert_eq!(
        v["ferr_rms_ff"]["mass"]["sigma"],
        serde_json::json!([0.0003, null])
    );
    assert_eq!(
        v["ferr_rms_ff"]["mass"]["windows"],
        serde_json::json!([5, 1])
    );
    assert_eq!(
        v["ferr_rms_ff"]["viscous"]["rms"],
        serde_json::json!([0.0011, 0.0009])
    );
    assert_eq!(
        v["ferr_rms_ff"]["viscous"]["sigma"],
        serde_json::json!([0.0002, 0.0001])
    );
    assert_eq!(
        v["ferr_rms_ff"]["coulomb"]["rms"],
        serde_json::json!([null, 0.0007])
    );
    assert_eq!(
        v["ferr_rms_ff"]["coulomb"]["sigma"],
        serde_json::json!([null, 0.00005])
    );
    assert_eq!(
        v["ferr_rms_ff"]["coulomb"]["windows"],
        serde_json::json!([0, 2])
    );
}

#[test]
#[should_panic(expected = "one mode name per structure row")]
fn render_ferr_json_fails_loudly_on_mode_count_mismatch() {
    let structure = Structure::new(vec![vec![1.0]]);
    let r = FerrFitResult {
        params: PhysicalParams {
            mass: vec![0.0],
            viscous: vec![0.0],
            coulomb: vec![0.0],
        },
        param_stderr: vec![0.0, 0.0, 0.0],
        jerk: vec![],
        jerk_stderr: vec![],
        ferr_rms: vec![0.0],
        condition: 1.0,
        samples: 1,
    };
    let empty: Vec<TransientRms> = vec![];
    let _ = render_ferr_json(
        &structure,
        &["x", "y"],
        &r,
        &[0.0, 0.0],
        &[0.0, 0.0],
        0,
        &empty,
        &empty,
        &empty,
    );
}

/// A command->telemetry timing skew turns `alpha*acc(t-d)` into
/// `alpha*acc(t) - alpha*d*jerk(t)`: without a jerk column the second term
/// correlates with accel over corner-rich excitation and lands in the mass
/// coefficient as bias, which is exactly the failure mode that walked the
/// bench tuner to 2x the rms-optimal mass. The jerk nuisance column must
/// absorb it.
#[test]
fn jerk_column_absorbs_command_to_ferr_timing_skew() {
    let frame = vec![vec![0.5, 0.5], vec![0.5, -0.5]];
    let alpha = [2.0e-5, -1.5e-5];
    let dt = 0.001;
    let shift = 2;
    let biased = synth_shifted_ferr(&frame, &alpha, shift);
    let rb = fit_ferr(&biased, &FitOptions::default()).unwrap();
    let mut debiased = biased.clone();
    debiased.jerk_mode = jerk_from_acc(&biased.acc_mode, dt);
    let rd = fit_ferr(&debiased, &FitOptions::default()).unwrap();
    let _ = rb;
    for k in 0..2 {
        let debiased_err = (rd.params.mass[k] - alpha[k]).abs() / alpha[k].abs();
        assert!(
            debiased_err < 0.1,
            "mode {k}: de-biased mass {} vs truth {}",
            rd.params.mass[k],
            alpha[k]
        );
        let expected_jerk = -alpha[k] * shift as f64 * dt;
        assert_eq!(
            rd.jerk[k].signum(),
            expected_jerk.signum(),
            "mode {k}: jerk coef {} sign vs expected -alpha*delta = {expected_jerk}",
            rd.jerk[k]
        );
    }
}

#[test]
fn unshifted_ferr_keeps_jerk_statistically_zero_and_mass_unbiased() {
    let frame = vec![vec![0.5, 0.5], vec![0.5, -0.5]];
    let alpha = [2.0e-5, -1.5e-5];
    let gamma = [0.08, -0.05];
    let mut input = synth_ferr(&frame, &alpha, &gamma);
    input.jerk_mode = jerk_from_acc(&input.acc_mode, 0.001);
    let r = fit_ferr(&input, &FitOptions::default()).unwrap();
    for k in 0..2 {
        assert!(
            (r.params.mass[k] - alpha[k]).abs() < 0.15 * alpha[k].abs(),
            "mass[{k}] = {} vs truth {}",
            r.params.mass[k],
            alpha[k]
        );
        assert!(
            r.jerk[k].abs() < 4.0 * r.jerk_stderr[k],
            "mode {k} jerk = {} not statistically zero (se {})",
            r.jerk[k],
            r.jerk_stderr[k]
        );
    }
}

#[test]
fn onset_bias_reads_the_first_excursion_sign_per_mode() {
    use servo_ident::prep::onset_bias;
    let dt = 0.001;
    let n = 400;
    let mut acc = vec![0.0; n];
    // two accel steps: +8000 at 50, -8000 at 200
    for k in 50..150 {
        acc[k] = 8000.0;
    }
    for k in 200..300 {
        acc[k] = -8000.0;
    }
    // mode 0 lags at torque application (under-fed): ferr follows sign(a)
    // for the first 5 ms of every step, then the "drive compensation"
    // reverses it - the whole-capture mean is ~0, the onset is not.
    let mut lag = vec![0.0; n];
    let mut over = vec![0.0; n];
    for (start, s) in [(50usize, 1.0f64), (200, -1.0)] {
        for k in start..start + 5 {
            lag[k] = 0.004 * s;
            over[k] = -0.004 * s;
        }
        for k in start + 5..start + 100 {
            lag[k] = -0.0002 * s;
            over[k] = 0.0002 * s;
        }
    }
    let (bias, windows) = onset_bias(&[acc.clone(), acc], &[lag, over], dt, 0.008);
    assert_eq!(windows, 4, "two steps per mode");
    assert!(
        bias[0] > 0.001,
        "lagging mode must read under-fed: {bias:?}"
    );
    assert!(
        bias[1] < -0.001,
        "overshooting mode must read over-fed: {bias:?}"
    );
}

#[test]
fn onset_bias_is_zero_without_excitation() {
    use servo_ident::prep::onset_bias;
    let (bias, windows) = onset_bias(&[vec![0.0; 100]], &[vec![0.5; 100]], 0.001, 0.008);
    assert_eq!(windows, 0);
    assert_eq!(bias, vec![0.0]);
}
