use servo_ident::ferr_out::render_ferr_json;
use servo_ident::fit::{fit_ferr, FerrFitResult, FitInput, FitOptions};
use servo_ident::model::{coulomb_sign, PhysicalParams, Structure};

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
        extra: Vec::new(),
    }
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
        ferr_rms: vec![0.012, 0.009],
        condition: 12.3,
        samples: 4321,
    };
    let json = render_ferr_json(&structure, &["x", "y"], &r);
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(v["version"], 1);
    assert_eq!(v["modes"], serde_json::json!(["x", "y"]));
    assert_eq!(v["coef"]["mass"], serde_json::json!([1.23e-8, -4.5e-9]));
    assert_eq!(v["coef"]["viscous"], serde_json::json!([2.0e-4, -1.0e-4]));
    assert_eq!(v["coef"]["coulomb"], serde_json::json!([0.01, -0.02]));
    assert_eq!(v["stderr"]["mass"], serde_json::json!([4.5e-10, 3.0e-10]));
    assert_eq!(v["stderr"]["viscous"], serde_json::json!([1.0e-5, 8.0e-6]));
    assert_eq!(v["stderr"]["coulomb"], serde_json::json!([2.0e-3, 1.5e-3]));
    assert_eq!(v["ferr_rms"], serde_json::json!([0.012, 0.009]));
    assert_eq!(v["samples"], 4321);
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
        ferr_rms: vec![0.0],
        condition: 1.0,
        samples: 1,
    };
    let _ = render_ferr_json(&structure, &["x", "y"], &r);
}
