//! Usage: servo-ident --capture run.csv --structure scalar|corexy \
//!   --axes x[,b] --out profile.toml \
//!   [--rated-torque-nm T --rotor-inertia-kgm2 J --rotation-distance-mm D]
#![allow(clippy::exit)]

use servo_ident::capture::{
    parse_capture_csv, steady_accel_keep, tracking_keep, PlateauOptions, TrackingOptions,
};
use servo_ident::fit::residual_by_motor;
use servo_ident::fit::{fit, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::prep::{band_limited_rms, prep, PrepOptions};
use servo_ident::profile_out::{c0006_recommendation, render_profile};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn opt_f64(args: &[String], key: &str) -> Option<f64> {
    arg(args, key).map(|v| {
        v.parse().unwrap_or_else(|_| {
            eprintln!("servo-ident: bad {key} {v:?}");
            std::process::exit(1);
        })
    })
}

fn req(args: &[String], key: &str) -> String {
    arg(args, key).unwrap_or_else(|| {
        eprintln!("servo-ident: missing required {key}");
        std::process::exit(1);
    })
}

const KNOWN_KEYS: [&str; 11] = [
    "--capture",
    "--structure",
    "--axes",
    "--out",
    "--rated-torque-nm",
    "--rotor-inertia-kgm2",
    "--rotation-distance-mm",
    "--cutoff-hz",
    "--blank-ms",
    "--max-delay-ms",
    "--ripple-period-mm",
];

fn reject_unknown_flags(args: &[String]) {
    let mut i = 1;
    while i < args.len() {
        let a = &args[i];
        if !KNOWN_KEYS.contains(&a.as_str()) {
            eprintln!("servo-ident: unknown argument {a:?}");
            std::process::exit(1);
        }
        i += 2;
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    reject_unknown_flags(&args);
    let structure = match req(&args, "--structure").as_str() {
        "scalar" => Structure::CartesianScalar,
        "corexy" => Structure::CoreXY,
        other => {
            eprintln!("servo-ident: unknown structure {other}");
            std::process::exit(1);
        }
    };
    let axes_arg = req(&args, "--axes");
    let axes: Vec<&str> = axes_arg.split(',').map(str::trim).collect();
    if axes.len() != structure.axis_count() {
        eprintln!(
            "servo-ident: {} axes given, structure needs {}",
            axes.len(),
            structure.axis_count()
        );
        std::process::exit(1);
    }

    let capture_path = req(&args, "--capture");
    let text = std::fs::read_to_string(&capture_path).unwrap_or_else(|e| {
        eprintln!("servo-ident: read {capture_path}: {e}");
        std::process::exit(1);
    });
    let cap = parse_capture_csv(&text, &axes).unwrap_or_else(|e| {
        eprintln!("servo-ident: capture invalid: {e:?}");
        std::process::exit(1);
    });
    let total = cap.t.len();
    let mut prep_opts = PrepOptions::default();
    if let Some(v) = opt_f64(&args, "--cutoff-hz") {
        prep_opts.cutoff_hz = v;
    }
    if let Some(v) = opt_f64(&args, "--blank-ms") {
        prep_opts.blank_reversal_s = v / 1000.0;
    }
    if let Some(v) = opt_f64(&args, "--max-delay-ms") {
        prep_opts.max_delay_s = v / 1000.0;
    }
    prep_opts.ripple_period_mm =
        opt_f64(&args, "--ripple-period-mm").or_else(|| opt_f64(&args, "--rotation-distance-mm"));
    let pp = prep(&cap, &prep_opts);
    eprintln!(
        "prep: {} segments, accel->torque delay {:.2} ms removed",
        pp.segments,
        pp.delay_s * 1000.0
    );
    let track = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let plateau = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let keep: Vec<usize> = (0..total)
        .filter(|&k| pp.valid[k] && track[k] && plateau[k])
        .collect();
    let tracked = (0..total).filter(|&k| pp.valid[k] && track[k]).count();
    eprintln!(
        "masks: prep+tracking kept {tracked}/{total}, +steady-accel plateaus kept {}/{total}",
        keep.len()
    );
    if tracked == 0 {
        eprintln!(
            "servo-ident: the drive never tracked the commanded trajectory — \
             stiction breakaway or an untuned/lagging loop; enable velocity \
             feedforward and check the mechanics, then re-capture"
        );
        std::process::exit(2);
    }
    if keep.is_empty() {
        eprintln!(
            "servo-ident: no steady-accel plateaus in capture — strokes too short \
             or jerk-limited accel never holds; lengthen strokes or lower accel"
        );
        std::process::exit(2);
    }
    let pick = |cols: &[Vec<f64>]| -> Vec<Vec<f64>> {
        cols.iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect()
    };
    let input = FitInput {
        structure,
        acc: pick(&pp.acc),
        vel: pick(&pp.vel),
        cf: pick(&pp.cf),
        cr: pick(&pp.cr),
        torque: pick(&pp.torque),
        extra: pp.extra.iter().map(|cols| pick(cols)).collect(),
    };
    let r = fit(&input, &FitOptions::default()).unwrap_or_else(|e| {
        eprintln!("servo-ident: refusing to emit a profile: {e:?}");
        std::process::exit(2);
    });

    eprintln!(
        "fit: {} samples/motor, rms residual {:.2} (0.1% rated), condition {:.1e}",
        r.samples, r.rms_residual, r.condition
    );
    if prep_opts.cutoff_hz > 0.0 {
        let full = FitInput {
            structure,
            acc: pp.acc.clone(),
            vel: pp.vel.clone(),
            cf: pp.cf.clone(),
            cr: pp.cr.clone(),
            torque: pp.torque.clone(),
            extra: pp.extra.clone(),
        };
        let res = residual_by_motor(&full, &r.params, &r.extra_params);
        let keep_mask: Vec<bool> = (0..total)
            .map(|k| pp.valid[k] && track[k] && plateau[k])
            .collect();
        let inband = band_limited_rms(&res, &pp.t, &keep_mask, prep_opts.cutoff_hz);
        eprintln!(
            "in-band (<= {:.0} Hz) rms residual: {:.2} (0.1% rated) — model error \
             in the band a feedforward model controls; the raw residual above \
             also counts ripple, loop transients and quantization",
            prep_opts.cutoff_hz, inband
        );
    }
    let names: Vec<String> = match structure {
        Structure::CartesianScalar => ["mass", "viscous", "coulomb_fwd", "coulomb_rev"]
            .iter()
            .map(|s| (*s).to_string())
            .collect(),
        Structure::CoreXY => {
            let mut v = vec!["mass_diag".to_string(), "mass_off".to_string()];
            for a in &axes {
                v.push(format!("viscous_{a}"));
                v.push(format!("coulomb_fwd_{a}"));
                v.push(format!("coulomb_rev_{a}"));
            }
            v
        }
    };
    for (name, se) in names.iter().zip(&r.param_stderr) {
        eprintln!("  stderr {name}: {se:.4}");
    }
    for (axis, coeffs) in axes.iter().zip(&r.extra_params) {
        if coeffs.len() == 2 {
            let amp = (coeffs[0] * coeffs[0] + coeffs[1] * coeffs[1]).sqrt();
            let phase = libm::atan2(coeffs[1], coeffs[0]).to_degrees();
            eprintln!(
                "  pulley ripple {axis}: {amp:.1} (0.1% rated) at {phase:.0} deg \
                 — eccentricity torque absorbed by the nuisance columns"
            );
        }
    }
    let min_diag = (0..r.params.mass.len())
        .map(|i| r.params.mass[i][i])
        .fold(f64::INFINITY, f64::min);
    let physical = min_diag > 0.0;

    if let (Some(t), Some(j), Some(d)) = (
        opt_f64(&args, "--rated-torque-nm"),
        opt_f64(&args, "--rotor-inertia-kgm2"),
        opt_f64(&args, "--rotation-distance-mm"),
    ) {
        let n = r.params.mass.len();
        if n == 2 {
            // The sign of the fitted off-diagonal follows the capture frame
            // (invert_direction flips it per drive), so the eigen-directions
            // are labeled by magnitude, not by which formula produced them.
            let sum = r.params.mass[0][0] + r.params.mass[0][1];
            let diff = r.params.mass[0][0] - r.params.mass[0][1];
            let m_light = sum.min(diff);
            let m_heavy = sum.max(diff);
            eprintln!(
                "recommended C00.06 (light direction): {:.0}%",
                c0006_recommendation(m_light, t, d, j)
            );
            eprintln!(
                "heavy-direction equivalent (reference only): {:.0}%",
                c0006_recommendation(m_heavy, t, d, j)
            );
        } else {
            eprintln!(
                "recommended C00.06 (light direction): {:.0}%",
                c0006_recommendation(r.params.mass[0][0], t, d, j)
            );
        }
    }

    if !physical {
        eprintln!(
            "servo-ident: fitted diagonal mass {min_diag:.5} <= 0 is physically \
             impossible — C00.06 is J_load/J_rotor and load inertia cannot be \
             negative (drive accepts 0..12000%). The captured torque runs opposite \
             to the commanded acceleration: a drive torque-polarity / \
             invert_direction sign mismatch, not a real inertia. No profile written \
             — fix the capture sign convention and re-run."
        );
        return;
    }

    let rms = vec![r.rms_residual; axes.len()];
    let profile = render_profile(&r.params, &axes, &rms);
    let out = req(&args, "--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| {
        eprintln!("servo-ident: write {out}: {e}");
        std::process::exit(1);
    });
    eprintln!("profile written to {out}");
}
