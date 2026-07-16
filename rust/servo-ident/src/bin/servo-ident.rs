//! Usage: servo-ident --capture run.csv \
//!   --frame "r0c0,r0c1,...;r1c0,r1c1,..." --modes x[,y] --axes a[,b,...] \
//!   --out profile.toml \
//!   [--rated-torque-nm T --rotor-inertia-kgm2 J --rotation-distance-mm D]
#![allow(clippy::exit)]

use servo_ident::capture::{parse_capture_csv, Capture};
use servo_ident::fit::residual_by_motor;
use servo_ident::fit::{fit, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::pipeline::{fit_input, prepare};
use servo_ident::prep::{band_limited_rms, PrepOptions};
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

const KNOWN_KEYS: [&str; 12] = [
    "--capture",
    "--frame",
    "--modes",
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

fn parse_frame(spec: &str) -> Vec<Vec<f64>> {
    spec.split(';')
        .map(|row| {
            row.split(',')
                .map(|e| {
                    e.trim().parse::<f64>().unwrap_or_else(|_| {
                        eprintln!("servo-ident: bad frame entry {e:?}");
                        std::process::exit(1);
                    })
                })
                .collect()
        })
        .collect()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    reject_unknown_flags(&args);
    let frame = parse_frame(&req(&args, "--frame"));
    let modes_arg = req(&args, "--modes");
    let modes: Vec<&str> = modes_arg.split(',').map(str::trim).collect();
    if modes.len() != frame.len() {
        eprintln!(
            "servo-ident: {} modes given, frame has {} rows",
            modes.len(),
            frame.len()
        );
        std::process::exit(1);
    }
    let structure = Structure::new(frame.clone());
    let axes_arg = req(&args, "--axes");
    let axes: Vec<&str> = axes_arg.split(',').map(str::trim).collect();
    if axes.len() != structure.axis_count() {
        eprintln!(
            "servo-ident: {} axes given, frame has {} columns",
            axes.len(),
            structure.axis_count()
        );
        std::process::exit(1);
    }

    if args.iter().filter(|a| *a == "--capture").count() > 1 {
        eprintln!(
            "servo-ident: --capture given more than once — the fit consumes exactly one capture"
        );
        std::process::exit(1);
    }
    let capture_path = req(&args, "--capture");
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

    let text = std::fs::read_to_string(&capture_path).unwrap_or_else(|e| {
        eprintln!("servo-ident: read {capture_path}: {e}");
        std::process::exit(1);
    });
    let cap: Capture = parse_capture_csv(&text, &axes).unwrap_or_else(|e| {
        eprintln!("servo-ident: capture {capture_path} invalid: {e:?}");
        std::process::exit(1);
    });
    let (prepared, stats) = prepare(&cap, &structure, &prep_opts);
    eprintln!(
        "prep: {} segments, delay {:.2} ms; prep+tracking kept {}/{}, \
         +steady-accel plateaus kept {}/{}",
        stats.segments,
        stats.delay_s * 1000.0,
        stats.tracked,
        stats.total,
        stats.kept,
        stats.total
    );
    if stats.tracked == 0 {
        eprintln!(
            "servo-ident: {capture_path}: the drive never tracked the commanded \
             trajectory — stiction breakaway or an untuned/lagging loop; enable \
             velocity feedforward and check the mechanics, then re-capture"
        );
        std::process::exit(2);
    }
    if stats.kept == 0 {
        eprintln!(
            "servo-ident: {capture_path}: no steady-accel plateaus — strokes too short \
             or jerk-limited accel never holds; lengthen strokes or lower accel"
        );
        std::process::exit(2);
    }

    let input = fit_input(&structure, &prepared);
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
            structure: structure.clone(),
            acc_mode: prepared.pp.acc_mode.clone(),
            vel_mode: prepared.pp.vel_mode.clone(),
            cs_mode: prepared.pp.cs_mode.clone(),
            torque: prepared.pp.torque.clone(),
            extra: prepared.pp.extra.clone(),
        };
        let res = residual_by_motor(&full, &r.params, &r.extra_params);
        let inband = band_limited_rms(&res, &prepared.pp.t, &prepared.keep, prep_opts.cutoff_hz);
        eprintln!(
            "in-band (<= {:.0} Hz) rms residual: {:.2} (0.1% rated)",
            prep_opts.cutoff_hz, inband
        );
    }
    let mut names: Vec<String> = Vec::with_capacity(3 * modes.len());
    for m in &modes {
        names.push(format!("mass_{m}"));
        names.push(format!("viscous_{m}"));
        names.push(format!("coulomb_{m}"));
    }
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
    let min_mass = r.params.mass.iter().copied().fold(f64::INFINITY, f64::min);
    let physical = min_mass > 0.0;

    if let (Some(t), Some(j), Some(d)) = (
        opt_f64(&args, "--rated-torque-nm"),
        opt_f64(&args, "--rotor-inertia-kgm2"),
        opt_f64(&args, "--rotation-distance-mm"),
    ) {
        for (k, (m, mass)) in modes.iter().zip(&r.params.mass).enumerate() {
            let drive_share = structure.frame[k]
                .iter()
                .fold(0.0_f64, |acc, f| acc.max(f.abs()));
            eprintln!(
                "recommended C00.06 (mode {m}): {:.0}%",
                c0006_recommendation(*mass * drive_share, t, d, j)
            );
        }
    }

    if !physical {
        eprintln!(
            "servo-ident: fitted mode mass {min_mass:.5} <= 0 is physically \
             impossible — C00.06 is J_load/J_rotor and load inertia cannot be \
             negative (drive accepts 0..12000%). The captured torque runs opposite \
             to the commanded acceleration: a drive torque-polarity / \
             invert_direction sign mismatch, not a real inertia. No profile written \
             — fix the capture sign convention and re-run."
        );
        return;
    }

    let per_motor = residual_by_motor(&input, &r.params, &r.extra_params);
    let rms: Vec<f64> = per_motor
        .iter()
        .map(|res| (res.iter().map(|e| e * e).sum::<f64>() / res.len() as f64).sqrt())
        .collect();
    let profile = render_profile(&r.params, &axes, &modes, &frame, &rms);
    let out = req(&args, "--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| {
        eprintln!("servo-ident: write {out}: {e}");
        std::process::exit(1);
    });
    eprintln!("profile written to {out}");
}
