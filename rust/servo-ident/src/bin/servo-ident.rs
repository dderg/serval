//! Usage: servo-ident --capture run.csv \
//!   --frame "r0c0,r0c1,...;r1c0,r1c1,..." --modes x[,y] --axes a[,b,...] \
//!   --out profile.toml \
//!   [--rated-torque-nm T --rotor-inertia-kgm2 J --rotation-distance-mm D]
#![allow(clippy::exit)]

use servo_ident::capture::{parse_capture_csv, Capture};
use servo_ident::fit::residual_by_motor;
use servo_ident::fit::{fit, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::pipeline::{pooled_input, prepare, split_captures, Prepared};
use servo_ident::prep::{band_limited_rms, PrepOptions};
use servo_ident::profile_out::{c0006_recommendation, render_profile, PairSplit};
use servo_ident::split::{fit_pair_splits, PairReport};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn args_all(args: &[String], key: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < args.len() {
        if args[i] == key {
            if let Some(v) = args.get(i + 1) {
                out.push(v.clone());
            }
            i += 2;
        } else {
            i += 1;
        }
    }
    out
}

fn parse_signs(spec: &str, n_slots: usize) -> Vec<f64> {
    let v: Vec<f64> = spec
        .split(',')
        .map(|s| match s.trim() {
            "1" | "+1" => 1.0,
            "-1" => -1.0,
            other => {
                eprintln!("servo-ident: --signs entry {other:?} must be 1 or -1");
                std::process::exit(1);
            }
        })
        .collect();
    if v.len() != n_slots {
        eprintln!(
            "servo-ident: --signs has {} entries, frame has {n_slots} slots",
            v.len()
        );
        std::process::exit(1);
    }
    v
}

fn report_splits(reports: &[PairReport], axes: &[&str]) -> Vec<PairSplit> {
    const LABELS: [&str; 6] = ["I0", "I1", "V0", "V1", "C0", "C1"];
    let mut out = Vec::with_capacity(reports.len());
    for r in reports {
        let a = axes[r.split.first];
        let b = axes[r.split.second];
        eprintln!(
            "pair {a}/{b} (λ={:+.0}): rms(D) {:.2} -> {:.2} (odd model), {} samples",
            r.lambda, r.rms_before, r.rms_after, r.samples
        );
        for i in 0..6 {
            eprintln!(
                "  w_{} = {:+.6e}  (stderr {:.2e}, t = {:+.2})",
                LABELS[i], r.split.w[i], r.w_stderr[i], r.w_tvalue[i]
            );
        }
        eprintln!(
            "  diag |F_I| coeff {:+.4e} (contrib {:.3}), |F_V| coeff {:+.4e} (contrib {:.3}); \
             largest odd contrib {:.3}",
            r.even_coeff[0],
            r.even_contribution[0],
            r.even_coeff[1],
            r.even_contribution[1],
            r.max_odd_contribution
        );
        for (c, off) in r.intercepts.iter().enumerate() {
            eprintln!("  diag intercept[capture {c}] {off:+.3}");
        }
        if r.role_dependent {
            eprintln!(
                "  WARNING pair {a}/{b}: role-dependent split detected — check belt \
                 tension/pulley drag; not fed forward"
            );
        }
        out.push(r.split.clone());
    }
    out
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

const KNOWN_KEYS: [&str; 13] = [
    "--capture",
    "--frame",
    "--modes",
    "--axes",
    "--out",
    "--signs",
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

    let pairs = structure.pairs();
    let signs: Option<Vec<f64>> =
        arg(&args, "--signs").map(|s| parse_signs(&s, structure.axis_count()));
    if !pairs.is_empty() && signs.is_none() {
        eprintln!(
            "servo-ident: frame has {} motor pair(s); --signs \"±1,...\" (one per \
             slot) is required so the belt coordinate and differential transfer",
            pairs.len()
        );
        std::process::exit(1);
    }

    let capture_paths = args_all(&args, "--capture");
    if capture_paths.is_empty() {
        eprintln!("servo-ident: missing required --capture");
        std::process::exit(1);
    }
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

    let mut prepared: Vec<Prepared> = Vec::new();
    for path in &capture_paths {
        let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
            eprintln!("servo-ident: read {path}: {e}");
            std::process::exit(1);
        });
        let cap: Capture = parse_capture_csv(&text, &axes).unwrap_or_else(|e| {
            eprintln!("servo-ident: capture {path} invalid: {e:?}");
            std::process::exit(1);
        });
        let (pr, stats) = prepare(cap, &structure, &prep_opts);
        eprintln!(
            "prep [{path}]: {} segments, delay {:.2} ms; prep+tracking kept {}/{}, \
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
                "servo-ident: {path}: the drive never tracked the commanded \
                 trajectory — stiction breakaway or an untuned/lagging loop; enable \
                 velocity feedforward and check the mechanics, then re-capture"
            );
            std::process::exit(2);
        }
        if stats.kept == 0 {
            eprintln!(
                "servo-ident: {path}: no steady-accel plateaus — strokes too short \
                 or jerk-limited accel never holds; lengthen strokes or lower accel"
            );
            std::process::exit(2);
        }
        prepared.push(pr);
    }

    let input = pooled_input(&structure, &prepared);
    let r = fit(&input, &FitOptions::default()).unwrap_or_else(|e| {
        eprintln!("servo-ident: refusing to emit a profile: {e:?}");
        std::process::exit(2);
    });

    eprintln!(
        "fit: {} pooled samples/motor, rms residual {:.2} (0.1% rated), condition {:.1e}",
        r.samples, r.rms_residual, r.condition
    );
    if prep_opts.cutoff_hz > 0.0 {
        for (ci, pr) in prepared.iter().enumerate() {
            let full = FitInput {
                structure: structure.clone(),
                acc_mode: pr.pp.acc_mode.clone(),
                vel_mode: pr.pp.vel_mode.clone(),
                cs_mode: pr.pp.cs_mode.clone(),
                torque: pr.pp.torque.clone(),
                extra: pr.pp.extra.clone(),
            };
            let res = residual_by_motor(&full, &r.params, &r.extra_params);
            let inband = band_limited_rms(&res, &pr.pp.t, &pr.keep, prep_opts.cutoff_hz);
            eprintln!(
                "in-band (<= {:.0} Hz) rms residual [capture {ci}]: {:.2} (0.1% rated)",
                prep_opts.cutoff_hz, inband
            );
        }
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
        for (m, mass) in modes.iter().zip(&r.params.mass) {
            eprintln!(
                "recommended C00.06 (mode {m}): {:.0}%",
                c0006_recommendation(*mass, t, d, j)
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

    let pair_splits: Vec<PairSplit> = if pairs.is_empty() {
        Vec::new()
    } else {
        for (path, pr) in capture_paths.iter().zip(&prepared) {
            if !pr.cap.has_positions() {
                eprintln!(
                    "servo-ident: frame has motor pairs but {path} carries no \
                     pos_<axis> columns — the load-share split needs commanded \
                     positions; re-capture with them"
                );
                std::process::exit(2);
            }
        }
        let signs = signs.as_ref().expect("signs required when pairs present");
        let scaps = split_captures(&prepared);
        let reports = fit_pair_splits(&structure, &r.params, signs, prep_opts.cutoff_hz, &scaps);
        report_splits(&reports, &axes)
    };

    let per_motor = residual_by_motor(&input, &r.params, &r.extra_params);
    let rms: Vec<f64> = per_motor
        .iter()
        .map(|res| (res.iter().map(|e| e * e).sum::<f64>() / res.len() as f64).sqrt())
        .collect();
    let profile = render_profile(&r.params, &axes, &modes, &frame, &rms, &pair_splits);
    let out = req(&args, "--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| {
        eprintln!("servo-ident: write {out}: {e}");
        std::process::exit(1);
    });
    eprintln!("profile written to {out}");
}
