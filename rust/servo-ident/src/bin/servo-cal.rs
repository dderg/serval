//! servo-cal: the Rust analysis core for servo `.scap` captures.
//!
//! Subcommands:
//!   analyze <run-dir>              write results.json + plot_series.json, print table
//!   analyze --scap <file.scap>     single capture, per-drive table to stdout
//!       [--json <path>]            also write a results-shaped step object
//!       [--dump-csv <path>]        write per-drive raw series CSV
//!       [--combine <spec> --axis X] add the CoreXY belt combine
//!   fit --capture <file.scap> --frame "r0;r1" --modes x[,y] --axes a[,b,...]
//!       --out profile.toml [--rated-torque-nm T --rotor-inertia-kgm2 J
//!       --rotation-distance-mm D --cutoff-hz C --blank-ms B --max-delay-ms M
//!       --ripple-period-mm P]
//!   serve --dir <captures_root> [--port 8085] [--host 127.0.0.1]
//!       [--live-sock /tmp/kalico-ethercat.sock.live]
//!   demo <out-dir> [--fixtures <dir>]
#![allow(clippy::exit)]

use std::path::{Path, PathBuf};

use servo_ident::analyze::{analyze_capture, analyze_run, dump_csv, print_step};
use servo_ident::demo::{build_demo, default_fixtures_dir};
use servo_ident::fit::residual_by_motor;
use servo_ident::fit::{fit, FitOptions};
use servo_ident::fit_driver::scap_to_capture;
use servo_ident::http;
use servo_ident::live_stream::{LiveTap, DEFAULT_IDLE_TIMEOUT, DEFAULT_TAP_SOCKET};
use servo_ident::metrics::{DEFAULT_SETTLE_BAND_COUNTS, DEFAULT_TORQUE_LIMIT_PER_MILLE};
use servo_ident::model::Structure;
use servo_ident::pipeline::{fit_input, full_fit_input, prepare};
use servo_ident::prep::{band_limited_rms, PrepOptions};
use servo_ident::profile_out::{c0006_recommendation, render_profile};
use servo_ident::scap::Scap;
use servo_ident::serve;
use servo_ident::split::{fit_pair_splits, report_pair_splits, SplitCapture};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn opt_f64(args: &[String], key: &str) -> Option<f64> {
    arg(args, key).map(|v| {
        v.parse().unwrap_or_else(|_| {
            eprintln!("servo-cal: bad {key} {v:?}");
            std::process::exit(1);
        })
    })
}

fn req(args: &[String], key: &str) -> String {
    arg(args, key).unwrap_or_else(|| {
        eprintln!("servo-cal: missing required {key}");
        std::process::exit(1);
    })
}

fn die(msg: &str) -> ! {
    eprintln!("servo-cal: {msg}");
    std::process::exit(1);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let sub = args.get(1).map(String::as_str).unwrap_or_else(|| {
        die("usage: servo-cal <analyze|fit|serve|demo> ...");
    });
    match sub {
        "analyze" => cmd_analyze(&args),
        "fit" => cmd_fit(&args),
        "serve" => cmd_serve(&args),
        "demo" => cmd_demo(&args),
        other => die(&format!(
            "unknown subcommand {other:?} (want analyze|fit|serve|demo)"
        )),
    }
}

fn cmd_serve(args: &[String]) {
    let dir = arg(args, "--dir").unwrap_or_else(|| die("serve needs --dir <captures_root>"));
    let captures_root = PathBuf::from(&dir);
    if !captures_root.is_dir() {
        die(&format!("{dir} is not a directory"));
    }
    let host = arg(args, "--host").unwrap_or_else(|| "127.0.0.1".to_string());
    let port: u16 = arg(args, "--port")
        .map(|p| {
            p.parse()
                .unwrap_or_else(|_| die(&format!("bad --port {p:?}")))
        })
        .unwrap_or(8085);
    let live_sock = arg(args, "--live-sock").unwrap_or_else(|| DEFAULT_TAP_SOCKET.to_string());
    let tap = LiveTap::new(PathBuf::from(&live_sock), DEFAULT_IDLE_TIMEOUT);
    let listener = http::bind(&host, port).unwrap_or_else(|e| die(&e));
    println!("servo-cal serve: {dir} on http://{host}:{port} (live tap: {live_sock})");
    http::run(listener, move |req| {
        serve::handle_with_live_tap(&captures_root, &tap, req)
    });
}

fn cmd_demo(args: &[String]) {
    let out_dir = args
        .get(2)
        .filter(|a| !a.starts_with("--"))
        .unwrap_or_else(|| die("demo needs an <out-dir>"));
    let fixtures_dir = match arg(args, "--fixtures") {
        Some(f) => PathBuf::from(f),
        None => default_fixtures_dir().unwrap_or_else(|e| die(&e)),
    };
    if !fixtures_dir.is_dir() {
        die(&format!(
            "fixtures dir {} not found — pass --fixtures <path>",
            fixtures_dir.display()
        ));
    }
    let run_dirs = build_demo(Path::new(out_dir), &fixtures_dir).unwrap_or_else(|e| die(&e));
    for dir in &run_dirs {
        println!("wrote {}", dir.display());
    }
    println!("\nnext: servo-cal serve --dir {out_dir} --port 8085");
}

fn cmd_analyze(args: &[String]) {
    if let Some(scap_path) = arg(args, "--scap") {
        let cap = Scap::load(&scap_path).unwrap_or_else(|e| die(&e));
        if let Some(csv) = arg(args, "--dump-csv") {
            dump_csv(&cap, Path::new(&csv)).unwrap_or_else(|e| die(&e));
            eprintln!("wrote per-drive series to {csv}");
        }
        let combine = arg(args, "--combine");
        let axis = arg(args, "--axis");
        let ff_lead: usize = arg(args, "--ff-lead-cycles")
            .map(|v| {
                v.parse()
                    .unwrap_or_else(|_| die(&format!("bad --ff-lead-cycles {v:?}")))
            })
            .unwrap_or(0);
        let name = Path::new(&scap_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("capture")
            .to_string();
        let (step, _plot) = analyze_capture(
            &cap,
            &name,
            DEFAULT_SETTLE_BAND_COUNTS,
            DEFAULT_TORQUE_LIMIT_PER_MILLE,
            combine.as_deref(),
            axis.as_deref(),
            None,
            ff_lead,
            None,
        )
        .unwrap_or_else(|e| die(&e));
        println!("file: {scap_path}");
        print_step(&step);
        if let Some(json_path) = arg(args, "--json") {
            let text = serde_json::to_string_pretty(&step).unwrap_or_else(|e| die(&format!("{e}")));
            std::fs::write(&json_path, text)
                .unwrap_or_else(|e| die(&format!("write {json_path}: {e}")));
            eprintln!("wrote {json_path}");
        }
        return;
    }
    let dir = args
        .get(2)
        .filter(|a| !a.starts_with("--"))
        .unwrap_or_else(|| die("analyze needs a run directory or --scap <file>"));
    if arg(args, "--dump-csv").is_some() {
        die("--dump-csv works with --scap <file>, not a run directory");
    }
    let incremental = args.iter().any(|a| a == "--incremental");
    analyze_run(Path::new(dir), incremental).unwrap_or_else(|e| die(&e));
}

const FIT_KEYS: [&str; 13] = [
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
    "--drive",
];

fn reject_unknown_fit_flags(args: &[String]) {
    let mut i = 2;
    while i < args.len() {
        let a = &args[i];
        if !FIT_KEYS.contains(&a.as_str()) {
            die(&format!("unknown fit argument {a:?}"));
        }
        i += 2;
    }
}

fn parse_frame(spec: &str) -> Vec<Vec<f64>> {
    spec.split(';')
        .map(|row| {
            row.split(',')
                .map(|e| {
                    e.trim()
                        .parse::<f64>()
                        .unwrap_or_else(|_| die(&format!("bad frame entry {e:?}")))
                })
                .collect()
        })
        .collect()
}

fn cmd_fit(args: &[String]) {
    reject_unknown_fit_flags(args);
    let frame = parse_frame(&req(args, "--frame"));
    let modes_arg = req(args, "--modes");
    let modes: Vec<&str> = modes_arg.split(',').map(str::trim).collect();
    if modes.len() != frame.len() {
        die(&format!(
            "{} modes given, frame has {} rows",
            modes.len(),
            frame.len()
        ));
    }
    let structure = Structure::new(frame.clone());
    let axes_arg = req(args, "--axes");
    let axes: Vec<&str> = axes_arg.split(',').map(str::trim).collect();
    if axes.len() != structure.axis_count() {
        die(&format!(
            "{} axes given, frame has {} columns",
            axes.len(),
            structure.axis_count()
        ));
    }
    if args.iter().filter(|a| *a == "--capture").count() > 1 {
        die("--capture given more than once — the fit consumes exactly one capture");
    }
    let capture_path = req(args, "--capture");
    let mut prep_opts = PrepOptions::default();
    if let Some(v) = opt_f64(args, "--cutoff-hz") {
        prep_opts.cutoff_hz = v;
    }
    if let Some(v) = opt_f64(args, "--blank-ms") {
        prep_opts.blank_reversal_s = v / 1000.0;
    }
    if let Some(v) = opt_f64(args, "--max-delay-ms") {
        prep_opts.max_delay_s = v / 1000.0;
    }
    prep_opts.ripple_period_mm =
        opt_f64(args, "--ripple-period-mm").or_else(|| opt_f64(args, "--rotation-distance-mm"));

    let cap = Scap::load(&capture_path).unwrap_or_else(|e| die(&e));
    let fit_cap = scap_to_capture(&cap, &axes).unwrap_or_else(|e| die(&e));
    let (prepared, stats) = prepare(&fit_cap, &structure, &prep_opts);
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
            "servo-cal: {capture_path}: the drive never tracked the commanded \
             trajectory — stiction breakaway or an untuned/lagging loop; enable \
             velocity feedforward and check the mechanics, then re-capture"
        );
        std::process::exit(2);
    }
    if stats.kept == 0 {
        eprintln!(
            "servo-cal: {capture_path}: no steady-accel plateaus — strokes too short \
             or jerk-limited accel never holds; lengthen strokes or lower accel"
        );
        std::process::exit(2);
    }

    let input = fit_input(&structure, &prepared);
    let r = fit(&input, &FitOptions::default()).unwrap_or_else(|e| {
        eprintln!("servo-cal: refusing to emit a profile: {e:?}");
        std::process::exit(2);
    });
    eprintln!(
        "fit: {} samples/motor, rms residual {:.2} (0.1% rated), condition {:.1e}",
        r.samples, r.rms_residual, r.condition
    );
    let full = full_fit_input(&structure, &prepared);
    let full_residual = residual_by_motor(&full, &r.params, &r.extra_params);
    if prep_opts.cutoff_hz > 0.0 {
        let inband = band_limited_rms(
            &full_residual,
            &prepared.pp.t,
            &prepared.keep,
            prep_opts.cutoff_hz,
        );
        eprintln!(
            "in-band (<= {:.0} Hz) rms residual: {:.2} (0.1% rated)",
            prep_opts.cutoff_hz, inband
        );
    }
    let split_capture = SplitCapture {
        cap: &fit_cap,
        residual_filt: &full_residual,
        keep: &prepared.keep,
    };
    let pair_reports = fit_pair_splits(&structure, &r.params, prep_opts.cutoff_hz, &split_capture)
        .unwrap_or_else(|e| {
            eprintln!("servo-cal: refusing to emit a profile: pair split fit failed: {e:?}");
            std::process::exit(2);
        });
    let pair_splits = report_pair_splits(&pair_reports, &axes);
    let min_mass = r.params.mass.iter().copied().fold(f64::INFINITY, f64::min);
    let physical = min_mass > 0.0;

    if let (Some(t), Some(j), Some(d)) = (
        opt_f64(args, "--rated-torque-nm"),
        opt_f64(args, "--rotor-inertia-kgm2"),
        opt_f64(args, "--rotation-distance-mm"),
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
            "servo-cal: fitted mode mass {min_mass:.5} <= 0 is physically \
             impossible — a torque-polarity / invert_direction sign mismatch, \
             not a real inertia. No profile written."
        );
        return;
    }

    let per_motor = residual_by_motor(&input, &r.params, &r.extra_params);
    let rms: Vec<f64> = per_motor
        .iter()
        .map(|res| (res.iter().map(|e| e * e).sum::<f64>() / res.len() as f64).sqrt())
        .collect();
    let profile = render_profile(&r.params, &axes, &modes, &frame, &rms, &pair_splits);
    let out = req(args, "--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| die(&format!("write {out}: {e}")));
    eprintln!("profile written to {out}");
}
