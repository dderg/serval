//! `servo-cal demo`: builds run directories from the committed bench
//! fixtures (`test/fixtures/servo_captures`) so the dashboard can be
//! exercised without a bench. Three simulated notch-tuning attempts share
//! the same two gain-sweep steps (`s550` safe, `s700` target) and differ
//! only in the `0x2001.0x31` ambient notch value their manifests record —
//! the invisible-independent-variable case the dashboard's ambient-diff
//! column exists to surface.

use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use flate2::read::GzDecoder;
use serde_json::json;

use crate::analyze::{build_run, write_run_outputs};
use crate::time_fmt::{iso8601_utc, stamp_utc};

const SAFE_SCAP: &str = "cal_p880_s550_i2273_20260710_151516.scap.gz";
const SAFE_ACCEL: &str = "cal_p880_s550_i2273_accel_20260710_151519.csv.gz";
const TARGET_SCAP: &str = "cal_p1120_s700_i1786_20260710_151521.scap.gz";
const TARGET_ACCEL: &str = "cal_p1120_s700_i1786_accel_20260710_151524.csv.gz";

pub struct DemoAttempt {
    pub suffix: &'static str,
    pub notch: i64,
}

/// Three attempts, oldest first: the ambient notch value walks 3 -> 1 -> 0,
/// same order the real session in the plan's motivation narrated.
pub const DEMO_ATTEMPTS: [DemoAttempt; 3] = [
    DemoAttempt {
        suffix: "attempt1",
        notch: 3,
    },
    DemoAttempt {
        suffix: "attempt2",
        notch: 1,
    },
    DemoAttempt {
        suffix: "attempt3",
        notch: 0,
    },
];

/// `test/fixtures/servo_captures` resolved from the running binary's path:
/// `<repo>/rust/target/<profile>/servo-cal` -> `<repo>`. Overridable via
/// `--fixtures` for a relocated binary.
pub fn default_fixtures_dir() -> Result<PathBuf, String> {
    let exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
    let repo_root = exe.ancestors().nth(4).ok_or_else(|| {
        format!(
            "{}: expected <repo>/rust/target/<profile>/servo-cal — pass --fixtures explicitly",
            exe.display()
        )
    })?;
    Ok(repo_root.join("test/fixtures/servo_captures"))
}

fn gunzip(src: &Path, dst: &Path) -> Result<(), String> {
    let file = std::fs::File::open(src)
        .map_err(|e| format!("open {}: {e} (try --fixtures)", src.display()))?;
    let mut decoder = GzDecoder::new(file);
    let mut out = Vec::new();
    decoder
        .read_to_end(&mut out)
        .map_err(|e| format!("gunzip {}: {e}", src.display()))?;
    std::fs::write(dst, out).map_err(|e| format!("write {}: {e}", dst.display()))
}

fn manifest_json(attempt: &DemoAttempt, created: SystemTime) -> serde_json::Value {
    let created_utc = iso8601_utc(created);
    json!({
        "version": 1,
        "experiment": "gain_sweep",
        "tag": format!("cal_{}", attempt.suffix),
        "created_utc": created_utc,
        "axis": "X",
        "kinematics": "corexy",
        "git_rev": "demo",
        "session_id": format!("demo-{}", attempt.suffix),
        "stroke_plan": {
            "start": 30.0, "end": 220.0, "speed": 100.0, "accel": 3000.0,
            "iterations": 1, "dwell_ms": 700
        },
        "motors": [
            {"name": "motor_a", "invert": false, "rotation_distance": 40.0, "counts_per_mm": 3276.8}
        ],
        "belts": "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1",
        "steps": [
            {
                "name": "s550",
                "swept": {"position": 880, "speed": 550, "integral": 2273},
                "applied": [{"servo": "motor_a", "addr": "0x2001.0x01", "type": "u16", "value": 880}],
                "capture": "step_s550.scap",
                "accel": "step_s550_accel.csv"
            },
            {
                "name": "s700",
                "swept": {"position": 1120, "speed": 700, "integral": 1786},
                "applied": [{"servo": "motor_a", "addr": "0x2001.0x01", "type": "u16", "value": 1120}],
                "capture": "step_s700.scap",
                "accel": "step_s700_accel.csv"
            }
        ],
        "ambient": {
            "journal_params": {"motor_a": {"0x2001.0x31": attempt.notch}},
            "param_writes_since_last_run": [
                {"servo": "motor_a", "addr": "0x2001.0x31", "value": attempt.notch, "time_utc": created_utc}
            ]
        }
    })
}

/// Build `DEMO_ATTEMPTS` under `out_dir`, gunzipping captures from
/// `fixtures_dir` and running `analyze` on each so `results.json` /
/// `plot_series.json` exist. Returns the run directories, oldest first.
pub fn build_demo(out_dir: &Path, fixtures_dir: &Path) -> Result<Vec<PathBuf>, String> {
    std::fs::create_dir_all(out_dir).map_err(|e| format!("mkdir {}: {e}", out_dir.display()))?;
    let mut run_dirs = Vec::new();
    for attempt in &DEMO_ATTEMPTS {
        let created = SystemTime::now();
        let run_dir = out_dir.join(format!("cal_{}_{}", attempt.suffix, stamp_utc(created)));
        std::fs::create_dir_all(&run_dir)
            .map_err(|e| format!("mkdir {}: {e}", run_dir.display()))?;

        gunzip(
            &fixtures_dir.join(SAFE_SCAP),
            &run_dir.join("step_s550.scap"),
        )?;
        gunzip(
            &fixtures_dir.join(SAFE_ACCEL),
            &run_dir.join("step_s550_accel.csv"),
        )?;
        gunzip(
            &fixtures_dir.join(TARGET_SCAP),
            &run_dir.join("step_s700.scap"),
        )?;
        gunzip(
            &fixtures_dir.join(TARGET_ACCEL),
            &run_dir.join("step_s700_accel.csv"),
        )?;

        let manifest = manifest_json(attempt, created);
        std::fs::write(
            run_dir.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).map_err(|e| format!("{e}"))?,
        )
        .map_err(|e| format!("write manifest.json: {e}"))?;

        let (results, plot) = build_run(&run_dir)?;
        write_run_outputs(&run_dir, &results, &plot)?;

        run_dirs.push(run_dir);
        std::thread::sleep(Duration::from_millis(15));
    }
    Ok(run_dirs)
}
