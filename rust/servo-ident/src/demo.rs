//! `servo-cal demo`: builds run directories from the committed bench
//! fixtures (`test/fixtures/servo_captures`) so the dashboard can be
//! exercised without a bench. Three simulated notch-tuning attempts share
//! the same two gain-sweep steps (`s550` safe, `s700` target) and differ
//! only in the `0x2001.0x31` ambient notch value their manifests record —
//! the invisible-independent-variable case the dashboard's ambient-diff
//! column exists to surface.
//!
//! `attempt1`'s `s700` capture also gets a synthetic resonance stamped onto
//! its following-error channel (see `inject_decaying_resonance`) so the demo
//! can show the PSD overlay and the resonance-driven verdict fallback to
//! `s550` — `attempt2`/`attempt3` stay on the untouched fixture bytes.

use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use core::f64::consts::PI;

use flate2::read::GzDecoder;
use serde_json::json;

use crate::analyze::{build_run, write_run_outputs};
use crate::metrics::target_motion_segments;
use crate::scap::{Channel, Scap, RECORD_PREFIX_SIZE};
use crate::time_fmt::{iso8601_utc, stamp_utc};

const RESONANCE_ATTEMPT_SUFFIX: &str = "attempt1";
const RESONANCE_FREQ_HZ: f64 = 230.0;
const RESONANCE_AMPLITUDE_COUNTS: f64 = 12_000.0;
const RESONANCE_DECAY_TAU_S: f64 = 0.1;
const RESONANCE_DECAY_WINDOW_S: f64 = 0.5;

const SAFE_SCAP: &str = "cal_p880_s550_i2273_20260710_151516.scap.gz";
const SAFE_ACCEL: &str = "cal_p880_s550_i2273_accel_20260710_151519.csv.gz";
const TARGET_SCAP: &str = "cal_p1120_s700_i1786_20260710_151521.scap.gz";
const TARGET_ACCEL: &str = "cal_p1120_s700_i1786_accel_20260710_151524.csv.gz";

/// One entry of `servo_tuning.PANEL_PARAMS`, mirrored by hand — see
/// `docs/rewrite/servo-tuning-profiles.md`'s `PANEL_PARAMS` table, the
/// source of truth `klippy/extras/servo_tuning.py` derives its `addr` from
/// via `c_code_to_addr`. Kept here only so the demo dashboard has a
/// plausible `drive_state.json` to render; a mismatch against the Python
/// map is a documentation problem, not a wire contract this crate enforces.
struct DemoPanelParam {
    name: &'static str,
    c_code: &'static str,
    addr: &'static str,
    unit: &'static str,
    scale: f64,
    group: &'static str,
    description: &'static str,
    autofill: Option<&'static str>,
}

const DEMO_PANEL_PARAMS: [DemoPanelParam; 10] = [
    DemoPanelParam {
        name: "position_gain",
        c_code: "C01.00",
        addr: "0x2001.0x01",
        unit: "0.1 rad/s",
        scale: 10.0,
        group: "gains",
        description: "C01.00 position loop gain; autofilled from speed_gain as round(speed_gain * 1.6)",
        autofill: Some("gain_position_from_speed"),
    },
    DemoPanelParam {
        name: "speed_gain",
        c_code: "C01.01",
        addr: "0x2001.0x02",
        unit: "0.1 Hz",
        scale: 10.0,
        group: "gains",
        description: "C01.01 speed loop gain; the autofill source for position_gain and integral_time",
        autofill: None,
    },
    DemoPanelParam {
        name: "integral_time",
        c_code: "C01.02",
        addr: "0x2001.0x03",
        unit: "0.01 ms",
        scale: 100.0,
        group: "gains",
        description: "C01.02 speed integral time; autofilled from speed_gain as round(1250000 / speed_gain)",
        autofill: Some("gain_integral_from_speed"),
    },
    DemoPanelParam {
        name: "freq_cutoff",
        c_code: "C01.03",
        addr: "0x2001.0x04",
        unit: "Hz",
        scale: 1.0,
        group: "filters",
        description: "C01.03 speed loop filter cutoff frequency; bench rule-of-thumb \u{2248} speed_gain/10 \u{d7} 0.4, drive default 200",
        autofill: None,
    },
    DemoPanelParam {
        name: "adaptive_notch_mode",
        c_code: "C01.30",
        addr: "0x2001.0x31",
        unit: "",
        scale: 1.0,
        group: "notch",
        description: "C01.30 adaptive notch tuning mode: 0=locked, 1=retune after every restart, 2=auto, 3=restart adaptive tuning now",
        autofill: None,
    },
    DemoPanelParam {
        name: "gain_mode",
        c_code: "C00.04",
        addr: "0x2000.0x05",
        unit: "",
        scale: 1.0,
        group: "load",
        description: "C00.04 auto-tuning mode: 0=manual, 1=standard/stiffness table",
        autofill: None,
    },
    DemoPanelParam {
        name: "inertia_ratio",
        c_code: "C00.06",
        addr: "0x2000.0x07",
        unit: "%",
        scale: 1.0,
        group: "load",
        description: "C00.06 load inertia ratio",
        autofill: None,
    },
    DemoPanelParam {
        name: "c02_60",
        c_code: "C02.60",
        addr: "0x2002.0x61",
        unit: "",
        scale: 1.0,
        group: "experimental",
        description: "name unknown - bench-noted value 2000; identify in the vendor manual",
        autofill: None,
    },
    DemoPanelParam {
        name: "c02_62",
        c_code: "C02.62",
        addr: "0x2002.0x63",
        unit: "",
        scale: 1.0,
        group: "experimental",
        description: "name unknown - bench-noted value 30; identify in the vendor manual",
        autofill: None,
    },
    DemoPanelParam {
        name: "c02_63",
        c_code: "C02.63",
        addr: "0x2002.0x64",
        unit: "",
        scale: 1.0,
        group: "experimental",
        description: "name unknown - bench-noted value 150; identify in the vendor manual",
        autofill: None,
    },
];

const DEMO_MOTORS: [&str; 4] = ["motor_a", "motor_a1", "motor_b", "motor_b1"];

/// c_code -> current raw reading, shared by every `DEMO_MOTORS` entry — the
/// demo has no per-motor drift to show, only the panel's shape.
fn demo_readings() -> [(&'static str, i64); 10] {
    [
        ("C01.00", 880),
        ("C01.01", 550),
        ("C01.02", 2273),
        ("C01.03", 220),
        ("C01.30", 0),
        ("C00.04", 0),
        ("C00.06", 150),
        ("C02.60", 2000),
        ("C02.62", 30),
        ("C02.63", 150),
    ]
}

/// `gain_mode` (C00.04) and `inertia_ratio` (C00.06) are pinned in every
/// demo motor's `[motor] params:` block — the panel's cue that editing them
/// live won't survive a restart until the config is updated too.
const DEMO_PINNED_C_CODES: [&str; 2] = ["C00.04", "C00.06"];

/// Write `<out_dir>/drive_state.json` in the shape `SERVO_DUMP_TUNING`
/// produces (`docs/rewrite/servo-tuning-profiles.md`), so the tuning panel
/// has something plausible to render without a bench.
fn write_demo_drive_state(out_dir: &Path) -> Result<(), String> {
    let readings: std::collections::BTreeMap<&str, i64> = demo_readings().into_iter().collect();
    let params: Vec<serde_json::Value> = DEMO_PANEL_PARAMS
        .iter()
        .map(|p| {
            json!({
                "name": p.name,
                "c_code": p.c_code,
                "addr": p.addr,
                "type": "u16",
                "unit": p.unit,
                "scale": p.scale,
                "group": p.group,
                "description": p.description,
                "autofill": p.autofill,
            })
        })
        .collect();
    let motor_readings = json!(readings);
    let motors: serde_json::Value = DEMO_MOTORS
        .iter()
        .map(|m| (m.to_string(), motor_readings.clone()))
        .collect::<serde_json::Map<_, _>>()
        .into();
    let pinned: serde_json::Value = DEMO_PINNED_C_CODES
        .iter()
        .map(|c| (c.to_string(), json!(readings[c])))
        .collect::<serde_json::Map<_, _>>()
        .into();
    let config_pins: serde_json::Value = DEMO_MOTORS
        .iter()
        .map(|m| (m.to_string(), pinned.clone()))
        .collect::<serde_json::Map<_, _>>()
        .into();
    let payload = json!({
        "version": 1,
        "created_utc": iso8601_utc(SystemTime::now()),
        "params": params,
        "motors": motors,
        "config_pins": config_pins,
    });
    let path = out_dir.join("drive_state.json");
    let tmp = out_dir.join("drive_state.json.tmp");
    std::fs::write(
        &tmp,
        serde_json::to_string_pretty(&payload).map_err(|e| format!("{e}"))?,
    )
    .map_err(|e| format!("write {}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, &path).map_err(|e| format!("rename to {}: {e}", path.display()))
}

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

fn eff_offset(ch: &Channel, drive_idx: usize, block_size: usize) -> usize {
    if ch.offset >= RECORD_PREFIX_SIZE {
        ch.offset + drive_idx * block_size
    } else {
        ch.offset
    }
}

fn find_channel<'a>(cap: &'a Scap, name: &str) -> Result<&'a Channel, String> {
    cap.channel(name)
        .ok_or_else(|| format!("capture has no channel {name:?} to patch"))
}

/// Stamp a decaying `RESONANCE_FREQ_HZ` sinusoid onto every drive's
/// following-error channel for `RESONANCE_DECAY_WINDOW_S` after each of its
/// motion segments ends, keeping `position_actual = target_counts -
/// following_error` exact so `ferr_crosscheck_max` stays zero. Operates on
/// already-gunzipped record bytes; the on-disk fixture is never touched.
fn inject_decaying_resonance(bytes: &[u8]) -> Result<Vec<u8>, String> {
    let cap = Scap::from_bytes(bytes)?;
    let nl = bytes
        .iter()
        .position(|&b| b == b'\n')
        .ok_or("capture has no header line")?;
    let body_start = nl + 1;
    let record_size = cap.header.record_size;
    let n_drives = cap.header.drives.len();
    let block_size = (record_size - RECORD_PREFIX_SIZE) / n_drives;
    let fs = cap.fs();
    let decay_samples = (RESONANCE_DECAY_WINDOW_S * fs).round() as usize;

    let ferr_ch = find_channel(&cap, "following_error")?;
    let target_ch = find_channel(&cap, "target_counts")?;
    let pos_ch = find_channel(&cap, "position_actual")?;
    let (ferr_size, target_size, pos_size) = (
        ferr_ch.dtype.itemsize(),
        target_ch.dtype.itemsize(),
        pos_ch.dtype.itemsize(),
    );

    let mut out = bytes.to_vec();
    for idx in 0..n_drives {
        let target = cap.read_i64(idx, "target_counts")?;
        let segs = target_motion_segments(&target, fs);
        let ferr_eff = eff_offset(ferr_ch, idx, block_size);
        let target_eff = eff_offset(target_ch, idx, block_size);
        let pos_eff = eff_offset(pos_ch, idx, block_size);
        for (_, move_end) in segs {
            for k in 0..decay_samples {
                let r = move_end + k;
                if r >= cap.n_records {
                    break;
                }
                let t = k as f64 / fs;
                let signal = RESONANCE_AMPLITUDE_COUNTS
                    * libm::exp(-t / RESONANCE_DECAY_TAU_S)
                    * libm::sin(2.0 * PI * RESONANCE_FREQ_HZ * t);

                let rec_off = body_start + r * record_size;
                let ferr_off = rec_off + ferr_eff;
                let old_ferr = ferr_ch.dtype.read_i64(&out[ferr_off..ferr_off + ferr_size]);
                let new_ferr = old_ferr.saturating_add(signal.round() as i64);
                ferr_ch
                    .dtype
                    .write_i64_saturating(&mut out[ferr_off..ferr_off + ferr_size], new_ferr)?;

                let target_off = rec_off + target_eff;
                let target_val = target_ch
                    .dtype
                    .read_i64(&out[target_off..target_off + target_size]);
                let new_pos = target_val - new_ferr;
                let pos_off = rec_off + pos_eff;
                pos_ch
                    .dtype
                    .write_i64_saturating(&mut out[pos_off..pos_off + pos_size], new_pos)?;
            }
        }
    }
    Ok(out)
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
        let target_scap_path = run_dir.join("step_s700.scap");
        gunzip(&fixtures_dir.join(TARGET_SCAP), &target_scap_path)?;
        if attempt.suffix == RESONANCE_ATTEMPT_SUFFIX {
            let bytes = std::fs::read(&target_scap_path)
                .map_err(|e| format!("read {}: {e}", target_scap_path.display()))?;
            let injected = inject_decaying_resonance(&bytes)?;
            std::fs::write(&target_scap_path, injected)
                .map_err(|e| format!("write {}: {e}", target_scap_path.display()))?;
        }
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
    write_demo_drive_state(out_dir)?;
    Ok(run_dirs)
}
