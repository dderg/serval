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

use std::collections::BTreeMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use core::f64::consts::PI;

use flate2::read::GzDecoder;
use schemars::JsonSchema;
use serde::Serialize;
use ts_rs::TS;

use crate::analyze::{build_run, write_run_outputs};
use crate::metrics::target_motion_segments;
use crate::results::Applied;
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

/// One entry of `servo_tuning.PANEL_PARAMS`, mirrored — see
/// `docs/rewrite/servo-tuning-profiles.md`'s `PANEL_PARAMS` table, the
/// source of truth `klippy/extras/servo_tuning.py` derives its `addr` from
/// via `c_code_to_addr`. Kept here only so the demo dashboard has a
/// plausible `drive_state.json` to render; a mismatch against the Python
/// map is a documentation problem, not a wire contract this crate enforces.
struct DemoPanelParam {
    name: String,
    c_code: String,
    addr: String,
    unit: &'static str,
    group: &'static str,
    description: String,
    options: Option<&'static [(&'static str, &'static str)]>,
    reading: i64,
}

fn plain_param(
    name: &str,
    c_code: &str,
    addr: &str,
    unit: &'static str,
    group: &'static str,
    description: &str,
    reading: i64,
) -> DemoPanelParam {
    DemoPanelParam {
        name: name.into(),
        c_code: c_code.into(),
        addr: addr.into(),
        unit,
        group,
        description: description.into(),
        options: None,
        reading,
    }
}

const SPEED_FEEDBACK_FILTER_OPTIONS: &[(&str, &str)] = &[
    ("0", "internal setting"),
    ("1", "low-pass filter"),
    ("2", "overlapping average"),
    ("3", "speed observer"),
    ("4", "no filter"),
];

/// A6-EC manual 7.10: notch n occupies C01.40+3(n-1) .. +2. Slots 1-3
/// carry the bench-noted values, 4-5 sit at the drive's parked defaults.
const NOTCH_READINGS: [[i64; 3]; 5] = [
    [345, 160, 200],
    [225, 200, 120],
    [140, 100, 350],
    [8000, 0, 1000],
    [8000, 0, 1000],
];

fn demo_panel_params() -> Vec<DemoPanelParam> {
    let mut params = vec![
        plain_param(
            "position_gain",
            "C01.00",
            "0x2001.0x01",
            "0.1 rad/s",
            "gains",
            "C01.00 position loop gain",
            880,
        ),
        plain_param(
            "speed_gain",
            "C01.01",
            "0x2001.0x02",
            "0.1 Hz",
            "gains",
            "C01.01 speed loop gain",
            550,
        ),
        plain_param(
            "integral_time",
            "C01.02",
            "0x2001.0x03",
            "0.01 ms",
            "gains",
            "C01.02 speed integral time",
            2273,
        ),
        plain_param(
            "torque_filter_cutoff",
            "C01.03",
            "0x2001.0x04",
            "Hz",
            "filters",
            "C01.03 1st torque reference filter cutoff frequency (manual 7.3)",
            220,
        ),
    ];
    let notch_kinds: [(&str, &str, &'static str); 3] = [
        ("freq", "center frequency", "Hz"),
        ("width", "width level", "0.1%"),
        ("depth", "depth level", "0.1%"),
    ];
    for n in 1usize..=5 {
        for (k, (kind, kind_desc, unit)) in notch_kinds.iter().enumerate() {
            let code = 0x40 + (n - 1) * 3 + k;
            params.push(plain_param(
                &format!("notch_{n}_{kind}"),
                &format!("C01.{code:02X}"),
                &format!("0x2001.0x{:02x}", code + 1),
                unit,
                "notch",
                &format!("C01.{code:02X} {kind_desc} of the notch {n} (manual 7.10)"),
                NOTCH_READINGS[n - 1][k],
            ));
        }
    }
    params.push(DemoPanelParam {
        options: Some(SPEED_FEEDBACK_FILTER_OPTIONS),
        ..plain_param(
            "speed_feedback_filter",
            "C01.10",
            "0x2001.0x11",
            "",
            "speed_observer",
            "C01.10 speed feedback filter; 3 enables the speed observer (manual 7.11)",
            3,
        )
    });
    params.extend([
        plain_param(
            "speed_observer_gain",
            "C02.30",
            "0x2002.0x31",
            "0.1 Hz",
            "speed_observer",
            "C02.30 speed observer gain (manual 7.11)",
            8000,
        ),
        plain_param(
            "speed_observer_inertia",
            "C02.31",
            "0x2002.0x32",
            "0.1%",
            "speed_observer",
            "C02.31 speed observer inertia correction (manual 7.11)",
            1000,
        ),
        plain_param(
            "speed_observer_cutoff",
            "C02.32",
            "0x2002.0x33",
            "Hz",
            "speed_observer",
            "C02.32 speed observer feedback low-pass cutoff (manual 7.11)",
            0,
        ),
        plain_param(
            "disturbance_gain",
            "C02.60",
            "0x2002.0x61",
            "0.1 Hz",
            "disturbance_observer",
            "C02.60 disturbance observer gain (manual 7.12)",
            2000,
        ),
        plain_param(
            "disturbance_inertia",
            "C02.61",
            "0x2002.0x62",
            "0.1%",
            "disturbance_observer",
            "C02.61 disturbance observer inertia correction coefficient (manual 7.12)",
            1000,
        ),
        plain_param(
            "disturbance_cutoff",
            "C02.62",
            "0x2002.0x63",
            "Hz",
            "disturbance_observer",
            "C02.62 disturbance observer low-pass cutoff frequency (manual 7.12)",
            30,
        ),
        plain_param(
            "disturbance_comp_torque",
            "C02.63",
            "0x2002.0x64",
            "0.1%",
            "disturbance_observer",
            "C02.63 disturbance observer compensation torque percentage (manual 7.12)",
            150,
        ),
    ]);
    params
}

const DEMO_MOTORS: [&str; 4] = ["motor_a", "motor_a1", "motor_b", "motor_b1"];

/// `notch_1_freq` deliberately disagrees on one motor so the demo exercises
/// the panel's per-motor grid highlight for drifted values.
const DEMO_DISAGREEING_C_CODE: &str = "C01.40";
const DEMO_DISAGREEING_MOTOR: &str = "motor_b";
const DEMO_DISAGREEING_VALUE: i64 = 400;

/// C-codes pinned in every demo motor's `[motor] params:` block — the
/// panel's cue that editing them live won't survive a restart until the
/// config is updated too. Currently none, but the demo keeps exercising
/// the mechanism end-to-end.
const DEMO_PINNED_C_CODES: [&str; 0] = [];

#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct DriveStateParam {
    name: String,
    c_code: String,
    addr: String,
    #[serde(rename = "type")]
    ty: &'static str,
    unit: &'static str,
    group: &'static str,
    description: String,
    options: Option<BTreeMap<String, String>>,
}

impl From<&DemoPanelParam> for DriveStateParam {
    fn from(p: &DemoPanelParam) -> Self {
        DriveStateParam {
            name: p.name.clone(),
            c_code: p.c_code.clone(),
            addr: p.addr.clone(),
            ty: "u16",
            unit: p.unit,
            group: p.group,
            description: p.description.clone(),
            options: p.options.map(|pairs| {
                pairs
                    .iter()
                    .map(|(v, label)| (v.to_string(), label.to_string()))
                    .collect()
            }),
        }
    }
}

/// Motor-space -> cartesian position map for the live spatial view, the
/// SERVO_DUMP_TUNING mirror of `servo_strokes.spatial_frame`: `axes` are
/// motor names (frame columns), each column folds the motor's invert sign
/// in, so `mode_pos[k] = sum(frame[k][s] * drive_frame_pos_mm[s])`.
#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct SpatialFrame {
    modes: Vec<String>,
    axes: Vec<String>,
    frame: Vec<Vec<f64>>,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct DriveStatePayload {
    version: i64,
    created_utc: String,
    params: Vec<DriveStateParam>,
    motors: BTreeMap<String, BTreeMap<String, i64>>,
    config_pins: BTreeMap<String, BTreeMap<String, i64>>,
    slots: BTreeMap<String, usize>,
    spatial: Option<SpatialFrame>,
}

/// The JSON Schema `handle_drive_state`'s response must satisfy: it adds one
/// field (`age_s`) on top of `DriveStatePayload`, which an unrestricted
/// `additionalProperties` schema still accepts.
pub fn drive_state_schema() -> schemars::Schema {
    schemars::schema_for!(DriveStatePayload)
}

/// The demo machine is CoreXY AWD — two motors per belt, none inverted —
/// so every column carries 1/(2 belts * 2 drives).
fn demo_spatial_frame() -> SpatialFrame {
    SpatialFrame {
        modes: vec!["x".to_string(), "y".to_string()],
        axes: DEMO_MOTORS.iter().map(|m| m.to_string()).collect(),
        frame: vec![vec![0.25, 0.25, 0.25, 0.25], vec![0.25, 0.25, -0.25, -0.25]],
    }
}

/// Write `<out_dir>/drive_state.json` in the shape `SERVO_DUMP_TUNING`
/// produces (`docs/rewrite/servo-tuning-profiles.md`), so the tuning panel
/// has something plausible to render without a bench.
fn write_demo_drive_state(out_dir: &Path) -> Result<(), String> {
    let panel = demo_panel_params();
    let readings: BTreeMap<String, i64> = panel
        .iter()
        .map(|p| (p.c_code.clone(), p.reading))
        .collect();
    let params: Vec<DriveStateParam> = panel.iter().map(DriveStateParam::from).collect();
    let motors: BTreeMap<String, BTreeMap<String, i64>> = DEMO_MOTORS
        .iter()
        .map(|m| {
            let mut motor_readings = readings.clone();
            if *m == DEMO_DISAGREEING_MOTOR {
                motor_readings.insert(DEMO_DISAGREEING_C_CODE.to_string(), DEMO_DISAGREEING_VALUE);
            }
            (m.to_string(), motor_readings)
        })
        .collect();
    let pinned: BTreeMap<String, i64> = DEMO_PINNED_C_CODES
        .iter()
        .map(|c| (c.to_string(), readings[*c]))
        .collect();
    let config_pins: BTreeMap<String, BTreeMap<String, i64>> = DEMO_MOTORS
        .iter()
        .map(|m| (m.to_string(), pinned.clone()))
        .collect();
    let slots: BTreeMap<String, usize> = DEMO_MOTORS
        .iter()
        .enumerate()
        .map(|(slot, m)| (m.to_string(), slot))
        .collect();
    let payload = DriveStatePayload {
        version: 1,
        created_utc: iso8601_utc(SystemTime::now()),
        params,
        motors,
        config_pins,
        slots,
        spatial: Some(demo_spatial_frame()),
    };
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

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoStrokePlan {
    start: f64,
    end: f64,
    speed: f64,
    accel: f64,
    iterations: i64,
    dwell_ms: i64,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoMotorSpec {
    name: &'static str,
    invert: bool,
    rotation_distance: f64,
    counts_per_mm: f64,
}

const DEMO_MOTOR_SPECS: [DemoMotorSpec; 4] = [
    DemoMotorSpec {
        name: "motor_a",
        invert: false,
        rotation_distance: 40.0,
        counts_per_mm: 3276.8,
    },
    DemoMotorSpec {
        name: "motor_a1",
        invert: false,
        rotation_distance: 40.0,
        counts_per_mm: 3276.8,
    },
    DemoMotorSpec {
        name: "motor_b",
        invert: false,
        rotation_distance: 40.0,
        counts_per_mm: 3276.8,
    },
    DemoMotorSpec {
        name: "motor_b1",
        invert: false,
        rotation_distance: 40.0,
        counts_per_mm: 3276.8,
    },
];

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoSweptGains {
    position: i64,
    speed: i64,
    integral: i64,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoStep {
    name: &'static str,
    swept: DemoSweptGains,
    applied: Vec<Applied>,
    capture: &'static str,
    accel: &'static str,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoParamWrite {
    servo: &'static str,
    addr: &'static str,
    value: i64,
    time_utc: String,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoAmbient {
    journal_params: BTreeMap<String, BTreeMap<String, i64>>,
    param_writes_since_last_run: Vec<DemoParamWrite>,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
struct DemoManifest {
    version: i64,
    experiment: &'static str,
    tag: String,
    created_utc: String,
    axis: &'static str,
    kinematics: &'static str,
    git_rev: &'static str,
    session_id: String,
    stroke_plan: DemoStrokePlan,
    motors: &'static [DemoMotorSpec],
    belts: &'static str,
    spatial: SpatialFrame,
    steps: Vec<DemoStep>,
    ambient: DemoAmbient,
}

fn demo_applied(addr: &'static str, value: i64) -> Vec<Applied> {
    vec![Applied {
        servo: "motor_a".to_string(),
        addr: addr.to_string(),
        ty: "u16".to_string(),
        value: serde_json::Value::from(value),
    }]
}

fn manifest_json(attempt: &DemoAttempt, created: SystemTime) -> DemoManifest {
    let created_utc = iso8601_utc(created);
    DemoManifest {
        version: 1,
        experiment: "gain_sweep",
        tag: format!("cal_{}", attempt.suffix),
        created_utc: created_utc.clone(),
        axis: "X",
        kinematics: "corexy",
        git_rev: "demo",
        session_id: format!("demo-{}", attempt.suffix),
        stroke_plan: DemoStrokePlan {
            start: 30.0,
            end: 220.0,
            speed: 100.0,
            accel: 3000.0,
            iterations: 1,
            dwell_ms: 700,
        },
        motors: &DEMO_MOTOR_SPECS,
        belts: "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1",
        spatial: demo_spatial_frame(),
        steps: vec![
            DemoStep {
                name: "s550",
                swept: DemoSweptGains {
                    position: 880,
                    speed: 550,
                    integral: 2273,
                },
                applied: demo_applied("0x2001.0x01", 880),
                capture: "step_s550.scap",
                accel: "step_s550_accel.csv",
            },
            DemoStep {
                name: "s700",
                swept: DemoSweptGains {
                    position: 1120,
                    speed: 700,
                    integral: 1786,
                },
                applied: demo_applied("0x2001.0x01", 1120),
                capture: "step_s700.scap",
                accel: "step_s700_accel.csv",
            },
        ],
        ambient: DemoAmbient {
            journal_params: BTreeMap::from([(
                "motor_a".to_string(),
                BTreeMap::from([("0x2001.0x31".to_string(), attempt.notch)]),
            )]),
            param_writes_since_last_run: vec![DemoParamWrite {
                servo: "motor_a",
                addr: "0x2001.0x31",
                value: attempt.notch,
                time_utc: created_utc,
            }],
        },
    }
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
