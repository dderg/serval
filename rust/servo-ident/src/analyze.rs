//! The `servo-cal analyze` pipeline: per-drive metrics + PSD peaks +
//! resonance, optional CoreXY combine and accelerometer PSD, a typed verdict,
//! and the `results.json` / `plot_series.json` writers plus the stdout table.

use std::collections::BTreeMap;
use std::path::Path;

use serde_json::Value;

use crate::combine::{compute_corexy_combine, peak_abs, rms};
use crate::frf::{
    active_slice, differential_series, find_modes, welch_frf, COHERENCE_MIN, DEFAULT_NPERSEG,
};
use crate::metrics::{
    compute_metrics, drive_series, motion_segments, DriveSeries, DEFAULT_SETTLE_BAND_COUNTS,
    DEFAULT_TORQUE_LIMIT_PER_MILLE,
};
use crate::psd::{moving_psd, top_peaks, welch_psd};
use crate::resonance::{detect_resonance, recommend_accel};
use crate::results::{
    AccelResult, Combined, DifferentialResult, DriveResult, Manifest, ManifestSpatial, PlotAccel,
    PlotCombined, PlotDifferential, PlotDrive, PlotPath, PlotPsd, PlotPsdAccel, PlotSeries,
    PlotStep, Results, Step, StepResult, Verdict,
};
use crate::ringdown::{
    compute_step_ringdown, ringdown_verdict_reason, RingdownOptions, DEFAULT_GUARD_MS,
    RINGDOWN_BAND_HZ, RINGDOWN_WINDOW_MARGIN_MS,
};
use crate::scap::Scap;

const MAX_SERIES_POINTS: usize = 2000;
pub const MAX_PATH_POINTS: usize = 4000;
const PSD_PEAK_COUNT: usize = 5;

fn stride_for(n: usize) -> usize {
    if n <= MAX_SERIES_POINTS {
        1
    } else {
        n.div_ceil(MAX_SERIES_POINTS)
    }
}

/// Decimation indices for a path polyline: an even stride over the capture
/// plus the final sample, so the stroke's endpoint survives.
pub fn path_indices(n: usize, max_points: usize) -> Vec<usize> {
    assert!(max_points >= 2, "path budget must fit both endpoints");
    if n == 0 {
        return Vec::new();
    }
    let stride = n.div_ceil(max_points).max(1);
    let mut idxs: Vec<usize> = (0..n).step_by(stride).collect();
    if *idxs.last().unwrap() != n - 1 {
        idxs.push(n - 1);
    }
    idxs
}

/// Commanded and actual toolhead XY in mm through the manifest's spatial
/// frame (motor counts -> cartesian, invert signs folded into the frame).
/// `Ok(None)` when the frame has no x+y modes or the capture lacks one of
/// the frame's motors — a single-axis capture, not an error.
pub fn xy_path(cap: &Scap, spatial: &ManifestSpatial) -> Result<Option<PlotPath>, String> {
    if spatial.frame.len() != spatial.modes.len()
        || spatial.frame.iter().any(|r| r.len() != spatial.axes.len())
    {
        return Err(format!(
            "manifest spatial frame shape does not match its {} mode(s) x {} axis(es)",
            spatial.modes.len(),
            spatial.axes.len()
        ));
    }
    let (Some(xi), Some(yi)) = (
        spatial.modes.iter().position(|m| m == "x"),
        spatial.modes.iter().position(|m| m == "y"),
    ) else {
        return Ok(None);
    };
    let mut motors = Vec::new();
    for (s, motor) in spatial.axes.iter().enumerate() {
        let Some(idx) = cap.drive_index(motor) else {
            return Ok(None);
        };
        let cpm = cap.header.drives[idx].counts_per_mm;
        if cpm <= 0.0 {
            return Err(format!("drive {motor:?} has counts_per_mm {cpm}"));
        }
        motors.push((s, idx, cpm));
    }
    let idxs = path_indices(cap.n_records, MAX_PATH_POINTS);
    let mut path = PlotPath {
        cmd_x_mm: vec![0.0; idxs.len()],
        cmd_y_mm: vec![0.0; idxs.len()],
        act_x_mm: vec![0.0; idxs.len()],
        act_y_mm: vec![0.0; idxs.len()],
    };
    for (s, idx, cpm) in motors {
        let target = cap.read_i64(idx, "target_counts")?;
        let actual = cap.read_i64(idx, "position_actual")?;
        let cx = spatial.frame[xi][s] / cpm;
        let cy = spatial.frame[yi][s] / cpm;
        for (j, &k) in idxs.iter().enumerate() {
            path.cmd_x_mm[j] += cx * target[k] as f64;
            path.cmd_y_mm[j] += cy * target[k] as f64;
            path.act_x_mm[j] += cx * actual[k] as f64;
            path.act_y_mm[j] += cy * actual[k] as f64;
        }
    }
    Ok(Some(path))
}

fn read_accel_csv(path: &Path) -> Result<(Vec<f64>, Vec<f64>), String> {
    let text =
        std::fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    let mut t = Vec::new();
    let mut mag = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let f: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse::<f64>())
            .collect::<Result<_, _>>()
            .map_err(|_| format!("{}: non-numeric accel row {line:?}", path.display()))?;
        if f.len() < 4 {
            return Err(format!(
                "{}: expected time,accel_x,accel_y,accel_z rows",
                path.display()
            ));
        }
        t.push(f[0]);
        mag.push((f[1] * f[1] + f[2] * f[2] + f[3] * f[3]).sqrt());
    }
    if t.len() < 2 {
        return Err(format!("{}: too few accel samples", path.display()));
    }
    Ok((t, mag))
}

struct DriveAnalysis {
    series: DriveSeries,
    result: DriveResult,
    freq_hz: Vec<f64>,
    psd: Vec<f64>,
}

fn analyze_drive(
    cap: &Scap,
    idx: usize,
    settle_band: i64,
    torque_limit: i64,
    fs: f64,
    ff_lead_samples: usize,
) -> Result<DriveAnalysis, String> {
    let series = drive_series(cap, idx)?;
    let metrics = compute_metrics(&series, settle_band, torque_limit, fs, ff_lead_samples)?;
    let segs = motion_segments(&series.flags);
    let (freq_hz, psd) = moving_psd(&series, &segs, fs)?;
    let psd_peaks = top_peaks(&freq_hz, &psd, PSD_PEAK_COUNT);
    let resonance = detect_resonance(&freq_hz, &psd);
    Ok(DriveAnalysis {
        series,
        result: DriveResult {
            metrics,
            psd_peaks,
            resonance,
        },
        freq_hz,
        psd,
    })
}

struct AccelAnalysis {
    result: AccelResult,
    t: Vec<f64>,
    mag: Vec<f64>,
    freq_hz: Vec<f64>,
    psd: Vec<f64>,
}

fn step_flags(drives: &BTreeMap<String, DriveResult>) -> Vec<String> {
    let mut flags = Vec::new();
    if drives.values().any(|d| d.resonance.detected) {
        flags.push("resonance_detected".to_string());
    }
    if drives.values().any(|d| d.metrics.torque.rail_detected) {
        flags.push("torque_saturated".to_string());
    }
    if drives
        .values()
        .any(|d| d.metrics.moves.iter().any(|m| m.settle_window_truncated))
    {
        flags.push("settle_window_truncated".to_string());
    }
    flags
}

/// Analyze one capture into a `StepResult` and a `PlotStep`.
pub fn analyze_capture(
    cap: &Scap,
    name: &str,
    settle_band: i64,
    torque_limit: i64,
    belts: Option<&str>,
    axis: Option<&str>,
    accel_path: Option<&Path>,
    ff_lead_samples: usize,
    spatial: Option<&ManifestSpatial>,
) -> Result<(StepResult, PlotStep), String> {
    let fs = cap.fs();
    let n = cap.n_records;
    let mut analyses: Vec<(String, DriveAnalysis)> = Vec::new();
    for (idx, dname) in cap.drive_names().into_iter().enumerate() {
        analyses.push((
            dname,
            analyze_drive(cap, idx, settle_band, torque_limit, fs, ff_lead_samples)?,
        ));
    }

    let combine = match belts {
        Some(spec) => Some(compute_corexy_combine(cap, spec, axis)?),
        None => None,
    };
    let combined = combine.as_ref().map(|c| {
        let on: Vec<f64> = (0..c.on_ferr.len())
            .filter(|&k| c.moving[k])
            .map(|k| c.on_ferr[k])
            .collect();
        let cross: Vec<f64> = (0..c.cross_ferr.len())
            .filter(|&k| c.moving[k])
            .map(|k| c.cross_ferr[k])
            .collect();
        Combined {
            on_ferr_peak_mm: peak_abs(&on),
            on_ferr_rms_mm: rms(&on),
            cross_ferr_peak_mm: peak_abs(&cross),
        }
    });

    let accel = match accel_path {
        Some(path) => {
            let (t, mag) = read_accel_csv(path)?;
            let afs = t.len() as f64 / (t[t.len() - 1] - t[0]);
            let (freq_hz, psd) = welch_psd(&mag, afs)?;
            if freq_hz.len() > MAX_SERIES_POINTS {
                return Err(format!(
                    "step {name:?}: accel psd has {} bins, over the {MAX_SERIES_POINTS} cap",
                    freq_hz.len()
                ));
            }
            Some(AccelAnalysis {
                result: AccelResult {
                    present: true,
                    psd_peaks: top_peaks(&freq_hz, &psd, PSD_PEAK_COUNT),
                },
                t,
                mag,
                freq_hz,
                psd,
            })
        }
        None => None,
    };

    let mut psd_freq_hz: Option<Vec<f64>> = None;
    let mut per_drive_psd: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for (dname, a) in &analyses {
        match &psd_freq_hz {
            Some(existing) if existing.len() != a.freq_hz.len() => {
                return Err(format!(
                    "step {name:?}: drive {dname:?} psd grid has {} bins, expected {} \
                     (drives must share the Welch grid)",
                    a.freq_hz.len(),
                    existing.len()
                ));
            }
            Some(_) => {}
            None => psd_freq_hz = Some(a.freq_hz.clone()),
        }
        per_drive_psd.insert(dname.clone(), a.psd.clone());
    }
    let psd_freq_hz = psd_freq_hz.ok_or_else(|| format!("step {name:?} has no drives"))?;
    if psd_freq_hz.len() > MAX_SERIES_POINTS {
        return Err(format!(
            "step {name:?}: following-error psd has {} bins, over the {MAX_SERIES_POINTS} cap",
            psd_freq_hz.len()
        ));
    }

    let mut drives = BTreeMap::new();
    let mut series_by_name: BTreeMap<String, DriveSeries> = BTreeMap::new();
    for (dname, a) in analyses {
        drives.insert(dname.clone(), a.result);
        series_by_name.insert(dname, a.series);
    }
    let flags = step_flags(&drives);

    let stride = stride_for(n);
    let idxs: Vec<usize> = (0..n).step_by(stride).collect();
    let t_s: Vec<f64> = idxs.iter().map(|&k| k as f64 / fs).collect();
    let first_flags = &series_by_name.values().next().unwrap().flags;
    let moving: Vec<(f64, f64)> = motion_segments(first_flags)
        .into_iter()
        .map(|(s, e)| (s as f64 / fs, e as f64 / fs))
        .collect();
    let mut plot_drives = BTreeMap::new();
    for (dname, s) in &series_by_name {
        plot_drives.insert(
            dname.clone(),
            PlotDrive {
                ferr_counts: idxs.iter().map(|&k| s.following_error[k]).collect(),
                torque_per_mille: idxs.iter().map(|&k| s.torque[k] as f64).collect(),
            },
        );
    }
    let plot_combined = combine.as_ref().map(|c| PlotCombined {
        on_ferr_mm: idxs.iter().map(|&k| c.on_ferr[k]).collect(),
        cross_ferr_mm: idxs.iter().map(|&k| c.cross_ferr[k]).collect(),
    });
    let plot_accel = accel.as_ref().map(|a| {
        let astride = stride_for(a.t.len());
        let ai: Vec<usize> = (0..a.t.len()).step_by(astride).collect();
        PlotAccel {
            t_s: ai.iter().map(|&k| a.t[k] - a.t[0]).collect(),
            magnitude: ai.iter().map(|&k| a.mag[k]).collect(),
        }
    });
    let plot_psd_accel = accel.as_ref().map(|a| PlotPsdAccel {
        freq_hz: a.freq_hz.clone(),
        psd: a.psd.clone(),
    });
    let path = match spatial {
        Some(sp) => xy_path(cap, sp)?,
        None => None,
    };

    let step_result = StepResult {
        name: name.to_string(),
        drives,
        combined,
        accel: accel.map(|a| a.result),
        differential: None,
        ringdown: None,
        flags,
    };
    let plot_step = PlotStep {
        name: name.to_string(),
        fs_hz: fs,
        stride,
        t_s,
        moving,
        drives: plot_drives,
        combined: plot_combined,
        accel: plot_accel,
        differential: None,
        ringdown: None,
        path,
        psd: PlotPsd {
            freq_hz: psd_freq_hz,
            per_drive: per_drive_psd,
            accel: plot_psd_accel,
        },
    };
    Ok((step_result, plot_step))
}

pub fn analyze_differential_capture(
    cap: &Scap,
    name: &str,
    freq_start: f64,
    freq_end: f64,
) -> Result<(StepResult, PlotStep), String> {
    let fs = cap.fs();
    let n = cap.n_records;
    let diff = differential_series(cap)?;
    let span = active_slice(&diff.cmd_mm)?;
    let cmd = &diff.cmd_mm[span.clone()];
    let frf = welch_frf(cmd, &diff.act_mm[span.clone()], fs, DEFAULT_NPERSEG)?;
    let torque_frf = welch_frf(cmd, &diff.torque[span.clone()], fs, DEFAULT_NPERSEG)?;
    let modes = find_modes(&frf, freq_start, freq_end)?;

    let display_lo = (freq_start * 0.5).max(frf.freqs[1]);
    let display_hi = freq_end * 1.2;
    let band: Vec<usize> = (0..frf.freqs.len())
        .filter(|&i| frf.freqs[i] >= display_lo && frf.freqs[i] <= display_hi)
        .collect();
    let bidx: Vec<usize> = band
        .iter()
        .copied()
        .step_by(stride_for(band.len()))
        .collect();
    let mag = frf.magnitude();
    let torque_mag = torque_frf.magnitude();
    let db = |v: f64| 20.0 * libm::log10(v.max(1e-12));
    let plot_differential = PlotDifferential {
        freq_hz: bidx.iter().map(|&i| frf.freqs[i]).collect(),
        mag_db: bidx.iter().map(|&i| db(mag[i])).collect(),
        phase_deg: bidx
            .iter()
            .map(|&i| libm::atan2(frf.im[i], frf.re[i]).to_degrees())
            .collect(),
        coherence: bidx.iter().map(|&i| frf.coherence[i]).collect(),
        torque_db: bidx.iter().map(|&i| db(torque_mag[i])).collect(),
        coherence_min: COHERENCE_MIN,
        band: (freq_start, freq_end),
        modes: modes.clone(),
    };

    let stride = stride_for(n);
    let idxs: Vec<usize> = (0..n).step_by(stride).collect();
    let t_s: Vec<f64> = idxs.iter().map(|&k| k as f64 / fs).collect();
    let moving = vec![(span.start as f64 / fs, span.end as f64 / fs)];
    let mut plot_drives = BTreeMap::new();
    let mut psd_freq_hz: Option<Vec<f64>> = None;
    let mut per_drive_psd: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for (idx, dname) in cap.drive_names().into_iter().enumerate() {
        let s = drive_series(cap, idx)?;
        let (freq_hz, psd) = welch_psd(&s.following_error[span.clone()], fs)?;
        match &psd_freq_hz {
            Some(existing) if existing.len() != freq_hz.len() => {
                return Err(format!(
                    "step {name:?}: drive {dname:?} psd grid has {} bins, expected {} \
                     (drives must share the Welch grid)",
                    freq_hz.len(),
                    existing.len()
                ));
            }
            Some(_) => {}
            None => psd_freq_hz = Some(freq_hz),
        }
        per_drive_psd.insert(dname.clone(), psd);
        plot_drives.insert(
            dname,
            PlotDrive {
                ferr_counts: idxs.iter().map(|&k| s.following_error[k]).collect(),
                torque_per_mille: idxs.iter().map(|&k| s.torque[k] as f64).collect(),
            },
        );
    }
    let psd_freq_hz = psd_freq_hz.ok_or_else(|| format!("step {name:?} has no drives"))?;
    if psd_freq_hz.len() > MAX_SERIES_POINTS {
        return Err(format!(
            "step {name:?}: following-error psd has {} bins, over the {MAX_SERIES_POINTS} cap",
            psd_freq_hz.len()
        ));
    }

    let step_result = StepResult {
        name: name.to_string(),
        drives: BTreeMap::new(),
        combined: None,
        accel: None,
        differential: Some(DifferentialResult {
            pair: diff.pair,
            segments: frf.segments,
            modes,
        }),
        ringdown: None,
        flags: Vec::new(),
    };
    let plot_step = PlotStep {
        name: name.to_string(),
        fs_hz: fs,
        stride,
        t_s,
        moving,
        drives: plot_drives,
        combined: None,
        accel: None,
        differential: Some(plot_differential),
        ringdown: None,
        path: None,
        psd: PlotPsd {
            freq_hz: psd_freq_hz,
            per_drive: per_drive_psd,
            accel: None,
        },
    };
    Ok((step_result, plot_step))
}

pub fn compute_verdict(
    experiment: &str,
    steps: &[StepResult],
    manifest: &[Step],
) -> Result<Verdict, String> {
    let has_flag = |sr: &StepResult, f: &str| sr.flags.iter().any(|x| x == f);
    let clean =
        |sr: &StepResult| !has_flag(sr, "resonance_detected") && !has_flag(sr, "torque_saturated");
    let find_step = |name: &str| manifest.iter().find(|m| m.name == name);

    match experiment {
        "gain_sweep" | "refine_sweep" => {
            let key = |m: &Step| -> Option<f64> {
                if experiment == "gain_sweep" {
                    m.swept_value("speed").or_else(|| m.swept_max())
                } else {
                    m.swept_max()
                }
            };
            let mut best: Option<(&StepResult, f64)> = None;
            for sr in steps {
                if !clean(sr) {
                    continue;
                }
                let mstep = find_step(&sr.name)
                    .ok_or_else(|| format!("step {:?} missing from manifest", sr.name))?;
                let Some(k) = key(mstep) else { continue };
                if best.map_or(true, |(_, bk)| k > bk) {
                    best = Some((sr, k));
                }
            }
            match best {
                Some((sr, _)) => {
                    let mstep = find_step(&sr.name).unwrap();
                    Ok(Verdict {
                        recommended_step: Some(sr.name.clone()),
                        reason: "highest gain step without resonance or torque rail".to_string(),
                        flags: Vec::new(),
                        apply: Some(mstep.applied.clone()),
                    })
                }
                None => Ok(Verdict {
                    recommended_step: None,
                    reason: "every step flags resonance or a torque rail".to_string(),
                    flags: Vec::new(),
                    apply: None,
                }),
            }
        }
        "accel_sweep" => {
            let mut accel_steps: Vec<(f64, bool)> = Vec::new();
            for sr in steps {
                let mstep = find_step(&sr.name)
                    .ok_or_else(|| format!("step {:?} missing from manifest", sr.name))?;
                let a = mstep
                    .swept_value("accel")
                    .or_else(|| mstep.swept_max())
                    .ok_or_else(|| format!("accel_sweep step {:?} has no swept accel", sr.name))?;
                accel_steps.push((a, has_flag(sr, "torque_saturated")));
            }
            let (chosen, note) = recommend_accel(&accel_steps);
            match chosen {
                Some(a) => {
                    let name = steps
                        .iter()
                        .zip(&accel_steps)
                        .find(|(_, (av, _))| *av == a)
                        .map(|(sr, _)| sr.name.clone());
                    let apply = name
                        .as_ref()
                        .and_then(|n| find_step(n))
                        .map(|m| m.applied.clone());
                    Ok(Verdict {
                        recommended_step: name,
                        reason: note,
                        flags: Vec::new(),
                        apply,
                    })
                }
                None => Ok(Verdict {
                    recommended_step: None,
                    reason: note,
                    flags: Vec::new(),
                    apply: None,
                }),
            }
        }
        "inertia_sweep" => Ok(Verdict {
            recommended_step: None,
            reason: "no automatic pick — read the overshoot trend across steps manually"
                .to_string(),
            flags: Vec::new(),
            apply: None,
        }),
        "dynamics_refine" => Ok(Verdict {
            recommended_step: None,
            reason: "scale pick is computed host-side by SERVO_REFINE_DYNAMICS".to_string(),
            flags: Vec::new(),
            apply: None,
        }),
        "differential" => {
            let first = steps
                .first()
                .ok_or_else(|| "differential run has no steps".to_string())?;
            let d = first.differential.as_ref().ok_or_else(|| {
                format!(
                    "differential step {:?} carries no differential result",
                    first.name
                )
            })?;
            let mode_lines: Vec<String> = d
                .modes
                .iter()
                .map(|m| {
                    let damping = m
                        .damping
                        .map(|z| format!("{z:.3}"))
                        .unwrap_or_else(|| "-".to_string());
                    format!(
                        "{:.1} Hz |H| {:.1} dB ζ {damping} coh {:.2}",
                        m.freq_hz, m.gain_db, m.coherence
                    )
                })
                .collect();
            Ok(Verdict {
                recommended_step: None,
                reason: format!("modes: {}", mode_lines.join("; ")),
                flags: Vec::new(),
                apply: None,
            })
        }
        "ringdown" => {
            let per_step: Vec<_> = steps
                .iter()
                .map(|sr| {
                    sr.ringdown.as_ref().ok_or_else(|| {
                        format!("ringdown step {:?} carries no ringdown result", sr.name)
                    })
                })
                .collect::<Result<_, _>>()?;
            Ok(Verdict {
                recommended_step: None,
                reason: ringdown_verdict_reason(&per_step),
                flags: Vec::new(),
                apply: None,
            })
        }
        "tracking" | "inertia_grid" => Ok(Verdict {
            recommended_step: None,
            reason: "not a sweep".to_string(),
            flags: Vec::new(),
            apply: None,
        }),
        other => Err(format!("unknown experiment {other:?}")),
    }
}

pub fn build_run(dir: &Path) -> Result<(Results, PlotSeries), String> {
    build_run_reusing(dir, BTreeMap::new())
}

/// Load prior `results.json` / `plot_series.json` as a per-step-name cache so
/// re-analyzing an append-only run only pays for the new steps. Sweeps that
/// analyze after every eval (SERVO_REFINE_DYNAMICS, gain sweeps) otherwise go
/// quadratic in run length — the growing between-eval stall starves the
/// motion stream. Only sound while the analyzer parameters are unchanged, so
/// it is opt-in (`servo-cal analyze --incremental`).
pub fn build_run_incremental(dir: &Path) -> Result<(Results, PlotSeries), String> {
    let mut cache = BTreeMap::new();
    let results_text = std::fs::read_to_string(dir.join("results.json"));
    let plot_text = std::fs::read_to_string(dir.join("plot_series.json"));
    if let (Ok(results_text), Ok(plot_text)) = (results_text, plot_text) {
        let prev_results: Results = serde_json::from_str(&results_text)
            .map_err(|e| format!("prior results.json parse: {e}"))?;
        let prev_plot: PlotSeries = serde_json::from_str(&plot_text)
            .map_err(|e| format!("prior plot_series.json parse: {e}"))?;
        if prev_results.steps.len() != prev_plot.steps.len() {
            return Err(format!(
                "prior results.json has {} steps but plot_series.json has {} — \
                 stale outputs, re-run a full analyze",
                prev_results.steps.len(),
                prev_plot.steps.len()
            ));
        }
        for (sr, ps) in prev_results.steps.into_iter().zip(prev_plot.steps) {
            if sr.name != ps.name {
                return Err(format!(
                    "prior results step {:?} does not match plot step {:?} — \
                     stale outputs, re-run a full analyze",
                    sr.name, ps.name
                ));
            }
            cache.insert(sr.name.clone(), (sr, ps, prev_results.fs_hz));
        }
    }
    build_run_reusing(dir, cache)
}

fn build_run_reusing(
    dir: &Path,
    mut cache: BTreeMap<String, (StepResult, PlotStep, f64)>,
) -> Result<(Results, PlotSeries), String> {
    let manifest_path = dir.join("manifest.json");
    let text = std::fs::read_to_string(&manifest_path)
        .map_err(|e| format!("read {}: {e}", manifest_path.display()))?;
    let manifest: Manifest =
        serde_json::from_str(&text).map_err(|e| format!("manifest parse: {e}"))?;
    if manifest.steps.is_empty() {
        return Err("manifest lists no steps".to_string());
    }
    let settle_band = DEFAULT_SETTLE_BAND_COUNTS;
    let torque_limit = DEFAULT_TORQUE_LIMIT_PER_MILLE;
    let plan_f64 = |key: &str| {
        manifest
            .stroke_plan
            .get(key)
            .and_then(Value::as_f64)
            .ok_or_else(|| {
                format!(
                    "{} manifest is missing stroke_plan.{key}",
                    manifest.experiment
                )
            })
    };
    let differential_band = if manifest.experiment == "differential" {
        Some((plan_f64("freq_start")?, plan_f64("freq_end")?))
    } else {
        None
    };
    let ringdown_plan = if manifest.experiment == "ringdown" {
        let dwell_ms = plan_f64("dwell_ms")?;
        let iterations = plan_f64("iterations")?;
        if dwell_ms <= RINGDOWN_WINDOW_MARGIN_MS {
            return Err(format!(
                "ringdown stroke_plan.dwell_ms {dwell_ms} leaves no window \
                 past the {RINGDOWN_WINDOW_MARGIN_MS} ms margin"
            ));
        }
        Some((
            RingdownOptions {
                guard_s: DEFAULT_GUARD_MS / 1000.0,
                window_s: (dwell_ms - RINGDOWN_WINDOW_MARGIN_MS) / 1000.0,
                band_hz: RINGDOWN_BAND_HZ,
            },
            (iterations as usize) * 2,
        ))
    } else {
        None
    };
    let mut step_results = Vec::new();
    let mut plot_steps = Vec::new();
    let mut fs_hz = 0.0;
    for step in &manifest.steps {
        if let Some((sr, ps, cached_fs)) = cache.remove(&step.name) {
            fs_hz = cached_fs;
            step_results.push(sr);
            plot_steps.push(ps);
            continue;
        }
        let cap = Scap::load(dir.join(&step.capture).to_str().unwrap())?;
        fs_hz = cap.fs();
        let (mut sr, mut ps) = match differential_band {
            Some((freq_start, freq_end)) => {
                analyze_differential_capture(&cap, &step.name, freq_start, freq_end)?
            }
            None => {
                let accel_path = step.accel.as_ref().map(|a| dir.join(a));
                analyze_capture(
                    &cap,
                    &step.name,
                    settle_band,
                    torque_limit,
                    manifest.belts.as_deref(),
                    manifest.axis.as_deref(),
                    accel_path.as_deref(),
                    manifest.ff_lead_cycles as usize,
                    manifest.spatial.as_ref(),
                )?
            }
        };
        if let Some((opts, expected_strokes)) = &ringdown_plan {
            let accel_path = step.accel.as_ref().map(|a| dir.join(a));
            let (rr, pr) = compute_step_ringdown(
                &cap,
                &step.name,
                manifest.belts.as_deref(),
                manifest.axis.as_deref(),
                accel_path.as_deref(),
                step.stops.as_deref(),
                *expected_strokes,
                opts,
            )?;
            if rr
                .sources
                .iter()
                .any(|s| s.modes.iter().any(|m| m.zeta < 0.0))
            {
                sr.flags.push("ringdown_growing_oscillation".to_string());
            }
            sr.ringdown = Some(rr);
            ps.ringdown = Some(pr);
        }
        step_results.push(sr);
        plot_steps.push(ps);
    }
    let verdict = compute_verdict(&manifest.experiment, &step_results, &manifest.steps)?;
    let results = Results {
        version: 1,
        fs_hz,
        settle_band_counts: settle_band,
        torque_limit_per_mille: torque_limit,
        steps: step_results,
        verdict,
    };
    let plot = PlotSeries {
        version: 1,
        steps: plot_steps,
    };
    Ok((results, plot))
}

/// Write `results.json` and `plot_series.json` for a run directory. Shared
/// by `analyze_run` (which also prints the table) and `serve`'s
/// analyze-on-demand endpoint (which stays quiet — the response body is the
/// reading surface there).
pub fn write_run_outputs(dir: &Path, results: &Results, plot: &PlotSeries) -> Result<(), String> {
    let results_json =
        serde_json::to_string_pretty(results).map_err(|e| format!("serialize results: {e}"))?;
    let plot_json =
        serde_json::to_string(plot).map_err(|e| format!("serialize plot_series: {e}"))?;
    std::fs::write(dir.join("results.json"), results_json)
        .map_err(|e| format!("write results.json: {e}"))?;
    std::fs::write(dir.join("plot_series.json"), plot_json)
        .map_err(|e| format!("write plot_series.json: {e}"))
}

pub fn analyze_run(dir: &Path, incremental: bool) -> Result<(), String> {
    let (results, plot) = if incremental {
        build_run_incremental(dir)?
    } else {
        build_run(dir)?
    };
    write_run_outputs(dir, &results, &plot)?;
    print_results(&results);
    Ok(())
}

fn print_step_drive(name: &str, dr: &DriveResult) {
    let m = &dr.metrics;
    println!("  {name}: {} samples, {} move(s)", m.samples, m.moves.len());
    let tq = &m.torque;
    if tq.rail_detected {
        println!(
            "    torque: peak {} per-mille ({:.0}% rated); rail {:.1}% of moving ({} samples, {:.0} ms; longest burst {:.0} ms)",
            tq.peak, tq.peak_pct_rated, tq.rail_pct_moving, tq.rail_samples, tq.rail_ms, tq.longest_burst_ms
        );
    } else {
        println!(
            "    torque: no rail (peak {} per-mille, {:.0}% rated)",
            tq.peak, tq.peak_pct_rated
        );
    }
    for mv in &m.moves {
        let settle = match mv.settle_ms {
            Some(s) => format!("{s:.1} ms"),
            None if mv.settle_window_truncated => "unknown (capture ended)".to_string(),
            None => "NEVER".to_string(),
        };
        println!(
            "    move {} [{:.1}..{:.1} ms]: ferr peak {:.0}, rms {:.1}, overshoot {:.0}, settle {settle}",
            mv.index, mv.start_ms, mv.end_ms, mv.ferr_peak, mv.ferr_rms, mv.overshoot
        );
    }
    println!(
        "    resonance: {} (ratio {:.1}, peak {:.1} Hz); psd peaks: {}",
        if dr.resonance.detected {
            "DETECTED"
        } else {
            "clear"
        },
        dr.resonance.ratio,
        dr.resonance.peak_hz,
        dr.psd_peaks
            .iter()
            .map(|(f, p)| format!("{f:.1}Hz={p:.2e}"))
            .collect::<Vec<_>>()
            .join(", ")
    );
}

pub fn print_step(sr: &StepResult) {
    println!("step: {}  flags: [{}]", sr.name, sr.flags.join(", "));
    for (name, dr) in &sr.drives {
        print_step_drive(name, dr);
    }
    if let Some(c) = &sr.combined {
        println!(
            "  combined: on-axis ferr peak {:.4} mm rms {:.4} mm, cross-axis peak {:.4} mm",
            c.on_ferr_peak_mm, c.on_ferr_rms_mm, c.cross_ferr_peak_mm
        );
    }
    if let Some(a) = &sr.accel {
        println!(
            "  accel psd peaks: {}",
            a.psd_peaks
                .iter()
                .map(|(f, p)| format!("{f:.1}Hz={p:.2e}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if let Some(r) = &sr.ringdown {
        println!(
            "  ringdown (guard {:.0} ms, window {:.0} ms):",
            r.guard_ms, r.window_ms
        );
        for src in &r.sources {
            let modes = if src.modes.is_empty() {
                "no ring above the noise floor".to_string()
            } else {
                src.modes
                    .iter()
                    .map(|m| {
                        format!(
                            "{:.1} Hz ζ {:.3} [{:.3}..{:.3}] {:.2} µm ({} tails, r² {:.2})",
                            m.freq_hz, m.zeta, m.zeta_lo, m.zeta_hi, m.disp_um, m.tails, m.r2
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("; ")
            };
            println!(
                "    {} [{}]: {} tails, noise {:.3}; {}",
                src.source, src.unit, src.tails, src.noise_floor, modes
            );
        }
    }
    if let Some(d) = &sr.differential {
        println!(
            "  differential modes ({}, {} Welch segments):",
            d.pair.join(" vs "),
            d.segments
        );
        println!("    freq      |H| peak      damping    coherence");
        for m in &d.modes {
            let damping = m
                .damping
                .map(|z| format!("{z:.4}"))
                .unwrap_or_else(|| "-".to_string());
            println!(
                "    {:7.1} Hz  {:6.2} dB     {:>8}     {:.2}",
                m.freq_hz, m.gain_db, damping, m.coherence
            );
        }
    }
}

pub fn print_results(results: &Results) {
    println!(
        "fs {:.0} Hz, settle band {} counts, torque limit {} per-mille",
        results.fs_hz, results.settle_band_counts, results.torque_limit_per_mille
    );
    for sr in &results.steps {
        print_step(sr);
    }
    let v = &results.verdict;
    match &v.recommended_step {
        Some(name) => println!("verdict: recommend {name} — {}", v.reason),
        None => println!("verdict: no pick — {}", v.reason),
    }
}

pub fn dump_csv(cap: &Scap, out: &Path) -> Result<(), String> {
    let n = cap.n_records;
    let fs = cap.fs();
    let names = cap.drive_names();
    let mut series = Vec::new();
    for (idx, _) in names.iter().enumerate() {
        series.push(drive_series(cap, idx)?);
    }
    let mut text = String::from("t_s");
    for name in &names {
        text.push_str(&format!(
            ",following_error_{name},torque_{name},target_{name},actual_{name}"
        ));
    }
    text.push('\n');
    for k in 0..n {
        text.push_str(&format!("{:.6}", k as f64 / fs));
        for s in &series {
            text.push_str(&format!(
                ",{},{},{},{}",
                s.following_error_i[k], s.torque[k], s.target[k], s.position_actual[k]
            ));
        }
        text.push('\n');
    }
    std::fs::write(out, text).map_err(|e| format!("write {}: {e}", out.display()))?;
    Ok(())
}
