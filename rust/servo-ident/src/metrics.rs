//! Per-drive tracking metrics ported from `scripts/servo_capture.py`
//! (`compute_metrics`, `torque_summary`, settle/overshoot detection and the
//! motion-segment helpers). Algorithmic parity with the Python is the
//! contract these functions must hold; the golden parity test is the gate.

use serde::Serialize;

use crate::scap::{Scap, FLAG_MOTION_ACTIVE};

pub const SETTLE_HOLD_MS: f64 = 50.0;
pub const DEFAULT_SETTLE_BAND_COUNTS: i64 = 50;
pub const DEFAULT_TORQUE_LIMIT_PER_MILLE: i64 = 1400;

#[derive(Debug, Serialize)]
pub struct Move {
    #[serde(rename = "move")]
    pub index: usize,
    pub start_ms: f64,
    pub end_ms: f64,
    pub ferr_peak: f64,
    pub ferr_rms: f64,
    pub overshoot: f64,
    pub settle_ms: Option<f64>,
    pub settle_window_truncated: bool,
}

#[derive(Debug, Serialize)]
pub struct TorqueSummary {
    pub peak: i64,
    pub peak_pct_rated: f64,
    pub moving_samples: usize,
    pub rail_detected: bool,
    pub rail_level: i64,
    pub rail_samples: usize,
    pub rail_pct_moving: f64,
    pub rail_ms: f64,
    pub longest_burst_ms: f64,
}

#[derive(Debug, Serialize)]
pub struct Metrics {
    pub samples: usize,
    pub moves: Vec<Move>,
    pub torque_saturation_pct: f64,
    pub torque: TorqueSummary,
    pub ferr_crosscheck_max: i64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ff_velocity_offset_max: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ff_torque_offset_max: Option<i64>,
}

pub struct DriveSeries {
    pub following_error: Vec<f64>,
    pub following_error_i: Vec<i64>,
    pub target: Vec<i64>,
    pub position_actual: Vec<i64>,
    pub torque: Vec<i64>,
    pub flags: Vec<i64>,
    pub velocity_offset: Option<Vec<i64>>,
    pub torque_offset: Option<Vec<i64>>,
}

pub fn drive_series(cap: &Scap, idx: usize) -> Result<DriveSeries, String> {
    let following_error_i = cap.read_i64(idx, "following_error")?;
    let following_error = following_error_i.iter().map(|&v| v as f64).collect();
    Ok(DriveSeries {
        following_error,
        following_error_i,
        target: cap.read_i64(idx, "target_counts")?,
        position_actual: cap.read_i64(idx, "position_actual")?,
        torque: cap.read_i64(idx, "torque_actual")?,
        flags: cap.read_i64(idx, "flags")?,
        velocity_offset: if cap.has_channel("velocity_offset") {
            Some(cap.read_i64(idx, "velocity_offset")?)
        } else {
            None
        },
        torque_offset: if cap.has_channel("torque_offset") {
            Some(cap.read_i64(idx, "torque_offset")?)
        } else {
            None
        },
    })
}

pub fn motion_segments(flags: &[i64]) -> Vec<(usize, usize)> {
    if flags.is_empty() {
        return Vec::new();
    }
    let moving: Vec<bool> = flags.iter().map(|&f| f & FLAG_MOTION_ACTIVE != 0).collect();
    segments_from_mask(&moving)
}

fn segments_from_mask(moving: &[bool]) -> Vec<(usize, usize)> {
    let n = moving.len();
    if n == 0 {
        return Vec::new();
    }
    let mut bounds = vec![0usize];
    for k in 1..n {
        if moving[k] != moving[k - 1] {
            bounds.push(k);
        }
    }
    bounds.push(n);
    let mut out = Vec::new();
    for i in 0..bounds.len() - 1 {
        if moving[bounds[i]] {
            out.push((bounds[i], bounds[i + 1]));
        }
    }
    out
}

pub fn target_motion_segments(target: &[i64], fs: f64) -> Vec<(usize, usize)> {
    let n = target.len();
    if n == 0 {
        return Vec::new();
    }
    let mut moving = vec![false; n];
    for k in 1..n {
        moving[k] = target[k] - target[k - 1] != 0;
    }
    let close = (0.02 * fs).round_ties_even() as usize;
    let stationary = segments_from_mask(&moving.iter().map(|&m| !m).collect::<Vec<_>>());
    for (s, e) in stationary {
        if e - s <= close && s > 0 && e < n {
            for m in moving.iter_mut().take(e).skip(s) {
                *m = true;
            }
        }
    }
    segments_from_mask(&moving)
        .into_iter()
        .filter(|&(s, e)| e - s > close)
        .collect()
}

fn settle_index(err: &[f64], band: f64, hold: usize) -> Option<usize> {
    if err.len() < hold {
        return None;
    }
    let mut run = 0usize;
    for (j, &v) in err.iter().enumerate() {
        if v.abs() <= band {
            run += 1;
            if run >= hold {
                return Some(j + 1 - hold);
            }
        } else {
            run = 0;
        }
    }
    None
}

fn longest_true_run(mask: &[bool]) -> usize {
    let mut best = 0usize;
    let mut cur = 0usize;
    for &m in mask {
        if m {
            cur += 1;
            best = best.max(cur);
        } else {
            cur = 0;
        }
    }
    best
}

pub fn torque_summary(d: &DriveSeries, torque_limit: i64, fs: f64) -> TorqueSummary {
    let torque: Vec<i64> = d.torque.iter().map(|&t| t.abs()).collect();
    let ms_per_sample = 1000.0 / fs;
    let moving: Vec<bool> = d
        .flags
        .iter()
        .map(|&f| f & FLAG_MOTION_ACTIVE != 0)
        .collect();
    let moving_n = moving.iter().filter(|&&m| m).count();
    let peak = torque.iter().copied().max().unwrap_or(0);
    let mut s = TorqueSummary {
        peak,
        peak_pct_rated: peak as f64 / 10.0,
        moving_samples: moving_n,
        rail_detected: false,
        rail_level: torque_limit,
        rail_samples: 0,
        rail_pct_moving: 0.0,
        rail_ms: 0.0,
        longest_burst_ms: 0.0,
    };
    if peak < torque_limit || moving_n == 0 {
        return s;
    }
    let on_rail: Vec<bool> = (0..torque.len())
        .map(|k| torque[k] >= torque_limit && moving[k])
        .collect();
    let rail_samples = on_rail.iter().filter(|&&r| r).count();
    s.rail_detected = true;
    s.rail_samples = rail_samples;
    s.rail_pct_moving = 100.0 * rail_samples as f64 / moving_n as f64;
    s.rail_ms = rail_samples as f64 * ms_per_sample;
    s.longest_burst_ms = longest_true_run(&on_rail) as f64 * ms_per_sample;
    s
}

pub fn compute_metrics(
    d: &DriveSeries,
    settle_band: i64,
    torque_limit: i64,
    fs: f64,
) -> Result<Metrics, String> {
    let n = d.following_error.len();
    if n == 0 {
        return Err("capture contains no records".to_string());
    }
    let ms_per_sample = 1000.0 / fs;
    let hold = (SETTLE_HOLD_MS * fs / 1000.0).round_ties_even() as usize;
    let ferr = &d.following_error;
    let band = settle_band as f64;
    let segs = target_motion_segments(&d.target, fs);
    let mut moves = Vec::with_capacity(segs.len());
    for (idx, &(s, e)) in segs.iter().enumerate() {
        let move_err = &ferr[s..e];
        let post_end = if idx + 1 < segs.len() {
            segs[idx + 1].0
        } else {
            n
        };
        let post = &ferr[e..post_end];
        let settle_sample = settle_index(post, band, hold);
        let overshoot_end = settle_sample.unwrap_or(post.len());
        let settle_ms = settle_sample.map(|x| x as f64 * ms_per_sample);
        let ferr_peak = move_err.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
        let ferr_rms =
            (move_err.iter().map(|&v| v * v).sum::<f64>() / move_err.len() as f64).sqrt();
        let overshoot = if overshoot_end > 0 {
            post[..overshoot_end]
                .iter()
                .fold(0.0_f64, |m, &v| m.max(v.abs()))
        } else {
            0.0
        };
        moves.push(Move {
            index: idx,
            start_ms: s as f64 * ms_per_sample,
            end_ms: e as f64 * ms_per_sample,
            ferr_peak,
            ferr_rms,
            overshoot,
            settle_ms,
            settle_window_truncated: settle_sample.is_none() && post.len() < hold,
        });
    }
    let torque_abs: Vec<i64> = d.torque.iter().map(|&t| t.abs()).collect();
    let saturated = torque_abs.iter().filter(|&&t| t >= torque_limit).count();
    let torque_saturation_pct = 100.0 * saturated as f64 / n.max(1) as f64;
    let ferr_crosscheck_max = (0..n)
        .map(|k| (d.target[k] - d.position_actual[k] - d.following_error_i[k]).abs())
        .max()
        .unwrap_or(0);
    let mut metrics = Metrics {
        samples: n,
        moves,
        torque_saturation_pct,
        torque: torque_summary(d, torque_limit, fs),
        ferr_crosscheck_max,
        ff_velocity_offset_max: None,
        ff_torque_offset_max: None,
    };
    if let Some(vel_off) = &d.velocity_offset {
        let moving: Vec<bool> = d
            .flags
            .iter()
            .map(|&f| f & FLAG_MOTION_ACTIVE != 0)
            .collect();
        let max_moving = |series: &[i64]| -> i64 {
            (0..n)
                .filter(|&k| moving[k])
                .map(|k| series[k].abs())
                .max()
                .unwrap_or(0)
        };
        metrics.ff_velocity_offset_max = Some(max_moving(vel_off));
        metrics.ff_torque_offset_max =
            Some(d.torque_offset.as_ref().map(|t| max_moving(t)).unwrap_or(0));
    }
    Ok(metrics)
}
