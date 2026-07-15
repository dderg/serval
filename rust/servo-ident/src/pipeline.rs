//! Shared multi-capture plumbing for the `fit` path of both CLIs: prep one
//! capture into its mode channels + keep mask, and pool several prepped
//! captures into one `FitInput` (mode fit) and the `SplitCapture` set (pair
//! load-share fit). The per-tool reporting stays in each binary.

use crate::capture::{steady_accel_keep, tracking_keep, Capture, PlateauOptions, TrackingOptions};
use crate::fit::FitInput;
use crate::model::Structure;
use crate::prep::{prep, PrepOptions, Prepped};
use crate::split::SplitCapture;

pub struct Prepared {
    pub cap: Capture,
    pub pp: Prepped,
    /// valid ∧ tracking ∧ steady-accel — the exact keep set the mode fit uses.
    pub keep: Vec<bool>,
}

pub struct PrepStats {
    pub total: usize,
    pub segments: usize,
    pub delay_s: f64,
    pub tracked: usize,
    pub kept: usize,
}

pub fn prepare(cap: Capture, structure: &Structure, opts: &PrepOptions) -> (Prepared, PrepStats) {
    let pp = prep(&cap, structure, opts);
    let total = cap.t.len();
    let track = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let plateau = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let keep: Vec<bool> = (0..total)
        .map(|k| pp.valid[k] && track[k] && plateau[k])
        .collect();
    let tracked = (0..total).filter(|&k| pp.valid[k] && track[k]).count();
    let kept = keep.iter().filter(|v| **v).count();
    let stats = PrepStats {
        total,
        segments: pp.segments,
        delay_s: pp.delay_s,
        tracked,
        kept,
    };
    (Prepared { cap, pp, keep }, stats)
}

pub fn pooled_input(structure: &Structure, prepared: &[Prepared]) -> FitInput {
    let n_modes = structure.mode_count();
    let n_motors = structure.axis_count();
    let e = prepared
        .first()
        .map_or(0, |p| p.pp.extra.first().map_or(0, Vec::len));
    let mut acc_mode = vec![Vec::new(); n_modes];
    let mut vel_mode = vec![Vec::new(); n_modes];
    let mut cs_mode = vec![Vec::new(); n_modes];
    let mut torque = vec![Vec::new(); n_motors];
    let mut extra: Vec<Vec<Vec<f64>>> = if e > 0 {
        vec![vec![Vec::new(); e]; n_motors]
    } else {
        Vec::new()
    };
    for pr in prepared {
        let idx: Vec<usize> = (0..pr.keep.len()).filter(|&k| pr.keep[k]).collect();
        for md in 0..n_modes {
            for &k in &idx {
                acc_mode[md].push(pr.pp.acc_mode[md][k]);
                vel_mode[md].push(pr.pp.vel_mode[md][k]);
                cs_mode[md].push(pr.pp.cs_mode[md][k]);
            }
        }
        for m in 0..n_motors {
            for &k in &idx {
                torque[m].push(pr.pp.torque[m][k]);
            }
        }
        if e > 0 {
            for m in 0..n_motors {
                for c in 0..e {
                    for &k in &idx {
                        extra[m][c].push(pr.pp.extra[m][c][k]);
                    }
                }
            }
        }
    }
    FitInput {
        structure: structure.clone(),
        acc_mode,
        vel_mode,
        cs_mode,
        torque,
        extra,
    }
}

pub fn split_captures(prepared: &[Prepared]) -> Vec<SplitCapture<'_>> {
    prepared
        .iter()
        .map(|pr| SplitCapture {
            cap: &pr.cap,
            torque_filt: &pr.pp.torque,
            keep: &pr.keep,
        })
        .collect()
}
