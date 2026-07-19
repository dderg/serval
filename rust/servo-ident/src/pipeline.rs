//! Shared capture plumbing for the `fit` path of both CLIs: prep the capture
//! into its mode channels + keep mask, and reduce it to the `FitInput` the
//! fit consumes. The per-tool reporting stays in each binary.

use crate::capture::{steady_accel_keep, tracking_keep, Capture, PlateauOptions, TrackingOptions};
use crate::fit::FitInput;
use crate::model::Structure;
use crate::prep::{prep, PrepOptions, Prepped};

pub struct Prepared {
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

pub fn prepare(cap: &Capture, structure: &Structure, opts: &PrepOptions) -> (Prepared, PrepStats) {
    let pp = prep(cap, structure, opts);
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
    (Prepared { pp, keep }, stats)
}

pub fn fit_input(structure: &Structure, prepared: &Prepared) -> FitInput {
    let idx: Vec<usize> = (0..prepared.keep.len())
        .filter(|&k| prepared.keep[k])
        .collect();
    let select = |chan: &[f64]| -> Vec<f64> { idx.iter().map(|&k| chan[k]).collect() };
    FitInput {
        structure: structure.clone(),
        acc_mode: prepared.pp.acc_mode.iter().map(|c| select(c)).collect(),
        vel_mode: prepared.pp.vel_mode.iter().map(|c| select(c)).collect(),
        cs_mode: prepared.pp.cs_mode.iter().map(|c| select(c)).collect(),
        snap_mode: prepared.pp.snap_mode.iter().map(|c| select(c)).collect(),
        torque: prepared.pp.torque.iter().map(|c| select(c)).collect(),
        extra: prepared
            .pp
            .extra
            .iter()
            .map(|per_motor| per_motor.iter().map(|c| select(c)).collect())
            .collect(),
    }
}

pub fn full_fit_input(structure: &Structure, prepared: &Prepared) -> FitInput {
    FitInput {
        structure: structure.clone(),
        acc_mode: prepared.pp.acc_mode.clone(),
        vel_mode: prepared.pp.vel_mode.clone(),
        cs_mode: prepared.pp.cs_mode.clone(),
        snap_mode: prepared.pp.snap_mode.clone(),
        torque: prepared.pp.torque.clone(),
        extra: prepared.pp.extra.clone(),
    }
}
