//! Builds a `servo-ident` fit `Capture` directly from a `.scap` file,
//! replacing the Python `export_ident_csv` bridge. The fitter regresses
//! measured torque against the planner's commanded accel/velocity, so only
//! motion-active samples with the commanded-kinematics channels (capture
//! format v2) are usable — a v1 capture fails loudly.

use crate::capture::Capture;
use crate::scap::{Scap, FLAG_MOTION_ACTIVE};

/// Build the fit input from a capture. `axes` are drive names, in the order
/// the chosen `Structure` expects.
pub fn scap_to_capture(cap: &Scap, axes: &[&str]) -> Result<Capture, String> {
    if !cap.has_channel("accel_cmd") {
        return Err(
            "capture predates the commanded-kinematics channels (format v2); \
             re-capture before fitting dynamics"
                .to_string(),
        );
    }
    let cycle = cap.read_i64(0, "cycle_index")?;
    let flags = cap.read_i64(0, "flags")?;
    let n = cap.n_records;
    if n == 0 {
        return Err("capture contains no records".to_string());
    }
    let cyc0 = cycle[0];
    let cycle_ns = cap.header.cycle_ns as f64;
    let moving: Vec<bool> = flags.iter().map(|&f| f & FLAG_MOTION_ACTIVE != 0).collect();
    let keep: Vec<usize> = (0..n).filter(|&k| moving[k]).collect();
    if keep.is_empty() {
        return Err("capture has no motion-active samples to fit".to_string());
    }
    let t: Vec<f64> = keep
        .iter()
        .map(|&k| (cycle[k] - cyc0) as f64 * cycle_ns * 1e-9)
        .collect();

    let mut acc = Vec::with_capacity(axes.len());
    let mut vel = Vec::with_capacity(axes.len());
    let mut vel_act = Vec::with_capacity(axes.len());
    let mut torque = Vec::with_capacity(axes.len());
    for &name in axes {
        let idx = cap
            .drive_index(name)
            .ok_or_else(|| format!("axis {name:?} is not a drive in the capture"))?;
        let cpm = cap.header.drives[idx].counts_per_mm;
        let accel_cmd = cap.read_f64(idx, "accel_cmd")?;
        let vel_cmd = cap.read_f64(idx, "vel_cmd")?;
        let velocity_actual = cap.read_f64(idx, "velocity_actual")?;
        let torque_actual = cap.read_f64(idx, "torque_actual")?;
        acc.push(keep.iter().map(|&k| accel_cmd[k]).collect());
        vel.push(keep.iter().map(|&k| vel_cmd[k]).collect());
        vel_act.push(keep.iter().map(|&k| velocity_actual[k] / cpm).collect());
        torque.push(keep.iter().map(|&k| torque_actual[k]).collect());
    }
    Ok(Capture {
        t,
        acc,
        vel,
        vel_act,
        torque,
    })
}
