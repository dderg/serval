// Update-stream generator for the phase-stepping (XDIRECT) buzz.
//
// Where the STEP/DIR exact-crossing buzz (`buzz_gen`) emits one incremental step
// per microstep crossing — an interrupt rate that scales 1:1 with microstepping
// and overruns the delivery path at 256 — this generator instead schedules
// *absolute position* updates and lets the leaf write coil currents directly.
// An XDIRECT write says "be at this position now", so a late write degrades to
// mild phase noise instead of a lost step, and microstepping drops out (the
// PHASE_LUT is the resolution).
//
// Update instants are the union of:
//   * level changes on a chosen displacement grid `grid_mm` (≈ constant distance
//     between updates -> in the constant-`accel_per_hz` regime, constant update
//     rate across a whole sweep, because peak carrier velocity is constant), and
//   * carrier extrema (velocity zeros) — emitted exactly so the commanded
//     amplitude is pinned, not clipped to the nearest grid line.
//
// The emitted value is the signed axis offset in PHASE_LUT microsteps; the leaf
// adds it to the parked base and each motor's phase offset.

use crate::buzz_gen::{
    ToneError, ToneParams, cycle_at, position_rel, refine_extremum, velocity_rel,
};

/// Resumable position in the update stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectCursor {
    pub t: f32,
    pub last_offset: i32,
    pub v_prev: f32,
}

impl XdirectCursor {
    #[must_use]
    pub fn start(p: &ToneParams) -> Self {
        Self {
            t: 0.0,
            last_offset: 0,
            v_prev: velocity_rel(p, 0.0),
        }
    }
}

/// One scheduled XDIRECT update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectUpdate {
    pub cycle_abs: u32,
    /// Signed axis offset from the parked base, in PHASE_LUT microsteps.
    pub offset_steps: i32,
    /// Exact update time (absolute seconds from anchor); carried for tests/resume.
    pub t: f32,
}

/// Default target update rate (Hz) for a host-armed XDIRECT buzz; `grid_steps` is
/// derived from it so the SPI/IRQ load stays bounded across any microstep size.
/// Set near the motion sample rate: that is the cadence normal phase-stepping
/// motion already drives smoothly, and it stays just under the step-output timer's
/// re-arm cap. Lower rates leave the fast (near-zero-crossing) sections too coarse.
pub const DEFAULT_XDIRECT_UPDATE_HZ: f32 = 10000.0;

/// Static config for the update stream, alongside the shared `ToneParams`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectConfig {
    /// PHASE_LUT microstep size in mm (rotation_distance / (full_steps * 256)).
    pub lut_step_mm: f32,
    /// Emit a grid update once the offset has moved this many LUT microsteps from
    /// the last emitted one. `1` == one update per LUT microstep (finest);
    /// larger coarsens the rate. Extrema are emitted regardless.
    pub grid_steps: i32,
}

impl XdirectConfig {
    /// Derive `grid_steps` so the peak update rate ≈ `target_rate_hz`. Peak carrier
    /// velocity `A*omega` is constant across a constant-`accel_per_hz` sweep, so a
    /// fixed grid span gives a flat rate; this just inverts `rate = v_peak / (grid *
    /// lut_step)` for `grid`.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn for_rate(lut_step_mm: f32, amplitude_mm: f32, omega: f32, target_rate_hz: f32) -> Self {
        let v_peak = (amplitude_mm * omega).abs().max(f32::MIN_POSITIVE);
        let raw = v_peak / (target_rate_hz.max(1.0) * lut_step_mm.max(f32::MIN_POSITIVE));
        let grid_steps = (libm::roundf(raw) as i32).max(1);
        Self {
            lut_step_mm,
            grid_steps,
        }
    }
}

#[inline]
#[must_use]
fn offset_at(p: &ToneParams, cfg: &XdirectConfig, t: f32) -> i32 {
    #[allow(clippy::cast_possible_truncation)]
    let o = libm::roundf(position_rel(p, t) / cfg.lut_step_mm) as i32;
    o
}

/// Fine forward-scan step: small enough that the offset moves at most ~one grid
/// span per step (so no grid line is skipped near the fast zero-crossings) and a
/// fraction of the fastest carrier half-period (so two samples never straddle
/// more than one extremum), capped by the remaining window.
#[inline]
#[must_use]
fn scan_dt(p: &ToneParams, cfg: &XdirectConfig, after: f32) -> f32 {
    let omega_hi = (p.omega.abs() + p.mu.abs() * p.total_seconds).max(f32::MIN_POSITIVE);
    let half_period = core::f32::consts::PI / omega_hi;
    // Bound on |dq/dt|: carrier term `A*omega_inst` plus the envelope-ramp slope
    // `A/ramp` (only nonzero on the ramps, but bounding it everywhere is safe).
    let ramp = p.ramp_seconds.max(f32::MIN_POSITIVE);
    let v_max = (p.amplitude_mm.abs() * (omega_hi + 1.0 / ramp)).max(f32::MIN_POSITIVE);
    // Target ~half a grid span per step so the emit never overshoots a full span.
    let grid_traverse = 0.5 * (cfg.grid_steps.max(1) as f32) * cfg.lut_step_mm / v_max;
    let remaining = (p.total_seconds - after).max(f32::MIN_POSITIVE);
    (half_period / 8.0)
        .min(grid_traverse)
        .min(remaining)
        .max(1.0e-7)
}

/// Next update strictly after `cursor`, or `Err(Done)` once the window closes.
///
/// Forward progress is structural: `t` advances by at least `scan_dt` each
/// iteration (an extremum split lands strictly inside the step), so the loop is
/// bounded by `remaining / dt`.
pub fn next_update(
    p: &ToneParams,
    cfg: &XdirectConfig,
    cursor: XdirectCursor,
) -> Result<(XdirectUpdate, XdirectCursor), ToneError> {
    let mut t = cursor.t;
    let mut v_prev = cursor.v_prev;
    let total = p.total_seconds;

    loop {
        if t >= total {
            return Err(ToneError::Done);
        }
        let dt = scan_dt(p, cfg, t);
        let sample = (t + dt).min(total);
        let v_sample = velocity_rel(p, sample);

        // A carrier extremum strictly inside (t, sample): emit it exactly so the
        // peak amplitude is pinned, not rounded to a grid line.
        if v_prev != 0.0 && v_sample != 0.0 && v_prev * v_sample < 0.0 {
            if let Ok(ext) = refine_extremum(p, t, sample) {
                if ext > t && ext < sample {
                    let offset = offset_at(p, cfg, ext);
                    if offset != cursor.last_offset {
                        let next = XdirectCursor {
                            t: ext,
                            last_offset: offset,
                            v_prev: velocity_rel(p, ext),
                        };
                        return Ok((
                            XdirectUpdate {
                                cycle_abs: cycle_at(p, ext),
                                offset_steps: offset,
                                t: ext,
                            },
                            next,
                        ));
                    }
                    t = ext;
                    v_prev = velocity_rel(p, ext);
                    continue;
                }
            }
        }

        let offset = offset_at(p, cfg, sample);
        let at_end = sample >= total;
        // Grid line crossed, OR the closing sample: the envelope returns to base
        // (offset 0) at `total`, and that net-zero update must fire even when it is
        // within `grid_steps` of the last one, so the motor parks exactly on base.
        let emit = (offset - cursor.last_offset).abs() >= cfg.grid_steps
            || (at_end && offset != cursor.last_offset);
        if emit {
            let next = XdirectCursor {
                t: sample,
                last_offset: offset,
                v_prev: v_sample,
            };
            return Ok((
                XdirectUpdate {
                    cycle_abs: cycle_at(p, sample),
                    offset_steps: offset,
                    t: sample,
                },
                next,
            ));
        }
        if at_end {
            return Err(ToneError::Done);
        }

        t = sample;
        v_prev = v_sample;
    }
}

#[cfg(test)]
mod tests;
