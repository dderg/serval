// Update-stream generator for the phase-stepping (XDIRECT) buzz.
//
// Where the STEP/DIR exact-crossing buzz (`buzz_gen`) emits one incremental step
// per microstep crossing — an interrupt rate that scales 1:1 with microstepping
// and overruns the delivery path at 256 — this generator schedules *absolute
// position* updates and lets the leaf write coil currents directly. An XDIRECT
// write says "be at this position now", so a late write degrades to mild phase
// noise instead of a lost step, and microstepping drops out (the PHASE_LUT is the
// resolution).
//
// Updates land on a uniform CARRIER-PHASE grid: spacing `dphi = 2*pi/N`, so the
// realized update rate is exactly `N * f_inst(t)` and sweeps lock-step with the
// buzz frequency. Uniform-in-phase (hence uniform-in-time within a segment) keeps
// the zero-order-hold spectral images at clean harmonics of the tone instead of
// smearing them into the audible/measurement band — which a displacement grid,
// being uniform in space and so non-uniform in time, does not. `N` is divisible by
// 4 with the grid anchored at a turning point, so every quarter-phase (both
// extrema and both zero crossings) is a grid point: amplitude is pinned at the
// extrema with no special case, and the alignment survives an `N` change because
// `N` only ever changes at a turning point (velocity zero), where a spacing change
// costs no movement artefact.
//
// The emitted value is the signed axis offset in PHASE_LUT microsteps; the leaf
// adds it to the parked base and each motor's phase offset.

use crate::buzz_gen::{ToneError, ToneParams, amp_eff, cycle_at, omega_inst, position_rel};
use core::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;

/// Default target update rate (Hz) for a host-armed XDIRECT buzz; it bounds `N`
/// from above so the SPI/IRQ load stays capped at any microstep size. The realized
/// rate is the exact multiple `N * f_inst` at or just under this — never this value
/// itself, because a rate that does not divide the carrier period is what smears
/// the spectrum. Set near the motion sample rate (the cadence normal phase motion
/// already drives smoothly) and just under the step-output timer's re-arm cap.
pub const DEFAULT_XDIRECT_UPDATE_HZ: f32 = 10_000.0;

/// Static config for the update stream, alongside the shared `ToneParams`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectConfig {
    /// PHASE_LUT microstep size in mm (rotation_distance / (full_steps * 256)).
    pub lut_step_mm: f32,
    /// Upper bound on the realized update rate (Hz). Caps `N`; never the rate.
    pub target_rate_hz: f32,
}

impl XdirectConfig {
    #[must_use]
    pub fn new(lut_step_mm: f32, target_rate_hz: f32) -> Self {
        Self {
            lut_step_mm,
            target_rate_hz,
        }
    }
}

/// Resumable position in the update stream. The grid is anchored at `seg_phi0` (a
/// quarter-phase point) and advances by `2*pi/seg_n` until it reaches the segment's
/// closing turning point `seg_end`, where `seg_n` is recomputed for the next
/// half-cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectCursor {
    pub t: f32,
    seg_phi0: f32,
    seg_end: f32,
    seg_n: i32,
    k: i32,
    pub last_offset: i32,
}

impl XdirectCursor {
    #[must_use]
    pub fn start(p: &ToneParams, cfg: &XdirectConfig) -> Self {
        // Anchor at the opening zero crossing (phi = 0); the first turning point is
        // pi/2. With seg_n divisible by 4 the turning point is a grid point.
        Self {
            t: 0.0,
            seg_phi0: 0.0,
            seg_end: 0.5 * PI,
            seg_n: choose_n(p, cfg, 0.0),
            k: 0,
            last_offset: 0,
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

/// Invert `phi = omega*t + 0.5*mu*t^2` for the positive-frequency branch, `t >= 0`.
/// The `2*phi / (omega + omega_inst)` form avoids the small-`mu` cancellation of
/// the bare quadratic root and unifies the `mu == 0` (tone) case to `phi/omega`.
#[inline]
#[must_use]
fn time_at_phase(p: &ToneParams, phi: f32) -> f32 {
    let disc = (p.omega * p.omega + 2.0 * p.mu * phi).max(0.0);
    let root = libm::sqrtf(disc);
    2.0 * phi / (p.omega + root).max(f32::MIN_POSITIVE)
}

/// Per-cycle update count for the segment starting at time `t`: the exact multiple
/// of the instantaneous frequency that fits under `target_rate`, capped so the grid
/// is never finer than LUT resolution, floored at 4, and rounded up to a multiple
/// of 4 so the quarter-phases stay grid points.
#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn choose_n(p: &ToneParams, cfg: &XdirectConfig, t: f32) -> i32 {
    let f_inst = (omega_inst(p, t).abs() / TWO_PI).max(f32::MIN_POSITIVE);
    let n_rate = libm::roundf(cfg.target_rate_hz.max(1.0) / f_inst) as i32;
    let levels =
        (libm::roundf(amp_eff(p, t).abs() / cfg.lut_step_mm.max(f32::MIN_POSITIVE)) as i32).max(1);
    let n = n_rate.min(4 * levels).max(4);
    n + (4 - n % 4) % 4
}

#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn offset_at(p: &ToneParams, cfg: &XdirectConfig, t: f32) -> i32 {
    libm::roundf(position_rel(p, t) / cfg.lut_step_mm) as i32
}

/// Grid-step count from `seg_phi0` to `seg_end`. The span is `pi/2` (the opening
/// segment) or `pi` (a full half-cycle); both are integer multiples of `2*pi/seg_n`
/// because `seg_n` is divisible by 4.
#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn seg_steps(seg_phi0: f32, seg_end: f32, seg_n: i32) -> i32 {
    libm::roundf((seg_end - seg_phi0) * seg_n as f32 / TWO_PI) as i32
}

/// Next update strictly after `cursor`, or `Err(Done)` once the window closes.
///
/// One grid point per call: advance `k`, snap to the segment's turning point when
/// `k` reaches it (recomputing `seg_n` there), solve the time from the phase, and
/// emit. When the next grid time would pass `total_seconds`, a single net-zero
/// update parks the axis exactly on base.
pub fn next_update(
    p: &ToneParams,
    cfg: &XdirectConfig,
    cursor: XdirectCursor,
) -> Result<(XdirectUpdate, XdirectCursor), ToneError> {
    let total = p.total_seconds;
    if cursor.t >= total {
        return Err(ToneError::Done);
    }

    let steps = seg_steps(cursor.seg_phi0, cursor.seg_end, cursor.seg_n);
    let k1 = cursor.k + 1;
    let at_turn = k1 >= steps;
    #[allow(clippy::cast_precision_loss)]
    let phi = if at_turn {
        cursor.seg_end
    } else {
        cursor.seg_phi0 + k1 as f32 * (TWO_PI / cursor.seg_n as f32)
    };
    let t = time_at_phase(p, phi);

    if t >= total {
        // Net-zero close: the envelope reaches 0 at `total`, so land one update
        // exactly there to park on base. Off the phase grid by design; the top
        // guard (`cursor.t >= total`) makes it fire exactly once.
        let next = XdirectCursor {
            t: total,
            last_offset: 0,
            k: k1,
            ..cursor
        };
        return Ok((
            XdirectUpdate {
                cycle_abs: cycle_at(p, total),
                offset_steps: 0,
                t: total,
            },
            next,
        ));
    }
    if t <= cursor.t {
        return Err(ToneError::NonMonotonic);
    }

    let offset = offset_at(p, cfg, t);
    let next = if at_turn {
        // Turning points are exactly pi apart; advance by pi (not a floor-based
        // search) so an f32-exact turning phase never collapses the next segment.
        XdirectCursor {
            t,
            seg_phi0: phi,
            seg_end: phi + PI,
            seg_n: choose_n(p, cfg, t),
            k: 0,
            last_offset: offset,
        }
    } else {
        XdirectCursor {
            t,
            k: k1,
            last_offset: offset,
            ..cursor
        }
    };
    Ok((
        XdirectUpdate {
            cycle_abs: cycle_at(p, t),
            offset_steps: offset,
            t,
        },
        next,
    ))
}

#[cfg(test)]
mod tests;
