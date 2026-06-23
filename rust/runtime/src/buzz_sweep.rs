// Stepped-frequency buzz sweep for the STEP/DIR (pulse) path.
//
// A smooth analytic chirp (`mu != 0`) has no closed-form crossing inverse, so on
// a pulse axis every microstep crossing falls back to a numerical scan and the
// foreground starves (see `buzz_gen`). This generator delivers the same frequency
// band as a staircase of fixed-frequency tones instead — exactly the shape
// mainline's `VibrationPulseTestGenerator` plays — so every segment stays on the
// `mu = 0` closed-form path and never touches the scan.
//
// The sweep is decomposed into HALF-LOBE segments, each one zero crossing to the
// next (a half carrier period, `pi` of phase). Within a segment the carrier
// frequency is constant; the segment is a plain `mu = 0` tone whose displacement
// `sign * A * sin(omega * tau)` is zero at both ends by construction — so
// consecutive lobes abut at the parked base with no fade and no gap. The frequency
// steps up one notch per lobe (`f += hz_per_sec * half_period`, mainline's
// resolution), and the amplitude follows the same `f_start / f` taper the chirp
// used, so peak carrier velocity `A * omega` is held constant across the flat top.
// That makes the flat-top lobe boundaries continuous in BOTH position and velocity
// (only acceleration steps, which is harmless); the single trapezoidal envelope is
// sampled per-segment, giving small velocity steps only inside the short ramps.
//
// Each lobe's microstep crossings are produced by `buzz_gen::next_crossing` on a
// per-segment flat tone — the existing, tested closed-form solver — so the only new
// logic here is the lobe sequencing: pick the next frequency, amplitude, sign, and
// anchor, and roll over to the next lobe when the current one is exhausted.

use crate::buzz_gen::{
    ToneCrossing, ToneCursor, ToneError, ToneParams, cycle_at, envelope, next_crossing,
};

const TWO_PI: f32 = 2.0 * core::f32::consts::PI;

/// The sweep is parameterized by the same `ToneParams` the chirp uses: `omega` is
/// the start carrier, `mu` the chirp slope, and `mu / (2*pi)` is therefore the
/// sweep rate in Hz/s. `amplitude_mm` is the displacement at the start frequency;
/// every segment tapers it by `f_start / f`.
#[inline]
#[must_use]
fn start_hz(p: &ToneParams) -> f32 {
    p.omega / TWO_PI
}

#[inline]
#[must_use]
fn hz_per_sec(p: &ToneParams) -> f32 {
    p.mu / TWO_PI
}

#[inline]
#[must_use]
fn end_hz(p: &ToneParams) -> f32 {
    (p.omega + p.mu * p.total_seconds) / TWO_PI
}

/// Step the segment frequency by one half-period worth of sweep, clamped to the
/// band end so the tail runs at exactly `f_end` until the duration is spent.
#[inline]
#[must_use]
fn step_freq(p: &ToneParams, f: f32) -> f32 {
    let half_period = 0.5 / f;
    let next = f + hz_per_sec(p) * half_period;
    let end = end_hz(p);
    if hz_per_sec(p) >= 0.0 {
        next.min(end)
    } else {
        next.max(end)
    }
}

/// Resumable position in the sweep: which lobe (its frequency, start time, and
/// sign parity) and where inside that lobe's crossing stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepCursor {
    seg_f: f32,
    seg_t0: f32,
    flip: bool,
    inner: ToneCursor,
    done: bool,
}

impl SweepCursor {
    #[must_use]
    pub fn start(p: &ToneParams) -> Self {
        Self {
            seg_f: start_hz(p),
            seg_t0: 0.0,
            flip: false,
            inner: ToneCursor::start(),
            done: false,
        }
    }
}

/// The flat `mu = 0` tone for the lobe starting at `cursor`. Its local time runs
/// `[0, half_period]`; the anchor is shifted by the lobe start so `cycle_at` still
/// stamps absolute MCU cycles. Amplitude folds the `f_start / f` taper and the
/// whole-sweep envelope sampled at the lobe midpoint; the sign alternates per lobe
/// so the lobes string into a continuous carrier.
#[must_use]
fn lobe_params(p: &ToneParams, cursor: &SweepCursor) -> ToneParams {
    let f = cursor.seg_f;
    let omega = TWO_PI * f;
    let half_period = 0.5 / f;
    let env = envelope(
        cursor.seg_t0 + 0.5 * half_period,
        p.total_seconds,
        p.ramp_seconds,
    );
    let amplitude_mm = p.amplitude_mm * (start_hz(p) / f) * env;
    let sign = if cursor.flip { -p.sign } else { p.sign };
    ToneParams {
        omega,
        mu: 0.0,
        amplitude_mm,
        sign,
        base_mm: p.base_mm,
        microstep_distance: p.microstep_distance,
        anchor_cycle: cycle_at(p, cursor.seg_t0),
        cycles_per_second: p.cycles_per_second,
        total_seconds: half_period,
        ramp_seconds: f32::MIN_POSITIVE,
    }
}

/// Advance `cursor` to the next lobe, or mark the sweep done once the duration is
/// spent. The lobe boundary is a zero crossing, so the frequency/amplitude/sign
/// change here costs no position step.
#[must_use]
fn next_lobe(p: &ToneParams, cursor: SweepCursor) -> SweepCursor {
    let half_period = 0.5 / cursor.seg_f;
    let seg_t0 = cursor.seg_t0 + half_period;
    if seg_t0 >= p.total_seconds {
        return SweepCursor {
            done: true,
            ..cursor
        };
    }
    SweepCursor {
        seg_f: step_freq(p, cursor.seg_f),
        seg_t0,
        flip: !cursor.flip,
        inner: ToneCursor::start(),
        done: false,
    }
}

/// Next microstep edge of the sweep, strictly after `cursor`, with the advanced
/// cursor. Rolls over lobes internally: when the current lobe's tone is exhausted
/// it steps to the next frequency and keeps looking, so every returned crossing is
/// a real edge. `Err(Done)` once the whole band has played and parked on base.
pub fn next_crossing_sweep(
    p: &ToneParams,
    mut cursor: SweepCursor,
) -> Result<(ToneCrossing, SweepCursor), ToneError> {
    if cursor.done {
        return Err(ToneError::Done);
    }
    loop {
        let tp = lobe_params(p, &cursor);
        // A lobe whose tapered amplitude cannot reach the first half-microstep
        // never crosses a gridline; skip it without invoking the solver so a long
        // low-amplitude ramp tail cannot spin the segment loop.
        if tp.amplitude_mm < 0.5 * p.microstep_distance {
            let advanced = next_lobe(p, cursor);
            if advanced.done {
                return Err(ToneError::Done);
            }
            cursor = advanced;
            continue;
        }
        match next_crossing(&tp, cursor.inner) {
            Ok(c) => {
                let next = SweepCursor {
                    inner: ToneCursor {
                        level: c.level,
                        t_cursor: c.t,
                    },
                    ..cursor
                };
                let crossing = ToneCrossing {
                    cycle_abs: c.cycle_abs,
                    dir: c.dir,
                    t: cursor.seg_t0 + c.t,
                    level: c.level,
                };
                return Ok((crossing, next));
            }
            Err(ToneError::Done) => {
                let advanced = next_lobe(p, cursor);
                if advanced.done {
                    return Err(ToneError::Done);
                }
                cursor = advanced;
            }
            Err(e) => return Err(e),
        }
    }
}

#[cfg(test)]
mod tests;
