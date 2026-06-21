use core::sync::atomic::Ordering;

use portable_atomic::{AtomicU8, AtomicU32};

use crate::buzz_gen::ToneParams;
use crate::stepping_state::MAX_AXES;

/// Absolute sanity ceilings. These are NOT the physical safety policy (peak
/// acceleration / velocity clamping) — that lives on the host, which derives
/// amplitude from `accel_per_hz` and refuses anything violent. These bounds
/// only reject obviously-corrupt command arguments so a wire glitch cannot ask
/// the MCU to oscillate at kHz or by meters.
const MAX_FREQ_MILLIHZ: u32 = 2_000_000;
const MAX_AMPLITUDE_NM: u32 = 5_000_000;
// Up to a few minutes so a slow full-band sweep (e.g. 5->135 Hz at 1 Hz/s) fits
// in a single continuous chirp rather than being chunked.
const MAX_DURATION_MS: u32 = 300_000;

const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
const NM_PER_MM: f64 = 1.0e6;

/// Foreground-written, ISR-read excitation request. Only `seq` plus the scalar
/// parameters cross the task/ISR boundary, all as atomics (boundary rule B5).
/// `seq` is bumped last with `Release`; the ISR loads it first with `Acquire`,
/// so observing a new `seq` guarantees every parameter write is visible — a
/// seqlock-lite handshake with no torn reads. `amplitude_nm` is the displacement
/// at `freq_start`; `freq_start == freq_end` is a fixed-frequency buzz, otherwise
/// a linear chirp.
#[derive(Debug)]
pub struct BuzzControl {
    seq: AtomicU32,
    axis_mask: AtomicU8,
    sign_mask: AtomicU8,
    freq_start_millihz: AtomicU32,
    freq_end_millihz: AtomicU32,
    amplitude_nm: AtomicU32,
    duration_ms: AtomicU32,
    ramp_ms: AtomicU32,
}

impl BuzzControl {
    pub const fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            axis_mask: AtomicU8::new(0),
            sign_mask: AtomicU8::new(0),
            freq_start_millihz: AtomicU32::new(0),
            freq_end_millihz: AtomicU32::new(0),
            amplitude_nm: AtomicU32::new(0),
            duration_ms: AtomicU32::new(0),
            ramp_ms: AtomicU32::new(0),
        }
    }
}

impl Default for BuzzControl {
    fn default() -> Self {
        Self::new()
    }
}

/// One axis's resolved excitation curve plus the signed axis index, ready to
/// latch into a `buzz_stream` slot. `sign` folds the per-axis excitation sign;
/// the per-axis `base_mm`/`microstep_distance`/`anchor_cycle` are filled in by
/// the engine, which owns those, before arming the stream.
#[derive(Debug, Clone, Copy)]
pub struct AxisExcitation {
    pub axis_idx: usize,
    pub omega: f64,
    pub mu: f64,
    pub amplitude_mm: f64,
    pub sign: f64,
    pub total_seconds: f64,
    pub ramp_seconds: f64,
}

/// Engine-resident excitation arm. One excitation event at a time (a resonance
/// test drives a single cartesian axis, which maps to one or two motor axes that
/// must stay phase-coherent — they share one carrier phase by construction, the
/// same `omega`/`mu`/anchor). Holds only the foreground seqlock; the streaming
/// state lives per-axis in `buzz_stream`.
#[allow(missing_debug_implementations)]
pub struct Buzz {
    control: BuzzControl,
    last_seq: u32,
}

impl Buzz {
    pub const fn new() -> Self {
        Self {
            control: BuzzControl::new(),
            last_seq: 0,
        }
    }

    /// Foreground entry: stage a new excitation request. Writes parameters then
    /// bumps `seq` (Release) so a consistent set is observable. Returns 0 on
    /// success, -1 on out-of-range arguments (caller shuts down loudly).
    ///
    /// `amplitude_nm == 0` or `duration_ms == 0` is the disarm form: a fresh
    /// `seq` with zero work. Duration and ramp are physical milliseconds.
    #[allow(clippy::too_many_arguments)]
    pub fn arm(
        &self,
        num_axes: u8,
        axis_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
    ) -> i32 {
        let axis_bits = if num_axes >= 8 {
            0xFFu8
        } else {
            (1u8 << num_axes) - 1
        };
        let disarm = amplitude_nm == 0 || duration_ms == 0 || axis_mask == 0;
        if !disarm {
            if axis_mask & !axis_bits != 0 {
                return -1;
            }
            for f in [freq_start_millihz, freq_end_millihz] {
                if f == 0 || f > MAX_FREQ_MILLIHZ {
                    return -1;
                }
            }
            if amplitude_nm > MAX_AMPLITUDE_NM {
                return -1;
            }
            if duration_ms > MAX_DURATION_MS {
                return -1;
            }
        }
        let c = &self.control;
        c.axis_mask.store(axis_mask, Ordering::Relaxed);
        c.sign_mask.store(sign_mask, Ordering::Relaxed);
        c.freq_start_millihz
            .store(freq_start_millihz, Ordering::Relaxed);
        c.freq_end_millihz
            .store(freq_end_millihz, Ordering::Relaxed);
        c.amplitude_nm.store(amplitude_nm, Ordering::Relaxed);
        c.duration_ms.store(duration_ms, Ordering::Relaxed);
        c.ramp_ms.store(ramp_ms, Ordering::Relaxed);
        c.seq.fetch_add(1, Ordering::Release);
        0
    }

    /// True if `arm` published a request not yet consumed by `take_excitations`.
    #[inline]
    pub fn has_pending(&self) -> bool {
        self.control.seq.load(Ordering::Acquire) != self.last_seq
    }

    /// Consume the latest request and resolve it into per-axis excitation
    /// curves (absolute seconds, signed amplitude). Returns an empty list for
    /// the disarm form. The engine fills the per-axis base/microstep/anchor and
    /// arms the streams. Marks the request consumed so `has_pending` clears.
    pub fn take_excitations(&mut self) -> heapless::Vec<AxisExcitation, MAX_AXES> {
        let seq = self.control.seq.load(Ordering::Acquire);
        self.last_seq = seq;

        let c = &self.control;
        let axis_mask = c.axis_mask.load(Ordering::Relaxed);
        let sign_mask = c.sign_mask.load(Ordering::Relaxed);
        let freq_start_millihz = c.freq_start_millihz.load(Ordering::Relaxed);
        let freq_end_millihz = c.freq_end_millihz.load(Ordering::Relaxed);
        let amplitude_nm = c.amplitude_nm.load(Ordering::Relaxed);
        let duration_ms = c.duration_ms.load(Ordering::Relaxed);
        let ramp_ms = c.ramp_ms.load(Ordering::Relaxed);

        let mut out = heapless::Vec::new();
        if axis_mask == 0
            || amplitude_nm == 0
            || freq_start_millihz == 0
            || freq_end_millihz == 0
            || duration_ms == 0
        {
            return out;
        }

        let freq_start_hz = f64::from(freq_start_millihz) * 1.0e-3;
        let freq_end_hz = f64::from(freq_end_millihz) * 1.0e-3;
        let total_seconds = f64::from(duration_ms) * 1.0e-3;
        let omega = TWO_PI * freq_start_hz;
        let mu = TWO_PI * (freq_end_hz - freq_start_hz) / total_seconds;
        let amp_mag = f64::from(amplitude_nm) / NM_PER_MM;

        // Clamp the ramp so up- and down-ramps never overlap; a triangular
        // envelope (ramp == total/2) is the degenerate-but-valid floor.
        let ramp_seconds = (f64::from(ramp_ms) * 1.0e-3)
            .min(0.5 * total_seconds)
            .max(f64::MIN_POSITIVE);

        for i in 0..MAX_AXES {
            if axis_mask & (1 << i) == 0 {
                continue;
            }
            let sign = if sign_mask & (1 << i) != 0 { -1.0 } else { 1.0 };
            let _ = out.push(AxisExcitation {
                axis_idx: i,
                omega,
                mu,
                amplitude_mm: amp_mag,
                sign,
                total_seconds,
                ramp_seconds,
            });
        }
        out
    }
}

impl AxisExcitation {
    /// Complete the curve with the engine-owned per-axis base, microstep grid,
    /// MCU cycle rate, and the arm-time anchor cycle.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn into_params(
        self,
        base_mm: f64,
        microstep_distance: f64,
        cycles_per_second: f64,
        anchor_cycle: u32,
    ) -> ToneParams {
        // The arm-time curve derivation runs in f64 (millihz/nm wire values, the
        // chirp slope), then narrows to the solver's f32 hot-path numerics here.
        // `cycles_per_second` stays f64 for the per-crossing `cycle_at` promotion.
        ToneParams {
            omega: self.omega as f32,
            mu: self.mu as f32,
            amplitude_mm: self.amplitude_mm as f32,
            sign: self.sign as f32,
            base_mm: base_mm as f32,
            microstep_distance: microstep_distance as f32,
            anchor_cycle,
            cycles_per_second,
            total_seconds: self.total_seconds as f32,
            ramp_seconds: self.ramp_seconds as f32,
        }
    }
}

impl Default for Buzz {
    fn default() -> Self {
        Self::new()
    }
}

/// Continuous-time trapezoidal envelope in [0, 1], parameterized in seconds.
/// Zero at `t == 0` and `t == total`, unity across the flat top. This is the
/// canonical envelope; `buzz_gen::envelope` is the same shape used by the
/// crossing solver and its brute-force oracle.
#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn envelope(t: f64, total: f64, ramp: f64) -> f64 {
    f64::from(crate::buzz_gen::envelope(
        t as f32,
        total as f32,
        ramp as f32,
    ))
}

#[cfg(test)]
mod tests;
