use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

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

const TWO_PI: f32 = 2.0 * core::f32::consts::PI;
const NM_PER_MM: f32 = 1.0e6;

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

/// ISR-private latched parameters. Recomputed only when a new `seq` is observed.
/// A linear chirp: the per-tick phase increment slews from `omega_tick_start` to
/// `omega_tick_end` across `total_ticks`. `amp_mm` is the displacement amplitude
/// at `freq_start`; the running amplitude is scaled by `f_start / f(t)` so peak
/// velocity stays constant and peak acceleration grows only linearly with
/// frequency (the constant-`accel_per_hz` regime). A fixed-frequency buzz is the
/// degenerate case `omega_tick_start == omega_tick_end`.
#[derive(Debug, Clone, Copy)]
struct BuzzParams {
    omega_tick_start: f32,
    omega_tick_end: f32,
    omega_sec_start: f32,
    amp_mm: [f32; MAX_AXES],
    total_ticks: u32,
    ramp_ticks: u32,
}

impl BuzzParams {
    const fn idle() -> Self {
        Self {
            omega_tick_start: 0.0,
            omega_tick_end: 0.0,
            omega_sec_start: 0.0,
            amp_mm: [0.0; MAX_AXES],
            total_ticks: 0,
            ramp_ticks: 0,
        }
    }
}

/// Per-axis additive contribution a buzz makes to one tick's dispatch inputs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BuzzSample {
    /// Added to `p_end` (end-of-sample target position, mm).
    pub offset: f32,
    /// Added to `p_sample_start` (start-of-sample position, mm).
    pub sample_start_offset: f32,
    /// Added to `v_end` (end-of-sample velocity, mm/s).
    pub velocity: f32,
}

impl BuzzSample {
    pub const ZERO: Self = Self {
        offset: 0.0,
        sample_start_offset: 0.0,
        velocity: 0.0,
    };
}

/// Engine-resident excitation generator. One excitation event at a time (a
/// resonance test drives a single cartesian axis, which maps to one or two
/// motor axes that must stay phase-coherent — so they share this one phase
/// accumulator rather than arming independently).
#[allow(missing_debug_implementations)]
pub struct Buzz {
    control: BuzzControl,
    params: BuzzParams,
    last_seq: u32,
    active: bool,
    phase: f32,
    tick: u32,
    prev_offset: [f32; MAX_AXES],
}

impl Buzz {
    pub const fn new() -> Self {
        Self {
            control: BuzzControl::new(),
            params: BuzzParams::idle(),
            last_seq: 0,
            active: false,
            phase: 0.0,
            tick: 0,
            prev_offset: [0.0; MAX_AXES],
        }
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.active
    }

    #[inline]
    pub fn affects_axis(&self, axis_idx: usize) -> bool {
        self.active && self.params.amp_mm.get(axis_idx).is_some_and(|a| *a != 0.0)
    }

    /// Foreground entry: stage a new excitation request. Writes parameters then
    /// bumps `seq` (Release) so the ISR latches a consistent set. Returns 0 on
    /// success, -1 on out-of-range arguments (caller shuts down loudly).
    ///
    /// `amplitude_nm == 0` or `duration_ms == 0` is the disarm form: a fresh
    /// `seq` with zero work, which the ISR latches as "inactive". Duration and
    /// ramp are physical milliseconds; the ISR converts them to sample ticks
    /// using its own clock, so the host never needs to know the tick rate.
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

    /// ISR entry, once per tick before the per-axis dispatch loop. Latches a new
    /// request if `seq` changed, then advances the phase/tick state for THIS
    /// tick. Must be paired with `sample(axis_idx)` reads for the same tick.
    pub fn poll(&mut self, sample_rate_hz: f32) {
        let seq = self.control.seq.load(Ordering::Acquire);
        if seq != self.last_seq {
            self.latch(seq, sample_rate_hz);
        }
    }

    fn latch(&mut self, seq: u32, sample_rate_hz: f32) {
        self.last_seq = seq;
        self.phase = 0.0;
        self.tick = 0;
        self.prev_offset = [0.0; MAX_AXES];

        let c = &self.control;
        let axis_mask = c.axis_mask.load(Ordering::Relaxed);
        let sign_mask = c.sign_mask.load(Ordering::Relaxed);
        let freq_start_millihz = c.freq_start_millihz.load(Ordering::Relaxed);
        let freq_end_millihz = c.freq_end_millihz.load(Ordering::Relaxed);
        let amplitude_nm = c.amplitude_nm.load(Ordering::Relaxed);
        let duration_ms = c.duration_ms.load(Ordering::Relaxed);
        let ramp_ms = c.ramp_ms.load(Ordering::Relaxed);

        let ms_to_ticks = |ms: u32| (ms as f32 * sample_rate_hz / 1000.0) as u32;
        let total_ticks = ms_to_ticks(duration_ms);

        if axis_mask == 0
            || amplitude_nm == 0
            || freq_start_millihz == 0
            || freq_end_millihz == 0
            || total_ticks < 2
            || sample_rate_hz <= 0.0
        {
            self.active = false;
            self.params = BuzzParams::idle();
            return;
        }

        let freq_start_hz = freq_start_millihz as f32 * 1.0e-3;
        let freq_end_hz = freq_end_millihz as f32 * 1.0e-3;
        let amp_mag = amplitude_nm as f32 / NM_PER_MM;
        let mut amp_mm = [0.0f32; MAX_AXES];
        for (i, slot) in amp_mm.iter_mut().enumerate() {
            if axis_mask & (1 << i) != 0 {
                let sign = if sign_mask & (1 << i) != 0 { -1.0 } else { 1.0 };
                *slot = sign * amp_mag;
            }
        }

        // Clamp the ramp so up- and down-ramps never overlap; a triangular
        // envelope (ramp == total/2) is the degenerate-but-valid floor.
        #[allow(clippy::integer_division)]
        let half = total_ticks / 2;
        let ramp = ms_to_ticks(ramp_ms).min(half).max(1);

        self.params = BuzzParams {
            omega_tick_start: TWO_PI * freq_start_hz / sample_rate_hz,
            omega_tick_end: TWO_PI * freq_end_hz / sample_rate_hz,
            omega_sec_start: TWO_PI * freq_start_hz,
            amp_mm,
            total_ticks,
            ramp_ticks: ramp,
        };
        self.active = true;
    }

    /// Instantaneous per-tick phase increment for the linear chirp at `tick`.
    #[inline]
    fn omega_tick_at(&self, tick: u32) -> f32 {
        let p = &self.params;
        if p.total_ticks <= 1 {
            return p.omega_tick_start;
        }
        let frac = tick as f32 / p.total_ticks as f32;
        p.omega_tick_start + (p.omega_tick_end - p.omega_tick_start) * frac
    }

    /// Amplitude scale `f_start / f(t)` that tapers displacement as the chirp
    /// climbs, holding peak velocity constant. Unity for a fixed-frequency buzz.
    #[inline]
    fn amp_scale_at(&self, tick: u32) -> f32 {
        let omega = self.omega_tick_at(tick);
        if omega != 0.0 {
            self.params.omega_tick_start / omega
        } else {
            1.0
        }
    }

    /// Per-axis additive contribution for the current tick. Call after `poll`
    /// and before `advance`, once per affected axis.
    #[inline]
    pub fn sample(&self, axis_idx: usize) -> BuzzSample {
        if !self.active {
            return BuzzSample::ZERO;
        }
        let Some(&amp) = self.params.amp_mm.get(axis_idx) else {
            return BuzzSample::ZERO;
        };
        if amp == 0.0 {
            return BuzzSample::ZERO;
        }
        let env = envelope(self.tick, self.params.total_ticks, self.params.ramp_ticks);
        let scale = self.amp_scale_at(self.tick);
        let (sin, cos) = (libm::sinf(self.phase), libm::cosf(self.phase));
        let prev = self.prev_offset.get(axis_idx).copied().unwrap_or(0.0);
        // velocity amplitude amp*scale*omega(t) == amp*omega_sec_start (constant
        // across the chirp), so the displacement taper and the rising frequency
        // cancel in the velocity term.
        BuzzSample {
            offset: env * amp * scale * sin,
            sample_start_offset: prev,
            velocity: env * amp * self.params.omega_sec_start * cos,
        }
    }

    /// Advance phase/tick after all per-axis samples for this tick are taken.
    /// Records this tick's offsets as the next tick's sample-start, and
    /// deactivates once the duration is spent (final emitted offset is exactly
    /// zero, so accumulated step counts return to base — net-zero excitation).
    pub fn advance(&mut self) {
        if !self.active {
            return;
        }
        let env = envelope(self.tick, self.params.total_ticks, self.params.ramp_ticks);
        let omega_tick = self.omega_tick_at(self.tick);
        let scale = self.amp_scale_at(self.tick);
        let factor = env * scale * libm::sinf(self.phase);
        for (i, slot) in self.prev_offset.iter_mut().enumerate() {
            let amp = self.params.amp_mm.get(i).copied().unwrap_or(0.0);
            *slot = amp * factor;
        }
        self.phase += omega_tick;
        if self.phase >= TWO_PI {
            self.phase -= TWO_PI;
        }
        self.tick += 1;
        if self.tick >= self.params.total_ticks {
            self.active = false;
            self.params = BuzzParams::idle();
        }
    }
}

impl Default for Buzz {
    fn default() -> Self {
        Self::new()
    }
}

/// Trapezoidal amplitude envelope in [0, 1]. Zero at the first tick and at the
/// final tick (`total - 1`), so a buzz both starts and ends with no offset:
/// clean spectral content (no step transient) and exact net-zero displacement.
#[inline]
pub fn envelope(tick: u32, total: u32, ramp: u32) -> f32 {
    if total == 0 || tick >= total {
        return 0.0;
    }
    let ramp = ramp.max(1);
    let up = if tick < ramp {
        tick as f32 / ramp as f32
    } else {
        1.0
    };
    let down_start = total.saturating_sub(ramp);
    let down = if tick >= down_start {
        let into = (tick - down_start) as f32 + 1.0;
        1.0 - into / ramp as f32
    } else {
        1.0
    };
    up.min(down).max(0.0)
}

#[cfg(test)]
mod tests;
