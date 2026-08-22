pub const ERR_BUZZ_NOT_ENABLED: i32 = -828;
pub const ERR_BUZZ_STREAMING: i32 = -829;
pub const ERR_BUZZ_BUSY: i32 = -830;
pub const ERR_BUZZ_AXIS_MASK: i32 = -2;
pub const ERR_BUZZ_FREQ_ZERO: i32 = -3;

pub const MAX_BUZZ_SLOTS: usize = 8;

#[derive(Clone, Copy, Debug, Default)]
struct ToneParams {
    omega: f32,
    mu: f32,
    amplitude_mm: f32,
    total_seconds: f32,
    ramp_seconds: f32,
}

#[allow(missing_debug_implementations)]
pub struct BuzzOsc {
    params: ToneParams,
    slot_mask: u8,
    sign_mask: u8,
    base_counts: [i32; MAX_BUZZ_SLOTS],
    anchor_ns: u64,
    anchored: bool,
    active: bool,
}

impl BuzzOsc {
    #[must_use]
    pub fn new() -> Self {
        Self {
            params: ToneParams::default(),
            slot_mask: 0,
            sign_mask: 0,
            base_counts: [0; MAX_BUZZ_SLOTS],
            anchor_ns: 0,
            anchored: false,
            active: false,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn arm(
        &mut self,
        num_slots: u8,
        slot_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
        base_counts: [i32; MAX_BUZZ_SLOTS],
    ) -> i32 {
        self.active = false;
        let disarm = amplitude_nm == 0 || duration_ms == 0 || slot_mask == 0;
        if disarm {
            return if slot_mask == 0 && amplitude_nm != 0 && duration_ms != 0 {
                -1
            } else {
                0
            };
        }
        let slot_bits = if num_slots >= 8 {
            u8::MAX
        } else {
            (1u8 << num_slots) - 1
        };
        if slot_mask & !slot_bits != 0 {
            return ERR_BUZZ_AXIS_MASK;
        }
        if freq_start_millihz == 0 || freq_end_millihz == 0 {
            return ERR_BUZZ_FREQ_ZERO;
        }
        let total_seconds = duration_ms as f32 * 1.0e-3;
        let omega = 2.0 * core::f32::consts::PI * freq_start_millihz as f32 * 1.0e-3;
        let omega_end = 2.0 * core::f32::consts::PI * freq_end_millihz as f32 * 1.0e-3;
        self.params = ToneParams {
            omega,
            mu: (omega_end - omega) / total_seconds,
            amplitude_mm: amplitude_nm as f32 * 1.0e-6,
            total_seconds,
            ramp_seconds: (ramp_ms as f32 * 1.0e-3)
                .min(0.5 * total_seconds)
                .max(f32::MIN_POSITIVE),
        };
        self.slot_mask = slot_mask;
        self.sign_mask = sign_mask;
        self.base_counts = base_counts;
        self.anchored = false;
        self.active = true;
        0
    }

    #[must_use]
    pub fn active(&self) -> bool {
        self.active
    }

    #[must_use]
    pub fn drives_slot(&self, slot: usize) -> bool {
        self.active && slot < MAX_BUZZ_SLOTS && self.slot_mask & (1 << slot) != 0
    }

    #[must_use]
    pub fn slot_sign(&self, slot: usize) -> f32 {
        if slot < MAX_BUZZ_SLOTS && self.sign_mask & (1 << slot) != 0 {
            -1.0
        } else {
            1.0
        }
    }

    #[must_use]
    pub fn base_counts(&self, slot: usize) -> i32 {
        self.base_counts[slot]
    }

    pub fn clear(&mut self) {
        self.active = false;
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn eval(&mut self, now_ns: u64) -> Option<(f32, f32, f32)> {
        if !self.active {
            return None;
        }
        if !self.anchored {
            self.anchor_ns = now_ns;
            self.anchored = true;
        }
        let t = now_ns.saturating_sub(self.anchor_ns) as f64 as f32 * 1.0e-9;
        if t >= self.params.total_seconds {
            self.active = false;
            return None;
        }
        Some(sample_rel(self.params, t))
    }
}

impl Default for BuzzOsc {
    fn default() -> Self {
        Self::new()
    }
}

fn envelope(t: f32, total: f32, ramp: f32) -> f32 {
    if total <= 0.0 || t <= 0.0 || t >= total {
        return 0.0;
    }
    (t / ramp)
        .min(1.0)
        .min(((total - t) / ramp).min(1.0))
        .max(0.0)
}

fn omega_inst(params: ToneParams, t: f32) -> f32 {
    params.omega + params.mu * t
}

fn amplitude(params: ToneParams, t: f32) -> f32 {
    if params.mu == 0.0 {
        return params.amplitude_mm;
    }
    let omega = omega_inst(params, t);
    if omega.abs() <= f32::MIN_POSITIVE {
        params.amplitude_mm
    } else {
        params.amplitude_mm * params.omega / omega
    }
}

fn amplitude_rate(params: ToneParams, t: f32) -> f32 {
    if params.mu == 0.0 {
        return 0.0;
    }
    let omega = omega_inst(params, t);
    if omega.abs() <= f32::MIN_POSITIVE {
        0.0
    } else {
        -params.amplitude_mm * params.omega * params.mu / (omega * omega)
    }
}

fn amplitude_accel(params: ToneParams, t: f32) -> f32 {
    if params.mu == 0.0 {
        return 0.0;
    }
    let omega = omega_inst(params, t);
    if omega.abs() <= f32::MIN_POSITIVE {
        0.0
    } else {
        2.0 * params.amplitude_mm * params.omega * params.mu * params.mu / (omega * omega * omega)
    }
}

fn sample_rel(params: ToneParams, t: f32) -> (f32, f32, f32) {
    let ramp = params.ramp_seconds.max(f32::MIN_POSITIVE);
    let env = envelope(t, params.total_seconds, ramp);
    let env_rate = if t <= 0.0 || t >= params.total_seconds {
        0.0
    } else if t < ramp {
        1.0 / ramp
    } else if t > params.total_seconds - ramp {
        -1.0 / ramp
    } else {
        0.0
    };
    let amp = amplitude(params, t);
    let amp_rate = amplitude_rate(params, t);
    let amp_accel = amplitude_accel(params, t);
    let omega = omega_inst(params, t);
    let phase = (params.omega + 0.5 * params.mu * t) * t;
    let sin = libm::sinf(phase);
    let cos = libm::cosf(phase);
    let position = env * amp * sin;
    let velocity = env_rate * amp * sin + env * amp_rate * sin + env * amp * omega * cos;
    let sin_coeff = 2.0 * env_rate * amp_rate + env * amp_accel - env * amp * omega * omega;
    let cos_coeff =
        2.0 * env_rate * amp * omega + 2.0 * env * amp_rate * omega + env * amp * params.mu;
    let acceleration = sin_coeff * sin + cos_coeff * cos;
    (position, velocity, acceleration)
}

#[cfg(test)]
mod tests;
