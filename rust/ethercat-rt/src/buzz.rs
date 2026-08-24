use trajectory::BuzzProfile;

pub const ERR_BUZZ_NOT_ENABLED: i32 = -828;
pub const ERR_BUZZ_STREAMING: i32 = -829;
pub const ERR_BUZZ_BUSY: i32 = -830;
pub const ERR_BUZZ_AXIS_MASK: i32 = -2;
pub const ERR_BUZZ_FREQ_ZERO: i32 = -3;
pub const ERR_BUZZ_PROFILE: i32 = -4;

pub const MAX_BUZZ_SLOTS: usize = 8;

#[derive(Debug)]
pub struct BuzzOsc {
    profile: Option<BuzzProfile>,
    slot_mask: u8,
    sign_mask: u8,
    base_counts: [i32; MAX_BUZZ_SLOTS],
    anchor_ns: u64,
    anchored: bool,
}

impl BuzzOsc {
    #[must_use]
    pub fn new() -> Self {
        Self {
            profile: None,
            slot_mask: 0,
            sign_mask: 0,
            base_counts: [0; MAX_BUZZ_SLOTS],
            anchor_ns: 0,
            anchored: false,
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
        self.profile = None;
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
        let Ok(profile) = BuzzProfile::try_new(
            f64::from(amplitude_nm) * 1.0e-6,
            f64::from(freq_start_millihz) * 1.0e-3,
            f64::from(freq_end_millihz) * 1.0e-3,
            f64::from(duration_ms) * 1.0e-3,
            f64::from(ramp_ms) * 1.0e-3,
            0.0,
        ) else {
            return ERR_BUZZ_PROFILE;
        };
        self.profile = Some(profile);
        self.slot_mask = slot_mask;
        self.sign_mask = sign_mask;
        self.base_counts = base_counts;
        self.anchored = false;
        0
    }

    #[must_use]
    pub fn active(&self) -> bool {
        self.profile.is_some()
    }

    #[must_use]
    pub fn drives_slot(&self, slot: usize) -> bool {
        self.active() && slot < MAX_BUZZ_SLOTS && self.slot_mask & (1 << slot) != 0
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
        self.profile = None;
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn eval(&mut self, now_ns: u64) -> Option<(f32, f32, f32)> {
        let duration = self.profile.as_ref()?.duration();
        if !self.anchored {
            self.anchor_ns = now_ns;
            self.anchored = true;
        }
        let t = now_ns.saturating_sub(self.anchor_ns) as f64 * 1.0e-9;
        if t >= duration {
            self.profile = None;
            return None;
        }
        let sample = self.profile.as_ref()?.eval(t);
        Some((
            sample.position as f32,
            sample.velocity as f32,
            sample.acceleration as f32,
        ))
    }
}

impl Default for BuzzOsc {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
