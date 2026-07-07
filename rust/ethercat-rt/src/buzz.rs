use runtime::buzz::Buzz;
use runtime::buzz_gen::{sample_rel, ToneParams};

pub const ERR_BUZZ_NOT_ENABLED: i32 = -828;
pub const ERR_BUZZ_STREAMING: i32 = -829;
pub const ERR_BUZZ_BUSY: i32 = -830;

#[allow(missing_debug_implementations)]
pub struct BuzzOsc {
    params: ToneParams,
    base_counts: i32,
    anchor_ns: u64,
    anchored: bool,
    active: bool,
}

impl BuzzOsc {
    #[must_use]
    pub fn new() -> Self {
        Self {
            params: ToneParams {
                omega: 0.0,
                mu: 0.0,
                amplitude_mm: 0.0,
                sign: 1.0,
                base_mm: 0.0,
                microstep_distance: 1.0,
                anchor_cycle: 0,
                cycles_per_second: 1.0,
                total_seconds: 0.0,
                ramp_seconds: 0.0,
            },
            base_counts: 0,
            anchor_ns: 0,
            anchored: false,
            active: false,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn arm(
        &mut self,
        axis_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
        base_counts: i32,
    ) -> i32 {
        self.active = false;
        let mut buzz = Buzz::new();
        let rc = buzz.arm(
            1,
            axis_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        );
        if rc != 0 {
            return rc;
        }
        let excitations = buzz.take_excitations();
        let Some(first) = excitations.first() else {
            return -1;
        };
        self.params = (*first).into_params(0.0, 1.0, 1.0, 0);
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
    pub fn base_counts(&self) -> i32 {
        self.base_counts
    }

    #[cfg(test)]
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
        let t = now_ns.saturating_sub(self.anchor_ns) as f64 * 1.0e-9;
        if t as f32 >= self.params.total_seconds {
            self.active = false;
            return None;
        }
        Some(sample_rel(&self.params, t as f32))
    }
}

impl Default for BuzzOsc {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
