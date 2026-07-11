//! Differential belt-pair damper.
//!
//! Two drives sharing one belt form a lightly damped rotor-vs-rotor mode
//! coupled through belt compliance; each drive's own position loop cannot
//! damp it because damping needs the RELATIVE velocity, which only the host
//! sees. The bank injects an antisymmetric torque-offset pair proportional
//! to the low-passed differential velocity — a virtual dashpot between the
//! two rotors. On synchronized motion the differential velocity is zero, so
//! the damper spends no torque and is invisible to the carriage.

pub const ERR_DAMPER_BAD_SLOT: i32 = -831;
pub const ERR_DAMPER_BAD_CLAMP: i32 = -832;
pub const ERR_DAMPER_BAD_LPF: i32 = -833;
pub const ERR_DAMPER_SLOT_IN_USE: i32 = -834;

pub const MAX_DAMPER_CLAMP_TENTHS: u16 = 1000;
pub const MIN_DAMPER_LPF_MILLIHZ: u32 = 1_000;
pub const MAX_DAMPER_LPF_MILLIHZ: u32 = 2_000_000;

struct PairDamper {
    slot_a: usize,
    slot_b: usize,
    gain_tenths_per_mm_s: f64,
    clamp_tenths: f64,
    lpf_alpha: f64,
    filtered_diff_mm_s: f64,
}

pub struct DiffDamperBank {
    cycle_s: f64,
    dampers: Vec<PairDamper>,
}

impl DiffDamperBank {
    pub fn new(cycle_ns: i64) -> Self {
        assert!(cycle_ns > 0, "damper bank needs a positive cycle time");
        Self {
            cycle_s: cycle_ns as f64 * 1e-9,
            dampers: Vec::new(),
        }
    }

    pub fn set(
        &mut self,
        num_slaves: usize,
        slot_a: u8,
        slot_b: u8,
        gain_milli: u32,
        clamp_tenths: u16,
        lpf_millihz: u32,
    ) -> i32 {
        let (a, b) = (usize::from(slot_a), usize::from(slot_b));
        if a == b || a >= num_slaves || b >= num_slaves {
            return ERR_DAMPER_BAD_SLOT;
        }
        let same_pair =
            |d: &PairDamper| (d.slot_a, d.slot_b) == (a, b) || (d.slot_a, d.slot_b) == (b, a);
        if gain_milli == 0 {
            self.dampers.retain(|d| !same_pair(d));
            return 0;
        }
        if clamp_tenths == 0 || clamp_tenths > MAX_DAMPER_CLAMP_TENTHS {
            return ERR_DAMPER_BAD_CLAMP;
        }
        if !(MIN_DAMPER_LPF_MILLIHZ..=MAX_DAMPER_LPF_MILLIHZ).contains(&lpf_millihz) {
            return ERR_DAMPER_BAD_LPF;
        }
        if self
            .dampers
            .iter()
            .any(|d| !same_pair(d) && [d.slot_a, d.slot_b].iter().any(|&s| s == a || s == b))
        {
            return ERR_DAMPER_SLOT_IN_USE;
        }
        let lpf_tau_s = 1.0 / (2.0 * std::f64::consts::PI * f64::from(lpf_millihz) * 1e-3);
        let damper = PairDamper {
            slot_a: a,
            slot_b: b,
            gain_tenths_per_mm_s: f64::from(gain_milli) * 1e-3,
            clamp_tenths: f64::from(clamp_tenths),
            lpf_alpha: self.cycle_s / (lpf_tau_s + self.cycle_s),
            filtered_diff_mm_s: 0.0,
        };
        self.dampers.retain(|d| !same_pair(d));
        self.dampers.push(damper);
        0
    }

    pub fn active(&self) -> bool {
        !self.dampers.is_empty()
    }

    pub fn reset_filters(&mut self) {
        for d in &mut self.dampers {
            d.filtered_diff_mm_s = 0.0;
        }
    }

    /// Feed one cycle of host-frame velocities (mm/s per slot) and add each
    /// pair's antisymmetric torque (host-frame, 0.1% rated) into `out_tenths`.
    pub fn accumulate(&mut self, vel_mm_s: &[f64], out_tenths: &mut [f32]) {
        for d in &mut self.dampers {
            let diff = vel_mm_s[d.slot_a] - vel_mm_s[d.slot_b];
            assert!(diff.is_finite(), "non-finite differential velocity");
            d.filtered_diff_mm_s += d.lpf_alpha * (diff - d.filtered_diff_mm_s);
            let torque = (-d.gain_tenths_per_mm_s * d.filtered_diff_mm_s)
                .clamp(-d.clamp_tenths, d.clamp_tenths);
            out_tenths[d.slot_a] += torque as f32;
            out_tenths[d.slot_b] -= torque as f32;
        }
    }
}

#[cfg(test)]
mod tests;
