//! Differential belt-pair trim: standstill zeroing of the pair fight.
//! Homing following-error asymmetry, servo-sync seed variance and thermal
//! drift trap strain between the two drives sharing one belt; the resulting
//! static fight re-tensions the spans asymmetrically and feeds the belt's
//! resonances. Whenever the pair sits at commanded standstill the trim
//! integrates its low-passed differential torque into a small antisymmetric
//! position offset — the pair unwinds against itself while the carriage
//! never moves.
//!
//! Quiescence gating is the whole design: during motion a differential
//! torque is legitimate (commanded feedforward, direction- and
//! position-dependent load in the inner loop), so both the low-pass filter
//! and the integrator freeze on any cycle where the pair is not quiescent —
//! only the held offset keeps being reported. The caller decides what
//! quiescent means (rings empty, no buzz, strain-comp ramp settled), and a
//! per-pair settle window keeps the trim blind while torque relaxes and the
//! telemetry lag drains after motion stops.
//!
//! The offset rides on top of the streamed targets and is deliberately NOT
//! part of the command anchor, and a torque drop (SERVO_SYNC, M84, idle
//! timeout) resets the trim outright — the sync release is the new zero.

pub const ERR_TRIM_BAD_SLOT: i32 = -851;
pub const ERR_TRIM_BAD_CLAMP: i32 = -852;
pub const ERR_TRIM_BAD_LPF: i32 = -853;
pub const ERR_TRIM_SLOT_IN_USE: i32 = -854;
pub const ERR_TRIM_BAD_GAIN: i32 = -855;
pub const ERR_TRIM_BAD_SETTLE: i32 = -856;

/// mm/s of offset slew per 1% differential torque, in millionths.
pub const MAX_TRIM_GAIN_MICRO: u32 = 2_000_000;
pub const MAX_TRIM_CLAMP_UM: u16 = 500;
pub const MIN_TRIM_LPF_MILLIHZ: u32 = 100;
pub const MAX_TRIM_LPF_MILLIHZ: u32 = 100_000;
pub const MAX_TRIM_SETTLE_MS: u32 = 60_000;
/// Hard cap on the offset slew regardless of how hard the pair fights, so a
/// torque transient (crash, rail) cannot yank the targets.
const MAX_TRIM_SLEW_MM_S: f64 = 2.0;

struct PairTrim {
    slot_a: usize,
    slot_b: usize,
    gain_mm_s_per_pct: f64,
    clamp_mm: f64,
    lpf_alpha: f64,
    settle_cycles: u64,
    quiet_cycles: u64,
    filtered_diff_tenths: f64,
    offset_mm: f64,
    clamp_warning_pending: bool,
    clamp_warned: bool,
}

impl PairTrim {
    fn integrating(&self) -> bool {
        self.quiet_cycles > self.settle_cycles
    }
}

pub struct DiffTrimBank {
    cycle_s: f64,
    trims: Vec<PairTrim>,
}

impl DiffTrimBank {
    pub fn new(cycle_ns: i64) -> Self {
        assert!(cycle_ns > 0, "trim bank needs a positive cycle time");
        Self {
            cycle_s: cycle_ns as f64 * 1e-9,
            trims: Vec::new(),
        }
    }

    pub fn set(
        &mut self,
        num_slaves: usize,
        slot_a: u8,
        slot_b: u8,
        gain_micro: u32,
        clamp_um: u16,
        lpf_millihz: u32,
        settle_ms: u32,
    ) -> i32 {
        let (a, b) = (usize::from(slot_a), usize::from(slot_b));
        if a == b || a >= num_slaves || b >= num_slaves {
            return ERR_TRIM_BAD_SLOT;
        }
        let same_pair =
            |t: &PairTrim| (t.slot_a, t.slot_b) == (a, b) || (t.slot_a, t.slot_b) == (b, a);
        if gain_micro == 0 {
            self.trims.retain(|t| !same_pair(t));
            return 0;
        }
        if gain_micro > MAX_TRIM_GAIN_MICRO {
            return ERR_TRIM_BAD_GAIN;
        }
        if clamp_um == 0 || clamp_um > MAX_TRIM_CLAMP_UM {
            return ERR_TRIM_BAD_CLAMP;
        }
        if !(MIN_TRIM_LPF_MILLIHZ..=MAX_TRIM_LPF_MILLIHZ).contains(&lpf_millihz) {
            return ERR_TRIM_BAD_LPF;
        }
        if settle_ms > MAX_TRIM_SETTLE_MS {
            return ERR_TRIM_BAD_SETTLE;
        }
        if self
            .trims
            .iter()
            .any(|t| !same_pair(t) && [t.slot_a, t.slot_b].iter().any(|&s| s == a || s == b))
        {
            return ERR_TRIM_SLOT_IN_USE;
        }
        let lpf_tau_s = 1.0 / (2.0 * std::f64::consts::PI * f64::from(lpf_millihz) * 1e-3);
        let trim = PairTrim {
            slot_a: a,
            slot_b: b,
            gain_mm_s_per_pct: f64::from(gain_micro) * 1e-6,
            clamp_mm: f64::from(clamp_um) * 1e-3,
            lpf_alpha: self.cycle_s / (lpf_tau_s + self.cycle_s),
            settle_cycles: (f64::from(settle_ms) * 1e-3 / self.cycle_s).ceil() as u64,
            quiet_cycles: 0,
            filtered_diff_tenths: 0.0,
            offset_mm: 0.0,
            clamp_warning_pending: false,
            clamp_warned: false,
        };
        self.trims.retain(|t| !same_pair(t));
        self.trims.push(trim);
        0
    }

    pub fn active(&self) -> bool {
        !self.trims.is_empty()
    }

    pub fn reset(&mut self) {
        for t in &mut self.trims {
            t.quiet_cycles = 0;
            t.filtered_diff_tenths = 0.0;
            t.offset_mm = 0.0;
            t.clamp_warning_pending = false;
            t.clamp_warned = false;
        }
    }

    /// Feed one cycle of mechanical-frame torques (0.1% rated per slot) and
    /// add each pair's antisymmetric offset (host-frame mm) into `offset_mm`.
    /// A pair filters and integrates only after both of its slots have been
    /// quiescent for the settle window; any non-quiescent cycle restarts the
    /// window. A frozen pair still reports its held offset so the caller's
    /// commands stay continuous.
    pub fn update(
        &mut self,
        torque_mech_tenths: &[f64],
        quiescent: &[bool],
        offset_mm: &mut [f64],
    ) {
        for t in &mut self.trims {
            if quiescent[t.slot_a] && quiescent[t.slot_b] {
                t.quiet_cycles += 1;
            } else {
                t.quiet_cycles = 0;
            }
            if t.integrating() {
                let diff_tenths =
                    (torque_mech_tenths[t.slot_a] - torque_mech_tenths[t.slot_b]) / 2.0;
                assert!(diff_tenths.is_finite(), "non-finite differential torque");
                t.filtered_diff_tenths += t.lpf_alpha * (diff_tenths - t.filtered_diff_tenths);
                let slew_mm_s = (-t.gain_mm_s_per_pct * t.filtered_diff_tenths / 10.0)
                    .clamp(-MAX_TRIM_SLEW_MM_S, MAX_TRIM_SLEW_MM_S);
                let unclamped = t.offset_mm + slew_mm_s * self.cycle_s;
                t.offset_mm = unclamped.clamp(-t.clamp_mm, t.clamp_mm);
                if unclamped != t.offset_mm && !t.clamp_warned {
                    t.clamp_warned = true;
                    t.clamp_warning_pending = true;
                }
            }
            offset_mm[t.slot_a] += t.offset_mm;
            offset_mm[t.slot_b] -= t.offset_mm;
        }
    }

    /// Live internal state per pair, for periodic telemetry: (slot_a,
    /// slot_b, offset_mm, filtered differential torque in 0.1%, integrating).
    pub fn snapshot(&self) -> Vec<(usize, usize, f64, f64, bool)> {
        self.trims
            .iter()
            .map(|t| {
                (
                    t.slot_a,
                    t.slot_b,
                    t.offset_mm,
                    t.filtered_diff_tenths,
                    t.integrating(),
                )
            })
            .collect()
    }

    /// One-shot clamp notification per arm/reset: the pair that just hit its
    /// offset clamp, for the caller to log. Hitting the clamp means residual
    /// fight remains beyond the trim's authority (or the feedback sign is
    /// wrong for this mechanics), which the user must hear about.
    pub fn drain_clamp_warning(&mut self) -> Option<(usize, usize)> {
        for t in &mut self.trims {
            if t.clamp_warning_pending {
                t.clamp_warning_pending = false;
                return Some((t.slot_a, t.slot_b));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests;
