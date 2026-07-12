//! Belt strain release: de-energize every selected drive at once, let the
//! mechanics relax freely, re-energize. With all rotors free nothing
//! constrains the relaxation through the CoreXY coupling and no stiction band
//! has to be crossed by an energized partner — the belts simply spring to
//! their neutral state. CiA402 enable seeds each drive's target at its actual
//! position and streaming is always relative, so no position is lost.
//!
//! One cycle-driven state machine per SyncRelease command: measure the
//! standstill torques (the fight metric), disable the selected slots, wait
//! for every encoder to go quiet, re-enable, then measure again — the final
//! torques are the pass/fail verdict.

pub const ERR_SYNC_BUSY: i32 = -840;
pub const ERR_SYNC_NOT_ENABLED: i32 = -841;
pub const ERR_SYNC_STREAMING: i32 = -842;
pub const ERR_SYNC_BAD_MASK: i32 = -843;
pub const ERR_SYNC_SETTLE_TIMEOUT: i32 = -844;
pub const ERR_SYNC_FINAL_TORQUE: i32 = -846;
pub const ERR_PIECES_DURING_SYNC: i32 = -847;
pub const ERR_SYNC_ABORTED: i32 = -849;

/// Matches the fixed-size arrays in the SyncReleaseResponse wire message.
pub const MAX_RELEASE_SLOTS: usize = 4;

/// A coasting rotor is "settled" once its encoder stays within this band of a
/// quiet anchor for `quiet_cycles`. Judged on 6064h position, NOT 606Ch
/// velocity: the A6-EC velocity estimate carries hundreds to thousands of
/// counts/s of standstill noise, so no velocity threshold tight enough to
/// mean "settled" is ever met, while encoder position noise is a few counts.
const POSITION_QUIET_MM: f64 = 0.01;

#[derive(Debug, Clone, Copy)]
pub struct SyncParams {
    pub torque_ok_tenth_pct: u16,
    pub settle_timeout_cycles: u64,
    pub measure_cycles: u64,
    pub quiet_cycles: u64,
}

/// One cycle's drive readings for every slot, supplied by the caller;
/// entries outside the release mask are ignored.
#[derive(Debug, Clone, Copy)]
pub struct SyncInputs {
    pub torque: [i16; MAX_RELEASE_SLOTS],
    pub position: [i32; MAX_RELEASE_SLOTS],
}

/// What the caller must do this cycle. Exactly one action per poll; the
/// machine advances only after the caller performs it (Disable/Enable are
/// blocking drive-chain calls, reported back via `enable_finished`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncStep {
    Idle,
    /// CiA402-disable every masked slot (blocking), then keep polling.
    DisableAll,
    /// CiA402-enable every masked slot (blocking; enable seeds
    /// target=actual), then call `enable_finished` with the settled
    /// positions.
    EnableAll,
    /// Terminal: respond to the host and drop the machine.
    Done(SyncReport),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncReport {
    pub result: i32,
    pub torque_baseline: [i32; MAX_RELEASE_SLOTS],
    pub torque_final: [i32; MAX_RELEASE_SLOTS],
    pub released_delta_counts: [i32; MAX_RELEASE_SLOTS],
    /// True once the masked slots were re-enabled at settled positions, so
    /// the caller must clear their stream state and shift their report
    /// anchors even on a non-zero result.
    pub reseeded: bool,
}

impl SyncReport {
    fn blank() -> Self {
        SyncReport {
            result: 0,
            torque_baseline: [0; MAX_RELEASE_SLOTS],
            torque_final: [0; MAX_RELEASE_SLOTS],
            released_delta_counts: [0; MAX_RELEASE_SLOTS],
            reseeded: false,
        }
    }
}

#[derive(Debug)]
struct Meas {
    sums: [i64; MAX_RELEASE_SLOTS],
    cycles: u64,
}

impl Meas {
    fn new() -> Self {
        Meas {
            sums: [0; MAX_RELEASE_SLOTS],
            cycles: 0,
        }
    }

    fn push(&mut self, torque: &[i16; MAX_RELEASE_SLOTS]) {
        for (sum, &t) in self.sums.iter_mut().zip(torque.iter()) {
            *sum += i64::from(t);
        }
        self.cycles += 1;
    }

    fn avg(&self) -> [i32; MAX_RELEASE_SLOTS] {
        let n = self.cycles.max(1) as i64;
        self.sums.map(|sum| (sum / n) as i32)
    }
}

#[derive(Debug, Clone, Copy)]
enum Phase {
    MeasureBaseline,
    AwaitDisable,
    CoastSettle,
    AwaitEnable { fail_result: i32 },
    MeasureFinal,
    Finished,
}

#[derive(Debug)]
pub struct SyncRelease {
    params: SyncParams,
    slot_mask: u8,
    counts_per_mm: [f64; MAX_RELEASE_SLOTS],
    phase: Phase,
    meas: Meas,
    report: SyncReport,
    position_at_disable: [i32; MAX_RELEASE_SLOTS],
    settle_waited: u64,
    settle_quiet: [u64; MAX_RELEASE_SLOTS],
    settle_anchor: [i32; MAX_RELEASE_SLOTS],
}

impl SyncRelease {
    pub fn begin(
        params: SyncParams,
        slot_mask: u8,
        counts_per_mm: [f64; MAX_RELEASE_SLOTS],
    ) -> Result<Self, i32> {
        if slot_mask == 0 || usize::from(slot_mask) >> MAX_RELEASE_SLOTS != 0 {
            return Err(ERR_SYNC_BAD_MASK);
        }
        Ok(SyncRelease {
            params,
            slot_mask,
            counts_per_mm,
            phase: Phase::MeasureBaseline,
            meas: Meas::new(),
            report: SyncReport::blank(),
            position_at_disable: [0; MAX_RELEASE_SLOTS],
            settle_waited: 0,
            settle_quiet: [0; MAX_RELEASE_SLOTS],
            settle_anchor: [0; MAX_RELEASE_SLOTS],
        })
    }

    pub fn masked_slots(&self) -> impl Iterator<Item = usize> + '_ {
        (0..MAX_RELEASE_SLOTS).filter(move |s| self.slot_mask & (1 << s) != 0)
    }

    fn quiet_counts(&self, slot: usize) -> i32 {
        (POSITION_QUIET_MM * self.counts_per_mm[slot].abs())
            .ceil()
            .max(1.0) as i32
    }

    fn measure_done(&mut self) -> Option<[i32; MAX_RELEASE_SLOTS]> {
        if self.meas.cycles >= self.params.measure_cycles {
            let avg = self.meas.avg();
            self.meas = Meas::new();
            Some(avg)
        } else {
            None
        }
    }

    fn settle_step(&mut self, position: &[i32; MAX_RELEASE_SLOTS]) -> Result<bool, i32> {
        self.settle_waited += 1;
        let mask = self.slot_mask;
        for s in (0..MAX_RELEASE_SLOTS).filter(|s| mask & (1 << s) != 0) {
            if position[s].wrapping_sub(self.settle_anchor[s]).abs() <= self.quiet_counts(s) {
                self.settle_quiet[s] += 1;
            } else {
                self.settle_quiet[s] = 0;
                self.settle_anchor[s] = position[s];
            }
        }
        let quiet_cycles = self.params.quiet_cycles;
        if self
            .masked_slots()
            .all(|s| self.settle_quiet[s] >= quiet_cycles)
        {
            return Ok(true);
        }
        if self.settle_waited >= self.params.settle_timeout_cycles {
            return Err(ERR_SYNC_SETTLE_TIMEOUT);
        }
        Ok(false)
    }

    /// Called by the caller after performing `EnableAll`; a failed enable is
    /// terminal for the caller (drive without torque on a belt), so the
    /// machine only handles success here. The final torques are measured
    /// even on a failed release so the report shows the true end state
    /// instead of blank zeros.
    pub fn enable_finished(&mut self, position: &[i32; MAX_RELEASE_SLOTS]) {
        self.report.reseeded = true;
        for s in 0..MAX_RELEASE_SLOTS {
            self.report.released_delta_counts[s] =
                position[s].wrapping_sub(self.position_at_disable[s]);
        }
        let Phase::AwaitEnable { fail_result } = self.phase else {
            panic!("enable_finished outside AwaitEnable phase");
        };
        self.report.result = fail_result;
        self.phase = Phase::MeasureFinal;
    }

    pub fn poll(&mut self, inputs: &SyncInputs) -> SyncStep {
        match self.phase {
            Phase::MeasureBaseline => {
                self.meas.push(&inputs.torque);
                if let Some(avg) = self.measure_done() {
                    self.report.torque_baseline = avg;
                    self.position_at_disable = inputs.position;
                    self.phase = Phase::AwaitDisable;
                    return SyncStep::DisableAll;
                }
                SyncStep::Idle
            }
            Phase::AwaitDisable => {
                self.settle_anchor = inputs.position;
                self.phase = Phase::CoastSettle;
                SyncStep::Idle
            }
            Phase::CoastSettle => match self.settle_step(&inputs.position) {
                Ok(true) => {
                    self.phase = Phase::AwaitEnable { fail_result: 0 };
                    SyncStep::EnableAll
                }
                Ok(false) => SyncStep::Idle,
                Err(code) => {
                    self.phase = Phase::AwaitEnable { fail_result: code };
                    SyncStep::EnableAll
                }
            },
            Phase::AwaitEnable { .. } => {
                panic!("poll during AwaitEnable — caller must call enable_finished first")
            }
            Phase::MeasureFinal => {
                self.meas.push(&inputs.torque);
                if let Some(avg) = self.measure_done() {
                    self.report.torque_final = avg;
                    let ok = u32::from(self.params.torque_ok_tenth_pct);
                    let too_high = self
                        .masked_slots()
                        .any(|s| self.report.torque_final[s].unsigned_abs() > ok);
                    if self.report.result == 0 && too_high {
                        self.report.result = ERR_SYNC_FINAL_TORQUE;
                    }
                    self.phase = Phase::Finished;
                    return SyncStep::Done(self.report);
                }
                SyncStep::Idle
            }
            Phase::Finished => SyncStep::Done(self.report),
        }
    }
}

#[cfg(test)]
mod tests;
