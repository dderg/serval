//! Belt-pair synchronization: release the strain two drives on one belt have
//! built up against each other (frame expansion, homing preload) without
//! moving the axis.
//!
//! Sequence, one cycle-driven state machine per SyncPair command:
//! coast the secondary (CiA402 disable, belt back-drives the rotor), dither
//! the primary through the stiction band so the strain fully dissipates, then
//! re-enable the secondary at its settled position. The next stream anchors
//! it there (streaming is always relative), so no host-side offset exists.
//! Standstill torque is measured at every phase — it is both the fight
//! metric and the pass/fail verdict.

use crate::buzz::BuzzOsc;
use crate::scale::mm_to_counts;

pub const ERR_SYNC_BUSY: i32 = -840;
pub const ERR_SYNC_NOT_ENABLED: i32 = -841;
pub const ERR_SYNC_STREAMING: i32 = -842;
pub const ERR_SYNC_BAD_AXIS: i32 = -843;
pub const ERR_SYNC_SETTLE_TIMEOUT: i32 = -844;
pub const ERR_SYNC_TORQUE_RESIDUAL: i32 = -845;
pub const ERR_SYNC_FINAL_TORQUE: i32 = -846;
pub const ERR_PIECES_DURING_SYNC: i32 = -847;
pub const ERR_SYNC_BAD_DITHER: i32 = -848;
pub const ERR_SYNC_ABORTED: i32 = -849;

/// Secondary rotor is "settled" once its encoder stays within this band of a
/// quiet anchor for `quiet_cycles`. Judged on 606Ch position, NOT 606Ch
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
    pub dither_amplitude_nm: u32,
    pub dither_freq_millihz: u32,
    pub dither_duration_ms: u16,
}

/// One cycle's drive readings, supplied by the caller.
#[derive(Debug, Clone, Copy)]
pub struct SyncInputs {
    pub now_ns: u64,
    pub torque_primary: i16,
    pub torque_secondary: i16,
    pub position_secondary: i32,
}

/// What the caller must do this cycle. Exactly one action per poll; the
/// machine advances only after the caller performs it (Disable/Enable are
/// blocking drive-chain calls, reported back via `enable_finished`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncStep {
    Idle,
    /// Command the primary drive to this absolute count target this cycle.
    SetPrimaryTarget(i32),
    /// CiA402-disable the secondary (blocking), then keep polling.
    DisableSecondary,
    /// CiA402-enable the secondary (blocking; enable seeds target=actual),
    /// then call `enable_finished(rc)`.
    EnableSecondary,
    /// Terminal: respond to the host and drop the machine.
    Done(SyncReport),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncReport {
    pub result: i32,
    pub torque_baseline_primary: i32,
    pub torque_baseline_secondary: i32,
    pub torque_released: i32,
    pub torque_dithered: i32,
    pub torque_final_primary: i32,
    pub torque_final_secondary: i32,
    pub released_delta_counts: i32,
    /// True when the secondary was re-enabled at a settled position, so the
    /// caller must clear its stream state and shift its report anchor even on
    /// a non-zero result.
    pub secondary_reseeded: bool,
}

impl SyncReport {
    fn blank() -> Self {
        SyncReport {
            result: 0,
            torque_baseline_primary: 0,
            torque_baseline_secondary: 0,
            torque_released: 0,
            torque_dithered: 0,
            torque_final_primary: 0,
            torque_final_secondary: 0,
            released_delta_counts: 0,
            secondary_reseeded: false,
        }
    }
}

#[derive(Debug)]
struct Meas {
    sum_primary: i64,
    sum_secondary: i64,
    cycles: u64,
}

impl Meas {
    fn new() -> Self {
        Meas {
            sum_primary: 0,
            sum_secondary: 0,
            cycles: 0,
        }
    }

    fn push(&mut self, primary: i16, secondary: i16) {
        self.sum_primary += i64::from(primary);
        self.sum_secondary += i64::from(secondary);
        self.cycles += 1;
    }

    fn avg(&self) -> (i32, i32) {
        let n = self.cycles.max(1) as i64;
        (
            (self.sum_primary / n) as i32,
            (self.sum_secondary / n) as i32,
        )
    }
}

#[derive(Debug, Clone, Copy)]
enum Phase {
    MeasureBaseline,
    AwaitDisable,
    CoastSettle {
        waited: u64,
        quiet: u64,
        anchor: i32,
    },
    MeasureReleased,
    Dither,
    PostDitherSettle {
        waited: u64,
        quiet: u64,
        anchor: i32,
    },
    MeasureDithered,
    AwaitEnable {
        fail_result: i32,
    },
    MeasureFinal,
    Finished,
}

#[allow(missing_debug_implementations)]
pub struct PairSync {
    params: SyncParams,
    cmd_counts_per_mm_primary: f64,
    counts_per_mm_secondary: f64,
    primary_base_target: i32,
    phase: Phase,
    meas: Meas,
    report: SyncReport,
    position_at_disable: i32,
    dither: BuzzOsc,
    dither_returned_to_base: bool,
}

impl PairSync {
    /// `primary_base_target` is the primary's current commanded hold target;
    /// the dither oscillates around it and must end exactly there.
    pub fn begin(
        params: SyncParams,
        cmd_counts_per_mm_primary: f64,
        counts_per_mm_secondary: f64,
        primary_base_target: i32,
    ) -> Result<Self, i32> {
        let mut dither = BuzzOsc::new();
        let ramp_ms = u32::from(params.dither_duration_ms) / 4;
        let rc = dither.arm(
            1,
            0x01,
            0x00,
            params.dither_freq_millihz,
            params.dither_freq_millihz,
            params.dither_amplitude_nm,
            u32::from(params.dither_duration_ms),
            ramp_ms,
            [0; crate::buzz::MAX_BUZZ_SLOTS],
        );
        if rc != 0 {
            return Err(ERR_SYNC_BAD_DITHER);
        }
        Ok(PairSync {
            params,
            cmd_counts_per_mm_primary,
            counts_per_mm_secondary,
            primary_base_target,
            phase: Phase::MeasureBaseline,
            meas: Meas::new(),
            report: SyncReport::blank(),
            position_at_disable: 0,
            dither,
            dither_returned_to_base: false,
        })
    }

    fn quiet_counts(&self) -> i32 {
        (POSITION_QUIET_MM * self.counts_per_mm_secondary.abs())
            .ceil()
            .max(1.0) as i32
    }

    fn measure_done(&mut self) -> Option<(i32, i32)> {
        if self.meas.cycles >= self.params.measure_cycles {
            let avg = self.meas.avg();
            self.meas = Meas::new();
            Some(avg)
        } else {
            None
        }
    }

    fn settle_step(
        waited: &mut u64,
        quiet: &mut u64,
        anchor: &mut i32,
        position: i32,
        quiet_counts: i32,
        quiet_cycles: u64,
        timeout: u64,
    ) -> Result<bool, i32> {
        *waited += 1;
        if position.wrapping_sub(*anchor).abs() <= quiet_counts {
            *quiet += 1;
        } else {
            *quiet = 0;
            *anchor = position;
        }
        if *quiet >= quiet_cycles {
            return Ok(true);
        }
        if *waited >= timeout {
            return Err(ERR_SYNC_SETTLE_TIMEOUT);
        }
        Ok(false)
    }

    /// Called by the caller after performing `EnableSecondary`; a failed
    /// enable is terminal for the caller (drive without torque on a belt),
    /// so the machine only handles success here.
    pub fn enable_finished(&mut self, position_secondary: i32) {
        self.report.secondary_reseeded = true;
        self.report.released_delta_counts =
            position_secondary.wrapping_sub(self.position_at_disable);
        let Phase::AwaitEnable { fail_result } = self.phase else {
            panic!("enable_finished outside AwaitEnable phase");
        };
        if fail_result != 0 {
            self.report.result = fail_result;
            self.phase = Phase::Finished;
        } else {
            self.phase = Phase::MeasureFinal;
        }
    }

    pub fn poll(&mut self, inputs: &SyncInputs) -> SyncStep {
        match self.phase {
            Phase::MeasureBaseline => {
                self.meas
                    .push(inputs.torque_primary, inputs.torque_secondary);
                if let Some((p, s)) = self.measure_done() {
                    self.report.torque_baseline_primary = p;
                    self.report.torque_baseline_secondary = s;
                    self.position_at_disable = inputs.position_secondary;
                    self.phase = Phase::AwaitDisable;
                    return SyncStep::DisableSecondary;
                }
                SyncStep::Idle
            }
            Phase::AwaitDisable => {
                self.phase = Phase::CoastSettle {
                    waited: 0,
                    quiet: 0,
                    anchor: inputs.position_secondary,
                };
                SyncStep::Idle
            }
            Phase::CoastSettle {
                mut waited,
                mut quiet,
                mut anchor,
            } => {
                match Self::settle_step(
                    &mut waited,
                    &mut quiet,
                    &mut anchor,
                    inputs.position_secondary,
                    self.quiet_counts(),
                    self.params.quiet_cycles,
                    self.params.settle_timeout_cycles,
                ) {
                    Ok(true) => {
                        self.phase = Phase::MeasureReleased;
                        SyncStep::Idle
                    }
                    Ok(false) => {
                        self.phase = Phase::CoastSettle {
                            waited,
                            quiet,
                            anchor,
                        };
                        SyncStep::Idle
                    }
                    Err(code) => {
                        self.phase = Phase::AwaitEnable { fail_result: code };
                        SyncStep::EnableSecondary
                    }
                }
            }
            Phase::MeasureReleased => {
                self.meas
                    .push(inputs.torque_primary, inputs.torque_secondary);
                if let Some((p, _s)) = self.measure_done() {
                    self.report.torque_released = p;
                    self.phase = Phase::Dither;
                }
                SyncStep::Idle
            }
            Phase::Dither => match self.dither.eval(inputs.now_ns) {
                Some((rel_mm, _vel, _acc)) => {
                    let counts = self.primary_base_target.wrapping_add(mm_to_counts(
                        f64::from(rel_mm),
                        self.cmd_counts_per_mm_primary,
                    ));
                    SyncStep::SetPrimaryTarget(counts)
                }
                None => {
                    if self.dither_returned_to_base {
                        self.phase = Phase::PostDitherSettle {
                            waited: 0,
                            quiet: 0,
                            anchor: inputs.position_secondary,
                        };
                        SyncStep::Idle
                    } else {
                        self.dither_returned_to_base = true;
                        SyncStep::SetPrimaryTarget(self.primary_base_target)
                    }
                }
            },
            Phase::PostDitherSettle {
                mut waited,
                mut quiet,
                mut anchor,
            } => {
                match Self::settle_step(
                    &mut waited,
                    &mut quiet,
                    &mut anchor,
                    inputs.position_secondary,
                    self.quiet_counts(),
                    self.params.quiet_cycles,
                    self.params.settle_timeout_cycles,
                ) {
                    Ok(true) => {
                        self.phase = Phase::MeasureDithered;
                        SyncStep::Idle
                    }
                    Ok(false) => {
                        self.phase = Phase::PostDitherSettle {
                            waited,
                            quiet,
                            anchor,
                        };
                        SyncStep::Idle
                    }
                    Err(code) => {
                        self.phase = Phase::AwaitEnable { fail_result: code };
                        SyncStep::EnableSecondary
                    }
                }
            }
            Phase::MeasureDithered => {
                self.meas
                    .push(inputs.torque_primary, inputs.torque_secondary);
                if let Some((p, _s)) = self.measure_done() {
                    self.report.torque_dithered = p;
                    let fail_result =
                        if p.unsigned_abs() > u32::from(self.params.torque_ok_tenth_pct) {
                            ERR_SYNC_TORQUE_RESIDUAL
                        } else {
                            0
                        };
                    self.phase = Phase::AwaitEnable { fail_result };
                    return SyncStep::EnableSecondary;
                }
                SyncStep::Idle
            }
            Phase::AwaitEnable { .. } => {
                panic!("poll during AwaitEnable — caller must call enable_finished first")
            }
            Phase::MeasureFinal => {
                self.meas
                    .push(inputs.torque_primary, inputs.torque_secondary);
                if let Some((p, s)) = self.measure_done() {
                    self.report.torque_final_primary = p;
                    self.report.torque_final_secondary = s;
                    let ok = u32::from(self.params.torque_ok_tenth_pct);
                    if p.unsigned_abs() > ok || s.unsigned_abs() > ok {
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
