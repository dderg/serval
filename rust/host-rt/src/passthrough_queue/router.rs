use std::sync::Arc;
use std::time::Instant;

use indexmap::IndexMap;

use crate::clock::{Clock, HostSecs, PrintTime, instant_to_f64};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);

impl McuHandle {
    pub fn raw(&self) -> u32 {
        self.0
    }

    pub fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
}

#[derive(Debug)]
pub enum RouterError {
    UnknownMcu(McuHandle),
    NoClockEstimate(McuHandle),
}

impl std::fmt::Display for RouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownMcu(h) => write!(f, "unknown mcu handle {}", h.raw()),
            Self::NoClockEstimate(h) => write!(
                f,
                "mcu handle {} has no valid clocksync record — the record was \
                 invalidated by a (re)connect and no fresh estimate has arrived",
                h.raw()
            ),
        }
    }
}

impl std::error::Error for RouterError {}

/// A record this old has missed samples. Measured healthy sim worlds gap up to
/// ~9 s (klippy's outlier rejection drops samples and a loaded reactor defers
/// the timer), so this is a loud degradation signal, not a hard stop.
pub const DEGRADED_CLOCK_RECORD_AGE_SECS: f64 =
    3.0 * crate::clock_regression::NON_RESONANT_GET_CLOCK_PERIOD_SECS;

/// A record older than the regression's own sample window contains no live
/// sample at all: clocksync has stopped feeding the router and every
/// projection off it is an open-loop extrapolation. Anchoring a step stream on
/// one is a hard error.
pub const MAX_CLOCK_RECORD_AGE_SECS: f64 = crate::clock_regression::REGRESSION_WINDOW_SECS;

/// One MCU's live host↔MCU clock map. Present only while a record seeded
/// after the MCU's current boot epoch is live: a (re)connect drops it, so no
/// projection can silently run off the previous epoch's numbers.
#[derive(Debug, Clone, Copy)]
struct ClockEst {
    /// Measured ticks-per-host-second from the clocksync regression; drifts
    /// around the nominal frequency by ppm. Used to extrapolate what the
    /// MCU's clock reads at a host instant — never to define print_time.
    clock_freq: f64,
    /// Host instant of the regression's decay-weighted sample centroid, which
    /// legitimately trails the newest sample by up to `1/decay` periods. It is
    /// the projection's anchor point, NOT a measure of the record's freshness.
    clock_offset: f64,
    last_clock: u64,
    /// Whether the publishing clocksync had latched convergence. Anchoring
    /// step streams on an unconverged estimate is rejected.
    converged: bool,
    /// Host instant at which the router accepted this estimate. The only
    /// honest freshness measure: `host_now - updated_at` counts the missed
    /// `get_clock` samples.
    updated_at: f64,
}

/// The record numbers plus the clock they project to at a host instant —
/// what a re-anchor reports so a wrong record is visible in the log.
#[derive(Debug, Clone, Copy)]
pub struct ClockRecordSnapshot {
    pub clock_freq: f64,
    pub clock_offset: f64,
    pub last_clock: u64,
    pub converged: bool,
    pub projected_now: u64,
    /// Seconds since the router last accepted an estimate for this MCU.
    pub age_secs: f64,
    /// Seconds between the regression centroid and now: the projection's lever
    /// arm, which grows to `1/decay` periods on a perfectly healthy record.
    pub centroid_lag_secs: f64,
}

#[derive(Debug)]
struct McuRecord {
    est: Option<ClockEst>,
    /// The datasheet CLOCK_FREQ. `print_time` is defined as
    /// `clock / nominal_freq`, so converting through the regression frequency
    /// instead accumulates ppm × uptime of error (seconds after hours).
    nominal_freq: f64,
}

impl McuRecord {
    fn est(&self, mcu: McuHandle) -> Result<&ClockEst, RouterError> {
        self.est.as_ref().ok_or(RouterError::NoClockEstimate(mcu))
    }
}

pub struct PassthroughRouter {
    mcus: IndexMap<McuHandle, McuRecord>,
    next_handle: u32,
    clock: Arc<dyn Clock + Send + Sync>,
}

impl std::fmt::Debug for PassthroughRouter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PassthroughRouter")
            .field("mcus", &self.mcus)
            .field("next_handle", &self.next_handle)
            .finish()
    }
}

impl PassthroughRouter {
    pub fn with_clock(clock: Arc<dyn Clock + Send + Sync>) -> Self {
        Self {
            mcus: IndexMap::new(),
            next_handle: 0,
            clock,
        }
    }

    pub fn claim_mcu(&mut self, _label: &str) -> McuHandle {
        let handle = McuHandle(self.next_handle);
        self.next_handle += 1;
        self.mcus.insert(
            handle,
            McuRecord {
                est: None,
                nominal_freq: 0.0,
            },
        );
        handle
    }

    /// Drop this MCU's clock record. Every (re)connect calls this: the MCU
    /// restarts its counter at zero, so the previous epoch's
    /// `(offset, last_clock)` pair projects a clock that is wrong by the
    /// previous boot's uptime. Projections fail loudly until a fresh estimate
    /// arrives.
    pub fn invalidate_clock_est(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        let dropped = rec.est.take();
        tracing::info!(
            subsystem = "clocksync",
            event = "invalidate_clock_est",
            mcu = ?mcu,
            had_record = dropped.is_some(),
            dropped_freq = dropped.map(|e| e.clock_freq),
            dropped_offset = dropped.map(|e| e.clock_offset),
            dropped_last_clock = dropped.map(|e| e.last_clock),
            "[clock-seed] clock record invalidated by (re)connect"
        );
        Ok(())
    }

    pub fn clock_est_converged(&self, mcu: McuHandle) -> bool {
        self.mcus
            .get(&mcu)
            .and_then(|r| r.est)
            .is_some_and(|e| e.converged)
    }

    /// The live record plus the clock it projects at this instant. `None`
    /// when the record is absent or invalidated.
    pub fn clock_record(&self, mcu: McuHandle) -> Option<ClockRecordSnapshot> {
        let est = self.mcus.get(&mcu)?.est?;
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - est.clock_offset) * est.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected_now = est.last_clock.wrapping_add(delta.max(0.0) as u64);
        Some(ClockRecordSnapshot {
            clock_freq: est.clock_freq,
            clock_offset: est.clock_offset,
            last_clock: est.last_clock,
            converged: est.converged,
            projected_now,
            age_secs: host_now - est.updated_at,
            centroid_lag_secs: host_now - est.clock_offset,
        })
    }

    pub fn set_nominal_freq(&mut self, mcu: McuHandle, freq_hz: f64) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.nominal_freq = freq_hz;
        Ok(())
    }

    pub fn release_mcu(&mut self, handle: McuHandle) {
        self.mcus.swap_remove(&handle);
    }

    pub fn set_clock_est(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset: f64,
        last_clock: u64,
    ) -> Result<(), RouterError> {
        tracing::info!(
            subsystem = "clocksync",
            event = "set_clock_est",
            mcu = ?mcu,
            freq,
            offset,
            last_clock,
            "[clock-seed] set_clock_est"
        );
        let now = instant_to_f64(self.clock.now());
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.est = Some(ClockEst {
            clock_freq: freq,
            clock_offset: offset,
            last_clock,
            converged: true,
            updated_at: now,
        });
        Ok(())
    }

    /// Set the router's clock record from a Python-side clocksync estimate.
    ///
    /// `offset_raw` is `time_avg + min_half_rtt` in CLOCK_MONOTONIC_RAW seconds
    /// (what Python's `_handle_clock` computes and the mirror callback exports).
    /// `host_now_raw` is accepted for API compatibility but is NOT used in the
    /// projection — using it would embed the Python→Rust GIL-hop latency ε
    /// directly into `clock_offset`, biasing every subsequent projection by ε
    /// (up to tens of ms on a loaded Pi 3B).
    ///
    /// Instead, `CLOCK_MONOTONIC_RAW` is read here in Rust at the same instant
    /// as `instant_to_f64(self.clock.now())`, so the conversion constant
    /// `raw_at_anchor = raw_now - instant_now` is computed without any
    /// cross-runtime latency and `clock_offset = offset_raw - raw_at_anchor`
    /// is exact up to µs sample skew.
    pub fn set_clock_est_rebased(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset_raw: f64,
        last_clock: u64,
        converged: bool,
        _host_now_raw: f64,
    ) -> Result<(), RouterError> {
        let bridge_now_instant = instant_to_f64(self.clock.now());
        let bridge_now_raw = crate::clock::monotonic_raw_secs();
        let clock_offset = offset_raw - (bridge_now_raw - bridge_now_instant);
        tracing::debug!(
            subsystem = "clocksync",
            event = "set_clock_est_rebased",
            mcu = ?mcu,
            freq,
            offset_raw,
            bridge_now_raw,
            bridge_now_instant,
            clock_offset,
            last_clock,
            converged,
            "[clock-seed] set_clock_est_rebased"
        );
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.est = Some(ClockEst {
            clock_freq: freq,
            clock_offset,
            last_clock,
            converged,
            updated_at: bridge_now_instant,
        });
        Ok(())
    }

    pub fn set_clock_est_from_sample(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        host_send: Instant,
        mcu_at_send: u64,
    ) -> Result<(), RouterError> {
        let clock_offset = instant_to_f64(host_send);
        tracing::info!(
            subsystem = "clocksync",
            event = "set_clock_est_from_sample",
            mcu = ?mcu,
            freq,
            clock_offset,
            mcu_at_send,
            "[clock-seed] set_clock_est_from_sample"
        );
        let now = instant_to_f64(self.clock.now());
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.est = Some(ClockEst {
            clock_freq: freq,
            clock_offset,
            last_clock: mcu_at_send,
            converged: true,
            updated_at: now,
        });
        Ok(())
    }

    /// Convert an MCU tick count to a wall-clock `OffsetDateTime`.
    ///
    /// Returns `None` when this MCU has no live clock record.
    ///
    /// `estimated = true` when the tick is more than one frequency-second from
    /// the anchor, i.e. significant extrapolation.
    pub fn wall_time_at_mcu(
        &self,
        mcu: McuHandle,
        mcu_ticks: u64,
    ) -> Option<(time::OffsetDateTime, bool)> {
        let rec = self.mcus.get(&mcu)?.est?;
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (rec.last_clock as f64);
        let mcu_host_instant = rec.clock_offset + delta_ticks / rec.clock_freq;
        let now_instant = instant_to_f64(self.clock.now());
        let delta_from_now = mcu_host_instant - now_instant;
        let wall_now = std::time::SystemTime::now();
        let wall_time = if delta_from_now >= 0.0 {
            wall_now
                .checked_add(std::time::Duration::from_secs_f64(delta_from_now))
                .unwrap_or(wall_now)
        } else {
            wall_now
                .checked_sub(std::time::Duration::from_secs_f64(-delta_from_now))
                .unwrap_or(wall_now)
        };
        let estimated = delta_ticks.abs() / rec.clock_freq > 1.0;
        Some((time::OffsetDateTime::from(wall_time), estimated))
    }

    pub fn ack_clock_and_freq(&self, mcu: McuHandle) -> Option<(u64, f64)> {
        let rec = self.mcus.get(&mcu)?.est?;
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Some((projected, rec.clock_freq))
    }

    pub fn compute_ack_clock(&self, mcu: McuHandle) -> Result<u64, RouterError> {
        let rec = self
            .mcus
            .get(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?
            .est(mcu)?;
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Ok(projected)
    }

    pub fn host_now_secs(&self) -> f64 {
        instant_to_f64(self.clock.now())
    }

    /// The scheduling timeline (`print_time ≡ clock / nominal_freq`) at a
    /// host instant, from the reference MCU's record: the regression
    /// extrapolates what the clock reads at `host`, the nominal frequency
    /// names that tick count in print_time seconds. Only the MCU whose clock
    /// defines the timeline (the primary) gives a meaningful answer. `None`
    /// until both a clock estimate and the nominal frequency are set.
    pub fn print_time_at_host(
        &self,
        reference_mcu: McuHandle,
        host: HostSecs,
    ) -> Option<PrintTime> {
        let rec = self.mcus.get(&reference_mcu)?;
        let nominal_freq = rec.nominal_freq;
        let rec = rec.est?;
        if rec.clock_freq <= 0.0 || nominal_freq <= 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let clock = (rec.last_clock as f64) + (host.get() - rec.clock_offset) * rec.clock_freq;
        Some(PrintTime::new(clock / nominal_freq))
    }

    /// [`Self::print_time_at_host`] at this instant, from one clock read.
    pub fn print_time_now(&self, reference_mcu: McuHandle) -> Option<PrintTime> {
        self.print_time_at_host(reference_mcu, HostSecs::from_instant(self.clock.now()))
    }

    pub fn clock_to_host_secs(&self, mcu: McuHandle, mcu_clock: u64) -> Option<f64> {
        let rec = self.mcus.get(&mcu)?.est?;
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_clock as f64) - (rec.last_clock as f64);
        Some(rec.clock_offset + delta_ticks / rec.clock_freq)
    }

    /// Inverse of [`Self::print_time_at_host`]: `print_time` names a tick
    /// count via the nominal frequency; the regression places that tick on
    /// the host clock.
    pub fn print_time_to_host_secs(
        &self,
        reference_mcu: McuHandle,
        print_time: f64,
    ) -> Option<f64> {
        let rec = self.mcus.get(&reference_mcu)?;
        let nominal_freq = rec.nominal_freq;
        let rec = rec.est?;
        if rec.clock_freq <= 0.0 || nominal_freq <= 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let clock = print_time * nominal_freq;
        Some(rec.clock_offset + (clock - rec.last_clock as f64) / rec.clock_freq)
    }

    pub fn host_time_to_mcu_clock(
        &self,
        mcu: McuHandle,
        host_time_secs: f64,
    ) -> Result<u64, RouterError> {
        let rec = self
            .mcus
            .get(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?
            .est(mcu)?;
        let delta = (host_time_secs - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        tracing::trace!(
            subsystem = "motion",
            event = "host_time_to_mcu_clock",
            mcu = ?mcu,
            host_time_secs,
            clock_offset = rec.clock_offset,
            last_clock = rec.last_clock,
            clock_freq = rec.clock_freq,
            result_ns = projected,
            "[project] host_time_to_mcu_clock"
        );
        Ok(projected)
    }

    pub fn log_seg0_lead(&self, mcu: McuHandle, seg0_host_secs: f64, t0: f64) {
        let rec = match self.mcus.get(&mcu) {
            Some(r) => r,
            None => {
                tracing::warn!(
                    subsystem = "motion",
                    event = "seg0_lead_unknown_mcu",
                    mcu = ?mcu,
                    "[seg0-lead] UNKNOWN mcu"
                );
                return;
            }
        };
        let Some(rec) = rec.est else {
            tracing::warn!(
                subsystem = "motion",
                event = "seg0_lead_not_synced",
                mcu = ?mcu,
                t0,
                seg0_host_secs,
                "[seg0-lead] no clock record (not yet synced)"
            );
            return;
        };
        let delta = (seg0_host_secs - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let start_time = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        let ack_now = self.compute_ack_clock(mcu).unwrap_or(0);
        let lead_ticks = start_time as i64 - ack_now as i64;
        let lead_us = (lead_ticks as f64 / rec.clock_freq) * 1e6;
        if lead_ticks < 0 {
            tracing::warn!(
                subsystem = "motion",
                event = "seg0_start_in_past",
                mcu = ?mcu,
                freq = rec.clock_freq,
                offset = rec.clock_offset,
                last_clock = rec.last_clock,
                t0,
                seg0_host_secs,
                start_time,
                ack_now,
                lead_ticks,
                lead_us,
                "[seg0-lead] segment 0 starts behind the MCU ack clock (in the past)"
            );
        } else {
            tracing::debug!(
                subsystem = "motion",
                event = "seg0_lead",
                mcu = ?mcu,
                freq = rec.clock_freq,
                offset = rec.clock_offset,
                last_clock = rec.last_clock,
                t0,
                seg0_host_secs,
                start_time,
                ack_now,
                lead_ticks,
                lead_us,
                "[seg0-lead] segment 0 lead over the MCU ack clock"
            );
        }
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod clock_record_lifecycle_tests;

#[cfg(test)]
mod clock_record_freshness_tests;
