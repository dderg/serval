use std::sync::Arc;
use std::time::Instant;

use indexmap::IndexMap;

use crate::clock::{Clock, instant_to_f64};

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
}

impl std::fmt::Display for RouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownMcu(h) => write!(f, "unknown MCU handle {}", h.0),
        }
    }
}

impl std::error::Error for RouterError {}

#[derive(Debug)]
struct McuRecord {
    clock_freq: f64,
    clock_offset: f64,
    last_clock: u64,
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
                clock_freq: 0.0,
                clock_offset: 0.0,
                last_clock: 0,
            },
        );
        handle
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
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = offset;
        rec.last_clock = last_clock;
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
        _host_now_raw: f64,
    ) -> Result<(), RouterError> {
        let bridge_now_instant = instant_to_f64(self.clock.now());
        let bridge_now_raw = crate::clock::monotonic_raw_secs();
        let clock_offset = offset_raw - (bridge_now_raw - bridge_now_instant);
        tracing::info!(
            subsystem = "clocksync",
            event = "set_clock_est_rebased",
            mcu = ?mcu,
            freq,
            offset_raw,
            bridge_now_raw,
            bridge_now_instant,
            clock_offset,
            last_clock,
            "[clock-seed] set_clock_est_rebased"
        );
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = clock_offset;
        rec.last_clock = last_clock;
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
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = clock_offset;
        rec.last_clock = mcu_at_send;
        Ok(())
    }

    /// Convert an MCU tick count to a wall-clock `OffsetDateTime`.
    ///
    /// Returns `None` when no clock record has been set for this MCU
    /// (i.e. `clock_freq == 0.0` — no `set_clock_est_rebased` call yet).
    ///
    /// `estimated = true` when the tick is more than one frequency-second from
    /// the anchor, i.e. significant extrapolation.
    pub fn wall_time_at_mcu(
        &self,
        mcu: McuHandle,
        mcu_ticks: u64,
    ) -> Option<(time::OffsetDateTime, bool)> {
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
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
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Some((projected, rec.clock_freq))
    }

    pub fn compute_ack_clock(&self, mcu: McuHandle) -> Result<u64, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        if rec.clock_freq == 0.0 {
            return Ok(0);
        }
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Ok(projected)
    }

    pub fn host_now_secs(&self) -> f64 {
        instant_to_f64(self.clock.now())
    }

    pub fn clock_to_host_secs(&self, mcu: McuHandle, mcu_clock: u64) -> Option<f64> {
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_clock as f64) - (rec.last_clock as f64);
        Some(rec.clock_offset + delta_ticks / rec.clock_freq)
    }

    pub fn print_time_to_host_secs(
        &self,
        reference_mcu: McuHandle,
        print_time: f64,
    ) -> Option<f64> {
        let rec = self.mcus.get(&reference_mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        Some(rec.clock_offset + print_time - (rec.last_clock as f64) / rec.clock_freq)
    }

    pub fn host_time_to_mcu_clock(
        &self,
        mcu: McuHandle,
        host_time_secs: f64,
    ) -> Result<u64, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        if rec.clock_freq == 0.0 {
            return Ok(0);
        }
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
        if rec.clock_freq == 0.0 {
            tracing::warn!(
                subsystem = "motion",
                event = "seg0_lead_not_synced",
                mcu = ?mcu,
                t0,
                seg0_host_secs,
                "[seg0-lead] clock_freq=0 (not yet synced)"
            );
            return;
        }
        let start_time = self
            .host_time_to_mcu_clock(mcu, seg0_host_secs)
            .unwrap_or(0);
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
