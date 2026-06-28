use std::collections::{HashMap, VecDeque};

use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use runtime::piece_ring::PieceEntry;

use crate::pump::AxisKey;

pub const HISTORY_CAPACITY: usize = 4096;

#[derive(Debug, thiserror::Error)]
pub enum HistoryError {
    #[error(
        "query host time {queried:.6}s precedes retained motion history for axis \
         {key:?} (window {window_start:.6}..{window_end:.6}s)"
    )]
    BeforeRetainedWindow {
        key: AxisKey,
        queried: f64,
        window_start: f64,
        window_end: f64,
    },

    #[error(
        "query host time {queried:.6}s is in the future for axis {key:?} \
         (now≈{now_host:.6}s) — motion history answers the past only"
    )]
    QueryInFuture {
        key: AxisKey,
        queried: f64,
        now_host: f64,
    },

    #[error("non-finite query host time for axis {key:?}: {queried}")]
    NonFiniteQuery { key: AxisKey, queried: f64 },

    #[error("no motion history recorded for axis {0:?}")]
    NoHistoryForAxis(AxisKey),
}

#[derive(Debug, Clone, Copy)]
pub struct HistoryPiece {
    pub start_host: f64,
    pub start_clock: u64,
    pub end_clock: u64,
    pub duration_secs: f32,
    pub coeffs: [f32; 4],
}

impl HistoryPiece {
    pub fn from_entry(entry: &PieceEntry, nominal_freq_hz: u32, host_secs: f64) -> Self {
        let end_clock = entry.end_time(nominal_freq_hz as f32);
        Self {
            start_host: host_secs,
            start_clock: entry.start_time,
            end_clock,
            duration_secs: entry.duration,
            coeffs: entry.coeffs,
        }
    }

    fn end_host(&self) -> f64 {
        self.start_host + f64::from(self.duration_secs)
    }

    fn endpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            host: self.end_host(),
            position: f64::from(self.coeffs[3]),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisState {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy)]
struct AxisEndpoint {
    host: f64,
    position: f64,
}

impl AxisEndpoint {
    fn hold_state(&self) -> AxisState {
        AxisState {
            position: self.position,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }
}

#[inline]
pub fn eval_bernstein_cubic(coeffs: [f32; 4], u: f64) -> f64 {
    let v = 1.0 - u;
    let b0 = f64::from(coeffs[0]);
    let b1 = f64::from(coeffs[1]);
    let b2 = f64::from(coeffs[2]);
    let b3 = f64::from(coeffs[3]);
    v * v * v * b0 + 3.0 * v * v * u * b1 + 3.0 * v * u * u * b2 + u * u * u * b3
}

fn eval_state(piece: &HistoryPiece, host_t: f64) -> AxisState {
    let span = f64::from(piece.duration_secs);
    let u = if span > 0.0 {
        ((host_t - piece.start_host) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let v = 1.0 - u;
    let b0 = f64::from(piece.coeffs[0]);
    let b1 = f64::from(piece.coeffs[1]);
    let b2 = f64::from(piece.coeffs[2]);
    let b3 = f64::from(piece.coeffs[3]);
    let t = span;
    let (velocity, acceleration) = if t > 0.0 {
        let db = 3.0 * ((b1 - b0) * v * v + 2.0 * (b2 - b1) * v * u + (b3 - b2) * u * u);
        let d2b = 6.0 * ((b2 - 2.0 * b1 + b0) * v + (b3 - 2.0 * b2 + b1) * u);
        (db / t, d2b / (t * t))
    } else {
        (0.0, 0.0)
    };
    AxisState {
        position: eval_bernstein_cubic(piece.coeffs, u),
        velocity,
        acceleration,
    }
}

#[derive(Debug, Default)]
pub struct HistoryStore {
    rings: HashMap<AxisKey, VecDeque<HistoryPiece>>,
    endpoints: HashMap<AxisKey, AxisEndpoint>,
}

impl HistoryStore {
    pub fn record(
        &mut self,
        key: AxisKey,
        entry: &PieceEntry,
        nominal_freq_hz: u32,
        host_secs: f64,
    ) {
        if !host_secs.is_finite() {
            tracing::error!(
                subsystem = "motion",
                event = "history_non_finite_host",
                mcu = key.mcu_id,
                axis = key.axis,
                start_clock = entry.start_time,
                "[history] non-finite host time for piece — skipping record"
            );
            return;
        }
        let piece = HistoryPiece::from_entry(entry, nominal_freq_hz, host_secs);
        let ring = self.rings.entry(key).or_default();
        if let Some(last) = ring.back() {
            if piece.start_clock < last.start_clock {
                let regress_ticks = last.start_clock - piece.start_clock;
                let regress_us = regress_ticks as f64 * 1.0e6 / f64::from(nominal_freq_hz);
                let host_delta_us = (piece.start_host - last.start_host) * 1.0e6;
                tracing::warn!(
                    subsystem = "motion",
                    event = "history_order_jitter",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    regress_ticks,
                    regress_us,
                    host_delta_us,
                    "[history-jitter] projected MCU tick regressed; host schedule time delta"
                );
            }
            if piece.start_host < last.start_host {
                tracing::warn!(
                    subsystem = "motion",
                    event = "history_host_out_of_order",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    start_host = piece.start_host,
                    last_start_host = last.start_host,
                    "[history] host schedule time regressed vs previous piece — late segment / ordering bug"
                );
            }
        }
        if ring.len() == HISTORY_CAPACITY {
            ring.pop_front();
        }
        self.endpoints.insert(key, piece.endpoint());
        ring.push_back(piece);
    }

    /// Endpoints are kept, so an axis the re-anchored segment does not re-record
    /// (e.g. X/Y during a Z-only probe move) still answers `state_at_host` with
    /// its held position instead of `NoHistoryForAxis` — which the beacon probe
    /// position lookup depends on.
    pub fn drop_pieces_on_reanchor(&mut self) {
        for ring in self.rings.values_mut() {
            ring.clear();
        }
    }

    pub fn rebase_axis(&mut self, key: AxisKey, host: f64, position: f64) {
        self.rings.entry(key).or_default().clear();
        self.endpoints.insert(key, AxisEndpoint { host, position });
    }

    pub fn last_endpoint_host(&self, key: AxisKey) -> f64 {
        self.endpoints.get(&key).map_or(0.0, |e| e.host)
    }

    pub fn final_position(&self, key: AxisKey) -> Option<f64> {
        self.endpoints.get(&key).map(|e| e.position)
    }

    pub fn state_at_host(
        &self,
        key: AxisKey,
        host_t: f64,
        now_host: Option<f64>,
    ) -> Result<AxisState, HistoryError> {
        if !host_t.is_finite() {
            return Err(HistoryError::NonFiniteQuery {
                key,
                queried: host_t,
            });
        }
        let ring = self.rings.get(&key).filter(|r| !r.is_empty());
        let hold = match ring {
            Some(ring) => {
                let idx = ring.partition_point(|p| p.start_host <= host_t);
                if idx == 0 {
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: host_t,
                        window_start: ring.front().map_or(0.0, |p| p.start_host),
                        window_end: ring.back().map_or(0.0, |p| p.end_host()),
                    });
                }
                let piece = &ring[idx - 1];
                if host_t < piece.end_host() {
                    return Ok(eval_state(piece, host_t));
                }
                piece.endpoint()
            }
            None => *self
                .endpoints
                .get(&key)
                .ok_or(HistoryError::NoHistoryForAxis(key))?,
        };
        if let Some(now_host) = now_host {
            if host_t > now_host {
                return Err(HistoryError::QueryInFuture {
                    key,
                    queried: host_t,
                    now_host,
                });
            }
        }
        Ok(hold.hold_state())
    }
}

pub fn clock_to_host(
    router: &PassthroughRouter,
    source: McuHandle,
    clock: u64,
) -> Result<f64, String> {
    router
        .clock_to_host_secs(source, clock)
        .ok_or_else(|| format!("clock_to_host_secs returned None for source mcu {source:?}"))
}

pub fn clock_between_mcus(
    router: &PassthroughRouter,
    source: McuHandle,
    target: McuHandle,
    clock: u64,
) -> Result<u64, String> {
    if source == target {
        return Ok(clock);
    }
    let host_secs = clock_to_host(router, source, clock)?;
    router
        .host_time_to_mcu_clock(target, host_secs)
        .map_err(|e| format!("host_time_to_mcu_clock failed for target mcu {target:?}: {e:?}"))
}

#[cfg(test)]
mod tests;
