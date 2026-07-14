use std::collections::{HashMap, VecDeque};

use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use runtime::piece_ring::{MAX_PIECE_COEFFS, PieceEntry};

use crate::kinematics::{KinematicsKind, KinematicsModule};
use crate::types::AxisKey;

pub const HISTORY_CAPACITY: usize = 4096;

pub const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

/// Position-domain tolerance (mm) for treating a piece as a held rest: every
/// Chebyshev coefficient above the constant term must fall within it.
const REST_COEFF_EPS: f64 = 1e-6;

#[derive(Debug, thiserror::Error)]
pub enum HistoryError {
    #[error(
        "query host time {queried:.6}s precedes retained motion history for axis \
         {key:?} (window {window_start:.6}..{window_end:.6}s, {ring_len} pieces \
         retained, {evicted} evicted, first piece {first_dur_s:.6}s)"
    )]
    BeforeRetainedWindow {
        key: AxisKey,
        queried: f64,
        window_start: f64,
        window_end: f64,
        ring_len: usize,
        evicted: u64,
        first_dur_s: f64,
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
    pub coeff_count: u8,
    pub coeffs: [f32; MAX_PIECE_COEFFS],
}

impl HistoryPiece {
    pub fn from_entry(entry: &PieceEntry, nominal_freq_hz: u32, host_secs: f64) -> Self {
        let end_clock = entry.end_time(nominal_freq_hz as f32);
        Self {
            start_host: host_secs,
            start_clock: entry.start_time,
            end_clock,
            duration_secs: entry.duration,
            coeff_count: entry.coeff_count,
            coeffs: entry.coeffs,
        }
    }

    fn end_host(&self) -> f64 {
        self.start_host + f64::from(self.duration_secs)
    }

    fn live_coeffs(&self) -> &[f32] {
        let n = (self.coeff_count as usize).clamp(1, MAX_PIECE_COEFFS);
        &self.coeffs[..n]
    }

    fn end_position(&self) -> f64 {
        self.live_coeffs().iter().map(|&a| f64::from(a)).sum()
    }

    fn endpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            host: self.end_host(),
            position: self.end_position(),
        }
    }

    fn is_rest_at(&self, position: f64) -> bool {
        let coeffs = self.live_coeffs();
        let constant = coeffs[1..]
            .iter()
            .all(|&c| f64::from(c).abs() <= REST_COEFF_EPS);
        constant && (self.end_position() - position).abs() <= REST_COEFF_EPS
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisState {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

/// `motor_state[i]` is the retained-history answer for motor axis `i`
/// (0=first CoreXY/Cartesian motor, 1=second, 2=Z, 3=extruder) — the
/// lowerer's output frame, e.g. CoreXY A/B, not cartesian X/Y. Inverts
/// through `kin` to the cartesian axes named in `AXIS_NAMES`, mirroring
/// `position_query::assemble_cartesian`'s live-query counterpart. Z and E
/// pass straight through under every kinematics tag defined today (both
/// `COREXY_MOTOR_TO_AXIS` and the cartesian identity have an identity Z
/// row), independent of whether X/Y resolved. A coupled cartesian axis is
/// omitted rather than computed from a missing motor as zero.
pub fn assemble_cartesian_state(
    motor_state: [Option<AxisState>; 4],
    kin: &KinematicsModule,
) -> HashMap<String, (f64, f64, f64)> {
    let mut out = HashMap::new();
    if let Some(e) = motor_state[3] {
        out.insert(
            AXIS_NAMES[3].to_string(),
            (e.position, e.velocity, e.acceleration),
        );
    }
    if let Some(z) = motor_state[2] {
        out.insert(
            AXIS_NAMES[2].to_string(),
            (z.position, z.velocity, z.acceleration),
        );
    }
    match kin.kind() {
        KinematicsKind::Cartesian => {
            for (axis, name) in AXIS_NAMES.iter().enumerate().take(2) {
                if let Some(st) = motor_state[axis] {
                    out.insert(
                        (*name).to_string(),
                        (st.position, st.velocity, st.acceleration),
                    );
                }
            }
        }
        KinematicsKind::CoreXy => {
            if let (Some(m0), Some(m1)) = (motor_state[0], motor_state[1]) {
                let pos = kin.inverse([m0.position, m1.position, 0.0]);
                let vel = kin.inverse([m0.velocity, m1.velocity, 0.0]);
                let accel = kin.inverse([m0.acceleration, m1.acceleration, 0.0]);
                out.insert(AXIS_NAMES[0].to_string(), (pos[0], vel[0], accel[0]));
                out.insert(AXIS_NAMES[1].to_string(), (pos[1], vel[1], accel[1]));
            }
        }
    }
    out
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

/// The rest an axis provably held before a restarted ring: pieces are the only
/// way an axis moves, so the span `[from, until]` answers with the endpoint
/// position. `from` is the start of the trailing run of rest pieces preceding
/// the drop — not the last piece's scheduled end — so a dwell that straddled
/// the re-anchor stays answerable; anything earlier was real motion and must
/// fail. Kept separate from the ring because capacity eviction moves the ring's
/// front past `until`, and queries in that evicted gap must still fail.
#[derive(Debug, Clone, Copy)]
struct HoldBeforeRing {
    endpoint: AxisEndpoint,
    from: f64,
    until: f64,
}

/// f64 Clenshaw over the Chebyshev series at `cu ∈ [−1, 1]`.
#[inline]
fn clenshaw_f64<I: DoubleEndedIterator<Item = f64>>(coeffs: I, cu: f64) -> f64 {
    let mut coeffs = coeffs;
    let Some(a0) = coeffs.next() else {
        return 0.0;
    };
    let mut b1 = 0.0_f64;
    let mut b2 = 0.0_f64;
    for ak in coeffs.rev() {
        let b0 = ak + 2.0 * cu * b1 - b2;
        b2 = b1;
        b1 = b0;
    }
    a0 + cu * b1 - b2
}

#[inline]
pub fn eval_chebyshev(coeffs: &[f32], cu: f64) -> f64 {
    clenshaw_f64(coeffs.iter().map(|&c| f64::from(c)), cu)
}

fn chebyshev_derivative(a: &[f64]) -> Vec<f64> {
    let n = a.len();
    if n <= 1 {
        return vec![0.0];
    }
    let mut d = vec![0.0; n - 1];
    d[n - 2] = 2.0 * (n - 1) as f64 * a[n - 1];
    for j in (0..n.saturating_sub(2)).rev() {
        let d_j2 = d.get(j + 2).copied().unwrap_or(0.0);
        d[j] = d_j2 + 2.0 * (j + 1) as f64 * a[j + 1];
    }
    d[0] *= 0.5;
    d
}

fn eval_at_u(piece: &HistoryPiece, u: f64) -> AxisState {
    let cu = 2.0 * u - 1.0;
    let a: Vec<f64> = piece.live_coeffs().iter().map(|&c| f64::from(c)).collect();
    let t = f64::from(piece.duration_secs);
    let position = eval_chebyshev(piece.live_coeffs(), cu);
    let (velocity, acceleration) = if t > 0.0 {
        let du_dt = 2.0 / t;
        let dv = chebyshev_derivative(&a);
        let da = chebyshev_derivative(&dv);
        (
            clenshaw_f64(dv.iter().copied(), cu) * du_dt,
            clenshaw_f64(da.iter().copied(), cu) * du_dt * du_dt,
        )
    } else {
        (0.0, 0.0)
    };
    AxisState {
        position,
        velocity,
        acceleration,
    }
}

fn eval_state(piece: &HistoryPiece, host_t: f64) -> AxisState {
    let span = f64::from(piece.duration_secs);
    let u = if span > 0.0 {
        ((host_t - piece.start_host) / span).clamp(0.0, 1.0)
    } else {
        0.0
    };
    eval_at_u(piece, u)
}

fn eval_state_at_clock(piece: &HistoryPiece, clock: u64) -> AxisState {
    let span = piece.end_clock.saturating_sub(piece.start_clock);
    let u = if span > 0 {
        (clock.saturating_sub(piece.start_clock) as f64 / span as f64).clamp(0.0, 1.0)
    } else {
        0.0
    };
    eval_at_u(piece, u)
}

fn trailing_rest_start(ring: &VecDeque<HistoryPiece>, endpoint: AxisEndpoint) -> f64 {
    let mut start = endpoint.host;
    for piece in ring.iter().rev() {
        if !piece.is_rest_at(endpoint.position) {
            break;
        }
        start = piece.start_host;
    }
    start
}

#[derive(Debug, Default)]
pub struct HistoryStore {
    rings: HashMap<AxisKey, VecDeque<HistoryPiece>>,
    endpoints: HashMap<AxisKey, AxisEndpoint>,
    evicted: HashMap<AxisKey, u64>,
    holds_before_ring: HashMap<AxisKey, HoldBeforeRing>,
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
        if ring.is_empty() {
            if let Some(hold) = self.holds_before_ring.get_mut(&key) {
                if piece.start_host < hold.endpoint.host {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "history_hold_rewound",
                        mcu = key.mcu_id,
                        axis = key.axis,
                        start_host = piece.start_host,
                        endpoint_host = hold.endpoint.host,
                        "[history] first piece after re-anchor precedes the held endpoint — clamping hold coverage"
                    );
                }
                hold.until = piece.start_host;
            } else if let Some(endpoint) = self.endpoints.get(&key).copied() {
                if endpoint.host <= piece.start_host {
                    self.holds_before_ring.insert(
                        key,
                        HoldBeforeRing {
                            endpoint,
                            from: endpoint.host,
                            until: piece.start_host,
                        },
                    );
                }
            }
        }
        let prev = ring.back().map(|p| (p.start_clock, p.start_host));
        if let Some((last_clock, last_host)) = prev {
            if piece.start_clock < last_clock {
                let regress_ticks = last_clock - piece.start_clock;
                let regress_us = regress_ticks as f64 * 1.0e6 / f64::from(nominal_freq_hz);
                let host_delta_us = (piece.start_host - last_host) * 1.0e6;
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
            if piece.start_host < last_host {
                tracing::warn!(
                    subsystem = "motion",
                    event = "history_host_out_of_order",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    start_host = piece.start_host,
                    last_start_host = last_host,
                    "[history] host schedule time regressed vs previous piece — superseding stale tail"
                );
                while ring.back().is_some_and(|p| p.start_host > piece.start_host) {
                    ring.pop_back();
                }
            }
        }
        if ring.len() == HISTORY_CAPACITY {
            ring.pop_front();
            *self.evicted.entry(key).or_default() += 1;
        }
        self.endpoints.insert(key, piece.endpoint());
        ring.push_back(piece);
    }

    /// Endpoints are kept, so an axis the re-anchored segment does not re-record
    /// (e.g. X/Y during a Z-only probe move) still answers `state_at_host` with
    /// its held position instead of `NoHistoryForAxis` — which the beacon probe
    /// position lookup depends on.
    pub fn drop_pieces_on_reanchor(&mut self) {
        let dropped: usize = self.rings.values().map(VecDeque::len).sum();
        tracing::info!(
            subsystem = "motion",
            event = "history_drop_on_reanchor",
            dropped,
            "[history] stream re-anchored — dropped retained pieces, endpoints held"
        );
        for (key, ring) in self.rings.iter_mut() {
            if ring.is_empty() {
                continue;
            }
            if let Some(endpoint) = self.endpoints.get(key).copied() {
                self.holds_before_ring.insert(
                    *key,
                    HoldBeforeRing {
                        endpoint,
                        from: trailing_rest_start(ring, endpoint),
                        until: f64::INFINITY,
                    },
                );
            }
            ring.clear();
        }
    }

    pub fn rebase_axis(&mut self, key: AxisKey, host: f64, position: f64) {
        tracing::info!(
            subsystem = "motion",
            event = "history_rebase_axis",
            mcu = key.mcu_id,
            axis = key.axis,
            host,
            position,
            "[history] axis rebased to an externally set position"
        );
        self.rings.entry(key).or_default().clear();
        self.holds_before_ring.remove(&key);
        self.endpoints.insert(key, AxisEndpoint { host, position });
    }

    pub fn final_position(&self, key: AxisKey) -> Option<f64> {
        self.endpoints.get(&key).map(|e| e.position)
    }

    /// Axis state at an MCU clock reading from the same MCU the pieces were
    /// sent to. Pieces execute at exactly their wire start clock, so
    /// evaluating by clock is exact where `state_at_host` goes through the
    /// clock↔host mapping twice (once keying the piece at send, once
    /// converting the query) and inherits the sync estimate's jitter between
    /// those two moments — an error that scales with axis velocity and, in
    /// the simulator, with `VTIME_SPEED`. `host_t` is the clock's host-time
    /// projection, used only for the hold fallbacks, where the position is
    /// constant and mapping jitter cannot bias it.
    pub fn state_at_clock(
        &self,
        key: AxisKey,
        clock: u64,
        host_t: f64,
        now_host: Option<f64>,
    ) -> Result<AxisState, HistoryError> {
        let Some(ring) = self.rings.get(&key).filter(|r| !r.is_empty()) else {
            return self.state_at_host(key, host_t, now_host);
        };
        // Reverse scan instead of binary search: recorded start clocks are
        // projections and may regress a few ticks across re-syncs
        // (`history_order_jitter`), which would break a sorted-ring
        // precondition. Trip queries are rare, so O(len) is fine.
        let Some(piece) = ring.iter().rev().find(|p| p.start_clock <= clock) else {
            return self.state_at_host(key, host_t, now_host);
        };
        if clock < piece.end_clock {
            return Ok(eval_state_at_clock(piece, clock));
        }
        if let Some(now_host) = now_host {
            if host_t > now_host {
                return Err(HistoryError::QueryInFuture {
                    key,
                    queried: host_t,
                    now_host,
                });
            }
        }
        Ok(piece.endpoint().hold_state())
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
                    let held_rest_before_ring = self
                        .holds_before_ring
                        .get(&key)
                        .filter(|hold| hold.from <= host_t && host_t <= hold.until);
                    if let Some(hold) = held_rest_before_ring {
                        return Ok(hold.endpoint.hold_state());
                    }
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: host_t,
                        window_start: ring.front().map_or(0.0, |p| p.start_host),
                        window_end: ring.back().map_or(0.0, |p| p.end_host()),
                        ring_len: ring.len(),
                        evicted: self.evicted.get(&key).copied().unwrap_or(0),
                        first_dur_s: ring.front().map_or(0.0, |p| f64::from(p.duration_secs)),
                    });
                }
                let piece = &ring[idx - 1];
                if host_t < piece.end_host() {
                    return Ok(eval_state(piece, host_t));
                }
                piece.endpoint()
            }
            None => {
                let endpoint = *self
                    .endpoints
                    .get(&key)
                    .ok_or(HistoryError::NoHistoryForAxis(key))?;
                if let Some(hold) = self.holds_before_ring.get(&key) {
                    if host_t < hold.from {
                        return Err(HistoryError::BeforeRetainedWindow {
                            key,
                            queried: host_t,
                            window_start: hold.from,
                            window_end: hold.endpoint.host,
                            ring_len: 0,
                            evicted: self.evicted.get(&key).copied().unwrap_or(0),
                            first_dur_s: 0.0,
                        });
                    }
                }
                endpoint
            }
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

#[cfg(test)]
pub(crate) fn clock_between_mcus(
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
