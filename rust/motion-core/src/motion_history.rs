use std::collections::{HashMap, VecDeque};

use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use trajectory::{ClockedMotorSpan, ContinuousError, Pva};

use crate::kinematics::{KinematicsKind, KinematicsModule};
use crate::types::AxisKey;

pub const HISTORY_CAPACITY: usize = 4096;

pub const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

#[derive(Debug, thiserror::Error)]
pub enum HistoryError {
    #[error(
        "query host time {queried:.6}s precedes retained motion history for axis \
         {key:?} (window {window_start:.6}..{window_end:.6}s, {ring_len} spans \
         retained, {evicted} evicted, first span {first_dur_s:.6}s)"
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

    #[error("continuous evaluation failed for axis {key:?}")]
    Evaluation {
        key: AxisKey,
        #[source]
        source: ContinuousError,
    },
}

#[derive(Debug, Clone)]
pub struct HistorySpan {
    pub span: ClockedMotorSpan,
    start_position: f64,
    end_position: f64,
}

impl HistorySpan {
    pub fn try_new(span: ClockedMotorSpan) -> Result<Self, ContinuousError> {
        let start_position = span.signal.position(span.stream_t_start)?;
        let end_position = span.signal.position(span.stream_t_end)?;
        Ok(Self {
            span,
            start_position,
            end_position,
        })
    }

    fn start_host(&self) -> f64 {
        self.span.start_host
    }

    fn end_host(&self) -> f64 {
        self.span.end_host
    }

    fn duration_secs(&self) -> f64 {
        self.span.end_host - self.span.start_host
    }

    fn endpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            host: self.end_host(),
            position: self.end_position,
        }
    }

    fn startpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            host: self.start_host(),
            position: self.start_position,
        }
    }

    fn is_rest_at(&self, position: f64) -> bool {
        self.span.signal.is_explicit_hold
            && self.start_position.to_bits() == self.end_position.to_bits()
            && self.end_position.to_bits() == position.to_bits()
    }

    /// The host anchors carry the clock↔host estimate captured when the view
    /// was dispatched; interpolating between them reproduces `eval_at_clock`'s
    /// affine map with the host-domain skew that map had at send time.
    fn stream_time_at_host(&self, host_t: f64) -> f64 {
        let host_span = self.end_host() - self.start_host();
        if host_span <= 0.0 {
            return self.span.stream_t_start;
        }
        let fraction = ((host_t - self.start_host()) / host_span).clamp(0.0, 1.0);
        self.span.stream_t_start + fraction * (self.span.stream_t_end - self.span.stream_t_start)
    }

    fn state_at_host(&self, host_t: f64) -> Result<AxisState, ContinuousError> {
        self.span
            .signal
            .eval_pva(self.stream_time_at_host(host_t))
            .map(axis_state)
    }

    fn state_at_clock(&self, clock: u64) -> Result<AxisState, ContinuousError> {
        self.span.eval_at_clock(clock).map(axis_state)
    }
}

fn axis_state(value: Pva) -> AxisState {
    AxisState {
        position: value.position,
        velocity: value.velocity,
        acceleration: value.acceleration,
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

/// The rest an axis provably held before a restarted ring: spans are the only
/// way an axis moves, so the interval `[from, until]` answers with the endpoint
/// position. `from` is the start of the trailing run of explicit-hold spans
/// preceding the drop — not the last span's scheduled end — so a dwell that
/// straddled the re-anchor stays answerable; anything earlier was real motion
/// and must fail. Kept separate from the ring because capacity eviction moves
/// the ring's front past `until`, and queries in that evicted gap must still
/// fail.
#[derive(Debug, Clone, Copy)]
struct HoldBeforeRing {
    endpoint: AxisEndpoint,
    from: f64,
    until: f64,
}

fn trailing_rest_start(ring: &VecDeque<HistorySpan>, endpoint: AxisEndpoint) -> f64 {
    let mut start = endpoint.host;
    for span in ring.iter().rev() {
        if !span.is_rest_at(endpoint.position) {
            break;
        }
        start = span.start_host();
    }
    start
}

#[derive(Debug, Default)]
pub struct HistoryStore {
    rings: HashMap<AxisKey, VecDeque<HistorySpan>>,
    endpoints: HashMap<AxisKey, AxisEndpoint>,
    evicted: HashMap<AxisKey, u64>,
    holds_before_ring: HashMap<AxisKey, HoldBeforeRing>,
    rebase_boundaries: HashMap<AxisKey, VecDeque<AxisEndpoint>>,
}

impl HistoryStore {
    fn rebase_boundary_at(&self, key: AxisKey, host_t: f64) -> Option<AxisEndpoint> {
        self.rebase_boundaries
            .get(&key)?
            .iter()
            .rev()
            .find(|boundary| boundary.host <= host_t)
            .copied()
    }
    /// Records one absolute main-trajectory view at the moment its endpoint
    /// takes ownership. Base-relative nudge and buzz overlays are never
    /// recorded: logical lane history stays in absolute coordinates.
    pub fn record(&mut self, key: AxisKey, span: ClockedMotorSpan) -> Result<(), HistoryError> {
        let clock_freq_hz = span.clock_freq_hz;
        let recorded = HistorySpan::try_new(span)
            .map_err(|source| HistoryError::Evaluation { key, source })?;
        let start_host = recorded.start_host();
        let start_clock = recorded.span.start_clock;
        let ring = self.rings.entry(key).or_default();
        if ring.is_empty() {
            if let Some(hold) = self.holds_before_ring.get_mut(&key) {
                if start_host < hold.endpoint.host {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "history_hold_rewound",
                        mcu = key.mcu_id,
                        axis = key.axis,
                        start_host,
                        endpoint_host = hold.endpoint.host,
                        "[history] first span after re-anchor precedes the held endpoint — clamping hold coverage"
                    );
                }
                hold.until = start_host;
            } else if let Some(endpoint) = self.endpoints.get(&key).copied() {
                if endpoint.host <= start_host {
                    self.holds_before_ring.insert(
                        key,
                        HoldBeforeRing {
                            endpoint,
                            from: endpoint.host,
                            until: start_host,
                        },
                    );
                }
            }
        }
        let prev = ring.back().map(|s| (s.span.start_clock, s.start_host()));
        if let Some((last_clock, last_host)) = prev {
            if start_clock < last_clock {
                let regress_ticks = last_clock - start_clock;
                let regress_us = regress_ticks as f64 * 1.0e6 / clock_freq_hz;
                let host_delta_us = (start_host - last_host) * 1.0e6;
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
            if start_host < last_host {
                tracing::warn!(
                    subsystem = "motion",
                    event = "history_host_out_of_order",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    start_host,
                    last_start_host = last_host,
                    "[history] host schedule time regressed vs previous span — superseding stale tail"
                );
                while ring.back().is_some_and(|s| s.start_host() > start_host) {
                    ring.pop_back();
                }
            }
        }
        if ring.len() == HISTORY_CAPACITY {
            ring.pop_front();
            *self.evicted.entry(key).or_default() += 1;
        }
        self.endpoints.insert(key, recorded.endpoint());
        ring.push_back(recorded);
        Ok(())
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
            "[history] stream re-anchored — dropped retained spans, endpoints held"
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
        self.rings
            .entry(key)
            .or_default()
            .retain(|recorded| recorded.start_host() < host);
        let boundary = AxisEndpoint { host, position };
        let boundaries = self.rebase_boundaries.entry(key).or_default();
        while boundaries.back().is_some_and(|prior| prior.host >= host) {
            boundaries.pop_back();
        }
        if boundaries.len() == HISTORY_CAPACITY {
            boundaries.pop_front();
        }
        boundaries.push_back(boundary);
        self.endpoints.insert(key, boundary);
    }

    pub fn final_position(&self, key: AxisKey) -> Option<f64> {
        self.endpoints.get(&key).map(|e| e.position)
    }

    pub fn is_tracked(&self, key: AxisKey) -> bool {
        self.rings.get(&key).is_some_and(|r| !r.is_empty()) || self.endpoints.contains_key(&key)
    }

    /// True only when nothing recorded for `key` precedes `clock` (axis MCU
    /// clock domain — host projections drift and cannot gate this): no
    /// eviction, no pre-ring hold (both imply older motion existed), and
    /// `clock` lies before the ring's first span — or the axis was never
    /// recorded at all.
    pub fn predates_all_recorded_motion(&self, key: AxisKey, clock: u64) -> bool {
        if self.evicted.get(&key).copied().unwrap_or(0) != 0 {
            return false;
        }
        if self.holds_before_ring.contains_key(&key) {
            return false;
        }
        match self
            .rings
            .get(&key)
            .and_then(std::collections::VecDeque::front)
        {
            Some(front) => clock < front.span.start_clock,
            None => !self.endpoints.contains_key(&key),
        }
    }
    pub fn initial_hold_state(&self, key: AxisKey) -> Option<AxisState> {
        if self.evicted.get(&key).copied().unwrap_or(0) != 0
            || self.holds_before_ring.contains_key(&key)
        {
            return None;
        }
        self.rings
            .get(&key)?
            .front()
            .map(|recorded| recorded.startpoint().hold_state())
    }

    /// Axis state at an MCU clock reading from the same MCU the spans were
    /// sent to. A view's exact fractional clock anchor inverts straight back
    /// to stream time, where `state_at_host` interpolates the two host
    /// anchors captured at send and inherits the sync estimate's jitter
    /// between those two moments — an error that scales with axis velocity
    /// and, in the simulator, with `VTIME_SPEED`. `host_t` is the clock's
    /// host-time projection, used only for the hold fallbacks, where the
    /// position is constant and mapping jitter cannot bias it.
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
        let found = ring.iter().rev().find(|s| s.span.start_clock <= clock);
        if let Some(boundary) = self.rebase_boundary_at(key, host_t) {
            if found.is_none_or(|recorded| boundary.host > recorded.start_host()) {
                return Ok(boundary.hold_state());
            }
        }
        let Some(recorded) = found else {
            return self.state_at_host(key, host_t, now_host);
        };
        if clock < recorded.span.end_clock {
            return recorded
                .state_at_clock(clock)
                .map_err(|source| HistoryError::Evaluation { key, source });
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
        Ok(recorded.endpoint().hold_state())
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
        let boundary = self.rebase_boundary_at(key, host_t);
        let ring = self.rings.get(&key).filter(|r| !r.is_empty());
        let hold = match ring {
            Some(ring) => {
                let idx = ring.partition_point(|s| s.start_host() <= host_t);
                if let Some(boundary) = boundary {
                    if idx == 0 || boundary.host > ring[idx - 1].start_host() {
                        return Ok(boundary.hold_state());
                    }
                }
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
                        window_start: ring.front().map_or(0.0, HistorySpan::start_host),
                        window_end: ring.back().map_or(0.0, HistorySpan::end_host),
                        ring_len: ring.len(),
                        evicted: self.evicted.get(&key).copied().unwrap_or(0),
                        first_dur_s: ring.front().map_or(0.0, HistorySpan::duration_secs),
                    });
                }
                let recorded = &ring[idx - 1];
                if host_t < recorded.end_host() {
                    return recorded
                        .state_at_host(host_t)
                        .map_err(|source| HistoryError::Evaluation { key, source });
                }
                recorded.endpoint()
            }
            None => {
                if let Some(boundary) = boundary {
                    boundary
                } else {
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
pub(crate) mod tests;
