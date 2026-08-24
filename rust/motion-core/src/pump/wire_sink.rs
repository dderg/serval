use super::sample_sink::SampleEndpoint;
use super::stepcompress_sink::StepcompressEndpoint;
use super::{AxisFrame, AxisKey, SendError, SpanSink};
use crate::axis_transport::AxisTransports;
use crate::lock_ext::LockExt;
use ethercat_rt::setpoint_fill::ChainFiller;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::Weak;
use std::time::Duration;
use trajectory::ClockedMotorSpan;

/// An EtherCAT endpoint's host-side setpoint filler: it turns the staged
/// spans into one entry per DC cycle, which is the only thing the endpoint
/// executes.
pub type RingFiller = Arc<Mutex<ChainFiller>>;

/// The node stamps its DC grid in nanoseconds, so the anchor the pump reads
/// for an EtherCAT mcu is only a grid clock when the router clocks that mcu
/// at 1 GHz. Anything else means the arming instant and the grid it must be
/// placed on are two different clocks.
pub const DC_GRID_CLOCK_FREQ_HZ: f64 = ethercat_rt::setpoint_fill::CLOCK_FREQ_HZ;

/// Cycles the filler covers in one `PushSampleRuns`, and therefore the ring
/// headroom a lane must report before the pump ships another window. The
/// endpoint refuses a larger block outright, so this is a wire limit, not a
/// tuning knob.
const FILL_WINDOW_CYCLES: u32 = ethercat_rt::setpoint::MAX_FILL_CYCLES as u32;

/// Views one EtherCAT lane may hold staged at once: the one it is converting
/// and its successor, which is the whole depth `ChainFiller::free_span_slots`
/// reports.
const ETHERCAT_SPAN_SLOTS_PER_LANE: usize = 2;

/// Lane groups the pump must not mix in one transaction, because a bundle is
/// atomic per endpoint. Only their inequality matters to the pump.
pub const LANE_GROUP_PULSE: u8 = 0;
pub const LANE_GROUP_PHASE: u8 = 1;

/// An EtherCAT endpoint as the pump reaches it: the claim's socket plus the
/// filler built from the grid that claim reported.
pub struct EtherCatRing {
    pub conn: Weak<host_rt::mcu_serial_conn::McuSerialConn>,
    pub ring: RingFiller,
}

/// Every lane's transport, keyed by mcu. One mcu legally appears in both
/// `stepcompress` and `samples` — a modulated X beside a pulsed Z is two lane
/// kinds on one board — and a single phase-capable lane legally appears in
/// both, because a phase motor homed on StallGuard carries a classic step/dir
/// binding as well. Membership therefore says only what a lane *could* stream
/// through; `transports` says which one owns it now. `ethercat` excludes both.
pub struct WireSink {
    pub stepcompress: HashMap<u32, Arc<Mutex<StepcompressEndpoint>>>,
    pub samples: HashMap<u32, Arc<Mutex<SampleEndpoint>>>,
    pub ethercat: HashMap<u32, EtherCatRing>,
    pub transports: Arc<AxisTransports>,
    pub timeout: Duration,
}

impl WireSink {
    fn stepcompress_of(&self, mcu_id: u32) -> Option<&Arc<Mutex<StepcompressEndpoint>>> {
        self.stepcompress.get(&mcu_id)
    }

    fn samples_of(&self, mcu_id: u32) -> Option<&Arc<Mutex<SampleEndpoint>>> {
        self.samples.get(&mcu_id)
    }

    fn drives_pulse_lane(&self, key: AxisKey) -> bool {
        self.transports.is_pulse(key)
            && self
                .stepcompress_of(key.mcu_id)
                .is_some_and(|e| e.lock_ok().drives_axis(key.axis))
    }

    fn drives_sample_lane(&self, key: AxisKey) -> bool {
        self.transports.is_phase(key)
            && self
                .samples_of(key.mcu_id)
                .is_some_and(|e| e.lock_ok().drives_axis(key.axis))
    }

    /// One `PushSampleRuns` transaction with the endpoint, with the
    /// fatal-vs-transient split the pump's error handling depends on.
    fn call_push_sample_runs(
        &self,
        mcu_id: u32,
        conn: &host_rt::mcu_serial_conn::McuSerialConn,
        lanes: Vec<mcu_protocol::messages::LaneRun>,
    ) -> Result<mcu_protocol::messages::PushSampleRunsResponse, SendError> {
        use host_rt::transport::TransportError;
        use mcu_protocol::codec::Decode as _;

        let msg = mcu_protocol::messages::PushSampleRuns { lanes };
        let body = mcu_protocol::codec::Encode::encoded_to_vec(&msg);
        let (_kind, resp_body) = conn
            .kalico_call_on_channel(
                mcu_protocol::MCU_CHANNEL_PIECES,
                mcu_protocol::MessageKind::PushSampleRuns,
                body,
                self.timeout,
            )
            .map_err(|e| {
                if matches!(&e, TransportError::Closed | TransportError::Io(_)) {
                    SendError::Fatal(format!("ethercat PushSampleRuns mcu {mcu_id}: {e:?}"))
                } else {
                    SendError::Transient(format!("ethercat PushSampleRuns mcu {mcu_id}: {e:?}"))
                }
            })?;
        mcu_protocol::messages::PushSampleRunsResponse::decode(&resp_body).map_err(|e| {
            SendError::Transient(format!("decode PushSampleRunsResponse mcu {mcu_id}: {e:?}"))
        })
    }

    /// Stage the frames' spans in the filler, then ship contiguous per-lane
    /// runs until the endpoint's reported headroom no longer covers another
    /// full fill window. The headroom is the only pacing signal — the filler
    /// samples the whole staged trajectory, so without it a deep bundle would
    /// overrun the ring instead of arriving one window at a time.
    ///
    /// A failed bundle is re-sent byte-identically by the pump, and a staged
    /// sample stream is not idempotent — the views are already in the filler
    /// and its lanes have already moved on. So every error path drops the
    /// stage of the axes it touched: the re-send restages them and the
    /// resulting run re-anchors, discarding whatever the endpoint accepted
    /// from the failed attempt.
    fn send_sample_runs(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
        ring: &RingFiller,
    ) -> Result<(), SendError> {
        let result = self.fill_sample_runs(mcu_id, frames, ring);
        if result.is_err() {
            let mut filler = ring.lock_ok();
            for frame in frames {
                filler.cut_axis(frame.axis);
            }
        }
        result
    }

    fn fill_sample_runs(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
        ring: &RingFiller,
    ) -> Result<(), SendError> {
        let conn = self
            .ethercat
            .get(&mcu_id)
            .ok_or_else(|| {
                SendError::Fatal(format!(
                    "setpoint-ring send for mcu {mcu_id}, which has no ethercat transport"
                ))
            })?
            .conn
            .upgrade()
            .ok_or_else(|| {
                SendError::Fatal(format!(
                    "ethercat conn for mcu {mcu_id} detached (released)"
                ))
            })?;
        let mut filler = ring.lock_ok();
        for frame in frames {
            if !filler.drives_axis(frame.axis) {
                return Err(SendError::Fatal(format!(
                    "ethercat mcu {mcu_id}: axis {} has no setpoint lane — the filler was \
                     built from a lane set that does not cover the pump's axes",
                    frame.axis
                )));
            }
            filler.push_spans(frame.axis, &frame.spans).map_err(|e| {
                SendError::Fatal(format!(
                    "ethercat mcu {mcu_id}: axis {} cannot stage its spans ({}): {e:?}",
                    frame.axis,
                    e.as_str()
                ))
            })?;
        }
        loop {
            let lanes = filler.drain().map_err(|e| {
                SendError::Fatal(format!(
                    "ethercat mcu {mcu_id}: setpoint fill failed ({}): {e:?}",
                    e.as_str()
                ))
            })?;
            if lanes.is_empty() {
                return Ok(());
            }
            let response = self.call_push_sample_runs(mcu_id, &conn, lanes)?;
            if response.result != mcu_protocol::result_codes::OK {
                super::transit_trace::emit_result_fault_snapshot("mcu_reject", response.result);
                return Err(SendError::mcu_reject(mcu_id, response.result));
            }
            let mut headroom = u32::MAX;
            for depth in &response.lanes {
                if !filler.drives_axis(depth.axis_idx) {
                    return Err(SendError::Fatal(format!(
                        "ethercat mcu {mcu_id}: PushSampleRunsResponse reported depth for \
                         axis {}, which is not a lane of this endpoint",
                        depth.axis_idx
                    )));
                }
                headroom = headroom.min(depth.free_cycles);
            }
            filler
                .observe_grid(response.grid_index, response.grid_clock)
                .map_err(|e| {
                    SendError::Fatal(format!(
                        "ethercat mcu {mcu_id}: setpoint grid rejected ({}): {e:?}",
                        e.as_str()
                    ))
                })?;
            if !filler.wants_drain() || headroom < FILL_WINDOW_CYCLES {
                return Ok(());
            }
        }
    }

    /// Drop the staged setpoints of every named ring lane. Called wherever the
    /// endpoint discards motion it already accepted, so host and endpoint
    /// re-anchor together instead of the next run continuing a stream the ring
    /// no longer holds.
    fn cut_ring_lanes(&self, keys: &[AxisKey]) {
        for key in keys {
            if let Some(ec) = self.ethercat.get(&key.mcu_id) {
                ec.ring.lock_ok().cut_axis(key.axis);
            }
        }
    }

    fn no_transport(&self, key: AxisKey, what: &str) -> SendError {
        SendError::Fatal(format!(
            "{what}: mcu {} axis {} belongs to no endpoint — it is neither a pulse lane of a \
             stepcompress endpoint, nor a phase lane of a sample endpoint, nor an ethercat \
             drive; init_planner registered a lane with no transport",
            key.mcu_id, key.axis
        ))
    }
}

impl SpanSink for WireSink {
    /// Single-axis convenience — the pump drives WireSink via `send_mcu_frames`;
    /// this exists only to satisfy the trait and routes through the same path.
    fn send_frame(
        &self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError> {
        let frame = AxisFrame {
            axis: key.axis,
            spans: spans.to_vec(),
            new_head,
            room,
            guard_recorded_ns: 0,
            guard_mcu_clock: 0,
        };
        self.send_mcu_frames(key.mcu_id, std::slice::from_ref(&frame))
            .map(|()| mcu_protocol::result_codes::OK)
    }

    /// The filler holds one active view and one successor per lane, so a
    /// bundle may not carry more than that: past it `push_spans` refuses the
    /// stage outright.
    fn bundle_limits(&self, mcu_id: u32) -> super::BundleLimits {
        if self.ethercat.contains_key(&mcu_id) {
            return super::BundleLimits {
                spans_per_axis: ETHERCAT_SPAN_SLOTS_PER_LANE,
            };
        }
        super::messages::SERIAL_BUNDLE_LIMITS
    }

    fn lane_group(&self, key: AxisKey) -> u8 {
        if self.drives_sample_lane(key) {
            LANE_GROUP_PHASE
        } else {
            LANE_GROUP_PULSE
        }
    }

    fn mark_reanchor(&self, key: AxisKey, at_start_clock: u64, epoch_freq: Option<f64>) {
        if self.drives_pulse_lane(key) {
            self.stepcompress_of(key.mcu_id)
                .expect("a pulse lane named its own endpoint")
                .lock_ok()
                .mark_reanchor(key.axis, at_start_clock, epoch_freq);
            return;
        }
        if self.drives_sample_lane(key) {
            self.samples_of(key.mcu_id)
                .expect("a phase lane named its own endpoint")
                .lock_ok()
                .mark_reanchor(key.axis, at_start_clock, epoch_freq)
                .unwrap_or_else(|e| {
                    panic!(
                        "mark_reanchor: sample endpoint mcu {} rejected its own axis {}: {e}",
                        key.mcu_id, key.axis
                    )
                });
        }
    }

    fn mark_seam_gap(&self, key: AxisKey, at_start_clock: u64) {
        if self.drives_pulse_lane(key) {
            self.stepcompress_of(key.mcu_id)
                .expect("a pulse lane named its own endpoint")
                .lock_ok()
                .mark_seam_gap(key.axis, at_start_clock);
            return;
        }
        if self.drives_sample_lane(key) {
            self.samples_of(key.mcu_id)
                .expect("a phase lane named its own endpoint")
                .lock_ok()
                .mark_seam_gap(key.axis, at_start_clock)
                .unwrap_or_else(|e| {
                    panic!(
                        "mark_seam_gap: sample endpoint mcu {} rejected its own axis {}: {e}",
                        key.mcu_id, key.axis
                    )
                });
        }
    }

    fn on_barrier_ack(&self, mcu_id: u32, oid: u8, seq: u32) -> Result<(), SendError> {
        let oid = u32::from(oid);
        if let Some(endpoint) = self.stepcompress_of(mcu_id) {
            let mut endpoint = endpoint.lock_ok();
            if endpoint.owns_oid(oid) {
                return endpoint.on_barrier_ack(oid, seq);
            }
        }
        if let Some(endpoint) = self.samples_of(mcu_id) {
            let mut endpoint = endpoint.lock_ok();
            if endpoint.owns_oid(oid) {
                return endpoint.on_barrier_ack(oid, seq);
            }
        }
        Err(SendError::Fatal(format!(
            "barrier ack oid={oid} seq={seq} arrived for mcu {mcu_id}, which has no endpoint \
             owning that stepper oid"
        )))
    }

    fn flush_keys(&self, keys: &[AxisKey]) -> Result<(), SendError> {
        self.cut_ring_lanes(keys);
        let mut pulse_axes: HashMap<u32, Vec<u8>> = HashMap::new();
        for key in keys {
            if self.drives_pulse_lane(*key) {
                pulse_axes.entry(key.mcu_id).or_default().push(key.axis);
            }
        }
        for (mcu_id, axes) in pulse_axes {
            self.stepcompress_of(mcu_id)
                .expect("pulse lanes named their own endpoint")
                .lock_ok()
                .abort_axes(&axes)?;
        }
        Ok(())
    }

    fn cut_staged(&self, keys: &[AxisKey]) -> Result<(), SendError> {
        self.flush_keys(keys)
    }

    fn drain_tick_mcus(&self) -> Vec<u32> {
        self.ethercat.keys().copied().collect()
    }

    fn drain_tick(&self, mcu_id: u32) -> Result<(), SendError> {
        match self.ethercat.get(&mcu_id) {
            Some(ec) if ec.ring.lock_ok().wants_drain() => {
                self.send_sample_runs(mcu_id, &[], &ec.ring)
            }
            _ => Ok(()),
        }
    }

    fn wants_drain_tick(&self, mcu_id: u32) -> bool {
        self.ethercat
            .get(&mcu_id)
            .is_some_and(|ec| ec.ring.lock_ok().wants_drain())
    }

    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        if let Some(ec) = self.ethercat.get(&mcu_id) {
            return self.send_sample_runs(mcu_id, frames, &ec.ring);
        }
        let Some(&first) = frames.first().map(|f| &f.axis) else {
            return Ok(());
        };
        let key = AxisKey {
            mcu_id,
            axis: first,
        };
        if self.drives_pulse_lane(key) {
            return self
                .stepcompress_of(mcu_id)
                .expect("a pulse lane named its own endpoint")
                .lock_ok()
                .send_frames(mcu_id, frames);
        }
        if self.drives_sample_lane(key) {
            return self
                .samples_of(mcu_id)
                .expect("a phase lane named its own endpoint")
                .lock_ok()
                .send_frames(mcu_id, frames);
        }
        Err(self.no_transport(key, "send_mcu_frames"))
    }
}

#[cfg(test)]
#[path = "wire_sink_tests.rs"]
mod wire_sink_tests;

#[cfg(test)]
#[path = "wire_sink_ring_tests.rs"]
mod wire_sink_ring_tests;
