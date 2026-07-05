use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;

use crate::clock::{Clock, RealClock};
use crate::host_io::ReactorCommand;
use crate::host_io::events::EventDispatcher;
use crate::host_io::identify::IdentifySeqState;
use crate::host_io::mcu_session::{
    McuDispatchResult, McuTransportState, PendingMcuCall, build_kalico_frame,
    build_kalico_identify_frame, dispatch_mcu_frame,
};
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::rtt::RttEstimator;
use crate::host_io::runtime_events::{FaultEvent, StatusEvent};
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::host_io::window::{AwaitingResponse, UnackedWindow};
use crate::transport::TransportError;
use mcu_transport::demux::{Frame, KlipperFrame, PollOutcome};
use runtime::error::FaultCode;

pub struct Reactor {
    pub(crate) io: SerialFrameIo,
    pub(crate) parser: Arc<MsgProtoParser>,
    pub(crate) submission_rx: Receiver<ReactorCommand>,
    pub(crate) unacked_window: UnackedWindow,
    pub(crate) awaiting_response: AwaitingResponse,
    pub(crate) rtt: RttEstimator,
    pub(crate) event_dispatcher: EventDispatcher,

    pub(crate) send_seq: u64,
    pub(crate) receive_seq: u64,
    pub(crate) last_ack_seq: u64,
    pub(crate) ignore_nak_seq: u64,
    pub(crate) retransmit_seq: u64,
    pub(crate) rtt_sample_seq: u64,
    pub(crate) rtt_sample_armed: bool,

    pub(crate) state: ReactorState,

    pub(crate) closed_via_shutdown: bool,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) pending_submissions: VecDeque<PendingSubmission>,

    /// When `get_clock_async` is in flight: the CLOCK_MONOTONIC_RAW sent-time
    /// captured before the frame was written to wire.  The next unsolicited
    /// "clock" response matching this will be delivered as a PassthroughResponse
    /// with RAW RTT stamps rather than going through the generic path.
    pub(crate) pending_clock_sent_raw: Option<f64>,

    /// Queued fire-and-forget payloads; the bool marks a `get_clock` frame
    /// whose RAW send stamp is captured at the actual wire write.
    pub(crate) pending_fire_and_forget: VecDeque<(Vec<u8>, bool)>,
    /// Piece-channel (motion) frames, keyed by correlation id, awaiting a
    /// shallow kernel tty queue; see `drain_piece_frames` for the priority
    /// rule this enforces.
    pub(crate) pending_piece_frames: VecDeque<(u32, Vec<u8>)>,
    pub(crate) pending_outbound_order: VecDeque<PendingOutboundKind>,
    pub(crate) zero_byte_first_seen: Option<Instant>,
    pub(crate) last_recv_time: Instant,
    pub(crate) last_write_time: Instant,
    pub(crate) zero_byte_consec: u32,
    pub(crate) clock: Arc<dyn Clock>,
    pub(crate) transport_state: McuTransportState,
    pub(crate) interceptors: crate::host_io::interceptor::InterceptorTable,
}

pub(crate) struct PendingSubmission {
    pub call_id: u64,
    pub payload: Vec<u8>,
    pub expected_response_name: String,
    pub completion:
        std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
    pub deadline: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PendingOutboundKind {
    Submission,
    FireAndForget,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReactorState {
    Active,
    Closed,
}

impl Reactor {
    pub fn new(
        io: SerialFrameIo,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        seq: IdentifySeqState,
        config: crate::host_io::McuHostIoConfig,
    ) -> Self {
        Self::new_with_clock(
            io,
            parser,
            submission_rx,
            status_snapshot,
            seq,
            config,
            Arc::new(RealClock),
        )
    }

    pub fn new_with_clock(
        io: SerialFrameIo,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        seq: IdentifySeqState,
        config: crate::host_io::McuHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self {
        let event_dispatcher = EventDispatcher::new(
            Arc::clone(&status_snapshot),
            config.trace_capacity,
            config.host_event_capacity,
        );
        Self {
            io,
            parser,
            submission_rx,
            unacked_window: UnackedWindow::default(),
            awaiting_response: AwaitingResponse::default(),
            rtt: RttEstimator::default(),
            event_dispatcher,
            send_seq: seq.next_send_seq_abs,
            receive_seq: seq.mcu_receive_seq_abs,
            last_ack_seq: seq.mcu_receive_seq_abs.saturating_sub(1),
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            rtt_sample_armed: false,
            state: ReactorState::Active,
            closed_via_shutdown: false,
            pending_host_fault: None,
            pending_clock_sent_raw: None,
            pending_submissions: VecDeque::new(),
            pending_fire_and_forget: VecDeque::new(),
            pending_piece_frames: VecDeque::new(),
            pending_outbound_order: VecDeque::new(),
            zero_byte_first_seen: None,
            last_recv_time: clock.now(),
            last_write_time: clock.now(),
            zero_byte_consec: 0,
            clock,
            transport_state: McuTransportState::default(),
            interceptors: crate::host_io::interceptor::InterceptorTable::new(),
        }
    }

    #[cfg(any(test, feature = "test-harness"))]
    pub fn new_for_tests(
        port: Box<dyn serialport::SerialPort>,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        config: crate::host_io::McuHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self {
        Self::new_with_clock(
            SerialFrameIo::new(port),
            parser,
            submission_rx,
            status_snapshot,
            IdentifySeqState {
                next_send_seq_abs: 1,
                mcu_receive_seq_abs: 1,
            },
            config,
            clock,
        )
    }

    pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static WRITE_SEQ: AtomicU64 = AtomicU64::new(0);
        let seq = WRITE_SEQ.fetch_add(1, Ordering::Relaxed);
        let t0 = std::time::Instant::now();
        let proto = if !frame.is_empty() && frame[0] == 0x55 {
            "kalico"
        } else {
            "klipper"
        };
        let bytes = frame.len();
        // No drain here: waiting for the wire makes this single thread deaf to
        // responses for the frame's whole line time (~20 ms/KiB at 500 kbaud),
        // which times out unrelated transactions during heavy piece traffic.
        // The kernel tty buffer queues the bytes; drain_piece_frames bounds
        // how deep piece traffic may fill it, so control frames written here
        // are never far from the wire.
        let result = self.io.write_all(frame);
        if result.is_ok() {
            self.last_write_time = std::time::Instant::now();
        }
        let dt = t0.elapsed();
        if dt > std::time::Duration::from_millis(20) {
            let outq_after = self.io.bytes_to_write().unwrap_or(u32::MAX);
            tracing::warn!(
                subsystem = "mcu-comms",
                event = "slow_frame_write",
                proto,
                bytes,
                dt_ms = dt.as_secs_f64() * 1000.0,
                outq_after,
                ok = result.is_ok(),
                "frame write exceeded 20ms — kernel out-queue not draining"
            );
        }
        tracing::trace!(
            subsystem = "mcu-comms",
            event = "frame_write",
            tid = ?std::thread::current().id(),
            seq,
            proto,
            bytes,
            dt_ms = dt.as_secs_f64() * 1000.0,
            result = ?result.as_ref().map(|_| "OK"),
            first8 = ?&frame[..frame.len().min(8)],
            "frame write"
        );
        result
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RetransmitTrigger {
    NakDriven,
    TimeoutDriven,
}

const PENDING_SUBMISSION_CEILING: usize = 256;
pub const PENDING_FIRE_AND_FORGET_CEILING: usize = 256;
pub(crate) const PENDING_PIECE_FRAMES_CEILING: usize = 64;

/// Max bytes of kalico (piece) traffic allowed in the kernel tty out-buffer
/// before further kalico frames are held back. Klipper-channel control
/// commands write unconditionally, so this is the most piece traffic a
/// control frame can ever be queued behind — small enough to bound control
/// latency to a few ms of wire time, large enough (vs the ~1 ms reactor
/// tick) to keep the wire saturated with pieces when nothing else wants it.
pub(crate) const PIECE_OUTQ_BUDGET_BYTES: u32 = 2048;
const MAX_RETRY_COUNT: u32 = 8;

// Retry exhaustion alone is not sufficient to declare Closed: under Renode
// (1 µs quantum) a long-running MCU command can stall status emission for
// several seconds wall while the wire remains healthy. Only close when
// retry exhaustion coincides with genuine MCU silence.
const MCU_SILENCE_FOR_CLOSE: Duration = Duration::from_secs(120);

const MAX_SUBMITS_PER_ITER: usize = 4;
const READ_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(1);
const ZERO_BYTE_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(100);

impl Reactor {
    pub(crate) fn dispatch_submission(
        &mut self,
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: std::sync::mpsc::SyncSender<
            Result<crate::transport::MessageParams, TransportError>,
        >,
        deadline: Instant,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.pending_submissions.len() >= PENDING_SUBMISSION_CEILING {
                let _ = completion.send(Err(TransportError::Backpressure));
                return Ok(());
            }
            self.pending_submissions.push_back(PendingSubmission {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            });
            self.pending_outbound_order
                .push_back(PendingOutboundKind::Submission);
            return Ok(());
        }

        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);

        let sent_time_raw = crate::clock::monotonic_raw_secs();
        self.write_frame(&frame)?;

        let now = self.clock.now();
        self.unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: frame,
                sent_at: now,
                retry_count: 0,
            });
        let _trace_name = expected_response_name.clone();
        self.awaiting_response
            .push(crate::host_io::window::AwaitEntry {
                call_id,
                seq,
                expected_response_name,
                completion,
                submitted_at: now,
                deadline,
                abandoned: false,
                sent_time_raw,
            })?;
        tracing::trace!(
            subsystem = "mcu-comms",
            event = "await_response",
            tid = ?std::thread::current().id(),
            call_id,
            seq,
            name = %_trace_name,
            await_len = self.awaiting_response.len(),
            "push await entry"
        );

        if !self.rtt_sample_armed {
            self.rtt_sample_seq = seq;
            self.rtt_sample_armed = true;
        }
        Ok(())
    }

    pub(crate) fn dispatch_fire_and_forget(
        &mut self,
        payload: Vec<u8>,
        is_get_clock: bool,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.pending_fire_and_forget.len() >= PENDING_FIRE_AND_FORGET_CEILING {
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "fire_and_forget_ceiling",
                    ceiling = PENDING_FIRE_AND_FORGET_CEILING,
                    "dispatch_fire_and_forget: pending_fire_and_forget at ceiling; refusing payload"
                );
                return Err(TransportError::Backpressure);
            }
            self.pending_fire_and_forget
                .push_back((payload, is_get_clock));
            self.pending_outbound_order
                .push_back(PendingOutboundKind::FireAndForget);
            return Ok(());
        }
        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);
        // get_clock send stamps MUST be captured at the wire write, not at
        // command processing: on a busy link (beacon's status stream) the
        // frame can queue for multiple ms, and an early stamp pairs the
        // response clock with a fictitious send time — observed as a
        // constant +5.6ms outlier on every beacon clocksync sample.
        if is_get_clock {
            self.pending_clock_sent_raw = Some(crate::clock::monotonic_raw_secs());
        }
        if let Err(e) = self.write_frame(&frame) {
            if is_get_clock {
                self.pending_clock_sent_raw = None;
            }
            return Err(e);
        }

        let now = self.clock.now();
        self.unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: frame,
                sent_at: now,
                retry_count: 0,
            });
        Ok(())
    }

    pub(crate) fn drain_pending_submissions(&mut self) {
        while !self.unacked_window.is_full() {
            let Some(kind) = self.pending_outbound_order.pop_front() else {
                break;
            };
            match kind {
                PendingOutboundKind::Submission => {
                    let Some(p) = self.pending_submissions.pop_front() else {
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "outbound_order_missing_submission",
                            "pending outbound order referenced missing submission"
                        );
                        continue;
                    };
                    let completion = p.completion.clone();
                    if let Err(e) = self.dispatch_submission(
                        p.call_id,
                        p.payload,
                        p.expected_response_name,
                        completion,
                        p.deadline,
                    ) {
                        let is_io = matches!(e, TransportError::Io(_));
                        if is_io {
                            self.transition_closed_on_io_fault(
                                "drain_pending_submissions/submission",
                                &e,
                            );
                        }
                        let _ = p.completion.send(Err(e));
                        if is_io {
                            return;
                        }
                    }
                }
                PendingOutboundKind::FireAndForget => {
                    let Some((payload, is_get_clock)) = self.pending_fire_and_forget.pop_front()
                    else {
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "outbound_order_missing_fire_and_forget",
                            "pending outbound order referenced missing fire-and-forget"
                        );
                        continue;
                    };
                    if let Err(e) = self.dispatch_fire_and_forget(payload, is_get_clock) {
                        if matches!(e, TransportError::Io(_)) {
                            self.transition_closed_on_io_fault(
                                "drain_pending_submissions/fire_and_forget",
                                &e,
                            );
                            return;
                        }
                        tracing::warn!(
                            subsystem = "mcu-comms",
                            event = "fire_and_forget_redispatch_error",
                            error = %e,
                            "drain_pending_submissions: fire-and-forget redispatch error"
                        );
                    }
                }
            }
        }
    }

    /// Write queued piece frames while the kernel tty queue is
    /// shallow. Piece frames yield to control traffic: klipper-channel frames
    /// (heater/fan PWM, endstops, clocksync) write unconditionally, while a
    /// piece frame waits until pending wire bytes drop under the budget — so
    /// a control command is never queued behind more than
    /// `PIECE_OUTQ_BUDGET_BYTES` of piece bytes. Without this, a print-start
    /// piece flood keeps the tty queue deep, control acks inflate, and a
    /// `queue_digital_out` arrives seconds late.
    pub(crate) fn drain_piece_frames(&mut self) {
        while !self.pending_piece_frames.is_empty() {
            match self.io.bytes_to_write() {
                Ok(pending) if pending > PIECE_OUTQ_BUDGET_BYTES => return,
                Ok(_) => {}
                Err(e) => {
                    self.transition_closed_on_io_fault("drain_piece_frames/outq_poll", &e);
                    return;
                }
            }
            let (cid, frame) = self
                .pending_piece_frames
                .pop_front()
                .expect("checked non-empty");
            if let Err(e) = self.write_frame(&frame) {
                if matches!(e, TransportError::Io(_)) {
                    self.transition_closed_on_io_fault("drain_piece_frames/write_frame", &e);
                }
                if let Some(p) = self.transport_state.pending.remove(&cid) {
                    let _ = p.completion.send(Err(e));
                }
                return;
            }
        }
    }

    fn update_receive_seq(&mut self, rseq: u64) -> Result<(), TransportError> {
        if self.unacked_window.is_empty() {
            self.send_seq = rseq;
            self.receive_seq = rseq;
            return Ok(());
        }
        let popped = self.unacked_window.pop_acked(rseq);
        for entry in &popped {
            if self.rtt_sample_armed && entry.seq >= self.rtt_sample_seq {
                let rtt = self.clock.now() - entry.sent_at;
                self.rtt.update(rtt);
                self.rtt_sample_armed = false;
                break;
            }
        }
        self.receive_seq = rseq;
        Ok(())
    }

    pub(crate) fn handle_ack_nak(&mut self, wire_seq_nibble: u8) -> Result<(), TransportError> {
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);

        if rseq > self.receive_seq {
            self.update_receive_seq(rseq)?;
        }

        if self.last_ack_seq < rseq {
            self.last_ack_seq = rseq;
        } else if rseq > self.ignore_nak_seq && !self.unacked_window.is_empty() {
            self.write_retransmit(RetransmitTrigger::NakDriven)?;
        }
        Ok(())
    }

    pub(crate) fn write_retransmit(
        &mut self,
        trigger: RetransmitTrigger,
    ) -> Result<(), TransportError> {
        let buf = {
            let frames: Vec<&[u8]> = self
                .unacked_window
                .iter()
                .map(|e| e.frame_bytes.as_slice())
                .collect();
            crate::host_io::wire::build_retransmit_buffer(frames)
        };
        self.write_frame(&buf)?;

        match trigger {
            RetransmitTrigger::NakDriven => {
                if self.receive_seq < self.retransmit_seq {
                    self.ignore_nak_seq = self.retransmit_seq;
                } else {
                    self.ignore_nak_seq = self.receive_seq;
                }
            }
            RetransmitTrigger::TimeoutDriven => {
                self.ignore_nak_seq = self.send_seq;
            }
        }
        self.retransmit_seq = self.send_seq;
        self.rtt_sample_armed = false;

        let now = self.clock.now();
        let silence = now.duration_since(self.last_recv_time);
        for entry in self.unacked_window.iter_mut() {
            entry.retry_count += 1;
            if entry.retry_count >= MAX_RETRY_COUNT && silence >= MCU_SILENCE_FOR_CLOSE {
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "retransmit_exhausted",
                    retry_count = entry.retry_count,
                    seq = entry.seq,
                    silence_ms = silence.as_millis() as u64,
                    "MCU silent through retransmit budget — closing transport"
                );
                self.state = ReactorState::Closed;
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code: FaultCode::HostRetransmitExhausted.as_u16(),
                    fault_detail: entry.retry_count,
                    segment_id: 0,
                    synthesized: false,
                });
                return Err(TransportError::Closed);
            }
        }

        if matches!(trigger, RetransmitTrigger::TimeoutDriven) {
            self.rtt.backoff();
        }
        Ok(())
    }
}

impl Reactor {
    pub(crate) fn handle_inbound_frame(
        &mut self,
        frame: KlipperFrame,
    ) -> Result<(), TransportError> {
        let bytes = frame.bytes();
        if bytes.len() < crate::host_io::wire::MESSAGE_MIN {
            return Ok(());
        }
        let wire_seq_nibble = bytes[1] & 0x0F;
        if bytes.len() == crate::host_io::wire::MESSAGE_MIN {
            self.handle_ack_nak(wire_seq_nibble)?;
            return Ok(());
        }
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);
        let rseq_jump = rseq.saturating_sub(self.receive_seq);
        if rseq_jump > 1 {
            tracing::warn!(
                subsystem = "mcu-comms",
                event = "rx_seq_jump",
                receive_seq_prev = self.receive_seq,
                receive_seq_new = rseq,
                jump = rseq_jump,
                "receive_seq jumped >1: MCU dropped a response or we missed a frame"
            );
        }
        if rseq != self.receive_seq {
            self.update_receive_seq(rseq)?;
        }
        let decoded = match self.parser.decode(bytes) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "decode_error",
                    error = ?e,
                    bytes_len = bytes.len(),
                    first16 = ?&bytes[..bytes.len().min(16)],
                    "frame decode error"
                );
                return Ok(());
            }
        };

        match decoded {
            crate::host_io::parser::DecodedFrame::Response { name, params } => {
                if name == "shutdown" || name == "is_shutdown" {
                    self.fail_pending_on_mcu_shutdown(&name, &params);
                }
                let await_len_before = self.awaiting_response.len();
                if let Some(idx) = self.awaiting_response.find_match(&name) {
                    let entry = self.awaiting_response.remove(idx);
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "await_response",
                        tid = ?std::thread::current().id(),
                        %name,
                        idx,
                        await_len = await_len_before,
                        matched_call_id = entry.call_id,
                        matched_seq = entry.seq,
                        "solicited response matched"
                    );
                    let mut params = params;
                    params.sent_time_raw = entry.sent_time_raw;
                    params.recv_time_raw = crate::clock::monotonic_raw_secs();
                    let _ = entry.completion.send(Ok(params));
                } else {
                    let oid = params.fields.get("oid").and_then(|v| match v {
                        crate::transport::MessageValue::U32(n) => Some(*n),
                        crate::transport::MessageValue::I32(n) => Some(*n as u32),
                        _ => None,
                    });
                    if name.contains("software_trip") || name.contains("trsync_state") {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            params = ?params,
                            "unsolicited frame (software_trip/trsync_state)"
                        );
                    } else {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            "unsolicited frame"
                        );
                    }
                    if name == "clock" {
                        if let Some(sent_raw) = self.pending_clock_sent_raw.take() {
                            let recv_raw = crate::clock::monotonic_raw_secs();
                            let mut stamped = params.clone();
                            stamped.sent_time_raw = sent_raw;
                            stamped.recv_time_raw = recv_raw;
                            let event =
                                crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                                    name,
                                    params: stamped,
                                };
                            self.dispatch_runtime_event(event);
                            return Ok(());
                        }
                    }

                    self.interceptors.dispatch(&name, oid, &params);

                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "unsolicited_no_interceptor",
                        tid = ?std::thread::current().id(),
                        %name,
                        await_len = await_len_before,
                        "unsolicited frame with no interceptor match"
                    );
                    let event = crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                        name,
                        params,
                    };
                    self.dispatch_runtime_event(event);
                }
            }
            crate::host_io::parser::DecodedFrame::Output { name, params } => {
                let oid = params.fields.get("oid").and_then(|v| match v {
                    crate::transport::MessageValue::U32(n) => Some(*n),
                    crate::transport::MessageValue::I32(n) => Some(*n as u32),
                    _ => None,
                });
                let interceptor_count = self.interceptors.entry_count();
                if interceptor_count > 0 {
                    tracing::debug!(
                        subsystem = "trip-relay",
                        event = "output_frame_intercepted",
                        %name,
                        ?oid,
                        interceptor_count,
                        "output frame dispatched through interceptors"
                    );
                }
                self.interceptors.dispatch(&name, oid, &params);
                let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                self.dispatch_runtime_event(event);
            }
        }
        Ok(())
    }

    fn dispatch_runtime_event(&mut self, event: crate::host_io::runtime_events::RuntimeEvent) {
        self.event_dispatcher.dispatch(event);
    }

    pub(crate) fn handle_kalico_frame(&mut self, channel: u8, payload: &[u8]) {
        match dispatch_mcu_frame(&mut self.transport_state, channel, payload) {
            McuDispatchResult::Handled | McuDispatchResult::Ignored => {}
            McuDispatchResult::Event(ev) => {
                self.dispatch_runtime_event(ev);
            }
        }
    }
}

impl Reactor {
    fn poll_serial(&mut self) {
        let t0 = std::time::Instant::now();
        let deadline = self.clock.now() + READ_TIMEOUT;
        let outcome = self.io.poll_frames_until(deadline);
        let dt = t0.elapsed();
        if dt > std::time::Duration::from_millis(5) {
            let label: &'static str = match &outcome {
                Ok(PollOutcome::Frames { .. }) => "Frames",
                Ok(PollOutcome::Timeout) => "Timeout",
                Ok(PollOutcome::PhantomZero) => "PhantomZero",
                Err(_) => "Err",
            };
            tracing::debug!(
                subsystem = "mcu-comms",
                event = "slow_poll",
                dt_ms = dt.as_secs_f64() * 1000.0,
                outcome = label,
                "poll_serial exceeded 5ms"
            );
        }
        match outcome {
            Ok(PollOutcome::Frames { frames, errors }) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
                if !frames.is_empty() || !errors.is_empty() {
                    self.last_recv_time = self.clock.now();
                }
                for e in errors {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "kalico_stream_error",
                        error = %e,
                        "kalico stream error"
                    );
                }
                for f in frames {
                    match f {
                        Frame::Klipper(kf) => {
                            if let Err(e) = self.handle_inbound_frame(kf) {
                                tracing::error!(
                                    subsystem = "mcu-comms",
                                    event = "inbound_frame_fatal",
                                    error = ?e,
                                    "inbound frame handling failed (ack/retransmit write?) — \
                                     closing transport"
                                );
                                return;
                            }
                        }
                        Frame::Kalico { channel, payload } => {
                            self.handle_kalico_frame(channel, &payload);
                        }
                    }
                }
            }
            Ok(PollOutcome::Timeout) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
            }
            Ok(PollOutcome::PhantomZero) => {
                self.zero_byte_consec = self.zero_byte_consec.saturating_add(1);
                let now = self.clock.now();
                let first = *self.zero_byte_first_seen.get_or_insert(now);
                if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                    let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                    let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "usb_drop_phantom_zero",
                        silence_ms = %silence_ms,
                        since_write_ms = %since_write_ms,
                        consec_zero = self.zero_byte_consec,
                        debounce = ?ZERO_BYTE_DEBOUNCE,
                        "[usb-drop] PhantomZero (Ok(0) past debounce window)"
                    );
                    self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                        fault_code: FaultCode::HostDisconnect.as_u16(),
                        fault_detail: 0,
                        segment_id: 0,
                        synthesized: false,
                    });
                    self.state = ReactorState::Closed;
                }
            }
            Err(e) => {
                let now = self.clock.now();
                let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                let (os_errno, io_kind) = match &e {
                    TransportError::Io(io) => (io.raw_os_error(), Some(io.kind())),
                    _ => (None, None),
                };
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "usb_drop_poll_error",
                    silence_ms = %silence_ms,
                    since_write_ms = %since_write_ms,
                    consec_zero = self.zero_byte_consec,
                    os_errno = ?os_errno,
                    io_kind = ?io_kind,
                    error = ?e,
                    "[usb-drop] poll error"
                );
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code: FaultCode::HostDisconnect.as_u16(),
                    fault_detail: 0,
                    segment_id: 0,
                    synthesized: false,
                });
                self.state = ReactorState::Closed;
            }
        }
    }
}

impl Reactor {
    pub(crate) fn transition_closed_on_io_fault(
        &mut self,
        context: &'static str,
        error: &TransportError,
    ) {
        let (os_errno, io_kind) = match error {
            TransportError::Io(io) => (io.raw_os_error(), Some(io.kind())),
            _ => (None, None),
        };
        let drain_curve: Vec<String> = (0..10)
            .map(|_| {
                let depth = self
                    .io
                    .bytes_to_write()
                    .map(|b| b.to_string())
                    .unwrap_or_else(|e| format!("err:{e}"));
                std::thread::sleep(std::time::Duration::from_millis(20));
                depth
            })
            .collect();
        tracing::error!(
            subsystem = "mcu-comms",
            event = "transport_io_fault",
            context,
            os_errno = ?os_errno,
            io_kind = ?io_kind,
            error = %error,
            unacked_n = self.unacked_window.len(),
            pending_piece_frames = self.pending_piece_frames.len(),
            outq_drain_curve_20ms = %drain_curve.join(","),
            "transport IO fault; transitioning Closed"
        );
        if self.pending_host_fault.is_none() {
            self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                fault_code: FaultCode::HostDisconnect.as_u16(),
                fault_detail: 0,
                segment_id: 0,
                synthesized: false,
            });
        }
        self.state = ReactorState::Closed;
    }

    fn handle_command(&mut self, cmd: crate::host_io::ReactorCommand) {
        use crate::host_io::ReactorCommand;
        match cmd {
            ReactorCommand::Submit {
                call_id,
                cmd,
                expected_response_name,
                completion,
                deadline,
            } => match self.parser.encode(&cmd) {
                Ok(payload) => {
                    if let Err(e) = self.dispatch_submission(
                        call_id,
                        payload,
                        expected_response_name,
                        completion.clone(),
                        deadline,
                    ) {
                        if matches!(e, TransportError::Io(_)) {
                            self.transition_closed_on_io_fault("handle_command/submit", &e);
                        }
                        let _ = completion.send(Err(e));
                    }
                }
                Err(e) => {
                    let _ = completion.send(Err(TransportError::Parse(format!("{e:?}"))));
                }
            },
            ReactorCommand::SubmitTyped {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            } => {
                tracing::debug!(
                    subsystem = "mcu-comms",
                    event = "submit_typed",
                    call_id,
                    resp = %expected_response_name,
                    payload_len = payload.len(),
                    unacked = self.unacked_window.len(),
                    pending_sub = self.pending_submissions.len(),
                    state = ?self.state,
                    "SubmitTyped"
                );
                if let Err(e) = self.dispatch_submission(
                    call_id,
                    payload,
                    expected_response_name,
                    completion.clone(),
                    deadline,
                ) {
                    if matches!(e, TransportError::Io(_)) {
                        self.transition_closed_on_io_fault("handle_command/submit_typed", &e);
                    }
                    let _ = completion.send(Err(e));
                }
            }
            ReactorCommand::Abandon(call_id) => {
                self.awaiting_response.mark_abandoned(call_id);
            }
            ReactorCommand::Shutdown => {
                self.state = ReactorState::Closed;
                self.closed_via_shutdown = true;
            }
            ReactorCommand::MarkExpectedDisconnect => {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "expected_disconnect",
                    transport_pending = self.transport_state.pending.len(),
                    await_n = self.awaiting_response.len(),
                    unacked_n = self.unacked_window.len(),
                    "MarkExpectedDisconnect received"
                );
                self.closed_via_shutdown = true;
            }
            ReactorCommand::AttachHeartbeatCallback(wrapper) => {
                self.event_dispatcher.heartbeat_callback = Some(wrapper.0);
            }
            ReactorCommand::SetMcuLogHook(wrapper) => {
                self.event_dispatcher
                    .set_mcu_log_hook(move |e| (wrapper.0)(e));
            }
            ReactorCommand::SubscribeFault { sender, reply } => {
                let result = self.event_dispatcher.fault_latch.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeTrace { sender, reply } => {
                let result = self.event_dispatcher.trace_ring.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeRuntimeEvents {
                priority,
                bulk,
                reply,
            } => {
                let result = self
                    .event_dispatcher
                    .runtime_event_dispatcher
                    .subscribe(priority, bulk);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeHostEvents { sender, reply } => {
                let result = self
                    .event_dispatcher
                    .host_event_dispatcher
                    .subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::FireAndForget { cmd } => match self.parser.encode(&cmd) {
                Ok(payload) => {
                    let cmd_disp = if cmd.len() > 120 {
                        &cmd[..120]
                    } else {
                        cmd.as_str()
                    };
                    let head: Vec<String> = payload
                        .iter()
                        .take(16)
                        .map(|b| format!("{b:02x}"))
                        .collect();
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "fire_and_forget_sent",
                        cmd = %cmd_disp,
                        payload_len = payload.len(),
                        head = %head.join(","),
                        "FireAndForget encoded OK"
                    );
                    if let Err(e) = self.dispatch_fire_and_forget(payload, false) {
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "fire_and_forget_send_error",
                            cmd = %cmd_disp,
                            error = %e,
                            "FireAndForget dispatch failed"
                        );
                        if matches!(e, TransportError::Io(_)) {
                            self.transition_closed_on_io_fault(
                                "handle_command/fire_and_forget",
                                &e,
                            );
                        }
                    }
                }
                Err(e) => {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "fire_and_forget_encode_error",
                        cmd = ?cmd,
                        error = ?e,
                        "FireAndForget encode failed"
                    );
                }
            },
            ReactorCommand::FireAndForgetTyped { payload } => {
                if let Err(e) = self.dispatch_fire_and_forget(payload, false) {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "fire_and_forget_typed_send_error",
                        error = %e,
                        "FireAndForgetTyped: send error"
                    );
                    if matches!(e, TransportError::Io(_)) {
                        self.transition_closed_on_io_fault(
                            "handle_command/fire_and_forget_typed",
                            &e,
                        );
                    }
                }
            }
            ReactorCommand::McuIdentify {
                completion,
                deadline: _,
            } => {
                let cid = self.transport_state.allocate_correlation_id();
                let frame = build_kalico_identify_frame(cid);
                if self.transport_state.identify_pending.is_some() {
                    let _ = completion.send(Err(TransportError::Backpressure));
                    return;
                }
                self.transport_state.identify_pending = Some(completion);
                if let Err(e) = self.write_frame(&frame) {
                    if matches!(e, TransportError::Io(_)) {
                        self.transition_closed_on_io_fault("handle_command/mcu_identify", &e);
                    }
                    if let Some(c) = self.transport_state.identify_pending.take() {
                        let _ = c.send(Err(e));
                    }
                }
            }
            ReactorCommand::McuCall {
                channel,
                kind,
                body,
                completion,
                deadline,
            } => {
                if !self.transport_state.identified {
                    let _ = completion.send(Err(TransportError::Parse(
                        "kalico transport not yet identified".into(),
                    )));
                    return;
                }
                if self.pending_piece_frames.len() >= PENDING_PIECE_FRAMES_CEILING {
                    let _ = completion.send(Err(TransportError::Backpressure));
                    return;
                }
                let cid = self.transport_state.allocate_correlation_id();
                let frame = build_kalico_frame(channel, kind, cid, &body);
                self.transport_state.pending.insert(
                    cid,
                    PendingMcuCall {
                        completion,
                        deadline,
                    },
                );
                self.pending_piece_frames.push_back((cid, frame));
                self.drain_piece_frames();
            }
            ReactorCommand::GetClockAndDeliver => match self.parser.encode("get_clock") {
                Ok(payload) => {
                    // The RAW send stamp is captured inside
                    // dispatch_fire_and_forget at the actual wire write —
                    // never here, where the frame may still queue behind a
                    // busy link for milliseconds.
                    if let Err(e) = self.dispatch_fire_and_forget(payload, true) {
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "get_clock_async_send_error",
                            error = %e,
                            "GetClockAndDeliver dispatch failed"
                        );
                        if matches!(e, TransportError::Io(_)) {
                            self.transition_closed_on_io_fault(
                                "handle_command/get_clock_and_deliver",
                                &e,
                            );
                        }
                    }
                }
                Err(e) => {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "get_clock_async_encode_error",
                        error = ?e,
                        "GetClockAndDeliver: encode 'get_clock' failed"
                    );
                }
            },
            ReactorCommand::Noop => {}
            ReactorCommand::RegisterInterceptor {
                msg_name,
                oid,
                callback,
                reply,
            } => {
                let id = self.interceptors.register(msg_name, oid, callback);
                let _ = reply.send(id);
            }
            ReactorCommand::UnregisterInterceptor { id } => {
                self.interceptors.unregister(id);
            }
        }
    }
}

impl Reactor {
    fn fail_pending_on_mcu_shutdown(
        &mut self,
        response_name: &str,
        params: &crate::transport::MessageParams,
    ) {
        if self.awaiting_response.len() == 0 && self.pending_submissions.is_empty() {
            return;
        }
        let reason = params
            .fields
            .get("static_string_id")
            .and_then(|v| match v {
                crate::transport::MessageValue::U32(n) => Some(*n as i32),
                crate::transport::MessageValue::I32(n) => Some(*n),
                _ => None,
            })
            .and_then(|id| self.parser.static_strings.get(&id).cloned())
            .unwrap_or_else(|| format!("unresolved reason ({response_name})"));
        tracing::error!(
            subsystem = "mcu-comms",
            event = "mcu_shutdown_fail_fast",
            response = %response_name,
            %reason,
            awaiting = self.awaiting_response.len(),
            pending = self.pending_submissions.len(),
            "MCU reports shutdown; failing pending calls instead of timing out"
        );
        for entry in self.awaiting_response.drain_all() {
            let _ = entry
                .completion
                .send(Err(TransportError::McuShutdown(reason.clone())));
        }
        for p in self.pending_submissions.drain(..) {
            let _ = p
                .completion
                .send(Err(TransportError::McuShutdown(reason.clone())));
        }
    }

    fn flush_all_completions(&mut self) {
        self.pending_clock_sent_raw = None;
        for entry in self.awaiting_response.drain_all() {
            let _ = entry.completion.send(Err(TransportError::Closed));
        }
        self.unacked_window.clear();
        for p in self.pending_submissions.drain(..) {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        self.pending_fire_and_forget.clear();
        self.pending_piece_frames.clear();
        self.pending_outbound_order.clear();

        let drained: Vec<PendingMcuCall> = self
            .transport_state
            .pending
            .drain()
            .map(|(_, v)| v)
            .collect();
        for p in drained {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        if let Some(c) = self.transport_state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Closed));
        }
    }

    pub(crate) fn gc_transport_pending(&mut self) {
        let now = self.clock.now();
        let expired: Vec<u32> = self
            .transport_state
            .pending
            .iter()
            .filter_map(|(cid, p)| if p.deadline <= now { Some(*cid) } else { None })
            .collect();
        for cid in expired {
            if let Some(p) = self.transport_state.pending.remove(&cid) {
                let _ = p.completion.send(Err(TransportError::Timeout));
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum TickOutcome {
    Continue,
    Closed,
}

impl Reactor {
    pub fn run(&mut self) {
        loop {
            if matches!(self.tick_once(), TickOutcome::Closed) {
                break;
            }
        }
    }

    pub fn exited_gracefully(&self) -> bool {
        self.closed_via_shutdown
    }

    pub fn tick_once(&mut self) -> TickOutcome {
        let t_tick = std::time::Instant::now();

        let s1 = std::time::Instant::now();
        for _ in 0..MAX_SUBMITS_PER_ITER {
            match self.submission_rx.try_recv() {
                Ok(cmd) => self.handle_command(cmd),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.state = ReactorState::Closed;
                    break;
                }
            }
        }

        let t_step1 = s1.elapsed();

        let s2 = std::time::Instant::now();
        self.poll_serial();
        let t_step2 = s2.elapsed();

        let s3 = std::time::Instant::now();
        self.drain_pending_submissions();
        let t_step3 = s3.elapsed();

        let s3b = std::time::Instant::now();
        self.drain_piece_frames();
        let t_step3b = s3b.elapsed();

        let s4 = std::time::Instant::now();
        if let Some(front) = self.unacked_window.front() {
            let now = self.clock.now();
            if now >= front.sent_at + self.rtt.current_rto() {
                let unacked_n = self.unacked_window.len();
                let front_seq = front.seq;
                let rto_ms = self.rtt.current_rto().as_millis() as u64;
                let gap_since_recv_ms = now.duration_since(self.last_recv_time).as_millis() as u64;
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "retransmit_timeout",
                    front_seq,
                    unacked_n,
                    rto_ms,
                    gap_since_recv_ms,
                    "[retransmit] RTO fired, resending oldest unacked frame — \
                     gap_since_recv_ms = time since any inbound (corrupt frames count as inbound): \
                     large/growing = link silent; small = inbound alive but no valid ACK (corruption/desync)"
                );
                if let Err(e) = self.write_retransmit(RetransmitTrigger::TimeoutDriven) {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "retransmit",
                        front_seq = front_seq,
                        unacked_n = unacked_n,
                        error = ?e,
                        "retransmit error"
                    );
                    if matches!(e, TransportError::Io(_)) {
                        self.transition_closed_on_io_fault("tick_once/retransmit", &e);
                    }
                }
            }
        }
        let t_step4 = s4.elapsed();

        if let Some(fault) = self.pending_host_fault.take() {
            self.event_dispatcher.fault_latch.dispatch(fault);
        }

        self.event_dispatcher.host_event_dispatcher.drain_pending();

        let now = self.clock.now();
        let evicted = self.awaiting_response.evict_expired(now);
        for entry in evicted {
            let _ = entry
                .completion
                .send(Err(TransportError::DispatcherTimeout));
        }

        self.gc_transport_pending();

        if self.state == ReactorState::Closed {
            self.flush_all_completions();
            return TickOutcome::Closed;
        }

        let dt_tick = t_tick.elapsed();
        if dt_tick > std::time::Duration::from_millis(5) {
            tracing::debug!(
                subsystem = "mcu-comms",
                event = "slow_tick",
                dt_ms = dt_tick.as_secs_f64() * 1000.0,
                step1_ms = t_step1.as_secs_f64() * 1000.0,
                step2_ms = t_step2.as_secs_f64() * 1000.0,
                step3_ms = t_step3.as_secs_f64() * 1000.0,
                step3b_ms = t_step3b.as_secs_f64() * 1000.0,
                step4_ms = t_step4.as_secs_f64() * 1000.0,
                "tick_once exceeded 5ms"
            );
        }
        TickOutcome::Continue
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod a1_seq_wrap;

#[cfg(test)]
mod a2_nak_rto;

#[cfg(test)]
mod a4_nak_submit_race;

#[cfg(test)]
mod a3_awaiting_response_gc;

#[cfg(test)]
mod piece_priority;

#[cfg(test)]
mod a8_fire_and_forget_backpressure;

#[cfg(test)]
mod fire_and_forget_typed_routing;

#[cfg(test)]
mod io_fault_propagation;

#[cfg(test)]
mod a9_mcu_shutdown_fail_fast;
