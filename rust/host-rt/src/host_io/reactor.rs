use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;

use crate::clock::{Clock, RealClock};
use crate::host_io::ReactorCommand;
use crate::host_io::events::EventDispatcher;
use crate::host_io::fire_and_forget_depth::FireAndForgetDepth;
use crate::host_io::identify::IdentifySeqState;
use crate::host_io::mcu_session::McuTransportState;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::rtt::RttEstimator;
use crate::host_io::runtime_events::{FaultEvent, StatusEvent};
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::host_io::window::{AwaitingResponse, UnackedWindow};
use crate::transport::TransportError;

mod command;
mod inbound;
mod io_fault;
mod lifecycle;
mod outbound;
mod seq_window;

use outbound::OutboundQueues;
use seq_window::SeqWindow;

pub struct Reactor {
    pub(crate) io: SerialFrameIo,
    pub(crate) parser: Arc<MsgProtoParser>,
    pub(crate) submission_rx: Receiver<ReactorCommand>,
    pub(crate) unacked_window: UnackedWindow,
    pub(crate) awaiting_response: AwaitingResponse,
    pub(crate) rtt: RttEstimator,
    pub(crate) event_dispatcher: EventDispatcher,

    pub(crate) seq_window: SeqWindow,

    pub(crate) state: ReactorState,

    pub(crate) closed_via_shutdown: bool,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) outbound: OutboundQueues,

    /// When `get_clock_async` is in flight: the CLOCK_MONOTONIC_RAW sent-time
    /// captured before the frame was written to wire.  The next unsolicited
    /// "clock" response matching this will be delivered as a PassthroughResponse
    /// with RAW RTT stamps rather than going through the generic path.
    pub(crate) pending_clock_sent_raw: Option<f64>,

    pub(crate) zero_byte_first_seen: Option<Instant>,
    pub(crate) last_recv_time: Instant,
    pub(crate) last_write_time: Instant,
    pub(crate) zero_byte_consec: u32,
    pub(crate) clock: Arc<dyn Clock>,
    pub(crate) transport_state: McuTransportState,
    pub(crate) interceptors: crate::host_io::interceptor::InterceptorTable,
    pub(crate) mcu_label: Arc<str>,
    pub(crate) last_ack_age_warn: Instant,
    pub(crate) worst_ack_age: std::time::Duration,
    pub(crate) last_ff_wait_warn: Instant,
    pub(crate) last_channel_wait_warn: Instant,
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
        fire_and_forget_depth: Arc<FireAndForgetDepth>,
    ) -> Self {
        Self::new_with_clock(
            io,
            parser,
            submission_rx,
            status_snapshot,
            seq,
            config,
            Arc::new(RealClock),
            fire_and_forget_depth,
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
        fire_and_forget_depth: Arc<FireAndForgetDepth>,
    ) -> Self {
        let mcu_label: Arc<str> = config.mcu_label.as_deref().unwrap_or("unknown").into();
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
            seq_window: SeqWindow::new(seq.next_send_seq_abs, seq.mcu_receive_seq_abs),
            state: ReactorState::Active,
            closed_via_shutdown: false,
            pending_host_fault: None,
            pending_clock_sent_raw: None,
            outbound: OutboundQueues::new(fire_and_forget_depth),
            zero_byte_first_seen: None,
            last_recv_time: clock.now(),
            last_write_time: clock.now(),
            zero_byte_consec: 0,
            clock,
            transport_state: McuTransportState::default(),
            interceptors: crate::host_io::interceptor::InterceptorTable::new(),
            last_ack_age_warn: Instant::now(),
            worst_ack_age: std::time::Duration::ZERO,
            last_ff_wait_warn: Instant::now(),
            last_channel_wait_warn: Instant::now(),
            mcu_label,
        }
    }

    pub fn mcu_label(&self) -> &str {
        &self.mcu_label
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
            Arc::new(FireAndForgetDepth::default()),
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RetransmitTrigger {
    NakDriven,
    TimeoutDriven,
}

const PENDING_SUBMISSION_CEILING: usize = 256;
const MAX_RETRY_COUNT: u32 = 8;

// Retry exhaustion alone is not sufficient to declare Closed: under Renode
// (1 µs quantum) a long-running MCU command can stall status emission for
// several seconds wall while the wire remains healthy. Only close when
// retry exhaustion coincides with genuine MCU silence.
const MCU_SILENCE_FOR_CLOSE: Duration = Duration::from_secs(120);

const MAX_SUBMITS_PER_ITER: usize = 4;
const READ_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(1);
const ZERO_BYTE_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(100);

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
                    self.close_if_io_fault("tick_once/retransmit", &e);
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
mod a8_fire_and_forget_backpressure;

#[cfg(test)]
mod fire_and_forget_typed_routing;

#[cfg(test)]
mod io_fault_propagation;

#[cfg(test)]
mod a9_mcu_shutdown_fail_fast;
