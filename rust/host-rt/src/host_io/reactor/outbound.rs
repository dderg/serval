use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::host_io::CommandTiming;
use crate::host_io::reactor::{
    PENDING_FIRE_AND_FORGET_CEILING, PENDING_SUBMISSION_CEILING, PIECE_OUTQ_BUDGET_BYTES, Reactor,
};
use crate::transport::TransportError;

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

pub(crate) enum ScheduledPayload {
    FireAndForget(Vec<u8>),
    Submission {
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion:
            std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
        timeout: Duration,
    },
}

pub(crate) struct ScheduledCommand {
    pub timing: CommandTiming,
    pub payload: ScheduledPayload,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct ClockEstimate {
    freq: f64,
    offset: f64,
    last_clock: u64,
}

impl ClockEstimate {
    pub(crate) fn from_raw(
        freq: f64,
        offset_raw: f64,
        last_clock: u64,
        now: Instant,
    ) -> Result<Self, TransportError> {
        if !freq.is_finite() || freq <= 0.0 || !offset_raw.is_finite() {
            return Err(TransportError::Parse("invalid MCU clock estimate".into()));
        }
        let offset =
            offset_raw - (crate::clock::monotonic_raw_secs() - crate::clock::instant_to_f64(now));
        Ok(Self {
            freq,
            offset,
            last_clock,
        })
    }

    fn projected_clock(&self, at: Instant) -> Result<u64, TransportError> {
        let projected =
            self.last_clock as f64 + (crate::clock::instant_to_f64(at) - self.offset) * self.freq;
        if !projected.is_finite() || !(0.0..=u64::MAX as f64).contains(&projected) {
            return Err(TransportError::Parse(
                "MCU clock estimate projected outside u64".into(),
            ));
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Ok(projected as u64)
    }
}

#[derive(Default)]
pub(crate) struct OutboundQueues {
    pub(crate) pending_submissions: VecDeque<PendingSubmission>,
    /// Queued fire-and-forget payloads; the bool marks a `get_clock` frame
    /// whose RAW send stamp is captured at the actual wire write.
    pub(crate) pending_fire_and_forget: VecDeque<(Vec<u8>, bool)>,
    /// Piece-channel (motion) frames, keyed by correlation id, awaiting a
    /// shallow kernel tty queue; see `drain_piece_frames` for the priority
    /// rule this enforces.
    pub(crate) pending_piece_frames: VecDeque<(u32, Vec<u8>)>,
    pub(crate) pending_outbound_order: VecDeque<PendingOutboundKind>,
    pub(crate) scheduled_timed: VecDeque<ScheduledCommand>,
    pub(crate) scheduled_background: VecDeque<ScheduledCommand>,
}

impl OutboundQueues {
    pub(crate) fn enqueue_submission(&mut self, submission: PendingSubmission) {
        self.pending_submissions.push_back(submission);
        self.pending_outbound_order
            .push_back(PendingOutboundKind::Submission);
    }

    pub(crate) fn enqueue_fire_and_forget(&mut self, payload: Vec<u8>, is_get_clock: bool) {
        self.pending_fire_and_forget
            .push_back((payload, is_get_clock));
        self.pending_outbound_order
            .push_back(PendingOutboundKind::FireAndForget);
    }
}

impl Reactor {
    pub(crate) fn enqueue_scheduled(
        &mut self,
        timing: CommandTiming,
        payload: ScheduledPayload,
    ) -> Result<(), TransportError> {
        if matches!(timing, CommandTiming::Immediate) {
            return self.dispatch_scheduled(ScheduledCommand { timing, payload });
        }
        let needs_clock = match timing {
            CommandTiming::Timed { .. } => true,
            CommandTiming::Background { min_clock } => min_clock != 0,
            CommandTiming::Immediate => false,
        };
        if needs_clock {
            self.predicted_ack_clock(scheduled_payload_len(&payload))?;
        }
        let queue = match timing {
            CommandTiming::Background { .. } => &mut self.outbound.scheduled_background,
            CommandTiming::Timed { .. } => &mut self.outbound.scheduled_timed,
            CommandTiming::Immediate => unreachable!(),
        };
        if queue.len() >= PENDING_FIRE_AND_FORGET_CEILING {
            return Err(TransportError::Backpressure);
        }
        queue.push_back(ScheduledCommand { timing, payload });
        Ok(())
    }

    fn predicted_ack_clock(&self, payload_len: usize) -> Result<(u64, f64), TransportError> {
        let estimate = self.clock_estimate.ok_or_else(|| {
            TransportError::Parse("timed command has no MCU clock estimate".into())
        })?;
        let frame_len = crate::host_io::wire::MESSAGE_MIN.saturating_add(payload_len);
        let wire_time = self.io.predicted_wire_time(frame_len)?;
        Ok((
            estimate.projected_clock(self.clock.now() + wire_time)?,
            estimate.freq,
        ))
    }

    fn timed_front_is_eligible(&self) -> Result<bool, TransportError> {
        let Some(command) = self.outbound.scheduled_timed.front() else {
            return Ok(false);
        };
        let CommandTiming::Timed {
            min_clock,
            req_clock,
        } = command.timing
        else {
            return Err(TransportError::Parse(
                "non-timed command entered timed queue".into(),
            ));
        };
        let (ack_clock, freq) =
            self.predicted_ack_clock(scheduled_payload_len(&command.payload))?;
        if ack_clock < min_clock {
            return Ok(false);
        }
        if req_clock == 0 {
            return Ok(true);
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let send_ahead_ticks = (freq * 0.100) as u64;
        Ok(req_clock <= ack_clock.saturating_add(send_ahead_ticks))
    }

    fn background_front_is_eligible(&self) -> Result<bool, TransportError> {
        let Some(command) = self.outbound.scheduled_background.front() else {
            return Ok(false);
        };
        let CommandTiming::Background { min_clock } = command.timing else {
            return Err(TransportError::Parse(
                "non-background command entered background queue".into(),
            ));
        };
        if min_clock == 0 {
            return Ok(true);
        }
        let (ack_clock, _) = self.predicted_ack_clock(scheduled_payload_len(&command.payload))?;
        Ok(ack_clock >= min_clock)
    }

    fn dispatch_scheduled(&mut self, command: ScheduledCommand) -> Result<(), TransportError> {
        match command.payload {
            ScheduledPayload::FireAndForget(payload) => {
                self.dispatch_fire_and_forget(payload, false)
            }
            ScheduledPayload::Submission {
                call_id,
                payload,
                expected_response_name,
                completion,
                timeout,
            } => self.dispatch_submission(
                call_id,
                payload,
                expected_response_name,
                completion,
                self.clock.now() + timeout,
            ),
        }
    }

    pub(crate) fn drain_scheduled_commands(&mut self) {
        while !self.unacked_window.is_full() {
            let timed_ready = match self.timed_front_is_eligible() {
                Ok(ready) => ready,
                Err(error) => {
                    let command = self
                        .outbound
                        .scheduled_timed
                        .pop_front()
                        .expect("timed eligibility requires a front command");
                    reject_scheduled(command, error);
                    continue;
                }
            };
            if timed_ready {
                let command = self
                    .outbound
                    .scheduled_timed
                    .pop_front()
                    .expect("timed front was eligible");
                if let Err(error) = self.dispatch_scheduled(command) {
                    self.close_if_io_fault("drain_scheduled_commands", &error);
                    break;
                }
                continue;
            }
            let background_ready = match self.background_front_is_eligible() {
                Ok(ready) => ready,
                Err(error) => {
                    let command = self
                        .outbound
                        .scheduled_background
                        .pop_front()
                        .expect("background eligibility requires a front command");
                    reject_scheduled(command, error);
                    continue;
                }
            };
            if !background_ready {
                break;
            }
            let command = self
                .outbound
                .scheduled_background
                .pop_front()
                .expect("background front was eligible");
            if let Err(error) = self.dispatch_scheduled(command) {
                self.close_if_io_fault("drain_scheduled_commands", &error);
            }
            break;
        }
    }
}

impl Reactor {
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
            if self.outbound.pending_submissions.len() >= PENDING_SUBMISSION_CEILING {
                let _ = completion.send(Err(TransportError::Backpressure));
                return Ok(());
            }
            self.outbound.enqueue_submission(PendingSubmission {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            });
            return Ok(());
        }

        let seq = self.seq_window.next_send_seq();
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

        self.seq_window.arm_rtt_sample(seq);
        Ok(())
    }

    pub(crate) fn dispatch_fire_and_forget(
        &mut self,
        payload: Vec<u8>,
        is_get_clock: bool,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.outbound.pending_fire_and_forget.len() >= PENDING_FIRE_AND_FORGET_CEILING {
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "fire_and_forget_ceiling",
                    ceiling = PENDING_FIRE_AND_FORGET_CEILING,
                    "dispatch_fire_and_forget: pending_fire_and_forget at ceiling; refusing payload"
                );
                return Err(TransportError::Backpressure);
            }
            self.outbound.enqueue_fire_and_forget(payload, is_get_clock);
            return Ok(());
        }
        let seq = self.seq_window.next_send_seq();
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
            let Some(kind) = self.outbound.pending_outbound_order.pop_front() else {
                break;
            };
            match kind {
                PendingOutboundKind::Submission => {
                    let Some(p) = self.outbound.pending_submissions.pop_front() else {
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
                        let is_io =
                            self.close_if_io_fault("drain_pending_submissions/submission", &e);
                        let _ = p.completion.send(Err(e));
                        if is_io {
                            return;
                        }
                    }
                }
                PendingOutboundKind::FireAndForget => {
                    let Some((payload, is_get_clock)) =
                        self.outbound.pending_fire_and_forget.pop_front()
                    else {
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "outbound_order_missing_fire_and_forget",
                            "pending outbound order referenced missing fire-and-forget"
                        );
                        continue;
                    };
                    if let Err(e) = self.dispatch_fire_and_forget(payload, is_get_clock) {
                        if self.close_if_io_fault("drain_pending_submissions/fire_and_forget", &e) {
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
        while !self.outbound.pending_piece_frames.is_empty() {
            match self.io.bytes_to_write() {
                Ok(pending) if pending > PIECE_OUTQ_BUDGET_BYTES => return,
                Ok(_) => {}
                Err(e) => {
                    self.transition_closed_on_io_fault("drain_piece_frames/outq_poll", &e);
                    return;
                }
            }
            let (cid, frame) = self
                .outbound
                .pending_piece_frames
                .pop_front()
                .expect("checked non-empty");
            if let Err(e) = self.write_frame(&frame) {
                self.close_if_io_fault("drain_piece_frames/write_frame", &e);
                if let Some(p) = self.transport_state.pending.remove(&cid) {
                    let _ = p.completion.send(Err(e));
                }
                return;
            }
        }
    }
}

fn scheduled_payload_len(payload: &ScheduledPayload) -> usize {
    match payload {
        ScheduledPayload::FireAndForget(payload) => payload.len(),
        ScheduledPayload::Submission { payload, .. } => payload.len(),
    }
}

fn reject_scheduled(command: ScheduledCommand, error: TransportError) {
    if let ScheduledPayload::Submission { completion, .. } = command.payload {
        let _ = completion.send(Err(error));
    } else {
        tracing::error!(
            subsystem = "mcu-comms",
            event = "scheduled_command_rejected",
            error = %error,
            "scheduled fire-and-forget command rejected"
        );
    }
}
