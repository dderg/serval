use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Instant;

use crate::host_io::fire_and_forget_depth::{FIRE_AND_FORGET_HIGH_WATER, FireAndForgetDepth};
use crate::host_io::reactor::{PENDING_SUBMISSION_CEILING, Reactor};
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

pub(crate) struct OutboundQueues {
    pub(crate) pending_submissions: VecDeque<PendingSubmission>,
    /// Queued fire-and-forget payloads; the bool marks a `get_clock` frame
    /// whose RAW send stamp is captured at the actual wire write.
    pub(crate) pending_fire_and_forget: VecDeque<(Vec<u8>, bool)>,
    pub(crate) pending_outbound_order: VecDeque<PendingOutboundKind>,
    pub(crate) fire_and_forget_depth: Arc<FireAndForgetDepth>,
}

impl OutboundQueues {
    pub(crate) fn new(fire_and_forget_depth: Arc<FireAndForgetDepth>) -> Self {
        Self {
            pending_submissions: VecDeque::new(),
            pending_fire_and_forget: VecDeque::new(),
            pending_outbound_order: VecDeque::new(),
            fire_and_forget_depth,
        }
    }

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
        self.publish_fire_and_forget_depth();
    }

    pub(crate) fn pop_fire_and_forget(&mut self) -> Option<(Vec<u8>, bool)> {
        let front = self.pending_fire_and_forget.pop_front();
        self.publish_fire_and_forget_depth();
        front
    }

    fn publish_fire_and_forget_depth(&self) {
        self.fire_and_forget_depth
            .publish(self.pending_fire_and_forget.len());
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
        // which times out unrelated transactions. The kernel tty buffer queues
        // the bytes.
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
            if self.outbound.pending_fire_and_forget.len() == FIRE_AND_FORGET_HIGH_WATER {
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "fire_and_forget_high_water",
                    queued_blocks = self.outbound.pending_fire_and_forget.len(),
                    high_water = FIRE_AND_FORGET_HIGH_WATER,
                    "fire-and-forget queue past its high water mark; bulk senders are being refused"
                );
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
                    let Some((payload, is_get_clock)) = self.outbound.pop_fire_and_forget() else {
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
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "fire_and_forget_redispatch_error",
                            error = %e,
                            "drain_pending_submissions: queued fire-and-forget block lost to a \
                             non-IO write error"
                        );
                    }
                }
            }
        }
    }
}
