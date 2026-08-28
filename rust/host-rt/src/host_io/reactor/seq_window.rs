use crate::host_io::reactor::RetransmitTrigger;
use crate::host_io::reactor::{MAX_RETRY_COUNT, MCU_SILENCE_FOR_CLOSE, Reactor, ReactorState};
use crate::transport::TransportError;
use runtime::error::FaultCode;

pub(crate) struct SeqWindow {
    pub(crate) send_seq: u64,
    pub(crate) receive_seq: u64,
    pub(crate) last_ack_seq: u64,
    pub(crate) ignore_nak_seq: u64,
    pub(crate) retransmit_seq: u64,
    pub(crate) rtt_sample_seq: u64,
    pub(crate) rtt_sample_armed: bool,
}

impl SeqWindow {
    pub(crate) fn new(send_seq: u64, receive_seq: u64) -> Self {
        Self {
            send_seq,
            receive_seq,
            last_ack_seq: receive_seq.saturating_sub(1),
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            rtt_sample_armed: false,
        }
    }

    pub(crate) fn next_send_seq(&mut self) -> u64 {
        let seq = self.send_seq;
        self.send_seq += 1;
        seq
    }

    pub(crate) fn reset_to(&mut self, seq: u64) {
        self.send_seq = seq;
        self.receive_seq = seq;
    }

    pub(crate) fn arm_rtt_sample(&mut self, seq: u64) {
        if !self.rtt_sample_armed {
            self.rtt_sample_seq = seq;
            self.rtt_sample_armed = true;
        }
    }

    pub(crate) fn rtt_sample_matches(&self, entry_seq: u64) -> bool {
        self.rtt_sample_armed && entry_seq >= self.rtt_sample_seq
    }

    pub(crate) fn disarm_rtt_sample(&mut self) {
        self.rtt_sample_armed = false;
    }

    pub(crate) fn set_ignore_nak_for_retransmit(&mut self, trigger: RetransmitTrigger) {
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
    }
}

impl Reactor {
    pub(super) fn update_receive_seq(&mut self, rseq: u64) -> Result<(), TransportError> {
        if self.unacked_window.is_empty() {
            self.seq_window.reset_to(rseq);
            return Ok(());
        }
        let popped = self.unacked_window.pop_acked(rseq);
        if let Some(oldest) = popped.first() {
            let now = self.clock.now();
            let age = now - oldest.sent_at;
            if age > self.worst_ack_age {
                self.worst_ack_age = age;
            }
            if now.duration_since(self.last_ack_age_warn).as_millis() >= 500 {
                let worst = self.worst_ack_age;
                self.last_ack_age_warn = now;
                self.worst_ack_age = std::time::Duration::ZERO;
                if worst > std::time::Duration::from_millis(1) {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "ack_age_high",
                        mcu = %self.mcu_label,
                        age_ms = worst.as_millis() as u64,
                        age_us = worst.as_micros() as u64,
                        popped = popped.len(),
                        unacked_after = self.unacked_window.len(),
                        "worst ack round trip in the last window - the \
                         12-frame unacked window turns over at this cadence"
                    );
                }
            }
        }
        for entry in &popped {
            if self.seq_window.rtt_sample_matches(entry.seq) {
                let rtt = self.clock.now() - entry.sent_at;
                self.rtt.update(rtt);
                self.seq_window.disarm_rtt_sample();
                break;
            }
        }
        self.seq_window.receive_seq = rseq;
        Ok(())
    }

    pub(crate) fn handle_ack_nak(&mut self, wire_seq_nibble: u8) -> Result<(), TransportError> {
        let rseq =
            crate::host_io::wire::decode_absolute(self.seq_window.receive_seq, wire_seq_nibble);

        if rseq > self.seq_window.receive_seq {
            self.update_receive_seq(rseq)?;
        }

        if self.seq_window.last_ack_seq < rseq {
            self.seq_window.last_ack_seq = rseq;
        } else if rseq > self.seq_window.ignore_nak_seq && !self.unacked_window.is_empty() {
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

        self.seq_window.set_ignore_nak_for_retransmit(trigger);

        let now = self.clock.now();
        let silence = now.duration_since(self.last_recv_time);
        for entry in self.unacked_window.iter_mut() {
            entry.retry_count += 1;
            entry.sent_at = now;
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
