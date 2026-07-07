use crate::host_io::reactor::RetransmitTrigger;

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
