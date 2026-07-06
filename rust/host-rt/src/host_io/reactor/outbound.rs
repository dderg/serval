use std::collections::VecDeque;
use std::time::Instant;

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
