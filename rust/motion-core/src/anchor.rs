pub(crate) const CONTIGUITY_EPS: f64 = 1e-6;
pub const DEFAULT_LEAD_SECS: f64 = 0.25;

/// A continuing stream whose next segment starts closer to the playhead than
/// this cannot reliably reach the drive before its start time (transport +
/// send latency ate a 1.5 ms margin on the bench, latching a -308
/// PieceStartInPast drive fault at the post-homing continuation). Hitting the
/// floor while the machine sits at rest is an idle resume and re-anchors;
/// hitting it mid-motion means continuous motion is already lost — fatal.
pub const LOW_MARGIN_WARN_SECS: f64 = 0.020;

/// How a segment relates to the anchored stream it lands in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamEpoch {
    /// Same anchored timeline; the segment extends the running stream.
    Continuation,
    /// First segment or a timeline reset (idle restart): the stream starts
    /// over and its position is legitimately redefined — there is no previous
    /// end to hold it contiguous with.
    Reposition,
    /// Resume-from-rest re-anchor: the timeline shifted forward across an
    /// idle gap, but the motion content is the same continuous track parked
    /// at rest, so position continuity across the seam is still mandatory.
    Reanchor,
}

impl StreamEpoch {
    #[must_use]
    pub fn is_fresh(self) -> bool {
        self != Self::Continuation
    }

    #[must_use]
    pub fn position_redefined(self) -> bool {
        self == Self::Reposition
    }
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
    last_ends_at_rest: bool,
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            last_ends_at_rest: true,
            lead_secs: DEFAULT_LEAD_SECS,
        }
    }

    /// Map a committed segment's stream time onto the absolute host clock.
    /// `host_now` is the current playhead in host-monotonic seconds
    /// (`router.host_now_secs()`); the dispatched lead reconciles against the
    /// same clock. `ends_at_rest` reports whether this segment's trajectory
    /// is at rest at its end — it decides how the *next* segment's underrun
    /// is judged. Returns `(t0, fresh)` where the segment's absolute start is
    /// `t0 + seg_t_start` and `fresh` marks a (re)anchor.
    ///
    /// The timeline floats ahead of the playhead and is only re-anchored when
    /// it must be: the first segment, a backward jump (idle restart), or an
    /// idle resume — the playhead overran the committed end (or the margin
    /// fell below `LOW_MARGIN_WARN_SECS`) while the machine sat **at rest**,
    /// where a forward re-anchor loses nothing. The same conditions arriving
    /// while the previous segment ended in motion mean the producer fell
    /// behind mid-stream and continuous motion is already lost: that aborts
    /// the process instead of hiding the gap behind a re-anchor. A healthy
    /// stream never gets near the floor because the committed frontier stays
    /// `buffer_time` ahead of the playhead.
    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
        ends_at_rest: bool,
    ) -> (f64, StreamEpoch) {
        let epoch = match self.t0 {
            None => StreamEpoch::Reposition,
            Some(t0) => {
                if seg_t_start + CONTIGUITY_EPS < self.last_t_end {
                    StreamEpoch::Reposition
                } else {
                    let margin_s = t0 + seg_t_start - host_now;
                    if margin_s >= LOW_MARGIN_WARN_SECS {
                        StreamEpoch::Continuation
                    } else if self.last_ends_at_rest {
                        tracing::info!(
                            subsystem = "motion",
                            event = "anchor_idle_resume",
                            margin_s,
                            seg_t_start,
                            "[anchor] resuming from rest across an idle gap — \
                             re-anchoring forward"
                        );
                        StreamEpoch::Reanchor
                    } else if margin_s < 0.0 {
                        tracing::error!(
                            subsystem = "motion",
                            event = "anchor_underrun",
                            gap_s = -margin_s,
                            seg_t_start,
                            "[anchor-underrun] playhead overran the committed \
                             end mid-motion — continuous motion is lost"
                        );
                        crate::worker::fatal(&format!(
                            "anchor underrun: playhead overran the committed \
                             end by {:.6}s at seg_t_start={seg_t_start:.6} \
                             while the trajectory was mid-motion — the \
                             producer fell behind playback",
                            -margin_s
                        ));
                    } else {
                        tracing::error!(
                            subsystem = "motion",
                            event = "anchor_low_margin",
                            margin_s,
                            host_now,
                            t0,
                            seg_t_start,
                            seg_t_end,
                            last_t_end = self.last_t_end,
                            "[anchor] mid-motion continuation margin below the \
                             transport floor — this piece cannot reliably beat \
                             its start time to the drive"
                        );
                        crate::worker::fatal(&format!(
                            "anchor low margin: mid-motion continuation margin \
                             {margin_s:.6}s is below the \
                             {LOW_MARGIN_WARN_SECS}s transport floor at \
                             seg_t_start={seg_t_start:.6}"
                        ));
                    }
                }
            }
        };

        if epoch.is_fresh() {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "reanchor",
            };
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(
                subsystem = "motion",
                event = "anchor_decision",
                host_now,
                t0,
                seg_t_start,
                seg_t_end,
                lead_secs = self.lead_secs,
                last_t_end = self.last_t_end,
                condition,
                "[anchor-decision] fresh anchor"
            );
        }
        self.last_t_end = seg_t_end;
        self.last_ends_at_rest = ends_at_rest;
        (self.t0.unwrap(), epoch)
    }

    pub fn t0(&self) -> Option<f64> {
        self.t0
    }
}

#[cfg(test)]
mod tests;
