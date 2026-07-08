pub(crate) const CONTIGUITY_EPS: f64 = 1e-6;
pub const DEFAULT_LEAD_SECS: f64 = 0.25;

/// How a segment relates to the anchored stream it lands in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamEpoch {
    /// Same anchored timeline; the segment extends the running stream.
    Continuation,
    /// First segment or a timeline reset (idle restart): the stream starts
    /// over and its position is legitimately redefined — there is no previous
    /// end to hold it contiguous with.
    Reposition,
    /// Underrun re-anchor: the timeline shifted forward but the motion content
    /// is the same continuous track, so position continuity across the seam is
    /// still mandatory.
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
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            lead_secs: DEFAULT_LEAD_SECS,
        }
    }

    /// Map a committed segment's stream time onto the absolute host clock.
    /// `host_now` is the current playhead in host-monotonic seconds
    /// (`router.host_now_secs()`); the dispatched lead reconciles against the
    /// same clock. Returns `(t0, fresh)` where the segment's absolute start is
    /// `t0 + seg_t_start` and `fresh` marks a (re)anchor.
    ///
    /// The timeline floats ahead of the playhead and is only re-anchored when it
    /// must be: the first segment, a backward jump (idle restart), or a genuine
    /// **underrun** — the playhead has overrun where this segment was scheduled
    /// (the producer fell behind playback). On underrun we re-anchor forward and
    /// continue, a brief stutter, rather than aborting the print; mainline's
    /// stall-not-crash behaviour. A healthy stream never underruns because the
    /// committed frontier stays `buffer_time` ahead of the playhead.
    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> (f64, StreamEpoch) {
        let epoch = match self.t0 {
            None => StreamEpoch::Reposition,
            Some(t0) => {
                let timeline_reset = seg_t_start + CONTIGUITY_EPS < self.last_t_end;
                let underrun = !timeline_reset && t0 + seg_t_start < host_now;
                if underrun {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "anchor_underrun",
                        gap_s = host_now - (t0 + seg_t_start),
                        seg_t_start,
                        "[anchor-underrun] playhead overran the committed end; \
                         re-anchoring forward (stutter)"
                    );
                }
                if timeline_reset {
                    StreamEpoch::Reposition
                } else if underrun {
                    StreamEpoch::Reanchor
                } else {
                    StreamEpoch::Continuation
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
            tracing::info!(host_now, t0, seg_t_start, condition, "[anchor-decision]");
        }
        self.last_t_end = seg_t_end;
        (self.t0.unwrap(), epoch)
    }

    pub fn t0(&self) -> Option<f64> {
        self.t0
    }
}

#[cfg(test)]
mod tests;
