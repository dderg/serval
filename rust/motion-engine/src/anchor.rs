pub(crate) const CONTIGUITY_EPS: f64 = 1e-6;
pub const DEFAULT_LEAD_SECS: f64 = 0.25; // This is the lead time for first piece that we emit

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

    /// Map a committed segment's stream time onto the absolute host/MCU clock.
    /// `host_now` is the current playhead (MCU `est_print_time` projected into
    /// the host-seconds domain). Returns `(t0, fresh)` where the segment's
    /// absolute start is `t0 + seg_t_start` and `fresh` marks a (re)anchor.
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
    ) -> (f64, bool) {
        let reanchor = match self.t0 {
            None => true,
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
                timeline_reset || underrun
            }
        };

        if reanchor {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "reanchor",
            };
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(host_now, t0, seg_t_start, condition, "[anchor-decision]");
        }
        self.last_t_end = seg_t_end;
        (self.t0.unwrap(), reanchor)
    }

    pub fn t0(&self) -> Option<f64> {
        self.t0
    }
}

#[cfg(test)]
mod tests;
