pub(crate) const CONTIGUITY_EPS: f64 = 1e-6;
pub const DEFAULT_LEAD_SECS: f64 = 0.25;

/// A continuing stream whose next segment starts closer to the playhead than
/// this cannot reliably reach the drive before its start time (transport +
/// send latency ate a 1.5 ms margin on the bench, latching a -308
/// PieceStartInPast drive fault at the post-homing continuation). Hitting the
/// floor while the machine sits at rest is an idle resume and re-anchors;
/// hitting it mid-motion means continuous motion is already lost — fatal.
pub const LOW_MARGIN_WARN_SECS: f64 = 0.020;

/// A stream resuming across a drain-to-rest keeps its standing anchor only
/// while the decayed margin still clears this floor; below it the segment
/// re-anchors forward, which loses nothing at rest. Sized for the measured
/// worst case around endstop reconciles, where the mcu-bound USB link
/// backs up for 50-100 ms and a post-probe continuation sent with the 20 ms
/// mid-motion floor arrives behind its own start clock ("Timer too close").
pub const PARKED_REANCHOR_FLOOR_SECS: f64 = 0.150;

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
    /// The stream time itself jumped forward across a drained-to-rest hole
    /// (a dwell): the standing anchor keeps the timing, but the interval has
    /// no pieces, so per-lane seams downstream must be cut exactly like a
    /// re-anchor. Position continuity across the seam is still mandatory.
    Rejoin,
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

    /// The anchor re-derived `t0` for this epoch: retained absolute-time
    /// state (motion history) no longer maps onto the new timeline.
    #[must_use]
    pub fn retimed(self) -> bool {
        matches!(self, Self::Reposition | Self::Reanchor)
    }
}

/// The anchor's pure verdict for a segment, separated from its side effects
/// so the fatal verdicts can be asserted in a unit test without the abort.
#[derive(Clone, Copy, Debug, PartialEq)]
enum AnchorClass {
    Reposition,
    Continuation {
        margin_s: f64,
    },
    /// Below the margin floor but the previous segment ended at rest: a
    /// legitimate idle resume, re-anchored forward with `margin_s` recorded.
    IdleResume {
        margin_s: f64,
    },
    /// Healthy margin, but the stream time jumped forward across a hole the
    /// previous segment left at rest (a dwell emitted no pieces for the
    /// interval): keep the standing anchor, cut downstream seams.
    Rejoin {
        hole_s: f64,
        margin_s: f64,
    },
    /// A forward stream-time hole while the previous segment ended
    /// mid-motion: the producer dropped part of the trajectory — fatal.
    HoleMidMotionFatal {
        hole_s: f64,
    },
    /// The playhead overran the committed end while mid-motion — fatal.
    UnderrunFatal {
        gap_s: f64,
        t0: f64,
    },
    /// Margin below the transport floor while mid-motion — fatal.
    LowMarginFatal {
        margin_s: f64,
        t0: f64,
    },
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
    parked: bool,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            parked: true,
        }
    }

    /// The anchor's verdict for a segment, before any side effect (logging,
    /// abort, `t0` update). Kept pure so the fatal verdicts are unit-testable
    /// without aborting the process — [`anchor_segment`] is the thin wrapper
    /// that logs, aborts, or updates `t0` from it.
    fn classify(&self, seg_t_start: f64, host_now: f64) -> AnchorClass {
        let Some(t0) = self.t0 else {
            return AnchorClass::Reposition;
        };
        if seg_t_start + CONTIGUITY_EPS < self.last_t_end {
            return AnchorClass::Reposition;
        }
        let margin_s = t0 + seg_t_start - host_now;
        let hole_s = seg_t_start - self.last_t_end;
        let floor_s = if self.parked {
            PARKED_REANCHOR_FLOOR_SECS
        } else {
            LOW_MARGIN_WARN_SECS
        };
        if margin_s >= floor_s {
            if hole_s <= CONTIGUITY_EPS {
                AnchorClass::Continuation { margin_s }
            } else if self.parked {
                AnchorClass::Rejoin { hole_s, margin_s }
            } else {
                AnchorClass::HoleMidMotionFatal { hole_s }
            }
        } else if self.parked {
            AnchorClass::IdleResume { margin_s }
        } else if margin_s < 0.0 {
            AnchorClass::UnderrunFatal {
                gap_s: -margin_s,
                t0,
            }
        } else {
            AnchorClass::LowMarginFatal { margin_s, t0 }
        }
    }

    /// Map a committed segment's stream time onto the absolute host clock.
    /// `host_now` is the current playhead in host-monotonic seconds
    /// (`router.host_now_secs()`); the dispatched lead reconciles against the
    /// same clock. Returns `(t0, fresh)` where the segment's absolute start is
    /// `t0 + seg_t_start` and `fresh` marks a (re)anchor.
    ///
    /// The timeline floats ahead of the playhead and is only re-anchored when
    /// it must be: the first segment, a backward jump (idle restart), or an
    /// idle resume — the playhead overran the committed end (or the margin
    /// fell below `LOW_MARGIN_WARN_SECS`) after the stream drained and
    /// [`Anchor::mark_parked`] declared the machine stopped, where a forward
    /// re-anchor loses nothing. The same conditions arriving with committed
    /// motion past the last park mean the producer fell behind mid-stream and
    /// continuous motion is already lost: that aborts the process instead of
    /// hiding the gap behind a re-anchor. A healthy stream never gets near
    /// the floor because the committed frontier stays `buffer_time` ahead of
    /// the playhead.
    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> (f64, StreamEpoch) {
        let epoch = match self.classify(seg_t_start, host_now) {
            AnchorClass::Reposition => StreamEpoch::Reposition,
            AnchorClass::Continuation { .. } => StreamEpoch::Continuation,
            AnchorClass::IdleResume { margin_s } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "anchor_idle_resume",
                    margin_s,
                    seg_t_start,
                    lead_secs = DEFAULT_LEAD_SECS,
                    "[anchor] resuming from rest across an idle gap — re-anchoring forward"
                );
                StreamEpoch::Reanchor
            }
            AnchorClass::Rejoin { hole_s, margin_s } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "anchor_rejoin",
                    hole_s,
                    margin_s,
                    seg_t_start,
                    "[anchor] stream time jumped a drained-to-rest hole \
                     (dwell) — keeping the standing anchor, cutting \
                     downstream seams"
                );
                StreamEpoch::Rejoin
            }
            AnchorClass::HoleMidMotionFatal { hole_s } => {
                tracing::error!(
                    subsystem = "motion",
                    event = "anchor_hole_mid_motion",
                    hole_s,
                    seg_t_start,
                    last_t_end = self.last_t_end,
                    "[anchor] forward stream-time hole while the previous \
                     segment ended mid-motion — trajectory content is missing"
                );
                crate::worker::fatal(&format!(
                    "anchor hole mid-motion: stream time jumped {hole_s:.6}s \
                     forward at seg_t_start={seg_t_start:.6} while the \
                     previous segment ended in motion — the producer dropped \
                     part of the trajectory"
                ));
            }
            AnchorClass::UnderrunFatal { gap_s, t0 } => {
                tracing::error!(
                    subsystem = "motion",
                    event = "anchor_underrun",
                    gap_s,
                    seg_t_start,
                    "[anchor-underrun] playhead overran the committed end \
                     mid-motion — continuous motion is lost"
                );
                crate::worker::fatal(&format!(
                    "anchor underrun: playhead overran the committed end by \
                     {gap_s:.6}s at seg_t_start={seg_t_start:.6} while the \
                     trajectory was mid-motion (t0={t0:.6}) — the producer \
                     fell behind playback"
                ));
            }
            AnchorClass::LowMarginFatal { margin_s, t0 } => {
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
                     {margin_s:.6}s is below the {LOW_MARGIN_WARN_SECS}s \
                     transport floor at seg_t_start={seg_t_start:.6}"
                ));
            }
        };

        if epoch.retimed() {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "reanchor",
            };
            self.t0 = Some(host_now + DEFAULT_LEAD_SECS - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(
                subsystem = "motion",
                event = "anchor_decision",
                host_now,
                t0,
                seg_t_start,
                seg_t_end,
                lead_secs = DEFAULT_LEAD_SECS,
                last_t_end = self.last_t_end,
                condition,
                "[anchor-decision] fresh anchor"
            );
        }
        self.last_t_end = seg_t_end;
        self.parked = false;
        (self.t0.unwrap(), epoch)
    }

    /// The stream drained: everything committed is planned to rest and the
    /// chains' trailing decay is carried to rest with it, so the machine is
    /// stopped at `last_t_end` and a later resume may re-anchor forward.
    /// Declared by the drain rather than read off the committed track's end
    /// derivative: a trailing derivative-gain stage (pressure advance) leaves
    /// the parked extruder's commanded velocity at `k·ë`, which is nonzero
    /// wherever the profile stops with acceleration still applied — every
    /// stop, once jerk is unlimited.
    pub fn mark_parked(&mut self) {
        self.parked = true;
    }

    pub fn t0(&self) -> Option<f64> {
        self.t0
    }
}

#[cfg(test)]
mod tests;
