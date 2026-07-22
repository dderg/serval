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

/// The anchor's pure verdict for a segment, separated from its side effects
/// so the fatal verdicts can be asserted in a unit test without the abort.
#[derive(Clone, Copy, Debug, PartialEq)]
enum AnchorClass {
    Reposition,
    Continuation,
    /// Below the margin floor but the previous segment ended at rest: a
    /// legitimate idle resume, re-anchored forward with `margin_s` recorded.
    IdleResume {
        margin_s: f64,
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

/// The simulator drives the MCU on a virtual clock that legally races ahead
/// of the host projection (mirrors `pump_past_guard_secs` and the MCU's
/// `CONFIG_MCU_SIM` timer-in-past gating), so a host-side underrun there is
/// infrastructure jitter, not a producer that fell behind — recover instead
/// of aborting. On real hardware there is no such slip and the abort stands.
fn anchor_faults_recover_in_sim() -> bool {
    static SIM: std::sync::LazyLock<bool> =
        std::sync::LazyLock::new(|| std::env::var_os("MCU_SIM_SOCK_DIR").is_some());
    *SIM
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
    last_ends_at_rest: bool,
    lead_secs: f64,
    /// When true, a mid-motion underrun/low-margin re-anchors instead of
    /// aborting; set only under the simulator's racing virtual clock.
    recover_faults: bool,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            last_ends_at_rest: true,
            lead_secs: DEFAULT_LEAD_SECS,
            recover_faults: anchor_faults_recover_in_sim(),
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
        if margin_s >= LOW_MARGIN_WARN_SECS {
            AnchorClass::Continuation
        } else if self.last_ends_at_rest {
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
        let epoch = match self.classify(seg_t_start, host_now) {
            AnchorClass::Reposition => StreamEpoch::Reposition,
            AnchorClass::Continuation => StreamEpoch::Continuation,
            AnchorClass::IdleResume { margin_s } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "anchor_idle_resume",
                    margin_s,
                    seg_t_start,
                    "[anchor] resuming from rest across an idle gap — \
                     re-anchoring forward"
                );
                StreamEpoch::Reanchor
            }
            AnchorClass::UnderrunFatal { gap_s, t0 } if self.recover_faults => {
                tracing::warn!(
                    subsystem = "motion",
                    event = "anchor_underrun_sim_recover",
                    gap_s,
                    seg_t_start,
                    t0,
                    "[anchor-underrun] playhead overran the committed end \
                     mid-motion — re-anchoring (simulator virtual-clock slip, \
                     fatal on real hardware)"
                );
                StreamEpoch::Reanchor
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
            AnchorClass::LowMarginFatal { margin_s, t0 } if self.recover_faults => {
                tracing::warn!(
                    subsystem = "motion",
                    event = "anchor_low_margin_sim_recover",
                    margin_s,
                    seg_t_start,
                    t0,
                    "[anchor] mid-motion margin below the transport floor — \
                     re-anchoring (simulator virtual-clock slip, fatal on real \
                     hardware)"
                );
                StreamEpoch::Reanchor
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
