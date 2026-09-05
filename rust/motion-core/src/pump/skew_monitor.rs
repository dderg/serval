/// Verdict tiers for the host-projected mcu clock measured against the clock
/// the mcu itself stamped on a barrier ack. The projection is what every send
/// margin and lateness guard is computed from; when it diverges from the
/// mcu's own clock the pipeline is pacing against a lie.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewVerdict {
    InBounds,
    Warn,
    Fatal,
}

/// The echo is stamped when the mcu processes the barrier and compared when
/// the reply lands, so the projection legitimately leads it by the return
/// latency plus queue time — hence the asymmetric window.
pub const SKEW_WARN_LOW_SECS: f64 = -0.001;
pub const SKEW_WARN_HIGH_SECS: f64 = 0.020;

/// Beyond this the projection is not noise but a broken clock model; three
/// consecutive echoes rule out a single delayed reply.
pub const SKEW_FATAL_SECS: f64 = 0.100;
pub const SKEW_FATAL_CONSECUTIVE: u32 = 3;

#[derive(Debug, Default)]
pub struct SkewMonitor {
    consecutive_beyond_fatal: u32,
}

impl SkewMonitor {
    pub fn observe(&mut self, skew_secs: f64) -> SkewVerdict {
        if skew_secs.abs() > SKEW_FATAL_SECS {
            self.consecutive_beyond_fatal += 1;
            if self.consecutive_beyond_fatal >= SKEW_FATAL_CONSECUTIVE {
                return SkewVerdict::Fatal;
            }
            return SkewVerdict::Warn;
        }
        self.consecutive_beyond_fatal = 0;
        if (SKEW_WARN_LOW_SECS..=SKEW_WARN_HIGH_SECS).contains(&skew_secs) {
            SkewVerdict::InBounds
        } else {
            SkewVerdict::Warn
        }
    }
}
