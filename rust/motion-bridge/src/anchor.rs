use std::sync::OnceLock;

const CONTIGUITY_EPS: f64 = 1e-6;

const LEAD_DEFAULT_SECS: f64 = 0.25;
const LEAD_MAX_SECS: f64 = 30.0;

static LEAD_SECS: OnceLock<f64> = OnceLock::new();

/// Returns the host dispatch-latency/jitter budget in seconds.
///
/// Read once from `KALICO_ANCHOR_LEAD_SECS` on first call; subsequent calls
/// return the cached value.  Absent → 0.25 s.  Set-but-unparsable or outside
/// (0.0, 30.0] → panic (fail loud at bridge init).
pub fn lead_secs() -> f64 {
    *LEAD_SECS.get_or_init(|| {
        match std::env::var("KALICO_ANCHOR_LEAD_SECS") {
            Err(_) => LEAD_DEFAULT_SECS,
            Ok(raw) => {
                let v: f64 = raw.trim().parse().unwrap_or_else(|_| {
                    panic!(
                        "KALICO_ANCHOR_LEAD_SECS={raw:?} is not a valid f64 — \
                         set it to a number in (0.0, 30.0] or leave it unset"
                    )
                });
                assert!(
                    v > 0.0 && v <= LEAD_MAX_SECS,
                    "KALICO_ANCHOR_LEAD_SECS={v} is out of range (0.0, {LEAD_MAX_SECS}] — \
                     use a positive value no greater than {LEAD_MAX_SECS} s"
                );
                v
            }
        }
    })
}

#[derive(Debug, Clone, Copy)]
pub struct SegmentLate {
    pub scheduled_host: f64,
    pub host_now: f64,
    pub gap_s: f64,
    pub seg_t_start: f64,
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
        }
    }

    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> Result<(f64, bool), SegmentLate> {
        let lead = lead_secs();
        let reanchor = match self.t0 {
            None => true,
            Some(t0) => {
                let timeline_reset = seg_t_start + CONTIGUITY_EPS < self.last_t_end;
                let starvation = t0 + seg_t_start < host_now;

                if starvation && !timeline_reset {
                    let scheduled_host = t0 + seg_t_start;
                    let gap_s = host_now - scheduled_host;
                    return Err(SegmentLate {
                        scheduled_host,
                        host_now,
                        gap_s,
                        seg_t_start,
                    });
                }

                timeline_reset
            }
        };

        if reanchor {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "backward-jump",
            };
            self.t0 = Some(host_now + lead - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(host_now, t0, seg_t_start, condition, "[anchor-decision]");
        }
        self.last_t_end = seg_t_end;
        Ok((self.t0.unwrap(), reanchor))
    }
}

#[cfg(test)]
mod tests;
