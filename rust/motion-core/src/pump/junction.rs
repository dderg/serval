use super::AxisKey;
use std::collections::BTreeMap;
use trajectory::ClockedMotorSpan;

pub fn junction_jumps(
    first_start_ticks: u64,
    first_host: f64,
    prev_end_ticks: u64,
    prev_end_host: f64,
    approx_freq_hz: f64,
) -> (f64, f64) {
    let tick_jump_us =
        (first_start_ticks as i64 - prev_end_ticks as i64) as f64 / approx_freq_hz * 1e6;
    let host_jump_us = (first_host - prev_end_host) * 1e6;
    (tick_jump_us, host_jump_us)
}

#[derive(Clone, Copy)]
struct JunctionEnd {
    end_ticks: u64,
    end_host: f64,
    end_pos: f64,
    source_line: u32,
}

pub const JUNCTION_POSITION_LOG_MM: f64 = 0.0125;
pub const JUNCTION_POSITION_FATAL_MM: f64 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct JunctionSeam {
    pub key: AxisKey,
    pub prev_end_pos: f64,
    pub next_start_pos: f64,
    pub prev_end_host: f64,
    pub next_start_host: f64,
    pub prev_source_line: u32,
    pub next_source_line: u32,
    pub prev_end_ticks: u64,
    pub first_start_ticks: u64,
}

impl JunctionSeam {
    #[must_use]
    pub fn jump(&self) -> f64 {
        (self.next_start_pos - self.prev_end_pos).abs()
    }

    #[must_use]
    pub fn is_fatal(&self) -> bool {
        self.jump() >= JUNCTION_POSITION_FATAL_MM
    }
}

fn span_endpoint_position(key: AxisKey, span: &ClockedMotorSpan, t: f64) -> f64 {
    span.signal.position(t).unwrap_or_else(|error| {
        panic!(
            "mcu{} axis{}: dispatched span from line {} does not evaluate at stream time {t}: \
             {error}",
            key.mcu_id, key.axis, span.signal.source_line
        )
    })
}

#[derive(Default)]
pub struct JunctionTracker {
    ends: BTreeMap<AxisKey, JunctionEnd>,
}

impl JunctionTracker {
    pub fn forget(&mut self, key: AxisKey) {
        self.ends.remove(&key);
    }

    pub fn observe(
        &mut self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
        source_line: u32,
    ) -> Option<JunctionSeam> {
        let first = spans.first()?;
        if first.signal.motor_mask != 0 {
            return None;
        }
        let next_start_pos = span_endpoint_position(key, first, first.stream_t_start);
        let seam = self.ends.get(&key).map(|prev| JunctionSeam {
            key,
            prev_end_pos: prev.end_pos,
            next_start_pos,
            prev_end_host: prev.end_host,
            next_start_host: first.start_host,
            prev_source_line: prev.source_line,
            next_source_line: source_line,
            prev_end_ticks: prev.end_ticks,
            first_start_ticks: first.start_clock,
        });
        let last = spans.last().unwrap();
        self.ends.insert(
            key,
            JunctionEnd {
                end_ticks: last.end_clock,
                end_host: last.end_host,
                end_pos: span_endpoint_position(key, last, last.stream_t_end),
                source_line,
            },
        );
        seam
    }

    pub fn observe_msg(
        &mut self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
        epoch: crate::anchor::StreamEpoch,
        source_line: u32,
    ) -> Option<JunctionSeam> {
        if epoch.position_redefined() {
            self.forget(key);
        }
        self.observe(key, spans, source_line)
    }
}

pub(super) fn check_junction_position_continuity(seam: &JunctionSeam) {
    let jump = seam.jump();
    if jump >= JUNCTION_POSITION_LOG_MM {
        tracing::error!(
            subsystem = "motion",
            event = "junction_position_discontinuity",
            key = ?seam.key,
            fatal = jump >= JUNCTION_POSITION_FATAL_MM,
            prev_end = seam.prev_end_pos,
            next_start = seam.next_start_pos,
            jump_mm = jump,
            prev_end_host = seam.prev_end_host,
            next_start_host = seam.next_start_host,
            prev_source_line = seam.prev_source_line,
            next_source_line = seam.next_source_line,
            "[junction-pos] position discontinuity"
        );
    }
    if jump >= JUNCTION_POSITION_FATAL_MM {
        panic!(
            "junction position discontinuity on mcu{} axis{}: prev span ends at \
             {} (host t={:.6}, line {}), next starts at {} (host t={:.6}, line \
             {}), |Δ|={jump}mm — this becomes a one-sample step burst on the MCU \
             (fault -300/-310)",
            seam.key.mcu_id,
            seam.key.axis,
            seam.prev_end_pos,
            seam.prev_end_host,
            seam.prev_source_line,
            seam.next_start_pos,
            seam.next_start_host,
            seam.next_source_line,
        );
    }
}
