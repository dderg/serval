use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use super::sched::{AxisFrame, AxisQueue};
use crate::types::AxisKey;

const EMIT_INTERVAL: Duration = Duration::from_secs(1);

pub(super) fn front_margin_secs(start_time: u64, mcu_now: u64, freq: f64) -> f64 {
    (start_time as i64).wrapping_sub(mcu_now as i64) as f64 / freq
}

struct McuMarginWindow {
    window_start: Instant,
    sends: u32,
    min_margin_secs: f64,
    worst_axis: u8,
    worst_occupancy: u32,
    worst_ring_depth: u32,
    worst_staged: usize,
    last_heartbeat: Option<Instant>,
}

impl McuMarginWindow {
    fn new(now: Instant) -> Self {
        Self {
            window_start: now,
            sends: 0,
            min_margin_secs: f64::INFINITY,
            worst_axis: 0,
            worst_occupancy: 0,
            worst_ring_depth: 0,
            worst_staged: 0,
            last_heartbeat: None,
        }
    }

    fn reset_window(&mut self, now: Instant) {
        self.window_start = now;
        self.sends = 0;
        self.min_margin_secs = f64::INFINITY;
        self.worst_axis = 0;
        self.worst_occupancy = 0;
        self.worst_ring_depth = 0;
        self.worst_staged = 0;
    }
}

/// Tracks how close each MCU's sends run to its playhead. The effective send
/// margin for dense piece streams is bounded by the MCU ring's time depth
/// (ring_depth x piece_duration), not `lead_secs`; this makes that margin
/// observable so a -308 has a lead-up trail instead of appearing out of thin
/// air.
pub(super) struct SendMarginTracker {
    per_mcu: BTreeMap<u32, McuMarginWindow>,
}

impl SendMarginTracker {
    pub(super) fn new() -> Self {
        Self {
            per_mcu: BTreeMap::new(),
        }
    }

    pub(super) fn note_heartbeat(&mut self, mcu_id: u32) {
        let now = Instant::now();
        self.per_mcu
            .entry(mcu_id)
            .or_insert_with(|| McuMarginWindow::new(now))
            .last_heartbeat = Some(now);
    }

    pub(super) fn observe_send(
        &mut self,
        mcu_id: u32,
        bundle: &[AxisFrame],
        freq: f64,
        queues: &BTreeMap<AxisKey, AxisQueue>,
    ) {
        if freq <= 0.0 {
            return;
        }
        let now = Instant::now();
        let w = self
            .per_mcu
            .entry(mcu_id)
            .or_insert_with(|| McuMarginWindow::new(now));
        w.sends += 1;
        for frame in bundle {
            let Some(front) = frame.pieces.first() else {
                continue;
            };
            let margin = front_margin_secs(front.start_time, frame.guard_mcu_clock, freq);
            if margin < w.min_margin_secs {
                w.min_margin_secs = margin;
                w.worst_axis = frame.axis;
                let key = AxisKey {
                    mcu_id,
                    axis: frame.axis,
                };
                if let Some(q) = queues.get(&key) {
                    w.worst_occupancy = q.pushed.wrapping_sub(q.consumed);
                    w.worst_ring_depth = q.ring_depth;
                    w.worst_staged = q.pieces.len();
                }
            }
        }
        if now.duration_since(w.window_start) >= EMIT_INTERVAL && w.min_margin_secs.is_finite() {
            let min_margin_ms = w.min_margin_secs * 1e3;
            let heartbeat_age_ms = w
                .last_heartbeat
                .map(|t| now.duration_since(t).as_secs_f64() * 1e3);
            if w.min_margin_secs < crate::anchor::LOW_MARGIN_WARN_SECS {
                tracing::warn!(
                    subsystem = "motion",
                    event = "pump_send_margin",
                    mcu = mcu_id,
                    min_margin_ms,
                    sends = w.sends,
                    worst_axis = w.worst_axis,
                    occupancy = w.worst_occupancy,
                    ring_depth = w.worst_ring_depth,
                    staged = w.worst_staged,
                    heartbeat_age_ms,
                    "[pump-margin] send margin below the anchor warn floor — a transport \
                     stall this long becomes a -308"
                );
            } else {
                tracing::info!(
                    subsystem = "motion",
                    event = "pump_send_margin",
                    mcu = mcu_id,
                    min_margin_ms,
                    sends = w.sends,
                    worst_axis = w.worst_axis,
                    occupancy = w.worst_occupancy,
                    ring_depth = w.worst_ring_depth,
                    staged = w.worst_staged,
                    heartbeat_age_ms,
                    "send margin over the last sample period"
                );
            }
            w.reset_window(now);
        }
    }
}
