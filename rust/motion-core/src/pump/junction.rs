use super::AxisKey;
use runtime::piece_ring::PieceEntry;
use std::collections::BTreeMap;

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
    end_pos: f32,
    source_line: u32,
}

pub const JUNCTION_POSITION_LOG_MM: f32 = 0.0125;
pub const JUNCTION_POSITION_FATAL_MM: f32 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct JunctionSeam {
    pub key: AxisKey,
    pub prev_end_pos: f32,
    pub next_start_pos: f32,
    pub prev_end_host: f64,
    pub next_start_host: f64,
    pub prev_source_line: u32,
    pub next_source_line: u32,
    pub prev_end_ticks: u64,
    pub first_start_ticks: u64,
}

impl JunctionSeam {
    #[must_use]
    pub fn jump(&self) -> f32 {
        (self.next_start_pos - self.prev_end_pos).abs()
    }

    #[must_use]
    pub fn is_fatal(&self) -> bool {
        self.jump() >= JUNCTION_POSITION_FATAL_MM
    }
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
        pieces: &[(PieceEntry, f64)],
        source_line: u32,
        freq: f64,
    ) -> Option<JunctionSeam> {
        let (first_entry, first_host) = pieces.first()?;
        if first_entry.motor_mask != 0 {
            return None;
        }
        let seam = self.ends.get(&key).map(|prev| JunctionSeam {
            key,
            prev_end_pos: prev.end_pos,
            next_start_pos: first_entry.pos_start(),
            prev_end_host: prev.end_host,
            next_start_host: *first_host,
            prev_source_line: prev.source_line,
            next_source_line: source_line,
            prev_end_ticks: prev.end_ticks,
            first_start_ticks: first_entry.start_time,
        });
        let (last_entry, last_host) = pieces.last().unwrap();
        #[allow(clippy::cast_possible_truncation)]
        let last_end_ticks = last_entry.end_time(freq as f32);
        let last_end_host = last_host + last_entry.duration as f64;
        self.ends.insert(
            key,
            JunctionEnd {
                end_ticks: last_end_ticks,
                end_host: last_end_host,
                end_pos: last_entry.pos_end(),
                source_line,
            },
        );
        seam
    }

    pub fn observe_msg(
        &mut self,
        key: AxisKey,
        pieces: &[(PieceEntry, f64)],
        epoch: crate::anchor::StreamEpoch,
        source_line: u32,
        freq: Option<f64>,
    ) -> Option<JunctionSeam> {
        if epoch.position_redefined() {
            self.forget(key);
        }
        self.observe(key, pieces, source_line, freq?)
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
            "junction position discontinuity on mcu{} axis{}: prev piece ends at \
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
