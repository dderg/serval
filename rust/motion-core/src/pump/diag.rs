use runtime::piece_ring::PieceEntry;

use super::AxisKey;
use super::junction::{JunctionSeam, junction_jumps};
use super::sched::AxisFrame;

pub(super) fn log_piece_submit(
    mcu_id: u32,
    axis: u8,
    freq: Option<f32>,
    piece: &PieceEntry,
    prev_end: Option<u64>,
) -> Option<u64> {
    let end_ticks: u64 = freq.map_or(0, |f| piece.end_time(f));
    let gap_ticks_in_frame: i64 = prev_end.map_or(0, |pe| piece.start_time as i64 - pe as i64);
    tracing::trace!(
        subsystem = "motion",
        event = "pump_piece_submit",
        mcu = mcu_id,
        axis = axis,
        start_time = piece.start_time,
        duration_s = piece.duration,
        end_ticks,
        gap_ticks_in_frame,
        motor_mask = piece.motor_mask,
        "[pump-submit] piece submitted to MCU \
         (gap_ticks_in_frame: 0=contiguous, <0=overlap, >0=gap)"
    );
    if freq.is_some() {
        Some(end_ticks)
    } else {
        prev_end
    }
}

/// Drip cohorts release by piece count against a real-time playhead, so the
/// lead each burst reaches the pump with is the signal for whether the floor
/// is keeping ahead. Emitted per enqueue while a cohort is armed.
pub(super) fn log_drip_enqueue_lead(
    key: AxisKey,
    pieces: &[(PieceEntry, f64)],
    ack_now: u64,
    freq: f64,
) {
    let first_start = pieces[0].0.start_time;
    let produce_lead_us = (first_start as i64 - ack_now as i64) as f64 / freq * 1e6;
    let durs = pieces.iter().map(|p| p.0.duration);
    let min_dur = durs.clone().fold(f32::INFINITY, f32::min);
    let max_dur = durs.clone().fold(0.0_f32, f32::max);
    let total: f32 = durs.sum();
    tracing::warn!(
        subsystem = "motion",
        event = "drip_enqueue_lead",
        mcu = key.mcu_id,
        axis = key.axis,
        n = pieces.len(),
        produce_lead_us,
        min_dur_us = min_dur * 1e6,
        max_dur_us = max_dur * 1e6,
        total_secs = total,
        "[drip-diag] pieces reached pump with this much lead before their start"
    );
}

/// Classifies the tick/host jump across a stream seam and logs it. A jump that
/// diverges past 50us between the MCU-tick and host projections (or runs the
/// tick clock backwards) is anomalous on a continuing stream — but expected on
/// a fresh stream, which restarts both clocks.
pub(super) fn log_junction_jump(
    seam: &JunctionSeam,
    source_line: u32,
    fresh_stream: bool,
    freq: f64,
) {
    let (tick_jump_us, host_jump_us) = junction_jumps(
        seam.first_start_ticks,
        seam.next_start_host,
        seam.prev_end_ticks,
        seam.prev_end_host,
        freq,
    );
    let anomalous = tick_jump_us < -50.0 || (tick_jump_us - host_jump_us).abs() > 50.0;
    if fresh_stream || !anomalous {
        tracing::debug!(
            subsystem = "motion",
            event = "junction_jump",
            key = ?seam.key,
            tick_jump_us,
            host_jump_us,
            fresh = fresh_stream,
            "[junction] jump"
        );
    } else {
        let reason = if tick_jump_us < -50.0 {
            "overlap_risk"
        } else {
            "projection_divergence"
        };
        tracing::warn!(
            subsystem = "motion",
            event = "junction_jump_anomalous",
            key = ?seam.key,
            tick_jump_us,
            host_jump_us,
            fresh = fresh_stream,
            reason,
            prev_source_line = seam.prev_source_line,
            next_source_line = source_line,
            "[junction] anomalous jump"
        );
    }
}

/// The MCU-clock projection of the front piece at send time, logged while a
/// drip cohort is armed so the release lead can be traced against the
/// playhead.
pub(super) fn log_send_projection(mcu_id: u32, mcu_now: u64, freq: f64, bundle: &[AxisFrame]) {
    if let Some(front) = bundle.first().and_then(|af| af.pieces.first()) {
        tracing::warn!(
            subsystem = "motion",
            event = "pump_send_projection",
            mcu = mcu_id,
            projected_now = mcu_now,
            front_start = front.start_time,
            release_lead_us = ((front.start_time as i64 - mcu_now as i64) as f64 / freq * 1e6),
            "[drip-diag] projection at send"
        );
    }
}
