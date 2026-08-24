use trajectory::ClockedMotorSpan;

use super::junction::{JunctionSeam, junction_jumps};

pub(super) fn log_span_submit(
    mcu_id: u32,
    axis: u8,
    span: &ClockedMotorSpan,
    prev_end: Option<u64>,
) -> Option<u64> {
    let gap_ticks_in_frame: i64 = prev_end.map_or(0, |pe| span.start_clock as i64 - pe as i64);
    tracing::trace!(
        subsystem = "motion",
        event = "pump_span_submit",
        mcu = mcu_id,
        axis = axis,
        start_clock = span.start_clock,
        duration_s = span.stream_t_end - span.stream_t_start,
        end_clock = span.end_clock,
        gap_ticks_in_frame,
        motor_mask = span.signal.motor_mask,
        "[pump-submit] span submitted to MCU \
         (gap_ticks_in_frame: 0=contiguous, <0=overlap, >0=gap)"
    );
    Some(span.end_clock)
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
