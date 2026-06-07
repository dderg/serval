use crate::fault_sink::FaultSink;
use crate::piece_ring::{PieceEntry, RingDescriptor};

#[derive(Debug, Clone, Copy)]
pub struct ArmedPiece {
    pub mono_coeffs: [f32; 4],
    pub vel_coeffs: [f32; 3],
    pub piece_start_cycles: u64,
    pub piece_end_cycles: u64,
}

/// Advance the axis to the correct piece for `now`, returning
/// `(position, velocity)` if an active piece exists.
///
/// Walk/load split: only the landed piece is monomialised (to_monomial is
/// the dominant cost; calling it for every expired piece would be O(walk-depth)).
#[inline]
pub fn get_position_and_velocity<F: FaultSink>(
    armed: &mut Option<ArmedPiece>,
    ring: &mut RingDescriptor,
    storage: &[PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    axis_idx: usize,
    fault: &F,
) -> Option<(f32, f32)> {
    if let Some(p) = &*armed {
        if now < p.piece_end_cycles {
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_HORNER);
            return Some(eval_horner(
                &p.mono_coeffs,
                &p.vel_coeffs,
                p.piece_start_cycles,
                now,
                cycles_per_second,
            ));
        }
        #[cfg(feature = "host")]
        {
            let end = p.piece_end_cycles;
            let overshoot = now.saturating_sub(end);
            if overshoot > 30_000 {
                let start = p.piece_start_cycles;
                eprintln!(
                    "[piece-trace] axis={axis_idx} now={now} armed_expired_late: \
                     start={start} end={end} overshoot={overshoot}"
                );
            }
        }
        *armed = None;
        ring.advance_counter();
    }

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_WALK);
    let walk_start = crate::isr_phase::cyccnt();
    let slot = get_piece_for_time(
        ring,
        storage,
        now,
        sample_period_cycles,
        cycles_per_second,
        axis_idx,
        fault,
    )?;
    crate::isr_phase::walk_account(crate::isr_phase::cyccnt().wrapping_sub(walk_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_MONOMIAL);
    let mono_start = crate::isr_phase::cyccnt();
    // SAFETY: `slot` is `ring_offset + tail` from `get_piece_for_time` →
    // `ring.front_slot()`. `configure_axis` guarantees
    // `ring_offset + ring_depth <= storage.len()`, and `tail < ring_depth`
    // always holds (tail advances mod ring_depth). Therefore `slot <
    // storage.len()`.
    #[allow(clippy::indexing_slicing)]
    let p = arm_and_load(armed, &storage[slot], cycles_per_second);
    crate::isr_phase::monomial_account(crate::isr_phase::cyccnt().wrapping_sub(mono_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_HORNER);
    Some(eval_horner(
        &p.mono_coeffs,
        &p.vel_coeffs,
        p.piece_start_cycles,
        now,
        cycles_per_second,
    ))
}

/// Walk the ring to the entry whose window contains `now`, retiring elapsed
/// entries WITHOUT monomialising them. Returns the physical slot index of the
/// landed piece, or `None` when the ring is empty.
///
/// **Hardware behavior** (`not(feature = "host")`):
/// Hard-faults `PieceStartInPast` when an expired piece's start is more than
/// `drift_budget + sample_period_cycles` in the past — genuine engine starvation
/// (MCU suspended long enough to miss a piece). The fault fires and returns `None`.
///
/// **MACH_LINUX host behavior** (`feature = "host"`):
/// Expired pieces are ALWAYS silently retired — never faulted. The Linux
/// kernel CFS scheduler can delay the tick thread by up to ~2ms, which is
/// enough to expire one or two consecutive 0.8–1.6ms homing pieces in a
/// single missed-tick event. Silent retire lets the engine skip to the next
/// active piece and continue motion. `PieceStartInPast` is never signalled
/// on the host; ring-empty (`None`) is the only starvation signal.
///
/// **Both**: a still-active piece (`now < end_time`) is always adopted,
/// regardless of how late its start was observed.
#[inline]
fn get_piece_for_time<F: FaultSink>(
    ring: &mut RingDescriptor,
    storage: &[PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    axis_idx: usize,
    fault: &F,
) -> Option<usize> {
    const MAX_START_IN_PAST_SECS: f32 = 200e-6;
    let drift_budget = (MAX_START_IN_PAST_SECS * cycles_per_second) as u64;
    let fault_tolerance = drift_budget + u64::from(sample_period_cycles);
    loop {
        let slot = ring.front_slot()?;
        // SAFETY: `slot` is `ring_offset + tail` from `front_slot()`.
        // `configure_axis` guarantees `ring_offset + ring_depth <= storage.len()`,
        // and `tail < ring_depth` always holds. Therefore `slot < storage.len()`.
        #[allow(clippy::indexing_slicing)]
        let entry = &storage[slot];
        let end_time = entry.end_time(cycles_per_second);
        if now < end_time {
            // Piece is still active — adopt it unconditionally. Even if its
            // start is late (tick jitter on MACH_LINUX), eval_horner at `now`
            // gives the correct position within the active window.
            return Some(slot);
        }
        // Piece has expired (now >= end_time).
        #[cfg(not(feature = "host"))]
        {
            // Hardware: fault if the piece was missed by more than the tolerance.
            // On real silicon the tick is driven by TIM5 at SCHED_FIFO-equivalent
            // priority; missing a piece by > fault_tolerance indicates genuine
            // starvation (IWDG stall, bus lock, etc.) and position is lost.
            let deficit_cycles = now.saturating_sub(entry.start_time);
            if deficit_cycles > fault_tolerance {
                let deficit_us =
                    (deficit_cycles as f32 * (1.0e6_f32 / cycles_per_second)) as u32;
                fault.piece_start_in_past(axis_idx, deficit_us);
                return None;
            }
        }
        #[cfg(feature = "host")]
        {
            // MACH_LINUX: silently retire expired pieces regardless of deficit.
            // The Linux CFS scheduler can delay the tick thread by up to ~2ms,
            // expiring one or two consecutive short pieces in a single sleep
            // overrun. Faulting here would abort motion on a recoverable
            // transient; just skip and find the next active piece.
            let _ = (fault, fault_tolerance); // not used on host
            let deficit_cycles = now.saturating_sub(entry.start_time);
            let deficit_us = (deficit_cycles as f32 * (1.0e6_f32 / cycles_per_second)) as u32;
            eprintln!(
                "[piece-trace] axis={axis_idx} now={now} SKIP-EXPIRED: \
                 start={} end={end_time} deficit_us={deficit_us} \
                 ring_head={} ring_retired={} ring_tail={}",
                entry.start_time,
                ring.head,
                ring.retired,
                ring.tail,
            );
        }
        ring.advance_counter();
    }
}

#[inline]
fn arm_and_load<'a>(
    armed: &'a mut Option<ArmedPiece>,
    entry: &PieceEntry,
    cycles_per_second: f32,
) -> &'a ArmedPiece {
    let (mono, vel) = entry.to_monomial();
    armed.insert(ArmedPiece {
        mono_coeffs: mono,
        vel_coeffs: vel,
        piece_start_cycles: entry.start_time,
        piece_end_cycles: entry.end_time(cycles_per_second),
    })
}

#[inline]
pub fn eval_horner(
    mono: &[f32; 4],
    vel: &[f32; 3],
    piece_start_cycles: u64,
    now: u64,
    cycles_per_second: f32,
) -> (f32, f32) {
    let elapsed_cycles = now.saturating_sub(piece_start_cycles);
    let t = if cycles_per_second > 0.0 {
        elapsed_cycles as f32 / cycles_per_second
    } else {
        0.0_f32
    };
    let p = mono[0] + t * (mono[1] + t * (mono[2] + t * mono[3]));
    let v = vel[0] + t * (vel[1] + t * vel[2]);
    (p, v)
}
