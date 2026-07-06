use crate::chebyshev::{clenshaw, derivative_series};
use crate::fault_sink::FaultSink;
use crate::piece_ring::{MAX_PIECE_COEFFS, PieceEntry, RingDescriptor};

/// A piece armed for the ISR: the Chebyshev position series plus both
/// derivative series, computed once at arm. Per tick the evaluation is
/// `u = elapsed_cycles·inv_scale − 1` and one Clenshaw pass per series.
#[derive(Debug, Clone, Copy)]
pub struct ArmedPiece {
    pub cheb: [f32; MAX_PIECE_COEFFS],
    pub vel: [f32; MAX_PIECE_COEFFS],
    pub acc: [f32; MAX_PIECE_COEFFS],
    pub n: u8,
    /// `2 / (duration · cycles_per_second)` — cycles → u without a divide.
    pub inv_scale: f32,
    pub piece_start_cycles: u64,
    pub piece_end_cycles: u64,
}

impl ArmedPiece {
    #[inline]
    fn u_at(&self, now: u64) -> f32 {
        #[allow(clippy::cast_precision_loss)]
        let elapsed = now.saturating_sub(self.piece_start_cycles) as f32;
        let u = elapsed * self.inv_scale - 1.0;
        debug_assert!(u <= 1.0 + 1e-3, "u = {u} escaped the piece domain");
        u
    }

    #[inline]
    fn series(a: &[f32; MAX_PIECE_COEFFS], n: usize) -> &[f32] {
        a.get(..n.clamp(1, MAX_PIECE_COEFFS)).unwrap_or(a)
    }

    #[inline]
    pub fn eval_pos_vel(&self, now: u64) -> (f32, f32) {
        let u = self.u_at(now);
        let n = self.n as usize;
        let pos = clenshaw(Self::series(&self.cheb, n), u);
        let vel = clenshaw(Self::series(&self.vel, n.saturating_sub(1)), u);
        (pos, vel)
    }

    #[inline]
    pub fn eval_accel(&self, now: u64) -> f32 {
        let u = self.u_at(now);
        let n = self.n as usize;
        clenshaw(Self::series(&self.acc, n.saturating_sub(2)), u)
    }
}

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
    let mut just_armed = false;
    get_position_and_velocity_armed(
        armed,
        ring,
        storage,
        now,
        sample_period_cycles,
        cycles_per_second,
        axis_idx,
        fault,
        &mut just_armed,
    )
}

#[inline]
#[allow(clippy::too_many_arguments)]
pub fn get_position_and_velocity_armed<F: FaultSink>(
    armed: &mut Option<ArmedPiece>,
    ring: &mut RingDescriptor,
    storage: &[PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    axis_idx: usize,
    fault: &F,
    just_armed: &mut bool,
) -> Option<(f32, f32)> {
    *just_armed = false;
    if let Some(p) = &*armed {
        if now < p.piece_end_cycles {
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_CLENSHAW);
            return Some(p.eval_pos_vel(now));
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

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ARM);
    let arm_start = crate::isr_phase::cyccnt();
    // SAFETY: `slot` is `ring_offset + tail` from `get_piece_for_time` →
    // `ring.front_slot()`. `configure_axis` guarantees
    // `ring_offset + ring_depth <= storage.len()`, and `tail < ring_depth`
    // always holds (tail advances mod ring_depth). Therefore `slot <
    // storage.len()`.
    #[allow(clippy::indexing_slicing)]
    let p = arm_and_load(armed, &storage[slot], cycles_per_second);
    *just_armed = true;
    crate::isr_phase::arm_account(crate::isr_phase::cyccnt().wrapping_sub(arm_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_CLENSHAW);
    Some(p.eval_pos_vel(now))
}

// CRITICAL: the fault-check runs BEFORE the `now < end` window return so
// that a cold-adopted front is always checked; inverting this order silently
// drops the deficit fault for the first piece after a ring refill.
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
    // mcu-sim: the virtual clock races far ahead of klippy's clock
    // estimate, so the grace window must absorb sim jitter.
    #[cfg(not(feature = "mcu-sim"))]
    const MAX_START_IN_PAST_SECS: f32 = 200e-6;
    #[cfg(feature = "mcu-sim")]
    const MAX_START_IN_PAST_SECS: f32 = 10.0;
    let drift_budget = (MAX_START_IN_PAST_SECS * cycles_per_second) as u64;
    let fault_tolerance = drift_budget + u64::from(sample_period_cycles);
    loop {
        let slot = ring.front_slot()?;
        // SAFETY: `slot` is `ring_offset + tail` from `front_slot()`.
        // `configure_axis` guarantees `ring_offset + ring_depth <= storage.len()`,
        // and `tail < ring_depth` always holds. Therefore `slot < storage.len()`.
        #[allow(clippy::indexing_slicing)]
        let entry = &storage[slot];
        let deficit_cycles = now.saturating_sub(entry.start_time);
        if deficit_cycles > fault_tolerance {
            let deficit_us = (deficit_cycles as f32 * (1.0e6_f32 / cycles_per_second)) as u32;
            fault.piece_start_in_past(axis_idx, deficit_us);
            return None;
        }
        if now < entry.end_time(cycles_per_second) {
            return Some(slot);
        }
        ring.advance_counter();
    }
}

#[inline]
pub fn arm_piece(entry: &PieceEntry, cycles_per_second: f32) -> ArmedPiece {
    debug_assert!(
        entry.duration > 0.0,
        "piece with non-positive duration reached arm — write acceptance must reject it"
    );
    debug_assert!(
        entry.coeff_count >= 1 && (entry.coeff_count as usize) <= MAX_PIECE_COEFFS,
        "coeff_count {} reached arm — write acceptance must reject it",
        entry.coeff_count
    );
    let n = (entry.coeff_count as usize).clamp(1, MAX_PIECE_COEFFS);
    let du_dt = 2.0 / entry.duration;
    let mut vel = [0.0_f32; MAX_PIECE_COEFFS];
    let nv = derivative_series(
        entry.coeffs.get(..n).unwrap_or(&entry.coeffs),
        du_dt,
        &mut vel,
    );
    let mut acc = [0.0_f32; MAX_PIECE_COEFFS];
    derivative_series(vel.get(..nv).unwrap_or(&vel), du_dt, &mut acc);
    ArmedPiece {
        cheb: entry.coeffs,
        vel,
        acc,
        n: n as u8,
        inv_scale: 2.0 / (entry.duration * cycles_per_second),
        piece_start_cycles: entry.start_time,
        piece_end_cycles: entry.end_time(cycles_per_second),
    }
}

#[inline]
fn arm_and_load<'a>(
    armed: &'a mut Option<ArmedPiece>,
    entry: &PieceEntry,
    cycles_per_second: f32,
) -> &'a ArmedPiece {
    armed.insert(arm_piece(entry, cycles_per_second))
}
