// TIM5 ISR body — clock widening, inter-arrival gap guard, `Engine::tick`, and
// the endstop trip detector + freeze consumer.
//
// Call order each tick:
//   1. Widen raw CYCCNT → `now` (u64).
//   2. Publish widened clock unconditionally (freeze path must not skip this —
//      skipping pegs the foreground scheduler; see warning at line ~88).
//   3. Call `endstop::tick()` with the PREVIOUS tick's per-axis Q16 velocities
//      and the live per-stepper step counts from `shared.stepper_counts`.
//   4a. AbortNow → freeze: skip `engine.tick`, clear `last_tick_now` (so the
//       inter-arrival guard does not fire on unfreeze), return.
//   4b. Continue → run the inter-arrival guard and `engine.tick` as before.
//
// Freeze contract:
//   AbortNow latches via `endstop::ARM.state` staying `Tripping`/`TrippedReady`
//   across ticks. No separate flag needed: `endstop::tick()` returns AbortNow
//   on every ISR call until `endstop::arm()` transitions state back to Armed.
//   `last_tick_now = None` on frozen ticks guards the gap check so unfreezing
//   does not raise TickIntervalExceeded.
//
// Recovery path:
//   host: disarm_endstop → runtime_reset (engine.reset()) → seed_position →
//         arm_endstop (→ endstop::arm() resets state → Armed). After arm(),
//         tick() returns Continue and engine.tick runs normally. The ring is
//         cleared by engine.reset(), so abandoned pieces never replay.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::endstop::TripAction;
use crate::fault_helpers::raise_tick_interval_exceeded;
use crate::state::SharedState;

#[cfg(feature = "motion-module-stepper")]
pub use crate::dispatch_stepper::{AXIS_A, AXIS_B, AXIS_E, AXIS_Z, DISPLACEMENT_THRESHOLD_MM};

pub use crate::stepping_state::N_AXES;

// C-side scheduler accessor for the most-recently-dispatched timer func.
// Read only on the `-311` fault path. MCU/sim link only; host/test → 0.
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn sched_last_dispatched_func() -> u32;
}

// Stacked exception-frame captures from the TIM5 naked-wrapper shim
// (src/stm32/runtime_tick_*.c). Read only on the `-311` fault path:
//   - `runtime_tim5_stacked_pc()`: instruction about to execute when TIM5 preempted.
//   - `runtime_tim5_stacked_exc()`: stacked xPSR exception number (0 = thread).
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn runtime_tim5_stacked_pc() -> u32;
    fn runtime_tim5_stacked_exc() -> u32;
}

#[inline]
fn tim5_stacked_pc() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free volatile frame read. Safe from the TIM5 ISR.
    unsafe {
        runtime_tim5_stacked_pc()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

#[inline]
fn tim5_stacked_exc() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free volatile frame read. Safe from the TIM5 ISR.
    unsafe {
        runtime_tim5_stacked_exc()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

#[inline]
fn last_dispatched_func() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free ring-buffer index read. Safe from the TIM5 ISR.
    unsafe {
        sched_last_dispatched_func()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

const TICK_GAP_FAULT_MULT: u64 = 2;

pub fn isr_sample_tick(
    isr: &mut crate::state::IsrState,
    shared: &SharedState,
    storage: &mut [crate::piece_ring::PieceEntry],
    raw_cyccnt: u32,
) {
    let body_start = unsafe { cyccnt_read() };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_ENTER);

    bump_relaxed(isr.engine.tick_counter.inner_atomic());

    let now = isr.widen_state.widen(raw_cyccnt);

    // Publish unconditionally — skipping on a fault tick freezes the
    // foreground clock and pegs the scheduler.
    crate::clock::publish_widened_now(shared, now);

    let after_widen = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_widen_cycles_max,
        after_widen.wrapping_sub(body_start),
    );

    let after_arm = after_widen;
    update_max(
        &shared.isr_arm_cycles_max,
        after_arm.wrapping_sub(after_widen),
    );

    // A disarm of a frozen arm abandons the rest of the homing move; the
    // purge retires those pieces (returning the host's ring credits) before
    // the engine could ever evaluate them.
    if crate::endstop::take_ring_purge_request() {
        isr.engine.purge_rings();
    }

    // Endstop trip detector — runs BEFORE engine.tick so AbortNow freezes this
    // tick's step dispatch rather than the next one.
    //
    // v_per_axis_q16: previous tick's per-axis velocity magnitudes in Q16.16
    // (mm/s * 65536). One-tick stale is acceptable for IgnoreUntilMoving.
    //
    // stepper_counts: shared.stepper_counts indexed by stepper OID — the same
    // array the host reads via kalico_runtime_get_stepper_count for position
    // rewind after a trip. We snapshot it into a stack array to form a &[i32]
    // without allocating. MAX_STEPPER_OIDS = 16, so this is always bounded.
    let v_q16 = isr.engine.v_per_axis_q16();
    let counts = collect_stepper_counts(shared);
    let trip_action = crate::endstop::tick(now, v_q16, &counts);

    if trip_action == TripAction::AbortNow {
        // Freeze: skip step dispatch, clear last_tick_now so the gap guard
        // does not trip on unfreeze (the idle-gap rule: None straddles gaps).
        isr.last_tick_now = None;
        crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
        return;
    }

    // Inter-arrival guard: only fires when the previous tick was active
    // (last_tick_now == Some). Idle/boot ticks leave it None so the guard
    // never trips during config or between moves.
    let period = isr.engine.sample_period_cycles as u64;
    if let Some(last) = isr.last_tick_now {
        let gap = now.wrapping_sub(last);
        if period != 0 && gap > period * TICK_GAP_FAULT_MULT {
            // Integer division is intentional: `gap_ticks` is the integer
            // count of sample periods elapsed, used as a fault detail tag.
            #[allow(clippy::integer_division)]
            let gap_ticks = (gap / period) as u32;
            // Store before the fault code latches so the host always sees
            // populated values. Stacked PC is the primary addr2line target.
            shared
                .tick_blocker_pc
                .store(tim5_stacked_pc(), Ordering::Release);
            shared
                .tick_blocker_exc
                .store(tim5_stacked_exc(), Ordering::Release);
            shared
                .tick_blocker_func
                .store(last_dispatched_func(), Ordering::Release);
            raise_tick_interval_exceeded(shared, gap_ticks);
            isr.last_tick_now = Some(now);
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
            return;
        }
    }

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_TICK);
    let active = {
        let crate::state::IsrState { engine, .. } = isr;
        engine.tick(now, shared, storage)
    };

    let body_end = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_eval_cycles_max,
        body_end.wrapping_sub(after_arm),
    );

    // Some only when this tick was active; idle ticks clear it so the gap
    // check never straddles an idle gap.
    isr.last_tick_now = if active { Some(now) } else { None };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
}

/// Read the DWT cycle counter via `isr_phase::cyccnt()` — sole declaration
/// there avoids duplicate `extern "C"` symbols at link time.
#[inline]
unsafe fn cyccnt_read() -> u32 {
    crate::isr_phase::cyccnt()
}

/// Snapshot `shared.stepper_counts` into a stack array for the endstop tick.
///
/// Returns an array of length `MAX_STEPPER_OIDS` where index == stepper OID.
/// The endstop's `publish_snapshot` indexes into this slice by OID, matching
/// the arm's `stepper_oids` list: `stepper_counts.get(oid as usize)`.
///
/// Uses `Relaxed` loads: the counts are updated by the same ISR (dispatch path),
/// so this read is coherent within the ISR execution. Cross-ISR ordering is not
/// needed here — the host reads counts via `kalico_runtime_get_stepper_count`
/// under a foreground lock, not from this snapshot.
#[inline]
fn collect_stepper_counts(shared: &SharedState) -> [i32; crate::state::MAX_STEPPER_OIDS] {
    let mut out = [0i32; crate::state::MAX_STEPPER_OIDS];
    for (slot, atomic) in out.iter_mut().zip(shared.stepper_counts.iter()) {
        *slot = atomic.load(Ordering::Relaxed);
    }
    out
}

#[inline]
pub(crate) fn update_max(slot: &portable_atomic::AtomicU32, val: u32) {
    let prev = slot.load(Ordering::Relaxed);
    if val > prev {
        slot.store(val, Ordering::Relaxed);
    }
}

#[inline]
pub(crate) fn bump_relaxed(slot: &portable_atomic::AtomicU32) {
    let prev = slot.load(Ordering::Relaxed);
    slot.store(prev.wrapping_add(1), Ordering::Relaxed);
}

#[cfg(test)]
mod tests;
