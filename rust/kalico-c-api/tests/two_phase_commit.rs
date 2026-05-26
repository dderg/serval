//! Two-phase segment commit protocol tests.
//!
//! Exercises the push -> commit / abort flow defined by
//! `runtime_handle_push_segment`, `runtime_handle_commit_segment`, and
//! `runtime_handle_abort_pending`. All cases run sequentially in a single
//! `#[test]` to avoid inter-test ordering issues with the global `INIT_DONE`
//! singleton.

#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Host-side stubs for `extern "C"` symbols the runtime FFI declares.
// In the MCU build these come from `src/runtime_tick.c` and the H7 timer
// driver; on host we provide them here so the linker resolves cleanly.
// ---------------------------------------------------------------------------

#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_enable() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_disable() {}

// Configurable stubs: tests can change STUB_CYCCNT / STUB_WIDENED to inject
// specific clock values without rebuilding. Defaults reproduce the original
// fixed values so all pre-existing cases are unaffected.
//
// STUB_CYCCNT_STEP: added to STUB_CYCCNT on each call, simulating advancing
// hardware. Set to 0 for the old fixed-value behavior.
static STUB_CYCCNT: AtomicU32 = AtomicU32::new(0);
static STUB_CYCCNT_STEP: AtomicU32 = AtomicU32::new(0);
static STUB_WIDENED: AtomicU64 = AtomicU64::new(1_000_000_000);

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    let v = STUB_CYCCNT.load(Ordering::Relaxed);
    let step = STUB_CYCCNT_STEP.load(Ordering::Relaxed);
    if step > 0 {
        STUB_CYCCNT.store(v.wrapping_add(step), Ordering::Relaxed);
    }
    v
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_reset_stepper_bindings() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_irq_save() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_irq_restore(_flags: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_host_now_us() -> u64 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_widened_host_clock() -> u64 {
    STUB_WIDENED.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Dummy push parameters.
//
// All curve handles set to UNUSED_SENTINEL (0xFFFF_FFFF). Timing uses
// values large enough to pass the min_segment_cycles check
// (520 MHz / 40 kHz * 2 = 26 000 cycles minimum).
// ---------------------------------------------------------------------------
const UNUSED: u32 = 0xFFFF_FFFF;
const CARTESIAN_XYZ_AND_E: u8 = 1;
const E_MODE_TRAVEL: u8 = 2;

/// Duration in MCU clock cycles that comfortably exceeds
/// `min_segment_cycles` (26 000 at 520 MHz / 40 kHz).
const SEG_DURATION: u64 = 500_000;

/// Helper: push a segment with the given `id` and valid dummy parameters.
/// `t_start` and `t_end` are set to provide a valid duration.
///
/// Returns the raw FFI return code.
unsafe fn push(handle: *mut kalico_c_api::KalicoRuntime, id: u32) -> i32 {
    let mut accepted_id: u32 = 0;
    let mut credit_epoch: u32 = 0;
    // SAFETY: `handle` is the published runtime pointer from
    // `runtime_handle_create`; all dummy parameters are valid per the
    // push_segment_impl validation (kinematics=1, e_mode=2, duration >
    // min_segment_cycles). Out-params point to local stack variables.
    unsafe {
        kalico_c_api::runtime_handle_push_segment(
            handle,
            id,
            UNUSED,
            UNUSED,
            UNUSED,
            UNUSED,
            0,             // t_start — placeholder; real timing comes from commit
            SEG_DURATION,  // t_end — must satisfy t_end > t_start + min_segment_cycles
            CARTESIAN_XYZ_AND_E,
            E_MODE_TRAVEL,
            0, // extrusion_ratio_bits
            &mut accepted_id as *mut u32,
            &mut credit_epoch as *mut u32,
        )
    }
}

#[test]
fn two_phase_commit_protocol() {
    // -----------------------------------------------------------------------
    // 1. Create runtime handle — must return non-null.
    // -----------------------------------------------------------------------
    let rt = kalico_c_api::runtime_handle_create();
    assert!(!rt.is_null(), "runtime_handle_create must return non-null");

    // -----------------------------------------------------------------------
    // 2. Push stores in staged slot (not SPSC). Push id=1 succeeds.
    // -----------------------------------------------------------------------
    let rc = unsafe { push(rt, 1) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "push(id=1) should succeed — staged slot is empty"
    );

    // -----------------------------------------------------------------------
    // 3. Push while staged slot occupied -> ERR_PENDING_SLOT_OCCUPIED.
    // -----------------------------------------------------------------------
    let rc = unsafe { push(rt, 2) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_PENDING_SLOT_OCCUPIED,
        "push(id=2) should fail — staged slot holds id=1"
    );

    // -----------------------------------------------------------------------
    // 4. Commit with wrong ID -> ERR_SEGMENT_ID_MISMATCH.
    // -----------------------------------------------------------------------
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            99,         // wrong segment_id
            1_000_000,  // t_start_clock
            SEG_DURATION,
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_SEGMENT_ID_MISMATCH,
        "commit(id=99) should fail — staged segment has id=1"
    );

    // -----------------------------------------------------------------------
    // 5. Commit with correct ID succeeds. out_segment_id == 1.
    // -----------------------------------------------------------------------
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            1,          // correct segment_id
            1_000_000,  // t_start_clock (cold start — non-zero)
            SEG_DURATION,
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "commit(id=1) should succeed");
    assert_eq!(out_id, 1, "out_segment_id should be 1 after commit");

    // -----------------------------------------------------------------------
    // 6. Commit with empty staged slot -> ERR_NO_PENDING_SEGMENT.
    // -----------------------------------------------------------------------
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            1,
            2_000_000,
            SEG_DURATION,
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_NO_PENDING_SEGMENT,
        "commit on empty staged slot should fail"
    );

    // -----------------------------------------------------------------------
    // 7. Push after commit succeeds (staged slot is now clear). Push id=2.
    // -----------------------------------------------------------------------
    let rc = unsafe { push(rt, 2) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "push(id=2) should succeed — staged slot was cleared by commit"
    );

    // -----------------------------------------------------------------------
    // 8. Abort with correct ID succeeds. out_aborted_id == 2.
    // -----------------------------------------------------------------------
    let mut out_aborted: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_abort_pending(rt, 2, &mut out_aborted as *mut u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "abort(id=2) should succeed");
    assert_eq!(
        out_aborted, 2,
        "out_aborted_id should be 2 after aborting id=2"
    );

    // -----------------------------------------------------------------------
    // 9. Abort with wrong ID -> ERR_SEGMENT_ID_MISMATCH.
    //    Push id=3 first, then abort with wrong id=99.
    // -----------------------------------------------------------------------
    let rc = unsafe { push(rt, 3) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "push(id=3) should succeed after abort cleared the slot"
    );
    let mut out_aborted: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_abort_pending(rt, 99, &mut out_aborted as *mut u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_SEGMENT_ID_MISMATCH,
        "abort(id=99) should fail — staged segment has id=3"
    );

    // -----------------------------------------------------------------------
    // 10. Abort on empty slot -> OK with out_aborted_id == 0.
    //     First abort id=3 correctly, then abort again on empty slot.
    // -----------------------------------------------------------------------
    let mut out_aborted: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_abort_pending(rt, 3, &mut out_aborted as *mut u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "abort(id=3) should succeed"
    );
    assert_eq!(out_aborted, 3, "out_aborted_id should be 3");

    let mut out_aborted: u32 = 0xDEAD;
    let rc = unsafe {
        kalico_c_api::runtime_handle_abort_pending(rt, 0, &mut out_aborted as *mut u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "abort on empty slot should return OK (idempotent)"
    );
    assert_eq!(
        out_aborted, 0,
        "out_aborted_id should be 0 when nothing was staged"
    );

    // -----------------------------------------------------------------------
    // 11. Chaining: commit with t_start_clock == 0 chains from previous
    //     t_end. Push id=4, commit with t_start_clock=0 and
    //     duration_clocks=100_000. The segment chains from the previous
    //     commit's t_end (1_000_000 + 500_000 = 1_500_000). We can't
    //     directly inspect the segment's t_start, but the commit must
    //     succeed.
    // -----------------------------------------------------------------------
    let rc = unsafe { push(rt, 4) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "push(id=4) should succeed for chaining test"
    );
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            4,        // correct segment_id
            0,        // t_start_clock == 0 -> chain from last committed t_end
            100_000,  // duration_clocks
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "commit(id=4) with t_start_clock=0 (chaining) should succeed"
    );
    assert_eq!(out_id, 4, "out_segment_id should be 4 after chained commit");

    // ── Case 12: next push after chained commit succeeds. Push id=5. ───────
    //
    // (Placeholder sequence-advance so id=10 used in Cases 13/14 is not
    // accidentally equal to a prior commit id, keeping inter-case id tracking
    // straightforward. We abort immediately so the staged slot is free.)
    let rc = unsafe { push(rt, 5) };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "push(id=5) should succeed after chained commit"
    );
    let mut out_aborted: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_abort_pending(rt, 5, &mut out_aborted as *mut u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "abort(id=5) should succeed");

    // ── Case 13: spurious-wrap regression — `seed` vs `reinit` ─────────────
    //
    // Regression test for the cold-start widen_state initialisation bug:
    // the old code called reinit(raw, last_widened) where `raw` was sampled a
    // few cycles BEFORE `last_widened`, so `raw < last_widened as u32` was
    // ALWAYS true. reinit() interpreted that as a real 32-bit wrap and bumped
    // `high` by 2^32 ≈ 8.26 s at 520 MHz. Every subsequent segment had
    // t_start deep in the past → LATE_ARM.
    //
    // The fix: replace reinit(raw, last_widened) with seed(last_widened) which
    // force-sets BOTH halves from the known-good 64-bit value.
    //
    // Scenario: widened clock ≈ 149 s uptime (77_410_000_000 cycles at 520 MHz).
    // CYCCNT low 32 bits = 77_410_000_000 % 2^32 = 100_131_264.
    // Simulated "raw sampled first" = 100_131_164 (100 cycles earlier).
    // With reinit: raw(100_131_164) < captured_low(100_131_264) → spurious
    //   high += 2^32; ISR sees now ≈ 77_410_000_000 + 4_294_967_296.
    //   t_start (widened + 130_000_000) is ~4.16 billion cycles in the past →
    //   LATE_ARM.
    // With seed: high = 77_410_000_000 & !0xFFFF_FFFF, last_low = 100_131_264.
    //   ISR reads fresh CYCCNT ≈ 100_131_264 + small_delta → no spurious wrap,
    //   now ≈ 77_410_000_000 + small_delta < t_start → PARKED. No fault.

    const WIDENED_CASE13: u64 = 77_410_000_000;
    const CYCCNT_LOW_CASE13: u32 = (WIDENED_CASE13 % (1u64 << 32)) as u32;

    // Flush the runtime to drain all queued segments from Cases 5-11
    // without driving the ISR (which would LATE_ARM on the old-domain
    // segments due to the spurious reinit wrap). flush() synchronously
    // retires everything and resets to Idle.
    let mut flush_epoch: u32 = 0;
    let rc = unsafe {
        kalico_c_api::kalico_runtime_stream_flush(rt, &mut flush_epoch as *mut u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 13 setup: flush should succeed");

    // Switch stubs to the Case 13 clock domain.
    STUB_WIDENED.store(WIDENED_CASE13, Ordering::Relaxed);
    STUB_CYCCNT_STEP.store(0, Ordering::Relaxed);

    // Push and commit while CYCCNT doesn't matter (seed doesn't read it,
    // push doesn't use it for timing).
    let rc = unsafe {
        kalico_c_api::runtime_handle_push_segment(
            rt, 10,
            0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE,
            0, 500_000,
            1, 2, 0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 13: push id=10 should succeed");

    let t_start_case13: u64 = WIDENED_CASE13 + 130_000_000;
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            10,
            t_start_case13,
            500_000,
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 13: commit id=10 should succeed");

    // Set CYCCNT to a value PAST the seed point before driving the ISR.
    // On the real MCU, the ISR fires microseconds after seed(), so CYCCNT
    // has advanced past low32(WIDENED). +500 ≈ 1µs at 520 MHz.
    STUB_CYCCNT.store(CYCCNT_LOW_CASE13 + 500, Ordering::Relaxed);
    unsafe { kalico_c_api::kalico_runtime_tick_sample(rt) };

    let status = unsafe { kalico_c_api::runtime_handle_status(rt) };
    let last_error = unsafe { kalico_c_api::runtime_handle_last_error(rt) };
    assert_ne!(
        status, 3,
        "Case 13: engine must NOT fault after seed()-initialised cold start (got status={})",
        status,
    );
    assert_eq!(
        last_error, 0,
        "Case 13: last_error must be 0 after correct seed (got 0x{:x})",
        last_error,
    );

    // ── Case 14: stale t_start_clock triggers LATE_ARM fault ────────────────
    //
    // Formerly Case 12. Reset stubs to the defaults so the widened clock is
    // back at 1_000_000_000 and the deliberately stale t_start_clock=1 is
    // unambiguously in the past.
    // Flush to retire Case 13's parked segment and return to Idle.
    let mut flush_epoch: u32 = 0;
    let rc = unsafe {
        kalico_c_api::kalico_runtime_stream_flush(rt, &mut flush_epoch as *mut u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 14 setup: flush should succeed");

    STUB_CYCCNT.store(0, Ordering::Relaxed);
    STUB_CYCCNT_STEP.store(0, Ordering::Relaxed);
    STUB_WIDENED.store(1_000_000_000, Ordering::Relaxed);

    // Push segment id=20 (well clear of id=10 used in Case 13).
    let rc = unsafe {
        kalico_c_api::runtime_handle_push_segment(
            rt, 20,
            0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE,
            0, 500_000,
            1, 2, 0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 14: push id=20 should succeed");

    // Commit with a STALE t_start_clock — value 1, which is ~1 billion
    // cycles behind the widened clock reseeded to 1_000_000_000.
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            20,
            1,        // t_start_clock = 1 (deliberately stale)
            100_000,
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "Case 14: commit itself should succeed (enqueues to SPSC)");

    unsafe { kalico_c_api::kalico_runtime_tick_sample(rt) };

    let status = unsafe { kalico_c_api::runtime_handle_status(rt) };
    let last_error = unsafe { kalico_c_api::runtime_handle_last_error(rt) };

    assert_eq!(
        status, 3,
        "Case 14: engine should be in Fault state after LATE_ARM (got status={})",
        status,
    );
    assert_eq!(
        last_error,
        kalico_c_api::KALICO_FAULT_LATE_ARM,
        "Case 14: last_error should be KALICO_FAULT_LATE_ARM (0x0010), got 0x{:x}",
        last_error,
    );
}
