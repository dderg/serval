//! Two-phase segment commit protocol tests.
//!
//! Exercises the push -> commit / abort flow defined by
//! `runtime_handle_push_segment`, `runtime_handle_commit_segment`, and
//! `runtime_handle_abort_pending`. All eleven cases run sequentially in
//! a single `#[test]` to avoid inter-test ordering issues with the global
//! `INIT_DONE` singleton.

#![allow(unsafe_code, non_upper_case_globals)]

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

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_reset_stepper_bindings() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

/// Returns a reasonable widened clock value (~2 s at 520 MHz).
/// `commit_segment_impl` calls this during the TIM5 re-enable path when
/// the runtime status is Idle or Drained.
#[unsafe(no_mangle)]
pub extern "C" fn runtime_widened_host_clock() -> u64 {
    1_000_000_000
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

    // ── Case 12: stale t_start_clock triggers LATE_ARM fault ────────
    //
    // Reproduces the G28 X instacrash (2026-05-26): cold-start t_start_clock
    // computed at push time was stale by commit time. The ISR found t_start
    // in the past and faulted with LATE_ARM.
    //
    // Commits a segment with a deliberately stale t_start_clock (value 1,
    // which is ~2s behind the widened clock seeded at 1_000_000_000),
    // drives the ISR, and verifies the engine faults with LATE_ARM instead
    // of silently rebasing.

    // Push segment id=10 (well past any previous IDs from cases 1-11).
    let rc = unsafe {
        kalico_c_api::runtime_handle_push_segment(
            rt, 10,
            0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE, 0xFFFF_FFFE, // UNUSED handles
            0, 500_000, // t_start/t_end (ignored by push in two-phase)
            1, // kinematics = CartesianXyzAndE
            2, // e_mode = Travel
            0, // extrusion_ratio_bits
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "push id=10 should succeed");

    // Commit with a STALE t_start_clock — value 1, which is ~1 billion
    // cycles behind the widened clock (seeded at 1_000_000_000 by the
    // runtime_widened_host_clock stub). This reproduces the bug where
    // push-time timing was passed through to commit instead of being
    // recomputed fresh.
    let mut out_id: u32 = 0;
    let rc = unsafe {
        kalico_c_api::runtime_handle_commit_segment(
            rt,
            10,       // segment_id
            1,        // t_start_clock = 1 (deliberately stale / in the past)
            100_000,  // duration_clocks
            &mut out_id as *mut u32,
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "commit itself should succeed (enqueues to SPSC)");

    // Drive the ISR — this will dequeue the segment and try to arm it.
    // The arm check should find t_start far in the past and fault.
    unsafe {
        kalico_c_api::kalico_runtime_tick_sample(rt);
    }

    // Verify the engine faulted with LATE_ARM.
    let status = unsafe { kalico_c_api::runtime_handle_status(rt) };
    let last_error = unsafe { kalico_c_api::runtime_handle_last_error(rt) };

    assert_eq!(
        status, 3, // RuntimeStatus::Fault = 3
        "engine should be in Fault state after LATE_ARM (got status={})",
        status,
    );
    assert_eq!(
        last_error,
        kalico_c_api::KALICO_FAULT_LATE_ARM,
        "last_error should be KALICO_FAULT_LATE_ARM (0x0010), got 0x{:x}",
        last_error,
    );
}
