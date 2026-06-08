#![allow(unsafe_code)]

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn diag_sq_overflow_capture(qlen: u32, running: u32);
    fn diag_sq_first_push_capture(delta_cycles: i32, cyccnt: u32);
    fn diag_sq_reset_run_flags();
}

#[inline]
pub(crate) fn sq_overflow_capture(qlen: u32, running: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: stores two u32 fields in the persistent diag struct.
    // Called from TIM5 ISR under the sole-producer invariant.
    unsafe {
        diag_sq_overflow_capture(qlen, running);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = (qlen, running);
    }
}

#[inline]
pub(crate) fn sq_first_push_capture(delta_cycles: i32, cyccnt: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: writes first-push timing fields guarded by sq_first_push_seen.
    // Only the first call per run has observable effect.
    unsafe {
        diag_sq_first_push_capture(delta_cycles, cyccnt);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = (delta_cycles, cyccnt);
    }
}

#[inline]
pub fn sq_reset_run_flags() {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: zeroes the per-run sq_* fields in the persistent diag struct.
    // Called from kalico_runtime_reset under the C IRQ guard.
    unsafe {
        diag_sq_reset_run_flags();
    }
}
