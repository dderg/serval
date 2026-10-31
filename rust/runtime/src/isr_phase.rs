#![allow(unsafe_code)]

// Phase constants — must match src/generic/fault_handler.h exactly.
pub(crate) const RT_PHASE_ISR_ENTER: u32 = 1;
pub(crate) const RT_PHASE_TICK: u32 = 4;
pub(crate) const RT_PHASE_ISR_EXIT: u32 = 9;

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn runtime_set_isr_phase(phase: u32);
    fn runtime_cyccnt_read() -> u32;
}

#[inline]
pub(crate) fn set_phase(phase: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `runtime_set_isr_phase` performs a single volatile store to a
    // persistent diagnostic struct. No side effects beyond the store; safe to
    // call from any ISR context.
    unsafe {
        runtime_set_isr_phase(phase);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = phase;
    }
}

#[inline]
pub(crate) fn cyccnt() -> u32 {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `runtime_cyccnt_read` is a single DWT CYCCNT MMIO read.
    // Side-effect-free and safe from any ISR context.
    unsafe {
        runtime_cyccnt_read()
    }
    #[cfg(any(test, feature = "host"))]
    {
        0
    }
}
