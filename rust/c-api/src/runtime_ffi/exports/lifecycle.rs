use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_NOT_INIT, RUNTIME_ERR_NULL_PTR, RUNTIME_OK, Runtime,
    RuntimeContext, SharedState, UnsafeCell, guarded_ctx, rt_storage, runtime_cyccnt_read,
};

#[unsafe(no_mangle)]
pub extern "C" fn runtime_handle_create() -> *mut Runtime {
    // Plain store not compare_exchange: Renode H7 v1.16 silently drops STREXB, leaving INIT_DONE=0 after CAS succeeds in code.
    if INIT_DONE.load(Ordering::Relaxed) {
        return core::ptr::null_mut();
    }
    // SAFETY: single-threaded init; no other context observes rt_storage until INIT_DONE is published. rt_storage provenance covers the full buffer; RuntimeContext fits (const_assert above).
    unsafe {
        #[cfg(target_os = "none")]
        let rt_ptr: *mut RuntimeContext = rt_storage.get().cast::<RuntimeContext>();
        #[cfg(not(target_os = "none"))]
        let rt_ptr: *mut RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
        debug_assert_eq!(
            (rt_ptr as usize) % core::mem::align_of::<RuntimeContext>(),
            0,
            "rt_storage alignment mismatch — linker placed it unaligned"
        );
        RuntimeContext::init(rt_ptr);
        INIT_DONE.store(true, Ordering::Release);
        rt_ptr.cast::<Runtime>()
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_tick_sample(rt: *mut Runtime) {
    let ctx = guarded_ctx!(rt);
    // SAFETY: rt non-null, INIT_DONE=true. TIM5 is the sole writer of IsrState. UnsafeCell::raw_get yields provenance without a shared ref.
    unsafe {
        let raw = runtime_cyccnt_read();
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let isr: &mut IsrState = &mut *isr_ptr;
        let shared: &SharedState = &*shared_ptr;
        runtime::tick::isr_sample_tick(isr, shared, raw);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_seed_widen(rt: *mut Runtime, baseline_widened_clock: u64) {
    if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
        return;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).widen_state.seed_high(baseline_widened_clock);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_get_stepper_count(rt: *mut Runtime, oid: u8) -> i32 {
    use runtime::state::MAX_STEPPER_OIDS;
    if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
        return 0;
    }
    if oid as usize >= MAX_STEPPER_OIDS {
        return 0;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        (*shared_ptr).stepper_counts[oid as usize].load(Ordering::Acquire)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_seed_position(
    rt: *mut Runtime,
    x_q16: i32,
    y_q16: i32,
    z_q16: i32,
) -> i32 {
    if rt.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let x = x_q16 as f32 / 65536.0;
    let y = y_q16 as f32 / 65536.0;
    let z = z_q16 as f32 / 65536.0;
    let ctx = rt.cast::<RuntimeContext>();
    // SAFETY: foreground-only; single-threaded command dispatch, no concurrent TIM5.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.seed_position([x, y, z]);
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_clock_sync_request(
    rt: *mut Runtime,
    request_id: u32,
    host_send_time_lo: u32,
    host_send_time_hi: u32,
    out_mcu_clock: *mut u64,
) -> i32 {
    if rt.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    // SAFETY: single u32 reads of Klipper globals, safe from non-ISR context.
    let mcu_clock = unsafe {
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        let low = timer_read_time();
        let high = stats_send_time_high + ((low < stats_send_time) as u32);
        ((high as u64) << 32) | (low as u64)
    };
    let _ = (request_id, host_send_time_lo, host_send_time_hi);
    if !out_mcu_clock.is_null() {
        unsafe { *out_mcu_clock = mcu_clock };
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_reset(rt: *mut Runtime) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground under C-side IRQ guard; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.reset();
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        for slot in shared.phase_slot_idx.iter() {
            slot.store(0xFF, Ordering::Release);
        }
        shared.phase_motor_count.store(0, Ordering::Release);
        for m in shared.step_modes.iter() {
            m.store(runtime::state::StepMode::StepTime as u8, Ordering::Release);
        }
    }
    #[cfg(not(any(test, feature = "host")))]
    runtime::step_queue::reset_all_queues();
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_now_ticks(rt: *mut Runtime) -> u64 {
    if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
        return 0;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        runtime::clock::read_widened_now(&*shared_ptr)
    }
}
