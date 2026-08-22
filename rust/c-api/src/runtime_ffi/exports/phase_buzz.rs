use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_INVALID_HANDLE,
    RUNTIME_ERR_NOT_INIT, RUNTIME_ERR_NULL_PTR, RUNTIME_OK, Runtime, RuntimeContext, SharedState,
    UnsafeCell, guarded_ctx,
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_bind_phase_motor(
    rt: *mut Runtime,
    motor_idx: u8,
    slot_idx: u8,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_INVALID_HANDLE, RUNTIME_ERR_NOT_INIT);
    // SAFETY: phase_slot_idx/phase_motor_count/step_modes are atomics in
    // SharedState; shared &SharedState, no &mut. Foreground-only caller.
    unsafe {
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        match runtime::state::bind_phase_motor(shared, motor_idx, slot_idx) {
            Ok(()) => RUNTIME_OK,
            Err(_) => RUNTIME_ERR_INVALID_ARG,
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_phase_jog_to(
    rt: *mut Runtime,
    stepper_oid: u8,
    target_phase: u16,
    max_microsteps_per_sample: u16,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; &SharedState borrow is independent of &mut IsrState — SharedState is atomics-only.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let shared: &SharedState = &*shared_ptr;
        (*isr_ptr)
            .engine
            .phase_jog_to(shared, stepper_oid, target_phase, max_microsteps_per_sample)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_phase_align_to(
    rt: *mut Runtime,
    stepper_oid: u8,
    target_phase: u16,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.phase_align_to(stepper_oid, target_phase)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_seed_axis_count(
    rt: *mut Runtime,
    axis_idx: u8,
    count: i32,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.seed_axis_count(axis_idx, count)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_get_phase_state(
    rt: *mut Runtime,
    stepper_oid: u8,
    out_axis_idx: *mut u8,
    out_mode: *mut u8,
    out_phase: *mut u16,
    out_settled: *mut u8,
) -> i32 {
    if rt.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if out_axis_idx.is_null() || out_mode.is_null() || out_phase.is_null() || out_settled.is_null()
    {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let Some(q) = (*isr_ptr).engine.phase_state(stepper_oid) else {
            return RUNTIME_ERR_INVALID_ARG;
        };
        *out_axis_idx = q.axis_idx;
        *out_mode = q.mode;
        *out_phase = q.phase;
        *out_settled = u8::from(q.settled);
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_get_xdirect_write_count() -> u32 {
    #[cfg(not(target_os = "none"))]
    {
        runtime::test_xdirect_capture::count() as u32
    }
    #[cfg(target_os = "none")]
    {
        0
    }
}
