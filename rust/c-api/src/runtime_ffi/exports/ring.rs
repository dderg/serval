use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_NOT_INIT,
    RUNTIME_ERR_NULL_PTR, RUNTIME_OK, Runtime, RuntimeContext, SharedState, UnsafeCell,
    guarded_ctx,
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_configure_axis(
    rt: *mut Runtime,
    axis_idx: u8,
    mode: u8,
    microstep_distance_f32_bits: u32,
    bindings_ptr: *const runtime::stepping_state::StepperBindingRust,
    stepper_count: u8,
) -> i32 {
    if rt.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let mode_enum = match mode {
        0 => runtime::stepping_state::StepMode::Pulse,
        1 => runtime::stepping_state::StepMode::Phase,
        _ => return RUNTIME_ERR_INVALID_ARG,
    };
    let mstep_dist = f32::from_bits(microstep_distance_f32_bits);
    let bindings: &[runtime::stepping_state::StepperBindingRust] = if stepper_count == 0 {
        &[]
    } else if bindings_ptr.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    } else {
        // SAFETY: caller guarantees bindings_ptr is valid for stepper_count elements; slice borrow does not escape.
        unsafe { core::slice::from_raw_parts(bindings_ptr, stepper_count as usize) }
    };
    let ctx = rt.cast::<RuntimeContext>();
    // SAFETY: foreground-only; §11.2 raw-pointer projection; command dispatch serialised against TIM5.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let rc = (*isr_ptr)
            .engine
            .configure_axis(axis_idx, mode_enum, mstep_dist, bindings);
        if rc != RUNTIME_OK {
            return rc;
        }
        let shared_ptr: *const runtime::state::SharedState = core::ptr::addr_of!((*ctx).shared);
        if (axis_idx as usize) < runtime::state::MAX_STEPPER_OIDS {
            let step_mode = (*shared_ptr).step_modes[axis_idx as usize]
                .load(core::sync::atomic::Ordering::Acquire);
            if step_mode == runtime::state::StepMode::Modulated as u8 {
                if let Some(Some(axis)) = (*isr_ptr).engine.stepping_axes.get_mut(axis_idx as usize)
                {
                    axis.mode.store(
                        runtime::stepping_state::StepMode::Phase as u8,
                        core::sync::atomic::Ordering::Release,
                    );
                }
            }
        }
        RUNTIME_OK
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_set_axis_mode(
    rt: *mut Runtime,
    axis_idx: u8,
    new_mode: u8,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.set_axis_mode(axis_idx, new_mode)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_set_stepper_offset(
    rt: *mut Runtime,
    stepper_idx: u8,
    delta_microsteps: i32,
    max_microsteps_per_sample: u16,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; &SharedState borrow is independent of &mut IsrState — SharedState is atomics-only.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let shared: &SharedState = &*shared_ptr;
        (*isr_ptr).engine.set_stepper_offset(
            shared,
            stepper_idx,
            delta_microsteps,
            max_microsteps_per_sample,
        )
    }
}
