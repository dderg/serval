use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_NOT_INIT,
    RUNTIME_ERR_NULL_PTR, Runtime, RuntimeContext, RuntimeStatus, SharedState, UnsafeCell,
    guarded_ctx,
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_status(rt: *mut Runtime) -> u8 {
    let ctx = guarded_ctx!(rt, RuntimeStatus::Fault as u8);
    // SAFETY: read-only SharedState atomics; no &mut.
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        (*shared_ptr).runtime_status.load(Ordering::Acquire)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_last_error(rt: *mut Runtime) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: read-only SharedState atomics; no &mut.
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        (*shared_ptr).last_error.load(Ordering::Acquire)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_tick_counter(rt: *mut Runtime) -> u32 {
    let ctx = guarded_ctx!(rt, 0);
    // SAFETY: ISR is sole writer of IsrState; atomic read is safe from foreground.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.tick_counter()
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_fault_detail(rt: *mut Runtime) -> u32 {
    let ctx = guarded_ctx!(rt, 0);
    // SAFETY: read-only SharedState atomics; no &mut.
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        (*shared_ptr).fault_detail.load(Ordering::Acquire)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_tick_blocker_pc(rt: *mut Runtime) -> u32 {
    let ctx = guarded_ctx!(rt, 0);
    // SAFETY: read-only SharedState atomics; no &mut.
    unsafe {
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        (*shared_ptr).tick_blocker_pc.load(Ordering::Acquire)
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_get_heartbeat(
    rt: *mut Runtime,
    out_engine_state: *mut u8,
    out_fault_code: *mut u16,
    out_retired: *mut u32,
    max_axes: usize,
) -> i32 {
    if rt.is_null()
        || out_engine_state.is_null()
        || out_fault_code.is_null()
        || out_retired.is_null()
    {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let engine = &(*isr_ptr).engine;

        let engine_state = engine.status() as u8;
        let fault_code = (engine.last_error() as u32 & 0xFFFF) as u16;
        let num_axes = engine.num_axes as usize;
        let counts = engine.retired_counts();
        let n_write = num_axes.min(max_axes);

        core::ptr::write(out_engine_state, engine_state);
        core::ptr::write(out_fault_code, fault_code);
        for i in 0..n_write {
            out_retired.add(i).write(counts[i]);
        }
        #[allow(clippy::cast_possible_truncation)]
        let result = n_write as i32;
        result
    }
}

/// Foreground-only, call under `irq_save` — the ISR mutates the armed
/// piece and a torn u64 read would fabricate a bogus stall window.
/// Returns 1 with the armed piece window, 0 when nothing is armed,
/// negative on error. `out_occupancy` is the axis ring depth in pieces.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_axis_head_window(
    rt: *mut Runtime,
    axis_idx: u32,
    out_start: *mut u64,
    out_end: *mut u64,
    out_occupancy: *mut u32,
) -> i32 {
    if rt.is_null() || out_start.is_null() || out_end.is_null() || out_occupancy.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let engine = &(*isr_ptr).engine;
        let idx = axis_idx as usize;
        if idx >= engine.num_axes as usize {
            return RUNTIME_ERR_INVALID_ARG;
        }
        *out_occupancy = engine.occupancy_counts().get(idx).copied().unwrap_or(0);
        match engine.armed_window(idx) {
            Some((start, end)) => {
                *out_start = start;
                *out_end = end;
                1
            }
            None => {
                *out_start = 0;
                *out_end = 0;
                0
            }
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_query_motor_state(
    rt: *mut Runtime,
    out_slots: *mut u8,
    out_pos_q16: *mut i32,
    out_vel_q16: *mut i32,
    max: usize,
) -> i32 {
    use runtime::stepping_state::MAX_AXES;
    if rt.is_null() || out_slots.is_null() || out_pos_q16.is_null() || out_vel_q16.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let engine = &(*isr_ptr).engine;
        let num = (engine.num_axes as usize).min(MAX_AXES);
        let mut n = 0usize;
        for i in 0..num {
            if n >= max {
                break;
            }
            if let Some((p, v)) = engine.motor_state(i) {
                out_slots.add(n).write(i as u8);
                #[allow(clippy::cast_possible_truncation)]
                out_pos_q16.add(n).write((p * 65536.0) as i32);
                #[allow(clippy::cast_possible_truncation)]
                out_vel_q16.add(n).write((v * 65536.0) as i32);
                n += 1;
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let result = n as i32;
        result
    }
}
