// FFI seam for the sample-stream executor. `src/sample_commands.c` owns the
// DECL_COMMANDs and the wire decode; everything here is a thin projection onto
// `Engine`'s sample entry points, which latch their own faults.

use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_NOT_INIT,
    RUNTIME_ERR_NULL_PTR, RUNTIME_OK, Runtime, RuntimeContext, SharedState, UnsafeCell,
    guarded_ctx,
};

use runtime::sample_exec::widen_wire_clock;

/// # Safety
/// `data` must be valid for `data_len` bytes, or null when `data_len == 0`.
unsafe fn payload<'a>(data: *const u8, data_len: u16) -> Option<&'a [u8]> {
    if data_len == 0 {
        return Some(&[]);
    }
    if data.is_null() {
        return None;
    }
    // SAFETY: caller guarantees `data` covers `data_len` bytes; the borrow does
    // not outlive the command that produced it.
    Some(unsafe { core::slice::from_raw_parts(data, usize::from(data_len)) })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_anchor(
    rt: *mut Runtime,
    oid: u8,
    clock: u32,
    position: i32,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground command path, serialised against TIM5 by the caller's
    // irq_save; raw-pointer projection never forms `&mut RuntimeContext`.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        let now = runtime::clock::read_widened_now(shared);
        (*isr_ptr)
            .engine
            .sample_anchor(shared, oid, widen_wire_clock(now, clock), position);
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_run(
    rt: *mut Runtime,
    oid: u8,
    interval_ticks: u32,
    count: u8,
    data: *const u8,
    data_len: u16,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: as `runtime_sample_anchor`, plus the payload contract above.
    unsafe {
        let Some(bytes) = payload(data, data_len) else {
            return RUNTIME_ERR_NULL_PTR;
        };
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        (*isr_ptr)
            .engine
            .sample_push_run(shared, oid, interval_ticks, count, bytes);
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_overlay(
    rt: *mut Runtime,
    oid: u8,
    clock: u32,
    interval_ticks: u32,
    count: u8,
    data: *const u8,
    data_len: u16,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: as `runtime_sample_anchor`, plus the payload contract above.
    unsafe {
        let Some(bytes) = payload(data, data_len) else {
            return RUNTIME_ERR_NULL_PTR;
        };
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        let now = runtime::clock::read_widened_now(shared);
        (*isr_ptr).engine.sample_push_overlay(
            shared,
            oid,
            widen_wire_clock(now, clock),
            interval_ticks,
            count,
            bytes,
        );
    }
    RUNTIME_OK
}

/// Executed position for `sample_get_position`. Mirrors `stepper_get_position`:
/// what actually reached the coils, not what is queued.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_query(
    rt: *mut Runtime,
    oid: u8,
    out_clock: *mut u64,
    out_position: *mut i32,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    if out_clock.is_null() || out_position.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    // SAFETY: foreground read of ISR-owned lane state under the caller's
    // irq_save; out pointers checked non-null above.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        let Some((clock, position)) = (*isr_ptr).engine.sample_executed(oid) else {
            runtime::fault_helpers::raise_sample_lane_unknown(shared, oid);
            return RUNTIME_ERR_INVALID_ARG;
        };
        out_clock.write(clock);
        out_position.write(position);
    }
    RUNTIME_OK
}

/// trsync trip: publish a halt at `halt_clock`. Safe from the trip's IRQ
/// context — the next tick applies it, so `IsrState` is never touched here.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_halt(rt: *mut Runtime, halt_clock: u64) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: publishes through `SharedState` atomics only.
    unsafe {
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        runtime::engine::Engine::sample_request_halt(shared, halt_clock);
    }
    RUNTIME_OK
}
