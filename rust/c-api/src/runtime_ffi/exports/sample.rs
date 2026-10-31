// FFI seam for the sample-stream executor. `src/sample_commands.c` owns the
// DECL_COMMANDs and the wire decode; everything here is a thin projection onto
// `Engine`'s sample entry points, which latch their own faults.

use super::{
    FaultCode, INIT_DONE, IsrState, Ordering, Runtime, RuntimeContext, SharedState, UnsafeCell,
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
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
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
    FaultCode::None.as_i32()
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
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
    // SAFETY: as `runtime_sample_anchor`, plus the payload contract above.
    unsafe {
        let Some(bytes) = payload(data, data_len) else {
            return FaultCode::NullPtr.as_i32();
        };
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        (*isr_ptr)
            .engine
            .sample_push_run(shared, oid, interval_ticks, count, bytes);
    }
    FaultCode::None.as_i32()
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
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
    // SAFETY: as `runtime_sample_anchor`, plus the payload contract above.
    unsafe {
        let Some(bytes) = payload(data, data_len) else {
            return FaultCode::NullPtr.as_i32();
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
    FaultCode::None.as_i32()
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
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
    if out_clock.is_null() || out_position.is_null() {
        return FaultCode::NullPtr.as_i32();
    }
    // SAFETY: foreground read of ISR-owned lane state under the caller's
    // irq_save; out pointers checked non-null above.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        let Some((clock, position)) = (*isr_ptr).engine.sample_executed(oid) else {
            runtime::fault_helpers::raise_sample_lane_unknown(shared, oid);
            return FaultCode::InvalidArg.as_i32();
        };
        out_clock.write(clock);
        out_position.write(position);
    }
    FaultCode::None.as_i32()
}

/// trsync trip: publish a halt at `halt_clock`. Safe from the trip's IRQ
/// context — the next tick applies it, so `IsrState` is never touched here.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_halt(rt: *mut Runtime, halt_clock: u64) -> i32 {
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
    // SAFETY: publishes through `SharedState` atomics only.
    unsafe {
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        runtime::engine::Engine::sample_request_halt(shared, halt_clock);
    }
    FaultCode::None.as_i32()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_barrier(rt: *mut Runtime, oid: u8, seq: u32) -> i32 {
    let ctx = guarded_ctx!(rt, FaultCode::NullPtr.as_i32(), FaultCode::NotInit.as_i32());
    // SAFETY: as `runtime_sample_anchor`.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
        (*isr_ptr).engine.sample_push_barrier(shared, oid, seq);
    }
    FaultCode::None.as_i32()
}

/// Pop one fence playback has passed. Returns 1 when one was written to the
/// out params, 0 when none is ready. The caller loops until 0.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_sample_take_barrier_ack(
    rt: *mut Runtime,
    out_oid: *mut u8,
    out_seq: *mut u32,
) -> i32 {
    let ctx = guarded_ctx!(rt, 0, 0);
    if out_oid.is_null() || out_seq.is_null() {
        return 0;
    }
    // SAFETY: foreground pop of lane-owned state under the caller's irq_save;
    // out pointers checked non-null above.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let Some((oid, seq)) = (*isr_ptr).engine.sample_take_barrier_ack() else {
            return 0;
        };
        out_oid.write(oid);
        out_seq.write(seq);
    }
    1
}
