use super::{
    INIT_DONE, IsrState, Ordering, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_NOT_INIT,
    RUNTIME_ERR_NULL_PTR, RUNTIME_OK, Runtime, RuntimeContext, SharedState, UnsafeCell,
    event_log_emit, guarded_ctx,
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_configure_axis(
    rt: *mut Runtime,
    axis_idx: u8,
    mode: u8,
    microstep_distance_f32_bits: u32,
    ring_depth: u16,
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
    let total_ring_pieces = runtime::state::TOTAL_RING_PIECES;
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let rc = (*isr_ptr).engine.configure_axis(
            axis_idx,
            mode_enum,
            mstep_dist,
            ring_depth as usize,
            bindings,
            total_ring_pieces,
        );
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
pub unsafe extern "C" fn runtime_gate_pieces(rt: *mut Runtime) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.gate_pieces();
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_pieces_gated(rt: *mut Runtime) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        i32::from((*isr_ptr).engine.pieces_gated())
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_ungate_pieces(rt: *mut Runtime) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr).engine.ungate_pieces()
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_write_piece(
    rt: *mut Runtime,
    axis_idx: u8,
    start_slot: u16,
    index: u8,
    piece_ptr: *const u8,
) -> i32 {
    if rt.is_null() || piece_ptr.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    // SAFETY: §11.2 foreground-only. ISR pops ring tail; foreground writes slots only, never advances head here. piece_ptr is unaligned (protocol frame offset); PieceEntry has no invalid bit patterns.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let ps_ptr: *mut [runtime::piece_ring::PieceEntry; runtime::state::TOTAL_RING_PIECES] =
            UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).piece_storage));
        let storage: &mut [runtime::piece_ring::PieceEntry] = &mut *ps_ptr;
        let Some(axis) = (*isr_ptr)
            .engine
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return RUNTIME_ERR_INVALID_ARG;
        };
        if !axis.ring.is_configured() {
            return RUNTIME_ERR_INVALID_ARG;
        }
        let depth = axis.ring.ring_depth;
        let slot = (start_slot as usize + index as usize) % depth;
        let entry = core::ptr::read_unaligned(piece_ptr.cast::<runtime::piece_ring::PieceEntry>());
        // Write acceptance: a piece the ISR cannot arm must never enter the
        // ring. The parser rejects bad coeff_count already; duration is
        // checked here because 2/duration is computed at arm.
        if entry.coeff_count == 0
            || entry.coeff_count as usize > runtime::piece_ring::MAX_PIECE_COEFFS
            || !(entry.duration > 0.0)
        {
            return RUNTIME_ERR_INVALID_ARG;
        }
        axis.ring.write_slot(storage, slot, entry);
    }
    RUNTIME_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_commit_head(rt: *mut Runtime, axis_idx: u8, new_head: u32) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: §11.2 foreground-only. ring.head is a plain u32 written only by foreground; on single-core ARMv7E-M exception entry/return are memory barriers — no explicit fence needed.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        if (*isr_ptr).engine.pieces_gated() {
            return runtime::error::RUNTIME_ERR_STREAM_HALTED;
        }
        let Some(axis) = (*isr_ptr)
            .engine
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return RUNTIME_ERR_INVALID_ARG;
        };
        if !axis.ring.is_configured() {
            return RUNTIME_ERR_INVALID_ARG;
        }
        match axis.ring.commit_head(new_head) {
            runtime::piece_ring::CommitOutcome::Applied
            | runtime::piece_ring::CommitOutcome::Stale => {}
            runtime::piece_ring::CommitOutcome::Overcommit => {
                const LOG_LEVEL_ERROR: u8 = 3;
                const CODE_FLAG_OVERCOMMIT: u16 = 0x100;
                event_log_emit(
                    LOG_LEVEL_ERROR,
                    runtime::log_codes::SUBSYSTEM_RUNTIME,
                    runtime::log_codes::EVENT_RUNTIME_RING_STATE,
                    u16::from(axis_idx) | CODE_FLAG_OVERCOMMIT,
                    axis.ring.head,
                    axis.ring.retired,
                );
                return runtime::error::RUNTIME_ERR_RING_FULL;
            }
        }
    }
    RUNTIME_OK
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
pub unsafe extern "C" fn runtime_set_axis_step_budget(
    rt: *mut Runtime,
    axis_idx: u8,
    max_steps_per_sample: u32,
) -> i32 {
    let ctx = guarded_ctx!(rt, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_NOT_INIT);
    // SAFETY: foreground-only; §11.2 raw-pointer projection.
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        (*isr_ptr)
            .engine
            .set_axis_step_budget(axis_idx, max_steps_per_sample)
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

#[cfg(feature = "host")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_install_step_queues(rt: *mut Runtime, queues: *mut u8) -> i32 {
    if rt.is_null() || queues.is_null() {
        return RUNTIME_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return RUNTIME_ERR_NOT_INIT;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let q0 = queues.cast::<runtime::step_queue::StepQueue>();
        let ptrs: [*mut runtime::step_queue::StepQueue; runtime::stepping_state::N_AXES] = [
            q0,
            q0.add(1),
            q0.add(2),
            q0.add(3),
            q0.add(4),
            q0.add(5),
            q0.add(6),
            q0.add(7),
        ];
        (*isr_ptr).engine.test_install_step_queues(ptrs);
    }
    RUNTIME_OK
}
