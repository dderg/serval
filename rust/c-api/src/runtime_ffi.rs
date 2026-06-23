// Every FFI entry projects to &mut FgState or &mut IsrState (disjoint memory) via
// core::ptr::addr_of! + UnsafeCell::raw_get; no &mut RuntimeContext is ever materialised.
// See docs/rewrite/mcu-c-rust-boundary.md.

#![allow(unsafe_code)]

#[cfg(feature = "header-runtime")]
pub mod exports {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, Ordering};

    use runtime::RT_STORAGE_SIZE;
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        RUNTIME_ERR_CAPABILITY_MISSING, RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_INVALID_HANDLE,
        RUNTIME_ERR_NOT_INIT, RUNTIME_ERR_NULL_PTR, RUNTIME_ERR_PROTOCOL_VERSION_UNSUPPORTED,
        RUNTIME_OK,
    };
    use runtime::state::{IsrState, RuntimeContext, SharedState};

    #[allow(missing_debug_implementations)]
    #[repr(C)]
    pub struct Runtime {
        _private: [u8; 0],
    }

    #[cfg(target_os = "none")]
    unsafe extern "C" {
        static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
    }

    #[cfg(not(target_os = "none"))]
    #[repr(C, align(16))]
    struct HostRtStorage(UnsafeCell<[u8; RT_STORAGE_SIZE]>);
    // SAFETY: half-split aliasing + INIT_DONE guard ensure no concurrent &mut; UnsafeCell::raw_get is the only access path.
    #[cfg(not(target_os = "none"))]
    unsafe impl Sync for HostRtStorage {}
    #[cfg(not(target_os = "none"))]
    #[allow(non_upper_case_globals)]
    static rt_storage: HostRtStorage = HostRtStorage(UnsafeCell::new([0u8; RT_STORAGE_SIZE]));

    const _: () = {
        assert!(
            core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
            "RuntimeContext outgrew RT_STORAGE_SIZE — bump Kconfig storage size"
        );
    };

    const _: () = {
        assert!(
            core::mem::align_of::<RuntimeContext>() <= 16,
            "RuntimeContext alignment > 16 — bump _Alignas in runtime_storage.c"
        );
    };

    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    unsafe extern "C" {
        fn runtime_cyccnt_read() -> u32;
        fn event_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
    }

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
    pub unsafe extern "C" fn runtime_handle_check_blob_version(
        payload_ptr: *const u8,
        payload_len: u32,
    ) -> i32 {
        if payload_ptr.is_null() || payload_len == 0 {
            return RUNTIME_ERR_PROTOCOL_VERSION_UNSUPPORTED;
        }
        // SAFETY: caller contracts payload_ptr is valid for payload_len bytes.
        let blob = unsafe { core::slice::from_raw_parts(payload_ptr, payload_len as usize) };
        match runtime::wire::check_version(blob) {
            Ok(()) => RUNTIME_OK,
            Err(_) => RUNTIME_ERR_PROTOCOL_VERSION_UNSUPPORTED,
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_tick_sample(rt: *mut Runtime) {
        if rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: rt non-null, INIT_DONE=true. TIM5 is the sole writer of IsrState; ISR owns ring tail, foreground writes only HEAD positions not yet seen by ISR. UnsafeCell::raw_get yields provenance without a shared ref.
        unsafe {
            let raw = runtime_cyccnt_read();
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let ps_ptr: *mut [runtime::piece_ring::PieceEntry; runtime::state::TOTAL_RING_PIECES] =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).piece_storage));
            let storage: &mut [runtime::piece_ring::PieceEntry] = &mut *ps_ptr;
            let isr: &mut IsrState = &mut *isr_ptr;
            let shared: &SharedState = &*shared_ptr;
            runtime::tick::isr_sample_tick(isr, shared, storage, raw);
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_status(rt: *mut Runtime) -> u8 {
        if rt.is_null() {
            return RuntimeStatus::Fault as u8;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RuntimeStatus::Fault as u8;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).runtime_status.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_error(rt: *mut Runtime) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).last_error.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_counter(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: ISR is sole writer of IsrState; atomic read is safe from foreground.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.tick_counter()
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_get_tick_counter(rt: *mut Runtime) -> u32 {
        unsafe { runtime_handle_tick_counter(rt) }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_widened_now(rt: *mut Runtime) -> u64 {
        // rt is unused (widening reads Klipper globals), but kept for ABI stability.
        let _ = rt;
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        // SAFETY: timer_read_time is a u32 MMIO read, safe from non-ISR context. stats_send_time* are u32 globals; torn reads self-correct within ~5 s.
        unsafe {
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            ((high as u64) << 32) | (low as u64)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_modulated_elapsed_lo(rt: *mut Runtime) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_modulated_elapsed_lo
                .load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_modulated_duration_lo(rt: *mut Runtime) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_modulated_duration_lo
                .load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_modulated_retire_attempts(rt: *mut Runtime) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .modulated_retire_attempts
                .load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_modulated_retire_successes(rt: *mut Runtime) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .modulated_retire_successes
                .load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_retire_consumers_after_clear(
        rt: *mut Runtime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_retire_consumers_after_clear
                .load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_fault_detail(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).fault_detail.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_func.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_pc(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_pc.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_exc(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_exc.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_get_axis_steps_per_mm(
        rt: *mut Runtime,
        oid: u8,
    ) -> f32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_steps_per_mm(oid as usize)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_seed_widen(
        rt: *mut Runtime,
        baseline_widened_clock: u64,
    ) {
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
    pub unsafe extern "C" fn runtime_get_axis_motor(rt: *mut Runtime, oid: u8) -> f32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_last_motor(oid as usize)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_get_last_timing(
        rt: *mut Runtime,
        now_out: *mut u64,
        t_start_out: *mut u64,
        duration_out: *mut u64,
    ) {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let (n, ts, dur) = (*isr_ptr).engine.debug_last_timing();
            if !now_out.is_null() {
                *now_out = n;
            }
            if !t_start_out.is_null() {
                *t_start_out = ts;
            }
            if !duration_out.is_null() {
                *duration_out = dur;
            }
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_get_axis_accumulator(rt: *mut Runtime, oid: u8) -> f64 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_accumulator(oid as usize)
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
    pub unsafe extern "C" fn runtime_stream_flush(
        rt: *mut Runtime,
        out_credit_epoch: *mut u32,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        // SAFETY: rt non-null + INIT_DONE verified; flush() performs its own half-split projections.
        unsafe { runtime::stream::flush(rt.cast::<RuntimeContext>(), out_credit_epoch) }
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
    pub unsafe extern "C" fn runtime_set_step_mode(
        rt: *mut Runtime,
        stepper_idx: u8,
        mode: u8,
        mcu_supports_phase: u8,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let mode = match runtime::state::StepMode::from_u8(mode) {
            Some(m) => m,
            None => return RUNTIME_ERR_INVALID_ARG,
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: step_modes are AtomicU8 in SharedState; shared &SharedState, no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match runtime::state::set_step_mode(shared, stepper_idx, mode, mcu_supports_phase != 0)
            {
                Ok(()) => RUNTIME_OK,
                Err(runtime::state::SetStepModeError::CapabilityMissing) => {
                    RUNTIME_ERR_CAPABILITY_MISSING
                }
                Err(runtime::state::SetStepModeError::OutOfRange) => RUNTIME_ERR_INVALID_HANDLE,
            }
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_bind_phase_motor(
        rt: *mut Runtime,
        motor_idx: u8,
        slot_idx: u8,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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
    pub unsafe extern "C" fn runtime_enqueue_success_lo(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared
                .producer_enqueue_success_total
                .load(Ordering::Acquire) as u32
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_push_seg_all_unused_lo(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.push_segment_all_unused_total.load(Ordering::Acquire) as u32
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_last_push_x_handle(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_x_handle_packed.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_last_push_y_handle(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_y_handle_packed.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_last_push_consumers_remaining(rt: *mut Runtime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_consumers_remaining.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_get_step_mode(rt: *mut Runtime, stepper_idx: u8) -> u8 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() || (stepper_idx as usize) >= MAX_STEPPER_OIDS {
            return 0xFF;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0xFF;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: step_modes are AtomicU8 in SharedState; shared &SharedState, no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            shared.step_modes[stepper_idx as usize].load(Ordering::Acquire)
        }
    }

    fn runtime_handle_or_null() -> Option<*const RuntimeContext> {
        if !INIT_DONE.load(Ordering::Acquire) {
            return None;
        }
        #[cfg(target_os = "none")]
        let rt_ptr: *const RuntimeContext = {
            // SAFETY: .get() on C extern static yields a raw pointer without an aliasing ref; gated by INIT_DONE.
            #[allow(unsafe_code)]
            unsafe {
                rt_storage.get().cast::<RuntimeContext>()
            }
        };
        #[cfg(not(target_os = "none"))]
        let rt_ptr: *const RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
        Some(rt_ptr)
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn runtime_get_dispatcher_floor_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: rt_storage projection; read-only SharedState atomics, no &mut.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr)
                .dispatcher_floor_cycles
                .load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn runtime_get_sample_period_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: rt_storage projection; read-only SharedState atomics, no &mut.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).sample_period_cycles.load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

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
                    if let Some(Some(axis)) =
                        (*isr_ptr).engine.stepping_axes.get_mut(axis_idx as usize)
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
    pub unsafe extern "C" fn runtime_reset(rt: *mut Runtime) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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
    pub unsafe extern "C" fn runtime_discard_pending(rt: *mut Runtime) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.discard_pending();
        }
        RUNTIME_OK
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_gate_pieces(rt: *mut Runtime) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.gate_pieces();
        }
        RUNTIME_OK
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_ungate_pieces(rt: *mut Runtime) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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
            let entry =
                core::ptr::read_unaligned(piece_ptr.cast::<runtime::piece_ring::PieceEntry>());
            axis.ring.write_slot(storage, slot, entry);
        }
        RUNTIME_OK
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_commit_head(
        rt: *mut Runtime,
        axis_idx: u8,
        new_head: u32,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
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

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_phase_jog_to(
        rt: *mut Runtime,
        stepper_oid: u8,
        target_phase: u16,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; &SharedState borrow is independent of &mut IsrState — SharedState is atomics-only.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            (*isr_ptr).engine.phase_jog_to(
                shared,
                stepper_oid,
                target_phase,
                max_microsteps_per_sample,
            )
        }
    }

    #[unsafe(no_mangle)]
    #[allow(clippy::too_many_arguments)]
    pub unsafe extern "C" fn runtime_resonance_buzz(
        rt: *mut Runtime,
        axis_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; §11.2 raw-pointer projection; command dispatch serialised against TIM5.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let now_cycle = runtime_cyccnt_read();
            (*isr_ptr).engine.resonance_buzz(
                &*shared_ptr,
                axis_mask,
                sign_mask,
                freq_start_millihz,
                freq_end_millihz,
                amplitude_nm,
                duration_ms,
                ramp_ms,
                now_cycle,
            )
        }
    }

    /// Step-output consumer entry for a phase-mode buzz: drive `axis_idx`'s coils
    /// to base + `offset_steps` via XDIRECT. Called from `step_output_event` (TIM3
    /// ISR), which forwards the runtime handle. Safe against the motion tick: TIM3
    /// and TIM5 share NVIC priority (cannot interleave) and the tick skips its
    /// phase dispatch for an XDIRECT-buzzing axis, so this is the sole coil writer.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_emit_xdirect(
        rt: *mut Runtime,
        axis_idx: u8,
        offset_steps: i32,
    ) {
        if rt.is_null() {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: §11.2 raw-pointer projection; sole coil writer (see doc above).
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*isr_ptr)
                .engine
                .emit_xdirect_buzz(axis_idx as usize, offset_steps, &*shared_ptr);
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_phase_align_to(
        rt: *mut Runtime,
        stepper_oid: u8,
        target_phase: u16,
    ) -> i32 {
        if rt.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; §11.2 raw-pointer projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.phase_align_to(stepper_oid, target_phase)
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
        if out_axis_idx.is_null()
            || out_mode.is_null()
            || out_phase.is_null()
            || out_settled.is_null()
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

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_get_occupancy(
        rt: *mut Runtime,
        out_occupancy: *mut u32,
        max_axes: usize,
    ) -> i32 {
        if rt.is_null() || out_occupancy.is_null() {
            return RUNTIME_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RUNTIME_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let engine = &(*isr_ptr).engine;
            let num_axes = engine.num_axes as usize;
            let counts = engine.occupancy_counts();
            let n_write = num_axes.min(max_axes);
            for i in 0..n_write {
                out_occupancy.add(i).write(counts[i]);
            }
            #[allow(clippy::cast_possible_truncation)]
            let result = n_write as i32;
            result
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
}
