#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::{Mutex, OnceLock};

use runtime::stepping_state::{StepperBindingRust, TMC_CS_OID_NONE};

#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

/// `cargo test` runs intra-binary tests in parallel by default.  Because
/// `INIT_DONE` is a process-global boolean and the runtime pointer is a
/// non-thread-safe raw pointer, every test that calls into the FFI must hold
/// this lock for the duration of its FFI calls.
static TEST_LOCK: Mutex<()> = Mutex::new(());

/// # SAFETY
///
/// All FFI calls in this file are serialised by `TEST_LOCK`, so no two
/// threads ever call into the runtime concurrently; the internal FFI guards
/// (`INIT_DONE` check, null-pointer check) provide a second layer.
struct RtHandle(*mut kalico_c_api::KalicoRuntime);

// SAFETY: see the `RtHandle` doc comment above — serialisation by TEST_LOCK
// plus internal FFI guards makes Send + Sync sound here.
unsafe impl Send for RtHandle {}
unsafe impl Sync for RtHandle {}

static RUNTIME: OnceLock<RtHandle> = OnceLock::new();

fn rt() -> *mut kalico_c_api::KalicoRuntime {
    RUNTIME
        .get_or_init(|| {
            let handle = kalico_c_api::runtime_handle_create();
            assert!(!handle.is_null(), "runtime_handle_create returned null");

            let bindings = [
                StepperBindingRust {
                    stepper_oid: 10,
                    tmc_cs_oid: TMC_CS_OID_NONE,
                    _pad: [0; 2],
                },
                StepperBindingRust {
                    stepper_oid: 11,
                    tmc_cs_oid: TMC_CS_OID_NONE,
                    _pad: [0; 2],
                },
                StepperBindingRust {
                    stepper_oid: 12,
                    tmc_cs_oid: TMC_CS_OID_NONE,
                    _pad: [0; 2],
                },
            ];
            let rc = unsafe {
                kalico_c_api::kalico_runtime_configure_axis(
                    handle,
                    2,
                    0, // mode = Pulse
                    0.00125_f32.to_bits(),
                    64,
                    bindings.as_ptr(),
                    3,
                )
            };
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure_axis failed: {rc}");

            RtHandle(handle)
        })
        .0
}

fn piece_bytes(start_time: u64) -> [u8; 32] {
    let mut piece = [0u8; 32];
    piece[0..8].copy_from_slice(&start_time.to_le_bytes());
    piece[20..24].copy_from_slice(&0.0125_f32.to_le_bytes());
    piece[24..28].copy_from_slice(&0.01_f32.to_le_bytes());
    piece
}

#[test]
fn write_and_commit_correction_roundtrip() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    let piece = piece_bytes(1_000_000);
    unsafe {
        let rc =
            kalico_c_api::kalico_runtime_write_correction_piece(handle, 2, 0, 0, piece.as_ptr());
        assert_eq!(rc, kalico_c_api::KALICO_OK, "write failed: {rc}");
        let rc = kalico_c_api::kalico_runtime_commit_correction(handle, 2, 1, 1);
        assert_eq!(rc, kalico_c_api::KALICO_OK, "commit failed: {rc}");
    }
}

#[test]
fn commit_correction_rejects_when_axis_busy() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    let piece = piece_bytes(2_000_000);
    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_piece(handle, 2, 0, 0, piece.as_ptr());
        assert_eq!(rc, kalico_c_api::KALICO_OK);
        let rc = kalico_c_api::kalico_runtime_commit_head(handle, 2, 1);
        assert_eq!(rc, kalico_c_api::KALICO_OK);

        let rc =
            kalico_c_api::kalico_runtime_write_correction_piece(handle, 2, 0, 0, piece.as_ptr());
        assert_eq!(rc, kalico_c_api::KALICO_OK);
        let rc = kalico_c_api::kalico_runtime_commit_correction(handle, 2, 1, 1);
        assert_eq!(rc, runtime::error::KALICO_ERR_MOTION_IN_PROGRESS);

        kalico_c_api::kalico_runtime_discard_pending(handle);
    }
}

#[test]
fn normal_commit_rejects_when_correction_active() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    let piece = piece_bytes(3_000_000);
    unsafe {
        let rc =
            kalico_c_api::kalico_runtime_write_correction_piece(handle, 2, 0, 0, piece.as_ptr());
        assert_eq!(rc, kalico_c_api::KALICO_OK);
        let rc = kalico_c_api::kalico_runtime_commit_correction(handle, 2, 1, 1);
        assert_eq!(rc, kalico_c_api::KALICO_OK);

        let rc = kalico_c_api::kalico_runtime_write_piece(handle, 2, 0, 0, piece.as_ptr());
        assert_eq!(rc, kalico_c_api::KALICO_OK);
        let rc = kalico_c_api::kalico_runtime_commit_head(handle, 2, 1);
        assert_eq!(rc, runtime::error::KALICO_ERR_CORRECTION_IN_PROGRESS);

        kalico_c_api::kalico_runtime_discard_pending(handle);
    }
}

#[test]
fn correction_ffi_null_rt_is_null_ptr_error() {
    let _guard = TEST_LOCK.lock().unwrap();
    let piece = piece_bytes(0);
    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_correction_piece(
            core::ptr::null_mut(),
            2,
            0,
            0,
            piece.as_ptr(),
        );
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
        let rc = kalico_c_api::kalico_runtime_commit_correction(core::ptr::null_mut(), 2, 1, 1);
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
    }
}
