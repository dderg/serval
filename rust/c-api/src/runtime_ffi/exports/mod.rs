use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use runtime::RT_STORAGE_SIZE;
use runtime::engine::RuntimeStatus;
use runtime::error::{
    RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_INVALID_HANDLE, RUNTIME_ERR_NOT_INIT,
    RUNTIME_ERR_NULL_PTR, RUNTIME_OK,
};
use runtime::state::{IsrState, RuntimeContext, SharedState};

mod diag;
mod lifecycle;
mod phase_buzz;
mod ring;

pub use diag::*;
pub use lifecycle::*;
pub use phase_buzz::*;
pub use ring::*;

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

macro_rules! guarded_ctx {
    ($rt:expr) => {{
        if $rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        $rt.cast::<RuntimeContext>()
    }};
    ($rt:expr, $default:expr) => {
        guarded_ctx!($rt, $default, $default)
    };
    ($rt:expr, $null_default:expr, $init_default:expr) => {{
        if $rt.is_null() {
            return $null_default;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return $init_default;
        }
        $rt.cast::<RuntimeContext>()
    }};
}
use guarded_ctx;
