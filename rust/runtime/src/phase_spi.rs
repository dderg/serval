// Safe Rust facade over the C-owned blocking TMC5160 register SPI primitives
// (`src/stm32/phase_stepping_spi.c`). The orchestrator in `phase_handover`
// calls this one API; the backend resolves per build — real `extern "C"` on
// MCU / Linux-MCU firmware, a recording shim on host/test so the enter/exit
// sequence is exercisable without hardware.
//
// A register read or RMW that times out returns `None` so the caller raises a
// fail-loud fault instead of trusting a garbage value (a zero MSCNT would snap
// the rotor to electrical angle 0).

pub const GCONF_ADDR: u8 = 0x00;
pub const CHOPCONF_ADDR: u8 = 0x6C;
pub const MSCNT_ADDR: u8 = 0x6A;
pub const GCONF_DIRECT_MODE: u32 = 1 << 16;
pub const GCONF_EN_PWM: u32 = 1 << 2;

#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
mod backend {
    #![allow(unsafe_code)]

    unsafe extern "C" {
        fn phase_stepping_enable_writes();
        fn phase_stepping_disable_writes();
        fn phase_spi_write_register(motor_idx: u8, addr: u8, val: u32) -> i32;
        fn phase_spi_read_register(motor_idx: u8, addr: u8, out: *mut u32) -> i32;
        fn phase_spi_rmw_register(
            motor_idx: u8,
            addr: u8,
            mask: u32,
            set_bits: u32,
            verified: *mut u32,
        ) -> i32;
    }

    pub fn enable_writes() {
        // SAFETY: C global-flag toggle, no aliasing constraints.
        unsafe { phase_stepping_enable_writes() }
    }

    pub fn disable_writes() {
        // SAFETY: C global-flag toggle, no aliasing constraints.
        unsafe { phase_stepping_disable_writes() }
    }

    pub fn write_register(motor_idx: u8, addr: u8, val: u32) -> bool {
        // SAFETY: scalar-only C call; the C side validates motor_idx/bus.
        unsafe { phase_spi_write_register(motor_idx, addr, val) == 0 }
    }

    pub fn read_register(motor_idx: u8, addr: u8) -> Option<u32> {
        let mut out: u32 = 0;
        // SAFETY: `out` is a live local; C writes through it only on success.
        let rc = unsafe { phase_spi_read_register(motor_idx, addr, &mut out) };
        (rc == 0).then_some(out)
    }

    pub fn rmw_register(motor_idx: u8, addr: u8, mask: u32, set_bits: u32) -> Option<u32> {
        let mut verified: u32 = 0;
        // SAFETY: `verified` is a live local; C writes through it only on success.
        let rc = unsafe { phase_spi_rmw_register(motor_idx, addr, mask, set_bits, &mut verified) };
        (rc == 0).then_some(verified)
    }
}

#[cfg(all(any(test, feature = "host"), not(feature = "mcu-linux")))]
mod backend {
    use std::sync::{Mutex, MutexGuard, OnceLock};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Op {
        EnableWrites,
        DisableWrites,
        Write {
            motor: u8,
            addr: u8,
            val: u32,
        },
        Read {
            motor: u8,
            addr: u8,
        },
        Rmw {
            motor: u8,
            addr: u8,
            mask: u32,
            set_bits: u32,
        },
    }

    struct Shim {
        ops: Vec<Op>,
        regs: std::collections::HashMap<(u8, u8), u32>,
        mscnt: std::collections::HashMap<u8, u32>,
    }

    fn shim() -> &'static Mutex<Shim> {
        static SHIM: OnceLock<Mutex<Shim>> = OnceLock::new();
        SHIM.get_or_init(|| {
            Mutex::new(Shim {
                ops: Vec::new(),
                regs: std::collections::HashMap::new(),
                mscnt: std::collections::HashMap::new(),
            })
        })
    }

    fn lock() -> MutexGuard<'static, Shim> {
        shim().lock().unwrap_or_else(|p| p.into_inner())
    }

    pub fn enable_writes() {
        lock().ops.push(Op::EnableWrites);
    }

    pub fn disable_writes() {
        lock().ops.push(Op::DisableWrites);
    }

    pub fn write_register(motor_idx: u8, addr: u8, val: u32) -> bool {
        let mut g = lock();
        g.ops.push(Op::Write {
            motor: motor_idx,
            addr,
            val,
        });
        g.regs.insert((motor_idx, addr), val);
        true
    }

    pub fn read_register(motor_idx: u8, addr: u8) -> Option<u32> {
        let mut g = lock();
        g.ops.push(Op::Read {
            motor: motor_idx,
            addr,
        });
        if addr == super::MSCNT_ADDR {
            return Some(g.mscnt.get(&motor_idx).copied().unwrap_or(0));
        }
        Some(g.regs.get(&(motor_idx, addr)).copied().unwrap_or(0))
    }

    pub fn rmw_register(motor_idx: u8, addr: u8, mask: u32, set_bits: u32) -> Option<u32> {
        let mut g = lock();
        g.ops.push(Op::Rmw {
            motor: motor_idx,
            addr,
            mask,
            set_bits,
        });
        let cur = g.regs.get(&(motor_idx, addr)).copied().unwrap_or(0);
        let next = (cur & !mask) | (set_bits & mask);
        g.regs.insert((motor_idx, addr), next);
        Some(next)
    }

    // ---- test instrumentation ----

    pub fn test_clear() {
        let mut g = lock();
        g.ops.clear();
        g.regs.clear();
        g.mscnt.clear();
    }

    pub fn test_set_mscnt(motor_idx: u8, mscnt: u32) {
        lock().mscnt.insert(motor_idx, mscnt);
    }

    pub fn test_set_register(motor_idx: u8, addr: u8, val: u32) {
        lock().regs.insert((motor_idx, addr), val);
    }

    pub fn test_get_register(motor_idx: u8, addr: u8) -> u32 {
        lock().regs.get(&(motor_idx, addr)).copied().unwrap_or(0)
    }

    pub fn test_ops() -> Vec<Op> {
        lock().ops.clone()
    }
}

pub use backend::*;
