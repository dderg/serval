//! Nanoseconds on the host-wide `CLOCK_MONOTONIC_RAW` timeline.
//!
//! `std::time::Instant` is per-process; a shared POSIX clock is required for
//! piece `start_time` values to be comparable between the host pump and this
//! endpoint. `CLOCK_MONOTONIC_RAW` is used (not `CLOCK_MONOTONIC`) because the
//! host's `instant_to_f64` anchor is also RAW-domain on Linux
//! (`host-rt` uses `CLOCK_MONOTONIC_RAW` for `monotonic_raw_secs`), so
//! the init-planner seed that pairs `Instant::now()` with this value lands both
//! sides in the same domain and the linear mapping (freq 1e9) stays exact.
//!
//! The DC cycle grid lives in `CLOCK_MONOTONIC` instead — `clock_nanosleep`
//! cannot sleep on RAW — so [`raw_from_monotonic_ns`] bridges grid timestamps
//! into the piece timeline. The bridge is a paired read of both clocks; its
//! noise is the ~100 ns between the two `clock_gettime` calls, and the slow
//! NTP-slew drift between the domains shows up only as a common-mode time
//! offset (a pure delay on every axis), so neither corrupts commanded
//! positions.
//!
//! On macOS (CI / development) `CLOCK_MONOTONIC_RAW` exists and is available
//! via libc. The endpoint binary only ships on Linux; the macOS path runs
//! in tests only.
#![allow(unsafe_code)]

fn clock_ns(clock: libc::clockid_t) -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, fully-initialized `timespec`; `clock_gettime`
    // only writes through the pointer and returns 0 on success for a valid
    // clock id. Both clock ids used here exist on Linux and macOS.
    let rc = unsafe { libc::clock_gettime(clock, &mut ts) };
    assert_eq!(rc, 0, "clock_gettime({clock}) failed");
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

#[must_use]
pub fn monotonic_ns() -> u64 {
    clock_ns(libc::CLOCK_MONOTONIC_RAW)
}

#[must_use]
pub fn raw_from_monotonic_ns(mono_ns: u64) -> u64 {
    let raw = clock_ns(libc::CLOCK_MONOTONIC_RAW) as i64;
    let mono = clock_ns(libc::CLOCK_MONOTONIC) as i64;
    let converted = mono_ns as i64 + (raw - mono);
    assert!(
        converted >= 0,
        "monotonic->raw conversion underflowed: mono_ns={mono_ns} raw={raw} mono={mono}"
    );
    converted as u64
}

#[cfg(test)]
mod tests;
