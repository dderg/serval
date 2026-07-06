use super::{clock_ns, monotonic_ns, raw_from_monotonic_ns};

const CONVERSION_TOLERANCE_NS: u64 = 50_000_000;

#[test]
fn converting_current_monotonic_lands_on_current_raw() {
    let mono_now = clock_ns(libc::CLOCK_MONOTONIC);
    let converted = raw_from_monotonic_ns(mono_now);
    let raw_now = monotonic_ns();
    assert!(
        converted.abs_diff(raw_now) < CONVERSION_TOLERANCE_NS,
        "converted={converted} raw_now={raw_now}"
    );
}

#[test]
fn conversion_preserves_intervals() {
    let mono_now = clock_ns(libc::CLOCK_MONOTONIC);
    let step_ns = 1_000_000;
    let a = raw_from_monotonic_ns(mono_now);
    let b = raw_from_monotonic_ns(mono_now + step_ns);
    let interval = b.wrapping_sub(a);
    assert!(
        interval.abs_diff(step_ns) < CONVERSION_TOLERANCE_NS,
        "interval={interval} expected~{step_ns}"
    );
}

#[test]
fn monotonic_ns_advances() {
    let a = monotonic_ns();
    let b = monotonic_ns();
    assert!(b >= a);
}
