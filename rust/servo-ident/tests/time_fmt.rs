//! `time_fmt` unit-conversion checks: the epoch, a leap-day boundary, a
//! pre-epoch instant, and one real timestamp cross-checked against Python's
//! `datetime` (the value this module exists to reproduce without a `time`
//! crate dependency).

use std::time::{Duration, SystemTime};

use servo_ident::time_fmt::{iso8601_utc, stamp_utc};

fn at(unix_secs: i64) -> SystemTime {
    if unix_secs >= 0 {
        SystemTime::UNIX_EPOCH + Duration::from_secs(unix_secs as u64)
    } else {
        SystemTime::UNIX_EPOCH - Duration::from_secs((-unix_secs) as u64)
    }
}

#[test]
fn epoch_formats_as_1970() {
    assert_eq!(iso8601_utc(at(0)), "1970-01-01T00:00:00Z");
    assert_eq!(stamp_utc(at(0)), "19700101_000000");
}

#[test]
fn leap_day_2000() {
    assert_eq!(iso8601_utc(at(951_782_400)), "2000-02-29T00:00:00Z");
}

#[test]
fn pre_epoch_instant() {
    assert_eq!(iso8601_utc(at(-1)), "1969-12-31T23:59:59Z");
}

#[test]
fn matches_python_datetime_reference() {
    assert_eq!(iso8601_utc(at(1_783_696_516)), "2026-07-10T15:15:16Z");
    assert_eq!(stamp_utc(at(1_783_696_516)), "20260710_151516");
}
