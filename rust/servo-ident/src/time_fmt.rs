//! UTC timestamp formatting for the dashboard and the demo generator —
//! `iso8601_utc` for `results.json`/`manifest.json`-style stamps,
//! `stamp_utc` for the `<tag>_<YYYYmmdd_HHMMSS>` run-directory suffix.
//! Both derive from `SystemTime` with Howard Hinnant's `civil_from_days`
//! algorithm so the crate carries no `time`/`chrono` dependency.

use std::time::SystemTime;

fn civil_from_unix(secs: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = secs.div_euclid(86400);
    let rem = secs.rem_euclid(86400);
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = if m <= 2 { y + 1 } else { y };
    let (hh, mm, ss) = (rem / 3600, (rem % 3600) / 60, rem % 60);
    (year, m, d, hh as u32, mm as u32, ss as u32)
}

fn unix_secs(t: SystemTime) -> i64 {
    match t.duration_since(SystemTime::UNIX_EPOCH) {
        Ok(d) => d.as_secs() as i64,
        Err(e) => -(e.duration().as_secs() as i64),
    }
}

pub fn iso8601_utc(t: SystemTime) -> String {
    let (y, mo, d, h, mi, s) = civil_from_unix(unix_secs(t));
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}Z")
}

pub fn stamp_utc(t: SystemTime) -> String {
    let (y, mo, d, h, mi, s) = civil_from_unix(unix_secs(t));
    format!("{y:04}{mo:02}{d:02}_{h:02}{mi:02}{s:02}")
}
