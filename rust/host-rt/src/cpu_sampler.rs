#![allow(unsafe_code)]

//! Per-thread CPU sampling for the whole host process via
//! `/proc/self/task/*/stat`. The klippy python interpreter and every Rust
//! pipeline thread live in one process, so sampling here attributes CPU to
//! the GIL thread and each named stage thread in a single view.

#[derive(Debug, Clone)]
pub struct ThreadCpu {
    pub tid: i32,
    pub name: String,
    pub cpu_ticks: u64,
}

#[derive(Debug, Clone)]
pub struct ProcessCpu {
    pub threads: Vec<ThreadCpu>,
    pub rss_bytes: u64,
    pub ticks_per_sec: u64,
}

#[cfg(not(target_os = "linux"))]
pub fn sample_process() -> Option<ProcessCpu> {
    None
}

#[cfg(target_os = "linux")]
pub fn sample_process() -> Option<ProcessCpu> {
    let ticks_per_sec = u64::try_from(unsafe { libc::sysconf(libc::_SC_CLK_TCK) }).ok()?;
    let page_size = u64::try_from(unsafe { libc::sysconf(libc::_SC_PAGESIZE) }).ok()?;
    let mut threads = Vec::new();
    for entry in std::fs::read_dir("/proc/self/task").ok()? {
        let entry = entry.ok()?;
        let tid: i32 = match entry.file_name().to_string_lossy().parse() {
            Ok(tid) => tid,
            Err(_) => continue,
        };
        let stat = match std::fs::read_to_string(entry.path().join("stat")) {
            Ok(stat) => stat,
            Err(_) => continue,
        };
        if let Some((name, cpu_ticks)) = parse_stat_line(&stat) {
            threads.push(ThreadCpu {
                tid,
                name,
                cpu_ticks,
            });
        }
    }
    let statm = std::fs::read_to_string("/proc/self/statm").ok()?;
    let rss_pages: u64 = statm.split_whitespace().nth(1)?.parse().ok()?;
    Some(ProcessCpu {
        threads,
        rss_bytes: rss_pages * page_size,
        ticks_per_sec,
    })
}

/// `stat` format: `pid (comm) state ppid ...` — comm may itself contain
/// spaces and parens, so the comm field ends at the *last* `)`. Returns the
/// thread name and utime+stime in clock ticks (fields 14 and 15).
pub fn parse_stat_line(stat: &str) -> Option<(String, u64)> {
    let open = stat.find('(')?;
    let close = stat.rfind(')')?;
    let name = stat.get(open + 1..close)?.to_string();
    let mut rest = stat.get(close + 1..)?.split_whitespace();
    let utime: u64 = rest.nth(11)?.parse().ok()?;
    let stime: u64 = rest.next()?.parse().ok()?;
    Some((name, utime + stime))
}

#[cfg(test)]
mod cpu_sampler_tests;
