//! Per-stage CPU telemetry: samples `/proc/self/task/*/stat` for the named
//! pipeline threads and logs each stage's CPU share once per second. Runs off
//! the hot path entirely — the sampler is its own thread and the stages are
//! never touched. Exits when the pipeline's `CommittedFrontier` is dropped.

use std::sync::Weak;
use std::time::Duration;

use super::CommittedFrontier;

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
const SAMPLE_PERIOD: Duration = Duration::from_secs(1);
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
const STAGE_PREFIXES: [&str; 6] = [
    "kalico-fit",
    "kalico-plan",
    "kalico-lower",
    "kalico-shape",
    "kalico-dispat",
    "push-pieces-pu",
];

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(super) fn parse_stat_comm_and_cpu_ticks(stat: &str) -> Result<(String, u64), String> {
    let open = stat
        .find('(')
        .ok_or_else(|| format!("no '(' in thread stat {stat:?}"))?;
    let (head, after_comm) = stat
        .rsplit_once(')')
        .ok_or_else(|| format!("no ')' after comm in thread stat {stat:?}"))?;
    let comm = head
        .get(open + 1..)
        .ok_or_else(|| format!("empty comm in thread stat {stat:?}"))?;
    let mut fields = after_comm.split_ascii_whitespace();
    const UTIME_INDEX_AFTER_COMM: usize = 11;
    let utime: u64 = fields
        .nth(UTIME_INDEX_AFTER_COMM)
        .ok_or_else(|| format!("thread stat has too few fields for utime: {stat:?}"))?
        .parse()
        .map_err(|e| format!("utime is not a u64 ({e}) in thread stat {stat:?}"))?;
    let stime: u64 = fields
        .next()
        .ok_or_else(|| format!("thread stat has too few fields for stime: {stat:?}"))?
        .parse()
        .map_err(|e| format!("stime is not a u64 ({e}) in thread stat {stat:?}"))?;
    Ok((comm.to_string(), utime + stime))
}

#[cfg(target_os = "linux")]
pub(super) fn spawn_sampler(frontier: Weak<CommittedFrontier>) {
    std::thread::Builder::new()
        .name("kalico-stage-cpu".into())
        .spawn(move || run(&frontier))
        .expect("spawn kalico-stage-cpu thread");
}

#[cfg(not(target_os = "linux"))]
pub(super) fn spawn_sampler(_frontier: Weak<CommittedFrontier>) {}

#[cfg(target_os = "linux")]
fn run(frontier: &Weak<CommittedFrontier>) {
    const USER_HZ: u64 = 100;
    let mut previous: std::collections::HashMap<String, u64> = std::collections::HashMap::new();
    while frontier.upgrade().is_some() {
        std::thread::sleep(SAMPLE_PERIOD);
        let Ok(entries) = std::fs::read_dir("/proc/self/task") else {
            return;
        };
        for entry in entries.flatten() {
            let Ok(stat) = std::fs::read_to_string(entry.path().join("stat")) else {
                continue;
            };
            let Ok((comm, ticks)) = parse_stat_comm_and_cpu_ticks(&stat) else {
                continue;
            };
            if !STAGE_PREFIXES.iter().any(|p| comm.starts_with(p)) {
                continue;
            }
            if let Some(prev) = previous.insert(comm.clone(), ticks) {
                let busy_ms = (ticks.saturating_sub(prev)) * 1000 / USER_HZ;
                let busy_pct = busy_ms as f64 / SAMPLE_PERIOD.as_millis() as f64 * 100.0;
                tracing::info!(
                    subsystem = "motion",
                    event = "stage_cpu",
                    stage = %comm,
                    busy_ms,
                    busy_pct,
                    "pipeline stage CPU over the last sample period"
                );
            }
        }
    }
}

#[cfg(test)]
#[path = "stage_cpu_tests.rs"]
mod stage_cpu_tests;
