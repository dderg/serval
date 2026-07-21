//! Periodic whole-process CPU attribution, emitted through the structured
//! log pipeline. One summary event per interval plus one event per busy
//! thread — enough to tell whether starvation stutters coincide with the
//! python GIL thread, a specific pipeline stage, or overall CPU exhaustion.

use std::time::{Duration, Instant};

use host_rt::cpu_sampler::{ProcessCpu, sample_process};

const SAMPLE_INTERVAL: Duration = Duration::from_secs(5);
const BUSY_THREAD_FLOOR_PCT: f64 = 2.0;

pub(crate) fn spawn(pump_queue: crossbeam_channel::Sender<crate::pump::EnqueueMsg>) {
    let spawned = std::thread::Builder::new()
        .name("cpu-watch".into())
        .spawn(move || run(&pump_queue));
    if spawned.is_err() {
        tracing::warn!(
            subsystem = "motion",
            event = "cpu_watch_unavailable",
            "failed to spawn the cpu-watch sampler thread"
        );
    }
}

fn run(pump_queue: &crossbeam_channel::Sender<crate::pump::EnqueueMsg>) {
    let Some(mut prev) = sample_process() else {
        tracing::info!(
            subsystem = "motion",
            event = "cpu_watch_unavailable",
            "per-thread CPU sampling not supported on this platform"
        );
        return;
    };
    let mut prev_at = Instant::now();
    loop {
        std::thread::sleep(SAMPLE_INTERVAL);
        let Some(cur) = sample_process() else { return };
        let now = Instant::now();
        emit(&prev, &cur, now.duration_since(prev_at), pump_queue.len());
        prev = cur;
        prev_at = now;
    }
}

fn emit(prev: &ProcessCpu, cur: &ProcessCpu, elapsed: Duration, pump_queue_len: usize) {
    let interval_ticks = elapsed.as_secs_f64() * cur.ticks_per_sec as f64;
    if interval_ticks <= 0.0 {
        return;
    }
    let mut total_pct = 0.0;
    for thread in &cur.threads {
        let prev_ticks = prev
            .threads
            .iter()
            .find(|p| p.tid == thread.tid)
            .map_or(0, |p| p.cpu_ticks);
        let delta = thread.cpu_ticks.saturating_sub(prev_ticks);
        let cpu_pct = delta as f64 / interval_ticks * 100.0;
        total_pct += cpu_pct;
        if cpu_pct >= BUSY_THREAD_FLOOR_PCT {
            tracing::info!(
                subsystem = "motion",
                event = "cpu_thread_sample",
                thread = thread.name.as_str(),
                tid = thread.tid,
                cpu_pct,
                "cpu-watch thread sample"
            );
        }
    }
    tracing::info!(
        subsystem = "motion",
        event = "cpu_process_sample",
        total_pct,
        rss_mb = cur.rss_bytes as f64 / (1024.0 * 1024.0),
        n_threads = cur.threads.len(),
        pump_queue_len,
        "cpu-watch process sample"
    );
}
