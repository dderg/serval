use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{LazyLock, Mutex};
use std::time::Instant;

use crate::lock_ext::LockExt;

const TRACE_CAPACITY: usize = 4096;
const FAULT_TRACE_RECORDS: u64 = 64;
const TRANSPORT_ERROR_RESULT: i32 = i32::MIN;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct TransitTraceRecord {
    pub(super) sequence: u64,
    pub(super) mcu_id: u32,
    pub(super) axis: u8,
    pub(super) piece_count: u32,
    pub(super) room: u32,
    pub(super) guard_recorded_ns: u64,
    pub(super) guard_mcu_clock: u64,
    pub(super) send_started_ns: u64,
    pub(super) send_elapsed_ns: u64,
    pub(super) host_front_start_time: u64,
    pub(super) result: i32,
}

static TRACE_EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);
static NEXT_SEQUENCE: AtomicU64 = AtomicU64::new(0);
static TRACE: Mutex<VecDeque<TransitTraceRecord>> = Mutex::new(VecDeque::new());
static EMITTED_RESULTS: Mutex<[i32; 16]> = Mutex::new([i32::MAX; 16]);

pub(super) fn trace_now_ns() -> u64 {
    TRACE_EPOCH.elapsed().as_nanos() as u64
}

pub(super) fn record(mut record: TransitTraceRecord) {
    record.sequence = NEXT_SEQUENCE.fetch_add(1, Ordering::Relaxed);
    let mut trace = TRACE.lock_ok();
    if trace.len() == TRACE_CAPACITY {
        trace.pop_front();
    }
    trace.push_back(record);
}

pub(super) fn snapshot_last(limit: u64) -> Vec<TransitTraceRecord> {
    let trace = TRACE.lock_ok();
    let start = trace.len().saturating_sub(limit as usize);
    trace.range(start..).copied().collect()
}

pub(super) fn dump_last_to_stderr(limit: u64) {
    for record in snapshot_last(limit) {
        eprintln!("pump-transit: {record:?}");
    }
    let _ = std::io::Write::flush(&mut std::io::stderr());
}

pub(super) fn transport_error_result() -> i32 {
    TRANSPORT_ERROR_RESULT
}

pub(super) fn emit_result_fault_snapshot(trigger: &'static str, result: i32) {
    let mut emitted = EMITTED_RESULTS.lock_ok();
    if emitted.contains(&result) {
        return;
    }
    let Some(slot) = emitted.iter_mut().find(|slot| **slot == i32::MAX) else {
        return;
    };
    *slot = result;
    drop(emitted);
    emit_fault_snapshot(trigger, result);
}

pub fn emit_fault_snapshot(trigger: &'static str, result: i32) {
    let records = snapshot_last(FAULT_TRACE_RECORDS);
    tracing::error!(
        subsystem = "motion",
        event = "transit_fault_trace",
        trigger,
        fault_result = result,
        records = ?records,
        "pump transit trace captured after fault"
    );
}
