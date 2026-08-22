use std::sync::LazyLock;
use std::sync::Mutex;
use std::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::time::Instant;

const TRACE_CAPACITY: usize = 4096;
const FAULT_TRACE_RECORDS: u64 = 64;
const INVALID_SEQUENCE: u64 = u64::MAX;
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

struct TransitTraceSlot {
    committed_sequence: AtomicU64,
    mcu_id: AtomicU32,
    axis: AtomicU8,
    piece_count: AtomicU32,
    room: AtomicU32,
    guard_recorded_ns: AtomicU64,
    guard_mcu_clock: AtomicU64,
    send_started_ns: AtomicU64,
    send_elapsed_ns: AtomicU64,
    host_front_start_time: AtomicU64,
    result: AtomicI32,
}

impl TransitTraceSlot {
    const fn new() -> Self {
        Self {
            committed_sequence: AtomicU64::new(INVALID_SEQUENCE),
            mcu_id: AtomicU32::new(0),
            axis: AtomicU8::new(0),
            piece_count: AtomicU32::new(0),
            room: AtomicU32::new(0),
            guard_recorded_ns: AtomicU64::new(0),
            guard_mcu_clock: AtomicU64::new(0),
            send_started_ns: AtomicU64::new(0),
            send_elapsed_ns: AtomicU64::new(0),
            host_front_start_time: AtomicU64::new(0),
            result: AtomicI32::new(0),
        }
    }

    fn write(&self, sequence: u64, record: TransitTraceRecord) {
        self.committed_sequence
            .store(INVALID_SEQUENCE, Ordering::Release);
        self.mcu_id.store(record.mcu_id, Ordering::Relaxed);
        self.axis.store(record.axis, Ordering::Relaxed);
        self.piece_count
            .store(record.piece_count, Ordering::Relaxed);
        self.room.store(record.room, Ordering::Relaxed);
        self.guard_recorded_ns
            .store(record.guard_recorded_ns, Ordering::Relaxed);
        self.guard_mcu_clock
            .store(record.guard_mcu_clock, Ordering::Relaxed);
        self.send_started_ns
            .store(record.send_started_ns, Ordering::Relaxed);
        self.send_elapsed_ns
            .store(record.send_elapsed_ns, Ordering::Relaxed);
        self.host_front_start_time
            .store(record.host_front_start_time, Ordering::Relaxed);
        self.result.store(record.result, Ordering::Relaxed);
        self.committed_sequence.store(sequence, Ordering::Release);
    }

    fn read(&self, sequence: u64) -> Option<TransitTraceRecord> {
        if self.committed_sequence.load(Ordering::Acquire) != sequence {
            return None;
        }
        let record = TransitTraceRecord {
            sequence,
            mcu_id: self.mcu_id.load(Ordering::Relaxed),
            axis: self.axis.load(Ordering::Relaxed),
            piece_count: self.piece_count.load(Ordering::Relaxed),
            room: self.room.load(Ordering::Relaxed),
            guard_recorded_ns: self.guard_recorded_ns.load(Ordering::Relaxed),
            guard_mcu_clock: self.guard_mcu_clock.load(Ordering::Relaxed),
            send_started_ns: self.send_started_ns.load(Ordering::Relaxed),
            send_elapsed_ns: self.send_elapsed_ns.load(Ordering::Relaxed),
            host_front_start_time: self.host_front_start_time.load(Ordering::Relaxed),
            result: self.result.load(Ordering::Relaxed),
        };
        (self.committed_sequence.load(Ordering::Acquire) == sequence).then_some(record)
    }
}

static TRACE_EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);
static NEXT_SEQUENCE: AtomicU64 = AtomicU64::new(0);
static TRACE_SLOTS: [TransitTraceSlot; TRACE_CAPACITY] =
    [const { TransitTraceSlot::new() }; TRACE_CAPACITY];
static EMITTED_RESULTS: Mutex<[i32; 16]> = Mutex::new([i32::MAX; 16]);

pub(super) fn trace_now_ns() -> u64 {
    TRACE_EPOCH.elapsed().as_nanos() as u64
}

pub(super) fn record(mut record: TransitTraceRecord) {
    let sequence = NEXT_SEQUENCE.fetch_add(1, Ordering::Relaxed);
    record.sequence = sequence;
    TRACE_SLOTS[sequence as usize % TRACE_CAPACITY].write(sequence, record);
}

pub(super) fn snapshot_last(limit: u64) -> Vec<TransitTraceRecord> {
    let end = NEXT_SEQUENCE.load(Ordering::Acquire);
    let start = end.saturating_sub(limit.min(TRACE_CAPACITY as u64));
    (start..end)
        .filter_map(|sequence| TRACE_SLOTS[sequence as usize % TRACE_CAPACITY].read(sequence))
        .collect()
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
    let mut emitted = EMITTED_RESULTS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
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
        "lock-free pump transit trace captured after fault"
    );
}
