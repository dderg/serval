use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{channel, sync_channel, Receiver, Sender, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::cli::EC_RT_MAX_SLAVES;
use crate::thread_prio::demote_to_normal_scheduling;

pub const ERR_CAPTURE_ACTIVE: i32 = -320;
pub const ERR_CAPTURE_NOT_ACTIVE: i32 = -321;
pub const ERR_CAPTURE_FILE: i32 = -322;
pub const ERR_CAPTURE_OVERFLOW: i32 = -323;
pub const ERR_CAPTURE_BAD_ARG: i32 = -324;
pub const ERR_CAPTURE_BAD_DRIVE_LIST: i32 = -325;

pub const CAPTURE_RING_CAPACITY: usize = 4096;

pub const MAX_DRIVES: usize = EC_RT_MAX_SLAVES;
pub const RECORD_PREFIX_SIZE: usize = 9;
pub const DRIVE_BLOCK_SIZE: usize = 28;
pub const MAX_RECORD_SIZE: usize = RECORD_PREFIX_SIZE + MAX_DRIVES * DRIVE_BLOCK_SIZE;

#[must_use]
pub const fn record_size(drive_count: usize) -> usize {
    RECORD_PREFIX_SIZE + drive_count * DRIVE_BLOCK_SIZE
}

pub const FLAG_TORQUE_ENABLED: u8 = 1 << 0;
pub const FLAG_MOTION_ACTIVE: u8 = 1 << 1;

const OFF_CYCLE_INDEX: usize = 0;
const OFF_FLAGS: usize = 8;

const DOFF_TARGET_COUNTS: usize = 0;
const DOFF_POSITION_ACTUAL: usize = 4;
const DOFF_FOLLOWING_ERROR: usize = 8;
const DOFF_TORQUE_ACTUAL: usize = 12;
const DOFF_STATUSWORD: usize = 14;
const DOFF_ERROR_CODE: usize = 16;
const DOFF_VELOCITY_OFFSET: usize = 18;
const DOFF_TORQUE_OFFSET: usize = 22;
const DOFF_VELOCITY_ACTUAL: usize = 24;

const WRITER_SYNC_INTERVAL: Duration = Duration::from_secs(1);
const WRITER_RECV_TIMEOUT: Duration = Duration::from_millis(100);
const IO_THREAD_STACK: usize = 512 * 1024;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DriveSample {
    pub target_counts: i32,
    pub position_actual: i32,
    pub velocity_actual: i32,
    pub following_error: i32,
    pub torque_actual: i16,
    pub statusword: u16,
    pub error_code: u16,
    pub velocity_offset: i32,
    pub torque_offset: i16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureRecord {
    pub cycle_index: u64,
    pub flags: u8,
    pub drive_count: u8,
    pub drives: [DriveSample; MAX_DRIVES],
}

impl CaptureRecord {
    #[must_use]
    pub fn new(cycle_index: u64, flags: u8) -> Self {
        Self {
            cycle_index,
            flags,
            drive_count: 0,
            drives: [DriveSample::default(); MAX_DRIVES],
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CaptureDriveConfig {
    pub slot: u8,
    pub name: String,
    pub counts_per_mm: f64,
    pub rotation_distance: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CaptureConfig {
    pub path: String,
    pub started_utc: String,
    pub drives: Vec<CaptureDriveConfig>,
    pub cycle_ns: i64,
    pub started_mono_ns: u64,
}

#[derive(Debug, PartialEq, Eq)]
pub struct StopOutcome {
    pub result: i32,
    pub samples: u64,
    pub overflow_cycle: Option<u64>,
}

fn encode_drive(block: &mut [u8], d: &DriveSample) {
    block[DOFF_TARGET_COUNTS..DOFF_TARGET_COUNTS + 4]
        .copy_from_slice(&d.target_counts.to_le_bytes());
    block[DOFF_POSITION_ACTUAL..DOFF_POSITION_ACTUAL + 4]
        .copy_from_slice(&d.position_actual.to_le_bytes());
    block[DOFF_FOLLOWING_ERROR..DOFF_FOLLOWING_ERROR + 4]
        .copy_from_slice(&d.following_error.to_le_bytes());
    block[DOFF_TORQUE_ACTUAL..DOFF_TORQUE_ACTUAL + 2]
        .copy_from_slice(&d.torque_actual.to_le_bytes());
    block[DOFF_STATUSWORD..DOFF_STATUSWORD + 2].copy_from_slice(&d.statusword.to_le_bytes());
    block[DOFF_ERROR_CODE..DOFF_ERROR_CODE + 2].copy_from_slice(&d.error_code.to_le_bytes());
    block[DOFF_VELOCITY_OFFSET..DOFF_VELOCITY_OFFSET + 4]
        .copy_from_slice(&d.velocity_offset.to_le_bytes());
    block[DOFF_TORQUE_OFFSET..DOFF_TORQUE_OFFSET + 2]
        .copy_from_slice(&d.torque_offset.to_le_bytes());
    block[DOFF_VELOCITY_ACTUAL..DOFF_VELOCITY_ACTUAL + 4]
        .copy_from_slice(&d.velocity_actual.to_le_bytes());
}

pub fn encode_record(r: &CaptureRecord) -> ([u8; MAX_RECORD_SIZE], usize) {
    let n = r.drive_count as usize;
    let size = record_size(n);
    let mut b = [0u8; MAX_RECORD_SIZE];
    b[OFF_CYCLE_INDEX..OFF_CYCLE_INDEX + 8].copy_from_slice(&r.cycle_index.to_le_bytes());
    b[OFF_FLAGS] = r.flags;
    for (d, drive) in r.drives[..n].iter().enumerate() {
        let base = RECORD_PREFIX_SIZE + d * DRIVE_BLOCK_SIZE;
        encode_drive(&mut b[base..base + DRIVE_BLOCK_SIZE], drive);
    }
    (b, size)
}

/// Slot-agnostic drive-list checks the I/O thread can run without the slave
/// count: non-empty, no more than `MAX_DRIVES`, no duplicate slot. The endpoint
/// adds the range check (`slot < num_slaves`) where it knows the topology.
pub fn validate_drive_list(drives: &[CaptureDriveConfig]) -> Result<(), i32> {
    if drives.is_empty() || drives.len() > MAX_DRIVES {
        return Err(ERR_CAPTURE_BAD_DRIVE_LIST);
    }
    for (i, d) in drives.iter().enumerate() {
        if drives[..i].iter().any(|prev| prev.slot == d.slot) {
            return Err(ERR_CAPTURE_BAD_DRIVE_LIST);
        }
    }
    Ok(())
}

pub fn any_slot_out_of_range(slots: &[u8], num_slaves: usize) -> bool {
    slots.iter().any(|&slot| usize::from(slot) >= num_slaves)
}

fn json_string_safe(s: &str) -> bool {
    s.chars()
        .all(|c| (c.is_ascii_graphic() || c == ' ') && c != '"' && c != '\\')
}

pub fn header_json(cfg: &CaptureConfig) -> String {
    let p = RECORD_PREFIX_SIZE;
    let mut channels = String::new();
    for (name, dtype, offset) in [
        ("cycle_index", "u64", OFF_CYCLE_INDEX),
        ("flags", "u8", OFF_FLAGS),
        ("target_counts", "i32", p + DOFF_TARGET_COUNTS),
        ("position_actual", "i32", p + DOFF_POSITION_ACTUAL),
        ("following_error", "i32", p + DOFF_FOLLOWING_ERROR),
        ("torque_actual", "i16", p + DOFF_TORQUE_ACTUAL),
        ("statusword", "u16", p + DOFF_STATUSWORD),
        ("error_code", "u16", p + DOFF_ERROR_CODE),
        ("velocity_offset", "i32", p + DOFF_VELOCITY_OFFSET),
        ("torque_offset", "i16", p + DOFF_TORQUE_OFFSET),
        ("velocity_actual", "i32", p + DOFF_VELOCITY_ACTUAL),
    ] {
        if !channels.is_empty() {
            channels.push(',');
        }
        channels.push_str(&format!(
            "{{\"name\":\"{name}\",\"dtype\":\"{dtype}\",\"offset\":{offset}}}"
        ));
    }
    let mut drives = String::new();
    for d in &cfg.drives {
        if !drives.is_empty() {
            drives.push(',');
        }
        drives.push_str(&format!(
            "{{\"name\":\"{}\",\"counts_per_mm\":{},\"rotation_distance\":{}}}",
            d.name, d.counts_per_mm, d.rotation_distance
        ));
    }
    format!(
        concat!(
            "{{\"version\":1,\"cycle_ns\":{},\"record_size\":{},",
            "\"started_utc\":\"{}\",\"started_mono_ns\":{},",
            "\"drives\":[{}],",
            "\"channels\":[{}]}}\n",
        ),
        cfg.cycle_ns,
        record_size(cfg.drives.len()),
        cfg.started_utc,
        cfg.started_mono_ns,
        drives,
        channels,
    )
}

enum WriterHook {
    None,
    Gate(Receiver<()>),
    #[cfg(test)]
    FailAfterHeader(SyncSender<()>),
}

enum IoMsg {
    Open {
        cfg: CaptureConfig,
        hook: WriterHook,
        records: Receiver<CaptureRecord>,
        reply: SyncSender<i32>,
    },
    Finalize {
        failure: Option<(u64, i32)>,
        reply: SyncSender<StopOutcome>,
    },
}

struct ActiveCapture {
    tx: SyncSender<CaptureRecord>,
    failure: Option<(u64, i32)>,
}

/// All file I/O runs on one persistent `capture-io` thread, spawned once.
/// The DC thread's start/push/stop are channel operations only: file opens,
/// writes, fsync, and rename stall for SD-card eternities (100+ ms), and a
/// pause in cyclic frames beyond ~3 ms trips the drive's sync-error counter
/// (ErC1.1 / AL 0x001a). Thread creation is banned on the DC thread for the
/// same reason — under mlockall(MCL_FUTURE) a pthread spawn prefaults and
/// locks the new stack, which is milliseconds by itself.
pub struct Capture {
    capacity: usize,
    control: Sender<IoMsg>,
    service: Option<JoinHandle<()>>,
    active: Option<ActiveCapture>,
}

impl Default for Capture {
    fn default() -> Self {
        Self::new()
    }
}

impl Capture {
    pub fn new() -> Self {
        Self::with_capacity(CAPTURE_RING_CAPACITY)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        let (control, control_rx) = channel();
        let service = std::thread::Builder::new()
            .name("capture-io".into())
            .stack_size(IO_THREAD_STACK)
            .spawn(move || service_loop(control_rx))
            .expect("spawn capture-io thread");
        Self {
            capacity,
            control,
            service: Some(service),
            active: None,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }

    /// Blocking start — for the stub endpoint and tests, where there is no
    /// realtime cycle to starve.
    pub fn start(&mut self, cfg: CaptureConfig) -> i32 {
        let pending = self.start_inner(cfg, WriterHook::None);
        let claimed = pending.claimed();
        let rc = pending.wait();
        if rc != 0 && claimed {
            self.clear_failed_start();
        }
        rc
    }

    /// Non-blocking start: the file open happens on the capture-io thread
    /// and the result arrives through the handle. Records pushed before the
    /// open resolves buffer in the ring. If the handle yields a non-zero rc
    /// the caller must invoke `clear_failed_start`.
    pub fn start_async(&mut self, cfg: CaptureConfig) -> PendingStart {
        self.start_inner(cfg, WriterHook::None)
    }

    #[cfg(test)]
    pub(crate) fn start_gated(&mut self, cfg: CaptureConfig, gate: Receiver<()>) -> i32 {
        let pending = self.start_inner(cfg, WriterHook::Gate(gate));
        let claimed = pending.claimed();
        let rc = pending.wait();
        if rc != 0 && claimed {
            self.clear_failed_start();
        }
        rc
    }

    #[cfg(test)]
    pub(crate) fn start_writer_fails(&mut self, cfg: CaptureConfig) -> (i32, Receiver<()>) {
        let (done_tx, done_rx) = sync_channel(1);
        let pending = self.start_inner(cfg, WriterHook::FailAfterHeader(done_tx));
        let claimed = pending.claimed();
        let rc = pending.wait();
        if rc != 0 && claimed {
            self.clear_failed_start();
        }
        (rc, done_rx)
    }

    fn start_inner(&mut self, cfg: CaptureConfig, hook: WriterHook) -> PendingStart {
        let immediate = |rc: i32| {
            let (tx, rx) = sync_channel(1);
            let _ = tx.send(rc);
            PendingStart { rx, claimed: false }
        };
        if self.active.is_some() {
            return immediate(ERR_CAPTURE_ACTIVE);
        }
        if !json_string_safe(&cfg.started_utc)
            || cfg.drives.iter().any(|d| !json_string_safe(&d.name))
        {
            return immediate(ERR_CAPTURE_BAD_ARG);
        }
        if let Err(rc) = validate_drive_list(&cfg.drives) {
            return immediate(rc);
        }
        let (tx, records) = sync_channel(self.capacity);
        let (reply, reply_rx) = sync_channel(1);
        self.control
            .send(IoMsg::Open {
                cfg,
                hook,
                records,
                reply,
            })
            .expect("capture-io thread is gone");
        self.active = Some(ActiveCapture { tx, failure: None });
        PendingStart {
            rx: reply_rx,
            claimed: true,
        }
    }

    /// Discard the active slot after `PendingStart` resolved non-zero. The
    /// service already abandoned the session; no Finalize is owed.
    pub fn clear_failed_start(&mut self) {
        self.active = None;
    }

    pub fn push(&mut self, record: CaptureRecord) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if active.failure.is_some() {
            return;
        }
        match active.tx.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_OVERFLOW));
            }
            Err(TrySendError::Disconnected(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_FILE));
            }
        }
    }

    /// Blocking stop — for the stub endpoint and tests.
    pub fn stop(&mut self) -> StopOutcome {
        self.stop_async().wait()
    }

    /// Non-blocking stop: drops the record channel and asks the capture-io
    /// thread to drain, fsync, and (on failure) rename. Poll the handle from
    /// the cycle; the outcome arrives when the file is durable.
    pub fn stop_async(&mut self) -> PendingStop {
        let (tx, rx) = sync_channel(1);
        let Some(active) = self.active.take() else {
            let _ = tx.send(StopOutcome {
                result: ERR_CAPTURE_NOT_ACTIVE,
                samples: 0,
                overflow_cycle: None,
            });
            return PendingStop { rx };
        };
        drop(active.tx);
        self.control
            .send(IoMsg::Finalize {
                failure: active.failure,
                reply: tx,
            })
            .expect("capture-io thread is gone");
        PendingStop { rx }
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        self.active = None;
        let (sink, _) = channel();
        let _ = std::mem::replace(&mut self.control, sink);
        if let Some(service) = self.service.take() {
            let _ = service.join();
        }
    }
}

pub struct PendingStart {
    rx: Receiver<i32>,
    claimed: bool,
}

impl PendingStart {
    pub fn try_take(&self) -> Option<i32> {
        self.rx.try_recv().ok()
    }

    pub fn wait(self) -> i32 {
        self.rx.recv().expect("capture-io thread died")
    }

    /// Whether this start took the active slot. Rejections (already active,
    /// bad arguments) never claim it — `clear_failed_start` after such a
    /// failure would kill the live capture instead.
    pub fn claimed(&self) -> bool {
        self.claimed
    }
}

pub struct PendingStop {
    rx: Receiver<StopOutcome>,
}

impl PendingStop {
    pub fn try_take(&self) -> Option<StopOutcome> {
        self.rx.try_recv().ok()
    }

    pub fn wait(self) -> StopOutcome {
        self.rx.recv().expect("capture-io thread died")
    }
}

fn service_loop(control: Receiver<IoMsg>) {
    demote_to_normal_scheduling();
    while let Ok(msg) = control.recv() {
        match msg {
            IoMsg::Finalize { reply, .. } => {
                let _ = reply.send(StopOutcome {
                    result: ERR_CAPTURE_NOT_ACTIVE,
                    samples: 0,
                    overflow_cycle: None,
                });
            }
            IoMsg::Open {
                cfg,
                hook,
                records,
                reply,
            } => {
                let path = PathBuf::from(&cfg.path);
                let session = match open_session(&path) {
                    Ok(file) => {
                        let _ = reply.send(0);
                        Some(run_session(file, header_json(&cfg), hook, records))
                    }
                    Err(rc) => {
                        let _ = reply.send(rc);
                        None
                    }
                };
                let Some(written) = session else {
                    continue;
                };
                match control.recv() {
                    Ok(IoMsg::Finalize { failure, reply }) => {
                        let _ = reply.send(compose_outcome(&path, written, failure));
                    }
                    Ok(IoMsg::Open { reply, .. }) => {
                        let _ = reply.send(ERR_CAPTURE_ACTIVE);
                    }
                    Err(_) => return,
                }
            }
        }
    }
}

fn open_session(path: &PathBuf) -> Result<File, i32> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() && std::fs::create_dir_all(parent).is_err() {
            return Err(ERR_CAPTURE_FILE);
        }
    }
    File::create(path).map_err(|_| ERR_CAPTURE_FILE)
}

fn run_session(
    mut file: File,
    header: String,
    hook: WriterHook,
    rx: Receiver<CaptureRecord>,
) -> Result<u64, (u64, String)> {
    file.write_all(header.as_bytes())
        .map_err(|e| (0u64, format!("capture header write: {e}")))?;
    match hook {
        WriterHook::None => {}
        WriterHook::Gate(g) => {
            let _ = g.recv();
        }
        #[cfg(test)]
        WriterHook::FailAfterHeader(done_tx) => {
            let _ = done_tx.send(());
            return Err((0, "injected".into()));
        }
    }
    let mut written = 0u64;
    let mut last_sync = Instant::now();
    loop {
        match rx.recv_timeout(WRITER_RECV_TIMEOUT) {
            Ok(r) => {
                let (buf, size) = encode_record(&r);
                file.write_all(&buf[..size])
                    .map_err(|e| (written, format!("capture record write: {e}")))?;
                written += 1;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if last_sync.elapsed() >= WRITER_SYNC_INTERVAL {
            file.sync_data()
                .map_err(|e| (written, format!("capture fsync: {e}")))?;
            last_sync = Instant::now();
        }
    }
    file.sync_data()
        .map_err(|e| (written, format!("capture final fsync: {e}")))?;
    Ok(written)
}

fn compose_outcome(
    path: &PathBuf,
    written: Result<u64, (u64, String)>,
    failure: Option<(u64, i32)>,
) -> StopOutcome {
    let (mut result, mut overflow_cycle) = (0i32, None);
    if let Some((cycle, code)) = failure {
        result = code;
        overflow_cycle = Some(cycle);
    }
    let samples = match written {
        Ok(n) => n,
        Err((n, _)) if result == 0 => {
            result = ERR_CAPTURE_FILE;
            n
        }
        Err((n, _)) => n,
    };
    if result != 0 {
        let failed = path.with_extension("failed.scap");
        if std::fs::rename(path, &failed).is_err() {
            result = ERR_CAPTURE_FILE;
        }
    }
    StopOutcome {
        result,
        samples,
        overflow_cycle,
    }
}

#[cfg(test)]
mod tests;
