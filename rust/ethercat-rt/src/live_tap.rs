//! File-less live telemetry: a unix-socket tap the dashboard reads while a
//! human watches, independent of `Capture` — a live viewer must never
//! occupy the one file-capture slot a calibration sweep needs.
//!
//! Protocol per connection: one scap-v2 header line (fresh `started_utc`,
//! drive names `slot0..slotN` — logical motor names live in klippy config,
//! which this process never sees), then fixed-size capture records for as
//! long as the client stays connected. One client at a time; the next
//! connect is served after the current one goes away.
//!
//! The DC thread pushes through a bounded preallocated channel and never
//! blocks: when the tap thread or its client can't keep up, records are
//! dropped and counted, and the stream self-describes the gap through the
//! `cycle_index` jump — the viewer renders a gap instead of stale data
//! pretending to be live. A slow client is disconnected by write timeout
//! rather than allowed to stall the drain.

use std::io::Write;
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use crate::capture::{
    encode_record, header_json, CaptureConfig, CaptureDriveConfig, CaptureRecord,
};
use crate::clock::monotonic_ns;
use crate::thread_prio::demote_to_normal_scheduling;

/// One second of backlog at the 4 kHz DC cycle — a same-host reader polls
/// far more often, so hitting this means the reader is wedged and dropping
/// is the correct outcome.
pub const LIVE_TAP_RING_CAPACITY: usize = 4096;
const CLIENT_WRITE_TIMEOUT: Duration = Duration::from_millis(200);
const RECV_TIMEOUT: Duration = Duration::from_millis(100);
const TAP_THREAD_STACK: usize = 512 * 1024;

/// One tap drive per slave slot, named `slot<N>` — logical motor names
/// live in klippy config, which this process never sees; the dashboard
/// maps them via drive_state.json's `slots` object.
pub fn slot_configs(
    counts_per_mm: &[f64],
    rotation_distance: &[f64],
    invert: &[bool],
) -> Vec<CaptureDriveConfig> {
    assert_eq!(counts_per_mm.len(), rotation_distance.len());
    assert_eq!(counts_per_mm.len(), invert.len());
    (0..counts_per_mm.len())
        .map(|slot| CaptureDriveConfig {
            slot: slot as u8,
            name: format!("slot{slot}"),
            counts_per_mm: counts_per_mm[slot],
            rotation_distance: rotation_distance[slot],
            invert: invert[slot],
        })
        .collect()
}

pub struct LiveTap {
    subscribed: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    tx: SyncSender<CaptureRecord>,
    service: Option<JoinHandle<()>>,
}

impl LiveTap {
    /// Bind and spawn before PREOP bringup: the thread spawn and the
    /// record-channel buffer prefault multiple milliseconds under
    /// mlockall(MCL_FUTURE), which would trip the drives' sync watchdog if
    /// it happened while the DC loop is pumping.
    pub fn spawn(
        socket_path: &str,
        drives: Vec<CaptureDriveConfig>,
        cycle_ns: i64,
    ) -> std::io::Result<Self> {
        let _ = std::fs::remove_file(socket_path);
        let listener = UnixListener::bind(socket_path)?;
        listener.set_nonblocking(true)?;
        // 0o666: the endpoint runs as root; the dashboard server does not.
        std::fs::set_permissions(socket_path, std::fs::Permissions::from_mode(0o666))?;
        let subscribed = Arc::new(AtomicBool::new(false));
        let dropped = Arc::new(AtomicU64::new(0));
        let (tx, rx) = sync_channel(LIVE_TAP_RING_CAPACITY);
        let thread_subscribed = Arc::clone(&subscribed);
        let thread_dropped = Arc::clone(&dropped);
        let service = std::thread::Builder::new()
            .name("live-tap".into())
            .stack_size(TAP_THREAD_STACK)
            .spawn(move || {
                service_loop(
                    &listener,
                    &rx,
                    &thread_subscribed,
                    &thread_dropped,
                    &drives,
                    cycle_ns,
                );
            })?;
        Ok(Self {
            subscribed,
            dropped,
            tx,
            service: Some(service),
        })
    }

    pub fn has_subscriber(&self) -> bool {
        self.subscribed.load(Ordering::Relaxed)
    }

    /// DC-thread side: never blocks, never allocates. Full channel means
    /// the reader is behind; the record is dropped and the gap shows up in
    /// the client's `cycle_index` sequence.
    pub fn push(&self, record: CaptureRecord) {
        match self.tx.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

impl Drop for LiveTap {
    fn drop(&mut self) {
        let (sink, _) = sync_channel(1);
        let _ = std::mem::replace(&mut self.tx, sink);
        if let Some(service) = self.service.take() {
            let _ = service.join();
        }
    }
}

fn utc_now() -> String {
    let format =
        time::macros::format_description!("[year]-[month]-[day]T[hour]:[minute]:[second]Z");
    time::OffsetDateTime::now_utc()
        .format(format)
        .expect("fixed format never fails")
}

enum SessionEnd {
    ClientGone,
    Shutdown,
}

fn service_loop(
    listener: &UnixListener,
    rx: &Receiver<CaptureRecord>,
    subscribed: &AtomicBool,
    dropped: &AtomicU64,
    drives: &[CaptureDriveConfig],
    cycle_ns: i64,
) {
    demote_to_normal_scheduling();
    loop {
        match listener.accept() {
            Ok((stream, _)) => {
                if let SessionEnd::Shutdown =
                    serve_client(stream, rx, subscribed, dropped, drives, cycle_ns)
                {
                    return;
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                match rx.recv_timeout(RECV_TIMEOUT) {
                    Ok(_) | Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            }
            Err(e) => {
                tracing::error!(
                    subsystem = "ethercat",
                    event = "live_tap_accept_error",
                    error = %e,
                    "live tap listener failed; tap disabled until restart"
                );
                return;
            }
        }
    }
}

fn serve_client(
    stream: UnixStream,
    rx: &Receiver<CaptureRecord>,
    subscribed: &AtomicBool,
    dropped: &AtomicU64,
    drives: &[CaptureDriveConfig],
    cycle_ns: i64,
) -> SessionEnd {
    let mut stream = stream;
    let _ = stream.set_write_timeout(Some(CLIENT_WRITE_TIMEOUT));
    let header = header_json(&CaptureConfig {
        path: String::new(),
        started_utc: utc_now(),
        drives: drives.to_vec(),
        cycle_ns,
        started_mono_ns: monotonic_ns(),
    });
    if stream.write_all(header.as_bytes()).is_err() {
        return SessionEnd::ClientGone;
    }
    while rx.try_recv().is_ok() {}
    let dropped_before = dropped.load(Ordering::Relaxed);
    subscribed.store(true, Ordering::Release);
    tracing::info!(
        subsystem = "ethercat",
        event = "live_tap_client_connected",
        "live tap streaming"
    );
    let mut sent = 0u64;
    let end = loop {
        match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(record) => {
                let (buf, size) = encode_record(&record);
                if stream.write_all(&buf[..size]).is_err() {
                    break SessionEnd::ClientGone;
                }
                sent += 1;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break SessionEnd::Shutdown,
        }
    };
    subscribed.store(false, Ordering::Release);
    tracing::info!(
        subsystem = "ethercat",
        event = "live_tap_client_disconnected",
        records_sent = sent,
        records_dropped = dropped.load(Ordering::Relaxed) - dropped_before,
        "live tap client gone"
    );
    end
}

#[cfg(test)]
mod tests;
