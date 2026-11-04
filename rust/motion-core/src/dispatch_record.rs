//! Dev-only record of what the live host dispatched: every shaped segment
//! with the host-time origin it was anchored on, every drain to rest, and
//! every planner-bypassing nudge. The bench analyzer judges captured steps
//! against it instead of the offline plan, which a pacer brake invalidates.
//! Enabled by [`DISPATCH_RECORD_DIR_ENV`]; when unset nothing is built. A
//! writer that cannot keep up is fatal, never lossy.

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError, bounded};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;
use trajectory::ContinuousSegment;

pub const DISPATCH_RECORD_DIR_ENV: &str = "SERVAL_DISPATCH_RECORD_DIR";
pub const DISPATCH_RECORD_MAGIC: [u8; 8] = *b"SVDREC1\0";
pub const RECORDED_AXES: u32 = 4;
/// The largest spacing between two samples inside a recorded segment.
pub const SAMPLE_INTERVAL_S: f64 = 0.5e-3;
pub const HEADER_LEN: usize = 20;
pub const SEGMENT_FRESH_EPOCH: u8 = 1 << 0;
pub const SEGMENT_REST_AT_END: u8 = 1 << 1;
const CHANNEL_ENTRIES: usize = 1 << 16;
const IDLE_FLUSH: Duration = Duration::from_millis(100);

static SESSIONS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RecordKind {
    Segment = 1,
    Drain = 2,
    Nudge = 3,
}

/// Why the stream drained to rest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DrainCause {
    /// A G-code rest: a dwell, M400, a wait for moves, a jog or a stop.
    Requested = 0,
    /// The pacer braked a producer that fell behind the playhead.
    PacerBrake = 1,
}

pub fn encode_header(out: &mut Vec<u8>) {
    out.extend_from_slice(&DISPATCH_RECORD_MAGIC);
    out.extend_from_slice(&RECORDED_AXES.to_le_bytes());
    out.extend_from_slice(&SAMPLE_INTERVAL_S.to_le_bytes());
}

/// Samples a segment over `[t_start, t_end]` takes: both ends, no two
/// further apart than [`SAMPLE_INTERVAL_S`].
pub fn sample_count(t_start: f64, t_end: f64) -> Result<u32, String> {
    if !(t_start.is_finite() && t_end.is_finite() && t_end >= t_start) {
        return Err(format!(
            "segment span [{t_start}, {t_end}] is not a finite forward interval"
        ));
    }
    let intervals = checked_cast::f64_to_u32(((t_end - t_start) / SAMPLE_INTERVAL_S).ceil())
        .map_err(|e| format!("segment span [{t_start}, {t_end}] needs {e} samples"))?;
    intervals
        .checked_add(1)
        .map(|n| n.max(2))
        .ok_or_else(|| format!("segment span [{t_start}, {t_end}] overflows the sample count"))
}

/// One SEGMENT record: the segment anchored on host origin `t0`, sampled
/// per [`sample_count`] on its own carrier. On error `out` is left as it
/// was.
pub fn encode_segment(
    out: &mut Vec<u8>,
    seg: &ContinuousSegment,
    t0: f64,
    fresh_epoch: bool,
) -> Result<(), String> {
    let start = out.len();
    let encoded = append_segment(out, seg, t0, fresh_epoch);
    if encoded.is_err() {
        out.truncate(start);
    }
    encoded
}

fn append_segment(
    out: &mut Vec<u8>,
    seg: &ContinuousSegment,
    t0: f64,
    fresh_epoch: bool,
) -> Result<(), String> {
    let axes = RECORDED_AXES as usize;
    if seg.axes.len() < axes {
        return Err(format!(
            "segment at line {} carries {} axes; the record needs {axes}",
            seg.source_line,
            seg.axes.len()
        ));
    }
    let n = sample_count(seg.t_start, seg.t_end)?;
    let flags = if fresh_epoch { SEGMENT_FRESH_EPOCH } else { 0 }
        | if seg.rest_at_end {
            SEGMENT_REST_AT_END
        } else {
            0
        };
    out.reserve(1 + 3 * 8 + 4 + 1 + 4 + n as usize * axes * 3 * 8);
    out.push(RecordKind::Segment as u8);
    out.extend_from_slice(&t0.to_le_bytes());
    out.extend_from_slice(&seg.t_start.to_le_bytes());
    out.extend_from_slice(&seg.t_end.to_le_bytes());
    out.extend_from_slice(&seg.source_line.to_le_bytes());
    out.push(flags);
    out.extend_from_slice(&n.to_le_bytes());
    let last = n - 1;
    let span = seg.t_end - seg.t_start;
    for k in 0..n {
        let t = if k == last {
            seg.t_end
        } else {
            seg.t_start + span * f64::from(k) / f64::from(last)
        };
        for axis in 0..axes {
            let pva = seg.eval_axis(axis, t).map_err(|e| {
                format!("segment at line {} axis {axis} t={t}: {e}", seg.source_line)
            })?;
            out.extend_from_slice(&pva.position.to_le_bytes());
            out.extend_from_slice(&pva.velocity.to_le_bytes());
            out.extend_from_slice(&pva.acceleration.to_le_bytes());
        }
    }
    Ok(())
}

pub fn encode_drain(out: &mut Vec<u8>, cause: DrainCause, moves_sent: u64) {
    out.push(RecordKind::Drain as u8);
    out.push(cause as u8);
    out.extend_from_slice(&moves_sent.to_le_bytes());
}

pub fn encode_nudge(out: &mut Vec<u8>, nudge: &RecordedNudge) {
    out.push(RecordKind::Nudge as u8);
    out.extend_from_slice(&nudge.t0.to_le_bytes());
    out.extend_from_slice(&nudge.t_start.to_le_bytes());
    out.extend_from_slice(&nudge.t_end.to_le_bytes());
    out.extend_from_slice(&nudge.mcu_id.to_le_bytes());
    out.push(nudge.axis);
}

/// A planner-bypassing nudge as dispatched: anchored on host origin `t0`
/// over stream times `[t_start, t_end]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecordedNudge {
    pub t0: f64,
    pub t_start: f64,
    pub t_end: f64,
    pub mcu_id: u32,
    pub axis: u8,
}

enum Entry {
    Segment {
        seg: ContinuousSegment,
        t0: f64,
        fresh_epoch: bool,
    },
    Drain {
        cause: DrainCause,
        moves_sent: u64,
    },
    Nudge(RecordedNudge),
}

impl Entry {
    fn encode(&self, out: &mut Vec<u8>) -> Result<(), String> {
        match self {
            Self::Segment {
                seg,
                t0,
                fresh_epoch,
            } => encode_segment(out, seg, *t0, *fresh_epoch),
            Self::Drain { cause, moves_sent } => {
                encode_drain(out, *cause, *moves_sent);
                Ok(())
            }
            Self::Nudge(nudge) => {
                encode_nudge(out, nudge);
                Ok(())
            }
        }
    }
}

type Fault = Arc<Mutex<Option<String>>>;

/// The live dispatch record. The dispatching threads only hand entries to a
/// bounded channel; a writer thread samples, encodes and writes them.
pub struct DispatchRecorder {
    path: PathBuf,
    tx: Option<Sender<Entry>>,
    writer: Option<JoinHandle<Result<(), String>>>,
    fault: Fault,
}

impl DispatchRecorder {
    /// The recorder [`DISPATCH_RECORD_DIR_ENV`] asks for, `None` when unset.
    pub fn from_env() -> Option<Arc<Self>> {
        let dir = std::env::var_os(DISPATCH_RECORD_DIR_ENV)?;
        Some(
            Self::open(Path::new(&dir))
                .unwrap_or_else(|error| panic!("{DISPATCH_RECORD_DIR_ENV}: {error}")),
        )
    }

    pub fn open(dir: &Path) -> Result<Arc<Self>, String> {
        let session = SESSIONS.fetch_add(1, Ordering::Relaxed);
        let path = dir.join(format!(
            "dispatch_record_p{}_s{session}.bin",
            std::process::id()
        ));
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| format!("dispatch record: cannot create {}: {e}", path.display()))?;
        let mut file = BufWriter::new(file);
        let mut header = Vec::with_capacity(HEADER_LEN);
        encode_header(&mut header);
        file.write_all(&header)
            .map_err(|e| format!("dispatch record: cannot write {}: {e}", path.display()))?;
        let (tx, rx) = bounded(CHANNEL_ENTRIES);
        let mut recorder = Self::from_sender(tx, path);
        let fault = Arc::clone(&recorder.fault);
        recorder.writer = Some(
            std::thread::Builder::new()
                .name("dispatch-record".to_string())
                .spawn(move || {
                    let written = write_entries(&rx, file);
                    if let Err(error) = &written {
                        latch(&fault, error.clone());
                    }
                    written
                })
                .map_err(|e| format!("dispatch record: cannot spawn writer: {e}"))?,
        );
        tracing::info!(
            subsystem = "motion",
            event = "dispatch_record_open",
            path = %recorder.path.display(),
            "[dispatch-record] recording every dispatched segment and drain"
        );
        Ok(Arc::new(recorder))
    }

    fn from_sender(tx: Sender<Entry>, path: PathBuf) -> Self {
        Self {
            path,
            tx: Some(tx),
            writer: None,
            fault: Arc::default(),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn record_segment(&self, seg: &ContinuousSegment, t0: f64, fresh_epoch: bool) {
        self.push(Entry::Segment {
            seg: seg.clone(),
            t0,
            fresh_epoch,
        });
    }

    pub fn record_drain(&self, cause: DrainCause, moves_sent: u64) {
        self.push(Entry::Drain { cause, moves_sent });
    }

    pub fn record_nudge(&self, nudge: RecordedNudge) {
        self.push(Entry::Nudge(nudge));
    }

    fn push(&self, entry: Entry) {
        let tx = self.tx.as_ref().expect("the sender lives until drop");
        match tx.try_send(entry) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => latch(
                &self.fault,
                format!("writer backlog full ({CHANNEL_ENTRIES} entries); the record fell behind"),
            ),
            Err(TrySendError::Disconnected(_)) => {
                latch(&self.fault, "writer stopped".to_string());
            }
        }
    }

    /// The first fault since the record opened: the file no longer holds
    /// everything dispatched.
    pub fn check(&self) -> Result<(), String> {
        match self
            .fault
            .lock()
            .expect("dispatch record fault lock")
            .as_ref()
        {
            Some(message) => Err(format!(
                "dispatch record {}: {message}",
                self.path.display()
            )),
            None => Ok(()),
        }
    }

    fn close(&mut self) -> Result<(), String> {
        self.tx.take();
        if let Some(writer) = self.writer.take() {
            writer
                .join()
                .map_err(|_| "writer thread panicked".to_string())??;
        }
        self.check()
    }
}

impl Drop for DispatchRecorder {
    fn drop(&mut self) {
        if let Err(error) = self.close() {
            tracing::error!(
                subsystem = "motion",
                event = "dispatch_record_close_failed",
                path = %self.path.display(),
                %error,
                "[dispatch-record] the record file is incomplete"
            );
            if !std::thread::panicking() {
                panic!("the dispatch record file is incomplete: {error}");
            }
        }
    }
}

fn latch(fault: &Mutex<Option<String>>, message: String) {
    fault
        .lock()
        .expect("dispatch record fault lock")
        .get_or_insert(message);
}

fn write_entries(rx: &Receiver<Entry>, mut file: BufWriter<File>) -> Result<(), String> {
    let mut encoded = Vec::new();
    loop {
        match rx.recv_timeout(IDLE_FLUSH) {
            Ok(entry) => {
                encoded.clear();
                entry.encode(&mut encoded)?;
                file.write_all(&encoded).and_then(|()| {
                    if matches!(entry, Entry::Drain { .. }) {
                        file.flush()
                    } else {
                        Ok(())
                    }
                })
            }
            Err(RecvTimeoutError::Timeout) => file.flush(),
            Err(RecvTimeoutError::Disconnected) => {
                return file.flush().map_err(|e| format!("final flush failed: {e}"));
            }
        }
        .map_err(|e| format!("write failed: {e}"))?;
    }
}

#[cfg(test)]
pub(crate) mod tests;
