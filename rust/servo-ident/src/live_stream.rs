//! File-less live telemetry: a consumer for the ethercat-rt capture tap
//! unix socket, feeding `GET /api/live_tap`.
//!
//! Wire contract (frozen): connecting to the tap socket yields exactly one
//! scap v2 JSON header line terminated by `'\n'` — the same header a
//! `.scap` file starts with — then an unbounded stream of fixed-size binary
//! records in the header's layout. `cycle_index` increments by one per DC
//! cycle; a jump greater than one means the tap dropped records under
//! backpressure, which is surfaced to clients as a gap, never an error.
//! The stream has no end: this consumer connects lazily on the first poll,
//! keeps the last [`RING_SECONDS`] of records in memory, and hangs up once
//! the dashboard stops polling for the idle timeout — that hang-up is what
//! turns the RT-side tap off.
//!
//! [`LiveTap::poll`] answers one of three statuses, always as HTTP 200:
//! `{"status":"connecting"}` while no session is streaming yet,
//! `{"status":"unreachable","reason":..}` after a connect/read failure
//! (the poll that sees it also kicks off a fresh connect), and
//! `{"status":"streaming",..}` with `fs_hz`, `cycle_ns`, `drive_names`
//! (header order), `counts_per_mm` (same order) and `next_cycle` — plus,
//! when the client echoes `next_cycle` back as `since_cycle`:
//! `first_cycle`, `stride`, `drives:{name:{ferr,torque,target,pos}}`, and
//! `moving` for samples strictly after `since_cycle`, thinned to at most
//! [`MAX_POINTS_PER_RESPONSE`] points per series.
//!
//! Guaranteed invariant: a data response never spans a `cycle_index`
//! discontinuity — it ends at the last record before the first hole, with
//! `next_cycle` at that record, so within any single response sample `i`
//! lies exactly at cycle `first_cycle + i*stride`. The client sees a hole
//! (dropped records, or a `since_cycle` older than the ring — never an
//! error) as the next response's `first_cycle` jumping past
//! `since_cycle + 1`.
//!
//! `ferr`/`torque` are served in the host (kinematic) frame: the header's
//! per-drive `invert` flag negates both, unlike the drive-frame per-drive
//! series everywhere else — this feed exists to compare motors doing the
//! same physical move, and mirrored pairs must look alike.
//!
//! `target`/`pos` (commanded `target_counts` and encoder
//! `position_actual`) stay drive-frame raw: the dashboard's spatial view
//! maps them to cartesian through the `spatial` frame SERVO_DUMP_TUNING
//! writes into drive_state.json, and that frame already folds the invert
//! sign in — applying it here too would cancel it back out.

use std::collections::VecDeque;
use std::io::{ErrorKind, Read};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use serde_json::json;

use crate::scap::{Channel, Dtype, Header, FLAG_MOTION_ACTIVE};

pub const DEFAULT_TAP_SOCKET: &str = "/tmp/kalico-ethercat.sock.live";
pub const DEFAULT_IDLE_TIMEOUT: Duration = Duration::from_secs(10);

const RING_SECONDS: u64 = 30;
const MAX_POINTS_PER_RESPONSE: usize = 2_000;
const HEADER_LINE_CAP: usize = 65_536;
const MIN_READ_TICK: Duration = Duration::from_millis(20);

pub struct LiveTap {
    shared: Arc<Shared>,
}

struct Shared {
    socket_path: PathBuf,
    idle_timeout: Duration,
    inner: Mutex<Inner>,
}

struct Inner {
    last_poll: Instant,
    session: Session,
}

enum Session {
    Idle,
    Connecting,
    Streaming(Stream),
    Failed { reason: String },
}

struct Stream {
    cycle_ns: u64,
    drive_names: Vec<String>,
    sign: Vec<i64>,
    counts_per_mm: Vec<f64>,
    ring: Ring,
    timing: Option<Timing>,
}

#[derive(Clone, Copy)]
struct Timing {
    skips: u32,
    late_frames: u32,
    lateness_ns: i32,
}

struct Ring {
    cap: usize,
    cycle: VecDeque<u64>,
    flags: VecDeque<u8>,
    ferr: Vec<VecDeque<i32>>,
    torque: Vec<VecDeque<i16>>,
    target: Vec<VecDeque<i32>>,
    pos: Vec<VecDeque<i32>>,
}

impl Ring {
    fn new(header: &Header) -> Ring {
        let cap = usize::try_from(RING_SECONDS * 1_000_000_000 / header.cycle_ns)
            .unwrap_or(usize::MAX)
            .max(1);
        let n_drives = header.drives.len();
        Ring {
            cap,
            cycle: VecDeque::new(),
            flags: VecDeque::new(),
            ferr: vec![VecDeque::new(); n_drives],
            torque: vec![VecDeque::new(); n_drives],
            target: vec![VecDeque::new(); n_drives],
            pos: vec![VecDeque::new(); n_drives],
        }
    }

    fn trim(&mut self) {
        while self.cycle.len() > self.cap {
            self.cycle.pop_front();
            self.flags.pop_front();
            for f in &mut self.ferr {
                f.pop_front();
            }
            for t in &mut self.torque {
                t.pop_front();
            }
            for t in &mut self.target {
                t.pop_front();
            }
            for p in &mut self.pos {
                p.pop_front();
            }
        }
    }
}

impl LiveTap {
    pub fn new(socket_path: PathBuf, idle_timeout: Duration) -> LiveTap {
        LiveTap {
            shared: Arc::new(Shared {
                socket_path,
                idle_timeout,
                inner: Mutex::new(Inner {
                    last_poll: Instant::now(),
                    session: Session::Idle,
                }),
            }),
        }
    }

    pub fn poll(&self, since_cycle: Option<u64>) -> serde_json::Value {
        let mut inner = self.shared.lock_inner();
        inner.last_poll = Instant::now();
        match &inner.session {
            Session::Idle => {
                spawn_session(&self.shared, &mut inner);
                json!({ "status": "connecting" })
            }
            Session::Connecting => json!({ "status": "connecting" }),
            Session::Failed { reason } => {
                let reason = reason.clone();
                spawn_session(&self.shared, &mut inner);
                json!({ "status": "unreachable", "reason": reason })
            }
            Session::Streaming(stream) if stream.ring.cycle.is_empty() => {
                json!({ "status": "connecting" })
            }
            Session::Streaming(stream) => match since_cycle {
                None => attach_payload(stream),
                Some(since) => samples_payload(stream, since),
            },
        }
    }
}

fn fs_hz(cycle_ns: u64) -> f64 {
    1e9 / cycle_ns as f64
}

fn attach_payload(stream: &Stream) -> serde_json::Value {
    let Some(&newest) = stream.ring.cycle.back() else {
        return json!({ "status": "connecting" });
    };
    json!({
        "status": "streaming",
        "fs_hz": fs_hz(stream.cycle_ns),
        "cycle_ns": stream.cycle_ns,
        "drive_names": stream.drive_names,
        "counts_per_mm": stream.counts_per_mm,
        "next_cycle": newest,
        "timing": timing_json(stream),
    })
}

fn timing_json(stream: &Stream) -> serde_json::Value {
    match stream.timing {
        Some(t) => json!({
            "skips": t.skips,
            "late_frames": t.late_frames,
            "lateness_ns": t.lateness_ns,
        }),
        None => serde_json::Value::Null,
    }
}

fn samples_payload(stream: &Stream, since: u64) -> serde_json::Value {
    let ring = &stream.ring;
    let start = ring.cycle.partition_point(|&c| c <= since);
    let mut end = start;
    while end < ring.cycle.len() && (end == start || ring.cycle[end] == ring.cycle[end - 1] + 1) {
        end += 1;
    }
    let n = end - start;
    let stride = n.div_ceil(MAX_POINTS_PER_RESPONSE).max(1);
    let kept: Vec<usize> = (start..end).step_by(stride).collect();
    let moving: Vec<bool> = kept
        .iter()
        .map(|&i| i64::from(ring.flags[i]) & FLAG_MOTION_ACTIVE != 0)
        .collect();
    let mut drives = serde_json::Map::new();
    for (d, name) in stream.drive_names.iter().enumerate() {
        let sign = stream.sign[d];
        let ferr: Vec<i64> = kept
            .iter()
            .map(|&i| sign * i64::from(ring.ferr[d][i]))
            .collect();
        let torque: Vec<i64> = kept
            .iter()
            .map(|&i| sign * i64::from(ring.torque[d][i]))
            .collect();
        let target: Vec<i64> = kept.iter().map(|&i| i64::from(ring.target[d][i])).collect();
        let pos: Vec<i64> = kept.iter().map(|&i| i64::from(ring.pos[d][i])).collect();
        drives.insert(
            name.clone(),
            json!({ "ferr": ferr, "torque": torque, "target": target, "pos": pos }),
        );
    }
    json!({
        "status": "streaming",
        "fs_hz": fs_hz(stream.cycle_ns),
        "cycle_ns": stream.cycle_ns,
        "drive_names": stream.drive_names,
        "counts_per_mm": stream.counts_per_mm,
        "next_cycle": if n == 0 { since } else { ring.cycle[end - 1] },
        "first_cycle": ring.cycle.get(start),
        "stride": stride,
        "drives": drives,
        "moving": moving,
        "timing": timing_json(stream),
    })
}

impl Shared {
    fn lock_inner(&self) -> MutexGuard<'_, Inner> {
        self.inner.lock().expect("live tap state lock poisoned")
    }

    fn idled(&self) -> bool {
        self.lock_inner().last_poll.elapsed() >= self.idle_timeout
    }
}

fn spawn_session(shared: &Arc<Shared>, inner: &mut Inner) {
    inner.session = Session::Connecting;
    let shared = Arc::clone(shared);
    std::thread::spawn(move || {
        let end = run_session(&shared);
        let mut inner = shared.lock_inner();
        inner.session = match end {
            Ok(SessionEnd::IdleStop) => Session::Idle,
            Err(reason) => Session::Failed { reason },
        };
    });
}

enum SessionEnd {
    IdleStop,
}

enum Step<T> {
    Ready(T),
    IdleStop,
}

fn run_session(shared: &Shared) -> Result<SessionEnd, String> {
    let mut stream = UnixStream::connect(&shared.socket_path)
        .map_err(|e| format!("connect {}: {e}", shared.socket_path.display()))?;
    let tick = (shared.idle_timeout / 4).max(MIN_READ_TICK);
    stream
        .set_read_timeout(Some(tick))
        .map_err(|e| format!("set_read_timeout: {e}"))?;
    let header_line = match read_header_line(&mut stream, shared)? {
        Step::IdleStop => return Ok(SessionEnd::IdleStop),
        Step::Ready(line) => line,
    };
    let header = Header::parse_line(&header_line).map_err(|e| format!("tap {e}"))?;
    let layout = RecordLayout::resolve(&header)?;
    {
        let mut inner = shared.lock_inner();
        inner.session = Session::Streaming(Stream {
            cycle_ns: header.cycle_ns,
            drive_names: header.drives.iter().map(|d| d.name.clone()).collect(),
            sign: header
                .drives
                .iter()
                .map(|d| if d.invert { -1 } else { 1 })
                .collect(),
            counts_per_mm: header.drives.iter().map(|d| d.counts_per_mm).collect(),
            ring: Ring::new(&header),
            timing: None,
        });
    }
    let mut record = vec![0u8; header.record_size];
    loop {
        if let Step::IdleStop = read_full(&mut stream, &mut record, shared)? {
            return Ok(SessionEnd::IdleStop);
        }
        if let Step::IdleStop = append_record(shared, &layout, &record)? {
            return Ok(SessionEnd::IdleStop);
        }
    }
}

fn retriable(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted
    )
}

fn read_header_line(stream: &mut UnixStream, shared: &Shared) -> Result<Step<Vec<u8>>, String> {
    let mut line = Vec::new();
    let mut byte = [0u8; 1];
    loop {
        match stream.read(&mut byte) {
            Ok(0) => return Err("tap closed before sending a header".to_string()),
            Ok(_) => {
                if byte[0] == b'\n' {
                    return Ok(Step::Ready(line));
                }
                line.push(byte[0]);
                if line.len() > HEADER_LINE_CAP {
                    return Err(format!(
                        "tap header exceeds {HEADER_LINE_CAP} bytes without a newline"
                    ));
                }
            }
            Err(e) if retriable(&e) => {
                if shared.idled() {
                    return Ok(Step::IdleStop);
                }
            }
            Err(e) => return Err(format!("tap header read: {e}")),
        }
    }
}

fn read_full(stream: &mut UnixStream, buf: &mut [u8], shared: &Shared) -> Result<Step<()>, String> {
    let mut filled = 0usize;
    while filled < buf.len() {
        match stream.read(&mut buf[filled..]) {
            Ok(0) => {
                return Err(if filled == 0 {
                    "tap closed the connection".to_string()
                } else {
                    format!("tap closed mid-record ({filled} of {} bytes)", buf.len())
                })
            }
            Ok(n) => filled += n,
            Err(e) if retriable(&e) => {
                if shared.idled() {
                    return Ok(Step::IdleStop);
                }
            }
            Err(e) => return Err(format!("tap read: {e}")),
        }
    }
    Ok(Step::Ready(()))
}

struct Slot {
    dtype: Dtype,
    offset: usize,
}

impl Slot {
    fn read(&self, record: &[u8]) -> i64 {
        self.dtype.read_i64(&record[self.offset..])
    }
}

struct RecordLayout {
    cycle: Slot,
    flags: Slot,
    ferr: Vec<Slot>,
    torque: Vec<Slot>,
    target: Vec<Slot>,
    pos: Vec<Slot>,
    /// RT-loop health counters — absent on captures from older endpoints.
    skip_count: Option<Slot>,
    late_frames: Option<Slot>,
    frame_lateness_ns: Option<Slot>,
}

impl RecordLayout {
    fn resolve(header: &Header) -> Result<RecordLayout, String> {
        let channel = |name: &str| -> Result<&Channel, String> {
            header
                .channel(name)
                .ok_or_else(|| format!("tap header has no channel {name:?}"))
        };
        let prefix = |name: &str| -> Result<Slot, String> {
            let ch = channel(name)?;
            Ok(Slot {
                dtype: ch.dtype,
                offset: ch.offset,
            })
        };
        let per_drive = |name: &str| -> Result<Vec<Slot>, String> {
            let ch = channel(name)?;
            Ok((0..header.drives.len())
                .map(|i| Slot {
                    dtype: ch.dtype,
                    offset: header.eff_offset(ch, i),
                })
                .collect())
        };
        Ok(RecordLayout {
            cycle: prefix("cycle_index")?,
            flags: prefix("flags")?,
            ferr: per_drive("following_error")?,
            torque: per_drive("torque_actual")?,
            target: per_drive("target_counts")?,
            pos: per_drive("position_actual")?,
            skip_count: prefix("skip_count").ok(),
            late_frames: prefix("late_frames").ok(),
            frame_lateness_ns: prefix("frame_lateness_ns").ok(),
        })
    }
}

fn append_record(
    shared: &Shared,
    layout: &RecordLayout,
    record: &[u8],
) -> Result<Step<()>, String> {
    let mut inner = shared.lock_inner();
    if inner.last_poll.elapsed() >= shared.idle_timeout {
        return Ok(Step::IdleStop);
    }
    let Session::Streaming(stream) = &mut inner.session else {
        return Err("live tap session replaced while its reader was running".to_string());
    };
    let cycle = layout.cycle.read(record) as u64;
    if let Some(&prev) = stream.ring.cycle.back() {
        if cycle <= prev {
            return Err(format!(
                "tap cycle_index went from {prev} to {cycle}; it must strictly increase"
            ));
        }
    }
    let ring = &mut stream.ring;
    ring.cycle.push_back(cycle);
    ring.flags.push_back(layout.flags.read(record) as u8);
    for (drive, slot) in layout.ferr.iter().enumerate() {
        ring.ferr[drive].push_back(slot.read(record) as i32);
    }
    for (drive, slot) in layout.torque.iter().enumerate() {
        ring.torque[drive].push_back(slot.read(record) as i16);
    }
    for (drive, slot) in layout.target.iter().enumerate() {
        ring.target[drive].push_back(slot.read(record) as i32);
    }
    for (drive, slot) in layout.pos.iter().enumerate() {
        ring.pos[drive].push_back(slot.read(record) as i32);
    }
    ring.trim();
    if let (Some(sk), Some(lf), Some(ln)) = (
        &layout.skip_count,
        &layout.late_frames,
        &layout.frame_lateness_ns,
    ) {
        stream.timing = Some(Timing {
            skips: sk.read(record) as u32,
            late_frames: lf.read(record) as u32,
            lateness_ns: ln.read(record) as i32,
        });
    }
    Ok(Step::Ready(()))
}
