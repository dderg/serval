use std::io::{self, ErrorKind, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use mcu_transport::demux::{Demuxer, Frame};

use crate::wire::{decode_command, Command};

/// Reads and decodes on a companion thread: a single frame's decode is
/// atomic and a strain-comp map measured 305 us — more than the RT loop's
/// whole latch margin — so no on-thread byte budget can bound it. The RT
/// side only pops decoded commands from a wait-free rtrb ring and writes
/// responses; the socket stays nonblocking because reader and writer share
/// the fd and the RT thread's writes must never block. The rings are rtrb,
/// not mpsc: an mpsc `try_recv` spin-waits on a sender caught mid-write, so
/// a preempted reader thread could stall the FIFO-80 DC thread (802 µs of
/// `dispatch_ns` on the 2026-07-27 bench) — rtrb pop either sees a published
/// slot or reports empty, never waits.
pub struct FrameServer {
    cmd_rx: rtrb::Consumer<Command>,
    writer_rx: rtrb::Consumer<UnixStream>,
    writer: Option<Box<dyn Write + Send>>,
    pending: Vec<u8>,
    pending_since: Option<Instant>,
    pending_max: usize,
    session_ended: Arc<AtomicBool>,
    closed: Arc<AtomicBool>,
}

impl core::fmt::Debug for FrameServer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FrameServer")
            .field("conn", &self.writer.is_some())
            .finish_non_exhaustive()
    }
}

const READER_POLL: Duration = Duration::from_millis(2);

/// Commands the reader may decode ahead of the RT dispatch loop. The
/// dispatch budget drains every cycle (250 µs), so this only fills when the
/// RT loop stops dispatching; the reader then stops reading and the socket
/// backpressures the host.
const CMD_QUEUE_CAPACITY: usize = 1024;

const MAX_PENDING_BYTES: usize = 2 * 1024 * 1024;
const PENDING_STALL_DEADLINE: Duration = Duration::from_secs(5);

enum WriteStep {
    Done,
    Blocked,
    Failed(io::Error),
}

fn nb_write_all<W: Write + ?Sized>(writer: &mut W, buf: &[u8]) -> (usize, WriteStep) {
    let mut off = 0;
    while off < buf.len() {
        match writer.write(&buf[off..]) {
            Ok(0) => return (off, WriteStep::Blocked),
            Ok(n) => off += n,
            Err(e) if e.kind() == ErrorKind::Interrupted => {}
            Err(e) if e.kind() == ErrorKind::WouldBlock => return (off, WriteStep::Blocked),
            Err(e) => return (off, WriteStep::Failed(e)),
        }
    }
    (off, WriteStep::Done)
}

struct ReaderShared {
    cmd_tx: rtrb::Producer<Command>,
    writer_tx: rtrb::Producer<UnixStream>,
    session_ended: Arc<AtomicBool>,
    closed: Arc<AtomicBool>,
}

impl FrameServer {
    pub fn bind(path: &str) -> std::io::Result<Self> {
        let _ = std::fs::remove_file(path);
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        // 0o666: endpoint runs as root; non-root clients (motion-engine) must connect.
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o666))?;
        let (cmd_tx, cmd_rx) = rtrb::RingBuffer::new(CMD_QUEUE_CAPACITY);
        let (writer_tx, writer_rx) = rtrb::RingBuffer::new(1);
        let session_ended = Arc::new(AtomicBool::new(false));
        let closed = Arc::new(AtomicBool::new(false));
        let shared = ReaderShared {
            cmd_tx,
            writer_tx,
            session_ended: Arc::clone(&session_ended),
            closed: Arc::clone(&closed),
        };
        std::thread::Builder::new()
            .name("ec-rt-sock-read".into())
            .spawn(move || reader_loop(listener, shared))
            .map_err(std::io::Error::other)?;
        Ok(Self {
            cmd_rx,
            writer_rx,
            writer: None,
            pending: Vec::new(),
            pending_since: None,
            pending_max: 0,
            session_ended,
            closed,
        })
    }

    /// Pick up the writer half once the reader thread has accepted a client,
    /// then service any buffered backpressure so a transient host stall drains.
    pub fn pump(&mut self) {
        if self.writer.is_none() {
            if let Ok(w) = self.writer_rx.pop() {
                self.writer = Some(Box::new(w));
            }
        }
        self.flush_pending();
    }

    pub fn pop_command(&mut self) -> Option<Command> {
        self.cmd_rx.pop().ok()
    }

    pub fn poll_commands(&mut self) -> Vec<Command> {
        self.pump();
        let mut cmds = Vec::new();
        while let Ok(cmd) = self.cmd_rx.pop() {
            cmds.push(cmd);
        }
        cmds
    }

    pub fn respond(&mut self, frame: &[u8]) {
        if self.writer.is_none() {
            return;
        }
        if !self.pending.is_empty() {
            self.flush_pending();
            if self.writer.is_none() {
                return;
            }
            if !self.pending.is_empty() {
                self.stash(frame);
                return;
            }
        }
        self.write_frame(frame);
    }

    pub fn client_connected(&self) -> bool {
        self.writer.is_some()
    }

    pub fn session_ended(&self) -> bool {
        self.session_ended.load(Ordering::Acquire)
    }

    pub fn respond_and_close(&mut self, frame: &[u8]) {
        if self.writer.is_none() {
            eprintln!("ec-rt: respond_and_close called with no client — frame dropped");
            self.session_ended.store(true, Ordering::Release);
            return;
        }
        self.flush_pending();
        self.respond(frame);
        self.writer = None;
        self.clear_pending();
        self.session_ended.store(true, Ordering::Release);
    }

    fn write_frame(&mut self, frame: &[u8]) {
        let (sent, step) = {
            let writer = self.writer.as_mut().expect("write_frame requires a writer");
            nb_write_all(writer, frame)
        };
        match step {
            WriteStep::Done => {}
            WriteStep::Blocked => self.stash(&frame[sent..]),
            WriteStep::Failed(e) => self.fail_write(e),
        }
    }

    fn flush_pending(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        if self.writer.is_none() {
            self.clear_pending();
            return;
        }
        let (sent, step) = {
            let writer = self
                .writer
                .as_mut()
                .expect("flush_pending requires a writer");
            nb_write_all(writer, &self.pending)
        };
        match step {
            WriteStep::Done => {
                self.pending.clear();
                self.finish_episode();
            }
            WriteStep::Blocked => {
                if sent > 0 {
                    self.pending.drain(..sent);
                }
                self.enforce_pending_limits();
            }
            WriteStep::Failed(e) => self.fail_write(e),
        }
    }

    fn stash(&mut self, bytes: &[u8]) {
        self.pending.extend_from_slice(bytes);
        self.start_episode_if_needed();
        self.enforce_pending_limits();
    }

    fn start_episode_if_needed(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let pending_bytes = self.pending.len();
        if self.pending_since.is_none() {
            self.pending_since = Some(Instant::now());
            self.pending_max = pending_bytes;
            eprintln!("ec-rt: bridge socket full — buffering ({pending_bytes} bytes)");
            tracing::warn!(
                subsystem = "ethercat",
                event = "bridge_backpressure",
                pending_bytes,
                "bridge socket full; buffering frames"
            );
        } else if pending_bytes > self.pending_max {
            self.pending_max = pending_bytes;
        }
    }

    fn finish_episode(&mut self) {
        if let Some(since) = self.pending_since.take() {
            let stalled_ms = since.elapsed().as_millis() as u64;
            let max_pending_bytes = self.pending_max;
            self.pending_max = 0;
            eprintln!(
                "ec-rt: bridge backpressure recovered (peak {max_pending_bytes} bytes over {stalled_ms} ms)"
            );
            tracing::info!(
                subsystem = "ethercat",
                event = "bridge_backpressure_recovered",
                max_pending_bytes,
                stalled_ms,
                "bridge write backlog drained; session healthy"
            );
        }
    }

    fn enforce_pending_limits(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let pending_bytes = self.pending.len();
        let stalled = self.pending_since.map_or(Duration::ZERO, |t| t.elapsed());
        if pending_bytes > MAX_PENDING_BYTES || stalled > PENDING_STALL_DEADLINE {
            let stalled_ms = stalled.as_millis() as u64;
            eprintln!(
                "ec-rt: bridge write backlog exceeded limits — ending session ({pending_bytes} bytes, {stalled_ms} ms)"
            );
            tracing::error!(
                subsystem = "ethercat",
                event = "bridge_stalled",
                pending_bytes,
                stalled_ms,
                "bridge write backlog exceeded limits — ending session"
            );
            self.writer = None;
            self.clear_pending();
            self.session_ended.store(true, Ordering::Release);
        }
    }

    fn fail_write(&mut self, e: io::Error) {
        eprintln!("ec-rt: write error: {e}");
        tracing::error!(
            subsystem = "ethercat",
            event = "bridge_write_error",
            error = %e,
            "bridge write failed — ending session"
        );
        self.writer = None;
        self.clear_pending();
        self.session_ended.store(true, Ordering::Release);
    }

    fn clear_pending(&mut self) {
        self.pending.clear();
        self.pending_since = None;
        self.pending_max = 0;
    }

    #[cfg(test)]
    pub(crate) fn with_writer_for_test(writer: Box<dyn Write + Send>) -> Self {
        let (_cmd_tx, cmd_rx) = rtrb::RingBuffer::new(1);
        let (_writer_tx, writer_rx) = rtrb::RingBuffer::new(1);
        Self {
            cmd_rx,
            writer_rx,
            writer: Some(writer),
            pending: Vec::new(),
            pending_since: None,
            pending_max: 0,
            session_ended: Arc::new(AtomicBool::new(false)),
            closed: Arc::new(AtomicBool::new(false)),
        }
    }

    #[cfg(test)]
    pub(crate) fn pending_len(&self) -> usize {
        self.pending.len()
    }

    #[cfg(test)]
    pub(crate) fn backpressure_active(&self) -> bool {
        self.pending_since.is_some()
    }

    #[cfg(test)]
    pub(crate) fn force_pending_since(&mut self, when: Instant) {
        self.pending_since = Some(when);
    }
}

impl Drop for FrameServer {
    fn drop(&mut self) {
        self.closed.store(true, Ordering::Release);
    }
}

fn reader_loop(listener: UnixListener, mut shared: ReaderShared) {
    let stream = loop {
        if shared.closed.load(Ordering::Acquire) || shared.session_ended.load(Ordering::Acquire) {
            return;
        }
        match listener.accept() {
            Ok((stream, _)) => {
                // Stays nonblocking: reader and RT-side writer share the fd,
                // and the writer must never block on a full buffer.
                let _ = stream.set_nonblocking(true);
                eprintln!("ec-rt: client connected");
                match stream.try_clone() {
                    Ok(w) => {
                        if shared.writer_tx.push(w).is_err() {
                            return;
                        }
                    }
                    Err(e) => {
                        eprintln!("ec-rt: stream clone failed: {e}");
                        shared.session_ended.store(true, Ordering::Release);
                        return;
                    }
                }
                break stream;
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => std::thread::sleep(READER_POLL),
            Err(e) => {
                eprintln!("ec-rt: accept error: {e}");
                std::thread::sleep(READER_POLL);
            }
        }
    };
    read_loop(stream, &mut shared);
}

fn read_loop(mut stream: UnixStream, shared: &mut ReaderShared) {
    let mut demux = Demuxer::new();
    let mut buf = [0u8; 4096];
    loop {
        if shared.closed.load(Ordering::Acquire) {
            return;
        }
        match stream.read(&mut buf) {
            Ok(0) => {
                eprintln!("ec-rt: client disconnected");
                shared.session_ended.store(true, Ordering::Release);
                return;
            }
            Ok(n) => {
                let (frames, errs) = demux.feed_slice(&buf[..n]);
                for e in &errs {
                    eprintln!("ec-rt: stream error: {e:?}");
                }
                for f in frames {
                    if let Frame::Kalico { channel, payload } = f {
                        match decode_command(channel, &payload) {
                            Ok(cmd) => {
                                if push_command(shared, cmd).is_break() {
                                    return;
                                }
                            }
                            Err(e) => eprintln!("ec-rt: bad command: {e:?}"),
                        }
                    }
                }
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {
                std::thread::sleep(READER_POLL)
            }
            Err(e) => {
                eprintln!("ec-rt: read error: {e}");
                shared.session_ended.store(true, Ordering::Release);
                return;
            }
        }
    }
}

/// Blocks the reader (never the RT thread) when the command ring is full,
/// re-trying until the RT side drains it or the session ends.
fn push_command(shared: &mut ReaderShared, cmd: Command) -> std::ops::ControlFlow<()> {
    let mut cmd = cmd;
    loop {
        if shared.cmd_tx.is_abandoned() || shared.closed.load(Ordering::Acquire) {
            return std::ops::ControlFlow::Break(());
        }
        match shared.cmd_tx.push(cmd) {
            Ok(()) => return std::ops::ControlFlow::Continue(()),
            Err(rtrb::PushError::Full(returned)) => {
                cmd = returned;
                std::thread::sleep(READER_POLL);
            }
        }
    }
}

#[cfg(test)]
#[path = "server_tests.rs"]
mod server_tests;
