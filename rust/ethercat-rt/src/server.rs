use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::time::Duration;

use mcu_transport::demux::{Demuxer, Frame};

use crate::wire::{decode_command, Command};

/// Reads and decodes on a companion thread: a single frame's decode is
/// atomic and a strain-comp map measured 305 us — more than the RT loop's
/// whole latch margin — so no on-thread byte budget can bound it. The RT
/// side only try_recvs decoded commands and writes responses; the socket
/// stays nonblocking because reader and writer share the fd and the RT
/// thread's writes must never block.
pub struct FrameServer {
    cmd_rx: Receiver<Command>,
    writer_rx: Receiver<UnixStream>,
    writer: Option<UnixStream>,
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

struct ReaderShared {
    cmd_tx: Sender<Command>,
    writer_tx: Sender<UnixStream>,
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
        let (cmd_tx, cmd_rx) = channel();
        let (writer_tx, writer_rx) = channel();
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
            session_ended,
            closed,
        })
    }

    /// Pick up the writer half once the reader thread has accepted a client.
    pub fn pump(&mut self) {
        if self.writer.is_none() {
            if let Ok(w) = self.writer_rx.try_recv() {
                self.writer = Some(w);
            }
        }
    }

    pub fn pop_command(&mut self) -> Option<Command> {
        self.cmd_rx.try_recv().ok()
    }

    pub fn poll_commands(&mut self) -> Vec<Command> {
        self.pump();
        let mut cmds = Vec::new();
        while let Ok(cmd) = self.cmd_rx.try_recv() {
            cmds.push(cmd);
        }
        cmds
    }

    pub fn respond(&mut self, frame: &[u8]) {
        if let Some(stream) = self.writer.as_mut() {
            if let Err(e) = stream.write_all(frame) {
                eprintln!("ec-rt: write error: {e}");
                self.writer = None;
                self.session_ended.store(true, Ordering::Release);
            }
        }
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
        self.respond(frame);
        self.writer = None;
        self.session_ended.store(true, Ordering::Release);
    }
}

impl Drop for FrameServer {
    fn drop(&mut self) {
        self.closed.store(true, Ordering::Release);
    }
}

fn reader_loop(listener: UnixListener, shared: ReaderShared) {
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
                        if shared.writer_tx.send(w).is_err() {
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
    read_loop(stream, &shared);
}

fn read_loop(mut stream: UnixStream, shared: &ReaderShared) {
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
                                if shared.cmd_tx.send(cmd).is_err() {
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
