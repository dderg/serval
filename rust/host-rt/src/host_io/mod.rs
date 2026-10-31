pub mod byte_link;
pub mod call_handle;
pub mod can_link;
pub mod events;
pub mod fire_and_forget_depth;
pub mod identify;
pub(crate) mod interceptor;
pub mod link_health;
pub mod mcu_session;
pub mod parser;
pub mod reactor;
pub mod rtt;
pub mod runtime_events;
pub mod serial_frame_io;
pub use identify::IdentifySeqState;
pub use interceptor::InterceptorId;
pub use serial_frame_io::SerialFrameIo;
#[cfg(any(test, feature = "test-harness"))]
pub mod test_harness;
pub mod window;
pub mod wire;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Sender, SyncSender};
use std::thread::JoinHandle;
use std::time::Duration;

use arc_swap::ArcSwap;

use crate::host_io::fire_and_forget_depth::FireAndForgetDepth;
use crate::host_io::link_health::LinkHealth;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::runtime_events::{FaultEvent, McuLogEvent, RuntimeEvent, StatusEvent};
use crate::transport::{MessageParams, SubscribeError, TransportError};

const DEFAULT_BAUD: u32 = 250_000;
const IDENTIFY_TIMEOUT: Duration = Duration::from_millis(15_000);
const RUNTIME_EVENT_CAPACITY: usize = 512;
const RUNTIME_EVENT_BULK_CAPACITY: usize = 4096;

pub enum ReactorCommand {
    Call {
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: SyncSender<Result<MessageParams, TransportError>>,
        deadline: std::time::Instant,
    },
    Abandon(u64),
    AttachHeartbeatCallback(Arc<dyn Fn(&[u32], &[u64]) + Send + Sync>),
    SetMcuLogHook(Box<dyn Fn(McuLogEvent) + Send + Sync>),
    SubscribeFault {
        sender: SyncSender<FaultEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeRuntimeEvents {
        priority: SyncSender<RuntimeEvent>,
        bulk: SyncSender<RuntimeEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    FireAndForget {
        payload: Vec<u8>,
    },
    /// A burst of encoded commands to pack into as few Klipper message
    /// blocks as the 64-byte block allows before it reaches the wire.
    /// `reserved_blocks` is the admission capacity the sender claimed on
    /// [`FireAndForgetDepth`]; the reactor releases it once the burst is
    /// queued or abandoned.
    FireAndForgetBatch {
        payloads: Vec<Vec<u8>>,
        reserved_blocks: usize,
        enqueued_at: std::time::Instant,
    },
    McuIdentify {
        completion:
            SyncSender<Result<crate::host_io::mcu_session::IdentifyOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    McuCall {
        kind: mcu_protocol::MessageKind,
        body: Vec<u8>,
        completion: SyncSender<Result<crate::host_io::mcu_session::McuCallOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    /// Encode and send `get_clock` (with the reactor's own per-MCU parser —
    /// the bridge-level parser carries whichever MCU's dictionary was set
    /// last and must never encode for a specific MCU). The send timestamp is
    /// captured in the reactor immediately before the wire write; when the
    /// "clock" response arrives as an unsolicited frame the reactor injects
    /// honest RAW timestamps and delivers it as a PassthroughResponse
    /// runtime event.
    GetClockAndDeliver,
    Shutdown,
    Noop,
    RegisterInterceptor {
        msg_name: String,
        oid: Option<u32>,
        callback: Box<dyn Fn(&MessageParams) + Send + Sync>,
        reply: SyncSender<crate::host_io::InterceptorId>,
    },
    UnregisterInterceptor {
        id: crate::host_io::InterceptorId,
    },
    MarkExpectedDisconnect,
}

pub struct McuHostIo {
    submission_tx: Sender<ReactorCommand>,
    next_call_id: AtomicU64,
    reactor_handle: Option<JoinHandle<()>>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    parser: Arc<MsgProtoParser>,
    link_health: Arc<LinkHealth>,
    clock: Arc<dyn crate::clock::Clock>,
    raw_identify_bytes: Vec<u8>,
    is_critical: Arc<AtomicBool>,
    fire_and_forget_depth: Arc<FireAndForgetDepth>,
}

impl std::fmt::Debug for McuHostIo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuHostIo")
            .field("next_call_id", &self.next_call_id.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Drop for McuHostIo {
    fn drop(&mut self) {
        let _ = self.submission_tx.send(ReactorCommand::Shutdown);
        if let Some(h) = self.reactor_handle.take() {
            let _ = h.join();
        }
    }
}

impl McuHostIo {
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        Self::open_with_label(path, baud, path)
    }

    pub fn open_default(path: &str) -> Result<Self, TransportError> {
        Self::open(path, DEFAULT_BAUD)
    }

    #[cfg(target_family = "unix")]
    pub fn open_pipe(path: &str) -> Result<Self, TransportError> {
        Self::open_pipe_with_label(path, path)
    }

    #[cfg(target_family = "unix")]
    pub fn open_pipe_with_label(path: &str, label: &str) -> Result<Self, TransportError> {
        use std::os::unix::io::FromRawFd;

        // SAFETY: `libc::open` and `TTYPort::from_raw_fd` are both unsafe FFI
        // boundaries. We check the return value of `open` before using the fd.
        #[allow(unsafe_code)]
        let port_box: Box<dyn serialport::SerialPort> = {
            let cpath = std::ffi::CString::new(path)
                .map_err(|e| TransportError::Io(std::io::Error::other(e)))?;
            // O_CLOEXEC: the host transport fd must never be inherited by an
            // EtherCAT endpoint child (spawned via Command, which only sets
            // CLOEXEC on its own pipes). An inherited copy of this pts fd would
            // keep TIOCEXCL held even after the parent closes its copy, so the
            // next klippy's attach_serial would still get EBUSY.
            let fd = unsafe {
                libc::open(
                    cpath.as_ptr(),
                    libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC,
                )
            };
            if fd < 0 {
                return Err(TransportError::Io(std::io::Error::last_os_error()));
            }
            #[allow(unsafe_code)]
            unsafe {
                let mut tio: libc::termios = std::mem::zeroed();
                if libc::tcgetattr(fd, &mut tio) == 0 {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "termios_setup",
                        phase = "pre-cfmakeraw",
                        path,
                        iflag = tio.c_iflag,
                        oflag = tio.c_oflag,
                        cflag = tio.c_cflag,
                        lflag = tio.c_lflag,
                        "termios pre-cfmakeraw"
                    );
                    libc::cfmakeraw(&mut tio);
                    if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                        let err = std::io::Error::last_os_error();
                        libc::close(fd);
                        return Err(TransportError::Io(std::io::Error::other(format!(
                            "tcsetattr({path}): {err}"
                        ))));
                    }
                    let mut tio2: libc::termios = std::mem::zeroed();
                    if libc::tcgetattr(fd, &mut tio2) == 0 {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "termios_setup",
                            phase = "post-cfmakeraw",
                            path,
                            iflag = tio2.c_iflag,
                            oflag = tio2.c_oflag,
                            cflag = tio2.c_cflag,
                            lflag = tio2.c_lflag,
                            vmin = tio2.c_cc[libc::VMIN],
                            vtime = tio2.c_cc[libc::VTIME],
                            "termios post-cfmakeraw"
                        );
                    }
                }
            }
            let port = unsafe { Box::new(serialport::TTYPort::from_raw_fd(fd)) };
            #[allow(unsafe_code)]
            unsafe {
                let mut tio3: libc::termios = std::mem::zeroed();
                if libc::tcgetattr(fd, &mut tio3) == 0 {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "termios_setup",
                        phase = "post-serialport",
                        path,
                        iflag = tio3.c_iflag,
                        oflag = tio3.c_oflag,
                        cflag = tio3.c_cflag,
                        lflag = tio3.c_lflag,
                        vmin = tio3.c_cc[libc::VMIN],
                        vtime = tio3.c_cc[libc::VTIME],
                        "termios post-serialport"
                    );
                }
            }
            port
        };
        Self::open_with_port(port_box, label)
    }

    pub fn open_with_label(path: &str, baud: u32, label: &str) -> Result<Self, TransportError> {
        let port_box: Box<dyn serialport::SerialPort> = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| {
                TransportError::Io(std::io::Error::other(format!(
                    "serialport::open({path}@{baud}): {e}"
                )))
            })?;
        Self::open_with_port(port_box, label)
    }

    pub fn open_with_port(
        mut port_box: Box<dyn serialport::SerialPort>,
        label: &str,
    ) -> Result<Self, TransportError> {
        let _ = serialport::SerialPort::set_timeout(&mut *port_box, Duration::from_millis(100));
        Self::open_with_link(Box::new(port_box), label)
    }

    #[cfg(target_os = "linux")]
    pub fn open_canbus_with_label(
        interface: &str,
        uuid: u64,
        label: &str,
    ) -> Result<Self, TransportError> {
        let link = crate::host_io::can_link::CanLink::open(interface, uuid, IDENTIFY_TIMEOUT)
            .map_err(|e| {
                TransportError::Io(std::io::Error::new(
                    e.kind(),
                    format!("canbus open({interface}, uuid={uuid:012x}): {e}"),
                ))
            })?;
        Self::open_with_link(Box::new(link), label)
    }

    #[cfg(not(target_os = "linux"))]
    pub fn open_canbus_with_label(
        interface: &str,
        uuid: u64,
        _label: &str,
    ) -> Result<Self, TransportError> {
        Err(TransportError::Io(std::io::Error::other(format!(
            "canbus transport requires SocketCAN (Linux); cannot open {interface} uuid={uuid:012x}"
        ))))
    }

    pub fn open_with_link(
        link: Box<dyn crate::host_io::byte_link::ByteLink>,
        label: &str,
    ) -> Result<Self, TransportError> {
        let mut io = crate::host_io::serial_frame_io::SerialFrameIo::new_boxed(link);

        let (parser_owned, raw_identify_bytes, identify_seq) =
            identify::identify_handshake(&mut io, IDENTIFY_TIMEOUT)?;

        let mcu_can_data_rate = parser_owned
            .numeric_constant("CANBUS_DATA_FREQUENCY")
            .unwrap_or(0);
        io.try_enable_fd(u32::try_from(mcu_can_data_rate).unwrap_or(0))
            .map_err(TransportError::Io)?;

        let parser = Arc::new(parser_owned);
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));

        let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let link_health = Arc::new(LinkHealth::default());
        let reactor_link_health = Arc::clone(&link_health);
        let reactor_label: Arc<str> = label.into();
        let reactor_clock = Arc::clone(&clock);
        let is_critical = Arc::new(AtomicBool::new(true));
        let reactor_is_critical = Arc::clone(&is_critical);
        let fire_and_forget_depth = Arc::new(FireAndForgetDepth::default());
        let reactor_fire_and_forget_depth = Arc::clone(&fire_and_forget_depth);
        let reactor_handle = std::thread::spawn(move || {
            tracing::info!(
                subsystem = "mcu-comms",
                event = "reactor_spawn",
                thread_id = ?std::thread::current().id(),
                "port-bound reactor starting"
            );
            let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
                io,
                crate::host_io::reactor::ReactorSetup {
                    parser: reactor_parser,
                    submission_rx,
                    status_snapshot: reactor_status,
                    seq: identify_seq,
                    mcu_label: reactor_label,
                    link_health: reactor_link_health,
                    fire_and_forget_depth: reactor_fire_and_forget_depth,
                },
                reactor_clock,
            );
            reactor.run();
            if !reactor.exited_gracefully() {
                let critical = reactor_is_critical.load(Ordering::Acquire);
                if !critical {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "reactor_exit_non_critical",
                        mcu = reactor.mcu_label(),
                        thread_id = ?std::thread::current().id(),
                        "transport closed via IO error on NON-CRITICAL MCU; reactor exiting without abort — klippy reconnect path will handle recovery"
                    );
                    let _ = std::io::Write::flush(&mut std::io::stderr());
                    return;
                }
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "reactor_exit_on_fault",
                    mcu = reactor.mcu_label(),
                    thread_id = ?std::thread::current().id(),
                    "EXIT_ON_FAULT — transport closed via IO error on CRITICAL MCU; aborting klippy so systemd restarts it"
                );
                let _ = std::io::Write::flush(&mut std::io::stderr());
                std::thread::sleep(std::time::Duration::from_millis(100));
                if std::env::var_os("NO_EXIT_ON_FAULT").is_none() {
                    std::process::abort();
                }
            }
        });

        Ok(Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: Some(reactor_handle),
            status_snapshot,
            parser,
            link_health,
            clock,
            raw_identify_bytes,
            is_critical,
            fire_and_forget_depth,
        })
    }

    /// Build a `McuHostIo` from an already-open port, skipping the blocking
    /// identify handshake. The reactor still runs and owns the fd, so this is a
    /// faithful seam for teardown tests (Drop must send Shutdown, join the
    /// reactor, and close the fd) without needing a wire-protocol responder.
    #[cfg(any(test, feature = "test-harness"))]
    pub fn from_port_skip_identify(port_box: Box<dyn serialport::SerialPort>, label: &str) -> Self {
        use crate::host_io::identify::IdentifySeqState;

        let io = crate::host_io::serial_frame_io::SerialFrameIo::new(port_box);
        let parser = Arc::new(MsgProtoParser::new_empty());
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
        let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
        let identify_seq = IdentifySeqState {
            next_send_seq_abs: 1,
            mcu_receive_seq_abs: 0,
        };

        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let link_health = Arc::new(LinkHealth::default());
        let reactor_link_health = Arc::clone(&link_health);
        let reactor_label: Arc<str> = label.into();
        let reactor_clock = Arc::clone(&clock);
        let is_critical = Arc::new(AtomicBool::new(false));
        let fire_and_forget_depth = Arc::new(FireAndForgetDepth::default());
        let reactor_fire_and_forget_depth = Arc::clone(&fire_and_forget_depth);
        let reactor_handle = std::thread::spawn(move || {
            let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
                io,
                crate::host_io::reactor::ReactorSetup {
                    parser: reactor_parser,
                    submission_rx,
                    status_snapshot: reactor_status,
                    seq: identify_seq,
                    mcu_label: reactor_label,
                    link_health: reactor_link_health,
                    fire_and_forget_depth: reactor_fire_and_forget_depth,
                },
                reactor_clock,
            );
            reactor.run();
        });

        Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: Some(reactor_handle),
            status_snapshot,
            parser,
            link_health,
            clock,
            raw_identify_bytes: Vec::new(),
            is_critical,
            fire_and_forget_depth,
        }
    }

    pub fn fire_and_forget_depth(&self) -> &Arc<FireAndForgetDepth> {
        &self.fire_and_forget_depth
    }
}

impl McuHostIo {
    pub fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let payload = self
            .parser
            .encode(cmd)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;
        self.call_payload(payload, expected_response_name, timeout)
    }

    fn call_payload(
        &self,
        payload: Vec<u8>,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let call_id = self.next_call_id.fetch_add(1, Ordering::Relaxed);
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;

        self.submission_tx
            .send(ReactorCommand::Call {
                call_id,
                payload,
                expected_response_name: expected_response_name.to_string(),
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;

        let handle = crate::host_io::call_handle::CallHandle {
            call_id,
            submission_tx: self.submission_tx.clone(),
        };

        match rx.recv_timeout(timeout) {
            Ok(r) => {
                handle.defuse();
                r
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    pub fn is_alive(&self) -> bool {
        self.submission_tx.send(ReactorCommand::Noop).is_ok()
    }

    pub fn set_critical(&self, critical: bool) {
        self.is_critical.store(critical, Ordering::Release);
    }

    pub fn is_critical(&self) -> bool {
        self.is_critical.load(Ordering::Acquire)
    }

    pub fn link_health(&self) -> Arc<LinkHealth> {
        Arc::clone(&self.link_health)
    }

    pub fn attach_heartbeat_callback(&self, cb: Arc<dyn Fn(&[u32], &[u64]) + Send + Sync>) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::AttachHeartbeatCallback(cb));
    }

    pub fn set_mcu_log_hook(&self, hook: Box<dyn Fn(McuLogEvent) + Send + Sync>) {
        let _ = self.submission_tx.send(ReactorCommand::SetMcuLogHook(hook));
    }

    pub fn subscribe_fault(&self) -> Result<std::sync::mpsc::Receiver<FaultEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeFault {
                sender,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_runtime_event_subscription(
        &self,
    ) -> Result<
        (
            std::sync::mpsc::Receiver<RuntimeEvent>,
            std::sync::mpsc::Receiver<RuntimeEvent>,
        ),
        SubscribeError,
    > {
        let (priority_tx, priority_rx) = std::sync::mpsc::sync_channel(RUNTIME_EVENT_CAPACITY);
        let (bulk_tx, bulk_rx) = std::sync::mpsc::sync_channel(RUNTIME_EVENT_BULK_CAPACITY);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeRuntimeEvents {
                priority: priority_tx,
                bulk: bulk_tx,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok((priority_rx, bulk_rx))
    }

    pub fn status(&self) -> std::sync::Arc<crate::host_io::runtime_events::StatusEvent> {
        self.status_snapshot.load_full()
    }

    pub fn raw_identify_bytes(&self) -> &[u8] {
        &self.raw_identify_bytes
    }

    pub fn send_with_response(
        &self,
        cmd: &str,
        response: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.call(cmd, response, timeout)
    }

    pub fn register_frame_interceptor(
        &self,
        msg_name: &str,
        oid: Option<u32>,
        callback: Box<dyn Fn(&crate::transport::MessageParams) + Send + Sync>,
    ) -> Result<InterceptorId, TransportError> {
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::RegisterInterceptor {
                msg_name: msg_name.to_owned(),
                oid,
                callback,
                reply: reply_tx,
            })
            .map_err(|_| TransportError::Closed)?;
        reply_rx.recv().map_err(|_| TransportError::Closed)
    }

    pub fn unregister_frame_interceptor(&self, id: InterceptorId) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::UnregisterInterceptor { id })
            .map_err(|_| TransportError::Closed)
    }

    pub fn send_fire_and_forget(&self, cmd: &str) -> Result<(), TransportError> {
        let payload = self
            .parser
            .encode(cmd)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;
        self.submission_tx
            .send(ReactorCommand::FireAndForget { payload })
            .map_err(|_| TransportError::Closed)
    }

    /// Ask the reactor to encode+send `get_clock` and stamp the matching
    /// "clock" response with CLOCK_MONOTONIC_RAW timestamps (send stamp
    /// captured in the reactor immediately before the wire write). The
    /// response is delivered as a `PassthroughResponse { name: "clock", .. }`
    /// runtime event with `MessageParams::sent_time_raw` / `recv_time_raw`
    /// filled in, so the Python `_bridge_event_poller` can inject them into
    /// `#sent_time` / `#receive_time` before dispatching to `_handle_clock`.
    pub fn get_clock_async(&self) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::GetClockAndDeliver)
            .map_err(|_| TransportError::Closed)
    }

    pub fn mark_expected_disconnect(&self) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::MarkExpectedDisconnect)
            .map_err(|_| TransportError::Closed)
    }

    pub fn send_args<K: AsRef<str>>(
        &self,
        name: &str,
        args: &[(K, crate::host_io::parser::ArgValue)],
    ) -> Result<(), TransportError> {
        let payload = self
            .parser
            .encode_args(name, args)
            .map_err(|e| TransportError::Parse(format!("{name}: {e:?}")))?;
        self.submission_tx
            .send(ReactorCommand::FireAndForget { payload })
            .map_err(|_| TransportError::Closed)
    }

    /// Hand a whole burst of commands to the reactor as one unit so it can
    /// pack them into full message blocks. Sending them one at a time
    /// through `send_args` would let the reactor drain the channel between
    /// them and frame each command on its own block.
    ///
    /// Rejected wholesale — nothing encoded, nothing queued — once the
    /// reactor's fire-and-forget queue is at its high water mark, so a caller
    /// that treats [`TransportError::Backpressure`] as retryable re-offers the
    /// identical burst in the identical order. Any burst the reactor does
    /// accept reaches the wire; the reactor has no discard path.
    ///
    /// Admission claims the burst's block capacity atomically, so concurrent
    /// senders cannot each read the same sub-high-water depth and overshoot
    /// it together. A closed reactor refuses with [`TransportError::Closed`].
    pub fn send_args_batch(
        &self,
        frames: &[(&str, Vec<(String, crate::host_io::parser::ArgValue)>)],
    ) -> Result<(), TransportError> {
        let reserved_blocks = frames.len();
        self.fire_and_forget_depth.reserve(reserved_blocks)?;
        let mut payloads = Vec::with_capacity(frames.len());
        for (name, args) in frames {
            match self.parser.encode_args(name, args) {
                Ok(payload) => payloads.push(payload),
                Err(e) => {
                    self.fire_and_forget_depth.release(reserved_blocks);
                    return Err(TransportError::Parse(format!("{name}: {e:?}")));
                }
            }
        }
        self.submission_tx
            .send(ReactorCommand::FireAndForgetBatch {
                payloads,
                reserved_blocks,
                enqueued_at: std::time::Instant::now(),
            })
            .map_err(|_| {
                self.fire_and_forget_depth.release(reserved_blocks);
                TransportError::Closed
            })
    }

    pub fn call_args<K: AsRef<str>>(
        &self,
        name: &str,
        args: &[(K, crate::host_io::parser::ArgValue)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let payload = self
            .parser
            .encode_args(name, args)
            .map_err(|e| TransportError::Parse(format!("{name}: {e:?}")))?;
        self.call_payload(payload, expected_response_name, timeout)
    }

    pub fn kalico_identify(
        &self,
        timeout: Duration,
    ) -> Result<crate::host_io::mcu_session::IdentifyOutcome, TransportError> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;
        self.submission_tx
            .send(ReactorCommand::McuIdentify {
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;
        match rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    pub fn mcu_call(
        &self,
        kind: mcu_protocol::MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(mcu_protocol::MessageKind, Vec<u8>), TransportError> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;
        self.submission_tx
            .send(ReactorCommand::McuCall {
                kind,
                body,
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;
        match rx.recv_timeout(timeout) {
            Ok(Ok(crate::host_io::mcu_session::McuCallOutcome::Response { kind, body })) => {
                Ok((kind, body))
            }
            Ok(Ok(crate::host_io::mcu_session::McuCallOutcome::Reset)) => {
                Err(TransportError::Closed)
            }
            Ok(Err(e)) => Err(e),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }
}

#[cfg(any(test, feature = "test-harness"))]
impl McuHostIo {
    pub fn from_submission_tx_for_test(
        tx: Sender<ReactorCommand>,
        reactor_handle: Option<std::thread::JoinHandle<()>>,
    ) -> Arc<Self> {
        Arc::new(Self {
            submission_tx: tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle,
            status_snapshot: Arc::new(ArcSwap::from_pointee(
                crate::host_io::runtime_events::StatusEvent::default(),
            )),
            parser: Arc::new(crate::host_io::parser::MsgProtoParser::new_empty()),
            link_health: Arc::new(LinkHealth::default()),
            clock: Arc::new(crate::clock::RealClock),
            raw_identify_bytes: Vec::new(),
            is_critical: Arc::new(AtomicBool::new(false)),
            fire_and_forget_depth: Arc::new(FireAndForgetDepth::default()),
        })
    }
}

#[cfg(test)]
mod test_internals;

#[cfg(test)]
mod fire_and_forget_batch_admission;
