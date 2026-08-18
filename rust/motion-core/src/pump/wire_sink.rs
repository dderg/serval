use super::stepcompress_sink::StepcompressEndpoint;
use super::{AxisFrame, AxisKey, PieceSink, SendError};
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::Weak;
use std::time::{Duration, Instant};

pub enum McuTransport {
    Serial(Weak<host_rt::host_io::McuHostIo>),
    EtherCat(Weak<host_rt::mcu_serial_conn::McuSerialConn>),
    Stepcompress(Arc<Mutex<StepcompressEndpoint>>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
            Self::Stepcompress(_) => write!(f, "McuTransport::Stepcompress"),
        }
    }
}

pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
    /// Current (mcu_now_ticks, freq_hz) from the clock regression — the same
    /// record the pump's in-past guard reads. Used for transit diagnostics
    /// and to cap the PushPieces retry budget by the front piece's remaining
    /// scheduling lead.
    pub clock_of: Arc<dyn Fn(u32) -> Option<(u64, f64)> + Send + Sync>,
    /// Per-transaction budget for serial MCUs; `[printer] pieces_wire_budget`
    /// overrides the UART-sized default so USB transports can amortize their
    /// per-transaction round trip over bigger frames.
    pub serial_limits: super::BundleLimits,
    /// Bundles the pump may keep in flight per serial MCU
    /// (`[printer] pieces_inflight`). 1 = classic stop-and-wait.
    pub serial_window: usize,
}

/// Trace-relevant slice of one submitted frame, retained by the pending
/// handle so the transit record is written when the response arrives.
struct FrameTraceMeta {
    axis: u8,
    piece_count: u32,
    room: u32,
    guard_recorded_ns: u64,
    guard_mcu_clock: u64,
    host_front_start_time: u64,
}

/// In-flight serial `PushPieces`: resolves from the reactor's correlation
/// map; the reactor completes it with `TransportError::Timeout` at the
/// attempt deadline, so `poll`/`wait` never hang past it.
struct SerialPendingSend {
    mcu_id: u32,
    rx: std::sync::mpsc::Receiver<
        Result<host_rt::host_io::mcu_session::McuCallOutcome, host_rt::transport::TransportError>,
    >,
    deadline: Instant,
    send_started_ns: u64,
    send_started_at: Instant,
    frames: Vec<FrameTraceMeta>,
    resolved: bool,
}

impl SerialPendingSend {
    fn resolve(
        &mut self,
        outcome: Result<
            Result<
                host_rt::host_io::mcu_session::McuCallOutcome,
                host_rt::transport::TransportError,
            >,
            std::sync::mpsc::RecvTimeoutError,
        >,
    ) -> Option<Result<(), SendError>> {
        use host_rt::host_io::mcu_session::McuCallOutcome;
        use host_rt::transport::TransportError;
        let call_result = match outcome {
            Ok(Ok(McuCallOutcome::Response { kind: _, body })) => Ok(body),
            Ok(Ok(McuCallOutcome::Reset)) => Err(SendError::Fatal(format!(
                "serial PushPieces mcu {}: transport reset mid-call",
                self.mcu_id
            ))),
            Ok(Err(
                e @ (TransportError::Closed
                | TransportError::Io(_)
                | TransportError::McuShutdown(_)),
            )) => Err(SendError::Fatal(format!(
                "serial PushPieces mcu {}: {e:?}",
                self.mcu_id
            ))),
            Ok(Err(e)) => Err(SendError::Transient(format!(
                "serial PushPieces mcu {}: {e:?}",
                self.mcu_id
            ))),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                if Instant::now() < self.deadline {
                    return None;
                }
                Err(SendError::Transient(format!(
                    "serial PushPieces mcu {}: no response within the attempt deadline",
                    self.mcu_id
                )))
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(SendError::Fatal(format!(
                "serial PushPieces mcu {}: reactor gone",
                self.mcu_id
            ))),
        };
        self.resolved = true;
        let send_elapsed_ns = self.send_started_at.elapsed().as_nanos() as u64;
        let body = match call_result {
            Ok(body) => body,
            Err(error) => {
                record_transport_error_traces(
                    self.mcu_id,
                    &self.frames,
                    self.send_started_ns,
                    send_elapsed_ns,
                );
                return Some(Err(error));
            }
        };
        use mcu_protocol::codec::Decode as _;
        let response = match mcu_protocol::messages::PushPiecesResponse::decode(&body) {
            Ok(r) => r,
            Err(e) => {
                record_transport_error_traces(
                    self.mcu_id,
                    &self.frames,
                    self.send_started_ns,
                    send_elapsed_ns,
                );
                return Some(Err(SendError::Transient(format!(
                    "decode PushPiecesResponse mcu {}: {e:?}",
                    self.mcu_id
                ))));
            }
        };
        Some(record_response_traces(
            self.mcu_id,
            &self.frames,
            self.send_started_ns,
            send_elapsed_ns,
            &response,
        ))
    }
}

impl super::PendingSend for SerialPendingSend {
    fn poll(&mut self) -> Option<Result<(), SendError>> {
        assert!(!self.resolved, "PendingSend polled after resolution");
        match self.rx.try_recv() {
            Ok(outcome) => self.resolve(Ok(outcome)),
            Err(std::sync::mpsc::TryRecvError::Empty) => {
                if Instant::now() >= self.deadline {
                    self.resolve(Err(std::sync::mpsc::RecvTimeoutError::Timeout))
                } else {
                    None
                }
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.resolve(Err(std::sync::mpsc::RecvTimeoutError::Disconnected))
            }
        }
    }

    fn wait(&mut self, cap: Duration) -> Option<Result<(), SendError>> {
        assert!(!self.resolved, "PendingSend waited after resolution");
        let until_deadline = self.deadline.saturating_duration_since(Instant::now());
        let outcome = self
            .rx
            .recv_timeout(cap.min(until_deadline).max(Duration::from_millis(1)));
        self.resolve(outcome)
    }
}

fn record_transport_error_traces(
    mcu_id: u32,
    frames: &[FrameTraceMeta],
    send_started_ns: u64,
    send_elapsed_ns: u64,
) {
    for frame in frames {
        super::transit_trace::record(super::transit_trace::TransitTraceRecord {
            sequence: 0,
            mcu_id,
            axis: frame.axis,
            piece_count: frame.piece_count,
            room: frame.room,
            guard_recorded_ns: frame.guard_recorded_ns,
            guard_mcu_clock: frame.guard_mcu_clock,
            send_started_ns,
            send_elapsed_ns,
            host_front_start_time: frame.host_front_start_time,
            mcu_front_start_time: 0,
            arrival_clock: 0,
            result: super::transit_trace::transport_error_result(),
        });
    }
    super::transit_trace::emit_result_fault_snapshot(
        "transport_error",
        super::transit_trace::transport_error_result(),
    );
}

fn record_response_traces(
    mcu_id: u32,
    frames: &[FrameTraceMeta],
    send_started_ns: u64,
    send_elapsed_ns: u64,
    response: &mcu_protocol::messages::PushPiecesResponse,
) -> Result<(), SendError> {
    let result = response.result;
    for frame in frames {
        let mcu_front_start_time = response
            .axes
            .iter()
            .find(|axis| axis.axis_idx == frame.axis)
            .map_or(0, |axis| axis.front_start_time);
        super::transit_trace::record(super::transit_trace::TransitTraceRecord {
            sequence: 0,
            mcu_id,
            axis: frame.axis,
            piece_count: frame.piece_count,
            room: frame.room,
            guard_recorded_ns: frame.guard_recorded_ns,
            guard_mcu_clock: frame.guard_mcu_clock,
            send_started_ns,
            send_elapsed_ns,
            host_front_start_time: frame.host_front_start_time,
            mcu_front_start_time,
            arrival_clock: response.arrival_clock,
            result,
        });
    }
    if result != mcu_protocol::result_codes::OK {
        super::transit_trace::emit_result_fault_snapshot("mcu_reject", result);
        return Err(SendError::mcu_reject(mcu_id, result));
    }
    Ok(())
}

/// Per-attempt response wait for a serial `PushPieces` re-request, before the
/// frame's own wire time is added. A few × the small-frame ~6 ms RTT, so a
/// healthy response still returns on arrival while a lost one is re-requested
/// fast.
const PUSHPIECES_ATTEMPT_TIMEOUT: Duration = Duration::from_millis(30);

/// Worst-case one-way serial cost per frame byte (250 kbaud, 10 bits/byte),
/// doubled for the echo path. A batched frame (up to `MAX_PER_FRAME` pieces per
/// axis, ~2 KiB) legitimately spends tens of milliseconds on the wire; the
/// attempt timeout must grow with the frame or every large frame times out and
/// retransmits.
const SERIAL_ROUND_TRIP_PER_BYTE: Duration = Duration::from_micros(80);

fn pushpieces_attempt_timeout(body_len: usize) -> Duration {
    PUSHPIECES_ATTEMPT_TIMEOUT + SERIAL_ROUND_TRIP_PER_BYTE * body_len as u32
}

/// Total serial `PushPieces` re-requests. The whole burst blocks the
/// single-threaded pump, so for drip-sized frames the budget
/// (`* PUSHPIECES_ATTEMPT_TIMEOUT` ≈ 90 ms) must stay under the drip lead
/// (`DRIP_WINDOW_SECS` = 100 ms) — otherwise the retry itself starves the MCU
/// of later pieces and recreates the very `-308` it prevents. Batched print
/// frames get a larger per-attempt timeout (wire time scales with frame size)
/// but also run under the much deeper `MAX_LEAD_SECS` lead — the attempt-count
/// cap alone can exceed a shallow post-re-anchor lead (250 ms), so the retry
/// loop is additionally capped by the front piece's remaining lead (`deadline`).
/// Recovers isolated corruption; sustained corruption gives up fast to the
/// loud in-past-guard backstop.
const PUSHPIECES_MAX_ATTEMPTS: u32 = 3;

/// Bounded re-request policy for serial `PushPieces` (the frame is idempotent on
/// the real MCU: slot-addressed write + absolute `commit_head`, stale = no-op).
/// `attempt_call` performs one request/response with the short per-attempt
/// timeout. Returns `Ok` on the first success; `Fatal` (no further attempts) on a
/// genuine MCU failure (`Closed`/`Io` dead transport, `McuShutdown`) so it surfaces
/// loud instead of being buried under the retry budget; `Transient` once the budget
/// is spent on recoverable corruption so the existing pump path + in-past guard
/// remain the backstop. `deadline` is when the bundle's front piece enters the
/// MCU's past: retrying beyond it cannot succeed, so the loop stops there and
/// names the transport as unresponsive instead of burning the remaining budget.
/// Pure (no I/O of its own) so the policy is unit-testable with a scripted
/// `attempt_call`.
pub(crate) fn pushpieces_retransmit_serial<F>(
    mcu_id: u32,
    max_attempts: u32,
    deadline: Option<Instant>,
    mut attempt_call: F,
) -> Result<Vec<u8>, SendError>
where
    F: FnMut() -> Result<Vec<u8>, host_rt::transport::TransportError>,
{
    use host_rt::transport::TransportError;
    let started = Instant::now();
    let mut attempt: u32 = 0;
    loop {
        attempt += 1;
        match attempt_call() {
            Ok(b) => return Ok(b),
            Err(
                e @ (TransportError::Closed
                | TransportError::Io(_)
                | TransportError::McuShutdown(_)),
            ) => {
                return Err(SendError::Fatal(format!(
                    "serial PushPieces mcu {mcu_id}: {e:?}"
                )));
            }
            Err(e) => {
                if deadline.is_some_and(|d| Instant::now() >= d) {
                    let unresponsive_ms = started.elapsed().as_millis() as u64;
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "pushpieces_giveup_lead_expired",
                        mcu = mcu_id,
                        attempts = attempt,
                        unresponsive_ms,
                        error = ?e,
                        "[pushpieces] transport unresponsive through the front piece's entire scheduling lead — giving up; in-past guard aborts next"
                    );
                    return Err(SendError::Transient(format!(
                        "serial PushPieces mcu {mcu_id}: transport unresponsive for \
                         {unresponsive_ms} ms — no response within the front piece's \
                         remaining lead after {attempt} attempts ({e:?})"
                    )));
                }
                if attempt >= max_attempts {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "pushpieces_giveup",
                        mcu = mcu_id,
                        attempts = attempt,
                        error = ?e,
                        "[pushpieces] budget exhausted — giving up (serial); in-past guard is the backstop"
                    );
                    return Err(SendError::Transient(format!(
                        "serial PushPieces mcu {mcu_id}: no response after {attempt} attempts ({e:?})"
                    )));
                }
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "pushpieces_retry",
                    mcu = mcu_id,
                    attempt,
                    max_attempts,
                    error = ?e,
                    "[pushpieces] no/lost response — re-requesting idempotent frame (serial)"
                );
            }
        }
    }
}

impl WireSink {
    pub fn stepcompress_ring_depth(&self, mcu_id: u32) -> Option<u32> {
        match self.transports.get(&mcu_id)? {
            McuTransport::Stepcompress(endpoint) => Some(
                endpoint
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .ring_depth(),
            ),
            _ => None,
        }
    }

    /// The wall-clock instant at which the bundle's earliest piece enters the
    /// MCU's past — retrying a send beyond it cannot succeed. `None` when the
    /// clock regression has no record yet (attempt-count cap still applies).
    fn front_lead_deadline(&self, mcu_id: u32, frames: &[AxisFrame]) -> Option<Instant> {
        let front = frames
            .iter()
            .filter_map(|f| f.pieces.first())
            .map(|p| p.start_time)
            .min()?;
        let (mcu_now, freq) = (self.clock_of)(mcu_id)?;
        if freq <= 0.0 {
            return None;
        }
        let lead_secs = front.saturating_sub(mcu_now) as f64 / freq;
        Some(Instant::now() + Duration::from_secs_f64(lead_secs))
    }

    fn encode_push_pieces(frames: &[AxisFrame]) -> Vec<u8> {
        let axes: Vec<mcu_protocol::messages::AxisPieces> = frames
            .iter()
            .map(|f| {
                let mut pieces_bytes =
                    Vec::with_capacity(f.pieces.len() * runtime::piece_ring::PIECE_ENTRY_BYTES);
                for p in &f.pieces {
                    p.to_wire_bytes(&mut pieces_bytes);
                }
                mcu_protocol::messages::AxisPieces {
                    axis_idx: f.axis,
                    piece_count: f.pieces.len() as u8,
                    start_slot: f.start_slot,
                    new_head: f.new_head,
                    pieces_bytes,
                }
            })
            .collect();
        let msg = mcu_protocol::messages::PushPieces { axes };
        mcu_protocol::codec::Encode::encoded_to_vec(&msg)
    }

    fn frame_trace_meta(frames: &[AxisFrame]) -> Vec<FrameTraceMeta> {
        frames
            .iter()
            .map(|f| FrameTraceMeta {
                axis: f.axis,
                piece_count: f.pieces.len() as u32,
                room: f.room,
                guard_recorded_ns: f.guard_recorded_ns,
                guard_mcu_clock: f.guard_mcu_clock,
                host_front_start_time: f.pieces.first().map_or(0, |piece| piece.start_time),
            })
            .collect()
    }

    fn call_push_pieces(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<mcu_protocol::messages::PushPiecesResponse, SendError> {
        use host_rt::transport::TransportError;

        let body = Self::encode_push_pieces(frames);

        let transport = self.transports.get(&mcu_id).ok_or_else(|| {
            SendError::Transient(format!(
                "WireSink: no transport for mcu_id {mcu_id}; \
                     this is a logic bug in init_planner — the MCU was enqueued \
                     without registering its transport"
            ))
        })?;

        let resp_body = match transport {
            McuTransport::Serial(weak) => {
                let io = weak.upgrade().ok_or_else(|| {
                    SendError::Transient(format!("McuHostIo for mcu {mcu_id} detached"))
                })?;
                let attempt_timeout = pushpieces_attempt_timeout(body.len());
                let deadline = self.front_lead_deadline(mcu_id, frames);
                pushpieces_retransmit_serial(mcu_id, PUSHPIECES_MAX_ATTEMPTS, deadline, || {
                    io.kalico_call_on_channel(
                        mcu_protocol::MCU_CHANNEL_PIECES,
                        mcu_protocol::MessageKind::PushPieces,
                        body.clone(),
                        attempt_timeout,
                    )
                    .map(|(_kind, b)| b)
                })?
            }
            McuTransport::EtherCat(weak) => {
                let conn = weak.upgrade().ok_or_else(|| {
                    SendError::Fatal(format!(
                        "ethercat conn for mcu {mcu_id} detached (released)"
                    ))
                })?;
                let (_kind, b) = conn
                    .kalico_call_on_channel(
                        mcu_protocol::MCU_CHANNEL_PIECES,
                        mcu_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| {
                        if matches!(&e, TransportError::Closed | TransportError::Io(_)) {
                            SendError::Fatal(format!("ethercat PushPieces mcu {mcu_id}: {e:?}"))
                        } else {
                            SendError::Transient(format!("ethercat PushPieces mcu {mcu_id}: {e:?}"))
                        }
                    })?;
                b
            }
            McuTransport::Stepcompress(_) => {
                return Err(SendError::Fatal(format!(
                    "PushPieces attempted on stepcompress mcu {mcu_id}"
                )));
            }
        };

        use mcu_protocol::codec::Decode as _;
        mcu_protocol::messages::PushPiecesResponse::decode(&resp_body).map_err(|e| {
            SendError::Transient(format!("decode PushPiecesResponse mcu {mcu_id}: {e:?}"))
        })
    }
}

impl PieceSink for WireSink {
    /// Single-axis convenience — the pump drives WireSink via `send_mcu_frames`;
    /// this exists only to satisfy the trait and routes through the same path.
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[runtime::piece_ring::PieceEntry],
        start_slot: u16,
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError> {
        let frame = AxisFrame {
            axis: key.axis,
            pieces: pieces.to_vec(),
            start_slot,
            new_head,
            room,
            guard_recorded_ns: 0,
            guard_mcu_clock: 0,
        };
        self.send_mcu_frames(key.mcu_id, std::slice::from_ref(&frame))
            .map(|()| mcu_protocol::result_codes::OK)
    }

    fn bundle_limits(&self, mcu_id: u32) -> super::BundleLimits {
        match self.transports.get(&mcu_id) {
            Some(McuTransport::EtherCat(_)) => super::BundleLimits {
                wire_budget: 8192,
                pieces_per_axis: 255,
            },
            Some(McuTransport::Serial(_) | McuTransport::Stepcompress(_)) | None => {
                self.serial_limits
            }
        }
    }

    fn send_window(&self, mcu_id: u32) -> usize {
        match self.transports.get(&mcu_id) {
            Some(McuTransport::Serial(_)) => self.serial_window.max(1),
            _ => 1,
        }
    }

    fn submit_mcu_frames(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<Box<dyn super::PendingSend>, SendError> {
        let Some(McuTransport::Serial(weak)) = self.transports.get(&mcu_id) else {
            let outcome = self.send_mcu_frames(mcu_id, frames);
            return Ok(Box::new(super::messages::ResolvedSend(Some(outcome))));
        };
        let io = weak
            .upgrade()
            .ok_or_else(|| SendError::Fatal(format!("McuHostIo for mcu {mcu_id} detached")))?;
        let body = Self::encode_push_pieces(frames);
        let attempt_timeout = pushpieces_attempt_timeout(body.len());
        let send_started_ns = super::transit_trace::trace_now_ns();
        let send_started_at = Instant::now();
        let rx = io
            .kalico_submit_on_channel(
                mcu_protocol::MCU_CHANNEL_PIECES,
                mcu_protocol::MessageKind::PushPieces,
                body,
                attempt_timeout,
            )
            .map_err(|e| SendError::Fatal(format!("serial PushPieces mcu {mcu_id}: {e:?}")))?;
        Ok(Box::new(SerialPendingSend {
            mcu_id,
            rx,
            deadline: send_started_at + attempt_timeout,
            send_started_ns,
            send_started_at,
            frames: Self::frame_trace_meta(frames),
            resolved: false,
        }))
    }

    fn mark_reanchor(&self, key: AxisKey, at_start_clock: u64, epoch_freq: Option<f64>) {
        if let Some(McuTransport::Stepcompress(endpoint)) = self.transports.get(&key.mcu_id) {
            endpoint
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .mark_reanchor(key.axis, at_start_clock, epoch_freq);
        }
    }

    fn mark_seam_gap(&self, key: AxisKey, at_start_clock: u64) {
        if let Some(McuTransport::Stepcompress(endpoint)) = self.transports.get(&key.mcu_id) {
            endpoint
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .mark_seam_gap(key.axis, at_start_clock);
        }
    }

    fn seam_basis(&self, key: AxisKey) -> Option<super::sched::SeamBasis> {
        match self.transports.get(&key.mcu_id) {
            Some(McuTransport::Stepcompress(endpoint)) => endpoint
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .seam_basis(key.axis),
            _ => None,
        }
    }

    fn on_barrier_ack(&self, mcu_id: u32, oid: u8, seq: u32) -> Result<(), SendError> {
        match self.transports.get(&mcu_id) {
            Some(McuTransport::Stepcompress(endpoint)) => endpoint
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .on_barrier_ack(u32::from(oid), seq),
            _ => Err(SendError::Fatal(format!(
                "stepcompress_barrier_ack oid={oid} seq={seq} arrived for mcu {mcu_id}, which \
                 has no stepcompress endpoint"
            ))),
        }
    }

    fn flush_keys(&self, keys: &[AxisKey]) -> Result<(), SendError> {
        let mut axes_by_mcu: HashMap<u32, Vec<u8>> = HashMap::new();
        for key in keys {
            axes_by_mcu.entry(key.mcu_id).or_default().push(key.axis);
        }
        for (mcu_id, axes) in axes_by_mcu {
            if let Some(McuTransport::Stepcompress(endpoint)) = self.transports.get(&mcu_id) {
                endpoint
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .abort_axes(&axes)?;
            }
        }
        Ok(())
    }

    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        debug_assert!(
            frames.iter().all(|f| f.pieces.len() <= 255),
            "PushPieces axis block exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        if let Some(McuTransport::Stepcompress(endpoint)) = self.transports.get(&mcu_id) {
            return endpoint
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .send_frames(mcu_id, frames);
        }

        let send_started_ns = super::transit_trace::trace_now_ns();
        let send_started_at = Instant::now();
        let response = self.call_push_pieces(mcu_id, frames);
        let send_elapsed_ns = send_started_at.elapsed().as_nanos() as u64;
        let response = match response {
            Ok(response) => response,
            Err(error) => {
                for frame in frames {
                    super::transit_trace::record(super::transit_trace::TransitTraceRecord {
                        sequence: 0,
                        mcu_id,
                        axis: frame.axis,
                        piece_count: frame.pieces.len() as u32,
                        room: frame.room,
                        guard_recorded_ns: frame.guard_recorded_ns,
                        guard_mcu_clock: frame.guard_mcu_clock,
                        send_started_ns,
                        send_elapsed_ns,
                        host_front_start_time: frame
                            .pieces
                            .first()
                            .map_or(0, |piece| piece.start_time),
                        mcu_front_start_time: 0,
                        arrival_clock: 0,
                        result: super::transit_trace::transport_error_result(),
                    });
                }
                super::transit_trace::emit_result_fault_snapshot(
                    "transport_error",
                    super::transit_trace::transport_error_result(),
                );
                return Err(error);
            }
        };
        let result = response.result;
        for frame in frames {
            let mcu_front_start_time = response
                .axes
                .iter()
                .find(|axis| axis.axis_idx == frame.axis)
                .map_or(0, |axis| axis.front_start_time);
            super::transit_trace::record(super::transit_trace::TransitTraceRecord {
                sequence: 0,
                mcu_id,
                axis: frame.axis,
                piece_count: frame.pieces.len() as u32,
                room: frame.room,
                guard_recorded_ns: frame.guard_recorded_ns,
                guard_mcu_clock: frame.guard_mcu_clock,
                send_started_ns,
                send_elapsed_ns,
                host_front_start_time: frame.pieces.first().map_or(0, |piece| piece.start_time),
                mcu_front_start_time,
                arrival_clock: response.arrival_clock,
                result,
            });
        }
        if result != mcu_protocol::result_codes::OK {
            super::transit_trace::emit_result_fault_snapshot("mcu_reject", result);
            return Err(SendError::mcu_reject(mcu_id, result));
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "wire_sink_tests.rs"]
mod wire_sink_tests;
