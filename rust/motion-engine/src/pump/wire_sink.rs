use super::{AxisFrame, AxisKey, PieceSink, SendError};
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::Weak;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{LazyLock, Mutex};
use std::time::{Duration, Instant};

/// Per-frame `[transit-diag]` healthy-path logging on the pump send thread
/// throttled delivery below real-time on dense streams (the structured write +
/// `SystemTime` + format ran synchronously between transport pushes). The alert
/// path still fires every frame; the healthy lead sample is emitted once per
/// this many frames to keep coarse delivery-lead telemetry at negligible cost.
const TRANSIT_DIAG_HEALTHY_SAMPLE_STRIDE: u64 = 64;
static TRANSIT_DIAG_FRAME_SEQ: AtomicU64 = AtomicU64::new(0);

/// Previous `PushPieces` frame on one axis, kept solely so the per-frame
/// transit diagnostic can decompose where dispatch lead is being spent:
/// `Δarrival_lead = schedule_advance − mcu_clock_advance`. Separating the
/// host's planned advance (`front_start_time` delta) from the MCU clock's
/// real-time advance (`arrival_clock` delta), and bracketing both with the
/// wall-clock send gap and the blocking-call duration, tells late-arrival
/// apart from host-pacing starvation, transport stall, and MCU clock burn.
struct PrevTransitFrame {
    send_instant: Instant,
    front_start_time: u64,
    arrival_clock: u64,
    arrival_lead_ticks: i64,
}

static TRANSIT_PREV_FRAME: LazyLock<Mutex<HashMap<AxisKey, PrevTransitFrame>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

pub enum McuTransport {
    Serial(Weak<host_rt::host_io::McuHostIo>),
    EtherCat(Weak<host_rt::mcu_serial_conn::McuSerialConn>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
        }
    }
}

pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
    pub freq_of: Arc<dyn Fn(u32) -> Option<f64> + Send + Sync>,
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
/// but also run under the much deeper `MAX_LEAD_SECS` lead. Recovers isolated
/// corruption; sustained corruption gives up fast to the loud in-past-guard
/// backstop.
const PUSHPIECES_MAX_ATTEMPTS: u32 = 3;

/// Bounded re-request policy for serial `PushPieces` (the frame is idempotent on
/// the real MCU: slot-addressed write + absolute `commit_head`, stale = no-op).
/// `attempt_call` performs one request/response with the short per-attempt
/// timeout. Returns `Ok` on the first success; `Fatal` (no further attempts) on a
/// genuine MCU failure (`Closed`/`Io` dead transport, `McuShutdown`) so it surfaces
/// loud instead of being buried under the retry budget; `Transient` once the budget
/// is spent on recoverable corruption so the existing pump path + in-past guard
/// remain the backstop. Pure (no I/O of its own) so the policy is unit-testable
/// with a scripted `attempt_call`.
pub(crate) fn pushpieces_retransmit_serial<F>(
    mcu_id: u32,
    max_attempts: u32,
    mut attempt_call: F,
) -> Result<Vec<u8>, SendError>
where
    F: FnMut() -> Result<Vec<u8>, host_rt::transport::TransportError>,
{
    use host_rt::transport::TransportError;
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
    fn call_push_pieces(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<mcu_protocol::messages::PushPiecesResponse, SendError> {
        use host_rt::transport::TransportError;

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
        let body = mcu_protocol::codec::Encode::encoded_to_vec(&msg);

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
                pushpieces_retransmit_serial(mcu_id, PUSHPIECES_MAX_ATTEMPTS, || {
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
        };

        use mcu_protocol::codec::Decode as _;
        mcu_protocol::messages::PushPiecesResponse::decode(&resp_body).map_err(|e| {
            SendError::Transient(format!("decode PushPiecesResponse mcu {mcu_id}: {e:?}"))
        })
    }

    /// Emit the per-axis transit diagnostic for one axis of a just-completed
    /// frame. `front_start_time` is this axis' echo from the response;
    /// `arrival_clock` is the frame-global MCU clock; `send_started_at` /
    /// `send_elapsed_us` belong to the whole MCU round-trip.
    #[allow(clippy::too_many_arguments)]
    fn emit_transit_diag(
        &self,
        key: AxisKey,
        host_front_start_time: u64,
        piece_count: usize,
        room: u32,
        send_started_at: Instant,
        send_elapsed_us: f64,
        front_start_time: u64,
        arrival_clock: u64,
    ) {
        let arrival_lead_ticks = front_start_time as i64 - arrival_clock as i64;
        let zero_st = host_front_start_time == 0;
        let past_arrival = arrival_lead_ticks < 0;
        let frame_seq = TRANSIT_DIAG_FRAME_SEQ.fetch_add(1, Ordering::Relaxed);
        let healthy_sample = frame_seq % TRANSIT_DIAG_HEALTHY_SAMPLE_STRIDE == 0;

        let prev = {
            let mut map = TRANSIT_PREV_FRAME.lock().expect("transit prev-frame map");
            map.insert(
                key,
                PrevTransitFrame {
                    send_instant: send_started_at,
                    front_start_time,
                    arrival_clock,
                    arrival_lead_ticks,
                },
            )
        };

        if !(zero_st || past_arrival || healthy_sample) {
            return;
        }
        let approx_freq_hz = (self.freq_of)(key.mcu_id);
        let host_send_secs = {
            use std::time::{SystemTime, UNIX_EPOCH};
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0)
        };
        // Clock not yet synced -> the µs conversion is meaningless; render N/A.
        // Alert gating uses arrival_lead_ticks (tick domain), so the ALERT still
        // fires without a frequency.
        let arrival_lead_us = approx_freq_hz
            .map(|f| format!("{:.1}", (arrival_lead_ticks as f64 / f) * 1e6))
            .unwrap_or_else(|| "N/A".to_owned());

        let ticks_to_us = |ticks: i64| {
            approx_freq_hz
                .map(|f| format!("{:.1}", (ticks as f64 / f) * 1e6))
                .unwrap_or_else(|| "N/A".to_owned())
        };
        // Where did the lead go since this axis' previous frame?
        //   Δarrival_lead = schedule_advance − mcu_clock_advance
        // schedule_advance: how much further ahead the host planned.
        // mcu_clock_advance: how much the MCU clock (real time) moved.
        // send_gap: wall time between our two sends to this axis.
        let (send_gap_us, schedule_advance_us, mcu_clock_advance_us, delta_arrival_lead_us) =
            match &prev {
                Some(p) => (
                    format!(
                        "{:.1}",
                        send_started_at.duration_since(p.send_instant).as_secs_f64() * 1e6
                    ),
                    ticks_to_us(front_start_time as i64 - p.front_start_time as i64),
                    ticks_to_us(arrival_clock as i64 - p.arrival_clock as i64),
                    ticks_to_us(arrival_lead_ticks - p.arrival_lead_ticks),
                ),
                None => (
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                ),
            };
        if zero_st || past_arrival {
            let alert = if zero_st && past_arrival {
                "host_start_time=0 (clock-sync gap) AND piece in MCU past"
            } else if zero_st {
                "host_start_time=0 (router clock_freq=0 at dispatch — clock-sync gap)"
            } else {
                "piece arrived in MCU past (arrival_lead<0) — PieceStartInPast risk"
            };
            tracing::warn!(
                subsystem = "motion",
                event = "transit_diag_alert",
                mcu = key.mcu_id,
                axis = key.axis,
                host_front_start_time,
                mcu_front_start_time = front_start_time,
                arrival_clock,
                arrival_lead_ticks,
                arrival_lead_us = %arrival_lead_us,
                host_send_unix_secs = host_send_secs,
                send_elapsed_us,
                send_gap_us = %send_gap_us,
                schedule_advance_us = %schedule_advance_us,
                mcu_clock_advance_us = %mcu_clock_advance_us,
                delta_arrival_lead_us = %delta_arrival_lead_us,
                piece_count,
                room,
                alert,
                "[transit-diag] alert"
            );
        } else {
            tracing::info!(
                subsystem = "motion",
                event = "transit_diag",
                mcu = key.mcu_id,
                axis = key.axis,
                host_front_start_time,
                mcu_front_start_time = front_start_time,
                arrival_clock,
                arrival_lead_ticks,
                arrival_lead_us = %arrival_lead_us,
                host_send_unix_secs = host_send_secs,
                send_elapsed_us,
                send_gap_us = %send_gap_us,
                schedule_advance_us = %schedule_advance_us,
                mcu_clock_advance_us = %mcu_clock_advance_us,
                delta_arrival_lead_us = %delta_arrival_lead_us,
                piece_count,
                room,
                "[transit-diag]"
            );
        }
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
        };
        self.send_mcu_frames(key.mcu_id, std::slice::from_ref(&frame))
            .map(|()| mcu_protocol::result_codes::OK)
    }

    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        debug_assert!(
            frames.iter().all(|f| f.pieces.len() <= 255),
            "PushPieces axis block exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        let send_started_at = Instant::now();
        let resp = self.call_push_pieces(mcu_id, frames)?;
        let send_elapsed_us = send_started_at.elapsed().as_secs_f64() * 1e6;

        // Per-axis transit-diag from the response's per-axis echo against the
        // frame-global arrival clock. Emitted even on a fatal frame — a negative
        // arrival_lead is exactly the PieceStartInPast signature we want logged.
        for f in frames {
            let Some(diag) = resp.axes.iter().find(|a| a.axis_idx == f.axis) else {
                continue;
            };
            let key = AxisKey {
                mcu_id,
                axis: f.axis,
            };
            let host_front_start_time = f.pieces.first().map(|p| p.start_time).unwrap_or(0);
            self.emit_transit_diag(
                key,
                host_front_start_time,
                f.pieces.len(),
                f.room,
                send_started_at,
                send_elapsed_us,
                diag.front_start_time,
                resp.arrival_clock,
            );
        }

        if resp.result != mcu_protocol::result_codes::OK {
            return Err(SendError::retryable_mcu_reject(mcu_id, resp.result));
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "wire_sink_tests.rs"]
mod wire_sink_tests;
