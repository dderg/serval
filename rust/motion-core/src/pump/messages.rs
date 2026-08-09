use crate::lock_ext::LockExt;
use std::sync::Arc;

use runtime::piece_ring::PieceEntry;

use super::drip::DripArm;
use super::sched::AxisFrame;
use crate::types::AxisKey;

pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<(PieceEntry, f64)>,
    pub epoch: crate::anchor::StreamEpoch,
    pub lead_secs: f64,
    pub source_line: u32,
    pub epoch_freq: Option<f64>,
}

/// Records each piece into the motion-history store at the moment it is
/// accepted by the MCU, so the store mirrors what the MCU can actually
/// execute. Recording at dispatch time instead would flood the ring with an
/// entire move up front — a long homing move evicts its own start before the
/// endstop trip is resolved against it.
///
/// A piece carries its span in seconds, so placing its end on the MCU clock
/// needs the rate that clock actually runs at — the same measured rate the
/// producer spaced the start clocks with, and the executor turns the span
/// back into ticks with. The configured crystal is only a stand-in for the
/// window before the first sync estimate lands: on a board whose measured
/// rate drifts from its nameplate, keying the history off the nameplate
/// stretches every piece and drags trip reconstruction a velocity-scaled
/// distance away from where the axis really is.
pub struct HistoryRecorder {
    pub store: Arc<std::sync::Mutex<crate::motion_history::HistoryStore>>,
    pub nominal_freqs: Arc<std::sync::Mutex<std::collections::HashMap<u32, u32>>>,
}

impl HistoryRecorder {
    pub(super) fn record(
        &self,
        key: AxisKey,
        piece: &PieceEntry,
        measured_freq_hz: Option<f64>,
        host_t: f64,
    ) {
        let clock_freq_hz = match measured_freq_hz {
            Some(freq) if freq.is_finite() && freq > 0.0 => freq,
            _ => {
                let nominal = *self
                    .nominal_freqs
                    .lock_ok()
                    .get(&key.mcu_id)
                    .unwrap_or_else(|| {
                        panic!(
                            "no nominal clock frequency registered for mcu {} \
                             — set_nominal_clock_freq was not called before streaming",
                            key.mcu_id
                        )
                    });
                tracing::warn!(
                    subsystem = "motion",
                    event = "history_clock_freq_unmeasured",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    nominal,
                    "[history] no measured clock rate for this mcu — keying the piece \
                     off the configured crystal"
                );
                f64::from(nominal)
            }
        };
        self.store
            .lock_ok()
            .record(key, piece, clock_freq_hz, host_t);
    }
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    /// Retired piece counts indexed by AXIS, not by the reporting endpoint's
    /// motor/slot order — the pump keys its queues by `AxisKey`. Endpoints
    /// whose native counters are motor- or slot-indexed re-index first.
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Heartbeat(HeartbeatMsg),
    Flush(Vec<AxisKey>),
    Halt {
        keys: Vec<AxisKey>,
        ack: std::sync::mpsc::SyncSender<()>,
    },
    Resume(Vec<AxisKey>),
    DripArm(DripArm),
    DripDisarm(u64),
    StepcompressBarrierAck {
        mcu_id: u32,
        oid: u8,
        seq: u32,
    },
    Barrier(std::sync::mpsc::SyncSender<()>),
    Shutdown,
}

#[derive(Debug)]
pub enum SendError {
    Fatal(String),
    Halted(String),
    Transient(String),
}

impl SendError {
    pub(super) fn mcu_reject(mcu_id: u32, result: i32) -> Self {
        let message = format!("mcu {mcu_id} rejected PushPieces frame: result {result}");
        let halted = result == mcu_protocol::result_codes::STREAM_HALTED
            || result == mcu_protocol::result_codes::EC_PIECES_WHILE_HALTED;
        if halted {
            Self::Halted(message)
        } else {
            Self::Transient(message)
        }
    }
}

impl std::fmt::Display for SendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fatal(s) => write!(f, "fatal: {s}"),
            Self::Halted(s) => write!(f, "halted: {s}"),
            Self::Transient(s) => write!(f, "transient: {s}"),
        }
    }
}

/// Per-transaction limits the scheduler must respect for one MCU's transport.
#[derive(Clone, Copy, Debug)]
pub struct BundleLimits {
    pub wire_budget: usize,
    pub pieces_per_axis: usize,
}

/// Conservative default: a 1 KiB frame is ~20 ms of wire at 500 kbaud, and 32
/// pieces per axis is the largest frame the slowest MCU foreground has proven
/// to process without tripping its watchdog stall budget.
pub const SERIAL_BUNDLE_LIMITS: BundleLimits = BundleLimits {
    wire_budget: 1024,
    pieces_per_axis: 32,
};

pub trait PieceSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError>;

    /// How much one bundled transaction to `mcu_id` may carry. Transports
    /// whose round-trip cost is per-transaction rather than per-byte (and
    /// whose receiver is not a watchdog-policed MCU foreground) override this
    /// to amortize the round trip.
    fn bundle_limits(&self, _mcu_id: u32) -> BundleLimits {
        SERIAL_BUNDLE_LIMITS
    }

    /// Note that the first piece of a fresh anchor epoch for `key` starts at
    /// `at_start_clock`, a clock bearing no relation to the timeline the
    /// transport still holds. Transports that keep a host-side committed
    /// stream cut it exactly at that piece; the piece-ring transports carry
    /// the discontinuity on the wire, so this is a no-op for them.
    ///
    /// A bundle may span the boundary, so the mark names the piece rather
    /// than the bundle.
    fn mark_reanchor(&self, _key: AxisKey, _at_start_clock: u64, _epoch_freq: Option<f64>) {}

    /// Note that the stream time jumped a drained-to-rest hole (a dwell)
    /// and the next piece for `key` starts at `at_start_clock`, later than
    /// the previous piece's projected end. The position is unchanged and no
    /// steps span the hole, so transports that validate seam contiguity
    /// sanction a forward-only jump; piece-ring transports carry absolute
    /// clocks and need nothing.
    fn mark_seam_gap(&self, _key: AxisKey, _at_start_clock: u64) {}

    /// How this transport's seam consumer reprojects a piece `duration` into
    /// clock ticks for `key`. Hold merging rewrites durations, so it must use
    /// this basis; the live clock estimate drifts against the frozen epoch
    /// slope a host-side committed stream already sent frames on, and over a
    /// lane held for a whole layer that drift becomes a hard seam gap.
    /// `None` = pieces reach the mcu walker untouched.
    fn seam_basis(&self, _key: AxisKey) -> Option<super::sched::SeamBasis> {
        None
    }

    /// Deliver every axis frame destined for `mcu_id` as one bundled
    /// transaction. A whole bundle either lands or it doesn't — the caller
    /// commits the ring bookkeeping for all axes only on `Ok`, so a failed
    /// bundle re-sends byte-identical frames to the same ring slots.
    ///
    /// The default fans out to per-axis `send_frame`; a transport that can
    /// pack multiple axes into one round-trip overrides this to collapse the
    /// per-frame overhead that dominates dense-stream delivery.
    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        for f in frames {
            self.send_frame(
                AxisKey {
                    mcu_id,
                    axis: f.axis,
                },
                &f.pieces,
                f.start_slot,
                f.new_head,
                f.room,
            )?;
        }
        Ok(())
    }

    /// Routes a classic-stepping `stepcompress_barrier_ack` to the endpoint
    /// that issued the barrier.
    fn on_barrier_ack(&self, mcu_id: u32, oid: u8, seq: u32) -> Result<(), SendError> {
        Err(SendError::Fatal(format!(
            "stepcompress_barrier_ack oid={oid} seq={seq} arrived for mcu {mcu_id}, which has \
             no stepcompress endpoint"
        )))
    }
}

pub struct PumpCallbacks {
    pub ring_depth_of: Box<dyn Fn(AxisKey) -> u32 + Send>,
    pub mcu_clock_of: Box<dyn Fn(u32) -> Option<(u64, f64)> + Send>,
    pub on_fatal_transport: Box<dyn Fn(AxisKey) + Send>,
    pub on_abandon: Box<dyn Fn(AxisKey, u32) + Send>,
    pub on_drip_stall: Box<dyn Fn(String) + Send>,
}

impl PumpCallbacks {
    pub fn noop(ring_depth: u32) -> Self {
        Self {
            ring_depth_of: Box::new(move |_| ring_depth),
            mcu_clock_of: Box::new(|_| None),
            on_fatal_transport: Box::new(|_| {}),
            on_abandon: Box::new(|_, _| {}),
            on_drip_stall: Box::new(|_| {}),
        }
    }
}
