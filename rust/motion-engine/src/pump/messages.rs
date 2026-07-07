use std::sync::Arc;

use runtime::piece_ring::PieceEntry;

use super::drip::DripArm;
use super::sched::AxisFrame;
use crate::types::AxisKey;

pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
    pub lead_secs: f64,
    pub source_line: u32,
}

/// Records each piece into the motion-history store at the moment it is
/// accepted by the MCU, so the store mirrors what the MCU can actually
/// execute. Recording at dispatch time instead would flood the ring with an
/// entire move up front — a long homing move evicts its own start before the
/// endstop trip is resolved against it.
pub struct HistoryRecorder {
    pub store: Arc<std::sync::Mutex<crate::motion_history::HistoryStore>>,
    pub nominal_freqs: Arc<std::sync::Mutex<std::collections::HashMap<u32, u32>>>,
}

impl HistoryRecorder {
    pub(super) fn record(&self, key: AxisKey, piece: &PieceEntry, host_t: f64) {
        let nominal_freq = *self
            .nominal_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&key.mcu_id)
            .unwrap_or_else(|| {
                panic!(
                    "no nominal clock frequency registered for mcu {} \
                     — set_nominal_clock_freq was not called before streaming",
                    key.mcu_id
                )
            });
        self.store.lock().unwrap_or_else(|p| p.into_inner()).record(
            key,
            piece,
            nominal_freq,
            host_t,
        );
    }
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Heartbeat(HeartbeatMsg),
    Flush(Vec<AxisKey>),
    DripArm(DripArm),
    DripDisarm(u64),
    Barrier(std::sync::mpsc::SyncSender<()>),
    Shutdown,
}

#[derive(Debug)]
pub enum SendError {
    Fatal(String),
    Transient(String),
}

impl SendError {
    pub(super) fn retryable_mcu_reject(mcu_id: u32, result: i32) -> Self {
        Self::Transient(format!(
            "mcu {mcu_id} rejected PushPieces frame: result {result}"
        ))
    }
}

impl std::fmt::Display for SendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fatal(s) => write!(f, "fatal: {s}"),
            Self::Transient(s) => write!(f, "transient: {s}"),
        }
    }
}

pub trait PieceSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError>;

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
