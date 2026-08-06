use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Instant;

use host_rt::host_io::McuHostIo;
use host_rt::mcu_serial_conn::McuSerialConn;

use crate::lock_ext::LockExt;

/// Everything a homing run's lifecycle touches: the run itself, an
/// early-arriving terminal-report buffer, the drip cohort it homes under, and
/// the result channel `home_axis_poll` drains. `finish` clears all four
/// together — the shape every homing exit path (poll, abort, drive fault)
/// needs.
#[derive(Default)]
pub(crate) struct HomingState {
    pub(crate) run: Arc<Mutex<Option<HomingRun>>>,
    pub(crate) pending_trips: Arc<Mutex<Vec<(u32, u8, u64)>>>,
    /// (endstop_mcu, endstop_id, host_secs) of every endstop armed since the
    /// last homing run started. `home_axis_start` consumes the earliest arm
    /// belonging to its endstop set as the staleness window's start: a trip
    /// is genuine from the moment the endstop is armed, which happens before
    /// the run is registered — an endstop already loaded past its threshold
    /// (e.g. pair strain against a hard stop) trips in that gap.
    pub(crate) recent_arms: Arc<Mutex<Vec<(u32, u8, f64)>>>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) result: Mutex<
        Option<
            crossbeam_channel::Receiver<
                Result<(geometry::MachinePos, geometry::MachinePos, u64), String>,
            >,
        >,
    >,
}

impl HomingState {
    pub(super) fn finish(&self) {
        *self.active_drip_cohort.lock_ok() = None;
        *self.run.lock_ok() = None;
        *self.result.lock_ok() = None;
        self.pending_trips.lock_ok().clear();
    }

    pub(super) fn note_arm(&self, mcu: u32, endstop_id: u8, host_secs: f64) {
        self.drop_buffered_trips_for(mcu, endstop_id);
        let mut arms = self.recent_arms.lock_ok();
        arms.retain(|&(m, e, _)| m != mcu || e != endstop_id);
        arms.push((mcu, endstop_id, host_secs));
    }

    pub(super) fn take_arm_window_start(&self, trips: &[(u32, u8)]) -> Option<f64> {
        let arms = std::mem::take(&mut *self.recent_arms.lock_ok());
        arms.iter()
            .filter(|(mcu, endstop_id, _)| trips.contains(&(*mcu, *endstop_id)))
            .map(|&(_, _, host_secs)| host_secs)
            .min_by(f64::total_cmp)
    }

    pub(super) fn drop_buffered_trips_for(&self, mcu: u32, endstop_id: u8) {
        self.pending_trips
            .lock_ok()
            .retain(|&(m, e, _)| m != mcu || e != endstop_id);
    }
}

/// The flush/drain poll bookkeeping `wait_moves_*` and `motion_drain_*` share:
/// in-flight flush waits keyed by id, the drain-poll's own flush receiver, and
/// the lagging-wait diagnostic timer.
pub(crate) struct FlushState {
    pub(crate) pending: Mutex<HashMap<u64, FlushWait>>,
    pub(crate) pending_drain: Mutex<Option<crossbeam_channel::Receiver<Option<Instant>>>>,
    pub(crate) drain_wait_diag: Mutex<Option<super::drain_wait::DrainWaitDiag>>,
    /// Starts at 1: id 0 is never handed out.
    pub(crate) next_id: AtomicU64,
}

impl Default for FlushState {
    fn default() -> Self {
        Self {
            pending: Mutex::new(HashMap::new()),
            pending_drain: Mutex::new(None),
            drain_wait_diag: Mutex::new(None),
            next_id: AtomicU64::new(1),
        }
    }
}

/// The push-pieces-pump's control handle, join handle, and backlog counter —
/// set together by `spawn_pipeline` and torn down together by `shutdown`.
#[derive(Default)]
pub(crate) struct PumpHandles {
    pub(crate) tx: Arc<Mutex<Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>>>,
    pub(crate) thread: Mutex<Option<JoinHandle<()>>>,
    pub(crate) backlog: Arc<AtomicU64>,
    pub(crate) pacer: Mutex<Option<crate::pump::StepcompressPacer>>,
}

/// The background live-position poller's cache, join handle, and stop flag —
/// spawned together by `spawn_live_position_poll_thread`, joined together by
/// `shutdown`.
pub(crate) struct PositionPoll {
    pub(crate) cache: Arc<Mutex<(HashMap<String, (f64, f64)>, Instant)>>,
    pub(crate) thread: Mutex<Option<JoinHandle<()>>>,
    pub(crate) stop: Arc<AtomicBool>,
}

impl Default for PositionPoll {
    fn default() -> Self {
        Self {
            cache: Arc::new(Mutex::new((HashMap::new(), Instant::now()))),
            thread: Mutex::new(None),
            stop: Arc::new(AtomicBool::new(false)),
        }
    }
}

/// Fault causes latched for klippy to poll and report — a drive fault
/// surfaced by an EtherCAT heartbeat, or the reason an EtherCAT endpoint died.
#[derive(Default)]
pub(crate) struct LatchedFaults {
    pub(crate) drive: Arc<Mutex<HashMap<u32, u16>>>,
    pub(crate) endpoint_death: Arc<Mutex<HashMap<u32, String>>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TripMember {
    pub(crate) endstop_mcu: u32,
    pub(crate) endstop_id: u8,
    pub(crate) remote_freeze: Option<RemoteFreeze>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RemoteFreeze {
    pub(crate) motor_mcu: u32,
    pub(crate) motor_idx: u8,
    pub(crate) stepper_idx: u8,
}

pub(crate) struct HomingRun {
    pub(crate) cohort: u64,
    pub(crate) remaining_trips: Vec<TripMember>,
    pub(crate) axis_key: crate::types::AxisKey,
    pub(crate) all_axis_keys: Vec<crate::types::AxisKey>,
    pub(crate) window_start_host: f64,
    pub(crate) notify: crossbeam_channel::Sender<
        Result<(geometry::MachinePos, geometry::MachinePos, u64), String>,
    >,
}

pub(crate) struct McuConnection {
    pub(crate) label: String,
    pub(crate) host_io: Option<Arc<McuHostIo>>,
    pub(crate) runtime_rx_priority:
        Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    pub(crate) runtime_rx_bulk: Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    pub(crate) runtime_caps: Option<mcu_protocol::messages::RuntimeCapsResponse>,
    pub(crate) identify_caps: u64,
    pub(crate) mcu_transport_supported: bool,
    pub(crate) ethercat_socket: Option<String>,
    pub(crate) endpoint_process: Option<std::process::Child>,
    pub(crate) endpoint_conn: Option<Arc<McuSerialConn>>,
    pub(crate) ethercat_slot_axes: Vec<usize>,
}

/// One EtherCAT drive slot as `[ethercat_node]` declares it in klippy. The
/// endpoint process is launched with one flag group per drive; every field
/// here maps to a `--flag` in `endpoint_args`. Extracted by attribute from the
/// Python `EthercatDrive` namedtuple, so a reordered field on either side
/// fails loud instead of silently swapping, say, `axis` and `chain_index`.
#[derive(Debug, Clone, pyo3::FromPyObject)]
pub(crate) struct EthercatDrive {
    pub(crate) chain_index: i32,
    pub(crate) axis: usize,
    pub(crate) counts_per_mm: f64,
    pub(crate) rotation_distance: f64,
    pub(crate) following_error_counts: Option<u32>,
    pub(crate) max_torque_tenth_pct: Option<u16>,
    pub(crate) velocity_ff: bool,
    pub(crate) ff_max_torque: f64,
    pub(crate) invert_direction: bool,
    pub(crate) dynamics_profile: Option<String>,
}

#[derive(Debug, Clone)]

pub(crate) struct FlushWait {
    pub(crate) rx: Option<crossbeam_channel::Receiver<Option<std::time::Instant>>>,
    pub(crate) deadline: Option<std::time::Instant>,
}
