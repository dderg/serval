use std::collections::HashMap;
use std::sync::atomic::AtomicBool;
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Instant;

use host_rt::host_io::McuHostIo;
use host_rt::mcu_serial_conn::McuSerialConn;

use motion_core::lock_ext::LockExt;

type HomingResult = Result<(geometry::MachinePos, geometry::MachinePos, u64), String>;

#[derive(Default)]
pub(crate) struct HomingState {
    pub(super) lifecycle: Mutex<HomingLifecycle>,
    partial_ready: Condvar,
}

#[derive(Default)]
pub(super) enum HomingPhase {
    #[default]
    Idle,
    Registering(u64, u32),
    Active(HomingRun),
    Completing(u64, u32),
    Finished(u64, HomingResult),
    Retiring(u64),
}

#[derive(Default)]
pub(super) struct HomingLifecycle {
    pub(super) phase: HomingPhase,
    pub(super) pending_suppresses: usize,
    pub(super) pending_trips: Vec<(u32, u8, u64)>,
    recent_arms: Vec<(u32, u8, f64)>,
    pub(super) failure: Option<String>,
    consumer_cancelled: bool,
}

impl HomingState {
    pub(super) fn begin(&self, cohort: u64, axis_mcu: u32) -> Result<(), String> {
        let mut state = self.lifecycle.lock_ok();
        if !matches!(state.phase, HomingPhase::Idle) {
            return Err("home_axis: homing lifecycle is already owned".into());
        }
        state.phase = HomingPhase::Registering(cohort, axis_mcu);
        Ok(())
    }
    pub(super) fn arm(&self, cohort: u64) -> Result<(), String> {
        let state = self.lifecycle.lock_ok();
        assert!(matches!(state.phase, HomingPhase::Registering(id, _) if id == cohort));
        if let Some(error) = &state.failure {
            return Err(error.clone());
        }
        Ok(())
    }

    pub(super) fn register(&self, run: HomingRun) -> Result<(), String> {
        let mut state = self.lifecycle.lock_ok();
        if !matches!(state.phase, HomingPhase::Registering(id, _) if id == run.cohort) {
            return Err("home_axis: registration lost lifecycle ownership".into());
        }
        if let Some(error) = state.failure.take() {
            return Err(error);
        }
        state.phase = HomingPhase::Active(run);
        Ok(())
    }

    pub(super) fn complete(&self, cohort: u64, result: HomingResult) {
        let mut state = self.lifecycle.lock_ok();
        let HomingPhase::Completing(id, _) = &state.phase else {
            panic!("homing completion without terminal ownership");
        };
        assert_eq!(*id, cohort, "homing completion cohort mismatch");
        if state.consumer_cancelled {
            state.release_result(cohort);
        } else {
            let result = state.failure.take().map_or(result, Err);
            state.phase = HomingPhase::Finished(cohort, result);
        }
    }

    pub(super) fn poll(&self) -> Result<Option<HomingResult>, String> {
        let mut state = self.lifecycle.lock_ok();
        match state.phase {
            HomingPhase::Idle => Err("home_axis_poll: no homing in progress".into()),
            HomingPhase::Finished(_, ref result)
                if result.is_err() || state.pending_suppresses == 0 =>
            {
                let HomingPhase::Finished(cohort, result) = std::mem::take(&mut state.phase) else {
                    unreachable!()
                };
                state.release_result(cohort);
                Ok(Some(result))
            }
            _ => Ok(None),
        }
    }

    pub(super) fn cancel_registration(&self) {
        let mut state = self.lifecycle.lock_ok();
        assert!(matches!(state.phase, HomingPhase::Registering(..)));
        state.phase = HomingPhase::Idle;
        state.pending_trips.clear();
        state.failure = None;
        state.consumer_cancelled = false;
    }

    pub(super) fn note_arm(&self, mcu: u32, endstop_id: u8, host_secs: f64) {
        let mut state = self.lifecycle.lock_ok();
        state
            .pending_trips
            .retain(|&(m, e, _)| m != mcu || e != endstop_id);
        state
            .recent_arms
            .retain(|&(m, e, _)| m != mcu || e != endstop_id);
        state.recent_arms.push((mcu, endstop_id, host_secs));
    }

    pub(super) fn take_arm_window_start(&self, trips: &[(u32, u8)]) -> Option<f64> {
        let arms = std::mem::take(&mut self.lifecycle.lock_ok().recent_arms);
        arms.iter()
            .filter(|(mcu, endstop_id, _)| trips.contains(&(*mcu, *endstop_id)))
            .map(|&(_, _, host_secs)| host_secs)
            .min_by(f64::total_cmp)
    }
    pub(super) fn interrupt(&self, mcu: Option<u32>, error: String) -> Option<HomingRun> {
        let mut state = self.lifecycle.lock_ok();
        let axis_mcu = match &state.phase {
            HomingPhase::Registering(_, mcu) | HomingPhase::Completing(_, mcu) => *mcu,
            HomingPhase::Active(run) => run.axis_key.mcu_id,
            HomingPhase::Idle | HomingPhase::Finished(..) | HomingPhase::Retiring(_) => {
                return None;
            }
        };
        if mcu.is_some_and(|mcu| {
            motion_core::homing::route_drive_fault(mcu, Some(axis_mcu))
                != motion_core::homing::DriveFaultRoute::HomingError
        }) {
            return None;
        }
        state.failure.get_or_insert(error);
        state.take_terminal(|_| true)
    }

    pub(super) fn abort(&self) -> Option<HomingRun> {
        let mut state = self.lifecycle.lock_ok();
        match state.phase {
            HomingPhase::Idle | HomingPhase::Retiring(_) => None,
            HomingPhase::Finished(cohort, _) => {
                state.release_result(cohort);
                None
            }
            HomingPhase::Registering(..) | HomingPhase::Active(_) | HomingPhase::Completing(..) => {
                state.consumer_cancelled = true;
                state.failure.get_or_insert_with(|| "homing aborted".into());
                state.take_terminal(|_| true)
            }
        }
    }

    pub(super) fn wait_for_pending_suppresses(&self, cohort: u64) -> Result<(), String> {
        let state = self.lifecycle.lock_ok();
        assert!(matches!(state.phase, HomingPhase::Completing(id, _) if id == cohort));
        let (state, timeout) = self
            .partial_ready
            .wait_timeout_while(state, std::time::Duration::from_secs(4), |state| {
                state.pending_suppresses != 0
            })
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if timeout.timed_out() && state.pending_suppresses != 0 {
            return Err(format!(
                "StepperSuppress: {} partial call(s) did not finish before terminal Stop",
                state.pending_suppresses
            ));
        }
        Ok(())
    }

    pub(super) fn retire_partial(&self, cohort: u64, error: Option<String>) -> Option<HomingRun> {
        let mut state = self.lifecycle.lock_ok();
        match &mut state.phase {
            HomingPhase::Finished(id, result) if *id == cohort => {
                if result.is_ok() {
                    if let Some(error) = error {
                        *result = Err(error);
                    }
                }
            }
            HomingPhase::Active(run) if run.cohort == cohort => {
                if let Some(error) = error {
                    state.failure.get_or_insert(error);
                }
            }
            HomingPhase::Completing(id, _) if *id == cohort => {
                if let Some(error) = error {
                    state.failure.get_or_insert(error);
                }
            }
            HomingPhase::Retiring(id) if *id == cohort => {}
            _ => panic!("partial homing work outlived its lifecycle"),
        }
        assert!(
            state.pending_suppresses != 0,
            "partial homing work retired twice"
        );
        state.pending_suppresses -= 1;
        self.partial_ready.notify_all();
        if state.pending_suppresses == 0 && matches!(state.phase, HomingPhase::Retiring(_)) {
            state.phase = HomingPhase::Idle;
        }
        if state.failure.is_some() {
            state.take_terminal(|run| run.cohort == cohort)
        } else {
            None
        }
    }
}

impl HomingLifecycle {
    fn release_result(&mut self, cohort: u64) {
        self.pending_trips.clear();
        self.failure = None;
        self.consumer_cancelled = false;
        self.phase = if self.pending_suppresses == 0 {
            HomingPhase::Idle
        } else {
            HomingPhase::Retiring(cohort)
        };
    }

    pub(super) fn trip_run(&mut self, trip: (u32, u8, u64)) -> Option<&mut HomingRun> {
        match &mut self.phase {
            HomingPhase::Idle | HomingPhase::Registering(..) => {
                self.pending_trips.push(trip);
                None
            }
            HomingPhase::Active(run) => Some(run),
            HomingPhase::Completing(..) | HomingPhase::Finished(..) | HomingPhase::Retiring(_) => {
                None
            }
        }
    }
    pub(super) fn take_terminal(
        &mut self,
        matches: impl FnOnce(&HomingRun) -> bool,
    ) -> Option<HomingRun> {
        let HomingPhase::Active(run) = &self.phase else {
            return None;
        };
        if !matches(run) {
            return None;
        }
        let cohort = run.cohort;
        let axis_mcu = run.axis_key.mcu_id;
        let HomingPhase::Active(run) =
            std::mem::replace(&mut self.phase, HomingPhase::Completing(cohort, axis_mcu))
        else {
            unreachable!()
        };
        Some(run)
    }
}

#[derive(Default)]
pub(crate) struct FlushState {
    pub(crate) pending_drain: Mutex<Option<crossbeam_channel::Receiver<Result<(), String>>>>,
    pub(crate) drain_wait_diag: Mutex<Option<super::drain_wait::DrainWaitDiag>>,
}

/// The pump's control handle, join handle and the per-lane-kind
/// pacers — set together by `spawn_pipeline` and torn down together by
/// `shutdown`.
#[derive(Default)]
pub(crate) struct PumpHandles {
    pub(crate) tx: Arc<Mutex<Option<crossbeam_channel::Sender<motion_core::pump::PumpMsg>>>>,
    pub(crate) thread: Mutex<Option<JoinHandle<()>>>,
    pub(crate) pacer: Mutex<Option<motion_core::pump::StepcompressPacer>>,
    pub(crate) sample_pacer: Mutex<Option<motion_core::pump::SamplePacer>>,
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
    /// The stepper the keyed endstop froze on the mcu. A trip is answered by
    /// cutting and reseeding exactly this motor's stream, so its identity — not
    /// its index within a klippy lane — is what the host resolves against.
    pub(crate) stepper_oid: u32,
}

pub(crate) struct HomingRun {
    pub(crate) cohort: u64,
    pub(crate) remaining_trips: Vec<TripMember>,
    pub(crate) axis_key: motion_core::types::AxisKey,
    pub(crate) all_axis_keys: Vec<motion_core::types::AxisKey>,
    pub(crate) window_start_host: f64,
    pub(crate) start_pos: geometry::MachinePos,
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
    pub(crate) sample_grid: Option<super::ethercat_endpoint::SampleGrid>,
    /// The pump's setpoint filler for this endpoint, built at claim time
    /// because that is where the drives' command scale and the dynamics
    /// profile are still in hand. Only an EtherCAT connection has one.
    pub(crate) ring_filler: Option<motion_core::pump::RingFiller>,
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

/// Everything `[ethercat_node]` decides about the endpoint process itself,
/// extracted by attribute from the Python `EthercatNodeClaim` namedtuple.
#[derive(Debug, Clone, pyo3::FromPyObject)]
pub(crate) struct EthercatNodeClaim {
    pub(crate) label: String,
    pub(crate) socket_path: String,
    pub(crate) interface: String,
    pub(crate) endpoint_binary: String,
    pub(crate) cycle_us: u32,
    pub(crate) dynamics_profile: Option<String>,
    pub(crate) late_tolerance_us: Option<f64>,
    pub(crate) group_delay_us: Option<f64>,
}

/// A claimed EtherCAT endpoint's live resources, handed to the engine's MCU
/// table once the handshake and the sample grid have been accepted.
pub(crate) struct EthercatRegistration {
    pub(crate) label: String,
    pub(crate) socket_path: String,
    pub(crate) child: std::process::Child,
    pub(crate) conn: McuSerialConn,
    pub(crate) slot_axes: Vec<usize>,
    pub(crate) sample_grid: super::ethercat_endpoint::SampleGrid,
    pub(crate) ring_filler: motion_core::pump::RingFiller,
}
