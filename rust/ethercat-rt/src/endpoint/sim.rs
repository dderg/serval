//! No-hardware endpoint: a simulated [`DriveChain`] plus the `EndpointCtx`
//! constructor that runs the real command dispatch and DC loop over it.
//!
//! Both the drive-off `ethercat-rt-stub` binary and the unit tests build their
//! endpoint here, so the stub answers the protocol with the same code the
//! hardware endpoint runs — the only thing swapped out is the bus.

use std::sync::atomic::{AtomicI16, AtomicU16, Ordering};
use std::sync::Arc;

use super::drive::DriveChain;
use super::EndpointCtx;
use crate::capture::Capture;
use crate::clock::monotonic_ns;
use crate::damper::DiffDamperBank;
use crate::ffi::EcTelemetry;
use crate::live_tap::{self, LiveTap};
use crate::mailbox::MailboxWorker;
use crate::sdo::SdoBus;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;
use crate::trim::DiffTrimBank;

/// Rotor model: every slot tracks its commanded target with a constant
/// following error, optionally sliding at a fixed uncommanded rate (the raw
/// encoder motion the damper differentiates) and reporting an injectable
/// torque (what the trim integrates and the sensorless endstop trips on).
pub struct SimDrive {
    cycle_ns: u64,
    /// Grid-aligned wake time of the current cycle; the DC grid the endpoint
    /// indexes its setpoint ring with.
    wake_ns: u64,
    /// Sleep to `wake_ns` in `cycle()`. The stub paces itself so its grid
    /// tracks wall time; tests step the grid explicitly and never sleep.
    paced: bool,
    targets: Vec<i32>,
    following_error: Vec<i32>,
    drift_counts_per_cycle: Vec<f64>,
    drifted_counts: Vec<f64>,
    torque_offsets: Vec<i16>,
    velocity_offsets: Vec<i32>,
    torques: Arc<Vec<AtomicI16>>,
    /// `enable_all` result — nonzero simulates a failed CiA402 walk.
    enable_rc: i32,
    /// Latch a drive error after this many commanded target writes, i.e.
    /// after this many cycles of played motion.
    fault_after_writes: Option<u32>,
    writes: u32,
    error_code: AtomicU16,
}

impl SimDrive {
    #[must_use]
    pub fn new(num_slaves: usize) -> Self {
        Self {
            cycle_ns: 0,
            wake_ns: 0,
            paced: false,
            targets: vec![0; num_slaves],
            following_error: vec![0; num_slaves],
            drift_counts_per_cycle: vec![0.0; num_slaves],
            drifted_counts: vec![0.0; num_slaves],
            torque_offsets: vec![0; num_slaves],
            velocity_offsets: vec![0; num_slaves],
            torques: Arc::new((0..num_slaves).map(|_| AtomicI16::new(0)).collect()),
            enable_rc: 0,
            fault_after_writes: None,
            writes: 0,
            error_code: AtomicU16::new(0),
        }
    }

    /// Advance a real DC grid and sleep between exchanges, so the endpoint's
    /// grid index tracks wall time the way a synced bus does.
    #[must_use]
    pub fn paced(num_slaves: usize, cycle_ns: u64) -> Self {
        Self {
            cycle_ns,
            wake_ns: monotonic_ns(),
            paced: true,
            ..Self::new(num_slaves)
        }
    }

    #[must_use]
    pub fn with_following_error(mut self, counts: Vec<i32>) -> Self {
        self.following_error = counts;
        self
    }

    #[must_use]
    pub fn with_drift(mut self, counts_per_cycle: Vec<f64>) -> Self {
        self.drift_counts_per_cycle = counts_per_cycle;
        self
    }

    #[must_use]
    pub fn with_torques(self, torques: Vec<i16>) -> Self {
        for (cell, t) in self.torques.iter().zip(torques) {
            cell.store(t, Ordering::Relaxed);
        }
        self
    }

    #[must_use]
    pub fn with_enable_rc(mut self, rc: i32) -> Self {
        self.enable_rc = rc;
        self
    }

    #[must_use]
    pub fn with_fault_after_writes(mut self, writes: Option<u32>) -> Self {
        self.fault_after_writes = writes;
        self
    }

    /// Handle for injecting a measured torque from another thread — the CoE
    /// mailbox worker writing 6077h on the stub's object dictionary.
    #[must_use]
    pub fn torque_handle(&self) -> Arc<Vec<AtomicI16>> {
        Arc::clone(&self.torques)
    }
}

impl DriveChain for SimDrive {
    fn cycle_time_ns(&self) -> u64 {
        self.wake_ns
    }

    fn cycle(&mut self) -> (i32, i64) {
        for (pos, drift) in self
            .drifted_counts
            .iter_mut()
            .zip(&self.drift_counts_per_cycle)
        {
            *pos += drift;
        }
        self.wake_ns += self.cycle_ns;
        if self.paced {
            let now = monotonic_ns();
            if self.wake_ns > now {
                std::thread::sleep(std::time::Duration::from_nanos(self.wake_ns - now));
            }
        }
        (3 * self.targets.len() as i32, 0)
    }

    fn enable_all(&mut self) -> i32 {
        self.enable_rc
    }

    fn disable_all(&mut self) {}

    fn shutdown(&mut self) {}

    fn set_target_position(&mut self, slot: usize, counts: i32) {
        self.targets[slot] = counts;
        self.writes += 1;
        if self.fault_after_writes.is_some_and(|n| self.writes == n) {
            self.error_code.store(0x8611, Ordering::Relaxed);
        }
    }

    fn set_velocity_offset(&mut self, slot: usize, counts_per_s: i32) {
        self.velocity_offsets[slot] = counts_per_s;
    }

    fn set_torque_offset(&mut self, slot: usize, tenths_pct: i16) {
        self.torque_offsets[slot] = tenths_pct;
    }

    fn position_actual(&self, slot: usize) -> i32 {
        self.targets[slot] - self.following_error[slot] + self.drifted_counts[slot].round() as i32
    }

    fn velocity_actual(&self, _slot: usize) -> i32 {
        0
    }

    fn torque_actual(&self, slot: usize) -> i16 {
        self.torques[slot].load(Ordering::Relaxed)
    }

    /// Reported once, then cleared: the endpoint parks on the first non-zero
    /// read, and a permanently faulted drive would re-park every re-enable.
    fn error_code(&self, _slot: usize) -> u16 {
        self.error_code.swap(0, Ordering::Relaxed)
    }

    fn telemetry(&self, slot: usize) -> EcTelemetry {
        EcTelemetry {
            target_position: self.targets[slot],
            position_actual: self.position_actual(slot),
            following_error: self.following_error[slot],
            torque_actual: self.torque_actual(slot),
            torque_offset: self.torque_offsets[slot],
            velocity_offset: self.velocity_offsets[slot],
            ..EcTelemetry::default()
        }
    }

    fn dump_al_state(&self) {}
}

/// Everything the simulated endpoint needs that is not derivable.
pub struct SimConfig<'a> {
    /// Already-bound control socket — the caller owns the claim handshake.
    pub server: FrameServer,
    pub live_tap_socket: &'a str,
    /// Axis each slot follows, as the `--slave`/`--axis` groups declare it.
    /// A lane run must name a slot that follows the axis it claims, so this
    /// is the stub's topology.
    pub slave_axes: Vec<u8>,
    pub counts_per_mm: Vec<f64>,
    pub cycle_ns: i64,
    /// Cycles between telemetry beats; `u64::MAX` silences them.
    pub telemetry_period: u64,
}

const SIM_ROTATION_DISTANCE: f64 = 40.0;
/// Same 2 m/s discontinuity bound bringup derives, at the 250 µs test cycle.
const SIM_JUMP_LOG_COUNTS: i64 = 1638;

pub fn sim_endpoint(
    cfg: SimConfig<'_>,
    drive: SimDrive,
    bus: impl SdoBus + Send + 'static,
) -> EndpointCtx {
    sim_endpoint_with_drive(cfg, Box::new(drive), bus)
}

pub(super) fn sim_endpoint_with_drive(
    cfg: SimConfig<'_>,
    drive: Box<dyn DriveChain>,
    bus: impl SdoBus + Send + 'static,
) -> EndpointCtx {
    let SimConfig {
        server,
        live_tap_socket,
        slave_axes,
        counts_per_mm: counts,
        cycle_ns,
        telemetry_period,
    } = cfg;
    let num_slaves = slave_axes.len();
    assert_eq!(counts.len(), num_slaves, "one counts_per_mm per slot");
    let rotation_distance = vec![SIM_ROTATION_DISTANCE; num_slaves];
    let invert = vec![false; num_slaves];
    let live_tap = LiveTap::spawn(
        live_tap_socket,
        live_tap::slot_configs(&counts, &rotation_distance, &invert),
        cycle_ns,
    )
    .expect("bind live tap socket");
    EndpointCtx {
        server,
        drive,
        num_slaves,
        counts_per_mm: counts.clone(),
        invert,
        cmd_counts_per_mm: counts,
        rotation_distance,
        slave_axes,
        velocity_ff: vec![false; num_slaves],
        torque_clamp_tenths: vec![0; num_slaves],
        jump_log_counts: vec![SIM_JUMP_LOG_COUNTS; num_slaves],
        cycle_ns,
        group_delay_ns: 0,
        telemetry_period,
        dynamics: None,
        pin: super::cycle::PinState::default(),
        drive_dirs: vec![1.0; num_slaves],
        drive_scratch: super::cycle::DriveScratch::new(num_slaves),
        run_limits: vec![(0, 0); num_slaves],
        sp_rings: (0..num_slaves)
            .map(|slot| ethercat_setpoint::setpoint::SetpointRing::new(slot, cycle_ns as u32))
            .collect(),
        grid: ethercat_setpoint::setpoint::SampleGrid::new(cycle_ns as u64),
        ring_origin: vec![None; num_slaves],
        sp_play_scratch: vec![None; num_slaves],
        sp_fill_scratch: Vec::with_capacity(ethercat_setpoint::setpoint::MAX_FILL_CYCLES),
        reclaim: crate::reclaim::Reclaim::spawn(),
        last_grid_index: 0,
        last_grid_clock: 0,
        damper: DiffDamperBank::new(cycle_ns),
        trim: DiffTrimBank::new(cycle_ns),
        comp: crate::strain_comp::StrainCompBank::new(cycle_ns),
        last_counts: vec![None; num_slaves],
        last_written_offset: vec![0; num_slaves],
        report_anchor: vec![None; num_slaves],
        last_streamed_target: vec![None; num_slaves],
        suppressed: vec![false; num_slaves],
        last_sent_retired: 0,
        heartbeat_sent: false,
        gate: TorqueGate::new(),
        capture: Capture::new(),
        live_tap,
        tap_slots: (0..num_slaves as u8).collect(),
        cycle_index: 0,
        mailbox: MailboxWorker::spawn(bus, |_, _, _| 0),
        pending_starts: Vec::new(),
        pending_stops: Vec::new(),
        pending_seed: None,
        capture_slots: Vec::new(),
        prdiv: 0,
        ff_saturation: 0,
        wkc_consecutive: 0,
        latched_drive_err: 0,
        sensorless: SensorlessBank::new(num_slaves),
        stream_halt: StreamHalt::default(),
        late_tolerance_ns: None,
        timing_armed: false,
        baseline_reanchor_count: 0,
        late_frames: 0,
        late_max_ns: i64::MIN,
        skip_count_policed: 0,
        late_frames_total: 0,
        last_lateness_ns: 0,
        last_dispatch_ns: 0,
        last_pre_work_ns: 0,
        prev_exchange_ns: 0,
        prev_exchange_return: None,
        last_nivcsw: 0,
        spans: super::cycle::CycleSpans::default(),
    }
}
