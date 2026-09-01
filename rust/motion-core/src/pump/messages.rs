use crate::lock_ext::LockExt;
use std::sync::Arc;

use ethercat_rt::buzz::MAX_BUZZ_SLOTS;
use trajectory::continuous::ProfileError;
use trajectory::{BuzzProfile, ClockedMotorSpan};

use super::drip::DripArm;
use super::sched::AxisFrame;
use crate::types::AxisKey;

pub struct EnqueueMsg {
    pub key: AxisKey,
    pub spans: Vec<ClockedMotorSpan>,
    pub epoch: crate::anchor::StreamEpoch,
    pub lead_secs: f64,
    pub source_line: u32,
    pub epoch_freq: Option<f64>,
    pub batch_end: bool,
}

/// Records each dispatched view into the motion-history store when its
/// transport endpoint takes ownership, so the store mirrors work that can
/// reach the MCU. Recording at dispatch time instead would flood the ring
/// with an entire move up front — a long homing move evicts its own start
/// before the endstop trip is resolved against it.
///
/// A [`ClockedMotorSpan`] already carries the exact clock anchor and the rate
/// the producer projected it on, so the store needs nothing else to place the
/// view on the MCU clock.
pub struct HistoryRecorder {
    pub store: Arc<std::sync::Mutex<crate::motion_history::HistoryStore>>,
}

impl HistoryRecorder {
    pub(super) fn record(
        &self,
        key: AxisKey,
        span: ClockedMotorSpan,
    ) -> Result<(), crate::motion_history::HistoryError> {
        self.store.lock_ok().record(key, span)
    }
}

/// Which wire path finished the views a heartbeat reports. A dual-transport
/// lane is a member of two endpoints at once, and each one only ever retires
/// the views the pump routed through it, so their counts are separate
/// odometers that the pump adds up. Collapsing them into one number per axis
/// lets whichever endpoint reports last — the idle one, with a frozen count —
/// erase the active one's progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetiredBy {
    Pulse,
    Phase,
    EtherCat,
}

impl RetiredBy {
    pub const COUNT: usize = 3;
}

/// One endpoint's retirement report. `axes` names the axis each count belongs
/// to: two endpoints share one mcu when a board carries both pulse and phase
/// lanes, and neither may speak for the other's axes.
pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub axes: Vec<u8>,
    pub consumed_counts: Option<Vec<u32>>,
    pub retired_counts: Vec<u32>,
    pub retired_by: RetiredBy,
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
    StepcompressFatal {
        mcu_id: u32,
        error: String,
    },
    /// A projection rebase (nudge-path re-anchor) invalidated every lane
    /// seam on the named lane's MCU without giving that lane any views to
    /// carry the cut. The pump forwards it to the endpoint so the lane's
    /// stream is cut at `at_start_clock` on the new epoch slope before its
    /// next views arrive.
    MarkReanchor {
        key: AxisKey,
        at_start_clock: u64,
        epoch_freq: Option<f64>,
    },
    /// One resonance sweep, armed across every transport it names in one
    /// pass. It rides the control channel rather than the span stream
    /// because arming mutates transport state the pump alone may touch
    /// while it is pushing.
    Buzz {
        params: BuzzParams,
        reply: std::sync::mpsc::SyncSender<Result<BuzzToken, String>>,
    },
    Barrier(std::sync::mpsc::SyncSender<()>),
    Shutdown,
}

/// The scalar description of one resonance sweep. Every route of one arming
/// is driven from this same wave: the host-evaluated routes share the
/// [`BuzzProfile`] it builds, and the EtherCAT node, which runs its own
/// oscillator, is armed from the very numbers that profile was built from.
#[derive(Clone, Copy, Debug)]
pub struct BuzzWave {
    pub freq_start_millihz: u32,
    pub freq_end_millihz: u32,
    pub amplitude_nm: u32,
    pub duration_ms: u32,
    pub ramp_ms: u32,
}

impl BuzzWave {
    #[must_use]
    pub fn duration_secs(&self) -> f64 {
        f64::from(self.duration_ms) * 1.0e-3
    }

    pub fn profile(&self) -> Result<BuzzProfile, ProfileError> {
        if self.amplitude_nm == 0 {
            return Err(ProfileError::ZeroDisplacement);
        }
        BuzzProfile::try_new(
            f64::from(self.amplitude_nm) * 1.0e-6,
            f64::from(self.freq_start_millihz) * 1.0e-3,
            f64::from(self.freq_end_millihz) * 1.0e-3,
            self.duration_secs(),
            f64::from(self.ramp_ms) * 1.0e-3,
            0.0,
        )
    }
}

/// One driven lane of a phase route: the axis its sample endpoint indexes
/// lanes by, and the direction the sweep drives it.
#[derive(Clone, Copy, Debug)]
pub struct BuzzLane {
    pub axis: u8,
    pub sign: f64,
}

/// Where every route of one arming starts. The pump resolves it once per mcu
/// and hands it to each endpoint: a route that anchored itself would start
/// its lanes at whatever instant the pump happened to reach that transport,
/// and the axes of one sweep would no longer be in phase. The EtherCAT node
/// runs the oscillator on its own DC grid, so its filler snaps this instant
/// to the first grid cycle at or after it rather than resolving its own.
#[derive(Clone, Copy, Debug)]
pub struct BuzzStart {
    pub clock: u64,
    pub clock_freq_hz: f64,
}

/// Which transport a route drives. One arming may name a given transport of a
/// given mcu once: a second route onto the same endpoint would find it already
/// armed and refuse mid-pass, with the earlier routes already sweeping.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BuzzTransport {
    Pulse,
    Phase,
    Ethercat,
}

/// One transport a resonance sweep drives. A machine mixes them freely — a
/// servo Y beside a pulsed Z beside a phase-stepped X — and all three are
/// armed from one wave in one pass, so the axes of one buzz stay in phase.
pub enum BuzzRoute {
    Pulse {
        mcu_id: u32,
        endpoint: Arc<std::sync::Mutex<super::StepcompressEndpoint>>,
        axis_mask: u8,
        sign_mask: u8,
    },
    Phase {
        mcu_id: u32,
        endpoint: Arc<std::sync::Mutex<super::SampleEndpoint>>,
        lanes: Vec<BuzzLane>,
    },
    Ethercat {
        mcu_id: u32,
        filler: super::RingFiller,
        slot_mask: u8,
        sign_mask: u8,
    },
}

impl std::fmt::Debug for BuzzRoute {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pulse {
                mcu_id,
                axis_mask,
                sign_mask,
                ..
            } => f
                .debug_struct("Pulse")
                .field("mcu_id", mcu_id)
                .field("axis_mask", axis_mask)
                .field("sign_mask", sign_mask)
                .finish_non_exhaustive(),
            Self::Phase { mcu_id, lanes, .. } => f
                .debug_struct("Phase")
                .field("mcu_id", mcu_id)
                .field("lanes", lanes)
                .finish_non_exhaustive(),
            Self::Ethercat {
                mcu_id,
                slot_mask,
                sign_mask,
                ..
            } => f
                .debug_struct("Ethercat")
                .field("mcu_id", mcu_id)
                .field("slot_mask", slot_mask)
                .field("sign_mask", sign_mask)
                .finish_non_exhaustive(),
        }
    }
}

impl BuzzRoute {
    #[must_use]
    pub fn mcu_id(&self) -> u32 {
        match self {
            Self::Pulse { mcu_id, .. }
            | Self::Phase { mcu_id, .. }
            | Self::Ethercat { mcu_id, .. } => *mcu_id,
        }
    }

    #[must_use]
    pub fn transport(&self) -> BuzzTransport {
        match self {
            Self::Pulse { .. } => BuzzTransport::Pulse,
            Self::Phase { .. } => BuzzTransport::Phase,
            Self::Ethercat { .. } => BuzzTransport::Ethercat,
        }
    }

    /// The bits this route drives on its transport: axes for the two host
    /// transports, drive slots for the EtherCAT node. Two routes of one
    /// arming that claim a bit twice would fight over the same motor.
    #[must_use]
    pub fn driven_mask(&self) -> u8 {
        match self {
            Self::Pulse { axis_mask, .. } => *axis_mask,
            Self::Phase { lanes, .. } => lanes
                .iter()
                .filter(|lane| lane.axis < 8)
                .fold(0u8, |bits, lane| bits | (1 << lane.axis)),
            Self::Ethercat { slot_mask, .. } => *slot_mask,
        }
    }

    /// Everything the route can be held to before it is touched: it names a
    /// motor the transport actually drives, the transport is idle, and the
    /// start it was handed is one the transport can be armed on. Every
    /// condition `arm` would refuse is checked here, so the pump can clear
    /// every route of one buzz before arming the first.
    pub fn ready(&self, start: BuzzStart) -> Result<(), String> {
        match self {
            Self::Pulse {
                mcu_id,
                endpoint,
                axis_mask,
                ..
            } => {
                let endpoint = endpoint.lock_ok();
                if !endpoint.accepts_buzz_mask(*axis_mask) {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} carries no pulse motor of axis mask \
                         0x{axis_mask:02x}"
                    ));
                }
                let motors = endpoint.buzz_slot_count();
                if motors > MAX_BUZZ_SLOTS {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} carries {motors} pulse motors, above the \
                         {MAX_BUZZ_SLOTS}-motor buzz limit"
                    ));
                }
                if !endpoint.buzz_complete() {
                    return Err(format!(
                        "resonance buzz rejected: mcu {mcu_id} pulse endpoint is still busy"
                    ));
                }
                Ok(())
            }
            Self::Phase {
                mcu_id,
                endpoint,
                lanes,
            } => {
                if lanes.is_empty() {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} phase route names no lane"
                    ));
                }
                let mut endpoint = endpoint.lock_ok();
                let mut seen = 0u8;
                for lane in lanes {
                    if !lane.sign.is_finite() || lane.sign == 0.0 {
                        return Err(format!(
                            "resonance buzz: mcu {mcu_id} drives axis {} with sign {}",
                            lane.axis, lane.sign
                        ));
                    }
                    if !endpoint.drives_axis(lane.axis) {
                        return Err(format!(
                            "resonance buzz: mcu {mcu_id} sample endpoint drives no axis {}",
                            lane.axis
                        ));
                    }
                    if lane.axis >= 8 {
                        return Err(format!(
                            "resonance buzz: mcu {mcu_id} phase route names axis {}, above the \
                             eight axes one arming can address",
                            lane.axis
                        ));
                    }
                    if seen & (1 << lane.axis) != 0 {
                        return Err(format!(
                            "resonance buzz: mcu {mcu_id} phase route names axis {} twice",
                            lane.axis
                        ));
                    }
                    seen |= 1 << lane.axis;
                }
                if !endpoint
                    .buzz_complete()
                    .map_err(|error| format!("resonance buzz: mcu {mcu_id}: {error}"))?
                {
                    return Err(format!(
                        "resonance buzz rejected: mcu {mcu_id} phase lanes are still sweeping"
                    ));
                }
                if !endpoint
                    .transport_quiescent()
                    .map_err(|error| format!("resonance buzz: mcu {mcu_id}: {error}"))?
                {
                    return Err(format!(
                        "resonance buzz rejected: mcu {mcu_id} phase lanes still carry trajectory"
                    ));
                }
                Ok(())
            }
            Self::Ethercat {
                mcu_id,
                filler,
                slot_mask,
                ..
            } => {
                if *slot_mask == 0 {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} ethercat route selects no drive slot"
                    ));
                }
                if start.clock_freq_hz != super::wire_sink::DC_GRID_CLOCK_FREQ_HZ {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} is clocked at {} Hz, so its arming instant \
                         is not on the node's nanosecond DC grid",
                        start.clock_freq_hz
                    ));
                }
                let filler = filler.lock_ok();
                if filler.buzz_active() || filler.wants_drain() {
                    return Err(format!(
                        "resonance buzz rejected: mcu {mcu_id} setpoint filler is still busy"
                    ));
                }
                Ok(())
            }
        }
    }

    pub fn arm(
        &self,
        profile: &Arc<BuzzProfile>,
        wave: BuzzWave,
        start: BuzzStart,
    ) -> Result<(), String> {
        match self {
            Self::Pulse {
                mcu_id,
                endpoint,
                axis_mask,
                sign_mask,
            } => endpoint
                .lock_ok()
                .arm_buzz(*axis_mask, *sign_mask, profile, start.clock)
                .map_err(|error| {
                    format!("resonance buzz: mcu {mcu_id} pulse endpoint refused it: {error}")
                }),
            Self::Phase {
                mcu_id,
                endpoint,
                lanes,
            } => {
                let BuzzStart {
                    clock,
                    clock_freq_hz,
                } = start;
                endpoint
                    .lock_ok()
                    .arm_buzz(lanes, profile, clock, clock_freq_hz)
                    .map_err(|error| {
                        format!(
                            "resonance buzz: mcu {mcu_id} sample endpoint refused the overlay: \
                             {error}"
                        )
                    })
            }
            Self::Ethercat {
                mcu_id,
                filler,
                slot_mask,
                sign_mask,
            } => {
                let result = filler.lock_ok().arm_buzz(
                    *slot_mask,
                    *sign_mask,
                    wave.freq_start_millihz,
                    wave.freq_end_millihz,
                    wave.amplitude_nm,
                    wave.duration_ms,
                    wave.ramp_ms,
                    start.clock,
                );
                if result != 0 {
                    return Err(format!(
                        "resonance buzz: mcu {mcu_id} setpoint filler rejected it (result {result})"
                    ));
                }
                Ok(())
            }
        }
    }

    pub fn complete(&self) -> Result<bool, String> {
        match self {
            Self::Pulse { endpoint, .. } => Ok(endpoint.lock_ok().buzz_complete()),
            Self::Phase {
                mcu_id, endpoint, ..
            } => endpoint
                .lock_ok()
                .buzz_complete()
                .map_err(|error| format!("resonance buzz: mcu {mcu_id}: {error}")),
            Self::Ethercat { filler, .. } => {
                let filler = filler.lock_ok();
                Ok(!filler.buzz_active() && !filler.wants_drain())
            }
        }
    }
}

/// The one instant every route of one mcu starts on, resolved once per mcu so
/// pulse, phase and EtherCAT lanes of one sweep stay in phase.
pub(super) fn anchored_start(
    mcu_id: u32,
    clock_of: &dyn Fn(u32) -> Option<(u64, f64)>,
    lead_secs: f64,
) -> Result<BuzzStart, String> {
    let (now, clock_freq_hz) = clock_of(mcu_id).ok_or_else(|| {
        format!("resonance buzz: mcu {mcu_id} has no synced clock to anchor the sweep on")
    })?;
    if !clock_freq_hz.is_finite() || clock_freq_hz <= 0.0 {
        return Err(format!(
            "resonance buzz: mcu {mcu_id} reports a clock frequency of {clock_freq_hz} Hz, so no \
             start instant can be anchored on it"
        ));
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let clock = now.saturating_add((clock_freq_hz * lead_secs) as u64);
    Ok(BuzzStart {
        clock,
        clock_freq_hz,
    })
}

#[derive(Debug)]
pub struct BuzzParams {
    pub routes: Arc<[BuzzRoute]>,
    pub wave: BuzzWave,
}

/// The one handle an arming hands back. Completion is asked of the routes
/// that buzz actually armed, so a caller polls its own sweep rather than
/// every endpoint the machine happens to own.
#[derive(Debug)]
pub struct BuzzToken {
    routes: Arc<[BuzzRoute]>,
}

impl BuzzToken {
    #[must_use]
    pub fn new(routes: Arc<[BuzzRoute]>) -> Self {
        Self { routes }
    }

    pub fn complete(&self) -> Result<bool, String> {
        for route in self.routes.iter() {
            if !route.complete()? {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

#[derive(Debug)]
pub enum SendError {
    Fatal(String),
    Halted(String),
    Transient(String),
}

impl SendError {
    pub(super) fn mcu_reject(mcu_id: u32, result: i32) -> Self {
        let message = format!("mcu {mcu_id} rejected a motion frame: result {result}");
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
    pub spans_per_axis: usize,
}

/// Serial endpoints accept a full host-side ring in one transaction. This
/// amortizes the serial response latency while each endpoint still enforces
/// its own advertised room before accepting the batch.
pub const SERIAL_BUNDLE_LIMITS: BundleLimits = BundleLimits { spans_per_axis: 8 };

/// What one sweep of [`SpanSink::drain_tick`] found: nothing left staged
/// anywhere, at least one endpoint still owing a further window, or the mcu
/// whose window the transport refused.
#[derive(Debug)]
pub enum DrainTick {
    Quiet,
    Pending,
    Failed { mcu_id: u32, error: SendError },
}

pub trait SpanSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
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

    /// Which endpoint on the lane's mcu owns it. A bundle is atomic per
    /// endpoint, so the pump never mixes two groups in one transaction — one
    /// mcu can carry a pulse lane and a phase lane at once, and each is
    /// committed on its own transport's answer. A sink with a single endpoint
    /// per mcu leaves this at the one group.
    fn lane_group(&self, _key: AxisKey) -> u8 {
        0
    }

    /// Note that the first view of a fresh anchor epoch for `key` starts at
    /// `at_start_clock`, a clock bearing no relation to the timeline the
    /// transport still holds. Every transport keeps a host-side committed
    /// stream, so it cuts that stream exactly at that view.
    ///
    /// A bundle may span the boundary, so the mark names the view rather
    /// than the bundle.
    fn mark_reanchor(&self, _key: AxisKey, _at_start_clock: u64, _epoch_freq: Option<f64>) {}

    /// Note that the stream time jumped a drained-to-rest hole (a dwell)
    /// and the next view for `key` starts at `at_start_clock`, later than
    /// the previous view's end clock. The position is unchanged and no
    /// steps span the hole, so a transport that validates seam contiguity
    /// sanctions a forward-only jump.
    fn mark_seam_gap(&self, _key: AxisKey, _at_start_clock: u64) {}

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
                &f.spans,
                f.new_head,
                f.room,
            )?;
        }
        Ok(())
    }

    fn flush_keys(&self, _keys: &[AxisKey]) -> Result<(), SendError> {
        Ok(())
    }

    /// Drop every named endpoint's accepted and staged motion before the pump
    /// acknowledges a halt. The endpoint must publish abandonment against its
    /// absolute odometers before new motion can resume.
    fn cut_staged(&self, _keys: &[AxisKey]) -> Result<(), SendError> {
        Ok(())
    }

    /// Ship one further window to every endpoint still holding samples the
    /// pump has not shipped — a host-generated source (a buzz) or trajectory
    /// left over past one fill window — and report whether any endpoint owes
    /// another window after that.
    fn drain_tick(&self) -> DrainTick {
        DrainTick::Quiet
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
