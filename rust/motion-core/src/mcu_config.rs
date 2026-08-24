use crate::kinematics::SPATIAL_AXES;
use crate::types::AxisKey;
use runtime::segment::KinematicTag;
use std::collections::HashSet;

pub const KINEMATICS_COREXY: u8 = KinematicTag::CoreXy as u8;

const _: () = assert!(
    KinematicTag::CoreXy as u8 == 0,
    "KinematicTag::CoreXy discriminant must be 0 — the Python↔Rust init_planner \
     topology tuples mirror it numerically (see segment.rs); renumbering breaks that contract",
);

/// How one lane's motion reaches its motor: step/dir pulses the host
/// compresses into `queue_step` frames, or absolute sample runs the mcu's
/// phase executor interpolates. A property of the lane, never of the board —
/// one mcu legally carries a modulated X beside a pulsed Z.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum LaneKind {
    #[default]
    Pulse = 0,
    Phase = 1,
    /// A phase lane whose motor also carries a classic `config_stepper`
    /// binding, because its config needs pulse-mode windows: sensorless homing
    /// drives the trip move through the classic step queue while StallGuard is
    /// armed. Both bindings exist from config time and the host routes the lane
    /// by its current mode.
    PhaseWithPulse = 2,
}

pub const LANE_KIND_PULSE: u8 = LaneKind::Pulse as u8;
pub const LANE_KIND_PHASE: u8 = LaneKind::Phase as u8;
pub const LANE_KIND_PHASE_WITH_PULSE: u8 = LaneKind::PhaseWithPulse as u8;

const _: () = assert!(
    LANE_KIND_PULSE == 0 && LANE_KIND_PHASE == 1 && LANE_KIND_PHASE_WITH_PULSE == 2,
    "LaneKind discriminants are mirrored numerically by the klippy/motion_setup.py \
     LANE_KIND_* table the init_planner topology tuples carry; renumbering \
     breaks that contract",
);

impl LaneKind {
    #[must_use]
    pub fn from_tag(tag: u8) -> Option<Self> {
        match tag {
            LANE_KIND_PULSE => Some(Self::Pulse),
            LANE_KIND_PHASE => Some(Self::Phase),
            LANE_KIND_PHASE_WITH_PULSE => Some(Self::PhaseWithPulse),
            _ => None,
        }
    }

    #[must_use]
    pub fn pulse_capable(self) -> bool {
        matches!(self, Self::Pulse | Self::PhaseWithPulse)
    }

    #[must_use]
    pub fn phase_capable(self) -> bool {
        matches!(self, Self::Phase | Self::PhaseWithPulse)
    }
}

/// Which step compressor one motor uses to encode its sampled step times.
/// Classic is the default; high precision is an explicit per-motor opt-in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StepcompressEncoder {
    HighPrecision,
    #[default]
    Classic,
}

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;

#[derive(Debug, Clone, PartialEq, Default)]
pub struct McuTopologyInput {
    pub mcu_id: u32,
    pub axes: Vec<u8>,
    pub kinematics: u8,
    pub max_motor_velocity: Vec<f64>,
    pub lane_kinds: Vec<u8>,
    pub motor_counts: Vec<u8>,
    pub microstep_distance: Vec<f64>,
    pub invert_dir: Vec<bool>,
    pub stepper_oids: Vec<u32>,
    pub move_queue_slots: u32,
    pub step_pulse_seconds: Vec<f64>,
    pub high_precision_step_compress: Vec<bool>,
    pub stepcompress_max_error_secs: f64,
    /// The mcu's own sample-executor rate (Hz), as klippy read it from the
    /// firmware's advertised `MOTION_SAMPLE_RATE_HZ`. Zero when the mcu has no
    /// phase lane to run at it.
    pub phase_sample_rate: f64,
    /// Runs each phase lane's mcu-side ring holds, as klippy read it from the
    /// firmware's advertised `SAMPLE_RUNS_PER_LANE`. Zero when the mcu has no
    /// phase lane.
    pub phase_ring_depth: u32,
}

#[derive(Debug, Clone, Default)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    pub axes: Vec<usize>,
    pub kinematics: u8,
    /// How each entry of `axes` reaches its motor. Same indexing as
    /// `max_motor_velocity`.
    pub lane_kinds: Vec<LaneKind>,
    /// Motor-frame velocity ceiling (mm/s) per entry of `axes`: the fastest
    /// this axis's MCU can physically emit steps. Tracks are validated
    /// against it at enqueue so an overspeed track fails loud on the host
    /// instead of latching -310 on the MCU.
    pub max_motor_velocity: Vec<f64>,
    /// Slots served by the ethercat-rt endpoint: torque-gated drives whose
    /// rings must stay empty while parked, so pure-hold lanes are never
    /// enqueued for them.
    pub ethercat: bool,
    pub motor_counts: Vec<u8>,
    pub microstep_distance: Vec<f64>,
    pub invert_dir: Vec<bool>,
    pub stepper_oids: Vec<u32>,
    pub move_queue_slots: u32,
    /// Per entry of `axes`: the settle the mcu enforces around every pulse
    /// (`config_stepper step_pulse_ticks`, in seconds). The step shim keeps
    /// consecutive runs at least this far apart so a re-armed classic
    /// stepper never loads a move behind its own pending unstep.
    pub step_pulse_seconds: Vec<f64>,
    pub stepcompress_encoders: Vec<StepcompressEncoder>,
    /// Rate the mcu's sample executor consumes phase-lane runs at, firmware
    /// truth rather than a host choice. Positive whenever a lane is
    /// [`LaneKind::Phase`].
    pub phase_sample_rate: f64,
    /// Runs one phase lane's mcu-side ring holds — the ceiling the host's
    /// in-flight window must never cross, firmware truth rather than a host
    /// choice. Positive whenever a lane is [`LaneKind::Phase`].
    pub phase_ring_depth: u32,
    /// Only meaningful with `StepcompressEncoder::Classic`: the max_error
    /// budget in seconds the encoder may introduce per sub-sample step time.
    /// `build_endpoint` converts it to ticks with the measured clock
    /// frequency it alone holds.
    pub stepcompress_max_error_secs: f64,
}

impl McuAxisConfig {
    #[must_use]
    pub fn motor_velocity_ceiling(&self, axis_idx: usize) -> f64 {
        let configured_index = self
            .axes
            .iter()
            .position(|&a| a == axis_idx)
            .unwrap_or_else(|| {
                panic!(
                    "mcu{} axis{axis_idx} has no motor velocity configuration",
                    self.mcu_id
                )
            });
        *self
            .max_motor_velocity
            .get(configured_index)
            .unwrap_or_else(|| {
                panic!(
                    "mcu{} axis{axis_idx} is missing its validated motor velocity ceiling",
                    self.mcu_id
                )
            })
    }

    #[must_use]
    pub fn lane_kind(&self, axis_idx: usize) -> LaneKind {
        let configured_index = self
            .axes
            .iter()
            .position(|&a| a == axis_idx)
            .unwrap_or_else(|| {
                panic!(
                    "mcu{} axis{axis_idx} is not a configured lane of this mcu",
                    self.mcu_id
                )
            });
        self.lane_kinds[configured_index]
    }

    #[must_use]
    pub fn pulse_capable(&self, lane: usize) -> bool {
        self.lane_kinds[lane].pulse_capable()
    }

    #[must_use]
    pub fn phase_capable(&self, lane: usize) -> bool {
        self.lane_kinds[lane].phase_capable()
    }
    #[must_use]
    pub fn motor_range(&self, lane: usize) -> std::ops::Range<usize> {
        let start = self.motor_counts[..lane]
            .iter()
            .map(|&count| usize::from(count))
            .sum();
        start..start + usize::from(self.motor_counts[lane])
    }

    #[must_use]
    pub fn motor_axis(&self, motor: usize) -> Option<usize> {
        self.motor_counts.iter().enumerate().find_map(|(lane, _)| {
            self.motor_range(lane)
                .contains(&motor)
                .then_some(self.axes[lane])
        })
    }

    #[must_use]
    pub fn has_pulse_lanes(&self) -> bool {
        self.lane_kinds.iter().any(|k| k.pulse_capable())
    }

    #[must_use]
    pub fn has_phase_lanes(&self) -> bool {
        self.lane_kinds.iter().any(|k| k.phase_capable())
    }

    #[must_use]
    pub fn pulse_capable_axes(&self) -> Vec<usize> {
        self.capable_axes(LaneKind::pulse_capable)
    }

    #[must_use]
    pub fn phase_capable_axes(&self) -> Vec<usize> {
        self.capable_axes(LaneKind::phase_capable)
    }

    fn capable_axes(&self, capable: fn(LaneKind) -> bool) -> Vec<usize> {
        self.axes
            .iter()
            .zip(&self.lane_kinds)
            .filter(|&(_, &kind)| capable(kind))
            .map(|(&axis, _)| axis)
            .collect()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum KinematicsConfigError {
    #[error("mcu handle {handle}: unknown kinematics tag {tag}; known: 0=corexy, 1=cartesian")]
    UnknownTag { handle: u32, tag: u8 },
    #[error(
        "mcu handle {handle}: corexy kinematics requires both X (axis {AXIS_X}) and \
         Y (axis {AXIS_Y}) on the same mcu, got axes {axes:?}"
    )]
    CorexyMissingXy { handle: u32, axes: Vec<usize> },
    #[error(
        "mcu handle {handle}: {ceiling_count} motor velocity ceilings for {axis_count} axes; \
         every configured axis requires exactly one ceiling"
    )]
    VelocityCeilingCount {
        handle: u32,
        axis_count: usize,
        ceiling_count: usize,
    },
    #[error(
        "mcu handle {handle}: unknown lane kind tag {tag}; \
         known: {LANE_KIND_PULSE}=pulse, {LANE_KIND_PHASE}=phase"
    )]
    UnknownLaneKind { handle: u32, tag: u8 },
    #[error(
        "mcu handle {handle}: {field} has {got} entries for {axis_count} axes; \
         every configured axis requires exactly one entry"
    )]
    PerAxisVectorLength {
        handle: u32,
        field: &'static str,
        axis_count: usize,
        got: usize,
    },
    #[error(
        "mcu handle {handle}: {field} has {got} entries for {motor_count} motors; \
         every configured motor requires exactly one entry"
    )]
    PerMotorVectorLength {
        handle: u32,
        field: &'static str,
        motor_count: usize,
        got: usize,
    },
    #[error("mcu handle {handle}: logical axis {axis} has no configured motors")]
    EmptyMotorGroup { handle: u32, axis: usize },
    #[error(
        "mcu handle {handle}: its pulse lanes require the mcu's advertised move_count \
         (move_queue_slots) to be positive, got 0"
    )]
    PulseLaneMoveQueueSlots { handle: u32 },
    #[error(
        "mcu handle {handle}: it carries phase lanes, so the firmware's advertised \
         MOTION_SAMPLE_RATE_HZ must reach the host as a finite positive rate, got {rate}"
    )]
    PhaseLaneSampleRate { handle: u32, rate: f64 },
    #[error(
        "mcu handle {handle}: it carries phase lanes, so the firmware's advertised \
         SAMPLE_RUNS_PER_LANE must reach the host as a positive ring depth — without it \
         the host cannot pace its in-flight window and would overrun the lane ring"
    )]
    PhaseLaneRingDepth { handle: u32 },
}

pub fn build_mcu_configs<S: ::std::hash::BuildHasher>(
    mcus: &[McuTopologyInput],
    ethercat_mcu_ids: &HashSet<u32, S>,
) -> Result<Vec<McuAxisConfig>, KinematicsConfigError> {
    mcus.iter()
        .map(|topology| {
            crate::kinematics::KinematicsModule::from_tag(topology.kinematics).map_err(|_| {
                KinematicsConfigError::UnknownTag {
                    handle: topology.mcu_id,
                    tag: topology.kinematics,
                }
            })?;
            let axes: Vec<usize> = topology.axes.iter().map(|&a| a as usize).collect();
            if topology.kinematics == KINEMATICS_COREXY
                && !(axes.contains(&AXIS_X) && axes.contains(&AXIS_Y))
            {
                return Err(KinematicsConfigError::CorexyMissingXy {
                    handle: topology.mcu_id,
                    axes,
                });
            }
            if topology.max_motor_velocity.len() != axes.len() {
                return Err(KinematicsConfigError::VelocityCeilingCount {
                    handle: topology.mcu_id,
                    axis_count: axes.len(),
                    ceiling_count: topology.max_motor_velocity.len(),
                });
            }
            for (field, got) in [
                ("lane_kinds", topology.lane_kinds.len()),
                ("motor_counts", topology.motor_counts.len()),
            ] {
                if got != axes.len() {
                    return Err(KinematicsConfigError::PerAxisVectorLength {
                        handle: topology.mcu_id,
                        field,
                        axis_count: axes.len(),
                        got,
                    });
                }
            }
            for (&axis, &count) in axes.iter().zip(&topology.motor_counts) {
                if count == 0 {
                    return Err(KinematicsConfigError::EmptyMotorGroup {
                        handle: topology.mcu_id,
                        axis,
                    });
                }
            }
            let motor_count = topology.motor_counts.iter().map(|&n| usize::from(n)).sum();
            for (field, got) in [
                ("microstep_distance", topology.microstep_distance.len()),
                ("invert_dir", topology.invert_dir.len()),
                ("stepper_oids", topology.stepper_oids.len()),
                ("step_pulse_seconds", topology.step_pulse_seconds.len()),
                (
                    "high_precision_step_compress",
                    topology.high_precision_step_compress.len(),
                ),
            ] {
                if got != motor_count {
                    return Err(KinematicsConfigError::PerMotorVectorLength {
                        handle: topology.mcu_id,
                        field,
                        motor_count,
                        got,
                    });
                }
            }
            let lane_kinds: Vec<LaneKind> = topology
                .lane_kinds
                .iter()
                .map(|&tag| {
                    LaneKind::from_tag(tag).ok_or(KinematicsConfigError::UnknownLaneKind {
                        handle: topology.mcu_id,
                        tag,
                    })
                })
                .collect::<Result<_, _>>()?;
            let ethercat = ethercat_mcu_ids.contains(&topology.mcu_id);
            let move_queue_slots = topology.move_queue_slots;
            let pulse_capable = lane_kinds.iter().any(|k| k.pulse_capable());
            if !ethercat && pulse_capable && move_queue_slots == 0 {
                return Err(KinematicsConfigError::PulseLaneMoveQueueSlots {
                    handle: topology.mcu_id,
                });
            }
            let phase_sample_rate = topology.phase_sample_rate;
            let phase_capable = lane_kinds.iter().any(|k| k.phase_capable());
            if phase_capable && (!phase_sample_rate.is_finite() || phase_sample_rate <= 0.0) {
                return Err(KinematicsConfigError::PhaseLaneSampleRate {
                    handle: topology.mcu_id,
                    rate: phase_sample_rate,
                });
            }
            let phase_ring_depth = topology.phase_ring_depth;
            if phase_capable && phase_ring_depth == 0 {
                return Err(KinematicsConfigError::PhaseLaneRingDepth {
                    handle: topology.mcu_id,
                });
            }
            Ok(McuAxisConfig {
                mcu_id: topology.mcu_id,
                axes,
                kinematics: topology.kinematics,
                lane_kinds,
                max_motor_velocity: topology.max_motor_velocity.clone(),
                ethercat,
                motor_counts: topology.motor_counts.clone(),
                microstep_distance: topology.microstep_distance.clone(),
                invert_dir: topology.invert_dir.clone(),
                stepper_oids: topology.stepper_oids.clone(),
                move_queue_slots,
                step_pulse_seconds: topology.step_pulse_seconds.clone(),
                stepcompress_encoders: topology
                    .high_precision_step_compress
                    .iter()
                    .map(|&enabled| {
                        if enabled {
                            StepcompressEncoder::HighPrecision
                        } else {
                            StepcompressEncoder::Classic
                        }
                    })
                    .collect(),
                phase_sample_rate,
                phase_ring_depth,
                stepcompress_max_error_secs: topology.stepcompress_max_error_secs,
            })
        })
        .collect()
}

pub fn motor_frame(cfg: &McuAxisConfig, axes: [f64; SPATIAL_AXES]) -> [f64; SPATIAL_AXES] {
    crate::kinematics::KinematicsModule::from_tag(cfg.kinematics)
        .expect("build_mcu_configs validated the kinematics tag")
        .forward(axes)
}

/// A follower lane (the extruder) has no spatial coordinate to re-anchor to,
/// so every re-anchor restarts it here: `stream_open` and `home_drip` hand
/// the pipeline a rest position whose follower entry is this origin, and the
/// phase-lane MCU seed zeroes its non-spatial motor positions to match. The
/// host-side holders of that lane's frame — the step shim's counter, the
/// classic MCU counter it seeds, and the retained motion history — take the
/// same value, so the first piece after a re-anchor asks for no displacement
/// at all instead of the whole accumulated extrusion in one sample.
pub const FOLLOWER_REANCHOR_ORIGIN_MM: f64 = 0.0;

pub fn reanchor_home_pos(gcode: geometry::GcodePos) -> [f64; SPATIAL_AXES + 1] {
    [gcode.x(), gcode.y(), gcode.z(), FOLLOWER_REANCHOR_ORIGIN_MM]
}

pub fn reanchor_stream_pos(gcode: geometry::GcodePos) -> Vec<f64> {
    reanchor_home_pos(gcode).to_vec()
}

fn reanchor_axis_mm(motor: &[f64; SPATIAL_AXES], axis: usize) -> f64 {
    motor
        .get(axis)
        .copied()
        .unwrap_or(FOLLOWER_REANCHOR_ORIGIN_MM)
}

/// Rebase targets for a cartesian stop position (a homing/probe trip's
/// result, or any other `SET_KINEMATIC_POSITION`-style external set).
/// The retained motion history is motor frame everywhere else — live pieces
/// are the lowerer's output (e.g. CoreXY A/B) — so a rebase fed raw cartesian
/// would leave axis 0/1 cartesian-valued until the next live piece overwrites
/// them, while every other reader of that axis (including a kinematics
/// inversion) assumes motor frame. The input is [`geometry::MachinePos`]
/// because the ring is post-surface-warp: a caller holding a gcode position
/// must convert through the active mesh first. Same per-cfg transform as
/// `build_serial_seed_sends` uses to seed the MCUs for the same event, and
/// the same follower origin as [`reanchor_stream_pos`].
pub fn reanchor_axis_targets(
    configs: &[McuAxisConfig],
    cartesian: geometry::MachinePos,
) -> Vec<(AxisKey, f64)> {
    configs
        .iter()
        .flat_map(|cfg| {
            let motor = motor_frame(cfg, cartesian.0);
            cfg.axes
                .iter()
                .map(move |&axis| {
                    (
                        AxisKey {
                            mcu_id: cfg.mcu_id,
                            axis: axis as u8,
                        },
                        reanchor_axis_mm(&motor, axis),
                    )
                })
                .collect::<Vec<_>>()
        })
        .collect()
}

/// Per-motor step-counter seeds for one mcu's pulse-capable lanes at a
/// re-anchor, in the order its stepcompress endpoint holds them. Spatial
/// motors take the motor-frame stop position; followers take
/// [`FOLLOWER_REANCHOR_ORIGIN_MM`], the origin the stream odometer restarts
/// them at. A phase-only lane keeps no host step counter, so it contributes
/// nothing; a dual-transport lane does, because its classic counter must stay
/// aligned for the next switch into pulse mode.
pub fn stepcompress_seed_counts(
    cfg: &McuAxisConfig,
    pos: geometry::MachinePos,
) -> Result<Vec<i64>, String> {
    seed_counts(cfg, pos, McuAxisConfig::pulse_capable, "pulse")
}

/// The sample-endpoint counterpart: one seed per phase-capable lane, in the
/// order the sample endpoint holds its lanes.
pub fn sample_seed_counts(
    cfg: &McuAxisConfig,
    pos: geometry::MachinePos,
) -> Result<Vec<i64>, String> {
    seed_counts(cfg, pos, McuAxisConfig::phase_capable, "phase")
}

fn seed_counts(
    cfg: &McuAxisConfig,
    pos: geometry::MachinePos,
    capable: fn(&McuAxisConfig, usize) -> bool,
    what: &str,
) -> Result<Vec<i64>, String> {
    let motor = motor_frame(cfg, pos.0);
    cfg.axes
        .iter()
        .enumerate()
        .filter(|&(lane, _)| capable(cfg, lane))
        .flat_map(|(lane, &axis)| {
            let range = cfg.motor_range(lane);
            let motors: Vec<usize> = if what == "pulse" {
                range.collect()
            } else {
                vec![range.start]
            };
            motors
                .into_iter()
                .map(move |motor_index| (axis, motor_index))
        })
        .map(|(axis, motor_index)| {
            let quantum = *cfg.microstep_distance.get(motor_index).ok_or_else(|| {
                format!(
                    "position seed: mcu {} axis {axis} motor {motor_index} is a {what} lane \
                     with no microstep distance",
                    cfg.mcu_id
                )
            })?;
            if quantum <= 0.0 || !quantum.is_finite() {
                return Err(format!(
                    "position seed: mcu {} axis {axis} motor {motor_index} has microstep \
                     distance {quantum}, which is not a positive length",
                    cfg.mcu_id
                ));
            }
            let mm = reanchor_axis_mm(&motor, axis);
            #[allow(clippy::cast_possible_truncation)]
            Ok((mm / quantum).round() as i64)
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedSend {
    pub mcu_id: u32,
    pub x_q16: i32,
    pub y_q16: i32,
    pub z_q16: i32,
}

pub fn encode_q16(mm: f64) -> i32 {
    let raw = mm * 65536.0;
    raw.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

pub fn build_seed_sends(configs: &[McuAxisConfig], pos: geometry::MachinePos) -> Vec<SeedSend> {
    build_serial_seed_sends(configs, &HashSet::new(), pos)
}

/// MCU step-counter seeds for an externally set position. The counters count
/// physical steps, so the input is [`geometry::MachinePos`]: seeding them
/// from a raw gcode position while a mesh is active shifts the machine frame
/// by `correction_at(x, y)` on every reseed — the contact-probe ratchet.
///
/// Only an mcu carrying phase lanes takes this seed: the sample executor is
/// the one transport whose position the mcu itself owns. A pulse lane's
/// counter lives on the host ([`StepcompressLane::mm_to_steps`] into the
/// shim), and EtherCAT drives are seeded through their own homing finalize.
pub fn build_serial_seed_sends<S: ::std::hash::BuildHasher>(
    configs: &[McuAxisConfig],
    ethercat_mcu_ids: &HashSet<u32, S>,
    pos: geometry::MachinePos,
) -> Vec<SeedSend> {
    let takes_runtime_seed = |cfg: &&McuAxisConfig| {
        !ethercat_mcu_ids.contains(&cfg.mcu_id) && cfg.lane_kinds.contains(&LaneKind::Phase)
    };
    configs
        .iter()
        .filter(takes_runtime_seed)
        .map(|cfg| {
            let m = motor_frame(cfg, pos.0);
            SeedSend {
                mcu_id: cfg.mcu_id,
                x_q16: encode_q16(m[0]),
                y_q16: encode_q16(m[1]),
                z_q16: encode_q16(m[2]),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
