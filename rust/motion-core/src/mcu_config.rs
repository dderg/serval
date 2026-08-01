use crate::kinematics::SPATIAL_AXES;
use crate::types::AxisKey;
use runtime::segment::KinematicTag;
use std::collections::{HashMap, HashSet};

pub const KINEMATICS_COREXY: u8 = KinematicTag::CoreXy as u8;

const _: () = assert!(
    KinematicTag::CoreXy as u8 == 0,
    "KinematicTag::CoreXy discriminant must be 0 — the Python↔Rust init_planner \
     topology tuples mirror it numerically (see segment.rs); renumbering breaks that contract",
);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SteppingMode {
    #[default]
    Piece = 0,
    Stepcompress = 1,
}

pub const STEPPING_MODE_PIECE: u8 = SteppingMode::Piece as u8;
pub const STEPPING_MODE_STEPCOMPRESS: u8 = SteppingMode::Stepcompress as u8;

const _: () = assert!(
    STEPPING_MODE_PIECE == 0 && STEPPING_MODE_STEPCOMPRESS == 1,
    "SteppingMode discriminants are mirrored numerically by the klippy/mcu.py \
     STEPPING_MODES table the init_planner topology tuples carry; renumbering \
     breaks that contract",
);

impl SteppingMode {
    #[must_use]
    pub fn from_tag(tag: u8) -> Option<Self> {
        match tag {
            STEPPING_MODE_PIECE => Some(Self::Piece),
            STEPPING_MODE_STEPCOMPRESS => Some(Self::Stepcompress),
            _ => None,
        }
    }
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
    pub stepping_mode: u8,
    pub microstep_distance: Vec<f64>,
    pub invert_dir: Vec<bool>,
    pub stepper_oids: Vec<u32>,
    pub stepcompress_sample_rate: f64,
    pub move_queue_slots: u32,
}

#[derive(Debug, Clone, Default)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    pub axes: Vec<usize>,
    pub kinematics: u8,
    pub caps: McuCaps,
    /// Motor-frame velocity ceiling (mm/s) per entry of `axes`: the fastest
    /// this axis's MCU can physically emit steps. Tracks are validated
    /// against it at enqueue so an overspeed track fails loud on the host
    /// instead of latching -310 on the MCU.
    pub max_motor_velocity: Vec<f64>,
    /// Slots served by the ethercat-rt endpoint: torque-gated drives whose
    /// rings must stay empty while parked, so pure-hold lanes are never
    /// enqueued for them.
    pub ethercat: bool,
    pub stepping_mode: SteppingMode,
    pub microstep_distance: Vec<f64>,
    pub invert_dir: Vec<bool>,
    pub stepper_oids: Vec<u32>,
    pub stepcompress_sample_rate: f64,
    pub move_queue_slots: u32,
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct McuCaps {
    pub total_piece_memory: u32,
}

impl From<mcu_protocol::messages::RuntimeCapsResponse> for McuCaps {
    fn from(r: mcu_protocol::messages::RuntimeCapsResponse) -> Self {
        Self {
            total_piece_memory: r.total_piece_memory,
        }
    }
}

impl McuCaps {
    pub fn total_pieces(&self) -> usize {
        self.total_piece_memory as usize / core::mem::size_of::<runtime::piece_ring::PieceEntry>()
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
        "no runtime caps recorded for mcu_handle {handle} — \
         refusing to size piece rings by guess"
    )]
    CapsMissing { handle: u32 },
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
        "mcu handle {handle}: unknown stepping_mode tag {tag}; \
         known: {STEPPING_MODE_PIECE}=piece, {STEPPING_MODE_STEPCOMPRESS}=stepcompress"
    )]
    UnknownSteppingMode { handle: u32, tag: u8 },
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
        "mcu handle {handle}: stepping_mode: stepcompress requires a finite positive \
         stepcompress_sample_rate (Hz), got {rate}"
    )]
    StepcompressSampleRate { handle: u32, rate: f64 },
    #[error(
        "mcu handle {handle}: stepping_mode: piece must carry \
         stepcompress_sample_rate 0.0, got {rate}"
    )]
    PieceSampleRate { handle: u32, rate: f64 },
    #[error(
        "mcu handle {handle}: stepping_mode: stepcompress requires the mcu's advertised \
         move_count (move_queue_slots) to be positive, got 0"
    )]
    StepcompressMoveQueueSlots { handle: u32 },
    #[error("mcu handle {handle}: stepping_mode: piece must carry move_queue_slots 0, got {slots}")]
    PieceMoveQueueSlots { handle: u32, slots: u32 },
}

pub fn build_mcu_configs<S: ::std::hash::BuildHasher>(
    mcus: &[McuTopologyInput],
    caps_by_handle: &HashMap<u32, McuCaps, S>,
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
            let stepping_mode = SteppingMode::from_tag(topology.stepping_mode).ok_or(
                KinematicsConfigError::UnknownSteppingMode {
                    handle: topology.mcu_id,
                    tag: topology.stepping_mode,
                },
            )?;
            for (field, got) in [
                ("microstep_distance", topology.microstep_distance.len()),
                ("invert_dir", topology.invert_dir.len()),
                ("stepper_oids", topology.stepper_oids.len()),
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
            let rate = topology.stepcompress_sample_rate;
            let move_queue_slots = topology.move_queue_slots;
            match stepping_mode {
                SteppingMode::Stepcompress => {
                    if !rate.is_finite() || rate <= 0.0 {
                        return Err(KinematicsConfigError::StepcompressSampleRate {
                            handle: topology.mcu_id,
                            rate,
                        });
                    }
                    if move_queue_slots == 0 {
                        return Err(KinematicsConfigError::StepcompressMoveQueueSlots {
                            handle: topology.mcu_id,
                        });
                    }
                }
                SteppingMode::Piece => {
                    if rate != 0.0 {
                        return Err(KinematicsConfigError::PieceSampleRate {
                            handle: topology.mcu_id,
                            rate,
                        });
                    }
                    if move_queue_slots != 0 {
                        return Err(KinematicsConfigError::PieceMoveQueueSlots {
                            handle: topology.mcu_id,
                            slots: move_queue_slots,
                        });
                    }
                }
            }
            let caps = caps_by_handle.get(&topology.mcu_id).copied().ok_or(
                KinematicsConfigError::CapsMissing {
                    handle: topology.mcu_id,
                },
            )?;
            Ok(McuAxisConfig {
                mcu_id: topology.mcu_id,
                axes,
                kinematics: topology.kinematics,
                caps,
                max_motor_velocity: topology.max_motor_velocity.clone(),
                ethercat: false,
                stepping_mode,
                microstep_distance: topology.microstep_distance.clone(),
                invert_dir: topology.invert_dir.clone(),
                stepper_oids: topology.stepper_oids.clone(),
                stepcompress_sample_rate: rate,
                move_queue_slots,
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
/// piece-mode MCU seed zeroes its non-spatial motor positions to match. The
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

/// Per-motor step-counter seeds for one stepcompress MCU at a re-anchor, in
/// the MCU's own motor order. Spatial motors take the motor-frame stop
/// position; followers take [`FOLLOWER_REANCHOR_ORIGIN_MM`], the origin the
/// stream odometer restarts them at.
pub fn stepcompress_seed_counts(
    cfg: &McuAxisConfig,
    pos: geometry::MachinePos,
) -> Result<Vec<i64>, String> {
    let motor = motor_frame(cfg, pos.0);
    cfg.axes
        .iter()
        .map(|&axis| {
            let key = AxisKey {
                mcu_id: cfg.mcu_id,
                axis: axis as u8,
            };
            let lane = crate::homing::stepcompress_lane(cfg, key)?.ok_or_else(|| {
                format!(
                    "position seed: stepcompress mcu {} axis {axis} has no shim lane",
                    cfg.mcu_id
                )
            })?;
            Ok(lane.mm_to_steps(reanchor_axis_mm(&motor, axis)))
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
/// Stepcompress MCUs are excluded alongside EtherCAT ones: classic stepping
/// has no MCU-side "set position" command by design — the host owns the step
/// counter and the MCU only reports it back via `stepper_get_position`. Their
/// seed is [`StepcompressLane::mm_to_steps`] into the host shim.
pub fn build_serial_seed_sends<S: ::std::hash::BuildHasher>(
    configs: &[McuAxisConfig],
    ethercat_mcu_ids: &HashSet<u32, S>,
    pos: geometry::MachinePos,
) -> Vec<SeedSend> {
    let takes_runtime_seed = |cfg: &&McuAxisConfig| {
        !ethercat_mcu_ids.contains(&cfg.mcu_id) && cfg.stepping_mode != SteppingMode::Stepcompress
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
