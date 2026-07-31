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

/// Motor-frame rebase targets for a cartesian stop position (a homing/probe
/// trip's result, or any other `SET_KINEMATIC_POSITION`-style external set).
/// The retained motion history is motor frame everywhere else — live pieces
/// are the lowerer's output (e.g. CoreXY A/B) — so a rebase fed raw cartesian
/// would leave axis 0/1 cartesian-valued until the next live piece overwrites
/// them, while every other reader of that axis (including a kinematics
/// inversion) assumes motor frame. The input is [`geometry::MachinePos`]
/// because the ring is post-surface-warp: a caller holding a gcode position
/// must convert through the active mesh first. Same per-cfg transform as
/// `build_serial_seed_sends` uses to seed the MCUs for the same event.
pub fn spatial_rebase_targets(
    configs: &[McuAxisConfig],
    cartesian: geometry::MachinePos,
) -> Vec<(AxisKey, f64)> {
    configs
        .iter()
        .flat_map(|cfg| {
            let motor = motor_frame(cfg, cartesian.0);
            cfg.axes
                .iter()
                .filter(|&&a| a < SPATIAL_AXES)
                .map(move |&axis| {
                    (
                        AxisKey {
                            mcu_id: cfg.mcu_id,
                            axis: axis as u8,
                        },
                        motor[axis],
                    )
                })
                .collect::<Vec<_>>()
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
mod topology_tests {
    use super::*;
    use std::collections::HashMap;

    const FOLLOWER_E: usize = 3;

    #[test]
    fn build_mcu_configs_two_mcu_corexy_with_e() {
        let mut caps = HashMap::new();
        caps.insert(
            7u32,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        );
        caps.insert(
            9u32,
            McuCaps {
                total_piece_memory: 32 * 1024,
            },
        );
        let mcus = vec![
            McuTopologyInput {
                mcu_id: 7,
                axes: vec![AXIS_X as u8, AXIS_Y as u8, FOLLOWER_E as u8],
                kinematics: 0,
                max_motor_velocity: vec![f64::INFINITY; 3],
                stepping_mode: STEPPING_MODE_PIECE,
                microstep_distance: vec![0.0125; 3],
                invert_dir: vec![false; 3],
                stepper_oids: vec![1, 2, 3],
                stepcompress_sample_rate: 0.0,
                move_queue_slots: 0,
            },
            McuTopologyInput {
                mcu_id: 9,
                axes: vec![AXIS_Z as u8],
                kinematics: 1,
                max_motor_velocity: vec![f64::INFINITY],
                stepping_mode: STEPPING_MODE_STEPCOMPRESS,
                microstep_distance: vec![0.0025],
                invert_dir: vec![true],
                stepper_oids: vec![4],
                stepcompress_sample_rate: 20_000.0,
                move_queue_slots: 128,
            },
        ];
        let cfgs = build_mcu_configs(&mcus, &caps).unwrap();
        assert_eq!(cfgs.len(), 2);
        assert_eq!(cfgs[0].mcu_id, 7);
        assert_eq!(cfgs[0].axes, vec![AXIS_X, AXIS_Y, FOLLOWER_E]);
        assert_eq!(cfgs[0].kinematics, 0);
        assert_eq!(
            cfgs[0].caps,
            McuCaps {
                total_piece_memory: 62 * 1024
            }
        );
        assert_eq!(cfgs[0].stepping_mode, SteppingMode::Piece);
        assert_eq!(cfgs[0].stepper_oids, vec![1, 2, 3]);
        assert_eq!(cfgs[1].mcu_id, 9);
        assert_eq!(cfgs[1].axes, vec![AXIS_Z]);
        assert_eq!(cfgs[1].kinematics, 1);
        assert_eq!(cfgs[1].stepping_mode, SteppingMode::Stepcompress);
        assert_eq!(cfgs[1].invert_dir, vec![true]);
    }

    #[test]
    fn build_mcu_configs_missing_caps_is_an_error() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, AXIS_Y as u8],
            kinematics: 0,
            max_motor_velocity: vec![f64::INFINITY; 2],
            microstep_distance: vec![0.0125; 2],
            invert_dir: vec![false; 2],
            stepper_oids: vec![1, 2],
            ..Default::default()
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::CapsMissing { handle: 7 }
        ));
    }

    #[test]
    fn build_mcu_configs_unknown_tag_is_loud() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8],
            kinematics: 9,
            max_motor_velocity: vec![f64::INFINITY],
            ..Default::default()
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::UnknownTag { handle: 7, tag: 9 }
        ));
    }

    #[test]
    fn build_mcu_configs_corexy_without_xy_is_loud() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, FOLLOWER_E as u8],
            kinematics: 0,
            max_motor_velocity: vec![f64::INFINITY; 2],
            ..Default::default()
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::CorexyMissingXy { handle: 7, .. }
        ));
    }

    #[test]
    fn build_mcu_configs_requires_one_velocity_ceiling_per_axis() {
        let caps = HashMap::from([(
            7,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        )]);
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, AXIS_Y as u8],
            kinematics: KINEMATICS_COREXY,
            max_motor_velocity: vec![100.0],
            ..Default::default()
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::VelocityCeilingCount {
                handle: 7,
                axis_count: 2,
                ceiling_count: 1,
            }
        ));
    }

    #[test]
    fn build_mcu_configs_requires_one_microstep_distance_per_axis() {
        let caps = HashMap::from([(
            7,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        )]);
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, AXIS_Y as u8],
            kinematics: KINEMATICS_COREXY,
            max_motor_velocity: vec![100.0, 100.0],
            microstep_distance: vec![0.0125],
            invert_dir: vec![false; 2],
            stepper_oids: vec![1, 2],
            ..Default::default()
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::PerAxisVectorLength {
                handle: 7,
                field: "microstep_distance",
                axis_count: 2,
                got: 1,
            }
        ));
    }

    #[test]
    fn build_mcu_configs_unknown_stepping_mode_is_loud() {
        let caps = HashMap::from([(
            7,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        )]);
        let mcus = vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, AXIS_Y as u8],
            kinematics: KINEMATICS_COREXY,
            max_motor_velocity: vec![100.0, 100.0],
            stepping_mode: 7,
            microstep_distance: vec![0.0125; 2],
            invert_dir: vec![false; 2],
            stepper_oids: vec![1, 2],
            stepcompress_sample_rate: 0.0,
            move_queue_slots: 0,
        }];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::UnknownSteppingMode { handle: 7, tag: 7 }
        ));
    }

    fn sample_rate_topology(stepping_mode: u8, rate: f64) -> Vec<McuTopologyInput> {
        vec![McuTopologyInput {
            mcu_id: 7,
            axes: vec![AXIS_X as u8, AXIS_Y as u8],
            kinematics: KINEMATICS_COREXY,
            max_motor_velocity: vec![100.0, 100.0],
            stepping_mode,
            microstep_distance: vec![0.0125; 2],
            invert_dir: vec![false; 2],
            stepper_oids: vec![1, 2],
            stepcompress_sample_rate: rate,
            move_queue_slots: if stepping_mode == STEPPING_MODE_STEPCOMPRESS {
                128
            } else {
                0
            },
        }]
    }

    #[test]
    fn stepcompress_without_sample_rate_is_loud() {
        let caps = HashMap::from([(
            7,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        )]);
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mcus = sample_rate_topology(STEPPING_MODE_STEPCOMPRESS, rate);
            let err = build_mcu_configs(&mcus, &caps).unwrap_err();
            assert!(matches!(
                err,
                KinematicsConfigError::StepcompressSampleRate { handle: 7, .. }
            ));
        }
        let cfgs = build_mcu_configs(
            &sample_rate_topology(STEPPING_MODE_STEPCOMPRESS, 20_000.0),
            &caps,
        )
        .unwrap();
        assert_eq!(cfgs[0].stepcompress_sample_rate, 20_000.0);
    }

    #[test]
    fn piece_mode_rejects_nonzero_sample_rate() {
        let caps = HashMap::from([(
            7,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        )]);
        let err =
            build_mcu_configs(&sample_rate_topology(STEPPING_MODE_PIECE, 1.0), &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::PieceSampleRate { handle: 7, .. }
        ));
    }
}

#[cfg(test)]
mod seed_tests {
    use super::*;

    const FOLLOWER_E: usize = 3;

    fn corexy_cfg() -> McuAxisConfig {
        McuAxisConfig {
            ethercat: false,
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y, FOLLOWER_E],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        }
    }
    fn cartesian_z_cfg() -> McuAxisConfig {
        McuAxisConfig {
            ethercat: false,
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        }
    }

    #[test]
    fn motor_frame_transforms_corexy_passes_through_cartesian() {
        assert_eq!(
            motor_frame(&corexy_cfg(), [150.0, 150.0, 0.0]),
            [300.0, 0.0, 0.0]
        );
        assert_eq!(
            motor_frame(&corexy_cfg(), [10.0, 4.0, 0.0]),
            [14.0, 6.0, 0.0]
        );
        assert_eq!(
            motor_frame(&cartesian_z_cfg(), [150.0, 150.0, 50.0]),
            [150.0, 150.0, 50.0]
        );
    }

    #[test]
    fn spatial_rebase_targets_are_motor_frame_not_cartesian() {
        // A homing/probe trip's stop position (e.g. bed-mesh or z_tilt's
        // per-point probe descend, both ending in toolhead.set_position) is
        // cartesian. On CoreXY the rebased axis-0/1 values must be A/B motor
        // positions — the same frame commit_sent_bundle records live pieces
        // in — not the raw x/y, or a later cartesian-inverting reader (like
        // motion_state_at_clock) double-transforms an already-correct value.
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let targets = spatial_rebase_targets(&configs, geometry::MachinePos([270.0, 5.0, 12.5]));

        let get = |mcu_id: u32, axis: u8| {
            targets
                .iter()
                .find(|(k, _)| k.mcu_id == mcu_id && k.axis == axis)
                .unwrap_or_else(|| panic!("no rebase target for mcu {mcu_id} axis {axis}"))
                .1
        };
        assert!((get(1, 0) - 275.0).abs() < 1e-9, "motor0 (x+y)");
        assert!((get(1, 1) - 265.0).abs() < 1e-9, "motor1 (x-y)");
        assert!((get(2, 2) - 12.5).abs() < 1e-9, "z passes through");
        assert!(
            !targets
                .iter()
                .any(|(k, _)| k.mcu_id == 1 && k.axis as usize == FOLLOWER_E),
            "the extruder is not a spatial axis and must not be rebased here"
        );
    }

    #[test]
    fn encode_q16_is_mm_times_65536_rounded() {
        assert_eq!(encode_q16(0.0), 0);
        assert_eq!(encode_q16(50.0), 3_276_800);
        assert_eq!(encode_q16(150.0), 9_830_400);
        assert_eq!(encode_q16(300.0), 19_660_800);
    }

    #[test]
    fn build_seed_sends_applies_per_mcu_transform() {
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let sends = build_seed_sends(&configs, geometry::MachinePos([150.0, 150.0, 50.0]));
        assert_eq!(sends.len(), 2);

        let octo = sends.iter().find(|s| s.mcu_id == 1).expect("octopus seed");
        assert_eq!(octo.x_q16, encode_q16(300.0));
        assert_eq!(octo.y_q16, encode_q16(0.0));
        assert_eq!(octo.z_q16, encode_q16(50.0));

        let z = sends.iter().find(|s| s.mcu_id == 2).expect("f446 seed");
        assert_eq!(z.x_q16, encode_q16(150.0));
        assert_eq!(z.y_q16, encode_q16(150.0));
        assert_eq!(z.z_q16, encode_q16(50.0));
    }

    #[test]
    fn build_serial_seed_sends_skips_ethercat_node() {
        let ec_cfg = McuAxisConfig {
            ethercat: false,
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        };
        let serial_cfg = McuAxisConfig {
            ethercat: false,
            mcu_id: 2,
            axes: vec![AXIS_Y, AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        };
        let configs = vec![ec_cfg, serial_cfg];
        let ethercat_mcu_ids: HashSet<u32> = [1u32].into_iter().collect();

        let sends = build_serial_seed_sends(
            &configs,
            &ethercat_mcu_ids,
            geometry::MachinePos([100.0, 50.0, 10.0]),
        );

        assert!(
            sends.iter().all(|s| s.mcu_id != 1),
            "EtherCAT mcu_id=1 must not appear in serial seed sends; got: {sends:?}"
        );
        assert_eq!(
            sends.len(),
            1,
            "exactly one send for the serial MCU; got {sends:?}"
        );
        let serial = &sends[0];
        assert_eq!(serial.mcu_id, 2);
        assert_eq!(serial.x_q16, encode_q16(100.0));
        assert_eq!(serial.y_q16, encode_q16(50.0));
        assert_eq!(serial.z_q16, encode_q16(10.0));
    }

    #[test]
    fn build_serial_seed_sends_skips_stepcompress_mcu() {
        let sc_cfg = McuAxisConfig {
            ethercat: false,
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
            stepping_mode: SteppingMode::Stepcompress,
            ..Default::default()
        };
        let piece_cfg = McuAxisConfig {
            ethercat: false,
            mcu_id: 2,
            axes: vec![AXIS_Y, AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        };
        let sends = build_serial_seed_sends(
            &[sc_cfg, piece_cfg],
            &HashSet::<u32>::new(),
            geometry::MachinePos([100.0, 50.0, 10.0]),
        );
        assert_eq!(sends.len(), 1, "got {sends:?}");
        assert_eq!(sends[0].mcu_id, 2);
    }

    #[test]
    fn build_serial_seed_sends_all_serial_matches_build_seed_sends() {
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let ethercat_mcu_ids: HashSet<u32> = HashSet::new();
        let serial_sends = build_serial_seed_sends(
            &configs,
            &ethercat_mcu_ids,
            geometry::MachinePos([150.0, 150.0, 50.0]),
        );
        let full_sends = build_seed_sends(&configs, geometry::MachinePos([150.0, 150.0, 50.0]));
        assert_eq!(
            serial_sends, full_sends,
            "with no EtherCAT nodes, build_serial_seed_sends must match build_seed_sends"
        );
    }

    #[test]
    fn build_serial_seed_sends_all_ethercat_returns_empty() {
        let ec_cfg_1 = McuAxisConfig {
            ethercat: false,
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        };
        let ec_cfg_2 = McuAxisConfig {
            ethercat: false,
            mcu_id: 3,
            axes: vec![AXIS_Y],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
            ..Default::default()
        };
        let configs = vec![ec_cfg_1, ec_cfg_2];
        let ethercat_mcu_ids: HashSet<u32> = [1u32, 3u32].into_iter().collect();
        let sends = build_serial_seed_sends(
            &configs,
            &ethercat_mcu_ids,
            geometry::MachinePos([100.0, 50.0, 10.0]),
        );
        assert!(
            sends.is_empty(),
            "all-EtherCAT topology must produce zero serial seed sends; got {sends:?}"
        );
    }
}
