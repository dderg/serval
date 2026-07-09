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

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;

#[derive(Debug, Clone)]
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
}

impl McuAxisConfig {
    #[must_use]
    pub fn motor_velocity_ceiling(&self, axis_idx: usize) -> f64 {
        self.axes
            .iter()
            .position(|&a| a == axis_idx)
            .and_then(|i| self.max_motor_velocity.get(i))
            .copied()
            .unwrap_or(f64::INFINITY)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
}

pub fn build_mcu_configs<S: ::std::hash::BuildHasher>(
    mcus: &[(u32, Vec<u8>, u8, Vec<f64>)],
    caps_by_handle: &HashMap<u32, McuCaps, S>,
) -> Result<Vec<McuAxisConfig>, KinematicsConfigError> {
    mcus.iter()
        .map(|(handle, axes, tag, max_motor_velocity)| {
            crate::kinematics::KinematicsModule::from_tag(*tag).map_err(|_| {
                KinematicsConfigError::UnknownTag {
                    handle: *handle,
                    tag: *tag,
                }
            })?;
            let axes: Vec<usize> = axes.iter().map(|&a| a as usize).collect();
            if *tag == KINEMATICS_COREXY && !(axes.contains(&AXIS_X) && axes.contains(&AXIS_Y)) {
                return Err(KinematicsConfigError::CorexyMissingXy {
                    handle: *handle,
                    axes,
                });
            }
            assert!(
                max_motor_velocity.len() == axes.len(),
                "mcu handle {handle}: {} motor velocity ceilings for {} axes — the \
                 topology tuples must align them",
                max_motor_velocity.len(),
                axes.len(),
            );
            let caps = caps_by_handle
                .get(handle)
                .copied()
                .ok_or(KinematicsConfigError::CapsMissing { handle: *handle })?;
            Ok(McuAxisConfig {
                mcu_id: *handle,
                axes,
                kinematics: *tag,
                caps,
                max_motor_velocity: max_motor_velocity.clone(),
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
pub fn build_serial_seed_sends<S: ::std::hash::BuildHasher>(
    configs: &[McuAxisConfig],
    ethercat_mcu_ids: &HashSet<u32, S>,
    pos: geometry::MachinePos,
) -> Vec<SeedSend> {
    let reachable_over_serial_transport =
        |cfg: &&McuAxisConfig| !ethercat_mcu_ids.contains(&cfg.mcu_id);
    configs
        .iter()
        .filter(reachable_over_serial_transport)
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
            (
                7u32,
                vec![AXIS_X as u8, AXIS_Y as u8, FOLLOWER_E as u8],
                0u8,
                vec![f64::INFINITY; 3],
            ),
            (9u32, vec![AXIS_Z as u8], 1u8, vec![f64::INFINITY]),
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
        assert_eq!(cfgs[1].mcu_id, 9);
        assert_eq!(cfgs[1].axes, vec![AXIS_Z]);
        assert_eq!(cfgs[1].kinematics, 1);
    }

    #[test]
    fn build_mcu_configs_missing_caps_is_an_error() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![(
            7u32,
            vec![AXIS_X as u8, AXIS_Y as u8],
            0u8,
            vec![f64::INFINITY; 2],
        )];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::CapsMissing { handle: 7 }
        ));
    }

    #[test]
    fn build_mcu_configs_unknown_tag_is_loud() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![(7u32, vec![AXIS_X as u8], 9u8, vec![f64::INFINITY])];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::UnknownTag { handle: 7, tag: 9 }
        ));
    }

    #[test]
    fn build_mcu_configs_corexy_without_xy_is_loud() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![(
            7u32,
            vec![AXIS_X as u8, FOLLOWER_E as u8],
            0u8,
            vec![f64::INFINITY; 2],
        )];
        let err = build_mcu_configs(&mcus, &caps).unwrap_err();
        assert!(matches!(
            err,
            KinematicsConfigError::CorexyMissingXy { handle: 7, .. }
        ));
    }
}

#[cfg(test)]
mod seed_tests {
    use super::*;

    const FOLLOWER_E: usize = 3;

    fn corexy_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y, FOLLOWER_E],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
        }
    }
    fn cartesian_z_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
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
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
        };
        let serial_cfg = McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Y, AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
            max_motor_velocity: Vec::new(),
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
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
        };
        let ec_cfg_2 = McuAxisConfig {
            mcu_id: 3,
            axes: vec![AXIS_Y],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
            max_motor_velocity: Vec::new(),
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
