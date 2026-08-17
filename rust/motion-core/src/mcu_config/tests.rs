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
            step_pulse_seconds: vec![2e-6; 3],
            stepcompress_encoder: "hp".to_string(),
            stepcompress_max_error_secs: 0.0,
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
            step_pulse_seconds: vec![2e-6; 1],
            stepcompress_encoder: "hp".to_string(),
            stepcompress_max_error_secs: 0.0,
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
        step_pulse_seconds: vec![2e-6; 2],
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
        step_pulse_seconds: vec![2e-6; 2],
        stepcompress_encoder: "hp".to_string(),
        stepcompress_max_error_secs: 0.0,
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
        step_pulse_seconds: vec![2e-6; 2],
        stepcompress_encoder: "hp".to_string(),
        stepcompress_max_error_secs: 0.0,
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

fn encoder_topology(encoder: &str, max_error_secs: f64) -> Vec<McuTopologyInput> {
    let mut mcus = sample_rate_topology(STEPPING_MODE_STEPCOMPRESS, 20_000.0);
    mcus[0].stepcompress_encoder = encoder.to_string();
    mcus[0].stepcompress_max_error_secs = max_error_secs;
    mcus
}

#[test]
fn unknown_stepcompress_encoder_is_loud() {
    let caps = HashMap::from([(
        7,
        McuCaps {
            total_piece_memory: 62 * 1024,
        },
    )]);
    let err = build_mcu_configs(&encoder_topology("bogus", 0.0), &caps).unwrap_err();
    assert!(matches!(
        err,
        KinematicsConfigError::UnknownStepcompressEncoder { handle: 7, got }
            if got == "bogus"
    ));
}

#[test]
fn stepcompress_encoder_and_max_error_reach_axis_config() {
    let caps = HashMap::from([(
        7,
        McuCaps {
            total_piece_memory: 62 * 1024,
        },
    )]);
    let cfgs = build_mcu_configs(&encoder_topology("classic", 1e-5), &caps).unwrap();
    assert_eq!(cfgs[0].stepcompress_encoder, StepcompressEncoder::Classic);
    assert_eq!(cfgs[0].stepcompress_max_error_secs, 1e-5);
    let cfgs = build_mcu_configs(&encoder_topology("hp", 0.0), &caps).unwrap();
    assert_eq!(
        cfgs[0].stepcompress_encoder,
        StepcompressEncoder::HighPrecision
    );
    assert_eq!(cfgs[0].stepcompress_max_error_secs, 0.0);
}

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
fn reanchor_axis_targets_are_motor_frame_not_cartesian() {
    // A homing/probe trip's stop position (e.g. bed-mesh or z_tilt's
    // per-point probe descend, both ending in toolhead.set_position) is
    // cartesian. On CoreXY the rebased axis-0/1 values must be A/B motor
    // positions — the same frame commit_sent_bundle records live pieces
    // in — not the raw x/y, or a later cartesian-inverting reader (like
    // motion_state_at_clock) double-transforms an already-correct value.
    let configs = vec![corexy_cfg(), cartesian_z_cfg()];
    let targets = reanchor_axis_targets(&configs, geometry::MachinePos([270.0, 5.0, 12.5]));

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
    assert_eq!(
        get(1, FOLLOWER_E as u8),
        FOLLOWER_REANCHOR_ORIGIN_MM,
        "the follower lane has no spatial coordinate: it rebases to the \
         origin the stream odometer restarts it at"
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

fn stepcompress_toolhead_cfg() -> McuAxisConfig {
    McuAxisConfig {
        ethercat: false,
        mcu_id: 1,
        axes: vec![FOLLOWER_E],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 32 * 1024,
        },
        max_motor_velocity: Vec::new(),
        stepping_mode: SteppingMode::Stepcompress,
        microstep_distance: vec![7.73 / (200.0 * 16.0)],
        invert_dir: vec![true],
        stepper_oids: vec![4],
        stepcompress_sample_rate: 10_000.0,
        move_queue_slots: 128,
        step_pulse_seconds: vec![2e-6; 1],
        stepcompress_encoder: StepcompressEncoder::HighPrecision,
        stepcompress_max_error_secs: 0.0,
    }
}

fn stepcompress_corexy_cfg() -> McuAxisConfig {
    McuAxisConfig {
        ethercat: false,
        mcu_id: 2,
        axes: vec![AXIS_X, AXIS_Y, FOLLOWER_E],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
        max_motor_velocity: Vec::new(),
        stepping_mode: SteppingMode::Stepcompress,
        microstep_distance: vec![0.0125, 0.0125, 0.0025],
        invert_dir: vec![false; 3],
        stepper_oids: vec![1, 2, 3],
        stepcompress_sample_rate: 10_000.0,
        move_queue_slots: 128,
        step_pulse_seconds: vec![2e-6; 3],
        stepcompress_encoder: StepcompressEncoder::HighPrecision,
        stepcompress_max_error_secs: 0.0,
    }
}

#[test]
fn stepcompress_seed_counts_put_a_follower_lane_at_the_stream_origin() {
    // The bench crash: 13 mm of net extrusion sat in the retained history
    // while `stream_open` restarted the odometer's follower coordinate at
    // zero, so the seeded shim counter and the first piece after the
    // re-anchor disagreed by the whole extrude — one sample of 18828 steps.
    let cfg = stepcompress_toolhead_cfg();
    let counts = stepcompress_seed_counts(&cfg, geometry::MachinePos([60.0, 60.0, 20.0])).unwrap();
    let lane_origin = reanchor_stream_pos(geometry::GcodePos([60.0, 60.0, 20.0]))[3];
    let lane = crate::homing::stepcompress_lane(
        &cfg,
        AxisKey {
            mcu_id: cfg.mcu_id,
            axis: FOLLOWER_E as u8,
        },
    )
    .unwrap()
    .unwrap();
    assert_eq!(counts, vec![lane.mm_to_steps(lane_origin)]);
    assert_eq!(counts, vec![0]);
}

#[test]
fn stepcompress_seed_counts_are_motor_frame_for_spatial_lanes() {
    let cfg = stepcompress_corexy_cfg();
    let counts = stepcompress_seed_counts(&cfg, geometry::MachinePos([10.0, 4.0, 0.0])).unwrap();
    assert_eq!(
        counts,
        vec![(14.0 / 0.0125) as i64, (6.0 / 0.0125) as i64, 0]
    );
}

#[test]
fn stepcompress_seed_counts_reject_a_piece_mode_config() {
    let err =
        stepcompress_seed_counts(&corexy_cfg(), geometry::MachinePos([1.0, 2.0, 3.0])).unwrap_err();
    assert!(err.contains("no shim lane"), "{err}");
}

#[test]
fn reanchor_targets_share_the_follower_origin_with_the_stream_odometer() {
    let configs = vec![stepcompress_toolhead_cfg(), cartesian_z_cfg()];
    let machine = geometry::MachinePos([60.0, 60.0, 20.0]);
    let targets = reanchor_axis_targets(&configs, machine);
    let follower = targets
        .iter()
        .find(|(key, _)| key.axis == FOLLOWER_E as u8)
        .expect("the follower lane is rebased alongside the spatial ones");
    assert_eq!(
        follower.1,
        reanchor_stream_pos(geometry::GcodePos([60.0, 60.0, 20.0]))[3]
    );
    assert_eq!(
        reanchor_home_pos(geometry::GcodePos([60.0, 60.0, 20.0])),
        [60.0, 60.0, 20.0, FOLLOWER_REANCHOR_ORIGIN_MM]
    );
}
