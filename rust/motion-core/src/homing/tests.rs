use crate::lock_ext::LockExt;
use std::sync::{Arc, Mutex};

use runtime::piece_ring::PieceEntry;

use host_rt::passthrough_queue::PassthroughRouter;

use crate::homing::{
    STALE_TRIP_HARD_LIMIT_S, reconstruct_axis_position, trajectory_final_position,
};
use crate::mcu_config::{AXIS_X, AXIS_Z};
use crate::motion_history::HistoryStore;
use crate::types::AxisKey;

const FREQ: f64 = 180_000_000.0;

fn make_linear_piece(
    start_time: u64,
    duration_secs: f32,
    pos_start: f32,
    pos_end: f32,
) -> PieceEntry {
    let mut coeffs = [0.0_f32; runtime::piece_ring::MAX_PIECE_COEFFS];
    coeffs[0] = (pos_start + pos_end) / 2.0;
    coeffs[1] = (pos_end - pos_start) / 2.0;
    PieceEntry {
        start_time,
        coeffs,
        duration: duration_secs,
        motor_mask: 0,
        coeff_count: 2,
        ..PieceEntry::zeroed()
    }
}

fn router_with_clock(mcu_id: u32, freq: f64) -> Arc<Mutex<PassthroughRouter>> {
    let clock: Arc<dyn host_rt::clock::Clock + Send + Sync> = Arc::new(host_rt::clock::RealClock);
    let mut router = PassthroughRouter::with_clock(clock);
    for i in 0..mcu_id {
        let _ = router.claim_mcu(&format!("dummy-{i}"));
    }
    let handle = router.claim_mcu(&format!("mcu-{mcu_id}"));
    assert_eq!(
        handle.raw(),
        mcu_id,
        "handle must equal mcu_id for test correctness"
    );
    let _ =
        router.set_clock_est_from_sample(handle, freq, std::time::Instant::now(), 1_000_000_000);
    Arc::new(Mutex::new(router))
}

fn shared(store: HistoryStore) -> Arc<Mutex<HistoryStore>> {
    Arc::new(Mutex::new(store))
}

fn host_of(router: &Arc<Mutex<PassthroughRouter>>, mcu_id: u32, clock: u64) -> f64 {
    router
        .lock_ok()
        .clock_to_host_secs(crate::types::mcu_handle_from_raw(mcu_id), clock)
        .expect("test router must resolve clock_to_host_secs")
}

fn record_synced(
    store: &mut HistoryStore,
    router: &Arc<Mutex<PassthroughRouter>>,
    key: AxisKey,
    e: &PieceEntry,
    freq: f64,
) {
    let host = host_of(router, key.mcu_id, e.start_time);
    store.record(key, e, freq, host);
}

#[test]
fn same_mcu_trip_clock_exact_reconstruction() {
    const MCU_ID: u32 = 1;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let piece_start: u64 = 1_000_000;
    let duration_secs: f32 = 0.025;
    #[allow(clippy::cast_possible_truncation)]
    let duration_ticks = (duration_secs as f64 * FREQ_F64) as u64;

    let piece = make_linear_piece(piece_start, duration_secs, 0.0, 50.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(&mut store, &router, key, &piece, FREQ);

    let trip_clock = piece_start + duration_ticks / 2;

    let result = reconstruct_axis_position(
        MCU_ID,
        trip_clock,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    );
    let pos = result.expect("same-MCU reconstruction must succeed");

    assert!(
        (pos - 25.0).abs() < 0.5,
        "midpoint of 0..50mm piece should be ~25mm, got {pos:.4}"
    );
}

#[test]
fn trip_at_piece_start_returns_start_position() {
    const MCU_ID: u32 = 2;
    const FREQ_F64: f64 = 520_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let piece_start: u64 = 5_000_000_000;
    let piece = make_linear_piece(piece_start, 0.025, 10.0, 30.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_Z as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(&mut store, &router, key, &piece, 520_000_000.0_f64);

    let result = reconstruct_axis_position(
        MCU_ID,
        piece_start,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    );
    let pos = result.expect("trip at piece start must succeed");
    assert!(
        (pos - 10.0).abs() < 0.5,
        "expected start position 10mm, got {pos:.4}"
    );
}

#[test]
fn trip_outside_trajectory_window_holds_last_position() {
    const MCU_ID: u32 = 3;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let piece_start: u64 = 1_000_000;
    let duration_secs: f32 = 0.025;
    let piece = make_linear_piece(piece_start, duration_secs, 0.0, 10.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(&mut store, &router, key, &piece, FREQ);

    #[allow(clippy::cast_possible_truncation)]
    let way_after = piece_start + (duration_secs as f64 * FREQ_F64) as u64 + 9_999_999;
    let result = reconstruct_axis_position(
        MCU_ID,
        way_after,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    );
    let pos = result.expect("trip after last piece holds endpoint position");
    assert!(
        (pos - 10.0).abs() < 0.5,
        "expected endpoint 10mm, got {pos:.4}"
    );
}

#[test]
fn trip_before_the_first_recorded_piece_clamps_to_run_start() {
    const MCU_ID: u32 = 4;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let piece_start: u64 = 1_000_000_000;
    let piece = make_linear_piece(piece_start, 0.025, 0.0, 10.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(&mut store, &router, key, &piece, FREQ);

    let before = piece_start - 1;
    let pos = reconstruct_axis_position(
        MCU_ID,
        before,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    )
    .expect("a trip before the axis's first-ever piece clamps to run start");
    assert!((pos + 777.0).abs() < 1e-12, "expected -777.0, got {pos}");
}

#[test]
fn trip_before_a_reanchor_hold_with_prior_motion_errors() {
    const MCU_ID: u32 = 4;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(
        &mut store,
        &router,
        key,
        &make_linear_piece(1_000_000_000, 0.025, 0.0, 10.0),
        FREQ,
    );
    store.drop_pieces_on_reanchor();
    record_synced(
        &mut store,
        &router,
        key,
        &make_linear_piece(3_000_000_000, 0.025, 10.0, 20.0),
        FREQ,
    );

    let before_all = 500_000_000;
    let err = reconstruct_axis_position(
        MCU_ID,
        before_all,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    )
    .unwrap_err();
    assert!(
        err.contains("precedes retained"),
        "prior motion existed: the clamp must not swallow this, got: {err}"
    );
}

#[test]
fn no_history_for_axis_clamps_to_run_start() {
    const MCU_ID: u32 = 5;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let store = HistoryStore::default();

    let pos = reconstruct_axis_position(
        MCU_ID,
        12_345_678,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        42.5,
    )
    .expect("an axis with no motion since attach clamps to the run start");
    assert!((pos - 42.5).abs() < 1e-12, "expected 42.5, got {pos}");
}

#[test]
fn multiple_pieces_trip_in_second_piece() {
    const MCU_ID: u32 = 6;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let duration_secs: f32 = 0.025;
    #[allow(clippy::cast_possible_truncation)]
    let duration_ticks = (duration_secs as f64 * FREQ_F64) as u64;

    let piece1_start: u64 = 1_000_000;
    let piece2_start = piece1_start + duration_ticks;

    let piece1 = make_linear_piece(piece1_start, duration_secs, 0.0, 50.0);
    let piece2 = make_linear_piece(piece2_start, duration_secs, 50.0, 100.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(&mut store, &router, key, &piece1, FREQ);
    record_synced(&mut store, &router, key, &piece2, FREQ);

    let trip_clock = piece2_start + duration_ticks / 2;
    let result = reconstruct_axis_position(
        MCU_ID,
        trip_clock,
        key,
        &router,
        &shared(store),
        f64::NEG_INFINITY,
        -777.0,
    );
    let pos = result.expect("trip in second piece must succeed");
    assert!(
        (pos - 75.0).abs() < 1.0,
        "midpoint of 50..100mm second piece should be ~75mm, got {pos:.4}"
    );
}

#[test]
fn trip_just_before_the_arm_window_clamps_to_run_start() {
    const MCU_ID: u32 = 7;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(
        &mut store,
        &router,
        key,
        &make_linear_piece(1_000_000, 0.01, 0.0, 5.0),
        FREQ,
    );

    let window_start_host = host_of(&router, MCU_ID, 2_000_000);
    let pos = reconstruct_axis_position(
        MCU_ID,
        1_500_000,
        key,
        &router,
        &shared(store),
        window_start_host,
        3.25,
    )
    .expect("a trip within jitter of the arm window clamps to the run start");
    assert!((pos - 3.25).abs() < 1e-12, "expected 3.25, got {pos}");
}

#[test]
fn trip_a_second_before_the_arm_window_is_rejected() {
    const MCU_ID: u32 = 7;
    const FREQ_F64: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ_F64);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut store = HistoryStore::default();
    record_synced(
        &mut store,
        &router,
        key,
        &make_linear_piece(1_000_000, 0.01, 0.0, 5.0),
        FREQ,
    );

    let trip_clock = 1_500_000;
    let window_start_host = host_of(&router, MCU_ID, trip_clock) + STALE_TRIP_HARD_LIMIT_S + 0.001;
    let err = reconstruct_axis_position(
        MCU_ID,
        trip_clock,
        key,
        &router,
        &shared(store),
        window_start_host,
        3.25,
    )
    .unwrap_err();
    assert!(
        err.contains("mis-synced clock"),
        "error must name the broken clock model, got: {err}"
    );
}

#[test]
fn trajectory_final_position_single_piece() {
    let key = AxisKey {
        mcu_id: 10,
        axis: AXIS_X as u8,
    };
    let piece = make_linear_piece(1_000_000, 0.025, 5.0, 45.0);
    let mut store = HistoryStore::default();
    store.record(key, &piece, FREQ, 0.0);

    let pos =
        trajectory_final_position(key, &shared(store)).expect("single-piece store must succeed");
    assert!(
        (pos - 45.0).abs() < 1e-4,
        "final position must equal piece end position 45.0, got {pos:.6}"
    );
}

#[test]
fn trajectory_final_position_multi_piece_takes_last() {
    let key = AxisKey {
        mcu_id: 11,
        axis: AXIS_X as u8,
    };
    let piece1 = make_linear_piece(1_000_000, 0.025, 0.0, 50.0);
    let piece2 = make_linear_piece(5_500_000, 0.025, 50.0, 82.5);
    let mut store = HistoryStore::default();
    store.record(key, &piece1, FREQ, 0.0);
    store.record(key, &piece2, FREQ, 1.0);

    let pos =
        trajectory_final_position(key, &shared(store)).expect("multi-piece store must succeed");
    assert!(
        (pos - 82.5).abs() < 1e-4,
        "final position must equal last piece's end position 82.5, got {pos:.6}"
    );
}

#[test]
fn trajectory_final_position_missing_key_errors() {
    let key = AxisKey {
        mcu_id: 12,
        axis: AXIS_X as u8,
    };
    let store = HistoryStore::default();

    let result = trajectory_final_position(key, &shared(store));
    assert!(
        result.is_err(),
        "missing key must return Err, got: {result:?}"
    );
    let msg = result.unwrap_err();
    assert!(
        msg.contains("no recorded motion") || msg.contains("no trajectory"),
        "error must mention missing pieces, got: {msg}"
    );
}

#[test]
fn trajectory_final_position_constant_piece() {
    let key = AxisKey {
        mcu_id: 13,
        axis: AXIS_Z as u8,
    };
    let piece = runtime::piece_ring::PieceEntry {
        start_time: 0,
        duration: 0.01,
        coeff_count: 1,
        coeffs: {
            let mut c = [0.0_f32; runtime::piece_ring::MAX_PIECE_COEFFS];
            c[0] = 99.0;
            c
        },
        ..runtime::piece_ring::PieceEntry::zeroed()
    };
    let mut store = HistoryStore::default();
    store.record(key, &piece, FREQ, 0.0);

    let pos =
        trajectory_final_position(key, &shared(store)).expect("constant-piece store must succeed");
    assert!(
        (pos - 99.0).abs() < 1e-4,
        "constant piece endpoint must be 99.0, got {pos:.6}"
    );
}

mod drive_fault_routing_tests {
    use crate::homing::{DriveFaultRoute, route_drive_fault};

    #[test]
    fn homing_active_on_faulting_mcu_routes_to_homing_error() {
        assert_eq!(route_drive_fault(7, Some(7)), DriveFaultRoute::HomingError);
    }

    #[test]
    fn homing_on_other_mcu_latches_for_klippy() {
        assert_eq!(
            route_drive_fault(7, Some(3)),
            DriveFaultRoute::LatchForKlippy
        );
    }

    #[test]
    fn idle_fault_latches_for_klippy() {
        assert_eq!(route_drive_fault(7, None), DriveFaultRoute::LatchForKlippy);
    }
}

mod broadcast_stop_tests {
    use crate::homing::broadcast_stop;
    use mcu_protocol::messages::StopResponse;
    use std::collections::HashSet;

    #[test]
    fn collects_discard_clock_from_the_axis_mcu() {
        let ids: HashSet<u32> = [1, 2].into_iter().collect();
        let clock = broadcast_stop(&ids, 2, |mcu_id| {
            Ok(StopResponse {
                result: 0,
                discard_clock: u64::from(mcu_id) * 100,
            })
        })
        .unwrap();
        assert_eq!(clock, 200);
    }

    #[test]
    fn missing_transport_fails_loudly() {
        let ids: HashSet<u32> = [1, 7].into_iter().collect();
        let err = broadcast_stop(&ids, 1, |mcu_id| {
            if mcu_id == 7 {
                Err("Stop: no transport for mcu 7".to_owned())
            } else {
                Ok(StopResponse {
                    result: 0,
                    discard_clock: 42,
                })
            }
        })
        .unwrap_err();
        assert!(err.contains("no transport for mcu 7"), "got: {err}");
        assert!(err.contains("Stop broadcast failed"), "got: {err}");
    }

    #[test]
    fn rejected_result_is_an_error() {
        let ids: HashSet<u32> = [1].into_iter().collect();
        let err = broadcast_stop(&ids, 1, |_| {
            Ok(StopResponse {
                result: -5,
                discard_clock: 0,
            })
        })
        .unwrap_err();
        assert!(
            err.contains("Stop rejected by mcu 1: result=-5"),
            "got: {err}"
        );
    }

    #[test]
    fn axis_mcu_without_a_discard_clock_is_an_error() {
        let ids: HashSet<u32> = [2].into_iter().collect();
        let err = broadcast_stop(&ids, 9, |_| {
            Ok(StopResponse {
                result: 0,
                discard_clock: 5,
            })
        })
        .unwrap_err();
        assert!(err.contains("did not report a discard clock"), "got: {err}");
    }
}

mod corexy_reconstruction_tests {
    use super::{FREQ, make_linear_piece, record_synced, router_with_clock, shared};
    use crate::homing::{final_cartesian_position, reconstruct_cartesian_position};
    use crate::mcu_config::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};
    use crate::motion_history::HistoryStore;
    use crate::types::AxisKey;
    use runtime::segment::KinematicTag;

    fn corexy_cfg(mcu_id: u32) -> McuAxisConfig {
        McuAxisConfig {
            mcu_id,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KinematicTag::CoreXy as u8,
            caps: McuCaps {
                total_piece_memory: 4096,
            },
            max_motor_velocity: Vec::new(),
            ethercat: false,
            ..Default::default()
        }
    }

    fn z_cfg(mcu_id: u32) -> McuAxisConfig {
        McuAxisConfig {
            mcu_id,
            axes: vec![AXIS_Z],
            kinematics: KinematicTag::CoreXy as u8,
            caps: McuCaps {
                total_piece_memory: 4096,
            },
            max_motor_velocity: Vec::new(),
            ethercat: false,
            ..Default::default()
        }
    }

    fn key(mcu_id: u32, axis: usize) -> AxisKey {
        AxisKey {
            mcu_id,
            axis: axis as u8,
        }
    }

    #[test]
    fn trip_reconstruction_inverts_both_motor_lanes() {
        const MCU_ID: u32 = 20;
        const FREQ_F64: f64 = 180_000_000.0;
        let router = router_with_clock(MCU_ID, FREQ_F64);

        let piece_start: u64 = 1_000_000;
        let duration_secs: f32 = 0.025;
        #[allow(clippy::cast_possible_truncation)]
        let duration_ticks = (duration_secs as f64 * FREQ_F64) as u64;

        // Pure-X homing move from (0, 40): lane A = x + y goes 40 -> 140,
        // lane B = x - y goes -40 -> 60.
        let mut store = HistoryStore::default();
        record_synced(
            &mut store,
            &router,
            key(MCU_ID, AXIS_X),
            &make_linear_piece(piece_start, duration_secs, 40.0, 140.0),
            FREQ,
        );
        record_synced(
            &mut store,
            &router,
            key(MCU_ID, AXIS_Y),
            &make_linear_piece(piece_start, duration_secs, -40.0, 60.0),
            FREQ,
        );

        const Z_MCU: u32 = 120;
        {
            use crate::lock_ext::LockExt;
            let mut r = router.lock_ok();
            for i in (MCU_ID + 1)..Z_MCU {
                let _ = r.claim_mcu(&format!("dummy-{i}"));
            }
            let h = r.claim_mcu("mcu-z");
            assert_eq!(h.raw(), Z_MCU);
            let _ =
                r.set_clock_est_from_sample(h, FREQ_F64, std::time::Instant::now(), 1_000_000_000);
        }
        let z_host = super::host_of(&router, MCU_ID, piece_start);
        store.record(
            key(Z_MCU, AXIS_Z),
            &make_linear_piece(piece_start, duration_secs, 5.0, 5.0),
            FREQ,
            z_host,
        );

        let trip_clock = piece_start + duration_ticks / 2;
        let cart = reconstruct_cartesian_position(
            MCU_ID,
            trip_clock,
            &[corexy_cfg(MCU_ID), z_cfg(Z_MCU)],
            &router,
            &shared(store),
            f64::NEG_INFINITY,
            geometry::MachinePos([0.0; 3]),
        )
        .expect("corexy reconstruction must succeed");

        assert!(
            (cart.0[0] - 50.0).abs() < 0.5,
            "x = (A+B)/2 midway must be ~50, got {:.4}",
            cart.0[0]
        );
        assert!(
            (cart.0[1] - 40.0).abs() < 0.5,
            "y = (A-B)/2 must stay ~40 through a pure-X move, got {:.4}",
            cart.0[1]
        );
        assert!(
            (cart.0[2] - 5.0).abs() < 1e-6,
            "z lane on the second mcu must be read, got {:.4}",
            cart.0[2]
        );
    }

    #[test]
    fn trip_reconstruction_fails_when_xy_lanes_live_on_another_unlisted_mcu() {
        // The Trident PRINT_START regression: Z homes on the bottom MCU whose
        // config has no A/B lanes. Assembling a cartesian position from that
        // one config silently returned x=y=0, so mesh-active probing unwarped
        // at the wrong XY and ratcheted the frame every touch. A config list
        // that cannot cover every spatial lane must be a loud error, never
        // zeros.
        const Z_MCU: u32 = 23;
        let router = router_with_clock(Z_MCU, 180_000_000.0);
        let mut store = HistoryStore::default();
        record_synced(
            &mut store,
            &router,
            key(Z_MCU, AXIS_Z),
            &make_linear_piece(1_000_000, 0.025, 10.0, 0.0),
            FREQ,
        );

        let err = reconstruct_cartesian_position(
            Z_MCU,
            2_000_000,
            &[z_cfg(Z_MCU)],
            &router,
            &shared(store),
            f64::NEG_INFINITY,
            geometry::MachinePos([0.0; 3]),
        )
        .unwrap_err();
        assert!(
            err.contains("not configured on any mcu"),
            "xy lanes absent from the config list must fail loudly, got: {err}"
        );
    }

    #[test]
    fn lane_with_no_motion_since_attach_clamps_to_the_run_start() {
        const MCU_ID: u32 = 21;
        let router = router_with_clock(MCU_ID, 180_000_000.0);

        let mut store = HistoryStore::default();
        record_synced(
            &mut store,
            &router,
            key(MCU_ID, AXIS_X),
            &make_linear_piece(1_000_000, 0.025, 0.0, 100.0),
            FREQ,
        );

        let cart = reconstruct_cartesian_position(
            MCU_ID,
            2_000_000,
            &[corexy_cfg(MCU_ID), z_cfg(121)],
            &router,
            &shared(store),
            f64::NEG_INFINITY,
            geometry::MachinePos([10.0, 40.0, 5.0]),
        )
        .expect("the B lane never moved since attach: it clamps to its start");
        // A: linear 0->100mm over 4.5M ticks, read 1M ticks in = 22.222mm.
        // B: no motion since attach, held at forward([10,40,5])[1] = -30.
        // x=(A+B)/2, y=(A-B)/2, z passes through.
        let a = 100.0 * (1.0e6 / 4.5e6);
        let b = 10.0 - 40.0;
        assert!(
            (cart.0[0] - (a + b) / 2.0).abs() < 0.5,
            "x=(A+B)/2 expected {:.3}, got {:.4}",
            (a + b) / 2.0,
            cart.0[0]
        );
        assert!(
            (cart.0[1] - (a - b) / 2.0).abs() < 0.5,
            "y=(A-B)/2 expected {:.3}, got {:.4}",
            (a - b) / 2.0,
            cart.0[1]
        );
        assert!(
            (cart.0[2] - 5.0).abs() < 1e-6,
            "z passes through, got {:.4}",
            cart.0[2]
        );
    }

    #[test]
    fn final_position_inverts_both_motor_lanes() {
        const MCU_ID: u32 = 22;
        let mut store = HistoryStore::default();
        store.record(
            key(MCU_ID, AXIS_X),
            &make_linear_piece(1_000_000, 0.025, 0.0, 300.0),
            FREQ,
            0.0,
        );
        store.record(
            key(MCU_ID, AXIS_Y),
            &make_linear_piece(1_000_000, 0.025, 0.0, 100.0),
            FREQ,
            0.0,
        );

        const Z_MCU: u32 = 122;
        store.record(
            key(Z_MCU, AXIS_Z),
            &make_linear_piece(1_000_000, 0.025, 2.0, 2.0),
            FREQ,
            0.0,
        );

        let cart = final_cartesian_position(&[corexy_cfg(MCU_ID), z_cfg(Z_MCU)], &shared(store))
            .expect("final corexy position must succeed");
        assert!(
            (cart.0[0] - 200.0).abs() < 1e-3 && (cart.0[1] - 100.0).abs() < 1e-3,
            "A=300 B=100 must invert to x=200 y=100, got {cart:?}"
        );
    }
}

mod stepcompress_reconcile_tests {
    use crate::homing::{
        StepcompressLane, reconcile_stepcompress_axis, reconcile_stepcompress_lanes,
    };
    use crate::mcu_config::{AXIS_X, AXIS_Y, McuAxisConfig, McuCaps, SteppingMode};
    use crate::types::AxisKey;
    use runtime::segment::KinematicTag;
    use std::cell::RefCell;

    const MCU_ID: u32 = 3;
    const MICROSTEP: f64 = 0.0125;

    fn cfg(stepping_mode: SteppingMode) -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: MCU_ID,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KinematicTag::CoreXy as u8,
            caps: McuCaps {
                total_piece_memory: 4096,
            },
            max_motor_velocity: Vec::new(),
            ethercat: false,
            stepping_mode,
            microstep_distance: vec![MICROSTEP, MICROSTEP],
            invert_dir: vec![false, true],
            stepper_oids: vec![11, 12],
            stepcompress_sample_rate: match stepping_mode {
                SteppingMode::Stepcompress => 20_000.0,
                SteppingMode::Piece => 0.0,
            },
            move_queue_slots: match stepping_mode {
                SteppingMode::Stepcompress => 128,
                SteppingMode::Piece => 0,
            },
            step_pulse_seconds: vec![2e-6, 2e-6],
        }
    }

    fn key(axis: usize) -> AxisKey {
        AxisKey {
            mcu_id: MCU_ID,
            axis: axis as u8,
        }
    }

    #[test]
    fn agreeing_readback_returns_history_position_and_reseeds_shim() {
        let history_position = 40.0;
        let reseeds: RefCell<Vec<(usize, i64)>> = RefCell::new(Vec::new());
        let pos = reconcile_stepcompress_axis(
            &cfg(SteppingMode::Stepcompress),
            key(AXIS_X),
            history_position,
            &|lane| {
                assert_eq!(lane.oid, 11);
                assert_eq!(lane.motor, 0);
                Ok(3200)
            },
            &|lane, count| {
                reseeds.borrow_mut().push((lane.motor, count));
                Ok(())
            },
        )
        .expect("agreeing readback must reconcile");
        assert_eq!(pos, history_position);
        assert_eq!(reseeds.into_inner(), vec![(0, 3200)]);
    }

    #[test]
    fn readback_within_one_microstep_is_accepted() {
        let history_position = 3200.0 * MICROSTEP + MICROSTEP * 0.9;
        let pos = reconcile_stepcompress_axis(
            &cfg(SteppingMode::Stepcompress),
            key(AXIS_X),
            history_position,
            &|_| Ok(3200),
            &|_, _| Ok(()),
        )
        .expect("sub-microstep divergence must be accepted");
        assert_eq!(pos, history_position);
    }

    #[test]
    fn inverted_lane_negates_the_readback() {
        let lane_steps = 800_i64;
        let history_position = -(lane_steps as f64) * MICROSTEP;
        let pos = reconcile_stepcompress_axis(
            &cfg(SteppingMode::Stepcompress),
            key(AXIS_Y),
            history_position,
            &|lane| {
                assert!(lane.invert_dir);
                assert_eq!(lane.oid, 12);
                Ok(lane_steps)
            },
            &|_, _| Ok(()),
        )
        .expect("inverted lane must reconcile against the negated count");
        assert_eq!(pos, history_position);
    }

    #[test]
    fn divergence_beyond_one_microstep_is_a_loud_error() {
        let reseeded = RefCell::new(false);
        let err = reconcile_stepcompress_axis(
            &cfg(SteppingMode::Stepcompress),
            key(AXIS_X),
            40.0,
            &|_| Ok(3200 + 3),
            &|_, _| {
                *reseeded.borrow_mut() = true;
                Ok(())
            },
        )
        .unwrap_err();
        assert!(err.contains("mcu=3"), "got: {err}");
        assert!(err.contains("axis=0"), "got: {err}");
        assert!(err.contains("expected=40.000000mm"), "got: {err}");
        assert!(err.contains("actual=40.037500mm"), "got: {err}");
        assert!(err.contains("exceeds one microstep"), "got: {err}");
        assert!(
            !reseeded.into_inner(),
            "a diverged lane must not re-seed the shim"
        );
    }

    #[test]
    fn piece_mode_axis_never_queries_the_mcu() {
        let pos = reconcile_stepcompress_axis(
            &cfg(SteppingMode::Piece),
            key(AXIS_X),
            17.5,
            &|_| panic!("piece-mode homing must not call stepper_get_position"),
            &|_, _| panic!("piece-mode homing must not re-seed a step shim"),
        )
        .expect("piece mode returns the history position untouched");
        assert_eq!(pos, 17.5);
    }

    #[test]
    fn lane_sweep_skips_piece_mode_mcus_and_covers_every_stepcompress_lane() {
        let mut piece_cfg = cfg(SteppingMode::Piece);
        piece_cfg.mcu_id = 9;
        let configs = vec![piece_cfg, cfg(SteppingMode::Stepcompress)];
        let queried: RefCell<Vec<(u32, u32)>> = RefCell::new(Vec::new());
        reconcile_stepcompress_lanes(
            &configs,
            |k| {
                assert_eq!(k.mcu_id, MCU_ID);
                Ok(0.0)
            },
            &|lane| {
                queried.borrow_mut().push((lane.mcu_id, lane.oid));
                Ok(0)
            },
            &|_, _| Ok(()),
        )
        .expect("mixed-mode sweep must succeed");
        assert_eq!(queried.into_inner(), vec![(MCU_ID, 11), (MCU_ID, 12)]);
    }

    #[test]
    fn missing_oid_for_a_stepcompress_lane_is_a_loud_error() {
        let mut broken = cfg(SteppingMode::Stepcompress);
        broken.stepper_oids = vec![11];
        let err =
            reconcile_stepcompress_axis(&broken, key(AXIS_Y), 0.0, &|_| Ok(0), &|_, _| Ok(()))
                .unwrap_err();
        assert!(err.contains("has no stepper oid"), "got: {err}");
    }

    #[test]
    fn steps_to_mm_honours_direction_polarity() {
        let forward = StepcompressLane {
            mcu_id: MCU_ID,
            axis: 0,
            motor: 0,
            oid: 11,
            microstep_distance: MICROSTEP,
            invert_dir: false,
        };
        let inverted = StepcompressLane {
            invert_dir: true,
            ..forward
        };
        assert_eq!(forward.steps_to_mm(-80), -1.0);
        assert_eq!(inverted.steps_to_mm(-80), 1.0);
    }

    #[test]
    fn mm_to_steps_is_trajectory_signed_regardless_of_polarity() {
        let forward = StepcompressLane {
            mcu_id: MCU_ID,
            axis: 0,
            motor: 0,
            oid: 11,
            microstep_distance: MICROSTEP,
            invert_dir: false,
        };
        let inverted = StepcompressLane {
            invert_dir: true,
            ..forward
        };
        assert_eq!(forward.mm_to_steps(-1.0), -80);
        assert_eq!(inverted.mm_to_steps(-1.0), -80);
        for lane in [forward, inverted] {
            assert_eq!(
                lane.steps_to_mm(lane.trajectory_steps(lane.mm_to_steps(2.5))),
                2.5
            );
        }
    }
}
