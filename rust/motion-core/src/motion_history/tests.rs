use std::sync::Arc;

use host_rt::passthrough_queue::PassthroughRouter;
use nurbs::chebyshev::monomial_tau_to_chebyshev;
use runtime::piece_ring::{MAX_PIECE_COEFFS, PieceEntry};

use crate::motion_history::{HISTORY_CAPACITY, HistoryError, HistoryPiece, HistoryStore};
use crate::types::AxisKey;

fn stub_router_two_mcus() -> PassthroughRouter {
    let clock: Arc<dyn host_rt::clock::Clock + Send + Sync> = Arc::new(host_rt::clock::RealClock);
    let mut router = PassthroughRouter::with_clock(clock);
    let h0 = router.claim_mcu("mcu-0");
    assert_eq!(h0.raw(), 0);
    let h1 = router.claim_mcu("mcu-1");
    let h2 = router.claim_mcu("mcu-2");
    assert_eq!(h1.raw(), 1);
    assert_eq!(h2.raw(), 2);
    router
        .set_clock_est(h1, 1_000_000.0, 0.0, 0)
        .expect("set_clock_est mcu-1");
    router
        .set_clock_est(h2, 3_000_000.0, 0.0, 0)
        .expect("set_clock_est mcu-2");
    router
}

#[test]
fn clock_between_mcus_round_trips_through_host_secs() {
    let router = stub_router_two_mcus();
    let got = crate::motion_history::clock_between_mcus(
        &router,
        crate::types::mcu_handle_from_raw(1),
        crate::types::mcu_handle_from_raw(2),
        1_000_000,
    )
    .unwrap();
    assert_eq!(got, 3_000_000);
}

const FREQ: u32 = 520_000_000;

fn key() -> AxisKey {
    AxisKey { mcu_id: 7, axis: 2 }
}

fn entry(start_time: u64, duration: f32, bernstein_cubic: [f32; 4]) -> PieceEntry {
    let [b0, b1, b2, b3] = bernstein_cubic.map(f64::from);
    let d = f64::from(duration);
    let mono = if d > 0.0 {
        [
            b0,
            3.0 * (b1 - b0) / d,
            3.0 * (b2 - 2.0 * b1 + b0) / (d * d),
            (b3 - 3.0 * b2 + 3.0 * b1 - b0) / (d * d * d),
        ]
    } else {
        [b0, 0.0, 0.0, 0.0]
    };
    let cheb = monomial_tau_to_chebyshev(&mono, d);
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    for (dst, &src) in coeffs.iter_mut().zip(cheb.iter()) {
        *dst = src as f32;
    }
    PieceEntry {
        start_time,
        duration,
        motor_mask: 0,
        coeff_count: cheb.len() as u8,
        coeffs,
        ..PieceEntry::zeroed()
    }
}

fn linear(start_time: u64, duration: f32, p0: f32, p1: f32) -> PieceEntry {
    let mut coeffs = [0.0_f32; MAX_PIECE_COEFFS];
    coeffs[0] = (p0 + p1) / 2.0;
    coeffs[1] = (p1 - p0) / 2.0;
    PieceEntry {
        start_time,
        duration,
        motor_mask: 0,
        coeff_count: 2,
        coeffs,
        ..PieceEntry::zeroed()
    }
}

fn h(clock: u64) -> f64 {
    clock as f64 / f64::from(FREQ)
}

fn rec(store: &mut HistoryStore, key: AxisKey, e: PieceEntry) {
    let host = h(e.start_time);
    store.record(key, &e, FREQ, host);
}

#[test]
fn end_clock_matches_isr_formula() {
    let e = entry(1_000, 0.0123, [0.0; 4]);
    let hp = HistoryPiece::from_entry(&e, FREQ, h(1_000));
    assert_eq!(hp.end_clock, e.end_time(FREQ as f32));
    assert_eq!(hp.start_clock, 1_000);
}

#[test]
fn linear_piece_position_velocity_acceleration() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let mid = h(FREQ as u64 / 2);
    let st = store
        .state_at_host(key(), mid, Some(f64::INFINITY))
        .unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
    assert!((st.velocity - 10.0).abs() < 1e-6);
    assert!(st.acceleration.abs() < 1e-6);
}

#[test]
fn quadratic_piece_derivatives() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), entry(0, 1.0, [0.0, 0.0, 5.0, 15.0]));
    let mid = h(FREQ as u64 / 2);
    let st = store
        .state_at_host(key(), mid, Some(f64::INFINITY))
        .unwrap();
    assert!((st.position - 3.75).abs() < 1e-5);
    assert!((st.velocity - 15.0).abs() < 1e-5);
    assert!((st.acceleration - 30.0).abs() < 1e-4);
}

#[test]
fn gap_between_pieces_holds_previous_endpoint() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 0.001, 0.0, 10.0));
    let gap_start = HistoryPiece::from_entry(&linear(0, 0.001, 0.0, 10.0), FREQ, 0.0).end_clock;
    rec(
        &mut store,
        key(),
        linear(gap_start + 1_000_000, 0.001, 10.0, 20.0),
    );
    let st = store
        .state_at_host(key(), h(gap_start + 500_000), Some(f64::INFINITY))
        .unwrap();
    assert!((st.position - 10.0).abs() < 1e-6);
    assert_eq!(st.velocity, 0.0);
    assert_eq!(st.acceleration, 0.0);
}

#[test]
fn after_last_piece_holds_when_not_future() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 0.001, 0.0, 10.0));
    let end = store
        .state_at_host(key(), h(519_999), Some(f64::INFINITY))
        .unwrap();
    assert!((end.position - 10.0).abs() < 1e-4);
    let held = store
        .state_at_host(key(), h(5_000_000), Some(h(10_000_000)))
        .unwrap();
    assert!((held.position - 10.0).abs() < 1e-6);
}

#[test]
fn hold_in_the_future_is_an_error() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 0.001, 0.0, 10.0));
    let err = store
        .state_at_host(key(), h(5_000_000), Some(h(1_000_000)))
        .unwrap_err();
    assert!(matches!(err, HistoryError::QueryInFuture { .. }));
}

#[test]
fn inside_committed_future_piece_evaluates() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let st = store
        .state_at_host(key(), h(FREQ as u64 / 2), Some(h(1_000)))
        .unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
}

#[test]
fn before_window_is_an_error() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(1_000_000, 0.001, 0.0, 10.0));
    let err = store
        .state_at_host(key(), h(500), Some(f64::INFINITY))
        .unwrap_err();
    assert!(matches!(err, HistoryError::BeforeRetainedWindow { .. }));
}

#[test]
fn unknown_axis_is_an_error() {
    let store = HistoryStore::default();
    let err = store
        .state_at_host(key(), 0.0, Some(f64::INFINITY))
        .unwrap_err();
    assert!(matches!(err, HistoryError::NoHistoryForAxis(_)));
}

#[test]
fn non_finite_query_is_an_error() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let err = store
        .state_at_host(key(), f64::NAN, Some(f64::INFINITY))
        .unwrap_err();
    assert!(matches!(err, HistoryError::NonFiniteQuery { .. }));
}

#[test]
fn rebase_clears_ring_and_answers_from_register() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    store.rebase_axis(key(), h(2_000_000_000), 42.0);
    let held = store
        .state_at_host(key(), h(2_000_000_500), Some(h(3_000_000_000)))
        .unwrap();
    assert!((held.position - 42.0).abs() < 1e-9);
    let held_before = store
        .state_at_host(key(), h(1_000), Some(f64::INFINITY))
        .unwrap();
    assert!((held_before.position - 42.0).abs() < 1e-9);
}

#[test]
fn eviction_keeps_capacity_and_reports_true_window() {
    let mut store = HistoryStore::default();
    let dur = 0.001_f32;
    let dur_ticks = (dur * FREQ as f32) as u64;
    for i in 0..(HISTORY_CAPACITY as u64 + 10) {
        rec(&mut store, key(), linear(i * dur_ticks, dur, 0.0, 1.0));
    }
    let err = store
        .state_at_host(key(), 0.0, Some(f64::INFINITY))
        .unwrap_err();
    match err {
        HistoryError::BeforeRetainedWindow { window_start, .. } => {
            assert!((window_start - h(10 * dur_ticks)).abs() < 1e-12);
        }
        other => panic!("expected BeforeRetainedWindow, got {other:?}"),
    }
}

#[test]
fn drop_pieces_on_reanchor_keeps_unrecorded_axis_answerable() {
    let mut store = HistoryStore::default();
    let moving = AxisKey { mcu_id: 7, axis: 2 };
    let stationary = AxisKey { mcu_id: 7, axis: 0 };
    rec(&mut store, moving, linear(0, 1.0, 0.0, 10.0));
    rec(&mut store, stationary, linear(0, 1.0, 3.0, 3.0));

    store.drop_pieces_on_reanchor();

    let held = store
        .state_at_host(stationary, h(5_000_000), Some(h(10_000_000)))
        .unwrap();
    assert!((held.position - 3.0).abs() < 1e-6);

    rec(&mut store, moving, linear(2_000_000_000, 1.0, 10.0, 20.0));
    assert_eq!(store.final_position(moving), Some(20.0));
}

#[test]
fn rest_between_endpoint_and_next_ring_answers_held_position() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    store.drop_pieces_on_reanchor();
    rec(&mut store, key(), linear(2_000_000_000, 1.0, 10.0, 20.0));

    let held = store
        .state_at_host(key(), h(1_500_000_000), Some(f64::INFINITY))
        .unwrap();
    assert!((held.position - 10.0).abs() < 1e-6);
    assert_eq!(held.velocity, 0.0);

    let err = store
        .state_at_host(key(), h(FREQ as u64 / 2), Some(f64::INFINITY))
        .unwrap_err();
    assert!(
        matches!(err, HistoryError::BeforeRetainedWindow { .. }),
        "times inside the dropped motion are not a held rest: {err:?}"
    );
}

#[test]
fn eviction_does_not_stretch_the_pre_ring_hold() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    store.drop_pieces_on_reanchor();
    let dur = 0.001_f32;
    let dur_ticks = (dur * FREQ as f32) as u64;
    let ring_start = 2_000_000_000_u64;
    for i in 0..(HISTORY_CAPACITY as u64 + 10) {
        rec(
            &mut store,
            key(),
            linear(ring_start + i * dur_ticks, dur, 0.0, 1.0),
        );
    }

    let held = store
        .state_at_host(key(), h(1_500_000_000), Some(f64::INFINITY))
        .unwrap();
    assert!((held.position - 10.0).abs() < 1e-6);

    let evicted_t = h(ring_start + 5 * dur_ticks);
    let err = store
        .state_at_host(key(), evicted_t, Some(f64::INFINITY))
        .unwrap_err();
    assert!(
        matches!(err, HistoryError::BeforeRetainedWindow { .. }),
        "evicted motion must not answer as a held rest: {err:?}"
    );
}

#[test]
fn empty_ring_query_inside_dropped_motion_fails_loud() {
    let mut store = HistoryStore::default();
    // Moving piece over [0,1]s, 0->10. At host_t=0.5 the axis is mid-motion at
    // 5.0 and has NOT reached the endpoint 10.0 — that time is dropped motion.
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    store.drop_pieces_on_reanchor();

    let err = store.state_at_host(key(), 0.5, Some(2.0)).unwrap_err();
    assert!(
        matches!(err, HistoryError::BeforeRetainedWindow { .. }),
        "empty-ring query inside dropped motion must fail loud, not return a \
         position the axis never reached: {err:?}"
    );

    // A time at/after the endpoint host is a provable held rest and still answers.
    let held = store.state_at_host(key(), 1.5, Some(2.0)).unwrap();
    assert!(
        (held.position - 10.0).abs() < 1e-9,
        "held rest position {}",
        held.position
    );
    assert_eq!(held.velocity, 0.0);
    assert_eq!(held.acceleration, 0.0);
}

#[test]
fn trailing_rest_run_extends_hold_coverage_before_endpoint() {
    let mut store = HistoryStore::default();
    // Move 0->10 over [0,1]s, then hold at 10 over [1,2]s (a rest piece).
    // The endpoint host is 2.0, but the axis was provably at 10.0 from 1.0 on.
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let rest_start = FREQ as u64; // 1.0s in clock ticks
    rec(&mut store, key(), linear(rest_start, 1.0, 10.0, 10.0));

    store.drop_pieces_on_reanchor();
    rec(&mut store, key(), linear(4_000_000_000, 1.0, 10.0, 20.0));
    assert_eq!(store.final_position(key()), Some(20.0));

    // A time inside the old trailing rest run (1.0..2.0), before the endpoint
    // host, answers the held rest even though the ring is now non-empty.
    let in_rest = store
        .state_at_host(key(), h(rest_start) + 0.5, Some(f64::INFINITY))
        .unwrap();
    assert!(
        (in_rest.position - 10.0).abs() < 1e-6,
        "rest-run position {}",
        in_rest.position
    );
    assert_eq!(in_rest.velocity, 0.0);

    // A time inside the earlier move stays unanswerable — it was real motion.
    let err = store
        .state_at_host(key(), 0.5, Some(f64::INFINITY))
        .unwrap_err();
    assert!(
        matches!(err, HistoryError::BeforeRetainedWindow { .. }),
        "moving span before the rest run must stay unanswerable: {err:?}"
    );
}

#[test]
fn rebase_to_earlier_clock_accepts_post_rewind_pieces() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(3_000_000, 1.0, 0.0, 5.0));
    let held = store.final_position(key()).unwrap();
    store.rebase_axis(key(), h(2_000_000), held);
    rec(&mut store, key(), linear(2_500_000, 1.0, 5.0, 6.0));
    assert_eq!(store.final_position(key()), Some(6.0));
}

#[test]
fn backward_host_supersedes_stale_tail() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.5, 0.0, 10.0), FREQ, 1.0);
    store.record(key(), &linear(0, 0.5, 50.0, 60.0), FREQ, 0.2);
    let st = store
        .state_at_host(key(), 0.4, Some(f64::INFINITY))
        .unwrap();
    assert!((st.position - 54.0).abs() < 1e-6);
    let held = store
        .state_at_host(key(), 1.2, Some(f64::INFINITY))
        .unwrap();
    assert!((held.position - 60.0).abs() < 1e-6);
}

fn axis_key(axis: u8) -> AxisKey {
    AxisKey { mcu_id: 7, axis }
}

#[test]
fn assemble_cartesian_state_corexy_inverts_motor_frame() {
    use crate::kinematics::KinematicsModule;
    use crate::motion_history::{AxisState, assemble_cartesian_state};
    use runtime::segment::KinematicTag;

    let expected_x = 10.0;
    let expected_y = 4.0;
    let motor_a = expected_x + expected_y;
    let motor_b = expected_x - expected_y;
    let motor_state = [
        Some(AxisState {
            position: motor_a,
            velocity: 1.0,
            acceleration: 0.0,
        }),
        Some(AxisState {
            position: motor_b,
            velocity: -1.0,
            acceleration: 0.0,
        }),
        Some(AxisState {
            position: 2.0,
            velocity: 0.0,
            acceleration: 0.0,
        }),
        None,
    ];
    let kin = KinematicsModule::from_tag(KinematicTag::CoreXy as u8).unwrap();
    let out = assemble_cartesian_state(motor_state, &kin);
    assert!(
        (out["x"].0 - expected_x).abs() < 1e-9,
        "x={:?}",
        out.get("x")
    );
    assert!(
        (out["y"].0 - expected_y).abs() < 1e-9,
        "y={:?}",
        out.get("y")
    );
    assert_eq!(out["z"], (2.0, 0.0, 0.0));
    assert!(!out.contains_key("e"));
}

#[test]
fn assemble_cartesian_state_cartesian_is_passthrough() {
    use crate::kinematics::KinematicsModule;
    use crate::motion_history::{AxisState, assemble_cartesian_state};
    use runtime::segment::KinematicTag;

    let motor_state = [
        Some(AxisState {
            position: 10.0,
            velocity: 1.0,
            acceleration: 0.1,
        }),
        Some(AxisState {
            position: 20.0,
            velocity: -1.0,
            acceleration: 0.2,
        }),
        Some(AxisState {
            position: 5.0,
            velocity: 0.0,
            acceleration: 0.0,
        }),
        Some(AxisState {
            position: 2.0,
            velocity: 3.0,
            acceleration: 0.0,
        }),
    ];
    let kin = KinematicsModule::from_tag(KinematicTag::Cartesian as u8).unwrap();
    let out = assemble_cartesian_state(motor_state, &kin);
    assert_eq!(out["x"], (10.0, 1.0, 0.1));
    assert_eq!(out["y"], (20.0, -1.0, 0.2));
    assert_eq!(out["z"], (5.0, 0.0, 0.0));
    assert_eq!(out["e"], (2.0, 3.0, 0.0));
}

#[test]
fn assemble_cartesian_state_corexy_omits_xy_when_one_motor_missing() {
    use crate::kinematics::KinematicsModule;
    use crate::motion_history::{AxisState, assemble_cartesian_state};
    use runtime::segment::KinematicTag;

    // Only motor0 resolved; motor1's history is unanswerable. Reporting x/y
    // from a single motor would silently invent a position the axis never
    // proved it held — omit both rather than guess. Z stays independent.
    let motor_state = [
        Some(AxisState {
            position: 15.0,
            velocity: 0.0,
            acceleration: 0.0,
        }),
        None,
        Some(AxisState {
            position: 3.0,
            velocity: 0.0,
            acceleration: 0.0,
        }),
        None,
    ];
    let kin = KinematicsModule::from_tag(KinematicTag::CoreXy as u8).unwrap();
    let out = assemble_cartesian_state(motor_state, &kin);
    assert!(!out.contains_key("x"));
    assert!(!out.contains_key("y"));
    assert_eq!(out["z"], (3.0, 0.0, 0.0));
}

#[test]
fn corexy_history_round_trip_reproduces_bench_symptom() {
    // Reproduces the trident-bench beacon scan: motor0/motor1 pieces
    // recorded through the ring (as commit_sent_bundle does) must invert to
    // the commanded cartesian XY, not the raw CoreXY A/B sum/difference that
    // motion_state_at_clock used to leak straight through.
    use crate::kinematics::KinematicsModule;
    use crate::motion_history::assemble_cartesian_state;
    use runtime::segment::KinematicTag;

    let cart_x0 = 25.0;
    let cart_y0 = 25.0;
    let cart_x1 = 275.0;
    let cart_y1 = 25.0;
    let mut store = HistoryStore::default();
    rec(
        &mut store,
        axis_key(0),
        linear(
            0,
            1.0,
            (cart_x0 + cart_y0) as f32,
            (cart_x1 + cart_y1) as f32,
        ),
    );
    rec(
        &mut store,
        axis_key(1),
        linear(
            0,
            1.0,
            (cart_x0 - cart_y0) as f32,
            (cart_x1 - cart_y1) as f32,
        ),
    );

    let mid = h(FREQ as u64 / 2);
    let mut motor_state = [None; 4];
    for axis in 0..2u8 {
        motor_state[axis as usize] = store
            .state_at_host(axis_key(axis), mid, Some(f64::INFINITY))
            .ok();
    }
    let kin = KinematicsModule::from_tag(KinematicTag::CoreXy as u8).unwrap();
    let out = assemble_cartesian_state(motor_state, &kin);

    let expected_x = (cart_x0 + cart_x1) / 2.0;
    let expected_y = (cart_y0 + cart_y1) / 2.0;
    assert!(
        (out["x"].0 - expected_x).abs() < 1e-6,
        "x={:?} want {expected_x}",
        out.get("x")
    );
    assert!(
        (out["y"].0 - expected_y).abs() < 1e-6,
        "y={:?} want {expected_y}",
        out.get("y")
    );
}

#[test]
fn rebase_after_probe_trip_round_trips_through_cartesian_inversion() {
    // Reproduces the z_tilt/beacon-probe incident: every proximity probe
    // ends in toolhead.set_position(x, y, z), which rebases the retained
    // history to the trip's stop position. Before spatial_rebase_targets
    // existed, that rebase stored raw cartesian x/y under axis 0/1 — the
    // same slots live CoreXY pieces store in motor frame — so querying
    // through assemble_cartesian_state's kinematics inversion afterward
    // double-transformed an already-correct position into garbage (the
    // bench's "probe at 197.500,-47.500" from a real (150,245) point).
    use crate::kinematics::KinematicsModule;
    use crate::mcu_config::{McuAxisConfig, McuCaps, spatial_rebase_targets};
    use crate::motion_history::assemble_cartesian_state;
    use runtime::segment::KinematicTag;

    let configs = vec![McuAxisConfig {
        max_motor_velocity: Vec::new(),
        mcu_id: 1,
        axes: vec![0, 1, 2],
        kinematics: KinematicTag::CoreXy as u8,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }];
    let cart_x = 150.0;
    let cart_y = 245.0;
    let cart_z = 1.965;

    let mut store = HistoryStore::default();
    for (key, value) in
        spatial_rebase_targets(&configs, geometry::MachinePos([cart_x, cart_y, cart_z]))
    {
        store.rebase_axis(key, 0.0, value);
    }

    let mut motor_state = [None; 4];
    for axis in 0..3u8 {
        motor_state[axis as usize] = store
            .state_at_host(AxisKey { mcu_id: 1, axis }, 0.0, Some(f64::INFINITY))
            .ok();
    }
    let kin = KinematicsModule::from_tag(KinematicTag::CoreXy as u8).unwrap();
    let out = assemble_cartesian_state(motor_state, &kin);

    assert!(
        (out["x"].0 - cart_x).abs() < 1e-9,
        "x={:?} want {cart_x} — got the mangled (x+y)/2, (x-y)/2 pair if this fails",
        out.get("x")
    );
    assert!(
        (out["y"].0 - cart_y).abs() < 1e-9,
        "y={:?} want {cart_y}",
        out.get("y")
    );
    assert!((out["z"].0 - cart_z).abs() < 1e-9);
}

#[test]
fn host_clock_round_trip_is_identity() {
    let router = stub_router_two_mcus();
    let h = crate::types::mcu_handle_from_raw(1);
    let clock = 12_345_678_u64;
    let host = router.clock_to_host_secs(h, clock).expect("synced mcu");
    let back = router
        .host_time_to_mcu_clock(h, host)
        .expect("synced mcu inverse");
    assert!(
        (back as i64 - clock as i64).abs() <= 1,
        "T then T^-1 must return the original clock (got {back}, want {clock})"
    );
}

#[test]
fn state_at_clock_matches_state_at_host_when_mapping_is_exact() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let clock = FREQ as u64 / 4;
    let by_clock = store
        .state_at_clock(key(), clock, h(clock), Some(f64::INFINITY))
        .unwrap();
    let by_host = store
        .state_at_host(key(), h(clock), Some(f64::INFINITY))
        .unwrap();
    assert!((by_clock.position - by_host.position).abs() < 1e-9);
    assert!((by_clock.position - 2.5).abs() < 1e-6);
    assert!((by_clock.velocity - 10.0).abs() < 1e-6);
}

#[test]
fn state_at_clock_is_immune_to_host_mapping_skew() {
    let mut store = HistoryStore::default();
    // The piece was keyed with a host time 40 ms later than its clock
    // implies — the sync estimate drifted between send and query. 10 mm/s
    // over 40 ms is a 0.4 mm bias in the host-domain answer.
    let skew_s = 0.040;
    let e = linear(0, 1.0, 0.0, 10.0);
    store.record(key(), &e, FREQ, skew_s);
    let clock = FREQ as u64 / 2;
    let by_clock = store
        .state_at_clock(key(), clock, h(clock), Some(f64::INFINITY))
        .unwrap();
    let by_host = store
        .state_at_host(key(), h(clock), Some(f64::INFINITY))
        .unwrap();
    assert!(
        (by_clock.position - 5.0).abs() < 1e-6,
        "clock-domain answer must track the executed trajectory, got {}",
        by_clock.position
    );
    assert!(
        (by_host.position - 4.6).abs() < 1e-6,
        "host-domain answer is expected to carry the 0.4 mm skew bias, got {}",
        by_host.position
    );
}

#[test]
fn state_at_clock_holds_endpoint_in_gap_after_piece() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(0, 1.0, 0.0, 10.0));
    let after_end = 2 * FREQ as u64;
    let st = store
        .state_at_clock(key(), after_end, h(after_end), Some(f64::INFINITY))
        .unwrap();
    assert!((st.position - 10.0).abs() < 1e-6);
    assert!(st.velocity.abs() < 1e-9);
}

#[test]
fn state_at_clock_before_ring_falls_back_to_host_hold() {
    let mut store = HistoryStore::default();
    rec(&mut store, key(), linear(FREQ as u64, 1.0, 3.0, 3.0));
    store.drop_pieces_on_reanchor();
    rec(&mut store, key(), linear(4 * FREQ as u64, 1.0, 3.0, 7.0));
    let held = store
        .state_at_clock(
            key(),
            3 * FREQ as u64,
            h(3 * FREQ as u64),
            Some(f64::INFINITY),
        )
        .unwrap();
    assert!((held.position - 3.0).abs() < 1e-6);
}
