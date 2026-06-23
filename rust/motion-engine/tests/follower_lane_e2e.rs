#![allow(deprecated)]

use std::sync::{Arc, Mutex};

use _motion_engine::classify::classify_and_build;
use _motion_engine::config::{
    AxisDecl, AxisRegistry, LimitSection, PlannerConfig, PostProcessorDecl, PostProcessorSet,
};
use _motion_engine::dispatch::{McuAxisConfig, McuCaps};
use _motion_engine::enqueue::enqueue_segment;
use _motion_engine::motion_history::HistoryStore;
use _motion_engine::planner::{DispatchError, PlannerHandle};
use _motion_engine::pump::AxisKey;
use trajectory::ShapedSegment;

const FOLLOWER_AXIS: usize = 3;
const EXTRUSION_RATIO: f64 = 0.05;
const LEG_MM: f64 = 40.0;
const CLOCK_HZ: f64 = 1_000_000.0;

fn follower_registry(extra_pp: &[&str]) -> AxisRegistry {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"]
        .iter()
        .map(|name| AxisDecl {
            name: (*name).to_string(),
            follows: vec![],
            motors: vec![],
            post_processors: vec![],
        })
        .collect();
    if extra_pp.contains(&"is_xy") {
        decls[0].post_processors = vec!["is_xy".to_string()];
        decls[1].post_processors = vec!["is_xy".to_string()];
    }
    decls.push(AxisDecl {
        name: "e".to_string(),
        follows: vec!["x".into(), "y".into(), "z".into()],
        motors: vec![],
        post_processors: vec!["pa".to_string()],
    });
    AxisRegistry::try_new(decls).unwrap()
}

fn follower_config(with_kernel: bool, pa_k: f64) -> PlannerConfig {
    let pps = if with_kernel {
        vec!["pa", "is_xy"]
    } else {
        vec!["pa"]
    };
    let registry = follower_registry(&pps);
    let mut decls = vec![PostProcessorDecl {
        name: "pa".into(),
        ty: "linear_pressure_advance".into(),
        params: vec![("k".into(), pa_k)],
    }];
    if with_kernel {
        decls.push(PostProcessorDecl {
            name: "is_xy".into(),
            ty: "smooth_mzv".into(),
            params: vec![("frequency_hz".into(), 40.0)],
        });
    }
    let set = PostProcessorSet::try_new(&registry, &decls).unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections = vec![
        LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(200.0),
            max_accel: Some(2000.0),
            max_jerk: None,
        },
        LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(10.0),
            max_accel: Some(80.0),
            max_jerk: None,
        },
        LimitSection {
            name: "extrusion".into(),
            axes: vec![FOLLOWER_AXIS],
            max_velocity: Some(75.0),
            max_accel: Some(1500.0),
            max_jerk: None,
        },
    ];
    cfg.axis_registry = registry;
    cfg.post_processors = set;
    cfg.fit_tolerance_mm = 0.05;
    cfg
}

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<Mutex<Vec<ShapedSegment>>>,
) {
    let recorded = Arc::new(Mutex::new(Vec::new()));
    let rec = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

fn noop_nudge_dispatch()
-> Arc<dyn Fn(u32, &_motion_engine::nudge::NudgePiece) -> Result<(), DispatchError> + Send + Sync> {
    Arc::new(|_mcu_id: u32, _np: &_motion_engine::nudge::NudgePiece| Ok(()))
}

fn run_two_leg_print(
    with_kernel: bool,
    pa_k: f64,
    second_leg: (f64, f64),
) -> (Vec<ShapedSegment>, f64) {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(
        follower_config(with_kernel, pa_k),
        dispatch,
        noop_nudge_dispatch(),
    );
    let leg2_len = (second_leg.0 * second_leg.0 + second_leg.1 * second_leg.1).sqrt();
    h.submit_move(
        classify_and_build(
            [0.0; 3],
            LEG_MM,
            0.0,
            0.0,
            &[(FOLLOWER_AXIS, EXTRUSION_RATIO * LEG_MM)],
            120.0,
        )
        .unwrap(),
    )
    .unwrap();
    h.submit_move(
        classify_and_build(
            [LEG_MM, 0.0, 0.0],
            second_leg.0,
            second_leg.1,
            0.0,
            &[(FOLLOWER_AXIS, EXTRUSION_RATIO * leg2_len)],
            120.0,
        )
        .unwrap(),
    )
    .unwrap();
    h.flush().unwrap();
    h.shutdown();
    let segs = recorded.lock().unwrap().clone();
    assert!(!segs.is_empty());
    let nominal_total = EXTRUSION_RATIO * (LEG_MM + leg2_len);
    (segs, nominal_total)
}

fn follower_history(segs: &[ShapedSegment]) -> (HistoryStore, AxisKey, Vec<u64>) {
    let cfg = vec![McuAxisConfig {
        mcu_id: 7,
        axes: vec![0, 1, 2, FOLLOWER_AXIS],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }];
    let key = AxisKey {
        mcu_id: 7,
        axis: FOLLOWER_AXIS as u8,
    };
    let mut store = HistoryStore::default();
    let mut piece_clocks = Vec::new();
    let mut fresh = true;
    for seg in segs {
        let msgs = enqueue_segment(
            seg,
            &cfg,
            seg.t_start,
            fresh,
            0.0,
            1.0,
            |_mcu, hs| (hs * CLOCK_HZ) as u64,
            None,
        );
        fresh = false;
        for msg in msgs.iter().filter(|m| m.key == key) {
            for (entry, _) in &msg.pieces {
                store.record(key, entry, CLOCK_HZ as u32);
                piece_clocks.push(entry.start_time);
            }
        }
    }
    assert!(
        !piece_clocks.is_empty(),
        "lane-3 PushPieces must arrive for the follower axis"
    );
    (store, key, piece_clocks)
}

#[test]
fn follower_lane_passthrough_endpoint_matches_nominal_ledger() {
    let (segs, nominal_total) = run_two_leg_print(false, 0.02, (0.0, LEG_MM));
    for seg in &segs {
        assert_eq!(seg.axes.len(), 4, "registry-wide track vector");
    }
    let (store, key, _) = follower_history(&segs);
    let physical = store.final_position(key).unwrap();
    assert!(
        (physical - nominal_total).abs() < 0.05,
        "passthrough follower endpoint {physical} must match nominal {nominal_total}"
    );
}

#[test]
fn follower_lane_kernel_shortfall_is_accepted_not_corrected() {
    let (segs, nominal_total) = run_two_leg_print(true, 0.0, (LEG_MM, 8.0));
    let (store, key, _) = follower_history(&segs);
    let physical = store.final_position(key).unwrap();
    assert!(
        physical < nominal_total - 1e-4,
        "smoothing the corner taken at speed must shorten the realized path: physical {physical} \
         vs nominal {nominal_total}"
    );
    assert!(
        physical > 0.9 * nominal_total,
        "shortfall should be the corner rounding only, got {physical}"
    );
}

#[test]
fn follower_history_mid_move_is_between_endpoints_and_monotone() {
    let (segs, _) = run_two_leg_print(false, 0.0, (0.0, LEG_MM));
    let (store, key, clocks) = follower_history(&segs);
    let first = *clocks.first().unwrap();
    let last = *clocks.last().unwrap();
    let end = store.final_position(key).unwrap();
    let mut prev = f64::NEG_INFINITY;
    let n = 50;
    for i in 0..=n {
        let clock = first + (last - first) * i / n;
        let st = store.state_at_clock(key, clock, Some(u64::MAX)).unwrap();
        assert!(
            st.position >= prev - 1e-6,
            "positive-ratio follower track must be monotone (clock {clock}: \
             {} < {prev})",
            st.position
        );
        assert!(st.position >= -1e-6 && st.position <= end + 1e-6);
        prev = st.position;
    }
}

fn z_segment_10mm() -> geometry::segment::CubicSegment {
    use geometry::segment::SourceRange;
    use nurbs::VectorNurbs;
    let lerp = |t: f64| [0.0, 0.0, 10.0 * t];
    let cps = vec![lerp(0.0), lerp(1.0 / 3.0), lerp(2.0 / 3.0), lerp(1.0)];
    let xyz = VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps)
        .unwrap();
    geometry::segment::CubicSegment::try_new(
        xyz,
        vec![],
        15.0,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

fn probe_report(cfg: &PlannerConfig) -> trajectory::streaming::ReplanReport {
    let chains = cfg.post_processors.compile(&cfg.axis_registry).unwrap();
    let ctx = trajectory::streaming::ReplanContext {
        limits: cfg.to_temporal_limits().unwrap(),
        chains: chains.clone(),
        fit_tolerance_mm: cfg.fit_tolerance_mm,
        beta_max_iters: cfg.beta_max_iters,
        beta_convergence_ratio: cfg.beta_convergence_ratio,
        worker_threads: 1,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        fallback_initial_v: 0.0,
        safety_mode: trajectory::plan_velocity::SafetyMode::WorstCaseFuture,
        force_full_resolve: false,
    };
    let home = vec![0.0; chains.n_axes()];
    let mut state = trajectory::streaming::ShaperState::new(&home, &chains);
    state.append_and_replan(z_segment_10mm(), &ctx).unwrap()
}

#[test]
fn probe_neptune_exact_limits() {
    let mut cfg = follower_config(false, 0.0);
    cfg.fit_tolerance_mm = 0.005;
    cfg.limit_sections = vec![
        LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(800.0),
            max_accel: Some(30000.0),
            max_jerk: None,
        },
        LimitSection {
            name: "y".into(),
            axes: vec![1],
            max_velocity: Some(50.0),
            max_accel: Some(4000.0),
            max_jerk: None,
        },
        LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(25.0),
            max_accel: Some(100.0),
            max_jerk: None,
        },
        LimitSection {
            name: "extruder".into(),
            axes: vec![3],
            max_velocity: Some(75.0),
            max_accel: Some(1500.0),
            max_jerk: None,
        },
    ];
    let r = probe_report(&cfg);
    assert!(
        r.plan.beta_converged,
        "neptune-limit z lift must converge (got {} iterations)",
        r.plan.beta_iterations
    );
    assert!(r.plan.beta_iterations <= 4);
}
