use super::*;
use crate::classify::classify_and_build;
use std::sync::atomic::AtomicUsize;

fn counting_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<AtomicUsize>,
) {
    let counter = Arc::new(AtomicUsize::new(0));
    let c = Arc::clone(&counter);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            c.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });
    (cb, counter)
}

fn relaxed_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.fit_tolerance_mm = 0.05;
    c
}

fn long_move() -> ClassifiedMove {
    classify_and_build([0.0; 3], 200.0, 0.0, 0.0, &[], 200.0).unwrap()
}

#[test]
fn submit_and_flush_dispatches_segments() {
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(counter.load(Ordering::Relaxed) > 0, "dispatch never called");
    assert!(h.last_move_time() > 0.0, "print_time not advanced");

    h.shutdown();
}

#[test]
fn shutdown_joins_cleanly() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
    h.shutdown();
    assert!(h.join_handle.is_none());
}

#[test]
fn dwell_advances_print_time_and_unblocks() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);

    h.dwell(0.25).unwrap();
    assert!((h.last_move_time() - 0.25).abs() < 1e-9);

    h.shutdown();
}

#[test]
fn update_runtime_caps_processed_without_error() {
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.update_runtime_caps(RuntimeCaps {
        velocity: Some(200.0),
        accel: Some(2000.0),
    })
    .unwrap();

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(counter.load(Ordering::Relaxed) > 0);
    h.shutdown();
}

fn smooth_mzv_xy_post_processors(freq_x: f64, freq_y: f64) -> crate::config::PostProcessorSet {
    crate::config::PostProcessorSet::try_new(
        &mzv_registry(),
        &[
            crate::config::PostProcessorDecl {
                name: "is_x".into(),
                ty: "smooth_mzv".into(),
                params: vec![("frequency_hz".into(), freq_x)],
            },
            crate::config::PostProcessorDecl {
                name: "is_y".into(),
                ty: "smooth_mzv".into(),
                params: vec![("frequency_hz".into(), freq_y)],
            },
        ],
    )
    .unwrap()
}

fn mzv_registry() -> crate::config::AxisRegistry {
    crate::config::AxisRegistry::try_new(
        [("x", "is_x"), ("y", "is_y"), ("z", "")]
            .iter()
            .map(|(name, pp)| crate::config::AxisDecl {
                name: (*name).to_string(),
                follows: vec![],
                motors: vec![],
                post_processors: if pp.is_empty() {
                    vec![]
                } else {
                    vec![(*pp).to_string()]
                },
            })
            .collect(),
    )
    .unwrap()
}

#[test]
fn update_post_processor_processed_without_error() {
    let (dispatch, _counter) = counting_dispatch();
    let mut cfg = PlannerConfig::default();
    cfg.post_processors = smooth_mzv_xy_post_processors(60.0, 60.0);
    cfg.axis_registry = mzv_registry();
    let mut h = PlannerHandle::spawn(cfg, dispatch);

    h.update_post_processor("is_x", "frequency_hz", 80.0)
        .unwrap();
    assert!(
        h.update_post_processor("ghost", "frequency_hz", 80.0)
            .is_err()
    );
    assert!(h.update_post_processor("is_x", "k", 0.1).is_err());

    h.shutdown();
}

fn segment_capturing_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<std::sync::Mutex<Vec<ShapedSegment>>>,
) {
    let captured = Arc::new(std::sync::Mutex::new(Vec::new()));
    let c = Arc::clone(&captured);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            c.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, captured)
}

fn pa_on_x_config(k: f64) -> PlannerConfig {
    let registry = crate::config::AxisRegistry::try_new(
        ["x", "y", "z"]
            .iter()
            .map(|name| crate::config::AxisDecl {
                name: (*name).to_string(),
                follows: vec![],
                motors: vec![],
                post_processors: if *name == "x" {
                    vec!["pa".to_string()]
                } else {
                    vec![]
                },
            })
            .collect(),
    )
    .unwrap();
    let mut cfg = relaxed_config();
    cfg.post_processors = crate::config::PostProcessorSet::try_new(
        &registry,
        &[crate::config::PostProcessorDecl {
            name: "pa".into(),
            ty: "linear_pressure_advance".into(),
            params: vec![("k".into(), k)],
        }],
    )
    .unwrap();
    cfg.axis_registry = registry;
    cfg
}

fn peak_x_track_speed(batch: &[ShapedSegment]) -> f64 {
    let mut peak = 0.0_f64;
    for seg in batch {
        let n = 200;
        let h = (seg.t_end - seg.t_start) / n as f64;
        for i in 0..n {
            let t = seg.t_start + i as f64 * h;
            let a = nurbs::eval::eval(&seg.axes[0], t);
            let b = nurbs::eval::eval(&seg.axes[0], t + h);
            peak = peak.max(((b - a) / h).abs());
        }
    }
    peak
}

#[test]
fn update_post_processor_applies_to_new_plans_only() {
    let (dispatch, captured) = segment_capturing_dispatch();
    let mut h = PlannerHandle::spawn(pa_on_x_config(0.0), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let batch_a: Vec<ShapedSegment> = captured.lock().unwrap().clone();
    assert!(!batch_a.is_empty());

    h.update_post_processor("pa", "k", 0.05).unwrap();
    let count_after_update = captured.lock().unwrap().len();
    assert_eq!(
        count_after_update,
        batch_a.len(),
        "runtime tuning must not re-emit dispatched pieces"
    );

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let batch_b: Vec<ShapedSegment> = captured.lock().unwrap()[count_after_update..].to_vec();
    assert!(!batch_b.is_empty());

    let peak_a = peak_x_track_speed(&batch_a);
    let peak_b = peak_x_track_speed(&batch_b);
    assert!(
        peak_b > peak_a + 4.0,
        "PA gain k=0.05 must visibly boost the peak emitted track speed \
         while accelerating (got peak_a={peak_a}, peak_b={peak_b})"
    );

    h.shutdown();
}

#[test]
fn submit_triggers_replan_per_move() {
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(
        counter.load(Ordering::Relaxed) > 0,
        "submit_move did not trigger per-move dispatch",
    );
    assert!(h.last_move_time() > 0.0);
    h.shutdown();
}

#[test]
fn drop_without_explicit_shutdown_does_not_hang() {
    let (dispatch, _counter) = counting_dispatch();
    let h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
    drop(h);
}

#[test]
fn z_only_move_after_homing_xy_shaped_axes_are_constant() {
    use crate::classify::classify_and_build;

    let mut cfg = PlannerConfig::default();
    cfg.limit_sections = vec![
        crate::config::LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(1000.0),
            max_accel: Some(70000.0),
            max_jerk: None,
        },
        crate::config::LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(5.0),
            max_accel: Some(100.0),
            max_jerk: None,
        },
    ];
    cfg.post_processors = smooth_mzv_xy_post_processors(186.0, 122.0);
    cfg.axis_registry = mzv_registry();

    let replan_ctx = build_replan_context(&cfg);
    let mut state = ShaperState::new(&[0.0; 3], &replan_ctx.chains);
    let emit_ctx = EmitContext {
        chains: &replan_ctx.chains,
    };

    let do_move =
        |state: &mut ShaperState, start: [f64; 3], dx: f64, dy: f64, dz: f64, feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, &[], feed)
                .expect("classify_and_build should succeed for valid moves");
            state
                .append_and_replan(m.segment, &replan_ctx)
                .expect("append_and_replan should succeed");
            state
                .emit_committed(&emit_ctx)
                .expect("emit_committed should succeed")
        };

    let do_flush = |state: &mut ShaperState| -> Vec<ShapedSegment> {
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero should succeed")
    };

    state.reset(&[-154.5, 0.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [-154.5, 0.0, 0.0], 454.5, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset(&[300.0, 0.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [300.0, 0.0, 0.0], -5.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [295.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [195.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);

    state.reset(&[95.0, -151.5, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [95.0, -151.5, 0.0], 0.0, 453.5, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset(&[95.0, 302.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [95.0, 302.0, 0.0], 0.0, -5.0, 0.0, 100.0);
    let _ = do_move(&mut state, [95.0, 297.0, 0.0], 55.0, -165.0, 0.0, 300.0);
    let _ = do_flush(&mut state);

    state.reset(&[150.0, 132.0, 344.0], &replan_ctx.chains);

    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.0, 0.0, -342.0, &[], 8.0)
        .expect("classify Z move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("append Z move");

    let mut z_segments: Vec<trajectory::ShapedSegment> = Vec::new();
    z_segments.extend(state.emit_committed(&emit_ctx).expect("emit_committed"));
    z_segments.extend(
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero"),
    );

    assert!(
        !z_segments.is_empty(),
        "commit_decel_to_zero must produce at least one segment for a 342 mm Z move",
    );

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;

    for seg in &z_segments {
        let dev_x = seg.axes[0]
            .control_points()
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1]
            .control_points()
            .iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
    }

    assert!(
        max_dev_x < 0.01,
        "Z-only move after XY homing: X deviated by {max_dev_x:.6} mm from 150.0 \
         (expected < 10µm)",
    );

    assert!(
        max_dev_y < 0.01,
        "Z-only move after XY homing: Y deviated by {max_dev_y:.6} mm from 132.0 \
         (expected < 10µm)",
    );
}

#[test]
fn z_move_with_tiny_x_after_homing_xy_deviation_proportional() {
    use crate::classify::classify_and_build;

    let mut cfg = PlannerConfig::default();
    cfg.limit_sections = vec![
        crate::config::LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(1000.0),
            max_accel: Some(70000.0),
            max_jerk: None,
        },
        crate::config::LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(5.0),
            max_accel: Some(100.0),
            max_jerk: None,
        },
    ];
    cfg.post_processors = smooth_mzv_xy_post_processors(186.0, 122.0);
    cfg.axis_registry = mzv_registry();

    let replan_ctx = build_replan_context(&cfg);
    let mut state = ShaperState::new(&[0.0; 3], &replan_ctx.chains);
    let emit_ctx = EmitContext {
        chains: &replan_ctx.chains,
    };

    let do_move =
        |state: &mut ShaperState, start: [f64; 3], dx: f64, dy: f64, dz: f64, feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, &[], feed).expect("classify");
            state
                .append_and_replan(m.segment, &replan_ctx)
                .expect("replan");
            state.emit_committed(&emit_ctx).expect("emit")
        };
    let do_flush = |state: &mut ShaperState| -> Vec<trajectory::ShapedSegment> {
        state.commit_decel_to_zero(&emit_ctx).expect("flush")
    };

    state.reset(&[-154.5, 0.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [-154.5, 0.0, 0.0], 454.5, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset(&[300.0, 0.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [300.0, 0.0, 0.0], -5.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [295.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [195.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset(&[95.0, -151.5, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [95.0, -151.5, 0.0], 0.0, 453.5, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset(&[95.0, 302.0, 0.0], &replan_ctx.chains);
    let _ = do_move(&mut state, [95.0, 302.0, 0.0], 0.0, -5.0, 0.0, 100.0);
    let _ = do_move(&mut state, [95.0, 297.0, 0.0], 55.0, -165.0, 0.0, 300.0);
    let _ = do_flush(&mut state);

    state.reset(&[150.0, 132.0, 344.0], &replan_ctx.chains);
    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.1, 0.0, -342.0, &[], 8.0)
        .expect("classify Z+tiny-X move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for seg in &segs {
        let dev_x = seg.axes[0]
            .control_points()
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1]
            .control_points()
            .iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
    }

    assert!(
        max_dev_x < 1.0,
        "tiny-X move: X deviated {max_dev_x:.3}mm from 150.0 (expected < 1mm for 0.1mm input)",
    );
    assert!(
        max_dev_y < 0.01,
        "tiny-X move: Y deviated {max_dev_y:.6}mm from 132.0 (expected < 10µm)",
    );
}

fn capturing_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<std::sync::Mutex<Vec<(f64, f64)>>>,
) {
    let log = Arc::new(std::sync::Mutex::new(Vec::new()));
    let l = Arc::clone(&log);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            l.lock().unwrap().push((seg.t_start, seg.t_end));
            Ok(())
        });
    (cb, log)
}

#[test]
fn quiescence_keeps_timeline_monotone_next_move_does_not_rewind() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    fn wait_for_commits(h: &PlannerHandle, target: u32) {
        let start = std::time::Instant::now();
        while h.commit_fire_count() < target {
            assert!(
                start.elapsed() < Duration::from_secs(5),
                "commit fired only {} of {target} times within 5s",
                h.commit_fire_count()
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    h.submit_move(long_move()).unwrap();
    wait_for_commits(&h, 1);
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    log.lock().unwrap().clear();
    std::thread::sleep(Duration::from_millis(400));
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, &[], 200.0).unwrap();
    h.submit_move(m2).unwrap();
    wait_for_commits(&h, 2);
    let m2_min_t_start = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(s, _)| s)
        .fold(f64::INFINITY, f64::min);
    assert!(
        m2_min_t_start.is_finite(),
        "move 2 produced no dispatched segments"
    );

    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound: move 2 started at {m2_min_t_start}, move 1 ended at {m1_max_t_end}"
    );

    h.shutdown();
}

#[test]
fn move_after_idle_resumes_with_dispatch_lead() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    let idle_extra = 0.3;
    std::thread::sleep(Duration::from_secs_f64(idle_extra));

    log.lock().unwrap().clear();
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, &[], 200.0).unwrap();
    h.submit_move(m2).unwrap();
    h.flush().unwrap();
    let m2_min_t_start = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(s, _)| s)
        .fold(f64::INFINITY, f64::min);
    assert!(
        m2_min_t_start.is_finite(),
        "move 2 produced no dispatched segments"
    );

    // flush() returns at wall time sync + m1_end + LEAD, then we idle a bit
    // more; the resume must anchor at elapsed-since-sync at receipt PLUS a
    // full dispatch lead, or the replan solve eats the cushion and seg0
    // reaches the MCU already in the past (-308 PieceStartInPast on jog).
    let min_expected = m1_max_t_end + LEAD + idle_extra + LEAD - 0.1;
    assert!(
        m2_min_t_start >= min_expected,
        "move after idle anchored too early: started {m2_min_t_start:.3}, \
         expected >= {min_expected:.3} (m1 ended {m1_max_t_end:.3})"
    );

    h.shutdown();
}

#[test]
fn z_only_move_no_prior_xy_motion() {
    use crate::classify::classify_and_build;

    let mut cfg = PlannerConfig::default();
    cfg.limit_sections = vec![
        crate::config::LimitSection {
            name: "gantry".into(),
            axes: vec![0, 1],
            max_velocity: Some(1000.0),
            max_accel: Some(70000.0),
            max_jerk: None,
        },
        crate::config::LimitSection {
            name: "z".into(),
            axes: vec![2],
            max_velocity: Some(5.0),
            max_accel: Some(100.0),
            max_jerk: None,
        },
    ];
    cfg.post_processors = smooth_mzv_xy_post_processors(186.0, 122.0);
    cfg.axis_registry = mzv_registry();

    let replan_ctx = build_replan_context(&cfg);
    let mut state = ShaperState::new(&[0.0; 3], &replan_ctx.chains);
    let emit_ctx = EmitContext {
        chains: &replan_ctx.chains,
    };

    state.reset(&[150.0, 132.0, 344.0], &replan_ctx.chains);

    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.0, 0.0, -342.0, &[], 8.0)
        .expect("classify Z move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for (i, seg) in segs.iter().enumerate() {
        let cps_x = seg.axes[0].control_points();
        let cps_y = seg.axes[1].control_points();
        let dev_x = cps_x
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = cps_y
            .iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
        eprintln!(
            "[no_prior] seg[{i}]: t=[{:.3},{:.3}] X dev={:.6}mm Y dev={:.6}mm",
            seg.t_start, seg.t_end, dev_x, dev_y,
        );
    }

    assert!(
        max_dev_x < 0.01,
        "Z-only move without prior XY motion: X deviated by {max_dev_x:.6}mm (expected < 10µm)",
    );
    assert!(
        max_dev_y < 0.01,
        "Z-only move without prior XY motion: Y deviated by {max_dev_y:.6}mm (expected < 10µm)",
    );
}

#[test]
#[ignore]
fn flush_blocks_until_motion_complete_by_clock() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);
    let t0 = std::time::Instant::now();
    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let elapsed = t0.elapsed().as_secs_f64();
    assert!(
        elapsed >= 0.25 * 0.9,
        "flush returned too early: {:.4}s",
        elapsed
    );
    h.shutdown();
}

fn peak_speed_of_single_x_move(max_velocity: f64, max_accel: f64, feedrate: f64) -> f64 {
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections[0].max_velocity = Some(max_velocity);
    cfg.limit_sections[0].max_accel = Some(max_accel);

    let replan_ctx = build_replan_context(&cfg);
    let mut state = ShaperState::new(&[0.0; 3], &replan_ctx.chains);
    let emit_ctx = EmitContext {
        chains: &replan_ctx.chains,
    };

    state.reset(&[0.0; 3], &replan_ctx.chains);
    let m = classify_and_build([0.0; 3], 600.0, 0.0, 0.0, &[], feedrate)
        .expect("classify_and_build should succeed");
    state
        .append_and_replan(m.segment, &replan_ctx)
        .expect("append_and_replan should succeed");

    let mut segs: Vec<ShapedSegment> = Vec::new();
    segs.extend(
        state
            .emit_committed(&emit_ctx)
            .expect("emit_committed should succeed"),
    );
    segs.extend(
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero should succeed"),
    );
    assert!(!segs.is_empty(), "move produced no shaped segments");

    let mut peak = 0.0_f64;
    for seg in &segs {
        let vel: Vec<nurbs::ScalarNurbs<f64>> =
            seg.axes.iter().map(nurbs::eval::derivative).collect();
        const SAMPLE_DT: f64 = 2e-4;
        let steps = ((seg.t_end - seg.t_start) / SAMPLE_DT).ceil().max(1.0) as usize;
        for i in 0..=steps {
            let t = seg.t_start + (seg.t_end - seg.t_start) * (i as f64) / (steps as f64);
            let speed = vel
                .iter()
                .map(|d| nurbs::eval::eval(d, t).powi(2))
                .sum::<f64>()
                .sqrt();
            peak = peak.max(speed);
        }
    }
    peak
}

#[test]
fn motion_at_velocity_limit_cruises_at_limit() {
    let peak = peak_speed_of_single_x_move(1000.0, 50_000.0, 1000.0);
    assert!(
        (peak - 1000.0).abs() < 15.0,
        "feedrate at machine limit (1000 mm/s): peak speed {peak:.1} mm/s, expected ≈ 1000",
    );
}

#[test]
fn motion_above_velocity_limit_clamps_to_limit() {
    let peak = peak_speed_of_single_x_move(1000.0, 50_000.0, 1100.0);
    assert!(
        (peak - 1000.0).abs() < 15.0,
        "feedrate above machine limit (1100 > 1000 mm/s): \
         peak speed {peak:.1} mm/s, expected clamp to ≈ 1000",
    );
}

#[test]
fn flush_then_move_dispatches_without_error() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    log.lock().unwrap().clear();
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, &[], 200.0).unwrap();
    h.submit_move(m2).unwrap();
    h.flush().unwrap();
    let m2_log = log.lock().unwrap().clone();

    assert!(!m2_log.is_empty(), "move 2 produced no dispatched segments");

    let m2_min_t_start = m2_log.iter().map(|&(s, _)| s).fold(f64::INFINITY, f64::min);

    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound across flush boundary: \
         move 2 t_start={m2_min_t_start:.6} < move 1 t_end={m1_max_t_end:.6}",
    );

    h.shutdown();
}

#[test]
fn resume_lead_covers_observed_worst_case() {
    let observed_worst = 0.867;
    let lead = resume_lead_secs(observed_worst);
    assert!(
        lead > observed_worst,
        "resume cushion {lead} must exceed the {observed_worst}s solve so seg0 \
         starts strictly after the solve finishes",
    );
    assert!(
        lead <= MAX_RESUME_LEAD_SECS + 1e-12,
        "resume cushion {lead} must stay at or below the MCU horizon cap \
         {MAX_RESUME_LEAD_SECS}",
    );
}

#[test]
fn resume_lead_never_below_floor() {
    assert_eq!(
        resume_lead_secs(0.0),
        LEAD,
        "a zero recent-solve estimate must fall back to the legacy LEAD floor",
    );
    assert_eq!(
        resume_lead_secs(-1.0),
        LEAD,
        "a degenerate negative estimate must still clamp up to LEAD",
    );
}

#[test]
fn resume_lead_capped_at_mcu_horizon() {
    let lead = resume_lead_secs(10.0);
    assert_eq!(
        lead, MAX_RESUME_LEAD_SECS,
        "an absurd estimate must clamp to the MCU dispatch-ahead ceiling",
    );
    assert!(
        lead < crate::pump::MAX_LEAD_SECS,
        "the capped cushion {lead} must stay strictly under the pump horizon \
         {} so the pump never holds seg0",
        crate::pump::MAX_LEAD_SECS,
    );
}

fn update_worst(worst: &mut f64, replan_secs: f64) {
    *worst = replan_secs.max(*worst * WORST_REPLAN_DECAY);
}

#[test]
fn worst_replan_estimate_tracks_spike_then_decays() {
    let mut worst = INITIAL_WORST_REPLAN_SECS;
    for replan in [0.06_f64, 0.06, 0.06] {
        update_worst(&mut worst, replan);
    }
    update_worst(&mut worst, 0.867);
    assert!(
        worst >= 0.867,
        "an 867ms spike must lift the estimate to at least the spike: got {worst}",
    );

    update_worst(&mut worst, 0.06);
    assert!(
        worst >= 0.6,
        "one fast solve right after a spike must not collapse the estimate; \
         got {worst}",
    );
    assert!(
        resume_lead_secs(worst) >= MAX_RESUME_LEAD_SECS - 1e-9,
        "the post-spike estimate must still demand the maximum safe cushion",
    );

    for _ in 0..200 {
        update_worst(&mut worst, 0.06);
    }
    assert!(
        worst <= 0.07,
        "after a long quiet steady state the estimate must relax toward the \
         median solve cost; got {worst}",
    );
}
