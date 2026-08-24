use super::*;

const SQUARE: &str = "\
G90
G1 X0 Y0 F3000
G1 X20 Y0
G1 X20 Y20
G1 X0 Y20
G1 X0 Y0
";

#[test]
fn parse_drops_origin_and_zero_length_moves() {
    let limits = default_stream_config().limits;
    let moves = parse_gcode_to_moves(SQUARE, limits);
    assert_eq!(
        moves.len(),
        4,
        "origin-establishing move is consumed; four cornering moves remain"
    );
}

#[test]
fn relative_mode_is_honored() {
    let limits = default_stream_config().limits;
    let abs = "G90\nG1 X0 Y0 F3000\nG1 X10 Y0\nG1 X10 Y10\n";
    let rel = "G90\nG1 X0 Y0 F3000\nG91\nG1 X10 Y0\nG1 X0 Y10\n";
    assert_eq!(
        parse_gcode_to_moves(abs, limits).len(),
        parse_gcode_to_moves(rel, limits).len(),
        "relative and absolute encodings of the same path yield the same move count"
    );
}

#[test]
fn run_schedule_reports_sane_structure() {
    let report = run_schedule(SQUARE, default_stream_config());
    assert_eq!(report.moves, 4);
    assert!(report.segments > 0, "lowering must emit segments");
    assert!(report.worst() >= 0.0);
}

#[test]
fn pipeline_replay_is_seam_free() {
    let a = run_schedule(SQUARE, default_stream_config());
    let b = run_schedule(SQUARE, default_stream_config());
    for rep in [&a, &b] {
        assert_eq!(
            rep.fatal(),
            0,
            "square must replay without a fatal seam; worst {:?}",
            rep.worst_fatal()
        );
    }
}

const CRASH_VORON_CUBE: &str = include_str!("crash_voron_cube.gcode");

fn bench_config() -> StreamConfig {
    let mut cfg = default_stream_config();
    cfg.limits =
        VelocityLimits::try_new(500.0, 8000.0, 20.0, f64::INFINITY).expect("bench limits valid");
    cfg
}

#[test]
fn arc_fit_voron_cube_perimeter_is_c0() {
    let rep = run_schedule(CRASH_VORON_CUBE, bench_config());
    assert_eq!(
        rep.fatal(),
        0,
        "fatal junction discontinuity (worst={:.4} mm): {:?}",
        rep.worst(),
        rep.worst_fatal()
    );
}

fn extruder_pa_smooth_chain_set() -> trajectory::AxisChainSet {
    extruder_chain_set_with_k(0.03)
}

fn extruder_chain_set_with_k(k: f64) -> trajectory::AxisChainSet {
    extruder_chain_set(k, None)
}

/// Smooth pressure advance: the follower's own kernel convolves its
/// PA-boosted projection — the cascaded window the shaper must gate on.
fn smooth_pa_on_extruder_chain_set() -> trajectory::AxisChainSet {
    extruder_chain_set(0.03, Some(0.015))
}

fn extruder_chain_set(k: f64, e_smooth_time: Option<f64>) -> trajectory::AxisChainSet {
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![k],
    );
    let st = trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![0.02],
    );
    let spatial_chain =
        trajectory::CompiledChain::compile(std::slice::from_ref(&st)).expect("kernel compiles");
    let mut e_instances = vec![pa];
    if let Some(t_sm) = e_smooth_time {
        e_instances.push(trajectory::PostProcessorInstance::new(
            "st_e",
            &trajectory::algos::SmoothTriangle,
            vec![t_sm],
        ));
    }
    let e_chain =
        trajectory::CompiledChain::compile(&e_instances).expect("extruder chain compiles");
    trajectory::AxisChainSet {
        chains: vec![
            spatial_chain.clone(),
            spatial_chain.clone(),
            spatial_chain,
            e_chain,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn axis_position(seg: &ContinuousSegment, axis: usize, t: f64) -> f64 {
    seg.eval_axis(axis, t)
        .expect("shaped axis evaluates at its own endpoint")
        .position
}

fn worst_track_seam(segs: &[ContinuousSegment], axis: usize) -> f64 {
    segs.windows(2)
        .map(|w| {
            let prev_end = axis_position(&w[0], axis, w[0].t_end);
            let next_start = axis_position(&w[1], axis, w[1].t_start);
            (next_start - prev_end).abs()
        })
        .fold(0.0, f64::max)
}

/// Real print gcode (retractions, unretracts, arcs) with the extruder chain
/// active, drained mid-stream at the cadence the pacer uses when the feed
/// runs dry. Every track on every axis must stay continuous through it.
#[test]
#[ignore = "requires continuous finite-jerk follower input"]
fn voron_cube_with_extruder_kernel_survives_pacer_drains() {
    let config = bench_config();
    let moves = parse_gcode_to_moves(CRASH_VORON_CUBE, config.limits);
    assert!(
        moves
            .iter()
            .any(|m| m.segment.spatial.is_none() && !m.segment.followers.is_empty()),
        "gcode must contain extrude-only moves for this test to mean anything"
    );
    let handle = setup_stages(
        config,
        extruder_pa_smooth_chain_set(),
        vec![0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    for (i, m) in moves.into_iter().enumerate() {
        handle.input.send(m.into()).expect("pipeline accepts move");
        if i % 40 == 39 {
            handle
                .input
                .send(motion_pipeline::StreamInput::Drain)
                .expect("pipeline accepts drain");
        }
    }
    drop(handle.input);
    let segs = collector.join().expect("collector thread");
    assert!(
        segs.len() > 100,
        "expected a full print's worth of segments"
    );
    for axis in 0..4 {
        let worst = worst_track_seam(&segs, axis);
        assert!(
            worst < 0.0125,
            "axis {axis} track jumps {worst:.6} mm across a shaped-segment seam"
        );
    }
}

/// The cascaded case: the extruder's own kernel rides its projection while
/// the pacer drains mid-stream. Every axis must stay continuous through the
/// doubled (leader + follower) support windows.
#[test]
#[ignore = "requires continuous finite-jerk follower input"]
fn voron_cube_with_smooth_pa_on_extruder_survives_pacer_drains() {
    let config = bench_config();
    let moves = parse_gcode_to_moves(CRASH_VORON_CUBE, config.limits);
    let handle = setup_stages(
        config,
        smooth_pa_on_extruder_chain_set(),
        vec![0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    for (i, m) in moves.into_iter().enumerate() {
        handle.input.send(m.into()).expect("pipeline accepts move");
        if i % 40 == 39 {
            handle
                .input
                .send(motion_pipeline::StreamInput::Drain)
                .expect("pipeline accepts drain");
        }
    }
    drop(handle.input);
    let segs = collector.join().expect("collector thread");
    assert!(
        segs.len() > 100,
        "expected a full print's worth of segments"
    );
    for axis in 0..4 {
        let worst = worst_track_seam(&segs, axis);
        assert!(
            worst < 0.0125,
            "axis {axis} track jumps {worst:.6} mm across a shaped-segment seam"
        );
    }
}

/// Same replay, observed at the dispatched-span level through enqueue — the
/// stream the junction monitor and the MCU actually see.
#[test]
fn voron_cube_with_extruder_kernel_has_no_span_seams() {
    let config = bench_config();
    let moves = parse_gcode_to_moves(CRASH_VORON_CUBE, config.limits);
    let rep = run_moves_with_chains(&moves, config, extruder_pa_smooth_chain_set(), Some(40));
    assert_eq!(
        rep.boundaries.len(),
        0,
        "span-level seams (worst {:.6} mm): first {:?}",
        rep.worst(),
        rep.boundaries.first()
    );
}

/// SET_PRESSURE_ADVANCE mid-print (a PA calibration pattern does this every
/// band): drain, swap the chain set to a new k, keep printing. The E track
/// must stay continuous through the swap.
#[test]
fn extruder_track_is_continuous_across_a_pressure_advance_change() {
    let config = default_stream_config();
    let band = |x0: f64, line: u32| {
        build_move(
            [x0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            EXTRUDER_AXIS,
            0.5,
            config.limits,
            30.0,
            line,
        )
        .expect("band move builds")
    };

    let handle = setup_stages(
        config,
        extruder_chain_set_with_k(0.03),
        vec![0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    let mut x0 = 0.0;
    for (i, k) in [0.03, 0.042, 0.054, 0.066].into_iter().enumerate() {
        if i > 0 {
            handle
                .input
                .send(motion_pipeline::StreamInput::Drain)
                .expect("pipeline accepts drain");
            handle
                .input
                .send(motion_pipeline::StreamInput::Control(
                    motion_pipeline::Control::SetAxisChains(extruder_chain_set_with_k(k)),
                ))
                .expect("pipeline accepts chain swap");
        }
        for _ in 0..3 {
            handle
                .input
                .send(band(x0, (i * 3) as u32).into())
                .expect("pipeline accepts move");
            x0 += 10.0;
        }
    }
    drop(handle.input);
    let segs = collector.join().expect("collector thread");
    let worst = worst_track_seam(&segs, EXTRUDER_AXIS);
    assert!(
        worst < 1e-3,
        "extruder track jumps {worst:.6} mm across a PA change"
    );
}

fn bell_leader_extruder_chain_set(e_smooth_time: f64) -> trajectory::AxisChainSet {
    let bell =
        trajectory::PostProcessorInstance::new("bell", &trajectory::algos::SmoothBell, vec![0.002]);
    let spatial =
        trajectory::CompiledChain::compile(std::slice::from_ref(&bell)).expect("bell compiles");
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![0.025],
    );
    let st = trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![e_smooth_time],
    );
    let e_chain = trajectory::CompiledChain::compile(&[pa, st]).expect("extruder chain compiles");
    trajectory::AxisChainSet {
        chains: vec![
            spatial.clone(),
            spatial,
            trajectory::CompiledChain::default(),
            e_chain,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

/// Changing the extruder kernel's smooth_time mid-print (SET_PRESSURE_ADVANCE
/// SMOOTH_TIME=…) swaps the kernel at a forced rest. The spans committed by
/// the pre-swap flush were convolved with the old kernel while its window
/// still straddled the deceleration's pressure-advance transient; the resumed
/// track convolves the same kept history with the new kernel, which weighs
/// that transient differently — without a settling hold the seam steps by
/// ~k·ė and the MCU takes it as a one-sample burst (fault -310, trident
/// 2026-07-13 18:55:33: 0.0475 mm = 106 µsteps).
#[test]
fn extruder_track_is_continuous_across_a_smooth_time_change() {
    let config = default_stream_config();
    let gcode = "\
G90
G1 X0 Y0 F3600
G1 X20 Y0 E0.68
G1 X20 Y20 E1.36
G1 X0 Y20 E2.04
G1 X0 Y0 E2.72
G1 X20 Y0 E3.40
G1 X20 Y20 E4.08
G1 X0 Y20 E4.76
G1 X0 Y0 E5.44
G1 X20 Y0 E6.12
";
    let moves = parse_gcode_to_moves(gcode, config.limits);
    assert_eq!(moves.len(), 9, "nine extruding perimeter sides");

    let handle = setup_stages(
        config,
        bell_leader_extruder_chain_set(0.022),
        vec![0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    let swaps = [0.017, 0.03];
    for (i, m) in moves.into_iter().enumerate() {
        if i > 0 && i % 3 == 0 {
            handle
                .input
                .send(motion_pipeline::StreamInput::Drain)
                .expect("pipeline accepts drain");
            handle
                .input
                .send(motion_pipeline::StreamInput::Control(
                    motion_pipeline::Control::SetAxisChains(bell_leader_extruder_chain_set(
                        swaps[i / 3 - 1],
                    )),
                ))
                .expect("pipeline accepts chain swap");
        }
        handle.input.send(m.into()).expect("pipeline accepts move");
    }
    drop(handle.input);
    let segs = collector.join().expect("collector thread");
    for axis in 0..4 {
        let worst = worst_track_seam(&segs, axis);
        assert!(
            worst < 1e-5,
            "axis {axis} track jumps {worst:.6} mm across a smooth_time change"
        );
    }
}

/// A drain flushed while the shaper's window is clamped ("signal constant past
/// the rest") must not disagree with the shaped trajectory once motion
/// resumes: the seam becomes a one-sample step burst on the MCU (fault -310).
#[test]
fn extruder_kernel_track_is_continuous_across_a_drain() {
    let config = default_stream_config();
    let m1 = build_move(
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        EXTRUDER_AXIS,
        0.5,
        config.limits,
        30.0,
        0,
    )
    .expect("printing move 1 builds");
    let m2 = build_move(
        [10.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        EXTRUDER_AXIS,
        1.0,
        config.limits,
        40.0,
        1,
    )
    .expect("unretract-like move builds");

    let handle = setup_stages(
        config,
        extruder_pa_smooth_chain_set(),
        vec![0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    handle.input.send(m1.into()).expect("pipeline accepts m1");
    handle
        .input
        .send(motion_pipeline::StreamInput::Drain)
        .expect("pipeline accepts drain");
    std::thread::sleep(std::time::Duration::from_millis(200));
    handle.input.send(m2.into()).expect("pipeline accepts m2");
    drop(handle.input);
    let segs = collector.join().expect("collector thread");

    assert!(
        segs.len() >= 2,
        "expected segments on both sides of the drain"
    );
    let worst = worst_track_seam(&segs, EXTRUDER_AXIS);
    assert!(
        worst < 1e-3,
        "extruder track jumps {worst:.6} mm across a shaped-segment seam"
    );
}
