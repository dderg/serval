//! A pressure advance on a ratio-ramped follower, with no kernel after it:
//! the advance reads only the extrusion laid along the leaders' path, so
//! retracts, primes and the rest of the extrude-only moves keep their raw
//! trapezoids; the extruder velocity stays continuous through retracts,
//! travels, wipes and z-hops; and a step that would reach the motor is
//! refused loudly.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{StreamConfig, StreamInput};
use trajectory::{
    AxisChainSet, CompiledChain, ContinuousSegment, FollowerLimits, PostProcessorInstance, algos,
};

const EXTRUDER_AXIS: usize = 3;
const SAMPLE_DT_S: f64 = 1e-5;
const ADVANCE_S: f64 = 0.02;
const EXTRUDE_ONLY_VELOCITY: f64 = 40.0;
const EXTRUDE_ONLY_ACCEL: f64 = 2_000.0;
/// A velocity step of the kinds a ratio-ramped advance refuses — `k·r·a`
/// at a leader's acceleration step or a turnaround — moves the extruder by
/// a millimetre per second or more within one sample, 10⁵ mm/s²; motion
/// with no step stays under half that.
const STEP_FREE_ACCELERATION: f64 = 50_000.0;

fn leaders_with(shaped: bool, e: CompiledChain) -> AxisChainSet {
    let shaper = |name: &str, hz: f64| {
        if !shaped {
            return CompiledChain::default();
        }
        CompiledChain::compile(&[PostProcessorInstance::new(
            name,
            &algos::SmoothMzv,
            vec![hz],
        )])
        .expect("smooth_mzv compiles")
    };
    AxisChainSet {
        chains: vec![
            shaper("shx", 191.0),
            shaper("shy", 129.4),
            CompiledChain::default(),
            e,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn ramped(advance: Option<f64>) -> CompiledChain {
    let chain = match advance {
        Some(k) => CompiledChain::compile(&[PostProcessorInstance::new(
            "pa",
            &algos::LinearPressureAdvance,
            vec![k],
        )])
        .expect("linear pressure advance compiles"),
        None => CompiledChain::default(),
    };
    chain.with_follower_limits(FollowerLimits {
        max_velocity_mm_s: EXTRUDE_ONLY_VELOCITY,
        max_accel_mm_s2: EXTRUDE_ONLY_ACCEL,
    })
}

fn config() -> StreamConfig {
    let mut config = default_stream_config();
    config.fit_tol_mm = 0.005;
    config.integration_tol = 1e-7;
    config.max_extrude_only_velocity_mm_s = EXTRUDE_ONLY_VELOCITY;
    config.max_extrude_only_accel_mm_s2 = EXTRUDE_ONLY_ACCEL;
    config.limits = geometry::VelocityLimits::try_new(
        1000.0,
        20_000.0,
        geometry::CornerBudget::Shaped(0.04),
        f64::INFINITY,
    )
    .expect("test limits are valid");
    config
}

fn shaped(gcode: &str, chains: AxisChainSet) -> Vec<ContinuousSegment> {
    let mut script: Vec<StreamInput> = parse_gcode_to_moves(gcode, config().limits)
        .into_iter()
        .map(StreamInput::from)
        .collect();
    script.push(StreamInput::Drain);
    collect_shaped_segments_from_script(script, config(), chains)
}

/// Retract, z-hop, travel, unhop, unretract and extrude again; then a wipe
/// that retracts while it travels back along the line it just laid.
const RETRACT_TRAVEL_WIPE: &str = "G90\nM83\nG1 X0 Y0 Z0.2 F9000\n\
    G1 X30 Y0 E1.0\nG1 X30 Y20 E0.66\n\
    G1 E-0.8 F2400\nG1 Z0.6 F1200\nG1 X60 Y40 F18000\nG1 Z0.2 F1200\nG1 E0.8 F2400\n\
    G1 X90 Y40 E1.0 F9000\nG1 X60 Y40 E-0.5\nG1 X40 Y40\n\
    G1 E0.5 F2400\nG1 X40 Y60 E0.66 F12000\nG1 X10 Y60 E1.0\n";

struct Track {
    t: Vec<f64>,
    leader_speed: Vec<f64>,
    e: Vec<f64>,
    ve: Vec<f64>,
}

fn sample(segments: &[ContinuousSegment]) -> Track {
    let mut track = Track {
        t: Vec::new(),
        leader_speed: Vec::new(),
        e: Vec::new(),
        ve: Vec::new(),
    };
    for segment in segments {
        let mut t = segment.t_start;
        while t < segment.t_end {
            let speed = (0..EXTRUDER_AXIS)
                .map(|axis| {
                    segment
                        .eval_axis(axis, t)
                        .expect("leader evaluates")
                        .velocity
                })
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            let e = segment.eval_axis(EXTRUDER_AXIS, t).expect("e evaluates");
            track.t.push(t);
            track.leader_speed.push(speed);
            track.e.push(e.position);
            track.ve.push(e.velocity);
            t += SAMPLE_DT_S;
        }
    }
    track
}

#[test]
fn extrude_only_moves_keep_their_raw_motion_under_a_ramped_advance() {
    let advanced = sample(&shaped(
        RETRACT_TRAVEL_WIPE,
        leaders_with(true, ramped(Some(ADVANCE_S))),
    ));
    let plain = sample(&shaped(
        RETRACT_TRAVEL_WIPE,
        leaders_with(true, ramped(None)),
    ));
    assert_eq!(
        advanced.t.len(),
        plain.t.len(),
        "the extruder chain must not change the timeline"
    );
    let mut extrude_only_samples = 0;
    for index in 0..advanced.t.len() {
        if plain.leader_speed[index] > 1e-6 || plain.ve[index] == 0.0 {
            continue;
        }
        extrude_only_samples += 1;
        assert!(
            (advanced.ve[index] - plain.ve[index]).abs() < 1e-3,
            "at t={} the leaders rest and the extruder moves on its own at {} mm/s, yet the \
             advanced one moves at {} mm/s",
            advanced.t[index],
            plain.ve[index],
            advanced.ve[index]
        );
        assert!(
            advanced.ve[index].abs() <= EXTRUDE_ONLY_VELOCITY * (1.0 + 1e-6),
            "at t={} an extrude-only move runs at {} mm/s past its {EXTRUDE_ONLY_VELOCITY} mm/s \
             cap",
            advanced.t[index],
            advanced.ve[index]
        );
    }
    assert!(
        extrude_only_samples > 1000,
        "the fixture must retract and prime at rest, sampled {extrude_only_samples} times"
    );
}

#[test]
fn a_ramped_advance_is_velocity_continuous_through_retracts_wipes_and_z_hops() {
    let track = sample(&shaped(
        RETRACT_TRAVEL_WIPE,
        leaders_with(true, ramped(Some(ADVANCE_S))),
    ));
    let (jump, at) = track
        .ve
        .windows(2)
        .enumerate()
        .map(|(index, pair)| ((pair[1] - pair[0]).abs(), track.t[index]))
        .fold((0.0, 0.0), |best, candidate| {
            if candidate.0 > best.0 {
                candidate
            } else {
                best
            }
        });
    let acceleration = jump / SAMPLE_DT_S;
    assert!(
        acceleration < STEP_FREE_ACCELERATION,
        "the advanced extruder velocity moves {jump} mm/s within one {SAMPLE_DT_S} s sample at \
         t={at}: {acceleration} mm/s², a step"
    );
}

#[test]
fn a_ramped_advance_lays_the_material_of_the_unadvanced_projection() {
    let advanced = shaped(
        RETRACT_TRAVEL_WIPE,
        leaders_with(true, ramped(Some(ADVANCE_S))),
    );
    let plain = shaped(RETRACT_TRAVEL_WIPE, leaders_with(true, ramped(None)));
    let end = |segments: &[ContinuousSegment]| {
        let last = segments.last().expect("segments emitted");
        last.eval_axis(EXTRUDER_AXIS, last.t_end)
            .expect("e evaluates at the end")
            .position
    };
    let (advanced, plain) = (end(&advanced), end(&plain));
    assert!(
        (advanced - plain).abs() < 1e-6,
        "the advanced follower ends at {advanced} mm, the unadvanced one at {plain} mm"
    );
}

#[test]
#[should_panic(expected = "ratio-ramped follower would step its velocity")]
fn a_ramped_advance_on_unshaped_leaders_is_refused() {
    shaped(
        "G90\nM83\nG1 X0 Y0 F9000\nG1 X30 Y0 E1.0\nG1 X30 Y20 E0.66\n",
        leaders_with(false, ramped(Some(ADVANCE_S))),
    );
}

#[test]
fn a_turnaround_between_extruding_lines_keeps_the_advanced_velocity_continuous() {
    let track = sample(&shaped(
        "G90\nM83\nG1 X0 Y0 F9000\nG1 X30 Y0 E1.0\nG1 X5 Y0 E0.83\nG1 X25 Y0 E0.66\n",
        leaders_with(true, ramped(Some(ADVANCE_S))),
    ));
    let largest = track
        .ve
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .fold(0.0, f64::max);
    assert!(
        largest / SAMPLE_DT_S < STEP_FREE_ACCELERATION,
        "the extruder velocity moves {largest} mm/s within one sample around the turnarounds"
    );
}

#[test]
fn a_travel_that_turns_back_along_the_line_just_laid_keeps_the_advanced_velocity_continuous() {
    let track = sample(&shaped(
        "G90\nM83\nG1 X0 Y0 F3000\nG1 X30 Y0 E1.0\nG1 X5 Y0 F9000\n",
        leaders_with(true, ramped(Some(ADVANCE_S))),
    ));
    let largest = track
        .ve
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .fold(0.0, f64::max);
    assert!(
        largest / SAMPLE_DT_S < STEP_FREE_ACCELERATION,
        "the extruder velocity moves {largest} mm/s within one sample at the turnaround"
    );
}
