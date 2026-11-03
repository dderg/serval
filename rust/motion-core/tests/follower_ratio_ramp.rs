//! A follower's ratio ramp spreads every extrusion-ratio step over the path
//! the leaders cover in the ramp time around it: the extruder velocity is
//! continuous through travel↔extrusion and flow changes taken at speed, the
//! ramp spans `v·τ` of path, and the material is the unramped projection's
//! exactly.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{StreamConfig, StreamInput};
use trajectory::{AxisChainSet, CompiledChain, ContinuousSegment, PostProcessorInstance, algos};

const EXTRUDER_AXIS: usize = 3;
const SAMPLE_DT_S: f64 = 1e-5;
const RATIO: f64 = 0.0333;

fn trident_leaders_with(e: CompiledChain) -> AxisChainSet {
    let shaper = |name: &str, hz: f64| {
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

fn ramped(ramp_time: f64) -> AxisChainSet {
    trident_leaders_with(CompiledChain::default().with_ratio_ramp_time(ramp_time))
}

fn config() -> StreamConfig {
    let mut config = default_stream_config();
    config.fit_tol_mm = 0.005;
    config.integration_tol = 1e-7;
    config.limits = geometry::VelocityLimits::try_new(1000.0, 20_000.0, 0.04, f64::INFINITY)
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

/// Lines along x at `speed` mm/s, each `(length, flow)` with flow the
/// extrusion per mm in units of [`RATIO`]; 0 is travel.
fn colinear(speed: f64, legs: &[(f64, f64)]) -> String {
    let mut gcode = format!("G90\nM83\nG1 X0 Y0 F{:.0}\n", speed * 60.0);
    let mut x = 0.0;
    for &(length, flow) in legs {
        x += length;
        if flow == 0.0 {
            gcode.push_str(&format!("G1 X{x:.4}\n"));
        } else {
            gcode.push_str(&format!("G1 X{x:.4} E{:.6}\n", flow * RATIO * length));
        }
    }
    gcode
}

struct Track {
    path: Vec<f64>,
    speed: Vec<f64>,
    e: Vec<f64>,
    ve: Vec<f64>,
}

fn sample(segments: &[ContinuousSegment]) -> Track {
    let mut track = Track {
        path: Vec::new(),
        speed: Vec::new(),
        e: Vec::new(),
        ve: Vec::new(),
    };
    let mut s = 0.0;
    for segment in segments {
        let mut t = segment.t_start;
        while t < segment.t_end {
            let x = segment.eval_axis(0, t).expect("x evaluates");
            let y = segment.eval_axis(1, t).expect("y evaluates");
            let e = segment.eval_axis(EXTRUDER_AXIS, t).expect("e evaluates");
            let speed = libm::hypot(x.velocity, y.velocity);
            s += speed * SAMPLE_DT_S;
            track.path.push(s);
            track.speed.push(speed);
            track.e.push(e.position);
            track.ve.push(e.velocity);
            t += SAMPLE_DT_S;
        }
    }
    track
}

fn final_e(segments: &[ContinuousSegment]) -> f64 {
    let last = segments.last().expect("segments emitted");
    last.eval_axis(EXTRUDER_AXIS, last.t_end)
        .expect("e evaluates at the end")
        .position
}

/// The largest extruder velocity change between neighbouring samples.
fn largest_velocity_jump(track: &Track) -> f64 {
    track
        .ve
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .fold(0.0, f64::max)
}

/// Path length over which the laid ratio climbs from 5% to 95% of `RATIO`
/// at a travel-to-extrusion seam.
fn rise_width(track: &Track) -> f64 {
    let ratio_at = |index: usize| track.ve[index] / track.speed[index].max(1e-9);
    let moving = |index: usize| track.speed[index] > 1.0;
    let low = (0..track.ve.len())
        .find(|&i| moving(i) && ratio_at(i) > 0.05 * RATIO)
        .expect("the ratio rises");
    let high = (low..track.ve.len())
        .find(|&i| moving(i) && ratio_at(i) > 0.95 * RATIO)
        .expect("the ratio reaches the line's");
    track.path[high] - track.path[low]
}

#[test]
fn a_ratio_step_taken_at_speed_is_a_velocity_step_without_a_ramp() {
    let gcode = colinear(150.0, &[(30.0, 0.0), (30.0, 1.0), (30.0, 0.0)]);
    let jump = largest_velocity_jump(&sample(&shaped(
        &gcode,
        trident_leaders_with(CompiledChain::default()),
    )));
    assert!(
        jump > 0.9 * RATIO * 150.0,
        "the unramped follower steps by the ratio times the speed, got {jump}"
    );
}

#[test]
fn a_ratio_ramp_keeps_the_extruder_velocity_continuous_at_speed() {
    for gcode in [
        colinear(150.0, &[(30.0, 0.0), (30.0, 1.0), (30.0, 0.0)]),
        colinear(200.0, &[(30.0, 1.0), (20.0, 1.5), (20.0, 0.5), (30.0, 1.0)]),
    ] {
        let jump = largest_velocity_jump(&sample(&shaped(&gcode, ramped(0.01))));
        assert!(
            jump < 0.02,
            "a ramped follower's velocity moves {jump} mm/s within one {SAMPLE_DT_S} s sample"
        );
    }
}

#[test]
fn a_ratio_ramp_lays_exactly_the_unramped_material() {
    for gcode in [
        colinear(150.0, &[(30.0, 0.0), (30.0, 1.0), (30.0, 0.0)]),
        colinear(200.0, &[(30.0, 1.0), (20.0, 1.5), (20.0, 0.5), (30.0, 1.0)]),
        colinear(
            150.0,
            &[(20.0, 1.0), (0.4, 0.0), (0.3, 1.0), (0.2, 0.0), (20.0, 1.0)],
        ),
        "G90\nM83\nG1 X0 Y0 F9000\nG1 X20 Y0 E0.666\nG1 X20 Y20\nG1 X40 Y20 E0.999\n\
         G1 X40 Y30 E0.2\nG1 X10 Y30\n"
            .to_string(),
    ] {
        let plain = final_e(&shaped(
            &gcode,
            trident_leaders_with(CompiledChain::default()),
        ));
        let ramped = final_e(&shaped(&gcode, ramped(0.015)));
        assert!(
            (ramped - plain).abs() < 1e-9,
            "the ramped follower ends at {ramped} mm, the unramped one at {plain}"
        );
    }
}

#[test]
fn a_ratio_ramp_spans_the_path_the_leaders_cover_in_its_time() {
    let ramp_time = 0.01;
    let widths: Vec<(f64, f64)> = [50.0, 100.0, 200.0]
        .into_iter()
        .map(|speed| {
            let gcode = colinear(speed, &[(40.0, 0.0), (40.0, 1.0)]);
            (
                speed,
                rise_width(&sample(&shaped(&gcode, ramped(ramp_time)))),
            )
        })
        .collect();
    for (speed, width) in widths {
        let expected = 0.74 * speed * ramp_time;
        assert!(
            (width - expected).abs() < 0.1 * expected,
            "at {speed} mm/s the ratio rises over {width} mm, expected {expected} mm"
        );
    }
}

#[test]
fn ramps_on_segments_shorter_than_their_reach_stay_within_the_commanded_ratios() {
    let gcode = colinear(
        150.0,
        &[
            (20.0, 1.0),
            (0.4, 0.0),
            (0.3, 1.0),
            (0.2, 0.0),
            (0.5, 1.5),
            (20.0, 1.0),
        ],
    );
    let track = sample(&shaped(&gcode, ramped(0.02)));
    for index in 0..track.ve.len() {
        assert!(
            track.ve[index] >= -1e-6,
            "the follower retracts at {} mm: {} mm/s",
            track.path[index],
            track.ve[index]
        );
        let moving = track.speed[index] > 1.0;
        assert!(
            !moving || track.ve[index] <= 1.5 * RATIO * track.speed[index] * (1.0 + 1e-3),
            "the follower outruns the largest commanded ratio at {} mm: {} mm/s at {} mm/s",
            track.path[index],
            track.ve[index],
            track.speed[index]
        );
    }
    let jump = largest_velocity_jump(&track);
    assert!(
        jump < 0.1,
        "ramps a fifth of a millimetre wide at 150 mm/s move the velocity {jump} mm/s per sample"
    );
}

#[test]
fn a_ratio_ramp_lays_the_unramped_material_through_retracts_and_drains() {
    let print = "G90\nM83\nG1 X0 Y0 F9000\nG1 X20 Y0 E0.666\nG1 E-0.8 F2400\n\
                 G1 X20 Y20 F9000\nG1 E0.8 F2400\nG1 X40 Y20 E0.666 F9000\nG1 X60 Y20\n\
                 G1 X60 Y40 E0.666\nG1 X0 Y0\n";
    let script = |chains: AxisChainSet| {
        let mut script: Vec<StreamInput> = Vec::new();
        for _ in 0..2 {
            script.extend(
                parse_gcode_to_moves(print, config().limits)
                    .into_iter()
                    .map(StreamInput::from),
            );
            script.push(StreamInput::Drain);
        }
        collect_shaped_segments_from_script(script, config(), chains)
    };
    let plain = final_e(&script(trident_leaders_with(CompiledChain::default())));
    let ramped = final_e(&script(ramped(0.015)));
    assert!(
        (ramped - plain).abs() < 1e-9,
        "the ramped follower ends at {ramped} mm, the unramped one at {plain}"
    );
}

/// A short smoothing kernel after a ratio ramp and an advance: the ramp
/// takes the ratio steps, the kernel only bounds what the advance asks of
/// the extruder.
fn ramped_with_kernel(smooth_time: f64) -> AxisChainSet {
    let chain = CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![0.02]),
        PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![smooth_time]),
    ])
    .expect("advance and kernel compile")
    .with_ratio_ramp_time(0.015);
    trident_leaders_with(chain)
}

#[test]
fn a_ratio_ramp_runs_ahead_of_a_short_kernel() {
    let gcode = colinear(
        150.0,
        &[
            (20.0, 1.0),
            (0.4, 0.0),
            (0.3, 1.0),
            (0.2, 0.0),
            (20.0, 1.5),
            (20.0, 0.0),
        ],
    );
    let kernel_only = |smooth_time: f64| {
        let chain = CompiledChain::compile(&[
            PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![0.02]),
            PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![smooth_time]),
        ])
        .expect("advance and kernel compile");
        trident_leaders_with(chain)
    };
    let hybrid = sample(&shaped(&gcode, ramped_with_kernel(0.005)));
    let kernel = sample(&shaped(&gcode, kernel_only(0.005)));
    let plain = final_e(&shaped(
        &gcode,
        trident_leaders_with(CompiledChain::default()),
    ));
    let hybrid_end = *hybrid.e.last().expect("sampled");
    assert!(
        (hybrid_end - plain).abs() < 1e-6,
        "the ramp under a kernel ends at {hybrid_end} mm, the unramped projection at {plain}"
    );
    let (hybrid_jump, kernel_jump) = (
        largest_velocity_jump(&hybrid),
        largest_velocity_jump(&kernel),
    );
    assert!(
        hybrid_jump < 0.5 * kernel_jump,
        "the ramp leaves the kernel {hybrid_jump} mm/s per sample to smooth, the kernel alone \
         {kernel_jump}"
    );
}
