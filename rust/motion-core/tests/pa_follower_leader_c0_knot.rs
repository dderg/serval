use std::f64::consts::TAU;
use std::fmt::Write;

use geometry::VelocityLimits;
use motion_core::seam_test_harness::{
    collect_shaped_segments_scripted, default_stream_config, parse_gcode_to_moves,
};
use trajectory::{AxisChainSet, CompiledChain, ContinuousSegment, PostProcessorInstance};

const EXTRUDER_AXIS: usize = 3;
const PRESSURE_ADVANCE_S: f64 = 0.03;
const SMOOTH_TIME_S: f64 = 0.02;
const RADIUS_MM: f64 = 20.0;
const CHORDS_PER_LAP: u32 = 72;
const LAPS: u32 = 2;
const FEED_MM_S: f64 = 200.0;
const EXTRUSION_PER_MM: f64 = 0.04;
const MAX_VELOCITY_MM_S: f64 = 500.0;
const MAX_ACCEL_MM_S2: f64 = 8000.0;

fn unsmoothed_linear_advance(toolhead: CompiledChain) -> AxisChainSet {
    let advance = CompiledChain::compile(&[PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![PRESSURE_ADVANCE_S],
    )])
    .expect("a lone linear advance compiles");
    AxisChainSet {
        chains: vec![
            toolhead.clone(),
            toolhead,
            CompiledChain::default(),
            advance,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn smoothing_kernel() -> CompiledChain {
    CompiledChain::compile(&[PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![SMOOTH_TIME_S],
    )])
    .expect("a lone smoothing kernel compiles")
}

fn extruded_circle() -> String {
    let chord = TAU * RADIUS_MM / f64::from(CHORDS_PER_LAP);
    let mut gcode = format!(
        "G90\nM83\nG1 X{:.4} Y100 Z0.2 F{}\n",
        100.0 + RADIUS_MM,
        FEED_MM_S * 60.0
    );
    for index in 1..=CHORDS_PER_LAP * LAPS {
        let angle = TAU * f64::from(index) / f64::from(CHORDS_PER_LAP);
        writeln!(
            gcode,
            "G1 X{:.4} Y{:.4} E{:.5}",
            100.0 + RADIUS_MM * libm::cos(angle),
            100.0 + RADIUS_MM * libm::sin(angle),
            chord * EXTRUSION_PER_MM
        )
        .expect("writing to a String cannot fail");
    }
    gcode
}

fn print_circle(chains: AxisChainSet) -> (Vec<ContinuousSegment>, f64) {
    let mut config = default_stream_config();
    config.limits =
        VelocityLimits::try_new(MAX_VELOCITY_MM_S, MAX_ACCEL_MM_S2, 20.0, f64::INFINITY)
            .expect("bench limits are valid");
    let step_tolerance = 0.5 * config.fit_tol_mm * EXTRUSION_PER_MM;
    let moves = parse_gcode_to_moves(&extruded_circle(), config.limits);
    assert_eq!(moves.len(), (CHORDS_PER_LAP * LAPS) as usize);
    (
        collect_shaped_segments_scripted(&moves, config, chains, None),
        step_tolerance,
    )
}

fn extruder_position(segment: &ContinuousSegment, t: f64) -> f64 {
    segment.axes[EXTRUDER_AXIS]
        .eval_pva(t)
        .expect("the extruder evaluates inside its segment")
        .position
}

fn assert_extruder_never_steps(segments: &[ContinuousSegment], step_tolerance: f64) {
    let rate_ceiling =
        EXTRUSION_PER_MM * (MAX_VELOCITY_MM_S + PRESSURE_ADVANCE_S * MAX_ACCEL_MM_S2);
    for pair in segments.windows(2) {
        let step = (extruder_position(&pair[1], pair[1].t_start)
            - extruder_position(&pair[0], pair[0].t_end))
        .abs();
        assert!(
            step <= step_tolerance,
            "extruder steps {step} mm across the seam at {}",
            pair[1].t_start
        );
    }
    for segment in segments {
        let axis = &segment.axes[EXTRUDER_AXIS];
        let mut breaks = axis.breakpoints();
        breaks.extend([segment.t_start, segment.t_end]);
        breaks.retain(|&t| t >= segment.t_start && t <= segment.t_end);
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        for knot in &breaks[1..breaks.len() - 1] {
            let before = extruder_position(segment, f64::from_bits(knot.to_bits() - 1));
            let after = extruder_position(segment, f64::from_bits(knot.to_bits() + 1));
            assert!(
                (after - before).abs() <= step_tolerance,
                "extruder steps {} mm at the knot {knot}",
                (after - before).abs()
            );
        }
        for window in breaks.windows(2) {
            let bounds = axis
                .pva_bounds(window[0], window[1])
                .expect("a window inside its segment");
            let rate = bounds.velocity_min.abs().max(bounds.velocity_max.abs());
            assert!(
                rate <= rate_ceiling,
                "extruder runs {rate} mm/s on [{}, {}], above the {rate_ceiling} mm/s a \
                 {PRESSURE_ADVANCE_S} s advance can ask at the configured limits",
                window[0],
                window[1]
            );
        }
    }
}

#[test]
fn unsmoothed_advance_over_shaped_leaders_never_steps_the_extruder() {
    let (segments, step_tolerance) = print_circle(unsmoothed_linear_advance(smoothing_kernel()));
    assert_extruder_never_steps(&segments, step_tolerance);
}

#[test]
fn unsmoothed_advance_over_raw_leaders_never_steps_the_extruder() {
    let (segments, step_tolerance) =
        print_circle(unsmoothed_linear_advance(CompiledChain::default()));
    assert_extruder_never_steps(&segments, step_tolerance);
}
