//! A follower projected along shaped leaders must not depend on how far into
//! the stream it runs: the same layer started 2^16 s later has to extrude the
//! same way, and each segment's extrusion starts on the span table boundary
//! its predecessor ended on instead of a sliver past it.

use motion_pipeline::{Pipeline, StreamConfig, TrajectoryItem};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    AxisDecl, PostProcessorDecl, SNAPSHOT_MAX_BUFFER_MOVES, TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
    TRAJECTORY_FIT_TOL_MM, VELOCITY_INTEGRATION_TOL, plan_steps,
};
use trajectory::ContinuousSegment;

const MAX_VELOCITY: f64 = 1000.0;
const MAX_ACCEL: f64 = 10_000.0;
const EXTRUDER: usize = 3;
const EIGHTEEN_HOURS_S: f64 = 65_536.0;
const SAMPLES_PER_SEGMENT: usize = 64;
const MIN_DEVICE_INTERVAL_S: f64 = 2e-9;

fn axis(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).into()).collect(),
        motors: Vec::new(),
        post_processors: post_processors.iter().map(|s| (*s).into()).collect(),
        ratio_ramp_time: None,
    }
}

fn shaper(name: &str, ty: &str, frequency_hz: f64) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: vec![("frequency_hz".into(), frequency_hz)],
    }
}

fn dense_top_layer_lines(count: usize) -> String {
    let path = format!(
        "{}/../../tools/sim/gcode/voron_dense_top_layers.gcode",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(path).expect("read dense top layer gcode");
    text.lines().take(count).collect::<Vec<_>>().join("\n")
}

fn plan_from(stream_start_s: f64) -> Vec<ContinuousSegment> {
    let waypoints = parse_gcode(
        &dense_top_layer_lines(300),
        &pipeline_snapshot::CartesianLimits::without_z_caps(
            MAX_VELOCITY,
            MAX_ACCEL,
            f64::INFINITY,
            0.0,
        ),
    )
    .expect("parse");
    let limits = geometry::VelocityLimits::try_new(MAX_VELOCITY, MAX_ACCEL, 0.21, f64::INFINITY)
        .expect("limits");
    let steps = plan_steps(&waypoints, limits).expect("steps");
    let config = StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: TRAJECTORY_FIT_TOL_MM,
        fit_tol_accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    };
    let registry = planner_config::AxisRegistry::try_new(vec![
        axis("x", &[], &["shaper_x"]),
        axis("y", &[], &["shaper_y"]),
        axis("z", &[], &[]),
        axis("e", &["x", "y", "z"], &[]),
    ])
    .expect("axes");
    let chains = planner_config::PostProcessorSet::try_new(
        &registry,
        &[
            shaper("shaper_x", "smooth_mzv", 50.0),
            shaper("shaper_y", "smooth_zv", 44.0),
        ],
    )
    .expect("post-processors")
    .compile(&registry)
    .expect("chains");
    let [x, y, z, _] = waypoints[0].position;
    let mut pipeline = Pipeline::new(config, chains, vec![x, y, z, 0.0], stream_start_s);
    let mut segments = Vec::new();
    let mut collect = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    for step in steps {
        assert!(step.feed(|input| pipeline.feed(input, &mut collect)));
    }
    assert!(pipeline.finish(&mut collect));
    segments
}

#[test]
fn a_layer_extrudes_the_same_eighteen_hours_into_the_stream() {
    let early = plan_from(0.0);
    let late = plan_from(EIGHTEEN_HOURS_S);
    assert_eq!(early.len(), late.len());
    let mut worst = (0.0_f64, 0.0_f64);
    for (early_segment, late_segment) in early.iter().zip(&late) {
        let duration = early_segment.t_end - early_segment.t_start;
        for k in 0..=SAMPLES_PER_SEGMENT {
            let offset = duration * k as f64 / SAMPLES_PER_SEGMENT as f64;
            let early_e = early_segment
                .eval_axis(EXTRUDER, early_segment.t_start + offset)
                .expect("early e")
                .position;
            let late_t = (late_segment.t_start + offset).min(late_segment.t_end);
            let late_e = late_segment
                .eval_axis(EXTRUDER, late_t)
                .expect("late e")
                .position;
            let gap = (early_e - late_e).abs();
            if gap > worst.0 {
                worst = (gap, early_segment.t_start + offset);
            }
        }
    }
    assert!(
        worst.0 <= TRAJECTORY_FIT_TOL_MM,
        "e differs by {} mm at t={} once the stream starts {EIGHTEEN_HOURS_S} s later",
        worst.0,
        worst.1
    );
}

#[test]
fn no_extruder_segment_opens_on_a_sliver_piece() {
    for segment in plan_from(0.0) {
        let first_piece_end = segment.axes[EXTRUDER]
            .breakpoints()
            .into_iter()
            .filter(|&t| t > segment.t_start)
            .fold(segment.t_end, f64::min);
        let first_piece = first_piece_end - segment.t_start;
        assert!(
            first_piece >= MIN_DEVICE_INTERVAL_S,
            "extruder segment [{}, {}] opens on a {first_piece} s piece",
            segment.t_start,
            segment.t_end
        );
    }
}
