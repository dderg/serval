//! Scratch investigation tool (not committed): print the exact kinematic
//! parameters of the *fitted* moves around a given source line, in full float
//! precision, so a standalone probe can replay the velocity planner on them.
//!
//!   cargo run -p pipeline-snapshot --example kin_extract -- <file.gcode> \
//!       <max_velocity> <max_accel> <scv> <max_jerk> <line>

use crossbeam_channel::unbounded;
use geometry::path::CurvatureProfile;
use motion_pipeline::StreamInput;
use motion_pipeline::fit_stage::FitStage;
use pipeline_snapshot::build_moves;
use pipeline_snapshot::waypoints::parse_gcode;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("gcode path");
    let max_velocity: f64 = args.next().expect("max_velocity").parse().unwrap();
    let max_accel: f64 = args.next().expect("max_accel").parse().unwrap();
    let scv: f64 = args.next().expect("scv").parse().unwrap();
    let max_jerk: f64 = args.next().expect("max_jerk").parse().unwrap();
    let line: u32 = args.next().expect("line").parse().unwrap();

    let text = std::fs::read_to_string(&path).expect("read gcode");
    let waypoints = parse_gcode(&text, max_velocity).expect("parse gcode");
    let corner_deviation = geometry::corner_deviation_from_scv(scv, max_accel);
    let limits =
        geometry::VelocityLimits::try_new(max_velocity, max_accel, corner_deviation, max_jerk)
            .expect("limits");
    let moves = build_moves(&waypoints, limits).expect("moves");

    let (fitted_tx, fitted_rx) = unbounded();
    let mut fit = FitStage::new(geometry::CornerFitConfig::default()).into_driver(fitted_tx);
    for m in moves {
        assert!(fit.feed(m.into()));
    }
    assert!(fit.finish());

    let mut i = 0usize;
    while let Ok(item) = fitted_rx.try_recv() {
        let m = match item {
            StreamInput::Move(m) => m,
            _ => continue,
        };
        i += 1;
        if m.source.start_line < line.saturating_sub(1) || m.source.start_line > line + 1 {
            continue;
        }
        let (kind, len, k0, k1, sigma) = match &m.segment.spatial {
            Some(seg) => {
                let (k0, k1) = seg.kappa_endpoints();
                let kind = match seg {
                    geometry::path::Segment::Line(_) => "line",
                    geometry::path::Segment::Arc(_) => "arc",
                    geometry::path::Segment::Clothoid(_) => "clothoid",
                };
                (kind, seg.s_len(), k0, k1, seg.dkappa_ds(0.0))
            }
            None => (
                "virtual",
                m.segment.virtual_path_mm.unwrap_or(0.0),
                0.0,
                0.0,
                0.0,
            ),
        };
        println!(
            "fitted {i} line {} {kind} len={:.17e} k0={:.17e} k1={:.17e} sigma={:.17e} \
             feedrate={:.17e} max_v={:.17e} accel={:.17e} jerk={:.17e}",
            m.source.start_line,
            len,
            k0,
            k1,
            sigma,
            m.feedrate_mm_s,
            m.limits.max_velocity_mm_s,
            m.limits.accel_mm_s2,
            m.limits.max_jerk_mm_s3,
        );
    }
}
