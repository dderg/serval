use serde::Deserialize;

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{SnapshotParams, pipeline_snapshot, pipeline_snapshot_streaming};

use crate::config_text;

pub const PARTIAL_BATCH_SEGMENTS: usize = 64;

#[derive(Deserialize)]
struct PlaygroundConfig {
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    max_jerk: f64,
    #[serde(default)]
    max_extrude_only_velocity: Option<f64>,
    #[serde(default)]
    max_extrude_only_accel: Option<f64>,
    #[serde(default)]
    max_path_deviation: Option<f64>,
    #[serde(default)]
    max_accel_deviation: Option<f64>,
    #[serde(default)]
    post_processor_config: String,
}

type Waypoint = (f64, f64, f64, f64, f64);

fn parse_inputs(
    gcode_text: &str,
    config_json: &str,
) -> Result<(Vec<Waypoint>, SnapshotParams), String> {
    let cfg: PlaygroundConfig =
        serde_json::from_str(config_json).map_err(|e| format!("config: {e}"))?;
    // JSON has no Infinity literal, so 0 encodes "jerk limiting off" — the
    // same convention printer.cfg uses for max_jerk.
    let max_jerk = if cfg.max_jerk == 0.0 {
        f64::INFINITY
    } else {
        cfg.max_jerk
    };
    let waypoints = parse_gcode(gcode_text, cfg.max_velocity).map_err(|e| e.to_string())?;
    let (axis_decls, post_processor_decls) =
        config_text::parse(&cfg.post_processor_config).map_err(|e| e.to_string())?;
    Ok((
        waypoints,
        SnapshotParams {
            max_velocity: cfg.max_velocity,
            max_accel: cfg.max_accel,
            square_corner_velocity: cfg.square_corner_velocity,
            max_jerk,
            max_extrude_only_velocity: cfg.max_extrude_only_velocity,
            max_extrude_only_accel: cfg.max_extrude_only_accel,
            max_path_deviation: cfg.max_path_deviation,
            max_accel_deviation: cfg.max_accel_deviation,
            axis_decls,
            post_processor_decls,
        },
    ))
}

pub fn plan_json(gcode_text: &str, config_json: &str) -> Result<String, String> {
    let (waypoints, params) = parse_inputs(gcode_text, config_json)?;
    let snap = pipeline_snapshot(&waypoints, params).map_err(|e| e.to_string())?;
    serde_json::to_string(&snap).map_err(|e| e.to_string())
}

/// Same result as [`plan_json`] (byte-identical final JSON), but invokes
/// `on_partial` with schema-complete prefix snapshots as trajectory pieces
/// accumulate, one call per [`PARTIAL_BATCH_SEGMENTS`] shaped segments.
pub fn plan_json_streaming(
    gcode_text: &str,
    config_json: &str,
    mut on_partial: impl FnMut(&str),
) -> Result<String, String> {
    let (waypoints, params) = parse_inputs(gcode_text, config_json)?;
    let snap = pipeline_snapshot_streaming(&waypoints, params, PARTIAL_BATCH_SEGMENTS, |partial| {
        let json = serde_json::to_string(partial)
            .expect("partial snapshot must serialize like the full one");
        on_partial(&json);
    })
    .map_err(|e| e.to_string())?;
    serde_json::to_string(&snap).map_err(|e| e.to_string())
}
