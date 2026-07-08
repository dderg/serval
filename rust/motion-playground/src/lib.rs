mod config_text;

use serde::Deserialize;
use wasm_bindgen::prelude::*;

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{SnapshotParams, pipeline_snapshot};

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

#[derive(Deserialize)]
struct PlaygroundConfig {
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    max_jerk: f64,
    #[serde(default)]
    arc_fit: Option<u32>,
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

/// Plans the pasted gcode under the given config and returns the snapshot
/// JSON — the same schema the snapshot baselines use, directly consumable by
/// the snapshot-viewer `TrajectoryData`.
#[wasm_bindgen]
pub fn plan(gcode_text: &str, config_json: &str) -> Result<String, JsValue> {
    let cfg: PlaygroundConfig = serde_json::from_str(config_json)
        .map_err(|e| JsValue::from_str(&format!("config: {e}")))?;
    // JSON has no Infinity literal, so 0 encodes "jerk limiting off" — the
    // same convention printer.cfg uses for max_jerk.
    let max_jerk = if cfg.max_jerk == 0.0 {
        f64::INFINITY
    } else {
        cfg.max_jerk
    };
    let waypoints =
        parse_gcode(gcode_text, cfg.max_velocity).map_err(|e| JsValue::from_str(&e.to_string()))?;
    let (axis_decls, post_processor_decls) = config_text::parse(&cfg.post_processor_config)
        .map_err(|e| JsValue::from_str(&e.to_string()))?;
    let snap = pipeline_snapshot(
        &waypoints,
        SnapshotParams {
            max_velocity: cfg.max_velocity,
            max_accel: cfg.max_accel,
            square_corner_velocity: cfg.square_corner_velocity,
            max_jerk,
            arc_fit: cfg.arc_fit,
            max_extrude_only_velocity: cfg.max_extrude_only_velocity,
            max_extrude_only_accel: cfg.max_extrude_only_accel,
            max_path_deviation: cfg.max_path_deviation,
            max_accel_deviation: cfg.max_accel_deviation,
            axis_decls,
            post_processor_decls,
        },
    )
    .map_err(|e| JsValue::from_str(&e.to_string()))?;
    serde_json::to_string(&snap).map_err(|e| JsValue::from_str(&e.to_string()))
}
