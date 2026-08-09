mod config_text;
pub mod plan_core;

use wasm_bindgen::prelude::*;

#[cfg(test)]
mod tests;

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Plans the pasted gcode under the given config and returns the snapshot
/// JSON — the same schema the snapshot baselines use, directly consumable by
/// the snapshot-viewer `TrajectoryData`.
#[wasm_bindgen]
pub fn plan(gcode_text: &str, config_json: &str) -> Result<String, JsValue> {
    plan_core::plan_json(gcode_text, config_json).map_err(|e| JsValue::from_str(&e))
}

/// Like [`plan`] (byte-identical final JSON), but invokes `on_partial` with
/// the JSON string of a schema-complete partial snapshot — the trajectory
/// pieces produced so far — every [`plan_core::PARTIAL_BATCH_SEGMENTS`]
/// shaped segments, so the UI can draw the trajectory as it grows.
#[wasm_bindgen]
pub fn plan_streaming(
    gcode_text: &str,
    config_json: &str,
    on_partial: &js_sys::Function,
) -> Result<String, JsValue> {
    let mut callback_error: Option<JsValue> = None;
    let json = plan_core::plan_json_streaming(gcode_text, config_json, |partial_json| {
        if callback_error.is_none() {
            if let Err(e) = on_partial.call1(&JsValue::NULL, &JsValue::from_str(partial_json)) {
                callback_error = Some(e);
            }
        }
    })
    .map_err(|e| JsValue::from_str(&e))?;
    match callback_error {
        Some(e) => Err(e),
        None => Ok(json),
    }
}
