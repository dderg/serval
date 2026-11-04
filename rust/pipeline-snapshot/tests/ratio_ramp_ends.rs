//! A ratio ramp's ends hold the extruder within `[axis e] max_accel`
//! (#676). Where the leaders slow into a corner next to a ratio step, or
//! cross a short travel between two slow corners, the width the leaders'
//! own speed sets falls off faster than the envelope may, and the slow
//! node's cone used to narrow the window below the width the ramp's rule
//! assumed, so its ends stepped the extruder's acceleration past the limit.
//!
//! On the whole dense print that left trident_ramp with six excesses (at
//! t = 35.5383, 45.4608, 61.4338, 65.3078, 69.4011 and 75.9382 s, the worst
//! 5889.2 mm/s² against 5062.7), all at the corner cluster near x 120,
//! y 150 where a short travel joins two extrusions, one layer each. Each
//! layer planned on its own from rest reproduces the mechanism at the
//! layers listed here.

use corpus::extruder::{ExtruderExcess, ToolheadPhase};

mod corpus;

const DENSE_PRINT: &str = "tools/sim/gcode/voron_dense_top_layers.gcode";

/// Layer `index` of `gcode` on its own: from the extruder reset that opens
/// it to the next, in the print's relative extrusion mode.
fn layer(gcode: &str, index: usize) -> String {
    let mut layers: Vec<Vec<&str>> = Vec::new();
    for line in gcode.lines() {
        if line.starts_with("G92 E0") {
            layers.push(Vec::new());
        }
        if let Some(layer) = layers.last_mut() {
            layer.push(line);
        }
    }
    let lines = layers
        .get(index)
        .unwrap_or_else(|| panic!("the print has {} layers, not {index}", layers.len()));
    format!("M83\n{}\n", lines.join("\n"))
}

fn assert_ramp_ends_hold(config: &str, layers: &[usize]) {
    let gcode = std::fs::read_to_string(corpus::repo().join(DENSE_PRINT)).expect("read gcode");
    let config = corpus::repo().join(config);
    let failures: Vec<String> = std::thread::scope(|scope| {
        let planned: Vec<_> = layers
            .iter()
            .map(|&index| {
                let (gcode, config) = (&gcode, &config);
                scope.spawn(move || {
                    let layer = layer(gcode, index);
                    let snapshot =
                        corpus::plan("layer", &layer, config).expect("a layer plans motion");
                    let audit = corpus::extruder::extruder_audit(&snapshot, &layer, config)
                        .expect("the config limits its extruder");
                    audit
                        .excesses
                        .iter()
                        .filter(|excess| {
                            matches!(
                                excess,
                                ExtruderExcess::Acceleration {
                                    phase: ToolheadPhase::Moving,
                                    ..
                                }
                            )
                        })
                        .map(|excess| format!("layer {index}: {excess}"))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        planned
            .into_iter()
            .flat_map(|layer| layer.join().expect("a layer's plan panicked"))
            .collect()
    });
    assert!(
        failures.is_empty(),
        "{}: ratio ramps leave the extruder's acceleration limit:\n{}",
        config.display(),
        failures.join("\n")
    );
}

#[test]
fn ratio_ramps_by_slow_corners_hold_the_trident_extruder_limits() {
    assert_ramp_ends_hold(
        "tools/bench/configs/trident_ramp.cfg",
        &[20, 21, 25, 28, 38],
    );
}

#[test]
fn ratio_ramps_by_slow_corners_hold_the_i3_direct_drive_extruder_limits() {
    assert_ramp_ends_hold(
        "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg",
        &[4, 9, 12, 24, 32],
    );
}

#[test]
fn ratio_ramps_by_slow_corners_hold_the_high_accel_extruder_limits() {
    assert_ramp_ends_hold(
        "tools/bench/configs/profiles/p4_high_accel_ramp.cfg",
        &[20, 21, 29, 31],
    );
}
