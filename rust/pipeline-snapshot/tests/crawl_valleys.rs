//! A near-reversal between a travel and the extrusion back along it: the
//! shaped toolhead turns at a crawl of under a millimetre per second without
//! passing through zero, and the path speed's second derivative carries the
//! tangent's turning `a⊥²/v` there. A ratio-ramped advance reading the path
//! speed through it asked the extruder for several times its acceleration
//! limit; it now releases its lead around the crawl.

mod corpus;

use corpus::extruder::{ExtruderExcess, ToolheadPhase};

/// `tools/sim/gcode/trident_follower_cache_regression.gcode` lines 868-873,
/// whose crawl at t≈20.896 s on the P2 i3 ratio-ramp config asked for
/// 26283 mm/s² against an 8517 mm/s² bound.
const TRAVEL_THEN_EXTRUSION_BACK: &str = "G90
M83
G1 X188.365 Y137.075 F60000
G1 F24766.062
G1 X186.617 Y137.075 E.06163
G1 X186.365 Y140.824 F60000
G1 F24766.062
G1 X186.365 Y139.075 E.06163
G1 X179.522 Y145.918 E.34113
";

#[test]
fn a_ratio_ramped_advance_keeps_the_extruder_within_its_limits_through_a_crawl() {
    let config = corpus::repo().join("tools/bench/configs/p2_i3_direct_drive_ramp_cd0.01.cfg");
    let snapshot = corpus::plan(
        "travel_then_extrusion_back",
        TRAVEL_THEN_EXTRUSION_BACK,
        &config,
    )
    .expect("the repro plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, TRAVEL_THEN_EXTRUSION_BACK, &config)
        .expect("the config limits the extruder");
    let moving: Vec<&ExtruderExcess> = audit
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
        .collect();
    assert!(moving.is_empty(), "{moving:#?}");
}

/// `tools/sim/gcode/voron_first_layers.gcode` lines 897-900: an extrusion,
/// the travel back along it and the extrusion out again, whose crawl at
/// t≈63.92 s on the P1 bowden kernel profile drove the extruder to
/// 49.4 mm/s against its 45 mm/s limit as the advance rebuilt the lead it
/// released there.
const EXTRUSION_TRAVEL_BACK_AND_OUT: &str = "G90
M83
G1 X114.495 Y98.898 F3000
G1 X109.03 Y104.363 E.26235
G1 X114.088 Y98.711 F9000
G1 F5878.936
G1 X108.31 Y104.489 E.27739
G1 X107.16 Y105.195 E.04579
";

#[test]
fn a_kernel_smoothed_advance_keeps_the_extruder_within_its_velocity_limit_out_of_a_crawl() {
    let config = corpus::repo().join("tools/bench/configs/profiles/p1_bowden_bedslinger_kernel.cfg");
    let snapshot = corpus::plan(
        "extrusion_travel_back_and_out",
        EXTRUSION_TRAVEL_BACK_AND_OUT,
        &config,
    )
    .expect("the repro plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, EXTRUSION_TRAVEL_BACK_AND_OUT, &config)
        .expect("the config limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{}: {excesses:#?}", audit.summary());
}
