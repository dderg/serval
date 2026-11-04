//! A pressure advance with no kernel after it reads the shaped leaders'
//! acceleration as extruder velocity, `k·r·a`. The shaped signal's
//! acceleration is continuous; a leader fit that steps it at a knot, still
//! within `max_accel_deviation`, would hand the extruder a velocity step of
//! `k·r` times that step, an unbounded acceleration.

use corpus::extruder::ExtruderExcess;

mod corpus;

/// The opening perimeters of the dense print, where the P1 bowden
/// bedslinger's k = 0.5 advance stepped the extruder by up to 0.5 mm/s at
/// the x and y fits' knots.
const DENSE_PRINT_HEAD_LINES: usize = 60;

#[test]
fn a_kernel_free_advance_over_shaped_leaders_never_steps_the_extruder_velocity() {
    let config = corpus::repo().join("tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg");
    let print = std::fs::read_to_string(
        corpus::repo().join("tools/sim/gcode/voron_dense_top_layers.gcode"),
    )
    .expect("the dense print reads");
    let gcode = print
        .lines()
        .take(DENSE_PRINT_HEAD_LINES)
        .collect::<Vec<_>>()
        .join("\n");
    let snapshot = corpus::plan("dense_print_head", &gcode, &config).expect("the head plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("the profile limits its extruder");
    let steps: Vec<String> = audit
        .excesses
        .iter()
        .filter(|excess| matches!(excess, ExtruderExcess::VelocityStep { .. }))
        .map(ToString::to_string)
        .collect();
    assert!(
        steps.is_empty(),
        "{} extruder velocity steps, the first {:?}",
        steps.len(),
        &steps[..steps.len().min(5)]
    );
}
