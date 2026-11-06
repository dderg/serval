//! A ratio step just past a rest holds the extruder within its limits.
//!
//! Lines 880–930 of `trident_follower_cache_regression` on the kernel-free
//! fast bench config: a retract, a travel and a prime, then a 2 µm
//! extrusion at half the ratio of the perimeter it opens. The shaped
//! leaders cross that ratio step 0.19 mm past where they rest, at 58 mm/s,
//! so the rest narrows its ramp. The first width node past the rest lay
//! 14.8 mm on, at a slow corner whose target was far under the rest's cone
//! there, and the width rose off the rest toward it at 0.21 per mm instead
//! of the cone's 0.25 the ramp's budgets read: the extruder reached
//! 24800 mm/s² against its 20000 mm/s² `max_accel`.

mod corpus;

const PRIME_INTO_EXTRUSION: corpus::PrintSlice = corpus::PrintSlice {
    print: "trident_follower_cache_regression",
    lines: 880..=930,
    idle_s: 0.0,
};

#[test]
fn a_ratio_step_just_past_a_prime_holds_the_extruder_limits() {
    let config = corpus::repo().join("tools/bench/configs/fast.cfg");
    let print = corpus::sim_print(PRIME_INTO_EXTRUSION.print);
    let gcode = corpus::print_slice(&print, &PRIME_INTO_EXTRUSION);
    let snapshot = corpus::plan("prime into extrusion", &gcode, &config).expect("the slice plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("the fast bench limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{excesses:#?}");
}
