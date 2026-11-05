//! A retract and a prime planned at `max_extrude_only_velocity`, which the
//! P4 profile also lends `[axis e]` as its `max_velocity`, cruise at it
//! through the extruder chain's smoothing kernel: the fit of the smoothed
//! extruder holds the cruise and never leaves the cap.

mod corpus;

const P4_KERNEL: &str = "tools/bench/configs/profiles/p4_high_accel_kernel.cfg";

const RETRACT_AT_THE_CAP: &str = "G21
G90
M83
G1 X0 Y0 F3000
G1 X30.00000 Y0.00000 E0.997804
G1 E-0.800000
G1 X30.00000 Y20.00000
G1 E0.800000
G1 X60.00000 Y20.00000 E0.997804
";

#[test]
fn a_retract_and_prime_cruising_at_the_extruder_cap_stay_within_it() {
    let config = corpus::repo().join(P4_KERNEL);
    let snapshot = corpus::plan("retract at the cap", RETRACT_AT_THE_CAP, &config)
        .expect("the retract case plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, RETRACT_AT_THE_CAP, &config)
        .expect("the P4 profile limits its extruder");
    assert!(
        audit.peak_velocity >= 50.0 * (1.0 - 1e-9),
        "the retract must reach the 50 mm/s cap, yet e peaks at {} mm/s",
        audit.peak_velocity
    );
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}
