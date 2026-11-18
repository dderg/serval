mod corpus;

/// #946, from a stock Benchy on the shipped `p4_high_accel_ramp`: a prime,
/// two short extrusions, then a travel straight back over them. The rest
/// valley of the prime closes only on the travel, whose opening seam is a
/// turnaround: reading the path speed through it, the advance stepped the
/// follower's velocity by `2·k·r·a`, from -1.76 to +1.76 mm/s.
const TURNAROUND_CLOSING_A_PRIME: &str = "G90
M83
SET_VELOCITY_LIMIT ACCEL=15000
G1 X145 Y151.1 F42000
G1 X150.865 Y151.419 E.2
G1 X155.468 Y151.692
G1 E.5 F3600
G1 X155.502 Y151.467 E.00947
G1 X155.535 Y151.242 E.01002
G1 X155.468 Y151.692 F42000
";

#[test]
fn a_turnaround_closing_a_primes_rest_keeps_the_extruder_within_its_limits() {
    let config = corpus::repo().join("tools/bench/configs/profiles/p4_high_accel_ramp.cfg");
    let snapshot = corpus::plan("turnaround past a prime", TURNAROUND_CLOSING_A_PRIME, &config)
        .expect("the gcode moves");
    let audit = corpus::extruder::extruder_audit(&snapshot, TURNAROUND_CLOSING_A_PRIME, &config)
        .expect("the config limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{}: {excesses:#?}", audit.summary());
}
