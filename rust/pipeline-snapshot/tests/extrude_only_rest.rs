//! An extrude-only move is the extruder's alone: the toolhead rests through
//! it, so the extrude-only caps it is planned under are all the extruder
//! runs at. The leaders' shaped path keeps moving for a kernel's reach past
//! a raw stop and before a raw start, and the extruder follows that motion
//! at the extrusion ratio it reads there; a retract or prime overlapping it
//! would carry both.

mod corpus;

/// A retract after an extrusion, a 70 000 mm/s² travel and a prime before
/// the next extrusion, on the Trident whose advance rides a ratio ramp:
/// before, the prime's start carried the travel's braking spill to
/// 5913 mm/s² of extruder acceleration against the 5000 mm/s² cap.
const RETRACT_TRAVEL_PRIME: &str = "M83
G92 E0
G1 X100 Y100 Z0.2 F60000
G1 X110 Y100 E0.4 F25953.656
G1 X110.3 Y100.2 E0.0123
G1 E-.3 F3600
G1 X110.4 Y98.8 F60000
G1 E.3 F2400
G1 X120 Y98.8 E0.4 F25953.656
";

const EXTRUDE_ONLY_ACCEL: f64 = 5000.0;
const ACCEL_DEVIATION: f64 = 50.0;
const PROBES_PER_REST: usize = 4000;

#[test]
fn the_extruder_never_adds_the_leaders_spill_to_an_extrude_only_move() {
    let config = corpus::repo().join("tools/bench/configs/trident_ramp.cfg");
    let snapshot = corpus::plan("retract_travel_prime", RETRACT_TRAVEL_PRIME, &config)
        .expect("the repro plans");
    let trajectory = &snapshot.trajectory;
    let rests: Vec<(f64, f64)> = trajectory
        .spans
        .windows(2)
        .map(|pair| (pair[0].t_end, pair[1].t_start))
        .filter(|(stop, start)| start > stop)
        .collect();
    assert_eq!(
        rests.len(),
        2,
        "the toolhead rests through the retract and the prime: {rests:?}"
    );
    for (stop, start) in rests {
        for k in 0..=PROBES_PER_REST {
            let t = stop + (start - stop) * k as f64 / PROBES_PER_REST as f64;
            let accel = trajectory
                .eval_axis(
                    pipeline_snapshot::EXTRUDER_AXIS,
                    t,
                    pipeline_snapshot::SampleSide::Right,
                )
                .expect("extruder carrier")
                .acceleration;
            assert!(
                accel.abs() <= EXTRUDE_ONLY_ACCEL + ACCEL_DEVIATION,
                "at t={t}, in the rest [{stop}, {start}], the extruder accelerates at \
                 {accel} mm/s² against the {EXTRUDE_ONLY_ACCEL} mm/s² extrude-only cap"
            );
        }
    }
}
