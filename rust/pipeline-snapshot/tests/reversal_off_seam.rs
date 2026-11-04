//! A path reversal between a move that brakes and one that launches unlike
//! it: shaping spreads the raw stop over the kernels' support, so the shaped
//! toolhead turns around off the raw seam, inside one of the moves. The
//! leaders' path speed still passes through zero there with its rate
//! stepping from `−a` to `+a`, and a ratio-ramped advance reading it must
//! release its lead over that point as at a seam's turnaround.

mod corpus;

/// The head of `tools/sim/gcode/voron_first_layers.gcode` that the P3
/// Voron's ratio-ramped advance refused at t≈126.8 s with a velocity step
/// of −6.1 → +6.1 mm/s: a 1.7 mm extrusion at 99 mm/s, then a 150 mm/s
/// travel back along it. The shaped turnaround lands about a millisecond
/// before the raw seam.
const EXTRUSION_THEN_TRAVEL_BACK: &str = "G90
M83
G1 X98.898 Y120.133 F9000
G1 F5932.264
G1 X98.898 Y121.801 E.05612
G1 X98.898 Y109.06 F9000
";

const SAMPLE_DT_S: f64 = 1e-5;
/// A velocity step of `k·r·a` moves the extruder by a millimetre per second
/// or more within one sample, 10⁵ mm/s²; motion with no step stays under
/// half that.
const STEP_FREE_ACCELERATION: f64 = 50_000.0;

#[test]
fn a_ratio_ramped_advance_stays_velocity_continuous_through_a_turnaround_off_the_seam() {
    let config = corpus::repo().join("tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg");
    let snapshot = corpus::plan(
        "extrusion_then_travel_back",
        EXTRUSION_THEN_TRAVEL_BACK,
        &config,
    )
    .expect("the repro plans");
    let trajectory = &snapshot.trajectory;
    let rows = trajectory.rows(pipeline_snapshot::EXTRUDER_AXIS);
    let (t_start, t_end) = (
        rows.first().expect("extruder rows").t0,
        rows.last().expect("extruder rows").t1,
    );
    let samples = ((t_end - t_start) / SAMPLE_DT_S) as usize - 1;
    let velocity = |t: f64| {
        trajectory
            .eval_axis(
                pipeline_snapshot::EXTRUDER_AXIS,
                t,
                pipeline_snapshot::SampleSide::Right,
            )
            .expect("extruder carrier")
            .velocity
    };
    let (largest, at) = (0..samples)
        .map(|index| t_start + index as f64 * SAMPLE_DT_S)
        .map(|t| ((velocity(t + SAMPLE_DT_S) - velocity(t)).abs(), t))
        .fold((0.0, t_start), |best, candidate| {
            if candidate.0 > best.0 {
                candidate
            } else {
                best
            }
        });
    assert!(
        largest / SAMPLE_DT_S < STEP_FREE_ACCELERATION,
        "the extruder velocity moves {largest} mm/s within one {SAMPLE_DT_S} s sample at t={at}"
    );
}
