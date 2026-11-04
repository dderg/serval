//! A ratio-ramped advance keeps its whole lead into and out of a rest where
//! the full advance fits the extruder's limits: a single line from rest to
//! rest on the kernel-free bench configs, whose full advance asks a fifth
//! of the extruder's acceleration, used to fade the lead over the last and
//! first few millimetres (half of `k·r·v` a millimetre before the stop), so
//! the nozzle still held the advance's pressure when the toolhead stopped.
//! A diagonal line, which the axes' unlike shapers curve into its stop,
//! lays the tail's material as a linear fall across the spill; read at the
//! speed the leaders cruise well past the spill, that slope seam passed for
//! a step the width field narrows past the extruder's limits, and the lead
//! was released from 30 ms out.

mod corpus;

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{EXTRUDER_AXIS, SampleSide, Snapshot, pipeline_snapshot_for_config};
use planner_config::from_doc::planner_config_from_text;

const ADVANCE_K_S: f64 = 0.03;
const EXTRUDER_MAX_ACCEL_MM_S2: f64 = 20000.0;
const LINE_MM: f64 = 30.0;
const SPEED_MM_S: f64 = 300.0;
const SAMPLES: usize = 20000;
const RATIO: f64 = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
const MARKS_MM: [f64; 5] = [3.0, 2.0, 1.0, 0.5, 0.25];
const KEPT_LEAD: f64 = 0.9;
const HEADINGS_DEG: [f64; 2] = [0.0, 45.0];

/// Whether the shaped axes leave a rest along the line: a curved start
/// lays the lead's material as a valley inside the spill, which a leaving
/// side still reads at the fastest the leaders cross it.
// TODO: hold curved starts to the kept lead too once the width field's
// windows across a leaving spill follow its envelope.
fn starts_straight(heading_deg: f64) -> bool {
    heading_deg % 90.0 == 0.0
}

fn plan(config_text: &str, heading: (f64, f64)) -> Snapshot {
    let planner = planner_config_from_text(config_text).expect("the config reads");
    let program = format!(
        "G21\nG90\nM83\nG1 X0 Y0 F{:.0}\nG1 X{:.9} Y{:.9} E{:.9}\n",
        SPEED_MM_S * 60.0,
        LINE_MM * heading.0,
        LINE_MM * heading.1,
        RATIO * LINE_MM
    );
    let waypoints = parse_gcode(&program, &planner.cartesian).expect("the program parses");
    pipeline_snapshot_for_config(&waypoints, &planner, None).expect("the line plans")
}

/// The lead `e − r·s` as a fraction of the full advance's `k·r·v` where
/// the toolhead first passes each mark along the line, and the most the
/// full advance asks of the extruder, `r·|a + k·j|`.
struct LeadReading {
    after_start: Vec<(f64, f64)>,
    before_stop: Vec<(f64, f64)>,
    full_advance_accel: f64,
}

fn read_lead(snapshot: &Snapshot, heading: (f64, f64)) -> LeadReading {
    let trajectory = &snapshot.trajectory;
    let mut after_start: Vec<f64> = MARKS_MM.iter().rev().copied().collect();
    let mut before_stop: Vec<f64> = MARKS_MM.to_vec();
    let mut reading = LeadReading {
        after_start: Vec::new(),
        before_stop: Vec::new(),
        full_advance_accel: 0.0,
    };
    for i in 0..=SAMPLES {
        let t = trajectory.t_end() * i as f64 / SAMPLES as f64;
        let x = trajectory.eval_axis(0, t, SampleSide::Right).expect("x");
        let y = trajectory.eval_axis(1, t, SampleSide::Right).expect("y");
        let e = trajectory
            .eval_axis(EXTRUDER_AXIS, t, SampleSide::Right)
            .expect("e");
        let along = |of_x: f64, of_y: f64| of_x * heading.0 + of_y * heading.1;
        let s = along(x.position, y.position);
        let v = along(x.velocity, y.velocity);
        reading.full_advance_accel = reading.full_advance_accel.max(
            (RATIO
                * along(
                    x.acceleration + ADVANCE_K_S * x.jerk,
                    y.acceleration + ADVANCE_K_S * y.jerk,
                ))
            .abs(),
        );
        let kept = (e.position - RATIO * s) / (ADVANCE_K_S * RATIO * v);
        if after_start.first().is_some_and(|&mark| s >= mark) {
            reading.after_start.push((after_start.remove(0), kept));
        }
        if before_stop
            .first()
            .is_some_and(|&mark| s >= LINE_MM - mark && v > 0.0)
        {
            reading.before_stop.push((before_stop.remove(0), kept));
        }
    }
    reading
}

#[test]
fn a_kernel_free_advance_keeps_its_lead_into_and_out_of_a_rest_it_can_afford() {
    for bench_config in ["tools/bench/configs/fast.cfg", "tools/bench/printer.cfg"] {
        let config = corpus::kernel_free(bench_config, EXTRUDER_MAX_ACCEL_MM_S2);
        for heading_deg in HEADINGS_DEG {
            let heading = (
                heading_deg.to_radians().cos(),
                heading_deg.to_radians().sin(),
            );
            let reading = read_lead(&plan(&config, heading), heading);
            let line = format!("{bench_config}, heading {heading_deg}°");
            assert!(
                reading.full_advance_accel < EXTRUDER_MAX_ACCEL_MM_S2,
                "{line}: precondition: the full advance asks {} mm/s² of the extruder",
                reading.full_advance_accel
            );
            assert_eq!(reading.after_start.len(), MARKS_MM.len(), "{line}");
            assert_eq!(reading.before_stop.len(), MARKS_MM.len(), "{line}");
            let held_after_start: &[(f64, f64)] = if starts_straight(heading_deg) {
                &reading.after_start
            } else {
                &[]
            };
            assert!(
                held_after_start
                    .iter()
                    .chain(&reading.before_stop)
                    .all(|&(_, kept)| kept >= KEPT_LEAD),
                "{line}: the full advance asks at most {:.0} mm/s² (A_e \
                 {EXTRUDER_MAX_ACCEL_MM_S2}), yet it keeps (mm from the rest, fraction of k·r·v) \
                 {:.3?} after the start and {:.3?} before the stop",
                reading.full_advance_accel,
                reading.after_start,
                reading.before_stop
            );
        }
    }
}
