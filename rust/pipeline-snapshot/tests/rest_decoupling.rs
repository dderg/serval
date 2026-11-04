//! A full stop decouples planning: the motion after a rest plans the same
//! from rest as it does after everything before it. analyze.py relies on
//! this when it plans every rest-delimited epoch of a bench print as its
//! own stream and holds the capture against that reference.

use std::path::Path;

use pipeline_snapshot::SampleSide;
use pipeline_snapshot::waypoints::Waypoint;

mod corpus;

const EXTRUDER_AXIS: usize = 3;
const PROBES_PER_EPOCH: usize = 400;
/// Two plans of the same toolhead motion read the same position to this,
/// far below a microstep: what differs is the rounding of times measured
/// from a later stream origin.
const TOOLHEAD_POSITION_TOLERANCE_MM: f64 = 1e-6;
/// The extruder's fit may cut its pieces differently over a stream tens of
/// metres of filament deep, within its own fit tolerance; its travel within
/// an epoch still agrees to a twentieth of a bench extruder microstep.
const EXTRUDER_TRAVEL_TOLERANCE_MM: f64 = 1e-4;
const TRAVERSAL_TOLERANCE_S: f64 = 1e-9;

/// The print as analyze.py splits it: the runs of moves between two rests,
/// each planned from rest on the waypoint the previous run ended on.
fn split_at_rests(waypoints: &[Waypoint]) -> (Vec<Vec<Waypoint>>, Vec<f64>) {
    let mut epochs = vec![vec![waypoints[0]]];
    let mut dwells = Vec::new();
    for waypoint in &waypoints[1..] {
        let mut waypoint = *waypoint;
        if let Some(dwell) = waypoint.dwell_before_s.take() {
            dwells.push(dwell);
            let rest = *epochs
                .last()
                .and_then(|epoch| epoch.last())
                .expect("an epoch");
            epochs.push(vec![rest]);
        }
        epochs.last_mut().expect("an epoch").push(waypoint);
    }
    (epochs, dwells)
}

fn position(snapshot: &pipeline_snapshot::Snapshot, axis: usize, t: f64) -> f64 {
    snapshot
        .trajectory
        .eval_axis(axis, t, SampleSide::Right)
        .unwrap_or_else(|e| panic!("axis {axis} at {t}: {e}"))
        .position
}

fn assert_rest_decoupled(name: &str, gcode: &str, config: &Path) {
    let waypoints = corpus::waypoints(gcode, config);
    let (epochs, dwells) = split_at_rests(&waypoints);
    assert!(
        epochs.len() > 2,
        "{name}: the print must rest between epochs"
    );
    let stream = corpus::plan_waypoints(name, &waypoints, config);
    let planned = epochs
        .iter()
        .map(|epoch| corpus::plan_waypoints(name, epoch, config))
        .collect::<Vec<_>>();
    let split_total = planned
        .iter()
        .map(|epoch| epoch.traversal_time_s)
        .sum::<f64>()
        + dwells.iter().sum::<f64>();
    assert!(
        (stream.traversal_time_s - split_total).abs() <= TRAVERSAL_TOLERANCE_S,
        "{name}: the stream takes {} s, its epochs planned apart {split_total} s",
        stream.traversal_time_s
    );
    let mut offset = 0.0;
    for (index, epoch) in planned.iter().enumerate() {
        let duration = epoch.traversal_time_s;
        let stream_e_start = position(&stream, EXTRUDER_AXIS, offset);
        let epoch_e_start = position(epoch, EXTRUDER_AXIS, 0.0);
        for probe in 0..PROBES_PER_EPOCH {
            let local = duration * probe as f64 / PROBES_PER_EPOCH as f64;
            for axis in 0..EXTRUDER_AXIS {
                let difference =
                    (position(&stream, axis, offset + local) - position(epoch, axis, local)).abs();
                assert!(
                    difference <= TOOLHEAD_POSITION_TOLERANCE_MM,
                    "{name}: epoch {index} axis {axis} {local} s in reads {difference} mm \
                     apart planned in the stream and from rest"
                );
            }
            let travel_difference = ((position(&stream, EXTRUDER_AXIS, offset + local)
                - stream_e_start)
                - (position(epoch, EXTRUDER_AXIS, local) - epoch_e_start))
                .abs();
            assert!(
                travel_difference <= EXTRUDER_TRAVEL_TOLERANCE_MM,
                "{name}: epoch {index} extruder travel {local} s in differs by \
                 {travel_difference} mm planned in the stream and from rest"
            );
        }
        offset += duration + dwells.get(index).copied().unwrap_or(0.0);
    }
}

fn rest_ladder() -> String {
    std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/rest_ladder.gcode"))
        .expect("rest ladder gcode")
}

#[test]
fn the_rest_ladder_with_pressure_advance_plans_each_epoch_from_rest_as_in_the_stream() {
    assert_rest_decoupled(
        "rest_ladder",
        &rest_ladder(),
        &corpus::repo().join("tools/bench/configs/rest_ladder.cfg"),
    );
}

#[test]
fn the_bench_epochs_print_plans_each_epoch_from_rest_as_in_the_stream() {
    let gcode = std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/epochs.gcode"))
        .expect("epochs gcode");
    assert_rest_decoupled(
        "epochs",
        &gcode,
        &corpus::repo().join("tools/bench/printer.cfg"),
    );
}

/// The bench printer's advance with no smoothing kernel, under the
/// extrude-only caps it then needs: the ratio ramp and the advance's rest
/// valleys alone decouple the epochs.
#[test]
fn the_bench_epochs_print_with_a_kernel_free_advance_plans_each_epoch_from_rest_as_in_the_stream() {
    let gcode = std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/epochs.gcode"))
        .expect("epochs gcode");
    let config = std::fs::read_to_string(corpus::repo().join("tools/bench/printer.cfg"))
        .expect("bench config")
        .replace("post_processors: pa, st", "post_processors: pa")
        .replace(
            "[virtual_sdcard]",
            "max_extrude_only_velocity: 50\nmax_extrude_only_accel: 5000\n\n[virtual_sdcard]",
        );
    let dir = std::env::temp_dir().join(format!("rest_decoupling_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let path = dir.join("printer_kernel_free.cfg");
    std::fs::write(&path, config).expect("kernel-free config writes");
    assert_rest_decoupled("epochs_kernel_free", &gcode, &path);
}

#[test]
fn the_corner_flow_print_with_ramped_ratio_steps_plans_each_epoch_from_rest_as_in_the_stream() {
    let gcode = std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/corner_flow.gcode"))
        .expect("corner flow gcode");
    assert_rest_decoupled(
        "corner_flow",
        &format!("SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG92 E0\n{gcode}"),
        &corpus::repo().join("tools/bench/configs/ramp.cfg"),
    );
}
