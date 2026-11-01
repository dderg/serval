//! The live pipeline runs its shaper on a dedicated thread. On a slice of a
//! real print, with drains, barriers and a reset mixed into the stream, it
//! must emit exactly what the synchronous pipeline emits, item for item.

use std::path::Path;

use motion_core::live_config;
use motion_core::seam_test_harness::parse_gcode_to_moves_with;
use motion_pipeline::{
    Control, DispatchCommand, Pipeline, StreamConfig, StreamInput, TrajectoryItem, advance_odometer,
};
use trajectory::AxisChainSet;

const SLICE_MOVES: usize = 420;
const DRAIN_EVERY: usize = 37;
const BARRIER_EVERY: usize = 53;
const RESET_AT: usize = 211;

fn repo_file(relative: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn live_setup() -> (StreamConfig, AxisChainSet, Vec<geometry::Move>) {
    let path = "tools/bench/configs/fast.cfg";
    let doc = config_doc::Document::parse(&repo_file(path), path).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    let planner =
        planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config");
    let config = live_config::stream_config(&planner).expect("stream config");
    let chains = planner.compile_active_chains().expect("chains compile");
    let moves = parse_gcode_to_moves_with(
        &repo_file("tools/sim/gcode/voron_dense_top_layers.gcode"),
        |delta| live_config::move_limits(&planner, delta).expect("move limits"),
    );
    (
        config,
        chains,
        moves.into_iter().take(SLICE_MOVES).collect(),
    )
}

/// The stream the ingress would send: the moves, a drain every
/// `DRAIN_EVERY`, a drained barrier every `BARRIER_EVERY`, and one drained
/// reset to where the toolhead already is.
fn stream(moves: &[geometry::Move]) -> Vec<StreamInput> {
    let mut position = vec![0.0; 4];
    let mut items = Vec::new();
    for (index, m) in moves.iter().enumerate() {
        advance_odometer(&mut position, m);
        items.push(StreamInput::Move(m.clone()));
        let fed = index + 1;
        if fed % DRAIN_EVERY == 0 {
            items.push(StreamInput::Drain);
        }
        if fed % BARRIER_EVERY == 0 {
            let (reply, _) = crossbeam_channel::bounded(1);
            items.push(StreamInput::Drain);
            items.push(StreamInput::Control(Control::Dispatch(
                DispatchCommand::Barrier(reply),
            )));
        }
        if fed == RESET_AT {
            items.push(StreamInput::Drain);
            items.push(StreamInput::Control(Control::Reset {
                pos: position.clone(),
            }));
        }
    }
    items
}

fn describe(item: &TrajectoryItem) -> String {
    match item {
        TrajectoryItem::Seg(segment) => format!("Seg {segment:?}"),
        TrajectoryItem::Parked => "Parked".to_string(),
        TrajectoryItem::Control(Control::Dispatch(DispatchCommand::Barrier(_))) => {
            "Barrier".to_string()
        }
        TrajectoryItem::Control(control) => format!("Control {control:?}"),
    }
}

#[test]
fn the_staged_pipeline_emits_what_the_synchronous_one_does_on_a_real_print() {
    let (config, chains, moves) = live_setup();
    assert_eq!(moves.len(), SLICE_MOVES);

    let mut synchronous = Pipeline::new(config, chains.clone(), vec![0.0; 4], 0.0);
    let mut expected = Vec::new();
    let mut collect = |item: TrajectoryItem| {
        expected.push(describe(&item));
        true
    };
    for item in stream(&moves) {
        assert!(synchronous.feed(item, &mut collect));
    }
    assert!(synchronous.finish(&mut collect));

    let (emitted_tx, emitted) = crossbeam_channel::unbounded();
    let mut staged = Pipeline::new(config, chains, vec![0.0; 4], 0.0)
        .into_staged(move |item| emitted_tx.send(describe(&item)).is_ok());
    for item in stream(&moves) {
        assert!(staged.feed(item));
    }
    assert!(staged.finish());
    let emitted: Vec<String> = emitted.try_iter().collect();

    let count = |prefix: &str| {
        expected
            .iter()
            .filter(|item| item.starts_with(prefix))
            .count()
    };
    assert!(count("Seg") > SLICE_MOVES / 10);
    assert_eq!(count("Barrier"), SLICE_MOVES / BARRIER_EVERY);
    assert_eq!(count("Control Reset"), 1);
    assert!(count("Parked") >= SLICE_MOVES / DRAIN_EVERY);
    assert_eq!(emitted.len(), expected.len());
    for (index, (staged, synchronous)) in emitted.iter().zip(&expected).enumerate() {
        assert!(
            staged == synchronous,
            "item {index} differs: staged {staged:.200} vs synchronous {synchronous:.200}"
        );
    }
}
