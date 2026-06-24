use std::sync::OnceLock;

use _motion_engine::seam_harness::{
    Cadence, CommitSchedule, Move, default_stream_config, parse_gcode_to_moves, run_moves,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

const CUBE: &str = include_str!("gcode/crash_short_cube.gcode");

const MIN_WINDOW: usize = 4;
const MAX_WINDOW: usize = 40;

fn corpus() -> &'static [Move] {
    static CORPUS: OnceLock<Vec<Move>> = OnceLock::new();
    CORPUS.get_or_init(|| parse_gcode_to_moves(CUBE, default_stream_config().limits))
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 128,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/seam_schedule_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn seam_continuity_under_windowed_schedule(
        start in 0usize..1024,
        len in MIN_WINDOW..=MAX_WINDOW,
        cap in 1usize..24,
        forced in proptest::collection::vec(0usize..MAX_WINDOW, 0..2),
    ) {
        let corpus = corpus();
        let n = corpus.len();
        prop_assume!(n > MIN_WINDOW);
        let start = start % (n - MIN_WINDOW);
        let end = (start + len).min(n);
        let window = &corpus[start..end];
        prop_assume!(window.len() >= MIN_WINDOW);

        let schedule = CommitSchedule {
            cadence: Cadence::FixedCap(cap),
            force_after_move: forced.clone(),
        };
        let report = run_moves(window, default_stream_config(), &schedule)
            .expect("window drives the real stream commit path without a planner error");
        prop_assert_eq!(
            report.fatal(),
            0,
            "window [{}..{}) cap={} forced={:?}: fatal C0 seam; worst {:?}",
            start,
            end,
            cap,
            forced,
            report.worst_fatal()
        );
    }
}
