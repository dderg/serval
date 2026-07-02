use std::sync::OnceLock;

use _motion_engine::seam_test_harness::{
    Move, default_stream_config, parse_gcode_to_moves, run_moves,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

const MIN_WINDOW: usize = 4;
const MAX_WINDOW: usize = 40;

fn corpus() -> &'static [Move] {
    static CORPUS: OnceLock<Vec<Move>> = OnceLock::new();
    CORPUS.get_or_init(|| parse_gcode_to_moves(NEPTUNE, default_stream_config().limits))
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
    fn seam_continuity_over_fuzzed_windows(
        start in 0usize..1024,
        len in MIN_WINDOW..=MAX_WINDOW,
    ) {
        let corpus = corpus();
        let n = corpus.len();
        prop_assume!(n > MIN_WINDOW);
        let start = start % (n - MIN_WINDOW);
        let end = (start + len).min(n);
        let window = &corpus[start..end];
        prop_assume!(window.len() >= MIN_WINDOW);

        let report = run_moves(window, default_stream_config());
        prop_assert_eq!(
            report.fatal(),
            0,
            "window [{}..{}): fatal C0 seam; worst {:?}",
            start,
            end,
            report.worst_fatal()
        );
    }
}
