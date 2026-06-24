use _motion_engine::seam_harness::{
    Cadence, CommitSchedule, SeamReport, default_stream_config, run_schedule,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

const CORPUS: &[(&str, &str)] = &[(
    "crash_short_cube",
    include_str!("gcode/crash_short_cube.gcode"),
)];

fn run(name_idx: usize, schedule: &CommitSchedule) -> SeamReport {
    let (_, gcode) = CORPUS[name_idx % CORPUS.len()];
    run_schedule(gcode, default_stream_config(), schedule)
        .expect("corpus gcode drives the real stream commit path without a planner error")
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 48,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/seam_schedule_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn seam_continuity_under_fuzzed_schedule(
        corpus_idx in 0usize..CORPUS.len(),
        cap in 8usize..28,
        forced in proptest::collection::vec(0usize..456, 0..3),
    ) {
        let schedule = CommitSchedule {
            cadence: Cadence::FixedCap(cap),
            force_after_move: forced.clone(),
        };
        let report = run(corpus_idx, &schedule);
        prop_assert_eq!(
            report.fatal(),
            0,
            "corpus[{}] cap={} forced={:?}: fatal C0 seam; worst {:?}",
            corpus_idx,
            cap,
            forced,
            report.worst_fatal()
        );
    }
}
