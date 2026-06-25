use _motion_engine::seam_test_harness::{
    CommitSchedule, SeamReport, default_stream_config, run_schedule,
};

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

fn report_at_cap(cap: usize) -> SeamReport {
    run_schedule(
        NEPTUNE,
        default_stream_config(),
        &CommitSchedule::fixed_cap(cap),
    )
    .expect("neptune fixture drives the real stream commit path without a planner error")
}

fn assert_continuous(cap: usize) {
    let report = report_at_cap(cap);
    assert_eq!(
        report.fatal(),
        0,
        "commit cap {cap}: {} fatal C0 seam(s); worst {:?}",
        report.fatal(),
        report.worst_fatal()
    );
}

fn assert_clean(cap: usize) {
    let report = report_at_cap(cap);
    assert_eq!(
        report.fatal(),
        0,
        "commit cap {cap}: {} fatal C0 seam(s); worst {:?}",
        report.fatal(),
        report.worst_fatal()
    );
    assert_eq!(
        report.worst(),
        0.0,
        "commit cap {cap}: a clean cadence must record no seam at all, worst was {}",
        report.worst()
    );
}

#[test]
fn seam_continuity_cap_8() {
    assert_continuous(8);
}

#[test]
fn seam_continuity_cap_16() {
    assert_continuous(16);
}

#[test]
fn seam_continuity_cap_24() {
    assert_continuous(24);
}

#[test]
fn seam_continuity_cap_32() {
    assert_clean(32);
}

#[test]
fn seam_continuity_cap_64() {
    assert_clean(64);
}

#[test]
fn seam_continuity_cap_256() {
    assert_clean(256);
}

#[test]
fn forced_commit_then_replan_is_continuous() {
    let schedule = CommitSchedule {
        cadence: _motion_engine::seam_test_harness::Cadence::FixedCap(64),
        force_after_move: vec![40, 120, 200],
    };
    let report = run_schedule(NEPTUNE, default_stream_config(), &schedule)
        .expect("forced commits drive the replan path without a planner error");
    assert_eq!(
        report.fatal(),
        0,
        "forced commit at a non-clean seam then replan: {} fatal C0 seam(s); worst {:?}",
        report.fatal(),
        report.worst_fatal()
    );
}

#[test]
fn report_is_deterministic_across_repeats() {
    let a = report_at_cap(8);
    let b = report_at_cap(8);
    assert_eq!(a, b, "the report must replay byte-identically");
}
