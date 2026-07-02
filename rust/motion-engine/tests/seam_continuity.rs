use _motion_engine::seam_test_harness::{SeamReport, default_stream_config, run_schedule};

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

fn report() -> SeamReport {
    run_schedule(NEPTUNE, default_stream_config())
}

/// The pipeline's own emission boundaries (finality barrier, input-empty
/// drains) must leave no seam at all: every boundary is C0 by construction.
#[test]
fn pipeline_output_is_clean() {
    let report = report();
    assert_eq!(
        report.fatal(),
        0,
        "{} fatal C0 seam(s); worst {:?}",
        report.fatal(),
        report.worst_fatal()
    );
    assert_eq!(
        report.worst(),
        0.0,
        "pipeline emission must record no seam at all, worst was {}",
        report.worst()
    );
}

/// The property fuzzer minimized the production seam to this exact window
/// (corpus[163..200), seam at line 177). Pin it as a fast regression so an
/// emission-boundary change that reopens it fails immediately.
#[test]
fn fuzz_minimal_window_is_c0() {
    use _motion_engine::seam_test_harness::{parse_gcode_to_moves, run_moves};
    let corpus = parse_gcode_to_moves(NEPTUNE, default_stream_config().limits);
    let window = &corpus[163..200.min(corpus.len())];
    let report = run_moves(window, default_stream_config());
    assert_eq!(
        report.fatal(),
        0,
        "minimal window must be C0; worst {:?}",
        report.worst_fatal()
    );
}
