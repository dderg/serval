use super::*;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityConfig, VelocityLimits, line_move};
use nurbs::eval::eval;

fn cfg(keep_secs: f64) -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig::default(),
        fit_tol_mm: 1e-3,
        keep_secs,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
    }
}

fn ctx(line_no: u32, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> geometry::Move {
    line_move(start, end, e, ctx(line_no, 80.0)).unwrap()
}

#[test]
fn collinear_jogs_commit_at_the_seam_without_stopping() {
    let mut s = StreamState::new(cfg(0.0), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0));

    let committed = s.commit(false).unwrap();
    assert!(!committed.is_empty());
    assert_eq!(s.buffered(), 1);
    assert!(
        s.entry_velocity() > 1.0,
        "seam velocity {} should be cruising, not stopped",
        s.entry_velocity()
    );
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn flush_commits_everything_to_rest() {
    let mut s = StreamState::new(cfg(1.0), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0));

    assert!(s.commit(false).unwrap().is_empty());

    let committed = s.commit(true).unwrap();
    assert!(!committed.is_empty());
    assert!(s.is_empty());
    assert_eq!(s.entry_velocity(), 0.0);
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 100.0).abs() < 1e-6);
}

#[test]
fn blended_corner_is_never_split_by_a_commit() {
    let mut s = StreamState::new(cfg(0.0), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0));

    assert!(
        s.commit(false).unwrap().is_empty(),
        "must not commit across a blended corner"
    );
    assert_eq!(s.buffered(), 2);

    assert!(!s.commit(true).unwrap().is_empty());
    assert!(s.is_empty());
}

#[test]
fn odometer_accumulates_extrusion_across_commits() {
    let mut s = StreamState::new(
        cfg(1.0),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
    );
    s.push(line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0));
    s.push(line(2, [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], 4.0));

    let committed = s.commit(true).unwrap();
    assert!(s.is_empty());
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 8.0).abs() < 1e-3);
}

#[test]
fn committed_trajectory_is_time_contiguous() {
    let mut s = StreamState::new(cfg(1.0), AxisChainSet::default(), &[0.0, 0.0, 0.0], 2.0);
    s.push(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0));
    s.push(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0));
    s.push(line(3, [60.0, 0.0, 0.0], [90.0, 0.0, 0.0], 0.0));

    let committed = s.commit(true).unwrap();
    assert_eq!(committed[0].t_start, 2.0);
    for w in committed.windows(2) {
        assert!(
            (w[1].t_start - w[0].t_end).abs() < 1e-9,
            "time gap between segments"
        );
    }
}

#[test]
fn advance_idle_reanchors_committed_time_after_a_gap() {
    let mut s = StreamState::new(cfg(1.0), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0));
    let first = s.commit(true).unwrap();
    let after_first = first.last().unwrap().t_end;

    let idle_gap_past_horizon_secs = 50.0;
    s.advance_idle(after_first + idle_gap_past_horizon_secs);
    s.push(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0));
    let second = s.commit(true).unwrap();
    assert!(
        second[0].t_start >= after_first + idle_gap_past_horizon_secs - 1e-9,
        "second move must start at the re-anchored time, got {}",
        second[0].t_start
    );
    s.advance_idle(0.0);
    assert!(s.t_committed() >= second.last().unwrap().t_end - 1e-9);
}
