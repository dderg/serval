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
    let mut s = StreamState::new(cfg(0.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0));

    let committed = s.commit(false).unwrap();
    // The collinear junction is a clean seam: the first jog commits, the second is kept.
    assert!(!committed.is_empty());
    assert_eq!(s.buffered(), 1);
    // No stop between the jogs: the carried seam velocity is well above rest.
    assert!(
        s.entry_velocity() > 1.0,
        "seam velocity {} should be cruising, not stopped",
        s.entry_velocity()
    );
    // Committed boundary position is the seam (x = 50).
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn flush_commits_everything_to_rest() {
    let mut s = StreamState::new(cfg(1.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0));

    // keep_secs large => nothing commits without force.
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
    // A 90-degree corner is blended (clothoid); there is no clean seam, so a
    // non-forced commit must emit nothing rather than split the blend.
    let mut s = StreamState::new(cfg(0.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0));

    assert!(
        s.commit(false).unwrap().is_empty(),
        "must not commit across a blended corner"
    );
    assert_eq!(s.buffered(), 2);

    // Force still drains it to rest.
    assert!(!s.commit(true).unwrap().is_empty());
    assert!(s.is_empty());
}

#[test]
fn odometer_accumulates_extrusion_across_commits() {
    let mut s = StreamState::new(cfg(1.0), &[0.0, 0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0));
    s.push(line(2, [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], 4.0));

    let committed = s.commit(true).unwrap();
    assert!(s.is_empty());
    let last = committed.last().unwrap();
    // Extruder (axis 3) reached the cumulative delta 8.0 at the final boundary.
    assert!((eval(&last.axes[3], last.t_end) - 8.0).abs() < 1e-3);
}

#[test]
fn committed_trajectory_is_time_contiguous() {
    let mut s = StreamState::new(cfg(1.0), &[0.0, 0.0, 0.0], 2.0);
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
