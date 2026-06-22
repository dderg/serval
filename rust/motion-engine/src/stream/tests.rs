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
        max_buffer_moves: 64,
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
fn blended_corner_commits_through_the_blend_without_stopping() {
    // A 90-degree corner is blended (a biclothoid). The blend rejoins the
    // outgoing line at zero curvature, so the commit runs through the whole
    // blend and keeps the outgoing move as a head-trimmed remainder — never
    // splitting the blend itself, and never stopping at the corner.
    let mut s = StreamState::new(cfg(0.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0));
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0));

    let committed = s.commit(false).unwrap();
    assert!(!committed.is_empty(), "the blend must commit");
    assert_eq!(
        s.buffered(),
        1,
        "outgoing move kept as a head-trimmed remainder"
    );
    assert!(
        s.entry_velocity() > 1.0,
        "corner is rounded, not stopped: seam velocity {}",
        s.entry_velocity()
    );

    // The kept remainder still ends at the original move endpoint.
    let rest = s.commit(true).unwrap();
    assert!(s.is_empty());
    let last = rest.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
    assert!((eval(&last.axes[1], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn continuous_blended_chain_drains_without_a_single_stop() {
    // A gentle zigzag: every corner is shallow enough to blend, so there is no
    // unblended seam anywhere. The continuity commit must still drain it (the
    // old clean-seam-only commit would hang here forever), and no interior seam
    // may drop to rest — that would be the stutter we are eliminating.
    let mut s = StreamState::new(cfg(0.05), &[0.0, 0.0, 0.0], 0.0);
    let pts = [
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [40.0, 3.0, 0.0],
        [60.0, 0.0, 0.0],
        [80.0, 3.0, 0.0],
        [100.0, 0.0, 0.0],
        [120.0, 3.0, 0.0],
    ];
    for (i, w) in pts.windows(2).enumerate() {
        s.push(line(i as u32 + 1, w[0], w[1], 0.0));
    }
    let pushed = pts.len() - 1;

    let mut all: Vec<trajectory::ShapedSegment> = Vec::new();
    let mut progressed = true;
    while progressed {
        let committed = s.commit(false).unwrap();
        progressed = !committed.is_empty();
        for seg in committed {
            if let Some(prev) = all.last() {
                assert!(
                    s.entry_velocity() >= 0.0,
                    "entry velocity must stay defined"
                );
                assert!(
                    (seg.t_start - prev.t_end).abs() < 1e-9,
                    "time gap between committed segments"
                );
            }
            all.push(seg);
        }
    }
    assert!(
        s.buffered() < pushed,
        "continuity commit must drain a fully-blended chain ({} of {} left)",
        s.buffered(),
        pushed
    );
    // Every interior seam carried real speed: the planner never paused.
    assert!(
        s.entry_velocity() > 1.0,
        "final carried seam velocity {} should be cruising",
        s.entry_velocity()
    );

    let rest = s.commit(true).unwrap();
    for w in rest.windows(2) {
        assert!((w[1].t_start - w[0].t_end).abs() < 1e-9);
    }
    if let (Some(prev), Some(first)) = (all.last(), rest.first()) {
        assert!((first.t_start - prev.t_end).abs() < 1e-9);
    }
    assert!(s.is_empty());
}

#[test]
fn head_trim_preserves_position_and_extrusion_continuity() {
    // Commit through a blend with extrusion, then verify the kept remainder
    // resumes exactly where the committed trajectory ended (no gap, no overlap)
    // in both the spatial axes and the extruder.
    let mut s = StreamState::new(cfg(0.05), &[0.0, 0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0));
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 5.0));
    s.push(line(3, [50.0, 50.0, 0.0], [100.0, 50.0, 0.0], 5.0));

    let first = s.commit(false).unwrap();
    assert!(!first.is_empty());
    let seam = first.last().unwrap();
    let seam_x = eval(&seam.axes[0], seam.t_end);
    let seam_y = eval(&seam.axes[1], seam.t_end);
    let seam_e = eval(&seam.axes[3], seam.t_end);

    let rest = s.commit(true).unwrap();
    let resume = rest.first().unwrap();
    assert!((eval(&resume.axes[0], resume.t_start) - seam_x).abs() < 1e-6);
    assert!((eval(&resume.axes[1], resume.t_start) - seam_y).abs() < 1e-6);
    assert!((eval(&resume.axes[3], resume.t_start) - seam_e).abs() < 1e-6);

    // Total extrusion across the whole (3-move) path is conserved: 15 mm.
    let last = rest.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 15.0).abs() < 1e-3);
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

#[test]
fn advance_idle_reanchors_committed_time_after_a_gap() {
    let mut s = StreamState::new(cfg(1.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0));
    let first = s.commit(true).unwrap();
    let after_first = first.last().unwrap().t_end;

    // Long idle gap: the machine has caught up well past the committed horizon.
    s.advance_idle(after_first + 50.0);
    s.push(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0));
    let second = s.commit(true).unwrap();
    assert!(
        second[0].t_start >= after_first + 50.0 - 1e-9,
        "second move must start at the re-anchored time, got {}",
        second[0].t_start
    );
    // never rewinds:
    s.advance_idle(0.0);
    assert!(s.t_committed() >= second.last().unwrap().t_end - 1e-9);
}
