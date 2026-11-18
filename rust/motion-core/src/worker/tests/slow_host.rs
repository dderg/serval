//! #983: on the Pi bench a short lead-in at print speed followed by a long
//! slow section aborted klippy at the print start. The shaper let the
//! lowerer's 8.4 ms rest hold through first while the planning front still
//! held the motion behind it, and the Pi took 0.28-0.84 s to plan its way to
//! that motion: the stream had set off on the hold, and its playhead
//! overran it. Here the real pipeline plans the bench's print on a mock
//! clock, and the producer takes the bench's longest gap to hand on the
//! motion behind each stream's first segment.

use super::*;
use crate::anchor::LOW_MARGIN_WARN_SECS;
use crate::seam_test_harness::LiveScript;
use geometry::GcodePos;
use pipeline_snapshot::PlanStep;

/// crawl_s5's gap between the first segment and the next on the Pi bench.
const BENCH_GAP_AFTER_FIRST_SEGMENT: Duration = Duration::from_millis(840);
const CRAWL_S: f64 = 60.0;
const PIECE_S: f64 = 2.0;
const SIDES: usize = 6;
const PRINT_FEED: f64 = 6000.0;
const E_PER_MM: f64 = 0.0374;
const SQUARE_HEADINGS: [(f64, f64); 4] = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
const REST_BEFORE_THE_CRAWL: &str = "G4 P500";

/// A dispatched segment the anchor would have aborted on: its start was
/// `margin_s` ahead of the playhead, under the transport floor, mid-motion.
struct Overrun {
    seg_t_start: f64,
    margin_s: f64,
}

/// Anchors every segment as the pump does. One the anchor would abort on is
/// recorded instead and the stream taken as parked, so the print runs on
/// and the test reports every overrun.
#[derive(Clone)]
struct OverrunWatch {
    inner: AnchoredSink,
    overruns: Arc<Mutex<Vec<Overrun>>>,
}

impl SegmentSink for OverrunWatch {
    fn dispatch(&mut self, seg: &ContinuousSegment) -> Result<(), DispatchError> {
        let host_now = self.inner.router.lock_ok().host_now_secs();
        {
            let mut anchor = self.inner.anchor.lock_ok();
            if anchor.planned_start(seg.t_start, host_now).is_none() {
                let committed_end = anchor
                    .committed_end_host()
                    .expect("only an anchored stream overruns");
                self.overruns.lock_ok().push(Overrun {
                    seg_t_start: seg.t_start,
                    margin_s: committed_end - host_now,
                });
                anchor.mark_parked();
            }
        }
        self.inner.dispatch(seg)
    }
    fn dispatch_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
    ) -> Result<(), DispatchError> {
        self.inner.dispatch_nudge(mcu_id, axis, motor_mask, profile)
    }
    fn mark_parked(&mut self) {
        self.inner.mark_parked();
    }
    fn follow_stream_to(&mut self, stream_t: f64) {
        self.inner.follow_stream_to(stream_t);
    }
    fn set_off_in(&self, seg: &ContinuousSegment) -> Option<Duration> {
        self.inner.set_off_in(seg)
    }
}

fn fmt_mm(value: f64) -> String {
    let text = format!("{value:.4}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn extrude(x: f64, y: f64, mm: f64, feed: f64) -> String {
    format!(
        "G1 X{} Y{} E{} F{feed}",
        fmt_mm(x),
        fmt_mm(y),
        fmt_mm(mm * E_PER_MM)
    )
}

/// The bench's crawl rows (`crawl_gcode.py`): a 2.3 s lead-in at 100 mm/s,
/// `before_crawl`, then `M220 S<factor_pct>` and `CRAWL_S` of extruding
/// square laps in `PIECE_S` pieces that never come to rest, then the travel
/// that ends the crawl and a retract.
fn crawl_print(factor_pct: f64, crawl_feed: f64, before_crawl: &[&str]) -> String {
    let speed_mm_s = crawl_feed / 60.0 * factor_pct / 100.0;
    let side_mm = speed_mm_s * CRAWL_S / SIDES as f64;
    let pieces = (side_mm / (speed_mm_s * PIECE_S)).round();
    let piece_mm = side_mm / pieces;
    let mut lines = vec![
        "G90".to_string(),
        "M83".to_string(),
        "G1 Z0.2 F600".to_string(),
        "G1 X40 Y50 F12000".to_string(),
        extrude(140.0, 50.0, 100.0, PRINT_FEED),
        extrude(140.0, 60.0, 10.0, PRINT_FEED),
        extrude(40.0, 60.0, 100.0, PRINT_FEED),
    ];
    lines.extend(before_crawl.iter().map(ToString::to_string));
    lines.push(format!("M220 S{factor_pct}"));
    let (mut x, mut y) = (40.0, 60.0);
    for side in 0..SIDES {
        let (dx, dy) = SQUARE_HEADINGS[side % SQUARE_HEADINGS.len()];
        for _ in 0..pieces as usize {
            x += dx * piece_mm;
            y += dy * piece_mm;
            lines.push(extrude(x, y, piece_mm, crawl_feed));
        }
    }
    lines.push("M220 S100".to_string());
    lines.push(format!("G1 X40 Y{} F12000", fmt_mm(y + 10.0)));
    lines.push("G1 E-0.5 F2400".to_string());
    lines.join("\n") + "\n"
}

fn bench_planner() -> planner_config::PlannerConfig {
    let path = source_tree::repo_file("tools/bench/printer.cfg");
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    planner_config::from_doc::planner_config_from_text(&text).expect("bench planner config")
}

/// Prints `gcode` on the bench printer's config with each stream's
/// producer handing on the motion behind its first segment the bench's
/// gap later, and returns every segment the anchor would have aborted on.
fn overruns_printing(gcode: &str) -> Vec<Overrun> {
    let script = LiveScript::new(gcode, &bench_planner(), None, GcodePos([0.0; 3]))
        .unwrap_or_else(|e| panic!("crawl print: {e}"));
    let clock = VirtualClock::start();
    let timeline = virtual_timeline(&clock);
    let watch = OverrunWatch {
        inner: AnchoredSink::on(&timeline),
        overruns: Arc::default(),
    };
    let mut h = spawn_recorded(
        script.config.clone(),
        script.chains.clone(),
        script.home.clone(),
        watch.clone(),
        timeline,
        None,
        Production::GapAfterFirstSegments {
            clock: Arc::clone(&clock),
            gap: BENCH_GAP_AFTER_FIRST_SEGMENT,
        },
    );
    for step in script.steps {
        match step {
            PlanStep::Move(m) => submit_on(&clock, &h, &m),
            PlanStep::Rest { dwell_s } => h
                .dwell_try_start(dwell_s)
                .expect("the dwell is queued")
                .recv()
                .expect("the dwell is planned"),
        }
    }
    h.flush().unwrap();
    h.shutdown();
    std::mem::take(&mut *watch.overruns.lock_ok())
}

fn assert_starts_without_overrun(factor_pct: f64, crawl_feed: f64, before_crawl: &[&str]) {
    let overruns: Vec<(f64, f64)> =
        overruns_printing(&crawl_print(factor_pct, crawl_feed, before_crawl))
            .iter()
            .map(|overrun| (overrun.seg_t_start, overrun.margin_s))
            .collect();
    assert!(
        overruns.is_empty(),
        "a {CRAWL_S} s crawl at {} mm/s after a 2.3 s lead-in and {before_crawl:?} left \
         segments (stream start, margin s) within the {LOW_MARGIN_WARN_SECS} s transport \
         floor of the playhead: {overruns:?}",
        crawl_feed / 60.0 * factor_pct / 100.0
    );
}

#[test]
fn a_crawl_at_one_mm_s_after_a_short_lead_in_starts_ahead_of_its_playhead() {
    assert_starts_without_overrun(1.0, 6000.0, &[]);
}

#[test]
fn a_crawl_at_two_and_a_half_mm_s_after_a_short_lead_in_starts_ahead_of_its_playhead() {
    assert_starts_without_overrun(5.0, 3000.0, &[]);
}

#[test]
fn a_crawl_at_ten_mm_s_after_a_short_lead_in_starts_ahead_of_its_playhead() {
    assert_starts_without_overrun(10.0, 6000.0, &[]);
}

/// The stream comes to rest inside the print and sets off again.
#[test]
fn a_crawl_after_a_rest_inside_the_print_sets_off_ahead_of_its_playhead() {
    assert_starts_without_overrun(5.0, 3000.0, &[REST_BEFORE_THE_CRAWL]);
}

/// The gap a producer that keeps feeding leaves between two moves: klippy
/// parses and submits the next line while the ingress has already taken
/// the last one.
const FEEDING_GAP: Duration = Duration::from_millis(20);

/// #1003: crawl_s20 on the Pi. A stream at rest whose planning front holds
/// its slow section found its inbox empty between two of klippy's moves
/// after its prime, counted from its first move, had run out, and braked to
/// rest: the drain planned every move held to rest, and the toolhead
/// stopped where they ended, 108 s into the slow section. Nothing was in
/// flight to protect, and klippy was still feeding.
#[test]
fn a_stream_at_rest_never_brakes_while_its_producer_keeps_feeding() {
    let script = LiveScript::new(
        &crawl_print(20.0, PRINT_FEED, &[]),
        &bench_planner(),
        None,
        GcodePos([0.0; 3]),
    )
    .unwrap_or_else(|e| panic!("crawl print: {e}"));
    let clock = VirtualClock::start();
    let timeline = virtual_timeline(&clock);
    let mut h = spawn_on_timeline(
        script.config.clone(),
        script.chains.clone(),
        script.home.clone(),
        AnchoredSink::on(&timeline),
        timeline,
    );
    for step in script.steps {
        let PlanStep::Move(m) = step else {
            panic!("the crawl print holds no rest");
        };
        submit_on(&clock, &h, &m);
        clock.sleep(FEEDING_GAP);
    }
    let brakes = (
        h.links.pacer_drains.idle.load(Ordering::SeqCst),
        h.links.pacer_drains.busy.load(Ordering::SeqCst),
    );
    h.flush().unwrap();
    h.shutdown();
    assert_eq!(
        brakes,
        (0, 0),
        "the pacer braked (idle, busy) a stream at rest while its producer kept \
         feeding a move every {FEEDING_GAP:?}"
    );
}
