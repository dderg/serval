//! Where the advance valleys release a ratio-ramped advance
//! ([`motion_pipeline::valley_audit`]): no crawl valley holds the advance
//! at zero further from its crawl than half the leaders' reach there, and
//! how much path each case releases it over, which the snapshot review
//! reads (`scripts/review_baselines.py`).

mod corpus;

use std::sync::Mutex;

use geometry::VALLEY_RAMP_TIME_S;
use motion_pipeline::valley_audit::{LaidCrawl, take_laid, take_released};

/// Held across an audited plan: the audit gathers what every thread lays
/// into one record.
static AUDITING: Mutex<()> = Mutex::new(());

/// The farthest a crawl valley holds the advance at zero from its crawl,
/// as a share of the leaders' reach there.
const ZERO_REACH_SHARE: f64 = 0.5;

/// How far past an exact bound the floats of a laid stretch may land.
const ROUNDING_MM: f64 = 1e-9;

fn laid_crawls(name: &str, gcode: &str, config: &std::path::Path) -> Vec<LaidCrawl> {
    let _auditing = AUDITING.lock().expect("an audited plan panicked");
    take_laid();
    corpus::plan(name, gcode, config).expect("the draw plans motion");
    take_laid()
}

/// The farthest a laid zero stretch lies from its crawl.
fn zero_reach(crawl: &LaidCrawl) -> f64 {
    (crawl.turn - crawl.zero.0).max(crawl.zero.1 - crawl.turn)
}

#[test]
fn crawl_valleys_hold_the_advance_at_zero_within_half_their_reach() {
    for draw in &corpus::crawl_cascade::DRAWS {
        let laid = laid_crawls(draw.name, draw.gcode, &draw.config());
        assert!(!laid.is_empty(), "{}: no crawl valley laid", draw.name);
        for crawl in &laid {
            assert!(
                zero_reach(crawl) <= ZERO_REACH_SHARE * crawl.reach + ROUNDING_MM,
                "{}: {crawl:?} holds the advance at zero {} mm from its crawl, past half its reach",
                draw.name,
                zero_reach(crawl)
            );
        }
    }
}

#[test]
fn leaders_leave_a_crawl_valleys_zero_stretch_within_half_its_ramp_time() {
    let half_ramp = 0.5 * VALLEY_RAMP_TIME_S;
    for draw in &corpus::crawl_cascade::DRAWS {
        for crawl in laid_crawls(draw.name, draw.gcode, &draw.config()) {
            let covered = crawl.speed * half_ramp + 0.5 * crawl.rate * half_ramp * half_ramp;
            assert!(
                zero_reach(&crawl) <= covered + ROUNDING_MM,
                "{}: {crawl:?} holds the advance at zero {} mm from its crawl, past the {covered} \
                 mm its leaders cover in {half_ramp} s",
                draw.name,
                zero_reach(&crawl)
            );
        }
    }
}

fn report_released(cases: Vec<(String, String, std::path::PathBuf)>) {
    for (name, gcode, config) in cases {
        let _auditing = AUDITING.lock().expect("an audited plan panicked");
        take_released();
        corpus::plan(&name, &gcode, &config);
        let released = take_released();
        println!(
            "RELEASED {name} any={} crawl={}",
            released.any, released.crawl
        );
    }
}

/// One `RELEASED <case> any=<mm> crawl=<mm>` line per snapshot case: the
/// path where any valley, and where a crawl valley, reads the advance a
/// weight below 1.
#[test]
#[ignore = "a report for scripts/review_baselines.py"]
fn every_snapshot_case_reports_the_path_it_releases_the_advance_over() {
    report_released(corpus::snapshot_cases());
}

/// The same report for every corpus case `CORPUS_FILTER` and
/// `CORPUS_CONFIGS` keep.
#[test]
#[ignore = "a report, not a check"]
fn every_corpus_case_reports_the_path_it_releases_the_advance_over() {
    report_released(
        corpus::corpus()
            .into_iter()
            .map(|case| (case.name, case.gcode, case.config))
            .collect(),
    );
}
