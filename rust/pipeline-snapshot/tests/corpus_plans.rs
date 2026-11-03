//! Every corpus case plans through the production pipeline. Heavy (the full
//! sim prints), so ignored by default; `scripts/corpus.sh` runs it. Every
//! case runs even when an earlier one panics, and all failures are listed.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::time::Instant;

mod corpus;

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_else(|| "non-string panic".to_string())
}

#[test]
#[ignore = "full sim prints; run with scripts/corpus.sh"]
fn every_corpus_case_plans() {
    let cases = corpus::corpus();
    assert!(
        !cases.is_empty(),
        "CORPUS_FILTER/CORPUS_CONFIGS select no case"
    );
    let mut failures = Vec::new();
    for case in &cases {
        let started = Instant::now();
        let planned = catch_unwind(AssertUnwindSafe(|| {
            corpus::plan(&case.name, &case.gcode, &case.config)
                .unwrap_or_else(|| panic!("fewer than two waypoints"))
        }));
        match planned {
            Ok(snapshot)
                if snapshot.traversal_time_s.is_finite() && snapshot.traversal_time_s > 0.0 =>
            {
                println!(
                    "{:<70} {:7.2} s planned in {:6.2} s wall",
                    case.name,
                    snapshot.traversal_time_s,
                    started.elapsed().as_secs_f64()
                );
            }
            Ok(snapshot) => failures.push(format!(
                "{}: traversal time {}",
                case.name, snapshot.traversal_time_s
            )),
            Err(payload) => failures.push(format!("{}: {}", case.name, panic_message(payload))),
        }
    }
    println!("{} corpus cases, {} failed", cases.len(), failures.len());
    assert!(
        failures.is_empty(),
        "corpus cases failed:\n{}",
        failures.join("\n")
    );
}
