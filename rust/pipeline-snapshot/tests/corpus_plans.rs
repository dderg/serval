//! Every corpus case plans through the production pipeline and holds the
//! config's z caps and extruder limits. Heavy (the full sim prints on every
//! machine profile), so ignored by default; `scripts/corpus.sh` runs it.
//! The cases plan side by side, every case runs even when another one
//! panics, and all failures are listed.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

mod corpus;

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_else(|| "non-string panic".to_string())
}

/// What planning one case reports, and where it fails.
#[derive(Default)]
struct Outcome {
    report: Vec<String>,
    failures: Vec<String>,
}

fn plan_case(case: &corpus::Case) -> Outcome {
    let mut outcome = Outcome::default();
    let started = Instant::now();
    let planned = catch_unwind(AssertUnwindSafe(|| {
        corpus::plan(&case.name, &case.gcode, &case.config)
            .unwrap_or_else(|| panic!("fewer than two waypoints"))
    }));
    match planned {
        Ok(snapshot)
            if snapshot.traversal_time_s.is_finite() && snapshot.traversal_time_s > 0.0 =>
        {
            outcome.report.push(format!(
                "{:<70} {:7.2} s planned in {:6.2} s wall",
                case.name,
                snapshot.traversal_time_s,
                started.elapsed().as_secs_f64()
            ));
            if let Some(excess) = corpus::z_caps_excess(&snapshot, &case.config) {
                outcome.failures.push(format!("{}: {excess}", case.name));
            }
            if let Some(audit) =
                corpus::extruder::extruder_audit(&snapshot, &case.gcode, &case.config)
            {
                outcome
                    .report
                    .push(format!("{:<70} {}", "", audit.summary()));
                if let Some(excess) =
                    corpus::known_extruder_excesses::extruder_excess(&case.name, &audit)
                {
                    outcome.failures.push(format!("{}: {excess}", case.name));
                }
            }
        }
        Ok(snapshot) => outcome.failures.push(format!(
            "{}: traversal time {}",
            case.name, snapshot.traversal_time_s
        )),
        Err(payload) => outcome
            .failures
            .push(format!("{}: {}", case.name, panic_message(payload))),
    }
    outcome
}

#[test]
#[ignore = "full sim prints; run with scripts/corpus.sh"]
fn every_corpus_case_plans() {
    let cases = corpus::corpus();
    assert!(
        !cases.is_empty(),
        "CORPUS_FILTER/CORPUS_CONFIGS select no case"
    );
    let outcomes: Vec<Mutex<Outcome>> = cases.iter().map(|_| Mutex::default()).collect();
    let next = AtomicUsize::new(0);
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                loop {
                    let index = next.fetch_add(1, Ordering::Relaxed);
                    let Some(case) = cases.get(index) else {
                        break;
                    };
                    *outcomes[index].lock().expect("an outcome lock poisoned") = plan_case(case);
                }
            });
        }
    });
    let mut failures = Vec::new();
    for outcome in outcomes {
        let outcome = outcome.into_inner().expect("an outcome lock poisoned");
        for line in outcome.report {
            println!("{line}");
        }
        failures.extend(outcome.failures);
    }
    println!("{} corpus cases, {} failed", cases.len(), failures.len());
    assert!(
        failures.is_empty(),
        "corpus cases failed:\n{}",
        failures.join("\n")
    );
}
