//! Schema-validates every JSON payload `serve` actually hands the SPA
//! against the `JsonSchema` derived on its Rust struct — the JS side reads
//! these bodies by field name with no further checking, so a Rust rename
//! that silently drops or retypes a field is exactly what this test exists
//! to catch before it reaches the dashboard.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::time::Duration;

use jsonschema::Validator;
use schemars::Schema;
use serde_json::Value;

use servo_ident::demo::{build_demo, drive_state_schema};
use servo_ident::results::{PlotSeries, Results};
use servo_ident::serve::{RunPath, RunSummary};
use servo_ident::{http, serve};

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "servo_cal_schema_{label}_{}_{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn spawn_server(captures_root: PathBuf) -> u16 {
    let listener = http::bind("127.0.0.1", 0).expect("bind ephemeral port");
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        http::run(listener, move |req| serve::handle(&captures_root, req));
    });
    port
}

fn get(port: u16, path: &str) -> Value {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect");
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let req = format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n");
    stream.write_all(req.as_bytes()).unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let text = String::from_utf8_lossy(&raw);
    let mut parts = text.splitn(2, "\r\n\r\n");
    let head = parts.next().unwrap_or("");
    let body = parts.next().unwrap_or("");
    let status: u16 = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .expect("status line");
    assert_eq!(status, 200, "GET {path}: {body}");
    serde_json::from_str(body).unwrap_or_else(|e| panic!("GET {path}: body is not json: {e}"))
}

fn validator_for(schema: &Schema, label: &str) -> Validator {
    let schema_value = serde_json::to_value(schema).unwrap();
    jsonschema::validator_for(&schema_value)
        .unwrap_or_else(|e| panic!("{label}: generated schema does not compile: {e}"))
}

fn assert_matches(validator: &Validator, instance: &Value, label: &str) {
    let errors: Vec<String> = validator
        .iter_errors(instance)
        .map(|e| format!("{} at {}", e, e.instance_path()))
        .collect();
    assert!(
        errors.is_empty(),
        "{label}: served payload violates its own JSON Schema:\n{}",
        errors.join("\n")
    );
}

/// Builds a demo captures root, serves it over the real HTTP router, and
/// checks every response body against the `JsonSchema` its Rust type
/// derives — the same contract `docs/rewrite/servo-cal-contracts.md`
/// describes in prose, now machine-checked on both ends.
#[test]
fn served_payloads_match_their_derived_json_schemas() {
    let root = temp_dir("payloads");
    let run_dirs = build_demo(&root, &fixture_dir()).expect("build_demo");
    let port = spawn_server(root.clone());

    let results_schema = schemars::schema_for!(Results);
    let plot_series_schema = schemars::schema_for!(PlotSeries);
    let run_summary_list_schema = schemars::schema_for!(Vec<RunSummary>);
    let run_path_schema = schemars::schema_for!(RunPath);
    let drive_state_schema = drive_state_schema();

    let results_validator = validator_for(&results_schema, "results.json");
    let plot_series_validator = validator_for(&plot_series_schema, "plot_series.json");
    let runs_list_validator = validator_for(&run_summary_list_schema, "runs list");
    let run_path_validator = validator_for(&run_path_schema, "run path");
    let drive_state_validator = validator_for(&drive_state_schema, "drive_state");

    for run_dir in &run_dirs {
        let name = run_dir.file_name().unwrap().to_string_lossy().into_owned();

        let results = get(port, &format!("/api/runs/{name}/results"));
        assert_matches(
            &results_validator,
            &results,
            &format!("{name}: results.json"),
        );

        let plot_series = get(port, &format!("/api/runs/{name}/plot_series"));
        assert_matches(
            &plot_series_validator,
            &plot_series,
            &format!("{name}: plot_series.json"),
        );

        let run_path = get(port, &format!("/api/runs/{name}/path"));
        assert_matches(&run_path_validator, &run_path, &format!("{name}: path"));
    }

    let runs_list = get(port, "/api/runs");
    assert_matches(&runs_list_validator, &runs_list, "runs list");

    let drive_state = get(port, "/api/drive_state");
    assert_matches(&drive_state_validator, &drive_state, "drive_state");

    std::fs::remove_dir_all(&root).ok();
}
