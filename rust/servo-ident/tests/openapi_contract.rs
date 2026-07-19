//! The OpenAPI document is the one schema source for the `serve` JSON API.
//! These tests pin it down on five axes: the `/api/openapi.json` route
//! actually serves it, its path/method inventory matches the router, every
//! `$ref` it emits resolves inside the same document, real served payloads
//! validate against the very schemas the document publishes, and the
//! committed `web/openapi.json` artifact stays byte-semantically identical to
//! `document()`.

use std::collections::BTreeSet;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::time::Duration;

use jsonschema::Validator;
use serde_json::{json, Value};

use servo_ident::demo::build_demo;
use servo_ident::live_stream::LiveTap;
use servo_ident::openapi::document;
use servo_ident::{http, serve};

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("servo_openapi_{label}_{nanos}"));
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

fn collect_refs(value: &Value, out: &mut Vec<String>) {
    match value {
        Value::Object(map) => {
            for (key, child) in map {
                if key == "$ref" {
                    if let Value::String(s) = child {
                        out.push(s.clone());
                    }
                } else {
                    collect_refs(child, out);
                }
            }
        }
        Value::Array(items) => items.iter().for_each(|c| collect_refs(c, out)),
        _ => {}
    }
}

fn validator_from(doc: &Value, schema: &Value) -> Validator {
    let root = json!({
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "components": doc["components"].clone(),
        "allOf": [schema],
    });
    jsonschema::validator_for(&root)
        .unwrap_or_else(|e| panic!("schema does not compile: {e}\n{root}"))
}

fn assert_matches(validator: &Validator, instance: &Value, label: &str) {
    let errors: Vec<String> = validator
        .iter_errors(instance)
        .map(|e| format!("{} at {}", e, e.instance_path()))
        .collect();
    assert!(
        errors.is_empty(),
        "{label}: served payload violates its document schema:\n{}",
        errors.join("\n")
    );
}

fn ok_schema<'a>(doc: &'a Value, path: &str, method: &str) -> &'a Value {
    &doc["paths"][path][method]["responses"]["200"]["content"]["application/json"]["schema"]
}

#[test]
fn document_is_openapi_31_and_deterministic() {
    let doc = document();
    assert_eq!(doc["openapi"], "3.1.0");
    assert!(doc["paths"].is_object());
    assert!(doc["components"]["schemas"].is_object());
    assert_eq!(
        serde_json::to_string(&document()).unwrap(),
        serde_json::to_string(&doc).unwrap(),
        "document() must serialize identically on every call",
    );
}

#[test]
fn openapi_route_serves_the_document() {
    let root = temp_dir("route");
    let port = spawn_server(root.clone());
    let served = get(port, "/api/openapi.json");
    assert_eq!(served, document(), "served /api/openapi.json != document()");
    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn path_method_inventory_matches_the_router() {
    let doc = document();
    let mut actual = BTreeSet::new();
    for (path, ops) in doc["paths"].as_object().expect("paths is object") {
        for method in ops.as_object().expect("operations object").keys() {
            actual.insert(format!("{} {path}", method.to_uppercase()));
        }
    }
    let expected: BTreeSet<String> = [
        "GET /api/openapi.json",
        "GET /api/runs",
        "GET /api/drive_state",
        "GET /api/live",
        "GET /api/live/{name}",
        "GET /api/live_tap",
        "GET /api/runs/{name}/manifest",
        "GET /api/runs/{name}/results",
        "GET /api/runs/{name}/plot_series",
        "GET /api/runs/{name}/path",
        "GET /api/runs/{name}/strain",
        "POST /api/runs/{name}/analyze",
        "POST /api/runs/{name}/note",
        "DELETE /api/runs/{name}",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    assert_eq!(actual, expected);
}

#[test]
fn every_ref_resolves_within_the_document() {
    let doc = document();
    let mut refs = Vec::new();
    collect_refs(&doc, &mut refs);
    assert!(!refs.is_empty(), "document emits no $ref at all");
    for reference in refs {
        let pointer = reference
            .strip_prefix('#')
            .unwrap_or_else(|| panic!("non-local $ref: {reference}"));
        assert!(doc.pointer(pointer).is_some(), "dangling $ref: {reference}",);
    }
}

#[test]
fn served_payloads_validate_against_document_components() {
    let doc = document();
    let root = temp_dir("payloads");
    let run_dirs = build_demo(&root, &fixture_dir()).expect("build_demo");
    let port = spawn_server(root.clone());

    let results_v = validator_from(&doc, ok_schema(&doc, "/api/runs/{name}/results", "get"));
    let plot_v = validator_from(&doc, ok_schema(&doc, "/api/runs/{name}/plot_series", "get"));
    let path_v = validator_from(&doc, ok_schema(&doc, "/api/runs/{name}/path", "get"));
    let runs_v = validator_from(&doc, ok_schema(&doc, "/api/runs", "get"));
    let drive_v = validator_from(&doc, ok_schema(&doc, "/api/drive_state", "get"));
    let live_v = validator_from(&doc, ok_schema(&doc, "/api/live", "get"));
    let manifest_v = validator_from(&doc, ok_schema(&doc, "/api/runs/{name}/manifest", "get"));

    for run_dir in &run_dirs {
        let name = run_dir.file_name().unwrap().to_string_lossy().into_owned();
        assert_matches(
            &results_v,
            &get(port, &format!("/api/runs/{name}/results")),
            &format!("{name}: results"),
        );
        assert_matches(
            &plot_v,
            &get(port, &format!("/api/runs/{name}/plot_series")),
            &format!("{name}: plot_series"),
        );
        assert_matches(
            &path_v,
            &get(port, &format!("/api/runs/{name}/path")),
            &format!("{name}: path"),
        );
        assert_matches(
            &manifest_v,
            &get(port, &format!("/api/runs/{name}/manifest")),
            &format!("{name}: manifest"),
        );
    }

    assert_matches(&runs_v, &get(port, "/api/runs"), "runs list");
    assert_matches(&drive_v, &get(port, "/api/drive_state"), "drive_state");
    assert_matches(&live_v, &get(port, "/api/live"), "live");

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn committed_openapi_json_matches_the_document() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web/openapi.json");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "read {}: {e}\nregenerate with: cargo run -p servo-ident --bin servo-openapi > rust/servo-ident/web/openapi.json",
            path.display()
        )
    });
    let committed: Value = serde_json::from_str(&text)
        .unwrap_or_else(|e| panic!("{}: not valid json: {e}", path.display()));
    assert_eq!(
        committed,
        document(),
        "committed web/openapi.json is stale; regenerate with: \
         cargo run -p servo-ident --bin servo-openapi > rust/servo-ident/web/openapi.json",
    );
}

#[test]
fn live_tap_status_shapes_validate_against_document() {
    let doc = document();
    let validator = validator_from(&doc, ok_schema(&doc, "/api/live_tap", "get"));

    let dir = temp_dir("live_tap");
    let tap = LiveTap::new(dir.join("absent.sock"), Duration::from_millis(200));

    let connecting = tap.poll(None);
    assert_eq!(
        connecting["status"], "connecting",
        "first idle poll connects lazily"
    );
    assert_matches(&validator, &connecting, "live_tap connecting");

    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let unreachable = loop {
        let payload = tap.poll(None);
        if payload["status"] == "unreachable" {
            break payload;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "live tap never reported unreachable for a missing socket"
        );
        std::thread::sleep(Duration::from_millis(20));
    };
    assert!(
        unreachable["reason"].is_string(),
        "unreachable payload carries a reason string: {unreachable}"
    );
    assert_matches(&validator, &unreachable, "live_tap unreachable");

    assert!(
        !validator.is_valid(&json!({ "status": "connecting", "reason": "x" })),
        "the connecting variant forbids unknown fields"
    );
    assert!(
        !validator.is_valid(&json!({ "status": "streaming" })),
        "the streaming variant requires its attach fields"
    );

    std::fs::remove_dir_all(&dir).ok();
}
