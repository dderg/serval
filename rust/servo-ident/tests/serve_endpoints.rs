//! `servo-cal serve` endpoint tests: bind an ephemeral port, run the real
//! `http`/`serve` router against a temp captures root built by
//! `servo_ident::demo::build_demo`, and drive it with a hand-rolled HTTP
//! client (the server itself is hand-rolled — no client dependency either).

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use serde_json::Value;

use servo_ident::demo::build_demo;
use servo_ident::{assets, http, serve};

struct HttpResult {
    status: u16,
    body: String,
}

fn request(port: u16, method: &str, path: &str) -> HttpResult {
    request_with_body(port, method, path, "")
}

fn request_with_body(port: u16, method: &str, path: &str, body: &str) -> HttpResult {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect");
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let req = format!(
        "{method} {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    stream.write_all(req.as_bytes()).unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let text = String::from_utf8_lossy(&raw);
    let mut parts = text.splitn(2, "\r\n\r\n");
    let head = parts.next().unwrap_or("");
    let body = parts.next().unwrap_or("").to_string();
    let status: u16 = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .expect("status line");
    HttpResult { status, body }
}

fn spawn_server(captures_root: PathBuf) -> u16 {
    let listener = http::bind("127.0.0.1", 0).expect("bind ephemeral port");
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        http::run(listener, move |req| serve::handle(&captures_root, req));
    });
    port
}

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "servo_cal_serve_{label}_{}_{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn demo_root(label: &str) -> (PathBuf, Vec<PathBuf>) {
    let root = temp_dir(label);
    let run_dirs = build_demo(&root, &fixture_dir()).expect("build_demo");
    (root, run_dirs)
}

/// A bare manifest good enough for `list_runs` (which never touches step
/// captures) — used to build run dirs with names chosen so that
/// alphabetical order and creation order disagree, defeating a listing
/// that (bug!) sorted on name instead of the real filesystem mtime.
fn write_bare_manifest(run_dir: &Path, tag: &str) {
    std::fs::create_dir_all(run_dir).unwrap();
    let manifest = serde_json::json!({
        "version": 1,
        "experiment": "tracking",
        "tag": tag,
        "axis": "X",
        "steps": [{"name": "s1", "capture": "step_s1.scap"}]
    });
    std::fs::write(
        run_dir.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .unwrap();
}

#[test]
fn listing_orders_by_real_mtime_not_by_name() {
    let root = temp_dir("mtime_order");
    // "b_run" is written first (oldest) and "a_run" second (newest); a
    // name-based sort would put "b_run" first since 'b' > 'a', which is
    // exactly backwards from the real creation order this test asserts.
    write_bare_manifest(&root.join("b_run"), "b");
    std::thread::sleep(Duration::from_millis(30));
    write_bare_manifest(&root.join("a_run"), "a");

    let port = spawn_server(root.clone());
    let resp = request(port, "GET", "/api/runs");
    assert_eq!(resp.status, 200);
    let runs: Value = serde_json::from_str(&resp.body).unwrap();
    let names: Vec<&str> = runs
        .as_array()
        .unwrap()
        .iter()
        .map(|r| r["name"].as_str().unwrap())
        .collect();
    assert_eq!(
        names,
        vec!["a_run", "b_run"],
        "newest by mtime must lead, not newest by name"
    );

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn runs_list_is_newest_first_with_verdict_summary() {
    let (root, run_dirs) = demo_root("list");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs");
    assert_eq!(resp.status, 200);
    let runs: Value = serde_json::from_str(&resp.body).unwrap();
    let runs = runs.as_array().unwrap();
    assert_eq!(runs.len(), 3);

    // build_demo returns oldest-first; the endpoint must reverse that.
    let names: Vec<&str> = runs.iter().map(|r| r["name"].as_str().unwrap()).collect();
    let expected_newest_first: Vec<String> = run_dirs
        .iter()
        .rev()
        .map(|d| d.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names, expected_newest_first);

    for run in runs {
        assert_eq!(run["has_results"], Value::Bool(true));
        let verdict = &run["verdict"];
        assert!(!verdict.is_null(), "demo runs are analyzed up front");
        // attempt1's s700 carries a synthetic resonance (see
        // servo_ident::demo), so its verdict falls back to s550; attempt2
        // and attempt3 stay on the untouched fixture and still pick s700.
        let name = run["name"].as_str().unwrap();
        let expected_step = if name.contains("attempt1") {
            "s550"
        } else {
            "s700"
        };
        assert_eq!(
            verdict["recommended_step"],
            Value::from(expected_step),
            "{name}"
        );
        assert!(!verdict["reason"].as_str().unwrap().is_empty());
        assert_eq!(run["experiment"], Value::from("gain_sweep"));
    }

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn manifest_results_plot_series_serve_raw_json() {
    let (root, run_dirs) = demo_root("files");
    let port = spawn_server(root.clone());
    let name = run_dirs[0]
        .file_name()
        .unwrap()
        .to_string_lossy()
        .into_owned();

    for file in ["manifest", "results", "plot_series"] {
        let resp = request(port, "GET", &format!("/api/runs/{name}/{file}"));
        assert_eq!(resp.status, 200, "{file} endpoint");
        let parsed: Value = serde_json::from_str(&resp.body).expect("valid json body");
        assert_eq!(parsed["version"], Value::from(1));
        if file == "plot_series" {
            for step in parsed["steps"].as_array().unwrap() {
                let psd = &step["psd"];
                let freq_hz = psd["freq_hz"].as_array().expect("psd.freq_hz array");
                assert!(freq_hz.len() <= 2000, "psd.freq_hz must be <= 2000 bins");
                let per_drive = psd["per_drive"].as_object().expect("psd.per_drive object");
                assert!(!per_drive.is_empty());
                for (drive, series) in per_drive {
                    assert_eq!(
                        series.as_array().unwrap().len(),
                        freq_hz.len(),
                        "drive {drive} psd length must match freq_hz"
                    );
                }
                let accel = &psd["accel"];
                assert!(
                    !accel.is_null(),
                    "demo steps carry an accel capture, psd.accel must not be null"
                );
                let accel_freq = accel["freq_hz"].as_array().unwrap();
                let accel_psd = accel["psd"].as_array().unwrap();
                assert_eq!(accel_freq.len(), accel_psd.len());
            }
        }
    }

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn missing_run_is_404_with_reason_body() {
    let (root, _run_dirs) = demo_root("404");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs/does_not_exist/manifest");
    assert_eq!(resp.status, 404);
    let parsed: Value = serde_json::from_str(&resp.body).expect("404 body is json");
    assert!(parsed["error"].as_str().unwrap().contains("does_not_exist"));

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn directory_traversal_run_name_is_rejected() {
    let (root, _run_dirs) = demo_root("traversal");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs/../manifest");
    assert_ne!(resp.status, 200);
    let resp2 = request(port, "GET", "/api/runs/a%2F..%2F..%2Fetc/manifest");
    assert_ne!(resp2.status, 200);

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn index_page_serves_html() {
    let (root, _run_dirs) = demo_root("index");
    let port = spawn_server(root.clone());

    let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
    stream
        .write_all(b"GET / HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
        .unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let text = String::from_utf8_lossy(&raw);
    let mut lines = text.lines();
    assert!(lines.next().unwrap().contains("200"));
    assert!(text.to_lowercase().contains("content-type: text/html"));
    assert!(text.contains("<title>servo-cal</title>"));

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn built_assets_are_served_with_correct_mime_and_body() {
    let (root, _run_dirs) = demo_root("assets");
    let port = spawn_server(root.clone());

    assert!(assets::BUILT_ASSETS.len() >= 3, "expect html + js + css");
    for asset in assets::BUILT_ASSETS {
        let name = asset.path;
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        stream
            .write_all(
                format!("GET /{name} HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n").as_bytes(),
            )
            .unwrap();
        let mut raw = Vec::new();
        stream.read_to_end(&mut raw).unwrap();
        let header_end = raw
            .windows(4)
            .position(|w| w == b"\r\n\r\n")
            .expect("response has a header terminator");
        let head = String::from_utf8_lossy(&raw[..header_end]);
        assert!(
            head.lines().next().unwrap().contains("200"),
            "GET /{name} did not return 200"
        );
        assert!(
            head.to_lowercase()
                .contains(&format!("content-type: {}", asset.mime)),
            "GET /{name} missing content-type {}",
            asset.mime
        );
        assert_eq!(
            &raw[header_end + 4..],
            asset.body,
            "GET /{name} body did not match the embedded bundle"
        );
    }

    let resp = request(port, "GET", "/does-not-exist.js");
    assert_eq!(resp.status, 404);

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn analyze_on_demand_regenerates_stale_results() {
    let (root, run_dirs) = demo_root("analyze");
    let run_dir = &run_dirs[0];
    let name = run_dir.file_name().unwrap().to_string_lossy().into_owned();

    // Delete results.json/plot_series.json to simulate a fresh capture that
    // has never been analyzed.
    std::fs::remove_file(run_dir.join("results.json")).unwrap();
    std::fs::remove_file(run_dir.join("plot_series.json")).unwrap();

    let port = spawn_server(root.clone());
    let resp = request(port, "POST", &format!("/api/runs/{name}/analyze"));
    assert_eq!(resp.status, 200);
    let results: Value = serde_json::from_str(&resp.body).unwrap();
    // run_dirs[0] is attempt1 (build_demo returns oldest first), whose
    // injected s700 resonance falls the verdict back to s550.
    assert_eq!(results["verdict"]["recommended_step"], Value::from("s550"));
    assert!(run_dir.join("results.json").is_file());
    assert!(run_dir.join("plot_series.json").is_file());

    // Second call: results.json is now fresh, no capture changed underneath
    // it, so the endpoint must serve it back rather than re-running.
    let first_mtime = mtime(&run_dir.join("results.json"));
    std::thread::sleep(Duration::from_millis(20));
    let resp2 = request(port, "POST", &format!("/api/runs/{name}/analyze"));
    assert_eq!(resp2.status, 200);
    let second_mtime = mtime(&run_dir.join("results.json"));
    assert_eq!(
        first_mtime, second_mtime,
        "fresh results.json must not be recomputed"
    );

    // Touch a capture (rewrite identical bytes) so it postdates results.json;
    // analyze must recompute even though the on-disk content is unchanged.
    std::thread::sleep(Duration::from_millis(20));
    let capture_path = run_dir.join("step_s550.scap");
    let bytes = std::fs::read(&capture_path).unwrap();
    std::fs::write(&capture_path, bytes).unwrap();
    let resp3 = request(port, "POST", &format!("/api/runs/{name}/analyze"));
    assert_eq!(resp3.status, 200);
    let third_mtime = mtime(&run_dir.join("results.json"));
    assert!(
        third_mtime > second_mtime,
        "a newer capture must trigger re-analysis"
    );

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn delete_run_removes_the_directory() {
    let (root, run_dirs) = demo_root("delete");
    let run_dir = &run_dirs[0];
    let name = run_dir.file_name().unwrap().to_string_lossy().into_owned();
    let port = spawn_server(root.clone());

    let resp = request(port, "DELETE", &format!("/api/runs/{name}"));
    assert_eq!(resp.status, 200);
    let body: Value = serde_json::from_str(&resp.body).unwrap();
    assert_eq!(body["deleted"], Value::from(name.as_str()));
    assert!(!run_dir.exists(), "run directory must be gone");

    let resp = request(port, "GET", "/api/runs");
    let runs: Value = serde_json::from_str(&resp.body).unwrap();
    assert!(runs
        .as_array()
        .unwrap()
        .iter()
        .all(|r| r["name"] != *name.as_str()));

    let resp = request(port, "DELETE", &format!("/api/runs/{name}"));
    assert_eq!(resp.status, 404, "second delete must 404");
    let resp = request(port, "DELETE", "/api/runs/../escape");
    assert_eq!(resp.status, 404, "path traversal must be rejected");

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn note_roundtrips_without_marking_results_stale() {
    let (root, run_dirs) = demo_root("note");
    let run_dir = &run_dirs[0];
    let name = run_dir.file_name().unwrap().to_string_lossy().into_owned();
    let port = spawn_server(root.clone());

    let listed_note = |port| {
        let resp = request(port, "GET", "/api/runs");
        let runs: Value = serde_json::from_str(&resp.body).unwrap();
        runs.as_array()
            .unwrap()
            .iter()
            .find(|r| r["name"] == *name.as_str())
            .unwrap()["note"]
            .clone()
    };
    assert_eq!(listed_note(port), Value::Null);

    let results_mtime_before = mtime(&run_dir.join("results.json"));
    std::thread::sleep(Duration::from_millis(20));
    let resp = request_with_body(
        port,
        "POST",
        &format!("/api/runs/{name}/note"),
        r#"{"note": "  best run so far  "}"#,
    );
    assert_eq!(resp.status, 200);
    assert_eq!(listed_note(port), Value::from("best run so far"));

    // A note is commentary, not an analysis input: analyze must keep the
    // existing results even though note.txt now postdates results.json.
    let resp = request(port, "POST", &format!("/api/runs/{name}/analyze"));
    assert_eq!(resp.status, 200);
    assert_eq!(
        mtime(&run_dir.join("results.json")),
        results_mtime_before,
        "writing a note must not trigger re-analysis"
    );

    // Empty note clears it and removes the file.
    let resp = request_with_body(
        port,
        "POST",
        &format!("/api/runs/{name}/note"),
        r#"{"note": ""}"#,
    );
    assert_eq!(resp.status, 200);
    assert_eq!(listed_note(port), Value::Null);
    assert!(!run_dir.join("note.txt").exists());

    let resp = request_with_body(
        port,
        "POST",
        "/api/runs/no_such_run/note",
        r#"{"note":"x"}"#,
    );
    assert_eq!(resp.status, 404);
    let resp = request_with_body(port, "POST", &format!("/api/runs/{name}/note"), "not json");
    assert_eq!(resp.status, 400);

    std::fs::remove_dir_all(&root).ok();
}

fn mtime(path: &Path) -> std::time::SystemTime {
    std::fs::metadata(path).unwrap().modified().unwrap()
}

/// `servo-cal demo` writes `drive_state.json` straight into the captures
/// root it's given, so `GET /api/drive_state` must serve it back with the
/// full `PANEL_PARAMS`/motors/config_pins shape plus a freshly computed
/// `age_s` (never cached — the file was just written by `build_demo`, so
/// its age must read close to zero, not stale from an earlier request).
#[test]
fn drive_state_endpoint_serves_shape_and_a_fresh_age() {
    let (root, _run_dirs) = demo_root("drive_state");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/drive_state");
    assert_eq!(resp.status, 200);
    let parsed: Value = serde_json::from_str(&resp.body).expect("drive_state is json");
    assert_eq!(parsed["version"], Value::from(1));
    assert_eq!(parsed["params"].as_array().unwrap().len(), 29);
    assert_eq!(parsed["motors"].as_object().unwrap().len(), 4);
    assert_eq!(parsed["config_pins"].as_object().unwrap().len(), 4);
    let age_s = parsed["age_s"].as_f64().expect("age_s must be a number");
    assert!(
        age_s >= 0.0 && age_s < 30.0,
        "age_s {age_s} should be near zero right after build_demo"
    );

    std::thread::sleep(Duration::from_secs(1));
    let resp2 = request(port, "GET", "/api/drive_state");
    let parsed2: Value = serde_json::from_str(&resp2.body).unwrap();
    let age_s2 = parsed2["age_s"].as_f64().unwrap();
    assert!(
        age_s2 > age_s,
        "age_s must grow between requests, not be cached: {age_s} then {age_s2}"
    );

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn drive_state_endpoint_404s_with_reason_when_absent() {
    let root = temp_dir("drive_state_missing");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/drive_state");
    assert_eq!(resp.status, 404);
    let parsed: Value = serde_json::from_str(&resp.body).expect("404 body is json");
    let reason = parsed["error"].as_str().unwrap();
    assert!(reason.contains("drive_state.json"), "{reason}");

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn path_endpoint_serves_full_resolution_xy_per_step() {
    let (root, run_dirs) = demo_root("full_path");
    let port = spawn_server(root.clone());
    let name = run_dirs[0]
        .file_name()
        .unwrap()
        .to_string_lossy()
        .into_owned();

    let plot = request(port, "GET", &format!("/api/runs/{name}/plot_series"));
    assert_eq!(plot.status, 200);
    let plot: Value = serde_json::from_str(&plot.body).unwrap();

    let resp = request(port, "GET", &format!("/api/runs/{name}/path"));
    assert_eq!(resp.status, 200, "{}", resp.body);
    let payload: Value = serde_json::from_str(&resp.body).unwrap();
    assert_eq!(payload["version"], 1);
    let steps = payload["steps"].as_array().unwrap();
    assert_eq!(steps.len(), plot["steps"].as_array().unwrap().len());

    for (full, preview) in steps.iter().zip(plot["steps"].as_array().unwrap()) {
        assert_eq!(full["name"], preview["name"]);
        assert_eq!(full["truncated"], false);
        let n_records = full["n_records"].as_u64().unwrap() as usize;
        let cap_name = format!("step_{}.scap", full["name"].as_str().unwrap());
        let cap =
            servo_ident::scap::Scap::load(run_dirs[0].join(&cap_name).to_str().unwrap()).unwrap();
        assert_eq!(n_records, cap.n_records);
        let paired = cap.n_records - servo_ident::analyze::TARGET_TO_ACTUAL_SKEW_CYCLES;
        for series in ["cmd_x_mm", "cmd_y_mm", "act_x_mm", "act_y_mm"] {
            let full_len = full["path"][series].as_array().unwrap().len();
            let preview_len = preview["path"][series].as_array().unwrap().len();
            assert_eq!(full_len, paired, "{cap_name} {series}");
            assert!(
                full_len > preview_len,
                "{cap_name} {series}: full {full_len} not denser than preview {preview_len}"
            );
        }
    }

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn path_endpoint_404s_for_a_run_without_a_spatial_frame() {
    let root = temp_dir("path_no_spatial");
    write_bare_manifest(&root.join("flat_run"), "flat");
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs/flat_run/path");
    assert_eq!(resp.status, 404);
    let parsed: Value = serde_json::from_str(&resp.body).expect("404 body is json");
    assert!(parsed["error"].as_str().unwrap().contains("spatial"));

    std::fs::remove_dir_all(&root).ok();
}
