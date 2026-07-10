//! `servo-cal serve`: run-directory listing, raw manifest/results/plot_series
//! serving, `drive_state.json` passthrough for the tuning panel,
//! analyze-on-demand, and the embedded SPA — the HTTP routes bound to
//! `docs/rewrite/servo-cal-contracts.md`'s `serve` section and
//! `docs/rewrite/servo-tuning-profiles.md`'s tuning panel backend. The
//! transport is `crate::http`; this module only ever answers pure functions
//! of the filesystem under `captures_root`.

use std::path::Path;
use std::time::SystemTime;

use serde::{Deserialize, Serialize};

use crate::analyze::{build_run, write_run_outputs};
use crate::assets;
use crate::http::{Request, Response};
use crate::live;
use crate::results::Manifest;
use crate::time_fmt::iso8601_utc;

#[derive(Debug, Serialize, Deserialize)]
pub struct VerdictSummary {
    pub recommended_step: Option<String>,
    pub reason: String,
    pub flags: Vec<String>,
}

#[derive(Deserialize)]
struct ResultsVerdictOnly {
    verdict: VerdictSummary,
}

#[derive(Debug, Serialize)]
pub struct RunSummary {
    pub name: String,
    pub mtime_utc: String,
    pub experiment: String,
    pub tag: String,
    pub axis: Option<String>,
    pub has_results: bool,
    pub verdict: Option<VerdictSummary>,
}

fn valid_run_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

fn mtime(path: &Path) -> Result<SystemTime, String> {
    std::fs::metadata(path)
        .and_then(|m| m.modified())
        .map_err(|e| format!("stat {}: {e}", path.display()))
}

/// Scan `captures_root` for directories holding `manifest.json`, newest
/// (by manifest mtime) first.
pub fn list_runs(captures_root: &Path) -> Result<Vec<RunSummary>, String> {
    let entries = std::fs::read_dir(captures_root)
        .map_err(|e| format!("read {}: {e}", captures_root.display()))?;
    let mut runs = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|e| format!("{}: {e}", captures_root.display()))?;
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let manifest_path = path.join("manifest.json");
        if !manifest_path.is_file() {
            continue;
        }
        let name = entry.file_name().to_string_lossy().into_owned();
        let text = std::fs::read_to_string(&manifest_path)
            .map_err(|e| format!("read {}: {e}", manifest_path.display()))?;
        let manifest: Manifest = serde_json::from_str(&text)
            .map_err(|e| format!("{}: manifest parse: {e}", manifest_path.display()))?;
        let results_path = path.join("results.json");
        let has_results = results_path.is_file();
        let verdict = if has_results {
            let rtext = std::fs::read_to_string(&results_path)
                .map_err(|e| format!("read {}: {e}", results_path.display()))?;
            let parsed: ResultsVerdictOnly = serde_json::from_str(&rtext)
                .map_err(|e| format!("{}: results parse: {e}", results_path.display()))?;
            Some(parsed.verdict)
        } else {
            None
        };
        let manifest_mtime = mtime(&manifest_path)?;
        runs.push((
            manifest_mtime,
            RunSummary {
                name,
                mtime_utc: iso8601_utc(manifest_mtime),
                experiment: manifest.experiment,
                tag: manifest.tag,
                axis: manifest.axis,
                has_results,
                verdict,
            },
        ));
    }
    // Sort on the raw `SystemTime`, not the whole-second `mtime_utc` string —
    // a sweep completes in tens of seconds, so successive runs in a fast
    // demo or test can land in the same displayed second; only the
    // sub-second `SystemTime` still orders them correctly. Name is a last,
    // deterministic tiebreak for genuine same-instant ties.
    runs.sort_by(|(a_mtime, a), (b_mtime, b)| {
        b_mtime.cmp(a_mtime).then_with(|| b.name.cmp(&a.name))
    });
    Ok(runs.into_iter().map(|(_, r)| r).collect())
}

fn newest_input_mtime(run_dir: &Path) -> Result<SystemTime, String> {
    let mut newest = SystemTime::UNIX_EPOCH;
    for entry in
        std::fs::read_dir(run_dir).map_err(|e| format!("read {}: {e}", run_dir.display()))?
    {
        let entry = entry.map_err(|e| format!("{}: {e}", run_dir.display()))?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name == "results.json" || name == "plot_series.json" {
            continue;
        }
        let m = mtime(&entry.path())?;
        if m > newest {
            newest = m;
        }
    }
    Ok(newest)
}

fn needs_analyze(run_dir: &Path) -> Result<bool, String> {
    let results_path = run_dir.join("results.json");
    if !results_path.is_file() {
        return Ok(true);
    }
    Ok(newest_input_mtime(run_dir)? > mtime(&results_path)?)
}

fn read_run_file(captures_root: &Path, name: &str, file: &str) -> Response {
    if !valid_run_name(name) {
        return Response::not_found(&format!("invalid run name {name:?}"));
    }
    let path = captures_root.join(name).join(file);
    match std::fs::read_to_string(&path) {
        Ok(text) => Response::json(200, text),
        Err(e) => Response::not_found(&format!("{}: {e}", path.display())),
    }
}

fn handle_list(captures_root: &Path) -> Response {
    match list_runs(captures_root) {
        Ok(runs) => Response::json(
            200,
            serde_json::to_string(&runs).expect("RunSummary always serializes"),
        ),
        Err(e) => Response::text(500, "text/plain", e),
    }
}

/// Age of `mtime` relative to now, in seconds. `SystemTime` arithmetic can
/// only fail when `mtime` is in the future (clock skew between writer and
/// server); that reads as zero age rather than a negative one.
fn age_seconds(mtime: SystemTime) -> f64 {
    SystemTime::now()
        .duration_since(mtime)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// `GET /api/drive_state`: the tuning panel's only data source. Serves
/// `<captures_root>/drive_state.json` verbatim except for one added
/// top-level field, `age_s` — seconds since the file's mtime, computed
/// fresh on every request (never cached) so the dashboard's staleness
/// banner reflects reality even when the file itself hasn't changed since
/// the last SERVO_DUMP_TUNING. 404 with a reason when the file is absent —
/// SERVO_DUMP_TUNING has never run against this captures_root.
fn handle_drive_state(captures_root: &Path) -> Response {
    let path = captures_root.join("drive_state.json");
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) => {
            return Response::not_found(&format!(
                "{}: {e} (run SERVO_DUMP_TUNING first)",
                path.display()
            ))
        }
    };
    let file_mtime = match mtime(&path) {
        Ok(m) => m,
        Err(e) => return Response::text(500, "text/plain", e),
    };
    let mut value: serde_json::Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => {
            return Response::text(500, "text/plain", format!("{}: parse: {e}", path.display()))
        }
    };
    let Some(obj) = value.as_object_mut() else {
        return Response::text(
            500,
            "text/plain",
            format!(
                "{}: expected a JSON object at the top level",
                path.display()
            ),
        );
    };
    obj.insert(
        "age_s".to_string(),
        serde_json::json!(age_seconds(file_mtime)),
    );
    Response::json(200, value.to_string())
}

fn handle_analyze(captures_root: &Path, name: &str) -> Response {
    if !valid_run_name(name) {
        return Response::not_found(&format!("invalid run name {name:?}"));
    }
    let run_dir = captures_root.join(name);
    if !run_dir.join("manifest.json").is_file() {
        return Response::not_found(&format!("no such run {name:?}"));
    }
    let stale = match needs_analyze(&run_dir) {
        Ok(s) => s,
        Err(e) => return Response::text(500, "text/plain", e),
    };
    if !stale {
        return read_run_file(captures_root, name, "results.json");
    }
    let (results, plot) = match build_run(&run_dir) {
        Ok(rp) => rp,
        Err(e) => return Response::text(500, "text/plain", e),
    };
    if let Err(e) = write_run_outputs(&run_dir, &results, &plot) {
        return Response::text(500, "text/plain", e);
    }
    Response::json(
        200,
        serde_json::to_string(&results).expect("Results always serializes"),
    )
}

/// Newest flat capture in the root, with its current size and age — the
/// live page polls this to notice a capture starting or growing.
fn handle_live_status(captures_root: &Path) -> Response {
    let newest = match live::newest_flat_scap(captures_root) {
        Ok(n) => n,
        Err(e) => return Response::text(500, "text/plain", e),
    };
    let Some(path) = newest else {
        return Response::json(200, serde_json::json!({ "capture": null }).to_string());
    };
    let meta = match path.metadata() {
        Ok(m) => m,
        Err(e) => return Response::text(500, "text/plain", format!("{}: {e}", path.display())),
    };
    let mtime = meta.modified().ok();
    Response::json(
        200,
        serde_json::json!({
            "capture": {
                "name": path.file_name().and_then(|n| n.to_str()),
                "size_bytes": meta.len(),
                "age_s": mtime.map(age_seconds),
            }
        })
        .to_string(),
    )
}

fn query_param(raw_path: &str, key: &str) -> Option<String> {
    let query = raw_path.split_once('?')?.1;
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(k, _)| *k == key)
        .map(|(_, v)| v.to_string())
}

fn handle_live_tail(captures_root: &Path, name: &str, raw_path: &str) -> Response {
    if !live::valid_capture_name(name) {
        return Response::not_found(&format!("invalid capture name {name:?}"));
    }
    let path = captures_root.join(name);
    if !path.is_file() {
        return Response::not_found(&format!("no such capture {name:?}"));
    }
    let offset: u64 = match query_param(raw_path, "offset").as_deref() {
        None => 0,
        Some(text) => match text.parse() {
            Ok(v) => v,
            Err(_) => {
                return Response::text(400, "text/plain", format!("bad offset {text:?}"));
            }
        },
    };
    match live::tail_scap(&path, offset) {
        Ok(payload) => Response::json(200, payload.to_string()),
        Err(e) => Response::text(500, "text/plain", e),
    }
}

/// Route one HTTP request against `captures_root`. Pure given the
/// filesystem state — no globals, easy to drive from tests without a
/// socket.
pub fn handle(captures_root: &Path, req: &Request) -> Response {
    let path = req.path.split('?').next().unwrap_or("");
    let segments: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
    match (req.method.as_str(), segments.as_slice()) {
        ("GET", []) => Response::text(
            200,
            "text/html; charset=utf-8",
            assets::INDEX_HTML.to_string(),
        ),
        ("GET", ["app.js"]) => {
            Response::text(200, "application/javascript", assets::APP_JS.to_string())
        }
        ("GET", ["app.css"]) => Response::text(200, "text/css", assets::APP_CSS.to_string()),
        ("GET", ["api", "runs"]) => handle_list(captures_root),
        ("GET", ["api", "drive_state"]) => handle_drive_state(captures_root),
        ("GET", ["api", "live"]) => handle_live_status(captures_root),
        ("GET", ["api", "live", name]) => handle_live_tail(captures_root, name, &req.path),
        ("GET", ["api", "runs", name, "manifest"]) => {
            read_run_file(captures_root, name, "manifest.json")
        }
        ("GET", ["api", "runs", name, "results"]) => {
            read_run_file(captures_root, name, "results.json")
        }
        ("GET", ["api", "runs", name, "plot_series"]) => {
            read_run_file(captures_root, name, "plot_series.json")
        }
        ("POST", ["api", "runs", name, "analyze"]) => handle_analyze(captures_root, name),
        _ => Response::not_found(&format!("no such route: {} {}", req.method, req.path)),
    }
}
