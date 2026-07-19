//! Exports the served wire structs to TypeScript via ts-rs and guards the
//! committed bindings in `web/src/generated/` for freshness: the test
//! rewrites the directory from the current Rust types and fails if that
//! changed anything on disk — commit the regenerated files to go green.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use ts_rs::{Config, TS};

use servo_ident::demo::DriveStatePayload;
use servo_ident::results::{PlotSeries, Results};
use servo_ident::serve::{DeleteResponse, LiveStatus, NoteResponse, RunPath, RunSummary};
use servo_ident::strain::StrainMap;

fn export_all(out_dir: &Path) {
    let cfg = Config::new().with_out_dir(out_dir).with_large_int("number");
    Results::export_all(&cfg).expect("export Results");
    PlotSeries::export_all(&cfg).expect("export PlotSeries");
    RunSummary::export_all(&cfg).expect("export RunSummary");
    NoteResponse::export_all(&cfg).expect("export NoteResponse");
    DeleteResponse::export_all(&cfg).expect("export DeleteResponse");
    LiveStatus::export_all(&cfg).expect("export LiveStatus");
    RunPath::export_all(&cfg).expect("export RunPath");
    DriveStatePayload::export_all(&cfg).expect("export DriveStatePayload");
    StrainMap::export_all(&cfg).expect("export StrainMap");
}

fn collect_files(root: &Path, dir: &Path, files: &mut BTreeMap<String, String>) {
    let entries = std::fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
    for entry in entries {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            collect_files(root, &path, files);
            continue;
        }
        let rel = path
            .strip_prefix(root)
            .unwrap()
            .to_string_lossy()
            .into_owned();
        let content = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        files.insert(rel, content);
    }
}

fn read_dir_files(dir: &Path) -> BTreeMap<String, String> {
    let mut files = BTreeMap::new();
    collect_files(dir, dir, &mut files);
    files
}

#[test]
fn committed_ts_bindings_match_rust_wire_structs() {
    let generated_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web/src/generated");
    let committed = read_dir_files(&generated_dir);

    let scratch = std::env::temp_dir().join(format!(
        "servo_ident_ts_bindings_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&scratch).unwrap();
    export_all(&scratch);
    let fresh = read_dir_files(&scratch);
    std::fs::remove_dir_all(&scratch).ok();

    if committed == fresh {
        return;
    }

    std::fs::remove_dir_all(&generated_dir).unwrap();
    std::fs::create_dir_all(&generated_dir).unwrap();
    for (name, content) in &fresh {
        let path = generated_dir.join(name);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, content).unwrap();
    }

    let mut diffs = Vec::new();
    for name in committed.keys() {
        if !fresh.contains_key(name) {
            diffs.push(format!("stale file removed: {name}"));
        }
    }
    for (name, content) in &fresh {
        match committed.get(name) {
            None => diffs.push(format!("new file: {name}")),
            Some(old) if old != content => diffs.push(format!("changed: {name}")),
            Some(_) => {}
        }
    }
    panic!(
        "web/src/generated/ was stale; it has been regenerated from the Rust \
         wire structs — review and commit the changes:\n{}",
        diffs.join("\n")
    );
}
