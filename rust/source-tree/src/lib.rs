use std::ffi::OsString;
use std::path::{Path, PathBuf};

const MANIFEST_DIR_VAR: &str = "CARGO_MANIFEST_DIR";
const CRATE_DEPTH_BELOW_REPO: &str = "../..";

pub fn manifest_dir_from(var: Option<OsString>) -> PathBuf {
    let dir = var.unwrap_or_else(|| {
        panic!("{MANIFEST_DIR_VAR} is not set: run the test through cargo or nextest")
    });
    PathBuf::from(dir)
}

pub fn manifest_dir() -> PathBuf {
    manifest_dir_from(std::env::var_os(MANIFEST_DIR_VAR))
}

pub fn repo_root_from(manifest_dir: &Path) -> PathBuf {
    manifest_dir.join(CRATE_DEPTH_BELOW_REPO)
}

pub fn repo_root() -> PathBuf {
    repo_root_from(&manifest_dir())
}

pub fn repo_file(relative: &str) -> PathBuf {
    repo_root().join(relative)
}

const SKIPPED_DIRS: [&str; 1] = ["target"];

fn collect_rust_sources(dir: &Path, found: &mut Vec<PathBuf>) {
    let entries =
        std::fs::read_dir(dir).unwrap_or_else(|e| panic!("read source dir {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("dir entry").path();
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or_default();
        if path.is_dir() {
            if !SKIPPED_DIRS.contains(&name) && !name.starts_with('.') {
                collect_rust_sources(&path, found);
            }
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            found.push(path);
        }
    }
}

pub fn rust_sources() -> Vec<PathBuf> {
    let mut sources = Vec::new();
    collect_rust_sources(&repo_root().join("rust"), &mut sources);
    sources.sort();
    sources
}
