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
