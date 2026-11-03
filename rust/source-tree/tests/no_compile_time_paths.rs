use std::fs;
use std::path::Path;

const FORBIDDEN: &str = concat!("env!(\"CARGO_", "MANIFEST_DIR\")");

fn rust_sources(dir: &Path, found: &mut Vec<std::path::PathBuf>) {
    for entry in fs::read_dir(dir).expect("read source dir") {
        let path = entry.expect("dir entry").path();
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
        if path.is_dir() {
            if name != "target" && !name.starts_with('.') {
                rust_sources(&path, found);
            }
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            found.push(path);
        }
    }
}

#[test]
fn no_rust_source_bakes_the_manifest_dir_in_at_compile_time() {
    let mut sources = Vec::new();
    rust_sources(&source_tree::repo_root().join("rust"), &mut sources);
    assert!(sources.len() > 100, "scanned only {} files", sources.len());
    let offenders: Vec<_> = sources
        .iter()
        .filter(|path| fs::read_to_string(path).is_ok_and(|text| text.contains(FORBIDDEN)))
        .collect();
    assert!(
        offenders.is_empty(),
        "a test binary reused from another worktree would read that worktree's files; \
         use source_tree::manifest_dir()/repo_file() instead: {offenders:?}"
    );
}
