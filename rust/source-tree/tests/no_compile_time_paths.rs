use std::fs;

const FORBIDDEN: &str = concat!("env!(\"CARGO_", "MANIFEST_DIR\")");
const MIN_SOURCES_SCANNED: usize = 100;

#[test]
fn no_rust_source_bakes_the_manifest_dir_in_at_compile_time() {
    let sources = source_tree::rust_sources();
    assert!(
        sources.len() > MIN_SOURCES_SCANNED,
        "scanned only {} files",
        sources.len()
    );
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
