use source_tree::{manifest_dir, manifest_dir_from, repo_file, repo_root, repo_root_from};
use std::path::{Path, PathBuf};

#[test]
fn the_manifest_dir_is_whatever_the_runtime_environment_says() {
    assert_eq!(
        manifest_dir_from(Some("/elsewhere/rust/nurbs".into())),
        PathBuf::from("/elsewhere/rust/nurbs")
    );
}

#[test]
#[should_panic(expected = "CARGO_MANIFEST_DIR is not set")]
fn a_missing_manifest_dir_is_refused_not_replaced_by_a_guess() {
    manifest_dir_from(None);
}

#[test]
fn the_repo_root_is_two_levels_above_the_crate() {
    assert_eq!(
        repo_root_from(Path::new("/elsewhere/rust/nurbs")),
        PathBuf::from("/elsewhere/rust/nurbs/../..")
    );
}

#[test]
fn the_running_crates_manifest_dir_holds_this_crates_manifest() {
    assert!(manifest_dir().join("Cargo.toml").is_file());
    assert!(repo_root().join("rust/source-tree/Cargo.toml").is_file());
    assert!(repo_file("rust/source-tree/Cargo.toml").is_file());
}
