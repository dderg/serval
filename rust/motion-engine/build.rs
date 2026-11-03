use sha2::{Digest, Sha256};
use std::fmt::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

const ID_HEX_CHARS: usize = 40;
const UNKNOWN: &str = "unknown";

fn repo_root() -> PathBuf {
    let manifest =
        std::env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is set by cargo");
    Path::new(&manifest).join("../..")
}

fn source_listing(root: &Path) -> Option<Vec<Vec<u8>>> {
    let output = Command::new("git")
        .args(["ls-files", "-z", "-co", "--exclude-standard", "--", "rust"])
        .current_dir(root)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let mut names: Vec<Vec<u8>> = output
        .stdout
        .split(|byte| *byte == 0)
        .filter(|name| !name.is_empty())
        .map(<[u8]>::to_vec)
        .collect();
    names.sort();
    names.dedup();
    Some(names)
}

fn tree_build_id(root: &Path) -> String {
    let Some(names) = source_listing(root) else {
        return UNKNOWN.to_string();
    };
    let mut hasher = Sha256::new();
    for name in names {
        let path = root.join(String::from_utf8_lossy(&name).as_ref());
        let Ok(content) = std::fs::read(&path) else {
            continue;
        };
        hasher.update(&name);
        hasher.update([0]);
        hasher.update(content.len().to_string().as_bytes());
        hasher.update([0]);
        hasher.update(&content);
    }
    let mut digest = String::new();
    for byte in hasher.finalize() {
        write!(digest, "{byte:02x}").expect("writing to a String cannot fail");
    }
    digest[..ID_HEX_CHARS].to_string()
}

const ALWAYS_RERUN: &str =
    "cargo:rerun-if-changed=no-such-file-so-the-id-is-recomputed-every-build";

fn main() {
    let root = repo_root();
    println!("{ALWAYS_RERUN}");
    println!(
        "cargo:rustc-env=SERVAL_ENGINE_BUILD_ID={}",
        tree_build_id(&root)
    );
}
