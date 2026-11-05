use std::{env, fs, path::PathBuf};

const BUILD_ID_ENV: &str = "CAPTURE_FW_BUILD_ID";

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed={BUILD_ID_ENV}");
    let build_id = env::var(BUILD_ID_ENV).unwrap_or_else(|_| "unknown".to_string());
    println!("cargo:rustc-env={BUILD_ID_ENV}={build_id}");
}
