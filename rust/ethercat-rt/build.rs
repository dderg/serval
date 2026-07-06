use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=IGH_DIR");
    println!("cargo:rerun-if-env-changed=IGH_LIB_DIR");
    println!("cargo:rerun-if-changed=build.rs");

    // The EtherCAT master FFI is compiled and linked only under the `hw`
    // feature. Without it (the default), the crate is pure Rust — so
    // `cargo test`/`cargo build` run the scale/wire/curves unit tests on any
    // machine, and in CI, without a master library installed. `hw` compiles the
    // IgH backend (csrc/libecrt_igh.c) and links libethercat.
    if env::var_os("CARGO_FEATURE_HW").is_none() {
        return;
    }

    build_igh();
}

fn build_igh() {
    let igh_dir =
        PathBuf::from(env::var("IGH_DIR").unwrap_or_else(|_| "/opt/etherlab".to_string()));
    let igh_lib_dir = env::var("IGH_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| igh_dir.join("lib"));

    println!("cargo:rerun-if-changed=csrc/libecrt_igh.c");
    println!("cargo:rerun-if-changed=csrc/libecrt.h");

    cc::Build::new()
        .file("csrc/libecrt_igh.c")
        .include("csrc")
        .include(igh_dir.join("include"))
        .opt_level(2)
        .flag("-Wall")
        .compile("ecrt_igh");

    println!("cargo:rustc-link-search=native={}", igh_lib_dir.display());
    println!("cargo:rustc-link-lib=ethercat");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=m");
}
