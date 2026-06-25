use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=SOEM_DIR");
    println!("cargo:rerun-if-env-changed=SOEM_LIB_DIR");
    println!("cargo:rerun-if-env-changed=IGH_DIR");
    println!("cargo:rerun-if-env-changed=IGH_LIB_DIR");
    println!("cargo:rerun-if-changed=build.rs");

    // The EtherCAT master FFI is compiled and linked only under the `hw`
    // feature. Without it (the default), the crate is pure Rust — so
    // `cargo test`/`cargo build` run the scale/wire/curves unit tests on any
    // machine, and in CI, without a master library installed. The backend is a
    // compile-time choice over the same ec_rt_* surface: `igh` (implies `hw`)
    // selects the IgH skeleton + libethercat; plain `hw` selects SOEM.
    if env::var_os("CARGO_FEATURE_HW").is_none() {
        return;
    }

    if env::var_os("CARGO_FEATURE_IGH").is_some() {
        build_igh();
        return;
    }

    build_soem();
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

fn build_soem() {
    let soem_dir = PathBuf::from(env::var("SOEM_DIR").unwrap_or_else(|_| {
        format!(
            "{}/ethercat/SOEM",
            env::var("HOME").expect("HOME must be set")
        )
    }));
    let soem_lib_dir = env::var("SOEM_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| soem_dir.join("build"));

    // Mirrors the include flags the standalone bench Makefile used to pass.
    let soem_includes = ["soem", "osal", "osal/linux", "oshw/linux", "oshw"];

    println!("cargo:rerun-if-changed=csrc/libecrt.c");
    println!("cargo:rerun-if-changed=csrc/libecrt.h");

    let mut build = cc::Build::new();
    build
        .file("csrc/libecrt.c") // self-#defines _GNU_SOURCE on line 1, before any include
        .include("csrc")
        .opt_level(2) // match the bench-proven Makefile (-O2), not cc's release -O3
        .flag("-Wall");
    for inc in soem_includes {
        build.include(soem_dir.join(inc));
    }
    build.compile("ecrt");

    println!("cargo:rustc-link-search=native={}", soem_lib_dir.display());
    println!("cargo:rustc-link-lib=static=soem");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=m");
}
