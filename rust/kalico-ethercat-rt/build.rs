use std::env;

fn main() {
    // Directory holding libecrt.a (built by bench/Makefile on the target host).
    // Override with ECRT_LIB_DIR; default to the repo's bench/ directory
    // relative to this crate's manifest.
    let lib_dir = env::var("ECRT_LIB_DIR")
        .unwrap_or_else(|_| format!("{}/../../bench", env!("CARGO_MANIFEST_DIR")));
    println!("cargo:rustc-link-search=native={lib_dir}");
    println!("cargo:rustc-link-lib=static=ecrt");

    // SOEM static lib. Override SOEM_LIB_DIR to point at the directory
    // containing libsoem.a (typically ~/ethercat/SOEM/build on the Pi).
    if let Ok(soem_dir) = env::var("SOEM_LIB_DIR") {
        println!("cargo:rustc-link-search=native={soem_dir}");
    }
    println!("cargo:rustc-link-lib=static=soem");

    // Platform libs required by SOEM and the RT scheduling calls in libecrt.c.
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=m");

    // Rebuild if the C shim sources change.
    println!("cargo:rerun-if-changed=../../bench/libecrt.c");
    println!("cargo:rerun-if-changed=../../bench/libecrt.h");
}
