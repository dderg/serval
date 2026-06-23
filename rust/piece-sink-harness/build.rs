// Compile the real MCU parser (src/piece_sink.c) on the host, alongside a stub
// that fakes its seam (runtime FFI, timer, transport) and records what it did,
// so the Rust tests can drive and inspect it. The AddressSanitizer memory gate
// is a standalone clang build — see scripts/fuzz-piece-sink.sh — because the
// stable Rust toolchain does not link the ASan runtime into a test binary.
fn main() {
    cc::Build::new()
        .file("../../src/piece_sink.c")
        .file("csrc/harness_stub.c")
        .include("../../src")
        .include("../c-api/include")
        .warnings(true)
        .flag_if_supported("-std=c11")
        .compile("piece_sink_harness");

    println!("cargo:rerun-if-changed=csrc/harness_stub.c");
    println!("cargo:rerun-if-changed=../../src/piece_sink.c");
    println!("cargo:rerun-if-changed=../../src/mcu_transport_dispatch.h");
}
