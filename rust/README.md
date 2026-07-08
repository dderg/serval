# Kalico Rust Workspace

First-party Rust code for the kalico motion stack rewrite.

## Layout

- `nurbs/` — Layer 0 mathematical foundations (NURBS eval, arc-length, algebra); host-only, f64.
- `c-api/` — umbrella staticlib + cbindgen FFI surface for kalico's Rust runtime. cbindgen-generated header at `c-api/include/runtime.h` (checked in).

## Build

Host (default — for tests, linting, host-side use):

    cargo build
    cargo test

MCU (H723 = Cortex-M7 with double-precision FPU):

    cargo build --release --no-default-features --features mcu-h7 --target thumbv7em-none-eabi

The Klipper Make build picks up the resulting staticlib at `target/thumbv7em-none-eabi/release/libc_api.a` and the C header at `c-api/include/runtime.h`.

## Toolchain

Pinned via `rust-toolchain.toml`. Update intentionally with regression testing — embedded codegen is sensitive to compiler version. FPU flag strings in `.cargo/config.toml` may need to track LLVM target-feature renames across toolchain versions; verify on bumps.

## C link contract

- C side `#include`s `c-api/include/runtime.h` (committed; CI verifies regen is a no-op).
- C side links against `libc_api.a`.
- Type ownership: C never frees Rust-allocated memory; constructors/destructors come in pairs across the FFI boundary. Pointer types are opaque to C.
