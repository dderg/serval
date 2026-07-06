//! Host harness for the MCU `piece_sink` parser (`src/piece_sink.c`).
//!
//! The build script compiles the real parser on the host with its seam faked by
//! `csrc/harness_stub.c`. The tests then check it three ways: behavioural
//! invariants, a differential round-trip against the Rust `mcu-protocol` codec
//! (host encoder → C parser → host decoder must agree), and a byte-stream fuzz.
//! The AddressSanitizer memory gate (out-of-bounds detection over millions of
//! random frames) is a standalone clang build: `scripts/fuzz-piece-sink.sh`.
//!
//! Everything is test-only; the crate has no production surface.

#[cfg(test)]
mod tests;
