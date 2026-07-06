#!/usr/bin/env bash
# AddressSanitizer/UBSan memory gate for the MCU piece_sink parser
# (src/piece_sink.c). Compiles it on the host with its seam stubbed and throws
# millions of random byte streams at it; ASan aborts on any out-of-bounds access.
# Run before flashing firmware that changes the parser. Exits non-zero on a trap.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cc="${CC:-clang}"
out="${TMPDIR:-/tmp}/piece_sink_fuzz"

"$cc" -fsanitize=address,undefined -fno-sanitize-recover=all \
    -O1 -g -std=c11 -Wall -Wextra -Werror \
    -I"$root/src" -I"$root/rust/c-api/include" \
    "$root/src/piece_sink.c" \
    "$root/rust/piece-sink-harness/csrc/harness_stub.c" \
    "$root/rust/piece-sink-harness/csrc/fuzz_main.c" \
    -o "$out"

echo "running piece_sink ASan/UBSan fuzz ..."
"$out"
echo "piece_sink fuzz: clean (no sanitizer trap)"
