#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../rust"
cargo run -p c-api --bin gen-headers --no-default-features --features host,header-runtime
echo "runtime.h regenerated."
