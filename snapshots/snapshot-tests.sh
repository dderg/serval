#!/usr/bin/env bash
set -o pipefail

make -f Makefile.rust motion-engine

# Ensure cargo/rustup are on PATH (macOS Homebrew or manual installs may not add ~/.cargo/bin)
export PATH="$HOME/.cargo/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNAPSHOT_VIEWER="$SCRIPT_DIR/../rust/snapshot-viewer"
WASM_OUT="$SCRIPT_DIR/web/static/wasm"
PORT="${SNAPSHOT_PORT:-8765}"
PYTHON="${PYTHON:-python3}"

# The WASM viewer is only served by web/server.py (--view and the interactive
# review path). --ci never serves it, so building it there would compile
# wasm-pack and the viewer just to throw both away. Build lazily, on demand.
ensure_viewer_built() {
  if ! command -v rustup &>/dev/null; then
    echo "error: rustup not found — install Rust via https://rustup.rs" >&2
    exit 1
  fi
  # wasm-pack builds the viewer with the toolchain pinned by rust/rust-toolchain.toml,
  # not the repo-root default — install the wasm32 target for *that* toolchain.
  if ! (cd "$SNAPSHOT_VIEWER" && rustup target list --installed 2>/dev/null | grep -q wasm32-unknown-unknown); then
    echo "installing wasm32 target for the viewer toolchain..."
    (cd "$SNAPSHOT_VIEWER" && rustup target add wasm32-unknown-unknown 2>&1) || {
      echo "error: failed to install wasm32-unknown-unknown target" >&2
      echo "run manually: (cd $SNAPSHOT_VIEWER && rustup target add wasm32-unknown-unknown)" >&2
      exit 1
    }
  fi
  if ! command -v wasm-pack &>/dev/null && ! [ -x "$HOME/.cargo/bin/wasm-pack" ]; then
    echo "installing wasm-pack..."
    cargo install wasm-pack 2>&1 || {
      echo "error: failed to install wasm-pack" >&2
      echo "run manually: cargo install wasm-pack" >&2
      exit 1
    }
  fi

  local wp="${WASM_PACK:-wasm-pack}"
  command -v "$wp" &>/dev/null || wp="$HOME/.cargo/bin/wasm-pack"

  if [ ! -f "$WASM_OUT/snapshot_viewer_bg.wasm" ] || \
     [ "$SNAPSHOT_VIEWER/src/lib.rs" -nt "$WASM_OUT/snapshot_viewer_bg.wasm" ]; then
    echo "building snapshot-viewer WASM..."
    "$wp" build --target web --release --out-dir "$WASM_OUT" "$SNAPSHOT_VIEWER" 2>&1
  fi
}

ci=0
view_baselines=0
run_args=()
for arg in "$@"; do
  if [ "$arg" = "--ci" ]; then
    ci=1
  elif [ "$arg" = "--view" ]; then
    view_baselines=1
  else
    run_args+=("$arg")
  fi
done

if [ "$ci" = 1 ] && [ "$view_baselines" = 1 ]; then
  echo "--ci and --view cannot be used together" >&2
  exit 2
fi

if [ "$view_baselines" = 1 ]; then
  ensure_viewer_built
  exec "$PYTHON" "$SCRIPT_DIR/web/server.py" --mode baselines --port "$PORT"
fi

if [ "$ci" = 1 ]; then
  exec "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
fi

if "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"; then
  exit 0
fi

ensure_viewer_built

url="http://127.0.0.1:$PORT"
cat <<EOF

==================================================================
 Snapshots changed — a human needs to approve them.
 Review and accept at:  $url
==================================================================
EOF

trap 'exit 130' INT
"$PYTHON" "$SCRIPT_DIR/web/server.py" --port "$PORT"
trap - INT

echo
echo "re-checking snapshots after review..."
exec "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
