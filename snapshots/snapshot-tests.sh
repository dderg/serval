#!/usr/bin/env bash
set -o pipefail

make -f Makefile.rust motion-engine

# Install wasm-pack and wasm32 target if not present
if ! rustup target list --installed 2>/dev/null | grep -q wasm32-unknown-unknown; then
  echo "installing wasm32 target..."
  rustup target add wasm32-unknown-unknown 2>&1
fi
if ! command -v wasm-pack &>/dev/null && ! [ -x "$HOME/.cargo/bin/wasm-pack" ]; then
  echo "installing wasm-pack..."
  cargo install wasm-pack 2>&1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNAPSHOT_VIEWER="$SCRIPT_DIR/../rust/snapshot-viewer"
WASM_OUT="$SCRIPT_DIR/web/static/wasm"

WP="${WASM_PACK:-wasm-pack}"
command -v "$WP" &>/dev/null || WP="$HOME/.cargo/bin/wasm-pack"

# Build the WASM interactive viewer if output is missing or source is newer
if [ ! -f "$WASM_OUT/snapshot_viewer_bg.wasm" ] || \
   [ "$SNAPSHOT_VIEWER/src/lib.rs" -nt "$WASM_OUT/snapshot_viewer_bg.wasm" ]; then
  echo "building snapshot-viewer WASM..."
  "$WP" build --target web --release --out-dir "$WASM_OUT" "$SNAPSHOT_VIEWER" 2>&1
fi
PORT="${SNAPSHOT_PORT:-8765}"
PYTHON="${PYTHON:-python3}"

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
  exec "$PYTHON" "$SCRIPT_DIR/web/server.py" --mode baselines --port "$PORT"
fi

if [ "$ci" = 1 ]; then
  exec "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
fi

if "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"; then
  exit 0
fi

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
