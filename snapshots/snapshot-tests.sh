#!/usr/bin/env bash
set -o pipefail

make -f Makefile.rust motion-engine-fast

# Ensure cargo/rustup are on PATH (macOS Homebrew or manual installs may not add ~/.cargo/bin)
export PATH="$HOME/.cargo/bin:$PATH"

# Force wasm-pack to use the rustup-managed toolchain (avoids macOS Homebrew/system Rust conflicts)
if command -v rustup &>/dev/null; then
  export RUSTUP_TOOLCHAIN="$(rustup show active-toolchain | awk '{print $1}')"
fi

# Install wasm-pack and wasm32 target if not present
if ! rustup target list --installed 2>/dev/null | grep -q wasm32-unknown-unknown; then
  if command -v rustup &>/dev/null; then
    echo "installing wasm32 target..."
    rustup target add wasm32-unknown-unknown 2>&1 || {
      echo "error: failed to install wasm32-unknown-unknown target" >&2
      echo "run manually: rustup target add wasm32-unknown-unknown" >&2
      exit 1
    }
  else
    echo "error: rustup not found — install Rust via https://rustup.rs" >&2
    exit 1
  fi
fi
# Double-check: rustc must actually know the target (guards against PATH mismatches)
if ! rustc --print target-list 2>/dev/null | grep -q wasm32-unknown-unknown; then
  echo "error: wasm32-unknown-unknown target not available to rustc" >&2
  echo "ensure rustup manages your Rust install: https://rustup.rs" >&2
  exit 1
fi
if ! command -v wasm-pack &>/dev/null && ! [ -x "$HOME/.cargo/bin/wasm-pack" ]; then
  echo "installing wasm-pack..."
  cargo install wasm-pack 2>&1 || {
    echo "error: failed to install wasm-pack" >&2
    echo "run manually: cargo install wasm-pack" >&2
    exit 1
  }
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

# Build the playground WASM (the pipeline itself, for /playground) if stale.
# It links half the workspace, so compare against every source it pulls in.
MOTION_PLAYGROUND="$SCRIPT_DIR/../rust/motion-playground"
PLAYGROUND_OUT="$SCRIPT_DIR/web/static/wasm-playground"
playground_stale() {
  [ ! -f "$PLAYGROUND_OUT/motion_playground_bg.wasm" ] && return 0
  local newer
  newer=$(find "$SCRIPT_DIR/../rust" \
    \( -path "*/motion-playground/src/*" -o -path "*/pipeline-snapshot/src/*" \
       -o -path "*/motion-pipeline/src/*" -o -path "*/geometry/src/*" \
       -o -path "*/trajectory/src/*" -o -path "*/nurbs/src/*" \
       -o -path "*/gcode/src/*" \) \
    -name "*.rs" -newer "$PLAYGROUND_OUT/motion_playground_bg.wasm" -print -quit)
  [ -n "$newer" ]
}
if playground_stale; then
  echo "building motion-playground WASM..."
  "$WP" build --target web --release --out-dir "$PLAYGROUND_OUT" "$MOTION_PLAYGROUND" 2>&1
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

RESULTS_DIR="$(mktemp -d -t snapshot-results)"
trap 'rm -rf "$RESULTS_DIR"' EXIT

if "$PYTHON" "$SCRIPT_DIR/run.py" --results-dir "$RESULTS_DIR" "${run_args[@]}"; then
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
"$PYTHON" "$SCRIPT_DIR/web/server.py" --port "$PORT" --results "$RESULTS_DIR"
trap - INT

echo
echo "re-checking snapshots after review..."
"$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
exit $?
