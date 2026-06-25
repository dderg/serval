#!/usr/bin/env bash
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${SNAPSHOT_PORT:-8765}"
PYTHON="${PYTHON:-python3}"

ci=0
run_args=()
for arg in "$@"; do
  if [ "$arg" = "--ci" ]; then ci=1; else run_args+=("$arg"); fi
done

# CI: a plain pass/fail run — no server, no review.
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

# Ctrl-C bails out entirely: the server stops with it and the script exits,
# nothing accepted, no re-check. Only a clean exit — Accept all, where the
# server shuts itself down — falls through to the re-check below.
trap 'exit 130' INT
"$PYTHON" "$SCRIPT_DIR/web/server.py" --port "$PORT"
trap - INT

echo
echo "re-checking snapshots after review..."
exec "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
