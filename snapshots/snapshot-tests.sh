#!/usr/bin/env bash
#
# Snapshot tests — a standalone test pillar (not pytest, not the py/sim suites).
#
#   snapshots/snapshot-tests.sh            # local: on a change, open the review
#   snapshots/snapshot-tests.sh --ci       # CI: fail like a plain test, no server
#   snapshots/snapshot-tests.sh -k clean   # extra args pass through to run.py
#
# On a change the review web server runs ONLY while this script runs: visit the
# printed URL, Accept all, and the server stops itself; the script re-checks and
# exits with the final status. Nothing is left listening afterward.

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

# Keep going on Ctrl-C so the server stops and we still re-check below.
trap ':' INT
"$PYTHON" "$SCRIPT_DIR/web/server.py" --port "$PORT"
trap - INT

echo
echo "re-checking snapshots after review..."
exec "$PYTHON" "$SCRIPT_DIR/run.py" "${run_args[@]}"
