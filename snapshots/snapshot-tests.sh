#!/usr/bin/env bash
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
