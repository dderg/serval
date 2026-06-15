#!/usr/bin/env bash
# Build + run the klippy-in-loop sim entirely in Docker on macOS.
# Mounts the repo into the container, builds klipper.elf and the engine
# .so, then runs the sim harness. No Pi or remote machine needed.
#
#   ./tools/sim_klippy/run_local.sh "G28 X"
#
# First run takes ~5 min (image build + cargo first-build). Cached after.
set -euo pipefail

REPO="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
IMG="kalico-sim:latest"
CONTAINER_HOME=/work
SCRIPT_ARGS="${*:-G28 X}"

docker build -q -t "$IMG" -f "$REPO/tools/sim_klippy/Dockerfile" "$REPO/tools/sim_klippy" >/dev/null

# --tmpfs /tmp keeps the unix socket and PTY symlinks ephemeral.
docker run --rm -i \
    -v "$REPO":$CONTAINER_HOME \
    -w $CONTAINER_HOME \
    --tmpfs /tmp:exec \
    "$IMG" \
    bash -c "set -e
      cp .config.linux .config
      make olddefconfig >/dev/null
      make -j\$(nproc) 2>&1 | tail -5
      make -f Makefile.rust motion-engine 2>&1 | tail -3
      # Remove any stale misnamed motion_engine.so that shadows motion_engine.py.
      # The correct native module is always _motion_engine.so (built above).
      rm -f klippy/motion_engine.so 2>/dev/null || true
      mkdir -p /work/tools/sim_klippy/.local-logs
      python3 tools/sim_klippy/run.py $SCRIPT_ARGS"
