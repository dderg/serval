#!/bin/bash
# Kalico Simulator — build the Docker image and run it.
#
# Usage:
#   ./run.sh                          # self-test print (current tree)
#   ./run.sh --gcode benchy.gcode     # print a G-code file
#   ./run.sh test                     # run the e2e pytest suite
#   ./run.sh test -k probe            # subset of the e2e suite
#   ./run.sh serve                    # long-lived printer for Moonraker
#   ./run.sh shell                    # bash inside the image
#   ./run.sh --branch sota-motion     # build+run a specific branch
#   ./run.sh --no-cache               # force a full rebuild
#   ./run.sh --privileged             # SCHED_FIFO for jitter-sensitive runs

set -euo pipefail

# BuildKit is required for the Dockerfile's cache mounts (--mount=type=cache).
export DOCKER_BUILDKIT=1

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GIT_DIR="$(cd "$REPO_ROOT" && git rev-parse --git-common-dir 2>/dev/null || echo "$REPO_ROOT/.git")"
MAIN_REPO="$(cd "$GIT_DIR/.." 2>/dev/null && pwd || echo "$REPO_ROOT")"

MODE="run"
BRANCH=""
GCODE=""
EXTRA_ARGS=()
DOCKER_ARGS=(--rm)
DOCKER_BUILD_ARGS=()

case "${1:-}" in
    test|serve|shell)
        MODE="$1"
        shift
        ;;
esac

while [[ $# -gt 0 ]]; do
    case $1 in
        --branch|-b)
            BRANCH="$2"
            shift 2
            ;;
        --gcode|-g)
            GCODE="$2"
            shift 2
            ;;
        --privileged)
            DOCKER_ARGS+=(--privileged)
            shift
            ;;
        --no-cache)
            DOCKER_BUILD_ARGS+=(--no-cache)
            shift
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

# Cache key partitions the BuildKit compile caches (Rust target/, per-MCU
# firmware OUT dirs) by branch, so two branches building in parallel get
# independent caches instead of serializing or clobbering each other.
CACHE_KEY="${BRANCH:-$(cd "$REPO_ROOT" && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo head)}"
CACHE_KEY="${CACHE_KEY//\//-}"
DOCKER_BUILD_ARGS+=(--build-arg "SIM_CACHE_KEY=${CACHE_KEY}")

# The image tag is branch-partitioned for the same reason as the cache key:
# a shared "kalico-sim" tag lets a concurrent session on another worktree
# silently retag the image between one session's build and its test run.
IMAGE_TAG="kalico-sim-${CACHE_KEY}"

echo "=== Kalico Simulator ==="
echo "  Mode:      $MODE"
echo "  Branch:    ${BRANCH:-HEAD}"
echo "  Image:     $IMAGE_TAG"
echo "  Cache key: $CACHE_KEY"
echo ""

if [[ -n "$BRANCH" ]]; then
    # Extract the branch into a unique, self-cleaning staging dir. A unique
    # path per invocation means concurrent builds never race on a shared
    # context dir. BuildKit keys its layer cache on file content hashes,
    # not the context path, so a fresh temp dir does not defeat caching.
    BUILD_CTX="$(mktemp -d "${TMPDIR:-/tmp}/kalico-sim-ctx.XXXXXX")"
    trap 'rm -rf "$BUILD_CTX"' EXIT
    echo "Extracting branch '$BRANCH' to $BUILD_CTX ..."
    (cd "$MAIN_REPO" && git archive "$BRANCH") | tar -x -C "$BUILD_CTX"
    # Overlay current simulator tools from the worktree so local edits to
    # run.sh / Dockerfile / configs are tested without committing.
    mkdir -p "$BUILD_CTX/tools/sim"
    cp -a "$SCRIPT_DIR"/. "$BUILD_CTX/tools/sim/"
    docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
        -t "$IMAGE_TAG" \
        -f "$BUILD_CTX/tools/sim/Dockerfile" \
        "$BUILD_CTX"
else
    docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
        -t "$IMAGE_TAG" \
        -f "$SCRIPT_DIR/Dockerfile" \
        "$REPO_ROOT"
fi

case "$MODE" in
    test)
        # The virtual clock is one shared /dev/shm segment per container,
        # so e2e tests run sequentially inside one container. Parallelism
        # comes from separate docker runs (namespaces fully isolate them).
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} --entrypoint python3 "$IMAGE_TAG" \
            -m pytest tools/sim/tests -m needs_elf -v -p no:cacheprovider \
            ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
        ;;
    serve)
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} -i "$IMAGE_TAG" --serve ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
        ;;
    shell)
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} -it --entrypoint bash "$IMAGE_TAG"
        ;;
    run)
        if [[ -n "$GCODE" ]]; then
            GCODE_ABS="$(cd "$(dirname "$GCODE")" && pwd)/$(basename "$GCODE")"
            DOCKER_ARGS+=(-v "$GCODE_ABS:/gcode/$(basename "$GCODE"):ro")
            EXTRA_ARGS+=(--gcode "/gcode/$(basename "$GCODE")")
        fi
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} "$IMAGE_TAG" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
        ;;
esac
