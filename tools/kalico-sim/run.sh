#!/bin/bash
# Kalico Simulator — build and run in Docker.
#
# Usage:
#   ./run.sh                          # Test current working tree (HEAD)
#   ./run.sh --branch sota-motion     # Test a specific branch
#   ./run.sh --gcode benchy.gcode     # Print a G-code file
#   ./run.sh --privileged             # Enable SCHED_FIFO for homing
#   ./run.sh --no-cache               # Force a full rebuild

set -euo pipefail

# BuildKit is required for the Dockerfile's cache mounts (--mount=type=cache).
export DOCKER_BUILDKIT=1

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GIT_DIR="$(cd "$REPO_ROOT" && git rev-parse --git-common-dir 2>/dev/null || echo "$REPO_ROOT/.git")"
MAIN_REPO="$(cd "$GIT_DIR/.." 2>/dev/null && pwd || echo "$REPO_ROOT")"

BRANCH=""
GCODE=""
EXTRA_ARGS=""
DOCKER_ARGS="--rm"
DOCKER_BUILD_ARGS=""
TAG_SUFFIX=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --branch|-b)
            BRANCH="$2"
            TAG_SUFFIX="-${2//\//-}"
            shift 2
            ;;
        --gcode|-g)
            GCODE="$2"
            shift 2
            ;;
        --privileged)
            DOCKER_ARGS="$DOCKER_ARGS --privileged"
            shift
            ;;
        --no-cache)
            DOCKER_BUILD_ARGS="$DOCKER_BUILD_ARGS --no-cache"
            shift
            ;;
        --verbose|-v)
            EXTRA_ARGS="$EXTRA_ARGS --verbose"
            shift
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

IMAGE_TAG="kalico-sim${TAG_SUFFIX}"

# Cache key partitions the BuildKit compile caches (Rust target/, per-MCU
# firmware OUT dirs) by branch, so two different branches building in parallel
# get independent caches instead of serializing or clobbering each other.
CACHE_KEY="${BRANCH:-$(cd "$REPO_ROOT" && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo head)}"
CACHE_KEY="${CACHE_KEY//\//-}"
DOCKER_BUILD_ARGS="$DOCKER_BUILD_ARGS --build-arg SIM_CACHE_KEY=${CACHE_KEY}"

echo "=== Kalico Simulator ==="
echo "  Branch:    ${BRANCH:-HEAD}"
echo "  Image:     $IMAGE_TAG"
echo "  Cache key: $CACHE_KEY"
echo "  Main repo: $MAIN_REPO"
echo "  G-code:    ${GCODE:-none (basic test)}"
echo ""

if [[ -n "$BRANCH" ]]; then
    # Extract the branch into a unique, self-cleaning staging dir. A unique
    # path per invocation means concurrent builds (same or different branch)
    # never race on a shared context dir. BuildKit keys its layer cache on
    # file *content* hashes, not the context path, so a fresh temp dir does
    # not defeat caching.
    BUILD_CTX="$(mktemp -d "${TMPDIR:-/tmp}/kalico-sim-ctx.XXXXXX")"
    trap 'rm -rf "$BUILD_CTX"' EXIT
    echo "Extracting branch '$BRANCH' to $BUILD_CTX ..."
    (cd "$MAIN_REPO" && git archive "$BRANCH") | tar -x -C "$BUILD_CTX"
    # Overlay current simulator tools from the worktree so local edits to
    # run.sh / Dockerfile / configs are tested without committing.
    mkdir -p "$BUILD_CTX/tools/kalico-sim"
    cp -a "$SCRIPT_DIR"/. "$BUILD_CTX/tools/kalico-sim/"
    echo "Building Docker image '$IMAGE_TAG' from $BUILD_CTX ..."
    # shellcheck disable=SC2086
    docker build \
        $DOCKER_BUILD_ARGS \
        -t "$IMAGE_TAG" \
        -f "$BUILD_CTX/tools/kalico-sim/Dockerfile" \
        "$BUILD_CTX"
else
    echo "Building Docker image '$IMAGE_TAG' from repo root ..."
    # shellcheck disable=SC2086
    docker build \
        $DOCKER_BUILD_ARGS \
        -t "$IMAGE_TAG" \
        -f "$SCRIPT_DIR/Dockerfile" \
        "$REPO_ROOT"
fi

if [[ -n "$GCODE" ]]; then
    GCODE_ABS="$(cd "$(dirname "$GCODE")" && pwd)/$(basename "$GCODE")"
    DOCKER_ARGS="$DOCKER_ARGS -v $GCODE_ABS:/gcode/$(basename "$GCODE"):ro"
    EXTRA_ARGS="$EXTRA_ARGS --gcode /gcode/$(basename "$GCODE")"
fi

echo "Running simulation..."
# shellcheck disable=SC2086
docker run $DOCKER_ARGS "$IMAGE_TAG" $EXTRA_ARGS
