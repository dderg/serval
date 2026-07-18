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

# The image tag is branch-partitioned so agents/sessions on different
# worktrees never race on one tag: with a single shared "kalico-sim" tag, a
# concurrent session could silently retag the image between this session's
# build and its test run.
BUILD_BRANCH="${BRANCH:-$(cd "$REPO_ROOT" && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo head)}"
# Detached HEAD (e.g. GitHub Actions' PR checkout) yields the literal string
# "HEAD", which is not a valid docker image name — fall back to the commit.
[[ "$BUILD_BRANCH" == "HEAD" ]] && BUILD_BRANCH="$(cd "$REPO_ROOT" && git rev-parse --short HEAD 2>/dev/null || echo head)"
IMAGE_TAG="kalico-sim-$(echo "${BUILD_BRANCH//\//-}" | tr '[:upper:]' '[:lower:]')"

echo "=== Kalico Simulator ==="
echo "  Mode:      $MODE"
echo "  Branch:    ${BUILD_BRANCH}"
echo "  Image:     $IMAGE_TAG"
echo ""

if [[ -z "$BRANCH" ]]; then
    # BuildKit's local context scan trusts mtimes; on macOS file sharing it
    # has been observed to serve stale file CONTENT for files whose mtimes
    # it thought it knew — the built image then quietly disagrees with the
    # worktree. Bumping every source mtime forces a re-read. Layer caching
    # is unaffected: it keys on content, so untouched-in-content files
    # still hit cache.
    (cd "$REPO_ROOT" && find Makefile pyproject.toml src lib scripts klippy rust tools \
        \( -name target -o -name target-linux -o -name __pycache__ \
           -o -name third_party_repos \) -prune -o -type f -exec touch {} +)
fi

build_image() {
    # One retry absorbs transient registry/network failures (e.g. a cargo
    # crate download aborting mid-unpack on a CI runner). Deterministic
    # build errors replay from the layer cache and fail fast the second
    # time, so real breakage stays loud.
    local ctx="$1" dockerfile="$2"
    if ! docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
            -t "$IMAGE_TAG" -f "$dockerfile" "$ctx"; then
        echo "docker build failed; retrying once (transient network flakes)" >&2
        docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
            -t "$IMAGE_TAG" -f "$dockerfile" "$ctx"
    fi
}

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
    build_image "$BUILD_CTX" "$BUILD_CTX/tools/sim/Dockerfile"
else
    build_image "$REPO_ROOT" "$SCRIPT_DIR/Dockerfile"
fi

case "$MODE" in
    test)
        # Sequential by default: klippy runs on the real clock, so CPU
        # contention from concurrent worlds stutters its timing budgets
        # into flakes (anchor underruns, missed trip windows). Each
        # SimWorld has its own virtual-clock shm segment, so SIM_TEST_JOBS=N
        # opts into pytest-xdist parallelism when the flake risk is
        # acceptable. SIM_TEST_TARGETS narrows the run to specific test
        # files (used by the CI shards).
        [[ -n "${VTIME_SPEED:-}" ]] && DOCKER_ARGS+=(-e VTIME_SPEED)
        XDIST_ARGS=()
        [[ -n "${SIM_TEST_JOBS:-}" ]] && XDIST_ARGS=(-n "$SIM_TEST_JOBS")
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} --entrypoint python3 "$IMAGE_TAG" \
            -m pytest ${SIM_TEST_TARGETS:-tools/sim/tests} \
            -m needs_elf -v -p no:cacheprovider \
            ${XDIST_ARGS[@]+"${XDIST_ARGS[@]}"} \
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
