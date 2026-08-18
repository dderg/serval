#!/bin/bash
# Kalico Simulator — build the Docker image and run it.
#
# Usage:
#   ./run.sh                          # self-test print (current tree)
#   ./run.sh --gcode benchy.gcode     # print a G-code file
#   ./run.sh test                     # run the e2e pytest suite
#   ./run.sh test -k probe            # subset of the e2e suite
#   ./run.sh test --keep-logs         # keep every world's logs in .sim-logs/
#   ./run.sh test --verbose           # stream the image build instead of hiding it
#   ./run.sh serve                    # long-lived printer for Moonraker
#   ./run.sh shell                    # bash inside the image
#   ./run.sh --branch main            # build+run a specific branch
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
DOCKER_ARGS=(--rm --ulimit memlock=-1:-1)
DOCKER_BUILD_ARGS=()
KEEP_LOGS=""
VERBOSE=""

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
        --keep-logs)
            KEEP_LOGS="$REPO_ROOT/.sim-logs"
            shift
            ;;
        --verbose)
            VERBOSE=1
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
[[ -n "$KEEP_LOGS" ]] && echo "  Logs:      $KEEP_LOGS"
echo ""

if [[ -z "$BRANCH" ]]; then
    # BuildKit's local context scan trusts mtimes; on macOS file sharing it
    # has been observed to serve stale file CONTENT for files whose mtimes
    # it thought it knew — the built image then quietly disagrees with the
    # worktree. Bumping content-changed mtimes forces a re-read of exactly
    # the risky files without invalidating the host's cargo/make caches
    # (touch_changed.py hashes the tree against a manifest in .cache/).
    python3 "$SCRIPT_DIR/touch_changed.py" "$REPO_ROOT" \
        "$REPO_ROOT/.cache/sim-ctx-manifest.json" \
        Makefile pyproject.toml src lib scripts klippy rust tools
fi

# A docker build narrates every layer even when all of them are cached: over a
# thousand lines that say nothing on success and everything on failure. A
# terminal and GitHub Actions keep the live stream (progress, job logs);
# anything else — an agent or a script capturing stdout — gets silence on
# success and the complete log on failure.
run_quiet() {
    if [[ -n "$VERBOSE" || -t 2 || -n "${GITHUB_ACTIONS:-}" ]]; then
        "$@" >&2
        return
    fi
    local log rc=0
    log="$(mktemp)"
    "$@" >"$log" 2>&1 || rc=$?
    [[ $rc -eq 0 ]] || { printf '%s failed (rc=%s):\n' "$1" "$rc" >&2; cat "$log" >&2; }
    rm -f "$log"
    return "$rc"
}

build_image() {
    # One retry absorbs transient registry/network failures (e.g. a cargo
    # crate download aborting mid-unpack on a CI runner). Deterministic
    # build errors replay from the layer cache and fail fast the second
    # time, so real breakage stays loud.
    local ctx="$1" dockerfile="$2"
    if ! run_quiet docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
            -t "$IMAGE_TAG" -f "$dockerfile" "$ctx"; then
        echo "docker build failed; retrying once (transient network flakes)" >&2
        run_quiet docker build ${DOCKER_BUILD_ARGS[@]+"${DOCKER_BUILD_ARGS[@]}"} \
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
        # Each SimWorld has its own virtual-clock shm segment, so worlds
        # parallelize cleanly; local runs default to 4 pytest-xdist
        # workers. SIM_TEST_JOBS=N overrides, SIM_TEST_JOBS=0 forces
        # sequential. SIM_TEST_TARGETS narrows the run to specific test
        # files (used by the CI shards).
        [[ -n "${VTIME_SPEED:-}" ]] && DOCKER_ARGS+=(-e VTIME_SPEED)
        [[ -n "${RUST_LOG:-}" ]] && DOCKER_ARGS+=(-e RUST_LOG)
        XDIST_ARGS=()
        if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
            # CI shards stay sequential unless the workflow opts in: their
            # 2-core runners stutter klippy's real-time budgets into flakes.
            [[ -n "${SIM_TEST_JOBS:-}" ]] && XDIST_ARGS=(-n "$SIM_TEST_JOBS")
        elif [[ "${SIM_TEST_JOBS:-4}" != 0 ]]; then
            XDIST_ARGS=(-n "${SIM_TEST_JOBS:-4}")
        fi
        # Every SimWorld lives in a pytest tmp dir that dies with the
        # container, taking klippy.log, the MCU logs and the structured
        # events/*.jsonl store with it. --keep-logs puts pytest's basetemp on
        # a host mount instead, so world0/, world1/ … survive the run. pytest
        # wipes and recreates its basetemp, which a mount point cannot be —
        # hence the subdirectory.
        if [[ -n "$KEEP_LOGS" ]]; then
            mkdir -p "$KEEP_LOGS"
            DOCKER_ARGS+=(-v "$KEEP_LOGS:/sim-logs")
            EXTRA_ARGS+=(--basetemp=/sim-logs/run)
        fi
        rc=0
        docker run ${DOCKER_ARGS[@]+"${DOCKER_ARGS[@]}"} --entrypoint python3 "$IMAGE_TAG" \
            -m pytest ${SIM_TEST_TARGETS:-tools/sim/tests} \
            -m needs_elf -v -p no:cacheprovider \
            ${XDIST_ARGS[@]+"${XDIST_ARGS[@]}"} \
            ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} || rc=$?
        # The container writes as root; hand the tree back so the host user
        # can read, grep and delete it without sudo.
        if [[ -n "$KEEP_LOGS" && "$(id -u)" != 0 ]]; then
            docker run --rm -v "$KEEP_LOGS:/sim-logs" --entrypoint chown \
                "$IMAGE_TAG" -R "$(id -u):$(id -g)" /sim-logs >/dev/null 2>&1 || true
        fi
        exit "$rc"
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
