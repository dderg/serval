# syntax=docker/dockerfile:1
#
# Multi-stage build — ordered from least- to most-frequently-changed.
# Each stage's expensive work is cached independently; only the changed
# stage and everything after it re-executes on a rebuild.
#
#   toolchain   apt packages + Rust toolchain  (changes: never)
#   rust-build  motion-engine cdylib           (cache-mounted target/+registry)
#   firmware    C firmware ELFs + chelper      (cache-mounted per-MCU OUT)
#   runtime     final image — no build tools   (changes: klippy/, tools/)

# --------------------------------------------------------------------
# Stage 1 — toolchain
# Installs OS packages and Rust. Changes almost never.
# --------------------------------------------------------------------
FROM ubuntu:24.04 AS toolchain

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-cffi \
    python3-greenlet \
    python3-serial \
    python3-jinja2 \
    python3-markupsafe \
    libffi-dev \
    libudev-dev \
    pkg-config \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain stable --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

# --------------------------------------------------------------------
# Stage 2 — rust-build
# Builds the motion-engine cdylib. BuildKit cache mounts persist the cargo
# registry and the compiled target/ across builds (in the BuildKit daemon,
# not the image), so a rebuild after a source edit recompiles only the
# crates that changed instead of the whole workspace + dep graph. The
# target cache is keyed by SIM_CACHE_KEY (branch) so two branches building
# in parallel get independent caches; the registry is content-addressed and
# safe to share concurrently. The built .so is copied out of the (ephemeral)
# cache mount into a real layer path so the runtime stage can COPY --from it.
# --------------------------------------------------------------------
FROM toolchain AS rust-build

ARG SIM_CACHE_KEY=default

WORKDIR /kalico/rust

COPY rust/ /kalico/rust/
COPY tools/kalico-sim/patches/patch_rust_traces.py /tmp/patch_rust_traces.py
RUN python3 /tmp/patch_rust_traces.py /kalico/rust/motion-engine/src/bridge.rs

RUN --mount=type=cache,target=/kalico/rust/target,id=rust-target-${SIM_CACHE_KEY},sharing=locked \
    --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    --mount=type=cache,target=/root/.cargo/git,sharing=shared \
    cargo build -p motion-engine --release && \
    mkdir -p /artifacts && \
    cp /kalico/rust/target/release/lib_motion_engine.so /artifacts/

# --------------------------------------------------------------------
# Stage 3 — firmware
# Builds the two MACH_LINUX ELFs (H7, F4), the libvtime shims, and
# the klippy chelper .so. Completely independent of Rust source.
# --------------------------------------------------------------------
FROM toolchain AS firmware

ARG SIM_CACHE_KEY=default

WORKDIR /kalico

# Copy what the C/firmware build needs.
COPY Makefile /kalico/Makefile
COPY src/ /kalico/src/
COPY lib/ /kalico/lib/
COPY scripts/ /kalico/scripts/
COPY klippy/ /kalico/klippy/
# Only the parts of tools/ the firmware build actually touches — deliberately
# NOT the volatile runner.py / emulators. Copying all of tools/ here meant a
# one-character edit to runner.py invalidated this layer and forced a full
# firmware (H7 + F4 + chelper) rebuild on every Python-only change.
COPY tools/check_async_event_registration.py /kalico/tools/check_async_event_registration.py
COPY tools/kalico-sim/libvtime/ /kalico/tools/kalico-sim/libvtime/
COPY tools/kalico-sim/configs/ /kalico/tools/kalico-sim/configs/
COPY tools/kalico-sim/patches/ /kalico/tools/kalico-sim/patches/
COPY tools/sim_klippy/ /kalico/tools/sim_klippy/

# Build shims (libvtime.so + libsim_intercept.so)
RUN make -C tools/kalico-sim/libvtime

# Copy Rust source for c-api build (make needs `cd rust`)
COPY rust/ /kalico/rust/

# Patch source tree for MACH_LINUX builds (idempotent).
RUN bash tools/kalico-sim/patches/fix_linux_build.sh /kalico

RUN mkdir -p /sim_elfs

# Build H7 MACH_LINUX ELF.
#
# Each MCU builds into its own OUT dir (out-h7/, out-f4/) so the two configs
# never share object/descriptor state — this replaces the old `make clean`
# (needed only because both built into a shared out/) and lets each MCU's
# objects persist in a BuildKit cache mount for incremental rebuilds. The
# cache is keyed by SIM_CACHE_KEY (branch) so parallel branch builds stay
# isolated. Both MCUs are MACH_LINUX sim builds with identical Rust features,
# so they share one target-linux cache for the c-api staticlib (built
# by src/Makefile into rust/target-linux/) — no per-MCU thrash, and H7 (which
# runs first) keeps its cache warm across builds instead of rebuilding cold.
RUN --mount=type=cache,target=/kalico/out-h7,id=fw-h7-${SIM_CACHE_KEY},sharing=locked \
    --mount=type=cache,target=/kalico/rust/target-linux,id=fw-rust-linux-${SIM_CACHE_KEY},sharing=locked \
    --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    set -e; \
    if [ -f tools/sim_klippy/configs/h7-sim.config ]; then \
        cp tools/sim_klippy/configs/h7-sim.config .config; \
    else \
        cp tools/kalico-sim/configs/h7-sim.config .config; \
    fi; \
    make olddefconfig OUT=out-h7/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-h7/; \
    cp out-h7/klipper.elf /sim_elfs/klipper-h7-sim.elf

# Build F4 MACH_LINUX ELF (independent OUT dir; shares the target-linux
# c-api cache with H7 — same MACH_LINUX features, so no thrash).
RUN --mount=type=cache,target=/kalico/out-f4,id=fw-f4-${SIM_CACHE_KEY},sharing=locked \
    --mount=type=cache,target=/kalico/rust/target-linux,id=fw-rust-linux-${SIM_CACHE_KEY},sharing=locked \
    --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    set -e; \
    if [ -f tools/sim_klippy/configs/f4-sim.config ]; then \
        cp tools/sim_klippy/configs/f4-sim.config .config; \
    else \
        cp tools/kalico-sim/configs/f4-sim.config .config; \
    fi; \
    make olddefconfig OUT=out-f4/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-f4/; \
    cp out-f4/klipper.elf /sim_elfs/klipper-f4-sim.elf; \
    cp out-f4/klipper.dict /sim_elfs/klipper.dict

# Build klippy chelper .so
RUN cd klippy && python3 chelper/__init__.py

# Fetch pinned third-party klippy plugins (no-op if the script is absent).
# The target directory is always created so the runtime COPY --from is safe.
RUN mkdir -p tools/sim_klippy/printer_real/third_party_repos; \
    if [ -f tools/sim_klippy/fetch_plugins.sh ]; then \
        bash tools/sim_klippy/fetch_plugins.sh 2>/dev/null || true; \
    fi

# --------------------------------------------------------------------
# Stage 4 — runtime (final image)
# No build toolchain. Copies only the artifacts and Python sources
# needed to run the simulator.
# --------------------------------------------------------------------
FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    libpython3.12t64 \
    python3-pip \
    python3-cffi \
    python3-greenlet \
    python3-serial \
    python3-jinja2 \
    python3-markupsafe \
    libffi8 \
    libudev1 \
    gdb \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

WORKDIR /kalico

# Python source first — artifacts copied after so they are not overwritten.
COPY klippy/ /kalico/klippy/
COPY tools/kalico-sim/ /kalico/tools/kalico-sim/
COPY tools/sim_klippy/ /kalico/tools/sim_klippy/

# Copy firmware ELFs
COPY --from=firmware /sim_elfs/ /kalico/out/

# Overlay chelper compiled .so files on top of the Python source copy.
COPY --from=firmware /kalico/klippy/chelper/ /kalico/klippy/chelper/

# Copy libvtime shims
COPY --from=firmware \
    /kalico/tools/kalico-sim/libvtime/libvtime.so \
    /kalico/tools/kalico-sim/libvtime/libvtime_clock_only.so \
    /kalico/tools/kalico-sim/libvtime/libsim_intercept.so \
    /kalico/tools/kalico-sim/libvtime/

# Overlay _motion_engine.so on top of the Python source copy.
COPY --from=rust-build \
    /artifacts/lib_motion_engine.so \
    /kalico/klippy/_motion_engine.so

# Copy third-party plugins fetched during the firmware stage (may not exist).
COPY --from=firmware /kalico/tools/sim_klippy/printer_real/third_party_repos/ \
    /kalico/tools/sim_klippy/printer_real/third_party_repos/

# Apply sim-only Python traces. Can't go in fix_linux_build.sh because
# the runtime stage re-copies klippy/ fresh from the build context.
COPY tools/kalico-sim/patches/patch_klippy_traces.py /tmp/patch_klippy_traces.py
RUN python3 /tmp/patch_klippy_traces.py klippy/motion.py klippy/motion_engine.py klippy/mcu.py

RUN mkdir -p /gcode /tmp/kalico_sim_gcodes

ENTRYPOINT ["python3", "tools/kalico-sim/runner.py"]
CMD []
