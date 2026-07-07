# syntax=docker/dockerfile:1
#
# Multi-stage build — ordered from least- to most-frequently-changed.
# Each stage's expensive work is cached independently; only the changed
# stage and everything after it re-executes on a rebuild.
#
#   toolchain   apt packages + Rust toolchain  (changes: never)
#   rust-build  motion-engine cdylib           (registry cache only)
#   firmware    C firmware ELFs + chelper      (no compile caches)
#   runtime     final image — no build tools   (changes: klippy/, tools/)
#
# Compile outputs (cargo target/, make OUT dirs) are deliberately NOT
# cache-mounted: make and cargo decide freshness by mtime, and a persistent
# artifact dir paired with COPY'd sources went stale in practice — up to and
# including a firmware ELF speaking an older wire protocol than the host it
# shipped with. Skipping unchanged work is the layer cache's job: it keys on
# file CONTENT, so when sources really change the whole stage recompiles
# from a clean slate, and when they don't the stage is skipped entirely.
# Only the cargo registry/git caches remain — cargo addresses those by
# content, so they cannot serve stale artifacts.

# --------------------------------------------------------------------
# Stage 1 — toolchain
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
# Builds the motion-engine cdylib from a clean target dir every time the
# rust/ content changes; the layer cache skips the stage when it doesn't.
# --------------------------------------------------------------------
FROM toolchain AS rust-build

WORKDIR /kalico/rust

COPY rust/ /kalico/rust/

RUN --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    --mount=type=cache,target=/root/.cargo/git,sharing=shared \
    cargo build -p motion-engine --release && \
    mkdir -p /artifacts && \
    cp /kalico/rust/target/release/lib_motion_engine.so /artifacts/

# --------------------------------------------------------------------
# Stage 3 — firmware
# Builds the two MACH_LINUX ELFs (H7, F4), the preload shims, and the
# klippy chelper .so. The sim relaxations are first-class code gated by
# CONFIG_MCU_SIM (set in tools/sim/configs/*), not build-time patches.
# --------------------------------------------------------------------
FROM toolchain AS firmware

WORKDIR /kalico

# Copy what the C/firmware build needs. Deliberately NOT all of tools/sim —
# copying the volatile python (cli.py, world.py, tests) here would force
# a full firmware rebuild on every harness edit.
COPY Makefile /kalico/Makefile
COPY src/ /kalico/src/
COPY lib/ /kalico/lib/
COPY scripts/ /kalico/scripts/
COPY klippy/ /kalico/klippy/
COPY tools/check_async_event_registration.py /kalico/tools/check_async_event_registration.py
COPY tools/sim/preload/ /kalico/tools/sim/preload/
COPY tools/sim/configs/ /kalico/tools/sim/configs/
COPY tools/sim/fetch_plugins.sh /kalico/tools/sim/fetch_plugins.sh

# Build shims (libvtime.so + libsim_intercept.so)
RUN make -C tools/sim/preload

# Copy Rust source for the c-api staticlib build (make needs `cd rust`)
COPY rust/ /kalico/rust/

RUN mkdir -p /sim_elfs

# Each MCU builds into its own OUT dir (out-h7/, out-f4/) so the two
# configs never share object/descriptor state. Both are always built from
# clean OUT dirs — a fresh ELF whenever any input changed, never a stale
# incremental one. Both MCUs are MACH_LINUX sim builds with identical Rust
# features, so the second build reuses the first's target-linux staticlib.
RUN --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    set -e; \
    cp tools/sim/configs/h7-sim.config .config; \
    make olddefconfig OUT=out-h7/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-h7/; \
    cp out-h7/klipper.elf /sim_elfs/klipper-h7-sim.elf

RUN --mount=type=cache,target=/root/.cargo/registry,sharing=shared \
    set -e; \
    cp tools/sim/configs/f4-sim.config .config; \
    make olddefconfig OUT=out-f4/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-f4/; \
    cp out-f4/klipper.elf /sim_elfs/klipper-f4-sim.elf; \
    cp out-f4/klipper.dict /sim_elfs/klipper.dict

# Build klippy chelper .so
RUN cd klippy && python3 chelper/__init__.py

# Fetch pinned third-party klippy plugins (beacon).
RUN bash tools/sim/fetch_plugins.sh /kalico/tools/sim/third_party_repos

# --------------------------------------------------------------------
# Stage 4 — runtime (final image)
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
    python3-pytest \
    libffi8 \
    libudev1 \
    gdb \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

WORKDIR /kalico

# Python source first — artifacts copied after so they are not overwritten.
COPY klippy/ /kalico/klippy/
COPY tools/__init__.py /kalico/tools/__init__.py
COPY tools/sim/ /kalico/tools/sim/
COPY pyproject.toml /kalico/pyproject.toml

# Firmware ELFs
COPY --from=firmware /sim_elfs/ /kalico/out/

# Overlay chelper compiled .so files on top of the Python source copy.
COPY --from=firmware /kalico/klippy/chelper/ /kalico/klippy/chelper/

# Preload shims
COPY --from=firmware \
    /kalico/tools/sim/preload/libvtime.so \
    /kalico/tools/sim/preload/libvtime_clock_only.so \
    /kalico/tools/sim/preload/libsim_intercept.so \
    /kalico/tools/sim/preload/

# Overlay _motion_engine.so on top of the Python source copy.
COPY --from=rust-build \
    /artifacts/lib_motion_engine.so \
    /kalico/klippy/_motion_engine.so

# Third-party plugins fetched during the firmware stage.
COPY --from=firmware /kalico/tools/sim/third_party_repos/ \
    /kalico/tools/sim/third_party_repos/

# Symlink the beacon plugin into klippy/extras so config [beacon]
# sections resolve; PYTHONPATH for its imports is set by world.py.
RUN for f in beacon.py beacon_motion_engine.py; do \
        src=/kalico/tools/sim/third_party_repos/beacon_klipper/$f; \
        if [ -f "$src" ]; then ln -sf "$src" /kalico/klippy/extras/$f; fi; \
    done

RUN mkdir -p /gcode

ENTRYPOINT ["python3", "tools/sim/cli.py"]
CMD []
