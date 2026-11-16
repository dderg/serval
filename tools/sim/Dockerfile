# syntax=docker/dockerfile:1
#
#   toolchain       apt packages + the rust-toolchain.toml compiler
#   rust-src        rust/ pruned per consumer by prune_rust.py
#   cargo-registry  every crate in Cargo.lock (changes with manifests only)
#   rust-build      motion-engine/config/shaper-ident cdylibs
#   firmware        MACH_LINUX ELFs (H7, F4, SC)
#   preload, chelper, plugins   small independent artifacts
#   runtime         final image — no build tools
#
# Compile outputs (cargo target/, make OUT dirs) are deliberately NOT
# cache-mounted: make and cargo decide freshness by mtime, and a persistent
# artifact dir paired with COPY'd sources went stale in practice — up to and
# including a firmware ELF speaking an older wire protocol than the host it
# shipped with. Skipping unchanged work is the layer cache's job: it keys on
# file CONTENT, so a stage recompiles from a clean slate when its inputs
# change and is skipped when they don't. Each build stage gets only its own
# inputs: rust-build the host crates' closure, firmware the c-api closure,
# so an edit to a crate one side does not link leaves the other cached.
# The registry is a layer, not a shared cache mount: two stages unpacking
# into one mount raced past cargo's lock, which lives in CARGO_HOME
# (`.package-cache ... File exists`).

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
    sh -s -- -y --default-toolchain none --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

# The toml's channel for the host target only, overriding the toml for every
# directory: its cross targets and lint components are not needed here.
COPY rust/rust-toolchain.toml /toolchain/rust-toolchain.toml
RUN channel="$(sed -n 's/^channel = "\(.*\)"/\1/p' /toolchain/rust-toolchain.toml)" && \
    rustup toolchain install "$channel" && \
    rustup override set "$channel" --path /
ENV RUSTUP_AUTO_INSTALL=0

FROM toolchain AS rust-src

COPY rust/ /src/rust/
COPY tools/sim/prune_rust.py /src/prune_rust.py
RUN python3 /src/prune_rust.py /src/rust /pruned/skeleton && \
    python3 /src/prune_rust.py /src/rust /pruned/host \
        motion-engine config-py shaper-ident && \
    python3 /src/prune_rust.py /src/rust /pruned/firmware c-api

FROM toolchain AS cargo-registry

RUN --mount=type=bind,from=rust-src,source=/pruned/skeleton,target=/skeleton \
    cd /skeleton && \
    cargo fetch --locked --target "$(rustc -vV | sed -n 's/^host: //p')"
ENV CARGO_NET_OFFLINE=true

FROM cargo-registry AS rust-build

COPY --from=rust-src /pruned/host/ /kalico/rust/
WORKDIR /kalico/rust

RUN cargo build --locked -p motion-engine -p config-py -p shaper-ident --release && \
    mkdir -p /artifacts && \
    cp /kalico/rust/target/release/lib_motion_engine.so /artifacts/ && \
    cp /kalico/rust/target/release/lib_config_doc.so /artifacts/ && \
    cp /kalico/rust/target/release/lib_shaper_ident.so /artifacts/

# The sim relaxations are first-class code gated by CONFIG_MCU_SIM (set in
# tools/sim/configs/*), not build-time patches.
FROM cargo-registry AS firmware

WORKDIR /kalico

COPY Makefile /kalico/Makefile
COPY src/ /kalico/src/
COPY lib/ /kalico/lib/
COPY scripts/buildcommands.py scripts/check-gcc.sh scripts/find-firmware-extras.sh \
    /kalico/scripts/
COPY klippy/msgproto.py /kalico/klippy/msgproto.py
COPY tools/check_async_event_registration.py /kalico/tools/check_async_event_registration.py
COPY tools/sim/configs/ /kalico/tools/sim/configs/
COPY --from=rust-src /pruned/firmware/ /kalico/rust/

RUN mkdir -p /sim_elfs

# Each firmware builds into its own OUT dir (out-h7/, out-f4/, out-sc/) so
# the configs never share object/descriptor state. All are always built from
# clean OUT dirs — a fresh ELF whenever any input changed, never a stale
# incremental one. All are MACH_LINUX sim builds with identical Rust
# features, so later builds reuse the first's target-linux staticlib.
# out-sc/ leaves CONFIG_MOTION_RUNTIME off, so it is a step/dir only board:
# every lane is host-computed step times over queue_step, no sample executor.
RUN set -e; \
    cp tools/sim/configs/h7-sim.config .config; \
    make olddefconfig OUT=out-h7/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-h7/; \
    cp out-h7/klipper.elf /sim_elfs/klipper-h7-sim.elf; \
    cp out-h7/klipper.dict /sim_elfs/klipper-h7-sim.dict

RUN set -e; \
    cp tools/sim/configs/f4-sim.config .config; \
    make olddefconfig OUT=out-f4/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-f4/; \
    cp out-f4/klipper.elf /sim_elfs/klipper-f4-sim.elf; \
    cp out-f4/klipper.dict /sim_elfs/klipper.dict; \
    cp out-f4/klipper.dict /sim_elfs/klipper-f4-sim.dict

RUN set -e; \
    cp tools/sim/configs/sc-sim.config .config; \
    make olddefconfig OUT=out-sc/ 2>/dev/null || true; \
    make -j$(nproc) OUT=out-sc/; \
    cp out-sc/klipper.elf /sim_elfs/klipper-sc-sim.elf; \
    cp out-sc/klipper.dict /sim_elfs/klipper-sc-sim.dict

FROM toolchain AS preload

COPY tools/sim/preload/ /kalico/tools/sim/preload/
RUN make -C /kalico/tools/sim/preload

FROM toolchain AS chelper

COPY klippy/chelper/ /kalico/klippy/chelper/
RUN cd /kalico/klippy && python3 chelper/__init__.py

FROM toolchain AS plugins

COPY tools/sim/fetch_plugins.sh /kalico/tools/sim/fetch_plugins.sh
RUN bash /kalico/tools/sim/fetch_plugins.sh /kalico/tools/sim/third_party_repos

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
    python3-pytest-xdist \
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
COPY tools/bench/dispatch_record.py /kalico/tools/bench/dispatch_record.py
COPY tools/bench/gcode/ /kalico/tools/bench/gcode/
COPY pyproject.toml /kalico/pyproject.toml

# Firmware ELFs
COPY --from=firmware /sim_elfs/ /kalico/out/

# Overlay chelper compiled .so files on top of the Python source copy.
COPY --from=chelper /kalico/klippy/chelper/ /kalico/klippy/chelper/

# Preload shims
COPY --from=preload \
    /kalico/tools/sim/preload/libvtime.so \
    /kalico/tools/sim/preload/libsim_intercept.so \
    /kalico/tools/sim/preload/

# Overlay the native modules on top of the Python source copy.
COPY --from=rust-build \
    /artifacts/lib_motion_engine.so \
    /kalico/klippy/_motion_engine.so
COPY --from=rust-build \
    /artifacts/lib_config_doc.so \
    /kalico/klippy/_config_doc.so
COPY --from=rust-build \
    /artifacts/lib_shaper_ident.so \
    /kalico/klippy/_shaper_ident.so

COPY --from=plugins /kalico/tools/sim/third_party_repos/ \
    /kalico/tools/sim/third_party_repos/

# Symlink the beacon plugin into klippy/extras so config [beacon]
# sections resolve; PYTHONPATH for its imports is set by world.py.
RUN for f in beacon.py beacon_motion_engine.py; do \
        src=/kalico/tools/sim/third_party_repos/beacon_klipper/$f; \
        if [ -f "$src" ]; then ln -sf "$src" /kalico/klippy/extras/$f; fi; \
    done && \
    for f in scanner.py scanner_motion_engine.py; do \
        src=/kalico/tools/sim/third_party_repos/cartographer_klipper/$f; \
        if [ -f "$src" ]; then ln -sf "$src" /kalico/klippy/extras/$f; fi; \
    done

RUN mkdir -p /gcode

ENTRYPOINT ["python3", "tools/sim/cli.py"]
CMD []
