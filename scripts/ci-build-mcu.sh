#!/bin/bash
set -euo pipefail

MAIN_DIR="${PWD}"
DICTDIR="${DICTDIR:-${MAIN_DIR}/out/ci-mcu-kalico-dicts}"
mkdir -p "${DICTDIR}"

shopt -s nullglob
CONFIGS=(test/configs/*.config)
if [ ${#CONFIGS[@]} -eq 0 ]; then
    echo "ERROR: no test/configs/*.config found" >&2
    exit 1
fi
CONFIGS+=(tools/bench/f401.config)

for TARGET in "${CONFIGS[@]}"; do
    NAME="$(basename "${TARGET}" .config)"

    if grep -qxF 'CONFIG_MACH_LINUX=y' "${TARGET}"; then
        echo "=============== skip ${NAME} (native MACH_LINUX — see linux-mcu job)"
        continue
    fi

    echo "::group::=============== build ${NAME}"

    make clean
    make distclean
    unset CC

    cp "${TARGET}" .config
    make olddefconfig

    # An undefined Rust FFI symbol fails the link here.
    make V=1 -j"$(nproc)"

    size out/klipper.elf
    python3 tools/bench/step_benchmark_emu.py out
    python3 tools/bench/endstop_edge_emu.py out
    # `make distclean` wipes out/ (including DICTDIR); recreate before stashing.
    mkdir -p "${DICTDIR}"
    cp out/klipper.dict "${DICTDIR}/${NAME}.dict"

    echo "=============== ok ${NAME}"
    echo "::endgroup::"
done

make clean
make distclean
echo "All kalico MCU firmware targets built + linked OK."
