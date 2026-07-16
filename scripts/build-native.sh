#!/usr/bin/env bash
# build-native.sh — the one entry point for host-side Rust artifacts.
#
# Builds and installs everything klippy needs on the machine it runs on:
#   klippy/_config_doc.so     config parser (klippy refuses to start without it)
#   klippy/_motion_engine.so  motion engine cdylib
# and, when requested or auto-detected, the bench-side binaries:
#   rust/target/snapshot/servo-ident
#   rust/target/release/ethercat-rt        (--ethercat hw; needs IgH libs)
#   rust/target/release/ethercat-rt-stub   (--ethercat stub)
#
# Usage:
#   scripts/build-native.sh [--fast] [--config-only] [--bench]
#                           [--ethercat hw|stub|none]
#
#   --fast         motion engine under the `snapshot` cargo profile
#                  (float-identical, much faster rebuilds; snapshot dev loop)
#   --config-only  just klippy/_config_doc.so (python-only environments)
#   --bench        servo-ident + EtherCAT endpoint (hw if /opt/etherlab
#                  exists, else stub) on top of the default artifacts
#   --ethercat     override the --bench EtherCAT auto-detection
#
# Fail-fast; artifacts are verified to exist after each build.
set -euo pipefail

cd "$(dirname "$0")/.."

FAST=0
CONFIG_ONLY=0
BENCH=0
ETHERCAT=auto
while [ $# -gt 0 ]; do
    case "$1" in
        --fast)        FAST=1 ;;
        --config-only) CONFIG_ONLY=1 ;;
        --bench)       BENCH=1 ;;
        --ethercat)    shift; ETHERCAT="${1:?--ethercat needs hw|stub|none}" ;;
        -h|--help)     sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "build-native.sh: unknown argument '$1'" >&2; exit 2 ;;
    esac
    shift
done

require() { test -e "$1" || { echo "build-native.sh: ERROR: $1 missing after build" >&2; exit 1; }; }

if [ "$CONFIG_ONLY" = 1 ]; then
    make -f Makefile.rust config-doc
    require klippy/_config_doc.so
    exit 0
fi

if [ "$FAST" = 1 ]; then
    make -f Makefile.rust motion-engine-fast
else
    make -f Makefile.rust motion-engine
fi
require klippy/_config_doc.so
require klippy/_motion_engine.so

if [ "$BENCH" = 1 ]; then
    make -f Makefile.rust servo-ident
    require rust/target/snapshot/servo-ident

    if [ "$ETHERCAT" = auto ]; then
        if [ -d /opt/etherlab ]; then ETHERCAT=hw; else ETHERCAT=stub; fi
    fi
    case "$ETHERCAT" in
        hw)
            make -f Makefile.rust ethercat-endpoint-hw
            require rust/target/release/ethercat-rt
            ;;
        stub)
            make -f Makefile.rust ethercat-stub
            require rust/target/release/ethercat-rt-stub
            ;;
        none) ;;
        *) echo "build-native.sh: --ethercat must be hw|stub|none" >&2; exit 2 ;;
    esac
fi

echo "build-native.sh: done."
