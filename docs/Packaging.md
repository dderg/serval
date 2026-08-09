# Packaging Serval

Serval is not a Python-only application. A usable package contains the Python host, the normal C helper, and native Rust artifacts built for the target host architecture. It must also be paired with firmware built from the same source revision when it drives a printer.

> A generic wheel or a package that only installs Python dependencies is **not** a supported Serval motion installation. It will fail at startup or when motion is constructed because the native configuration and motion modules are absent.

## Package inputs

- Python requirements and supported interpreter versions are declared in `pyproject.toml`.
- The root firmware build uses `Makefile`, `.config`, and `src/Kconfig`.
- The Rust workspace/toolchain lives in `rust/`; Cargo selects the pinned toolchain there.
- The native host build entry point is `scripts/build-native.sh`.

Build the native modules on the target architecture (or a compatible build environment):

```bash
./scripts/build-native.sh
```

The normal result must include:

| Artifact | Destination | Purpose |
| --- | --- | --- |
| `_config_doc.so` | `klippy/_config_doc.so` | Native configuration support; Klippy requires it at startup. |
| `_motion_engine.so` | `klippy/_motion_engine.so` | Rust/PyO3 motion engine. |
| `_shaper_ident.so` | `klippy/_shaper_ident.so` | Resonance-identification numeric core. |

The loader also supports `KALICO_NATIVE_DIR` for environments such as the CI image that keep native artifacts outside a bind-mounted source tree. A package using that arrangement must install all required modules there and set the environment for its service. Validate startup and real motion—not merely `import klippy`—against the packaged layout.

## Firmware release boundary

Serval's host-to-MCU protocol streams trajectory pieces. Package host changes and firmware changes as one compatibility unit: rebuild with `make menuconfig && make`, flash every participating MCU, and record the source revision and menu configuration. See [Quickstart](Quickstart.md) and [Hardware support](Hardware_Support.md) for supported targets. Never advertise an independently upgradable host package unless its firmware compatibility has been demonstrated.

## Validation before publishing

1. Build native artifacts in the intended target environment.
2. Compile Python bytecode if the distribution requires it: `python3 -m compileall klippy`.
3. Run relevant checks from [Developer guide](Development.md), including `./scripts/ci.sh quick` and the documentation build for documentation changes.
4. Install into a clean target-like environment; confirm the service finds the native modules, configuration parser, and Python dependencies.
5. For a hardware release, flash matching firmware on supported hardware and record the board, transport, drive mode, and test evidence. A simulator pass alone is insufficient.

For versions built from a source archive without `.git`, use `scripts/make_version.py` to create `klippy/.version` when that is required by the distribution's version policy. Preserve GPL and third-party license notices from the source tree.
