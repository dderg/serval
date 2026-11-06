"""Build the MCU data dictionary klippy needs for batch (file output) mode.

The firmware is compiled out of tree with one of the stock test/configs and
only its klipper.dict is used; no board is involved. `make` is incremental,
so calling build() on every run costs well under a second once built.
"""

from __future__ import annotations

import pathlib
import subprocess

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD_ROOT = pathlib.Path(__file__).resolve().parent / ".build"
DEFAULT_MCU_CONFIG = "stm32h723"


def build(mcu_config: str = DEFAULT_MCU_CONFIG) -> pathlib.Path:
    source_config = REPO_ROOT / "test" / "configs" / (mcu_config + ".config")
    if not source_config.is_file():
        raise FileNotFoundError("no MCU config %s" % (source_config,))
    build_dir = BUILD_ROOT / mcu_config
    out_dir = build_dir / "out"
    kconfig = build_dir / ".config"
    seed_copy = build_dir / "seed.config"
    build_dir.mkdir(parents=True, exist_ok=True)
    seed = source_config.read_bytes()
    if not seed_copy.exists() or seed_copy.read_bytes() != seed:
        kconfig.write_bytes(seed)
        _make(out_dir, kconfig, "olddefconfig")
        seed_copy.write_bytes(seed)
    _make(out_dir, kconfig)
    dictionary = out_dir / "klipper.dict"
    if not dictionary.is_file():
        raise RuntimeError("firmware build did not produce %s" % (dictionary,))
    return dictionary


def _make(out_dir: pathlib.Path, kconfig: pathlib.Path, *targets: str):
    cmd = [
        "make",
        "-j8",
        "OUT=%s/" % (out_dir,),
        "KCONFIG_CONFIG=%s" % (kconfig,),
        *targets,
    ]
    res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if res.returncode:
        raise RuntimeError(
            "MCU dictionary build failed (%s):\n%s"
            % (" ".join(cmd), (res.stdout + res.stderr)[-4000:])
        )
