"""Run a firmware build's step self-benchmark in an instruction emulator.

uv run --with unicorn --with pyelftools python tools/bench/step_benchmark_emu.py \
    <out_dir>...

Each <out_dir> holds a built klipper.elf and its autoconf.h. The emulator's
timer counts executed instructions, so the benchmark reports instructions per
step: one cycle per instruction, no wait states. Fails when a result is
outside [MIN_INSTRUCTIONS_PER_STEP, MAX_INSTRUCTIONS_PER_STEP].
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB, Uc
from unicorn.arm_const import (
    UC_ARM_REG_LR,
    UC_ARM_REG_R0,
    UC_ARM_REG_R1,
    UC_ARM_REG_SP,
    UC_CPU_ARM_CORTEX_M0,
    UC_CPU_ARM_CORTEX_M3,
    UC_CPU_ARM_CORTEX_M4,
    UC_CPU_ARM_CORTEX_M7,
)

MIN_INSTRUCTIONS_PER_STEP = 40
MAX_INSTRUCTIONS_PER_STEP = 600
INSTRUCTION_LIMIT = 20_000_000
PAGE = 0x1000
STACK_BASE = 0x70000000
STACK_SIZE = 0x100000
STOP = 0x70200000
DWT_CYCCNT = 0xE0001004
G0_TIM2_CNT = 0x40000024


class EmulationError(RuntimeError):
    pass


def autoconf(out_dir: Path) -> dict[str, str]:
    found = {}
    for line in (out_dir / "autoconf.h").read_text().splitlines():
        match = re.match(r"#define CONFIG_(\w+) (.*)", line)
        if match:
            found[match.group(1)] = match.group(2).strip('"')
    return found


def cpu_model(conf: dict[str, str]) -> int:
    if conf.get("MACH_STM32H7") == "1":
        return UC_CPU_ARM_CORTEX_M7
    if conf.get("MACH_STM32G0") == "1":
        return UC_CPU_ARM_CORTEX_M0
    if conf.get("MACH_STM32F1") == "1":
        return UC_CPU_ARM_CORTEX_M3
    return UC_CPU_ARM_CORTEX_M4


def symbol(elf: ELFFile, prefix: str) -> int:
    for sym in elf.get_section_by_name(".symtab").iter_symbols():
        if sym.name == prefix or sym.name.startswith(prefix + "."):
            return sym["st_value"]
    raise EmulationError(f"no symbol {prefix!r}")


def map_image(uc: Uc, elf: ELFFile) -> None:
    pages = set()
    for seg in elf.iter_segments():
        if seg["p_type"] != "PT_LOAD" or not seg["p_memsz"]:
            continue
        for start, size in (
            (seg["p_vaddr"], seg["p_memsz"]),
            (seg["p_paddr"], seg["p_filesz"]),
        ):
            for page in range(start // PAGE, (start + size + PAGE - 1) // PAGE):
                pages.add(page)
    for page in sorted(pages):
        uc.mem_map(page * PAGE, PAGE)
    for seg in elf.iter_segments():
        if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]:
            data = seg.data()
            uc.mem_write(seg["p_paddr"], data)
            uc.mem_write(seg["p_vaddr"], data)


def measure(out_dir: Path) -> tuple[int, int]:
    conf = autoconf(out_dir)
    with open(out_dir / "klipper.elf", "rb") as handle:
        elf = ELFFile(handle)
        entry = symbol(elf, "stepper_bench_measure")
        result = symbol(elf, "bench_result")
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(cpu_model(conf))
        map_image(uc, elf)
    executed = 0
    timer_register = (
        G0_TIM2_CNT if conf.get("MACH_STM32G0") == "1" else DWT_CYCCNT
    )

    def count(emu, _address, _size, _user):
        nonlocal executed
        executed += 1
        if executed > INSTRUCTION_LIMIT:
            emu.emu_stop()

    def read(_emu, offset, _size, base):
        return executed if base + offset == timer_register else 0

    def ignore(*_args):
        pass

    for base, size in ((0x40000000, 0x20000000), (0xE0000000, 0x100000)):
        uc.mmio_map(
            base,
            size,
            lambda e, o, s, u, b=base: read(e, o, s, b),
            None,
            ignore,
            None,
        )
    uc.mem_map(STACK_BASE, STACK_SIZE + PAGE)
    uc.mem_map(STOP, PAGE)
    uc.mem_write(STOP, b"\x00\xbf" * 4)
    uc.hook_add(UC_HOOK_CODE, count)
    uc.reg_write(UC_ARM_REG_SP, STACK_BASE + STACK_SIZE)
    uc.reg_write(UC_ARM_REG_LR, STOP | 1)
    uc.reg_write(UC_ARM_REG_R0, result)
    uc.reg_write(UC_ARM_REG_R1, result + 4)
    uc.emu_start(entry | 1, STOP)
    if executed > INSTRUCTION_LIMIT:
        raise EmulationError(
            f"{out_dir}: benchmark ran past {INSTRUCTION_LIMIT} instructions"
        )
    return struct.unpack("<II", uc.mem_read(result, 8))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out_dirs", nargs="+", type=Path)
    args = parser.parse_args()
    failures = []
    for out_dir in args.out_dirs:
        if autoconf(out_dir).get("MCU_SIM") == "1":
            print(f"{out_dir}: MCU_SIM build reports simulated=1, not run")
            continue
        pulsed, both_edge = measure(out_dir)
        print(
            f"{out_dir}: pulsed {pulsed} instructions/step, "
            f"both-edge {both_edge} instructions/step"
        )
        for name, value in (("pulsed", pulsed), ("both-edge", both_edge)):
            if (
                not MIN_INSTRUCTIONS_PER_STEP
                <= value
                <= MAX_INSTRUCTIONS_PER_STEP
            ):
                failures.append(f"{out_dir}: {name} {value} instructions/step")
    if failures:
        print(
            "outside "
            f"[{MIN_INSTRUCTIONS_PER_STEP}, {MAX_INSTRUCTIONS_PER_STEP}]: "
            + "; ".join(failures),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
