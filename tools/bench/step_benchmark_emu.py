"""Run a firmware build's step self-benchmark in an instruction emulator.

uv run --with unicorn --with pyelftools python tools/bench/step_benchmark_emu.py \
    <out_dir>...

Each <out_dir> holds a built klipper.elf and its autoconf.h. The emulator's
timer counts executed instructions and it delivers the timer interrupt
(SysTick, or TIM2 on G0) the firmware programs, by calling its handler as
the core would once the interrupt is due and PRIMASK is clear. The benchmark
times one isolated motor's steps through that interrupt, so it reports
instructions per step from the handler's entry to its exit: one cycle per
instruction, no wait states, no hardware stacking. Fails when a result is
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
    UC_ARM_REG_PC,
    UC_ARM_REG_PRIMASK,
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
CONFIGURED_MOVE_COUNT = 1024
PAGE = 0x1000
STACK_BASE = 0x70000000
STACK_SIZE = 0x100000
STOP = 0x70200000
IRQ_RETURN = 0x70201000
DWT_CYCCNT = 0xE0001004
SYSTICK_LOAD = 0xE000E014
SYSTICK_VAL = 0xE000E018
SCB_ICSR = 0xE000ED04
ICSR_PENDSTSET = 1 << 26
G0_TIM2_CNT = 0x40000024
G0_TIM2_CCR1 = 0x40000034


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


class TimerInterrupt:
    """The one interrupt the benchmark needs: when it is due, in executed
    instructions, from what the firmware wrote to the timer registers. It
    starts due, as timer_init leaves it kicked."""

    def __init__(self, g0: bool):
        self.g0 = g0
        self.load = 0
        self.due: int | None = 0

    def write(self, address: int, value: int, now: int) -> None:
        if self.g0:
            if address == G0_TIM2_CCR1:
                self.due = value
        elif address == SYSTICK_LOAD:
            self.load = value
        elif address == SYSTICK_VAL:
            self.due = now + self.load if self.load else None
        elif address == SCB_ICSR and value & ICSR_PENDSTSET:
            self.due = now


def measure(out_dir: Path) -> tuple[int, int]:
    conf = autoconf(out_dir)
    g0 = conf.get("MACH_STM32G0") == "1"
    with open(out_dir / "klipper.elf", "rb") as handle:
        elf = ELFFile(handle)
        entry = symbol(elf, "stepper_bench_measure")
        result = symbol(elf, "bench_result")
        move_count = symbol(elf, "move_count")
        shutdown = symbol(elf, "sched_shutdown") & ~1
        handler = symbol(
            elf, "TIMx_IRQHandler" if g0 else "SysTick_Handler"
        ) & ~1
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(cpu_model(conf))
        map_image(uc, elf)
    uc.mem_write(move_count, struct.pack("<I", CONFIGURED_MOVE_COUNT))
    shut_down = False
    executed = 0
    in_handler = False
    interrupt_pending = False
    interrupt = TimerInterrupt(g0)
    timer_register = G0_TIM2_CNT if g0 else DWT_CYCCNT

    def count(emu, address, _size, _user):
        nonlocal executed, shut_down, interrupt_pending
        executed += 1
        if address == shutdown:
            shut_down = True
            emu.emu_stop()
        if executed > INSTRUCTION_LIMIT:
            emu.emu_stop()
        if (
            not in_handler
            and interrupt.due is not None
            and executed - interrupt.due >= 0
            and not emu.reg_read(UC_ARM_REG_PRIMASK)
        ):
            interrupt_pending = True
            emu.emu_stop()

    def read(_emu, offset, _size, base):
        return executed if base + offset == timer_register else 0

    def write(_emu, offset, _size, value, base):
        interrupt.write(base + offset, value, executed)

    for base, size in ((0x40000000, 0x20000000), (0xE0000000, 0x100000)):
        uc.mmio_map(
            base,
            size,
            lambda e, o, s, u, b=base: read(e, o, s, b),
            None,
            lambda e, o, s, v, u, b=base: write(e, o, s, v, b),
            None,
        )
    uc.mem_map(STACK_BASE, STACK_SIZE + PAGE)
    uc.mem_map(STOP, 2 * PAGE)
    uc.mem_write(STOP, b"\x00\xbf" * 4)
    uc.mem_write(IRQ_RETURN, b"\x00\xbf" * 4)
    uc.hook_add(UC_HOOK_CODE, count)
    uc.reg_write(UC_ARM_REG_SP, STACK_BASE + STACK_SIZE)
    uc.reg_write(UC_ARM_REG_LR, STOP | 1)
    uc.reg_write(UC_ARM_REG_R0, result)
    uc.reg_write(UC_ARM_REG_R1, result + 4)
    resume = entry | 1
    while True:
        uc.emu_start(resume, STOP)
        if shut_down or executed > INSTRUCTION_LIMIT or not interrupt_pending:
            break
        interrupt_pending = False
        interrupt.due = None
        interrupted = uc.context_save()
        resume = uc.reg_read(UC_ARM_REG_PC) | 1
        in_handler = True
        uc.reg_write(UC_ARM_REG_LR, IRQ_RETURN | 1)
        uc.emu_start(handler | 1, IRQ_RETURN)
        in_handler = False
        uc.context_restore(interrupted)
    if shut_down:
        raise EmulationError(
            f"{out_dir}: benchmark shut the MCU down (configured MCU, "
            f"move_count={CONFIGURED_MOVE_COUNT})"
        )
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
