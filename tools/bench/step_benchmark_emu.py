"""Run a firmware build's step self-benchmark in an instruction emulator.

uv run --with unicorn --with pyelftools python tools/bench/step_benchmark_emu.py \
    <out_dir>...

Each <out_dir> holds a built klipper.elf and its autoconf.h. The emulator's
timer counts executed instructions and it delivers the timer interrupt
(SysTick, or TIM2 on G0) the firmware programs, by calling its handler as
the core would once the interrupt is due and PRIMASK is clear, after the
core's exception entry cycles. The benchmark times one isolated motor's steps
through that interrupt, so it reports instructions per step: one cycle per
instruction, no wait states. Fails when a result is outside
[MIN_INSTRUCTIONS_PER_STEP, MAX_INSTRUCTIONS_PER_STEP].

It then judges the step edges: the instruction that writes the benchmark's
step sink, against the waketime of the timer that wrote it. A second
benchmark run sets a bare timer shortly before each step, so steps also
reach the sink from the dispatch loop instead of from their own interrupt.
An edge is judged on the irq path when its timer is the first the interrupt
dispatches, the interrupt was taken the moment it was due, and it was not due
after the waketime. It is judged on the loop path when the dispatch loop was
spinning on the timer up to the waketime: its last two reads before it are
each at most SPIN_POLL_GAP instructions apart.
Other edges (deferred, queued behind another dispatch) are only checked for
being early. Fails when an edge is early, or when one dispatch function's
judged edges spread over more than EDGE_LATENESS_SPREAD_LIMIT instructions
across both paths.
"""

from __future__ import annotations

import argparse
import itertools
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import (
    UC_ARCH_ARM,
    UC_HOOK_CODE,
    UC_HOOK_MEM_WRITE,
    UC_MODE_MCLASS,
    UC_MODE_THUMB,
    Uc,
)
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
EDGE_LATENESS_SPREAD_LIMIT = 8
SPIN_POLL_GAP = 6
PRECURSOR_LEADS = range(40, 400, 30)
PRECURSOR_INSERT_MARGIN = 400
INSTRUCTION_LIMIT = 20_000_000
CONFIGURED_MOVE_COUNT = 1024
CORTEX_M0_PLUS_EXCEPTION_ENTRY_CYCLES = 15
CORTEX_M3_M4_M7_EXCEPTION_ENTRY_CYCLES = 12
PAGE = 0x1000
STACK_BASE = 0x70000000
STACK_SIZE = 0x100000
STOP = 0x70200000
IRQ_RETURN = 0x70201000
CALL_RETURN = 0x70201100
PRECURSOR_TIMER = 0x70201200
PRECURSOR_FUNC = 0x70201300
THUMB_NOP = b"\x00\xbf"
THUMB_MOVS_R0_SF_DONE = b"\x00\x20"
THUMB_BX_LR = b"\x70\x47"
TIMER_FUNC_OFFSET = 4
TIMER_WAKETIME_OFFSET = 8
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


def cpu_model(conf: dict[str, str]) -> tuple[int, int]:
    if conf.get("MACH_STM32H7") == "1":
        return UC_CPU_ARM_CORTEX_M7, CORTEX_M3_M4_M7_EXCEPTION_ENTRY_CYCLES
    if conf.get("MACH_STM32G0") == "1":
        return UC_CPU_ARM_CORTEX_M0, CORTEX_M0_PLUS_EXCEPTION_ENTRY_CYCLES
    if conf.get("MACH_STM32F1") == "1":
        return UC_CPU_ARM_CORTEX_M3, CORTEX_M3_M4_M7_EXCEPTION_ENTRY_CYCLES
    return UC_CPU_ARM_CORTEX_M4, CORTEX_M3_M4_M7_EXCEPTION_ENTRY_CYCLES


def symbol(elf: ELFFile, prefix: str) -> tuple[int, int]:
    for sym in elf.get_section_by_name(".symtab").iter_symbols():
        if sym.name == prefix or sym.name.startswith(prefix + "."):
            return sym["st_value"], sym["st_size"]
    raise EmulationError(f"no symbol {prefix!r}")


def code_address(elf: ELFFile, prefix: str) -> int:
    return symbol(elf, prefix)[0] & ~1


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


@dataclass
class TakenInterrupt:
    due: int
    taken_on_time: bool
    dispatched: int = 0


@dataclass
class Dispatch:
    timer: int
    func: int
    waketime: int
    path: str | None


@dataclass
class Edge:
    dispatch: Dispatch
    time: int

    @property
    def lateness(self) -> int:
        return self.time - self.dispatch.waketime


@dataclass
class EdgeSpread:
    path: str
    count: int
    earliest: int
    latest: int


class Firmware:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        conf = autoconf(out_dir)
        g0 = conf.get("MACH_STM32G0") == "1"
        model, self.exception_entry_cycles = cpu_model(conf)
        with open(out_dir / "klipper.elf", "rb") as handle:
            elf = ELFFile(handle)
            self.benchmark = code_address(elf, "stepper_bench_measure")
            self.result = symbol(elf, "bench_result")[0]
            move_count = symbol(elf, "move_count")[0]
            self.shutdown = code_address(elf, "sched_shutdown")
            self.handler = code_address(
                elf, "TIMx_IRQHandler" if g0 else "SysTick_Handler"
            )
            self.add_timer = code_address(elf, "sched_add_timer")
            history, history_size = symbol(elf, "sched_dispatch_history_addr")
            sink, sink_size = symbol(elf, "sink")
            self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
            self.uc.ctl_set_cpu_model(model)
            map_image(self.uc, elf)
        uc = self.uc
        uc.mem_write(move_count, struct.pack("<I", CONFIGURED_MOVE_COUNT))
        self.executed = 0
        self.shut_down = False
        self.masked = False
        self.interrupt_pending = False
        self.interrupt = TimerInterrupt(g0)
        self.taken: TakenInterrupt | None = None
        self.polls: list[int] = []
        self.dispatch: Dispatch | None = None
        self.edges: list[Edge] = []
        self.step_timer: int | None = None
        self.precursor_pending = False
        self.precursor_leads = itertools.cycle(PRECURSOR_LEADS)
        timer_register = G0_TIM2_CNT if g0 else DWT_CYCCNT

        def count(emu, address, _size, _user):
            self.executed += 1
            if address == self.shutdown:
                self.shut_down = True
                emu.emu_stop()
            if self.executed > INSTRUCTION_LIMIT:
                emu.emu_stop()
            if (
                not self.masked
                and self.interrupt.due is not None
                and self.executed - self.interrupt.due >= 0
                and not emu.reg_read(UC_ARM_REG_PRIMASK)
            ):
                self.interrupt_pending = True
                emu.emu_stop()

        def read(_emu, offset, _size, base):
            if base + offset != timer_register:
                return 0
            self.polls.append(self.executed)
            return self.executed

        def write(_emu, offset, _size, value, base):
            self.interrupt.write(base + offset, value, self.executed)

        def dispatched(_emu, _access, _address, _size, timer, _user):
            if self.taken is None:
                raise EmulationError("timer dispatched outside the interrupt")
            waketime = self.word(timer + TIMER_WAKETIME_OFFSET)
            self.dispatch = Dispatch(
                timer=timer,
                func=self.word(timer + TIMER_FUNC_OFFSET),
                waketime=waketime,
                path=self.path(waketime),
            )
            self.taken.dispatched += 1
            self.polls = []
            if timer == PRECURSOR_TIMER:
                self.precursor_pending = False

        def stepped(_emu, _access, _address, _size, _value, _user):
            if self.dispatch is None:
                raise EmulationError("step sink written outside a dispatch")
            self.edges.append(Edge(self.dispatch, self.executed))
            self.step_timer = self.dispatch.timer

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
        for sentinel in (STOP, IRQ_RETURN, CALL_RETURN):
            uc.mem_write(sentinel, THUMB_NOP * 4)
        uc.mem_write(PRECURSOR_FUNC, THUMB_MOVS_R0_SF_DONE + THUMB_BX_LR)
        uc.hook_add(UC_HOOK_CODE, count)
        uc.hook_add(
            UC_HOOK_MEM_WRITE,
            dispatched,
            begin=history,
            end=history + history_size - 1,
        )
        uc.hook_add(
            UC_HOOK_MEM_WRITE, stepped, begin=sink, end=sink + sink_size - 1
        )

    def word(self, address: int) -> int:
        return struct.unpack("<I", self.uc.mem_read(address, 4))[0]

    def path(self, waketime: int) -> str | None:
        taken = self.taken
        if not taken.dispatched:
            if taken.taken_on_time and waketime - taken.due >= 0:
                return "irq"
            return None
        before = [poll for poll in self.polls if waketime - poll > 0]
        spinning = (
            len(before) >= 2
            and waketime - before[-1] <= SPIN_POLL_GAP
            and before[-1] - before[-2] <= SPIN_POLL_GAP
        )
        return "loop" if spinning else None

    def run_benchmark(self, precursors: bool) -> tuple[int, int]:
        uc = self.uc
        uc.reg_write(UC_ARM_REG_SP, STACK_BASE + STACK_SIZE)
        uc.reg_write(UC_ARM_REG_LR, STOP | 1)
        uc.reg_write(UC_ARM_REG_R0, self.result)
        uc.reg_write(UC_ARM_REG_R1, self.result + 4)
        resume = self.benchmark | 1
        while True:
            uc.emu_start(resume, STOP)
            self.require_running()
            if not self.interrupt_pending:
                break
            self.take_interrupt()
            resume = uc.reg_read(UC_ARM_REG_PC) | 1
            if precursors:
                self.set_precursor()
        return struct.unpack("<II", uc.mem_read(self.result, 8))

    def take_interrupt(self) -> None:
        due = self.interrupt.due
        self.taken = TakenInterrupt(due, taken_on_time=self.executed == due)
        self.interrupt_pending = False
        self.interrupt.due = None
        self.executed += self.exception_entry_cycles
        self.polls = []
        interrupted = self.uc.context_save()
        self.subroutine(self.handler, IRQ_RETURN)
        self.uc.context_restore(interrupted)
        self.taken = None
        self.dispatch = None

    def subroutine(self, function: int, sentinel: int) -> None:
        self.masked = True
        self.uc.reg_write(UC_ARM_REG_LR, sentinel | 1)
        self.uc.emu_start(function | 1, sentinel)
        self.masked = False
        self.require_running()

    def set_precursor(self) -> None:
        """Set a bare timer some instructions before the benchmark stepper's
        next edge, so that edge is dispatched right after another timer."""
        if self.precursor_pending or self.step_timer is None:
            return
        step_waketime = self.word(self.step_timer + TIMER_WAKETIME_OFFSET)
        waketime = step_waketime - next(self.precursor_leads)
        if waketime - self.executed < PRECURSOR_INSERT_MARGIN:
            return
        self.uc.mem_write(
            PRECURSOR_TIMER,
            struct.pack("<III", 0, PRECURSOR_FUNC | 1, waketime & 0xFFFFFFFF),
        )
        interrupted = self.uc.context_save()
        self.uc.reg_write(UC_ARM_REG_R0, PRECURSOR_TIMER)
        self.subroutine(self.add_timer, CALL_RETURN)
        self.uc.context_restore(interrupted)
        self.precursor_pending = True

    def require_running(self) -> None:
        if self.shut_down:
            raise EmulationError(
                f"{self.out_dir}: benchmark shut the MCU down (configured MCU, "
                f"move_count={CONFIGURED_MOVE_COUNT})"
            )
        if self.executed > INSTRUCTION_LIMIT:
            raise EmulationError(
                f"{self.out_dir}: benchmark ran past {INSTRUCTION_LIMIT} instructions"
            )


def edge_spreads(edges: list[Edge]) -> dict[int, list[EdgeSpread]]:
    by_func: dict[int, dict[str, list[int]]] = {}
    for edge in edges:
        if edge.dispatch.path is None:
            continue
        by_func.setdefault(edge.dispatch.func, {}).setdefault(
            edge.dispatch.path, []
        ).append(edge.lateness)
    return {
        func: [
            EdgeSpread(path, len(lateness), min(lateness), max(lateness))
            for path, lateness in sorted(paths.items())
        ]
        for func, paths in by_func.items()
    }


def edge_failures(out_dir: Path, edges: list[Edge]) -> list[str]:
    failures = []
    early = [e.lateness for e in edges if e.lateness < 0]
    if early:
        failures.append(
            f"{out_dir}: {len(early)} step edges early, earliest {min(early)} "
            "instructions"
        )
    spreads_by_func = edge_spreads(edges)
    if not spreads_by_func:
        failures.append(f"{out_dir}: no step edge was judged")
    for func, spreads in spreads_by_func.items():
        paths = [s.path for s in spreads]
        if paths != ["irq", "loop"]:
            failures.append(
                f"{out_dir}: dispatch {func:#x} judged only on {paths}"
            )
            continue
        spread = max(s.latest for s in spreads) - min(
            s.earliest for s in spreads
        )
        if spread > EDGE_LATENESS_SPREAD_LIMIT:
            failures.append(
                f"{out_dir}: dispatch {func:#x} edge lateness spreads {spread} "
                f"instructions over the irq and loop paths "
                f"(limit {EDGE_LATENESS_SPREAD_LIMIT})"
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out_dirs", nargs="+", type=Path)
    args = parser.parse_args()
    failures = []
    for out_dir in args.out_dirs:
        if autoconf(out_dir).get("MCU_SIM") == "1":
            print(f"{out_dir}: MCU_SIM build reports simulated=1, not run")
            continue
        firmware = Firmware(out_dir)
        pulsed, both_edge = firmware.run_benchmark(precursors=False)
        firmware.run_benchmark(precursors=True)
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
                failures.append(
                    f"{out_dir}: {name} {value} instructions/step outside "
                    f"[{MIN_INSTRUCTIONS_PER_STEP}, {MAX_INSTRUCTIONS_PER_STEP}]"
                )
        for func, spreads in edge_spreads(firmware.edges).items():
            print(
                f"{out_dir}: dispatch {func:#x} step edge lateness: "
                + ", ".join(
                    f"{s.path} {s.earliest}..{s.latest} (n={s.count})"
                    for s in spreads
                )
                + " instructions"
            )
        failures += edge_failures(out_dir, firmware.edges)
    if failures:
        print("; ".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
