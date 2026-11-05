"""Check a firmware build's endstop trip timing in an instruction emulator.

uv run --with unicorn --with pyelftools python tools/bench/endstop_edge_emu.py \
    <out_dir>...

Each <out_dir> holds a built klipper.elf and its autoconf.h (STM32 builds).
One instruction is one timer tick. The emulator configures endstops through
the firmware's own commands, arms them with a short poll period, drives a pin
active at a chosen instruction and delivers the interrupts the firmware
enabled (its EXTI lines and its timer) as the core would, once due and with
PRIMASK clear. It then reads the trip clock the endstop stamped and notes when
the stop gate engaged (the write that latches `stop_gated`):

- edge: the pin goes active while the MCU idles between polls. The trip clock
  must lie within EDGE_LATENCY_LIMIT instructions after the edge and the stop
  gate must engage within STOP_LATENCY_LIMIT: the firmware timestamped the
  edge itself and stopped from there, not from the poll that would have seen
  it up to a poll period later.
- armed active: the pin is already active when armed, so no edge can fire.
  The first poll must catch it, stamping the midpoint between arming and
  that poll.
- shared line: a second endstop on another port's pin of the same number
  cannot take that EXTI line and must fall back to the poll, stamping the
  midpoint of the poll window that saw the pin.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from elftools.elf.elffile import ELFFile
from step_benchmark_emu import (
    DWT_CYCCNT,
    G0_TIM2_CNT,
    PAGE,
    THUMB_NOP,
    EmulationError,
    TimerInterrupt,
    autoconf,
    code_address,
    cpu_model,
    symbol,
)
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
    UC_ARM_REG_PRIMASK,
    UC_ARM_REG_R0,
    UC_ARM_REG_SP,
)

EDGE_LATENCY_LIMIT = 60
STOP_LATENCY_LIMIT = 400
POLL_SLACK = 250
REST_TICKS = 4000
INSTRUCTION_LIMIT = 2_000_000
ENDSTOP_UNBOUND = 0xFF
TRSYNC_TRIGGER_REASON = 1
TRSYNC_EXPIRE_REASON = 4
LINE = 9
PIN_PA9 = LINE
PIN_PB9 = 16 + LINE
STACK_BASE = 0x70000000
STACK_SIZE = 0x100000
STOP = 0x70200000
IRQ_RETURN = STOP + 0x100
CALL_RETURN = STOP + 0x200
IDLE = STOP + 0x300
ARGS = STOP + 0x400
THUMB_BRANCH_SELF = b"\xfe\xe7"
NVIC_ISER = 0xE000E100
NVIC_ICER = 0xE000E180
NVIC_WORDS = 8


@dataclass(frozen=True)
class Family:
    gpio_a: int
    gpio_b: int
    idr: int
    exti: int
    rising: int
    falling: int
    mask: int
    pending: tuple[int, ...]
    line_irq: dict[int, int]


def _irqs(groups: list[tuple[range, int]]) -> dict[int, int]:
    return {line: irq for lines, irq in groups for line in lines}


CLASSIC_IRQS = _irqs(
    [
        (range(0, 1), 6),
        (range(1, 2), 7),
        (range(2, 3), 8),
        (range(3, 4), 9),
        (range(4, 5), 10),
        (range(5, 10), 23),
        (range(10, 16), 40),
    ]
)
FAMILIES = {
    "f1": Family(
        gpio_a=0x40010800,
        gpio_b=0x40010C00,
        idr=0x08,
        exti=0x40010400,
        rising=0x08,
        falling=0x0C,
        mask=0x00,
        pending=(0x14,),
        line_irq=CLASSIC_IRQS,
    ),
    "f4": Family(
        gpio_a=0x40020000,
        gpio_b=0x40020400,
        idr=0x10,
        exti=0x40013C00,
        rising=0x08,
        falling=0x0C,
        mask=0x00,
        pending=(0x14,),
        line_irq=CLASSIC_IRQS,
    ),
    "g0": Family(
        gpio_a=0x50000000,
        gpio_b=0x50000400,
        idr=0x10,
        exti=0x40021800,
        rising=0x00,
        falling=0x04,
        mask=0x80,
        pending=(0x0C, 0x10),
        line_irq=_irqs([(range(0, 2), 5), (range(2, 4), 6), (range(4, 16), 7)]),
    ),
    "h7": Family(
        gpio_a=0x58020000,
        gpio_b=0x58020400,
        idr=0x10,
        exti=0x58000000,
        rising=0x00,
        falling=0x04,
        mask=0x80,
        pending=(0x88,),
        line_irq=CLASSIC_IRQS,
    ),
}


def family_of(conf: dict[str, str]) -> str:
    for name in ("F1", "F4", "G0", "H7"):
        if conf.get(f"MACH_STM32{name}") == "1":
            return name.lower()
    raise EmulationError("not an STM32 build")


def struct_offsets(elf: ELFFile, name: str) -> dict[str, int]:
    for cu in elf.get_dwarf_info().iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag != "DW_TAG_structure_type" or not die.has_children:
                continue
            attr = die.attributes.get("DW_AT_name")
            if attr is None or attr.value.decode() != name:
                continue
            found = {
                child.attributes["DW_AT_name"].value.decode(): child.attributes[
                    "DW_AT_data_member_location"
                ].value
                for child in die.iter_children()
                if child.tag == "DW_TAG_member"
                and "DW_AT_data_member_location" in child.attributes
            }
            if found:
                return found
    raise EmulationError(f"no DWARF struct {name}")


class Machine:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        conf = autoconf(out_dir)
        self.family = FAMILIES[family_of(conf)]
        g0 = conf.get("MACH_STM32G0") == "1"
        model, self.exception_entry_cycles = cpu_model(conf)
        with open(out_dir / "klipper.elf", "rb") as handle:
            elf = ELFFile(handle)
            self.vectors = symbol(elf, "VectorTable")[0]
            self.oids = symbol(elf, "oids")[0]
            alloc_end = symbol(elf, "alloc_end")[0]
            heap = symbol(elf, "_persistent_diag_end")[0]
            self.move_count = symbol(elf, "move_count")[0]
            self.stop_gated = symbol(elf, "stop_gated")[0]
            self.shutdown = code_address(elf, "sched_shutdown")
            self.timer_handler = code_address(
                elf, "TIMx_IRQHandler" if g0 else "SysTick_Handler"
            )
            self.allocate_oids = code_address(elf, "command_allocate_oids")
            self.config_endstop = code_address(elf, "command_config_endstop")
            self.query_endstop = code_address(elf, "command_query_endstop")
            self.config_trsync = code_address(elf, "command_config_trsync")
            self.trsync_start = code_address(elf, "command_trsync_start")
            self.arm_trsync = code_address(elf, "command_endstop_arm_trsync")
            self.endstop = struct_offsets(elf, "endstop")
            self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
            self.uc.ctl_set_cpu_model(model)
            self._map(elf, int(conf["RAM_START"], 0), int(conf["RAM_SIZE"], 0))
        self.uc.mem_write(alloc_end, struct.pack("<I", (heap + 7) & ~7))
        self.uc.mem_write(self.move_count, struct.pack("<I", 0))
        self.executed = 0
        self.until: int | None = None
        self.masked = False
        self.shut_down = False
        self.timer = TimerInterrupt(g0)
        self.timer_register = G0_TIM2_CNT if g0 else DWT_CYCCNT
        self.nvic = [0] * NVIC_WORDS
        self.exti: dict[int, int] = {}
        self.pending = 0
        self.edge_pin: int | None = None
        self.edge_time: int | None = None
        self.edge_seen = False
        self.pending_irq: int | None = None
        self.gated_at: int | None = None
        self.uc.hook_add(UC_HOOK_CODE, self._tick)
        self.uc.hook_add(
            UC_HOOK_MEM_WRITE,
            self._gated,
            begin=self.stop_gated,
            end=self.stop_gated,
        )
        for base, size in ((0x40000000, 0x20000000), (0xE0000000, 0x100000)):
            self.uc.mmio_map(
                base,
                size,
                lambda e, o, s, u, b=base: self._read(b + o),
                None,
                lambda e, o, s, v, u, b=base: self._write(b + o, v),
                None,
            )

    def _map(self, elf: ELFFile, ram_start: int, ram_size: int) -> None:
        uc = self.uc
        ram = range(ram_start // PAGE, (ram_start + ram_size + PAGE - 1) // PAGE)
        pages = set(ram)
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD" or not seg["p_memsz"]:
                continue
            for start, size in (
                (seg["p_vaddr"], seg["p_memsz"]),
                (seg["p_paddr"], seg["p_filesz"]),
            ):
                pages.update(range(start // PAGE, (start + size + PAGE - 1) // PAGE))
        for page in sorted(pages):
            uc.mem_map(page * PAGE, PAGE)
        for seg in elf.iter_segments():
            if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]:
                data = seg.data()
                uc.mem_write(seg["p_paddr"], data)
                uc.mem_write(seg["p_vaddr"], data)
        uc.mem_map(STACK_BASE, STACK_SIZE + PAGE)
        uc.mem_map(STOP, PAGE)
        for sentinel in (STOP, IRQ_RETURN, CALL_RETURN):
            uc.mem_write(sentinel, THUMB_NOP * 4)
        uc.mem_write(IDLE, THUMB_BRANCH_SELF)

    def word(self, address: int, size: int = 4) -> int:
        fmt = {1: "<B", 4: "<I", 8: "<Q"}[size]
        return struct.unpack(fmt, self.uc.mem_read(address, size))[0]

    def pin_level(self, port_base: int) -> int:
        if self.edge_pin is None or self.executed < self.edge_time:
            return 0
        port = self.family.gpio_a if self.edge_pin < 16 else self.family.gpio_b
        return 1 << (self.edge_pin % 16) if port == port_base else 0

    def _read(self, address: int) -> int:
        family = self.family
        if address == self.timer_register:
            return self.executed & 0xFFFFFFFF
        if address in (family.gpio_a + family.idr, family.gpio_b + family.idr):
            return self.pin_level(address - family.idr)
        offset = address - family.exti
        if offset in family.pending:
            return self.pending
        return self.exti.get(offset, 0)

    def _write(self, address: int, value: int) -> None:
        self.timer.write(address, value, self.executed)
        if NVIC_ISER <= address < NVIC_ISER + 4 * NVIC_WORDS:
            self.nvic[(address - NVIC_ISER) // 4] |= value
        elif NVIC_ICER <= address < NVIC_ICER + 4 * NVIC_WORDS:
            self.nvic[(address - NVIC_ICER) // 4] &= ~value
        offset = address - self.family.exti
        if offset in self.family.pending:
            self.pending &= ~value
        elif 0 <= offset < 0x100:
            self.exti[offset] = value

    def _irq_enabled(self, irq: int) -> bool:
        return bool(self.nvic[irq // 32] & (1 << (irq % 32)))

    def _due_exti(self) -> int | None:
        armed = self.pending & self.exti.get(self.family.mask, 0)
        for line in range(16):
            irq = self.family.line_irq[line]
            if armed & (1 << line) and self._irq_enabled(irq):
                return irq
        return None

    def _tick(self, emu, address, _size, _user):
        self.executed += 1
        if address == self.shutdown:
            self.shut_down = True
            emu.emu_stop()
            return
        if self.executed > INSTRUCTION_LIMIT:
            emu.emu_stop()
            return
        if (
            self.edge_time is not None
            and not self.edge_seen
            and self.executed >= self.edge_time
        ):
            self.edge_seen = True
            line_bit = 1 << (self.edge_pin % 16)
            if self.exti.get(self.family.rising, 0) & line_bit:
                self.pending |= line_bit
        if self.masked or emu.reg_read(UC_ARM_REG_PRIMASK):
            return
        irq = self._due_exti()
        if irq is None and self.timer.due is not None:
            if self.executed - self.timer.due >= 0:
                irq = -1
        if irq is not None:
            self.pending_irq = irq
            emu.emu_stop()
        elif self.until is not None and self.executed >= self.until:
            emu.emu_stop()

    def require_running(self) -> None:
        if self.shut_down:
            raise EmulationError(f"{self.out_dir}: the endstop shut the MCU down")
        if self.executed > INSTRUCTION_LIMIT:
            raise EmulationError(
                f"{self.out_dir}: ran past {INSTRUCTION_LIMIT} instructions"
            )

    def _subroutine(self, function: int, sentinel: int) -> None:
        self.masked = True
        self.uc.reg_write(UC_ARM_REG_LR, sentinel | 1)
        self.uc.emu_start(function | 1, sentinel)
        self.masked = False
        self.require_running()

    def call(self, function: int, *args: int) -> None:
        self.uc.mem_write(ARGS, struct.pack(f"<{len(args)}I", *args))
        self.uc.reg_write(UC_ARM_REG_SP, STACK_BASE + STACK_SIZE)
        self.uc.reg_write(UC_ARM_REG_R0, ARGS)
        self._subroutine(function, CALL_RETURN)

    def _take(self, irq: int) -> None:
        if irq < 0:
            self.timer.due = None
            handler = self.timer_handler
        else:
            handler = self.word(self.vectors + 4 * (16 + irq)) & ~1
        self.executed += self.exception_entry_cycles
        interrupted = self.uc.context_save()
        self._subroutine(handler, IRQ_RETURN)
        self.uc.context_restore(interrupted)

    def idle_until(self, until: int) -> None:
        self.until = until
        self.uc.reg_write(UC_ARM_REG_SP, STACK_BASE + STACK_SIZE)
        while self.executed < until:
            self.pending_irq = None
            self.uc.emu_start(IDLE | 1, STOP)
            self.require_running()
            if self.pending_irq is not None:
                self._take(self.pending_irq)
        self.until = None

    def setup(self, pins: list[int]) -> None:
        self.trsync_oid = len(pins)
        self.call(self.allocate_oids, len(pins) + 1)
        for oid, pin in enumerate(pins):
            self.call(
                self.config_endstop,
                oid,
                oid,
                pin,
                1,
                0,
                ENDSTOP_UNBOUND,
                ENDSTOP_UNBOUND,
                0,
            )
        self.call(self.config_trsync, self.trsync_oid)

    def arm(self, oid: int) -> int:
        self.call(self.trsync_start, self.trsync_oid, 0, 0, TRSYNC_EXPIRE_REASON)
        self.call(self.arm_trsync, oid, self.trsync_oid, TRSYNC_TRIGGER_REASON)
        armed = self.executed
        self.call(self.query_endstop, oid, REST_TICKS)
        return armed

    def endstop_field(self, oid: int, field: str, size: int) -> int:
        data = self.word(self.word(self.oids) + 8 * oid + 4)
        return self.word(data + self.endstop[field], size)

    def trip(self, oid: int) -> int:
        if not self.endstop_field(oid, "tripped", 1) & 0xFF:
            raise EmulationError(f"{self.out_dir}: endstop {oid} never tripped")
        return self.endstop_field(oid, "trip_clock", 8) & 0xFFFFFFFF

    def _gated(self, _emu, _access, _address, _size, value, _user):
        if value and self.gated_at is None:
            self.gated_at = self.executed

    def stop_lag(self) -> int:
        if self.gated_at is None:
            raise EmulationError(f"{self.out_dir}: the stop gate never engaged")
        return self.gated_at - self.edge_time


def edge_failures(out_dir: Path) -> tuple[dict, list[str]]:
    failures = []
    edge = Machine(out_dir)
    edge.setup([PIN_PA9])
    armed = edge.arm(0)
    edge.edge_pin = PIN_PA9
    edge.edge_time = armed + REST_TICKS + REST_TICKS // 10
    edge.idle_until(edge.edge_time + 3 * REST_TICKS)
    trip_lag = edge.trip(0) - edge.edge_time
    stop_lag = edge.stop_lag()
    for name, lag, limit in (
        ("trip clock", trip_lag, EDGE_LATENCY_LIMIT),
        ("stop gate", stop_lag, STOP_LATENCY_LIMIT),
    ):
        if not 0 <= lag <= limit:
            failures.append(
                f"{out_dir}: edge: {name} {lag:+d} instructions from the edge, "
                f"outside [0, {limit}]"
            )

    armed_active = Machine(out_dir)
    armed_active.setup([PIN_PA9])
    armed_active.edge_pin = PIN_PA9
    armed_active.edge_time = 0
    armed = armed_active.arm(0)
    armed_active.idle_until(armed + 3 * REST_TICKS)
    midpoint_lag = armed_active.trip(0) - (armed + REST_TICKS // 2)
    if not 0 <= midpoint_lag <= POLL_SLACK:
        failures.append(
            f"{out_dir}: armed active: trip clock {midpoint_lag:+d} from the "
            f"midpoint of arming and the first poll, outside [0, {POLL_SLACK}]"
        )

    shared = Machine(out_dir)
    shared.setup([PIN_PA9, PIN_PB9])
    armed = shared.arm(1)
    shared.edge_pin = PIN_PB9
    shared.edge_time = armed + REST_TICKS + REST_TICKS * 9 // 10
    shared.idle_until(shared.edge_time + 3 * REST_TICKS)
    window_midpoint = armed + REST_TICKS + REST_TICKS // 2
    shared_lag = shared.trip(1) - window_midpoint
    if not 0 <= shared_lag <= POLL_SLACK:
        failures.append(
            f"{out_dir}: shared line: trip clock {shared_lag:+d} from the poll "
            f"window midpoint, outside [0, {POLL_SLACK}]"
        )
    report = {"edge_trip": trip_lag, "edge_stop": stop_lag}
    return report, failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out_dirs", nargs="+", type=Path)
    args = parser.parse_args()
    failures = []
    for out_dir in args.out_dirs:
        if autoconf(out_dir).get("MCU_SIM") == "1":
            print(f"{out_dir}: MCU_SIM build, not run")
            continue
        report, found = edge_failures(out_dir)
        print(
            f"{out_dir}: endstop edge trip +{report['edge_trip']}, stop gate "
            f"+{report['edge_stop']} instructions after the edge"
        )
        failures += found
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
