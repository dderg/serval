from __future__ import annotations

import re

import pi
import pytest

BEFORE = {
    0x40023830: 0x00000003,
    0x40023808: 0x0000900A,
    0x40020008: 0x8EA80008,
    0x40020024: 0x000AA000,
    0x40020000: 0x2A800004,
}


def _rmw_line(address: int, before: int, after: int) -> str:
    return f"BENCH_RMW {address:#010x} {before:#010x} {after:#010x}"


def _output(after: dict[int, int] | None = None, window: str = "4") -> str:
    lines = ["Info : SWD DPIDR 0x2ba01477 DPv1"]
    for address, set_bits, clear_bits in pi.MCO_WRITES:
        before = BEFORE[address]
        written = (before & ~clear_bits) | set_bits
        lines.append(
            _rmw_line(address, before, (after or {}).get(address, written))
        )
    if window:
        lines.append(f"BENCH_HALT_WINDOW_MS {window}")
    lines.append("shutdown command invoked")
    return "\n".join(lines) + "\n"


def test_every_register_write_runs_inside_one_halted_window():
    commands = pi.mco_openocd_commands()
    window = re.fullmatch(
        r"init; bench_while_halted \{(.*)\}; shutdown", commands
    )
    assert window
    assert window.group(1).split("; ") == [
        f"bench_rmw {a:#010x} {s:#010x} {c:#010x}" for a, s, c in pi.MCO_WRITES
    ]


def test_the_live_config_defines_the_procs_the_commands_call():
    config = pi.MCO_LIVE_CFG.read_text()
    for proc in (
        "bench_while_halted",
        "bench_rmw",
        "bench_require_trusted_bus_reads",
    ):
        assert f"proc {proc} " in config


def test_registers_reading_back_as_written_report_the_halt_window():
    assert pi.checked_mco_halt_window_ms(_output(window="6")) == 6


def test_a_register_reading_back_other_than_written_fails_loudly():
    with pytest.raises(pi.BenchError, match="0x40020000 read back 0x00000000"):
        pi.checked_mco_halt_window_ms(_output(after={0x40020000: 0}))


def test_a_register_without_a_read_back_fails_loudly():
    output = _output().replace(
        _rmw_line(0x40023808, 0x0000900A, 0x0640900A), ""
    )
    with pytest.raises(pi.BenchError, match="no read-back of 0x40023808"):
        pi.checked_mco_halt_window_ms(output)


def test_output_without_a_halt_window_fails_loudly():
    with pytest.raises(pi.BenchError, match="no halt window"):
        pi.checked_mco_halt_window_ms(_output(window=""))


def test_a_halt_window_over_the_limit_fails_loudly():
    over = str(pi.MCO_HALT_WINDOW_LIMIT_MS + 1)
    with pytest.raises(pi.BenchError, match="over the 50 ms limit"):
        pi.checked_mco_halt_window_ms(_output(window=over))


def test_enable_mco_runs_the_local_live_config_on_the_pi(monkeypatch):
    pushed = []
    commands = []
    monkeypatch.setattr(
        pi, "scp_to_pi", lambda local, remote: pushed.append((local, remote))
    )

    def ssh(command):
        commands.append(command)
        return _output()

    monkeypatch.setattr(pi, "ssh", ssh)
    assert pi.enable_mco() == 4
    assert pushed == [(pi.MCO_LIVE_CFG, pi.PI_MCO_LIVE_CFG)]
    assert commands == [
        f"openocd -f {pi.PI_MCO_LIVE_CFG} -c '{pi.mco_openocd_commands()}' 2>&1"
    ]
