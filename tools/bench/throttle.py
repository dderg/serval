"""Host throttling for the Pi: klipper CPU affinity and cpufreq ceiling."""

from __future__ import annotations

import contextlib
from collections.abc import Iterator
from dataclasses import dataclass

import pi

DROPIN_DIR = "/etc/systemd/system/klipper.service.d"
DROPIN = f"{DROPIN_DIR}/bench-throttle.conf"
CPUFREQ = "/sys/devices/system/cpu/cpufreq"
KHZ_PER_MHZ = 1000


@dataclass(frozen=True)
class Throttle:
    cpus: int | None
    max_mhz: int | None

    def record(self) -> dict:
        return {"host_cpus": self.cpus, "host_max_mhz": self.max_mhz}


def cpu_count() -> int:
    return int(pi.ssh("nproc").strip())


def policies() -> list[str]:
    names = pi.ssh(f"ls -d {CPUFREQ}/policy*").split()
    if not names:
        raise pi.BenchError(f"no cpufreq policies under {CPUFREQ}")
    return names


def read_khz(policy: str, attribute: str) -> int:
    return int(pi.ssh(f"cat {policy}/{attribute}").strip())


def discrete_khz(policy: str) -> list[int]:
    listing = pi.ssh(
        f"cat {policy}/scaling_available_frequencies 2>/dev/null || true"
    ).split()
    return sorted(int(freq) for freq in listing)


def validate_frequency(policy: str, khz: int) -> None:
    mhz = khz // KHZ_PER_MHZ
    discrete = discrete_khz(policy)
    if discrete:
        if khz not in discrete:
            raise pi.BenchError(
                f"{mhz} MHz not available on {policy}; "
                f"available MHz: {[f // KHZ_PER_MHZ for f in discrete]}"
            )
        return
    low = read_khz(policy, "cpuinfo_min_freq")
    high = read_khz(policy, "cpuinfo_max_freq")
    if not low <= khz <= high:
        raise pi.BenchError(
            f"{mhz} MHz outside {policy} range "
            f"{low // KHZ_PER_MHZ}..{high // KHZ_PER_MHZ} MHz"
        )


def write_max_khz(policy: str, khz: int) -> None:
    pi.ssh(f"echo {khz} | sudo tee {policy}/scaling_max_freq >/dev/null")
    applied = read_khz(policy, "scaling_max_freq")
    if applied != khz:
        raise pi.BenchError(
            f"{policy}/scaling_max_freq reads {applied}, expected {khz}"
        )


def apply(throttle: Throttle, original_khz: dict[str, int]) -> None:
    if throttle.cpus is not None:
        total = cpu_count()
        if not 1 <= throttle.cpus <= total:
            raise pi.BenchError(
                f"--host-cpus {throttle.cpus} outside 1..{total}"
            )
        pi.ssh(
            f"sudo mkdir -p {DROPIN_DIR} && printf "
            f"'[Service]\\nCPUAffinity=0-{throttle.cpus - 1}\\n' "
            f"| sudo tee {DROPIN} >/dev/null"
        )
    if throttle.max_mhz is not None:
        khz = throttle.max_mhz * KHZ_PER_MHZ
        for policy in original_khz:
            validate_frequency(policy, khz)
        for policy in original_khz:
            write_max_khz(policy, khz)
    pi.ssh("sudo systemctl daemon-reload")
    pi.restart_klipper()


def restore(original_khz: dict[str, int]) -> None:
    pi.ssh(f"sudo rm -f {DROPIN}")
    for policy, khz in original_khz.items():
        write_max_khz(policy, khz)
    pi.ssh("sudo systemctl daemon-reload")
    pi.restart_klipper()
    leftover = pi.ssh(f"ls {DROPIN} 2>/dev/null || true").strip()
    if leftover:
        raise pi.BenchError(f"throttle drop-in {leftover} still present")


@contextlib.contextmanager
def throttled(throttle: Throttle | None) -> Iterator[dict]:
    if throttle is None or (throttle.cpus is None and throttle.max_mhz is None):
        yield {}
        return
    original_khz = {
        policy: read_khz(policy, "scaling_max_freq") for policy in policies()
    }
    try:
        apply(throttle, original_khz)
        yield throttle.record()
    finally:
        restore(original_khz)
