"""A paced sleep on a clock that stopped is a wedge: it raises like every
other virtual wait. It spun forever before, which hid the scenario
fuzzer's seed 34 (a panicked shaper froze virtual time) behind a 900 s
worker budget."""

import pytest

from tools.sim.world import HarnessPacer, SimError, VirtualDeadline

pytestmark = pytest.mark.sim_unit

FROZEN_NS = 5_000_000_000
SHORT_WEDGE_S = 0.05


class FrozenClock:
    def now_ns(self) -> int:
        return FROZEN_NS

    def claim_pacer(self) -> int:
        return 0

    def hold_at(self, slot: int, vtime_ns: int) -> None:
        pass

    def release_pacer(self, slot: int) -> None:
        pass

    def holds(self) -> str:
        return "frozen test clock"


def test_a_sleep_on_a_frozen_clock_reports_the_wedge(monkeypatch):
    monkeypatch.setattr(VirtualDeadline, "WEDGE_REAL_S", SHORT_WEDGE_S)
    with HarnessPacer(FrozenClock()) as pacer:
        with pytest.raises(SimError, match="wedged"):
            pacer.sleep(0.01)
