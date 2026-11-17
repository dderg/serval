"""The hunt driver sizes each batch to the slot time it targets."""

import pytest

from tools.sim import hunt

pytestmark = pytest.mark.sim_unit

TEN_MINUTES_S = 600.0


def test_a_batch_fills_the_target_at_the_last_rate():
    assert hunt.next_size(40, 300.0, TEN_MINUTES_S) == 80
    assert hunt.next_size(40, 1200.0, TEN_MINUTES_S) == 20


def test_a_batch_stays_inside_its_bounds():
    assert hunt.next_size(1, 3600.0, TEN_MINUTES_S) == hunt.MIN_BATCH
    assert hunt.next_size(1000, 10.0, TEN_MINUTES_S) == hunt.MAX_BATCH
    assert hunt.next_size(0, 100.0, TEN_MINUTES_S) == hunt.MIN_BATCH
