import pytest

from tools.sim.world import LOG_FENCE_NEEDLE, SimError, log_through_fence

pytestmark = pytest.mark.sim_unit

DECISION = b"homing: Y needs rehome: True (traveled=0.5 min_home_dist=5.0)\n"
FENCE = b"FIRMWARE_NAME:Kalico\n"


class LaggingLog:
    """A klippy.log whose writer thread lands each queued record only after
    `lag` further reads, the way a starved runner delays it."""

    def __init__(self, queued, lag):
        self.queued = list(queued)
        self.lag = lag
        self.data = b""
        self.reads = 0

    def read(self):
        self.reads += 1
        if self.queued and self.reads > self.lag:
            self.data += self.queued.pop(0)
        return self.data


def never_expires():
    return False


def test_a_record_logged_before_the_fence_is_returned_even_when_the_writer_lags():
    log = LaggingLog([DECISION, FENCE], lag=3)
    data = log_through_fence(log.read, 0, never_expires, lambda: None)
    assert DECISION in data and LOG_FENCE_NEEDLE in data


def test_a_fence_that_predates_the_offset_does_not_count():
    log = LaggingLog([FENCE], lag=0)
    stale = log.read()
    log.queued = [DECISION]
    expired = iter([False, True])
    with pytest.raises(SimError, match="did not appear"):
        log_through_fence(
            log.read, len(stale), lambda: next(expired), lambda: None
        )


def test_a_fence_that_never_lands_fails_loudly():
    log = LaggingLog([DECISION], lag=0)
    expired = iter([False, False, True])
    with pytest.raises(SimError, match="did not appear"):
        log_through_fence(log.read, 0, lambda: next(expired), lambda: None)
