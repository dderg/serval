from . import structured_log

UNBOUNDED = float("inf")
SLOW_WAIT_LOG_S = 1.0
# Fallback ceiling between polls when parked on the engine wakeup fd: the fd
# wakes the waiter the moment the engine has something for it, so this only
# bounds how long a lost wakeup could stall before the next poll notices.
PARK_FALLBACK_S = 0.5


class EngineWaitTimeout(Exception):
    def __init__(self, what, deadline_s):
        Exception.__init__(
            self,
            "%s: motion engine wait timed out after %.1fs" % (what, deadline_s),
        )
        self.what = what
        self.deadline_s = deadline_s


def wait_for(printer, poll, what, deadline_s, interval_s=0.005, park=None):
    """Poll `poll` until it returns a non-None value. Uniform semantics for
    every host-side wait on the motion engine: shutdown always aborts the
    wait, a finite `deadline_s` raises EngineWaitTimeout, and waits slower
    than SLOW_WAIT_LOG_S emit a structured-log event. Pass UNBOUNDED as
    `deadline_s` only where the wait is legitimately unbounded (e.g. draining
    queued motion that may include arbitrarily long dwells). `park`, when
    given, replaces the interval sleep: it blocks until the engine's wakeup
    fd fires (or PARK_FALLBACK_S lapses), so the waiter resumes the moment
    the engine frees space or resolves a fence instead of on a poll grid."""
    reactor = printer.get_reactor()
    start = reactor.monotonic()
    slow_logged = False
    while True:
        result = poll()
        if result is not None:
            if slow_logged:
                structured_log.event(
                    "motion",
                    "engine_wait_done",
                    what=what,
                    waited_s=round(reactor.monotonic() - start, 3),
                )
            return result
        if printer.is_shutdown():
            raise printer.command_error(
                "%s: shutdown while waiting on the motion engine" % (what,)
            )
        waited = reactor.monotonic() - start
        if waited >= deadline_s:
            raise EngineWaitTimeout(what, deadline_s)
        if not slow_logged and waited >= SLOW_WAIT_LOG_S:
            slow_logged = True
            structured_log.event(
                "motion",
                "engine_wait_slow",
                what=what,
                waited_s=round(waited, 3),
            )
        if park is not None:
            park(PARK_FALLBACK_S)
        else:
            reactor.pause(reactor.monotonic() + interval_s)
