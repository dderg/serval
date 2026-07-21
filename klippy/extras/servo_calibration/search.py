from __future__ import annotations

import math

from .dynamics import TUNE_RELATIVE_CLAMP

Z = 2.0
REFINE_Z = 1.0
MAX_REFINE_PROBES = 3


def _checked_sigma(sigma: float) -> float:
    if sigma is None or math.isnan(sigma):
        raise ValueError("sigma must be a finite number (got %r)" % (sigma,))
    if sigma < 0.0:
        raise ValueError("sigma must be non-negative (got %r)" % (sigma,))
    return sigma


class RmsLineSearch:
    """Empirical 1-D descent for one (term, mode) of SERVO_TUNE_DYNAMICS:
    the objective is the MEASURED transient-window following-error rms of
    that mode - the rms taken only over the short windows right after each
    commanded transition, where feedforward has authority before the inner
    servo loop (disturbance observer + integrator) corrects the excursion.
    Whole-capture rms diluted these transients ~10x and was blind to what
    FF controls; the correlation nulls far from the rms minimum (the bench
    walked mass to 2.2x baseline while raw rms rose every round), so
    correlations are demoted to direction hints and diagnostics. Protocol:
    construct with the already-measured start point, run the trial in
    `trial`, `feed` the measured (rms, sigma), repeat until `done`; `best`
    then holds the winner. An improvement counts only when it clears a 2-
    sigma deadband measured from the per-window rms scatter
    (rms < best_rms - Z*hypot(sigma, best_sigma)) - there is no fixed
    tolerance knob. First probe follows `hint`'s sign - except starting AT
    the lower bound, where down is unactionable and the probe goes up
    regardless (a zero-valued friction term with a downhill regression
    hint would otherwise finish with zero probes, untested); a failed
    first probe flips once, a march grows the step (capped at `clamp` of
    the current value) while the rms keeps clearing the deadband, and the
    first non-improving trial starts an iterative parabolic refine through
    the bracket around the best point: up to MAX_REFINE_PROBES vertex
    probes, each accepted at the looser REFINE_Z (the bracket is already
    established, so a 1-sigma win inside it is real more often than not,
    and the march's 2-sigma gate left genuine few-percent refinements
    permanently stuck on the incumbent - the bench pinned ff lead to its
    first march probe for a whole day this way). A REFINE_Z-only accept
    moves `best` but does NOT set `improved`: `improved` means the full
    2-sigma gate was cleared and is what the tune's outer pass loop keys
    on - counting sub-noise polish as pass improvement made the bench
    orbit one lead value for 50 captures. Trials clamp to `lo`; a
    trial that lands on an already-measured value ends the search."""

    def __init__(
        self,
        value: float,
        rms: float,
        sigma: float,
        step: float,
        lo: float = 0.0,
        hi: float = math.inf,
        hint: float = 1.0,
        grow: float = 1.6,
        clamp: float = TUNE_RELATIVE_CLAMP,
    ):
        if step <= 0.0:
            raise ValueError("step must be positive (got %r)" % (step,))
        if value < lo:
            raise ValueError(
                "start value %.6g below lower bound %.6g" % (value, lo)
            )
        if value > hi:
            raise ValueError(
                "start value %.6g above upper bound %.6g" % (value, hi)
            )
        self.best = value
        self.best_rms = rms
        self.best_sigma = _checked_sigma(sigma)
        self.lo = lo
        self.hi = hi
        self.step = step
        self.direction = 1.0 if hint >= 0.0 or value == lo else -1.0
        self.grow = grow
        self.clamp = clamp
        self.history: list[tuple[float, float, float]] = [(value, rms, sigma)]
        self.trial: float | None = None
        self.done = False
        self.note = ""
        self.improved = False
        self._flipped = False
        self._refine_probes = 0
        self._advance_from(value)

    def _tried(self, value: float) -> bool:
        return any(
            abs(value - v) <= 1e-12 + 1e-9 * abs(value)
            for v, _r, _s in self.history
        )

    def _finish(self, note: str) -> None:
        self.done = True
        self.trial = None
        self.note = note

    def _advance_from(self, value: float) -> None:
        trial = min(max(value + self.direction * self.step, self.lo), self.hi)
        if self._tried(trial):
            if trial == self.lo:
                self._finish("bounded at %.6g" % (self.lo,))
            elif trial == self.hi:
                self._finish("bounded at %.6g" % (self.hi,))
            else:
                self._finish("no further improvement")
            return
        self.trial = trial

    def _parabolic_vertex(self) -> float | None:
        points = sorted(self.history, key=lambda p: p[0])
        for i in range(1, len(points) - 1):
            v0, r0, _s0 = points[i]
            if v0 != self.best:
                continue
            (va, ra, _sa), (vb, rb, _sb) = points[i - 1], points[i + 1]
            num = (v0 - va) ** 2 * (r0 - rb) - (v0 - vb) ** 2 * (r0 - ra)
            den = (v0 - va) * (r0 - rb) - (v0 - vb) * (r0 - ra)
            if den == 0.0:
                return None
            vertex = v0 - 0.5 * num / den
            if not va < vertex < vb:
                return None
            return min(max(vertex, self.lo), self.hi)
        return None

    def feed(self, rms: float, sigma: float) -> None:
        if self.done or self.trial is None:
            raise ValueError("feed() without an outstanding trial")
        sigma = _checked_sigma(sigma)
        value = self.trial
        self.history.append((value, rms, sigma))
        deadband = math.hypot(sigma, self.best_sigma)
        significant = rms < self.best_rms - Z * deadband
        accepted = significant or (
            self._refine_probes > 0
            and rms < self.best_rms - REFINE_Z * deadband
        )
        if accepted:
            self.best = value
            self.best_rms = rms
            self.best_sigma = sigma
            self.improved = self.improved or significant
            if self._refine_probes:
                self._refine_or_finish("refined to the bracket minimum")
                return
            cap = self.clamp * abs(value)
            self.step = (
                min(self.step * self.grow, cap)
                if cap > 0.0
                else (self.step * self.grow)
            )
            self._advance_from(value)
            return
        if self._refine_probes:
            self._refine_or_finish("refine did not beat the bracket best")
            return
        if not self.improved and not self._flipped:
            self._flipped = True
            self.direction = -self.direction
            self._advance_from(self.best)
            return
        self._refine_or_finish(
            "no further improvement"
            if self.improved
            else "start already optimal"
        )

    def _refine_or_finish(self, note: str) -> None:
        if self._refine_probes >= MAX_REFINE_PROBES:
            self._finish(note)
            return
        vertex = self._parabolic_vertex()
        if vertex is None or self._tried(vertex):
            self._finish(note)
            return
        self._refine_probes += 1
        self.trial = vertex
