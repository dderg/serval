"""The lookahead's profile shapes that the max_corner_accel bound
(chelper/corner_deviation.c) relies on, over random move sequences,
junction limits, cruise ratios and lazy flushes. For a move ending at a
junction passed at speed v, and the move starting there:

- a move that accelerates starts at its junction limit (max_start_v2),
  so its acceleration ends no later than (c - v) / accel before its end,
  c the peak of an accel-decel triangle from that start, c^2 =
  (max_start_v2 + 2 accel d + v^2) / 2 (prev_decel_time);
- a move starting at v decelerates no sooner than (c - v) / accel, c^2 =
  (max(v^2, 2 accel d) + v^2) / 2, the triangle down to a stop
  (next_decel_time);
- consecutive moves meet at one speed.
"""

import math
import random

import pytest

from klippy import toolhead

MOVES = 300
SEEDS = range(12)
RELATIVE = 1e-9
TIME_SLACK = 1e-9


class _Toolhead:
    junction_deviation = 0.0
    extra_axes = ()

    def __init__(self, rng):
        self.rng = rng
        self.max_accel = 10000.0
        self.max_velocity = 500.0
        self.max_accel_to_decel = self.max_accel

    def junction_limiter(self, prev_move, move, cap_v2):
        return min(cap_v2, self.rng.choice((0.0, 1.0, 0.2, 0.05)) * cap_v2)


def _moves(seed):
    rng = random.Random(seed)
    th = _Toolhead(rng)
    lookahead = toolhead.LookAheadQueue()
    pos, heading, flushed = [0.0, 0.0, 0.0, 0.0], 0.0, []
    for _ in range(MOVES):
        th.max_accel = rng.choice((2000.0, 10000.0, 30000.0))
        th.max_accel_to_decel = th.max_accel * (1.0 - rng.random())
        heading += rng.uniform(-math.pi, math.pi)
        length = math.exp(rng.uniform(math.log(0.01), math.log(50.0)))
        end = [
            pos[0] + length * math.cos(heading),
            pos[1] + length * math.sin(heading),
            0.0,
            0.0,
        ]
        move = toolhead.Move(th, pos, end, rng.uniform(5.0, 500.0))
        if lookahead.add_move(move) or rng.random() < 0.1:
            flushed += lookahead.flush(lazy=True)
        pos = end
    flushed += lookahead.flush()
    return flushed


def _end_v(move):
    return move.cruise_v - move.accel * move.decel_t


@pytest.mark.parametrize("seed", SEEDS)
def test_lookahead_profiles_have_the_shapes_the_bound_assumes(seed):
    moves = _moves(seed)
    assert len(moves) == MOVES
    for move in moves:
        a, d = move.accel, move.move_d
        if move.accel_t > 0.0:
            assert move.start_v**2 == pytest.approx(
                move.max_start_v2, rel=RELATIVE, abs=1e-9
            )
            end = _end_v(move)
            peak = math.sqrt(0.5 * (move.max_start_v2 + 2.0 * a * d + end**2))
            latest = (peak - end) / a
            assert move.cruise_t + move.decel_t >= latest - TIME_SLACK
        if move.decel_t > 0.0:
            start = move.start_v
            reach_v2 = max(start**2, 2.0 * a * d)
            lowest = math.sqrt(0.5 * (reach_v2 + start**2))
            earliest = (lowest - start) / a
            assert move.accel_t + move.cruise_t >= earliest - TIME_SLACK
    for prev, move in zip(moves, moves[1:]):
        assert _end_v(prev) == pytest.approx(move.start_v, rel=1e-6, abs=1e-6)
