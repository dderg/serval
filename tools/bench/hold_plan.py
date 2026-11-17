"""Split suite rows into bench holds that fit lock.MAX_HOLD_S.

EXPECTED_ROW_S is a row's hardware seconds inside a hold: settle, config
push and FIRMWARE_RESTART, the print or session, the log fetch; not its
analysis, which runs after the hold. The values are the slowest of the
suites 20261116-044338 (quick), -053529, -071238, -131614 and -212819
(run.json written minus run dir created), rounded up to 10 s; the soak rows
are their prints' planned seconds plus the session overhead those suites
show. test_hold_plan.py holds the table against the newest summary under
~/bench/suites. A hold plans at most HOLD_ROWS_S of rows: the cap less a
RESTORE_RESERVE_S for a hold that must re-sync a tree another holder
replaced (rsync + build-native.sh on the Pi + klipper restart) and a
HOLD_MARGIN_S for a row running past its estimate. A row the table does not
know is planned as DEFAULT_ROW_S, a hold of its own.
"""

from __future__ import annotations

import lock
import soak_gcode

HOLD_MARGIN_S = 300.0
RESTORE_RESERVE_S = 300.0
HOLD_ROWS_S = lock.MAX_HOLD_S - HOLD_MARGIN_S - RESTORE_RESERVE_S
DEFAULT_ROW_S = HOLD_ROWS_S
PRINT_OVERHEAD_S = 80.0
SESSION_OVERHEAD_S = 120.0
SOAK_SESSION_ROW_S = (
    2 * soak_gcode.SESSION_PRINT.planned_print_s + SESSION_OVERHEAD_S
)
SOAK_LONG_ROW_S = soak_gcode.LONG_PRINT.planned_print_s + PRINT_OVERHEAD_S
EXPECTED_ROW_S = {
    "first_layers": 240,
    "speed_ladder": 180,
    "speed_ladder_fast": 170,
    "extrude_ladder": 140,
    "epochs": 100,
    "rest_ladder": 170,
    "dense_top": 120,
    "dense_top_fast_throttled": 120,
    "corner_flow": 130,
    "corner_budget_default": 120,
    "single_motor_cap": 110,
    "mesh_full_print": 240,
    "pause_park_resume_mesh": 240,
    "print_twice": 270,
    "cancel_then_print": 300,
    "pause_resume": 240,
    "pause_cancel": 110,
    "cancel_plain": 100,
    "pause_sparse": 190,
    "cancel_macro_mesh": 100,
    "end_print_mesh": 170,
    "firmware_restart": 110,
    "estop": 120,
    "estop_g4": 130,
    "soak_mesh_session": SOAK_SESSION_ROW_S,
    "soak_throttled": SOAK_LONG_ROW_S,
}


class PlanError(RuntimeError):
    pass


def expected_s(name: str) -> float:
    return EXPECTED_ROW_S.get(name, DEFAULT_ROW_S)


def fits(name: str, remaining_s: float) -> bool:
    return expected_s(name) + HOLD_MARGIN_S <= remaining_s


def plan(names: list[str]) -> list[list[str]]:
    holds: list[list[str]] = []
    used_s = HOLD_ROWS_S
    for name in names:
        row_s = expected_s(name)
        if row_s > HOLD_ROWS_S:
            raise PlanError(
                f"{name} expects {row_s:.0f} s of hardware, over the "
                f"{HOLD_ROWS_S:.0f} s one hold plans: shorten the row"
            )
        if used_s + row_s > HOLD_ROWS_S:
            holds.append([])
            used_s = 0.0
        holds[-1].append(name)
        used_s += row_s
    return holds


def planned_s(names: list[str]) -> float:
    return sum(expected_s(name) for name in names)
