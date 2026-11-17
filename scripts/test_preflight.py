from __future__ import annotations

import json
import re
from pathlib import Path

import preflight
import pytest
from preflight_gcode import FrontEnd, Origin, Replay, Treatment, treatment

from klippy.extras.gcode_move import UNSUPPORTED_MOTION

ROOT = Path(__file__).resolve().parent.parent
BENCH_CONFIG = ROOT / "tools" / "bench" / "printer.cfg"
REFUSED = (
    ROOT / "rust" / "pipeline-snapshot" / "src" / "waypoints" / "refused.rs"
)
WAYPOINTS = ROOT / "rust" / "pipeline-snapshot" / "src" / "waypoints.rs"

BED_MESH = """
[bed_mesh]
mesh_min: 80, 80
mesh_max: 140, 140
probe_count: 3, 3
zero_reference_position: 110, 110
fade_start: 1
fade_end: 10
fade_target: 0
"""

SAVED_PROFILE = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh {name}]
#*# version = 1
#*# points =
#*#   0.10, 0.00, -0.10
#*#   0.05, 0.00, -0.05
#*#   -0.10, 0.00, 0.10
#*# min_x = 80.0
#*# max_x = 140.0
#*# min_y = 80.0
#*# max_y = 140.0
#*# x_count = 3
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
"""

MACROS = """
[gcode_macro PURGE]
gcode:
  G1 X5 Y5 F6000
  G1 X50 E10

[gcode_macro START]
gcode:
  G28
  PURGE
  G1 Z{params.LAYER|default(0.2)|float}

[gcode_macro HEAT_SOAK]
gcode:
  {% if printer.heater_bed.temperature < 60 %}M190 S60{% endif %}

[gcode_macro PAUSE]
rename_existing: BASE_PAUSE
gcode:
  BASE_PAUSE
  G1 Z5
"""

PRINT = "START LAYER=0.3\nHEAT_SOAK\nG1 X10\nPAUSE\nG2 X1 Y1 I1 J0\n"


def printer_cfg(tmp_path: Path, *sections: str) -> Path:
    path = tmp_path / "printer.cfg"
    path.write_text(BENCH_CONFIG.read_text() + "".join(sections))
    return path


def test_a_saved_profile_reaches_the_engine_as_klippy_hands_it(tmp_path):
    front = FrontEnd(
        printer_cfg(tmp_path, BED_MESH, SAVED_PROFILE.format(name="wavy"))
    )
    assert front.mesh_profiles() == ["wavy"]
    assert front.load_mesh_profile("wavy").request == {
        "points": [0.1, 0.0, -0.1, 0.05, 0.0, -0.05, -0.1, 0.0, 0.1],
        "x_min": 80.0,
        "y_min": 80.0,
        "dx": 30.0,
        "dy": 30.0,
        "nx": 3,
        "ny": 3,
        "tension": 0.2,
        "fade": (1.0, 10.0, 0.0),
        "zero_ref_x": 110.0,
        "zero_ref_y": 110.0,
        "z_velocity_limit": None,
        "z_accel_limit": None,
    }


def test_the_replay_takes_the_mesh_the_print_loads(tmp_path):
    front = FrontEnd(
        printer_cfg(tmp_path, BED_MESH, SAVED_PROFILE.format(name="wavy"))
    )
    front.expand("G1 X1\nBED_MESH_PROFILE LOAD=wavy\nG1 X2\nBED_MESH_CLEAR\n")
    choice = preflight.choose_mesh(front, None)
    assert (choice.change.profile, choice.change.origin) == ("wavy", Origin(2))
    cleared = preflight.unreplayed_mesh_changes(front, choice)
    assert [(c.profile, c.origin) for c in cleared] == [(None, Origin(4))]


def test_without_a_load_the_replay_takes_the_default_profile(tmp_path):
    front = FrontEnd(
        printer_cfg(tmp_path, BED_MESH, SAVED_PROFILE.format(name="default"))
    )
    front.expand("G1 X1\n")
    assert preflight.choose_mesh(front, None).change.profile == "default"


def test_an_unknown_mesh_profile_is_refused(tmp_path):
    front = FrontEnd(
        printer_cfg(tmp_path, BED_MESH, SAVED_PROFILE.format(name="wavy"))
    )
    with pytest.raises(preflight.PreflightError, match="--mesh flat"):
        preflight.choose_mesh(front, "flat")


def test_macros_expand_with_their_origin_and_unevaluable_ones_are_skipped(
    tmp_path,
):
    front = FrontEnd(printer_cfg(tmp_path, MACROS))
    front.expand(PRINT)
    assert [(c.text, c.origin) for c in front.commands] == [
        ("G28", Origin(1, ("START",))),
        ("G1 X5 Y5 F6000", Origin(1, ("START", "PURGE"))),
        ("G1 X50 E10", Origin(1, ("START", "PURGE"))),
        ("G1 Z0.3", Origin(1, ("START",))),
        ("G1 X10", Origin(3)),
        ("PAUSE", Origin(4, ("PAUSE",))),
        ("G1 Z5", Origin(4, ("PAUSE",))),
        ("G2 X1 Y1 I1 J0", Origin(5)),
    ]
    [skipped] = front.skipped
    assert skipped.origin == Origin(2, ("HEAT_SOAK",))
    assert "printer['heater_bed']" in skipped.reason


def test_the_replay_plans_motion_and_reports_what_it_drops(tmp_path):
    front = FrontEnd(printer_cfg(tmp_path, MACROS))
    front.expand(PRINT)
    replay = Replay.of(front.commands)
    assert replay.lines == [
        "G1 X5 Y5 F6000",
        "G1 X50 E10",
        "G1 Z0.3",
        "G1 X10",
        "G1 Z5",
    ]
    assert set(replay.by_treatment(Treatment.HOMING)) == {"G28"}
    assert set(replay.by_treatment(Treatment.PRINT_FLOW)) == {"PAUSE"}
    assert replay.by_treatment(Treatment.STOPS_PRINT)["G2"].first == Origin(5)


class FailingBench(preflight.Bench):
    """The bench as it reports a planner panic in its checked replay."""

    def __init__(self, failure: dict):
        self.failure = failure

    def build(self):
        pass

    def check(self, arguments):
        record = preflight.FAILURE_PREFIX + json.dumps(self.failure)
        return preflight.BenchRun(1, record + "\n", "thread 'main' panicked")

    def time(self, arguments):
        raise AssertionError("a failed check is not timed")


def test_a_planner_panic_fails_with_the_print_line(
    tmp_path, capsys, monkeypatch
):
    monkeypatch.setattr(
        preflight.engine_build_id, "tree_build_id", lambda root: "tree"
    )
    gcode = tmp_path / "print.gcode"
    gcode.write_text("G90\nG1 X1 F600\nG1 X2\nSTART\nG1 X3\n")
    failure = {
        "kind": "panic",
        "message": "shaper: follower seam weld",
        "location": "rust/motion-pipeline/src/shaper.rs:853:29",
        "fed_line": 5,
        "emitted_line": 3,
        "stream_t": 1.25,
    }
    verdict = preflight.main(
        [str(gcode), "--config", str(printer_cfg(tmp_path, MACROS)), "--no-pi"],
        bench_factory=lambda: FailingBench(failure),
    )
    out = capsys.readouterr().out
    assert verdict == preflight.Verdict.FAIL
    assert "VERDICT FAIL" in out
    assert "last move fed to the planner: line 4 via START > PURGE" in out
    assert "last segment emitted: line 3, ending at stream 1.250 s" in out


def test_a_gcode_error_names_the_print_line(tmp_path):
    front = FrontEnd(printer_cfg(tmp_path, MACROS))
    front.expand("G1 X1 F600\nSTART\n")
    replay = Replay.of(front.commands)
    message = preflight.at_origin("line 3: G1 F-5: invalid speed", replay)
    assert message == "line 2 via START > PURGE: G1 F-5: invalid speed"


def refused_commands() -> set[str]:
    source = REFUSED.read_text()
    numbered = set()
    for letter, majors in re.findall(r"\(b'([GM])', ([0-9 |]+),", source):
        numbered |= {letter + major.strip() for major in majors.split("|")}
    return numbered | set(re.findall(r'"([A-Z_]+)"', source))


def mirrored_extended() -> set[str]:
    block = (
        WAYPOINTS.read_text().split("const MIRRORED_EXTENDED")[1].split("];")[0]
    )
    return set(re.findall(r'"([A-Z_]+)"', block))


def test_every_command_the_planner_refuses_has_a_treatment():
    unhandled = {
        name
        for name in refused_commands()
        if treatment(name) in (Treatment.PLANNED, Treatment.NO_MOTION)
    }
    assert unhandled == set()


def test_every_extended_command_the_planner_mirrors_is_planned():
    assert {treatment(name) for name in mirrored_extended()} == {
        Treatment.PLANNED
    }


def test_every_command_the_live_host_refuses_is_refused_by_the_planner():
    assert set(UNSUPPORTED_MOTION) <= refused_commands()
    assert {treatment(name) for name in UNSUPPORTED_MOTION} == {
        Treatment.STOPS_PRINT
    }


def test_a_curve_the_live_host_runs_is_not_reported_as_dropped():
    assert treatment("G5") == Treatment.CURVE


class CheckedBench(FailingBench):
    """The bench as it passes the planner check; timing must not run."""

    def check(self, arguments):
        return preflight.BenchRun(0, "", "")


def test_verdict_only_skips_the_exclusive_timing_pass(
    tmp_path, capsys, monkeypatch
):
    monkeypatch.setattr(
        preflight.engine_build_id, "tree_build_id", lambda root: "tree"
    )
    gcode = tmp_path / "print.gcode"
    gcode.write_text("G90\nG1 X1 F600\nG1 X2\n")
    verdict = preflight.main(
        [
            str(gcode),
            "--config",
            str(printer_cfg(tmp_path, MACROS)),
            "--verdict-only",
        ],
        bench_factory=lambda: CheckedBench({}),
    )
    assert verdict != preflight.Verdict.FAIL
    assert "VERDICT" in capsys.readouterr().out
