from __future__ import annotations

import sim_config
import sim_probe

STEP_MM = {"a": 0.0125, "b": 0.0125, "z": 0.0025, "e": 0.0025}
DIR_SIGN = {"a": 1, "b": 1, "z": 1, "e": 1}


def _built(pin):
    return sim_config.build(
        sim_probe.line_spec(pin), 1, STEP_MM, DIR_SIGN, (50.0, 50.0, 10.0)
    )


def test_a_driven_line_is_triggered_at_the_start_and_alone():
    built = _built("y_endstop")
    (wall,) = built.walls
    assert wall.pin == sim_config.PINS["y_endstop"]
    assert wall.active_high and wall.at_or_below
    z_steps = built.origin[sim_config.CHANNELS.index("z")]
    assert z_steps <= wall.threshold
    assert _built(None).walls == ()


def test_each_endstop_is_read_against_its_own_line():
    states = {"x": "open", "y": "TRIGGERED", "z": "open"}
    assert sim_probe.line_reading(states, None) == {
        "x_endstop": False,
        "y_endstop": True,
        "z_endstop": False,
    }
    assert sim_probe.line_reading(states, "open")["probe"] is False
    assert sim_probe.line_reading({}, "TRIGGERED") == {"probe": True}
