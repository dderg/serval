"""SET_INPUT_SHAPER right after motion, to a shaper reaching further back
in time than the one it replaces (fuzz seed 1044, minimized; no
corner_deviation): stepcompress rejected the steps ("Invalid sequence")
and the printer shut down."""

import pathlib

from tools.motion_eval import evaluate, run, sweep

GCODE = (
    pathlib.Path(__file__).parent / "data" / "shaper_switch_after_motion.gcode"
)
START = (108.49584534769207, 223.41326211837966, 20.621123185990953)


def test_switching_to_a_longer_shaper_after_motion_keeps_printing():
    raw = run.run_klippy(
        sweep.CONFIGS["bench"]["cfg"], GCODE.read_text(), START
    )
    assert (
        len(raw.capture["shapers"])
        == GCODE.read_text().count("SET_INPUT_SHAPER") + 1
    )
    evaluate.executed_motion(raw.capture, evaluate.DEFAULT_DT)
