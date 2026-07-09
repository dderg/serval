import textwrap

import pytest
from viz_pipeline import read_printer_config


def _write_cfg(tmp_path, text):
    path = tmp_path / "printer.cfg"
    path.write_text(textwrap.dedent(text))
    return path


def test_no_axis_sections_yields_empty_lists(tmp_path):
    cfg = _write_cfg(
        tmp_path,
        """
        [printer]
        max_velocity: 300
        max_accel: 1000
        square_corner_velocity: 0
        max_jerk: 100000
        """,
    )
    data = read_printer_config(cfg)
    assert data.axis_sections == []
    assert data.post_processor_sections == []
    assert data.max_velocity == 300.0


def test_axis_and_post_processor_sections_are_parsed(tmp_path):
    cfg = _write_cfg(
        tmp_path,
        """
        [printer]
        max_velocity: 300
        max_accel: 1000
        square_corner_velocity: 0
        max_jerk: 100000

        [post_processor is_xy]
        type: smooth_bell
        smooth_time: 0.0243

        [axis x]
        post_processors: is_xy

        [axis y]
        post_processors: is_xy
        """,
    )
    data = read_printer_config(cfg)
    names = {a.name for a in data.axis_sections}
    assert names == {"x", "y"}
    assert len(data.post_processor_sections) == 1
    pp = data.post_processor_sections[0]
    assert pp.name == "is_xy"
    assert pp.type == "smooth_bell"
    assert pp.params == [("smooth_time", 0.0243)]


def test_undeclared_post_processor_reference_fails_loudly(tmp_path):
    cfg = _write_cfg(
        tmp_path,
        """
        [printer]
        max_velocity: 300
        max_accel: 1000
        square_corner_velocity: 0
        max_jerk: 100000

        [axis x]
        post_processors: nope
        """,
    )
    with pytest.raises(Exception):
        read_printer_config(cfg)
