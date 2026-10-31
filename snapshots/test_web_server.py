"""Unit tests for the playground case endpoints in web/server.py.

These exercise the config-mapping payload builders directly (no socket, no
_motion_engine): parsing a case's .cfg into the playground's config shape and
extracting the raw [axis]/[post_processor] text is pure Python logic.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "web"))
import server  # noqa: E402


def test_playground_cases_lists_discovered_cases():
    cases = server.playground_cases()
    assert cases, "expected at least one snapshot case"
    for entry in cases:
        assert set(entry) == {"name", "group", "config", "gcode"}
        assert entry["name"].startswith(entry["group"] + "/")


def test_playground_case_maps_config_shape():
    name = server.playground_cases()[0]["name"]
    payload = server.playground_case(name)
    assert payload["name"] == name
    assert payload["gcode"].strip()
    config = payload["config"]
    for key in (
        "max_velocity",
        "max_accel",
        "max_jerk",
        "max_path_deviation",
        "max_accel_deviation",
        "post_processor_config",
    ):
        assert key in config
    assert ("corner_deviation" in config) != (
        "square_corner_velocity" in config
    )
    assert isinstance(config["max_velocity"], float)


def test_playground_case_extracts_axis_post_processor_text():
    names = {c["name"] for c in server.playground_cases()}
    name = next(n for n in names if n.startswith("post_processor/smooth_mzv/"))
    payload = server.playground_case(name)
    text = payload["config"]["post_processor_config"]
    assert "[axis x]" in text
    assert "[post_processor shaper]" in text
    assert "[printer]" not in text
    assert "max_velocity" not in text


def test_playground_case_unknown_name_raises():
    with pytest.raises(KeyError):
        server.playground_case("no/such/case")


def test_snapshot_comparison_requires_matching_schema_version():
    current = {"schema_version": 2, "trajectory": {}}
    assert server.snapshots_share_schema(current, current)
    assert not server.snapshots_share_schema(
        {"traj_x_pieces": []},
        current,
    )
    assert not server.snapshots_share_schema(
        {"schema_version": 1},
        current,
    )
