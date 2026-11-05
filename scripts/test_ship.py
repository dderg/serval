from pathlib import Path

import pytest
import ship


def test_python_changes_trigger_the_py_gate():
    assert ship.needs_py_gate(["klippy/mcu.py"])
    assert ship.needs_py_gate(["test/test_x.py"])
    assert ship.needs_py_gate(["tools/sim/world.py"])
    assert ship.needs_py_gate(["scripts/ci.sh", "scripts/land_queue.py"])


def test_rust_docs_and_shell_only_changes_skip_the_py_gate():
    assert not ship.needs_py_gate(["rust/geometry/src/lib.rs", "docs/x.md"])
    assert not ship.needs_py_gate(["scripts/ci.sh", "tools/sim/run.sh"])


def test_quick_always_runs_and_py_only_when_needed():
    assert ship.gate_commands(["rust/a.rs"], False) == [
        ["./scripts/ci.sh", "quick"]
    ]
    assert ship.gate_commands(["klippy/a.py"], False) == [
        ["./scripts/ci.sh", "quick"],
        ["./scripts/ci.sh", "py"],
    ]


def test_no_gates_skips_everything():
    assert ship.gate_commands(["klippy/a.py"], True) == []


def test_exactly_one_body_source_is_required(tmp_path: Path):
    with pytest.raises(ship.ShipError):
        ship.body_text(None, None)
    with pytest.raises(ship.ShipError):
        ship.body_text("a", tmp_path / "b.md")
    assert ship.body_text("text", None) == "text"
    (tmp_path / "b.md").write_text("from file")
    assert ship.body_text(None, tmp_path / "b.md") == "from file"


def test_a_moved_throughput_baseline_needs_an_accept_block():
    changed = ["rust/a.rs", ship.THROUGHPUT_BASELINE]
    assert ship.throughput_accept_problem(changed, "no block") is not None
    body = "Faster.\n\n```throughput-accept\nfewer fits per window\n```\n"
    assert ship.throughput_accept_problem(changed, body) is None
    assert ship.throughput_accept_problem(["rust/a.rs"], "no block") is None
