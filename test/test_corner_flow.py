import json
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import corner_flow as cf  # noqa: E402

C45_AT_50 = ("tau", "", 0, 50.0, 0.02, (0.4, 0.2, 1.75))


def test_a_case_whose_engine_aborts_fails_alone_with_the_panic_message():
    abort = (
        "import os, sys; "
        "sys.stderr.write(\"thread 'kalico-shaper' panicked at a.rs:1:1:\\n"
        'shaper: boom\\nnote: run with RUST_BACKTRACE=1\\n"); '
        "sys.stderr.flush(); os.abort()"
    )
    row = cf.run_isolated(C45_AT_50, [sys.executable, "-c", abort])
    assert (row.case, row.speed, row.sweep) == ("c45", 50.0, "tau")
    assert row.error == "engine panic: shaper: boom"


def test_a_case_whose_process_dies_without_a_panic_names_its_exit():
    row = cf.run_isolated(
        C45_AT_50, [sys.executable, "-c", "import sys; sys.exit(3)"]
    )
    assert row.error == "engine exited 3"


def test_a_completed_case_returns_the_row_its_process_measured():
    measured = cf.Row(
        "c45", 50.0, sweep="tau", peak=1.5, corners=[{"peak": 0.015}]
    )
    report = [
        sys.executable,
        "-c",
        "import sys; print('engine chatter'); print(sys.argv[1])",
        json.dumps(asdict(measured)),
    ]
    returned = cf.run_isolated(C45_AT_50, report)
    assert json.dumps(asdict(returned)) == json.dumps(asdict(measured))
