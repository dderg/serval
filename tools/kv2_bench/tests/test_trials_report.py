import json

from tools.kv2_bench import trials_report


def write_run(path, start, shutdown=None):
    path.mkdir()
    (path / "run.json").write_text(
        json.dumps(
            {
                "print_start_pi_monotonic": start,
                "print_end_pi_monotonic": None if shutdown else start + 10,
            }
        )
    )
    events = {"klippy:shutdown": shutdown} if shutdown else {}
    stall = {
        "at": start + 4.9, "max_overdue_s": 0.3, "overdue_timer": "flush",
        "toolhead": {"queuing_state": "main"}, "main_stack": ["  File x, in junction_v2\n"],
        "pyspy": "Thread 1\n  junction_v2",
    }  # fmt: skip
    (path / "probe.json").write_text(
        json.dumps(
            {
                "events": events,
                "late_samples": [
                    [start - 1, 0.5],
                    [start + 1, 0.004],
                    [start + 4.95, 0.3],
                ],
                "late_events": [
                    {"at": start + 4.95, "toolhead": {"queuing_state": "main"}}
                ],
                "stalls": [stall] if shutdown else [],
                "pyspy_native": True,
            }
        )
    )


def test_failure_rate_lateness_and_stall_stack(tmp_path):
    write_run(tmp_path / "ok", 100.0)
    write_run(tmp_path / "bad", 200.0, shutdown=205.0)
    runs = [
        {"gate": "trials", "config": "cd", "run_dir": str(tmp_path / "ok")},
        {
            "gate": "trials",
            "config": "cd",
            "run_dir": str(tmp_path / "bad"),
            "error": "shutdown",
        },
    ]
    (tmp_path / "session.json").write_text(json.dumps({"runs": runs}))
    cd = trials_report.summarize(tmp_path)["cd"]
    assert (cd["runs"], cd["failures"], cd["failure_rate"]) == (2, 1, 0.5)
    assert cd["per_run_max_late_ms"]["max"] == 300.0
    assert cd["late_ge_20ms_by_toolhead_state"] == {"main": 2}
    failed = cd["failed_runs"][0]
    assert failed["shutdown_since_print_start_s"] == 5.0
    stall = failed["stalls_before_shutdown"][0]
    assert stall["main_stack"] == ["File x, in junction_v2"]
    assert stall["pyspy"] == ["Thread 1", "  junction_v2"]
