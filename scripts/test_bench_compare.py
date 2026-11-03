import bench_compare as bc

OUTPUT = """\
moves=100 segments=40 stream_secs=60.0 wall_secs=12.5 overall_x=4.80 deepest_held_moves=3
stage breakdown (wall share, calls, per move):
  fit                            8.000s   64.0%  calls=100      80.0 us/move
  plan                           2.500s   20.0%  calls=100      25.0 us/move
worst stream windows (t_lo..t_hi realtime_x):
"""


def test_a_replay_report_is_parsed_into_wall_realtime_factor_and_stages():
    run = bc.parse_run(OUTPUT)
    assert run == {
        "wall_s": 12.5,
        "x": 4.8,
        "stages": {"fit": 8.0, "plan": 2.5},
    }


def test_a_report_without_a_summary_fails_loudly():
    try:
        bc.parse_run("garbage")
    except bc.BenchCompareError as exc:
        assert "no summary line" in str(exc)
    else:
        raise AssertionError("expected BenchCompareError")


def test_medians_are_taken_per_field_and_stage_and_a_missing_stage_counts_as_zero():
    runs = [
        {"wall_s": 10.0, "cpu_s": 30.0, "x": 5.0, "stages": {"fit": 6.0}},
        {"wall_s": 12.0, "cpu_s": 36.0, "x": 4.0, "stages": {"fit": 8.0}},
        {
            "wall_s": 11.0,
            "cpu_s": 33.0,
            "x": 4.5,
            "stages": {"fit": 7.0, "plan": 1.0},
        },
    ]
    assert bc.median_run(runs) == {
        "wall_s": 11.0,
        "cpu_s": 33.0,
        "x": 4.5,
        "stages": {"fit": 7.0, "plan": 0.0},
    }


def test_the_table_reports_the_change_of_b_against_a():
    a = {"wall_s": 10.0, "cpu_s": 30.0, "x": 5.0, "stages": {"fit": 6.0}}
    b = {"wall_s": 11.0, "cpu_s": 33.0, "x": 4.5, "stages": {"fit": 7.2}}
    table = bc.comparison_table("fast", a, b, (1.0, 2.0))
    assert "+10.0%" in table and "-10.0%" in table and "+20.0%" in table
