import math

import pytest
import review_baselines as rb


def test_an_expectation_reads_every_key():
    expectation = rb.Expectation.parse(
        "cases=post_processor/*;axes=e,x;dp=1e-3;dv=0.5;da=40;duration=2e-3;"
        "material=1e-6;pieces=any"
    )
    assert expectation.cases == "post_processor/*"
    assert expectation.axes == ("e", "x")
    assert expectation.bounds == {"p": 1e-3, "v": 0.5, "a": 40.0}
    assert expectation.duration == 2e-3
    assert expectation.material == 1e-6
    assert expectation.pieces == math.inf


def test_an_empty_expectation_allows_motion_but_no_timing_material_or_piece_blowup():
    expectation = rb.Expectation.parse("")
    assert expectation.axes == ("x", "y", "z", "e")
    assert expectation.bounds == {"p": math.inf, "v": math.inf, "a": math.inf}
    assert (expectation.duration, expectation.material, expectation.pieces) == (
        0.0,
        0.0,
        2.0,
    )


@pytest.mark.parametrize("spec", ["axes=q", "speed=3", "axes"])
def test_a_malformed_expectation_fails_loudly(spec):
    with pytest.raises(rb.ReviewError):
        rb.Expectation.parse(spec)


def test_the_first_matching_expectation_applies():
    narrow = rb.Expectation.parse("cases=post_processor/*;axes=e")
    broad = rb.Expectation.parse("axes=x,y")
    assert (
        rb.expectation_for("post_processor/zv/corner", [narrow, broad])
        is narrow
    )
    assert (
        rb.expectation_for("arc_fit/printer/circle", [narrow, broad]) is broad
    )
    assert rb.expectation_for("arc_fit/printer/circle", [narrow]) is None


def test_expectations_come_from_the_fenced_block_in_a_pr_body():
    body = (
        "Fixes the seam.\n\n```snapshot-expect\ncases=post_processor/*;axes=e\n\n"
        "axes=x,y;dp=1e-4\n```\nmore text"
    )
    assert rb.expectations_from_body(body) == [
        "cases=post_processor/*;axes=e",
        "axes=x,y;dp=1e-4",
    ]
    assert rb.expectations_from_body("no block") == []


def test_gcode_extrusion_follows_absolute_relative_and_resets():
    gcode = "\n".join(
        [
            "G90",
            "G1 X1 E1.0",
            "G1 X2 E1.5 ; comment E9",
            "G92 E0",
            "G1 X3 E0.25",
            "M83",
            "G1 X4 E0.5",
            "G1 E-0.2",
        ]
    )
    assert rb.gcode_extrusion(gcode) == pytest.approx(1.5 + 0.25 + 0.5 - 0.2)


def test_planned_limit_excess_is_the_worst_ratio_over_the_span_caps():
    snapshot = {
        "trajectory": {
            "spans": [
                {
                    "limits": {
                        "max_velocity_mm_s": 100.0,
                        "accel_mm_s2": 1000.0,
                    },
                    "phases": [
                        {
                            "v0": 0.0,
                            "v1": 100.0,
                            "law": "const_accel",
                            "a0": 1000.0,
                        },
                        {
                            "v0": 100.0,
                            "v1": 50.0,
                            "law": "disk_rail",
                            "accel": 1100.0,
                        },
                    ],
                }
            ]
        }
    }
    assert rb.planned_limit_excess(snapshot) == pytest.approx(0.1)


def _snapshot(axes, spans):
    return {"trajectory": {"axes": axes, "spans": spans}}


def test_the_start_shift_reads_every_axis_not_only_the_analytic_spans():
    retract = {
        "carrier": {"kind": "analytic", "span": 0},
        "t0": 0.0,
        "t1": 0.057,
    }
    fitted = {
        "carrier": {"kind": "piecewise_relative_spline"},
        "t0": 0.0,
        "t1": 0.057,
    }
    hold = {
        "carrier": {"kind": "hold", "position": 1.0},
        "t0": 0.0,
        "t1": 0.057,
    }
    base = _snapshot(
        [[hold], [retract]], [{"t_start": 0.0}, {"t_start": 0.057}]
    )
    head = _snapshot([[hold], [fitted]], [{"t_start": 0.057}])
    assert rb.start_shift(base, head) == 0.0
