"""Golden parity tests for the servo capture analysis pipeline.

Freezes the scripts/servo_capture.py metrics on two real bench captures
(test/fixtures/servo_captures/) so the planned Rust port can prove output
parity and so unintended Python-side changes surface before the port lands.
See the fixtures README for provenance; regenerate after an intentional
metrics change with:

    uv run python test/test_servo_capture_goldens.py --regen
"""

import gzip
import json
import os
import sys
import tempfile

import pytest

FIXTURE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "fixtures", "servo_captures"
)
GOLDENS_PATH = os.path.join(FIXTURE_DIR, "goldens.json")

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"
    ),
)

from servo_capture import (  # noqa: E402
    compute_corexy_combine,
    compute_metrics,
    load_capture,
    motion_segments,
    moving_psd,
    top_peaks,
)

CAPTURES = [
    "cal_p880_s550_i2273_20260710_151516.scap",
    "cal_p1120_s700_i1786_20260710_151521.scap",
]
SETTLE_BAND_COUNTS = 50
TORQUE_LIMIT_PER_MILLE = 1400
COMBINE_SPEC = "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1"
COMBINE_AXIS = "X"
SERIES_STRIDE = 1000
REL_TOL = 1e-9


def _extract_capture(name):
    with gzip.open(os.path.join(FIXTURE_DIR, name + ".gz"), "rb") as f:
        raw = f.read()
    tmp = tempfile.NamedTemporaryFile(suffix=".scap", delete=False)
    tmp.write(raw)
    tmp.close()
    return tmp.name


def analyze_capture(name):
    path = _extract_capture(name)
    try:
        header, _, _ = load_capture(path)
        fs = 1e9 / header["cycle_ns"]
        per_drive = {}
        drive_datas = []
        for idx, d in enumerate(header["drives"]):
            _, data, _ = load_capture(path, d["name"])
            drive_datas.append((idx, d["name"], data))
            segs = motion_segments(data["flags"])
            freqs, psd = moving_psd(data, segs, fs)
            per_drive[d["name"]] = {
                "metrics": compute_metrics(
                    data, SETTLE_BAND_COUNTS, TORQUE_LIMIT_PER_MILLE, fs=fs
                ),
                "psd_peaks": top_peaks(freqs, psd),
            }
        combine = compute_corexy_combine(
            header, drive_datas, COMBINE_SPEC, COMBINE_AXIS
        )
        moving = combine["moving"]
        on = combine["on_ferr"][moving]
        cross = combine["cross_ferr"][moving]
        return {
            "fs": fs,
            "settle_band_counts": SETTLE_BAND_COUNTS,
            "torque_limit_per_mille": TORQUE_LIMIT_PER_MILLE,
            "drives": per_drive,
            "combine": {
                "spec": COMBINE_SPEC,
                "axis": COMBINE_AXIS,
                "on_ferr_peak_mm": float(abs(on).max()),
                "on_ferr_rms_mm": float((on**2).mean() ** 0.5),
                "cross_ferr_peak_mm": float(abs(cross).max()),
                "on_ferr_sampled_mm": [
                    float(v) for v in combine["on_ferr"][::SERIES_STRIDE]
                ],
                "cross_ferr_sampled_mm": [
                    float(v) for v in combine["cross_ferr"][::SERIES_STRIDE]
                ],
            },
        }
    finally:
        os.unlink(path)


def _assert_matches(actual, golden, where):
    assert type(actual) is type(golden) or (
        isinstance(actual, (int, float)) and isinstance(golden, (int, float))
    ), "%s: type %s vs golden %s" % (where, type(actual), type(golden))
    if isinstance(golden, dict):
        assert sorted(actual) == sorted(golden), "%s: keys differ" % (where,)
        for key in golden:
            _assert_matches(actual[key], golden[key], "%s.%s" % (where, key))
    elif isinstance(golden, list):
        assert len(actual) == len(golden), "%s: length differs" % (where,)
        for i, (a, g) in enumerate(zip(actual, golden)):
            _assert_matches(a, g, "%s[%d]" % (where, i))
    elif isinstance(golden, float):
        assert actual == pytest.approx(golden, rel=REL_TOL, abs=1e-12), (
            "%s: %r vs golden %r" % (where, actual, golden)
        )
    else:
        assert actual == golden, "%s: %r vs golden %r" % (where, actual, golden)


def _load_goldens():
    with open(GOLDENS_PATH) as f:
        return json.load(f)


@pytest.mark.parametrize("capture", CAPTURES)
def test_metrics_match_goldens(capture):
    goldens = _load_goldens()
    _assert_matches(
        json.loads(json.dumps(analyze_capture(capture))),
        goldens[capture],
        capture,
    )


def _regen():
    goldens = {name: analyze_capture(name) for name in CAPTURES}
    with open(GOLDENS_PATH, "w") as f:
        json.dump(goldens, f, indent=1, sort_keys=True)
        f.write("\n")
    print("wrote %s" % (GOLDENS_PATH,))


if __name__ == "__main__":
    if "--regen" not in sys.argv[1:]:
        raise SystemExit("usage: %s --regen" % (sys.argv[0],))
    _regen()
