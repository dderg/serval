"""Unit tests for the per-SGT affine stitching in scripts/fit_pa_from_load.py.

A stitched capture re-measures each seam's boundary velocities in both
adjacent sgt blocks; these tests pin the map recovery (gain AND offset,
composed across blocks), the legacy single-sgt passthrough, and every
fail-loud rejection path.
"""

import importlib.util
import pathlib

import pytest

np = pytest.importorskip("numpy")

FITTER_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "scripts"
    / "fit_pa_from_load.py"
)
_spec = importlib.util.spec_from_file_location("fit_pa_from_load", FITTER_PATH)
fitter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fitter)

DWELL = 3.0


def build_capture(blocks, ref_levels, affine, noise=0.0, seed=0):
    """Schedule + samples from per-velocity reference levels, distorted
    per block by the true affine (r_ref = g * r_local + o)."""
    rng = np.random.default_rng(seed)
    schedule = []
    times = []
    values = []
    t = 0.0
    for sgt, vels in blocks:
        g, o = affine[sgt]
        for v in vels:
            schedule.append((t, t + DWELL, float(v), sgt))
            ts = np.arange(t + 0.1, t + DWELL, 0.05)
            r_ref = ref_levels[v] + rng.normal(0.0, noise, len(ts))
            times.extend(ts)
            values.extend((r_ref - o) / g)
            t += DWELL
        t += 1.0
    return schedule, np.asarray(times), np.asarray(values)


REF_LEVELS = {1: 460.0, 2: 400.0, 4: 320.0, 7: 250.0, 11: 170.0, 16: 90.0}
TRUE_AFFINE = {8: (1.0, 0.0), 6: (0.85, -90.0), 4: (0.7, -200.0)}
STITCH_BLOCKS = [
    (8, [1, 2, 4]),
    (6, [2, 4, 7, 11]),
    (4, [7, 11, 16]),
]


def test_three_block_affine_recovery_and_composition():
    schedule, times, values = build_capture(
        STITCH_BLOCKS, REF_LEVELS, TRUE_AFFINE, noise=1.0
    )
    stitched, ref_sgt, did = fitter.stitch_sgt(schedule, times, values)
    assert did and ref_sgt == 8
    for t0, t1, vel, _ in schedule:
        mask = (times >= t0 + 0.5 * DWELL) & (times <= t1)
        level = float(np.median(stitched[mask]))
        assert level == pytest.approx(REF_LEVELS[vel], abs=3.0)


def test_repeated_sgt_reuses_its_map():
    blocks = STITCH_BLOCKS + [(8, [7, 1, 16, 1])]
    schedule, times, values = build_capture(blocks, REF_LEVELS, TRUE_AFFINE)
    stitched, _, _ = fitter.stitch_sgt(schedule, times, values)
    tail = times >= schedule[-4][0]
    assert np.allclose(stitched[tail], values[tail])


def test_legacy_untagged_capture_passes_through():
    schedule, times, values = build_capture(
        [(8, [1, 4, 16])], REF_LEVELS, TRUE_AFFINE
    )
    legacy = [(t0, t1, v, None) for t0, t1, v, _ in schedule]
    stitched, ref_sgt, did = fitter.stitch_sgt(legacy, times, values)
    assert not did and ref_sgt is None
    assert stitched is values


def test_single_sgt_capture_is_not_stitched():
    schedule, times, values = build_capture(
        [(8, [1, 4, 16])], REF_LEVELS, TRUE_AFFINE
    )
    stitched, ref_sgt, did = fitter.stitch_sgt(schedule, times, values)
    assert not did and ref_sgt == 8


def test_mixed_tagged_and_untagged_rejected():
    schedule, times, values = build_capture(
        STITCH_BLOCKS, REF_LEVELS, TRUE_AFFINE
    )
    schedule[2] = schedule[2][:3] + (None,)
    with pytest.raises(ValueError, match="mixes sgt-tagged"):
        fitter.stitch_sgt(schedule, times, values)


def test_insufficient_overlap_rejected():
    blocks = [(8, [1, 2]), (6, [2, 7, 11])]
    schedule, times, values = build_capture(blocks, REF_LEVELS, TRUE_AFFINE)
    with pytest.raises(ValueError, match="shares 1 velocities"):
        fitter.stitch_sgt(schedule, times, values)


def test_overlap_without_level_separation_rejected():
    levels = dict(REF_LEVELS)
    levels[2] = levels[4] + 5.0
    blocks = [(8, [1, 2, 4]), (6, [2, 4, 7])]
    schedule, times, values = build_capture(blocks, levels, TRUE_AFFINE)
    with pytest.raises(ValueError, match="gain unidentifiable"):
        fitter.stitch_sgt(schedule, times, values)


def test_non_affine_seam_rejected():
    blocks = [(8, [1, 2, 4, 7]), (6, [1, 2, 4, 7, 11])]
    schedule, times, values = build_capture(blocks, REF_LEVELS, TRUE_AFFINE)
    for i, (t0, t1, vel, sgt) in enumerate(schedule):
        if sgt == 6 and vel == 2:
            mask = (times >= t0) & (times <= t1)
            values[mask] += 60.0
    with pytest.raises(ValueError, match="not affine"):
        fitter.stitch_sgt(schedule, times, values)


def test_gain_outside_family_rejected():
    affine = dict(TRUE_AFFINE)
    affine[6] = (0.2, 30.0)
    schedule, times, values = build_capture(
        [(8, [1, 2, 4]), (6, [2, 4, 7])], REF_LEVELS, affine
    )
    with pytest.raises(ValueError, match="outside plausible"):
        fitter.stitch_sgt(schedule, times, values)


def test_parse_capture_reads_legacy_and_tagged_rows(tmp_path):
    legacy = tmp_path / "v1.csv"
    legacy.write_text(
        "# pa_ident v1\n# smooth_time=0.030000\nS,1.0,4.0,2.0\nD,2.5,400\n"
    )
    schedule, times, values, smooth = fitter.parse_capture(legacy)
    assert schedule == [(1.0, 4.0, 2.0, None)] and smooth == 0.03

    tagged = tmp_path / "v2.csv"
    tagged.write_text(
        "# pa_ident v2\n# smooth_time=0.030000\n"
        "S,1.0,4.0,2.0,8\nS,5.0,8.0,4.0,6\nD,2.5,400\n"
    )
    schedule, _, _, _ = fitter.parse_capture(tagged)
    assert schedule == [(1.0, 4.0, 2.0, 8), (5.0, 8.0, 4.0, 6)]

    header_sgt = tmp_path / "v1s.csv"
    header_sgt.write_text(
        "# pa_ident v2\n# smooth_time=0.030000\n# sgt=8\n"
        "S,1.0,4.0,2.0\nD,2.5,400\n"
    )
    schedule, _, _, _ = fitter.parse_capture(header_sgt)
    assert schedule == [(1.0, 4.0, 2.0, 8)]
