import importlib.util
import json
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_diff_report.py",
)
_spec = importlib.util.spec_from_file_location(
    "servo_diff_report_script", _SCRIPT
)
sdr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sdr)

FS = 4000.0
MODE_HZ = 92.0
MODE_ZETA = 0.05


def chirp(f0, f1, seconds, fs):
    t = np.arange(int(seconds * fs)) / fs
    return np.sin(2.0 * np.pi * (f0 * t + (f1 - f0) / (2.0 * seconds) * t**2))


def resonant_response(x, fs, f0, zeta):
    spectrum = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    h = f0**2 / (f0**2 - f**2 + 2j * zeta * f0 * f)
    return np.fft.irfft(spectrum * h, len(x))


def test_welch_frf_recovers_resonance_frequency_and_damping():
    x = chirp(30.0, 150.0, 24.0, FS)
    y = resonant_response(x, FS, MODE_HZ, MODE_ZETA)
    freqs, frf, coherence, segments = sdr.welch_frf(x, y, FS, 4096)
    assert segments >= sdr.MIN_SEGMENTS
    modes = sdr.find_modes(freqs, frf, coherence, 30.0, 150.0)
    assert len(modes) == 1
    mode = modes[0]
    assert mode["freq_hz"] == pytest.approx(MODE_HZ, abs=2.0)
    assert mode["gain"] == pytest.approx(1.0 / (2.0 * MODE_ZETA), rel=0.25)
    assert mode["damping"] == pytest.approx(MODE_ZETA, rel=0.5)


def test_find_modes_fails_loudly_without_coherent_response():
    rng = np.random.default_rng(7)
    x = chirp(30.0, 150.0, 12.0, FS)
    y = rng.normal(size=len(x))
    freqs, frf, coherence, _ = sdr.welch_frf(x, y, FS, 4096)
    with pytest.raises(SystemExit, match="no coherent"):
        sdr.find_modes(freqs, frf, coherence, 30.0, 150.0)


def test_active_slice_trims_quiet_head_and_tail():
    cmd = np.zeros(10000)
    cmd[4000:6000] = np.sin(np.linspace(0.0, 60.0, 2000))
    span = sdr.active_slice(cmd)
    assert 3900 <= span.start <= 4100
    assert 5900 <= span.stop <= 6100


def test_active_slice_fails_loudly_on_flat_command():
    with pytest.raises(SystemExit, match="no differential excitation"):
        sdr.active_slice(np.zeros(1000))


def test_welch_rejects_captures_too_short_for_segments():
    with pytest.raises(SystemExit, match="too short"):
        sdr.welch_frf(np.ones(300), np.ones(300), FS, 4096)


def test_parse_pair_requires_exactly_two_motors():
    assert sdr.parse_pair("motor_a:1+motor_a1:1") == [
        ("motor_a", 1),
        ("motor_a1", 1),
    ]
    with pytest.raises(SystemExit, match="one belt of two motors"):
        sdr.parse_pair("motor_a:1")
    with pytest.raises(SystemExit, match="one belt of two motors"):
        sdr.parse_pair("motor_a:1,motor_a1:1")


COUNTS_PER_MM = 100000.0


def synth_pair_capture(tmp_path, invert_second=False):
    cmd_mm = 0.05 * chirp(30.0, 150.0, 24.0, FS)
    diff_act_mm = resonant_response(2.0 * cmd_mm, FS, MODE_HZ, MODE_ZETA)
    n = len(cmd_mm)
    drive_sign = -1.0 if invert_second else 1.0
    channels = [
        {"name": "cycle_index", "dtype": "u64", "offset": 0},
        {"name": "flags", "dtype": "u8", "offset": 8},
        {"name": "target_counts", "dtype": "i32", "offset": 9},
        {"name": "position_actual", "dtype": "i32", "offset": 13},
        {"name": "torque_actual", "dtype": "i16", "offset": 17},
    ]
    record_size = 9 + 2 * 10
    header = {
        "version": 2,
        "cycle_ns": int(1e9 / FS),
        "record_size": record_size,
        "started_utc": "2026-07-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [
            {
                "name": "motor_a",
                "counts_per_mm": COUNTS_PER_MM,
                "rotation_distance": 40.0,
                "invert": False,
            },
            {
                "name": "motor_a1",
                "counts_per_mm": COUNTS_PER_MM,
                "rotation_distance": 40.0,
                "invert": invert_second,
            },
        ],
        "channels": channels,
    }
    dtype = np.dtype(
        {
            "names": [
                "cycle_index",
                "flags",
                "t0",
                "p0",
                "q0",
                "t1",
                "p1",
                "q1",
            ],
            "formats": ["<u8", "u1", "<i4", "<i4", "<i2", "<i4", "<i4", "<i2"],
            "offsets": [0, 8, 9, 13, 17, 19, 23, 27],
            "itemsize": record_size,
        }
    )
    records = np.zeros(n, dtype=dtype)
    records["cycle_index"] = np.arange(n)
    records["t0"] = np.round(cmd_mm * COUNTS_PER_MM)
    records["p0"] = np.round(0.5 * diff_act_mm * COUNTS_PER_MM)
    records["t1"] = np.round(drive_sign * -cmd_mm * COUNTS_PER_MM)
    records["p1"] = np.round(drive_sign * -0.5 * diff_act_mm * COUNTS_PER_MM)
    path = os.path.join(str(tmp_path), "diff_20260710_120000.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        f.write(records.tobytes())
    return path


def test_analyze_finds_the_mode_in_a_synthetic_capture(tmp_path):
    path = synth_pair_capture(tmp_path)
    result = sdr.analyze(
        path,
        [("motor_a", 1), ("motor_a1", 1)],
        30.0,
        150.0,
        4096,
    )
    assert len(result["modes"]) == 1
    assert result["modes"][0]["freq_hz"] == pytest.approx(MODE_HZ, abs=2.0)


def test_analyze_applies_pair_signs_for_inverted_motors(tmp_path):
    path = synth_pair_capture(tmp_path, invert_second=True)
    result = sdr.analyze(
        path,
        [("motor_a", 1), ("motor_a1", -1)],
        30.0,
        150.0,
        4096,
    )
    assert result["modes"][0]["freq_hz"] == pytest.approx(MODE_HZ, abs=2.0)
