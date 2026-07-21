import pytest

from klippy.extras import servo_calibration

try:
    import tomllib
except ImportError:
    tomllib = None

pytestmark = pytest.mark.skipif(
    tomllib is None, reason="parse_dynamics_profile requires tomllib (3.11+)"
)

BASELINE_TOML = """\
version = 6
axes = ["motor_a", "motor_b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]
fit_rms_residual = [0.5, 0.5]
"""

AWD_TOML = """\
version = 6
axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]
modes = ["x", "y"]
frame = [[0.25, 0.25, -0.25, 0.25], [0.25, 0.25, 0.25, -0.25]]
mass = [0.020, 0.030]
viscous = [0.004, 0.005]
coulomb = [1.0, 1.5]

[[pair]]
slots = ["motor_a", "motor_a1"]
direction_split = 0.05

[[pair]]
slots = ["motor_b", "motor_b1"]
direction_split = -0.1
"""

OLD_AWD_TOML = AWD_TOML.split("\n[[pair]]", 1)[0] + "\n"

AWD_PAIRS = [
    {"slots": ["motor_a", "motor_a1"], "direction_split": 0.05},
    {"slots": ["motor_b", "motor_b1"], "direction_split": -0.1},
]

BASELINE_MASS = [0.020, 0.030]
BASELINE_VISCOUS = [0.004, 0.005]
BASELINE_COULOMB = [1.0, 1.5]


def test_parse_old_v6_dynamics_profile_without_pairs():
    p = servo_calibration.parse_dynamics_profile(BASELINE_TOML)
    assert p["axes"] == ["motor_a", "motor_b"]
    assert p["modes"] == ["x", "y"]
    assert p["frame"] == [[0.5, 0.5], [0.5, -0.5]]
    assert p["mass"] == BASELINE_MASS
    assert p["viscous"] == BASELINE_VISCOUS
    assert p["coulomb"] == BASELINE_COULOMB
    assert p["pairs"] == []


@pytest.mark.parametrize(
    "axes, message",
    [
        ('["motor_a", ""]', "non-empty strings"),
        ('["motor_a", "   "]', "non-empty strings"),
        ('["motor_a", 7]', "non-empty strings"),
        ('["motor_a", "motor_a"]', "unique"),
    ],
)
def test_parse_dynamics_profile_requires_unique_nonempty_axis_names(
    axes, message
):
    text = BASELINE_TOML.replace(
        'axes = ["motor_a", "motor_b"]', "axes = " + axes
    )
    with pytest.raises(ValueError, match=message):
        servo_calibration.parse_dynamics_profile(text)


def test_axis_uniqueness_is_checked_before_pair_mapping():
    text = AWD_TOML.replace(
        'axes = ["motor_a", "motor_a1", "motor_b", "motor_b1"]',
        'axes = ["motor_a", "motor_a", "motor_b", "motor_b1"]',
    )
    with pytest.raises(ValueError, match="axes must be unique"):
        servo_calibration.parse_dynamics_profile(text)


def test_parse_dynamics_profile_parses_signed_pairs():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    assert p["axes"] == ["motor_a", "motor_a1", "motor_b", "motor_b1"]
    assert p["pairs"] == AWD_PAIRS


def test_pair_slot_order_transforms_coefficient_by_frame_lambda():
    swapped = AWD_TOML.replace(
        'slots = ["motor_a", "motor_a1"]\ndirection_split = 0.05',
        'slots = ["motor_a1", "motor_a"]\ndirection_split = -0.05',
    ).replace(
        'slots = ["motor_b", "motor_b1"]\ndirection_split = -0.1',
        'slots = ["motor_b1", "motor_b"]\ndirection_split = -0.1',
    )
    pairs = servo_calibration.parse_dynamics_profile(swapped)["pairs"]
    assert pairs == [
        {"slots": ["motor_a1", "motor_a"], "direction_split": -0.05},
        {"slots": ["motor_b1", "motor_b"], "direction_split": -0.1},
    ]


def test_parse_dynamics_profile_rejects_violations():
    with pytest.raises(ValueError, match="refit with SERVO_FIT_DYNAMICS"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 6", "version = 1")
        )
    with pytest.raises(ValueError, match="refit with SERVO_FIT_DYNAMICS"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("version = 6", "version = 5")
        )
    with pytest.raises(ValueError, match="direction_split"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML
            + '\n[[pair]]\nslots = ["motor_a", "motor_b"]\n'
            + "belt_position_split = [0.02, -0.0003]\n"
        )
    with pytest.raises(ValueError, match="frame"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "frame = [[0.5, 0.5], [0.5, -0.5]]", "frame = [[0.5, 0.5]]"
            )
        )
    with pytest.raises(ValueError, match="mass"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("mass = [0.020, 0.030]", "mass = [0.020]")
        )
    with pytest.raises(ValueError, match="viscous"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.005]", "viscous = [0.004]"
            )
        )
    with pytest.raises(ValueError, match="non-finite"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace(
                "viscous = [0.004, 0.005]", "viscous = [0.004, nan]"
            )
        )


@pytest.mark.parametrize("value", ["nan", "0.5", "-0.5", "true"])
def test_parse_dynamics_profile_rejects_bad_direction_split(value):
    with pytest.raises(ValueError, match="direction_split"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "direction_split = 0.05", "direction_split = " + value
            )
        )


def test_parse_dynamics_profile_rejects_pair_slot_violations():
    with pytest.raises(ValueError, match="not among profile axes"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace('motor_a1"]', 'motor_z"]', 1)
        )
    with pytest.raises(ValueError, match="more than one pair"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                'slots = ["motor_b", "motor_b1"]',
                'slots = ["motor_a", "motor_b1"]',
            )
        )
    with pytest.raises(ValueError, match="exact equal or opposite"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "frame = [[0.25, 0.25, -0.25, 0.25]",
                "frame = [[0.25, 0.2, -0.25, 0.25]",
            )
        )


def test_parse_dynamics_profile_rejects_global_split_and_orientation():
    with pytest.raises(ValueError, match="not a global field"):
        servo_calibration.parse_dynamics_profile(
            BASELINE_TOML.replace("mass =", "direction_split = 0.1\nmass =")
        )
    with pytest.raises(ValueError, match="orientation is not supported"):
        servo_calibration.parse_dynamics_profile(
            AWD_TOML.replace(
                "direction_split = 0.05",
                "direction_split = 0.05\norientation = -1",
            )
        )


def test_equal_or_opposite_columns():
    eq = servo_calibration._equal_or_opposite_columns
    assert eq([0.5, 0.5], [0.5, 0.5]) is True
    assert eq([0.5, -0.5], [-0.5, 0.5]) is True
    assert eq([0.5, 0.5], [0.5, -0.5]) is False
    assert eq([0.5, 0.5], [0.25, 0.25]) is False
    # an all-zero column is never a pair partner
    assert eq([0.0, 0.0], [0.0, 0.0]) is False


def test_discover_dynamics_pairs_uses_equal_or_opposite_columns():
    p = servo_calibration.parse_dynamics_profile(OLD_AWD_TOML)
    assert servo_calibration.discover_dynamics_pairs(p) == [
        {"slots": ["motor_a", "motor_a1"], "direction_split": 0.0},
        {"slots": ["motor_b", "motor_b1"], "direction_split": 0.0},
    ]
    # the zero column has no partner and axes are each claimed at most once
    mixed = dict(p)
    mixed["axes"] = ["a", "a1", "zero", "u", "u1"]
    mixed["frame"] = [[1.0, 1.0, 0.0, 2.0, 4.0], [0.0, 0.0, 0.0, 1.0, 2.0]]
    assert servo_calibration.discover_dynamics_pairs(mixed) == [
        {"slots": ["a", "a1"], "direction_split": 0.0}
    ]
    ambiguous = dict(p)
    ambiguous["axes"] = ["a", "b", "c"]
    ambiguous["frame"] = [[0.5, 0.5, -0.5]]
    with pytest.raises(ValueError, match="ambiguous equal/opposite"):
        servo_calibration.discover_dynamics_pairs(ambiguous)


def test_add_dynamics_direction_split_applies_delta_and_guards():
    p = servo_calibration.parse_dynamics_profile(AWD_TOML)
    added = servo_calibration.add_dynamics_direction_split(p, 0, -0.2)
    assert added["pairs"][0]["direction_split"] == pytest.approx(-0.15)
    assert added["pairs"][1] == AWD_PAIRS[1]
    assert added["mass"] == p["mass"]
    assert added["pairs"] is not p["pairs"]
    with pytest.raises(ValueError, match=r"abs\(value\) < 0.5"):
        servo_calibration.add_dynamics_direction_split(p, 0, 0.45)
    with pytest.raises(ValueError, match=r"abs\(value\) < 0.5"):
        servo_calibration.add_dynamics_direction_split(p, 1, -0.45)
