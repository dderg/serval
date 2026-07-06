import pytest

from klippy.extras.resonance_buzz import buzz_axis_to_motor_mask


def test_corexy_x_drives_both_motors_in_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("x", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b000


def test_corexy_y_drives_both_motors_anti_phase():
    axis_mask, sign_mask = buzz_axis_to_motor_mask("y", coupled=True)
    assert axis_mask == 0b011
    assert sign_mask == 0b010


def test_corexy_z_is_single_slot():
    assert buzz_axis_to_motor_mask("z", coupled=True) == (0b100, 0b000)


def test_cartesian_axes_map_one_to_one():
    assert buzz_axis_to_motor_mask("x", coupled=False) == (0b001, 0b000)
    assert buzz_axis_to_motor_mask("y", coupled=False) == (0b010, 0b000)
    assert buzz_axis_to_motor_mask("z", coupled=False) == (0b100, 0b000)


def test_case_insensitive():
    assert buzz_axis_to_motor_mask(
        "X", coupled=True
    ) == buzz_axis_to_motor_mask("x", coupled=True)


def test_unsupported_axis_raises():
    with pytest.raises(ValueError, match="unsupported buzz axis"):
        buzz_axis_to_motor_mask("e", coupled=False)
