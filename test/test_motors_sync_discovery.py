from klippy.extras.motors_sync import rail_center


def test_rail_center_is_midpoint():
    assert rail_center(0.0, 350.0) == 175.0
    assert rail_center(-10.0, 10.0) == 0.0
