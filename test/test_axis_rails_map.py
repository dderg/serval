from klippy.motion_kinematics import _LinearKinematics


def test_axis_rails_maps_lane_index_to_rail():
    kin = _LinearKinematics.__new__(_LinearKinematics)
    kin.rails = ["rail_x", "rail_y", "rail_z"]
    assert kin._axis_rails() == {0: "rail_x", 1: "rail_y", 2: "rail_z"}
