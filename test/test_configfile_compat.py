from klippy.configfile import PrinterConfig


def test_kinematics_type_is_mirrored_to_legacy_printer_config():
    config = object.__new__(PrinterConfig)
    config.status_settings = {
        "kinematics": {"type": "corexy"},
        "printer": {"max_velocity": 300.0},
    }
    config.status_raw_config = {
        "kinematics": {"type": "corexy", "axis_x": "x"},
        "printer": {"max_velocity": "300"},
    }

    config._mirror_kinematics_to_legacy_printer()

    assert config.status_settings["printer"]["kinematics"] == "corexy"
    assert config.status_raw_config["printer"]["kinematics"] == "corexy"
