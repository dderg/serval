from klippy.extras import homing as homing_mod


def test_trigger_too_early_short_with_margin():
    assert homing_mod._trigger_too_early(2.0, 15.0, 0.5) is True


def test_trigger_too_early_at_tolerance_edge_is_early():
    # 15 - 14.5 = 0.5 >= 0.5 -> early
    assert homing_mod._trigger_too_early(14.5, 15.0, 0.5) is True


def test_trigger_too_early_within_tolerance_band_not_early():
    # 15 - 14.6 = 0.4 < 0.5 -> not early
    assert homing_mod._trigger_too_early(14.6, 15.0, 0.5) is False


def test_trigger_too_early_beyond_min_not_early():
    assert homing_mod._trigger_too_early(100.0, 15.0, 0.5) is False


def test_trigger_too_early_disabled_when_min_zero():
    assert homing_mod._trigger_too_early(0.0, 0.0, 0.5) is False
