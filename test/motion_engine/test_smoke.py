import pytest

motion_engine = pytest.importorskip("klippy._motion_engine")


def test_module_exports_engine_class():
    assert hasattr(motion_engine, "MotionEngine")


def test_engine_instantiates():
    engine = motion_engine.MotionEngine()
    assert engine.version() != ""


def test_claim_mcu_returns_int():
    engine = motion_engine.MotionEngine()
    handle = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    assert isinstance(handle, int)


def test_claim_two_mcus_returns_distinct_handles():
    engine = motion_engine.MotionEngine()
    h1 = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    h2 = engine.claim_mcu("mcu2", "/dev/ttyACM1", 250000)
    assert h1 != h2


def test_release_mcu_invalidates_handle():
    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.release_mcu(h)
    with pytest.raises(RuntimeError, match="unknown mcu_handle"):
        engine.get_identify_data(h)


def test_unknown_mcu_handle_raises_runtime_error():
    engine = motion_engine.MotionEngine()
    with pytest.raises(RuntimeError, match="unknown mcu_handle"):
        engine.get_identify_data(999)
