import pathlib
import re

from klippy import motion_engine

KLIPPY_DIR = pathlib.Path(motion_engine.__file__).resolve().parent


def _real_engine_surface():
    native = set(dir(motion_engine._native.MotionEngine))
    wrapper = set(dir(motion_engine.MotionEngineWrapper))
    return native | wrapper


def test_stub_noop_methods_exist_on_real_engine():
    missing = motion_engine._STUB_NOOP_METHODS - _real_engine_surface()
    assert not missing, (
        "_STUB_NOOP_METHODS lists methods absent from the real engine "
        "surface (stale entries?): %s" % sorted(missing)
    )


def test_stub_concrete_methods_exist_on_real_engine():
    concrete = {
        name
        for name in vars(motion_engine._StubEngine)
        if not name.startswith("__")
    }
    missing = concrete - _real_engine_surface()
    assert not missing, (
        "_StubEngine defines methods absent from the real engine "
        "surface (stale entries?): %s" % sorted(missing)
    )


ENGINE_CALL_RE = re.compile(r"\b(?:self\.)?engine\.([a-z_][a-z0-9_]*)\(")


def test_all_klippy_engine_call_sites_resolve():
    surface = _real_engine_surface()
    unresolved = {}
    for path in sorted(KLIPPY_DIR.rglob("*.py")):
        names = set(ENGINE_CALL_RE.findall(path.read_text()))
        missing = names - surface
        if missing:
            unresolved[str(path.relative_to(KLIPPY_DIR))] = sorted(missing)
    assert not unresolved, (
        "klippy calls engine methods that exist neither on the native "
        "MotionEngine nor on MotionEngineWrapper: %s" % unresolved
    )
