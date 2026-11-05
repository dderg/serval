from __future__ import annotations

from klippy import motion_engine

if motion_engine._native is None:
    raise RuntimeError(
        "test/motion_engine exercises the native _motion_engine module, which "
        "is not built; build it with '%s'." % motion_engine.NATIVE_BUILD_HINT
    )
