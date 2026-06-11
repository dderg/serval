# Probe Interface Restoration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the probe orchestration surface (`ProbePointsHelper`, `get_lift_speed`, `multi_probe_begin/end`, full-position `run_probe`) so bed_mesh / z_tilt / quad_gantry_level / screws_tilt_adjust / axis_twist_compensation load and run, with z_tilt's per-motor adjust step failing loudly.

**Architecture:** All host Python. `klippy/extras/probe.py` gains the accessor methods and a `ProbePointsHelper` ported from this repo's `main` branch (`git show main:klippy/extras/probe.py`, class at line 824) minus the RetrySession/nozzle-scrubber tier. `z_tilt.py`'s `ZAdjustHelper.adjust_steppers` is replaced with a report-then-raise stub (the bridge has no per-motor move primitive; the legacy `set_trapq` juggling would silently move all Z motors in lockstep). Spec: `docs/superpowers/specs/2026-06-11-probe-interface-restoration-design.md`.

**Tech Stack:** Python (klippy extras), pytest (`test/` directory, run from repo root), kalico-sim (`tools/kalico-sim/runner.py` probe-test variants, run in Docker via `tools/kalico-sim/run.sh`).

**Conventions that apply:**
- Unit tests live in a separate file from the tested code (repo rule).
- No comments narrating code; fail loudly on unexpected states.
- Python files touched get `ruff format` before commit (repo uses ruff).
- No `Co-Authored-By` trailers in commits.

---

### Task 1: `PrinterProbe` accessors and full-position `run_probe`

**Files:**
- Modify: `klippy/extras/probe.py`
- Test: `test/test_probe_logic.py` (extend)

- [ ] **Step 1: Write the failing tests**

Append to `test/test_probe_logic.py`:

```python
class _FakeGCmd:
    def __init__(self, params=None):
        self._params = params or {}

    def get_float(self, name, default=None, above=None, minval=None):
        return float(self._params.get(name, default))


def test_get_lift_speed_returns_config_value_without_gcmd():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    probe.lift_speed = 7.5
    assert probe.get_lift_speed() == 7.5


def test_get_lift_speed_honors_gcmd_override():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    probe.lift_speed = 7.5
    assert probe.get_lift_speed(_FakeGCmd({"LIFT_SPEED": 3.0})) == 3.0
    assert probe.get_lift_speed(_FakeGCmd()) == 7.5


def test_multi_probe_lifecycle_is_noop():
    from klippy.extras.probe import PrinterProbe

    probe = PrinterProbe.__new__(PrinterProbe)
    assert probe.multi_probe_begin() is None
    assert probe.multi_probe_end() is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_probe_logic.py -v`
Expected: the three new tests FAIL with `AttributeError: 'PrinterProbe' object has no attribute 'get_lift_speed'` (and `multi_probe_begin`); existing tests PASS.

- [ ] **Step 3: Implement the accessors and result-shape change**

In `klippy/extras/probe.py`, add three methods to `PrinterProbe` after `get_offsets` (line 101):

```python
    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass
```

Change the tail of `run_probe` (currently `return calc_probe_z_result(measured, method)`) to return the full toolhead position with the measured Z:

```python
        epos = list(toolhead.get_position()[:3])
        epos[Z_AXIS] = calc_probe_z_result(measured, method)
        return epos
```

Adapt `cmd_PROBE` to the new shape (replace the whole method body):

```python
    def cmd_PROBE(self, gcmd):
        pos = self.run_probe(gcmd)
        gcmd.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (pos[0], pos[1], pos[2])
        )
        self.last_z_result = pos[2]
```

(The old body read `toolhead.get_position()` before probing; the new one reports the position `run_probe` returned. No other callers of `run_probe` exist in our tree yet except `axis_twist_compensation.py:322`, which already expects `pos[2]` — the new shape fixes it.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_probe_logic.py -v`
Expected: all PASS.

- [ ] **Step 5: Format and commit**

```bash
ruff format klippy/extras/probe.py test/test_probe_logic.py
git add klippy/extras/probe.py test/test_probe_logic.py
git commit -m "feat(probe): get_lift_speed, multi_probe lifecycle, full-position run_probe"
```

---

### Task 2: `ZAdjustHelper.adjust_steppers` fail-loud stub

**Files:**
- Modify: `klippy/extras/z_tilt.py:37-77` (the `adjust_steppers` method)
- Test: `test/test_z_tilt_adjust_stub.py` (create)

- [ ] **Step 1: Write the failing test**

Create `test/test_z_tilt_adjust_stub.py`:

```python
import pytest


class _FakeGCode:
    def __init__(self):
        self.messages = []

    def respond_info(self, msg):
        self.messages.append(msg)


class _CommandError(Exception):
    pass


class _FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class _FakePrinter:
    command_error = _CommandError

    def __init__(self):
        self.gcode = _FakeGCode()
        self.handlers = []

    def register_event_handler(self, event, handler):
        self.handlers.append((event, handler))

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode


class _FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "z_tilt"


def test_adjust_steppers_reports_then_raises_not_implemented():
    from klippy.extras.z_tilt import ZAdjustHelper

    printer = _FakePrinter()
    helper = ZAdjustHelper(_FakeConfig(printer), 2)
    helper.z_steppers = [_FakeStepper("stepper_z"), _FakeStepper("stepper_z1")]
    with pytest.raises(_CommandError, match="not yet implemented"):
        helper.adjust_steppers([0.01, -0.01], 5.0)
    assert any("stepper_z1 = -0.01" in m for m in printer.gcode.messages)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_z_tilt_adjust_stub.py -v`
Expected: FAIL — the current implementation calls `printer.lookup_object("toolhead")`, which trips the fake's `assert name == "gcode"` (no raise of `_CommandError`).

- [ ] **Step 3: Replace `adjust_steppers` with the stub**

In `klippy/extras/z_tilt.py`, replace the entire `adjust_steppers` method (lines 37-77, from `def adjust_steppers` through `toolhead.set_position(curpos)`) with:

```python
    def adjust_steppers(self, adjustments, speed):
        gcode = self.printer.lookup_object("gcode")
        stepstrs = [
            "%s = %.6f" % (s.get_name(), a)
            for s, a in zip(self.z_steppers, adjustments)
        ]
        gcode.respond_info(
            "Z adjustments needed:\n%s" % ("\n".join(stepstrs),)
        )
        raise self.printer.command_error(
            "per-motor Z adjustment is not yet implemented"
        )
```

Keep the `import logging` line — `logging` is still used at z_tilt.py:200 and :224.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_z_tilt_adjust_stub.py -v`
Expected: PASS.

- [ ] **Step 5: Format and commit**

```bash
ruff format klippy/extras/z_tilt.py test/test_z_tilt_adjust_stub.py
git add klippy/extras/z_tilt.py test/test_z_tilt_adjust_stub.py
git commit -m "feat(z_tilt): fail loudly on per-motor Z adjustment under the bridge"
```

---

### Task 3: `ProbePointsHelper` port

**Files:**
- Modify: `klippy/extras/probe.py` (add imports + class)
- Test: `test/test_probe_points_helper.py` (create)

Reference: `git show main:klippy/extras/probe.py` lines 824-1006 is the Kalico original. The port below deletes the `RetrySession` plumbing, the nozzle scrubber, and the rapid_scan silent downgrade (any METHOD other than `automatic`/`manual` is now a hard error).

- [ ] **Step 1: Write the failing tests**

Create `test/test_probe_points_helper.py`:

```python
import pytest


class _GCodeError(Exception):
    pass


class _ConfigError(Exception):
    pass


class _FakeGCmd:
    error = _GCodeError

    def __init__(self, params=None):
        self._params = params or {}

    def get(self, name, default=None):
        return self._params.get(name, default)

    def get_float(self, name, default=None, above=None, minval=None):
        return float(self._params.get(name, default))

    def get_int(self, name, default=None, minval=None, maxval=None):
        v = self._params.get(name, default)
        return v if v is None else int(v)


class _FakeGCode:
    def create_gcode_command(self, command, commandline, params):
        return _FakeGCmd(params)

    def register_command(self, cmd, func, *args, **kwargs):
        pass


class _FakeToolhead:
    def __init__(self):
        self.moves = []
        self.position = [0.0, 0.0, 10.0, 0.0]

    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

    def get_last_move_time(self):
        return 0.0

    def get_position(self):
        return list(self.position)


class _FakeProbe:
    def __init__(self, offsets=(0.0, 0.0, 1.5), measured_z=1.5):
        self.offsets = offsets
        self.measured_z = measured_z
        self.lifecycle = []
        self.probes = 0

    def get_lift_speed(self, gcmd=None):
        return 4.0

    def get_offsets(self):
        return self.offsets

    def multi_probe_begin(self):
        self.lifecycle.append("begin")

    def multi_probe_end(self):
        self.lifecycle.append("end")

    def run_probe(self, gcmd):
        self.probes += 1
        return [10.0 * self.probes, 20.0, self.measured_z]


class _FakePrinter:
    config_error = _ConfigError

    def __init__(self, objects):
        self.objects = objects

    def lookup_object(self, name, default="__raise__"):
        if name in self.objects:
            return self.objects[name]
        if default == "__raise__":
            raise self.config_error("unknown object %s" % (name,))
        return default


class _FakeConfig:
    def __init__(self, printer, options=None):
        self._printer = printer
        self._options = options or {}

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "fake_section"

    def get(self, name, default="__required__"):
        return self._options.get(name, None if default == "__required__" else default)

    def getfloat(self, name, default=None, above=None, minval=None):
        v = self._options.get(name, default)
        return v if v is None else float(v)

    def getboolean(self, name, default=None):
        return bool(self._options.get(name, default))

    def getlists(self, name, seps=(",", "\n"), parser=float, count=2):
        raw = self._options[name]
        return [
            tuple(parser(p) for p in line.split(","))
            for line in raw.strip().split("\n")
        ]


def _make_helper(probe_module, finalize, points="10,10\n50,50", options=None):
    toolhead = _FakeToolhead()
    fake_probe = _FakeProbe()
    printer = _FakePrinter(
        {"gcode": _FakeGCode(), "toolhead": toolhead, "probe": fake_probe}
    )
    opts = {"points": points}
    opts.update(options or {})
    config = _FakeConfig(printer, opts)
    helper = probe_module.ProbePointsHelper(config, finalize)
    return helper, toolhead, fake_probe


@pytest.fixture()
def probe_module(monkeypatch):
    from klippy.extras import manual_probe, probe

    monkeypatch.setattr(
        manual_probe, "verify_no_manual_probe", lambda printer: None
    )
    return probe


def test_automatic_probing_collects_all_points(probe_module):
    calls = []

    def finalize(offsets, results):
        calls.append((offsets, [list(r) for r in results]))

    helper, toolhead, fake_probe = _make_helper(probe_module, finalize)
    helper.start_probe(_FakeGCmd())
    assert fake_probe.probes == 2
    assert fake_probe.lifecycle == ["begin", "end"]
    assert len(calls) == 1
    offsets, results = calls[0]
    assert offsets == (0.0, 0.0, 1.5)
    assert results == [[10.0, 20.0, 1.5], [20.0, 20.0, 1.5]]


def test_finalize_retry_reprobes_batch(probe_module):
    outcomes = iter(["retry", None])
    calls = []

    def finalize(offsets, results):
        calls.append(len(results))
        return next(outcomes)

    helper, toolhead, fake_probe = _make_helper(probe_module, finalize)
    helper.start_probe(_FakeGCmd())
    assert calls == [2, 2]
    assert fake_probe.probes == 4


def test_use_offsets_shifts_target_positions(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    helper.use_xy_offsets(True)
    fake_probe.offsets = (24.0, 5.0, 1.5)
    helper.start_probe(_FakeGCmd())
    xy_moves = [m for m, speed in toolhead.moves if m[0] is not None]
    assert xy_moves[0][:2] == [10.0 - 24.0, 10.0 - 5.0]


def test_horizontal_move_z_below_z_offset_rejected(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    fake_probe.offsets = (0.0, 0.0, 6.0)
    with pytest.raises(_GCodeError, match="horizontal_move_z"):
        helper.start_probe(_FakeGCmd())


def test_rapid_scan_method_rejected(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    with pytest.raises(_GCodeError, match="METHOD"):
        helper.start_probe(_FakeGCmd({"METHOD": "rapid_scan"}))


def test_minimum_points_enforced(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    with pytest.raises(_ConfigError):
        helper.minimum_points(3)
    helper.minimum_points(2)


def test_update_probe_points_replaces_and_validates(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    helper.update_probe_points([(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)], 3)
    assert helper.get_probe_points() == [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
    with pytest.raises(_ConfigError):
        helper.update_probe_points([(1.0, 1.0)], 2)


def test_helper_get_lift_speed_uses_probe_value_after_start(probe_module):
    helper, toolhead, fake_probe = _make_helper(
        probe_module, lambda o, r: None
    )
    helper.start_probe(_FakeGCmd())
    assert helper.get_lift_speed() == 4.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_probe_points_helper.py -v`
Expected: FAIL at import/attribute level — `module 'klippy.extras.probe' has no attribute 'ProbePointsHelper'`.

- [ ] **Step 3: Add the class to `klippy/extras/probe.py`**

Add imports at the top of the file (after `from klippy import pins`):

```python
import math

from . import manual_probe
```

Append the class after `PrinterProbe` (before `load_config`):

```python
class ProbePointsHelper:
    def __init__(
        self,
        config,
        finalize_callback,
        default_points=None,
        option_name="points",
        use_offsets=False,
        enable_horizontal_z_clearance=False,
    ):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object("gcode")
        if default_points is None or config.get(option_name, None) is not None:
            self.probe_points = config.getlists(
                option_name, seps=(",", "\n"), parser=float, count=2
            )
        def_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.horizontal_move_z = self.default_horizontal_move_z = def_move_z
        self.enable_horizontal_z_clearance = enable_horizontal_z_clearance
        self.horizontal_z_clearance = self.default_horizontal_z_clearance = None
        if enable_horizontal_z_clearance:
            z_clearance = config.getfloat("horizontal_z_clearance", None)
            self.default_horizontal_z_clearance = z_clearance
            self.horizontal_z_clearance = z_clearance
        self.adaptive_horizontal_move_z = config.getboolean(
            "adaptive_horizontal_move_z", False
        )
        self.min_horizontal_move_z = config.getfloat(
            "min_horizontal_move_z", 1.0
        )
        self.speed = config.getfloat("speed", 50.0, above=0.0)
        self.use_offsets = config.getboolean(
            "use_probe_xy_offsets", use_offsets
        )
        self.enforce_lift_speed = config.getboolean("enforce_lift_speed", False)
        self.lift_speed = self.speed
        self.probe_offsets = (0.0, 0.0, 0.0)
        self.results = []

    def get_probe_points(self):
        return self.probe_points

    def minimum_points(self, n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name)
            )

    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)

    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        return self.lift_speed

    def _lift_toolhead(self):
        toolhead = self.printer.lookup_object("toolhead")
        speed = self.lift_speed
        if not self.results and not self.enforce_lift_speed:
            speed = self.speed
        z_pos = self.horizontal_move_z
        if self.horizontal_z_clearance is not None and self.results:
            z_pos = toolhead.get_position()[2] + self.horizontal_z_clearance
        toolhead.manual_move([None, None, z_pos], speed)

    def _next_pos(self):
        nextpos = list(self.probe_points[len(self.results)])
        if self.use_offsets:
            nextpos[0] -= self.probe_offsets[0]
            nextpos[1] -= self.probe_offsets[1]
        return nextpos

    def _move_next(self):
        toolhead = self.printer.lookup_object("toolhead")
        done = False
        finalize = len(self.results) >= len(self.probe_points)
        if finalize:
            toolhead.get_last_move_time()
            res = self.finalize_callback(self.probe_offsets, self.results)
            if isinstance(res, (int, float)):
                if res == 0:
                    done = True
                if self.adaptive_horizontal_move_z:
                    error = math.ceil(res)
                    self.horizontal_move_z = max(
                        error + self.probe_offsets[2],
                        self.min_horizontal_move_z,
                    )
            elif res != "retry":
                done = True
        self._lift_toolhead()
        if finalize:
            self.results = []
        if done:
            return True
        toolhead.manual_move(self._next_pos(), self.speed)
        return False

    def start_probe(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        probe = self.printer.lookup_object("probe", None)
        method = gcmd.get("METHOD", "automatic").lower()
        if method not in ("automatic", "manual"):
            raise gcmd.error(
                "METHOD=%s is not supported (use automatic or manual)"
                % (method,)
            )
        self.results = []
        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float("HORIZONTAL_MOVE_Z", def_move_z)
        if self.enable_horizontal_z_clearance:
            self.horizontal_z_clearance = gcmd.get_float(
                "HORIZONTAL_Z_CLEARANCE", self.default_horizontal_z_clearance
            )
        enforce_lift_speed = gcmd.get_int(
            "ENFORCE_LIFT_SPEED", None, minval=0, maxval=1
        )
        if enforce_lift_speed is not None:
            self.enforce_lift_speed = enforce_lift_speed
        if probe is None or method == "manual":
            self.lift_speed = self.speed
            self.probe_offsets = (0.0, 0.0, 0.0)
            self._manual_probe_start()
            return
        self.lift_speed = probe.get_lift_speed(gcmd)
        self.probe_offsets = probe.get_offsets()
        if self.horizontal_move_z < self.probe_offsets[2]:
            raise gcmd.error(
                "horizontal_move_z can't be less than probe's z_offset"
            )
        probe.multi_probe_begin()
        while True:
            done = self._move_next()
            if done:
                break
            pos = probe.run_probe(gcmd)
            self.results.append(pos)
        probe.multi_probe_end()

    def _manual_probe_start(self):
        done = self._move_next()
        if not done:
            gcmd = self.gcode.create_gcode_command("", "", {})
            manual_probe.ManualProbeHelper(
                self.printer, gcmd, self._manual_probe_finalize
            )

    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.results.append(kin_pos)
        self._manual_probe_start()
```

Differences from the `main`-branch original, for the record: no `RetrySession` (`_next_pos` returns the plain point; `start_probe` has no `retry_session.start/reset_all/end` calls and `run_probe` is called without a session argument), no `logging.info` per probe, and unknown `METHOD` values (including `rapid_scan`) raise instead of silently probing manually or downgrading.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_probe_points_helper.py test/test_probe_logic.py -v`
Expected: all PASS.

- [ ] **Step 5: Verify consumers import cleanly**

Run: `python3 -c "import klippy.extras.z_tilt, klippy.extras.quad_gantry_level, klippy.extras.screws_tilt_adjust, klippy.extras.bed_mesh, klippy.extras.axis_twist_compensation; print('ok')"`
Expected: `ok` (these modules reference `probe.ProbePointsHelper` at class-construction time, not import time, but the import check catches syntax/import-level breakage).

- [ ] **Step 6: Format and commit**

```bash
ruff format klippy/extras/probe.py test/test_probe_points_helper.py
git add klippy/extras/probe.py test/test_probe_points_helper.py
git commit -m "feat(probe): restore ProbePointsHelper (Kalico interface, no retry tier)"
```

---

### Task 4: kalico-sim end-to-end variant `points`

**Files:**
- Modify: `tools/kalico-sim/runner.py` (`PROBE_TEST_VARIANTS` at line 1184, `_generate_probe_config` at line 1201, `run_probe_test` command sequence around line 1540)

- [ ] **Step 1: Add the variant and config sections**

In `PROBE_TEST_VARIANTS` (runner.py:1184), add `"points"`:

```python
PROBE_TEST_VARIANTS = (
    "virtual",
    "safe-z",
    "gpio-z",
    "no-probe",
    "conflict",
    "pullup",
    "remote",
    "points",
)
```

In `_generate_probe_config`, treat `points` like `virtual` for the Z endstop (the final `else` branch already does this — `endstop_pin: probe:z_virtual_endstop`, `probe_pin = "gpiochip0/gpio202"`), and add an ecosystem section block. After the `remote_section` assignment block, add:

```python
    points_sections = ""
    if variant == "points":
        points_sections = """
[stepper_z1]
step_pin: gpiochip0/gpio9
dir_pin: gpiochip0/gpio10
enable_pin: !gpiochip0/gpio11
microsteps: 16
rotation_distance: 4

[z_tilt]
z_positions:
    0, 125
    250, 125
points:
    50, 125
    200, 125
speed: 50
horizontal_move_z: 8

[bed_mesh]
mesh_min: 30, 10
mesh_max: 200, 200
probe_count: 3, 3
speed: 50
horizontal_move_z: 8

[screws_tilt_adjust]
screw1: 50, 50
screw1_name: front left
screw2: 200, 50
screw2_name: front right
screw3: 125, 200
screw3_name: back
speed: 50
horizontal_move_z: 8
screw_thread: CW-M4

[axis_twist_compensation]
calibrate_start_x: 30
calibrate_end_x: 200
calibrate_y: 125
"""
```

and append `{points_sections}` to the returned f-string right after `{safe_z_section}{probe_section}{remote_section}`.

(`[stepper_z1]` joins the Z rail via `BridgeKinematics._register_axis(..., extras=("1",))` — motion_toolhead.py:96 — giving z_tilt the two Z steppers it requires; the bridge drives both in lockstep on slot 2, which is fine for everything except the adjust step we expect to fail. mesh_min respects the probe offsets `x_offset: 24, y_offset: 5` already in the generated `[probe]` section.)

- [ ] **Step 2: Add the command sequence to `run_probe_test`**

In `run_probe_test`, the non-remote branch currently runs QUERY_PROBE → PROBE-before-home → G28 → PROBE → PROBE_ACCURACY → QUERY_PROBE. For `variant == "points"`, after the existing `query-probe-open-after` check, add:

```python
                    if variant == "points":
                        resp = send_gcode(
                            api_socket, "SCREWS_TILT_ADJUST", timeout=300
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        check(
                            "screws-tilt-adjust",
                            not resp.get("error")
                            and "front left" in out
                            and "back" in out,
                            resp.get("error") or "screw report present",
                        )

                        resp = send_gcode(
                            api_socket, "BED_MESH_CALIBRATE", timeout=600
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        check(
                            "bed-mesh-calibrate",
                            not resp.get("error")
                            and "Mesh Bed Leveling Complete" in out,
                            resp.get("error") or "mesh completed",
                        )

                        resp = send_gcode(
                            api_socket, "Z_TILT_ADJUST", timeout=300
                        )
                        out, offset = _log_tail_since(klippy_log, offset)
                        err = str(resp.get("error", ""))
                        check(
                            "z-tilt-fails-loudly",
                            "per-motor Z adjustment is not yet implemented"
                            in err,
                            err or "expected not-yet-implemented error",
                        )
                        check(
                            "z-tilt-reports-adjustments",
                            "Z adjustments needed" in out,
                            "measured deviations reported before the raise",
                        )
```

The expected-Z assertions for the existing checks key off `variant`; `points` shares the `virtual` expectations. Update the two spots that special-case variants:

- `g28-z-trigger-height` check condition: `if variant in ("virtual", "safe-z", "points"):`
- `expected_z` selection: `elif variant in ("virtual", "points"): expected_z = 6.5`

The final `no-shutdown` check already runs for every variant and covers "Z_TILT_ADJUST errored but did not shut the printer down".

(The completion message is verified present at `klippy/extras/bed_mesh.py:1144`: `self.gcode.respond_info("Mesh Bed Leveling Complete")`.)

- [ ] **Step 3: Run the sim variant**

Run from repo root: `./tools/kalico-sim/run.sh --probe-test points`
(Docker build + run; consult the `kalico-sim` skill if invocation details differ.)
Expected output: `PROBE TEST RESULT (variant=points)` with every check PASS, including `screws-tilt-adjust`, `bed-mesh-calibrate`, `z-tilt-fails-loudly`, `z-tilt-reports-adjustments`, `no-shutdown`.

- [ ] **Step 4: Run the pre-existing variants to confirm no regression**

Run: `./tools/kalico-sim/run.sh --probe-test virtual` and `./tools/kalico-sim/run.sh --probe-test remote`
Expected: all checks PASS on both.

- [ ] **Step 5: Format and commit**

```bash
ruff format tools/kalico-sim/runner.py
git add tools/kalico-sim/runner.py
git commit -m "test(sim): probe ecosystem variant — bed_mesh, screws, z_tilt fail-loud"
```

---

### Task 5: Full verification pass

**Files:** none (verification only)

- [ ] **Step 1: Full Python test suite**

Run: `python3 -m pytest test/ -x -q`
Expected: all tests pass (pre-existing failures, if any, must match a clean checkout — verify with `git stash && python3 -m pytest test/ -x -q && git stash pop` if anything unrelated fails).

- [ ] **Step 2: Rust suite untouched sanity**

Run from `rust/`: `cargo nextest run`
Expected: pass (no Rust files were modified; this catches accidental cross-tree breakage only).

- [ ] **Step 3: Format check**

Run: `ruff format --check klippy/extras/probe.py klippy/extras/z_tilt.py test/test_probe_logic.py test/test_probe_points_helper.py test/test_z_tilt_adjust_stub.py tools/kalico-sim/runner.py`
Expected: no diffs. (Rust untouched, so `cargo fmt --all --check` is not required, but run it if any `.rs` file shows in `git status`.)

- [ ] **Step 4: Review the diff against the spec**

Run: `git log --oneline d4fb9293c..HEAD` and `git diff d4fb9293c..HEAD --stat`
Confirm every spec section maps to a commit: PrinterProbe accessors (Task 1), fail-loud stub (Task 2), ProbePointsHelper (Task 3), sim coverage (Task 4).
