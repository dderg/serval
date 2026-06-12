# Beacon Fork Seam Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the `dderg/beacon_klipper` integration layer onto this tree's provider contract / `motion_state_at` / probe interface, and validate the full capability matrix in kalico-sim.

**Architecture:** All new integration code lives in one new file in the fork repo, `beacon_kalico.py` (class `KalicoSeam`); `beacon.py` keeps its device-protocol bulk and delegates through one-line seams. The dead upstream integration classes (trsync/trdispatch ceremony, endstop contract wrappers, homing-event listeners) are deleted. Kalico-repo changes are limited to the sim harness (emulator + runner test variants).

**Tech Stack:** Python (fork: plain pytest with fakes; kalico: kalico-sim docker harness). No Rust changes.

**Spec:** `docs/superpowers/specs/2026-06-12-beacon-fork-seam-design.md`

---

## Reference map (read these before starting)

| What | Where |
|---|---|
| Reference provider (the pattern KalicoSeam follows) | `klippy/extras/sim_remote_endstop.py` (whole file, 130 lines) |
| `RemoteBridgeEndstop` (endstop the seam returns) | `klippy/bridge_endstop.py:62-96` |
| Provider contract dispatch + `trip_move` | `klippy/extras/homing.py:75-80` (`_homed_axis_position`), `158-199` (resolve), `352-420` (`trip_move`) |
| `trip_move` caller pattern (descents copy this) | `klippy/extras/probe.py` `_probe_once` |
| `motion_state_at` Python API | `klippy/motion_bridge.py:436-445` |
| `motion_state_at` error strings (classify against these) | `rust/motion-bridge/src/motion_history.rs:10-35` |
| Beacon emulator | `tools/kalico-sim/emulators/beacon_mcu.py` |
| Sim runner beacon wiring | `tools/kalico-sim/runner.py:482-512` (third_party), `575-623` (homing test), `1192+` (config gen), `2353+` (CLI) |
| Fork seam call sites | beacon.py: imports 10-35, init 143-198, `setup_pin` 457, `_probing_move_to_probing_height` 523, `_probe_contact` 618, poke 1401, autocal 1483, streaming 938-1040, classes 2167-2638 |

Key facts established during planning (do not re-derive):

- Our klippy puts the **repo root** on `sys.path` (`klippy/klippy.py:7`), so extras must use `from klippy import pins` style. beacon.py's bare `import pins` / `from mcu import MCU` **will not import** — this is the first failure the port fixes.
- `bridge.motion_state_at(mcu, clock=…)` accepts a clock **in the source MCU's domain** (the beacon's own ticks) and converts internally. Returns `{"x"|"y"|"z"|"e": (pos, vel, accel)}` and **raises RuntimeError if any configured axis fails** — the error strings to classify are `"is in the future"`, `"precedes retained motion history"`, `"no motion history recorded"`.
- The relay (`RemoteBridgeEndstop.arm` → Rust interceptor) fires on **any** terminal `trsync_state` (`can_trigger == 0`) regardless of reason; reason validation is the provider's job (`trip_move_end` / `measured_trip_position`).
- In `trip_move`'s `finally`, `endstop.disarm()` runs **before** `provider.trip_move_end(entry)` (homing.py:402-412), and `trip_move_end` exceptions are NOT swallowed — `trip_move_end` must therefore force a terminal report itself (send `trsync_trigger reason=HOST_REQUEST`) instead of waiting for one that may never come, and must accept HOST_REQUEST as a clean outcome (the success-path reason check lives in `measured_trip_position` / the descend helper, which only run on success).
- The trsync deadman on the beacon side is **not** the motion deadman (that is the drip window on the motion MCU). A relaxed 200 ms window with a 50 ms heartbeat is correct; do not copy upstream's 25 ms `trsync_timeout`.
- Docker `COPY`s the worktree, so the fork checkout used by sim must be a real directory at `tools/sim_klippy/printer_real/third_party_repos/beacon_klipper` (path is gitignored). That clone IS the working copy for all fork tasks.
- Trident-bench memory `feedback_bench_firmware_flow` does not apply here — no MCU firmware changes; this is host Python + sim only.

**Spec deviation (approved rationale inline):** the spec lists `BeaconHomingState` among deleted classes, but it is the argument beacon passes when **sending** `homing:home_rails_begin/end` around auto-calibrate, and the spec separately requires those self-sent events to keep working. Keep the class; delete only the **listener** pair.

---

### Task 1: Fork working copy, upstream remote, red baseline

**Files:**
- Create: `tools/sim_klippy/printer_real/third_party_repos/beacon_klipper/` (git clone, gitignored)

- [ ] **Step 1: Clone the fork into the sim's third_party path and add the upstream remote**

```bash
mkdir -p tools/sim_klippy/printer_real/third_party_repos
git clone git@github.com:dderg/beacon_klipper.git \
    tools/sim_klippy/printer_real/third_party_repos/beacon_klipper
cd tools/sim_klippy/printer_real/third_party_repos/beacon_klipper
git remote add upstream https://github.com/beacon3d/beacon_klipper.git
git fetch upstream
git log --oneline -1   # expect 7c71e98 "Compatibility fix"
```

If `beacon3d/beacon_klipper` is not the upstream URL (404 on fetch), find the real one with `gh api "search/repositories?q=beacon_klipper" --jq '.items[].full_name'` and use that; record it in the final commit message.

- [ ] **Step 2: Run the existing proximity homing sim test to capture the red baseline**

```bash
cd /Users/daniladergachev/Developer/kalico/.worktrees/beacon-support
tools/kalico-sim/run.sh --homing-test
```

Expected: FAIL. The klippy log must show beacon failing to load — either `ImportError` on `import pins` / `from mcu import MCU` (bare-import style) or `from .homing import HomingMove` (class deleted). Save the exact error text; it is the baseline this project turns green. If it fails for an unrelated reason (docker build, MCU sim), fix that first — it predates this work.

- [ ] **Step 3: Commit nothing yet** (the clone is gitignored; baseline is informational).

---

### Task 2: `beacon_kalico.py` — module skeleton and pure helpers

All fork-side tasks run in `tools/sim_klippy/printer_real/third_party_repos/beacon_klipper/`.

**Files:**
- Create: `beacon_kalico.py`
- Create: `test_beacon_kalico.py`

- [ ] **Step 1: Write the failing tests for error classification**

```python
# test_beacon_kalico.py
import sys
import types


def _install_klippy_stubs():
    if "klippy" in sys.modules:
        return
    klippy = types.ModuleType("klippy")
    pins = types.ModuleType("klippy.pins")

    class PinsError(Exception):
        pass

    pins.error = PinsError
    bridge_endstop = types.ModuleType("klippy.bridge_endstop")

    class FakeRemoteBridgeEndstop:
        def __init__(self, printer, mcu, trsync_oid):
            self.printer = printer
            self.mcu = mcu
            self.trsync_oid = trsync_oid
            self.endstop_id = 99

    bridge_endstop.RemoteBridgeEndstop = FakeRemoteBridgeEndstop
    klippy.pins = pins
    sys.modules["klippy"] = klippy
    sys.modules["klippy.pins"] = pins
    sys.modules["klippy.bridge_endstop"] = bridge_endstop


_install_klippy_stubs()

import beacon_kalico  # noqa: E402


def test_classify_future():
    msg = (
        "motion_state_at: query clock 123 is in the future for axis "
        "AxisKey { mcu_id: 1, axis: 2 } (now≈100) — motion history "
        "answers the past only"
    )
    assert beacon_kalico.classify_history_error(msg) == beacon_kalico.ERR_FUTURE


def test_classify_before_window():
    msg = "query clock 5 precedes retained motion history for axis ..."
    assert (
        beacon_kalico.classify_history_error(msg)
        == beacon_kalico.ERR_BEFORE_WINDOW
    )


def test_classify_no_history():
    msg = "no motion history recorded for axis AxisKey { mcu_id: 1, axis: 2 }"
    assert (
        beacon_kalico.classify_history_error(msg)
        == beacon_kalico.ERR_NO_HISTORY
    )


def test_classify_unknown_is_none():
    assert beacon_kalico.classify_history_error("segfault adjacent") is None
```

- [ ] **Step 2: Run to verify failure**

```bash
cd tools/sim_klippy/printer_real/third_party_repos/beacon_klipper
python3 -m pytest test_beacon_kalico.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'beacon_kalico'`.

- [ ] **Step 3: Write the module skeleton**

```python
# beacon_kalico.py
# Kalico integration seam for the Beacon fork. Everything that touches the
# kalico motion engine lives in this file; beacon.py keeps the device
# protocol and delegates here. Design:
# docs/superpowers/specs/2026-06-12-beacon-fork-seam-design.md (kalico repo).
import logging

from klippy import pins
from klippy.bridge_endstop import RemoteBridgeEndstop

REASON_ENDSTOP_HIT = 1
REASON_HOST_REQUEST = 2
REASON_COMMS_TIMEOUT = 4

MODE_PROXIMITY = "proximity"
MODE_CONTACT = "contact"

Z_AXIS = 2

TRSYNC_WINDOW = 0.200
TRSYNC_HEARTBEAT = 0.050
TERMINAL_REASON_DEADLINE = 2.0
FUTURE_RETRY_PAUSE = 0.050
CRUISE_ACCEL_TOLERANCE = 1.0

ERR_FUTURE = "future"
ERR_BEFORE_WINDOW = "before_window"
ERR_NO_HISTORY = "no_history"


def classify_history_error(message):
    if "is in the future" in message:
        return ERR_FUTURE
    if "precedes retained motion history" in message:
        return ERR_BEFORE_WINDOW
    if "no motion history recorded" in message:
        return ERR_NO_HISTORY
    return None
```

- [ ] **Step 4: Run tests, expect PASS**

```bash
python3 -m pytest test_beacon_kalico.py -v
```

- [ ] **Step 5: Commit (fork repo)**

```bash
git add beacon_kalico.py test_beacon_kalico.py
git commit -m "kalico seam: module skeleton + motion-history error classification"
```

---

### Task 3: `KalicoSeam` — trsync plumbing, heartbeat, terminal reason

**Files:**
- Modify: `beacon_kalico.py`
- Modify: `test_beacon_kalico.py`

- [ ] **Step 1: Write failing tests with fakes**

Append to `test_beacon_kalico.py`:

```python
class FakeReactor:
    NEVER = 9e99

    def __init__(self):
        self.now = 100.0
        self.timers = []
        self.paused = []

    def monotonic(self):
        return self.now

    def register_timer(self, cb, when):
        self.timers.append((cb, when))
        return (cb, when)

    def unregister_timer(self, handle):
        self.timers.remove(handle)

    def pause(self, until):
        self.paused.append(until)
        self.now = max(self.now, until)


class FakeCommandError(Exception):
    pass


class FakePrinter:
    def __init__(self, objects=None):
        self.command_error = FakeCommandError
        self.config_error = FakeCommandError
        self.reactor = FakeReactor()
        self.objects = objects or {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default="__raise__"):
        if name in self.objects:
            return self.objects[name]
        if default != "__raise__":
            return default
        raise FakeCommandError("missing object %s" % name)


class FakeCommand:
    def __init__(self, log):
        self.log = log

    def send(self, args=()):
        self.log.append(list(args))


class FakeMcu:
    def __init__(self):
        self.oids = 0
        self.config_cmds = []
        self.responses = {}
        self.sent = {}
        self.config_cbs = []

    def create_oid(self):
        self.oids += 1
        return self.oids

    def register_config_callback(self, cb):
        self.config_cbs.append(cb)

    def add_config_cmd(self, cmd):
        self.config_cmds.append(cmd)

    def lookup_command(self, fmt, cq=None):
        name = fmt.split()[0]
        self.sent.setdefault(name, [])
        return FakeCommand(self.sent[name])

    def register_response(self, cb, name, oid=None):
        self.responses[(name, oid)] = cb

    def estimated_print_time(self, eventtime):
        return eventtime

    def print_time_to_clock(self, print_time):
        return int(print_time * 1000)

    def clock32_to_clock64(self, clock32):
        return clock32


class FakeBeacon:
    def __init__(self, printer, mcu):
        self.printer = printer
        self._mcu = mcu
        self.model = object()
        self.trigger_distance = 2.0
        self.z_settling_time = 1
        self.applied_thresholds = 0
        self.sampled_async = 0
        self.cmd_log = {}
        for name in (
            "beacon_home_cmd",
            "beacon_stop_home_cmd",
            "beacon_contact_home_cmd",
            "beacon_contact_stop_home_cmd",
        ):
            log = []
            self.cmd_log[name] = log
            setattr(self, name, FakeCommand(log))
        self.beacon_contact_set_latency_min_cmd = None
        self.beacon_contact_set_sensitivity_cmd = None
        self.contact_latency_min = 0
        self.contact_sensitivity = 0
        self.mcu_contact_probe = None

    def _apply_threshold(self):
        self.applied_thresholds += 1

    def _sample_async(self):
        self.sampled_async += 1
        return {"freq": 1.0, "dist": 2.0, "temp": 25.0}


def make_seam():
    printer = FakePrinter()
    mcu = FakeMcu()
    beacon = FakeBeacon(printer, mcu)
    seam = beacon_kalico.KalicoSeam(beacon)
    for cb in mcu.config_cbs:
        cb()
    return seam, beacon, printer, mcu


def test_seam_config_allocates_trsync():
    seam, beacon, printer, mcu = make_seam()
    assert mcu.config_cmds == ["config_trsync oid=%d" % seam.trsync_oid]
    assert ("trsync_state", seam.trsync_oid) in mcu.responses


def test_terminal_reason_recorded():
    seam, beacon, printer, mcu = make_seam()
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]
    handler({"can_trigger": 1, "trigger_reason": 0})
    assert seam.last_reason is None
    handler({"can_trigger": 0, "trigger_reason": 1})
    assert seam.last_reason == beacon_kalico.REASON_ENDSTOP_HIT


def test_proximity_begin_arms_device_and_heartbeat():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    assert beacon.applied_thresholds == 1
    assert beacon.sampled_async == 1
    assert mcu.sent["trsync_start"] == [
        [seam.trsync_oid, 0, 0, beacon_kalico.REASON_COMMS_TIMEOUT]
    ]
    assert beacon.cmd_log["beacon_home_cmd"] == [
        [seam.trsync_oid, beacon_kalico.REASON_ENDSTOP_HIT, 0]
    ]
    assert len(mcu.sent["trsync_set_timeout"]) == 1
    assert len(printer.reactor.timers) == 1


def test_proximity_begin_requires_model():
    seam, beacon, printer, mcu = make_seam()
    beacon.model = None
    try:
        seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                              "trigger_height": 2.0})
        assert False, "expected command_error"
    except FakeCommandError:
        pass


def test_trip_move_end_forces_terminal_and_accepts_host_request():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]

    real_send = mcu.sent["trsync_trigger"].append

    def trigger_and_report(args):
        real_send(args)
        handler({"can_trigger": 0,
                 "trigger_reason": beacon_kalico.REASON_HOST_REQUEST})

    mcu.sent["trsync_trigger"] = type(
        "L", (list,), {"append": lambda self, a: trigger_and_report(a)}
    )()
    seam.trip_move_end({})
    assert beacon.cmd_log["beacon_stop_home_cmd"] == [[]]
    assert printer.reactor.timers == []
    assert seam.last_reason == beacon_kalico.REASON_HOST_REQUEST


def test_trip_move_end_raises_on_comms_timeout():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]
    handler({"can_trigger": 0,
             "trigger_reason": beacon_kalico.REASON_COMMS_TIMEOUT})
    try:
        seam.trip_move_end({})
        assert False, "expected command_error"
    except FakeCommandError:
        pass
```

- [ ] **Step 2: Run, expect FAIL** (`AttributeError: module 'beacon_kalico' has no attribute 'KalicoSeam'`)

- [ ] **Step 3: Implement `KalicoSeam` core**

Append to `beacon_kalico.py`:

```python
class KalicoSeam:
    def __init__(self, beacon):
        self.beacon = beacon
        self.printer = beacon.printer
        self.mcu = beacon._mcu
        self.trsync_oid = self.mcu.create_oid()
        self.endstop = RemoteBridgeEndstop(
            self.printer, self.mcu, trsync_oid=self.trsync_oid
        )
        self.last_reason = None
        self._mode = None
        self._heartbeat_timer = None
        self._trsync_start_cmd = None
        self._trsync_set_timeout_cmd = None
        self._trsync_trigger_cmd = None
        self._dropped_samples = 0
        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(
            self._handle_trsync_state, "trsync_state", self.trsync_oid
        )

    def _build_config(self):
        self.mcu.add_config_cmd("config_trsync oid=%d" % (self.trsync_oid,))
        self._trsync_start_cmd = self.mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c"
        )
        self._trsync_set_timeout_cmd = self.mcu.lookup_command(
            "trsync_set_timeout oid=%c clock=%u"
        )
        self._trsync_trigger_cmd = self.mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )

    def _handle_trsync_state(self, params):
        if not params["can_trigger"]:
            self.last_reason = params["trigger_reason"]

    def _arm_trsync(self):
        self.last_reason = None
        self._trsync_start_cmd.send(
            [self.trsync_oid, 0, 0, REASON_COMMS_TIMEOUT]
        )
        reactor = self.printer.get_reactor()
        self._send_heartbeat(reactor.monotonic())
        self._heartbeat_timer = reactor.register_timer(
            self._heartbeat, reactor.monotonic() + TRSYNC_HEARTBEAT
        )

    def _send_heartbeat(self, eventtime):
        expire = self.mcu.estimated_print_time(eventtime) + TRSYNC_WINDOW
        self._trsync_set_timeout_cmd.send(
            [self.trsync_oid, self.mcu.print_time_to_clock(expire)]
        )

    def _heartbeat(self, eventtime):
        self._send_heartbeat(eventtime)
        return eventtime + TRSYNC_HEARTBEAT

    def trip_move_begin(self, entry):
        mode = self._mode if self._mode is not None else MODE_PROXIMITY
        beacon = self.beacon
        if mode == MODE_PROXIMITY:
            if beacon.model is None:
                raise self.printer.command_error("No Beacon model loaded")
            beacon._apply_threshold()
            beacon._sample_async()
            self._arm_trsync()
            beacon.beacon_home_cmd.send(
                [self.trsync_oid, REASON_ENDSTOP_HIT, 0]
            )
        else:
            self._check_hotend_temp()
            beacon._sample_async()
            self._arm_trsync()
            if beacon.beacon_contact_set_latency_min_cmd is not None:
                beacon.beacon_contact_set_latency_min_cmd.send(
                    [beacon.contact_latency_min]
                )
            if beacon.beacon_contact_set_sensitivity_cmd is not None:
                beacon.beacon_contact_set_sensitivity_cmd.send(
                    [beacon.contact_sensitivity]
                )
            beacon.beacon_contact_home_cmd.send(
                [self.trsync_oid, REASON_ENDSTOP_HIT, 0]
            )

    def _check_hotend_temp(self):
        contact_probe = self.beacon.mcu_contact_probe
        toolhead = self.printer.lookup_object("toolhead")
        extruder = toolhead.get_extruder()
        if extruder is None or contact_probe is None:
            return
        curtime = self.printer.get_reactor().monotonic()
        cur_temp = extruder.get_heater().get_status(curtime)["temperature"]
        if cur_temp >= contact_probe.max_hotend_temp:
            raise self.printer.command_error(
                "Current hotend temperature %.1f exceeds maximum allowed"
                " temperature %.1f" % (cur_temp, contact_probe.max_hotend_temp)
            )

    def trip_move_end(self, entry):
        reactor = self.printer.get_reactor()
        if self._heartbeat_timer is not None:
            reactor.unregister_timer(self._heartbeat_timer)
            self._heartbeat_timer = None
        mode = self._mode if self._mode is not None else MODE_PROXIMITY
        beacon = self.beacon
        if mode == MODE_PROXIMITY:
            beacon.beacon_stop_home_cmd.send([])
        else:
            beacon.beacon_contact_stop_home_cmd.send([])
        self._trsync_trigger_cmd.send([self.trsync_oid, REASON_HOST_REQUEST])
        deadline = reactor.monotonic() + TERMINAL_REASON_DEADLINE
        while self.last_reason is None:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "beacon: no terminal trsync_state received after homing"
                )
            reactor.pause(reactor.monotonic() + 0.010)
        if self.last_reason not in (REASON_ENDSTOP_HIT, REASON_HOST_REQUEST):
            raise self.printer.command_error(
                "beacon: trsync terminated with reason %d"
                % (self.last_reason,)
            )
```

Note for the `test_trip_move_end_forces_terminal_and_accepts_host_request` fake: `beacon_stop_home_cmd.send()` is called with no args in upstream beacon.py; the seam standardizes on `send([])` (FakeCommand records `[]`). When editing beacon.py later, the real `lookup_command(...).send()` accepts both.

- [ ] **Step 4: Run tests, expect PASS** (`python3 -m pytest test_beacon_kalico.py -v`)

- [ ] **Step 5: Commit (fork repo)**

```bash
git add beacon_kalico.py test_beacon_kalico.py
git commit -m "kalico seam: trsync arming, heartbeat deadman, terminal-reason validation"
```

---

### Task 4: `KalicoSeam` — provider hooks (`setup_bridge_endstop`, `measured_trip_position`)

**Files:**
- Modify: `beacon_kalico.py`
- Modify: `test_beacon_kalico.py`

- [ ] **Step 1: Write failing tests**

Append to `test_beacon_kalico.py`:

```python
def test_setup_bridge_endstop_validates_pin():
    seam, beacon, printer, mcu = make_seam()
    good = {"pin": "z_virtual_endstop", "invert": 0, "pullup": 0}
    assert seam.setup_bridge_endstop(good, 2) is seam.endstop
    import klippy.pins as pins_mod
    for bad, axis in (
        ({"pin": "nope", "invert": 0, "pullup": 0}, 2),
        ({"pin": "z_virtual_endstop", "invert": 1, "pullup": 0}, 2),
        ({"pin": "z_virtual_endstop", "invert": 0, "pullup": 0}, 0),
    ):
        try:
            seam.setup_bridge_endstop(bad, axis)
            assert False, "expected pins.error"
        except pins_mod.error:
            pass


def test_measured_trip_position_proximity_returns_sampled_dist():
    seam, beacon, printer, mcu = make_seam()
    seam.last_reason = beacon_kalico.REASON_ENDSTOP_HIT
    beacon._sample = lambda skip, count: (1.987, [{"pos": [0, 0, 2.0]}])
    assert seam.measured_trip_position(2, [0, 0, 2.0], [0, 0, 1.9]) == 1.987


def test_measured_trip_position_rejects_non_endstop_reason():
    seam, beacon, printer, mcu = make_seam()
    seam.last_reason = beacon_kalico.REASON_HOST_REQUEST
    try:
        seam.measured_trip_position(2, [0, 0, 2.0], [0, 0, 1.9])
        assert False, "expected command_error"
    except FakeCommandError:
        pass


def test_measured_trip_position_no_model_declines():
    seam, beacon, printer, mcu = make_seam()
    seam.last_reason = beacon_kalico.REASON_ENDSTOP_HIT
    beacon.model = None
    assert seam.measured_trip_position(2, [0, 0, 2.0], [0, 0, 1.9]) is None


def test_measured_trip_position_inf_dist_raises():
    seam, beacon, printer, mcu = make_seam()
    seam.last_reason = beacon_kalico.REASON_ENDSTOP_HIT
    beacon._sample = lambda skip, count: (float("inf"), [])
    try:
        seam.measured_trip_position(2, [0, 0, 2.0], [0, 0, 1.9])
        assert False, "expected command_error"
    except FakeCommandError:
        pass
```

- [ ] **Step 2: Run, expect FAIL**

- [ ] **Step 3: Implement**

Append to `KalicoSeam`:

```python
    def setup_bridge_endstop(self, pin_params, axis):
        if pin_params["pin"] != "z_virtual_endstop" or axis != Z_AXIS:
            raise pins.error(
                "beacon only provides z_virtual_endstop on the Z axis"
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error(
                "Can not pullup/invert beacon virtual endstop"
            )
        return self.endstop

    def measured_trip_position(self, axis, trip_pos, final_pos):
        if self.last_reason != REASON_ENDSTOP_HIT:
            raise self.printer.command_error(
                "beacon: homing completed with trsync reason %s, not"
                " endstop-hit" % (self.last_reason,)
            )
        beacon = self.beacon
        if beacon.model is None:
            return None
        dist, samples = beacon._sample(beacon.z_settling_time, 10)
        if math.isinf(dist):
            logging.error(
                "beacon post-homing adjustment measured samples %s", samples
            )
            raise self.printer.command_error(
                "Toolhead stopped below model range"
            )
        return dist
```

Add `import math` to the module imports.

- [ ] **Step 4: Run tests, expect PASS**

- [ ] **Step 5: Commit (fork repo)**

```bash
git add beacon_kalico.py test_beacon_kalico.py
git commit -m "kalico seam: provider hooks — endstop setup + measured trip position"
```

---

### Task 5: `KalicoSeam` — motion-history queries and descend helpers

**Files:**
- Modify: `beacon_kalico.py`
- Modify: `test_beacon_kalico.py`

- [ ] **Step 1: Write failing tests**

Append to `test_beacon_kalico.py`:

```python
class FakeBridge:
    def __init__(self):
        self.state = {"x": (1.0, 0.0, 0.0), "y": (2.0, 0.0, 0.0),
                      "z": (3.0, -5.0, 0.0)}
        self.errors = []
        self.calls = []

    def motion_state_at(self, mcu, clock=None, print_time=None):
        self.calls.append(clock)
        if self.errors:
            raise RuntimeError(self.errors.pop(0))
        return self.state


def make_seam_with_bridge():
    seam, beacon, printer, mcu = make_seam()
    bridge = FakeBridge()
    printer.objects["motion_bridge"] = bridge
    return seam, beacon, printer, mcu, bridge


def test_position_at_clock_returns_xyz():
    seam, beacon, printer, mcu, bridge = make_seam_with_bridge()
    assert seam.position_at_clock(1234) == [1.0, 2.0, 3.0]


def test_position_at_clock_no_history_returns_none():
    seam, beacon, printer, mcu, bridge = make_seam_with_bridge()
    bridge.errors = ["no motion history recorded for axis ..."]
    assert seam.position_at_clock(1234) is None


def test_position_at_clock_before_window_drops_and_counts():
    seam, beacon, printer, mcu, bridge = make_seam_with_bridge()
    bridge.errors = ["query clock 1 precedes retained motion history ..."]
    assert seam.position_at_clock(1234) is None
    assert seam._dropped_samples == 1


def test_position_at_clock_future_retries_once_then_raises():
    seam, beacon, printer, mcu, bridge = make_seam_with_bridge()
    bridge.errors = ["query clock 9 is in the future for axis ..."]
    assert seam.position_at_clock(1234) == [1.0, 2.0, 3.0]
    assert len(bridge.calls) == 2
    assert printer.reactor.paused != []
    bridge.errors = [
        "query clock 9 is in the future for axis ...",
        "query clock 9 is in the future for axis ...",
    ]
    try:
        seam.position_at_clock(1234)
        assert False, "expected RuntimeError"
    except RuntimeError:
        pass


def test_position_at_clock_unknown_error_propagates():
    seam, beacon, printer, mcu, bridge = make_seam_with_bridge()
    bridge.errors = ["motion_state_at: no axes configured on the bridge"]
    try:
        seam.position_at_clock(1234)
        assert False, "expected RuntimeError"
    except RuntimeError:
        pass


def test_cruise_check():
    assert beacon_kalico.is_cruise_acceleration(0.0)
    assert beacon_kalico.is_cruise_acceleration(0.5)
    assert not beacon_kalico.is_cruise_acceleration(50.0)
    assert not beacon_kalico.is_cruise_acceleration(-50.0)
```

- [ ] **Step 2: Run, expect FAIL**

- [ ] **Step 3: Implement queries and descents**

Append to `beacon_kalico.py` (module level):

```python
def is_cruise_acceleration(accel):
    return abs(accel) <= CRUISE_ACCEL_TOLERANCE
```

Append to `KalicoSeam`:

```python
    def _bridge(self):
        return self.printer.lookup_object("motion_bridge")

    def position_at_clock(self, clock64):
        state = self._motion_state(int(clock64))
        if state is None:
            return None
        try:
            return [state["x"][0], state["y"][0], state["z"][0]]
        except KeyError:
            return None

    def position_at_clock32(self, clock32):
        return self.position_at_clock(self.mcu.clock32_to_clock64(clock32))

    def _motion_state(self, clock64, retried=False):
        try:
            return self._bridge().motion_state_at(self.mcu, clock=clock64)
        except RuntimeError as e:
            kind = classify_history_error(str(e))
            if kind == ERR_NO_HISTORY:
                return None
            if kind == ERR_BEFORE_WINDOW:
                self._dropped_samples += 1
                if self._dropped_samples == 1:
                    logging.warning(
                        "beacon: dropping stream sample older than retained"
                        " motion history: %s", e
                    )
                return None
            if kind == ERR_FUTURE and not retried:
                reactor = self.printer.get_reactor()
                reactor.pause(reactor.monotonic() + FUTURE_RETRY_PAUSE)
                return self._motion_state(clock64, retried=True)
            raise

    def proximity_descend(self, gcmd, bottom_z, speed):
        self._descend(gcmd, MODE_PROXIMITY, bottom_z, speed)

    def contact_descend(self, gcmd, bottom_z, speed):
        trip_pos, final_pos = self._descend(
            gcmd, MODE_CONTACT, bottom_z, speed
        )
        detect_clock = self._query_detect_clock()
        state = self._bridge().motion_state_at(
            self.mcu, clock=self.mcu.clock32_to_clock64(detect_clock)
        )
        z_pos, z_vel, z_accel = state["z"]
        if not is_cruise_acceleration(z_accel):
            raise self.printer.command_error(
                "beacon: contact triggered while %s (z accel %.3f mm/s^2)"
                % ("decelerating" if z_accel * z_vel > 0 else "accelerating",
                   z_accel)
            )
        return [final_pos[0], final_pos[1], z_pos]

    def _descend(self, gcmd, mode, bottom_z, speed):
        printer = self.printer
        toolhead = printer.lookup_object("toolhead")
        homing_obj = printer.lookup_object("homing")
        bridge = printer.lookup_object("motion_bridge")
        if gcmd is None:
            gcode = printer.lookup_object("gcode")
            gcmd = gcode.create_gcode_command(
                "BEACON_DESCEND", "BEACON_DESCEND", {}
            )
        start_z = toolhead.get_position()[Z_AXIS]
        max_travel = start_z - bottom_z
        if max_travel <= 0.0:
            raise printer.command_error(
                "beacon: descend target %.3f is not below current Z %.3f"
                % (bottom_z, start_z)
            )
        self._mode = mode
        try:
            trip_pos, final_pos = homing_obj.trip_move(
                gcmd,
                toolhead,
                bridge,
                Z_AXIS,
                -1.0,
                speed,
                max_travel,
                {
                    "endstop": self.endstop,
                    "provider": self.beacon,
                    "trigger_height": None,
                },
            )
        finally:
            self._mode = None
        if self.last_reason != REASON_ENDSTOP_HIT:
            raise printer.command_error(
                "beacon: descend completed with trsync reason %s, not"
                " endstop-hit" % (self.last_reason,)
            )
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = final_pos[Z_AXIS]
        toolhead.set_position(newpos)
        return trip_pos, final_pos

    def _query_detect_clock(self):
        beacon = self.beacon
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + 0.5
        while True:
            ret = beacon.beacon_contact_query_cmd.send([])
            if ret["triggered"]:
                return ret["detect_clock"]
            now = reactor.monotonic()
            if now >= deadline:
                raise self.printer.command_error(
                    "Timeout getting contact time"
                )
            reactor.pause(now + 0.001)
```

The trip-clock state in `_mode` is read by `trip_move_begin`/`trip_move_end` (called by `trip_move` between the set and the `finally` reset) — that is the whole mode-dispatch mechanism. G28 never sets `_mode`, so provider hooks default to proximity, which is correct: contact G28 goes through `BEACON_AUTO_CALIBRATE`, never through `_home_axis`.

- [ ] **Step 4: Run tests, expect PASS** (`python3 -m pytest test_beacon_kalico.py -v`)

- [ ] **Step 5: Commit (fork repo)**

```bash
git add beacon_kalico.py test_beacon_kalico.py
git commit -m "kalico seam: motion-history sample queries + trip_move descend helpers"
```

---

### Task 6: beacon.py surgery — imports, init, streaming seam

**Files:**
- Modify: `beacon.py`

No unit tests here (this is integration wiring); the sim connect test in Task 9 is the verification. Make each numbered edit exactly.

- [ ] **Step 1: Fix the import block** (beacon.py lines 10-35)

Delete: `import chelper`, `from .homing import HomingMove`.
Replace bare klippy imports with package-style and import the seam:

```python
from klippy import pins
from klippy.mcu import MCU
from klippy.clocksync import SecondarySync
from klippy import configfile
from klippy import msgproto
from . import beacon_kalico
```

(`import pins`, `from mcu import MCU, MCU_trsync`, `from clocksync import SecondarySync`, `import configfile`, `import msgproto` all go away. `MCU_trsync` has no replacement import — nothing uses it after this task.)

Also delete the now-unused `TRSYNC_TIMEOUT_DEFAULT = 0.025` constant and the `trsync_timeout` config read (init, ~line 88-90) — the seam owns the deadman window.

- [ ] **Step 2: Replace the endstop objects in `BeaconProbe.__init__`** (~lines 159-162)

Old:

```python
        self._endstop_shared = BeaconEndstopShared(self)
        self.mcu_probe = BeaconEndstopWrapper(self)
        self.mcu_contact_probe = BeaconContactEndstopWrapper(self, config)
        self._current_probe = "proximity"
```

New:

```python
        self.kalico_seam = beacon_kalico.KalicoSeam(self)
        self.mcu_contact_probe = BeaconContactProbe(self, config)
        self._current_probe = "proximity"
```

- [ ] **Step 3: Route the provider contract through BeaconProbe**

Beacon registers itself as the `probe` pins chip (line 180), so homing.py finds the hooks on `BeaconProbe`. Add these methods next to `setup_pin` (~line 457):

```python
    def setup_bridge_endstop(self, pin_params, axis):
        return self.kalico_seam.setup_bridge_endstop(pin_params, axis)

    def get_position_endstop(self):
        return self.trigger_distance

    def trip_move_begin(self, entry):
        self.kalico_seam.trip_move_begin(entry)

    def trip_move_end(self, entry):
        self.kalico_seam.trip_move_end(entry)

    def measured_trip_position(self, axis, trip_pos, final_pos):
        return self.kalico_seam.measured_trip_position(
            axis, trip_pos, final_pos
        )
```

And make `setup_pin` fail loudly (the kalico homing path never calls it; anything else reaching it is unported):

```python
    def setup_pin(self, pin_type, pin_params):
        raise pins.error(
            "beacon on kalico resolves probe:z_virtual_endstop via the"
            " homing provider contract; setup_pin has no users"
        )
```

- [ ] **Step 4: Fix the `query_endstops` registration** (~line 94-95)

The registered object must be the seam's endstop (it has `query_endstop`):

```python
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self.kalico_seam.endstop, "probe")
```

Note ordering: this code runs in `__init__` *before* Step 2 creates the seam — move the `query_endstops` registration to after the seam exists, or create the seam earlier (right after `self.cmd_queue = self._mcu.alloc_command_queue()`). Creating the seam early is correct: it only needs `printer` and `_mcu`.

- [ ] **Step 5: Replace the trapq readiness guard**

Init (~line 143): `self.trapq = None` → `self.kalico_ready = False`.
`_build_config` (~line 402): `self.trapq = self.toolhead.get_trapq()` → `self.kalico_ready = True`.
`_handle_beacon_data` (~line 1005): `if self.trapq is None:` → `if not self.kalico_ready:`.

- [ ] **Step 6: Replace `_get_position_at_time`**

Delete the method (lines 1032-1040). In `_stream_flush_message` (~line 944), the sample already has `clock` as a 64-bit value (line 938). Replace:

```python
            pos = self._get_position_at_time(time)
```

with:

```python
            pos = self.kalico_seam.position_at_clock(clock)
```

In `cmd_BEACON_POKE` (~line 1450), replace:

```python
                    armpos = self._get_position_at_time(
                        self._clock32_to_time(self.last_contact_msg["armed_clock"])
                    )
```

with:

```python
                    armpos = self.kalico_seam.position_at_clock32(
                        self.last_contact_msg["armed_clock"]
                    )
```

- [ ] **Step 7: Sanity check — module parses**

```bash
python3 -m py_compile beacon.py beacon_kalico.py
```

Expected: silence (exit 0). (`from klippy import ...` resolves at runtime inside klippy, not here — py_compile only parses.)

- [ ] **Step 8: Commit (fork repo)**

```bash
git add beacon.py
git commit -m "kalico: package imports, seam wiring, motion_state_at streaming backbone"
```

---

### Task 7: beacon.py surgery — descents and dead-class deletion

**Files:**
- Modify: `beacon.py`

- [ ] **Step 1: Replace the proximity probing dive** (`_probing_move_to_probing_height`, ~line 523)

```python
    def _probing_move_to_probing_height(self, speed):
        curtime = self.reactor.monotonic()
        status = self.kinematics.get_status(curtime)
        self.kalico_seam.proximity_descend(
            None, status["axis_minimum"][2], speed
        )
```

(The old `phoming.probing_move` / `HINT_TIMEOUT` rewrapping goes away; `trip_move`'s own errors name the failure.) `self.phoming` (assigned in `_handle_connect`, line 267) loses its last consumer — delete the assignment too.

- [ ] **Step 2: Replace the contact descent** (`_probe_contact`, ~line 618)

```python
    def _probe_contact(self, speed):
        self.toolhead.get_last_move_time()
        self._sample_async()
        epos = self.kalico_seam.contact_descend(None, -2.0, speed)
        epos[2] += self.get_z_compensation_value(epos)
        self.gcode.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (epos[0], epos[1], epos[2])
        )
        return epos
```

(The `printer.is_shutdown()` rewrapping is dropped — a shutdown surfaces its own error.)

- [ ] **Step 3: Replace the poke descent** (`cmd_BEACON_POKE`, ~lines 1443-1447)

Old:

```python
                    hmove = HomingMove(
                        self.printer, [(self.mcu_contact_probe, "contact")]
                    )
                    pos[2] = bottom
                    epos = hmove.homing_move(pos, speed, probe_pos=True)[:3]
```

New:

```python
                    epos = self.kalico_seam.contact_descend(
                        gcmd, bottom, speed
                    )
```

- [ ] **Step 4: Replace the auto-calibrate descent** (`cmd_BEACON_AUTO_CALIBRATE`, ~lines 1538-1548)

Old:

```python
                try:
                    hmove = HomingMove(
                        self.printer, [(self.mcu_contact_probe, "contact")]
                    )
                    epos = hmove.homing_move(home_pos, speed, probe_pos=True)
                except self.printer.command_error:
                    if self.printer.is_shutdown():
                        raise self.printer.command_error(
                            "Homing failed due to printer shutdown"
                        )
                    raise
                finally:
                    set_max_accel(old_max_accel)
```

New:

```python
                try:
                    epos = self.kalico_seam.contact_descend(
                        gcmd, home_pos[2], speed
                    )
                finally:
                    set_max_accel(old_max_accel)
```

- [ ] **Step 5: Slim the contact-probe class and delete the dead classes**

Replace `BeaconContactEndstopWrapper` (lines 2329-2431) with a config-holder only — it keeps its three real jobs (activate/deactivate gcode templates, hotend ceiling) and loses the endstop contract:

```python
class BeaconContactProbe:
    def __init__(self, beacon, config):
        gcode_macro = beacon.printer.load_object(config, "gcode_macro")
        self.activate_gcode = gcode_macro.load_template(
            config, "contact_activate_gcode", ""
        )
        self.deactivate_gcode = gcode_macro.load_template(
            config, "contact_deactivate_gcode", ""
        )
        self.max_hotend_temp = config.getfloat(
            "contact_max_hotend_temperature", 180.0
        )
```

Delete entirely:
- `BeaconEndstopShared` (lines 2167-2237)
- `BeaconEndstopWrapper` (lines 2239-2327) — its post-home `_sample` adjustment now lives in `KalicoSeam.measured_trip_position`; its `homing:home_rails_begin/end` **listener** registrations die with it.

Keep: `BeaconHomingState` (lines 2626-2638) and the two `send_event("homing:home_rails_*")` calls in auto-calibrate (lines 1521-1522, 1593) — beacon *sending* these events feeds our live listeners (gcode_move, z_thermal_adjust). This is the documented spec deviation.

Keep `BeaconHomingHelper` untouched — its G28 wrap (`register_command("G28", None)` chaining) works against our homing.py's G28.

- [ ] **Step 6: Sweep for stragglers**

```bash
grep -n "HomingMove\|MCU_trsync\|trdispatch\|chelper\|trapq\|_endstop_shared\|mcu_probe\b\|BeaconEndstopWrapper\|BeaconEndstopShared\|BeaconContactEndstopWrapper\|phoming" beacon.py
```

Expected hits: only `self.trapq`-free code (zero trapq hits), `mcu_contact_probe` (the slim class), and comments. Anything else is an unported call site — port it the same way before proceeding (e.g. `query_endstop`-style uses of the deleted wrappers). Known one to check: BeaconMeshHelper / scanning code paths around lines 2640-3335 reference `beacon._sample`-family only, but verify the grep is clean.

Then re-run unit tests + parse check:

```bash
python3 -m pytest test_beacon_kalico.py -v && python3 -m py_compile beacon.py
```

- [ ] **Step 7: Update `install.sh` to link both files**

Find the line symlinking `beacon.py` into `klippy/extras/` and add the same for `beacon_kalico.py`.

- [ ] **Step 8: Commit (fork repo)**

```bash
git add beacon.py install.sh
git commit -m "kalico: descents via trip_move; delete trsync/trdispatch endstop ceremony"
```

---

### Task 8: Emulator — emit `beacon_contact` telemetry

The poke path reads `last_contact_msg["armed_clock"]/["latency"]/["error"]`, populated by the device's `beacon_contact` message. The emulator never sends it.

**Files:**
- Modify: `tools/kalico-sim/emulators/beacon_mcu.py`

- [ ] **Step 1: Capture the armed clock in `_handle_beacon_contact_home`** (line 442)

Add at the top of the handler:

```python
        armed_clock = self._now_clock()
```

and inside `_fire()` after `self._contact_trigger_clock = self._now_clock()`:

```python
            self._send_msg(
                "beacon_contact armed_clock=%u trigger_clock=%u"
                " detect_clock=%u latency=%c error=%c",
                armed_clock=armed_clock,
                trigger_clock=self._contact_trigger_clock,
                detect_clock=self._contact_trigger_clock,
                latency=0,
                error=0,
            )
```

If `self._parser.lookup_command` rejects the format (message absent from `IDENTIFY_BLOB`), the `_send_msg` helper already logs-and-drops — in that case check `beacon_identify_dict.py` for the actual field layout of `beacon_contact` and match it exactly; the message exists in the real device dictionary (beacon.py registers a handler for it at line 195-198).

- [ ] **Step 2: Verify the emulator still imports**

```bash
python3 -c "import sys; sys.path.insert(0, 'tools/kalico-sim/emulators'); sys.path.insert(0, '.'); import importlib.util; spec = importlib.util.spec_from_file_location('beacon_mcu', 'tools/kalico-sim/emulators/beacon_mcu.py'); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); print('ok')"
```

Expected: `ok`.

- [ ] **Step 3: Commit (kalico repo)**

```bash
git add tools/kalico-sim/emulators/beacon_mcu.py
git commit -m "sim: beacon emulator emits beacon_contact telemetry on contact trigger"
```

---

### Task 9: Runner — `--beacon-test` variants + connect smoke test

**Files:**
- Modify: `tools/kalico-sim/runner.py`

- [ ] **Step 1: Add the CLI surface**

Next to `--homing-test` (line 2381):

```python
    parser.add_argument(
        "--beacon-test",
        choices=BEACON_TEST_VARIANTS,
        help="Run one beacon fork validation variant (implies the beacon"
        " stub + beacon config)",
    )
```

and pass `beacon_test=args.beacon_test` through `run_simulation(...)` (line 2444), threading it exactly like the existing `homing_test` parameter (signature ~line 1031, beacon stub + config activation; reuse the same `_generate_beacon_homing_config` path — read it before editing and extend rather than fork it).

Define near `PROBE_TEST_VARIANTS` (line 1300):

```python
BEACON_TEST_VARIANTS = (
    "connect",
    "contact",
    "probe",
    "poke",
    "mesh",
    "accel",
)
```

- [ ] **Step 2: Implement the variant scripts**

Add a dispatch in `run_simulation` where `homing_test` is handled (line 575). Each variant follows the same skeleton as the homing test (send gcode → collect responses → fail on response error / log `shutdown:` / timeout). Scripts:

| Variant | G-code sequence | Pass criteria |
|---|---|---|
| `connect` | `G4 P2000` | klippy ready, no shutdown, klippy.log contains no `Traceback` and no `beacon` config error |
| `contact` | `SET_KINEMATIC_POSITION X=150 Y=150 Z=10`, `G4 P1000`, `BEACON_AUTO_CALIBRATE` | command succeeds; log contains `Collected` (sample collection ran) |
| `probe` | `SET_KINEMATIC_POSITION X=150 Y=150 Z=100`, `G4 P1000`, `G28 Z`, `PROBE PROBE_METHOD=proximity SAMPLES=2`, `PROBE PROBE_METHOD=contact SAMPLES=1`, `PROBE_ACCURACY SAMPLES=3` | all commands succeed |
| `poke` | `SET_KINEMATIC_POSITION X=150 Y=150 Z=10`, `G4 P1000`, `BEACON_POKE TOP=5 BOTTOM=-0.3` | command succeeds; response includes `Triggered at:` and `Armed at:` (proves `position_at_clock32` + `beacon_contact` telemetry) |
| `mesh` | `SET_KINEMATIC_POSITION X=150 Y=150 Z=100`, `G4 P1000`, `G28 Z`, `BED_MESH_CALIBRATE` | succeeds; requires `[bed_mesh]` in the generated config for this variant (add a minimal section: `mesh_min: 20,20`, `mesh_max: 280,280`, `probe_count: 3,3`, plus beacon scanning defaults) |
| `accel` | `ACCELEROMETER_MEASURE CHIP=beacon`, `G4 P1000`, `ACCELEROMETER_MEASURE CHIP=beacon NAME=test` | succeeds; no shutdown. If the emulator's identify blob lacks `BEACON_HAS_ACCEL=1` constant, beacon skips accel setup — then assert the command fails with "no accelerometer" and mark the variant as exercising the graceful path; note it in the task summary |

Streaming-with-positions is asserted by `poke` (its CSV/armed-position math runs `position_at_clock` on live stream samples during motion). Temp-comp NVM load is exercised by every variant at connect (`_build_config` → `beacon_nvm_read`).

- [ ] **Step 3: Run the connect variant — this is the project's first green light**

```bash
tools/kalico-sim/run.sh --beacon-test connect
```

Expected: PASS. Debug loop: read `klippy.log` from the sim output dir; every remaining `ImportError`/`AttributeError` in beacon.py is an unported call site — fix in the fork clone, re-run (docker `COPY`s the tree, so just re-running rebuilds with the edit).

- [ ] **Step 4: Commit (kalico repo)**

```bash
git add tools/kalico-sim/runner.py
git commit -m "sim: --beacon-test variants for the beacon fork validation matrix"
```

---

### Task 10: Proximity G28 green (the original red baseline)

- [ ] **Step 1: Run**

```bash
tools/kalico-sim/run.sh --homing-test
```

Expected: PASS. This exercises: provider resolution (`setup_bridge_endstop` on the chip), proximity arming (`beacon_home` + threshold trigger in the emulator), the Rust relay, `trip_move`, and `measured_trip_position`'s post-home `_sample()` override.

Debugging notes if red:
- `G28: axis Z has no endstop` → the sim config's `[stepper_z] endstop_pin` isn't `probe:z_virtual_endstop` — check `_generate_beacon_homing_config` (runner.py:1192+).
- `must not set position_endstop` → remove `position_endstop` from the generated `[stepper_z]` (the provider supplies it now).
- Trip fires but homed Z wrong → compare `measured_trip_position`'s returned dist against the emulator's `_z_to_frequency` model and the config's saved beacon model coefficients (runner.py:1282).

- [ ] **Step 2: No commit unless fixes were needed** (commit any with `fix:` prefixes in the respective repo).

---

### Task 11: Contact, probe, poke variants green

- [ ] **Step 1: Contact**

```bash
tools/kalico-sim/run.sh --beacon-test contact
```

Covers: `_mode` dispatch, `beacon_contact_home` arming, contact trigger relay, `_query_detect_clock`, cruise-phase validation via `motion_state_at` accel, autocal sampling loop, the self-sent `homing:home_rails_*` events, model creation (`_calibrate` streams during a controlled descent).

Likely first failure: cruise check — if the emulator's 0.5 s contact timer fires while Z is still in the acceleration ramp, either lengthen the descent (AUTO_CALIBRATE from `Z=10` at `autocal_speed` 3 mm/s gives ≥3 s of cruise; 0.5 s lands in ramp only if accel is tiny) or set the stub's `_homing_trigger_delay` to 1.0 in `_start_beacon`. Tune the emulator, not the tolerance.

- [ ] **Step 2: Probe**

```bash
tools/kalico-sim/run.sh --beacon-test probe
```

Covers: `BeaconProbeWrapper.run_probe` both methods, `_probing_move_to_probing_height` (proximity descend), `_probe_contact` (contact descend), PROBE_ACCURACY.

- [ ] **Step 3: Poke**

```bash
tools/kalico-sim/run.sh --beacon-test poke
```

Covers: streaming session during motion, `position_at_clock` on live samples, `position_at_clock32(armed_clock)`, emulator `beacon_contact` telemetry.

- [ ] **Step 4: Commit fixes** as they land (fork repo for seam fixes, kalico repo for harness fixes).

---

### Task 12: Mesh and accel variants green

- [ ] **Step 1: Mesh**

```bash
tools/kalico-sim/run.sh --beacon-test mesh
```

Covers: `BeaconMeshHelper` scanning path (streamed samples → positions via `position_at_clock` while the toolhead sweeps), bed_mesh integration, `get_offsets` consumption.

- [ ] **Step 2: Accel**

```bash
tools/kalico-sim/run.sh --beacon-test accel
```

Per the Task 9 table: green either as a real accel session (if the identify blob advertises `BEACON_HAS_ACCEL`) or as the asserted graceful no-accelerometer error.

- [ ] **Step 3: Interface-conformance unit test (fork repo)**

Append to `test_beacon_kalico.py` — pins the probe-interface surface our `ProbePointsHelper` consumers require (z_tilt/QGL/screws_tilt can't run in the single-Z sim, so the interface is the testable contract):

```python
def test_probe_wrapper_presents_points_helper_interface():
    import ast

    tree = ast.parse(open("beacon.py").read())
    wrapper = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.ClassDef) and n.name == "BeaconProbeWrapper"
    )
    methods = {n.name for n in wrapper.body if isinstance(n, ast.FunctionDef)}
    required = {
        "run_probe",
        "get_offsets",
        "get_lift_speed",
        "multi_probe_begin",
        "multi_probe_end",
    }
    assert required <= methods
```

Run: `python3 -m pytest test_beacon_kalico.py -v` — PASS.

- [ ] **Step 4: Commit (fork repo)**

```bash
git add test_beacon_kalico.py
git commit -m "test: pin ProbePointsHelper-facing wrapper interface"
```

---

### Task 13: Full matrix, lint, wrap-up

- [ ] **Step 1: Run every variant back-to-back**

```bash
for v in connect contact probe poke mesh accel; do
  tools/kalico-sim/run.sh --beacon-test "$v" || { echo "FAILED: $v"; break; }
done
tools/kalico-sim/run.sh --homing-test
```

Expected: all PASS.

- [ ] **Step 2: Kalico-repo checks**

```bash
ruff check tools/kalico-sim/ && ruff format --check tools/kalico-sim/
cd rust && cargo nextest run -p motion-bridge && cd ..
```

(No Rust changes are expected; the nextest run is a regression guard since the seam leans on `motion_state_at` and the remote-trigger relay.)

- [ ] **Step 3: Fork unit tests one last time**

```bash
cd tools/sim_klippy/printer_real/third_party_repos/beacon_klipper
python3 -m pytest test_beacon_kalico.py -v
```

- [ ] **Step 4: Final commits and status**

- Fork repo: ensure working tree clean, history reads as the task sequence above. Do not push without checking with Danila (master of a shared fork).
- Kalico repo (`beacon-support` branch): emulator + runner commits from Tasks 8-9, plus a docs touch-up: add a one-line pointer in `docs/kalico-rewrite/beacon-fork-survey.md` under the Decision section — `Implemented: see docs/superpowers/specs/2026-06-12-beacon-fork-seam-design.md and dderg/beacon_klipper@<sha>`.
- Report which variants are green, any emulator capabilities that were stubbed around (accel constant, contact timer), and the remaining bench-validation step (interactive, motion commands individually approved — out of scope for this plan).

---

## Self-review notes

- **Spec coverage:** repo layout/merge strategy → Task 1; seam 1 (homing/endstop, both modes, single trsync, heartbeat, measured position) → Tasks 3-5 + 10-11; seam 2 (streaming, connect flag, error table) → Tasks 5-6; seam 3 (probe wrapper kept, listeners deleted, self-sent events kept) → Tasks 7, 12; validation matrix rows 1-10 → connect (1, 10), homing-test (2), contact (3, 8), probe (4 partially — PPH consumers via interface test in Task 12), poke (5, 7), mesh (6), accel (9). Contract gaps: none planned; any discovered become separate kalico PRs per spec.
- **Known judgment calls encoded above:** `BeaconHomingState` kept (spec deviation, documented); trsync deadman relaxed to 200 ms (beacon-side deadman is not the motion deadman); `trip_move_end` accepts HOST_REQUEST (unwind path) with ENDSTOP_HIT enforced on success paths only; cruise validation by `|accel| ≤ 1 mm/s²` threshold replacing upstream's sign-of-constant-accel test.
