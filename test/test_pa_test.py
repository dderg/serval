import math
import types

import pytest

from klippy.extras.pa_test import PA_TOWER_FILENAME, PATest


class StubConfig:
    def __init__(self, printer, options=None):
        self.printer = printer
        self.options = options or {}
        self.error = RuntimeError

    def get_printer(self):
        return self.printer

    def getfloat(self, key, default=None, **kwargs):
        value = self.options.get(key, default)
        return None if value is None else float(value)

    def getint(self, key, default=None, **kwargs):
        return int(self.options.get(key, default))


class StubGcode:
    def __init__(self):
        self.commands = {}
        self.scripts = []

    def register_command(self, name, callback, desc=None):
        self.commands[name] = callback

    def run_script_from_command(self, script):
        self.scripts.append(script)


class CommandError(Exception):
    pass


class StubGcmd:
    error = CommandError

    def __init__(self, params):
        self.params = params

    def get(self, key, default="!missing"):
        value = self.params.get(key, default)
        if value == "!missing":
            raise CommandError("missing '%s'" % key)
        return value

    def get_float(self, key, default="!missing", **kwargs):
        value = self.get(key, default)
        return None if value is None else float(value)

    def get_int(self, key, default="!missing", **kwargs):
        return int(self.get(key, default))


class StubHeater:
    def get_status(self, eventtime):
        return {"target": 210.0}


class StubExtruder:
    def get_heater(self):
        return StubHeater()


class StubKin:
    def get_status(self, eventtime=None):
        coord = types.SimpleNamespace
        return {
            "axis_minimum": coord(x=0.0, y=0.0, z=0.0),
            "axis_maximum": coord(x=250.0, y=220.0, z=250.0),
        }


class StubToolhead:
    def get_extruder(self):
        return StubExtruder()

    def get_kinematics(self):
        return StubKin()

    def get_status(self, eventtime):
        return {"square_corner_velocity": 5.0}


class StubReactor:
    def monotonic(self):
        return 0.0


class StubPrinter:
    def __init__(self, objects):
        self.objects = objects
        self.config_error = RuntimeError

    def register_event_handler(self, event, callback):
        pass

    def lookup_object(self, name, default="!missing"):
        if name in self.objects:
            return self.objects[name]
        if default == "!missing":
            raise KeyError(name)
        return default

    def get_reactor(self):
        return StubReactor()


def make(sdcard=None, options=None):
    gcode = StubGcode()
    objects = {"gcode": gcode, "toolhead": StubToolhead()}
    if sdcard is not None:
        objects["virtual_sdcard"] = sdcard
    printer = StubPrinter(objects)
    obj = PATest(StubConfig(printer, options))
    obj._connect()
    return obj, gcode


class StubSdcard:
    def __init__(self, dirname):
        self.sdcard_dirname = str(dirname)
        self.active = False

    def is_active(self):
        return self.active


BASE_PARAMS = {"NOZZLE": "0.4", "TARGET_TEMP": "210", "HEIGHT": "2.0"}


def generate(obj, extra=None):
    params = dict(BASE_PARAMS)
    params.update(extra or {})
    return list(obj.generate_gcode(StubGcmd(params)))


def parse_moves(lines):
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    moves = []
    for line in lines:
        if not line.startswith("G1 "):
            continue
        words = dict(
            (w[0], float(w[1:])) for w in line.split()[1:] if w[0] != "E"
        )
        e = next((float(w[1:]) for w in line.split()[1:] if w[0] == "E"), None)
        prev = dict(pos)
        pos.update((k, v) for k, v in words.items() if k in pos)
        dist = math.dist(
            (prev["X"], prev["Y"], prev["Z"]), (pos["X"], pos["Y"], pos["Z"])
        )
        moves.append((dict(pos), e, dist, words.get("F")))
    return moves


def test_tower_gcode_is_wellformed():
    sd = StubSdcard("/tmp")
    obj, _ = make(sdcard=sd)
    lines = generate(obj, {"FINAL_GCODE_ID": "end_pa_test"})
    moves = parse_moves(lines)
    assert moves, "no moves generated"
    zs = [m[0]["Z"] for m in moves]
    assert all(b >= a for a, b in zip(zs, zs[1:])), "Z must be monotonic"
    for pos, e, dist, feed in moves:
        assert 88.0 <= pos["X"] <= 162.0
        assert 68.0 <= pos["Y"] <= 152.0
        if e is not None:
            assert 0.0 < e < dist
    feeds = {feed for _, _, _, feed in moves if feed is not None}
    expected = {v * 60.0 for v in (25.0, 50.0, 80.0, 5.0)}
    assert feeds <= expected


def test_extrusion_tracks_path_length():
    sd = StubSdcard("/tmp")
    obj, _ = make(sdcard=sd)
    moves = parse_moves(generate(obj))
    ratios = sorted(e / dist for _, e, dist, _ in moves if e and dist > 0.01)
    clusters = []
    for r in ratios:
        if not clusters or r > clusters[-1] * 1.02:
            clusters.append(r)
    assert len(clusters) == 2, "one extrusion ratio per width: %s" % clusters


def test_temp_mismatch_fails_loudly():
    sd = StubSdcard("/tmp")
    obj, _ = make(sdcard=sd)
    with pytest.raises(CommandError, match="target temp"):
        generate(obj, {"TARGET_TEMP": "250"})


def test_print_pa_tower_writes_file_and_starts_print(tmp_path):
    sd = StubSdcard(tmp_path)
    obj, gcode = make(sdcard=sd)
    params = dict(BASE_PARAMS)
    gcode.commands["PRINT_PA_TOWER"](StubGcmd(params))
    written = (tmp_path / PA_TOWER_FILENAME).read_text().splitlines()
    assert written[0] == "M83"
    assert gcode.scripts == [
        "SDCARD_PRINT_FILE FILENAME=%s" % (PA_TOWER_FILENAME,)
    ]


def test_busy_sdcard_rejects_new_tower(tmp_path):
    sd = StubSdcard(tmp_path)
    sd.active = True
    obj, gcode = make(sdcard=sd)
    with pytest.raises(CommandError, match="already running"):
        gcode.commands["PRINT_PA_TOWER"](StubGcmd(dict(BASE_PARAMS)))
    assert not (tmp_path / PA_TOWER_FILENAME).exists()
