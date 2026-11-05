"""Hardware validation probes for the simulated peripherals (capture-fw/PHASE2.md).

Each subcommand holds the bench lock and prints one JSON document that a
validation step in PHASE2.md judges. The printer must run the config the
probe is given (push it with `deploy.py config --config configs/full.cfg`).

  query OBJECT...      klippy printer objects (temperatures, tmc2209 state)
  gcode SCRIPT         run a script, print the console responses it produced
  endstops             QUERY_ENDSTOPS as klippy sees the simulated pins
  capture              sim capture around a gcode sequence + simulator report
  heat                 heat one heater, compare klippy's reading with the model
"""

from __future__ import annotations

import argparse
import json
import secrets
import shutil
import time
import urllib.parse
from pathlib import Path

import analyze
import lock
import pi
import sim_config
import sim_report
from capture import Capture

BENCH = Path(__file__).resolve().parent
DEFAULT_SIM = BENCH / "configs" / "full.sim.json"
DEFAULT_CONFIG = BENCH / "configs" / "full.cfg"
HEATER_OBJECTS = {"extruder": "extruder", "bed": "heater_bed"}
HEATER_COMMANDS = {"extruder": "M104", "bed": "M140"}
POLL_S = 0.5
PLATEAU_S = 10.0
GCODE_STORE_COUNT = 200


class ProbeError(RuntimeError):
    pass


def query(objects: list[str]) -> dict:
    path = "/printer/objects/query?" + "&".join(
        urllib.parse.quote(o) for o in objects
    )
    return pi.http("GET", path)["status"]


def _gcode_store() -> list[dict]:
    return pi.http("GET", f"/server/gcode_store?count={GCODE_STORE_COUNT}")[
        "gcode_store"
    ]


def gcode(script: str) -> list[str]:
    seen = max((entry["time"] for entry in _gcode_store()), default=0.0)
    pi.gcode_script(script)
    return [
        entry["message"]
        for entry in _gcode_store()
        if entry["time"] > seen and entry["type"] == "response"
    ]


def endstops() -> dict:
    pi.gcode_script("QUERY_ENDSTOPS")
    return pi.http("GET", "/printer/query_endstops/status")


def _parse_xyz(text: str) -> tuple[float, float, float]:
    x, y, z = (float(v) for v in text.split(","))
    return x, y, z


def _sim(args) -> sim_config.SimConfig:
    motors = analyze.read_motors(
        analyze._config_sections(args.config.read_text())
    )
    return sim_config.load(
        args.sim,
        config_id=secrets.randbelow(0xFFFF) + 1,
        step_mm={m.role: m.step_mm for m in motors},
        dir_sign={m.role: m.dir_sign for m in motors},
        start_xyz=args.start,
    )


def _out_dir(args, name: str) -> Path:
    out = args.out or Path("/tmp") / f"sim-probe-{name}-{int(time.time())}"
    out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.config, out / "printer.cfg")
    return out


def _report(out: Path, config: Path) -> dict:
    capture = analyze.load_capture(out)
    if capture.aux is None:
        raise ProbeError(f"{out} has no aux records")
    motors = analyze.read_motors(analyze._config_sections(config.read_text()))
    return sim_report.report(
        capture.meta,
        capture.aux,
        config.read_text(),
        motors,
        analyze.capture_positions(capture),
    )


def run_capture(args) -> dict:
    sim = _sim(args)
    out = _out_dir(args, "capture")
    capture = Capture(out, sim.both_edges_mask, events=args.events, sim=sim)
    responses: list[dict] = []
    started = time.monotonic()
    capture.start()
    try:
        x, y, z = args.start
        pi.gcode_script(f"SET_KINEMATIC_POSITION X={x} Y={y} Z={z}")
        for script in args.gcode:
            capture.check()
            responses.append({"script": script, "responses": gcode(script)})
        pi.gcode_script("M400")
        while time.monotonic() - started < args.seconds:
            capture.check()
            time.sleep(0.1)
    finally:
        capture.stop()
    toolhead = query(["toolhead"])["toolhead"]
    return {
        "out": str(out),
        "klippy_position": toolhead["position"][:3],
        "klippy_homed_axes": toolhead["homed_axes"],
        "gcode": responses,
        "simulator": _report(out, args.config),
    }


def _plateau_mean(times: list[float], temps: list[float], end: float) -> float:
    window = [t for s, t in zip(times, temps) if s >= end - PLATEAU_S]
    if not window:
        raise ProbeError("no klippy readings in the plateau window")
    return sum(window) / len(window)


def run_heat(args) -> dict:
    sim = _sim(args)
    out = _out_dir(args, "heat")
    obj = HEATER_OBJECTS[args.heater]
    capture = Capture(out, sim.both_edges_mask, sim=sim)
    readings: list[tuple[float, float, float]] = []
    started = time.monotonic()
    capture.start()
    try:
        pi.gcode_script(f"{HEATER_COMMANDS[args.heater]} S{args.target}")
        while time.monotonic() - started < args.seconds:
            capture.check()
            status = query([obj])[obj]
            readings.append(
                (
                    time.monotonic() - started,
                    status["temperature"],
                    status["power"],
                )
            )
            time.sleep(POLL_S)
    finally:
        pi.gcode_script(f"{HEATER_COMMANDS[args.heater]} S0")
        capture.stop()
    report = _report(out, args.config)
    capture_data = analyze.load_capture(out)
    seconds = sim_report.record_seconds(capture_data.aux, "counts")
    index = sim_report.HEATER_NAMES.index(args.heater)
    model = capture_data.aux["temp_centi"][:, index] / 100.0
    end = seconds[-1]
    model_plateau = float(model[seconds >= end - PLATEAU_S].mean())
    times = [r[0] for r in readings]
    temps = [r[1] for r in readings]
    klippy_plateau = _plateau_mean(times, temps, min(end, times[-1]))
    return {
        "out": str(out),
        "target_c": args.target,
        "klippy_first_c": temps[0],
        "model_first_c": float(model[0]),
        "klippy_plateau_c": klippy_plateau,
        "model_plateau_c": model_plateau,
        "klippy_minus_model_c": klippy_plateau - model_plateau,
        "klippy_max_c": max(temps),
        "klippy_final_power": readings[-1][2],
        "simulator": report,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    lock.add_timeout_argument(parser)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("query").add_argument("objects", nargs="+")
    commands.add_parser("gcode").add_argument("script")
    commands.add_parser("endstops")
    for name in ("capture", "heat"):
        sub = commands.add_parser(name)
        sub.add_argument("--sim", type=Path, default=DEFAULT_SIM)
        sub.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
        sub.add_argument("--start", type=_parse_xyz, default=(0.0, 0.0, 0.0))
        sub.add_argument("--seconds", type=float, required=True)
        sub.add_argument("--out", type=Path)
    capture = commands.choices["capture"]
    capture.add_argument("--events", action="store_true")
    capture.add_argument("--gcode", action="append", default=[])
    heat = commands.choices["heat"]
    heat.add_argument("--heater", choices=sorted(HEATER_OBJECTS), required=True)
    heat.add_argument("--target", type=float, required=True)
    args = parser.parse_args()
    with lock.bench_lock(args.lock_timeout):
        if args.command == "query":
            result = query(args.objects)
        elif args.command == "gcode":
            result = gcode(args.script)
        elif args.command == "endstops":
            result = endstops()
        elif args.command == "capture":
            result = run_capture(args)
        else:
            result = run_heat(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
