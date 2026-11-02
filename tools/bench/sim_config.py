"""Simulated-peripheral config for the F407 (tools/bench/capture-fw/PHASE2.md).

A run's JSON (e.g. configs/full.sim.json) names walls in toolhead mm or
motor steps and heater model parameters; `build` turns it into the binary
config the firmware takes with its `C` command, using the printer config's
step sizes and the run's starting position as the origin.
"""

from __future__ import annotations

import json
import struct
from dataclasses import asdict, dataclass, field
from pathlib import Path

CMD_CONFIG = b"C"
TAG_ORIGIN = 1
TAG_WALL = 2
TAG_HEATER = 3
TAG_EXPECT_MCO = 4
MAX_PAYLOAD = 512
CHANNELS = ("a", "b", "z", "e")
PINS = {"x_endstop": 0, "y_endstop": 1, "z_endstop": 2, "probe": 3}
HEATERS = {"extruder": 0, "bed": 1}
HEATER_FIELDS = (
    "power_w",
    "capacity_j_per_k",
    "loss_w_per_k",
    "fan_loss_w_per_k",
    "ambient_c",
    "r25_ohm",
    "beta",
    "pullup_ohm",
)
WALL_ACTIVE_HIGH = 1 << 0
WALL_AT_OR_BELOW = 1 << 1
TRIGGERS = {"at_or_below": True, "at_or_above": False}
SPEC_KEYS = {"expect_mco", "both_edges", "walls", "heaters"}
WALL_KEYS = {"pin", "axis", "trigger", "at_mm", "at_steps", "active_high"}


class SimConfigError(ValueError):
    pass


@dataclass(frozen=True)
class Wall:
    pin: int
    active_high: bool
    at_or_below: bool
    coeffs: tuple[int, int, int, int]
    threshold: int


@dataclass(frozen=True)
class Heater:
    index: int
    params: tuple[float, ...]


@dataclass(frozen=True)
class SimConfig:
    config_id: int
    origin: tuple[int, int, int, int] = (0, 0, 0, 0)
    walls: tuple[Wall, ...] = ()
    heaters: tuple[Heater, ...] = ()
    expect_mco: bool = False
    both_edges_mask: int = 0
    source: dict = field(default_factory=dict, compare=False)


def _entry(tag: int, body: bytes) -> bytes:
    return struct.pack("<BB", tag, len(body)) + body


def encode(config: SimConfig) -> bytes:
    out = bytearray(struct.pack("<H", config.config_id))
    out += _entry(TAG_ORIGIN, struct.pack("<4i", *config.origin))
    for wall in config.walls:
        flags = (WALL_ACTIVE_HIGH if wall.active_high else 0) | (
            WALL_AT_OR_BELOW if wall.at_or_below else 0
        )
        out += _entry(
            TAG_WALL,
            struct.pack(
                "<BB4bi", wall.pin, flags, *wall.coeffs, wall.threshold
            ),
        )
    for heater in config.heaters:
        out += _entry(
            TAG_HEATER, struct.pack("<B8f", heater.index, *heater.params)
        )
    out += _entry(TAG_EXPECT_MCO, bytes([int(config.expect_mco)]))
    if len(out) > MAX_PAYLOAD:
        raise SimConfigError(
            f"sim config of {len(out)} B exceeds {MAX_PAYLOAD}"
        )
    return bytes(out)


def describe(config: SimConfig) -> dict:
    return {
        "config_id": config.config_id,
        "origin": list(config.origin),
        "walls": [asdict(wall) for wall in config.walls],
    }


def _check_keys(spec: dict, allowed: set[str], what: str) -> None:
    unknown = set(spec) - allowed
    if unknown:
        raise SimConfigError(f"{what}: unknown keys {sorted(unknown)}")


def command(config: SimConfig) -> bytes:
    payload = encode(config)
    return CMD_CONFIG + struct.pack("<H", len(payload)) + payload


def _steps(mm: float, step_mm: float, what: str) -> int:
    steps = mm / step_mm
    rounded = round(steps)
    if abs(steps - rounded) > 1e-6:
        raise SimConfigError(f"{what} {mm} mm is not a whole number of steps")
    return rounded


def _wall(
    spec: dict, step_mm: dict[str, float], dir_sign: dict[str, int]
) -> Wall:
    _check_keys(spec, WALL_KEYS, f"wall {spec}")
    if ("at_mm" in spec) == ("at_steps" in spec):
        raise SimConfigError(
            f"wall {spec}: give exactly one of at_mm, at_steps"
        )
    try:
        pin = PINS[spec["pin"]]
        at_or_below = TRIGGERS[spec["trigger"]]
    except KeyError as exc:
        raise SimConfigError(f"wall {spec}: unknown {exc}") from exc
    axis = spec["axis"]
    active_high = bool(spec.get("active_high", True))
    if axis in ("x", "y"):
        if "at_steps" in spec:
            raise SimConfigError(f"wall {spec}: toolhead walls take at_mm")
        if step_mm["a"] != step_mm["b"]:
            raise SimConfigError("CoreXY walls need equal a/b step sizes")
        sign = 1 if axis == "x" else -1
        steps = _steps(spec["at_mm"], step_mm["a"], f"{axis} wall")
        coeffs = (dir_sign["a"], sign * dir_sign["b"], 0, 0)
        return Wall(pin, active_high, at_or_below, coeffs, 2 * steps)
    if axis in CHANNELS:
        coeffs = tuple(dir_sign[c] * int(c == axis) for c in CHANNELS)
        if "at_steps" in spec:
            threshold = int(spec["at_steps"])
        else:
            threshold = _steps(spec["at_mm"], step_mm[axis], f"{axis} wall")
        return Wall(pin, active_high, at_or_below, coeffs, threshold)
    raise SimConfigError(f"wall axis {axis!r} is not x, y or a motor")


def _heater(name: str, spec: dict) -> Heater:
    if name not in HEATERS:
        raise SimConfigError(f"unknown heater {name!r}")
    _check_keys(spec, set(HEATER_FIELDS), f"heater {name}")
    missing = [f for f in HEATER_FIELDS if f not in spec]
    if missing:
        raise SimConfigError(f"heater {name} misses {missing}")
    return Heater(HEATERS[name], tuple(float(spec[f]) for f in HEATER_FIELDS))


def corexy_origin(
    start_xyz: tuple[float, float, float],
    step_mm: dict[str, float],
    dir_sign: dict[str, int],
) -> tuple[int, int, int, int]:
    x, y, z = start_xyz
    motor = {
        "a": _steps(x + y, step_mm["a"], "start a"),
        "b": _steps(x - y, step_mm["b"], "start b"),
        "z": _steps(z, step_mm["z"], "start z"),
        "e": 0,
    }
    return tuple(dir_sign[c] * motor[c] for c in CHANNELS)


def build(
    spec: dict,
    config_id: int,
    step_mm: dict[str, float],
    dir_sign: dict[str, int],
    start_xyz: tuple[float, float, float],
) -> SimConfig:
    _check_keys(spec, SPEC_KEYS, "sim config")
    mask = 0
    for role in spec.get("both_edges", []):
        if role not in CHANNELS:
            raise SimConfigError(f"both_edges role {role!r} is not a channel")
        mask |= 1 << CHANNELS.index(role)
    return SimConfig(
        config_id=config_id,
        origin=corexy_origin(start_xyz, step_mm, dir_sign),
        walls=tuple(_wall(w, step_mm, dir_sign) for w in spec.get("walls", [])),
        heaters=tuple(
            _heater(n, h) for n, h in spec.get("heaters", {}).items()
        ),
        expect_mco=bool(spec.get("expect_mco", False)),
        both_edges_mask=mask,
        source=spec,
    )


def load(
    path: Path,
    config_id: int,
    step_mm: dict[str, float],
    dir_sign: dict[str, int],
    start_xyz: tuple[float, float, float],
) -> SimConfig:
    return build(
        json.loads(path.read_text()), config_id, step_mm, dir_sign, start_xyz
    )
