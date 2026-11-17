"""Shrink a failing scenario: drop trailing prints, timeline actions,
program segments and END_PRINT, and set printer knobs to their plainest
value, while the same failure still reproduces."""

from __future__ import annotations

import dataclasses
from typing import Callable, Iterator

from tools.sim.fuzzing.printer import (
    CancelMacro,
    MeshKind,
    PauseMacro,
    PrinterSpec,
    Topology,
)
from tools.sim.fuzzing.program import Kind, Program, Segment
from tools.sim.fuzzing.scenario import Lead, Scenario, with_parts
from tools.sim.fuzzing.timeline import Act

Reproduces = Callable[[list[Scenario]], list[bool]]
LABEL = "EXCLUDE_OBJECT_"

PLAIN_KNOBS = {
    "shaper_x": None,
    "shaper_y": None,
    "extruder_kernel": None,
    "heaters": False,
    "idle_timeout_s": None,
    "end_steps": (),
    "pause_macro": PauseMacro.BASE,
    "cancel_macro": CancelMacro.BASE,
    "mesh_fade": False,
    "mesh_at_start": False,
}
PROCESSOR_OF_KNOB = {
    "shaper_x": "fz_shaper_x",
    "shaper_y": "fz_shaper_y",
    "extruder_kernel": "fz_kernel",
}
NO_MESH = {
    "mesh": MeshKind.NONE,
    "mesh_points": (),
    "mesh_fade": False,
    "mesh_at_start": False,
}
NO_PA = {"pa_k": None, "extruder_kernel": None}


def _targets(scenario: Scenario, act: Act, key: str) -> set:
    return {a.param(key) for a in scenario.timeline if a.act is act}


def _knob_changes(scenario: Scenario) -> Iterator[dict]:
    spec = scenario.printer
    named = _targets(scenario, Act.SET_POST_PROCESSOR, "name")
    for knob, plain in PLAIN_KNOBS.items():
        if PROCESSOR_OF_KNOB.get(knob) not in named:
            yield {knob: plain}
    if spec.pa_k is not None and not named & {"fz_pa", "fz_kernel"}:
        yield NO_PA
    if spec.topology is Topology.XY_ON_AUX and "f4" not in _targets(
        scenario, Act.LINK_FAULT, "mcu"
    ):
        yield {"topology": Topology.ONE_MCU}
    if spec.mesh is not MeshKind.NONE and not _uses_mesh(scenario):
        yield NO_MESH
    if spec.runout_sensor and not _targets(scenario, Act.RUNOUT, "hold_s"):
        yield {"runout_sensor": False}
    if spec.hotend_model and Kind.HEAT_WAIT not in _kinds(scenario):
        yield {"hotend_model": False}


def _kinds(scenario: Scenario) -> set:
    return {kind for p in scenario.programs for kind in p.kinds()}


def _uses_mesh(scenario: Scenario) -> bool:
    acts = {a.act for a in scenario.timeline}
    kinds = _kinds(scenario)
    return bool(acts & {Act.MESH_LOAD, Act.MESH_CLEAR}) or Kind.MESH in kinds


def _unlabelled(prog: Program) -> Program:
    segments = tuple(
        Segment(
            s.kind,
            tuple(line for line in s.lines if not line.startswith(LABEL)),
        )
        for s in prog.segments
        if s.kind is not Kind.OBJECTS
    )
    return dataclasses.replace(prog, segments=segments)


def _without_objects(scenario: Scenario) -> Scenario:
    """[exclude_object] off and the object labels it needs gone."""
    programs = tuple(_unlabelled(p) for p in scenario.programs)
    plainer = dataclasses.replace(scenario.printer, exclude_object=False)
    return dataclasses.replace(scenario, printer=plainer, programs=programs)


def _plainer_printers(scenario: Scenario) -> Iterator[Scenario]:
    spec = scenario.printer
    for change in _knob_changes(scenario):
        plainer: PrinterSpec = dataclasses.replace(spec, **change)
        if plainer != spec:
            yield dataclasses.replace(scenario, printer=plainer)
    if spec.exclude_object and not _targets(scenario, Act.EXCLUDE_OBJECT, "name"):
        yield _without_objects(scenario)


def _without_action(scenario: Scenario, index: int) -> Scenario:
    actions = scenario.timeline[:index] + scenario.timeline[index + 1 :]
    return with_parts(scenario, actions=actions)


def _without_segment(scenario: Scenario, name: str, index: int) -> Scenario:
    programs = tuple(
        dataclasses.replace(
            p, segments=p.segments[:index] + p.segments[index + 1 :]
        )
        if p.name == name
        else p
        for p in scenario.programs
    )
    return with_parts(scenario, programs=programs)


def _without_end_print(scenario: Scenario, name: str) -> Scenario:
    programs = tuple(
        dataclasses.replace(p, end_print=False) if p.name == name else p
        for p in scenario.programs
    )
    return with_parts(scenario, programs=programs)


def _without_last_print(scenario: Scenario) -> Scenario:
    last = len(scenario.prints) - 1
    prints = scenario.prints[:last]
    actions = tuple(a for a in scenario.timeline if a.print_index != last)
    programs = tuple(p for p in scenario.programs if p.name in prints)
    return dataclasses.replace(
        scenario,
        prints=prints,
        timeline=actions,
        programs=programs,
        leads=scenario.leads[:last],
    )


def candidates(scenario: Scenario) -> Iterator[Scenario]:
    """Every scenario one element smaller or one knob plainer, the
    biggest cuts first."""
    if len(scenario.prints) > 1:
        yield _without_last_print(scenario)
    if any(lead != Lead() for lead in scenario.leads):
        yield dataclasses.replace(scenario, leads=())
    for index in reversed(range(len(scenario.timeline))):
        yield _without_action(scenario, index)
    for prog in scenario.programs:
        for index in reversed(range(len(prog.segments))):
            yield _without_segment(scenario, prog.name, index)
        if prog.end_print:
            yield _without_end_print(scenario, prog.name)
    yield from _plainer_printers(scenario)


def minimize(
    scenario: Scenario, reproduces: Reproduces, batch: int
) -> Scenario:
    """Greedy: run the candidates `batch` at a time and keep the first
    that still fails, until none does."""
    current = scenario
    while True:
        smaller = _first_reproducing(
            list(candidates(current)), reproduces, batch
        )
        if smaller is None:
            return current
        current = smaller


def _first_reproducing(
    options: list[Scenario], reproduces: Reproduces, batch: int
):
    for start in range(0, len(options), batch):
        chunk = options[start : start + batch]
        for option, failed in zip(chunk, reproduces(chunk)):
            if failed:
                return option
    return None
