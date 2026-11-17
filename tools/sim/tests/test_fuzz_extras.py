"""The scenario fuzzer's first-print extras: heat-up waits on a thermal
model, object labels and EXCLUDE_OBJECT, a filament runout switch, M220,
M221 and SET_VELOCITY_LIMIT from the console, idle gaps and
SDCARD_RESET_FILE before a print. They draw from streams of their own,
so the printer, programs and actions a seed drew before stay as they
were."""

import dataclasses
import random
import re

import pytest

from tools.sim.fuzzing import minimize, printer, program, scenario, thermal
from tools.sim.fuzzing.timeline import Act, draw_print_actions

pytestmark = pytest.mark.sim_unit

SEEDS = range(12000, 12200)
PTYS = ("/tmp/h7", "/tmp/f4", "/tmp/gcodes")
SHIM_DEFAULT_ADC = 3900
EXTRA_ACTS = {
    Act.SPEED_FACTOR,
    Act.EXTRUDE_FACTOR,
    Act.VELOCITY_LIMIT,
    Act.RUNOUT,
    Act.EXCLUDE_OBJECT,
}
EXTRA_KINDS = (program.Kind.HEAT_WAIT, program.Kind.OBJECTS)
TARGET = re.compile(r"(?:\bS|TARGET=)(\d+)\b")


def _child(seed: int, part: str) -> random.Random:
    return random.Random(f"{seed}:{part}")


def _without_extras(prog: program.Program) -> program.Program:
    segments = tuple(
        program.Segment(
            s.kind,
            tuple(line for line in s.lines if not line.startswith("EXCLUDE")),
        )
        for s in prog.segments
        if s.kind not in EXTRA_KINDS
    )
    return dataclasses.replace(prog, segments=segments)


def test_the_extras_leave_what_a_seed_drew_before():
    for seed in SEEDS:
        chosen = scenario.generate(seed)
        plain = printer.draw_printer(_child(seed, "printer"))
        assert dataclasses.replace(
            chosen.printer,
            runout_sensor=False,
            exclude_object=False,
            hotend_model=False,
        ) == plain
        rng = _child(seed, "timeline")
        drawn = []
        for index, name in enumerate(chosen.prints):
            prog = _without_extras(chosen.program(name))
            duration = scenario.estimate_s(
                program.render(prog), plain.max_velocity
            )
            drawn += draw_print_actions(rng, plain, index, duration)
        kept = [a for a in chosen.timeline if a.act not in EXTRA_ACTS]
        assert kept == drawn


def test_every_extra_is_drawn():
    drawn = [scenario.generate(seed) for seed in SEEDS]
    acts = {a.act for s in drawn for a in s.timeline}
    kinds = {k for s in drawn for p in s.programs for k in p.kinds()}
    assert EXTRA_ACTS <= acts
    assert set(EXTRA_KINDS) <= kinds
    assert any(lead.idle_s for s in drawn for lead in s.leads)
    assert any(lead.reset_file for s in drawn for lead in s.leads)


def test_heat_waits_need_the_thermal_model_and_stay_below_max_temp():
    for seed in SEEDS:
        chosen = scenario.generate(seed)
        for prog in chosen.programs:
            waits = [
                s for s in prog.segments if s.kind is program.Kind.HEAT_WAIT
            ]
            assert not waits or chosen.printer.hotend_model
            for segment in waits:
                temps = [
                    int(match)
                    for line in segment.lines
                    for match in TARGET.findall(line)
                ]
                low, high = program.HEAT_WAIT_C
                assert temps and all(low <= t <= high for t in temps)


def test_object_labels_pair_up_and_are_defined_first():
    for seed in SEEDS:
        for prog in scenario.generate(seed).programs:
            names = program.object_names(prog)
            if not names:
                continue
            assert prog.segments[0].kind is program.Kind.OBJECTS
            for segment in prog.segments[1:]:
                starts = [
                    line.split("NAME=")[1]
                    for line in segment.lines
                    if line.startswith("EXCLUDE_OBJECT_START")
                ]
                ends = [
                    line.split("NAME=")[1]
                    for line in segment.lines
                    if line.startswith("EXCLUDE_OBJECT_END")
                ]
                assert starts == ends
                assert set(starts) <= set(names)


def test_the_minimizer_drops_unused_extras():
    for seed in SEEDS:
        chosen = scenario.generate(seed)
        stripped = dataclasses.replace(
            chosen, timeline=(), programs=tuple(map(_without_extras, chosen.programs))
        )
        spec = stripped.printer
        smaller = list(minimize.candidates(stripped))
        printers = {c.printer for c in smaller}
        assert not spec.runout_sensor or dataclasses.replace(
            spec, runout_sensor=False
        ) in printers
        assert not spec.hotend_model or dataclasses.replace(
            spec, hotend_model=False
        ) in printers
        if any(lead != scenario.Lead() for lead in chosen.leads):
            assert any(c.leads == () for c in smaller)
    labelled = next(
        s
        for s in map(scenario.generate, SEEDS)
        if s.printer.exclude_object
        and not any(a.act is Act.EXCLUDE_OBJECT for a in s.timeline)
        and any(program.object_names(p) for p in s.programs)
    )
    plain = minimize._without_objects(labelled)
    assert not plain.printer.exclude_object
    for prog in plain.programs:
        lines = program.render(prog)
        assert not [line for line in lines if "EXCLUDE_OBJECT" in line]


def test_the_extras_render_into_the_config():
    for seed in SEEDS:
        spec = scenario.generate(seed).printer
        text = printer.render_config(spec, *PTYS)
        runout = f"[filament_switch_sensor {printer.RUNOUT_SENSOR}]"
        assert (runout in text) == spec.runout_sensor
        assert ("[exclude_object]" in text) == spec.exclude_object
        pins = printer.hotend_pins(text)
        assert pins.sensor_type in thermal.THERMISTOR_POINTS


@pytest.mark.parametrize("temp_c", (25.0, 60.0, 150.0, 190.0, 240.0))
def test_the_thermistor_reads_back_the_temperature_it_was_set_to(temp_c):
    sensor = thermal.thermistor("EPCOS 100K B57560G104F")
    assert sensor.temp_c(sensor.fraction(temp_c)) == pytest.approx(temp_c)
    count = sensor.adc_count(temp_c)
    assert sensor.temp_c(count / thermal.ADC_MAX) == pytest.approx(
        temp_c, abs=1.0
    )


def test_the_shim_default_reads_as_room_temperature():
    sensor = thermal.thermistor("EPCOS 100K B57560G104F")
    room = sensor.temp_c(SHIM_DEFAULT_ADC / thermal.ADC_MAX)
    assert abs(room - thermal.AMBIENT_C) < 3.0


def test_a_hotend_heats_to_a_print_temperature_and_cools_off():
    model = thermal.HotendModel()
    target, step_s, elapsed_s = 190.0, thermal.SAMPLE_S, 0.0
    while model.temp_c < target:
        model.step(1.0, step_s)
        elapsed_s += step_s
    assert 30.0 < elapsed_s < 120.0
    for _ in range(round(60.0 / step_s)):
        model.step(1.0 if model.temp_c < target else 0.0, step_s)
    assert model.temp_c == pytest.approx(target, abs=1.0)
    for _ in range(round(600.0 / step_s)):
        model.step(0.0, step_s)
    assert model.temp_c < thermal.AMBIENT_C + 10.0
