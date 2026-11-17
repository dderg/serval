"""The scenario fuzzer's pure parts: generation is a function of the seed,
a saved scenario replays exactly, the timeline file round-trips, and the
minimizer keeps only what the failure needs."""

import dataclasses

import pytest

from tools.sim.fuzzing import (
    checks,
    minimize,
    printer,
    program,
    scenario,
    timeline,
)
from tools.sim.fuzzing.timeline import Act, Action

pytestmark = pytest.mark.sim_unit

SEEDS = range(40)
PTYS = ("/tmp/h7", "/tmp/f4", "/tmp/gcodes")


def test_a_seed_draws_the_same_scenario_every_time():
    for seed in SEEDS:
        assert scenario.generate(seed) == scenario.generate(seed)


def test_seeds_draw_different_printers_programs_and_timelines():
    drawn = [scenario.generate(seed) for seed in SEEDS]
    assert len({s.printer for s in drawn}) == len(drawn)
    assert len({s.programs for s in drawn}) == len(drawn)
    assert len({s.timeline for s in drawn if s.timeline}) > len(drawn) // 2


def test_a_saved_scenario_loads_back_equal(tmp_path):
    for seed in SEEDS:
        original = scenario.generate(seed)
        directory = tmp_path / str(seed)
        scenario.save(original, directory)
        assert scenario.load(directory) == original


def test_the_timeline_file_round_trips():
    actions = tuple(
        a for seed in SEEDS for a in scenario.generate(seed).timeline
    )
    assert actions
    assert timeline.from_text(timeline.to_text(actions)) == actions


def test_an_edited_timeline_file_replays(tmp_path):
    original = scenario.generate(3)
    scenario.save(original, tmp_path)
    edited = "# a human edit\nprint=0 at=1.250 pause_resume hold_s=0.50\n"
    (tmp_path / scenario.TIMELINE_FILE).write_text(edited)
    loaded = scenario.load(tmp_path)
    assert loaded.timeline == (
        Action(0, 1.25, Act.PAUSE_RESUME, (("hold_s", "0.50"),)),
    )


def test_every_drawn_config_renders_with_its_marks_and_motors():
    for seed in SEEDS:
        spec = scenario.generate(seed).printer
        text = printer.render_config(spec, *PTYS)
        assert f"[output_pin {printer.MARK_PIN}]" in text
        axes = {m.axis for m in printer.motors(spec, text)}
        assert axes in ({"x", "y", "z", "e"}, {"a", "b", "z", "e"})


def test_the_model_tracks_relative_and_absolute_modes():
    model = program.Model((100.0, 100.0, 10.0, 0.0))
    model.run(
        [
            "G91",
            "G1 X5 Y-5 E1",
            "G90",
            "M82",
            "G92 E0",
            "G1 X0 E2",
            "M83",
            "G1 E0.5",
        ]
    )
    assert model.position == [0.0, 95.0, 10.0, 3.5]


def test_rendered_probes_alternate_the_mark_pin():
    for seed in SEEDS:
        chosen = scenario.generate(seed)
        for prog in chosen.programs:
            model = program.Model(scenario.START_POSITION + (0.0,))
            model.run(program.render(prog))
            values = [probe.value for probe in model.probes]
            assert values == [1 - i % 2 for i in range(len(values))]


def test_the_done_pin_flips_before_end_print_starts():
    done = f"SET_PIN PIN={printer.DONE_PIN} VALUE=1"
    for seed in SEEDS:
        for prog in scenario.generate(seed).programs:
            lines = program.render(prog)
            body_end = lines.index(done)
            assert lines[body_end + 1].startswith("G4 P")
            hold_ms = float(lines[body_end + 1][4:])
            assert hold_ms >= program.PROBE_HOLD_MS
            assert lines[body_end + 2] == "M400"
            assert ("END_PRINT" in lines) == prog.end_print
            assert "END_PRINT" not in lines[:body_end]


def test_lattice_counts_are_exact_on_a_line_and_bracket_off_it():
    assert checks.lattice_counts(100.1, 80.0) == {8008}
    assert checks.lattice_counts(59.99999999999997, 80.0) == {4800}
    assert checks.lattice_counts(100.105, 80.0) == {8008, 8009}


def _with_marker(chosen):
    """A scenario that fails while it keeps one CANCEL action and one
    LATTICE segment of program A."""
    has_action = any(a.act is Act.CANCEL for a in chosen.timeline)
    first = chosen.programs[0]
    has_segment = any(s.kind is program.Kind.LATTICE for s in first.segments)
    return has_action and has_segment


def _planted() -> scenario.Scenario:
    chosen = scenario.generate(11)
    lattice = program.Segment(
        program.Kind.LATTICE, ("G90", "G1 X100 Y100", program.PROBE)
    )
    first = chosen.programs[0]
    padded = dataclasses.replace(
        first, segments=first.segments + (lattice,), end_print=True
    )
    actions = chosen.timeline + (
        Action(0, 2.0, Act.HEATER, (("style", "m104"), ("target", "150"))),
        Action(0, 3.0, Act.CANCEL),
    )
    return dataclasses.replace(
        chosen,
        programs=(padded,) + chosen.programs[1:],
        prints=("A", "A"),
        timeline=actions,
    )


def test_the_minimizer_keeps_only_what_the_failure_needs():
    planted = _planted()
    calls = []

    def reproduces(options):
        calls.append(len(options))
        return [_with_marker(option) for option in options]

    smallest = minimize.minimize(planted, reproduces, batch=4)
    assert [a.act for a in smallest.timeline] == [Act.CANCEL]
    assert [s.kind for s in smallest.programs[0].segments] == [
        program.Kind.LATTICE
    ]
    assert smallest.prints == ("A",)
    assert not smallest.programs[0].end_print
    assert all(size <= 4 for size in calls)


def test_a_panic_signature_carries_its_message():
    stdout = (
        "klippy starting\n"
        "thread 'kalico-shaper' panicked at /kalico/rust/motion-pipeline/"
        "src/shaper.rs:853:29:\n"
        "shaper: axis 3: shaping window needs unavailable history at t=11.1\n"
    )
    panic = checks.first_panic(stdout)
    assert panic.endswith("shaping window needs unavailable history at t=11.1")
    violation = checks.Violation(checks.Check.PANIC, panic)
    assert "unavailable history" in checks.signature(violation)
    assert checks.first_panic("klippy starting\n") is None


def test_probes_flipped_unwatched_are_counted_not_checked():
    from tools.sim.fuzzing.driver import probes_to_skip

    assert probes_to_skip(1, paused=False) == 0
    assert probes_to_skip(1, paused=True) == 1
    assert probes_to_skip(3, paused=False) == 3
