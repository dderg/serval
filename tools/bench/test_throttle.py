from __future__ import annotations

import re

import throttle

LEFT_THROTTLED = (
    f"{throttle.REMOVED_DROPIN} {throttle.DROPIN}\n"
    f"uncapped {throttle.CPUFREQ}/policy0 from 1500000 to 2400000 kHz\n"
    "orphaned job cargo killed (pid 4242)\n"
)


def _pi(monkeypatch, reply: str) -> list[str]:
    calls = []

    def ssh(command: str) -> str:
        calls.append(command)
        return reply if command == throttle.SETTLE_HOST else ""

    monkeypatch.setattr(throttle.pi, "ssh", ssh)
    monkeypatch.setattr(
        throttle.pi, "restart_klipper", lambda: calls.append("restart")
    )
    return calls


def test_a_host_at_rest_is_left_alone(monkeypatch):
    calls = _pi(monkeypatch, "")
    assert throttle.settle_host() == []
    assert calls == [throttle.SETTLE_HOST]


def test_a_host_a_killed_run_left_throttled_is_restored(monkeypatch):
    calls = _pi(monkeypatch, LEFT_THROTTLED)
    actions = throttle.settle_host()
    assert actions == [
        *LEFT_THROTTLED.splitlines(),
        "klipper restarted without the bench drop-ins",
    ]
    assert calls[1:] == ["sudo systemctl daemon-reload", "restart"]


def test_only_a_removed_drop_in_restarts_klipper(monkeypatch):
    uncapped = LEFT_THROTTLED.splitlines()[1]
    calls = _pi(monkeypatch, uncapped)
    assert throttle.settle_host() == [uncapped]
    assert "restart" not in calls


def test_the_settle_script_never_kills_its_own_shell():
    jobs = {
        "-f 'build-nativ[e].sh'": "./scripts/build-native.sh",
        "-f 'examples/gcode_replay_benc[h]'": "examples/gcode_replay_bench x",
        "-f '^bin[s]/[^ ]+ trees/'": "bins/calibration trees/calibration/a",
    }
    assert set(jobs) <= set(throttle.ORPHANED_JOBS)
    for job, command_line in jobs.items():
        pattern = job.split(" ", 1)[1].strip("'")
        assert re.search(pattern, command_line)
        assert not re.search(pattern, throttle.SETTLE_HOST)
