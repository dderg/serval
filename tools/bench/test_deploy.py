from __future__ import annotations

import deploy
import pytest


def test_a_remote_file_missing_from_the_tree_is_stale():
    remote = ["rust/motion-engine/build.rs", "rust/a.rs", "klippy/x.py"]
    local = ["rust/a.rs", "klippy/x.py"]
    assert deploy.stale_remote_files(remote, local) == [
        "rust/motion-engine/build.rs"
    ]


def test_build_outputs_are_never_stale():
    remote = [
        ".bench-deploy.json",
        "klippy/_motion_engine.so",
        "klippy/chelper/c_helper.Linux-aarch64.so",
        "klippy/__pycache__/gcode.cpython-313.pyc",
        "rust/target/release/x",
        ".git/HEAD",
    ]
    assert deploy.stale_remote_files(remote, []) == []


def test_files_the_tree_carries_are_kept():
    names = ["a.py", "rust/b.rs"]
    assert deploy.stale_remote_files(names, names) == []


def _flash_target_bench(
    monkeypatch, tmp_path, reported_suffix: str | None = None
):
    calls = {"make": [], "recorded": [], "mco": 0}
    monkeypatch.setattr(
        deploy.firmware_build_id, "target_build_id", lambda tree, config: "f00d"
    )

    def run_checked(command, cwd, env=None):
        calls["make"].append((command, cwd))
        if command[-1].startswith("-j"):
            (tmp_path / "out/bench-f401/klipper.elf").write_bytes(b"elf")
        return ""

    monkeypatch.setattr(deploy, "run_checked", run_checked)
    monkeypatch.setattr(deploy, "git_output", lambda tree, *args: "abc\n")
    monkeypatch.setattr(deploy.pi, "scp_to_pi", lambda *a: None)
    monkeypatch.setattr(deploy.pi, "ssh", lambda command: deploy.VERIFIED)
    monkeypatch.setattr(deploy, "wait_pi_serial", lambda: None)
    monkeypatch.setattr(deploy.pi, "firmware_restart", lambda: None)

    def enable_mco():
        assert calls["recorded"], "DBGMCU_CR is cleared after the flash lands"
        calls["mco"] += 1
        return 3

    monkeypatch.setattr(deploy.pi, "enable_mco", enable_mco)
    monkeypatch.setattr(
        deploy.pi,
        "mcu_version",
        lambda: f"v0.13-dirty-fw{reported_suffix or 'f00d'}",
    )
    monkeypatch.setattr(
        deploy.pi,
        "record_firmware",
        lambda board, identity: calls["recorded"].append((board, identity)),
    )
    return calls


def test_the_target_firmware_builds_in_its_own_tree_and_carries_its_id(
    monkeypatch, tmp_path
):
    calls = _flash_target_bench(monkeypatch, tmp_path)
    deploy.cmd_flash_target(tmp_path)

    out = tmp_path / "out/bench-f401"
    for command, cwd in calls["make"]:
        assert cwd == tmp_path
        assert f"OUT={out}/" in command
        assert (
            f"KCONFIG_CONFIG={tmp_path / 'tools/bench/f401.config'}" in command
        )
    assert "VERSION_EXTRA=-fwf00d" in calls["make"][-1][0]
    ((board, identity),) = calls["recorded"]
    assert board == "target"
    assert identity["build_id"] == "f00d"
    assert identity["worktree"] == str(tmp_path)
    assert identity["elf"] == str(out / "klipper.elf")
    assert calls["mco"] == 1


def test_a_target_reporting_another_build_is_refused_and_not_recorded(
    monkeypatch, tmp_path
):
    calls = _flash_target_bench(monkeypatch, tmp_path, reported_suffix="beef")
    with pytest.raises(deploy.pi.BenchError, match="not the build f00d"):
        deploy.cmd_flash_target(tmp_path)
    assert calls["recorded"] == []


def test_a_sync_keeps_what_the_boards_were_flashed_with(monkeypatch, tmp_path):
    written = []
    flashed = {"target": {"build_id": "f00d"}, "capture": {"build_id": "beef"}}
    monkeypatch.setattr(deploy, "local_files", lambda tree: [])
    monkeypatch.setattr(deploy, "rsync_changes", lambda tree, files: [])
    monkeypatch.setattr(deploy.pi, "ssh", lambda command: "")
    monkeypatch.setattr(deploy, "deploy_identity", lambda tree: {"head": "abc"})
    monkeypatch.setattr(deploy.pi, "flashed_firmware", lambda: flashed)
    monkeypatch.setattr(deploy.pi, "write_deployed", written.append)
    deploy.cmd_sync(tmp_path)
    assert written == [{"head": "abc", "firmware": flashed}]


def test_a_target_reset_ends_by_clearing_sleep_debug_through_the_mco_attach(
    monkeypatch,
):
    steps = []
    monkeypatch.setattr(
        deploy.pi, "ssh", lambda command: steps.append("openocd reset") or ""
    )
    monkeypatch.setattr(deploy, "wait_pi_serial", lambda: None)
    monkeypatch.setattr(
        deploy.pi, "restart_klipper", lambda: steps.append("restart klipper")
    )
    monkeypatch.setattr(
        deploy.pi, "enable_mco", lambda: steps.append("mco + DBGMCU_CR 0") or 4
    )
    deploy.cmd_reset_target()
    assert steps == ["openocd reset", "restart klipper", "mco + DBGMCU_CR 0"]


def test_a_target_reset_whose_mco_attach_fails_fails_loudly(monkeypatch):
    def enable_mco():
        raise deploy.pi.BenchError("F401 DBGMCU_CR read back ['0x00000007']")

    monkeypatch.setattr(deploy.pi, "ssh", lambda command: "")
    monkeypatch.setattr(deploy, "wait_pi_serial", lambda: None)
    monkeypatch.setattr(deploy.pi, "restart_klipper", lambda: None)
    monkeypatch.setattr(deploy.pi, "enable_mco", enable_mco)
    with pytest.raises(deploy.pi.BenchError, match="DBGMCU_CR"):
        deploy.cmd_reset_target()
