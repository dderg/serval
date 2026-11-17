import os
import pathlib
import shutil
import subprocess

import pytest

ROOT = pathlib.Path(__file__).parent.parent
LEGACY_CONFIG = ROOT / "test" / "configs" / "stm32h723.config"
BENCH_CONFIG = ROOT / "tools" / "bench" / "f401.config"
MARKER = "CONFIG_SERVAL_CONFIG_MIGRATED=y"
STEPPER_ON = "CONFIG_MOTION_MODULE_STEPPER=y"
STEPPER_OFF = "# CONFIG_MOTION_MODULE_STEPPER is not set"
STEPPER_FEATURE = "motion-module-stepper"
OLDER_THAN_CONFIG = 1_000_000_000


def _build_tree(tmp_path):
    tree = tmp_path / "tree"
    tree.mkdir()
    shutil.copy2(ROOT / "Makefile", tree / "Makefile")
    shutil.copytree(
        ROOT / "src",
        tree / "src",
        ignore=shutil.ignore_patterns("__pycache__", "extras"),
    )
    (tree / "src" / "extras").mkdir()
    for shared in ("lib", "scripts", "rust"):
        (tree / shared).symlink_to(ROOT / shared)
    for kconfig in (tree / "src").rglob("Kconfig*"):
        os.utime(kconfig, (OLDER_THAN_CONFIG, OLDER_THAN_CONFIG))
    return tree


def _committed_configs():
    listed = subprocess.run(
        ["git", "ls-files", "*.config"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    return [ROOT / name for name in listed.stdout.split()]


COMMITTED_CONFIGS = _committed_configs()


def _make(tree, *args):
    env = {k: v for k, v in os.environ.items() if k != "CROSS_PREFIX"}
    return subprocess.run(
        ["make", *args],
        cwd=tree,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )


def _make_dry_run(tree):
    return _make(tree, "-n", "all")


def _assert_untouched(tree, committed):
    assert (tree / ".config").read_text() == committed.read_text()
    assert not (tree / ".config.old").exists()


def _write_foreign_config(tree):
    lines = LEGACY_CONFIG.read_text().splitlines(keepends=True)
    foreign = [line for line in lines if line.strip() != MARKER]
    (tree / ".config").write_text("".join(foreign))


def test_the_committed_configs_include_the_bench_config():
    assert BENCH_CONFIG in COMMITTED_CONFIGS


@pytest.mark.parametrize(
    "committed", COMMITTED_CONFIGS, ids=lambda path: str(path.relative_to(ROOT))
)
def test_a_build_in_place_leaves_a_committed_config_unchanged(tmp_path, committed):
    tree = _build_tree(tmp_path)
    shutil.copy(committed, tree / ".config")

    _make_dry_run(tree)

    _assert_untouched(tree, committed)


def test_the_bench_firmware_build_leaves_its_config_unchanged(tmp_path):
    tree = _build_tree(tmp_path)
    shutil.copy(BENCH_CONFIG, tree / ".config")

    _make(tree, "olddefconfig")
    _make_dry_run(tree)

    _assert_untouched(tree, BENCH_CONFIG)


def test_a_foreign_config_is_migrated_before_features_are_selected(tmp_path):
    tree = _build_tree(tmp_path)
    _write_foreign_config(tree)
    assert STEPPER_ON not in (tree / ".config").read_text()

    result = _make_dry_run(tree)

    migrated = (tree / ".config").read_text()
    assert MARKER in migrated
    assert STEPPER_ON in migrated
    assert STEPPER_FEATURE in result.stdout
    assert "olddefconfig" in result.stderr


def test_a_migrated_config_keeps_an_explicit_stepper_choice(tmp_path):
    tree = _build_tree(tmp_path)
    _write_foreign_config(tree)
    _make_dry_run(tree)
    config = tree / ".config"
    config.write_text(config.read_text().replace(STEPPER_ON, STEPPER_OFF))

    result = _make_dry_run(tree)

    assert STEPPER_OFF in config.read_text()
    assert STEPPER_FEATURE not in result.stdout
    assert "olddefconfig" not in result.stderr
