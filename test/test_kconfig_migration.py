import os
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).parent.parent
LEGACY_CONFIG = ROOT / "test" / "configs" / "stm32h723.config"
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


def _make_dry_run(tree):
    env = {k: v for k, v in os.environ.items() if k != "CROSS_PREFIX"}
    return subprocess.run(
        ["make", "-n", "all"],
        cwd=tree,
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )


def test_a_foreign_config_is_migrated_before_features_are_selected(tmp_path):
    tree = _build_tree(tmp_path)
    shutil.copy(LEGACY_CONFIG, tree / ".config")
    assert STEPPER_ON not in (tree / ".config").read_text()

    result = _make_dry_run(tree)

    migrated = (tree / ".config").read_text()
    assert MARKER in migrated
    assert STEPPER_ON in migrated
    assert STEPPER_FEATURE in result.stdout
    assert "olddefconfig" in result.stderr


def test_a_migrated_config_keeps_an_explicit_stepper_choice(tmp_path):
    tree = _build_tree(tmp_path)
    shutil.copy(LEGACY_CONFIG, tree / ".config")
    _make_dry_run(tree)
    config = tree / ".config"
    config.write_text(config.read_text().replace(STEPPER_ON, STEPPER_OFF))

    result = _make_dry_run(tree)

    assert STEPPER_OFF in config.read_text()
    assert STEPPER_FEATURE not in result.stdout
    assert "olddefconfig" not in result.stderr
