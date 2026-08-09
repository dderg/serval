from pathlib import Path

import pytest

from serval_bot.policy import Capability, Mode, PolicyError, PolicySet


def test_policy_modes_enforce_capabilities(tmp_path: Path) -> None:
    path = tmp_path / "repositories.toml"
    path.write_text(
        """
[repositories."dderg/serval"]
mode = "shadow"
bot_login = "serval-bot"
maintainers = ["@Dderg"]
sim_workflow = "ci-sim-e2e.yaml"
"""
    )
    policy = PolicySet.load(path).require("DDERG/SERVAL")
    assert policy.mode is Mode.SHADOW
    assert not policy.permits(Capability.COMMENT)
    assert policy.is_maintainer("dderg")


def test_policy_rejects_invalid_repository_key(tmp_path: Path) -> None:
    path = tmp_path / "repositories.toml"
    path.write_text('[repositories.invalid]\nmode = "shadow"\n')
    with pytest.raises(PolicyError, match="owner/name"):
        PolicySet.load(path)


def test_policy_rejects_configured_base_branch(tmp_path: Path) -> None:
    path = tmp_path / "repositories.toml"
    path.write_text('[repositories."dderg/serval"]\nbase_branch = "sota-motion"\n')
    with pytest.raises(PolicyError, match=r"unknown policy fields.*base_branch"):
        PolicySet.load(path)
