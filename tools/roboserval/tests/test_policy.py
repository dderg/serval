import pytest

from serval_bot.policy import Capability, Mode, PolicyError, PolicySet


def test_policy_modes_enforce_capabilities() -> None:
    policy = PolicySet.parse(
        """
[repositories."dderg/serval"]
mode = "shadow"
bot_login = "serval-bot"
maintainers = ["@Dderg"]
sim_workflow = "ci-sim-e2e.yaml"
"""
    ).require("DDERG/SERVAL")
    assert policy.mode is Mode.SHADOW
    assert not policy.permits(Capability.COMMENT)
    assert policy.is_maintainer("dderg")


def test_policy_rejects_invalid_repository_key() -> None:
    with pytest.raises(PolicyError, match="owner/name"):
        PolicySet.parse('[repositories.invalid]\nmode = "shadow"\n')


def test_policy_rejects_configured_base_branch() -> None:
    with pytest.raises(PolicyError, match=r"unknown policy fields.*base_branch"):
        PolicySet.parse('[repositories."dderg/serval"]\nbase_branch = "sota-motion"\n')


def test_policy_rejects_bot_login_as_maintainer() -> None:
    with pytest.raises(PolicyError, match="bot login cannot be a maintainer"):
        PolicySet.parse(
            """
[repositories."dderg/serval"]
bot_login = "roboserval"
maintainers = ["RoboServal[bot]"]
"""
        )
