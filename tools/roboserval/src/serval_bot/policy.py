from __future__ import annotations

import tomllib
from dataclasses import dataclass
from enum import StrEnum
from typing import Any


def normalize_login(login: str) -> str:
    return login.strip().removesuffix("[bot]").casefold()


class PolicyError(RuntimeError):
    pass


class Mode(StrEnum):
    SHADOW = "shadow"
    TRIAGE = "triage"
    MAINTAINER = "maintainer"


class Capability(StrEnum):
    LABEL = "label"
    COMMENT = "comment"
    REVIEW = "review"
    DISPATCH_SIM = "dispatch_sim"
    READ_SIM = "read_sim"


_CAPABILITIES: dict[Mode, frozenset[Capability]] = {
    Mode.SHADOW: frozenset(),
    Mode.TRIAGE: frozenset(
        {Capability.LABEL, Capability.COMMENT, Capability.REVIEW, Capability.DISPATCH_SIM, Capability.READ_SIM}
    ),
    Mode.MAINTAINER: frozenset(
        {Capability.LABEL, Capability.COMMENT, Capability.REVIEW, Capability.DISPATCH_SIM, Capability.READ_SIM}
    ),
}


@dataclass(slots=True, frozen=True)
class RepositoryPolicy:
    repo: str
    mode: Mode
    bot_login: str
    maintainers: frozenset[str]
    sim_workflow: str

    def permits(self, capability: Capability) -> bool:
        return capability in _CAPABILITIES[self.mode]

    def is_maintainer(self, login: str) -> bool:
        return normalize_login(login) in self.maintainers


@dataclass(slots=True, frozen=True)
class PolicySet:
    repositories: dict[str, RepositoryPolicy]

    @classmethod
    def parse(cls, content: str) -> PolicySet:
        raw = tomllib.loads(content)
        repositories = raw.get("repositories")
        if not isinstance(repositories, dict) or not repositories:
            raise PolicyError('policy must define at least one [repositories."owner/repo"] table')
        parsed: dict[str, RepositoryPolicy] = {}
        for repo, value in repositories.items():
            parsed[repo.casefold()] = _parse_repository(repo, value)
        return cls(parsed)

    def require(self, repo: str) -> RepositoryPolicy:
        policy = self.repositories.get(repo.casefold())
        if policy is None:
            raise PolicyError(f"repository is not allowlisted: {repo}")
        return policy


def _require_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PolicyError(f"{field} must be a non-empty string")
    return value.strip()


def _parse_repository(repo: str, value: Any) -> RepositoryPolicy:
    if not isinstance(value, dict):
        raise PolicyError(f"repository policy must be a table: {repo}")
    unknown = set(value) - {"mode", "bot_login", "maintainers", "sim_workflow"}
    if unknown:
        raise PolicyError(f"unknown policy fields for {repo}: {', '.join(sorted(unknown))}")
    if repo.count("/") != 1:
        raise PolicyError(f"repository key must be owner/name: {repo}")
    try:
        mode = Mode(value.get("mode", "shadow"))
    except ValueError as exc:
        raise PolicyError(f"invalid mode for {repo}: {value.get('mode')}") from exc
    maintainers_raw = value.get("maintainers", [])
    if not isinstance(maintainers_raw, list) or not all(isinstance(item, str) for item in maintainers_raw):
        raise PolicyError(f"maintainers must be a string array for {repo}")
    maintainers = frozenset(normalize_login(item.strip().lstrip("@")) for item in maintainers_raw if item.strip())
    bot_login = _require_text(value.get("bot_login", "serval-bot"), f"{repo}.bot_login").lstrip("@")
    if normalize_login(bot_login) in maintainers:
        raise PolicyError(f"bot login cannot be a maintainer: {repo}.{bot_login}")
    return RepositoryPolicy(
        repo=repo,
        mode=mode,
        bot_login=bot_login,
        maintainers=maintainers,
        sim_workflow=_require_text(value.get("sim_workflow", "ci-sim-e2e.yaml"), f"{repo}.sim_workflow"),
    )
