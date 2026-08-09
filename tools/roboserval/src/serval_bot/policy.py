from __future__ import annotations

import tomllib
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any


class PolicyError(RuntimeError):
    pass


class Mode(StrEnum):
    SHADOW = "shadow"
    TRIAGE = "triage"
    MAINTAINER = "maintainer"


class Capability(StrEnum):
    LABEL = "label"
    COMMENT = "comment"
    DISPATCH_SIM = "dispatch_sim"
    READ_SIM = "read_sim"


_CAPABILITIES: dict[Mode, frozenset[Capability]] = {
    Mode.SHADOW: frozenset(),
    Mode.TRIAGE: frozenset({Capability.LABEL, Capability.COMMENT}),
    Mode.MAINTAINER: frozenset({Capability.LABEL, Capability.COMMENT, Capability.DISPATCH_SIM, Capability.READ_SIM}),
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
        return login.casefold() in self.maintainers


@dataclass(slots=True, frozen=True)
class PolicySet:
    repositories: dict[str, RepositoryPolicy]

    @classmethod
    def load(cls, path: Path) -> PolicySet:
        with path.open("rb") as stream:
            raw = tomllib.load(stream)
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
    maintainers = frozenset(item.strip().lstrip("@").casefold() for item in maintainers_raw if item.strip())
    return RepositoryPolicy(
        repo=repo,
        mode=mode,
        bot_login=_require_text(value.get("bot_login", "serval-bot"), f"{repo}.bot_login").lstrip("@"),
        maintainers=maintainers,
        sim_workflow=_require_text(value.get("sim_workflow", "ci-sim-e2e.yaml"), f"{repo}.sim_workflow"),
    )
