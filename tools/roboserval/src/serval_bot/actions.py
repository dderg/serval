from __future__ import annotations

import json
import re
import threading
from dataclasses import dataclass
from typing import Any

from serval_bot.database import Action, ActionConflict, Database, Event
from serval_bot.policy import Capability, RepositoryPolicy
from serval_bot.proxy_client import ProxyClient


class ActionDenied(RuntimeError):
    pass


_SIM_COMMENT_EVENTS = frozenset({"issue_comment.created", "pull_request_review.requested"})

_SIM_DIRECTIVE_RE = re.compile(
    r"@(?P<login>[\w-]+) +(?:please +)?(?:"
    r"reproduce in the simulator|"
    r"run this in the simulator|"
    r"dispatch the simulator|"
    r"simulate this crash|"
    r"start a simulator run of the attached model"
    r")[.!]?",
    re.IGNORECASE,
)


def parse_sim_directive(bot_login: str, body: str) -> bool:
    """Return True only when body is exactly a supported imperative directive.

    Full-body match against the closed set of positive imperative forms
    addressed to the configured bot login, optionally ending in one terminal
    period or exclamation mark. Anything else — acknowledgements, past-run
    descriptions, questions, negation, exclusion, contradictions, additional
    or multiline clauses — returns False.
    """
    match = _SIM_DIRECTIVE_RE.fullmatch(body.strip())
    return match is not None and match.group("login").casefold() == bot_login.casefold()


_SIM_REF_RE = re.compile(r"farm/(?P<issue>[0-9]+)-(?P<slug>[a-z0-9]+(?:-[a-z0-9]+)*)")
_SIM_HEAD_SHA_RE = re.compile(r"[0-9a-f]{40}")


def is_simulator_directive(event: Event, policy: RepositoryPolicy) -> bool:
    """True when a mention event is an exact, maintainer-authorized simulator directive."""
    if event.event_type not in _SIM_COMMENT_EVENTS:
        return False
    if not policy.is_maintainer(event.actor):
        return False
    comment = event.payload.get("comment")
    body = comment.get("body") if isinstance(comment, dict) else None
    return isinstance(body, str) and parse_sim_directive(policy.bot_login, body)


_DISPATCH_LOCKS: dict[tuple[str, str, str], threading.Lock] = {}
_DISPATCH_LOCKS_GUARD = threading.Lock()


def _dispatch_lock(repo: str, workflow: str, ref: str) -> threading.Lock:
    key = (repo, workflow, ref)
    with _DISPATCH_LOCKS_GUARD:
        lock = _DISPATCH_LOCKS.get(key)
        if lock is None:
            lock = _DISPATCH_LOCKS[key] = threading.Lock()
        return lock


def _workflow_name_matches(workflow: Any, expected: str) -> bool:
    return isinstance(workflow, str) and (workflow == expected or workflow.rsplit("/", 1)[-1] == expected)


def _validate_run_identity(result: dict[str, Any], ref: str, workflow: str) -> None:
    run_id = result.get("run_id")
    if not isinstance(run_id, int) or run_id <= 0:
        raise ActionDenied(f"proxy returned an invalid simulator run id: {run_id!r}")
    if result.get("requested_ref") != ref:
        raise ActionDenied(f"proxy dispatched a different ref than requested: {result.get('requested_ref')!r}")
    if not isinstance(result.get("ref"), str) or not result["ref"]:
        raise ActionDenied(f"proxy did not report the actual dispatch ref: {result.get('ref')!r}")
    if not _workflow_name_matches(result.get("workflow"), workflow):
        raise ActionDenied(
            f"proxy returned a run outside the configured simulator workflow: {result.get('workflow')!r}"
        )


@dataclass(slots=True)
class ActionGateway:
    database: Database
    event: Event
    policy: RepositoryPolicy
    default_branch: str
    proxy: ProxyClient | None

    def classify(self, primary: str, priority: str | None, area: list[str], rationale: str) -> str:
        if self.event.event_type != "issues.opened":
            raise ActionDenied("classification is only permitted for newly opened issues")
        self._require_unique("classify")
        allowed_primary = {"bug", "documentation", "question", "enhancement", "duplicate", "invalid", "upstream"}
        if primary not in allowed_primary:
            raise ActionDenied(f"unsupported primary classification: {primary}")
        if priority is not None and priority not in {"p0", "p1", "p2", "p3"}:
            raise ActionDenied(f"unsupported priority: {priority}")
        labels = [primary, *(f"area:{item}" for item in area)]
        if priority is not None:
            labels.append(f"priority:{priority}")
        arguments = {"primary": primary, "priority": priority, "area": area, "rationale": rationale, "labels": labels}
        return self._mutate(
            Capability.LABEL,
            "classify",
            arguments,
            lambda proxy: proxy.add_labels(self.event.repo, self.event.issue_number, labels),
        )

    def post_comment(self, body: str) -> str:
        normalized = body.strip()
        if not normalized:
            raise ActionDenied("comment body is empty")
        self._require_unique("comment")
        if self.event.event_type == "issues.opened":
            classification = self.database.find_action(self.event.delivery_id, "classify")
            if classification is None or classification.state not in {"proposed", "applied"}:
                raise ActionDenied("a new issue must be successfully classified before it is commented on")
        if is_simulator_directive(self.event, self.policy):
            dispatches = self._sim_dispatches()
            reads = [
                action
                for action in self.database.actions_for_delivery(self.event.delivery_id)
                if action.kind.startswith("sim_result")
            ]
            if not dispatches or not reads:
                raise ActionDenied("a simulator directive requires dispatch_sim then sim_result before any comment")
            if self.policy.permits(Capability.DISPATCH_SIM):
                for dispatch in dispatches:
                    run_id = (dispatch.result or {}).get("run_id") if dispatch.result is not None else None
                    run = (
                        self.database.workflow_run(self.event.repo, self.event.issue_number, run_id)
                        if isinstance(run_id, int)
                        else None
                    )
                    if run is None or run["status"] != "completed":
                        raise ActionDenied(
                            f"simulator run {run_id!r} has not completed; poll get_simulator_result before commenting"
                        )
        return self._mutate(
            Capability.COMMENT,
            "comment",
            {"body": normalized},
            lambda proxy: proxy.post_comment(self.event.repo, self.event.issue_number, normalized),
        )

    def search_issues(self, query: str) -> str:
        if self.proxy is None:
            raise ActionDenied("GitHub proxy is required to search issues")
        return json.dumps(self.proxy.search_issues(self.event.repo, query), sort_keys=True)

    def dispatch_sim(self, ref: str, head_sha: str | None) -> str:
        self._require_sim_context("dispatch simulation")
        self._require_sim_ref(ref, head_sha)
        kind = f"dispatch_sim:{ref}:{head_sha or 'default'}"
        self._require_unique(kind)

        def execute(proxy: ProxyClient) -> dict[str, Any]:
            with _dispatch_lock(self.event.repo, self.policy.sim_workflow, ref):
                result = proxy.dispatch_sim(
                    self.event.repo, self.event.issue_number, self.policy.sim_workflow, ref, head_sha
                )
                _validate_run_identity(result, ref, self.policy.sim_workflow)
                claimed = self.database.claim_workflow_run(
                    self.event.repo,
                    self.event.issue_number,
                    self.policy.sim_workflow,
                    result["ref"],
                    result["run_id"],
                    str(result["url"]),
                    str(result["status"]),
                    result.get("conclusion"),
                )
                if not claimed:
                    raise ActionDenied(f"workflow run {result['run_id']} was already claimed by a previous dispatch")
            return result

        return self._mutate(
            Capability.DISPATCH_SIM,
            kind,
            {"workflow": self.policy.sim_workflow, "ref": ref, "head_sha": head_sha},
            execute,
        )

    def sim_result(self, run_id: int) -> str:
        if not isinstance(run_id, int) or run_id <= 0:
            raise ActionDenied(f"invalid simulator run id: {run_id}")
        self._require_sim_context("read simulation results")
        if not self._sim_dispatches():
            raise ActionDenied("no recorded dispatch precedes a simulator result read")
        kind = f"sim_result:{run_id}"

        def execute(proxy: ProxyClient) -> dict[str, Any]:
            recorded = self.database.workflow_run(self.event.repo, self.event.issue_number, run_id)
            if recorded is None or recorded["workflow"] != self.policy.sim_workflow:
                raise ActionDenied(
                    f"no recorded dispatch associates simulator run {run_id} with "
                    f"{self.event.repo}#{self.event.issue_number}"
                )
            result = proxy.sim_result(self.event.repo, run_id)
            if result.get("run_id") != run_id:
                raise ActionDenied(f"proxy returned a different run than requested: {result.get('run_id')!r}")
            if not _workflow_name_matches(result.get("workflow"), self.policy.sim_workflow):
                raise ActionDenied(
                    f"run {run_id} is not from the configured simulator workflow: {result.get('workflow')!r}"
                )
            if result.get("ref") != recorded["ref"]:
                raise ActionDenied(
                    f"run {run_id} is not on the recorded ref {recorded['ref']!r}: {result.get('ref')!r}"
                )
            self.database.update_workflow_run_status(run_id, str(result["status"]), result.get("conclusion"))
            return result

        return self._mutate(Capability.READ_SIM, kind, {"run_id": run_id}, execute, upsert=True)

    def _require_sim_ref(self, ref: str, head_sha: str | None) -> None:
        farm = _SIM_REF_RE.fullmatch(ref) if isinstance(ref, str) else None
        if farm is None and ref != self.default_branch:
            raise ActionDenied(
                f"simulation ref must be the default branch ({self.default_branch}) "
                f"or farm/{self.event.issue_number}-<slug>: {ref!r}"
            )
        if farm is not None and farm.group("issue") != str(self.event.issue_number):
            raise ActionDenied(
                f"simulation ref must be the default branch ({self.default_branch}) "
                f"or farm/{self.event.issue_number}-<slug>: {ref!r}"
            )
        if farm is not None and head_sha is None:
            raise ActionDenied(
                f"the issue-scoped simulation ref requires its exact 40-character HEAD SHA: {head_sha!r}"
            )
        if head_sha is not None and (not isinstance(head_sha, str) or _SIM_HEAD_SHA_RE.fullmatch(head_sha) is None):
            raise ActionDenied(f"invalid simulator HEAD SHA (expected [0-9a-f]{{40}}): {head_sha!r}")

    def _sim_dispatches(self) -> list[Action]:
        return [
            action
            for action in self.database.actions_for_delivery(self.event.delivery_id)
            if action.kind.startswith("dispatch_sim") and action.state in {"applied", "proposed"}
        ]

    def _require_sim_context(self, action: str) -> None:
        if not self.policy.is_maintainer(self.event.actor):
            raise ActionDenied(f"actor is not authorized to {action}: {self.event.actor}")
        if self.event.event_type not in _SIM_COMMENT_EVENTS:
            raise ActionDenied(f"{action} requires an explicit mention comment")
        comment = self.event.payload.get("comment")
        body = comment.get("body") if isinstance(comment, dict) else None
        if not isinstance(body, str) or not parse_sim_directive(self.policy.bot_login, body):
            raise ActionDenied(f"comment does not unambiguously direct {action}")

    def _require_unique(self, kind: str) -> None:
        if self.database.find_action(self.event.delivery_id, kind) is not None:
            raise ActionDenied(f"{kind} is already recorded for delivery {self.event.delivery_id}")

    def _record(self, kind: str, arguments: dict[str, Any], state: str) -> int:
        try:
            return self.database.add_action(self.event, kind, arguments, state)
        except ActionConflict as exc:
            raise ActionDenied(str(exc)) from exc

    def _mutate(
        self,
        capability: Capability,
        kind: str,
        arguments: dict[str, Any],
        operation: Any,
        *,
        upsert: bool = False,
    ) -> str:
        existing = self.database.find_action(self.event.delivery_id, kind) if upsert else None
        if not self.policy.permits(capability):
            action_id = existing.id if existing is not None else self._record(kind, arguments, "proposed")
            return json.dumps({"action_id": action_id, "state": "proposed", "mode": self.policy.mode}, sort_keys=True)
        if self.proxy is None:
            raise ActionDenied(f"GitHub proxy is required to apply {kind}")
        action_id = existing.id if existing is not None else self._record(kind, arguments, "proposed")
        try:
            result = operation(self.proxy)
        except Exception as exc:
            failure = {"error": f"{type(exc).__name__}: {exc}"}
            self.database.update_action(action_id, "failed", failure)
            raise
        self.database.update_action(action_id, "applied", result)
        return json.dumps({"action_id": action_id, "state": "applied", "result": result}, sort_keys=True)
