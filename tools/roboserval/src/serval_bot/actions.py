from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from serval_bot.database import Database, Event
from serval_bot.policy import Capability, RepositoryPolicy
from serval_bot.proxy_client import ProxyClient


class ActionDenied(RuntimeError):
    pass


@dataclass(slots=True)
class ActionGateway:
    database: Database
    event: Event
    policy: RepositoryPolicy
    default_branch: str
    proxy: ProxyClient | None

    def classify(self, primary: str, priority: str | None, area: list[str], rationale: str) -> str:
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
        if not self.policy.is_maintainer(self.event.actor):
            raise ActionDenied(f"actor is not authorized to dispatch simulation: {self.event.actor}")
        if ref != self.default_branch and not ref.startswith("farm/"):
            raise ActionDenied(f"simulation ref is outside the bot namespace: {ref}")

        def execute(proxy: ProxyClient) -> dict[str, Any]:
            result = proxy.dispatch_sim(self.event.repo, self.policy.sim_workflow, ref, head_sha)
            self.database.record_workflow_run(
                self.event.repo,
                self.event.issue_number,
                self.policy.sim_workflow,
                ref,
                int(result["run_id"]),
                str(result["url"]),
                str(result["status"]),
                result.get("conclusion"),
            )
            return result

        return self._mutate(
            Capability.DISPATCH_SIM,
            "dispatch_sim",
            {"workflow": self.policy.sim_workflow, "ref": ref, "head_sha": head_sha},
            execute,
        )

    def sim_result(self, run_id: int) -> str:
        if not self.policy.is_maintainer(self.event.actor):
            raise ActionDenied(f"actor is not authorized to read simulation runs: {self.event.actor}")
        if not self.policy.permits(Capability.READ_SIM):
            raise ActionDenied(f"repository mode denies simulation results: {self.policy.mode}")
        if self.proxy is None:
            raise ActionDenied("GitHub proxy is required to read simulation runs")
        result = self.proxy.sim_result(self.event.repo, run_id)
        self.database.record_workflow_run(
            self.event.repo,
            self.event.issue_number,
            self.policy.sim_workflow,
            self.default_branch,
            run_id,
            str(result["url"]),
            str(result["status"]),
            result.get("conclusion"),
        )
        return json.dumps(result, sort_keys=True)

    def _mutate(
        self,
        capability: Capability,
        kind: str,
        arguments: dict[str, Any],
        operation: Any,
    ) -> str:
        if not self.policy.permits(capability):
            action_id = self.database.add_action(self.event, kind, arguments, "proposed")
            return json.dumps({"action_id": action_id, "state": "proposed", "mode": self.policy.mode}, sort_keys=True)
        if self.proxy is None:
            raise ActionDenied(f"GitHub proxy is required to apply {kind}")
        action_id = self.database.add_action(self.event, kind, arguments, "proposed")
        try:
            result = operation(self.proxy)
        except Exception as exc:
            failure = {"error": f"{type(exc).__name__}: {exc}"}
            self.database.update_action(action_id, "failed", failure)
            raise
        self.database.update_action(action_id, "applied", result)
        return json.dumps({"action_id": action_id, "state": "applied", "result": result}, sort_keys=True)
