# Acceptance Auditor Review Prompt — Input Shaper Port

Use the acceptance-auditor role from `bmad-quick-dev`.

Read these context files first:

```text
_bmad-output/implementation-artifacts/spec-input-shaper-port.md
_bmad-output/specs/spec-input-shaper-port/SPEC.md
_bmad-output/specs/spec-input-shaper-port/code-map.md
```

Then review the diff from baseline:

```text
bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6
```

Diff command:

```sh
git diff bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6
```

Also inspect these untracked artifacts:

```text
_bmad-output/implementation-artifacts/spec-input-shaper-port.md
_bmad-output/specs/spec-input-shaper-port/.decision-log.md
_bmad-output/specs/spec-input-shaper-port/SPEC.md
_bmad-output/specs/spec-input-shaper-port/code-map.md
```

Do not use graphify.

Audit whether the implementation satisfies every task, acceptance criterion, and frozen constraint. Pay special attention to:

- PA-only byte identity and zero-support fit-grid reuse.
- Smooth shaper output matching the `ShapedSignal` oracle.
- Two-sided convolution windows: past history, future lookahead, live-edge hold-back, and no internal clamp.
- Axis-agnostic processor application.
- Runtime `frequency_hz` fail-loud validation.
- Legacy stack marked but not deleted.

Return findings first, ordered by severity. For each finding, classify whether it is likely `intent_gap`, `bad_spec`, `patch`, `defer`, or `reject`, with a short rationale.
