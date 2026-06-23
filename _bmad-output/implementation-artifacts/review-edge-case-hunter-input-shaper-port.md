# Edge Case Hunter Review Prompt — Input Shaper Port

Use the `bmad-review-edge-case-hunter` skill.

Review the implementation for unhandled edge cases and boundary conditions. You may inspect the project, but do not use graphify. Focus on stream boundaries, shaper support windows, history/lookahead availability, axis counts, PA/shaper ordering, numerical finiteness, and commit/force behavior.

Baseline commit:

```text
bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6
```

Primary diff command:

```sh
git diff bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6
```

Untracked review artifacts:

```text
_bmad-output/implementation-artifacts/spec-input-shaper-port.md
_bmad-output/specs/spec-input-shaper-port/.decision-log.md
_bmad-output/specs/spec-input-shaper-port/SPEC.md
_bmad-output/specs/spec-input-shaper-port/code-map.md
```

Return only edge-case findings. For each finding, include the concrete path, trigger condition, expected bad behavior, and a minimal test that would expose it.
