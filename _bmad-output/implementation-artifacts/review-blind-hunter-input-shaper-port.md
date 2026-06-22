# Blind Hunter Review Prompt — Input Shaper Port

Use the `bmad-review-adversarial-general` skill.

You receive only the diff, not the implementation spec or project context. Review for defects, regressions, suspicious logic, incomplete behavior, and test gaps visible from the patch alone.

Baseline commit:

```text
bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6
```

Construct the diff from the workspace root:

```sh
git diff bfdf06e2b5cb10b5b4a8d8e412c30d08731605a6 -- \
  rust/motion-engine/src/config/tests.rs \
  rust/motion-engine/src/lowering.rs \
  rust/motion-engine/src/lowering/tests.rs \
  rust/motion-engine/src/stream.rs \
  rust/motion-engine/src/stream/tests.rs \
  rust/motion-engine/src/stream_planner/tests.rs \
  rust/trajectory/src/beta.rs \
  rust/trajectory/src/emit_shaped.rs \
  rust/trajectory/src/emit_shaped/tests.rs \
  rust/trajectory/src/kernel/tests.rs \
  rust/trajectory/src/lib.rs \
  rust/trajectory/src/plan_velocity/tests.rs \
  rust/trajectory/src/post_processor.rs \
  rust/trajectory/src/post_processor/tests.rs \
  rust/trajectory/src/shaper.rs \
  rust/trajectory/src/streaming/state.rs \
  rust/trajectory/src/streaming/tests.rs \
  rust/trajectory/tests/binding_report.rs \
  rust/trajectory/tests/follower_rows.rs
```

Do not inspect files outside this diff. Return findings only, ordered by severity, with file and line references when possible. If no findings, say so and mention residual risk.
