# Documentation guide and scope

Serval is evolving quickly, so its documentation is deliberately organized by **audience** and **confidence**, rather than presenting every inherited page as equally current.

## Start here

| Need | Read |
| --- | --- |
| Understand what is different and whether it is ready for your machine | [README](../README.md) and [Feature status](Feature_Status.md) |
| Move an existing Klipper/Kalico installation | [Quickstart](Quickstart.md), then [Config migration](Config_Migration.md) |
| Write or review a Serval motion configuration | [Motion configuration reference](Config_Reference_Motion.md) |
| Understand internals or contribute | [Architecture](Architecture.md) and [Developer guide](Development.md) |
| Use an inherited Kalico/Klipper feature | [Overview](Overview.md) and the linked feature reference |

## Authority and freshness

1. **Executable code and tests are authoritative** for current behavior.
2. **Reference pages** document accepted configuration, commands, and user workflows. They should state defaults, units, constraints, prerequisites, failure modes, and a safe example.
3. **Feature status** states the confidence tier and known limits. A feature being present in a reference does not imply it is safe for every printer.
4. **`docs/rewrite/`, `docs/plans/`, `docs/human-spec/`, and `docs/superpowers/`** are engineering design, investigation, or planning records. They may explain intent and history, but are not normative installation instructions.
5. Many pages are inherited from Kalico/Klipper. They remain useful for unchanged subsystems, but users operating the new motion path must follow the Serval quickstart, migration, and motion-reference pages where they differ.

## Keeping pages open to iteration

Documentation should be easy to revise without becoming vague. Make a focused change when a measurement, test, configuration parser, command handler, or supported-hardware result changes. Prefer precise statements such as “validated in simulator on 2026-07” over broad claims such as “supported.” Link evidence—tests, source locations, issue/PR, or reproducible command—when describing a limit or measured behavior.

When a claim is uncertain, document the boundary and verification state in [Feature status](Feature_Status.md), not as an unstated assumption in an installation procedure. Never invent a safe value, board support, or recovery action. Safety-critical uncertainty should stop the procedure and direct the reader to the known supported path.

## Review checklist

- Is the page named in [Overview](Overview.md) and site navigation where appropriate?
- Are commands copyable, scoped to the correct host/firmware environment, and explicit about destructive or privilege-requiring steps?
- Are units, defaults, validation bounds, compatibility impact, and rollback/recovery behavior stated?
- Do links, anchors, and code paths resolve under `mkdocs build --strict`?
- Does a status claim distinguish real hardware, simulator, and exploratory work?

For contributor mechanics and validation commands, see [Developer guide](Development.md).
