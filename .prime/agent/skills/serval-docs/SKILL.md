---
name: serval-docs
description: Author, audit, or review user/developer documentation for Serval. Use when documenting motion features, configuration, G-code, hardware support, releases, or changes to this repository's docs.
---
# Serval documentation

Read `docs/Documentation_Guide.md` and `AGENTS.md`. Code and tests are authoritative. Serval-specific pages override inherited Kalico/Klipper pages; keep the support tier explicit: solid, simulator/bench verified, or exploratory.

## Ownership

- topology/limits/axes/motors/processors: `docs/Config_Reference_Motion.md`
- migration/compatibility: `Config_Migration.md`, `Config_Changes.md`
- commands/status/API: `G-Codes.md`, `Status_Reference.md`, `API_Server.md`
- support evidence/limits: `Feature_Status.md`
- architecture/developer behavior: `Architecture.md`, `Development.md`

Document units, defaults, bounds, prerequisites, failure modes, compatibility impact, and recovery. Do not promise board safety based only on compilation or simulation. Do not promote historical `docs/rewrite/` plans to operator instructions.

Add every public page to both `docs/Overview.md` and `docs/_kalico/mkdocs.yml`. Validate:

```bash
./scripts/ci.sh docs
git diff --check
```
