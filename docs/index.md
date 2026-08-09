---
hide:
  - toc
title: Serval Documentation
---

# Serval documentation

Serval is an experimental fork of Kalico that replaces the conventional motion stack with a Rust streaming planner and MCU-side trajectory execution. It is not a drop-in firmware update: the host native modules, configuration model, and MCU protocol move together.

> **Read before flashing:** only the support tiers in [Feature status](Feature_Status.md) describe what has been exercised. Confirm your board is supported, preserve a known-good configuration and firmware image, and follow [Quickstart](Quickstart.md) exactly when moving a printer.

## Choose a path

- **Evaluate or install Serval:** [README](../README.md) → [Quickstart](Quickstart.md) → [Config migration](Config_Migration.md).
- **Configure the motion system:** [Motion configuration reference](Config_Reference_Motion.md).
- **Understand limits and safety boundaries:** [Feature status](Feature_Status.md) and [Installation: host memory requirements](Installation.md#host-memory-requirements).
- **Understand the implementation or contribute:** [Architecture](Architecture.md) and [Developer guide](Development.md).
- **Use an inherited Kalico subsystem:** begin at [Overview](Overview.md). Those pages are retained for unchanged functionality; Serval-specific instructions take precedence for motion.

[Documentation guide](Documentation_Guide.md) explains page authority, scope, and how this living documentation is maintained.
