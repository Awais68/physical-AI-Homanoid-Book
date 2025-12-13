# Physical AI Monitoring Constitution

## Core Principles

### I. Test-First (NON-NEGOTIABLE)
TDD mandatory: Tests written → User approved → Tests fail → Then implement. Red-Green-Refactor cycle strictly enforced. All functionality MUST have corresponding tests before implementation.

### II. Library-First Design
Every feature starts as a standalone library. The edge_health package MUST be self-contained, independently testable, and documented. Clear purpose required - no organizational-only modules.

### III. CLI Interface
Every library exposes functionality via CLI or ROS2 command-line tools. Text in/out protocol: stdin/args → stdout, errors → stderr. Support JSON + human-readable formats for all APIs.

### IV. Integration Testing
Focus areas requiring integration tests: ROS2 topic communication, hardware sensor integration, multi-node deployment scenarios, inter-service communication.

### V. Observability
Text I/O ensures debuggability. Structured logging required for all health metrics, alerts, and system events. All logs MUST be stored locally and accessible for dashboard consumption.

### VI. Graceful Degradation
System MUST handle hardware sensor failures without crashing. Fallback values or degraded modes MUST be documented. 95% of failures MUST be handled gracefully.

## Quality Gates

- All PRs must pass unit tests before merge
- Integration tests required for ROS2 node changes
- Code coverage minimum: 80%
- All ROS2 interfaces must be documented

## Governance

Constitution supersedes all other practices. Amendments require documentation and team approval.

**Version**: 1.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12
