# AGC-in-Rust: Transformation Tracking

This directory tracks the systematic transformation of Comanche055 (Command Module) AGC assembly code into idiomatic Rust.

## Tracking Files

| File | Purpose |
|------|---------|
| [progress.md](progress.md) | Overall transformation progress dashboard |
| [tasks.md](tasks.md) | Active tasks, priorities, and assignment tracking |
| [specifications.md](specifications.md) | Status of all component specs |
| [infrastructure.md](infrastructure.md) | Build system, tooling, dependencies, CI/CD |
| [validation.md](validation.md) | Test coverage, VirtualAGC validation, benchmarks |
| [decisions.md](decisions.md) | Architecture decision records (ADRs) |
| [blockers.md](blockers.md) | Current blockers and unresolved questions |

## Agent Workflow

Work proceeds through a pipeline of specialized agents defined in `.claude/agents/`:

1. **analyst** — reads AGC assembly source and reference docs, produces functional specs per component
2. **architect** — designs Rust architecture based on functional specs
3. **developer** — implements Rust code following architect guidelines and analyst specs
4. **tester** — writes unit tests (per public interface) and system tests

Each agent reads outputs from the prior stage. The analyst's functional specs are the primary input to the architect and developer.

## Creating a New Component

1. **Spec Phase**: Create spec in `../specs/` using appropriate template
2. **Track**: Add entry to [specifications.md](specifications.md) and [tasks.md](tasks.md)
3. **Implement**: Use the developer agent with the spec
4. **Validate**: Update [validation.md](validation.md) with test results and VirtualAGC comparison
5. **Close**: Update [progress.md](progress.md) and mark task complete

## Status Meanings

| Status | Meaning |
|---|---|
| Not Started | Spec not yet created |
| Spec In Progress | Writing/reviewing spec |
| Spec Complete | Spec approved, ready for implementation |
| In Progress | Code being written |
| Testing | Tests being written or debugged |
| Blocked | Waiting on dependency or question resolution |
| Complete | Implemented, tested, reviewed, integrated |

## Quick Commands

```bash
# View overall progress
cat transformation/progress.md

# See active tasks
grep "In Progress" transformation/tasks.md

# Check blockers
cat transformation/blockers.md

# Find incomplete specs
grep "Not Started\|In Progress" transformation/specifications.md
```

## Related Documentation

- [specs/README.md](../specs/README.md) — Spec templates and workflow
- [docs/architecture.md](../docs/architecture.md) — Full software architecture
- [docs/testing.md](../docs/testing.md) — VirtualAGC integration test strategy
- [docs/optimization.md](../docs/optimization.md) — Rust Embedded Book compliance
- [AGENTS.md](../AGENTS.md) — Coding conventions
- [WORKSPACE.md](../WORKSPACE.md) — Development environment setup
