# AGC-in-Rust — Agent Workflow

The project uses an **agile, sub-agents with parallel execution workflow** with five roles. Each
role corresponds to a specialised Claude agent in `.claude/agents/` or a skill
in `.claude/skills/`. Roles run concurrently whenever their dependencies allow —
the system is designed so that specification, architecture, development, and
testing overlap rather than execute in strict sequence.

> Orchestration reference. This document is read by the orchestrator (main session)
> when deciding how to route and parallelise work. It is intentionally **not**
> loaded into every subagent — the coding rules each agent must follow live in
> `CLAUDE.md`, which auto-loads into all subagents.

## Roles

| Role | Responsibility | Primary inputs | Primary outputs |
|---|---|---|---|
| **Analyst / Spec** | Read the AGC assembler source (`docs/agc-source/*.agc`) and PDF references, produce a functional specification per component | Comanche055 `.agc` files, AGC book, CIS book | `specs/<module>.md` — API, invariants, scale factors, test cases |
| **Architect** | Decide the Rust shape: module boundaries, types, ownership, interrupt model, restart safety | Functional specs, `docs/architecture.md`, ADRs | Architecture decisions, module layout, trait signatures |
| **Developer (Dev)** | Implement the Rust code that satisfies the spec and the architecture | Spec + architecture + existing code | Rust source files in `agc-core/`, `agc-sim/` |
| **Tester** | Write unit tests, scenario tests, VirtualAGC fixture checks | Spec test cases, implementation | Test files in `agc-core/src/**/tests`, `agc-test/tests/` |
| **PO / Validation** | Cross-check the finished implementation against the AGC source and PDF references | `.claude/skills/validation/SKILL.md`, local AGC source cache | Validation report (CONFIRMED / WRONG / APPROXIMATE / NOT FOUND) |

## Flow

```uml
Analyst/Spec  Arch           Dev              Test             PO/Validation
     │         │              │                │                     │
     │─ Spec ──▶              │                │                     │
     │◀ · · · ·│              │                │                     │
     │         │─ Design ────▶│ + write test ─▶|                     │
     │         │─ Develop ───▶│                │                     │
     │         │              │                │                     │
     │         │              │── Test ───────▶│                     │
     │         │              │◀· · · · · · · ·│                     │
     │         │              │                │                     │
     │         │              │                │-── AGC / PDF ──────▶│
     │         │              │                │                     │
     │◀ · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·│
     │· · · · ·▶ (revised spec feeds back in)                        │
```

Solid arrows are **hand-offs** (data flows forward). Dashed arrows are
**feedback loops** (something downstream caused upstream rework).

## Key practices

- **Sub-agents run in parallel.** Multiple developer agents can work on
  independent modules simultaneously (e.g. `math/kepler.rs`, `math/lambert.rs`,
  `navigation/conics.rs`. Dispatch them in a single message with multiple `Agent` tool
  calls so they execute concurrently, not sequentially.
- **Spec ↔ Analyst is bidirectional.** The Architect and the Developer can
  push back to the Analyst when the spec is incomplete or contradictory. The
  Analyst revises the spec and the downstream work re-runs.
- **Test is co-developed with Dev, not after.** The Tester writes test cases
  in parallel with the implementation. Unit tests in the Rust file are the
  Developer's responsibility; scenario and fixture tests belong to the Tester.
- **Validation (PO) is the final gate.** Before a milestone is marked complete,
  run the validation skill against the AGC source. Any `WRONG` item blocks
  merge; `APPROXIMATE` items must be documented as intentional deviations.
- **The AGC assembler source and PDFs are the single source of truth.** When
  in doubt, read `docs/agc-source/<file>.agc` or the AGC book (Frank O'Brien)
  and update `docs/agc-reference-constants.md` with the new finding.
- **Tracking is mandatory.** Every completed task updates
  `transformation/progress.md`, `transformation/tasks.md`, and the relevant
  spec status in `transformation/specifications.md`. Also update the README
  files in corresponding to the task.

## When to use which agent

| Trigger | Agent |
|---|---|
| New AGC routine to port | **analyst-reengineer** — read the `.agc` file, write a spec |
| Architectural decision (new module, new trait, shared state strategy) | **architect** — produce an ADR and update `docs/architecture.md` |
| Implement a spec / refactor existing code | **developer** — follow the spec, match existing conventions |
| Failing build, clippy warning, test regression, no_std violation | **debugger** — diagnose and fix |
| Write tests for new or existing code | **tester** — unit, scenario, fixture |
| Review a Rust change for correctness, API design, `no_std` safety | **code-review** — structured review |
| Verify an implementation against the AGC source | **validation** skill — CONFIRMED / WRONG / APPROXIMATE |
| Research a question that spans multiple files | **Explore** — targeted or deep codebase search |
| Design a non-trivial implementation before coding | **Plan** — step-by-step plan |
| Housekeeping: workspace, Cargo, CI, feature flags | **workspace-maintenance** skill |

## Parallelism rules of thumb

1. Independent modules → launch dev agents **in parallel** in one message.
2. Dependent modules → run agents **sequentially**, feeding the output of one
   into the prompt of the next.
3. Research → always prefer parallel exploration (multiple Explore agents
   or one agent with parallel tool calls).
4. Validation of a milestone → run **one validator per sub-system** in
   parallel (e.g. math modules, control modules, guidance modules separately).
5. Never spawn a new agent for a task a single tool call can solve — prefer
   `Read` / `Grep` / `Glob` directly when the target is known.
