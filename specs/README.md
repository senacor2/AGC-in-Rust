# Specifications

This directory contains functional specifications for porting Comanche055 (Command Module) AGC assembly to idiomatic Rust. Each spec is the handoff document between the **analyst-reengineer** agent (who reads the AGC source) and the **developer** agent (who writes the Rust).

## Status Overview

See [`transformation/specifications.md`](../transformation/specifications.md) for the full tracking table.

**Milestone 1 — complete:**

| Component | Rust module | Status |
|---|---|---|
| Types (`CduAngle`, `Vec3`, `Mat3x3`, `Met`, `DeltaV`) | `agc-core/src/types/` | Complete |
| HAL traits (`AgcHardware` + 8 sub-traits) | `agc-core/src/hal/` | Complete |
| Executive (7-slot job table, priority dispatch, alarm 1202) | `agc-core/src/executive/scheduler.rs` | Complete |
| Waitlist (8-slot delta-time chain, T3RUPT dispatch) | `agc-core/src/executive/waitlist.rs` | Complete |
| Restart protection (phase tables, 6 groups) | `agc-core/src/executive/restart.rs` | Complete |
| Alarm system (1202, 1210, 1211) | `agc-core/src/services/alarm.rs` | Complete |
| Fresh-start / restart sequences | `agc-core/src/services/fresh_start.rs` | Complete |
| Simulated HAL + DSKY TUI | `agc-sim/` | Complete |

## Spec-Driven Workflow

```
AGC source
    └─► analyst-reengineer  →  spec file in specs/
            └─► architect   →  docs/architecture.md update
                    └─► developer    →  agc-core Rust code
                            └─► tester  →  agc-test tests
                                    └─► update agc-sim TUI
```

1. **analyst-reengineer** reads `docs/AGC Symbolic Listing.md` and the Apollo-11 GitHub source, then writes a spec here.
2. **architect** reviews the spec against `docs/architecture.md` and adds any necessary design notes.
3. **developer** implements the spec. Every new observable state must also be wired into `agc-sim` (see [agc-sim/README.md](../agc-sim/README.md)).
4. **tester** writes unit tests, scenario tests (using `SimHardware`), and spec-linked tests.

## What a Spec Contains

A spec file documents the following before any Rust is written:

- **AGC source reference** — file, routine, and line range in Comanche055
- **Behavior summary** — what the routine/module does, in plain language
- **Rust API** — proposed type signatures, ownership model, module path
- **Scale factors** — AGC fixed-point units → `f64` SI units (must be explicit)
- **Invariants** — pre/post-conditions, restart safety requirements, no-heap constraints
- **Test cases** — at least 3, ideally from a VirtualAGC run
- **DSKY / sim impact** — what new state or events `agc-sim` needs to expose

### AGC source reference format

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc
Routine:    KEPRTN
Lines:      234–456
```

## AGC Quick Reference (Comanche055)

### Memory

| Region | Size | Purpose |
|---|---|---|
| Erasable (RAM) | 2 048 words | Defined in `ERASABLE_ASSIGNMENTS.agc` |
| Fixed (ROM) | 36 864 words | 36 switchable banks |
| Channels | 12-bit registers | I/O-mapped hardware |

### Scaling Conventions

All AGC arithmetic is scaled fixed-point. **Document scale factors in every spec.**

| Quantity | AGC unit | Scale | `f64` SI unit |
|---|---|---|---|
| Position | words | B-28 (1 unit = 2²⁸ m) | metres |
| Velocity | words | B-7 (1 unit = 2⁷ m/s) | m/s |
| Time | centiseconds | B-14 | seconds |
| CDU angle | counts | 2¹⁵ counts = full revolution | radians |

### Common Patterns in `agc-core`

| AGC pattern | Rust equivalent |
|---|---|
| EXECUTIVE cooperative scheduler | `executive::Executive`, `JobEntry` |
| WAITLIST delta-time chain | `executive::Waitlist` |
| PHASE_TABLE restart protection | `executive::RestartProtection` |
| AGC interpretive language | Eliminated — plain `f64` Rust functions (ADR-001) |
| Shared state (ISR + foreground) | `Mutex<RefCell<T>>` via `cortex_m::interrupt::free` |

## `agc-sim` Integration Rule

Every spec for a new component **must** include an "agc-sim impact" section:

```
## agc-sim Impact

- DskyDisplayState: add field `imu_aligned: bool`
- SimLog: emit `.info("IMU coarse-align complete")` on transition
- dsky_terminal.rs: render a FINE ALIGN light in the lights row
- dsky_demo.rs: no new keyboard bindings needed
```

The developer is responsible for landing both the `agc-core` implementation and the `agc-sim` update in the same PR.

## Next Specs to Write

These are the highest-priority unstarted components (see `transformation/specifications.md` for the full list):

1. `specs/math-linalg.md` — vector/matrix ops (`math/linalg`) replacing AGC interpretive VLOAD/DOT/CROSS
2. `specs/math-trig.md` — trig wrappers (`math/trig`) with AGC domain conventions
3. `specs/math-kepler.md` — universal-variable Kepler solver (`CONIC_SUBROUTINES.agc/KEPRTN`)
4. `specs/services-average-g.md` — 2-second PIPA integration cycle (`SERVICER207.agc`)
5. `specs/navigation-state-vector.md` — `StateVector`, coordinate frames, `ERASABLE_ASSIGNMENTS.agc`

## Related Documentation

| File | Purpose |
|---|---|
| [`docs/architecture.md`](../docs/architecture.md) | Type conventions, module boundaries, HAL design |
| [`docs/testing.md`](../docs/testing.md) | VirtualAGC fixture capture strategy, tolerances |
| [`AGENTS.md`](../AGENTS.md) | Coding rules, embedded constraints, no_std rules |
| [`transformation/specifications.md`](../transformation/specifications.md) | Status of all specs |
| [`agc-sim/README.md`](../agc-sim/README.md) | DSKY TUI layout, keyboard map, sim modules |
