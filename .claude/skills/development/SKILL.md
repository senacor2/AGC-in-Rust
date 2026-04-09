---
name: development
description: Use when implementing, refactoring, or extending Rust code in AGC-in-Rust — modules, HAL traits, navigation functions, guidance algorithms, interrupt handlers, Cargo changes, or embedded no_std patterns.
argument-hint: Describe the Rust feature, refactor, or AGC component to implement
---

# Rust Development — AGC-in-Rust

## When To Use

- Implement a new AGC component from a spec in `specs/`
- Refactor existing `agc-core` code without changing behavior
- Add or extend HAL sub-traits, newtypes, or math functions
- Improve error handling, tests, or Cargo configuration
- Build embedded bare-metal code for the `thumbv7em-none-eabihf` target
- Update `agc-sim` to expose new state or events in the DSKY TUI

## Procedure

### Phase 0 — Orientation (read before writing any code)

Read these files to understand the current state, constraints, and what's already done:

| File | What you learn |
|---|---|
| `docs/architecture.md` | Type conventions, module boundaries, HAL design, ADRs |
| `specs/README.md` | Spec-driven workflow, scaling conventions, agc-sim integration rule |
| `transformation/progress.md` | Which milestones are done, current test counts, what's next |
| `transformation/specifications.md` | Status of every spec (Not started → Complete) |
| `transformation/tasks.md` | Backlog and completed tasks per milestone |
| `transformation/infrastructure.md` | Workspace layout, dependencies, feature flags, build targets |
| `transformation/validation.md` | Test status, VirtualAGC fixtures, timing budgets |
| `docs/agc-reference-constants.md` | Pre-validated AGC constants, algorithms, key codes |

### Phase 1 — Understand the task

1. Identify the milestone and component from `transformation/progress.md` and `transformation/tasks.md`.
2. **Read the spec** in `specs/` — use it as the source of truth for API, scale factors, invariants, and test cases.
3. Read the corresponding AGC assembler source from `docs/agc-source/*.agc` (see Source File Map in `docs/agc-reference-constants.md`).
4. Check `docs/agc-reference-constants.md` for any pre-validated constants or algorithm descriptions.
5. Confirm runtime constraints from `transformation/infrastructure.md`: `#![no_std]`, `#![no_main]`, no heap, interrupt model, `thumbv7em-none-eabihf`.

### Phase 2 — Implement

6. Make the simplest design that fits the existing codebase. Match conventions in `docs/architecture.md`.
7. **Type conventions**:
   - Navigation/guidance math → `f64`, SI units
   - CDU angles, PIPA counts, channel words → `u16` / `i16`
   - Expose physical quantities through newtypes: `CduAngle`, `Met`, `DeltaV`
   - Vectors and matrices: `Vec3 = [f64; 3]`, `Mat3x3 = [[f64; 3]; 3]`
8. **Shared mutable state** (interrupt handlers + foreground): `static Mutex<RefCell<T>>`, accessed via `interrupt::free`. Never `static mut`.
9. **AGC source cross-reference** in doc comments:
   ```rust
   /// AGC source: Comanche055/CONIC_SUBROUTINES.agc, KEPRTN routine.
   pub fn kepler_step(...) -> (Vec3, Vec3) { ... }
   ```
10. **Restart safety**: bracket multi-step computations with `state.restart.set_phase(...)`.
11. **Constants**: use AGC values from `docs/agc-reference-constants.md`, not modern IAU/WGS84 values (fidelity wins).

### Phase 3 — Test and validate

12. Add tests. Math functions need at least one VirtualAGC fixture case (see `docs/testing.md §7`).
13. Add AGC constant assertion tests to `agc-core/src/tests/agc_constants.rs` for any new constants.
14. Run the validation skill (`/validation`) against the AGC source to verify correctness.

### Phase 4 — Update tracking and agc-sim

15. Update `transformation/specifications.md` status when done.
16. Update `transformation/progress.md` with new test counts and milestone status.
17. Update `transformation/tasks.md` — check off completed tasks.
18. Update `transformation/validation.md` — mark test statuses.
19. Wire new observable state into `agc-sim` (see agc-sim integration rule in `specs/README.md`).
20. Validate: `cargo fmt`, `cargo clippy -- -D warnings`, `cargo test`, `cargo build --target thumbv7em-none-eabihf -p agc-core`.

## Design Heuristics

- Use meaningful names and standard Rust conventions (`snake_case`/`PascalCase`/`SCREAMING_SNAKE_CASE`).
- Prefer plain data structures before clever abstractions.
- Keep `AgcState` fields private; expose accessors only where needed.
- For HAL structs: add `free()`, implement `embedded-hal` traits, use typestate for operational modes.
- For interrupt handlers: use `#[interrupt]` from the device PAC crate, not `cortex-m-rt` directly.
- No `unwrap`/`expect` in `agc-core` production code — use `Option`, program alarms, or restart.
- Keep interrupt handlers short. Move long work to Executive jobs.
- Avoid redundant comments. Add comments for scaling factors, AGC source mapping, safety invariants.

## Delivery Checklist

### Code quality
- [ ] API matches `docs/architecture.md` conventions
- [ ] No heap allocation in `agc-core`
- [ ] No `static mut` — `Mutex<RefCell<T>>` used for shared state
- [ ] `#[interrupt]` sourced from device PAC
- [ ] AGC source cross-referenced in doc comments
- [ ] Scaling factors documented where `f64` ↔ AGC fixed-point conversion occurs
- [ ] Restart protection applied to multi-step computations
- [ ] No `unwrap`/`expect` in production code
- [ ] No `dbg!`, `hprintln!`, or commented-out code remains

### AGC fidelity
- [ ] Constants match AGC values in `docs/agc-reference-constants.md` (not modern values)
- [ ] New constants have assertion tests in `agc-core/src/tests/agc_constants.rs`
- [ ] Algorithm structure matches AGC source (or deviation documented as APPROXIMATE)
- [ ] Validation skill run confirms CONFIRMED on all items

### Testing
- [ ] Tests include at least one VirtualAGC fixture case for math functions
- [ ] `cargo build --target thumbv7em-none-eabihf -p agc-core` passes
- [ ] `cargo test --workspace` passes with zero failures

### Tracking updates
- [ ] `transformation/specifications.md` — spec status updated
- [ ] `transformation/progress.md` — milestone status and test counts updated
- [ ] `transformation/tasks.md` — completed tasks checked off
- [ ] `transformation/validation.md` — test statuses updated
- [ ] `docs/agc-reference-constants.md` — new constants/algorithms added
- [ ] `agc-sim` — new observable state wired into TUI (if applicable)

## Results Format

**Changes Made:** files created/modified, what changed, new dependencies if any

**Validation:**
```
cargo fmt          → clean
cargo clippy       → clean
cargo test         → X passed
cargo build (M4F)  → success
```

**Next Steps:** 2–3 logical follow-ups (spec status update, fixture capture, timing measurement, etc.)
