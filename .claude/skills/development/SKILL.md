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

1. Inspect crate layout, `Cargo.toml`, and nearby code before editing.
2. **Read the spec** in `specs/` — use it as the source of truth for API, scale factors, invariants, and test cases.
3. Confirm runtime constraints: `#![no_std]`, `#![no_main]`, no heap, interrupt model, `thumbv7em-none-eabihf`.
4. Make the simplest design that fits the existing codebase. Match conventions in `docs/architecture.md`.
5. **Type conventions**:
   - Navigation/guidance math → `f64`, SI units
   - CDU angles, PIPA counts, channel words → `u16` / `i16`
   - Expose physical quantities through newtypes: `CduAngle`, `Met`, `DeltaV`
   - Vectors and matrices: `Vec3 = [f64; 3]`, `Mat3x3 = [[f64; 3]; 3]`
6. **Shared mutable state** (interrupt handlers + foreground): `static Mutex<RefCell<T>>`, accessed via `interrupt::free`. Never `static mut`.
7. **AGC source cross-reference** in doc comments:
   ```rust
   /// AGC source: Comanche055/CONIC_SUBROUTINES.agc, KEPRTN routine.
   pub fn kepler_step(...) -> (Vec3, Vec3) { ... }
   ```
8. **Restart safety**: bracket multi-step computations with `state.restart.set_phase(...)`.
9. Add tests. Math functions need at least one VirtualAGC fixture case (see `docs/testing.md §7`).
10. Update `transformation/specifications.md` status when done.
11. Validate: `cargo fmt`, `cargo clippy -- -D warnings`, `cargo test`, `cargo build --target thumbv7em-none-eabihf -p agc-core`.

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

- [ ] API matches `docs/architecture.md` conventions
- [ ] No heap allocation in `agc-core`
- [ ] No `static mut` — `Mutex<RefCell<T>>` used for shared state
- [ ] `#[interrupt]` sourced from device PAC
- [ ] AGC source cross-referenced in doc comments
- [ ] Scaling factors documented where `f64` ↔ AGC fixed-point conversion occurs
- [ ] Restart protection applied to multi-step computations
- [ ] Tests include at least one VirtualAGC fixture case for math functions
- [ ] `cargo build --target thumbv7em-none-eabihf -p agc-core` passes
- [ ] No `dbg!`, `hprintln!`, or commented-out code remains

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
