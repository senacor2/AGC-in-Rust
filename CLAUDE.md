# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project uses AI agents to port the Apollo Guidance Computer (AGC) to idiomatic Rust. The scope is the **Comanche055** module (Command Module), covering earth-to-moon-and-back travel. Lunar landing is out of scope. The goal is to re-create the abstractions lost when the original AGC assembler code was written, producing readable and maintainable Rust.

The target system is a bare-metal, hard real-time computer with very limited memory and CPU. There is no operating system — task scheduling is part of the navigation software itself.

**Fidelity principle**: where Rust idiom and AGC fidelity conflict, fidelity wins. Navigation errors kill people.

## Agent Workflow

Work proceeds through a pipeline of specialized agents in `.claude/agents/`:
**analyst-reengineer** (AGC source → functional specs) → **architect** (specs → Rust architecture) → **developer** (specs + architecture → Rust) → **tester** (tests). Each agent reads the prior stage's output; the analyst's specs are the primary input to architect and developer.

Full roles, hand-off flow, agent-selection triggers, and parallelism rules: **`docs/workflow.md`**.

## Key Reference Material

- `docs/AGC Symbolic Listing.md` — markdown conversion of the formal AGC hardware/software specification (Block 2 AGC, Comanche/Colossus 2D for Apollo 13)
- [Apollo-11 source on GitHub](https://github.com/chrislgarry/Apollo-11) — digitized AGC assembler source (Comanche055 = Command Module)
- [AGC Assembly Language Manual](https://www.ibiblio.org/apollo/assembly_language_manual.html) — machine, interpreter, and pseudocode instruction descriptions

## Coding Rules & Constraints

These are the canonical rules for all agents. (This file auto-loads into every subagent; rules live here once, not duplicated in agent files.)

**System constraints** — the Rust implementation reflects the original AGC: hard real-time scheduling (no OS; the software owns the scheduler), minimal memory footprint, robust error recovery (always return to a safe state). Inputs: stellar positions, inertial platform (orientation + acceleration). Outputs: thruster control (attitude) and main engine control (velocity). Crew interface: a DSKY-style console for invoking programs.

**Code style**
- Stable Rust only, no nightly. Follow existing module/naming patterns before adding abstractions.
- Standard naming: `snake_case` (fns/vars/modules), `PascalCase` (types/traits), `SCREAMING_SNAKE_CASE` (constants).
- Keep public APIs small and explicit. Prefer `&T` / `&mut T` / owned values; never expose `RefCell`/`UnsafeCell`/`Mutex` in a public API (they are shared-state implementation details).
- Prefer concrete types → generics → `dyn Trait` only where callers need dynamic dispatch.
- Avoid `unwrap`/`expect` in flight code: any failure not statically ruled out is handled or triggers the GOJAM restart path explicitly.
- `Result`-based errors are for the `agc-sim` host crate. In `agc-core` (no_std, no heap) use `Option` and program alarms (`alarm::raise`).
- Comment Rust-specific nuance, invariants, safety, scale factors, and non-obvious AGC→Rust mappings — not self-evident code.

**Embedded / no_std (override general style on conflict)**
- **No heap.** `alloc`/`Vec`/`Box`/`String`/`HashMap` are forbidden in `agc-core`; all structures are statically sized.
- **No `static mut`.** Shared mutable state uses `cortex_m::interrupt::Mutex<RefCell<T>>`, accessed via `cortex_m::interrupt::free(|cs| ...)`. Raw `static mut` is a Clippy error.
- **No blocking.** Interrupt handlers and Waitlist tasks must not block, spin-wait, or run long computations — establish long work as an Executive job.
- **No unwinding.** `panic = "abort"`; every panic triggers GOJAM (hardware restart). Don't rely on `Drop` for pre-restart cleanup.
- **f64 for all navigation math.** Fixed-point was a hardware constraint, not a requirement; `i16`/`u16` only for raw hardware values (CDU angles, PIPA counts, channel words).
- **No interpreter.** Re-implement interpretive-language routines as plain `f64` Rust functions; do not build the AGC VM.
- **Restart safety.** Multi-step computations that must survive restart use the phase-table pattern (`state.restart.set_phase(...)`; see `executive/restart.rs`).

**Architecture**
- Keep domain logic (navigation, guidance, DAP) separate from the HAL and the Executive scheduler.
- All mutable state lives in `AgcState`, passed by `&mut` through foreground code. State also touched by ISRs is extracted into a dedicated `static Mutex<RefCell<T>>`; ISRs access that static directly (narrowest view), never `&mut AgcState`.
- The HAL boundary (`AgcHardware` and its sub-traits) is the only place flight software touches hardware — no peripheral register access outside `hal/`.
- Bare-metal HAL structs implement `free()` (C-FREE), applicable `embedded-hal` traits (C-HAL-TRAITS), and typestate type params for operational modes (C-PIN-STATE).
- The `#[interrupt]` attribute is re-exported from the device PAC crate, not from `cortex-m-rt` directly.

**Conventions**
- Document public modules, types, and functions. Navigation/guidance functions document input units+scale, output units+scale, and the corresponding AGC source routine + file.
- Physical-quantity newtypes (`CduAngle`, `Met`, `DeltaV`) document their unit and scale factor at the struct level.
- Every `unsafe` block is justified in a comment naming the invariant upheld (not "safe here").
- Document non-obvious constants: meaning, AGC source, units. Prefer `#[expect(lint, reason = "...")]` over `#[allow(lint)]`.

**AGC source cross-reference** — every Rust fn implementing a specific AGC routine carries a doc-comment cross-reference:

```rust
/// Solve Kepler's equation for the universal variable.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPRTN routine.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) { ... }
```

## Build & Test

- Validate all changes with `cargo fmt`, `cargo clippy -- -D warnings`, and `cargo test`.
- `agc-core` must always build with `cargo build --target thumbv7em-none-eabihf` (bare-metal, hard-float). A change that breaks the embedded build is not mergeable.
- Unit tests in `agc-core` run on the host (`#[cfg(test)]`) and must not use any `std` feature gated behind the `sim` flag. Integration tests in `agc-test` use the `agc-sim` hosted HAL and are the home for end-to-end scenario testing.
- Math-function tests include at least one case from a VirtualAGC reference run (see `docs/testing.md`).
- No `dbg!`, `println!`, or temporary `hprintln!` in finished changes.
- Validate implemented features against the AGC source: https://github.com/chrislgarry/Apollo-11/tree/master/Comanche055
