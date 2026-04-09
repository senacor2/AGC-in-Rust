# Architecture Decision Records (ADRs)

Full rationale and trade-offs for each decision are in `docs/architecture.md §15`.
This file is the index and status tracker.

---

## ADR-001: Interpreter Elimination

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Implement all navigation and guidance computations as direct Rust functions using `f64`. Do not re-implement the AGC interpretive language VM.

**Rationale**: The interpreter was a space-saving measure on a 15-bit, 2K-ROM machine. On a Cortex-M4F with hardware FPU, native `f64` functions are faster, smaller, type-checked, and directly testable.

**Trade-off**: Translation from interpretive code requires careful understanding of each routine's numerical intent. Mitigation: unit tests compare outputs against VirtualAGC reference runs. See `docs/testing.md`.

---

## ADR-002: Single AgcState Structure

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Collect all mutable state into one `AgcState` struct, passed by `&mut` reference through foreground code.

**Rationale**: Eliminates `static mut` and `unsafe` from most of the codebase. Makes data flow explicit. Enables unit testing without global state.

**Trade-off**: State shared with interrupt handlers cannot be inside `AgcState` (interrupt handlers are called by hardware and cannot receive function arguments). Such state lives in dedicated `static Mutex<RefCell<T>>` variables. `cortex_m::interrupt::Mutex` is heap-free — it is a plain struct wrapper with zero allocation overhead.

---

## ADR-003: Native Types Instead of Ones-Complement

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: `f64` for all navigation and guidance math. `i16`/`u16` for raw hardware values (channel words, CDU angles, PIPA counts). No custom `AgcWord` type.

**Rationale**: The AGC's ones-complement arithmetic was a hardware constraint, not a navigation requirement. `f64` provides 53 bits of mantissa and eliminates scale-factor bookkeeping errors.

**Trade-off**: Floating-point requires an FPU. All viable Cortex-M targets (M4F and above) include a hardware FPU. Soft-float is not acceptable for the DAP timing budget.

---

## ADR-004: Restart Protection as Explicit State Machine

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Preserve the AGC's phase-table restart mechanism rather than relying on Rust's safety guarantees alone.

**Rationale**: Rust prevents software bugs but not hardware faults (RAM bit flips, power glitches). The phase-table mechanism provides recovery from any cause of restart.

---

## ADR-005: HAL Trait for Hardware Isolation

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Define hardware interaction through the `AgcHardware` trait with focused sub-traits per subsystem. Separate bare-metal and simulation implementations.

**Rationale**: The flight software must be testable on a development host without hardware. The trait boundary is the natural seam.

**Trade-off**: Use monomorphization (generics) not trait objects in the hot path to eliminate vtable overhead.

---

## ADR-006: No Dynamic Memory Allocation

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: `#![no_std]` with no `alloc` crate. All data structures statically sized.

**Rationale**: The original AGC had no heap. Heap fragmentation in a long-running real-time system is a reliability hazard. Fixed-size structures have deterministic access times.

---

## ADR-007: Rust Embedded Ecosystem as Target Platform

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Target the `rust-embedded` ecosystem (`cortex-m`, `cortex-m-rt`, `embedded-hal`) with a minimum of Cortex-M4F.

**Rationale**: `embedded-hal` traits align directly with the `AgcHardware` abstraction layer. Cortex-M4F guarantees a hardware FPU required for the DAP's 100ms timing budget with `f64`.

---

## ADR-008: Mutex\<RefCell\<T\>\> for Interrupt-Shared State (not raw static mut)

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: All mutable state shared between interrupt handlers and foreground code uses `cortex_m::interrupt::Mutex<RefCell<T>>`. Raw `static mut` is forbidden.

**Rationale**: `cortex_m::interrupt::Mutex` is heap-free (a plain struct wrapper around `UnsafeCell`). Its `borrow(cs)` method requires a `CriticalSection` token that can only be obtained inside `interrupt::free`. The compiler therefore statically guarantees that shared state is never accessed outside a critical section — no runtime overhead. See `docs/optimization.md §1`.

---

## ADR-009: Profile-Specific Panic Handler

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: Dev builds log `PanicInfo` via semihosting then restart. Release builds restart immediately. No `panic-halt` dependency.

**Rationale**: Silent panics are undebuggable. Only one `#[panic_handler]` is permitted per binary; `panic-halt` would conflict. See `docs/optimization.md §2`.

---

## ADR-010: PAC-Sourced \#\[interrupt\] Attribute

**Date**: 2026-04-09 | **Status**: Accepted

**Decision**: The `#[interrupt]` attribute is re-exported from the device PAC crate (e.g., `stm32f4`), not used directly from `cortex-m-rt`.

**Rationale**: Using the PAC's re-export causes the compiler to verify that the interrupt name actually exists on the target device. A typo in an interrupt name with `cortex-m-rt` directly compiles silently but produces an unregistered handler. See `docs/optimization.md §3`.

---

## Open / Proposed ADRs

| ID | Topic | Status |
|---|---|---|
| ADR-011 | Specific MCU target (STM32F405 vs STM32F7 vs other) | Proposed — needs hardware decision |
| ADR-012 | RTIC vs hand-rolled Executive for interrupt scheduling | Proposed — see `docs/optimization.md §1` |
