---
name: debugging
description: Use when debugging Rust compiler errors, borrow checker issues, clippy warnings, failing tests, panics, embedded no_std failures, bare-metal hard faults, ISR races, or VirtualAGC fixture mismatches in the AGC-in-Rust project.
argument-hint: Describe the compiler error, failing test, panic, hard fault, or incorrect navigation output
---

# Rust Debugging — AGC-in-Rust

## When To Use

- Compiler or borrow checker errors
- Failing Rust tests or runtime panics
- Clippy warnings that signal correctness issues
- Bare-metal failures: hard faults, stack overflows, ISR races
- Navigation output diverging from VirtualAGC fixtures
- Peripheral ownership conflicts or interrupt handler issues

## Procedure

1. Reproduce with the smallest relevant command: `cargo test <name>`, `cargo check`, or `cargo build --target thumbv7em-none-eabihf -p agc-core`.
2. Read the full error chain before editing — the first error usually causes several downstream ones.
3. Determine host-only vs. bare-metal-specific. Use the correct target triple for bare-metal failures.
4. Classify: ownership/lifetime · type mismatch · trait resolution · ISR concurrency · logic bug · scaling error.
5. Check `specs/` for the relevant spec if the bug is in AGC transformation code. Incorrect scale factors and misunderstood AGC semantics are common root causes.
6. Fix the root cause. Prefer borrowing over cloning. Follow `snake_case`/`PascalCase`/`SCREAMING_SNAKE_CASE`.
7. Update the spec if the bug reveals a spec error.
8. Add a regression test. Re-run validation and report.

## Debugging Heuristics

### Borrow Checker
- Identify: who owns the value, who borrows it, how long each reference must live.
- `AgcState` is threaded by `&mut` through foreground code — check for conflicting borrows before splitting.

### Embedded / ISR
- **Hard fault**: inspect `ef.pc` in `HardFault` handler against `cargo objdump` disassembly to find the faulting instruction. If no `HardFault` handler exists, add one (see `docs/optimization.md §6`).
- **Stack overflow**: enable `flip-link` in `.cargo/config.toml`; this turns a silent stack overflow into a clean HardFault.
- **ISR race**: confirm shared state uses `Mutex<RefCell<T>>` (not raw `static mut`) and is accessed only inside `interrupt::free`. Check every touch point — both the ISR and the foreground job.
- **Handler never fires**: confirm `#[interrupt]` is from the device PAC (e.g. `use stm32f4::interrupt`), not `cortex-m-rt` directly. A wrong name compiles silently.
- **Panic in release**: confirm the release `#[panic_handler]` calls `SCB::sys_reset()` and does not reference any unsafe global pointer.

### Navigation / Scaling
- If state vector diverges vs. VirtualAGC fixture: check `from_agc_word` / `from_agc_dword` scale factors in `agc-test/src/oracle/memory_dump.rs` against `docs/testing.md §6`.
- PIPA counts to m/s: scale B-14, 1 count = 2⁻¹⁴ m/s per AGC centisecond tick. Check `services/average_g.rs`.
- CDU angle to radians: `(raw as f64) * (TAU / 65536.0)` — in `CduAngle::to_radians`.

### Timing
- T5RUPT budget is 20 ms. If exceeded, profile `control/dap.rs` → `control/attitude.rs`. `f64` ops on Cortex-M4F are fast but matrix inversions are not.
- T6RUPT budget is 0.5 ms — the jet on/off path in `control/rcs_logic.rs` must be minimal.

## Exit Criteria

- Failure reproduces before the fix and no longer reproduces after
- Fix preserves intended semantics
- Unsafe change or lint suppression is justified in a comment
- Regression test added when practical
- No `dbg!`, temporary `hprintln!`, or commented-out code remains
