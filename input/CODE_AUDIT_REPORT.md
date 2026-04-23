# Code Audit Report

## Project
AGC-in-Rust

## Audit Date
2026-04-22

## Scope
- Static code audit across `agc-core`, `agc-sim`, and `agc-test`.
- Architecture and safety compliance check against `AGENTS.md`.
- High-risk pattern review (`unsafe`, `static mut`, panic paths, scheduling reliability).

## Methodology
- Repository inventory and targeted source inspection.
- Pattern scans for:
  - `unwrap` / `expect` / `panic`-style abort paths
  - `unsafe` and `static mut`
  - scheduler/waitlist error handling
  - no-heap and no-`std` compliance in `agc-core`
- Manual review of critical runtime modules:
  - `services/fresh_start.rs`
  - `executive/waitlist.rs`
  - `control/dap.rs`
  - `services/average_g.rs`
  - key guidance/program entry points

## Executive Summary
The repository shows strong structure and good no-heap discipline in `agc-core`, but the audit identified **4 major correctness/safety issues**, including one critical issue in FRESH START behavior and two high-severity reliability/architecture violations.

## Findings

### 1) Critical: FRESH START does not fully reset operational state
**Severity:** Critical  
**Why it matters:** A FRESH START should guarantee a clean, safe baseline. Incomplete reset allows stale operational callbacks/state to survive and execute unexpectedly.

**Evidence:**
- `agc-core/src/services/fresh_start.rs:29` resets selected fields but does not perform full state sanitation.
- `agc-core/src/programs/p00.rs:27` documents P00 init behavior required to cancel active burn/control state.
- `agc-core/src/services/v_n.rs:120` + `agc-core/src/services/v_n.rs:248` shows `pending_v50` callback execution path on `PRO`.

**Risk:**
- Stale callback execution after restart/reset flow.
- Residual guidance/control flags persisting across FRESH START boundary.

---

### 2) High: Waitlist scheduling failures are ignored in periodic control loops
**Severity:** High  
**Why it matters:** Ignoring `ScheduleResult::Full` can silently terminate periodic control/navigation behavior.

**Evidence:**
- `agc-core/src/control/dap.rs:201` and `agc-core/src/control/dap.rs:286` discard scheduling result.
- `agc-core/src/services/average_g.rs:124` discards initial SERVICER scheduling result.
- `agc-core/src/executive/waitlist.rs:59` defines explicit full-queue behavior via `ScheduleResult::Full`.

**Risk:**
- DAP/SERVICER task chains can stop without alarm escalation or deterministic fallback.

---

### 3) High: Raw `static mut` restart table violates project architecture rule
**Severity:** High  
**Why it matters:** The project explicitly bans `static mut` shared mutable state. Current usage introduces UB risk as the codebase evolves.

**Evidence:**
- Policy: `AGENTS.md:20` ("No `static mut`").
- Implementation: `agc-core/src/services/fresh_start.rs:119` (`pub static mut RESTART_GROUP_TABLE ...`).
- Unsafe access: `agc-core/src/services/fresh_start.rs:166`.

**Risk:**
- Unsafe global mutation model conflicts with intended interrupt-safe ownership model.

---

### 4) Medium: Recoverable input faults handled with `assert!` panics in runtime paths
**Severity:** Medium  
**Why it matters:** In flight software, avoid panic-driven control flow for recoverable/operator-facing errors.

**Evidence (representative):**
- `agc-core/src/programs/p37.rs:51`, `:84`, `:96`
- `agc-core/src/guidance/targeting.rs:280`
- `agc-core/src/guidance/rendezvous.rs:212`, `:251`, `:307`
- `agc-core/src/math/linalg.rs:37`

**Risk:**
- Input/edge-case faults trigger abort/restart behavior where alarm-and-reject may be safer.

## Positive Observations
- `agc-core` appears free of heap containers and `std`-based runtime usage in production code paths.
- Module boundaries (guidance/navigation/control/services) are generally clear and maintainable.
- Test coverage breadth is substantial, especially for math and program behavior cases.

## Validation Status
Runtime/tooling validation could not be executed in this audit environment because Rust toolchain commands were unavailable (`cargo`, `rustc`, `rustup` not found).  
As a result, this report is a **static audit** and does not include pass/fail results for:
- `cargo fmt --check`
- `cargo clippy -- -D warnings`
- `cargo test`
- `cargo build --target thumbv7em-none-eabihf`

## Recommended Remediation Order
1. Fix FRESH START reset completeness and ensure stale callback/state cannot survive reset.
2. Enforce explicit handling of all waitlist scheduling outcomes in DAP and SERVICER paths.
3. Replace `static mut` restart table with the project-approved shared-state pattern.
4. Replace panic-style runtime precondition checks with alarmed rejection paths where recoverable.

## Suggested Follow-up Audit Gate
After fixes, require all of the following before merge:
1. Full lint/format/test/build run including `thumbv7em-none-eabihf`.
2. Regression tests for FRESH START state cleanliness and waitlist saturation behavior.
3. Focused restart-path tests confirming restart-group dispatch remains deterministic.
