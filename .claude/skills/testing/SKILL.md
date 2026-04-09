---
name: testing
description: Use when writing, reviewing, or fixing Rust tests in AGC-in-Rust — unit tests, integration tests, VirtualAGC fixture tests, property tests, scenario tests, embedded no_std test strategies, or timing compliance checks.
argument-hint: Describe the testing task, failing test, or coverage gap
---

# Rust Testing — AGC-in-Rust

## When To Use

- Add unit tests for a new AGC component
- Write scenario tests that drive the Executive/Waitlist through realistic job/task sequences
- Write spec-linked tests that explicitly cite the AGC source they validate
- Write VirtualAGC fixture tests for math/navigation functions
- Add fixture capture tooling to `agc-test/src/oracle/`
- Debug failing `cargo test` runs or fixture mismatches
- Plan the host-vs-hardware test split for a new module
- Write property tests for scheduler invariants

## Test Levels

| Level | Where | Docker needed? | Target |
|---|---|---|---|
| **Unit tests** | `agc-core/src/**/mod.rs` `#[cfg(test)]` | No | Individual functions |
| **Scenario tests** | `agc-test/tests/scenario_*.rs` | No | Executive + Waitlist + SimHardware |
| **Spec-linked tests** | Any `#[cfg(test)]` block | No | Behavioral AGC invariants |
| **Math fixture tests** | `agc-test/tests/navigation_accuracy.rs` | No — fixtures committed | `math/`, `navigation/`, `guidance/` |
| **SERVICER cycle tests** | `agc-test/tests/servicer_cycle.rs` | No — fixtures committed | `services/average_g.rs` |
| **End-to-end channel trace** | `agc-test/tests/integration_e2e.rs` | Yes (`VAGC_AVAILABLE=1`) | Full program runs |
| **Property tests** | `agc-test/tests/` | No | Scheduler invariants |

## Procedure

1. Choose the right level (see table above).
2. For **scenario tests**: use `SimHardware` from `agc-sim`; drive `AgcState` through a scripted sequence of job establishments, waitlist insertions, and dispatches; assert observable state after each step.
3. For **spec-linked tests**: open the AGC source referenced in `docs/AGC Symbolic Listing.md`; identify the behavioral invariant (alarm code, dispatch order, phase value); write the test; add a `// AGC source:` comment citing the file and routine.
4. For **fixture tests**: load JSON from `agc-test/fixtures/`; use `from_agc_word` / `from_agc_dword` for conversion; assert within tolerances from `docs/testing.md §5`.
5. Keep tests deterministic — no sleeps, no shared mutable globals, no external services.
6. Run narrow first (`cargo test <name>`), then `cargo test --workspace`.

## Writing Scenario Tests

Scenario tests run a scripted multi-step sequence through the real Executive + Waitlist + SimHardware, proving that the scheduler behaves identically to the AGC across a realistic interaction sequence.

**Pattern:**
```rust
/// AGC source: EXECUTIVE.agc — EXEC loop priority dispatch and 1202 recovery.
#[test]
fn scenario_priority_dispatch_and_overflow_recovery() {
    let mut state = AgcState::new();
    let mut hw = SimHardware::new();

    // Step 1: establish two jobs at different priorities
    state.executive.establish_job(job_a, 3, 0, &mut state.alarms);
    state.executive.establish_job(job_b, 5, 0, &mut state.alarms);

    // Step 2: fill remaining slots to force overflow
    for _ in 0..5 {
        state.executive.establish_job(job_noop, 1, 0, &mut state.alarms);
    }
    // Step 3: overflow triggers alarm 1202
    let result = state.executive.establish_job(job_noop, 1, 0, &mut state.alarms);
    assert!(result.is_none());
    assert!(state.alarms.is_raised(AlarmCode::ExecutiveOverflow));

    // Step 4: clear one slot; alarm clears on crew acknowledge
    state.executive.complete_job(0);
    state.alarms.clear_all();
    assert!(!state.alarms.is_raised(AlarmCode::ExecutiveOverflow));
}
```

**What scenarios to cover (Milestone 1):**
- Priority dispatch: multiple jobs at different priorities — highest always runs first
- Alarm 1202: all 7 slots full → overflow alarm → slot freed → recovery
- Waitlist ordering: tasks scheduled out of order arrive in correct delta-time sequence
- Waitlist-to-job handoff: a task dispatched from Waitlist establishes a new job
- Restart recovery: non-idle phases re-dispatched; idle phases skipped
- Fresh start: all state cleared; zero alarms; zero active jobs

## Writing Spec-Linked Tests

Every behavioral test should cite its AGC source. Use this pattern:

```rust
/// Validates: alarm 1202 fires on Executive overflow.
/// AGC source: EXECUTIVE.agc — NOVAC/FINDVAC; ALARM_AND_ABORT.agc — code 01202.
/// Reference: docs/AGC Symbolic Listing.md §Executive.
#[test]
fn alarm_1202_on_executive_overflow() { ... }
```

The comment makes the test self-documenting and lets reviewers trace back to the original AGC behavior.

## Writing Fixture Tests

Fixtures live in `agc-test/fixtures/` as JSON. See `docs/testing.md §6` for the `from_agc_word` / `from_agc_dword` conversion utilities.

```rust
// Example pattern for a math fixture test
#[test]
fn kepler_step_matches_virtualagc() {
    let cases: Vec<KeplerCase> = serde_json::from_str(
        include_str!("../fixtures/kepler_cases.json")
    ).unwrap();
    for case in cases {
        let (r, v) = kepler_step(case.r0, case.v0, case.dt, MU_EARTH);
        assert_vec3_close(r, case.expected_r, 1.0);       // 1 metre tolerance
        assert_vec3_close(v, case.expected_v, 0.01);      // 0.01 m/s tolerance
    }
}
```

Tolerances (from `docs/testing.md §5`):

| Computation | Tolerance |
|---|---|
| Kepler solver | 1×10⁻⁹ rad true anomaly |
| Lambert targeting | 0.1 m/s delta-V |
| SERVICER cycle (2s) | 1 m position, 0.01 m/s velocity |

## Fixture Capture Tooling (`agc-test/src/oracle/`)

When VirtualAGC is available, use the oracle module to capture reference data:

```
agc-test/src/oracle/
  mod.rs
  vagc_socket.rs      ← TCP client for yaAGC channel protocol (port 19697)
  memory_dump.rs      ← parser for yaAGC --debug erasable output
  fixture_capture.rs  ← one-shot: run scenario, capture memory dump, write JSON fixture
```

Run capture with `VAGC_AVAILABLE=1 cargo test --test fixture_capture`.
Commit the resulting JSON files; subsequent `cargo test` runs use them without Docker.

## Heuristics

- Prefer small fixtures built in code; use JSON fixtures only when VirtualAGC reference data is needed.
- Mock the HAL (`agc-sim` SimHardware) for scenario and unit tests; use real fixture data for nav accuracy tests.
- For `no_std` paths, explicitly test fixed-capacity behavior and allocation-free invariants.
- Keep ISR-related tests deterministic — do not assert on timing jitter.
- Test error paths: 1202 alarm on Executive overflow, 1210 on Waitlist overflow.
- Include invalid inputs and edge cases for math functions, not only happy paths.
- Each scenario test must be traceable to an AGC source file and routine.

## Delivery Checklist

- [ ] Tests prove the intended AGC behavior, not just that the call succeeds
- [ ] Scenario tests use `SimHardware` from `agc-sim` for realistic end-to-end execution
- [ ] Every behavioral test has a `// AGC source:` comment
- [ ] Fixture tests reference `docs/testing.md §5` tolerances
- [ ] Scheduler tests cover all invariants (priority dispatch, overflow, restart, waitlist order)
- [ ] No `dbg!`, `println!`, or leftover debug output in test code
- [ ] `cargo test --workspace` passes with zero failures

## Results Format

**Changes Made:** test files created/modified, fixtures added, skill updated

**Validation:**
```
cargo test <specific_test> → PASSED
cargo test --workspace     → X passed
```

**Next Steps:** fixture capture needed, coverage gaps, timing validation on hardware, etc.
