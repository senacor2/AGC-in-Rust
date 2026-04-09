---
name: testing
description: Use when writing, reviewing, or fixing Rust tests in AGC-in-Rust — unit tests, integration tests, VirtualAGC fixture tests, property tests, embedded no_std test strategies, or timing compliance checks.
argument-hint: Describe the testing task, failing test, or coverage gap
---

# Rust Testing — AGC-in-Rust

## When To Use

- Add unit tests for a new AGC component
- Write VirtualAGC fixture tests for math/navigation functions
- Debug failing `cargo test` runs or fixture mismatches
- Plan the host-vs-hardware test split for a new module
- Write property tests for scheduler invariants

## Test Levels (from `docs/testing.md`)

| Level | Where | Docker needed? | Target |
|---|---|---|---|
| Math fixture tests | `agc-test/tests/navigation_accuracy.rs` | No — fixtures committed | `math/`, `navigation/`, `guidance/` |
| SERVICER cycle tests | `agc-test/tests/servicer_cycle.rs` | No — fixtures committed | `services/average_g.rs` |
| End-to-end channel trace | `agc-test/tests/integration_e2e.rs` | Yes (`VAGC_AVAILABLE=1`) | Full program runs |
| Unit tests | `agc-core/src/**/mod.rs` `#[cfg(test)]` | No | Individual functions |
| Property tests | `agc-test/tests/` | No | Scheduler invariants |

## Procedure

1. Decide scope: unit (implementation detail), fixture (math correctness vs. VirtualAGC), integration (full scenario), or property (invariant).
2. Reproduce the current failure or gap with the narrowest `cargo test` target.
3. For embedded `agc-core`: host tests validate pure logic; hardware-in-loop validates peripheral timing.
4. Keep tests deterministic — no sleeps, no shared mutable global state, no external services.
5. Add regression tests when fixing bugs.
6. Run the narrow test, then broaden to `cargo test --workspace`.

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

## Writing Scheduler Tests

```rust
// Core invariants to cover for Executive + Waitlist
#[test] fn highest_priority_job_runs_first() {}
#[test] fn all_slots_exhausted_triggers_alarm_1202() {}
#[test] fn job_completes_and_frees_slot() {}
#[test] fn waitlist_fires_after_delta_t() {}
#[test] fn chained_waitlist_fires_in_order() {}
#[test] fn restart_resumes_at_saved_phase() {}
```

Property tests (via `proptest`):
- `free_slots + active_slots == 7` after any sequence of start/end calls
- Next-to-fire Waitlist task always has the smallest remaining delta

## Heuristics

- Prefer small fixtures built in code; use JSON fixtures only when VirtualAGC reference data is needed.
- Mock the HAL (`agc-sim` SimHardware) for unit tests; use real fixture data for nav accuracy tests.
- For `no_std` paths, explicitly test fixed-capacity behavior and allocation-free invariants.
- Keep ISR-related tests deterministic — do not assert on timing jitter.
- Test error paths: 1202 alarm on Executive overflow, 1211 on Waitlist overflow.
- Include invalid inputs and edge cases for math functions, not only happy paths.

## Delivery Checklist

- [ ] Tests prove the intended behavior, not just that the call succeeds
- [ ] Fixture tests reference `docs/testing.md §5` tolerances
- [ ] Scheduler tests cover all invariants listed in `transformation/validation.md`
- [ ] No `dbg!`, `println!`, or leftover debug output in test code
- [ ] `cargo test` (and `cargo nextest run`) pass

## Results Format

**Changes Made:** test files created/modified, fixtures added

**Validation:**
```
cargo test <specific_test> → PASSED
cargo nextest run          → X passed
```

**Next Steps:** fixture capture needed, coverage gaps, timing validation on hardware, etc.
