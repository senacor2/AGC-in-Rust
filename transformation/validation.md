# Validation

Full strategy: `docs/testing.md`. This file tracks status.

## VirtualAGC Fixture Capture

Fixtures are captured once from VirtualAGC (Docker), committed to `agc-test/fixtures/`, and then used in normal `cargo test` runs without Docker.

| Fixture | AGC Program | Status |
|---------|-------------|--------|
| `kepler_cases.json` | P30 External Delta-V | Not captured |
| `lambert_cases.json` | P30 External Delta-V | Not captured |
| `servicer_pipa_trace.json` | P11 orbit insertion | Not captured |
| `p40_burn_channel_trace.bin` | P40 SPS burn | Not captured |

## Unit Tests (Milestone 1 — Executive + Waitlist)

| Test | Status |
|------|--------|
| Highest-priority job runs first | Not started |
| All job slots exhausted → alarm 1202 | Not started |
| Job completes and frees slot | Not started |
| Waitlist fires after delta-T | Not started |
| Chained waitlist entries fire in delta order | Not started |
| Restart resumes at saved phase | Not started |
| Zero-priority job never preempts higher-priority job | Not started |

## Unit Tests (Milestone 2 — Navigation)

| Test | Status |
|------|--------|
| Kepler solver matches VirtualAGC fixture (< 1×10⁻⁹ rad) | Not started |
| Lambert targeting matches fixture (< 0.1 m/s delta-V) | Not started |
| SERVICER 2-second cycle position error < 1 m | Not started |
| SERVICER 2-second cycle velocity error < 0.01 m/s | Not started |

## Integration Tests (requires `VAGC_AVAILABLE=1`)

| Test | Status |
|------|--------|
| P40 SPS burn channel trace matches golden | Not started |
| P37 Return to Earth guidance output matches golden | Not started |

## Property Tests

| Property | Status |
|----------|--------|
| Job slot invariant: `free + active == 7` for any sequence | Not started |
| Waitlist ordering: next-to-fire always has smallest delta | Not started |

## Coverage Target

80% line coverage on `executive/` and `navigation/` before each milestone is declared complete.
Tool: `cargo llvm-cov` or `cargo tarpaulin`.

## Timing Compliance

| Interrupt | Budget | Measured | Status |
|-----------|--------|----------|--------|
| T6RUPT | 0.5 ms | — | Not measured |
| T5RUPT (DAP) | 20 ms | — | Not measured |
| T3RUPT (Waitlist dispatch) | 5 ms | — | Not measured |
| T4RUPT (periodic I/O) | 10 ms | — | Not measured |
