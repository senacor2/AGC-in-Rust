# AGC-in-Rust: Testing Strategy with VirtualAGC as Oracle

## Overview

The primary verification challenge for this project is **D1 (Interpreter Elimination)**: every navigation and guidance computation that was originally written in the AGC interpretive language must be re-implemented as a plain Rust `f64` function, and those functions must produce the same results as the original software.

The strategy is to use the Docker-based VirtualAGC (yaAGC) as a **reference oracle**: feed identical inputs to both VirtualAGC and the Rust implementation, then compare outputs.


## 1. What VirtualAGC Exposes

The `run-virtualagc.sh` script starts an interactive GUI via noVNC (port 6080). That is not useful for automated testing. VirtualAGC (yaAGC) also has a **socket-based I/O protocol** — the AGC simulation communicates with peripheral simulators (DSKY, IMU, etc.) over TCP using a simple channel-word protocol.

**Key capability**: yaAGC speaks a line-oriented binary protocol on port 19697. Each peripheral connects on its own port. A test harness can connect as a fake peripheral, inject channel words (simulated IMU PIPA counts, CDU angles), and read the AGC's output channel words (RCS commands, DSKY display, gimbal commands).

Additionally, yaAGC can dump erasable memory on demand via `--debug=erasable`, providing direct access to the AGC's internal state (state vectors, phase registers, flag words) after a known computation.


## 2. Three Levels of Integration

### Level 1 — Math Function Validation (highest value, easiest)

**Target**: `math/`, `navigation/`, `guidance/` modules.

**Approach**:
- Pick a known AGC interpretive subroutine (e.g., KEPRTN / Kepler solver, LAMBERT).
- Use yaAGC's `--debug` mode to dump erasable memory before and after a known program sequence (e.g., P30 External Delta-V with canned inputs).
- Extract numerical inputs/outputs from those memory dumps, converting AGC fixed-point words to `f64` using the documented scale factors.
- Commit the extracted values as JSON fixtures in `agc-test/fixtures/`.
- Write Rust unit tests asserting that `kepler_step()`, `lambert()`, etc. produce the same results within defined tolerance.

This level does **not** require the container to be running during `cargo test`. VirtualAGC is run once to capture reference data; the fixtures are committed and tests run without Docker.

### Level 2 — SERVICER / Navigation Cycle (medium complexity)

**Target**: `services/average_g.rs`, `navigation/integration.rs`.

**Approach**:
- Drive yaAGC into P11 (Earth orbit insertion monitor), which activates the SERVICER.
- Inject a scripted sequence of PIPA counts via the channel protocol.
- After N cycles, read the CSM state vector from AGC erasable memory.
- Assert that the Rust SERVICER integration produces the same position and velocity within navigation-grade precision (< 1 meter after 100 seconds).

### Level 3 — Full Program End-to-End (highest confidence, most complex)

**Target**: major mode programs (P40 SPS burn, P37 Return to Earth).

**Approach**:
- Use yaAGC scenario scripting to run a complete program sequence.
- Capture all output channel words (RCS jet mask, gimbal commands) as a golden trace.
- Replay the same sequence through `agc-sim` and compare the output channel-word trace.


## 3. Test Infrastructure Layout

```
agc-test/
  fixtures/
    kepler_cases.json          ← dumped from VirtualAGC memory, committed
    servicer_pipa_trace.json   ← PIPA input sequence + expected state vector
    p40_burn_channel_trace.bin ← golden output channel words

  src/
    oracle/
      mod.rs
      vagc_socket.rs           ← TCP client implementing the yaAGC channel protocol
      memory_dump.rs           ← parser for yaAGC --debug erasable output
      fixture_capture.rs       ← one-shot tool: run scenario, capture, save fixture

  tests/
    navigation_accuracy.rs     ← fixture-based, no Docker required
    servicer_cycle.rs          ← fixture-based, no Docker required
    integration_e2e.rs         ← requires Docker; gated by VAGC_AVAILABLE env var
```

Level 1 and 2 tests always run in CI. Level 3 tests run only when `VAGC_AVAILABLE=1` is set (Docker available), making them suitable for nightly or manual runs.


## 4. The yaAGC Channel Protocol

Each message is 4 bytes:

```
[channel: u8] [value_hi: u8] [value_lo: u8] [0x00]
```

Channels map directly to the AGC I/O channel table. Relevant channels for test driving:

| Channel | Direction       | Meaning                      |
|---------|-----------------|------------------------------|
| 010     | AGC → periph    | DSKY display relay           |
| 014     | periph → AGC    | PIPA X accumulator count     |
| 015     | periph → AGC    | PIPA Y accumulator count     |
| 016     | periph → AGC    | PIPA Z accumulator count     |
| 030–033 | AGC → periph    | RCS jet on/off commands      |
| 030–035 | periph → AGC    | CDU gimbal angles            |

The `vagc_socket.rs` module implements a minimal peripheral simulator that:

1. Connects to yaAGC on port 19697.
2. Sends pre-scripted PIPA/CDU channel words at the correct timing.
3. Records all received channel words with timestamps.
4. After the scenario, triggers an erasable memory dump and parses it.


## 5. Tolerance and Acceptance Criteria

Exact bit-for-bit `f64` agreement with AGC fixed-point results is not the goal. Tolerances are defined based on what the original software itself accepted (convergence checks, alarm thresholds):

| Computation            | Tolerance                                              |
|------------------------|--------------------------------------------------------|
| Kepler solver          | True anomaly within 1×10⁻⁹ radians                    |
| Lambert targeting      | Delta-V vector within 0.1 m/s                          |
| SERVICER cycle (2s)    | Position < 1 m, velocity < 0.01 m/s                   |
| Entry guidance         | Cross-range landing error < 1 km                       |

Alarm thresholds from `tables/alarm_codes.rs` and the assembly comments (e.g., the convergence check in KEPRTN) are the authoritative source for tolerance values.


## 6. AGC Fixed-Point Conversion

A central utility in the test harness handles conversion from AGC memory words to `f64`:

```rust
/// Convert a raw AGC 15-bit word to f64.
/// `scale` is the B-scale exponent: value = raw * 2^scale.
pub fn from_agc_word(raw: u16, scale: i8) -> f64 {
    (raw as i16 as f64) * (2.0_f64).powi(scale as i32)
}

/// Convert a double-precision AGC word pair (high, low) to f64.
pub fn from_agc_dword(hi: u16, lo: u16, scale: i8) -> f64 {
    let combined = ((hi as i32) << 14) | (lo as i32 & 0x3FFF);
    (combined as f64) * (2.0_f64).powi(scale as i32)
}
```

Scale factors for each erasable memory location are documented in the Comanche055 assembly listing comments and in `docs/AGC Symbolic Listing.md`.


## 7. Recommended Implementation Order

### Step 1: Fixture capture tooling (no Rust code required yet)

Write `fixture_capture.rs` and run it against yaAGC with P30 (External Delta-V) using a known TIG and target orbit. Capture the erasable memory state before and after the Lambert/Kepler interpretive blocks. Commit as `fixtures/kepler_cases.json` and `fixtures/lambert_cases.json`.

This step can be done before any navigation Rust code exists.

### Step 2: Math unit tests against fixtures

Implement `navigation_accuracy.rs` using the committed fixtures. These tests will initially fail (as the functions don't exist yet) and become the acceptance gate for each function as it is implemented.

### Step 3: SERVICER cycle test

Once `services/average_g.rs` and `navigation/integration.rs` exist, capture a PIPA injection trace and implement `servicer_cycle.rs`.

### Step 4: End-to-end channel trace tests

After the major mode programs are implemented, capture golden channel-word traces from P40 and P37 and implement `integration_e2e.rs`.


## 8. Key Decisions

| Question | Decision |
|---|---|
| When does Docker run in CI? | Only in nightly / manual runs, gated by `VAGC_AVAILABLE=1` |
| Fixture format | JSON (human-readable, diffable in PR review) |
| AGC scale-factor conversion | Central `from_agc_word` / `from_agc_dword` utility in `agc-test/src/oracle/` |
| Tolerance source | AGC alarm thresholds and convergence checks from assembly listing |
| Fixture freshness | Fixtures are regenerated and committed when the reference scenario changes; diffs are reviewed as part of the PR |
