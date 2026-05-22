# AGC-in-Rust: Testing Strategy with VirtualAGC as Oracle

## Overview

The primary verification challenge for this project is **D1 (Interpreter
Elimination)**: every navigation and guidance computation that was
originally written in the AGC interpretive language must be re-implemented
as a plain Rust `f64` function, and those functions must produce the same
results as the original software.

The strategy uses the Podman-based VirtualAGC (`yaAGC`) as a **reference
oracle**: drive it with controlled inputs, capture its outputs, and compare
against the Rust implementations. To keep CI fast and hermetic, fixtures
are **captured once locally** (requires a working VirtualAGC build) and
**committed as JSON** so the validation tests themselves run without
yaAGC, Podman, or any external dependency.

This document is the strategy contract. Per-fixture documentation lives
in [`docs/fixtures.md`](fixtures.md); per-program test catalogs live in
the individual `agc-test/tests/*.rs` files.

---

## 1. Three-layer test model

| Layer | Where it runs | Purpose | yaAGC needed? |
|---|---|---|---|
| **0. Pure unit tests** | `agc-core/src/**/*.rs::tests` | Per-function algorithmic correctness; edge-case sweeps. | No |
| **1. JSON fixture tests** | `agc-test/tests/*.rs` | Validate Rust outputs against committed reference data. | No — JSON committed |
| **2. Live VAGC capture** | `cargo run --features vagc-capture --bin capture_*` | Refresh JSON when a fixture must change. Run by a developer locally; output committed. | Yes (local) |
| **3. End-to-end channel-trace** | (future, MS-E7b) | Drive a full P61→P67 entry through yaAGC over the channel protocol and compare cycle-by-cycle to `agc-sim`. | Yes |

CI runs Layers 0 and 1 on every push. Layer 2 is a developer workflow.
Layer 3 is tracked in #35 (MS-E7b) and not yet implemented.

---

## 2. Layer 1 — JSON fixture tests

The bulk of the cross-AGC validation surface lives here. Each fixture
file is a JSON list of cases; each case has `inputs`, `expected`, and
per-output `tolerance`. A Rust test in `agc-test/tests/*.rs` loads the
JSON via `include_str!`, iterates cases, and asserts the Rust
implementation matches `expected` within tolerance.

Fixture files (committed under `agc-test/fixtures/`):

```
agc-test/fixtures/
  gravity_cases.json              -- earth_gravity / moon_gravity
  kepler_cases.json               -- math::kepler::kepler_step
  lambert_cases.json              -- math::lambert
  servicer_cycle_cases.json       -- services::average_g cycle
  orbit_propagation_cases.json    -- navigation::integration coast
  rendezvous_cases.json           -- guidance::rendezvous
  targeting_cases.json            -- guidance::targeting
  kalman_cases.json               -- Kalman filter
  vagc_constants.json             -- AGC-stored constants (extracted via yaYUL)
  entry/
    huntest_cases.json            -- P64 HUNTEST (Phase-3 scaffold; MS-E3b)
    huntest_inputs.toml           -- input definitions for capture_huntest
```

Fixture provenance:
- **Analytically computed** values (Lambert, Kepler) — derived from first
  principles using the same physical constants as the Rust implementation
  and an independent oracle (textbook formulas, Python `poliastro`, etc.).
- **VAGC-captured** values (entry/* and any future huntest / upcontrol /
  glim sets) — produced by the routine-level capture harness in §3.

See [`docs/fixtures.md`](fixtures.md) for per-fixture content.

---

## 3. Layer 2 — Routine-level VAGC fixture-capture harness

### 3.1 Why it exists

The entry-guidance B-track issues (#32 MS-E3b, #33 MS-E4b, #34 MS-E6b)
each need a small JSON fixture (~6 cases per routine) that records the
real Comanche055 AGC's HUNTEST / UPCONTRL / GLIM inputs and outputs at
representative entry states. The committed JSON then validates the Rust
re-implementations.

### 3.2 Architecture (four phases)

```
[ Phase 0 ]  ~/virtualagc/Comanche055/MAIN.agc            (source)
              │
              │  agc-test/scripts/assemble_comanche055.sh
              ▼
            MAIN.agc.bin (73 728 B rope) + MAIN.agc.lst (text symtab)

[ Phase 1 ]  agc-test/src/vagc_harness.rs (offline data layer)
              ├── Symtab::load(*.lst)            symbol → AgcAddress
              ├── CoreImage::load/save(*.core)   yaAGC's text dump format
              ├── CoreImage::read_sp/write_sp    single-precision word I/O
              ├── CoreImage::read_dp/write_dp    double-precision pair I/O
              └── read_scaled/write_scaled       AGC fixed-point ↔ f64

[ Phase 2 ]  agc-test/src/vagc_harness.rs::YaAgcRun (subprocess wrapper)
              ├── RunMode::WallClockDump        --nodebug + --dump-time=N
              └── RunMode::Debugger             --command=FILE: BREAK / CONT
                                                / COREDUMP / QUIT

[ Phase 3 ]  agc-test/src/bin/capture_<routine>.rs (developer-only)
              ├── reads cases from TOML
              ├── per-case: patch core ▶ run yaAGC ▶ read back ▶ write JSON
              └── gated behind `vagc-capture` Cargo feature
                  (so CI never builds this binary or its `toml` dep)

[ Phase 4 ]  agc-test/tests/entry_fixtures.rs (CI-visible test)
              ├── loads JSON via include_str!
              └── asserts Rust implementation matches expected ± tolerance
```

### 3.3 Adding a new captured fixture

To add a new routine (say, UPCONTRL for #33 MS-E4b):

1. Duplicate `agc-test/src/bin/capture_huntest.rs` → `capture_upcontrol.rs`.
   Update the routine name and the routine-specific case patcher.
2. Add a `[[bin]]` entry to `agc-test/Cargo.toml` with
   `required-features = ["vagc-capture"]`.
3. Create `agc-test/fixtures/entry/upcontrol_inputs.toml` defining the
   variables (with AGC name, B-scale, SP/DP, input/output) and the
   case grid.
4. Run the capture:
   ```sh
   bash agc-test/scripts/assemble_comanche055.sh   # one-time
   cargo run -p agc-test --features vagc-capture --bin capture_upcontrol -- \
       agc-test/fixtures/entry/upcontrol_inputs.toml \
       agc-test/fixtures/entry/upcontrol_cases.json
   ```
5. Add an `upcontrol_fixtures_match` test in
   `agc-test/tests/entry_fixtures.rs` that loads the JSON and drives
   `agc_core::guidance::entry::upcontrol_step` for each case.
6. Commit the JSON; CI now validates that routine.

### 3.4 Phase-3 scaffold caveat (current state)

`capture_huntest` builds and exercises the full pipeline (TOML → patch
core → yaAGC → dump → JSON), but it does **not yet drive the AGC to
actually execute HUNTEST**. Reaching HUNTEST in flight requires
DSKY + PIPA scripting that is the subject of #35 (MS-E7b). Without
it, the AGC is in its prelaunch state when erasable is patched, so
the captured "outputs" round-trip the inputs without any AGC math.

The committed `huntest_cases.json` therefore reflects the round-trip
identity, and `huntest_fixtures_round_trip` asserts this in CI.
When MS-E7b lands, the per-routine binaries swap to "drive AGC
through P63 → 0.05g → P64 → COREDUMP at routine exit" and the
assertion logic flips to comparing `compute_ld_command`'s output
to the captured `expected`. The surrounding pipeline (TOML, JSON,
CI loader, vagc_harness library) stays the same.

---

## 4. Layer 3 (future) — End-to-end channel-trace

Tracked in #35 (MS-E7b). The intended approach:

- Drive yaAGC through a complete P61 → P67 entry scenario by
  acting as a fake peripheral on yaAGC's socket-based I/O protocol
  (port 19697 by default, configurable via `--port=N`).
- Inject scripted PIPA / CDU channel words at the correct timing.
- Capture all AGC → peripheral channel writes (RCS jet commands,
  DSKY register updates, alarm codes).
- Replay the same scenario through `agc-sim` and assert the channel-
  word trace matches cycle-by-cycle.

The channel protocol is a 4-byte line-oriented binary:

```
[channel: u8] [value_hi: u8] [value_lo: u8] [0x00]
```

Relevant channels for driving / observing entry:

| Channel | Direction | Meaning |
|---|---|---|
| 010 | AGC → periph | DSKY display relay |
| 014 | periph → AGC | PIPA X accumulator |
| 015 | periph → AGC | PIPA Y |
| 016 | periph → AGC | PIPA Z |
| 030–033 | AGC → periph | RCS jet commands |
| 030–035 | periph → AGC | CDU gimbal angles |

This is a meaningful chunk of work in its own right (~3–5 days per
the harness plan). The current `vagc_harness::YaAgcRun` wrapper
covers process orchestration; the channel-protocol client would be
a new module (`agc-test/src/vagc_channel.rs`) consuming it.

---

## 5. AGC fixed-point conversion

`agc-test/src/agc_convert.rs` provides:

```rust
pub fn from_agc_word(raw: u16, scale: i8) -> f64;
pub fn from_agc_dword(hi: u16, lo: u16, scale: i8) -> f64;
pub fn to_agc_dword(value: f64, scale: i8) -> (u16, u16);
```

The `scale` parameter is the **LSB exponent**. For an AGC variable
documented with B-scaling B+N (max value = 2^N), pass
`scale = N − 28` for DP variables — see the module docs and worked
examples in `agc-test/src/agc_convert.rs`.

Scale factors per erasable address are documented in the Comanche055
listing comments (now also dumped to `~/virtualagc/Comanche055/MAIN.agc.lst`
by the assemble script) and in `~/virtualagc/Comanche055/ENTRY_LEXICON.agc`
for entry-guidance variables.

The capture binaries use `vagc_harness::ScaledVar` (which bundles the
address, scale, and SP/DP flag) and `read_scaled` / `write_scaled` to
hide the conversion boilerplate.

---

## 6. Tolerance and acceptance criteria

Exact bit-for-bit `f64` agreement with AGC fixed-point results is
**not** the goal. Tolerances are defined based on what the original
software itself accepted (convergence checks, alarm thresholds):

| Computation | Tolerance |
|---|---|
| Kepler solver | True anomaly within 1×10⁻⁹ rad |
| Lambert targeting | ΔV vector within 0.1 m/s |
| SERVICER cycle (2 s) | Position < 1 m, velocity < 0.01 m/s |
| HUNTEST (#32 MS-E3b) | L/D within 1×10⁻⁴, range-to-go within 1×10⁻³ rev |
| UPCONTRL (#33 MS-E4b) | L/D within 1×10⁻⁴ |
| PREDICT3 / GLIM (#34 MS-E6b) | L/D within 1×10⁻⁴, range-to-go within 1×10⁻³ rev |
| Entry end-to-end (MS-E7b) | Cross-range landing error < 1 km |

Per-fixture tolerances live inside the JSON case (`tolerance` map).
The acceptance constants for entry guidance derive from
`REENTRY_CONTROL.agc` thresholds (e.g. the `25NM` convergence check on
line 1545 sets `HUNTEST_CONVERGED_KM`).

---

## 7. Quick reference — common commands

```sh
# One-time bootstrap: assemble Comanche055 (idempotent)
bash agc-test/scripts/assemble_comanche055.sh

# Run all host tests including JSON fixtures (CI does this)
cargo test -p agc-core -p agc-protocol -p agc-imu-platform -p agc-sim -p agc-test

# Capture a fixture (developer only; needs ~/virtualagc/ build)
cargo run -p agc-test --features vagc-capture --bin capture_huntest -- \
    agc-test/fixtures/entry/huntest_inputs.toml \
    agc-test/fixtures/entry/huntest_cases.json

# Run just the entry fixture tests
cargo test -p agc-test --test entry_fixtures
```

---

## 8. Key decisions

| Question | Decision |
|---|---|
| When does yaAGC run in CI? | Never — only Layer 2 capture needs it, and that's a developer workflow. CI runs Layers 0 + 1. |
| Fixture format | JSON (human-readable, diffable in PR review). TOML for capture input case-lists (better for hand-editing nested cases). |
| AGC scale-factor conversion | Central `agc_convert::{from_agc_word, from_agc_dword, to_agc_dword}` utility. |
| yaAGC capture mode | File-based `core` dump (text, `%06o\n` per word) via `--dump-time` or `--command=FILE` with `COREDUMP filename`. Socket-based channel protocol is reserved for Layer 3 (#35 MS-E7b). |
| Fixture freshness | Fixtures are regenerated and committed when the reference scenario changes; diffs reviewed in the PR. CI never auto-refreshes. |
| Tolerance source | AGC alarm thresholds and convergence checks from `REENTRY_CONTROL.agc` and other Comanche055 listings. |
