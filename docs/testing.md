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
| **3. End-to-end channel-trace** | (deferred — see §4 and #49) | Drive a full P61→P67 entry through yaAGC over the channel protocol and compare cycle-by-cycle to `agc-sim`. | Yes |
| **Scenario-API mission tests** | `agc-test/tests/phase_*.rs`, `full_mission.rs` (#23 track) | Declarative end-to-end mission tests against `agc-sim`. See §5 for the API. | No |

CI runs Layers 0 and 1 plus the Scenario-API mission tests on every
push. Layer 2 is a developer workflow. Layer 3 is deferred — see §4.

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

> **Update post-MS-E7i:** the live channel-trace track was attempted
> end-to-end via the MS-E7c–i sub-track (#36 through #45) and is now
> deferred indefinitely — see #49 for the standing parking-lot issue
> and `entry_channel_trace.md` for the diagnostic record. The trajectory-
> level math validation it was meant to provide is covered by per-routine
> textbook-reference fixtures (#32, #33, #34) instead.

---

## 5. Scenario API for end-to-end mission tests

The Scenario API at `agc-sim/src/scenario.rs` is the declarative test
driver for the end-to-end mission-testing track (#23). It composes a
typed list of `Event`s into a `Scenario`, then `run_scenario` walks the
events against an `(AgcState, SimHardware)` pair using the same soft-
executive pumps (`WaitlistPump`, `DapPump`, `T4Pump`,
`pump_pipa_into_state`, `pump_engine_to_hw`, `pump_rcs_to_hw`) that
`dsky_sim`'s render loop ticks.

This is orthogonal to Layers 1–3 — it is *not* a fixture format and
does *not* compare against yaAGC. It is the in-Rust counterpart of
"hand-typed integration test" with an ergonomic builder and a uniform
failure model.

### 5.1 Data model

```text
Scenario {
    name, events,
    tick_cs,          // default 10 (100 ms) — SERVICER / DAP tick
    coast_step_cs,    // default 6_000 (60 s) — coast outer step, MS-T2
}
Event {
    SeedState(SeedStateSpec)              // csm + met + refsmmat
    SeedGroundTruth(StateVector)          // executor-held ground truth (MS-T2)
    AdvanceMet(SimDuration)               // walk the executive forward
    AdvanceCoast(SimDuration)             // gravity-driven ground truth (MS-T2)
    KeyPress(Key)                         // crew keystroke (one tick after)
    UplinkWord(u16)                       // ScriptedUplink push (one tick after)
    OpticsSighting { star_id }            // STUB until MS-T3
    LandmarkSighting { table, index }     // STUB until MS-T3
    ExpectMajorMode(u8)
    ExpectDsky(DskyExpect)                // verb / noun / r0/r1/r2 / flashing — all optional
    ExpectCsmStateClose { ground_truth, pos_tol_m, vel_tol_m_s }
    ExpectAgcMatchesGroundTruth { pos_tol_m, vel_tol_m_s }  // (MS-T2)
    ExpectAlarm(u16)
    Comment(&'static str)                 // documentation in test traces
}
```

`SimDuration(u32)` wraps mission **centiseconds** (the AGC's native
unit, matching `Met(u32)`). Construct via `SimDuration::cs(n) /
ms(n) / seconds(n) / minutes(n)`. Using a dedicated wrapper instead of
`std::time::Duration` makes cs-aligned time arithmetic exact and
const-friendly.

### 5.2 Builder

`ScenarioBuilder` exposes one typed method per Event variant — no
generic `event(Event)` escape hatch, so renames stay caught by the
type checker. It also provides sugar helpers for common DSKY
sequences:

| Method | What it pushes |
|---|---|
| `.key(Key)` / `.keys(&[Key])` | `KeyPress` events |
| `.digit(u8)` / `.digits(u32)` | `KeyPress(Digit(...))` MSB-first |
| `.enter()` / `.pro()` / `.verb()` / `.noun()` | Single-key shortcuts |
| `.verb_noun(u8)` | Verb-then-two-digits (no ENTR) |
| `.v25_load_three(noun, [i32; 3])` | Full V25 Nxx ENTR +v ENTR +v ENTR +v ENTR sequence with signed values |
| `.v71_p27_block_update(addr, &[(sign, mag)])` | Full V71 ENTR addr ENTR count ENTR ±v ENTR-each sequence |
| `.advance(SimDuration)` / `.advance_coast(SimDuration)` | Pumps for the requested duration |
| `.seed_ground_truth(StateVector)` | Initialise the executor's ground-truth state (MS-T2) |
| `.uplink_word(u16)` / `.optics_sighting(u8)` / `.landmark_sighting(...)` | One-shot inputs |
| `.comment(&'static str)` | Trace marker |
| `.expect_major_mode(u8)` / `.expect_dsky(DskyExpect)` / `.expect_csm_state_close(...)` / `.expect_alarm(u16)` | Assertions evaluated at the event's position |
| `.expect_agc_matches_ground_truth(pos_tol_m, vel_tol_m_s)` | Compare AGC `csm_state` against the executor's ground truth (MS-T2) |
| `.tick_cs(u32)` | Override the default 10 cs (must not exceed `DAP_PERIOD_CS`) |
| `.coast_step_cs(u32)` | Override the default 60 s coast outer step (MS-T2) |
| `.build()` | Produce the `Scenario` |

The `seed_state()` sub-builder returns a `SeedStateBuilder` that
exposes `.position_km(x, y, z)`, `.velocity_m_s(x, y, z)`, `.frame(...)`,
`.met(Met)`, `.refsmmat([[f64; 3]; 3])`, `.refsmmat_identity()`,
`.from_state_vector(StateVector)`, and `.done()` to push the
`Event::SeedState` and return the parent builder.

### 5.3 Executor semantics

`pub fn run_scenario(scenario: &Scenario, state: &mut AgcState, hw: &mut SimHardware)`
walks the events in order. A private `RunContext` owns the three
pumps; test code never sees them.

For each `AdvanceMet(dur)` (or each `KeyPress` / `UplinkWord`'s
single post-event tick), the executor runs this exact sequence per
slice:

```text
state.time = Met(state.time.0 + tick_cs);
hw.timers.set_time(state.time.0);
hw.tick(tick_s);
pump_pipa_into_state(state, hw);
dap_pump.tick(state, hw);
waitlist_pump.tick(state, hw);
t4_pump.tick(state, hw);
pump_engine_to_hw(state, hw);
pump_rcs_to_hw(state, hw);
```

PIPA before DAP so `dap_step` sees fresh CDU/PIPA; T4 after Waitlist
so an uplink word that lands this slice doesn't race a pending V37;
engine and RCS mirror last so the HAL reflects the final staging.

`run_scenario` asserts `tick_cs <= DAP_PERIOD_CS` at entry to catch
configurations that would skip DAP cycles.

**For `AdvanceCoast(dur)`** (MS-T2 — coast, no thrust), the executor
runs a two-tier loop. Per outer step (`coast_step_cs`, default 60 s):

1. Advance the executor-held ground truth via
   `physics::advance_ground_truth(spacecraft, &mut gt, coast_step_s)`
   (RK4 Cowell, Earth J2 + Moon third-body — see §5.7).
2. Run one SERVICER cycle's worth of inner ticks (200 cs at the
   default `tick_cs = 10`) using a coast-mode pump sequence that
   **skips** `hw.tick(dt_s)`, `dap_pump`, `pump_engine_to_hw`, and
   `pump_rcs_to_hw`. PIPA + Waitlist + T4 still run so SERVICER
   processes its gravity integration with PIPA = 0 (no thrust).
3. The remaining time in the outer step advances `state.time` and
   the Waitlist countdown in a single bump.

Per-event log line: `[scenario {name}] coast +{dur} → MET {s}s,
ground_truth {present|absent}`.

### 5.4 Failure model

`Expect*` failures **panic** with a uniform message:

```text
scenario "<name>": event #<idx> (<variant>) failed at MET <cs>cs (<s>s):
  <reason>; expected <x>, got <y>
```

Panics are idiomatic for `#[test]` code and put the failure right in
the test output. `run_scenario` returns `()`; no `Result`.
`ExpectDsky` with a NaN register fast-fails with "DSKY register NaN
— likely uninitialised noun" rather than producing a misleading
comparison.

### 5.5 Deferred variants

`OpticsSighting` and `LandmarkSighting` are **no-op stubs** with a
one-line stderr warning until MS-T3 (#26) wires the sensor sims.
Phase-test authors can stage full Apollo-8 scenarios against the
API today and have them progress as MS-T3 lands, without
panic-on-compose.

`AdvanceCoast` is **no longer a stub** as of MS-T2 (PR #53,
amendments in this PR). It runs the two-tier loop documented in
§5.3 against an executor-held ground truth (seeded by
`SeedGroundTruth`).

### 5.6 Ground-truth oracle (MS-T2)

`physics::advance_ground_truth` uses
`agc_core::navigation::integration::propagate_coast` (RK4 Cowell,
4th-order, with J2 + Moon third-body) as the ground-truth oracle —
**not** `kepler_step` (pure two-body, originally specified in #25).
The choice was made during MS-T2 implementation so the AGC-vs-ground-
truth comparison isolates integrator order (the SERVICER's
trapezoidal `average_g_step` vs RK4 Cowell) under a shared physics
model.

Three tests pin the three layers of validation:

| Test | Location | What it pins |
|---|---|---|
| `tc_phys_advance_ground_truth_subdivision_self_consistency` | `agc-sim/src/physics.rs::tests` | 90 × 60s `advance_ground_truth` matches single `propagate_coast(5400s)` within 1 km — sub-step accumulation accuracy. |
| `tc_phys_coast_24h_leo_vs_kepler_two_body` | `agc-sim/src/physics.rs::tests` | 24h `advance_ground_truth` vs hourly `kepler_step` stays within 2.5 Mm / 2.5 km/s — pins the J2-secular drift physics-model gap. Regression catch for ground-truth integrator changes. |
| `tc_ms_t2_coast_24h_agc_tracks_ground_truth` | `agc-test/tests/p70_coast_24h_leo.rs` | AGC `csm_state` from SERVICER stays within 5 km / 5 m/s of `advance_ground_truth` over 24h. The MS-T2 exit criterion (amended from #25's original "1 km vs `kepler_step`" wording — see PR for the physics derivation). |

#25 was amended to reflect the implementation choice. The original
"1 km vs `kepler_step`" wording was unphysical for any propagator
including J2 (~1.9 Mm/day phase divergence from the 0.144 %
J2-corrected mean motion in LEO).

### 5.7 Worked example — `p40_sps_burn.rs`

The MS-T1 proof point is `agc-test/tests/p40_sps_burn.rs`. It
composes three sub-scenarios that share an `(AgcState, SimHardware)`
pair to preserve the intermediate-state assertions that cross the
V25-N81 / P40-init / PRO boundaries:

```text
phase1a: seed state vector via V71 P27 block update +
         select P30, load TIG (V25 N33), load ΔV (V25 N81)
phase1b: select P40 (V37 ENTR 40 ENTR), tick the executive once
phase1c: arm the burn (PRO)
[direct burn loop: TIG jump + per-tick assertion of ignition / cutoff]
```

The burn loop itself is kept as direct state manipulation because the
test asserts within a single DAP cycle of TIG — finer-grained than
the event level. `ScenarioBuilder` is for declarative event
sequences; intra-cycle assertions stay direct.

### 5.8 Adding a new mission-phase test

1. Pick a `phase_<name>.rs` file under `agc-test/tests/` matching the
   layout in `specs/end-to-end-mission-testing-plan.md` §7.
2. Construct a single `Scenario` (or split into sub-scenarios per the
   P40 example) with a `seed_state()`, the relevant key /
   uplink-word inputs, and one `expect_*` per phase invariant.
3. Run it: `run_scenario(&scenario, &mut state, &mut hw);`.
4. Add a `cargo test` smoke check to CI by leaving the test
   un-`#[ignore]`d.

The end-to-end mission-testing plan (#23) sequences these
phase tests, the inter-phase handoff tests, and the full-mission
walkthrough as MS-T4 through MS-T7.

---

## 6. AGC fixed-point conversion

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

## 7. Tolerance and acceptance criteria

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

## 8. Quick reference — common commands

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

## 9. Key decisions

| Question | Decision |
|---|---|
| When does yaAGC run in CI? | Never — only Layer 2 capture needs it, and that's a developer workflow. CI runs Layers 0 + 1. |
| Fixture format | JSON (human-readable, diffable in PR review). TOML for capture input case-lists (better for hand-editing nested cases). |
| AGC scale-factor conversion | Central `agc_convert::{from_agc_word, from_agc_dword, to_agc_dword}` utility. |
| yaAGC capture mode | File-based `core` dump (text, `%06o\n` per word) via `--dump-time` or `--command=FILE` with `COREDUMP filename`. Socket-based channel protocol is reserved for Layer 3 (#35 MS-E7b). |
| Fixture freshness | Fixtures are regenerated and committed when the reference scenario changes; diffs reviewed in the PR. CI never auto-refreshes. |
| Tolerance source | AGC alarm thresholds and convergence checks from `REENTRY_CONTROL.agc` and other Comanche055 listings. |
