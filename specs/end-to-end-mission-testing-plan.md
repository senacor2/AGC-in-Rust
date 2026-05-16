# End-to-End Mission Testing Plan

**Status**: Approved
**Scope**: Build a layered test harness in `agc-test` that exercises the AGC core through an Apollo 8 mission profile from on-orbit TLI initiation to drogue deploy at entry final phase. Identify and plan the `agc-sim` extensions (scenario runner, ground-truth propagator, attitude model, sensor simulators) and core additions (lunar landmark table) required to make this possible.
**Reference mission**: Apollo 8 abbreviated profile (CSM-only, lunar orbit + return).
**Capstone deliverable**: `agc-test/tests/full_mission.rs` — one test that flies the full mission and passes in CI.

---

## 1. Goal

Verify functional completeness of the AGC core by flying the canonical CSM-only "earth-to-moon-and-back" mission entirely through the `agc-sim` HAL, asserting at each phase boundary that AGC commands and onboard state match the expected trajectory. The full mission walkthrough (MS-T7) is the project's headline acceptance criterion.

## 2. Reference mission — Apollo 8

| Phase | Programs | Duration | Trajectory milestone |
|---|---|---|---|
| On-orbit start (CMC IDLING) | P00 | seconds | LEO circular, 185 km altitude |
| TLI burn | P15 monitor + P40 | ~5 min | Vp ≈ 10.8 km/s, C3 hyperbolic-w.r.t.-Earth |
| Translunar coast + MCC | P23 + P30 + P40 | ~3 days | Two MCC opportunities |
| Lunar Orbit Insertion | P30 + P40 | ~4 min | 60 nmi × 170 nmi lunar orbit |
| Lunar orbit nav + alignment | P22 + P52 | ~20 hours | 10 revolutions |
| Trans-Earth Injection | P30 (or P37) + P40 | ~3 min | Free-return trajectory home |
| Trans-Earth coast + MCC | P23 + P30 + P40 | ~2.5 days | One MCC opportunity |
| Entry | P61 → P67 | ~10 min | 36 000 ft/s entry interface, Pacific splashdown |

The plan does not run real-time; phases are stepped through at simulator speed (seconds, not days).

## 3. Current state

### `agc-sim` (host-side simulator)

| Module | LOC | Status |
|---|---|---|
| `hardware.rs` | 530 | Working HAL impl |
| `physics.rs` | 194 | **SPS-thrust-only** PIPA model; no gravity, no attitude, no drag |
| `runtime.rs` | 487 | "Soft executive" pumps (PIPA, engine, RCS, DAP, waitlist) |
| `dsky_ui.rs` | 690 | Interactive DSKY simulator binary |
| `scenario.rs` | **5 (stub)** | Empty — header comment only |

### `agc-test`

| Test | LOC | Status |
|---|---|---|
| `p40_sps_burn.rs` | 355 | **The only end-to-end test today** — drives P40 via keystrokes through `SimHardware`. Template for everything below. |
| `navigation_accuracy.rs` | 1305 | Fixture-based unit-grade tests |
| `dsky_interaction.rs` | 4 (stub) | Empty |
| `restart_recovery.rs` | 4 (stub) | Empty |
| `timing_compliance.rs` | 4 (stub) | Empty |

### Reusable tables already in `agc-core`

| Table | Source | Status |
|---|---|---|
| `STAR_CATALOG[37]` in `navigation/star_catalog.rs` | Comanche055/STAR_TABLES.agc | **Complete** |
| `LANDMARK_TABLE[9]` in `programs/p22.rs:227` | Earth landmarks | **Complete** |
| Lunar landmark table | — | **Missing** — required for P22 lunar orbit nav (MS-T3) |

## 4. Architecture: layered tests

Three layers, each independently valuable:

- **Layer 1 — Phase-level tests** (MS-T4, MS-T6). One test per mission phase. The test seeds the state vector, runs the relevant programs, asserts end-state. Template = `p40_sps_burn.rs`.
- **Layer 2 — Inter-phase handoff tests** (MS-T5). Validate clean transitions: P40 cutoff → P00 → next program, SOI crossings, etc.
- **Layer 3 — Full mission walkthrough** (MS-T7). One big test chains all phases end-to-end with `kepler_step` driven coast propagation. Verifies the AGC's onboard state remains consistent with ground truth across the day-scale coasts.

No real-time closed-loop simulation. The simulator's physics models PIPA pulses during powered flight; orbital coast propagation remains the AGC's job (via SERVICER + onboard gravity integrator) — which is the system under test.

## 5. Gaps blocking each layer

### 5.1 Already planned (other tracks)

| Gap | Layer | Planned in |
|---|---|---|
| Entry guidance math (P64/P65/P66/P67) | MS-T6 | #3 |
| Uplink path (UPRUPT + V70/V72/V73) | MS-T4 MCC scenarios | #11 |
| GMST + ECEF transforms | MS-T6 entry phase | #17 |

### 5.2 New to this plan

| Gap | Where | Milestone |
|---|---|---|
| Scenario runner (today a 5-LOC stub) | `agc-sim/src/scenario.rs` | MS-T1 |
| Ground-truth coast propagator | `agc-sim/src/physics.rs` (extend) | MS-T2 |
| Attitude dynamics model | `agc-sim/src/physics.rs` (extend) | MS-T3 |
| Star + landmark sensor simulators | `agc-sim/src/sensors.rs` (new) | MS-T3 |
| **Lunar landmark table** | `agc-core` — new module or extend P22 | MS-T3 |
| Atmospheric drag in `agc-sim` | `agc-sim/src/physics.rs` (extend) | MS-T6 |

### 5.3 To confirm during MS-T1 review (not assumed missing)

- Continuous-coast SERVICER cycling: the AGC must propagate state via the gravity integrator during multi-day coasts.
- SOI crossing logic: who switches `csm_state.frame` from `EarthInertial` to `MoonInertial` and back? Confirm during MS-T1.
- REFSMMAT propagation after P52: alignment-derived REFSMMAT must flow into SERVICER platform-to-inertial rotation.

## 6. Design decisions (locked)

1. **Reference mission**: Apollo 8 abbreviated profile.
2. **Scenario format**: pure Rust builder API (`ScenarioBuilder`). No YAML/JSON parser. Tests compose scenarios in code.
3. **Ground-truth propagator**: reuse `agc_core::math::kepler::kepler_step` directly from `agc-sim`. No duplication.
4. **Coverage cut**: include the full mission walkthrough (MS-T7) as the capstone deliverable.
5. **Star and landmark tables**: real Apollo-era data. The existing 37-star catalog and 9-entry Earth landmark table are reused as-is. A new lunar landmark table is created in this work (5–8 named craters: Aristarchus, Triesnecker, Copernicus, etc.).

## 7. Module layout

```
agc-sim/src/scenario.rs               (new content — replaces 5-LOC stub)
  pub struct Scenario { events: Vec<Event>, name: &'static str }
  pub enum Event {
      SeedState { csm: StateVector, met: Met, refsmmat: Mat3x3 },
      AdvanceMet(Duration),                // tick simulator forward
      AdvanceCoast(Duration),              // gravity-driven ground truth, no PIPAs
      KeyPress(Key),                       // crew keystroke
      UplinkWord(u16),                     // via ScriptedUplink (from #12)
      OpticsSighting { star_id: u8 },      // P51/P52
      LandmarkSighting { table: LandmarkTable, index: u8 },  // Earth or Moon
      ExpectMajorMode(u8),
      ExpectDsky { verb, noun, r0, r1, r2, tol_pct },
      ExpectCsmStateClose { ground_truth: StateVector, pos_tol_m, vel_tol_m_s },
      ExpectAlarm(u16),
      Comment(&'static str),               // documentation in test traces
  }
  pub struct ScenarioBuilder { ... }       // ergonomic builder API
  pub fn run_scenario(scenario, &mut AgcState, &mut SimHardware);

agc-sim/src/physics.rs                (extend — currently 194 LOC)
  pub enum GravityBody { Earth, Moon }
  pub struct Spacecraft {
      // existing SPS-only fields stay
      pub gravity_enabled: bool,
      pub current_body: GravityBody,
      pub atmosphere_enabled: bool,
      pub attitude: Attitude,              // quaternion + commanded
  }
  pub struct Attitude {
      pub q: [f64; 4],
      pub commanded_q: [f64; 4],
      pub slew_tau_s: f64,                 // first-order lag
  }
  pub fn advance_ground_truth(sc, state: &mut StateVector, dt);
  // Reuses agc_core::math::kepler::kepler_step for coast propagation.

agc-sim/src/sensors.rs                (new — ~250 LOC)
  pub struct StarSensor;                   // uses agc_core::navigation::star_catalog
  pub struct LandmarkSensor;               // uses LANDMARK_TABLE + LUNAR_LANDMARK_TABLE
  pub fn star_los_in_platform(star_id, attitude, refsmmat) -> Vec3;
  pub fn landmark_los_in_platform(table, index, csm_pos, attitude, refsmmat) -> Vec3;

agc-core/src/navigation/landmarks.rs  (new — ~150 LOC)
  // Lunar landmark table — selenocentric, MoonFixed coordinates.
  pub struct LunarLandmarkEntry {
      pub name: &'static str,
      pub lat_rad: f64,
      pub lon_rad: f64,
      pub alt_m: f64,                      // above mean lunar radius
  }
  pub const LUNAR_LANDMARK_TABLE: [LunarLandmarkEntry; 8] = [...];
  // Earth landmarks stay in programs/p22.rs (existing); may be moved here later.

agc-test/tests/phase_<name>.rs        (new — one per phase)
  phase_tli.rs              // P15 + P40 → hyperbolic state vector
  phase_translunar.rs       // P23 marks + P30 + P40 MCC
  phase_loi.rs              // P30 + P40 → lunar orbit
  phase_lunar_orbit.rs      // P22 nav (lunar landmarks) + P52 align
  phase_tei.rs              // P30 (or P37) + P40 → return trajectory
  phase_transearth.rs       // P23 marks + MCC
  phase_entry.rs            // P61 → P67 (depends on #3)

agc-test/tests/handoffs.rs            (new)
  // Mode transitions, SOI crossings, REFSMMAT propagation across P52→burn

agc-test/tests/full_mission.rs        (new — capstone, MS-T7)
  // Apollo 8 walkthrough, chains MS-T4 + MS-T6 phases end-to-end
```

## 8. Implementation milestones

### MS-T1 — Scenario runner infrastructure
- Flesh out `agc-sim/src/scenario.rs` with `Scenario`, `Event`, `ScenarioBuilder`, `run_scenario`.
- Refactor `agc-test/tests/p40_sps_burn.rs` to use the new API as a proof point. Should reduce LOC.
- Documentation: extend `docs/testing.md` with a "Scenario API" section.
- **Confirm during this milestone** (per §5.3): continuous-coast SERVICER cycling, SOI crossing ownership, REFSMMAT propagation after P52. If any of these turn out to need code changes, raise follow-up issues and link to this one.
- **Exit criterion**: P40 test refactored via the new API and passing; no behaviour change; section in `docs/testing.md` documenting the API.

### MS-T2 — Ground-truth orbital propagator
- Extend `agc-sim/physics.rs` with `GravityBody`, gravity-driven coast propagation. Reuse `agc_core::math::kepler::kepler_step`. SOI switch in ground truth tracks the AGC's frame.
- New `Event::AdvanceCoast(dt)` integrates ground-truth state without producing PIPA pulses (PIPAs measure non-gravitational accel only; coast = no pulses).
- **Exit criterion**: a 24-hour coast from LEO matches a `kepler_step` reference within 1 km position drift; AGC's internal state stays within tolerance against ground truth.

### MS-T3 — Attitude, sensors, and lunar landmark table
- Add `Attitude` (quaternion + commanded + first-order-lag slew) to `agc-sim/physics.rs`. Match DAP's commanded attitude with a configurable time constant.
- New `agc-sim/src/sensors.rs`:
  - `star_los_in_platform(star_id, attitude, refsmmat)` returns the line-of-sight unit vector in the IMU stable-member frame, using `STAR_CATALOG`.
  - `landmark_los_in_platform(...)` for both Earth landmarks (existing `LANDMARK_TABLE`) and lunar landmarks (new).
- New `agc-core/src/navigation/landmarks.rs`:
  - `LunarLandmarkEntry` and `LUNAR_LANDMARK_TABLE` (8 named features: e.g., Tycho, Copernicus, Aristarchus, Censorinus, Maskelyne F, etc., with selenographic coords from authoritative sources). Document each with reference to the Apollo lunar landmark series (LM-1 … LM-N).
  - Wire into `programs/p22.rs` as an alternate table consulted when `csm_state.frame == MoonInertial`.
- `Event::OpticsSighting { star_id }` and `Event::LandmarkSighting { table, index }` produce CDU readings the AGC consumes.
- **Exit criterion**: P52 IMU realignment scenario from two star sightings produces a REFSMMAT close to a known truth matrix; P22 with one lunar landmark in lunar orbit produces a Kalman update consistent with the seeded ground-truth state.

### MS-T4 — Phase-level integration tests (Layer 1, non-entry)
- One test per mission phase (excluding entry):
  - `phase_tli.rs` — P15 monitor + P40 SPS burn → hyperbolic state.
  - `phase_translunar.rs` — P23 cislunar nav marks + P30 MCC + P40.
  - `phase_loi.rs` — P30 LOI targeting + P40 → lunar orbit.
  - `phase_lunar_orbit.rs` — P22 with lunar landmarks + P52 alignment.
  - `phase_tei.rs` — P30 or P37 + P40 → return trajectory.
  - `phase_transearth.rs` — P23 marks + MCC.
- Each test composes a `Scenario` and asserts end-state via `ExpectCsmStateClose` plus phase-specific DSKY assertions.
- **Exit criterion**: all six phase tests pass.

### MS-T5 — Inter-phase handoff tests (Layer 2)
- `agc-test/tests/handoffs.rs`:
  - P40 cutoff → P00 → V37 → next program — no state corruption.
  - P23 marks update CSM state vector → P30 reads the updated state — MCC targeting consumes uplink-corrected nav.
  - SOI crossing Earth → Moon and Moon → Earth — frame transitions are clean; gravity body switches; nav stays consistent.
  - P52 alignment → next burn — new REFSMMAT propagates into SERVICER platform-to-inertial rotation.
- **Exit criterion**: all four handoffs pass with explicit state-invariant assertions.

### MS-T6 — Entry phase test
- `phase_entry.rs` — exercises P61 → P67 in `agc-sim` end-to-end with atmospheric drag enabled.
- Requires atmospheric drag in `agc-sim/physics.rs` (small addition — exponential atmosphere from entry-guidance MS-E1).
- **Depends on #3 (entry guidance) complete**, specifically MS-E7.
- **Exit criterion**: closed-loop entry scenario lands within target footprint; aligns with entry-guidance MS-E7 exit criterion.

### MS-T7 — Full mission walkthrough (Layer 3 capstone)
- `agc-test/tests/full_mission.rs` chains MS-T4 + MS-T6 phases end-to-end via the scenario builder.
- One test, on the order of 800–1200 LOC; runs in seconds.
- Verifies onboard state vs. ground-truth state at every phase boundary.
- **Exit criterion**: full mission test passes; documented as the project's functional-completeness acceptance criterion in `README.md`.

## 9. Test strategy

- **Layer 1 (MS-T4 + MS-T6)** delivers most of the value: one test per phase, fast, focused, debuggable.
- **Layer 2 (MS-T5)** catches integration mistakes that only show at boundaries.
- **Layer 3 (MS-T7)** is the credibility test — `cargo test full_mission` becomes the project's functional acceptance gate.
- No new VirtualAGC fixture capture in this plan — Layer 1 tests use phase-local expected values; Layer 3 verifies internal consistency.
- All tests run in CI without `VAGC_AVAILABLE` flag.

## 10. Sequencing and dependencies

| Milestone | Depends on | Can start when |
|---|---|---|
| MS-T1 | None | Now |
| MS-T2 | MS-T1 | After MS-T1 |
| MS-T3 | MS-T2 | After MS-T2 |
| MS-T4 | MS-T3, #11 (uplink for MCC) | After MS-T3 and uplink track lands |
| MS-T5 | MS-T4 | After MS-T4 |
| MS-T6 | MS-T3, #3 entry guidance complete | After entry track lands |
| MS-T7 | MS-T5, MS-T6 | Last |

MS-T1 has no dependencies and can begin immediately.

## 11. GitHub issue seed

New label `mission-testing` (color `#c2e0c6`) — scopes all issues in this plan. Existing labels reused.

| Title | Labels |
|---|---|
| End-to-end mission testing (Apollo 8) — implementation tracking | `mission-testing`, `milestone` |
| MS-T1: Scenario runner infrastructure | `mission-testing`, `milestone`, `infrastructure`, `enhancement` |
| MS-T2: Ground-truth orbital propagator in agc-sim | `mission-testing`, `milestone`, `infrastructure`, `enhancement` |
| MS-T3: Attitude, sensors, and lunar landmark table | `mission-testing`, `milestone`, `infrastructure`, `enhancement` |
| MS-T4: Phase-level integration tests (Layer 1, non-entry) | `mission-testing`, `milestone`, `enhancement` |
| MS-T5: Inter-phase handoff tests | `mission-testing`, `milestone`, `enhancement` |
| MS-T6: Entry phase end-to-end test | `mission-testing`, `milestone`, `enhancement` |
| MS-T7: Full Apollo 8 mission walkthrough (capstone) | `mission-testing`, `milestone`, `enhancement` |
