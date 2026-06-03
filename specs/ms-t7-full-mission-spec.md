# MS-T7 — Apollo 8 full-mission walkthrough capstone

**Status**: Draft, for developer consumption
**Implements**: GitHub issue #30 (parent: end-to-end mission testing, GH #24)
**Target files**:
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-test/tests/full_mission.rs` (new — single integration test, ~800–1000 LOC)
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/agc-test/src/entry_scenario.rs` (one minor extraction — see §4)
- `/Users/Juergen.Schiewe/dev/AGC-in-Rust/README.md` (one paragraph under References / Testing — see §5)

**Predecessors**: MS-T4 (`phase_tli` / `phase_translunar` / `phase_loi` / `phase_lunar_orbit` /
`phase_tei` / `phase_transearth`), MS-T5 handoffs (issue #28, merged), MS-T6
`phase_entry` (issue #29, merged).

---

## 1. Goal and acceptance criterion

### 1.1 Goal

A single `#[test]` function that chains all seven Apollo 8 mission phases —
TLI, trans-lunar coast, LOI, lunar orbit, TEI, trans-earth coast, entry — end
to end on the **same** `AgcState` and `SimHardware` instance, driven by the
`agc-sim::ScenarioBuilder` API. The test is the project's functional
completeness gate: if it passes, the AGC-in-Rust port can fly an Apollo 8
profile through the navigation stack from launch to drogue deploy with no
manual intervention beyond the documented DSKY key sequences.

This is a **walkthrough**, not a high-fidelity end-to-end propagation.
Phase boundaries reseed `state.csm_state` to the historically correct
checkpoint values, identical to the per-phase tests. The Moon-ephemeris-epoch
limitation (`moon_position` hardcoded to Apollo 11 / July 16 1969 vs.
Apollo 8 launch December 21 1968) makes a continuous 146-hour propagation
physically meaningless; the checkpoint-reseed pattern is the documented
architectural workaround (see `phase_translunar.rs` module doc).

### 1.2 Acceptance criterion (the single anchor assertion)

The test passes iff **all** of the following are true after the seven phases
have been chained:

1. `state.entry.drogue_deployed == true` at the end of the entry phase.
2. `state.entry.phase == EntryPhase::Final`.
3. The great-circle miss distance from the splashdown sub-satellite point
   to the configured target is `<= 3000 km` (the `setup_state_lunar_return`
   threshold, matching `entry_e2e.rs` and `phase_entry.rs` — issue #30 is a
   *walkthrough* gate, not a new accuracy gate).
4. `state.alarm.code == 0` at every phase boundary (no AGC restart or program
   alarm fired during the run).
5. Every per-phase post-condition listed in §3 holds (these are the
   "verifies onboard state vs. ground-truth state at every phase boundary"
   checks called out in issue #30).

Concretely the test ends with:

```rust
let (lat, lon) = sub_satellite_lat_lon(&state);
let miss_km = haversine_km(lat, lon, state.entry.target_lat_rad, state.entry.target_lon_rad);
assert!(state.entry.drogue_deployed, "drogue must deploy");
assert_eq!(state.entry.phase, EntryPhase::Final);
assert!(miss_km <= 3000.0, "miss = {miss_km:.0} km exceeds 3000 km gate");
assert_eq!(state.alarm.code, 0);
```

This is the *only* user-visible success criterion. The granular per-phase
post-conditions in §3 act as fast-fail diagnostics: if the chain breaks, the
test reports which phase boundary went wrong before reaching the splashdown
check.

---

## 2. State and hardware lifecycle

### 2.1 Single mutable `state` and `hw`

```rust
let mut state: AgcState = AgcState::new();
let mut hw: SimHardware = SimHardware::new();
```

These are declared once at the top of the test and flow through every phase
unchanged. No `AgcState::new()` is called again inside the test body. This
matches the chain pattern used by `phase_loi.rs` and `phase_tei.rs` for
their internal sub-phases.

### 2.2 MET timeline — discrete, not continuous

**Decision (open question 2 confirmed):** MS-T7 does **not** maintain a
continuous MET across the 146-hour mission. Each phase resets
`state.time = Met(checkpoint_cs)` at its boundary, identical to the
per-phase tests. The reasoning:

- Continuous propagation would integrate ~146 hours of trajectory through
  `moon_position` hardcoded to the Apollo 11 epoch — physically wrong.
- The historical Apollo 8 MET checkpoints (TLI ignition T+2:50:41, LOI-1
  T+69:08:20, TEI T+89:19:16, EI T+146:46:12.8) are the test's reference
  clock; we honor them by jumping to each.
- The per-phase tests already work this way and are merged. MS-T7 chains
  them without changing this contract.

Each phase block in §3 starts with an explicit reseed of
`state.csm_state` and `state.time` to its checkpoint. The reseed is
performed by a `ScenarioBuilder` containing `seed_state()` + (where
appropriate) `seed_ground_truth()`, identical to the per-phase tests.

### 2.3 Pump usage

Coast phases use `ScenarioBuilder::advance_coast(...)` exclusively — the
scenario runner internally drives `WaitlistPump`, `DapPump`, PIPA pump, and
ground-truth propagation. The burn phases (LOI, TEI) require a manual
post-arm `dap_pump.tick(...)` loop because the burn loop must run faster
than the scenario builder's coast step; this is the pattern from
`phase_loi.rs` Phase D and `phase_tei.rs` Phase D, **inlined verbatim**.

### 2.4 Code reuse strategy

**Decision (open question 1 confirmed):** **Inline** the relevant
fixture-seed + key-sequence + assertion blocks from each phase test
directly into `full_mission.rs`. **Do not** extract a shared
`agc-test/src/mission_phases.rs` module.

Rationale: per-phase tests will continue to evolve independently (e.g. when
issue #51 lands, the synthetic reseeds in trans-lunar and trans-earth may
become unnecessary). Shared helpers couple the per-phase tests to the
walkthrough test and slow iteration on either side. The walkthrough is a
single read-from-top file that auditors can scan linearly to convince
themselves "yes, this chains seven historical mission phases on the same
AGC state".

The **one exception** is `phase_entry.rs::run_entry_phase`, which is
already a function. §4 specifies an extraction to make it callable with a
caller-supplied `AgcState`.

### 2.5 Test runtime budget

**Target wall-clock budget: < 30 s** for the full chain on a modern host.
Component breakdown:

- TLI + trans-lunar (six checkpoints, mostly coast): ~3 s
- LOI burn loop (~300 s sim time at 10 ms tick = 30 000 ticks): ~5 s
- Lunar orbit (five checkpoints + 8-rev oracle propagation): ~3 s
- TEI burn loop (~350 s sim time + 600 s post coast): ~6 s
- Trans-earth (five checkpoints): ~3 s
- Entry (up to 20 min sim time at 2 s ticks = 600 ticks): ~5 s

If wall-clock exceeds 60 s the developer should report it back; we may
need to compress the lunar-orbit or trans-earth coast windows (currently
600 s and 300 s respectively — both are sized for the per-phase test's
tolerance check and could be cut to 60 s in the walkthrough).

---

## 3. Phase-by-phase plan

For each phase, the spec states:
- **Reseed**: the SV constants and the `state.time` value to set.
- **Drive**: the DSKY/scenario events that exercise the phase.
- **Post-condition**: the boundary assertions to enforce.

All constants below are copied from the per-phase test files. Cite the
phase test as the source of truth in a comment, e.g.
`// constants from phase_tli.rs:53-79`.

### 3.1 Phase 1 — TLI (source: `agc-test/tests/phase_tli.rs`)

**Reseed**:
- Build `park_sv` via `parking_orbit_sv(PARKING_INSERTION_MET_CS)` —
  inline the helper from `phase_tli.rs:102-112`.
- `state.time = Met(PARKING_INSERTION_MET_CS)` via `seed_state()`.
- Seed ground truth to the same SV.

**Drive**:
1. V37 ENTR 15 ENTR → activate P15 (TLI monitor).
2. `.advance_coast(SimDuration::seconds(2))` — settle.
3. `.expect_major_mode(15)`, `.expect_dsky(V16 N44 non-flashing)`.
4. `.advance_coast(SimDuration::seconds(PARKING_COAST_SECONDS))` —
   coast 9 546 s to TLI ignition.
5. `.expect_agc_matches_ground_truth(5_000.0, 5.0)`.
6. **Outside the scenario**: compute `v_post_tli` arithmetically by adding
   `TLI_DV_MPS` prograde to `state.csm_state.velocity`. Verify
   `|v_post| ∈ [10_790, 10_890]` m/s.
7. Run a second scenario: `seed_state(post_tli_sv).seed_ground_truth(...).advance_coast(seconds(TLI_BURN_SECONDS + 3600)).expect_agc_matches_ground_truth(50_000.0, 5.0)`.

**Post-condition (phase boundary assertion)**:
- `state.csm_state.frame == EarthInertial`.
- Specific energy `ε > MAX_C3_ENERGY` (-2.5e6 J/kg).
- `r_hat · v > 0` (outbound).

### 3.2 Phase 2 — Trans-lunar coast (source: `agc-test/tests/phase_translunar.rs`)

**Reseed strategy**: this phase uses six sub-checkpoints (Phases 1–6 in
`phase_translunar.rs`). MS-T7 inlines **all six** sub-scenarios in order.
For each sub-scenario the developer copies the corresponding block from
`phase_translunar.rs:265-515`, using the constants `POST_TLI_MET_CS`,
`MCC2_MET_CS`, `MID_TRANSIT_MET_CS`, `HIGH_APOGEE_MET_CS`,
`POST_SOI_MET_CS`, `MCC4_MET_CS`, `LOI1_MET_CS`.

**Drive**: for each sub-checkpoint:
- `seed_state(...).seed_ground_truth(...)`.
- `.advance_coast(SimDuration::seconds(300))` (`200` for Phase 6).
- `.expect_agc_matches_ground_truth(1_000.0, 1.0)`.

The MCC ΔVs (MCC-2 = 2.35 m/s, MCC-4 = 0.43 m/s) are applied
arithmetically to the oracle SV before reseeding, using
`n_hat_perp_in_plane(r, v)` — inline this helper from
`phase_translunar.rs:234-239`.

**Post-condition (phase boundary assertion, after sub-Phase 6)**:
- `state.csm_state.frame == EarthInertial`.
- `‖r‖ ∈ [1.3e8, 1.75e8]` m (inbound on the ellipse).
- `‖v‖ ∈ [1_000, 2_500]` m/s.
- `r_hat · v_hat < 0` (inbound).
- Frame at sub-Phase 4b end was `MoonInertial` (intermediate check; allow
  the assertion only if 4b ran — i.e. include it inline like
  `phase_translunar.rs:435`).

### 3.3 Phase 3 — LOI (source: `agc-test/tests/phase_loi.rs`)

**Reseed**:
- Build `sv_pre_peri` via `pre_pericynthion_sv()` — inline the helper
  from `phase_loi.rs:191-217`.
- The seed MET is `LOI1_TIG_MET_CS - SETTLE_CS` (= `TIG - 300s`); the SV
  carries epoch `LOI1_TIG_MET_CS`. The intentional inconsistency is
  documented in `phase_loi.rs:255-263`.

**Drive** — five inline sub-phases (A, B, C, D, E from `phase_loi.rs`):

- **Phase A** (setup): `seed_state(sv_pre_peri).met(TIG - SETTLE_CS).refsmmat_identity().command_attitude([1,0,0,0])`, then V37 ENTR 30 ENTR + V25 N33 ENTR + (LOI1_TIG_H, LOI1_TIG_M, LOI1_TIG_S100) digits via the scenario builder's `digits/enter` methods.
- **Phase B**: `.v25_load_three(81, [LOI1_DV_MPS, 0, 0])`.
- **Phase C**: V37 ENTR 40 ENTR + PRO; expect V50 N99 flashing then armed.
- **Phase D** (burn loop): manual `state.csm_state = sv_pre_peri; state.time = Met(LOI1_TIG_MET_CS); hw.timers.set_time(...)`, then the burn loop from `phase_loi.rs:428-466` verbatim with `max_iters = 5_000`.

**Post-condition (phase boundary assertion, Phase E)**:
- `state.burn.burn_active == false`, `state.engine_thrusting == false`.
- `state.csm_state.frame == MoonInertial`.
- `apoapsis_altitude_moon(&elements) ∈ [265_000, 365_000]` m.
- `periapsis_altitude_moon(&elements) ∈ [91_000, 131_000]` m.
- `orbital_period(&elements, MU_MOON) ∈ [7_440, 8_040]` s.

### 3.4 Phase 4 — Lunar orbit (source: `agc-test/tests/phase_lunar_orbit.rs`)

**Reseed**: build `sv_initial` (equatorial 60 nm circular MCI at
`LOI2_END_MET_CS`) — inline the constants and `v_circ_at_alt(...)` helper
from `phase_lunar_orbit.rs:108-161`.

**Drive** — five sub-phases (Phases 1–5 in `phase_lunar_orbit.rs`):

- Init: `state.csm_state = sv_initial; state.time = Met(LOI2_END_MET_CS); state.refsmmat = IDENTITY_REFSMMAT; p22_init(&mut state);` (mirrors `phase_lunar_orbit.rs:182-185`).
- **Sub 1**: rev 1 baseline `.advance_coast(seconds(600)).expect_agc_matches_ground_truth(2_000, 2.0)`.
- **Sub 2**: rev 3 + P22 Mount Marilyn (index 5) — `.landmark_sighting(LandmarkTable::Moon, 5)`, capture `phase2_mark_count` / `phase2_reject_count`.
- **Sub 3**: rev 5 plain coast.
- **Sub 4**: rev 7 + P22 Boot Hill (index 6) — capture mark counts again.
- **Sub 5**: TEI epoch settle — propagate oracle to `TEI_MET_CS - 200s`, seed, `.advance_coast(seconds(200))`.

Use `advance_n_revs(...)` helper inlined from `phase_lunar_orbit.rs:119-123`.

**Post-condition (phase boundary assertion, after Sub 5)**:
- `state.csm_state.frame == MoonInertial`.
- altitude `∈ [100_000, 130_000]` m.
- speed `∈ [1_600, 1_670]` m/s.
- specific energy drift `< 0.5%` vs initial circular ε.
- `phase2_mark_count + phase4_mark_count >= 2`.

### 3.5 Phase 5 — TEI (source: `agc-test/tests/phase_tei.rs`)

**Reseed**: `sv_circ = pre_tei_sv()` — inline helper from
`phase_tei.rs:131-140`. Seed MET = `TEI_MET_CS - SETTLE_CS`.

**Drive** — five sub-phases (A, B, C, D, E from `phase_tei.rs`):

Identical structure to LOI but mirrored: V25 N81 = `[+TEI_DV_MPS, 0, 0]`,
P40 prograde burn. Phase D burn loop has `max_iters = 5_000` and includes
the **600 s post-burn coast** (`phase_tei.rs:494-505`) which confirms
hyperbolic departure.

**Post-condition (phase boundary assertion, after Phase E)**:
- `state.csm_state.frame == MoonInertial`.
- `‖v‖ ∈ [2_640, 2_740]` m/s.
- `ε > 0.5e6` J/kg.
- `elements.e > 1.0` (hyperbolic).
- `‖r_after_coast‖ > ‖r_cutoff‖` (receding).

### 3.6 Phase 6 — Trans-earth coast (source: `agc-test/tests/phase_transearth.rs`)

**Reseed strategy**: five sub-phases (Phases 1–5 in `phase_transearth.rs`).

**Drive** — for each sub-checkpoint:

- **Sub 1** (post-TEI MCI): `post_tei_sv_mci()` inlined from
  `phase_transearth.rs:155-167`. Coast 300 s, tol 2 km / 2 m/s.
- **Sub 2** (synthetic ECI at SOI exit): hardcoded SV
  `r=[1.5e8,0,0], v=[-1500,200,0]` at `SOI_EXIT_MET_CS`, frame `EarthInertial`. Coast 300 s.
- **Sub 3** (MCC-5 ECI): propagate oracle from Sub 2 seed to `MCC5_MET_CS`, apply 1.463 m/s `n_hat_perp_in_plane`, seed, coast 300 s.
- **Sub 4** (mid-coast ECI): propagate to `MID_COAST_MET_CS`, coast 300 s, tol 5 km / 5 m/s.
- **Sub 5** (synthetic EI seed): construct from `EI_ALT_M=121_920`, `EI_SPEED_MPS=11_040`, `EI_FPA_DEG=-6.48` (see `phase_transearth.rs:420-430`). Coast 10 s, tol 2 km / 2 m/s.

**Post-condition (phase boundary assertion, after Sub 5)**:
- `state.csm_state.frame == EarthInertial`.
- altitude `∈ [EI_ALT_M - 20km, EI_ALT_M + 20km]`.
- speed `∈ [EI_SPEED_MPS - 200, EI_SPEED_MPS + 200]`.
- `r_hat · v_hat < 0`.
- `|FPA - EI_FPA_DEG| < 5°`.

### 3.7 Phase 7 — Entry (source: `agc-test/tests/phase_entry.rs` + `agc-test/src/entry_scenario.rs`)

**Reseed**: see §4. The trans-earth Phase 6 end-state (Sub 5) is a
position on the +X axis at EI altitude with the documented velocity
decomposition. This state's frame is `EarthInertial`. The entry pipeline
needs additional `state.entry.target_lat_rad` / `target_lon_rad` /
`gha_epoch_rad` fields populated.

**Decision (open question 3 confirmed):** **Option (b)** — extract a
public helper `run_entry_phase(state: &mut AgcState, hw: &mut SimHardware, miss_km_tol: f64)` from `phase_entry.rs`. See §4 for the exact extraction.

**Drive**:
1. Populate entry-targeting fields in `state.entry` (target lat=0, lon=45°E
   per `setup_state_lunar_return` convention; gha_epoch=0).
2. Sync `state.csm_state.epoch = state.time`.
3. Call `agc_test::entry_scenario::run_entry_phase_scenario(&mut state, &mut hw, 3000.0)` (the new public function from §4).

**Post-condition (phase boundary assertion, and the run's final acceptance assertion §1.2)**:
- `state.entry.drogue_deployed == true`.
- `state.entry.phase == EntryPhase::Final`.
- `miss_km <= 3000.0`.
- `state.alarm.code == 0`.

---

## 4. Helpers and gaps

### 4.1 Extract `run_entry_phase_scenario` from `phase_entry.rs`

`phase_entry.rs::run_entry_phase` (line 46) currently lives in the test
module and takes `(name, seed: AgcState, miss_km_tol)`. It creates its own
`SimHardware`. To plug into MS-T7 we need:

```rust
// In agc-test/src/entry_scenario.rs, append:

use agc_sim::{run_scenario, ScenarioBuilder, SimHardware, scenario::SimDuration};
use agc_core::services::v_n::Key;
use agc_core::services::average_g::start_servicer;
use agc_core::programs::p61_p67::EntryPhase;

/// Drive the V37 P61 → P62 → P63 sequence and coast through entry on the
/// supplied `state` + `hw`. Asserts drogue deploys within `miss_km_tol` km.
///
/// This is the body of `phase_entry.rs::run_entry_phase` with the
/// `AgcState`/`SimHardware` ownership inverted: the caller owns them so
/// the entry phase can be chained after another mission phase.
pub fn run_entry_phase_scenario(
    state: &mut AgcState,
    hw: &mut SimHardware,
    miss_km_tol: f64,
) {
    // ... copy body of phase_entry.rs:46-132 verbatim, replacing
    // `let mut state = seed;` and `let mut hw = SimHardware::new();`
    // with the borrowed parameters.
}
```

Two cosmetic changes:
1. The `name` argument drops to a single static string `"phase_entry"`
   (scenario names are only used for log output).
2. `phase_entry.rs::run_entry_phase` becomes a thin wrapper that owns its
   own `state`/`hw` and calls `run_entry_phase_scenario`. The two existing
   test functions (`tc_phase_entry_direct_leo` and `tc_phase_entry_lunar_return`) keep working unchanged.

### 4.2 No other extractions

Everything else stays as inline code in `full_mission.rs`. In particular:

- The six trans-lunar sub-scenarios are inlined as-is from
  `phase_translunar.rs`.
- The five LOI sub-phases are inlined as-is from `phase_loi.rs`, including
  the manual burn loop in Phase D.
- The five TEI sub-phases are inlined as-is from `phase_tei.rs`, including
  the manual burn loop and post-burn coast in Phase D/E.
- The five trans-earth sub-checkpoints are inlined as-is from
  `phase_transearth.rs`.
- `parking_orbit_sv`, `derive_post_tli_sv`, `n_hat_perp_in_plane`,
  `pre_pericynthion_sv`, `v_circ_at_alt`, `advance_n_revs`, `pre_tei_sv`,
  `post_tei_sv_mci`: all inlined as private functions in `full_mission.rs`
  (with a `// from phase_xxx.rs:NN` comment on each).

If any of these helpers later need to be DRY-ed up, the refactor will
happen in a follow-up issue *after* MS-T7 has been green for a release.

### 4.3 Trans-earth → entry hand-over wiring

After trans-earth Sub 5 completes, `state.csm_state` carries
`(r=[r_ei,0,0], v=[v_radial, v_tangential, 0])` in `EarthInertial` frame
at MET `EI_MET_CS`. The entry pipeline expects identical conventions:
`make_initial_state` in `entry_scenario.rs:165-178` builds the same shape.
However, the entry phase additionally needs:

- `state.entry.target_lat_rad = 0.0;`
- `state.entry.target_lon_rad = 45.0_f64.to_radians();` (matches
  `setup_state_lunar_return`, 5004 km downrange Pacific splashdown)
- `state.gha_epoch_rad = 0.0;` (GHA = 0 at MET = 0 → ECI ≡ ECEF, same
  simplification the entry tests use)
- `state.csm_state.epoch = state.time;` (mirrors
  `phase_entry.rs:107`).

These four assignments happen in the test body *between* trans-earth Sub 5
and the `run_entry_phase_scenario` call, **not** inside the new helper.
Keeping them visible at the call site documents how the trans-earth and
entry phase boundaries glue together.

### 4.4 No-op for issue #51

The full-mission test does **not** test the open SOI auto-handover issue
(`average_g_step` does not call `soi_check`, GH #51). The reseed pattern
makes that defect invisible to MS-T7. This is intentional and consistent
with all six per-phase tests. When #51 is fixed, MS-T7 may simplify, but
that is out of scope here.

---

## 5. README addition

Add the following paragraph to `/Users/Juergen.Schiewe/dev/AGC-in-Rust/README.md`.
Insert it as a new top-level section between "Directory Structure" and
"References" (so around line 25):

```markdown
## Testing

The integration test suite lives in `agc-test/tests/`. The functional-
completeness gate is **`cargo test --test full_mission`**: a single
end-to-end Apollo 8 walkthrough that chains all seven mission phases —
TLI, trans-lunar coast (with MCC-2 and MCC-4), LOI, lunar orbit (eight
revolutions with P22 landmark marks), TEI, trans-earth coast (with MCC-5),
and entry — on the same `AgcState` and `SimHardware` instance. The test
verifies AGC onboard state against the ground-truth oracle at every phase
boundary and asserts the spacecraft reaches drogue deploy within 3000 km
of the configured splashdown target. Runtime budget: under 30 s.

Per-phase tests (`phase_tli.rs`, `phase_translunar.rs`, `phase_loi.rs`,
`phase_lunar_orbit.rs`, `phase_tei.rs`, `phase_transearth.rs`,
`phase_entry.rs`) exercise individual mission segments in isolation and
should be the first place to look when a specific phase breaks.
```

No other documentation changes are required for MS-T7.

---

## 6. Test file skeleton

```rust
//! MS-T7 — Apollo 8 full-mission walkthrough capstone.
//!
//! Implements GitHub issue #30. Spec: `specs/ms-t7-full-mission-spec.md`.
//!
//! Chains all seven Apollo 8 mission phases — TLI, trans-lunar, LOI,
//! lunar orbit, TEI, trans-earth, entry — on the same `AgcState` and
//! `SimHardware`. The test is the project's functional-completeness gate.
//!
//! ## Why phase boundaries reseed `state.csm_state`
//!
//! `moon_position` is anchored to the Apollo 11 launch epoch (1969-07-16);
//! Apollo 8 launched 1968-12-21. Continuous propagation across the
//! 146-hour mission would diverge from the historical trajectory by
//! hundreds of thousands of kilometres. All seven per-phase tests handle
//! this by reseeding `state.csm_state` at each historical MET checkpoint;
//! MS-T7 honours the same contract. See `phase_translunar.rs` module doc.

use agc_core::AgcState;
use agc_sim::SimHardware;

// Constants and helpers inlined from phase_*.rs (cite source on each).

#[test]
fn tc_full_mission_apollo_8_end_to_end() {
    let mut state = AgcState::new();
    let mut hw   = SimHardware::new();

    // ── Phase 1: TLI ─────────────────────────────────────────────────
    // ... (see §3.1)

    // ── Phase 2: trans-lunar coast ───────────────────────────────────
    // ... (see §3.2)

    // ── Phase 3: LOI ─────────────────────────────────────────────────
    // ... (see §3.3)

    // ── Phase 4: lunar orbit ─────────────────────────────────────────
    // ... (see §3.4)

    // ── Phase 5: TEI ─────────────────────────────────────────────────
    // ... (see §3.5)

    // ── Phase 6: trans-earth coast ───────────────────────────────────
    // ... (see §3.6)

    // ── Phase 7: entry ───────────────────────────────────────────────
    // ... (see §3.7)

    // ── Final acceptance assertion (§1.2) ────────────────────────────
    let (lat, lon) = agc_test::entry_scenario::sub_satellite_lat_lon(&state);
    let miss_km = agc_test::entry_sim::haversine_km(
        lat, lon,
        state.entry.target_lat_rad,
        state.entry.target_lon_rad,
    );
    assert!(state.entry.drogue_deployed, "drogue must deploy by end of run");
    assert_eq!(state.entry.phase, agc_core::programs::p61_p67::EntryPhase::Final);
    assert!(miss_km <= 3000.0, "miss = {miss_km:.0} km exceeds 3000 km gate");
    assert_eq!(state.alarm.code, 0, "no AGC alarms over the full mission");
}
```

LOC estimate: ~900 lines including comments and the inline helpers
(closer to 1000 once the burn-loop bodies for LOI and TEI are duplicated
verbatim from their phase tests).

---

## 7. Open questions / consultation requests for the user

None blocking. The four open questions in issue #30's brief are resolved
in this spec:

1. **Code reuse strategy**: inline. See §2.4.
2. **MET timeline**: discrete reseed per phase. See §2.2.
3. **`phase_entry.rs` reuse**: extract `run_entry_phase_scenario`. See §4.1.
4. **Success criterion**: drogue deploy within 3000 km + no alarms. See §1.2.
5. **Runtime budget**: < 30 s. See §2.5.

Two notes for the developer:

- If `run_entry_phase_scenario` extraction reveals that
  `phase_entry.rs::run_entry_phase` does anything `state`-mutating before
  the first `start_servicer` that depends on a *fresh* `AgcState`, flag it
  back in the PR. A scan of the current code shows only V37 keystrokes
  and `start_servicer`, both of which are order-tolerant — but the test
  is the proof.
- The `phase_translunar.rs` Phase 4b synthetic MCI seed (50 000 km from
  Moon, MCI frame) is included in MS-T7 verbatim. It briefly leaves the
  spacecraft in `MoonInertial` between trans-lunar Sub 4a (ECI) and
  Sub 5 (ECI again). MS-T7 ends Phase 2 in ECI, just like the per-phase
  test. Auditors who read top-to-bottom will see one frame flip-flop
  inside Phase 2; the existing module doc in `phase_translunar.rs`
  explains why.

---

## 8. Out of scope (explicit non-goals)

- No new Rust modules outside `full_mission.rs` and the
  `run_entry_phase_scenario` extraction.
- No relaxation of any per-phase tolerance.
- No fix for the Moon-ephemeris-epoch limitation (open issue, not blocking).
- No fix for GH #51 (SERVICER `soi_check` wiring).
- No P23 cislunar marks (gated on GH #57).
- No PTC (passive thermal control) rotation.
- No CM/SM separation modelling.

---

## 9. Implementation checklist for the developer

In order:

1. Add `run_entry_phase_scenario(&mut state, &mut hw, miss_km_tol)` to
   `agc-test/src/entry_scenario.rs` (§4.1). Verify
   `cargo test -p agc-test --test phase_entry` still passes after the
   refactor.
2. Create `agc-test/tests/full_mission.rs` with the skeleton from §6.
3. Fill in Phase 1 (TLI) by inlining from `phase_tli.rs`. Run
   `cargo test --test full_mission` — assert it reaches the end of
   Phase 1 with the documented post-conditions.
4. Repeat for Phases 2–7, one at a time. After each phase, the test
   should reach the next phase boundary with all assertions green.
5. Add the README paragraph (§5).
6. Run `cargo clippy --all-targets -- -D warnings` and
   `cargo test --all-targets` to confirm CI cleanliness.
7. Push branch `feature/30-full-mission`. Link to issue #30. Acceptance
   criterion: `cargo test --test full_mission` passes in CI under 30 s
   wall time.
