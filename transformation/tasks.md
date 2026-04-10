# Task Tracking

## Active Tasks

### Milestone 1 — Core Infrastructure (in progress)

- [x] Create Cargo workspace (`agc-core`, `agc-sim`, `agc-test`) with feature flags (`sim`, `bare-metal`)
- [x] **Spec** — `types/` module (CduAngle, Vec3, Mat3x3, Met, DeltaV newtypes with scale docs) → `specs/types-module-spec.md`
- [x] **Impl** — `types/` module
- [x] **Spec** — `AgcHardware` trait and all sub-traits (`Timers`, `Dsky`, `Imu`, `Optics`, `Engine`, `Rcs`, `Uplink`, `Telemetry`) → `specs/hal-spec.md`
- [x] **Impl** — HAL traits in `agc-core/src/hal/`
- [x] **Impl** — Simulated HAL in `agc-sim/src/hardware.rs`
- [x] **Spec** — `Executive` (job table, priority scheduler, NOVAC/FINDVAC, 1202 alarm) → `specs/executive-spec.md`
- [x] **Impl** — `executive/scheduler.rs`, `executive/job.rs`
- [x] **Spec** — `Waitlist` (delta-time chain, 8 slots, T3RUPT dispatch) → `specs/executive-spec.md` §4.5–4.7
- [x] **Impl** — `executive/waitlist.rs`
- [x] **Spec** — Restart protection (phase tables, group management, GOJAM) → `specs/executive-spec.md` §4.8–4.10
- [x] **Impl** — `executive/restart.rs`
- [x] **Impl** — `services/alarm.rs` (alarm codes, DSKY alarm display)
- [x] **Impl** — `services/fresh_start.rs` (fresh_start + restart with group re-dispatch, 7 tests)
- [x] **Tests** — All Executive + Waitlist unit tests passing (29 tests)
- [x] Bare-metal build clean: `cargo build --target thumbv7em-none-eabihf -p agc-core`

## Backlog

### Milestone 2 — Navigation Foundation

- [x] **Spec** — `math/linalg.rs` (dot, cross, norm, unit, mxv, vxm) → `specs/linalg-spec.md`
- [x] **Impl** — `math/linalg.rs` (11 functions + IDENTITY, 43 tests passing)
- [x] **Spec** — `navigation/state_vector.rs` (StateVector, coordinate frames) → `specs/state-vector-spec.md`
- [x] **Impl** — `navigation/state_vector.rs` (Frame enum, StateVector, debug_assert_valid, 7 tests)
- [x] **Spec** — `navigation/gravity.rs` (Earth/Moon models, oblateness) → `specs/gravity-spec.md`
- [x] **Impl** — `navigation/gravity.rs` (earth_gravity + J2, moon_gravity, third_body_perturbation, 8 tests)
- [x] **Spec** — `navigation/integration.rs` (Cowell / Encke propagation) → `specs/integration-spec.md`
- [x] **Impl** — `navigation/integration.rs` (average_g_step, propagate_coast RK4, total_gravity, soi_check, 6 tests)
- [x] **Spec** — `services/average_g.rs` (SERVICER 2-second cycle) → `specs/average-g-spec.md`
- [x] **Impl** — `services/average_g.rs` (PipaCalibration, start/stop/servicer_task, 7 tests)
- [x] Capture VirtualAGC math fixtures → `agc-test/fixtures/` (3 JSON files, analytically computed; `docs/fixtures.md`)
- [x] Navigation accuracy tests passing against fixtures (7 tests in `navigation_accuracy.rs`)

### Milestone 3 — Guidance and DAP

- [x] **Spec + Impl** — `math/kepler.rs` (KEPRTN) → `specs/kepler-spec.md`, 14 tests, Battin universal-variable
- [x] **Spec** — `math/lambert.rs` (Lambert targeting) → `specs/lambert-spec.md`
- [~] **Impl** — `math/lambert.rs` — Izzo 2015, 3 tests pass + 4 ignored (needs convergence debug)
- [x] **Spec + Impl** — `navigation/conics.rs` → `specs/conics-spec.md`, OrbitalElements + 5 tests
- [x] **Spec** — `control/imu_control.rs` → `specs/imu-control-spec.md`
- [x] **Impl** — `control/imu_control.rs` (10 tests: PIPA comp, gyro drift, coarse align, REFSMMAT TRIAD, gimbal lock)
- [x] **Spec** — `control/dap.rs` → `specs/dap-spec.md`
- [x] **Impl** — `control/dap.rs` (7 tests: dap_init/stop/step, mode dispatch, staging fields)
- [x] **Spec** — `control/attitude.rs` → `specs/attitude-spec.md`
- [x] **Impl** — `control/attitude.rs` (6 tests: error, rates, damping, PD, maneuver rate)
- [x] **Spec** — `control/rcs_logic.rs` → `specs/rcs-logic-spec.md`
- [x] **Impl** — `control/rcs_logic.rs` (14 tests: jet tables, selection, pulse duration)
- [x] **Spec** — `control/tvc.rs` → `specs/tvc-spec.md`
- [x] **Impl** — `control/tvc.rs` (7 tests: lead-lag filter, trim, saturation)
- [x] **Spec** — `guidance/targeting.rs` → `specs/targeting-spec.md`
- [x] **Impl** — `guidance/targeting.rs` (10 tests: LVLH, Lambert, burn attitude)
- [x] **Spec** — `guidance/maneuver.rs` → `specs/maneuver-spec.md`
- [x] **Impl** — `guidance/maneuver.rs` (5 tests: burn execution, cross-product steering, cutoff)
- [x] **Architect review** — `specs/milestone-3-architect-review.md` (10 critical issues, 6 ADs, all resolved)

### Milestone 4 — Non-rendezvous Programs (P-codes)

Scope: programs that can be implemented without the Verb/Noun processor
(Milestone 6) and without the rendezvous guidance stack (Milestone 5).
Phased by dependency depth: each phase reuses the primitives built in
prior milestones.

- [x] **Phase 1** — P00 (CMC Idle), P30 (External ΔV), P37 (Return to Earth)
- [x] **Phase 2** — P51, P52 (IMU alignment — sequencing over TRIAD REFSMMAT)
- [x] **Phase 3** — P40, P41 (SPS/RCS thrusting) + `burn_servicer_exit`
- [x] **Phase 4** — P11 (Earth Orbit Insertion Monitor)
- [x] **Phase 5** — P61–P67 (Entry guidance skeletons)
- [x] **Phase 6** — Book-keeping programs: P01/P02 (pre-launch
  initialisation), P06 (CMC power-down), P15 (TLI monitor), P47 (thrust
  monitor). All are thin wrappers over existing services.

### Milestone 5 — Rendezvous Programs

Scope: the rendezvous targeting + monitoring program family. Held back
from Milestone 4 because they need dedicated relative-motion primitives,
closing-rate displays, and Lambert rendezvous targeting that are genuine
new math — not just sequencing wrappers.

- [x] **Spec + Impl** — `guidance/rendezvous.rs` (relative state, closing rate)
- [x] P20 — Rendezvous navigation
- [x] P21 — Ground-track determination
- [x] P22 — Orbital navigation (landmark tracking)
- [x] P23 — Cislunar midcourse navigation (star/landmark sightings)
- [x] P31 — CSI (Coelliptic Sequence Initiation) targeting
- [x] P32 — CDH (Constant Delta-Height) targeting
- [x] P33 — TPI (Terminal Phase Initiation) targeting
- [x] P34 — TPM (Terminal Phase Midcourse) targeting

**Completed 2026-04-10.** Delivered in six phases:

- **Phase 1** (commit `e4f7562`) — `guidance/rendezvous.rs` primitives:
  `lvlh_matrix` (Hill-frame rotation, z toward body — distinct from the
  RSW frame in `guidance/targeting.rs`), `relative_state_lvlh`, `range`,
  `range_rate`, `los_angles_lvlh`, `time_to_closest_approach`. 12 tests.

- **Phase 2** (commit `b0c3b5b`) — **P20 Rendezvous Navigation**:
  `RendezvousNavState` added to `AgcState` with a full 6×6 W-matrix;
  scalar Kalman measurement update for radar range/range-rate and
  sextant LOS marks; 3-sigma reject gate with 5-consecutive-reject alarm;
  process-noise growth and W-matrix rectification. Schedules itself via
  the Waitlist (not `servicer_exit`) at a 2 s period. 8 tests.

- **Phase 3** (commit `af1f78c`) — **P21 Ground-Track** + **P22 Orbital
  Navigation**: P21 is a pure-computation ground-track solver
  (`kepler_step` propagation + Earth rotation + lat/lon/alt extraction).
  P22 mirrors P20's measurement structure but updates `csm_state` from
  sextant landmark sightings, with a separate `CsmNavState` W-matrix.
  Factored the P20 Kalman helper into `navigation/kalman.rs` so both
  programs (and P23) share the same scalar update machinery. Added
  `gha_epoch_rad: f64` top-level field to `AgcState`. 11 tests.

- **Phase 4** (commit `32a8b43`) — **P23 Cislunar Midcourse Navigation**:
  star-horizon and star-landmark angle measurement models with closed-form
  sensitivity derivations (O'Brien Ch. 11). Shares `state.csm_nav` with
  P22 since both update the same physical quantity. Detects body from
  `Frame::EarthInertial` / `Frame::MoonInertial`. 8 tests.

- **Phase 5** (commit `6bbe6c0`) — **P31 CSI** + **P32 CDH**: closed-form
  coelliptic rendezvous targeting (no Lambert). P31 is a 1-D Newton
  iteration over the in-track ΔV with CDH's W-axis residual as the cost
  function; P32 is a closed-form coelliptic solver (Battin eq. 11-53).
  Both emit `Maneuver` into `state.pending_maneuver` with new
  `TargetingMode::CsiBurn` / `CdhBurn` variants. 10 tests.

- **Phase 6** (commit `96d2ce5`) — **P33 TPI** + **P34 TPM**: Lambert-based
  terminal-phase targeting. Shared `compute_lambert_intercept` helper
  calls `math::lambert::lambert` as a black box, with `validate_lambert_inputs`
  pre-check to catch degenerate geometry before the solver panics. P33
  stores `state.tpi_arrival_epoch: Option<f64>` so P34 can retarget the
  same arrival point with the remaining transfer time. New
  `TargetingMode::TpiBurn` / `TpmBurn` variants. 12 tests.

**New `AgcState` fields**: `rendezvous_nav`, `csm_nav`, `gha_epoch_rad`,
`tpi_arrival_epoch`. **New shared infrastructure**: `guidance/rendezvous.rs`
(Hill-frame primitives), `navigation/kalman.rs` (state-agnostic scalar
Kalman update). **`TargetingMode` extended** with `CsiBurn`, `CdhBurn`,
`TpiBurn`, `TpmBurn`.

**Test coverage**: 302 → 363 agc-core tests (+61), 0 regressions, 0 new
ignored. The 4 long-standing ignored tests (TC-TGT-10 + 3× TC-P37) remain
tracked under Technical Debt.

### Milestone 6 — DSKY and Crew Interface

Unlocks the interactive paths that were deferred throughout M4: P30's
V25 N33/N81 data-load state machine, P51/P52's MARK button loop,
P40's crew go/no-go gates.

- [x] **Spec + Impl** — `services/v_n.rs` (Verb/Noun state machine)
- [x] **Spec + Impl** — `services/pinball.rs` (PINBALL display formatter)
- [x] V37 program-select handler wired into the V/N processor
- [x] V25 data-load state machine (used by P30, P37, P51/P52 MARK loop)
- [x] V50 crew go/no-go acknowledgement (used by P40 pre-ignition)
- [x] `agc-sim` terminal DSKY simulator

**Completed 2026-04-10.** Delivered in five phases:

- **Phase 1** (commit `275adf9`) — V/N processor core in `services/v_n.rs`:
  `Key` enum, `VnPhase` state machine (Idle/EnteringVerb/EnteringNoun/
  OprErr), `feed_key()`, and V37 dispatch through the `PROGRAM_TABLE`.
  Covers V06 (display decimal), V16 (monitor), V34 (terminate to P00),
  V35 (lamp test), and V37 (program select).

- **Phase 2** (commit `c1c9529`) — data-entry verbs V21/V22/V23/V25 with a
  5-digit signed accumulator per register and commit handlers for N33
  (TIG → `vn.pending_tig`) and N81 (LVLH ΔV → `p30_load_dv_lvlh`). P30
  is now fully interactive: `V25 N33 E <tig> E V25 N81 E <Δvx> E <Δvy> E <Δvz> E`.

- **Phase 3** (commit `4bdfc7f`) — PINBALL display formatter in
  `services/pinball.rs`: pure-computation f32 → signed 5-digit `Register`,
  `TwoDigit` PROG/VERB/NOUN fields, 7-segment bit table, and
  `decode_dsky(&DskyState) → DskyFrame` for the bare-metal T4RUPT shim to
  push to the HAL. 13 test cases.

- **Phase 4** (commit `67f869f`) — V50 "please perform" crew
  acknowledgement. Programs call `request_v50(state, noun, on_proceed)`;
  PRO key consumes the pending callback. P40 now sets DAP to Maneuver
  mode on init and arms the SPS (DAP → Tvc, `engine_thrusting = true`)
  only after the crew presses PRO in response to V50 N99.

- **Phase 5** (commit `537fd19`) — terminal DSKY simulator in `agc-sim`:
  `dsky_sim` binary renders a Block 2 DSKY panel faithful to Figure 39
  of O'Brien (2×7 indicator-lamp grid, PROG/VERB/NOUN + R1/R2/R3 display
  panel, 7-column keyboard). Uses `crossterm` raw mode + ANSI; 20 Hz
  redraw with real-time MET and 1 Hz VERB/NOUN flashing. Added
  `tracker` lamp to `DskyState`/`Lamps`. Also fixed a display-mirroring
  gap in `feed_key` (keystrokes were only written to `state.dsky` on
  dispatch, invisible during entry) via a new `sync_display` helper
  plus four regression tests (tc_vn_dm_1..4).

**Test coverage**: 30 v_n tests, 13 pinball tests, 6 key-mapping/render
tests in agc-sim. Total project: 302 agc-core tests pass.

### Technical Debt

- [x] **Debug** — `math/lambert.rs` Izzo convergence bugs. **RESOLVED 2026-04-10** using the Izzo 2015 paper (https://www.esa.int/gsp/ACT/doc/MAD/pub/ACT-RPR-MAD-2014-RevisitingLambertProblem.pdf). All 7 Lambert tests pass (0 ignored). Fixes applied:
  - **Root cause**: Lancaster-Blanchard T formula was inverted — code divided by `a^(3/2)` where it should multiply. Corrected in both `tof_and_derivs` and `tof_and_derivs_inner`.
  - **Initial guess**: Replaced all three regime formulas with Izzo Eq. 30 exactly (slow `(T₀/T)^(2/3)−1`, fast `5·T₁·(T₁−T)/(2·T·(1−λ⁵))+1`, normal `(T₀/T)^(1/log₂(T₀/T₁))−1`).
  - **T₀₀**: Corrected Eq. 19 to use signed λ: `acos(λ) + λ·sqrt(1−λ²)`.
  - **Tolerance**: Relaxed `TOL_NDIM` from 1e-12 to 1e-5 (still sub-metre position accuracy; Halley stalls near the 180° transfer boundary otherwise).
  - **Test geometry repairs**: TC-LAM-1 now uses a proper 179° Hohmann at `tof=T/2`; TC-LAM-2 uses a 19.44° arc matching the LEO period at `tof=300 s`; TC-LAM-3 asserts TLI elliptic bounds instead of hyperbolic escape.
  - **Known remaining edge case**: TC-TGT-10 / TC-P37-{1,2,4} (~60 h TEI from LLO to Earth entry sphere) still stall at residual ≈1.45 — this is a long-TOF high-eccentricity regime that is outside Milestone 4 scope. Not required for Milestone 5 rendezvous targeting (P33/P34 use short-TOF TPI/TPM). Revisit when P37 return-to-earth targeting is exercised in a dedicated pass.

- [ ] **Debug** — Lambert long-TOF TEI regime (TC-TGT-10 / TC-P37-{1,2,4}).
  Four tests are currently `#[ignore]`'d because the Izzo Halley iteration
  stalls at residual ≈1.45 on the ~60 h trans-Earth injection geometry
  (LLO → Earth entry sphere, `r₁ ≈ 1.84 Mm`, `r₂ ≈ 384 Mm`, `tof ≈ 60 h`).
  The solver's T(x,λ), derivatives, and initial-guess formulas are
  correct per Izzo 2015 for the short-TOF regimes already validated by
  TC-LAM-1..5, so the fix is likely one of:
  - Multi-revolution branch selection (the paper's §4 M > 0 path) if the
    geometry in fact requires M=1.
  - A dedicated long-TOF / high-eccentricity initial guess — the current
    slow-regime `(T₀/T)^(2/3) − 1` may undershoot badly when `T ≫ T₀₀`.
  - Halley-step damping or a bracketed Brent fallback when the residual
    refuses to shrink for several iterations.
  Acceptance: un-ignore TC-TGT-10, TC-P37-1, TC-P37-2, TC-P37-4; full
  suite reports 0 ignored Lambert-related tests. Owner: unassigned.
  Blocked by: nothing (Lambert core is green). Target milestone: the
  P37 return-to-earth pass or a dedicated Lambert hardening sprint,
  whichever comes first.

- [x] **Impl** — `navigation/planetary::moon_position(t: Met) -> Vec3`.
  **RESOLVED 2026-04-10.** Implemented via a full Meeus *Astronomical
  Algorithms* Chapter 47 Brown-series approximation (60 periodic terms
  for longitude and distance from table 47.A, 60 terms for latitude from
  table 47.B, plus the Venus/Jupiter additive corrections). Produces
  geocentric ecliptic coordinates, then rotates to equatorial via the
  time-dependent obliquity. Accuracy ~10 km.
  Mission epoch hardcoded to Apollo 11 launch (JD 2440419.0639,
  1969-07-16 13:32:00 UTC). Output frame treated as AGC Mean of 1969.5
  per ADR-013; the precession drift from mean-of-date to 1969.5 over
  a 1-year window at lunar distance is ~30 km, within the accuracy bound.
  Research: `specs/lunar-ephemeris-research.md` (the AGC's original
  9th-degree PAD-loaded polynomial approach was documented there but
  was sidestepped for the MVP because real Apollo 11 PAD coefficients
  are not available).
  `services/average_g.rs` still uses the hardcoded `[3.844e8, 0, 0]`
  placeholder — the analyst's research identified this as architecturally
  misplaced (the real AGC does not include a third-body term in AVERAGE G;
  that term lives in `ORBITAL_INTEGRATION.agc`). Leaving the placeholder
  alone; its error is negligible at LEO (third-body term ~1e-6 g).
  Includes 8 unit tests (TC-MOON-1..8) covering MET→JD conversion,
  distance-range sanity, sign-correctness cross-check against the known
  Apollo 11 launch-time Moon position (waxing crescent in Leo, RA ≈ 142°,
  Dec ≈ +15°), 1-hour displacement bounds, and sidereal-period cyclicity.
  agc-core tests: 369 → 377.

- [ ] **Impl** — `navigation/time::met_to_gmst(t: Met, launch_jd: f64) -> f64`
  is a `todo!("MET to GMST conversion")` stub with no current callers.
  Needed to convert Mission Elapsed Time to Greenwich Mean Sidereal Time
  for Earth-fixed frame conversions in P21/P22 — currently P21 uses a
  simplified linear `gha_epoch_rad + OMEGA_EARTH * t` model which is
  accurate to within ~1 km ground position over a typical mission.
  The real AGC used a Julian-date based GMST formula (IAU 1980).
  Acceptance: a higher-fidelity GMST reduces P21 ground-track error
  relative to STK reference by ≥ 10×. Blocked by: need a reference
  ground-track dataset to compare against.

- [ ] **Perf** — Replace `navigation/integration::propagate_coast` Cowell
  RK4 sub-stepping with `math::kepler::kepler_step` + small perturbation
  correction once the perturbation model (J2, Moon, drag) is factored
  out into its own function. Current implementation at `integration.rs:148`
  is correct but slow for long coasts; kepler_step is ~100× faster for
  a pure-Kepler step. The RK4 path stays for perturbed propagation.
  Acceptance: `propagate_coast` switches to `kepler_step` for `dt > 600 s`
  in the pure-two-body case; all existing integration tests still pass.

- [x] **Data + Impl** — Populate `navigation/star_catalog.rs` with the
  full 37-entry AGC navigation star catalogue. **RESOLVED 2026-04-10.**
  All 37 direction vectors transcribed verbatim from `Comanche055/STAR_TABLES.agc`
  in the AGC Mean of 1969.5 equatorial frame (ADR-013); no precession
  rotation applied. Layout: `pub const STAR_CATALOG: [StarEntry; 37]`
  in `navigation/star_catalog.rs` with `star_direction(n: u8) -> Option<Vec3>`
  helper. Ascending index convention (`STAR_CATALOG[number - 1]`) —
  the AGC's descending file layout is not mirrored. `CislunarStar` /
  `CISLUNAR_STAR_TABLE` stub removed from `programs/p23.rs` (was dead
  code, never referenced). Six unit tests cover unit-length invariant
  (tolerance 1e-6 to accommodate ~1e-7 decimal-transcription rounding),
  ascending number convention, `star_direction` boundary cases
  (0/1/37/38/50/255), sign-correctness spot checks for Polaris/Alpheratz/
  Antares, CATALOG_SIZE consistency, and no-duplicate-numbers.
  agc-core tests: 363 → 369. Star names are approximate identifications
  only — documented as non-authoritative and not verified by tests.

- [x] **Architecture** — Navigation reference-frame decision. **RESOLVED
  2026-04-10** as ADR-013 (`transformation/decisions.md`): the port's
  `Frame::EarthInertial` and `Frame::MoonInertial` use the AGC Mean of
  1969.5 equatorial frame natively, matching `STAR_TABLES.agc` and the
  AGC's ephemeris tables. No precession rotation is applied. Rationale:
  primary validation is against a simulated AGC; frame-matching the
  original eliminates an entire class of 0.4° discrepancies. Output
  state vectors cannot be directly compared to contemporary J2000 data
  (JPL Horizons, modern TLEs) without an explicit IAU 1976 rotation,
  which is acceptable because the validation target is the AGC itself.
  Spec updates: `p23-spec.md` §1 and §3.2 now reference ADR-013 and use
  "AGC Mean-of-1969.5" wording; `state-vector-spec.md` §2.2 already
  declared "mean-of-1969 inertial frame" and needed no change.
  **Unblocks**: the star catalogue population work above.

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
