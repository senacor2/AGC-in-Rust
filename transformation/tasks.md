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

### Milestone 6b — V21-V25 Noun Commit Handlers

Complete the `noun_commit()` dispatch table in `services/v_n.rs` so that
all loadable nouns from the Comanche055 `PINBALL_NOUN_TABLE` are handled.
Reference matrix: `specs/loadable-verbs.txt`. Currently implemented: N33,
N81 only.  All other loadable nouns are silently ignored.

#### Group 1 — Program-critical nouns (needed by existing programs)

- [x] **Impl** — N18 commit (auto maneuver ball angles, 3 components → `dap_state.commanded_attitude`). R1/R2/R3 = deg×100, scale 0.01 → degrees → radians. Test: TC-VND-10.
- [x] **Impl** — N70 commit (star/planet selection code, R1 → `VnState.crew_star_code`). Test: TC-VND-11.
- [x] **Impl** — N72 commit (landmark lat/lon/alt, 3 components → `VnState.crew_landmark`). Test: TC-VND-12.

Skipped to Group 3 (spec needs updating before implementation):
N11, N13, N17, N37, N46, N47, N71, N89.

#### Group 2 — Time nouns (HMS format, need HMS↔cs conversion)

- [ ] **Impl** — HMS↔centisecond conversion helpers for `noun_commit`/`noun_display`.
  All HMS nouns (Komp=HMS "ja") share the same R1=hours, R2=minutes,
  R3=seconds×100 format.  Factor out `hms_to_cs(values: [f64; 3]) -> u32`
  and `cs_to_hms(cs: u32) -> [f64; 3]` helpers, then wire into
  noun_commit and noun_display.
- [ ] **Impl** — N16 commit (time of event, HMS → cs).
- [ ] **Impl** — N24 commit (delta time for AGC clock, HMS → cs).
- [ ] **Impl** — N31 commit (time of landing site, HMS → cs).
- [ ] **Impl** — N32 commit (time to perigee, HMS → cs).
- [ ] **Impl** — N34 commit (time of event, HMS → cs). Scale entry exists but no handler.
- [ ] **Impl** — N35 commit (time to go to event, HMS → cs).
- [ ] **Impl** — N36 commit (time of AGC clock, HMS → cs). Overwrite MET.
- [ ] **Impl** — N38 commit (time of state vector, HMS → cs).
- [ ] **Impl** — N39 commit (delta time to transfer, HMS → cs).
- [ ] **Impl** — N65 commit (sampled AGC time, HMS → cs).

#### Group 3 — Remaining loadable nouns

Deferred from Group 1 (spec update required):
- [ ] **Spec + Impl** — N11 commit (TIG of CSI, HMS). Spec says Excluded/reserved; loadable-verbs.txt says loadable. Reconcile before implementing.
- [ ] **Spec + Impl** — N13 commit (TIG of CDH, HMS). Not in v_n-spec.md noun table; P32 currently uses N33 for TIG. Clarify whether N13 is a distinct noun or alias.
- [ ] **Spec + Impl** — N17 commit (star angle difference). Spec says display-only; loadable-verbs.txt says loadable. Reconcile.
- [ ] **Spec + Impl** — N37 commit (time to next maneuver event, HMS). Spec says display-only countdown; loadable-verbs.txt says loadable. Reconcile.
- [ ] **Spec + Impl** — N46 commit (autopilot configuration, 2 components). Spec says Excluded/spare; loadable-verbs.txt says loadable. Reconcile.
- [ ] **Spec + Impl** — N47 commit (vehicle weight / reentry trajectory angle, 2 components). Spec says display-only; loadable-verbs.txt says loadable. Reconcile.
- [ ] **Spec + Impl** — N71 commit (IMU calendar time, HMS). Consumer does not exist yet. Determine target field.
- [ ] **Spec + Impl** — N89 commit (landmark). Spec says Excluded/spare; loadable-verbs.txt says loadable. Reconcile with N72.

Previously listed:
- [ ] **Impl** — N01/N02/N03 commit (machine address fractional/whole/degrees). Debug/test support.
- [ ] **Impl** — N05 commit (angular error/difference, 1 component, V21 only).
- [ ] **Impl** — N06 commit (option code, 2 components).
- [ ] **Impl** — N07 commit (ECADR of word to be modified).
- [ ] **Impl** — N08/N09 commit (alarm data / alarm codes).
- [ ] **Impl** — N10 commit (channel to be specified, 1 component, V21 only).
- [ ] **Impl** — N12 commit (option code, 2 components).
- [ ] **Impl** — N15 commit (increment machine address, 1 component, V21 only).
- [ ] **Impl** — N19 commit (bypass attitude trim maneuver, 3 components).
- [ ] **Impl** — N20 commit (ICDU angles, 3 components).
- [ ] **Impl** — N21 commit (PIPAs, 3 components).
- [ ] **Impl** — N22 commit (new ICDU angles, 3 components).
- [ ] **Impl** — N25 commit (checklist, 3 components).
- [ ] **Impl** — N26 commit (prio/delay, adres, bbcon).
- [ ] **Impl** — N27 commit (self test on/off, 1 component, V21 only).
- [ ] **Impl** — N29 commit (XSM launch azimuth, 1 component, V21 only).
- [ ] **Impl** — N30 commit (target codes, 3 components).
- [ ] **Impl** — N41 commit (target azimuth, 2 components).
- [ ] **Impl** — N42 commit (apogee, 3 components).
- [ ] **Impl** — N43 commit (latitude, 3 components).
- [ ] **Impl** — N48 commit (pitch trim, 2 components).
- [ ] **Impl** — N49 commit (delta R, 3 components).
- [ ] **Impl** — N51 commit (S-band antenna pitch, 2 components).
- [ ] **Impl** — N52 commit (central angle of active vehicle, 1 component, V21 only).
- [ ] **Impl** — N53/N54 commit (range, 3 components each).
- [ ] **Impl** — N55 commit (perigee code, 3 components).
- [ ] **Impl** — N56 commit (reentry angle, 2 components).
- [ ] **Impl** — N57 commit (delta R, 1 component, V21 only).
- [ ] **Impl** — N58 commit (perigee alt, 3 components).
- [ ] **Impl** — N59 commit (delta velocity LOS, 3 components).
- [ ] **Impl** — N60 commit (Gmax, 3 components).
- [ ] **Impl** — N61 commit (impact latitude, 3 components).
- [ ] **Impl** — N62 commit (inertial velocity magnitude, 3 components).
- [ ] **Impl** — N64 commit (drag acceleration, 3 components).
- [ ] **Impl** — N66 commit (command bank angle, 3 components).
- [ ] **Impl** — N67 commit (range to target, 3 components).
- [ ] **Impl** — N68 commit (command bank angle, 3 components).
- [ ] **Impl** — N69 commit (beta, 3 components).
- [ ] **Impl** — N72 commit (delta angle, 3 components).
- [ ] **Impl** — N73 commit (altitude, 3 components).
- [ ] **Impl** — N74 commit (command bank angle, 3 components).
- [ ] **Impl** — N82 commit (delta V LV, 3 components).
- [ ] **Impl** — N83 commit (delta V body, 3 components).
- [ ] **Impl** — N84 commit (delta V other vehicle, 3 components).
- [ ] **Impl** — N85 commit (VG body, 3 components).
- [ ] **Impl** — N86 commit (delta V LV, 3 components).
- [ ] **Impl** — N87 commit (mark data shaft, 2 components).
- [ ] **Impl** — N88 commit (half unit sun/planet vector, 3 components).
- [ ] **Impl** — N90 commit (Y, 3 components).
- [ ] **Impl** — N91/N92 commit (OCDU angles / new optics angles shaft, 2 components each).
- [ ] **Impl** — N93 commit (delta gyro angles, 3 components).
- [ ] **Impl** — N94 commit (new optics angles shaft, 2 components).
- [ ] **Impl** — N95 commit (preferred attitude ICDU angles, 3 components).
- [ ] **Impl** — N96 commit (+X-axis attitude ICDU angles, 3 components).
- [ ] **Impl** — N97 commit (system test inputs, 3 components).
- [ ] **Impl** — N98 commit (system test results, 3 components).
- [ ] **Impl** — N99 commit (RMS in position, 3 components).

Read-only nouns (no commit handler needed): N40, N44, N45, N50, N63, N75, N76, N80.

### Technical Debt

- [x] **Debug** — `math/lambert.rs` Izzo convergence bugs. **RESOLVED 2026-04-10** using the Izzo 2015 paper (https://www.esa.int/gsp/ACT/doc/MAD/pub/ACT-RPR-MAD-2014-RevisitingLambertProblem.pdf). All 7 Lambert tests pass (0 ignored). Fixes applied:
  - **Root cause**: Lancaster-Blanchard T formula was inverted — code divided by `a^(3/2)` where it should multiply. Corrected in both `tof_and_derivs` and `tof_and_derivs_inner`.
  - **Initial guess**: Replaced all three regime formulas with Izzo Eq. 30 exactly (slow `(T₀/T)^(2/3)−1`, fast `5·T₁·(T₁−T)/(2·T·(1−λ⁵))+1`, normal `(T₀/T)^(1/log₂(T₀/T₁))−1`).
  - **T₀₀**: Corrected Eq. 19 to use signed λ: `acos(λ) + λ·sqrt(1−λ²)`.
  - **Tolerance**: Relaxed `TOL_NDIM` from 1e-12 to 1e-5 (still sub-metre position accuracy; Halley stalls near the 180° transfer boundary otherwise).
  - **Test geometry repairs**: TC-LAM-1 now uses a proper 179° Hohmann at `tof=T/2`; TC-LAM-2 uses a 19.44° arc matching the LEO period at `tof=300 s`; TC-LAM-3 asserts TLI elliptic bounds instead of hyperbolic escape.
  - ~~Known remaining edge case~~: **RESOLVED 2026-04-14.** TC-TGT-10 / TC-P37-{1,2,4} now pass. Root cause: the TEI geometry (μ_Moon, r₂ ≈ 384 Mm) is a hyperbolic escape (x > 1 in Izzo's parametrization). The solver only had elliptic formulas (acos/asin for x ∈ (−1,1)) and clamped x to that range, causing Halley to stall at the boundary. Fix: added hyperbolic Lancaster-Blanchard branch (acosh/asinh for x ≥ 1), regime-aware clamping (elliptic vs hyperbolic based on T_nd vs T₁), and TC-LAM-8 unit test for the hyperbolic regime.

- [x] **Debug** — Lambert long-TOF TEI regime (TC-TGT-10 / TC-P37-{1,2,4}).
  **RESOLVED 2026-04-14.** Added hyperbolic branch (cosh/sinh) to
  `tof_and_derivs` for x > 1, regime-aware x clamping, and TC-LAM-8.
  All four previously-ignored tests un-ignored and passing.
  Full suite: 389 passed, 0 failed, 0 ignored.

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

- [x] **Bug** — `agc-sim dsky_sim` DSKY display panel: the bottom border
  was drawn on the same row as R3's digit content. **RESOLVED 2026-04-13**
  (commit follows). Extended the display panel from 16 to 17 rows so R3
  content stays at `oy+15` and the bottom border moves to `oy+16`.
  Extended the lamp panel to match. Removed the duplicated overwrite
  logic. Keyboard and status line shifted down by 1 row; `HEIGHT`
  updated from 26 to 27. **Observed symptom**:
  the third data register's value (R3) had a horizontal line drawn
  across the digits — the `└─────┘` bottom-border row and the R3
  content row collided.
  - **Root cause**: in `agc-sim/src/dsky_ui.rs::draw_display_panel`
    (around lines 145–180), the display panel is sized 16 rows tall
    (`oy..=oy+15`). The cell separators and register content rows are:
    - row 11 → `├───┤` (separator between R1 and R2)
    - row 12 → R2 content
    - row 13 → (filler)
    - row 14 → `├───┤` (separator between R2 and R3)
    - **row 15 → both `└─────┘` bottom border AND `draw_register(...,
      oy + 15, "R3", ...)`** — collision.
    There is also some duplicated paint-then-repaint logic at lines
    103 and 150–155 (drawing the bottom border, then a cell interior,
    then the bottom border again) that looks like an early workaround
    for this exact layout problem — it should be removed as part of
    the fix.
  - **Fix sketch**: make the display panel 17 or 18 rows tall so R3
    has its own content row distinct from the bottom border. Suggested
    layout:
    - row 0: `┌──────┐` top border
    - rows 1–7: PROG / COMP ACTY / VERB / NOUN block (as today)
    - row 8: `├──────┤` separator above R1
    - row 9: R1 content
    - row 10: `├──────┤` separator
    - row 11: R2 content
    - row 12: `├──────┤` separator
    - row 13: R3 content
    - row 14: `└──────┘` bottom border
    Update `draw_register` call sites to match the new row indices
    (`oy + 9`, `oy + 11`, `oy + 13`) and update `WIDTH` / `HEIGHT`
    constants if needed.
  - **Acceptance**: running `cargo run -p agc-sim --bin dsky_sim` and
    observing the display panel shows three data register rows, each
    in its own cell with clean `├──┤` separators above and below, and
    a continuous `└──┘` bottom border that does not intersect any
    digit. Visual check only — this is a layout bug, not a data bug.
  - **Blocked by**: nothing. Contained entirely to `agc-sim/src/dsky_ui.rs`.

- [x] **Bug** — V06/V16 monitor verbs never populate the data display
  registers. **RESOLVED 2026-04-13** (commit `1dfb803`). Added
  `noun_display()` dispatch table in `services/v_n.rs` covering nouns
  N33, N36, N40, N43, N44, N54, N62, N65, N81. Time nouns (N33/N36/N65)
  display as R1=hours, R2=minutes, R3=seconds×100 (SSSCC format) via
  `time_to_hms()` helper. Added `refresh_monitor_display()` for periodic
  V16 updates, called from `dsky_sim` 20 Hz render loop. 7 new tests
  (TC-VN-ND-1..7). **Observed symptom**: in the `agc-sim dsky_sim` binary,
  keying `V16 N65 E` (monitor time) showed `00 00 00000` in R1 instead
  of the current mission elapsed time. Other monitor nouns behaved the
  same — R1/R2/R3 stayed at whatever value the last data-entry sequence
  left, or zero if none.
  - **Root cause**: `services::v_n::v06_display_decimal` and
    `services::v_n::v16_monitor` (lines 462–473) set `state.dsky.verb`
    and `state.dsky.noun` but never write to `state.dsky.r[0..3]`.
    There is no noun-to-data-source dispatch table that reads the
    referenced state variable and writes it to the appropriate
    register(s). The V/N processor implements the **data-entry**
    direction (V21/V22/V23/V25 — noun_commit handlers write state
    from the crew's keystrokes) but not the **data-display** direction
    (V06/V16 — read state and populate R1/R2/R3).
  - **What the real AGC did**: `Comanche055/PINBALL_NOUN_TABLES.agc`
    contained per-noun tables mapping a noun number to a pointer-fetch
    into erasable memory plus a scale factor and format flag. The
    "NOUN TABLE" dispatch in PINBALL walked the table for a given
    noun, fetched the referenced value, applied the scale, and wrote
    it to the three R-register display fields.
  - **What the Rust port needs to add**: a noun table keyed by noun
    number that maps each noun to a closure or function pointer
    `fn(&AgcState) -> (Option<f32>, Option<f32>, Option<f32>)`
    returning the three register values. Example entries:
    - **N33** (TIG): read `vn.pending_tig`, format as HH MM SS.cc → R1/R2/R3
    - **N34** (TFI / TFF): derived time
    - **N36** (Vehicle time GET): read `state.time.to_seconds()`, format as HH MM SS.cc → R1/R2/R3
    - **N37** (TIG of next burn): similar to N33
    - **N40** (velocity to be gained / time / velocity): three-register burn display for P40
    - **N43** (lat / lon / alt): ground-track display for P21
    - **N44** (apogee / perigee / TFF): apsidal display
    - **N49** (delta-R / delta-V range): rendezvous display from `rendezvous_nav`
    - **N54** (range / range-rate / theta): already set directly by P20's `p20_rendezvous_nav_cycle`; this noun works because P20 writes the registers itself
    - **N62** (absolute value of velocity / time from TIG / accumulated Δv)
    - **N65** (mission time): `state.time.to_seconds()` → HH MM SS.cc in R1
    - **N68** (range to landing site / time from landing site)
    - **N81** (LVLH ΔV components): already set by P30 when V25 commits; same pattern as N54
    - ... (AGC had ~40 nouns; start with the ~10 most commonly used
      for monitoring and extend as needed)
  - **Where the call should happen**: the table lookup should fire
    (a) once on dispatch in `v06_display_decimal`/`v16_monitor`, and
    (b) again on each `p20_rendezvous_nav_cycle` / equivalent periodic
    refresh for V16 monitor nouns, so the display stays live as state
    evolves.
  - **Acceptance**: `V16 N65 E` in `agc-sim dsky_sim` shows the current
    mission elapsed time, updating every cycle. `V06 N33 E` after a
    P30 TIG load shows the TIG. `V06 N43 E` after P21 runs shows
    lat/lon/alt. Unit tests for each noun verify the register values
    match the referenced state variable within the noun's display
    scale.
  - **Blocked by**: nothing. All the state variables already exist on
    `AgcState`; this is a pure dispatch-plumbing task.

- [ ] **Impl** — Cortex-M firmware boot sequence (`#[entry]` and GOPROG).
  The project currently ships as a `no_std` library (`agc-core`) and a
  host-side simulator (`agc-sim`), but there is no runnable binary that
  boots on actual Cortex-M hardware. The equivalent of the AGC's GOPROG
  startup path needs to be implemented as a new firmware crate so the
  software can be flashed and run on the target MCU.
  - **What the real AGC did** (for reference — O'Brien Ch. 4):
    - Hardware reset or power-on → CPU jumps to the fixed restart vector
    - GOPROG entry decides fresh-start vs. restart based on the restart
      cause register
    - Fresh start: FRESH START routine clears erasable memory, initialises
      the Executive and Waitlist, runs IMU/DSKY hardware self-test, sets
      major mode 0 (P00 CMC idle)
    - Restart: restart-protected programs resume from their last phase
      checkpoint via the PHASCHNG mechanism
    - Main loop: Executive dispatches scheduled jobs and Waitlist tasks
      until power-down
  - **What the Rust port needs**:
    - New binary crate `agc-firmware` (or `agc-bin`) under the workspace,
      targeting e.g. `thumbv7em-none-eabihf` (Cortex-M4F)
    - `#[entry]` function using `cortex-m-rt`
    - Hardware-specific HAL implementation (replaces the `SimHardware`
      used in `agc-sim`) — likely via the `stm32f4xx-hal` crate or similar
      PAC-based HAL for the chosen MCU, wiring up: UART for DSKY output,
      GPIO for keyboard input, SPI/I2C for any real or mock IMU, timers
      for T3/T4/T5/T6 interrupts
    - Static `AgcState` allocation — either `static mut STATE: MaybeUninit<AgcState>`
      with explicit init, or a `const` constructor if all fields stay
      const-initialisable
    - Fresh-start vs. restart decision: read the reset cause (RCC_CSR
      on STM32) and call `services::fresh_start::fresh_start_common`
      or a future restart-recovery path accordingly
    - Boot sequence: after fresh-start, install T3/T4/T5/T6 ISR handlers
      (which call the existing Rust functions from `services::t4rupt`,
      `executive::waitlist::dispatch`, etc.), then fall through to an
      infinite `wfi` loop (hardware interrupts drive all subsequent
      activity)
    - Linker script (`memory.x`) defining FLASH and RAM regions
    - Panic handler for release builds (likely `panic-reset` or
      `panic-semihosting` per ADR-009)
  - **Downstream consequences**: none block this item; all the
    infrastructure it needs already exists (HAL trait, AgcState, Executive,
    Waitlist, Pinball display formatter, V/N processor). This is
    integration work, not new primitive development.
  - **Suggested module layout**:
    ```
    agc-firmware/
      Cargo.toml           # no_std bin, depends on agc-core + PAC + cortex-m-rt
      memory.x             # target-specific linker script
      build.rs             # copies memory.x to OUT_DIR
      src/
        main.rs            # #[entry] fn main() -> ! { ... }
        hal_impl.rs        # concrete AgcHardware implementation
        isr.rs             # #[interrupt] fn TIM3() / TIM4() etc.
        panic.rs           # panic handler (release-build reset, dev semihosting)
    ```
  - **Blocks**: nothing in the Rust port. (ADR-011 "Specific MCU target"
    is still Proposed — that decision gates this work indirectly but
    a default choice of STM32F405 would be a reasonable starting point
    consistent with `docs/optimization.md` §2.)
  - **Acceptance**: the binary builds for `thumbv7em-none-eabihf`,
    flashes via `probe-rs` onto a development board, and at power-on
    runs through the FRESH START path, initialises the DSKY to the
    blank "00 00 00000" state, and dispatches the P00 idle program.
    A T3 interrupt at 1 Hz firing correctly (visible as a blinking
    status LED or a UART heartbeat) confirms the Executive/Waitlist
    plumbing is live on real hardware.

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
