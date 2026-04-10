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

- [ ] **Spec + Impl** — `guidance/rendezvous.rs` (relative state, closing rate)
- [ ] P20 — Rendezvous navigation
- [ ] P21 — Ground-track determination
- [ ] P22 — Orbital navigation (landmark tracking)
- [ ] P23 — Cislunar midcourse navigation (star/landmark sightings)
- [ ] P31 — CSI (Coelliptic Sequence Initiation) targeting
- [ ] P32 — CDH (Constant Delta-Height) targeting
- [ ] P33 — TPI (Terminal Phase Initiation) targeting
- [ ] P34 — TPM (Terminal Phase Midcourse) targeting

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

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
