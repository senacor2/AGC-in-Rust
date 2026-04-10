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
- [ ] **Impl** — `control/imu_control.rs`
- [x] **Spec** — `control/dap.rs` → `specs/dap-spec.md`
- [ ] **Impl** — `control/dap.rs`
- [x] **Spec** — `control/attitude.rs` → `specs/attitude-spec.md`
- [ ] **Impl** — `control/attitude.rs`
- [x] **Spec** — `control/rcs_logic.rs` → `specs/rcs-logic-spec.md`
- [ ] **Impl** — `control/rcs_logic.rs`
- [x] **Spec** — `control/tvc.rs` → `specs/tvc-spec.md`
- [ ] **Impl** — `control/tvc.rs`
- [x] **Spec** — `guidance/targeting.rs` → `specs/targeting-spec.md`
- [ ] **Impl** — `guidance/targeting.rs`
- [x] **Spec** — `guidance/maneuver.rs` → `specs/maneuver-spec.md`
- [ ] **Impl** — `guidance/maneuver.rs`
- [x] **Architect review** — `specs/milestone-3-architect-review.md` (10 critical issues, 6 ADs, all resolved)

### Milestone 4 — Programs (P-codes)

- [ ] P00 — CMC Idling
- [ ] P11 — Earth orbit insertion
- [ ] P40/P41 — SPS/RCS thrusting
- [ ] P51/P52 — IMU alignment
- [ ] P61–P67 — Entry guidance sequence
- [ ] P30 — External Delta-V (needed for Lambert fixture tests)
- [ ] P37 — Return to Earth
- [ ] Remaining P-codes

### Milestone 5 — DSKY and Crew Interface

- [ ] **Spec + Impl** — `services/v_n.rs` (Verb/Noun state machine)
- [ ] **Spec + Impl** — `services/display.rs` (PINBALL display driver)
- [ ] `agc-sim` terminal DSKY simulator

### Technical Debt

- [ ] **Debug** — `math/lambert.rs` Izzo convergence bugs. 4 tests currently `#[ignore]`:
  - `tc_lam_1_leo_to_meo_90deg` — residual ~6e-7 (close but not converging to 1e-12)
  - `tc_lam_2_leo_rendezvous` — velocity magnitude wrong (1329 m/s vs expected 7668 m/s)
  - `tc_lam_3_tli_like` — Halley iteration diverges (residual 3.6) on long TOF
  - `tc_lam_5_retrograde_long_way` — retrograde (λ<0) branch diverges (residual 3.0)
  - Likely causes: initial guess formula, TOF derivative expression near boundaries, or sign convention in the λ<0 branch. Needs dedicated debugging session.

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
