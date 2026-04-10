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

- [~] **Debug** — `math/lambert.rs` Izzo convergence bugs. Status: analyst review complete (see `specs/lambert-spec.md` Appendix A), 3 partial fixes applied, 4 tests remain `#[ignore]`. Breakdown:
  - **Applied fixes**:
    - Bug 1: `h_hat` sign for retrograde transfers — correct per Izzo formula
    - Bug 2: `T₁ = (2/3)(1-λ³)` now uses signed λ³ — correct regime boundary for retrograde
    - Bug 3: Initial guess for T >> T₀₀ clamped to x₀ ≥ -0.5 (stopgap, not full Izzo Eq. 23-24)
  - **Remaining open tests** (each with a specific diagnosis):
    - `tc_lam_1_hohmann_400_to_1200km` — **test geometry FIXED** (was `tc_lam_1_leo_to_meo_90deg` with ill-posed tof=T/4). Now uses a proper Hohmann transfer (179° arc, tof=T/2) with analytical vis-viva expected values. Lambert converges to residual ~1.2e-5, just over the 1e-6 tolerance. **Real Lambert bug**: the near-Hohmann (dnu close to π) regime needs tolerance tightening or better T'' handling near the numerical boundary.
    - `tc_lam_2_leo_circular_arc_5min` — **test geometry FIXED** (was pathological 0.3° arc in 300s). Now uses a 19.44° arc matching the 300s at circular velocity — a zero-delta-V baseline where Lambert should return v_circ. **Real Lambert bug**: Lambert converges but to the wrong x value, producing |v1| ≈ 5404 m/s instead of 7668 m/s (ratio ~√2). Suggests a factor-of-2 error in the T(x,λ) formula or velocity reconstruction near the minimum-energy regime (x ≈ 1).
    - `tc_lam_3_tli_like` — **initial guess insufficient** for long TOF (TLI, T_nd >> T_00). Stopgap clamp to x₀=-0.5 did not help. Fix: implement Izzo (2015) Eq. 23-24 exactly.
    - `tc_lam_5_retrograde_long_way` — **retrograde branch still diverges** (residual 3.0) despite Bug 1+2 fixes. Needs further investigation of sign dependencies in T(x,λ) for λ<0.
  - **Summary**: The test geometries are now all physically consistent. The remaining failures are genuine Lambert algorithm bugs in 4 regimes: near-Hohmann (TC-LAM-1), near-minimum-energy (TC-LAM-2), long TOF (TC-LAM-3), and retrograde (TC-LAM-5).
  - **Analyst follow-up**: Could not fetch the Izzo 2015 paper PDF from the environment. Verified formulas by mathematical derivation and reference to pykep C++ source. Concluded: γ, T(x,λ), velocity reconstruction, and derivative signs are all correct per Izzo. Suspected bugs were initial guess stopgap (Fix 1) and Newton overshoot (Fix 2) — applied and tested, but did NOT resolve any of the 4 failing cases.
  - **Deeper investigation for TC-LAM-2 (manual calculation)**:
    - Circular orbit baseline: x_correct ≈ 0.6447, gives T(x,λ) ≈ 0.3809 matching T_nd ≈ 0.3798 (verified by hand computation)
    - Code's Halley iteration converges to x ≈ 0.36 (wrong root)
    - Since T and the initial guess are correct, the bug must be in **T' or T'' derivative formulas** — the Halley step is computing a wrong step direction
  - **Next session priority**: Instrument the Halley iteration with per-step logging. Compare T(x), T'(x), T''(x) at x=0.6 and x=0.5 against a hand-computed reference. Look for sign errors or missing factor-of-2 in the derivative formulas.
  - **Cannot-fix-in-this-environment obstacles**: Paper PDF access blocked; pykep source comparison blocked by sandbox; need actual paper or side-by-side with a known-good reference.
  - **Working baseline**: TC-LAM-4 (lunar orbit), TC-LAM-6 (anti-parallel panic), TC-LAM-7 (zero separation panic) all pass — 3/7 tests covering the panic paths and one nominal short-arc case.

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
