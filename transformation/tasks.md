# Task Tracking

## Active Tasks

_None — foundation phase complete, implementation not yet started_

## Backlog

### Milestone 1 — Core Infrastructure

- [ ] Create Cargo workspace (`agc-core`, `agc-sim`, `agc-test`) with feature flags (`sim`, `bare-metal`)
- [ ] **Spec** — `types/` module (CduAngle, Vec3, Mat3x3, Met, DeltaV newtypes with scale docs)
- [ ] **Impl** — `types/` module
- [ ] **Spec** — `AgcHardware` trait and all sub-traits (`Timers`, `Dsky`, `Imu`, `Optics`, `Engine`, `Rcs`, `Uplink`, `Telemetry`)
- [ ] **Impl** — HAL traits in `agc-core/src/hal/`
- [ ] **Impl** — Simulated HAL in `agc-sim/src/hardware.rs`
- [ ] **Spec** — `Executive` (job table, priority scheduler, NOVAC/FINDVAC, 1202 alarm)
- [ ] **Impl** — `executive/scheduler.rs`, `executive/job.rs`
- [ ] **Spec** — `Waitlist` (delta-time chain, 8 slots, T3RUPT dispatch)
- [ ] **Impl** — `executive/waitlist.rs`
- [ ] **Spec** — Restart protection (phase tables, group management, GOJAM)
- [ ] **Impl** — `executive/restart.rs`
- [ ] **Impl** — `services/alarm.rs` (alarm codes, DSKY alarm display)
- [ ] **Impl** — `services/fresh_start.rs`
- [ ] **Tests** — All Executive + Waitlist unit tests passing
- [ ] Bare-metal build clean: `cargo build --target thumbv7em-none-eabihf -p agc-core`

### Milestone 2 — Navigation Foundation

- [x] **Spec** — `math/linalg.rs` (dot, cross, norm, unit, mxv, vxm)
- [x] **Impl** — `math/linalg.rs`
- [x] **Spec** — `navigation/state_vector.rs` (StateVector, coordinate frames)
- [x] **Impl** — `navigation/state_vector.rs`
- [x] **Spec** — `navigation/gravity.rs` (Earth/Moon models, oblateness)
- [x] **Impl** — `navigation/gravity.rs`
- [x] **Spec** — `navigation/integration.rs` (Cowell / Encke propagation)
- [x] **Impl** — `navigation/integration.rs`
- [x] **Spec** — `services/average_g.rs` (SERVICER 2-second cycle)
- [x] **Impl** — `services/average_g.rs`
- [x] Capture VirtualAGC math fixtures (orbital energy conservation)
- [x] Navigation accuracy tests passing against fixtures

### Milestone 3 — Guidance and DAP

- [x] **Spec + Impl** — `math/kepler.rs` (universal-variable Kepler solver)
- [x] **Spec + Impl** — `math/lambert.rs` (Lambert targeting, universal variable)
- [x] **Spec + Impl** — `navigation/conics.rs` (orbital elements, vis-viva, apsides)
- [x] **Spec + Impl** — `control/dap.rs` (T5RUPT DAP supervisor)
- [x] **Spec + Impl** — `control/attitude.rs` (phase-plane, rate damping)
- [x] **Spec + Impl** — `control/rcs_logic.rs` (16-jet SM RCS selection)
- [x] **Spec + Impl** — `control/tvc.rs` (SPS gimbal steering, gain scheduling)
- [x] **Spec + Impl** — `guidance/targeting.rs` (TIG, burn time, maneuver plan)
- [x] **Spec + Impl** — `guidance/maneuver.rs` (VG tracking, cross-product steering)
- [ ] **Spec + Impl** — `control/imu_control.rs` (coarse/fine align, typestate) — deferred to Milestone 4

### Milestone 4 — Programs (P-codes)

- [x] P00 — CMC Idling
- [x] P11 — Earth orbit insertion monitor
- [x] P30 — External Delta-V targeting (crew-entered VGTIG)
- [x] P37 — Return to Earth (Kepler propagation + Lambert solver)
- [x] P40/P41 — SPS/RCS thrusting (6-phase burn execution)
- [x] P51/P52 — IMU alignment (SMNB two-vector determination)
- [x] P61–P67 — Entry guidance (7-phase: prep → separation → blackout → constant-g → skip → bank steer → drogue)
- [x] `control/imu_control.rs` — IMU coarse/fine align, typestate modes

### Milestone 5 — DSKY and Crew Interface

- [ ] **Spec + Impl** — `services/v_n.rs` (Verb/Noun state machine)
- [ ] **Spec + Impl** — `services/display.rs` (PINBALL display driver)
- [ ] `agc-sim` terminal DSKY simulator

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
