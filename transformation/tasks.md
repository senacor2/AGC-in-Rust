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

- [ ] **Spec** — `math/linalg.rs` (dot, cross, norm, unit, mxv, vxm)
- [ ] **Impl** — `math/linalg.rs`
- [ ] **Spec** — `navigation/state_vector.rs` (StateVector, coordinate frames)
- [ ] **Impl** — `navigation/state_vector.rs`
- [ ] **Spec** — `navigation/gravity.rs` (Earth/Moon models, oblateness)
- [ ] **Impl** — `navigation/gravity.rs`
- [ ] **Spec** — `navigation/integration.rs` (Cowell / Encke propagation)
- [ ] **Impl** — `navigation/integration.rs`
- [ ] **Spec** — `services/average_g.rs` (SERVICER 2-second cycle)
- [ ] **Impl** — `services/average_g.rs`
- [ ] Capture VirtualAGC math fixtures (see `docs/testing.md §6`)
- [ ] Navigation accuracy tests passing against fixtures

### Milestone 3 — Guidance and DAP

- [ ] **Spec + Impl** — `math/kepler.rs` (KEPRTN)
- [ ] **Spec + Impl** — `math/lambert.rs` (Lambert targeting)
- [ ] **Spec + Impl** — `navigation/conics.rs`
- [ ] **Spec + Impl** — `control/imu_control.rs` (coarse/fine align, typestate)
- [ ] **Spec + Impl** — `control/dap.rs` (T5RUPT driven)
- [ ] **Spec + Impl** — `control/attitude.rs`
- [ ] **Spec + Impl** — `control/rcs_logic.rs` (jet select, T6RUPT timing)
- [ ] **Spec + Impl** — `control/tvc.rs`
- [ ] **Spec + Impl** — `guidance/targeting.rs`
- [ ] **Spec + Impl** — `guidance/maneuver.rs`

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

## Completed

- [x] Architecture — `docs/architecture.md`
- [x] Testing strategy — `docs/testing.md`
- [x] Rust Embedded Book compliance — `docs/optimization.md`
- [x] ADRs — `transformation/decisions.md`
