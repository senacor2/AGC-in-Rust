# Transformation Progress

**Last Updated**: 2026-04-09 (Milestone 4 complete)

## Foundation (Complete)

- [x] Architecture designed — `docs/architecture.md`
- [x] Testing strategy defined — `docs/testing.md`
- [x] Rust Embedded Book compliance analysis — `docs/optimization.md`
- [x] All ADRs documented — `transformation/decisions.md`
- [x] Agent workflow defined — `CLAUDE.md`, `.claude/agents/`

## Milestone 1: Core Infrastructure

| Task | Status |
|---|---|
| Cargo workspace scaffold (`agc-core`, `agc-sim`, `agc-test`) | Complete |
| `types/` — `CduAngle`, `Vec3`, `Mat3x3`, `Met`, `DeltaV` newtypes | Complete |
| `hal/` — `AgcHardware` trait + all sub-traits | Complete |
| `hal/` — Simulated HAL implementation in `agc-sim` | Complete |
| `executive/` — Job table, priority scheduler, `EXEC` loop | Complete |
| `executive/` — Waitlist (delta-time chain, 8 slots) | Complete |
| `executive/` — Restart protection / phase tables | Complete |
| `services/alarm.rs` — Program alarm system (1202, 1210, 1211) | Complete |
| `services/fresh_start.rs` — FRESH START / RESTART sequences | Complete |
| All Executive + Waitlist unit tests passing | Complete (25 tests) |

**Status**: Complete

## Milestone 2: Navigation Foundation

| Task | Status |
|---|---|
| `math/` — linalg (dot, cross, norm, mxv), trig wrappers | Complete |
| `navigation/state_vector.rs` — StateVector, coordinate frames | Complete |
| `navigation/integration.rs` — Cowell / Encke propagation | Complete |
| `navigation/gravity.rs` — Earth/Moon gravity models | Complete |
| `services/average_g.rs` — SERVICER 2-second nav cycle | Complete |
| Math fixtures captured from VirtualAGC (see `docs/testing.md`) | Complete (orbital energy conservation) |
| Navigation accuracy tests passing against VirtualAGC fixtures | Complete (3 scenario tests) |

**Status**: Complete

## Milestone 3: Guidance and DAP

| Task | Status |
|---|---|
| `math/kepler.rs` — Universal-variable Kepler solver | Complete |
| `math/lambert.rs` — Lambert's problem (universal variable) | Complete |
| `navigation/conics.rs` — Orbital elements, vis-viva, apsides | Complete |
| `control/dap.rs` — Digital Autopilot supervisor (T5RUPT) | Complete |
| `control/attitude.rs` — Phase-plane attitude error, rate damping | Complete |
| `control/rcs_logic.rs` — SM RCS jet selection (16-jet topology) | Complete |
| `control/tvc.rs` — TVC gimbal steering, gain scheduling | Complete |
| `guidance/targeting.rs` — TIG, delta-V, burn time (Tsiolkovsky) | Complete |
| `guidance/maneuver.rs` — VG tracking, cross-product steering | Complete |

**Status**: Complete

## Milestone 4: Programs (P-codes)

| Task | Status |
|---|---|
| P00 — CMC Idling | Complete |
| P11 — Earth orbit insertion monitor | Complete |
| P30 — External Delta-V targeting | Complete |
| P37 — Return to Earth (Lambert) | Complete |
| P40/P41 — SPS/RCS thrusting (6-phase burn execution) | Complete |
| P51/P52 — IMU alignment (SMNB two-vector method) | Complete |
| P61–P67 — Entry guidance (7-phase state machine) | Complete |
| `control/imu_control.rs` — IMU coarse/fine align, typestate | Complete |

**Status**: Complete

## Milestone 5: DSKY and Crew Interface

| Task | Status |
|---|---|
| `services/v_n.rs` — Verb/Noun processor (PINBALL) | Not Started |
| `services/display.rs` — Display formatting, flashing | Not Started |
| `hal/dsky.rs` — DSKY relay/display peripheral interface | Not Started |
| DSKY simulator in `agc-sim` | Not Started |

**Status**: Not Started

## Metrics

| | Count |
|---|---|
| Rust source files | 43 |
| Unit tests passing | 171 |
| Scenario tests passing | 10 |
| Clippy warnings | 0 |
| VirtualAGC fixture cases | 1 (circular orbit energy conservation) |
