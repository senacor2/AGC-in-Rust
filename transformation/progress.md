# Transformation Progress

**Last Updated**: 2026-04-09 (Milestone 1 complete)

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
| `math/` — linalg (dot, cross, norm, mxv), trig wrappers | Not Started |
| `navigation/state_vector.rs` — StateVector, coordinate frames | Not Started |
| `navigation/integration.rs` — Cowell / Encke propagation | Not Started |
| `navigation/gravity.rs` — Earth/Moon gravity models | Not Started |
| `services/average_g.rs` — SERVICER 2-second nav cycle | Not Started |
| Math fixtures captured from VirtualAGC (see `docs/testing.md`) | Not Started |
| Navigation accuracy tests passing against VirtualAGC fixtures | Not Started |

**Status**: Not Started — depends on Milestone 1

## Milestone 3: Guidance and DAP

| Task | Status |
|---|---|
| `math/kepler.rs` — Kepler equation solver | Not Started |
| `math/lambert.rs` — Lambert's problem | Not Started |
| `navigation/conics.rs` — Conic trajectory routines | Not Started |
| `control/dap.rs` — Digital Autopilot supervisor | Not Started |
| `control/attitude.rs` — Rate damping, attitude hold, maneuver | Not Started |
| `control/tvc.rs` — Thrust Vector Control | Not Started |
| `control/rcs_logic.rs` — Jet select logic | Not Started |
| `guidance/targeting.rs` — TIG computation | Not Started |
| `guidance/maneuver.rs` — Delta-V, cross-product steering | Not Started |

**Status**: Not Started — depends on Milestone 2

## Milestone 4: Programs (P-codes)

| Task | Status |
|---|---|
| P00 — CMC Idling | Not Started |
| P11 — Earth orbit insertion monitor | Not Started |
| P40/P41 — SPS/RCS thrusting | Not Started |
| P51/P52 — IMU alignment | Not Started |
| P61–P67 — Entry programs | Not Started |
| Remaining P-codes (P01, P06, P15, P20–P23, P30–P37, P47) | Not Started |

**Status**: Not Started — depends on Milestone 3

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
| Rust source files | 18 |
| Unit tests passing | 25 |
| Clippy warnings | 0 |
| VirtualAGC fixture cases | 0 |
