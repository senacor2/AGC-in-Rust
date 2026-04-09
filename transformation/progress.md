# Transformation Progress

**Last Updated**: 2026-04-09

## Foundation (Complete)

- [x] Architecture designed — `docs/architecture.md`
- [x] Testing strategy defined — `docs/testing.md`
- [x] Rust Embedded Book compliance analysis — `docs/optimization.md`
- [x] All ADRs documented — `transformation/decisions.md`
- [x] Agent workflow defined — `CLAUDE.md`, `.claude/agents/`

## Milestone 1: Core Infrastructure

| Task | Status |
|---|---|
| Cargo workspace scaffold (`agc-core`, `agc-sim`, `agc-test`) | **Done** |
| Spec — `types/` module | **Done** → `specs/types-module-spec.md` |
| Impl — `types/` (`CduAngle`, `Vec3`, `Mat3x3`, `Met`, `DeltaV`) | **Done** — 16 tests passing |
| Spec — `AgcHardware` + all sub-traits | **Done** → `specs/hal-spec.md` |
| Impl — HAL traits in `agc-core/src/hal/` | **Done** — aligned with spec |
| Impl — Simulated HAL in `agc-sim` | **Done** — 26 tests passing |
| Spec — Executive, Waitlist, Restart protection | **Done** → `specs/executive-spec.md` |
| Impl — `executive/scheduler.rs`, `executive/job.rs` | Skeleton — needs full impl per spec |
| Impl — `executive/waitlist.rs` | Skeleton — `todo!()` stubs |
| Impl — `executive/restart.rs` | Basic structs done — needs restart sequence |
| Impl — `services/alarm.rs` (alarm codes, DSKY alarm display) | **Done** |
| Impl — `services/fresh_start.rs` (FRESH START / RESTART) | Stub only |
| All Executive + Waitlist unit tests passing | Not Started |
| Bare-metal build clean | Not Tested |

**Status**: In Progress — specs complete, 6 of 14 items implemented and tested

## Milestone 2: Navigation Foundation

| Task | Status |
|---|---|
| `math/linalg.rs` — dot, cross, norm, mxv, vxm, transpose, mxm | **Done** — 5 tests passing |
| `math/trig.rs` — sin, cos, asin, acos, atan2 via libm | **Done** |
| `navigation/state_vector.rs` — StateVector, coordinate frames | **Done** — struct defined |
| `navigation/gravity.rs` — Earth/Moon gravity models | Stub — `todo!()` |
| `navigation/integration.rs` — Cowell / Encke propagation | Stub — `todo!()` |
| `services/average_g.rs` — SERVICER 2-second nav cycle | Stub |
| Math fixtures captured from VirtualAGC | Not Started |
| Navigation accuracy tests passing against fixtures | Not Started |

**Status**: Partially started — linalg and trig done, remaining items depend on M1 completion

## Milestone 3: Guidance and DAP

| Task | Status |
|---|---|
| `math/kepler.rs` — Kepler equation solver | Stub — `todo!()` |
| `math/lambert.rs` — Lambert's problem | Stub — `todo!()` |
| `navigation/conics.rs` — Conic trajectory routines | Stub |
| `control/dap.rs` — Digital Autopilot supervisor | Stub — `DapState` defined |
| `control/attitude.rs` — Rate damping, attitude hold, maneuver | Stub |
| `control/tvc.rs` — Thrust Vector Control | Stub — `TvcState` defined |
| `control/rcs_logic.rs` — Jet select logic | Stub |
| `guidance/targeting.rs` — TIG computation | Stub — `Maneuver` struct defined |
| `guidance/maneuver.rs` — Delta-V, cross-product steering | Stub |

**Status**: Not Started — depends on Milestone 2

## Milestone 4: Programs (P-codes)

| Task | Status |
|---|---|
| P00 — CMC Idling | Stub — `todo!()` |
| P11 — Earth orbit insertion monitor | Stub — `todo!()` |
| P40/P41 — SPS/RCS thrusting | Stub — `todo!()` |
| P51/P52 — IMU alignment | Stub — `todo!()` |
| P61–P67 — Entry programs | Stub — `todo!()` |
| Remaining P-codes (P01, P06, P15, P20–P23, P30–P37, P47) | Stub — `todo!()` |

**Status**: Not Started — depends on Milestone 3

## Milestone 5: DSKY and Crew Interface

| Task | Status |
|---|---|
| `services/v_n.rs` — Verb/Noun processor (PINBALL) | Stub |
| `services/display.rs` — Display formatting, flashing | Stub — `DskyState` defined |
| DSKY simulator in `agc-sim` | Stub |

**Status**: Not Started

## Metrics

| | Count |
|---|---|
| Rust source files | 64 |
| Unit tests passing | 42 (16 types + 5 linalg + 26 HAL sim - 5 overlap) |
| Spec documents | 3 (types, hal, executive) |
| Clippy warnings | Not checked |
| VirtualAGC fixture cases | 0 |
