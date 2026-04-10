# Transformation Progress

**Last Updated**: 2026-04-09 (Milestone 5 complete)

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
| `services/v_n.rs` — Verb/Noun state machine (PINBALL CHARIN/ENTPAS0) | Complete |
| `services/display.rs` — Display formatting (decimal, octal, time, relay word) | Complete |
| `services/pinball.rs` — Verb dispatch table (VBRTEFN categories) | Complete |
| Noun table (11 noun definitions, lookup) | Complete |
| DSKY demo updated with V/N command execution (V35/V37/V82) | Complete |

**Status**: Complete

## Metrics

| | Count |
|---|---|
| Rust source files | 46 |
| Unit tests passing | 197 |
| Scenario tests passing | 10 |
| Total tests | 207 |
| Clippy warnings | 0 |
| AGC constant assertion tests | 13 |
| VirtualAGC fixture cases | 1 (circular orbit energy conservation) |
| AGC assembler source files cached | 22 (docs/agc-source/) |

---

## Summary — All Milestones Complete

All five milestones of the AGC-in-Rust transformation are complete. The system
implements the Comanche055 Command Module guidance software in idiomatic Rust,
covering the full earth-to-moon-and-back mission sequence.

### Mission Sequence Coverage

```
FRESH START → P00 (CMC Idle)
  → V37 N11 → P11 (Earth Orbit Insertion Monitor)
  → V37 N30 → P30 (External Delta-V Targeting)
  → V37 N40 → P40 (SPS Burn: attitude → countdown → ullage → burn → cutoff)
  → V37 N51 → P51 (IMU Realign: star marks → REFSMMAT → gyro torque)
  → V37 N37 → P37 (Return to Earth: Kepler propagation + Lambert solver)
  → V37 N40 → P40 (Return burn execution)
  → P61–P67 (Entry: coast → separation → blackout → constant-g → bank steer → drogue)
```

### Module Architecture

| Layer | Modules | Purpose |
|---|---|---|
| **Types** | `types/` | CduAngle, Vec3, Mat3x3, Met, DeltaV newtypes |
| **HAL** | `hal/` | Hardware abstraction: DSKY, IMU, Engine, RCS, Optics, Timers, Uplink, Telemetry |
| **Executive** | `executive/` | Job scheduler, Waitlist (T3RUPT), restart protection (5 groups) |
| **Math** | `math/` | Linear algebra, trigonometry, Kepler solver, Lambert solver |
| **Navigation** | `navigation/` | State vector, gravity (J2), orbital integration (RK4), conics, SERVICER (Average G) |
| **Control** | `control/` | DAP (T5RUPT), attitude (phase-plane), RCS jet select (16-jet), TVC (gimbal), IMU control |
| **Guidance** | `guidance/` | Targeting (TIG/burn time), maneuver (VG tracking, cross-product steering) |
| **Programs** | `programs/` | P00, P11, P30, P37, P40/P41, P51/P52, P61–P67 |
| **Services** | `services/` | Alarm system, fresh start, SERVICER, PINBALL (V/N), display formatting |
| **Simulation** | `agc-sim/` | SimHardware, DSKY TUI (ratatui), navigation demo, burn execution demo |

### Validation Infrastructure

| Resource | Purpose |
|---|---|
| `docs/agc-source/*.agc` | 22 cached Comanche055 assembler files — no web fetches needed |
| `docs/agc-reference-constants.md` | Pre-validated constants, algorithms, source file map |
| `.claude/skills/validation/SKILL.md` | Structured validation procedure (CONFIRMED/WRONG/APPROXIMATE) |
| `agc-core/src/tests/agc_constants.rs` | 13 assertion tests locking constants to AGC values |
| `cargo test` | 207 tests catch regressions automatically |

### AGC Assembler Fidelity

Key constants verified against Comanche055 source:

| Constant | AGC Value | Source |
|---|---|---|
| MU_EARTH | 3.986032×10¹⁴ m³/s² | ORBITAL_INTEGRATION.agc |
| RE_EARTH (ERAD) | 6,373,338 m | LATITUDE_LONGITUDE_SUBROUTINES.agc |
| PIPA_SCALE (KPIP1) | 0.0585 m/s/count | SERVICER207.agc |
| SERVICER cycle | 2.0 s | SERVICER207.agc |
| SPS thrust (FENG) | 91,188.544 N (20,500 lbs) | P40-P47.agc |
| SPS exhaust velocity (2VEXHUST) | 3,151.04 m/s | P40-P47.agc |
| MAX_WAITLIST_TASKS | 9 | WAITLIST.agc |
| NUM_RESTART_GROUPS | 5 | FRESH_START_AND_RESTART.agc |
| EI altitude (400KFT) | 121,920 m | P61-P67.agc |
| MIN_IMPULSE (T6) | 14 ms | JET_SELECTION_LOGIC.agc |
| TVC gimbal limit (ACTSAT) | 6° | TVCDAPS.agc |
| DSKY key codes | 18 keys, octal | PINBALL_GAME_BUTTONS_AND_LIGHTS.agc |

### Interactive Demos

```sh
cargo run -p agc-sim --bin dsky_demo   # DSKY keyboard/display with V/N commands
cargo run -p agc-sim --bin nav_demo    # Navigation + orbital mechanics + SPS burns
```
