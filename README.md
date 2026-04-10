# AGC-in-Rust

AI agents port the Apollo Guidance Computer to idiomatic Rust.

The goal is to transform the AGC Comanche055 assembler code — the flight
software for the **Command Module** — into readable, maintainable Rust,
re-creating the abstractions that were lost when the original was written in
1960s assembler. Scope: **earth → moon → earth travel**. Lunar landing is
out of scope.

All five milestones are complete. The system can execute a full Apollo
mission sequence: fresh start → orbit insertion → targeting → SPS burn →
IMU alignment → return targeting → entry through drogue deploy.

## Quick Start

```sh
# Run the unified simulator (interactive DSKY + navigation + mission timeline)
cargo run -p agc-sim --bin agc_sim

# Pre-loaded scenarios
cargo run -p agc-sim --bin agc_sim -- --scenario launch   # P11 launch monitor
cargo run -p agc-sim --bin agc_sim -- --scenario burn     # P40 SPS burn demo
cargo run -p agc-sim --bin agc_sim -- --scenario free     # Free flight sandbox

# Run the full test suite
cargo test --workspace

# Build for the embedded target (bare-metal thumbv7em-none-eabihf)
cargo build --target thumbv7em-none-eabihf -p agc-core
```

Switch scenarios live with **F1 / F2 / F3**. Drive the DSKY with standard
Apollo verb/noun keys (`V` `3` `7` `N` `4` `0` `Enter` → enter program P40).
Press `+` / `-` to accelerate/slow time for demos. `Q` to quit.

See [`agc-sim/README.md`](agc-sim/README.md) for the full keyboard map and
presentation playbook.

## Workspace Layout

| Crate | Purpose |
|---|---|
| **`agc-core`** | `#![no_std]` AGC core: executive, HAL traits, navigation, guidance, control, programs, services. Builds for `thumbv7em-none-eabihf`. |
| **`agc-sim`**  | Host-side `std` crate: simulated HAL, DSKY TUI, unified mission simulator. |
| **`agc-test`** | Integration and scenario tests that exercise `agc-core` via `agc-sim`. |

## What's Implemented

All 5 milestones complete — **207 tests passing**.

| Milestone | Modules | Highlights |
|---|---|---|
| **1. Core Infrastructure** | `executive/`, `hal/`, `types/`, `services/alarm`, `services/fresh_start` | Priority scheduler, Waitlist (9-slot delta-time chain), restart protection (5 groups), 1202/1210/1211 alarms |
| **2. Navigation Foundation** | `math/linalg`, `math/trig`, `navigation/*`, `services/average_g` | State vector (ECI / body / stable-member), Earth+Moon gravity with J2, SERVICER predictor-corrector (Störmer-Verlet) |
| **3. Guidance and DAP** | `math/kepler`, `math/lambert`, `navigation/conics`, `control/*`, `guidance/*` | Universal-variable Kepler+Lambert, classical orbital elements, T5RUPT DAP, 16-jet RCS selection, TVC, cross-product steering |
| **4. Programs (P-codes)** | `programs/p00–p67`, `control/imu_control` | P00 idle, P11 monitor, P30 targeting, P37 return (Lambert), P40/P41 burn execution, P51/P52 IMU alignment, P61–P67 entry |
| **5. DSKY and Crew Interface** | `services/v_n`, `services/display`, `services/pinball` | PINBALL verb/noun state machine, display formatting (decimal/octal/time), noun table, 18 DSKY key codes |

**Complete mission sequence supported:**
```
FRESH START → P00 (idle) → V37N11 → P11 (orbit monitor)
  → V37N30 → P30 (target Δv) → V37N40 → P40 (SPS burn)
  → V37N51 → P51 (IMU realign) → V37N37 → P37 (return)
  → V37N40 → P40 (return burn) → P61–P67 (entry) → drogue deploy
```

## Architecture

The Rust implementation preserves AGC constraints and fidelity:

- **Hard real-time, no OS** — the software owns the scheduler (Executive + Waitlist)
- **Bare-metal `#![no_std]`** — zero heap, static allocation, `Mutex<RefCell<T>>` for shared state
- **Interrupt-driven** — T3RUPT (Waitlist), T4RUPT (DSKY relay), T5RUPT (DAP), T6RUPT (RCS jets)
- **AGC-value fidelity** — constants match Comanche055 exactly, not modern IAU/WGS84 values
- **Robust recovery** — phase tables bracket multi-step computations for restart safety

Key AGC constants are locked to assembler values via regression tests
(see [`agc-core/src/tests/agc_constants.rs`](agc-core/src/tests/agc_constants.rs)):

| Constant | Value | AGC Source |
|---|---|---|
| `MU_EARTH` | 3.986032×10¹⁴ m³/s² | `ORBITAL_INTEGRATION.agc` |
| `RE_EARTH` (ERAD) | 6,373,338 m (Fischer ellipsoid) | `LATITUDE_LONGITUDE_SUBROUTINES.agc` |
| `PIPA_SCALE` (KPIP1) | 0.0585 m/s per count | `SERVICER207.agc` |
| `CYCLE_DT` | 2.0 s | `SERVICER207.agc` |
| `SPS_THRUST_N` (FENG) | 91,188.544 N (20,500 lbs) | `P40-P47.agc` |
| `SPS_VE_MS` (2VEXHUST) | 3,151.04 m/s | `P40-P47.agc` |
| `MAX_WAITLIST_TASKS` | 9 | `WAITLIST.agc` |
| `NUM_RESTART_GROUPS` | 5 | `FRESH_START_AND_RESTART.agc` |
| `ENTRY_INTERFACE_M` (400KFT) | 121,920 m | `P61-P67.agc` |
| DSKY key codes | 18 keys, octal-encoded | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` |

See [`docs/architecture.md`](docs/architecture.md) for the full design and
[`docs/agc-reference-constants.md`](docs/agc-reference-constants.md) for the
complete AGC constant catalogue.

## Validation Infrastructure

| Resource | Purpose |
|---|---|
| `docs/agc-source/*.agc` | 22 cached Comanche055 assembler files (no web fetches needed) |
| `docs/agc-reference-constants.md` | Pre-extracted constants, algorithms, source file map |
| `.claude/skills/validation/SKILL.md` | Structured validation procedure (CONFIRMED/WRONG/APPROXIMATE) |
| `agc-core/src/tests/agc_constants.rs` | 13 assertion tests locking constants to AGC values |
| `cargo test --workspace` | 207 tests catch regressions automatically |

## AI Agent Workflow

Work proceeds through specialised agents defined in `.claude/agents/` and
skills in `.claude/skills/`:

1. **analyst-reengineer** — reads AGC assembler source, produces functional specs
2. **architect** — designs the Rust architecture from the specs
3. **developer** — implements Rust code following the spec and architecture
4. **code-review** / **validation** — validates the Rust implementation against the AGC source
5. **tester** — writes unit tests and scenario tests

See [`CLAUDE.md`](CLAUDE.md) for the collaboration guide and
[`transformation/progress.md`](transformation/progress.md) for the full
milestone history.

## References

- [Apollo-11 source on GitHub](https://github.com/chrislgarry/Apollo-11) — digitised AGC assembler source
- Frank O'Brien — *The Apollo Guidance Computer: Architecture and Operation*
- W. David Woods — *How Apollo Flew To The Moon*
- James E. Tomayko — *Computers in Spaceflight* (NASA CR-182505)
- [Block II AGC Assembly Language Manual](https://www.ibiblio.org/apollo/assembly_language_manual.html)
- [AGC Symbolic Listing Information](https://www.ibiblio.org/apollo/Documents/SymbolicListingInformation.pdf)
- [Rust Embedded Book](https://github.com/rust-embedded) — target environment reference
