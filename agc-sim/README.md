# agc-sim — Host-side AGC simulator

`agc-sim` is the host-side (std, no embedded target) crate that lets you
run `agc-core` — the AGC flight software — on a developer machine without
a Nucleo board, a Pico bridge, or a BMI088 IMU.

It provides:

- a software implementation of the `AgcHardware` HAL (timers, DSKY, IMU,
  optics, engine, RCS, SECS, uplink, telemetry) — see `src/hardware.rs`,
- a simple spacecraft dynamics model (`src/physics.rs`) that responds to
  SPS / RCS commands and feeds PIPA pulses back into the simulated IMU,
- a "soft executive" (`src/runtime.rs`) that mirrors just enough of the
  bare-metal `Executive::run` loop to make the AGC autonomously progress
  on a host,
- a scripted scenario runner and `ScenarioBuilder` (`src/scenario.rs`)
  used by all integration tests in `agc-test/`,
- an interactive terminal DSKY (`src/bin/dsky_sim.rs`) that renders the
  display at ~20 Hz and accepts crew keystrokes from the keyboard.

## What it is for

- **Integration testing.** Every test under `agc-test/tests/` (per-phase
  tests and the `full_mission` end-to-end Apollo 8 walkthrough) runs
  against `agc-sim::SimHardware`. The crate is the dependency that lets
  those tests exist at all.
- **Live demos / desk walkthroughs.** The `dsky_sim` binary renders a
  textual DSKY panel plus an SPS / RCS propulsion strip in a terminal,
  so you can demo a P40 burn or a TEI scenario without bench hardware.
- **Designing and debugging V/N sequences.** Use the scripted-uplink `@`
  prompt in `dsky_sim` (see below) to replay or step through a long
  keystroke load instead of typing it by hand.

## What it is **not** — limits vs VirtualAGC

`agc-sim` is intentionally narrow. It is not a clean-room AGC emulator
the way [VirtualAGC](https://www.ibiblio.org/apollo/) is. Specifically:

| | VirtualAGC | agc-sim |
|---|---|---|
| Source of truth | Bit-accurate emulation of the Block-2 AGC machine, executing the original Comanche/Colossus rope binaries | Runs `agc-core` (the Rust port) — emulates the *spacecraft* the AGC talks to, not the AGC itself |
| Memory model | Eight erasable banks + 36 fixed banks, octal addresses, banked CPU | Flat Rust heap; AGC erasables map to `AgcState` struct fields |
| Interpreter | Implements the AGC interpretive language faithfully | Interpreter has been eliminated entirely (ADR-001) |
| Instruction cycle | 11.72 µs MCT, cycle-accurate timing | No instruction cycle — Rust code runs at native speed |
| Scheduler | Executive scans CORESET; Waitlist drives T3RUPT exactly | "Soft executive" pumps (`WaitlistPump`, `DapPump`, `T4Pump`) approximate the bare-metal schedule using wall-clock time |
| Spacecraft dynamics | None — the AGC runs against logged hardware traces | `Spacecraft` integrates SPS thrust, RCS torques, atmospheric drag, two-body gravity (Earth + Moon with SOI handover) and atmospheric heating |
| Atmosphere & entry | Not modelled | Exponential atmosphere, Sutton–Graves heating, entry-corridor dynamics (sufficient for P61–P67 sweeps) |
| Telemetry | The historical downlink lists | Full Comanche055 downlink frame is generated but written to an in-memory `Vec<u16>` plus an optional timestamped `.dnlk` file (see `SimTelemetry`) |
| Crew DSKY | Hardware-faithful electroluminescent panel emulation | ANSI-terminal 7-segment redraw at 20 Hz; flashing, lamps, R1/R2/R3, PROG/VERB/NOUN |

In short: **VirtualAGC tells you "what would the historical AGC do given
this rope-binary?". agc-sim tells you "what does the Rust port do given
this `AgcState`, when wrapped in a plausible spacecraft?".** They serve
different verification questions and are not interchangeable.

The historical reference traces (PIPA tapes, MSFN downlink fixtures, P52
sightings, etc.) used elsewhere in this project are still consumed
against VirtualAGC where they apply — see `docs/fixtures.md` and
`docs/testing.md`.

## How to start it

The simulator needs no extra setup beyond a working Rust toolchain (see
[`WORKSPACE.md`](../WORKSPACE.md) for the project-wide setup).

### Interactive DSKY

```sh
cargo run -p agc-sim --bin dsky_sim
```

This opens an alternate-screen terminal UI. Keys:

| Key | Effect |
|---|---|
| `0`–`9`, `+`, `-` | Numeric / sign input to the V/N processor |
| `v` / `n` | VERB / NOUN |
| `Enter` | ENTR |
| `p` | PRO |
| `c` | CLR |
| `r` | RSET |
| `k` | KEY REL |
| `@` | Open a script-file prompt — type a path, `Enter` loads the file into the uplink FIFO so its keystrokes replay as if the ground had uplinked them |
| `q` / `Esc` / `Ctrl-C` | Quit |

Mission Elapsed Time advances from wall-clock once the program starts.
The first frame shows P00 (CMC Idling) with all registers blank.

The render loop also writes a timestamped binary downlink log
(`dnlk_*.bin`, two big-endian bytes per word) into the current directory.
Delete it after a session if you don't need the trace.

### Scenario tests (non-interactive)

```sh
# All integration tests (uses agc-sim under the hood)
cargo test -p agc-test

# A specific scenario
cargo test -p agc-test --test p40_sps_burn
cargo test --test full_mission
```

These tests build a `ScenarioBuilder`, drive it through `run_scenario`
against a `SimHardware`, and assert against the scripted `Expect*`
events.

## How to use it — guided walkthrough

For a concrete end-to-end demonstration, follow
**[`docs/p40_burn_demo.md`](../docs/p40_burn_demo.md)**. It walks the
operator through the full V/N sequence to ignite the SPS engine for
~15 seconds in `dsky_sim`:

1. **V71 / P27 block update** — seed a 400 km circular Earth orbit
   directly into `state.csm_state.position` / `velocity`.
2. **V37 → P30** — start External-ΔV targeting.
3. **V25 N33** — load TIG (Time of Ignition) five minutes in the future.
4. **V25 N81** — load LVLH ΔV `[+21, 0, 0]` (along-track 21 m/s).
5. **V37 → P40** — switch to SPS thrust execution.
6. **PRO** — arm the engine; AGC fires `hw.engine.sps_enable(true)` at TIG.
7. **Watch V16 N40** — accumulated ΔV climbs from 0 to ~21 m/s over
   ~14 s; SERVICER autonomously cuts the engine off.

That document also explains how `SimHardware::tick` turns engine
commands into PIPA pulses and how the soft executive drives the burn to
completion. A companion **TEI** walkthrough lives at
[`docs/tei_burn_demo.md`](../docs/tei_burn_demo.md) and uses
[`scripts/tei_demo.dsky`](scripts/tei_demo.dsky) (loadable via the `@`
prompt) to seed a 111 km lunar parking orbit.

## Crate layout

| File | Contents |
|---|---|
| `src/lib.rs` | Public re-exports (`SimHardware`, `ScenarioBuilder`, …) |
| `src/hardware.rs` | `SimHardware`, `SimTimers`, `SimImu`, `SimDsky`, `SimEngine`, `SimRcs`, `SimTelemetry`, `SimSecs`, `SimOptics` |
| `src/physics.rs` | `Spacecraft` dynamics (mass, thrust, attitude quaternion, atmospheric drag, gravity, SOI handover) |
| `src/runtime.rs` | Soft-executive pumps: `WaitlistPump`, `DapPump`, `T4Pump`, `pump_*_to_hw`, `pump_pipa_into_state` |
| `src/scenario.rs` | `Event`, `Scenario`, `ScenarioBuilder`, `run_scenario`, `Expect*` assertions |
| `src/sensors.rs` | Synthetic star / landmark line-of-sight generators for P22/P23/P51/P52 |
| `src/uplink.rs` | `ScriptedUplink` — drives keystrokes from a `.dsky` script file |
| `src/dsky_ui.rs` | ANSI-terminal DSKY renderer (used by `dsky_sim`) |
| `src/bin/dsky_sim.rs` | Interactive DSKY binary |
| `scripts/*.dsky` | Sample DSKY scripts loadable via the `@` prompt |

## References

- Architecture: [`docs/architecture.md`](../docs/architecture.md)
- Test strategy: [`docs/testing.md`](../docs/testing.md)
- Domain vocabulary: [`docs/glossary.md`](../docs/glossary.md)
- P40 burn walk-through: [`docs/p40_burn_demo.md`](../docs/p40_burn_demo.md)
- TEI burn walk-through: [`docs/tei_burn_demo.md`](../docs/tei_burn_demo.md)
