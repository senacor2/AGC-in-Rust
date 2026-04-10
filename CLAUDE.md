# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project uses AI agents to port the Apollo Guidance Computer (AGC) to idiomatic Rust. The scope is the **Comanche055** module (Command Module), covering earth-to-moon-and-back travel. Lunar landing is out of scope. The goal is to re-create the abstractions lost when the original AGC assembler code was written, producing readable and maintainable Rust.

The target system is a bare-metal, hard real-time computer with very limited memory and CPU. There is no operating system — task scheduling is part of the navigation software itself.

## Agent Workflow

Work proceeds through a pipeline of specialized agents defined in `.claude/agents/`:

1. **analyst-reengineer** — reads the AGC assembler source and reference docs, produces functional specifications per component
2. **architect** — designs the Rust architecture based on functional specs; uses `EnterPlanMode`/`ExitPlanMode`
3. **developer** — implements Rust code following architect guidelines and analyst specs
4. **tester** — writes Rust unit tests (per public interface) and system tests

Each agent reads outputs from the prior stage. The analyst's functional specs are the primary input to the architect and developer.

## Key Reference Material

- `docs/AGC Symbolic Listing.md` — markdown conversion of the formal AGC hardware/software specification (Block 2 AGC, Comanche/Colossus 2D for Apollo 13)
- `/Users/Juergen.Schiewe/Documents/Digital Editions/The Apollo Guidance Computer.pdf` — Frank O'Brien: *The Apollo Guidance Computer - Architecture and Operation*. Comprehensive reference on AGC hardware, software architecture, and mission operations. Use for understanding the Executive, Waitlist, interpreter, navigation algorithms, and DSKY interface in depth.
- [Apollo-11 source on GitHub](https://github.com/chrislgarry/Apollo-11) — digitized AGC assembler source (Comanche055 = Command Module)
- [AGC Assembly Language Manual](https://www.ibiblio.org/apollo/assembly_language_manual.html) — machine, interpreter, and pseudocode instruction descriptions
- [Izzo 2015 "Revisiting Lambert's problem"](https://www.esa.int/gsp/ACT/doc/MAD/pub/ACT-RPR-MAD-2014-RevisitingLambertProblem.pdf) — the Lambert solver algorithm used in `math/lambert.rs`. Key equations: Eq. 18 (T formula), Eq. 19 (T₀₀ with signed λ), Eq. 21 (T₁), Eq. 22 (derivatives), Eq. 30 (initial guess piecewise formulas for slow/normal/fast regimes). Retrievable via WebFetch; extract text with `pdftotext` (from `brew install poppler`).

## Build & Test

```sh
cargo build                                                    # build (host)
cargo build --target thumbv7em-none-eabihf -p agc-core         # bare-metal build
cargo test                                                     # run all tests
cargo test -p agc-core -- executive                            # run tests for a module
cargo test <name>                                              # run a single test by name
cargo clippy                                                   # lint
```

## Architecture Constraints

The Rust implementation must reflect the original AGC constraints:
- Hard real-time scheduling (no OS; the software owns the scheduler)
- Minimal memory footprint
- Robust error recovery — always return to a safe state on errors
- Inputs: stellar positions, inertial navigation platform (orientation + acceleration)
- Outputs: thruster control (orientation changes), main engine control (velocity changes)
- Crew interface: simple console (DSKY-style) for invoking navigation programs
