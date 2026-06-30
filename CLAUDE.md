# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project uses AI agents to port the Apollo Guidance Computer (AGC) to idiomatic Rust. The scope is the **Comanche055** module (Command Module), covering earth-to-moon-and-back travel. Lunar landing is out of scope. The goal is to re-create the abstractions lost when the original AGC assembler code was written, producing readable and maintainable Rust.

The target system is a bare-metal, hard real-time computer with very limited memory and CPU. There is no operating system — task scheduling is part of the navigation software itself.

## Agent Workflow

Work proceeds through a pipeline of specialized subagents defined in `.claude/agents/`:

1. **analyst-reengineer** — reads the AGC assembler source and reference docs, produces functional specifications per component. This step is optional, if there is no legacy code involved. The analyst-reengineer however can be consulted later in the process if AGC-related questions arise.
2. **orbital-mechanics** - produces a specification of the underlying physics of spaceflight for the architect. This step is optional but the orbital-mechanics agent must be consulted when issues arise concerning the physical model.
2. **architect** — designs the Rust architecture based on functional specs; uses `EnterPlanMode`/`ExitPlanMode`
3. **developer** — implements Rust code following architect guidelines and analyst specs
4. **tester** — writes Rust unit tests (per public interface) and system tests
5. **debugger** - invoked when tests break or do not deliver the expected results. The developer may ask the debugger about the actual behaviour of the code.

Each agent reads outputs from the prior stage. The analyst's functional specs are the primary input to the architect and developer.

## Key Reference Material

- `input/AGC Symbolic Listing.md` — markdown conversion of the formal AGC hardware/software specification (Block 2 AGC, Comanche/Colossus 2D for Apollo 13)
- `input/AGC Quick Reference.md` - a brief overview of the AGC machine
and interpreter instructions, registers, interrupts and I/O ports.
- `/Users/Juergen.Schiewe/Documents/Digital Editions/The Apollo Guidance Computer.pdf` — Frank O'Brien: *The Apollo Guidance Computer - Architecture and Operation*. Comprehensive reference on AGC hardware, software architecture, and mission operations. Use for understanding the Executive, Waitlist, interpreter, navigation algorithms, and DSKY interface in depth.
- [Apollo-11 source on GitHub](https://github.com/chrislgarry/Apollo-11) — digitized AGC assembler source (Comanche055 = Command Module)
- [AGC Assembly Language Manual](https://www.ibiblio.org/apollo/assembly_language_manual.html) — machine, interpreter, and pseudocode instruction descriptions
- [Izzo 2015 "Revisiting Lambert's problem"](https://www.esa.int/gsp/ACT/doc/MAD/pub/ACT-RPR-MAD-2014-RevisitingLambertProblem.pdf) — the Lambert solver algorithm used in `math/lambert.rs`. Key equations: Eq. 18 (T formula), Eq. 19 (T₀₀ with signed λ), Eq. 21 (T₁), Eq. 22 (derivatives), Eq. 30 (initial guess piecewise formulas for slow/normal/fast regimes). Retrievable via WebFetch; extract text with `pdftotext` (from `brew install poppler`).

## Build & Test

Rust has been installed via rustup. The rust proxies can be found
in `/opt/homebrew/opt/rustup/bin`.

```sh
cargo build                                                                  # build host crates (default-members)
cargo build --target thumbv7em-none-eabihf -p agc-core                       # agc-core for thumbv7em
cargo build -p agc-board-nucleo-f767 --target thumbv7em-none-eabihf          # Nucleo board binary
cargo build -p agc-bridge-pico       --target thumbv6m-none-eabi             # Pico bridge binary
cargo test                                                                   # run all tests
cargo test -p agc-core -- executive                                          # run tests for a module
cargo test <name>                                                            # run a single test by name
cargo clippy                                                                 # lint
```

Always pass `-p <crate>` when building a board binary. A bare
`cargo build --target thumbv7em-none-eabihf` (or `--workspace`) pulls both
board crates into one build graph, which makes Cargo unify the conflicting
`critical-section` impl features and triggers the `RawRestoreStateInner`
compile error (see issue #102).

## Version control and task tracking

The project is held in a git repository which is synched to the github senacor2/AGC-in-Rust remote.
All tasks shall be tracked as github issues. Any task lists in markdown files or tasks left in specifactions
are deprecated. Update the issue status when you start working on the ticket and when you close the work package. Issues shall have acceptance criteria and before the issue is closed, the criteria must be fulfilled.

We use feature branching to implement major changes with one feature per change.
The feature branch must be linked to the top-level issue and the name must match the change.


## Architecture Constraints

The Rust implementation must reflect the original AGC constraints:

- Hard real-time scheduling (no OS; the software owns the scheduler)
- Minimal memory footprint
- Robust error recovery — always return to a safe state on errors
- Inputs: stellar positions, inertial navigation platform (orientation + acceleration)
- Outputs: thruster control (orientation changes), main engine control (velocity changes)
- Crew interface: simple console (DSKY-style) for invoking navigation programs

## License

The source code of this project is under GPLv3 as described in the [README](README.md). 
Dependencies shall only be selected if their license is compatible with integration into a GPLv3 project.
Also make sure that the SPDX-License-Identifier is added to each new source file.
