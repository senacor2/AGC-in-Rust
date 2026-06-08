# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project uses AI agents to port the Apollo Guidance Computer (AGC) to idiomatic Rust. The scope is the **Comanche055** module (Command Module), covering earth-to-moon-and-back travel. Lunar landing is out of scope. The goal is to re-create the abstractions lost when the original AGC assembler code was written, producing readable and maintainable Rust.

The target system is a bare-metal, hard real-time computer with very limited memory and CPU. There is no operating system — task scheduling is part of the navigation software itself.

**System constraints** — the Rust implementation reflects the original AGC: hard real-time scheduling (no OS; the software owns the scheduler), minimal memory footprint, robust error recovery (always return to a safe state). Inputs: stellar positions, inertial platform (orientation + acceleration). Outputs: thruster control (attitude) and main engine control (velocity). Crew interface: a DSKY-style console for invoking programs.

**Fidelity principle**: where Rust idiom and AGC fidelity conflict, fidelity wins. Navigation errors kill people.

## Agent Workflow

Work proceeds through a pipeline of specialized agents in `.claude/agents/`:
**analyst-reengineer** (AGC source → functional specs) → **architect** (specs → Rust architecture) → **developer** (specs + architecture → Rust) → **tester** (tests). Each agent reads the prior stage's output; the analyst's specs are the primary input to architect and developer.

Full roles, hand-off flow, agent-selection triggers, and parallelism rules: **`docs/workflow.md`**.

## Key Reference Material

- `docs/AGC Symbolic Listing.md` — markdown conversion of the formal AGC hardware/software specification (Block 2 AGC, Comanche/Colossus 2D for Apollo 13)
- [Apollo-11 source on GitHub](https://github.com/chrislgarry/Apollo-11) — digitized AGC assembler source (Comanche055 = Command Module)
- [AGC Assembly Language Manual](https://www.ibiblio.org/apollo/assembly_language_manual.html) — machine, interpreter, and pseudocode instruction descriptions

## Coding Rules

The Rust coding rules, embedded/no_std constraints, AGC cross-reference convention, and validation steps live with the agents that apply them, in `.claude/agents/` — each agent definition carries the rules relevant to its role (developer = full set; code-review, debugger, tester, architect = their slice). The orchestration model (roles, hand-offs, parallelism) is in `docs/workflow.md`.

## Workspace

- `agc-core/` — flight software, `#![no_std]`/`#![no_main]`, no heap; bare-metal target `thumbv7em-none-eabihf`
- `agc-sim/` — host-side simulator with `std`; provides the `AgcHardware` simulation impl
- `agc-test/` — integration test harness; VirtualAGC fixtures in `agc-test/fixtures/`
- `docs/architecture.md` (types, module boundaries, HAL, ADRs), `docs/testing.md` (VirtualAGC strategy), `docs/optimization.md` (embedded compliance), `specs/` (per-module specs)
