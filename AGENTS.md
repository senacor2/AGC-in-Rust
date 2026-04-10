# AGC-in-Rust: Agent and Contributor Guidelines

## Code Style

- Use stable Rust only. No nightly features.
- Follow existing module and naming patterns before introducing new abstractions.
- Use standard Rust naming conventions: `snake_case` for functions, variables, and modules; `PascalCase` for types and traits; `SCREAMING_SNAKE_CASE` for constants.
- Keep public APIs small and explicit. Prefer structs, enums, and traits with clear ownership semantics over hidden side effects.
- Prefer `&T`, `&mut T`, or owned values in public APIs. Never expose `RefCell<T>`, `UnsafeCell<T>`, or `Mutex<T>` in a public API — these are implementation details of the shared-state layer.
- Prefer concrete types first, then generics, and only `dyn Trait` where callers genuinely require dynamic dispatch.
- Avoid `unwrap` and `expect` in flight-software code. Any failure that cannot be statically ruled out must either be handled or trigger the GOJAM restart path explicitly.
- `Result`-based error handling is for the `agc-sim` host crate. In `agc-core` (no_std, no heap), use `Option` and program alarms (`alarm::raise`) instead.
- Avoid redundant comments. Add comments for Rust-specific nuance, invariants, safety, scaling factors, and non-obvious AGC-to-Rust mapping decisions, not for code that is self-evident.

## Embedded / no_std Rules

These override general Rust style where they conflict.

- **No heap.** `alloc`, `Vec`, `Box`, `String`, `HashMap` are forbidden in `agc-core`. All data structures are statically sized. Violations break the `no_std` build.
- **No `static mut`.** Shared mutable state uses `cortex_m::interrupt::Mutex<RefCell<T>>` (zero heap, zero OS). Access always goes through `cortex_m::interrupt::free(|cs| ...)`. Raw `static mut` is a Clippy error in this codebase.
- **No blocking.** Interrupt handlers and Waitlist tasks must not block, spin-wait, or perform long computations. If the work is too long for a task, establish it as a job via the Executive.
- **No unwinding.** `panic = "abort"` is set in `Cargo.toml`. Every panic triggers GOJAM (hardware restart). Do not rely on `Drop` for cleanup that must run before restart.
- **f64 for all navigation math.** The AGC's ones-complement fixed-point was a hardware constraint, not a navigation requirement. `f64` eliminates the entire class of scale-factor bookkeeping errors. `i16`/`u16` are used only for raw hardware values (CDU angles, PIPA counts, channel words).
- **No interpreter.** Do not implement the AGC interpretive language VM. Every routine that was written in the interpretive language is re-implemented as a plain `f64` Rust function.
- **Restart safety.** Any multi-step computation that must survive a hardware restart must use the phase-table pattern (`state.restart.set_phase(...)`). See `executive/restart.rs`.

## Architecture

- Keep domain logic (navigation, guidance, DAP) separate from the HAL and from the Executive scheduler.
- All mutable state lives in `AgcState` and is passed by `&mut` reference through foreground code. State that is also touched by interrupt handlers is extracted into a dedicated `static Mutex<RefCell<T>>`.
- Interrupt handlers receive the narrowest possible view of state. They access their designated static directly; they do not receive `&mut AgcState`.
- The HAL boundary (`AgcHardware` and its sub-traits) is the only place the flight software touches hardware. No peripheral register access outside `hal/`.
- Bare-metal HAL structs must implement `free()` (C-FREE), implement applicable `embedded-hal` traits (C-HAL-TRAITS), and use typestate type parameters for operational modes (C-PIN-STATE).
- The `#[interrupt]` attribute must be re-exported from the device PAC crate, not from `cortex-m-rt` directly.

## Build and Test

- Validate all changes with `cargo fmt`, `cargo clippy -- -D warnings`, and `cargo test`.
- `agc-core` must always build with `cargo build --target thumbv7em-none-eabihf` (bare-metal, hard-float). A change that breaks the embedded build is not mergeable.
- Unit tests in `agc-core` run on the host (`#[cfg(test)]`) and must not use any `std` feature gated behind the `sim` feature flag.
- Integration tests in `agc-test` use the `agc-sim` hosted HAL. They are the primary place for end-to-end scenario testing.
- Math function tests must include at least one case from a VirtualAGC reference run (see `docs/testing.md`).
- Do not leave `dbg!`, `println!`, or temporary `hprintln!` calls in finished changes.
- Validate all implemented features or tasks with the source code found in https://github.com/chrislgarry/Apollo-11/tree/master/Comanche055 

## Conventions

- Document public modules, types, and functions. Navigation and guidance functions must document: input units and scale, output units and scale, and the corresponding AGC source routine name and file.
- Physical quantity newtypes (`CduAngle`, `Met`, `DeltaV`) must document their unit and scale factor in the struct-level doc comment.
- Any `unsafe` block must be justified in a comment immediately above it. The justification must name the invariant being upheld, not just say "safe here".
- Document non-obvious constants: what they represent, their AGC source, and their units.
- Prefer `#[expect(lint, reason = "...")]` over `#[allow(lint)]` when suppressing Clippy lints.

## AGC Source Cross-References

Every Rust function that implements a specific AGC routine must carry a doc comment cross-reference:

```rust
/// Solve Kepler's equation for the universal variable.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPRTN routine.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3) { ... }
```

## Project Goals

**Scope**: Comanche055 (Command Module), covering Earth-to-Moon-and-back. Lunar landing (Luminary / LM) is out of scope.

**Fidelity principle**: Where Rust idiom and AGC fidelity conflict, fidelity wins. Navigation errors kill people.

**Deliverable**: A `no_std` Rust crate that runs on a Cortex-M4F bare-metal target and produces the same navigation, guidance, and control outputs as Comanche055 for the same inputs.

**Long-term**: Physical DSKY, real IMU, actual embedded target. The last step is just rocket science.

## Development Process

The project uses an **agile, sub-agents with parallel exection workflow** with five roles. Each
role corresponds to a specialised Claude agent in `.claude/agents/` or a skill
in `.claude/skills/`. Roles run concurrently whenever their dependencies allow —
the system is designed so that specification, architecture, development, and
testing overlap rather than execute in strict sequence.

### Roles

| Role | Responsibility | Primary inputs | Primary outputs |
|---|---|---|---|
| **Analyst / Spec** | Read the AGC assembler source (`docs/agc-source/*.agc`) and PDF references, produce a functional specification per component | Comanche055 `.agc` files, AGC book, CIS book | `specs/<module>.md` — API, invariants, scale factors, test cases |
| **Architect** | Decide the Rust shape: module boundaries, types, ownership, interrupt model, restart safety | Functional specs, `docs/architecture.md`, ADRs | Architecture decisions, module layout, trait signatures |
| **Developer (Dev)** | Implement the Rust code that satisfies the spec and the architecture | Spec + architecture + existing code | Rust source files in `agc-core/`, `agc-sim/` |
| **Tester** | Write unit tests, scenario tests, VirtualAGC fixture checks | Spec test cases, implementation | Test files in `agc-core/src/**/tests`, `agc-test/tests/` |
| **PO / Validation** | Cross-check the finished implementation against the AGC source and PDF references | `.claude/skills/validation/SKILL.md`, local AGC source cache | Validation report (CONFIRMED / WRONG / APPROXIMATE / NOT FOUND) |

### Flow

```uml
Analyst/Spec  Arch           Dev              Test             PO/Validation
     │         │              │                │                     │
     │─ Spec ──▶              │                │                     │
     │◀ · · · ·│              │                │                     │
     │         │─ Design ────▶│ + write test ─▶|                     │
     │         │─ Develop ───▶│                │                     │
     │         │              │                │                     │
     │         │              │── Test ───────▶│                     │
     │         │              │◀· · · · · · · ·│                     │
     │         │              │                │                     │
     │         │              │                │-── AGC / PDF ──────▶│
     │         │              │                │                     │
     │◀ · · · · · · · · · · · · · · · · · · · · · · · · · · · · · · ·│
     │· · · · ·▶ (revised spec feeds back in)                        │
```

Solid arrows are **hand-offs** (data flows forward). Dashed arrows are
**feedback loops** (something downstream caused upstream rework).

### Key practices

- **Sub-agents run in parallel.** Multiple developer agents can work on
  independent modules simultaneously (e.g. `math/kepler.rs`, `math/lambert.rs`,
  `navigation/conics.rs`. Dispatch them in a single message with multiple `Agent` tool
  calls so they execute concurrently, not sequentially.
- **Spec ↔ Analyst is bidirectional.** The Architect and the Developer can
  push back to the Analyst when the spec is incomplete or contradictory. The
  Analyst revises the spec and the downstream work re-runs.
- **Test is co-developed with Dev, not after.** The Tester writes test cases
  in parallel with the implementation. Unit tests in the Rust file are the
  Developer's responsibility; scenario and fixture tests belong to the Tester.
- **Validation (PO) is the final gate.** Before a milestone is marked complete,
  run the validation skill against the AGC source. Any `WRONG` item blocks
  merge; `APPROXIMATE` items must be documented as intentional deviations.
- **The AGC assembler source and PDFs are the single source of truth.** When
  in doubt, read `docs/agc-source/<file>.agc` or the AGC book (Frank O'Brien)
  and update `docs/agc-reference-constants.md` with the new finding.
- **Tracking is mandatory.** Every completed task updates
  `transformation/progress.md`, `transformation/tasks.md`, and the relevant
  spec status in `transformation/specifications.md`. Also update the README 
  files in corresponding to the task. 

### When to use which agent

| Trigger | Agent |
|---|---|
| New AGC routine to port | **analyst-reengineer** — read the `.agc` file, write a spec |
| Architectural decision (new module, new trait, shared state strategy) | **architect** — produce an ADR and update `docs/architecture.md` |
| Implement a spec / refactor existing code | **developer** — follow the spec, match existing conventions |
| Failing build, clippy warning, test regression, no_std violation | **debugger** — diagnose and fix |
| Write tests for new or existing code | **tester** — unit, scenario, fixture |
| Review a Rust change for correctness, API design, `no_std` safety | **code-review** — structured review |
| Verify an implementation against the AGC source | **validation** skill — CONFIRMED / WRONG / APPROXIMATE |
| Research a question that spans multiple files | **Explore** — targeted or deep codebase search |
| Design a non-trivial implementation before coding | **Plan** — step-by-step plan |
| Housekeeping: workspace, Cargo, CI, feature flags | **workspace-maintenance** skill |

### Parallelism rules of thumb

1. Independent modules → launch dev agents **in parallel** in one message.
2. Dependent modules → run agents **sequentially**, feeding the output of one
   into the prompt of the next.
3. Research → always prefer parallel exploration (multiple Explore agents
   or one agent with parallel tool calls).
4. Validation of a milestone → run **one validator per sub-system** in
   parallel (e.g. math modules, control modules, guidance modules separately).
5. Never spawn a new agent for a task a single tool call can solve — prefer
   `Read` / `Grep` / `Glob` directly when the target is known.

