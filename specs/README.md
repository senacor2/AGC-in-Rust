# Spec Templates

This directory contains specification templates for systematically transforming Comanche055 (Command Module) AGC assembly into idiomatic Rust.

## Spec-Driven Workflow

1. **Choose the right template** based on what you are transforming
2. **Fill out the spec** — AGC source reference, Rust API design, test cases, scaling factors
3. **Review** — design errors are cheaper to fix in a spec than in code
4. **Implement** — hand the spec to the developer agent
5. **Validate** — run tests, compare against VirtualAGC fixtures
6. **Iterate** — update spec if design issues emerge, then regenerate

## Available Templates

| Template | Use For | Example AGC Components |
|----------|---------|----------------------|
| `routine-spec.md` | Individual subroutines | KEPRTN, LAMBERT, BANKCALL |
| `module-spec.md` | Complete modules | EXECUTIVE, WAITLIST, SERVICER |
| `guidance-algorithm-spec.md` | Guidance P-programs | P40 SPS burn, P61–P67 entry |
| `interrupt-spec.md` | Interrupt handlers | T3RUPT, T4RUPT, T5RUPT, T6RUPT |
| `data-structure-spec.md` | Memory structures, tables | StateVector, phase tables, noun tables |

## AGC Quick Reference (Comanche055)

### Memory
- **Erasable**: 2048 words RAM — `ERASABLE_ASSIGNMENTS.agc`
- **Fixed**: 36864 words ROM in 36 switchable banks
- **Channels**: 12-bit I/O-mapped hardware registers

### Scaling Conventions
All AGC arithmetic is scaled fixed-point. **Always document scale factors in specs.**
- Positions: meters, scale B-28 (1 unit = 2²⁸ m)
- Velocities: m/s, scale B-7 (1 unit = 2⁷ m/s)
- Time: centiseconds, scale B-14
- Angles: half-revolutions or radians
- CDU angles: full revolution = 2¹⁵ counts (B-1 revolutions)

In the Rust port, these are `f64` values in SI units. The spec must document the conversion from AGC fixed-point to `f64` so fixture tests can be written correctly.

### Common Patterns
- **Cooperative multitasking**: EXECUTIVE runs highest-priority ready job; jobs yield voluntarily
- **Deferred tasks**: WAITLIST schedules tasks by delta-time via T3RUPT
- **Restart protection**: PHASE_TABLE_MAINTENANCE checkpoints long calculations
- **Interpretive language**: all eliminated — replaced by plain `f64` Rust functions (ADR-001)

## Spec Quality Checklist

Before handing a spec to the developer agent:

- [ ] AGC source file and line range referenced
- [ ] All erasable variables and their AGC addresses listed
- [ ] Scale factors documented for all fixed-point values
- [ ] Corresponding `f64` SI units documented
- [ ] Input/output preconditions and postconditions stated
- [ ] Edge cases and error handling specified
- [ ] At least 3 test cases with expected values (ideally from VirtualAGC run)
- [ ] Rust API signature designed (types, ownership, lifetimes)
- [ ] Invariants explicitly stated
- [ ] Consistency with `docs/architecture.md` checked (types, module boundaries, HAL usage)

## AGC Source Reference Format

Always include exact source reference:

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc
Routine: KEPRTN
Lines: 234–456
```

## Transformation Strategy

### Bottom-Up (Recommended for infrastructure)
1. Data structures (`types/`, `navigation/state_vector`)
2. Math utilities (`math/linalg`, `math/trig`)
3. Core services (`executive/`, `services/`)
4. Guidance algorithms (`math/kepler`, `math/lambert`, `guidance/`)
5. Interrupt handlers (`hal/interrupts`, `services/t4rupt`)

### Top-Down (Useful for P-codes)
1. Define the `MajorMode` trait API
2. Stub algorithm interfaces
3. Implement with real functions once math foundation exists

## Using Specs with the Developer Agent

```
I have a spec for transforming AGC [routine/module/algorithm] to Rust.
The spec is at specs/[filename].

Please implement it following:
- docs/architecture.md for type and module conventions
- AGENTS.md for coding rules
- No heap, no static mut, f64 for nav math, i16/u16 for hardware values
- Cross-reference the AGC source in doc comments
```

## Related Documentation

- `docs/architecture.md` — Full architecture and type conventions
- `docs/testing.md` — VirtualAGC fixture capture strategy
- `AGENTS.md` — Coding conventions and embedded rules
- `transformation/specifications.md` — Status of all specs
