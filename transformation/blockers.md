# Blockers and Open Questions

## Active Blockers

_None_

## Open Questions

### Q1: MCU Target Selection (ADR-011)

**Question**: Which specific MCU should be the primary bare-metal target?

**Options**:
1. **STM32F405** (Cortex-M4F, 168 MHz, 192 KB RAM, 1 MB flash) — minimum viable, widely available, good `probe-rs` support
2. **STM32F7** (Cortex-M7, 216 MHz, 512 KB RAM, 1 MB flash) — preferred for navigation integration cycle headroom
3. **RP2040** — dual-core, but no hardware FPU on M0+ cores; **not acceptable** (ADR-003 requires FPU)

**Blocking**: Final linker script, PAC crate selection, interrupt name mapping.

**Resolution**: Create ADR-011. STM32F405 recommended as minimum; STM32F746 recommended if timing budget is tight during hardware-in-the-loop testing.

---

### Q2: RTIC vs Hand-Rolled Executive (ADR-012)

**Question**: Should the interrupt scheduling layer use RTIC or the hand-rolled Executive + Waitlist?

**Context**: The Rust Embedded Book recommends RTIC for systems with static priorities and shared resources — exactly what the AGC models. RTIC would replace the hand-rolled critical-section management and provide compile-time deadlock-freedom guarantees. The nav/guidance code in `agc-core` would be unchanged.

**Trade-off**:
- **RTIC**: Compile-time safety, no hand-rolled `interrupt::free`, message passing built-in, active maintenance
- **Hand-rolled**: Exact AGC Executive semantics, full control, no framework dependency, more educational value for the methodology presentation

**Blocking**: Architecture of `executive/scheduler.rs` — the implementation differs significantly between the two approaches.

**Resolution**: Create ADR-012. Decision needed before Milestone 1 implementation starts.
