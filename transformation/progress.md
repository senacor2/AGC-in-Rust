# Transformation Progress

**Last Updated**: 2026-07-08

This file tracks the coarse milestone status of the port. For the detailed
feature-by-feature inventory against the Comanche055 rope, see the latest
status report (`transformation/status-report-2026-07-08.md`). Task-level
tracking lives in GitHub issues per `CLAUDE.md`.

## Foundation (Complete)

- [x] Architecture designed — `docs/architecture.md`
- [x] Testing strategy defined — `docs/testing.md`
- [x] Rust Embedded Book compliance analysis — `docs/optimization.md`
- [x] All ADRs documented — `transformation/decisions.md`
- [x] Agent workflow defined — `CLAUDE.md`, `.claude/agents/`
- [x] Project licensed GPL-3.0-or-later (#183)

## Build phase (Milestones 1–5) — Complete

The original build-out milestones are all done. The host workspace builds
clean, `cargo clippy` is warning-free, and the full pure-Rust test suite
passes.

| Milestone | Theme | Status |
|---|---|---|
| M1 | Core infrastructure (types, HAL, Executive, Waitlist, Restart, alarm, fresh-start) | **Complete** |
| M2 | Navigation foundation (linalg, trig, state vector, gravity, integration, SERVICER) | **Complete** |
| M3 | Guidance & DAP (kepler, lambert, conics, DAP, attitude, TVC, RCS, targeting) | **Complete** |
| M4 | Programs (P00–P67 on the Apollo-8 path) | **Complete** |
| M5 | DSKY & crew interface (PINBALL verb/noun, display, agc-sim DSKY) | **Complete** |

Notable items that were "Not Started" in the 2026-04-09 snapshot and are now
done: full Executive/Waitlist implementation, `navigation/{gravity,integration}`,
`services/average_g` (SERVICER), the entire guidance/DAP stack, all Apollo-8-path
P-codes, and the PINBALL verb/noun processor with an interactive agc-sim DSKY.

## Apollo-8 mission milestones (`milestone-plan-2026-06-07.md`) — Complete

All four milestones of the Apollo-8 plan closed between 2026-06-07 and 2026-07-06.

| Milestone | Theme | Status | Issues |
|---|---|---|---|
| **M-A** | Finish partial (🟡) items on the Apollo-8 critical path | **Complete** | #120 → #124–#128, #146 |
| **M-B** | Plug remaining ❌ Apollo-8 narrative gaps | **Complete** | #121 → #130–#133, #141 (#129 closed *not applicable*) |
| **M-C** | Fidelity, hardening, validation | **Complete** | #122 → #134–#136 |
| **M-D** | agc-sim status & warning lights wiring | **Complete** | #123 → #137–#140 |

Cross-cutting since 2026-06-05: alarm-code centralization + reconcile (#115, #182),
spec writing/audit (#153, #158–#162), glossary + agc-sim README (#74, #60),
REFSMMAT half-scale encoding (#185), and the interactive sextant/optics-alignment
thread (#109, #176, #174, #175, #177, #178).

**Result:** the port is feature-complete for the Apollo-8 mission narrative
(Earth orbit → TLI → cislunar → LOI → lunar orbit → TEI → entry → splash).

## Open work

| Item | Issue | Status |
|---|---|---|
| VirtualAGC entry co-simulation validation | #49 | **Open (deferred)** — pure-Rust entry chain green; two yaAGC closed-loop entry co-sim tests still fail on clock-decoupling / boot-state fidelity |
| Bare-metal board port (Nucleo-F767 / Pico bridge) | #99, #95 | Deferred |
| Higher-fidelity sextant (trunnion limit + P52 acquisition maneuver) | #201 | Open |
| Simulator handbook | #155 | Open |
| TEMP lamp wiring | #180 | Open (needs temperature HAL) |
| LM rendezvous, P12/P53/P76, hardware/self-test DSKY surface | — | Out of Apollo-8 scope (long-term backlog) |

## Metrics

| | Count |
|---|---|
| Rust source files (excl. target) | 178 (89 in `agc-core/src`) |
| Tests passing | 976 |
| Tests failing | 2 (`entry_e2e_vagc` closed-loop co-sim — tracked under #49) |
| Spec documents (`specs/`) | 64 |
| Clippy warnings | 0 |
| VirtualAGC fixture files | 28 (`agc-test/fixtures`) |
