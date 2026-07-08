# AGC-in-Rust: Status / Gap Report

**Date:** 2026-07-08
**Supersedes:** `transformation/status-report-2026-06-05.md`
**Purpose:** Refresh the 2026-06-05 inventory of the Rust port against the Comanche055 (Apollo 11 CM) reference rope, recording what has been closed under the Apollo-8 milestone plan (`milestone-plan-2026-06-07.md`) and what remains.
**Methodology:** Same as the 2026-06-05 report — read the Rust sources under `agc-core/src/{programs,services,control,guidance}/`, cross-referenced against the Comanche055 assembler tree. Status deltas verified against merged PRs (#142–#202) and the GitHub issue tracker.

---

## 0. What changed since 2026-06-05

The 2026-06-05 report produced the Apollo-8 milestone plan (2026-06-07), which filtered the raw Comanche055 gap list down to the items on the Apollo-8 critical path (Earth orbit → TLI → cislunar → LOI → lunar orbit → TEI → entry → splash; no LM, no landing, bare-metal deferred). **All four milestones (M-A, M-B, M-C, M-D) of that plan are now closed.** Highlights:

**Milestone A — finish partial (🟡) items on the critical path** (parent #120):
- **#124 (M-A.1)** — P62 now fires the CM/SM separation pyro through the `Secs` HAL trait; the scenario runner pumps SECS staging into `SimSecs`. P62 is no longer partial.
- **#125 (M-A.2)** — Entry display nouns **N63, N64, N66, N67, N68** wired into `noun_display` (RTGO/VIO, drag/Vmagi, roll command/cross-range, range-to-target, Rdot).
- **#126 (M-A.3)** — Crew-accessible verb dispatch for **V36, V46, V69, V93, V96** — capabilities that already existed but were not reachable by verb number.
- **#127 (M-A.4)** — **V82 / R30** orbital-parameter display wired, including **TFF** (time-to-free-fall at 300 kft).
- **#128 (M-A.5)** — **N43** lat/lon/alt now routed through `geodetic_from_inertial` in the noun pipeline (no more R-reg side-channel).
- **#146** — Fixed N44 apsis display to dispatch on the state vector's frame.

**Milestone B — remaining ❌ Apollo-8 narrative gaps** (parent #121):
- **#130 (M-B.2)** — **V94** cislunar attitude-maneuver dispatch.
- **#131 (M-B.3)** — **V32** recycle and **V33** proceed-without-DSKY-input.
- **#132 (M-B.4)** — GIMBAL_LOCK_AVOIDANCE wired into the DAP (roll-away when critical).
- **#133 (M-B.5)** — **POODOO / GOTOPOOH** deterministic restart-recovery chain.
- **#141 (M-B.6)** — **N08 / N09** diagnostic alarm nouns; `AlarmState` extended with a 3-deep code FIFO, call-site tag, and `ercount`.
- **#129 (M-B.1, P70 TLI targeting) — closed as *not applicable*.** P70 does not exist in Comanche055: TLI was always ground-computed and loaded via P30 (external ΔV) with P15 as the passive monitor. Adding on-board TLI targeting would extend the AGC beyond its historical scope, so it is out of scope rather than a gap. The `phase_tli` integration test models the S-IVB burn as an impulsive reseed, consistent with the hardware boundary.

**Milestone C — fidelity / hardening** (parent #122):
- **#134 (M-C.1)** — KALCMANU optimal-axis attitude steering replaces the simplified P/I path in the DAP.
- **#135 (M-C.2)** — DOWN-TELEMETRY MSFN format: `CMCSTADL` encoded into the AGC one's-complement word stream, with simulator file output.
- **#136 (M-C.3)** — **P02** proper gyrocompass loop against a software Earth-rate provider (`SimEarthRate`). P02 is no longer partial.

**Milestone D — agc-sim status & warning lights** (parent #123):
- **#137–#140** — `services::lamps::refresh_lamps` driver pass on T4RUPT; `SimDsky::set_lamp` records events for tests; caution/warning colour split + alarm-code footer in `dsky_ui`; DSKY lamp snapshot tests; V35 lamp-test auto-revert.

**Cross-cutting infrastructure & documentation (not in the original plan):**
- **#115 / #182** — All alarm codes centralised into `tables/alarm_codes.rs` and reconciled against the AGC Quick Reference (invented codes given a reserved range).
- **#183** — Project placed under **GPL-3.0-or-later**; CLAUDE.md updated so agents check dependency licence compatibility and add SPDX headers.
- **#153 / #158 / #159 / #160 / #161 / #162** — Missing specs written (P01/P02, P06, P15, P29, P47; guidance/math; navigation; services) and the whole spec set audited against the implementation (3-part audit).
- **#60 / #74** — agc-sim README; project glossary (`docs/glossary.md`).
- **Optics / sextant UI thread:** **#109** sextant UI concept → **#176** interactive sextant panel in `dsky_sim` (reticle + slew + MARK) → **#174/#175/#177** keystroke MARK path + P51/P52 dispatch by `major_mode` → **#178** fine-alignment walkthrough doc and demo slew tuning (with modelling-limitations note). This is the first crew-driven, end-to-end IMU-alignment path in the simulator.
- **#185** — REFSMMAT now encoded at AGC half-scale (B−1), matching `reference_state_vector_scaling`.

**Entry / VirtualAGC validation (#49 — still open, deferred):**
Sustained work closed the **P62 → P63 wake gap**: entry-aligned REFSMMAT to clear the P62 S61.1 IMU alarm, a validated EXDAP gate (`|CALFA| ≥ cos 45° AND CALFA > 0`), and closed-loop CDU injection driving heat-shield-forward attitude (X_body = −unit(V)). The pure-Rust entry chain is green and `tc_e7i_j` passes live; `tc_e7e` asserts full P62→P63→P64 program progression. **What remains open under #49** is the yaAGC co-simulation closed-loop entry tests (`tc_e7e_vagc_entry_direct_leo_closed_loop`, `tc_e7e_vagc_entry_lunar_return_closed_loop`), which still fail — the clock-decoupling / boot-state fidelity between the Rust core and the yaAGC reference is the outstanding item. See `project_p62_wake_gap_root_cause`.

**Test status (2026-07-08):** The pure-Rust workspace suite passes (agc-core 688 unit tests, agc-sim 137, agc-test 82, plus platform/quat/frame/downlink/entry integration suites). The only failures are the two `entry_e2e_vagc` closed-loop co-simulation cases above, which are the tracked open item of the deferred #49.

---

## 1. Executive Summary

Since 2026-06-05 the port has moved from "≈50–55 % of the CM-relevant program space with numerous crew-visible plumbing gaps" to **feature-complete for the Apollo-8 mission narrative**. Every 🟡 (partial) row of the 2026-06-05 report that lay on the Apollo-8 path has been closed, and the small set of ❌ Apollo-8-essential gaps has been plugged. The remaining ❌ items are all either LM-side rendezvous, lunar-landing, or bare-metal-hardware paths that were intentionally excluded from the Apollo-8 scope.

The DSKY layer is now materially more complete: the entry-guidance nouns (N63–N68), the alarm-diagnostic nouns (N08/N09), the orbital-parameter display (V82/R30/N44 with TFF), and six previously-orphaned verbs (V32, V33, V36, V46, V69, V93, V94, V96) are all crew-accessible. Every lamp in the `Lamps` struct is now driven by a real internal condition and covered by snapshot tests.

The two big remaining engineering frontiers are unchanged in kind but sharpened: (a) **VirtualAGC co-simulation fidelity** (#49) — the Rust core and yaAGC must agree closely enough through the entry sequence for the co-sim tests to pass; and (b) the **bare-metal board port** (#99, #95), still deferred.

---

## 2. Programs (P00 – P79+)

Key: ✅ Implemented · 🟡 Partial · ❌ Gap · ⚪ Out of scope (intentional)
Only rows whose status changed since 2026-06-05 are annotated **[Δ]**.

| P# | Comanche055 purpose | Status | Notes |
|----|---------------------|--------|-------|
| P00 | CMC Idle | ✅ | `programs/p00.rs` |
| P01 | Pre-launch IMU init (cage) | ✅ | `programs/p01_p02.rs` |
| P02 | Gyrocompassing | ✅ **[Δ]** | #136: real gyrocompass loop against `SimEarthRate` software provider; was 🟡 |
| P06 | CMC Power-down | ✅ | `programs/p06.rs` |
| P07 | IMU performance test | ⚪ | Self-check hardware (G6) |
| P08 | Gyro torquing | ⚪ | IMU hardware path (G5) |
| P11 | Earth orbit insertion monitor | ✅ | `programs/p11.rs` |
| P12 | Powered flight monitor | ❌ | Not ported; shares a block with P11 (not on Apollo-8 path) |
| P15 | TLI monitor | ✅ | `programs/p15.rs` |
| P20 | Rendezvous navigation | ✅ | `programs/p20.rs`; scalar Kalman |
| P21 | Ground-track determination | ✅ | `programs/p21.rs` |
| P22 | Orbital navigation (landmark) | ✅ | `programs/p22.rs` |
| P23 | Cislunar midcourse navigation | ✅ | `programs/p23.rs` |
| P27 | Update liaison (V70–V73) | ✅ | `services/v_n.rs` |
| P29 | Time-of-longitude | ✅ | `programs/p29.rs` |
| P30 | External ΔV targeting | ✅ | `programs/p30.rs` |
| P31 | Lambert aim-point (CSI) | ✅ | `programs/p31.rs` |
| P32 | CDH targeting | ✅ | `programs/p32.rs` |
| P33 | TPI targeting | ✅ | `programs/p33.rs` |
| P34 | TPM midcourse | ✅ | `programs/p34.rs` |
| P35 | TPF final approach | ❌ | LM-active rendezvous; excluded from Apollo-8 scope |
| P37 | Return to Earth (TEI) | ✅ | `programs/p37.rs` |
| P38 / P39 | Stable-orbit rendezvous | ❌ | LM rendezvous; excluded |
| P40 | SPS thrusting | ✅ | `programs/p40_p41.rs` |
| P41 | RCS thrusting | ✅ | `programs/p40_p41.rs` |
| P47 | Thrust monitor | ✅ | `programs/p47.rs` |
| P51 | IMU orientation determination | ✅ **[Δ]** | `programs/p51_p52.rs`; now crew-drivable via keystroke MARK (#174/#175/#177) |
| P52 | IMU realignment | ✅ **[Δ]** | dispatched by `major_mode`; interactive MARK loop |
| P53 | External ΔV determination | ❌ | Post-LM-sep drift; excluded |
| P61 | Entry preparation | ✅ | `programs/p61_p67.rs` |
| P62 | CM/SM separation | ✅ **[Δ]** | #124: fires sep pyro via `Secs` HAL; was 🟡 (only gap in the entry chain) |
| P63 | Pre-0.05g monitoring | ✅ | `entry_servicer_exit` hook |
| P64 | Closed-loop entry guidance | ✅ | `guidance/entry.rs`; 111 km miss on lunar return |
| P65 | Up-control / skip-out | ✅ | `upcontrol_step` |
| P66 | Ballistic hold | ✅ | `ballistic_step` |
| P67 | Final phase / drogue | ✅ | `final_phase_step`; Sutton–Graves heating |
| P70 | TLI targeting | ⚪ **[Δ]** | #129 closed *not applicable* — not a real Comanche055 program; TLI is ground-computed via P30 |
| P72–P75, P78 | LM-active rendezvous | ⚪ | LM-side (G1) |
| P76 | Target ΔV | ❌ | LM state ΔV event; excluded from Apollo-8 |

**Programs summary:** All Apollo-8-path programs are ✅. The 2026-06-05 partials (P02, P62) are closed. Remaining ❌ (P12, P35, P38, P39, P53, P76) and P70-as-⚪ are all outside the Apollo-8 scope filter.

---

## 3. Routines (R00 – R7x) & embedded service routines

Changes since 2026-06-05:

| Routine | Prev | Now | Notes |
|---------|------|-----|-------|
| R30 (orbit param display, V82→N44) | 🟡 | ✅ **[Δ]** | #127: TFF computed; V82 dispatched |
| R52 / auto-optics (P51/P52 sextant) | 🟡 | 🟡→✅(crew path) **[Δ]** | #174–#178: keystroke MARK + interactive sextant panel; auto-CDU slew still modelled by manual slew in sim |
| R60 / KALCMANU steering | 🟡 | ✅ **[Δ]** | #134: optimal-axis steering replaces simplified P/I |
| GIMBAL_LOCK_AVOIDANCE | 🟡 | ✅ **[Δ]** | #132: avoidance maneuver wired to DAP |
| DOWN-TELEMETRY | 🟡 | ✅ **[Δ]** | #135: MSFN one's-complement encoding |
| ALARM_AND_ABORT (POODOO/GOTOPOOH) | 🟡 | ✅ **[Δ]** | #133: deterministic restart-recovery chain |
| Lamp driver (`services/lamps.rs`) | — (absent) | ✅ **[Δ]** | #137: `refresh_lamps` T4RUPT pass drives every lamp |

Still 🟡 / ❌ and intentionally so: R21/R22/R23/R31/R34/R36/R63 (rendezvous DSKY), R35 (lunar landmark), R02 (hardware IMU fault path), R62 (V49 crew maneuver) — all outside Apollo-8 scope.

---

## 4. Verbs — changes since 2026-06-05

| V# | Prev | Now | Notes |
|----|------|-----|-------|
| V32 Recycle | ❌ | ✅ **[Δ]** | #131 |
| V33 Proceed w/o DSKY | ❌ | ✅ **[Δ]** | #131 |
| V36 Fresh start | 🟡 | ✅ **[Δ]** | #126 dispatch arm |
| V46 Establish G+C | 🟡 | ✅ **[Δ]** | #126 |
| V69 Cause restart | 🟡 | ✅ **[Δ]** | #126 |
| V82 Orbit param display | 🟡 | ✅ **[Δ]** | #127 (+ TFF) |
| V93 Enable W-matrix init | 🟡 | ✅ **[Δ]** | #126 |
| V94 Cislunar attitude maneuver | ❌ | ✅ **[Δ]** | #130 |
| V96 Terminate integration → P00 | 🟡 | ✅ **[Δ]** | #126 |
| V51 Please mark (P51/P52) | ❌ | 🟡→✅(crew path) **[Δ]** | #175 keystroke MARK wired for alignment |

Fully wired verb count grew from **13** (2026-06-05) to roughly **21** on the Apollo-8 path. Remaining gaps (V40–V45, V47–V49, V54–V58, V75–V81, V83–V90, V97–V98, etc.) are all hardware-CDU, LM-rendezvous, or self-test verbs excluded by the scope filter.

---

## 5. Nouns — changes since 2026-06-05

| N# | Prev | Now | Notes |
|----|------|-----|-------|
| N08 Alarm data (adres/bbank/ercount) | ❌ | ✅ **[Δ]** | #141 |
| N09 Alarm codes (3-deep FIFO) | ❌ | ✅ **[Δ]** | #141 |
| N43 Lat/lon/alt | 🟡 (stub) | ✅ **[Δ]** | #128 via `geodetic_from_inertial` |
| N44 Apogee/perigee/TFF | 🟡 | ✅ **[Δ]** | #127 TFF + #146 frame-correct apsis |
| N63 RTGO/VIO/TFE | 🟡 | ✅ **[Δ]** | #125 |
| N64 Drag/Vmagi/range-to-splash | 🟡 | ✅ **[Δ]** | #125 |
| N66 Roll cmd/cross-range/down-range | 🟡 | ✅ **[Δ]** | #125 |
| N67 Range-to-target/lat/lon | 🟡 | ✅ **[Δ]** | #125 |
| N68 Roll cmd/Vmagi/Rdot | 🟡 | ✅ **[Δ]** | #125 |

Wired-noun count grew from ≈23 to ≈33 on the Apollo-8 path. The full entry-guidance display chain (N63–N68) and the alarm-diagnostic pair (N08/N09) are now complete. Remaining ❌ nouns (N45, N55–N59, N73, N75, N82, N84–N88, N99, etc.) are rendezvous / engineering / LM nouns outside the Apollo-8 scope.

---

## 6. agc-sim status & warning lights (Milestone D — new since 2026-06-05)

Previously only 3 of 10 lamps were ever written. Now (#137–#140) every lamp boolean is driven from a single source of truth in `services::lamps::refresh_lamps` on the T4RUPT path, `SimDsky::set_lamp` records events for tests, `dsky_ui` distinguishes caution (yellow) from warning (red) with an alarm-code footer, and `tests/dsky_lamps_snapshot.rs` asserts lamp behaviour across scenarios (gimbal-lock march, 1107 alarm, V35 lamp test with auto-revert, sextant mark window, V21 key-rel). Remaining: `temp` lamp is deferred (#180 — no temperature HAL).

---

## 7. Remaining work (post-Apollo-8-milestones)

With M-A through M-D closed, the open backlog is:

1. **#49 — VirtualAGC co-simulation validation (deferred, the primary open engineering item).** The pure-Rust entry chain is green; the two yaAGC closed-loop entry co-sim tests still fail on clock-decoupling / boot-state fidelity. This is the last piece of the P62→P63 wake-gap arc.
2. **#99 / #95 — bare-metal board port.** Finish the Nucleo-F767 / Pico bridge port and shrink the board binary. Still deferred behind the host-side functional work.
3. **#201 — higher-fidelity sextant.** Trunnion travel limit + P52 star-acquisition RCS maneuver (builds on the #174–#178 optics thread).
4. **#155 — simulator handbook.** End-user documentation for driving the agc-sim through the Apollo-8 timeline.
5. **#180 — TEMP lamp** (needs a temperature HAL source).
6. **Long-term backlog (out of Apollo-8 scope):** LM-active rendezvous (P35, P38, P39, P72–P78; R21–R63 rendezvous displays), P12, P53, P76, and the hardware/self-test DSKY surface. These remain deliberately unported.

---

## 8. Intentional-Gap Rationale Catalogue

Unchanged from 2026-06-05 (G1 LM-specific · G2 lunar landing · G3 telemetry infra · G4 pre-launch/pad · G5 hardware-only path · G6 AGC self-check · G7 AGC-internal debug). Note that **G3 is now partially retired**: MSFN downlink encoding shipped in #135, so V74-class telemetry is no longer blocked on the encoding format itself.
