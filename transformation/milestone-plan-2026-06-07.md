# AGC-in-Rust — Apollo-8 Milestone Plan

**Date:** 2026-06-07
**Author:** planning pass following `transformation/status-report-2026-06-05.md`
**Scope:** Close the gaps identified in the status report that are required for a complete **Apollo-8-style mission** (Earth orbit → TLI → cislunar → LOI → lunar orbit → TEI → entry → splash). **No LM. No lunar landing. Bare-metal port deferred.**
**Ordering principle:** items already partly implemented (🟡 in the status report) come before items that are missing completely (❌). Within each milestone, items are ordered by mission-narrative dependency.

---

## 1. Scope filter applied to the status report

The status report mixes Comanche055 (Apollo 11 CM) gaps with items that have no operational meaning for Apollo 8. This plan removes the following classes entirely (they stay as long-term backlog):

| Excluded class | Status-report items | Reason |
|---|---|---|
| LM-active rendezvous | P32–P35 (already ported but unused in A8), P38, P39, P72–P78 | Apollo 8 carried no LM |
| Rendezvous DSKY support | R21, R22, R23, R31, R34, R36, R63; V47, V51, V53–V58, V66, V75–V77, V79–V90, V94 (rendezvous half), V97–V98 | Same |
| Post-sep IMU drift | P53 | Only meaningful after LM jettison |
| Bare-metal-only hardware paths | R02; V40–V45, V59, V64, V87, V88; lamp `temp`, noun N51; FRESH START via channel-11 caution-reset | Deferred until the bare-metal port resumes |
| AGC self-test / engineering diagnostics | V07, V17, V27, V30, V31, V68, V91, V92; P07, P08; N01–N03, N91–N98 | No software analogue |
| Pre-launch pad-specific | V78, N29 | Out of mission scope |
| Telemetry encoding format | V74 erasable dump (downlink raw bytes) | Deferred (covered separately in M-C) |

After filtering, the remaining gap list — sorted by current implementation depth — is what the milestones below close.

---

## 2. Milestone overview

| Milestone | Theme | Predominant gap class | Branchable? |
|---|---|---|---|
| **M-A** Finish partial items on the Apollo-8 critical path | 🟡 items already implemented but not wired | crew-visible | yes, per item |
| **M-B** Plug the few remaining ❌ gaps that block the Apollo-8 narrative | ❌ Apollo-8-essential | mission | yes |
| **M-C** Fidelity, hardening, and validation | ❌ depth/quality | engineering | yes |
| **M-D** agc-sim status & warning lights wiring | new cross-cut | crew-visible | single branch |

Each milestone item maps to (or will create) a GitHub issue per `CLAUDE.md`. Item IDs `M-x.n` are placeholders; replace with the issue number when filed.

---

## 3. Milestone A — Finish partial items on the Apollo-8 critical path

All items in this milestone correspond to a 🟡 row in the status report: capability is already implemented in `agc-core`, but the crew-visible plumbing (verb dispatch, noun-display arm, or HAL effector) is missing. These have the highest value-per-LOC.

### M-A.1 P62 SM-separation pyro HAL command  *(already on `feature/62-sm-sep-pyro`)*
- Status-report row: P62 🟡 (§2)
- `programs/p61_p67.rs` does the phase transition + `dap_stop` but does not fire SM separation.
- Work: extend the `Secs` HAL trait (it already has `fire_csm_separation` in `agc-sim/src/hardware.rs:209`) so `agc-core` invokes it from the P62 entry path. Wire SimHardware's `csm_separation_fired` into the scenario telemetry.
- Acceptance: P62 unit test asserts `fire_csm_separation` is called exactly once; end-to-end entry scenario shows the SM-sep event on the timeline.

### M-A.2 Entry display nouns N63, N64, N66, N67, N68 *(noun_display arms only)*
- Status-report rows: §5, all five marked 🟡 with the note "data present in `entry.*` — display arm missing".
- All underlying floats are already computed each SERVICER cycle in `programs/p61_p67.rs`.
- Work: add five arms to `services/v_n.rs:noun_display`, format-tested against the AGC scaling tables in `input/AGC Quick Reference.md` (range km × 0.1, drag in g, angles in deg).
- Acceptance: per-noun snapshot test of `decode_dsky` after seeding `entry.*` fields.

### M-A.3 Crew-accessible verb dispatch for capabilities that already exist
Status report §4 lists six 🟡 verbs where "capability is present but not crew-accessible by that verb number." Apollo-8-relevant subset:

| Verb | Backing capability already in tree | Dispatch site |
|---|---|---|
| V36 | `services/fresh_start.rs:fresh_start` | add to `verb_takes_no_noun` + `dispatch_verb_noun` |
| V46 | `services/average_g.rs:start_servicer` | dispatch arm |
| V69 | `services/fresh_start.rs` restart dispatcher | dispatch arm |
| V93 | `programs/p22.rs:p22_rectify_w_matrix` | dispatch arm; also rectify P20/P23 W-matrix |
| V96 | alias to `v34_terminate` | dispatch arm |

- Work: each is a small `match` arm in `services/v_n.rs:dispatch_verb_noun`.
- Acceptance: one DSKY scenario test per verb covering the key sequence and the resulting state change.

### M-A.4 V82 / R30 — orbital-parameter display with TFF and DELRSPL
- Status-report rows: V82 🟡, R30 🟡, N44 🟡.
- Apogee/perigee/half-period already computed for N44.
- Work:
  1. Add **TFF** (time-to-free-fall at 300 kft) — a one-dimensional Kepler propagation from the current state to the EI altitude on the descending arc. Reuse `navigation::conics` for the time-to-radius solver.
  2. Add **DELRSPL** (predicted splash-point miss) only after entry guidance is committed (N50 / part of entry-phase status), but the R30 hook should already expose the slot.
  3. Wire V82 into `dispatch_verb_noun` so it pages directly to N44 (the same code path as V16N44, with one-shot display).
- Acceptance: unit test against an analytic conic for TFF; V82 scenario test prints N44.

### M-A.5 N43 lat/lon/alt — populate from `geodetic_from_inertial`
- Status-report row: N43 🟡 — `noun_display` currently returns `(0, 0, 0)` while `programs/p21.rs` writes the R-registers directly.
- Work: add a `noun_display` arm for N43 that converts `state.csm_state` via `navigation::frames::geodetic_from_inertial` + `gmst`, returning lat (deg), lon (deg), alt (km). Then strip the inline R-reg writes from `p21.rs`.
- Acceptance: P21 scenario test reads N43 through `decode_dsky` (no R-reg side-channel).

**Milestone A exit criterion:** every 🟡 row of the status report that lies on the Apollo-8 path is closed. P02 and TVCGEN3FILTERS remain 🟡 (Earth-rate HAL is bare-metal-coupled — deferred with the bare-metal port).

---

## 4. Milestone B — Plug the remaining Apollo-8 narrative gaps

These are ❌ rows in the status report, but each one breaks the Apollo-8 narrative.

### M-B.1 P70 — TLI targeting
- Status-report row: P70 ❌ (§2 item 1 in §8 "Suggested priority").
- Currently TLI must be hand-loaded as a P30 external ΔV; P70 should compute the burn from the parking orbit. Apollo 8 used P15 for the burn monitor on top of S-IVB; here P70 computes the targeting and hands the result to P30/P15.
- Work: port `P37,P70.agc`. Reuse `math/lambert.rs` for the patch-conic intercept of the Earth-departure asymptote toward the lunar SOI rendezvous point. Outputs `pending_maneuver`.
- Acceptance: regression — TEI-demo-style scenario at TLI, comparing burn ΔV magnitude to a JPL Horizons-derived target within 0.5 %.

### M-B.2 V94 — cislunar attitude-maneuver dispatch
- Status-report row: V94 ❌.
- P23 marks are already implemented; what is missing is the crew verb that orients the spacecraft so the sextant can acquire the next star-horizon pair.
- Work: dispatch V94 → `attitude::auto_maneuver_to(preferred_attitude_from_p23_target)`. Uses the existing DAP and the (still simplified) attitude-error path.
- Acceptance: scenario test where V94 issued mid-P23 finds the spacecraft pointing within 1° of the commanded attitude after a maneuver settling time.

### M-B.3 V32 recycle & V33 proceed-without-DSKY-input
- Status-report row: V32 ❌, V33 ❌.
- V33 is the non-V50 PRO equivalent used by P20/P22/P23 for phase advancement; V32 re-enters the current major mode. Both are tiny dispatch arms.
- Work: `services/v_n.rs` dispatch arms with a `try_recycle()` helper on the program-table entry.
- Acceptance: P22 scenario test — PRO at an idle prompt advances to next mark; V32 at the same prompt re-enters the program.

### M-B.4 GIMBAL_LOCK_AVOIDANCE → DAP wiring
- Status-report row: §3 — warning/critical thresholds are checked in `control/imu_control.rs` but the automated avoidance maneuver does not fire.
- Work: when `is_gimbal_lock_critical` returns true while a maneuver is active, DAP must command a roll-away maneuver around the spacecraft long axis (per Frank O'Brien §15.5). This is small (one branch in DAP) but operationally important for cislunar PTC.
- Acceptance: synthetic IMU scenario marches gimbals into the critical band; DAP issues the roll-away command; lamp `gimbal_lock` lights (via M-D, see below).

### M-B.5 POODOO / GOTOPOOH restart-recovery chain
- Status-report row: ALARM_AND_ABORT — "POODOO/GOTOPOOH restart recovery chain not fully wired" (§3).
- Apollo 8 saw program alarms during the cislunar phase; the recovery had to be deterministic.
- Work: a `services/alarm.rs:poodoo()` that aborts the active program, raises 1410-class alarm, schedules `fresh_start::partial_restart` and goes to P00; `gotopooh()` is the milder variant.
- Acceptance: unit test — induced alarm in P22 ends in P00 with restart group consistent with the restart spec.

### M-B.6 N08 alarm data and N09 alarm codes — diagnostic nouns
- Status-report rows: N08 ❌, N09 ❌ (§5). Both are octal-display diagnostic nouns crewed via `V05N08` / `V05N09` (and `V16` for snapshot).
- Together with M-B.5 these are the crew-visible side of the alarm chain. Apollo 8 saw real PROG alarms in cislunar; without N09 the crew cannot read out which codes fired, and without N08 ground cannot tell *where* the alarm was raised.
- Work:
  1. **`AlarmState` extension.** Today `services/alarm.rs` keeps `code` (most recent) and `code2` (previous). Add a third slot `code1` ("first" / oldest in the FIFO window) and shift on `raise`. Add `adres: u16` (call-site / module id), `bbank: u16` (kept 0 in this port — there are no banks; reserved for fidelity), and `ercount: u16` (alarm/restart counter, incremented by both `raise` and `services::fresh_start::restart`).
  2. **N09 display arm.** Add `noun_display` arm returning `(code1, code2, code)` as three octal registers (all-octal noun, no scaling).
  3. **N08 display arm.** Add `noun_display` arm returning `(adres, bbank, ercount)` as three octal registers.
  4. **Capture call-site at raise time.** Extend `AlarmState::raise(code, adres)` so each call passes a small `u16` site tag (one tag per program/service that raises alarms — e.g. `P22_MEAS`, `P40_BURN`, `AVG_G`, `UPLINK`). Tags live alongside the existing alarm-code constants in `tables/alarm_codes.rs` (per `feedback_alarm_codes.md`).
- Acceptance:
  - Unit test on `AlarmState`: three successive `raise` calls leave `code1`, `code2`, `code` populated with the oldest, middle, newest values and `ercount == 3`.
  - DSKY scenario: induce an `EXEC_OVERFLOW` (1202), then key `V05N09` — R1/R2/R3 show `00000 / 00000 / 01202` (octal). Issue a second alarm → R3 shows the new code, R2 shows 1202, R1 still 00000.
  - DSKY scenario: induce an alarm from inside P22, key `V05N08` — `adres` shows the P22 site tag, `ercount` matches the alarm/restart count.

---

## 5. Milestone C — Fidelity, hardening, validation

Lower priority but still Apollo-8-relevant.

### M-C.1 KALCMANU optimal-attitude steering
- Status-report rows: R60 🟡, KALCMANU 🟡.
- Replace the current simplified P/I steering in `control/dap.rs` with the proper KALCMANU quaternion-path solver. Required for accurate PTC settle times and for the V49 verb (when LM-side rendezvous is restored later).
- Acceptance: comparison against the analytic eigenaxis maneuver for several body-rate / inertia tensor combinations.

### M-C.2 DOWN-TELEMETRY MSFN format
- Status-report row: DOWN-TELEMETRY — "Downlink list structure present; real MSFN-format telemetry encoding not implemented."
- Required if we want the agc-sim to publish a true Apollo-8 downlink stream (post-mission validation against historical track files).
- Acceptance: capture downlink frames, decode with a reference parser, compare against a Comanche055 downlink fixture in `agc-test`.

### M-C.3 P02 — proper gyrocompass loop *(without bare-metal Earth-rate HAL)*
- Status-report row: P02 🟡.
- Implement the integration loop against a software-only Earth-rate model from `physics/`. The hardware coupling is replaced by a `SimEarthRate` provider in `agc-sim/src/sensors.rs`. When the bare-metal port resumes, the same trait is implemented against the real Earth-rate HAL.
- Acceptance: convergence test — starting from a 30° initial misalignment, P02 settles within the documented tolerance.

---

## 6. Milestone D — agc-sim status and warning lights wiring  *(new)*

Today only three of the ten lamps in `agc_core::services::pinball::Lamps` are ever written:
`stby` (P06), `uplink_activity` (uplink service) and `opr_err` (parse error + uplink overrun); `restart_flag` is set by `fresh_start` but never cleared. The remaining six (`no_att`, `key_rel`, `gimbal_lock`, `temp`, `prog_alarm`, `comp_acty`, `tracker`) are wired through `decode_dsky` → `dsky_ui` but their source booleans never change.

The goal of M-D is to make every lamp boolean reflect a real internal condition so the agc-sim DSKY panel matches Frank O'Brien chapter 7 lamp semantics.

### M-D.1 Add a "lamp driver" pass to the T4RUPT path
A new function `services::lamps::refresh_lamps(state, hw)` is called from `services/t4rupt.rs` *before* `decode_dsky`. It sets the relevant `state.dsky.*` booleans from a single source of truth:

| Lamp | Driving condition | Source |
|---|---|---|
| `comp_acty` | computer activity pulse (≥1 PINBALL/Waitlist tick in the last T4 window) | scheduler tick counter latched for ~100 ms |
| `no_att` | IMU not aligned | `state.imu.alignment != ImuAlignmentState::Aligned` |
| `gimbal_lock` | gimbal critical | `imu_control::is_gimbal_lock_critical(cdu)` |
| `key_rel` | DSKY load awaiting next register or V37 mid-sequence | `state.dsky.load_state.is_some()` |
| `prog_alarm` | alarm raised, not yet acknowledged | `state.alarm.lit` |
| `tracker` | sextant mark pending (P22/P23 active mark window) | `state.optics.mark_window_open` |
| `restart_flag` | clear on next successful V37 program-mode change | `services::fresh_start` writes set, V37 dispatch writes clear |
| `temp` | deferred (no temperature HAL in agc-sim) — leave wired to `state.dsky.temp` which stays false | n/a |
| `stby` | already correct — P06 / V37E00E sets, V37E37E clears | small fix: V37 to a non-P06 major mode must clear it |
| `uplink_activity` | already correct | unchanged |

### M-D.2 Implement `SimDsky::set_lamp`
Today `agc-sim/src/hardware.rs:118` is `fn set_lamp(&mut self, _lamp: Lamp, _on: bool) {}`. After M-D.1 the lamp truth is in `state.dsky.*` and the UI reads it via `decode_dsky`, so set_lamp's job in agc-sim is purely a debug-log channel (and a test surface). Replace with a record-into-`Vec` (drained per render frame) so tests can assert that `set_lamp` was called with the expected `(Lamp, bool)` events.

### M-D.3 dsky_ui visual feedback parity
`agc-sim/src/dsky_ui.rs:103` `lamp_grid` already pulls every variant — so once the underlying booleans move, the UI lights up. The remaining UI work:
- distinguish *caution* (yellow) from *warning* (red) per O'Brien §7.2 — small palette change in `dsky_ui.rs`;
- add a one-line "alarm code" footer under R3 when `prog_alarm` is lit (read from `state.alarm.code`).

### M-D.4 Snapshot tests
Add a `tests/dsky_lamps_snapshot.rs` integration test in `agc-sim` that drives scenarios and asserts on the rendered `DskyFrame`:
- gimbal march into critical → `gimbal_lock` lights
- raise an 1107 alarm → `prog_alarm` lights, footer shows `01107`
- V35 → all lamps light for ~5 s then revert
- enter P22, push a sextant mark window → `tracker` lights
- run V21 → `key_rel` lights between digits

**Milestone D exit criterion:** every entry in the `Lamps` struct is observably driven by a real condition in at least one scenario test, and a manual run of the agc-sim TUI shows lamps changing through the Apollo-8 timeline.

---

## 7. Sequencing recommendation

The current branch `feature/62-sm-sep-pyro` (M-A.1) is already in progress and should land first. Recommended sequencing after that:

1. **M-A.2** entry display nouns — completes the entry-phase crew interface, which is the most visible gap.
2. **M-A.3** crew-accessible verb dispatch — five tiny PRs; each one closes a status-report row.
3. **M-D** lights wiring — independent of A.2/A.3 and immediately user-visible in agc-sim demos; can run in parallel.
4. **M-A.4** + **M-A.5** R30/N44 TFF and N43.
5. **M-B.1** P70 TLI targeting — the largest narrative gap.
6. **M-B.2–B.5** in parallel; small. **M-B.6** (N08/N09) ships together with — or right after — M-B.5: the AlarmState extension is shared, and the crew-visible alarm read-out only makes sense once POODOO/GOTOPOOH is in place.
7. **M-C** in priority order (KALCMANU first, then downlink format, then P02 loop).

---

## 8. Out-of-scope reminder

The following remain explicitly deferred and are NOT part of this plan:

- All bare-metal targets (`agc-board-nucleo-f767`, `agc-bridge-pico`, `thumbv*` builds, related HAL hardware paths).
- Any LM-side rendezvous (P32–P35 are already in tree but Apollo 8 will not exercise them; P38, P39, P72–P78 stay missing).
- Lunar landing (P63 ballistic-hold descent variants, P64/P65/P66 landing-phase math distinct from the entry-phase port).
- Self-test / engineering DSKY surface (V07, V17, V27, V91, V92).
- MSFN downlink encoding only enters in M-C.2 — agc-sim demos still rely on the in-process telemetry log.

These will be revisited once Apollo 8 is fully demonstrable end-to-end.
