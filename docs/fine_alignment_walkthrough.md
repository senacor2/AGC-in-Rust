# Fine-Alignment Walkthrough (clearing NO ATT)

A guided walk-through of the keystroke + optics-MARK sequence that takes the
IMU from a caged FRESH START through P51 (coarse) and P52 (fine) alignment,
extinguishing the **NO ATT** warning lamp. Companion to the interactive sextant
(#176) and the scenario test
`agc-sim/src/scenario.rs::tc_scn_keystroke_mark_p51_then_p52`.

Audience: a live demo of the host-side simulator (`dsky_sim`), or a
desk-walkthrough of the P51/P52 alignment flow. Sibling of
[`tei_burn_demo.md`](tei_burn_demo.md) and [`p40_burn_demo.md`](p40_burn_demo.md).

Concept & design: [`sextant_ui_concept.md`](sextant_ui_concept.md).

## Goal

Drive the platform `Caged → CoarseAligned → FineAligned` from the DSKY keyboard,
watching **NO ATT** go out the moment fine alignment completes.

## The state machine and the NO ATT lamp

`NO ATT` is lit whenever the platform is not fine-aligned
(`agc-core/src/services/lamps.rs`: `no_att = imu_alignment_state != FineAligned`).
A FRESH START boots into `Caged`, so `NO ATT` is on at power-up — correct.

| State | Meaning | NO ATT | How you leave it |
|-------|---------|--------|------------------|
| `Caged` | platform not oriented | **on** | P51 + two star marks → `CoarseAligned` |
| `CoarseAligned` | rough orientation known | **on** | P52 + two star marks → `FineAligned` |
| `FineAligned` | precise REFSMMAT | **off** | — |

Each program needs **two** star sightings. A sighting = select the star (N70),
slew the sextant onto it, and MARK. The two marks of a pair must point in clearly
different directions (two marks in the same direction are collinear and raise
program alarm `0o220` — "IMU NOT ALIGNED"); the walkthrough uses two stars ~60°
apart.

## Keys

| Key | Action |
|-----|--------|
| `V` `N` `E` `P` `C` `R` `K` digits `+` `-` | the DSKY keypad (as elsewhere) |
| `↑` `↓` | slew trunnion − / + |
| `←` `→` | slew shaft − / + |
| `Shift` + arrow | fine slew (≈0.5°; coarse ≈5°) |
| `M` | **MARK** the entered star at the current optics angles |
| `Q` / `Ctrl-C` | quit |

The sextant panel (right of the DSKY) shows the reticle: `┼` is the current line
of sight, `★` is the selected star, live `SHAFT`/`TRUN`, the angular `off`set,
and `MARK n/2`. When the star is outside the field an edge arrow (`▶◀▲▼`) points
toward it; slew that way until `★` appears and drifts to centre.

## Demo stars

| Star | N70 code | Aim for (≈) | Note |
|------|----------|-------------|------|
| **Polaris** | `05` | SHAFT ≈ **31°**, TRUN ≈ **1°** | near the initial pointing (little slew) |
| **Alpheratz** | `01` | SHAFT ≈ **2°**, TRUN ≈ **61°** | ~60° from Polaris |

Rough pointing is enough to *complete* the alignment (the state transition only
needs two non-collinear marks); centring precisely (offset → 0, `★` turns green)
just makes the recovered REFSMMAT more accurate.

**Star entry — `V21 N70`.** Enter the star code with **V21** (load R1): after the
final `E` the code commits to `vn.crew_star_code` and the sextant panel picks up
the target immediately. (`V25 N70` also works but loads three registers, so it
does **not** commit — and the panel keeps showing "no star" — until you `E`
through R2 and R3 as well; the scenario-runner helper `v25_load_three` does that.)

## Walkthrough

Run it: `cargo run -p agc-sim --bin dsky_sim`. At start `NO ATT` is lit.

### P51 — coarse alignment (`Caged → CoarseAligned`)

1. **Select P51:** `V 3 7 E 5 1 E`  → PROG shows `51`, VERB/NOUN flash `06 70`.
2. **First star (Polaris):** `V 2 1 N 7 0 E 0 5 E`  → the panel targets Polaris.
3. **Slew onto it:** press `→` until `SHAFT ≈ 31°`; trunnion is already ≈0–1°.
   Watch `★` slide toward `┼` and `off` shrink.
4. **Mark:** `M`  → status: `MARK 1 of 2 — Polaris (#5) buffered`.
5. **Second star (Alpheratz):** `V 2 1 N 7 0 E 0 1 E`.
6. **Slew onto it:** `↑` until `TRUN ≈ 61°`, `←` until `SHAFT ≈ 2°`.
7. **Mark:** `M`  → status: `MARK 2 of 2 — Alpheratz (#1): Caged → CoarseAligned`.

`NO ATT` is still on (coarse only).

### P52 — fine alignment (`CoarseAligned → FineAligned`)

8. **Select P52:** `V 3 7 E 5 2 E`.
9. Repeat the two-star mark: `V 2 1 N 7 0 E 0 5 E`, slew to Polaris, `M`; then
   `V 2 1 N 7 0 E 0 1 E`, slew to Alpheratz, `M`.
10. On the second mark the status shows `… CoarseAligned → FineAligned` and
    **NO ATT goes out.**

## Verifying NO ATT extinguished

- **Visually:** the `NO ATT` cell (top-right lamp block) drops from red to dim.
- **In a test:** assert `state.imu_alignment_state == FineAligned` (that is exactly
  what `lamps.rs` maps to `no_att = false`).

## Paths for tests / automation

The interactive slew + `M` are terminal key events and are *not* expressible in
the `.dsky` uplink script format (which only carries `feed_key` keystrokes). Drive
the alignment programmatically instead:

- **Scenario runner** (keystroke path, end to end):
  `ScenarioBuilder::v37_select`, `.v25_load_three(70, [code,0,0])`, `.crew_star_mark()`
  — see `tc_scn_keystroke_mark_p51_then_p52` in `agc-sim/src/scenario.rs`, which
  goes FRESH-START `Caged` → `FineAligned` entirely through the keyboard pipeline.
- **Sextant session** (optics path): `MarkSession::{slew, mark}` in
  `agc-sim/src/sextant.rs` — see `tc_sxt_two_mark_p52_reaches_fine`.
- **Direct calls** (unit level): `pxx_mark_align` / `p51_mark_align` /
  `p52_mark_align` in `agc-core/src/programs/p51_p52.rs` with pre-computed star
  vectors.
