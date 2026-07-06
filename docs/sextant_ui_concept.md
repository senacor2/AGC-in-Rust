# Concept: Simulated sextant for the agc-sim TUI (#109)

**Status**: Visual + interaction concept **for approval** (issue #109). Implementation
is tracked separately (#176). The approved visual style is the **reticle starfield
with manual slew**.

**Motivation**: better demonstrations of interactive IMU alignment. The keystroke
MARK logic (P51/P52, consuming `vn.crew_star_code`) already exists (#175/#177); today
the sighting is synthesised from ground truth. This concept adds a sextant *view* so a
crew can point the optics at a star and press MARK from the TUI, closing the loop that
extinguishes `NO ATT`.

---

## 1. Apollo optics primer (what we are modelling)

The CM optics for alignment is the **Sextant (SXT)**: a narrow (~1.8°) field with a
fixed line and a movable **star line**. The crew drives the star line with two angles:

- **Shaft (SA)** — rotation about the optical axis.
- **Trunnion (TA)** — deflection of the star line off the axis.

The crew slews SA/TA to superimpose the star on the reticle crosshair, then presses
**MARK**, latching the optics CDU angles at that instant. Two star marks determine the
platform orientation (the TRIAD solution behind `refsmmat_from_star_sightings`).

Both angles already exist in the sim as `SimOptics { shaft, trunnion }`
(`agc-sim/src/hardware.rs`), and `agc-core/src/control/sextant.rs` provides the exact
mapping we need:

```
los_body_from_cdu(shaft, trunnion) -> Vec3        // where the optics points
cdu_from_los_body(los_body)        -> (shaft, trunnion)   // inverse
```

## 2. Visual concept — the reticle starfield panel (approved)

A framed sextant field-of-view panel, rendered in the existing `dsky_ui.rs` style
(crossterm box-drawing + the established caution/warning colours), placed **below the
DSKY keypad** (≈ 40 cols × 10 rows; exact geometry a #176 detail):

```
┌ SEXTANT ─── SHAFT 214.7° · TRUN 31.2° ┐
│            ·      ·                    │
│                 ·        ✦             │
│           ·      ┼                     │
│                      ★  ← Pollux       │
│      ·                        ·        │
│ STAR 16 POLLUX   offset 2.4°           │
│ slew ↑↓←→        centre & press [M]ARK │
│ MARK 1 of 2                            │
└────────────────────────────────────────┘
```

Glyphs / colour:
- `┼` — reticle centre = the current optics line of sight (fixed at panel centre).
- `★` — the **target** star (the one selected via N70), coloured; blinks or turns
  green when within the mark tolerance.
- `·` — other catalogue stars in the field, for context (DIM).
- `✦` — optional decorative bright star.
- Header shows live SHAFT/TRUN; footer shows the selected star, angular `offset`, the
  slew hint, and the `MARK n of 2` counter.

### 2.1 Projection model (how the star gets its screen position)

The reticle centre is *always* the current optics pointing,
`p = los_body_from_cdu(shaft, trunnion)`. The target star's true body-frame direction
`s` comes from its catalogue inertial vector rotated through the (truth) REFSMMAT and
attitude — the same `star_los_in_platform` machinery the scenario runner uses. The
star is drawn at the angular offset of `s` from `p`, projected onto the reticle plane
(local tangent-plane / gnomonic projection), scaled so the panel spans the sextant
FOV. As the crew slews, `p` moves toward `s`, and the star slides to centre. The
**`offset`** readout is `acos(p·s)` in degrees.

## 3. Interaction concept — manual slew + MARK

The DSKY keypad stays the crew's command surface; the sextant panel adds slew + mark:

| Key | Action |
|-----|--------|
| `↑` / `↓` | slew **trunnion** − / + (coarse) |
| `←` / `→` | slew **shaft** − / + (coarse) |
| `Shift`+arrow (or `[` / `]`) | fine slew step (optional) |
| `M` | **MARK** — latch the current shaft/trunnion as a sighting |

(Exact bindings are a #176 detail; arrows for slew + `M` for mark is the proposal.)

### 3.1 End-to-end flow (extends #175, all from the TUI)

1. `V37 E 51 E` — select **P51** (coarse). `NO ATT` is lit (platform `Caged`).
2. `V25 N70 E <code> E` — enter the first star code → `vn.crew_star_code`
   (existing `feed_key` path). The sextant panel highlights that star as the target.
3. **Slew** shaft/trunnion until `offset` is within tolerance (star at centre).
4. **`M`** (MARK) → build the sighting from the *actual* latched angles and feed it
   into the mark pipeline (the #175 `CrewStarMark` logic).
5. Repeat 2–4 for the second star → `pxx_mark_align` runs P51 → **`CoarseAligned`**.
6. `V37 E 52 E`, then two more N70 + slew + MARK → P52 → **`FineAligned`** →
   **`NO ATT` extinguishes.**

This is exactly the `tc_scn_keystroke_mark_p51_then_p52` sequence, but with a real
pointing step between star selection and mark.

### 3.2 Feedback & edge cases

- **Centred indicator**: the target star turns green / the footer shows `● ON MARK`
  when `offset < tol`; MARK outside tolerance is allowed but recorded with that error
  (realistic crew error) — or gated, see §4.
- **Mark counter**: `MARK n of 2`; after two, the pipeline dispatches and the panel
  reports the resulting alignment state.
- **Out-of-FOV**: if the star is far off, draw an edge arrow (`▶`) pointing toward it
  rather than clipping.
- **Bad pair**: near-collinear stars raise the existing alarm `220` — surface it in
  the footer, don't panic.

## 4. Key design decision for #176 approval

**Does MARK use the crew's actual slewed pointing, or snap to truth?**

- **Actual pointing (recommended):** MARK records `los_body_from_cdu(shaft, trunnion)`
  at the press. A well-centred mark yields a REFSMMAT close to truth; a sloppy one
  carries the crew's pointing error into the alignment. This is the demonstrative,
  realistic choice and exercises the full CDU→LOS pipeline. It differs from #175 today
  (which synthesises the exact truth LOS).
- **Snap-to-truth (simpler):** MARK ignores the slewed angles and uses the exact star
  LOS (today's #175 behaviour). Cleaner but the slewing becomes cosmetic.

Recommendation: **actual pointing**, with a tolerance gate that *warns* (not blocks) on
a large offset, so demos can show both a good alignment and the effect of a bad mark.

## 5. What #176 will need to build

- **A truth attitude + REFSMMAT in `dsky_sim`.** The interactive binary currently has
  only `AgcState` + DSKY — no spacecraft attitude. #176 must add a (fixed or
  configurable) truth attitude/REFSMMAT so `star_los_in_platform` can produce the
  target direction. *This is the main new dependency.*
- **The reticle renderer** — a new panel in the `dsky_ui.rs` style (projection §2.1).
- **Slew input** in the `dsky_sim` event loop (arrow keys → `SimOptics` shaft/trunnion).
- **MARK wiring** — on `M`, build the sighting from the latched angles and run the
  #175 mark logic. Recommend factoring the scenario `CrewStarMark` handler's
  buffer-pair-and-dispatch into a shared helper both call sites use.
- Reuse: `SimOptics` (angles), `control::sextant::{los_body_from_cdu, cdu_from_los_body}`,
  `star_los_in_platform`, `pxx_mark_align`, `STAR_CATALOG` (names/directions).

## 6. Out of scope (this concept / #176)

- The Scanning Telescope (wide-field SCT) and its reticle.
- Automatic optics drive / star-acquisition assist.
- Landmark tracking optics (P22) — the same panel could show it later, but the concept
  targets star alignment (P51/P52).

## 7. Open questions for approval

1. Panel placement — below the keypad (proposed) vs. to the right of the DSKY?
2. Slew bindings — arrows + `M` (proposed) vs. WASD, and one vs. two slew step sizes?
3. §4 — actual-pointing (recommended) vs. snap-to-truth for the first #176 cut?
4. Truth attitude for `dsky_sim` — a fixed default, or crew-settable (e.g. a `V25`
   noun) for varied demos?
