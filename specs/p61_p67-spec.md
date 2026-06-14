# Specification: `programs/p61_p67` Module — Entry Guidance Programs

## Change Log

- **2026-06-13** — Audited under issue #159 by analyst-reengineer agent. Updated §1 and §2 to reflect the full `EntryState` struct (14 fields) and extended `EntryPhase` enum (Ballistic, Skip, Constd variants) implemented in later milestones; §6 already documented these additions but §1–§2 were stale from the original MS4 Phase 5 skeleton spec.

**Status**: Approved for implementation (Milestone 4 Phase 5 — skeletons; §6 extends with closed-loop guidance, Milestone MS-E3)
**Module path**: `agc-core/src/programs/p61_p67.rs`
**Architecture reference**: `docs/architecture.md` §7.2 (P-programs), §10 (entry guidance)
**AGC source files**:
- `Comanche055/P61-P67.agc` — entry guidance program entry blocks
- `Comanche055/REENTRY_CONTROL.agc` — entry control law
- `Comanche055/ENTRY_LEXICON.agc` — entry-specific state variables

---

## 1. Purpose and Scope

P61–P67 are the family of entry-guidance programs that execute during
Earth atmospheric entry after the Trans-Earth Coast phase. They sequence
the vehicle through entry preparation, CM/SM separation, pre-0.05g
monitoring, closed-loop entry guidance, and final drogue deployment.

The MS4 Phase 5 skeletons establish the phase state machine, major-mode/DSKY
wiring, and the inter-program handoff contract. Milestone MS-E3 added the
closed-loop guidance math (roll steering, HUNTEST range prediction, `select_phase`
divergence to Ballistic, and the extended `EntryState` fields in §6).

### What this module provides

- `EntryPhase` enum: `Idle → Preparation → Separation → PreEntry → Entry → Final`,
  plus `Ballistic` (HUNTEST divergence hold), `Skip` (UPCONTRL closed-loop),
  and `Constd` (constant-drag guidance) added in MS-E3.
- `EntryState` struct in `AgcState`: full entry guidance state block (see §2).
- `P61_MAJOR_MODE … P67_MAJOR_MODE` constants.
- `PRIORITY: JobPriority = 10` — one tier above the background monitors.
- `init_p61 … init_p67` entry points.
- `p63_check_threshold` — advances `PreEntry → Entry` when
  `sensed_acceleration_g >= 0.05`.
- `p67_deploy_drogue` — sets `drogue_deployed: bool`. Drogue hardware actuation
  is a HAL concern out of scope here.
- `G0_MPS2: f64 = 9.806_65` — standard gravity (m/s²) for g-loading conversion.
- `ENTRY_THRESHOLD_G: f64 = 0.05` — the 0.05g threshold constant.

### What this module does NOT provide

- Real sensed-acceleration integration — `sensed_acceleration_g` is
  updated by `entry_servicer_exit` in `guidance::entry` each SERVICER cycle (not
  by the skeleton entry points directly).
- CM/SM separation pyrotechnic commands — the HAL SECS interface does
  not yet exist; P62 only updates phase state and DAP mode.
- Drogue and main parachute HAL actuation.

---

## 2. `EntryState`

The full struct as implemented (all fields zero-initialised unless stated):

```rust
pub struct EntryState {
    /// Current entry-guidance phase.
    pub phase: EntryPhase,                  // Default: Idle

    /// Sensed spacecraft acceleration (g units).
    /// Updated each SERVICER cycle by `entry_servicer_exit` from
    /// `state.servicer_last_dv_inertial`.
    pub sensed_acceleration_g: f64,         // Default: 0.0

    /// Inertial altitude rate d|r|/dt (m/s, positive = climbing).
    /// Updated each SERVICER cycle. Equals r · v / |r|.
    /// AGC: V16N64 R2 (HDOT).
    pub r_dot_mps: f64,                     // Default: 0.0

    /// Roll command the entry guidance law is holding (radians).
    /// 0 = lift up, positive = right-bank.
    /// Updated each SERVICER cycle by `guidance::entry::resolve_roll`.
    /// AGC: `ROLLC` (REENTRY_CONTROL.agc:1308).
    pub roll_command_rad: f64,              // Default: 0.0

    /// Great-circle range from sub-satellite point to target landing site (km).
    /// Updated each SERVICER cycle.
    pub target_range_km: f64,              // Default: 0.0

    /// Predicted total range to target (km) — output of `predict_range`.
    /// AGC: `ASP`.
    pub predicted_range_km: f64,           // Default: 0.0

    /// Signed downrange error in km: `target_range_km - predicted_range_km`.
    /// Drives the HUNTEST L/D update. AGC: `DIFF`.
    pub downrange_error_km: f64,           // Default: 0.0

    /// Signed cross-range distance (km). Positive = right of track.
    /// Used by `resolve_roll` to choose bank direction.
    /// AGC: `LATANG`.
    pub crossrange_km: f64,               // Default: 0.0

    /// Last computed vertical L/D command (dimensionless, range [-LAD, LAD]).
    /// Output of `compute_ld_command`. AGC: `L/D`.
    pub ld_command: f64,                  // Default: 0.0

    /// HUNTEST iterated reference L/D (`LEWD` in REENTRY_CONTROL.agc).
    /// Initialised to `entry_tables::LEWD_INIT` on first HUNTEST pass.
    pub lewd_ref: f64,                    // Default: 0.0

    /// HUNTEST iteration step (`DLEWD` in REENTRY_CONTROL.agc).
    /// Updated each cycle by Newton step.
    pub dlewd: f64,                       // Default: 0.0

    /// Previous downrange error (km) — `DIFFOLD` in REENTRY_CONTROL.agc.
    /// Saved at end of each HUNTEST pass for the next cycle's Newton step.
    pub diffold_km: f64,                  // Default: 0.0

    /// SKIPPER nonlinear gain — `FACTOR` in REENTRY_CONTROL.agc.
    /// Updated each `Skip` cycle by UPCONTRL's CONTINU2 block.
    pub factor: f64,                      // Default: 0.0

    /// `false` until the first SERVICER cycle in `EntryPhase::Entry`.
    /// On the first cycle, `lewd_ref` and `dlewd` are initialised from
    /// `entry_tables` constants (FOREHUNT block in REENTRY_CONTROL.agc).
    pub hunt_initialized: bool,           // Default: false

    /// True after P67 has commanded drogue deployment.
    pub drogue_deployed: bool,            // Default: false
}
```

Added to `AgcState` as `entry: EntryState`, initialised to `Default::default()`.

---

## 3. Program Alarms

| Code | Trigger                                                    |
|------|------------------------------------------------------------|
| 231  | P62 invoked while entry phase is not `Preparation`.        |
| 232  | P63 invoked while entry phase is not `Separation`.         |
| 233  | P64 invoked while sensed_acceleration_g < 0.05 (pre-0.05g).|
| 234  | P67 invoked while entry phase is not `Entry`.              |

Alarms are "soft" — they set `alarm.code`/`alarm.lit` but do **not**
abort the program. The major mode is still advanced so the crew can
manually override if needed.

---

## 4. Program Behaviours

### 4.1 `init_p61` — Entry Preparation

- `entry.phase = Preparation`
- `major_mode = 61`, `dsky.prog = 61`, `dsky.verb = 6`, `dsky.noun = 61`
- Display `r[0]` = target range (stub 0), `r[1]` = 0, `r[2]` = 0
- No alarm.

### 4.2 `init_p62` — CM/SM Separation

- Alarm 231 if `entry.phase != Preparation` (but still advance).
- `entry.phase = Separation`
- `major_mode = 62`, `dsky.prog = 62`, `dsky.verb = 6`, `dsky.noun = 62`
- `state.pending_maneuver = None` (any stale ΔV is void post-separation).
- Transition DAP to `AttitudeHold` (CM-only RCS control).

### 4.3 `init_p63` — Pre-0.05g Entry Initialisation

- Alarm 232 if `entry.phase != Separation` (but still advance).
- `entry.phase = PreEntry`
- `major_mode = 63`, `dsky.prog = 63`, `dsky.verb = 16`, `dsky.noun = 64`
  (continuously updated entry status)
- `dsky.r[0]` = `entry.sensed_acceleration_g as f32`
- `dsky.r[1]` = 0 (stub)
- `dsky.r[2]` = 0 (stub)

### 4.4 `p63_check_threshold`

Called from tests (and eventually the SERVICER) with the current sensed
acceleration in g units already staged in `state.entry.sensed_acceleration_g`.

- If `entry.phase == PreEntry` and `sensed_acceleration_g >= 0.05`:
  - `entry.phase = Entry`
  - Return `true`.
- Otherwise return `false`.

### 4.5 `init_p64` — Closed-Loop Entry Guidance

- Alarm 233 if `entry.sensed_acceleration_g < 0.05` (early invocation).
- `entry.phase = Entry` (force, even if phase was PreEntry).
- `major_mode = 64`, `dsky.prog = 64`, `dsky.verb = 16`, `dsky.noun = 64`.
- `entry.roll_command_rad = 0.0` — stub.
- Display same triplet as P63.

### 4.6 `init_p67` — Drogue Deploy / Final Phase

- Alarm 234 if `entry.phase != Entry`.
- `entry.phase = Final`
- `major_mode = 67`, `dsky.prog = 67`, `dsky.verb = 6`, `dsky.noun = 67`.
- Call `p67_deploy_drogue(state)`.

### 4.7 `p67_deploy_drogue`

- `entry.drogue_deployed = true`.
- Future: call `hw.secs().deploy_drogue()`.

---

## 5. Test Cases

### TC-P61-1: `init_p61` sets phase = Preparation and major_mode = 61.

### TC-P62-1: `init_p62` from Preparation advances to Separation and clears pending_maneuver.

### TC-P62-2: `init_p62` from Idle raises alarm 231 but still advances.

### TC-P63-1: `init_p63` from Separation advances to PreEntry.

### TC-P63-2: `p63_check_threshold` with g = 0.04 returns false and stays PreEntry.

### TC-P63-3: `p63_check_threshold` with g = 0.08 returns true and advances to Entry.

### TC-P64-1: `init_p64` with g = 0.10 sets phase = Entry, no alarm.

### TC-P64-2: `init_p64` with g = 0.02 raises alarm 233.

### TC-P67-1: `init_p67` from Entry sets phase = Final and drogue_deployed = true.

### TC-P67-2: `init_p67` from Preparation raises alarm 234 but still advances.

---

## 6. P64 closed-loop guidance (HUNTEST / INITROLL)

The phase state machine and DSKY wiring in §1–§5 are the static surface of
the P64 entry point. The closed-loop math behind it lives in
`agc-core/src/guidance/entry.rs` and runs each SERVICER cycle.

### 6.1 `EntryPhase::Ballistic`

New variant (MS-E3) — destination for a `select_phase` divergence. The DAP holds the
last `EntryRoll(roll_command_rad)` command while in this state; the P66
program proper is a separate concern (see `entry-guidance-plan.md`).

### 6.2 `EntryPhase::Skip` and `EntryPhase::Constd`

Two further variants added in MS-E3:

- `Skip` — UPCONTRL / up-control phase: entered from `Entry` when HUNTEST converges.
  The SKIPPER feedback law maintains the converged trajectory using `ΔL/D` from
  `(RDOT − RDOTREF)` and `(V − VREF)` errors. AGC source:
  `REENTRY_CONTROL.agc:875–1020`.

- `Constd` — constant-drag closed-loop guidance: entered from `Entry` when HUNTEST
  diverges (`|range_error| > entry_tables::RANGE_ERR_THRESHOLD_KM`). Flies a
  constant-drag reference profile `D0 = KA3·LEQ + KA4` with K1D / K2D feedback.
  AGC source: `REENTRY_CONTROL.agc:1023` (`DCONSTD`). Exits to `Skip` once range
  error falls back inside `HUNTEST_CONVERGED_KM`, to `Final` at `V < VFINAL1`, or
  to `Ballistic` if drag drops below `Q7F`.

### 6.3 `EntryState` fields used by closed-loop guidance

| Field | Type | Role | AGC label |
|---|---|---|---|
| `predicted_range_km` | `f64` | Output of `guidance::entry::predict_range` | `ASP` |
| `downrange_error_km` | `f64` | `target_range - predicted_range` | `DIFF` |
| `crossrange_km` | `f64` | Signed cross-range, drives bank sign | `LATANG` |
| `ld_command` | `f64` | Output of `compute_ld_command` (clamped to ±LAD) | `L/D` |
| `lewd_ref` | `f64` | HUNTEST iterated reference L/D | `LEWD` |
| `dlewd` | `f64` | HUNTEST iteration step | `DLEWD` |
| `diffold_km` | `f64` | Previous DIFF (Newton "old" point) | `DIFFOLD` |
| `hunt_initialized` | `bool` | FOREHUNT init flag | implicit |

All `f64`, zero-initialised; the bool starts `false`. The const constructor
`EntryState::new` adds them.

### 6.4 Closed-loop call sequence in `entry_servicer_exit`

After the sensed-g / R-dot / range-to-go / threshold-trip work, the SERVICER
cycle runs the closed loop — only when `state.entry.phase == EntryPhase::Entry`:

1. `state.entry.predicted_range_km = entry::predict_range(state)`
2. `let upd = entry::compute_ld_command(state)` — Newton step on `LEWD`.
3. Copy `upd` into `EntryState`: `ld_command`, `lewd_ref`, `dlewd`,
   `diffold_km` (also mirrored to `downrange_error_km`); set
   `hunt_initialized = true`.
4. `state.entry.roll_command_rad = entry::resolve_roll(state, ld_command)`.
5. `state.dap_state.mode = DapMode::EntryRoll(roll_command_rad)`.
6. `if let Some(next) = entry::select_phase(state) { state.entry.phase = next; }`.

`init_p64` is a thin pass-through that advances the phase and writes the
V16N64 DSKY display; the actual bank command is produced one SERVICER tick
later — matching the AGC, where the P64 entry point is also a handoff into
the cyclic guidance task.

### 6.5 Public API in `guidance::entry`

| Symbol | Returns | Purpose |
|---|---|---|
| `predict_range(state) -> f64` | km | Tabulated reference range (`RTOGO`) plus `LEWD` sensitivity |
| `compute_ld_command(state) -> LdUpdate` | struct | HUNTEST Newton step + saturation to `±LAD` |
| `resolve_roll(state, ld_cmd) -> f64` | rad | `ROLLC = acos(ld/LAD)`; sign from cross-range |
| `select_phase(state) -> Option<EntryPhase>` | enum | `V < VFINAL1 → Final`; `\|DIFF\| > 500 km → Ballistic`; else `None` |

The reference-profile table in `guidance/entry_tables.rs` is transcribed from
`REENTRY_CONTROL.agc:1369–1467`; each constant carries a doc-comment mapping
to its AGC label and B-scaling.

### 6.6 Test cases (closed-loop math)

Under `agc-core/src/guidance/entry.rs::tests` and `entry_tables::tests`:

- `tc_mse3_pr_{1..4}` — `predict_range`: minimal at the slowest sample,
  large at hyper-velocity, monotone in V, sensitive to `LEWD`.
- `tc_mse3_ld_{1..4}` — `compute_ld_command`: FOREHUNT init, Newton update,
  saturation to LAD, `LEWDPTR` clamp.
- `tc_mse3_rr_{1..4}` — `resolve_roll`: `acos` magnitude at `±LAD` and 0;
  sign from cross-range.
- `tc_mse3_sp_{1..3}` — `select_phase`: V → Final, nominal stays in Entry,
  range-divergence → Ballistic.
- `tc_mse3_cr_{1..2}` — `crossrange_km` helper sanity.
- `tc_tab_{1..6}` — reference-profile table sanity.

Tolerances: 1e-4 on L/D, 1e-3 on roll (rad), 1 km on range — aligned with
the fixture tolerance in `specs/entry-guidance-plan.md` §6.2.
