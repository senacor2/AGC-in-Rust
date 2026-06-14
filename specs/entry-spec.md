# Specification: `guidance/entry` Module — Entry Guidance Kernel

**Status**: Approved for implementation
**Module path**: `agc-core/src/guidance/entry.rs`
**Companion data module**: `agc-core/src/guidance/entry_tables.rs`
**AGC source counterpart**: `Comanche055/REENTRY_CONTROL.agc` (primary), `Comanche055/ENTRY_LEXICON.agc` (variable names)

**Related specs**:
- `specs/p61_p67-spec.md` — program-level state machine that calls every function here
- `specs/entry-guidance-plan.md` — milestone plan; do not restate
- `specs/dap-spec.md` — DAP `EntryRoll(f64)` mode driven by `resolve_roll`
- `specs/average-g-spec.md` — sensed-g source feeding `entry.sensed_acceleration_g`

---

## Glossary

| Term | Meaning |
|---|---|
| HUNTEST | AGC label for the iterative Newton loop that predicts pull-out velocity `V₁` and exit velocity `VL` from the current drag / altitude-rate state |
| INITROLL | The first bank-command computation after the 0.05g threshold trips; implemented in `compute_ld_command` |
| UPCONTRL / SKIPPER | P65 closed-loop law: tracks a reference (`VREF`, `RDOTREF`) derived from the frozen HUNTEST intermediates |
| PREDICT3 | AGC tabulated range-prediction look-up (REENTRY_CONTROL.agc:1369–1467); primary oracle for P67, fallback for `predict_range` |
| RANGER | The analytic (non-tabulated) range-prediction block in REENTRY_CONTROL.agc:654–732 |
| GLIMITER | AGC deceleration limiter at REENTRY_CONTROL.agc:1247; clips `L/D` to `LAD` when deceleration is excessive |
| CONSTD / DCONSTD | Constant-drag closed-loop branch (REENTRY_CONTROL.agc:1023–1059); entered when HUNTEST iteration diverges |
| DOWNCNTL | Branch of UPCONTRL for `V > V₁` (vehicle more energetic than the pull-out prediction); REENTRY_CONTROL.agc:1061–1091 |
| FOREHUNT | Initialisation block that seeds `LEWD` / `DLEWD` / `DIFFOLD` on the first HUNTEST cycle |
| LEWDPTR | Safety clamp in the Newton step: if `LEWD + ΔLEWD < 0`, halve the step instead |
| L/D (vertical) | Dimensionless lift-to-drag ratio; a command in `[-LAD, +LAD]`. Positive = lift-up |
| LAD | Maximum vehicle L/D ratio (0.30); from `LADPAD` in ENTRY_LEXICON.agc |
| LOD | Final-phase nominal L/D (0.18); from `LODPAD` |
| LEWD | `L/D` reference tracked by HUNTEST (iterated each cycle) |
| DLEWD | Newton iteration step on LEWD |
| DIFF | Downrange error = `target_range_km − predicted_range_km` (km) |
| DIFFOLD | Previous-cycle DIFF; one side of the Newton secant |
| RDOT / HDOT | `dr/dt` (m/s); positive = climbing. AGC V16N64 R2 display |
| FACTOR / F1 | SKIPPER nonlinear gain = `(A1 − Q7F)/(D − Q7F)`; updated when `D > Q7MIN` |
| VL | Exit velocity from up-control; the point at which the trajectory re-enters the atmosphere after a skip |
| VLMIN | Minimum acceptable `VL`; below this HUNTEST cannot generate a meaningful pull-out prediction |
| VFINAL1 | Velocity threshold below which any closed-loop phase transitions to P67 Final |
| KEP | P66 ballistic hold (coast arc); entered from UPCONTRL when drag falls below `Q7F` |
| GAMMAL | Approximate flight-path angle at pull-out; derived from HUNTEST via the DHOOK / AHOOKDV second-order correction |
| ASKEP | Keplerian-arc range component from pull-out to entry interface |
| ASP | Total predicted range in Earth revolutions; converted to km for output |

---

## §1 Purpose and Scope

`guidance/entry.rs` is the **closed-loop entry guidance kernel**. It provides pure mathematical functions that are called every SERVICER cycle (nominally 2 s) once the spacecraft has tripped the 0.05 g threshold.

### What this module provides

- Range prediction from current state to the velocity at which pull-out deceleration peaks (`predict_range`), built on the HUNTEST intermediates and the RANGER analytic block, with graceful fallback to the PREDICT3 table.
- An accessor for the HUNTEST exit velocity (`predicted_exit_velocity_mps`).
- A Newton-iteration L/D command generator (`compute_ld_command` — HUNTEST / INITROLL / FOREHUNT paths).
- A bank-angle resolver (`resolve_roll` — ROLLC / INITROLL).
- A P65 UPCONTRL / SKIPPER step function (`upcontrol_step`), including DOWNCNTL, branch-3 full-lift-up, and the nonlinear `FACTOR` gain.
- A constant-drag (CONSTD) closed-loop step function (`constd_step`).
- A P66 ballistic hold (`ballistic_step` — freeze attitude, no L/D update).
- A P67 final-phase PREDICT3 step function (`final_phase_step` — terminal range control).
- A phase-transition arbiter (`select_phase`).
- A cross-range helper (`crossrange_km`, `pub(crate)`).
- The conversion constant `NM_TO_KM`.
- The private GLIMITER deceleration limiter (`glimiter_ld`), shared by every step function.

### What this module does NOT provide

- The program state machine (`EntryPhase` enum), `EntryState` struct, or the DSKY wiring — all of those live in `programs/p61_p67.rs`.
- Sensed-acceleration integration — `sensed_acceleration_g` is updated upstream by the SERVICER before any function here is called.
- DAP actuation — functions return commands; the SERVICER copies them to `state.dap_state.mode = DapMode::EntryRoll(roll_command_rad)`.
- Target-site navigation (range-to-go updates) — `p61_p67.rs` calls into `compute_range_to_go_km` and stores results in `state.entry` before calling guidance functions.
- Skip-out P65 state machine wiring (the `select_phase` function decides the transition; the SERVICER in `p61_p67.rs` applies it).

---

## §2 AGC Background

The Rust functions translate specific named blocks in `Comanche055/REENTRY_CONTROL.agc`. The mapping is:

| Rust function | AGC block / lines | Program |
|---|---|---|
| `huntest_setup` (private) | `HUNTEST` setup, lines 500–649 | P64 |
| `predict_range` (RANGER) | `RANGER`, lines 654–732 | P64 |
| `predict_range_table` | `PREDICT3`, lines 1369–1467 | P67 (oracle) |
| `predicted_exit_velocity_mps` | Extracts `VL` from `huntest_setup` | P64 |
| `compute_ld_command` | Newton step lines 744–860; `FOREHUNT` line 861 | P64 |
| `resolve_roll` | `L355` block, line 1308; lateral-switch `L353`, line 1296 | P64 |
| `upcontrol_step` | `UPCNTRL3`, lines 882–1091 | P65 |
| `constd_step` | `DCONSTD` / `CONSTD1`, lines 1023–1059 | P65/P64 diverged |
| `constd_dref_agc` | Bare `D0 = KA3·LEQ + KA4` from lines 441–447 | Utility |
| `ballistic_step` | `KEP` block, line 1098 | P66 |
| `final_phase_step` | Final-phase `PREDICT3` law, lines 1139–1235 | P67 |
| `glimiter_ld` (private) | `GLIMITER`, lines 1247–1267 | Every phase |
| `select_phase` | Transition tests lines 431–734, 895, 902, 1023 | All |
| `crossrange_km` | Small-angle approximation replacing `LATANG` derivation | P64 |

Key AGC variable names preserved in Rust field/constant names: `LEWD`, `DLEWD`, `DIFFOLD`, `GAMMAL`, `GAMMAL1`, `DHOOK`, `AHOOKDV`, `FACTOR`, `VBARS`, `ALP`, `FACT1`, `FACT2`, `VL`, `VREF`, `RDOTREF`, `PREDANG`, `RTOGO`, `THETAH`.

---

## §3 Rust API

### 3.1 Public types

```rust
/// Result of one HUNTEST or phase-step iteration.
/// Pure return value; SERVICER copies fields into EntryState.
#[derive(Clone, Copy, Debug)]
pub struct LdUpdate {
    pub ld_command: f64,      // Vertical L/D command, saturated to [-LAD, LAD]
    pub lewd_new: f64,        // Updated LEWD reference
    pub dlewd_new: f64,       // Updated DLEWD step
    pub diffold_new_km: f64,  // Current DIFF, becomes DIFFOLD next cycle
    pub factor_new: f64,      // SKIPPER FACTOR (F1) — pass-through on non-SKIPPER paths
}
```

### 3.2 Public constant

```rust
pub const NM_TO_KM: f64 = 1.852;  // Exact; AGC tables are in nautical miles
```

### 3.3 Public functions grouped by phase

#### Pre-entry / prediction

```rust
pub fn predict_range(state: &AgcState) -> f64
```
Predicted total range (km) from the current inertial state to the point where deceleration peaks at `VFINAL`. Implements RANGER (analytic) with fallback to PREDICT3 (tabulated) on any degenerate intermediate. Always returns a finite value.

```rust
pub fn predicted_exit_velocity_mps(state: &AgcState) -> f64
```
Returns `VL` (m/s) from the current HUNTEST setup. Convenience accessor for the SERVICER display (`VPRED` / V16N63 R2). Returns 0.0 before initialisation or on a degenerate setup.

#### HUNTEST / INITROLL (P64)

```rust
pub fn compute_ld_command(state: &AgcState) -> LdUpdate
```
One Newton iteration on `LEWD`. On the first cycle (`hunt_initialized == false`) initialises from `FOREHUNT` constants. Applies LEWDPTR safety clamp, then LIMITL/D and GLIMITER. Does not modify `state`; returns an `LdUpdate` for the SERVICER to copy back.

```rust
pub fn resolve_roll(state: &AgcState, ld_cmd: f64) -> f64
```
Converts a vertical L/D command to a bank angle (rad). Magnitude = `acos(ld_cmd / LAD)`; sign from `crossrange_km` (positive cross-range → negative bank, i.e., bank left). A hysteresis dead-band `LD_CMIN_RATIO` prevents chatter near the track centreline.

#### Up-control (P65 UPCONTRL / SKIPPER)

```rust
pub fn upcontrol_step(state: &AgcState) -> LdUpdate
```
One P65 SKIPPER iteration. Four internal branches in priority order:
1. `D < Q7F_G` — freeze L/D (above sensible atmosphere; AGC `KEP` branch).
2. `V > V₁` — DOWNCNTL (see below).
3. `D > A0` or `D > C20_G` — full lift-up `L/D = LAD`.
4. Nominal SKIPPER law with nonlinear `FACTOR` gain.

LEWD is held frozen during UPCONTRL; `dlewd_new` carries the current-cycle `ΔL/D`.

#### Constant-drag fallback (CONSTD)

```rust
pub fn constd_step(state: &AgcState) -> LdUpdate
```
One CONSTD iteration. Builds reference drag `D0 = KA3·LEQ + KA4`, computes `RDOTREF = -2·HS·D0/V`, then evaluates `L/D = LEQ·C/D0/256 + K2D·(RDOT−RDOTREF) + K1D·(D−DREF)`. Does not iterate LEWD or touch FACTOR — both are passed through unchanged so HUNTEST can resume when `select_phase` returns to Skip.

```rust
pub(crate) fn constd_dref_agc(v_mps: f64, rdot_mps: f64) -> f64
```
Returns the bare `LEQ·C/D0` component (AGC-normalised drag units). Unit-test utility / math lock-in. Not wired into the phase dispatcher; `constd_step` absorbs the full formula.

#### P66 Ballistic hold

```rust
pub fn ballistic_step(state: &AgcState) -> LdUpdate
```
Returns the previous cycle's `ld_command`, `lewd_ref`, and `diffold_km` unchanged; sets `dlewd_new = 0`. The DAP holds the last bank command. No automatic return to closed-loop from Ballistic — only `select_phase`'s global `V < VFINAL1` terminal check fires.

#### P67 Final phase (PREDICT3)

```rust
pub fn final_phase_step(state: &AgcState) -> LdUpdate
```
Linearly interpolates the reference profile at the current velocity to get `RTOGO`, `AREF`, `RDOTREF`, `F1 (drange_da)`, `F2 (drange_drdot)`, and `Y (drange_dld)`. Computes:
```
PREDANG = RTOGO + F1·(D − AREF) + F2·(RDOT − RDOTREF)
L/D = LOD + (THETAH − PREDANG) / Y
```
Saturates to `±LAD`, applies GLIMITER. LEWD is frozen from the HUNTEST/UPCONTRL phase.

#### Phase transition

```rust
pub fn select_phase(state: &AgcState) -> Option<EntryPhase>
```
Evaluates transition tests against the current velocity and range error. Returns `Some(next_phase)` or `None` (stay). See §4.6 for the full decision table.

#### Cross-range helper

```rust
pub(crate) fn crossrange_km(state: &AgcState) -> f64
```
Signed cross-range (km) from sub-satellite point to the great-circle through the target. Uses small-angle approximation `R_EARTH · sin(Δlon) · cos(target_lat)`; sign positive = right of northward direction to target. Returns 0.0 if the position vector is zero.

---

## §4 Functional Requirements

### 4.1 `predict_range` — HUNTEST + RANGER

**Inputs read from `state`**: `csm_state.velocity` (3-vector m/s), `entry.r_dot_mps`, `entry.sensed_acceleration_g`, `entry.hunt_initialized`, `entry.lewd_ref`, `entry.ld_command`.

**Computation**:
1. Calls private `huntest_setup(state) -> Option<HuntestSetup>`. If `None`, delegates immediately to `predict_range_table`.
2. Computes AGC-normalised intermediates: `v_norm = V / (2·VSAT)`, `rdot_norm = RDOT / (2·VSAT)`.
3. **ASKEP** component (Keplerian-arc range):
   ```
   COSG/2 = (1 − GAMMAL²) / 2
   E/4    = sqrt((VBARS − 0.5)·VBARS·(COSG/2)²·4 + 1/16)
   ASKEP  = asin(VBARS·(COSG/2)·GAMMAL / (E/4)) / π   [revolutions]
   ```
   Returns table fallback if `bracket ≤ 0` or `E/4 < 0.05`.
4. **ASP1** (final-phase range, linear in `VL`):
   ```
   Q2 = LAD·Q21 + Q22
   ASP1 = Q2 + Q3·VL_norm   [revolutions]
   ```
5. **ASPUP** (up-phase range, logarithmic):
   ```
   ASPUP = -C12 · log(|V1²·Q7F / (VBARS·A0)| ∨ 1e-12) / GAMMAL1
   ```
   Returns table fallback if `|ASPUP| > 0.2 rev`.
6. **ASPDWN** (pull-out range):
   ```
   ASPDWN = KC3 · RDOT_norm · V_norm / A0_agc / LAD
   ```
7. **ASP3** (γ-correction):
   ```
   ASP3 = Q5 · (Q6 − GAMMAL)
   ```
8. `ASP = ASKEP + ASP1 + ASPUP + ASP3 + ASPDWN`, converted: `km = ASP · 2π · R_EARTH · 1e-3`.
9. Returns table fallback if `!asp_km.is_finite()` or `asp_km` outside `[0, 100 000]`.

All normalisations use `VSAT_MPS` and `FPSS_805_MPS2` verbatim from the AGC constants. This guarantees the dimensionless coefficient values in `entry_tables.rs` apply directly.

### 4.2 `huntest_setup` — Private HUNTEST Intermediate Computation

REENTRY_CONTROL.agc lines 500–649.

**Guards that return `None`** (any degenerate state aborts the setup):
- `v_norm < 1e-6`
- `tem1b` (= `LAD` if descending, `LEWD` if ascending) < 1e-6
- `|1 - ALP| < 1e-6`
- `|A0_agc| < 1e-6`
- `Q7F·FACT2 + ALP < 0` (would produce imaginary VL)
- `VL_mps < VLMIN_MPS` or `|VL_norm| < 1e-6`
- `|GAMMAL1| < 1e-9`
- `|DHOOK| < 1e-12`

**V₁ lead reduction**: if `ld_command < 0` (last cycle commanded lift-down), `V1_norm` is reduced by `VQUIT / (2·VSAT)` before ALP and FACT1 are computed. Mirrors the AGC `VQUIT` lead at REENTRY_CONTROL.agc:545.

**DHOOK / AHOOKDV / GAMMAL second-order correction** (lines 616–640):
```
DHOOK       = ((1 − V1/FACT1)² − ALP) / FACT2     [= A0_agc algebraically]
AHOOKDV     = DHOOK / (64·Q7F) − CHOOK
DVL         = V1 − VL
correction  = (AHOOKDV + 1/16) · CH1 · DVL² / (DHOOK · VBARS)
GAMMAL      = max(0, GAMMAL1 − correction)          [BMN NEGAMA clamp]
```

**A1 selection** (lines 502 + 532–535): `A1 = D_agc` when `RDOT < 0` (descending); `A1 = A0_agc` when `RDOT ≥ 0` (climbing / skip-out). Used by SKIPPER `FACTOR` gain in `upcontrol_step`.

### 4.3 `predict_range_table` — PREDICT3 Fallback

Formula: `ASP = RTOGO(V) + (LEWD − LAD) · drange_dld(V)`, all in nm, converted to km.

`lookup_reference(v)` linearly interpolates the 13-sample table; saturates at the velocity bounds (no extrapolation).

This function is `pub(crate)` — accessible by tests and by `p61_p67.rs`, but not part of the external API. It is the reliable baseline when the analytic RANGER block leaves its valid regime.

### 4.4 `compute_ld_command` — HUNTEST Newton Iteration

**FOREHUNT initialisation** (first cycle, `hunt_initialized == false`):
```
LEWD_prev  = LEWD_INIT   (0.15)
DLEWD_prev = DLEWD_INIT  (-0.05)
DIFFOLD    = 0
```

**Newton step** (REENTRY_CONTROL.agc lines 744–760):
```
DIFF  = target_range_km − predicted_range_km
denom = DIFFOLD − DIFF
ΔLEWD = if |denom| > 1e-6:  DLEWD_prev · DIFF / denom
        else:                DLEWD_prev            [BOV overflow guard]
```

**LEWDPTR clamp** (line 797):
```
if LEWD_prev + ΔLEWD < 0:  ΔLEWD = -LEWD_prev / 2
```

**Saturation + GLIMITER**:
```
ld_command = glimiter_ld(D, RDOT, V, clamp(LEWD_prev + ΔLEWD, -LAD, LAD))
```

`FACTOR` is carried through unchanged — HUNTEST does not touch it.

### 4.5 `resolve_roll` — Bank Command

```
magnitude = acos(clamp(ld_cmd / LAD, -1, +1))
```

**Lateral switch** (line 1296): the sign of the bank command is chosen to reduce cross-range error. Dead-band threshold = `LD_CMIN_RATIO · R_EARTH_km · 1e-3` km (≈ 0.0061 km). Inside the band, bank sign stays positive (AGC trim-attitude default).

Convention: `crossrange_km > 0` (right of track) → negative bank (bank left). `crossrange_km < 0` (left of track) → positive bank (bank right).

### 4.6 `select_phase` — Phase Transition Logic

Global terminal-velocity check (applied to every phase before the per-phase logic):
```
V < VFINAL1_MPS → Some(Final)
```

Per-phase logic:

| From | Condition | To |
|---|---|---|
| Entry | `\|DIFF\| > RANGE_ERR_THRESHOLD_KM` (500 km) | Constd |
| Entry | `\|DIFF\| < HUNTEST_CONVERGED_KM` (46.3 km) | Skip |
| Entry | otherwise | None |
| Constd | `\|DIFF\| < HUNTEST_CONVERGED_KM` | Skip |
| Constd | otherwise | None |
| Skip | `V − VL < C18_MPS` (V − VL < 152.4 m/s) | Final |
| Skip | `D < Q7F_G` (0.186 g) | Ballistic |
| Skip | `\|DIFF\| > RANGE_ERR_THRESHOLD_KM` | Constd |
| Skip | otherwise | None |
| Ballistic | (global V check only) | None |

Note: CONSTD does not transition to Ballistic on low drag. Allowing that would freeze the closed loop and dramatically overshoot peak deceleration.

### 4.7 `upcontrol_step` — SKIPPER (P65)

Four branches in evaluation order:

1. **Freeze** (`D < Q7F_G`): return previous `ld_command`, `lewd_ref`, `diffold_km`. AGC `KEP`, line 895.
2. **DOWNCNTL** (`V > V₁`):
   ```
   v1_minus_v  = V1 − V   (negative since V > V₁)
   RDTR        = LAD · v1_minus_v
   ld_cand     = LAD + K2D · (RDOT_norm − RDTR)
   DREF        = (V/V₁)²·A0 − (V₁−V)²·LAD / (2·C1·HS)
   L/D         = clamp(ld_cand + K1D·(D − DREF), -LAD, LAD)
   ```
   Then GLIMITER.
3. **Full lift-up** (`D > A0_g` or `D > C20_G`): `L/D = LAD`.
4. **Nominal SKIPPER**:
   ```
   VREF    = FACT1 · (1 − sqrt(FACT2·D + ALP))
   RDOTREF = LEWD · (V1 − VREF)
   FACTOR  = if D > Q7MIN_G: (A1 − Q7F) / (D − Q7F)  else: factor_prev
   ΔL/D    = -((RDOT − RDOTREF)·F1/KB1 + V − VREF)·F1/KB2
   ```
   Nonlinear gain compression (lines 989–998): if `|ΔL/D| > PT1_OVER_16`:
   ```
   ΔL/D_compressed = POINT1·|ΔL/D| + PT1_OVER_16   [with sign of ΔL/D]
   ```
   Then `L/D = clamp(LEWD + ΔL/D, -LAD, LAD)` and GLIMITER.

LEWD is not updated in UPCONTRL — the returned `lewd_new` equals `lewd_prev`.

### 4.8 `constd_step` — Constant-Drag Closed Loop (CONSTD)

```
V_norm  = V / (2·VSAT)
LEQ     = 4·V_norm² − 1
D0      = KA3·LEQ + KA4
C/D0    = -4 / D0
RDOTREF = -TWO_HS·D0 / V_norm
L/D     = LEQ·C/D0/256 + K2D·(RDOT_norm − RDOTREF_norm) + K1D·(D_agc − D0)
```

The `/256` on the `LEQ·C/D0` term reproduces the AGC's post-`SL 8D` B-scaling where `K1D_AGC` and `K2D_AGC` already absorb the `×256` factor (see constant table, §6).

Returns frozen `LdUpdate` if `|D0| < 1e-9`.

### 4.9 `ballistic_step` — P66 Attitude Hold

Returns an `LdUpdate` that copies `ld_command`, `lewd_ref`, and `diffold_km` from state, with `dlewd_new = 0` and `factor_new` unchanged. No range prediction is performed.

### 4.10 `final_phase_step` — P67 PREDICT3 Terminal Law

Calls `lookup_reference(V)` to get the interpolated `ReferencePoint`. Computes:
```
PREDANG = p.range_to_go_nm + p.drange_da_nm_per_g · (D_g + p.neg_aref_g)
                            + p.drange_drdot_nm_per_mps · (RDOT − p.rdot_ref_mps)
L/D     = LOD + (THETAH_nm − PREDANG) / p.drange_dld_nm
```
`THETAH_nm = target_range_km / NM_TO_KM`. Saturates to `±LAD`, applies GLIMITER. LEWD, DLEWD, DIFFOLD, and FACTOR are all carried through unchanged.

### 4.11 `glimiter_ld` — GLIMITER Deceleration Limiter (Private)

Three-regime clip (REENTRY_CONTROL.agc:1247–1267):
```
D ≤ GMAX/2 (4 g):   pass through
D >  GMAX  (8 g):   return LAD
else:
  XLIM = sqrt(2HS·(GMAX−D)·(LEQ_stored/GMAX + LAD) + 2HSGMXSQ / VSQUARE)
  if RDOT_norm + XLIM ≥ 0:  pass through
  else:                       return LAD
```
If the XLIM argument goes negative (low V, pathological LEQ), conservatively returns LAD.

`LEQ_stored = (VSQUARE − 1) / 4` and the comparison `1/GMAX_stored = 0.5` reproduce the AGC's fixed-point form exactly.

---

## §5 Constants Reference

### From `entry_tables.rs` (all `pub`)

| Constant | Value | AGC label | Meaning |
|---|---|---|---|
| `LAD_NOMINAL` | 0.30 | `LADPAD` | Maximum vehicle L/D |
| `LOD_NOMINAL` | 0.18 | `LODPAD` | Final-phase nominal L/D |
| `LEWD_INIT` | 0.15 | `LEWD1` | HUNTEST initial reference L/D |
| `DLEWD_INIT` | -0.05 | `DLEWD0` | HUNTEST initial iteration step |
| `LD_CMIN_RATIO` | `LAD · cos(15°) ≈ 0.2895` | `L/DCMINR` | Lateral-switch hysteresis L/D ratio |
| `VSAT_MPS` | 7 853.516 8 | `VSAT` | Circular-orbit reference velocity |
| `VFINAL1_MPS` | `≈ 8 229.6` | `VFINAL1` | Terminal-phase entry velocity threshold |
| `VFINAL_MPS` | `≈ 8 101.0` | `VFINAL` | PREDICT3 table lower bound |
| `VLMIN_MPS` | `≈ 5 482.5` | `VLMIN` | Minimum VL for valid HUNTEST |
| `VQUIT_MPS` | `≈ 304.6` | `VQUIT` | V₁ lead reduction for lift-down |
| `TWO_C1_HS_AGC` | 0.021 598 326 4 | `2C1HS` | Drag-normalisation product |
| `Q7F_AGC` | 0.007 453 416 1 | `Q7F` | Minimum drag for HUNTEST (AGC units) |
| `Q7F_G` | `Q7F_AGC × 25` | — | Q7F in g |
| `Q7MIN_G` | `40/805 × 25 ≈ 1.242 g` | `Q7MIN` | Threshold for FACTOR update in UPCONTRL |
| `CHOOK` | `1/64` | `CHOOK` | AHOOKDV correction base |
| `CH1` | 0.64 | `CH1` | GAMMAL correction scale |
| `ONE_SIXTEENTH` | 0.0625 | `1/16TH` | Additive term in GAMMAL correction |
| `AHOOKDV_DIVISOR` | 64.0 | `SR 6` | Divisor in AHOOKDV = DHOOK / (64·Q7F) |
| `FPSS_805_MPS2` | `25 × G_AGC_MPS2` | `805 FPSS` | AGC drag scale factor (m/s²) |
| `GMAX_HALF_G` | 4.0 | `GMAX/2` | Lower GLIMITER threshold |
| `GMAX_G` | 8.0 | `GMAX` | Upper GLIMITER threshold |
| `TWO_HS_AGC` | 0.017 278 661 1 | `2HS` | Atmospheric scale-height product |
| `TWO_HS_GMAX_SQ_AGC` | 0.000 030 571 7 | `2HSGMXSQ` | GLIMITER altitude-margin constant |
| `RANGE_ERR_THRESHOLD_KM` | 500.0 | (design choice) | HUNTEST divergence threshold → Constd |
| `HUNTEST_CONVERGED_KM` | `25 × 1.852 ≈ 46.3` | `25NM` | HUNTEST convergence threshold → Skip |
| `C18_MPS` | `500 × 0.3048 ≈ 152.4` | `C18` | PREFINAL velocity margin (V − VL) |
| `C20_G` | `175/805 × 25 ≈ 5.43 g` | `C20` | Max-lift-up drag threshold |
| `C21_G` | `140/805 × 25 ≈ 4.35 g` | `C21` | Lateral-switch suppression threshold |
| `LAT_BIAS_RAD` | `≈ 1.88e-4 rad` | `LATBIAS` | Half-NM dead-band |
| `KB1` | 3.4 | `1/KB1` | SKIPPER position-gain divisor |
| `KB2_MPS` | `0.0034 × 2·VSAT ≈ 53.4 m/s` | `1/KB2` | SKIPPER velocity-gain divisor |
| `PT1_OVER_16` | 0.006 25 | `PT1/16` | SKIPPER gain-reduction onset |
| `POINT1` | 0.1 | `POINT1` | SKIPPER compression slope |
| `Q21_AGC` | `500/21 600` | `Q21` | Final-phase range LAD coefficient |
| `Q22_AGC` | `-1152/21 600` | `Q22` | Final-phase range constant |
| `Q3_AGC` | 0.167 003 132 | `Q3` | Final-phase VL slope |
| `Q5_AGC` | 0.326 388 889 | `Q5` | γ-correction range coefficient |
| `Q6_RAD` | 0.034 9 | `Q6` | γ-correction zero offset (≈ 2°) |
| `KC3_AGC` | -0.024 762 223 2 | `KC3` | Pull-out range scaling (AGC-native) |
| `C12_AGC` | 0.006 845 729 01 | `C12` | Up-range log coefficient |
| `K1D_AGC` | 8.05 | `K1D` | DOWNCNTL/CONSTD drag-error gain (post-SL8) |
| `K2D_AGC` | -51.532 395 008 | `K2D` | DOWNCNTL/CONSTD RDOT-error gain (post-SL8) |
| `KA3_AGC` | 0.447 204 97 | `KA3` | D0 equilibrium-drag slope |
| `KA4_AGC` | 0.049 689 441 | `KA4` | D0 equilibrium-drag floor |

### From `entry.rs`

| Constant | Value | Meaning |
|---|---|---|
| `NM_TO_KM` | 1.852 | Unit conversion; exact |

---

## §6 Numerical Notes

### 6.1 NaN and non-finite policy

Every public function guarantees a finite return value. `predict_range` has five explicit fallback-to-table guards:
1. `huntest_setup` returns `None` on any degenerate intermediate.
2. `bracket ≤ 0` before the `sqrt` in ASKEP.
3. `E/4 < 0.05` (ASKEP saturation regime).
4. `|ASPUP| > 0.2 rev` (log term leaves valid range).
5. `!asp_km.is_finite()` or out of `[0, 100 000]` km.

`glimiter_ld` guards `inner < 0.0` before `sqrt` (conservative: return LAD). `constd_step` guards `|D0| < 1e-9` (return frozen LdUpdate). `final_phase_step` guards `|drange_dld| < 1e-9` (return LOD_NOMINAL).

### 6.2 Test tolerances

Tolerances used in the test suite, establishing the effective numerical accuracy requirements:

| Function family | Tolerance |
|---|---|
| `predict_range` absolute value | ±1 km (band-check); exact identity where fallback fires |
| `compute_ld_command` Newton step | 1e-9 (exact f64 arithmetic expected for the secant formula) |
| `resolve_roll` bank angle | 1e-9 rad |
| SKIPPER `FACTOR` gain | ±0.05 |
| `final_phase_step` nominal output | ±0.02 in L/D |
| DHOOK / GAMMAL correction | 5e-5 |

### 6.3 Velocity normalisation convention

All AGC-native constants use `V / (2·VSAT_MPS)` normalisation. The 2·VSAT factor appears in: ASKEP, ASPUP, ASPDWN, ASP1, SKIPPER `VREF` / `RDOTREF`, CONSTD `RDOTREF`, DOWNCNTL, GLIMITER, and `crossrange_km`. Any future change to `VSAT_MPS` must propagate to all of these simultaneously.

### 6.4 CONSTD K1D / K2D scaling

The AGC stores `K1D` and `K2D` as pre-shift values (before the `SL 8D` at line 1057). The Rust constants `K1D_AGC = 8.05` and `K2D_AGC = -51.532` absorb that `×256` factor. The `LEQ·C/D0` term in `constd_step` is explicitly divided by 256 to match the AGC's stored-scale bias. This is a deliberate asymmetry: the two feedback terms (`K1D·ΔD`, `K2D·ΔRDOT`) are physical-scale; the bias term is raw-scale.

### 6.5 KC3 unit history

Prior to issue #42, `KC3` was stored as an SI-converted value (`-0.619 nm/(m²/s²)`) that incorrectly applied SI units directly without the AGC's `(2·VSAT)²` velocity normalisation and `FPSS_805` drag normalisation, inflating `ASPDWN` by a factor of ~1163. The current `KC3_AGC = -0.024 762 223 2` is the AGC-literal value; all normalisations are applied at the call site in `predict_range`. The regression test `tc_42_aspdwn_regression` pins the corrected result at ≈ 1773 km for the reference inputs.

### 6.6 `predict_range` analytic vs. table regime

The analytic RANGER block is only valid in the AGC's designed skip-out regime: `VBARS ≳ 0.5`, `log(V1²·Q7F / (VBARS·A0)) ≈ 0`, and `GAMMAL1` in the few-hundredths-of-a-radian band. For all sub-VFINAL1 / nominal descent inputs the three guards (`bracket`, `E/4`, `|ASPUP|`) route to the table. Test `tc_mse3a_pr_1` confirms the ratio is 1.000 across this regime, asserting fallback consistency rather than RANGER-vs-PREDICT3 numerical agreement. Closing the analytic-vs-table gap for arbitrary inputs is deferred (tracked as a follow-up to issue #10).

---

## §7 Dependencies

### From `agc-core`

| Dependency | Symbol used | Purpose |
|---|---|---|
| `agc-core::AgcState` | whole struct | State access (read-only) |
| `agc-core::programs::p61_p67` | `EntryPhase`, `G0_MPS2` | Phase enum, standard g |
| `agc-core::programs::p21` | `R_EARTH` | Earth mean radius (m) |
| `agc-core::navigation::state_vector` | `inertial_to_earth_fixed` | ECEF position for crossrange |
| `agc-core::navigation::time` | `met_to_gha` | GHA epoch for ECEF rotation |
| `agc-core::guidance::entry_tables` | All constants + `lookup_reference` | See §5 |

### External crates

| Crate | Usage |
|---|---|
| `libm` | `sqrt`, `asin`, `acos`, `atan2`, `log`, `sin`, `cos` — no_std-compatible math |
| `core::f64::consts` | `PI`, `FRAC_PI_2` |

### Sibling data module

`agc-core/src/guidance/entry_tables.rs` holds:
- All named constants (§5).
- `ReferencePoint` struct.
- `REFERENCE_PROFILE: [ReferencePoint; 13]` — 13-sample AGC profile (REENTRY_CONTROL.agc:1369–1467).
- `lookup_reference(v_mps) -> ReferencePoint` — linear interpolation with endpoint clamping.

---

## §8 Test Cases

All tests live in `agc-core/src/guidance/entry.rs::tests`. The fixture function `fixture(v_mps)` builds a canonical `AgcState` with CSM at 6 500 km radius on +X, velocity along +Y, target range 1 500 km, and phase `Entry`.

### predict_range (TC-MSE3-PR-*)

| ID | Input | Expected |
|---|---|---|
| PR-1 | V ≈ 303 m/s (VREFER[0]) | range ≤ 20 km |
| PR-2 | V = 10 000 m/s | 500–2 500 km |
| PR-3 | Sweep V_lo → V_hi (10 steps) | range monotone non-decreasing |
| PR-4 | V = 7 500 m/s, LEWD 0.10 vs 0.25 | higher LEWD → longer range |

**Regression tests** (TC-42-*):
- `tc_42_aspdwn_regression`: ASPDWN at (v=9 km/s, ṙ=−200 m/s, A0=0.01357, LAD=0.30) ≈ 1 773 km ± 100 km.
- `tc_42_asp1_regression`: ASP1 at (LAD=0.30, VL=7 644 m/s) ≈ 1 395 km ± 50 km.

**Additional fallback / NaN tests** (TC-MSE3A-PR-*):
- PR-A1: analytic vs. table ratio = 1.000 at VFINAL1+200, 9 000, 10 000 m/s (fallback consistency).
- PR-A2: below VLMIN, analytic delegates exactly to table.
- PR-A3: no NaN for pathological `(v, lewd)` combinations.

### DHOOK / GAMMAL (TC-MSE3B-DHOOK-*)

| ID | Input | Expected |
|---|---|---|
| DHOOK-1 | v=10 km/s, ṙ=−50 m/s, D=0.2 g | A0≈0.009242, GAMMAL1≈0.003450, GAMMAL≈0.001025 |
| DHOOK-2 | v=9 km/s, ṙ=−200 m/s, D=0.5 g | GAMMAL clamped to 0.0 (BMN NEGAMA) |

### compute_ld_command (TC-MSE3-LD-*)

| ID | Scenario | Expected |
|---|---|---|
| LD-1 | First cycle (FOREHUNT) | lewd_new = 0.20, diffold = 50 km |
| LD-2 | Known (DIFF=40, DIFFOLD=100, DLEWD=0.05) | ΔLEWD = 0.05·40/60 |
| LD-3 | Large positive step → lewd_new_raw > LAD | ld_command = LAD_NOMINAL |
| LD-4 | Huge negative step → LEWD + ΔLEWD < 0 | ΔLEWD = -LEWD/2, lewd_new = 0.05 |

### resolve_roll (TC-MSE3-RR-*)

| ID | Input | Expected |
|---|---|---|
| RR-1 | L/D = LAD | bank = 0 |
| RR-2 | L/D = 0 | \|bank\| = π/2 |
| RR-3 | L/D = -LAD | \|bank\| = π |
| RR-4 | crossrange > 0 (right of track) | bank < 0; crossrange < 0 → bank > 0 |

### select_phase (TC-MSE3-SP-*, TC-MSE4-SP-*, TC-MSE5-SP-*, TC-MSE86-SP-*)

| ID | From | Condition | Expected |
|---|---|---|---|
| SP-1 | Entry | V < VFINAL1 | Some(Final) |
| SP-2 | Entry | \|DIFF\| = 200 km (between thresholds) | None |
| SP-3 | Entry | \|DIFF\| = 1 500 km | Some(Constd) |
| MSE4-SP-1 | Entry | \|DIFF\| < HUNTEST_CONVERGED_KM / 2 | Some(Skip) |
| MSE4-SP-2 | Skip | nominal (D=0.5g, DIFF=200 km) | None |
| MSE4-SP-3 | Skip | \|DIFF\| = 1 500 km | Some(Constd) |
| MSE5-SP-1 | Skip | D = 0.10 g < Q7F_G | Some(Ballistic) |
| MSE5-SP-2 | Ballistic | D=0.5g, DIFF=0 | None |
| MSE5-SP-3 | Ballistic | V < VFINAL1 | Some(Final) |
| MSE86-SP-1 | Constd | \|DIFF\| = 1 500 km | None |
| MSE86-SP-2 | Constd | \|DIFF\| = 0 | Some(Skip) |
| MSE86-SP-3 | Constd | D < Q7F_G, \|DIFF\| large | None (no Ballistic exit) |
| MSE86-SP-4 | Constd | V < VFINAL1 | Some(Final) |

### crossrange (TC-MSE3-CR-*)

| ID | Scenario | Expected |
|---|---|---|
| CR-1 | CSM at target lon | crossrange ≈ 0 |
| CR-2 | CSM 1° east of target | crossrange ≈ 111 km |

### upcontrol_step / SKIPPER (TC-MSE4-UC-*, TC-MSE4B-*)

| ID | Scenario | Expected |
|---|---|---|
| UC-1 | D = Q7F_G/2 | ld_command frozen, lewd_new unchanged |
| UC-2 | D = C20_G × 2 | ld_command = LAD |
| UC-3 | Steep descent (RDOT=−300 m/s), D=0.5g | ΔL/D ≠ 0, \|ld_command\| ≤ LAD |
| UC-4 | Massive descent (RDOT=−5000 m/s) | \|ld_command\| ≤ LAD (saturated) |
| F1-1 | Climbing (RDOT=+50), D=1.5g | F1 ≈ 1.140 (amplified gain) |
| F1-2 | Descending (RDOT=−100), D=1.5g | F1 = 1.0 (unity, A1=D) |
| F1-3 | D=0.5g < Q7MIN_G | FACTOR frozen at previous value (2.5) |
| DOWNCNTL-1 | V > V₁, D=1g | ld_command ≈ 0.180 |
| CONSTD-1 | `constd_dref_agc` at V=10 km/s | bare DREF ≈ -7.59 (AGC units) |

### constd_step (TC-MSE86-CS-*)

| ID | Scenario | Expected |
|---|---|---|
| CS-1 | D=1.5g, RDOT=−300 m/s | \|ld_command\| ≤ LAD |
| CS-2 | lewd_ref=0.123, factor=1.7 | both passed through unchanged |

### ballistic_step (TC-MSE5-BS-*)

| ID | Scenario | Expected |
|---|---|---|
| BS-1 | frozen state | all fields identical to input; dlewd = 0 |

### final_phase_step (TC-MSE6-FP-*, TC-MSE6B-*)

| ID | Scenario | Expected |
|---|---|---|
| FP-1 | V below VFINAL1 | \|ld_command\| ≤ LAD |
| FP-2 | D=AREF, RDOT=RDOTREF, target=RTOGO | ld_command = LOD_NOMINAL |
| FP-3 | target > RTOGO → need more lift | ld_command > LOD; target < RTOGO → ld_command < LOD |
| PREDANG-1 | i=10 sample, D=5g, RDOT=−100 m/s | ld_command ≈ LOD_NOMINAL (PREDANG ≈ THETAH) |
| GLIMITER-1 | D=1g (below 4g) | L/D passes through |
| GLIMITER-2 | D=9g (above 8g) | L/D = LAD |
| GLIMITER-3 | D=5g, RDOT=−500 m/s | L/D = LAD (XLIM branch) |

---

## §9 Spec Quality Checklist

- [x] Module path and AGC counterpart identified
- [x] Scope boundary drawn (vs. `p61_p67.rs` orchestration)
- [x] All `pub` and `pub(crate)` functions documented with signature, inputs, and computation
- [x] Private `huntest_setup`, `glimiter_ld`, `downcntl_ld` described at functional level
- [x] Every AGC block cross-referenced by line number
- [x] All constants in `entry_tables.rs` listed with AGC label and value
- [x] All degenerate-input guards documented (§6.1)
- [x] Unit normalisation convention (2·VSAT) explicitly stated (§6.3)
- [x] KC3 scaling fix and regression test documented (§6.5)
- [x] CONSTD K1D/K2D ×256 asymmetry documented (§6.4)
- [x] Analytic-vs-table regime documented, gap flagged as open (§6.6)
- [x] All ~55 named tests listed in §8 with inputs and expected outcomes
- [x] Phase-transition decision table complete (§4.6)
- [x] Dependencies fully enumerated (§7)
- [x] No functions invented that are not present in the source file
- [x] No change log section (new spec)
