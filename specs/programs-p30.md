# Spec: P30 — External Delta-V Targeting

## AGC Source Reference

```
File:     Comanche055/P30-P37.agc
Routines: P30 (page 636), P30/P31 (shared entry, page 636),
          CNTNUP30 (page 636), PARAM30 (page 637), S30.1 (pages 639-640),
          REFTEST / FLASHMGA / DISPMGA (pages 637-638)
Pages:    635-641
```

Supporting routines:
```
File:     Comanche055/P30-P37.agc
Routine:  COMPTGO / CLOKTASK / DISP45 — countdown display timer
File:     Comanche055/CONIC_SUBROUTINES.agc
Routine:  THISPREC (page referenced in S30.1 as Encke propagation to TIG)
Routine:  PERIAPO1 (computes HAPO / HPER from state at TIG + DV)
```

## Behavior Summary

P30 is the External Delta-V Targeting Program. It is invoked when the crew (or an uplinked command) provides a planned maneuver defined by two numbers: a Time of Ignition (TIG) and a delta-V vector in the Local Vertical / Local Horizontal (LVLH) frame. P30 computes derived quantities for crew verification, stores a `BurnTarget` for P40 (the thrusting program), and then returns to P00 idle.

P30 does **not** execute the burn. It is purely a parameter acceptance and verification display program.

### Calling sequence (crew interaction)

1. Crew selects P30 via `V37 N30 ENTER`.
2. `P30/P31` shared entry: sets `UPDATFLG` and `TRACKFLG`, then flashes `V06 N33` (time of ignition).
3. Crew enters TIG (in hours, minutes, seconds) via DSKY ENTER.
4. `CNTNUP30`: flashes `V06 N81` (delta-V components in LVLH: X radial, Y along-track, Z out-of-plane).
5. Crew enters three delta-V components via DSKY ENTER.
6. Calls `S30.1` (the targeting subroutine).
7. `PARAM30`: displays `V06 N42` — apogee altitude (R1 NM), perigee altitude (R2 NM), delta-V magnitude (R3 fps), and then optionally MGA (maximum gimbal angle) if REFSMMAT is valid.
8. On PROCEED: calls REFTEST to optionally display MGA via `DISPMGA / FLASHMGA`.
9. Sets `XDELVFLG` (external delta-V flag, Bit 8 Flag 2) to signal P40 that the burn target is valid.
10. Calls `GOTOPOOH` — returns to P00 idle. The computed `BurnTarget` is held in erasable memory for P40 to consume.

### S30.1 — Targeting subroutine

S30.1 is the computational core of P30. Given `TIG` and `DELVSLV` (delta-V in LVLH):

1. **Propagate to TIG**: calls `THISPREC` (Encke precision propagator) with `TDEC1 = TIG` to compute `RTIG` (position at TIG) and `VTIG` (velocity at TIG). Scale: B+29 m for position, B+7 m/cs for velocity.
2. **Build LOMAT (local orientation matrix)**: constructs the LVLH-to-ECI rotation at TIG:
   - Col 0: `UNIT(RTIG)` (radial, LVLH +X)
   - The cross product `RTIG × VTIG` gives the orbit-normal.
   - Col 1: along-track direction.
   Stored internally as the matrix used by `LOMAT` (interpreting DELVSLV into reference coordinates).
3. **Rotate DELVSLV to ECI**: `DELVSIN = LOMAT × DELVSLV`. Scale: B+7 m/cs.
4. **Compute delta-V magnitude**: `VGDISP = |DELVSIN|`. Scale: B+7 m/cs.
5. **Compute post-burn state**: `VTIG_FINAL = VTIG + DELVSIN`.
6. **Compute conic parameters**: calls `PERIAPO1` with `(RTIG, VTIG_FINAL)` to find apogee (`HAPO`) and perigee (`HPER`) altitudes. Scale: B+29 m.

Outputs of S30.1: `RTIG`, `VTIG`, `HAPO`, `HPER`, `DELVSIN`, `VGDISP`.

### Display registers

| DSKY register | Noun | Content | Units |
|---|---|---|---|
| R1 of N33 | 33 | TIG (hours and minutes) | HH:MM.SS |
| R1 of N81 | 81 | Delta-V radial (DELVSLV X) | fps |
| R2 of N81 | 81 | Delta-V along-track (DELVSLV Y) | fps |
| R3 of N81 | 81 | Delta-V out-of-plane (DELVSLV Z) | fps |
| R1 of N42 | 42 | Apogee altitude (HAPO) | NM |
| R2 of N42 | 42 | Perigee altitude (HPER) | NM |
| R3 of N42 | 42 | Delta-V magnitude (VGDISP) | fps |

### Flags set/cleared

| Flag | Bit | Word | Action |
|---|---|---|---|
| `UPDATFLG` | Bit 7 | Flag 1 | Set on entry, cleared on P30 exit |
| `TRACKFLG` | Bit 5 | Flag 1 | Set on entry |
| `XDELVFLG` | Bit 8 | Flag 2 | Set on successful exit (signals P40) |

### Relationships to other programs

- **P40**: reads `TIG`, `DELVSIN`, `VGDISP`, and `XDELVFLG` to execute the burn. P30 must complete before P40 is invoked.
- **P31**: shares the `P30/P31` entry routine. P31 differs in that it uses Lambert targeting (`AGAIN` solver) for intercept guidance; P30 uses the simpler external-delta-V path.
- **V82**: can call S30.1 during P11 monitoring for splash-down error computation (DELRSPL subroutine, pages 643-645), but that is outside P30's scope.

## Rust API

Module path: `agc_core::programs::p30_ext_dv`

```rust
/// Enter P30 External Delta-V Targeting.
///
/// Records TIG and the crew-supplied delta-V vector in LVLH frame into
/// `state.burn_target`. Sets the UPDATFLG and TRACKFLG in state.flags.
/// Sets the restart phase for Group 4 (TC PHASCHNG / OCT 00014 in AGC).
///
/// This function does not perform computation — it stores inputs and
/// sets the restart protection phase. Call `compute_target()` next.
///
/// AGC source: Comanche055/P30-P37.agc P30/P31 common entry, page 636.
///
/// Inputs:
///   `tig`          — Time of Ignition as Mission Elapsed Time, centiseconds.
///   `delta_v_lvlh` — Desired delta-V in LVLH frame, m/s.
///                    Convention: [radial, along-track, out-of-plane].
pub fn enter(state: &mut AgcState, tig: Met, delta_v_lvlh: Vec3);

/// Compute the BurnTarget from stored TIG and DELVSLV.
///
/// Performs the S30.1 algorithm:
///   1. Propagates the current state vector to TIG via Kepler
///      (`math::kepler::kepler`), obtaining RTIG and VTIG.
///   2. Calls `guidance::targeting::predict_vg_at_ignition` to rotate
///      DELVSLV (LVLH) to DELVSIN (ECI) via the LOMAT construction.
///   3. Calls `guidance::targeting::burn_duration` with SPS constants
///      to compute TGO (time to go = burn duration).
///   4. Calls `navigation::conics::perigee_apogee` with (RTIG, VTIG + DELVSIN)
///      to compute HPER and HAPO.
///   5. Stores the complete `BurnTarget` into `state.burn_target`.
///   6. Sets XDELVFLG.
///
/// Returns the populated `BurnTarget`. Never fails for valid inputs;
/// alarm 1520 is raised (via `alarm::raise`) if REFSMMAT is not valid
/// and MGA display is requested.
///
/// AGC source: Comanche055/P30-P37.agc S30.1 (pages 639-640).
pub fn compute_target(state: &mut AgcState) -> BurnTarget;

/// Display the P30 computed summary on the DSKY (N42).
///
/// Renders:
///   R1 = HAPO apogee altitude in NM (state.hapo / 1852.0)
///   R2 = HPER perigee altitude in NM (state.hper / 1852.0)
///   R3 = delta-V magnitude in fps   (|DELVSIN| * 3.28084)
/// Issues V06 N42 monitor display.
///
/// Does not block; returns immediately after writing display registers.
///
/// AGC source: Comanche055/P30-P37.agc PARAM30 label, page 637.
pub fn display_summary(state: &AgcState, hw: &mut dyn AgcHardware);
```

### AgcState fields read / written

| Field | AGC register | Direction | Notes |
|---|---|---|---|
| `state.tig` | `TIG` | Read/Write | Time of ignition, centiseconds |
| `state.delvslv` | `DELVSLV` | Write | Crew-input delta-V in LVLH, m/s |
| `state.delvsin` | `DELVSIN` | Write | Rotated delta-V in ECI, m/s |
| `state.vgdisp` | `VGDISP` | Write | Delta-V magnitude, m/s |
| `state.rtig` | `RTIG` | Write | Position at TIG, m |
| `state.vtig` | `VTIG` | Write | Velocity at TIG, m/s |
| `state.hapo` | `HAPO` | Write | Apogee altitude, m |
| `state.hper` | `HPER` | Write | Perigee altitude, m |
| `state.burn_target` | (composite) | Write | Full BurnTarget for P40 |
| `state.flags.xdelvflg` | `XDELVFLG` | Write | Set when target is valid |
| `state.flags.updatflg` | `UPDATFLG` | Write | Set on entry |

### Scale factors

| Quantity | Rust unit | AGC scale | DSKY display unit |
|---|---|---|---|
| TIG | centiseconds (Met) | B+28 cs | HH:MM.SS |
| Delta-V LVLH / ECI | m/s (f64) | B+7 m/cs | fps (×3.28084) |
| Position RTIG | m (f64) | B+29 m | — |
| Apogee / Perigee | m (f64) | B+29 m | NM (÷1852) |
| Delta-V magnitude | m/s (f64) | B+7 m/cs | fps |

### Restart safety

- `enter()` calls `state.restart.set_phase(4, 4)` (`TC PHASCHNG / OCT 00014`).
- `compute_target()` is purely computational with no intermediate restart state required.
- If a restart occurs between `enter()` and `compute_target()`, Group 4 phase 4 causes re-entry at the beginning of P30 input display (`P30PHSI` label in AGC).

## Invariants

1. P30 issues **no engine, RCS, or TVC commands**. All outputs are data (registers and flags).
2. `compute_target()` must be called after `enter()` and before `display_summary()`.
3. `compute_target()` propagates to TIG using the Kepler conic approximation, matching the AGC's `THISPREC` Encke propagation for durations up to a few hours. For longer arcs, numerical integration via `navigation::integration` should be substituted (outside P30's scope).
4. `XDELVFLG` must be set by `compute_target()` before P40 can use the burn target. P40 reads this flag on entry.
5. No heap allocation. All intermediate values fit in `f64` local variables or `AgcState` fields.
6. `display_summary()` is read-only with respect to `AgcState`.

## Test Cases

### TC-P30-1: compute_target rotates prograde LVLH burn to ECI
```
Setup:    state.nav = circular LEO at r=[6556370, 0, 0] m, v=[0, 7784, 0] m/s.
          TIG = current MET (burn now).
          DELVSLV = [0.0, 50.0, 0.0] m/s (prograde).
Action:   p30_ext_dv::enter(&mut state, tig, [0.0, 50.0, 0.0]);
          let bt = p30_ext_dv::compute_target(&mut state);
Expected: bt.delta_v_lvlh == [0.0, 50.0, 0.0]
          DELVSIN (ECI) ≈ [0.0, 50.0, 0.0] (for this geometry, prograde = +Y ECI)
          |DELVSIN| ≈ 50.0 m/s
          state.flags.xdelvflg == true
```

### TC-P30-2: compute_target produces valid apogee/perigee for a 50 m/s prograde burn
```
Setup:    Circular 185 km LEO. TIG = now.
          DELVSLV = [0.0, 50.0, 0.0] m/s (prograde raising burn).
Action:   enter() + compute_target()
Expected: state.hapo > 185_000.0 m  (apogee raised above 185 km)
          state.hper ≈ 185_000.0 m  (perigee unchanged to within 1 km for small burn)
          state.hper < state.hapo
```

### TC-P30-3: display_summary writes correct NM and fps values
```
Setup:    state.hapo = 300_000.0 m, state.hper = 185_000.0 m,
          state.vgdisp = 50.0 m/s (delta-V magnitude).
Action:   p30_ext_dv::display_summary(&state, &mut hw)
Expected: hw.dsky.r1 ≈ 162 (300000 ÷ 1852 ≈ 162 NM, within 1)
          hw.dsky.r2 ≈ 100 (185000 ÷ 1852 ≈ 100 NM, within 1)
          hw.dsky.r3 ≈ 164 (50.0 × 3.28084 ≈ 164 fps, within 1)
          hw.dsky.noun == 42
```

## agc-sim Impact

- `DskyState`: `display_summary()` writes N42 display (R1=apogee NM, R2=perigee NM, R3=delta-V fps). No new display fields needed; routes through existing `hw.dsky().set_display()`.
- `MissionState` panel: add `apogee_km` and `perigee_km` fields updated from `state.hapo` / `state.hper` when `xdelvflg` is true.
- `SimLog`: emit `log::info!("P30 target computed: TIG={} DELVSLV={:?}", tig, delta_v_lvlh)` from `compute_target()`.
- New keyboard binding: V37 → P30 (MM 30) already dispatches via the existing V37 handler.
- Scenario `--scenario burn`: auto-invokes `p30_ext_dv::enter()` with a 50 m/s prograde burn target before selecting P40.
