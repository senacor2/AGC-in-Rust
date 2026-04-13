# Functional Specification: P61–P67 — Entry Guidance

## AGC Source Reference

```
AGC source:  Comanche055/P61-P67.agc
Routines:    P61 (page 789), P62 (page 792), P63 (page 795), P64 (page 797),
             P65 (page 798), P66 (page 799), P67 (page 800), P67.1 (page 800),
             P67.2 (page 802), S61.1 (page 803), S61.2 (page 806),
             S62.3 (gimbal angles at .05g), FISHCALC (page 812),
             VGAMCALC (page 813)
Supporting:  Comanche055/ENTRY_LEXICON.agc (pages 837-843)
             Constants, gains, scaling conventions for entry guidance
Lines:       P61-P67.agc pages 789–818
```

---

## Behavior Summary

P61–P67 is the entry guidance sequence for Command Module reentry from lunar
or Earth-orbit trajectories. It is a 7-phase state machine driven by sensed
atmospheric drag acceleration (D, measured via PIPAs) and computed range-to-go.

The crew invokes P61 via V37 before atmospheric interface. The program
progresses automatically through the phases as the vehicle descends. Each
phase change is triggered by a physical condition (altitude, drag level,
velocity, range error) rather than by elapsed time.

### Phase Summary

| Phase | Program | Trigger Condition | Purpose |
|---|---|---|---|
| P61PreEntry | P61 | Entered via V37 | Display entry predictions; compute EMS initialization data |
| P62SepCM | P62 | Falls through from P61 or V37 | CM/SM separation readiness; compute .05G attitude; maneuver to entry attitude |
| P63EntryInit | P63 | Sensed drag begins to rise; P62 hands off | Hold entry attitude; sense .05G threshold |
| P64Post05G | P64 | Sensed drag D ≥ 0.05G | Start entry guidance: constant drag phase; select roll attitude |
| P65Upcontrol | P65 | Range-to-go < 25 NM of target AND V > 27 000 FPS | Up-control phase: steer to controlled exit condition |
| P66Ballistic | P66 | Drag D < Q7 FPSS (in P65) | Ballistic (trim) phase: hold trim attitude to relative velocity |
| P67Final | P67 | RDOT < 0 AND V < VL + 500 FPS (from P65), or V < 27 000 FPS at .2G (from P64) | Final phase: range and lateral corrections; drogue deploy |

---

## Constants (from P61-P67.agc and ENTRY_LEXICON.agc)

| Name | AGC source | Rust value | Unit | Notes |
|---|---|---|---|---|
| `ENTRY_INTERFACE_M` | S61.2: `400KFT 2DEC 121920 B-29` | `121_920.0` | m | 400 kft altitude = entry interface |
| `EMSALT_ORBITAL_M` | Comment: `284843 FT` | `86_759.2` | m | EMS interface (orbital reentry, pad-loaded) |
| `EMSALT_LUNAR_M` | Comment: `297431 FT` | `90_657.0` | m | EMS interface (lunar reentry, pad-loaded) |
| `POINT_05G_THRESHOLD` | ENTRY_LEXICON: `.05GSW = CM/FLAGS bit 3`; trigger when sensed D ≥ 0.05×G0 | `0.05 * 9.80665` | m/s^2 | 0.05 g-onset threshold |
| `POINT_2G_THRESHOLD` | P64 comment: "select P67 if V < 27000 FPS when .2G occurs" | `0.2 * 9.80665` | m/s^2 | 0.2 g level for P67 selection from P64 |
| `VFINAL1_FPS` | ENTRY_LEXICON: `VFINAL1 = 27000 FPS` | `8229.6` | m/s | Velocity threshold to enter upcontrol vs final phase |
| `VLMIN_FPS` | ENTRY_LEXICON: `VLMIN = 18000 FPS` | `5486.4` | m/s | Minimum VL for upcontrol solution |
| `VQUIT_FPS` | ENTRY_LEXICON: `VQUIT = 1000 FPS` | `304.8` | m/s | Velocity to stop steering (P67 final) |
| `RANGE_25NM_M` | ENTRY_LEXICON: `25NM tolerance = 25 NM` | `46_300.0` | m | Range-to-go threshold to enter P65 |
| `Q7F_FPSS` | ENTRY_LEXICON: `Q7F = 6 FPSS` (minimum drag for upcontrol) | `1.8288` | m/s^2 | Minimum drag to maintain upcontrol |
| `VSAT_FPS` | ENTRY_LEXICON: `VSAT = 25766.1973 FPS` | `7853.0` | m/s | Satellite velocity at Earth radius |
| `C18_BIAS_FPS` | ENTRY_LEXICON: `C18 = 500 FPS` | `152.4` | m/s | P65 → P67 velocity bias (VL + 500 FPS trigger) |
| `EARTH_RADIUS_FT` | ENTRY_LEXICON: `RE = 21202900 FT` | `6_461_844.7` | m | Entry Earth radius |
| `LADPAD_NOM` | ENTRY_LEXICON: `LADPAD = 0.3` | `0.3` | dimensionless | Nominal vehicle L/D ratio (pad-loaded) |
| `LODPAD_NOM` | ENTRY_LEXICON: `LODPAD = 0.18` | `0.18` | dimensionless | Final phase L/D (pad-loaded) |
| `HEADSUP_LIFT_DOWN` | P61.4: `CA BIT14; DXCH ROLLC` = 180 deg | `core::f64::consts::PI` | rad | Roll angle for lift-down entry |
| `HEADSUP_LIFT_UP` | P61.4: `NOOP; DXCH ROLLC` = 0 deg | `0.0` | rad | Roll angle for lift-up entry |

Note: Drogue deploy altitude is not an explicit constant in P61-P67.agc. P67
terminates guidance when `V_earth ≤ 1000 FPS` (VQUIT) — drogue deploy is an
external command issued by the crew or a separate ELS program not modeled here.
The spec records P67 terminal condition as `V ≤ VQUIT_FPS`.

---

## Rust API

**Module path:** `agc_core::programs::p61_entry`

### Types

```rust
/// Seven-phase monotonic entry guidance state machine.
/// AGC source: P61-P67.agc NEWMODEX calls (MM 61..67) and RTB dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EntryPhase {
    /// P61: Pre-entry calculations and predictions display.
    /// Entry trigger: V37 crew selection.
    /// AGC label: P61 (page 789); display N60, N63.
    P61PreEntry,
    /// P62: CM/SM separation readiness; entry attitude maneuver.
    /// Entry trigger: Falls through from P61, or V37 MM=62.
    /// AGC label: P62 (page 792); CM/DAPIC started; ROLLC computed.
    P62SepCM,
    /// P63: Hold entry attitude; sense 0.05G onset.
    /// Entry trigger: P62 hands off after attitude maneuver complete.
    /// AGC label: P63 (page 795); P63FLAG set; display N64.
    P63EntryInit,
    /// P64: Post-0.05G; constant drag guidance phase.
    /// Entry trigger: Sensed drag D ≥ POINT_05G_THRESHOLD.
    /// AGC label: P64 (page 797); RTB from reentry control (DANZIG/INITROLL).
    P64Post05G,
    /// P65: Up-control phase; steer to controlled exit.
    /// Entry trigger: Range-to-go < 25 NM AND V > VFINAL1.
    /// AGC label: P65 (page 798); GOTOADDR set to UPCONTRL.
    P65Upcontrol,
    /// P66: Ballistic/trim phase; hold attitude in trim.
    /// Entry trigger: Drag D < Q7 FPSS (from P65).
    /// AGC label: P66 (page 799); KEP2 ballistic integration.
    P66Ballistic,
    /// P67: Final phase; range and lateral corrections, guidance termination.
    /// Entry trigger: RDOT < 0 AND V < VL + 500 FPS (from P65), or
    ///                V < VFINAL1 at 0.2G (from P64).
    /// AGC label: P67 (page 800); terminates when V ≤ VQUIT.
    P67Final,
}

/// Active entry guidance state.
#[derive(Clone, Copy, Debug)]
pub struct EntryState {
    /// Current guidance phase.
    pub phase: EntryPhase,
    /// Target splash-down latitude (degrees, WGS-84 approximate).
    /// AGC erasable: LAT(SPL) scaled /360 (fraction of revolution).
    /// Rust: f64 degrees.
    pub target_lat_deg: f64,
    /// Target splash-down longitude (degrees).
    /// AGC erasable: LNG(SPL) scaled /360.
    pub target_lon_deg: f64,
    /// Predicted range-to-go to target (meters).
    /// AGC erasable: RTGO (THETAH/360); ENTRY_LEXICON: max 21600 NM.
    pub range_to_go_m: f64,
    /// Current roll command (radians).
    /// AGC erasable: ROLLC (1 revolution scale).
    pub rollc_rad: f64,
    /// Lift-up / lift-down selection (+1 = lift down, -1 = lift up).
    /// AGC erasable: HEADSUP.
    pub headsup: i8,
    /// Sensed drag acceleration (m/s^2). Updated each SERVICER cycle.
    /// AGC erasable: D (total accel), scaled 805 FPSS max.
    pub drag_acc_ms2: f64,
    /// Inertial velocity magnitude (m/s). Updated each SERVICER cycle.
    /// AGC erasable: VMAGI (-7) M/CS.
    pub vi_ms: f64,
    /// Altitude rate (m/s). Positive = climbing.
    /// AGC erasable: RDOT (2 VSAT scale).
    pub rdot_ms: f64,
    /// Exit velocity for upcontrol (VL). Set during P64.
    /// AGC erasable: VL (2 VSAT scale). ENTRY_LEXICON: VLMIN = 18000 FPS.
    pub vl_ms: f64,
    /// Minimum drag threshold for upcontrol (Q7).
    /// AGC erasable: Q7. ENTRY_LEXICON: Q7F = 6 FPSS minimum.
    pub q7_ms2: f64,
    /// True once guidance termination has been commanded (V ≤ VQUIT).
    pub guidance_terminated: bool,
    /// Maximum predicted entry acceleration (display only).
    /// AGC erasable: GMAX (100 GMAX, -14 G-S scale).
    pub gmax_g: f64,
    /// Predicted velocity at 400 kft entry interface.
    /// AGC erasable: VPRED (-7) M/CS.
    pub vpred_ms: f64,
    /// Predicted flight-path angle at 400 kft.
    /// AGC erasable: GAMMAEI (GAMMA/360).
    pub gammaei_rad: f64,
    /// Time-to-entry interface (centiseconds from current state).
    /// AGC erasable: TTE (-28 CS).
    pub tte_cs: i64,
}
```

### Functions

```rust
/// Enter entry guidance at P61 (pre-entry calculations).
///
/// Sets `phase = P61PreEntry`. Initializes HEADSUP = -1 (lift up default).
/// Locks out extended verbs (EXTVBACT = BIT14). Calls S61.1 (IMU/state-vector
/// check). Calls S61.2 (computes GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE).
/// Prompts crew for LAT(SPL), LNG(SPL), HEADSUP via N61 flash.
///
/// AGC source: P61-P67.agc P61 entry (page 789-791).
/// Sets EXTVBACT; calls S61.1, S61.2; displays N60, N63.
pub fn enter(state: &mut AgcState, hw: &mut dyn AgcHardware) -> EntryState;

/// Advance entry guidance by one executive cycle.
///
/// Reads current sensed state from navigation (drag, Vi, Rdot, range-to-go),
/// evaluates phase transition conditions, issues attitude commands, and
/// updates the `EntryState`. Each transition is monotonic (no backtracking).
///
/// Phase transition conditions:
///
///   P61PreEntry → P62SepCM:
///     Crew presses PROCEED on N61 or N63 display (or V37 MM=62).
///     AGC label: P61 fallthrough ".... THEN FALL INTO P62" (page 791).
///
///   P62SepCM → P63EntryInit:
///     CM/DAP attitude maneuver to entry attitude complete (ALFA within ±45°).
///     AGC label: WAKEP62 task wakes P63 (page 793); or direct if CMDAPMOD = -1.
///     Also: CM/DAPON called to enable entry DAP.
///
///   P63EntryInit → P64Post05G:
///     Sensed drag D ≥ POINT_05G_THRESHOLD (0.05 × G0).
///     The .05GSW flag (CM/FLAGS bit 3) is set. P64 entered via RTB from
///     reentry control STARTENT.
///     AGC label: P64 TC NEWMODEX MM 64; TC DANZIG (page 797).
///
///   P64Post05G → P65Upcontrol:
///     Range-to-go < 25 NM AND V > VFINAL1_FPS (27 000 FPS / 8229.6 m/s).
///     Upcontrol solution must exist (VL > VLMIN = 18 000 FPS / 5486.4 m/s).
///     AGC label: P65 TC NEWMODEX MM 65; GOTOADDR = UPCONTRL (page 798).
///
///   P64Post05G → P67Final (direct):
///     V < VFINAL1 when D ≥ 0.2G, OR no upcontrol solution (VL ≤ VLMIN).
///     AGC label: P64 function 2 and 4 (page 797 comments).
///
///   P65Upcontrol → P66Ballistic:
///     Drag D < Q7 FPSS.
///     AGC label: P66 TC NEWMODEX MM 66; entered "WHEN D < Q7 FPSS" (page 799).
///
///   P65Upcontrol → P67Final:
///     RDOT < 0 AND V < VL + C18 (VL + 500 FPS / 152.4 m/s).
///     AGC label: P65 function B (page 798 comment).
///
///   P66Ballistic → P67Final:
///     Drag builds back up to Q7 + 0.5 FPSS (0.1524 m/s^2) — re-enters upcontrol
///     or conditions met for P67. In practice P66 can loop to P65 or P67.
///     AGC label: P66 continues at KEP2 ballistic; returns to reentry control.
///
///   P67Final (terminal):
///     When V ≤ VQUIT_FPS (1000 FPS / 304.8 m/s): entry DAP off; `guidance_terminated = true`.
///     AGC label: P67.1 GOFLASH N67; CS THREE; MASK CM/FLAGS (page 800-801).
///
/// Invariants:
///   - `phase` is strictly monotonically non-decreasing.
///   - Engine is never commanded during entry guidance (guidance outputs are
///     roll commands only, issued via DAP through `state.flagwrds`).
///   - P67Final is the only terminal phase. After `guidance_terminated = true`,
///     tick is a no-op.
///
/// AGC source: P61-P67.agc full reentry control flow; ENTRY_LEXICON constants.
pub fn tick(entry_state: &mut EntryState, state: &mut AgcState, hw: &mut dyn AgcHardware);
```

---

## Scale Factors

| Quantity | AGC scale | Rust f64 unit |
|---|---|---|
| Position (RN, RONE) | B-29 m | m |
| Velocity (VN, VIO, VPRED) | B-7 m/cs = 128 m/s | m/s |
| Drag (D, Q7) | 805 FPSS max (feet/s^2) | m/s^2 |
| Range (RTGO, THETAH) | THETAH/360 = fraction of revolution | m (convert via RE × angle_rad) |
| Latitude/longitude | /360 (fraction of revolution) | degrees (×360) |
| Roll command (ROLLC) | 1 revolution | radians (×2π) |
| Time (TTE) | B-28 cs | centiseconds as i64 |
| GMAX | 100 × GMAX, B-14 G-S | g-force (×0.01) |
| HEADSUP | +1 = lift down, -1 = lift up | i8 |
| GAMMAEI | GAMMA/360 (fraction of revolution) | radians (×2π) |

---

## Key Subroutines (called from tick — not separately implemented)

| Subroutine | AGC file / label | Purpose |
|---|---|---|
| `s61_1()` | P61-P67.agc S61.1 (page 803) | Validate IMU orientation; check AVERAGEG on; update state vector if needed. Alarms: 01426 (IMU unsatisfactory), 01427 (IMU reversed) |
| `s61_2()` | P61-P67.agc S61.2 (page 806) | Compute GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE for display |
| `s62_3()` | P61-P67.agc P62.3 (page 794) | Compute desired CDU angles for entry attitude; fills CPHI for N22 |
| `fishcalc()` | P61-P67.agc FISHCALC (page 812) | Compute Fischer ellipsoid radius at current latitude for range calc |
| `vgamcalc()` | P61-P67.agc VGAMCALC (page 813) | Compute predicted velocity and gamma at terminal radius |
| `startent()` | Referenced via ENTCADR; reentry control sequencer | Entry guidance inner loop — 2 s SERVICER cycle; senses drag, updates GOTOADDR |

---

## Entry Guidance Equations Summary

P64 constant-drag controller selects initial roll attitude and D0:

```
D0 = f(KA1, KA2, KA3, KA4 from ENTRY_LEXICON)
L/D = LADPAD (pad-loaded, nominal 0.3)
Drag threshold KA: if D < KA → lift up, if D > KA → lift down
```

P65 up-control phase (UPCONTRL): iterates L/D command to achieve exit at
(VL, GAMMAL). Uses gains KB1=3.4, KB2=0.0034 from ENTRY_LEXICON.

P66 ballistic: zero roll command (trim to relative velocity vector). Exits
when drag builds back to Q7 + 0.5 FPSS.

P67 final phase: table-lookup for RTOGO, lateral corrections. Terminates
at VQUIT = 304.8 m/s.

---

## Invariants

1. `phase` is monotonically non-decreasing. Phase regression raises an alarm.
2. Entry DAP is active (CM/DAPON called in P62) for all phases P62 through P67.
3. The SPS and RCS translational jets are never commanded during entry.
   Roll commands are issued via the entry DAP (ROLLC register in `state.flagwrds`).
4. `guidance_terminated = true` is the only terminal condition. tick returns
   immediately if this flag is set.
5. EXTVBACT lock must be cleared by P67 exit (or emergency P00 exit).
6. S61.1 must be called at P61 and P62 entry. If IMU fails (alarm 01426/01427),
   a 10-second hold is applied before exit.

---

## DSKY / agc-sim Impact

- `P61PreEntry`: Flash N61 (LAT/LNG/HEADSUP input). Flash N60 (GMAX, VPRED,
  GAMMAEI). Flash N63 (RTGO, VIO, TTE). `DskyEvent::FlashN61`, `FlashN60`, `FlashN63`.
- `P62SepCM`: Display N61 again (re-confirm target). Display N22 (CDU desired
  angles from S62.3). Illuminate UPLINK ACTIVITY lamp for CM/SM separation command.
- `P63EntryInit`: Display N64 (G, VI, R-to-splash). `DskyEvent::DisplayN64`.
- `P64Post05G`: Display N74 (ROLLC, VI, D). `DskyEvent::DisplayN74`.
- `P65Upcontrol`: Flash N69 (ROLLC, Q7, VL) for crew confirm.
  `DskyEvent::FlashN69`.
- `P66Ballistic`: Display N22 (OGA, IGA, MGA gimbal angles).
- `P67Final`: Flash N67 (RTOGO, LAT, LONG). `DskyEvent::FlashN67`.
  Emit `SimLog::info("P67 guidance terminated — drogue deploy condition met")`.
- New `DskyDisplayState` field: `entry_phase: Option<EntryPhase>` (for TUI
  rendering of the current phase).
- No new keyboard bindings needed (crew uses standard V/N/ENTR/PRO).

---

## Test Cases

| # | Name | Setup | Expected |
|---|---|---|---|
| T1 | P61 enters correctly | `enter()` with valid state vector | `phase == P61PreEntry`; EXTVBACT set; S61.1 called |
| T2 | P61 → P62 on crew proceed | Inject DSKY PROCEED; call `tick()` | `phase == P62SepCM`; CM/DAPON called |
| T3 | P62 → P63 on attitude converge | Simulate entry attitude achieved (ALFA within 45°); call `tick()` | `phase == P63EntryInit`; P63FLAG set |
| T4 | P63 → P64 on 0.05G onset | Set `drag_acc_ms2 = 0.05 * 9.80665 + 0.01`; call `tick()` | `phase == P64Post05G`; .05GSW flag set |
| T5 | P64 → P67 direct (V < 27000 FPS at 0.2G) | Set `vi_ms = 7000.0`, `drag_acc_ms2 = 0.2 * 9.80665 + 0.01`; call `tick()` in P64 | `phase == P67Final` (bypasses P65) |
| T6 | P64 → P65 on range < 25 NM | Set `range_to_go_m = 40_000.0`, `vi_ms = 8500.0`, `vl_ms = 6000.0`; call `tick()` in P64 | `phase == P65Upcontrol` |
| T7 | P65 → P66 on D < Q7 | Set `drag_acc_ms2 = q7_ms2 * 0.9`; call `tick()` in P65 | `phase == P66Ballistic` |
| T8 | P65 → P67 on RDOT < 0 and V < VL+500 | Set `rdot_ms = -10.0`, `vi_ms = vl_ms + 100.0`; call `tick()` in P65 | `phase == P67Final` |
| T9 | P67 terminal at VQUIT | Set `vi_ms = 250.0` (< VQUIT); call `tick()` in P67 | `guidance_terminated == true`; entry DAP off |
| T10 | Phase monotonicity guard | Attempt to set `phase = P63EntryInit` from `P65Upcontrol` | Alarm raised; phase unchanged |
| T11 | IMU unsatisfactory alarm in S61.1 | Simulate IMU Y-axis > 30° from VAR; call `enter()` | Alarm 01426 raised; 10-second hold applied |
