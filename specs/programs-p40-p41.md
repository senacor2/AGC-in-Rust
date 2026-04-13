# Functional Specification: P40/P41 — SPS and RCS Burn Execution

## AGC Source Reference

```
AGC source:  Comanche055/P40-P47.agc
Routines:    P40CSM (page 684), P41CSM (page 688), TIGBLNK (page 690),
             TIG-30 (page 693), TIG-5 (page 694), TIG-0/IGNITION (page 695),
             DOTVCON/DOSTRULL (page 696), ENGINOFF/DOSPSOFF (page 698),
             SPSOFF/BESTTRIM (page 699), POSTBURN (page 686), S40.1 (page 710)
Supporting:  Comanche055/POWERED_FLIGHT_SUBROUTINES.agc (CDUTRIG, AX*SR*T)
             Comanche055/KALCMANU_STEERING.agc (KALCMANU attitude maneuver)
Lines:       P40-P47.agc pages 684–736
```

---

## Behavior Summary

P40 executes an SPS (main engine) burn. P41 executes an RCS translational burn
using the +X (forward-facing) jets for delta-V maneuvers too small for the SPS.

Both programs execute the same 6-phase state machine. They differ only in which
engine is commanded and which thrust constant is loaded. The calling program
(P30 or crew via DSKY) has already solved the maneuver targeting and stored the
result in VGTIG (velocity-to-go at TIG), TIG (time of ignition), and F (thrust
magnitude) in erasable memory before invoking P40/P41.

### Phase Sequence

1. **Attitude** — Slew the vehicle to the computed burn attitude via KALCMANU.
2. **Countdown** — Display TIG countdown (CLOKTASK every 1 s on N40); at
   TIG−30 s schedule the ullage arm task (TIGAVEG → TIG-5).
3. **Ullage** — At TIG−5 s arm the ullage; at TIG−0 s fire +X RCS jets to
   settle propellant in tanks (20 s duration for SPS burns).
4. **Burn** — Enable the main engine (SPS for P40, +X RCS for P41);
   SERVICER loop calls STEERING (cross-product VG steering) every 2 s.
5. **Cutoff** — STEERING schedules ENGINOFF when TGO ≤ 1 s; SPSOFF shuts
   down engine, records TEVENT, resets ENGONFLG.
6. **Trim** — 2.5 s tail-off delay (DOTVCRCS), then BESTTRIM updates pitch and
   yaw trim offsets (PACTOFF/YACTOFF); POSTBURN displays N85 residuals.

---

## Constants (from P40-P47.agc)

| Name | AGC declaration | Rust value | Unit | Notes |
|---|---|---|---|---|
| `SPS_THRUST` | `FENG 2DEC 9.1188544 B-7` | `91_188.544` | N | 20 500 lbf SPS |
| `RCS_ULLAGE_THRUST` | `FRCS2 2DEC .087437837 B-7` | `874.38` | N | 4-jet ullage, 199.6 cos(10°) lbs |
| `TIG_MINUS_30_CS` | `SEC29.96 2DEC 2996` | `2996` | centiseconds | Used by TIGBLNK LONGCALL |
| `TIG_MINUS_25_CS` | `SEC24.96 DEC 2496` | `2496` | centiseconds | Used by TIGAVEG → TIG-5 |
| `TIG_MINUS_5_CS` | `5SEC DEC 500` | `500` | centiseconds | TIG-5 waitlist delta |
| `SPS_TAILOFF_CS` | `DEC 250` (at DOSPSOFF) | `250` | centiseconds | 2.5 s SPS tail-off delay |
| `TVC_BUILDUP_CS` | `DEC 40` (at PREPTVC) | `40` | centiseconds | 0.4 s TVC thrust buildup |
| `ULLAGE_DURATION_CS` | `DEC 160` (at DOTVCON) | `160` | centiseconds | 1.6 s to steering start (total 2.0 s from IGNITION) |
| `TRIM_ONLY_DELAY_CS` | `5SEC DEC 500` (MRKRTMP=0 branch) | `500` | centiseconds | Trim-only task delay |
| `FULL_TEST_DELAY_CS` | `18SEC DEC 1800` | `1800` | centiseconds | 18 s gimbal test delay |
| `MASS_LOSS_3S` | `3MDOT DEC 86.6175796 B-16` | — | kg/cs | 3 s mass loss at 63.8 lbs/s |

Note: The exact ullage duration constant `TULLAGE` (named in the task brief) does
not appear as a named label in P40-P47.agc. The effective ullage-off time is
derived from two back-to-back FIXDELAY calls: `DEC 40` + `DEC 160` = 200 cs
= 2.0 s from IGNITION to DOSTRULL. Use `ULLAGE_DURATION_S: f64 = 2.0`.

---

## Rust API

**Module path:** `agc_core::programs::p40_thrusting`

### Types

```rust
/// Which engine provides thrust for this burn.
/// AGC source: P40-P47.agc — P40CSM clears ENG2FLAG (→ SPS);
///             P41CSM sets ENG2FLAG (→ RCS +X).
pub enum ThrustMode {
    /// SPS main engine. P40CSM path. Sets ENGONFLG in FLAGWRD5.
    Sps,
    /// +X RCS translational jets. P41CSM path. Sets ENG2FLAG in FLAGWRD7.
    RcsPlusX,
}

/// Six-phase monotonic burn state machine.
/// AGC source: Derived from phase-table entries in P40-P47.agc
///             (PHASCHNG, NEWPHASE calls across the TIG sequence).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum BurnPhase {
    /// Phase 1: Slewing vehicle to burn attitude via KALCMANU.
    /// Entry: P40CSM/P41CSM calls R60CSM (attitude maneuver). AGC label: P40SXTY.
    Attitude,
    /// Phase 2: Countdown display active; TIG-30/TIG-5 tasks scheduled.
    /// Entry: R60CSM completes; TIMRFLAG set; CLOKTASK starts. AGC label: P40TTOG.
    Countdown,
    /// Phase 3: +X RCS ullage jets on; propellant settling.
    /// Entry: TIG-0 fires IGNITION, STRULLSW set. AGC label: IGNITION.
    Ullage,
    /// Phase 4: Main engine on; VG steering active.
    /// Entry: DOTVCON enables TVC and kills ullage RCS. AGC label: DOTVCON.
    Burn,
    /// Phase 5: ENGINOFF task scheduled; engine commanded off.
    /// Entry: STEERING calls ENGINOFF when TGO reaches 1 centisecond.
    ///        AGC label: ENGINOFF, DOSPSOFF, SPSOFF.
    Cutoff,
    /// Phase 6: Post-burn; trim residuals, display N85.
    /// Entry: DOTVCRCS completes tail-off delay. AGC label: DOTVCRCS, BESTTRIM, POSTBURN.
    Trim,
}

/// Burn target parameters (set by P30/P31 targeting or DSKY entry).
/// AGC source: P40-P47.agc — loaded into VGTIG, TIG, F before P40 entry.
#[derive(Clone, Copy, Debug)]
pub struct BurnTarget {
    /// Velocity-to-go vector in inertial coordinates at TIG.
    /// AGC erasable: VGTIG, scaled B-7 m/cs. Rust: f64 m/s.
    pub vg_tig: [f64; 3],
    /// Absolute time of ignition in centiseconds from epoch.
    /// AGC erasable: TIG, scaled B-28 cs.
    pub tig_cs: i64,
    /// Nominal thrust magnitude used by S40.13.
    /// AGC erasable: F, scaled B-7 N×10^4.
    /// Rust: f64 newtons (use SPS_THRUST_N or derived RCS value).
    pub thrust_n: f64,
    /// Vehicle mass at TIG.
    /// AGC erasable: CSMMASS / WEIGHT/G. Rust: f64 kg.
    pub mass_kg: f64,
    /// Thrust mode (SPS or RCS +X).
    pub mode: ThrustMode,
}

/// Persistent state for the active P40/P41 burn program.
#[derive(Clone, Copy, Debug)]
pub struct P40State {
    /// Current phase of the burn state machine.
    pub phase: BurnPhase,
    /// Burn target computed by targeting.
    pub target: BurnTarget,
    /// Velocity-to-go vector in inertial coordinates (updated each SERVICER cycle).
    /// AGC erasable: VGPREV/VGTIG. Rust: f64 m/s per axis.
    pub vg: [f64; 3],
    /// Time-to-go in centiseconds (computed by S40.13 / STEERING).
    pub tgo_cs: i32,
    /// Accumulated delta-V magnitude delivered (for residuals display).
    pub dv_delivered_ms: f64,
    /// True once ENGINOFF has been commanded (Cutoff phase entry guard).
    pub engine_off_commanded: bool,
    /// Trim offsets applied at Trim phase.
    /// AGC erasable: PACTOFF (pitch), YACTOFF (yaw). Rust: CduAngle.
    pub trim_pitch: i16,
    pub trim_yaw: i16,
}
```

### Functions

```rust
/// Initialize and enter the P40/P41 burn program.
///
/// Sets phase to `Attitude`, loads target into P40State, calls KALCMANU
/// attitude maneuver via `control::attitude` to slew to burn attitude.
/// Clears ENG2FLAG for P40 (SPS), sets ENG2FLAG for P41 (RCS +X).
///
/// Pre-conditions:
///   - `target.vg_tig` must be non-zero.
///   - `target.tig_cs` must be in the future (> current MET).
///   - IMU must be FineAligned (caller's responsibility to check).
///
/// AGC source: P40-P47.agc P40CSM / P41CSM entry sequences (page 684/688).
/// Sets PFRATFLG, calls S40.1 (via guidance::targeting), then R60CSM.
pub fn enter(state: &mut AgcState, target: BurnTarget) -> P40State;

/// Advance the P40/P41 burn state machine by one executive cycle.
///
/// Called from the Executive each cycle while a P40/P41 burn is active.
/// Evaluates the current MET (`now`) against scheduled phase transition
/// times derived from TIG and issues hardware commands as required.
///
/// Phase transition rules (monotonic — no backtracking):
///   Attitude → Countdown: KALCMANU complete (attitude error < deadband).
///     AGC label: end of R60CSM, TIMRFLAG set at P40TTOG.
///   Countdown → Ullage: elapsed since TIG == 0 (TIG-0 arrival).
///     AGC label: IGNITION.
///   Ullage → Burn: elapsed since IGNITION >= ULLAGE_DURATION_S (2.0 s).
///     AGC label: DOTVCON (after DEC 40 + DEC 160 fixdelay).
///   Burn → Cutoff: TGO <= 1 cs (ENGINOFF scheduled by STEERING).
///     AGC label: ENGINOFF.
///   Cutoff → Trim: SPS_TAILOFF_S (2.5 s) elapsed since ENGINOFF.
///     AGC label: DOTVCRCS → BESTTRIM → POSTBURN.
///
/// Hardware effects per phase:
///   Ullage: `hw.rcs().fire_jets(ULLAGE_JETS)` — +X RCS forward jets.
///   Burn (SPS): `hw.engine().set_engine_enable(true)`.
///   Burn (RCS+X): `hw.rcs().fire_jets(PLUS_X_JETS)`.
///   Cutoff: `hw.engine().set_engine_enable(false)` or `hw.rcs().all_jets_off()`.
///   Trim: `hw.engine().trim_pitch(burn_state.trim_pitch)`,
///         `hw.engine().trim_yaw(burn_state.trim_yaw)`.
///
/// Invariants enforced:
///   - Engine enable forbidden while `phase < BurnPhase::Ullage`.
///   - Phase is strictly monotonic; any backward transition panics in debug.
///   - On `phase == Cutoff`, sets `engine_off_commanded = true`.
///
/// AGC source: P40-P47.agc timing chain: TIGBLNK, TIGAVEG, TIG-5, TIG-0,
///             IGNITION, DOTVCON, DOSTRULL, ENGINOFF, SPSOFF, DOTVCRCS, POSTBURN.
pub fn tick(
    burn_state: &mut P40State,
    state: &mut AgcState,
    hw: &mut dyn AgcHardware,
    now: Met,
);

/// Exit the burn program, null residuals, and safe the engine.
///
/// Forces `hw.engine().set_engine_enable(false)` regardless of current phase.
/// Forces `hw.rcs().all_jets_off()`.
/// Sets `phase = Trim` (terminal state).
/// Resets ENGONFLG in `state.flagwrds[4]` (FLAGWRD5 bit 7).
///
/// AGC source: P40-P47.agc POST41 → GOTOPOOH path (page 687/698).
pub fn exit(burn_state: &mut P40State, state: &mut AgcState, hw: &mut dyn AgcHardware);
```

---

## Scale Factors

| Quantity | AGC scale | Rust f64 unit |
|---|---|---|
| VGTIG, VG | B-7 m/cs (×128 m/s) | m/s |
| TIG | B-28 cs | centiseconds as i64 |
| F (thrust) | B-7 N×10^4 | newtons |
| CSMMASS | B-16 kg | kg |
| PACTOFF, YACTOFF | CDU counts | i16 |
| MET (now) | centiseconds | `Met` newtype (cs as i64) |

---

## Invariants

1. `phase` is monotonically non-decreasing. No transition from a higher phase
   to a lower one is permitted. Violated transitions must raise alarm
   `alarm::raise(AlarmCode::P40PhaseRegress)`.
2. `hw.engine().set_engine_enable(true)` must never be called when
   `phase < BurnPhase::Ullage`. Caller must enforce this pre-condition.
3. On entry to `Cutoff`, `engine_off_commanded` must be set to `true` before
   any subsequent tick call can return.
4. `exit()` is safe to call from any phase (emergency abort).
5. P40 and P41 share `P40State` and `tick`; the branch at `ThrustMode`
   determines which HAL sub-trait is called.

---

## DSKY / agc-sim Impact

- When entering `Countdown` phase: display N40 (TIG countdown) via
  `CLOKTASK` equivalent — emit `DskyEvent::DisplayN40(tgo_cs)` each
  sim cycle.
- When entering `Burn` phase: illuminate ENGINE LIGHT on DSKY
  (`DskyIo::set_relay(RelayWord::ENGINE_ON, true)`).
- When entering `Trim` phase: display N85 (residuals: VGBODY components)
  via `DskyEvent::DisplayN85`.
- `SimLog`: emit `.info("P40 burn ignition")` at `Ullage` entry,
  `.info("P40 engine cutoff")` at `Cutoff` entry.
- New `AgcState` fields needed: none beyond existing `flagwrds`.
- The `P40State` struct is not stored inside `AgcState` — it is held by the
  program-dispatch layer (passed as `&mut P40State` alongside `&mut AgcState`).

---

## Test Cases

| # | Name | Setup | Expected |
|---|---|---|---|
| T1 | Attitude→Countdown transition | `enter()` with valid target; call `tick()` once with attitude error = 0 | `phase == Countdown`; TIMRFLAG set |
| T2 | Countdown→Ullage at TIG | Set `now = tig_cs`; call `tick()` in Countdown | `phase == Ullage`; `hw.rcs()` fires ullage jets |
| T3 | Ullage→Burn after 2.0 s | Set `now = tig_cs + 200`; call `tick()` in Ullage | `phase == Burn`; SPS engine enabled (P40) or +X jets on (P41) |
| T4 | Burn→Cutoff when TGO≤1 | Set `tgo_cs = 1`; call `tick()` in Burn | `phase == Cutoff`; `engine_off_commanded == true`; engine disabled |
| T5 | Cutoff→Trim after 2.5 s tailoff | Advance time 250 cs from engine-off; call `tick()` | `phase == Trim`; trim_pitch/trim_yaw applied to engine HAL |
| T6 | Full SPS burn happy path | Run all ticks in sequence with simulated time advance | Phase progresses Attitude→Countdown→Ullage→Burn→Cutoff→Trim without regression |
| T7 | Emergency exit from Burn | Call `exit()` while `phase == Burn` | Engine disabled; jets off; `phase == Trim` |
| T8 | Phase monotonicity guard | Attempt to set `phase = Attitude` from `Countdown` directly | Alarm raised; phase unchanged |
| T9 | P41 RCS mode | `enter()` with `ThrustMode::RcsPlusX`; run full sequence | No SPS enable ever called; +X RCS fired during Burn |

---

## AGC Cross-References Summary

| AGC label | Phase | Action |
|---|---|---|
| `P40CSM` (p.684) | Entry | Clears ENG2FLAG, sets up CSTEER/FENG, calls S40.1+S40.2,3 |
| `P41CSM` (p.688) | Entry | Sets ENG2FLAG, loads FRCS2 thrust, branches to P40S/F |
| `P40SXTY` (p.684) | Attitude | Calls R60CSM (KALCMANU attitude maneuver) |
| `P40TTOG` (p.684) | Countdown | Starts CLOKTASK/N40 display, TIMRFLAG |
| `TIGBLNK` (p.690) | Countdown | Scheduled TIG-30: schedules TIGAVEG in 5 cs, blanks display |
| `TIGAVEG` (p.693) | Countdown→Ullage | TIG-30 task: sets up V06N40, schedules TIG-5 at TIG-24.96 s |
| `TIG-5` (p.694) | Countdown | Schedules TIG-0 in 500 cs, starts S40.13 (TGO compute) |
| `TIG-0` (p.695) | Countdown→Ullage | Sets IGN flag, waits for V99P crew proceed |
| `IGNITION` (p.695) | Ullage entry | Saves OGAD, enables SPS (SPSON: WOR DSALMOUT bit 13) |
| `DOTVCON` (p.696) | Ullage→Burn | Arms TVC, kills ullage RCS (T5IDLOC), starts DOSTRULL in 160 cs |
| `DOSTRULL` (p.696) | Burn | Calls STEERULL (set STEERSW) and ULAGEOFF (zero CHAN5) |
| `ENGINOFF` (p.697) | Burn→Cutoff | TCR E6SETTER, calls SPSOFF |
| `SPSOFF` (p.699) | Cutoff | Resets ENGONFLG (FLAGWRD5 bit 7), turns off SPS (WAND DSALMOUT) |
| `BESTTRIM` (p.699) | Trim | Updates PACTOFF/YACTOFF from DELPBAR/DELYBAR |
| `POSTBURN` (p.686) | Trim | V16N40 flash, then V16N85 N85 residuals display |
