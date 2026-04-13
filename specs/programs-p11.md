# Spec: P11 — Earth Orbit Insertion Monitor

## AGC Source Reference

```
File:     Comanche055/P11.agc
Routines: P11 (page 535), REP11 (page 539), REP11A (page 536),
          VHHDOT (page 539), S11.1 (page 540+), MATRXJOB (page 537),
          ATERTASK / ATERJOB (page 540)
Pages:    533–550
```

Supporting routines called from P11:
```
File:     Comanche055/SERVICER207.agc
Routines: PREREAD1 (initialise Average-G), NORMLIZE
File:     Comanche055/FRESH_START_AND_RESTART.agc
Routines: GOTOPOOH (normal exit path on crew V37N00)
```

## Behavior Summary

P11 monitors booster-powered ascent from liftoff through Earth orbit insertion. It does **not** control the Saturn V booster (the Saturn IGM handles that); it observes, integrates the CSM state vector via Average-G, and displays navigation parameters on the DSKY.

### Entry paths

1. **Normal**: P02 (gyrocompass) calls P11 automatically when the liftoff discrete is received (`T3RUPT` liftoff detection). This is the nominal ascent path.
2. **Backup**: crew enters `V75 ENTER` at any time to force P11 activation.

### Initialization sequence (P11 label, pages 535-537)

On entry P11 performs this sequence, protected by phase table Group 3 and Group 5:

1. **Zero CMC clock at liftoff**: double-word swap of `TIME2 / TIME1` registers to zero, saving the pre-liftoff time in `TLIFTOFF`.
2. **Update TEPHEM**: `TEPHEM` is corrected by adding `TLIFTOFF` so the ephemeris time-base tracks the zeroed clock.
3. **Start Average-G** (`PREREAD1`): zeros PIPA accumulators, initialises the servicer at `PREREAD1` phase. This starts the 2-second PIPA-read / state-integration heartbeat via the Waitlist.
4. **Set major mode to 11**: `TC NEWMODEX / MM 11`.
5. **Clear DSKY** (in case invoked via V75): `TC BANKCALL / CADR CLEANDSP`.
6. **Store liftoff CDU angles** (`CDUX / CDUY / CDUZ`) into `OGC / IGC / MGC` for the FDAI attitude-error display baseline.
7. **Compute initial position and velocity vectors** `RN1 / VN1`: calls `LALOTORV` to convert pad latitude, longitude, and altitude to an ECI position vector, and computes the Earth-surface velocity at launch azimuth.
8. **Compute prelaunch REFSMMAT**: constructs the Reference Stable-Member Matrix from three unit vectors:
   - `UNIT_Z = UNIT(-R)` (local vertical, upward)
   - `UNIT_X = UNIT(A)` where A is the horizontal vector at the launch azimuth
   - `UNIT_Y = UNIT_Z × UNIT_X`
   Stores the resulting 3×3 matrix in `REFSMMAT` and sets `REFSMFLG`.
9. **Set AVGEXIT pointer**: loads `P11SCADR` (a 2CADR to `VHHDOT`) into `AVGEXIT`, so the servicer calls `VHHDOT` at the end of every 2-second Average-G cycle.
10. **Set 1/PIPADT** = 2 seconds (integration cycle rate).
11. **Schedule ATERTASK** (attitude-error display task) on the Waitlist at 0.5 s.
12. **Call NORMLIZE** (via POSTJUMP): completes first-cycle initialisation of `GDT/2`.

### Ongoing cycle (VHHDOT, every 2 seconds)

`VHHDOT` is the Average-G exit hook, called by the servicer after each `CALCRVG` state update:
1. Calls `S11.1` to compute display quantities: inertial velocity magnitude `VMAGI` (fps), altitude rate `HDOT` (fps), altitude above pad radius `ALTI` (NM).
2. Issues `V06 N62` monitor display:
   - R1: inertial velocity VI (fps)
   - R2: HDOT — altitude rate (fps)
   - R3: H — altitude above pad (NM)

### Attitude-error display (ATERJOB, every 0.5 s)

A low-priority job runs approximately every 0.5 s (or less, depending on load):
- From liftoff to RPSTART (~0 to +10 s): desired attitude = attitude stored at liftoff in `OGC/IGC/MGC`.
- From RPSTART to POLYSTOP (~+10 s to +133 s): desired attitude given by CMC pitch and roll polynomial evaluations (Saturn roll-out and pitch-over).
- Sends body-axis attitude errors to the FDAI needles via `NEEDLER`.
- Disables itself at TIME1 overflow (timer wrap, approximately 82 minutes into flight).

### Crew interaction

While below 300,000 ft altitude:
- Monitor display of time to perigee (R1 hours, R2 minutes).
- Monitor display of apogee altitude (R1 NM), perigee altitude (R2 NM), time-of-free-fall (R3 minutes/seconds).
- Pressing PROCEED returns to the nominal N62 display.

### Normal exit

Crew enters `V37 ENTER / 00 ENTER` (or any non-P11 major mode via V37). Control passes to `GOTOPOOH` or the selected program. P11 has no timed automatic termination.

### What P11 does NOT do

- Does not command engines, jets, or RCS.
- Does not command TVC.
- Does not send guidance to Saturn (the Saturn IGM is autonomous).
- Does not modify `REFSMMAT` after the initial computation (the prelaunch REFSMMAT is fixed for the duration of P11).

## Rust API

Module path: `agc_core::programs::p11_eoi`

```rust
/// Enter P11 Earth Orbit Insertion Monitor.
///
/// Performs the full P11 initialization sequence:
/// - Records liftoff time by zeroing CMC clock (TIME2/TIME1) and computing TLIFTOFF.
/// - Updates TEPHEM via TLIFTOFF correction.
/// - Starts Average-G servicer via `services::average_g::AverageG::start()`.
/// - Sets major mode register to 11 (`state.modreg = 11`).
/// - Stores liftoff CDU angles for attitude-error display.
/// - Computes initial RN/VN from pad coordinates via LALOTORV.
/// - Computes and stores prelaunch REFSMMAT.
/// - Sets AVGEXIT to the VHHDOT hook.
/// - Schedules the attitude-error display task on the Waitlist.
/// - Sets restart protection (Group 3 and Group 5 phases).
///
/// Must be called from an Executive job context (not from an ISR).
/// Hardware IMU must be at least CoarseAligned before entry.
///
/// AGC source: Comanche055/P11.agc P11 label, pages 535-537.
///
/// Inputs: `state` — mutable AgcState; `hw` — AgcHardware (reads CDU angles,
///         schedules tasks on Waitlist).
pub fn enter(state: &mut AgcState, hw: &mut dyn AgcHardware);

/// Advance the P11 navigation display by one Average-G cycle (every 2 s).
///
/// Computes and updates the three DSKY display quantities via S11.1:
///   R1: inertial velocity magnitude (m/s, displayed in fps)
///   R2: altitude rate HDOT (m/s, displayed in fps)
///   R3: altitude above pad radius (m, displayed in NM)
///
/// Called from the Average-G servicer exit hook (AVGEXIT pointer).
/// Does not modify the state vector; that is done by AverageG::tick().
///
/// AGC source: Comanche055/P11.agc VHHDOT label, page 539.
pub fn tick(state: &mut AgcState, hw: &mut dyn AgcHardware);

/// Exit P11 (crew-requested via V37 or alarm path).
///
/// Clears the AVGEXIT hook (sets it to a no-op address).
/// Clears the attitude-error display Waitlist task.
/// Does NOT stop the Average-G servicer (the servicer continues running
/// for navigation continuity; the calling program takes ownership).
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc GOTOPOOH path
///   (the crew's V37N00 causes this on P11 exit).
pub fn exit(state: &mut AgcState);
```

### AgcState fields read / written

| Field | AGC register | Direction | Notes |
|---|---|---|---|
| `state.modreg` | `MODREG` | Write | Set to 11 on enter |
| `state.tliftoff` | `TLIFTOFF` | Write | MET of liftoff (centiseconds) |
| `state.refsmmat` | `REFSMMAT` | Write | 3×3 launch-alignment matrix |
| `state.refsmflg` | `REFSMFLG` | Write | Set on REFSMMAT valid |
| `state.avgexit` | `AVGEXIT` | Write | Pointer to VHHDOT |
| `state.nav` | `RN / VN` | Read | Read by tick() for display |
| `state.dsky` | `DSPTAB` | Write | N62 monitor display |

### Scale factors

| Quantity | Rust unit | DSKY display unit | AGC scale |
|---|---|---|---|
| Inertial velocity VI | m/s (f64) | fps (×3.28084) | B+7 m/cs |
| Altitude rate HDOT | m/s (f64) | fps | B+7 m/cs |
| Altitude H | m (f64) | NM (÷1852) | B+29 m |

### Restart safety

P11 uses the standard two-phase-change pattern:

- **Group 3 phase 4** (`TC 2PHSCHNG / OCT 40514 / OCT 00073`): active during the ATERTASK scheduling step.
- **Group 3 phase 5** (`TC PHASCHNG / OCT 05023`): active during TEPHEM correction and PREREAD1 call.
- **Group 5 phase 0** (`CS ZERO / ZL / TS LIFTTEMP / DXCH -PHASE5`): deactivates Group 5 (prelaunch protection) as soon as liftoff is confirmed.
- `REP11` is the restart entry point that re-evaluates which phase was interrupted and branches accordingly.

In Rust: `enter()` calls `state.restart.set_phase(3, 5)` immediately before calling `average_g.start()`. The restart handler must test Group 3 phase and resume from the correct point.

## Invariants

1. P11 is **read-only** with respect to propulsion: no engine or RCS commands are issued.
2. The Average-G servicer (`AverageG`) is started by `enter()` and must already have IMU coarse alignment before first PIPA read.
3. `tick()` is called exclusively from the Average-G exit hook, not from foreground code.
4. `exit()` must be idempotent (safe to call if already exited).
5. `state.refsmmat` is computed once at liftoff; `tick()` must not overwrite it.
6. No heap allocation. No blocking in any of the three functions.

## Test Cases

### TC-P11-1: enter() sets major mode to 11
```
Setup:    state.modreg = 0 (P00)
          state.nav = circular LEO state vector at 185 km
          hw.imu = CoarseAligned with known REFSMMAT
Action:   p11_eoi::enter(&mut state, &mut hw)
Expected: state.modreg == 11
          p00_idle::is_active(&state) == false
          state.refsmflg == true
          state.avgexit points to the VHHDOT hook
```

### TC-P11-2: tick() updates DSKY N62 display quantities
```
Setup:    After enter(); state.nav has v = 7784 m/s (LEO circular speed),
          altitude = 185 km above pad radius (6371 km + 185 km = 6556 km).
Action:   p11_eoi::tick(&mut state, &mut hw)
Expected: hw.dsky.r1 ≈ 25537 fps (7784 × 3.28084, within 1%)
          hw.dsky.r3 ≈ 99.8 NM (185000 ÷ 1852, within 1%)
          hw.dsky.noun == 62
```

### TC-P11-3: exit() clears avgexit hook
```
Setup:    p11_eoi::enter(&mut state, &mut hw)
          state.avgexit is non-null (VHHDOT pointer)
Action:   p11_eoi::exit(&mut state)
Expected: state.avgexit == null/None (no further VHHDOT calls from servicer)
          state.modreg unchanged (exit does not change modreg; V37 does)
```

## agc-sim Impact

- `MissionState` panel: add display of `VI` (inertial velocity in m/s), `HDOT` (altitude rate in m/s), `H` (altitude in km) updated each time `tick()` is called.
- `SimLog`: emit `log::info!("P11 liftoff at MET {}")` from `enter()`.
- The N62 display already works through `hw.dsky().set_display(noun=62, r1, r2, r3)`; no new DSKY infrastructure required.
- `agc_sim` scenario `--scenario launch`: auto-invokes `p11_eoi::enter()` at T=0.
