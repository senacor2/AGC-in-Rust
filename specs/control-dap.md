# Spec: Digital Autopilot (DAP) Supervisor

## AGC Source References

| File | Routine | Pages |
|---|---|---|
| `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc` | `RCSATT`, `REDORCS`, `SETT5`, `FRESHDAP`, `REDAP`, `T5PHASE2`, `ZEROJET` | 1002–1024 |
| `Comanche055/TVCEXECUTIVE.agc` | `TVCEXEC`, `VARGAINS`, `ROLLPREP` | 945–950 |
| `Comanche055/TVCDAPS.agc` | `PITCHDAP`, `YAWDAP`, `DAPINIT`, `ACTLIM` | 961–978 |

---

## Behavior Summary

The Digital Autopilot (DAP) is the top-level control-cycle supervisor. In RCS mode it is
driven entirely by the T5RUPT hardware interrupt; in TVC mode the pitch and yaw loops run
as self-perpetuating T5 tasks while the roll axis stays under RCS control via a Waitlist task.

### T5RUPT Period

The T5 interrupt fires every **100 ms** in steady-state RCS operation.

The period is composed of two phases that together sum to 100 ms:
- **Phase 1** (rate filter + attitude error, `RCSATT`): reset for Phase 2 in **20 ms**
  (`DELTATT2 = OCT 37776`, from `RCS-CSM_DIGITAL_AUTOPILOT.agc` line 90).
- **Phase 2** (manual command decode + jet selection dispatch, `T5PHASE2`): reset for
  Phase 1 in **80 ms** (`DELTATT = OCT 37770`, line 89).

The 20 ms + 80 ms = 100 ms total rate is confirmed by the comment
`"PHASE 1 (RATEFILTER) BEGINS CYCLING 100 MS FROM NOW AND EVERY 100MS THEREAFTER"`
in `REDAP` initialization (line 578).

In TVC mode each pitch and yaw DAP task fires alternately at the TVC sample interval
`T5TVCDT` (pad-loaded; nominally ~40 ms half-period, 80 ms full cycle per axis pair).
The roll DAP is a Waitlist task called every **500 ms** by `TVCEXEC` (`CAF .5SEC`,
`TVCEXECUTIVE.agc` line 97).

### DAP Mode State Machine

```
                  set_mode(Idle)
                 ┌─────────────────────────────────┐
                 │                                 │
 Power-on ──► Idle ──set_mode(Rcs)──► Rcs ──set_mode(Tvc)──► Tvc
                       ◄─set_mode(Idle)─┘         └─set_mode(Rcs)──►  Rcs
```

Transition rules (from `SETT5`, `TVCEXECUTIVE.agc` `TVCEXFIN`):
- **Idle → Rcs**: crew invokes V46E or autopilot is enabled; `FRESHDAP` initialisation runs.
- **Rcs → Tvc**: SPS engine on (P40 calls `DOTVCON`); `TVCDAPON → TVCINIT4` arm the TVC T5 chain.
- **Tvc → Rcs**: SPS off; `FLAGWRD6` bits 15,14 cleared to signal termination (`TVCEXFIN`).
- **Any → Idle**: S/C control switch moved to SCS, or crew action (`CHAN31` bit 15 = 1).

T5PHASE variable encoding (`RCS-CSM_DIGITAL_AUTOPILOT.agc` lines 104–113):
```
T5PHASE = + (positive)  → run FRESHDAP (initialize / turn on autopilot)
         = +0 (zero)    → run T5PHASE2 (Phase 2)
         = -  (negative) → run REDAP   (restart autopilot)
         = -0 (minus zero) → run RCSATT Phase 1 (rate filter)
```

### RCS Control Cycle (per 100 ms)

```
T5RUPT fires
  │
  ├─ Check CHAN31 bit 15 (IMU power + S/C control switch)
  │     if not fully enabled → set NORATE flag, zero errors, reschedule in 100 ms
  │
  ├─ Phase 1 (RCSATT, 20 ms window):
  │     Rate filter (RATEFILT / DRHOLOOP / ADOTLOOP)
  │     AMGB rotation matrix × CDU delta → body-axis rate estimate DRHO
  │     Update ADOT (smoothed angular acceleration)
  │     Schedule AMBGUPDT every 1 s to refresh AMGB matrix
  │     Interpolate commanded angle CDUXD += DELCDUX (steering increment)
  │     Compute attitude error: ERRORX/Y/Z = THETADX - CDUX
  │     Update FDAI error display (NEEDLER)
  │
  ├─ Phase 2 (T5PHASE2, 80 ms window):
  │     Decode manual RHC commands from CHAN31
  │     Determine HOLDFLAG state (attitude hold vs. auto steer)
  │     TAU/TAU1/TAU2 ← jet on-time targets per axis
  │     Dispatch to JETSLECT (jet selection logic)
  │
  └─ JETSLECT phase (JET_SELECTION_LOGIC.agc):
        Look up PYTABLE / RTABLE → PWORD1, YWORD1, RWORD1
        Compute jet on-times BLAST/BLAST1/BLAST2 (minimum 14 ms)
        Sort on-times → schedule T6 interrupt chain
        Write PWORD1+YWORD1 → CHAN5 (PYJETS), RWORD1 → CHAN6 (ROLLJETS)
        Reset T5PHASE for next Phase 1
```

### TVC Control Cycle

```
TVCEXEC (Waitlist, every 500 ms):
  │  Update OGANOW (roll CDU), OGAERR, FDAI needle (AK)
  │  Schedule ROLLDAP Waitlist task (3 cs delay)
  │  Update variable gains via MASSPROP + S40.15
  │  One-shot / repetitive trim corrections (CG.CORR)
  │
PITCHDAP (T5, every T5TVCDT):
  │  Compute MCDUYDOT / MCDUZDOT (CDU rate differences)
  │  Integrate body-axis pitch rate error into PERRB
  │  ERRORLIM clamp → FWDFLTR (6th-order cascade filter with VARK gain)
  │  ACTLIM clamp at ACTSAT (253 counts = 6°)
  │  Increment TVCPITCH error counter → analog signal to actuator
  │  Precompute filter nodes for next pass
  │  Schedule YAWDAP
  │
YAWDAP (T5, every T5TVCDT, staggered):
  │  Same structure as PITCHDAP for yaw axis (YERRB, YCMD, TVCYAW)
  │  Schedule PITCHDAP
  │
ROLLDAP (Waitlist task):
     Phase-plane switching logic (OGA error + rate → ROLLFIRE)
     T6-timed RCS roll jet firings (minimum 15 ms)
```

---

## Rust API

Module: `agc_core::control::dap`

### Constant (to be defined in `agc_core::control::constants`)

```rust
/// T5RUPT period for the RCS DAP, centiseconds.
///
/// Phase 1 = 20 cs + Phase 2 = 80 cs = 100 cs total.
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   DELTATT = OCT 37770 (80 ms), DELTATT2 = OCT 37776 (20 ms).
///   Comment in REDAP: "PHASE 1 (RATEFILTER) BEGINS CYCLING 100 MS FROM NOW".
pub const T5RUPT_PERIOD_CS: u32 = 10; // 10 centiseconds = 100 ms
```

(Note: TIME5 is a countdown register whose value encodes the delay in units of 10 ms
per tick. `OCT 37770` = −8 octal = 80 ms; `OCT 37776` = −2 = 20 ms. The 100 ms
combined period is the authoritative constant.)

### Types

```rust
/// Target attitude for the DAP, expressed as three CDU commanded angles.
///
/// Corresponds to CDUXD/CDUYD/CDUZD erasable registers.
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc, DCDUINCR routine.
pub struct AttitudeTarget {
    pub x: agc_core::types::CduAngle,
    pub y: agc_core::types::CduAngle,
    pub z: agc_core::types::CduAngle,
}

/// DAP operating mode.
///
/// Mirrors the T5PHASE + FLAGWRD6 state machine in Comanche055.
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc (T5PHASE encoding, pp. 1002-1003);
///             TVCEXECUTIVE.agc TVCEXFIN (TVC termination).
pub enum DapMode {
    /// No autopilot activity. T5RUPT not hooked; jets all off.
    Idle,
    /// RCS attitude hold / auto-maneuver. Driven by T5RUPT at 100 ms.
    Rcs,
    /// TVC (SPS burn) mode. Pitch/yaw driven by T5 tasks; roll by Waitlist.
    Tvc,
}

/// Digital Autopilot supervisor state.
///
/// All fields are `Copy` / statically sized — no heap.
/// Shared-mutable access (ISR ↔ foreground) must be wrapped in
/// `cortex_m::interrupt::Mutex<RefCell<Dap>>` by the caller.
pub struct Dap {
    pub mode: DapMode,
    /// Commanded attitude (CDUXD/CDUYD/CDUZD).
    pub target: AttitudeTarget,
    /// T5 phase counter. Mirrors T5PHASE erasable.
    /// Positive → FRESHDAP, zero → Phase2, negative → Phase1/REDAP.
    pub t5_phase: i16,
    /// HOLDFLAG: positive = attitude hold, negative = auto steer.
    /// AGC source: HOLDFLAG erasable, RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1007.
    pub hold_flag: i16,
    /// Cumulative attitude errors (ERRORX/Y/Z), scaled 180 degrees.
    pub error: [i16; 3],
}
```

### Functions

```rust
/// Process one T5RUPT tick of the RCS DAP.
///
/// Must complete in under 1 ms (no blocking, no heap, no spin-wait).
/// Called from an ISR context — caller must hold the Mutex critical section.
///
/// Reads CDU angles from `hw.imu()`, computes attitude error,
/// delegates to `rcs_logic::select_jets`, and fires `hw.rcs().fire_jets()`.
/// In TVC mode this function is a no-op; the TVC T5 chain is self-managed.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   RCSATT (Phase 1), T5PHASE2 (Phase 2), JETSLECT handoff.
pub fn t5rupt_tick(dap: &mut Dap, hw: &mut dyn AgcHardware);

/// Change the DAP operating mode.
///
/// Idle → Rcs: runs FRESHDAP initialisation (zeros rate filter variables,
///   sets T5PHASE = 0, hooks T5RUPT to RCSATT).
/// Rcs → Tvc: arms PITCHDAP/YAWDAP T5 chain; ROLLDAP scheduled via Waitlist.
/// Any → Idle: calls hw.rcs().all_jets_off(); unhooks T5RUPT.
///
/// AGC source: FRESHDAP (RCS-CSM_DIGITAL_AUTOPILOT.agc p. 1014),
///             TVCEXEC (TVCEXECUTIVE.agc p. 946),
///             TVCEXFIN (TVCEXECUTIVE.agc p. 949).
pub fn set_mode(dap: &mut Dap, mode: DapMode, hw: &mut dyn AgcHardware);
```

---

## Scale Factors

| Quantity | AGC register | AGC scale | `f64` conversion |
|---|---|---|---|
| CDU angle | CDUX/Y/Z | 2^15 counts = π rad | `CduAngle::to_radians()` |
| Attitude error | ERRORX/Y/Z | 1 unit = 180° / 2^14 | `(i16 as f64) / 32768.0 * π` |
| Angular rate (DRHO) | DRHO/1/2 | scaled 180°, DP | `(DP as f64) / 2^28 * π` |
| T5 timer reload | TIME5 | 10 ms per unit (negative count) | — |
| Min jet pulse | — | 14 ms (23 counts of TIME6) | `Met::from_centiseconds(1)` + truncation |

---

## Invariants

- **ISR-safe**: `t5rupt_tick` must never block, never allocate, and must complete in <1 ms.
  All state is in the `Dap` struct passed by exclusive reference held through
  `cortex_m::interrupt::free(|cs| ...)`. The `Dap` struct itself must NOT be exposed
  in the public API as a `Mutex<RefCell<T>>` — that is a caller responsibility.
- **No heap**: `Dap` is a `Copy`-capable struct. No `Vec`, `Box`, or `String` anywhere.
- **Mode safety**: In `Idle` mode, `t5rupt_tick` must be a no-op (jets off, no CDU read).
- **GOJAM path**: If `hw.imu()` returns an error indication (IMU tilt), `t5rupt_tick`
  must call `alarm::raise(AlarmCode::ImuTilt)` and return without firing jets.
- **Restart safety**: `T5PHASE` value must be saved to the phase table before any
  multi-step computation. Use `state.restart.set_phase(GroupId::Dap, phase)`.

---

## Test Cases

1. **Mode transition Idle → Rcs**: Call `set_mode(Idle → Rcs)`. Assert `dap.mode == Rcs`,
   `dap.t5_phase == 0` (FRESHDAP sets phase to 0 for Phase 2 startup),
   and that `hw.rcs().current_command()` is `JetCommand::OFF` at entry.

2. **Idle mode is a no-op**: Construct `Dap` with `mode = Idle`. Call `t5rupt_tick`.
   Assert that `hw.rcs().current_command()` remains `JetCommand::OFF` and no CDU read
   is issued (mock IMU read count unchanged).

3. **RCS tick calls jet selector**: Construct `Dap` in `Rcs` mode with a positive pitch
   error (ERRORX > deadband). Call `t5rupt_tick`. Assert that `hw.rcs().current_command()`
   has a non-zero `pitch_yaw` field (PJETS bits set) and zero `roll` field.

4. **TVC tick calls gimbal drive**: Construct `Dap` in `Tvc` mode. Call `t5rupt_tick`.
   Assert that `hw.engine()` receives a gimbal angle command via `set_gimbal_angles` and
   that `hw.rcs()` is NOT written (roll goes through Waitlist, not T5RUPT directly).

---

## agc-sim Impact

- `MissionState` panel: add `dap_mode: &str` field (rendered as "DAP: IDLE/RCS/TVC").
- `SimLog`: emit `.info("DAP mode → RCS")` / `.info("DAP mode → TVC")` on transitions.
- `SimHardware`: `t5rupt_tick` is called from the simulation main loop on a 100 ms
  simulated timer; no new keyboard bindings needed.
