# Functional Specification: Fresh Start and Restart Sequences

```
AGC source: Comanche055/FRESH_START_AND_RESTART.agc
Routines:   SLAP1, DOFSTART, GOPROG, GOPROG3, ENEMA, STARTSUB, STARTSB2, MR.KLEAN
Pages:      181-210
```

Secondary references:
- `EXECUTIVE.agc` pp. 1219-1220 (`DUMMYJOB` idle loop, `ENDOFJOB`)
- `IMU_MODE_SWITCHING_ROUTINES.agc` pp. 1425-1426 (`SETCOARS` — coarse align)
- `ALARM_AND_ABORT.agc` pp. 1495-1496 (`POODOO`, `WHIMPER`, `ENEMA` linkage)
- `ERASABLE_ASSIGNMENTS.agc` lines 1721 (`FAILREG`), various (`MODREG`, phase tables)

---

## 1. Behavior Summary

The AGC defines two distinct initialization paths that share common subroutines.
Understanding which state is preserved and which is zeroed is the central concern
for the Rust port.

### 1.1 Vocabulary

| AGC term | Meaning |
|---|---|
| **FRESH START** | Cold initialization. All user state is erased. Triggered by crew via `VERB 36 ENTER` or power-on at location 4000 (GOJAM) with unrecoverable conditions |
| **RESTART** (GOPROG) | Warm restart triggered by a GOJAM interrupt when the AGC believes erasable memory is intact. Phase tables guide resumption of interrupted programs |
| **ENEMA** | Software restart triggered by a major-mode change (`V37`) or by `BAILOUT`/`POODOO` through `WHIMPER`. Also a warm restart but initiated from within software |
| **GOJAM** | Hardware-generated interrupt at fixed address 4000 (octal), the AGC's panic handler entry point |
| **Phase table** | Pair of erasable words per restart group (`PHASE1`/`-PHASE1` … `PHASE6`/`-PHASE6`). Each pair holds a complemented copy; agreement proves the restart point was stored atomically |
| **DUMMYJOB** | The Executive's idle loop. Both fresh-start and restart paths end here by transferring control to `DUMMYJOB+2` which issues `RELINT` |

### 1.2 FRESH START (SLAP1 / DOFSTART)

**Source:** `FRESH_START_AND_RESTART.agc` pp. 183-185, routines `SLAP1` and `DOFSTART`.

#### Entry Points

- **SLAP1** (p. 183, line 145): Manual fresh start. Invoked by Pinball when the crew enters `VERB 36 ENTER`. Calls `STARTSUB`, then clears FAILREG/ERCOUNT/REDOCTR, then falls into `DOFSTART`.
- **DOFSTART** (p. 184, line 169): Machine-initiated fresh start. Also reached from `GOPROG` when restart conditions are unrecoverable.

#### What DOFSTART Does (Order of Operations)

1. **Zero ERESTORE and SMODE** (`TS ERESTORE`, `TS SMODE`) — must not be removed from DOFSTART (comment in source).
2. **Zero UPSVFLAG** — clear update state-vector request.
3. **Zero output channels 5 and 6** (RCS jet commands off via `WRITE CHAN5`, `WRITE CHAN6`).
4. **Zero channel 11 (DSALMOUT)** — all DSKY discrete outputs off (engine off, lights off).
5. **Zero channels 12, 13, 14** — IMU and optics outputs cleared.
6. **Zero numerous navigation-support registers**: WTOPTION, DNLSTCOD, NVSAVE, EBANKTEM, RATEINDX, TRKMKCNT, VHFCNT, EXTVBACT.
7. **IMU coarse align**: If the "NO ATT" bit was already set (DSPTAB+11 bit 4/6 pattern), the IMU is placed back into coarse align by ORing BIT6 onto CHAN12 (`SETCOARS` path). Source: lines 200-201.
8. **Call MR.KLEAN** — zero all 6 phase table pairs (groups 1-6).
9. **MODREG := -0** (CS ZERO / TS MODREG) — mode register to "no program".
10. **RESTREG := PRIO30** — display priority initialized.
11. **IMODES30 := IM30INIF** (octal 37411) — IMU mode flag init: inhibit IMU FAIL for 5 sec, set PIP ISSW.
12. **OPTIND := -1** — kill coarse optics.
13. **OPTMODES := OPTINITF** (octal 130) — optics mode init.
14. **IMODES33 := IM33INIT** (= PRIO16) — no PIP or TM fail signals.
15. **T5LOC := T5IDLER** — let T5 idle (T5 interrupt uses idle handler).
16. **Initialize FLAGWRD0-8** from SWINIT table (all zeros except specific bits preserved: NODOP01 in FLAGWRD1, REFSMMAT in FLAGWRD3, CMOONFLG/LMOONFLG/SUFFLAG in FLAGWRD8).
17. **Transfer control to DUMMYJOB+2** via `TC POSTJUMP / CADR DUMMYJOB+2`, which issues `RELINT` and enters the Executive idle loop. P00 (idle program) is not explicitly launched as a job here — DUMMYJOB scans for pending jobs and, finding none, waits. The effect is P00-idle behavior.

#### State Zeroed by Fresh Start

| Category | Registers / Memory |
|---|---|
| Alarm registers | FAILREG, FAILREG+1, FAILREG+2, ERCOUNT, REDOCTR |
| Output channels | CHAN5, CHAN6, CHAN11 (DSALMOUT), CHAN12, CHAN13, CHAN14 |
| Navigation switches | UPSVFLAG, SMODE, ERESTORE, WTOPTION, DNLSTCOD, NVSAVE, EBANKTEM |
| All phase table groups | -PHASE1/PHASE1 through -PHASE6/PHASE6 (via MR.KLEAN) |
| Mode register | MODREG (set to -0, meaning "no program") |
| IMU and optics modes | IMODES30, IMODES33, OPTIND, OPTMODES (re-initialized, not zeroed) |
| Flag words 0-8 | Initialized from SWINIT (mostly zeros, some bits preserved — see §1.2 above) |
| Waitlist task lists | LST1[0..7] := NEG1/2, LST2[0..17] := ENDTASK (via STARTSB2) |
| Executive job table | All PRIORITY registers set to -0 (available) (via STARTSB2) |
| VAC areas | VAC1USE..VAC5USE re-initialized (via STARTSB2) |
| DSKY display registers | DSPTAB[0..10] blanked (via STARTSB2) |
| Display support | VERBREG, NOUNREG, DSPLOCK, MONSAVE, CLPASS, etc. zeroed (via STARTSB2) |

#### State NOT Cleared by DOFSTART

Fresh start preserves:
- FLAGWRD1 bit `NOP01BIT` (NODOP01 flag — ground-commanded do-not-operate flag)
- FLAGWRD3 bit 13 (REFSMMAT valid flag)
- FLAGWRD8 bits OCT6200 (CMOONFLG, LMOONFLG, SUFFLAG — mission-configuration bits)

These survive because the source explicitly `MASK`s them out before applying `SWINIT`.

### 1.3 RESTART / GOPROG (Warm Restart)

**Source:** `FRESH_START_AND_RESTART.agc` pp. 186-189, routine `GOPROG`.

GOPROG is the GOJAM handler. The AGC hardware transfers control to ROM address
4000 (octal) on any GOJAM. In the Rust port, the panic handler maps to this path.

#### GOPROG Decision Tree

```
GOPROG
  ├─ Increment REDOCTR (restart counter)
  ├─ Save Q/SUPERBNK to RSBBQ
  ├─ TC VAC5STOR (snapshot erasables for debugging)
  ├─ Read CHAN33:
  │    BIT15 (OSC FAIL) = 0? → go to BUTTONS
  │    BIT15 = 1 (power transient): → BUTTONS
  ├─ CHAN33 BIT14 (AGC WARNING) = 0? → FRESH START (NONAVKEY+1 = DOFSTART)
  └─ BUTTONS:
       ├─ TC LIGHTSET (check Mark Reject + Error Light Reset → DOFSTART if both)
       ├─ Check ERESTORE (ERASCHK integrity):
       │    ERESTORE != 0 and != valid → FRESH START
       │    ERESTORE == SKEEP7 → restore erasable memory, TC STARTSUB
       └─ ELRSKIP: warm restart continues
            ├─ Re-initialize T5 autopilot slot (FLAGWRD6 bits 14-15)
            ├─ Reset integration flags in RASFLAG
            ├─ Restore OPTMODES (preserve failure inhibits in IMODES30)
            ├─ Preserve PROG ALARM, GIMBAL LOCK, NO ATT lamps from DSPTAB+11
            ├─ If NO ATT lamp was ON: TC IBNKCALL SETCOARS (IMU → coarse align)
            ├─ If engine command was on: turn engine ON (BIT13 on DSALMOUT)
            └─ TCF GOPROG3
```

#### GOPROG3 — Common Warm-Restart Completion

Both `GOPROG` and `ENEMA` converge at `GOPROG3`:

1. **Verify phase tables** (PCLOOP): For each of 5 restart groups (NUMGRPS = 5),
   check that `PHASE_n XOR -PHASE_n == -0`. If mismatch: raise alarm 1107
   (`PhaseTableError`) and divert to DOFSTART (forced fresh start).
2. **Display major mode** (TC MMDSPLAY): Shows the current MODREG value on DSKY PROG.
3. **RCS DAP stop-rate**: If RCS DAP was running (FLAGWRD6 bits 15-14 = 01), call
   STOPRATE to zero DELCDUS, WBODYS, and BIASES.
4. **Scan all restart groups** (NXTRST loop): For each group with a non-zero PHASE,
   call `RESTARTS` to reschedule the pending job/task/longcall.
5. **If no groups active** (MPAC+6 == 0): check MODREG.
   - If MODREG == -0 (no program), call `GOTOPOOH` (request P00 via V50N07 flash).
   - If MODREG != -0, fall to ENDRSTRT (resume current program display).
6. **ENDRSTRT**: Transfer control to `DUMMYJOB+2` (`TC POSTJUMP / CADR DUMMYJOB+2`).

#### ENEMA (Software Restart)

**Source:** `FRESH_START_AND_RESTART.agc` p. 189-190, routine `ENEMA`.

ENEMA is a software-initiated warm restart triggered by V37 or by BAILOUT/POODOO
through the WHIMPER path. It:
1. Calls `LIGHTSET` (same fresh-start-if-buttons-pressed check).
2. Calls `STARTSB2` (re-initialize Waitlist, Executive, VAC, DSKY, channels 11-14).
3. Clears integration flags in RASFLAG.
4. If TVC was on, reschedules TVCEXEC task.
5. Falls through to GOPROG3.

`ENEMA` is aliased as `GOPROG2` (the source reads `GOPROG2 EQUALS ENEMA`).

### 1.4 STARTSUB and STARTSB2

**STARTSUB** (p. 190, line 504):
1. Set downlink phase pointer (DNTMGOTO := LDNPHAS1).
2. TIME3 := POSMAX (37777 octal).
3. TIME4 := POSMAX - 2 (37775).
4. TIME5 := POSMAX - 3 (37774).
5. Fall through to STARTSB2.

**STARTSB2** (p. 190, line 515):
1. Write CHAN11 (DSALMOUT): mask off UPLINK ACTY, TEMP CAUTION, KR, FLASH, OP.ERROR.
2. Write CHAN13: clear TEST ALARMS, STANDBY ENABLE.
3. Clear R21MARK, P21FLAG bits from FLAGWRD2; set SKIPVHF flag.
4. Set EBANK for E3 (for waitlist).
5. **Initialize Waitlist delta-T list**: LST1[0..7] := NEG1/2 (= -0.5 centiseconds).
6. **Initialize Waitlist 2CADR list**: LST2[0..17] := -ENDTASK (complement of ENDTASK).
   LST2 odd words := -ENDTASK+1.
7. **Clear Executive core sets**: PRIORITY[0,12,24,36,48,60,72] := -0 (available).
8. Clear NEWJOB and DSRUPTSW.
9. **Re-initialize VAC area pointers**: VAC1USE..VAC5USE set to their base addresses.
10. **Blank DSKY**: DSPTAB[0..10] cleared (loop over 11 registers).
11. Zero display support registers: DELAYLOC, R1SAVE, INLINK, DSPCNT, CADRSTOR,
    REQRET, CLPASS, DSPLOCK, MONSAVE, MONSAVE1, VERBREG, NOUNREG, DSPLIST,
    MARKSTAT, IMUCADR, OPTCADR, RADCADR, ATTCADR, LGYRO, FLAGWRD4.
12. NOUT := NOUTCON.
13. EXTVBACT: preserve bit 14, clear rest.
14. SELFRET := LESCHK (self-check return address).
15. DSPCOUNT := -VD1.
16. TC Q (return to caller).

### 1.5 MR.KLEAN

**Source:** `FRESH_START_AND_RESTART.agc` p. 185, line 264.

Zeros all 6 phase table pairs by storing NEG0 (double-word all-zeros) into
`-PHASE2/PHASE2`, `-PHASE4/PHASE4`, `-PHASE1/PHASE1`, `-PHASE3/PHASE3`,
`-PHASE5/PHASE5`, `-PHASE6/PHASE6`. Called within `DOFSTART` and also independently
(P00KLEAN, V37KLEAN variants entry at different points in MR.KLEAN).

### 1.6 P00 Handoff

Neither DOFSTART nor GOPROG3 directly schedules a P00 job. Instead both paths
transfer to `DUMMYJOB+2` which:
1. Issues `RELINT` (re-enables interrupts).
2. Sets NEWJOB := -0 (no active jobs flagged).
3. Turns off the green ACTIVITY light.
4. Enters the ADVAN scan loop: polls NEWJOB for any pending job.

When no jobs are pending (fresh start state), the computer idles in ADVAN. The
crew interacts via DSKY — entering `VERB 37 ENTER / 00 ENTER` invokes V37 which
begins the P00 initialization sequence via GOTOPOOH.

In the Rust implementation, the "idle loop" maps to the main `loop {}` of `main.rs`
which calls `executive::step()` repeatedly. After `fresh_start()` returns, the
executive's job table is empty and it enters the idle spin. P00 is considered
"running" implicitly; the MODREG is set to 00 only after the crew initiates it
through the DSKY.

---

## 2. Rust API

### 2.1 Module Path

`agc_core::services::fresh_start`

### 2.2 AgcState Dependency

Both functions receive `&mut AgcState`. `AgcState` is defined in `agc_core::lib`
(or `agc_core::state`) and owns all erasable-memory analogs:

```rust
pub struct AgcState {
    /// Phase table pairs for restart groups 1-6.
    /// Indexed as phase[n-1].value / phase[n-1].complement.
    pub restart: RestartProtection,

    /// Alarm history ring buffer.
    /// Access via services::alarm functions, not directly.
    pub alarm: AlarmState,       // or accessed via ALARM_STATE static

    /// Current major mode (MODREG).  -0 means "no program".
    pub modreg: i16,

    /// IMU mode flags word 30 (IMODES30).
    pub imodes30: u16,

    /// IMU mode flags word 33 (IMODES33).
    pub imodes33: u16,

    /// Optics mode flags (OPTMODES).
    pub optmodes: u16,

    /// Restart/restart-loop counter (REDOCTR).
    pub redoctr: u16,

    /// ERESTORE: used by ERASCHK integrity check.
    pub erestore: u16,

    /// Flag words 0-8 (FLAGWRD0..FLAGWRD8).
    pub flagwrds: [u16; 9],

    // ... other fields: waitlist state, executive state, navigation state ...
}
```

The developer should not create new fields not listed here without consulting the
spec. Fields that DOFSTART explicitly touches are listed in §1.2.

### 2.3 fresh_start

```rust
/// Execute a FRESH START (DOFSTART / SLAP1 path).
///
/// Zeros all erasable user state, re-initializes hardware channels, places
/// the IMU in coarse align, clears the alarm history, and clears the phase
/// tables.  On return, the system is in the "waiting for P00" idle state.
///
/// This function must be called once from `main` on power-on, and is also
/// called whenever an unrecoverable condition forces a cold re-initialization.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   SLAP1 (line 145), DOFSTART (line 169), STARTSUB (line 504),
///   STARTSB2 (line 515), MR.KLEAN (line 264). Pages 183-192.
///
/// # Post-conditions
///
/// - `state.alarm` is fully cleared (`alarm::clear_all()` called).
/// - All 6 restart groups have zero phase values.
/// - `state.modreg == -1` (ones-complement -0, stored as `i16::MIN` or
///   a sentinel `NO_PROGRAM` constant).
/// - IMU channel 12 has coarse-align bit (BIT4) set via `hw.imu().set_coarse_align()`.
/// - All DSKY display registers are blank.
/// - The executive job table is empty (all slots available).
/// - The Waitlist is re-initialized with ENDTASK sentinels.
/// - Output channels 5, 6, 11, 12, 13, 14 are cleared.
/// - PROG light is off.
///
/// # Parameters
///
/// - `state`: mutable reference to the entire AGC erasable-memory state.
/// - `hw`: mutable reference to the HAL hardware instance.
pub fn fresh_start<H: AgcHardware>(state: &mut AgcState, hw: &mut H);
```

### 2.4 restart

```rust
/// Execute a RESTART (GOPROG / warm-restart path).
///
/// Called from the panic handler (GOJAM equivalent) when the system believes
/// erasable memory is intact.  Verifies phase table integrity, re-initializes
/// hardware I/O, and reschedules any programs that were interrupted.
///
/// If phase table verification fails (alarm 1107), this function calls
/// `fresh_start` internally and does not return to the GOPROG3 path.
///
/// AGC source: Comanche055/FRESH_START_AND_RESTART.agc
///   GOPROG (line 290), ELRSKIP (line 340), GOPROG3 (line 410),
///   STARTSUB (line 504), STARTSB2 (line 515). Pages 186-192.
///
/// # Post-conditions (warm restart succeeded)
///
/// - Phase tables intact (all 5 groups verified).
/// - PROG light ON (restart is always alarmed implicitly by the GOJAM event).
/// - Hardware channels 11-14 re-initialized (via STARTSB2 path).
/// - T5 autopilot handler restored from FLAGWRD6.
/// - IMU left in coarse align if NO ATT lamp was on.
/// - All active restart groups are rescheduled by `executive::restart::reschedule`.
/// - `state.modreg` shown on DSKY PROG field.
/// - REDOCTR incremented by 1.
///
/// # Post-conditions (phase table failure → fresh start)
///
/// Same as `fresh_start()` post-conditions plus alarm code 1107 in history.
///
/// # Parameters
///
/// - `state`: mutable reference to the AGC erasable-memory state.
/// - `hw`: mutable reference to the HAL hardware instance.
pub fn restart<H: AgcHardware>(state: &mut AgcState, hw: &mut H);
```

### 2.5 main.rs Entry Point Semantics

`main.rs` is the bare-metal entry point (no OS, `#![no_main]` with `cortex-m-rt`):

```rust
// agc-core/src/main.rs
#[entry]
fn main() -> ! {
    // 1. Hardware initialisation (clocks, peripherals) — HAL responsibility.
    let mut hw = <TargetHardware as AgcHardware>::init();

    // 2. Construct zeroed AgcState.
    let mut state = AgcState::default();

    // 3. Execute FRESH START — mirrors SLAP1/DOFSTART in the AGC source.
    services::fresh_start::fresh_start(&mut state, &mut hw);

    // 4. Enter the Executive idle loop — mirrors DUMMYJOB in the AGC source.
    //    This loop never returns.
    loop {
        executive::scheduler::step(&mut state, &mut hw);
    }
}
```

The panic handler (GOJAM path) must call `restart()`:

```rust
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Safety: we reach here only once per GOJAM; AgcState and hardware are
    // in a partially-consistent state.  restart() is designed to handle this.
    //
    // SAFETY: we cannot take a reference to `state` here without unsafe.
    // The implementation must use a global `static Mutex<RefCell<AgcState>>`
    // for the portion of state touched by the panic handler.
    // This is consistent with the AGC's use of dedicated erasable registers
    // that are always accessible from any context (FAILREG, REDOCTR, etc.).
    unsafe {
        // Actual implementation deferred to developer (link-section concern).
    }
    loop {}
}
```

The architecture document (§2) notes that restart safety requires a `static`
accessible from the panic handler. The link-section / memory layout details are
deferred to the architect and are out of scope for this Milestone 1 spec.

### 2.6 State Preserved Through Restart

The following fields must survive a warm restart (not cleared by `restart()`):

| Field | AGC register | Why preserved |
|---|---|---|
| `flagwrds[1]` bit `NOP01BIT` | FLAGWRD1 NODOP01 | Ground-commanded do-not-operate flag |
| `flagwrds[3]` bit 13 | FLAGWRD3 REFSMMAT | Reference-frame matrix valid |
| `flagwrds[8]` bits OCT6200 | FLAGWRD8 CMOONFLG/LMOONFLG/SUFFLAG | Mission config |
| `imodes30` `IFAILINH` bits | IMODES30 | Failure inhibit bits preserved on hardware restart |
| Phase table data | PHASE1..-PHASE6 | Used by GOPROG3 to reschedule interrupted programs |
| `redoctr` | REDOCTR | Restart counter; incremented, not cleared |

State that `restart()` re-initialises (same as STARTSB2 path):

| Category | AGC registers |
|---|---|
| Waitlist task lists | LST1[0..7], LST2[0..17] |
| Executive job table | PRIORITY[0..72 step 12] |
| VAC area pointers | VAC1USE..VAC5USE |
| DSKY registers | DSPTAB[0..10], VERBREG, NOUNREG, etc. |
| Output channels | CHAN11 (partial mask), CHAN13 (partial mask) |
| Flag word bits | FLAGWRD2 bits (R21MARK, P21FLAG, set SKIPVHF) |
| Optics flags | OPTCADR, RADCADR, etc. |

---

## 3. Scale Factors

No fixed-point arithmetic is required in fresh_start or restart themselves.
Timer values written to TIME3/TIME4/TIME5 in STARTSUB are:

| AGC register | Value written | Meaning |
|---|---|---|
| TIME3 | POSMAX = 0o37777 = 16383 | Maximum countdown before first T3RUPT |
| TIME4 | 16381 (POSMAX - 2) | T4 initialisation |
| TIME5 | 16380 (POSMAX - 3) | T5 initialisation |

These are `u16` hardware register writes through the HAL timer interface. No
`f64` conversion is needed.

The IMU mode initialisation value:
- `IM30INIF = 0o37411` (octal 37411 = decimal 16137): bit pattern written to IMODES30.
- `IM33INIT = PRIO16` (the constant used for 16-priority scheduling): used as IMODES33.

---

## 4. Invariants

### 4.1 Post-fresh_start State

After `fresh_start()` returns:
- `alarm::prog_light_on() == false`
- `alarm::most_recent() == None`
- `state.restart.all_groups_zero() == true` (all 6 phase pairs are zero)
- `state.modreg` encodes "no program" (implementation uses a `NO_PROGRAM` sentinel,
  e.g., `i16::MIN` or a named constant `MODREG_NONE`)
- The IMU is in coarse align (channel 12 bit 4 = 1)
- All DSKY display fields are blank
- No jobs are active in the Executive job table
- The Waitlist is fully populated with ENDTASK sentinels

### 4.2 Post-restart State (warm restart, phase tables valid)

After `restart()` returns (no fresh start triggered):
- `state.redoctr` is one greater than before the restart
- Phase tables are unchanged from what they were before the restart
- Interrupted programs have been rescheduled by `executive::restart::reschedule`
- DSKY registers are re-blanked
- Hardware channels are re-initialized

### 4.3 Phase Table Integrity Check

`restart()` must call `executive::restart::verify_phase_tables()` before
rescheduling. If any group fails the complement check, the function must:
1. Call `alarm::raise(AlarmCode::PhaseTableError)`
2. Call `fresh_start()` (not return)

This is a tail-call from the AGC's `PTBAD` label: `TCF DOFSTART`.

### 4.4 No Heap

Neither function may call any allocator. All state is in `AgcState` (stack or
static). The only dynamic dispatch is through the `AgcHardware` trait.

### 4.5 Re-entrance and ISR Safety

`fresh_start()` and `restart()` are not ISR-safe; they must be called only from
foreground (non-interrupt) context, or from the panic handler where interrupts are
already disabled. Internally they call `alarm::clear_all()` which is ISR-safe, but
the outer functions themselves are not re-entrant.

### 4.6 Hardware Channel Side Effects

Both functions perform side-effecting writes to the HAL:
- `hw.rcs().write_channel5(0)` and `hw.rcs().write_channel6(0)` (RCS jets off)
- `hw.engine().write_dsalmout(masked_value)` (CHAN11 partial clear)
- `hw.imu().write_chan12(bits)` (IMU mode bits)
- `hw.telemetry().write_chan13(bits)`, `hw.optics().write_chan14(bits)`
- `hw.imu().set_coarse_align()` (fresh start and potentially warm restart)

These HAL calls must not be elided even if the state fields look unchanged.

---

## 5. Test Cases

### Test 1: fresh_start clears all alarms and phase tables

```
Given:  AgcState with:
          - alarm: two alarms raised (PhaseTableError, NoCoreSets)
          - restart.group[1].phase = 5 (non-zero)
          - modreg = 11 (P11 running)
        SimHardware (all outputs initially in unknown state).
Action: fresh_start(&mut state, &mut hw).
Assert:
  - alarm::prog_light_on() == false
  - alarm::most_recent() == None
  - state.restart.group[0..6] all have phase == 0 and complement == 0
  - state.modreg == NO_PROGRAM sentinel
  - hw.dsky().prog_field() == 0  (blank)
  - hw.dsky().verb_field() == 0
  - hw.dsky().noun_field() == 0
  - hw.imu().coarse_align_active() == true
  - hw.rcs().channel5() == 0
  - hw.rcs().channel6() == 0
```

### Test 2: restart with valid phase tables reschedules group and preserves REDOCTR increment

```
Given:  AgcState with:
          - state.redoctr = 3
          - restart group 2 set to phase 4 with valid complement (-4)
          - all other groups zero
          - alarm history empty
        SimHardware.
Action: restart(&mut state, &mut hw).
Assert:
  - state.redoctr == 4  (incremented)
  - executive::restart has been called for group 2 (verify via a mock/flag)
  - alarm::most_recent() == None  (no 1107 alarm)
  - DSKY blank (STARTSB2 path ran)
  - state.restart.group[1].phase still == 4 (phase preserved, not zeroed)
```

### Test 3: restart with corrupted phase table triggers fresh start (alarm 1107)

```
Given:  AgcState with:
          - restart group 3: phase = 7, complement = -6  (mismatch: should be -7)
          - alarm history empty
          - state.modreg = 40 (P40 nominally running)
        SimHardware.
Action: restart(&mut state, &mut hw).
Assert:
  - alarm::most_recent() == Some(AlarmCode::PhaseTableError)  // code 1107
  - state.modreg == NO_PROGRAM  (fresh start ran)
  - state.restart.group[2].phase == 0  (MR.KLEAN called)
  - hw.imu().coarse_align_active() == true  (fresh start completed)
  - state.redoctr was incremented (GOPROG increments before checking)
```

---

## 6. agc-sim Impact

### 6.1 TUI Banner

On `fresh_start()`, the sim must display a full-width banner in the Mission Log panel:
```
===  FRESH START  ===
```

On `restart()` (warm), emit:
```
---  RESTART (warm)  REDOCTR=N  ---
```

Both banners should include the simulated Mission Elapsed Time at the moment of
the event.

### 6.2 DskyDisplayState

Add fields:
```rust
pub in_fresh_start: bool,  // true briefly during fresh_start() execution
pub last_restart_count: u16,  // mirrors state.redoctr
```

`in_fresh_start` is used to render the PROG/VERB/NOUN fields as blank during the
transition (the AGC blanks DSPTAB before handing off to DUMMYJOB).

### 6.3 P00 Display

After `fresh_start()` completes, the PROG display must show `00` within one
Executive cycle. This happens when GOTOPOOH is invoked or when the crew enters
V37N00. The sim may pre-populate `prog_field = 0` to show P00 immediately after
fresh start for visual consistency with the AGC's MODREG=-0 state.

### 6.4 Scenario Hooks

The `--scenario launch` scenario must trigger `fresh_start()` at t=0.
All scenario start functions should call `fresh_start()` before scheduling
any navigation tasks, consistent with the AGC boot sequence.

### 6.5 F1/F2/F3 Scenario Switch

When the user presses F1/F2/F3 in the TUI to switch scenarios, the sim should
call `fresh_start()` (not `restart()`) to ensure a clean slate, then load the
new scenario's initial state. This matches the "VERB 36 ENTER" path.
