# Functional Specification: Program Alarm System

```
AGC source: Comanche055/ALARM_AND_ABORT.agc
Routines:   ALARM, ALARM2, BAILOUT, POODOO, VARALARM, CURTAINS, PROGLARM, BORTENT
Pages:      1493-1496
```

Secondary references:
- `EXECUTIVE.agc` pp. 1211-1212 (codes 1201, 1202 raised via `BAILOUT`)
- `WAITLIST.agc` pp. 1222-1223 (codes 1203, 1204 raised via `POODOO`/`BAILOUT`)
- `IMU_MODE_SWITCHING_ROUTINES.agc` p. 1441-1442 (code 1210 raised via `POODOO`)
- `FRESH_START_AND_RESTART.agc` pp. 186-189 (code 1107, 1110 behaviour)
- `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` p. various (code 1206 via `POODOO`)
- `ERASABLE_ASSIGNMENTS.agc` line 1721 (`FAILREG ERASE +2` — 3-word ring)

---

## 1. Behavior Summary

The AGC alarm system allows any routine, whether running under Executive control or
inside an interrupt handler, to record a non-fatal or semi-fatal condition and signal
the crew via the DSKY PROG (program alarm) light.

### 1.1 Alarm Registers (FAILREG)

The AGC maintains three consecutive erasable-memory words: `FAILREG`, `FAILREG+1`,
and `FAILREG+2`. Together they form a 3-slot history buffer. The fill algorithm is
linear, not circular wrap-around at the hardware level:

- If `FAILREG` is zero, the new code goes into `FAILREG` and the PROG light is lit
  (`PROGLARM`).
- Else if `FAILREG+1` is zero, the new code goes into `FAILREG+1` (no additional
  light change — `MULTEXIT`).
- Else the new code ORs with `BIT15` and is stored in `FAILREG+2` (`MULTFAIL`),
  which acts as an overflow indicator.

`ERASABLE_ASSIGNMENTS.agc` line 1721: `FAILREG ERASE +2  # B(3)PRM 3 ALARM CODE REGISTERS`

The Rust implementation models this as a ring buffer of fixed capacity 3, consistent
with the 3-register AGC hardware layout. On overflow the oldest slot is overwritten
(the AGC ORed BIT15 into slot 2 to mark overflow; the Rust implementation tracks
the overflow flag separately in `AlarmState`).

### 1.2 Entry Points

| AGC label | Rust equivalent | Caller constraint |
|---|---|---|
| `ALARM` | `raise(code)` on `AlarmState` | ISR-safe, no alloc |
| `ALARM2` | internal path — same `raise()` | not exposed |
| `BAILOUT` | `raise_bailout(code)` | Executive/Waitlist overflow |
| `POODOO` | `raise_poodoo(code)` | Interpretive-language overflow |
| `VARALARM` | `raise(code)` (same function) | ISR-safe, no alloc |
| `CURTAINS` | unused in Milestone 1 | — |

**ALARM** (p. 1493): Called with `TC ALARM` / `OCT NNNNN`. It is callable from both
interrupt and foreground contexts (`INHINT` at entry, `RELINT` at exit via `MULTEXIT`).
It records the code into `FAILREG[0..2]`, sets the PROG light if this is the first
alarm since last reset, then returns to the caller.

**BAILOUT** (p. 1495): Called when a subsystem cannot continue safely. Stores
erasables for debugging, records the alarm code via `BORTENT`, then transfers to
`WHIMPER`, which resumes at `ENEMA` (software restart). Severity: forces a software
restart (warm restart) of all programs.

**POODOO** (p. 1496): Similar to BAILOUT but checks the `V37FLBIT` flag first;
if Average-G (SERVICER) is running, it diverts to BAILOUT instead of the normal
POODOO path. The normal POODOO path clears `STATEFLG`, `REINTFLG`, `NODOFLAG`,
calls `MR.KLEAN` (zeroes phase tables), then falls through to `WHIMPER` — effectively
a software restart.

**VARALARM** (p. 1496): An alternate alarm entry where the code arrives in the A
register rather than inline in the instruction stream. Turns on the PROG light but
does not display. Used for computed alarm codes.

### 1.3 PROG Light Mechanism

`PROGLARM` (p. 1494):
```
CS  DSPTAB +11D
MASK OCT40400        # bits 8 (PROG alarm) and 9 (TEST ALARM)
ADS DSPTAB +11D
```
`OCT40400` = octal 40400, which is bits 8 and 9 set. The relevant bit for the PROG
light on DSPTAB+11 is bit 9 (0-indexed from 1). In the Rust HAL, the DSKY trait's
`set_prog_light(on: bool)` call maps to this bit.

### 1.4 Alarm Code Catalogue

The following codes are confirmed in the Comanche055 source files present in this
repository. All codes are octal.

| Code (octal) | Decimal | Rust variant | Raised from | Severity | AGC label/source |
|---|---|---|---|---|---|
| 01201 | 641 | `NoVacArea` | Executive `FINDVAC2` when all 5 VAC areas in use | BAILOUT (restart) | `EXECUTIVE.agc` line 146 |
| 01202 | 642 | `NoCoreSets` | Executive `NOVAC3` when all 7 core sets allocated | BAILOUT (restart) | `EXECUTIVE.agc` line 205 |
| 01203 | 643 | `WaitlistOverflow` | Waitlist `WTABORT` — more than 9 tasks scheduled | BAILOUT (restart) | `WAITLIST.agc` lines 339-340 |
| 01204 | 644 | `WaitlistNegDt` | Waitlist `WATLST0-` — DT <= 0 | POODOO (soft restart) | `WAITLIST.agc` line 152 |
| 01206 | 646 | `DspDoubleSleep` | Pinball `DSPABORT` — two jobs sleeping on display | POODOO | `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` line 3173 |
| 01210 | 648 | `DeviceConflict` | IMU mode switching `MODABORT`/`GOMANUR` — two jobs using same device | POODOO | `IMU_MODE_SWITCHING_ROUTINES.agc` lines 831, 848 |
| 01107 | 583 | `PhaseTableError` | `GOPROG3/PTBAD` — phase table complement mismatch on restart | ALARM + DOFSTART | `FRESH_START_AND_RESTART.agc` line 468 |
| 01520 | 848 | `MmChangeNotAllowed` | `V37/CANTR00` — V37 invoked while NODOFLAG set | ALARM (continue) | `FRESH_START_AND_RESTART.agc` line 872 |
| 01103 | 579 | `CcsHole` | `CCSHOLE` in fixed-fixed — internal consistency error | POODOO | `ALARM_AND_ABORT.agc` line 204 |
| 00206 | 134 | `ImuZeroInGimbalLock` | `IMUZERO` — attempt to zero IMU CDUs while in gimbal lock + coarse align | ALARM (continue) | `IMU_MODE_SWITCHING_ROUTINES.agc` line 66 |
| 00217 | 143 | `Curtains` | `CURTAINS` — internal safety check | ALARM (continue) | `ALARM_AND_ABORT.agc` line 208 |
| 01110 | 584 | `RestartNoActiveGroups` | Documented in `FRESH_START_AND_RESTART.agc` header; raised via `GOTOPOOH` path when no restart groups are active after GOPROG3 completes group scan | Informational, falls through to DUMMYJOB/P00 | `FRESH_START_AND_RESTART.agc` p. 182, 134 |

**Codes 1201 and 1202 confirmed present** in `EXECUTIVE.agc` lines 146 and 205.
**Code 1203 confirmed present** in `WAITLIST.agc` line 340.
**Code 1204 confirmed present** in `WAITLIST.agc` line 152.
**Code 1210 confirmed present** in `IMU_MODE_SWITCHING_ROUTINES.agc` lines 831 and 848.
**Code 1211 (ErasableChecksum) is not raised** by any `.agc` file in this repository.
It is listed in `docs/agc-reference-constants.md` but there is no `OCT 1211` in any
of the digitised source files. This code may correspond to the ERASCHK self-test
path in `FRESH_START_AND_RESTART.agc` (pp. 186-187) which forces `DOFSTART` rather
than raising an explicit alarm. The Rust implementation should reserve the variant
but not raise it from any core path in Milestone 1.

**Code 1110** is documented in the FRESH_START_AND_RESTART.agc header as "RESTART
WITH NO ACTIVE GROUPS" but no explicit `OCT 1110` instruction exists in the source.
The equivalent path goes through GOTOPOOH which flashes V50N07. The variant is
reserved for the Rust enum.

### 1.5 Severity Classification

| Severity | AGC mechanism | Rust response |
|---|---|---|
| **Continue** | `TC ALARM` / `TC VARALARM` — records code, lights PROG, returns | Record in ring buffer, light PROG, return |
| **Soft-restart (POODOO)** | Phase table clear + WHIMPER + ENEMA | Call `restart()` (warm restart path) |
| **Bailout-restart** | VAC5STOR + WHIMPER + ENEMA | Call `restart()` (warm restart path) |
| **Fresh-start** | DOFSTART invocation | Call `fresh_start()` |

---

## 2. Rust API

### 2.1 Module Path

`agc_core::services::alarm`

### 2.2 Types

```rust
/// Program alarm code.  Values are the octal constants from ALARM_AND_ABORT.agc
/// and the files that raise them.  The discriminant is the octal value cast to u16.
///
/// AGC source: Comanche055/ALARM_AND_ABORT.agc, EXECUTIVE.agc, WAITLIST.agc,
///             IMU_MODE_SWITCHING_ROUTINES.agc, FRESH_START_AND_RESTART.agc
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum AlarmCode {
    /// No VAC area available (Executive overflow).
    /// EXECUTIVE.agc: FINDVAC2, `TC BAILOUT / OCT 1201`
    NoVacArea            = 0o1201,

    /// No core set available (Executive overflow).
    /// EXECUTIVE.agc: NOVAC3, `TC BAILOUT / OCT 1202`
    NoCoreSets           = 0o1202,

    /// Waitlist overflow — more than 9 tasks.
    /// WAITLIST.agc: WTABORT, `TC BAILOUT / OCT 1203`
    WaitlistOverflow     = 0o1203,

    /// Waitlist called with zero or negative delta-T.
    /// WAITLIST.agc: WATLST0-, `TC POODOO / OCT 1204`
    WaitlistNegDt        = 0o1204,

    /// Two jobs attempting to sleep on DSKY simultaneously.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc: DSPABORT, `TC POODOO / OCT 1206`
    DspDoubleSleep       = 0o1206,

    /// Phase table complement mismatch on restart.
    /// FRESH_START_AND_RESTART.agc: PTBAD, `TC ALARM / OCT 1107`
    PhaseTableError      = 0o1107,

    /// Restart with no active restart groups.
    /// FRESH_START_AND_RESTART.agc: documented in header, p. 182 and 134.
    RestartNoActiveGroups = 0o1110,

    /// Two programs contending for same IMU/attitude device.
    /// IMU_MODE_SWITCHING_ROUTINES.agc: MODABORT/GOMANUR, `TC POODOO / OCT 1210`
    DeviceConflict       = 0o1210,

    /// Erasable memory checksum failure (reserved; not raised in Milestone 1).
    /// Would be raised by ERASCHK/SELFCHK path. No explicit OCT 1211 in source.
    ErasableChecksum     = 0o1211,

    /// V37 major-mode change attempted while NODOFLAG is set.
    /// FRESH_START_AND_RESTART.agc: CANTR00, `TC ALARM / OCT 1520`
    MmChangeNotAllowed   = 0o1520,

    /// Internal consistency error (CCSHOLE trap).
    /// ALARM_AND_ABORT.agc: CCSHOLE, OCT1103
    CcsHole              = 0o1103,

    /// IMU CDU zero attempted while in gimbal lock + coarse align.
    /// IMU_MODE_SWITCHING_ROUTINES.agc: IMUZERO, `TC ALARM / OCT 00206`
    ImuZeroInGimbalLock  = 0o0206,

    /// CURTAINS safety alarm.
    /// ALARM_AND_ABORT.agc: CURTAINS, OCT217
    Curtains             = 0o0217,
}

impl AlarmCode {
    /// Return the raw octal u16 value of this alarm code.
    pub const fn value(self) -> u16 {
        self as u16
    }
}

/// Severity of an alarm code, determining the restart response.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlarmSeverity {
    /// Record in history and light PROG; execution continues.
    Continue,
    /// Soft restart: clear phase tables, run ENEMA (warm restart).
    SoftRestart,
    /// Bailout: preserve erasables snapshot, run ENEMA (warm restart).
    Bailout,
}

impl AlarmCode {
    pub const fn severity(self) -> AlarmSeverity {
        match self {
            AlarmCode::NoVacArea
            | AlarmCode::NoCoreSets
            | AlarmCode::WaitlistOverflow => AlarmSeverity::Bailout,

            AlarmCode::WaitlistNegDt
            | AlarmCode::DspDoubleSleep
            | AlarmCode::DeviceConflict
            | AlarmCode::ErasableChecksum
            | AlarmCode::CcsHole => AlarmSeverity::SoftRestart,

            AlarmCode::PhaseTableError
            | AlarmCode::RestartNoActiveGroups
            | AlarmCode::MmChangeNotAllowed
            | AlarmCode::ImuZeroInGimbalLock
            | AlarmCode::Curtains => AlarmSeverity::Continue,
        }
    }
}
```

### 2.3 Alarm History State

```rust
/// Fixed-size alarm history ring buffer, matching FAILREG (3 words) in AGC
/// erasable memory.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc line 1721
///   `FAILREG ERASE +2  # B(3)PRM 3 ALARM CODE REGISTERS`
///
/// Capacity is 3, matching the three FAILREG registers.  On the fourth alarm
/// the oldest entry is evicted and `overflow` is set.  This differs slightly
/// from the AGC's BIT15-OR-into-slot-2 approach: the ring-buffer model is more
/// legible in Rust while preserving the invariant that no more than 3 recent
/// codes are held without heap allocation.
pub struct AlarmState {
    /// Ring buffer: entries[head % 3] is the oldest slot.
    entries: [Option<AlarmCode>; 3],
    /// Index of next write position (0..=u8::MAX, modulo 3).
    head: u8,
    /// Number of valid entries currently stored (saturates at 3).
    count: u8,
    /// Set true once more than 3 alarms have been raised without a reset.
    /// Mirrors the AGC BIT15 marker written into FAILREG+2.
    overflow: bool,
    /// True when the PROG alarm light should be illuminated.
    prog_light: bool,
}
```

### 2.4 Global Static and ISR-Safe Access

```rust
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

/// Module-level singleton, matching the AGC's single FAILREG bank.
/// Access always through `cortex_m::interrupt::free(|cs| ALARM_STATE.borrow(cs).borrow_mut()...)`.
pub static ALARM_STATE: Mutex<RefCell<AlarmState>> = Mutex::new(RefCell::new(AlarmState::new()));
```

The static is never exposed directly in the public API. All callers use the
`raise()` function, which handles the critical section internally.

### 2.5 Public Functions

```rust
impl AlarmState {
    /// Construct the initial (all-clear) state.
    /// Called during `fresh_start` to satisfy the invariant that all alarms are
    /// clear after a fresh start.
    pub const fn new() -> Self;

    /// Record an alarm and illuminate the PROG light.
    ///
    /// - ISR-safe: enters a `cortex_m::interrupt::free` critical section.
    /// - Never allocates.
    /// - Stores code in the ring buffer (overwrites oldest on overflow).
    /// - Sets `prog_light = true` on the first alarm since reset.
    ///
    /// AGC source: ALARM_AND_ABORT.agc, ALARM routine, p. 1493.
    ///
    /// Returns the severity of the alarm so the caller can decide whether
    /// to trigger a restart.  The caller is responsible for acting on
    /// `AlarmSeverity::Bailout` / `AlarmSeverity::SoftRestart` (by calling
    /// `services::fresh_start::restart`).
    pub fn raise(code: AlarmCode) -> AlarmSeverity;

    /// Clear all alarm registers and extinguish the PROG light.
    ///
    /// Called by `fresh_start()` and `restart()` (via SKIPSIM path in source).
    /// AGC source: FRESH_START_AND_RESTART.agc SKIPSIM block, lines 159-163.
    pub fn clear_all();

    /// Return the most recent alarm code recorded, or `None` if history is empty.
    pub fn most_recent() -> Option<AlarmCode>;

    /// Return a copy of the current ring buffer contents (oldest first).
    /// Used by the DSKY display and the sim TUI.
    pub fn history() -> [Option<AlarmCode>; 3];

    /// Return whether the PROG alarm light should be on.
    pub fn prog_light_on() -> bool;
}
```

All four query functions (`most_recent`, `history`, `prog_light_on`, `clear_all`)
also use `cortex_m::interrupt::free` so they are safe to call from any context.

The free-function wrappers exported from the module call through to `AlarmState`:

```rust
/// Raise a program alarm.  Convenience wrapper around AlarmState::raise.
///
/// AGC source: ALARM_AND_ABORT.agc, ALARM entry point, p. 1493.
pub fn raise(code: AlarmCode) -> AlarmSeverity { AlarmState::raise(code) }

/// Clear all alarms.  Called during fresh_start and restart initialisation.
///
/// AGC source: FRESH_START_AND_RESTART.agc, SKIPSIM block, lines 159-163.
pub fn clear_all() { AlarmState::clear_all() }
```

---

## 3. Scale Factors

No fixed-point arithmetic is involved. `AlarmCode` discriminants are `u16` octal
values matching the AGC source constants directly. No scaling is required.

---

## 4. Invariants

### 4.1 ISR Safety
`raise()` must execute entirely within a `cortex_m::interrupt::free` critical
section. It must not call any function that could re-enable interrupts (`RELINT`
equivalent) during the history-update phase.

### 4.2 No Allocation
Neither `raise()` nor any query function may call any allocator. The ring buffer
is a `[Option<AlarmCode>; 3]` — stack-allocated, size known at compile time.

### 4.3 No `Mutex<T>` in Public API
`ALARM_STATE` is `pub(crate)` at most. Callers always use the free functions
(`raise`, `clear_all`, `most_recent`, `history`, `prog_light_on`).

### 4.4 PROG Light Consistency
The PROG light field `prog_light` in `AlarmState` is `true` if and only if at
least one alarm has been recorded since the last `clear_all()`. No alarm code
ever clears the light; only `clear_all()` does.

### 4.5 Post-clear Invariant
After `clear_all()`: `entries == [None; 3]`, `head == 0`, `count == 0`,
`overflow == false`, `prog_light == false`.

### 4.6 Overflow Marking
When `count` would exceed 3 before decrement, `overflow` is set to `true` and
the oldest entry is evicted. The PROG light remains on.

### 4.7 Panic-abort
`panic = "abort"` is set. A panic in `raise()` triggers GOJAM (hardware restart).
Because GOJAM may re-enter `raise()` (via restart alarms), `raise()` must be
re-entrant with respect to the critical section (cortex-m guarantees CS nesting
is safe if PRIMASK is already set).

---

## 5. Test Cases

### Test 1: Basic alarm raise and PROG light

```
Given:  fresh AlarmState (all-clear).
Action: raise(AlarmCode::NoCoreSets).
Assert:
  - most_recent() == Some(AlarmCode::NoCoreSets)
  - prog_light_on() == true
  - history() == [Some(AlarmCode::NoCoreSets), None, None]
  - severity returned == AlarmSeverity::Bailout
```

### Test 2: Ring buffer holds three entries; overflow on fourth

```
Given:  fresh AlarmState.
Action: raise(AlarmCode::WaitlistOverflow)       // slot 0
        raise(AlarmCode::WaitlistNegDt)          // slot 1
        raise(AlarmCode::DeviceConflict)         // slot 2
        raise(AlarmCode::PhaseTableError)        // triggers overflow
Assert:
  - overflow == true
  - history()[0] == Some(AlarmCode::WaitlistNegDt)   // oldest evicted
  - history()[1] == Some(AlarmCode::DeviceConflict)
  - history()[2] == Some(AlarmCode::PhaseTableError)
  - most_recent() == Some(AlarmCode::PhaseTableError)
  - prog_light_on() == true
```

### Test 3: clear_all resets every field

```
Given:  AlarmState after raising two alarms (WaitlistOverflow, NoCoreSets).
Action: clear_all().
Assert:
  - history() == [None, None, None]
  - most_recent() == None
  - prog_light_on() == false
  - overflow == false   // verified via a subsequent raise
```

---

## 6. agc-sim Impact

### 6.1 DskyDisplayState

Add a field to `dsky_state::DskyDisplayState`:

```rust
/// True when the PROG alarm indicator should be lit.
pub prog_alarm: bool,
/// The most recent alarm code to display in the alarm readout area, or 0.
pub last_alarm_code: u16,
```

### 6.2 TUI Rendering (dsky_terminal.rs)

The "lights" row of the DSKY panel must include a `PROG` indicator cell:
- When `prog_alarm` is `true`, render `[PROG]` with reverse-video or a highlight colour.
- When `false`, render `[ -- ]` or a dim placeholder.

Below the lights row (or in the Mission Log panel), show the most recent alarm code
in four-digit octal: e.g. `ALARM 1202`.

### 6.3 SimLog

When `AlarmState::raise` is called, the sim should emit a log entry:
```
ALARM  raised  code=01202  severity=Bailout
```

### 6.4 No New Keyboard Bindings

No additional DSKY key bindings are required for the alarm system.
The RSET (Reset) key handler should call `alarm::clear_all()` when it runs
`DOFSTART`/fresh-start, consistent with the AGC source behaviour in `SKIPSIM`.
