# Functional Specification: Verb Dispatch Table (PINBALL)

```
AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
Routines:   VERBFAN, VBFANDIR, VERBTAB, ENTPAS0, GOEXTVB
Pages:      325-326 (BANK 41, SETLOC PINBALL2)

Secondary:
AGC source: Comanche055/EXTENDED_VERBS.agc
Routines:   GOEXTVB, LST2FAN, V82PERF
Pages:      236-267 (BANK 7, SETLOC EXTVERBS)
```

---

## 1. Behavior Summary

After `ENTPAS0` validates the verb/noun combination, it jumps to `VERBFAN` (page 325). VERBFAN decides between regular verbs (V00–V39) and extended verbs (V40–V99):

- **Regular verbs (V00–V39)**: dispatched via `VBFANDIR` which uses `INDEX VERBREG` into `VERBTAB` (a CADR table of 40 entries) and jumps with `TC BANKJUMP`.
- **Extended verbs (V40–V99)**: `MPAC = VERBREG - 40`, then `TC POSTJUMP CADR GOEXTVB`. In `EXTENDED_VERBS.agc`, `GOEXTVB` uses `INDEX MPAC / TC LST2FAN` — another 60-entry jump table (`LST2FAN`, page 236).

```
LST2CON = DEC 40    # (page 325) first extended verb number
```

### 1.1 Regular Verb Table (VERBTAB, pages 325-326)

Selected minimum viable verbs for M5:

| Verb | AGC label | Description |
|---|---|---|
| V00 | `GODSPALM` | Illegal — operator alarm |
| V01 | `DSPA` | Display octal component 1 (R1) |
| V02 | `DSPB` | Display octal component 2 (R2) |
| V03 | `DSPC` | Display octal component 3 (R3) |
| V04 | `DSPAB` | Display octal components 1, 2 (R1, R2) |
| V05 | `DSPABC` | Display octal components 1, 2, 3 (R1–R3) |
| V06 | `DECDSP` | Decimal display |
| V07 | `DSPDPDEC` | DP decimal display (R1, R2) |
| V11 | `MONITOR` | Monitor octal component 1 (1 Hz refresh) |
| V16 | `MONITOR` | Monitor decimal (1 Hz refresh) |
| V21 | `ALOAD` | Load component 1 (R1) |
| V22 | `BLOAD` | Load component 2 (R2) |
| V23 | `CLOAD` | Load component 3 (R3) |
| V24 | `ABLOAD` | Load components 1, 2 (R1, R2) |
| V25 | `ABCLOAD` | Load components 1, 2, 3 (R1–R3) |
| V27 | `DSPFMEM` | Display fixed memory (octal) |
| V33 | `VBPROC` | Proceed without data |
| V34 | `VBTERM` | Terminate current test or load |
| V35 | `VBTSTLTS` | Test lights (lamp test) |
| V36 | `SLAP1` | Fresh start |
| V37 | `MMCHANG` | Change major mode (program) |

Verbs V08–V10, V18–V20, V26, V28–V29, V38–V39 are spare (`GODSPALM`) in Comanche055.

### 1.2 Extended Verb Table (LST2FAN, page 236)

Selected M5 extended verbs:

| Verb | AGC label | Description |
|---|---|---|
| V40 | `VBZERO` | Zero IMU CDU angles |
| V82 | `V82PERF` | Request orbit parameters display (R30) |

### 1.3 V35 Lamp Test (VBTSTLTS)

`VBTSTLTS` (referenced from VERBTAB page 326) turns on all display relays simultaneously. In the Rust implementation this calls `hw.dsky().lamp_test(true)`.

### 1.4 V34 Terminate (VBTERM)

`VBTERM` (page 367): sets `LOADSTAT = -1`, calls `KILMONON` (sets kill-monitor bit in MONSAVE1), calls `RELDSP` (releases display lock, turns off KEY REL light), calls `FLASHOFF`, then `RECALTST` (checks if ENDIDLE has a pending callback). In Rust: kill any active monitor, release flash, return control to idle.

### 1.5 V37 Change Program (MMCHANG)

`MMCHANG` (page 364): demands exactly 2 decimal digits be typed into the noun display position (reusing ND1/ND2 as the major mode entry field). On ENTER, the new program code is in A, and `MODROUTB = V37` in the service routines is called. Rust: calls `hw.dsky().set_prog(new_mm)`, then dispatches to the program module.

### 1.6 V82 Orbit Parameters (V82PERF)

`V82PERF` (`EXTENDED_VERBS.agc`, page 248): calls R30 (orbit parameters display). This calculates apogee, perigee, and TFF from the current state vector, then uses N44 with V06 to display them.

### 1.7 Verb Dispatch Error Handling

Unknown verbs and spares resolve to `GODSPALM → DSPALARM` (page 364). In NVSUB-initiated contexts, this becomes `TC POODOO / OCT 01501`. In keyboard-initiated contexts, `CHARALRM → FALTON → ENDOFJOB` (operator error light on, job exits cleanly).

---

## 2. Rust API

### 2.1 Module Path

`agc_core::services::pinball`

### 2.2 Types

```rust
use crate::AgcState;
use crate::hal::AgcHardware;

/// Result of executing a verb.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VerbResult {
    /// Verb executed successfully; no further crew action needed.
    Ok,
    /// Verb/noun combination is illegal; caller sets OPERATOR ERROR light.
    Error,
    /// Verb requests crew input; VERB/NOUN display should flash.
    /// Used by load verbs (V21–V25) and please-perform verbs (V50, V51).
    Flash,
}

/// A verb implementation function.
///
/// Corresponds to an entry in the AGC VERBTAB or LST2FAN tables.
/// All parameters are passed by reference (no closures, no heap allocation).
/// Noun is passed as a raw u8 decimal code (0–99).
///
/// AGC source: each VERBTAB entry is a CADR pointing to a routine that
/// executes under the PINBALL Executive job at priority 30000.
pub type VerbFn = fn(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;
```

### 2.3 Verb Table

```rust
/// Number of regular verb slots (V00–V39).
const REGULAR_VERB_COUNT: usize = 40;

/// Number of extended verb slots (V40–V99).
const EXTENDED_VERB_COUNT: usize = 60;

/// Static dispatch table for regular verbs V00–V39.
/// None entries correspond to spare/illegal verbs (dispatch → VerbResult::Error).
///
/// Memory cost: 40 × size_of::<Option<VerbFn>>() = 40 × 8 = 320 bytes on a 64-bit host,
/// 40 × 4 = 160 bytes on Cortex-M4F (32-bit function pointers).
/// This is acceptable for the AGC ROM equivalent.
///
/// AGC source: VERBTAB (CADR table), pages 325-326.
pub static VERB_TABLE: [Option<VerbFn>; REGULAR_VERB_COUNT];

/// Static dispatch table for extended verbs V40–V99.
/// Index 0 = V40, index 59 = V99.
///
/// Memory cost: 60 × 4 = 240 bytes on Cortex-M4F.
///
/// AGC source: LST2FAN (TC table), EXTENDED_VERBS.agc page 236.
pub static EXTENDED_VERB_TABLE: [Option<VerbFn>; EXTENDED_VERB_COUNT];
```

### 2.4 Dispatch Function

```rust
/// Dispatch a verb/noun pair to the appropriate verb function.
///
/// Implements VERBFAN logic:
///   - verb 0–39: look up VERB_TABLE[verb].
///   - verb 40–99: look up EXTENDED_VERB_TABLE[verb - 40].
///   - None entry or out-of-range: return VerbResult::Error.
///
/// This does not set the OPERATOR ERROR light; the caller (`VnState::entr_press`)
/// is responsible for that based on the returned VerbResult.
///
/// AGC source: VERBFAN, VBFANDIR, LST2CON = DEC 40.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 325.
/// EXTENDED_VERBS.agc, page 236 (GOEXTVB).
pub fn dispatch(
    verb: u8,
    state: &mut AgcState,
    hw: &mut dyn AgcHardware,
    noun: u8,
) -> VerbResult;
```

### 2.5 Individual Verb Implementations (Signatures Only)

The developer writes these as private functions and registers them in `VERB_TABLE`:

```rust
/// V01: Display noun component 1 (R1) in octal.
/// AGC: DSPA, page 331.
fn verb_01_display_octal_r1(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V06: Display noun in decimal (all components).
/// AGC: DECDSP, page 333.
fn verb_06_display_decimal(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V11: Monitor octal component 1 at 1 Hz.
/// AGC: MONITOR, pages 355-357.
fn verb_11_monitor_octal(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V16: Monitor decimal at 1 Hz.
/// AGC: MONITOR (same routine, verb number selects display format).
fn verb_16_monitor_decimal(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V21: Load R1 (ALOAD).
/// AGC: ALOAD → REQDATX → PUTCOM, pages 343-348.
fn verb_21_load_r1(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V24: Load R1 and R2 (ABLOAD).
/// AGC: ABLOAD, page 344.
fn verb_24_load_r1_r2(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V25: Load R1, R2, and R3 (ABCLOAD).
/// AGC: ABCLOAD, page 343.
fn verb_25_load_r1_r2_r3(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V27: Display fixed memory location in octal.
/// AGC: DSPFMEM, page 358.
fn verb_27_display_fixed_mem(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V34: Terminate current test or load request.
/// AGC: VBTERM, page 367.
fn verb_34_terminate(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V35: Lamp test (test all display lights).
/// AGC: VBTSTLTS, page 326 (VERBTAB entry).
fn verb_35_lamp_test(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V37: Change major mode (program).
/// AGC: MMCHANG, pages 364-365.
fn verb_37_change_program(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;

/// V82: Request orbit parameters display (R30).
/// AGC: V82PERF, EXTENDED_VERBS.agc page 248.
fn verb_82_orbit_params(state: &mut AgcState, hw: &mut dyn AgcHardware, noun: u8) -> VerbResult;
```

---

## 3. Scale Factors

No new scale factors are introduced at the verb dispatch layer. Individual verb implementations use scale factors defined by the noun's `FieldDesc` (see `specs/services-noun-table.md`). Verb dispatch is purely integer routing; all `f64` conversions happen inside the `VerbFn` body.

---

## 4. Invariants

1. **Unknown verb → `VerbResult::Error`, not a panic**: `dispatch` never panics. An out-of-range verb code (> 99) or a `None` table entry returns `VerbResult::Error`.
2. **Function pointers only**: `VERB_TABLE` and `EXTENDED_VERB_TABLE` contain `Option<fn(...)>` (function pointers). No closures, no trait objects, no heap allocation.
3. **Noun is not validated at dispatch level**: the individual `VerbFn` validates the noun if needed (e.g., V40 requires noun = 20 or 91 per `OP/INERT`). Dispatch only routes.
4. **V34 always succeeds**: `verb_34_terminate` returns `VerbResult::Ok` and always kills the active monitor, regardless of noun.
5. **V35 is noun-independent**: lamp test activates all relays regardless of `noun`.
6. **V37 requires exactly 2 digits**: `verb_37_change_program` returns `VerbResult::Flash` until 2 decimal digits are entered, then `VerbResult::Ok`.

---

## 5. Test Cases

### Test 1: V06 dispatches and displays

```
Given:  AgcState with noun 36 (MET) and TIME2 = 366100 centiseconds (1h01m01s)
Action: dispatch(6, &mut state, &mut hw, 36)
Assert:
  - VerbResult::Ok returned
  - hw.dsky().r1() == format_decimal(1, true)    // 1 hour
  - hw.dsky().r2() == format_decimal(1, true)    // 1 minute
  - hw.dsky().r3() == format_decimal(100, true)  // 1.00 second
```

### Test 2: V34 terminates monitor

```
Given:  AgcState with active monitor (MONSAVE != 0), flash=true
Action: dispatch(34, &mut state, &mut hw, 0)
Assert:
  - VerbResult::Ok
  - state.vn.flash == false
  - state.vn.mode == InputMode::Idle
  - hw.dsky().key_rel_light() == false
```

### Test 3: V35 lamp test

```
Given:  AgcState, any noun
Action: dispatch(35, &mut state, &mut hw, 0)
Assert:
  - VerbResult::Ok
  - hw.dsky().all_lights_on() == true
```

### Test 4: V37 changes program

```
Given:  AgcState in program P00
Action: dispatch(37, &mut state, &mut hw, 40)
         // noun 40 encodes program P40
Assert:
  - VerbResult::Ok
  - state.current_program == 40
  - hw.dsky().prog() == [4, 0]   // PROG display shows "40"
```

### Test 5: Unknown verb → Error

```
Given:  any AgcState
Action: dispatch(100, &mut state, &mut hw, 0)   // out of range
        dispatch(8,   &mut state, &mut hw, 0)   // V08 = spare in Comanche055
Assert:
  - VerbResult::Error in both cases
  - no panic
  - AgcState unchanged
```

---

## 6. agc-sim Impact

- `DskyDisplayState`: add `prog_display: [u8; 2]` for the PROG (major mode) two-digit field, updated by V37.
- `dsky_terminal.rs`: render `PROG` field from `prog_display`.
- `command_dispatch.rs`: after `dispatch()` returns `VerbResult::Flash`, set `DskyDisplayState.flash_vn = true`.
- `SimLog`: emit `VERB  v={:02}  n={:02}  result={:?}` for each dispatch call.
- `dsky_demo.rs` / scenario files: V37N40 auto-arms the SPS burn scenario per `AGENTS.md` simulation requirements.
