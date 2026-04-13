# Functional Specification: Verb/Noun Keyboard State Machine

```
AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
Routines:   CHARIN, CHARIN2, VERB, NOUN, ENTER, ENTPAS0, ENTPASHI,
            CLEAR, VBRELDSP, CHARALRM, FALTON, NUM, 89TEST, GETINREL
Pages:      313–329 (BANK 40, SETLOC PINBALL1)
```

---

## 1. Behavior Summary

The keyboard and display program (called "PINBALL GAME" in the AGC source) processes one 5-bit key code at a time. Each key depression activates interrupt KEYRUPT1/KEYRUPT2, which places the code into MPAC and schedules an Executive job at `CHARIN`. The main state is encoded in three AGC erasable registers: `VERBREG`, `NOUNREG`, and `DSPCOUNT`. `DSPCOUNT` is the display position counter that tracks where the next digit goes and which field (verb, noun, R1, R2, R3) is currently being entered.

### 1.1 Key Code Table

From `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`, page 313:

| AGC 5-bit code (octal) | Decimal | Key |
|---|---|---|
| 00001 | 1 | Digit 1 |
| 00002 | 2 | Digit 2 |
| ... | ... | ... |
| 01001 | 9 | Digit 9 |
| 10000 | 16 | Digit 0 |
| 10001 | 17 | VERB |
| 10010 | 18 | ERROR RESET |
| 11001 | 25 | KEY RELEASE |
| 11010 | 26 | + (plus sign) |
| 11011 | 27 | - (minus sign) |
| 11100 | 28 | ENTER |
| 11110 | 30 | CLEAR |
| 11111 | 31 | NOUN |

Codes not in the above list jump to `CHARALRM` (operator error).

### 1.2 CHARIN State Machine

`CHARIN` (page 315, BANK 40) is the entry point for every key except ERROR RESET. It:

1. Sets `DSPLOCK = 1` to block the display system during processing.
2. Checks `CADRSTOR`: if non-zero and the key is not ERROR RESET, turns on the KEY RELEASE light (`RELDSPON`) to remind the operator to re-establish the flashing display they have obscured.
3. Dispatches to one of: `NUM` (digits 0–9), `VERB`, `NOUN`, `POSGN`/`NEGSGN` (signs), `ENTERJMP` (ENTER), `CLEAR`, `VBRELDSP` (KEY REL), `ERROR` (ERROR RESET), or `CHARALRM` (illegal key).

The `INRELTAB` table (page 319) maps `DSPCOUNT` (0–19 octal) to the current input register (INREL): 0 = VERBREG, 1 = NOUNREG, 2 = XREG (R1), 3 = YREG (R2), 4 = ZREG (R3).

### 1.3 Digit Entry (NUM)

- `DECBRNCH` encodes the active sign type: +0 = octal, +1 = plus decimal, +2 = minus decimal.
- Decimal mode accumulates digits into XREG/YREG/ZREG (high word) and XREGLP/YREGLP/ZREGLP (low word) using `DECTOBIN` (10× previous + new digit, double-precision).
- Octal mode assembles 3 bits at a time via cyclic shift into the register selected by INREL.
- `89TEST` (page 316): digits 8 and 9 are rejected when `DECBRNCH` is zero (octal mode).
- When `DSPCOUNT` reaches the critical count (`CRITCON`, 5 chars for data registers), digit entry stops (DSPCOUNT goes negative).

### 1.4 VERB and NOUN Keys

`VERB` (page 319): Clears VERBREG, sets DSPCOUNT to VD1 (octal 23, decimal 19), blanks the verb display pair, sets DECBRNCH = 1 (decimal), REQRET = 0 (for ENTPAS0), ENTRET = TC ENDOFJOB.

`NOUN` (page 319): Same pattern but sets DSPCOUNT to ND1 (octal 21, decimal 17), clearing NOUNREG.

### 1.5 ENTER / ENTPAS0

`ENTER` (page 323, BANK 41, SETLOC PINBALL2): clears CLPASS, sets ENTRET = TC ENDOFJOB, then branches on REQRET:

- **REQRET positive (pass 0)** → `ENTPAS0`: executes the current verb/noun combination.
- **REQRET negative (higher pass)** → `ENTPASHI`: accepts a data word entered for a load verb. Enforces that 5 decimal characters were typed (alarms if fewer); resets REQRET positive, turns off flash.

`ENTPAS0` (page 324):
1. Clears DECBRNCH.
2. Blocks further numeric input (sets DSPCOUNT negative).
3. Tests VERBREG: if verb ≥ LOWVERB (decimal 28), skips the noun test and jumps to `VERBFAN`.
4. Otherwise reads the noun table (`LODNNTAB` via DXCH Z bank-switch) to validate the noun.
5. Mixed nouns go through `MIXNOUN`; normal nouns proceed to VERBFAN.
6. VERBFAN (page 325): verbs 00–39 indexed into `VERBTAB`; verbs 40–99 dispatched through `GOEXTVB` (EXTENDED_VERBS).

### 1.6 CLEAR

`CLEAR` (page 321): blanks R3 → R2 → R1 on successive presses (tracked by `CLPASS`). Zeroes XREG/YREG/ZREG and the corresponding decimal component bits in DECBRNCH. Uses `5BLANK` / `2BLANK` / `CLR5` to blank the display relays. Illegal if INREL is 0 or 1 (verb/noun field).

### 1.7 KEY RELEASE (VBRELDSP)

`VBRELDSP` (page 368): turns off the upact light (channel 11 bit 3). If `CADRSTOR` is full and the external monitor bit is set, unsuspends the monitor (`UNSUSPEN`). Otherwise calls `RELDSP` to release the display lock and wake any job waiting in DSPLIST.

### 1.8 Error Handling

`CHARALRM` / `FALTON` (page 364): illegal key or illegal state → sets the OPERATOR ERROR light (bit 7 of channel 11) and jumps to ENDOFJOB. The error light is cleared by the ERROR RESET key.

---

## 2. Rust API

### 2.1 Module Path

`agc_core::services::v_n`

### 2.2 Types

```rust
/// Current phase of the keyboard input state machine.
///
/// AGC source: encodes the combined state of DSPCOUNT, DECBRNCH, and REQRET.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 315-325.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InputMode {
    /// No active entry; DSPCOUNT is negative (blocked).
    Idle,
    /// VERB key was pressed; entering first or second verb digit.
    EnteringVerb,
    /// NOUN key was pressed; entering first or second noun digit.
    EnteringNoun,
    /// A load verb requested data; entering digits for register 1, 2, or 3.
    /// The u8 is the 1-based register index (1 = R1/XREG, 2 = R2/YREG, 3 = R3/ZREG).
    EnteringData(u8),
}

/// Sign mode for the current data register entry.
///
/// AGC source: DECBRNCH erasable register (low 2 bits).
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pages 315-318.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SignMode {
    /// Octal entry (DECBRNCH = +0).
    Octal,
    /// Plus decimal entry (DECBRNCH bit1 set by POSGN).
    PlusDecimal,
    /// Minus decimal entry (DECBRNCH bit2 set by NEGSGN).
    MinusDecimal,
}

/// Result returned by `char_in` after processing one key code.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CharResult {
    /// Key was accepted; state has advanced but no action is needed yet.
    Accepted,
    /// Key was illegal for the current state; OPERATOR ERROR light must be set.
    Rejected,
    /// ENTER was pressed in pass-0 and a complete verb/noun pair is ready.
    /// The caller should invoke `entr_press`.
    Complete(u8 /* verb */, u8 /* noun */),
}

/// Result returned by `entr_press` after executing the verb/noun.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EntrResult {
    /// Verb executed successfully.
    Ok,
    /// Verb/noun combination is illegal; OPERATOR ERROR light set.
    Error,
    /// Verb requires the VERB/NOUN display to flash (e.g., load verb awaiting data).
    Flash,
}

/// All keyboard and display state for PINBALL.
///
/// AGC equivalents: VERBREG, NOUNREG, XREG/YREG/ZREG, XREGLP/YREGLP/ZREGLP,
/// DSPCOUNT, DECBRNCH, REQRET, CLPASS, DSPLOCK, CADRSTOR, MONSAVE.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc pages 309-312 (erasable assignments).
pub struct VnState {
    /// Two-digit verb buffer (0–99).  None = not yet entered.
    pub verb_buf: Option<u8>,
    /// Two-digit noun buffer (0–99).  None = not yet entered.
    pub noun_buf: Option<u8>,
    /// Data buffers for R1, R2, R3 as scaled fixed-point.
    /// Matches XREG (index 0), YREG (index 1), ZREG (index 2).
    /// None = register not yet loaded.
    pub data_buf: [Option<i32>; 3],
    /// Current input mode; encodes the DSPCOUNT / REQRET state.
    pub mode: InputMode,
    /// Sign mode for the active data entry (mirrors DECBRNCH low 2 bits).
    pub sign_mode: SignMode,
    /// True when the VERB/NOUN lights should flash (set by load verbs and FLASHON).
    pub flash: bool,
    /// Digit count within the current field (0–5 for data, 0–2 for verb/noun).
    /// Mirrors DSPCOUNT decremented form.
    pub digit_count: u8,
    /// Number of successive CLR presses on the current entry (mirrors CLPASS).
    pub clpass: u8,
}
```

### 2.3 Public Functions

```rust
impl VnState {
    /// Construct the fresh-start initial state (all registers zeroed/blank).
    ///
    /// AGC source: FRESH_START_AND_RESTART.agc, STARTSUB — initialises VERBREG,
    /// NOUNREG, DSPCOUNT, DECBRNCH, REQRET, CLPASS, DSPLOCK, CADRSTOR to zero.
    pub const fn new() -> Self;

    /// Process one 5-bit key code from the DSKY keyboard.
    ///
    /// Implements `CHARIN` (page 315) through the full dispatch table.
    /// Key codes: digits 1–9 = codes 1–9, digit 0 = code 16 (octal 20),
    /// VERB = 17, NOUN = 31, ENTER = 28, CLR = 30, KEY REL = 25,
    /// + = 26, - = 27.  All others return `CharResult::Rejected`.
    ///
    /// AGC source: CHARIN, CHARIN2, NUM, 89TEST, VERB, NOUN, POSGN, NEGSGN.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 315-321.
    pub fn char_in(&mut self, key: u8) -> CharResult;

    /// Execute the current verb/noun pair (ENTPAS0 path).
    ///
    /// Called after `char_in` returns `CharResult::Complete`.  Dispatches
    /// through VERBFAN → VERBTAB (verbs 0–39) or GOEXTVB (verbs 40–99).
    ///
    /// AGC source: ENTER, ENTPAS0, VERBFAN, VERBTAB.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 323-326.
    pub fn entr_press(
        &mut self,
        state: &mut AgcState,
        hw: &mut dyn AgcHardware,
    ) -> EntrResult;

    /// Handle the CLR key (CLEAR routine, page 321).
    ///
    /// Successive calls blank R3, R2, R1.  Illegal when mode is Idle,
    /// EnteringVerb, or EnteringNoun (those call CHARALRM in the AGC).
    ///
    /// AGC source: CLEAR, CLR5, 5BLANK, LEGALTST.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 321-322.
    pub fn clr_press(&mut self);

    /// Handle the KEY REL key (VBRELDSP routine, page 368).
    ///
    /// Releases the display lock, turns off the KEY RELEASE light, and
    /// re-enables monitor operation if one was suspended.
    ///
    /// AGC source: VBRELDSP, RELDSP, RELDSP1.
    /// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 368.
    pub fn key_rel_press(&mut self);
}
```

### 2.4 Scale Factors

- `verb_buf` and `noun_buf` are raw decimal codes, no scaling (integers 0–99).
- `data_buf[i]` stores the decoded integer value in AGC units (the calling verb's `VerbFn` performs any SI conversion using the noun's `FieldDesc.scale`).
- Digit accumulation is ×10 per digit for decimal (DECTOBIN, page 317), ×8 for octal.

---

## 3. Invariants

1. **`InputMode` transitions are deterministic**: VERB key → `EnteringVerb`; NOUN key → `EnteringNoun`; load verb dispatch → `EnteringData(r)`; CLR when Idle → no-op (not an error in Rust, mirroring AGC LEGALTST).
2. **Illegal key codes set OPERATOR ERROR**: `char_in` returns `CharResult::Rejected` and the caller is responsible for asserting the error light via `hw.dsky().set_error_light(true)`.
3. **Buffers are cleared on CLR**: after `clr_press()`, `data_buf[r]` for the current register is `None` and `digit_count` resets.
4. **No heap**: all fields in `VnState` are fixed-size. No `Vec`, `String`, or `Box` is permitted.
5. **5-digit maximum for data entry**: when `digit_count` reaches 5 (matching `CRITCON` = decimal 18 counting down from R1D1 = decimal 14), further digit keys return `CharResult::Rejected` without modifying state.
6. **Digit 8/9 are rejected in octal mode** (matches `89TEST`, page 316).
7. **No nested ENTR**: `entr_press` is only valid in `InputMode::Idle` (after digit entry blocked DSPCOUNT). Calling it in any other mode returns `EntrResult::Error`.

---

## 4. Test Cases

### Test 1: Verb entry

```
Given:  VnState::new()
Action: char_in(17)       // VERB key
        char_in(3)        // digit '3'
        char_in(4)        // digit '4'
Assert:
  - verb_buf == Some(34)
  - mode == InputMode::Idle (DSPCOUNT blocked after 2nd digit)
  - CharResult::Accepted returned for both digit presses
```

### Test 2: Noun entry

```
Given:  VnState::new()
Action: char_in(31)       // NOUN key
        char_in(3)        // digit '3'
        char_in(6)        // digit '6'
Assert:
  - noun_buf == Some(36)
  - mode == InputMode::Idle
```

### Test 3: Data entry (R1 decimal, plus sign)

```
Given:  VnState with verb=21 (ALOAD), noun=36, mode=EnteringData(1)
Action: char_in(26)       // '+' sign
        char_in(1), char_in(2), char_in(3), char_in(4), char_in(5)  // digits 12345
Assert:
  - data_buf[0] == Some(12345)
  - sign_mode == SignMode::PlusDecimal
  - digit_count == 5
  - next char_in(1) → CharResult::Rejected (DSPCOUNT blocked)
```

### Test 4: Bad input (illegal key code)

```
Given:  VnState::new(), mode == Idle
Action: char_in(0)        // code 0 → CHARALRM in AGC (maps to position 0 in dispatch, TC CHARALRM)
Assert:
  - CharResult::Rejected
  - VnState unchanged (no digit stored)
```

### Test 5: CLR press

```
Given:  VnState with data_buf = [Some(42), Some(7), None], mode=EnteringData(1)
Action: clr_press()
Assert:
  - data_buf[1] == None   (R2/YREG cleared, matching 5BLANK on YREG)
  - clpass incremented
  - digit_count == 0
```

---

## 5. agc-sim Impact

- `DskyDisplayState`: add `error_light: bool`, `key_rel_light: bool`, `flash_vn: bool`.
- `dsky_terminal.rs`: render the OPERATOR ERROR and KEY RELEASE indicator cells in the lights row.  Flash the VERB/NOUN display fields when `flash_vn` is true (toggle once per render tick).
- `SimLog`: emit `VN  key=0x{:02X}  result={:?}` on each `char_in` call.
- `command_dispatch.rs`: map keyboard events to the 5-bit AGC key codes per the table in section 1.1, then call `vn_state.char_in(code)`.
