# Functional Specification: Display Formatting

```
AGC source: Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc
Routines:   DSPDECWD, DSPDECNR, DSPDC2NR, DSP2DEC, DSPOCTWO, DSPIN, DSPSIGN,
            DSPRND, HMSOUT, M/SOUT, 2BLANK, 5BLANK, DSPDECVN
Pages:      359-364 (BANK 40/42, multiple SETLOC)

Secondary:
AGC source: Comanche055/T4RUPT_PROGRAM.agc
Routines:   DSPOUT, DSPOUTSB, DSPLAY, RELTAB, RELTAB11
Pages:      133-136 (BANK 12, SETLOC T4RUP / BLOCK 02 FFTAG12)
```

---

## 1. Behavior Summary

The AGC displays information on the DSKY (Display and Keyboard Unit) via a hardware relay system. The CPU writes to output channel 10 (OUT0) through a table called `DSPTAB` (11 words of erasable memory, `DSPTAB` through `DSPTAB+10`). The T4 interrupt (`DSPOUT`) scans this table every 20 ms and writes one relay word per pass.

### 1.1 Relay Word Encoding

From `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`, pages 313-314 and `T4RUPT_PROGRAM.agc`, pages 133-134:

```
OUT0 format:  AAAABCCCCCDDDDD
  AAAA  = relay word selector (which display pair is energised)
  B     = bit 11 of DSPTAB entry (sign bit: + or -)
  CCCCC = 5-bit relay code for the LEFT character of the pair
  DDDDD = 5-bit relay code for the RIGHT character of the pair
```

`RELTAB` (BLOCK 02, FFTAG12, page 133) is a 12-entry packed table; upper 4 bits are the relay word selector, lower 5 bits are the relay code for digit 0 (used as an offset). The 11 `DSPTAB` entries correspond to display positions:

| DSPTAB index | Relay word | Display positions |
|---|---|---|
| 10 | 1011 | MD1 (25), MD2 (24) |
| 9 | 1010 | VD1 (23), VD2 (22) |
| 8 | 1001 | ND1 (21), ND2 (20) |
| 7 | 1000 | R1D1 (16) alone |
| 6 | 0111 | +R1 sign, R1D2 (15), R1D3 (14) |
| 5 | 0110 | -R1 sign, R1D4 (13), R1D5 (12) |
| 4 | 0101 | +R2 sign, R2D1 (11), R2D2 (10) |
| 3 | 0100 | -R2 sign, R2D3 (7), R2D4 (6) |
| 2 | 0011 | R2D5 (5), R3D1 (4) |
| 1 | 0010 | +R3 sign, R3D2 (3), R3D3 (2) |
| 0 | 0001 | -R3 sign, R3D4 (1), R3D5 (0) |

### 1.2 Digit-to-Relay-Code Table

From `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`, page 314:

| Digit | 5-bit relay code (binary) | Hex |
|---|---|---|
| blank | 00000 | 0x00 |
| 0 | 10101 | 0x15 |
| 1 | 00011 | 0x03 |
| 2 | 11001 | 0x19 |
| 3 | 11011 | 0x1B |
| 4 | 01111 | 0x0F |
| 5 | 11110 | 0x1E |
| 6 | 11100 | 0x1C |
| 7 | 10011 | 0x13 |
| 8 | 11101 | 0x1D |
| 9 | 11111 | 0x1F |

The blank code (0x00) represents "no data" — distinct from a displayed zero (0x15). This distinction is preserved throughout the formatting routines.

### 1.3 DSPDECWD — Decimal Word Display

`DSPDECWD` (page 359): Converts `(MPAC, MPAC+1)` to a signed 5-digit decimal display. Steps:
1. `DSPSIGN` (page 359): examines MPAC sign bit. If negative, converts to magnitude and calls `-ON` (turns on minus sign relay). If positive, calls `+ON`. Sets sign relay in DSPTAB.
2. `DSPRND` (page 359): rounds by adding 5×10⁻⁶ (constant `DECROUND = OCT 02476`), to handle display truncation.
3. Loops 5 times: multiplies the magnitude by `BINCON` (10, per repeated multiply-by-10 to extract digits), extracts the integer part via `RELTAB` indexed by the digit value 0–9, writes the 5-bit relay code into DSPTAB via `DSPIN`.

`DSPDECNR` (page 360): Same as DSPDECWD but skips the rounding step.

`DSPDC2NR` (page 360): 2-digit decimal display (for verb/noun codes). No rounding, 2 iterations.

`DSPDECVN` (page 361): Displays the verb or noun code as 2-digit decimal. Pre-scales by 0.01 (constant `VNDSPCON = OCT 00244`), then uses `DSPDC2NR`.

### 1.4 DSPOCTWO — Octal Word Display

`DSPOCTWO` (page 361): Displays a 15-bit AGC word as 5 octal digits.
- Blanks sign positions (bit14 set in DSPCOUNT to suppress sign relay).
- Extracts 3 bits at a time via complement-cyclic-shift: `CS/CYL` three times to get the top 3 bits, mask with `DSPMSK = SEVEN` (3 bits), index into `RELTAB` for the relay code, write to DSPTAB.
- Repeats 5 times.

### 1.5 HMSOUT — Hours:Minutes:Seconds Display

`HMSOUT` (page 338, BANK 42): Decodes a DP AGC time value in centiseconds and displays it across R1/R2/R3.

1. Reads fresh DP data from the noun address.
2. `TPAGREE`: makes the DP pair agree in sign.
3. `SEPSECNR`: separates seconds from minutes+hours via multiply by `SECON1 = 2^12 / 6000`.
4. Seconds (mod 60) displayed in R3 via `DSPDECWD` (format `0XX.XX`).
5. `SEPMIN`: separates whole minutes from hours via multiply by `MINCON1 = 1/15`.
6. Minutes (mod 60) displayed in R2 via `DSPDECWD`.
7. Hours displayed in R1 via `DSPDECWD`.

Scale factor for the DP time word: 1 AGC unit = 1 centisecond (2^(-14) of a full-scale value = 163.84 seconds; the centisecond value is obtained by reading the raw erasable time registers TIME1/TIME2 which count in centiseconds).

### 1.6 M/SOUT — Minutes:Seconds Display

`M/SOUT` (page 339): Displays a DP time value as `MM B SS` (minutes in D1D2, blank in D3, seconds in D4D5, per SF code 01001). Limits to 59:59 (`M/SCON3`). The middle digit (RxD3) is forced blank via `DSPIN` with code=0.

### 1.7 Blank vs. Zero

The AGC explicitly distinguishes "no data" (blank relay code 0x00) from "data equals zero" (relay code 0x15). `5BLANK` / `2BLANK` write `BLANKCON = OCT 4000` to DSPTAB entries (which causes the entry to be skipped by `DSPOUT` until overwritten positively). `DSPDECWD` with a value of zero writes relay code 0x15 for each digit position.

---

## 2. Rust API

### 2.1 Module Path

`agc_core::services::display`

### 2.2 Constants

```rust
/// 5-bit relay codes for digits 0–9, indexed by digit value.
/// Blank is represented by index 10.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 314.
/// "THE 5-BIT OUTPUT RELAY CODES ARE: BLANK 00000, 0 10101, 1 00011, ..."
pub const RELAY_CODES: [u8; 11] = [
    0b10101, // 0
    0b00011, // 1
    0b11001, // 2
    0b11011, // 3
    0b01111, // 4
    0b11110, // 5
    0b11100, // 6
    0b10011, // 7
    0b11101, // 8
    0b11111, // 9
    0b00000, // blank (index 10)
];

/// Index to use for a blank digit in RELAY_CODES.
pub const BLANK: u8 = 10;

/// Sign relay: bit 11 of a DSPTAB entry represents the sign character.
/// Bit 11 on = minus sign lit.  Bit 11 off = plus sign lit (or no sign).
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 313-314 (DSPTAB format).
pub const SIGN_BIT: u16 = 1 << 10; // bit 11 (1-indexed), bit 10 (0-indexed)
```

### 2.3 Display Record Types

```rust
/// A formatted 5-digit display field (one of R1, R2, R3, VERB, NOUN, PROG).
/// Each element is an index into RELAY_CODES (0–9 for digits, 10 for blank).
///
/// Layout: [D1, D2, D3, D4, D5] left-to-right.
pub type DisplayField = [u8; 5];

/// Sign state for a register's leading sign position.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sign {
    /// Plus sign relay lit.
    Plus,
    /// Minus sign relay lit.
    Minus,
    /// Sign position blank (no sign, as for octal or verb/noun displays).
    Blank,
}

/// Complete formatted state for one R register (sign + 5 digits).
#[derive(Clone, Copy, Debug)]
pub struct RegisterDisplay {
    pub sign: Sign,
    pub digits: DisplayField,
}

/// A time value formatted across three display registers.
#[derive(Clone, Copy, Debug)]
pub struct TimeDisplay {
    /// Hours in R1 (0–999 range; 5 digits, format HHHHH or 0HHHH).
    pub r1: RegisterDisplay,
    /// Minutes in R2 (0–59; format 00MMM).
    pub r2: RegisterDisplay,
    /// Seconds in R3 (0–59.99; format SS.ss with implied decimal).
    pub r3: RegisterDisplay,
}
```

### 2.4 Formatting Functions

```rust
/// Format a signed 32-bit integer as a 5-digit decimal display field.
///
/// The value is clamped to the range [-99999, +99999].  Values outside
/// this range saturate: the display shows 99999 with the appropriate sign.
/// This matches the AGC's TESTOFUF alarm-and-recycle for out-of-range inputs,
/// re-mapped to saturation in Rust (no heap, no panic).
///
/// Justification for saturation vs. error: the developer spec must pick one.
/// Saturation is chosen here because it is safe (no UB, no panic) and the
/// astronaut can observe the saturated value and investigate.
///
/// AGC source: DSPDECWD (rounds, then extracts 5 digits).
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 359-360.
///
/// Returns: sign indicator and 5 relay-code indices (into RELAY_CODES).
pub fn format_decimal(value: i32, with_sign: bool) -> RegisterDisplay;

/// Format a 15-bit octal value as 5 octal digits (no sign).
///
/// Only the low 15 bits of `value` are used (AGC word is 15 bits + sign).
/// Sign position is left blank.
///
/// AGC source: DSPOCTWO.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 361.
pub fn format_octal(value: u16) -> RegisterDisplay;

/// Format a mission elapsed time (in centiseconds) across R1/R2/R3.
///
/// Scale: 1 unit = 1 centisecond.
/// R1 = whole hours (0–999), R2 = minutes mod 60 (0–59),
/// R3 = seconds mod 60 with two implied decimal places (0–59.99).
///
/// AGC source: HMSOUT.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 338-341 (BANK 42).
///
/// Rounds to the nearest centisecond before splitting (matches RNDCON = OCT 00062).
pub fn format_time(centiseconds: u32) -> TimeDisplay;

/// Format a minutes:seconds value (in centiseconds) for a single register.
///
/// Used by M/S (minutes/seconds) display format (SF code 01001).
/// Limits to 59:59 (5999 centiseconds) per AGC M/SCON3.
/// Returns: sign=Plus, D1D2 = minutes, D3 = blank, D4D5 = seconds.
///
/// AGC source: M/SOUT.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 339-341 (BANK 42).
pub fn format_min_sec(centiseconds: u32) -> RegisterDisplay;

/// Return a blank 5-digit display field (all digits = BLANK relay code).
///
/// AGC source: 5BLANK / 2BLANK write BLANKCON to DSPTAB.
/// PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, pages 322.
pub const fn blank() -> RegisterDisplay;

/// Convert a digit value 0–9 to its 5-bit relay code.
///
/// Returns RELAY_CODES[10] (blank) for any value outside 0–9.
///
/// AGC source: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, page 314 relay table.
pub const fn digit_to_relay(d: u8) -> u8;

/// Convert a `RegisterDisplay` to the two DSPTAB u16 words that DSPOUT would write.
///
/// This is the inverse of the DSPIN / DSPOUT formatting used in T4RUPT.
/// Used by agc-sim to render the display without running the T4 interrupt.
///
/// AGC source: DSPIN, 11DSPIN, DSPOUT.
/// T4RUPT_PROGRAM.agc, pages 134-136.
pub fn to_dsptab(display: &RegisterDisplay, dsptab_hi: &mut u16, dsptab_lo: &mut u16);
```

### 2.5 Scale Factors

| Quantity | AGC unit | `f64` SI | Notes |
|---|---|---|---|
| Centiseconds (time) | 1 count = 1/100 second | seconds × 100 | TIME2 counts centiseconds |
| Decimal display | 5-digit integer 00000–99999 | dimensionless | verb/noun specific scale applied before calling |
| Octal display | raw 15-bit AGC word | dimensionless | bits displayed directly |

---

## 3. Invariants

1. **No heap**: all return types are `Copy` structs with fixed size. No `Vec` or `String`.
2. **Deterministic**: given the same input, `format_decimal`, `format_octal`, `format_time` always return the same output.
3. **Overflow-safe**: `format_decimal` saturates at ±99999; it does not panic. Matches the intent of AGC `TESTOFUF → ALMCYCLE` (which would alarm, not crash — Rust saturation is the equivalent safe behaviour).
4. **Blank vs. zero**: `blank()` returns `[BLANK; 5]` (all indices = 10, relay code 0x00). `format_decimal(0, true)` returns `[0,0,0,0,0]` (all indices = 0, relay code 0x15). These are distinct.
5. **Digit range**: `digit_to_relay` returns relay code 0x00 for any digit outside 0–9 (i.e., it treats out-of-range as blank, not as panic).
6. **Sign handling**: `format_decimal` with `with_sign = false` returns `Sign::Blank` and a 5-digit magnitude. Used for verb/noun fields where AGC blanks the sign relay.

---

## 4. Test Cases

### Test 1: Positive decimal

```
format_decimal(12345, true)
→ sign = Plus
→ digits = [1, 2, 3, 4, 5]
→ relay codes = [0x03, 0x19, 0x1B, 0x0F, 0x1E]
```

### Test 2: Negative decimal

```
format_decimal(-7, true)
→ sign = Minus
→ digits = [0, 0, 0, 0, 7]
→ relay codes = [0x15, 0x15, 0x15, 0x15, 0x13]
```

### Test 3: Zero

```
format_decimal(0, true)
→ sign = Plus
→ digits = [0, 0, 0, 0, 0]
→ relay codes = [0x15, 0x15, 0x15, 0x15, 0x15]
// distinct from blank() which returns relay codes [0x00, 0x00, 0x00, 0x00, 0x00]
```

### Test 4: Octal

```
format_octal(0b101_011_001_111_000)
// = octal 53170 = decimal 22136
→ sign = Blank
→ digits = [5, 3, 1, 7, 0]
→ relay codes = [0x1E, 0x1B, 0x03, 0x13, 0x15]
```

### Test 5: Time formatting

```
format_time(3661 * 100)  // 1 hour, 1 minute, 1 second = 366100 centiseconds
→ r1: sign=Plus, digits=[0,0,0,0,1]  // 1 hour
→ r2: sign=Plus, digits=[0,0,0,0,1]  // 1 minute
→ r3: sign=Plus, digits=[0,1,0,0,0]  // 1.00 second (format 0SS.ss)
```

### Test 6: Blank field

```
blank()
→ sign = Blank
→ digits = [10, 10, 10, 10, 10]
→ all relay codes = 0x00
// blank() is distinct from format_decimal(0, ...) where relay code is 0x15
```

---

## 5. agc-sim Impact

- `DskyDisplayState`: replace raw `u8` digit arrays with `RegisterDisplay` for R1/R2/R3 and `DisplayField` for VERB/NOUN/PROG fields.
- `dsky_terminal.rs`: map `RegisterDisplay.digits[i]` → `RELAY_CODES[d]` → 7-segment render. Render `Sign::Minus` as `-`, `Sign::Plus` as `+`, `Sign::Blank` as ` `.
- No new keyboard bindings required.
- `SimLog`: when a `RegisterDisplay` is updated, emit `DISP  R{n}={:+06}` in decimal for human readability (using `format_decimal` internally).
