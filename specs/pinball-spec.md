# Specification: `services/pinball` — DSKY Display Formatting

**Status**: Approved for implementation (Milestone 6 Phase 3)
**Module path**: `agc-core/src/services/pinball.rs`
**HAL reference**: `specs/hal-spec.md` §6 (`Dsky::write_row`, `set_lamp`)
**State reference**: `services/display.rs` (`DskyState`)

---

## 1. Purpose

`pinball.rs` is the pure-computation display formatter. It translates
the contents of `DskyState` into a decoded `DskyFrame` structure ready
for a hardware ISR shim to push to the HAL. All conversion — f32 →
signed-decimal digits, digit → 7-segment bit pattern — lives here.

No HAL access. The bare-metal T4RUPT display shim calls
`decode_dsky(&state.dsky)` and then iterates over the resulting frame,
invoking `hw.dsky().write_row(...)` and `set_lamp(...)`. The V/N
processor and every program stays ignorant of segment encoding.

The name "pinball" follows the historical AGC nomenclature for this
layer (PINBALL_GAME_BUTTONS_AND_LIGHTS.agc).

---

## 2. Types

```rust
/// One register's decoded display state (sign + five digits).
pub struct Register {
    /// -1 (minus), +1 (plus), or 0 (blank — value was zero).
    pub sign: i8,
    /// Five decimal digits, most-significant first, each 0..=9.
    pub digits: [u8; 5],
    /// True if |value| exceeded 99999 and the display was clamped.
    pub overflow: bool,
}

/// Decoded PROG/VERB/NOUN fields (each a pair of digits).
pub struct TwoDigit {
    pub tens: u8,   // 0..=9
    pub units: u8,  // 0..=9
}

/// A fully decoded DSKY frame ready to be written to the hardware.
pub struct DskyFrame {
    pub prog: TwoDigit,
    pub verb: TwoDigit,
    pub noun: TwoDigit,
    pub r1: Register,
    pub r2: Register,
    pub r3: Register,
    /// Indicator lamps verbatim from DskyState (convenience copy).
    pub lamps: Lamps,
    /// Set when V35 lamp test is active — every lamp in the frame is forced on.
    pub lamp_test: bool,
    /// Verb/Noun flash indicator.
    pub flashing: bool,
}

pub struct Lamps {
    pub uplink_activity: bool,
    pub no_att: bool,
    pub stby: bool,
    pub key_rel: bool,
    pub opr_err: bool,
    pub restart: bool,
    pub gimbal_lock: bool,
    pub temp: bool,
    pub prog_alarm: bool,
    pub comp_acty: bool,
}
```

---

## 3. Formatting rules

### 3.1 `format_register(value: f32) -> Register`

1. **Round-to-nearest-integer**: `n = libm::round(value) as i64`.
2. **Sign extraction**:
   - `n == 0`          → `sign = 0`, blank the sign lamp.
   - `n > 0`           → `sign = +1`.
   - `n < 0`           → `sign = -1`.
3. **Magnitude**: `|n|`.
4. **Overflow**:
   - If `|n| > 99_999`: set `overflow = true`, clamp magnitude to
     `99_999` (all-9s display). Caller can use the overflow flag to
     light the OPR ERR lamp if desired; Phase 3 does not raise any
     alarm here.
5. **Digit extraction** (most-significant first):
   - `digits[0] = mag / 10_000 % 10`
   - `digits[1] = mag /  1_000 % 10`
   - `digits[2] = mag /    100 % 10`
   - `digits[3] = mag /     10 % 10`
   - `digits[4] = mag          % 10`

Non-finite input (`NaN`, `inf`) is mapped to `Register {
sign: 0, digits: [0; 5], overflow: true }` — a blank display with the
overflow flag set. This is a defensive guard, not expected during
normal program operation.

### 3.2 `format_two_digit(n: u8) -> TwoDigit`

`n` is taken modulo 100 (three-digit codes are out of range for the
two-digit PROG/VERB/NOUN fields). Returns `{ tens: n / 10, units: n % 10 }`.

### 3.3 `decode_dsky(state: &DskyState) -> DskyFrame`

Straightforward field-by-field translation:

```rust
DskyFrame {
    prog: format_two_digit(state.prog),
    verb: format_two_digit(state.verb),
    noun: format_two_digit(state.noun),
    r1:   format_register(state.r[0]),
    r2:   format_register(state.r[1]),
    r3:   format_register(state.r[2]),
    lamps: Lamps {
        uplink_activity: state.uplink_activity,
        no_att:          state.no_att,
        stby:            state.stby,
        key_rel:         state.key_rel,
        opr_err:         state.opr_err,
        restart:         state.restart_flag,
        gimbal_lock:     state.gimbal_lock,
        temp:            state.temp,
        prog_alarm:      state.prog_alarm,
        comp_acty:       state.comp_acty,
    },
    lamp_test: state.lamp_test_active,
    flashing:  state.flashing,
}
```

---

## 4. Seven-segment encoding

```rust
/// 7-segment bit pattern for a single decimal digit.
///
/// Bit layout (a common-cathode convention):
///   bit 0 = top (a)
///   bit 1 = top-right (b)
///   bit 2 = bottom-right (c)
///   bit 3 = bottom (d)
///   bit 4 = bottom-left (e)
///   bit 5 = top-left (f)
///   bit 6 = middle (g)
pub fn digit_to_segments(digit: u8) -> u8;
```

Table:

| Digit | Bits         | Hex |
|-------|--------------|-----|
| 0     | 0011 1111    | 0x3F |
| 1     | 0000 0110    | 0x06 |
| 2     | 0101 1011    | 0x5B |
| 3     | 0100 1111    | 0x4F |
| 4     | 0110 0110    | 0x66 |
| 5     | 0110 1101    | 0x6D |
| 6     | 0111 1101    | 0x7D |
| 7     | 0000 0111    | 0x07 |
| 8     | 0111 1111    | 0x7F |
| 9     | 0110 1111    | 0x6F |
| blank | 0000 0000    | 0x00 |

`digit_to_segments(d)` for `d > 9` returns `0` (blank) rather than
panic — the upstream formatter guarantees valid inputs but the
boundary behaviour is defined for defensive programming.

---

## 5. Test cases

### TC-PB-1: format_register(0.0)
sign = 0, digits all zero, overflow = false.

### TC-PB-2: format_register(12345.0)
sign = +1, digits = [1,2,3,4,5], overflow = false.

### TC-PB-3: format_register(-7.0)
sign = -1, digits = [0,0,0,0,7].

### TC-PB-4: format_register(99999.0)
sign = +1, digits = [9,9,9,9,9], overflow = false.

### TC-PB-5: format_register(100000.0)
sign = +1, digits = [9,9,9,9,9], overflow = true.

### TC-PB-6: format_register(-100000.0)
sign = -1, digits = [9,9,9,9,9], overflow = true.

### TC-PB-7: format_register(3.7)
Rounds to 4: sign = +1, digits = [0,0,0,0,4].

### TC-PB-8: format_register(-2.5)
Banker's round would give −2; `libm::round` (half-away-from-zero)
gives −3. Spec follows libm: digits = [0,0,0,0,3], sign = -1.

### TC-PB-9: format_register(NaN)
sign = 0, digits = [0; 5], overflow = true.

### TC-PB-10: format_two_digit(37)
tens = 3, units = 7.

### TC-PB-11: format_two_digit(105)
105 mod 100 = 5 → tens = 0, units = 5.

### TC-PB-12: digit_to_segments sanity
Every digit 0..9 returns the table value; digit 10..255 returns 0.

### TC-PB-13: decode_dsky end-to-end
Construct a DskyState with prog=37, verb=6, noun=40, r=[100.0, -2.5, 0.0],
opr_err=true; call decode_dsky; verify prog/verb/noun split, register
values, and the opr_err lamp bit.
