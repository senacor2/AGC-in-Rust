# Spec: RCS Jet Selection Logic (16-Jet CSM)

## AGC Source References

| File | Routine | Pages |
|---|---|---|
| `Comanche055/JET_SELECTION_LOGIC.agc` | `JETSLECT`, `PWORD`, `YWORD`, `RWORD`, `TABPCOM`, `TABYCOM`, `TABRCOM`, `TABRZCMD`, `PITCHTIM`, `YAWTIME`, `ROLLTIME`, `T6SETUP`, `T6START`, `REPLACER`, `REPLACEP`, `REPLACEY` | 1039–1062 |
| `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc` | `ZEROJET` (minimum impulse T6 setup) | 1015–1016 |

---

## Behavior Summary

### CSM 16-Jet Topology

The CSM has 16 RCS thrusters arranged in 4 quads (A, B, C, D), each providing
pitch, yaw, and roll torques. The four quads are paired as AC (pitch primary) and BD
(yaw/roll primary, interchangeable with AC for roll):

```
Quad A: jets A1-A4  (AC quad — pitch axis primary)
Quad B: jets B1-B4  (BD quad — yaw axis primary)
Quad C: jets C1-C4  (AC quad — pitch axis backup)
Quad D: jets D1-D4  (BD quad — yaw axis backup)
```

The selection logic selects 1- or 2-jet combinations per axis to satisfy the
rotation command (TAU) while satisfying translation commands simultaneously.

### Channel Bit Encoding

Two output channels carry jet commands (confirmed from `JET_SELECTION_LOGIC.agc`
lines 841–846, `T6START`):

**Channel 5 (PYJETS — pitch and yaw jets):**
- Bits 1–4 (`PJETS = 01417 octal`): pitch jet selection (AC quad jets)
- Bits 5–8 (`YJETS = 06360 octal`): yaw jet selection (BD quad jets)
- The combined PWORD + YWORD word is written to channel 5.

**Channel 6 (ROLLJETS — roll jets):**
- Bits 9–11 (`ACRJETS = 03760 octal`): AC quad roll jets
- Bits 12–14 (`BDRJETS = 34017 octal`): BD quad roll jets
- RWORD1 is written to channel 6.

### Jet Selection Table — PYTABLE

The pitch/yaw table is a 15-entry lookup indexed by rotation command (0/+/−)
and X-translation command (0/+/−), with additional entries for single-quad
failure fallback (`JET_SELECTION_LOGIC.agc` lines 172–186):

```
PYTABLE:
Index  Value(oct)  Meaning
  0    00000       No rotation, no translation
  1    05125       +Pitch, no translation (PWORD1 bits = 01417 masked)
  2    05252       −Pitch, no translation
  3    00231       No rotation, +X translation
  4    02421       +Pitch, +X translation
  5    02610       −Pitch, +X translation
  6    00146       No rotation, −X translation
  7    02504       +Pitch, −X translation
  8    02442       −Pitch, −X translation
  9    00000       No rotation, A(B) quad failed
 10    02421       +Pitch, A(B) quad failed
 11    02442       −Pitch, A(B) quad failed
 12    00000       No rotation, C(D) quad failed
 13    02504       +Pitch, C(D) quad failed
 14    02610       −Pitch, C(D) quad failed
```

The PJETS mask (01417 octal) extracts pitch bits; YJETS mask (06360 octal)
extracts yaw bits from the same table.

### Roll Table — RTABLE

15-entry table for roll (indexed by roll command × translation, with AC or BD
quad failure entries). RWORD bits are split by `ACRJETS = 03760 octal` (AC roll)
and `BDRJETS = 34017 octal` (BD roll) (`JET_SELECTION_LOGIC.agc` lines 388–410).

Key entries (octal):
```
RTABLE (AC roll half, masked with ACRJETS=03760):
  0    11000   No roll
  1    22125   +Roll
  2    00252   −Roll
  ...  (same pattern for translation combinations)

RTABLE (BD roll half, masked with BDRJETS=34017):
  same entries but in bits 12-14
```

### Minimum Impulse Time

The minimum jet firing time is **14 ms**, enforced by the T6 interrupt timer.

Confirmed from `JET_SELECTION_LOGIC.agc` line 568: `=14MS DEC 23` (23 TIME6 counts
at 0.625 ms/count = 14.375 ms ≈ 14 ms). Also confirmed by the comment at line 462:
"TO INSURE THAT JETS ARE NOT FIRED FOR LESS THAN A MINIMUM IMPULSE (14MS)."

T6 is initialized at `ZEROJET` (`RCS-CSM_DIGITAL_AUTOPILOT.agc` line 653):
`CAF =+14MS → TS TIME6`.

The Rust constant should be `14` centiseconds represented as `Met::from_centiseconds(1)` +
a note that it is actually 1.4 cs (14 ms); use the raw value `14_u32` in TIME6 units
or express as `Met(1)` (closest centisecond representation).

### Jet Command Assembly Process

```
Phase 3 (JETSLECT, every 100 ms):
  1. Check CHAN31 for manual translation commands → set XNDX1, XNDX2, YNDX, ZNDX.
  2. PWORD: CCS TAU1 → PINDEX (0=no, 1=+pitch, 2=−pitch).
     Check RACFAIL → select TABPCOM index or failure override.
     PYTABLE[XNDX1 + PINDEX] → PWORD1 (masked with PJETS).
  3. YWORD: CCS TAU2 → YINDEX; PYTABLE[XNDX2 + YINDEX] → YWORD1 (masked with YJETS).
  4. RWORD: CCS TAU → RINDEX; ACORBD flag selects AC or BD quad.
     RTABLE[YNDX + RINDEX] → RWORD1 (masked with ACRJETS or BDRJETS).
  5. On-time calculations (PITCHTIM/YAWTIME/ROLLTIME):
     TAU / NJET[NJETS] → BLAST (raw on-time); clamp to [14ms, 0.1s].
     TAU ← TAU − BLAST × NJETS (decrement remaining impulse).
  6. Assemble PWORD2, YWORD2, RWORD2 (post-rotation translation continuation).
  7. Sort BLAST/BLAST1/BLAST2 → T6SETUP (determines T6 interrupt sequence).
  8. T6START: write RWORD1 → CHAN6, (PWORD1|YWORD1) → CHAN5.
     After BLAST time: write RWORD2 → CHAN6, PWORD2 (with YJETS bits preserved) → CHAN5.
```

### Mapping `JetDecision` to Channel Bits

| Axis | Decision | PYTABLE index | Channel 5 bits (PJETS masked) |
|---|---|---|---|
| Pitch | `Positive` | 1 | `05125 & 01417 = 01025` (jets A3,A4,C1,C2) |
| Pitch | `Negative` | 2 | `05252 & 01417 = 01212` (jets A1,A2,C3,C4) |
| Yaw | `Positive` | 1 | `05125 & 06360 = 04100` (jets B1,B2,D3,D4) |
| Yaw | `Negative` | 2 | `05252 & 06360 = 04040` (jets B3,B4,D1,D2) |
| Roll | `Positive` | 1 | `22125 & 03760 = 02120` (AC roll jets) |
| Roll | `Negative` | 2 | `00252 & 03760 = 00240` (AC roll jets) |

Note: exact bit assignments depend on quad health. The table lookup handles quad failures.
For the simplified Rust `select_jets` (no-failure, no-translation case), use the
table index 1 (no translation, row 1 = + rotation) and 2 (row 2 = − rotation).

---

## Rust API

Module: `agc_core::control::rcs_logic`

Reuse `agc_core::hal::rcs::JetCommand` (already defined with `pitch_yaw: u16` and
`roll: u16` fields). Do NOT redefine it here.

```rust
use agc_core::hal::rcs::JetCommand;
use agc_core::control::attitude::JetDecision;
use agc_core::types::time::Met;
```

### Constants

```rust
/// Minimum RCS jet impulse duration: 14 ms.
///
/// All jet on-times are clamped to at least this value before being written
/// to the T6 timer.  Expressed as raw TIME6 counts (1 count ≈ 0.625 ms):
///   14 ms / 0.625 ms = 22.4 → rounded to 23 counts.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc `=14MS DEC 23` (line 568).
///             Comment at line 462: "MINIMUM IMPULSE (14MS)".
///             Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc ZEROJET `CAF =+14MS TS TIME6`.
pub const MIN_IMPULSE_TIME6_COUNTS: u32 = 23;

/// Minimum RCS jet impulse duration expressed as Met centiseconds (rounded up: 2 cs = 20 ms).
///
/// Use `Met::from_centiseconds(MIN_IMPULSE_CS)` when scheduling T6 via the Waitlist.
/// The true value is 14 ms; the 20 ms (2 cs) value is the nearest centisecond above.
pub const MIN_IMPULSE_CS: u32 = 2;
```

### Functions

```rust
/// Translate per-axis jet decisions into a channel word pair (JetCommand).
///
/// Implements the PYTABLE / RTABLE lookup for the no-failure, no-translation
/// case (the common flight case outside of manual translation commands or
/// quad failures).  Does not handle X/Y/Z translation overlaps or quad
/// failure overrides — those are handled by the full JETSLECT routine which
/// is only needed when `RACFAIL` or `RBDFAIL` flags are set, or when
/// manual translation is active.
///
/// Output:
///   `JetCommand::pitch_yaw` — written to channel 5 (PYJETS)
///   `JetCommand::roll`      — written to channel 6 (ROLLJETS)
///
/// All-None → JetCommand { pitch_yaw: 0, roll: 0 } = JetCommand::OFF.
///
/// Invariants:
///   - No heap, no blocking, deterministic table lookup.
///   - Pitch and yaw bits never overlap (PJETS and YJETS masks are disjoint).
///   - Roll bits are placed in channel 6 only.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc
///   PWORD (PYTABLE pitch lookup, p. 1039),
///   YWORD (PYTABLE yaw lookup, p. 1041),
///   RWORD (RTABLE roll lookup, p. 1043),
///   TABPCOM / TABYCOM / TABRCOM entry points,
///   T6START channel write sequence (p. 1061).
pub fn select_jets(
    pitch: JetDecision,
    yaw: JetDecision,
    roll: JetDecision,
) -> JetCommand;

/// Return the minimum jet impulse duration as a `Met` value.
///
/// Callers use this to set T6 timers and ensure no jet is fired for less than
/// 14 ms.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc `=14MS DEC 23` (line 568).
pub fn min_impulse_duration() -> Met {
    Met::from_centiseconds(MIN_IMPULSE_CS)
}
```

### Internal Lookup Tables (implementation detail, not public API)

The developer should implement these as `const` arrays mirroring the AGC tables:

```
// PYTABLE: 15 entries, each u16 (12-bit AGC word zero-extended)
const PYTABLE: [u16; 15] = [
    0o00000, 0o05125, 0o05252,  // no-fail, no-trans: none/+/-
    0o00231, 0o02421, 0o02610,  // no-fail, +X trans
    0o00146, 0o02504, 0o02442,  // no-fail, -X trans
    0o00000, 0o02421, 0o02442,  // A(B) quad failed
    0o00000, 0o02504, 0o02610,  // C(D) quad failed
];

// RTABLE: 15 entries (AC+BD roll combined)
const RTABLE: [u16; 15] = [
    0o11000, 0o22125, 0o00252,  // no-fail, no-trans
    0o11231, 0o15421, 0o04610,  // no-fail, +Y(Z) trans
    0o11146, 0o15504, 0o04442,  // no-fail, -Y(Z) trans
    0o11000, 0o15504, 0o04610,  // A(B) quad failed
    0o11000, 0o15421, 0o04442,  // C(D) quad failed
];

// Masks
const PJETS: u16  = 0o01417;
const YJETS: u16  = 0o06360;
const ACRJETS: u16 = 0o03760;  // matches agc_core::hal::rcs::ACRJETS_MASK
const BDRJETS: u16 = 0o34017;  // matches agc_core::hal::rcs::BDRJETS_MASK
```

The developer MUST verify that `ACRJETS` and `BDRJETS` match the existing constants
in `agc_core::hal::rcs` (`ACRJETS_MASK = 0o03760`, `BDRJETS_MASK = 0o34017`).

---

## Scale Factors

| Quantity | AGC format | Rust type |
|---|---|---|
| Channel 5 word | 12-bit AGC ones-complement | `u16` (zero-extended, bit 16 unused) |
| Channel 6 word | 12-bit AGC ones-complement | `u16` (zero-extended) |
| Jet on-time TAU | Scaled 0.1 s = 160 units | `f64` seconds (computed upstream) |
| Min impulse TIME6 | 23 counts × 0.625 ms | `u32` counts or `Met` (2 cs rounded up) |

---

## Invariants

- **No heap**: all lookups are `const` array indexing. No `Vec` or `HashMap`.
- **Finite, deterministic**: given the same three `JetDecision` values, `select_jets`
  always returns the same `JetCommand`. No floating-point arithmetic; pure integer
  table lookup.
- **Disjoint channels**: pitch and yaw bits never collide in channel 5 (`PJETS ∩ YJETS = 0`).
  Roll is entirely in channel 6. This is a hard invariant: `assert_eq!(PJETS_MASK & YJETS_MASK, 0)`.
- **All None → OFF**: `select_jets(None, None, None)` must return `JetCommand::OFF`.
- **No translation/failure handling in this function**: `select_jets` covers the
  pure rotation case. Translation overlay and quad-failure fallback are left to a future
  `select_jets_full` extension that also takes translation indices and failure flags.

---

## Test Cases

1. **All None → zero command**: Call `select_jets(None, None, None)`. Assert
   `result == JetCommand::OFF` (both fields 0).

2. **Pitch Positive → expected PYJETS bits**: Call `select_jets(Positive, None, None)`.
   Assert `result.pitch_yaw & PJETS_MASK == 0o01025` and `result.roll == 0`.
   (PYTABLE[1] = 05125; 05125 & 01417 = 01025.)

3. **Roll Negative → expected ROLLJETS bits**: Call `select_jets(None, None, Negative)`.
   Assert `result.roll & ACRJETS_MASK == 0o00240` and `result.pitch_yaw == 0`.
   (RTABLE[2] = 00252; 00252 & 03760 = 00240 for AC roll negative.)

4. **All three simultaneous → combined bits**: Call `select_jets(Positive, Positive, Positive)`.
   Assert `result.pitch_yaw` has both PJETS and YJETS bits set (non-zero in both ranges)
   and `result.roll & ACRJETS_MASK != 0`. Specifically:
   - `pitch_yaw & PJETS_MASK` = positive pitch bits
   - `pitch_yaw & YJETS_MASK` = positive yaw bits
   - `roll & ACRJETS_MASK` = positive AC-roll bits

---

## agc-sim Impact

- `SimHardware.rcs`: `fire_jets()` is already implemented via `RcsImpl`.
- `MissionState` panel: add `rcs_active: bool` flag (true when any jet bit is non-zero).
- `SimLog`: emit `.debug("RCS jets fired: ch5={:04o} ch6={:04o}")` on each non-zero command.
- No new DSKY bindings needed.
