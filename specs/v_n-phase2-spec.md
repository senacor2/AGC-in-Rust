# Specification: Verb/Noun Data Entry Verbs (Phase 2)

**Status**: Approved for implementation (Milestone 6 Phase 2)
**Module path**: `agc-core/src/services/v_n.rs` (extended)
**Depends on**: Phase 1 V/N core state machine
**Target demo**: end-to-end V25 N33 → V25 N81 → P30 targeting flow

---

## 1. Scope

Extends the Phase 1 V/N processor with the four data-entry verbs that
load values from the crew into the AGC:

| Verb | Function                              | Registers loaded |
|------|---------------------------------------|------------------|
| V21  | Load component 1 of current noun      | R1                |
| V22  | Load component 2 of current noun      | R2                |
| V23  | Load component 3 of current noun      | R3                |
| V25  | Load all three components             | R1, R2, R3        |

Each component is a signed 5-digit decimal integer. The initial sign
defaults to `+`; the crew may press `+` or `-` at the start of a
component to override. After pressing ENTR, the accumulator commits to
the target register and the next register is requested (V25) or the
load completes (V21/V22/V23).

### What this phase adds

- `EnteringData` phase in `VnPhase`.
- Numeric accumulator: signed `i32`, five-digit maximum (`|buf| ≤ 99_999`).
- Plus/minus sign handling (only valid as the first character after
  VERB…NOUN…ENTR or after component commit).
- Per-noun scale table mapping raw integers to the target unit.
- Noun commit handlers: Phase 2 implements N33 (TIG, centiseconds) and
  N81 (LVLH ΔV, m/s integer) to unlock P30.
- `VnState.pending_tig: Option<Met>` — staging field for P30 data load.
- End-to-end test: V25 N33 E <digits> E → V25 N81 E <dx> E <dy> E <dz> E
  results in `state.pending_maneuver.is_some()`.

### What this phase does NOT add

- Octal display verbs V01/V02/V03.
- Fixed-point fractional entry (e.g. "1234" meaning 12.34).
- Monitor versions V11/V12/V13 (Phase 4).
- V15 (test crewed noun) and other less common verbs.

---

## 2. Extended state machine

```rust
pub enum VnPhase {
    // ... Phase 1 variants ...
    /// Data entry in progress for a V21/V22/V23/V25 load.
    EnteringData {
        /// The verb that initiated the load (21, 22, 23, or 25).
        verb: u8,
        /// The noun being loaded.
        noun: u8,
        /// Which register (0, 1, or 2) we are currently loading.
        reg_index: u8,
        /// How many registers total this verb loads (1 for V21-23, 3 for V25).
        total_regs: u8,
        /// Accumulator sign (+1 or -1). Default +1.
        sign: i8,
        /// Number of digits accumulated in the current component (0..5).
        digits: u8,
        /// Absolute value of the accumulator (0..99_999).
        buf: u32,
        /// Register values committed so far (scaled by noun_scale).
        committed: [f64; 3],
    },
}
```

### Transitions for EnteringData

| Key        | Action                                                             |
|------------|--------------------------------------------------------------------|
| digit (0 digits, V25 first) | first digit — replace sign=+, accumulate |
| digit      | accumulate: `buf = buf*10 + d`, clamp at 5 digits; `>5` → OPR ERR  |
| `+`        | only valid at `digits == 0` → sign = +1; else OPR ERR              |
| `-`        | only valid at `digits == 0` → sign = -1; else OPR ERR              |
| ENTR       | commit current register: `committed[reg_index] = sign*buf*scale`. If `reg_index+1 < total_regs`: advance to next register, reset sign/digits/buf. Otherwise finish: call `noun_commit(verb, noun, committed)` and phase → Idle. |
| VERB       | restart: phase → EnteringVerb                                      |
| CLR        | abort the load: phase → Idle                                       |
| RSET       | clear OPR ERR lamp (same as anywhere)                              |
| any other  | OPR ERR                                                            |

### Dispatch entry

In `dispatch_verb_noun`, verbs 21/22/23/25 initiate a load:

```rust
21 | 22 | 23 => start_load(state, verb, noun, 1, reg_index = verb - 21),
25            => start_load(state, verb, noun, 3, reg_index = 0),
```

where `start_load` transitions the phase to `EnteringData` with
`committed = [0; 3]`, `sign = +1`, `digits = 0`, `buf = 0`.

---

## 3. Noun scale table

The scale converts the accumulated signed integer into the target unit
before handing off to the noun commit function.

```rust
fn noun_scale(noun: u8) -> f64 {
    match noun {
        33 => 1.0,    // N33: TIG — centiseconds, integer
        34 => 1.0,    // N34: TFI — centiseconds, integer (Phase 2 placeholder)
        81 => 1.0,    // N81: ΔV LVLH — m/s, integer
        _  => 1.0,    // default: pass-through
    }
}
```

Real AGC nouns use much finer scales (e.g. N81 is B-7 m/s scaling
≈ 0.00784 m/s per bit). Phase 2 uses unit scales for test clarity;
a refined table is a later refinement.

---

## 4. Noun commit handlers

```rust
fn noun_commit(state: &mut AgcState, verb: u8, noun: u8, values: [f64; 3]) {
    match noun {
        33 => noun_33_commit_tig(state, values[0]),
        81 => noun_81_commit_dv_lvlh(state, values),
        _  => { /* no-op for Phase 2 */ }
    }
}
```

### `noun_33_commit_tig`

Stashes the loaded TIG (centiseconds as `u32`) into
`state.vn.pending_tig = Some(Met(values[0] as u32))`. Does NOT call
any program — waits for the delta-V to also be loaded.

### `noun_81_commit_dv_lvlh`

- If `state.vn.pending_tig.is_none()`: raise program alarm 240
  ("ΔV load without TIG") and return.
- Otherwise: take the stashed TIG, pass TIG + loaded [dv_x, dv_y, dv_z]
  to `programs::p30::p30_load_dv_lvlh(state, tig, dv)`.
- This fires whether or not the major mode is P30 — the crew may have
  selected V25 N81 outside P30 by mistake; P30's own validation
  (alarm 210 for TIG in the past) covers the rest.

### New field on `VnState`

```rust
pub pending_tig: Option<Met>,
```

Initialised to `None`. Set by `noun_33_commit_tig`. Consumed
(`.take()`) by `noun_81_commit_dv_lvlh`.

### New alarm code

```rust
const ALARM_DV_LOAD_WITHOUT_TIG: u16 = 240;
```

---

## 5. Test cases

### TC-VND-1: V21 N81 E +100 E
Loads 100.0 into R1 only.
Assert `VnState.phase == Idle`, the load reached `noun_commit` with
`values[0] == 100.0` — easy to verify by setting a known noun that
stashes into an observable field (reuse N33 to check TIG stash).

### TC-VND-2: V25 N33 E +50000 E
Pressing ENTR after 5 digits commits `pending_tig = Some(Met(50_000))`.

### TC-VND-3: V25 N81 E +100 E +0 E +0 E with prior pending_tig
Happy path: `state.pending_maneuver` becomes `Some(_)` after all three
registers are loaded, and `state.vn.pending_tig` is consumed back to
`None`.

### TC-VND-4: V25 N81 without prior V25 N33 raises alarm 240
Load N81 directly without TIG; assert alarm 240 and no pending_maneuver.

### TC-VND-5: Minus sign before first digit
`V25 N81 E - 1 0 0 E + 0 E + 0 E` with pending TIG; assert the first
component is −100.

### TC-VND-6: Sign after digit raises OPR ERR
`V25 N81 E 1 + ...` — plus/minus after a digit is an error.

### TC-VND-7: Six-digit overflow raises OPR ERR
`V25 N81 E 1 2 3 4 5 6 ...` — the sixth digit is rejected.

### TC-VND-8: CLR during data entry aborts the load
`V25 N81 E 1 2 3 CLR` — phase returns to Idle, no commit.

### TC-VND-9: V21 loads R1 only and commits immediately
V21 N33 E 12345 E → `pending_tig == Some(Met(12_345))` (noun 33
commits from the first value).

### TC-VND-10: End-to-end V25 N33 → V25 N81 → P30
1. `init_p30` (set major_mode=30, etc.)
2. Feed V25 N33 E 500000 E (TIG = 5000 s from epoch)
3. Feed V25 N81 E 100 E 0 E 0 E (100 m/s prograde)
4. Assert `state.pending_maneuver.is_some()` with
   `target_dv` magnitude ≈ 100 m/s.
