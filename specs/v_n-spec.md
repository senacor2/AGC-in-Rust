# Specification: `services/v_n` — Verb/Noun Processor

**Status**: Approved for implementation (Milestone 6 Phase 1)
**Module path**: `agc-core/src/services/v_n.rs`
**Architecture reference**: `docs/architecture.md` §11 (DSKY and Crew Interface)
**HAL reference**: `specs/hal-spec.md` §6 (`Dsky` sub-trait)
**Programs reference**: `specs/p00-spec.md` (V37E00E destination)
**AGC source files**:
- `Comanche055/PINBALL_NOUN_TABLES.agc`
- `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc`
- `Comanche055/KEYRUPT,_UPRUPT.agc`

---

## 1. Purpose and Scope

The Verb/Noun (V/N) processor is the crew interface state machine. It
translates keystrokes arriving from the DSKY into Verb/Noun commands
and dispatches them to the appropriate handler (program select, display
request, data load, control verb).

**Milestone 6 Phase 1 scope**: the core state machine, key-code
definitions, and the most important dispatch paths — V37 (program
select), V06 (decimal display), V16 (monitor), V34 (terminate), V35
(lamp test). Data-entry verbs (V21/V22/V23/V25) and crew-acknowledgement
verbs (V33/V50) are later phases.

### What this phase provides

- `Key` enum: canonical AGC DSKY keys (0–9, VERB, NOUN, +, −, CLR, PRO,
  KEY REL, ENTR, RSET).
- `VnPhase` enum tracking the input state: `Idle`, `EnteringVerb`,
  `EnteringNoun`, `Ready` (both entered, awaiting ENTR), `OprErr`.
- `VnState` struct added to `AgcState`: current phase + two-digit verb
  buffer + two-digit noun buffer.
- `feed_key(state, key)` — the single entry point called by the KEYRUPT
  ISR shim. Drives the state machine and, on ENTR, invokes
  `dispatch_verb_noun`.
- `dispatch_verb_noun(state, verb, noun)` — routes to the implemented
  verb handlers.
- Support for **V37 E NN E** → `PROGRAM_TABLE[NN](state)`.
- Support for **V06 NN E** → display current noun value (no-op beyond
  setting `dsky.verb/noun`; the active program is expected to populate
  the `dsky.r` registers on its next update).
- Support for **V16 NN E** → same as V06 but flags the display as a
  continuous monitor (`dsky.verb = 16`).
- Support for **V34 E** → "terminate" — returns to P00.
- Support for **V35 E** → "lamp test" — toggles all lamps on for one
  cycle (stub: sets a lamp-test flag; real hardware sequencing is HAL).
- Unrecognised verbs / missing noun / out-of-range values raise the
  OPR ERR indicator and abort the current entry.

### What this phase does NOT provide

- Data-entry verbs V21/V22/V23/V25 (Phase 2).
- PINBALL display formatting (Phase 3).
- V33/V50 crew acknowledgement (Phase 4).
- Interactive P30/P40/P51/P52 wiring (Phase 4).
- Extended verbs V27 (self-test), V47 (restart), etc.

---

## 2. Key Codes

The AGC DSKY uses a 5-bit key matrix. Code values (Comanche055
PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, table KEYTEMP1):

| Key     | Code (decimal) |
|---------|----------------|
| `0`     | 16             |
| `1`     | 1              |
| `2`     | 2              |
| `3`     | 3              |
| `4`     | 4              |
| `5`     | 5              |
| `6`     | 6              |
| `7`     | 7              |
| `8`     | 8              |
| `9`     | 9              |
| `VERB`  | 17             |
| `NOUN`  | 31             |
| `+`     | 26             |
| `−`     | 27             |
| `CLR`   | 30             |
| `PRO`   | 25             |
| `KEY REL` | 25 (alias — real AGC used shared code) |
| `ENTR`  | 28             |
| `RSET`  | 18             |

(The PRO/KEY-REL collision in the historical hardware is resolved in
this port by treating them as the same logical key; the crew context
disambiguates.)

The `Key` enum provides a typed wrapper; `Key::from_code(u8)` maps a
HAL keypress to the enum or returns `None` for unknown codes.

---

## 3. State Machine

```rust
pub enum VnPhase {
    /// Nothing in progress — waiting for VERB or a control key.
    Idle,
    /// VERB pressed, accumulating up to two digits.
    EnteringVerb { digits: u8, buf: u8 },
    /// NOUN pressed after verb complete, accumulating up to two digits.
    EnteringNoun { verb: u8, digits: u8, buf: u8 },
    /// Both verb and noun received; awaiting ENTR.
    Ready { verb: u8, noun: u8 },
    /// OPR ERR — abandoning current entry.
    OprErr,
}

pub struct VnState {
    pub phase: VnPhase,  // Default: Idle
}
```

### Transitions

| Phase              | Key       | Next phase                                   |
|--------------------|-----------|----------------------------------------------|
| Idle               | VERB      | `EnteringVerb { digits: 0, buf: 0 }`         |
| Idle               | RSET      | Idle (clears OPR ERR lamp, alarm lit)        |
| Idle               | CLR       | Idle (no-op)                                 |
| Idle               | other     | Idle + `OprErr` lamp                         |
| EnteringVerb       | digit     | `EnteringVerb { digits+1, buf*10+d }`        |
| EnteringVerb (2 d) | NOUN      | `EnteringNoun { verb: buf, digits:0, buf:0 }`|
| EnteringVerb (<2)  | NOUN      | OprErr                                       |
| EnteringVerb       | ENTR      | (verbs taking no noun: V35) → dispatch       |
| EnteringVerb       | VERB      | restart verb entry                           |
| EnteringVerb       | CLR       | `Idle`                                       |
| EnteringNoun       | digit     | accumulate                                   |
| EnteringNoun (2 d) | ENTR      | dispatch                                     |
| EnteringNoun       | VERB      | restart the whole entry                      |
| EnteringNoun       | CLR       | `Idle`                                       |
| OprErr             | RSET      | clear lamp, `Idle`                           |
| OprErr             | any       | stay until RSET                              |

Digit accumulation is decimal; the buffer is clamped to 2 digits. A
verb or noun with `buf > 99` is an internal invariant violation and
panics in debug builds.

### Dispatch (verbs implemented in Phase 1)

```rust
fn dispatch_verb_noun(state: &mut AgcState, verb: u8, noun: u8) {
    match verb {
        6  => v06_display_decimal(state, noun),
        16 => v16_monitor(state, noun),
        34 => v34_terminate(state),
        35 => v35_lamp_test(state),
        37 => v37_program_select(state, noun),
        _  => raise_opr_err(state),
    }
}
```

### Verb handlers

- **V06 (Display Decimal)** — sets `dsky.verb = 6`, `dsky.noun = noun`,
  `dsky.flashing = false`. The active program will populate `dsky.r`
  on its next update cycle.

- **V16 (Monitor)** — same as V06 but with `dsky.verb = 16` so the
  active program knows to continuously refresh the display.

- **V34 (Terminate)** — calls `programs::p00::init(state)`. Clears
  `pending_maneuver`, stops any burn, DAP → AttitudeHold if running.

- **V35 (Lamp Test)** — sets all DSKY indicator lamps on for one cycle.
  In this Phase 1 implementation we set a `lamp_test_active` flag on
  `DskyState` that the T4RUPT display shim reads to drive the lamps on.

- **V37 (Change Program)** — looks up `PROGRAM_TABLE[noun]`:
  - if `Some(init_fn)`: calls `init_fn(state)` and passes the returned
    `JobPriority` to the Executive (Phase 1 uses the simplified direct
    call — Executive scheduling of the returned priority is the caller's
    responsibility, mirroring how tests already invoke init functions
    directly).
  - if `None`: raises OPR ERR (unknown or out-of-scope program).

### OPR ERR

`raise_opr_err(state)` sets `state.dsky.opr_err = true` and returns the
V/N state to `OprErr`. The crew clears it with the RSET key.

---

## 4. AgcState integration

Add `vn: VnState` to `AgcState`. Initialised in `AgcState::new` as
`VnState { phase: VnPhase::Idle }`.

The single public entry point `feed_key(state, key)` is called by the
KEYRUPT ISR shim (bare metal) or by the test harness.

---

## 5. Test Cases

### TC-VN-1: Key::from_code round trip.
All canonical codes map correctly; `Key::from_code(255)` returns `None`.

### TC-VN-2: V37E00E selects P00.
Fresh state; feed `V 3 7 N 0 0 E` sequence; assert
`state.major_mode == 0`, phase returns to `Idle`.

### TC-VN-3: V37E30E selects P30 and leaves `major_mode = 30`.
Same as TC-VN-2 but for program 30.

### TC-VN-4: V06N40E sets the display to V06/N40 without mutating burn state.
Seeds a noun, dispatches V06 N40, asserts `dsky.verb = 6`,
`dsky.noun = 40`, no other fields changed.

### TC-VN-5: V34E terminates to P00.
Sets major_mode = 40; feeds V 3 4 E; asserts major_mode back to 0.

### TC-VN-6: V35E sets lamp_test_active.
Feeds V 3 5 E; asserts `dsky.lamp_test_active == true`.

### TC-VN-7: Unknown verb raises OPR ERR.
Feeds V 9 9 N 0 0 E; asserts `dsky.opr_err == true` and phase = OprErr.

### TC-VN-8: RSET clears OPR ERR.
From OprErr phase, feed RSET; assert phase = Idle and opr_err lamp cleared.

### TC-VN-9: VERB during EnteringNoun restarts the entry.
Feeds V 3 7 N 0 V; asserts phase is back to EnteringVerb with empty buffer.

### TC-VN-10: CLR from EnteringVerb returns to Idle.
Feeds V 3 CLR; asserts phase = Idle.

### TC-VN-11: V37 with unknown program number raises OPR ERR.
Feeds V 3 7 N 9 9 E (slot 99 is None); asserts OPR ERR.

### TC-VN-12: Single-digit verb followed by NOUN raises OPR ERR.
Feeds V 3 N; asserts phase = OprErr.
