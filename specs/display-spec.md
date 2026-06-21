# Specification: `services/display` Module — DSKY Display State

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/display.rs`
**Architecture reference**: `docs/architecture.md` §7 (DSKY services)
**Related specs**:
- `specs/pinball-spec.md` — owns the `Lamps` struct and the `decode_dsky` function that flattens `DskyState` into a frame for the bridge.
- `specs/v_n-spec.md` — owns the V/N processor that writes `DskyState`.
- `specs/t4rupt-spec.md` — runs the periodic shim that reads `DskyState`, builds the bridge frame, and writes changed rows to hardware via the `Dsky` HAL trait.
- `specs/hal-spec.md` — `Dsky` sub-trait.
**Glossary cross-reference**: `docs/glossary.md` — DSKY, PROG / VERB / NOUN, R1 / R2 / R3, lamps.

---

## 1. Purpose and Scope

`services::display` owns the in-memory **shadow copy of the DSKY
display**. It is a one-struct module: `DskyState` aggregates every
visible field on the panel — major mode (`PROG`), `VERB`, `NOUN`, three
data registers (`R1`, `R2`, `R3`), the verb/noun flash, and one bit per
indicator lamp.

Updates flow in one direction:

```
programs / services         T4RUPT shim
  ─writes→   DskyState   ─reads→   pinball::decode_dsky → Dsky HAL
```

Programs (P00–P67) and service routines (`v_n`, `pinball`, etc.) write
to `state.dsky`; the T4RUPT shim reads it, asks `pinball::decode_dsky`
to produce a wire-format `DskyFrame`, then pushes the changed rows to
the physical DSKY through the `Dsky` HAL sub-trait.

### What this module provides

- `DskyState` — a single `#[derive(Clone, Copy, Debug, Default)]`
  struct holding the entire current DSKY display.

### What this module does NOT provide

- **Field decoding / formatting**. R1, R2, R3 are stored as `f32` and
  the unit / scale interpretation is the per-noun job of
  `services::pinball::decode_dsky` and the noun tables in
  `services::v_n`.
- **A read path**. Programs do not consult `DskyState` to decide
  behaviour; it is exclusively the visible-state shadow.
- **Lamp test sequencing**. The `lamp_test_active` flag is a one-shot
  request that the T4RUPT display shim consumes and clears; the actual
  V35 verb that sets the flag lives in `services::v_n`.
- **Any HAL call**. The HAL interaction happens in the T4RUPT shim,
  not here.

---

## 2. Rust API

### 2.1 `DskyState`

```rust
#[derive(Clone, Copy, Debug, Default)]
pub struct DskyState {
    pub prog: u8,
    pub verb: u8,
    pub noun: u8,
    pub r: [f32; 3],
    pub flashing: bool,
    pub uplink_activity: bool,
    pub no_att: bool,
    pub stby: bool,
    pub key_rel: bool,
    pub opr_err: bool,
    pub restart_flag: bool,
    pub gimbal_lock: bool,
    pub temp: bool,
    pub prog_alarm: bool,
    pub comp_acty: bool,
    pub tracker: bool,
    pub lamp_test_active: bool,
}
```

### 2.2 Field semantics

| Field | Meaning | Writer | Reader |
|---|---|---|---|
| `prog` | Currently displayed PROG (major-mode) number. | Programs at entry; `services::fresh_start` to 0. | `pinball::decode_dsky` → `DskyFrame.prog`. |
| `verb` | Active VERB code. | V/N processor; programs that prompt for input. | `decode_dsky` → `DskyFrame.verb`. |
| `noun` | Active NOUN code. | V/N processor; programs that prompt for input. | `decode_dsky` → `DskyFrame.noun`. |
| `r[0..3]` | Data registers R1 / R2 / R3 (5-digit signed decimal, semantics noun-specific). | Programs and SERVICER-exit hooks; per-noun update functions in `services/v_n.rs`. | `decode_dsky` formats per noun. |
| `flashing` | Verb/Noun flash — crew input request. | Set by programs entering an input phase; cleared at commit or `gotopooh`. | T4RUPT display shim toggles the visible blink. |
| `uplink_activity` | UPLINK ACTY lamp. | `uplink::poll_uplink` sets when a non-zero word is drained, clears on a quiet poll. | T4 display shim. |
| `no_att` | NO ATT lamp — IMU caged or REFSMMAT invalid. | IMU control / P51-52. | T4. |
| `stby` | STBY lamp. | `programs::p06::init`. | T4. |
| `key_rel` | KEY REL lamp. | V/N processor when display is seized. | T4. |
| `opr_err` | OPR ERR lamp. | V/N processor on illegal input. | T4. |
| `restart_flag` | RESTART lamp. | `services::fresh_start::restart`. | T4. |
| `gimbal_lock` | GIMBAL LOCK lamp. | IMU control / DAP. | T4. |
| `temp` | TEMP lamp (IMU heater fault). | Health-monitor (future). | T4. |
| `prog_alarm` | PROG alarm lamp companion. | `services::alarm` lights via `alarm.lit` flag — `prog_alarm` is a separate panel bit that may be set independently for milestone-specific tests. | T4. |
| `comp_acty` | COMP ACTY lamp — Executive is dispatching a job. | `executive::scheduler` for each dispatch. | T4. |
| `tracker` | TRACKER lamp — IMU optical alignment activity (P51, P52, P22 marks). | Optics-mark handlers. | T4. |
| `lamp_test_active` | One-shot V35 lamp-test request. | `services::v_n` on V35 ENTR. | T4 display shim drives every lamp on for one cycle, then clears the flag. |

---

## 3. Functional Requirements

There are **no functions** in this module. The struct's behaviour is
its `Copy + Default` derivation: every field defaults to its
type-default (`0` for the numeric fields, `false` for the bools,
`0.0` for the `f32` registers).

Use patterns:

- **Program entry**: a program writes `state.dsky.prog = NN`,
  `state.dsky.verb = VV`, `state.dsky.noun = NN`,
  `state.dsky.flashing = false_or_true`. Other fields untouched.
- **Lamp control**: any subsystem flips its named bit; the T4RUPT shim
  later converts the boolean to a row write on the `Dsky` HAL.
- **Reset**: `services::fresh_start::fresh_start` replaces the whole
  `DskyState` (via `*state = AgcState::new()`); `services::alarm::_return_to_p00`
  clears `prog`, `verb`, `noun`, and `flashing` and `opr_err` but
  preserves the rest (notably `restart_flag`).

---

## 4. Module Layout

```
src/services/display.rs
└── pub struct DskyState { … 16 fields … }
```

No constants, no functions, no tests in the file. The struct is
exercised through every program and service test.

---

## 5. Spec Quality Checklist

- [x] Module purpose explicitly limited to the shadow display state
      (§1).
- [x] Each field's writer and reader documented (§2.2).
- [x] No functional behaviour to specify — explicitly noted (§3).
- [x] Cross-references to `pinball-spec`, `v_n-spec`, and `t4rupt-spec`
      that own the producers and consumers.
- [x] No alarm-code coupling, no HAL calls — explicitly recorded as
      out of scope (§1).
