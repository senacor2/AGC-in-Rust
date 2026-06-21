# Specification: `services/alarm` Module — Alarm State and Soft-Abort Recovery

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/alarm.rs`
**Architecture reference**: `docs/architecture.md` §7 (Services), §10 (Error handling)
**Related specs**:
- `specs/v_n-spec.md` — diagnostic display nouns V05N08 / V05N09 read the FIFO and call-site.
- `specs/executive-spec.md` — Executive raises `EXEC_OVERFLOW` / `NO_VAC` through `AlarmState::raise`.
- `specs/p00-spec.md` — the program both POODOO and GOTOPOOH return to.
- `specs/fresh-start-spec.md` — RESTART increments `ercount` and preserves the FIFO; FRESH START zeroes the whole struct.
- `specs/pinball-spec.md` — `decode_dsky` mirrors `alarm.lit` into the PROG alarm lamp.
**Glossary cross-reference**: `docs/glossary.md` — POODOO, GOTOPOOH, FAILREG, ERCOUNT, V05N08, V05N09.
**AGC source files**:
- `Comanche055/ALARM_AND_ABORT.agc` — `POODOO`, `GOTOPOOH`, `2BADAD` and the underlying alarm-set path.
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `FAILREG`, `FAILREG+1`, `FAILREG+2`, `ERCOUNT`.
- `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc` §8 — the alarm-code table.

---

## 1. Purpose and Scope

`services::alarm` owns the **runtime alarm state** and the two
soft-abort recovery paths that the AGC uses to bail out of a program
that has hit a non-fatal but non-continuable condition.

Three pieces:

- `AlarmState` — the 3-deep FIFO of recent alarm codes plus the
  call-site tag, the alarm/restart counter, and the PROG-alarm lamp
  bit. Mirror of erasables `FAILREG`, `FAILREG+1`, `FAILREG+2`, and
  `ERCOUNT`.
- `poodoo(state, code)` — hard soft-abort: raise the alarm, light the
  lamp, and return to P00.
- `gotopooh(state)` — soft soft-abort: return to P00 *without* raising
  an alarm.

The actual alarm-code values live in `tables::alarm_codes`. This
module does not hold any inline `const ALARM_*` — see the memory
note `feedback_alarm_codes.md`.

### What this module provides

- `pub struct AlarmState { fifo: [u16; 3], adres: u16, ercount: u16, lit: bool }`
- `impl AlarmState` — `code()`, `raise(code, adres)`, `reset()`.
- `pub fn poodoo(state, alarm_code)` — abort with alarm.
- `pub fn gotopooh(state)` — abort without alarm.
- A private `_return_to_p00(state)` helper shared by both abort paths.

### What this module does NOT provide

- **Alarm code values.** Codes live in `tables::alarm_codes`. Adding
  a new alarm class means adding a constant to that table, not to
  this module.
- **Hard restart.** A watchdog timeout / GOJAM goes through
  `services::fresh_start::restart`, not POODOO.
- **Program registration.** Programs raise their own alarms via
  `state.alarm.raise(code, SITE_X)` directly; no callback table.

---

## 2. AGC Background

In Comanche055 the alarm system is rooted in `ALARM_AND_ABORT.agc`.
The relevant erasable cells are:

| AGC erasable | This module | Display |
|---|---|---|
| `FAILREG` | `fifo[0]` | V05N09 R1 (oldest) |
| `FAILREG+1` | `fifo[1]` | V05N09 R2 (middle) |
| `FAILREG+2` | `fifo[2]` | V05N09 R3 (newest) |
| `ALMCADR` / `ADRES` | `adres` | V05N08 R1 (call site) |
| `ERCOUNT` | `ercount` | V05N08 R3 (counter) |
| PROG alarm lamp | `lit` | DSKY panel bit |

`POODOO` is the hard path — used when a computation cannot continue
(imaginary roots in Lambert, IMU not aligned for a star sighting,
state vector out of range). It raises the supplied alarm code, lights
the PROG lamp, and returns to P00 while preserving navigation state.

`GOTOPOOH` is the soft path — used when the crew bails out of an
input dialog (V34 PROCEED out of context, RSET mid-mark) or a
non-fatal anomaly is detected. No alarm; the display just falls back
to P00 idle.

Both paths clear the scheduler, the DAP, and the burn / engine
staging fields so the vehicle ends up in a quiescent state. Both
preserve `csm_state`, `target_state`, `refsmmat`, `time`, `csm_nav`,
`rendezvous_nav`, `gha_epoch_rad`, and `liftoff_time`.

---

## 3. Rust API

### 3.1 `AlarmState`

```rust
#[derive(Clone, Copy, Debug, Default)]
pub struct AlarmState {
    pub fifo: [u16; 3],   // [oldest, middle, newest]
    pub adres: u16,       // SITE_* tag captured at raise time
    pub ercount: u16,     // alarm/restart counter (saturating)
    pub lit: bool,        // PROG alarm lamp
}

impl AlarmState {
    pub fn code(&self) -> u16;                  // == self.fifo[2]
    pub fn raise(&mut self, code: u16, adres: u16);
    pub fn reset(&mut self);                    // clears `lit` only
}
```

### 3.2 Free functions

```rust
pub fn poodoo(state: &mut AgcState, alarm_code: u16);
pub fn gotopooh(state: &mut AgcState);
```

Both take `&mut AgcState`. Both are `no_std`-safe and have no other
dependencies on the HAL.

---

## 4. Functional Requirements

### 4.1 `AlarmState::raise`

1. Shift the FIFO: `fifo[0] ← fifo[1]; fifo[1] ← fifo[2]; fifo[2] ← code`.
2. Set `adres ← code-site tag` (one of `tables::alarm_codes::SITE_*`).
3. `ercount = ercount.saturating_add(1)`.
4. `lit = true`.

The FIFO is always shifted, even when re-raising the same code, so a
storm of identical alarms still records the chronology in `ercount`.

### 4.2 `AlarmState::reset`

Sets `lit = false`. **Does not** clear the FIFO, `adres`, or
`ercount` — the crew may still want to inspect V05N08 / V05N09 after
acknowledging the lamp.

### 4.3 `poodoo(state, alarm_code)`

1. `state.alarm.raise(alarm_code, SITE_POODOO)`.
2. Call `_return_to_p00(state)`.

Postconditions:
- `alarm.code() == alarm_code`, `alarm.lit == true`.
- `major_mode == 0`, `dsky.prog == 0`.
- Scheduler (Executive + Waitlist) is empty.
- Guidance staging (`burn`, `pending_maneuver`, `servicer_exit`,
  `engine_thrusting`, `drogue_deploy_pending`, `csm_separation_pending`)
  cleared.
- DAP / TVC commands (`dap_state`, `rcs_commanded_jets`,
  `rcs_commanded_pulse_cs`, `sps_gimbal_cmd`) cleared.
- DSKY: `prog/verb/noun = 0`, `flashing = false`, `opr_err = false`.
- Navigation state (`csm_state`, `target_state`, `refsmmat`, `time`,
  `csm_nav`, `rendezvous_nav`, `gha_epoch_rad`, `liftoff_time`)
  unchanged.

### 4.4 `gotopooh(state)`

Same as POODOO except no alarm is raised — `state.alarm` is
untouched.

### 4.5 `_return_to_p00` (internal)

The common tail of both abort paths. Documented as one ordered
block so future fields added to `AgcState` have a clear convention:
add to `_return_to_p00` iff a partially-built value would corrupt
the next program; otherwise leave it to `fresh_start`.

---

## 5. Test Cases

| ID | Coverage |
|---|---|
| TC-ALARM-1 | `raise` stacks codes correctly (single raise sets `fifo[2]`, second raise shifts), `adres`/`ercount` updated, `lit` set. |
| TC-ALARM-1b | 3-deep FIFO — three raises fill `fifo[0..3]`; a fourth drops the oldest. |
| TC-ALARM-2 | `reset` clears `lit` only; FIFO, `adres`, `ercount` preserved. |
| TC-ALARM-3 | `poodoo` from P23 returns to P00, clears scheduler, preserves nav state. |
| TC-ALARM-4 | `gotopooh` returns to P00 without touching `alarm.code()` or `lit`. |
| TC-ALARM-5 | `poodoo` mid-burn (P40, `engine_thrusting == true`) stops the engine and clears `dsky.flashing`. |

All located in the in-file `#[cfg(test)] mod tests`.

---

## 6. Restart and Backup Behaviour

- `AlarmState` is **not** part of `BackupState` (see
  `specs/backup-spec.md`). On RESTART, `services::fresh_start::restart`
  preserves the in-RAM `alarm` struct *and* increments `ercount`, so
  V05N08 R3 shows the new restart event. On FRESH START the struct is
  zeroed.

---

## 7. Module Layout

```
src/services/alarm.rs
├── pub struct AlarmState { fifo, adres, ercount, lit }
├── impl AlarmState { code, raise, reset }
├── pub fn poodoo(state, alarm_code)
├── pub fn gotopooh(state)
├── fn _return_to_p00(state)         (private)
└── #[cfg(test)] mod tests           (TC-ALARM-1..5, 1b)
```

---

## 8. Spec Quality Checklist

- [x] FIFO order, lamp semantics, and call-site tag rules documented
      (§4.1, §4.2).
- [x] POODOO vs GOTOPOOH difference (with alarm / without) called out
      (§4.3 vs §4.4).
- [x] Postconditions of `_return_to_p00` listed field-by-field (§4.3).
- [x] Alarm-code values explicitly delegated to
      `tables::alarm_codes` per project memory rule (§1).
- [x] Cross-references to `executive-spec`, `v_n-spec`,
      `fresh-start-spec`, and `backup-spec` for the producers and
      consumers of `AlarmState`.
- [x] Restart / backup interaction recorded (§6).
