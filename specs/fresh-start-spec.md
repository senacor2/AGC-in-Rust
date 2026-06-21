# Specification: `services/fresh_start` Module — FRESH START and RESTART Recovery

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/fresh_start.rs`
**Architecture reference**: `docs/architecture.md` §6 (Executive), §10 (Restart protection)
**Related specs**:
- `specs/executive-spec.md` §5 — restart protection, the phase-register conventions, and the comparison table.
- `specs/backup-spec.md` — `BackupState`, `snapshot_for_restart`, `restore_from_backup`, `invalidate`. The bare-metal boot path drives FRESH START into `invalidate` and RESTART into `restore_from_backup → restart`.
- `specs/alarm-spec.md` — RESTART increments `ercount`; FRESH START zeroes the whole `AlarmState`.
- `specs/p00-spec.md` — the program both flows hand control to.
**Glossary cross-reference**: `docs/glossary.md` — FRESH START, RESTART, GOJAM, phase register, restart group.
**AGC source files**:
- `Comanche055/FRESH_START_AND_RESTART.agc` — the AGC's FRESH START entry, RESTART entry, and verb dispatch table.
- `Comanche055/RESTARTS_ROUTINE.agc` — phase-register sweep and restart-group dispatch.

---

## 1. Purpose and Scope

`services::fresh_start` owns the two top-level recovery flows that
bring the AGC into a known state at boot time:

- **FRESH START** — full re-initialisation. Used on power-on,
  crew-initiated reset, or after any RESTART that the boot path
  refuses to honour (invalid magic, version mismatch, CRC fail). Zeroes
  *every* field of `AgcState` except a tiny documented survives list,
  then hands control to the Executive idle loop (P00).
- **RESTART** — fast recovery from a transient fault (watchdog
  timeout, parity error, software-initiated GOJAM, brief V_BAT-held
  power blip). Preserves navigation state, clears the scheduler,
  lights the RESTART lamp, bumps `ercount`, and re-dispatches active
  restart groups from their saved phase registers.

The two entry points are `fresh_start(state)` and
`restart(state)` (with `restart_with_table(state, table)` as the
test-friendly form). Both return `()`. There is no error case — these
are the recovery path, not a normal control flow.

### What this module provides

- `pub fn fresh_start(state: &mut AgcState)`.
- `pub fn restart(state: &mut AgcState)`.
- `pub fn restart_with_table(state: &mut AgcState, table: &[RestartGroupEntry; 6])`.
- `pub struct RestartGroupEntry` and its `EMPTY` const.
- `pub const RESTART_GROUP_TABLE: [RestartGroupEntry; 6]` — currently
  six `EMPTY` slots, populated as programs grow restart-aware code.

### What this module does NOT provide

- **The bare-metal boot path.** The board crate reads BKPSRAM, calls
  `services::backup::restore_from_backup`, and on success calls
  `restart(state)`; on failure it calls `services::backup::invalidate`
  and `fresh_start(state)`. That logic lives in the board crate.
- **The reset cause discrimination.** Whatever decides "this is a
  RESTART vs a FRESH START" is the boot loader's job, informed by the
  RCC reset flags on STM32.
- **The Executive loop.** After FRESH START / RESTART the caller is
  expected to invoke `Executive::run(state, hw)`.

---

## 2. AGC Background

### 2.1 Restart groups and phase registers

The AGC tracks long-running computations as one of six **restart
groups**, each with a phase register. The phase is:

| Phase | Meaning |
|---|---|
| 0 (idle) | Group not currently active. |
| Positive even | A non-restart-protected job is mid-flight; re-create as job. |
| Positive odd  | A waitlist task is pending; re-schedule as task. |
| Negative | Group is mid-computation but the safe restart point is "from the top"; re-enter via the group's job entry. |

The phase register is updated whenever the program crosses a
"safe to interrupt here" boundary. On RESTART the dispatch table
maps each non-idle group's phase back to a `create_job` /
`schedule_task` call.

### 2.2 What survives what

Two override sets are documented in `executive-spec.md`:

- **Override 1 (FRESH START)** — fields that survive even FRESH
  START because they are uplink-only values that ground would have to
  re-uplink. Currently two: `gha_epoch_rad` (GHABASE) and
  `liftoff_time` (V70 uplink).
- **Override 2 (RESTART)** — navigation state. Everything in
  `BackupState` (see `specs/backup-spec.md`) plus the in-RAM mirror of
  the `alarm` struct.

---

## 3. Rust API

### 3.1 `fresh_start`

```rust
pub fn fresh_start(state: &mut AgcState);
```

Implemented as **snapshot-replace-restore**:

```rust
let saved_gha_epoch_rad = state.gha_epoch_rad;
let saved_liftoff_time  = state.liftoff_time;
*state = AgcState::new();
state.gha_epoch_rad = saved_gha_epoch_rad;
state.liftoff_time  = saved_liftoff_time;
```

The whole-struct replace is deliberate. The previous field-by-field
implementation had a recurring failure mode where new `AgcState`
fields added in later patches silently leaked stale state across a
FRESH START until somebody noticed in test. The replace pattern
guarantees the survives list is fully expressed in the function
itself — adding a field that should survive requires adding it
*here*; everything else is wiped.

### 3.2 `RestartGroupEntry`

```rust
#[derive(Clone, Copy)]
pub struct RestartGroupEntry {
    pub job_entry:    Option<fn(&mut AgcState)>,
    pub job_priority: u8,
    pub task_entry:   Option<fn(&mut AgcState)>,
    pub task_delay:   u16,           // centiseconds
    pub major_mode:   u8,
}

impl RestartGroupEntry {
    pub const EMPTY: Self = Self {
        job_entry: None, job_priority: 0,
        task_entry: None, task_delay: 1,
        major_mode: 0,
    };
}
```

Each restart group declares whether it can come back as a job, as a
task, or both — depending on its phase semantics. Programs that need
restart protection register a `pub const RESTART_ENTRY` of this type
in their own module; the entry is then wired into the group table.

### 3.3 `RESTART_GROUP_TABLE`

```rust
pub const RESTART_GROUP_TABLE: [RestartGroupEntry; NUM_RESTART_GROUPS] = [
    RestartGroupEntry::EMPTY, …,
];
```

`const` (not `static mut`) per the project's no-globals rule. Tests
that need a populated table call `restart_with_table` directly with a
stack-local fixture (see TC-RS-4).

All six slots are currently `EMPTY` because no production program
has wired its restart-aware code yet. Programs gain a row in this
table as their restart-protection implementation lands.

### 3.4 `restart` / `restart_with_table`

```rust
pub fn restart(state: &mut AgcState);
pub fn restart_with_table(
    state: &mut AgcState,
    table: &[RestartGroupEntry; NUM_RESTART_GROUPS],
);
```

`restart(state)` delegates to `restart_with_table(state, &RESTART_GROUP_TABLE)`.

---

## 4. Functional Requirements

### 4.1 `fresh_start`

Postconditions (= `AgcState::new()` plus the two survivors):

- `csm_state`, `target_state`, `time`, `refsmmat`, `major_mode`, all
  scheduler queues, all DAP/TVC/burn/engine staging, all V/N input
  state, entry/IMU/PIPA staging, alarm FIFO — **zero**.
- `gha_epoch_rad` and `liftoff_time` — preserved.

If `AgcState` gains a new field that *also* must survive, the
implementation, this spec, and a new TC-FS-x test must all be
updated. That is the deliberate auditability property of the
"replace, then re-inject" implementation.

### 4.2 `restart`

In order:

1. **Preserve navigation state.** `csm_state`, `target_state`,
   `refsmmat`, `time`, `gha_epoch_rad`, `major_mode`,
   `liftoff_time` — untouched.
2. **Clear the scheduler.**
   `state.executive = Executive::new(); state.waitlist = Waitlist::new();`
3. **Reset guidance/control to safe defaults.**
   `dap_state = Default::default(); tvc_state = Default::default();`
4. **DSKY.** Light `dsky.restart_flag`; clear `dsky.flashing` and
   `dsky.opr_err`.
5. **Alarm bookkeeping.** `alarm.ercount =
   alarm.ercount.saturating_add(1)`. The FIFO, `code()`, and `adres`
   are preserved so the crew can still inspect V05N08 / V05N09.
6. **Re-dispatch active groups.** For each `(group, entry)` pair in
   `table`:
   - If `phase.is_idle()`: skip.
   - If `phase.is_job()` (positive even) and `entry.job_entry` is
     `Some(f)`: `executive.create_job(entry.job_priority, f, entry.major_mode)`.
   - If `phase.is_task()` (positive odd) and `entry.task_entry` is
     `Some(f)`: `waitlist.schedule(entry.task_delay, f)`.
   - Else (negative phase = from-top): prefer `job_entry` if present,
     else `task_entry`.

Note that step 5 increments `ercount` *before* the re-dispatch, so a
group whose re-entered job immediately raises another alarm will
have the correct restart-then-alarm ordering reflected in V05N08.

### 4.3 `restart` vs FRESH START — comparison

| Aspect | FRESH START | RESTART |
|---|---|---|
| `csm_state` / `target_state` | Zeroed | Preserved |
| `refsmmat` | Zeroed | Preserved |
| `time` | Zeroed | Preserved |
| `gha_epoch_rad` | **Preserved** | Preserved |
| `liftoff_time` | **Preserved** | Preserved |
| `major_mode` | Zeroed (→ P00) | Preserved |
| `executive` / `waitlist` | Cleared | Cleared |
| `dap_state` / `tvc_state` | Zeroed | Zeroed |
| `alarm` FIFO / code | Zeroed | Preserved |
| `alarm.ercount` | Zeroed | **Incremented** |
| `dsky.restart_flag` | false | **true** |
| Restart-group dispatch | n/a | Per phase |

---

## 5. Test Cases

| ID | Coverage |
|---|---|
| TC-FS-1 | `fresh_start` zeroes nav state (`csm_state.position`, `velocity`, `time`, `major_mode`). |
| TC-FS-2 | `fresh_start` clears all six restart phases to idle. |
| TC-FS-3 | `fresh_start` clears the alarm FIFO and lamp. |
| TC-FS-4 | `fresh_start` clears burn / engine / TVC staging — the bug class that motivated the "replace and re-inject" rewrite. |
| TC-FS-5 | `fresh_start` clears `vn.pending_v50` and entry-phase state. |
| TC-FS-6 | `fresh_start` clears IMU alignment + PIPA staging. |
| TC-FS-7 | `fresh_start` preserves `gha_epoch_rad`. |
| TC-FS-8 | `fresh_start` preserves `liftoff_time`. |
| TC-RS-1 | `restart` preserves nav state (`csm_state`, `time`, `major_mode`). |
| TC-RS-2 | `restart` lights `dsky.restart_flag`. |
| TC-RS-2b | `restart` increments `ercount` while preserving `code()` and `adres`. |
| TC-RS-3 | `restart` clears the scheduler (with all phases idle, no re-dispatch). |
| TC-RS-4 | `restart_with_table` re-dispatches GROUP_3 (positive-even phase → job) and GROUP_5 (positive-odd → task) using a stack-local fixture table. |

---

## 6. Module Layout

```
src/services/fresh_start.rs
├── pub fn fresh_start
├── pub struct RestartGroupEntry { … }    + impl { EMPTY }
├── pub const RESTART_GROUP_TABLE: [RestartGroupEntry; 6]
├── pub fn restart
├── pub fn restart_with_table
└── #[cfg(test)] mod tests                 (TC-FS-1..8, TC-RS-1..4, TC-RS-2b)
```

---

## 7. Spec Quality Checklist

- [x] FRESH START's "replace and re-inject" implementation rationale
      documented (§3.1, §4.1).
- [x] Two-element survives list (`gha_epoch_rad`, `liftoff_time`)
      explicit (§3.1, §4.1, §4.3 table).
- [x] Phase semantics (idle / job-even / task-odd / from-top-negative)
      tied to the dispatch logic (§2.1, §4.2).
- [x] `ercount` increment-on-restart rule, with FIFO preservation,
      documented (§4.2 step 5).
- [x] FRESH START vs RESTART comparison table (§4.3) matches
      `executive-spec.md` §5.4.
- [x] `RESTART_GROUP_TABLE` `const` choice (no `static mut`)
      documented (§3.3).
- [x] Out-of-scope items (bare-metal boot path, reset cause
      discrimination, Executive loop) listed (§1).
- [x] Test coverage spans both flows and both `ercount` paths (§5).
