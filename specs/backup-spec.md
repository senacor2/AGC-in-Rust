# Specification: `services/backup` Module — RESTART-Survivable State

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/backup.rs`
**Architecture reference**: `docs/architecture.md` §7 (Services), §10 (Restart protection)
**Related specs**:
- `specs/fresh-start-spec.md` — owns the RESTART entry point that calls `restore_from_backup` at boot and `invalidate` on FRESH START.
- `specs/executive-spec.md` §5 — restart protection and phase registers.
- `specs/state-vector-spec.md` — `StateVector` field layout (part of the backup).
- `specs/imu-control-spec.md` — `GyroCompensation`, `ImuAlignmentState` (part of the backup).
- `specs/average-g-spec.md` — `PipaCalibration` (part of the backup).
**Glossary cross-reference**: `docs/glossary.md` — BKPSRAM, FRESH START, RESTART, GOJAM, GHA, REFSMMAT.
**AGC source files**:
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — the survives-RESTART erasables (state vectors, REFSMMAT, GHABASE, FAILREG family, phase registers).
- `Comanche055/RESTARTS_ROUTINE.agc` — the AGC's RESTART path that re-reads those erasables.

---

## 1. Purpose and Scope

`services::backup` defines the **persistent state subset** that must
survive a RESTART (watchdog timeout, parity error, GOJAM, brief power
blip) and provides three functions to take it in and out of
persistent storage.

On the bare-metal STM32F767ZI target the persistent storage is the
4 KB BKPSRAM region at `0x4002_4000`, held alive by a CR2032 on
V_BAT. The bare-metal boot path treats the BKPSRAM as a
`*mut BackupState` and calls `restore_from_backup` on every cold
start; on FRESH START it calls `invalidate` to scrub the region.

On the host (agc-sim / tests) `BackupState` is just a normal struct —
tests drive snapshot/restore round-trips and the `RestoreError`
variants exercise the boot-path fallback logic without any actual
non-volatile memory.

### What this module provides

- `BackupHeader` (16 bytes, `#[repr(C)]`) — magic / version / CRC.
- `BackupState` (`#[repr(C)]`) — header + the survives-RESTART payload.
- `RestoreError` — `MagicMismatch` / `VersionMismatch` / `ChecksumMismatch`.
- `MAGIC: [u8; 4]` and `VERSION: u32` constants.
- `snapshot_for_restart(state, backup)` — copy out, fill header, CRC.
- `restore_from_backup(state, backup) -> Result<(), RestoreError>` —
  validate, copy in.
- `invalidate(backup)` — zero the magic so the next boot can't mistake
  stale bytes for a valid backup.
- A `no_std`, no-table CRC-32/IEEE implementation
  (`compute_crc32`, `crc32_update`, both private).

### What this module does NOT provide

- **Persistence policy** — when to snapshot, how often, in which
  task. That belongs to whatever schedules the BKPSRAM write on the
  bare-metal target (currently planned as an Executive job after every
  successful SERVICER cycle).
- **A hardware driver** — there is no `unsafe` BKPSRAM pointer access
  in this module. The bare-metal binary owns that one-line cast.
- **(De)serialisation** — `BackupState` is `#[repr(C)]` and used as a
  raw struct in BKPSRAM. No formats, no byte order conversion. The
  `version` field is the lever that detects layout changes between
  firmware builds.

---

## 2. AGC Background

In Comanche055 the survives-RESTART erasables are scattered across
`ERASABLE_ASSIGNMENTS.agc`: `RN`/`VN` (state vectors), `REFSMMAT`,
`GHABASE`, `TIME1/2`, `IMODES30/33`, the phase registers, `MMNUMBER`
(major mode), and the flagword family. The AGC's `RESTARTS_ROUTINE`
re-reads those cells and re-dispatches active restart groups from the
phase registers.

This module is the Rust-side equivalent: one struct, one CRC, one
boot path. The exact field choices match the documented "survives
RESTART" set in `executive-spec.md` §5 and `fresh_start.rs`.

---

## 3. Rust API

### 3.1 Constants

```rust
pub const MAGIC: [u8; 4] = *b"AGC1";
pub const VERSION: u32 = 1;
```

`MAGIC` is the four-byte sentinel at offset 0. Absence in BKPSRAM
means uninitialised (first boot, battery removed) and triggers
`RestoreError::MagicMismatch`. `VERSION` is bumped whenever the
layout of `BackupState` changes — a downstream `restore_from_backup`
catches the mismatch and the caller falls back to FRESH START.

### 3.2 `BackupHeader` (16 bytes, fixed layout)

```rust
#[repr(C)]
pub struct BackupHeader {
    pub magic:   [u8; 4],
    pub version: u32,
    pub crc32:   u32,
    pub _pad:    u32,   // reserved, keeps payload 8-byte-aligned
}
```

`BackupHeader::ZERO` is a const all-zeros header — used by
`BackupState::zero()` to construct an "uninitialised" sentinel on the
host without touching unsafe memory.

The CRC field is at a fixed offset (8..12) so the CRC computation can
skip it.

### 3.3 `BackupState`

```rust
#[repr(C)]
pub struct BackupState {
    pub header:               BackupHeader,
    pub csm_state:            StateVector,
    pub target_state:         StateVector,
    pub refsmmat:             Mat3x3,
    pub time:                 Met,
    pub gha_epoch_rad:        f64,
    pub restart:              RestartProtection,
    pub pipa_cal:             PipaCalibration,
    pub gyro_comp:            GyroCompensation,
    pub last_drift_comp_time: Met,
    pub imu_alignment_state:  ImuAlignmentState,
    pub tpi_arrival_epoch:    Option<f64>,
    pub major_mode:           u8,
    pub flagwords:            [u16; 12],
}
```

| Field | Why preserved |
|---|---|
| `csm_state`, `target_state` | Navigation state — losing it ends the mission. |
| `refsmmat` | IMU-to-inertial; re-deriving needs P51/P52. |
| `time` | Mission clock. |
| `gha_epoch_rad` | Mission Control uplink, expensive to re-acquire. |
| `restart` | Phase registers — the whole point of restart protection. |
| `pipa_cal`, `gyro_comp`, `last_drift_comp_time` | Uplink-calibrated. |
| `imu_alignment_state` | Hardware platform's actual state. |
| `tpi_arrival_epoch` | Active rendezvous arrival time. |
| `major_mode` | So RESTART knows which program to re-enter. |
| `flagwords` | Persistent flags (e.g. `ENGINEON`). |

Everything else (scheduler queues, V/N input state, DSKY, alarm
struct, marks counters, …) is rebuilt by `fresh_start::restart`.

`BackupState::zero()` is a `const fn` that produces an all-zero
header + canonical-default payload — used as a host fixture; **not** a
valid backup (`restore_from_backup` returns `MagicMismatch`).

### 3.4 `RestoreError`

```rust
pub enum RestoreError {
    MagicMismatch,
    VersionMismatch   { found: u32, expected: u32 },
    ChecksumMismatch  { found: u32, expected: u32 },
}
```

All three are recoverable in the same way: the boot path falls back
to FRESH START.

### 3.5 Free functions

```rust
pub fn snapshot_for_restart(state: &AgcState, backup: &mut BackupState);
pub fn restore_from_backup(state: &mut AgcState, backup: &BackupState)
    -> Result<(), RestoreError>;
pub fn invalidate(backup: &mut BackupState);
```

---

## 4. Functional Requirements

### 4.1 `snapshot_for_restart`

1. Copy every payload field from `state` into `backup`.
2. Write `backup.header.magic = MAGIC` and `backup.header.version = VERSION`
   and `backup.header._pad = 0`.
3. Compute `backup.header.crc32 = compute_crc32(backup)`. Because the
   header's `crc32` slot is skipped during the computation, the order
   here (CRC last) is what makes the algorithm consistent.

Idempotent: calling `snapshot_for_restart` twice on the same
unchanging `state` produces an identical, valid backup
(TC-BACKUP-8).

### 4.2 `restore_from_backup`

1. Reject `MagicMismatch` if `backup.header.magic != MAGIC`.
2. Reject `VersionMismatch` if `backup.header.version != VERSION`.
3. Compute the CRC over `backup` (skipping bytes 8..12) and reject
   `ChecksumMismatch` if it doesn't match `backup.header.crc32`.
4. Copy every payload field into `state`.

`state` is **not** modified on any error — the caller can safely
trust its pre-call contents (TC-BACKUP-5).

### 4.3 `invalidate`

Sets `backup.header.magic = [0; 4]`. Sufficient to cause a future
`restore_from_backup` to fail at step 1 — the payload doesn't need
scrubbing because the next `snapshot_for_restart` will overwrite
it.

### 4.4 CRC-32/IEEE — no-table implementation

`compute_crc32(backup)` walks the backup's bytes via
`core::slice::from_raw_parts` and skips offsets 8..12 (the CRC field
itself). The polynomial is the reflected `0xEDB8_8320` form
(equivalent to the standard CRC-32/IEEE). No 256-entry lookup table
is used — the bare-metal target has tight flash budget and the CRC is
computed once per backup (cold/warm boot, not on every cycle).

The implementation is a private `#[no_std]`-safe inner loop; no
allocation, no panics, no float ops.

#### Safety note

The byte-slice view is sound because:
- `BackupState: Copy` and `#[repr(C)]` — layout is stable and
  POD-like.
- The `&[u8]` slice does not outlive the `&BackupState` borrow.
- The slice length is `core::mem::size_of::<BackupState>()` exactly,
  so there is no over-read.

---

## 5. Memory Budget

```
size_of::<BackupHeader>()  == 16  bytes  (asserted by TC-BACKUP-1)
size_of::<BackupState>()   <  4096 bytes (asserted by TC-BACKUP-2)
```

The 4 KB BKPSRAM region has comfortable headroom for the eventual
addition of further survives-RESTART fields without resizing.

---

## 6. Test Cases

| ID | Coverage |
|---|---|
| TC-BACKUP-1 | `BackupHeader` is exactly 16 bytes. |
| TC-BACKUP-2 | `BackupState` < 4 KB so it fits in BKPSRAM. |
| TC-BACKUP-3 | An all-zero backup fails restore with `MagicMismatch`. |
| TC-BACKUP-4 | Full round-trip — every payload field survives `snapshot → restore`. |
| TC-BACKUP-5 | Version tampering is detected; the target state is **not** modified. |
| TC-BACKUP-6 | A bit flipped in the payload (no header change) triggers `ChecksumMismatch`. |
| TC-BACKUP-7 | `invalidate` drops the magic so the next restore fails fast. |
| TC-BACKUP-8 | Double snapshot still verifies (idempotency). |

---

## 7. Module Layout

```
src/services/backup.rs
├── pub const MAGIC, VERSION
├── pub struct BackupHeader            (#[repr(C)], 16 bytes)
├── pub struct BackupState             (#[repr(C)], header + 13 payload fields)
├── impl BackupState { zero }
├── pub enum RestoreError
├── pub fn snapshot_for_restart
├── pub fn restore_from_backup
├── pub fn invalidate
├── fn compute_crc32                   (private, no-table CRC-32/IEEE)
├── fn crc32_update                    (private)
└── #[cfg(test)] mod tests             (TC-BACKUP-1..8)
```

---

## 8. Spec Quality Checklist

- [x] Header layout (offsets, sizes) documented (§3.2).
- [x] Field-by-field rationale for the backup payload (§3.3 table).
- [x] All three `RestoreError` variants and the caller's fallback
      (FRESH START) documented (§3.4, §4.2).
- [x] Idempotency of `snapshot_for_restart` and atomicity of
      `restore_from_backup` (state untouched on error) called out
      (§4.1, §4.2).
- [x] CRC-32/IEEE choice, table-free implementation, and `unsafe`
      slice rationale captured (§4.4).
- [x] BKPSRAM size budget asserted (§5).
- [x] Out-of-scope items (persistence policy, hardware driver,
      serialisation) explicitly listed (§1).
