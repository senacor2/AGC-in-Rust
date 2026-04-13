# Functional Specification: Executive Restart Protection

## AGC Source Reference

```
AGC source: Comanche055/FRESH_START_AND_RESTART.agc
Pages:      181–221 (MIT hardcopy pagination)
Routines:   GOPROG (page 186), GOPROG3 (page 188), NXTRST/PACTIVE/PINACT (page 189),
            MR.KLEAN (page 185), STARTSB2 (page 191), DOFSTART (page 184),
            NEWPHASE (page 221), PHASCHNG (called throughout Comanche055),
            RESTARTS (referenced via RACTCADR, page 189)

AGC source: Comanche055/RESTART_TABLES.agc
Pages:      211–221 (MIT hardcopy pagination)
Routines:   PRDTTAB, CADRTAB, SIZETAB, 1.2SPOT–6.2SPOT (even tables),
            1.3SPOT–6.3SPOT (odd tables), per-group phase spots

Supporting: ERASABLE_ASSIGNMENTS.agc
  -PHASE1..PHASE6  lines 1759–1770  (12 words — phase table)
  TBASE1..TBASE6   lines 1837–1847  (6 words — time-base registers)
  PHSPRDT1..6      lines 1838–1848  (6 words — per-group restart priorities)
  PHSNAME1..6      lines 2119–2129  (6 words — per-group restart name/CADR)
  REDOCTR          line 1915        (restart counter)
```

## Behavior Summary

Restart protection is the AGC's mechanism for resuming multi-step computations
after a hardware restart (GOJAM). When a computation spans several sequential
phases (e.g., IMU alignment → coarse align → fine align), each phase boundary
is bracketed by a phase table write. On restart, the AGC reads the phase table
to determine which phase was in progress and re-dispatches the computation
at the correct re-entry point.

### Restart Groups

The Comanche055 source uses **6 restart groups**, numbered 1–6.

Authoritative evidence:

- `FRESH_START_AND_RESTART.agc` line 658: `NUMGRPS EQUALS FIVE`
  — the GOPROG3 loop counter counts groups in descending order starting from 5,
  visiting groups 5, 4, 3, 2, 1. But the MR.KLEAN routine (page 185) clears
  `-PHASE6`/`PHASE6` in addition to groups 1–5, and `ERASABLE_ASSIGNMENTS.agc`
  allocates 12 words (`-PHASE1`..`PHASE6`, lines 1759–1770) = 6 pairs.
  `RESTART_TABLES.agc` defines spots up to `6.2SPOT` and `6.3SPOT`.
  `PHSNAME1`..`PHSNAME6` in ERASABLE_ASSIGNMENTS.agc (lines 2119–2129)
  also confirms 6 groups.

- **Reconciliation:** `NUMGRPS EQUALS FIVE` controls the GOPROG3 phase-table
  verification loop (verifying groups 1–5 in the standard restart path).
  Group 6 exists and has phase entries in `RESTART_TABLES.agc` but is not
  verified by the GOPROG3 loop — it is used for special cases (TVC DAP clock
  task, update program). The developer must allocate **6 groups** (indices 1–6)
  in the phase table, matching `MR.KLEAN` and `ERASABLE_ASSIGNMENTS.agc`.
  The authoritative constant from `docs/agc-reference-constants.md` is
  **`NUM_RESTART_GROUPS = 6`**.

### Phase Table Layout

For each group G (1–6), two consecutive AGC erasable words form the phase entry:

```
-PHASE_G   (ones-complement negative of the phase value)
 PHASE_G   (the phase value itself, positive)
```

A group is "active" (has a pending computation) when `PHASE_G != 0`.
A group is "inactive" when `PHASE_G == 0` (and `-PHASE_G == 0`, i.e., `-0`).

**Why two words?** The double write creates an atomic-read invariant: on
restart, the AGC checks `RXOR(−PHASE_G, PHASE_G)` — this must equal `-0`
(all-ones exclusive-OR in ones-complement = zero-with-sign). If the XOR is
non-zero, the write was interrupted mid-stream and the phase is corrupted.
GOPROG3 treats this as a phase-table error (alarm 1107) and falls through to
a fresh start.

In Rust the same atomic guarantee is achieved by writing both words inside
`interrupt::free` and using `core::sync::atomic::compiler_fence` (or the
equivalent cortex-m critical-section) to prevent reordering.

### PHASCHNG — Setting a Phase

`PHASCHNG` is the primary user-facing call to update the phase table.
It is placed in **fixed-fixed ROM** (so it survives bank switching during
restart). Calling sequence:

```agc
TC   PHASCHNG       ; jump to the routine
OCT  G×10 + P       ; inline word: high digit = group (1–6), low = phase (octal)
```

Example from `ALARM_AND_ABORT.agc` comment:
`TC PHASCHNG; OCT X.1` — group X, phase 1.

Example from `SERVICER207.agc`:
`TC PHASCHNG; OCT 10035` — the octal encodes group 5, phase 3 (5×10 + 3 = 53 octal = the raw encoding used by NEWPHASE for group 5).

The actual low-level implementation is `NEWPHASE` (RESTART_TABLES.agc page 220).
`NEWPHASE` receives:
- A = ±phase value (negative means "set TBASE too")
- Inline: group number (1–6)

Then:
1. `INHINT` (disable interrupts).
2. Index the group to compute the address of `−PHASE_G`.
3. Write the ones-complement negative of the phase to `−PHASE_G`.
4. Write the phase itself to `PHASE_G`.
5. Optionally write `-C(TIME1)` to `TBASE_G` if the phase was supplied negative.
6. `RELINT` and return to caller+2.

**TBASE.** Each group has an associated time-base register (`TBASE1`–`TBASE6`).
When a phase is set with a negative value (indicating "set TBASE"), the
current mission time (`−C(TIME1)`) is stored in the TBASE register. On restart,
TBASE allows the restart walker to compute how much time has elapsed since the
phase was set and whether a Waitlist re-schedule is needed immediately or
after the remaining delta-time.

### Restart Table Format (RESTART_TABLES.agc)

For each group G and each phase P, there is a "phase spot" `G.P_SPOT`
occupying 3 consecutive ROM words:

```
PRDTTAB (= 12000 relative):  priority word (FINDVAC) or delta-T (WAITLIST/LONGCALL)
CADRTAB (= 12001 relative):  high word of 2CADR (task/job address)
         (= 12002 relative):  low word of 2CADR (BBCON / bank selector)
```

Interpretation of `PRDTTAB`:
- **Positive priority** → restart as a FINDVAC job.
- **Negative priority** → restart as a NOVAC job (ones-complement of priority).
- **Positive number, 2CADR is negative** → restart as a WAITLIST task with
  this as delta-T in centiseconds.
- **Negative number, 2CADR is negative** → indirect: `PRDTTAB` is the `-GENADR`
  of an erasable location holding the delta-T.
- **`OCT 77777` (= -0), 2CADR is negative** → immediate WAITLIST restart
  (task fires immediately, effectively delta-T = 0).
- **GENADR of delta-T, 2CADR is negative** → LONGCALL with the referenced DP
  delta-T value.

Even phase spots (e.g., `3.2SPOT`) define two sequential restart entries (6
words): the first 3 words are an even entry; the next 3 words are a second
entry (used when phase must restart two things simultaneously, e.g., a job and
a Waitlist task).

Odd phase spots (e.g., `3.3SPOT`) define one entry (3 words).

### GOPROG — Hardware Restart Entry

`GOPROG` is the entry point at ROM address 4000, invoked by the AGC hardware
after GOJAM (power glitch, watchdog, software fault):

1. Increment `REDOCTR` (restart counter in erasable memory).
2. Save Q and SUPERBNK in `RSBBQ`.
3. Call `VAC5STOR` to capture erasable memory for debugging.
4. Check hardware condition bits (oscillator fail, AGC warning, mark-reject).
   If fatal hardware failure → `DOFSTART` (full fresh start, no recovery).
5. Otherwise → `STARTSUB` (minimal hardware re-init) → `GOPROG3`.

### GOPROG3 — Phase Table Verification and Dispatch

1. **Verify** phase table integrity: for each group G from 5 down to 1
   (using NUMGRPS = 5), read `−PHASE_G` into A and `PHASE_G` into L,
   then `RXOR` against `LCHAN` (the L register). The result must be `-0`.
   If any group has corrupted data, raise alarm 1107 and fall through to
   `DOFSTART`.
2. Display the current major mode on DSKY (`MMDSPLAY`).
3. **Walk active groups**: for each group G from 5 down to 1, if `PHASE_G > 0`
   (positive non-zero), the group is active. Call `RESTARTS` to re-dispatch
   the computation at the entry point defined by `G.PHASE_G_SPOT` in
   `RESTART_TABLES.agc`.
4. If no group was active and no active-flag set: display alarm 1110
   (restart with no active groups) and go to DUMMYJOB idle loop.

### RESTARTS Subroutine

`RESTARTS` (referenced via `RACTCADR` at page 189) is the runtime walker that
reads the restart table entry for (group G, phase P) and re-schedules the
computation:

1. Locate the spot `G.P_SPOT` by indexing `SIZETAB` with group and phase.
2. Read `PRDTTAB` and `CADRTAB` (3 words).
3. Based on the encoding:
   - Job (positive PRDTTAB, positive 2CADR): call `FINDVAC` or `NOVAC`.
   - Waitlist task (negative 2CADR): call `WAITLIST` with the delta-T.
   - LONGCALL: call `LONGCALL` with the DP delta-T from erasable.
   - Immediate Waitlist (PRDTTAB = -0): call `WAITLIST` with delta-T = 1.

### MR.KLEAN — Clear Phase Table

Clears all 6 groups by writing `-0` to every `-PHASEn`/`PHASEn` pair.
Called during fresh start and by V37 (major mode change) to kill active programs.

```agc
MR.KLEAN   INHINT
           EXTEND
           DCA    NEG0
           DXCH   -PHASE2      ; clears groups 2 and 1 (layout is adjacent)
           ...
           DXCH   -PHASE6      ; clears groups 5 and 6
           TC     Q
```

(From `FRESH_START_AND_RESTART.agc` page 185, lines 264–283.)

### Safe-State Defaults

On restart (GOPROG), all outputs default to safe state before phase table
walk begins:

| Output | Default action | AGC source |
|---|---|---|
| RCS jets | Write 0 to channels 5 and 6 | STARTSUB → `WRITE CHAN5/CHAN6` |
| DSKY lamps | Extinguish all except alarm/gimbal lock/no-att | GOPROG: `CA 9,6,4; MASK DSPTAB+11D` |
| Engine | Restore based on `ENGONBIT` flag | GOPROG: `WOR DSALMOUT bit 13` if flagged |
| TVC DAP | Stop rate | GOPROG3 / `STOPRATE` call |
| Waitlist | All tasks replaced with ENDTASK | STARTSB2 |
| Executive | All core sets freed | STARTSB2 |

### Persistent State

The phase table (`-PHASE1`..`PHASE6`, `TBASE1`..`TBASE6`) must survive a
hardware restart. In the original AGC, erasable memory (SRAM) is battery-backed
and its contents survive GOJAM. In the Rust port, the equivalent is a
memory region that is NOT zeroed during boot.

On a Cortex-M4F target, this is achieved with a custom linker section placed
in a `.noinit` (uninitialised) RAM region. The startup code must not clear
this region. The developer marks the phase table with `#[link_section = ".noinit"]`
(or the equivalent for the chosen linker script) so that a power-on reset
zeros it (no recovery possible) but a software/watchdog restart preserves it.

---

## Rust API

### Module Path

`agc_core::executive::restart`

### Types

```rust
/// Index of a restart group. Valid values: 1..=NUM_RESTART_GROUPS.
/// 0 is not a valid group index.
/// AGC source: groups 1–6 in FRESH_START_AND_RESTART.agc and RESTART_TABLES.agc.
pub type GroupId = u8;

/// Phase number within a group. Valid values: 0..=127.
/// 0 means "group inactive" (corresponds to +0 in the AGC phase word).
/// AGC source: phase spots 1.2SPOT, 1.3SPOT, ... encoded as octal phase number.
pub type Phase = u8;

/// Number of restart groups.
/// AGC source: ERASABLE_ASSIGNMENTS.agc -PHASE1..-PHASE6 (12 words = 6 pairs).
/// Note: NUMGRPS in the AGC source = FIVE (5) because GOPROG3's loop walks
/// groups 1–5 for standard verification. Group 6 exists and is used (TVC, P27)
/// but is not walked by GOPROG3. The Rust implementation allocates 6 groups.
pub const NUM_RESTART_GROUPS: usize = 6;

/// One group's phase state.
///
/// The AGC stores this as two consecutive erasable words:
///   word 0:  -phase (ones-complement negative)
///   word 1:  +phase (the phase value)
/// Both words must agree (XOR = -0) for the table to be valid.
///
/// AGC source: ERASABLE_ASSIGNMENTS.agc lines 1759–1770.
#[derive(Clone, Copy)]
pub struct GroupState {
    /// Current phase. 0 means this group is inactive.
    /// AGC source: PHASE_G register (positive value).
    pub phase: Phase,
    /// Time-base for this group: the negative of TIME1 at the moment
    /// this phase was set (used by RESTARTS to compute elapsed time).
    /// AGC source: TBASE1..TBASE6 (ERASABLE_ASSIGNMENTS.agc lines 1837–1847).
    pub tbase: i16,
    /// Per-group restart priority, set by the caller before PHASCHNG.
    /// Encoded in PHSPRDT1..6.
    /// AGC source: ERASABLE_ASSIGNMENTS.agc lines 1838–1848.
    pub restart_priority: u16,
}

/// The complete restart protection state for all 6 groups.
///
/// This struct must reside in a memory region that survives hardware restart.
/// On bare metal, mark with `#[link_section = ".noinit"]` and verify
/// integrity via the double-write invariant on every access.
///
/// AGC source: FRESH_START_AND_RESTART.agc; RESTART_TABLES.agc.
pub struct RestartProtection {
    /// Per-group state. Index 0 = group 1, ..., index 5 = group 6.
    pub(crate) groups: [GroupState; NUM_RESTART_GROUPS],
    /// Restart counter: incremented on every GOPROG entry.
    /// AGC source: REDOCTR (ERASABLE_ASSIGNMENTS.agc line 1915).
    pub(crate) restart_count: u16,
    /// Shadow array holding the ones-complement negatives of each group's
    /// phase value, for the double-write integrity check.
    /// AGC source: -PHASE1..-PHASE6 (ERASABLE_ASSIGNMENTS.agc lines 1759–1770,
    ///   odd entries).
    pub(crate) neg_phase: [i16; NUM_RESTART_GROUPS],
}

/// Describes how a group should be re-dispatched on restart.
/// Derived from reading the G.P_SPOT entry in RESTART_TABLES.agc.
/// AGC source: RESTART_TABLES.agc, PRDTTAB/CADRTAB fields.
pub enum RestartAction {
    /// Re-schedule as a FINDVAC job (positive priority in PRDTTAB).
    /// AGC source: RESTART_TABLES.agc — positive PRDTTAB, positive 2CADR.
    FindvacJob { priority: u16, entry: JobFn },
    /// Re-schedule as a NOVAC job (negative priority in PRDTTAB).
    /// AGC source: RESTART_TABLES.agc — negative PRDTTAB, positive 2CADR.
    NovacJob   { priority: u16, entry: JobFn },
    /// Re-schedule as a Waitlist task after `delta_cs` centiseconds.
    /// AGC source: RESTART_TABLES.agc — negative 2CADR, positive PRDTTAB.
    WaitlistTask { delta_cs: u16, task: TaskFn },
    /// Re-schedule as an immediate Waitlist task (PRDTTAB = -0 / OCT 77777).
    /// AGC source: RESTART_TABLES.agc — negative 2CADR, PRDTTAB = 77777.
    ImmediateTask { task: TaskFn },
}
```

### Functions

```rust
impl RestartProtection {
    /// Create a zero-initialized instance (all groups inactive).
    /// Used for fresh start only. On hardware restart the existing
    /// contents must be preserved, not re-initialized.
    ///
    /// AGC source: MR.KLEAN (FRESH_START_AND_RESTART.agc page 185).
    pub const fn new() -> Self;

    /// Set the phase for a group. Both the direct and shadow (negative)
    /// words are written inside a single `interrupt::free` critical section.
    ///
    /// `group` must be 1..=NUM_RESTART_GROUPS; panics in debug, wraps in release.
    /// `phase` = 0 clears the group (marks it inactive).
    ///
    /// Optionally sets the time-base (`tbase`) to `-TIME1` if `set_tbase` is true.
    ///
    /// AGC source: NEWPHASE / PHASCHNG (RESTART_TABLES.agc page 220–221).
    pub fn set_phase(
        &mut self,
        group: GroupId,
        phase: Phase,
        set_tbase: bool,
        time1: i16,
    );

    /// Read the current phase for a group.
    ///
    /// Returns 0 if the group is inactive.
    /// Does not check integrity; use `verify_integrity` for that.
    ///
    /// AGC source: CCS PHASE1 in GOPROG3 (FRESH_START_AND_RESTART.agc page 189).
    pub fn current_phase(&self, group: GroupId) -> Phase;

    /// Check phase table integrity for all 6 groups.
    ///
    /// For each group, verifies that `neg_phase[i] XOR phase[i]` equals
    /// the ones-complement of zero (i.e., the complement pair is consistent).
    ///
    /// Returns `Ok(())` if all groups pass.
    /// Returns `Err(group)` identifying the first corrupt group.
    ///
    /// AGC source: GOPROG3 phase-table verification loop using RXOR / LCHAN
    ///   (FRESH_START_AND_RESTART.agc page 188, PCLOOP).
    pub fn verify_integrity(&self) -> Result<(), GroupId>;

    /// Clear all groups (set all phases to 0).
    ///
    /// Called during fresh start and by V37 major-mode change.
    ///
    /// AGC source: MR.KLEAN (FRESH_START_AND_RESTART.agc page 185).
    pub fn clear_all(&mut self);

    /// Clear a specific group (set phase to 0).
    ///
    /// AGC source: `DCA NEG0; DXCH -PHASE_G` pattern throughout programs.
    pub fn clear_group(&mut self, group: GroupId);

    /// On-restart walker: scan groups 1–5 (NUMGRPS) for active phases and
    /// return the list of restart actions to perform.
    ///
    /// The caller (GOPROG3 equivalent) must call `verify_integrity` first
    /// and abort to fresh-start if it fails. Then call `on_restart` to
    /// collect actions and dispatch them via `Executive::add_job` /
    /// `Waitlist::schedule`.
    ///
    /// Groups are scanned in descending order (5 down to 1), matching the
    /// AGC GOPROG3 loop direction. Group 6 is intentionally excluded from
    /// the standard walk (matching `NUMGRPS = FIVE`).
    ///
    /// Returns an array of up to NUM_RESTART_GROUPS actions; entries beyond
    /// `count` are `None`.
    ///
    /// AGC source: GOPROG3 NXTRST/PACTIVE loop (page 189) and RESTARTS
    ///   subroutine (called via RACTCADR).
    pub fn on_restart(
        &self,
        tables: &RestartTables,
    ) -> ([Option<RestartAction>; NUM_RESTART_GROUPS], usize);

    /// Return the current restart counter value.
    /// AGC source: REDOCTR (ERASABLE_ASSIGNMENTS.agc line 1915).
    pub fn restart_count(&self) -> u16;

    /// Increment the restart counter (called at the start of GOPROG).
    /// AGC source: GOPROG `INCR REDOCTR` (FRESH_START_AND_RESTART.agc page 186).
    pub fn increment_restart_count(&mut self);
}

/// The compile-time restart tables (ROM equivalent of RESTART_TABLES.agc).
///
/// In the AGC, the tables live in fixed ROM. In Rust they are `const` arrays.
/// The developer must populate this from RESTART_TABLES.agc entries.
///
/// AGC source: RESTART_TABLES.agc SIZETAB / PRDTTAB / CADRTAB structure.
pub struct RestartTables {
    // Entries indexed by (group-1, phase/2) for even spots and (group-1, (phase-1)/2) for odd spots.
    // Developer: implement as a 2D array or match table, populated from RESTART_TABLES.agc.
    // This spec does not prescribe the internal layout; it prescribes the lookup behavior.
    pub(crate) _marker: core::marker::PhantomData<()>,
}

impl RestartTables {
    /// Look up the restart action for (group, phase).
    ///
    /// Returns `None` if no table entry exists for this (group, phase) pair.
    ///
    /// AGC source: RESTART_TABLES.agc SIZETAB / G.P_SPOT entries.
    pub fn lookup(&self, group: GroupId, phase: Phase) -> Option<RestartAction>;
}
```

### Persistent State via Linker Section

The `RestartProtection` struct must survive hardware restart. The developer
must declare it as a static in a `.noinit` section:

```rust
// NOTE: Do not implement this directly — it is an architecture decision.
// Document here for the developer.
//
// In the target linker script (memory.x), define:
//   .noinit (NOLOAD) : { *(.noinit) } > RAM
//
// In agc_core::executive::restart:
//   #[link_section = ".noinit"]
//   static mut RESTART_STATE: RestartProtection = RestartProtection::new();
//
// Safety invariant: RESTART_STATE is only accessed through the public API
// functions (set_phase, current_phase, on_restart), each of which wraps
// access in `cortex_m::interrupt::free`. The `static mut` is justified
// because the .noinit region cannot use `Mutex<RefCell<T>>` (it needs to be
// zero-initialized only on power-on, not on every reset — but cortex-m's
// Mutex::new is const). The developer should wrap in UnsafeCell and
// provide the interrupt::free access pattern explicitly.
//
// Alternative: use cortex_m::interrupt::Mutex<core::cell::UnsafeCell<...>>
// with manual pointer reads/writes inside interrupt::free.
```

This is an architecture decision deferred to the developer. The spec requires
that whatever mechanism is used:
- Writes to `phase` and `neg_phase` for the same group occur atomically
  (both in the same `interrupt::free` block).
- Reads occur inside `interrupt::free` as well.
- On a power-on reset (not a software restart), the region is zero, which
  maps to "all groups inactive" — a valid safe state.

---

## Scale Factors

The restart protection subsystem does not perform navigation math. Scale
factors apply only to the time-base field:

| Field | AGC encoding | Rust type | Notes |
|---|---|---|---|
| `phase` | Positive integer, 0..127 | `u8` | 0 = inactive, 1+ = phase number |
| `tbase` | `-C(TIME1)` in AGC time units (centiseconds) | `i16` | Stored as raw mission-time complement |
| `restart_priority` | 15-bit ones-complement priority | `u16` | Matches Executive priority encoding |
| `restart_count` | Monotonic counter in erasable | `u16` | Wraps after 65535 restarts |

---

## Invariants

1. **Double-write atomicity.** For every call to `set_phase`, both
   `groups[g].phase` and `neg_phase[g]` must be written within a single
   `interrupt::free` closure. A partial write (interrupted by T3RUPT)
   must still leave the pair in a detectable-corrupt state (RXOR ≠ -0),
   triggering alarm 1107 on the next restart.

2. **Integrity check before dispatch.** `on_restart` must only be called
   after `verify_integrity` has returned `Ok(())`. Calling `on_restart` on
   corrupted state invokes undefined behavior relative to the AGC model.

3. **No heap.** `RestartProtection` and `RestartTables` use only fixed-size
   arrays and `PhantomData`. No dynamic allocation.

4. **No `static mut` (preferred).** Where the linker-section constraint makes
   `static mut` unavoidable, it must be wrapped in `UnsafeCell` and accessed
   only inside `interrupt::free`. Every `unsafe` block must carry a comment
   justifying the invariant upheld.

5. **Phase 0 = inactive.** `set_phase(g, 0, ...)` is equivalent to
   `clear_group(g)`. After a clear, both `phase` and `neg_phase` for that
   group are zero.

6. **`verify_integrity` uses `Result`.** This is an exception to the
   `agc-core` "use `Option` not `Result`" rule. Phase-table integrity
   is a safety-critical structural check (not a runtime operational error);
   returning which group failed is necessary for the caller to raise the
   correct alarm (1107). The alarm call itself uses the `Option`-based
   `alarm::raise` path.

7. **Group 6 behavior.** Group 6 exists in the tables and can have its phase
   set/cleared. It is not walked by the standard `on_restart` loop (which
   mirrors NUMGRPS = FIVE). Code that needs Group 6 restart behavior
   (TVC DAP, P27 update program) must handle it separately via a direct
   `lookup` call.

---

## Test Cases

### Test 1 — Set and Read Phase

**Setup:** Fresh `RestartProtection`. Call `set_phase(group: 3, phase: 5, set_tbase: false, time1: 0)`.

**Expected:**
- `current_phase(3)` returns 5.
- `current_phase(1)`, `current_phase(2)`, `current_phase(4..6)` return 0.
- `verify_integrity()` returns `Ok(())`.
- Internal `neg_phase[2]` equals the ones-complement negative of 5 (i.e., `-5`
  in i16 ones-complement = `!5_i16 = -6` two's complement representation,
  but matching ones-complement convention).

### Test 2 — Clear and Verify

**Setup:** `RestartProtection` with groups 1 and 4 set to phases 3 and 7
respectively. Call `clear_group(4)`.

**Expected:**
- `current_phase(4)` returns 0.
- `current_phase(1)` still returns 3.
- `verify_integrity()` returns `Ok(())`.

Call `clear_all()`.

**Expected:**
- All `current_phase(g)` for g in 1..=6 return 0.
- `verify_integrity()` returns `Ok(())`.

### Test 3 — Simulated Mid-Computation Restart

**Setup:** A multi-phase computation that uses phases 1, 2, 3 for group 5:

1. Computation enters phase 1: `set_phase(5, 1, false, 0)`.
2. Computation reaches phase 2: `set_phase(5, 2, false, 0)`.
3. Restart occurs here (simulated by calling `on_restart` without completing phase 3).
4. `verify_integrity()` passes (both writes of phase 2 completed).
5. Call `on_restart(&tables)` with a mock `RestartTables` that maps
   (group 5, phase 2) → `RestartAction::FindvacJob { priority: 0x1000, entry: my_resume_fn }`.

**Expected:**
- `on_restart` returns an array with one `Some(RestartAction::FindvacJob {...})`
  at the group 5 position.
- The returned action's priority is 0x1000.
- The returned action's `entry` matches `my_resume_fn`.
- `current_phase(5)` is still 2 (on_restart is read-only).
- The caller is expected to call `Executive::add_job(0x1000, my_resume_fn)` next.

---

## agc-sim Impact

- `MissionState`: add field `restart_count: u16` (mirrors `RestartProtection::restart_count()`).
- `MissionState`: add field `active_restart_groups: u8` (bitmask, bit i set if group i+1 is active).
- `SimLog`: emit `.warn("RESTART: group {g} phase {p} — dispatching {action}")` during `on_restart`.
- `SimLog`: emit `.error("PHASE TABLE CORRUPT: group {g} — alarm 1107 → fresh start")` if `verify_integrity` fails.
- `dsky_terminal.rs`: no new DSKY lights; the restart counter can be displayed in the Mission State panel as "RST: {n}".
