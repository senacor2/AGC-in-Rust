# Functional Specification: Executive Scheduler

## AGC Source Reference

```
AGC source: Comanche055/EXECUTIVE.agc
Pages:      1208–1220 (MIT hardcopy pagination)
Routines:   NOVAC, FINDVAC, SPVAC, CHANG2, CHANJOB, ENDJOB1, EJSCAN,
            ENDOFJOB, DUMMYJOB, ADVAN, NUDIRECT, SUPDXCHZ
Supporting: ERASABLE_ASSIGNMENTS.agc (PRIORITY, LOC, BANKSET, PUSHLOC,
            MPAC, NEWJOB, NEWPRIO, NEWLOC, LOCCTR, VAC1USE–VAC5USE)
```

## Behavior Summary

The Executive is the cooperative, non-preemptive job scheduler for the AGC.
There is no operating system. The Executive implements a fixed-size table of
job slots (core sets), scans for the highest-priority runnable job, swaps
register state, and dispatches it. Jobs run to completion or until they
voluntarily yield via `ENDOFJOB`, `CHANG2`, `JOBSLEEP`, or `PRIOCHNG`.

### Concepts

**Core Set.** Each job occupies exactly one "core set," which is a contiguous
block of 12 AGC erasable registers:

```
MPAC    +0..+6   (7 words) — Multi-purpose accumulator, scratch
MODE    +7       (1 word)  — Type indicator (+1 TP, +0 DP, -1 vector)
LOC     +8       (1 word)  — Entry address of the job (positive = basic, negative = interpretive)
BANKSET +9       (1 word)  — Bank/superbank selector
PUSHLOC +10      (1 word)  — Packed interpretive push-down pointer and VAC base
PRIORITY+11      (1 word)  — Priority word; -0 means slot is free; positive = active; negative = sleeping
```

**Slot Count.** The AGC source (`ERASABLE_ASSIGNMENTS.agc`, line 1627) states:
`ERASE +71D # SEVEN SETS OF 12 REGISTERS EACH`. This means 7 core sets exist
(indices 0–6). The constant `NO.CORES` in EXECUTIVE.agc is `DEC 6`, which is
the loop count (one fewer because the scan starts at index 0 and counts down
from 6, visiting slots 0 through 6 inclusive). The authoritative constant from
`docs/agc-reference-constants.md` is **`MAX_JOBS = 7`**.

**Job Slot 0.** Core set 0 is the currently running job. The CHANJOB routine
swaps an incoming job into slot 0 before dispatching it.

**VAC Areas.** Jobs that use the AGC interpretive language require a "Vector
Accumulator" (VAC) area — a 43-word scratch buffer. Five VAC areas exist
(VAC1–VAC5, each 43 words at addresses 0400–0777 octal). In the Rust port
the interpretive language is not implemented (ADR-001), but the VAC area
concept still exists as the job's working memory buffer. Each VAC area has a
one-word use-register (`VACnUSE`): a non-negative value means the area is
reserved (the value encodes the VAC base address); zero (written after use)
means free.

**Priority.** Priority is a 15-bit positive integer (AGC word, ones-complement).
Higher numeric value = higher priority. Typical range observed in Comanche055
restart tables: 10 (octal) = 8 (decimal) to 37 (octal) = 31 (decimal).
Bit 15 is never set in a valid positive priority. When a slot is sleeping, its
PRIORITY word is stored as the ones-complement negative of the priority value.

### Scheduler Entry Points

The AGC source defines several entry points into the Executive; all disable
interrupts (`INHINT`) at entry and re-enable (`RELINT`) after state is safe.

| AGC routine | Rust equivalent | Purpose |
|---|---|---|
| `NOVAC` | `Executive::add_job(priority, addr, None)` | Add a job that needs no VAC area |
| `FINDVAC` | `Executive::add_job(priority, addr, Some(vac))` | Add a job that needs a VAC area; locates a free VAC |
| `SPVAC` | internal helper within `add_job` | FINDVAC variant when 2CADR is already in A/L |
| `CHANG2` | `Executive::yield_job()` | Voluntarily suspend an interpretive job |
| `CHANG1` | `Executive::yield_job()` | Voluntarily suspend a basic job |
| `ENDOFJOB` | `Executive::finish_job()` | Mark the current job complete, release slot |
| `JOBSLEEP` | `Executive::sleep_job(wakeup_addr)` | Suspend until an I/O event wakes it |
| `JOBWAKE` | `Executive::wake_job(addr)` | Wake a sleeping job by address match |
| `PRIOCHNG` | `Executive::change_priority(new_prio)` | Change priority of the running job |

### NOVAC / FINDVAC — Adding a Job

1. Save the caller's bank register.
2. (`FINDVAC` only) Scan `VAC1USE` through `VAC5USE`; take the first with a
   positive value (meaning it is free). Mark it reserved by zeroing the use
   register and packing the VAC base address into the low 9 bits of the new
   priority word.
3. If no VAC is free (`FINDVAC` path): call `BAILOUT` with alarm `OCT 1201`
   (no VAC areas). In the Rust port this raises alarm `1202` and returns
   `None` (see alarm mapping note below).
4. Scan the 7 core sets by examining each PRIORITY register. A slot is free
   when PRIORITY equals `-0` (ones-complement negative zero). Select the first
   free slot.
5. If no slot is free: call `BAILOUT` with alarm `OCT 1202`. In Rust this
   raises alarm `1202` via `alarm::raise(AlarmCode::ExecutiveOverflow)` and
   returns `None`. The computer does not abort; it attempts to continue the
   current activity.
6. Store the new priority in the found slot's PRIORITY register and the 2CADR
   (entry address + bank word) in LOC/BANKSET.
7. Compare the new job's priority with the currently running job (slot 0). If
   the new priority is strictly higher, record that a job swap is needed
   (`NEWJOB` is set to the new slot index).
8. Return to caller at call-site + 2 (i.e., past the inline 2CADR).

**Note on alarm code mapping.** The AGC source uses `OCT 1201` for "no VAC"
and `OCT 1202` for "no core set." Both conditions represent Executive overflow.
The reference constants table consolidates these as alarm `01202`
(`ExecutiveOverflow`). The Rust implementation raises a single `ExecutiveOverflow`
alarm for any scheduler overflow and returns `None`.

### EXEC Main Loop — Cooperative Dispatch

After each job finishes (`ENDOFJOB`) or yields (`CHANG2`), the scheduler
runs `EJSCAN` to find the runnable job with the highest positive PRIORITY
value:

1. Examine PRIORITY registers for all 7 slots at fixed offsets
   (`PRIORITY`, `PRIORITY+12D`, …, `PRIORITY+72D`).
2. Track the highest positive value found and the slot index it belongs to.
3. If no active job is found: enter `DUMMYJOB` (idle loop). The idle loop
   turns off the computer activity light, then spins polling `NEWJOB` until a
   new job appears (typically from a Waitlist T3RUPT callback or an I/O
   interrupt that calls NOVAC/FINDVAC).
4. If a job is found: set `NEWJOB` to the winning slot index, then call
   `CHANJOB` to perform the register swap.

### CHANJOB — Context Switch

`CHANJOB` (EXECUTIVE.agc, page 1213) swaps the MPAC (7 words), LOC, BANKSET,
PUSHLOC, and PRIORITY registers between core set 0 and the slot indexed by
`NEWJOB`. After the swap, slot 0 holds the new job's state and the old
job's state is preserved in its original slot. Dispatch is then via `DTCB`
(basic job, positive LOC) or via `INTRSM` (interpretive job, negative LOC).

In the Rust port there is no interpretive language. Every AGC routine that
used the interpretive language is re-implemented as a plain Rust function.
Therefore `CHANJOB` simplifies to: move the winning slot's fields into the
"current job" fields of `AgcState`, then call the function pointer stored
in `JobEntry::entry`.

### ENDOFJOB / ENDJOB1

When a job finishes:

1. Mark its slot free: write `-0` to PRIORITY.
2. If the job had a VAC area, release it by restoring the use register.
3. Call `EJSCAN` to find and dispatch the next highest-priority job.

### Interaction with Interrupts

The AGC runs at a fixed 1.024 MHz with 12-cycle (11.7 µs) machine cycles.
T3RUPT fires every 10 ms. Interrupt handlers can call `NOVAC`/`FINDVAC` to
schedule a new job (e.g., Waitlist calling back a foreground task). The
`INHINT`/`RELINT` pair surrounding each scheduler operation ensures
atomicity: no interrupt can fire between reading a slot's PRIORITY and
writing it.

In the Rust port, `INHINT`/`RELINT` map to `cortex_m::interrupt::free(|cs| ...)`.
All scheduler state (`Executive`) lives inside
`Mutex<RefCell<Executive>>` and is accessed only within `interrupt::free` closures.

---

## Rust API

### Module Path

`agc_core::executive::scheduler`

### Types

```rust
/// Priority of a job. Higher value = higher urgency.
/// Valid range: 1..=0x7FFF. Zero is reserved for "slot free".
pub type Priority = u16;

/// A function pointer representing the job entry point.
/// Must be a plain fn — no closures, no captures, no heap.
/// AGC cross-reference: LOC/BANKSET in each core set.
pub type JobFn = fn(&mut AgcState);

/// A Vector Accumulator area index (0-based, 0..=4).
/// None means the job does not require a VAC area (NOVAC path).
pub type VacIndex = u8;

/// One entry in the 7-slot job table.
///
/// Corresponds to one AGC "core set" (MPAC +0 through PRIORITY).
/// AGC source: ERASABLE_ASSIGNMENTS.agc, line 1618–1627.
pub struct JobEntry {
    /// Job entry point. None means the slot is free.
    /// AGC source: LOC + BANKSET registers.
    pub entry: Option<JobFn>,
    /// Priority. 0 means free (corresponds to -0 in ones-complement).
    /// Negative (sleeping) is encoded by the `sleeping` flag.
    pub priority: Priority,
    /// True when the job is suspended waiting for a JOBWAKE event.
    /// AGC source: JOBSLEEP stores -C(PRIORITY) to indicate sleep.
    pub sleeping: bool,
    /// Optional VAC area claimed by this job (FINDVAC path).
    /// AGC source: low 9 bits of the PRIORITY word when VAC is in use.
    pub vac: Option<VacIndex>,
}

/// The complete Executive scheduler state.
///
/// All fields are statically allocated arrays — no heap.
/// AGC source: EXECUTIVE.agc, routines NOVAC, FINDVAC, CHANJOB, EJSCAN.
pub struct Executive {
    /// The 7-slot job table.
    /// Slot 0 is the currently running job (swapped into place by CHANJOB).
    /// AGC source: ERASABLE_ASSIGNMENTS.agc, "DYNAMICALLY ALLOCATED CORE SETS
    ///   FOR JOBS (84D)" — 7 sets × 12 registers = 84 words.
    pub(crate) jobs: [JobEntry; MAX_JOBS],
    /// Index into `jobs` of the job currently executing (always 0 in the AGC).
    /// Maintained as a field here because Rust does not have a hardware
    /// swap instruction like DXCH.
    pub(crate) current: usize,
    /// VAC area availability: true = free, false = in use.
    /// AGC source: VAC1USE–VAC5USE in ERASABLE_ASSIGNMENTS.agc lines 1727–1736.
    pub(crate) vac_free: [bool; MAX_VAC_AREAS],
}

/// Maximum number of job slots.
/// AGC source: EXECUTIVE.agc `NO.CORES DEC 6` (loop counter = 6 → 7 slots).
/// Confirmed: ERASABLE_ASSIGNMENTS.agc line 1627 "SEVEN SETS OF 12 REGISTERS EACH".
pub const MAX_JOBS: usize = 7;

/// Number of VAC areas.
/// AGC source: ERASABLE_ASSIGNMENTS.agc lines 1727–1736 (VAC1–VAC5).
pub const MAX_VAC_AREAS: usize = 5;
```

### Functions

```rust
impl Executive {
    /// Create a zero-initialized Executive (all slots free, all VAC areas free).
    /// Called once during FRESH START / STARTSB2.
    /// AGC source: FRESH_START_AND_RESTART.agc STARTSB2, lines 562–583.
    pub const fn new() -> Self;

    /// Add a job to the scheduler (NOVAC path — no VAC area required).
    ///
    /// Returns `Some(slot_index)` on success.
    /// Returns `None` and raises alarm `ExecutiveOverflow` (1202) if all
    /// 7 slots are occupied.
    ///
    /// Must be called from within `cortex_m::interrupt::free`.
    /// AGC source: EXECUTIVE.agc NOVAC / NOVAC2 / CORFOUND.
    pub fn add_job(
        &mut self,
        priority: Priority,
        entry: JobFn,
    ) -> Option<usize>;

    /// Add a job that requires a VAC area (FINDVAC path).
    ///
    /// First locates a free VAC area; if none is available raises alarm 1202
    /// and returns `None`. Then falls through to `add_job` logic.
    ///
    /// AGC source: EXECUTIVE.agc FINDVAC / FINDVAC2 / VACFOUND.
    pub fn add_job_with_vac(
        &mut self,
        priority: Priority,
        entry: JobFn,
    ) -> Option<usize>;

    /// Find the highest-priority runnable (non-sleeping, non-None) job.
    ///
    /// Returns its slot index, or `None` if no runnable jobs exist
    /// (triggers DUMMYJOB idle loop in the caller).
    ///
    /// AGC source: EXECUTIVE.agc EJSCAN / EJ1 / EJ2.
    pub fn find_highest_priority(&self) -> Option<usize>;

    /// Dispatch the next job: swap the winning slot into current position
    /// and return its `JobFn`.
    ///
    /// Corresponds to CHANJOB in the AGC source.
    /// Returns `None` if no runnable job exists (caller enters idle loop).
    ///
    /// AGC source: EXECUTIVE.agc CHANJOB (page 1213).
    pub fn run_next(&mut self) -> Option<JobFn>;

    /// Mark the currently running job as complete and release its slot
    /// (and VAC area if any).
    ///
    /// AGC source: EXECUTIVE.agc ENDOFJOB / ENDJOB1.
    pub fn finish_job(&mut self);

    /// Voluntarily suspend the current job (lower-priority yield).
    /// The job remains in its slot but is deprioritized until a
    /// higher-priority job finishes.
    ///
    /// AGC source: EXECUTIVE.agc CHANG1 / CHANG2 / CHANJOB.
    pub fn yield_current(&mut self);

    /// Change the priority of the currently running job.
    ///
    /// If the new priority is lower than some other runnable job,
    /// `run_next()` is expected to be called afterward.
    ///
    /// AGC source: EXECUTIVE.agc PRIOCHNG / PRIOCH2.
    pub fn change_priority(&mut self, new_priority: Priority);

    /// Put the current job to sleep, waiting for a JOBWAKE signal.
    ///
    /// AGC source: EXECUTIVE.agc JOBSLEEP / JOBSLP1.
    pub fn sleep_current(&mut self, wakeup_addr: JobFn);

    /// Wake a sleeping job whose `entry` matches `wakeup_fn`.
    ///
    /// Scans all 7 slots for a sleeping job with matching entry.
    /// If found, clears the `sleeping` flag. Returns `true` if woken.
    ///
    /// AGC source: EXECUTIVE.agc JOBWAKE / JOBWAKE2 / WAKETEST.
    pub fn wake_job(&mut self, wakeup_fn: JobFn) -> bool;

    /// Return a reference to the currently active job entry (slot 0
    /// equivalent).
    pub fn current_job(&self) -> Option<&JobEntry>;
}
```

### Static Allocation

The `Executive` is stored in a `Mutex<RefCell<Executive>>` static:

```rust
// In agc_core::executive::scheduler (implementation sketch — not code)
// static EXECUTIVE: Mutex<RefCell<Executive>> = Mutex::new(RefCell::new(Executive::new()));
//
// Access pattern:
// cortex_m::interrupt::free(|cs| {
//     EXECUTIVE.borrow(cs).borrow_mut().add_job(prio, my_fn);
// });
```

No `static mut` is used. No heap allocations occur.

---

## Scale Factors

Priority values in the AGC are 15-bit ones-complement integers. In Rust they
are `u16` with the mapping:

| AGC octal | Decimal | Meaning |
|---|---|---|
| `00000` (= -0) | 0 | Slot free |
| `10000` | 4096 | Low-priority background job (typical) |
| `20000` | 8192 | Mid-priority navigation job |
| `37777` | 16383 | Highest representable priority |

The Rust port uses the same numeric values as the AGC source restart tables.
From `RESTART_TABLES.agc`: priority `OCT 10000` ≈ decimal 4096 is "low",
`OCT 32000` ≈ decimal 13312 is "high foreground", `OCT 37777` = `NEG1/2`
encodes "immediate restart" for Waitlist tasks.

---

## Invariants

1. **No heap.** `Executive` and `[JobEntry; 7]` are statically allocated.
   No `Vec`, `Box`, or `alloc` is used anywhere in this module.

2. **No `static mut`.** All shared state is accessed through
   `Mutex<RefCell<Executive>>` inside `cortex_m::interrupt::free` closures.

3. **Option-based errors, not Result.** `add_job` returns `Option<usize>`.
   On overflow, `alarm::raise(AlarmCode::ExecutiveOverflow)` is called and
   `None` is returned. The caller decides whether to propagate or absorb the
   error.

4. **Alarm 1202 on overflow.** When all 7 slots are occupied and a new job
   is requested, alarm `01202` (`ExecutiveOverflow`) is raised before
   returning `None`. Reference: EXECUTIVE.agc page 1212 `TC BAILOUT / OCT 1202`.

5. **Priority 0 = free.** A slot with `priority == 0` and `entry == None`
   is considered vacant. This is the only representation of a free slot.
   The invariant must hold after every `finish_job` call.

6. **ISR safety.** `add_job` and `wake_job` may be called from T3RUPT context.
   They must be called within `interrupt::free`. They must not block or
   spin-wait.

7. **No blocking in jobs.** Job functions (type `JobFn`) must not block.
   If a job needs to wait for I/O, it calls `sleep_current` and returns;
   a Waitlist task later calls `wake_job`.

8. **DUMMYJOB idle.** When `run_next` returns `None`, the caller must enter
   the idle polling loop (equivalent to AGC DUMMYJOB) rather than panic.

---

## Test Cases

### Test 1 — Single Job Dispatch

**Setup:** Fresh `Executive`. Call `add_job(priority: 1000, entry: my_fn)`.

**Expected:**
- `add_job` returns `Some(0)` (first free slot).
- `find_highest_priority()` returns `Some(0)`.
- `run_next()` returns `Some(my_fn)`.
- After the job function runs and calls `finish_job()`, `current_job()` returns `None`.

### Test 2 — Priority Ordering (Three Jobs)

**Setup:** Add three jobs with priorities 100, 300, 200 in that order.

**Expected:**
- `find_highest_priority()` returns the slot with priority 300.
- `run_next()` dispatches priority-300 job first.
- After `finish_job()`, `run_next()` dispatches priority-200 job.
- After `finish_job()`, `run_next()` dispatches priority-100 job.
- After `finish_job()`, `run_next()` returns `None`.

### Test 3 — Table Overflow Raises Alarm 1202

**Setup:** Add 7 jobs (filling all slots). Attempt to add an 8th job.

**Expected:**
- `add_job` for the 8th call returns `None`.
- `alarm::last_raised()` returns `Some(AlarmCode::ExecutiveOverflow)`.
- The existing 7 slots are unaffected.

---

## agc-sim Impact

- `MissionState` struct: add field `active_job_count: usize` (count of non-free slots).
- `SimLog`: emit a `.warn("EXEC OVERFLOW: alarm 1202")` log entry whenever `ExecutiveOverflow` is raised.
- No new DSKY display state required (the Executive does not drive the DSKY directly).
- `dsky_terminal.rs`: the green "computer activity" light (channel 11, bit 2 in the AGC) should track whether `run_next` returned `None` (idle) vs. a real job. Expose `Executive::is_idle() -> bool`.
