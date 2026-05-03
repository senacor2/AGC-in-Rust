# Specification: `executive/` Module — Executive, Waitlist, and Restart Protection

**Status**: Approved for implementation
**Module path**: `agc-core/src/executive/`
**Source files**: `mod.rs`, `job.rs`, `scheduler.rs`, `waitlist.rs`, `restart.rs`
**Architecture reference**: `docs/architecture.md` §5 "Executive and Waitlist", §6 "Restart Protection", §12 "Error Handling and Alarms"
**Types reference**: `specs/types-module-spec.md`
**HAL reference**: `specs/hal-spec.md` — `pet_watchdog()`, `hardware_restart()`, `Timers::arm_t3`
**AGC source reference**: `docs/AGC Symbolic Listing.md`
  - §IID: Counter cells NEWJOB (octal 0061), TIME3 (octal 0026)
  - §VIIA: Waitlist System for Tasks
  - §VIIB: Executive System for Jobs
  - §VIIC: Mechanization of Restart Capability
  - §IIH: Program interrupt #3 (T3RUPT, night-watchman)
  - Notation §14 (task scheduling notation), §15 (job scheduling notation)
**Spec checklist**: `specs/README.md` — all items satisfied (see §10)

---

## 1. Purpose and Scope

The `executive/` module is the core of the AGC's "operating system". It consists
of three cooperating subsystems:

1. **Executive** (`scheduler.rs`, `job.rs`): A cooperative, priority-based job
   scheduler. Long computations ("jobs") are registered with a priority and
   dispatched in priority order. This is the AGC's EXEC / CORESET mechanism,
   described in `docs/AGC Symbolic Listing.md` §VIIB.

2. **Waitlist** (`waitlist.rs`): A time-triggered task scheduler. Short,
   time-critical computations ("tasks") are inserted with a centisecond delay and
   fired by the T3RUPT interrupt when TIME3 overflows. Described in
   `docs/AGC Symbolic Listing.md` §VIIA.

3. **Restart Protection** (`restart.rs`): Phase registers that record computation
   progress so that a hardware restart can resume each restart group from a safe
   known state. Described in `docs/AGC Symbolic Listing.md` §VIIC.

These three subsystems together provide: (a) concurrent execution of multiple
long computations, (b) hard-deadline periodic actions, and (c) robustness against
hardware restarts.

### What this module is NOT

- It is not a pre-emptive scheduler. The Executive dispatches exactly one job at
  a time. Preemption occurs only between job invocations (when a higher-priority
  job is created while one is running), never mid-function.
- It is not a thread library. There is no stack switching, no heap, and no
  context save/restore beyond what the hardware interrupt mechanism provides.
- There is no dynamic VAC area pool. The original AGC's VAC (vector accumulator)
  areas were scratchpad blocks used by the interpretive language, which is
  eliminated in this port (ADR-001, `docs/architecture.md` §9). The
  NOVAC/FINDVAC distinction is collapsed to a single `create_job` entry point;
  alarm 1210 (no free VAC areas) cannot be generated and the corresponding
  alarm constant is retained in `tables/alarm_codes.rs` for reference only.

---

## 2. AGC Background

### 2.1 Jobs and the CORE SET Table (§VIIB)

A **job** in the AGC is any computation that is not a task. Jobs are stored in
the CORE SET table — a fixed array of job register sets. Each register set holds
the priority of the job and the address of its entry point. The original AGC
CORE SET table holds 7 entries.

The AGC's "EXEC" routine scans the CORE SET table for the highest-priority
ready job and dispatches it. The job runs until it completes (falls off the end
of its routine) or explicitly requests rescheduling (CHANG1 to change priority).
After a job completes, EXEC scans again. This is a strict, non-pre-emptive,
cooperative multitasking discipline.

New jobs are created by calling NOVAC (no VAC area needed) or FINDVAC (allocates
a VAC area from a free list). In this port both collapse to a single
`create_job` entry point, since no interpreter and therefore no VAC pool exist.

**NEWJOB** (erasable cell octal 0061): Reading this cell resets the hardware
night-watchman flip-flop. The Executive must sample NEWJOB on every loop
iteration. The `pet_watchdog()` HAL call implements this.

> AGC source: `docs/AGC Symbolic Listing.md` §IID, cell 0061:
> "Each time it is sampled, a flip-flop set by a signal with a 1.28-second
> period is reset. If the flip-flop is set when another 1.28-second period
> signal (0.64 out of phase with the first) occurs, a 'night watchman' fault
> [...] causing a hardware restart, is produced. Hence maximum allowable
> interval between samples ranges from 0.64 to 1.92 second."

### 2.2 Tasks and the Waitlist (§VIIA)

A **task** is a short sequence of computations triggered by a time criterion.
Tasks are inserted into the Waitlist with a delay in centiseconds. The notation
used in the symbolic listing is:

> "Call 'XXXX' in yy seconds"

The Waitlist keeps entries sorted by execution time, stored as a delta-time chain:
each entry holds the additional centiseconds to wait after the previous entry
fires. The first entry's delta is loaded into TIME3 as `(2^14 - delta_cs)`.
When TIME3 overflows, T3RUPT fires the earliest task, dequeues it, and reloads
TIME3 with the next entry's delta.

The original AGC Waitlist supports 8 concurrent pending tasks. Tasks have strict
constraints:
- Must run to completion without yielding.
- Must not block or perform long computation.
- May schedule a follow-on task (call WAITLIST again).
- May establish a new job (call NOVAC/FINDVAC → `create_job`).
- May write output channels directly.

For delays exceeding 16383 centiseconds (the maximum TIME3 load), the **long
waitlist** mechanism chains two tasks: the first task schedules the second with
the remaining time. In this port, callers exceeding 16383 cs must split their
delay themselves using this chaining pattern.

> AGC source: `docs/AGC Symbolic Listing.md` §IID, cell 0026 (TIME3):
> "Preset to appropriate value under program control (i.e. 2^14 - required delay
> in centi-seconds), and incremented by +1 each 0.01 second."

### 2.3 Restart Groups and Phase Tables (§VIIC)

After a hardware restart (parity error, night-watchman timeout, power transient,
or software-initiated GOJAM), the RESTART routine reads phase registers and
re-dispatches each active restart group. There are 6 restart groups, each with a
single phase register.

**Phase semantics**:
- `0` (IDLE): group is inactive — no restart action needed.
- Positive odd: re-dispatch the group as a Waitlist **task** at the indicated
  phase entry point.
- Positive even: re-dispatch the group as an Executive **job** at the indicated
  phase entry point.
- Negative: restart the group from the **top** of the current phase (used when
  the group is mid-update and the partial result is unsafe).

**Restart-safe coding pattern** (AGC PHASCHNG): Before a multi-step computation
that modifies shared state, the routine sets its group's phase register to a
safe restart point. After completing each step, it advances the phase. On
completion, it sets the phase back to IDLE. On restart, the phase register
directs re-dispatch to the appropriate step.

> AGC source: `docs/AGC Symbolic Listing.md` §VIIC:
> "TC PHASCHNG ; set phase for group N to value P"

---

## 3. Data Structures

### 3.1 JobPriority

```rust
/// Job priority. 0 = slot empty / idle. Higher value = higher priority.
/// The original AGC used octal values such as 37 for the autopilot job.
pub type JobPriority = u8;
```

**Invariants**:
- Priority 0 is reserved for the "empty slot" sentinel. The idle loop (P00
  dummy job) is not entered into the job table; it is the implicit behavior of
  the Executive when no jobs are ready.
- Two jobs may have the same priority. In that case, scanning order determines
  which runs first; the implementation iterates the table from index 0.
- Priority 255 is the maximum; it is reserved for the most critical jobs (e.g.,
  digital autopilot background). No job may be created with priority 0.

### 3.2 JobEntry

**File**: `agc-core/src/executive/job.rs`

```rust
/// Maximum number of concurrent executive jobs (matches the AGC CORE SET table).
pub const MAX_JOBS: usize = 7;

/// A single entry in the Executive job table.
#[derive(Clone, Copy)]
pub struct JobEntry {
    /// Priority of this job. 0 means the slot is empty.
    pub priority: JobPriority,
    /// The function that implements this job's computation.
    /// The job runs until it returns; preemption happens between invocations.
    pub entry: fn(&mut crate::AgcState),
    /// Major mode (program number) that created this job.
    /// Used by the restart mechanism to re-dispatch after a power-on restart.
    pub major_mode: u8,
}
```

The original AGC distinguished NOVAC vs FINDVAC jobs via a per-slot flag that
selected whether a VAC scratchpad was allocated for the interpretive language.
The interpreter is eliminated (ADR-001), so no VAC pool exists and the flag
has been removed.

**Invariants**:
- A slot is empty if and only if `priority == 0`.
- The `entry` function pointer in an empty slot is unspecified. Do not call it.
- `JobEntry::EMPTY` provides the canonical empty slot value with a no-op entry.

### 3.3 Executive

**File**: `agc-core/src/executive/scheduler.rs`

```rust
pub struct Executive {
    jobs: [JobEntry; MAX_JOBS],
    current_priority: JobPriority,
}
```

**Invariants**:
- `jobs` is a flat array; there is no separate free list. An empty slot has
  `priority == 0`.
- `current_priority` holds the priority of the currently executing job.
  Between job invocations (inside the `run` loop but not inside a job function),
  `current_priority` is 0.
- The `run` method never returns (return type `!`).

### 3.4 WaitlistEntry

**File**: `agc-core/src/executive/waitlist.rs`

```rust
pub const MAX_WAITLIST_TASKS: usize = 8;

#[derive(Clone, Copy)]
pub struct WaitlistEntry {
    /// Centiseconds until this task fires, measured as a delta from the
    /// previous entry in the sorted list (or from "now" for the first entry).
    pub delta_time: u16,
    /// The task function to run when the timer expires.
    pub task: fn(&mut crate::AgcState),
}
```

**Invariants**:
- The Waitlist is always kept in ascending order of absolute fire time.
  `delta_time` is always relative to the previous entry, NOT to "now". The
  absolute fire time of entry `i` is `sum(delta_time[0..=i])`.
- `delta_time` for any entry is at most 16383 (the maximum TIME3 load value).
  Callers needing longer delays must use the long-waitlist chaining pattern.
- The first entry's `delta_time` is the value loaded into TIME3 as
  `(2^14 - delta_time)`.

### 3.5 Waitlist

```rust
pub struct Waitlist {
    entries: [Option<WaitlistEntry>; MAX_WAITLIST_TASKS],
    count: usize,
}
```

**Invariants**:
- `count` is the number of `Some` entries.
- Entries are packed into indices `0..count`; there are no gaps.
- `entries[0]` is always the soonest task; `entries[count-1]` is the latest.

### 3.6 Phase

**File**: `agc-core/src/executive/restart.rs`

```rust
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Phase(pub i16);
```

| Value | Semantic | Restart action |
|-------|----------|----------------|
| `0` | IDLE | No restart needed |
| `+1, +3, +5, ...` (odd) | Active task | Re-schedule as Waitlist task |
| `+2, +4, +6, ...` (even) | Active job | Re-create as Executive job |
| `-1, -2, ...` (negative) | Mid-update | Restart group from top of phase |

### 3.7 RestartProtection

```rust
pub const NUM_RESTART_GROUPS: usize = 6;

pub struct RestartProtection {
    pub phases: [Phase; NUM_RESTART_GROUPS],
}
```

**Invariants**:
- Indexed by group constants `GROUP_1` (0) through `GROUP_6` (5).
- Every `phases[i]` is independently managed by the subsystem owning group `i`.
- On FRESH START, all phases are initialized to `Phase::IDLE`.
- On RESTART, all non-IDLE phases are read and acted upon.

---

## 4. Public API

### 4.1 `Executive::new`

```rust
pub const fn new() -> Self
```

**Preconditions**: None. Safe to call in `const` context.

**Postconditions**:
- All 7 job slots are `JobEntry::EMPTY`.
- `current_priority` is 0.

---

### 4.2 `Executive::create_job`

```rust
pub fn create_job(
    &mut self,
    priority: JobPriority,
    entry: fn(&mut AgcState),
    major_mode: u8,
) -> bool
```

Maps to both AGC NOVAC and FINDVAC. Since the interpretive language is
eliminated and no VAC pool exists, the original NOVAC/FINDVAC distinction
collapses to a single entry point. Alarm 1210 (no free VAC areas) cannot be
generated; only alarm 1202 applies on table-full failure.

**Preconditions**:
- `priority > 0`: priority 0 is the empty-slot sentinel and is illegal.
- `entry` must be a function that, when called, runs to completion and does not
  call `Executive::run` recursively.
- May be called from interrupt context (T3RUPT task establishing a job) or from
  a running job.

**Postconditions (success, returns `true`)**:
- One previously-empty slot now holds `JobEntry { priority, entry, major_mode }`.
- The new job will be dispatched on the next Executive scan iteration in which
  its priority is the highest.

**Postconditions (failure, returns `false`)**:
- All 7 slots were occupied. No slot is modified.
- The caller is responsible for raising alarm 1202 (§6.1). Then call
  `drop_lowest_job` if recovery is desired.

**Side effects**:
- None beyond the slot assignment. Does not immediately preempt a running job.

**Error conditions**:
- Returns `false` when all slots are full (alarm 1202 condition).

**Test cases**:

| # | Initial state | Call | Expected result |
|---|---------------|------|-----------------|
| TC-CJ-1 | All 7 slots empty | `create_job(20, f, 0)` | Returns `true`; slot 0 holds priority 20 |
| TC-CJ-2 | 6 slots occupied (priorities 1–6) | `create_job(7, f, 0)` | Returns `true`; one slot now holds priority 7 |
| TC-CJ-3 | All 7 slots occupied | `create_job(5, f, 0)` | Returns `false`; table unchanged; alarm 1202 must be raised by caller |
| TC-CJ-5 | Slot 0 occupied (priority 1), others empty | `create_job(1, g, 0)` | Returns `true`; slot 1 holds the new job (first empty slot is chosen) |
| TC-CJ-6 | All slots empty | `create_job(0, f, 0)` | Undefined behavior (priority 0 is reserved); spec requires `priority > 0` |

---

### 4.3 `Executive::drop_lowest_job`

```rust
pub fn drop_lowest_job(&mut self)
```

Called as part of alarm 1202 recovery to create a free slot by evicting the
lowest-priority non-critical job.

**Preconditions**:
- At least one slot has `priority > 0`. (If all slots are empty, this is a no-op.)

**Postconditions**:
- The slot with the minimum `priority` value among all occupied slots is set to
  `JobEntry::EMPTY`.
- If two slots have equal minimum priority, the one at the lower index is dropped.

**Error conditions**: None. Safe to call even if the table is empty.

**Test cases**:

| # | Initial state | Expected result |
|---|---------------|-----------------|
| TC-DL-1 | Priorities [10, 20, 5, 30, 15, 0, 0] | Slot 2 (priority 5) cleared; result [10, 20, 0, 30, 15, 0, 0] |
| TC-DL-2 | All slots empty | No-op; table unchanged |
| TC-DL-3 | Priorities [10, 10, 10, 0, 0, 0, 0] | Slot 0 (tie broken by lowest index) cleared |

---

### 4.4 `Executive::run`

```rust
pub fn run(&mut self, state: &mut AgcState, hw: &mut impl AgcHardware) -> !
```

The main scheduling loop. Never returns in normal operation.

**Preconditions**:
- Called exactly once, at system startup, after FRESH START or RESTART
  initialization completes.
- `hw` must be a fully initialized `AgcHardware` implementation.
- `state` must hold a valid `AgcState` (post-FRESH-START or post-RESTART).

**Loop invariant** (executed every iteration):
1. `hw.pet_watchdog()` is called to reset the night-watchman. This must happen
   within every 0.64–1.92 s window regardless of whether a job is ready.
2. `find_highest_priority_job()` scans the table for the highest-priority
   occupied slot.
3. If found: the job's `entry` function is called with `&mut state`. On return,
   the slot is cleared.
4. If not found: the loop spins without dispatching (this is the idle / P00
   state — "COMPUTER ACTIVITY" lamp is off).

**NEWJOB / preemption semantics**:
In the original AGC, a "NEWJOB" signal was set by interrupt code when a
higher-priority job was created. The Executive would then re-scan immediately
rather than continuing the current job. In this port, true preemption mid-function
is not implemented. Instead, preemption occurs between job invocations: when
the current job's `entry` function returns, `run` re-scans and dispatches the
new higher-priority job before re-running the previous one if it has re-registered
itself. This is sufficient because all jobs are short, cooperative functions that
return regularly. The `current_priority` field is exposed for jobs that wish to
implement their own yielding logic.

**Night-watchman contract**:
- `pet_watchdog()` is called unconditionally on every loop iteration, before the
  job scan. This ensures the watchdog is fed even when the idle loop is spinning
  with no jobs.
- Maximum safe call interval: 0.64–1.92 s (see `docs/AGC Symbolic Listing.md`
  §IID cell 0061, and `specs/hal-spec.md` §4.3).
- If a job function runs for longer than the watchdog window without returning,
  `pet_watchdog()` will not be called and the hardware will trigger a restart.
  Job implementations must therefore complete or yield within the watchdog budget.

**Postconditions**: This function does not return. The only exit is a hardware
restart triggered by `hw.hardware_restart()` in alarm handlers.

**Test cases**:

| # | Initial state | Scenario | Expected behavior |
|---|---------------|----------|-------------------|
| TC-RUN-1 | No jobs registered | `run` called | Loop spins; `pet_watchdog` called repeatedly; no job dispatched |
| TC-RUN-2 | One job (priority 10) | `run` called | Job dispatched; slot cleared after job returns; loop continues idle |
| TC-RUN-3 | Jobs at priorities [5, 20, 15] | `run` called | Priority-20 job dispatched first; on return, priority-15 next; then priority-5 |
| TC-RUN-4 | Job A (priority 10) running; interrupt creates job B (priority 30) | Job A's function returns normally | After A's slot is cleared, B is dispatched next (priority 30 > 10); preemption occurs between invocations |
| TC-RUN-5 | No jobs; `pet_watchdog` mock tracks call count | `run` iterates 1000 times | `pet_watchdog` called exactly 1000 times |

---

### 4.5 `Waitlist::new`

```rust
pub const fn new() -> Self
```

**Postconditions**: All 8 entries are `None`; `count` is 0.

---

### 4.6 `Waitlist::schedule`

```rust
pub fn schedule(&mut self, centiseconds: u16, task: fn(&mut AgcState)) -> bool
```

Insert a new task to fire `centiseconds` from now.

**Preconditions**:
- `centiseconds > 0`: scheduling a task with zero delay is undefined behavior.
  The minimum meaningful delay is 1 cs (10 ms).
- `centiseconds <= 16383`: the maximum TIME3 load is `2^14 - 1 = 16383 cs`.
  For longer delays, the caller must chain two tasks using the long-waitlist
  pattern (see §5.2).
- `task` must be a function that runs to completion quickly and does not block.
- Must not be called from inside `task` itself (recursive scheduling is
  permitted for follow-on tasks, but must target a different entry point).

**Implementation — sorted insertion**:
1. If `count >= MAX_WAITLIST_TASKS`, return `false` (alarm 1211 condition).
2. Find the insertion position `k` such that the new task's absolute time is
   between `entries[k-1]` and `entries[k]`.
3. Adjust `entries[k].delta_time` by subtracting `centiseconds` (the new
   task takes some of the existing delta).
4. Insert `WaitlistEntry { delta_time: adjusted_delta, task }` at position `k`.
5. Shift entries `[k+1..count]` one position right.
6. Increment `count`.
7. If `k == 0` (new task fires before all existing tasks), reload TIME3:
   call `hw.timers().arm_t3(centiseconds)`.

**Note on TIME3 reload**: The `schedule` function signature as specified above
does not take `hw` as a parameter. For cases where `k == 0`, the TIME3 reload
must be performed by the caller after `schedule` returns `true`, using the new
`centiseconds` value. Alternatively, an `arm_t3` signal flag can be returned.
The implementation must document which convention it uses. The recommended
approach is to have `schedule` return `Some(new_t3_load)` when a TIME3 reload
is needed, or `None` otherwise (replacing the `bool` return type with
`Option<u16>`). The caller (T3RUPT handler or job creating a task) then calls
`hw.timers().arm_t3(new_t3_load)`.

**Implementation note — return type adjustment**: The `bool` return type shown
in the current skeleton is a simplification. The full specification requires:

```rust
pub fn schedule(
    &mut self,
    centiseconds: u16,
    task: fn(&mut AgcState),
) -> ScheduleResult
```

where:

```rust
pub enum ScheduleResult {
    /// Task inserted. Caller must reload TIME3 with this value.
    OkReloadT3(u16),
    /// Task inserted. TIME3 does not need reloading (new task is not earliest).
    Ok,
    /// Waitlist full. Alarm 1211 must be raised.
    Full,
}
```

**Postconditions (success)**:
- `count` increased by 1.
- The Waitlist remains sorted (delta-chain invariant preserved).
- If the new task is the earliest, the caller has been notified to reload TIME3.

**Postconditions (failure)**:
- `count` unchanged; table unchanged; returns `ScheduleResult::Full`.

**Error conditions**:
- Returns `ScheduleResult::Full` when all 8 slots are occupied (alarm 1211).

**Test cases**:

| # | Initial Waitlist | Call | Expected |
|---|-----------------|------|----------|
| TC-SC-1 | Empty | `schedule(100, f)` | count=1; entries[0]={delta=100, task=f}; returns `OkReloadT3(100)` |
| TC-SC-2 | [{delta=200, task=f}] | `schedule(100, g)` | count=2; entries[0]={delta=100, g}, entries[1]={delta=100, f}; returns `OkReloadT3(100)` (new earliest) |
| TC-SC-3 | [{delta=50, task=f}] | `schedule(100, g)` | count=2; entries[0]={delta=50, f}, entries[1]={delta=50, g}; returns `Ok` (g fires at 50+50=100, after f at 50) |
| TC-SC-4 | 8 entries occupied | `schedule(10, h)` | returns `Full`; count unchanged |
| TC-SC-5 | [{delta=100, f}, {delta=100, g}] | `schedule(150, h)` | entries[0]={100,f}, [1]={50,h} (150-100=50), [2]={50,g} (200-150=50); returns `Ok` |

**Clarifying TC-SC-5**:
- `f` fires at absolute time 100 cs.
- `g` fires at absolute time 200 cs.
- `h` fires at absolute time 150 cs.
- Insertion order: `h` goes between `f` and `g`.
- Result: `[{100, f}, {50, h}, {50, g}]` (50+50=100 = delta from h to g).
- `h` is not the earliest task, so no T3 reload: returns `Ok`.

---

### 4.7 `Waitlist::dispatch`

```rust
pub fn dispatch(&mut self, state: &mut AgcState) -> Option<u16>
```

Called by the T3RUPT interrupt handler. Fires the earliest task and returns the
delta time to the next task (for reloading TIME3), or `None` if the Waitlist is
now empty.

**Preconditions**:
- Called from T3RUPT context (interrupts inhibited during dispatch).
- `count > 0`: if the Waitlist is empty when T3RUPT fires, this is an error
  condition (stale timer). The function returns `None` in this case.

**Implementation**:
1. If `count == 0`, return `None`.
2. Extract `entries[0]`.
3. Shift `entries[1..count]` one position left.
4. Decrement `count`.
5. Call `(entries[0].task)(state)`.
6. If `count > 0`, return `Some(entries[0].delta_time)` (now the new first entry).
7. If `count == 0`, return `None`.

**Note**: The T3RUPT handler reloads TIME3 using the returned value: if
`Some(delta)`, load `2^14 - delta` into TIME3 via `hw.timers().arm_t3(delta)`.
If `None`, TIME3 is not reloaded (no pending tasks).

**Postconditions**:
- The earliest task has been called and its slot is removed.
- Remaining entries are shifted forward; the delta-chain invariant is preserved.
- The returned value is the `delta_time` of the new first entry (if any).

**Test cases**:

| # | Before dispatch | After dispatch | Returned |
|---|----------------|---------------|----------|
| TC-DS-1 | [{50, f}] | [] count=0 | `None` (f called; list empty) |
| TC-DS-2 | [{50, f}, {100, g}] | [{100, g}] count=1 | `Some(100)` (f called; g is now first) |
| TC-DS-3 | Empty (count=0) | Unchanged | `None` (no-op; stale timer) |

---

### 4.8 `RestartProtection::new`

```rust
pub const fn new() -> Self
```

**Postconditions**: All 6 phase registers are `Phase::IDLE`.

---

### 4.9 `RestartProtection::set_phase`

```rust
pub fn set_phase(&mut self, group: usize, phase: Phase)
```

Record that `group` has entered phase `phase`.

**Preconditions**:
- `group < NUM_RESTART_GROUPS` (i.e., `group < 6`). Panics on out-of-bounds
  in debug; undefined behavior in release (prefer `debug_assert!`).
- `phase` must be a value meaningful to the group's restart handler — the
  restart routine will dispatch to the appropriate entry point based on this
  value.

**Postconditions**: `self.phases[group] == phase`.

**Side effects**: None beyond the array write. This is an in-place, atomic
(single-word) write on 32-bit targets; no further synchronization is needed
assuming cooperative scheduling.

---

### 4.10 `RestartProtection::phase`

```rust
pub fn phase(&self, group: usize) -> Phase
```

Read the phase for `group`.

**Preconditions**: `group < NUM_RESTART_GROUPS`.

**Postconditions**: Returns `self.phases[group]`.

---

## 5. Subsystem Behaviors

### 5.1 Executive Scheduling Loop

```
loop:
  pet_watchdog()           ← reset night-watchman every iteration
  if any job.priority > 0:
    i ← index of max(job.priority)
    current_priority ← jobs[i].priority
    jobs[i].entry(state)   ← run job to completion
    jobs[i] ← EMPTY        ← clear slot
    current_priority ← 0
  else:
    (idle / P00 — no COMPUTER ACTIVITY)
```

**Priority tie-breaking**: When two jobs have equal priority, the one at the
lower index in `jobs[]` is dispatched first. This is implementation-defined and
must not be relied upon by flight software; jobs of equal priority should be
treated as unordered.

**NEWJOB / preemption between jobs**: If a T3RUPT task calls `create_job` with
a higher priority than the currently running job, that new job will be dispatched
on the very next scan iteration, before any other equal-or-lower-priority jobs.
This is the AGC NEWJOB mechanism, implemented here at the job-boundary level
rather than mid-instruction.

### 5.2 Long Waitlist (Delays > 16383 cs)

TIME3 can express at most `16383` centiseconds (163.83 seconds, about 2.73
minutes). For longer delays:

```rust
// Caller wants to fire g() in 20000 cs (200 seconds).
// Split: fire a relay task in 16000 cs that reschedules g for the remaining 4000 cs.
fn relay_to_g(state: &mut AgcState) {
    state.waitlist.schedule(4000, g);
    // arm_t3 must be called if this is now the earliest task
}
// Then:
state.waitlist.schedule(16000, relay_to_g);
```

This chaining pattern is the direct equivalent of the AGC long-waitlist
mechanism. The relay task must be a named function, not a closure (no heap,
no closures in `no_std` without alloc).

### 5.3 T3RUPT Handler Integration

The T3RUPT handler is defined in `services/` (or `hal/interrupts.rs` for the
bare-metal target). It must:

1. Save context (handled by NVIC hardware on Cortex-M).
2. Call `state.waitlist.dispatch(&mut state)`.
3. If `Some(delta)` returned: call `hw.timers().arm_t3(delta)`.
4. If `None` returned: do not reload TIME3 (no pending tasks).
5. Restore context and return from interrupt.

**Constraint**: The T3RUPT handler must complete quickly (< 1 ms typical budget).
Long computations inside a task violate this contract and will cause T4RUPT and
T5RUPT timing jitter.

### 5.4 Restart Sequence

When `hw.hardware_restart()` is called (or when the hardware itself generates a
restart), the RESTART routine in `services/fresh_start.rs` executes:

1. Preserve navigation state (CSM/target state vectors, REFSMMAT, time) — these
   are in `AgcState` fields that survive across restarts by convention (they must
   not be zeroed in RESTART as they would in FRESH START).
2. Clear all Executive job slots.
3. Clear the Waitlist.
4. For each restart group `i` in `GROUP_1..=GROUP_6`:
   a. Read `state.restart.phases[i]`.
   b. If `Phase::IDLE`: do nothing.
   c. If positive even: call `state.executive.create_job(default_priority, group_entry_fn[i], major_mode)`.
   d. If positive odd: call `state.waitlist.schedule(default_delay, group_task_fn[i])`.
   e. If negative: call the group's restart-from-top entry point directly or
      schedule it with the negative phase as context.
5. Call `Executive::run`.

**FRESH START vs RESTART**:

| Attribute | FRESH START | RESTART |
|-----------|-------------|---------|
| Navigation state | Zeroed | Preserved |
| Phase registers | All IDLE | Read and acted upon |
| Job table | Cleared | Cleared then re-populated from phases |
| Waitlist | Cleared | Cleared then re-populated from phases |
| Major mode | Set to P00 | Restored from `major_mode` field |
| Alarm state | Cleared | `restart_flag` set on DSKY |
| DSKY display | Reset | RESTART indicator lit |

---

## 6. Alarm Codes

### 6.1 Alarm 1202 — Executive Overflow

**Trigger**: `create_job` returns `false` (all 7 CORE SET slots are occupied).

**Meaning**: The Executive job table is full; a new job cannot be created. This
is the famous "1202 Program Alarm" from Apollo 11, caused by the rendezvous
radar unexpectedly consuming Executive slots.

**Required response** (implemented by the alarm handler in `services/alarm.rs`):
1. Set `state.alarm.code = 0x1202` and `state.alarm.lit = true`.
2. Write alarm code to DSKY PROG display.
3. Call `state.executive.drop_lowest_job()` to free one slot.
4. Re-attempt `create_job`. If still fails, raise alarm again (pathological
   overload condition).
5. Do NOT restart. This alarm is advisory: the crew and ground see it but
   the computer continues.

**Historical note**: During Apollo 11, ground controllers recognized the 1202
alarm as non-critical because the computer was completing its cycle before the
next alarm. The correct design is to shed load and continue. This port preserves
that exact behavior.

### 6.2 Alarm 1210 — No Free VAC Areas (not used)

In the original AGC, FINDVAC raised this alarm when no free VAC scratchpad
was available for the interpretive language. The interpreter is eliminated
in this port (ADR-001), so no VAC pool exists and alarm 1210 cannot be
generated. The constant is retained in `tables/alarm_codes.rs` for reference
only.

### 6.3 Alarm 1211 — Waitlist Overflow

**Trigger**: `Waitlist::schedule` returns `ScheduleResult::Full` (all 8 slots
are occupied).

**Required response**:
1. Set `state.alarm.code = 0x1211` and `state.alarm.lit = true`.
2. Write alarm code to DSKY PROG display.
3. The new task is **dropped** — it is not inserted.
4. Do NOT restart. This is advisory.

**Test cases**:

| # | Scenario | Expected |
|---|----------|----------|
| TC-AL-1 | 7 jobs in table; `create_job` called | Returns `false`; alarm 1202 raised by caller; `drop_lowest_job` creates room |
| TC-AL-3 | After 1202 recovery via `drop_lowest_job`; `create_job` re-attempted | Returns `true`; alarm indicator remains lit (crew must acknowledge) |
| TC-AL-4 | 8 tasks in Waitlist; `schedule` called | Returns `Full`; task dropped; alarm 1211 raised |

---

## 7. HAL Interactions

### 7.1 `pet_watchdog()`

**Caller**: `Executive::run`, unconditionally on every loop iteration.

**Contract** (from `specs/hal-spec.md` §4.3):
- Resets the hardware night-watchman flip-flop.
- Must be called within a 0.64–1.92 s window.
- No return value; no error.
- Call frequency: once per Executive loop iteration.

**Failure mode**: If not called within the window, the hardware generates a
restart. This is the intended behavior for detecting stalled programs (infinite
loops in jobs, deadlocked interrupt handlers, etc.).

### 7.2 `hardware_restart() -> !`

**Caller**: Alarm handler (`services/alarm.rs`) for unrecoverable conditions;
panic handler.

**Contract**: Triggers an immediate hardware restart. Does not return.
Equivalent to AGC GOJAM.

**When called by Executive module**: The Executive itself does not call
`hardware_restart()` directly. It only calls `pet_watchdog()`. The restart
mechanism for unrecoverable alarm conditions is owned by `services/alarm.rs`.

### 7.3 `Timers::arm_t3(centiseconds: u16)`

**Caller**: T3RUPT handler (after `Waitlist::dispatch` returns `Some(delta)`),
and the first scheduler of a task into an empty Waitlist.

**Contract** (from `specs/hal-spec.md`, Timers sub-trait):
- Loads TIME3 with `(2^14 - centiseconds)`, which will overflow and trigger
  T3RUPT in exactly `centiseconds * 10 ms`.
- Precondition: `centiseconds <= 16383`.
- Called with interrupts inhibited (INHINT) when reloading from T3RUPT handler.

---

## 8. Restart-Safe Coding Pattern

Any subsystem that performs a multi-step computation across job invocations or
across task firings must protect its state against restart using this pattern:

```rust
fn my_restartable_job(state: &mut AgcState) {
    // Step 1: declare intent (can restart from step 1 if we restart here)
    // Phase value 2 = positive even = job (re-dispatch as Executive job)
    state.restart.set_phase(GROUP_3, Phase::new(2));

    // ... perform first step of computation ...

    // Step 2: advance phase (can restart from step 2)
    state.restart.set_phase(GROUP_3, Phase::new(4));

    // ... perform second step ...

    // Step 3: computation complete; clear phase
    state.restart.set_phase(GROUP_3, Phase::IDLE);
}
```

**Rules**:
1. Always call `set_phase` to a non-IDLE value **before** beginning any step
   that modifies shared state.
2. Always call `set_phase(IDLE)` after the computation is fully committed.
3. Choose odd phase values for groups that should be re-dispatched as tasks;
   even values for groups that should be re-dispatched as jobs.
4. Use negative phase values to indicate "restart from the very beginning of
   this phase" — this is for cases where a partial update is worse than no
   update (e.g., half-written state vector).
5. The phase value itself encodes which re-entry point to use. The restart
   handler dispatches to the correct entry point using a match on the phase value.

**Test cases**:

| # | Scenario | Expected |
|---|----------|----------|
| TC-RP-1 | Phase set to `Phase::new(2)` (positive even) | `phase.is_job() == true`; restart re-creates as Executive job |
| TC-RP-2 | Phase set to `Phase::new(3)` (positive odd) | `phase.is_task() == true`; restart re-schedules as Waitlist task |
| TC-RP-3 | Phase set to `Phase::new(-1)` (negative) | `phase.is_idle() == false`, `is_job() == false`, `is_task() == false`; restart handler re-runs group from top |
| TC-RP-4 | Phase set to IDLE | `phase.is_idle() == true`; restart does nothing for this group |
| TC-RP-5 | Restart occurs with GROUP_2 phase=4, GROUP_5 phase=0 | GROUP_2 job re-created; GROUP_5 untouched |

---

## 9. Module Structure and Re-exports

**File**: `agc-core/src/executive/mod.rs`

```rust
pub mod job;
pub mod restart;
pub mod scheduler;
pub mod waitlist;

pub use job::{JobEntry, JobPriority, MAX_JOBS};
pub use restart::{
    Phase, RestartProtection,
    GROUP_1, GROUP_2, GROUP_3, GROUP_4, GROUP_5, GROUP_6,
    NUM_RESTART_GROUPS,
};
pub use scheduler::Executive;
pub use waitlist::{ScheduleResult, Waitlist, WaitlistEntry, MAX_WAITLIST_TASKS};
```

All other modules in `agc-core` import from `crate::executive::*` or from
`crate::AgcState` fields. No module may import from `executive::scheduler`,
`executive::job`, etc. directly; always use the re-exported path.

---

## 10. Spec Quality Checklist

- [x] AGC source file and section referenced (§IID NEWJOB/TIME3, §VIIA, §VIIB, §VIIC)
- [x] All erasable variables and their AGC addresses listed (NEWJOB=0061, TIME3=0026)
- [x] Scale factors documented for all fixed-point values (TIME3: `2^14 - cs`, delta_time in centiseconds)
- [x] Corresponding f64 SI units documented (N/A for scheduler — all time values are integer centiseconds; `u16` is the correct type)
- [x] Input/output preconditions and postconditions stated for every public method
- [x] Edge cases and error handling specified (empty table, full table, empty waitlist, stale T3RUPT)
- [x] At least 3 test cases with expected values per component (TC-CJ, TC-DL, TC-RUN, TC-SC, TC-DS, TC-RP, TC-AL)
- [x] Rust API signatures designed (types, ownership — all `&mut self` or `&self`, no lifetimes needed, no heap)
- [x] Invariants explicitly stated (priority 0 = empty, delta-chain sorted, phase semantics)
- [x] Consistency with `docs/architecture.md` checked (§5, §6, §12; types match §3.1; HAL usage matches §4.1)
- [x] Consistency with `specs/hal-spec.md` checked (`pet_watchdog`, `hardware_restart`, `arm_t3`)
- [x] Consistency with `specs/types-module-spec.md` checked (no `AgcWord`, `u16` for time, `u8` for priority)
- [x] FRESH START vs RESTART distinction documented
- [x] Alarm codes (1202, 1211) documented with recovery procedures (1210 retained for reference but unreachable)
- [x] `no_std` compliance confirmed (no heap, no alloc, no closures, function pointers only)
- [x] Historical AGC context provided for 1202 alarm and NEWJOB mechanism
