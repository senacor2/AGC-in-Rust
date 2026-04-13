# Functional Specification: Executive Waitlist

## AGC Source Reference

```
AGC source: Comanche055/WAITLIST.agc
Pages:      1221–1235 (MIT hardcopy pagination)
Routines:   WAITLIST, TWIDDLE, WAIT2, WTLST4, WTLST5, WTLST2, T3RUPT,
            T3RUPT2, TASKOVER, FIXDELAY, VARDELAY, LONGCALL, LNGCALL2,
            LONGCYCL, LASTTIME, GETCADR, ENDTASK
Supporting: ERASABLE_ASSIGNMENTS.agc (LST1+0..+7, LST2+0..+17D,
            RUPTAGN, WAITEXIT, WAITADR, WAITTEMP, WAITBANK, TIME3)
            FRESH_START_AND_RESTART.agc (STARTSB2 — initialization)
```

## Behavior Summary

The Waitlist is the AGC's timer-driven task dispatcher. It provides a
way to schedule a function (a "task") to run after a specified number of
centiseconds. Tasks are short interrupt-level routines; long work must be
handed off to the Executive via `NOVAC`/`FINDVAC`.

The hardware timer `TIME3` fires T3RUPT approximately every 10 ms
(every 1 centisecond). T3RUPT is the sole mechanism that advances
the Waitlist chain and dispatches due tasks.

### Data Structures

The Waitlist is represented by two parallel arrays in AGC erasable memory:

**`LST1` (delta-time array, 8 words, `LST1+0` through `LST1+7`):**

Each word stores a packed negative delta-time, encoded as:
```
C(LST1+i) = -(T_{i+2} - T_{i+1}) + 1
```
where T_n is the absolute AGC time (in centiseconds) at which task n fires.
This is a **delta-time chain**: each entry is the time difference between
consecutive tasks, not absolute time. A value of `NEG1/2` (= −16384 in
ones-complement, equivalent to approximately 81.91 seconds) is the sentinel
used to fill unused slots; it corresponds to the `ENDTASK` slot.

The first task's firing time is encoded in `TIME3`:
```
C(TIME3) = 16384 - (T1 - T_now)     centiseconds
```
where T_now is the current AGC time and T1 is the fire time of the first task.

**`LST2` (2CADR array, 18 words, `LST2+0` through `LST2+17D`):**

Each pair of words holds a 2CADR (15-bit address + 9-bit bank selector) of
the corresponding task's entry function. Up to 9 tasks fit:
- `LST2+0`, `LST2+1` = 2CADR of task 1
- `LST2+2`, `LST2+3` = 2CADR of task 2
- …
- `LST2+16D`, `LST2+17D` = 2CADR of task 9

The 10th slot conceptually holds `ENDTASK` — a fixed-fixed address used as a
sentinel. At initialization, all `LST2` slots are filled with `ENDTASK`.

**Slot Count.** WAITLIST.agc (page 1221) explicitly states: "9 TASKS MAXIMUM".
`LST2 ERASE +17D` allocates 18 words = 9 two-word 2CADR slots.
`LST1 ERASE +7` allocates 8 delta-time words (intervals between 9 tasks).
The authoritative constant from `docs/agc-reference-constants.md` is
**`MAX_WAITLIST_TASKS = 9`**.

### WAITLIST Entry — Scheduling a Task

Calling sequence (AGC assembly):
```
L-1   CA    DELTAT          ; delay in centiseconds (1 ≤ DELTAT ≤ 16250)
L     TC    WAITLIST
L+1   2CADR DESIRED_TASK    ; 2-word inline task address
L+2   (minor half of 2CADR)
L+3   RELINT                ; return here
```

The `WAITLIST` routine (`WAITLIST.agc` page 1223):

1. Disables interrupts (`INHINT`).
2. Validates delta-T: if zero or negative, calls `POODOO` with alarm
   `OCT 1204` (waitlist call with zero or negative delta-T).
3. Reads the 2CADR from the two inline words following the `TC WAITLIST`.
4. Saves BBANK and jumps to `WAIT2` in a switched bank.

`WAIT2` (WAITLIST.agc pages 1226–1229):

1. Computes `TD - T1 + 1` where `TD` is the desired fire time
   and `T1` is the time of the head task (derived from `TIME3`).
2. Inserts the new task into the sorted delta-time chain:
   - If `TD < T1`: the new task fires before the current head. Update
     `TIME3` to reflect the new head, and push all existing tasks down.
   - Otherwise: walk the delta-time chain (`LST1+0` through `LST1+7`)
     comparing cumulative times until the correct insertion point is found,
     then shift entries down and insert.
3. Insertion is done by the `WTLST4` shuffle (pushing all 8 delta-time words
   and all 9 2CADR pairs through a chain of exchanges).
4. On overflow (no room for the 9th task): calls `BAILOUT` with
   `OCT 1203` (waitlist overflow). In Rust this raises alarm `1203`
   (`WaitlistOverflow`) and returns `None`.
5. Returns to caller at `L+3` (`RELINT`).

**Minimum granularity.** WAITLIST.agc page 1221 specifies:
`1 <= C(A) <= 16250D` — range is 1 to 16,250 centiseconds (0.01 s to 162.5 s).
Minimum task granularity is **1 centisecond**.

### T3RUPT — ISR Entry Point

T3RUPT fires approximately every 10 ms. The ISR entry (`T3RUPT`, WAITLIST.agc
page 1231):

1. Saves `BBANK` and `SUPERBNK` in `BANKRUPT`.
2. Saves `Q` in `QRUPT`.
3. Executes the `T3RUPT2` dispatch loop:
   a. Load `NEG1/2` into the accumulator.
   b. Shift `LST1+7` ← `LST1+6` ← … ← `LST1+0` ← `NEG1/2` (one position
      forward). This advances the chain by one step: the head task is popped
      off and each remaining task's position is moved up. `NEG1/2` is the
      sentinel for the empty tail slot.
   c. Add the popped delta-time to `TIME3`: `TIME3 += popped_delta + POSMAX`.
      This sets `TIME3` to fire at the next task's due time. If the result
      overflows, `RUPTAGN` is set to +1 to signal that another task is due
      immediately.
   d. Simultaneously shuffle `LST2+16D` ← `LST2+14D` ← … ← `LST2+0` ←
      `DCS ENDTASK` (pop head task's 2CADR, shift rest up, put ENDTASK at tail).
   e. Set BBANK/SUPERBNK from the popped 2CADR and `DTCB` (dispatch) to the
      task.

4. The task runs under interrupt-inhibit. It must be short.
5. Task ends with `TC TASKOVER`.

### TASKOVER — Task Return

`TASKOVER` (WAITLIST.agc page 1232):

1. Checks `RUPTAGN`. If `+1`, another task is due this T3RUPT (i.e., `TIME3`
   already overflowed again). Jump back to `T3RUPT2` to dispatch the next task.
2. If `RUPTAGN` is `+0`, no more tasks are due. Restore `BANKRUPT` and
   `QRUPT`, then execute `RESUME` to return from interrupt.

In Rust terms: `RUPTAGN` becomes a flag field in `Waitlist` that indicates
whether re-dispatch is needed within the same ISR invocation.

### LONGCALL

`LONGCALL` (WAITLIST.agc pages 1233–1235) extends the Waitlist for delta-times
exceeding 162.5 seconds. It chains multiple 81.91-second Waitlist slots until
the accumulated time reaches the target. This is not commonly used in flight
but must be present.

In the Rust port, `LONGCALL` is implemented as a helper that wraps repeated
calls to `schedule()` with intermediate relay tasks (`LONGCYCL`/`GETCADR`).

### TWIDDLE

`TWIDDLE` (WAITLIST.agc page 1223) is a WAITLIST variant that saves one word
of ROM by omitting the BBCON half of the 2CADR when the task is in the same
bank as the caller. In the Rust port this distinction disappears because Rust
function pointers already encode the full address.

### Initialization (STARTSB2)

`STARTSB2` (`FRESH_START_AND_RESTART.agc`, page 191) initializes the Waitlist
on every fresh start or software restart:
- `LST1+0` through `LST1+7` = `NEG1/2` (all delta-times set to sentinel).
- `LST2+0` through `LST2+17D` = `ENDTASK` (both words of each 2CADR slot
  filled with the ENDTASK sentinel).
- `TIME3` = `POSMAX` (so T3RUPT does not fire until a real task is scheduled).

---

## Rust API

### Module Path

`agc_core::executive::waitlist`

### Types

```rust
/// A task function pointer for Waitlist tasks.
///
/// No closures — no heap captures. All state the task needs must be
/// reachable through `&mut AgcState` or static Mutex<RefCell<T>>.
///
/// AGC source: 2CADR entries in LST2 (WAITLIST.agc / ERASABLE_ASSIGNMENTS.agc).
pub type TaskFn = fn(&mut AgcState);

/// One slot in the Waitlist.
///
/// Corresponds to one entry pair in (LST1[i], LST2[i*2..i*2+1]).
#[derive(Clone, Copy)]
pub struct WaitEntry {
    /// Time remaining until this task fires, in centiseconds.
    /// 0 means "this slot is the sentinel / ENDTASK".
    /// Stored as a positive delta-time in Rust (AGC encodes as negative).
    /// AGC source: LST1 delta-time encoding.
    pub delta_cs: u16,
    /// Task entry point. None encodes the ENDTASK sentinel.
    /// AGC source: LST2 2CADR fields.
    pub task: Option<TaskFn>,
}

/// The complete Waitlist state.
///
/// All fields are fixed-size arrays — no heap.
/// AGC source: LST1 (8 words) + LST2 (18 words) in ERASABLE_ASSIGNMENTS.agc.
pub struct Waitlist {
    /// Delta-time chain. `delta[i]` is the centisecond interval between
    /// task `i` and task `i+1`. `delta[7]` is always the sentinel (0
    /// in the Rust encoding, NEG1/2 in the AGC).
    ///
    /// AGC source: LST1+0..+7 (ERASABLE_ASSIGNMENTS.agc line 2105).
    pub(crate) delta: [u16; MAX_WAITLIST_TASKS - 1],
    /// Task entry points, one per slot.
    /// `tasks[0]` is the next task to fire.
    ///
    /// AGC source: LST2+0..+17D (ERASABLE_ASSIGNMENTS.agc line 2106).
    pub(crate) tasks: [Option<TaskFn>; MAX_WAITLIST_TASKS],
    /// Number of currently occupied slots (0..=MAX_WAITLIST_TASKS).
    pub(crate) count: usize,
    /// Re-dispatch flag. True when TIME3 overflowed (another task due now).
    /// AGC source: RUPTAGN (ERASABLE_ASSIGNMENTS.agc line 1739).
    pub(crate) ruptagn: bool,
    /// Centiseconds until the head task fires (mirrors TIME3 semantics).
    /// Updated by `schedule` and decremented by the T3RUPT handler.
    /// AGC source: TIME3 counter register (ERASABLE_ASSIGNMENTS.agc line 125).
    pub(crate) time3_remaining_cs: u16,
}

/// Maximum number of concurrent waitlisted tasks.
/// AGC source: WAITLIST.agc page 1221 "9 TASKS MAXIMUM".
/// Confirmed: LST2 ERASE +17D = 18 words = 9 two-word slots.
pub const MAX_WAITLIST_TASKS: usize = 9;

/// Minimum delta-time for a Waitlist entry, in centiseconds.
/// AGC source: WAITLIST.agc page 1221 "1 <= C(A) <= 16250D".
pub const MIN_DELTA_CS: u16 = 1;

/// Maximum delta-time for a single Waitlist entry, in centiseconds.
/// Corresponds to 162.5 seconds. Beyond this use LONGCALL chaining.
/// AGC source: WAITLIST.agc page 1221 "MOD NO-2 (DTMAX INCREASED TO 162.5 SEC)".
pub const MAX_DELTA_CS: u16 = 16_250;

/// Sentinel value for an unused LST1 slot (NEG1/2 in AGC encoding).
/// In the Rust encoding this is stored as u16::MAX to mean "no task".
const ENDTASK_SENTINEL: u16 = u16::MAX;
```

### Functions

```rust
impl Waitlist {
    /// Create a zero-initialized Waitlist (all slots are ENDTASK sentinel).
    /// Called once from `fresh_start::init()` during FRESH START / STARTSB2.
    ///
    /// AGC source: FRESH_START_AND_RESTART.agc STARTSB2 (page 191),
    ///   `CAF NEG1/2; TS LST1+7; ...; CS ENDTASK; TS LST2; ...`.
    pub const fn new() -> Self;

    /// Schedule a task to run after `delta_cs` centiseconds.
    ///
    /// Inserts the task into the sorted delta-time chain. Must be called
    /// with interrupts disabled (from within `interrupt::free`).
    ///
    /// Returns `Some(())` on success.
    /// Returns `None` and raises alarm `WaitlistOverflow` (1203) if
    /// all 9 slots are occupied.
    ///
    /// # Panics (debug only)
    /// Panics if `delta_cs == 0` (corresponds to AGC alarm 1204 POODOO).
    ///
    /// AGC source: WAITLIST.agc WAITLIST / WAIT2 / WTLST4 / WTLST5 / WTLST2.
    pub fn schedule(&mut self, delta_cs: u16, task: TaskFn) -> Option<()>;

    /// T3RUPT handler — called every centisecond from the hardware timer ISR.
    ///
    /// Decrements `time3_remaining_cs`. When it reaches zero, pops the
    /// head task from the chain, reloads `time3_remaining_cs` from the
    /// next delta entry, sets `ruptagn` if another task is immediately due,
    /// and returns the task function pointer to dispatch.
    ///
    /// Caller is responsible for invoking the returned `TaskFn` and then
    /// calling `taskover` to check for re-dispatch.
    ///
    /// Must be called from within the `#[interrupt] fn T3RUPT()` handler.
    /// Must NOT be called from foreground code.
    ///
    /// AGC source: WAITLIST.agc T3RUPT / T3RUPT2 (page 1231).
    pub fn t3rupt_tick(&mut self) -> Option<TaskFn>;

    /// Check re-dispatch flag after a task completes.
    ///
    /// Returns `Some(task_fn)` if another task was due during this T3RUPT
    /// (RUPTAGN was set), and advances the chain again.
    /// Returns `None` when all due tasks for this interrupt have been
    /// dispatched.
    ///
    /// Must be called from task context (within interrupt), not from
    /// foreground.
    ///
    /// AGC source: WAITLIST.agc TASKOVER (page 1232).
    pub fn taskover(&mut self) -> Option<TaskFn>;

    /// Return the number of currently scheduled tasks (0..=9).
    pub fn task_count(&self) -> usize;

    /// Return `true` if the Waitlist has no scheduled tasks.
    pub fn is_empty(&self) -> bool;

    /// Schedule a long-interval task (delta_cs > MAX_DELTA_CS).
    ///
    /// Chains intermediate relay tasks at MAX_DELTA_CS intervals until
    /// the remaining time fits in a single Waitlist slot.
    ///
    /// AGC source: WAITLIST.agc LONGCALL / LONGCYCL / LASTTIME / GETCADR
    ///   (pages 1233–1235).
    pub fn schedule_long(&mut self, delta_cs: u32, task: TaskFn) -> Option<()>;
}
```

### ISR Integration Pattern

```rust
// In agc_core (implementation sketch — not compilable code):
//
// static WAITLIST: Mutex<RefCell<Waitlist>> =
//     Mutex::new(RefCell::new(Waitlist::new()));
//
// #[interrupt]
// fn T3RUPT() {
//     // Phase 1: pop due task
//     let maybe_task = cortex_m::interrupt::free(|cs| {
//         WAITLIST.borrow(cs).borrow_mut().t3rupt_tick()
//     });
//
//     // Phase 2: run all due tasks for this interrupt epoch
//     let mut next = maybe_task;
//     while let Some(task_fn) = next {
//         // Provide minimal state view; tasks must not block.
//         task_fn(state_ref);
//         next = cortex_m::interrupt::free(|cs| {
//             WAITLIST.borrow(cs).borrow_mut().taskover()
//         });
//     }
// }
```

---

## Scale Factors

| Quantity | AGC encoding | Rust type | Notes |
|---|---|---|---|
| Delta-time | Centiseconds, ones-complement negative in LST1 | `u16` (positive) | `delta_cs = 1` → 10 ms granularity |
| TIME3 | `16384 - (T1 - T_now)` | `u16` | Maps to `time3_remaining_cs` field |
| Task address | 2CADR (15-bit addr + 9-bit BBCON) | `fn()` pointer | Rust collapses to a single word |
| RUPTAGN | 0 or +1 AGC word | `bool` | `true` = re-dispatch needed |

The AGC's TIME3 counter is a 15-bit counter that overflows (triggers T3RUPT)
when it reaches +0 from a preloaded positive value. The Rust port models this
as a simple countdown of centisecond ticks.

The conversions between centiseconds and SI seconds used at call sites:
```
delta_seconds = (delta_cs as f64) * 0.01
```

---

## Invariants

1. **No heap.** `Waitlist` uses only fixed-size arrays. No `Vec`, `Box`,
   or dynamic allocation.

2. **No `static mut`.** The global `Waitlist` is stored in
   `Mutex<RefCell<Waitlist>>`. All access goes through
   `cortex_m::interrupt::free`.

3. **ISR-safe.** `t3rupt_tick` and `taskover` are called exclusively from
   within the `T3RUPT` interrupt handler. They must complete in bounded time
   (no loops proportional to task count in the hot path; the chain shift is
   always exactly 8 or 9 exchanges).

4. **No blocking in tasks.** Task functions (type `TaskFn`) must not
   block, spin, or perform long computation. Work that cannot complete quickly
   must schedule an Executive job via `add_job`.

5. **Overflow alarm 1203.** When all 9 slots are occupied and `schedule` is
   called, alarm `01203` (`WaitlistOverflow`) is raised and `None` is returned.
   Reference: WAITLIST.agc page 1229 `TC BAILOUT / OCT 1203`.

6. **Zero delta-time forbidden.** `delta_cs == 0` is invalid. In the AGC,
   this triggers alarm 1204 (`POODOO`). In Rust, a debug assertion fires; in
   release mode the call is silently ignored and `None` is returned.

7. **Sorted order maintained.** After every `schedule` call, the task chain
   remains sorted by ascending fire time. The delta encoding guarantees this:
   each `delta[i] > 0` unless it is the sentinel.

8. **ENDTASK sentinel integrity.** `tasks[count]` (and all beyond) must be
   `None`. The chain-shift logic in `t3rupt_tick` must restore `None` to the
   tail after popping the head.

---

## Test Cases

### Test 1 — Single Task Scheduled and Fired

**Setup:** Fresh `Waitlist`. Call `schedule(delta_cs: 10, task: my_task)`.

**Expected:**
- `task_count()` returns 1.
- `t3rupt_tick()` called 9 times: returns `None` each time.
- `t3rupt_tick()` called the 10th time: returns `Some(my_task)`.
- After firing: `task_count()` returns 0, `is_empty()` returns `true`.
- `taskover()` returns `None` (no re-dispatch needed).

### Test 2 — Multi-Task Chain, Correct Order

**Setup:** Fresh `Waitlist`. Schedule three tasks:
- `schedule(5, task_a)`
- `schedule(20, task_b)`
- `schedule(8, task_c)`

After scheduling, the chain should be sorted: task_a (5 cs), task_c (3 cs
delta from a), task_b (12 cs delta from c).

**Expected:**
- After 5 ticks: `t3rupt_tick()` returns `Some(task_a)`.
- After 3 more ticks (total 8): `t3rupt_tick()` returns `Some(task_c)`.
- After 12 more ticks (total 20): `t3rupt_tick()` returns `Some(task_b)`.
- All tasks fired in correct chronological order.

### Test 3 — Overflow Raises Alarm 1203

**Setup:** Fill all 9 slots by calling `schedule` 9 times with different
delta values. Attempt a 10th `schedule` call.

**Expected:**
- `schedule` returns `None` on the 10th call.
- `alarm::last_raised()` returns `Some(AlarmCode::WaitlistOverflow)`.
- The 9 existing entries are undisturbed; `task_count()` still returns 9.

---

## agc-sim Impact

- `MissionState`: add field `waitlist_task_count: usize` — count of active Waitlist slots.
- `SimLog`: emit `.warn("WAITLIST OVERFLOW: alarm 1203")` whenever `WaitlistOverflow` is raised.
- `SimLog`: emit `.debug("T3RUPT: dispatching task")` on each `t3rupt_tick` that fires a task (debug builds only, guarded by `#[cfg(debug_assertions)]`).
- No new DSKY display state required; the Waitlist is internal scheduler state.
