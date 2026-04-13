# AGC-to-Rust Reimplementation — Test Specification

**Source:** *The Apollo Guidance Computer: Architecture and Operation* (Frank O'Brien)
**Scope:** EXECUTIVE + WAITLIST + Phase-Table Restart, plus the arithmetic foundation that underpins them.
**Target:** Rust reimplementation with native datatypes (two's complement `i16`/`i32`, `f64` where interpretive math is used).

Citations use the form `[AGC, ch. N, p. X]` referring to the printed book page in *The Apollo Guidance Computer*. These are the pages the spec was derived from; every test case should link back to its source paragraph so future maintainers can distinguish "bug in Rust" from "bug in spec interpretation."

---

## 0. How to Use This Specification

Each test case has five fields:

- **ID** — stable identifier for CI dashboards (`ARITH-001`, `EXEC-014`, etc.)
- **Source** — book page(s) the requirement is derived from
- **Setup** — preconditions the test must establish
- **Action** — the operation under test
- **Expected** — the required outcome, plus any tolerance

Tests are grouped into four modules that match the layered architecture: arithmetic is the foundation, the Executive sits on top of it, the Waitlist sits on top of the Executive, and the phase-table restart machinery spans both. Integration tests at the end exercise all four layers together.

**Policy on spec vs. original assembler divergence.** The spec is the source of truth. If a test derived from the spec fails against the original assembler's behavior, the divergence is documented in the test notes and the Rust implementation follows the spec. Known AGC bugs (for example, the famous 1202 alarm condition is a *design limit*, not a bug) are called out explicitly.

---

## 1. Module: Arithmetic and Number Systems

The AGC uses 15-bit one's complement with a distinct negative zero. The Rust reimplementation uses native two's complement `i16`/`i32`. This module exists to pin down every place where that mapping is lossy or semantically different, so the divergence is explicit and tested.

### 1.1 Word Layout

| | Bit 15 | Bits 14–0 |
|---|---|---|
| AGC word | Sign | Magnitude (one's complement) |

Range: `-16,383` to `+16,383`, with both `+0` (`00000₈`) and `-0` (`77777₈`) representable. [AGC, ch. 1, pp. 20–22]

### 1.2 Test Cases

#### ARITH-001 — Positive integer round-trip
- **Source:** p. 20, Fig. 3
- **Setup:** Decimal value 750.
- **Action:** Encode as an AGC word, decode to decimal.
- **Expected:** Bit pattern `000 001 011 101 110`; decoded value equals 750.

#### ARITH-002 — Negative integer encoding
- **Source:** pp. 20–21
- **Setup:** Decimal values −1, −2, −5, −16,383.
- **Action:** Encode each as an AGC word via bitwise complement of the positive value.
- **Expected:** −1 = `111 111 111 111 110`; −5 = `111 111 111 111 010`; −16,383 = `100 000 000 000 000`.

#### ARITH-003 — Range limits
- **Source:** p. 22
- **Action:** Encode `+16,383` and `−16,383`; encode `+16,384` and `−16,384`.
- **Expected:** `±16,383` succeed. `±16,384` must be rejected by the constructor (in Rust: `TryFrom` returns an error or `debug_assert!` fires). This is tighter than native `i16` range and must be enforced.

#### ARITH-004 — Negative zero is representable
- **Source:** p. 22
- **Action:** Encode `-0` (`77777₈`) and compare to `+0` (`00000₈`).
- **Expected:** Both values decode to the integer 0, but must remain distinguishable at the word level. The Rust representation must carry a sign-preservation tag or use an explicit enum variant for the handful of variables that legally hold negative zero (document these in `ERASABLE_ASSIGNMENTS` mapping).

#### ARITH-005 — End-around carry addition
- **Source:** pp. 21–22
- **Setup:** Two AGC words whose magnitudes are `−2` and `−5`.
- **Action:** Add per the AGC rule: ordinary binary addition, then add any carry out of bit 15 back into bit 0.
- **Expected:** Result equals `−7`. Intermediate naive addition gives `−8`; the end-around carry corrects to `−7`.

#### ARITH-006 — End-around carry test matrix
- **Source:** pp. 21–22
- **Action:** For every pair (a,b) drawn from {−16383, −8192, −1, −0, +0, +1, +8192, +16383}, compute AGC-addition and compare to the arithmetically correct result.
- **Expected:** All 64 combinations match. This is the exhaustive check that Rust `i32::wrapping_add` + end-around correction agrees with 1's complement.

#### ARITH-007 — Two's complement comparison
- **Source:** p. 28, Fig. 6
- **Setup:** Decimal −29 encoded as 1's complement (`111 111 111 100 010`) and as 2's complement (`111 111 111 100 011`).
- **Action:** Confirm that the two encodings differ by exactly 1 in the low bit and that naive reuse of 2's complement for AGC arithmetic will silently produce wrong results.
- **Expected:** Test passes if and only if the mapping layer explicitly converts between the two representations at the boundaries; no silent reinterpretation.

#### ARITH-008 — Overflow detection via modified 1's complement
- **Source:** p. 22, and ch. 1 TS instruction (Fig. 23, p. 79)
- **Setup:** Values `+16383` and `+1`.
- **Action:** Perform AGC add with overflow detection.
- **Expected:** Result indicates overflow condition. The Rust newtype must expose an overflow flag (equivalent to the AGC's "modified one's complement" accumulator state), not silently wrap.

#### ARITH-009 — Fractional round-trip
- **Source:** pp. 22–23, Fig. 4 ("Fractional powers of 2")
- **Setup:** Decimal 0.4375.
- **Action:** Encode as fractional AGC word (bit 14 = 2⁻¹, …, bit 0 = 2⁻¹⁴).
- **Expected:** Bit pattern `001 110 000 000 000`; reading back gives 0.4375 exactly.

#### ARITH-010 — Irrational approximation bound
- **Source:** p. 23
- **Setup:** Exact value 1/3.
- **Action:** Encode as a single-precision fractional AGC word.
- **Expected:** Decoded value 0.33331298828125, with error bounded by 2⁻¹⁴ ≈ 6.1 × 10⁻⁵. This test sets the single-precision fractional error budget for any higher-level nav test.

#### ARITH-011 — Double-precision concatenation
- **Source:** pp. 27–28
- **Setup:** Two adjacent 14-bit magnitude fields + two sign bits = 28-bit value.
- **Action:** Encode a value requiring more than 14 bits of precision.
- **Expected:** 28-bit datatype gives ≈ 8 decimal digits of precision, which per spec is "sufficient for its most critical role of guidance and navigation" (p. 23).

#### ARITH-012 — Double precision with mixed signs (legal)
- **Source:** p. 28
- **Setup:** A double-precision value whose upper word is positive and lower word is negative.
- **Action:** Decode it.
- **Expected:** Decoded value is computed as the arithmetic sum of the two independently-signed word values, and the result is valid. This surprising property must round-trip correctly — per the book example, (5·10) + (−7·1) = 43 is a valid representation of 43.

#### ARITH-013 — Interflow between double-precision words
- **Source:** p. 28
- **Setup:** A DP operation where the low word overflows.
- **Action:** Apply AGC "interflow" rule: the overflow from the low-order word carries into the high-order word.
- **Expected:** Interflow is distinguishable from ordinary overflow (which should trigger the alarm behavior of ARITH-008). Tests must exercise both directions.

#### ARITH-014 — Scaling precision bound
- **Source:** pp. 25–26
- **Action:** Property test: for a vector of navigation-domain values (velocities up to ~8000 m/s, distances up to lunar distance), verify that scaling choices preserve accuracy to at least the spec-stated 9 decimal digits / 28 bits.
- **Expected:** Maximum relative error across 10,000 `proptest` inputs is below the budget. This is the main numerical safety net for the Rust `f64` representation replacing the AGC's scaled fixed-point.

---

## 2. Module: The Executive (Job Scheduler)

The AGC Executive is a cooperative priority scheduler with preemptive interrupt elements. Jobs live in **Core Sets** — 12-word data areas in unswitched erasable memory. The CM has 6 Core Sets; the LM has 7. Interpretive jobs additionally allocate a **VAC (Vector Accumulator) area** of 43 words, 5 of which exist in the LM. [AGC, ch. 2, pp. 103–113]

### 2.1 Core Set Layout (Figure 30, p. 105)

| Octal offset | Field |
|---|---|
| 00 | MPAC |
| 01 | MPAC+1 |
| 02 | MPAC+2 |
| 03 | MPAC+3 |
| 04 | MPAC+4 |
| 05 | MPAC+5 |
| 06 | MPAC+6 |
| 07 | MPAC Mode |
| 10 | LOC (Program Counter) |
| 11 | BANKSET (BBANK) |
| 12 | PUSHLOC |
| 13 | PRIORITY / VAC Pointer |

The first 7 words double as Multipurpose Accumulator (MPAC) when interpretive code is running; when basic code runs, they are unstructured scratch space.

### 2.2 VAC Area Layout (Figure 32, p. 107)

| Octal offset | Field |
|---|---|
| 00 | In-use flag |
| 01–41 | 33 words for stack and temporaries |
| 42 | `|V(MPAC)|²` (after UNIT and ABVAL) |
| 43 | `|V(MPAC)|²` (cont.) |
| 44 | `|V(MPAC)|` (after UNIT) |
| 45 | `|V(MPAC)|` (cont.) |
| 46 | Index Register S1 |
| 47 | Index Register S2 |
| 50 | Step Register S1 |
| 51 | Step Register S2 |
| 52 | QPRET |

### 2.3 Sentinel Values

- **Available Core Set:** priority field = negative zero (`77777₈`) [p. 105]
- **Sleeping job priority:** priority field = complement of its original priority (e.g., 33 → −33) [p. 112]
- **Currently executing job:** always occupies Core Set 0 [p. 106]
- **NEWJOB location:** fixed at erasable address `00067₈` [p. 108]
- **Highest priority indicator:** `NEWJOB = +0` [p. 108]

### 2.4 Test Cases

#### EXEC-001 — Core Set table sizing
- **Source:** pp. 105–106
- **Action:** Instantiate a `CommandModuleScheduler` and a `LunarModuleScheduler`.
- **Expected:** CM has exactly 6 Core Sets; LM has exactly 7. The Rust type should make this a const generic or compile-time constant — not a runtime value.

#### EXEC-002 — Core Set 0 is reserved for executing job
- **Source:** p. 106
- **Setup:** Scheduler is idle (DUMMY running).
- **Action:** Start a new job via `NOVAC`.
- **Expected:** After dispatch, Core Set 0 holds the new job's state. No other index is valid for the "currently executing" role.

#### EXEC-003 — Negative-zero priority marks availability
- **Source:** pp. 105, 108
- **Setup:** A Core Set with priority field = `77777₈`.
- **Action:** Scan the Core Set table for an available entry.
- **Expected:** This Core Set is selected. This test pins the fact that *negative zero is semantically meaningful* in the scheduler — it is not interchangeable with positive zero. See ARITH-004.

#### EXEC-004 — NOVAC allocation and priority storage
- **Source:** pp. 105–106
- **Setup:** Empty scheduler; call `NOVAC` with priority 22 and entry address `0x1234`.
- **Action:** Allocate a Core Set for a basic job.
- **Expected:** Some Core Set (not necessarily Core Set 0 yet) has priority = 22, LOC = `0x1234`, BBANK = caller's bank setting.

#### EXEC-005 — FINDVAC allocates VAC area before Core Set
- **Source:** p. 107
- **Setup:** Scheduler with at least one available Core Set and at least one available VAC area.
- **Action:** Call `FINDVAC` for an interpretive job.
- **Expected:** A VAC area is marked in-use *first*; its address is stored in the Core Set's PRIORITY/VAC pointer word; the Core Set is then allocated.

#### EXEC-006 — FINDVAC fails if no VAC area
- **Source:** p. 107; and 1202 alarm history p. 106 footnote 3
- **Setup:** All 5 VAC areas in use.
- **Action:** Call `FINDVAC`.
- **Expected:** Allocation fails. The spec calls this condition the root of the historic 1202 alarm. The Rust API must return a distinct `NoVacArea` error so the alarm-handling layer can recognize it.

#### EXEC-007 — NOVAC fails if no Core Set
- **Source:** p. 106, footnote 3
- **Setup:** All Core Sets in use.
- **Action:** Call `NOVAC`.
- **Expected:** Allocation fails with a distinct `NoCoreSet` error, corresponding to the 1201 alarm.

#### EXEC-008 — Priority-based dispatch
- **Source:** pp. 108–110
- **Setup:** Core Set table containing jobs with priorities `31, -0, 14, -0, -0, 6, 22` (Figure 31 exactly).
- **Action:** Invoke the scheduler decision.
- **Expected:** Priority 31 job runs. On its completion, priority 22 runs next, then 14, then 6. DUMMY never runs while non-available entries remain.

#### EXEC-009 — NEWJOB semantics: +0
- **Source:** p. 108
- **Setup:** Running job tests NEWJOB.
- **Action:** Execute the CCS NEWJOB / TC CHANG1 sequence.
- **Expected:** When NEWJOB = +0 (positive zero, not negative zero), the current job continues. Negative zero or any other value must cause dispatch.

#### EXEC-010 — NEWJOB semantics: non-zero
- **Source:** p. 108
- **Setup:** A higher-priority job is placed in Core Set 3; NEWJOB is set to the address of Core Set 3.
- **Action:** Current job queries NEWJOB.
- **Expected:** Current job's state is swapped into the Core Set at the address in NEWJOB; the higher-priority job is swapped into Core Set 0; control transfers.

#### EXEC-011 — NEWJOB is at fixed address 00067₈
- **Source:** p. 108
- **Action:** Read the Rust memory model's NEWJOB symbol.
- **Expected:** Memory offset = `00067₈` = 55 decimal. This is a spec-defined fixed address because of hardware interaction with the Night Watchman (see EXEC-012).

#### EXEC-012 — Night Watchman 640 ms timeout
- **Source:** p. 109
- **Setup:** Run a job that never checks NEWJOB.
- **Action:** Let 640 ms of simulated time pass.
- **Expected:** System restart is triggered. This is the hardware-level safety net that detects a non-cooperating job. The Rust implementation must preserve this behavior (the cooperative scheduler is not just a performance detail — it has a safety property attached).

#### EXEC-013 — ENDOFJOB frees Core Set
- **Source:** p. 111
- **Setup:** A job running in Core Set 0 with non-zero priority.
- **Action:** Job calls `ENDOFJOB`.
- **Expected:** Priority field of the former Core Set is set to `77777₈` (negative zero). If a VAC area was attached, its address is saved in `VACUSE` and its in-use flag is cleared. The scheduler then scans for the next highest-priority job.

#### EXEC-014 — Self-priority change (PRIOCHNG)
- **Source:** pp. 110–111
- **Setup:** A job at priority 25 in Core Set 0; a waiting job at priority 20 in Core Set 3.
- **Action:** Running job calls `PRIOCHNG` with new priority 15.
- **Expected:** Current job's priority becomes 15, scheduler rescans, priority-20 job becomes highest, context swap occurs. A job cannot change *another* job's priority (only itself).

#### EXEC-015 — JOBSLEEP complements priority
- **Source:** p. 112
- **Setup:** A job at priority 33.
- **Action:** Call `JOBSLEEP`.
- **Expected:** Priority field contains `-33` (one's-complement negation of 33), which preserves the original value for restoration while marking the job inactive. `-33` must be distinguishable from the "available" sentinel (which is specifically negative zero, not any negative value).

#### EXEC-016 — JOBWAKE restores priority
- **Source:** p. 112
- **Setup:** Sleeping job with priority field = −33.
- **Action:** Another program calls `JOBWAKE` with the sleeping job's Core Set address.
- **Expected:** Priority is complemented back to +33; scheduler rescans and may dispatch.

#### EXEC-017 — DELAYJOB = JOBSLEEP + waitlist wake
- **Source:** p. 112
- **Setup:** A job calls `DELAYJOB` with a wait time of 500 ms.
- **Action:** Execute the call.
- **Expected:** Job's priority is complemented (sleep), and a waitlist task is inserted that will call `JOBWAKE` after 500 ms. Maximum concurrent sleepers: 3 in LM, 4 in CM.

#### EXEC-018 — DUMMY job when queue empty
- **Source:** p. 112
- **Setup:** All Core Sets are available except one marked as DUMMY.
- **Action:** Scheduler dispatch.
- **Expected:** DUMMY executes; the system does not halt. DUMMY's priority must be the lowest possible value so any real job preempts it.

#### EXEC-019 — CHANGJOB swap semantics
- **Source:** p. 110
- **Setup:** Current job in Core Set 0 with return-address info; higher-priority job in Core Set N.
- **Action:** Call `CHANGJOB`.
- **Expected:** Current job's return address and BBANK are saved into Core Set 0's LOC/BBANK fields, then Core Sets 0 and N are swapped, then LOC and BBANK are loaded from the new Core Set 0, then NEWJOB is reset to +0. This five-step sequence must be atomic from the caller's perspective.

#### EXEC-020 — Major Modes run at low priority
- **Source:** p. 113
- **Action:** Schedule a Major Mode program (e.g., P63 lunar landing).
- **Expected:** Its priority is low — lower than any routine job it spawns. This is counter-intuitive but per spec: a Major Mode that ran at high priority would starve the jobs it schedules.

---

## 3. Module: The Waitlist

The waitlist is a table of up to 7 deferred tasks, ordered chronologically by relative interval. It is driven by the `T3RUPT` interrupt which fires when `TIME3` overflows. One tick = 10 ms. [AGC, ch. 2, pp. 113–117]

### 3.1 Data Structures

- **LST1**: time intervals (relative, in 10 ms ticks)
- **LST2**: task entry addresses (paired with LST1)
- **TIME3**: countdown register; overflow triggers T3RUPT
- **ARUPT / LRUPT / QRUPT**: save slots for A, L, Q registers during waitlist task execution

### 3.2 Test Cases

#### WAIT-001 — TIME3 overflow drives T3RUPT
- **Source:** p. 114
- **Setup:** TIME3 contains 1 tick.
- **Action:** Advance simulated time by 10 ms.
- **Expected:** TIME3 overflows, T3RUPT fires, waitlist scheduler runs.

#### WAIT-002 — First task's interval lives in T3RUPT/TIME3
- **Source:** p. 114
- **Setup:** Schedule a task 140 ms in the future. No other tasks pending.
- **Action:** Inspect the waitlist state.
- **Expected:** LST1[0] = 14 ticks loaded into TIME3. LST2[0] = task 1's address. LST2 has a corresponding first entry; LST1's first slot holds the in-flight timer.

#### WAIT-003 — Chronological insertion — append to tail
- **Source:** p. 114
- **Setup:** Waitlist contains Task A at 30 ms from now and Task B at 70 ms from now. TIME3 shows 30 ms remaining. A new request comes for 180 ms from now.
- **Action:** Insert Task C.
- **Expected:** Per spec example (p. 114), the new LST1 interval for C equals `180 − 30 − 40 = 110 ms` (i.e., 11 ticks), and the entry is appended after Task B.

#### WAIT-004 — Chronological insertion — middle
- **Source:** pp. 114–115
- **Setup:** TIME3 = 20 ms, Task A at 20 ms, Task B at 100 ms after Task A.
- **Action:** Insert Task C at 60 ms from now (i.e., 40 ms after Task A).
- **Expected:** After insertion the order is A, C, B. C's LST1 interval = 40 ms after A = 4 ticks. B's LST1 interval is *recomputed* to be 60 ms after C (so the total from A→B remains 100 ms).

#### WAIT-005 — Task execution in interrupt-inhibited mode
- **Source:** pp. 113–114
- **Setup:** Schedule a waitlist task.
- **Action:** Fire it.
- **Expected:** The task runs with interrupts inhibited. No other waitlist task or job can dispatch until it yields or returns. This is a hard invariant — the Rust implementation must model this explicitly (e.g., a `critical_section` token or a scheduler state flag).

#### WAIT-006 — 5 ms informal time limit
- **Source:** p. 113
- **Setup:** A task that exceeds 5 ms of simulated execution time.
- **Action:** Run the scheduler with a long task.
- **Expected:** The scheduler does NOT enforce the 5 ms limit (spec is explicit: "There is no mechanism to interrupt or otherwise enforce this limit"). A test build may include a `debug_assert!` that warns past 5 ms, but release behavior is unchanged. This test pins the policy: spec-conformant behavior is "no enforcement."

#### WAIT-007 — A/L/Q register preservation
- **Source:** p. 114
- **Setup:** Set A, L, Q to known values; schedule a task that clobbers all three.
- **Action:** Run the waitlist task.
- **Expected:** After the task returns, A, L, Q are restored from ARUPT, LRUPT, QRUPT to their pre-task values.

#### WAIT-008 — One-shot semantics
- **Source:** p. 115
- **Setup:** Schedule a periodic-looking task.
- **Action:** Let it fire once.
- **Expected:** The task does NOT reschedule automatically. If the task wants to run periodically, it must call into the waitlist from within its own handler to schedule the next instance.

#### WAIT-009 — Entry removal compacts table
- **Source:** p. 115
- **Setup:** Waitlist with 4 entries; the head task fires.
- **Action:** Task completes.
- **Expected:** Entry 0 is removed; entries 1–3 shift up to positions 0–2; TIME3 is loaded with the new head's interval.

#### WAIT-010 — Capacity limits
- **Source:** Figure 34, p. 115
- **Setup:** Waitlist has 5 tasks + 2 available slots (Figure 34 depicts 7 slots total).
- **Action:** Attempt to schedule 3 more tasks.
- **Expected:** The first 2 succeed; the third fails with an explicit overflow indication (the scheduler cannot silently drop a waitlist task).

---

## 4. Module: Phase Tables and Restart Recovery

The restart system is the safety-critical heart of the AGC. Jobs are assigned to one of **six restart groups**; each group has one or more **phases**. When a program advances to a new safe checkpoint, it calls `PHASCHNG` with a group.phase identifier. On restart, the system re-enters each group at its last recorded phase. [AGC, ch. 2, pp. 115–119]

### 4.1 Phase Encoding

- **G.0** — Inhibit group restart (group is inactive, no action on restart)
- **G.1** — Restart last display on DSKY (common to all groups)
- **G.Even#** (2, 4, 6, …) — Execute TWO restart routines
- **G.Odd# > 1** (3, 5, 7, …) — Execute ONE restart routine

### 4.2 Test Cases

#### PHASE-001 — Six restart groups exist
- **Source:** p. 117
- **Action:** Inspect phase table structure.
- **Expected:** Exactly 6 groups. Group 2 is reserved for Servicer; group 4 is reserved for SPS burn / entry; group 5 is reserved for IMU management (per spec example).

#### PHASE-002 — DAP has no restart protection
- **Source:** p. 117
- **Action:** Enumerate which programs are restart-protected.
- **Expected:** The Digital Autopilot is explicitly NOT restart-protected. A failure during a DAP cycle is tolerable because the DAP runs again in ≤100 ms. This must be encoded in the Rust types: DAP-level code must not accept a `PhasChngHandle` because it has no group assignment.

#### PHASE-003 — G.0 inhibits restart for the group
- **Source:** p. 118
- **Setup:** Set phase table entry for group 3 to 0.
- **Action:** Trigger a restart.
- **Expected:** Group 3's restart routine is NOT invoked. The group is treated as inactive.

#### PHASE-004 — G.1 always restarts the last display
- **Source:** p. 118
- **Setup:** Group 2 at phase 1; DSKY holds the most recent display buffer.
- **Action:** Trigger a restart.
- **Expected:** The last DSKY display is redisplayed. No other restart processing occurs for group 2.

#### PHASE-005 — Even-phase executes two restart routines
- **Source:** p. 118
- **Setup:** Group 4 at phase 2; phase-table entry has two routine references.
- **Action:** Trigger a restart.
- **Expected:** Both restart routines execute, in the order they appear in the phase-table entry.

#### PHASE-006 — Odd-phase (>1) executes one restart routine
- **Source:** p. 118
- **Setup:** Group 4 at phase 3.
- **Action:** Trigger a restart.
- **Expected:** Exactly one restart routine runs.

#### PHASE-007 — Phase table stores bitwise-complement sanity check
- **Source:** p. 119
- **Setup:** Write a phase table entry via `PHASCHNG`.
- **Action:** Inspect storage.
- **Expected:** Each parameter is stored together with its bitwise complement. On restart, the sanity check reads both halves and fails loudly if `word ^ complement != all_ones`. This is the AGC's "detect erasable memory corruption" mechanism and must be preserved in the Rust reimplementation (even though RAM corruption is not a threat on modern hardware, the spec treats it as an invariant).

#### PHASE-008 — Phase table in unswitched erasable, restart table in unswitched fixed
- **Source:** p. 119
- **Action:** Inspect memory-model placement.
- **Expected:** Restart routine addresses live in ROM (unswitched fixed); current phase indicators live in RAM (unswitched erasable). In Rust: restart table is `&'static [RestartEntry]`, phase table is a mutable structure.

#### PHASE-009 — PHASCHNG Type A: fixed single-word parameter
- **Source:** p. 119
- **Action:** Call `PHASCHNG` with a Type A parameter.
- **Expected:** One fixed parameter word is consumed from the call site.

#### PHASE-010 — PHASCHNG Type C: variable-length parameter
- **Source:** p. 119
- **Action:** Call `PHASCHNG` with a Type C parameter (e.g., variable waitlist scheduling time).
- **Expected:** Additional parameter words are consumed as defined by the flag bits in the first parameter word.

#### PHASE-011 — Waitlist tasks have no restart protection
- **Source:** p. 117
- **Action:** Attempt to add restart protection to a waitlist task.
- **Expected:** API rejects the call. Waitlist tasks are explicitly excluded from restart protection per spec.

---

## 5. Integration Scenarios

These end-to-end tests exercise all four modules. They are intended to be run against both the Rust reimplementation and, where possible, a reference run of the Virtual AGC emulator for spot-check cross-validation.

### INT-001 — The 1201 Alarm Scenario
- **Source:** pp. 106 (fn 3), 117
- **Setup:** Saturate the Core Set table by queueing 7 jobs in the CM (6 Core Sets + 1 attempted overflow).
- **Action:** Attempt to schedule a 7th job via `NOVAC`.
- **Expected:** `NOVAC` returns `NoCoreSet`. The alarm-handling layer raises 1201. Lower-priority jobs are candidates for shedding. This is the historic "full Core Set table" condition.

### INT-002 — The 1202 Alarm Scenario
- **Source:** pp. 106 (fn 3), 107
- **Setup:** Allocate all 5 VAC areas in the LM. Continue running while a routine attempts to `FINDVAC` an interpretive job.
- **Action:** `FINDVAC` call.
- **Expected:** Returns `NoVacArea`. Alarm 1202 is raised. The Executive sheds jobs and recovers — it does not halt.

### INT-003 — Cooperative yield under priority inversion
- **Source:** pp. 109–110
- **Setup:** Low-priority job holding "the CPU" (simulated) with a 10 ms work loop that checks NEWJOB at the top of each iteration; schedule a higher-priority job mid-loop.
- **Action:** Let both run.
- **Expected:** Higher-priority job begins executing within 20 ms of being scheduled. If the low-priority job ever takes longer than 640 ms between NEWJOB checks, the Night Watchman restart fires (see EXEC-012).

### INT-004 — Full restart with phase recovery
- **Source:** pp. 116–119
- **Setup:** Start a multi-phase computation. At phase 3, call `PHASCHNG(group=2, phase=3)`. Continue into the phase-3 computation.
- **Action:** Mid-phase-3, trigger a restart.
- **Expected:** On restart, group 2 re-enters at phase 3 (its ONE restart routine runs). The computation resumes without re-running phases 1 and 2.

### INT-005 — DELAYJOB exercises three subsystems
- **Source:** p. 112
- **Action:** Schedule a job that calls `DELAYJOB(500ms)`, let time advance, verify wake-up.
- **Expected:** The test touches Executive (JOBSLEEP), Waitlist (scheduled wake task), and arithmetic (priority complementation). All three subsystems must be wired correctly for this to pass.

### INT-006 — Negative-zero round-trip across scheduler boundaries
- **Source:** pp. 105, 108
- **Action:** Run ARITH-004 (negative-zero representability) inside an EXEC-003 (available-Core-Set detection) test.
- **Expected:** The scheduler correctly distinguishes `-0` (available) from any other negative value (sleeping). This is the concrete place where the 1's complement / 2's complement mapping *must* be explicit.

---

## 6. Out of Scope (Documented Deferrals)

The following are present in the book but deliberately excluded from this spec because they belong to later phases of the reimplementation:

- Interpreter instruction set (pp. 160–197) — deferred to Phase 3 (math layer) once the Executive passes.
- DSKY / Pinball UI handler (pp. 123–140) — deferred to Phase 4 (I/O layer).
- Specific guidance programs P63, P20, etc. (ch. 4) — deferred until navigation is validated.
- Telemetry uplink and downlink (pp. 140–143) — deferred to I/O phase.
- Mission Program startup / FRESH_START (cross-referenced but not in Ch. 2 scope).

---

## 7. Test Infrastructure Recommendations

The spec as written is implementation-agnostic. The following are concrete suggestions for executing it in Rust:

- **`proptest`** for ARITH-006 exhaustive carry matrix and ARITH-014 property tests.
- **`cargo-nextest`** for parallel test execution and stable ordering of failures.
- **Named test modules** matching this document: `tests/arith.rs`, `tests/exec.rs`, `tests/wait.rs`, `tests/phase.rs`, `tests/integration.rs`.
- **Spec back-reference in each test:** every `#[test]` function should carry a `/// Source: AGC book p. XX` doc comment citing this document's test ID. A CI lint can enforce the presence of source citations.
- **Simulated time** must be injectable — the scheduler should not touch real clocks. This is non-negotiable for testing EXEC-012 (Night Watchman), WAIT-001 (T3RUPT), and any time-dependent test.
- **Deterministic RNG** for all property tests so failures are reproducible.

---

## 8. Source Document

All page numbers in this specification refer to: **O'Brien, Frank. *The Apollo Guidance Computer: Architecture and Operation.* Springer-Praxis, 2010.** Pages cited: 18–28 (number systems), 99–119 (Executive, Waitlist, Phase Tables). The book is 350 pages total; this spec covers Chapter 1 §§ "Properties of number systems" through "Double precision numbers", and Chapter 2 §§ "Scheduling" through "Phase tables and restart processing".
