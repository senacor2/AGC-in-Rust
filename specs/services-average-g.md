# Functional Specification: SERVICER Average-G (`agc-core/src/services/average_g.rs`)

## AGC Source Reference

```
AGC source: Comanche055/SERVICER207.agc
Routines:   PREREAD    (initialisation, pages 822-823)
            READACCS   (PIPA read + Waitlist reschedule, pages 823-826)
            SERVICER   (PIPA saturation check + PIPA compensation, pages 828-829)
            AVERAGEG   (calls CALCRVG, bulk-copies RN1→RN, exits, pages 828-829)
            CALCRVG    (predictor-corrector integration, pages 835-836)
            CALCGRAV   (gravity at predicted position, pages 835-836)
            NORMLIZE   (first-cycle initialisation of GDT/2, page 831)
            PIPASR     (read and clear PIPA counters, pages 832-834)
            SERVEXIT   (phase-change + ENDOFJOB, page 830)
Pages:      819-836

AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Labels:     RN, VN, PIPTIME, GDT/2, GOBL/2, RN1, VN1, PIPTIME1, GDT1/2,
            DELV (DELVX/Y/Z), DELVREF, KPIP1, REFSMMAT, UNITR, RMAG,
            AVGEXIT, PIPAGE, PIPCTR, DVCNTR, PHASE5 (restart group 5)
Pages:      79-81 (AGC hardcopy)
```

Constants: `docs/agc-reference-constants.md`

---

## Behavior Summary

The Average-G (SERVICER) routine is the 2-second navigation heartbeat of the AGC
during powered flight. It fires every 2 seconds via a Waitlist T3RUPT task
(`READACCS` is placed in the Waitlist with a delay of 2 seconds, `2CADR READACCS`
with `CAF 2SECS; TC WAITLIST`). When the `AVERAGEG` flag is set, READACCS
automatically re-schedules itself for the next 2-second cycle.

### The Full Cycle Sequence (one 2-second pass)

```
[T3RUPT fires, Waitlist calls READACCS]
1. PIPASR    — atomically read and zero the three PIPA counters (DELVX, DELVY, DELVZ)
               into DELV vector; timestamp in PIPTIME1.
2. READACCS  — if AVERAGEG flag set: schedule next READACCS in 2 seconds (TC WAITLIST).
               Schedule SERVICER as a job (TC FINDVAC / NOVAC).
3. SERVICER  — check PIPA saturation (-MAXDELV = −6398 counts for 2 s).
               If saturated: alarm 00205, skip compensation, jump to AVERAGEG.
               Call 1/PIPA compensation subroutine.
               Accumulate DVTOTAL.
4. AVERAGEG  — call CALCRVG (predictor-corrector step).
               Bulk-copy RN1,VN1,GDT1/2,GOBL1/2,PIPTIME1 → RN,VN,GDT/2,GOBL/2,PIPTIME
               via GENTRAN (14 words = 7 DP pairs, restart-protected).
               Phase-change for restart protection (PHASCHNG OCT 10035).
               Jump to AVGEXIT (caller-provided hook, or SERVEXIT).
```

### CALCRVG Algorithm (SERVICER207.agc page 835-836)

The heart of Average-G is `CALCRVG`, which is a trapezoidal predictor-corrector:

```
Input:
  RN      — current position, ECI, scaled B-29 m
  VN      — current velocity, ECI, scaled B-7 m/cs
  DELV    — PIPA counts (stable-member frame)
  GDT/2   — half-step gravity from previous cycle (B-7 m/cs)
  REFSMMAT — stable-member to ECI rotation matrix (18 words)

Step 1: Rotate DELV to ECI
  DELVREF = REFSMMAT^T × (DELV × KPIP1)    // KPIP1 = 0.074880 (B-7 m/cs per count)
  // KPIP1 scales DELV to same B-7 units as VN/GDT/2.
  // Note: 0.074880 in B-7 units = 0.074880 * 2^-7 m/cs = 0.074880/128*100 m/s/count
  //        = 0.05850 m/s/count = PIPA_SCALE from docs/agc-reference-constants.md

Step 2: Predictor (position)
  RN1 = RN + (VN + DELVREF/2 + GDT/2) * 2SEC(22)
  // 2SEC(22) = 200 B-22 cs in B-22, i.e. 200 centiseconds = 2 seconds
  // This advances position using velocity at the half-step

Step 3: Evaluate gravity at predicted position
  [CALCGRAV called with RN1 loaded in MPAC]
  GDT1/2 = -MU/|RN1|^3 * RN1 * dt/2   +  J2_oblateness * dt/2
  // Stores UNITR, RMAG for use later

Step 4: Corrector (velocity)
  VN1 = VN + DELVREF + GDT1/2 + GDT/2
  //        ^thrust    ^new grav   ^old grav  (trapezoidal gravity average)

Step 5: Save results
  RN1, VN1, GDT1/2, PIPTIME1 stored as temporaries.
  After phase-change, these are copied to RN, VN, GDT/2, PIPTIME.
```

The CALCGRAV call also computes `GOBL1/2` (oblateness half-step) separately via
the J2 terms (`20J`, `2J`, `RESQ`, `UNITW` in the source). For simplicity in
the Rust port, the J2 contribution is folded into `gdt_over_2` (see
`navigation-state-vector.md` notes).

### Restart Safety (Phase Table)

The AGC used `PHASCHNG` instructions at critical points in SERVICER to ensure
correct restart recovery. The AGC restart group for SERVICER is group 5
(`-PHASE5`, `NEWPHASE OCT 5`). Phase values observed in the source:

| AGC octal | Decimal | Meaning |
|---|---|---|
| OCT 16035 | = phase 5.7 | before PIPA compensation |
| OCT 10035 | = phase 5.4 | before CALCRVG |
| OCT 10035 | = phase 5.4 | after CALCRVG (reused) |
| OCT 00035 | = phase 5.0 | SERVEXIT (final phase) |

In the Rust port, the phase table hook maps to
`state.restart.set_phase(SERVICER_GROUP, phase)` calls bracketing each critical
section (PIPA read, integration, bulk copy).

---

## Constants

From `docs/agc-reference-constants.md` (all values authoritative from that file):

| Constant | Rust name | Value | Units | AGC source |
|---|---|---|---|---|
| PIPA scale factor | `PIPA_SCALE` | `0.0585` | m/s per count | SERVICER207.agc `KPIP1 2DEC 0.074880` |
| Cycle period | `CYCLE_DT` | `2.0` | seconds | SERVICER207.agc `CAF 2SECS` |
| PIPA saturation limit | `PIPA_MAX_COUNTS` | `6398` | counts | SERVICER207.agc `-MAXDELV DEC -6398` |
| Earth GM | `MU_EARTH` | `3.986_032e14` | m³/s² | ORBITAL_INTEGRATION.agc MUEARTH |
| Earth radius | `RE_EARTH` | `6_373_338.0` | m | LATITUDE_LONGITUDE_SUBROUTINES.agc ERAD |

Note on KPIP1: The AGC constant `KPIP1 2DEC 0.074880` is in units of (B-7 m/cs)
per count. Converting: 0.074880 × 2^-7 m/cs × 100 cs/s = 0.05850 m/s per count.
This matches the comment `# 1 PULSE = 5.85 CM/SEC` in SERVICER207.agc line 810.
The Rust `PIPA_SCALE = 0.0585 m/s/count` is the correct SI conversion.

---

## Public API

Module path: `agc_core::services::average_g`

### `AverageG` struct

```rust
/// SERVICER Average-G navigation cycle.
///
/// Implements the 2-second predictor-corrector integration cycle driven by
/// PIPA accelerometer readings. Called every 2 seconds via the Waitlist
/// (T3RUPT → READACCS task) while the AVERAGEG flag is set.
///
/// AGC source: Comanche055/SERVICER207.agc, AVERAGEG/CALCRVG routines, pages 835-836.
///
/// Restart safety: all multi-step operations are bracketed by
/// `state.restart.set_phase(SERVICER_RESTART_GROUP, phase)` calls.
/// A mid-cycle restart resumes from the last completed phase.
///
/// Invariants:
///   - No heap allocation.
///   - No `unwrap` or `expect`.
///   - PIPA read is atomic (INHINT/RELINT bracket in HAL, exposed as
///     `ImuIo::read_and_clear_pipa` which returns an `Option<[i16; 3]>`).
///   - If PIPA is saturated, raises alarm 00205 and skips the integration
///     step (state unchanged), matching SERVICER207.agc behaviour.
pub struct AverageG {
    state: StateVector,
}
```

### Constructor

```rust
impl AverageG {
    /// Initialise Average-G with the provided state vector.
    ///
    /// The `state.gdt_over_2()` field should be pre-initialised to
    /// `earth_gravity(&state.position()) * CYCLE_DT / 2` before the first cycle.
    /// This matches the AGC NORMLIZE routine which calls CALCGRAV to set GDT/2.
    ///
    /// AGC: NORMLIZE in SERVICER207.agc (page 831).
    pub fn new(initial_state: StateVector) -> Self;

    /// Return the current state vector (read-only view).
    pub fn state(&self) -> &StateVector;
}
```

### `cycle`

```rust
impl AverageG {
    /// Execute one 2-second Average-G cycle.
    ///
    /// Steps:
    ///   1. Read and atomically clear PIPA counts via `hw.imu().read_and_clear_pipa()`.
    ///      If `None` is returned (hardware not ready), return `Err(AvgGError::PipaNotReady)`.
    ///   2. Check saturation: if any |count| >= PIPA_MAX_COUNTS (6398), call
    ///      `hw.alarm().raise(AlarmCode::PipaSaturated)` and return the current
    ///      state unchanged as `Ok(self.state)`.
    ///   3. Set restart phase = PHASE_BEFORE_CALCRVG via `hw.restart()`.
    ///   4. Rotate PIPA counts to ECI:
    ///        dv_sm = counts * PIPA_SCALE    // per-axis, m/s, stable-member frame
    ///        refsmmat = hw.imu().refsmmat() // Mat3x3, SM→ECI rotation
    ///        dv_eci = mat_vec_mul(transpose(refsmmat), dv_sm)
    ///   5. Predictor (position):
    ///        v_half = self.state.velocity() + dv_eci * 0.5 + self.state.gdt_over_2()
    ///        r1 = self.state.position() + v_half * CYCLE_DT
    ///   6. Gravity at predicted position:
    ///        gdt_new = earth_gravity(&r1) * (CYCLE_DT / 2.0)
    ///        // includes J2 oblateness term via earth_gravity
    ///   7. Corrector (velocity):
    ///        v1 = self.state.velocity() + dv_eci + gdt_new + self.state.gdt_over_2()
    ///   8. Build new state:
    ///        t1 = self.state.time() + CYCLE_DT_CS  // CYCLE_DT_CS = 200 centiseconds
    ///        new_state = StateVector::with_gdt(r1, v1, t1, gdt_new)
    ///   9. Set restart phase = PHASE_AFTER_CALCRVG.
    ///  10. Commit: self.state = new_state.
    ///  11. Set restart phase = PHASE_SERVEXIT (final, safe state).
    ///  12. Return Ok(new_state).
    ///
    /// The `gdt_over_2` of the returned state is `gdt_new` (the half-step
    /// gravity computed at the predicted position r1), ready for the next cycle.
    ///
    /// Units: all internal computations in SI (m, m/s, s, m/s²).
    ///
    /// AGC source: CALCRVG (SERVICER207.agc page 835-836),
    ///             CALCGRAV (SERVICER207.agc page 835),
    ///             PIPASR (SERVICER207.agc pages 832-834).
    pub fn cycle(&mut self, hw: &mut dyn AgcHardware) -> Result<StateVector, AvgGError>;
}
```

### Error Type

```rust
/// Errors that can occur during an Average-G cycle.
///
/// These do not cause a GOJAM restart; the cycle skips and the state is
/// preserved unchanged, consistent with SERVICER207.agc alarm handling.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AvgGError {
    /// PIPA hardware not ready (read_and_clear_pipa returned None).
    PipaNotReady,
    /// One or more PIPA axes saturated (|count| >= PIPA_MAX_COUNTS).
    /// Alarm 00205 has been raised.
    /// AGC: SERVICER207.agc `-MAXDELV DEC -6398` and `TC ALARM / OCT 00205`.
    PipaSaturated,
}
```

### `initialize_gdt`

```rust
/// Initialise the `gdt_over_2` field of a `StateVector` using the current
/// gravitational acceleration.
///
/// Call this once before the first `AverageG::cycle` to replicate the
/// AGC NORMLIZE routine (which calls CALCGRAV to initialise GDT/2 before
/// the first Average-G pass).
///
/// Equivalent to:  state.with_gdt_over_2(earth_gravity(&state.position()) * (CYCLE_DT / 2.0))
///
/// AGC source: Comanche055/SERVICER207.agc, NORMLIZE routine (page 831).
pub fn initialize_gdt(state: StateVector) -> StateVector;
```

---

## Restart-Safety Rules

Phase constants (Rust names mapping to AGC PHASCHNG arguments):

```rust
/// Restart phase: before PIPA compensation and CALCRVG.
/// AGC: `TC PHASCHNG / OCT 16035` (SERVICER207.agc, before 1/PIPA call).
const PHASE_BEFORE_CALCRVG: u8 = 4;

/// Restart phase: during/after CALCRVG, before bulk copy.
/// AGC: `TC PHASCHNG / OCT 10035` (SERVICER207.agc, after CALCRVG call).
const PHASE_AFTER_CALCRVG: u8 = 4;   // same value in AGC; used twice

/// Restart phase: SERVEXIT — final safe state.
/// AGC: `TC PHASCHNG / OCT 00035` (SERVEXIT label).
const PHASE_SERVEXIT: u8 = 0;

/// Restart group for SERVICER.
/// AGC: `-PHASE5` / `NEWPHASE OCT 5`.
const SERVICER_RESTART_GROUP: u8 = 5;
```

On restart (FRESH_START_AND_RESTART.agc REREADAC label), the AGC checks `-PHASE5`
and resumes at READACCS if PIPAGE was non-zero at the time of restart. The Rust
restart mechanism must similarly branch on the saved phase to avoid double-reading
PIPAs.

---

## Executive / Waitlist Hook

The AGC schedules READACCS as a Waitlist task every 2 seconds:
```
CAF  2SECS
TC   WAITLIST
EBANK= AOG
2CADR READACCS
```
In the Rust port, the `AverageG::cycle` method is called by the Waitlist
task dispatcher. The caller (the `WaitlistTask` for READACCS) is responsible
for re-scheduling the next cycle via `waitlist.schedule(CYCLE_DT_CS, readaccs_task)`.
The `AverageG` struct itself does not hold a reference to the Waitlist.

---

## DSKY / agc-sim Impact

- The Mission State panel reads `AverageG::state()` every render tick to display
  MET, position radius (km), speed (m/s), and derived SMA/ECC/APO/PER.
- `SimHardware.imu()` must implement `read_and_clear_pipa() -> Option<[i16; 3]>`
  with a configurable injection queue for scenario testing.
- The `--scenario thrust` scenario must inject non-zero PIPA counts to exercise
  the thrust path through `cycle`.
- A "SERVICER ACTIVE" indicator in the sim log should be emitted at the start of
  each `cycle` call (log level DEBUG).

---

## Invariants

1. PIPA read is atomic: `ImuIo::read_and_clear_pipa` is the only call that reads
   and zeroes the PIPA registers. It must be implemented as an INHINT/RELINT
   critical section in the real HAL (interrupt-disabled read-then-zero). In the
   simulator, it is non-preemptive by construction.
2. `cycle` is idempotent under restart: if the restart occurs before
   `PHASE_AFTER_CALCRVG` is set, the cycle replays from PIPASR (re-reading PIPAs).
   If restart occurs after `PHASE_AFTER_CALCRVG`, the bulk copy from RN1→RN is
   replayed (idempotent because GENTRAN is a simple word copy).
3. `cycle` never calls `unwrap` or `expect`.
4. If PIPA saturation occurs, the state is not advanced (position/velocity
   unchanged). This matches the AGC: `TC ALARM / OCT 00205 / TC AVERAGEG` —
   the alarm is raised but AVERAGEG is still entered, which proceeds to copy
   the **unchanged** RN/VN into RN1/VN1 (no CALCRVG call happened).
   Correction: Re-reading the source, `TC AVERAGEG` after the alarm jumps
   past the CALCRVG call; `AVERAGEG` directly calls CALCRVG. Upon closer
   inspection, `TC AVERAGEG` at line `TC ALARM / OCT 00205 / TC AVERAGEG`
   in SERVICER207 skips the `1/PIPA` compensation and `DVTOTUP` steps and
   goes directly into AVERAGEG which calls CALCRVG. This means even on
   saturation, CALCRVG IS called with the uncompensated DELV. However,
   since the compensated DELV would be wildly wrong for a saturated PIPA,
   the safest Rust behaviour is to **skip CALCRVG and return unchanged state**
   with a `PipaSaturated` error, matching the intent (prevent corrupted state).
   This is flagged as an `APPROXIMATE` deviation for the validator.
5. No heap allocation.

---

## Test Cases

### Test 1 — Zero thrust (pure gravity coast)

```
// In zero thrust, PIPA counts are all zero.
// The cycle should advance position and velocity by pure gravity propagation.
r0 = [RE_EARTH + 200_000.0, 0.0, 0.0]   // 200 km LEO
v0 = [0.0, 7784.0, 0.0]                  // circular speed, m/s
gdt0 = earth_gravity(&r0) * (CYCLE_DT / 2.0)
state0 = StateVector::with_gdt(r0, v0, Met(0), gdt0)
avg_g = AverageG::new(state0)

// Inject zero PIPA counts
hw.imu().inject_pipa([0i16, 0, 0])
hw.imu().set_refsmmat(IDENTITY_MAT3)  // no rotation for test simplicity

result = avg_g.cycle(&mut hw).unwrap()

// After 2 s with circular velocity, position should have moved by ~15 568 m in y
// and decreased slightly in x (gravity curves the orbit).
// Energy should be conserved within SERVICER tolerance (< 0.01 m/s velocity change).
dv = result.speed() - v0
assert!(dv.abs() < 0.01)   // docs/testing.md §5: velocity < 0.01 m/s
dr = (result.position()[1] - v0 * CYCLE_DT).abs()
assert!(dr < 1.0)           // docs/testing.md §5: position < 1 m
```

### Test 2 — Constant thrust 1 m/s² for 2 s

```
// A constant 1 m/s² thrust along +x axis for one 2-second cycle.
// PIPA counts: 1 m/s² × 2 s / PIPA_SCALE = 2.0 / 0.0585 ≈ 34.2 counts/axis
// Round to 34 counts on x, 0 on y and z.
pipa_counts = [34i16, 0, 0]
expected_dv_x = 34.0 * PIPA_SCALE   // ≈ 1.989 m/s in x
expected_dr_x ≈ expected_dv_x / 2   // half-step average ≈ 0.995 m extra in x

hw.imu().inject_pipa(pipa_counts)
hw.imu().set_refsmmat(IDENTITY_MAT3)
result = avg_g.cycle(&mut hw).unwrap()

// velocity should increase by ~ expected_dv_x in x (plus gravity effect)
assert!((result.velocity()[0] - v0[0] - expected_dv_x).abs() < 0.01)
```

### Test 3 — Mid-cycle restart resume

```
// Simulate a restart that occurs between PIPA read and CALCRVG.
// After restart, the cycle should re-read PIPAs and complete normally.

// Setup: inject PIPA counts
hw.imu().inject_pipa([100i16, 0, 0])
hw.imu().set_refsmmat(IDENTITY_MAT3)

// Simulate restart mid-cycle by resetting phase to PHASE_BEFORE_CALCRVG
// (This verifies the restart table logic, not the AverageG struct directly.
// In the unit test, call cycle twice with the same PIPA injection to confirm
// idempotency: the second call after restart should produce the same result.)
result1 = avg_g1.cycle(&mut hw).unwrap()

// Fresh AverageG (restart simulation)
hw.imu().inject_pipa([100i16, 0, 0])   // re-inject (simulating re-read after restart)
avg_g2 = AverageG::new(state0)
result2 = avg_g2.cycle(&mut hw).unwrap()

// Both should produce the same output (idempotency)
assert!((result1.position()[0] - result2.position()[0]).abs() < 1e-10)
assert!((result1.velocity()[0] - result2.velocity()[0]).abs() < 1e-10)
```

### Test 4 — PIPA saturation

```
// Saturated PIPA: one axis exceeds PIPA_MAX_COUNTS (6398).
hw.imu().inject_pipa([6500i16, 0, 0])   // 6500 > 6398
hw.imu().set_refsmmat(IDENTITY_MAT3)
let initial_state = avg_g.state().clone()

result = avg_g.cycle(&mut hw)

// Should return Err(AvgGError::PipaSaturated)
assert_eq!(result, Err(AvgGError::PipaSaturated))
// State should be unchanged
assert_eq!(avg_g.state().position(), initial_state.position())
assert_eq!(avg_g.state().velocity(), initial_state.velocity())
// Alarm 00205 should have been raised
assert!(hw.alarm().last_code() == Some(AlarmCode::PipaSaturated))
```

---

## Notes and Ambiguities

1. **KPIP1 value**: The AGC constant `KPIP1 2DEC 0.074880` is in the AGC
   fixed-point system (B-7 m/cs per count), not SI. Converting:
   0.074880 (in B-7 m/cs) × 2^-7 × 100 cs/s = 0.05850 m/s/count.
   The Rust constant `PIPA_SCALE = 0.0585 m/s/count` is correct.
   The slightly different value in the header comment (0.074880 ≠ 0.0585) is
   explained by the B-7 scale factor; both refer to the same physical quantity.

2. **GENTRAN bulk copy**: The AGC uses `GENTRAN` (a word-copy primitive) to move
   `OCT31` words (= 25 decimal, which covers RN1+VN1+GDT1/2+GOBL1/2+PIPTIME1)
   from the "1" buffer to the permanent storage. The Rust equivalent is a simple
   `self.state = new_state`. The `OCT31` count of 25 words is documented for
   reference (6 + 6 + 6 + 6 + 1 = 25 DP registers = the header says `CAF OCT31`
   but the comment says `RN1,VN1,GOT102,GOBL1/2,PIPTIME1` which is 14 DP pairs
   = 28 words; the discrepancy may be the low-order PIPTIME only needing 2 words
   vs 6). This is an `AMBIGUITY`: the exact word count in GENTRAN (`OCT31` = 25)
   does not obviously match the named fields. The Rust port simply assigns all
   five fields atomically, avoiding the word-count issue.

3. **GOBL/2 vs GDT/2**: CALCGRAV separately computes `GOBL1/2` (the oblateness
   contribution) and adds it to `GDT1/2`. The Rust port folds both into
   `gdt_over_2` returned by `earth_gravity` (which includes J2). This is an
   `APPROXIMATE` deviation if the validator finds the oblateness term tracked
   separately matters for restart recovery (it was stored in separate erasable
   registers `GOBL/2 EQUALS GDT/2 +6`).

4. **1/PIPA compensation**: The AGC calls the `1/PIPA` subroutine to compensate
   for PIPA scale-factor errors. This is beyond the scope of M2. The Rust port
   applies only the fixed `PIPA_SCALE = 0.0585 m/s/count` without individual-axis
   compensation. Flag as `APPROXIMATE` deviation.

5. **DELVREF**: The AGC stores the rotated delta-V in a separate erasable vector
   `DELVREF` (scaled at B-7 m/cs). The Rust port computes this as a local
   variable `dv_eci` in the `cycle` method without persisting it between cycles.
   If future modules need DELVREF (e.g., the thrust monitor DVMON), add it as a
   field of `AverageG` at that point.
