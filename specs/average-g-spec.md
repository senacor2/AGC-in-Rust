# Specification: `services/average_g` Module — SERVICER (Average-G)

**Status**: Approved for implementation
**Module path**: `agc-core/src/services/average_g.rs`
**Architecture reference**: `docs/architecture.md` §7.4 "SERVICER (Average-G)"
**Integration reference**: `specs/integration-spec.md` §3 (`average_g_step` algorithm)
**State-vector reference**: `specs/state-vector-spec.md` §5.1 (SERVICER cycle), §2.4 (REFSMMAT)
**Gravity reference**: `specs/gravity-spec.md` §1 (calling convention)
**HAL reference**: `specs/hal-spec.md` §8 (`Imu` trait, `read_pipa`), §6 (`Timers`, `arm_t3`)
**Executive reference**: `specs/executive-spec.md` §4.6 (`Waitlist::schedule`), §5.3 (T3RUPT handler)
**AGC source files**:
- `Comanche055/AVERAGE_G_INTEGRATOR.agc` — SERVICER entry point, PIPA read, Average-G loop
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — PIPAX/Y/Z addresses, NBDX/Y/Z constants
- `Comanche055/INTEGRATION_INITIALIZATION.agc` — body selection, PIPA scale factor
**Spec checklist**: `specs/README.md` — all items satisfied (see §10)

---

## 1. Purpose and Scope

`services::average_g` implements the SERVICER — the 2-second repeating navigation
task that is the heartbeat of powered-flight guidance in the Apollo Guidance Computer.
It is the only code that reads PIPA (Pulse Integrating Pendulous Accelerometer) counts
from the IMU, compensates them for calibration errors, rotates the resulting delta-V
into the inertial reference frame via REFSMMAT, and drives the `average_g_step`
integrator in `navigation::integration`.

The SERVICER is not a major mode. It is a Waitlist task established by programs that
require active navigation (P11, P20, P40, and the entry programs P61–P67) and
cancelled when navigation is not needed (P00 idle, P06 power-down, etc.).

### What this module provides

- `start_servicer` — schedule the first SERVICER Waitlist task.
- `stop_servicer` — cancel an active SERVICER (set a flag that suppresses reschedule).
- `servicer_task` — the private 2-second task function called by the Waitlist.
- PIPA compensation constants (`PIPA_SCALE`, `PipaCalibration`) and the
  compensation pipeline.

### What this module does NOT provide

- The Average-G integration algorithm. That is in `navigation::integration::average_g_step`.
  By the time `average_g_step` is called, all PIPA processing is complete and the
  delta-V is already expressed in the inertial frame.
- Gravity computation. That is in `navigation::gravity`.
- Moon ephemeris. That is in `navigation::planetary::moon_position`.
- DSKY display formatting. The SERVICER calls `services::display` to request a
  display update; it does not format words directly.
- Cross-product steering (P40 TVC). The SERVICER invokes a program-specific exit
  hook via `AgcState::servicer_exit`; the TVC computation itself lives in `guidance`.
- SOI (sphere-of-influence) transition detection. That is deferred to `navigation::integration`.

---

## 2. AGC Background

### 2.1 The SERVICER in Comanche055

The SERVICER (sometimes called AVERAGE-G or AVERAGEG in the assembly listing) ran
as a repeating Waitlist task on a strict 2-second cycle derived from the T3RUPT
interrupt chain. The task was established by the first navigation-requiring program
to run after a FRESH START or restart, and self-rescheduled at the conclusion of
every cycle using another `WAITLIST` call for 200 centiseconds.

The name "Average-G" refers to the trapezoidal gravity-averaging scheme used
inside the integration step: gravity is evaluated at both the beginning and the
predicted end of the 2-second interval, and the average is used for the velocity
update. This makes the method second-order accurate for near-circular orbits.
See `specs/integration-spec.md` §2.1 for the full algorithm description.

### 2.2 PIPA Counts and Their Physical Meaning

Each PIPA (Pulse Integrating Pendulous Accelerometer) axis accumulates pulse
counts in a dedicated hardware counter cell:

| AGC symbol | Octal address | Axis |
|------------|---------------|------|
| `PIPAX`    | octal 0125    | IMU X (stable-member frame) |
| `PIPAY`    | octal 0126    | IMU Y (stable-member frame) |
| `PIPAZ`    | octal 0127    | IMU Z (stable-member frame) |

Each count represents a velocity increment of approximately **0.0585 m/s** on the
real hardware (the exact value is mission-calibrated and stored in the constant
`1/PIPADT` in the AGC erasable memory). Reading a counter cell is **destructive**:
the hardware resets the cell to zero on readout. The SERVICER must be the only
caller of `hw.imu().read_pipa()`.

> AGC source: `docs/AGC Symbolic Listing.md` §IID, counter cell table; counter
> cells 0125–0127 (PIPAX/Y/Z). The HAL specification confirms: "Each count
> represents a velocity increment of approximately 0.0585 m/s on the real
> hardware." See `specs/hal-spec.md` §8.2.

### 2.3 PIPA Compensation

The raw PIPA counts are not pure inertial velocity increments. They contain several
calibration errors that must be removed before the delta-V can be used in navigation:

1. **Bias (zero-offset drift)**: Each PIPA has a small constant offset, expressed
   as drift counts per unit time. In Comanche055 these are stored as `NBDX`, `NBDY`,
   `NBDZ` (Non-Bias-Drift for X/Y/Z). They are pre-subtracted from the raw counts.

2. **Scale factor**: The mapping from counts to m/s is `1/PIPADT` (the inverse of
   the PIPA delta-time constant), stored in AGC erasable as a double-precision
   fixed-point value. On the real hardware this is approximately 0.0585 m/s/count
   but the precise value is used for navigation accuracy.

3. **Misalignment**: The three PIPA axes are not perfectly orthogonal. A 3×3
   compensation matrix (small off-diagonal terms) corrects for this. The diagonal
   terms are 1.0 (already absorbed into the scale factor); the off-diagonal terms
   are small (typically < 1×10⁻³).

The Rust port stores these calibration constants in a `PipaCalibration` struct
field of `AgcState`. See §4.3.

### 2.4 REFSMMAT Rotation

After compensation, the delta-V is expressed in the stable-member (IMU platform)
frame. To integrate it into the state vector, it must be expressed in the
inertial frame (ECI or MCI). The rotation is:

```
delta_v_inertial = linalg::mxv(refsmmat, delta_v_platform)
```

REFSMMAT (`AgcState::refsmmat`) is a 3×3 orthonormal rotation matrix maintained by
the IMU alignment programs P51/P52. See `specs/state-vector-spec.md` §2.4.

### 2.5 Restart Protection

In Comanche055, the SERVICER used **restart Group 2** (PHASCHNG group 2) to
protect against hardware restarts mid-cycle. The phase values used are:

| Phase | Meaning | Restart action |
|-------|---------|----------------|
| `Phase(0)` | IDLE — SERVICER not running | No restart action |
| `Phase(1)` | Waitlist task active (before first cycle completes) | Re-schedule `servicer_task` as Waitlist task at 200 cs |
| `Phase(-1)` | Mid-update (state vector partially written) | Discard partial update; reschedule from top |

The phase is set to `Phase(1)` before the state vector is written (step 6 of the
cycle), and cleared to `Phase(0)` after self-rescheduling (step 8). This ensures
that if a restart occurs while the state vector is being updated, the restart
handler re-queues the task rather than leaving corrupted navigation data in place.

> Source: `specs/executive-spec.md` §2.3; `docs/architecture.md` §6 (restart protection
> pattern); `Comanche055/AVERAGE_G_INTEGRATOR.agc` PHASCHNG calls around the RN/VN
> store sequence.

### 2.6 Programs That Start and Stop the SERVICER

**Programs that call `start_servicer`** (schedule the first SERVICER task):

| Program | AGC purpose | Starts SERVICER because |
|---------|-------------|------------------------|
| P11 | Earth Orbit Insertion monitor | Tracks powered ascent; needs real-time delta-V integration |
| P15 | TLI monitor | Tracks TLI burn; needs delta-V integration for departure hyperbola |
| P20 | Rendezvous navigation | Requires live state vector propagation for target tracking |
| P40 | SPS thrusting (LOI, TEI, etc.) | Primary burn program; needs navigation and TVC steering updates |
| P41 | RCS thrusting | Small delta-V maneuvers; needs navigation |
| P47 | Thrust monitoring | Monitors ongoing burn; SERVICER already started by P40/P41 |
| P61–P67 | Entry programs | CM entry guidance requires continuous state vector updates |

**Programs that call `stop_servicer`** (cancel the SERVICER):

| Program | Why it stops the SERVICER |
|---------|--------------------------|
| P00 | CMC idling; no active navigation required |
| P06 | Power-down; all active tasks ceased |
| Any program termination returning to P00 | SERVICER is not needed during coast/idle |

During coasting (no active thrusting, but navigation is monitored), some programs
keep the SERVICER running at reduced cadence or stop it and rely on `propagate_coast`
for on-demand state vector updates. P20/P22/P23 (navigation without thrusting)
may or may not start the SERVICER depending on whether the IMU is in use.

The architectural design keeps the SERVICER active whenever an IMU-based navigation
state is being maintained in real time. It is not needed during coast if position
updates come only from P23 sightings or ground uplink.

---

## 3. Calibration Constants

### 3.1 `PipaCalibration` Struct

The PIPA compensation parameters are stored in `AgcState` as a `PipaCalibration`
struct. This struct is initialized at FRESH START (nominal values), overwritten
when updated calibration constants are uplinked from Mission Control, and
preserved across RESTART.

```rust
/// PIPA (accelerometer) calibration constants.
///
/// In Comanche055 these are stored in erasable memory (E1 bank) and loaded
/// from the fixed-memory constant tables at program start or updated by uplink.
///
/// AGC source: Comanche055/AVERAGE_G_INTEGRATOR.agc — NBDX/NBDY/NBDZ and
/// 1/PIPADT constant entries.
#[derive(Clone, Copy, Debug)]
pub struct PipaCalibration {
    /// PIPA scale factor: metres per second per raw count.
    ///
    /// Nominal value ≈ 0.0585 m/s/count.
    /// AGC name: 1/PIPADT (inverse of PIPA delta-time constant).
    /// Stored as double-precision fixed-point in erasable; converted to f64 here.
    pub scale: f64,

    /// Bias (zero-offset drift) in counts per 2-second interval for each axis.
    ///
    /// AGC names: NBDX (index 0), NBDY (index 1), NBDZ (index 2).
    /// These are subtracted from the raw counts before scaling.
    /// Units: counts per 2-second SERVICER interval (not counts/second).
    /// Nominal value: 0 (perfectly calibrated instrument).
    /// Typical flight value: small integer, order 1–5 counts/interval.
    pub bias: [i16; 3],

    /// PIPA misalignment compensation matrix.
    ///
    /// A 3×3 matrix applied after bias removal and scale-factor multiplication.
    /// The diagonal is 1.0; off-diagonal elements are small (< 1×10⁻³ rad).
    /// For a perfectly aligned instrument this is the identity matrix.
    /// AGC source: AVERAGE_G_INTEGRATOR.agc misalignment table entries.
    pub misalignment: [[f64; 3]; 3],
}

impl PipaCalibration {
    /// Nominal (uncalibrated) constants. Used at FRESH START.
    pub const NOMINAL: Self = Self {
        scale: 0.0585,          // m/s per count, approximate
        bias: [0, 0, 0],        // no bias correction
        misalignment: [         // identity (no misalignment correction)
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    };
}
```

The `PipaCalibration` struct is added as a field of `AgcState`:

```rust
// In AgcState (agc-core/src/lib.rs):
pub pipa_cal: PipaCalibration,
```

Initialized to `PipaCalibration::NOMINAL` in `AgcState::new()`.

### 3.2 SERVICER Active Flag

A flag bit in `AgcState::flagwords` indicates whether the SERVICER is currently
active. This flag is checked at the end of each cycle: if it is clear, the SERVICER
does not reschedule itself.

```rust
/// Bit position in flagwords[0] that indicates the SERVICER is active.
/// Set by `start_servicer`; cleared by `stop_servicer`.
/// AGC correspondence: AVEGFLAG or equivalent bit in FLAGWRD0.
pub const SERVICER_ACTIVE_BIT: u8 = 0;  // bit 0 of flagwords[0]
```

Helper accessors (inline functions, not trait methods):

```rust
fn is_servicer_active(state: &AgcState) -> bool {
    (state.flagwords[0] >> SERVICER_ACTIVE_BIT) & 1 != 0
}

fn set_servicer_active(state: &mut AgcState, active: bool) {
    if active {
        state.flagwords[0] |= 1 << SERVICER_ACTIVE_BIT;
    } else {
        state.flagwords[0] &= !(1 << SERVICER_ACTIVE_BIT);
    }
}
```

### 3.3 SERVICER Exit Hook

Programs that need computation after each SERVICER cycle (e.g., P40 cross-product
steering update) register a function pointer in `AgcState`. The SERVICER calls
this hook after the state vector is written back and before self-rescheduling.

```rust
// In AgcState (agc-core/src/lib.rs):
/// Optional program-specific callback called at the end of each SERVICER cycle.
///
/// P40 sets this to `guidance::tvc::cross_product_steering_update`.
/// P00 and programs that do not need a SERVICER exit set this to `None`.
pub servicer_exit: Option<fn(&mut AgcState)>,
```

The exit hook must be short (Waitlist task budget applies). If the hook needs to
perform long computation, it should call `state.executive.create_job(...)` to
schedule an Executive job, then return.

---

## 4. Function Specifications

### 4.1 `start_servicer`

```rust
/// Schedule the first SERVICER (Average-G) Waitlist task.
///
/// Called by navigation programs (P11, P20, P40, P61–P67) when they begin
/// active flight navigation. Safe to call when the SERVICER is already running:
/// the active-flag check prevents double-scheduling.
///
/// AGC correspondence: the WAITLIST call for "SERVICER in 2 seconds" made
/// at the entry points of the navigation programs in Comanche055.
///
/// # Preconditions
/// - `state` must be a valid `AgcState`.
/// - `hw` must be a fully initialised `AgcHardware` implementation.
/// - The IMU must be powered and aligned (callers are responsible for this
///   precondition; `start_servicer` does not check IMU status).
///
/// # Postconditions
/// - `is_servicer_active(state)` returns `true`.
/// - `state.restart.phases[GROUP_2]` is `Phase(1)` (restart-protected).
/// - The Waitlist contains a pending `servicer_task` entry with delta = 200 cs.
/// - TIME3 is armed (via `state.waitlist.schedule` → caller calls `arm_t3`).
///
/// # Side effects
/// - Sets `SERVICER_ACTIVE_BIT` in `state.flagwords[0]`.
/// - Sets restart group 2 phase to `Phase(1)`.
/// - Inserts `servicer_task` into the Waitlist at 200 centiseconds.
pub fn start_servicer<H: AgcHardware>(state: &mut AgcState, hw: &mut H)
```

**Implementation**:

```
1. If is_servicer_active(state): return (already running; idempotent).
2. set_servicer_active(state, true).
3. state.restart.set_phase(GROUP_2, Phase(1)).
4. match state.waitlist.schedule(200, servicer_task):
   ScheduleResult::OkReloadT3(delta) => hw.timers().arm_t3(delta),
   ScheduleResult::Ok               => { /* T3 already armed for earlier task */ }
   ScheduleResult::Full             => { services::alarm::raise(state, hw, 0x1211); }
```

The GROUP_2 constant is defined in `executive/restart.rs`:
```rust
pub const GROUP_2: usize = 1;  // index into RestartProtection::phases[]; 0-based
```

### 4.2 `stop_servicer`

```rust
/// Cancel the SERVICER (Average-G) task.
///
/// Sets the stop flag so that the next time `servicer_task` runs, it does not
/// reschedule itself. Any currently-pending Waitlist entry for `servicer_task`
/// will fire once more, then terminate gracefully.
///
/// This is the preferred approach: the Waitlist has no cancellation operation,
/// so we use a flag to prevent the final reschedule. The alternative — removing
/// the entry from the Waitlist by scanning the task function pointer — is
/// fragile and not required by the original AGC design.
///
/// AGC correspondence: clearing the AVEGFLAG or equivalent bit in program
/// exit sequences in Comanche055.
///
/// # Preconditions
/// - `state` must be a valid `AgcState`.
/// - Safe to call when the SERVICER is not running (idempotent).
///
/// # Postconditions
/// - `is_servicer_active(state)` returns `false`.
/// - `state.restart.phases[GROUP_2]` is `Phase(0)` (IDLE).
/// - The currently-queued `servicer_task` Waitlist entry (if any) will execute
///   one final time, observe the inactive flag, and terminate without rescheduling.
///
/// # Side effects
/// - Clears `SERVICER_ACTIVE_BIT` in `state.flagwords[0]`.
/// - Clears restart group 2 phase to `Phase(0)`.
/// - Clears `state.servicer_exit` to `None`.
pub fn stop_servicer(state: &mut AgcState)
```

**Implementation**:

```
1. set_servicer_active(state, false).
2. state.restart.set_phase(GROUP_2, Phase(0)).
3. state.servicer_exit = None.
```

### 4.3 `servicer_task`

```rust
/// The 2-second SERVICER Waitlist task.
///
/// This function is the heart of powered-flight navigation. It must be fast
/// enough to complete within the T3RUPT task timing budget (≤ 5 ms per
/// `specs/hal-spec.md` §5.4). The heavy computation (gravity evaluation,
/// integration) is delegated to `navigation::integration::average_g_step`,
/// which operates on f64 values and is called directly.
///
/// This function is private to the module (`pub(crate)` at most).
/// It is registered as a function pointer `fn(&mut AgcState)` in the Waitlist
/// and therefore must match the `fn(&mut AgcState)` signature exactly.
///
/// AGC correspondence: SERVICER entry point in
/// `Comanche055/AVERAGE_G_INTEGRATOR.agc`.
///
/// # Note on hardware parameter
/// Waitlist tasks in this port have the signature `fn(&mut AgcState)` with no
/// hardware parameter (see `specs/executive-spec.md` §3.4). Access to hardware
/// for self-rescheduling is handled by storing the hardware reference in
/// `AgcState::hw_ref` (a raw pointer, set before task dispatch by the T3RUPT
/// handler) or by the alternative design documented in §5 below.
fn servicer_task(state: &mut AgcState)
```

See §5 for the detailed hardware-access design decision.

---

## 5. The 2-Second Cycle — Step-by-Step Algorithm

The `servicer_task` function executes the following steps in order. All steps
must complete before the function returns. If any step would require more than
the T3RUPT task budget (5 ms), that step should spawn an Executive job for the
heavy work and return quickly from the task.

### Step 1 — Read PIPA Counts (destructive)

```rust
let raw_counts: [i16; 3] = hw.imu().read_pipa();
```

This is a destructive read: the PIPA counter cells are zeroed by the hardware
on readout. The returned values are the accumulated velocity pulses since the
last call (nominally the previous SERVICER cycle 2 seconds ago).

**Error condition**: If `raw_counts[i].abs() == i16::MAX` for any axis, the
PIPA counter has overflowed (the SERVICER was delayed more than ~5 minutes for
a 0.0585 m/s/count scale factor and 32767 counts). Raise alarm and do not
use the count for navigation. See §9.1.

### Step 2 — Apply Bias Correction

```rust
let cal = &state.pipa_cal;
let biased: [i32; 3] = [
    raw_counts[0] as i32 - cal.bias[0] as i32,
    raw_counts[1] as i32 - cal.bias[1] as i32,
    raw_counts[2] as i32 - cal.bias[2] as i32,
];
```

The bias values `cal.bias[i]` (AGC names: NBDX, NBDY, NBDZ) are the
pre-measured zero-offset drift counts per 2-second interval. Subtraction is
done in `i32` to avoid overflow.

**Note on NBDX/NBDY/NBDZ**: In the real AGC, these constants were measured
before launch and stored in erasable memory. They represent instrument drift
(acceleration bias integrated over 2 seconds into equivalent count error).
Typical values were small integers (a few counts per cycle, corresponding to
a few tenths of m/s per cycle of uncompensated drift). The exact values varied
per flight unit.

### Step 3 — Apply Scale Factor

```rust
let scaled: [f64; 3] = [
    biased[0] as f64 * cal.scale,
    biased[1] as f64 * cal.scale,
    biased[2] as f64 * cal.scale,
];
```

`cal.scale` is `1/PIPADT` ≈ 0.0585 m/s/count. The result is delta-V in m/s
in the stable-member (platform) frame, before misalignment correction.

### Step 4 — Apply Misalignment Correction

```rust
let delta_v_platform: Vec3 = linalg::mxv(cal.misalignment, scaled);
```

`linalg::mxv` is the 3×3 matrix-vector multiply from `math::linalg`. For the
nominal (identity) misalignment matrix this is a no-op, but the call must
always be made so that calibrated values take effect automatically.

### Step 5 — Rotate to Inertial Frame via REFSMMAT

```rust
let delta_v_inertial: Vec3 = linalg::mxv(state.refsmmat, delta_v_platform);
```

REFSMMAT maps stable-member frame vectors to the current inertial frame
(ECI or MCI, depending on `state.csm_state.frame`). See `specs/state-vector-spec.md`
§2.4. The result is the inertially-resolved velocity increment in m/s, ready
for direct addition to the state vector velocity.

### Step 6 — Integrate the State Vector

```rust
// Get Moon position for third-body perturbation
let moon_pos: Vec3 = navigation::planetary::moon_position(state.csm_state.epoch);

// Integrate: one Average-G step (2-second interval)
let new_sv = navigation::integration::average_g_step(
    state.csm_state,
    delta_v_inertial,
    2.0,           // dt = 2 seconds (the SERVICER cycle period)
    moon_pos,
);
```

The `average_g_step` function is the authoritative integration algorithm.
See `specs/integration-spec.md` §3 for the complete algorithm specification.
In summary it:
1. Computes gravity `g0` at the current position.
2. Estimates midpoint velocity: `v_half = sv.velocity + delta_v + g0 * (dt/2)`.
3. Computes new position: `new_r = sv.position + v_half * dt`.
4. Computes gravity `g1` at the new position.
5. Computes new velocity (trapezoidal average): `new_v = sv.velocity + delta_v + (g0+g1) * (dt/2)`.
6. Advances epoch: `new_epoch = sv.epoch + Met::from_centiseconds(200)`.

**Restart protection around the write-back**: Before writing the new state
vector, set the restart phase to `Phase(-1)` (mid-update guard). After
writing back, set it to `Phase(1)` (ready for next cycle).

```rust
// Arm restart protection for state-vector write-back
state.restart.set_phase(GROUP_2, Phase(-1));

// Write updated state vector
state.csm_state = new_sv;

// Advance mission time
state.time = new_sv.epoch;

// Restore phase to "task active, cycle complete"
state.restart.set_phase(GROUP_2, Phase(1));
```

### Step 7 — Update Display

The SERVICER is responsible for refreshing the DSKY display with the current
navigation state. The exact verb/noun depends on the active program:

| Active program | Typical display | Verb | Noun |
|----------------|-----------------|------|------|
| P11 | Velocity / altitude | V06 | N62 |
| P20 | Relative state | V16 | N63 |
| P40 | Delta-V remaining | V16 | N85 |
| Entry (P61–P67) | Entry range / velocity | V16 | N60 |

In the Rust port this is dispatched by calling `services::display::request_update(state)`
which causes the T4RUPT cycle to refresh the current major-mode display on the
next 120 ms display cycle. The SERVICER does not write DSKY rows directly.

```rust
services::display::request_update(state);
```

### Step 8 — Call Program-Specific Exit Hook

```rust
if let Some(exit_fn) = state.servicer_exit {
    exit_fn(state);
}
```

The exit hook is set by the active program when it starts the SERVICER. For P40
SPS burns, this calls `guidance::tvc::cross_product_steering_update(state)` to
compute the gimbal error angle for TVC. The hook must run to completion quickly
(same timing budget as the task).

### Step 9 — Self-Reschedule

```rust
if is_servicer_active(state) {
    match state.waitlist.schedule(200, servicer_task) {
        ScheduleResult::OkReloadT3(delta) => hw.timers().arm_t3(delta),
        ScheduleResult::Ok               => { /* earlier task exists; T3 already set */ }
        ScheduleResult::Full             => {
            services::alarm::raise(state, hw, 0x1211);
            // Do not reschedule; SERVICER is now stopped.
            set_servicer_active(state, false);
            state.restart.set_phase(GROUP_2, Phase(0));
        }
    }
} else {
    // stop_servicer was called during this cycle; do not reschedule.
    state.restart.set_phase(GROUP_2, Phase(0));
}
```

**Reschedule period**: 200 centiseconds = 2.000 seconds exactly.

The SERVICER is intentionally scheduled for a fixed 200 cs from now at each
cycle end, not from the cycle start time. This means that processing jitter
accumulates; a cycle that takes 3 ms will cause the next cycle to start 3 ms
late. This is acceptable because the PIPA counters accumulate continuously and
the full count since the last read is used regardless of when the read occurs.
The 2-second dt value passed to `average_g_step` is the nominal cycle period
and does not need to be adjusted for small processing jitter (< 1 ms is typical
for the T3RUPT dispatch overhead).

---

## 6. Hardware Access Design

### 6.1 Challenge: Waitlist Tasks and the Hardware Parameter

`Waitlist::schedule` accepts `fn(&mut AgcState)` — a function pointer with only
one parameter. The SERVICER needs `&mut AgcHardware` for:
- Step 1: `hw.imu().read_pipa()`.
- Step 9: `hw.timers().arm_t3(...)` (via `waitlist.schedule`).

This tension is a fundamental architectural constraint. Two resolution strategies
are specified; the implementation must choose one and document the choice in the
module's doc comment.

### 6.2 Strategy A — Hardware Reference Stored in AgcState (Recommended)

The T3RUPT handler, which calls `Waitlist::dispatch`, has access to both `state`
and `hw`. Before calling `dispatch`, it writes a raw pointer to `hw` into a
designated field of `AgcState`:

```rust
// In AgcState:
pub hw_ptr: *mut dyn AgcHardware_erased,  // set by T3RUPT handler; NULL outside interrupt
```

Because the AGC is single-threaded (only one interrupt active at a time on the
original hardware; single-threaded cooperative on the Rust port), this raw pointer
is safe to dereference inside `servicer_task` as long as the lifetime contract is
honoured. The T3RUPT handler sets `hw_ptr` before calling `dispatch` and clears
it immediately after `dispatch` returns.

This is the pattern most consistent with the original AGC, where the interrupt
handler pushed a "hardware context" onto the erasable stack before calling the
task.

**Implementation note**: Because `AgcHardware` is a generic trait (not an object-
safe trait-object in the current design), using a raw pointer requires an erasure
adapter. The alternative is to make `AgcHardware` object-safe or to use a
concrete hardware type. The implementation agent must resolve this with the
architecture team before implementation.

### 6.3 Strategy B — Split Task into Two Parts

The SERVICER task function itself (`servicer_task`) is only responsible for the
navigation computation (steps 2–6) and does not call hardware directly. Steps 1
and 9 are performed by a thin wrapper registered in the Waitlist:

```rust
fn servicer_wrapper(state: &mut AgcState) {
    // Step 1: read PIPA — requires hw access
    // Strategy B defers this: the T3RUPT handler calls read_pipa
    // and stores the result in a staging field of AgcState before
    // dispatching servicer_task.
    servicer_task(state);
    // Step 9: reschedule — the T3RUPT handler or a post-dispatch hook calls arm_t3
}
```

A staging field in `AgcState` holds the raw PIPA counts:

```rust
// In AgcState:
pub pipa_staging: [i16; 3],  // written by T3RUPT before dispatching servicer_task
```

This strategy avoids the raw-pointer complexity of Strategy A at the cost of
storing intermediate state in `AgcState`.

**Recommendation**: Strategy A (or a variant using a concrete type parameter
rather than a trait object) is preferred because it more closely mirrors the
AGC's interrupt handler design and avoids polluting `AgcState` with staging
fields. The implementation agent should validate the object-safety constraint
and choose accordingly, documenting the decision in the code.

---

## 7. Data Flow Summary

### 7.1 Inputs to `servicer_task`

| Source | Field/Method | Units | Description |
|--------|-------------|-------|-------------|
| `hw.imu().read_pipa()` | `[i16; 3]` | counts | Raw PIPA delta-V counts (destructive read) |
| `state.pipa_cal.bias` | `[i16; 3]` | counts/interval | PIPA bias (NBDX/NBDY/NBDZ) |
| `state.pipa_cal.scale` | `f64` | m/s/count | PIPA scale factor (1/PIPADT) |
| `state.pipa_cal.misalignment` | `[[f64;3];3]` | dimensionless | PIPA axis misalignment matrix |
| `state.refsmmat` | `Mat3x3` | dimensionless | REFSMMAT (platform → inertial rotation) |
| `state.csm_state` | `StateVector` | m, m/s, cs | Current CSM navigation state |
| `navigation::planetary::moon_position(t)` | `Vec3` | m | Moon position for third-body gravity |

### 7.2 Outputs of `servicer_task`

| Destination | Field | Description |
|------------|-------|-------------|
| `state.csm_state` | `StateVector` | Updated position, velocity, epoch |
| `state.time` | `Met` | Synchronised with `csm_state.epoch` |
| `state.waitlist` | (internal) | Next SERVICER task entry at +200 cs |
| `state.restart.phases[GROUP_2]` | `Phase` | Updated for restart protection |
| DSKY | (via display service) | Refreshed navigation display |
| TVC | (via servicer_exit hook) | Updated steering commands (P40 only) |

### 7.3 State Not Modified by the SERVICER

The SERVICER does not modify:
- `state.refsmmat` (set by P51/P52 only)
- `state.target_state` (set by P20/P22/P23 or uplink)
- `state.major_mode`
- `state.dap_state`, `state.tvc_state`
- Any alarm state (except potentially alarm raises on PIPA overflow)
- `state.flagwords` (except the `SERVICER_ACTIVE_BIT` managed by start/stop functions)

---

## 8. AGC Erasable Variable Mapping

### 8.1 PIPA Counter Cells

| AGC symbol | Octal address | Rust access | Description |
|------------|---------------|-------------|-------------|
| `PIPAX`    | 0125          | `hw.imu().read_pipa()[0]` | PIPA X axis counter (destructive read) |
| `PIPAY`    | 0126          | `hw.imu().read_pipa()[1]` | PIPA Y axis counter (destructive read) |
| `PIPAZ`    | 0127          | `hw.imu().read_pipa()[2]` | PIPA Z axis counter (destructive read) |

Scale: 1 count ≈ 0.0585 m/s (mission-calibrated). Hardware type: `i16`.

> AGC source: `docs/AGC Symbolic Listing.md` §IID, counter cell table at octal
> addresses 0125–0127.

### 8.2 Compensation Constants

| AGC symbol | Location | Scale | Rust field | Description |
|------------|----------|-------|------------|-------------|
| `NBDX`     | Erasable (E1 bank) | counts/interval | `pipa_cal.bias[0]` | X bias drift |
| `NBDY`     | Erasable (E1 bank) | counts/interval | `pipa_cal.bias[1]` | Y bias drift |
| `NBDZ`     | Erasable (E1 bank) | counts/interval | `pipa_cal.bias[2]` | Z bias drift |
| `1/PIPADT` | Erasable (E1 bank) | m/s per count (B-7 DP) | `pipa_cal.scale` | PIPA scale factor |
| PIPASR matrix | Fixed memory | dimensionless | `pipa_cal.misalignment` | 3×3 misalignment |

> AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc` — erasable assignments for
> NBDX/NBDY/NBDZ and the `1/PIPADT` constant table entry. The misalignment
> matrix (PIPASR) is from the fixed-memory constant tables in
> `INTEGRATION_INITIALIZATION.agc`.

### 8.3 State Vector Outputs

| AGC symbol | Octal address | Scale factor | Rust field | Description |
|------------|---------------|-------------|------------|-------------|
| `RN`       | 0306–0313     | B+28 m DP   | `csm_state.position` | Updated CSM position |
| `VN`       | 0314–0321     | B+7 m/s DP  | `csm_state.velocity` | Updated CSM velocity |
| `TEPHEM`   | 0340–0342     | B-28 s DP   | `csm_state.epoch` / `state.time` | Epoch of RN/VN |

---

## 9. Error Handling and Preconditions

### 9.1 PIPA Overflow

**Trigger**: `|raw_counts[i]| == i16::MAX` (32767) for any axis.

**Cause**: The PIPA counter overflowed because the SERVICER was delayed more
than ~32767 × 0.0585 m/s ≈ 1917 m/s worth of acceleration accumulated in the
counter (roughly 5 minutes at 1g). This is physically impossible in normal
powered flight (SPS burns are limited to ~10 minutes at most), but could occur
if the SERVICER was not scheduled (e.g., after a long coast with the SERVICER
inadvertently active).

**Required action**:
1. Raise program alarm (implementation-defined code, suggest 0x0105 for PIPA
   overflow, or use the nearest Comanche055 ISS-warning equivalent).
2. Set `hw.dsky().set_lamp(Lamp::NoAtt, true)` to warn crew.
3. Do **not** use the overflowed count in navigation. Replace with zero for
   the current cycle to avoid state-vector corruption.
4. Continue SERVICER scheduling (the overflow was a single-cycle event).

### 9.2 Waitlist Full (Alarm 1211)

If `state.waitlist.schedule(200, servicer_task)` returns `ScheduleResult::Full`,
the SERVICER cannot reschedule itself. This stops the navigation cycle.

**Required action**:
1. Raise alarm 1211 via `services::alarm::raise`.
2. Clear the `SERVICER_ACTIVE_BIT`.
3. Clear the restart phase to `Phase(0)`.
4. The navigation state will no longer be updated; crew must be notified via
   the DSKY alarm display.

### 9.3 Invalid State Vector after Integration

If `average_g_step` returns a `StateVector` with non-finite position or
velocity (NaN or infinity), this is a logic error in the integrator or upstream
data corruption.

**Required action**:
1. In debug builds: `debug_assert!` inside `average_g_step` catches this before
   return.
2. In release builds: the SERVICER checks `norm(new_sv.position).is_finite()`
   before writing back. If not finite, raise an alarm and do not write back.
3. Do not stop the SERVICER; attempt to recover with the unchanged old state
   vector on the next cycle.

### 9.4 Preconditions for `servicer_task`

- The IMU must be powered and the stable platform must not be caged (not
  checked inside `servicer_task`; the calling program is responsible).
- `state.refsmmat` must be a valid rotation matrix (orthonormal, determinant +1).
  The SERVICER does not validate this; P51/P52 is responsible.
- `state.csm_state` must satisfy the `StateVector` invariants (INV-1 through
  INV-3) from `specs/state-vector-spec.md` §7.

---

## 10. Interaction with Other Modules

| Module | Interaction |
|--------|-------------|
| `navigation::integration::average_g_step` | Called in Step 6; the core integration |
| `navigation::planetary::moon_position` | Called before Step 6 for third-body data |
| `navigation::gravity` | Called indirectly by `average_g_step` |
| `math::linalg::mxv` | Called in Steps 4 and 5 for matrix-vector multiply |
| `hal::Imu::read_pipa` | Called in Step 1; destructive PIPA read |
| `hal::Timers::arm_t3` | Called in Step 9 (via waitlist reschedule) |
| `executive::Waitlist::schedule` | Called in Step 9; self-reschedule |
| `executive::RestartProtection::set_phase` | Called before/after Step 6 write-back |
| `services::display::request_update` | Called in Step 7; triggers T4RUPT display refresh |
| `services::alarm::raise` | Called on PIPA overflow or Waitlist full |
| `guidance::tvc` | Called via `state.servicer_exit` hook (P40 only) |

---

## 11. Timing Budget

The SERVICER is a Waitlist task dispatched by T3RUPT. The T3RUPT handler must
complete within 5 ms (see `specs/hal-spec.md` §5.4, `specs/executive-spec.md` §5.3).

Estimated cycle time for the Rust port on a Cortex-M7 at 216 MHz:

| Step | Operation | Estimated cost |
|------|-----------|----------------|
| 1 | `read_pipa` (SPI read) | 50–200 µs (hardware-dependent) |
| 2–4 | Bias, scale, misalignment | < 10 µs |
| 5 | REFSMMAT multiply (`mxv`) | < 5 µs |
| 6a | `moon_position` (lookup/interpolation) | 10–100 µs (until ephemeris implemented) |
| 6b | `average_g_step` (2× gravity eval + arithmetic) | 200–500 µs |
| 7 | Display request | < 5 µs |
| 8 | Exit hook (P40 steering) | 50–500 µs |
| 9 | Waitlist reschedule | < 5 µs |
| **Total** | | **< 1.5 ms typical; < 3 ms worst case** |

If SPI communication in Step 1 or the gravity computation in Step 6b causes the
total to exceed 3 ms, the heavy computation should be moved to an Executive job
(Strategy described in §5.2 of `specs/executive-spec.md`): the task reads PIPA,
schedules itself, and creates a job for the integration; the job writes back the
result when it completes.

---

## 12. Test Cases

### TC-AG-1: PIPA Bias Compensation — Zero Bias Produces Raw Counts as Delta-V

**Purpose**: Verify that when all calibration constants are nominal (zero bias,
identity misalignment, unity scale) the PIPA counts map directly to delta-V.

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

// Set nominal calibration (zero bias, identity misalignment, exact 1 m/s/count)
state.pipa_cal = PipaCalibration {
    scale: 1.0,           // 1 m/s per count for easy arithmetic
    bias: [0, 0, 0],
    misalignment: [[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]],
};
state.refsmmat = [[1.0, 0.0, 0.0],   // identity REFSMMAT (platform = inertial)
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]];

// Inject known PIPA counts
hw.imu.pipa = [10, -5, 3];

// Place the spacecraft at a position where gravity is approximately zero
// (or use a stub moon_position that returns zero gravity contribution)
state.csm_state.position = [0.0, 0.0, 7_000_000.0]; // 7000 km
state.csm_state.velocity = [7500.0, 0.0, 0.0];
state.csm_state.epoch    = Met(0);
state.csm_state.frame    = Frame::EarthInertial;

let v_before = state.csm_state.velocity;
start_servicer(&mut state, &mut hw);
// Advance Waitlist by 200 cs to trigger servicer_task (test harness simulates T3RUPT)
simulate_t3rupt(&mut state, &mut hw, 200);

// Verify: velocity changed by approximately [10, -5, 3] m/s (plus gravity term)
// With scale=1.0, bias=[0,0,0], identity matrices: delta_v_inertial = [10, -5, 3]
let v_after = state.csm_state.velocity;
let dv = [v_after[0]-v_before[0], v_after[1]-v_before[1], v_after[2]-v_before[2]];
// The gravity term also contributes ~2*9.6 = 19.2 m/s downward; check only the
// PIPA contribution by examining the component perpendicular to gravity.
assert!((dv[0] - 10.0).abs() < 0.1, "X delta-V mismatch: {}", dv[0]);
assert!((dv[1] - (-5.0)).abs() < 0.1, "Y delta-V mismatch: {}", dv[1]);
```

### TC-AG-2: PIPA Compensation with Bias — Bias Subtracted Before Scaling

**Purpose**: Verify that NBDX/NBDY/NBDZ bias values are subtracted from raw counts.

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

state.pipa_cal = PipaCalibration {
    scale: 1.0,
    bias: [5, -3, 2],   // NBDX=5, NBDY=-3, NBDZ=2
    misalignment: [[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]],
};
state.refsmmat = [[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]];

hw.imu.pipa = [5, -3, 2];  // Raw counts equal to bias

// delta_v should be [5-5, -3-(-3), 2-2] * 1.0 = [0, 0, 0] m/s
// Only gravity should change velocity.

state.csm_state = circular_leo_state();  // helper: 7000 km circular orbit
let v_before = state.csm_state.velocity;

start_servicer(&mut state, &mut hw);
simulate_t3rupt(&mut state, &mut hw, 200);

// The velocity change should be entirely gravitational, not from PIPA counts
let v_after = state.csm_state.velocity;
let accel_magnitude = norm(vsub(v_after, v_before)) / 2.0; // over 2 seconds
// Earth gravity at 7000 km ≈ 8.15 m/s², integrated over 2 s ≈ 16.3 m/s magnitude
// But direction is radially inward; for circular orbit the net change should be small
// Check that there is no additional velocity in the tangential direction:
let tangential_dv = v_after[0] - v_before[0];  // simplified for circular orbit
assert!(tangential_dv.abs() < 0.01,
    "Unexpected tangential delta-V {}: bias not correctly removed", tangential_dv);
```

### TC-AG-3: REFSMMAT Rotation — Platform Delta-V Correctly Rotated to Inertial

**Purpose**: Verify that a known platform-frame delta-V is correctly rotated into
the inertial frame by REFSMMAT.

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

// REFSMMAT: rotate X-platform to Y-inertial (90° rotation about Z)
let sin90 = 1.0_f64;
let cos90 = 0.0_f64;
state.refsmmat = [
    [cos90, -sin90, 0.0],
    [sin90,  cos90, 0.0],
    [0.0,    0.0,   1.0],
];

state.pipa_cal = PipaCalibration {
    scale: 1.0,
    bias: [0, 0, 0],
    misalignment: [[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]],
};

hw.imu.pipa = [10, 0, 0];  // 10 counts on platform X axis

// Expected: delta_v_inertial = REFSMMAT * [10, 0, 0] = [0, 10, 0]
// i.e., 10 m/s in the inertial Y direction

state.csm_state = circular_leo_state();
let vy_before = state.csm_state.velocity[1];

start_servicer(&mut state, &mut hw);
simulate_t3rupt(&mut state, &mut hw, 200);

let vy_after = state.csm_state.velocity[1];
// Inertial Y velocity should increase by approximately 10 m/s (plus Y gravity term)
let vy_pipa_contribution = vy_after - vy_before - gravity_y_over_2s(); // subtract gravity
assert!((vy_pipa_contribution - 10.0).abs() < 0.1,
    "REFSMMAT rotation error: expected ~10 m/s in Y, got {}", vy_pipa_contribution);
```

### TC-AG-4: Full Servicer Cycle — Zero Thrust in LEO Propagates Under Gravity Only

**Purpose**: Verify that with zero PIPA counts the state vector propagates under
gravity alone (consistency check for the Average-G integrator called by SERVICER).

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

// Identity calibration, zero PIPA counts
state.pipa_cal = PipaCalibration::NOMINAL;
state.refsmmat = IDENTITY_MATRIX;
hw.imu.pipa = [0, 0, 0];

// Circular orbit at 400 km altitude (ISS-like)
const R_LEO: f64 = 6_378_137.0 + 400_000.0; // metres
const V_CIRC: f64 = 7669.0;                   // m/s (approx for 400 km)
state.csm_state = StateVector {
    position: [R_LEO, 0.0, 0.0],
    velocity: [0.0, V_CIRC, 0.0],
    epoch: Met(0),
    frame: Frame::EarthInertial,
};

let r_before = norm(state.csm_state.position);

start_servicer(&mut state, &mut hw);
simulate_t3rupt(&mut state, &mut hw, 200);  // one cycle

let r_after = norm(state.csm_state.position);
let epoch_after = state.csm_state.epoch;

// After 2 seconds on a circular orbit, radius should be essentially unchanged
assert!((r_after - r_before).abs() < 10.0,
    "Orbital radius changed unexpectedly: Δr = {} m", r_after - r_before);

// Epoch must have advanced by exactly 200 centiseconds
assert_eq!(epoch_after, Met(200), "Epoch not advanced by 200 cs");

// State vector frame must be preserved
assert_eq!(state.csm_state.frame, Frame::EarthInertial);
```

### TC-AG-5: Known Delta-V — Velocity Changes by Expected Amount

**Purpose**: Verify that a known PIPA count produces the expected velocity change
using the standard scale factor.

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

// Use the nominal scale factor
state.pipa_cal = PipaCalibration::NOMINAL;  // scale = 0.0585 m/s/count
state.refsmmat = IDENTITY_MATRIX;

hw.imu.pipa = [100, 0, 0];  // 100 counts on X axis

// Expected: delta_v_x = (100 - 0 bias) * 0.0585 m/s/count = 5.85 m/s
// After identity REFSMMAT: delta_v_inertial_x = 5.85 m/s

state.csm_state = circular_leo_state();
let vx_before = state.csm_state.velocity[0];

start_servicer(&mut state, &mut hw);
simulate_t3rupt(&mut state, &mut hw, 200);

let vx_after = state.csm_state.velocity[0];
let vx_pipa_contribution = vx_after - vx_before - gravity_x_over_2s();
assert!((vx_pipa_contribution - 5.85).abs() < 0.01,
    "Expected Δvx ≈ 5.85 m/s, got {}", vx_pipa_contribution);
```

### TC-AG-6: SERVICER Self-Rescheduling — Waitlist Contains Next Task After One Cycle

**Purpose**: Verify that after one SERVICER cycle completes, the Waitlist contains
the next `servicer_task` entry scheduled 200 cs in the future.

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

state.pipa_cal = PipaCalibration::NOMINAL;
state.refsmmat = IDENTITY_MATRIX;
hw.imu.pipa = [0, 0, 0];
state.csm_state = circular_leo_state();

start_servicer(&mut state, &mut hw);
assert_eq!(state.waitlist.count, 1, "SERVICER not scheduled after start");

// Trigger one cycle
simulate_t3rupt(&mut state, &mut hw, 200);

// After one cycle: SERVICER should have rescheduled itself
assert_eq!(state.waitlist.count, 1, "SERVICER did not reschedule");
let entry = state.waitlist.entries[0].unwrap();
assert_eq!(entry.delta_time, 200, "Next SERVICER not at 200 cs");
assert!(core::ptr::eq(entry.task as *const _, servicer_task as *const _),
    "Waitlist task is not servicer_task");
```

### TC-AG-7: `stop_servicer` — SERVICER Terminates After One More Cycle

**Purpose**: Verify that calling `stop_servicer` causes the SERVICER to terminate
after the current in-flight cycle completes (it does not reschedule).

```rust
let mut state = AgcState::new();
let mut hw = SimHardware::new();

state.pipa_cal = PipaCalibration::NOMINAL;
state.refsmmat = IDENTITY_MATRIX;
hw.imu.pipa = [0, 0, 0];
state.csm_state = circular_leo_state();

start_servicer(&mut state, &mut hw);
assert!(is_servicer_active(&state));

stop_servicer(&mut state);
assert!(!is_servicer_active(&state), "SERVICER still active after stop");

// The already-queued task in the Waitlist will fire once and not reschedule
simulate_t3rupt(&mut state, &mut hw, 200);

// Waitlist should be empty (no reschedule happened)
assert_eq!(state.waitlist.count, 0, "SERVICER rescheduled after stop");

// Restart phase should be cleared
assert_eq!(state.restart.phases[GROUP_2], Phase(0), "Restart phase not cleared");
```

---

## 13. Spec Quality Checklist

Per `specs/README.md`:

- [x] AGC source file referenced: `Comanche055/AVERAGE_G_INTEGRATOR.agc`,
      `Comanche055/ERASABLE_ASSIGNMENTS.agc`, `Comanche055/INTEGRATION_INITIALIZATION.agc`
- [x] All erasable variables and AGC addresses listed: PIPAX/Y/Z (0125–0127),
      NBDX/NBDY/NBDZ (E1 bank), 1/PIPADT, RN/VN/TEPHEM (0306–0342)
- [x] Scale factors documented: counts → m/s (×0.0585), positions B+28 m, velocities B+7 m/s
- [x] SI units documented throughout
- [x] Input/output preconditions and postconditions for all three public functions
- [x] Edge cases and error handling: PIPA overflow (§9.1), Waitlist full (§9.2), NaN check (§9.3)
- [x] Seven test cases with expected values (TC-AG-1 through TC-AG-7)
- [x] Rust API signatures designed (`start_servicer`, `stop_servicer`, `servicer_task`)
- [x] Invariants stated: PIPA is single-consumer (§2.2), restart group 2 phase semantics (§2.5)
- [x] Consistency with `docs/architecture.md` §7.4 verified
- [x] Cross-references to `integration-spec.md` §3, `state-vector-spec.md` §5.1,
      `hal-spec.md` §8, `executive-spec.md` §4.6 and §5.3

---

## 14. Design Decisions

### ADR-SVC-001: Hardware Parameter in Waitlist Tasks

**Decision**: Use Strategy A (raw pointer in `AgcState`) with a concrete
type-erased adapter, pending confirmation from the architecture team on whether
`AgcHardware` can be made object-safe without performance regression.

**Rationale**: The Waitlist task signature `fn(&mut AgcState)` is a fundamental
architectural constraint shared with the original AGC (the AGC also had a single
"state" argument for all Waitlist tasks — the erasable memory pointer). Changing
this signature would require modifying `executive/waitlist.rs` and all other
Waitlist tasks simultaneously. The raw-pointer approach is safe in this
single-threaded execution model and preserves the invariant.

**Alternative considered**: Adding `hw: &mut impl AgcHardware` to the Waitlist
task signature. Rejected because: (a) it requires `AgcHardware` to appear in
`WaitlistEntry`, making the type concrete and losing the generic abstraction;
(b) it requires every Waitlist task to declare a hardware parameter even if it
does not use hardware.

### ADR-SVC-002: SERVICER Stop via Flag, Not Waitlist Cancellation

**Decision**: `stop_servicer` clears a flag rather than removing the Waitlist entry.

**Rationale**: The Waitlist has no cancellation operation (matching the original
AGC design). Scanning the Waitlist for `servicer_task` by comparing function
pointers is fragile (function pointer equality is reliable in Rust for fn-items
but not guaranteed to be stable across optimization passes for closures). The
flag approach is simple, safe, and correct: the final queued task runs once,
observes the flag, and terminates cleanly without further rescheduling.

### ADR-SVC-003: Fixed 200 cs Reschedule Period

**Decision**: The SERVICER always reschedules at exactly 200 cs from the current
time, not from the nominal cycle start time.

**Rationale**: Consistent with the original AGC, which used `WAITLIST 200` at
the end of each cycle. The slight jitter (a few milliseconds depending on PIPA
SPI read time) is negligible because PIPA counts accumulate continuously and
the full count is always captured. For a 200 cs interval, 3 ms of jitter is
1.5 ppm of the cycle period, far below the navigation accuracy budget.
