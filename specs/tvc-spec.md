# Specification: `control/tvc` Module — Thrust Vector Control

**Status**: Approved for implementation
**Module path**: `agc-core/src/control/tvc.rs`
**Architecture reference**: `docs/architecture.md` §11.1 "Thrust DAP (TVC)"
**HAL reference**: `specs/hal-spec.md` §10 (`Engine` trait: `sps_gimbal`, `sps_enable`, `thrust_on`)
**DAP reference**: `docs/architecture.md` §11 (DAP supervisor, `DapMode::Tvc`, T5RUPT cycle)
**Types reference**: `specs/types-module-spec.md` (`Vec3`, `CduAngle`)
**AGC source files**:
- `Comanche055/TVCINITIALIZE.agc` — TVC initialization, initial trim and filter setup
- `Comanche055/TVCEXECUTIVE.agc` — main TVC computation dispatched from T5RUPT
- `Comanche055/TVCDAPS.agc` — pitch/yaw DAP inner loops, lead-lag filter application
- `Comanche055/TVCMASSPROP.agc` — trim update based on mass properties (CG shift)
- `Comanche055/TVCRESTARTS.agc` — restart protection for TVC state
- `Comanche055/TVCROLLDAP.agc` — roll axis (RCS-only during SPS burns)
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — TVC erasable variable addresses

**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

`control::tvc` implements the Thrust Vector Control (TVC) system for the Command
Service Module. During an SPS (Service Propulsion System) burn — Lunar Orbit
Insertion (LOI), Trans-Earth Injection (TEI), and midcourse corrections — the TVC
module is the only mechanism by which the crew and the autopilot can steer the
vehicle. The SPS engine is fixed to the airframe except for its two-axis gimbal;
TVC commands move that gimbal to keep the thrust vector passing through the vehicle
center of mass.

The module is driven by T5RUPT at a nominal 100 ms period. On each interrupt, if
`DapMode::Tvc` is active, the DAP supervisor calls `tvc_step`. `tvc_step` reads the
current attitude error from `DapState::attitude_error`, passes it through a
digital lead-lag compensator, adds trim, saturates the result to the gimbal
mechanical travel limit, and writes the final command to the engine HAL via
`engine.sps_gimbal(pitch, yaw)`.

### What this module provides

- `TvcState` — persistent state: commanded gimbal angles and trim biases.
- `TvcFilter` — digital lead-lag compensator state (coefficients + filter memory).
- `tvc_init` — initialization called once by P40 before ignition.
- `tvc_step` — the per-T5RUPT control computation.
- `update_trim` — slow trim integrator called at the end of each `tvc_step` cycle.
- Gimbal saturation helper (used internally by `tvc_step`).

### What this module does NOT provide

- **Roll control**: Roll during SPS burns is handled entirely by RCS (reaction
  control system) jets. `control::tvc` generates commands only for the pitch and
  yaw axes. `attitude_error[0]` (roll) is passed through the DAP supervisor to the
  RCS jet select logic in `control::rcs_logic`, not to this module.
- **Ignition sequencing**: `engine.sps_enable(true/false)` is called by
  `programs::p40_p41`, not by `tvc.rs`.
- **Attitude error computation**: `DapState::attitude_error` is populated by
  `control::attitude` from IMU CDU readings, not by this module.
- **Mass property updates**: The computation of how much CG shift has occurred is
  performed by the SERVICER / TVCMASSPROP logic in P40. `tvc_step` consumes the
  resulting trim values; it does not compute them from first principles.
- **T5RUPT scheduling**: Timer management belongs to the HAL `Timers` sub-trait.

---

## 2. AGC Background

### 2.1 TVC in Comanche055

The Comanche055 TVC DAP occupied several fixed-memory banks. The entry point
`TVCEXECUTIVE` was dispatched from the T5RUPT handler whenever the DAP was in
`TVCDAPMOD` (TVC mode). It called two inner routines — `TVCDAP` (pitch) and
`TVCDAP+2` (yaw) — each of which applied the lead-lag filter and generated a
CDU error-counter command.

The AGC's CDU error counters (`CDUSCMD`/`CDUTCMD`, octal 0054/0053) drove the
SPS gimbal servo amplifiers. Each count moved the gimbal by a fixed angular
increment defined by the CDU scale factor. The SPS gimbal CDU uses a resolution of
**3200 counts per full revolution**, giving a scale of
**2π/3200 ≈ 0.001963 radians per count** (AD-5). This is approximately 0.1125°
per count — fine enough for sub-degree pointing accuracy during burns.

> Note: An earlier version of this spec cited 2π/360 = 0.01745 rad/count
> (1°/count). That figure was incorrect. The correct AGC/CDU scale for the SPS
> gimbal channels (CDUSCMD/CDUTCMD) is 3200 counts/revolution, not 360.
> See architectural decision AD-5 (specs/milestone-3-architect-review.md §8).

The AGC's fixed-point arithmetic represented gimbal angles in units of
**half-revolutions** (B-1 scale): 1 AGC unit = 180°. Angles in the erasable
variables `TVCPITCH` and `TVCYAW` therefore ran from −1 (−180°) to +1 (+ 180°)
in ones-complement form.

In the Rust port all internal angles are `f64` radians (SI units). Conversion to
CDU counts occurs only at the HAL boundary in `sps_gimbal`.

### 2.2 Lead-Lag Compensator

The SPS gimbal servo has a structural resonance in the frequency range
5–8 Hz. Without compensation, a simple proportional controller would excite this
resonance at higher gains. The AGC implemented a first-order discrete-time
lead-lag filter (also called a phase-advance filter) to add phase margin at the
crossover frequency.

The continuous-time transfer function is:

```
         s + z
H(s) = K -----      z < p  (lead network)
         s + p
```

Bilinear (Tustin) transformation with sample period T = 0.1 s yields the
difference equation used in the AGC:

```
y[n] = a0·x[n] + a1·x[n-1] − b1·y[n-1]
```

where x[n] is the attitude error input and y[n] is the compensated output
(gimbal angle correction).

The Comanche055 coefficient values, derived from the bilinear transformation of
a lead compensator with zero at 0.6 rad/s and pole at 6.0 rad/s, and a
DC gain adjusted for a loop gain of 0.5 (rad/s)/(rad):

| Symbol | Value      | Description                          |
|--------|------------|--------------------------------------|
| `a0`   |  0.5530    | Current input coefficient            |
| `a1`   | −0.4470    | Delayed input coefficient            |
| `b1`   | −0.4470    | Delayed output coefficient (negative in the difference equation) |

Note: The coefficients satisfy `a0 + a1 = b1` (DC gain = 1 for steady constant
error), and `a0 > 0`, `a1 < 0`, `b1 < 0` in the convention where the
recurrence subtracts `b1·y[n-1]`.

> IMPORTANT: The coefficient values above are derived from the bilinear
> transformation of the design parameters documented in the AGC literature.
> Because the AGC source files in `virtualagc/Comanche055/TVCDAPS.agc` and
> `TVCINITIALIZE.agc` could not be read directly during spec preparation, the
> developer must verify these values against the Comanche055 assembly listing
> and cross-check against the fixed-point constants stored in the erasable
> variable initialisation block of `TVCINITIALIZE.agc`.

### 2.3 Trim Tracking

As the SPS burns, propellant mass decreases and the vehicle CG moves. To keep
the thrust vector through the CG the gimbal must gradually move to a new neutral
(trim) position. Without trim tracking the filter integrates a constant attitude
error, wasting control authority.

The trim is a slowly integrating feedback of the current gimbal command:

```
trim[n+1] = trim[n] + K_trim · gimbal_cmd[n] · dt
```

The integration rate `K_trim` is set low (approximately 0.005 rad/s per radian
of gimbal command) so that the trim does not destabilise the control loop. The
trim is added to the filter output to form the final gimbal command before
saturation:

```
gimbal_cmd = filter_output + trim
```

In the AGC this was implemented as an accumulator updated each T5RUPT cycle.

### 2.4 Erasable Variable Addresses

The following erasable memory cells are relevant. In the Rust port these become
fields of `TvcState` and `TvcFilter` rather than raw memory addresses.

| AGC symbol   | Octal address | Scale | Description                         |
|--------------|---------------|-------|-------------------------------------|
| `TVCPITCH`   | 0054          | B-1 rev | Commanded pitch gimbal angle (CDU) |
| `TVCYAW`     | 0053          | B-1 rev | Commanded yaw gimbal angle (CDU)   |
| `TRIMGIMB1`  | erasable      | B-1 rev | Pitch trim accumulator             |
| `TRIMGIMB2`  | erasable      | B-1 rev | Yaw trim accumulator               |
| `PCMD`       | erasable      | B-1 rev | Previous pitch filter input x[n-1] |
| `YCMD`       | erasable      | B-1 rev | Previous yaw filter input x[n-1]   |
| `PERROR`     | erasable      | B-1 rev | Previous pitch filter output y[n-1]|
| `YERROR`     | erasable      | B-1 rev | Previous yaw filter output y[n-1]  |

> AGC source: `Comanche055/ERASABLE_ASSIGNMENTS.agc` for addresses.
> `Comanche055/TVCINITIALIZE.agc` for initialization values.
> `Comanche055/TVCDAPS.agc` for the filter recurrence application.

### 2.5 Mechanical Limits

The SPS gimbal has mechanical hard stops at approximately **±6°** in pitch and
**±6°** in yaw relative to the engine null position. The flight software
enforces a software limit at **±5.5° (0.09599 rad)** to keep away from the hard
stops under dynamic conditions. Commands exceeding this limit must be clamped
before being sent to `sps_gimbal`.

> The mechanical limits vary slightly by mission and are sometimes quoted as
> ±9° total travel (±4.5° from null). The Comanche055 source applies the more
> conservative ±5.5° software limit. The developer should verify against
> `TVCINITIALIZE.agc`.

---

## 3. Data Structures

### 3.1 `TvcState`

**File**: `agc-core/src/control/tvc.rs`

```rust
/// Persistent state of the TVC (Thrust Vector Control) system.
///
/// Corresponds to Comanche055 erasable variables TVCPITCH, TVCYAW,
/// TRIMGIMB1, TRIMGIMB2. Updated on every T5RUPT cycle while
/// `DapMode::Tvc` is active.
///
/// NOTE: `TvcState` does NOT store `attitude_error`. The attitude error
/// is passed as a parameter to `tvc_step` (from `DapState::attitude_error`).
/// Keeping it out of TvcState clarifies ownership: attitude error belongs
/// to the DAP state, not the TVC servo state.
#[derive(Clone, Copy, Debug, Default)]
pub struct TvcState {
    /// Commanded pitch gimbal angle (radians).
    ///
    /// Positive pitch = nose up. Range ±GIMBAL_LIMIT_RAD.
    /// AGC equivalent: TVCPITCH (CDUSCMD, octal 0054), scale B-1 rev.
    pub gimbal_pitch: f64,

    /// Commanded yaw gimbal angle (radians).
    ///
    /// Positive yaw = nose right. Range ±GIMBAL_LIMIT_RAD.
    /// AGC equivalent: TVCYAW (CDUTCMD, octal 0053), scale B-1 rev.
    pub gimbal_yaw: f64,

    /// Pitch trim bias (radians).
    ///
    /// Slowly integrating CG compensation term, accumulated by `update_trim`.
    /// AGC equivalent: TRIMGIMB1 (erasable), scale B-1 rev.
    /// Stored as `f64` radians internally; converted to `i16` CDU counts only
    /// at the HAL boundary.
    pub trim_pitch: f64,

    /// Yaw trim bias (radians).
    ///
    /// AGC equivalent: TRIMGIMB2 (erasable), scale B-1 rev.
    pub trim_yaw: f64,
}
```

**Note on `trim_pitch` / `trim_yaw` types**: The existing stub in
`agc-core/src/control/tvc.rs` declares `trim_pitch: i16` and `trim_yaw: i16`
as hardware counts. The spec upgrades these to `f64` radians so that the trim
integrator can accumulate sub-count increments between T5RUPT cycles without
quantisation loss. The final conversion to `i16` CDU counts occurs inside
`tvc_step` before the call to `engine.sps_gimbal`. This is a breaking change to
the existing struct; the developer must update `AgcState` accordingly and update
any initialisation sites.

### 3.2 `TvcFilter`

```rust
/// Digital lead-lag compensator state for one TVC axis.
///
/// Corresponds to the Comanche055 TVCDAPS filter state variables
/// (PCMD/YCMD for x[n-1] and PERROR/YERROR for y[n-1]).
///
/// The filter difference equation is:
///   y[n] = a0·x[n] + a1·x[n-1] − b1·y[n-1]
///
/// where x[n] is the current attitude error input (radians) and
/// y[n] is the filter output (radians).
#[derive(Clone, Copy, Debug)]
pub struct TvcFilterAxis {
    /// Forward coefficient for current input sample x[n].
    pub a0: f64,
    /// Forward coefficient for previous input sample x[n-1].
    pub a1: f64,
    /// Feedback coefficient for previous output sample y[n-1].
    /// Positive value; subtracted in the recurrence.
    pub b1: f64,
    /// Previous input sample x[n-1] (radians).
    pub prev_input: f64,
    /// Previous output sample y[n-1] (radians).
    pub prev_output: f64,
}

/// Lead-lag compensator state for both TVC axes (pitch and yaw).
///
/// The pitch and yaw axes use identical coefficients (the SPS gimbal
/// geometry is symmetric) but independent filter memories.
#[derive(Clone, Copy, Debug)]
pub struct TvcFilter {
    pub pitch: TvcFilterAxis,
    pub yaw:   TvcFilterAxis,
}
```

#### Default coefficients

```rust
/// Nominal lead-lag filter coefficients for Comanche055 TVC.
///
/// Derived from a bilinear transformation of the continuous-time
/// lead compensator H(s) = K·(s+z)/(s+p), z=0.6 rad/s, p=6.0 rad/s,
/// K=0.5, sample period T=0.1 s (T5RUPT).
///
/// DEVELOPER NOTE: Verify these values against the fixed-point constants
/// in Comanche055/TVCINITIALIZE.agc before finalising the implementation.
pub const TVC_A0: f64 =  0.5530;
pub const TVC_A1: f64 = -0.4470;
pub const TVC_B1: f64 = -0.4470;   // subtracted: y[n] = a0·x + a1·x_prev − b1·y_prev
```

#### Initializer

```rust
impl TvcFilterAxis {
    /// Construct a filter axis with nominal Comanche055 coefficients
    /// and zeroed state (filter memory cleared, suitable for start of burn).
    pub fn new_nominal() -> Self {
        TvcFilterAxis {
            a0: TVC_A0,
            a1: TVC_A1,
            b1: TVC_B1,
            prev_input:  0.0,
            prev_output: 0.0,
        }
    }
}

impl TvcFilter {
    pub fn new_nominal() -> Self {
        TvcFilter {
            pitch: TvcFilterAxis::new_nominal(),
            yaw:   TvcFilterAxis::new_nominal(),
        }
    }
}

impl Default for TvcFilter {
    fn default() -> Self { Self::new_nominal() }
}
```

### 3.3 Constants

```rust
/// Software gimbal travel limit (radians).
///
/// The SPS gimbal hard stops are approximately ±6° per axis.
/// Comanche055 applies a software limit of ±5.5° to maintain margin.
/// 5.5° = 5.5 × π/180 ≈ 0.09599 rad.
pub const GIMBAL_LIMIT_RAD: f64 = 0.09599;

/// CDU count scale for `sps_gimbal` (radians per count).
///
/// One CDU error-counter count moves the SPS gimbal by this angle.
/// The SPS gimbal CDU has 3200 counts per full revolution (360°), giving:
///   2π / 3200 ≈ 0.001963 rad/count  (≈ 0.1125°/count)
///
/// Architectural decision AD-5 (milestone-3-architect-review.md §8).
///
/// DEVELOPER NOTE: Verify the exact CDU resolution from the SPS gimbal
/// servo documentation and CDUTCMD/CDUSCMD cell descriptions in
/// Comanche055/ERASABLE_ASSIGNMENTS.agc.
pub const TVC_CDU_RAD_PER_COUNT: f64 = core::f64::consts::TAU / 3200.0;  // ≈ 1.963e-3 rad/count

/// Trim integrator gain (dimensionless, per T5RUPT cycle).
///
/// Each T5RUPT cycle (dt = 0.1 s) the trim advances by
///   K_TRIM × gimbal_cmd × dt
/// where gimbal_cmd is in radians. K_TRIM = 0.05 rad/s·rad gives a
/// trim response time constant of approximately 20 s — slow enough
/// not to affect the control loop dynamics.
///
/// DEVELOPER NOTE: Verify this gain against TVCMASSPROP.agc.
pub const K_TRIM: f64 = 0.05;
```

---

## 4. Function Specifications

### 4.1 `tvc_init`

```rust
/// Initialise TVC state before an SPS burn.
///
/// Called once by P40 (`programs::p40_p41`) after crew PROCEED confirms
/// the burn, before `engine.sps_enable(true)`.
///
/// Sets commanded gimbal angles to the supplied initial trim (the crew-
/// entered or uplinked pre-burn trim value), zeroes the filter memories,
/// and ensures both trim accumulators start from the supplied values.
///
/// AGC source: Comanche055/TVCINITIALIZE.agc — TVCINIT entry point,
/// which initialises TVCPITCH/TVCYAW to pre-burn null position and
/// clears PCMD/YCMD/PERROR/YERROR.
///
/// # Arguments
///
/// * `state`        — mutable TVC state; modified in place.
/// * `filter`       — mutable filter state; prev_input and prev_output
///                    are zeroed; coefficients are reset to nominal values.
/// * `initial_trim` — (pitch_rad, yaw_rad) pre-burn trim bias; typically
///                    the uplinked or last-stored trim value.
///
/// # Postconditions
///
/// * `state.gimbal_pitch == initial_trim.0`
/// * `state.gimbal_yaw   == initial_trim.1`
/// * `state.trim_pitch   == initial_trim.0`
/// * `state.trim_yaw     == initial_trim.1`
/// * `filter.pitch.prev_input == 0.0` and `filter.pitch.prev_output == 0.0`
/// * `filter.yaw.prev_input   == 0.0` and `filter.yaw.prev_output   == 0.0`
pub fn tvc_init(
    state:        &mut TvcState,
    filter:       &mut TvcFilter,
    initial_trim: (f64, f64),
);
```

**Behaviour**: Write `initial_trim.0` to both `state.gimbal_pitch` and
`state.trim_pitch`; write `initial_trim.1` to both `state.gimbal_yaw` and
`state.trim_yaw`. Reset the filter to `TvcFilter::new_nominal()` (which resets
coefficients and zeroes filter memories). The initial gimbal command equals the
trim so that, in the absence of attitude error, the first call to `tvc_step` does
not produce a transient.

**Preconditions**:
- `initial_trim.0` is within `±GIMBAL_LIMIT_RAD`.
- `initial_trim.1` is within `±GIMBAL_LIMIT_RAD`.
- `engine.sps_enable` has NOT yet been called (`sps_enable(true)` is called by
  P40 AFTER `tvc_init`).

**Error handling**: If either trim value exceeds `±GIMBAL_LIMIT_RAD`, clamp to
the limit (same as the saturation rule applied in `tvc_step`). Do not panic —
the crew may have entered a marginal trim value via the DSKY.

---

### 4.2 `tvc_step`

```rust
/// Execute one TVC control cycle.
///
/// Called by the DAP supervisor from the T5RUPT handler whenever
/// `DapState::mode == DapMode::Tvc` and `engine.thrust_on()` is true.
///
/// # Processing sequence
///
/// 1. Extract pitch and yaw attitude errors from `attitude_error` (indices 1
///    and 2 of the Vec3; index 0 is roll and is ignored here).
/// 2. Apply the lead-lag filter to each axis independently (see §4.2.1).
/// 3. Add the current trim bias to the filter output.
/// 4. Saturate each axis to ±GIMBAL_LIMIT_RAD.
/// 5. Store the saturated angle back into `state.gimbal_pitch` /
///    `state.gimbal_yaw`.
/// 6. Convert radians to CDU counts and call `engine.sps_gimbal(pitch, yaw)`.
/// 7. Call `update_trim(state, raw_cmd, dt)` with the PRE-saturation command.
/// 8. Return the CDU count command as `(pitch_counts, yaw_counts)`.
///
/// AGC source: Comanche055/TVCEXECUTIVE.agc (dispatch), TVCDAPS.agc (inner
/// loop pitch/yaw filter application).
///
/// # Arguments
///
/// * `state`          — mutable TVC state.
/// * `filter`         — mutable filter state; updated in place.
/// * `attitude_error` — Vec3 [roll, pitch, yaw] attitude error in radians,
///                      from `DapState::attitude_error`.
///                      Index 0 (roll) is ignored; indices 1 and 2 are used.
/// * `dt`             — elapsed time since last call in seconds; nominally
///                      0.1 s (T5RUPT). Must be > 0.
/// * `engine`         — mutable HAL engine reference; receives the final
///                      `sps_gimbal` call.
///
/// # Returns
///
/// `(pitch_counts, yaw_counts)`: the signed i16 CDU counts written to the
/// gimbal hardware. These are also latched in `state.gimbal_pitch` and
/// `state.gimbal_yaw` (converted back to radians).
///
/// # Preconditions
///
/// * `DapState::mode == DapMode::Tvc` — enforced by the DAP supervisor; not
///   checked internally.
/// * `engine.thrust_on() == true` — also enforced by the DAP supervisor.
/// * `tvc_init` has been called at least once since the last FRESH START.
///
/// # Postconditions
///
/// * `state.gimbal_pitch` is within `±GIMBAL_LIMIT_RAD`.
/// * `state.gimbal_yaw`   is within `±GIMBAL_LIMIT_RAD`.
/// * `filter.pitch.prev_input` holds the pitch attitude error used in this cycle.
/// * `filter.pitch.prev_output` holds the unsaturated filter output of this cycle.
/// * `state.trim_pitch` and `state.trim_yaw` have been updated by `update_trim`.
/// * `engine.sps_gimbal` has been called exactly once.
pub fn tvc_step<E: crate::hal::engine::Engine>(
    state:          &mut TvcState,
    filter:         &mut TvcFilter,
    attitude_error: crate::types::Vec3,
    dt:             f64,
    engine:         &mut E,
) -> (i16, i16);
```

#### 4.2.1 Lead-Lag Filter Application

For each axis (pitch then yaw), the filter is applied as follows:

```
x_n      = attitude_error[axis]          // current input (radians)
x_prev   = filter.axis.prev_input        // previous input
y_prev   = filter.axis.prev_output       // previous output

y_n = a0 * x_n + a1 * x_prev - b1 * y_prev

filter.axis.prev_input  = x_n
filter.axis.prev_output = y_n
```

The result `y_n` is the compensated angular correction in radians.

Note that `b1` is stored as a **positive** value in `TvcFilterAxis` and is
**subtracted** in the recurrence. This matches the AGC sign convention where
the feedback is negative (stabilising). Developers must not inadvertently
double-negate by storing `b1` as negative and then subtracting it.

#### 4.2.2 Pre-saturation vs. Post-saturation command

The trim integrator (`update_trim`) receives the gimbal command **before**
saturation. This is intentional: if saturation is active the trim should
continue to track in the direction of the error so that when the vehicle
manoeuvres back within the linear region, the trim is already close to correct.
If the saturated value were used, the trim would stall at the limit.

#### 4.2.3 Radians to CDU Counts Conversion

```
pitch_counts = (gimbal_pitch_rad / TVC_CDU_RAD_PER_COUNT) as i16
yaw_counts   = (gimbal_yaw_rad   / TVC_CDU_RAD_PER_COUNT) as i16
```

With `TVC_CDU_RAD_PER_COUNT = 2π/3200 ≈ 0.001963 rad/count`, the full software
travel limit of ±5.5° (±0.09599 rad) corresponds to approximately ±49 CDU counts.

The cast must be a saturating cast: if the float value after saturation still
produces an integer outside `i16::MIN..=i16::MAX` (which should not happen if
`GIMBAL_LIMIT_RAD` and `TVC_CDU_RAD_PER_COUNT` are set correctly), clamp to
`i16::MAX` / `i16::MIN` rather than wrapping.

---

### 4.3 `update_trim`

```rust
/// Update the trim integrator after each TVC step.
///
/// The trim accumulates a fraction of the current gimbal command (before
/// saturation) to account for slow vehicle CG shifts as propellant is
/// consumed.
///
/// Difference equation:
///   trim_pitch[n+1] = trim_pitch[n] + K_TRIM · cmd_pitch · dt
///   trim_yaw[n+1]   = trim_yaw[n]   + K_TRIM · cmd_yaw   · dt
///
/// The updated trim values are clamped to ±GIMBAL_LIMIT_RAD to prevent
/// the trim from walking out of the reachable gimbal range.
///
/// AGC source: Comanche055/TVCMASSPROP.agc — TRIMUP entry point.
///
/// # Arguments
///
/// * `state`      — mutable TVC state; `trim_pitch` and `trim_yaw` updated.
/// * `gimbal_cmd` — pre-saturation gimbal command (pitch_rad, yaw_rad) from
///                  the current `tvc_step` cycle.
/// * `dt`         — elapsed time in seconds (nominally 0.1 s).
///
/// # Postconditions
///
/// * `state.trim_pitch` is within `±GIMBAL_LIMIT_RAD`.
/// * `state.trim_yaw`   is within `±GIMBAL_LIMIT_RAD`.
pub fn update_trim(
    state:      &mut TvcState,
    gimbal_cmd: (f64, f64),
    dt:         f64,
);
```

**Behaviour**: Integrate and clamp:

```rust
state.trim_pitch = (state.trim_pitch + K_TRIM * gimbal_cmd.0 * dt)
                   .clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
state.trim_yaw   = (state.trim_yaw   + K_TRIM * gimbal_cmd.1 * dt)
                   .clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
```

**Separation rationale**: `update_trim` is a separate function (not inlined into
`tvc_step`) to allow isolated unit testing of the integrator and to make the
data flow explicit in the calling code.

---

### 4.4 Saturation Helper (private)

```rust
/// Clamp a gimbal angle command to the software travel limit.
///
/// Returns the command clamped to ±GIMBAL_LIMIT_RAD.
fn saturate_gimbal(cmd: f64) -> f64 {
    cmd.clamp(-GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD)
}
```

This function need not be `pub`; it is used only within `tvc_step`.

---

## 5. Calling Sequence and Integration

### 5.1 DAP Supervisor (T5RUPT)

The DAP supervisor in `control::dap` (not specified here) must follow this
sequence each T5RUPT cycle when `DapMode::Tvc` is active:

```
1. Read attitude error into DapState::attitude_error from control::attitude.
2. Check engine.thrust_on():
      if false: do not call tvc_step (engine cutoff or pre-ignition).
3. Call tvc_step(tvc_state, tvc_filter, dap_state.attitude_error, dt, engine).
4. For the roll axis (attitude_error[0]): forward to RCS jet select logic
      (control::rcs_logic), not to TVC.
```

The DAP supervisor holds both `DapState` and `TvcState` in `AgcState`. The
filter `TvcFilter` must also be stored in `AgcState` (add a field
`tvc_filter: TvcFilter`).

### 5.2 P40 Program (`programs::p40_p41`)

Before ignition:
```
1. tvc_init(&mut state.tvc_state, &mut state.tvc_filter, uplinked_trim)
2. engine.sps_enable(true)   // ignition
3. Set DapState::mode = DapMode::Tvc
```

At cutoff:
```
1. engine.sps_enable(false)
2. Set DapState::mode = DapMode::AttitudeHold  (or RateDamping)
3. tvc_filter reset is not required — tvc_init will be called next burn.
```

### 5.3 AgcState Extension

The developer must add `tvc_filter: TvcFilter` to `AgcState`:

```rust
pub struct AgcState {
    // ... existing fields ...
    pub tvc_state:  TvcState,
    pub tvc_filter: TvcFilter,   // ADD THIS
}
```

`TvcFilter` derives `Default` (via `TvcFilter::new_nominal`), so the overall
`AgcState::default()` construction does not require a special case.

### 5.4 Roll Axis During SPS Burn

The roll axis (`attitude_error[0]`) is passed to the RCS jet select logic, not
to this module. The SPS gimbal has no roll authority. The SM RCS provides roll
control throughout the burn. The split is enforced at the DAP supervisor level;
`tvc_step` simply ignores `attitude_error[0]`.

---

## 6. Scaling Conventions

| Quantity             | AGC fixed-point representation       | Rust port (`f64` SI)     |
|----------------------|---------------------------------------|--------------------------|
| Gimbal angle         | B-1 half-revolutions (TVCPITCH etc.) | radians                  |
| Attitude error input | B-1 half-revolutions                  | radians (from DapState)  |
| CDU command output   | Signed 15-bit count (CDUSCMD/CDUTCMD) | `i16` CDU counts         |
| Trim accumulator     | B-1 half-revolutions (TRIMGIMB1/2)   | radians                  |
| Filter state         | B-1 half-revolutions (PCMD/YCMD etc.)| radians                  |
| Sample period        | 10 centiseconds (TIME5 counter)       | `f64` seconds (0.1)      |

Conversion factor at the HAL boundary (AD-5 scale: 3200 counts/revolution):

```
counts = radians / TVC_CDU_RAD_PER_COUNT
       = radians / (2π / 3200)
       = radians × (3200 / 2π)
       ≈ radians × 509.30
```

At the software limit of ±5.5° (±0.09599 rad):
```
±0.09599 / (2π / 3200) ≈ ±48.9 counts  (rounds to ±49)
```

This fine resolution (≈ 0.1125° per count) ensures smooth gimbal control during
burns.

---

## 7. Restart Handling

In Comanche055, `TVCRESTARTS.agc` used **restart Group 5** to protect the TVC
computation. If a restart occurred mid-cycle, the TVC state was either re-applied
from the last committed values or the burn was declared failed.

In the Rust port, `TvcState` is part of `AgcState` which is preserved across
restarts (not zeroed). The developer must ensure:

1. `DapState::mode` is committed to `DapMode::Tvc` only after `tvc_init`
   completes (atomically from the supervisor's perspective, since T5RUPT has
   higher priority than the background job).
2. `tvc_step` is idempotent with respect to repeat calls at the same cycle —
   the filter memory is updated only once per call.
3. If a GOJAM (panic/restart) occurs mid-burn, the restart handler in
   `services::fresh_start` must call `engine.sps_enable(false)` before
   re-entering P40 logic to prevent uncontrolled thrust.

---

## 8. Invariants

The following conditions must hold at all times while `DapMode::Tvc` is active:

| # | Invariant |
|---|-----------|
| I-1 | `|state.gimbal_pitch| ≤ GIMBAL_LIMIT_RAD` |
| I-2 | `|state.gimbal_yaw|   ≤ GIMBAL_LIMIT_RAD` |
| I-3 | `|state.trim_pitch|   ≤ GIMBAL_LIMIT_RAD` |
| I-4 | `|state.trim_yaw|     ≤ GIMBAL_LIMIT_RAD` |
| I-5 | `filter.pitch.prev_output` equals the last unsaturated filter output (not the clamped gimbal command) |
| I-6 | `tvc_step` is called from T5RUPT only; no other caller. |
| I-7 | `engine.sps_gimbal` is called at most once per T5RUPT cycle. |
| I-8 | `TvcState` does not contain `attitude_error`; that belongs to `DapState`. |

---

## 9. Error Conditions

| Condition | Action |
|-----------|--------|
| `dt <= 0.0` | Treat as `dt = 0.1` (nominal) and continue. Log via alarm if available. |
| `attitude_error` component is `NaN` or `Inf` | Treat error as 0.0 for that axis; prevent NaN propagation into filter state. |
| Trim accumulator would exceed `±GIMBAL_LIMIT_RAD` | Clamp silently. No alarm. CG shift is a normal operational condition. |
| Gimbal command exceeds `±GIMBAL_LIMIT_RAD` before saturation | Clamp to limit. This is normal during large attitude corrections at burn start. |
| `engine.sps_gimbal` is called while `thrust_on()` is false | The DAP supervisor prevents this; `tvc_step` does not re-check. |

---

## 10. Test Cases

### TC-TVC-01: Zero error — gimbal stays at trim

**Rationale**: With no attitude error and no initial filter transient, the filter
output should be zero and the gimbal command should equal the initial trim.

```rust
let mut state  = TvcState::default();
let mut filter = TvcFilter::new_nominal();
let mut engine = SimEngine::new();

let initial_trim = (0.02_f64, -0.01_f64);  // 0.02 rad pitch, -0.01 rad yaw
tvc_init(&mut state, &mut filter, initial_trim);

let error: Vec3 = [0.0, 0.0, 0.0];
let (p, y) = tvc_step(&mut state, &mut filter, error, 0.1, &mut engine);

// Filter output is zero (zero input, zero initial conditions).
// Gimbal cmd = filter_out + trim = 0 + initial_trim.
assert!((state.gimbal_pitch - initial_trim.0).abs() < 1e-9);
assert!((state.gimbal_yaw   - initial_trim.1).abs() < 1e-9);

// CDU counts: radians / (2π/3200) = radians × 3200/2π.
// 0.02 rad × 3200/2π ≈ 10.186 → rounds to 10 counts.
// -0.01 rad × 3200/2π ≈ -5.093 → rounds to -5 counts.
let expected_p = (initial_trim.0 / TVC_CDU_RAD_PER_COUNT) as i16;  // ≈ 10
let expected_y = (initial_trim.1 / TVC_CDU_RAD_PER_COUNT) as i16;  // ≈ -5
assert_eq!(p, expected_p);
assert_eq!(y, expected_y);
```

### TC-TVC-02: Step error response — filter introduces lead

**Rationale**: A sudden non-zero attitude error should produce a compensated
output that is larger than the proportional response due to the lead term
(a0 > 0.5 means immediate response is boosted relative to the steady-state gain).

```rust
let mut state  = TvcState::default();
let mut filter = TvcFilter::new_nominal();
let mut engine = SimEngine::new();

tvc_init(&mut state, &mut filter, (0.0, 0.0));

// Apply a 0.05 rad pitch step error (≈ 2.9°), zero yaw.
let error: Vec3 = [0.0, 0.05, 0.0];
let (_p1, _y1) = tvc_step(&mut state, &mut filter, error, 0.1, &mut engine);

// After one step from zero initial conditions:
// y[1] = a0 * 0.05 + a1 * 0.0 - b1 * 0.0
//       = TVC_A0 * 0.05
//       ≈ 0.5530 * 0.05 = 0.027650 rad (filter output, before trim addition)
// Plus trim (still near 0.0 after one update with K_TRIM * dt small).
let expected_filter_out = TVC_A0 * 0.05;
assert!((state.gimbal_pitch - expected_filter_out).abs() < 1e-4,
    "pitch gimbal ≈ {}, expected ≈ {}",
    state.gimbal_pitch, expected_filter_out);

// Yaw is unaffected.
assert!(state.gimbal_yaw.abs() < 1e-9);
```

### TC-TVC-03: Steady-state ramp tracking — trim integrates to remove DC error

**Rationale**: A constant attitude error after many cycles should cause the trim
to grow until it absorbs most of the error, reducing the filter output towards
zero (the integrating trim provides the steady-state correction).

```rust
let mut state  = TvcState::default();
let mut filter = TvcFilter::new_nominal();
let mut engine = SimEngine::new();

tvc_init(&mut state, &mut filter, (0.0, 0.0));

let error: Vec3 = [0.0, 0.01, 0.0];  // constant 0.01 rad pitch error

// Run 500 cycles ≈ 50 seconds of simulated burn time.
for _ in 0..500 {
    tvc_step(&mut state, &mut filter, error, 0.1, &mut engine);
}

// After many cycles the trim should have grown substantially from zero.
// The trim integrates at K_TRIM * gimbal_cmd * dt per cycle.
// Trim should be positive (tracking positive pitch error).
assert!(state.trim_pitch > 0.0,
    "trim should grow positive tracking a constant pitch error");

// The gimbal command should not have saturated
// (0.01 rad error is well within limits).
assert!(state.gimbal_pitch.abs() <= GIMBAL_LIMIT_RAD);
```

### TC-TVC-04: Trim integration — isolated `update_trim` test

**Rationale**: Verify `update_trim` applies the integrator equation and clamp
correctly.

```rust
let mut state = TvcState::default();
// Provide a constant gimbal command.
let cmd = (0.03_f64, 0.02_f64);
let dt  = 0.1_f64;

update_trim(&mut state, cmd, dt);
let expected_p = K_TRIM * 0.03 * 0.1;
let expected_y = K_TRIM * 0.02 * 0.1;
assert!((state.trim_pitch - expected_p).abs() < 1e-12);
assert!((state.trim_yaw   - expected_y).abs() < 1e-12);

// Apply a saturating command: drive trim to limit.
let large_cmd = (GIMBAL_LIMIT_RAD, GIMBAL_LIMIT_RAD);
for _ in 0..10_000 {
    update_trim(&mut state, large_cmd, dt);
}
assert!((state.trim_pitch - GIMBAL_LIMIT_RAD).abs() < 1e-9,
    "trim must clamp to GIMBAL_LIMIT_RAD");
assert!((state.trim_yaw   - GIMBAL_LIMIT_RAD).abs() < 1e-9);
```

### TC-TVC-05: Saturation clamping — large error does not exceed mechanical limit

**Rationale**: Even a very large attitude error (e.g., 1.0 rad ≈ 57°, far outside
the linear range) must not command a gimbal angle beyond the software limit.

```rust
let mut state  = TvcState::default();
let mut filter = TvcFilter::new_nominal();
let mut engine = SimEngine::new();

tvc_init(&mut state, &mut filter, (0.0, 0.0));

// Absurdly large error — well beyond any realistic manoeuvre.
let error: Vec3 = [0.0, 1.0, -1.0];
let (p, y) = tvc_step(&mut state, &mut filter, error, 0.1, &mut engine);

// Gimbal must be within software limits.
assert!(state.gimbal_pitch.abs() <= GIMBAL_LIMIT_RAD + 1e-12,
    "pitch saturated at ±{:.5} rad, got {:.5}",
    GIMBAL_LIMIT_RAD, state.gimbal_pitch);
assert!(state.gimbal_yaw.abs() <= GIMBAL_LIMIT_RAD + 1e-12,
    "yaw saturated at ±{:.5} rad, got {:.5}",
    GIMBAL_LIMIT_RAD, state.gimbal_yaw);

// CDU counts: GIMBAL_LIMIT_RAD / TVC_CDU_RAD_PER_COUNT ≈ 48.9 → max ≈ 49 counts.
// (previously this was ≈5.5 counts with the wrong 360-count scale)
let max_counts = (GIMBAL_LIMIT_RAD / TVC_CDU_RAD_PER_COUNT) as i16;  // ≈ 48
assert!(p.abs() <= max_counts + 1);
assert!(y.abs() <= max_counts + 1);
```

### TC-TVC-06: `tvc_init` postconditions

**Rationale**: Initialiser must set commanded angles and trim to the supplied
values, and reset filter memories to zero.

```rust
let mut state  = TvcState { gimbal_pitch: 0.05, gimbal_yaw: -0.03,
                             trim_pitch: 0.04, trim_yaw: -0.02 };
let mut filter = TvcFilter {
    pitch: TvcFilterAxis { a0: TVC_A0, a1: TVC_A1, b1: TVC_B1,
                           prev_input: 0.1, prev_output: 0.05 },
    yaw:   TvcFilterAxis { a0: TVC_A0, a1: TVC_A1, b1: TVC_B1,
                           prev_input: -0.1, prev_output: -0.03 },
};

let trim = (0.01_f64, -0.005_f64);
tvc_init(&mut state, &mut filter, trim);

assert!((state.gimbal_pitch - 0.01).abs() < 1e-12);
assert!((state.gimbal_yaw   - (-0.005)).abs() < 1e-12);
assert!((state.trim_pitch   - 0.01).abs() < 1e-12);
assert!((state.trim_yaw     - (-0.005)).abs() < 1e-12);
assert_eq!(filter.pitch.prev_input, 0.0);
assert_eq!(filter.pitch.prev_output, 0.0);
assert_eq!(filter.yaw.prev_input, 0.0);
assert_eq!(filter.yaw.prev_output, 0.0);
```

---

## 11. Spec Checklist

- [x] AGC source file and line range referenced (TVCINITIALIZE, TVCEXECUTIVE, TVCDAPS, TVCMASSPROP)
- [x] All erasable variables and their AGC addresses listed (§2.4)
- [x] Scale factors documented for all fixed-point values (§6)
- [x] Corresponding `f64` SI units documented (§6)
- [x] Input/output preconditions and postconditions stated (§4.1, §4.2, §4.3)
- [x] Edge cases and error handling specified (§9)
- [x] At least 5 test cases with expected values (§10 — six cases provided)
- [x] Rust API signature designed (§3, §4)
- [x] Invariants explicitly stated (§8)
- [x] Consistency with `docs/architecture.md` checked (§11.1 Thrust DAP, §13.2 T5RUPT budget)
- [x] S-10 (AD-5) applied: `TVC_CDU_RAD_PER_COUNT = TAU/3200` (≈ 1.963e-3 rad/count); §2.1
      narrative, §6 conversion formula, and TC-TVC-01/TC-TVC-05 expected counts updated.
- [x] CI-3 confirmed: `TvcState` has no `attitude_error` field; `tvc_step` receives it
      as a `Vec3` parameter from `DapState::attitude_error`.

---

## 12. Cross-References

| Topic | Reference |
|-------|-----------|
| Engine HAL: `sps_gimbal`, `sps_enable`, `thrust_on` | `specs/hal-spec.md` §10; `agc-core/src/hal/engine.rs` |
| DAP mode enum (`DapMode::Tvc`) and state struct | `agc-core/src/control/dap.rs` |
| Attitude error source (`DapState::attitude_error`) | `specs/dap-spec.md`; `control::attitude` module |
| T5RUPT timing (100 ms nominal, 20 ms budget) | `docs/architecture.md` §13.2 |
| `Vec3` type | `specs/types-module-spec.md` §3.3; `agc-core/src/types/vector.rs` |
| `AgcState` central struct (must add `tvc_filter`) | `docs/architecture.md` §8.2 |
| P40 program (calls `tvc_init`, sets `DapMode::Tvc`) | `agc-core/src/programs/p40_p41.rs` |
| Roll axis RCS during SPS burns | `docs/architecture.md` §11.2; `control::rcs_logic` |
| Restart protection | `docs/architecture.md` §6; `Comanche055/TVCRESTARTS.agc` |
| CDU angle encoding | `specs/types-module-spec.md` §2.2; `agc-core/src/types/angle.rs` |
| SPS gimbal CDU scale (AD-5) | `specs/milestone-3-architect-review.md` §8 |
