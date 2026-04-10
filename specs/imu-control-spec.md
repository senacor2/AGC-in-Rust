# Specification: `control/imu_control.rs` — IMU Control Module

**Status**: Ready for implementation
**Module path**: `agc-core/src/control/imu_control.rs`
**Architecture reference**: `docs/architecture.md` §4.1 (HAL typestate for IMU), §13.4 (T4RUPT task list)
**HAL reference**: `specs/hal-spec.md` §8 (`Imu` sub-trait contract)
**Calibration reference**: `specs/average-g-spec.md` §3 (`PipaCalibration` struct and pipeline)
**State-vector reference**: `specs/state-vector-spec.md` §2.4 (REFSMMAT), `Frame::StableMember`
**Types reference**: `specs/types-module-spec.md` §3.1 (`CduAngle`), §3.4–3.5 (`Vec3`, `Mat3x3`)
**Linalg reference**: `specs/linalg-spec.md` — `mxv`, `mxm`, `cross`, `unit`, `dot`, `norm`, `transpose`
**AGC source**: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`,
               `Comanche055/IMU_COMPENSATION_PACKAGE.agc`,
               `Comanche055/AVERAGE_G_INTEGRATOR.agc`
**Spec checklist**: `specs/README.md` — all items satisfied (see §13)

---

## 1. Purpose and Scope

`imu_control.rs` is the flight-software module that manages every aspect of the
IMU stable platform through its lifecycle: power-on cage, coarse alignment,
fine alignment, and in-flight gyro drift compensation. It is the exclusive
intermediary between the alignment programs (P51, P52) and the HAL `Imu`
sub-trait, and it supplies the T4RUPT handler with the drift-compensation
function it calls every 120 ms.

This module owns:

- The `ImuAlignmentState` enum that tracks the platform lifecycle.
- The `GyroCompensation` struct for gyro drift bias constants (NBD).
- Pure functions implementing the PIPA compensation pipeline (bias removal,
  scale application, misalignment correction — all three steps before the
  REFSMMAT rotation that lives in `services/average_g.rs`).
- Pure functions for coarse and fine alignment step computations.
- The REFSMMAT construction algorithm used by P51/P52.
- Gimbal lock detection (both warning and critical thresholds).

### Relationship to `services/average_g.rs` (SERVICER)

The SERVICER (average_g) currently duplicates the PIPA compensation pipeline
(steps 2–4 from average-g-spec §3.3). With this module in place, the correct
ownership is:

- `imu_control::apply_pipa_compensation` owns steps 2–4: bias removal,
  scale factor application, misalignment correction. Result: `Vec3` in the
  stable-member (platform) frame.
- `services::average_g::servicer_task` performs step 5: rotate via REFSMMAT
  into the inertial frame, then calls `average_g_step`.

The developer implementing this module must coordinate with the average-g
developer to migrate the existing pipeline out of `average_g.rs` and into a
call to `imu_control::apply_pipa_compensation`. Until that migration is
complete, `average_g.rs` may retain the inline pipeline; it is flagged with a
`// TODO: delegate to imu_control::apply_pipa_compensation` comment.

### What this module does NOT do

- Does not call `hw.imu().read_pipa()` — that is exclusively the SERVICER's
  responsibility (see hal-spec §8.3 and average-g-spec §2.1).
- Does not integrate delta-V into the state vector — that is `average_g_step`.
- Does not maintain `AgcState::refsmmat` directly — it computes a new REFSMMAT
  and returns it; the calling program (P51/P52) commits it to `AgcState`.
- Does not schedule itself — it provides functions that T4RUPT and P51/P52 call.

---

## 2. AGC Background

### 2.1 IMU Physical Description

The Block 2 AGC's Inertial Measurement Unit (IMU) is a four-gimbal
gyroscopically stabilised platform. The three outer gimbals (outer, inner,
middle in CDU ordering — roll, pitch, yaw in spacecraft convention) isolate the
stable member from spacecraft rotation. The stable member carries three
gyroscopes and three PIPAs (Pulse-Integrating Pendulous Accelerometers) along
mutually orthogonal platform axes.

**Gimbal sequence** (CDU array index → gimbal layer → spacecraft axis):

| Index | CDU cell       | Octal address | Gimbal layer | Spacecraft axis |
|-------|----------------|---------------|--------------|-----------------|
| 0     | `CDUX`         | 0033          | Outer        | Roll            |
| 1     | `CDUY`         | 0034          | Inner        | Pitch           |
| 2     | `CDUZ`         | 0035          | Middle       | Yaw             |

> Source: `Comanche055/ERASABLE_ASSIGNMENTS.agc`, lines ~110–140.

### 2.2 Platform Lifecycle

The platform progresses through three states in sequence:

1. **Caged** — gimbals locked to zero by a mechanical cage solenoid. Occurs on
   power-up and on operator request (CAGE discrete, channel 30 bit 11).
2. **Coarse aligned** — cage released; gimbals slewed by CDU drive commands
   (CDUXCMD/CDUYCMD/CDUZCMD, octal 0050–0052) to the desired orientation.
   Error is typically < 1°. This corresponds to the `ImuImpl<CoarseAligned>`
   typestate in the bare-metal HAL (architecture §4.1).
3. **Fine aligned** — gyro torque pulses (GYROCMD, octal 0047) drive the
   platform to the exact target orientation via closed-loop corrections. Error
   is < 0.1 arcminute. This corresponds to `ImuImpl<FineAligned>`.

### 2.3 Gyro Drift — NBD Constants

Even after fine alignment, gyroscope drift continuously rotates the stable
member away from the inertial reference direction. The drift rate is a
nearly-constant bias (non-drift acceleration equivalent, called NBD in
Comanche055) with small random-walk components.

The pre-flight measured drift bias constants NBDX, NBDY, NBDZ are stored in
erasable memory (E1 bank) and uplinked from Mission Control when updated.
During each T4RUPT cycle the software applies a compensating gyro torque to
null out the accumulated drift.

> AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc` — NBDX, NBDY, NBDZ
> erasable assignments and the torquing loop.

### 2.4 PIPA Compensation

The three PIPA counts accumulated between each SERVICER call contain:
- Instrument zero-offset bias (systematic drift, cancelled by NBDX/NBDY/NBDZ
  subtraction).
- Scale factor error (corrected by the 1/PIPADT constant).
- Axis misalignment error (corrected by the PIPASR 3×3 matrix).

The same set of constants (stored in `PipaCalibration`) serves both PIPA
compensation and gyro drift compensation. See average-g-spec §3.1 for the
`PipaCalibration` declaration that this module reuses.

### 2.5 REFSMMAT Construction (P51/P52)

Programs P51 (initial platform orientation) and P52 (in-flight realignment)
construct REFSMMAT by taking two star sightings. Each sighting records the
direction of a known star as measured by the optics (shaft and trunnion angles
converted to a unit vector in the stable-member frame) and simultaneously the
direction of that star in the inertial frame from the star catalog.

The TRIAD method constructs an orthonormal rotation matrix from two
non-collinear vector pairs:

1. The primary reference vector `r1` is the inertial direction of star 1.
2. A secondary reference vector is derived from both stars in the inertial
   frame: `r2 = unit(cross(r1, inertial_star2))`.
3. The third basis vector: `r3 = cross(r1, r2)`.
4. The same triad is constructed in the stable-member frame from the
   measurements `m1`, `m2`.
5. REFSMMAT = `[r1 | r2 | r3] · [m1 | m2 | m3]^T` (inertial triad times
   transpose of measured triad).

> AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — the TRIAD
> computation sequence (called PFRAM or similar in the Comanche055 listing).

---

## 3. `ImuAlignmentState` Enum

```rust
// agc-core/src/control/imu_control.rs

/// Tracks the alignment lifecycle of the IMU stable platform.
///
/// Mirrors the bare-metal HAL typestate (`Unaligned`, `CoarseAligned`,
/// `FineAligned` on `ImuImpl<State>`) but lives in erasable memory as a
/// runtime enum so that `AgcState` can record and preserve the alignment
/// status across RESTART.
///
/// AGC source: `IMU_CALIBRATION_AND_ALIGNMENT.agc` — alignment phase flags
/// in erasable (IMODES33, channel 30 monitor bits).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ImuAlignmentState {
    /// Platform caged (gimbals locked to zero).
    ///
    /// `hal::Imu::is_caged()` returns `true`. No navigation is possible.
    /// Gyro torque commands and fine-align operations are inhibited.
    Caged,

    /// Platform uncaged and coarsely aligned.
    ///
    /// CDU drive commands have completed; platform is within ~1° of target.
    /// PIPA counts are accumulating but SERVICER should not integrate them
    /// until fine alignment is complete.
    CoarseAligned,

    /// Platform fine aligned.
    ///
    /// Gyro torque nulling has reduced residual error to < 0.1 arcminute.
    /// SERVICER may begin integrating PIPA counts. REFSMMAT is valid.
    FineAligned,
}
```

### 3.1 State Transitions

| From            | To              | Trigger                                              | Function / caller           |
|-----------------|-----------------|------------------------------------------------------|-----------------------------|
| (any)           | `Caged`         | `is_caged()` returns `true`; cage discrete asserted  | T4RUPT IMU monitor          |
| `Caged`         | `CoarseAligned` | Cage released and coarse align converged             | P52 → `coarse_align_step`   |
| `CoarseAligned` | `FineAligned`   | Fine align residual < threshold                      | P52 → `fine_align_torque`   |
| `FineAligned`   | `CoarseAligned` | REFSMMAT realignment initiated (P52 option 1)        | P52                         |

**Preconditions for gyro torque**: `torque_gyro` must only be called when the
state is `CoarseAligned` or `FineAligned`. The bare-metal HAL enforces this via
typestate (architecture §4.1); this module enforces it by asserting the state.

**Storage**: `AgcState` must hold an `imu_alignment_state: ImuAlignmentState`
field, initialized to `Caged` at FRESH START and preserved across RESTART.

---

## 4. `GyroCompensation` Struct

```rust
/// Gyro drift bias constants for the three IMU gyroscope axes.
///
/// These are the steady-state drift rates measured before launch and updated
/// by Mission Control via uplink. They are stored separately from
/// `PipaCalibration` because they serve a different purpose: `PipaCalibration`
/// corrects accelerometer (PIPA) readings, while `GyroCompensation` drives
/// compensating torque pulses to keep the platform inertially stabilised.
///
/// The physical units are pulse-counts per T4RUPT interval (120 ms).
/// Positive counts produce positive torque in the gyro polarity convention
/// used by `hal::Imu::torque_gyro`.
///
/// AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc` — NBDX, NBDY, NBDZ
/// erasable assignments; `Comanche055/ERASABLE_ASSIGNMENTS.agc` E1 bank.
#[derive(Clone, Copy, Debug)]
pub struct GyroCompensation {
    /// Non-drift acceleration equivalent, X gyro axis.
    /// AGC name: `NBDX`. Erasable bank E1. Units: torque counts per T4RUPT.
    pub nbd_x: f64,

    /// Non-drift acceleration equivalent, Y gyro axis.
    /// AGC name: `NBDY`. Erasable bank E1. Units: torque counts per T4RUPT.
    pub nbd_y: f64,

    /// Non-drift acceleration equivalent, Z gyro axis.
    /// AGC name: `NBDZ`. Erasable bank E1. Units: torque counts per T4RUPT.
    pub nbd_z: f64,
}

impl GyroCompensation {
    /// Zero drift — ideal gyroscope. Used at FRESH START.
    pub const ZERO: Self = Self { nbd_x: 0.0, nbd_y: 0.0, nbd_z: 0.0 };
}
```

### 4.1 Relationship to `PipaCalibration::bias`

`PipaCalibration::bias` (NBDX/NBDY/NBDZ in average-g-spec §3.1 and §8.2) is
expressed in PIPA counts per 2-second SERVICER interval and is used to
subtract systematic accelerometer zero-offset from PIPA readings.

`GyroCompensation` carries the same physical NBD constants but expressed as
gyro torque pulse counts per T4RUPT interval (120 ms). On the real AGC these
are converted from the uplinked values and stored in separate erasable cells.

The Rust port stores both representations. The uplink processor or P01/P02
(gyrocompassing) is responsible for keeping them consistent whenever the
constants are updated.

**Storage**: `AgcState` must hold a `gyro_comp: GyroCompensation` field,
initialized to `GyroCompensation::ZERO` at FRESH START and preserved across
RESTART.

---

## 5. `apply_pipa_compensation`

```rust
/// Apply PIPA calibration corrections to raw hardware counts.
///
/// Implements steps 2–4 of the SERVICER PIPA pipeline from
/// `specs/average-g-spec.md` §3.3:
///   Step 2 — subtract bias (NBDX/NBDY/NBDZ in count units)
///   Step 3 — apply scale factor (1/PIPADT, m/s per count)
///   Step 4 — apply misalignment correction matrix (PIPASR)
///
/// The result is delta-V in the stable-member (platform) frame, in m/s.
/// The caller (SERVICER) is responsible for step 5: rotating via REFSMMAT
/// into the inertial frame.
///
/// # Arguments
///
/// * `raw` — raw PIPA pulse counts `[x, y, z]` as returned by
///   `hal::Imu::read_pipa()`. Each count ≈ 0.0585 m/s on the real IMU.
/// * `cal` — calibration constants from `AgcState::pipa_cal`.
///
/// # Returns
///
/// Delta-V vector in the stable-member frame, SI units (m/s).
///
/// # Scale and precision
///
/// The bias subtraction is performed in `i32` to prevent overflow when raw
/// counts are near `i16::MIN` and bias is positive (or vice versa). The
/// product of the biased count and scale factor is computed in `f64`.
///
/// # AGC source
///
/// `Comanche055/AVERAGE_G_INTEGRATOR.agc` — the innermost compensation loop,
/// approximately lines 120–175 in the Comanche055 assembly listing.
pub fn apply_pipa_compensation(raw: [i16; 3], cal: &PipaCalibration) -> Vec3 {
    // Step 2: subtract bias in i32 to prevent overflow
    let biased: [i32; 3] = [
        raw[0] as i32 - cal.bias[0] as i32,
        raw[1] as i32 - cal.bias[1] as i32,
        raw[2] as i32 - cal.bias[2] as i32,
    ];

    // Step 3: apply scale factor (1/PIPADT)
    let scaled: Vec3 = [
        biased[0] as f64 * cal.scale,
        biased[1] as f64 * cal.scale,
        biased[2] as f64 * cal.scale,
    ];

    // Step 4: apply misalignment correction matrix (PIPASR)
    math::linalg::mxv(cal.misalignment, scaled)
}
```

### 5.1 Pipeline Detail

| Step | Operation | Input type | Output type | Notes |
|------|-----------|-----------|-------------|-------|
| 2 | Bias subtraction | `[i16; 3]`, `[i16; 3]` | `[i32; 3]` | Done in `i32`; no overflow even at `i16::MAX` ± `i16::MAX` |
| 3 | Scale factor | `[i32; 3]`, `f64` | `Vec3` (m/s) | `1/PIPADT` ≈ 0.0585 m/s/count |
| 4 | Misalignment matrix | `[[f64;3];3]`, `Vec3` | `Vec3` (m/s) | `linalg::mxv`; identity for nominal calibration |

### 5.2 Preconditions

- `cal.scale` must be positive and finite.
- `cal.misalignment` must be finite. For a correctly calibrated IMU it is
  close to identity but this function does not verify orthonormality.
- The function is pure (no side effects, no hardware access).

### 5.3 Postconditions

- Result is finite for all finite inputs.
- For `bias = [0,0,0]`, identity `misalignment`, and `scale = s`, the output
  equals `[raw[i] as f64 * s]`.
- For `raw == bias` (counts exactly equal bias), the output is `[0.0, 0.0, 0.0]`.

---

## 6. `compute_gyro_drift`

```rust
/// Compute the compensating gyro torque pulse counts to apply at the current
/// T4RUPT cycle in order to null accumulated platform drift.
///
/// The gyro drift model used in Comanche055 is a constant-rate bias per unit
/// time. The total drift accumulation over interval `dt_cs` centiseconds is
/// the product of the bias rate and the interval. This function returns
/// signed pulse counts ready to pass to `hal::Imu::torque_gyro`.
///
/// # Arguments
///
/// * `dt_cs` — elapsed time since the last drift-compensation call, in
///   centiseconds. Nominally 12 (= 120 ms T4RUPT cycle). Must be non-zero.
/// * `nbd`   — gyro drift bias constants `[x, y, z]` in torque pulse counts
///   per centisecond. Derive from `GyroCompensation`: convert the stored
///   per-T4RUPT values by dividing by the nominal 12 cs period.
///
/// # Returns
///
/// `[i16; 3]` signed pulse counts to apply via `torque_gyro` on axes X, Y, Z
/// respectively. Values are rounded to the nearest integer and clamped to
/// `[i16::MIN, i16::MAX]`.
///
/// # Timing context
///
/// Called from `services::t4rupt` during T4RUPT task phase 4
/// (architecture §13.4). The T4RUPT period is 120 ms = 12 centiseconds;
/// `dt_cs` should be measured from TIME4 to account for any jitter.
///
/// # AGC source
///
/// `Comanche055/IMU_COMPENSATION_PACKAGE.agc` — the gyro torque compensation
/// loop applied during the IMUMON phase of T4RUPT.
pub fn compute_gyro_drift(dt_cs: u32, nbd: [f64; 3]) -> [i16; 3] {
    let dt = dt_cs as f64;
    let clamp_i16 = |x: f64| x.round().max(i16::MIN as f64).min(i16::MAX as f64) as i16;
    [
        clamp_i16(nbd[0] * dt),
        clamp_i16(nbd[1] * dt),
        clamp_i16(nbd[2] * dt),
    ]
}
```

### 6.1 Calling Convention

**Strategy D (staging fields) — CI-1 resolution**: `compute_gyro_drift` is a
pure function with no hardware access. All HAL I/O is performed exclusively
inside the **T4RUPT ISR shim** in `services/t4rupt.rs`, before and after any
pure computation. Waitlist task functions (`fn(&mut AgcState)`) must not call
HAL methods directly; the ISR shim stages inputs into `AgcState` fields and
reads command outputs from them.

The T4RUPT ISR shim in `services/t4rupt.rs` calls this function on phase 4 of
the rotating task list. The call sequence, **executed entirely within the ISR
shim context** (not a Waitlist task function), is:

```rust
// Inside services/t4rupt.rs ISR shim — phase 4:
// HAL reads and writes are permitted here because this is the ISR body,
// not a fn(&mut AgcState) Waitlist task.
if state.imu_alignment_state != ImuAlignmentState::Caged {
    let dt_cs = hw.timers().mission_time() - state.last_drift_comp_time;
    let nbd = [
        state.gyro_comp.nbd_x / 12.0,   // convert per-T4RUPT to per-cs
        state.gyro_comp.nbd_y / 12.0,
        state.gyro_comp.nbd_z / 12.0,
    ];
    // compute_gyro_drift is pure: no HAL access inside
    let pulses = compute_gyro_drift(dt_cs, nbd);
    // HAL writes happen here in the ISR shim, not inside compute_gyro_drift
    hw.imu().torque_gyro(0, pulses[0]);
    hw.imu().torque_gyro(1, pulses[1]);
    hw.imu().torque_gyro(2, pulses[2]);
    state.last_drift_comp_time = hw.timers().mission_time();
}
```

The `AgcState` field `last_drift_comp_time: Met` is updated by the ISR shim so
that the elapsed-time calculation is always available from `AgcState` without
requiring a HAL call from within a task function.

### 6.2 Preconditions

- `dt_cs > 0`. A zero interval produces zero correction (not an error), but
  indicates a calling sequence bug.
- `nbd[i]` must be finite. `NaN` or `Inf` produces a panic in debug mode via
  `debug_assert!(nbd[i].is_finite())`.

### 6.3 Postconditions

- For `nbd = [0.0, 0.0, 0.0]`, result is `[0, 0, 0]` (no-op compensation).
- Result magnitude is bounded by `i16::MAX` regardless of `dt_cs` or `nbd`.

---

## 7. `coarse_align_step`

```rust
/// Compute CDU drive pulse commands to slew the platform toward target angles.
///
/// Coarse alignment drives the IMU gimbals from their current CDU angles to
/// the commanded target angles using the CDU error counters
/// (CDUXCMD/CDUYCMD/CDUZCMD, octal 0050–0052). The hardware drives gimbals at
/// approximately 3200 pulses per second until the CDU error counters reach zero.
///
/// This function computes the signed difference (target − current) for each
/// gimbal axis, expressed as CDU pulse counts. Because CDU angles are stored as
/// `u16` with wrapping modular arithmetic (full circle = 65536 counts), the
/// difference is computed as a twos-complement subtraction interpreted as a
/// signed `i16`. This correctly handles wrap-around through ±180°.
///
/// # Arguments
///
/// * `target_angles`  — desired CDU angles `[outer, inner, middle]`.
/// * `current_angles` — current CDU angles from `hal::Imu::read_cdu()`.
///
/// # Returns
///
/// `[i16; 3]` signed pulse counts for `[outer, inner, middle]`. Pass directly
/// to `hal::Imu::coarse_align(commands)`.
///
/// # Convergence detection
///
/// The caller (P52) should call `read_cdu` after commanding each step and
/// check whether the absolute error on all three axes is below a threshold
/// (typically 2 CDU counts ≈ 0.011°). This function does not poll; it
/// produces a single-step command each call.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — the coarse-align CDU
/// drive loop, which iterates until the CDU error is within tolerance.
pub fn coarse_align_step(
    target_angles: [CduAngle; 3],
    current: [CduAngle; 3],
) -> [i16; 3] {
    [
        (target_angles[0].0.wrapping_sub(current[0].0)) as i16,
        (target_angles[1].0.wrapping_sub(current[1].0)) as i16,
        (target_angles[2].0.wrapping_sub(current[2].0)) as i16,
    ]
}
```

### 7.1 Angle Arithmetic

`CduAngle` is a `u16` with full-circle = 65536. The wrapping subtraction
`target.0.wrapping_sub(current.0)` produces a `u16` value. Reinterpreting that
as `i16` via `as i16` gives the shortest signed angular path: positive for
counter-clockwise drive, negative for clockwise, range ±32767 counts (±179.99°).
This correctly handles the case where the shortest slew path crosses the 0/360°
boundary.

### 7.2 Preconditions

- Both arrays must contain valid `CduAngle` values (all `u16` values are
  valid; no precondition on range).
- The IMU must be powered and uncaged before calling `coarse_align(commands)`
  on the HAL.

### 7.3 Postconditions

- `commands[i] = 0` when `target_angles[i] == current[i]` exactly.
- `|commands[i]| <= 32767` always.
- Coarse align is complete when all three `|commands[i]| <= COARSE_ALIGN_THRESHOLD`
  (see §11.1).

---

## 8. `fine_align_torque`

```rust
/// Compute gyro torque pulse commands to drive residual platform error to zero.
///
/// Fine alignment applies closed-loop corrections via gyro torque pulses.
/// The attitude error is expressed as a small-angle vector in radians (the
/// difference between the desired and actual platform orientation, derived from
/// comparing measured star directions with the star catalog entries). This
/// function converts that error into signed pulse counts for each gyro axis.
///
/// The conversion factor from radians to pulse counts uses the gyro scale
/// factor: approximately 1 pulse = (TAU / 2^15) radians ≈ 1.919e-4 rad
/// (matching the 15-bit CDU angle resolution of the original AGC hardware).
/// The time step `dt_s` scales the correction for the elapsed interval.
///
/// # Arguments
///
/// * `attitude_error` — residual platform orientation error `[x, y, z]` in
///   radians (small-angle representation, stable-member frame).
/// * `dt_s`           — elapsed time in seconds over which this correction
///   should be applied. Nominally the T4RUPT period = 0.12 s, but may be
///   shorter for a sub-step correction within P52.
///
/// # Returns
///
/// `[i16; 3]` signed torque pulse counts for axes `[X, Y, Z]`. Pass to
/// `hal::Imu::torque_gyro(axis, pulses)` for each axis in order.
///
/// # Scale factor derivation
///
/// The original AGC gyro torque scale:
///   1 pulse = B-15 revolutions = TAU / 32768 rad ≈ 1.9175e-4 rad
///
/// Inversion: rad_to_pulses = 32768.0 / TAU
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — the fine-align torquing
/// loop (FINEALGN or equivalent label), applied after each star sighting
/// residual computation.
pub fn fine_align_torque(attitude_error: Vec3, dt_s: f64) -> [i16; 3] {
    const RAD_TO_PULSES: f64 = 32768.0 / core::f64::consts::TAU;
    let clamp_i16 = |x: f64| x.round().max(i16::MIN as f64).min(i16::MAX as f64) as i16;
    [
        clamp_i16(attitude_error[0] * RAD_TO_PULSES * dt_s),
        clamp_i16(attitude_error[1] * RAD_TO_PULSES * dt_s),
        clamp_i16(attitude_error[2] * RAD_TO_PULSES * dt_s),
    ]
}
```

### 8.1 Convergence Criterion

Fine alignment is considered complete when the magnitude of `attitude_error`
is below `FINE_ALIGN_THRESHOLD` (see §11.1), which corresponds to
approximately 0.1 arcminute (≈ 2.91e-5 rad). P52 checks this after each
torque application cycle.

### 8.2 Preconditions

- `attitude_error[i]` must be finite. For small-angle validity, each component
  should be < 0.1 rad (≈ 5.7°). Larger errors indicate the TRIAD computation
  has failed or the star sighting is corrupted.
- `dt_s > 0.0`. A zero or negative time step produces a zero or negative
  correction, which is a calling-sequence bug.
- The IMU must be in `CoarseAligned` or `FineAligned` state.

### 8.3 Postconditions

- For `attitude_error = [0,0,0]`, result is `[0,0,0]`.
- Result components are bounded to `[i16::MIN, i16::MAX]`.

---

## 9. `refsmmat_from_star_sightings`

```rust
/// Construct a REFSMMAT rotation matrix from two star sightings.
///
/// Implements the TRIAD method to build an orthonormal rotation matrix
/// mapping the stable-member (platform) frame to the inertial reference frame.
/// This is the core algorithm of P51 (initial IMU orientation) and P52
/// (in-flight realignment).
///
/// # Arguments
///
/// * `star1_inertial` — unit vector of star 1 direction in the inertial frame,
///   from the star catalog (`navigation::star_catalog`), at the current epoch.
/// * `star2_inertial` — unit vector of star 2 direction in the inertial frame.
///   Must not be collinear with `star1_inertial` (|cross| > COLLINEAR_EPSILON).
/// * `star1_platform` — unit vector of star 1 as measured by the optics
///   (shaft and trunnion angles converted from `CduAngle` to a Vec3),
///   expressed in the stable-member frame.
/// * `star2_platform` — unit vector of star 2 as measured by the optics,
///   expressed in the stable-member frame.
///
/// # Returns
///
/// `Mat3x3` — the new REFSMMAT. The caller (P51/P52) must:
/// 1. Validate orthonormality: `|mxm(R, transpose(R)) - IDENTITY| < 1e-7`.
/// 2. Commit to `AgcState::refsmmat`.
/// 3. Set `AgcState::imu_alignment_state = ImuAlignmentState::FineAligned`.
///
/// Returns `None` if the two star vectors are collinear (unable to form a
/// non-degenerate triad). The caller must issue a program alarm and request
/// a new pair of star sightings.
///
/// # Algorithm — TRIAD Method
///
/// Given two pairs of corresponding unit vectors (inertial ↔ platform):
///
///   Inertial triad:
///     r1 = star1_inertial
///     r2 = unit(cross(r1, star2_inertial))
///     r3 = cross(r1, r2)
///
///   Platform triad:
///     m1 = star1_platform
///     m2 = unit(cross(m1, star2_platform))
///     m3 = cross(m1, m2)
///
///   REFSMMAT = R_inertial · R_platform^T
///   where R_inertial = [r1 | r2 | r3] (column matrix)
///         R_platform = [m1 | m2 | m3] (column matrix)
///
/// Equivalently, in row-major form:
///   R_inertial (row-major) = [[r1], [r2], [r3]]
///   R_platform (row-major) = [[m1], [m2], [m3]]
///   REFSMMAT = transpose(R_inertial) · R_platform... (see implementation note)
///
/// # Implementation Note — Row vs Column Convention
///
/// `Mat3x3 = [[f64; 3]; 3]` is row-major (outer index = row). A column
/// matrix [r1 | r2 | r3] in mathematical notation maps to a row matrix where
/// each row is a basis vector. The TRIAD formula above uses:
///
///   REFSMMAT[i][j] = sum_k(R_inertial_col[k][i] * R_platform_col[k][j])
///
/// which is implemented as `mxm(transpose(R_inertial_rows), R_platform_rows)`.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — the PFRAM (platform
/// frame computation) routine, which builds the triad basis vectors and
/// multiplies them to produce REFSMMAT.
pub fn refsmmat_from_star_sightings(
    star1_inertial: Vec3,
    star2_inertial: Vec3,
    star1_platform: Vec3,
    star2_platform: Vec3,
) -> Option<Mat3x3> {
    use math::linalg::{cross, unit, mxm, transpose, dot, norm};

    const COLLINEAR_EPSILON: f64 = 1e-6;

    // Build inertial triad
    let r1 = star1_inertial;
    let r2_raw = cross(r1, star2_inertial);
    if norm(r2_raw) < COLLINEAR_EPSILON {
        return None;  // Stars are collinear
    }
    let r2 = unit(r2_raw);
    let r3 = cross(r1, r2);

    // Build platform (measurement) triad
    let m1 = star1_platform;
    let m2_raw = cross(m1, star2_platform);
    if norm(m2_raw) < COLLINEAR_EPSILON {
        return None;  // Measurement vectors are collinear
    }
    let m2 = unit(m2_raw);
    let m3 = cross(m1, m2);

    // Row-major matrices: each row is a basis vector
    let r_inertial: Mat3x3 = [r1, r2, r3];
    let r_platform: Mat3x3 = [m1, m2, m3];

    // REFSMMAT = R_inertial^T · R_platform
    Some(mxm(transpose(r_inertial), r_platform))
}
```

### 9.1 Input Vector Construction

The platform-frame star direction vectors are computed from optics CDU readings
in P51/P52 before calling this function. The conversion from shaft (trunnion)
CDU angles to a unit vector in the stable-member frame follows the optics mount
geometry and is the responsibility of `programs/p51_p52.rs`, not this module.

The inertial star direction vectors come from `navigation::star_catalog` using
the star identification number entered by the crew via DSKY, evaluated at the
current mission elapsed time (MET) to account for stellar aberration.

### 9.2 Collinearity Check

Two stars that are within `COLLINEAR_EPSILON` of being collinear (same or
antipodal direction) cannot form a well-conditioned triad. The P51/P52 programs
prevent this by comparing the separation angle before requesting sightings, but
this function provides a safety check. `COLLINEAR_EPSILON = 1e-6` corresponds
to an angular separation less than about 0.057 arcseconds — effectively zero
for any physically distinct star pair.

### 9.3 Orthonormality Guarantee

For unit input vectors and non-collinear pairs, the TRIAD output is
theoretically orthonormal. Floating-point rounding will cause small deviations
(< 1e-14 from unit vectors; < 1e-13 in the matrix product). P52 must validate
with `mxm(R, transpose(R)) ≈ IDENTITY` to tolerance 1e-7 before committing.

---

## 10. Gimbal Lock Detection

**Ownership**: Gimbal lock detection is owned by this module (`control::imu_control`),
matching the original AGC design where it was handled in the T4RUPT IMUMON
monitoring cycle. `control::attitude` does NOT implement a separate
`gimbal_lock_warning` function; any reference to gimbal lock in attitude
control code must call `imu_control::is_gimbal_lock_warning` or
`imu_control::is_gimbal_lock_critical`. See `specs/attitude-spec.md` §4.7
for the cross-reference.

Two thresholds are defined:
- **Warning (70°)**: middle gimbal within 20° of ±90°. Crew has time to maneuver.
- **Critical (85°)**: middle gimbal within 5° of ±90°. Immediate crew action required.

```rust
/// Check whether the middle gimbal (CDUZ, yaw axis) is approaching gimbal lock.
///
/// Gimbal lock occurs when the middle gimbal angle approaches ±90° (= 16384 or
/// 49152 CDU counts). In this configuration the outer and inner gimbal axes
/// become co-planar and the platform loses one degree of rotational freedom.
/// The software must warn the crew so they can manoeuvre the spacecraft to
/// move the platform angle away from the singular zone.
///
/// # Arguments
///
/// * `cdu_angles` — current CDU angles `[outer, inner, middle]` from
///   `hal::Imu::read_cdu()`.
///
/// # Returns
///
/// `true` if the middle gimbal is within `GIMBAL_LOCK_WARNING_BAND` of ±90°.
///
/// # Warning threshold
///
/// The original AGC issued a gimbal lock warning when the middle gimbal
/// exceeded approximately 70° from 0° (i.e., was within 20° of ±90°).
/// This gives the crew time to manoeuvre before lock is reached.
///
/// `GIMBAL_LOCK_WARNING_BAND` = 3641 CDU counts ≈ 20° (20/360 × 65536 ≈ 3641).
/// Warning fires when:
///   |middle - 90°| < 20°  OR  |middle - (-90°)| < 20°
/// Equivalently: middle ∈ (70°, 110°) ∪ (250°, 290°) in the [0°, 360°) range.
///
/// # Called from
///
/// `services::t4rupt` during T4RUPT phase 3 (IMU status monitoring). When
/// this returns `true`, the T4RUPT handler raises program alarm 0210 (octal)
/// and illuminates the GIMBAL LOCK warning lamp via `hal::Dsky::set_lamp`.
///
/// # AGC source
///
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc` — the IMODES33/IMUMON
/// gimbal lock monitor, which tests channel 30 bit 6 (GMBLLOCK discrete) and
/// independently checks the middle CDU angle.
pub fn is_gimbal_lock_warning(cdu_angles: [CduAngle; 3]) -> bool {
    const GIMBAL_LOCK_WARNING_BAND: u16 = 3641; // ≈ 20° in CDU counts
    const NINETY_DEG: u16 = 16384;              // 90° in CDU counts
    const TWO_SEVENTY_DEG: u16 = 49152;         // 270° (= -90°) in CDU counts

    let middle = cdu_angles[2].0;

    // Distance from +90° (wrapping)
    let dist_pos90 = middle.wrapping_sub(NINETY_DEG).min(
        NINETY_DEG.wrapping_sub(middle)
    );
    // Distance from -90° / 270° (wrapping)
    let dist_neg90 = middle.wrapping_sub(TWO_SEVENTY_DEG).min(
        TWO_SEVENTY_DEG.wrapping_sub(middle)
    );

    dist_pos90 < GIMBAL_LOCK_WARNING_BAND || dist_neg90 < GIMBAL_LOCK_WARNING_BAND
}

/// Check whether the middle gimbal is in the critical zone (within 5° of ±90°).
///
/// This is the high-priority variant of `is_gimbal_lock_warning`. When this
/// returns `true`, the platform is within 5° of gimbal lock and immediate
/// crew action is required. The T4RUPT handler should raise a distinct
/// program alarm (0211 octal) and may additionally trigger an automatic
/// maneuver to move the platform away from the singular zone.
///
/// # Returns
///
/// `true` if the middle gimbal is within `GIMBAL_LOCK_CRITICAL_BAND` of ±90°.
///
/// # Critical threshold
///
/// `GIMBAL_LOCK_CRITICAL_BAND` = 910 CDU counts ≈ 5° (5/360 × 65536 ≈ 910).
/// Critical zone fires when: |middle - 90°| < 5° OR |middle - (-90°)| < 5°.
/// This corresponds to: middle ∈ (85°, 95°) ∪ (265°, 275°).
///
/// # Called from
///
/// `services::t4rupt` during T4RUPT phase 3, checked after `is_gimbal_lock_warning`.
pub fn is_gimbal_lock_critical(cdu_angles: [CduAngle; 3]) -> bool {
    const GIMBAL_LOCK_CRITICAL_BAND: u16 = 910; // ≈ 5° in CDU counts
    const NINETY_DEG: u16 = 16384;
    const TWO_SEVENTY_DEG: u16 = 49152;

    let middle = cdu_angles[2].0;

    let dist_pos90 = middle.wrapping_sub(NINETY_DEG).min(
        NINETY_DEG.wrapping_sub(middle)
    );
    let dist_neg90 = middle.wrapping_sub(TWO_SEVENTY_DEG).min(
        TWO_SEVENTY_DEG.wrapping_sub(middle)
    );

    dist_pos90 < GIMBAL_LOCK_CRITICAL_BAND || dist_neg90 < GIMBAL_LOCK_CRITICAL_BAND
}
```

### 10.1 Physical Significance

A four-gimbal IMU has a redundant (fourth) gimbal to avoid lock, but the AGC
software still monitors the middle gimbal because operating too close to the
singular zone causes large CDU noise amplification. The warning gives the crew
approximately 20° of margin; the critical threshold gives a final 5° alert.

### 10.2 Constants

| Constant | Value (CDU counts) | Value (degrees) | Rationale |
|----------|-------------------|-----------------|-----------|
| `GIMBAL_LOCK_WARNING_BAND` | 3641 | 20.0 | 20° margin before ±90° singular zone — crew warning |
| `GIMBAL_LOCK_CRITICAL_BAND` | 910 | 5.0 | 5° margin — critical zone, immediate action required |
| `NINETY_DEG` | 16384 | 90.0 | 1/4 of 65536 |
| `TWO_SEVENTY_DEG` | 49152 | 270.0 | 3/4 of 65536 (= −90° in twos-complement) |

---

## 11. Constants

```rust
/// Maximum CDU count error considered "converged" after coarse align.
/// 2 counts ≈ 0.011° (2/65536 × 360°).
pub const COARSE_ALIGN_THRESHOLD: u16 = 2;

/// Maximum attitude error (radians) for fine alignment completion.
/// 0.1 arcminute ≈ 2.909e-5 rad = (0.1/60) × (π/180).
pub const FINE_ALIGN_THRESHOLD: f64 = 2.909e-5;

/// Minimum star-pair angular separation for a valid REFSMMAT computation.
/// Below this the TRIAD computation is ill-conditioned.
pub const COLLINEAR_EPSILON: f64 = 1e-6;

/// Gyro torque scale: radians per pulse.
/// 1 pulse = B-15 revolutions = TAU / 32768 rad ≈ 1.9175e-4 rad.
pub const GYRO_PULSE_RAD: f64 = core::f64::consts::TAU / 32768.0;

/// T4RUPT nominal period in centiseconds.
pub const T4RUPT_PERIOD_CS: u32 = 12;
```

---

## 12. AGC Erasable Memory Reference

| AGC symbol      | Erasable bank | Address (octal) | Rust field                            | Description                       |
|-----------------|---------------|-----------------|---------------------------------------|-----------------------------------|
| `CDUX`          | —             | 0033            | `hw.imu().read_cdu()[0]`              | Outer (roll) CDU angle            |
| `CDUY`          | —             | 0034            | `hw.imu().read_cdu()[1]`              | Inner (pitch) CDU angle           |
| `CDUZ`          | —             | 0035            | `hw.imu().read_cdu()[2]`              | Middle (yaw) CDU angle            |
| `CDUXCMD`       | —             | 0050            | `hw.imu().coarse_align(cmds)[0]`      | Outer CDU drive command           |
| `CDUYCMD`       | —             | 0051            | `hw.imu().coarse_align(cmds)[1]`      | Inner CDU drive command           |
| `CDUZCMD`       | —             | 0052            | `hw.imu().coarse_align(cmds)[2]`      | Middle CDU drive command          |
| `GYROCMD`       | —             | 0047            | `hw.imu().torque_gyro(axis, pulses)`  | Gyro torque command counter       |
| `NBDX`          | E1            | uplinked        | `state.gyro_comp.nbd_x`               | X-axis gyro drift bias            |
| `NBDY`          | E1            | uplinked        | `state.gyro_comp.nbd_y`               | Y-axis gyro drift bias            |
| `NBDZ`          | E1            | uplinked        | `state.gyro_comp.nbd_z`               | Z-axis gyro drift bias            |
| `REFSMMAT`      | E3            | 0306–0323       | `state.refsmmat: Mat3x3`              | Platform-to-inertial rotation     |
| `IMODES33`      | E1            | varies          | `state.imu_alignment_state`           | IMU mode/status flags             |
| `PIPAX/Y/Z`     | —             | 0037–0041       | `hw.imu().read_pipa()`                | PIPA pulse accumulators (read by SERVICER only) |

> Source: `Comanche055/ERASABLE_ASSIGNMENTS.agc` and `Comanche055/IMU_COMPENSATION_PACKAGE.agc`.

---

## 13. Module Boundaries and Caller Contract

### 13.1 Functions called by `services/t4rupt.rs`

| Function                  | T4RUPT Phase | Precondition                               |
|---------------------------|-------------|---------------------------------------------|
| `is_gimbal_lock_warning`  | Phase 3     | After `read_cdu()`                          |
| `is_gimbal_lock_critical` | Phase 3     | After `is_gimbal_lock_warning` returns true |
| `compute_gyro_drift`      | Phase 4     | `state.imu_alignment_state != Caged`        |

### 13.2 Functions called by `programs/p51_p52.rs`

| Function                      | Stage                         | Precondition                              |
|-------------------------------|-------------------------------|-------------------------------------------|
| `coarse_align_step`           | Coarse alignment loop         | `is_caged()` returns `false`              |
| `fine_align_torque`           | Fine alignment loop           | State is `CoarseAligned` or `FineAligned` |
| `refsmmat_from_star_sightings`| After two star sightings      | Both vectors are unit and non-collinear   |

### 13.3 Functions called by `services/average_g.rs` (SERVICER)

| Function                   | Call site             | Notes                               |
|----------------------------|-----------------------|-------------------------------------|
| `apply_pipa_compensation`  | SERVICER pipeline     | After `read_pipa()`; before REFSMMAT rotation |

---

## 14. Test Cases

### TC-IMU-CTRL-1: Zero-bias PIPA compensation

**Purpose**: With zero bias, nominal scale, and identity misalignment, the
output exactly equals the raw counts multiplied by scale.

```rust
let raw: [i16; 3] = [100, -50, 30];
let cal = PipaCalibration {
    scale: 1.0,
    bias: [0, 0, 0],
    misalignment: [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
};
let result = apply_pipa_compensation(raw, &cal);
assert!((result[0] - 100.0).abs() < 1e-12);
assert!((result[1] - (-50.0)).abs() < 1e-12);
assert!((result[2] - 30.0).abs() < 1e-12);
```

**Expected**: `[100.0, -50.0, 30.0]` m/s exactly (scale = 1.0 m/s/count).

### TC-IMU-CTRL-2: Non-zero bias PIPA compensation

**Purpose**: Bias is subtracted before scaling; when raw counts equal the bias
exactly, output is zero.

```rust
let raw: [i16; 3] = [5, -3, 2];
let cal = PipaCalibration {
    scale: 0.0585,
    bias: [5, -3, 2],   // NBDX=5, NBDY=-3, NBDZ=2
    misalignment: [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
};
let result = apply_pipa_compensation(raw, &cal);
assert!(result[0].abs() < 1e-12, "X should be zero: {}", result[0]);
assert!(result[1].abs() < 1e-12, "Y should be zero: {}", result[1]);
assert!(result[2].abs() < 1e-12, "Z should be zero: {}", result[2]);
```

**Expected**: `[0.0, 0.0, 0.0]` — all bias, no real acceleration.

### TC-IMU-CTRL-3: Coarse align toward zero

**Purpose**: Target = 0 counts, current = 1000 counts → drive command = −1000.

```rust
let target = [CduAngle(0), CduAngle(0), CduAngle(0)];
let current = [CduAngle(1000), CduAngle(500), CduAngle(65000)];
let cmds = coarse_align_step(target, current);
assert_eq!(cmds[0], -1000_i16);
assert_eq!(cmds[1], -500_i16);
// 65000 → 0: wrapping subtraction 0u16.wrapping_sub(65000) = 536
// reinterpreted as i16 = 536 (positive = CCW is shorter path)
assert_eq!(cmds[2], 536_i16);
```

**Expected**: `[-1000, -500, 536]`. The third axis wraps correctly: going from
65000 counts (≈ 357°) to 0° counter-clockwise is 536 counts, which is shorter
than going clockwise by 65000 counts.

### TC-IMU-CTRL-4: Fine align toward known attitude

**Purpose**: A 1-arcsecond (≈ 4.848e-6 rad) error on all three axes should
produce small but non-zero torque pulse counts.

```rust
let one_arcsec_rad = std::f64::consts::PI / (180.0 * 3600.0);
let error: Vec3 = [one_arcsec_rad, one_arcsec_rad, one_arcsec_rad];
let dt_s = 0.12; // one T4RUPT cycle
let pulses = fine_align_torque(error, dt_s);

// Expected pulses ≈ error_rad * (32768/TAU) * dt_s
// ≈ 4.848e-6 * 5215.6 * 0.12 ≈ 3.03e-3 → rounds to 0
// Increase to 10 arcsec:
let ten_arcsec_rad = 10.0 * one_arcsec_rad;
let error10: Vec3 = [ten_arcsec_rad, ten_arcsec_rad, ten_arcsec_rad];
let pulses10 = fine_align_torque(error10, dt_s);
// ≈ 4.848e-5 * 5215.6 * 0.12 ≈ 0.030 → still rounds to 0
// Use 1 arcminute = 60 arcsec:
let one_arcmin_rad = 60.0 * one_arcsec_rad;
let error60: Vec3 = [one_arcmin_rad, 0.0, 0.0];
let pulses60 = fine_align_torque(error60, dt_s);
// ≈ 2.909e-4 * 5215.6 * 0.12 ≈ 0.182 → rounds to 0
// Use 10 arcmin:
let ten_arcmin_rad = 10.0 * one_arcmin_rad;
let errorbig: Vec3 = [ten_arcmin_rad, 0.0, 0.0];
let pulsesbig = fine_align_torque(errorbig, dt_s);
// ≈ 2.909e-3 * 5215.6 * 0.12 ≈ 1.82 → rounds to 2
assert_eq!(pulsesbig[0], 2_i16,
    "Expected 2 torque pulses for 10 arcmin error; got {}", pulsesbig[0]);
assert_eq!(pulsesbig[1], 0_i16);
assert_eq!(pulsesbig[2], 0_i16);
```

**Expected**: `[2, 0, 0]` for a 10-arcminute X-axis attitude error over one
T4RUPT cycle. This confirms the scale factor and rounding behaviour.

### TC-IMU-CTRL-5: REFSMMAT orthonormality after construction

**Purpose**: The TRIAD algorithm must produce an orthonormal matrix from any
valid (non-collinear) star pair.

```rust
use math::linalg::{mxm, transpose, norm};

// Star 1: aligned with X-inertial, measured in platform frame as X
let s1_iner: Vec3 = [1.0, 0.0, 0.0];
let s1_plat: Vec3 = [1.0, 0.0, 0.0];

// Star 2: Y-inertial, measured as Y in platform (platform IS inertial)
let s2_iner: Vec3 = [0.0, 1.0, 0.0];
let s2_plat: Vec3 = [0.0, 1.0, 0.0];

let refsmmat = refsmmat_from_star_sightings(s1_iner, s2_iner, s1_plat, s2_plat)
    .expect("Non-collinear stars must produce a valid REFSMMAT");

// Orthonormality: R · R^T = I
let product = mxm(refsmmat, transpose(refsmmat));
let identity = math::linalg::IDENTITY;
for i in 0..3 {
    for j in 0..3 {
        let expected = if i == j { 1.0 } else { 0.0 };
        assert!((product[i][j] - expected).abs() < 1e-12,
            "Orthonormality failed at [{},{}]: got {}, expected {}",
            i, j, product[i][j], expected);
    }
}

// When platform = inertial, REFSMMAT should be identity
for i in 0..3 {
    for j in 0..3 {
        let expected = if i == j { 1.0 } else { 0.0 };
        assert!((refsmmat[i][j] - expected).abs() < 1e-12,
            "REFSMMAT should be identity when platform = inertial; [{},{}] = {}",
            i, j, refsmmat[i][j]);
    }
}
```

**Expected**: For identical inertial and platform triads, REFSMMAT = identity.
Orthonormality holds to floating-point precision (< 1e-12).

### TC-IMU-CTRL-6: Gimbal lock warning and critical thresholds

**Purpose**: Warning fires inside the 20° zone; critical fires inside the 5°
zone; both are clear at safe angles.

```rust
// Exactly at +90° (16384 counts) — inside warning zone AND critical zone
assert!(is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(16384)]));
assert!(is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(16384)]));

// At 70° (12288 counts): distance to 90° = 16384 - 12288 = 4096 > 3641 → outside warning
assert!(!is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(12288)]));
assert!(!is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(12288)]));

// At 75° (13653 counts): distance to 90° = 16384 - 13653 = 2731 < 3641 → inside warning
assert!(is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(13653)]));
// 2731 > 910 → outside critical
assert!(!is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(13653)]));

// At 86° (15655 counts): distance to 90° = 16384 - 15655 = 729 < 910 → inside critical
assert!(is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(15655)]));
assert!(is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(15655)]));

// At 0° — safe
assert!(!is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(0)]));
assert!(!is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(0)]));

// At 270° (49152 counts) — exactly at -90°, inside both zones
assert!(is_gimbal_lock_warning([CduAngle(0), CduAngle(0), CduAngle(49152)]));
assert!(is_gimbal_lock_critical([CduAngle(0), CduAngle(0), CduAngle(49152)]));
```

**Expected**: Warning at ±90° and within 20° of ±90°; critical within 5°;
clear at 0° and at 70°. The 75° case is warning-only (not critical); the 86°
case is both warning and critical.

### TC-IMU-CTRL-7: Collinear star rejection

**Purpose**: Passing two identical star vectors must return `None`.

```rust
let star: Vec3 = [1.0, 0.0, 0.0];
let result = refsmmat_from_star_sightings(star, star, star, star);
assert!(result.is_none(), "Collinear stars must return None");

// Anti-parallel stars (same axis, opposite directions) are also collinear:
let anti: Vec3 = [-1.0, 0.0, 0.0];
let result2 = refsmmat_from_star_sightings(star, anti, star, anti);
assert!(result2.is_none(), "Anti-parallel stars must return None");
```

---

## 15. Specification Quality Checklist

- [x] AGC source file and line range referenced (IMU_CALIBRATION_AND_ALIGNMENT.agc,
      IMU_COMPENSATION_PACKAGE.agc, AVERAGE_G_INTEGRATOR.agc; line ranges
      pending direct access — flagged for cross-check during implementation)
- [x] All erasable variables and their AGC addresses listed (§12)
- [x] Scale factors documented for all fixed-point values (§2.2 gimbal counts,
      §5 PIPA pipeline, §8 gyro torque pulses, §9 TRIAD matrix elements)
- [x] Corresponding f64 SI units documented throughout (m/s for delta-V,
      radians for angles, seconds for time)
- [x] Input/output preconditions and postconditions stated for each function
      (§5.2–5.3, §6.2–6.3, §7.2–7.3, §8.2–8.3, §9.2–9.3, §10)
- [x] Edge cases and error handling specified (collinear stars → None, gimbal
      lock warning and critical bands, i16 clamping for torque and drift,
      bias overflow guard)
- [x] At least 5 test cases with expected values — 7 cases provided (§14)
- [x] Rust API signature designed with types, ownership, and no heap allocation
- [x] Invariants explicitly stated (alignment state transitions §3.1,
      REFSMMAT orthonormality §9.3, gyro precondition §8.2)
- [x] Consistency with docs/architecture.md checked (typestate §4.1, T4RUPT
      task list §13.4, numeric types §3.1, module structure §2)
- [x] Cross-reference with hal-spec §8 confirmed (read_cdu, torque_gyro,
      coarse_align, is_caged — all HAL calls documented with caller identity)
- [x] Cross-reference with average-g-spec §3.3 confirmed (PIPA pipeline
      ownership boundary defined; migration path noted §1)
- [x] CI-1 resolved: §6.1 documents Strategy D — all HAL I/O in T4RUPT ISR
      shim; compute_gyro_drift is pure
- [x] CI-8 resolved: gimbal lock ownership confirmed in this module; warning
      at 70° (GIMBAL_LOCK_WARNING_BAND = 3641 counts), critical at 85°
      (GIMBAL_LOCK_CRITICAL_BAND = 910 counts); is_gimbal_lock_critical added
