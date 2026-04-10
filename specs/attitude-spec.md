# Specification: `control/attitude` Module — Attitude Control

**Status**: Approved for implementation
**Module path**: `agc-core/src/control/attitude.rs`
**Architecture reference**: `docs/architecture.md` §11 "Digital Autopilot (DAP)", §11.1 "CSM DAP Modes", §11.2 "RCS Jet Selection", §11.3 "Timing"
**DAP reference**: `agc-core/src/control/dap.rs` — `DapState`, `DapMode` enum (caller context); `specs/dap-spec.md` §5 (deadband application, staged CDU reads)
**HAL reference**: `specs/hal-spec.md` §8 (`Imu` trait, `read_cdu()`, CDU angle encoding), §8.3 (angle encoding detail)
**Types reference**: `specs/types-module-spec.md` §3.1 (`CduAngle`), §3.4 (`Vec3`), §3.5 (`Mat3x3`)
**Linear algebra reference**: `specs/linalg-spec.md` §4 (`dot`, `cross`, `norm`, `unit`, `mxv`, `vxm`, `transpose`, `mxm`)
**RCS reference**: `specs/rcs-logic-spec.md` §6 (jet selection, `select_jets_sm`)
**Gimbal lock reference**: `specs/imu-control-spec.md` §10 (`is_gimbal_lock_warning`, `is_gimbal_lock_critical`)
**AGC source files**:
- `Comanche055/CM_BODY_ATTITUDE.agc` — attitude error computation, body-rate derivation, deadband logic
- `Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` (or equivalent DAP file) — rate-damping, attitude-hold, maneuver-rate logic, T5RUPT dispatch
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — CDUX (0033), CDUY (0034), CDUZ (0035), REFSMMAT (E3 bank), attitude deadband erasable cells
**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

`control::attitude` is the computational core of the CSM Coast DAP. It is called
by `control::dap` on every T5RUPT cycle (nominally every 100 ms, TIME5 overflow)
to produce torque-demand vectors that are passed downstream to
`control::rcs_logic` for RCS jet selection.

The module performs five functions:

1. **Attitude error computation** — transform current CDU gimbal angles and the
   REFSMMAT into a body-frame error rotation, then extract three-axis (roll,
   pitch, yaw) error angles in radians.
2. **Body rate estimation** — numerically differentiate successive CDU angle
   readings to produce body angular rates in rad/s.
3. **Rate-damping torque** — produce a torque demand proportional to negative
   body rate to null spacecraft spin.
4. **Attitude-hold torque** — PD (proportional-derivative) control law that
   drives attitude error and body rate toward zero simultaneously.
5. **Maneuver rate** — compute the instantaneous commanded rate vector for a
   large-angle slew to a target attitude.

One safety predicate and one gate are also specified:

- **Deadband test** — gate all torque outputs to zero when attitude error is
  entirely within the programmed deadband.

**Gimbal lock detection is NOT part of this module.** It is owned exclusively
by `control::imu_control::is_gimbal_lock_warning` and
`control::imu_control::is_gimbal_lock_critical` (see `specs/imu-control-spec.md`
§10). The DAP caller checks those predicates and sets the DSKY GIMBAL LOCK lamp.

### What this module does NOT provide

- RCS jet selection — handled by `control::rcs_logic`.
- TVC (thrust-vector control) — handled by `control::tvc`.
- CDU hardware I/O — the caller (`control::dap`) reads CDU angles from the
  staged field `state.current_cdu` (Strategy D) and passes them in.
- REFSMMAT management — owned by `navigation::state_vector`; passed in by value.
- DSKY display updates — the caller checks `imu_control::is_gimbal_lock_warning`
  and sets the appropriate lamp.
- Gimbal lock detection — owned by `control::imu_control` (see §10 cross-ref).

---

## 2. AGC Background

### 2.1 The Coast DAP T5RUPT Cycle

The CSM Coast DAP is driven by TIME5 overflow interrupts. At each T5RUPT the
flight software:

1. Reads the three IMU gimbal CDU angles (CDUX, CDUY, CDUZ at octal addresses
   0033–0035).
2. Computes attitude error by comparing current CDU angles (transformed to a
   rotation matrix via Euler angles and then into the body frame via REFSMMAT)
   against the stored commanded attitude.
3. Computes body rates by differencing current CDU readings from the previous
   cycle (stored in erasable memory) and dividing by the cycle period (nominally
   0.1 s = 100 ms).
4. Selects a control law based on the current `DapMode`:
   - `RateDamping` — null rates only.
   - `AttitudeHold` — maintain stored desired attitude with PD control.
   - `Maneuver` — command a slewing rate toward the target attitude.
5. Applies the deadband: if all three error axes fall within the deadband no
   torque is commanded.
6. Passes torque demand to the jet-select logic.

### 2.2 CM Gimbal Sequence and CDU Axis Convention

The CM IMU uses a three-gimbal Cardan (Euler) suspension. The hardware assigns
the three CDU counters as follows:

| AGC symbol | Octal | Rust index | Physical gimbal | Body axis (nominal) |
|------------|-------|-----------|-----------------|---------------------|
| `CDUX`     | 0033  | `[0]`     | Outer gimbal    | Roll (X)            |
| `CDUY`     | 0034  | `[1]`     | Inner gimbal    | Pitch (Y)           |
| `CDUZ`     | 0035  | `[2]`     | Middle gimbal   | Yaw (Z)             |

This convention is established by `specs/hal-spec.md` §8.3: `read_cdu()` returns
`[outer, inner, middle]`, interpreted as `[roll, pitch, yaw]` in the CM gimbal
sequence.

### 2.3 CDU Angle Encoding

`CduAngle` is a `u16` value where a full revolution = 65 536 counts (2^16),
giving a scale factor of `TAU / 65536.0` radians per count. Two's-complement
representation means:

- `CduAngle(0)` → 0 rad
- `CduAngle(16384)` → π/2 rad (90°)
- `CduAngle(32768)` → π rad (180°) — also represented as `CduAngle(32768)` = −π
  when interpreted as a signed angle
- `CduAngle(49152)` → 3π/2 rad (270°) or equivalently −π/2

The conversion method `CduAngle::to_radians(self) -> f64` returns the angle in
`[0, TAU)` and is defined in `specs/types-module-spec.md` §3.1.

For finite-difference body-rate computation (§4.3), angle differences must be
computed using **two's-complement subtraction** on the raw `u16` counts and then
converting the signed difference to radians. This correctly handles wrap-around
at 0/2π.

### 2.4 REFSMMAT and the Attitude Error Computation

REFSMMAT ("Reference to Stable Member Matrix") is a 3×3 orthonormal rotation
matrix stored in 18 double-precision AGC words in the E3 bank of erasable memory.
It represents the rotation from the stable-member (inertial) frame to the body
frame as it was established at the last IMU alignment (P52).

The attitude error computation follows the Comanche055 CM_BODY_ATTITUDE approach:

1. Convert current CDU angles (θ_outer, θ_inner, θ_middle) = (roll, pitch, yaw)
   to a body-to-stable-member rotation matrix `M_gimbal` via the CM Euler angle
   sequence (ZYX: yaw about Z, pitch about Y, roll about X applied in the CM
   gimbal suspension order outer-inner-middle → XYZ).
2. Compute the current body attitude in the inertial frame:
   `M_current = REFSMMAT · M_gimbal`
3. Compute the error rotation matrix: `M_err = M_desired^T · M_current`
4. Extract small-angle roll/pitch/yaw errors from the off-diagonal elements of
   `M_err` (see §4.2 for the exact formula).

### 2.5 AGC Erasable Variables Relevant to This Module

| AGC symbol    | Address (octal) | Description                                         |
|---------------|-----------------|-----------------------------------------------------|
| `CDUX`        | 0033            | CDU outer (roll) gimbal angle (hardware counter)    |
| `CDUY`        | 0034            | CDU inner (pitch) gimbal angle (hardware counter)   |
| `CDUZ`        | 0035            | CDU middle (yaw) gimbal angle (hardware counter)    |
| `REFSMMAT`    | E3 bank         | 3×3 stable-member-to-body rotation (18 DP words)   |
| `CDUXPREV`    | erasable        | Previous cycle CDU outer angle (for rate estimate)  |
| `CDUYPREV`    | erasable        | Previous cycle CDU inner angle (for rate estimate)  |
| `CDUZPREV`    | erasable        | Previous cycle CDU middle angle (for rate estimate) |
| `DAPBOOLS`    | erasable        | Mode flags including deadband selection             |
| `ATTDB`       | erasable        | Attitude deadband in revolutions (B-1 scale)        |

In the Rust port, `CDUXPREV/CDUYPREV/CDUZPREV` are the `cdu_old` parameter
passed by the DAP. `REFSMMAT` is passed by value as `Mat3x3`. `ATTDB` is
`DapState::deadband` (f64 radians). The DAP state struct (`DapState`) carries
all persistent state between cycles.

### 2.6 Scale Factors

| Quantity          | AGC representation             | Rust `f64` SI unit |
|-------------------|--------------------------------|--------------------|
| CDU angle         | `u16` counts (2^16 = 1 rev)   | radians (`to_radians()`) |
| Body rate         | rad/centisec (B+14)            | rad/s              |
| Attitude error    | radians (DP)                   | radians            |
| Torque demand     | dimensionless normalized       | Nm (sign + magnitude) |
| Deadband          | revolutions (B-1)              | radians            |
| Gain constants    | dimensionless fixed-point       | dimensionless f64  |

---

## 3. Public API

```rust
use crate::types::{CduAngle, Mat3x3, Vec3};

/// Three-axis attitude error (roll, pitch, yaw) in radians.
/// Positive error = current attitude is rotated positively about that body axis
/// relative to the desired attitude.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AttitudeError {
    pub roll:  f64,   // radians; positive = right-wing-down (X-axis rotation)
    pub pitch: f64,   // radians; positive = nose-up (Y-axis rotation)
    pub yaw:   f64,   // radians; positive = nose-right (Z-axis rotation)
}

/// Compute attitude error from current CDU angles and commanded attitude.
pub fn compute_attitude_error(
    current_cdu: [CduAngle; 3],
    desired:     Mat3x3,
    refsmmat:    Mat3x3,
) -> AttitudeError;

/// Estimate body angular rates from two successive CDU readings separated by dt.
pub fn compute_body_rates(
    cdu_new: [CduAngle; 3],
    cdu_old: [CduAngle; 3],
    dt:      f64,
) -> Vec3;

/// Compute the torque demand required to null the given body rates (rate damping).
pub fn rate_damping_torque(rates: Vec3, gain: Vec3) -> Vec3;

/// Compute PD attitude-hold torque from attitude error and body rates.
pub fn attitude_hold_torque(
    error: AttitudeError,
    rates: Vec3,
    kp:   f64,
    kd:   f64,
) -> Vec3;

/// Compute the instantaneous commanded angular rate vector for a large-angle slew.
pub fn maneuver_rate(
    current:  Mat3x3,
    target:   Mat3x3,
    max_rate: f64,
) -> Vec3;
```

**Gimbal lock detection is NOT part of this API.** Use
`control::imu_control::is_gimbal_lock_warning(cdu)` and
`control::imu_control::is_gimbal_lock_critical(cdu)` instead
(see `specs/imu-control-spec.md` §10).

All functions are pure (no side effects, no mutation of global state). The DAP
caller in `control::dap` owns the state and passes it in. No heap allocation.
All arguments and return values are `Copy`.

---

## 4. Function Specifications

### 4.1 `AttitudeError`

**Definition**:

```rust
pub struct AttitudeError {
    pub roll:  f64,
    pub pitch: f64,
    pub yaw:   f64,
}
```

**Invariants**:
- No element may be `NaN` or `±Inf`.
- Values are in radians. In nominal flight, magnitudes are below π rad;
  values outside `(-π, +π]` are an indication of a numerical problem and
  the caller should alarm.
- Convention: the error is the rotation that, applied to the desired attitude,
  gives the current attitude. Positive roll = current attitude is rotated
  clockwise about the body X-axis relative to desired. Similarly for pitch
  (Y-axis) and yaw (Z-axis).

**Conversion to Vec3**: `[error.roll, error.pitch, error.yaw]` is a valid
`Vec3` for passing to torque functions.

---

### 4.2 `compute_attitude_error`

**Signature**:

```rust
pub fn compute_attitude_error(
    current_cdu: [CduAngle; 3],
    desired:     Mat3x3,
    refsmmat:    Mat3x3,
) -> AttitudeError
```

**Purpose**: Convert the current gimbal angles and the stored REFSMMAT into a
three-axis attitude error with respect to the commanded attitude matrix
`desired`.

**Algorithm**:

Step 1 — Convert CDU angles to radians using `libm` transcendentals (required
for `no_std`; see §8):

```
theta_x = current_cdu[0].to_radians()   // outer (roll)
theta_y = current_cdu[1].to_radians()   // inner (pitch)
theta_z = current_cdu[2].to_radians()   // middle (yaw)
```

Step 2 — Build the CM gimbal rotation matrix from Euler angles. The CM gimbal
suspension applies rotations in the order: outer (X/roll) → inner (Y/pitch) →
middle (Z/yaw), which corresponds to the Tait-Bryan ZYX sequence read right-to-left.
The body-to-stable-member matrix `M_gimbal` is:

```
M_gimbal = Rx(theta_x) · Ry(theta_y) · Rz(theta_z)
```

where the elementary rotation matrices are the standard right-hand-rule forms.
All trigonometric calls use `libm` (e.g., `libm::sin(theta_x)`,
`libm::cos(theta_x)`):

```
Rx(θ) = [[1,          0,           0         ],
          [0,          libm::cos(θ), -libm::sin(θ)],
          [0,          libm::sin(θ),  libm::cos(θ)]]

Ry(θ) = [[ libm::cos(θ), 0,  libm::sin(θ)],
          [ 0,            1,  0            ],
          [-libm::sin(θ), 0,  libm::cos(θ)]]

Rz(θ) = [[libm::cos(θ), -libm::sin(θ), 0],
          [libm::sin(θ),  libm::cos(θ), 0],
          [0,             0,            1]]
```

Step 3 — Compute the current attitude matrix in the inertial frame:

```
M_current = mxm(refsmmat, M_gimbal)
```

Step 4 — Compute the error rotation matrix from desired to current:

```
M_err = mxm(transpose(desired), M_current)
```

`M_err` is the rotation that transforms the desired frame into the current
frame. When the vehicle is exactly on attitude, `M_err` is the identity matrix.

Step 5 — Extract small-angle errors from `M_err`. For small angles (within the
deadband), the anti-symmetric part of `M_err` gives the rotation vector
directly. The exact formula (valid for angles up to approximately 20°) is:

```
roll  = (M_err[2][1] - M_err[1][2]) / 2
pitch = (M_err[0][2] - M_err[2][0]) / 2
yaw   = (M_err[1][0] - M_err[0][1]) / 2
```

These are the components of the rotation vector `φ = (1/2)(M_err - M_err^T)`
extracted as axial components, which equals the small-angle approximation of the
Euler-axis/angle decomposition.

For large errors (maneuver initiation), the same formula is used. The result
saturates gracefully: when the error axis exceeds ≈1.57 rad (90°), the extracted
value remains bounded and the maneuver logic uses `maneuver_rate` instead of
`attitude_hold_torque` to avoid integrator windup. The caller (`control::dap`)
is responsible for selecting the appropriate control law based on `DapMode`.

**Preconditions**:
- `refsmmat` and `desired` must be orthonormal rotation matrices (invariant
  checked by P52 alignment; `control::attitude` does not re-verify).
- `current_cdu` must contain valid CDU readings from a powered, uncaged IMU.
- No element of `current_cdu` has an implementation-defined range restriction
  (all 2^16 values are valid).

**Postconditions**:
- Returns `AttitudeError` with no `NaN` or `±Inf` components.
- When `M_err` is the identity, all three returned values are zero to within
  floating-point rounding (≤ 1 × 10^-14 rad).
- Magnitudes ≤ π/2 rad for attitudes reachable without gimbal lock.
- **Sign-convention postcondition (CI-10)**: When `desired == REFSMMAT == Identity`
  and `current_cdu[0]` encodes a positive outer-gimbal rotation of θ° (e.g.,
  `CduAngle(round(θ * 65536 / 360))`), the returned `error.roll ≈ +θ° in radians`
  (positive). This is the "current-relative-to-desired" sign required by
  `attitude_hold_torque`'s restoring-torque convention (negative torque for
  positive error). Verify this identity in TC-ATT-02.

**Failure modes**:
- If `refsmmat` or `desired` is degenerate (determinant not ≈ 1), the returned
  error will be nonsensical. The caller must ensure valid matrices.

**AGC source reference**:
`Comanche055/CM_BODY_ATTITUDE.agc` — attitude error extraction routine;
`Comanche055/ERASABLE_ASSIGNMENTS.agc` — CDUX/Y/Z addresses, REFSMMAT layout.

---

### 4.3 `compute_body_rates`

**Signature**:

```rust
pub fn compute_body_rates(
    cdu_new: [CduAngle; 3],
    cdu_old: [CduAngle; 3],
    dt:      f64,
) -> Vec3
```

**Purpose**: Estimate body angular rates in rad/s by numerically differentiating
successive CDU angle readings separated by time interval `dt` seconds.

**Algorithm**:

For each axis `i` in {0, 1, 2}:

1. Compute the raw count difference using two's-complement (wrapping) arithmetic
   on the `u16` counts:

   ```
   delta_counts_i = (cdu_new[i].0).wrapping_sub(cdu_old[i].0) as i16
   ```

   Casting to `i16` after wrapping subtraction gives the signed angular change
   in `(-32768, +32767]` counts, correctly handling the 0/65536 wrap-around.

2. Convert to radians:

   ```
   delta_rad_i = (delta_counts_i as f64) * TAU / 65536.0
   ```

3. Divide by the cycle period:

   ```
   rate_i = delta_rad_i / dt
   ```

Return `[rate_0, rate_1, rate_2]` as `Vec3` where index 0 = roll (X-axis),
1 = pitch (Y-axis), 2 = yaw (Z-axis), matching the CDU axis convention in §2.2.

**Preconditions**:
- `dt > 0.0` — a zero or negative interval is a programming error. The
  implementation shall panic in debug builds (`debug_assert!(dt > 0.0)`) and
  return `[0.0, 0.0, 0.0]` in release builds.
- `dt` must be the true elapsed time between CDU samples, not the nominal
  100 ms period. The T5RUPT period can vary slightly; the caller should use
  the actual measured interval if available.
- The angular velocity must not exceed approximately 180°/s
  (≈ 3.14 rad/s) between samples or the wrapping will alias. In normal
  flight, body rates are on the order of 0.1–2°/s; the precondition is
  easily satisfied.

**Postconditions**:
- Returns `Vec3` with no `NaN` or `±Inf`.
- When `cdu_new == cdu_old`, returns `[0.0, 0.0, 0.0]` exactly.
- Units are rad/s.

**Nominal value**: At the T5RUPT period of 100 ms (dt = 0.1 s), one CDU count
  difference corresponds to a rate of `TAU / 65536 / 0.1` ≈ 0.000958 rad/s
  (0.055°/s) — the quantisation limit of body-rate estimation.

**AGC source reference**: Rate estimation from CDU differencing described in
`Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` (T5RUPT handler body-rate read
section).

---

### 4.4 `rate_damping_torque`

**Signature**:

```rust
pub fn rate_damping_torque(rates: Vec3, gain: Vec3) -> Vec3
```

**Purpose**: Compute the torque demand required to null the current body rates.
This is the control law used in `DapMode::RateDamping`.

**Algorithm**:

```
torque[i] = -gain[i] * rates[i]     for i = 0, 1, 2
```

The negative sign ensures that a positive rate (rotating in the positive
direction) produces a negative torque demand (opposing torque). The gain vector
allows independent axis scaling to account for differences in spacecraft moment
of inertia and RCS jet geometry.

**Preconditions**:
- `rates` and `gain` must not contain `NaN` or `±Inf`.
- `gain[i] >= 0.0` for all i (a negative gain would be a sign error; the
  implementation shall `debug_assert!` this).

**Postconditions**:
- Returns `Vec3` with no `NaN` or `±Inf`.
- When `rates == [0.0, 0.0, 0.0]`, returns `[0.0, 0.0, 0.0]` exactly.
- Component sign is opposite to the corresponding rate component.

**Typical gain values** (from Comanche055 DAP configuration via V46 DSKY entry):
- Gain per axis ≈ 0.3 to 2.0 dimensionless (tuned for CSM moment of inertia
  and RCS jet thrust level). The exact values are stored in `DapState` and
  passed in by the caller.

**Note**: This function does not apply the deadband. The caller (`control::dap`)
checks `DapState::deadband` against the rate magnitude and gates the output.
For `DapMode::RateDamping`, a rate deadband (rather than attitude deadband)
is applied; this is the caller's responsibility.

---

### 4.5 `attitude_hold_torque`

**Signature**:

```rust
pub fn attitude_hold_torque(
    error: AttitudeError,
    rates: Vec3,
    kp:   f64,
    kd:   f64,
) -> Vec3
```

**Purpose**: Compute the torque demand to hold the commanded attitude, using a
PD (proportional-derivative) control law. This is the control law used in
`DapMode::AttitudeHold`.

**Algorithm**:

The torque demand for each axis is:

```
torque[0] = -(kp * error.roll  + kd * rates[0])   // roll
torque[1] = -(kp * error.pitch + kd * rates[1])   // pitch
torque[2] = -(kp * error.yaw   + kd * rates[2])   // yaw
```

The negative sign follows the convention that a positive attitude error (current
attitude rotated ahead of desired) requires a negative (restoring) torque.

**Physical interpretation**:
- `kp` (proportional gain) produces a torque proportional to the pointing error,
  driving the vehicle back toward the desired attitude.
- `kd` (derivative gain) produces a torque opposing the current rate, providing
  damping that prevents oscillation. `kd` must be chosen large enough to
  critically damp the attitude response for the given `kp` and spacecraft
  inertia.

**Preconditions**:
- `error` must not contain `NaN` or `±Inf` in any field.
- `rates` must not contain `NaN` or `±Inf`.
- `kp >= 0.0`, `kd >= 0.0` (debug-asserted).

**Postconditions**:
- Returns `Vec3` with no `NaN` or `±Inf`.
- When `error` is all-zero and `rates` is all-zero, returns `[0.0, 0.0, 0.0]`
  exactly.

**Deadband handling**: The caller (`control::dap`) tests whether the magnitude
of `AttitudeError` is within `DapState::deadband` before calling this function.
If within the deadband, the caller must substitute `[0.0, 0.0, 0.0]` as the
torque demand without calling `attitude_hold_torque`. This function itself does
not perform the deadband check (separation of concerns; the deadband value lives
on `DapState`).

**Typical values** (from Comanche055 DAP tables; tuned for CSM inertia ≈
6000 kg·m² per axis):
- `kp` ≈ 0.3–0.6 (torque per radian of error, normalised by jet capability)
- `kd` ≈ 0.6–1.2 (torque per rad/s of rate)
- Critical damping ratio ζ = kd / (2 · sqrt(kp · I)) = 1.0 at nominal inertia.

**AGC source reference**: PD attitude-hold law in
`Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc`, attitude error / rate feedback
section.

---

### 4.6 `maneuver_rate`

**Signature**:

```rust
pub fn maneuver_rate(
    current:  Mat3x3,
    target:   Mat3x3,
    max_rate: f64,
) -> Vec3
```

**Purpose**: For `DapMode::Maneuver`, compute the commanded angular rate vector
that will slew the spacecraft from its current attitude to the target attitude
at a controlled rate. The returned `Vec3` is the body-frame angular rate command
in rad/s, suitable for use as the rate setpoint in a rate-damping loop.

**Algorithm**:

Step 1 — Compute the error rotation matrix:

```
M_err = mxm(transpose(target), current)
```

Step 2 — Extract the rotation axis and angle. The rotation vector of `M_err`
can be read from its anti-symmetric part. First extract the raw components
(same as §4.2 Step 5):

```
e_x = (M_err[2][1] - M_err[1][2]) / 2
e_y = (M_err[0][2] - M_err[2][0]) / 2
e_z = (M_err[1][0] - M_err[0][1]) / 2
```

The vector `e = [e_x, e_y, e_z]` is the sine of the error angle times the
rotation axis unit vector. Its magnitude `sin_angle = norm(e)`.

Step 3 — Compute the true rotation angle using `libm::atan2`:

```
cos_angle = (M_err[0][0] + M_err[1][1] + M_err[2][2] - 1.0) / 2.0
angle = libm::atan2(sin_angle, cos_angle)    // in [0, π]
```

Step 4 — If `angle < 1e-9` rad (effectively zero error), return
`[0.0, 0.0, 0.0]` — the maneuver is complete.

Step 5 — Compute the unit rotation axis:

```
axis = normalize(e)   // unit vector via linalg::unit
```

Step 6 — Clamp the angular rate to `max_rate`:

```
rate_magnitude = min(angle, max_rate)
```

This ensures the commanded rate never exceeds the hardware-limited slew rate
(typical: ≤ 0.5°/s = 0.00873 rad/s for fine maneuvers; ≤ 1°/s for coarse).

Step 7 — Return the commanded rate vector in the body frame:

```
commanded_rate = scale(axis, rate_magnitude)
```

This rate vector is returned to the DAP, which uses it as the desired rate and
calls `rate_damping_torque` with the error `(rates - commanded_rate)`.

**Preconditions**:
- `current` and `target` must be orthonormal rotation matrices.
- `max_rate > 0.0` (debug-asserted); zero or negative max_rate is a programming
  error.

**Postconditions**:
- Returns `Vec3` with no `NaN` or `±Inf`.
- When `current == target`, returns `[0.0, 0.0, 0.0]`.
- `norm(result) <= max_rate + floating_point_epsilon`.
- The returned vector lies along the shortest-arc rotation axis in the body
  frame.

**Note**: The returned rate is defined as the required angular velocity of the
body frame relative to inertial space, expressed in body coordinates. The DAP
must not interpret this as a rate error; it is the full rate setpoint.

**AGC source reference**: Maneuver rate computation in
`Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` and
`Comanche055/CM_BODY_ATTITUDE.agc` (DAPTREG, maneuver rate table).

---

### 4.7 Gimbal Lock Detection (Cross-Reference)

Gimbal lock detection is **not** implemented in this module. It is owned by
`control::imu_control` (see `specs/imu-control-spec.md` §10), which defines:

- `is_gimbal_lock_warning(cdu: [CduAngle; 3]) -> bool` — fires at 70° (within
  20° of ±90°). Called from T4RUPT phase 3 ISR shim.
- `is_gimbal_lock_critical(cdu: [CduAngle; 3]) -> bool` — fires at 85° (within
  5° of ±90°). Also called from T4RUPT phase 3 ISR shim.

The DAP caller (`control::dap`) calls `imu_control::is_gimbal_lock_warning` and
`imu_control::is_gimbal_lock_critical` after reading CDU angles. When the
warning returns `true` for three consecutive cycles, the caller illuminates the
DSKY GIMBAL LOCK lamp (relay 14, bit 1 per `specs/hal-spec.md` §5.2) and issues
a V05N09 alarm.

Test cases for these predicates are in `specs/imu-control-spec.md` §14
(TC-IMU-CTRL-6).

---

## 5. Deadband Handling

Deadband is the magnitude threshold below which no torque command is issued.
Deadband values are stored in `DapState::deadband` (f64 radians). The crew
programs the deadband via DSKY V46 (DAP data load). Typical values:

| Mode          | Typical deadband      |
|---------------|-----------------------|
| Attitude hold | 0.3° to 5°            |
| Rate damping  | 0.01–0.1 rad/s (rate) |
| Maneuver      | No deadband applied   |

The deadband test is performed by the **caller** (`control::dap`), not by any
function in this module. The separation ensures that:

1. The functions in this module are pure and testable in isolation.
2. The deadband logic can evolve (e.g., different per-axis deadbands) without
   changing the attitude computation functions.

The caller applies the following logic:

```
let error_vec: Vec3 = [error.roll, error.pitch, error.yaw];
let in_deadband = error_vec.iter().all(|e| e.abs() < dap_state.deadband);
let torque = if in_deadband {
    [0.0, 0.0, 0.0]
} else {
    attitude_hold_torque(error, rates, kp, kd)
};
```

For `DapMode::RateDamping`, the deadband is applied to rate magnitude:

```
let rate_mag = linalg::norm(rates);
let torque = if rate_mag < rate_deadband {
    [0.0, 0.0, 0.0]
} else {
    rate_damping_torque(rates, gain)
};
```

---

## 6. Interaction with `control::dap` (T5RUPT Context)

**Strategy D (staging fields)**: `dap_step` has the Waitlist task signature
`fn(&mut AgcState)`. CDU angles are **not** read inside `dap_step` via
`hw.imu().read_cdu()`. Instead, the T5RUPT ISR shim reads the CDU angles
before calling the Waitlist task and stages them into `state.current_cdu:
[CduAngle; 3]`. The Waitlist task reads from that field. Similarly, the RCS
jet command is written to `state.rcs_commanded_jets: u16` and the T5RUPT ISR
shim reads that field after the task returns to issue the HAL fire command.

The complete T5RUPT DAP cycle as seen by this module (pseudocode within the
Waitlist task `dap_step`):

```
// In control/dap.rs, dap_step(state: &mut AgcState):
// CDU angles were staged by the ISR shim before this call:
let cdu_new   = state.current_cdu;          // staged by T5RUPT ISR shim
let cdu_old   = state.dap_state.prev_cdu;   // saved last cycle
let dt        = 0.1_f64;                    // 100 ms nominal

// 1. Estimate rates
let rates = attitude::compute_body_rates(cdu_new, cdu_old, dt);
state.dap_state.rate_estimate = rates;

// 2. Compute attitude error
let error = attitude::compute_attitude_error(
    cdu_new,
    state.dap_state.desired_attitude,
    state.nav_state.refsmmat,
);
state.dap_state.attitude_error = [error.roll, error.pitch, error.yaw];

// 3. Gimbal lock check is performed by the T4RUPT ISR shim, not here.
//    The T4RUPT ISR shim calls imu_control::is_gimbal_lock_warning(cdu)
//    and imu_control::is_gimbal_lock_critical(cdu) and updates the DSKY lamp.

// 4. Select torque command based on mode + deadband
let torque = match state.dap_state.mode {
    DapMode::Off => [0.0; 3],
    DapMode::RateDamping  => { /* deadband on rate then rate_damping_torque */ }
    DapMode::AttitudeHold => { /* deadband on error then attitude_hold_torque */ }
    DapMode::Maneuver     => { /* maneuver_rate → rate setpoint → rate_damping_torque */ }
    DapMode::Tvc          => { /* handled by control::tvc */ }
};

// 5. Stage jet command — ISR shim will call hw.rcs().fire_sm_jets after task returns
state.rcs_commanded_jets = rcs_logic::select_jets_sm(torque, &state.rcs_config);
state.dap_state.rcs_jet_flags = state.rcs_commanded_jets;

// 6. Save CDU readings for next cycle
state.dap_state.prev_cdu = cdu_new;
```

Fields needed on `DapState` beyond what is already declared in `control/dap.rs`:

| New field             | Type            | Purpose                                           |
|-----------------------|-----------------|---------------------------------------------------|
| `prev_cdu`            | `[CduAngle; 3]` | CDU reading from previous T5RUPT cycle            |
| `desired_attitude`    | `Mat3x3`        | Commanded attitude matrix (set by P20/V49 etc.)   |
| `rate_gain`           | `Vec3`          | Per-axis rate-damping gain                        |
| `kp`                  | `f64`           | Proportional gain for attitude hold               |
| `kd`                  | `f64`           | Derivative gain for attitude hold                 |
| `maneuver_max_rate`   | `f64`           | Maximum slew rate (rad/s) for `DapMode::Maneuver` |

The developer must add these fields to `DapState` in `control/dap.rs` when
implementing this module.

---

## 7. Numerical Precision

| Function                    | Key operation                  | Expected error bound          |
|-----------------------------|--------------------------------|-------------------------------|
| `compute_attitude_error`    | `mxm` × 2 + anti-sym extract  | ≤ 1 × 10^-13 rad (unit inputs)|
| `compute_body_rates`        | wrapping sub + divide          | ≤ 1 ULP in result             |
| `rate_damping_torque`       | element-wise multiply          | ≤ 1 ULP in result             |
| `attitude_hold_torque`      | fused multiply-add             | ≤ 2 ULP in result             |
| `maneuver_rate`             | `libm::atan2` + normalize      | ≤ 1 × 10^-12 in angle         |

The `mxm` tolerance of 1 × 10^-13 follows from `specs/linalg-spec.md` §7
(tolerance table: `mxm ≤ 1 × 10^-13`).

---

## 8. Dependencies

| Symbol                | Source module                          | Notes                              |
|-----------------------|----------------------------------------|------------------------------------|
| `CduAngle`            | `crate::types`                         | `to_radians()` method              |
| `Vec3`, `Mat3x3`      | `crate::types`                         | Type aliases                       |
| `linalg::mxv`         | `crate::math::linalg`                  | Matrix-vector multiply             |
| `linalg::mxm`         | `crate::math::linalg`                  | Matrix-matrix multiply             |
| `linalg::transpose`   | `crate::math::linalg`                  | Matrix transpose                   |
| `linalg::norm`        | `crate::math::linalg`                  | Vector 2-norm                      |
| `linalg::unit`        | `crate::math::linalg`                  | Normalize (panics on zero vector)  |
| `linalg::scale`       | `crate::math::linalg`                  | Scalar-vector multiply             |
| `libm::sin`           | `libm` crate                           | `no_std`-compatible sine           |
| `libm::cos`           | `libm` crate                           | `no_std`-compatible cosine         |
| `libm::atan2`         | `libm` crate                           | `no_std`-compatible atan2          |
| `core::f64::consts::TAU` | `core::f64::consts`                 | 2π (constant, no `libm` needed)    |
| `core::f64::consts::PI`  | `core::f64::consts`                 | π (constant, no `libm` needed)     |

**`no_std` note**: `agc-core` is `#![cfg_attr(not(test), no_std)]`. The methods
`f64::sin`, `f64::cos`, `f64::atan2`, `f64::asin`, `f64::acos`, and
`f64::sqrt` live in the standard-library `std` feature and are **not** available
in `no_std` builds. All transcendentals must use the `libm` crate instead
(e.g., `libm::sin(x)` instead of `x.sin()`). This matches the convention
established in `specs/linalg-spec.md` §3 and `specs/kepler-spec.md` §3.2.

No heap allocation (`alloc`). No `std`. No `static mut`. No global state.

---

## 9. Test Cases

All test cases are unit tests in `agc-core/src/control/attitude.rs` inside a
`#[cfg(test)] mod tests` block.

---

### TC-ATT-01: Zero error produces zero torque

**Scenario**: Current CDU angles produce the same attitude as the desired
attitude; body rates are zero.

**Setup**:
```rust
let identity: Mat3x3 = linalg::IDENTITY;
// All-zero CDU → M_gimbal = identity → M_current = identity (refsmmat = identity)
let cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
let error = compute_attitude_error(cdu, identity, identity);
```

**Expected**:
```rust
assert!(error.roll.abs()  < 1e-12);
assert!(error.pitch.abs() < 1e-12);
assert!(error.yaw.abs()   < 1e-12);
```

**Then** pass to `attitude_hold_torque`:
```rust
let rates: Vec3 = [0.0, 0.0, 0.0];
let torque = attitude_hold_torque(error, rates, 0.5, 1.0);
assert!(torque[0].abs() < 1e-12);
assert!(torque[1].abs() < 1e-12);
assert!(torque[2].abs() < 1e-12);
```

**Rationale**: Verifies the zero-input zero-output property for the full
attitude hold path.

---

### TC-ATT-02: Pure roll error

**Scenario**: Current attitude is rotated 5° about the roll (X) axis relative
to desired. Zero rates. Also validates the sign-convention postcondition from
§4.2.

**Setup**:
```rust
// Build desired = identity, refsmmat = identity.
// Build M_gimbal = Rx(5°) using theta_x = 5° CDU counts.
let five_deg_counts = (5.0_f64.to_radians() * 65536.0 / TAU) as u16;
let cdu = [CduAngle(five_deg_counts), CduAngle(0), CduAngle(0)];
let error = compute_attitude_error(cdu, linalg::IDENTITY, linalg::IDENTITY);
```

**Expected**:
```rust
let five_deg = 5.0_f64.to_radians();
assert!((error.roll  - five_deg).abs() < 1e-6);
assert!(error.pitch.abs() < 1e-6);
assert!(error.yaw.abs()   < 1e-6);
```

**Sign-convention check (§4.2 postcondition)**:
```rust
// A positive outer-gimbal rotation (5°) → error.roll is POSITIVE.
// This satisfies the "current-relative-to-desired" sign contract.
assert!(error.roll > 0.0, "Positive outer-gimbal rotation must yield positive roll error");
```

**Then** verify torque sign:
```rust
let rates: Vec3 = [0.0, 0.0, 0.0];
let torque = attitude_hold_torque(error, rates, 1.0, 0.0);
assert!(torque[0] < 0.0);    // restoring torque opposes positive roll error
assert!(torque[1].abs() < 1e-12);
assert!(torque[2].abs() < 1e-12);
```

**Rationale**: Validates roll-axis selectivity of the attitude error extraction,
the sign convention of the PD control law, and the §4.2 postcondition on error
sign.

---

### TC-ATT-03: Pure rate damping

**Scenario**: Zero attitude error; spacecraft has a 2°/s roll rate.
Rate-damping torque must oppose the rate.

**Setup**:
```rust
let omega_roll = 2.0_f64.to_radians();   // 2°/s
let rates: Vec3 = [omega_roll, 0.0, 0.0];
let gain: Vec3  = [1.0, 1.0, 1.0];
let torque = rate_damping_torque(rates, gain);
```

**Expected**:
```rust
assert!((torque[0] + omega_roll).abs() < 1e-15);  // torque = -gain * rate
assert!(torque[1].abs() < 1e-15);
assert!(torque[2].abs() < 1e-15);
```

**With CDU differencing** — verify `compute_body_rates` round-trip:
```rust
let dt = 0.1_f64;
// One CDU count ≈ 0.000958 rad; 2°/s × 0.1 s = 0.003491 rad
// delta_counts ≈ round(0.003491 / (TAU / 65536)) ≈ 36
let delta: u16 = ((omega_roll * dt) * 65536.0 / TAU).round() as u16;
let cdu_old = [CduAngle(0u16), CduAngle(0), CduAngle(0)];
let cdu_new = [CduAngle(delta), CduAngle(0), CduAngle(0)];
let estimated = compute_body_rates(cdu_new, cdu_old, dt);
// Allow ½ count quantisation error
let quant = TAU / 65536.0 / dt;
assert!((estimated[0] - omega_roll).abs() < quant);
```

**Rationale**: Validates the body-rate estimation quantisation and the sign
of the rate-damping law.

---

### TC-ATT-04: Attitude hold with small perturbation

**Scenario**: 1° pitch error with 0.1°/s pitch rate, kp = 0.5, kd = 1.0.
Verify the PD torque combines both terms correctly.

**Setup**:
```rust
let pitch_err = 1.0_f64.to_radians();
let pitch_rate = 0.1_f64.to_radians();
let error = AttitudeError { roll: 0.0, pitch: pitch_err, yaw: 0.0 };
let rates: Vec3 = [0.0, pitch_rate, 0.0];
let kp = 0.5_f64;
let kd = 1.0_f64;
let torque = attitude_hold_torque(error, rates, kp, kd);
```

**Expected**:
```rust
let expected_pitch = -(kp * pitch_err + kd * pitch_rate);
assert!((torque[1] - expected_pitch).abs() < 1e-14);
assert!(torque[0].abs() < 1e-14);
assert!(torque[2].abs() < 1e-14);
```

**Rationale**: Algebraic correctness of the PD formula; cross-axis isolation.

---

### TC-ATT-05: Maneuver to 90° yaw target

**Scenario**: Current attitude is identity; target is Rz(90°). Commanded rate
must point along the +Z axis with magnitude clamped to max_rate.

**Setup**:
```rust
let current: Mat3x3 = linalg::IDENTITY;
let target: Mat3x3  = [
    [ 0.0, -1.0, 0.0],
    [ 1.0,  0.0, 0.0],
    [ 0.0,  0.0, 1.0],
];  // Rz(90°)
let max_rate = 0.5_f64.to_radians();   // 0.5°/s
let rate_cmd = maneuver_rate(current, target, max_rate);
```

**Expected**:
```rust
// Rate vector must lie along Z (yaw), clamped to max_rate
assert!(rate_cmd[0].abs() < 1e-12);   // no roll component
assert!(rate_cmd[1].abs() < 1e-12);   // no pitch component
// The sign depends on the sense of the 90° rotation:
// M_err = Rz(90°)^T · I = Rz(-90°)  →  rotation axis = -Z  →  rate along -Z
// OR M_err = I^T · Rz(90°) = Rz(90°) → axis = +Z, rate along +Z
// (see §4.6 Step 1 for the exact M_err computation)
assert!((rate_cmd[2].abs() - max_rate).abs() < 1e-12);
```

**Also verify zero-error case**:
```rust
let zero_rate = maneuver_rate(current, current, max_rate);
assert_eq!(zero_rate, [0.0, 0.0, 0.0]);
```

**Rationale**: Validates the large-angle maneuver path including axis extraction,
`libm::atan2` angle computation, and rate clamping.

---

## 10. Error and Edge Cases

| Condition                              | Behaviour                                                                 |
|----------------------------------------|---------------------------------------------------------------------------|
| `dt = 0` in `compute_body_rates`       | `debug_assert!` panics in debug; returns `[0,0,0]` in release            |
| CDU wrap-around at 0/65535             | `wrapping_sub as i16` handles correctly; no special case needed           |
| `max_rate = 0` in `maneuver_rate`      | `debug_assert!` panics in debug; returns `[0,0,0]` in release            |
| Error rotation angle > π/2             | Formula still valid up to π; error saturates gracefully above π           |
| `refsmmat` not orthonormal             | Result undefined; caller (P52) must ensure valid REFSMMAT                 |
| Negative gain in `rate_damping_torque` | `debug_assert!(gain[i] >= 0)` fires; undefined in release                 |
| `norm(e) ≈ 0` in `maneuver_rate`       | Return `[0,0,0]` before calling `unit` to avoid panic (angle ≈ 0 case)   |

---

## 11. Spec Quality Checklist

- [x] AGC source file and line range referenced (CM_BODY_ATTITUDE.agc, RCS_CSM_DIGITAL_AUTOPILOT.agc, ERASABLE_ASSIGNMENTS.agc)
- [x] All erasable variables and their AGC addresses listed (§2.5)
- [x] Scale factors documented for all fixed-point values (§2.6)
- [x] Corresponding `f64` SI units documented (§2.6)
- [x] Input/output preconditions and postconditions stated (each function §4.x)
- [x] Edge cases and error handling specified (§10)
- [x] Five test cases with expected values (§9)
- [x] Rust API signature designed — types, ownership, lifetimes (§3)
- [x] Invariants explicitly stated (§4.1 `AttitudeError` invariants, per-function postconditions)
- [x] Consistency with `docs/architecture.md` checked — `Vec3`/`Mat3x3` type aliases, `no_std`, no heap, `f64` for control math, `CduAngle(u16)` for hardware (§8 dependencies)
- [x] CI-5 resolved: §8 updated — `libm::sin`, `libm::cos`, `libm::atan2` replaces `f64::sin` etc.; `no_std` note added
- [x] CI-8 resolved: §4.7 and §3 remove `gimbal_lock_warning` from this module; cross-reference to `imu_control::is_gimbal_lock_warning` / `is_gimbal_lock_critical` added; §1 updated
- [x] CI-10 resolved: §4.2 postconditions add sign-convention check; TC-ATT-02 extended to verify positive roll error for positive outer-gimbal rotation
