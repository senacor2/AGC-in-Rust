# Spec: Attitude Error and Phase-Plane Logic

## AGC Source References

| File | Routine | Pages |
|---|---|---|
| `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc` | `RCSATT`, `RATEFILT`, `DRHOLOOP`, `ADOTLOOP`, `FREECHK`, `SETWBODY`, `MERUPDAT`, `FDAIDSP1/2`, `GETAKS`, `NEEDLER` | 1002–1024 |
| `Comanche055/KALCMANU_STEERING.agc` | `NEWDELHI`, `NEWANGL`, `INCRDCDU`, `MANUSTAT` | 414–419 |
| `Comanche055/JET_SELECTION_LOGIC.agc` | `PWORD` (TAU1/TAU2 check), `RWORD` (TAU check), phase-plane implicit in TAU generation | 1039–1062 |

---

## Behavior Summary

### Attitude Error Computation

The RCS DAP computes attitude error in body axes using a rotation matrix (AMGB)
that maps gimbal-axis CDU deltas into body-frame angular increments.

The process follows the comment block at `RCS-CSM_DIGITAL_AUTOPILOT.agc` p. 1005–1006:

```
DELRHO = AMGB × (CDU - CDU_prev)      (body-frame angle increment)

DRHO ← (1 - GAIN1) × DRHO + DELRHO - 0.1 × ADOT   (rate filter, eq. p.1004)

ADOT ← ADOT_prev + GAIN2 × DRHO + KMJ × DFT        (smoothed accel, p.1006)
```

The attitude **error** for autopilot use is the difference between the commanded angle
and the measured CDU angle, transformed through AMGB:

```
ERROR_body = AMGB × (THETAD - CDU_current)
```

where THETAD = THETADX/Y/Z (commanded angles in CDU counts, copied from CDUXD/Y/Z or
grabbed from present CDU when HOLDFLAG is positive).

In the code this maps to the erasable variables:
- `ERRORX` = pitch body-axis error (AGC: `TS ERRORX`, p. 1021 `MERUPDAT`)
- `ERRORY` = yaw body-axis error
- `ERRORZ` = roll body-axis error
All three are scaled **180 degrees per full-scale** (i.e., 1 unit = 180°/32768 ≈ 0.0055°).

For the FDAI display computation (`GETAKS`, p. 1023) the error in inertial axes is:
```
AK  = CTHETA - CDUX + AMGB1 × (WTEMP - CDUY)          (pitch needle)
AK1 = AMGB4 × (WTEMP - CDUY) + AMGB5 × (T5TEMP - CDUZ) (yaw needle)
AK2 = AMGB7 × (WTEMP - CDUY) + AMGB8 × (T5TEMP - CDUZ) (roll needle)
```

For the Rust implementation the AMGB matrix corresponds to the body-to-gimbal
rotation derived from REFSMMAT. The developer maps this to `Mat3x3` from
`agc_core::types::matrix`.

### Desired Attitude from KALCMANU

`KALCMANU_STEERING.agc` `NEWANGL` generates the commanded CDU angles `CDUXD/Y/Z` by:
1. Computing the new rotation matrix from stable-member to body axes: `MIS = MXM3(M, DEL)`.
2. Calling `DCMTOCDU` to extract three CDU Euler angles from the matrix.
3. Computing increments: `DELCDUX = (NCDU - BCDU) × QUADROT / 10` (angle per 0.1 s step).
4. Setting `HOLDFLAG = −1` to enable automatic steering (`MANUSTAT`, p. 416 line 141).

In Rust, the `compute_error` function receives pre-computed `desired_mat` (the REFSMMAT
times the desired attitude rotation) and the current CDU readings.

### Phase-Plane Switching Logic

The RCS DAP does NOT use a continuous proportional controller. It uses **bang-bang**
(on/off) jet firings determined by a phase-plane switching criterion.

The switching logic is implicit in the TAU computation:
- `TAU` (roll), `TAU1` (pitch), `TAU2` (yaw) are jet on-time accumulators.
- The jet selection logic checks the sign of TAU per axis to decide which direction
  to fire (`CCS TAU1` at `PWORD`, `JET_SELECTION_LOGIC.agc` line 105).
- `ERRORX/Y/Z` feed into TAU via the switching function at `JETS` (referenced from
  `SPNDXCHK`, `RCS-CSM_DIGITAL_AUTOPILOT.agc` line 806).

The phase-plane boundary is a straight-line switching function with slope `SLOPE = 0.24`
(initialized at `REDAP`, line 573: `CAF =.24` → `TS SLOPE`). The comment describes
this as "SLOPE = 0.6/SEC".

The deadband is encoded in `DBTABLE` (decoded by `S41.2` called from `REDAP`, line 551).
The crew-selectable deadband values are not visible in the files provided; the default
minimum deadband for attitude hold is approximately **0.3°** based on the AGC book
(O'Brien, pp. 312–334). The TVC roll DAP uses `DB` (from `TVCROLLDAP.agc` `ROLLOGIC`
line 248) scaled at 2^0 revolutions, with `OGA deadband = 5°` per the functional
description (p. 984).

For the Rust phase-plane function, the switching logic per axis is:

```
state = error + (SLOPE × rate)   (linear switching surface)

if abs(state) < deadband/2:
    JetDecision::None             (coast / inside deadband)
elif state > 0:
    JetDecision::Positive         (fire + direction jets)
else:
    JetDecision::Negative         (fire − direction jets)
```

The multiplier on rate (0.24 = slope parameter) represents the switching-line slope
in the (error, rate) phase plane. This implements the "parabola approximated by
straight line" policy described in `TVCROLLDAP.agc` pp. 986–988.

---

## Rust API

Module: `agc_core::control::attitude`

### Types

```rust
/// Body-axis attitude error in radians.
///
/// Positive pitch = nose up, positive yaw = nose left (body-frame convention).
/// Corresponds to ERRORX (pitch), ERRORY (yaw), ERRORZ (roll) in AGC erasable.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc MERUPDAT (p. 1020),
///             scaled 180 degrees full-scale (1 unit = π / 32768 rad).
pub struct AttitudeError {
    /// Pitch error, radians (body frame).
    pub pitch: f64,
    /// Yaw error, radians (body frame).
    pub yaw: f64,
    /// Roll error, radians (body frame).
    pub roll: f64,
}

/// Per-axis jet firing decision from the phase-plane switching function.
///
/// AGC source: Comanche055/JET_SELECTION_LOGIC.agc PWORD (TAU1 sign check, p. 1039);
///             RCS-CSM_DIGITAL_AUTOPILOT.agc T5PHASE2 TAU generation (p. 1017).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JetDecision {
    /// Fire positive-direction jets (+ torque on this axis).
    Positive,
    /// Fire negative-direction jets (− torque on this axis).
    Negative,
    /// Do not fire on this axis (inside deadband or rate already damping).
    None,
}
```

### Functions

```rust
/// Compute body-axis attitude error from current CDU angles and desired attitude matrix.
///
/// The desired attitude matrix `desired_mat` is the REFSMMAT composed with the
/// target rotation (e.g., from KALCMANU `NEWANGL` / `MIS` matrix).
/// The function extracts the error angles by comparing the matrix-derived Euler
/// angles against the current CDU readings (converted to radians).
///
/// Output is in radians; positive pitch = nose up, positive yaw = nose left.
/// All-zero error is returned when CDU angles exactly match the desired matrix.
///
/// Invariants:
///   - Input `desired_mat` must be a valid rotation matrix (orthonormal); behavior
///     is undefined for degenerate inputs (det ≠ ±1).
///   - Output components are always finite (no NaN/Inf) for finite inputs.
///   - The AMGB body-to-gimbal transform is the transpose of `desired_mat` composed
///     with the gimbal-frame identity; for the Rust port, caller passes the full
///     rotation matrix directly and the function extracts error by matrix difference.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   GETAKS (attitude error display, p. 1023), MERUPDAT (error accumulation, p. 1019),
///   AMGB1/4/5/7/8 matrix elements (p. 1006).
/// Comanche055/KALCMANU_STEERING.agc NEWANGL / DCMTOCDU (p. 414).
///
/// Units:
///   Input: `current_cdu` — raw CDU counts via `CduAngle`, converted internally to radians.
///   Input: `desired_mat` — dimensionless rotation matrix (Mat3x3).
///   Output: `AttitudeError` — radians.
pub fn compute_error(
    current_cdu: &[agc_core::types::CduAngle; 3],
    desired_mat: &agc_core::types::Mat3x3,
) -> AttitudeError;

/// Evaluate the phase-plane switching function for one body axis.
///
/// Implements the linear switching-surface approximation used throughout the
/// RCS and TVC roll DAPs.  The switching variable is:
///   `s = error + SLOPE × rate`
/// where SLOPE = 0.24 (≈ 0.6/s × sample_period).
///
/// Decision:
///   |s| < deadband/2  →  None
///   s > 0             →  Positive
///   s < 0             →  Negative
///
/// Caller must provide the per-axis deadband (confirm from DBTABLE decoding in S41.2;
/// default attitude-hold deadband ≈ 0.3° = 5.24e-3 rad).
///
/// Invariants:
///   - `deadband` must be > 0.0; function panics in debug, saturates to None in release
///     if deadband ≤ 0.0.
///   - `error == 0.0 && rate == 0.0` always returns `None`.
///   - Finite `error` and `rate` always produce a finite (non-NaN) decision.
///   - This function is pure: no side effects, no state.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc
///   SLOPE initialization (REDAP, line 572: `CAF =.24 → TS SLOPE`);
///   TAU generation at JETS label (p. 1020, referenced from SPNDXCHK).
/// Comanche055/TVCROLLDAP.agc ROLLOGIC (switching parabola / line, pp. 987–988).
///
/// Units:
///   `error`    — radians
///   `rate`     — radians/second
///   `deadband` — radians (must be > 0)
pub fn phase_plane_decision(error: f64, rate: f64, deadband: f64) -> JetDecision;
```

---

## Scale Factors

| Quantity | AGC register | AGC scale | `f64` conversion |
|---|---|---|---|
| CDU angle | CDUX/Y/Z | 2^15 counts = π rad | `CduAngle::to_radians()` |
| Attitude error | ERRORX/Y/Z | 180°/32768 per count | `(i16 as f64) / 32768.0 * π` |
| Phase-plane slope | SLOPE | `DEC .24` = 0.24 (dimensionless) | 0.24 directly |
| Body rate DRHO | DRHO/1/2 | 180° per full-scale (DP) | `(DP_value as f64) / 2^28 * π` rad/s |
| FDAI error needle | AK/AK1/AK2 | 180° full-scale; 384 bits = 16.875° | display only |
| TVC roll deadband | DB | 2^0 revolutions | `5.0 / 360.0` rev = `5° × 2π/360` rad |

---

## Invariants

- `deadband > 0.0` is a hard precondition; `phase_plane_decision` returns `None` if
  `deadband <= 0.0` in release builds and `debug_assert!(deadband > 0.0)` in debug.
- `compute_error` returns `AttitudeError { pitch: 0.0, yaw: 0.0, roll: 0.0 }` when
  `current_cdu` encodes the same orientation as `desired_mat`.
- Both functions are pure (`&` inputs only); no static state, no I/O.
- `f64` arithmetic: all intermediate results must be finite. If any CDU angle is
  NaN or Inf (hardware error), `compute_error` must return zero error and the caller's
  alarm path must handle the hardware fault.
- **Axis independence**: pitch, yaw, and roll errors are computed independently.
  The AMGB cross-coupling terms (`AMGB1`, `AMGB4`, `AMGB5`, `AMGB7`, `AMGB8`) are
  accounted for in `compute_error` via matrix multiplication; `phase_plane_decision`
  operates on a single already-decoupled axis value.

---

## Test Cases

1. **Zero error and rate returns None**: Call `phase_plane_decision(0.0, 0.0, 5.24e-3)`.
   Assert result is `JetDecision::None`.

2. **Positive error beyond deadband returns Positive**: Call
   `phase_plane_decision(0.01, 0.0, 5.24e-3)` (error = 0.01 rad ≈ 0.57°, deadband = 0.3°).
   Assert result is `JetDecision::Positive`.

3. **Phase-plane hysteresis edge (rate cancels error)**: Call
   `phase_plane_decision(0.01, -0.1, 5.24e-3)`.
   The switching variable `s = 0.01 + 0.24 × (−0.1) = 0.01 − 0.024 = −0.014`.
   Assert result is `JetDecision::Negative` (rate dominates, switch to opposite direction).

4. **Rate damping: negative rate cancels positive error inside deadband**:
   Call `phase_plane_decision(0.003, -0.015, 5.24e-3)`.
   `s = 0.003 + 0.24 × (−0.015) = 0.003 − 0.0036 = −0.0006`.
   `|s| = 0.0006 < 0.00262 = deadband/2`. Assert `JetDecision::None`.

5. **Pitch/yaw/roll independence**: Construct `current_cdu` where only Y-axis (yaw CDU)
   differs from `desired_mat`. Assert `compute_error` returns non-zero `yaw` and zero
   `pitch` and `roll` (within floating-point tolerance 1e-9 rad).

---

## agc-sim Impact

- `MissionState` panel: add `att_error_deg: [f32; 3]` (pitch/yaw/roll in degrees,
  one decimal place) to be rendered in the Mission State pane.
- `SimLog`: no new log events (error is continuous state, not an event).
- `dsky_terminal.rs`: no new DSKY fields required (FDAI needles are hardware;
  agc-sim represents them via the Mission State numbers).
