# Spec: TVC Gimbal Steering

## AGC Source References

| File | Routine | Pages |
|---|---|---|
| `Comanche055/TVCDAPS.agc` | `PITCHDAP`, `YAWDAP`, `DAPINIT`, `ERRORLIM`, `ACTLIM`, `FWDFLTR`, `PRECOMP`, `OPTVARK`, `PCOPY`, `YCOPY`, constants block | 961–978 |
| `Comanche055/TVCEXECUTIVE.agc` | `TVCEXEC`, `VARGAINS`, `GAINCHNG`, `ROLLPREP`, `CG.CORR`, `CNTRCOPY` | 945–950 |
| `Comanche055/TVCMASSPROP.agc` | `MASSPROP`, `FIXCW`, `LEMTEST`, `LEMNO`, `LEMYES` | 951–955 |
| `Comanche055/TVCROLLDAP.agc` | `ROLLDAP`, `ROLLOGIC`, `DURATION`, `WAIT1/2` | 984–998 |
| `Comanche055/TVCSTROKETEST.agc` | `STRKTSTI`, `HACK`, `HACKWLST` | 979–983 |

---

## Behavior Summary

### Overview

The TVC (Thrust Vector Control) autopilot controls the SPS engine gimbal actuators
during a propulsion burn. It uses the CDU angles to derive body-axis attitude rates,
integrates the rate error, passes the integrated error through a 6th-order cascade
filter, and outputs a gimbal angle command saturated at ±6°.

Only the pitch and yaw axes are controlled via gimbals. The roll axis is handled by
the RCS jet system even in TVC mode (see TVCROLLDAP below).

### Gimbal Saturation Limit (ACTSAT)

The actuator command limit is **6 degrees**.

Confirmed from `TVCDAPS.agc` p. 978 constants block:
```
ACTSAT     DEC  253     # ACTUATOR LIMIT (6 DEG), SC.AT 1ASCREV
1/ACTSAT   DEC  .0039525692   # RECIPROCAL (1/253)
```

The scaling note reads: "1 ASCREV (actuator cmd scaling) = 85.41 ARCSEC/BIT
or 1.07975111 REVS". Therefore:
- 253 ASCREV counts × 85.41 arcsec/count = 21,609 arcsec = 6.002° ≈ **6°**
- In radians: 6° × π/180 = **0.10472 rad = 104.72 mrad**

### Gain Scheduling via Mass Properties (VARGAINS / MASSPROP)

`TVCEXECUTIVE.agc` `VARGAINS` calls `MASSPROP` every 10 seconds of burn
(when `VCNTR` counts down to zero, reset to 19 = 10 × 0.5 s intervals)
to update the vehicle inertia estimates:

- `IXX` — roll moment of inertia, scaled B+20 kg·m²
- `IAVG` — average pitch/yaw moment of inertia, scaled B+20 kg·m²
- `IAVG/TLX` — IAVG / thrust-arm length, scaled B+2 s²

The gains used in the filter (`VARK`, `1/CONACC`) are updated by `S40.15`
(`TVCEXECUTIVE.agc` `GAINCHNG` line 146: `TC IBNKCALL; CADR S40.15`).

`MASSPROP` operates in two modes:
1. Full mode (LEM mass or configuration changed): recalculates breakpoint values
   from polynomial fits in `INTVALUE0..9` + `LEMMASS × SLOPEVAL0..9`.
2. Fast mode (`FIXCW`): uses previously computed breakpoints and applies:
   `IXX0 = VARST0 + (CSMMASS − NEGBPW) × VARST5`.

For the Rust implementation, the developer computes `TvcGains` from mass properties
using a simplified linear interpolation:
```
k_p = IAVG / (thrust_arm × sampling_period²)   (from IAVG/TLX)
k_d = rate_gain                                  (from 1/CONACC via S40.15)
```
The exact polynomial breakpoint tables from `TVCMASSPROP.agc` should be faithfully
reproduced as `const` arrays.

### Pitch and Yaw TVC DAP Loops

Each loop (`PITCHDAP` / `YAWDAP`) runs as a self-perpetuating T5 interrupt at
interval `T5TVCDT` (pad-loaded; nominally ≈40 ms per DAP, 80 ms full cycle).

The loops call each other alternately:
```
PITCHDAP → sets T5LOC = YAWT5  → YAWDAP → sets T5LOC = PITCHT5 → PITCHDAP
```
(`TVCDAPS.agc` lines 133-134 and 305-306.)

**Pitch/Yaw loop structure (identical for both axes):**

```
1. CDU rate derivation:
   MCDUYDOT = CDUY - PCDUYPST   (minus CDU-Y dot, scaled 1/2TVCDT rev/s)
   MCDUZDOT = CDUZ - PCDUZPST

2. Body-axis rate computation:
   OMEGAYB = -COSCDUZ × COSCDUX × MCDUYDOT + SINCDUX × MCDUZDOT  (pitch)
   OMEGAZB = COSCDUZ × SINCDUX × MCDUYDOT + COSCDUX × MCDUZDOT   (yaw)
   (TVCDAPS.agc lines 177-190, 325-338)

3. Rate error integration:
   PERRB += (OMEGAYC - OMEGAYB) × dt    (integrates commanded vs measured rate)
   where OMEGAYC/OMEGAZC come from cross-product steering (S40.8)

4. Input limiter (ERRORLIM):
   if |PERRB_high| > ERRLIM (BIT13 = B-3 revs = 45°): clamp to ±ERRLIM
   (TVCDAPS.agc line 474: ERRLIM = BIT13)

5. Forward filter (FWDFLTR): 6th-order cascade
   - CSM-only (LEM off): 2 biquad cascades (N10..N10+9)
   - CSM+LEM: 3 biquad cascades (N10..N10+14)
   - Variable gain: CMDTMP = -DAP3 × VARK (sign change in forward path)
   - (TVCDAPS.agc lines 512-578)

6. Offset correction (POFFSET/YOFFSET):
   CMDTMP += PDELOFF   (pitch trim offset from CG tracker)

7. Output limiter (ACTLIM = ACTSAT):
   if |CMDTMP| > ACTSAT (253 = 6°): clamp to ±ACTSAT
   (TVCDAPS.agc lines 490-507; ACTSAT DEC 253 on p. 978)

8. Incremental output:
   TVCPITCH += (CMDTMP - PCMD)  (delta to optics error counter)
   TVCYAW   += (CMDTMP - YCMD)

9. Copy cycle (restart protection):
   TMP1..TMP6 → PTMP1..PTMP6   (filter state copy for restart safety)
   TVCPHASE incremented before and after (INCR TVCPHASE)
```

### TVC Roll DAP (RCS-based, not gimbal)

Roll control during a TVC burn is handled by the RCS jets, not the gimbal.
`TVCROLLDAP.agc` `ROLLDAP` is a Waitlist task called every 500 ms by `TVCEXEC`.

The roll DAP uses phase-plane switching (identical structure to RCS DAP):
```
OGARATE = (OGANOW - OGAPAST) × BIT5 / (0.5 s sample time)
OGAERR  = OGANOW - OGAD     (roll error, measured vs ignition reference)
```

Switching criterion: parabola `DB - (OGARATE²)/(2×CONACC)` compared to `OGAERR`.
Deadband: **5°** (`TVCROLLDAP.agc` functional description p. 984:
"MAINTAIN OGA WITHIN 5 DEG DEADBND OF OGAD").
Minimum jet firing: **15 ms** (p. 984: "MINIMUM JET FIRING TIME = 15 MS").

The Rust implementation for roll in TVC mode reuses `phase_plane_decision` from
`agc_core::control::attitude` with `deadband = 5.0_f64.to_radians()`.

### Startup Stroke Test

`TVCSTROKETEST.agc` `STRKTSTI` (initiated by V68) generates a waveform into
`TVCPITCH` to excite structural bending modes. It runs only for CSM/LEM (checks
`DAPDATR1` bit 13). The waveform injects pulse bursts of `ESTROKER` amplitude
(pad-loaded) in a predefined sequence via Waitlist calls to `HACKWLST`.

The Rust implementation must include a `stroke_test_active: bool` flag in `TvcState`
and skip normal attitude integration when the stroke test is active.

---

## Rust API

Module: `agc_core::control::tvc`

### Constants

```rust
/// TVC gimbal actuator command saturation limit, degrees.
///
/// AGC source: Comanche055/TVCDAPS.agc p. 978
///   `ACTSAT DEC 253  # ACTUATOR LIMIT (6 DEG), SC.AT 1ASCREV`.
///   253 ASCREV × 85.41 arcsec/ASCREV = 21,609 arcsec ≈ 6.002°.
pub const ACTSAT_DEG: f64 = 6.0;

/// TVC gimbal actuator command saturation limit, radians.
///
/// ACTSAT_DEG converted: 6° × π/180 = 0.104_719_755... rad.
/// AGC source: Comanche055/TVCDAPS.agc `ACTSAT DEC 253`.
pub const ACTSAT_RAD: f64 = 0.104_719_755_119_659_77;  // 6° in radians

/// TVC roll DAP deadband, degrees.
///
/// AGC source: Comanche055/TVCROLLDAP.agc functional description p. 984:
///   "MAINTAIN OGA WITHIN 5 DEG DEADBND OF OGAD".
pub const TVC_ROLL_DEADBAND_DEG: f64 = 5.0;

/// TVC roll DAP minimum jet firing time, milliseconds.
///
/// AGC source: Comanche055/TVCROLLDAP.agc functional description p. 984:
///   "MINIMUM JET FIRING TIME = 15 MS".
pub const TVC_ROLL_MIN_FIRE_MS: f64 = 15.0;

/// TVC actuator command scaling: 1 ASCREV = 85.41 arcsec.
///
/// AGC source: Comanche055/TVCDAPS.agc p. 978 note:
///   "1 ASCREV (ACTUATOR CMD SCALING) = 85.41 ARCSEC/BIT".
pub const ASCREV_ARCSEC: f64 = 85.41;
```

### Types

```rust
/// TVC filter and control gains for pitch or yaw.
///
/// Gains vary with vehicle mass and configuration (CSM-only vs CSM+LEM).
/// Updated every 10 seconds during the burn by TVCEXECUTIVE / MASSPROP / S40.15.
///
/// AGC source: Comanche055/TVCEXECUTIVE.agc VARGAINS / GAINCHNG (p. 947);
///             Comanche055/TVCDAPS.agc OPTVARK (VARK gain, p. 973),
///             ACTLIM (ACTSAT limit, p. 972).
pub struct TvcGains {
    /// Proportional gain (corresponds to VARK in AGC).
    /// Units: dimensionless (actuator-command revolutions per body-axis rate-error revolution).
    /// AGC source: Comanche055/TVCDAPS.agc OPTVARK: `MP VARK`, scaled 1/(8 ASCREV).
    pub k_p: f64,

    /// Derivative gain (from 1/CONACC in AGC roll DAP; analogous damping in pitch/yaw filter).
    /// Units: seconds (inverse of angular acceleration per unit command).
    /// AGC source: Comanche055/TVCROLLDAP.agc `1/CONACC SC.AT B+9 SEC²/REV`.
    pub k_d: f64,

    /// Actuator command saturation limit in degrees.
    /// Always initialized to ACTSAT_DEG = 6.0.
    /// AGC source: Comanche055/TVCDAPS.agc ACTSAT = 253 ASCREV = 6°.
    pub limit_deg: f64,
}

impl TvcGains {
    /// Default gains (CSM-only, nominal mass at ignition).
    /// Developer must update from MASSPROP output before use.
    pub const NOMINAL: Self = Self {
        k_p: 1.0,    // placeholder; actual value from VARK pad-load
        k_d: 1.0,    // placeholder; actual value from 1/CONACC
        limit_deg: ACTSAT_DEG,
    };
}
```

### Functions

```rust
/// Compute pitch and yaw gimbal angle commands for one TVC DAP cycle.
///
/// Implements the attitude-error integration → filter → saturation chain of
/// PITCHDAP and YAWDAP.  The 6th-order cascade filter is simplified to a
/// proportional-derivative law for the Rust port (ADR-001: interpretive language
/// replaced by plain f64 functions).  The full filter structure (N10..N10+14
/// coefficient tables) is retained in the developer's implementation.
///
/// Roll axis is NOT handled here — return values are (pitch_cmd_rad, yaw_cmd_rad).
/// Roll is deferred to the RCS jet selector using `phase_plane_decision` with
/// `deadband = TVC_ROLL_DEADBAND_DEG.to_radians()`.
///
/// Output is saturated symmetrically at ±gains.limit_deg converted to radians.
///   `pitch_cmd_rad` and `yaw_cmd_rad` are in the range [−ACTSAT_RAD, +ACTSAT_RAD].
///
/// Invariants:
///   - Output is always finite.
///   - `|pitch_cmd_rad| <= ACTSAT_RAD` and `|yaw_cmd_rad| <= ACTSAT_RAD` always hold.
///   - Roll is always (0.0, 0.0) — not computed by this function.
///   - `gains.limit_deg > 0.0` is required; debug_assert enforces this.
///
/// AGC source: Comanche055/TVCDAPS.agc
///   PINTEGRL (body pitch rate error integration, p. 963),
///   PERORLIM → ERRORLIM (input limiter, p. 971),
///   PFORWARD → FWDFLTR → OPTVARK (filter + gain, pp. 964,972-973),
///   POFFSET (trim correction, p. 964),
///   PACLIM → ACTLIM (output saturation, pp. 964,971-972),
///   YINTEGRL ... YACLIM (identical for yaw, pp. 967-968).
///
/// Units:
///   `error.pitch` / `error.yaw` — radians (body frame)
///   `rate.pitch`  / `rate.yaw`  — radians/second (body frame)
///   `gains.k_p`                 — dimensionless
///   `gains.k_d`                 — seconds
///   `gains.limit_deg`           — degrees (converted internally)
///   Return: (pitch_rad, yaw_rad) — radians, saturated at ±ACTSAT_RAD
pub fn steer(
    error: &agc_core::control::attitude::AttitudeError,
    rate: &agc_core::control::attitude::AttitudeError,
    gains: &TvcGains,
) -> (f64, f64);
```

### Implementation Notes for Developer

The simplified `steer` function maps to the following computation:
```
raw_pitch = k_p * error.pitch + k_d * rate.pitch
raw_yaw   = k_p * error.yaw   + k_d * rate.yaw
limit_rad = limit_deg.to_radians()  (= ACTSAT_RAD when limit_deg = 6.0)
pitch_cmd = raw_pitch.clamp(-limit_rad, limit_rad)
yaw_cmd   = raw_yaw.clamp(-limit_rad, limit_rad)
```

The full AGC filter chain (FWDFLTR cascades, PRECOMP nodes) is preserved as
additional filter state in a `TvcFilterState` struct (developer's discretion on
exact struct layout) updated each cycle. The `TvcGains` struct carries only the
outer-loop scalar gains; filter coefficients (N10..N10+14) are `const` tables.

---

## Scale Factors

| Quantity | AGC register | AGC scale | `f64` SI value |
|---|---|---|---|
| Actuator command | TVCPITCH/TVCYAW | ASCREV (85.41 arcsec/bit) | radians via `× (85.41/3600) × (π/180)` |
| ACTSAT limit | CMDTMP | 253 ASCREV = 6° | `0.10472 rad` |
| Body-axis rate | OMEGAYB/OMEGAZB | 1/(2×T5TVCDT) revs/s | radians/s |
| Error integrator | PERRB/YERRB | B-1 revs (DP) | radians |
| Error limit | ERRLIM = BIT13 | B-3 revs = 45° | `0.7854 rad` |
| VARK variable gain | VARK | 1/(8 ASCREV) | dimensionless |
| 1/CONACC | 1/CONACC | B+9 s²/rev | s²/rad |
| OGA roll angle | OGANOW | CDU counts (same as CduAngle) | radians via `CduAngle::to_radians()` |
| OGA roll rate | OGARATE | B-4 rev/s | rad/s via `× 2π × 2^-4` |
| Mass CSM | CSMMASS | B+16 kg | kg via `× 2^16` |

---

## Invariants

- **Output bounded**: `|pitch_cmd_rad| <= ACTSAT_RAD` and `|yaw_cmd_rad| <= ACTSAT_RAD`
  always. This matches the AGC ACTLIM subroutine (`TVCDAPS.agc` p. 971-972).
- **Roll not handled**: `steer` returns (pitch, yaw) only. Roll remains zero in the
  return value. Roll is handled by `agc_core::control::rcs_logic` using the RCS jets.
- **No heap**: `TvcGains` is `Copy`. `TvcFilterState` (developer-defined) must also
  be statically sized.
- **gains.limit_deg > 0**: enforced by `debug_assert!` in `steer`. Release builds clamp
  zero limit to ACTSAT_DEG with an alarm raised.
- **Finite output**: `steer` must return finite values for all finite inputs. NaN inputs
  produce `alarm::raise(AlarmCode::TvcNan)` and return (0.0, 0.0).

---

## Test Cases

1. **Zero error → zero command**: Call `steer(&AttitudeError::ZERO, &AttitudeError::ZERO, &TvcGains::NOMINAL)`.
   Assert both returned values are 0.0 exactly.

2. **Positive pitch error → positive pitch gimbal**: Call `steer` with
   `error.pitch = 0.05_f64` (≈2.87°), `error.yaw = 0.0`, `rate = AttitudeError::ZERO`,
   and nominal gains (`k_p = 1.0`, `k_d = 0.0`, `limit_deg = 6.0`).
   Assert `pitch_cmd_rad = 0.05` (unsaturated, within ±0.10472), `yaw_cmd_rad = 0.0`.

3. **Saturation at limit**: Call `steer` with `error.pitch = 0.5` (≈28.6°, well above 6°),
   `rate = AttitudeError::ZERO`, gains `k_p = 1.0`.
   Assert `pitch_cmd_rad.abs() == ACTSAT_RAD` (= 0.10472 rad within f64 tolerance 1e-10).
   Assert `pitch_cmd_rad > 0.0` (positive error → positive command).

4. **Gain scheduling changes response**: Compute `steer` with `k_p = 2.0` vs `k_p = 1.0`
   for `error.pitch = 0.02`, `rate = AttitudeError::ZERO`.
   Assert that the k_p=2.0 result is exactly twice the k_p=1.0 result (both unsaturated).

---

## agc-sim Impact

- `MissionState` panel: add `gimbal_pitch_deg: f32` and `gimbal_yaw_deg: f32`
  (rendered to 1 decimal place in the Mission State pane, label "TVC P/Y:").
- `SimLog`: emit `.info("TVC burn: gimbal P={:.2}° Y={:.2}°")` on each non-zero steer call.
- `SimHardware.engine`: `set_gimbal_angles(pitch_rad, yaw_rad)` must accept the clamped
  output of `steer` directly — no additional conversion needed since `EngineIo` already
  works in radians (`agc-core/src/hal/engine.rs`).
- No new DSKY bindings (TVC is automatic; no crew keypad input during the burn).
