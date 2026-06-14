# Specification: `navigation/lunar_libration` Module — Moon-fixed → MCI Rotation

**Status**: Approved for implementation
**Module path**: `agc-core/src/navigation/lunar_libration.rs`
**Architecture reference**: `docs/architecture.md` §9 (Navigation Math)
**Related specs**:
- `specs/planetary-spec.md` — provides `met_to_jd`, the time base.
- `specs/p22-spec.md` and `specs/p21_p22-spec.md` — lunar-landmark consumer.
- `specs/state-vector-spec.md` — defines `Frame::MoonInertial` (MCI).
**Glossary cross-reference**: `docs/glossary.md` — MCI, MCMF, libration.
**Reference**:
- B. A. Archinal *et al.* (2018) — "Report of the IAU Working Group on
  Cartographic Coordinates and Rotational Elements: 2015",
  *Celestial Mechanics and Dynamical Astronomy* 130:22, Table 4 ("Moon").
- Eckhardt (1981) — equivalent analytical libration theory; series form
  matches the IAU 2015 polynomials used here.
- Seidelmann (ed.), *Explanatory Supplement to the Astronomical
  Almanac*, §6.27 — body-fixed → inertial rotation form
  `R_z(α₀ + 90°) · R_x(90° − δ₀) · R_z(−W)`.

---

## 1. Purpose and Scope

`navigation::lunar_libration` provides the **rotation matrix** that
maps Moon-fixed (Mean Earth / Polar Axis — "ME") Cartesian coordinates
into the AGC's MCI / J2000 mean-equatorial inertial frame. The module
exists for one reason: P22's lunar-landmark navigation needs to convert
selenographic landmark positions (stored as Moon-fixed coordinates)
into the MCI frame where the SERVICER integrates and the Kalman filter
operates.

Until issue #56 the project ignored libration — Moon-fixed coordinates
were treated as MCI coordinates. That introduces up to ~150 km
systematic bias on a single lunar landmark's inertial position. With
the 0.1 mrad sextant noise floor this bias dwarfs the Kalman
convergence target (~5 km) and corrupts the CSM state estimate during
sustained lunar-orbit navigation. The module fixes that.

### What this module provides

- `moon_fixed_to_inertial(epoch: Met) -> Mat3x3` — the only public
  function. Returns the body-fixed → inertial rotation at a given
  Mission Elapsed Time.

### What this module does NOT provide

- The inverse rotation. Callers that need MCI → Moon-fixed transpose
  the returned matrix (it is orthonormal); a dedicated helper has not
  been needed.
- Selenographic-spherical (lat, lon, alt) → Cartesian conversion. That
  is the responsibility of the caller (P22 / star_catalog) — the spec
  shows the formula in §1.1.
- Rates / angular velocity. Only the instantaneous rotation matrix.
  Time derivatives are not needed because the lunar landmark is
  sampled at the same time the matrix is computed; relative motion
  between consecutive marks is small enough that finite-difference is
  not required by the consumer.
- An MCMF helper enum on `Frame`. The matrix is consumed as a 3×3 and
  the result is a `Frame::MoonInertial` vector; no `Frame::MoonFixed`
  variant is introduced.

### 1.1 Use pattern

A caller that has a selenographic landmark `(lat_rad, lon_rad, alt_m)`
and wants the MCI position at MET `t` does:

```rust
let r_seleno = [
    (R_MOON + alt_m) * cos(lat) * cos(lon),
    (R_MOON + alt_m) * cos(lat) * sin(lon),
    (R_MOON + alt_m) * sin(lat),
];
let r_mci = mxv(moon_fixed_to_inertial(t), r_seleno);
```

`R_MOON` is `agc_core::navigation::gravity::R_MOON`.

---

## 2. Convention

### 2.1 Body-fixed frame (Mean Earth / Polar Axis, "ME")

The IAU "Mean Earth / Polar Axis" frame for the Moon:

- X-axis: prime meridian on the lunar equator.
- Z-axis: mean rotation pole.
- Y-axis: completes the right-handed triad.

This is the frame in which selenographic (lat, lon, alt) coordinates
are most directly Cartesian.

### 2.2 Inertial frame (MCI)

The AGC's MCI is treated as the J2000 mean-equatorial inertial frame
for the Apollo mission window. The precession difference between J2000
and Mean of 1969.5 is ~50 arcsec/yr × 1 yr × 384 400 km ≈ 30 km at the
lunar surface — within the accuracy budget set by ADR-013 and below the
~5 km Kalman convergence target.

### 2.3 Rotation form

Body-fixed → inertial:

```
M_b2i = R_z(α₀ + 90°) · R_x(90° − δ₀) · R_z(−W)
```

where:

- `α₀, δ₀` — right ascension and declination of the lunar rotation
  pole (J2000 frame).
- `W` — prime meridian angle, measured eastward from the ascending
  node of the lunar equator on the J2000 equator.
- `R_x, R_z` — active (right-handed) rotation matrices.

A Cartesian column vector in the body-fixed frame, multiplied by
`M_b2i`, becomes the equivalent vector in MCI.

---

## 3. Rust API

### 3.1 Function

```rust
pub fn moon_fixed_to_inertial(epoch: Met) -> Mat3x3;
```

Pure, total, allocation-free, `no_std`-safe.

### 3.2 Constants (private)

| Name | Value | Meaning |
|---|---|---|
| `J2000_JD` | `2_451_545.0` | J2000.0 Julian Day epoch |
| `DEG2RAD` | `π / 180` | Degrees → radians conversion |
| `ALPHA_E_SIN[13]` | tabulated (deg) | Periodic-term amplitudes for `α₀` |
| `DELTA_E_COS[13]` | tabulated (deg) | Periodic-term amplitudes for `δ₀` |
| `W_E_SIN[13]` | tabulated (deg) | Periodic-term amplitudes for `W` |
| `E_PHASE[13]` | `(constant_deg, rate_deg_per_day)` | Phase polynomials for the 13 IAU `E_i` arguments |

The 13 periodic terms (`E_1 … E_13`) are the physical-libration
oscillations from Archinal *et al.* (2018) Table 4 ("Moon"). The
amplitudes are entered as tabulated in degrees; the inner loop
multiplies them by the corresponding `sin` or `cos` of the phase
polynomial at the call time.

---

## 4. Functional Requirements

### 4.1 `moon_fixed_to_inertial(epoch) -> Mat3x3`

1. `jd = met_to_jd(epoch)` (via `navigation::planetary`).
2. `d = jd − J2000_JD` (days from J2000.0).
3. `t_cy = d / 36 525` (Julian centuries from J2000).
4. For each `i ∈ 0..13`, compute the phase angle
   `arg_i = (E_PHASE[i].0 + E_PHASE[i].1 · d) · DEG2RAD`. Compute
   `e_sin[i] = sin(arg_i)`, `e_cos[i] = cos(arg_i)`.
5. Compute the right ascension of the Moon's pole:
   ```
   α₀_deg = 269.9949 + 0.0031 · t_cy + Σ_i ALPHA_E_SIN[i] · e_sin[i]
   α₀     = α₀_deg · DEG2RAD
   ```
6. Compute the declination of the Moon's pole:
   ```
   δ₀_deg = 66.5392 + 0.0130 · t_cy + Σ_i DELTA_E_COS[i] · e_cos[i]
   δ₀     = δ₀_deg · DEG2RAD
   ```
7. Compute the prime meridian angle:
   ```
   W_deg = 38.3213 + 13.176_358_15 · d − 1.4e−12 · d² + Σ_i W_E_SIN[i] · e_sin[i]
   W     = W_deg · DEG2RAD
   ```
8. Compose the body-fixed → inertial rotation:
   ```
   φ = π/2 − δ₀
   ψ = π/2 + α₀
   M_b2i = R_z(ψ) · R_x(φ) · R_z(−W)
   ```
9. Return `M_b2i` as `Mat3x3`.

Postcondition: `M_b2i` is orthonormal with determinant `+1` (a proper
rotation). The IAU polynomials produce an orthonormal matrix by
construction; no explicit re-orthonormalisation is needed.

### 4.2 Helper functions (private)

`rotation_z(theta) -> Mat3x3` and `rotation_x(theta) -> Mat3x3` build
the elementary right-handed rotation matrices about the Z and X axes
respectively. Both are `#[inline]` and only depend on `libm::sin/cos`.

---

## 5. Numerical Notes

- The argument of `sin` and `cos` grows ~13 deg/day for the
  high-rate terms (`E_3`, `E_4`, `E_7`, `E_13`). After 10 years of
  mission time the argument is `~50 000` rad, well within `f64`
  precision; `libm::sin/cos` accept arbitrarily large arguments.
- The `d²` term in `W` (`−1.4e−12`) is the tiny secular-tidal
  correction. Over the Apollo 8 mission window (~7 days) its
  contribution is `~10⁻¹⁰` deg — negligible compared to the periodic
  terms — but carrying it costs nothing and keeps the implementation
  bit-faithful to IAU 2015.
- The matrix multiplication is the standard 3×3 product
  (`math::linalg::mxm`); no Strassen-style tricks are warranted.

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `libm::sin`, `libm::cos` | Trigonometric evaluation in the phase polynomials and the elementary rotations. |
| `crate::math::linalg::{mxm, transpose}` | 3×3 matrix multiplication (composition); `transpose` is used by the test module only. |
| `crate::navigation::planetary::met_to_jd` | MET → Julian Day conversion. |
| `crate::types::{Mat3x3, Met}` | Matrix and time types. |
| `core::f64::consts::PI` | Angle scaling. |

No global state. No I/O.

---

## 7. Module Layout

```
src/navigation/lunar_libration.rs
├── const J2000_JD: f64
├── const DEG2RAD: f64
├── const ALPHA_E_SIN[13]: f64
├── const DELTA_E_COS[13]: f64
├── const W_E_SIN[13]: f64
├── const E_PHASE[13]: (f64, f64)
├── pub fn moon_fixed_to_inertial(epoch: Met) -> Mat3x3
├── fn rotation_z(theta: f64) -> Mat3x3        (private, #[inline])
├── fn rotation_x(theta: f64) -> Mat3x3        (private, #[inline])
└── #[cfg(test)] mod tests
```

---

## 8. Test Cases

The implementation in
`agc-core/src/navigation/lunar_libration.rs::tests` provides:

| ID | What is verified |
|---|---|
| `tc_lib_1_rotation_is_orthonormal` | At each of MET 0, 1 day, 5 days, 15 days, `M · Mᵀ = I` within `1e-12` per entry and `det(M) ≈ 1` within `1e-12`. Proves the matrix is a proper rotation. |
| `tc_lib_2_secular_prime_meridian_rate` | The selenographic prime-meridian unit vector (selenographic (0°, 0°)) rotates between `12.5°` and `13.8°` in inertial space over one Earth day — the expected `13.18°` secular rate ± libration wobble (~3.5° peak amplitude on `W`). |
| `tc_lib_3_pole_direction_matches_iau_constants` | The body-Z axis maps to the inertial direction `(cos δ₀ · cos α₀, cos δ₀ · sin α₀, sin δ₀) ≈ (0, −0.398, +0.917)` within `±0.05` per component. The cleanest sign / composition check on the matrix. |
| `tc_lib_4_landmark_inertial_position_moves` | A landmark at selenographic `(0°, 0°)` moves more than 100 km in inertial space over a 10-day mission window. Smoke-test confirming that the model has the magnitude the issue calls out — ignoring libration would have introduced exactly this much bias. |

---

## 9. Spec Quality Checklist

- [x] IAU 2015 source cited (Archinal *et al.* Table 4) (§Reference, §3.2).
- [x] Rotation-form convention documented and matched to Seidelmann §6.27 (§2.3).
- [x] Apollo-era frame approximation (J2000 ≈ Mean of 1969.5)
      explicitly justified against the ADR-013 accuracy budget (§2.2).
- [x] Full algorithm steps with explicit polynomial constants and
      argument units (§4.1).
- [x] Dependencies listed (§6).
- [x] Test coverage summarised (§8).
- [x] Mission motivation (issue #56's ~150 km bias) recorded (§1).
