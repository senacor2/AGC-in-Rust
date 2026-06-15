# Specification: `navigation/planetary` Module — Lunar Ephemeris

**Status**: Approved for implementation
**Module path**: `agc-core/src/navigation/planetary.rs`
**Architecture reference**: `docs/architecture.md` §9 (Navigation Math)
**Related specs**:
- `specs/state-vector-spec.md` — defines `Frame::MoonInertial` (MCI) which this module's outputs feed into.
- `specs/lunar-libration-spec.md` — consumer of `met_to_jd`.
- `specs/integration-spec.md` — Cowell third-body perturbation against Moon position.
- `specs/conics-spec.md` — Lambert / Kepler solvers that use the inertial Moon position when working in the lunar SOI.
- `specs/lunar-ephemeris-research.md` — analyst-phase research note documenting the historical AGC's 9th-degree polynomial implementation and the rationale for the Rust port's choice of Meeus instead.
**Glossary cross-reference**: `docs/glossary.md` — MCI, JD, Mean of 1969.5.
**Reference**:
- Jean Meeus, *Astronomical Algorithms*, 2nd ed., Willmann-Bell 1998.
  - Chapter 47 "Position of the Moon" — primary algorithm source.
  - Table 47.A — 60 periodic terms for Σl (longitude) and Σr (distance).
  - Table 47.B — 60 periodic terms for Σb (latitude).
- Don Cross / CosineKitty `astronomy.c` — MIT-licensed cross-check
  implementation used during transcription.
**ADR**: ADR-013 ("Mean of 1969.5 ≈ J2000 for the Apollo window").

---

## 1. Purpose and Scope

`navigation::planetary` provides the **lunar ephemeris** for the Rust
port: Moon position and Moon inertial velocity relative to Earth, in
the AGC Mean of 1969.5 equatorial frame, as a function of Mission
Elapsed Time. It also provides the time-base conversion
`met_to_jd(epoch) -> Julian Day` that the lunar libration module
(`navigation::lunar_libration`) shares.

The algorithm is a **full 60-term Meeus Chapter 47 series** for Σl, Σr,
and Σb (the longitude, distance, and latitude perturbation sums), with
all standard Venus / Jupiter / `A_1, A_2, A_3` corrections. Accuracy
target: ~10 km position, ~1 m/s velocity, comfortably better than the
ADR-013 budget for the Apollo mission window.

The Rust port chose Meeus over the historical AGC's 9th-degree
polynomial because:

- The Meeus series is a well-documented, peer-reviewed source.
- The 60-term form is bounded and not extrapolative (the AGC polynomial
  was valid for a single mission and required a fresh PAD load for
  each).
- The implementation is one file with no PAD-load coupling.

The historical AGC's approach is documented in
`specs/lunar-ephemeris-research.md`; that note is kept as a reference
for future readers who want to understand the original implementation.

### What this module provides

- `APOLLO_11_LAUNCH_JD: f64 = 2440419.0639` — Julian Day at the
  hardcoded mission epoch (`Met(0)`).
- `MOON_MEAN_DISTANCE_KM: f64 = 385_000.56` — zero-point of the
  distance series, per Meeus eq. (47.1).
- `met_to_jd(epoch: Met) -> f64` — MET → Julian Day.
- `moon_position(epoch: Met) -> Vec3` — Moon position relative to Earth
  in MCI-equivalent metres.
- `moon_velocity(epoch: Met) -> Vec3` — Moon inertial velocity in
  metres / second (central-difference of `moon_position`, see §4.3).

### What this module does NOT provide

- **Sun position**. The original analyst research covered both
  `LUNPOS` and `SOLPOS`; the Rust port currently implements only the
  Moon. A `sun_position` will need to be added before any program that
  uses solar-attitude data (P52 star-occultation, P22 illumination
  gating). The research note (`lunar-ephemeris-research.md`) documents
  what the AGC's Sun computation did; that scope is deliberately
  deferred.
- A mission-epoch parameter. `APOLLO_11_LAUNCH_JD` is hard-coded. A
  future mission-support layer would take the epoch from a uplinked
  PAD load constant.
- Earth, planet, or asteroid ephemerides. Earth is the implicit origin
  of the Moon position vector; no other bodies are needed in
  Comanche055 scope.
- Higher-precision DE-series ephemerides. The Meeus 60-term form is
  ~10 km — adequate. A JPL DE440 wrapper would be a future option for
  high-fidelity reconstruction work but is not implemented.
- Light-time correction. The CSM is within ~1.3 light-seconds of the
  Moon at all times; navigation does not subtract the photon
  travel time.

---

## 2. Convention and Frame Note

### 2.1 Reference frame

The Meeus algorithm outputs **mean-of-date equatorial** coordinates.
For the Apollo mission window we treat this as identical to the AGC's
**Mean of 1969.5** equatorial frame. The precession difference between
the two is approximately:

```
50.3 arcsec/yr × 1 yr × 384 400 km · sin(50.3″) ≈ 30 km at lunar distance.
```

This is well within the module's `~10 km position` accuracy target and
the ADR-013 budget. Rust-port consumers of `moon_position` and
`moon_velocity` therefore treat the output as `Frame::MoonInertial`
without explicit precession correction.

### 2.2 Output units

- Position: **metres** (matches the crate-wide convention for
  positions).
- Velocity: **metres / second**.
- Distance constant `MOON_MEAN_DISTANCE_KM`: **kilometres** (this is
  the Meeus reference unit).

### 2.3 Hardcoded mission epoch

`APOLLO_11_LAUNCH_JD = 2440419.0639` — Apollo 11 launch (1969-07-16
13:32:00 UT). Derivation in the source: JD 2440222.5 = 1969-01-01
00:00 UT + 196 days + 13.5333 hours.

Any other mission (Apollo 8 demo, hypothetical re-launch) requires a
local override of this constant or a future epoch-parameterised API.

---

## 3. Rust API

### 3.1 Constants

```rust
pub const APOLLO_11_LAUNCH_JD: f64    = 2440419.0639;
pub const MOON_MEAN_DISTANCE_KM: f64  = 385_000.56;
```

Private constants:

- `DEG2RAD: f64 = π / 180`.
- `LR_TERMS: &[[f64; 6]]` — 60-row Meeus Table 47.A
  (each row `[D, M, M', F, Σl_amp, Σr_amp]`).
- `B_TERMS:  &[[f64; 5]]` — 60-row Meeus Table 47.B
  (each row `[D, M, M', F, Σb_amp]`).

### 3.2 Functions

```rust
pub fn met_to_jd(epoch: Met) -> f64;
pub fn moon_position(epoch: Met) -> Vec3;
pub fn moon_velocity(epoch: Met) -> Vec3;
```

A private helper `moon_position_at_jd(jd: f64) -> Vec3` factors the
inner series evaluator so that `moon_velocity` can sample at
`JD ± Δ` without round-tripping through the `u32`-backed `Met`
(which cannot represent times before `Met(0)`).

A private `norm_deg(deg: f64) -> f64` reduces an angle into `[0, 360)`
via `libm::fmod`.

---

## 4. Functional Requirements

### 4.1 `met_to_jd(epoch) -> f64`

```
JD = APOLLO_11_LAUNCH_JD + epoch.to_seconds() / 86400.0
```

Exact at `Met(0)`: `met_to_jd(Met::from_seconds(0.0)) ==
APOLLO_11_LAUNCH_JD` (no arithmetic, no rounding).

### 4.2 `moon_position(epoch) -> Vec3`

Composes `moon_position_at_jd(met_to_jd(epoch))`. The inner function
evaluates the Meeus Chapter 47 series:

| # | Step | Reference |
|---|---|---|
| 1 | `T_c = (JD − 2 451 545.0) / 36 525` (Julian centuries from J2000). | Meeus eq. 22.1 |
| 2 | Mean arguments `L', D, M, M', F` from polynomials in `T_c`, each `norm_deg` reduced. | Meeus pp. 338–339 |
| 3 | Eccentricity correction `E = 1 − 0.002516 · T_c − 7.4e−6 · T_c²`. | Meeus eq. 47.6 |
| 4 | Loop over the 60 rows of Table 47.A: `arg = a·D + b·M + c·M' + d·F`; multiply each amplitude by `E` (or `E²`) when `|M|` = 1 or 2. Accumulate `sigma_l` and `sigma_r`. | Meeus Table 47.A |
| 5 | Loop over the 60 rows of Table 47.B for `sigma_b`. | Meeus Table 47.B |
| 6 | Apply the Venus / Jupiter / `A_1, A_2, A_3` additive corrections to `sigma_l` and `sigma_b`. | Meeus p. 338 ("additional terms") |
| 7 | Geocentric ecliptic: `λ = L' + sigma_l / 1e6` (deg), `β = sigma_b / 1e6` (deg), `Δ = MOON_MEAN_DISTANCE_KM + sigma_r / 1000` (km). | Meeus eq. 47.1 |
| 8 | Mean obliquity `ε₀` from the polynomial in `T_c`. | Meeus p. 147 |
| 9 | Ecliptic → equatorial Cartesian: `x = Δ·cos β·cos λ`, `y = Δ·(cos β·sin λ·cos ε − sin β·sin ε)`, `z = Δ·(cos β·sin λ·sin ε + sin β·cos ε)`. | Meeus eq. 37.3 |
| 10 | Convert km → metres. | Crate-wide unit. |

Output is `[x_m, y_m, z_m]` as a `Vec3`.

**Postconditions** (verified by tests):

- For any MET in `[0, 27 days]`, `||r||` ∈ `[3.565e8, 4.067e8]` metres
  (perigee–apogee envelope).
- All three components are finite.

### 4.3 `moon_velocity(epoch) -> Vec3`

Central-difference of `moon_position_at_jd` in JD space (so the sample
at `Met(0) − 1 s` is representable):

```
H_SECONDS = 1.0
H_DAYS    = 1.0 / 86400.0
jd        = met_to_jd(epoch)
before    = moon_position_at_jd(jd − H_DAYS)
after     = moon_position_at_jd(jd + H_DAYS)
v[i]      = (after[i] − before[i]) / (2 · H_SECONDS)        for i = 0..3
```

**Error budget**: the lunar jerk is bounded by `v · ω² ≈ 10⁻⁸ m/s³`;
the truncation `(h²/6) · jerk ≈ 1.67e-9 m/s` is in the nanometre-per-second
regime. `f64` round-off in differencing positions of magnitude `~4·10⁸ m`
contributes at most `~10⁻⁷ m/s`. Both are well inside the 1 m/s
tolerance the function advertises against an outside-caller finite
difference.

---

## 5. Numerical Notes

- All trig and floating-point library calls go through `libm` for
  `no_std` determinism. The 120 sine / cosine calls per
  `moon_position` are individually cheap; the loop dominates.
- The eccentricity correction is applied row-by-row in the loop
  (factor `E`, `E²`, or `1.0` chosen by `|M_coef|`). Skipping it would
  introduce a ~10 km bias on the Moon position.
- The Apollo 11 launch epoch is hardcoded; any test that expects a
  specific Moon position must run against that epoch or override it.
- `norm_deg` uses `libm::fmod` with explicit wrap-up of negative
  remainders to `[0, 360)`. This matters because the polynomial
  arguments are sums of large positive and small negative numbers and
  can produce negative reduced values.

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `libm` (`sin`, `cos`, `fmod`, `fabs`) | All math; `no_std`. |
| `crate::types::{Met, Vec3}` | Time and 3-vector types. |
| `core::f64::consts::PI` | Angle scaling. |

No state. No I/O. No other `agc-core` module.

---

## 7. Module Layout

```
src/navigation/planetary.rs
├── pub const APOLLO_11_LAUNCH_JD: f64
├── pub const MOON_MEAN_DISTANCE_KM: f64
├── const DEG2RAD: f64
├── const LR_TERMS: &[[f64; 6]]              (60 rows, Meeus Table 47.A)
├── const B_TERMS:  &[[f64; 5]]              (60 rows, Meeus Table 47.B)
├── pub fn met_to_jd(epoch: Met) -> f64
├── pub fn moon_position(epoch: Met) -> Vec3
├── pub fn moon_velocity(epoch: Met) -> Vec3
├── fn moon_position_at_jd(jd: f64) -> Vec3   (private)
├── fn norm_deg(deg: f64) -> f64              (private, #[inline])
└── #[cfg(test)] mod tests
```

---

## 8. Test Cases

The implementation in `agc-core/src/navigation/planetary.rs::tests`
provides:

| ID | What is verified |
|---|---|
| `tc_moon_1_met_to_jd_zero` | `met_to_jd(Met::from_seconds(0.0)) == APOLLO_11_LAUNCH_JD` bit-exactly. |
| `tc_moon_2_met_to_jd_one_day` | `met_to_jd(Met::from_seconds(86400.0)) ≈ APOLLO_11_LAUNCH_JD + 1.0` within `1e-12`. |
| `tc_moon_3_launch_distance_in_range` | `||moon_position(0)||` falls in `[3.565e8, 4.067e8]` m (lunar perigee–apogee envelope). |
| `tc_moon_4_distance_bounded_over_mission` | Across MET 0, 1, 4, 8, 15 days, the distance stays within `380 000 km ± 30 000 km` — proves the series does not diverge over the mission window. |
| `tc_moon_5_finite_and_nonzero` | `moon_position(0)` is finite per component and `||·|| > 1e8 m` — guards against NaN / zero-vector regressions. |
| `tc_moon_6_launch_position_approximate` | Cross-check against an independent hand calculation: sign pattern `(x < 0, y > 0, z > 0)` and `~(2.91e8, 2.29e8, 0.99e8) m` within `1e8 m` per component. This is the only independent frame / sign cross-check on `moon_position`; the spec calls out that **the sign assertions must not be weakened** because they are the only frame correctness gate. |
| `tc_moon_7_one_hour_displacement` | Moon moves `[1000, 10 000] km` in one hour (orbital speed ≈ 1.02 km/s). |
| `tc_moon_8_sidereal_period_cyclicity` | After one sidereal month (27.3 days) the Moon returns to within `50 000 km` of its starting position — verifies the series is periodic and bounded. |
| `tc_moon_9_velocity_matches_central_diff` | `moon_velocity` agrees with a coarser (60 s) reference central-difference within 1 m/s at MET = 1, 4, 8, 15 days. |
| `tc_moon_10_velocity_magnitude_in_range` | `||moon_velocity||` lies in `[900, 1100] m/s` across the mission window — pins units, sign, and frame. |

---

## 9. Spec Quality Checklist

- [x] Meeus (1998) Chapter 47 source cited, with Tables 47.A and 47.B
      identified.
- [x] Cross-check implementation (Don Cross / CosineKitty `astronomy.c`)
      named.
- [x] Frame approximation (Meeus mean-of-date ≈ Mean of 1969.5 ≈ J2000)
      explicitly justified against the ADR-013 budget (§2.1).
- [x] All four public symbols (two constants, three functions)
      specified.
- [x] Algorithm steps spelled out with Meeus equation references
      (§4.2 table).
- [x] Velocity error budget documented (§4.3).
- [x] Hardcoded mission epoch surfaced as a known limitation (§1, §2.3).
- [x] Sun-position scope deferral surfaced as a known limitation (§1).
- [x] Cross-reference to the analyst research note that documents the
      historical AGC's 9th-degree polynomial (§Reference).
- [x] Dependencies listed (§6).
- [x] Test coverage summarised (§8).
