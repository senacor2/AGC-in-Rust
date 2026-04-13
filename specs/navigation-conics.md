# Functional Specification: Orbital Conics and Elements (`agc-core/src/navigation/conics`)

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc
Routines:   APSIDES   (periapsis/apoapsis radii and eccentricity, pages 1275-1277 / 1303-1304)
            GETECC    (eccentricity computation shared with APSIDES, page 1303)
            PARAM     (P, R1A, COGA — orbital parameter kernel, pages 1289-1290)
            GEOM      (unit vectors, sin/cos transfer angle, page 1291)
            TIMERAD   (radius-to-time with COSF computation, pages 1301-1303)
            KEPLERN   (vis-viva derived from KEPC1/KEPC2 inputs, pages 1277-1278)
Pages:      1275-1277 (APSIDES description), 1289-1292 (PARAM / GEOM),
            1301-1304 (TIMERAD / APSIDES implementation)

AGC source: Comanche055/P30-P37.agc
Routines:   S30.1 (displays HAPO/HPER from PERIAPO1 call, page 639-640)
            PARAM30 (displays apogee/perigee on DSKY via N42, page 637-638)
Pages:      637-643

AGC source: Comanche055/POWERED_FLIGHT_SUBROUTINES.agc
Routines:   PERIAPO1  (called by S30.1 and S31.1 to compute HAPO/HPER)
            Indirectly sets HAPO and HPER erasables for DSKY display
Pages:      1365-1372
```

Secondary references:
- `docs/agc-reference-constants.md` — MU_EARTH, MU_MOON, RE_EARTH
- `agc-core/src/math/linalg.rs` — `dot`, `cross`, `norm`, `norm_sq`, `scale`, `add`, `sub`, `unit`
- `agc-core/src/types/vector.rs` — `Vec3 = [f64; 3]`
- `agc-core/src/navigation/state_vector.rs` — `StateVector`

---

## 1. Behavior Summary

The AGC's CONIC_SUBROUTINES.agc contains several routines that compute classical orbital elements and derived quantities (apsides, eccentricity, orbital period) from an instantaneous state vector. These are not standalone "element conversion" routines in the traditional sense — the AGC computes them inline within the flow of APSIDES, PARAM, and TIMERAD — but the Rust port extracts them as named pure functions in `navigation::conics`.

The principal AGC routines and what they compute:

| AGC routine | Computes | Used in |
|---|---|---|
| PARAM | P (semi-latus rectum ratio), R1A (r/a), COGA (cot γ) | LAMBERT, TIMERAD, APSIDES, TIMETHET |
| GETECC / APSIDES | ECC, periapsis radius, apoapsis radius | P30 display (PARAM30), S30.1 via PERIAPO1 |
| TIMERAD | Computes COSF (true anomaly cosine), then arc-length | TIMETHET, orbital integration |
| KEPLERN init | ALPHA = 1/a (from energy), C1 (r·v/√μ), C2 (v²r/μ - 1) | Kepler propagator |

The `navigation::conics` Rust module makes these quantities available as pure functions on `(r: &Vec3, v: &Vec3, mu: f64)` inputs, returning classical orbital elements or scalar derived quantities. None of these functions perform iteration or modify state.

### 1.1 Orbital Elements from State Vector

Given position r and velocity v, the standard orbital element derivation maps directly to AGC PARAM + GETECC quantities:

```
h = r × v                       (specific angular momentum vector; AGC: VXV in GEOM)
e_vec = v × h / μ - r / |r|    (eccentricity vector; derived from AGC TIMERAD COSF computation)
a = -μ / (2 * ε)                (semi-major axis; from ALPHA = 1/a in KEPLERN)
  where ε = |v|²/2 - μ/|r|

e = |e_vec|                     (eccentricity; AGC: ECC at scale +3 in APSIDES/GETECC)
i = acos(h[2] / |h|)            (inclination; from h direction)
Ω = atan2(h[0], -h[1])          (RAAN; AGC does not use Ω directly — computed here for Rust)
ω = atan2(...)                  (argument of periapsis; not in AGC — computed here for Rust)
ν = atan2(...)                  (true anomaly; AGC computes COSF = cos ν in TIMERAD)
```

The AGC itself does not compute i, Ω, ω, ν as named outputs; it works with r, v, P, COGA, R1A, and UN (orbit plane normal). The Rust `OrbitalElements` struct assembles the full classical element set for the benefit of the agc-sim Mission State display panel and for use by other guidance modules.

### 1.2 APSIDES Routine (AGC pages 1303-1304, description pages 1275-1276)

APSIDES calls PARAM and GEOM, then calls `GETECC`:

```
ECC = √(1 - R1A * (D1/64))     (AGC: BDSU D1/64, then SQRT; D1/64 = P/r1 = p/r1)
    = √(1 - p/a)               (geometric identity)
    = √(R1A² - P * R1A + D1/64) (AGC formulation via GETECC, page 1303)
```

Exact AGC GETECC code (page 1303):
```
GETECC    DMP    SL4
              R1A
          BDSU   SQRT
              D1/64
          STORE  ECC
```

This computes `ECC = √(1/64 - R1A * R1A/64) * ... ` — actually the AGC code is:
- MPAC = R1A (ratio r1/a, scale +6)
- DMP SL4: square R1A, shift; result ≈ (r1/a)² in appropriate scale
- BDSU D1/64: subtract from 1/64
- SQRT: take square root

The mathematical interpretation: `ECC = √(1 - (1 - e²))... ` which simplifies to the standard `e = √(1 - p/a)` for elliptic orbits via `R1A = r1/a` and `P = p/r1`:
```
e = √(1 - P * R1A)    (valid for e < 1)
```

For hyperbolic orbits (R1A < 0), ECC > 1 and the AGC SQRT still yields the correct eccentricity.

**Periapsis radius** (page 1303-1304):
```
r_per = r1 * P / (1 + ECC)    # using AGC DMP SL1 / DDV
```

**Apoapsis radius** (page 1303-1304):
```
r_apo = r1 * P / (1 - ECC)    # using DAD / DDV with D1/8 = 1+ECC form
```

If apoapsis overflows (hyperbolic orbit, ECC ≥ 1), INFINAPO returns `LDPOSMAX` (the maximum representable position value, approximately 536,870,910 m for Earth). In Rust, the function returns `f64::INFINITY` for the apoapsis of hyperbolic/parabolic orbits.

**P30 DSKY display**: S30.1 (P30-P37.agc, page 639-640) calls `PERIAPO1` which invokes APSIDES to populate `HAPO` (apogee altitude) and `HPER` (perigee altitude) at scale B+29 m. These are displayed via Noun N42 (Apogee/Perigee altitudes). The Rust equivalent is `apoapsis_periapsis` called by the guidance layer and fed into the `MissionState` panel.

### 1.3 Vis-Viva Equation

The vis-viva equation gives orbital speed at radius r on an orbit with semi-major axis a:

```
v = √(μ * (2/r - 1/a))
```

This is not an explicit AGC subroutine name, but it underlies KEPLERN's initialization (`KEPC2 = r·v²/μ - 1`) and INITV's velocity construction (`ROOTMU * √(P/r1)`). In GETX (page 1292), vis-viva is implicit in the W-loop computation.

In Rust this is a trivial helper used by conics helpers and unit tests.

### 1.4 Period from Semi-Major Axis

Orbital period for elliptic orbits:

```
T = 2π * √(a³/μ)
```

The AGC computes this implicitly in KEPLERN's modulo reduction (PERIODCH loop, page 1278):
```
PERIOD = 2π / √μ * √(a³) = 2PISC * √(1/ALPHA³ * something)
```

Specifically, KEPLERN computes `PERIOD = (2π/√μ) * SQRT(1/ALPHA)^3` via:
- SQRT of `2PISC / ALPHA`: `2PISC 2DEC 6.28318530 B-6` (page 1288)
- `ALPHA 1REV SQRT BDDV 2PISC` sequence (pages 1277-1278)

For the Rust function, the formula is direct. For hyperbolic orbits (a < 0), `period` returns `f64::INFINITY`.

---

## 2. Rust API

**Module path**: `agc_core::navigation::conics`

```rust
/// Classical Keplerian orbital elements computed from a state vector.
///
/// All angles in radians. Distances in metres. Dimensionless eccentricity.
///
/// For rectilinear (zero angular momentum) trajectories, use `elements_from_state`
/// returns `None`. For hyperbolic orbits, `sma < 0` and `ecc > 1`.
///
/// AGC source: elements derived from PARAM, GETECC, and GEOM routines in
/// `Comanche055/CONIC_SUBROUTINES.agc` (pages 1289-1292, 1303).
pub struct OrbitalElements {
    /// Semi-major axis in metres. Negative for hyperbolic orbits.
    pub sma: f64,
    /// Eccentricity. 0 = circular, 0 < e < 1 = elliptic, e = 1 parabolic, e > 1 hyperbolic.
    pub ecc: f64,
    /// Inclination in radians, [0, π].
    pub inc: f64,
    /// Right ascension of ascending node (RAAN) in radians, [0, 2π).
    pub raan: f64,
    /// Argument of periapsis in radians, [0, 2π).
    pub argp: f64,
    /// True anomaly in radians, [0, 2π).
    pub true_anom: f64,
}

/// Compute classical orbital elements from a Cartesian state vector.
///
/// Returns `None` for degenerate cases:
/// - Zero or near-zero angular momentum (`|h| < 1e-6 m²/s`) — rectilinear trajectory.
/// - Zero or near-zero radius (`|r| < 1.0 m`) — singularity.
/// - Zero or negative `mu`.
///
/// For parabolic orbits (ε ≈ 0), `sma` will be very large (positive). The function
/// does not special-case parabolic; it returns a valid struct with large `sma`.
///
/// # Scale factors
/// - Input position: metres.
/// - Input velocity: m/s.
/// - Input mu: m³/s².
/// - Output angles: radians.
/// - Output distances (sma): metres.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, PARAM (page 1289), GETECC (page 1303),
///             GEOM (page 1291). Inclination / RAAN / argp are not directly in the AGC
///             (which uses the orbit-plane normal UN instead); they are computed here from h.
pub fn elements_from_state(r: &Vec3, v: &Vec3, mu: f64) -> Option<OrbitalElements> { ... }

/// Orbital speed from vis-viva equation: v = √(μ*(2/r - 1/a)).
///
/// # Arguments
/// - `r`: current radius in metres. Must be > 0.
/// - `a`: semi-major axis in metres. Negative for hyperbolic.
/// - `mu`: gravitational parameter in m³/s².
///
/// Returns 0.0 if `r <= 0.0` or `mu <= 0.0` (rather than NaN/panic).
/// Returns `f64::NAN` if the expression under the sqrt is negative (unphysical inputs).
///
/// AGC source: Implicit in `Comanche055/CONIC_SUBROUTINES.agc`, INITV velocity construction
///             (page 1299) and KEPLERN initialization (KEPC2 = r*v²/μ - 1, page 1277).
pub fn vis_viva(r: f64, a: f64, mu: f64) -> f64 { ... }

/// Apoapsis and periapsis radii from orbital elements.
///
/// Returns `(apoapsis_m, periapsis_m)` where:
/// - `apoapsis_m = a * (1 + e)` — `f64::INFINITY` for hyperbolic / parabolic (e ≥ 1)
/// - `periapsis_m = a * (1 - e)`
///
/// # Invariants
/// - Never panics.
/// - If `elements.ecc >= 1.0`, apoapsis is `f64::INFINITY`.
///   Corresponds to AGC INFINAPO branch returning LDPOSMAX (page 1303).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, APSIDES routine (page 1303-1304).
/// P30 display: `HAPO` / `HPER` erasables, displayed via N42 in PARAM30 (P30-P37.agc, page 637).
pub fn apoapsis_periapsis(elements: &OrbitalElements) -> (f64, f64) { ... }

/// Orbital period from semi-major axis.
///
/// `T = 2π * √(a³ / μ)` for elliptic orbits (a > 0).
///
/// Returns `f64::INFINITY` for parabolic/hyperbolic orbits (a ≤ 0).
/// Returns `0.0` if `mu <= 0.0`.
///
/// # AGC source
/// Implicit in `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN modulo reduction
/// (PERIODCH loop, page 1278): `SQRT 2PISC / ALPHA` chain, where
/// `2PISC = 2π * 2^-6` and `ALPHA = 1/a`.
pub fn period(a: f64, mu: f64) -> f64 { ... }
```

---

## 3. Constants

The following constants are used in the conics module and must be drawn from `docs/agc-reference-constants.md`:

| Constant | Value | Unit | AGC source |
|---|---|---|---|
| `MU_EARTH` | `3.986_032e14` | m³/s² | CONIC_SUBROUTINES.agc MUTABLE table (page 1305) |
| `MU_MOON` | `4.902_778e12` | m³/s² | CONIC_SUBROUTINES.agc MUTABLE table (page 1305) |
| `RE_EARTH` | `6_373_338.0` | m | ORBITAL_INTEGRATION.agc `RSPHERE`, confirmed in `docs/agc-reference-constants.md` |
| `COGUPLIM` | `0.999_511_597` | — | CONIC_SUBROUTINES.agc page 1288 — eccentricity near 1 limit (APSIDES/TIMERAD) |
| `MIN_RADIUS` | `1.0` | m | Singularity guard (no AGC source; safety invariant) |
| `MIN_H_SQ` | `1e-12` | m⁴/s² | Zero-angular-momentum guard (~= h < 1e-6 m²/s) |

Note on altitude vs radius: The AGC stores apogee/perigee as **altitudes** (`HAPO` = apoapsis altitude above Earth surface = `r_apo - RE_EARTH`). The Rust functions return and accept **radii from the central body centre**. The caller (guidance display layer) subtracts `RE_EARTH` to obtain altitude for DSKY display.

---

## 4. Scale Factors (AGC → Rust)

| AGC variable | AGC scale (Earth) | Rust unit | Notes |
|---|---|---|---|
| `RVEC` | B+29 m | metres (f64) | Position input |
| `VVEC` | B+7 m/cs | m/s (f64) | Velocity input (multiply AGC by 100) |
| `ECC` | B+3 (range 0-8) | dimensionless | Eccentricity; AGC max ~3, Rust uses f64 range |
| `R1A` | B+6 (ratio r1/a) | dimensionless | Negative for hyperbolic |
| `P` | B+4 (ratio p/r1) | dimensionless | Semi-latus rectum / r1 |
| `HAPO` / `HPER` | B+29 m | metres | Output of APSIDES for DSKY |
| `ALPHA` | B-22 (Earth) | 1/m | Reciprocal SMA in KEPLERN |

The Rust API takes and returns physical SI quantities (metres, m/s, m³/s²) with no scale-factor arithmetic in the caller.

---

## 5. Invariants

1. **No heap**: `OrbitalElements` is a plain flat struct; no `Vec`, `Box`, or dynamic memory.
2. **No `unwrap`**: all intermediate `Option` returns from `unit()` map to `Option::None` propagation.
3. **No panic**: any invalid input combination returns `None` (for `elements_from_state`) or a sentinel value (`f64::INFINITY`, `0.0`, `f64::NAN`). No `unwrap`, `expect`, or `panic!`.
4. **Finite inputs in, finite or documented-special outputs out**: `f64::NAN` output is only permitted by `vis_viva` when the radicand is negative (physically impossible orbit), and must be documented.
5. **Hyperbolic support**: `elements_from_state` works correctly for e > 1 (sma < 0). `apoapsis_periapsis` returns `f64::INFINITY` for the apoapsis.
6. **Inclination domain**: `inc` is in `[0, π]`; `raan` and `argp` are in `[0, 2π)`. Computed via `atan2` without domain restrictions on intermediate cross products.
7. **Zero-eccentricity edge cases**: for circular orbits (e ≈ 0), `argp` and `true_anom` are undefined by classical mechanics. The implementation returns `0.0` for both, matching the AGC convention of not computing these for near-circular orbits.
8. **Equatorial edge cases**: for equatorial orbits (i ≈ 0 or i ≈ π), `raan` is undefined. The implementation returns `0.0`, matching the AGC convention.

---

## 6. Test Cases

### TC-C-1: Circular LEO orbit (ecc ≈ 0, inc = 0°)

```
mu = 3.986_032e14
r  = 6_571_000.0     (m, 200 km altitude)
v_circ = √(mu / r) ≈ 7_784.26   (m/s)

r_vec = [r, 0.0, 0.0]
v_vec = [0.0, v_circ, 0.0]      (purely prograde, equatorial)

result = elements_from_state(&r_vec, &v_vec, mu)

Expected:
  result = Some(...)
  sma ≈ 6_571_000.0 m  (within 1 m)
  ecc ≈ 0.0             (within 1e-6)
  inc ≈ 0.0             (within 1e-6 radians)
  period(sma, mu) ≈ 5_405.0 s  (within 1 s)
  apoapsis_periapsis: both ≈ r (within 10 m)
  vis_viva(r, sma, mu) ≈ v_circ (within 0.01 m/s)
```

### TC-C-2: Geostationary Transfer Orbit (elliptic, ecc ≈ 0.73)

```
mu     = 3.986_032e14
r_per  = 6_556_370.0   (185 km altitude)
r_apo  = 42_164_170.0  (GEO radius)
a      = (r_per + r_apo) / 2 = 24_360_270.0
e      = (r_apo - r_per) / (r_apo + r_per) ≈ 0.7286

v_per  = √(mu * (2/r_per - 1/a)) ≈ 10_239.0 m/s

r_vec  = [r_per, 0.0, 0.0]
v_vec  = [0.0, v_per, 0.0]

result = elements_from_state(&r_vec, &v_vec, mu)

Expected:
  result = Some(...)
  sma ≈ 24_360_270.0 m  (within 1000 m)
  ecc ≈ 0.7286           (within 0.001)
  inc ≈ 0.0              (equatorial, within 1e-6 rad)
  (apo, per) = apoapsis_periapsis(&result)
  apo ≈ 42_164_170.0 m  (within 1000 m)
  per ≈  6_556_370.0 m  (within 1000 m)
```

### TC-C-3: ISS-like inclined orbit (i ≈ 51.6°)

```
mu = 3.986_032e14
a  = 6_778_000.0    (400 km altitude circular)
inc = 51.6° = 0.9005 radians

# Construct circular state vector at inclination
v_circ = √(mu / a) ≈ 7_668.0 m/s
r_vec = [a, 0.0, 0.0]
# Velocity in orbit plane tilted by inc from equatorial:
v_vec = [0.0, v_circ * cos(inc), v_circ * sin(inc)]
      ≈ [0.0, 4_780.0, 6_006.0]

result = elements_from_state(&r_vec, &v_vec, mu)

Expected:
  result = Some(...)
  sma ≈ 6_778_000.0  (within 100 m)
  ecc ≈ 0.0           (within 1e-4)
  inc ≈ 0.9005 rad   (within 0.001 rad ≈ 0.06°)
```

### TC-C-4: Vis-viva sanity check at both apsides

```
mu    = 3.986_032e14
r_per = 6_671_000.0
r_apo = 42_164_000.0
a     = (r_per + r_apo) / 2.0

v_at_per = vis_viva(r_per, a, mu)
v_at_apo = vis_viva(r_apo, a, mu)

Expected:
  v_at_per ≈ 10_218.0 m/s  (within 1 m/s)
  v_at_apo ≈  1_601.0 m/s  (within 1 m/s)

# Angular momentum conservation:
  v_at_per * r_per ≈ v_at_apo * r_apo  (within 1000 m²/s)

# Degenerate cases:
  vis_viva(0.0, a, mu) = 0.0   (zero radius guard)
  vis_viva(r_per, a, 0.0) = 0.0  (zero mu guard)
```

---

## 7. agc-sim Impact

- **MissionState panel fields**: the agc-sim Mission State panel displays `SMA`, `ECC`, `APO`, `PER`. These are populated by calling `elements_from_state` on the current state vector each display cycle, then `apoapsis_periapsis` to obtain the altitude values (subtracting `RE_EARTH`).
  - `sma_km`: `elements.sma / 1000.0`
  - `ecc`: `elements.ecc`
  - `apo_km`: `(apo_radius - RE_EARTH) / 1000.0`
  - `per_km`: `(per_radius - RE_EARTH) / 1000.0`
- **DskyDisplayState**: no new fields needed; the Mission State panel already has `sma`, `ecc`, `apo`, `per` placeholder fields from the M1/M2 TUI layout. The developer must wire the `conics` function outputs into these fields.
- **SimLog**: no new log lines required; orbital element computation is silent unless `elements_from_state` returns `None`, in which case emit `warn!("conics: degenerate state vector — orbital elements undefined")`.
- **Noun N42** (Apogee/Perigee display in P30): when Verb 06 Noun 42 is entered in agc-sim, the display layer calls `apoapsis_periapsis` and converts to altitude in metres for the R1/R2 registers. This requires wiring the conics module into the pinball (DSKY verb/noun handler) layer.
- **No new keyboard bindings** needed.
