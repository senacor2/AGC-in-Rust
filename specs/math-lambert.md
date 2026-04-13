# Functional Specification: Lambert Targeting (`agc-core/src/math/lambert`)

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc
Routines:   LAMBERT   (main entry, pages 1296-1300)
            LAMBLOOP  (primary iteration, pages 1297-1299)
            INITV     (compute VVEC from converged COGA, pages 1299-1300)
            TARGETV   (optional terminal-velocity computation via LAMENTER, page 1300)
            ITERATOR  (bisection/Newton search for independent variable, pages 1285-1286)
            GEOM      (geometry setup: sin/cos of transfer angle, unit vectors, page 1291)
            PARAM     (orbital parameter setup: P, COGA, R1A, page 1289-1290)
            GETX      (compute X from P and orbital parameters, pages 1292-1295)
            DELTIME   (Stumpff functions, same as Kepler, pages 1283-1284)
            NEWSTATE  (propagate state from X/COGA solution, pages 1287-1288)
            CHECKCTR  (iteration counter guard, page 1286)
Pages:      1296-1300 (LAMBERT main), 1285-1286 (ITERATOR), 1287-1295 (supporting),
            1305-1308 (erasable assignments)
```

Usage context:
```
AGC source: Comanche055/P30-P37.agc
Routines:   S31.1 (Lambert-based aimpoint guidance, page 641)
            AGAIN (state propagation calling Lambert indirectly)
Pages:      639-643
```

Secondary references:
- `docs/agc-reference-constants.md` — MU_EARTH, MU_MOON
- `agc-core/src/math/kepler.rs` — DELTIME functions (shared Stumpff logic)
- `agc-core/src/math/linalg.rs` — `dot`, `cross`, `norm`, `unit`, `scale`, `add`, `sub`
- `agc-core/src/types/vector.rs` — `Vec3 = [f64; 3]`

---

## 1. Behavior Summary

The LAMBERT subroutine solves the two-point boundary-value problem: given positions r1 and r2 and a transfer time dt, find the initial velocity v1 (and optionally the terminal velocity v2) of the unique conic arc connecting them. This is the core computation for the AGC's return-to-Earth targeting (P30/P31/P37 programs) and for any burn that must arrive at a specific point at a specific time.

The AGC's formulation uses the cotangent of the flight-path angle (COGA = cot γ, the cotangent of the angle between the position vector and the velocity vector) as the independent iteration variable, rather than the semi-latus rectum p or Lagrange's λ. This is the Gauss/Battin cotangent formulation.

### 1.1 Geometric Setup (GEOM routine)

Before iteration, LAMBERT calls GEOM to compute the geometry of the transfer:

1. Unit vectors: `UR1 = r1/|r1|`, `U2 = r2/|r2|`
2. Transfer angle geometry:
   - `CSTH = UR1 · U2` (cosine of transfer angle θ)
   - `SNTH = |UR1 × U2|` (sine of transfer angle; magnitude of cross product)
   - `GEOMSGN` determines short-way (θ < 180°, GEOMSGN = +0.5) vs long-way (θ > 180°, GEOMSGN = -0.5)
3. Orbit-plane unit normal: `UN = normalize(UR1 × U2)` signed by GEOMSGN (or supplied by caller if NORMSW set)
4. `1 - CSTH` (AGC: `1-CSTH`, scale +2): used in P computation

The short-way/long-way sense is controlled by `GEOMSGN`, a single-precision tag set to +0.5 or -0.5 by the caller before invoking LAMBERT. In the Rust API this is the `TransferDirection` enum.

**Degenerate case (COLINEAR)**: If `|UR1 × U2|` < machine epsilon (collinear r1 and r2 — either exactly 0° or 180° apart), GEOM falls through to the `COLINEAR` label which VSR1 (right-shifts) the near-zero result and retries UNITNORM. In Rust, collinear inputs (including exact 0° or 360° transfers) result in `converged: false`; see also `360LAMB` label in LAMBERT (page 1300, triggered when 1-CSTH rounds to zero).

### 1.2 Orbital Parameter Setup (PARAM routine)

PARAM computes, from (r1, v_initial_guess), the orbit parameters used throughout the iteration:
- `R1 = |r1|` in metres
- `COGA = cos(γ)/sin(γ)` where γ is the flight-path angle (angle between r and v, from local vertical)
- `R1A = r1/a = r1·v²/μ - 1` (ratio of radius to SMA; negative for hyperbolic)
- `P = R1 * sin²(γ)` ... actually `P = (SIN γ)² * R1` — AGC scale +4 means ratio of semi-latus rectum to r1

### 1.3 Iteration Variable and Main Loop (LAMBLOOP)

The iteration solves for COGA (cotangent of flight-path angle at r1) such that the transfer time from r1 to r2 matches TDESIRED.

**Bounds on COGA**:
- `COGAMAX` (upper bound): computed from geometry — `√(2r1/(r2(1-cosθ))) + snth/(1-cosθ)` — then clamped to COGUPLIM = 0.999511597 to prevent overflow in R1A calculation (page 1297, `COGUPLIM` constant, page 1288-1289).
- `COGAMIN` (lower bound): computed from geometry, clamped to COGLOLIM = -0.999511597.

**Per-iteration steps** (`LAMBLOOP`, page 1297):
1. Compute `P = (1 - CSTH) / (SNTH * COGA - (CSTH - ρ))` where `ρ = r1/r2` (AGC: `CSTH-RHO` erasable)
   - If P ≤ 0: trajectory is physically impossible; call HIENERGY or NEGP branch
2. Compute `R1A = 2 - P * (1 + COGA²)` (ratio r1/a; negative = hyperbolic)
3. Call GETX to compute universal variable X from P and R1A
4. Call DELTIME to compute transfer time T at X
5. Check `|T - TDESIRED| < EPSILONL` (tolerance `TDESIRED * 2^-19`, `BEE19` constant)
   - If converged: call INITV then exit
   - If not: call ITERATOR to compute new COGA (bisection bounded by [COGAMIN, COGAMAX])
6. ITERATOR adjusts COGA by a Newton-like step; if step exceeds bounds, bisects within bracket (MODNGDEL / MODPSDEL logic, pages 1285-1286). Clamps with factor DP9/10 = 0.9 when near a bound.
7. Iteration limit: ITERCTR initialized to 20D (`SSP ITERCTR 20D`, page 1296). CHECKCTR decrements each call. If ITERCTR reaches zero, BHIZ branches to `SUFFCHEK`.

**SUFFCHEK** (page 1299): if at iteration limit, checks whether `|TERRLAMB| < TDESIRED/4 + ONEBIT` (i.e. error is less than 25% plus one bit). If so, proceeds to INITV. Otherwise sets SOLNSW (failure flag) and returns.

**High-energy / Low-energy branches**:
- `HIENERGY` (page 1298): P overflows or R1A overflows or GETX yields XI > 50. COGA → new COGAMIN; halve DCOGA and retry.
- `LOENERGY` (page 1299): time T overflows (BIGTIME). COGA → new COGAMAX; halve DCOGA and retry.
- `NEGP` (page 1298): P ≤ 0 (impossible trajectory); checks DCOGA sign and bounces to HIENERGY or LOENERGY.

### 1.4 Velocity Construction (INITV)

Once COGA is converged, INITV (page 1299-1300) constructs VVEC (velocity at r1):

```
V_tan = √(μ * P / r1)         (tangential speed component)
V_rad = V_tan * COGA           (radial speed component = V_tan * cot(γ))

VVEC = V_rad * UR1 + V_tan * (UN × UR1)
```

AGC implementation: `ROOTMU * √(P/r1)` for V_tan (SL4 / DMP scaling), then `VXSC UR1` for radial and `VXV / VAD` for tangential via UN.

### 1.5 Terminal Velocity (TARGETV / LAMENTER)

If `VTARGTAG = 0` (caller requests terminal velocity), LAMBERT calls TARGETV which calls LAMENTER (shared with NEWSTATE, page 1287-1288) to propagate from X to get the state at r2. This yields VTARGET = v2.

In the Rust API, v2 is always computed (the AGC's VTARGTAG optimisation is eliminated for clarity).

### 1.6 Lambert Restrictions (from AGC comments, page 1266-1267)

1. Rectilinear trajectories (r1 parallel to r2) cannot be computed. Rust: `converged: false`.
2. Accuracy degrades as cos(θ) → +1.0 (near-zero transfer angle). Rust: if `1 - CSTH < f64::EPSILON`, return `converged: false`.
3. Flight-path angle γ must satisfy 1°47.5' < γ < 178°12.5' (i.e. COGA must be within [COGLOLIM, COGUPLIM]).
4. Negative transfer time is ambiguous. Rust: if `dt <= 0.0`, return `converged: false`.
5. Parameters must not exceed scaling limits. Rust: if |r1| < 1.0 m or |r2| < 1.0 m, return `converged: false`.

---

## 2. Rust API

**Module path**: `agc_core::math::lambert`

```rust
/// Direction of orbital transfer (controls orbit-plane orientation).
///
/// Short = transfer angle θ < 180° (GEOMSGN = +0.5 in AGC).
/// Long  = transfer angle θ > 180° (GEOMSGN = -0.5 in AGC).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT input GEOMSGN (page 1267).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TransferDirection {
    Short,
    Long,
}

/// Result of a Lambert boundary-value solve.
///
/// If `converged` is false, `v1` and `v2` are meaningless and must not be used.
/// The caller must invoke `alarm::raise` and return to the guidance idle state.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT output VVEC / VTARGET (page 1268).
pub struct LambertResult {
    /// Initial velocity at r1, ECI m/s.
    pub v1: Vec3,
    /// Terminal velocity at r2, ECI m/s.
    pub v2: Vec3,
    /// True if iteration converged within MAX_ITERATIONS.
    pub converged: bool,
    /// Number of COGA iterations performed.
    pub iterations: u32,
}

/// Solve Lambert's problem: find the velocity at r1 that takes a spacecraft
/// from r1 to r2 in time dt along a conic arc.
///
/// # Arguments
/// - `r1`: initial position, ECI metres.
/// - `r2`: target position, ECI metres.
/// - `dt`: transfer time in seconds. Must be > 0.0; negative returns `converged: false`.
/// - `mu`: gravitational parameter in m³/s².
/// - `dir`: short-way (θ < 180°) or long-way (θ > 180°) transfer.
///
/// # Returns
/// `LambertResult`. Never panics. Degenerate inputs (collinear r1/r2, zero dt,
/// zero radius) return `converged: false`.
///
/// # Invariants
/// - No heap allocation; no `unwrap`; no panic.
/// - Bounded to MAX_ITERATIONS COGA iterations.
/// - v2 is always computed (no VTARGTAG optimisation).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT (page 1296);
///             INITV (page 1299); TARGETV / LAMENTER (page 1300 / 1287).
pub fn lambert(r1: &Vec3, r2: &Vec3, dt: f64, mu: f64, dir: TransferDirection) -> LambertResult { ... }
```

---

## 3. Constants

| Constant | Value | Source | Notes |
|---|---|---|---|
| `MAX_ITERATIONS` | `20` | `CONIC_SUBROUTINES.agc` `SSP ITERCTR 20D` (page 1296) | Same as Kepler |
| `LAMBERT_TOL_FACTOR` | `2.0f64.powi(-19)` | AGC `BEE19` constant (page 1288): `D1/32 -1` = 2^-19 | Multiply by |dt| to get time tolerance EPSILONL |
| `COGUPLIM` | `0.999_511_597` | `CONIC_SUBROUTINES.agc`, `COGUPLIM 2DEC .999511597` (page 1288) | Upper bound on COGA to prevent R1A overflow |
| `COGLOLIM` | `-0.999_511_597` | `CONIC_SUBROUTINES.agc`, `COGLOLIM 2DEC -.999511597` (page 1288) | Lower bound on COGA |
| `BISECT_FACTOR` | `0.9` | `CONIC_SUBROUTINES.agc`, `DP9/10 2DEC .9` (page 1289) | Used in MODNGDEL/MODPSDEL to scale step when near a bound |
| `MU_EARTH` | `3.986_032e14` | Same as Kepler | |
| `MU_MOON` | `4.902_778e12` | Same as Kepler | |

---

## 4. Scale Factors (AGC → Rust)

| AGC variable | AGC scale (Earth) | Rust unit | Notes |
|---|---|---|---|
| `R1VEC` / `R2VEC` | B+29 metres | metres (f64) | Position inputs |
| `TDESIRED` | B+28 centiseconds | seconds (f64) | Transfer time |
| `VVEC` (v1 output) | B+7 m/cs | m/s (f64) | Multiply AGC value × 100 |
| `VTARGET` (v2 output) | B+7 m/cs | m/s (f64) | |
| `COGA` | B+5 | dimensionless | Cotangent of flight-path angle |
| `P` | B+4 | dimensionless | Ratio of semi-latus rectum to r1 |
| `R1A` | B+6 | dimensionless | Ratio r1/a |
| `EPSILONL` | B+28 × 2^-19 × |TDESIRED| | seconds | Time convergence tolerance |
| `SNTH` | B+1 | dimensionless | sin(θ), range [-1, 1] |
| `CSTH` | B+1 | dimensionless | cos(θ), range [-1, 1] |

---

## 5. Invariants

1. **No heap**: no `Vec`, `Box`, or dynamic allocation.
2. **No `unwrap`**: all `Option` values (from `unit()` calls) handled with early `converged: false`.
3. **No panic**: degenerate geometry always produces `converged: false`.
4. **Bounded iterations**: exits after MAX_ITERATIONS COGA steps regardless of convergence.
5. **Positive dt required**: `dt <= 0.0` → immediate `converged: false` (AGC restriction 4, page 1267).
6. **Collinear r1/r2**: `|sin(θ)| < 1e-10` → `converged: false`. This covers both 0° and 180° transfers (COLINEAR + 360LAMB branches).
7. **Result validity gate**: callers must check `converged` before using `v1`/`v2`. In flight code, a `false` result must trigger `alarm::raise`.

---

## 6. Test Cases

### TC-L-1: Hohmann-like LEO-to-LEO transfer

Two-burn Hohmann between 185 km circular and 400 km circular orbits.

```
mu = 3.986_032e14

r_185 = 6_556_370.0   (m, 185 km + Earth radius 6_371_370 m)
r_400 = 6_771_000.0   (m, 400 km + Earth radius)
a_transfer = (r_185 + r_400) / 2.0 = 6_663_685.0

# Half-period of transfer ellipse:
dt = π * √(a_transfer³ / mu) ≈ 1_697.0  (seconds)

r1 = [r_185, 0.0, 0.0]
r2 = [-r_400, 0.0, 0.0]   (opposite side, short-way = 180° — use Long or exact 180° handshake)

# Use short-way interpretation (direction = Short), θ ≈ 180°.
# To avoid the exact-180° singularity, offset r2 slightly:
r2 = [-r_400 * cos(0.001), r_400 * sin(0.001), 0.0]

Expected:
  converged = true
  |v1| ≈ 7_878.0 m/s  (first Hohmann burn speed)
  v1[0] ≈ 0.0  (tangential at perigee)
  v1[1] ≈ 7_878.0  (prograde)
  |v2| ≈ 7_666.0 m/s  (arrival speed at apogee)
  Tolerances: 10 m/s on magnitudes
```

### TC-L-2: Short-way vs long-way same endpoints

```
mu = 3.986_032e14
r1 = [6_571_000.0, 0.0, 0.0]
r2 = [0.0, 6_571_000.0, 0.0]   (90° apart, circular orbit)
dt = 1_200.0   (seconds)

short = lambert(r1, r2, dt, mu, Short)
long  = lambert(r1, r2, dt, mu, Long)

Expected:
  short.converged = true
  long.converged  = true
  short.v1 ≠ long.v1    (must differ — different orbit planes / directions)
  |short.v1| ≠ |long.v1|  (different energies for same dt)
  Both: energy = |v1|²/2 - mu/|r1| is conserved along arc to r2
        (check: |v2|²/2 - mu/|r2| ≈ same energy as v1, within 10 J/kg)
```

### TC-L-3: Collinear r1 and r2 — degenerate case (must not panic)

```
mu = 3.986_032e14
r1 = [6_571_000.0, 0.0, 0.0]
r2 = [7_000_000.0, 0.0, 0.0]   (exactly collinear: same direction)

result = lambert(r1, r2, 1800.0, mu, Short)

Expected:
  result.converged = false   (rectilinear trajectory not solvable)
  (no panic, no unwrap failure, no infinite loop)
```

### TC-L-4: Known transfer from P30/P31 usage — Earth return targeting

Simulated mid-course correction geometry (representative of P37 return-to-Earth scenario):

```
mu = 3.986_032e14

# Spacecraft at lunar distance on return trajectory, targeting Earth re-entry
# (representative; not a specific Apollo mission data point, but physically valid)
r1 = [300_000_000.0, 100_000_000.0, 0.0]    (m, ~0.3 lunar distances from Earth)
r2 = [6_500_000.0, 500_000.0, 0.0]          (m, near Earth, entry corridor)
dt = 72_000.0                                 (seconds, 20 hours)

result = lambert(r1, r2, dt, mu, Short)

Expected:
  result.converged = true
  Vis-viva at r1: |v1|²/2 - mu/|r1| ≈ |v2|²/2 - mu/|r2|  (within 100 J/kg)
  |v1| < 3_000.0 m/s   (reasonable mid-course speed at that distance)
  iterations <= 20
```

---

## 7. agc-sim Impact

- **No new DSKY display state**: Lambert is a pure computation callable from guidance programs.
- **P30/P31 linkage**: when V37N40 triggers a burn in agc-sim, the guidance layer calls `lambert` to compute the required `VVEC`. The delta-V is then displayed on the DSKY Mission State panel as `VG` (velocity-to-go).
- **SimLog**: emit `info`-level log on Lambert convergence failure: `"lambert: SOLNSW — no solution, iterations={n}, dir={dir:?}"`.
- **No new keyboard bindings** needed.
- **agc-sim scenario** `burn`: this scenario exercises Lambert indirectly. The developer must verify that the burn scenario runs without triggering `converged: false` in the nominal case.
