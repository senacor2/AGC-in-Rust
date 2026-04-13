# Functional Specification: Universal-Variable Kepler Propagator (`agc-core/src/math/kepler`)

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc
Routines:   KEPLERN  (entry label, pages 1277-1284)
            DELTIME  (Stumpff-function polynomial evaluation, pages 1283-1284)
            KEPPREP  (initial-guess computation, ORBITAL_INTEGRATION.agc pages 1334-1336)
            CHECKCTR (iteration counter guard, page 1286)
Pages:      1277-1284 (KEPLERN/DELTIME), 1305-1308 (erasable assignments)
```

Secondary references:
- `docs/agc-reference-constants.md` — MU_EARTH, MU_MOON, Kepler iteration limit (20)
- `agc-core/src/math/linalg.rs` — `dot`, `norm`, `scale`, `add`, `sub`, `unit`
- `agc-core/src/types/vector.rs` — `Vec3 = [f64; 3]`

---

## 1. Behavior Summary

KEPLERN is the AGC's conic-section propagator. Given a state vector (position RRECT, velocity VRECT) and a desired transfer time TAU (which may be positive, negative, or larger than one orbital period), it computes the new state vector (RCV, VCV) by solving Kepler's equation in the universal-variable (χ, also written X in the source) formulation.

The universal variable χ is defined by the equation:

```
t - t0 = (1/√μ) * [C1 * χ * C(z) + C2 * χ³ * S(z) + (r0/√μ) * χ²/2] / r(χ)
```

where:
- `z = α * χ²` and `α = 1/a` is the reciprocal of the semi-major axis (negative for hyperbolic)
- `C(z)` and `S(z)` are the Stumpff functions (see section 1.2)
- `C1 = r0·v0 / √μ` (radial velocity component, scaled)
- `C2 = r0·v0²/μ - 1` (energy-related scalar)

The single-variable formulation handles all conic types (elliptic, parabolic, hyperbolic) without branching on orbit type, except for the sign of α which determines the initial guess bounds.

### 1.1 Algorithm Outline

1. **Initialization** (KEPLERN entry):
   - Compute `C1 = dot(RRECT, VRECT) / √μ` (AGC: `KEPC1`, scale +17 Earth / +16 Moon)
   - Compute `C2 = |VRECT|² / μ * r0 - 1` (AGC: `KEPC2`, scale +6)
   - Compute `α = (1 - C2) / r0` (AGC: `ALPHA`, scale -22 Earth / -20 Moon; negative for hyperbolic)
   - Determine maximum χ (XMAX) to bound the search:
     - If α > 0 (elliptic/circular): `XMAX = 2π / √α` (one full orbit in χ-space)
     - If α ≤ 0 (hyperbolic): `XMAX = 50 / |α|` (scaled bound from AGC constant `-50SC`)
   - Modulo reduction: if |TAU| > orbital period, subtract integer multiples of the period (stored in TMODULO) until |TAU| < one period. The AGC source implements this via the `PERIODCH` loop (page 1278). For negative TAU, signs are preserved.
   - Bounds: set XMIN = 0, XMAX from above; negate XMIN/XMAX if TAU < 0 (via `STORBNDS`)
   - Initial guess X ← XKEPNEW. If the guess violates sign or magnitude constraints, fall back to `XMAX/2` (`BADX` branch, page 1278).

2. **Convergence tolerance** (KEPLERN, `DXCOMP` label, page 1278-1279):
   - `EPSILONT = |TAU| * 2^-22` (AGC constant `BEE22`, i.e. 1 bit at scale B-22)
   - This tolerance is on `|DELT - TAU|` (time residual), not on χ directly.

3. **Newton iteration** (`KEPLOOP`, pages 1279-1281):
   - For each iteration:
     a. Compute `z = α * χ²` (AGC: `XI`, scale +6)
     b. Call `DELTIME` to evaluate `S(z)` → `S(XI)` and `C(z) * χ²` → `XSQC(XI)`
     c. Compute time function T (Kepler's equation evaluated at current χ): see DELTIME section
     d. Compare `|T - TAU|` with EPSILONT:
        - If converged: jump to `KEPCONVG`
        - Else: Newton step `DELX = (TAU - T) / (dT/dχ)` where `dT/dχ = r(χ)/√μ`
     e. Bisection fallback: if the Newton step would exceed the [XMIN, XMAX] bracket, reduce it by factor 0.9 and clamp to the bracket (`NDXCHNGE` / `PDXCHNGE` labels, page 1280)
     f. Update X ← X + DELX; update TC ← T
   - Iteration count tracked in `ITERCTR` (initialized to 20D at KEPLERN entry, `SSP ITERCTR 20D` page 1277).
   - Counter guard: `CHECKCTR` routine (page 1286) decrements `ITERCTR` by 1 each iteration (via `CS ONE / INDEX FIXLOC / AD ITERCTR / TS ITERCTR`). When count reaches zero, `BHIZ KEPCONVG` fires and the loop exits without convergence.

4. **Time overflow guard** (`TIMEOVFL`, page 1281): if evaluating the time function overflows the AGC accumulator, X is moved to XMAX or XMIN (depending on sign) and DELX is halved.

5. **Final state computation** (`KEPCONVG`, pages 1281-1282):
   - Compute f and g Lagrange coefficients from χ, S(z), C(z):
     - `f = 1 - χ² * C(z) / r0`
     - `g = t - χ³ * S(z) / √μ`
   - New position: `RCV = f * RRECT + g * VRECT` (VSL4 / VAD in source)
   - New velocity computed via f-dot and g-dot using the final radius magnitude `RCNORM`.

### 1.2 Stumpff Functions C(z) and S(z) — the DELTIME Subroutine

The DELTIME routine (CONIC_SUBROUTINES.agc, page 1283) evaluates both Stumpff functions as Taylor series polynomial approximations using the `POLY` subroutine. The input is `XI = α * χ²` (called `z` in the literature).

**S(z)** is evaluated by an 8th-degree polynomial in `z` (9 coefficients):

```
S(z) ≈ 1/6 - z/120 + z²/5040 - ...
```

The AGC stores these as `2DEC` constants (page 1283):
```
0.083333334   (≈ 1/12, note: this is S(z)*2 due to scaling)
-0.266666684  (≈ -1/...  )
0.406349155
-0.361198675
0.210153242
-0.086221951
0.026268812
-0.006163316
0.001177342
-0.000199055
```

**C(z)** is evaluated by a second 8th-degree polynomial (page 1883-1884):
```
0.031250001   (≈ 1/32, corresponding to 1/2! in C-function series)
-0.166666719
0.355555413
-0.406347410
0.288962094
-0.140117894
0.049247387
-0.013081923
0.002806389
-0.000529414
```

After polynomial evaluation, DELTIME computes `XSQC(XI) = C(z) * χ²` (scale +33 or +31) and then assembles the time equation T:

```
T = C1 * χ² * C(z) + C2 * χ³ * S(z) + (r0 * χ) / √μ + (previous terms)
```

which is stored in `T (30D)` at scale +28.

The Rust implementations of `stumpff_c` and `stumpff_s` must reproduce these polynomial coefficients exactly. The standard closed-form definitions for reference:

```
S(z) = (√z - sin(√z)) / z^(3/2)     for z > 0
S(z) = (sinh(√(-z)) - √(-z)) / (-z)^(3/2)  for z < 0
S(0) = 1/6

C(z) = (1 - cos(√z)) / z            for z > 0
C(z) = (cosh(√(-z)) - 1) / (-z)     for z < 0
C(0) = 1/2
```

For the Rust implementation, using the Taylor series (as the AGC does) is preferred for numerical stability near z = 0 (near-parabolic and near-circular cases). The series converges for |z| < π² (elliptic), and for hyperbolic z the series must be evaluated with |z| provided.

---

## 2. Rust API

**Module path**: `agc_core::math::kepler`

```rust
/// Result of a universal-variable Kepler propagation step.
///
/// If `converged` is false, `r` and `v` contain the best-available approximation
/// (the state at the last completed iteration) and must NOT be used as navigation
/// truth. The caller must invoke `alarm::raise(AlarmCode::KeplerDiverged)` and
/// enter the appropriate restart path.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN (page 1277).
pub struct KeplerResult {
    /// Terminal position vector, ECI metres.
    pub r: Vec3,
    /// Terminal velocity vector, ECI m/s.
    pub v: Vec3,
    /// True if the Newton iteration converged within MAX_ITERATIONS.
    pub converged: bool,
    /// Number of Newton iterations actually performed.
    pub iterations: u32,
}

/// Propagate a conic state vector forward by `dt` seconds using the universal
/// variable (Battin/Goodyear) formulation.
///
/// # Arguments
/// - `r0`: initial position in ECI metres.
/// - `v0`: initial velocity in ECI m/s.
/// - `dt`: transfer time in seconds (positive = forward, negative = backward).
/// - `mu`: gravitational parameter in m³/s² (use `MU_EARTH` or `MU_MOON`).
///
/// # Returns
/// `KeplerResult` with terminal state and convergence flag. Never panics.
/// On non-convergence, returns `converged: false` with the last iterate.
///
/// # Invariants
/// - No heap allocation; no `unwrap`; no panic.
/// - Bounded to MAX_ITERATIONS Newton steps.
/// - Handles dt = 0 by returning the input state immediately (zero iterations).
/// - Handles dt > one orbital period via modulo reduction (TMODULO equivalent).
/// - Negative dt is fully supported (backward propagation).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN routine (page 1277).
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP (initial guess, page 1334).
pub fn kepler(r0: &Vec3, v0: &Vec3, dt: f64, mu: f64) -> KeplerResult { ... }

/// Stumpff function S(z).
///
/// Defined as:
///   S(z) = (√z - sin(√z)) / z^(3/2)    for z > 0
///   S(z) = (sinh(√(-z)) - √(-z)) / (-z)^(3/2)  for z < 0
///   S(0) = 1/6
///
/// Implemented as a degree-8 Taylor series to match the AGC DELTIME polynomial
/// (CONIC_SUBROUTINES.agc, page 1283). Series is valid for |z| < ~40 without
/// significant error accumulation.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, DELTIME (`S(XI)` output).
fn stumpff_s(z: f64) -> f64 { ... }

/// Stumpff function C(z).
///
/// Defined as:
///   C(z) = (1 - cos(√z)) / z            for z > 0
///   C(z) = (cosh(√(-z)) - 1) / (-z)     for z < 0
///   C(0) = 1/2
///
/// Implemented as a degree-8 Taylor series to match the AGC DELTIME polynomial
/// (CONIC_SUBROUTINES.agc, page 1283-1284, `XSQC(XI)` derivation).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, DELTIME.
fn stumpff_c(z: f64) -> f64 { ... }
```

---

## 3. Constants

| Constant | Value | Source | Notes |
|---|---|---|---|
| `MAX_ITERATIONS` | `20` | `CONIC_SUBROUTINES.agc` line `SSP ITERCTR 20D` (page 1277) | AGC used 20; `docs/agc-reference-constants.md` confirms |
| `KEPLER_TOL` | `1e-12` (seconds) | Derived | Convergence in time residual; tighter than AGC's BEE22 (≈ 2^-22 × dt) at nominal dt values; use `|DELT - TAU| < max(KEPLER_TOL, |TAU| * 2.0f64.powi(-22))` |
| `MU_EARTH` | `3.986_032e14` m³/s² | `CONIC_SUBROUTINES.agc` MUTABLE table (page 1305), line `2DEC* 3.986032 E10 B-36*` | See `docs/agc-reference-constants.md` |
| `MU_MOON` | `4.902_778e12` m³/s² | `CONIC_SUBROUTINES.agc` MUTABLE, line `2DEC 4.902778 E8 B-30` | |

Note: The AGC MUTABLE table (page 1305) also stores `√μ` and `1/√μ` for Earth and Moon. The Rust port computes these at initialization via `libm::sqrt`.

---

## 4. Scale Factors (AGC → Rust)

The AGC operated in ones-complement fixed-point with body-dependent scale factors. The Rust port uses plain `f64` SI.

| AGC variable | AGC scale (Earth) | SI unit | Notes |
|---|---|---|---|
| `RRECT` / `RCV` | B+29 (1 word = 2²⁹ m) | metres | Position 3-vector |
| `VRECT` / `VCV` | B+7 (1 word = 2⁷ m/cs) | m/s × 100 → m/s | Velocity; AGC uses m/centisecond |
| `TAU` | B+28 (centiseconds) | seconds × 100 → seconds | Transfer time |
| `X` (chi, χ) | B+17 (Earth) | √m | Universal variable |
| `ALPHA` | B-22 (Earth) | 1/m | Reciprocal SMA |
| `T (30D)` | B+28 (centiseconds) | seconds | Time function at current iterate |
| `EPSILONT` | B-22 × |TAU| | seconds | Convergence tolerance (time) |

Velocity conversion: AGC uses m/centisecond; Rust uses m/s. All velocity inputs/outputs are in m/s. Internally, the AGC time is in centiseconds (× 100); the Rust interface uses seconds throughout.

---

## 5. Invariants

1. **No heap**: `KeplerResult` is a plain struct; no `Vec`, `Box`, or `alloc` usage anywhere in this module.
2. **No `unwrap`**: all `Option` returns from linalg helpers (e.g. `unit`) are handled with early `converged: false` returns.
3. **Bounded iterations**: the loop exits after exactly `MAX_ITERATIONS` Newton steps even if not converged.
4. **No panic**: the function returns `converged: false` rather than panicking on degenerate inputs (zero r0, zero mu, etc.).
5. **dt = 0**: returns `{ r: *r0, v: *v0, converged: true, iterations: 0 }` immediately.
6. **Negative dt**: fully supported; bounds XMIN/XMAX are negated per AGC `STORBNDS` logic.
7. **Restart safety**: the function is stateless (no erasable side effects); it is safe to re-call after a restart without additional setup, unlike the AGC which required XPREV/TC to be preserved across calls.
8. **Finite outputs**: if `mu <= 0.0` or `norm(r0) < 1.0` (singularity guard), return `converged: false` immediately.

---

## 6. Test Cases

### TC-K-1: Circular LEO orbit, quarter-period propagation

```
mu  = 3.986_032e14   (MU_EARTH)
r0  = [6_571_000.0, 0.0, 0.0]          (m, 200 km altitude)
v0  = [0.0, 7_784.26, 0.0]             (m/s, circular velocity)
dt  = T_period / 4  ≈ 1_351.0          (seconds)

Expected outcome:
  r ≈ [0.0, 6_571_000.0, 0.0]     (rotated 90°)
  v ≈ [-7_784.26, 0.0, 0.0]       (tangential, rotated)
  converged = true
  |r| ≈ 6_571_000.0 m  (to within 1.0 m)
  |v| ≈ 7_784.26 m/s   (to within 0.01 m/s)
```

Tolerance: position 1 m, velocity 0.1 m/s.

### TC-K-2: Elliptic orbit, propagation from perigee to apogee

```
mu     = 3.986_032e14
r_per  = 6_571_000.0    (m, 200 km altitude perigee)
r_apo  = 42_164_000.0   (m, GEO altitude apogee)
a      = (r_per + r_apo) / 2 = 24_367_500.0   (SMA)
e      = (r_apo - r_per) / (r_apo + r_per) ≈ 0.7286
v_per  = √(mu * (2/r_per - 1/a)) ≈ 10_059.0   (m/s)
v_apo  = √(mu * (2/r_apo - 1/a)) ≈  1_567.0   (m/s)

r0  = [r_per, 0.0, 0.0]
v0  = [0.0, v_per, 0.0]
dt  = π * √(a³/mu)  ≈ 19_006.0     (half-period)

Expected:
  r ≈ [-r_apo, 0.0, 0.0]    (opposite side)
  v ≈ [0.0, -v_apo, 0.0]   (retrograde at apogee)
  converged = true
  |r| within 100 m of r_apo
  |v| within 1.0 m/s of v_apo
```

### TC-K-3: Hyperbolic flyby

```
mu   = 3.986_032e14
v_inf = 3_000.0             (m/s excess hyperbolic speed)
r_per = 6_671_000.0         (m, 300 km periapsis)
a_hyp = -mu / v_inf²  ≈ -44_289_244.0  (negative SMA for hyperbolic)
e     = 1.0 - r_per / a_hyp ≈ 1.1507
v_per = √(mu * (2/r_per - 1/a_hyp)) ≈ 10_618.0  (m/s)

# State near periapsis, propagate 3600 s forward:
r0  = [r_per, 0.0, 0.0]
v0  = [0.0, v_per, 0.0]
dt  = 3_600.0

Expected:
  converged = true
  |r| > r_per  (moving away from periapsis)
  Energy = v²/2 - mu/|r| ≈ v_inf²/2  (conserved to 1 J/kg)
```

Specific mechanical energy must be conserved to within 1 J/kg.

### TC-K-4: Zero transfer time — identity

```
r0  = [7_000_000.0, 1_000_000.0, 500_000.0]  (arbitrary)
v0  = [100.0, -7_500.0, 200.0]
dt  = 0.0

Expected:
  r = r0  (exactly)
  v = v0  (exactly)
  converged = true
  iterations = 0
```

### TC-K-5: Near-parabolic edge case (e ≈ 0.9999)

```
mu    = 3.986_032e14
r_per = 6_571_000.0
e     = 0.9999
a     = r_per / (1.0 - e) = 6_571_000_000.0  (very large SMA)
v_per = √(mu * (1 + e) / r_per) ≈ 11_184.0  (m/s, near escape velocity)

r0  = [r_per, 0.0, 0.0]
v0  = [0.0, v_per, 0.0]
dt  = 600.0   (10 minutes from periapsis)

Expected:
  converged = true   (may require all 20 iterations)
  |r| > r_per
  Energy ≈ -mu/(2*a) ≈ -0.0304 J/kg  (small negative, nearly parabolic)
  Energy conserved to within 10 J/kg
```

This case exercises the near-zero-alpha branch of the universal variable (z very close to 0), which stresses the Stumpff series convergence.

---

## 7. agc-sim Impact

- **No new DSKY display state** is introduced. The Kepler propagator is a pure computation called internally by orbital integration.
- **SimLog**: emit a debug-level log entry when `converged = false` to assist integration testing. The message format should be: `"kepler: did not converge after N iterations, dt={dt:.1}s"`.
- **MissionState panel**: the `sma`, `ecc`, `apo`, `per` fields displayed in the Mission State panel are computed from the post-Kepler state vector by `navigation::conics`; no direct Kepler output appears on the DSKY.
- **No new keyboard bindings** are needed.
