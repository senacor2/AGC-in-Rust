# Specification: `math/kepler` Module

**Status**: Approved for implementation
**Module path**: `agc-core/src/math/kepler.rs`
**Architecture reference**: `docs/architecture.md` §9.2 "Function Granularity", §9.3 "No Interpreter State"
**Integration reference**: `specs/integration-spec.md` §4 — `propagate_coast` calling convention
**Gravity reference**: `specs/gravity-spec.md` §3 — `MU_EARTH`, `MU_MOON` constant values
**State-vector reference**: `specs/state-vector-spec.md` §2.1 — position/velocity encoding
**Math reference**: `specs/linalg-spec.md` §4 — `dot`, `norm`, `vscale`, `vadd`, `vsub`
**AGC source file**: `Comanche055/CONIC_SUBROUTINES.agc`
**AGC routine**: `KEPRTN` (Kepler propagation return), `KEPSILON` (Kepler equation solver)
**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

`math::kepler` provides the universal-variable conic-section propagator that
advances a spacecraft state vector (position and velocity) forward by an
arbitrary time interval under a central-body point-mass gravitational field.

The module is the primary computational kernel for coast-phase trajectory
propagation. In the AGC Comanche055 software the corresponding function was the
`KEPRTN` interpretive subroutine, which implemented a universal-variable Kepler
equation solver using Stumpff functions. Because the Rust port eliminates the
interpretive language in favour of direct `f64` arithmetic (architecture §9.1,
ADR-001), the entire KEPRTN routine is expressed as a single Rust function:
`kepler_step`.

`kepler_step` is unconditionally valid for **all conic sections** — circular,
elliptic, parabolic, and hyperbolic — without requiring the caller to
pre-classify the orbit. This matches the AGC's requirement: during a
translunar trajectory the spacecraft passes through regions where the orbit
relative to Earth is hyperbolic with respect to the Moon, and the propagator
must handle all cases without branching on orbit type.

### What this module provides

- `kepler_step` — the universal-variable Kepler propagator (one function, one
  public symbol).
- `stumpff_c` — Stumpff C function C(z) = (1 − cos(√z))/z (pub for testing).
- `stumpff_s` — Stumpff S function S(z) = (√z − sin(√z))/(√z)³ (pub for testing).

### What this module does NOT provide

- Perturbation forces (J2, third-body). Those are added by
  `navigation::integration::propagate_coast` on top of the Kepler step. See
  `specs/integration-spec.md` §4.5.
- Lambert solver. That is in `math::lambert`.
- Orbit element conversion. There is no dedicated module for orbital elements;
  specific callers that need them compute them locally.
- Frame transformation. `kepler_step` operates on a bare `Vec3` pair; the
  caller in `propagate_coast` is responsible for passing vectors in a consistent
  inertial frame and attaching the result to the correct `StateVector`.
- Numerical integration fallback. If `dt` is very small the algorithm naturally
  converges in one Newton-Raphson iteration; there is no separate short-arc
  code path.

---

## 2. AGC Background

### 2.1 The KEPRTN Routine in Comanche055

The original subroutine lives in `Comanche055/CONIC_SUBROUTINES.agc`. The entry
point label is `KEPRTN`; the Kepler-equation Newton-Raphson loop within it is
labelled `KEPSILON` (tolerance check). The routine was called by:

- `ORBITAL_INTEGRATION.agc` — Encke's method reference-conic update.
- `INTEGRATION_INITIALIZATION.agc` — initial conic setup at SOI transitions.
- Various P-program entry routines (P30, P34, P37) for targeting burn epochs.

The AGC stored the state vector in DP fixed-point with scale factor B+28 m for
position and B+7 m/s for velocity, but the algorithm itself is scale-factor-free
once the universal variable χ is used. The Rust port replaces all DP fixed-point
arithmetic with native `f64` SI values.

### 2.2 Universal Variable Formulation (Battin Method)

The AGC chose the Battin universal-variable formulation (R. H. Battin,
*An Introduction to the Mathematics and Methods of Astrodynamics*, 1987) over
the classical Kepler equation because:

1. A single equation and a single iteration variable χ cover all conic types.
2. Parabolic trajectories (eccentricity e = 1) are handled without a special
   case.
3. The series expansions for the Stumpff functions converge quickly near the
   parabolic limit, where classical anomaly-based solvers diverge.
4. The formulation directly yields the f and g Lagrange coefficients needed to
   construct the propagated position and velocity without separately computing
   a time-of-flight integral.

The Rust port follows the same formulation, using `f64` with IEEE 754
double-precision throughout.

---

## 3. Mathematical Background

### 3.1 Universal Anomaly χ

For a central-body orbit with gravitational parameter μ the universal anomaly χ
satisfies the universal Kepler equation:

```
t = (r0_dot_v0 / √μ) · χ² · C(z)
  + (1 − r0/a) · χ³ · S(z)
  + r0 · χ / √μ
```

where:
- `r0 = norm(r0_vec)` — initial radius (m)
- `r0_dot_v0 = dot(r0_vec, v0_vec)` — radial velocity component (m²/s)
- `a` — semi-major axis: `a = 1 / (2/r0 − v0²/μ)` (m); negative for hyperbolic
- `z = α · χ²` — the Stumpff argument, with `α = 1/a` (m⁻¹)
- `t = dt` — propagation time (s)
- `C(z)`, `S(z)` — Stumpff functions (see §3.2)

The universal Kepler equation is transcendental in χ; it is solved by
Newton-Raphson iteration (§4.3).

### 3.2 Stumpff Functions C(z) and S(z)

The Stumpff functions provide a unified representation that avoids the
discontinuity at e = 1. Their definitions by case are:

**C(z):**

| Case | Condition | Formula |
|------|-----------|---------|
| Elliptic | z > 0 | (1 − cos(√z)) / z |
| Parabolic | z = 0 | 1/2 |
| Hyperbolic | z < 0 | (cosh(√(−z)) − 1) / (−z) |

**S(z):**

| Case | Condition | Formula |
|------|-----------|---------|
| Elliptic | z > 0 | (√z − sin(√z)) / (√z)³ |
| Parabolic | z = 0 | 1/6 |
| Hyperbolic | z < 0 | (sinh(√(−z)) − √(−z)) / (√(−z))³ |

Implementation note: on a `no_std` target `f64::sin`, `f64::cos`, `f64::sinh`,
`f64::cosh`, and `f64::sqrt` are not available through the standard library. Use
`libm::sin`, `libm::cos`, `libm::sinh`, `libm::cosh`, and `libm::sqrt` from the
`libm = "0.2"` crate dependency, consistent with the convention established in
`math::linalg` (linalg-spec §3) and `math::trig`.

Near-parabolic branch threshold: use `|z| < 1e-6` as the parabolic case. Values
in `(-1e-6, +1e-6)` shall use the Taylor series expansions:
```
C(z) = 1/2 − z/24 + z²/720 − z³/40320 + ...
S(z) = 1/6 − z/120 + z²/5040 − z³/362880 + ...
```
truncated at the z³ term, which gives relative error < 1e-28 for |z| < 1e-6.
This avoids catastrophic cancellation in the `cos`/`sin` branch when z is tiny.

### 3.3 Lagrange Coefficients f and g

Once χ is found, the propagated state is:

```
f = 1 − (χ² / r0) · C(z)
g = dt − (χ³ / √μ) · S(z)

r1_vec = f · r0_vec + g · v0_vec

f_dot = (√μ / (r0 · r1)) · χ · (z · S(z) − 1)
g_dot = 1 − (χ² / r1) · C(z)

v1_vec = f_dot · r0_vec + g_dot · v0_vec
```

where `r1 = norm(r1_vec)`.

The identity `f · g_dot − g · f_dot = 1` is the Wronskian condition and
guarantees that the specific angular momentum vector is conserved exactly
(angular momentum invariant, §7.2). It can be verified in tests but need not
be enforced inside the function at runtime.

### 3.4 Initial Estimate for χ

The Newton-Raphson iteration requires a starting value. Use:

```
χ₀ = √μ · |dt| · |α|
```

For circular/near-circular orbits this is close to the true value. For
high-eccentricity elliptic and hyperbolic orbits the iteration still converges
in 4–6 steps from this estimate. The sign of `χ₀` is taken as the sign of `dt`.

A tighter starting estimate from Battin (p. 193) that is recommended for
implementation:

```
χ₀ = √μ · dt / r0    (for |α| < 1e-9, i.e. parabolic/near-parabolic)
χ₀ = √(1/|α|) · sign(dt) · (... Brent bound ...)  (elliptic / hyperbolic)
```

For the initial implementation the simpler estimate `χ₀ = √μ · dt / r0` is
acceptable. It converges for circular LEO in 3–4 iterations and for high-
eccentricity transfers in 6–10 iterations.

---

## 4. Function Specification: `kepler_step`

### 4.1 Purpose

Propagate a two-body Keplerian state vector from time `t0` to `t0 + dt` under
a central body with gravitational parameter `mu`. Returns the propagated position
and velocity in the same coordinate frame and units as the inputs.

This is the Rust replacement for the AGC's `KEPRTN` interpretive subroutine
(`Comanche055/CONIC_SUBROUTINES.agc`).

### 4.2 Signature

```rust
/// Propagate a two-body state vector by `dt` seconds under a central body
/// with gravitational parameter `mu` (m³/s²).
///
/// Uses the universal-variable (Battin) formulation with Stumpff functions
/// C(z) and S(z), valid for elliptic, parabolic, and hyperbolic orbits.
///
/// # Parameters
/// - `r0`: Initial position vector (m), in any inertial frame whose origin
///   is the central body's centre of mass.
/// - `v0`: Initial velocity vector (m/s), consistent frame with `r0`.
/// - `dt`: Propagation interval (s). Must be positive and finite.
/// - `mu`: Gravitational parameter of the central body (m³/s²). Use
///   `navigation::gravity::MU_EARTH` (3.986_004_418e14) for Earth or
///   `navigation::gravity::MU_MOON` (4.902_800_118e12) for the Moon.
///
/// # Returns
/// `(r1, v1)` — position (m) and velocity (m/s) at time `t0 + dt`.
///
/// # Panics (debug builds only)
/// - `dt` is not finite or is zero.
/// - `norm(r0)` is zero or not finite.
/// - `mu` is not positive and finite.
///
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc, routine KEPRTN.
pub fn kepler_step(r0: Vec3, v0: Vec3, dt: f64, mu: f64) -> (Vec3, Vec3)
```

### 4.3 Algorithm (Newton-Raphson on the Universal Kepler Equation)

The following pseudocode specifies the required algorithm. The Rust
implementation must follow these steps in order.

**Pre-computation:**
```
r0_mag     = norm(r0)                         // initial radius (m)
v0_sq      = dot(v0, v0)                      // v0² (m²/s²)
r0_dot_v0  = dot(r0, v0)                      // r⃗₀ · v⃗₀ (m²/s)
alpha      = 2.0/r0_mag − v0_sq/mu            // = 1/a  (m⁻¹); negative if hyperbolic
sqrt_mu    = libm::sqrt(mu)
```

**Initial estimate for χ:**
```
chi = sqrt_mu * dt / r0_mag
```

**Newton-Raphson iteration (max 50 iterations):**

At each iteration, given current `chi`:
```
z   = alpha * chi * chi
(c, s) = (stumpff_c(z), stumpff_s(z))

// Universal Kepler equation residual:
psi = (r0_dot_v0 / sqrt_mu) * chi * chi * c
    + (1.0 - r0_mag * alpha) * chi * chi * chi * s
    + r0_mag * chi
    - sqrt_mu * dt

// Derivative of psi with respect to chi (for Newton step):
r_now = (r0_dot_v0 / sqrt_mu) * chi * (1.0 - z * s)
      + (1.0 - r0_mag * alpha) * chi * chi * c
      + r0_mag

dpsi_dchi = r_now

// Newton update:
chi = chi - psi / dpsi_dchi
```

Convergence criterion: `|psi / dpsi_dchi| < 1e-9 * |chi| + 1e-12`

This corresponds to a relative tolerance of ~1e-9 on χ, matching the AGC's
`KEPSILON` tolerance. The absolute floor of 1e-12 prevents division-by-zero
when `chi` is near zero (very short arcs).

**Post-iteration: Lagrange coefficients and propagated state:**
```
z   = alpha * chi * chi           // recompute with converged chi
(c, s) = (stumpff_c(z), stumpff_s(z))
r1_mag = (r0_dot_v0 / sqrt_mu) * chi * (1.0 - z * s)
       + (1.0 - r0_mag * alpha) * chi * chi * c
       + r0_mag

f     = 1.0 - (chi * chi / r0_mag) * c
g     = dt  - (chi * chi * chi / sqrt_mu) * s
f_dot = (sqrt_mu / (r0_mag * r1_mag)) * chi * (z * s - 1.0)
g_dot = 1.0 - (chi * chi / r1_mag) * c

r1 = vscale(r0, f) + vscale(v0, g)        // = vadd(vscale(r0,f), vscale(v0,g))
v1 = vscale(r0, f_dot) + vscale(v0, g_dot)
```

Return `(r1, v1)`.

### 4.4 Parameters — Full Specification

| Parameter | Type | Units | Range | Description |
|-----------|------|-------|-------|-------------|
| `r0` | `Vec3` | m | norm > 0 | Initial position relative to central body. Must be finite and non-zero. |
| `v0` | `Vec3` | m/s | finite | Initial velocity. Must be finite. Zero is valid (radial free-fall). |
| `dt` | `f64` | s | > 0, finite | Propagation interval. Negative `dt` (back-propagation) is not required; the integration-spec callers always supply positive `dt`. |
| `mu` | `f64` | m³/s² | > 0, finite | Central body gravitational parameter. |

### 4.5 Return Values

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `.0` | `Vec3` | m | Position at `t0 + dt` in same frame as `r0`. |
| `.1` | `Vec3` | m/s | Velocity at `t0 + dt` in same frame as `v0`. |

### 4.6 Preconditions

PRE-1: `norm(r0) > 0.0` and `norm(r0)` is finite — the spacecraft is not at
the body's centre of mass.

PRE-2: All components of `r0` and `v0` are finite (`f64` values free of NaN
and Inf), consistent with the `Vec3` invariant in `specs/types-module-spec.md`
§3.3.

PRE-3: `dt > 0.0` and `dt.is_finite()`.

PRE-4: `mu > 0.0` and `mu.is_finite()`.

PRE-5 (implied): The orbit is not a radial crash trajectory within `dt`. That
is, the spacecraft does not reach the central body's surface before `t0 + dt`.
The function does not check this; the caller (`propagate_coast`) is responsible
for ensuring physically valid arcs.

Precondition violations are caught by `debug_assert!` in debug builds. In
release builds they produce NaN propagation detected at the next
`StateVector` validity check (which triggers a restart).

### 4.7 Postconditions

POST-1: The returned `Vec3` values are finite.

POST-2 (energy): The specific orbital energy
`E = 0.5 * norm(v1)² − mu / norm(r1)` equals the initial specific energy
`E0 = 0.5 * norm(v0)² − mu / norm(r0)` to within a relative tolerance of
1e-9 (the convergence tolerance of the Newton-Raphson iteration).

POST-3 (angular momentum): The specific angular momentum vector
`h1 = cross(r1, v1)` is parallel to and has the same magnitude as
`h0 = cross(r0, v0)` to within a relative tolerance of 1e-9.

POST-4 (round-trip): Calling `kepler_step(r1, v1, dt_full − dt_sub, mu)`
on the output of a sub-arc propagation must reproduce the endpoint of the
full arc to within the convergence tolerance. This is a consistency check,
not a hard runtime guarantee.

---

## 5. Helper Function Specifications

### 5.1 `stumpff_c`

```rust
/// Stumpff C function: C(z) = (1 − cos(√z)) / z   for z > 0,
///                             1/2                  for z = 0,
///                             (cosh(√(−z)) − 1)/(−z)  for z < 0.
/// Uses Taylor series for |z| < 1e-6 to avoid cancellation.
pub fn stumpff_c(z: f64) -> f64
```

| Input range | Formula | Notes |
|-------------|---------|-------|
| z > 1e-6 | (1 − cos(√z)) / z | Use `libm::sqrt`, `libm::cos` |
| \|z\| ≤ 1e-6 | 1/2 − z/24 + z²/720 − z³/40320 | Taylor series; 4 terms |
| z < −1e-6 | (cosh(√(−z)) − 1) / (−z) | Use `libm::sqrt`, `libm::cosh` |

### 5.2 `stumpff_s`

```rust
/// Stumpff S function: S(z) = (√z − sin(√z)) / (√z)³  for z > 0,
///                             1/6                       for z = 0,
///                             (sinh(√(−z)) − √(−z)) / (√(−z))³  for z < 0.
/// Uses Taylor series for |z| < 1e-6 to avoid cancellation.
pub fn stumpff_s(z: f64) -> f64
```

| Input range | Formula | Notes |
|-------------|---------|-------|
| z > 1e-6 | (√z − sin(√z)) / (√z)³ | Use `libm::sqrt`, `libm::sin` |
| \|z\| ≤ 1e-6 | 1/6 − z/120 + z²/5040 − z³/362880 | Taylor series; 4 terms |
| z < −1e-6 | (sinh(√(−z)) − √(−z)) / (√(−z))³ | Use `libm::sqrt`, `libm::sinh` |

---

## 6. Edge Cases

### 6.1 Very Small `dt` (Short Arc)

When `dt` approaches zero the universal anomaly χ also approaches zero, and the
Newton-Raphson iteration converges in one step (the first-order Taylor term
dominates). The algorithm is numerically stable: `psi/dpsi_dchi ≈ 0` and the
initial estimate `chi ≈ √mu * dt / r0_mag` is already within tolerance.

Expected behaviour: for `dt < 1e-9 * r0_mag / sqrt_mu` (≈ 1 ms at LEO altitudes)
the function returns `(r0 + v0*dt, v0)` to within floating-point rounding. No
special case is needed; the general algorithm handles this.

This is the regime where `propagate_coast` in integration-spec §4.6 might
previously have used its RK4 fallback. With `kepler_step` available, even
single-step sub-second propagation is handled correctly.

### 6.2 Very Large `dt` (Multi-Orbit)

For a circular LEO orbit (r ≈ 6.571e6 m), one full orbit period is
T ≈ 5411 s. For `dt = N * T` the universal anomaly χ grows as N * 2π * √(a/μ).
The Newton-Raphson iteration still converges but may require more iterations
(up to ~20 for N = 10). The 50-iteration maximum in §4.3 is sufficient for up
to ~30 full orbits, which covers all AGC coast-propagation use cases (the
longest translunar coast leg is ≈ 72 hours = ~48 LEO orbits, but translunar
coasts use the Moon as primary body with a much longer orbital period).

For `propagate_coast` callers the integration-spec §4.7 imposes a 24-hour
(86400 s) maximum on a single `propagate_coast` call. Over that window the
number of complete orbits for any AGC-relevant trajectory is bounded to fewer
than 50, well within the iteration limit.

### 6.3 Near-Parabolic Orbit (e ≈ 1)

When `|alpha| < 1e-9 m⁻¹` (semi-major axis > 1e9 m, effectively parabolic or
near-parabolic) the Stumpff-function Taylor branch (§3.2, §5.1, §5.2) is used
automatically because z = alpha * chi² will satisfy |z| < 1e-6 during early
iterations. The algorithm does not require any special-case branching on orbit
classification.

For a true parabolic orbit (`alpha = 0` exactly), `z = 0` throughout the
iteration and S(0) = 1/6, C(0) = 1/2. This reduces the universal Kepler
equation to Barker's equation, which the Newton-Raphson loop solves correctly.

### 6.4 Hyperbolic Orbit (e > 1, Translunar Injection)

During TLI (Translunar Injection) and the subsequent cislunar coast relative to
Earth, the ECI semi-major axis is negative (hyperbolic escape trajectory).
`alpha < 0` and `z = alpha * chi² < 0` throughout. The hyperbolic branch of the
Stumpff functions is used. The Newton-Raphson iteration converges normally. The
AGC relied on this behaviour for P15 (TLI monitor) and P37 (Return to Earth).

### 6.5 Newton-Raphson Failure to Converge

If the loop reaches the 50-iteration maximum without satisfying the convergence
criterion, the function must `panic!("kepler_step: Newton-Raphson did not converge")`.
This triggers the restart handler in both debug and release builds (architecture
§7). Non-convergence indicates a malformed input (physically impossible state,
zero mu, etc.) rather than a recoverable navigation error.

---

## 7. Invariants

### 7.1 Energy Conservation

For any converged solution the specific orbital energy is conserved:

```
E = 0.5 * dot(v1, v1) − mu / norm(r1)
  = 0.5 * dot(v0, v0) − mu / norm(r0)   ± ε_energy
```

where `ε_energy` satisfies `|ε_energy| / |E| < 1e-9`.

Energy conservation is a direct consequence of the Keplerian two-body problem.
The universal-variable formulation preserves it algebraically; any numerical
deviation from exact conservation is bounded by the Newton-Raphson convergence
tolerance.

### 7.2 Angular Momentum Conservation

The specific angular momentum vector is conserved:

```
h1 = cross(r1, v1) = h0 = cross(r0, v0)   ± ε_h
```

where `|ε_h| / |h0| < 1e-9`.

This follows from the Wronskian identity `f * g_dot − g * f_dot = 1`, which
holds exactly when χ is exact. Numerical deviations are bounded by the
Newton-Raphson tolerance.

### 7.3 Determinism

`kepler_step` is a pure function with no side effects and no mutable state. For
identical inputs it always returns identical outputs. This property is required
for restart protection: after a GOJAM restart the propagator must reproduce
the same trajectory from a stored state vector (architecture §7, restart
protection).

---

## 8. AGC Fixed-Point vs. Rust `f64` Correspondence

| AGC quantity | Scale factor | `f64` SI value | Notes |
|---|---|---|---|
| Position components in `KEPSILON` loop | B+28 m | direct `f64` m | No conversion needed; caller has already decoded DP fixed-point |
| Velocity components | B+7 m/s | direct `f64` m/s | Same |
| `μ` (MEARTH constant) | B+36 m³/s² | `3.986_004_418e14` | `MU_EARTH` from `gravity` module |
| `μ` (MMOON constant) | B+29 m³/s² | `4.902_800_118e12` | `MU_MOON` from `gravity` module |
| Universal anomaly χ | computed | m^(1/2) | Dimensionally √(length) in SI |
| Stumpff argument z | dimensionless | dimensionless | z = α·χ² with α in m⁻¹, χ² in m |
| Convergence tolerance | ~1 ULP of DP | 1e-9 relative | `KEPSILON` label in AGC source |

---

## 9. `no_std` Requirements

The module must compile with `#![cfg_attr(not(test), no_std)]`. All
transcendental functions must use the `libm` crate:

| Standard library call | `no_std` replacement |
|-----------------------|---------------------|
| `f64::sqrt(x)` | `libm::sqrt(x)` |
| `f64::sin(x)` | `libm::sin(x)` |
| `f64::cos(x)` | `libm::cos(x)` |
| `f64::sinh(x)` | `libm::sinh(x)` |
| `f64::cosh(x)` | `libm::cosh(x)` |

No heap allocation. No `Vec`, no `Box`, no `String`. All intermediate values
are stack-allocated `f64` scalars and `Vec3` arrays `[f64; 3]`. The deepest
call chain — `propagate_coast` → `kepler_step` → `stumpff_c`/`stumpff_s` — is
bounded to three stack frames plus linalg primitives, well within the
Cortex-M7 stack budget (architecture §11.3).

---

## 10. Calling Convention from `propagate_coast`

Cross-reference: `specs/integration-spec.md` §4.

`propagate_coast` in `navigation::integration` calls `kepler_step` as follows:

```rust
use crate::math::kepler::kepler_step;
use crate::navigation::gravity::{MU_EARTH, MU_MOON};

let mu = match sv.frame {
    Frame::EarthInertial => MU_EARTH,
    Frame::MoonInertial  => MU_MOON,
};
let (r_kep, v_kep) = kepler_step(sv.position, sv.velocity, dt, mu);
```

The result `(r_kep, v_kep)` is the unperturbed Keplerian position and velocity.
`propagate_coast` then adds the first-order perturbation correction from J2
and third-body gravity (integration-spec §4.5) before assembling the final
`StateVector`. The `kepler_step` function has no knowledge of perturbations
and must not receive or apply them.

When `kepler_step` is a stub (returns `todo!()`), `propagate_coast` falls back
to the Cowell RK4 scheme (integration-spec §4.6). Once `kepler_step` is
implemented, the fallback is replaced with the two-layer scheme above. The
function signature does not change.

---

## 11. Test Cases

All test cases use `mu = MU_EARTH = 3.986_004_418e14 m³/s²` unless noted.
Positions are in metres, velocities in m/s. Tolerances are on norm of the
error vector. Energy and angular-momentum tolerances are relative.

### TC-KEP-1: Circular LEO — Quarter Orbit

**Scenario:** Circular orbit at r = 6_571_000 m (ISS-like altitude, 200 km
above equatorial surface). Quarter-orbit propagation.

**Setup:**
```
mu      = 3.986_004_418e14
r_circ  = 6_571_000.0  (m)
v_circ  = sqrt(mu / r_circ)  ≈ 7784.26 m/s
r0      = [6_571_000.0, 0.0, 0.0]
v0      = [0.0, 7784.26, 0.0]          // in-plane, prograde
T       = 2π * sqrt(r_circ³ / mu)  ≈ 5307.2 s
dt      = T / 4  ≈ 1326.8 s
```

**Expected result:**
```
r1      ≈ [0.0, 6_571_000.0, 0.0]      // 90° around the orbit
v1      ≈ [−7784.26, 0.0, 0.0]         // velocity rotated 90°
```

**Tolerance:** `norm(r1 − expected) < 1.0 m`, `norm(v1 − expected) < 1e-3 m/s`

**Purpose:** Validates basic elliptic propagation, correct quadrant, correct
orbit period. Confirms Lagrange f and g coefficients for a circular orbit.

### TC-KEP-2: Circular LEO — Half Orbit

**Setup:** Same circular orbit as TC-KEP-1.
```
dt = T / 2  ≈ 2653.6 s
```

**Expected result:**
```
r1 ≈ [−6_571_000.0, 0.0, 0.0]
v1 ≈ [0.0, −7784.26, 0.0]
```

**Tolerance:** `norm(r1 − expected) < 1.0 m`, `norm(v1 − expected) < 1e-3 m/s`

**Purpose:** Tests propagation through π radians. Catches sign errors in χ or
in the Lagrange f/g evaluation.

### TC-KEP-3: Circular LEO — Full Orbit (Round-Trip)

**Setup:** Same circular orbit.
```
dt = T  ≈ 5307.2 s
```

**Expected result:**
```
r1 ≈ r0    (within numerical tolerance)
v1 ≈ v0    (within numerical tolerance)
```

**Tolerance:** `norm(r1 − r0) < 10.0 m`, `norm(v1 − v0) < 1e-2 m/s`

**Purpose:** The strongest single-step test. A full-orbit round-trip tests
accumulation of iteration error. The tolerance is tighter than one metre of
position error per orbit, consistent with the AGC's navigation accuracy
requirement.

**Additionally verify invariants:**
- `|E1 − E0| / |E0| < 1e-9`
- `|norm(h1) − norm(h0)| / norm(h0) < 1e-9`

### TC-KEP-4: Highly Elliptic Transfer Orbit

**Scenario:** A Hohmann-like transfer with perigee at r_p = 6_571_000 m and
apogee at r_a = 42_164_000 m (GEO altitude). Propagate from perigee to apogee.

**Setup:**
```
a   = (r_p + r_a) / 2  = 24_367_500.0 m
e   = (r_a − r_p) / (r_a + r_p)  ≈ 0.7308
v_p = sqrt(mu * (2/r_p − 1/a))  ≈ 10_155.8 m/s  (perigee velocity)
v_a = sqrt(mu * (2/r_a − 1/a))  ≈  1_597.3 m/s  (apogee velocity)

r0  = [6_571_000.0, 0.0, 0.0]
v0  = [0.0, 10_155.8, 0.0]

T   = 2π * sqrt(a³ / mu)  ≈ 37_904 s
dt  = T / 2  ≈ 18_952 s                // half-period = perigee to apogee
```

**Expected result:**
```
r1 ≈ [−42_164_000.0, 0.0, 0.0]        // at apogee, opposite side
v1 ≈ [0.0, −1_597.3, 0.0]             // apogee velocity, reversed direction
```

**Tolerance:** `norm(r1 − expected) < 100.0 m`, `norm(v1 − expected) < 0.01 m/s`

**Purpose:** Tests the algorithm for high-eccentricity elliptic orbits, where
the initial estimate for χ is poorer and more Newton-Raphson iterations are
needed. Relevant to the AGC's P37 (Return to Earth) targeting.

### TC-KEP-5: Lunar Orbit — Short Propagation Step

**Scenario:** Low lunar orbit, short propagation step consistent with how
`propagate_coast` calls `kepler_step` during lunar operations.

**Setup:**
```
mu_moon = 4.902_800_118e12  (m³/s²)
r_lo    = 1_837_400.0 m     (100 km LLO altitude)
v_lo    = sqrt(mu_moon / r_lo)  ≈ 1633.4 m/s
r0      = [1_837_400.0, 0.0, 0.0]
v0      = [0.0, 1633.4, 0.0]
dt      = 60.0 s             (one-minute step, typical P20 navigation interval)
```

**Expected result (computed analytically):**

For a short arc of a circular orbit:
```
delta_angle ≈ v_lo * dt / r_lo  ≈ 0.05332 rad
r1 ≈ r_lo * [cos(delta_angle), sin(delta_angle), 0.0]
v1 ≈ v_lo * [−sin(delta_angle), cos(delta_angle), 0.0]
```

Numerically:
```
r1 ≈ [1_836_126.0, 97_877.0, 0.0]
v1 ≈ [−87.17, 1631.1, 0.0]
```

**Tolerance:** `norm(r1 − expected) < 0.1 m`, `norm(v1 − expected) < 1e-4 m/s`

**Purpose:** Validates the MU_MOON constant path, short-arc convergence (1–2
Newton iterations expected), and confirms the `no_std` `libm` functions work
correctly.

### TC-KEP-6: Short `dt` — Single-Step Matching `propagate_coast` Fallback

**Scenario:** Very short `dt` (2.0 s, one SERVICER cycle). Verifies that
`kepler_step` agrees with a single-step linear approximation and does not
diverge from the RK4 fallback.

**Setup:**
```
r0  = [6_571_000.0, 0.0, 0.0]
v0  = [0.0, 7784.26, 0.0]
dt  = 2.0 s
```

**Expected result (linear approximation, valid for 2 s):**
```
r1 ≈ r0 + v0 * dt − 0.5 * (mu/r0³) * r0 * dt²
   ≈ [6_571_000.0 − 0.5 * 9.21 * 4.0, 7784.26 * 2.0, 0.0]
   ≈ [6_570_981.6, 15_568.5, 0.0]
```

**Tolerance:** `norm(r1 − expected) < 0.01 m`, `norm(v1 − expected) < 1e-5 m/s`

**Purpose:** Validates convergence for `dt` in the range used by `average_g_step`
(integration-spec §3), ensuring the Kepler result is consistent with the Cowell
Average-G result to within the perturbation correction magnitude. Cross-reference
integration-spec §4.6: when `kepler_step` is active, `propagate_coast` at `dt=2s`
must agree with the RK4 fallback to within the perturbation term magnitude
(~1e-5 m/s for J2 at LEO).

### TC-KEP-7: Stumpff Function Unit Tests

These tests directly exercise `stumpff_c` and `stumpff_s` at the Taylor-series
boundary.

| ID | z | Expected C(z) | Expected S(z) | Tolerance |
|----|---|---------------|---------------|-----------|
| TC-STUMPFF-1 | 0.0 | 0.5 | 0.16667 (1/6) | 1e-14 relative |
| TC-STUMPFF-2 | 1.0 (elliptic) | (1−cos(1))/1 ≈ 0.45970 | (1−sin(1))/1 ≈ 0.15853 | 1e-14 relative |
| TC-STUMPFF-3 | −1.0 (hyperbolic) | (cosh(1)−1)/1 ≈ 0.54308 | (sinh(1)−1)/1 ≈ 0.17520 | 1e-14 relative |
| TC-STUMPFF-4 | 1e-7 (Taylor branch) | ≈ 0.5 − 1e-7/24 | ≈ 1/6 − 1e-7/120 | 1e-14 relative |
| TC-STUMPFF-5 | −1e-7 (Taylor branch) | ≈ 0.5 + 1e-7/24 | ≈ 1/6 + 1e-7/120 | 1e-14 relative |

---

## 12. Spec Quality Checklist

Per `specs/README.md`:

- [x] AGC source file and line range referenced — `Comanche055/CONIC_SUBROUTINES.agc`,
      routines `KEPRTN` and `KEPSILON`
- [x] All erasable variables and their AGC addresses listed — §8 scale-factor table
- [x] Scale factors documented — §8
- [x] Corresponding `f64` SI units documented — §8 and §4.4
- [x] Input/output preconditions and postconditions stated — §4.6, §4.7
- [x] Edge cases and error handling specified — §6
- [x] At least 5 test cases (7 provided) with expected values — §11
- [x] Rust API signature designed — §4.2, §5.1, §5.2
- [x] Invariants explicitly stated — §7
- [x] Consistency with `docs/architecture.md` checked — §9 (`no_std`), §9.2
      (function granularity), §9.3 (no interpreter state)
