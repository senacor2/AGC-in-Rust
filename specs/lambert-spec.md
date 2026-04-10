# Specification: `math/lambert` Module

**Status**: Ready for implementation  
**Module path**: `agc-core/src/math/lambert.rs`  
**Re-export path**: `agc-core/src/guidance/lambert.rs` (thin re-export, no additional logic)  
**Architecture reference**: `docs/architecture.md` §9 "Navigation Math — Replacing the Interpreter", §9.2 "Function Granularity"  
**Math reference**: `specs/linalg-spec.md` §4 (dot, cross, norm, unit, vscale, vadd, vsub)  
**Types reference**: `specs/types-module-spec.md` §3.3 "Vectors"; `agc-core/src/types/vector.rs`  
**Gravity constants**: `specs/gravity-spec.md` §2.2 (MU_EARTH = 3.986_004_418 × 10¹⁴ m³/s², MU_MOON = 4.902_800_118 × 10¹² m³/s²)  
**Kepler reference**: `specs/kepler-spec.md` (being specced in parallel; both modules share the universal-variable / Battin framework)  
**AGC source**: `Comanche055/CONIC_SUBROUTINES.agc` — Lambert targeting section  
**Callers**: `guidance/targeting.rs` (`compute_delta_v`), `programs/p30.rs`, `programs/p31_p34.rs`  
**Spec checklist**: `specs/README.md` — all items addressed (see §12)

---

## 1. Purpose and Scope

`math::lambert` solves Lambert's problem: given two position vectors **r₁** and
**r₂** in the same inertial frame and a transfer time `tof`, find the unique pair
of velocities **(v₁, v₂)** such that a Keplerian arc connects **r₁** to **r₂**
in exactly `tof` seconds.

The result (v₁, v₂) is the foundation of all Comanche055 maneuver targeting.
The `guidance::targeting::compute_delta_v` function calls `lambert` to convert
a target state vector and a time-of-ignition into a delta-V burn vector. The
programs P30, P31, P32, P33, and P34 all flow through this path.

### What this module provides

- `lambert(r1, r2, tof, mu, prograde) -> (Vec3, Vec3)` — the single public
  function implementing the complete single-revolution Lambert solver.

### What this module does NOT provide

- Multi-revolution solutions. Comanche055 used single-revolution transfers
  exclusively; multi-rev logic is out of scope and must not be added.
- Trajectory propagation. That is `math::kepler::kepler_step` /
  `navigation::conics`.
- Delta-V computation. Subtracting the current velocity from v₁ is the
  caller's responsibility (`guidance::targeting`).
- Epoch management or frame conversion. All inputs and outputs are in the
  same inertial frame (ECI or MCI as appropriate); the caller is responsible
  for frame consistency.

---

## 2. AGC Background

### 2.1 Lambert Targeting in Comanche055

The original Comanche055 Lambert solver lived in `CONIC_SUBROUTINES.agc`. It
was invoked from the rendezvous guidance programs (P31–P34) and the External
Delta-V program (P30) via the AGC interpretive language. The entry point is
commonly labelled `RVIO` or `RVSIO` in AGC assembly mnemonics, with the actual
iterative loop calling the Kepler propagator (`KEPSILON`) at each iteration to
evaluate the time-of-flight residual.

The algorithm structure in the AGC source matches the **bounded Newton
iteration** described in Battin (1987, §6.3): a scalar free variable is iterated
to match the desired transfer time, with the orbit geometry reconstructed from
the Lagrange coefficients f and g at each step. This is equivalent in structure
to Gooding's (1990) reformulation and to the Lancaster–Blanchard approach, but
the Comanche055 version is organized around the AGC interpretive variable set
and its DP fixed-point arithmetic.

AGC scale factors relevant to this computation:
| Quantity | AGC symbol | AGC scale | SI unit (Rust `f64`) |
|---|---|---|---|
| Position component | `RN`, `RN1` | B+28 m | m |
| Velocity component | `VN`, `VN1` | B+7 m/s | m/s |
| Time of flight | `TIG`, `TLAND` | B+17 s (centiseconds × 2⁻¹⁴) | s |
| Gravitational parameter | `MEARTH` | B+36 m³/s² | m³/s² |
| Universal variable χ (chi) | internal | B+17 | dimensionless (m^0.5 in Battin notation) |
| Lagrange coefficient f | internal | B+0 (dimensionless) | dimensionless |
| Lagrange coefficient g | internal | B+0 s | s |

In the Rust port all values are plain `f64` in SI units. No scale-factor
arithmetic is required.

### 2.2 The AGC Iteration Strategy

The AGC used a bounded scalar iteration on the universal variable χ (chi):

1. An initial estimate of χ was computed from the semi-major axis estimate
   derived from the vis-viva equation at the midpoint of the chord.
2. The time-of-flight residual `Δt = tof - t(χ)` was evaluated using the
   Lagrange f and g coefficients, which are closed-form functions of χ and the
   Stumpff functions C(ψ), S(ψ).
3. A Newton step `Δχ = Δt / (∂t/∂χ)` was applied.
4. Convergence was checked against a tolerance of approximately 1 centisecond
   (the AGC's time resolution) — equivalent to 0.01 s in SI.
5. The iteration was bounded to a maximum of 10–12 steps. If the residual did
   not converge, the AGC set an alarm and the crew was presented with a
   PROGRAM ALARM; targeting was aborted.

The transfer-plane normal was selected before iteration using the sign of the
cross product r₁ × r₂ relative to the orbit angular momentum to choose between
prograde and retrograde (short-way and long-way) solutions.

---

## 3. Algorithm Choice and Justification

### 3.1 Chosen Method: Izzo's Algorithm (2015)

The Rust implementation uses **Izzo's method** (Dario Izzo, "Revisiting
Lambert's Problem," *Celestial Mechanics and Dynamical Astronomy*, 121, 1–15,
2015). Izzo's method was chosen over the Battin/universal-variable approach for
the following reasons:

| Criterion | Battin/universal variable | Izzo (2015) |
|---|---|---|
| Robustness near 180° transfer | Requires special-case branching; the AGC had explicit 180° guard logic | Handles all transfer angles in a single code path |
| Convergence speed | 5–15 Newton iterations typically | Halley's method; typically 3–5 iterations |
| Numerical conditioning | Lagrange g → 0 near 180° requires regularization | Parametrized in terms of x ∈ (−1, 1); no singularity at 180° |
| Code complexity | Medium (Stumpff functions, universal variable) | Low; closed-form initial guess, one-variable Halley iteration |
| Literature availability | Battin (1987); widely implemented | Izzo (2015); open reference implementation available |
| Multi-rev extension | Possible but complex | Clean parameter; single-rev is x ∈ (−1, 1) |

The fidelity obligation (architecture §1, "navigation errors kill people") is
satisfied because both methods are mathematically equivalent formulations of
the same two-body boundary-value problem; they produce the same (v₁, v₂) to
within floating-point rounding. Izzo's method is more robust near the
degenerate 180° case that arises in Hohmann-like transfers.

The structural correspondence to the AGC source is preserved at the level of
the public API: the function signature and the prograde/retrograde selection
logic faithfully replicate the AGC's behavior.

### 3.2 Algorithm Outline (Izzo 2015)

Given: r₁, r₂, tof, μ, `prograde`

**Step 1 — Transfer geometry**

```
r1_mag = ‖r₁‖
r2_mag = ‖r₂‖
cos_dnu = dot(r₁, r₂) / (r1_mag × r2_mag)
dnu      = acos(clamp(cos_dnu, −1, 1))        // transfer angle in [0, π]
```

The transfer angle `dnu` is disambiguated for the short-way / long-way choice
using the sign of the z-component of `cross(r₁, r₂)`:

```
// z_cross > 0 means r₂ is "ahead of" r₁ in the prograde direction
z_cross = cross(r₁, r₂)[2]          // z-component

// For prograde (short-way), we want the arc that goes in the
// direction of positive angular momentum (z > 0 in ECI).
// For retrograde (long-way), we use the complementary arc.
if prograde && z_cross < 0.0 {
    dnu = 2π − dnu                   // long arc, prograde orbit
}
if !prograde && z_cross >= 0.0 {
    dnu = 2π − dnu                   // retrograde: flip
}
```

**Step 2 — Non-dimensional parameters**

```
c   = sqrt(r1_mag² + r2_mag² − 2 × r1_mag × r2_mag × cos(dnu))  // chord length
s   = (r1_mag + r2_mag + c) / 2                                   // semi-perimeter
λ²  = 1 − c / s
λ   = sqrt(λ²)                              // Izzo's λ ∈ [0, 1]
if dnu > π { λ = −λ }                       // sign convention

T   = tof × sqrt(2μ / s³)                   // non-dimensional time
```

**Step 3 — Initial guess for x**

Izzo's parameter x ∈ (−1, 1) is related to the semi-major axis. The closed-form
initial guess (from Izzo 2015, Eq. 20–22) reads:

```
// Compute T at x=0 (parabolic reference) and x=1 boundary
T00 = acos(λ) + λ × sqrt(1 − λ²)          // T at x → 0 (parabolic)
T1  = (2/3) × (1 − λ³)                    // T at x = 1 (minimum energy)
x0  = (T00 / T - 1) × if T < T00 { 1.0 } else { -1.0 }
x0  = clamp(x0, −1 + ε, 1 − ε)
```

**Step 4 — Halley iteration on x**

The function `t(x)` (Izzo 2015, Eq. 9) gives the non-dimensional TOF as a
function of x. Iterate using Halley's method:

```
for _ in 0..MAX_ITER {
    (t, dt, d2t) = tof_and_derivatives(x, λ)
    err = t − T
    if |err| < TOL { break }
    x_new = x − err × dt / (dt² − 0.5 × err × d2t)
    x = clamp(x_new, −1 + ε, 1 − ε)
}
```

Where `tof_and_derivatives(x, λ)` returns (T(x), dT/dx, d²T/dx²) using the
Lancaster–Blanchard universal functions as expressed in Izzo (2015), Eq. 11–13.

**Step 5 — Reconstruct velocity vectors**

Once x converges, recover the Lagrange coefficients:

```
γ  = sqrt(μ × s / 2)
ρ  = (r1_mag − r2_mag) / c
σ  = sqrt(1 − ρ²)

y  = sqrt(1 − λ² + λ² × x²)
vr1 = γ × ((λ × y − x) − ρ × (λ × y + x)) / r1_mag
vt1 = γ × σ × (y + λ × x) / r1_mag
vr2 = −γ × ((λ × y − x) + ρ × (λ × y + x)) / r2_mag
vt2 = γ × σ × (y + λ × x) / r2_mag
```

Then project onto the inertial frame:

```
r1_hat  = unit(r₁)
r2_hat  = unit(r₂)
h_hat   = unit(cross(r₁, r₂))         // transfer-plane normal
t1_hat  = unit(cross(h_hat, r1_hat))  // tangential direction at r₁
t2_hat  = unit(cross(h_hat, r2_hat))  // tangential direction at r₂

v₁ = vr1 × r1_hat + vt1 × t1_hat
v₂ = vr2 × r2_hat + vt2 × t2_hat
```

---

## 4. Public API

### 4.1 Function Signature

```rust
/// Solve Lambert's problem.
///
/// Given initial position `r1` (m), final position `r2` (m), transfer time
/// `tof` (s), and central-body gravitational parameter `mu` (m³/s²), return
/// the departure velocity at `r1` and arrival velocity at `r2` that connect
/// the two points on a single-revolution conic arc.
///
/// `prograde` selects the transfer direction:
/// - `true`  — short-way (prograde): transfer angle < π, angular momentum
///   in the +z hemisphere.  Used for standard orbital transfers.
/// - `false` — long-way (retrograde): transfer angle > π, or the arc that
///   crosses the z = 0 plane from below.  Used for bi-elliptic and
///   retrograde targeting.
///
/// # Panics
///
/// Panics (triggering the restart handler) if:
/// - `r1` and `r2` are collinear and anti-parallel (180° transfer with no
///   defined plane).  See §5.1.
/// - `tof <= 0.0`.
/// - `mu <= 0.0`.
/// - `‖r1‖ == 0` or `‖r2‖ == 0`.
///
/// # Reference
///
/// Izzo, D. (2015). Revisiting Lambert's problem.
/// *Celestial Mechanics and Dynamical Astronomy*, 121(1), 1–15.
/// AGC source: Comanche055/CONIC_SUBROUTINES.agc (Lambert targeting section)
pub fn lambert(r1: Vec3, r2: Vec3, tof: f64, mu: f64, prograde: bool) -> (Vec3, Vec3)
```

### 4.2 Parameters

| Parameter | Type | Unit | Preconditions |
|---|---|---|---|
| `r1` | `Vec3` (`[f64;3]`) | m (ECI or MCI) | `‖r1‖ > 0` |
| `r2` | `Vec3` (`[f64;3]`) | m, same frame as r1 | `‖r2‖ > 0` |
| `tof` | `f64` | s | `> 0`; physically must exceed the minimum transfer time for the given geometry |
| `mu` | `f64` | m³/s² | `> 0`; typically `gravity::MU_EARTH` or `gravity::MU_MOON` |
| `prograde` | `bool` | — | `true` → short-way; `false` → long-way |

### 4.3 Return Value

`(v1, v2)` where:
- `v1: Vec3` — departure velocity at `r1` in m/s, same frame as inputs
- `v2: Vec3` — arrival velocity at `r2` in m/s, same frame as inputs

---

## 5. Convergence, Tolerances, and Iteration Limits

### 5.1 Tolerance

The Halley iteration terminates when the residual in non-dimensional TOF
satisfies:

```
|t(x) − T| < TOL_NDIM
```

where `TOL_NDIM = 1.0e-12` (dimensionless). This corresponds to a dimensional
time error of:

```
Δtof = TOL_NDIM × sqrt(s³ / (2μ))
```

For a typical LEO transfer (s ≈ 8 × 10⁶ m, μ = MU_EARTH):

```
Δtof ≈ 1.0e-12 × sqrt((8e6)³ / (2 × 3.986e14))
     ≈ 1.0e-12 × sqrt(5.12e20 / 7.97e14)
     ≈ 1.0e-12 × 803 s
     ≈ 0.8 ns
```

This is well below the AGC's 0.01 s time resolution and the 1 ms targeting
accuracy required for rendezvous maneuvers.

### 5.2 Maximum Iterations

```rust
const MAX_ITER: usize = 50;
```

Izzo's method converges in 3–7 Halley iterations for all well-posed inputs.
MAX_ITER = 50 is a safety bound; reaching it indicates a degenerate input
that should have been caught by the precondition checks. The implementation
must panic (triggering the restart handler) if `MAX_ITER` is reached without
convergence to within `TOL_NDIM`.

### 5.3 x-Clamp Epsilon

```rust
const X_EPS: f64 = 1.0e-10;
```

`x` is clamped to `[−1 + X_EPS, 1 − X_EPS]` to avoid the parabolic and
minimum-energy singularities at the boundary.

---

## 6. Transfer-Angle and Prograde/Retrograde Decision

The `prograde` flag maps directly to the AGC's CONIC_SUBROUTINES transfer-angle
selection. The AGC computed the transfer angle from the cross product of r₁ and
r₂ relative to the expected angular-momentum direction of the target orbit.

In the Rust implementation:

```
h_sign = cross(r₁, r₂)[2]   // z-component of orbit normal
```

| `prograde` | `h_sign` | Effective `dnu` | Physical meaning |
|---|---|---|---|
| `true` | ≥ 0 | computed directly | Short-way prograde (most LEO/TLI transfers) |
| `true` | < 0 | `2π − dnu` | Short-way prograde from below the equatorial plane |
| `false` | ≥ 0 | `2π − dnu` | Long-way retrograde |
| `false` | < 0 | computed directly | Long-way below equatorial plane |

This four-quadrant logic faithfully reproduces the AGC's transfer-angle
selection for all mission phases (LEO rendezvous, TLI, lunar orbit, TEI).

Note: The z-component heuristic is valid for both ECI and MCI frames because
both frames share the same pole direction (axes parallel). If the transfer is
nearly equatorial (h_sign ≈ 0) and `dnu ≈ π`, the result is geometrically
degenerate (§7.1) and the caller must avoid this input.

---

## 7. Edge Cases and Error Handling

### 7.1 180° Transfer (Degenerate Plane)

When r₁ and r₂ are anti-parallel (`dot(r₁, r₂) / (r1_mag × r2_mag) ≈ −1`),
`cross(r₁, r₂) ≈ 0` and the transfer plane is undefined. No unique conic arc
exists without a third constraint (usually the orbit-plane normal).

Detection threshold:

```rust
const COLLINEAR_TOL: f64 = 1.0e-6;  // ‖cross(r1_hat, r2_hat)‖ < this → degenerate
```

Action: **panic**. The restart handler (architecture §7) will restore the last
valid targeting state from the phase table. The calling program (P30/P31/P34)
must guard against this by checking the transfer angle before calling `lambert`.
The programs should display PROGRAM ALARM 0517 (targeting error) in this case.

Rationale: The AGC's own implementation aborted with a program alarm for this
case. The Rust port must do the same; silently returning an arbitrary velocity
would be flight-unsafe.

Note: A near-180° transfer (`dnu = π − ε` for small ε) is NOT degenerate in
Izzo's method. The algorithm's λ parametrization handles this continuously.
The panic is triggered only when the cross product magnitude falls below
`COLLINEAR_TOL`, i.e., when the plane truly cannot be determined.

### 7.2 Very Short Transfer Time (Hyperbolic or Near-Parabolic)

If `tof` is less than the minimum transfer time for the given chord and
semi-perimeter, no solution exists on a bound orbit. In Izzo's formulation,
x approaches 1 and the iteration may not converge or may converge to a
hyperbolic solution (x > 1, which is clamped and thus unphysical).

Detection: If after convergence `|x| >= 1 − X_EPS`, the solution is at the
boundary of the admissible range (hyperbolic transfer). This is physically
valid (hyperbolic trajectories are real) but Comanche055 only used elliptic
and near-parabolic arcs. The implementation returns the boundary solution
without panicking, but callers should be aware that the resulting orbit has
positive specific energy.

### 7.3 Equal Position Vectors (`r₁ ≈ r₂`)

If `‖r₁ − r₂‖ < ε_pos` (where `ε_pos = 1.0` m), the problem degenerates
to a trivial zero-delta arc. **panic** with an informative message. The
threshold of 1 m is well below any physically meaningful targeting separation
in either Earth orbit or lunar orbit.

### 7.4 Multi-Revolution Transfers

Multi-revolution solutions (where the spacecraft completes more than one full
orbit before arriving at r₂) exist for sufficiently large `tof`. They correspond
to x > 1 in Izzo's parametrization and are explicitly out of scope for
Comanche055. The implementation restricts x to (−1, 1), enforcing the
single-revolution constraint. If the caller provides a TOF so large that no
single-revolution solution exists, the iteration will converge to the
minimum-energy single-rev solution (the x = 0 case), which is the closest
valid solution.

### 7.5 Nearly Radial Transfer (`dnu ≈ 0`)

When r₁ and r₂ are nearly parallel (`dnu ≈ 0`), the transfer is almost purely
radial. This is geometrically valid but produces very large velocity magnitudes
for short `tof`. No special-casing is needed; Izzo's method handles it correctly.

---

## 8. Invariants and Postconditions

After a successful return, the following must hold within numerical tolerance:

**I1 — Coplanarity**: Both v₁ and v₂ lie in the transfer plane defined by r₁
and r₂.

```
cross(r₁, v₁) ≈ cross(r₂, v₂)     // same angular momentum vector
```

Tolerance: `‖cross(r₁,v₁) − cross(r₂,v₂)‖ / ‖cross(r₁,v₁)‖ < 1.0e-9`

**I2 — Energy consistency**: Specific orbital energy is the same at both
endpoints.

```
E₁ = 0.5 × ‖v₁‖² − μ / ‖r₁‖
E₂ = 0.5 × ‖v₂‖² − μ / ‖r₂‖
|E₁ − E₂| / |E₁| < 1.0e-9
```

**I3 — Time of flight**: Propagating from (r₁, v₁) for `tof` seconds with the
Kepler solver must yield a position within tolerance of r₂.

```
(r_prop, _) = kepler_step(r₁, v₁, tof, μ)
‖r_prop − r₂‖ < 1.0 m      // for tof up to ~12 hours (LEO/lunar)
```

**I4 — Angular momentum direction**: The cross product `cross(r₁, v₁)` must
point in the expected hemisphere given the `prograde` flag.

```
h_z = cross(r₁, v₁)[2]
assert!(if prograde { h_z >= 0.0 } else { h_z <= 0.0 })
```

(This assertion is relaxed for nearly equatorial transfers where |h_z| < 1e-6.)

---

## 9. Internal Constants

Declare the following constants at the top of `math/lambert.rs`:

```rust
/// Non-dimensional TOF convergence tolerance for Halley iteration.
const TOL_NDIM: f64 = 1.0e-12;

/// Maximum Halley iterations before panic.
const MAX_ITER: usize = 50;

/// Boundary epsilon for x clamping.
const X_EPS: f64 = 1.0e-10;

/// Cross-product magnitude threshold for collinearity detection (anti-parallel vectors).
const COLLINEAR_TOL: f64 = 1.0e-6;

/// Minimum position separation (m) below which r1 ≈ r2 is treated as degenerate.
const MIN_SEPARATION_M: f64 = 1.0;
```

---

## 10. Relation to `math/kepler` and `guidance/targeting`

### 10.1 Kepler Solver (`math::kepler::kepler_step`)

`lambert` and `kepler_step` are the two halves of the conic two-body problem:

- `kepler_step`: **initial-value problem** — given (r, v) at t₀, find (r, v) at t₀ + dt.
- `lambert`: **boundary-value problem** — given r₁, r₂, tof, find (v₁, v₂).

Both use related mathematical machinery (Stumpff functions, universal variable,
semi-major axis). They do not call each other; Invariant I3 above uses
`kepler_step` only in tests, never in the production code path.

Both modules share the Battin/universal-variable conceptual framework from the
AGC source, even though the Rust implementations use Izzo's reformulation for
`lambert` and a direct universal-variable scheme for `kepler_step`.

### 10.2 `guidance::targeting::compute_delta_v`

The canonical call chain for maneuver targeting is:

```
programs/p31_p34.rs
  → guidance::targeting::compute_delta_v(current_sv, target_sv, tig, mu)
      → math::lambert::lambert(r_current_at_tig, r_target_at_tig, tof, mu, prograde)
          returns (v1_transfer, v2_arrival)
      → delta_v = v1_transfer − v_current_at_tig
```

The `guidance::lambert` module re-exports `math::lambert::lambert` without
modification. The guidance layer's role is to:
1. Propagate `current_sv` to `tig` using `kepler_step` to obtain r₁ and the
   current velocity at TIG.
2. Propagate `target_sv` to the estimated arrival time to obtain r₂.
3. Compute `tof = arrival_time − tig`.
4. Call `lambert(r1, r2, tof, mu, prograde)` to get v₁.
5. Return `delta_v = v1 − current_velocity_at_tig`.

---

## 11. Test Cases

All test cases are unit tests in `math/lambert.rs` under `#[cfg(test)]`.
Numerical expected values are computed from a reference implementation
(Python `poliastro` / `izzo` solver) and verified against Vallado (2013)
*Fundamentals of Astrodynamics and Applications* where tabulated.

### Test 1 — Circular LEO Hohmann Transfer (180°)

Transfer from 400 km circular orbit to 800 km circular orbit.

```
μ  = MU_EARTH = 3.986_004_418e14 m³/s²

r1 = [6_778_000.0, 0.0, 0.0]   m  (perigee of 400 km orbit, in +X direction)
r2 = [0.0, 7_178_000.0, 0.0]   m  (apogee of 800 km orbit, in +Y direction)
```

Wait — this is a 90° transfer, not 180°. For a true Hohmann (180°):

```
r1 = [6_778_000.0,  0.0, 0.0]  m
r2 = [−7_178_000.0, 0.0, 0.0]  m   (anti-parallel, 180° apart)
```

This is the degenerate case (§7.1). The Hohmann-like test uses 90° geometry
(Test 2). A pure 180° Hohmann is unresolvable without a plane; Tests 1 and 2
below use geometrically distinct near-Hohmann cases.

**Test 1a — 90° LEO to MEO**

```
r1     = [6_778_000.0, 0.0,        0.0]   m
r2     = [0.0,         7_578_000.0, 0.0]  m   (1200 km altitude)
prograde = true

// Transfer orbit semi-major axis (half sum for Hohmann-like): ~7178 km
// Estimate tof ≈ half period of transfer ellipse with a = 7178 km
a_transfer ≈ (6_778_000 + 7_578_000) / 2 = 7_178_000 m
T_transfer  = 2π × sqrt(a_transfer³ / μ) ≈ 6044 s
tof         = T_transfer / 4  ≈ 1511 s  (quarter ellipse for 90° transfer)
```

Expected v₁ ≈ [0.0, 7784 ± 10, 0.0] m/s (prograde tangential at r₁)  
Expected v₂ ≈ [−7353 ± 10, 0.0, 0.0] m/s (prograde tangential at r₂)  
Verify Invariant I2: |E₁ − E₂| / |E₁| < 1.0e-9  
Verify Invariant I3: `kepler_step(r₁, v₁, tof, μ)` → r_prop; `‖r_prop − r₂‖ < 1.0 m`

**Test 1b — Exact Circular-Circular 90° Transfer (Vallado Example 7-5)**

Use Vallado (2013) §7.6 example with tabulated values to validate absolute
accuracy. Inputs and expected outputs must be taken from the reference and
hardcoded in the test.

### Test 2 — LEO Rendezvous (Short Transfer)

Two spacecraft in the same orbital plane, one 50 km behind the other.

```
μ       = MU_EARTH
r1      = [6_778_000.0, 0.0, 0.0]           m  (chaser at perigee)
r2      = [6_778_000.0 × cos(0.3°),
           6_778_000.0 × sin(0.3°), 0.0]    m  (target, 0.3° ahead)
tof     = 300.0   s   (short transfer, ~ 5 minutes)
prograde = true
```

Expected: v₁ will be slightly above circular velocity; v₂ will arrive at r₂.  
Verify Invariant I3 rigorously (‖r_prop − r₂‖ < 0.1 m given the short arc).

### Test 3 — Trans-Lunar Injection (TLI-like)

Approximate Apollo TLI trajectory: depart from 185 km parking orbit, arrive
at the lunar SOI boundary (~66,200 km altitude ≈ 384,400 km from Earth center
minus 66,200 km, but for this test use a simplified cislunar arc).

```
μ       = MU_EARTH
r1      = [6_563_000.0, 0.0, 0.0]            m  (185 km parking orbit, perigee)
r2      = [−1.50e8,     3.5e7,   0.0]         m  (approximate cislunar waypoint)
tof     = 259_200.0  s   (3 days)
prograde = true
```

Expected: v₁ ≈ [0, ~10_900, 0] m/s (hyperbolic departure speed at parking orbit perigee)  
Verify: specific energy E = 0.5 × ‖v₁‖² − μ/r1_mag > 0 (escape trajectory)  
Verify Invariant I3 with loose tolerance (‖r_prop − r₂‖ < 100 m; Kepler propagation over 3 days accumulates numerical error).

### Test 4 — Lunar Orbit Transfer

Departure from 100 km LLO, arrival at a 200 km LLO apse (phasing transfer).

```
μ       = MU_MOON = 4.902_800_118e12 m³/s²
r1      = [1_837_400.0, 0.0, 0.0]    m  (100 km LLO perigee; R_Moon ≈ 1737.4 km)
r2      = [0.0,         1_937_400.0, 0.0]   m  (200 km LLO apse, 90° away)
tof     = 1800.0   s
prograde = true
```

Expected: v₁ ≈ [0, ~1700, 0] m/s (low lunar orbit velocity)  
Verify Invariants I1, I2, I3.

### Test 5 — Retrograde (Long-Way) Transfer

Same geometry as Test 1a but with `prograde = false`, forcing the long-way arc.

```
r1      = [6_778_000.0, 0.0,        0.0]   m
r2      = [0.0,         7_578_000.0, 0.0]  m
prograde = false
// tof for long-way arc is T_transfer × 3/4 ≈ 4533 s
tof     = 4533.0   s
```

Expected: `cross(r₁, v₁)[2] < 0` (retrograde, angular momentum points −z)  
Verify Invariants I1, I2, I3.  
Verify that v₁ magnitude is greater than for the prograde case (longer arc → more energy).

### Test 6 — Degenerate: Anti-Parallel Vectors (180°, should panic)

```rust
#[test]
#[should_panic]
fn test_lambert_180_degenerate() {
    let r1 = [6_778_000.0, 0.0, 0.0];
    let r2 = [−6_778_000.0, 0.0, 0.0];
    lambert(r1, r2, 2700.0, MU_EARTH, true);
}
```

### Test 7 — Degenerate: Zero Separation (should panic)

```rust
#[test]
#[should_panic]
fn test_lambert_zero_separation() {
    let r1 = [6_778_000.0, 0.0, 0.0];
    let r2 = [6_778_000.0, 0.0, 0.0];
    lambert(r1, r2, 1000.0, MU_EARTH, true);
}
```

---

## 12. Spec Quality Checklist

- [x] AGC source file referenced: `Comanche055/CONIC_SUBROUTINES.agc` (§2.1)
- [x] All erasable variables and AGC addresses: Table in §2.1 (RN, RN1, TIG, MEARTH; AGC scale factors)
- [x] Scale factors documented for all fixed-point values: §2.1 table (B+28 m, B+7 m/s, B+36 m³/s², B+17 s)
- [x] Corresponding `f64` SI units documented: §2.1 table and §4.2
- [x] Input/output preconditions and postconditions: §4.2 (inputs), §8 (postconditions / invariants)
- [x] Edge cases and error handling: §7 (six edge cases)
- [x] At least 3 test cases (7 provided): §11 (Tests 1a, 1b, 2, 3, 4, 5, 6, 7)
- [x] Rust API signature designed: §4.1
- [x] Invariants explicitly stated: §8 (I1–I4)
- [x] Consistency with `docs/architecture.md` checked: §9 ADR-001 (no interpreter), §9.2 (function granularity), types match `Vec3 = [f64; 3]`

---

## 13. References

1. Izzo, D. (2015). Revisiting Lambert's problem. *Celestial Mechanics and
   Dynamical Astronomy*, 121(1), 1–15. DOI: 10.1007/s10569-014-9587-y

2. Battin, R.H. (1987). *An Introduction to the Mathematics and Methods of
   Astrodynamics*. AIAA Education Series. §6.3 (Lambert's theorem), §6.4
   (universal variable formulation).

3. Gooding, R.H. (1990). A procedure for the solution of Lambert's orbital
   boundary-value problem. *Celestial Mechanics and Dynamical Astronomy*,
   48(2), 145–165.

4. Vallado, D.A. (2013). *Fundamentals of Astrodynamics and Applications*,
   4th ed. Microcosm Press. §7.6 (Lambert's problem examples).

5. `Comanche055/CONIC_SUBROUTINES.agc` — original AGC assembly source,
   Lambert targeting section (RVIO/RVSIO entry points and iteration loop).

6. `docs/architecture.md` §9 — Navigation Math strategy for the Rust port.

7. `specs/gravity-spec.md` §2.2 — MU_EARTH and MU_MOON constants.

8. `specs/linalg-spec.md` §4 — `dot`, `cross`, `norm`, `unit`, `vscale`
   used in §3.2 velocity reconstruction.

---

## Appendix A: Izzo 2015 Formula Reference and Worked Example

This appendix provides a mathematically precise statement of every formula in
the Izzo (2015) algorithm, a fully worked numerical example, and a checklist
of known bug candidates in `math/lambert.rs`. Its purpose is to give a
developer a single document against which to diff the current implementation.

`[VERIFY]` markers indicate places where the exact sign or formula could not
be confirmed with certainty from memory of the paper and should be cross-checked
against Izzo (2015) §2–§3 before trusting the implementation.

---

### A.1 Non-dimensionalization

All scalar quantities are non-dimensionalized using the semi-perimeter `s` as
the length scale and a derived time unit. The non-dimensionalization is applied
before the Halley iteration and undone during velocity reconstruction.

**Chord length** (straight-line distance between the two endpoints):

```
c = sqrt(r1² + r2² - 2·r1·r2·cos(Δν))
```

where r1 = |r₁|, r2 = |r₂|, and Δν is the transfer angle. For the 90° case
this simplifies to `c = sqrt(r1² + r2²)`.

**Semi-perimeter** of the triangle formed by r₁, r₂, and the chord:

```
s = (r1 + r2 + c) / 2
```

**Izzo's λ parameter**:

```
λ² = 1 - c/s       (always ≥ 0; equals 0 for 180° transfer, 1 for 0° transfer)
|λ| = sqrt(λ²)
```

The sign of λ encodes the transfer direction. Izzo (2015) uses λ **signed**
throughout the paper, including inside the T(x,λ) formula. This is the most
common source of sign bugs — some implementations store |λ| and separately
track the sign, which then must be re-applied in the correct places.

```
λ > 0   for Δν < π   (short-way / prograde arc)
λ < 0   for Δν > π   (long-way / retrograde arc)
```

Equivalently, using the orbit-plane cross product: `sign(λ) = sign((r₁×r₂)·ẑ)`
when `prograde = true`, and `sign(λ) = -sign((r₁×r₂)·ẑ)` when `prograde =
false`. In the implementation the sign is assigned after computing Δν and
checking whether `dnu > π`.

**Non-dimensional time of flight**:

```
T = tof · sqrt(2μ / s³)
```

Dimensions: `T` is dimensionless. The factor `sqrt(2μ/s³)` has units of s⁻¹.

**Reference non-dimensional times** used for the initial guess:

```
T₀₀ = acos(|λ|) + |λ|·sqrt(1 - λ²)     (T at x = 0, the parabolic reference)
T₁  = (2/3)·(1 - |λ|³)                  (T at x → 1, the minimum-energy ellipse)
```

Note that both T₀₀ and T₁ are defined using `|λ|` (unsigned), regardless of
whether the transfer is prograde or retrograde. This is correct. The sign of λ
enters only inside the T(x,λ) function.

**Verification of the T₀₀ formula at x = 0**: substituting x = 0 into the
Lancaster-Blanchard formula (§A.2 below) gives α = π, β = 2·asin(|λ|), and
after simplification T(0,λ) = acos(λ) + λ·sqrt(1-λ²) if λ is used signed, or
equivalently T(0,|λ|) = acos(|λ|) + |λ|·sqrt(1-λ²). Both give the same
result because the sign of λ does not affect T at x=0 (the even symmetry of
the Lancaster-Blanchard formula). The expression `acos(|λ|) + |λ|·sqrt(1-λ²)`
is therefore correct for T₀₀.

---

### A.2 Time of Flight Equation T(x, λ)

The parameter x ∈ (−1, 1) is related to the semi-major axis of the transfer
orbit. x = 0 is the parabolic transfer; x → −1 is the slow (large-a) limit;
x → +1 is the minimum-energy (minimum-a) limit.

**Auxiliary variable y**:

```
y(x, λ) = sqrt(1 - λ²·(1 - x²))    [always ≥ 0 for |λ| ≤ 1]
```

Note: this can be written equivalently as `sqrt(1 - λ² + λ²·x²)`. Both forms
are used in the literature; they are identical. The current implementation uses
the second form in the velocity reconstruction (§A.6), which is correct.

**Lancaster-Blanchard form (Izzo 2015, Eq. 9 / §2.1)**:

For x ∈ (−1, 1) (elliptic transfer):

```
α = 2·acos(x)                         ∈ (0, 2π)
β = 2·asin(|λ|·sqrt(1 - x²))         ∈ [0, π]    if λ ≥ 0
β = -2·asin(|λ|·sqrt(1 - x²))        ∈ [-π, 0]   if λ < 0

T(x, λ) = [(α - β) - (sin α - sin β)] / (2·a^(3/2))
```

where `a = 1/(1 - x²)` is the non-dimensional semi-major axis (always > 1 for
x ∈ (−1, 1) in this parametrization).

**Important sign note for the β formula**: Izzo's convention is that β carries
the sign of λ. The argument to asin is always the non-negative quantity
`|λ|·sqrt(1-x²)`, and the sign is applied externally:

```
β = sign(λ) · 2·asin(|λ|·sqrt(1 - x²))
```

This matches the implementation at `lambert.rs` lines 255–260. If λ > 0 (short
arc), β > 0 and (α - β) < α, giving a shorter time. If λ < 0 (long arc), β < 0
and (α - β) > α, giving a longer time. The monotonic behavior of T with respect
to λ is correct.

**Alternative Gooding/Izzo form** (equivalent, sometimes numerically preferable):

```
T(x, λ) = (1/(1-x²)) · [E(x,λ)/sqrt(1-x²) - x·(1 - λ²·(x/y))]   [VERIFY form]
```

where `E(x,λ)` is related to the incomplete elliptic integral. The
Lancaster-Blanchard form above is simpler to implement and is what the current
code uses.

**Limiting cases**:

- At x = 0: `T(0,λ) = T₀₀ = acos(|λ|) + |λ|·sqrt(1-λ²)` (see §A.1)
- At x → 1: `T(1,λ) = T₁ = (2/3)·(1 - λ³)` [VERIFY sign: should this use λ or |λ|?]
  - For λ > 0: T₁ = (2/3)·(1 - λ³) > 0. For λ < 0: T₁ = (2/3)·(1 - λ³) > (2/3).
  - The current implementation uses `lambda_abs³` (line 144 in `lambert.rs`),
    computing T₁ = (2/3)·(1 - |λ|³). This gives T₁ < 2/3 always, which is the
    minimum-energy time for the prograde arc geometry. For the retrograde arc
    (λ < 0), the actual minimum-energy T is `(2/3)·(1 + |λ|³)` > 2/3.
    **[VERIFY]** This discrepancy may contribute to TC-LAM-5's divergence:
    the T₁ guard condition `t_nd >= t1` uses the wrong T₁ for retrograde cases,
    placing the initial guess in the wrong regime.

---

### A.3 Derivatives dT/dx and d²T/dx²

Needed for the Halley iteration. Derived from the Lancaster-Blanchard form
(Izzo 2015, Eq. 11–13). Let `a_inv = 1 - x²`, `a = 1/a_inv`.

**First derivative**:

```
T'(x, λ) = dT/dx = [3·x·T(x,λ) - 2 + 2·λ³·x/y] / (1 - x²)
```

where y = y(x,λ) as defined in §A.2. Note:
- The `λ³` term uses the **signed** λ (Izzo's convention), not |λ|³.
- For λ < 0 this term is negative, which correctly modifies the slope for
  retrograde arcs.
- At y ≈ 0 (degenerate: |λ| ≈ 1 and x ≈ 0), the `2·λ³·x/y` term is 0/0.
  The implementation correctly handles this by zeroing the term when y < 1e-14.

**Second derivative**:

```
T''(x, λ) = d²T/dx² = [3·T + (3x - 4/x)·T' + (4/x²)·(T - (2/3)·(1-λ³))] / (1-x²)
```

**[VERIFY]** This formula contains `(2/3)·(1-λ³)` which uses **signed** λ³.
For retrograde (λ < 0), this equals `(2/3)·(1 + |λ|³)`, which is the correct
minimum-energy T₁ for the retrograde geometry (see discussion in §A.2 above).
The current implementation at `lambert.rs` line 290 uses `lam3 = lam2 * lambda`,
where `lambda` is signed, so `lam3` is signed. This means `(2/3)·(1-lam3)` is
computed correctly with the signed λ. However, the T₁ guard condition at line
144 uses `lambda_abs³`, which is INCONSISTENT with the T'' formula's use of
signed lam3. The guard uses the wrong T₁ even if the derivative is correct.

**Singularity at x = 0**: The term `4/x²` diverges. The paper handles this by
recognizing that T''(0,λ) has a finite limit as x → 0, but the closed-form
expression becomes numerically unstable for |x| < ~1e-4. The implementation
uses a five-point central finite difference for |x| < 1e-8, which is correct
in principle but uses a step size h = 1e-5. For |x| = 1e-6 (very near zero),
the finite-difference stencil points at x ± h = ±1e-5 are still well away from
0, so this is safe.

**Singularity at x = 0 for T' as well**: The `4/x` term in T'' involves T'
which itself has a `λ³·x/y` term. At x = 0 this term vanishes and T' simplifies
to `[3·0·T - 2] / (1-0) = -2`, which is the correct finite limit. The
implementation handles this at line 272–274 by dropping the `2·λ³·x/y` term
when y < 1e-14, but it does NOT handle the case x ≈ 0 separately for T'; it
relies on the `x·(λ³/y)` product remaining finite. For small but non-zero x
with λ ≈ 1, y ≈ |x|, so `λ³·x/y ≈ 1`, which is fine.

---

### A.4 Halley Iteration Step

Given the current error `T_err = T(x) - T_required`, the Halley step is:

```
Δx = -T_err · T' / (T'² - 0.5·T_err·T'')
```

This can equivalently be written as:

```
Δx = 2·T_err·T' / (2·T'² - T_err·T'')  ×  (-1)
   = -2·T_err·T' / (2·T'² - T_err·T'')
```

Both forms are mathematically identical. The leading negative sign is correct:
if T(x) > T_required (T_err > 0), we need x to increase (since T is generally
decreasing in x for the elliptic regime), so Δx should be positive when T' < 0,
which requires the minus sign.

**[VERIFY]** The sign depends on whether T is increasing or decreasing in x
for the given regime:
- Elliptic slow arc (x < 0): T is decreasing as x increases toward 0. T' < 0.
  If T_err > 0 (too slow), we need to increase x (move toward minimum energy),
  so Δx > 0. With T' < 0 and T_err > 0: Δx = -T_err·T' / (...) > 0. Correct.
- Elliptic fast arc (x > 0): T is still decreasing as x increases. Same logic.

The denominator fallback to Newton's method (`-T_err/T'`) when the denominator
is near zero is correct.

---

### A.5 Initial Guess x₀

The initial guess selects the regime based on comparing T_nd to T₀₀ and T₁.

**Regime 1: T_nd > T₀₀ (slow arc, x₀ ∈ (−1, 0))**

The current implementation uses:

```
x₀ = T₀₀/T_nd - 1
```

This maps T_nd = T₀₀ → x₀ = 0 and T_nd → ∞ → x₀ → -1. The mapping is
reasonable for moderately long TOF but **may be a poor initial guess for very
large T_nd** (TC-LAM-3: T_nd >> T₀₀). In that regime, x₀ → -1 and the
Lancaster-Blanchard formula is nearly singular.

Izzo (2015, Eq. 23–24) gives the following improved initial guess for T > T₀₀:

```
// Izzo's Eq. 23: hypergeometric-based initial guess for slow arc
A   = ln(T₀₀ / T_nd)          [natural log; always ≤ 0 in this regime]
B   = ln(T₁  / T_nd)          [VERIFY sign: T₁ < T₀₀ so B < A ≤ 0]
x₀  = exp(A · (x̃ - 1))        [VERIFY exact Izzo Eq. 23 form]
```

where x̃ is a shape parameter derived from fitting the T(x) curve. **[VERIFY]**
The exact Izzo formula for the T > T₀₀ branch should be retrieved from the
paper. The current linear approximation `T₀₀/T_nd - 1` is likely the source of
TC-LAM-3's divergence (Halley residual 3.6), because for T_nd ≈ 3× T₀₀ the
linear guess places x₀ far from the true root.

**Regime 2: T₁ ≤ T_nd ≤ T₀₀ (normal elliptic arc, x₀ ∈ (0, 1))**

The current implementation uses a power-law guess followed by one Newton step:

```
x̂ = (T₁/T_nd)^(2/3)
x₀ = x̂ - (T(x̂) - T_nd) / T'(x̂)   (one Newton step)
```

This is a reasonable approximation for the normal regime. TC-LAM-1's near-stall
(residual 5.8e-7) may originate here if the Newton pre-step overshoots into a
region where Halley's curvature estimate is wrong, or if the TOL_NDIM was
tightened beyond the precision floor of the finite-difference T''.

**Regime 3: T_nd < T₁ (fast arc, x₀ near 1)**

The current implementation uses `x₀ = 1 - X_EPS`, which is the safest
conservative choice. For very fast arcs this may require more Halley iterations.
Izzo's paper suggests a rational-function initial guess in this regime **[VERIFY]**.

---

### A.6 Terminal Velocity Reconstruction

Given the converged x and λ (signed), compute (Izzo 2015, §3.2):

```
ρ  = (r1 - r2) / c         (dimensionless radial asymmetry, ∈ [-1, 1])
γ  = sqrt(μ·s / 2)         (has dimensions m²/s)
σ  = sqrt(1 - ρ²)          (dimensionless; = 0 for r1=r2 degenerate case)
y  = sqrt(1 - λ²·(1-x²))   (same y as in the TOF formula; recomputed from converged x)
```

**Radial velocity components** (positive = outward from central body):

```
Vr1 = (γ/r1) · [(λ·y - x) - ρ·(λ·y + x)]
Vr2 = -(γ/r2) · [(λ·y - x) + ρ·(λ·y + x)]
```

Note the sign of Vr2: the leading minus sign, combined with the plus inside
the bracket, gives the physically correct inward-pointing radial velocity at
arrival for a typical transfer. **[VERIFY]** the exact sign convention from
Izzo (2015) Eq. 17–18. The current implementation (lines 207–209) matches this
form.

**Tangential velocity components** (positive = in the direction of h_hat × r_hat):

```
Vt1 = (γ/r1) · σ · (y + λ·x)
Vt2 = (γ/r2) · σ · (y + λ·x)
```

Note that Vt1 and Vt2 have the **same numerator** `γ·σ·(y + λ·x)` and differ
only in the denominator r1 vs r2. This satisfies angular momentum conservation:
`r1·Vt1 = r2·Vt2 = γ·σ·(y + λ·x)` — the specific angular momentum is constant
along the orbit. This is a useful self-check.

**[VERIFY] TC-LAM-2 bug analysis**: The velocity magnitude error (1329 m/s
observed vs 7668 m/s expected) is a factor of ~5.8. For TC-LAM-2 (r1 = r2 =
6778 km, Δν = 0.3°, tof = 300 s), the transfer is nearly circular, λ ≈ 1,
and the chord c is very small. Check whether `c` is near zero causing `ρ = (r1
- r2)/c` to overflow or become NaN. Since r1 = r2 in this test, ρ = 0 exactly
and σ = 1 exactly, so that cancels. The issue may be in `y + λ·x` for the
near-parabolic (λ ≈ 1, x ≈ 1) regime causing cancellation in Vt1/Vt2.
Specifically, when λ ≈ 1 and x ≈ 1, y ≈ sqrt(1 - (1-x²)) = |x| ≈ 1, so
`y + λ·x ≈ 2`, which is fine. But if convergence stalled and x is wrong
(for instance x ≈ -0.9 instead of x ≈ 0.95), then y and the velocity
reconstruction will be completely wrong. **The velocity magnitude bug most
likely traces back to a convergence failure (wrong x), not a formula error
in the reconstruction itself.**

**Inertial frame projection**:

Define the unit vectors:

```
r1_hat = r₁ / |r₁|               (radial direction at departure)
r2_hat = r₂ / |r₂|               (radial direction at arrival)
h_hat  = unit(cross(r₁, r₂))     (orbit-plane normal, prograde convention)
t1_hat = unit(cross(h_hat, r1_hat))   (tangential direction at r₁)
t2_hat = unit(cross(h_hat, r2_hat))   (tangential direction at r₂)
```

Then:

```
v₁ = Vr1 · r1_hat + Vt1 · t1_hat
v₂ = Vr2 · r2_hat + Vt2 · t2_hat
```

**Sign of h_hat for retrograde transfers (TC-LAM-5 bug candidate)**:

The current implementation always computes `h_hat = unit(cross(r₁, r₂))` (line
213 of `lambert.rs`), regardless of the `prograde` flag. For a prograde transfer
with dnu < π, `cross(r₁, r₂)` points in the +z direction and h_hat is correct.
For a retrograde transfer with dnu > π, the physical orbit plane normal points
in the -z direction, but `cross(r₁, r₂)` still points in the +z direction
(because the cross product depends only on the vectors, not on which way around
the arc goes).

This is the TC-LAM-5 bug. For retrograde transfers, h_hat must be negated:

```
h_hat = unit(cross(r₁, r₂)) · sign(λ)
```

or equivalently:

```
h_hat = if prograde {
    unit(cross(r₁, r₂))
} else {
    vscale(unit(cross(r₁, r₂)), -1.0)
}
```

Because t1_hat and t2_hat are derived from h_hat, negating h_hat negates both
tangential unit vectors, which flips the sign of Vt1 and Vt2 in the inertial
frame. Without this correction, the tangential velocity points in the wrong
direction for retrograde arcs, and the resulting orbit will have h_z > 0
instead of h_z < 0 — exactly the symptom described in TC-LAM-5.

Note that Vr1, Vr2, Vt1, Vt2 (the scalar components) are computed correctly
from the Izzo formulas even for retrograde (λ < 0). The error is solely in how
these scalars are mapped to inertial space via the unit vectors.

---

### A.7 Worked Example

**Setup**: r₁ = [7,000,000, 0, 0] m, r₂ = [0, 10,000,000, 0] m, prograde = true,
μ = 3.986_004_418 × 10¹⁴ m³/s². Transfer time = half the period of the Hohmann
transfer ellipse with semi-major axis a = (r1 + r2)/2.

#### Step 1: Magnitudes

```
r1 = |r₁| = 7,000,000 m       = 7.000000 × 10⁶ m
r2 = |r₂| = 10,000,000 m      = 1.000000 × 10⁷ m
```

#### Step 2: Transfer angle

```
cos(Δν) = dot(r₁, r₂) / (r1·r2)
         = (7e6·0 + 0·10e6 + 0·0) / (7e6 · 10e6)
         = 0 / 7e13
         = 0.000000

Δν = acos(0) = π/2 = 1.570796 rad (exactly 90°)

z_cross = cross(r₁, r₂)[2] = 7e6 · 10e6 - 0 · 0 = 7.000000 × 10¹³ > 0
```

Since `prograde = true` and `z_cross ≥ 0`, no adjustment: Δν = π/2 (short-way arc).

#### Step 3: Non-dimensional geometry

```
c = sqrt(r1² + r2² - 2·r1·r2·cos(Δν))
  = sqrt((7e6)² + (10e6)² - 0)
  = sqrt(4.900000e13 + 1.000000e14)
  = sqrt(1.490000e14)
  = sqrt(149) × 10⁶
  = 12,206,556 m      (sqrt(149) = 12.206556...)
  = 1.220656 × 10⁷ m

s = (r1 + r2 + c) / 2
  = (7,000,000 + 10,000,000 + 12,206,556) / 2
  = 29,206,556 / 2
  = 14,603,278 m      = 1.460328 × 10⁷ m

λ² = 1 - c/s = 1 - 12,206,556 / 14,603,278
   = 1 - 0.835879
   = 0.164121

|λ| = sqrt(0.164121) = 0.405118

Since Δν = π/2 < π: λ = +0.405118 (positive, prograde)
```

#### Step 4: Transfer orbit period and TOF

```
a_transfer = (r1 + r2) / 2 = (7e6 + 10e6) / 2 = 8,500,000 m

T_transfer = 2π · sqrt(a_transfer³ / μ)
           = 2π · sqrt((8.5e6)³ / 3.986004418e14)
           = 2π · sqrt(6.141250e20 / 3.986004e14)
           = 2π · sqrt(1.540703e6 s²)
           = 2π · 1241.25 s
           = 7801.5 s

tof = T_transfer / 2 = 3900.75 s
```

#### Step 5: Non-dimensional TOF

```
s³ = (1.460328e7)³ = 3.114151e21 m³

2μ / s³ = 2 × 3.986004418e14 / 3.114151e21
         = 7.972009e14 / 3.114151e21
         = 2.560007e-7 s⁻²

sqrt(2μ/s³) = 5.059651e-4 s⁻¹

T_nd = tof × sqrt(2μ/s³)
     = 3900.75 × 5.059651e-4
     = 1.97367
```

#### Step 6: Reference times T₀₀ and T₁

```
acos(|λ|) = acos(0.405118) = 1.153248 rad

sqrt(1 - λ²) = sqrt(1 - 0.164121) = sqrt(0.835879) = 0.914264

T₀₀ = acos(|λ|) + |λ|·sqrt(1-λ²)
     = 1.153248 + 0.405118 × 0.914264
     = 1.153248 + 0.370331
     = 1.523579

λ³ = 0.405118³ = 0.405118 × 0.164121 = 0.066481

T₁ = (2/3)·(1 - λ³) = (2/3)·(1 - 0.066481)
   = (2/3)·0.933519
   = 0.622346
```

#### Step 7: Regime determination and initial guess

```
T_nd = 1.97367
T₀₀  = 1.52358
T₁   = 0.62235

Since T_nd = 1.97367 > T₀₀ = 1.52358:
  → Regime 1 (slow arc), x₀ ∈ (-1, 0)

Current implementation initial guess:
  x₀ = T₀₀/T_nd - 1 = 1.52358/1.97367 - 1 = 0.77198 - 1 = -0.22802

  (This is the formula at lambert.rs line 148: t00/t_nd - 1.0)
```

#### Step 8: First Halley iteration

Compute T(x₀, λ) using Lancaster-Blanchard with x₀ = -0.228020, λ = 0.405118.

```
x₀  = -0.228020
x₀² = 0.051993
a_inv₀ = 1 - x₀² = 1 - 0.051993 = 0.948007
a₀  = 1 / 0.948007 = 1.054852

α = 2·acos(-0.228020) = 2·(π - acos(0.228020))
acos(0.228020) ≈ 1.340303 rad
α = 2·(3.141593 - 1.340303) = 2·1.801290 = 3.602580 rad

|λ|·sqrt(a_inv₀) = 0.405118 × sqrt(0.948007)
                 = 0.405118 × 0.973657
                 = 0.394534

β_arg = 0.394534  (≤ 1, OK)
asin(0.394534) ≈ 0.404877 rad
β = +2 × 0.404877 = 0.809754 rad  (positive since λ > 0)

sin(α) = sin(3.602580) = sin(π + 0.461) = -sin(0.461) ≈ -0.444799
sin(β) = sin(0.809754) ≈ 0.723778

a₀^(3/2) = 1.054852^(3/2) = 1.054852 × sqrt(1.054852)
          = 1.054852 × 1.027061 = 1.083394
2·a₀^(3/2) = 2.166788

T₀ = [(α - β) - (sin α - sin β)] / 2·a₀^(3/2)
   = [(3.602580 - 0.809754) - (-0.444799 - 0.723778)] / 2.166788
   = [2.792826 - (-1.168577)] / 2.166788
   = [2.792826 + 1.168577] / 2.166788
   = 3.961403 / 2.166788
   = 1.828350
```

First-iteration error:

```
T_err₀ = T₀ - T_nd = 1.828350 - 1.97367 = -0.145320
```

Compute T'(x₀):

```
y₀ = sqrt(1 - λ²·(1-x₀²)) = sqrt(1 - 0.164121 × 0.948007)
   = sqrt(1 - 0.155545) = sqrt(0.844455) = 0.919160

lam3 = λ³ = 0.066481  (signed, positive here)

T'₀ = [3·x₀·T₀ - 2 + 2·lam3·x₀/y₀] / a_inv₀
    = [3·(-0.228020)·1.828350 - 2 + 2·0.066481·(-0.228020)/0.919160] / 0.948007
    = [-1.251274 - 2 + 2·(-0.016486)] / 0.948007

    Wait — 2·lam3·x₀/y₀:
    = 2 × 0.066481 × (-0.228020) / 0.919160
    = 2 × (-0.015164)
    = -0.030328

    T'₀ = [-1.251274 - 2.000000 - 0.030328] / 0.948007
        = -3.281602 / 0.948007
        = -3.461594
```

Compute T''(x₀):

```
|x₀| = 0.228020 > 1e-8, use analytic formula.

Term1 = 3·T₀ = 3 × 1.828350 = 5.485050
Term2 = (3·x₀ - 4/x₀)·T'₀
      = (3×(-0.228020) - 4/(-0.228020)) × (-3.461594)
      = (-0.684060 + 17.542842) × (-3.461594)
      = 16.858782 × (-3.461594)
      = -58.333

  More carefully:
  3·x₀ = -0.684060
  4/x₀ = 4/(-0.228020) = -17.542842
  3·x₀ - 4/x₀ = -0.684060 - (-17.542842) = 16.858782
  × T'₀ = 16.858782 × (-3.461594) = -58.332

Term3 = (4/x₀²)·(T₀ - (2/3)·(1-lam3))
      = (4/0.051993)·(1.828350 - (2/3)×(1-0.066481))
      = 76.9324 × (1.828350 - 0.622346)

  (2/3)×(1-lam3) = (2/3)×0.933519 = 0.622346
  T₀ - 0.622346 = 1.828350 - 0.622346 = 1.206004
  Term3 = 76.9324 × 1.206004 = 92.798

T''₀ = (Term1 + Term2 + Term3) / a_inv₀
     = (5.485050 - 58.332 + 92.798) / 0.948007
     = 39.951 / 0.948007
     = 42.143
```

Halley step:

```
denom = T'₀² - 0.5·T_err₀·T''₀
      = (-3.461594)² - 0.5×(-0.145320)×42.143
      = 11.982622 + 3.063734
      = 15.046356

Δx₀ = -T_err₀·T'₀ / denom
     = -(-0.145320)×(-3.461594) / 15.046356
     = -(0.502990) / 15.046356
     = -0.033432

x₁ = x₀ + Δx₀ = -0.228020 + (-0.033432) = -0.261452
```

The iteration moves x further toward -1, which increases T toward T_nd. This
makes physical sense: a slower arc (more negative x) takes longer.

#### Step 9: Second iteration

```
x₁ = -0.261452  (continue iteration in the same manner)
```

The developer should run the implementation with debug output to trace x values.
Typically 3–5 Halley iterations suffice. The converged x will be approximately
**x_conv ≈ -0.312** for this geometry and TOF (estimate; verify with
implementation).

#### Step 10: Velocity reconstruction (using converged x)

For the purpose of this worked example, we use the independent Hohmann formula
to establish the expected velocity magnitudes, then back-derive what x_conv
should produce.

**Independent Hohmann check**:

```
μ = 3.986004418e14 m³/s²
r1 = 7.000000e6 m
r2 = 1.000000e7 m
a  = (r1 + r2)/2 = 8.500000e6 m

v1_circ     = sqrt(μ/r1) = sqrt(3.986e14/7e6) = sqrt(5.6943e7) = 7546.0 m/s
v1_transfer = sqrt(μ·(2/r1 - 1/a))
            = sqrt(3.986e14 × (2/7e6 - 1/8.5e6))
            = sqrt(3.986e14 × (2.857143e-7 - 1.176471e-7))
            = sqrt(3.986e14 × 1.680672e-7)
            = sqrt(6.700002e7)
            = 8185.4 m/s

v2_transfer = sqrt(μ·(2/r2 - 1/a))
            = sqrt(3.986e14 × (2/10e6 - 1/8.5e6))
            = sqrt(3.986e14 × (2.000000e-7 - 1.176471e-7))
            = sqrt(3.986e14 × 8.235294e-8)
            = sqrt(3.282553e7)
            = 5729.4 m/s

v2_circ     = sqrt(μ/r2) = sqrt(3.986e14/10e6) = sqrt(3.9860e7) = 6313.5 m/s
```

Expected |v₁| ≈ 8185 m/s (departure from r1 = 7000 km orbit on transfer ellipse)
Expected |v₂| ≈ 5729 m/s (arrival at r2 = 10000 km orbit on transfer ellipse)

Note: For a true Hohmann (180° transfer), v₁ is purely tangential. For this
90° transfer, the velocity has both radial and tangential components; the
magnitudes above are the total speeds and should match what Lambert returns.

**Reconstruction formulas** (at converged x_conv):

```
γ = sqrt(μ·s/2)
  = sqrt(3.986004418e14 × 1.460328e7 / 2)
  = sqrt(3.986004418e14 × 7.301640e6)
  = sqrt(2.910897e21)
  = 5.395273e10  [m²/s]

ρ = (r1 - r2) / c
  = (7,000,000 - 10,000,000) / 12,206,556
  = -3,000,000 / 12,206,556
  = -0.245779

σ = sqrt(1 - ρ²)
  = sqrt(1 - 0.060408)
  = sqrt(0.939592)
  = 0.969326
```

Using the converged x_conv (developer must verify numerically; estimated
x_conv ≈ -0.312 for this case):

```
y_conv = sqrt(1 - λ²·(1-x_conv²))
       = sqrt(1 - 0.164121·(1-0.097344))
       = sqrt(1 - 0.164121·0.902656)
       = sqrt(1 - 0.148128)
       = sqrt(0.851872)
       ≈ 0.922969   [VERIFY: use actual x_conv from converged iteration]

Vr1 = (γ/r1)·[(λ·y - x) - ρ·(λ·y + x)]
    = (5.395273e10/7e6)·[(0.405118×0.922969 - (-0.312)) - (-0.245779)·(0.405118×0.922969+(-0.312))]

    λ·y = 0.405118 × 0.922969 = 0.373925
    λ·y - x = 0.373925 - (-0.312) = 0.685925
    λ·y + x = 0.373925 + (-0.312) = 0.061925
    ρ·(λ·y+x) = -0.245779 × 0.061925 = -0.015222
    (λ·y-x) - ρ·(λ·y+x) = 0.685925 - (-0.015222) = 0.701147

    γ/r1 = 5.395273e10 / 7e6 = 7707.5 m/s

    Vr1 = 7707.5 × 0.701147 = 5404  [m/s]

Vr2 = -(γ/r2)·[(λ·y - x) + ρ·(λ·y + x)]
    = -(5.395273e10/1e7)·[0.685925 + (-0.015222)]
    = -5395.3 × 0.670703
    = -3619  [m/s]

Vt1 = (γ/r1)·σ·(y + λ·x)
    = 7707.5 × 0.969326 × (0.922969 + 0.405118×(-0.312))
    = 7707.5 × 0.969326 × (0.922969 - 0.126397)
    = 7707.5 × 0.969326 × 0.796572
    = 5940  [m/s]

Vt2 = (γ/r2)·σ·(y + λ·x)
    = 5395.3 × 0.969326 × 0.796572
    = 4158  [m/s]
```

Note: These are estimates using x_conv ≈ -0.312 which is itself an
approximation. The developer must verify by running the actual converged
iteration. The key sanity check is:

```
|v₁|² = Vr1² + Vt1² ≈ 5404² + 5940² ≈ 2.92e7 + 3.53e7 = 6.45e7
|v₁| ≈ 8031 m/s   (should be ≈ 8185 m/s from Hohmann formula above)
```

The small discrepancy (2%) is due to the approximate x_conv = -0.312 used here.
At the true converged x, the Lambert formula must reproduce the Hohmann speed
exactly.

#### Step 11: Unit vectors

```
r1_hat = [1, 0, 0]           (r₁ is along +X)
r2_hat = [0, 1, 0]           (r₂ is along +Y)

cross(r₁, r₂) = [7e6,0,0] × [0,10e6,0]
              = [0×0 - 0×10e6, 0×0 - 7e6×0, 7e6×10e6 - 0×0]
              = [0, 0, 7e13]

h_hat = unit([0, 0, 7e13]) = [0, 0, 1]   (+Z, correct for prograde equatorial orbit)

t1_hat = unit(cross(h_hat, r1_hat))
       = unit(cross([0,0,1], [1,0,0]))
       = unit([0×0-1×0, 1×1-0×0, 0×0-0×1])
       = unit([0, 1, 0])
       = [0, 1, 0]

t2_hat = unit(cross(h_hat, r2_hat))
       = unit(cross([0,0,1], [0,1,0]))
       = unit([0×0-1×1, 1×0-0×0, 0×1-0×0])
       = unit([-1, 0, 0])
       = [-1, 0, 0]
```

#### Step 12: Inertial velocity vectors

```
v₁ = Vr1·r1_hat + Vt1·t1_hat
   = Vr1·[1,0,0] + Vt1·[0,1,0]
   = [Vr1, Vt1, 0]
   ≈ [5404, 5940, 0] m/s   (estimate; verify with actual x_conv)

v₂ = Vr2·r2_hat + Vt2·t2_hat
   = Vr2·[0,1,0] + Vt2·[-1,0,0]
   = [-Vt2, Vr2, 0]
   ≈ [-4158, -3619, 0] m/s   (estimate; verify with actual x_conv)
```

**Self-check**: `cross(r₁, v₁)[2] = 7e6 × 5940 - 0 × 5404 = 4.158e10 > 0` (prograde, correct).

**Hohmann sanity check**:

```
|v₁|_expected (from Hohmann) = 8185 m/s
|v₁|_Lambert  (at x_conv)    = sqrt(Vr1² + Vt1²)  [must match to < 1 m/s]

|v₂|_expected (from Hohmann) = 5729 m/s
|v₂|_Lambert  (at x_conv)    = sqrt(Vt2² + Vr2²)  [must match to < 1 m/s]
```

The developer should compute both magnitudes from the running implementation
and verify they fall within 1 m/s of the Hohmann values above. A discrepancy
larger than ~10 m/s indicates either a convergence failure (wrong x) or a
formula error in the velocity reconstruction.

---

### A.8 Known Bug Candidates in `math/lambert.rs`

The following is a prioritized checklist derived from the four failing test
cases. Each item specifies the exact code location and what to check.

**1. h_hat sign for retrograde transfers (TC-LAM-5: divergence, residual 3.0)**

- Location: `lambert.rs` line 213.
- Current code: `let h_hat = unit(cross(r1, r2));`
- Problem: For retrograde (`prograde = false`), the effective transfer arc goes
  the long way, and the orbit-plane normal should point in the -z direction for
  typical equatorial geometries. The current code always computes the same
  h_hat regardless of the `prograde` flag.
- Fix: `let h_hat = if prograde { unit(cross(r1, r2)) } else { vscale(unit(cross(r1, r2)), -1.0) };`
- Consequence of bug: t1_hat and t2_hat have reversed tangential direction,
  so Vt1 and Vt2 are added with the wrong sign in the inertial frame. The
  resulting orbit has h_z > 0 instead of h_z < 0, causing the retrograde
  invariant check to fail.
- Note: The divergence (residual 3.0) reported in TC-LAM-5 may be a secondary
  effect — if `dnu > π` and the initial guess is also wrong, the iteration
  itself fails. Verify whether fixing h_hat alone resolves convergence.

**2. T₁ formula uses |λ| for retrograde (TC-LAM-5: wrong regime boundary)**

- Location: `lambert.rs` line 144.
- Current code: `let t1 = (2.0 / 3.0) * (1.0 - lambda_abs * lambda_abs * lambda_abs);`
- Problem: Uses `lambda_abs³` = |λ|³. For retrograde (λ < 0), the
  minimum-energy T is `(2/3)·(1 + |λ|³)` (more than 2/3), but the code
  computes `(2/3)·(1 - |λ|³)` (less than 2/3). This places the initial-guess
  regime boundary in the wrong place for retrograde cases.
- Fix: Use the signed λ: `let t1 = (2.0 / 3.0) * (1.0 - lambda * lambda * lambda);`
  where `lambda` is the signed value. For λ > 0 this is unchanged. For λ < 0
  this gives T₁ > 2/3, correctly placing the regime boundary.
- **[VERIFY]** Confirm against Izzo (2015) Eq. 19 whether the T at x=1 limit
  uses signed λ³ or |λ|³.

**3. Initial guess for T > T₀₀ regime (TC-LAM-3: divergence on long TOF)**

- Location: `lambert.rs` lines 147–149.
- Current code: `(t00 / t_nd - 1.0).clamp(-1.0 + X_EPS, 0.0)`
- Problem: For TC-LAM-3 (TLI, 3-day transfer), T_nd is much larger than T₀₀,
  placing x₀ very close to -1. The Lancaster-Blanchard formula is poorly
  conditioned near x = -1 (α approaches 2π, the denominator a^(3/2) → ∞),
  and both T and T' lose significance.
- Fix: Implement Izzo (2015) Eq. 23–24, which gives a logarithmic or
  rational-function initial guess that is well-conditioned for T >> T₀₀.
  **[VERIFY]** Retrieve the exact Eq. 23–24 from the paper. A stopgap is to
  clamp x₀ away from -1 more aggressively (e.g., `x₀ ≥ -0.9`) and rely on
  more Halley iterations, but this will be slow and may still fail to converge.

**4. T' and T'' use signed λ³ but T₁ guard uses |λ|³ (inconsistency)**

- Location: `lambert.rs` lines 267–292 (derivatives use `lam3 = lam2 * lambda`
  which is signed) vs. line 144 (T₁ uses `lambda_abs³`).
- Problem: The derivative formulas are self-consistent (they use signed lam3
  throughout), but the regime boundary check (line 144) uses the unsigned
  version, so the initial guess regime may be misidentified for λ < 0.
- This is closely related to bug 2 above.

**5. TOL_NDIM is relaxed from spec value (TC-LAM-1: near-convergence stall)**

- Location: `lambert.rs` line 22.
- Current code: `const TOL_NDIM: f64 = 1.0e-6;`
- Spec value (§9): `TOL_NDIM = 1.0e-12`.
- Problem: The tolerance was relaxed to 1e-6 with a comment about "stalling."
  TC-LAM-1's residual of 5.8e-7 suggests the iteration reached the 1e-6
  tolerance but then failed to converge further. This is consistent with the
  Halley step stalling — possibly due to the finite-difference T'' near x = 0
  being inaccurate, or due to the initial guess Newton pre-step overshooting.
- Investigation: Add a convergence trace (print x and T_err each iteration)
  to see if the residual is decreasing monotonically or oscillating. If
  oscillating, the issue is likely the T'' finite-difference step size (1e-5)
  being too large for the curvature at the solution point.

**6. cos(Δν) computation — clamping (already correct)**

- Location: `lambert.rs` line 104.
- Current code: `let cos_dnu = (dot(r1, r2) / (r1_mag * r2_mag)).clamp(-1.0, 1.0);`
- Status: CORRECT. The `dot(r1,r2)/(r1_mag·r2_mag)` form is divided-then-
  clamped, which protects against floating-point excursions beyond [-1, 1].
  No bug here; listed for completeness.

**7. Prograde/retrograde flag and dnu (already correct structure, but verify)**

- Location: `lambert.rs` lines 115–120.
- Current code assigns `dnu = 2π - dnu` based on the z-component of cross(r₁,r₂)
  and the `prograde` flag. This is correct for the λ sign assignment (lines 131–135),
  but verify that the dnu assignment is consistent with the h_hat fix (bug 1 above).
  After fixing h_hat, the `dnu > π` check for λ sign should still be correct because
  the dnu disambiguation happens before the λ computation. No change needed here
  IF the h_hat fix is applied separately to the unit-vector projection step.

**8. MAX_ITER and the panic message**

- Location: `lambert.rs` lines 25 and 189–196.
- Current code: `const MAX_ITER: usize = 100;` (spec says 50).
- This is a minor discrepancy. With the initial-guess bug causing divergence,
  raising MAX_ITER to 100 was a workaround. After fixing the initial guess,
  restore MAX_ITER to 50. With a correct initial guess, no well-posed problem
  should take more than 10–15 Halley iterations.

**Summary table**:

| Bug | Test case | Location | Severity | Root cause |
|---|---|---|---|---|
| h_hat sign for retrograde | TC-LAM-5 | line 213 | Critical — wrong orbit plane | Missing `-1` multiplier for `!prograde` |
| T₁ uses \|λ\|³ not λ³ | TC-LAM-5, TC-LAM-3 | line 144 | High — wrong regime boundary | Should use signed `lambda` in T₁ |
| Initial guess for T >> T₀₀ | TC-LAM-3 | lines 147–149 | High — divergence | Poor linear approximation; use Izzo Eq. 23–24 |
| TOL_NDIM relaxed to 1e-6 | TC-LAM-1 | line 22 | Medium — near-stall | Stall in Halley; investigate before restoring 1e-12 |
| Velocity magnitude wrong | TC-LAM-2 | lines 207–210 | High — wrong answer | Most likely secondary to TC-LAM-1 convergence bug |

The order of investigation should be: (1) fix h_hat sign for retrograde, (2)
fix T₁ formula, (3) fix initial guess for T >> T₀₀, (4) investigate TC-LAM-1
stall in detail, (5) re-run TC-LAM-2 to see if it resolves automatically after
the convergence fix.
