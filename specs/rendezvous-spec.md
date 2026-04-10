# Specification: `guidance/rendezvous` Module — Rendezvous Primitives

**Status**: Ready for implementation (Milestone 5 Phase 1)
**Module path**: `agc-core/src/guidance/rendezvous.rs`
**Architecture reference**: `docs/architecture.md` §7.2 (P20–P23, P31–P34 rows), §9 (Navigation Math)
**State-vector reference**: `specs/state-vector-spec.md` §4.1 `Frame`, §4.2 `StateVector`
**Math reference**: `specs/linalg-spec.md` §4 (`dot`, `cross`, `norm`, `unit`, `vscale`, `vadd`, `vsub`)
**Types reference**: `specs/types-module-spec.md` §3.3 `Vec3`, `Mat3x3`
**LVLH frame distinction**: This module defines the **rendezvous LVLH frame** (Hill/CW convention),
which differs from the RSW targeting frame used in `specs/targeting-spec.md` §2.2 and
`specs/p30-spec.md` §2.3. See §4 below for the explicit reconciliation.
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, Comanche055 directory):
- `Comanche055/R32,R33,R34,R35.agc` — relative-state display and LVLH conversion routines
- `Comanche055/INTERPLANETARY_SUBROUTINES.agc` — range and range-rate computation helpers
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `RONE`/`VONE` (target state, B+28/B+7 scale),
  `RELVEC`/`VELVEC` (relative position/velocity, B+28/B+7 scale)
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
Operation*, Springer-Praxis 2010. Rendezvous navigation: Chapter 11 (Rendezvous and
Navigation Programs), pp. 303–340; LVLH display routines R32–R35: pp. 327–333; range and
range-rate computation: pp. 315–318.

---

## 1. Purpose

`guidance::rendezvous` provides the pure-math primitives for the entire rendezvous
program family (P20–P23 navigation and tracking, P31–P34 maneuver targeting). Every
downstream program calls into this module to answer the same four basic situational
questions: Where is the target relative to me? How fast is the range changing? What
direction am I looking at the target? When do we get closest?

The functions in this module are stateless: they take inertial state vectors (position
and velocity in ECI or MCI) and return derived quantities. No program state, no DSKY
interaction, and no maneuver computation belongs here. The module is the rendezvous
analogue of `math::linalg` — a vocabulary of orbital mechanics primitives that the
program-layer code assembles into complete navigation cycles.

---

## 2. Module Path

`agc-core/src/guidance/rendezvous.rs`

The module must be declared in `agc-core/src/guidance/mod.rs` alongside the existing
`targeting`, `maneuver`, and `lambert` submodules.

---

## 3. Dependencies

| Crate-local module | Items used |
|--------------------|-----------|
| `math::linalg` | `dot`, `cross`, `norm`, `unit`, `vscale`, `vadd`, `vsub`, `mxv`, `transpose` |
| `types` | `Vec3`, `Mat3x3` |
| `navigation::state_vector` | `StateVector` (read-only; no mutation) |

No `no_std` allocation is required. All returned types are stack values (`f64`, `Vec3`,
`Mat3x3`, or small structs defined below).

---

## 4. Coordinate Frame Definition: Rendezvous LVLH

### 4.1 Purpose of a separate frame

The RSW (Radial–In-track–Cross-track) frame defined in `specs/targeting-spec.md` §2.2
orients the delta-V calculation for engine burns: its R axis points radially outward,
S is in-track prograde, W is the orbit normal. This is the frame used by P30 to accept
ground-uplinked burn vectors.

Rendezvous navigation and display programs use a **different** convention, here called
the **rendezvous LVLH frame**, that matches the Hill/Clohessy-Wiltshire linearisation
used in relative-motion analysis and in the AGC R32–R35 display routines (O'Brien
pp. 327–333). The axes are defined relative to the **target vehicle's** orbit, not the
chaser's.

### 4.2 Axis definitions (rendezvous LVLH of the target)

Let:
- `r_t` = inertial position of the target (m, ECI or MCI)
- `v_t` = inertial velocity of the target (m/s)
- `h_t` = angular momentum vector of the target = `cross(r_t, v_t)`

Then:

| Axis | Symbol | Definition | Positive direction |
|------|--------|------------|--------------------|
| x (along-track) | `x_hat` | `unit(cross(h_t, r_t))` | In the velocity direction for circular orbits; local horizontal along-track |
| y (out-of-plane) | `y_hat` | `unit(-h_t)` = `unit(cross(v_t, r_t))` | Opposite to the angular momentum vector (south in equatorial orbit) |
| z (radial) | `z_hat` | `unit(-r_t)` | Toward the gravitating body centre (downward); opposite to the RSW R-axis |

The three axes form a right-handed frame: `x_hat × y_hat = z_hat` (verifiable from the
definitions).

### 4.3 Reconciliation with the RSW targeting frame

| RSW axis (`targeting.rs`) | Rendezvous LVLH axis | Relationship |
|---------------------------|----------------------|-------------|
| R = `unit(r_t)` | z = `unit(-r_t)` | `z_hat = -R_hat` |
| S = `unit(cross(W, R))` | x = `unit(cross(h_t, r_t))` | `x_hat = S_hat` for circular orbits (identical in general) |
| W = `unit(cross(r_t, v_t))` | y = `unit(-h_t)` | `y_hat = -W_hat` |

The rendezvous LVLH frame therefore has z pointing toward Earth (inward radial) and
y pointing south for prograde equatorial orbits, consistent with the Hill equations as
presented in Battin (1987, §10.5) and with the R32–R35 display convention described by
O'Brien (p. 328): "the z-axis of the relative display is directed toward the Earth."

### 4.4 LVLH rotation matrix (inertial to rendezvous LVLH)

The 3×3 rotation matrix `M_I2L` that maps an inertial vector to the rendezvous LVLH
frame has the unit basis vectors as its rows:

```
M_I2L = [ x_hat^T ]   (row 0: along-track)
        [ y_hat^T ]   (row 1: out-of-plane / -h)
        [ z_hat^T ]   (row 2: radial-inward)
```

The inverse (LVLH to inertial) is the transpose: `M_L2I = M_I2L^T`.

---

## 5. Types

### 5.1 `LvlhState`

Relative position and velocity of the active vehicle (CSM) with respect to the target
vehicle, expressed in the rendezvous LVLH frame of the target.

```rust
/// Relative state of the active vehicle with respect to the target vehicle,
/// expressed in the rendezvous LVLH frame of the target vehicle.
///
/// All components are in SI units (m for position, m/s for velocity).
/// Axis conventions: x = along-track (velocity direction for circular orbits),
/// y = out-of-plane (opposite angular momentum), z = radial-inward (toward body).
pub struct LvlhState {
    /// Relative position vector in the LVLH frame (m).
    /// rho_lvlh = M_I2L * (r_active - r_target)
    pub rho: Vec3,
    /// Relative velocity vector in the LVLH frame (m/s).
    /// rho_dot_lvlh = M_I2L * (v_active - v_target)
    pub rho_dot: Vec3,
}
```

### 5.2 `LosAngles`

Line-of-sight angles from the active vehicle to the target, in the rendezvous LVLH
frame. These are the angles displayed by the AGC R33 routine (O'Brien p. 329).

```rust
/// Line-of-sight angles to the target expressed in the rendezvous LVLH frame.
///
/// Elevation: angle above the local horizontal plane (range [-π/2, +π/2]).
/// Azimuth: angle in the local horizontal plane measured from +x (along-track),
///          positive toward +y (out-of-plane), range (-π, +π].
pub struct LosAngles {
    /// Elevation angle (rad).  Positive = target is above local horizontal.
    pub elevation: f64,
    /// Azimuth angle (rad).  0 = directly ahead along-track; π/2 = out-of-plane.
    pub azimuth: f64,
}
```

---

## 6. Public API

### 6.1 `lvlh_matrix`

```rust
/// Compute the rotation matrix from the inertial frame to the rendezvous LVLH
/// frame of a target vehicle.
///
/// # Preconditions
/// - `r_target` must be non-zero (panics if `norm(r_target) == 0.0`).
/// - `r_target` and `v_target` must not be parallel (degenerate: radial flight;
///   see §7 Edge Cases).  Panics if `norm(cross(r_target, v_target)) == 0.0`.
///
/// # Postconditions
/// - The returned matrix is orthonormal: `M * M^T = I` (up to f64 rounding).
/// - Row 0 = x_hat (along-track), Row 1 = y_hat (out-of-plane), Row 2 = z_hat (radial-inward).
///
/// # Invariant
/// Given `r_t` on a circular orbit, `mxv(result, v_target)` has zero z-component
/// and positive x-component.
pub fn lvlh_matrix(r_target: Vec3, v_target: Vec3) -> Mat3x3
```

### 6.2 `relative_state_lvlh`

```rust
/// Convert two inertial state vectors into a relative state expressed in the
/// rendezvous LVLH frame of the target vehicle.
///
/// # Preconditions
/// - All same-frame preconditions as `lvlh_matrix` apply to the target state.
/// - Both state vectors must be expressed in the same inertial frame (ECI or MCI).
///   The caller is responsible for frame consistency; this function does not check.
///
/// # Postconditions
/// - `result.rho`     == M_I2L * (r_active - r_target)
/// - `result.rho_dot` == M_I2L * (v_active - v_target)
///
/// # Note
/// The LVLH frame rotates with the target's orbit; consequently `rho_dot` is NOT
/// the time derivative of `rho` in an inertial sense. It is the velocity of the
/// active vehicle relative to the target in the LVLH frame at the instant of
/// evaluation. Coriolis and centrifugal terms appear only when propagating `rho`
/// forward (Hill equations); this function evaluates the instantaneous snapshot only.
pub fn relative_state_lvlh(
    r_active:  Vec3,
    v_active:  Vec3,
    r_target:  Vec3,
    v_target:  Vec3,
) -> LvlhState
```

### 6.3 `range`

```rust
/// Scalar range (distance) between the active and target vehicles.
///
/// # Definition
///   range = |r_active - r_target|  (m)
///
/// # Preconditions
/// - None beyond finite, non-NaN inputs.
///
/// # Postconditions
/// - result >= 0.0
/// - result == 0.0 only when both vehicles are at the same inertial position.
///
/// # Note
/// This function does NOT require the LVLH frame to be constructed; it operates
/// directly on inertial vectors and is faster than extracting `norm(rho)` from
/// `relative_state_lvlh`. Callers that need only range should prefer this function.
pub fn range(r_active: Vec3, r_target: Vec3) -> f64
```

### 6.4 `range_rate`

```rust
/// Time derivative of range: the scalar rate at which the active vehicle is moving
/// away from (or toward) the target vehicle.
///
/// # Definition
///   range_rate = dot(rho_vec, rho_dot_vec) / |rho_vec|   (m/s)
///
///   where rho_vec     = r_active - r_target   (inertial relative position)
///         rho_dot_vec = v_active - v_target   (inertial relative velocity)
///
/// # Sign convention
///   Positive  => vehicles are separating (range increasing).
///   Negative  => vehicles are closing    (range decreasing).
///
/// # Preconditions
/// - `range(r_active, r_target)` must be > 0.0.  Panics if zero (see §7).
///
/// # Postconditions
/// - result == 0.0 when rho_dot is perpendicular to rho (instantaneous closest
///   approach or instantaneous farthest point).
/// - |result| <= norm(rho_dot_vec)  (Cauchy–Schwarz).
///
/// # AGC correspondence
/// This computation maps to the `RDOT` variable maintained by the R32 routine
/// (O'Brien p. 316).  The AGC stored `RDOT` at scale B+7 m/s in erasable.
pub fn range_rate(r_active: Vec3, v_active: Vec3, r_target: Vec3, v_target: Vec3) -> f64
```

### 6.5 `los_angles_lvlh`

```rust
/// Compute the line-of-sight elevation and azimuth from the active vehicle to
/// the target, expressed in the rendezvous LVLH frame of the target.
///
/// # Definition
///   Let rho_lvlh = [x, y, z] be the relative position in the LVLH frame.
///   horizontal_range = sqrt(x^2 + y^2)
///   elevation = atan2(-z_lvlh, horizontal_range)
///               (positive when target is above local horizontal, i.e. z_lvlh < 0)
///   azimuth   = atan2(y_lvlh, x_lvlh)
///               (0 when target is directly ahead along-track)
///
/// # Preconditions
/// - `lvlh.rho` must be non-zero; panics if `norm(lvlh.rho) == 0.0` (zero range).
///
/// # Postconditions
/// - elevation ∈ [-π/2, +π/2]
/// - azimuth   ∈ (-π,   +π]
///
/// # Degenerate case
/// When the target is directly overhead (x == 0, y == 0, z < 0) elevation = +π/2,
/// azimuth = 0 (defined by convention; azimuth is undefined but 0 is returned).
/// When the target is directly below (x == 0, y == 0, z > 0) elevation = -π/2,
/// azimuth = 0.
///
/// # AGC correspondence
/// Maps to the LOS angle computation in R33 (O'Brien pp. 329–330).  The AGC
/// display used shaft and trunnion angles of the CM optics; this function returns
/// the equivalent geometric angles without the optics CDU encoding.
pub fn los_angles_lvlh(lvlh: &LvlhState) -> LosAngles
```

### 6.6 `time_to_closest_approach`

```rust
/// Approximate time to closest approach (TCA) by linear extrapolation of the
/// current relative position and velocity.
///
/// # Definition
///   TCA = -dot(rho_vec, rho_dot_vec) / dot(rho_dot_vec, rho_dot_vec)   (s)
///
///   where rho_vec and rho_dot_vec are the inertial relative position and velocity.
///   This is the exact solution for constant-velocity (unforced) relative motion
///   and the first-order approximation for orbital relative motion.
///
/// # Sign convention
///   TCA > 0  => closest approach is in the future.
///   TCA < 0  => closest approach was in the past (vehicles are already diverging).
///   TCA = 0  => currently at closest approach (range_rate == 0).
///
/// # Preconditions
/// - `dot(rho_dot_vec, rho_dot_vec)` must be > 0.0; i.e. relative velocity is
///   non-zero.  Panics if relative velocity is zero (see §7).
///
/// # Postconditions
/// - When `range_rate(r_active, v_active, r_target, v_target) == 0.0`,
///   `time_to_closest_approach` returns 0.0.
/// - When the active vehicle is on a pure closing trajectory (range_rate < 0) and
///   no orbital curvature, TCA > 0.
///
/// # Limitation
/// The linear approximation degrades for TCA values longer than roughly one orbital
/// period. It is accurate to within ~1 % for typical Apollo rendezvous geometries
/// (range < 50 km, TCA < 30 min).  Callers requiring high-fidelity TCA for
/// longer horizons should propagate the state vectors with `math::kepler::kepler_step`
/// and iterate.
///
/// # AGC correspondence
/// The R34 display routine computed a "time of intercept" using this same linear
/// formula on the relative velocity vector (O'Brien p. 332).
pub fn time_to_closest_approach(
    r_active:  Vec3,
    v_active:  Vec3,
    r_target:  Vec3,
    v_target:  Vec3,
) -> f64
```

---

## 7. Algorithm Specifications

### 7.1 `lvlh_matrix` — Step-by-step

Given `r_target` (m) and `v_target` (m/s) in the inertial frame:

1. Compute the angular momentum vector:
   ```
   h = cross(r_target, v_target)        [m²/s, not normalised]
   ```
2. Compute the LVLH basis vectors:
   ```
   z_hat = unit(-r_target)              [radial-inward; panics if norm(r_target) == 0]
   y_hat = unit(-h)                     [out-of-plane; panics if norm(h) == 0]
   x_hat = cross(y_hat, z_hat)          [along-track; guaranteed unit if y and z are unit]
   ```
   Note: `x_hat = cross(y_hat, z_hat)` is derived from the right-hand rule and does
   not require a separate normalisation call because `y_hat` and `z_hat` are already
   orthonormal unit vectors and `y_hat ⊥ z_hat` is ensured when `r_t` and `h` are
   not parallel (which they cannot be by construction of the cross product).
3. Assemble the rotation matrix with basis vectors as rows:
   ```
   M_I2L = [ x_hat[0], x_hat[1], x_hat[2] ]   (row 0)
           [ y_hat[0], y_hat[1], y_hat[2] ]   (row 1)
           [ z_hat[0], z_hat[1], z_hat[2] ]   (row 2)
   ```

Reference: O'Brien p. 327 — the R32 routine constructs an equivalent matrix
(labelled the "RSW" or "LHRL" frame in different AGC documents) using the same
cross-product recipe; the sign conventions described here match the "downward Z"
form shown in the R32 display register annotation.

### 7.2 `relative_state_lvlh` — Step-by-step

1. Compute the LVLH rotation matrix: `M = lvlh_matrix(r_target, v_target)`.
2. Compute the inertial relative position: `rho_inertial = vsub(r_active, r_target)`.
3. Compute the inertial relative velocity: `rhodot_inertial = vsub(v_active, v_target)`.
4. Rotate both into the LVLH frame:
   ```
   rho     = mxv(M, rho_inertial)
   rho_dot = mxv(M, rhodot_inertial)
   ```
5. Return `LvlhState { rho, rho_dot }`.

Note on `rho_dot`: the LVLH frame is rotating (its axes change as the target orbits),
so the true time derivative of `rho` in the LVLH frame includes a Coriolis term
`-omega × rho`. The quantity `rho_dot` returned here is the **inertial** relative
velocity rotated into the LVLH frame, which is the quantity displayed to the crew
(it is the velocity difference observed in the non-rotating inertial sense,
expressed in local coordinates). This matches the convention used in the AGC R32
display (O'Brien p. 328) and is the correct input to the range-rate formula.

### 7.3 `range` — Step-by-step

1. `rho_inertial = vsub(r_active, r_target)`
2. Return `norm(rho_inertial)`

This is a two-line computation. The explicit function exists for clarity and to
provide the AGC erasable variable correspondence (`RNG` at scale B+28 m, in the
R32/R35 routines, O'Brien p. 316).

### 7.4 `range_rate` — Step-by-step

1. `rho     = vsub(r_active, r_target)`           (m)
2. `rho_dot = vsub(v_active, v_target)`           (m/s)
3. `rng     = norm(rho)`                          (m); panic if == 0.0
4. Return `dot(rho, rho_dot) / rng`              (m/s)

Mathematical derivation: `d/dt |rho| = d/dt sqrt(dot(rho, rho)) = dot(rho, rho_dot) / |rho|`.

AGC reference: The erasable variable `RDOT` (scale B+7 m/s) is computed by exactly
this formula in the R32 range/range-rate display routine. The sign convention
(positive = opening) is consistent with Comanche055 display noun N16 (range and
range-rate): positive `RDOT` meant the target was moving away
(O'Brien p. 316, Table 11.2).

### 7.5 `los_angles_lvlh` — Step-by-step

Given `lvlh.rho = [x, y, z]` (all in metres):

1. `horizontal_range = sqrt(x*x + y*y)`         (m)
2. `elevation = atan2(-z, horizontal_range)`     (rad)
   - Note the negation of z: the LVLH z-axis points toward Earth (downward), so a
     target at negative z (above the chaser in the radial sense) has positive elevation.
3. `azimuth = atan2(y, x)`                       (rad)
4. Panic if `norm([x, y, z]) == 0.0` (zero range — caller error).
5. Return `LosAngles { elevation, azimuth }`.

The `atan2` calls handle all quadrants correctly and avoid division-by-zero at the
poles. No special-casing is required in the normal path; the degenerate pole case
(`x == y == 0`) is handled gracefully by `atan2(0, 0) = 0` on IEEE 754 hardware,
which satisfies the specification's convention of returning azimuth = 0 at the poles.

AGC reference: The R33 sextant-pointing routine computed shaft and trunnion CDU angles
from the LOS unit vector in the LVLH frame using equivalent `atan2` operations
(O'Brien pp. 329–330).

### 7.6 `time_to_closest_approach` — Step-by-step

1. `rho     = vsub(r_active, r_target)`           (m)
2. `rho_dot = vsub(v_active, v_target)`           (m/s)
3. `speed2  = dot(rho_dot, rho_dot)`              (m²/s²); panic if == 0.0
4. Return `-dot(rho, rho_dot) / speed2`           (s)

Derivation: Minimise `f(t) = |rho + rho_dot * t|^2` over `t` (linear motion).
`df/dt = 2 * dot(rho + rho_dot*t, rho_dot) = 0` gives `t = -dot(rho, rho_dot) / dot(rho_dot, rho_dot)`.

AGC reference: The R34 display routine used an equivalent linear intercept estimate
(O'Brien p. 332) labeled "time of intercept" to guide the crew during the approach
phase. The formula is also cited in the CSI/CDH targeting documentation for Comanche055
as the initial estimate before iterative refinement.

---

## 8. Edge Cases

| Condition | Affected functions | Required behaviour |
|-----------|-------------------|--------------------|
| `norm(r_target) == 0.0` | `lvlh_matrix`, `relative_state_lvlh` | Panic. A target at the gravitating body centre is physically impossible in normal flight; the restart handler restores safe state. |
| `norm(cross(r_target, v_target)) == 0.0` (radial flight: `r_t ∥ v_t`) | `lvlh_matrix`, `relative_state_lvlh` | Panic. The LVLH frame is undefined for rectilinear trajectories. This cannot occur during normal rendezvous operations; if it does, a program alarm should have been raised upstream. |
| `norm(r_active - r_target) == 0.0` (same position) | `range_rate`, `los_angles_lvlh` | Panic. Range == 0 means the vehicles are at the same point in space; this is physically catastrophic. A downstream caller that needs to guard against near-zero range should check before calling. A sentinel float (e.g. `f64::INFINITY`) would mask a real error; panicking is correct. |
| `dot(rho_dot, rho_dot) == 0.0` (zero relative velocity) | `time_to_closest_approach` | Panic. If relative velocity is exactly zero, TCA is undefined (the range is constant). Callers should check `range_rate != 0.0` before requesting TCA and present "N/A" on the DSKY if not meaningful. |
| `range_rate > 0.0` and `time_to_closest_approach < 0.0` | `time_to_closest_approach` | Return negative TCA; no special case. This means closest approach was in the past (vehicles already diverging). The caller is responsible for interpreting the sign. |
| Very small `norm(rho_dot)` (near-stationary relative motion) | `time_to_closest_approach` | The linear formula returns a large magnitude TCA. No special case; the limitation is documented in the function's docstring. Callers using TCA for crew display should apply a display cap (e.g. 99:59:59). |
| `norm(lvlh.rho) > 0` but `x == y == 0` (directly overhead/below) | `los_angles_lvlh` | `atan2(0.0, 0.0)` is 0.0 per IEEE 754; azimuth = 0 is returned as specified. Elevation = ±π/2. No panic. |

---

## 9. Test Cases

### TC-REND-1: Circular orbit baseline — `relative_state_lvlh`

**Setup**: Target in a circular equatorial orbit at 300 km altitude.
```
r_t = [R_E + 300e3, 0.0, 0.0]   where R_E = 6_371_000.0 m
mu  = 3.986_004_418e14 m³/s²
v_circ = sqrt(mu / (R_E + 300e3))  ≈ 7726.0 m/s
v_t = [0.0, v_circ, 0.0]

r_a = r_t + [1000.0, 0.0, 0.0]   (1 km behind radially)
v_a = v_t                          (same velocity)
```

**Expected**: `relative_state_lvlh(r_a, v_a, r_t, v_t).rho` ≈ `[-0.0, 0.0, -1000.0]`
(1 km in the -z direction, i.e. radially above the target in LVLH; z points toward Earth
so the chaser 1 km farther from Earth has z_lvlh ≈ -1000 m).

`rho_dot` ≈ `[0.0, 0.0, 0.0]` (same velocity → zero relative velocity in inertial frame).

**Tolerance**: component-wise error < 1 m, < 0.001 m/s.

---

### TC-REND-2: Circular orbit baseline — `range` and `range_rate`

**Setup**: Same orbit as TC-REND-1.
Active vehicle 2 km directly ahead (in-track, i.e., along the velocity direction):
```
r_a = r_t + [0.0, 0.0, 0.0]         (same radial position)
r_a = r_t                            (start), then offset along velocity:
```
More precisely: active vehicle at angular displacement Δθ = 2000/(R_E + 300e3) rad
ahead. To keep it simple in Cartesian coordinates:
```
r_a = r_t                               (coincident — edge case; skip)
```

Better setup: target in circular orbit as before; active vehicle displaced 2 km in
the in-track direction:
```
r_a = [R_E + 300e3, 2000.0, 0.0]        (2 km ahead in Y at t=0, approximately)
v_a = v_t + [0.0, 0.0, 0.0]
```

**Expected**:
- `range(r_a, r_t)` ≈ 2000.0 m (tolerance < 1 m).
- `range_rate(r_a, v_a, r_t, v_t)` == 0.0 (same velocity, orthogonal separation).

---

### TC-REND-3: Closing approach — `range_rate` sign

**Setup**:
```
r_t = [7_000_000.0, 0.0, 0.0]   (7 Mm from Earth centre)
v_t = [0.0, 7500.0, 0.0]
r_a = [7_010_000.0, 0.0, 0.0]   (10 km radially outside target)
v_a = [-20.0, 7500.0, 0.0]      (20 m/s radially inward toward target)
```
**Expected**: `range_rate` ≈ -20.0 m/s (closing; tolerance < 0.01 m/s).

---

### TC-REND-4: Line-of-sight angles — directly ahead

**Setup**: Target at origin of LVLH; active vehicle displaced 5000 m in the +x
(along-track) direction: `lvlh.rho = [5000.0, 0.0, 0.0]`.
**Expected**: `elevation` = 0.0 rad, `azimuth` = 0.0 rad (target is directly ahead,
on the local horizontal, along-track).

---

### TC-REND-5: Line-of-sight angles — directly overhead (radial)

**Setup**: `lvlh.rho = [0.0, 0.0, -3000.0]` (target 3 km radially above — z_lvlh < 0
means the target is farther from Earth than the chaser).
**Expected**: `elevation` = π/2 rad, `azimuth` = 0.0 rad.

---

### TC-REND-6: Line-of-sight angles — 45° below and 45° starboard

**Setup**: `lvlh.rho = [1000.0, 1000.0, 1000.0]` (equal components; z > 0 → target is
below the local horizontal, i.e. closer to Earth).
**Expected**:
- `elevation` = `atan2(-1000.0, sqrt(1000^2 + 1000^2))` = `atan2(-1000.0, 1414.2)` ≈ -0.6155 rad (-35.26°).
- `azimuth`   = `atan2(1000.0, 1000.0)` = π/4 rad (45°).

---

### TC-REND-7: Time to closest approach — future intercept

**Setup**:
```
r_t = [7_000_000.0, 0.0, 0.0]
v_t = [0.0, 7500.0, 0.0]
r_a = [7_000_000.0, 10_000.0, 0.0]   (10 km ahead in-track)
v_a = [0.0, 7500.0 - 10.0, 0.0]      (10 m/s slower → closing at 10 m/s in-track)
```
`rho = [0.0, 10_000.0, 0.0]`, `rho_dot = [0.0, -10.0, 0.0]`.
`dot(rho, rho_dot) = 0*0 + 10000*(-10) + 0*0 = -100_000`.
`dot(rho_dot, rho_dot) = 100`.
**Expected**: TCA = -(-100_000) / 100 = 1000.0 s (tolerance < 0.01 s).

---

### TC-REND-8: Time to closest approach — zero relative velocity (panic)

**Setup**: `r_a = r_t + [100.0, 0.0, 0.0]`, `v_a = v_t` (identical velocities).
**Expected**: `time_to_closest_approach` panics (zero denominator).

---

### TC-REND-9: `lvlh_matrix` orthonormality check

**Setup**: Any valid `(r_t, v_t)` pair (use TC-REND-1 target state).
**Expected**: `M * M^T` is the identity matrix to within 1e-12 component-wise.

---

### TC-REND-10: `relative_state_lvlh` — range from LVLH equals inertial range

**Setup**: Use TC-REND-3 inputs.
**Expected**: `norm(relative_state_lvlh(r_a, v_a, r_t, v_t).rho)` equals
`range(r_a, r_t)` to within 1e-6 m (rotation preserves vector magnitude).

---

## 10. Open Questions for Architect Review

1. **LVLH frame naming**: The existing `targeting-spec.md` uses "LVLH" for the RSW
   frame (R = outward radial). This module uses "LVLH" for the Hill frame (z = inward
   radial, z = -R). An architecture decision is needed: either rename one frame
   consistently across all specs (suggested: call the targeting frame `RswFrame` and
   this frame `LvlhFrame`), or add an explicit type annotation to the rotation matrix
   (`LvlhMatrix` newtype) to prevent silent misuse. Until resolved the specs use
   prose disambiguation ("rendezvous LVLH" vs. "RSW targeting frame").

2. **`StateVector` vs raw `Vec3` pairs**: The public API currently takes raw `Vec3`
   pairs rather than `StateVector` structs to keep this module dependency-minimal. If
   the architect prefers consistent use of `StateVector` throughout, all six functions
   should be revised to accept `&StateVector` arguments; this adds a dependency on
   `navigation::state_vector` and would add a frame-consistency assertion
   (`debug_assert_eq!(active.frame, target.frame)`).

3. **`f64::atan2` in `no_std`**: `atan2` requires `libm` on bare-metal. The existing
   `math::trig` module should confirm that `atan2` is part of its exported API
   (currently the spec lists only `sin`, `cos`, `asin`, `acos`). If not, `atan2` must
   be added to `math::trig` before this module can be implemented.

4. **Near-zero range guard**: The spec requires panicking on zero range for
   `range_rate` and `los_angles_lvlh`. Some rendezvous programs may wish to suppress
   display updates (rather than reset) when range is below a noise threshold (e.g.,
   < 1 m). Whether this guard belongs here or in the program layer is an architectural
   judgment call.
