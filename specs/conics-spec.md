# Specification: `navigation/conics` Module

**Status**: Approved for implementation
**Module path**: `agc-core/src/navigation/conics.rs`
**Architecture reference**: `docs/architecture.md` §9.2 "Function Granularity", §9.4 "Gravity Model"
**State-vector reference**: `specs/state-vector-spec.md` §4.1 (Frame), §4.2 (StateVector)
**Gravity reference**: `specs/gravity-spec.md` §3 (MU_EARTH, MU_MOON, R_EARTH, R_MOON constants)
**Math reference**: `specs/linalg-spec.md` §4 (dot, cross, norm, unit, vscale)
**Kepler reference**: `agc-core/src/math/kepler.rs` (kepler_step)
**AGC source files**:
- `Comanche055/CONIC_SUBROUTINES.agc` — KEPRTN, REVUP, HANGLE, orbital element routines
**Spec checklist**: `specs/README.md` — all items satisfied (see §13)

---

## 1. Purpose and Scope

`navigation::conics` provides the classical orbital mechanics layer of the Apollo
Guidance Computer's navigation system. It converts between the two primary
representations of a spacecraft's trajectory: the **state vector** (position and
velocity at an epoch, in `navigation::state_vector`) and the **Keplerian orbital
elements** (the six constants that define a conic section trajectory in a central
gravitational field).

This module is called on demand — not at a fixed cycle rate — whenever a program
or targeting computation needs to express the current trajectory in terms of
orbital geometry: apoapsis and periapsis altitudes for crew display (P11, P21),
orbital period for ground-track prediction (P21), and element propagation for
rendezvous targeting (P31–P34) and midcourse correction (P37).

### What this module provides

- `OrbitalElements` struct: the six classical Keplerian elements plus epoch and
  frame tag.
- `state_to_elements` — convert a `StateVector` to `OrbitalElements` (the orbital
  mechanics inverse of `elements_to_state`).
- `elements_to_state` — convert `OrbitalElements` back to a `StateVector`.
- `orbital_period` — compute the period of an elliptic orbit from its elements.
- `periapsis_radius` and `apoapsis_radius` — compute apse radii from elements.
- `periapsis_altitude_earth`, `apoapsis_altitude_earth` — altitude above
  `R_EARTH` surface.
- `periapsis_altitude_moon`, `apoapsis_altitude_moon` — altitude above `R_MOON`
  surface.
- Re-export of `kepler_step` from `math::kepler` for callers that only need conic
  propagation without the full elements conversion.

### What this module does NOT provide

- Numerical integration. Cowell and Encke integration are in
  `navigation::integration`.
- Kepler equation solving. The universal-variable propagator is in
  `math::kepler::kepler_step`; `conics` re-exports it but does not re-implement
  it.
- Gravity computation. Gravitational acceleration vectors are in
  `navigation::gravity`.
- Orbit determination from multiple observations (e.g., Gauss method). The AGC
  did not perform onboard orbit determination from raw sensor data; this port
  follows the same boundary.
- Lambert's problem (two-point boundary value). That is in `math::lambert` and
  `guidance::lambert`.
- Non-Keplerian perturbations. J2 oblateness and third-body terms are modeled in
  `navigation::gravity` and applied by the integrator. The elements computed here
  are osculating (instantaneous) Keplerian elements, not mean elements.

---

## 2. AGC Background

### 2.1 Conic Subroutines in Comanche055

The original AGC codebase contained a set of conic-section trajectory routines
grouped in `Comanche055/CONIC_SUBROUTINES.agc`. The primary routines relevant to
this module are:

**KEPRTN** — The universal-variable Kepler propagator. Given a state vector
`(R, V)` and a time interval `dt`, it advanced the state along the best-fit
Keplerian conic using Battin's universal variable formulation. This is the AGC's
equivalent of `math::kepler::kepler_step`. KEPRTN was the most computationally
intensive routine in the guidance software; it used iterative root-finding
(Halley's method on Kepler's equation in the universal variable `chi`) with
convergence checked against a tolerance. In the Rust port, this routine is
implemented separately in `math::kepler` and re-exported here.

**HANGLE / REVUP** — Subroutines used to compute the orbital period and the
number of revolutions completed by a spacecraft in a given time, for the P21
ground-track display program. `REVUP` computed `dt / T` (number of orbits
elapsed) and the remaining fractional orbit angle. In this port, `orbital_period`
and a caller-level division provide the equivalent.

**Orbital element extraction** — Comanche055 computed apoapsis/periapsis altitudes
for crew display (P11 "Earth orbit insertion monitor" and the rendezvous programs)
by extracting the vis-viva energy, angular momentum magnitude, and eccentricity
from the current state vector. These intermediate quantities map directly to the
classical elements `a` and `e`.

### 2.2 Classical Elements in the AGC Context

The AGC did not maintain a persistent `OrbitalElements` struct in erasable memory.
Instead, the individual derived quantities (`a`, `e`, altitude) were computed
transiently from the current `RN`/`VN` state vector when needed and then
discarded. In the Rust port, `OrbitalElements` is a transient value type (`Copy`)
that serves the same role: it is computed on demand from a `StateVector`, used
for the required calculation, and not stored between calls.

The epoch carried by `OrbitalElements` in this port corresponds to the epoch of
the source `StateVector`, which itself corresponds to the AGC's `TEPHEM` register.

### 2.3 AGC Fixed-Point Encoding of Orbital Parameters

The AGC stored orbital-mechanics intermediate quantities in its standard
double-precision fixed-point format. The relevant scale factors for verification
of test cases are:

| Quantity | AGC scale | Physical meaning |
|----------|-----------|-----------------|
| Radius (`r`) | B+28 m | 1 DP LSB ≈ 1 m |
| Velocity (`v`) | B+7 m/s | 1 DP LSB ≈ 7.6 × 10⁻⁴ m/s |
| Specific energy (`ε`) | derived from above | m²/s² |
| Angular momentum (`h`) | B+35 m²/s | derived from r × v |
| Semi-major axis (`a`) | B+28 m | same scale as position |

In the Rust port all values are plain `f64` SI units. The scale factors are
documented here to allow fixture tests built from AGC erasable-memory dumps to
convert raw words to `f64` for comparison.

---

## 3. Constants Used

All constants are imported from `navigation::gravity`. No new constants are
defined in this module.

```rust
use crate::navigation::gravity::{MU_EARTH, MU_MOON, R_EARTH, R_MOON};
```

| Constant | Value | Use in this module |
|----------|-------|-------------------|
| `MU_EARTH` | 3.986_004_418 × 10¹⁴ m³/s² | Default `mu` for Earth-inertial state vectors |
| `MU_MOON` | 4.902_800_118 × 10¹² m³/s² | Default `mu` for Moon-inertial state vectors |
| `R_EARTH` | 6_378_137.0 m | Reference radius for Earth altitude helpers |
| `R_MOON` | 1_737_400.0 m | Reference radius for Moon altitude helpers |

The tolerance for detecting degenerate cases (circular, equatorial) is a
module-level constant:

```rust
/// Eccentricity below which an orbit is treated as circular (ω and ν undefined).
const CIRCULAR_ECC_TOL: f64 = 1.0e-6;

/// sin(i) below which an orbit is treated as equatorial (Ω undefined).
const EQUATORIAL_INC_TOL: f64 = 1.0e-6;
```

---

## 4. Data Structure: `OrbitalElements`

### 4.1 Declaration

```rust
// agc-core/src/navigation/conics.rs

/// Classical Keplerian orbital elements for a two-body conic trajectory.
///
/// These are **osculating elements**: they describe the instantaneous
/// best-fit Keplerian conic at `epoch`. For a perturbed trajectory (J2,
/// third-body) the elements change slowly with time; they are not conserved
/// between calls to `state_to_elements` at different epochs.
///
/// Convention: right-handed ECI or MCI frame consistent with the source
/// `StateVector::frame`. Angles are in radians.
#[derive(Clone, Copy, Debug)]
pub struct OrbitalElements {
    /// Semi-major axis (metres).
    ///
    /// Positive for elliptic orbits (e < 1), negative for hyperbolic orbits
    /// (e > 1). Zero for parabolic orbits (e = 1) is not representable; the
    /// function returns an error for that degenerate case (see §8).
    ///
    /// Derived from specific orbital energy: a = -μ / (2ε), where
    /// ε = v²/2 - μ/r.
    pub a: f64,

    /// Eccentricity (dimensionless, ≥ 0).
    ///
    /// e = 0: circular.  0 < e < 1: elliptic.  e = 1: parabolic (error case).
    /// e > 1: hyperbolic.
    pub e: f64,

    /// Inclination (radians, range [0, π]).
    ///
    /// Angle between the orbital plane and the equatorial plane of the
    /// reference body. i = 0 is a prograde equatorial orbit; i = π is
    /// a retrograde equatorial orbit.
    pub i: f64,

    /// Right ascension of the ascending node (RAAN, radians, range [0, 2π)).
    ///
    /// The angle in the equatorial plane from the X-axis (vernal equinox or
    /// ECI/MCI reference direction) to the ascending node vector.
    ///
    /// Undefined for equatorial orbits (sin(i) ≈ 0). When the orbit is
    /// equatorial, this field is set to 0.0 and the caller must check
    /// the `is_equatorial()` helper before using Ω.
    pub raan: f64,  // Ω

    /// Argument of periapsis (radians, range [0, 2π)).
    ///
    /// The angle in the orbital plane from the ascending node to the periapsis
    /// direction, measured in the direction of motion.
    ///
    /// Undefined for circular orbits (e ≈ 0). When the orbit is circular,
    /// this field is set to 0.0 and the caller must check the `is_circular()`
    /// helper before using ω.
    ///
    /// For equatorial orbits the argument of periapsis is measured from the
    /// X-axis directly (longitude of periapsis), not from the ascending node.
    pub aop: f64,   // ω

    /// True anomaly at epoch (radians, range [0, 2π)).
    ///
    /// The angle in the orbital plane from the periapsis direction to the
    /// current position, measured in the direction of motion.
    ///
    /// For circular orbits where ω is undefined, ν is measured from the
    /// ascending node (argument of latitude).
    /// For equatorial circular orbits, ν is measured from the X-axis
    /// (true longitude).
    pub nu: f64,    // ν

    /// Mission elapsed time at which these elements are valid.
    ///
    /// Copied directly from the source `StateVector::epoch`.
    /// 1 unit = 1 centisecond = 0.01 s.
    pub epoch: crate::types::Met,

    /// Coordinate frame of the source state vector.
    ///
    /// Determines which gravitating body (Earth or Moon) these elements
    /// describe a trajectory around. Must be `EarthInertial` or
    /// `MoonInertial`; never `StableMember`.
    pub frame: crate::navigation::state_vector::Frame,
}
```

### 4.2 Invariants

1. `e >= 0.0` always.
2. `i` is in `[0.0, PI]`.
3. `raan` is in `[0.0, TAU)` (normalised modulo 2π).
4. `aop` is in `[0.0, TAU)` (normalised modulo 2π).
5. `nu` is in `[0.0, TAU)` (normalised modulo 2π) for elliptic orbits; for
   hyperbolic orbits `nu` is in `(-nu_inf, +nu_inf)` where
   `nu_inf = acos(-1/e)`, and is not wrapped.
6. For a circular orbit (`e < CIRCULAR_ECC_TOL`): `aop = 0.0`, `nu` is argument
   of latitude (angle from ascending node) or true longitude if also equatorial.
7. For an equatorial orbit (`i.sin().abs() < EQUATORIAL_INC_TOL`): `raan = 0.0`.
8. `frame` is either `Frame::EarthInertial` or `Frame::MoonInertial`.

### 4.3 Helper Methods

```rust
impl OrbitalElements {
    /// Returns true when the orbit is circular within tolerance.
    ///
    /// When true, `aop` is meaningless and `nu` is argument of latitude
    /// or true longitude.
    pub fn is_circular(&self) -> bool {
        self.e < CIRCULAR_ECC_TOL
    }

    /// Returns true when the orbit is equatorial within tolerance.
    ///
    /// When true, `raan` is meaningless (set to 0.0). For a non-circular
    /// equatorial orbit, `aop` is the longitude of periapsis measured from
    /// the X-axis.
    pub fn is_equatorial(&self) -> bool {
        self.i.sin().abs() < EQUATORIAL_INC_TOL
    }

    /// Returns true when the orbit is hyperbolic (e > 1).
    pub fn is_hyperbolic(&self) -> bool {
        self.e >= 1.0
    }

    /// Returns the gravitational parameter appropriate for this frame.
    ///
    /// Selects MU_EARTH for EarthInertial, MU_MOON for MoonInertial.
    /// Panics if frame is StableMember (programming error).
    pub fn mu(&self) -> f64 {
        use crate::navigation::state_vector::Frame;
        use crate::navigation::gravity::{MU_EARTH, MU_MOON};
        match self.frame {
            Frame::EarthInertial => MU_EARTH,
            Frame::MoonInertial  => MU_MOON,
            Frame::StableMember  => panic!("OrbitalElements::mu: StableMember frame"),
        }
    }
}
```

---

## 5. Function Specifications

### 5.1 `state_to_elements`

```rust
pub fn state_to_elements(sv: StateVector, mu: f64) -> OrbitalElements
```

#### Purpose

Convert a Cartesian state vector to classical Keplerian orbital elements.
This is the primary way to obtain orbital geometry from the navigation state.

Called by:
- P11 (Earth orbit insertion monitor): to compute `a`, `e`, periapsis altitude,
  and apoapsis altitude for crew display.
- P21 (ground track): to compute `a` and `i` for ground-track period and
  inclination display.
- P31–P34 (rendezvous maneuver targeting): to characterise the current orbit
  and the target orbit before delta-V computation.
- P37 (return to Earth): to estimate the transfer orbit geometry.

#### Preconditions

- `sv.frame` is `EarthInertial` or `MoonInertial`. `StableMember` is a
  programming error; the function panics.
- `sv.position` is not the zero vector (spacecraft is not at the centre of the
  gravitating body). If `norm(sv.position) < 1.0 m`, the function panics.
- `mu > 0.0`.
- The state vector represents a non-parabolic orbit (`|e - 1| > 1e-6`). For
  parabolic orbits the semi-major axis is infinite; the function panics with a
  descriptive message (parabolic trajectories do not occur in Apollo missions).

#### Algorithm

The conversion follows the standard six-step classical orbital elements
extraction (Bate, Mueller & White, "Fundamentals of Astrodynamics", §2.4):

**Step 1 — Scalars.**
```
r  = norm(sv.position)          // m
v  = norm(sv.velocity)          // m/s
vr = dot(sv.position, sv.velocity) / r  // radial velocity, m/s
```

**Step 2 — Specific angular momentum vector.**
```
h_vec = cross(sv.position, sv.velocity)   // m²/s
h     = norm(h_vec)                       // m²/s
```
If `h < 1.0 m²/s`, the trajectory is rectilinear (zero angular momentum,
purely radial flight). This is degenerate; the function panics.

**Step 3 — Node vector (ascending node direction).**
```
k      = [0.0, 0.0, 1.0]            // Z-axis unit vector
n_vec  = cross(k, h_vec)            // points toward ascending node
n      = norm(n_vec)                 // zero for equatorial orbits
```

**Step 4 — Eccentricity vector (points toward periapsis).**
```
e_vec = (1/mu) * ((v² - μ/r)*r_vec - vr*v_vec)
      = vscale(1.0/mu,
               vsub(
                 vscale(v*v - mu/r, sv.position),
                 vscale(vr * r,     sv.velocity)
               ))
e     = norm(e_vec)
```
Alternatively (numerically equivalent, sometimes more stable):
```
e_vec = cross(sv.velocity, h_vec) / mu  -  unit(sv.position)
```

**Step 5 — Semi-major axis from specific energy.**
```
eps = v*v / 2.0  -  mu / r      // specific orbital energy, m²/s²
a   = -mu / (2.0 * eps)         // m  (negative for hyperbolic)
```
If `|eps| < 1.0e-6 * mu / r_typical` (parabolic orbit), panic.

**Step 6 — Inclination.**
```
i = acos(h_vec[2] / h)          // radians, range [0, π]
```
`h_vec[2]` is the Z-component of the angular momentum vector. Uses
`f64::acos`; result is in [0, π] without further clamping because
`h_vec[2] / h` is guaranteed to be in [-1, 1] when `h > 0`.

**Step 7 — RAAN.**
```
if n < EQUATORIAL_INC_TOL * h {
    // equatorial orbit: RAAN is undefined
    raan = 0.0;
} else {
    raan = acos(n_vec[0] / n);   // radians
    if n_vec[1] < 0.0 { raan = TAU - raan; }
}
```

**Step 8 — Argument of periapsis.**
```
if e < CIRCULAR_ECC_TOL {
    // circular orbit: AoP is undefined
    aop = 0.0;
} else if n < EQUATORIAL_INC_TOL * h {
    // equatorial non-circular: aop is longitude of periapsis from X-axis
    aop = acos(e_vec[0] / e);
    if e_vec[1] < 0.0 { aop = TAU - aop; }
} else {
    aop = acos(dot(n_vec, e_vec) / (n * e));
    if e_vec[2] < 0.0 { aop = TAU - aop; }  // periapsis below equatorial plane
}
```

**Step 9 — True anomaly.**
```
if e < CIRCULAR_ECC_TOL && n >= EQUATORIAL_INC_TOL * h {
    // circular non-equatorial: nu = argument of latitude
    nu = acos(dot(n_vec, sv.position) / (n * r));
    if sv.velocity[2] < 0.0 { nu = TAU - nu; }  // descending
} else if e < CIRCULAR_ECC_TOL && n < EQUATORIAL_INC_TOL * h {
    // circular equatorial: nu = true longitude
    nu = acos(sv.position[0] / r);
    if sv.velocity[0] > 0.0 { nu = TAU - nu; }
} else {
    nu = acos(dot(e_vec, sv.position) / (e * r));
    if vr < 0.0 { nu = TAU - nu; }  // past periapsis (radial velocity outward)
}
```
For hyperbolic orbits, `nu` may be negative (pre-periapsis approach). Retain
the signed value; do not wrap modulo 2π.

#### Postconditions

- `result.e >= 0.0`
- `result.i` in `[0, π]`
- `result.a < 0.0` iff the orbit is hyperbolic
- `result.epoch == sv.epoch`
- `result.frame == sv.frame`

#### Numerical Notes

The `acos` argument must be clamped to `[-1.0, 1.0]` before the call to guard
against floating-point rounding producing values marginally outside the domain:

```rust
fn safe_acos(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).acos()
}
```

This internal helper is not public. Use it at every `acos` call site within this
function.

---

### 5.2 `elements_to_state`

```rust
pub fn elements_to_state(el: OrbitalElements, mu: f64) -> StateVector
```

#### Purpose

Convert Keplerian orbital elements back to a Cartesian state vector. This is the
inverse of `state_to_elements`, used when it is more convenient to specify or
propagate an orbit in element form and then recover the state vector for
integration or display.

Called by rendezvous targeting (P31–P34) to construct the desired target orbit
state from planned orbital elements, and by simulation test harnesses to set up
initial conditions.

#### Preconditions

- `el.frame` is `EarthInertial` or `MoonInertial`. Panics for `StableMember`.
- `mu > 0.0`.
- For elliptic orbits: `el.a > 0.0` and `el.e < 1.0`.
- For hyperbolic orbits: `el.a < 0.0` and `el.e > 1.0`.
- Parabolic orbits (`el.e == 1.0`) are not supported; function panics.
- `el.i` is in `[0, π]`; `el.raan` is in `[0, 2π)`; `el.aop` is in `[0, 2π)`.
- For hyperbolic orbits, `el.nu` must satisfy `|nu| < acos(-1/e)` (the vehicle
  must be on the physical trajectory, not in the unphysical region beyond the
  asymptote).

#### Algorithm

**Step 1 — Semi-latus rectum.**
```
p = a * (1 - e²)   // m, for elliptic
p = a * (e² - 1)   // m, for hyperbolic (a < 0)
```
Unified: `p = a * (1.0 - e * e)` — this produces the correct positive `p`
for both cases since `a < 0` and `e > 1` gives `(e²-1) > 0` and `a*(1-e²)` =
`(-|a|)*(1-e²)` = `|a|*(e²-1) > 0`. Verify: for hyperbolic `a = -|a|`, `e>1`,
`1 - e² < 0`, so `p = (-|a|)(1-e²) = |a|(e²-1) > 0`. Correct.

**Step 2 — Position and velocity in perifocal frame (PQW).**

The perifocal frame has:
- P-axis: pointing toward periapsis (along e_vec direction)
- Q-axis: 90° ahead in the direction of motion
- W-axis: along angular momentum (h_vec direction = P × Q)

```
r_pqw = [p * cos(nu) / (1 + e * cos(nu)),
          p * sin(nu) / (1 + e * cos(nu)),
          0.0]

v_pqw = [sqrt(mu / p) * (-sin(nu)),
          sqrt(mu / p) * (e + cos(nu)),
          0.0]
```

**Step 3 — Rotation matrix from perifocal to inertial frame.**

The rotation is a composition of three Euler angle rotations:
1. Rotation by `-ω` (argument of periapsis) about the W-axis of the orbital
   plane (aligns P-axis with the ascending node direction).
2. Rotation by `-i` (inclination) about the intermediate X-axis (tilts the
   orbital plane).
3. Rotation by `-Ω` (RAAN) about the Z-axis of the inertial frame (rotates to
   the correct nodal longitude).

In matrix form (Bate §2.6, or Vallado §2.6), the rotation matrix R from
perifocal to inertial (ECI/MCI) is:

```
R[0][0] =  cos(Ω)cos(ω) - sin(Ω)sin(ω)cos(i)
R[0][1] = -cos(Ω)sin(ω) - sin(Ω)cos(ω)cos(i)
R[0][2] =  sin(Ω)sin(i)

R[1][0] =  sin(Ω)cos(ω) + cos(Ω)sin(ω)cos(i)
R[1][1] = -sin(Ω)sin(ω) + cos(Ω)cos(ω)cos(i)
R[1][2] = -cos(Ω)sin(i)

R[2][0] =  sin(ω)sin(i)
R[2][1] =  cos(ω)sin(i)
R[2][2] =  cos(i)
```

where Ω = `el.raan`, ω = `el.aop`, i = `el.i`.

**Step 4 — Apply rotation.**
```
position = linalg::mxv(R, r_pqw)   // Vec3, metres
velocity = linalg::mxv(R, v_pqw)   // Vec3, m/s
```

**Step 5 — Assemble StateVector.**
```rust
StateVector {
    position,
    velocity,
    epoch: el.epoch,
    frame: el.frame,
}
```

#### Postconditions

A round-trip through `state_to_elements` → `elements_to_state` must recover the
original state vector to within numerical precision:

```
|elements_to_state(state_to_elements(sv, mu), mu).position - sv.position| < 1.0 m
|elements_to_state(state_to_elements(sv, mu), mu).velocity - sv.velocity| < 0.01 m/s
```

These tolerances account for accumulated floating-point rounding through two
`acos` evaluations and the rotation matrix construction.

---

### 5.3 `orbital_period`

```rust
pub fn orbital_period(el: &OrbitalElements, mu: f64) -> f64
```

#### Purpose

Compute the orbital period of an elliptic or circular orbit from its elements.
Used by P21 (ground-track display program) to compute the number of complete
orbits and the fractional orbit remaining for a given time interval; also used
by rendezvous programs for phasing calculations.

The corresponding AGC routine is `HANGLE`/`REVUP` in `CONIC_SUBROUTINES.agc`.

#### Preconditions

- `el.e < 1.0` (elliptic or circular orbit). Panics if `el.is_hyperbolic()`,
  since hyperbolic trajectories do not have a period.
- `el.a > 0.0` (implied by `e < 1.0` for a well-formed element set).
- `mu > 0.0`.

#### Algorithm

```
T = 2π × sqrt(a³ / μ)     [seconds]
```

Using `libm::sqrt` for `no_std` compatibility (consistent with `linalg::norm`):

```rust
pub fn orbital_period(el: &OrbitalElements, mu: f64) -> f64 {
    assert!(!el.is_hyperbolic(), "orbital_period: undefined for hyperbolic orbit");
    core::f64::consts::TAU * libm::sqrt(el.a * el.a * el.a / mu)
}
```

#### Return value

Period in seconds (SI). For a circular LEO at 400 km altitude the result is
approximately 5559 s (≈ 92.7 minutes), consistent with the ISS orbital period.

---

### 5.4 `periapsis_radius`

```rust
pub fn periapsis_radius(el: &OrbitalElements) -> f64
```

Compute the periapsis radius (distance from the centre of the gravitating body
to the closest approach point) in metres.

```
r_p = a × (1 - e)
```

For hyperbolic orbits `a < 0` and `e > 1`, so `1 - e < 0` and
`a × (1 - e) = (-|a|)(1 - e) = |a|(e - 1) > 0`. The formula is numerically
correct for both elliptic and hyperbolic orbits.

Preconditions: `el.e >= 0.0`, `el.a != 0.0`.

Return value: metres (≥ 0).

---

### 5.5 `apoapsis_radius`

```rust
pub fn apoapsis_radius(el: &OrbitalElements) -> f64
```

Compute the apoapsis radius in metres.

```
r_a = a × (1 + e)
```

Preconditions: `el.e < 1.0` (elliptic or circular orbit). Panics for hyperbolic
orbits because hyperbolic trajectories have no apoapsis.

Return value: metres (≥ `periapsis_radius`).

---

### 5.6 `periapsis_altitude_earth`

```rust
pub fn periapsis_altitude_earth(el: &OrbitalElements) -> f64
```

Altitude of periapsis above the Earth's equatorial surface in metres.

```
h_p = periapsis_radius(el) - R_EARTH
```

Note: this is radius minus the reference spherical radius. For re-entry
computation it provides the closest-approach altitude to the reference
ellipsoid; for crew display (P11 NOUN 44) it is the standard apogee/perigee
display quantity. The result may be negative if the trajectory intersects the
Earth's surface.

---

### 5.7 `apoapsis_altitude_earth`

```rust
pub fn apoapsis_altitude_earth(el: &OrbitalElements) -> f64
```

Altitude of apoapsis above the Earth's equatorial surface:

```
h_a = apoapsis_radius(el) - R_EARTH
```

Panics for hyperbolic orbits (no apoapsis). For crew display this is the
standard apogee altitude shown by P11.

---

### 5.8 `periapsis_altitude_moon`

```rust
pub fn periapsis_altitude_moon(el: &OrbitalElements) -> f64
```

Altitude of periapsis above the Moon's mean surface:

```
h_p = periapsis_radius(el) - R_MOON
```

Called when `el.frame == Frame::MoonInertial`. The caller is responsible for
ensuring the frame is appropriate; this function does not check the frame.

---

### 5.9 `apoapsis_altitude_moon`

```rust
pub fn apoapsis_altitude_moon(el: &OrbitalElements) -> f64
```

Altitude of apoapsis above the Moon's mean surface:

```
h_a = apoapsis_radius(el) - R_MOON
```

Panics for hyperbolic orbits. Called during lunar orbit insertion (P30)
and translunar trajectory displays.

---

### 5.10 Re-export of `kepler_step`

```rust
pub use crate::math::kepler::kepler_step;
```

This re-export makes `kepler_step` available under the `navigation::conics`
namespace for callers who import all conic-trajectory tools from one place.
The function is implemented in `math::kepler`. See `agc-core/src/math/kepler.rs`
for the full specification of the universal-variable propagator. No additional
wrapping or dispatch is performed here.

---

## 6. Mu Dispatch Helpers

To simplify the common call pattern where the caller has a `StateVector` but
must supply the correct `mu` for the active frame, the following free functions
are provided:

```rust
/// Select the gravitational parameter appropriate for `sv.frame`.
///
/// Returns MU_EARTH for EarthInertial, MU_MOON for MoonInertial.
/// Panics for StableMember (programming error).
pub fn mu_for_frame(frame: Frame) -> f64 {
    use crate::navigation::gravity::{MU_EARTH, MU_MOON};
    match frame {
        Frame::EarthInertial => MU_EARTH,
        Frame::MoonInertial  => MU_MOON,
        Frame::StableMember  => panic!("mu_for_frame: StableMember has no gravity body"),
    }
}

/// Convert a StateVector to OrbitalElements, automatically selecting mu
/// from the state vector's frame.
///
/// Equivalent to `state_to_elements(sv, mu_for_frame(sv.frame))`.
pub fn sv_to_elements(sv: StateVector) -> OrbitalElements {
    state_to_elements(sv, mu_for_frame(sv.frame))
}
```

These helpers are used by P11 and P21 which always operate on the current
navigation state vector and therefore always know `sv.frame`.

---

## 7. Callers and Call Sites

| Caller module | Function used | Purpose |
|---------------|--------------|---------|
| `programs::p11` | `sv_to_elements`, `periapsis_altitude_earth`, `apoapsis_altitude_earth` | Display NOUN 44 (apo/peri altitudes) for Earth orbit insertion monitor |
| `programs::p20_p22` (P21) | `sv_to_elements`, `orbital_period`, `apoapsis_altitude_earth`, `periapsis_altitude_earth` | Ground-track display: period, inclination, node crossing time |
| `programs::p30` | `sv_to_elements`, `elements_to_state` | External delta-V targeting: characterise pre-burn orbit |
| `programs::p31_p34` | `sv_to_elements`, `orbital_period`, `periapsis_radius`, `apoapsis_radius` | Rendezvous targeting: phasing orbit calculation |
| `programs::p37` | `sv_to_elements`, `elements_to_state` | Return-to-Earth transfer orbit geometry |
| `navigation::integration` | `kepler_step` (via re-export) | Coast propagation on demand |
| Test harnesses | `elements_to_state` | Setting up orbital initial conditions |

---

## 8. Edge Cases and Error Handling

The module uses panics (rather than `Result`) for degenerate inputs, consistent
with the rest of the navigation codebase (see `docs/architecture.md` §3, which
establishes that navigation errors trigger the restart handler). All panics carry
a descriptive message string.

| Condition | Detection | Response |
|-----------|-----------|----------|
| `sv.frame == StableMember` | `match` on frame in `state_to_elements` | `panic!("state_to_elements: StableMember frame")` |
| Zero position vector | `norm(sv.position) < 1.0` | `panic!("state_to_elements: position is zero")` |
| Rectilinear trajectory (`h ≈ 0`) | `norm(h_vec) < 1.0` | `panic!("state_to_elements: zero angular momentum (rectilinear)")` |
| Parabolic orbit (`e ≈ 1`) | `(e - 1.0).abs() < 1e-6` | `panic!("state_to_elements: parabolic orbit not supported")` |
| `orbital_period` on hyperbolic orbit | `el.is_hyperbolic()` | `panic!("orbital_period: undefined for hyperbolic orbit")` |
| `apoapsis_radius` on hyperbolic orbit | `el.is_hyperbolic()` | `panic!("apoapsis_radius: undefined for hyperbolic orbit")` |
| `mu <= 0.0` | explicit check at entry | `panic!("conics: mu must be positive")` |

### Circular orbits (`e < CIRCULAR_ECC_TOL`)

The eccentricity vector `e_vec` has magnitude ≈ 0. Computing `unit(e_vec)` would
be numerically undefined. The algorithm avoids this by:
- Setting `aop = 0.0` unconditionally.
- Defining `nu` as argument of latitude (angle from ascending node direction to
  position vector) for inclined orbits, or as true longitude for equatorial
  orbits.
- Documenting this in the `OrbitalElements` field comments.

Callers that display ω or ν must call `el.is_circular()` before interpreting
those fields. P21 and P11 only display altitude and period; neither displays ω
or ν directly, so this edge case does not affect crew displays.

### Equatorial orbits (`sin(i) < EQUATORIAL_INC_TOL`)

The node vector `n_vec = cross(k, h_vec)` has magnitude ≈ 0 when h is nearly
parallel to the Z-axis (equatorial orbit). The algorithm avoids this by:
- Setting `raan = 0.0` unconditionally.
- Redefining `aop` as the longitude of periapsis (angle from X-axis to e_vec in
  the equatorial plane) when the orbit is non-circular.
- Documenting this in the `OrbitalElements` field comments.

Apollo Command Module parking orbits were always inclined (typically 28°–32°
for Kennedy Space Center launches, or up to 33° for lunar missions), so this
case arises only in test scenarios, not in nominal flight.

### Hyperbolic orbits (`e > 1`, `a < 0`)

All functions except `apoapsis_radius` and `orbital_period` work correctly for
hyperbolic orbits without modification. The `state_to_elements`/`elements_to_state`
round-trip is valid. The transearth and translunar coast arcs in the original
Apollo missions were hyperbolic with respect to the Moon (Moon SOI entry and
exit), so this case does appear in nominal operations.

For a hyperbolic orbit, `nu` is in the range `(-nu_inf, +nu_inf)` where
`nu_inf = acos(-1/e)`. The `nu` field is not wrapped modulo 2π for hyperbolic
orbits. Callers that display `nu` for crew use must be aware of this.

---

## 9. AGC Scale Factors and `f64` Conversion Table

Provided for authors of fixture tests that compare against AGC erasable-memory
dumps from VirtualAGC simulation runs.

| Quantity | AGC scale factor | Conversion to `f64` SI | Example (LEO, 400 km circular) |
|----------|-----------------|------------------------|-------------------------------|
| Position component | B+28 m | `w_dp × 2^28` m | r ≈ 6.778 × 10⁶ m |
| Velocity component | B+7 m/s | `w_dp × 2^7` m/s | v ≈ 7672 m/s |
| Semi-major axis | B+28 m | same as position | a ≈ 6.778 × 10⁶ m |
| Eccentricity | B-0 (dimensionless) | `w_dp` directly | e = 0.0 |
| Inclination | half-revolutions | `w_dp × π` rad | i = 0.5236 rad (30°) |
| RAAN | half-revolutions | `w_dp × π` rad | Ω = varies |
| Period | centiseconds | `w × 0.01` s | T ≈ 5559 s |

The `w_dp` notation means the double-precision fixed-point value:
`w_dp = w_hi × 2^-14 + w_lo × 2^-28` (dimensionless fraction).

---

## 10. `no_std` Constraints

This module compiles in the `no_std` environment of `agc-core`. The following
rules apply:

- All trigonometric functions (`acos`, `asin`, `atan2`, `sin`, `cos`, `sqrt`)
  must be called as `libm::acos`, `libm::sqrt`, etc. — not as `f64::acos()`.
  This is consistent with the convention established in `math::linalg` (see
  `specs/linalg-spec.md` §3).
- No heap allocation. All intermediate vectors are stack-allocated `[f64; 3]`
  arrays (the `Vec3` type alias).
- No `use std::...`. The module must only use `core::` and `libm::`.
- `core::f64::consts::TAU` and `core::f64::consts::PI` are used for π and 2π.

The `libm` dependency is already declared in `agc-core/Cargo.toml` by the
`navigation::gravity` and `math::linalg` modules.

---

## 11. Module-Level Imports and Structure

```rust
//! Conic section trajectory routines (Keplerian elements, orbit classification).
//!
//! Provides conversion between Cartesian state vectors and classical Keplerian
//! orbital elements, together with helper functions for orbital period, apse
//! radii, and altitude computations. Re-exports `kepler_step` from `math::kepler`
//! for callers who import all conic trajectory tools from one namespace.
//!
//! # AGC source reference
//!
//! AGC source: `Comanche055/CONIC_SUBROUTINES.agc`
//! Relevant routines: KEPRTN (propagation, re-exported as kepler_step),
//!   HANGLE/REVUP (period/revolutions, implemented as orbital_period),
//!   element extraction (implemented as state_to_elements).

use core::f64::consts::{PI, TAU};
use crate::types::{Vec3, Met};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::navigation::gravity::{MU_EARTH, MU_MOON, R_EARTH, R_MOON};
use crate::math::linalg::{dot, cross, norm, unit, vscale, vsub, vadd, mxv};

pub use crate::math::kepler::kepler_step;

const CIRCULAR_ECC_TOL:    f64 = 1.0e-6;
const EQUATORIAL_INC_TOL:  f64 = 1.0e-6;
```

---

## 12. Test Cases

All test cases use the `#[cfg(test)]` module and are compiled only for the
`std` test environment (not bare-metal). They use the `f64` SI values directly;
no AGC fixed-point encoding is required in the unit tests. Fixture tests that
compare against AGC memory dumps belong in `agc-test/tests/navigation_accuracy.rs`.

### Test Case 1: Circular LEO (400 km altitude, 28° inclination)

Representative of Apollo Earth parking orbit preparation.

```
r_vec = [6_778_137.0, 0.0, 0.0]  m   (on X-axis, at 400 km altitude)
v_vec = [0.0, v_c * cos(28°), v_c * sin(28°)]  m/s

v_c = sqrt(MU_EARTH / 6_778_137.0) ≈ 7669.8 m/s   (circular velocity at 400 km)
```

Expected elements:
- `a` ≈ 6_778_137.0 m (± 1 m)
- `e` ≈ 0.0 (< 1e-6)
- `i` ≈ 28.0° = 0.4887 rad (± 1e-5 rad)
- `orbital_period(el, MU_EARTH)` ≈ 5558 s (± 2 s)
- `periapsis_altitude_earth(el)` ≈ 400_000 m (± 1 m)
- `apoapsis_altitude_earth(el)` ≈ 400_000 m (± 1 m)
- Round-trip: `elements_to_state(state_to_elements(sv, MU_EARTH), MU_EARTH)` recovers
  position to within 1 m, velocity to within 0.01 m/s.

### Test Case 2: ISS-like Orbit (408 km × 416 km, 51.6° inclination)

Tests a slightly elliptic orbit with realistic inclination. This is the most
commonly referenced low Earth orbit in the mission planning context.

```
Perigee radius: R_EARTH + 408_000 = 6_786_137 m
Apogee  radius: R_EARTH + 416_000 = 6_794_137 m
i = 51.6° = 0.9006 rad, Ω = 45.0° = 0.7854 rad, ω = 90.0° = π/2, ν = 0.0 (at perigee)
```

Compute initial state with `elements_to_state`, then verify:
- `state_to_elements` round-trip recovers `a`, `e`, `i` to within their respective
  tolerances.
- `periapsis_altitude_earth` ≈ 408_000 m (± 100 m)
- `apoapsis_altitude_earth` ≈ 416_000 m (± 100 m)
- `orbital_period` ≈ 5563 s (± 5 s)

### Test Case 3: GTO Transfer Orbit (200 km × 35_786 km)

Tests a highly elliptic orbit, representative of the translunar injection
conic (the trans-lunar injection burn creates a trajectory with a lunar-distance
apogee). The GTO is a close analogue.

```
Perigee radius: R_EARTH + 200_000 = 6_578_137 m
Apogee  radius: R_EARTH + 35_786_000 = 42_164_137 m
i = 28.0°, Ω = 0.0, ω = 0.0, ν = 0.0 (at perigee)
```

Expected:
- `a` ≈ 24_371_137 m (± 1000 m)
- `e` ≈ 0.7258 (± 1e-4)
- `periapsis_altitude_earth` ≈ 200_000 m (± 100 m)
- `apoapsis_altitude_earth` ≈ 35_786_000 m (± 10_000 m)
- `orbital_period` ≈ 37_738 s ≈ 10.48 hours (± 10 s)
- `is_hyperbolic()` returns `false`

### Test Case 4: Lunar Parking Orbit (111 km circular, 0° inclination in MCI)

Representative of the Command Module's lunar orbit during the Apollo 11 mission.
Tests the `MoonInertial` frame path and the Moon altitude helpers.

```
frame = Frame::MoonInertial
r_vec = [R_MOON + 111_000, 0.0, 0.0]  m   (= [1_848_400, 0.0, 0.0] m)
v_c = sqrt(MU_MOON / (R_MOON + 111_000)) ≈ 1629.1 m/s (circular velocity)
v_vec = [0.0, 1629.1, 0.0]  m/s
```

Expected elements (`sv_to_elements` using `MU_MOON`):
- `a` ≈ 1_848_400 m (± 1 m)
- `e` < 1e-6 (circular)
- `i` ≈ 0.0 (equatorial — both `is_circular()` and `is_equatorial()` return true)
- `periapsis_altitude_moon(el)` ≈ 111_000 m (± 1 m)
- `apoapsis_altitude_moon(el)` ≈ 111_000 m (± 1 m)
- `orbital_period(el, MU_MOON)` ≈ 7127 s ≈ 118.8 minutes (± 2 s)

The `mu_for_frame(Frame::MoonInertial)` must return `MU_MOON`.

### Test Case 5: P21 Ground-Track Use Case (Inclined LEO, Period and RAAN)

P21 displays: orbital inclination, RAAN, and the time to next equatorial node
crossing. This test verifies that `state_to_elements` extracts meaningful `i`
and `raan` values for a realistic Apollo parking orbit scenario.

```
// Orbit: 185 km circular, 32° inclination, Ω = 125.4°, ω = 0° (circular), ν = 0°
a = R_EARTH + 185_000 = 6_563_137 m
e = 0.0, i = 32.0° = 0.5585 rad, Ω = 125.4° = 2.1888 rad
frame = Frame::EarthInertial
```

Build initial state with `elements_to_state`. Then call `sv_to_elements` and
verify:
- `el.i` is within 1e-4 rad of 0.5585 rad.
- `el.raan` is within 1e-4 rad of 2.1888 rad.
- `el.is_circular()` returns true.
- `orbital_period(el, MU_EARTH)` is within 5 s of `2π * sqrt(a³ / MU_EARTH)`.
- The number of orbits in 24 hours computed as `86400 / orbital_period(el, MU_EARTH)`
  is within 0.01 of the expected value (≈ 15.53 orbits/day), the quantity
  displayed by P21 for ground-track planning.

---

## 13. Spec Quality Checklist

- [x] AGC source file and line range referenced (`CONIC_SUBROUTINES.agc`: KEPRTN,
      HANGLE, REVUP)
- [x] All erasable variables and their AGC addresses listed (§2.2, §9 — TEPHEM
      epoch carried as `Met` in `StateVector`)
- [x] Scale factors documented for all fixed-point values (§2.3, §9)
- [x] Corresponding `f64` SI units documented (§3, §9)
- [x] Input/output preconditions and postconditions stated (§5.1–§5.9)
- [x] Edge cases and error handling specified (§8)
- [x] Five test cases with expected values (§12)
- [x] Rust API signatures designed: types (`Vec3`, `f64`, `Met`, `Frame`,
      `StateVector`), ownership (`Copy` by value throughout), lifetimes (none —
      all `Copy` types), `no_std` constraints (§10)
- [x] Invariants explicitly stated (§4.2)
- [x] Consistency with `docs/architecture.md` checked: §9.2 (function
      granularity), §9.4 (gravity model and body constants), §3.1 (f64 for
      navigation math), §3.3 (Vec3 / Mat3x3 type aliases)
