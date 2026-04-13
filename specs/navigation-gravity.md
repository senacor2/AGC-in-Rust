# Functional Specification: Gravity Models (`agc-core/src/navigation/gravity.rs`)

## AGC Source Reference

```
AGC source: Comanche055/ORBITAL_INTEGRATION.agc
Routines:   OBLATE (J2 oblateness acceleration, pages 1341-1343),
            GAMCOMP (gravity computation subroutine, pages 1338-1340),
            ACCOMP  (acceleration component dispatcher, pages 1337-1338),
            CALCGRAV (point-mass + J2 for SERVICER, page 835-836 in SERVICER207.agc),
            ITISMOON (Moon-gravity branch in CALCGRAV)
Pages:      1334-1354 (ORBITAL_INTEGRATION), 835-836 (SERVICER207)

AGC source: Comanche055/SERVICER207.agc
Routines:   CALCGRAV (lines immediately before CALCRVG)
Pages:      835-836

AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Labels:     PBODY (I(1) primary body selector), MOONFLAG (flag word 0, bit 12)
```

Constants authoritative source: `docs/agc-reference-constants.md`

---

## Behavior Summary

The AGC computed gravitational acceleration in two layers:

1. **Point-mass (Keplerian) gravity** — the dominant `−μ/r³ · r` term.
   Used in CALCGRAV (called by SERVICER every 2 seconds) and by the conic
   integration subroutines.

2. **J2 oblateness perturbation** — the OBLATE routine adds the first zonal
   harmonic of Earth's geopotential. It is applied during the full orbital
   integration (ORBITAL_INTEGRATION.agc / ACCOMP → OBLATE) and also in CALCGRAV
   via the `20J` and `2J` terms (J coefficient multiplied by unit position and
   Earth's rotation unit vector UNITW).

3. **Moon gravity** — when `MOONFLAG` is set (Moon-primary integration, or when
   computing third-body perturbation from the Moon), `MUEARTH` is replaced by
   `MUM` (= MUEARTH − 2 in the AGC constant table, i.e., MU_MOON). The Moon is
   treated as a point mass.

4. **Gravity switching** — The AGC switched the primary gravitational body at the
   sphere-of-influence boundary (RSPHERE = 64 373.76 km from Earth, defined in
   ORBITAL_INTEGRATION.agc). The Rust port exposes this via `PrimaryBody` (see
   `state_vector.rs`).

### CALCGRAV Algorithm (SERVICER207.agc, routine `CALCGRAV`)

The SERVICER calls CALCGRAV, not the full OBLATE routine. CALCGRAV computes:

```
1. UNITR = normalize(RN)          # unit position vector
2. rSQ   = |RN|^2  (stored in 34D)
3. rMag  = sqrt(rSQ)
4. J2_term = (RE/rMag)^2 * (dot(UNITR, UNITW)^2 - 1/20) * 20J * UNITR
           + (RE/rMag)^2 * dot(UNITR, UNITW) * 2J * UNITW
   where UNITW = Earth's rotation unit vector in reference (ECI) coords
5. GDT1/2 = −(MU_EARTH / rMag^3) * RN * (dt/2) + J2_term * (dt/2)
```

Source constants from SERVICER207.agc:
- `20J  2DEC* 3.24692010 E-2 B1*`  — corresponds to `20 × J2 = 20 × 1.082_626_68e-3 × ...`
  (the exact relationship to J2 includes the RE factor; see constants below)
- `2J   2DEC* 3.24692010 E-3 B1*`
- `RESQ 2DEC* 40.6809913 E12 B-59*` — RE^2 in AGC fixed-point

**For the Rust port**, the J2 contribution is computed using the standard
ECI formula derived from those AGC terms (see Public API below).

### OBLATE Algorithm (ORBITAL_INTEGRATION.agc, routine `OBLATE`)

OBLATE is the full-precision J2 perturbation used in the Nystrom integrator.
It also includes higher harmonics (J3, J4) via `J4REQ/J3` and `2J3RE/J2`
constants. For Milestone 2, only J2 is implemented; J3/J4 are out of scope
(flagged as `// TODO: add J3/J4 per ORBITAL_INTEGRATION.agc OBLATE routine`).

---

## Constants

All values from `docs/agc-reference-constants.md` (which cites
ORBITAL_INTEGRATION.agc). Use these exact values; do NOT substitute modern
IAU/WGS84 values.

```rust
/// Gravitational constants submodule.
/// All values sourced from docs/agc-reference-constants.md.
pub mod constants {
    /// Earth gravitational parameter (GM), m³/s².
    /// AGC: ORBITAL_INTEGRATION.agc `MUEARTH 2DEC* 3.986032 E10 B-36*`
    /// Value from docs/agc-reference-constants.md table.
    pub const MU_EARTH: f64 = 3.986_032e14;

    /// Moon gravitational parameter (GM), m³/s².
    /// AGC: ORBITAL_INTEGRATION.agc `MUM = MUEARTH -2` (second table entry)
    ///      `2DEC* 4.9027780 E8 B-30*` (scaled, same file)
    /// Value from docs/agc-reference-constants.md table.
    pub const MU_MOON: f64 = 4.902_778e12;

    /// Earth equatorial radius, metres.
    /// AGC: ORBITAL_INTEGRATION.agc `RSPHERE ~6373.338 km` / LATITUDE_LONGITUDE_SUBROUTINES.agc
    ///      `ERAD 2DEC 6373338 B-29 # PAD RADIUS`
    /// Value from docs/agc-reference-constants.md table.
    pub const RE_EARTH: f64 = 6_373_338.0;

    /// Earth J2 coefficient (dimensionless).
    /// AGC: ORBITAL_INTEGRATION.agc `J2REQSQ 2DEC* 1.75501139 E21 B-72*`
    ///      The J2 value is extracted from J2REQSQ = J2 * RE^2 / R^2 context.
    /// Value from docs/agc-reference-constants.md table.
    pub const J2_EARTH: f64 = 1.082_626_68e-3;

    /// Sphere of influence radius (Earth-Moon), metres.
    /// AGC: ORBITAL_INTEGRATION.agc `RSPHERE 2DEC 64373.76 E3 B-29`
    /// = 64 373 760 m = 64 373.76 km
    pub const RSPHERE: f64 = 64_373_760.0;

    /// Singularity guard: minimum |r| below which gravity returns zero, metres.
    /// Matches docs/agc-reference-constants.md point-mass guard.
    pub const R_MIN_GUARD: f64 = 1.0;
}
```

---

## Public API

Module path: `agc_core::navigation::gravity`

### `earth_gravity`

```rust
/// Gravitational acceleration from Earth: point mass + J2 oblateness.
///
/// Computes:
///   a_pm = −(MU_EARTH / |r|³) · r            (point-mass term)
///   a_J2 = J2 correction using J2_EARTH, RE_EARTH
///   return a_pm + a_J2
///
/// The J2 formula in ECI coordinates (derived from OBLATE / CALCGRAV):
///   Let u = r / |r|,  rMag = |r|,  z_hat = [0, 0, 1] (ECI pole)
///   k = (3/2) * J2_EARTH * MU_EARTH * RE_EARTH^2 / rMag^5
///   a_J2 = k * [ (5 * (u·z_hat)^2 − 1) * u  −  2 * (u·z_hat) * z_hat ]
///
/// Note: CALCGRAV uses UNITW (Earth rotation unit vector) instead of z_hat.
/// For ECI, UNITW ≈ z_hat = [0, 0, 1]. The Rust port uses the fixed z_hat.
/// If the AGC's precise UNITW is needed (non-zero x/y components), pass it
/// through a future `earth_gravity_unitw(r, unitw)` variant.
///
/// Invariant: never returns NaN or infinity for finite r with |r| > R_MIN_GUARD.
/// Singularity guard: if |r| <= R_MIN_GUARD (= 1.0 m), returns [0.0, 0.0, 0.0].
///
/// Units: r in metres → returns m/s²
///
/// AGC source: Comanche055/SERVICER207.agc, CALCGRAV routine (page 835);
///             Comanche055/ORBITAL_INTEGRATION.agc, OBLATE routine (page 1341).
pub fn earth_gravity(r: &Vec3) -> Vec3;
```

### `moon_gravity`

```rust
/// Gravitational acceleration from the Moon (point-mass only).
///
/// Computes:
///   r_moon_eci = moon_position_eci(t)    (stub for M2; see note below)
///   delta_r = r - r_moon_eci             (vehicle relative to Moon)
///   a = −(MU_MOON / |delta_r|³) · delta_r
///
/// For Milestone 2, moon_position_eci(t) is a STUB returning a fixed vector
/// at the mean lunar distance: [384_400_000.0, 0.0, 0.0] metres.
/// A real ephemeris call (LSPOS / LUNPOS routines in ORBITAL_INTEGRATION.agc)
/// is required for M3.
///
/// Singularity guard: if |delta_r| <= R_MIN_GUARD, returns [0.0, 0.0, 0.0].
///
/// Units: r in ECI metres → returns m/s²
///
/// AGC source: Comanche055/ORBITAL_INTEGRATION.agc, ACCOMP routine (line 130ff),
///             ITISMOON branch in CALCGRAV (SERVICER207.agc page 835).
pub fn moon_gravity(r: &Vec3, t: Met) -> Vec3;
```

### `total_gravity`

```rust
/// Total gravitational acceleration switching on the primary body.
///
/// When primary = PrimaryBody::Earth:
///   Returns earth_gravity(r) + moon_gravity(r, t)  [Earth + lunar perturbation]
///
/// When primary = PrimaryBody::Moon:
///   Returns moon_point_mass(r_from_moon) + earth_perturbation(r, t)
///   [Moon-centred, with Earth as third body -- out of scope for M2;
///    returns moon_gravity(r, t) only, with a TODO note]
///
/// The AGC switched primary body at the sphere-of-influence (RSPHERE) boundary
/// via the DOSWITCH / ORIGCHNG routines in ORBITAL_INTEGRATION.agc.
///
/// AGC source: Comanche055/ORBITAL_INTEGRATION.agc, CHKSWTCH / DOSWITCH / ORIGCHNG
///             (pages 1345-1346); MOONFLAG bit in ERASABLE_ASSIGNMENTS.agc.
pub fn total_gravity(r: &Vec3, t: Met, primary: PrimaryBody) -> Vec3;
```

### `sphere_of_influence_check`

```rust
/// Returns true if the vehicle's distance from Earth exceeds the sphere-of-influence
/// radius (RSPHERE = 64 373 760 m), signalling that Moon-centred integration
/// should be activated.
///
/// AGC: ORBITAL_INTEGRATION.agc, CHKSWTCH routine checks |RCV + RCONIC| vs RSPHERE.
pub fn sphere_of_influence_check(r_eci: &Vec3) -> bool;
```

---

## Invariants

1. `earth_gravity(r)` is never zero for `|r| > R_MIN_GUARD`. For any Earth-orbit
   position, the point-mass term dominates and the result is a finite, non-zero
   vector pointing toward Earth.
2. `earth_gravity(r)` returns `[0.0, 0.0, 0.0]` if and only if `|r| <= 1.0 m`.
3. All functions return finite `f64` components for all valid orbital inputs.
   NaN propagation from input is the caller's responsibility to prevent.
4. No heap allocation; all computations are in stack-allocated `f64` and `Vec3`.
5. No `unwrap` or `expect` calls.

---

## DSKY / agc-sim Impact

- The `agc-sim` physics engine (`agc-sim/src/physics.rs`) calls `total_gravity`
  each simulation tick to advance the reference trajectory.
- No new DSKY displays are required for the gravity module itself.
- The Mission State panel's APO/PER display depends indirectly on accurate gravity
  through the orbit propagation.

---

## Test Cases

### Test 1 — Earth surface gravity magnitude

```
r = [RE_EARTH, 0.0, 0.0]   // on the equator, |r| = 6 373 338 m
a = earth_gravity(&r)
|a| should be ≈ 9.80 m/s²  (AGC value; tolerance ± 0.02 m/s²)

// Derivation: MU_EARTH / RE_EARTH^2 = 3.986032e14 / (6_373_338)^2 ≈ 9.806 m/s²
// The J2 equatorial term slightly reduces the radial acceleration.
assert!(a[0] < 0.0)          // acceleration points toward Earth (negative x)
assert!((a[0].abs() - 9.80).abs() < 0.02)
assert!(a[1].abs() < 1e-10)  // no transverse component on equator
assert!(a[2].abs() < 1e-10)
```

### Test 2 — GEO altitude (point-mass dominated, J2 << PM)

```
r_geo = [42_164_000.0, 0.0, 0.0]   // GEO radius ≈ 42 164 km
a = earth_gravity(&r_geo)
// Point-mass: MU_EARTH / r_geo^2 = 3.986e14 / (4.2164e7)^2 ≈ 0.224 m/s²
// J2 at GEO is < 1e-6 m/s² — verify it is small but non-zero
a_pm_expected = 3.986_032e14 / (42_164_000.0_f64).powi(3) * 42_164_000.0
assert!((a[0].abs() - a_pm_expected).abs() / a_pm_expected < 1e-4)
```

### Test 3 — J2 polar vs equatorial bias

The J2 perturbation modifies the gravity vector differently at the pole vs equator.
At the pole (r along z-axis) J2 adds to the radial acceleration; at the equator
it reduces it.

```
r_equator = [RE_EARTH, 0.0, 0.0]
r_pole    = [0.0, 0.0, RE_EARTH]
a_eq  = earth_gravity(&r_equator)
a_pol = earth_gravity(&r_pole)
// At the equator: J2 reduces |a| (centrifugal-like effect from oblateness)
// At the pole:    J2 increases |a|
// The magnitude difference should be ~ 3 * J2 * MU/RE^2 * ... ≈ 0.052 m/s²
assert!(a_pol[2].abs() > a_eq[0].abs())   // polar gravity slightly stronger
delta = a_pol[2].abs() - a_eq[0].abs()
assert!(delta > 0.01 && delta < 0.1)      // order-of-magnitude check
```

### Test 4 — Sphere-of-influence crossover

```
// Just inside SOI: Earth gravity dominates
r_inside = [RSPHERE * 0.99, 0.0, 0.0]
assert!(!sphere_of_influence_check(&r_inside))

// Just outside SOI: Moon gravity switches
r_outside = [RSPHERE * 1.01, 0.0, 0.0]
assert!(sphere_of_influence_check(&r_outside))

// total_gravity with Earth primary at SOI boundary should not panic
a = total_gravity(&r_inside, Met(0), PrimaryBody::Earth)
assert!(a[0] < 0.0)   // Earth gravity still dominates
```

---

## Notes and Ambiguities

1. **CALCGRAV vs OBLATE**: SERVICER207.agc's `CALCGRAV` routine uses simplified
   J2 terms (`20J` and `2J` constants) rather than calling the full OBLATE routine.
   OBLATE is only called during the Nystrom full-orbital integration
   (ACCOMP → OBLATE). The Rust `earth_gravity` function implements the CALCGRAV
   J2 formula (using the standard ECI J2 expression), which is correct for the
   2-second SERVICER cycle. The integration module may call a more precise version
   if needed.

2. **UNITW**: CALCGRAV references `UNITW` (Earth rotation unit vector in reference
   coords). For a purely ECI frame aligned with the Earth's rotation axis, UNITW
   = [0, 0, 1]. The Rust port uses this fixed value. If UNITW has non-zero x/y
   components (epoch-dependent), this introduces a small error; flagged as an
   `APPROXIMATE` deviation for the validator.

3. **Moon ephemeris stub**: `moon_gravity` uses a fixed mean distance in M2.
   Real lunar ephemeris (LSPOS/LUNPOS) is required for M3 accuracy.

4. **Higher harmonics (J3, J4)**: The OBLATE routine in ORBITAL_INTEGRATION.agc
   includes J3 and J4 terms via `J4REQ/J3` and `2J3RE/J2` constants. These are
   explicitly out of scope for M2 and should be marked `// TODO` in the source.
