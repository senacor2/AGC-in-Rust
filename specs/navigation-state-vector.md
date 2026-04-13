# Functional Specification: Navigation State Vector (`agc-core/src/navigation/state_vector.rs`)

## AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Routines:   Erasable memory layout, pages 79-80 (MIT hardcopy pagination)
Key labels: RN, VN, PIPTIME, GDT/2, GOBL/2, RN1, VN1, PIPTIME1, GDT1/2, GOBL1/2,
            PBODY, REFSMMAT, UNITW, RTX2

AGC source: Comanche055/ORBITAL_INTEGRATION.agc
Routines:   KEPPREP, RECTIFY, ORIGCHNG, DIFEQ+0/+1/+2
Pages:      1334-1354

AGC source: Comanche055/SERVICER207.agc
Routines:   CALCRVG, CALCGRAV, NORMLIZE
Pages:      835-836
```

---

## Behavior Summary

The AGC navigation state is a position-velocity pair (RN, VN) in the Earth-Centred
Inertial (ECI) "reference coordinate system" (AGC terminology), plus a time tag
(PIPTIME) and the saved half-step gravity term (GDT/2) required by the
predictor-corrector integration scheme.

The Rust `StateVector` struct bundles position, velocity, and time into a single
owned, copy-able value. It carries no interior mutability and no heap allocation.
It is the central quantity passed through all navigation and integration APIs.

### Coordinate Frames

Three coordinate frames are used in Comanche055. The `StateVector` struct always
carries ECI coordinates. Frame transformations are applied at the HAL/IMU boundary
and are not embedded in the struct itself.

#### 1. Earth-Centred Inertial (ECI) — "Reference Coordinate System"

- Origin: Earth centre of mass.
- X axis: vernal equinox direction (approximately) at a reference epoch.
- Z axis: Earth rotation axis (north pole direction).
- Y axis: completes right-handed set.
- AGC name: "basic reference coordinate system" or simply "reference coords."
- All persistent state (RN, VN) is stored in this frame.
- AGC scale for position (RN): 2^(+29) metres per unit (B-28, i.e., one bit =
  2^(-28) × 2^29 m? — see Scale Factors section below).
  Actually per SERVICER207.agc header: `RN(6) REFERENCE COORD. SCALED AT 2(+29) M/CS`
  Wait: the comment is in M (metres), stated as `2(+29) M/CS` — but RN is a
  position, not a rate. The comment on CALCRVG line 799 clarifies:
  `STCALL RN1  # TEMP STORAGE OF RN SCALED 2(+29)M`.
  So position scale is 2^29 metres per full-scale. One AGC DP unit = 2^29 m /
  2^27 (DP range) = 4 m resolution. See Scale Factors section.
- AGC scale for velocity (VN): `2(+7) M/CS` per SERVICER207.agc header,
  i.e., 2^7 metres/centisecond per full scale = 128 m/cs = 12 800 m/s full scale.

#### 2. Stable-Member (SM) Frame — "Stable-Member Coordinate System"

- Fixed to the IMU gyroscopically-stabilised platform.
- REFSMMAT (Reference Stable-Member Matrix) rotates ECI → SM.
- DELV (PIPA delta-V accumulations) are measured in this frame.
- Transformation: SM = REFSMMAT · ECI_vector.
- REFSMMAT lives at `ERASABLE_ASSIGNMENTS.agc`: `REFSMMAT ERASE +17D # I(18D)PRM`.
- The Rust port exposes REFSMMAT as a `Mat3x3` in the IMU HAL; the
  `StateVector` struct itself does not carry the matrix.

#### 3. Body (Navigation Base / NB) Frame

- Fixed to the CSM structure.
- Relates to SM via the CDU gimbal angles (CDUX, CDUY, CDUZ, octal 32–34).
- Transformation: NB = R_z(CDUZ) · R_y(CDUY) · R_x(CDUX) · SM_vector
  (the AX*SR*T / CD*TR*GS routines in POWERED_FLIGHT_SUBROUTINES.agc).
- The `StateVector` struct does not carry body-frame data. Body-frame quantities
  (CDU angles, gyro rates) live in the HAL.

**Note on transformation matrices**: The REFSMMAT (SM ↔ ECI) and CDU-based
(NB ↔ SM) transforms are the responsibility of the HAL (`hal/imu.rs`) and the
control module (`control/imu_control.rs`). They are documented here for context
but are NOT part of the `StateVector` API.

---

## Scale Factors

These are the AGC fixed-point scales. The Rust port stores SI values (metres,
m/s) as `f64` directly; no scaling is needed in the core math. Conversions from
AGC words to SI happen only in test fixtures and HAL boundary code.

| Quantity | AGC scale | SI equivalent | Rust representation |
|---|---|---|---|
| Position (RN) | B-29 (2^29 m full-scale DP) | metres | `f64`, m |
| Velocity (VN) | B-7 m/cs (2^7 m/cs = 128 m/cs = 12800 m/s full-scale) | m/s | `f64`, m/s |
| Time (PIPTIME) | centiseconds | seconds | `Met` (u32 centiseconds) |
| GDT/2 | same as VN (B-7 m/cs) | m/s | `f64`, m/s (field of `StateVector`) |

The B-exponent convention: `2DEC* N B-k` means the double-word represents N ×
2^(−k) in full-scale units. For positions: `RN` is described as `SCALED AT 2(+29) M`
meaning one full-scale DP value = 2^29 metres ≈ 537 000 km. Earth–Moon distance
≈ 384 000 km < 2^29 m, so the field never overflows during a lunar mission.

Fixture conversion (for `agc-test/` only):
```
position_m = agc_dp_word * 2^29 / 2^28   (i.e., * 2.0)
velocity_ms = agc_dp_word * 2^7 / 2^7 * 100   (m/cs → m/s: × 100)
```

---

## Rust API

Module path: `agc_core::navigation::state_vector`

```rust
/// Navigation state vector in Earth-Centred Inertial (ECI) coordinates.
///
/// Bundles position (metres), velocity (m/s), and time (MET centiseconds).
/// Also carries `gdt_over_2`, the half-step gravity term saved by the
/// predictor-corrector integrator for use in the next cycle.
///
/// AGC equivalents:
///   RN        — ERASABLE_ASSIGNMENTS.agc, `RN ERASE +5`
///   VN        — ERASABLE_ASSIGNMENTS.agc, `VN ERASE +5`
///   PIPTIME   — ERASABLE_ASSIGNMENTS.agc, `PIPTIME ERASE +1`
///   GDT/2     — ERASABLE_ASSIGNMENTS.agc, `GDT/2 EQUALS PIPTIME +2`
///
/// Position scale: stored as SI metres (f64).
/// Velocity scale: stored as SI m/s (f64).
/// Time: stored as `Met` (centiseconds).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateVector {
    /// Position in ECI frame, metres.
    position: Vec3,
    /// Velocity in ECI frame, m/s.
    velocity: Vec3,
    /// Mission elapsed time of this state, in centiseconds.
    time: Met,
    /// Half-step gravity acceleration × dt, m/s.
    /// Equals `a_gravity(position) * dt / 2` from the previous SERVICER cycle.
    /// Zero on first cycle (NORMLIZE initialises GDT/2 via CALCGRAV).
    /// AGC: `GDT/2 EQUALS PIPTIME +2`
    gdt_over_2: Vec3,
}
```

### Constructor

```rust
impl StateVector {
    /// Construct a new state vector from position, velocity, and time.
    /// `gdt_over_2` is initialised to the zero vector; it is set by the
    /// SERVICER on the first call to CALCGRAV (NORMLIZE routine).
    ///
    /// AGC: NORMLIZE calls CALCGRAV to initialise GDT/2.
    pub fn new(position: Vec3, velocity: Vec3, time: Met) -> Self;

    /// Construct with an explicit `gdt_over_2` (used when restoring from
    /// erasable memory after a restart, or when the integrator needs to
    /// preserve the predictor term).
    pub fn with_gdt(position: Vec3, velocity: Vec3, time: Met, gdt_over_2: Vec3) -> Self;
}
```

### Accessors (all return by value or immutable reference; no `&mut self`)

```rust
impl StateVector {
    /// Position vector, ECI, metres.
    pub fn position(&self) -> Vec3;

    /// Velocity vector, ECI, m/s.
    pub fn velocity(&self) -> Vec3;

    /// Mission elapsed time of this state.
    pub fn time(&self) -> Met;

    /// Half-step gravity term saved from the previous integration cycle, m/s.
    /// Zero on a freshly constructed state.
    pub fn gdt_over_2(&self) -> Vec3;

    /// Return a new `StateVector` with position replaced; all other fields unchanged.
    pub fn with_position(self, r: Vec3) -> Self;

    /// Return a new `StateVector` with velocity replaced; all other fields unchanged.
    pub fn with_velocity(self, v: Vec3) -> Self;

    /// Return a new `StateVector` with time replaced; all other fields unchanged.
    pub fn with_time(self, t: Met) -> Self;

    /// Return a new `StateVector` with `gdt_over_2` replaced.
    pub fn with_gdt_over_2(self, gdt: Vec3) -> Self;

    /// Euclidean magnitude of the position vector, metres.
    /// Convenience wrapper around `math::linalg::norm`.
    pub fn radius(&self) -> f64;

    /// Euclidean magnitude of the velocity vector, m/s.
    pub fn speed(&self) -> f64;
}
```

### `PrimaryBody` Enum

The AGC used a flag called `MOONFLAG` (ERASABLE_ASSIGNMENTS.agc: `MOONFLAG = 003D`)
and the `PBODY` erasable register to switch between Earth-primary and Moon-primary
integration. In the Rust port this is a plain enum passed into gravity and
integration functions.

```rust
/// The gravitational primary body for integration.
///
/// AGC: MOONFLAG bit in flag word 0 (ERASABLE_ASSIGNMENTS.agc `MOONFLAG = 003D`).
/// AGC: PBODY erasable register (ERASABLE_ASSIGNMENTS.agc `PBODY ERASE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimaryBody {
    /// Earth-centred integration (MOONFLAG clear).
    Earth,
    /// Moon-centred integration (MOONFLAG set).
    Moon,
}
```

---

## Invariants

1. `StateVector` is `Copy`. No heap allocation is ever involved.
2. `position()` and `velocity()` return copies of the internal `Vec3` fields;
   they never return references into the struct, so there is no aliasing concern.
3. `with_position`, `with_velocity`, `with_time`, `with_gdt_over_2` all consume
   `self` and return a new `StateVector`. They do not mutate in place.
4. `gdt_over_2` is the zero vector immediately after `StateVector::new(...)`.
   The SERVICER must initialise it by calling `CALCGRAV` (Rust: `earth_gravity`)
   before the first integration step.
5. No `unwrap`, `expect`, or panics are permitted in this module.
6. `radius()` is never called when `|position|` could be zero in flight code.
   A singularity guard in the gravity module handles the zero-radius case.

---

## DSKY / agc-sim Impact

- The Mission State panel in `agc-sim` reads `position()` and `velocity()` for
  display of ECI radius (km), speed (m/s), and derived orbital elements.
- No new DSKY lights are needed.
- `MissionState` struct in `agc-sim` should store the latest `StateVector` and
  recompute SMA/ECC/APO/PER each cycle from it using `math::kepler`.

---

## Test Cases

### Test 1 — Round-trip accessor consistency

```
state = StateVector::new(
    [6_578_000.0, 0.0, 0.0],   // 200 km circular LEO, m
    [0.0, 7_784.0, 0.0],        // circular orbital speed, m/s
    Met::from_centiseconds(0),
)
assert_eq!(state.position(), [6_578_000.0, 0.0, 0.0])
assert_eq!(state.velocity(), [0.0, 7_784.0, 0.0])
assert_eq!(state.time(), Met(0))
assert_eq!(state.gdt_over_2(), [0.0, 0.0, 0.0])
assert!(state.radius() ≈ 6_578_000.0, tolerance 1e-6 m)
assert!(state.speed()  ≈ 7_784.0,     tolerance 1e-6 m/s)
```

### Test 2 — Immutable builder pattern

```
s0 = StateVector::new([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], Met(100))
s1 = s0.with_position([2.0, 0.0, 0.0])
assert_eq!(s0.position(), [1.0, 0.0, 0.0])   // s0 unchanged (Copy)
assert_eq!(s1.position(), [2.0, 0.0, 0.0])
assert_eq!(s1.velocity(), [0.0, 1.0, 0.0])   // velocity unchanged
assert_eq!(s1.time(),     Met(100))           // time unchanged
```

### Test 3 — AGC scale-factor cross-check (fixture test)

The AGC DP word for RN at 6 500 000 m altitude (approx) should yield
`6_500_000.0 / 2.0` in the B-29 scale (1 AGC unit = 2 m at B-29 DP).
Conversely, a test in `agc-test/` that converts a raw AGC memory dump:

```
// Raw AGC DP: hi=0x1860, lo=0x0000  →  combined = 0x0C300000
// Scaled at B-29: 0x0C300000 * 2 = 6 442 450 944 m  [bogus; this is a unit example]
// Fixture-based: compare StateVector.position() components against
//   from_agc_dword(hi, lo, 29) for each axis.
// Tolerance: < 1 m difference.
assert!(|rust_position_x - agc_position_x| < 1.0)  // per testing.md §5
```

---

## Notes and Ambiguities

1. The AGC's B-notation is slightly non-standard in the source comments. The
   comment `SCALED AT 2(+29) M` means the DP pair's full-scale represents
   2^29 metres. With a 27-bit signed DP mantissa this gives 1 LSB ≈ 4 metres.
   The Rust port uses `f64` which has 53 mantissa bits, so no precision is lost.

2. `GDT/2` in the AGC follows PIPTIME in memory (`GDT/2 EQUALS PIPTIME +2`) and
   is bulk-copied with RN/VN via `GENTRAN` in SERVICER (line 498-501). The Rust
   `StateVector` bundles all four fields to preserve this coupling.

3. `GOBL/2` (oblateness half-step gravity, `GOBL/2 EQUALS GDT/2 +6`) is a
   separate AGC field that accumulates the J2 contribution. For Milestone 2, the
   J2 gravity is folded into `gdt_over_2` rather than stored separately. If the
   validator finds a discrepancy, a separate `gobl_over_2: Vec3` field should be
   added; this is flagged as a known deviation.
