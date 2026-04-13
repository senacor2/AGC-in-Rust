# Functional Specification: Physical-Quantity Newtypes (`agc-core/src/types/`)

AGC source reference: `Comanche055/ERASABLE_ASSIGNMENTS.agc` (special registers, CDU/PIPA assignments, pages 39-41);
`docs/architecture.md` §3.2-3.3; `docs/agc-reference-constants.md` (scale-factor table).

---

## 1. Module Layout

```
agc-core/src/types/
    mod.rs      — re-exports all public types
    angle.rs    — CduAngle
    vector.rs   — Vec3, DeltaV
    matrix.rs   — Mat3x3
    time.rs     — Met
```

All types must compile with `#![no_std]` and must not use heap allocation.
Every public item must carry a doc comment that states unit and scale factor.

---

## 2. `CduAngle`

### AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Registers:  CDUX (octal 32), CDUY (octal 33), CDUZ (octal 34)
            CDUT (octal 35, optics trunnion), CDUS (octal 36, optics shaft)
            CDUXCMD (octal 50), CDUYCMD (octal 51), CDUZCMD (octal 52)
            CDUTCMD (octal 53), CDUSCMD (octal 54)
Module:     agc-core/src/types/angle.rs
```

### Behavior Summary

A CDU (Coupling Data Unit) angle represents one gimbal axis of the IMU or optics
assembly. The hardware counts full revolutions in 15-bit unsigned ones-complement
on the real AGC; in the Rust port the raw value is stored as a plain `u16` using
two's complement (the full 16-bit range gives 65536 counts per revolution, but
the AGC only used 15 bits so the range is 0..32767 for positive angles and
32768..65535 maps to the negative half, with 32768 = -0 in ones-complement; the
Rust representation simply treats the raw hardware word as twos-complement u16
wrapping modulo 65536).

Scale factor (from architecture.md §3.2 and ERASABLE_ASSIGNMENTS.agc): one
full revolution = 2^15 = 32768 counts in the original AGC hardware register.
The Rust port stores the raw u16 as supplied by the hardware interface.
Conversion to radians: `radians = (raw as f64) * (2π / 65536.0)`.

The coarse-align tolerance from IMU_MODE_SWITCHING_ROUTINES.agc (COARSTOL label,
page 1425) is -0.01111 in half-revolution units, which corresponds to 2 degrees
(approximately 364 counts at 2^15 counts per half-revolution).

### Rust API

```rust
/// CDU (Coupling Data Unit) gimbal angle.
///
/// Raw hardware value: u16, range 0..=65535.
/// Scale: 65536 counts = one full revolution (2π radians).
/// The AGC hardware uses 15-bit ones-complement internally; this newtype
/// stores the sign-extended, two's-complement equivalent (wrapping u16).
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
/// Registers: CDUX/Y/Z (octal 32-34), CDUT/S (octal 35-36).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
#[repr(transparent)]
pub struct CduAngle(pub u16);

impl CduAngle {
    /// Construct from a raw hardware count word.
    pub const fn from_counts(raw: u16) -> Self;

    /// Construct from radians, rounding to nearest count.
    /// Input is reduced modulo 2π before conversion.
    pub fn from_radians(rad: f64) -> Self;

    /// Convert to radians in the range [0, 2π).
    pub fn to_radians(self) -> f64;

    /// Return the raw hardware count.
    pub const fn counts(self) -> u16;

    /// Subtract two CDU angles, returning a signed difference in the range
    /// (-32768, 32767) counts (wrapping arithmetic).
    pub fn wrapping_diff(self, rhs: Self) -> i16;
}
```

Ops to implement:
- `Add<CduAngle>` — wrapping addition (for combining offsets), returns `CduAngle`
- `Sub<CduAngle>` — wrapping subtraction, returns `CduAngle`
- `Neg` — wrapping negation, returns `CduAngle`
- `core::fmt::Display` — format as degrees to one decimal place

No arithmetic semantics beyond wrapping add/sub; do not implement `Mul` or `Div`.

### Scale Factors

| Direction | Formula |
|---|---|
| counts → radians | `(counts as f64) * (core::f64::consts::TAU / 65536.0)` |
| radians → counts | `(rad / TAU * 65536.0).round() as u16` (wrapping on overflow) |
| counts → degrees | `(counts as f64) * (360.0 / 65536.0)` |

### Invariants

- The inner `u16` wraps modulo 65536 on all arithmetic. This is correct behavior
  because the CDU hardware wraps.
- `from_radians(to_radians(x))` round-trips with error at most 1 count
  (≤ 0.006 degrees).
- `CduAngle` is `Copy` and has no heap allocation.
- The zero value (`CduAngle(0)`) represents a gimbal angle of exactly 0 radians.

### Test Cases

1. Zero conversion: `CduAngle::from_counts(0).to_radians()` == 0.0
2. Half revolution: `CduAngle::from_counts(32768).to_radians()` ≈ π (within 1 ULP)
3. Wraparound: `CduAngle(0xFFFF).wrapping_diff(CduAngle(1))` == -2 (wrapping)
   - 0xFFFF = 65535 counts, 1 count; difference = 65534 which as i16 wraps to -2
   - Alternatively stated: diff = (65535u16.wrapping_sub(1)) as i16 = 65534 as i16 = -2
4. Coarse-align tolerance: `CduAngle::from_radians(2.0_f64.to_radians()).counts()`
   should equal approximately 364 (2° × 32768/180° ≈ 364 counts).

---

## 3. `Vec3`

### AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Usage:      Position vectors (RN, RN1), velocity vectors (VN, VN1),
            delta-V (DELVEET), gyro torque, PIPA reading.
Module:     agc-core/src/types/vector.rs
```

### Behavior Summary

`Vec3` is a 3-component, double-precision floating-point vector. It is used
for all navigational math: position vectors (m), velocity vectors (m/s),
delta-V vectors (m/s), and attitude rate vectors (rad/s). No scale factor
is embedded; the caller is responsible for tracking the physical dimension.

The AGC stored these in double-word (DP) interpretive format. The Rust port
uses `f64` directly, which is equivalent in precision to the AGC's DP
format (both are approximately 15 significant decimal digits).

### Rust API

```rust
/// 3-component double-precision vector.
///
/// Units and coordinate frame are context-dependent:
///   - Position: metres (ECI or body frame, as documented at call sites)
///   - Velocity: m/s
///   - Delta-V: m/s
///   - Attitude rate: rad/s
///
/// This is a type alias, not a newtype. Arithmetic ops are plain f64 ops
/// applied component-wise via the functions in `math::linalg`.
pub type Vec3 = [f64; 3];
```

Constructor helpers (free functions in `types/vector.rs`, not methods on Vec3):

```rust
/// Construct a Vec3 from three components.
pub const fn vec3(x: f64, y: f64, z: f64) -> Vec3;

/// The zero vector.
pub const ZERO_VEC3: Vec3 = [0.0, 0.0, 0.0];
```

All arithmetic (dot, cross, norm, scale, add, sub) lives in `math::linalg`.
Do not add arithmetic methods here; `Vec3` is deliberately kept as a plain array.

### Scale Factors

| Context | SI unit | AGC original scale |
|---|---|---|
| Position | metres | B-28 (1 AGC unit = 2^28 m; not used in Rust) |
| Velocity | m/s | B-7 (1 AGC unit = 2^7 m/s; not used in Rust) |
| Delta-V | m/s | same as velocity |
| Angle rates | rad/s | dimensionless |

The Rust port stores SI values directly. No scale-factor conversion is needed
at call sites; conversions from AGC-scale values to SI occur at HAL boundaries
(PIPA scale factor: 0.0585 m/s per count, from `docs/agc-reference-constants.md`).

### Invariants

- Individual components may be any finite `f64` value, including subnormals.
- NaN components must never arise from navigation code; callers are responsible
  for guarding against division by zero (singularity guard: if |r| < 1.0 m
  return zero, per `docs/agc-reference-constants.md`).
- `Vec3` is `Copy`, zero-sized overhead, no heap allocation.

### Test Cases

1. Zero vector: `vec3(0.0, 0.0, 0.0)` should equal `ZERO_VEC3`.
2. Equality: `vec3(1.0, 2.0, 3.0)[1]` == 2.0
3. Default round-trip through HAL: a PIPA reading of `[100i16, 0, 0]` converts
   to `vec3(100.0 * 0.0585, 0.0, 0.0)` = `vec3(5.85, 0.0, 0.0)` m/s.

---

## 4. `Mat3x3`

### AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Usage:      REFSMMAT (reference-to-stable-member matrix), coordinate transforms.
Module:     agc-core/src/types/matrix.rs
```

### Behavior Summary

`Mat3x3` is a 3×3 double-precision matrix used for coordinate frame transforms.
The primary instance is REFSMMAT (the Reference Stable Member Matrix), which
rotates ECI vectors into the IMU stable-member frame and vice versa. It is also
used for the W-matrix (state noise covariance in navigation filter).

Row-major storage: `mat[row][col]`.

### Rust API

```rust
/// 3×3 double-precision matrix.
///
/// Stored in row-major order: mat[row][col].
/// Used for rotation matrices (REFSMMAT), coordinate-frame transforms,
/// and the navigation W-matrix (covariance).
///
/// All matrix arithmetic is in `math::linalg`.
pub type Mat3x3 = [[f64; 3]; 3];
```

Constructor helpers (free functions in `types/matrix.rs`):

```rust
/// 3×3 identity matrix.
pub const IDENTITY_MAT3: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// 3×3 zero matrix.
pub const ZERO_MAT3: Mat3x3 = [[0.0; 3]; 3];

/// Construct from rows.
pub const fn mat3(row0: [f64; 3], row1: [f64; 3], row2: [f64; 3]) -> Mat3x3;
```

Matrix-vector multiply, transpose, and matrix-matrix multiply live in
`math::linalg`. `Mat3x3` itself is a plain array alias.

### Scale Factors

`Mat3x3` is dimensionless when used as a rotation matrix. When used as a
covariance (W matrix), individual components have units of (position_unit)^2 or
(velocity_unit)^2, documented at the call site.

### Invariants

- A rotation matrix must satisfy R^T R = I to within numerical precision
  (approximately 1e-12 Frobenius norm error after float arithmetic).
- There is no enforcement of the rotation invariant at the type level; the
  programmer ensures it through construction.
- `Mat3x3` is `Copy`.

### Test Cases

1. Identity transform: `mat_vec_mul(IDENTITY_MAT3, vec3(1.0, 2.0, 3.0))` == `[1.0, 2.0, 3.0]`
2. Transpose involution: for any rotation matrix R, `transpose(transpose(R))` == R
3. Orthogonality: if R is produced by `mat_from_refsmmat(...)`, then
   `mat_mat_mul(R, transpose(R))` should be within 1e-10 of `IDENTITY_MAT3`
   component-wise.

---

## 5. `Met` (Mission Elapsed Time)

### AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc
Registers:  TIME1 (octal 25), TIME2 (octal 24) — double-precision counter
            TIME3 (octal 26) — Waitlist interrupt timer
            TIME4 (octal 27) — T4RUPT periodic I/O timer
            TIME5 (octal 30) — T5RUPT DAP timer
            TIME6 (octal 31) — T6RUPT jet timing
Module:     agc-core/src/types/time.rs
```

### Behavior Summary

The AGC maintained mission elapsed time as a double-precision 15-bit counter
pair: TIME1 (low word) and TIME2 (high word). TIME1 was incremented every
centisecond (0.01 s) by the T3RUPT hardware clock. When TIME1 overflowed (after
32767 centiseconds = approximately 327.67 seconds), it carried into TIME2.
Together TIME1/TIME2 form a 28-bit counter (15 bits each, ones-complement) that
wraps after approximately 2^28 centiseconds ≈ 31.1 days.

The Rust port represents MET as a `u32` counting centiseconds. This gives:
- Resolution: 10 ms (one centisecond)
- Maximum: 2^32 centiseconds ≈ 497.1 days (well beyond any Apollo mission)
- Wrap behavior: wraps silently at 2^32 centiseconds

The choice of `u32` over `u64` is deliberate: the AGC's 28-bit counter fit in
two 15-bit words; `u32` is the smallest Rust integer type that covers the
full mission duration (Earth-Moon-Earth, approximately 8-12 days) with margin.
`u64` would be larger than necessary for a `no_std` embedded target where
register width matters.

Converting to seconds for navigation math is a one-liner (`cs as f64 / 100.0`)
and is done only at call sites that require `f64` seconds, not stored.

### Rust API

```rust
/// Mission Elapsed Time, stored in centiseconds (0.01 s per tick).
///
/// Corresponds to the AGC TIME1/TIME2 double-precision counter.
/// TIME1 = ERASABLE_ASSIGNMENTS.agc octal 25 (low word).
/// TIME2 = ERASABLE_ASSIGNMENTS.agc octal 24 (high word).
///
/// Scale: 1 unit = 0.01 seconds (1 centisecond).
/// Range: 0..=u32::MAX centiseconds (≈ 497 days; wraps silently).
///
/// Convert to f64 seconds at call sites: `met.as_secs_f64()`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
#[repr(transparent)]
pub struct Met(pub u32);

impl Met {
    /// Construct from a centisecond count (raw AGC TIME1/TIME2 pair value).
    pub const fn from_centiseconds(cs: u32) -> Self;

    /// Construct from seconds, truncating to nearest centisecond.
    pub fn from_secs(secs: f64) -> Self;

    /// Return the count in centiseconds.
    pub const fn as_centiseconds(self) -> u32;

    /// Convert to f64 seconds for use in navigation math.
    /// Only call at math sites; do not store the f64 value in state.
    pub fn as_secs_f64(self) -> f64;

    /// Wrapping addition of centiseconds (matches AGC TIME overflow behavior).
    pub fn wrapping_add_cs(self, delta_cs: u32) -> Self;

    /// Wrapping subtraction of centiseconds.
    pub fn wrapping_sub_cs(self, other: Self) -> u32;
}
```

Ops to implement:
- `Add<u32>` — add centiseconds, returns `Met` (wrapping)
- `Sub<Met>` — returns elapsed centiseconds as `u32` (wrapping)
- `core::fmt::Display` — format as `HH:MM:SS.cc`

### Scale Factors

| Unit | Relation to internal |
|---|---|
| 1 centisecond (AGC count) | 1 `Met` unit |
| 1 second | 100 `Met` units |
| 1 minute | 6000 `Met` units |
| 1 hour | 360000 `Met` units |

AGC scale factor for TIME registers (from docs/architecture.md, scaling table):
"Time: centiseconds, B-14 (1 unit = 2^-14 of some denominator)." In the Rust
port there is no fractional scaling; 1 unit = 1 centisecond exactly.

### Invariants

- `Met` wraps at `u32::MAX + 1` centiseconds. This is intentional; the AGC
  similarly wrapped (though after 31.1 days, not 497 days). Mission logic must
  not depend on the absence of wrap; inter-event timing uses delta subtraction.
- `from_secs(x).as_secs_f64()` round-trips with error at most 0.01 s.
- `Met` is `Copy` and zero heap.

### Test Cases

1. Round-trip: `Met::from_centiseconds(100).as_secs_f64()` == 1.0 exactly.
2. Display: `Met::from_centiseconds(360000 + 6000 + 100 + 5)` should display as
   `01:01:01.05` (1 hour, 1 minute, 1.05 seconds).
3. Wrap: `Met::from_centiseconds(u32::MAX).wrapping_add_cs(1)` == `Met(0)`.
4. Delta: `Met(200).wrapping_sub_cs(Met(100))` == 100 (centiseconds).

---

## 6. `DeltaV`

### AGC Source Reference

```
AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc, P30-P37.agc, P40-P47.agc
Registers:  DELVEET (delta-V target vector), DELVET (accumulated delta-V)
Usage:      SPS burn target delta-V; RCS maneuver delta-V; PIPA accumulation
Module:     agc-core/src/types/vector.rs
```

### Behavior Summary

`DeltaV` is a newtype wrapping `Vec3` (i.e., `[f64; 3]`) in metres per second.
Its purpose is compile-time unit safety: functions that expect a velocity change
vector accept `DeltaV`, not a bare `Vec3`, preventing accidental substitution
of a position or rate vector.

The distinction from `Vec3` is semantic only; no additional operations beyond
what `Vec3` supports are needed. The `DeltaV.0` field provides direct access
to the underlying `Vec3`.

SPS burn delta-V from docs/agc-reference-constants.md: a typical burn commands
50 m/s. PIPA scale factor 0.0585 m/s/count is used when accumulating PIPAs into
a `DeltaV`.

### Rust API

```rust
/// A velocity change vector, in metres per second (ECI or body frame).
///
/// Wraps Vec3 ([f64; 3]). Units: m/s.
/// Used for SPS delta-V targets (DELVEET), accumulated PIPA delta-V,
/// and RCS burn commands.
///
/// AGC source: Comanche055/ERASABLE_ASSIGNMENTS.agc (DELVEET registers).
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct DeltaV(pub Vec3);

impl DeltaV {
    /// Construct from three m/s components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self;

    /// Construct from a Vec3 already in m/s.
    pub const fn from_vec3(v: Vec3) -> Self;

    /// Return the underlying Vec3.
    pub const fn as_vec3(self) -> Vec3;

    /// Magnitude in m/s.
    pub fn magnitude(self) -> f64;

    /// Scale by a dimensionless factor (e.g., throttle fraction).
    pub fn scale(self, factor: f64) -> Self;

    /// Add two delta-V vectors (superposition).
    pub fn add(self, other: Self) -> Self;
}
```

Ops to implement:
- `Add<DeltaV>` — component-wise addition, returns `DeltaV`
- `Sub<DeltaV>` — component-wise subtraction, returns `DeltaV`
- `Mul<f64>` — scale, returns `DeltaV`
- `Neg` — negate all components, returns `DeltaV`
- `core::fmt::Display` — format as `[x, y, z] m/s`

### Scale Factors

Raw PIPA count to DeltaV: multiply each axis count by `PIPA_SCALE = 0.0585 m/s`
(from `docs/agc-reference-constants.md`, SERVICER207.agc: `KPIP1 5.85 CM/SEC`).

### Invariants

- Components may be any finite `f64`. `DeltaV(ZERO_VEC3)` represents no change
  in velocity.
- `DeltaV` is `Copy`.
- Callers must not store NaN or infinity; no runtime check is performed.

### Test Cases

1. Magnitude: `DeltaV::new(3.0, 4.0, 0.0).magnitude()` == 5.0 exactly.
2. Scale: `DeltaV::new(1.0, 0.0, 0.0).scale(50.0)` == `DeltaV::new(50.0, 0.0, 0.0)`.
3. PIPA conversion: `DeltaV::from_vec3([100_f64 * 0.0585, 0.0, 0.0])` has
   `.magnitude()` ≈ 5.85 m/s.

---

## 7. Module Public Re-export (`types/mod.rs`)

```rust
pub use angle::CduAngle;
pub use vector::{vec3, DeltaV, Vec3, ZERO_VEC3};
pub use matrix::{mat3, Mat3x3, IDENTITY_MAT3, ZERO_MAT3};
pub use time::Met;
```

No other items should be public from the `types` module.

---

## agc-sim Impact

- `SimHardware` stores CDU angles as `[CduAngle; 3]` for IMU and `[CduAngle; 2]`
  for optics (shaft, trunnion); the TUI must display these as degrees.
- MET display in the Mission State panel uses `Met::as_secs_f64()` and formats
  as `HH:MM:SS`.
- `DeltaV` is used in the Mission State panel for VG (velocity to go) display.
- No new DSKY light bindings needed for these types.
