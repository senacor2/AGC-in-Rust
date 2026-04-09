# Specification: `types/` Module

**Status**: Approved for implementation  
**Module path**: `agc-core/src/types/`  
**Source files**: `mod.rs`, `angle.rs`, `vector.rs`, `matrix.rs`  
**Architecture reference**: `docs/architecture.md` §3 "Type System Design"  
**Spec checklist**: `specs/README.md` — all items satisfied (see §10)

---

## 1. Purpose and Scope

The `types/` module defines the fundamental physical-quantity types used across
every subsystem of the AGC-in-Rust flight software. It is the lowest layer of
the crate dependency graph; every other module depends on it and nothing in this
module depends on any sibling module.

The five public types are:

| Rust type  | Physical quantity                         | Primitive representation |
|------------|-------------------------------------------|--------------------------|
| `CduAngle` | IMU/optics CDU gimbal angle               | `u16` (twos-complement counts) |
| `Met`      | Mission elapsed time                      | `u32` (centiseconds)     |
| `DeltaV`   | Maneuver delta-velocity vector            | `Vec3` (m/s)             |
| `Vec3`     | Position, velocity, acceleration, delta-V | `[f64; 3]` (SI units)    |
| `Mat3x3`   | Rotation / coordinate-frame transform     | `[[f64; 3]; 3]`          |

### What this module is NOT

- There is **no `AgcWord` type**. The original AGC's 15-bit ones-complement word
  format was a hardware constraint, not an algorithm requirement. All navigation
  and guidance math uses `f64`.
- There is **no fixed-point arithmetic**. Scale factors from the AGC source are
  converted once at the hardware boundary (`hal/` layer) and never appear inside
  flight-software calculations.
- `Vec3` and `Mat3x3` are **type aliases**, not newtypes. Mathematical helper
  functions (dot, cross, norm, matrix-vector multiply) live in `math::linalg`,
  not as methods here.

---

## 2. AGC Background: Original Data Representations

Understanding the AGC source requires knowing how the original hardware encoded
the physical quantities that these Rust types replace.

### 2.1 AGC Word Format

The Block 2 AGC uses 15-bit ones-complement fractional arithmetic (plus a 16th
parity bit in memory). A single-precision word holds a value in the range
(-1, +1) and must be scaled by a power-of-two **scale factor** (written B+n or
B-n in the original documentation) to recover the physical value.

> Reference: `docs/AGC Symbolic Listing.md`, §Notation item 10:
> "The scale factor of a quantity is the power of two by which the number in the
> computer (considered as a fraction in the range between -1 and +1) must be
> multiplied to obtain its true value."

Double-precision quantities occupy two consecutive erasable words (most
significant word first).

### 2.2 CDU Angle Encoding

CDU (Coupling Data Unit) angles are an exception to the ones-complement rule:
the original hardware delivers them in **twos-complement** form (see
`docs/AGC Symbolic Listing.md` §IIA: "Angle information is in twos complement
form").

- **Original AGC hardware** (15-bit register): Scale factor **B-1 revolutions**
  — 1 full revolution = 2^15 = 32768 counts in a signed 15-bit word. The most
  significant bit is the sign bit; negative angles are stored as counts in the
  range [32768, 65535] in twos-complement.
- **Rust port** (`u16`, 16-bit unsigned): A full revolution maps to
  **2^16 = 65536 counts**, giving uniform angular resolution across the full
  circle. Count 0 = 0°, count 32768 = 180° (= -180° in twos-complement
  convention), count 65535 = one step below 360°. The `u16` wraps at exactly
  one revolution, so wrapping arithmetic on counts is always correct.

  > Comanche055 CDU erasable cells: `CDUX` (octal 0033), `CDUY` (octal 0034),
  > `CDUZ` (octal 0035) — IMU gimbal angles (outer, inner, middle).
  > `OPTY` (octal 0036), `OPTX` (octal 0037) — optics shaft and trunnion.
  > Source: Comanche055/`ERASABLE_ASSIGNMENTS.agc`, lines ~110–140.

### 2.3 Mission Elapsed Time Encoding

In the AGC, time is maintained in the counter cells `TIME1`/`TIME2`
(a double-precision pair, octal 0024/0025). `TIME1` increments every 10 ms
(centiseconds × 10); the full double-precision pair overflows after ~31 days.
`TIME6` (octal 0017) is a fine-resolution down-counter used for RCS timing.

The Rust port uses a **`u32` centisecond counter** (`Met`) which wraps after
approximately 497 days — well beyond any mission profile.

  > Comanche055 source: `ERASABLE_ASSIGNMENTS.agc` — `TIME1` at octal 0024,
  > `TIME2` at octal 0025.

### 2.4 Velocity / Position Encoding

State vectors in the AGC use double-precision (two-word) scaled fixed-point:

- **Positions**: scale B+28 m — 1 LSB ≈ 2^28 m (≈ 268 million metres; the
  position fits in (-1, +1) double-precision, so maximum representable range
  is ±2^28 m ≈ ±268 Mm, spanning Earth-Moon distance).
- **Velocities**: scale B+7 m/s — 1 LSB ≈ 2^7 = 128 m/s; maximum ≈ ±128 m/s
  per double-precision fraction.
- **Delta-V**: same scale as velocity, B+7 m/s.

  > Comanche055 source: `ERASABLE_ASSIGNMENTS.agc` — position state vector
  > `RN` at octal 0306 (6 words, x/y/z double-precision);
  > velocity state vector `VN` at octal 0314 (6 words).

In the Rust port, `Vec3` holds plain `f64` SI values. Conversion from the
AGC double-precision fixed-point pair `(w_hi, w_lo)` to `f64` SI follows:

```
f64_value = (w_hi * 2^-14  +  w_lo * 2^-28)  *  scale_factor
```

where `scale_factor` is 2^28 for positions and 2^7 for velocities/delta-V,
as documented in `docs/architecture.md` §3.1 and `specs/README.md` §AGC Quick
Reference.

### 2.5 Rotation Matrix Encoding

REFSMMAT (Reference-to-Stable-Member Matrix) is a 3×3 matrix stored as 9
double-precision words in erasable memory beginning at octal 0306 bank E3
(`REFSMMAT`, Comanche055/`ERASABLE_ASSIGNMENTS.agc`). Each element is a unit
fraction (scale B+0 — the matrix is orthonormal, all elements in [-1, +1]).

In the Rust port, `Mat3x3 = [[f64; 3]; 3]` in row-major order holds the same
matrix with native `f64` precision. No scale conversion is needed (B+0 maps
directly).

---

## 3. Type Specifications

### 3.1 `CduAngle`

#### Declaration

```rust
// agc-core/src/types/angle.rs
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct CduAngle(pub u16);
```

#### Semantics

`CduAngle` wraps a raw hardware CDU count delivered by the IMU or optics
subsystem via I/O channels. The inner `u16` is the twos-complement angle value
exactly as read from the hardware register (I/O channel 30–33 octal range for
CDU read-back in Comanche055).

In the Rust `u16` representation, a full revolution is encoded as 2^16 = 65536
counts (see §2.2 above and `docs/architecture.md` §3.2). Count 32768 (0x8000)
represents exactly 180° / π radians, which is also -180° in the twos-complement
angular convention. Counts in the range [32768, 65535] represent the negative
half of the angular range (-π, 0) in the hardware convention but are stored as
unsigned values in `u16`.

#### Valid range and invariants

- **Storage invariant**: All `u16` values are structurally valid. There is no
  illegal bit pattern.
- **Semantic invariant**: Angles are only meaningful modulo 2^16 counts (one
  full revolution). Callers must not interpret a count of 32768–65535 as "more
  than one revolution"; it represents a negative angle in the range (-π, 0) in
  the twos-complement convention.
- **Wrapping**: Arithmetic on counts must use wrapping addition/subtraction
  (`u16::wrapping_add`, `u16::wrapping_sub`) to preserve the angular modular
  arithmetic. No saturation or panic on overflow is correct.

#### Scale factor conversion

| AGC representation | Scale factor | Rust `f64` unit |
|--------------------|--------------|-----------------|
| 1 CDU count (1 LSB) | B-1 revolutions | TAU / 65536 radians ≈ 9.587e-5 rad |
| 32768 counts | B-1 revolutions = 0.5 rev | π radians |
| 65536 counts (wraps to 0) | 1.0 rev | 2π radians = TAU |

Conversion formula:

```
radians = counts * (TAU / 65536)
        = counts * (2π / 2^16)
```

#### Methods

```rust
impl CduAngle {
    /// Convert raw CDU count to radians (f64).
    /// Precondition: none (all u16 values are valid).
    /// Postcondition: result is in [0, TAU) when count is in [0, 32767],
    ///                result is in [TAU/2, TAU) when count is in [32768, 65535]
    ///                (negative angles arrive as large positive counts).
    pub fn to_radians(self) -> f64 {
        (self.0 as f64) * (core::f64::consts::TAU / 65536.0)
    }

    /// Convert raw CDU count to degrees (f64).
    /// Postcondition: result is in [0.0, 360.0).
    pub fn to_degrees(self) -> f64 {
        self.to_radians() * (180.0 / core::f64::consts::PI)
    }
}
```

No `from_radians` constructor is provided in the core type. Construction from
a physical angle belongs in the HAL simulation layer (`agc-sim/`), which must
perform the inverse scaling:

```
count = (radians * 65536.0 / TAU).round() as u16
```

#### Debug format

`CduAngle` formats as `CduAngle(ddd.dddd°)` — the degree value, not the raw
count — to aid human readability of diagnostic output.

#### AGC erasable variable mapping

| Rust usage           | AGC cell    | Octal address | Description                    |
|----------------------|-------------|---------------|--------------------------------|
| `imu.read_cdu()[0]`  | `CDUX`      | 0033          | IMU outer gimbal angle         |
| `imu.read_cdu()[1]`  | `CDUY`      | 0034          | IMU inner gimbal angle         |
| `imu.read_cdu()[2]`  | `CDUZ`      | 0035          | IMU middle gimbal angle        |
| `optics.read_cdu()[0]` | `OPTY`    | 0036          | Optics shaft angle             |
| `optics.read_cdu()[1]` | `OPTX`    | 0037          | Optics trunnion angle          |

Source: Comanche055/`ERASABLE_ASSIGNMENTS.agc`

#### Test cases

| Test | Input count | Expected `to_radians()` | Expected `to_degrees()` | Rationale |
|------|-------------|-------------------------|-------------------------|-----------|
| TC-CDU-1 | `0x0000` (0) | 0.0 | 0.0 | Zero angle |
| TC-CDU-2 | `0x4000` (16384) | π/2 ≈ 1.5707963… | 90.0 | Quarter revolution |
| TC-CDU-3 | `0x8000` (32768) | π ≈ 3.1415926… | 180.0 | Half revolution (also -180° in twos-complement) |
| TC-CDU-4 | `0xC000` (49152) | 3π/2 ≈ 4.7123889… | 270.0 | Three-quarter revolution (= -90° in twos-complement) |
| TC-CDU-5 | `0xFFFF` (65535) | TAU × (65535/65536) ≈ 6.28278… | ≈ 359.9945… | One count below full revolution |

Tolerance for all floating-point comparisons: ±1 × 10^-10 radians.

---

### 3.2 `Met` (Mission Elapsed Time)

#### Declaration

```rust
// agc-core/src/types/angle.rs
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Debug)]
pub struct Met(pub u32);
```

#### Semantics

`Met` is a monotonic centisecond counter. One unit = 1/100 second = 10 ms.
The counter is incremented by the T3RUPT handler (every 10 ms) and must never
be used for sub-centisecond timing. The inner `u32` wraps after:

```
2^32 centiseconds / (100 cs/s × 86400 s/day) ≈ 497.1 days
```

No Apollo mission approaches this duration; wrap-around is not a concern for
operational use but must not be treated as overflow.

#### Valid range and invariants

- **Storage invariant**: All `u32` values are structurally valid.
- **Monotonicity**: Within a single mission (no restart), `Met` values are
  non-decreasing. After a restart, `Met` may be restored from the pre-restart
  value stored in `ERASABLE_ASSIGNMENTS.agc` cell `TIME1`/`TIME2`.
- **Conversion site**: `Met` must be converted to `f64` seconds **only at the
  call site** where floating-point math is required (e.g., state-vector
  integration, time-of-ignition computation). Storing `f64` seconds anywhere
  is forbidden; always store `Met`.

#### AGC erasable variable mapping

| Rust field       | AGC cells         | Octal addresses | Description                       |
|------------------|-------------------|-----------------|-----------------------------------|
| `Met.0` (high)   | `TIME2`           | 0025            | DP time MSW (overflows ~31 days)  |
| `Met.0` (low)    | `TIME1`           | 0024            | DP time LSW (10 ms ticks)         |

The AGC double-precision pair is read as:
`centiseconds = TIME2 * 2^14 + TIME1` (both in ones-complement scale B-14
seconds; conversion gives centiseconds).

Source: Comanche055/`ERASABLE_ASSIGNMENTS.agc`, Comanche055/`TIME_OF_FREE_FALL.agc`

#### Methods

```rust
impl Met {
    /// Convert to f64 seconds. Use only at math call sites.
    /// Postcondition: result >= 0.0
    pub fn to_seconds(self) -> f64 {
        self.0 as f64 / 100.0
    }

    /// Construct from f64 seconds (truncates to nearest centisecond toward zero).
    /// Precondition: s >= 0.0 and s * 100.0 <= u32::MAX as f64
    /// Postcondition: Met::from_seconds(s).to_seconds() is within 0.01 s of s
    pub fn from_seconds(s: f64) -> Self {
        Met((s * 100.0) as u32)
    }

    /// Elapsed centiseconds from an earlier Met to self.
    /// Uses wrapping subtraction to handle the rare counter wrap-around.
    /// Precondition: self >= earlier in mission time (wrapping handled correctly
    ///               only if the gap is less than 2^32 centiseconds ≈ 497 days).
    /// Postcondition: result == self.0.wrapping_sub(earlier.0)
    pub fn elapsed_since(self, earlier: Met) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }
}
```

#### Test cases

| Test | Input | Operation | Expected | Rationale |
|------|-------|-----------|----------|-----------|
| TC-MET-1 | `Met(0)` | `to_seconds()` | 0.0 | Zero time |
| TC-MET-2 | `Met(100)` | `to_seconds()` | 1.0 | Exactly 1 second |
| TC-MET-3 | `Met(8640000)` | `to_seconds()` | 86400.0 | Exactly 1 day |
| TC-MET-4 | `Met::from_seconds(1.5)` | `.0` | 150 | 1.5 s → 150 cs |
| TC-MET-5 | `Met::from_seconds(0.007)` | `.0` | 0 | Truncation below 0.01 s |
| TC-MET-6 | `Met(5).elapsed_since(Met(3))` | result | 2 | Forward elapsed time |
| TC-MET-7 | `Met(1).elapsed_since(Met(0xFFFFFFFF))` | result | 2 | Wrapping arithmetic |

---

### 3.3 `DeltaV`

#### Declaration

```rust
// agc-core/src/types/angle.rs
#[derive(Clone, Copy, Default, Debug)]
pub struct DeltaV(pub Vec3);
```

#### Semantics

`DeltaV` is a newtype around `Vec3` that carries the semantic annotation
"this vector is a maneuver delta-velocity in m/s". It exists solely to prevent
passing a position vector (also `Vec3`, but in metres) to a function that
expects a delta-V argument, or vice versa. The inner `Vec3` is always
interpreted as `[dv_x, dv_y, dv_z]` in m/s, in the same coordinate frame
as the calling context (usually the inertial navigation frame).

#### Valid range and invariants

- **Structural invariant**: The inner `Vec3` must not contain `NaN` or `±Inf`.
  Callers are responsible for validation at hardware input boundaries.
- **Magnitude range**: Mission-relevant delta-V magnitudes are on the order of
  1–3000 m/s (RCS minimum impulse ≈ 0.3 m/s; SPS burn ≈ 900 m/s). There is no
  compile-time range enforcement; range checks belong in guidance logic.
- **Units**: Always m/s. Never km/s or ft/s. No internal scaling.

#### AGC erasable variable mapping

| Rust field          | AGC cells        | Octal address | Scale         | Description                  |
|---------------------|------------------|---------------|---------------|------------------------------|
| `DeltaV.0[0..=2]`   | `DELVX`, `DELVY`, `DELVZ` | 0374–0401 | B+7 m/s DP | Accumulated PIPA delta-V     |
| `DeltaV.0[0..=2]`   | `DELVSLV+0..+5`  | varies        | B+7 m/s DP | Maneuver delta-V in TVC      |

Source: Comanche055/`ERASABLE_ASSIGNMENTS.agc`

AGC to Rust conversion for a DP velocity word pair `(w_hi, w_lo)`:

```
dv_m_per_s = (w_hi * 2^-14  +  w_lo * 2^-28)  *  2^7
           = w_hi * 2^-7   +  w_lo * 2^-21
```

where `w_hi` and `w_lo` are the signed fractional values of the two
ones-complement 15-bit words (i.e. integer_value / 2^14).

#### No additional methods

`DeltaV` has no methods of its own. Callers access the inner `Vec3` directly
via `.0` and pass it to `math::linalg` functions as needed.

#### Test cases

| Test | Construction | Assertion | Rationale |
|------|-------------|-----------|-----------|
| TC-DV-1 | `DeltaV([0.0, 0.0, 0.0])` | `dv.0[0] == 0.0 && dv.0[1] == 0.0 && dv.0[2] == 0.0` | Zero delta-V |
| TC-DV-2 | `DeltaV([100.0, 0.0, 0.0])` | `dv.0[0] == 100.0` | Single-axis burn, 100 m/s |
| TC-DV-3 | AGC DP pair: `w_hi = 0x0200` (512), `w_lo = 0x0000` for one axis | `dv.0[0] ≈ 512.0 / 2^7 = 4.0 m/s` | Scaled AGC fixed-point import |
| TC-DV-4 | `DeltaV([867.0, 0.0, 0.0])` | magnitude `≈ 867.0 m/s` | Realistic SPS burn magnitude (Apollo 11 LOI ≈ 867 m/s) |

For TC-DV-3, the expected f64 value from the conversion formula:
`(512 / 16384.0) * 128.0 = 4.0 m/s` exactly.

---

### 3.4 `Vec3`

#### Declaration

```rust
// agc-core/src/types/vector.rs
pub type Vec3 = [f64; 3];
```

#### Semantics

`Vec3` is a type alias (not a newtype) for a 3-element `f64` array. It is used
for all three-dimensional physical vector quantities in the flight software:

| Calling context         | Interpretation of `[x, y, z]`              | SI unit |
|-------------------------|--------------------------------------------|---------|
| Position state vector   | Components in inertial navigation frame    | m       |
| Velocity state vector   | Components in inertial navigation frame    | m/s     |
| Acceleration            | Components in inertial navigation frame    | m/s²    |
| Delta-V (wrapped)       | See `DeltaV`                               | m/s     |
| Unit direction vector   | Dimensionless, magnitude = 1.0             | —       |
| Body-axis angular rates | Components in body frame                   | rad/s   |
| Gyro torque commands    | Components in IMU frame                    | pulses  |

The semantic meaning of a `Vec3` is determined entirely by the function signature
that accepts or returns it. The type alias deliberately avoids encoding physical
units, because doing so would require wrapping every vector in a newtype and
would make `math::linalg` operations verbose.

Unit newtypes such as `DeltaV` are used only where the same underlying type
(Vec3) would otherwise be ambiguously overloaded at an API boundary (e.g., a
function that takes both a position and a delta-V).

#### Valid range and invariants

- **NaN invariant**: No `Vec3` passed between modules may contain `f64::NAN`.
  Functions in `math::linalg` are not required to check for NaN on entry;
  invariant enforcement is the caller's responsibility at hardware I/O boundaries.
- **Infinity invariant**: No `Vec3` may contain `f64::INFINITY` or
  `f64::NEG_INFINITY`.
- **Unit vector invariant (when applicable)**: When a `Vec3` is documented as a
  unit vector, its Euclidean magnitude must be within ±1×10^-9 of 1.0.
- **No compile-time range enforcement**: The type alias provides no bounds checks.
  Range validation belongs in the subsystem that receives external data.

#### Mathematical operations

All `Vec3` operations are plain functions in `math::linalg`:

```rust
// Function signatures (defined in math::linalg, not in types::)
fn dot(a: Vec3, b: Vec3) -> f64;
fn cross(a: Vec3, b: Vec3) -> Vec3;
fn norm(v: Vec3) -> f64;
fn normalize(v: Vec3) -> Vec3;         // panics if norm ≈ 0
fn scale(v: Vec3, s: f64) -> Vec3;
fn add(a: Vec3, b: Vec3) -> Vec3;
fn sub(a: Vec3, b: Vec3) -> Vec3;
fn mat_vec(m: Mat3x3, v: Vec3) -> Vec3;
```

The `types/` module provides only the type alias; no math is implemented here.

#### AGC erasable variable mapping (selected)

| Rust usage         | AGC cells                  | Octal base | Scale  | Description          |
|--------------------|----------------------------|------------|--------|----------------------|
| Position `Vec3`    | `RN` (6 words, DP x/y/z)  | 0306       | B+28 m | CSM inertial position |
| Velocity `Vec3`    | `VN` (6 words, DP x/y/z)  | 0314       | B+7 m/s | CSM inertial velocity |
| Unit vector        | `STARAD` (6 words)         | varies     | B+0    | Star/landmark unit vector |

Source: Comanche055/`ERASABLE_ASSIGNMENTS.agc`

#### Test cases

| Test | Construction / operation | Expected | Rationale |
|------|--------------------------|----------|-----------|
| TC-V3-1 | `let v: Vec3 = [3.0, 4.0, 0.0]; math::linalg::norm(v)` | 5.0 | Pythagorean triple — norm correctness |
| TC-V3-2 | `let a: Vec3 = [1.0, 0.0, 0.0]; let b: Vec3 = [0.0, 1.0, 0.0]; math::linalg::cross(a, b)` | `[0.0, 0.0, 1.0]` | Right-hand rule cross product |
| TC-V3-3 | `let a: Vec3 = [1.0, 2.0, 3.0]; let b: Vec3 = [4.0, 5.0, 6.0]; math::linalg::dot(a, b)` | 32.0 | Dot product: 1×4 + 2×5 + 3×6 = 32 |
| TC-V3-4 | `let v: Vec3 = [0.0, 0.0, 0.0]; math::linalg::norm(v)` | 0.0 | Zero vector norm |

Note: TC-V3-1 through TC-V3-4 technically exercise `math::linalg`, not `types/`
directly. They are included here because `Vec3` is the primary type under test;
the `math::linalg` spec will reference these cases.

---

### 3.5 `Mat3x3`

#### Declaration

```rust
// agc-core/src/types/matrix.rs
pub type Mat3x3 = [[f64; 3]; 3];
```

#### Semantics

`Mat3x3` is a type alias for a 3×3 matrix in **row-major** order:

```
mat[row][col]

mat[0] = [m00, m01, m02]   (first row)
mat[1] = [m10, m11, m12]   (second row)
mat[2] = [m20, m21, m22]   (third row)
```

It is used exclusively for rotation matrices and coordinate-frame transforms.
The primary instance in the flight software is **REFSMMAT** — the
Reference-to-Stable-Member Matrix that transforms vectors from the inertial
navigation frame to the IMU stable-member frame (and its transpose for the
reverse direction).

Other uses include body-to-inertial attitude matrices and intermediate rotation
results in alignment programs P52.

#### Valid range and invariants

- **Orthonormality invariant (rotation matrices)**: When a `Mat3x3` represents a
  rotation, all three rows must be unit vectors and mutually orthogonal:
  - `norm(mat[i]) ∈ [1.0 - ε, 1.0 + ε]` for i ∈ {0, 1, 2}, ε = 1×10^-9
  - `dot(mat[i], mat[j]) ∈ [-ε, +ε]` for i ≠ j
  - `determinant(mat) ∈ [1.0 - ε, 1.0 + ε]`
  These are postcondition invariants of the alignment routines in `programs/`
  and `control/imu_control.rs`, not enforced by the type itself.
- **NaN/Inf invariant**: No element may be `f64::NAN`, `f64::INFINITY`, or
  `f64::NEG_INFINITY`.
- **No compile-time range enforcement**: Same policy as `Vec3`.

#### Mathematical operations

All `Mat3x3` operations are plain functions in `math::linalg`:

```rust
fn mat_vec(m: Mat3x3, v: Vec3) -> Vec3;
fn mat_mul(a: Mat3x3, b: Mat3x3) -> Mat3x3;
fn transpose(m: Mat3x3) -> Mat3x3;
fn identity() -> Mat3x3;
```

#### AGC erasable variable mapping

| Rust usage       | AGC cells            | Octal base | Scale  | Description               |
|------------------|----------------------|------------|--------|---------------------------|
| `REFSMMAT`       | 18 words (DP 3×3)    | E3 bank    | B+0    | Inertial-to-stable-member |
| Body-to-inertial | `BIASX`/`BIASY`/`BIASZ` + derived | varies | B+0 | IMU bias matrix (computed) |

The REFSMMAT is stored in Comanche055 at `REFSMMAT` (erasable bank E3,
18 consecutive DP words), accessible via the EBANK switching mechanism
described in `ERASABLE_ASSIGNMENTS.agc`.

Conversion from AGC 9 DP word pairs to `Mat3x3 f64` elements:

```
mat[row][col] = (w_hi[row][col] * 2^-14  +  w_lo[row][col] * 2^-28)  *  1.0
              = w_hi[row][col] * 2^-14   +  w_lo[row][col] * 2^-28
```

(Scale B+0 — no additional power-of-two multiplier needed.)

#### Test cases

| Test | Construction / operation | Expected | Rationale |
|------|--------------------------|----------|-----------|
| TC-M3-1 | Identity × `[1.0, 2.0, 3.0]` | `[1.0, 2.0, 3.0]` | Identity matrix leaves vectors unchanged |
| TC-M3-2 | 90° rotation about Z axis: `[[0,-1,0],[1,0,0],[0,0,1]]` applied to `[1.0, 0.0, 0.0]` | `[0.0, 1.0, 0.0]` | Basic rotation correctness |
| TC-M3-3 | `transpose([[1,2,3],[4,5,6],[7,8,9]])` | `[[1,4,7],[2,5,8],[3,6,9]]` | Row↔column swap |
| TC-M3-4 | Orthonormal check on 90° Z-rotation: `norm(row_i) == 1.0` and `dot(row_i, row_j) == 0.0` for i≠j | all pass | Invariant verification |
| TC-M3-5 | `mat_mul(R_z90, transpose(R_z90))` | identity (within ε = 1×10^-14) | R × Rᵀ = I for rotation matrices |

---

## 4. Module Public API

The `mod.rs` re-exports provide the complete public surface of the module:

```rust
// agc-core/src/types/mod.rs
pub mod angle;
pub mod matrix;
pub mod vector;

pub use angle::{CduAngle, DeltaV, Met};
pub use matrix::Mat3x3;
pub use vector::Vec3;
```

Consumers import via `use agc_core::types::{CduAngle, DeltaV, Mat3x3, Met, Vec3}`.

No trait implementations beyond `Clone`, `Copy`, `Debug`, `Default`, `PartialEq`,
`Eq`, `PartialOrd`, `Ord` are required on the newtype wrappers. Arithmetic
operator overloads (`Add`, `Sub`, `Mul`) must **not** be implemented on `CduAngle`
or `Met`; callers must use the explicit wrapping methods to prevent silent
overflow bugs.

---

## 5. `no_std` Compatibility

All types in this module are `#![no_std]`-compatible:

- No heap allocation (`Box`, `Vec`, `String` are absent).
- Floating-point constants use `core::f64::consts`, not `std::f64::consts`.
- The `Debug` formatter uses `core::fmt`, not `std::fmt`.
- No `std::error::Error` trait implementations are required.

---

## 6. Dependency Policy

```
agc-core/src/types/
  angle.rs     depends on: vector.rs (for Vec3 in DeltaV)
  vector.rs    depends on: nothing
  matrix.rs    depends on: nothing
  mod.rs       re-exports all
```

`types/` must not import from any other sibling module (`hal`, `math`,
`navigation`, etc.). This keeps the dependency graph acyclic with `types/` at
the root.

---

## 7. Relation to Architecture Document

Cross-reference with `docs/architecture.md` §3:

| Architecture §3 statement | Spec section confirming compliance |
|---------------------------|------------------------------------|
| §3.1: No `AgcWord` type; `f64` for nav math | §1, §3.3–3.5 (no fixed-point arithmetic types defined) |
| §3.1: `u16` for CDU gimbal angles | §3.1 (CduAngle inner field is `u16`) |
| §3.1: `i16` for signed hardware quantities (PIPA, gyro) | Not in this module — belongs in `hal/imu.rs` |
| §3.1: `u32` centiseconds for MET | §3.2 (Met inner field is `u32`) |
| §3.2: `CduAngle` with full revolution = 2^16 = 65536 counts in u16 | §2.2, §3.1 |
| §3.2: `CduAngle` newtype with `to_radians` using `TAU / 65536.0` | §3.1 |
| §3.2: `Met` with `to_seconds` / `from_seconds` | §3.2 |
| §3.2: `DeltaV(Vec3)` newtype | §3.3 |
| §3.3: `Vec3 = [f64; 3]` type alias | §3.4 |
| §3.3: `Mat3x3 = [[f64; 3]; 3]` type alias | §3.5 |
| §3.3: linalg operations in `math::linalg`, not here | §3.4, §3.5 |
| §4.1 HAL `Imu` trait: `read_cdu() -> [CduAngle; 3]` | §3.1 (CduAngle is the type delivered by IMU HAL) |

---

## 8. Out of Scope

The following are intentionally **excluded** from this module:

- PIPA (Pulse Integrating Pendulous Accelerometer) delta-V accumulation — raw
  counts arrive as `i16` from `hal::Imu::read_pipa()`; accumulation and scaling
  to `DeltaV` happen in `services::average_g`.
- Coordinate frame labels / tags — there is no `Frame<Inertial>` type parameter.
  Frame correctness is enforced by documentation and convention.
- Quaternion representation — not used in the Comanche055 port; attitude is
  handled via `Mat3x3` REFSMMAT and CDU angles.
- Display formatting beyond `Debug` — engineering-unit display formatting
  (DSKY readouts) belongs in `services::display`.

---

## 9. AGC Source Reference Summary

| Type     | Comanche055 source file             | Relevant constructs                          |
|----------|-------------------------------------|----------------------------------------------|
| `CduAngle` | `ERASABLE_ASSIGNMENTS.agc` ~110–140 | `CDUX`, `CDUY`, `CDUZ`, `OPTY`, `OPTX`     |
| `Met`    | `ERASABLE_ASSIGNMENTS.agc` ~40–60   | `TIME1` (0024), `TIME2` (0025)               |
| `DeltaV` | `ERASABLE_ASSIGNMENTS.agc` ~370–410 | `DELVX`, `DELVY`, `DELVZ` (0374–0401)       |
| `Vec3`   | `ERASABLE_ASSIGNMENTS.agc` ~300–330 | `RN` (0306), `VN` (0314), `STARAD`          |
| `Mat3x3` | `ERASABLE_ASSIGNMENTS.agc` E3 bank  | `REFSMMAT` (18 DP words)                    |

All addresses are octal as used in the Comanche055 symbolic listing.
The assembly program header in `docs/AGC Symbolic Listing.md` §Introduction
identifies the listing as "COMANCHE Revision 072 (later 055)" assembled
17 October 1969, which is Comanche055 (COLOSSUS 2D, Apollo 13).

---

## 10. Spec Quality Checklist

- [x] AGC source file and line range referenced (§2, §9)
- [x] All erasable variables and their AGC addresses listed (§3.1–3.5 mapping tables)
- [x] Scale factors documented for all fixed-point values (§2.2–2.5, §3.1–3.5)
- [x] Corresponding `f64` SI units documented (§3.1–3.5)
- [x] Input/output preconditions and postconditions stated (§3.1–3.5 method docs)
- [x] Edge cases and error handling specified (§3.1 wrapping, §3.2 truncation, §3.4–3.5 NaN/Inf)
- [x] At least 3 test cases with expected values per type (§3.1: 5, §3.2: 7, §3.3: 4, §3.4: 4, §3.5: 5)
- [x] Rust API signature designed (types, ownership) (§3.1–3.5, §4)
- [x] Invariants explicitly stated (§3.1–3.5 valid range sections)
- [x] Consistency with `docs/architecture.md` checked (§7 cross-reference table)
