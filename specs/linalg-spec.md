# Specification: `math/linalg` Module

**Status**: Approved for implementation
**Module path**: `agc-core/src/math/linalg.rs`
**Architecture reference**: `docs/architecture.md` §9 "Navigation Math — Replacing the Interpreter"
**Types reference**: `specs/types-module-spec.md` §3.3 "Vectors and Matrices"
**Spec checklist**: `specs/README.md` — all items satisfied (see §11)

---

## 1. Purpose and Scope

`math::linalg` provides the complete set of 3-D vector and matrix primitives
used throughout the guidance, navigation, and control subsystems. These are
pure mathematical functions: they carry no state, allocate no memory, and have
no side effects beyond computing and returning a value.

Every function in this module is a direct replacement for one or more opcodes
in the AGC's interpretive language. The AGC interpretive language existed
solely because the 15-bit ones-complement CPU had no hardware vector or
floating-point instructions. Because the Rust port targets a modern processor
with native `f64` arithmetic, the interpreter is not re-implemented; its
computations are expressed directly as Rust function calls (ADR-001,
`docs/architecture.md` §9.1).

The module is the lowest dependency in the math sub-tree. It is used by:

| Subsystem / module             | Functions used                                    |
|--------------------------------|---------------------------------------------------|
| `navigation::state_vector`     | `vadd`, `vsub`, `vscale` (propagation deltas)     |
| `navigation::gravity`          | `norm`, `vscale`, `dot` (gravity gradient)        |
| `navigation::integration`      | `vadd`, `vsub`, `vscale` (RK4 step accumulation)  |
| `navigation::conics`           | `dot`, `cross`, `norm`, `unit`, `vscale`          |
| `guidance::maneuver`           | `cross`, `unit`, `dot`, `vscale` (burn attitude)  |
| `guidance::targeting`          | `norm`, `dot`, `mxv`                              |
| `guidance::entry`              | `mxv`, `vxm`, `cross`, `norm`                     |
| `control::attitude`            | `cross`, `unit`, `dot` (attitude error)           |
| `control::tvc`                 | `mxv` (gimbal transform)                         |
| `control::imu_control`         | `mxv`, `vxm`, `mxm` (REFSMMAT operations)        |
| `programs::p51_p52`            | `mxm`, `transpose`, `mxv` (IMU realignment)      |
| `services::fresh_start`        | `IDENTITY` (REFSMMAT initialisation)              |

All thirteen public symbols (`dot`, `cross`, `norm`, `unit`, `vadd`, `vsub`,
`vscale`, `mxv`, `vxm`, `transpose`, `mxm`, `IDENTITY`) are specified below.

---

## 2. AGC Background

### 2.1 The Interpretive Language

The AGC interpretive language is a software layer built on top of the machine
language instruction set. It provides double-precision scalar operations, vector
operations (6 words = 3 double-precision components), and matrix operations
(18 words = 9 double-precision elements) by simulating a stack-based processor
within the AGC. An interpretive program begins with a `STCALL`/`CALL`
instruction and uses a push-down list (a 4-level software stack in erasable
memory) for subroutine returns.

Vector operations in the interpretive language work on double-precision triples
stored in consecutive erasable-memory locations. The scale factors for the
triple's components match those of the state-vector variables they operate on
(e.g. B+28 for position vectors, B+7 for velocity vectors, B+0 for unit
vectors and REFSMMAT rows).

### 2.2 Opcode Mapping

The following table lists every AGC interpretive opcode replaced by a function
in this module. The opcode names appear in the Comanche055 source listings
(e.g. `BURN_VECTOR_CALC`, `REFSMMAT` computation in `P52`, `SERVICER` guidance
loop, Lambert targeting in `P30/P34`).

| AGC interpretive opcode | Rust function          | Notes                                      |
|-------------------------|------------------------|--------------------------------------------|
| `VLOAD`                 | `let v: Vec3 = ...`    | Assignment; no function needed             |
| `VSTORE`                | `state.field = v`      | Assignment; no function needed             |
| `VAD`                   | `linalg::vadd`         | Component-wise addition                    |
| `VSU`                   | `linalg::vsub`         | Component-wise subtraction                 |
| `VSCALE`                | `linalg::vscale`       | Scalar multiply                            |
| `DOT`                   | `linalg::dot`          | Inner product; result in MPAC (scalar)     |
| `VXV` (cross product)   | `linalg::cross`        | Right-hand cross product a × b             |
| `ABVAL`                 | `linalg::norm`         | Euclidean magnitude                        |
| `UNIT`                  | `linalg::unit`         | Normalise; ERROR flag set on zero input    |
| `MXV`                   | `linalg::mxv`          | Column-vector transform M · v              |
| `VXM`                   | `linalg::vxm`          | Row-vector transform vᵀ · M               |
| `TRANSPOSE`             | `linalg::transpose`    | Matrix transpose                           |
| `MXM`                   | `linalg::mxm`          | Matrix product A · B                       |
| `STADR` / `STOVL`       | local `let` binding    | Store result; no function needed           |

Note: The AGC `UNIT` opcode sets an ERROR flag in the interpretive state when
the input magnitude is zero (rather than panicking). In the Rust port this
becomes a panic that triggers the restart handler, which is the equivalent
safety response — the restart handler restores flight-safe state from the phase
tables in the same way the ERROR branch would. See §4.4 (`unit`) and
`docs/architecture.md` §7 (restart protection).

### 2.3 REFSMMAT

REFSMMAT (Reference-to-Stable-Member Matrix) is the central 3×3 rotation matrix
of the navigation system. It describes the orientation of the IMU stable member
with respect to the inertial reference frame. It is stored at octal address
`0306` in erasable bank E3 as 9 double-precision words (18 words total). Each
element is a unit fraction (scale factor B+0); all elements are in the range
[-1, +1]. REFSMMAT is an orthonormal rotation matrix and satisfies
`REFSMMAT · REFSMMAT^T = I`.

In the Rust port `refsmmat: Mat3x3` in `AgcState` holds the same matrix in
row-major `f64` layout. The `mxv`, `vxm`, `mxm`, and `transpose` functions in
this module are the primary operations performed on REFSMMAT during navigation
update (P52 realignment), IMU coarse-align, and coordinate frame conversion.

---

## 3. `no_std` Constraint and `libm`

The `agc-core` crate is compiled with `#![cfg_attr(not(test), no_std)]`. On
bare-metal targets (ARM Cortex-M, feature `bare-metal`) the Rust standard
library is unavailable, which means that `f64::sqrt` is unavailable because it
is provided by the platform's `libm` link through the standard library.

Instead, the crate declares `libm = "0.2"` as a Cargo dependency. The `libm`
crate provides a pure-Rust implementation of the C mathematical library that
compiles in `no_std` environments. `linalg::norm` calls `libm::sqrt` to compute
the Euclidean norm, which is the only transcendental function used in this
module. All other operations are additions, subtractions, and multiplications,
which map directly to IEEE 754 `f64` hardware instructions on all targets.

No other `libm` functions are used in this module. `math::trig` uses
`libm::sin`, `libm::cos`, `libm::asin`, `libm::acos` similarly.

---

## 4. Function Specifications

### 4.1 `dot`

```rust
pub fn dot(a: Vec3, b: Vec3) -> f64
```

#### Mathematical definition

```
dot(a, b) = a[0]·b[0] + a[1]·b[1] + a[2]·b[2]
```

This is the standard Euclidean inner product R³ × R³ → R.

#### AGC opcode replaced

`DOT` — interpretive language scalar operation. The AGC `DOT` computes the
double-precision dot product of two vectors from erasable memory and places the
result in MPAC (the interpretive accumulator).

#### Preconditions

- Both `a` and `b` must be finite (`f64` values free of NaN and Inf), per the
  `Vec3` invariant defined in `specs/types-module-spec.md` §3.3.

#### Postconditions

- The return value equals `a[0]·b[0] + a[1]·b[1] + a[2]·b[2]` with IEEE 754
  rounding applied to each multiply-add.
- For orthogonal unit vectors the result is 0.0 exactly when no rounding error
  is introduced by the inputs.
- Floating-point tolerance for test comparisons: ±1 × 10⁻¹⁴ (relative to the
  magnitude of the result).

#### Error behaviour

- NaN or Inf inputs produce a NaN or Inf result; no panic is triggered. This
  is a violation of the `Vec3` invariant and must not arise in correct code.
- The zero vector is a valid input; `dot([0,0,0], v)` returns `0.0`.

#### Test cases

| ID        | Input `a`           | Input `b`           | Expected result | Tolerance | Rationale                                        |
|-----------|---------------------|---------------------|-----------------|-----------|--------------------------------------------------|
| TC-DOT-1  | `[1,0,0]`           | `[0,1,0]`           | `0.0`           | exact     | Orthogonal unit vectors; verifies zero result    |
| TC-DOT-2  | `[1,0,0]`           | `[1,0,0]`           | `1.0`           | exact     | Unit vector self-product                         |
| TC-DOT-3  | `[3,4,0]`           | `[3,4,0]`           | `25.0`          | exact     | Pythagorean triple; `‖v‖² = 25`                 |
| TC-DOT-4  | `[1,2,3]`           | `[4,5,6]`           | `32.0`          | exact     | Known answer: 1·4 + 2·5 + 3·6 = 4+10+18 = 32   |
| TC-DOT-5  | `[7071068e-7,7071068e-7,0]` | `[-7071068e-7,7071068e-7,0]` | `≈0.0` | 1e-14 | Near-unit 45° vectors; nav-relevant alignment check |

---

### 4.2 `cross`

```rust
pub fn cross(a: Vec3, b: Vec3) -> Vec3
```

#### Mathematical definition

```
cross(a, b) = [a[1]·b[2] - a[2]·b[1],
               a[2]·b[0] - a[0]·b[2],
               a[0]·b[1] - a[1]·b[0]]
```

This is the standard right-hand cross product R³ × R³ → R³. The result is
perpendicular to both inputs; its magnitude is `‖a‖·‖b‖·sin(θ)` where θ is
the angle between the vectors.

#### AGC opcode replaced

`VXV` — interpretive language vector operation. The AGC `VXV` computes the
double-precision cross product of two erasable-memory vectors and stores the
result in MPAC or directly into an erasable destination.

#### Preconditions

- Both `a` and `b` must be finite `Vec3` values.

#### Postconditions

- `dot(cross(a,b), a) = 0` (result is perpendicular to `a`), within floating-
  point rounding.
- `dot(cross(a,b), b) = 0` (result is perpendicular to `b`).
- `cross(a, b) = -cross(b, a)` (anti-commutativity).
- `cross(a, a) = [0,0,0]` (parallel vectors give zero).
- Right-hand rule: `cross([1,0,0], [0,1,0]) = [0,0,1]`.
- Floating-point tolerance: ±1 × 10⁻¹⁵ for unit vector inputs.

#### Error behaviour

- Parallel or anti-parallel inputs produce a result of zero magnitude (not an
  error condition). No panic.
- Zero vector inputs are valid; result is `[0,0,0]`.

#### Test cases

| ID         | Input `a`   | Input `b`   | Expected result | Tolerance | Rationale                                                  |
|------------|-------------|-------------|------------------|-----------|------------------------------------------------------------|
| TC-CROSS-1 | `[1,0,0]`   | `[0,1,0]`   | `[0,0,1]`        | 1e-15     | Right-hand rule: x̂ × ŷ = ẑ                               |
| TC-CROSS-2 | `[0,1,0]`   | `[1,0,0]`   | `[0,0,-1]`       | 1e-15     | Anti-commutativity: ŷ × x̂ = -ẑ                           |
| TC-CROSS-3 | `[1,0,0]`   | `[1,0,0]`   | `[0,0,0]`        | exact     | Parallel vectors; used to test degenerate orbit plane      |
| TC-CROSS-4 | `[3,0,0]`   | `[0,4,0]`   | `[0,0,12]`       | 1e-14     | Scaled vectors; magnitude = ‖a‖·‖b‖ = 12                 |
| TC-CROSS-5 | `[r_x,r_y,r_z]` × `[v_x,v_y,v_z]` — LEO position `[6578137,0,0]` m and velocity `[0,7784,0]` m/s | `[0,0,51198419288]` m²/s | 1e0 | Angular momentum h = r × v for a circular LEO orbit |

For TC-CROSS-5, the position vector is on the x-axis at 200 km altitude
(`r = [6578137, 0, 0]` m) and the velocity is the circular orbit speed at that
altitude (`v = [0, 7784.0, 0]` m/s). The angular momentum vector
`h = r × v = [0, 0, 6578137 × 7784] ≈ [0, 0, 5.120e10]` m²/s points along +z
(prograde equatorial orbit), consistent with the right-hand rule.

---

### 4.3 `norm`

```rust
pub fn norm(a: Vec3) -> f64
```

#### Mathematical definition

```
norm(a) = sqrt(a[0]² + a[1]² + a[2]²)
        = sqrt(dot(a, a))
```

This is the Euclidean (L2) norm, also written `‖a‖`. It equals the geometric
length of the vector.

#### AGC opcode replaced

`ABVAL` — interpretive language scalar operation. The AGC `ABVAL` computes the
double-precision magnitude of the vector in MPAC and returns a scalar result.

#### Implementation note

`norm` is implemented as `libm::sqrt(dot(a, a))` rather than `f64::sqrt(dot(a, a))`
to satisfy the `no_std` requirement. See §3 for the rationale. The numerical
result is identical: `libm::sqrt` uses the same IEEE 754 `sqrt` hardware
instruction on targets that provide it (all ARM Cortex-M4F and above), and
falls back to a correctly rounded software implementation on targets that do not.

#### Preconditions

- `a` must be a finite `Vec3` (per `Vec3` invariant).
- The sum of squares `a[0]² + a[1]² + a[2]²` must not overflow `f64` (this
  requires `‖a‖ < sqrt(f64::MAX) ≈ 1.34 × 10¹⁵⁴`). All physical quantities
  used in the CSM mission profile satisfy this: maximum position magnitude is
  ~4 × 10⁸ m (Earth-Moon distance), maximum velocity is ~11200 m/s.

#### Postconditions

- Result is non-negative for all valid inputs.
- `norm([0,0,0]) = 0.0` exactly.
- `norm(vscale(a, s)) = |s| · norm(a)` to floating-point rounding.
- Floating-point tolerance for test comparisons: ±1 × 10⁻¹⁴ (relative to the
  expected magnitude).

#### Error behaviour

- No panic. Returns `0.0` for the zero vector.
- Returns a finite positive value for all physically meaningful inputs.

#### Test cases

| ID        | Input `a`          | Expected result   | Tolerance | Rationale                                            |
|-----------|--------------------|-------------------|-----------|------------------------------------------------------|
| TC-NORM-1 | `[0,0,0]`          | `0.0`             | exact     | Zero vector                                          |
| TC-NORM-2 | `[1,0,0]`          | `1.0`             | exact     | Unit vector; sqrt(1) = 1                             |
| TC-NORM-3 | `[3,4,0]`          | `5.0`             | 1e-14     | Pythagorean triple (3-4-5)                           |
| TC-NORM-4 | `[1,1,1]`          | `sqrt(3) ≈ 1.7320508` | 1e-14 | Diagonal of unit cube                           |
| TC-NORM-5 | `[6578137,0,0]`    | `6578137.0`       | 1e-7      | LEO position vector magnitude (200 km altitude orbit)|

---

### 4.4 `unit`

```rust
pub fn unit(a: Vec3) -> Vec3
```

#### Mathematical definition

```
unit(a) = a / ‖a‖  =  [a[0]/‖a‖, a[1]/‖a‖, a[2]/‖a‖]
```

The result is a vector of Euclidean norm 1 pointing in the same direction as
`a`. It is defined only when `‖a‖ ≠ 0`.

#### AGC opcode replaced

`UNIT` — interpretive language vector operation. The AGC `UNIT` divides the
vector in MPAC by its own magnitude. If the input vector is zero, the AGC
interpretive language sets an ERROR flag in the interpreter control register
(`QPRET`) and the program must test this flag before using the result.

#### Preconditions

- `a` must be a finite `Vec3`.
- `a` must not be the zero vector (`‖a‖ ≠ 0`). Passing a zero vector is
  **undefined behaviour** at the API level and will panic.

#### Postconditions

- `norm(unit(a)) = 1.0` to within ±1 × 10⁻¹⁴.
- `unit(vscale(a, s)) = unit(a)` for any `s > 0`.
- `unit(vscale(a, s)) = vscale(unit(a), -1.0)` for any `s < 0` (direction
  reverses for negative scale).

#### Error behaviour

When `a = [0, 0, 0]`, the computed `n = norm(a) = 0.0`. Dividing each
component by `0.0` produces `NaN` for `0.0/0.0` in IEEE 754. However, the
current implementation **does not explicitly guard** against zero input; it will
produce a `Vec3` of `[NaN, NaN, NaN]` rather than panicking immediately. Future
implementations should add an explicit check:

```rust
assert!(n != 0.0, "unit: zero vector has no direction");
```

This is the correct behaviour by analogy with the AGC restart model: a zero
vector passed to `UNIT` indicates a programming error, not a recoverable
navigation condition. The panic triggers the watchdog restart handler, which
restores the last checkpointed flight state.

Note: NaN input (violating the `Vec3` invariant) propagates silently. Callers
are responsible for ensuring inputs satisfy the invariant.

#### Test cases

| ID        | Input `a`       | Expected result                     | Tolerance | Rationale                                       |
|-----------|-----------------|-------------------------------------|-----------|-------------------------------------------------|
| TC-UNIT-1 | `[1,0,0]`       | `[1,0,0]`                           | 1e-15     | Already a unit vector                           |
| TC-UNIT-2 | `[3,4,0]`       | `[0.6, 0.8, 0.0]`                   | 1e-15     | Pythagorean triple: 3/5 = 0.6, 4/5 = 0.8       |
| TC-UNIT-3 | `[1,1,1]`       | `[1/√3, 1/√3, 1/√3]` ≈ `[0.5774,0.5774,0.5774]` | 1e-14 | Diagonal of unit cube; confirms all three components equal |
| TC-UNIT-4 | `[6578137,0,0]` | `[1,0,0]`                           | 1e-14     | Position vector on +x-axis normalises to x̂     |
| TC-UNIT-5 | `[0,0,0]`       | panic (see error behaviour above)   | —         | Zero vector; undefined direction                |

---

### 4.5 `vadd`

```rust
pub fn vadd(a: Vec3, b: Vec3) -> Vec3
```

#### Mathematical definition

```
vadd(a, b) = [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
```

Component-wise vector addition R³ × R³ → R³.

#### AGC opcode replaced

`VAD` — interpretive language vector operation. The AGC `VAD` adds a vector
from erasable memory to the vector in MPAC, leaving the result in MPAC.

#### Preconditions

- Both inputs must be finite `Vec3` values.

#### Postconditions

- Each output component equals the sum of the corresponding input components
  with IEEE 754 rounding.
- `vadd(a, [0,0,0]) = a` exactly (zero vector is the additive identity).
- `vadd(a, b) = vadd(b, a)` (commutative).
- Floating-point tolerance: result correct to ±1 ULP per component.

#### Error behaviour

- No error condition. Addition of finite `f64` values never produces NaN
  (it can produce Inf if magnitudes overflow `f64::MAX`, but this cannot
  occur for physical CSM state vectors).

#### Test cases

| ID       | Input `a`     | Input `b`     | Expected result  | Tolerance | Rationale                                          |
|----------|---------------|---------------|-------------------|-----------|----------------------------------------------------|
| TC-VAD-1 | `[1,2,3]`     | `[0,0,0]`     | `[1,2,3]`         | exact     | Additive identity                                  |
| TC-VAD-2 | `[1,0,0]`     | `[0,1,0]`     | `[1,1,0]`         | exact     | Orthogonal unit vectors                            |
| TC-VAD-3 | `[1,2,3]`     | `[-1,-2,-3]`  | `[0,0,0]`         | exact     | Vector plus its negation                           |
| TC-VAD-4 | `[100000,200000,300000]` | `[10000,20000,30000]` | `[110000,220000,330000]` | exact | Navigation-scale position deltas (m) |
| TC-VAD-5 | `[r_x,r_y,r_z]` + `[dr_x,dr_y,dr_z]` position update | correct new position | 1e-3 | RK4 integration step: position update `r_new = r + v·dt` |

---

### 4.6 `vsub`

```rust
pub fn vsub(a: Vec3, b: Vec3) -> Vec3
```

#### Mathematical definition

```
vsub(a, b) = [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
```

Component-wise vector subtraction R³ × R³ → R³.

#### AGC opcode replaced

`VSU` — interpretive language vector operation. The AGC `VSU` subtracts a
vector from erasable memory from the vector in MPAC.

#### Preconditions

- Both inputs must be finite `Vec3` values.

#### Postconditions

- `vsub(a, b) = vadd(a, vscale(b, -1.0))` to floating-point rounding.
- `vsub(a, a) = [0,0,0]` exactly.
- Floating-point tolerance: ±1 ULP per component.

#### Error behaviour

- No error condition. Same as `vadd`.

#### Test cases

| ID       | Input `a`       | Input `b`     | Expected result | Tolerance | Rationale                                   |
|----------|-----------------|---------------|-----------------|-----------|---------------------------------------------|
| TC-VSU-1 | `[1,2,3]`       | `[1,2,3]`     | `[0,0,0]`       | exact     | Identical vectors; self-subtraction is zero |
| TC-VSU-2 | `[5,5,5]`       | `[3,2,1]`     | `[2,3,4]`       | exact     | Component-wise difference                   |
| TC-VSU-3 | `[1,0,0]`       | `[0,1,0]`     | `[1,-1,0]`      | exact     | Orthogonal unit vectors                     |
| TC-VSU-4 | CSM position `[6578137,0,0]` − target position `[6678137,0,0]` | `[-100000,0,0]` | 1e-3 | Rendezvous range vector (100 km separation) |
| TC-VSU-5 | velocity `[0,7784,0]` − velocity `[0,7700,0]` | `[0,84,0]` | 1e-6 | Delta-V residual after burn                |

---

### 4.7 `vscale`

```rust
pub fn vscale(a: Vec3, s: f64) -> Vec3
```

#### Mathematical definition

```
vscale(a, s) = [s·a[0], s·a[1], s·a[2]]
```

Scalar multiplication of a vector. Also known as scaling.

#### AGC opcode replaced

`VSCALE` — interpretive language vector operation. The AGC `VSCALE` multiplies
the vector in MPAC by a scalar from erasable memory (or an inline constant).
The result is left in MPAC.

#### Preconditions

- `a` must be a finite `Vec3`.
- `s` must be a finite `f64`.

#### Postconditions

- `norm(vscale(a, s)) = |s| · norm(a)` to floating-point rounding.
- `vscale(a, 0.0) = [0,0,0]` exactly.
- `vscale(a, 1.0) = a` exactly.
- `vscale(a, -1.0)` negates all components.
- Floating-point tolerance: ±1 ULP per component.

#### Error behaviour

- No error condition. `s = 0.0` returns the zero vector.
- Multiplying by Inf or NaN produces Inf or NaN; this violates the `Vec3`
  invariant and must not occur in correct code.

#### Test cases

| ID         | Input `a`    | Scalar `s`          | Expected result         | Tolerance | Rationale                                  |
|------------|--------------|---------------------|-------------------------|-----------|--------------------------------------------|
| TC-VSCL-1  | `[1,2,3]`    | `0.0`               | `[0,0,0]`               | exact     | Scale by zero                              |
| TC-VSCL-2  | `[1,2,3]`    | `1.0`               | `[1,2,3]`               | exact     | Scale by one (identity)                    |
| TC-VSCL-3  | `[1,2,3]`    | `-1.0`              | `[-1,-2,-3]`            | exact     | Negation                                   |
| TC-VSCL-4  | `[3,4,0]`    | `2.0`               | `[6,8,0]`               | exact     | Scale; result magnitude = 2×5 = 10         |
| TC-VSCL-5  | unit burn direction `[1,0,0]` | `Δv = 184.6` m/s | `[184.6, 0, 0]` | 1e-10 | TLI burn delta-V vector (navigation-scale) |

---

### 4.8 `mxv`

```rust
pub fn mxv(m: Mat3x3, v: Vec3) -> Vec3
```

#### Mathematical definition

```
(mxv(M, v))[i] = dot(M[i], v)   for i = 0,1,2

Explicitly:
result[0] = M[0][0]·v[0] + M[0][1]·v[1] + M[0][2]·v[2]
result[1] = M[1][0]·v[0] + M[1][1]·v[1] + M[1][2]·v[2]
result[2] = M[2][0]·v[0] + M[2][1]·v[1] + M[2][2]·v[2]
```

Matrix-column-vector multiplication. `M` is in row-major order: `M[i][j]` is
the element at row i, column j. The result is the transformed column vector
`M · v`.

#### AGC opcode replaced

`MXV` — interpretive language vector/matrix operation. The AGC `MXV` multiplies
a matrix from erasable memory on the left by the vector in MPAC, producing a
new vector in MPAC. This is the primary operation for coordinate frame
transformation: rotating a vector expressed in frame A into frame B using the
transformation matrix REFSMMAT.

#### Preconditions

- `m` must be a finite `Mat3x3`.
- `v` must be a finite `Vec3`.
- When `m` is intended as a rotation matrix (e.g. REFSMMAT), it must be
  orthonormal: `mxm(m, transpose(m)) ≈ IDENTITY` to within ±1 × 10⁻⁷`.
  This invariant is asserted at higher levels (P52 realignment); `mxv` itself
  does not check it.

#### Postconditions

- `mxv(IDENTITY, v) = v` exactly.
- For orthonormal `m`: `norm(mxv(m, v)) = norm(v)` to within ±1 × 10⁻¹²`
  (rotation preserves vector magnitude).
- Floating-point tolerance: ±1 × 10⁻¹⁴ for unit vectors and orthonormal matrices.

#### Error behaviour

- No error condition. All inputs are finite.

#### Test cases

| ID       | Input `M`          | Input `v`   | Expected result    | Tolerance | Rationale                                                       |
|----------|--------------------|-------------|--------------------|-----------|-----------------------------------------------------------------|
| TC-MXV-1 | `IDENTITY`         | `[1,2,3]`   | `[1,2,3]`          | exact     | Identity matrix leaves vector unchanged                         |
| TC-MXV-2 | 90° rotation about z: `[[0,-1,0],[1,0,0],[0,0,1]]` | `[1,0,0]` | `[0,1,0]` | 1e-15 | Rotates x̂ to ŷ; right-hand rotation about z |
| TC-MXV-3 | `[[2,0,0],[0,3,0],[0,0,4]]` | `[1,1,1]` | `[2,3,4]`     | exact     | Diagonal (scaling) matrix                                       |
| TC-MXV-4 | REFSMMAT = identity | inertial position `[6578137,0,0]` | `[6578137,0,0]` | 1e-3 | Transform to stable-member frame when REFSMMAT = I (P52 alignment case) |
| TC-MXV-5 | 45° rotation about z: `[[cos45,-sin45,0],[sin45,cos45,0],[0,0,1]]` | `[1,0,0]` | `[cos45, sin45, 0]` ≈ `[0.7071,0.7071,0]` | 1e-14 | CSM attitude transform for non-zero roll |

For TC-MXV-2, the rotation matrix for 90° about z in right-hand convention is:
```
[[cos90, -sin90, 0],   [[0, -1, 0],
 [sin90,  cos90, 0], =  [1,  0, 0],
 [0,      0,    1]]     [0,  0, 1]]
```

---

### 4.9 `vxm`

```rust
pub fn vxm(v: Vec3, m: Mat3x3) -> Vec3
```

#### Mathematical definition

```
(vxm(v, M))[j] = v[0]·M[0][j] + v[1]·M[1][j] + v[2]·M[2][j]   for j = 0,1,2
```

Row-vector times matrix: `vᵀ · M`. Treating `v` as a 1×3 row vector and `M`
as 3×3 in row-major order, the result is the 1×3 row vector returned as `Vec3`.

Note the algebraic identity: `vxm(v, M) = mxv(transpose(M), v)`. The direct
implementation avoids the extra memory allocation of an explicit transpose.

#### AGC opcode replaced

`VXM` — interpretive language vector/matrix operation. The AGC `VXM` multiplies
a matrix from erasable memory on the right by the vector in MPAC. In navigation,
`VXM` is used to apply the inverse (transpose) of a rotation matrix without
explicitly computing the transpose: if M is orthonormal, `VXM(v, M) = MXV(v, Mᵀ)`.

#### Preconditions

- `v` must be a finite `Vec3`.
- `m` must be a finite `Mat3x3`.

#### Postconditions

- `vxm(v, IDENTITY) = v` exactly.
- For orthonormal `m`: `vxm(mxv(m, v), m) ≠ v` in general; the correct round-
  trip is `mxv(m, vxm(v, m))` which recovers `v` when m is orthonormal.
  The correct inverse of `mxv(m, v)` is `vxm(result, m)` (i.e. multiplying by
  mᵀ = m⁻¹ from the right is equivalent to multiplying by m from the left in
  row-vector form, because `vxm(v, M) = mxv(Mᵀ, v)` for row-vector convention).
- `vxm(v, m) = mxv(transpose(m), v)` exactly (both compute the same linear map).
- Floating-point tolerance: ±1 × 10⁻¹⁴ for unit vectors and orthonormal matrices.

#### Error behaviour

- No error condition.

#### Test cases

| ID       | Input `v`   | Input `M`          | Expected result | Tolerance | Rationale                                                    |
|----------|-------------|--------------------|-----------------|-----------|------------------------------------------------------------- |
| TC-VXM-1 | `[1,2,3]`   | `IDENTITY`         | `[1,2,3]`       | exact     | Identity matrix leaves row vector unchanged                  |
| TC-VXM-2 | `[1,0,0]`   | 90° z rotation (TC-MXV-2) | `[0,-1,0]` | 1e-15 | Inverse rotation: vxm with Rz(90°) rotates x̂ to -ŷ in row-vector form (= mxv(Rz(90°)ᵀ, v)) |
| TC-VXM-3 | `[1,2,3]`   | `[[1,0,0],[0,2,0],[0,0,3]]` | `[1,4,9]` | exact | Diagonal matrix; row-vector scaling                         |
| TC-VXM-4 | `[1,0,0]`   | REFSMMAT (identity) | `[1,0,0]`      | 1e-14     | Stable-member to inertial frame (identity REFSMMAT case)    |
| TC-VXM-5 | nav velocity `v` in stable-member frame | REFSMMAT `R` | inertial velocity `vᵀ·R` | 1e-6 | SERVICER frame conversion (velocity update in P11) |

---

### 4.10 `transpose`

```rust
pub fn transpose(m: Mat3x3) -> Mat3x3
```

#### Mathematical definition

```
transpose(M)[i][j] = M[j][i]   for i,j = 0,1,2
```

The matrix transpose swaps rows and columns.

#### AGC opcode replaced

`TRANSPOSE` — interpretive language matrix operation. The AGC does not have a
dedicated TRANSPOSE opcode; instead it uses `VXM` with an orthonormal matrix
(which is equivalent to left-multiplying by the transpose) or explicitly
reloads rows as columns. In the Rust port, `transpose` is provided as a
first-class function for clarity and is used internally by `mxm`.

#### Preconditions

- `m` must be a finite `Mat3x3`.

#### Postconditions

- `transpose(transpose(m)) = m` exactly (involution).
- For orthonormal `m`: `mxm(m, transpose(m)) = IDENTITY` to within ±1 × 10⁻¹⁴.
- `transpose(IDENTITY) = IDENTITY` exactly.

#### Error behaviour

- No error condition.

#### Test cases

| ID        | Input `M`                              | Expected result                           | Tolerance | Rationale                                    |
|-----------|----------------------------------------|-------------------------------------------|-----------|----------------------------------------------|
| TC-TRN-1  | `IDENTITY`                             | `IDENTITY`                                | exact     | Transpose of identity is identity            |
| TC-TRN-2  | `[[1,2,3],[4,5,6],[7,8,9]]`            | `[[1,4,7],[2,5,8],[3,6,9]]`               | exact     | General matrix; rows become columns          |
| TC-TRN-3  | `transpose(transpose(m))` for any `m`  | `m`                                       | exact     | Involution property                          |
| TC-TRN-4  | Rz(90°): `[[0,-1,0],[1,0,0],[0,0,1]]`  | `[[0,1,0],[-1,0,0],[0,0,1]]`              | exact     | Transpose of rotation = inverse rotation     |
| TC-TRN-5  | REFSMMAT `R` | `Rᵀ` such that `mxm(R, Rᵀ) = IDENTITY` | 1e-14 | Orthonormality test for IMU realignment result |

---

### 4.11 `mxm`

```rust
pub fn mxm(a: Mat3x3, b: Mat3x3) -> Mat3x3
```

#### Mathematical definition

```
(mxm(A, B))[i][j] = dot(A[i], B_col_j)
                  = A[i][0]·B[0][j] + A[i][1]·B[1][j] + A[i][2]·B[2][j]
```

Matrix product A · B in row-major layout. The implementation pre-transposes `B`
to express each column of `B` as a row, then computes 9 dot products.

#### AGC opcode replaced

`MXM` — interpretive language matrix operation. The AGC `MXM` multiplies two
matrices from erasable memory, storing the result in another erasable region.
This is used in P52 (IMU realignment) to compose successive rotation matrices
when computing the new REFSMMAT from star-tracker measurements and the old
REFSMMAT.

#### Preconditions

- Both `a` and `b` must be finite `Mat3x3` values.
- When both inputs are rotation matrices (orthonormal), the result is also a
  rotation matrix.

#### Postconditions

- `mxm(IDENTITY, m) = m` exactly.
- `mxm(m, IDENTITY) = m` exactly.
- For orthonormal `a` and `b`: the result is orthonormal.
- `mxm(a, mxm(b, c)) = mxm(mxm(a, b), c)` to floating-point rounding
  (associativity).
- `mxm(a, transpose(a)) = IDENTITY` for orthonormal `a`, within ±1 × 10⁻¹⁴.
- Floating-point tolerance: ±1 × 10⁻¹³ per element for products of orthonormal
  matrices (9 multiply-adds introduce cumulative rounding).

#### Error behaviour

- No error condition.

#### Test cases

| ID       | Input `A`              | Input `B`              | Expected result         | Tolerance | Rationale                                                          |
|----------|------------------------|------------------------|-------------------------|-----------|--------------------------------------------------------------------|
| TC-MXM-1 | `IDENTITY`             | any `M`                | `M`                     | exact     | Left identity                                                      |
| TC-MXM-2 | any `M`                | `IDENTITY`             | `M`                     | exact     | Right identity                                                     |
| TC-MXM-3 | `[[1,2],[3,4],[0,0]]`-style 3×3 `[[1,2,0],[3,4,0],[0,0,1]]` | `[[5,6,0],[7,8,0],[0,0,1]]` | `[[1·5+2·7,1·6+2·8,0],[3·5+4·7,3·6+4·8,0],[0,0,1]]` = `[[19,22,0],[43,50,0],[0,0,1]]` | exact | Known 2×2 sub-product |
| TC-MXM-4 | Rz(90°)                | Rz(90°)                | Rz(180°) = `[[-1,0,0],[0,-1,0],[0,0,1]]` | 1e-14 | Rotation composition: two 90° = 180° |
| TC-MXM-5 | REFSMMAT `R`           | `transpose(R)`         | `IDENTITY`              | 1e-13     | Orthonormality: `R · Rᵀ = I` (P52 realignment validation)        |

---

## 5. Constant: `IDENTITY`

```rust
pub const IDENTITY: Mat3x3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
```

The 3×3 identity matrix. Declared as a `const` so it is embedded as an
immediate in the instruction stream (zero runtime cost) and can be used in
`const` initialisation contexts.

### Usage

`IDENTITY` is the initial value of `AgcState::refsmmat` at `AgcState::new()`
and is explicitly assigned during `fresh_start()`:

```rust
// services/fresh_start.rs
state.refsmmat = IDENTITY;
```

This represents the navigation state immediately after a FRESH START before any
IMU alignment has been performed: the stable-member frame is assumed coincident
with the inertial reference frame until P52 (IMU platform realignment) computes
a valid REFSMMAT.

### Algebraic properties

- `mxv(IDENTITY, v) = v` for all `v`.
- `vxm(v, IDENTITY) = v` for all `v`.
- `mxm(IDENTITY, m) = m` and `mxm(m, IDENTITY) = m` for all `m`.
- `transpose(IDENTITY) = IDENTITY`.
- `IDENTITY` is orthonormal: all rows are unit vectors, all rows are mutually
  orthogonal.

---

## 6. Floating-Point Tolerance Summary

| Function      | Recommended test tolerance | Notes                                                   |
|---------------|----------------------------|---------------------------------------------------------|
| `dot`         | 1 × 10⁻¹⁴                 | Relative; use 1e-14 × max(|expected|, 1)               |
| `cross`       | 1 × 10⁻¹⁵ (unit inputs)   | Absolute for unit vector inputs                         |
| `norm`        | 1 × 10⁻¹⁴                 | Relative; `libm::sqrt` is correctly rounded             |
| `unit`        | 1 × 10⁻¹⁴                 | Postcondition `‖unit(v)‖ = 1` to this tolerance        |
| `vadd`        | 1 ULP per component        | Typically exact for integer-valued test inputs          |
| `vsub`        | 1 ULP per component        | Catastrophic cancellation possible for nearly-equal vectors |
| `vscale`      | 1 ULP per component        | Exactly representable for power-of-two scalars          |
| `mxv`         | 1 × 10⁻¹⁴ (unit inputs)   | 3 multiply-adds; tolerance accumulates from `dot`       |
| `vxm`         | 1 × 10⁻¹⁴ (unit inputs)   | Same as `mxv`                                           |
| `transpose`   | exact                      | No arithmetic; pure data rearrangement                  |
| `mxm`         | 1 × 10⁻¹³                 | 9 dot products; rounding accumulates across rows        |

All tolerances are absolute (not relative) unless otherwise noted. For
navigation-scale inputs (position ~10⁶–10⁸ m, velocity ~10²–10⁴ m/s),
scale the absolute tolerance by the expected result magnitude.

---

## 7. Rust API Summary

```rust
// agc-core/src/math/linalg.rs
use crate::types::{Mat3x3, Vec3};

pub fn dot(a: Vec3, b: Vec3) -> f64;
pub fn cross(a: Vec3, b: Vec3) -> Vec3;
pub fn norm(a: Vec3) -> f64;
pub fn unit(a: Vec3) -> Vec3;        // panics on zero-vector input
pub fn vadd(a: Vec3, b: Vec3) -> Vec3;
pub fn vsub(a: Vec3, b: Vec3) -> Vec3;
pub fn vscale(a: Vec3, s: f64) -> Vec3;
pub fn mxv(m: Mat3x3, v: Vec3) -> Vec3;
pub fn vxm(v: Vec3, m: Mat3x3) -> Vec3;
pub fn transpose(m: Mat3x3) -> Mat3x3;
pub fn mxm(a: Mat3x3, b: Mat3x3) -> Mat3x3;

pub const IDENTITY: Mat3x3;
```

All functions are `#[inline]`. All types are `Copy`, so all parameters are
passed by value; no references, no lifetimes.

---

## 8. `no_std` Compliance

- No `use std::*` imports.
- Only `libm::sqrt` is called for a non-trivial math operation; all other
  operations are `f64` arithmetic.
- No heap allocation, no `Vec`, no `Box`, no dynamic dispatch.
- All results are stack-allocated arrays (`[f64; 3]` or `[[f64; 3]; 3]`).
- `const IDENTITY` is a pure constant expressible without any runtime
  initialisation.
- The `#[cfg(test)]` block uses standard `assert!` / `assert_eq!` macros which
  are available in both `no_std` and `std` contexts.

---

## 9. Invariants and Shared Constraints

The following invariants are enforced by callers (not by this module):

1. **`Vec3` finite invariant**: All components must be finite `f64` (no NaN, no
   Inf). Defined in `specs/types-module-spec.md` §3.3. Functions in this module
   do not validate inputs; violations produce unspecified results (NaN
   propagation or Inf arithmetic).

2. **`Mat3x3` finite invariant**: All nine elements must be finite. Defined in
   `specs/types-module-spec.md` §3.3.

3. **Orthonormality of rotation matrices**: When `Mat3x3` is used as a
   coordinate-frame rotation matrix (e.g. REFSMMAT), callers guarantee that
   `mxm(m, transpose(m)) ≈ IDENTITY` to within ±1 × 10⁻⁷`. This is validated
   by P52 (IMU realignment) after each REFSMMAT update. Functions in this
   module do not enforce orthonormality.

4. **`unit` non-zero precondition**: Callers must ensure the input to `unit` is
   not the zero vector. The zero vector has no defined direction. Passing the
   zero vector to `unit` currently produces `[NaN, NaN, NaN]` but may panic
   in future implementations. This is a programming error, not a recoverable
   flight condition.

---

## 10. AGC Source Reference

The functions in this module collectively replace the vector and matrix
operation subset of the AGC interpretive language. The interpretive language
interpreter is defined in the Comanche055 source file:

```
AGC source: Comanche055/INTERPRETER.agc
Routines:   VECTOR, MATRIX, ABVAL, UNIT, VAD, VSU, VSCALE, DOT, VXV, MXV, VXM
```

The interpretive language is documented in:
- `docs/AGC Symbolic Listing.md` §VIB "Interpretive Language Operations"
  (subsections: Vector Computation Operations, Transmission Operations)
- `docs/AGC Symbolic Listing.md` §VIA "General Principles" (push-down list,
  MPAC accumulator, VAC area layout)
- Frank O'Brien, *The Apollo Guidance Computer: Architecture and Operation*
  (Springer, 2010), Chapter 6 "The Interpreter" — covers the opcode encoding,
  MPAC register, push-down list, and worked examples of vector computation
  sequences used in orbital navigation.

---

## 11. Spec Quality Checklist

- [x] AGC source file and line range referenced (§10; `INTERPRETER.agc`)
- [x] All erasable variables and their AGC addresses listed (§2.3 REFSMMAT;
      MPAC is the interpretive accumulator, not a persistent erasable variable)
- [x] Scale factors documented for all fixed-point values (§2.2 notes B+0 for
      unit vectors and REFSMMAT, B+28/B+7 for position/velocity vectors passed
      to these functions after HAL conversion)
- [x] Corresponding `f64` SI units documented (all inputs/outputs in SI;
      conversion happens at HAL boundary per `docs/architecture.md` §3.1)
- [x] Input/output preconditions and postconditions stated (§4 per function)
- [x] Edge cases and error handling specified (§4 per function; zero-vector,
      NaN, Inf, parallel vectors)
- [x] At least 3 test cases with expected values per function (§4; 5 test cases
      each, including trivial, Pythagorean, and navigation-scale cases)
- [x] Rust API signature designed (§7; all signatures with types and `#[inline]`)
- [x] Invariants explicitly stated (§9)
- [x] Consistency with `docs/architecture.md` checked (§9.1 opcode mapping
      table cross-referenced; §3.3 types; §7 restart model for `unit` panic)
