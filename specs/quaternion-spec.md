# Specification: `math/quaternion` Module — Quaternion Algebra

**Status**: Approved for implementation
**Module path**: `agc-core/src/math/quaternion.rs`
**Architecture reference**: `docs/architecture.md` §9 (Navigation Math)
**Companion modules**: `agc-core/src/math/linalg.rs` (vector / matrix primitives), `agc-core/src/types/matrix.rs` (`Mat3x3`), `agc-core/src/types/vector.rs` (`Vec3`)
**Glossary cross-reference**: `docs/glossary.md` — Quaternion convention, REFSMMAT.
**Convention ADR** (in source): scalar-first `[w, x, y, z]` (inertial → body). Documented in the module's source-file header.

---

## 1. Purpose and Scope

`math::quaternion` provides the small set of quaternion primitives the
project needs for attitude work: normalisation, slerp, conversion to
and from rotation matrices, and conversion to and from AGC CDU Euler
angles. The module is **pure** (no side effects, no global state) and
**allocation-free** — every function takes its operands by value and
returns a new value, which makes it safe to use from `#![no_std]`
bare-metal builds and from interrupt handlers.

The project's `Frame` enum (in `navigation::state_vector`) is a
discriminated tag, **not** a quaternion. Quaternion values in this
module are anonymous mathematical objects: callers are responsible for
remembering which frame pair (e.g. inertial → body) a particular
quaternion represents.

### What this module provides

- `Quat = [f64; 4]` — scalar-first `[w, x, y, z]` type alias.
- `quat_normalise(q) -> Quat` — divides by L2 norm.
- `quat_to_mat3x3(q) -> Mat3x3` — unit quaternion to 3×3 rotation matrix.
- `quat_from_mat3x3(m) -> Quat` — 3×3 rotation matrix to unit quaternion via Shepperd's branched method.
- `quat_slerp(q0, q1, alpha) -> Quat` — shortest-arc spherical linear interpolation.
- `euler_to_quat(euler) -> Quat` — AGC CDU `[roll, pitch, yaw]` (XYZ intrinsic Tait-Bryan) to scalar-first unit quaternion.
- `quat_to_euler(q) -> Vec3` — inverse of `euler_to_quat`.

### What this module does NOT provide

- Quaternion **multiplication** as a public function. Composition of
  rotations is currently done via the matrix representation
  (`quat_to_mat3x3` + `matmul3`). If a future module needs a hot-path
  `quat_mul`, it can be added without changing the convention.
- Conjugation / inverse helpers. The convention is "unit quaternion",
  so the inverse is the conjugate `[w, -x, -y, -z]`; callers that need
  it inline a single subtraction.
- Logarithm / exponential maps. Not needed at present.
- Per-frame typed wrappers. Quaternion values are anonymous mathematical
  objects; the frame they represent is tracked by the calling context.

---

## 2. Convention

### 2.1 Layout

Scalar-first: `q = [w, x, y, z]` where `w` is the real (scalar)
component and `(x, y, z)` is the imaginary (vector) component.

### 2.2 Direction

`inertial → body` (active rotation). A unit quaternion `q` corresponds
to the rotation matrix `M = quat_to_mat3x3(q)` such that
`v_body = M · v_inertial`. This is the same direction the REFSMMAT
matrix uses throughout `agc-core` (`v_inertial = REFSMMAT · v_platform`
defines REFSMMAT in the reverse direction — note the asymmetry — but
internally `agc-core` works in rotation-matrix space).

### 2.3 Unit-quaternion assumption

All entry points except `quat_normalise` and `quat_from_mat3x3`
**assume** unit norm. Passing a non-unit quaternion produces a
non-rotation matrix and is the caller's bug.

### 2.4 Sign ambiguity

A rotation has two quaternion representations differing only by sign
(`q` and `-q` both represent the same rotation). The implementation
does **not** enforce a canonical sign (e.g. `w ≥ 0`) on the output of
`quat_normalise`; it preserves the input sign. `quat_from_mat3x3` does
produce a canonical sign for the identity (Shepperd's `w`-dominant
branch yields `w = 1.0`); for other matrices the sign depends on which
branch is taken — both signs are mathematically equivalent.

---

## 3. Rust API

### 3.1 Types

```rust
pub type Quat = [f64; 4];
```

### 3.2 Functions

```rust
pub fn quat_normalise(q: Quat) -> Quat;
pub fn quat_to_mat3x3(q: Quat) -> Mat3x3;
pub fn quat_from_mat3x3(m: Mat3x3) -> Quat;
pub fn quat_slerp(q0: Quat, q1: Quat, alpha: f64) -> Quat;
pub fn euler_to_quat(euler: Vec3) -> Quat;
pub fn quat_to_euler(q: Quat) -> Vec3;
```

All functions take their arguments by value and return by value. None
of them mutate any state.

---

## 4. Functional Requirements

### 4.1 `quat_normalise(q) -> Quat`

Divides each component by `||q||₂ = sqrt(w² + x² + y² + z²)`.

**Degenerate case** (`||q||² < 1e-60`, i.e. a near-zero quaternion with
no meaningful rotation interpretation):

- In `cfg(test)` builds, **panics** with the message
  `"quat_normalise: zero quaternion"` so programming errors surface
  immediately in unit tests.
- In flight builds (`cfg(not(test))`), returns the input **unchanged**
  so the caller can implement its own recovery (typically: detect that
  the result is still degenerate and fall back to a safe default
  attitude).

**Postcondition** (non-degenerate): `||result||₂ = 1.0` within `f64`
rounding.

### 4.2 `quat_to_mat3x3(q) -> Mat3x3`

Returns the 3×3 rotation matrix `M` such that `v_body = M · v_inertial`
(active rotation, inertial → body).

The implementation expands the standard quaternion-to-matrix formula:

```
M[0][0] = 1 - 2(y² + z²)
M[0][1] = 2(xy - wz)
M[0][2] = 2(xz + wy)
M[1][0] = 2(xy + wz)
M[1][1] = 1 - 2(x² + z²)
M[1][2] = 2(yz - wx)
M[2][0] = 2(xz - wy)
M[2][1] = 2(yz + wx)
M[2][2] = 1 - 2(x² + y²)
```

The result is orthonormal iff `q` was a unit quaternion. For a unit
`q`, `M^T = M^{-1}`.

### 4.3 `quat_from_mat3x3(m) -> Quat`

Inverse of `quat_to_mat3x3`. Uses **Shepperd's branched method**: the
four candidate denominators are

```
t0 = 1 + tr(M)              ←  4w²
t1 = 1 + 2·M[0][0] - tr(M)  ←  4x²
t2 = 1 + 2·M[1][1] - tr(M)  ←  4y²
t3 = 1 + 2·M[2][2] - tr(M)  ←  4z²
```

The branch with the **largest** `t` is taken; the corresponding
quaternion component is extracted as `sqrt(t)/2` (positive root) and
the other three components are recovered from the matrix entries. This
avoids the numerical singularity at `tr(M) = -1` (180° rotations) that
the naive `w = sqrt(1 + tr(M))/2` formula hits.

The result is normalised before return. The round-trip
`quat_to_mat3x3(quat_from_mat3x3(M)) ≈ M` holds for any valid rotation
matrix.

### 4.4 `quat_slerp(q0, q1, alpha) -> Quat`

Spherical linear interpolation along the **shortest arc** between
`q0` and `q1`.

1. Compute `dot = q0 · q1`.
2. If `dot < 0`, replace `q1 ← -q1` and `dot ← -dot` (shortest-arc
   convention — preserves the rotation but selects the negative-sign
   representation when it lies on the shorter geodesic).
3. Clamp `dot ≤ 1.0` to avoid `acos` domain errors from floating-point
   rounding above 1.
4. `theta = acos(dot)`.
5. If `|theta| < 1e-10` (quaternions are nearly identical), fall back
   to **linear interpolation** of the four components followed by
   normalisation. This avoids the `sin(theta) → 0` division
   instability.
6. Otherwise:
   ```
   sin_theta = sin(theta)
   s0 = sin((1 - alpha) · theta) / sin_theta
   s1 = sin(alpha · theta) / sin_theta
   result = s0 · q0 + s1 · q1   (component-wise)
   ```

Endpoints: `slerp(q0, q1, 0.0) == q0`, `slerp(q0, q1, 1.0) == q1` (the
sign-flip from step 2 is reversed at the endpoint when `alpha = 1.0`
yields exactly the original `q1`).

### 4.5 `euler_to_quat([roll, pitch, yaw]) -> Quat`

Converts AGC CDU Euler angles (XYZ intrinsic Tait-Bryan, gimbal
suspension order `Rx(roll) · Ry(pitch) · Rz(yaw)`) to a scalar-first
unit quaternion. Matches the gimbal matrix produced by
`control::attitude::gimbal_matrix_from_euler`.

**Convention**: CDU index 0 = outer/roll, 1 = inner/pitch,
2 = middle/yaw — the AGC CDU register layout.

Result is `quat_normalise`d before return.

### 4.6 `quat_to_euler(q) -> [roll, pitch, yaw]`

Inverse of `euler_to_quat`. Standard XYZ intrinsic decomposition:

```
roll  = atan2(2(wx + yz), 1 - 2(x² + y²))
pitch = asin(2(wy - zx))           clamped to ±π/2 at gimbal-lock
yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
```

**Range**: `roll, yaw ∈ (−π, π]`, `pitch ∈ [−π/2, π/2]`.

**Gimbal-lock handling**: when `|sin(pitch)| ≥ 1.0` (rounded up by
floating-point in the pitch calculation), the pitch is **clamped to
±π/2** via `copysign(π/2, sinp)`. The decomposition is ambiguous in
that regime; the chosen clamp keeps the function total without
panicking.

---

## 5. Numerical Notes

- All trigonometric / square-root calls go through `libm` for `no_std`
  determinism.
- Default tolerances for round-trip identity tests are `1e-12` (slerp,
  Euler) and `1e-14` (identity quaternion → identity matrix).
- The Shepperd branching is correct for any matrix where one of the
  four `t` candidates is positive — i.e. for any proper rotation
  matrix. An improper rotation (`det(M) = -1`) violates this
  precondition and the result is undefined.

---

## 6. Dependencies

| Dependency | Used for |
|---|---|
| `libm` | `sqrt`, `sin`, `cos`, `acos`, `asin`, `atan2`, `copysign` (the standard math library cannot be used from `#![no_std]`). |
| `crate::types::{Mat3x3, Vec3}` | Type aliases for the 3×3 matrix and 3-vector primitives. |
| `core::f64::consts::FRAC_PI_2` | Gimbal-lock clamp value. |

No dependency on any other `agc-core` module. No state. No I/O.

---

## 7. Module Layout

```
src/math/quaternion.rs
├── pub type Quat = [f64; 4]
├── pub fn quat_normalise(q: Quat) -> Quat
├── pub fn quat_to_mat3x3(q: Quat) -> Mat3x3
├── pub fn quat_from_mat3x3(m: Mat3x3) -> Quat
├── pub fn quat_slerp(q0: Quat, q1: Quat, alpha: f64) -> Quat
├── pub fn euler_to_quat(euler: Vec3) -> Quat
├── pub fn quat_to_euler(q: Quat) -> Vec3
└── #[cfg(test)] mod tests
```

---

## 8. Test Cases

The implementation in `agc-core/src/math/quaternion.rs::tests` provides
the following representative cases. Each function has at least one
identity / round-trip case and (where the algorithm has interesting
branches) at least one specific-angle case.

### 8.1 Normalisation

| ID | What is verified |
|---|---|
| `tc_quat_normalise_unit_norm` | `[2,0,0,0]` normalises to `[1,0,0,0]`; the L2 norm of the result equals 1.0 within `1e-15`. |
| `tc_quat_normalise_negative_w_canonical` | `[-1,0,0,0]` stays `[-1,0,0,0]` (sign is **not** canonicalised by `quat_normalise`); norm equals 1.0. |

### 8.2 Quaternion → Matrix

| ID | What is verified |
|---|---|
| `tc_quat_to_mat3x3_identity` | Identity quaternion `[1,0,0,0]` produces the 3×3 identity matrix within `1e-15`. |
| `tc_quat_to_mat3x3_90deg_x_rotation` | 90° rotation about +X (`q = [cos45°, sin45°, 0, 0]`) maps `[0,1,0]` to `[0,0,1]` within `1e-14` — confirms the active-rotation, inertial → body convention. |

### 8.3 Slerp

| ID | What is verified |
|---|---|
| `tc_quat_slerp_endpoints_unchanged` | `slerp(q1, q2, 0.0) == q1` and `slerp(q1, q2, 1.0) == q2` within `1e-12` per component. |

### 8.4 Matrix → Quaternion (Shepperd)

| ID | What is verified |
|---|---|
| `tc_quat_from_mat3x3_identity_roundtrip` | Identity matrix produces `[±1, 0, 0, 0]`; round-trip back to matrix is identity within `1e-14`. |
| `tc_quat_from_mat3x3_90deg_x_roundtrip` | 90°-about-X round-trip recovers the source quaternion (up to sign) within `1e-12`; matrix round-trip within `1e-12`. |
| `tc_quat_from_mat3x3_180deg_y_roundtrip` | 180°-about-Y — the `tr(M) = -1` near-singular case. Shepperd's branching must select the y-dominant branch; matrix round-trip within `1e-12`. |
| `tc_quat_from_mat3x3_sign_convention_canonical` | `quat_from_mat3x3(identity)` has `w >= 0` — pins the sign branch against regression. |
| `tc_quat_from_mat3x3_arbitrary_rotation` | Arbitrary quaternion `[0.5, -0.5, 0.3, 0.7]` round-trips through the matrix representation (up to sign) within `1e-12`. |

### 8.5 Euler ↔ Quaternion

| ID | What is verified |
|---|---|
| `tc_euler_quat_identity` | `[0,0,0]` Euler ↔ `[1,0,0,0]` quaternion, both directions. |
| `tc_euler_quat_pure_roll` | 45° roll, all other axes zero, round-trips within `1e-12`. |
| `tc_euler_quat_pure_pitch` | 30° pitch, all other axes zero, round-trips within `1e-12`. |
| `tc_euler_quat_pure_yaw` | 90° yaw, all other axes zero, round-trips within `1e-12`. |
| `tc_euler_quat_combined` | Combined `[20°, 15°, 35°]` round-trips within `1e-12`. |

The gimbal-lock branch in `quat_to_euler` is not exercised by a
dedicated test case because the precise pitch value at the lock is
ambiguous; the clamp is documented in §4.6 as the policy.

---

## 9. Spec Quality Checklist

- [x] Source-file ADR (scalar-first `[w, x, y, z]`, inertial → body) referenced (§2).
- [x] All public functions specified with preconditions and postconditions (§4).
- [x] Degenerate-input behaviour documented for `quat_normalise` (§4.1).
- [x] Shepperd's branching rationale documented (§4.3).
- [x] Slerp shortest-arc handling and small-angle fallback documented (§4.4).
- [x] Gimbal-lock policy for `quat_to_euler` documented (§4.6).
- [x] Dependencies listed (§6).
- [x] Test coverage summarised (§8).
