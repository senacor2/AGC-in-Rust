# Functional Specification: Vector and Matrix Linear Algebra (`agc-core/src/math/linalg`)

```
AGC source: Comanche055/CONIC_SUBROUTINES.agc      (GEOM, KEPLERN, GETX, LAMROUT)
            Comanche055/SERVICER207.agc             (CALCGRAV, CALCRVG, NORMLIZE)
            Comanche055/ORBITAL_INTEGRATION.agc     (OBLATE, INTGRATE, DIFEQ0)
            Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc (CALCGA)
Pages:      CONIC 1277-1292; SERVICER 832-839; ORBITAL 325-470
```

Secondary references:
- `docs/architecture.md` §3 — type conventions; `f64` for all nav math
- `agc-core/src/types/vector.rs` — `Vec3 = [f64; 3]`
- `agc-core/src/types/matrix.rs` — `Mat3x3 = [[f64; 3]; 3]`, row-major
- `docs/agc-reference-constants.md` — scale factor table
- AGC Block II Interpretive Language Manual (ibiblio.org/apollo) — VLOAD, DOT, VXV,
  UNIT, NORM, ABVAL, MXV, VXM, VXSC, VSQ, VAD, VSU opcodes

---

## 1. Behavior Summary

The AGC interpretive language provided a set of double-precision vector and matrix
primitives that operated on the pushdown stack (MPAC and pushlist registers 0-39D).
All arithmetic was in scaled ones-complement fixed-point: position vectors were
B-29 (metres × 2^-29), velocity vectors were B-7 (m/cs × 2^-7), unit vectors were
B-1 (dimensionless, half-range). The Rust port eliminates the interpreter entirely
(ADR-001) and re-implements every interpretive routine as a plain `f64` function
operating in SI units. Scale-factor bookkeeping disappears; the programmer uses
physical units throughout.

The functions in `agc_core::math::linalg` are thin, pure, no-alloc wrappers. They
have no side effects, accept no mutable state, and never allocate. They form the
lowest layer of the navigation and guidance stack; every module above them calls
these functions rather than doing its own arithmetic.

### 1.1 AGC Interpretive Opcodes Replaced

| AGC opcode | Description | Rust replacement |
|---|---|---|
| `VLOAD` | Push double-precision vector onto stack | Not needed — Rust uses local variables |
| `DOT` | Dot product of two DP vectors (result scaled B-2 from B-1 inputs) | `dot(a, b)` |
| `VXV` | Cross product of two DP vectors (same scale as inputs) | `cross(a, b)` |
| `UNIT` | Normalise vector to unit length (scale B-1); sets OVFIND if input is zero | `unit(v)` → `Option<Vec3>` |
| `ABVAL` / `NORM` | Absolute value (magnitude) of vector | `norm(v)` |
| `VSQ` | Squared magnitude (scalar); result is B-2 from B-1 inputs | `norm_sq(v)` |
| `VXSC` | Multiply vector by scalar | `scale(v, s)` |
| `VAD` | Add two DP vectors | `add(a, b)` |
| `VSU` / `BVSU` / `BDSU` applied to vectors | Subtract two DP vectors | `sub(a, b)` |
| `MXV` | Matrix times vector | `mxv(m, v)` |
| `VXM` | Vector times matrix (= M^T v for rotation matrices) | `mxv(&transpose(m), v)` |
| `MXM3` | Matrix times matrix (used in KALCMANU_STEERING.agc NEWANGL, line 51) | `mxm(a, b)` |

### 1.2 Usage Examples from AGC Source

**KEPLERN** (`CONIC_SUBROUTINES.agc`, page 1277, label `KEPLERN`):
```
VLOAD* ...    # push MUTABLE vector
STOVL 14D     # store in erasable
    RRECT
UNIT  SSP     # unit(RRECT) → URRECT
    ITERCTR 20D
STODL URRECT
    36D
DOT SL1R      # dot(RRECT, VRECT) scaled
    VRECT
```
`UNIT` is called on the initial position vector `RRECT` to produce the unit
position vector `URRECT`, which is then used throughout the Kepler iteration.

**GEOM** (`CONIC_SUBROUTINES.agc`, page 1291, label `GEOM`):
```
UNIT            # unit(V2VEC) → U2
STODL U2
    36D
STOVL MAGVEC2
UNIT            # unit(R1VEC) → UR1
STORE UR1
DOT SL1         # dot(UR1, U2) → CSTH (cosine of transfer angle)
    U2
    ...
VXV VSL1        # cross(UR1, U2) → orbit-plane normal
    U2
UNIT BOV        # unit(normal); if zero vector (colinear), branch
    COLINEAR
UNITNORM STODL UN
```
This pattern — UNIT, DOT, VXV, UNIT — is the canonical way to compute the
transfer-orbit geometry. The Rust port calls `unit`, `dot`, `cross`, `unit`
in sequence, with the `unit` returning `None` for the colinear (degenerate) case.

**CALCGRAV** (`SERVICER207.agc`, page 832, label `CALCGRAV`):
```
UNIT PUSH       # UNITR = unit(RN)
STORE UNITR
    ...
DOT PUSH        # dot(UNITR, UNITW) — earth-rotation component
    UNITW
    ...
VXSC PDDL       # oblateness perturbation vector
    UNITR
    ...
VAD PUSH        # sum gravity components
    UNITR
```
`CALCGRAV` applies `UNIT` to the position vector and then uses `DOT`, `VXSC`, and
`VAD` to build the gravity acceleration vector. In the Rust port these become direct
`linalg` calls.

**OBLATE** (`ORBITAL_INTEGRATION.agc`, page 325, label `OBLATE`):
```
VLOAD VXV       # cross(504LM, ZUNIT)
    504LM
    ZUNIT
VAD VXM         # add ZUNIT, multiply by MMATRIX
    ZUNIT
    MMATRIX
UNIT            # normalise to UZ
```
The oblateness routine uses `VXV` (cross product) and `VXM` (vector×matrix).

**CALCGA** (`INFLIGHT_ALIGNMENT_ROUTINES.agc`, page 149, label `CALCGA`):
```
VLOAD VXV       # cross(XNB, YSM) → MGA direction
    XNB
    YSM
UNIT PUSH       # unit(MGA)
DOT             # dot(MGA, ZNB) → COS(OG)
    ZNB
DOT             # dot(MGA, YNB) → SIN(OG)
    YNB
```

---

## 2. Rust API

**Module path**: `agc_core::math::linalg`

All functions are `#[inline]`, pure (no side effects), no-alloc, `no_std`-safe.
They operate on the shared types from `agc_core::types`:
- `Vec3 = [f64; 3]` (from `agc-core/src/types/vector.rs`)
- `Mat3x3 = [[f64; 3]; 3]`, row-major (from `agc-core/src/types/matrix.rs`)

```rust
/// Dot product of two 3-vectors.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (DOT SL1, line ~1191);
///             `Comanche055/SERVICER207.agc`, CALCGRAV (DOT PUSH UNITW, line ~752).
/// AGC opcode: `DOT` (double-precision, result at B-2 from B-1 inputs in original).
/// Rust: plain f64 — no scale factor.
pub fn dot(a: &Vec3, b: &Vec3) -> f64

/// Cross product a × b.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (VXV VSL1, line ~1197);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, OBLATE (VLOAD VXV, line ~345).
/// AGC opcode: `VXV`.
/// Returns a Vec3; result is zero if a or b is zero.
pub fn cross(a: &Vec3, b: &Vec3) -> Vec3

/// Euclidean norm (magnitude) of a vector.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM via ABVAL opcode (line ~1213);
///             `Comanche055/SERVICER207.agc`, CALCRVG (ABVAL, line ~478).
/// AGC opcode: `ABVAL` / `NORM` (normalise + return scale count).
/// Uses `libm::sqrt` for `no_std` compatibility.
pub fn norm(v: &Vec3) -> f64

/// Squared Euclidean norm (avoids sqrt).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLERN (VSQ / DOT self, line ~607).
/// AGC opcode: `VSQ`.
pub fn norm_sq(v: &Vec3) -> f64

/// Normalise a vector to unit length.
///
/// Returns `None` if the input vector's norm is less than `f64::EPSILON`
/// (i.e., effectively zero). Never panics, never unwraps.
///
/// The original AGC `UNIT` opcode set the overflow indicator `OVFIND` for a
/// zero-vector input and returned a copy of the zero vector; the Rust port
/// uses `Option` instead to make the degenerate case explicit at compile time.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, GEOM (UNIT BOV COLINEAR, line ~1203);
///             `Comanche055/SERVICER207.agc`, CALCGRAV (UNIT PUSH, line ~745);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, OBLATE (UNIT, line ~351).
/// AGC opcode: `UNIT`.
pub fn unit(v: &Vec3) -> Option<Vec3>

/// Scale a vector by a scalar factor.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCGRAV (VXSC, line ~762);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ+1 (VXSC, line ~661).
/// AGC opcode: `VXSC`.
pub fn scale(v: &Vec3, s: f64) -> Vec3

/// Component-wise vector addition.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCGRAV (VAD PUSH, line ~771);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, GETRPSV (VSR VAD, line ~199).
/// AGC opcode: `VAD`.
pub fn add(a: &Vec3, b: &Vec3) -> Vec3

/// Component-wise vector subtraction (a − b).
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, DIFEQ0 (VSU XCHX,2, line ~186);
///             `Comanche055/ORBITAL_INTEGRATION.agc`, EARSPH (VSU ABVAL, line ~571).
/// AGC opcode: `VSU` / `BVSU`.
pub fn sub(a: &Vec3, b: &Vec3) -> Vec3

/// Matrix-vector multiply: M × v.
///
/// Row-major convention: `result[i] = sum_j m[i][j] * v[j]`.
///
/// AGC source: `Comanche055/SERVICER207.agc`, CALCRVG (VXM VSL1 REFSMMAT, line ~787).
/// AGC opcode: `MXV` (matrix × column vector); `VXM` is treated as M^T × v in
///             the AGC, equivalent to calling this function with `transpose(m)`.
pub fn mxv(m: &Mat3x3, v: &Vec3) -> Vec3

/// Matrix-matrix multiply: A × B (row-major).
///
/// `result[i][j] = sum_k a[i][k] * b[k][j]`.
///
/// AGC source: `Comanche055/KALCMANU_STEERING.agc`, NEWANGL (AXC,1 AXC,2 MIS DEL; CALL MXM3,
///             line ~49-51) — composition of two 3×3 rotation matrices.
/// AGC opcode: `MXM3`.
pub fn mxm(a: &Mat3x3, b: &Mat3x3) -> Mat3x3

/// Transpose a 3×3 matrix.
///
/// `result[i][j] = m[j][i]`.
///
/// AGC source: P51-P53.agc alignment routines; used to convert between SM↔NB
///             coordinate frames (TRG*SMNB / TRG*NBSM are transpose-of-rotation
///             applications in POWERED_FLIGHT_SUBROUTINES.agc lines ~162-228).
pub fn transpose(m: &Mat3x3) -> Mat3x3

/// Elementary rotation matrix about the X axis by angle `theta` (radians).
///
/// ```text
/// Rx(θ) = | 1     0       0    |
///          | 0  cos θ  -sin θ  |
///          | 0  sin θ   cos θ  |
/// ```
///
/// AGC source: built implicitly by CDU-to-DCM conversions in AX*SR*T
///             (POWERED_FLIGHT_SUBROUTINES.agc, pages 1369-1371).
/// Uses `libm::sin` / `libm::cos`.
pub fn rotx(theta: f64) -> Mat3x3

/// Elementary rotation matrix about the Y axis by angle `theta` (radians).
///
/// ```text
/// Ry(θ) = |  cos θ  0  sin θ |
///          |    0    1    0   |
///          | -sin θ  0  cos θ |
/// ```
pub fn roty(theta: f64) -> Mat3x3

/// Elementary rotation matrix about the Z axis by angle `theta` (radians).
///
/// ```text
/// Rz(θ) = | cos θ  -sin θ  0 |
///          | sin θ   cos θ  0 |
///          |   0       0    1 |
/// ```
pub fn rotz(theta: f64) -> Mat3x3
```

---

## 3. Scale Factors

The AGC used fixed-point scaling throughout. The Rust port eliminates all
scale factors — every function operates in SI units:

| AGC scale | Quantity | Rust unit |
|---|---|---|
| B-1 (half-range) | unit vectors | dimensionless `f64`, magnitude = 1.0 |
| B-29 | position vector | metres (`f64`) |
| B-7 | velocity vector | m/s (`f64`) |
| B-2 from B-1 inputs | `DOT` result | dimensionless `f64` |
| not applicable | `cross` result | inherits caller's units |

No conversion constants are needed in `linalg.rs` itself. Conversion between
physical units (m → counts, radians → CDU counts) happens in the HAL layer or
at the types boundary, not here.

---

## 4. Invariants

1. **No heap.** All inputs and outputs are `Vec3 = [f64; 3]` or `Mat3x3 = [[f64; 3]; 3]` — stack-allocated arrays.
2. **No panic.** No function calls `unwrap`, `expect`, `panic!`, or performs integer division. The only fallible path is `unit()` returning `None` for a zero-length vector.
3. **`unit()` never panics.** The threshold for "zero vector" is `norm_sq(v) < f64::EPSILON * f64::EPSILON` (i.e., `norm < f64::EPSILON ≈ 2.2e-16`). This matches the AGC's `UNIT BOV COLINEAR` pattern.
4. **Finite input.** All functions produce defined `f64` outputs for all finite `f64` inputs. Behaviour for NaN or Inf inputs is unspecified (callers are responsible for not generating NaN).
5. **`no_std` compatible.** Only `libm::sqrt`, `libm::sin`, `libm::cos` are used for non-trivial arithmetic; no `std::f64` methods.
6. **Pure functions.** No mutable static state, no side effects, no I/O.
7. **`rotx/roty/rotz` produce orthogonal matrices.** For any finite `theta`, the result satisfies `mxm(r, transpose(r)) ≈ IDENTITY_MAT3` to within `f64` rounding error.

---

## 5. Test Cases

### TC-LINALG-01: Orthogonal basis cross products
Verifies that the standard right-hand basis vectors satisfy e_x × e_y = e_z,
e_y × e_z = e_x, e_z × e_x = e_y, and that e_x × e_x = 0.

```
cross([1,0,0], [0,1,0]) == [0,0,1]   (within 1e-15)
cross([0,1,0], [0,0,1]) == [1,0,0]   (within 1e-15)
cross([0,0,1], [1,0,0]) == [0,1,0]   (within 1e-15)
cross([1,0,0], [1,0,0]) == [0,0,0]   (within 1e-15)
```

### TC-LINALG-02: Dot product of unit vectors
Verifies the dot-product of perpendicular unit vectors is 0 and of parallel
unit vectors is 1.

```
dot([1,0,0], [0,1,0]) == 0.0   (exact)
dot([1,0,0], [1,0,0]) == 1.0   (exact)
dot([0.6, 0.8, 0.0], [0.6, 0.8, 0.0]) == 1.0  (within 1e-15)
```

### TC-LINALG-03: Norm of a 3-4-5 vector
Classic Pythagorean triple confirms the `libm::sqrt` path.

```
norm([3.0, 4.0, 0.0]) == 5.0         (within 1e-12)
norm([0.0, 0.0, 0.0]) == 0.0         (exact)
norm_sq([3.0, 4.0, 0.0]) == 25.0     (exact)
```

### TC-LINALG-04: Zero-vector `unit` returns `None`
Critical safety invariant: the zero vector must not cause a divide-by-zero panic.

```
unit([0.0, 0.0, 0.0]) == None
unit([f64::MIN_POSITIVE * 0.5, 0.0, 0.0]) == None  (below epsilon threshold)
unit([1.0, 0.0, 0.0]) == Some([1.0, 0.0, 0.0])     (within 1e-15)
```

### TC-LINALG-05: AGC interpretive example — GEOM colinear detection
Derived from `CONIC_SUBROUTINES.agc` GEOM routine (page 1291). When R1VEC and
V2VEC are colinear (parallel), the cross product is the zero vector and `unit`
must return `None`, triggering the `COLINEAR` branch in the original code.

```
let r1 = [1.0, 0.0, 0.0];
let u2 = [2.0, 0.0, 0.0];   # parallel to r1
let normal = cross(&r1, &u2);
assert_eq!(normal, [0.0, 0.0, 0.0]);
assert!(unit(&normal).is_none());
```

### TC-LINALG-06: Matrix-vector multiply with identity
```
mxv(IDENTITY_MAT3, [3.0, -1.0, 7.0]) == [3.0, -1.0, 7.0]   (exact)
```

### TC-LINALG-07: Rotation matrix orthogonality
```
let r = rotx(0.3);
let rrt = mxm(r, transpose(r));
// rrt[i][j] == delta(i,j) within 1e-14
```

### TC-LINALG-08: `mxm` composition
```
let rz90 = rotz(PI/2.0);
let result = mxv(rz90, [1.0, 0.0, 0.0]);
// result ≈ [0.0, 1.0, 0.0]  (90° rotation of x-axis gives y-axis)
```

### TC-LINALG-09: `sub` is the left-inverse of `add`
```
let a = [1.0, 2.0, 3.0];
let b = [4.0, -1.0, 0.0];
sub(add(a, b), b) == a   (within 1e-15)
```

---

## 6. agc-sim Impact

- **No new DSKY state.** These are pure computational primitives with no observable
  hardware-side effects.
- **No new `SimLog` events.**
- **Prerequisite for subsequent milestones.** The `navigation`, `guidance`, and
  `control` modules all call `linalg` functions; the `agc-sim` demo scenarios
  (launch, burn, free) will exercise these functions indirectly via those modules.
- **Testing hook.** The `agc-sim` `MissionState` panel's position/velocity display
  will implicitly validate `norm` (displayed as orbital radius magnitude).

---

## 7. Ambiguities

- **`VXM` vs `MXV`**: In the AGC interpretive language, `MXV` = M × v (column vector)
  and `VXM` = v^T × M = (M^T × v)^T. For orthogonal rotation matrices (which are the
  overwhelmingly dominant use in Comanche055) M^T = M^-1, so `VXM` is equivalent to
  applying the inverse rotation. The Rust API exposes only `mxv`; callers that need
  `VXM` semantics call `mxv(&transpose(m), v)`. This is explicit and avoids a
  separate API entry point.
- **`norm` vs `norm_sq` precision**: For the unit-test tolerance on `norm_sq`, exact
  arithmetic suffices (no `sqrt`). For `norm`, the `libm::sqrt` result may differ from
  `core::f64::sqrt` by up to 0.5 ULP; tests use tolerance `1e-12`.
- **`rotx/roty/rotz` are not directly named in Comanche055 source.** They are implied
  by the CDU-to-DCM conversion in `AX*SR*T` (`POWERED_FLIGHT_SUBROUTINES.agc`, pages
  1369-1371). Including them in the spec anticipates their use in `control/attitude.rs`
  and `navigation/state_vector.rs`.
