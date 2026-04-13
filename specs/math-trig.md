# Functional Specification: Trigonometric Wrappers (`agc-core/src/math/trig`)

```
AGC source: Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc  (ARCTRIG, CALCGA; pages 1357-1364)
            Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc (ARCTAN, LAT-LONG, LALOTORV; pages 1236-1242)
            Comanche055/GROUND_TRACKING_DETERMINATION_PROGRAM.agc (DDV ASIN; page ~459)
            Comanche055/P30-P37.agc                      (SL1 ARCCOS; page ~428)
            Comanche055/P51-P53.agc                      (ASIN DAD; page ~707)
            Comanche055/ERASABLE_ASSIGNMENTS.agc          (ESCAPE/ESCAPE2 aliases; lines 1560, 1565)
```

Secondary references:
- AGC Block II Interpretive Language Manual (ibiblio.org/apollo) — SIN, COS, ASIN, ACOS opcodes
- `docs/architecture.md` §3 — `f64` for all navigation math; `libm` for `no_std` math
- `AGENTS.md` — no `std::f64`, must use `libm::*` under `no_std`

Note on trig source: Comanche055 does not contain a standalone SINCOS assembly file.
The SIN, COS, ASIN, ACOS, ATAN opcodes are part of the AGC Block II interpretive
language and are implemented in hardware microcode / interpreter ROM, not in the
flight software itself. Usage of these opcodes is documented throughout the Comanche055
source; the interpretive opcode definitions are in the Block II AGC Interpreter manual,
not in a Comanche055 source file. This is noted in the Ambiguities section.

---

## 1. Behavior Summary

The AGC interpretive language provided `SIN`, `COS`, `ASIN`, and `ACOS` opcodes that
operated on double-precision fixed-point values scaled at B-1 (range ±1.0 in half-
revolution units, where 1.0 = 2π radians = one full revolution ÷ 2). The AGC's
internal angle representation was fractional: 0.5 = 180°, 1.0 = 360° (i.e.,
1 unit = 2π radians in the AGC's full-range scaling, or π radians in its half-range
scaling for trig arguments).

The critical issue with ASIN and ACOS in the AGC was **domain overflow**: the
fixed-point product of two B-1 unit vectors computed via DOT could produce a result
slightly greater than +1.0 or slightly less than -1.0 due to rounding in the
ones-complement multiplication hardware. Feeding a value like 1.000000001 to the
hardware ASIN would cause an overflow exception (set OVFIND). The Comanche055
software dealt with this in two ways:

1. The ARCTAN subroutine (`LATITUDE_LONGITUDE_SUBROUTINES.agc`, lines 209-236)
   guards against the 0/0 case by checking `BZE` before calling `ASIN`.
2. The ARCTRIG routine (`INFLIGHT_ALIGNMENT_ROUTINES.agc`, lines 103-133) uses a
   branch on magnitude to choose between ACOS (for angles near ±90°) and ASIN (for
   angles near 0° or 180°) to avoid the near-boundary overflow.
3. The `ESCAPE` / `ESCAPE2` erasable aliases (`ERASABLE_ASSIGNMENTS.agc`, lines
   1560, 1565) are switch words used by the interpreter's arcsin/arccos routines
   to select alternate exit paths on overflow — confirming that the AGC interpreter
   itself had logic to handle out-of-range inputs gracefully.

The Rust port must replicate this domain protection explicitly. `f64` arithmetic
in the Rust port does not overflow for inputs slightly outside [-1, 1] (it returns
NaN from `libm::asin` instead), so the clamping must be applied by the Rust wrapper
before calling `libm::asin` or `libm::acos`. This preserves behavior parity: where
the AGC would have saturated to ±π/2 or ±π via its overflow path, the Rust port
explicitly clamps the argument.

### 1.1 ARCTRIG Usage Pattern

`INFLIGHT_ALIGNMENT_ROUTINES.agc`, ARCTRIG routine (pages 1357-1358):

```
ARCTRIG   DLOAD ABS
              SINTH             # |sin(θ)|
          DSU BMN
              QTSN45            # threshold: sin(45°)/4
              TRIG1             # branch if (-45,45) or (135,-135)

          DLOAD SL1
              COSTH
          ACOS SIGN             # ARCCOS(COS θ) with sign from SIN
              SINTH
          STORE THETA           # X = ARCCOS(COS) WITH SIGN(SIN)
          RVQ

TRIG1     DLOAD SL1
              SINTH
          ASIN                  # ARCSIN(SIN θ) for small angles
          STODL THETA           # X = ARCSIN(SIN) WITH SIGN(SIN)
```

This routine conditionally chooses ARCCOS or ARCSIN to compute the quadrant-correct
angle, explicitly avoiding the region near ±90° where ARCCOS is numerically poor and
the region near 0°/180° where ARCSIN is numerically poor. In the Rust port,
`asin_clamped` and `acos_clamped` are the building blocks this pattern uses.

### 1.2 ARCTAN Usage Pattern

`LATITUDE_LONGITUDE_SUBROUTINES.agc`, ARCTAN routine (page 1240, label `ARCTAN`):

```
ARCTAN   BOV              # clear overflow before start
             CLROVFLW
CLROVFLW DLOAD DSQ
             SINTH
         PDDL DSQ
             COSTH
         DAD              # SIN²+COS²
         BZE SQRT         # if zero, go to ARCTANXX (θ=0)
             ARCTANXX
         BDDV BOV         # SINTH/(SINTH²+COSTH²)
             SINTH
             ATAN=90       # overflow: angle ≈ ±90°, branch
         SR1 ASIN          # ASIN(normalized SIN) for main path
         STORE THETA
```

The `ASIN` call here receives `SINTH / sqrt(SIN²+COS²)` — a value that is
mathematically guaranteed to lie in [-1, 1] — but the intermediate computation
via fixed-point division could round slightly outside due to the ones-complement
representation. The `ESCAPE` / `ESCAPE2` flags in `ERASABLE_ASSIGNMENTS.agc`
(lines 1560, 1565) handle this at the interpreter level. The Rust wrapper clamp
achieves the same effect.

### 1.3 ASIN Usage in Navigation Programs

- `GROUND_TRACKING_DETERMINATION_PROGRAM.agc`, page 459: `DDV ASIN # U(R).U(V)` —
  computes flight-path angle as arcsin of the dot product of unit-range and unit-
  velocity vectors. The dot product of two unit vectors is mathematically in [-1,1]
  but can overshoot by one ULP.
- `P30-P37.agc`, page 428: `SL1 ARCCOS` — computes the range angle between recovery
  target and current ground track using arccos of a dot product.
- `P51-P53.agc`, page 707: `ASIN DAD 5DEGREES` — computes an arc sine then adds 5°
  to it for horizon clearance.

---

## 2. Rust API

**Module path**: `agc_core::math::trig`

All functions are `#[inline]`, pure (no side effects), no-alloc, `no_std`-safe.
All floating-point operations use `libm::*` — never `core::f64` methods — so that
the identical binary is produced for both host tests and the `thumbv7em-none-eabihf`
embedded target.

```rust
/// Sine of `theta` (radians).
///
/// Thin wrapper around `libm::sin`. Provided for consistency (all nav/guidance
/// code uses `math::trig::sin` rather than calling `libm` directly) and to
/// enable future instrumentation or override in simulation.
///
/// AGC opcode: `SIN` (B-1 input → B-1 output in AGC; Rust: radians → dimensionless).
/// AGC usage: `LATITUDE_LONGITUDE_SUBROUTINES.agc` LALOTORV (DLOAD SIN LAT, line ~127).
pub fn sin(theta: f64) -> f64

/// Cosine of `theta` (radians).
///
/// Thin wrapper around `libm::cos`.
///
/// AGC opcode: `COS` (B-1 input → B-1 output in AGC).
/// AGC usage: `LATITUDE_LONGITUDE_SUBROUTINES.agc` LALOTORV (COS PDDL LONG, line ~132);
///            `LATITUDE_LONGITUDE_SUBROUTINES.agc` CALLRTRP (BOFF COS 0 → COS(0)=1,
///            line ~68) — uses COS(0) to produce a non-zero value in MPAC.
pub fn cos(theta: f64) -> f64

/// Tangent of `theta` (radians).
///
/// Thin wrapper around `libm::tan`. Near ±π/2 the result is large but finite
/// (not NaN) for all representable `f64` values of `theta`.
///
/// AGC opcode: Not a direct interpretive opcode; synthesised as SIN/COS in AGC code.
/// Provided in the Rust API for callers in `control/` and `guidance/` that need
/// tangent directly (e.g., the SMCDURES rate matrix in POWERED_FLIGHT_SUBROUTINES.agc).
pub fn tan(theta: f64) -> f64

/// Arcsine with domain clamping.
///
/// Clamps `x` to `[-1.0, 1.0]` before calling `libm::asin`, then returns the
/// result in radians in the range `[-π/2, π/2]`.
///
/// The clamp is mandatory: AGC fixed-point dot products of unit vectors can
/// produce values slightly outside [-1, 1] due to ones-complement rounding.
/// In the Rust port, `f64` dot products of normalised vectors can similarly
/// overshoot by one ULP (e.g., 1.0000000000000002). Without clamping,
/// `libm::asin` returns NaN; with clamping it returns π/2 exactly.
///
/// AGC source: `LATITUDE_LONGITUDE_SUBROUTINES.agc` ARCTAN (SR1 ASIN, line ~221);
///             `INFLIGHT_ALIGNMENT_ROUTINES.agc` ARCTRIG / TRIG1 (ASIN, line ~118);
///             `GROUND_TRACKING_DETERMINATION_PROGRAM.agc` (DDV ASIN, line ~162);
///             `P51-P53.agc` (ASIN DAD, line ~707).
///             `ERASABLE_ASSIGNMENTS.agc` ESCAPE / ESCAPE2 (lines 1560, 1565) —
///             erasable switch words that handled domain overflow in the AGC interpreter.
/// AGC opcode: `ASIN` (with ESCAPE/ESCAPE2 overflow protection in interpreter).
pub fn asin_clamped(x: f64) -> f64

/// Arccosine with domain clamping.
///
/// Clamps `x` to `[-1.0, 1.0]` before calling `libm::acos`, then returns the
/// result in radians in the range `[0, π]`.
///
/// Same domain-protection rationale as `asin_clamped`.
///
/// AGC source: `INFLIGHT_ALIGNMENT_ROUTINES.agc` ARCTRIG (ACOS SIGN SINTH, line ~111);
///             `P30-P37.agc` (SL1 ARCCOS, line ~428);
///             `P51-P53.agc` comment (ARCCOS(OS1-OS2), line ~1092).
/// AGC opcode: `ACOS` (with ESCAPE/ESCAPE2 overflow protection in interpreter).
pub fn acos_clamped(x: f64) -> f64

/// Two-argument arctangent: `atan2(y, x)` in radians, range `(-π, π]`.
///
/// Thin wrapper around `libm::atan2`. Handles all quadrants correctly including
/// the x=0 case (returns ±π/2). Returns 0.0 for the degenerate `atan2(0, 0)` case
/// (matching POSIX/IEEE 754 behaviour and the AGC ARCTANXX zero-result path).
///
/// AGC source: `LATITUDE_LONGITUDE_SUBROUTINES.agc` ARCTAN routine (lines 209-236)
///             implements the equivalent of atan2(SINTH, COSTH), using ASIN with a
///             quadrant correction for the negative-cosine half-plane. The Rust
///             `atan2` wrapper replaces the entire ARCTAN subroutine.
/// AGC opcode: No single opcode; the AGC synthesised atan2 from ASIN + quadrant branches.
pub fn atan2(y: f64, x: f64) -> f64
```

---

## 3. Scale Factors

| AGC angle representation | Rust representation |
|---|---|
| B-1 fractional revolutions (range ±0.5, i.e., ±π radians = ±180°) | Radians (`f64`) |
| B-0 fractional revolutions (range ±1.0, i.e., ±2π radians = ±360°) | Radians (`f64`) |
| CDU counts (u16, 65536 counts/revolution) | Handled by `CduAngle::to_radians()` in `types/angle.rs` |

No conversion is needed inside `math/trig.rs`. All public functions accept and
return radians. Conversion from AGC's fractional-revolution convention to radians
happens at the call sites that read CDU registers (in `hal/imu.rs` via `CduAngle`)
or in the specific routines that deal with the fractional representation
(e.g., `LATITUDE_LONGITUDE_SUBROUTINES.agc` stores LAT/LONG in B-0 revolutions;
the Rust equivalents will convert at their own boundaries).

---

## 4. Invariants

1. **No heap.** All functions are pure `f64 → f64` or `(f64, f64) → f64`. No allocation.
2. **No panic.** No `unwrap`, `expect`, or integer division. All `f64` inputs produce
   defined `f64` outputs (NaN propagates silently for NaN inputs; callers are responsible).
3. **`libm` only.** All floating-point transcendentals use `libm::sin`, `libm::cos`,
   `libm::asin`, `libm::acos`, `libm::atan2`, `libm::tan`. No `core::f64::consts::*`
   methods that internally differ between host and embedded targets.
4. **Domain clamping is unconditional.** `asin_clamped` and `acos_clamped` always clamp,
   even when the input is exactly ±1.0. The clamp is cheap (two comparisons) and is
   never wrong.
5. **`atan2(0.0, 0.0)` returns `0.0`.** This matches the AGC ARCTAN `ARCTANXX` branch
   (store THETA = 0 when SIN²+COS² = 0). `libm::atan2(0.0, 0.0)` already returns 0.0
   per IEEE 754; this is stated explicitly as an invariant so callers can rely on it.
6. **`no_std` compatible.** No `extern crate std`; the `libm` crate is already a
   dependency of `agc-core` (stated in the task brief).
7. **Pure functions.** No mutable state, no side effects.

---

## 5. Test Cases

### TC-TRIG-01: Basic sin/cos values
```
sin(0.0)        == 0.0           (exact)
sin(PI/2.0)     == 1.0           (within 1e-15)
sin(PI)         ~= 0.0           (within 1e-15; libm may give ε ≈ 1.2e-16)
sin(3.0*PI/2.0) == -1.0          (within 1e-15)
cos(0.0)        == 1.0           (exact)
cos(PI/2.0)     ~= 0.0           (within 1e-15)
cos(PI)         == -1.0          (within 1e-15)
```

### TC-TRIG-02: Pythagorean identity
For any angle, `sin²(θ) + cos²(θ) == 1.0` to within `f64` rounding:
```
sin(1.23456)^2 + cos(1.23456)^2 == 1.0   (within 2e-15)
sin(5.678)^2   + cos(5.678)^2   == 1.0   (within 2e-15)
```

### TC-TRIG-03: `asin_clamped` with exact ±1.0 inputs
```
asin_clamped(1.0)   == PI/2.0    (within 1e-15)
asin_clamped(-1.0)  == -PI/2.0   (within 1e-15)
asin_clamped(0.0)   == 0.0       (exact)
```

### TC-TRIG-04: `asin_clamped` domain protection — overshoot by 1e-12
This is the critical test. A dot product of two unit vectors computed via `f64`
arithmetic can produce 1.0 + ε where ε ~ 1e-15 to 1e-12 due to accumulated
rounding. Without clamping, `libm::asin(1.0 + 1e-12)` returns NaN. With clamping
it must return π/2.

```
asin_clamped(1.0 + 1e-12)   == PI/2.0   (within 1e-15)
asin_clamped(-1.0 - 1e-12)  == -PI/2.0  (within 1e-15)
acos_clamped(1.0 + 1e-12)   == 0.0      (within 1e-15)
acos_clamped(-1.0 - 1e-12)  == PI       (within 1e-15)
```

AGC source anchor: matches the ESCAPE/ESCAPE2 overflow-handling in the Block II
interpreter (ERASABLE_ASSIGNMENTS.agc lines 1560, 1565). The AGC interpreter
produced an overflow when the ASIN input exceeded ±1.0 (in half-range) and branched
via ESCAPE to a saturation path; the Rust clamp achieves the same outcome.

### TC-TRIG-05: `atan2` quadrant correctness
```
atan2(0.0,  1.0)  ==  0.0         (exact)
atan2(1.0,  0.0)  ==  PI/2.0      (within 1e-15)
atan2(0.0, -1.0)  ==  PI          (within 1e-15)
atan2(-1.0, 0.0)  == -PI/2.0      (within 1e-15)
atan2(0.0,  0.0)  ==  0.0         (exact, per AGC ARCTANXX branch)
```

### TC-TRIG-06: AGC-derived example — flight-path angle
Derived from `GROUND_TRACKING_DETERMINATION_PROGRAM.agc` (page 459):
`DDV ASIN # U(R).U(V)` computes `asin(dot(unit_r, unit_v))`.

For a circular orbit, the velocity is perpendicular to the radius, so
`dot(unit_r, unit_v) = 0` and the flight-path angle should be 0.

```
let unit_r = [1.0_f64, 0.0, 0.0];
let unit_v = [0.0_f64, 1.0, 0.0];   # perpendicular = circular orbit
let gamma = asin_clamped(dot(&unit_r, &unit_v));
assert!((gamma - 0.0).abs() < 1e-15, "gamma = {gamma}");
```

For a radial (straight-up) trajectory, `dot = 1.0`:
```
let unit_r2 = [1.0_f64, 0.0, 0.0];
let unit_v2 = [1.0_f64, 0.0, 0.0];  # same direction = radial ascent
let gamma2 = asin_clamped(dot(&unit_r2, &unit_v2));
assert!((gamma2 - PI/2.0).abs() < 1e-15, "gamma2 = {gamma2}");
```

### TC-TRIG-07: `tan` and its relation to sin/cos
```
tan(PI/4.0)  == 1.0          (within 1e-14)
tan(0.0)     == 0.0          (exact)
tan(-PI/4.0) == -1.0         (within 1e-14)
```

---

## 6. agc-sim Impact

- **No new DSKY state.** These are pure computational primitives.
- **No new `SimLog` events.**
- **Prerequisite.** The `navigation`, `guidance`, and `control` modules call these
  wrappers. The latitude/longitude display (noun N43 or similar) in the `agc-sim`
  TUI will exercise `atan2` indirectly via `LATITUDE_LONGITUDE_SUBROUTINES.agc`
  port.
- **Test coverage.** The overshoot test (TC-TRIG-04) is particularly important for
  the `agc-sim` burn scenario: after a 50 m/s prograde burn, the orbit is slightly
  elliptic and the flight-path angle dot product may be exactly 1.0 ± ULP at
  apogee/perigee, where the clamp is exercised.

---

## 7. Ambiguities

- **No Comanche055 SINCOS source file.** The SIN, COS, ASIN, and ACOS opcodes are
  part of the Block II AGC interpreter ROM, not part of the Comanche055 flight
  software. There is no `SINCOS.agc` in the Comanche055 source. All references in
  this spec cite usage sites (routines that call the opcode) rather than the
  implementation site (which does not exist in this source set). The Block II AGC
  Interpreter manual (available at ibiblio.org/apollo, "AGC Assembly Language Manual")
  documents the opcodes formally. This is an intentional deviation from the spec
  format requirement to "cite the implementation file" — the implementation is in
  interpreter microcode, not assembler.
- **`ESCAPE` / `ESCAPE2` exact mechanism.** The `ERASABLE_ASSIGNMENTS.agc` aliases
  at lines 1560 and 1565 confirm that overflow handling existed in the ASIN/ACOS
  interpreter routines, but the exact saturation value (whether it clamped to the
  nearest valid result or triggered a restart) is not visible in the available
  Comanche055 source. The Rust clamp conservatively saturates to ±π/2 (for asin)
  and 0 or π (for acos), which matches the mathematical limit and is the safest choice.
- **`tan` opcode.** The AGC interpretive language manual lists SIN, COS, ASIN, ACOS
  but not a TAN opcode. The AGC code that needed tangent synthesised it from SIN/COS.
  The Rust `tan` wrapper is an addition beyond the AGC opcode set, provided for
  convenience in Rust callers. It introduces no fidelity risk because no Rust code
  is required to match an AGC TAN opcode.
- **`KALCMANU_STEERING.agc` trig.** The KALCMANU file uses no interpretive trig
  opcodes directly — it calls AX*SR*T (POWERED_FLIGHT_SUBROUTINES.agc) which uses
  pre-computed `SINCDU`/`COSCDU` tables prepared by `CDUTRIG`. The underlying sin/cos
  computation for CDU angles is therefore in `CDUTRIG` / `CDUTRIGS` (POWERED_FLIGHT_
  SUBROUTINES.agc, pages 1365-1367), not in KALCMANU itself. This is documented here
  to prevent a future analyst from searching KALCMANU for direct trig calls.
