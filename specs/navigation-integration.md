# Functional Specification: Orbital Integration (`agc-core/src/navigation/integration.rs`)

## AGC Source Reference

```
AGC source: Comanche055/ORBITAL_INTEGRATION.agc
Routines:   DIFEQ+0, DIFEQ+1, DIFEQ+2  (Nystrom integration stages, page 1348-1349)
            DIFEQCOM                     (common increment routine, page 1349)
            DIFEQ0                       (initialise Nystrom state, page 1345)
            TIMESTEP                     (outer integration loop, page 1345)
            RECTIFY                      (reset conic reference, page 1347)
            FBR3                         (advance time and call KEPPREP, page 1336)
            INTGRATE                     (entry from DOSWITCH, page 1346)
            RELOADSV                     (reload state vector on restart, page 1349)
Pages:      1334-1354

AGC source: Comanche055/SERVICER207.agc
Routines:   CALCRVG (predictor-corrector for 2-second cycle, page 835-836)
            CALCGRAV (gravity evaluation, page 835)
Pages:      835-836
```

Constants: `docs/agc-reference-constants.md` (MU_EARTH, J2_EARTH, RE_EARTH, CYCLE_DT).

---

## Behavior Summary

The AGC used two distinct integration schemes:

### Scheme A: SERVICER predictor-corrector (CALCRVG, 2-second cycle)

Used during powered flight and navigation coast to advance RN, VN by 2 seconds.
This is the Störmer-Verlet / leapfrog variant described in `docs/agc-reference-constants.md §Algorithms`.
See `specs/services-average-g.md` for the full SERVICER spec; the integration
module provides the underlying step primitive.

### Scheme B: Nystrom orbital integration (ORBITAL_INTEGRATION.agc)

Used by P20 (rendezvous navigation), P30/P37 targeting, and the W-matrix update.
The AGC implemented a 3-stage Nystrom second-order method (DIFEQ+0, DIFEQ+1,
DIFEQ+2 correspond to the beginning, midpoint, and end of the timestep):

```
Stage 0 (DIFEQ+0):
    PHIV = FV / 8                          # acceleration at beginning, scaled
    h = 0,  YV = r(t),  ZV = v(t)

Stage 1 (DIFEQ+1):
    PSIV = FV/2 + PHIV                     # accumulate midpoint correction
    PHIV = FV/2 + PHIV                     # update
    h += dt/2
    ALPHAV = ZV + h*(FV + ZV)             # predicted position at midpoint

Stage 2 (DIFEQ+2):
    YV += H*(2/3*PHIV + ZV)              # update position
    ZV += FV/8 + PSIV                    # update velocity
    h = DT (full step complete)
```

The variable `H` (time step) goes through 0, dt/2, dt across the three stages.
The vector `PHIV` accumulates force contributions; `PSIV` is an intermediate
corrector vector. `FV` is the total acceleration (gravity + perturbations) at
the current `ALPHAV` position.

**For Milestone 2**: The full Nystrom algorithm can be faithfully implemented
from the above. However, because the interpretive opcodes (VXSC, VSR, etc.) add
complexity to decode precisely, the developer MAY substitute **RK4** with a
`// TODO: replace with AGC Nystrom predictor-corrector to match ORBITAL_INTEGRATION.agc`
comment. RK4 achieves the same order-4 truncation error (the Nystrom is order 3
in position, RK4 is order 4 in both) with a cleaner implementation.

**Recommendation**: Use RK4 for Milestone 2 (per `docs/agc-reference-constants.md
§Algorithms: "Rust substitutes a 4th-order Runge-Kutta (RK4) for orbital
propagation"`). Mark with TODO. Nystrom faithful port is a Milestone 3 item.

---

## Public API

Module path: `agc_core::navigation::integration`

### Design decision: function vs struct

The `propagate` free-function form is preferred over a struct because:
- The time step `dt` varies (SERVICER uses 2 s; long-arc coast uses larger steps).
- The gravity function is injected by the caller (enabling testing with a simple
  `|r, t| earth_gravity(r)` closure without Moon perturbations).
- No state needs to persist between calls beyond the `StateVector` itself.
- A struct with `dt` baked in would require reconstruction when `dt` changes
  (coast vs thrust); the function form avoids that.

### `propagate`

```rust
/// Advance a state vector by one timestep `dt` seconds.
///
/// Algorithm (Milestone 2): 4th-order Runge-Kutta applied to the
/// second-order ODE  r'' = grav(r, t).
///
/// RK4 stages:
///   k1_r = v,                k1_v = grav(r, t)
///   k2_r = v + dt/2*k1_v,   k2_v = grav(r + dt/2*k1_r, t + dt/2)
///   k3_r = v + dt/2*k2_v,   k3_v = grav(r + dt/2*k2_r, t + dt/2)
///   k4_r = v + dt*k3_v,     k4_v = grav(r + dt*k3_r,   t + dt)
///   r_new = r + dt/6*(k1_r + 2*k2_r + 2*k3_r + k4_r)
///   v_new = v + dt/6*(k1_v + 2*k2_v + 2*k3_v + k4_v)
///   t_new = t + dt (rounded to nearest centisecond)
///
/// // TODO: replace with AGC Nystrom predictor-corrector (DIFEQ+0/+1/+2)
/// //        per Comanche055/ORBITAL_INTEGRATION.agc pages 1348-1349.
///
/// The gravity function `grav` receives the ECI position vector and the
/// time at that substep. Callers typically pass `navigation::gravity::total_gravity`
/// curried with a `PrimaryBody`:
///   `|r, t| total_gravity(r, t, PrimaryBody::Earth)`
///
/// Preconditions (callers must ensure):
///   - `dt` is finite and positive.
///   - `grav` never panics for any finite position.
///   - `|state.position()| > R_MIN_GUARD` (1.0 m) before calling.
///
/// The `gdt_over_2` field of the returned `StateVector` is updated to
/// `grav(r_new, t_new) * dt / 2` for use by SERVICER's predictor term.
///
/// AGC source: Comanche055/ORBITAL_INTEGRATION.agc, DIFEQ+0/+1/+2 (pages 1348-1349).
/// Substitution rationale: docs/agc-reference-constants.md §Algorithms §RK4.
///
/// Units: dt in seconds, positions in metres, velocities in m/s.
pub fn propagate(
    state: &StateVector,
    dt: f64,
    grav: &dyn Fn(&Vec3, Met) -> Vec3,
) -> StateVector;
```

### `nystrom_step` (optional, for future faithful port)

```rust
/// AGC Nystrom second-order predictor-corrector step.
///
/// Implements exactly DIFEQ+0, DIFEQ+1, DIFEQ+2 from
/// Comanche055/ORBITAL_INTEGRATION.agc pages 1348-1349.
///
/// This function is marked `#[cfg(feature = "nystrom")]` and is NOT
/// the default integration method in Milestone 2. It exists as a
/// reference for Milestone 3 validation.
///
/// Nystrom stages (from DIFEQCOM / DIFEQ+0/+1/+2):
///   Stage 0: PHIV = FV * (1/8);  H = 0
///   Stage 1: PSIV = PHIV + FV*(1/2); PHIV += FV*(1/2); H = dt/2
///   Stage 2: YV += H*(2/3*PHIV + ZV); ZV += PSIV + FV/8; H = dt
///   where FV = grav(ALPHAV, TET + H) at each stage
///   and ALPHAV advances via DIFEQCOM as H increments.
///
/// AGC source: Comanche055/ORBITAL_INTEGRATION.agc,
///   DIFEQ+0 (page 1348), DIFEQ+1 (page 1348), DIFEQ+2 (page 1349),
///   DIFEQCOM (page 1349).
#[cfg(feature = "nystrom")]
pub fn nystrom_step(
    state: &StateVector,
    dt: f64,
    grav: &dyn Fn(&Vec3, Met) -> Vec3,
) -> StateVector;
```

---

## Invariants

1. `propagate` is a pure function (no side effects, no static state).
2. The returned `StateVector.time()` equals `state.time() + dt` rounded to the
   nearest centisecond.
3. For a circular orbit with no perturbations, orbital energy
   `E = 0.5 * |v|^2 − MU_EARTH / |r|` is conserved. After one full orbital
   period (≈ 5400 s for LEO), `|ΔE / E| < 1e-6` (see Test 3 and `docs/testing.md §5`).
4. No `unwrap`, `expect`, or panics. If `dt <= 0.0` or is NaN, the behaviour is
   undefined and the developer must add a guard (`alarm::raise` or return the
   input state unchanged).
5. No heap allocation; all intermediate values (k1..k4) are stack `Vec3`.

---

## DSKY / agc-sim Impact

- The `agc-sim/src/physics.rs` reference trajectory uses `propagate` with
  `total_gravity(r, t, PrimaryBody::Earth)` at a 0.1 s step to produce a
  smooth visual trajectory.
- The SERVICER `cycle` method calls `propagate` with `dt = CYCLE_DT = 2.0` during
  coast phases (no PIPA thrust); during thrust the SERVICER uses its own
  CALCRVG predictor-corrector (see `specs/services-average-g.md`).
- No new DSKY lights required.

---

## Test Cases

### Test 1 — Energy conservation over one LEO orbit

```
// 200 km circular LEO
r = [RE_EARTH + 200_000.0, 0.0, 0.0]    // 6 573 338 m
v_circ = sqrt(MU_EARTH / |r|) ≈ 7784.26 m/s
v = [0.0, v_circ, 0.0]
state0 = StateVector::new(r, v, Met(0))

T_orbit = 2π * |r|^(3/2) / sqrt(MU_EARTH) ≈ 5308 s
dt = 10.0  // 10-second steps (RK4 stable at this step size for LEO)
N = T_orbit / dt as usize

grav = |r, _t| earth_gravity(r)  // point mass only, no J2 for this test
state_final = propagate N times from state0

E0 = 0.5 * v_circ^2 - MU_EARTH / |r|    // specific orbital energy, m²/s²
E1 = 0.5 * state_final.speed()^2 - MU_EARTH / state_final.radius()

// Tolerance from docs/testing.md §5 (SERVICER cycle: pos < 1 m, vel < 0.01 m/s)
// For a single full orbit with RK4 at dt=10s, expect energy conservation to 1e-6
assert!((E1 - E0).abs() / E0.abs() < 1e-6)
```

### Test 2 — Position continuity (dt → 0 limit)

```
// A very small step should change position by approximately v * dt
state = StateVector::new([6_578_000.0, 0.0, 0.0], [0.0, 7784.0, 0.0], Met(0))
dt_small = 1e-6   // 1 microsecond
state2 = propagate(&state, dt_small, &|r, _| earth_gravity(r))
expected_dy = 7784.0 * dt_small   // ≈ 7.784e-3 m
assert!((state2.position()[1] - expected_dy).abs() < 1e-10)
```

### Test 3 — Gravity injection (mock gravity test)

```
// With zero gravity, propagate should give straight-line motion
state = StateVector::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], Met(0))
state2 = propagate(&state, 5.0, &|_r, _t| [0.0, 0.0, 0.0])
assert!((state2.position()[0] - 5.0).abs() < 1e-12)
assert!((state2.velocity()[0] - 1.0).abs() < 1e-12)
assert_eq!(state2.time(), Met(500))  // 5.0 s = 500 cs
```

---

## Notes and Ambiguities

1. **Nystrom vs RK4**: The AGC DIFEQ+0/+1/+2 opcodes operate in the interpretive
   language with fractional fixed-point arithmetic. The coefficient `DP2/3`
   (`2DEC .6666666667`) and `VSR3` (right-shift 3, divide by 8) suggest a
   Störmer/Nystrom variant. The exact correspondence to the classical Nystrom
   formulas requires careful decode of the scale shifts. This is deferred to M3.
   RK4 is the accepted M2 substitution per `docs/agc-reference-constants.md`.

2. **`gdt_over_2` update**: The returned state's `gdt_over_2` field is set to
   `grav(r_new, t_new) * dt / 2`. This is used by the SERVICER predictor term
   (`RN1 = RN + (VN + DV/2 + GDT/2_old) * dt`). The integrator must update it
   so SERVICER and the propagator share consistent state.

3. **Step size stability**: RK4 is stable for orbital mechanics at dt ≤ ~60 s
   for LEO. The SERVICER uses dt = 2 s (well within stability). Longer steps
   (coast propagation, up to 300 s) must be validated in the M3 energy
   conservation tests.

4. **Rectify (RECTIFY subroutine)**: The AGC periodically "rectifies" the
   integration by re-establishing the conic reference (RRECT, VRECT). This
   prevents the Encke deviation `TDELTAV` from growing. In the RK4 port, since
   there is no Encke decomposition, rectification is not needed. The comment
   `// TODO: implement RECTIFY if Encke method is adopted` should appear in the code.
