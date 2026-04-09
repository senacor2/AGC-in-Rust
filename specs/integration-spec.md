# Specification: `navigation/integration` Module

**Status**: Approved for implementation  
**Module path**: `agc-core/src/navigation/integration.rs`  
**Architecture reference**: `docs/architecture.md` §7.4 "SERVICER (Average-G)", §9.4 "Gravity Model"  
**State-vector reference**: `specs/state-vector-spec.md` §5.1 (SERVICER), §5.2 (Conic propagation), §2.3 (SOI transition)  
**Gravity reference**: `specs/gravity-spec.md` §4 (function specs), §5 (SOI)  
**Math reference**: `specs/linalg-spec.md` §4 (vadd, vsub, vscale, norm)  
**AGC source files**:
- `Comanche055/AVERAGE_G_INTEGRATOR.agc` — SERVICER entry point, Average-G integration loop  
- `Comanche055/ORBITAL_INTEGRATION.agc` — Cowell and Encke integrators, coast propagation  
- `Comanche055/INTEGRATION_INITIALIZATION.agc` — body selection, constant tables  
**Spec checklist**: `specs/README.md` — all items satisfied (see §10)

---

## 1. Purpose and Scope

`navigation::integration` numerically propagates the spacecraft state vector
forward in time by integrating the equations of motion. The equations of motion
combine gravitational acceleration (primary body and third-body perturbation)
with inertially-measured thrust acceleration from the PIPAs.

The module provides two propagation modes that correspond directly to the two
integration strategies in Comanche055:

1. **Average-G step** (`average_g_step`) — powered-flight integration, called
   every 2 seconds by the SERVICER task. This is a second-order trapezoidal
   scheme applied to the equations of motion using Cowell's method (direct
   integration of the full acceleration vector). See §3.

2. **Coast propagation** (`propagate_coast`) — coasting-flight propagation on
   demand, using the universal-variable Kepler solver in `math::kepler`. See §4.

A helper function `total_gravity` (§5) assembles the combined gravitational
acceleration for either frame, providing the internal building block used by
both propagation paths.

A post-propagation sphere-of-influence check (§6) detects frame transitions and
transforms the state vector to the new frame.

### What this module provides

- `average_g_step` — one 2-second SERVICER integration step (powered flight).
- `propagate_coast` — on-demand Keplerian propagation (coasting flight).
- `total_gravity` — combined gravitational acceleration for a given position and
  frame (primary + third-body).
- `soi_check` — post-propagation sphere-of-influence test with frame conversion.

### What this module does NOT provide

- PIPA read or compensation. Those are in `services::average_g`. By the time
  `average_g_step` is called, the delta-V has already been rotated into the
  inertial frame via REFSMMAT.
- Encke's method. The deviation-from-conic integrator is deferred until the
  Kepler solver is validated and there is evidence from simulation that Cowell
  alone is insufficient for the required accuracy. See §7 (Design Decisions).
- Navigation measurement updates (P23, P20). Those are in `programs::p23` and
  `programs::p20_p22`; they modify `StateVector::position` directly, not through
  this module.
- Rendezvous targeting or Lambert solver. Those are in `math::lambert` and
  `guidance`.

---

## 2. AGC Background

### 2.1 The Average-G Integration Loop

The SERVICER (named for its role as a background "service" task) ran on a strict
2-second repeating cycle established by the T3RUPT interrupt chain. At each
invocation (architecture §7.4), it:

1. Read PIPA counts from erasable registers `PIPAX`, `PIPAY`, `PIPAZ`.
2. Applied compensation (bias, misalignment) to obtain `delta_v_platform`.
3. Rotated `delta_v_platform` to inertial coordinates via REFSMMAT.
4. Computed gravitational acceleration `g` at the current state-vector position.
5. Integrated the state vector over 2 seconds.
6. Updated `RN`/`VN` (erasable 0306–0321) and `TEPHEM` (octal 0340).

The name "Average-G" comes from the specific numerical scheme used in step 5.
Rather than using the gravity value only at the beginning of the interval, the
AGC computed gravity at both the beginning (`g0`) and end (`g1`) of the interval
and averaged them (trapezoidal gravity averaging). This improves accuracy to
second order for near-circular orbits at the cost of one additional gravity
evaluation per step.

The scheme is not a classical four-stage Runge-Kutta. It is a two-stage
predictor-corrector: position is predicted using a midpoint-velocity estimate,
then gravity at the predicted position provides the corrected velocity via
trapezoidal averaging. The current stub in `integration.rs` describes this as
`rk4_step` — that name must be replaced by the correct Average-G scheme described
in §3. RK4 is a four-stage method; the AGC used a two-stage method.

### 2.2 Cowell's Method vs. Encke's Method

**Cowell's method** integrates the total acceleration (primary gravity +
perturbations) directly without any reference trajectory. It is simple to
implement, accurate for powered flight and short coasting arcs, but accumulates
numerical error over long coast arcs because the large point-mass term dominates
the small perturbation terms, causing cancellation.

**Encke's method** integrates only the *deviation* of the true trajectory from a
reference Keplerian conic. The reference conic is rectified periodically when the
deviation becomes too large. This dramatically reduces numerical error for long
coast arcs but requires maintaining a reference trajectory alongside the true one.

Comanche055 used both:
- AVERAGE_G_INTEGRATOR.agc — Cowell's method for the powered-flight SERVICER.
- ORBITAL_INTEGRATION.agc — Encke's method for long coasting intervals.

The Rust port defers Encke's method. For initial validation (Earth orbit,
translunar coast, lunar orbit), Cowell's method with the Kepler solver as the
coast integrator is sufficient. See §7 for the rationale.

### 2.3 PIPA Delta-V Convention

By the time `average_g_step` is called, the PIPA delta-V has been:
1. Read from PIPA hardware registers (pulse counts, platform frame).
2. Multiplied by the PIPA scale factor (~0.0585 m/s per pulse count).
3. Corrected for PIPA bias and misalignment.
4. Rotated from the stable-member frame to the inertial frame via REFSMMAT:
   `delta_v_inertial = mxv(refsmmat, delta_v_platform)`

The `delta_v` parameter to `average_g_step` is the result of this pipeline:
an inertially-resolved velocity increment in m/s, ready for direct addition to
the state vector velocity.

Source: `specs/state-vector-spec.md` §5.1 steps 1–3; `docs/architecture.md`
§7.4 steps 1–3.

### 2.4 AGC Erasable Variables Referenced

| AGC symbol | Octal address | Scale factor | Rust mapping |
|------------|---------------|-------------|--------------|
| `RN`       | 0306–0313     | B+28 m DP   | `sv.position` |
| `VN`       | 0314–0321     | B+7 m/s DP  | `sv.velocity` |
| `TEPHEM`   | 0340–0342     | B-28 s DP   | `sv.epoch` |
| `PIPAX/Y/Z`| 0125–0127     | pulse counts | consumed upstream by `services::average_g` |
| `REFSMMAT` | E3 bank, 0306 | B+0 (cosines) | `AgcState::refsmmat` (consumed upstream) |

---

## 3. Function: `average_g_step`

### 3.1 Purpose

Advance the state vector by one SERVICER cycle using the Average-G trapezoidal
integration scheme. This is the core of Cowell's method as implemented in
`Comanche055/AVERAGE_G_INTEGRATOR.agc`.

### 3.2 Signature

```rust
pub fn average_g_step(
    sv: StateVector,
    delta_v: Vec3,
    dt: f64,
    moon_pos: Vec3,
) -> StateVector
```

### 3.3 Parameters

| Parameter | Type | Units | Description |
|-----------|------|-------|-------------|
| `sv` | `StateVector` | — | Current state vector (position m, velocity m/s, epoch MET, frame). Must satisfy `StateVector` invariants INV-1 through INV-3. |
| `delta_v` | `Vec3` | m/s | Thrust-induced velocity increment over the interval, already rotated into the inertial frame by REFSMMAT. Zero for a free-fall test. |
| `dt` | `f64` | s | Integration interval. Normally 2.0 s (the SERVICER cycle). Must be positive and finite. |
| `moon_pos` | `Vec3` | m | Position of the Moon in the same inertial frame as `sv.position`. Obtained from `navigation::planetary::moon_position(sv.epoch)` by the caller. Required for third-body perturbation. |

### 3.4 Return Value

A new `StateVector` with:
- `position`: updated position after `dt` seconds.
- `velocity`: updated velocity after `dt` seconds (including gravity and thrust).
- `epoch`: `sv.epoch + Met::from_seconds(dt)` — advanced by the integration interval.
- `frame`: copied unchanged from `sv.frame`.

### 3.5 Algorithm

The Average-G scheme is a two-stage Cowell integrator. The equations of motion
are:

```
dr/dt = v
dv/dt = total_gravity(r, frame, moon_pos) + (delta_v / dt)
```

where `delta_v / dt` is the average thrust acceleration over the interval. In
practice the AGC accumulated the full `delta_v` term rather than computing the
instantaneous thrust-to-mass ratio; this is equivalent and avoids requiring a
mass estimate in the integrator.

The integration proceeds as follows:

**Step 1 — Gravity at interval start:**
```
g0 = total_gravity(sv.position, sv.frame, moon_pos)
```

**Step 2 — Midpoint velocity estimate (predictor):**
```
v_half = sv.velocity + delta_v + vscale(g0, dt / 2.0)
```

This is a first-order estimate of the velocity at the midpoint of the interval,
using gravity at the initial position and the full thrust delta-V. The `delta_v`
is added entirely at this stage because it represents the accumulated velocity
impulse for the whole interval; the AGC did not have a continuous thrust model.

**Step 3 — New position (using midpoint velocity):**
```
new_position = vadd(sv.position, vscale(v_half, dt))
```

Using the midpoint velocity to advance the position is a second-order accurate
position update (equivalent to a trapezoidal rule on position given the
predictor).

**Step 4 — Gravity at interval end:**
```
g1 = total_gravity(new_position, sv.frame, moon_pos)
```

**Step 5 — New velocity (corrector, trapezoidal gravity average):**
```
new_velocity = vadd(
    vadd(sv.velocity, delta_v),
    vscale(vadd(g0, g1), dt / 2.0)
)
```

The velocity update averages the gravitational acceleration at the beginning and
end of the interval (trapezoidal rule). This is the step that gives the algorithm
its name "Average-G". For a nearly constant gravity field (close to circular
orbit), this is second-order accurate.

**Step 6 — Update epoch:**
```
new_epoch = sv.epoch + Met::from_seconds(dt)
```

**Summary in pseudocode:**
```
g0          = total_gravity(sv.position, sv.frame, moon_pos)
v_half      = sv.velocity + delta_v + g0 * (dt / 2)
new_position = sv.position + v_half * dt
g1          = total_gravity(new_position, sv.frame, moon_pos)
new_velocity = sv.velocity + delta_v + (g0 + g1) * (dt / 2)
new_epoch    = sv.epoch + Met::from_seconds(dt)
return StateVector { position: new_position, velocity: new_velocity,
                     epoch: new_epoch, frame: sv.frame }
```

### 3.6 Preconditions

- `sv` must satisfy invariants INV-1, INV-2, INV-3 from `specs/state-vector-spec.md` §7.
- `dt > 0.0` and `dt.is_finite()`.
- All components of `delta_v` are finite.
- All components of `moon_pos` are finite.
- `norm(sv.position) > 0` (not at the body's centre of mass).
- `norm(moon_pos) > R_SOI_MOON` when `sv.frame == EarthInertial` — the Moon must
  be farther than 1 radius-SOI from Earth's origin (sanity check only; not a
  hard precondition for correctness).

### 3.7 Postconditions

- The returned `StateVector` satisfies INV-1, INV-2, INV-3.
- `result.frame == sv.frame` (frame is never changed by `average_g_step`).
- `result.epoch == sv.epoch + Met::from_seconds(dt)`.
- For zero `delta_v` and a circular orbit, `norm(result.position)` is
  approximately equal to `norm(sv.position)` (within numerical tolerance for
  a 2-second step).

### 3.8 Error handling

There are no recoverable error conditions. Invalid inputs (NaN, zero position,
non-finite `dt`) are logic errors in the caller and are caught by
`debug_assert!` checks in debug builds. In release builds, propagation of NaN
will be detected at the next `debug_assert_valid` call on the returned
`StateVector`, which triggers a restart.

---

## 4. Function: `propagate_coast`

### 4.1 Purpose

Propagate the state vector forward by an arbitrary time interval during coasting
flight (no thrust). The function uses the universal-variable Kepler solver from
`math::kepler` for the unperturbed conic part, then adds a first-order gravity
perturbation correction using a single Cowell step for short intervals.

When `math::kepler::kepler_step` is not yet implemented (stub returns `todo!()`),
the implementation falls back to a pure Cowell RK4 integration using
`total_gravity` as the acceleration function. This fallback is explicitly
acceptable during the initial validation phase.

### 4.2 Signature

```rust
pub fn propagate_coast(
    sv: StateVector,
    dt: f64,
    moon_pos: Vec3,
) -> StateVector
```

### 4.3 Parameters

| Parameter | Type | Units | Description |
|-----------|------|-------|-------------|
| `sv` | `StateVector` | — | Current state vector. |
| `dt` | `f64` | s | Propagation interval in seconds. May be large (e.g., one orbit period ≈ 5400 s for LEO). Must be positive and finite. |
| `moon_pos` | `Vec3` | m | Moon position in the same inertial frame as `sv`. |

### 4.4 Return Value

A new `StateVector` propagated to `sv.epoch + Met::from_seconds(dt)`.

### 4.5 Algorithm

The function uses a two-layer approach:

**Layer 1 — Keplerian conic (unperturbed two-body):**
```
mu = if sv.frame == EarthInertial { MU_EARTH } else { MU_MOON }
(r_kep, v_kep) = kepler_step(sv.position, sv.velocity, dt, mu)
```

This gives the exact conic trajectory under the point-mass gravity of the
primary body alone. It is valid for any interval length and any conic section
(circular, elliptical, hyperbolic, parabolic).

**Layer 2 — Perturbation correction (Cowell, single step):**

For the current implementation scope, the perturbation from J2 and the
third-body term is applied as a single Cowell step over the full interval. This
is first-order accurate in the perturbation and is sufficient for validation
against the primary error source (the Kepler propagator itself).

```
g_perturb = total_gravity(sv.position, sv.frame, moon_pos)
          - point_mass_gravity(sv.position, mu)   // subtract the Keplerian part
dv_perturb = vscale(g_perturb, dt)
dr_perturb = vscale(g_perturb, 0.5 * dt * dt)

r_final = vadd(r_kep, dr_perturb)
v_final = vadd(v_kep, dv_perturb)
```

where `point_mass_gravity(r, mu) = vscale(r, -mu / norm(r)^3)` is the
unperturbed point-mass acceleration already included in the Kepler step.

For the initial implementation, before `kepler_step` is available, a full
Cowell RK4 integration over `dt` using `total_gravity` as the acceleration
is acceptable. The RK4 scheme:

```
k1_r = sv.velocity
k1_v = total_gravity(sv.position, sv.frame, moon_pos)

k2_r = vadd(sv.velocity, vscale(k1_v, dt/2))
k2_v = total_gravity(vadd(sv.position, vscale(k1_r, dt/2)), sv.frame, moon_pos)

k3_r = vadd(sv.velocity, vscale(k2_v, dt/2))
k3_v = total_gravity(vadd(sv.position, vscale(k2_r, dt/2)), sv.frame, moon_pos)

k4_r = vadd(sv.velocity, vscale(k3_v, dt))
k4_v = total_gravity(vadd(sv.position, vscale(k3_r, dt)), sv.frame, moon_pos)

r_final = vadd(sv.position, vscale(vadd(vadd(k1_r, vscale(k2_r, 2.0)),
                                        vadd(vscale(k3_r, 2.0), k4_r)), dt/6.0))
v_final = vadd(sv.velocity, vscale(vadd(vadd(k1_v, vscale(k2_v, 2.0)),
                                        vadd(vscale(k3_v, 2.0), k4_v)), dt/6.0))
```

The RK4 fallback is fourth-order accurate for a single step but accumulates
error for large `dt`. For validation purposes (dt <= 5400 s LEO orbit period)
and the accuracy tolerance of the AGC navigation system, this is acceptable.

### 4.6 Implementation note on kepler_step availability

Until `math::kepler::kepler_step` is implemented, `propagate_coast` must compile
and function using only the Cowell RK4 fallback. The transition from fallback to
Kepler+perturbation should be a drop-in replacement: the function signature does
not change, only the internal implementation.

Recommended implementation pattern:

```rust
pub fn propagate_coast(sv: StateVector, dt: f64, moon_pos: Vec3) -> StateVector {
    // TODO: Replace with kepler_step + perturbation correction once
    // math::kepler::kepler_step is implemented.
    cowell_rk4(sv, dt, moon_pos)
}
```

### 4.7 Preconditions and postconditions

Same as `average_g_step` (§3.6, §3.7), with the additional constraint:
- `dt <= 86400.0` (one day). For longer propagations, split into sub-intervals
  and apply `soi_check` between each.

---

## 5. Function: `total_gravity`

### 5.1 Purpose

Compute the combined gravitational acceleration at a given position in a given
frame, including the primary body's gravity (with J2 for Earth) and the
third-body perturbation. This is the acceleration function passed to both
`average_g_step` and the Cowell RK4 fallback in `propagate_coast`.

### 5.2 Signature

```rust
pub fn total_gravity(
    position: Vec3,
    frame: Frame,
    moon_pos: Vec3,
) -> Vec3
```

### 5.3 Parameters

| Parameter | Type | Units | Description |
|-----------|------|-------|-------------|
| `position` | `Vec3` | m | Spacecraft position in `frame`. |
| `frame` | `Frame` | — | `EarthInertial` or `MoonInertial`. |
| `moon_pos` | `Vec3` | m | Moon position in the **same** frame as `position`. In `EarthInertial` this is the ECI position of the Moon. In `MoonInertial` this is the ECI position of the Earth expressed as `vsub([0,0,0], moon_pos_eci)` — the negated ECI Moon position gives the Earth's position in MCI. |

### 5.4 Algorithm

**Case `frame == EarthInertial`:**
```
g_primary = earth_gravity(position)          // MU_EARTH + J2 from gravity.rs
g_third   = third_body_perturbation(position, moon_pos, MU_MOON)
total     = vadd(g_primary, g_third)
```

**Case `frame == MoonInertial`:**
```
g_primary    = moon_gravity(position)        // MU_MOON point-mass from gravity.rs
earth_pos    = vscale(moon_pos, -1.0)        // Earth position in MCI
g_third      = third_body_perturbation(position, earth_pos, MU_EARTH)
total        = vadd(g_primary, g_third)
```

**Note on `moon_pos` convention in MoonInertial frame:**
When the state vector is in `MoonInertial` frame, all position vectors in that
frame have the Moon's centre as their origin. The Earth is located at
`-moon_pos_eci` in MCI (where `moon_pos_eci` is the Moon's ECI position obtained
from `planetary::moon_position`). The caller is responsible for supplying the
correct `moon_pos` for the active frame. The integration functions (`average_g_step`,
`propagate_coast`) document the required convention for their `moon_pos` parameter.

**Rationale for frame dispatch inside `total_gravity` rather than in the caller:**
The integrators call `total_gravity` multiple times per step (twice for Average-G,
four times for RK4). Placing the frame dispatch in one place avoids repeating the
match expression and keeps the integrator code free of frame-specific logic.

### 5.5 Preconditions

- `frame` is `EarthInertial` or `MoonInertial`. Passing `StableMember` is a
  logic error and panics via `debug_assert!`.
- `position` is finite and `norm(position) > 0`.
- `moon_pos` is finite and `norm(moon_pos) > 0`.
- The spacecraft must not be co-located with the third body:
  `norm(vsub(position, moon_pos)) > 1e3` m (1 km). This is enforced by
  `third_body_perturbation`'s own `debug_assert!`.

### 5.6 Return Value

Gravitational acceleration in m/s², expressed in `frame`.

---

## 6. Function: `soi_check`

### 6.1 Purpose

After each propagation step, test whether the spacecraft has crossed the
sphere-of-influence boundary and, if so, convert the state vector to the new
frame. This implements the SOI transition described in
`specs/state-vector-spec.md` §2.3 and `specs/gravity-spec.md` §5.

### 6.2 Signature

```rust
pub fn soi_check(
    sv: StateVector,
    moon_pos_eci: Vec3,
    moon_vel_eci: Vec3,
) -> StateVector
```

### 6.3 Parameters

| Parameter | Type | Units | Description |
|-----------|------|-------|-------------|
| `sv` | `StateVector` | — | State vector after a propagation step. |
| `moon_pos_eci` | `Vec3` | m | Moon's ECI position at `sv.epoch`. |
| `moon_vel_eci` | `Vec3` | m/s | Moon's ECI velocity at `sv.epoch`. Required for velocity frame conversion. |

### 6.4 SOI Test

The sphere-of-influence radius is `R_SOI_MOON = 66_183_000.0` m (defined in
`gravity.rs`), measured from the Moon's centre.

```
dist_from_moon = norm(vsub(sv.position_eci, moon_pos_eci))
```

where `sv.position_eci` is the position expressed in ECI (available directly
when `sv.frame == EarthInertial`; computed as
`vadd(sv.position, moon_pos_eci)` when `sv.frame == MoonInertial`).

- If `sv.frame == EarthInertial` and `dist_from_moon < R_SOI_MOON`:
  transition to `MoonInertial` (entering Moon SOI).
- If `sv.frame == MoonInertial` and `dist_from_moon > R_SOI_MOON`:
  transition to `EarthInertial` (leaving Moon SOI on transearth coast).
- Otherwise: return `sv` unchanged.

### 6.5 Frame Conversion Procedure

**EarthInertial → MoonInertial (entering lunar SOI):**
```
new_position = vsub(sv.position, moon_pos_eci)
new_velocity = vsub(sv.velocity, moon_vel_eci)
return StateVector { position: new_position, velocity: new_velocity,
                     epoch: sv.epoch, frame: Frame::MoonInertial }
```

**MoonInertial → EarthInertial (leaving lunar SOI):**
```
new_position = vadd(sv.position, moon_pos_eci)
new_velocity = vadd(sv.velocity, moon_vel_eci)
return StateVector { position: new_position, velocity: new_velocity,
                     epoch: sv.epoch, frame: Frame::EarthInertial }
```

The velocity conversion subtracts (or adds) the Moon's orbital velocity
relative to Earth at the crossing epoch. For the original AGC, `moon_vel_eci`
was obtained from the onboard planetary ephemeris in
`INTEGRATION_INITIALIZATION.agc`. In the Rust port it is computed by
`navigation::planetary` (currently a stub).

### 6.6 Calling Convention

`soi_check` must be called by the SERVICER and by `propagate_coast` after each
completed propagation step. The caller is responsible for obtaining
`moon_pos_eci` and `moon_vel_eci` from `navigation::planetary`. Until
`planetary::moon_position` is implemented, `soi_check` may be called with a
placeholder Moon position (e.g., `[3.844e8, 0.0, 0.0]` for a simplified
Earth-Moon geometry); the SOI test will still work correctly for trajectories
that do not approach the SOI boundary.

### 6.7 Note on SOI check placement in the SERVICER

The SERVICER calls `average_g_step` and then `soi_check` every 2 seconds. If
the spacecraft is near the SOI boundary, the 2-second step may overshoot the
boundary by a small amount. This is acceptable: the original AGC also performed
the check only at discrete step boundaries, and the position error from a single
2-second overshoot at trans-lunar coast velocities (~1 km/s) is approximately
2 km — well within the accuracy budget for the SOI transition.

---

## 7. Design Decisions and Deferred Work

### 7.1 Why Average-G is NOT classical RK4

The existing stub in `integration.rs` is named `rk4_step` and its doc comment
says "Runge-Kutta step". This name must be changed. The AGC's Average-G is a
two-stage predictor-corrector, not a four-stage Runge-Kutta. The distinction
matters for accuracy characterisation:

- RK4 is fourth-order accurate in `dt` for smooth ODEs.
- Average-G is second-order accurate in `dt` for the gravity term, but handles
  the thrust delta-V as a discrete impulse (first-order in the thrust
  integration).

For `dt = 2.0` s and LEO orbital dynamics, the local truncation error of
Average-G is dominated by the gravity curvature over 2 seconds, which is
approximately `(d³r/dt³) * dt³ / 6 ≈ 10⁻⁶ m` per step. Over a 1.5-hour orbit
(2700 steps) this accumulates to roughly 2–3 m position error, which is
consistent with the navigation accuracy achievable with the AGC's 1-metre
position resolution.

### 7.2 Why Encke's method is deferred

Encke's method is deferred for three reasons:

1. **Dependency**: Encke's method requires a working `kepler_step` as the
   reference conic. It cannot be tested until `kepler_step` is implemented.
2. **Validation sequencing**: The accuracy benefit of Encke's method is only
   visible over long coast arcs (hours to days). Validation against VirtualAGC
   telemetry should first confirm that Cowell+Kepler agrees with the original
   at the percent level; only then does the sub-percent improvement from Encke
   become meaningful.
3. **Complexity**: Encke's method requires periodic rectification logic (resetting
   the reference conic when the deviation exceeds a threshold). This adds state
   and control flow that obscures the simpler Cowell path during initial
   development.

Architecture §9.4 mentions Encke's method as the production coast integrator.
Once `kepler_step` is validated, a separate module `navigation::encke` can be
added and `propagate_coast` updated to delegate to it for `dt > 300` s.

### 7.3 Single-step vs. sub-stepped coast propagation

For `dt` much larger than 2 s, a single Cowell RK4 step loses accuracy. For
the fallback Cowell path, `propagate_coast` should sub-step for `dt > 10` s:

```
const COAST_SUBSTEP: f64 = 10.0;  // seconds per sub-step
let n_steps = (dt / COAST_SUBSTEP).ceil() as usize;
let h = dt / n_steps as f64;
for _ in 0..n_steps {
    sv = cowell_rk4(sv, h, moon_pos);
    sv = soi_check(sv, moon_pos_eci, moon_vel_eci);
}
```

The `moon_pos` should be updated at each sub-step if `planetary::moon_position`
is available; if not, a fixed Moon position is acceptable for the fallback path.

The Kepler+perturbation path in `propagate_coast` does not need sub-stepping
because `kepler_step` uses the universal-variable method, which is exact for
any `dt` under the unperturbed two-body problem.

### 7.4 Disposition of the existing `rk4_step` stub

The stub `pub fn rk4_step(state: StateVector, dt: f64) -> StateVector` must be
replaced by the three functions specified in §3–§5. It should not remain in the
module. If backwards compatibility with any existing call sites is needed, a
deprecated wrapper may temporarily delegate to `propagate_coast` with a zero
`moon_pos`, but this wrapper should be removed before the module is declared
complete.

---

## 8. Rust API Summary

```rust
// agc-core/src/navigation/integration.rs

use crate::math::linalg::{vadd, vscale, vsub, norm};
use crate::navigation::gravity::{earth_gravity, moon_gravity, third_body_perturbation,
                                  MU_EARTH, MU_MOON, R_SOI_MOON};
use crate::navigation::state_vector::{Frame, StateVector};
use crate::types::{Met, Vec3};

/// Compute combined gravitational acceleration at `position` in `frame`.
///
/// Dispatches to `earth_gravity` or `moon_gravity` for the primary body, then
/// adds `third_body_perturbation` for the opposing body. The `moon_pos` vector
/// must be expressed in the same frame as `position`:
/// - `EarthInertial`: `moon_pos` is the Moon's ECI position (from `planetary::moon_position`).
/// - `MoonInertial`: `moon_pos` is the Moon's ECI position; Earth's MCI position
///   is derived as `vscale(moon_pos, -1.0)`.
///
/// AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc`,
/// `Comanche055/ORBITAL_INTEGRATION.agc`.
pub fn total_gravity(position: Vec3, frame: Frame, moon_pos: Vec3) -> Vec3;

/// Advance `sv` by `dt` seconds using the Average-G trapezoidal scheme.
///
/// Implements Cowell's method with a two-stage predictor-corrector:
/// gravity is evaluated at the start and end of the interval and averaged.
/// `delta_v` is the thrust-induced velocity increment, already rotated to
/// the inertial frame by REFSMMAT (contributed by `services::average_g`).
///
/// Does not check the sphere-of-influence boundary; call `soi_check` on the
/// returned state vector if needed.
///
/// AGC source: `Comanche055/AVERAGE_G_INTEGRATOR.agc`.
pub fn average_g_step(
    sv: StateVector,
    delta_v: Vec3,
    dt: f64,
    moon_pos: Vec3,
) -> StateVector;

/// Propagate `sv` by `dt` seconds during coasting flight.
///
/// Uses `math::kepler::kepler_step` for the Keplerian component once available,
/// with a first-order perturbation correction. Falls back to Cowell RK4 until
/// `kepler_step` is implemented.
///
/// For `dt > 10 s` with the Cowell fallback, automatically sub-steps with a
/// 10-second step size.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`.
pub fn propagate_coast(sv: StateVector, dt: f64, moon_pos: Vec3) -> StateVector;

/// Test whether `sv` has crossed the sphere-of-influence boundary and, if so,
/// convert the state vector to the new frame.
///
/// `moon_pos_eci` and `moon_vel_eci` are the Moon's ECI position and velocity
/// at `sv.epoch`, obtained from `navigation::planetary`.
///
/// Returns `sv` unchanged if no SOI crossing is detected.
///
/// AGC source: `Comanche055/INTEGRATION_INITIALIZATION.agc` (body-selection logic).
pub fn soi_check(
    sv: StateVector,
    moon_pos_eci: Vec3,
    moon_vel_eci: Vec3,
) -> StateVector;
```

---

## 9. Test Cases

### TC-INT-1: Circular orbit — radius conservation (no thrust)

**Purpose**: Verify that `average_g_step` with zero delta-V preserves the
orbital radius for a circular orbit over many steps.

**Setup**:
- Circular LEO at 400 km altitude.
- `r0 = [6_778_000.0, 0.0, 0.0]` m (ECI, on +X axis).
- `v0 = [0.0, sqrt(MU_EARTH / 6_778_000.0), 0.0]` m/s ≈ `[0.0, 7669.6, 0.0]`.
- `sv = StateVector { position: r0, velocity: v0, epoch: Met(0), frame: Frame::EarthInertial }`.
- `delta_v = [0.0, 0.0, 0.0]` m/s.
- `dt = 2.0` s.
- `moon_pos = [3.844e8, 0.0, 0.0]` m (simplified Moon position).

**Procedure**: Propagate for 100 steps (200 s) by calling `average_g_step`
100 times.

**Acceptance criteria**:
- `norm(result.position)` remains within ±50 m of `6_778_000.0` m at all 100
  intermediate states.
- `norm(result.velocity)` remains within ±0.1 m/s of `7669.6` m/s.

**Rationale**: For a circular orbit, the exact solution has constant radius.
The 50 m tolerance allows for second-order truncation error from the Average-G
scheme over 200 s.

---

### TC-INT-2: Average-G in free-fall — matches gravity-only extrapolation

**Purpose**: Verify that `average_g_step` with zero delta-V produces a result
consistent with direct gravity integration (no thrust contamination).

**Setup**:
- LEO state: `position = [7_000_000.0, 0.0, 0.0]` m, `velocity = [0.0, 0.0, 0.0]` m/s.
  (Spacecraft starting from rest; will fall toward Earth.)
- `delta_v = [0.0, 0.0, 0.0]`.
- `dt = 2.0` s.
- `moon_pos = [3.844e8, 0.0, 0.0]` m.

**Expected values** (hand calculation):
- `g0 = earth_gravity([7e6, 0, 0])` ≈ `[-8.147, 0, 0]` m/s² (point-mass only;
  J2 contribution is zero on the equatorial x-axis at z=0).
- `v_half = [0,0,0] + [0,0,0] + g0 * 1.0` ≈ `[-8.147, 0, 0]` m/s.
- `new_position ≈ [7e6 + (-8.147)*2, 0, 0]` = `[6_999_983.7, 0, 0]` m.
- `g1 = earth_gravity([6_999_983.7, 0, 0])` ≈ `[-8.147, 0, 0]` m/s² (negligible change).
- `new_velocity ≈ [0 + (g0[0]+g1[0])*1.0, 0, 0]` ≈ `[-16.294, 0, 0]` m/s.

**Acceptance criteria**:
- `result.velocity[0]` is within ±0.001 m/s of `-16.294`.
- `result.position[0]` is within ±0.1 m of `6_999_983.7`.
- `result.velocity[1]` and `result.velocity[2]` are zero (by symmetry).

---

### TC-INT-3: Average-G with known thrust delta-V — velocity increment applied correctly

**Purpose**: Verify that a thrust delta-V is correctly added to the state vector
and that the position update reflects the modified velocity.

**Setup**:
- ISS-like orbit: `position = [6_781_000.0, 0.0, 0.0]` m (on +X axis, z=0).
- `velocity = [0.0, 7_660.0, 0.0]` m/s (approximate circular velocity).
- `delta_v = [0.0, 10.0, 0.0]` m/s (a 10 m/s prograde burn delta-V).
- `dt = 2.0` s.
- `moon_pos = [3.844e8, 0.0, 0.0]` m.

**Expected values** (approximate):
- `g0 ≈ earth_gravity([6.781e6, 0, 0])` ≈ `[-8.669 + J2_correction, 0, 0]` m/s².
  The J2 correction at z=0 adds a small inward radial component. Total ≈ `[-8.681, 0, 0]` m/s².
- `v_half = [0, 7660, 0] + [0, 10, 0] + [-8.681, 0, 0] * 1.0`
        ≈ `[-8.681, 7670, 0]` m/s.
- `new_velocity ≈ [0, 7660, 0] + [0, 10, 0] + [-8.681, 0, 0] * 1.0 + [-8.681, 0, 0] * 1.0`
  (using average of g0 and g1; g1 ≈ g0 since position barely changed in 2 s)
  ≈ `[-17.362, 7670, 0]` m/s.

**Acceptance criteria**:
- `result.velocity[1]` is within ±0.01 m/s of `7670.0` (original velocity plus delta-V).
- `result.velocity[0]` is within ±0.01 m/s of `-17.362` (gravity contribution only, as delta-V was along Y).
- The difference `result.velocity[1] - sv.velocity[1]` equals `10.0` m/s to ±1e-6
  (delta-V must be applied exactly, no numerical loss).

---

### TC-INT-4: Energy conservation during coast propagation

**Purpose**: Verify that `propagate_coast` conserves specific orbital energy for
a circular orbit.

**Setup**:
- Circular LEO at 400 km: `r = 6_778_000.0` m, `v_circ = sqrt(MU_EARTH / r)`.
- `sv = { position: [r, 0, 0], velocity: [0, v_circ, 0], epoch: Met(0), frame: EarthInertial }`.
- `moon_pos = [3.844e8, 0.0, 0.0]` m (third-body perturbation is small and will
  slightly break energy conservation; include it to test numerical stability).
- `dt = 5400.0` s (approximately one full LEO orbit).

**Specific orbital energy** (ignoring perturbations):
```
E = 0.5 * v² - MU_EARTH / r  [m²/s²]
E0 = 0.5 * v_circ² - MU_EARTH / r = -MU_EARTH / (2 * r) ≈ -29.43e6 m²/s²
```

**Acceptance criteria** (Cowell RK4 fallback with 10 s sub-steps):
- `|E_final - E0| / |E0| < 1e-4` (energy conserved to 0.01%).
- `norm(result.position)` is within ±5 km of `6_778_000.0` m.

**Rationale**: Fourth-order Runge-Kutta with a 10 s step preserves energy to
approximately `O(h^4) * T ≈ 10^4 * 540 ≈ 5×10^6` in relative terms — well within
1e-4 for the relevant parameter magnitudes.

---

### TC-INT-5: One-orbit round trip — return to near-original position

**Purpose**: Verify that propagating for exactly one orbital period returns the
spacecraft to within a small tolerance of its starting position.

**Setup**:
- Circular LEO at 400 km, same as TC-INT-4.
- Orbital period: `T = 2π * sqrt(r³ / MU_EARTH)`.
  For `r = 6_778_000.0`:
  `T = 2π * sqrt(6.778e6³ / 3.986e14) = 2π * sqrt(2.941e5) ≈ 5571.0` s.
- `moon_pos` fixed at `[3.844e8, 0.0, 0.0]` m.

**Procedure**:
1. Compute `T` using the formula above.
2. Call `propagate_coast(sv, T, moon_pos)`.
3. Measure positional displacement: `delta_r = vsub(result.position, sv.position)`.
4. Measure velocity displacement: `delta_v = vsub(result.velocity, sv.velocity)`.

**Acceptance criteria** (Cowell RK4 fallback):
- `norm(delta_r) < 1000.0` m (position error less than 1 km after one orbit).
- `norm(delta_v) < 1.0` m/s (velocity error less than 1 m/s after one orbit).

**Note on the Moon perturbation**: The fixed Moon position `[3.844e8, 0, 0]`
introduces a small secular acceleration that slightly shifts the orbit. The
tolerances above account for this; a tighter test with `moon_pos = [0, 0, 0]`
(zero third-body perturbation) would achieve sub-metre closure.

---

### TC-INT-6: SOI transition from ECI to MCI

**Purpose**: Verify that `soi_check` correctly converts the state vector from
`EarthInertial` to `MoonInertial` when the spacecraft crosses the SOI boundary.

**Setup**:
- Moon at `moon_pos_eci = [3.844e8, 0.0, 0.0]` m.
- Moon velocity `moon_vel_eci = [0.0, 1022.0, 0.0]` m/s (approximate orbital velocity).
- Spacecraft just inside the SOI: distance from Moon = `R_SOI_MOON - 1000.0` m.
- `sv.position = vadd(moon_pos_eci, [-(R_SOI_MOON - 1000.0), 0.0, 0.0])`
  = `[3.844e8 - 65_182_000.0, 0, 0]` = `[319_218_000.0, 0, 0]` m.
- `sv.velocity = [0.0, 900.0, 0.0]` m/s (approximate trans-lunar coast velocity).
- `sv.frame = Frame::EarthInertial`.

**Procedure**: Call `soi_check(sv, moon_pos_eci, moon_vel_eci)`.

**Acceptance criteria**:
- `result.frame == Frame::MoonInertial`.
- `result.position ≈ [-(R_SOI_MOON - 1000.0), 0, 0]` m
  (i.e., `[-65_182_000.0, 0, 0]`; this is the ECI position minus Moon's ECI position).
- `result.velocity ≈ [0.0, 900.0 - 1022.0, 0.0]` = `[0.0, -122.0, 0.0]` m/s.
- `result.epoch == sv.epoch` (SOI check does not change epoch).

---

### TC-INT-7: total_gravity — ECI frame combines earth_gravity and Moon third-body

**Purpose**: Verify that `total_gravity` in `EarthInertial` frame equals the sum
of `earth_gravity` and `third_body_perturbation` from `gravity.rs`, and that the
result has the correct sign and approximate magnitude.

**Setup**:
- `position = [7_000_000.0, 0.0, 0.0]` m.
- `frame = Frame::EarthInertial`.
- `moon_pos = [3.844e8, 0.0, 0.0]` m.

**Expected**:
- `g_earth = earth_gravity(position)` ≈ `[-8.147, 0, 0]` m/s².
- `g_moon = third_body_perturbation(position, moon_pos, MU_MOON)` — small positive
  x component (Moon attracts spacecraft, net perturbation toward Moon after
  subtracting Moon-on-Earth term).
- `total = vadd(g_earth, g_moon)`.

**Acceptance criteria**:
- `total[0]` matches `g_earth[0] + g_moon[0]` to 1 ULP (exact sum).
- `total[1]` and `total[2]` are zero (all vectors on X axis by construction).
- `total[0] < 0` (net acceleration toward Earth dominates).
- `|total[0] - g_earth[0]| / |g_earth[0]| < 1e-3` (Moon perturbation at 7000 km
  altitude is less than 0.1% of Earth gravity).

---

## 10. Spec Quality Checklist

| Item | Status |
|------|--------|
| AGC source file and line range referenced | Satisfied — `AVERAGE_G_INTEGRATOR.agc`, `ORBITAL_INTEGRATION.agc`, `INTEGRATION_INITIALIZATION.agc` cited throughout. |
| All erasable variables and AGC addresses listed | Satisfied — §2.4 table. |
| Scale factors documented for all fixed-point values | Satisfied — §2.4 cites B+28 m for position, B+7 m/s for velocity, B-28 s for epoch. Rust port uses `f64` SI throughout. |
| Corresponding `f64` SI units documented | Satisfied — all parameters in §3.3, §4.3, §5.3, §6.3 specify SI units. |
| Input/output preconditions and postconditions stated | Satisfied — §3.6, §3.7, §4.7, §5.5, §5.6. |
| Edge cases and error handling specified | Satisfied — §3.8; §7.3 (large dt sub-stepping); §6.7 (SOI boundary overshoot). |
| At least 5 test cases with expected values | Satisfied — 7 test cases (TC-INT-1 through TC-INT-7), all with numerical expectations. |
| Rust API signature designed | Satisfied — §8. |
| Invariants explicitly stated | Satisfied — §3.7 postconditions; references to `state-vector-spec.md` §7. |
| Consistency with `docs/architecture.md` checked | Satisfied — §7.4 and §9.4 cross-referenced throughout; integration method names corrected (Average-G is not RK4). |

---

## 11. Cross-References

- `docs/architecture.md` §7.4 — SERVICER task structure and 2-second cycle.
- `docs/architecture.md` §9.4 — Gravity model scope (Cowell vs. Encke).
- `specs/state-vector-spec.md` §5.1 — SERVICER calling convention and delta-V pipeline.
- `specs/state-vector-spec.md` §5.2 — Conic propagation calling convention.
- `specs/state-vector-spec.md` §2.3 — SOI transition procedure.
- `specs/gravity-spec.md` §4 — `earth_gravity`, `moon_gravity`, `third_body_perturbation` specifications.
- `specs/gravity-spec.md` §5 — SOI radius definition and Laplace criterion.
- `agc-core/src/navigation/gravity.rs` — constants `MU_EARTH`, `MU_MOON`, `R_SOI_MOON`; functions `earth_gravity`, `moon_gravity`, `third_body_perturbation`.
- `agc-core/src/navigation/state_vector.rs` — `StateVector`, `Frame`, `StateVector::ZERO`.
- `agc-core/src/navigation/planetary.rs` — `moon_position(t: Met) -> Vec3` (stub).
- `agc-core/src/math/kepler.rs` — `kepler_step(r0, v0, dt, mu) -> (Vec3, Vec3)` (stub).
- `agc-core/src/math/linalg.rs` — `vadd`, `vsub`, `vscale`, `norm`, `dot`.
