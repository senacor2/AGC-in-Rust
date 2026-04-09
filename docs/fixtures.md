# Navigation Accuracy Test Fixtures

## Overview

The `agc-test/fixtures/` directory contains JSON fixture files that serve as
reference inputs and expected outputs for the navigation accuracy tests in
`agc-test/tests/navigation_accuracy.rs`. The fixtures enable **Level 1** and
**Level 2** tests from the testing strategy (`docs/testing.md`) to run without
a running VirtualAGC instance.

The fixtures combine two sources:

- **Analytically computed** reference values derived from first principles using
  the same physical constants as `agc-core/src/navigation/gravity.rs`.
- **VirtualAGC-derived** constants extracted from the Comanche055 assembly source
  via a native arm64 build of yaAGC/yaYUL (`vagc_constants.json`). These document
  the actual AGC constant values and quantify the differences from modern
  best-estimates used in the Rust implementation.

---

## 1. Fixture Files

### `gravity_cases.json`

**Purpose**: Regression tests for `earth_gravity` and `moon_gravity` functions.

**Content**: 8 test cases covering:
- Earth gravity at 5 altitudes: equatorial surface, 200 km LEO, 400 km LEO (near-ISS), GEO (35786 km), and cislunar midpoint (192000 km).
- Moon gravity at 3 altitudes: surface, 100 km LLO (Low Lunar Orbit), and 500 km.

**Case structure**:
```json
{
  "name": "earth_leo_400km_equatorial",
  "description": "...",
  "position_m": [6778137.0, 0.0, 0.0],
  "frame": "ECI",
  "body": "earth",
  "expected_accel_m_s2": [-8.68845, 0.0, 0.0],
  "tolerance_m_s2": 0.001
}
```

**Fields**:
- `name` — unique identifier, used in test failure messages.
- `description` — full computation trail showing how the expected value was derived.
- `position_m` — position vector in metres in the stated frame.
- `frame` — `"ECI"` (Earth-Centred Inertial) or `"MCI"` (Moon-Centred Inertial).
- `body` — `"earth"` (calls `earth_gravity`) or `"moon"` (calls `moon_gravity`).
- `expected_accel_m_s2` — analytically computed acceleration in m/s² `[ax, ay, az]`.
- `tolerance_m_s2` — per-component absolute tolerance in m/s².

---

### `servicer_cycle_cases.json`

**Purpose**: Regression tests for the SERVICER (Average-G) navigation cycle —
specifically the PIPA compensation pipeline: bias correction → scale factor →
misalignment correction → REFSMMAT rotation.

**Content**: 3 test cases:
1. **10-cycle zero-PIPA free-fall** — verifies orbital radius and speed
   conservation over 10 SERVICER cycles (20 seconds) with no thrust.
2. **Single-cycle prograde burn** — verifies that PIPA counts `[0, 100, 0]`
   produce a +5.85 m/s delta-V along the prograde direction (y-axis).
3. **Single-cycle radial burn** — verifies that PIPA counts `[100, 0, 0]`
   produce a +5.85 m/s delta-V along the radial direction (x-axis).

**Case structure**:
```json
{
  "name": "leo_prograde_burn_single_cycle",
  "description": "...",
  "initial_state": {
    "position_m": [6778000.0, 0.0, 0.0],
    "velocity_m_s": [0.0, 7668.64, 0.0],
    "epoch_cs": 0,
    "frame": "EarthInertial"
  },
  "pipa_sequence": [[0, 100, 0]],
  "pipa_cal": {
    "scale": 0.0585,
    "bias": [0, 0, 0],
    "misalignment": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  },
  "refsmmat": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  "moon_pos_m": [384400000.0, 0.0, 0.0],
  "expected_final_state": { ... },
  "position_tolerance_m": 200.0,
  "velocity_tolerance_m_s": 0.05,
  "num_cycles": 1,
  "note": "..."
}
```

**Fields**:
- `initial_state` — position (m), velocity (m/s), epoch (cs), frame.
- `pipa_sequence` — list of raw PIPA count triples, one per cycle.
- `pipa_cal` — PIPA calibration: scale (m/s/count), bias (counts/cycle), misalignment (3×3 matrix).
- `refsmmat` — Reference-to-Stable-Member matrix (3×3).
- `moon_pos_m` — Moon ECI position used in gravity computation (m). Must match the placeholder in `servicer_task` (`[3.844e8, 0, 0]`).
- `expected_final_state` — state vector after all `num_cycles` cycles.
- `position_tolerance_m` / `velocity_tolerance_m_s` — per-component absolute tolerances.
- `num_cycles` — number of SERVICER cycles to run; must equal `len(pipa_sequence)`.

**Test strategy note**: Single-cycle cases (`num_cycles == 1`) are run through
`servicer_task` directly; the delta-V is isolated by comparing against a
zero-PIPA reference run. Multi-cycle zero-PIPA cases are run through
`average_g_step` directly (bypassing the Waitlist scheduling mechanism, which
is limited to 8 concurrent entries by `MAX_WAITLIST_TASKS`).

---

### `orbit_propagation_cases.json`

**Purpose**: Regression tests for `propagate_coast`, the Cowell RK4 orbital
propagator.

**Content**: 4 test cases:
1. **LEO 10-second coast** — small step, verifies Keplerian position to 100 m.
2. **LEO 100-second coast** — medium step, verifies Keplerian position to 500 m.
3. **LEO one-orbit coast** (~5550 s) — verifies orbital radius and speed
   conservation rather than exact Cartesian position.
4. **LLO 100-second coast** — verifies lunar orbit propagation with Earth
   third-body perturbation.

**Case structure**:
```json
{
  "name": "leo_circular_coast_100s",
  "description": "...",
  "initial_state": { ... },
  "dt_s": 100.0,
  "moon_pos_m": [384400000.0, 0.0, 0.0],
  "expected_state": { ... },
  "position_tolerance_m": 500.0,
  "velocity_tolerance_m_s": 2.0,
  "note": "..."
}
```

**Fields**:
- `initial_state` / `expected_state` — same structure as `StateVector`.
- `dt_s` — propagation interval in seconds.
- `moon_pos_m` — Moon ECI position supplied to `propagate_coast`.
- `position_tolerance_m` / `velocity_tolerance_m_s` — per-component absolute tolerances (except for the one-orbit case, which uses radius/speed norm comparisons).

### `vagc_constants.json`

**Purpose**: Documents the actual gravity and integration constants extracted
from the Comanche055 AGC assembly source via VirtualAGC (yaYUL native arm64
build). These are NOT test inputs — they are a reference document that maps
each AGC constant to the corresponding Rust value and quantifies the difference.

**Content**:
- `gravity_constants` — MUEARTH, MUMOON, J2REQSQ, RSPHERE, RDE, RDM with AGC
  values, scale factors, SI conversions, and comparison against `gravity.rs`
- `erasable_addresses` — octal addresses for CDUX/Y/Z, PIPAX/Y/Z, RN, VN,
  REFSMMAT, TEPHEM
- `integration_constants` — DT/2MIN, DT/2MAX, OMEGMOON, J4REQ/J3, 2J3RE/J2,
  3J22R2MU
- `comparison_with_rust_implementation` — per-constant relative error analysis

**Source**: `~/virtualagc/Comanche055/ORBITAL_INTEGRATION.agc`,
`~/virtualagc/Comanche055/INTEGRATION_INITIALIZATION.agc`,
`~/virtualagc/Comanche055/ERASABLE_ASSIGNMENTS.agc`

**Key findings**:
- MU_EARTH: AGC 3.986032e14 vs Rust 3.986004e14 — 7 ppm difference (modern value more accurate)
- MU_MOON: AGC 4.902778e12 vs Rust 4.902800e12 — 5 ppm difference
- RSPHERE (SOI): AGC 64374 km vs Rust 66183 km — 2.7% difference (1960s mass ratio)
- J2REQSQ is a compound precomputed constant, not simple J2*R^2

**Test**: `test_vagc_constant_consistency` in `navigation_accuracy.rs` validates
that the Rust constants are within expected tolerance of the AGC values.

---

## 2. How Reference Values Were Computed

### Gravity cases

All expected values are computed from the closed-form formulas implemented in
`gravity.rs`, using the constants defined at module scope:

| Constant | Value |
|----------|-------|
| `MU_EARTH` | 3.986\_004\_418 × 10¹⁴ m³/s² |
| `MU_MOON` | 4.902\_800\_118 × 10¹² m³/s² |
| `R_EARTH` | 6\_378\_137.0 m |
| `J2_EARTH` | 1.082\_626\_68 × 10⁻³ |
| `R_MOON` | 1\_737\_400.0 m |

**Earth gravity at equatorial position [r, 0, 0]** (z = 0, J2 correction is
radially inward):
```
a_pm   = -MU_EARTH / r²
J2F    = 1.5 × J2_EARTH × MU_EARTH × R_EARTH²
a_j2   = -J2F / r⁴       (at equator, z=0)
a_total = a_pm + a_j2
```

**Moon gravity at [r, 0, 0]** (point-mass, no J2):
```
a = -MU_MOON / r²
```

Each fixture description field contains the full numerical computation trail
so that the expected value can be independently verified by hand.

### Servicer cases

The expected delta-V for single-cycle PIPA cases is computed as:
```
biased     = (pipa_count - bias) × scale
mis_dv     = misalignment × biased
inertial_dv = REFSMMAT × mis_dv
```

For the nominal calibration case (identity misalignment, identity REFSMMAT, zero
bias, scale = 0.0585 m/s/count):
```
delta_V = pipa_count × 0.0585 m/s
```

For the 10-cycle zero-PIPA coast, the acceptance criterion is orbital radius and
speed conservation derived from the circular orbit initial conditions.

### Orbit propagation cases

Expected positions and velocities are computed from the Keplerian two-body
circular orbit solution:

```
v_circ = sqrt(MU_EARTH / r)
θ(t)   = v_circ × t / r       (angle swept in time t)
x(t)   = r × cos(θ)
y(t)   = r × sin(θ)
vx(t)  = -v_circ × sin(θ)
vy(t)  = v_circ × cos(θ)
```

The tolerances account for:
1. The J2 oblateness perturbation implemented in `earth_gravity` (which is not
   included in the Keplerian reference).
2. Truncation error from the sub-stepped RK4 integrator (`COAST_SUBSTEP = 10 s`).
3. Moon third-body perturbation for the Earth-orbit cases (~1 × 10⁻⁶ m/s²,
   producing ~0.06 m error over 100 s).

The one-orbit case uses orbital energy conservation as the acceptance metric
instead of Cartesian position, since the exact return position depends on J2
precession which causes the orbit plane to rotate slightly over one orbit period.

---

## 3. How to Regenerate Fixtures from VirtualAGC

Once Podman is available on the development machine:

1. Pull the VirtualAGC Podman image:
   ```sh
   podman pull virtualagc/virtualagc
   ```

2. Start yaAGC with the Comanche055 binary and enable erasable-memory debug:
   ```sh
   podman run --rm -p 19697:19697 virtualagc/virtualagc \
     yaAGC --core=Comanche055.bin --debug=erasable
   ```

3. Connect the fixture-capture tool (to be implemented as
   `agc-test/src/oracle/fixture_capture.rs`):
   - Send P11 activation sequence over the channel-word protocol (port 19697).
   - Inject scripted PIPA counts at channels 014–016 for the desired number of
     cycles.
   - After the desired cycles, dump erasable memory addresses 0306–0321 (RN/VN)
     and 0340 (TEPHEM).
   - Convert the raw AGC double-precision words to `f64` using `from_agc_dword`
     with the documented scale factors (B+28 for position, B+7 for velocity).
   - Replace `expected_final_state` in `servicer_cycle_cases.json` with the
     converted values.

4. For gravity cases, VirtualAGC erasable memory can be read after an artificial
   SERVICER cycle that computes gravity at a known position (set via uplink or
   test-only direct erasable write). Extract the gravity vector from the AGC's
   internal state.

5. Run `cargo test -p agc-test` to verify that the Rust implementation matches
   the VirtualAGC reference values within the stated tolerances.

6. Commit the updated fixtures as part of the PR that introduces VirtualAGC
   integration, with a clear diff comment explaining the change from analytical
   to VirtualAGC-derived values.

---

## 4. Tolerance Rationale

| Fixture category | Tolerance | Rationale |
|-----------------|-----------|-----------|
| Gravity functions | 0.001 m/s² | Hand-calculation precision limit; the formula is exact, the reference value is rounded to 5 significant figures. |
| Cislunar Earth gravity | 0.0001 m/s² | Point-mass only at this range; tighter tolerance is achievable. |
| SERVICER delta-V | 0.05 m/s | PIPA scale factor is 0.0585 m/s/count; tolerance is ~0.1% of one count. |
| SERVICER radius (free-fall) | 50 m | Average-G scheme second-order accuracy over 20 s; consistent with TC-INT-1 (0.01% of r0 over 200 s). |
| SERVICER speed (free-fall) | 0.5 m/s | Speed variation in circular orbit over 20 s is well within this bound. |
| Orbit propagation position (10 s) | 100 m | J2 perturbation + RK4 local truncation error over 10 s. |
| Orbit propagation position (100 s) | 500 m | J2 perturbation (dominant) + accumulated RK4 error over 100 s. |
| Orbit propagation speed | 1–2 m/s | J2 perturbation contribution to speed error over 100 s. |
| One-orbit radius | 5000 m | Consistent with TC-INT-4; RK4 energy conservation < 0.01% over 5400 s yields < 5 km radius error. |
| Lunar orbit position | 500 m | Earth third-body perturbation (~3 × 10⁻⁵ m/s²) accumulates to ~0.15 m/s velocity error over 100 s, or ~8 m position error. Tolerance is conservative. |

The general principle: tolerances are set to detect implementation bugs (wrong
formula, wrong sign, wrong constant) while remaining insensitive to the
difference between the Rust `f64` implementation and the AGC's double-precision
fixed-point arithmetic.

---

## 5. How to Add New Fixtures

### Gravity case

1. Choose a physically interesting position (e.g., a specific mission phase).
2. Compute the expected acceleration using the formulas in §2.
3. Add a new JSON object to `gravity_cases.json`. Increment the case count in
   the description field.
4. Run `cargo test -p agc-test test_gravity_fixtures` to verify it passes.

### Servicer or orbit case

1. Choose initial conditions (position, velocity, frame).
2. For a single-cycle PIPA case: compute the expected delta-V using the pipeline
   formula in §2.
3. For an orbit case: compute the expected Keplerian final state using the
   formulas in §2, and select tolerances from the table in §4.
4. Add the new JSON object to the appropriate fixture file.
5. Run `cargo test -p agc-test` to verify.

### Naming convention

Fixture names use snake_case and describe the scenario:
`{body}_{scenario}_{duration_or_altitude}`

Examples:
- `earth_surface_equatorial_point_mass_and_j2`
- `leo_prograde_burn_single_cycle`
- `moon_llo_circular_coast_100s`
