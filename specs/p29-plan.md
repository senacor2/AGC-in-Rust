# P29 (Time-of-Longitude) Implementation Plan

**Status**: Approved
**Scope**: Implement P29, the inverse of P21 — crew enters a target longitude, program returns the GET at which the CSM ground track will cross that longitude, plus the latitude and altitude at that crossing.
**Module owner**: `agc-core/src/navigation/conics.rs` (solver), `agc-core/src/programs/p29.rs` (program shell — new).
**Blocked by**: #18 (GMST + ECEF MS-F1) — must land first; P29 directly uses `inertial_to_earth_fixed` and `met_to_gha`.
**AGC source**: `Comanche055/P20-P25.agc` (P29 entry sequence), `Comanche055/LAT-LONG_SUBROUTINES.agc`.

---

## 1. What P29 does

| Direction | Program |
|---|---|
| GET → lat/lon/alt | P21 |
| lon → GET, lat, alt | P29 |

Iteratively propagate the CSM state vector forward in time, rotate to ECEF, extract longitude, root-find on `longitude(t) - target_lon = 0`.

## 2. Mission criticality and dependencies

- **Mission criticality**: Low. Utility for ground-station pass prediction and landing-site planning; the mission can fly without it (ground supplies this normally).
- **Hard dependencies**:
  - #18 (MS-F1: `inertial_to_earth_fixed`, `met_to_gha`, `OMEGA_EARTH`) — must be merged first.
  - `math::kepler::kepler_step` — already implemented.
  - V/N data-entry layer — already exists in `services/v_n.rs`.

## 3. Current state

| Component | Status |
|---|---|
| `PROGRAM_TABLE[29]` | Empty (`None`) |
| `programs/p29.rs` | Does not exist |
| `specs/p29-plan.md` | This file |
| Pure-math solver `time_of_longitude(...)` | Does not exist |
| Symbolic listing reference | One line only (`input/AGC Quick Reference.md:636`) |

## 4. Algorithm

**Inputs**: `csm_pos`, `csm_vel`, `epoch_s`, `target_lon_rad`, `gha_epoch_rad`.

**Outputs**: `time_of_crossing_s` (GET), `lat_rad`, `alt_m`.

**Crossing count**: hard-coded to 1 (next crossing after epoch). Crew cannot request the Nth crossing in this implementation; that can be a later extension.

**Method** (Newton-Raphson on time):

1. Compute the current ground-track longitude `lon_now` via `inertial_to_earth_fixed(csm_pos, met_to_gha(epoch_s, gha_epoch_rad))`.
2. Compute the orbital period `T_orb` from energy: `a = -μ / 2E`, `T = 2π √(a³/μ)`.
3. Compute the longitude-drift rate `dlon/dt ≈ -OMEGA_EARTH + 2π / T_orb` for prograde orbits (ground track moves east at the inertial-period rate minus Earth-rotation rate).
4. Initial guess: `t₀ = epoch_s + (target_lon - lon_now) / (dlon/dt)`, wrapped into `[epoch_s, epoch_s + T_orb]`.
5. Newton iterations:
   - Propagate to candidate `t` via `kepler_step`.
   - Rotate to ECEF, extract `lon(t)`.
   - Update `t ← t - (lon(t) - target_lon) / (dlon/dt(t))` where the derivative is recomputed each step from the propagated state.
   - Convergence: `|lon(t) - target_lon| < 1e-5 rad` (≈ 100 m at the equator).
   - Cap at 20 iterations; failure → `P29Error::NoConvergence`.

**Edge cases**:
- Hyperbolic / parabolic trajectory: no orbital period → `P29Error::Hyperbolic`.
- Equatorial orbits crossing the target longitude only when the longitude lies in the orbital plane → handled generically by the Newton iteration converging on the unique crossing time.
- Retrograde orbits: sign of `dlon/dt` flips; handled because the formula uses the signed period derivative.
- Zero angular momentum (degenerate input): `P29Error::ZeroAngularMomentum`.

## 5. Module layout

```
navigation/conics.rs                   (extend — currently 889 LOC)
  pub fn time_of_longitude(
      csm_pos: Vec3, csm_vel: Vec3,
      epoch_s: f64,
      target_lon_rad: f64,
      gha_epoch_rad: f64,
  ) -> Result<TimeOfLongitudeResult, P29Error>;
  pub struct TimeOfLongitudeResult {
      pub time_of_crossing_s: f64,
      pub lat_rad: f64,
      pub alt_m: f64,
  }
  pub enum P29Error { Hyperbolic, NoConvergence, ZeroAngularMomentum }

programs/p29.rs                        (new — ~250–350 LOC)
  pub const P29_MAJOR_MODE: u8 = 29;
  pub const P29_PRIORITY: JobPriority = 7;  // same tier as P21
  pub fn p29_init(state: &mut AgcState) -> JobPriority;
  fn write_result_to_dsky(state, result);
  // Alarm codes (octal)
  const ALARM_P29_NO_CSM_SV: u16 = 0o01430;
  const ALARM_P29_HYPERBOLIC: u16 = 0o01431;
  const ALARM_P29_NO_CONV: u16 = 0o01432;

programs/mod.rs                        (extend)
  // PROGRAM_TABLE[29] = Some(p29::p29_init);
```

The solver lives in `conics.rs` as a pure orbital-mechanics function, consistent with `kepler_step` and the existing `OrbitalElements` helpers. The P29 program shell stays thin, matching the P21 pattern.

## 6. Crew interface (DSKY)

- **Input noun**: reuse Noun 89 (target geodetic point). Crew enters `R1 = lat (deg)` (ignored / informational), `R2 = lon (deg)` (consumed by P29), `R3 = alt (m)` (ignored / informational). Reusing N89 avoids growing the noun-table and matches the AGC convention for geodetic-point input.
- **Output noun**: Noun 34 (time). Display the converged `time_of_crossing_s` formatted as HMS.
- **Crossing count**: hard-coded to 1 (next crossing). No crew input for this.
- **Crew flow**: `V37 E 29 E` → P29 active, prompts for N89 via `V25 N89` (flashing). Crew loads longitude, presses ENTR. P29 runs solver, displays time on V06 N34. No further interaction.

## 7. Implementation milestones

### MS-P29-1 — `time_of_longitude` pure solver
- Implement the Newton-Raphson solver in `navigation/conics.rs`.
- Unit tests:
  - Circular equatorial LEO from `lon = 0`, target `lon = π/2`: time of crossing ≈ (¼ × T_orb) corrected for Earth rotation.
  - Same orbit, target `lon = π`: time ≈ (½ × T_orb) corrected.
  - Retrograde orbit: time-of-crossing sign correct.
  - Hyperbolic trajectory: returns `P29Error::Hyperbolic`.
  - Forced non-convergence (e.g., very low Newton iteration cap via a test-only constant): returns `P29Error::NoConvergence`.
  - Closed-loop round-trip: take the converged time, feed it back through `p21_compute_ground_track`, verify the longitude matches the input within solver tolerance.
- **Exit criterion**: solver unit tests pass; no callsite yet.

### MS-P29-2 — P29 program shell
- Implement `programs/p29.rs` with `p29_init` modeled on `p21_init`.
- Register in `PROGRAM_TABLE[29]`.
- Wire Noun 89 input (existing V/N data entry) and Noun 34 output.
- Alarms: `01430` (no CSM state vector), `01431` (hyperbolic), `01432` (no convergence).
- Integration tests in `programs/p29.rs::tests` modeled on `programs/p21.rs::tests`:
  - Full V37 P29 flow with a canned LEO scenario returns a sensible time.
  - Each of the three alarm conditions fires correctly.
  - DSKY shows V06 N34 after successful computation.
- **Exit criterion**: P29 selectable via V37, returns a sensible time for a canned LEO scenario; alarms fire on the three error conditions; existing P21 tests untouched.

## 8. Test strategy

- Unit tests in `navigation/conics.rs::tests` for the solver (closed-form expected values where the orbit geometry is simple).
- Integration tests in `programs/p29.rs::tests` for the program flow and alarms.
- No fixtures, no VirtualAGC capture — algorithm is closed-form and the existing P21 tests give us confidence that the underlying GMST/ECEF helpers are correct.

## 9. GitHub issue seed

Reuses the existing `navigation` label. No new labels needed.

| Title | Labels |
|---|---|
| P29 (Time-of-Longitude) — implementation tracking | `navigation`, `milestone` |
| MS-P29-1: `time_of_longitude` solver in `navigation/conics.rs` | `navigation`, `milestone`, `enhancement` |
| MS-P29-2: P29 program shell + DSKY wiring | `navigation`, `milestone`, `enhancement` |

Both milestone issues note `Blocked by #18` explicitly. They are not actionable until MS-F1 lands.
