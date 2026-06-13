# Specification: `programs/p29` Module — Time-of-Longitude (P29)

**Status**: Approved for implementation (supersedes `specs/p29-plan.md`)
**Module path**: `agc-core/src/programs/p29.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**Companion program**: P21 (Ground-Track Determination) — see `specs/p21_p22-spec.md`
**Conics reference**: `specs/conics-spec.md` — `time_of_longitude`, `P29Error`, `TimeOfLongitudeResult`
**State-vector reference**: `specs/state-vector-spec.md` — `inertial_to_earth_fixed`, `met_to_gha`
**Kepler reference**: `specs/kepler-spec.md` — `kepler_step` (universal-variable propagator)
**V/N reference**: `specs/v_n-spec.md` — `noun_89_commit_p29_target`, `vn.crew_p29_target`
**AGC source files**:
- `Comanche055/P20-P25.agc` — P29 entry sequence
- `Comanche055/LAT-LONG_SUBROUTINES.agc` — lat/lon conversion

---

## 1. Purpose and Scope

P29 is **Time-of-Longitude** — the inverse of P21:

| Direction | Program |
|---|---|
| GET → lat / lon / alt | P21 |
| lon → GET, lat, alt | P29 |

Given a crew-entered target geographic longitude, P29 propagates the
CSM state vector forward in time and Newton-iterates until the
ground-track crosses that longitude. The Mission Elapsed Time at the
crossing is displayed on the DSKY (Verb 06 Noun 34, formatted as
hours / minutes / seconds×100).

Mission context: **pass-prediction utility** — useful for ground-station
contact-window planning and landing-site/longitude crossing prediction.
Mission Control normally supplies these values via voice; P29 is the
AGC's autonomous fallback.

P29 makes no actuator commands and does not modify any navigation
state — it is a one-shot display-only computation triggered by a crew
request.

### What this module provides

- `P29_MAJOR_MODE: u8 = 29`.
- `P29_PRIORITY: JobPriority = 7` — same tier as P21.
- `p29_init(state)` — entry point registered in `PROGRAM_TABLE[29]`.
  Sets the major mode and flashes V25 N89 to prompt the crew for the
  target longitude.
- `p29_compute_and_display(state)` — runs the time-of-longitude solver
  and writes the result to V06 N34 (or raises an alarm).

### What this module does NOT provide

- Multiple-crossing selection. The solver returns the **next** crossing
  after the state-vector epoch; the crew cannot request the Nth
  crossing. This is the same constraint as the original AGC code.
- Lat / alt readout at the crossing point. The solver returns these
  in `TimeOfLongitudeResult` but P29 displays only the time (N34); the
  lat/alt fields are available to tests and future enhancements.
- An iterative refinement based on crew input — P29 is one-shot. Crew
  re-entry of a new N89 retriggers the solver via the noun-89 commit
  handler.

---

## 2. AGC Background

The historical CMC P29 used Newton's method on time to solve
`longitude(t) − target_longitude = 0`, propagating the state with the
shared `KEPRTN` universal-variable Kepler propagator. The Rust port
follows the same approach but lives in two pieces:

- The pure-math solver `navigation::conics::time_of_longitude` does the
  Newton iteration on `kepler_step` outputs.
- The program shell `programs::p29` handles DSKY prompting, alarm
  raising, and the HMS conversion of the result.

---

## 3. Algorithm (pure-math solver)

The solver lives in `agc-core/src/navigation/conics.rs`. P29's behaviour
is determined by it; the full algorithm spec is in `specs/conics-spec.md`,
and this section captures only the interface and the error categories.

**Signature**:

```rust
pub fn time_of_longitude(
    csm_pos: Vec3,
    csm_vel: Vec3,
    epoch_s: f64,
    target_lon_rad: f64,
    gha_epoch_rad: f64,
) -> Result<TimeOfLongitudeResult, P29Error>;

pub struct TimeOfLongitudeResult {
    pub time_of_crossing_s: f64,
    pub lat_rad: f64,
    pub alt_m: f64,
}

pub enum P29Error {
    Hyperbolic,
    NoConvergence,
    ZeroAngularMomentum,
}
```

**Method** (Newton-Raphson on time):

1. Compute the current ground-track longitude
   `lon_now = inertial_to_earth_fixed(csm_pos, met_to_gha(epoch_s, gha_epoch_rad))`.
2. Compute the orbital period
   `T_orb = orbital_period(elements, MU_EARTH)`.
3. Compute the longitude-drift rate
   `dlon/dt ≈ -Ω_E + 2π / T_orb` (prograde) — `Ω_E` is `OMEGA_EARTH`.
4. Initial guess
   `t₀ = epoch_s + (target_lon - lon_now) / (dlon/dt)`, wrapped into
   `[epoch_s, epoch_s + T_orb]`.
5. Newton iterations:
   - Propagate with `kepler_step`.
   - Rotate to ECEF, extract `lon(t)`.
   - Update `t ← t - (lon(t) - target_lon) / (dlon/dt(t))` where the
     derivative is recomputed each step from the propagated state.
   - Converge when `|lon(t) - target_lon| < 1e-5 rad` (≈ 100 m at the
     equator).
   - Iteration cap: 20. Failure → `P29Error::NoConvergence`.

**Error categories** mapped to P29 alarms (see §4).

---

## 4. Program Alarms

| Code | Octal | Trigger |
|---|---|---|
| 0o01430 (`ALARM_P29_NO_CSM_SV`) | 1430 | `state.csm_state.epoch == 0` at compute time (no valid SV loaded), or the noun-89 commit fired without a staged target (defensive). |
| 0o01431 (`ALARM_P29_HYPERBOLIC`) | 1431 | Solver returned `P29Error::Hyperbolic` (no orbital period). |
| 0o01432 (`ALARM_P29_NO_CONV`) | 1432 | Solver returned `P29Error::NoConvergence` **or** `P29Error::ZeroAngularMomentum` (the latter is a degenerate orbital-motion input mapped to the same crew-visible category). |

On any alarm the display is reset to `V06 N34 / 0 / 0 / 0`, the
`alarm.lit` flag is set, and `flashing = false`.

---

## 5. Functional Requirements

### 5.1 `p29_init`

1. Set `state.major_mode = 29`, `state.dsky.prog = 29`.
2. Clear any stale staged N89 target:
   `state.vn.crew_p29_target = None`.
3. Set `state.dsky.verb = 25` (load), `state.dsky.noun = 89` (target
   geodetic point), `r[0..3] = 0.0`, `flashing = true` (flashing prompt).
4. Return `P29_PRIORITY` (7).

The flashing V25 N89 cues the crew to load R1 = latitude (°,
informational), R2 = longitude (°, consumed), R3 = altitude (m,
informational).

### 5.2 `p29_compute_and_display`

Invoked from `services::v_n::noun_89_commit_p29_target` whenever the
crew commits N89 with `state.major_mode == 29`.

1. **Precondition check** — if `state.csm_state.epoch == 0`, raise
   alarm 0o01430 and return.
2. **Stage check** — if `state.vn.crew_p29_target.is_none()`, raise
   alarm 0o01430 (defensive — should not happen from the commit path).
3. Read `target_lat_deg, target_lon_deg, target_alt_m = vn.crew_p29_target.unwrap()`.
   Convert `target_lon_rad = target_lon_deg · π / 180`.
4. Call
   `time_of_longitude(csm_state.position, csm_state.velocity, epoch_s, target_lon_rad, gha_epoch_rad)`.
5. Branch on the result:
   - `Ok(result)`: format `result.time_of_crossing_s` as
     `(h, m, s × 100)`; write to `r[0..3]`. Set `verb = 6`, `noun = 34`,
     `flashing = false`. **No alarm.**
   - `Err(P29Error::Hyperbolic)`: raise alarm 0o01431 (via
     `raise_alarm`).
   - `Err(P29Error::NoConvergence)` **or** `Err(P29Error::ZeroAngularMomentum)`:
     raise alarm 0o01432.

`raise_alarm` resets the display (V06 N34 / 0 / 0 / 0, flashing = false)
and sets `alarm.code = code`, `alarm.lit = true`.

### 5.3 HMS conversion (display formatting)

```text
total_s = result.time_of_crossing_s         // absolute GET in seconds
h       = floor(total_s / 3600)
rem     = total_s − h · 3600
m       = floor(rem / 60)
s       = rem − m · 60
r[0]    = h as f32                          // hours
r[1]    = m as f32                          // minutes
r[2]    = (s · 100) as f32                  // seconds × 100 (cs precision)
```

(The `× 100` on the seconds field follows the AGC's Noun 34 layout, in
which R3 carries seconds in centi-second units to fit five decimal
digits.)

---

## 6. PROGRAM_TABLE Registration

```rust
PROGRAM_TABLE[29] = Some(p29::p29_init);
```

The noun-89 commit handler is registered in `services::v_n` and routes
into `p29_compute_and_display` when `state.major_mode == 29`. See
`specs/v_n-spec.md`.

---

## 7. Restart Protection

P29 has no restart group. The staged crew target lives in
`state.vn.crew_p29_target` and would survive a restart in the current
implementation; whether to preserve it across restart is left to a
future iteration. After restart the FRESH START path enters P00 and the
crew must re-enter P29.

---

## 8. Transitions

### Into P29

| Trigger | Source |
|---|---|
| Crew `V37 E 29 E` | V37 handler → `PROGRAM_TABLE[29]` |

### Out of P29

P29 has no continuation after `p29_compute_and_display` writes the
result; the crew typically V37 into another program. The N89 commit is
"sticky" — the crew can re-enter a different longitude and the solver
re-runs.

---

## 9. Test Cases

The implementation in `agc-core/src/programs/p29.rs::tests` provides:

| ID | What is verified |
|---|---|
| TC-P29-INIT-1 | `p29_init` sets `major_mode = 29`, `PROG = 29`, flashing `V25 N89`, and clears any stale staged target. |
| TC-P29-FLOW-1 | Nominal LEO solve: with a circular 300 km equatorial orbit, target lon 30 °E, `p29_compute_and_display` returns no alarm, sets `verb = 6`, `noun = 34`, and the reconstructed GET falls in the expected band (1 000 – 10 000 s ahead of epoch). |
| TC-P29-FLOW-1B | The same scenario but routed through the V/N noun-89 commit path — confirms end-to-end keystroke → compute integration. |
| TC-P29-FLOW-2 | Alarm 0o01430 fires when `csm_state.epoch == 0` (fresh-start sentinel). |
| TC-P29-FLOW-3 | Alarm 0o01431 fires when the trajectory is hyperbolic (v > v_escape). |
| TC-P29-FLOW-4 | Alarm 0o01432 fires when the solver hits the no-convergence / zero-angular-momentum branch (radial velocity input). |
| TC-P29-FLOW-5 | DSKY `prog` reads 29 after `p29_init`. |

Additional pure-solver tests (`navigation::conics::tests`) cover the
quarter-orbit and half-orbit reference cases.

---

## 10. Spec Quality Checklist

- [x] AGC source file referenced (`P20-P25.agc`).
- [x] Algorithm interface documented; full algorithm in `conics-spec.md` (§3).
- [x] All state fields touched by `p29_init` and `p29_compute_and_display` enumerated (§5).
- [x] Alarms documented with octal values (§4).
- [x] HMS conversion documented (§5.3).
- [x] PROGRAM_TABLE and V/N routing documented (§6).
- [x] Test coverage summarised (§9).
- [x] Plan superseded — `specs/p29-plan.md` removed in the same commit (§Status).
