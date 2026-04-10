# Specification: `programs/p11` Module — Earth Orbit Insertion Monitor (P11)

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p11.rs`
**Architecture reference**: `docs/architecture.md` §7.2 "Programs for the Command Module"
**Conics reference**: `specs/conics-spec.md` — `state_to_elements`, `apoapsis_altitude_earth`, `periapsis_altitude_earth`, `orbital_period`
**State-vector reference**: `specs/state-vector-spec.md` §2.1 (Frame), §3 (StateVector)
**SERVICER reference**: `specs/average-g-spec.md` §5 (servicer_exit hook)
**AGC source files**:
- `Comanche055/P11.agc` — Earth orbit insertion monitor main body
- `Comanche055/ORBITAL_INTEGRATION.agc` — underlying state propagation

---

## 1. Purpose and Scope

P11 is the Earth Orbit Insertion Monitor. It runs during and after
Saturn V powered ascent, starting automatically at liftoff, and
provides the crew with a continuously updated readout of the current
trajectory's apoapsis altitude, periapsis altitude, and orbital period.
Its role is to let the crew verify that the Instrument Unit has delivered
the spacecraft into the correct parking orbit.

P11 does no targeting and commands no actuators. It is a **passive
monitor** — its only outputs are the three DSKY display registers under
Verb 16 Noun 44 (the classical "Apogee / Perigee / TFF" triplet).

In this port we leverage the already-implemented `navigation::conics`
module rather than reproducing the AGC's HANGLE/REVUP routines.

### What this module provides

- `P11_MAJOR_MODE: u8 = 11`.
- `PRIORITY: JobPriority = 6` — background monitor tier.
- `init(state)` — entry point registered in `PROGRAM_TABLE[11]`.
  Sets major mode, installs the `p11_servicer_exit` hook so the display
  refreshes each SERVICER cycle, and performs an immediate first update.
- `p11_update(state)` — pure recomputation of the DSKY N44 display from
  the current `state.csm_state`. Called by the hook and directly from
  tests.
- `p11_servicer_exit(state)` — the SERVICER exit hook. Thin wrapper
  around `p11_update` that runs each 2-second cycle while P11 is active.

### What this module does NOT provide

- A dedicated display in km vs metres. For test determinism the DSKY
  registers carry metres (f32 precision).
- TFF (Time From Fictitious Perigee). The third register carries
  `orbital_period_seconds / 2` as a simplified proxy (half-period is the
  Apollo crew's rough guide to when they will reach apogee from the
  current position on a circular orbit). The true TFF routine is a
  Milestone 5 concern.
- Handling of hyperbolic trajectories in the display — if the current
  `csm_state` describes a hyperbolic orbit the module raises alarm 229
  ("orbit not elliptic") and leaves the display unchanged.

---

## 2. Program Alarms

| Code | Trigger                                                       |
|------|---------------------------------------------------------------|
| 229  | `csm_state` is hyperbolic (`OrbitalElements::is_hyperbolic`). |
| 230  | `csm_state.frame != Frame::EarthInertial`.                    |

---

## 3. Functional Requirements

### 3.1 `init`

On entry:

1. Assert (alarm 230) that `state.csm_state.frame == Frame::EarthInertial`.
   If the frame is wrong, raise alarm 230 and return `PRIORITY`
   without further state changes.
2. Set `state.major_mode = 11`.
3. Set `state.dsky.prog = 11`.
4. Set `state.dsky.verb = 16` (monitor — continuously updated) and
   `noun = 44` (apogee/perigee/TFF).
5. Set `state.dsky.flashing = false` (no crew input required).
6. Install the servicer exit hook: `state.servicer_exit = Some(p11_servicer_exit)`.
7. Call `p11_update(state)` once immediately so the display reflects
   the current orbit at program selection time.
8. Return `PRIORITY`.

### 3.2 `p11_update`

1. Convert `state.csm_state` to `OrbitalElements` via `sv_to_elements`.
2. If `elements.is_hyperbolic()`, raise alarm 229 and return without
   modifying the display.
3. Compute:
   - `apo_m = apoapsis_altitude_earth(&elements)` — metres.
   - `peri_m = periapsis_altitude_earth(&elements)` — metres.
   - `half_period_s = orbital_period(&elements, MU_EARTH) / 2.0`.
4. Write to DSKY:
   - `dsky.r[0] = apo_m as f32`
   - `dsky.r[1] = peri_m as f32`
   - `dsky.r[2] = half_period_s as f32`

### 3.3 `p11_servicer_exit`

Pass-through to `p11_update`. Used as a `servicer_exit` hook so the N44
display is refreshed every 2 seconds.

---

## 4. Test Cases

### TC-P11-1: `init` on EarthInertial state sets major_mode = 11.
Fresh AgcState with a circular LEO state vector; call `init`; assert
`major_mode == 11`, `dsky.prog == 11`, `dsky.verb == 16`, `dsky.noun == 44`,
`servicer_exit` installed, no alarm.

### TC-P11-2: `init` on MoonInertial state raises alarm 230.
AgcState with a lunar-orbit state vector; call `init`; assert
`alarm.code == 230`, major_mode NOT advanced to 11.

### TC-P11-3: `p11_update` on 400 km circular LEO.
Construct a circular LEO at 6 778 137 m (400 km); call `p11_update`;
assert `r[0] ≈ r[1] ≈ 400_000` m (within 1 m for a perfectly circular
orbit) and `r[2] > 0`.

### TC-P11-4: `p11_update` on elliptic orbit apogee ≈ 1200 km, perigee ≈ 400 km.
Construct a state vector at perigee of a 400×1200 km orbit (vis-viva
initial velocity); call `p11_update`; assert
`r[0] ≈ 1_200_000`, `r[1] ≈ 400_000` within 100 m.

### TC-P11-5: `p11_update` on hyperbolic trajectory raises alarm 229.
Construct a state vector with |v| > escape velocity; call `p11_update`;
assert `alarm.code == 229` and the display is not overwritten.

### TC-P11-6: `p11_servicer_exit` delegates to `p11_update`.
Install the hook via `init`; manually move `csm_state` to a new orbit;
call `p11_servicer_exit`; assert the DSKY reflects the new orbit.
