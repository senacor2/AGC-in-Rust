# Specification: Milestone 4 Phase 6 — Book-keeping Programs (P01/P02/P06/P15/P47)

**Status**: Approved for implementation (Milestone 4 Phase 6)
**Module paths**:
- `agc-core/src/programs/p01_p02.rs`
- `agc-core/src/programs/p06.rs`
- `agc-core/src/programs/p15.rs`
- `agc-core/src/programs/p47.rs`

**AGC source files**:
- `Comanche055/PRELAUNCH_INITIALIZATION.agc` — P01, P02 entry
- `Comanche055/P11.agc` — P11/P15 block
- `Comanche055/FRESH_START_AND_RESTART.agc` — P06 power-down path
- `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` — P47 thrust monitor

---

## 1. Purpose

Phase 6 closes out Milestone 4 with the remaining non-rendezvous,
non-entry P-codes that are thin sequencing or monitor wrappers over
services already built. None of these introduce new math or state; all
exercise the existing Executive, SERVICER, DAP, and conics primitives.

Programs in scope:

| Code | Name                                   | Role                                     |
|------|----------------------------------------|------------------------------------------|
| P01  | Pre-launch IMU initialization          | Cages the platform, clears alignment    |
| P02  | Gyrocompassing                         | Establishes platform azimuth on the pad  |
| P06  | CMC Power-down                         | Quiesces SERVICER/DAP, sets stby         |
| P15  | TLI (Trans-Lunar Injection) monitor    | Passive EI-orbit monitor, like P11       |
| P47  | Thrust monitor                         | Displays SERVICER-integrated ΔV/cycle    |

---

## 2. `P01` — Pre-launch IMU Initialization

### Actions

1. `major_mode = 1`, `dsky.prog = 1`.
2. `imu_alignment_state = Caged`.
3. `dsky.verb = 6`, `dsky.noun = 68` (pre-launch status cue).
4. `dsky.flashing = false`.
5. Return `PRIORITY = 3`.

### Alarms
None.

---

## 3. `P02` — Gyrocompassing

### Actions

1. **Preconditions**: `imu_alignment_state == Caged`. Otherwise raise
   alarm 235 (gyrocompass from wrong state) and still advance the
   major mode.
2. Simulate successful gyrocompass: `imu_alignment_state = CoarseAligned`.
3. `major_mode = 2`, `dsky.prog = 2`.
4. `dsky.verb = 6`, `dsky.noun = 68`.
5. Return `PRIORITY = 3`.

Note: the real AGC P02 runs the gyrocompass loop continuously (the
stable member aligns to local horizontal + earth-rotation vector over
several minutes). We model it as an instantaneous transition because
there is no HAL earth-rate source yet; the state-machine transition is
the contract that later milestones will build on.

### Alarms
- **235**: P02 invoked from a non-Caged alignment state.

---

## 4. `P06` — CMC Power-down (Standby)

### Actions

1. `stop_servicer(state)` — cancels the SERVICER Waitlist task.
2. `dap_stop(state)` — transitions DAP to `Off` and clears jet commands.
3. `state.pending_maneuver = None` — no coast-phase targeting state.
4. `state.servicer_exit = None` — no hooks remain.
5. `state.burn.burn_active = false`, `state.engine_thrusting = false`.
6. `dsky.stby = true` (standby indicator).
7. `dsky.prog = 6`, `major_mode = 6`.
8. `dsky.verb = 37`, `dsky.noun = 0` (awaiting V37 to wake up).
9. Return `PRIORITY = 1` (lowest).

### Alarms
None.

---

## 5. `P15` — TLI Monitor

### Actions

Passive monitor that refreshes DSKY V16N44 (apogee / perigee /
half-period) from the current `csm_state`, identical to P11 except the
major mode is 15 and the frame assertion is still EarthInertial (the
TLI burn is computed against Earth-centred elements).

1. If `csm_state.frame != EarthInertial`: raise alarm 236 and return.
2. `major_mode = 15`, `dsky.prog = 15`, `dsky.verb = 16`, `dsky.noun = 44`.
3. `dsky.flashing = false`.
4. Install `p15_servicer_exit` as the SERVICER exit hook.
5. Run one immediate update (`p15_update`), which delegates to the same
   `sv_to_elements → apoapsis/periapsis/orbital_period` pipeline P11 uses.
6. Return `PRIORITY = 6`.

### Alarms
- **236**: `csm_state.frame != EarthInertial`.
- **237**: Current trajectory is hyperbolic (post-TLI happy path — in
  that case the display would need a different noun; for Phase 6 we
  treat hyperbolic as an alarm and leave the display untouched).

Implementation note: P15 and P11 share almost all their code. Rather
than duplicate, P15 delegates directly to the existing P11 compute
routine and only substitutes its own `major_mode` / `dsky.prog` values.

---

## 6. `P47` — Thrust Monitor

### Actions

Passive monitor that displays the inertial delta-V the SERVICER
integrated in the most recent cycle. Intended for crew verification of
a non-nominal or uncommanded thrust event.

1. `major_mode = 47`, `dsky.prog = 47`, `dsky.verb = 16`, `dsky.noun = 83`
   (delta-V components).
2. Install `p47_servicer_exit` which copies
   `state.servicer_last_dv_inertial[0..3]` into `dsky.r[0..3]` (as f32).
3. Run one immediate update.
4. Return `PRIORITY = 6`.

### Alarms
None.

---

## 7. Test Cases

### P01
- **TC-P01-1**: `init_p01` sets major_mode = 1 and cages the platform.
- **TC-P01-2**: `init_p01` from FineAligned forces Caged regardless of
  prior alignment state.

### P02
- **TC-P02-1**: `init_p02` from Caged transitions to CoarseAligned,
  major_mode = 2, no alarm.
- **TC-P02-2**: `init_p02` from FineAligned raises alarm 235 but still
  advances major_mode.

### P06
- **TC-P06-1**: `init` clears servicer_exit, sets dap_state.mode = Off,
  clears pending_maneuver, sets dsky.stby = true, major_mode = 6.
- **TC-P06-2**: `init` sets PRIORITY = 1 and clears burn_active.

### P15
- **TC-P15-1**: `init` on circular LEO EarthInertial state sets
  major_mode = 15 and populates N44 with ≈ 400 km apogee/perigee.
- **TC-P15-2**: `init` on MoonInertial raises alarm 236.
- **TC-P15-3**: Hyperbolic post-escape trajectory raises alarm 237.

### P47
- **TC-P47-1**: `init` sets major_mode = 47 and dsky.noun = 83.
- **TC-P47-2**: After staging `servicer_last_dv_inertial = [1.5, -0.7, 0.3]`
  and calling `p47_servicer_exit`, the DSKY registers reflect those values.
