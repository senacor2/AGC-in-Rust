# Specification: `programs/p40_p41` Module — SPS and RCS Thrusting Programs

**Status**: Approved for implementation
**Module path**: `agc-core/src/programs/p40_p41.rs`
**Architecture reference**: `docs/architecture.md` §7.2 (P-programs), §8 (DAP), §9 (SERVICER)
**Maneuver reference**: `specs/maneuver-spec.md` — `BurnState`, `burn_init`, `burn_update`, `is_burn_complete`
**DAP reference**: `specs/dap-spec.md` §3 (`dap_init`, modes), §14 (mode transitions)
**SERVICER reference**: `specs/average-g-spec.md` §5 (start_servicer, servicer_exit hook)
**Targeting reference**: `specs/targeting-spec.md` — `Maneuver` type produced by P30/P31/P37
**AGC source files**:
- `Comanche055/P40-P47.agc` — P40 SPS and P41 RCS entry sequences
- `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc` — steering and cutoff logic
- `Comanche055/SERVICER207.agc` — SERVICER exit hook pattern

---

## 1. Purpose and Scope

P40 and P41 are the two thrusting-execution programs for the CSM. Both
consume a `Maneuver` previously deposited in `state.pending_maneuver` by
a targeting program (P30 External ΔV, P31/P34 rendezvous, or P37 Return
to Earth), translate it into a live `BurnState`, and steer the vehicle
through the burn until delta-V cutoff.

They differ in the actuator used:

- **P40** (SPS thrusting): uses the Service Propulsion System — the main
  engine. Requires Maneuver/TVC DAP modes, SPS enable discretes, and the
  thrust-vector-control (TVC) loop. Used for all burns ≳ 0.5 m/s.

- **P41** (RCS thrusting): uses only the Reaction Control System
  thrusters. No SPS, no TVC, no gimbal motion. AttitudeHold DAP mode
  with the RCS-logic layer accumulating pulses toward the delta-V
  target. Used for small trim burns (≲ 0.5 m/s) and ullage-only
  maneuvers.

Both programs are **sequencing wrappers** — the actual burn loop lives
in `guidance::maneuver` (`burn_init`, `burn_update`, `is_burn_complete`)
and the SERVICER (`services::average_g`). P40/P41 set up the hooks and
initial DAP mode, then hand off to the asynchronous SERVICER/DAP loops.

### What this module provides

- `P40_MAJOR_MODE = 40`, `P41_MAJOR_MODE = 41`.
- `PRIORITY: JobPriority = 12` — higher than the targeting programs
  (which ran as background jobs) because a burn is a real-time activity.
- `init_p40`, `init_p41` — entry points registered in `PROGRAM_TABLE`.

In addition, `guidance::maneuver` gains:

- `burn_servicer_exit(state: &mut AgcState)` — the SERVICER exit hook
  that both P40 and P41 install. Called by `servicer_task` each 2-second
  cycle. Reads the inertial delta-V just integrated by the SERVICER from
  the new `AgcState::servicer_last_dv_inertial` staging field, calls
  `burn_update`, and checks `is_burn_complete`. On completion, clears
  `burn.burn_active`, clears `engine_thrusting`, and drops the
  servicer_exit hook.

### What this module does NOT provide

- Interactive N40/N42 crew go/no-go sequencing (V50 prompts). Milestone 5.
- Average-G integration of thrust-time-series — that is already in
  `services::average_g::servicer_task`.
- TVC lead-lag filter tuning — lives in `control::tvc` (done in MS3).
- SPS enable/disable discretes — those are HAL calls in the ISR shim;
  P40 only sets the `state.engine_thrusting` staging flag.
- Ullage detection and auto-ignition sequencing. Phase 3 assumes the
  crew has already performed the pre-ignition checklist manually and
  that the program is being called "at ignition".

---

## 2. Program Alarms

| Code | Trigger                                                      |
|------|--------------------------------------------------------------|
| 224  | `pending_maneuver` is `None` on entry.                       |
| 225  | `pending_maneuver.tig` lies in the past (TIG slipped).       |
| 226  | `pending_maneuver.delta_v` magnitude is below threshold (< 0.05 m/s). |
| 227  | P40 invoked but burn magnitude < 0.5 m/s (should use P41).   |
| 228  | P41 invoked but burn magnitude ≥ 0.5 m/s (should use P40).   |

---

## 3. `init_p40` — SPS Burn Setup

On entry the function must:

1. **Validate `pending_maneuver`** — if `None`, raise alarm 224 and
   return without further state changes.
2. **Validate TIG** — `pending_maneuver.tig >= state.time`; otherwise
   alarm 225.
3. **Validate magnitude** — `|delta_v| >= 0.05 m/s`; otherwise alarm 226.
4. **Check SPS regime** — `|delta_v| >= SPS_MIN_DV = 0.5 m/s`; otherwise
   alarm 227 (should have used P41).
5. **Transfer maneuver to `burn`**: `state.burn = burn_init(maneuver)`.
6. **Consume `pending_maneuver`**: `state.pending_maneuver = None`.
7. **Install SERVICER exit hook**:
   `state.servicer_exit = Some(burn_servicer_exit)`.
8. **Start SERVICER** if not already: `start_servicer(state)`.
9. **Initialise DAP to `Maneuver` mode** via `dap_init` with
   `commanded_attitude` derived from `burn_attitude` (rotation matrix
   columns projected to Euler-like triplet — simplified here: store the
   first column of `burn_attitude` as the attitude target and let DAP
   settle into the direction. A full small-angle attitude projection
   belongs in a later refinement).
10. **Do not enable the engine yet** — `state.engine_thrusting` stays
    `false`. Ignition is triggered when DAP reports convergence to
    burn_attitude and TIG is reached. That trigger (at steady-state) is
    out of Phase 3 scope — a Milestone 5 concern.
11. **DSKY**: set `prog = 40`, `verb = 6`, `noun = 40` (burn status:
    TGO, accumulated DV, remaining DV).
12. Set `major_mode = 40`.
13. Return `PRIORITY`.

### Post-conditions (verifiable in tests)

- `state.burn.target_dv_inertial` equals the input maneuver's
  `delta_v.0`.
- `state.burn.burn_active == true`.
- `state.pending_maneuver == None`.
- `state.servicer_exit == Some(_)`.
- `state.dap_state.mode == DapMode::Maneuver`.
- `state.major_mode == 40`.
- `state.dsky.prog == 40`, `dsky.noun == 40`.
- On alarm 224: no other state fields are touched.

---

## 4. `init_p41` — RCS Burn Setup

Same structure as P40 with these differences:

- Step 4: check `|delta_v| < SPS_MIN_DV = 0.5 m/s`. Otherwise alarm 228.
- Step 9: `dap_init(state, DapMode::AttitudeHold)` — no TVC, no
  maneuver ramp. The RCS logic layer will null residual error by
  firing jets directly.
- Step 10: `state.engine_thrusting` stays `false` for all of P41 (no
  SPS engagement ever).
- Step 11: `prog = 41`, `noun = 40` (same burn display).

All other validation (224, 225, 226) and post-condition requirements are
identical.

---

## 5. `burn_servicer_exit` — SERVICER Exit Hook

Lives in `guidance::maneuver`. Signature: `fn(&mut AgcState)`.

Behaviour:

1. If `!state.burn.burn_active`, return immediately. (The hook may be
   called one more time after cutoff before P40 clears it.)
2. Read the inertial delta-V the SERVICER just integrated from
   `state.servicer_last_dv_inertial`.
3. Call `burn_update(&mut state.burn, measured_dv, 2.0)`.
4. Check `is_burn_complete(&state.burn, BURN_CUTOFF_TOLERANCE_MS)` with
   tolerance `0.3 m/s`.
5. If complete:
   - Set `state.burn.burn_active = false`.
   - Set `state.engine_thrusting = false`.
   - Clear the hook: `state.servicer_exit = None`.
   - Transition DAP to `AttitudeHold` so the vehicle continues to hold
     orientation after cutoff (post-burn hold).

### New AgcState staging field

```rust
/// Inertial-frame delta-V integrated by the SERVICER during the most
/// recent 2-second cycle. Written by `servicer_task` just before it
/// calls `state.servicer_exit`. Read by `burn_servicer_exit` during a
/// P40/P41 burn. Units: m/s.
pub servicer_last_dv_inertial: Vec3,
```

Initialised to `[0.0; 3]` in `AgcState::new`. Populated by
`servicer_task` after Step 6 (REFSMMAT rotation) and before Step 9 (exit
hook call) of the SERVICER pipeline.

---

## 6. Test Cases

### TC-P40-1: `init_p40` with no pending_maneuver raises alarm 224.

### TC-P40-2: `init_p40` with a past-TIG pending_maneuver raises alarm 225.

### TC-P40-3: `init_p40` with zero delta-V raises alarm 226.

### TC-P40-4: `init_p40` with sub-SPS delta-V (0.2 m/s) raises alarm 227.

### TC-P40-5: Happy path — 50 m/s prograde burn. Verify post-conditions
listed in §3: burn fields populated, pending_maneuver consumed, DAP in
Maneuver mode, servicer_exit installed, major_mode = 40.

### TC-P41-1: `init_p41` with no pending_maneuver raises alarm 224.

### TC-P41-2: `init_p41` with large delta-V (5 m/s) raises alarm 228.

### TC-P41-3: Happy path — 0.2 m/s trim burn. Verify DAP in AttitudeHold
mode and all other post-conditions.

### TC-MAN-6: `burn_servicer_exit` integrates one cycle.
Build a `BurnState` with target 10 m/s, stage `servicer_last_dv_inertial
= [3.0, 0.0, 0.0]`, call `burn_servicer_exit`, assert accumulated_dv
advanced by 3.0 and burn is not yet complete.

### TC-MAN-7: `burn_servicer_exit` triggers cutoff at completion.
BurnState target 10 m/s, accumulated 9.5, stage 0.8 m/s inertial delta-V
this cycle. After the hook runs, assert `burn_active == false`,
`engine_thrusting == false`, `servicer_exit == None`, and the DAP mode
transitioned to AttitudeHold.

### TC-MAN-8: `burn_servicer_exit` is a no-op when burn_active is false.
State with burn_active = false; call the hook; assert no fields change.
