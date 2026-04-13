# Functional Specification: Guidance Maneuver Execution

**Module path:** `agc_core::guidance::maneuver`  
**Milestone:** 3  
**Status:** Draft

---

## AGC Source References

| Routine | File | Pages | Role |
|---|---|---|---|
| `S40.8` | `docs/agc-source/P40-P47.agc` | 719-722 | Cross-product steering: updates VG, computes angular rate command OMEGAC, sets TGO, checks cutoff |
| `UPDATEVG` | `docs/agc-source/P40-P47.agc` | 699-701 | Per-SERVICER-cycle VG update dispatcher: calls S40.8 or S40.9 depending on flags |
| `STEERING` | `docs/agc-source/P40-P47.agc` | 701-702 | AVEGEXIT handler: calls UPDATEVG after each SERVICER cycle, checks engine-on flag for cutoff |
| `S40.1` | `docs/agc-source/P40-P47.agc` | 709-712 | Initialises VGTIG (initial VG) and UT (initial thrust direction) |
| `S40.13` | `docs/agc-source/P40-P47.agc` | 726-728 | TIMEBURN: sets IMPULSW when TGO < 4 s (switches to attitude-hold) |
| `S41.1` | `docs/agc-source/P40-P47.agc` | 717-718 | Transforms VG from reference to control (body) coordinates for display and DAP feed |
| `KALCMANU` | `docs/agc-source/KALCMANU_STEERING.agc` | 414-419 | Free-fall attitude maneuver generator: integrates CDU angle commands once per second; called during pre-burn attitude maneuver |

---

## Behavior Summary

### Velocity-to-Be-Gained (VG) Vector

VG is the instantaneous required velocity change remaining to complete the maneuver. It starts at the full `VGTIG` vector (computed by S40.1 / `predict_vg_at_ignition`) and is reduced each SERVICER cycle (every 2 seconds) by the measured thrust acceleration:

```
VG_new = VGPREV + DELVREF - BDT
```

Where:
- `VGPREV` — VG from the previous cycle (B+7 m/cs in AGC; m/s in Rust)
- `DELVREF` — measured delta-V from PIPA integration since last cycle (m/cs; m/s in Rust)
- `BDT` — gravity-and-steering correction term (m/cs; m/s in Rust): `BDT = CSTEER * BDT_vec - DELVREF`
  - For external delta-V burns (`CSTEER = 0`): `BDT` reduces to the negative of the last cycle's measured dV; the correction cancels gravity effects.
  - For aimpoint steering (`CSTEER = ECSTEER > 0`): `BDT` adds the S40.9 predicted velocity correction.

AGC: `S40.8` lines: `VLOAD BVSU DELVREF BDT; VAD VGPREV; STORE VG` (page 719-720).

In the Rust port, `BDT` is not separately tracked at the `ManeuverState` level. Instead, `update` receives the measured `thrust_accel` vector integrated over `dt`, and VG is updated as:

```
VG_new = VG_old - thrust_accel * dt
```

This is exact for constant thrust during `dt`, matching the AGC's 2-second predictor-corrector cycle.

### Cross-Product Steering Law

The AGC generates an angular rate command OMEGAC to rotate the vehicle so that the thrust vector tracks VG. The steering law at `XPRODUCT` (S40.8, page 721) is:

```
u_delvg = unit(DELVREF - CSTEER * BDT)    # unit vector along current measured thrust direction
u_vg    = unit(VG)                          # unit vector along desired thrust direction

omega_cmd = KPRIMEDT * (u_delvg × u_vg)    # scaled cross product → rate command
```

In vector notation, the cross product `u_thrust × u_vg` gives an angular rate axis perpendicular to both, with magnitude proportional to the sine of the misalignment angle. For small angles, `sin(θ) ≈ θ`, so this is a linear proportional controller.

The rate gain `KPRIMEDT` is scaled in the AGC at π/8 rad (units: revolutions per second per unit cross product, or equivalently 1/(2·TVCDT) rev/s). The Rust implementation uses a dimensionless gain constant (see Invariants).

For the Rust API, the desired thrust direction is defined as the unit vector along the current VG:

```
desired_thrust_direction = unit(VG)
```

This is the output of `desired_thrust_direction()`. The full rate command (OMEGAC) requires the current measured thrust direction (from DELVREF/PIPA) and is computed in the DAP layer (`control::attitude`), not in this module. The guidance layer's responsibility ends at publishing the desired thrust unit vector.

AGC source: `XPRODUCT` routine in S40.8, page 721-722; constants `KPRIMEDT`, `TWODT` on page 744.

### Engine Cutoff Criteria

S40.8 implements two cutoff paths:

1. **TGO < 4 seconds (IMPULSW path):** When the computed time-to-go (`TGO < FOURSEC = 400 cs`), S40.8 branches to `S40.81` which sets `IMPULSW`, zeroes `OMEGAC` (attitude hold), and clears `STEERSW`. The Waitlist task `ENGINOFF` is set up by `STEERING` to fire at `PIPTIME + TGO`. AGC: `TGO DSU FOURSEC BMN S40.81` (page 721).

2. **Thrust direction reversal alarm (INCRSVG):** If `unit(DELVG) · VG > 0` (the dot product goes positive, meaning thrust is now pointing away from VG — the engine has overshot), alarm code `01407` is raised and steering is terminated. AGC: `BPL INCRSVG` (page 720). In Rust this is mapped to `alarm::raise(AlarmCode::SteeringReversal)` and `cutoff = true`.

3. **Low thrust (LOTHRUST):** If `|DELVREF| * DPB-9 < DVTHRESH`, the engine is considered failed and `STEERSW` is cleared, `OMEGAC = 0`. This is the R40 thrust-fail detection path. In Rust, low thrust is detected by comparing `|thrust_accel| * dt < LOTHRUST_THRESHOLD` in `update`.

The Rust `cutoff` flag in `ManeuverState` is set by any of these three conditions and is monotonic: once set, it never clears.

### Interaction with DAP / Attitude Control

S40.8 writes OMEGAC (angular rate command in stable-member coordinates) each cycle. The TVC DAP reads OMEGAC to drive the SPS gimbal actuators. In Rust:

- `desired_thrust_direction()` returns the unit VG vector in ECI.
- The caller (`control::attitude`) converts this to a body-frame commanded attitude using REFSMMAT and feeds it to the DAP.
- KALCMANU (`KALCMANU_STEERING.agc`) is the equivalent for free-fall (non-thrusting) attitude maneuvers; it generates incremental CDU angle commands (`DELCDUX/Y/Z`) at 1-second intervals to rotate the vehicle to the pre-burn attitude. KALCMANU is called _before_ ignition (via R60CSM) and is not part of `ManeuverState`.

---

## Rust API

```rust
// agc_core::guidance::maneuver

use crate::guidance::targeting::BurnTarget;
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

/// Maneuver execution state.  Tracks VG shrinkage during the burn.
///
/// AGC equivalents: VG (B+7 m/cs), VGPREV, VGDISP, TGO, IMPULSW, STEERSW.
/// AGC source: Comanche055/P40-P47.agc, S40.8 erasable block (page 719).
pub struct ManeuverState {
    /// Current velocity-to-be-gained vector, ECI frame, m/s.
    /// AGC: VG, B+7 m/cs.  Decreases each `update` call.
    pub vg: Vec3,

    /// Current vehicle mass, kg.
    /// AGC: WEIGHT/G, SP B+16 kg.  Decremented by mdot*dt each cycle.
    pub mass: f64,

    /// Mission elapsed time at maneuver start (TIG), centiseconds.
    /// AGC: TIG, DP B+28 cs.
    pub burn_start: crate::types::Met,

    /// Mission elapsed time accumulated since ignition, centiseconds.
    /// AGC: derived from PIPTIME - TIG each cycle.
    pub burn_elapsed: crate::types::Met,

    /// True when the engine cutoff criterion has been satisfied.
    /// Monotonic: once true, never reverts to false.
    /// AGC: IMPULSW (TGO < 4 s path) or STEERSW cleared (reversal / low thrust).
    pub cutoff: bool,
}

/// Construct a new ManeuverState from a burn target and the current state vector.
///
/// Initialises VG to `predict_vg_at_ignition(current, target)`.
/// Sets `burn_elapsed` to zero and `cutoff` to false.
///
/// AGC source: Comanche055/P40-P47.agc, S40.1 initialisation block (pages 709-712).
///
/// Inputs: burn target (TIG, delta_v_lvlh, mass, thrust, isp),
///         current state vector (ECI position, velocity, MET).
pub fn new(target: &BurnTarget, current_state: &StateVector) -> ManeuverState;

/// Update VG and maneuver state given a measured thrust acceleration over time dt.
///
/// Per-cycle update implementing S40.8 VG update rule:
///   VG_new = VG_old - thrust_accel_integrated
///
/// where `thrust_accel` (m/s) is the velocity change delivered by the engine
/// during `dt` seconds, as measured by the PIPA integrator (DELVREF in AGC).
///
/// After updating VG:
/// 1. If |VG_new| < VG_CUTOFF_THRESHOLD_MS (0.3 m/s, see Invariants), set cutoff = true.
/// 2. If `dot(thrust_accel, VG_old) > 0.0` and `dt > 0.0` (thrust reversed past target),
///    set cutoff = true and call `alarm::raise(AlarmCode::SteeringReversal)`.
///    AGC: INCRSVG branch in S40.8, alarm code 01407.
/// 3. If cutoff is already true, this function is a no-op.
///
/// Mass is decremented by `(thrust / isp_ve) * dt` using the stored BurnTarget thrust
/// and exhaust velocity.  Mass is clamped to a minimum of 1.0 kg to avoid division by zero
/// in any downstream Tsiolkovsky recalculation.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 TGOCALC/XPRODUCT, pages 719-722.
///
/// Inputs:
///   state      — current navigation state (used for radius vector in gravity term)
///   dt         — time step, seconds (normally 2.0 s, one SERVICER cycle)
///   thrust_accel — integrated dV from engine during dt, m/s (DELVREF in AGC)
///   target     — burn target providing thrust and exhaust_velocity for mass update
pub fn update(
    &mut self,
    state: &StateVector,
    dt: f64,
    thrust_accel: &Vec3,
    target: &BurnTarget,
);

/// Desired thrust direction as a unit vector in the ECI frame.
///
/// Returns `Some(unit(VG))` while the burn is active.
/// Returns `None` when `cutoff == true` (engine off; no meaningful steering direction).
///
/// This is the guidance output fed to `control::attitude` for gimbal pointing.
/// The DAP converts this ECI vector to body-frame gimbal commands via REFSMMAT.
///
/// AGC source: Comanche055/P40-P47.agc, S40.8 XPRODUCT / `UT` unit vector (page 721).
/// AGC source: Comanche055/P40-P47.agc, S40.1 `UNIT; STOVL UT` (page 711).
///
/// Returns None when cutoff == true.
/// Returns None when |VG| == 0.0 (cannot normalise zero vector).
pub fn desired_thrust_direction(&self, state: &StateVector) -> Option<Vec3>;
```

---

## Constants

| Constant | Value | Unit | Derivation |
|---|---|---|---|
| `VG_CUTOFF_THRESHOLD` | 0.3 | m/s | Approximate equivalent of AGC TGO < 4 s at SPS thrust: 4 s × (F/m) ≈ 0.3 m/s residual; the AGC uses TGO-based cutoff rather than |VG|-based, but |VG| < 0.3 m/s is the effective physical criterion |
| `LOTHRUST_THRESHOLD` | 0.01 | m/s | Equivalent of `|DELVREF| * DPB-9 < DVTHRESH`; S40.8 page 720; used to detect engine failure |
| `MIN_MASS_KG` | 1.0 | kg | Guard against division-by-zero in Tsiolkovsky recalculation |

Note: The AGC uses TGO < 400 cs as the primary cutoff trigger (S40.13 / IMPULSW), not a |VG| threshold. The Rust implementation mirrors this by computing the predicted TGO in `update` using the `burn_duration` function from `guidance::targeting`. When the computed TGO < 4.0 s, `cutoff` is set. The |VG| < 0.3 m/s guard is a secondary safety net.

---

## Scale Factors

| AGC register | AGC scale | Rust representation |
|---|---|---|
| `VG`, `VGPREV` | vector B+7 m/cs | `Vec3` m/s |
| `DELVREF` | vector B+7 m/cs | `Vec3` m/s (input to `update` as `thrust_accel`) |
| `BDT` | vector B+7 m/cs | subsumed into `thrust_accel` for external delta-V burns |
| `TGO` | DP B+28 cs | `f64` seconds |
| `OMEGAC` | DP vector at π/8 rad units | not in this module; produced by `control::attitude` |
| `WEIGHT/G` | SP B+16 kg | `f64` kg in `ManeuverState.mass` |

---

## Invariants

1. `cutoff` is monotonic: `update` may set it to `true`, never to `false`. Once set, all subsequent `update` calls are no-ops and `desired_thrust_direction` returns `None`.
2. `desired_thrust_direction` returns `None` whenever `self.cutoff == true`.
3. `desired_thrust_direction` returns `None` whenever `|self.vg| == 0.0` (zero vector is not normalisable; this cannot be a valid steering direction).
4. `update` with `dt == 0.0` is a no-op: VG does not change, mass does not change, cutoff is not reconsidered. (Protects against zero-length timer ticks during restart recovery.)
5. `mass` is clamped to `MIN_MASS_KG` after each mass decrement. Mass never goes negative.
6. No heap allocation (`Vec`, `Box`, etc.) in any function. All state is inline in `ManeuverState`.
7. No `unwrap` in flight-software code. Division by |VG| in `desired_thrust_direction` is guarded by the zero-check before normalisation.
8. `new` never fails: it calls `predict_vg_at_ignition` which is guaranteed finite for finite inputs. The resulting `ManeuverState` always has a well-defined initial VG.

---

## Relationship to AGC Flags and Scheduler

In the AGC, S40.8 is called via the AVEGEXIT hook — i.e., it runs as part of the 2-second SERVICER cycle. In Rust, the equivalent is the `services::average_g` module calling `maneuver::update` at the end of each integration cycle.

The following AGC flag interactions are relevant:

| AGC flag | Bit | Meaning for maneuver | Rust equivalent |
|---|---|---|---|
| `STEERSW` | FLAG2 bit 11 | 0 = steering disabled (attitude hold) | `maneuver.cutoff == true` |
| `IMPULSW` | FLAG2 bit 9 | 1 = TGO < 4 s, impulsive burn | `maneuver.cutoff == true` |
| `XDELVFLG` | FLAG2 bit 8 | 1 = external delta-V (P30); 0 = aimpoint (P31/P37) | caller selects `predict_vg_at_ignition` vs Lambert result |
| `FIRSTFLG` | internal | S40.9: first pass through VG update | not needed in Rust; handled by `ManeuverState::new` init |
| `ENGONFLG` | FLAG5 bit 7 | 1 = SPS engine on | read from HAL (`hal::engine::is_engine_on()`) by the caller, not by `ManeuverState` |

---

## KALCMANU Interface Note

`KALCMANU_STEERING.agc` implements the _pre-burn_ attitude maneuver, not the powered steering. During free-fall attitude rotation, the CDU angle commands (`DELCDUX/Y/Z`) are generated at 1-Hz rate by `UPDTCALL` / `NEWDELHI` / `INCRDCDU`. The Rust equivalent is `control::attitude::kalcmanu_step()`, which is called from the 1-second Waitlist task — not from `ManeuverState::update`. The boundary is: `guidance::maneuver` handles the powered burn (TIG onward); `control::attitude::kalcmanu_step` handles the pre-burn rotation (P40 calls R60CSM before TIG).

---

## Test Cases

### TC-M1: VG decreases monotonically during a burn
- Setup: `new()` with BurnTarget { delta_v_lvlh = [0, 50, 0] m/s prograde, SPS thrust, mass = 28800 kg }; initial VG magnitude ≈ 50 m/s.
- Action: call `update` 5 times with `thrust_accel = [0, 3.0, 0]` m/s (≈ F/m × 2s per step), `dt = 2.0`.
- Expected: `|vg|` decreases by approximately 3.0 m/s per step; after 5 steps |vg| ≈ 35 m/s; `cutoff == false`.
- Tolerance: ±0.01 m/s per step.
- AGC source: S40.8 VG update loop, page 719-720.

### TC-M2: Cutoff fires when TGO < 4 s
- Setup: same BurnTarget as TC-M1; `mass = 28800 kg`.
- Action: call `update` with `thrust_accel` vectors large enough to reduce |VG| below the 4-second cutoff point. Continue until `cutoff == true`.
- Expected: `cutoff` transitions to `true` exactly once; after that, `desired_thrust_direction` returns `None` and further `update` calls do not change `vg`.
- Purpose: monotonicity and cutoff idempotency.
- AGC source: S40.8 branch `TGO DSU FOURSEC BMN S40.81`, page 721.

### TC-M3: Cross-product steering points thrust along VG when VG is not collinear with velocity
- Setup: `ManeuverState` with VG = [0.0, 50.0, 0.0] m/s (prograde) but current velocity = [7784.0, 0.0, 0.0] m/s (VG is perpendicular to velocity in this frame).
- Call: `desired_thrust_direction(state)`.
- Expected: returned unit vector has `|v| == 1.0` (unit vector); direction = [0.0, 1.0, 0.0] within 1e-6.
- Purpose: `desired_thrust_direction` is purely unit(VG) — not the velocity direction.
- AGC source: S40.1 `UNIT; STOVL UT`, page 711; S40.8 `VG UNIT VXV` direction derivation, page 721.

### TC-M4: Idempotent update for zero dt
- Setup: any valid `ManeuverState` with `cutoff == false`, `vg` = [0.0, 40.0, 0.0].
- Action: call `update(state, dt=0.0, thrust_accel=[0,3,0], target)`.
- Expected: `vg` is unchanged; `mass` is unchanged; `cutoff` is unchanged.
- Purpose: zero time-step must be a no-op (Invariant 4), protects restart recovery.

### TC-M5: Steering reversal alarm
- Setup: `ManeuverState` with `vg = [0.0, 5.0, 0.0]`; engine overshot so `thrust_accel = [0.0, -6.0, 0.0]` (opposite to VG, meaning thrust pointed away from VG, dot product > 0 when using old VG direction).
- Expected: After `update`, `cutoff == true`; `desired_thrust_direction` returns `None`.
- Purpose: validates INCRSVG alarm path.
- AGC source: S40.8 `BPL INCRSVG` branch, page 720; alarm code 01407.

---

## agc-sim Impact

- `MissionState` struct: add `vg_mag_ms: f64` (for display in the Mission State panel), `tgo_s: f64`, `maneuver_active: bool`.
- `SimLog`: emit `".info("Engine cutoff: |VG| = {:.2} m/s", vg_mag)` when `cutoff` transitions to `true`.
- `dsky_terminal.rs`: N85 display already allocated for DVTOTAL, VGX, VGY, VGZ; wire `vg` components from `ManeuverState.vg` to these fields.
- No new DSKY keyboard bindings required.
