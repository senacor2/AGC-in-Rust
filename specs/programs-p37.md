# Spec: P37 — Return-to-Earth Targeting

## AGC Source Reference

```
File:     Comanche055/P30-P37.agc
Routines: P30/P31 shared entry (page 636), S31.1 (pages 641-642)
          DELRSPL / AUGEKUGL (pages 643-647, splash-down error display)
File:     Comanche055/CONIC_SUBROUTINES.agc
Routines: LAMBERT / LAMBLOOP / INITV (pages 1296-1300) — Lambert solver
File:     Comanche055/P61-P67.agc
Constant: 400KFT (page 875) — entry interface altitude
```

Note: P37 itself is not a separate labelled routine in the P30-P37.agc file of Comanche055 (the file covers P30/P31 and shared subroutines). The return-to-Earth targeting function in Comanche055 is implemented as a combination of crew-entered TIG + Lambert targeting through the P31 / S31.1 path (AGAIN solver for Lambert), with the special constraint that the target position is the Earth entry interface at 400 kft above the Fischer ellipsoid. The constant `400KFT 2DEC 121920 B-29` (meters) appears at P61-P67.agc page 875.

For the Rust port, P37 is specified as a standalone pure-function module that encapsulates the Lambert solve plus entry-interface constraint, callable by the sim and by P40 preparation logic.

## Behavior Summary

P37 computes the velocity change required to return a spacecraft from cislunar space (lunar orbit or translunar coast) to Earth. It is the targeting program for the trans-Earth injection (TEI) burn.

### Inputs

- **Current state vector**: spacecraft position and velocity in ECI at the present moment.
- **Desired time of arrival** at Earth entry interface (t_arrival, as MET).
- **Entry interface radius**: nominally 400,000 ft = 121,920 m above Earth's geocentric radius. This is the `400KFT` constant from P61-P67.agc page 875.

### Algorithm (S31.1 / Lambert path)

1. **Compute target position**: the Earth entry interface is a sphere of radius `R_EARTH + ENTRY_INTERFACE_M`. The target point on this sphere is specified implicitly by the Lambert solver — P37 solves for any point on the entry sphere that is reachable in `dt = t_arrival - t_now` seconds, typically choosing the short-way transfer.
2. **Lambert solve**: calls `math::lambert::lambert(r_now, r_target, dt, MU_EARTH, TransferDirection::Short)` to find the required velocity `v_required` at the current position.
3. **Compute delta-V**: `delta_v = v_required - v_current` (ECI frame).
4. **Compute burn duration**: calls `guidance::targeting::burn_duration(|delta_v|, SPS_THRUST_N, SPS_VE_MS, mass)`.
5. **Return BurnTarget** with `tig = t_now` (burn immediately, or the caller may set a delayed TIG), `delta_v_lvlh = rotate_to_lvlh(delta_v)`.

### Entry interface constant

The entry interface altitude of 400,000 ft above the Fischer ellipsoid is the standard Apollo entry interface:

```
400KFT   2DEC   121920 B-29    # METERS
```

Source: `Comanche055/P61-P67.agc` page 875. This value (121,920 m) is the Rust constant `ENTRY_INTERFACE_M`.

The target sphere radius is `R_EARTH + ENTRY_INTERFACE_M`. For P37, `R_EARTH` is the mean equatorial radius used in the AGC: 6,373,336 m (from `RTRIAL` in P61-P67.agc page 873: `RPAD + 264643 ft = 20,909,901.57 ft = 6,373,336 m` which is the pad radius; for EI targeting the entry sphere radius is taken as pad radius + 400 kft, but the Lambert solver needs only an approximate radius — within 1% of the true value the trajectory geometry is insensitive to the exact Earth radius). For simplicity, the Rust port uses the standard Earth mean radius `R_EARTH = 6_371_000.0 m` (within 0.03% of the AGC value).

### Target position selection

The Lambert problem requires a specific `r2` position vector. P37 must choose the location on the entry sphere. The AGC approach (via the crew-entered trajectory data or uplinked splash-down target) provides a latitude and longitude for the recovery zone. In the Rust implementation:
- If a splash-down target `(lat, lon)` is available in `AgcState`, `r2` is computed from it.
- If not (no target loaded), the solver targets a point on the entry sphere at the same ecliptic longitude as the current position, displaced inbound along the hyperbolic arrival asymptote. This matches the AGC's default behavior when no specific recovery site is loaded.

### Failure mode

Lambert convergence can fail for:
- Transfer time ≤ 0 (dt ≤ 0).
- Collinear initial and final positions (rare for cislunar geometry, but possible if t_arrival is very short).
- Very long transfer times (> 10 days) causing Stumpff iteration to not converge within MAX_ITERATIONS = 60 bisection steps.

On failure, `solve()` returns `None` and the caller must invoke `alarm::raise(AlarmCode(0x0150))` (no specific Comanche055 alarm code for Lambert failure; use 0x0150 as a guidance alarm analog to the AGC's BAILOUT for navigation failures).

## Rust API

Module path: `agc_core::programs::p37_return`

```rust
/// Entry interface altitude above Earth geocentric radius, metres.
///
/// `400KFT 2DEC 121920 B-29` from Comanche055/P61-P67.agc page 875.
/// Equals 400,000 ft = 121,920 m.
pub const ENTRY_INTERFACE_M: f64 = 121_920.0;

/// Earth mean geocentric radius used for entry sphere, metres.
///
/// Used to compute the entry interface sphere radius:
///   r_ei = EARTH_RADIUS_M + ENTRY_INTERFACE_M
/// Value 6,371,000 m is the IAU mean equatorial radius, within 0.03%
/// of the AGC pad radius from P61-P67.agc page 873 (6,373,336 m).
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Solve the Lambert problem for a return-to-Earth trajectory.
///
/// Given the spacecraft's current state vector (in ECI), a desired arrival
/// time at Earth entry interface, and the entry interface radius, computes
/// the velocity required at the current position to reach the entry sphere
/// in the given time.
///
/// Returns `Some(BurnTarget)` when Lambert converges; `None` on convergence
/// failure (collinear geometry, non-positive dt, or exceeded MAX_ITERATIONS).
/// The caller must raise an alarm and return to P00 on `None`.
///
/// Algorithm:
///   1. dt = t_arrival - current.time() (in seconds). Return None if dt <= 0.
///   2. r_ei = EARTH_RADIUS_M + entry_radius. Choose r2 on the entry sphere.
///   3. Call math::lambert::lambert(r1, r2, dt, MU_EARTH, TransferDirection::Short).
///   4. delta_v_eci = lambert.v1 - current.velocity() (ECI m/s).
///   5. Rotate delta_v_eci to LVLH at current position via LOMAT construction.
///   6. Compute burn_duration with SPS constants and vehicle mass from state.
///   7. Pack into BurnTarget { tig: current.time(), delta_v_lvlh, mass, thrust, isp }.
///
/// Inputs:
///   `current`      — current state vector (ECI, m, m/s, centiseconds).
///   `t_arrival`    — desired MET at Earth entry interface (centiseconds).
///   `entry_radius` — altitude of entry interface above Earth geocentric radius, m.
///                    Nominal: ENTRY_INTERFACE_M = 121_920.0 m.
///
/// Output: `Some(BurnTarget)` or `None`.
///
/// AGC source: Comanche055/P30-P37.agc S31.1 (pages 641-642);
///             Comanche055/CONIC_SUBROUTINES.agc LAMBERT (page 1296).
///             Entry interface constant: Comanche055/P61-P67.agc page 875.
///
/// No side effects. Purely computational. No heap. No panic.
#[must_use]
pub fn solve(
    current: &StateVector,
    t_arrival: Met,
    entry_radius: f64,
) -> Option<BurnTarget>;
```

### Entry sphere target position selection

The internal helper (not part of the public API) chooses `r2` on the entry sphere:

```rust
/// Choose the target position on the entry sphere.
///
/// Uses the spacecraft's current position direction as a first approximation:
/// places r2 on the entry sphere along the inbound radial from the current
/// position (i.e., opposite to the unit position vector at the current time).
/// This matches the AGC's behavior when no specific splash-down target is loaded.
///
/// If `state.splash_target` is Some((lat, lon)), converts lat/lon to an ECI
/// vector at the estimated arrival time (using Earth's rotation rate) and places
/// r2 on the entry sphere at that direction.
fn entry_sphere_target(current: &StateVector, t_arrival: Met, r_ei: f64) -> Vec3;
```

### Scale factors

| Quantity | Rust unit | AGC scale (S31.1) | Notes |
|---|---|---|---|
| Entry interface altitude | m (f64) | B-29 m | `400KFT = 121920 B-29` |
| Transfer time dt | seconds (f64) | — | computed from Met centiseconds |
| Required velocity v1 | m/s (f64) | B+7 m/cs | Lambert output |
| Delta-V ECI | m/s (f64) | B+7 m/cs | `v1 - v_current` |
| Delta-V LVLH | m/s (f64) | `DELVLVC` B+7 m/cs | rotated for BurnTarget |
| Burn duration | seconds (f64) | `TPASS4` B+28 cs | from Tsiolkovsky |

### Restart safety

`solve()` is a pure function with no side effects on `AgcState`. It does not need restart phase protection. If it is invoked as part of a larger P37 program flow (setting up the BurnTarget for P40), the caller (`p37_return::program_enter`) must set a Group 4 restart phase before calling `solve()` and after storing the result.

### Relationship to AGC TPASS4

In S31.1 the AGC computes `TPASS4 = TIG + DELLT4` which is the time of arrival at the target (intercept time). In the Rust spec, `t_arrival` is the equivalent of `TPASS4`. The Lambert solver returns `v1` (required velocity at `r_now`) and `v2` (terminal velocity at `r_ei`).

## Invariants

1. `solve()` has **no side effects**: it does not write to `AgcState`, does not command hardware.
2. Returns `None` for any non-positive `dt = (t_arrival - current.time())`.
3. Returns `None` if the Lambert solver's `LambertResult.converged == false`.
4. The returned `BurnTarget.tig` is set to `current.time()` (burn at present position), not at a future TIG. Callers planning a delayed burn must adjust accordingly.
5. `entry_radius` must be positive. If `entry_radius <= 0.0` or `EARTH_RADIUS_M + entry_radius < |current.position()|`, the target is below the spacecraft and `solve()` returns `None`.
6. No heap allocation. No `unwrap`. No panic path.
7. Uses `math::lambert::lambert` from `agc_core::math::lambert`; must respect that module's invariants (non-collinear vectors, positive dt, positive mu).

## Test Cases

### TC-P37-1: Lunar orbit to Earth return (nominal)
```
Setup:    current.position = [384_400_000.0, 0.0, 0.0] m  (lunar distance)
          current.velocity = [0.0, 1_022.0, 0.0] m/s   (circular lunar orbit speed)
          current.time = Met(0)
          t_arrival = Met::from_secs(259_200)   (72 hours = 3 days return coast)
          entry_radius = ENTRY_INTERFACE_M       (121_920 m)
Action:   let result = p37_return::solve(&current, t_arrival, entry_radius);
Expected: result.is_some() == true
          let bt = result.unwrap();
          |bt.delta_v_lvlh| > 800.0 m/s   (TEI burn is ~900 m/s)
          |bt.delta_v_lvlh| < 1200.0 m/s  (physically bounded)
          bt.tig == current.time()
```

### TC-P37-2: Convergence failure — non-positive transfer time
```
Setup:    Same current state as TC-P37-1.
          t_arrival = current.time()  (dt = 0)
Action:   let result = p37_return::solve(&current, t_arrival, ENTRY_INTERFACE_M);
Expected: result.is_none() == true
```

### TC-P37-3: ENTRY_INTERFACE_M constant matches AGC source value
```
Action:   assert_eq within 1.0 m tolerance
Expected: p37_return::ENTRY_INTERFACE_M ≈ 121_920.0   (400,000 ft in metres)
          (400_000.0 * 0.3048 = 121_920.0 exactly)
```

## agc-sim Impact

- `MissionState` panel: add `tei_dv_ms` field (TEI delta-V magnitude in m/s) populated when P37 `solve()` succeeds.
- `SimLog`: emit `log::info!("P37 TEI solution: |dv|={:.1} m/s, burn_dur={:.1} s", ...)`.
- Scenario `--scenario free` (free-return / trans-Earth injection): after reaching lunar distance, auto-calls `p37_return::solve()` and passes the result to the P40 thrusting program.
- No new DSKY keyboard bindings; V37 N37 dispatches via existing V37 handler.
- The `entry_radius` parameter in the sim is fixed to `ENTRY_INTERFACE_M`; the crew cannot change it via DSKY (it is a pad-loaded constant in the AGC).
