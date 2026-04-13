# Functional Specification: Guidance Targeting

**Module path:** `agc_core::guidance::targeting`  
**Milestone:** 3  
**Status:** Draft

---

## AGC Source References

| Routine | File | Pages | Role |
|---|---|---|---|
| `P30` / `S30.1` | `docs/agc-source/P30-P37.agc` | 635-641 | External delta-V targeting: loads TIG, DELVSLV, propagates state to TIG, rotates delta-V to reference frame |
| `S31.1` | `docs/agc-source/P30-P37.agc` | 641-642 | Lambert / aimpoint targeting: computes required velocity at TIG from a stored target position |
| `P40CSM` | `docs/agc-source/P40-P47.agc` | 684-685 | Main burn program: loads `FENG`, sets `F`, reads `TIG`, calls `S40.1` to compute `VGTIG` and `UT` |
| `S40.1` | `docs/agc-source/P40-P47.agc` | 709-712 | Computes initial VG (velocity-to-be-gained) at TIG and initial thrust direction `UT` |
| `S40.13` | `docs/agc-source/P40-P47.agc` | 726-728 | TIMEBURN: computes predicted burn time (TGO) from VGTIG magnitude, engine thrust, and mass flow |
| Constants | `docs/agc-source/P40-P47.agc` | 689 | `FENG`, `2VEXHUST`, `EMDOT`, `3MDOT` |

---

## Behavior Summary

### Overview

The targeting subsystem answers one question before every powered maneuver: _given a desired velocity change and a time to execute it, what is the initial thrust direction, and how long will the engine burn?_ It is split into two distinct input paths:

**External delta-V (P30 path, `XDELVFLG = 1`):** The crew or Mission Control uplinks the time of ignition (TIG) and a delta-V vector expressed in Local Vertical / Local Horizontal (LVLH) coordinates. The AGC propagates the state vector forward to TIG, rotates the delta-V into reference (ECI-aligned) coordinates, and stores it as `DELVSIN`. This is the primary path for standard deorbit and trajectory correction burns.

**Lambert / aimpoint targeting (P31/S31.1 path, `XDELVFLG = 0`):** Used for the trans-lunar injection return (P37) and abort scenarios. The target position vector (`RTARG`) and time of arrival (`TPASS4`) are stored. S31.1 calls the Lambert solver (`INITVEL`) to find the required velocity at TIG, which becomes `VGTIG` directly.

In both paths, `S40.1` (called from P40CSM) converts the result to an initial velocity-to-be-gained vector `VGTIG` (scaled B+7 m/cs in AGC; m/s in Rust) and a unit thrust direction `UT`.

### Burn-time Prediction (S40.13 / Tsiolkovsky)

`S40.13` predicts engine-on time (TGO) before ignition so the crew can verify it, and to set up the `ENGINOFF` Waitlist task. The calculation applies the Tsiolkovsky rocket equation:

```
m_final = m0 * exp(-|delta_v| / v_exhaust)
burn_time = (m0 - m_final) / mdot
```

Equivalently, in terms of AGC inputs (thrust F, mass flow EMDOT, initial mass WEIGHT/G):

```
v_exhaust = F / EMDOT              (not computed directly; 2VEXHUST is stored)
TGO = (m0 / EMDOT) * (1 - exp(-|VGTIG| / v_exhaust))
```

The AGC `S40.13` uses a piecewise approximation for short burns (< 100 cs) and long burns (> 600 cs) to avoid interpreter overflow; the Rust implementation uses the exact closed-form equation valid for all burn durations.

The `IMPULSW` flag is set when TGO < 400 cs (4 seconds); in that regime the burn is treated as impulsive (attitude-hold, no cross-product steering). The Rust equivalent is the `cutoff` flag raised in `ManeuverState` (see `guidance-maneuver.md`).

---

## SPS Constants

Sourced from `docs/agc-source/P40-P47.agc` (page 689) and confirmed in `docs/agc-reference-constants.md`.

| Constant | AGC symbol | AGC encoding | SI value | Rust name |
|---|---|---|---|---|
| SPS thrust | `FENG` | `2DEC 9.1188544 B-7` (M-Newtons/E4) | 91 188.544 N | `SPS_THRUST_N` |
| Exhaust velocity (×2) | `2VEXHUST` | `2DEC 63.020792 B-7` (m/cs) | 3 151.0396 m/s | `SPS_VE_MS` |
| Specific impulse | derived | `SPS_VE_MS / G0` | 321.3 s | `SPS_ISP_S` |
| SPS mass flow | `EMDOT` | pad-loaded, B+3 kg/cs | ~29.0 kg/s | not a fixed constant |
| RCS ullage thrust | `FRCS2` | `2DEC .087437837 B-7` | 874.378 N (4-jet) | — |

Note: `2VEXHUST` encodes _twice_ the exhaust velocity. When used in the TGO computation (TGOCALC), it appears in the denominator of a `DDV` (double divide), which effectively divides by `v_exhaust` rather than `2 * v_exhaust`. The Rust API receives the true exhaust velocity `v_e = 2VEXHUST / 2 = 3151.04 m/s`.

The standard sea-level gravitational acceleration used for Isp derivation is g₀ = 9.80665 m/s².

---

## Coordinate Frames

| Frame | AGC name | Description |
|---|---|---|
| Reference / ECI | "ref coords" | Earth-centred inertial; REFSMMAT aligns stable-member axes to this frame |
| Local Vertical / Local Horizontal | LV coords, "LVC" | Body-aligned: +X = radial outward, +Y = velocity direction, +Z = orbit-normal |
| Stable-member (SM) | SM | IMU platform frame, related to ECI via REFSMMAT |

`DELVSLV` (P30 input) is in LVLH. `S30.1` rotates it to ECI using the Local Orientation Matrix (`LOMAT`) computed by `LOMAT` routine at TIG to give `DELVSIN`. `VGTIG` (S40.1 output) is in ECI/reference frame.

---

## Rust API

```rust
// agc_core::guidance::targeting

use crate::navigation::state_vector::StateVector;
use crate::types::{Met, Vec3};

/// Burn targeting parameters for a single SPS or RCS maneuver.
///
/// AGC equivalents: TIG (B+28 cs), DELVSIN / DELVSLV (B+7 m/cs vector),
/// WEIGHT/G (B+16 kg), F (B+7 M-Newtons), EMDOT (B+3 kg/cs).
///
/// AGC source: P40-P47.agc S40.1 erasable initialisation block (page 709-710).
pub struct BurnTarget {
    /// Time of ignition, mission elapsed time.
    /// AGC: TIG, DP B+28 centiseconds.
    pub tig: Met,

    /// Desired delta-V in LVLH frame, m/s.
    /// AGC: DELVSLV (B+7 m/cs); rotated to ECI by S30.1 → DELVSIN.
    pub delta_v_lvlh: Vec3,

    /// Vehicle mass at TIG, kg.
    /// AGC: WEIGHT/G, SP B+16 kg.
    pub mass: f64,

    /// Engine thrust, Newtons.
    /// AGC: F (FENG for SPS = 91188.544 N, stored B+7 M-Newtons/E4).
    pub thrust: f64,

    /// Exhaust velocity, m/s.
    /// AGC: 2VEXHUST / 2 = 3151.0396 m/s (see 2DEC 63.020792 B-7, page 689).
    pub isp: f64,   // stored as exhaust velocity (v_e = Isp * g0)
}

/// Predicted burn duration via the Tsiolkovsky rocket equation.
///
/// Returns `Some(seconds)` for valid inputs; `None` when mass <= 0.0 or thrust <= 0.0
/// (these are alarm conditions in the AGC — S40.13 assumed valid WEIGHT/G and F).
///
/// Formula: t_burn = (m0 / mdot) * (1 - exp(-|delta_v| / v_e))
///   where mdot = thrust / v_e.
///
/// For delta_v_mag == 0.0, returns Some(0.0).
///
/// AGC source: Comanche055/P40-P47.agc, S40.13 routine, pages 726-728.
/// The AGC uses a piecewise linear approximation; this Rust function uses the
/// exact closed-form expression.
///
/// Inputs: delta_v_mag in m/s, thrust in N, exhaust_velocity in m/s, mass in kg.
/// Output: burn duration in seconds.
pub fn burn_duration(
    delta_v_mag: f64,
    thrust: f64,
    exhaust_velocity: f64,
    mass: f64,
) -> Option<f64>;

/// Velocity-to-be-gained vector at TIG in the reference (ECI) frame.
///
/// For P30 external delta-V burns (`XDELVFLG = 1`):
///   Propagates `current` state forward to `target.tig`, computes the LVLH-to-ECI
///   rotation matrix at TIG, rotates `target.delta_v_lvlh` to ECI.
///   This mirrors S30.1 (DELVSLV → DELVSIN via LOMAT).
///
/// For Lambert / aimpoint burns (`XDELVFLG = 0`, not this function):
///   Use the Lambert solver in `math::lambert`; pass its result directly as VG.
///
/// Returns the VG vector in m/s (ECI), identical in meaning to AGC VGTIG (B+7 m/cs).
///
/// AGC source: Comanche055/P30-P37.agc, S30.1 (DELVSLV → DELVSIN), pages 639-640.
///             Comanche055/P40-P47.agc, S40.1 delta-V path, pages 709-711.
///
/// Inputs: current state vector (ECI), burn target.
/// Output: VG in ECI frame, m/s.
pub fn predict_vg_at_ignition(
    current: &StateVector,
    target: &BurnTarget,
) -> Vec3;

/// Canonical SPS engine constants from Comanche055.
///
/// Returns `(thrust_N, exhaust_velocity_m_s)`.
///
/// - thrust_N: 91 188.544 N
///   AGC: P40-P47.agc `FENG 2DEC 9.1188544 B-7` (page 689).
/// - exhaust_velocity_m_s: 3 151.0396 m/s
///   AGC: P40-P47.agc `2VEXHUST 2DEC 63.020792 B-7` (page 744);
///        stored as twice v_e; Rust returns the true v_e = 63.020792/2 m/cs × 100 cs/s.
///
/// AGC source: Comanche055/P40-P47.agc, constants block, pages 689, 744.
pub fn sps_constants() -> (f64, f64);
```

---

## Scale Factors

| AGC register | AGC scale | Rust representation |
|---|---|---|
| `TIG` | DP B+28 centiseconds | `Met` (centiseconds, `u32`) |
| `DELVSLV` | vector B+7 m/cs | `Vec3` (m/s; multiply m/cs × 100) |
| `DELVSIN` | vector B+7 m/cs | `Vec3` (m/s) |
| `VGTIG` | vector B+7 m/cs | `Vec3` (m/s) |
| `WEIGHT/G` | SP B+16 kg | `f64` kg |
| `F` (FENG) | DP B+7 M-Newtons (= kN × 10) | `f64` N |
| `EMDOT` | SP B+3 kg/cs (pad-loaded) | `f64` kg/s |
| `2VEXHUST` | DP B+7 m/cs (= 2×v_e) | `f64` m/s (halved) |
| `TGO` | DP B+28 centiseconds | `f64` s (×0.01) |

Conversion from m/cs to m/s: multiply by 100. The AGC centisecond time base appears throughout; all Rust APIs use SI seconds.

---

## Invariants

1. `burn_duration` with `mass <= 0.0` or `thrust <= 0.0` or `exhaust_velocity <= 0.0` returns `None`. In the AGC, these conditions are assumed non-reachable at S40.13 call time (WEIGHT/G and F are always positive pad-loaded values); in Rust they are guarded to prevent NaN/infinity propagation.
2. `burn_duration` with `delta_v_mag == 0.0` returns `Some(0.0)`.
3. `burn_duration` with finite positive inputs always returns `Some(finite)` — the intermediate `exp()` is bounded by the mass ratio.
4. `predict_vg_at_ignition` must return a finite vector for any finite `StateVector` and `BurnTarget`; no `unwrap` inside the function.
5. `sps_constants()` is a pure function returning compile-time constants; it must match the AGC source values exactly (no rounding beyond what f64 precision imposes).
6. No heap allocation; no `unwrap`; all functions are `#[must_use]`.

---

## P30 vs P37 Targeting Paths

| Path | AGC program | `XDELVFLG` | How VG is computed |
|---|---|---|---|
| External delta-V | P30 / S30.1 | 1 (set) | Crew/ground enters TIG + DELVSLV; S30.1 propagates state and rotates to ECI |
| Lambert return | P37 / S31.1 | 0 (clear) | Target position RTARG + TPASS4 stored; Lambert solver gives required velocity |

`predict_vg_at_ignition` implements the P30 (external delta-V) path only. The P37 Lambert path is computed by `math::lambert::lambert_solver` and passed directly into the `BurnTarget` by the caller.

The flag `XDELVFLG` (bit 8 of flag word 2, `ERASABLE_ASSIGNMENTS.agc`) controls which path P40CSM takes in `S40.1` (branch at `BOF XDELVFLG S40.1B`). The Rust equivalent is the caller choosing between `predict_vg_at_ignition` (P30) or using the Lambert result (P37) when constructing `BurnTarget`.

---

## Test Cases

### TC-T1: 50 m/s prograde SPS burn duration
- Input: `delta_v_mag = 50.0`, SPS constants from `sps_constants()`, `mass = 28_800.0` kg (typical CSM dry + propellant)
- Expected: `Some(t)` where `t = (28800 / mdot) * (1 - exp(-50 / 3151.04))` ≈ 14.4 s (hand-computed)
- Tolerance: < 0.1 s
- Purpose: validates Tsiolkovsky against hand calculation with flight-representative values

### TC-T2: Zero delta-V
- Input: `delta_v_mag = 0.0`, any positive thrust / exhaust_velocity / mass
- Expected: `Some(0.0)` exactly
- Purpose: boundary condition; ensures the `1 - exp(0)` = 0 branch is not special-cased incorrectly

### TC-T3: Tsiolkovsky sanity — compare exact vs AGC piecewise
- Input: `delta_v_mag = 3.0`, `thrust = 91188.544`, `exhaust_velocity = 3151.0396`, `mass = 28800.0`
- Hand-computed: `mdot = 91188.544 / 3151.0396 ≈ 28.941 kg/s`, `t = (28800 / 28.941) * (1 - exp(-3 / 3151.04)) ≈ 0.844 s`
- Expected: `Some(t)` within 1 ms; burn short enough to be in AGC's short-burn branch
- Purpose: ensures the exact formula is numerically stable for very short burns

### TC-T4: LVLH-to-ECI frame consistency
- Input: circular orbit at 185 km altitude; prograde burn (LVLH Y = 50 m/s, X = Z = 0)
- Expected: `predict_vg_at_ignition` result has magnitude ≈ 50 m/s; component along velocity vector ≈ 50 m/s; cross-track and radial components < 0.01 m/s
- Purpose: validates the LVLH → ECI rotation; a pure prograde burn must remain prograde after rotation

### TC-T5: Invalid inputs to burn_duration
- Input: `mass = -1.0` or `thrust = 0.0` or `exhaust_velocity = 0.0`
- Expected: `None`
- Purpose: guards all three singularity conditions

---

## agc-sim Impact

- `MissionState` struct: add `burn_target: Option<BurnTarget>` and `predicted_tgo_s: Option<f64>` fields.
- `SimLog`: emit `".info("TGO computed: {:.1}s", tgo)` when S40.13 equivalent fires.
- `dsky_terminal.rs`: N85 display (already exists in sim) should read `predicted_tgo_s` for the time-to-go readout.
- No new DSKY keyboard bindings needed.
