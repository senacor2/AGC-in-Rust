# Specification: `programs/p33` and `programs/p34` — TPI and TPM Rendezvous Targeting

**Status**: Ready for implementation (Milestone 5 Phase 6)
**Module paths**:
- `agc-core/src/programs/p33.rs` — new file (replaces stub in `programs/p31_p34.rs`)
- `agc-core/src/programs/p34.rs` — new file (replaces stub in `programs/p31_p34.rs`)
**Architecture reference**: `docs/architecture.md` §7.2 (P33/P34 rows), §9 (Navigation Math)
**Targeting reference**: `specs/targeting-spec.md` — `Maneuver` struct, `TargetingMode` enum,
  `burn_attitude`, `lvlh_to_inertial`
**Lambert reference**: `specs/lambert-spec.md` — `lambert(r1, r2, tof, mu, prograde)` public API
  and preconditions
**P31/P32 reference**: `specs/p31_p32-spec.md` — structural template; `propagate_to_tig` helper;
  LVLH frame conventions; `MU_EARTH`; alarm numbering
**P30 reference**: `specs/p30-spec.md` — `state.pending_maneuver` output pattern
**P20 reference**: `specs/p20-spec.md` — `state.rendezvous_nav.target_pos/target_vel/target_epoch`
  field names and semantics
**Kepler reference**: `specs/kepler-spec.md` — `kepler_step(r0, v0, dt, mu) -> (Vec3, Vec3)`
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, `Comanche055` directory):
- `Comanche055/P34,P35,P74,P75.agc` — P33 (labelled P34 in the AGC source numbering used in some
  files) and P34 (TPM) entry sequences and computation routines
- `Comanche055/ROUTINE_30,30.agc` — R30 shared targeting initialisation called by all targeting
  programs
- `Comanche055/P30,P31,P37,P40SUBROUTINES.agc` — IMPULSIVE subroutine and shared targeting
  helpers used by P30–P34
- `Comanche055/CONIC_SUBROUTINES.agc` — Lambert solver (`RVIO`/`LAMBERT` entry); the Rust port
  uses `math::lambert::lambert` as a black-box replacement
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `ETIG` (TPI TIG), `ELEV` (elevation angle at TPI,
  scale B+0 radians), `DTTPI` (TPI-to-intercept transfer time, scale B+17 s),
  `DELVEET1/2/3` (computed ΔV, scale B+7 m/s), `TTPI` (TPI arrival epoch)
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
  Operation*, Springer-Praxis 2010.
- Chapter 11, "Rendezvous and Navigation Programs" pp. 303–340
  - pp. 333–336: TPI geometry and elevation-angle concept; P33 crew workflow
  - pp. 336–338: P34 (TPM) midcourse correction concept and targeting path
  - pp. 337–340: full rendezvous timeline showing CSI → CDH → TPI → TPM sequence

---

## 1. Purpose and Program Roles

### 1.1 Mission context — the terminal phase sequence

The nominal Apollo rendezvous sequence after LM ascent proceeds through four
maneuver phases:

1. **CSI** — computed by P31; adjusts the chaser's orbit for the coelliptic
   phasing.
2. **CDH** — computed by P32; makes the chaser's orbit coelliptic with the
   target at constant altitude separation Δh.
3. **TPI** (Terminal Phase Initiation) — computed by **P33**; the intercept
   burn that launches the chaser directly at the target on a ~10-minute transfer
   arc. This is the final powered phase before braking.
4. **TPM** (Terminal Phase Midcourse) — computed by **P34**; one or more small
   RCS correction burns executed during the TPI transfer if the chaser drifts
   off the intended intercept trajectory.

After TPM the crew monitors range closure and executes braking maneuvers
(P47/RCS) to achieve docking approach. P33 and P34 are both one-shot programs:
the crew invokes each one, verifies the displayed ΔV, and then selects P41 (RCS
execution) to perform the burn.

Unlike P31 and P32, which use closed-form coelliptic geometry, P33 and P34 both
call the Lambert solver (`math::lambert::lambert`) to find the intercept transfer
velocity. The two programs are structurally identical; the difference is only in
the time interval and the definition of the "current" epoch.

### 1.2 P33 — Terminal Phase Initiation (TPI)

P33 computes the burn that, when executed at the TPI ignition time (TIG), will
carry the chaser to the target's position exactly `dt_tpi` seconds later (the
terminal-phase transfer time, nominally 10 minutes). The crew selects TIG to
achieve a desired elevation angle `E` of the target above the chaser's local
horizontal at TIG — typically 27.45° (130 mils) per Apollo mission rules. P33
displays the elevation angle computed from current state vectors so the crew can
judge timing, but does not iterate TIG to match `E` automatically. The crew
waits for the geometry to evolve, then invokes P33 at the right moment.

The output is a single `Maneuver` in `state.pending_maneuver` with
`mode = TargetingMode::TpiBurn`.

### 1.3 P34 — Terminal Phase Midcourse (TPM)

P34 is invoked during the TPI transfer (between TIG and arrival) if the crew
decides to trim the trajectory. It computes the additional burn required so
that the chaser still arrives at the **same target position** as the original
P33 solution, at the **same scheduled arrival epoch**. The time interval input
to P34 is `dt_midcourse`: the time from the MCC burn to the original scheduled
arrival, which is smaller than P33's `dt_tpi`.

Structurally, P34 calls exactly the same Lambert-based computation as P33 with
different state-vector epochs and a shorter transfer time. The key requirement
is that P34 must know the **original scheduled arrival epoch** that P33
established. This is stored in a new `AgcState` field `tpi_arrival_epoch`
(see §3.3 for justification).

The output is a `Maneuver` with `mode = TargetingMode::TpmBurn`.

---

## 2. Module Paths

- P33: `agc-core/src/programs/p33.rs`
- P34: `agc-core/src/programs/p34.rs`

Both replace the stubs in `agc-core/src/programs/p31_p34.rs`. After Phase 6,
`p31_p34.rs` is empty and can be removed; the `mod.rs` entry for it is deleted
and replaced with:

```rust
pub mod p33;
pub mod p34;
```

Entry points registered in `PROGRAM_TABLE`:

```rust
t[33] = Some(p33::p33_init);
t[34] = Some(p34::p34_init);
```

The `propagate_to_tig` helper defined in `p31.rs` (and re-exported via
`pub(super)`) is used by P33 and P34. Both new files import it as:

```rust
use super::p31::propagate_to_tig;
```

---

## 3. State Additions

### 3.1 Input fields consumed

Both programs read from the following existing `AgcState` fields:

| `AgcState` field | Type | Purpose |
|-----------------|------|---------|
| `state.csm_state.position` | `Vec3` (m, ECI) | Chaser position at current epoch |
| `state.csm_state.velocity` | `Vec3` (m/s, ECI) | Chaser velocity at current epoch |
| `state.csm_state.epoch` | `Met` (centiseconds) | Epoch of chaser state |
| `state.rendezvous_nav.target_pos` | `Vec3` (m, ECI) | Target position (from P20) |
| `state.rendezvous_nav.target_vel` | `Vec3` (m/s, ECI) | Target velocity (from P20) |
| `state.rendezvous_nav.target_epoch` | `f64` (seconds) | Epoch of target state |
| `state.vn.pending_tig` | `Option<Met>` (centiseconds) | Crew-entered TIG |
| `state.refsmmat` | `Mat3x3` | IMU alignment for burn attitude |

### 3.2 Output field written

| `AgcState` field | Type | Content after P33/P34 |
|-----------------|------|----------------------|
| `state.pending_maneuver` | `Option<Maneuver>` | The computed TPI or TPM burn |
| `state.tpi_arrival_epoch` | `Option<f64>` (seconds) | **Written by P33 only**; read by P34 |

`state.pending_maneuver` is set to `Some(maneuver)` on success and left
unchanged on alarm. `state.tpi_arrival_epoch` is set on every successful P33
execution.

### 3.3 New field: `tpi_arrival_epoch`

**Decision: add `state.tpi_arrival_epoch: Option<f64>` to `AgcState`.**

**Rationale.** P34 must aim at the same target position as P33 established. The
target position at arrival is `kepler_step(target_at_tpi_tig, dt_tpi)`. At P34
invocation the crew has already propagated past the P33 TIG, so the original
arrival epoch is not recoverable from current state without storing it. Three
alternatives were considered:

1. Store only `dt_tpi` (the original transfer time) and let P34 re-derive the
   arrival epoch as `p33_tig + dt_tpi`. This requires P34 to also remember the
   P33 TIG and re-propagate the target — more information to store.
2. Store the arrival **position vector** `r_t_arrive` directly. This eliminates
   the need for P34 to re-propagate the target, but consumes more erasable
   memory (3 `f64` fields vs. 1) and creates a stale-state risk if P20 updates
   the target vector between P33 and P34 (the arrival position should be
   re-derived from the latest target state, not frozen at P33 time).
3. Store the arrival **epoch** `tpi_arrival_epoch`. P34 re-derives the arrival
   position by propagating the current P20 target state to this epoch, using
   whatever the latest P20 target estimate is. This correctly incorporates any
   P20 updates between P33 and P34 execution, minimises memory (1 scalar), and
   matches the AGC erasable variable `TTPI` (see `ERASABLE_ASSIGNMENTS.agc`).

**Option 3 is adopted.** The field is `Option<f64>` (seconds from epoch) so that
the `Default` value is `None` and P34 can alarm immediately if P33 was never run.

Add to `AgcState`:

```rust
/// Arrival epoch stored by P33 at TPI computation time.
///
/// Seconds from mission epoch (same time base as `Met.to_seconds()`).
/// Set to `Some(epoch_s)` by `p33_init` on successful TPI computation.
/// Read by `p34_init`; if `None`, P34 raises alarm 01441.
/// Reset to `None` on FRESH START.
///
/// AGC erasable: TTPI (computed TPI arrival time; scale B+17 centiseconds).
pub tpi_arrival_epoch: Option<f64>,
```

Add `tpi_arrival_epoch: None` to `AgcState::new()`.

### 3.4 `TargetingMode` additions

Two new variants are added to the existing `TargetingMode` enum in
`guidance::targeting`:

```rust
/// P33 — Terminal Phase Initiation (TPI) burn.
///
/// Delta-V computed by Lambert solver targeting the LM's position at
/// TIG + dt_tpi. Transfer time nominally 10 minutes.
/// DSKY: V06 N37 (TIG), V06 N55 (elevation angle + transfer time),
///       V06 N81 (LVLH ΔV components).
TpiBurn,

/// P34 — Terminal Phase Midcourse (TPM) correction burn.
///
/// Delta-V computed by Lambert solver targeting the same arrival position
/// as the P33 TPI solution, with remaining time dt_midcourse.
/// DSKY: V06 N37 (TIG), V06 N81 (LVLH ΔV components).
TpmBurn,
```

P40 and P41 treat `TpiBurn` and `TpmBurn` identically to `Lambert` for burn
monitor purposes (same noun table: V06 N81 for LVLH ΔV display).

---

## 4. Public API

### 4.1 Constants

```rust
// agc-core/src/programs/p33.rs

/// Major mode number for P33.
pub const P33_MAJOR_MODE: u8 = 33;

/// Job priority for P33. Same as P31/P32 (foreground targeting programs).
pub const P33_PRIORITY: JobPriority = 10;

/// Default TPI-to-intercept transfer time (seconds).
/// Apollo nominally used 10 minutes. The crew may supply a different value
/// via V06 N55 (see §6). This constant is used when no crew entry is made.
///
/// AGC erasable: DTTPI, scale B+17 s.
pub const TPI_DEFAULT_TRANSFER_TIME_S: f64 = 600.0;

/// Minimum chaser-target separation (m) below which the geometry is
/// considered collinear or degenerate for Lambert purposes.
/// If `norm(r_t_arrive - r_c_tig) < TPI_MIN_SEPARATION_M`, alarm 01443.
pub const TPI_MIN_SEPARATION_M: f64 = 1_000.0;

/// Minimum transfer time (s) accepted by P33/P34. Protects the Lambert
/// solver against zero or near-zero TOF inputs.
pub const TPI_MIN_TOF_S: f64 = 60.0;

/// Staleness limit for the target state (centiseconds).
/// If `(tig_cs - target_epoch_cs) > TPI_STALE_TARGET_CS`, alarm 01442 is
/// raised (non-fatal: computation proceeds but DSKY shows staleness warning).
/// 30 minutes = 180_000 cs.
pub const TPI_STALE_TARGET_CS: u64 = 180_000;

/// Desired elevation angle of target above chaser local horizontal at TIG
/// (radians). Apollo used 27.45° = 0.4793 rad (130 mils).
/// Used only for display — P33 does not iterate TIG to match this angle.
pub const TPI_NOMINAL_ELEVATION_RAD: f64 = 0.4793;
```

```rust
// agc-core/src/programs/p34.rs

/// Major mode number for P34.
pub const P34_MAJOR_MODE: u8 = 34;

/// Job priority for P34.
pub const P34_PRIORITY: JobPriority = 10;

/// Minimum chaser-target range (m) below which P34 refuses to compute a
/// midcourse correction. If the two vehicles are this close, a midcourse
/// is physically meaningless and the crew should switch to braking.
///
/// 100 m corresponds to the final approach range where braking takes over.
pub const TPM_MIN_RANGE_M: f64 = 100.0;
```

### 4.2 P33 entry point

```rust
/// Entry point for P33 (Terminal Phase Initiation).
/// Registered in PROGRAM_TABLE[33].
///
/// Sets `state.major_mode = 33`. Reads `state.vn.pending_tig` for the TPI
/// TIG. Prompts for TPI transfer time via V06 N55 (see §6; currently
/// accepted as `dt_tpi` argument for Milestone 5). Calls
/// `compute_lambert_intercept` and displays the result via V06 N81.
/// Stores the result in `state.pending_maneuver` with
/// `mode = TargetingMode::TpiBurn`. Also stores
/// `state.tpi_arrival_epoch = Some(arrival_epoch_s)` for subsequent P34 use.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` must be non-zero; otherwise alarm
///   01440.
/// - `state.vn.pending_tig` must be `Some(_)`; otherwise alarm 01441.
///
/// # Post-conditions (success)
/// - `state.major_mode == 33`
/// - `state.dsky.prog == 33`
/// - `state.pending_maneuver == Some(tpi_maneuver)` where
///   `tpi_maneuver.mode == TargetingMode::TpiBurn`
/// - `state.tpi_arrival_epoch == Some(arrival_s)` where
///   `arrival_s = tig_s + dt_tpi`
/// - DSKY displays ΔV via V06 N81 and elevation angle via V06 N55.
///
/// # Post-conditions (alarm)
/// - `state.pending_maneuver` is unchanged.
/// - `state.tpi_arrival_epoch` is unchanged.
///
/// # Alarms
/// - 01440: target state is zero.
/// - 01441: `pending_tig` is None (no TIG entered by crew).
/// - 01442: target state stale (non-fatal; proceeds but displays warning).
/// - 01443: degenerate geometry (chaser and target arrival positions within
///   `TPI_MIN_SEPARATION_M`).
/// - 01444: Lambert non-convergence (from `validate_lambert_inputs` or
///   solver panic guard; see §5.3).
pub fn p33_init(state: &mut AgcState) -> JobPriority
```

### 4.3 P34 entry point

```rust
/// Entry point for P34 (Terminal Phase Midcourse).
/// Registered in PROGRAM_TABLE[34].
///
/// Sets `state.major_mode = 34`. Reads `state.tpi_arrival_epoch` to
/// determine the scheduled arrival epoch, then calls
/// `compute_lambert_intercept` with the remaining transfer time
/// `dt_midcourse = tpi_arrival_epoch - now_s`. Stores the result in
/// `state.pending_maneuver` with `mode = TargetingMode::TpmBurn`.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` must be non-zero; otherwise alarm
///   01440.
/// - `state.tpi_arrival_epoch` must be `Some(_)` (P33 must have run);
///   otherwise alarm 01441.
/// - `dt_midcourse` must be ≥ `TPI_MIN_TOF_S`; otherwise alarm 01443
///   (arrival has already passed or is too soon).
/// - Chaser-to-target range at current position must be ≥ `TPM_MIN_RANGE_M`;
///   otherwise alarm 01445 (too close for a midcourse; switch to braking).
///
/// # Post-conditions (success)
/// - `state.major_mode == 34`
/// - `state.dsky.prog == 34`
/// - `state.pending_maneuver == Some(tpm_maneuver)` where
///   `tpm_maneuver.mode == TargetingMode::TpmBurn`
/// - `state.tpi_arrival_epoch` is NOT modified (P33's value is preserved
///   for possible repeated P34 invocations during the same transfer).
///
/// # Alarms
/// - 01440: target state zero.
/// - 01441: `tpi_arrival_epoch` is None (P33 never ran).
/// - 01442: target state stale (non-fatal warning).
/// - 01443: `dt_midcourse ≤ TPI_MIN_TOF_S` or chaser already past arrival.
/// - 01444: Lambert non-convergence.
/// - 01445: chaser within `TPM_MIN_RANGE_M` of target (already in braking).
pub fn p34_init(state: &mut AgcState) -> JobPriority
```

### 4.4 Shared pure-computation helper

P33 and P34 share a single computation core. The inputs are identical in
structure; only the time intervals and calling context differ.

```rust
/// Compute a Lambert-based intercept burn given chaser and target state
/// at the burn epoch and a transfer time.
///
/// This is the pure-math core used by both P33 and P34. It does not touch
/// `AgcState` and is safe to call from tests without a full state.
///
/// # Arguments
/// - `r_c`: Chaser position at burn TIG (m, inertial ECI).
///   Caller must propagate with `kepler_step` from current epoch to TIG
///   before calling.
/// - `v_c`: Chaser velocity at burn TIG (m/s, inertial ECI).
/// - `r_t_arrive`: Target position at the intended arrival epoch (m,
///   inertial ECI). Caller must propagate the target state with `kepler_step`
///   to `tig_s + tof` before calling. For P34 this is
///   `tpi_arrival_epoch`, not `tig_s + new_tof`.
/// - `tof`: Transfer time from TIG to arrival (s). Must satisfy
///   `tof >= TPI_MIN_TOF_S`.
/// - `mu`: Gravitational parameter (m³/s²). Use `MU_EARTH`.
///
/// # Returns
/// `Ok(InterceptResult)` on success, `Err(InterceptError)` on failure.
///
/// # Algorithm
/// See §5.2 for step-by-step detail.
pub fn compute_lambert_intercept(
    r_c:        Vec3,
    v_c:        Vec3,
    r_t_arrive: Vec3,
    tof:        f64,
    mu:         f64,
) -> Result<InterceptResult, InterceptError>
```

```rust
/// Result of a successful Lambert intercept computation.
#[derive(Clone, Copy, Debug)]
pub struct InterceptResult {
    /// Delta-V in the LVLH frame at the burn TIG (m/s).
    /// [0] = R (radial), [1] = S (in-track), [2] = W (cross-track).
    pub dv_lvlh: Vec3,

    /// Delta-V magnitude (m/s). Equal to `norm(dv_inertial)`.
    pub dv_mag: f64,
}

/// Error conditions for a Lambert intercept computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterceptError {
    /// Chaser or target position vector is zero or below LEO altitude.
    DegenerateState,
    /// Transfer time is below `TPI_MIN_TOF_S` or non-positive.
    InvalidTimeInterval,
    /// Departure and arrival positions are within `TPI_MIN_SEPARATION_M`
    /// (collinear or same-point geometry; Lambert is undefined).
    DegenerateGeometry,
    /// r_c and r_t_arrive are anti-parallel (180° transfer with no
    /// defined plane; Lambert panics if called; rejected by pre-validation).
    AntiParallelVectors,
    /// Lambert solver did not converge (Halley iteration exhausted).
    /// This maps to the AGC PROGRAM ALARM displayed on the DSKY.
    LambertNotConverged,
}
```

### 4.5 Lambert input pre-validator

This helper is called by `compute_lambert_intercept` (and may be called by test
harnesses) before invoking `math::lambert::lambert`. It checks all preconditions
that, if violated, would cause the Lambert solver to panic.

```rust
/// Validate inputs to `math::lambert::lambert` before calling the solver.
///
/// The Lambert solver (`math::lambert::lambert`) panics on degenerate inputs
/// (anti-parallel vectors, zero-length vectors, zero or negative TOF/mu).
/// Calling this function first converts those panic conditions into
/// structured `Err` variants, enabling graceful alarm handling.
///
/// # Checks performed
/// 1. `norm(r1) > 1e6 m` and `norm(r2) > 1e6 m` (both positions above any
///    plausible body's surface — if either is zero, Lambert panics).
/// 2. `tof > 0.0` (Lambert panics on non-positive TOF).
/// 3. `mu > 0.0` (Lambert panics on non-positive mu).
/// 4. `norm(r2 - r1) > TPI_MIN_SEPARATION_M` (non-zero chord; if zero,
///    departure and arrival are the same point — degenerate).
/// 5. Anti-parallel check: if `dot(r1, r2) / (norm(r1) * norm(r2)) < -0.9999`
///    and `|cross(r1, r2)| < 1e-6 * norm(r1) * norm(r2)`, the transfer
///    angle is 180° with no defined plane; return `Err(AntiParallelVectors)`.
///
/// # Returns
/// `Ok(())` if all preconditions are satisfied and it is safe to call
/// `lambert(r1, r2, tof, mu, prograde)`.
/// `Err(InterceptError)` if any precondition fails.
pub fn validate_lambert_inputs(
    r1:  Vec3,
    r2:  Vec3,
    tof: f64,
    mu:  f64,
) -> Result<(), InterceptError>
```

### 4.6 Elevation angle helper (module-private but tested)

```rust
/// Compute the elevation angle of the target above the chaser's local
/// horizontal at a given epoch.
///
/// The elevation angle E is the angle between the line-of-sight vector
/// (r_t - r_c) and the chaser's local horizontal plane. The local
/// horizontal is perpendicular to the chaser's radius vector r_c.
///
/// E = asin( dot(unit(r_t - r_c), unit(r_c)) )
///
/// Positive E means the target is above the chaser's horizon.
///
/// # Arguments
/// - `r_c`: Chaser position (m, inertial).
/// - `r_t`: Target position at the same epoch (m, inertial).
///
/// # Returns
/// Elevation angle in radians, range [-π/2, +π/2].
/// Returns 0.0 if `norm(r_t - r_c) < 1.0` (same position; no defined LOS).
///
/// AGC display: shown via V06 N55 R1, in mils (1 mil = 0.001 rad for DSKY
/// display; actual AGC scale is B+0 radians in erasable `ELEV`).
pub(crate) fn elevation_angle(r_c: Vec3, r_t: Vec3) -> f64
```

---

## 5. Algorithm Specifications

### 5.1 Overview and Lambert relationship

Both P33 and P34 reduce to the same boundary-value problem:

> *Given departure position **r_c** at time t₁ and desired arrival position
> **r_t_arrive** at time t₂, find the departure velocity **v₁** such that a
> Keplerian arc connects the two positions in exactly tof = t₂ − t₁ seconds.*

This is Lambert's problem, solved by `math::lambert::lambert(r_c, r_t_arrive,
tof, MU_EARTH, prograde=true)` returning `(v_c_required, _)`. The ΔV is
`v_c_required − v_c_at_tig`.

The Lambert solver is treated as a black box. Its internal algorithm (Izzo 2015
Halley iteration) is described in `specs/lambert-spec.md`; it is not
re-derived or re-implemented here.

**Prograde flag**: All TPI/TPM transfers in the Apollo rendezvous geometry are
prograde (short-way, transfer angle < 180°). The `prograde` argument to Lambert
is always `true` in both P33 and P34.

### 5.2 `compute_lambert_intercept` — step-by-step

All inputs are defined in §4.4. The algorithm mirrors the AGC subroutine
`RVIO`/`LAMBERT` called from `P34,P35,P74,P75.agc` via the interpretive
language.

#### Step 0 — Validate inputs

```
// Precondition checks (return Err without calling Lambert):
if norm(r_c) < 1e6       →  return Err(DegenerateState)
if norm(r_t_arrive) < 1e6 →  return Err(DegenerateState)
if tof < TPI_MIN_TOF_S   →  return Err(InvalidTimeInterval)
if mu <= 0.0             →  return Err(DegenerateState)

validate_lambert_inputs(r_c, r_t_arrive, tof, mu)?
// (This also checks anti-parallel and zero chord; returns Err if they fail.)
```

#### Step 1 — Call Lambert solver

```
(v_c_required, _v_arrive) = lambert(r_c, r_t_arrive, tof, MU_EARTH, true)
```

`lambert` returns the departure velocity `v_c_required` at `r_c` and the
arrival velocity `_v_arrive` at `r_t_arrive`. The arrival velocity is not
used by P33 or P34 (no burn is planned at arrival in this phase).

#### Step 2 — Compute inertial ΔV

```
dv_inertial = v_c_required - v_c                   (m/s, inertial ECI)
dv_mag      = norm(dv_inertial)                    (m/s)
```

(Battin §11.5, eq. 11-68: the TPI burn magnitude is the norm of the velocity
difference between the required Lambert departure velocity and the current
chaser velocity.)

#### Step 3 — Convert to LVLH at TIG

The LVLH frame at the burn TIG uses the chaser state `(r_c, v_c)` as the
reference:

```
r_hat = unit(r_c)                                  (R-axis: radial outward)
h_vec = cross(r_c, v_c)                            (angular momentum vector)
w_hat = unit(h_vec)                                (W-axis: orbit normal)
s_hat = cross(w_hat, r_hat)                        (S-axis: along-track)

dv_lvlh[0] = dot(dv_inertial, r_hat)               (R component, m/s)
dv_lvlh[1] = dot(dv_inertial, s_hat)               (S component, m/s)
dv_lvlh[2] = dot(dv_inertial, w_hat)               (W component, m/s)
```

(LVLH convention from `specs/rendezvous-spec.md` §4, consistent with P31/P32.)

#### Step 4 — Return result

```
return Ok(InterceptResult { dv_lvlh, dv_mag })
```

### 5.3 P33 entry point — step-by-step (`p33_init`)

**Step 0 — Guard checks**

```
if norm(state.rendezvous_nav.target_pos) < 1.0
    → raise alarm 01440; return P33_PRIORITY

let Some(tig_cs) = state.vn.pending_tig.take()
    else → raise alarm 01441; return P33_PRIORITY
```

**Step 1 — Staleness check (non-fatal)**

```
target_epoch_cs = (state.rendezvous_nav.target_epoch * 100.0) as u64
tig_cs_u64 = tig_cs.0 as u64
if tig_cs_u64 > target_epoch_cs
    && (tig_cs_u64 - target_epoch_cs) > TPI_STALE_TARGET_CS
    → display DSKY staleness warning; do NOT abort
```

**Step 2 — Set major mode**

```
state.major_mode = P33_MAJOR_MODE   // 33
```

**Step 3 — Propagate chaser to TIG**

```
tig_s = tig_cs.to_seconds()        // centiseconds → seconds
chaser_epoch_s = state.csm_state.epoch.to_seconds()
dt_chaser = tig_s - chaser_epoch_s  // must be ≥ 0 (targeting into future)
if dt_chaser < 0.0 → raise alarm 01443; return P33_PRIORITY

(r_c_tig, v_c_tig) = kepler_step(
    state.csm_state.position,
    state.csm_state.velocity,
    dt_chaser,
    MU_EARTH)
```

**Step 4 — Compute elevation angle at TIG (display only)**

```
// Propagate target to TIG for elevation display.
target_epoch_s = state.rendezvous_nav.target_epoch
dt_target_to_tig = tig_s - target_epoch_s
(r_t_tig, _) = kepler_step(
    state.rendezvous_nav.target_pos,
    state.rendezvous_nav.target_vel,
    dt_target_to_tig,
    MU_EARTH)

elev_rad = elevation_angle(r_c_tig, r_t_tig)
// Display elev_rad via V06 N55 R1 (in mils = elev_rad / 0.001).
// The crew compares this to TPI_NOMINAL_ELEVATION_RAD (≈ 130 mils).
// P33 does NOT reject or loop if the angle differs from nominal;
// the crew decides whether to proceed or wait.
```

**Step 5 — Determine transfer time**

```
// In this milestone, dt_tpi is either:
//   a) A crew-entered value accepted via V06 N55 (future DSKY interactive
//      flow), or
//   b) TPI_DEFAULT_TRANSFER_TIME_S (600.0 s) if no crew entry was made.
// The dt_tpi value is passed as a parameter to p33_init in the current
// milestone implementation, following the same pattern as P31/P32.
// (See §6 for the DSKY sequence; Milestone 6 adds interactive entry.)
```

**Step 6 — Propagate target to arrival epoch**

```
arrival_epoch_s = tig_s + dt_tpi
dt_target_to_arrive = arrival_epoch_s - state.rendezvous_nav.target_epoch
(r_t_arrive, _) = kepler_step(
    state.rendezvous_nav.target_pos,
    state.rendezvous_nav.target_vel,
    dt_target_to_arrive,
    MU_EARTH)
```

(Target is propagated from its current P20 epoch, not from the TIG-epoch
intermediate state, to keep the propagation as one step and minimize
accumulation of Kepler error. O'Brien p. 334 notes that the AGC propagates
from the most recently updated state vector.)

**Step 7 — Compute Lambert intercept**

```
result = compute_lambert_intercept(r_c_tig, v_c_tig, r_t_arrive, dt_tpi, MU_EARTH)

match result:
    Err(DegenerateState)      → alarm 01440; return P33_PRIORITY
    Err(InvalidTimeInterval)  → alarm 01443; return P33_PRIORITY
    Err(DegenerateGeometry)   → alarm 01443; return P33_PRIORITY
    Err(AntiParallelVectors)  → alarm 01444; return P33_PRIORITY
    Err(LambertNotConverged)  → alarm 01444; return P33_PRIORITY
    Ok(res)                   → continue
```

**Step 8 — Build `Maneuver`**

```
dv_inertial = lvlh_to_inertial(r_c_tig, v_c_tig) matrix-times res.dv_lvlh
attitude    = burn_attitude(dv_inertial, state.refsmmat)

state.pending_maneuver = Some(Maneuver {
    tig:           Met::from_seconds(tig_s),
    delta_v:       DeltaV(dv_inertial),
    burn_attitude: attitude,
    mode:          TargetingMode::TpiBurn,
})
```

**Step 9 — Store arrival epoch**

```
state.tpi_arrival_epoch = Some(arrival_epoch_s)
```

**Step 10 — DSKY display**

```
// Display ΔV via V06 N81 (LVLH components in ft/s × 10⁻¹).
// Display TIG via V06 N37.
// Await crew acceptance (V33 E) or P41 selection.
```

### 5.4 P34 entry point — step-by-step (`p34_init`)

**Step 0 — Guard checks**

```
if norm(state.rendezvous_nav.target_pos) < 1.0
    → alarm 01440; return P34_PRIORITY

let Some(arrival_epoch_s) = state.tpi_arrival_epoch
    else → alarm 01441; return P34_PRIORITY

let Some(tig_cs) = state.vn.pending_tig.take()
    else → alarm 01441; return P34_PRIORITY
```

**Step 1 — Staleness check (non-fatal)**

Same as P33 Step 1.

**Step 2 — Set major mode**

```
state.major_mode = P34_MAJOR_MODE   // 34
```

**Step 3 — Compute dt_midcourse**

```
tig_s = tig_cs.to_seconds()
dt_midcourse = arrival_epoch_s - tig_s

if dt_midcourse < TPI_MIN_TOF_S
    → alarm 01443; return P34_PRIORITY
```

(If `dt_midcourse < TPI_MIN_TOF_S`, the scheduled arrival is already past or
imminent; no midcourse is possible. The crew should switch to P47 braking.)

**Step 4 — Range check**

```
range_m = norm(state.rendezvous_nav.target_pos - state.csm_state.position)
if range_m < TPM_MIN_RANGE_M
    → alarm 01445; return P34_PRIORITY
```

(Approximate range from current state vectors, not propagated; this is a
coarse guard against calling P34 when the vehicles are already in contact.)

**Step 5 — Propagate chaser to P34 TIG**

```
chaser_epoch_s = state.csm_state.epoch.to_seconds()
dt_chaser = tig_s - chaser_epoch_s
if dt_chaser < 0.0 → alarm 01443; return P34_PRIORITY

(r_c_tig, v_c_tig) = kepler_step(
    state.csm_state.position,
    state.csm_state.velocity,
    dt_chaser,
    MU_EARTH)
```

**Step 6 — Propagate target to arrival epoch**

```
dt_target_to_arrive = arrival_epoch_s - state.rendezvous_nav.target_epoch
(r_t_arrive, _) = kepler_step(
    state.rendezvous_nav.target_pos,
    state.rendezvous_nav.target_vel,
    dt_target_to_arrive,
    MU_EARTH)
```

Note: the target is propagated to the **original** P33 arrival epoch, not to a
new time. This is the defining characteristic of TPM: the aim point is
unchanged; only the chaser's actual position has deviated.

**Step 7 — Compute Lambert intercept**

```
result = compute_lambert_intercept(
    r_c_tig, v_c_tig, r_t_arrive, dt_midcourse, MU_EARTH)

// Error handling identical to P33 Step 7.
```

**Step 8 — Build `Maneuver`**

```
dv_inertial = lvlh_to_inertial(r_c_tig, v_c_tig) matrix-times result.dv_lvlh
attitude    = burn_attitude(dv_inertial, state.refsmmat)

state.pending_maneuver = Some(Maneuver {
    tig:           Met::from_seconds(tig_s),
    delta_v:       DeltaV(dv_inertial),
    burn_attitude: attitude,
    mode:          TargetingMode::TpmBurn,
})
```

`state.tpi_arrival_epoch` is **not** modified. P34 may be called multiple times
during the same transfer.

**Step 9 — DSKY display**

```
// Display ΔV via V06 N81.
// Display TIG via V06 N37.
```

### 5.5 Orbital mechanics identities used

| Equation | Identity | Reference |
|----------|----------|-----------|
| Lambert boundary-value problem | Given r₁, r₂, tof, find v₁, v₂ on conic arc | Lambert (1761); Izzo (2015) |
| ΔV = v_required − v_current | Impulsive maneuver assumption | Battin §11.5, eq. 11-68 |
| LVLH R-axis | `r_hat = unit(r)` | `specs/rendezvous-spec.md` §4 |
| LVLH W-axis | `w_hat = unit(cross(r, v))` | `specs/rendezvous-spec.md` §4 |
| LVLH S-axis | `s_hat = cross(w_hat, r_hat)` | `specs/rendezvous-spec.md` §4 |
| Elevation angle | `E = asin(dot(unit(r_t-r_c), unit(r_c)))` | O'Brien p. 333; AGC erasable `ELEV` |
| Kepler propagation | `kepler_step(r, v, dt, mu)` | `specs/kepler-spec.md`; universal variable |

---

## 6. DSKY Interaction

### 6.1 P33 DSKY sequence

| Step | Display | Verb/Noun | Content | Crew action |
|------|---------|-----------|---------|-------------|
| 1 | Major mode | V35 N33 | Prog 33 | Automatic |
| 2 | Request TIG | V06 N37 flashing | R1 = hours, R2 = minutes, R3 = seconds GET | `V25 E` enter TIG |
| 3 | Request transfer time | V06 N55 flashing | R1 = elevation (mils), R2 = transfer time (min × 10) | `V25 E` enter Δt, or `V34 E` for default 10 min |
| 4 | Computation | — | COMP lamp on | Automatic (Lambert + Kepler) |
| 5 | Elevation display | V06 N55 | R1 = computed elevation in mils; R2 = transfer time entered | Crew compares R1 to 130 mils |
| 6 | ΔV display | V06 N81 | R1 = ΔVx, R2 = ΔVy, R3 = ΔVz in ft/s (×10⁻¹, LVLH) | Crew verification |
| 7 | TIG display | V06 N37 | R1 = TIG GET hours, R2 = minutes, R3 = seconds | Crew acceptance `V33 E` |
| 8 | Standby | — | Await `V37 E 41 E` (P41 RCS) | Crew selects execution |

**AGC Verb/Noun references** (from `Comanche055/P34,P35,P74,P75.agc` and
`Comanche055/ROUTINE_30,30.agc`):

- **V06 N37** — Time display (TIG in GET hours/minutes/seconds). Consistent with
  P31/P32 TIG entry. AGC erasable `ETIG`.
- **V06 N55** — Two-register display: R1 = elevation angle at TIG (in mils;
  1 radian = 1000 mils for display), R2 = TPI transfer time (minutes × 10).
  AGC erasable `ELEV` (R1) and `DTTPI` (R2). The crew uses R1 to judge whether
  to accept the current TIG.
- **V06 N81** — Three-component ΔV in LVLH frame (ft/s × 10⁻¹, i.e., displayed
  as tenths of ft/s). Same noun used by P30 for external ΔV display. AGC
  erasable `DELVEET1/2/3` (scale B+7 m/s). O'Brien p. 334 cites N81 for TPI ΔV.

**Implementation note**: In Milestone 5, the interactive data-load sequences
(V25 flashing entry) are modeled as direct arguments to `p33_init`. The
Verb/Noun assignments above are recorded for fidelity and for the Milestone 6
DSKY interactive flow.

### 6.2 P34 DSKY sequence

| Step | Display | Verb/Noun | Content | Crew action |
|------|---------|-----------|---------|-------------|
| 1 | Major mode | V35 N34 | Prog 34 | Automatic |
| 2 | Request MCC TIG | V06 N37 flashing | R1/R2/R3 = MCC TIG GET | `V25 E` enter TIG |
| 3 | Computation | — | COMP lamp on | Automatic |
| 4 | ΔV display | V06 N81 | R1/R2/R3 = ΔVx/ΔVy/ΔVz LVLH (ft/s × 10⁻¹) | Crew verification |
| 5 | TIG display | V06 N37 | MCC TIG GET | Crew acceptance `V33 E` |
| 6 | Standby | — | Await P41 selection | Crew selects RCS execution |

P34 does not re-request the transfer time (it derives it from
`tpi_arrival_epoch`) and does not re-display elevation angle.

---

## 7. Program Alarms

| Alarm code | Condition | Program | Recovery |
|------------|-----------|---------|----------|
| 01440 | Target state is zero: `norm(target_pos) < 1.0 m`. P20 never ran or radar failure. | P33, P34 | Raise alarm; do not modify `pending_maneuver` or `tpi_arrival_epoch`. Crew must run P20 first. |
| 01441 | Required input not available: `pending_tig` is None (P33) or `tpi_arrival_epoch` is None (P34). | P33, P34 | Raise alarm; abort. Crew must enter TIG (P33) or run P33 first (P34). |
| 01442 | Target state epoch is stale: `(tig_cs - target_epoch_cs) > TPI_STALE_TARGET_CS`. Target state may be outdated. | P33, P34 | Non-fatal. Raise DSKY warning; proceed with computation. Crew may choose to re-run P20 for a fresher target state. |
| 01443 | Degenerate geometry or invalid time: `dt_tpi < TPI_MIN_TOF_S`, or `dt_midcourse < TPI_MIN_TOF_S`, or TIG in the past, or `norm(r_t_arrive - r_c_tig) < TPI_MIN_SEPARATION_M`. | P33, P34 | Raise alarm; abort. Crew must select a different TIG or wait for better geometry. |
| 01444 | Lambert solver cannot produce a solution: anti-parallel vectors (180° transfer, undefined plane) or Halley iteration non-convergence. | P33, P34 | Raise alarm; abort. Crew must select a different TIG or transfer time. |
| 01445 | Chaser already within `TPM_MIN_RANGE_M` of target. P34 is meaningless at this range; braking (P47) should be used instead. | P34 only | Raise alarm; abort. Crew selects P47 for final approach. |

Alarm codes 01440–01449 are reserved for P33/P34. Codes 01430–01439 are used by
P31/P32 (per `specs/p31_p32-spec.md` §7). Codes 01420–01429 are used by
P20–P23.

---

## 8. Edge Cases

### 8.1 Elevation angle does not match nominal at TIG

P33 displays the computed elevation angle but does not enforce it. The crew is
responsible for choosing a TIG at which the geometry is correct. If the crew
invokes P33 at a non-nominal TIG, P33 computes a valid (but possibly
non-optimal) Lambert solution and displays it. The crew may decline to execute
(not select P41) and wait for better geometry, then re-invoke P33.

The AGC source (`P34,P35,P74,P75.agc`) similarly does not implement any
iteration or rejection based on elevation angle; it is a display-only quantity
(O'Brien p. 333).

### 8.2 Lambert solver panic on degenerate input

`math::lambert::lambert` panics on anti-parallel r1/r2, zero positions, or
non-positive TOF/mu. The pre-validator `validate_lambert_inputs` (§4.5) is
called by `compute_lambert_intercept` before `lambert` is invoked. This converts
all panic-inducing conditions into `Err(InterceptError)` variants, which map to
alarm codes in the entry points. Under no circumstances is `lambert` called with
inputs that would trigger a panic; the validator is the mandatory gate.

Catching panics at runtime (via `std::panic::catch_unwind`) is not available in
`no_std` bare-metal Cortex-M7. Making the Lambert solver return `Result` instead
of panicking is out of scope for Phase 6. Pre-validation is the only mechanism.

### 8.3 Target state stale between P33 and P34

If P20 updates the target state vector between a P33 run and a subsequent P34
run, P34 uses the updated target state (propagated to `tpi_arrival_epoch`). This
is the correct behavior: a better target estimate should produce a better
midcourse. The `tpi_arrival_epoch` (stored as a scalar epoch) is insensitive to
target state updates — it is simply a time, not a frozen position. P34 always
re-derives the arrival position using the freshest available target state.

### 8.4 Target state never set

If `norm(state.rendezvous_nav.target_pos) < 1.0 m`, both P33 and P34 raise
alarm 01440 and return immediately without computing. This check uses the same
threshold as P31/P32 (1.0 m): any physically real LM orbit has
`r > 6.0e6 m` (LEO), so the default-zero state is unambiguously distinguishable.

### 8.5 Multiple P34 calls during a single TPI transfer

P34 does not modify `state.tpi_arrival_epoch`. The crew may invoke P34 multiple
times during the same TPI transfer (e.g., after each small midcourse execution).
Each P34 call reads the same arrival epoch and recomputes with the updated chaser
state, producing a fresh correction. This is the intended usage per O'Brien
p. 336 ("one or more midcourse corrections as required").

### 8.6 P33 re-run after TIG

If P33 is invoked after the original TIG has already passed (i.e., the crew
chose not to execute the first solution and is re-planning), the newly entered
TIG sets a new `tpi_arrival_epoch`. Previous P34 data is implicitly invalidated.
No explicit invalidation of `tpi_arrival_epoch` is needed because the new P33
overwrites it.

### 8.7 Near-zero ΔV (chaser already on intercept course)

If the chaser happens to already be on a trajectory that passes through the
target's arrival position, `dv_inertial ≈ [0, 0, 0]`. The result is stored as a
valid `Maneuver` with near-zero `delta_v` and `burn_attitude = IDENTITY` (per
the `Maneuver` invariant in `specs/targeting-spec.md` §3.1). P41 will execute a
near-zero burn correctly.

### 8.8 Transfer time too short for Kepler convergence

If `dt_tpi` is less than `TPI_MIN_TOF_S` (60 s), `compute_lambert_intercept`
returns `Err(InvalidTimeInterval)` before calling either Kepler or Lambert. The
60 s lower bound is conservative: the minimum physically meaningful TPI transfer
to a target 1–10 km away in LEO is approximately 60–120 s.

---

## 9. Test Cases

### TC-P33-1 — Circular coplanar intercept, analytical baseline

**Setup** (all vectors in ECI, origin at Earth center):
```
// CSM on circular LEO, 400 km altitude.  LM 10 km ahead in same orbit.
// TIG: CSM is at current epoch (no propagation needed, dt_chaser = 0).

r_c_tig = [6_778_000.0, 0.0, 0.0]          // m  (Earth radius + 400 km)
v_c_csm = [0.0, 7_669.0, 0.0]              // m/s  (circular speed at 400 km)

// LM is 10 km ahead in true anomaly.  For small angles:
// r_t ≈ r_c + 10_000 * s_hat = [6_778_000, 10_000, 0]
// (approximate; true anomaly separation ≈ 10_000 / 6_778_000 rad ≈ 0.00148 rad)
r_t_tig = [6_778_000.0, 10_000.0, 0.0]    // m  (target at TIG)
v_t_tig = [0.0, 7_669.0, 0.0]             // m/s  (target circular speed, same orbit)

dt_tpi = 600.0                              // s  (10-minute transfer)
mu     = MU_EARTH

// Propagate target forward 600 s to arrival:
// For circular orbit, target moves ~7_669 * 600 = 4_601_400 m along orbit.
// r_t_arrive ≈ kepler_step(r_t_tig, v_t_tig, 600.0, MU_EARTH)
r_t_arrive = kepler_step(r_t_tig, v_t_tig, 600.0, MU_EARTH).0
```

**Expected**:

For a circular coplanar intercept with the target ahead by a small angle,
the Lambert solution is close to the Clohessy-Wiltshire (CW/Hill) equations
result (Battin §11.5, eq. 11-75). For a 10-minute, 10 km intercept from a
400 km LEO circular orbit the expected ΔV magnitude is approximately 2–4 m/s
(dominated by the in-track S-axis component). The exact analytical value
requires the CW equations; the test tolerance is broad:

```
// Success criteria:
assert!(result.is_ok())
assert!(result.dv_mag > 0.5)    // non-trivial burn
assert!(result.dv_mag < 20.0)   // not excessively large for 10 km, 10 min
// Cross-track component should be zero for coplanar geometry:
assert!(result.dv_lvlh[2].abs() < 0.01)   // W-axis < 1 cm/s
```

**Convergence**: Lambert must converge (no `LambertNotConverged` error).

### TC-P33-2 — Zero ΔV case (chaser already on intercept trajectory)

**Setup**: Set `v_c_tig` to the exact Lambert departure velocity for the same
geometry as TC-P33-1. That is, call `lambert(r_c_tig, r_t_arrive, dt_tpi,
MU_EARTH, true)` once to get `(v_required, _)`, then set `v_c_tig = v_required`.

**Expected**:
```
assert!(result.is_ok())
assert!(result.dv_mag < 1e-6)   // effectively zero burn
assert!(result.dv_lvlh[0].abs() < 1e-6)
assert!(result.dv_lvlh[1].abs() < 1e-6)
assert!(result.dv_lvlh[2].abs() < 1e-6)
```

### TC-P33-3 — Degenerate target: zero position vector

**Setup**:
```
r_c = [6_778_000.0, 0.0, 0.0]
v_c = [0.0, 7_669.0, 0.0]
r_t_arrive = [0.0, 0.0, 0.0]   // zero target position
tof = 600.0
```

**Expected**:
```
assert_eq!(result, Err(InterceptError::DegenerateState))
```

### TC-P33-4 — Anti-parallel r1/r2 (180° transfer, undefined plane)

**Setup**:
```
r_c        = [6_778_000.0, 0.0,  0.0]
v_c        = [0.0, 7_669.0, 0.0]
r_t_arrive = [-6_778_000.0, 0.0, 0.0]   // exactly anti-parallel
tof        = 600.0
```

**Expected**:
```
assert_eq!(result, Err(InterceptError::AntiParallelVectors))
```

This confirms that `validate_lambert_inputs` catches the 180° case before the
Lambert solver is invoked.

### TC-P33-5 — Invalid TOF (below TPI_MIN_TOF_S)

**Setup**:
```
r_c = [6_778_000.0, 0.0, 0.0]
v_c = [0.0, 7_669.0, 0.0]
r_t_arrive = [6_778_000.0, 10_000.0, 0.0]
tof = 10.0   // < TPI_MIN_TOF_S = 60.0
```

**Expected**:
```
assert_eq!(result, Err(InterceptError::InvalidTimeInterval))
```

### TC-P33-6 — Integration test through `p33_init` (zero target state)

**Setup**: Construct a minimal `AgcState` with
`rendezvous_nav.target_pos = [0.0, 0.0, 0.0]` and `vn.pending_tig = Some(...)`.

**Expected**:
```
let prior = state.pending_maneuver.clone();
p33_init(&mut state);
assert_eq!(state.pending_maneuver, prior);   // unchanged
assert!(state.tpi_arrival_epoch.is_none());  // not written on alarm
// DSKY should show alarm 01440.
```

### TC-P33-7 — Elevation angle display (coplanar geometry)

**Setup**: Same geometry as TC-P33-1 at TIG (CSM at `[6_778_000, 0, 0]`,
LM at `[6_778_000, 10_000, 0]`).

**Expected elevation angle**:

The LOS vector is `[0, 10_000, 0]` (pure in-track). The chaser radius unit
vector is `[1, 0, 0]`. The elevation angle is:
```
E = asin(dot(unit([0, 10_000, 0]), unit([6_778_000, 0, 0])))
  = asin(dot([0, 1, 0], [1, 0, 0]))
  = asin(0.0)
  = 0.0 rad
```
A target directly ahead is at 0° elevation (on the local horizontal). This is
consistent with the start of TPI phasing: the crew waits until the elevation
angle rises to approximately 27.45° before executing TPI.

```
assert!((elevation_angle(r_c, r_t) - 0.0).abs() < 1e-6)
```

### TC-P34-1 — Near-zero midcourse correction (chaser on correct track)

**Setup**: Use the same geometry as TC-P33-1. Set the chaser state after TIG to
the Lambert departure state exactly (i.e., the burn was executed perfectly).
Invoke P34 with `dt_midcourse = 300.0 s` (halfway through the transfer).
The chaser is exactly on the intercept trajectory.

**Expected**:
```
assert!(result.is_ok())
assert!(result.dv_mag < 1e-3)   // near-zero correction (< 1 mm/s)
```

### TC-P34-2 — Midcourse correction for trajectory error

**Setup**: Same as TC-P34-1, but perturb the chaser velocity at "mid-transfer"
by adding `[0.0, 1.0, 0.0]` m/s (1 m/s in-track error).

**Expected**:
```
assert!(result.is_ok())
// The correction ΔV should be non-zero and approximately cancel the perturbation.
// For a 300 s remaining TOF, the correction is roughly 1 m/s order of magnitude.
assert!(result.dv_mag > 0.1)
assert!(result.dv_mag < 5.0)
```

### TC-P34-3 — P34 without prior P33 (missing tpi_arrival_epoch)

**Setup**: `state.tpi_arrival_epoch = None`, `state.vn.pending_tig = Some(...)`.

**Expected**:
```
p34_init(&mut state);
// DSKY shows alarm 01441.
assert!(state.pending_maneuver.is_none());
```

### TC-P34-4 — P34 dt_midcourse too small (arrival already passed)

**Setup**: `state.tpi_arrival_epoch = Some(100.0)` (100 s from epoch), but
current time corresponds to `tig_s = 95.0 s`, so `dt_midcourse = 5.0 s <
TPI_MIN_TOF_S`.

**Expected**:
```
// Alarm 01443; no maneuver stored.
assert!(state.pending_maneuver.is_none());
```

### TC-P34-5 — P34 within TPM_MIN_RANGE_M (already in braking range)

**Setup**: Set `csm_state.position = [6_778_000.0, 0.0, 0.0]` and
`rendezvous_nav.target_pos = [6_778_000.0, 50.0, 0.0]` (50 m separation, below
100 m threshold).

**Expected**:
```
// Alarm 01445; no maneuver stored.
assert!(state.pending_maneuver.is_none());
```

### TC-P33-8 — `validate_lambert_inputs` unit tests

Test each rejection path of `validate_lambert_inputs` independently:

| Sub-case | Input condition | Expected |
|----------|-----------------|----------|
| TC-P33-8a | `r1 = [0, 0, 0]` | `Err(DegenerateState)` |
| TC-P33-8b | `r2 = [0, 0, 0]` | `Err(DegenerateState)` |
| TC-P33-8c | `tof = 0.0` | `Err(InvalidTimeInterval)` |
| TC-P33-8d | `tof = -1.0` | `Err(InvalidTimeInterval)` |
| TC-P33-8e | `r1 = r2 = [6_778_000, 0, 0]` (zero chord) | `Err(DegenerateGeometry)` |
| TC-P33-8f | r1 anti-parallel to r2, zero cross product | `Err(AntiParallelVectors)` |
| TC-P33-8g | Valid inputs (TC-P33-1 geometry) | `Ok(())` |

---

## 10. Open Questions for Architect Review

1. **`p31_p34.rs` stub removal**: After Phase 6, `p31_p34.rs` contains only
   the two stubs `init_p33` / `init_p34`. The architect should decide whether to
   delete the file entirely or retain it as an empty module. The `mod.rs` entry
   must be updated to replace `pub mod p31_p34` with `pub mod p33; pub mod p34`.

2. **`propagate_to_tig` visibility**: The `propagate_to_tig` helper is currently
   defined in `p31.rs` and re-exported as `pub(super)`. P33 and P34 will share
   it. The architect should confirm that `pub(super)` is the right visibility,
   or move `propagate_to_tig` to a shared `programs::common` module if it is
   also needed by future programs (P37 already has its own equivalent).

3. **`AgcState::new()` const initializer**: `tpi_arrival_epoch: Option<f64>` is
   new. Since `AgcState::new()` is `const fn`, and `Option<f64>` is
   `const`-constructible, this is straightforward: add
   `tpi_arrival_epoch: None` to the `const fn new()` body.

4. **`TargetingMode` match exhaustiveness**: Adding `TpiBurn` and `TpmBurn` to
   `TargetingMode` will cause compile errors in any `match` statement over
   `TargetingMode` that lacks `_` arms. The developer must audit all match sites
   in `p40_p41.rs`, `v_n.rs`, and any test harnesses and add arms (typically
   routing `TpiBurn | TpmBurn` to the same branch as `Lambert`).

5. **Arrival velocity `_v_arrive` retention**: P33 currently discards the
   arrival velocity returned by Lambert. Future braking programs (P47 or a
   Phase 7 program) may want the predicted arrival velocity to compute a
   braking ΔV. The architect should decide whether to store `v_arrive` in
   `AgcState` now or defer it.
