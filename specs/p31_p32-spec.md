# Specification: `programs/p31` and `programs/p32` — CSI and CDH Rendezvous Targeting

**Status**: Ready for implementation (Milestone 5 Phase 5)
**Module paths**:
- `agc-core/src/programs/p31.rs` — new file
- `agc-core/src/programs/p32.rs` — new file
**Architecture reference**: `docs/architecture.md` §7.2 (P31/P32 rows), §9 (Navigation Math)
**Targeting reference**: `specs/targeting-spec.md` — `Maneuver` struct, `TargetingMode` enum,
  `burn_attitude`, `lvlh_to_inertial` / inverse
**P30 reference**: `specs/p30-spec.md` — output pattern (`state.pending_maneuver`), DSKY summary
  display, P40/P41 handoff
**Rendezvous reference**: `specs/rendezvous-spec.md` — LVLH / RSW frame conventions adopted
  throughout; see §4 for the R/S/W axis definition
**Kepler reference**: `specs/kepler-spec.md` — `kepler_step` signature (§4.2), `MU_EARTH` value
**P20 reference**: `specs/p20-spec.md` — `state.rendezvous_nav.target_pos / target_vel` field names
**P40/P41 reference**: `specs/p40_p41-spec.md` — how `state.pending_maneuver` is consumed
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, Comanche055 directory):
- `Comanche055/P31,P32.agc` — P31 and P32 entry sequences, CSI/CDH computation routines
- `Comanche055/P30,P31,P37,P40SUBROUTINES.agc` — shared targeting subroutines (IMPULSIVE,
  CDHTOCSI, S31.1)
- `Comanche055/ROUTINE_30,30.agc` — shared rendezvous targeting entry sequences (R30 common
  initialisation called by both P31 and P32)
- `Comanche055/P34,P35,P74,P75.agc` — adjacent programs for context
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `ETIG` (CSI TIG), `ECDH` (CDH TIG), `DELTAH`
  (desired altitude difference), `ELEV` (elevation angle at TPI), `N` (number of
  intermediate revolutions), `DELVEET1/2/3` (computed ΔV, scale B+7 m/s)
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
  Operation*, Springer-Praxis 2010.
- Chapter 11, "Rendezvous and Navigation Programs" pp. 303–340
  - pp. 337–340: CSI/CDH sequence summary; P31/P32 role in the nominal rendezvous timeline;
    crew workflow for both programs
- Chapter 10 (rendezvous context, mission sequence, CSI/CDH/TPI ordering)

---

## 1. Purpose and Program Roles

### 1.1 Mission context — the coelliptic rendezvous sequence

The nominal Apollo rendezvous sequence, after LM ascent, consists of four
maneuver phases:

1. **CSI** (Coelliptic Sequence Initiation) — a posigrade or retrograde burn that
   adjusts the chaser's (CSM's) orbit height so that, after N complete revolutions,
   the two vehicles are at the correct geometry for the CDH burn.
2. **CDH** (Constant Delta-Height) — a small burn that makes the chaser's orbit
   coelliptic with the target: same eccentricity vector magnitude, same orbital plane,
   constant altitude separation Δh (typically 10 nmi ≈ 18,520 m).
3. **TPI** (Terminal Phase Initiation) — a burn that initiates a direct intercept
   transfer from the coelliptic orbit to the target; computed by P34.
4. **TPM** / mid-course corrections — small RCS trim burns; computed by P35.

P31 computes the CSI burn (step 1). P32 computes the CDH burn (step 2). Both
programs deposit their result in `state.pending_maneuver` for execution by P40
(SPS) or P41 (RCS). P34 and P35 handle the subsequent phases.

The coelliptic sequence is preferred when the LM insertion orbit leaves a large
phasing problem. If the LM insertion is near-perfect, Mission Control may elect to
skip the coelliptic phase and use a direct Lambert intercept via P34 instead.

### 1.2 P31 — Coelliptic Sequence Initiation (CSI)

P31 computes the CSI burn to be performed at a crew-specified TIG (Time of
Ignition). The burn is in the local horizontal plane (pure in-track, S-axis in
LVLH) and adjusts the chaser's orbital period so that it arrives at the correct
geometry for the CDH burn exactly one half-revolution of the target orbit later.
After the CSI burn the chaser and target are on coplanar orbits with a height
difference Δh that will be maintained (on average) through the CDH burn.

The CSI computation in Comanche055 is **iterative**: it contains a 1-D Newton
solver over the CSI ΔV magnitude. The cost function is the out-of-plane (W-axis)
component of the required CDH ΔV — at convergence, the CDH burn is purely
in-plane (a pure S-axis correction), which is the definition of the coelliptic
geometry. Convergence is declared when `|Δv_csi_update| < 0.01 m/s`; a maximum
of 10 iterations is enforced (see §5.1).

### 1.3 P32 — Constant Delta-Height (CDH)

P32 computes the CDH burn to be performed at a crew-specified CDH TIG. The CDH
burn is always in the local horizontal plane (S-axis dominant; small R-axis
component is possible for eccentric orbits). Its purpose is to set up the exact
coelliptic geometry required for TPI: the chaser's orbit must have the same
eccentricity vector as the target's orbit, separated by exactly Δh in altitude.

P32 is a closed-form computation (no iteration): given the propagated state vectors
at the CDH epoch, the required post-burn velocity is computed directly from the
target's orbital elements. See §5.2.

---

## 2. Module Paths

- P31: `agc-core/src/programs/p31.rs`
- P32: `agc-core/src/programs/p32.rs`

Both are registered in `agc-core/src/programs/mod.rs`:

```rust
pub mod p31;
pub mod p32;
```

Entry points are registered in `PROGRAM_TABLE`:

- `PROGRAM_TABLE[31] = p31_init`
- `PROGRAM_TABLE[32] = p32_init`

---

## 3. State Additions

No new fields are added to `AgcState`. All inputs come from existing fields; both
programs write their result to the existing `state.pending_maneuver` field.

### 3.1 Input fields consumed

| `AgcState` field | Type | Purpose |
|-----------------|------|---------|
| `state.csm_state.pos` | `Vec3` (m, inertial) | Chaser position at current epoch |
| `state.csm_state.vel` | `Vec3` (m/s, inertial) | Chaser velocity at current epoch |
| `state.csm_state.epoch` | `Met` (centiseconds) | Epoch of chaser state |
| `state.rendezvous_nav.target_pos` | `Vec3` (m, inertial) | Target position (from P20) |
| `state.rendezvous_nav.target_vel` | `Vec3` (m/s, inertial) | Target velocity (from P20) |
| `state.rendezvous_nav.target_epoch` | `Met` (centiseconds) | Epoch of target state |
| `state.vn.pending_tig` | `Met` (centiseconds) | Crew-entered TIG for the upcoming burn |

### 3.2 Output field written

| `AgcState` field | Type | Content after P31/P32 |
|-----------------|------|----------------------|
| `state.pending_maneuver` | `Option<Maneuver>` | The computed CSI or CDH burn |

`state.pending_maneuver` is set to `Some(maneuver)` on success and left unchanged
(i.e. any prior value is preserved) on alarm. The `Maneuver.mode` field distinguishes
the burn type for the P40 burn monitor.

### 3.3 `TargetingMode` additions

The existing `TargetingMode` enum (in `guidance::targeting`) currently lists
`ExternalDeltaV`, `Lambert`, and `ReturnToEarth`. Two new variants are added:

```rust
/// P31 — Coelliptic Sequence Initiation burn.
///
/// Delta-V computed by the CSI Newton iteration (pure in-track, S-axis, LVLH).
/// DSKY: V06 N37 (TIG / ΔV summary), V06 N84 (ΔV in LVLH for display).
CsiBurn,

/// P32 — Constant Delta-Height burn.
///
/// Delta-V computed by the CDH closed-form formula (in-plane, LVLH).
/// DSKY: V06 N37 (TIG / ΔV summary), V06 N84 (ΔV in LVLH for display).
CdhBurn,
```

The architect should assess whether adding variants to `TargetingMode` requires a
`match` arm in `programs::p40_p41`; the only required change is that the P40
display branch treats `CsiBurn` and `CdhBurn` identically to `Lambert` (same
noun table: V06 N84).

---

## 4. Public API

### 4.1 Constants

```rust
// agc-core/src/programs/p31.rs

/// Major mode number for P31.
pub const P31_MAJOR_MODE: u8 = 31;

/// Job priority for P31. Same as P30/P34 (foreground targeting programs).
pub const P31_PRIORITY: JobPriority = 10;

/// Default desired altitude separation at coelliptic (m).
/// 10 nmi = 18,520 m exactly (1 nmi = 1852 m). Used when the crew has not
/// entered a custom Δh via V06 N58.
/// AGC erasable: DELTAH, scale B+28 m.
pub const DELTA_H_DEFAULT_M: f64 = 18_520.0;

/// Convergence tolerance for the CSI Newton iteration (m/s).
/// The iteration terminates when |Δv_csi_correction| < CSI_CONVERGE_TOL.
/// Chosen to be consistent with the AGC's ~0.01 ft/s display resolution
/// (≈ 0.003 m/s); 0.01 m/s is a safe conservative bound.
pub const CSI_CONVERGE_TOL: f64 = 0.01; // m/s

/// Maximum Newton iterations for the CSI solver.
/// Consistent with the AGC's hard loop-count limit on targeting iterations.
pub const CSI_MAX_ITER: u16 = 10;

/// Minimum in-track burn magnitude to consider CSI non-trivial (m/s).
/// If the computed |Δv_csi| < CSI_MIN_DV the result is still stored but flagged
/// as a near-zero burn in the DSKY display.
pub const CSI_MIN_DV: f64 = 0.1; // m/s
```

```rust
// agc-core/src/programs/p32.rs

/// Major mode number for P32.
pub const P32_MAJOR_MODE: u8 = 32;

/// Job priority for P32. Same as P31.
pub const P32_PRIORITY: JobPriority = 10;

/// Minimum chaser-target radial separation at CDH TIG below which the geometry
/// is considered degenerate (m). If |r_c_cdh| - |r_t_cdh| < CDH_MIN_DELTAH,
/// alarm 01432 is raised.
pub const CDH_MIN_DELTAH: f64 = 1_000.0; // m
```

### 4.2 P31 entry point

```rust
/// Entry point for P31 (Coelliptic Sequence Initiation).
/// Registered in PROGRAM_TABLE[31].
///
/// Sets `state.major_mode = 31`. Prompts the crew for CSI TIG (V06 N37),
/// CDH TIG (V06 N37 — second entry), and desired Δh (V06 N58). On crew
/// acceptance calls `compute_csi_delta_v` and displays the result via V06 N84.
/// Stores the result in `state.pending_maneuver` with `mode = CsiBurn`.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` and `target_vel` must be non-zero
///   (target state must exist); otherwise alarm 01430 is raised.
/// - `state.vn.pending_tig` must be set to the desired CSI TIG (crew-entered
///   before `p31_init` is called in the current milestone implementation).
///
/// # Post-conditions (success)
/// - `state.major_mode == 31`
/// - `state.dsky.prog == 31`
/// - `state.pending_maneuver == Some(csi_maneuver)` where
///   `csi_maneuver.mode == TargetingMode::CsiBurn`
/// - DSKY displays ΔV components via V06 N84.
///
/// # Post-conditions (alarm)
/// - `state.pending_maneuver` is unchanged.
/// - DSKY displays the alarm code.
///
/// # Alarms
/// - 01430: target state is zero (P20 never ran or radar failure).
/// - 01431: CSI Newton iteration did not converge in CSI_MAX_ITER steps.
/// - 01433: chaser/target states have incompatible epochs after propagation.
pub fn p31_init(state: &mut AgcState) -> JobPriority
```

### 4.3 P31 pure-computation helper

```rust
/// Compute the CSI delta-V vector (in LVLH frame) given raw inertial state vectors.
///
/// This is the pure-math core of P31, separated from `p31_init` so tests can
/// exercise it without a full `AgcState`. The function implements the 1-D Newton
/// iteration described in §5.1.
///
/// # Arguments
/// - `r_c`: Chaser position at the CSI TIG (m, inertial).
/// - `v_c`: Chaser velocity at the CSI TIG (m/s, inertial).
/// - `r_t_cdh`: Target position at the CDH TIG (m, inertial). The caller must
///   propagate the target state from its current epoch to the CDH epoch using
///   `kepler_step` before calling this function.
/// - `v_t_cdh`: Target velocity at the CDH TIG (m/s, inertial).
/// - `dt_csi_to_cdh`: Time from CSI TIG to CDH TIG (s). Must be positive.
/// - `delta_h`: Desired coelliptic altitude difference (m). Positive means
///   chaser is below the target. Typically `DELTA_H_DEFAULT_M` (18,520 m).
/// - `mu`: Gravitational parameter (m³/s²). Use `MU_EARTH` for Earth orbit.
///
/// # Returns
/// `Ok(CsiResult)` on convergence, `Err(CsiError)` on non-convergence or
/// degenerate geometry.
///
/// # Algorithm
/// See §5.1 for the full step-by-step derivation.
pub fn compute_csi_delta_v(
    r_c:            Vec3,
    v_c:            Vec3,
    r_t_cdh:        Vec3,
    v_t_cdh:        Vec3,
    dt_csi_to_cdh:  f64,
    delta_h:        f64,
    mu:             f64,
) -> Result<CsiResult, CsiError>
```

```rust
/// Result of a successful CSI computation.
#[derive(Clone, Copy, Debug)]
pub struct CsiResult {
    /// CSI delta-V in the LVLH frame at the CSI TIG (m/s).
    /// Component [0] = R (radial), [1] = S (in-track), [2] = W (cross-track).
    /// For a nominal coplanar CSI, [0] ≈ 0, [2] ≈ 0; [1] dominates.
    pub dv_lvlh: Vec3,

    /// Number of Newton iterations taken to reach convergence.
    pub iter_count: u16,

    /// Residual (magnitude of the final Newton correction) at convergence (m/s).
    pub residual: f64,
}

/// Error conditions for the CSI computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CsiError {
    /// Newton iteration did not converge in CSI_MAX_ITER steps.
    NotConverged,
    /// Target or chaser position vector is zero (degenerate state).
    DegenerateState,
    /// dt_csi_to_cdh is non-positive.
    InvalidTimeInterval,
}
```

### 4.4 P32 entry point

```rust
/// Entry point for P32 (Constant Delta-Height).
/// Registered in PROGRAM_TABLE[32].
///
/// Sets `state.major_mode = 32`. Prompts the crew for CDH TIG (V06 N37).
/// Calls `compute_cdh_delta_v` and displays the result via V06 N84.
/// Stores the result in `state.pending_maneuver` with `mode = CdhBurn`.
///
/// # Preconditions
/// - `state.rendezvous_nav.target_pos` and `target_vel` must be non-zero;
///   otherwise alarm 01430 is raised.
/// - `state.vn.pending_tig` must hold the CDH TIG (crew-entered before this call).
///
/// # Post-conditions (success)
/// - `state.major_mode == 32`
/// - `state.dsky.prog == 32`
/// - `state.pending_maneuver == Some(cdh_maneuver)` where
///   `cdh_maneuver.mode == TargetingMode::CdhBurn`
///
/// # Alarms
/// - 01430: target state zero.
/// - 01432: degenerate CDH geometry (|Δr_radial| < CDH_MIN_DELTAH).
pub fn p32_init(state: &mut AgcState) -> JobPriority
```

### 4.5 P32 pure-computation helper

```rust
/// Compute the CDH delta-V vector (in LVLH frame) given raw inertial state vectors.
///
/// Pure-math core of P32, usable in tests without `AgcState`.
/// Implements the closed-form coelliptic matching described in §5.2.
///
/// # Arguments
/// - `r_c`: Chaser position at CDH TIG (m, inertial). Caller must propagate
///   chaser state from its epoch to CDH TIG using `kepler_step`.
/// - `v_c`: Chaser velocity at CDH TIG (m/s, inertial).
/// - `r_t`: Target position at CDH TIG (m, inertial).
/// - `v_t`: Target velocity at CDH TIG (m/s, inertial).
/// - `delta_h`: Desired altitude separation (m). Same value used in P31.
///   Positive means chaser is below target.
/// - `mu`: Gravitational parameter (m³/s²).
///
/// # Returns
/// `Ok(CdhResult)` on success, `Err(CdhError)` if geometry is degenerate.
///
/// # Algorithm
/// See §5.2 for the full step-by-step derivation.
pub fn compute_cdh_delta_v(
    r_c:     Vec3,
    v_c:     Vec3,
    r_t:     Vec3,
    v_t:     Vec3,
    delta_h: f64,
    mu:      f64,
) -> Result<CdhResult, CdhError>
```

```rust
/// Result of a successful CDH computation.
#[derive(Clone, Copy, Debug)]
pub struct CdhResult {
    /// CDH delta-V in LVLH frame (m/s).
    /// Component [0] = R (radial), [1] = S (in-track), [2] = W (cross-track).
    /// For a coplanar CDH, [2] ≈ 0; [0] and [1] are both non-zero in general.
    pub dv_lvlh: Vec3,
}

/// Error conditions for the CDH computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CdhError {
    /// Radial separation at CDH is below CDH_MIN_DELTAH — cannot establish
    /// a meaningful coelliptic geometry.
    DegenerateGeometry,
    /// Target or chaser position is the zero vector.
    DegenerateState,
}
```

### 4.6 Shared propagation helper (module-private)

Both programs need to propagate state vectors from the current epoch to the burn
TIG. An internal helper is defined in a shared private scope:

```rust
/// Propagate an inertial state vector from `epoch_cs` (centiseconds) to `tig_cs`
/// using `math::kepler::kepler_step`.
///
/// Returns `(pos_at_tig, vel_at_tig)`.
/// Panics if `tig_cs < epoch_cs` (targeting into the past is a caller error).
fn propagate_to_tig(
    pos:      Vec3,
    vel:      Vec3,
    epoch_cs: Met,
    tig_cs:   Met,
    mu:       f64,
) -> (Vec3, Vec3)
```

This helper converts `Met` (centiseconds) to seconds by dividing by 100.0 and
calls `math::kepler::kepler_step(pos, vel, dt_s, mu)`. It is defined in
`p31.rs` and re-exported to `p32.rs` via `pub(super)`.

---

## 5. Algorithm Specifications

### 5.1 P31 — CSI Newton Iteration

#### Overview

The CSI burn is constrained to the local horizontal plane at the CSI TIG.
Because the pre-CSI orbit may be slightly non-circular or out-of-plane relative
to the target, a trial in-track ΔV produces a post-CSI orbit that, when
propagated forward to the CDH TIG, may have a cross-track component relative to
the target. The Newton iteration adjusts the in-track ΔV until the post-burn
orbit is coelliptic (cross-track error → 0) and achieves the target Δh.

The cost function is: **the out-of-plane (W-axis) component of the ΔV required
at the CDH epoch to match the coelliptic orbit**. At convergence, the W-axis
component is zero and the required CDH burn is purely in-plane.

This is the algorithm implemented in `P31,P32.agc` subroutine S31.1 and
described at a high level in O'Brien pp. 337–340.

#### Step-by-step

**Inputs to the iteration** (all at CSI TIG, inertial):
- `r_c`: chaser position (m)
- `v_c`: chaser velocity (m/s)
- `r_t_cdh`, `v_t_cdh`: target state at CDH TIG (m, m/s)
- `dt_csi_to_cdh`: CSI-to-CDH interval (s)
- `delta_h`: desired altitude separation (m)
- `mu`: gravitational parameter (m³/s²)

**Step 0 — Validate inputs**

```
if norm(r_c) < 1e6 m  →  return Err(DegenerateState)
if norm(r_t_cdh) < 1e6 m  →  return Err(DegenerateState)
if dt_csi_to_cdh <= 0  →  return Err(InvalidTimeInterval)
```

**Step 1 — Initial estimate for in-track ΔV**

Compute the chaser's current mean motion and the target's radius at CDH:

```
r_c_mag = norm(r_c)                                         (m)
r_t_cdh_mag = norm(r_t_cdh)                                 (m)
a_c = -mu / (v_c·v_c - 2*mu/r_c_mag)                       Vis-viva: semi-major axis (m)
      Note: v_c·v_c = dot(v_c, v_c)
```

The desired chaser semi-major axis after the CSI burn is derived from the target's
orbit height at CDH. The target's radial distance at CDH defines the "upper" orbit;
the chaser should be `delta_h` below it at CDH (the chaser is below the target in
the nominal profile):

```
r_c_desired_cdh = r_t_cdh_mag - delta_h                     (m)
```

Use Hohmann transfer intuition for the initial ΔV estimate: the chaser raises (or
lowers) its orbit so that its radius at CDH TIG matches `r_c_desired_cdh`. For a
half-revolution transfer from current chaser apoapsis to CDH, the transfer
semi-major axis is:

```
a_transfer = (r_c_mag + r_c_desired_cdh) / 2.0              (m)
```

The required speed at perigee of this transfer (= current chaser altitude) is:

```
v_required = sqrt(mu * (2.0/r_c_mag - 1.0/a_transfer))      (m/s)
```

The current chaser speed in the S-direction (in-track component) is:

```
v_c_s = dot(v_c, s_hat)
where s_hat = unit(cross(cross(r_c, v_c), r_c))             (LVLH S-axis)
```

Initial in-track ΔV estimate:

```
dv_s_0 = v_required - v_c_s                                  (m/s)
```

(This is an approximation; for a circular initial orbit it reduces to the exact
Hohmann result. Battin §11.1, eq. 11-10.)

**Step 2 — Newton iteration**

The iteration variable is `dv_s`, the S-axis ΔV component. The W-axis ΔV
component starts at 0 (a coplanar CSI is desired).

For each iteration `k = 0, 1, ..., CSI_MAX_ITER-1`:

a) **Apply trial CSI burn** — construct the post-CSI chaser velocity:

   ```
   r_hat = unit(r_c)                                         (R-axis unit vector)
   h_hat = unit(cross(r_c, v_c))                             (orbit normal)
   s_hat = unit(cross(h_hat, r_hat))                         (S-axis; Battin §11 convention)
   w_hat = h_hat                                             (W-axis = orbit normal)

   v_post_csi = v_c + dv_s * s_hat                           (m/s, inertial)
   ```

   Note: a pure coplanar CSI has zero R-axis and W-axis components.

b) **Propagate chaser from CSI TIG to CDH TIG**:

   ```
   (r_c_cdh, v_c_cdh) = kepler_step(r_c, v_post_csi, dt_csi_to_cdh, mu)
   ```

c) **Compute required CDH ΔV** using the closed-form CDH formula (step §5.2)
   applied to `(r_c_cdh, v_c_cdh, r_t_cdh, v_t_cdh, delta_h, mu)`. Let the result
   be `dv_cdh_lvlh`:

   ```
   dv_cdh_w = dv_cdh_lvlh[2]                                 (W-axis CDH residual, m/s)
   ```

d) **Convergence check**:

   ```
   if |dv_cdh_w| < CSI_CONVERGE_TOL  →  converged; store result
   ```

e) **Newton step** — compute numerical derivative of `dv_cdh_w` w.r.t. `dv_s`:

   ```
   eps = max(0.01 * |dv_s| + 0.001, 0.01)                   (finite-difference step, m/s)
   
   v_perturbed = v_c + (dv_s + eps) * s_hat
   (r_c_cdh_p, v_c_cdh_p) = kepler_step(r_c, v_perturbed, dt_csi_to_cdh, mu)
   dv_cdh_w_p = compute_cdh_delta_v(r_c_cdh_p, v_c_cdh_p, r_t_cdh, v_t_cdh, delta_h, mu)
                 .map(|r| r.dv_lvlh[2]).unwrap_or(dv_cdh_w)

   d_dv_cdh_w_d_dv_s = (dv_cdh_w_p - dv_cdh_w) / eps
   ```

f) **Guard against zero derivative** (degenerate orbit geometry):

   ```
   if |d_dv_cdh_w_d_dv_s| < 1e-6  →  break, return Err(NotConverged)
   ```

g) **Update**:

   ```
   dv_s = dv_s - dv_cdh_w / d_dv_cdh_w_d_dv_s
   ```

After `CSI_MAX_ITER` iterations without convergence: return `Err(NotConverged)`.

**Step 3 — Build LVLH ΔV and result**

After convergence, the CSI ΔV in LVLH is:

```
dv_csi_lvlh = [0.0, dv_s, 0.0]                               (R, S, W components, m/s)
```

Return `Ok(CsiResult { dv_lvlh: dv_csi_lvlh, iter_count: k+1, residual: |dv_cdh_w| })`.

**Step 4 — Build `Maneuver` in `p31_init`**

The entry-point function converts the LVLH ΔV to inertial and constructs the
`Maneuver`:

```
dv_inertial = lvlh_to_inertial(dv_csi_lvlh, r_c, v_c)        (m/s, inertial)
attitude    = burn_attitude(dv_inertial, state.refsmmat)
tig         = state.vn.pending_tig

maneuver = Maneuver {
    tig,
    delta_v: dv_inertial,
    burn_attitude: attitude,
    mode: TargetingMode::CsiBurn,
}
state.pending_maneuver = Some(maneuver)
```

Both `lvlh_to_inertial` and `burn_attitude` are functions in `guidance::targeting`
per `specs/targeting-spec.md` §3.

---

### 5.2 P32 — CDH Closed-Form Computation

#### Overview

The CDH burn adjusts the chaser's orbit so that it becomes **coelliptic** with
the target: the two orbits lie in the same plane (already ensured by CSI) and
have the same eccentricity vector magnitude, so that their altitude separation
remains constant through one complete revolution (Battin §11.4, pp. 534–540).

The coelliptic condition at any point on the orbit can be stated in terms of
specific angular momenta:

```
h_c_post = |r_t| * v_t_perp  ·  r_c_post / r_t          (Battin 11-53, simplified)
```

where `v_t_perp` is the component of target velocity perpendicular to the target
radius vector. In the Comanche055 implementation this is solved directly from the
orbit energy constraint, yielding a closed-form expression for the required post-burn
chaser speed.

#### Step-by-step

**Inputs** (all at CDH TIG, inertial):
- `r_c`: chaser position (m)
- `v_c`: chaser velocity (m/s)
- `r_t`: target position (m)
- `v_t`: target velocity (m/s)
- `delta_h`: desired altitude separation (m)
- `mu`: gravitational parameter (m³/s²)

**Step 0 — Validate**

```
r_c_mag = norm(r_c)
r_t_mag = norm(r_t)
if r_c_mag < 1e6 or r_t_mag < 1e6  →  return Err(DegenerateState)
delta_r_actual = r_t_mag - r_c_mag                            (+ means target above chaser)
if |delta_r_actual| < CDH_MIN_DELTAH  →  return Err(DegenerateGeometry)
```

**Step 1 — Target orbital angular momentum**

Compute the target's specific angular momentum vector and its in-plane transverse
unit vector:

```
h_t_vec = cross(r_t, v_t)                                     (m²/s, target angular momentum)
h_t     = norm(h_t_vec)                                       (m²/s, scalar)
r_t_hat = unit(r_t)                                            (radial unit vector at CDH)
w_hat   = unit(h_t_vec)                                        (orbit normal = W-axis at CDH)
s_t_hat = unit(cross(w_hat, r_t_hat))                          (S-axis at CDH in target frame)
```

**Step 2 — Required post-burn chaser angular momentum**

For the coelliptic condition the chaser's post-burn specific angular momentum
magnitude must equal the target's angular momentum scaled by the ratio of their
radii (Battin §11.4, eq. 11-53):

```
r_c_required = r_t_mag - delta_h                               (m; chaser radius at CDH)
h_c_required = h_t * r_c_required / r_t_mag                   (m²/s)
```

This is the key coelliptic relation. It ensures that at any angular position
`θ` measured from CDH, the altitude difference

```
Δh(θ) = r_t(θ) - r_c(θ)   ≈   delta_h   (constant to first order in Δh/r)
```

**Step 3 — Required post-burn in-track speed**

The chaser's post-burn speed must produce the required angular momentum while the
chaser remains at radius `r_c_mag` (the CDH burn is impulsive — no radial change):

```
v_c_s_required = h_c_required / r_c_mag                       (m/s; in-track speed required)
```

This follows from `h = r * v_perp` for a circular/near-circular burn where the
velocity is approximately perpendicular to the radius vector at the burn point.

For eccentric orbits there is also a radial component of the current velocity that
does not change (CDH is purely in-track in Comanche055):

```
v_c_r = dot(v_c, r_t_hat)                                     (current radial speed, m/s)
v_c_s = dot(v_c, s_t_hat)                                     (current in-track speed, m/s)
```

The post-burn velocity vector is:

```
v_c_post_r = v_c_r                                             (radial unchanged)
v_c_post_s = v_c_s_required                                    (in-track adjusted)
v_c_post   = v_c_post_r * r_t_hat + v_c_post_s * s_t_hat + 0 * w_hat   (m/s, inertial)
```

**Step 4 — Delta-V in inertial and LVLH**

```
dv_inertial = v_c_post - v_c                                   (m/s, inertial)
```

Convert to LVLH at the CDH TIG (chaser LVLH frame):

```
r_c_hat = unit(r_c)
h_c_vec = cross(r_c, v_c)
w_c_hat = unit(h_c_vec)                                        (chaser orbit normal)
s_c_hat = unit(cross(w_c_hat, r_c_hat))

dv_lvlh[0] = dot(dv_inertial, r_c_hat)                        (R component)
dv_lvlh[1] = dot(dv_inertial, s_c_hat)                        (S component)
dv_lvlh[2] = dot(dv_inertial, w_c_hat)                        (W component)
```

Return `Ok(CdhResult { dv_lvlh })`.

**Step 5 — Build `Maneuver` in `p32_init`**

```
(r_c_cdh, v_c_cdh) = propagate_to_tig(
    state.csm_state.pos, state.csm_state.vel,
    state.csm_state.epoch, state.vn.pending_tig, MU_EARTH)

(r_t_cdh, v_t_cdh) = propagate_to_tig(
    state.rendezvous_nav.target_pos, state.rendezvous_nav.target_vel,
    state.rendezvous_nav.target_epoch, state.vn.pending_tig, MU_EARTH)

cdh = compute_cdh_delta_v(r_c_cdh, v_c_cdh, r_t_cdh, v_t_cdh,
                          delta_h, MU_EARTH)?

dv_inertial = lvlh_to_inertial(cdh.dv_lvlh, r_c_cdh, v_c_cdh)
attitude    = burn_attitude(dv_inertial, state.refsmmat)

state.pending_maneuver = Some(Maneuver {
    tig:           state.vn.pending_tig,
    delta_v:       dv_inertial,
    burn_attitude: attitude,
    mode:          TargetingMode::CdhBurn,
})
```

---

### 5.3 Orbital mechanics identities used

The following standard identities underpin both algorithms. Reference numbers
are from Battin, *An Introduction to the Mathematics and Methods of
Astrodynamics*, AIAA Education Series, 1987.

| Equation | Identity | Reference |
|----------|----------|-----------|
| Vis-viva | `v² = μ(2/r - 1/a)` | Battin §3.1, eq. 3-11 |
| Specific angular momentum | `h = norm(cross(r, v))` | Battin §2.3, eq. 2-19 |
| Coelliptic condition | `h_c / h_t = r_c / r_t` | Battin §11.4, eq. 11-53 |
| LVLH R-axis | `r_hat = unit(r)` | specs/rendezvous-spec.md §4 |
| LVLH W-axis | `w_hat = unit(cross(r, v))` | specs/rendezvous-spec.md §4 |
| LVLH S-axis | `s_hat = unit(cross(w_hat, r_hat))` | specs/rendezvous-spec.md §4 |

The coelliptic condition `h_c / h_t = r_c / r_t` (Battin eq. 11-53) is the
mathematical core of both programs. CSI iterates on the CSI burn magnitude until
this condition (equivalently: zero W-axis CDH residual) will hold at the CDH
epoch. CDH enforces this condition directly by solving for the required in-track
speed.

---

## 6. DSKY Interaction

### 6.1 P31 DSKY sequence

| Step | Display | Verb/Noun | Content | Crew action |
|------|---------|-----------|---------|-------------|
| 1 | Major mode | V35 N31 | Prog 31 | Automatic |
| 2 | Request CSI TIG | V06 N37 flashing | R1 = hours, R2 = minutes, R3 = seconds of GET | `V25 E` then enter TIG |
| 3 | Request CDH TIG | V06 N37 flashing | Re-display for second entry | `V25 E` then enter CDH TIG |
| 4 | Request Δh | V06 N58 flashing | R1 = Δh in feet (×10⁻¹ display) | `V25 E` enter Δh, or `V34 E` accept default |
| 5 | Computation | — | COMP lamp on during iteration | Automatic |
| 6 | ΔV display | V06 N84 | R1 = ΔVx, R2 = ΔVy, R3 = ΔVz in ft/s (×10⁻¹) | Crew verification |
| 7 | TIG display | V06 N37 | R1 = TIG hours, R2 = minutes, R3 = seconds | Crew acceptance `V33 E` |
| 8 | Standby | — | Await `V37 E 40 E` (P40) or `V37 E 41 E` (P41) | Crew selects execution program |

**AGC Verb/Noun references** (from `Comanche055/P31,P32.agc` and
`Comanche055/ROUTINE_30,30.agc`):

- **V06 N37** — Time display (TIG in GET H:M:S). Used throughout the rendezvous
  program family for TIG entry and verification. AGC erasable `ETIG` (CSI TIG).
- **V06 N58** — Altitude difference display. Erasable `DELTAH` (scale B+28 m;
  converted to/from feet for DSKY display at 1 ft = 0.3048 m).
- **V06 N84** — ΔV components in LVLH (ft/s × 10⁻¹). Erasable `DELVEET1/2/3`
  (scale B+7 m/s). Same noun used by P34 for TPI ΔV display.
- **V33** — Proceed (crew acceptance). Cancels the flashing request.
- **V34** — Accept default (used at step 4 to accept `DELTA_H_DEFAULT_M`).

### 6.2 P32 DSKY sequence

| Step | Display | Verb/Noun | Content | Crew action |
|------|---------|-----------|---------|-------------|
| 1 | Major mode | V35 N32 | Prog 32 | Automatic |
| 2 | Request CDH TIG | V06 N37 flashing | CDH TIG in GET | `V25 E` enter TIG |
| 3 | Computation | — | COMP lamp on | Automatic (closed-form, fast) |
| 4 | ΔV display | V06 N84 | CDH ΔV components | Crew verification |
| 5 | TIG display | V06 N37 | CDH TIG | Crew acceptance `V33 E` |
| 6 | Standby | — | Await P40/P41 selection | Crew selects execution |

P32 does not request Δh again; it uses the value already stored from the P31
session (in `state.pending_delta_h`, or equivalently, P32 re-reads
`DELTA_H_DEFAULT_M` unless P31 already wrote a crew-modified value).

**Implementation note**: In this milestone the interactive DSKY data-load
sequences (V25 flashing entry) are modeled as direct arguments to `p31_init` and
`p32_init` (same pattern as P30 per `specs/p30-spec.md` §1.2). The Verb/Noun
assignments above are recorded for fidelity and for Milestone 6 (DSKY
interactive flow).

---

## 7. Program Alarms

| Alarm code | Condition | Program | Recovery |
|------------|-----------|---------|----------|
| 01430 | Target state is zero (`norm(target_pos) < 1 m`) — P20 never ran or tracking was lost. | P31, P32 | Raise alarm, do not modify `pending_maneuver`, return to idle. Crew must run P20 to establish target state before retrying. |
| 01431 | CSI Newton iteration did not converge in `CSI_MAX_ITER` steps. | P31 | Raise alarm, do not store maneuver. Display last `dv_s` estimate on DSKY for crew awareness; crew may retry with a different TIG. |
| 01432 | CDH geometry degenerate: `|r_c_mag - r_t_mag| < CDH_MIN_DELTAH` (chaser and target nearly at same altitude at CDH epoch). The burn cannot achieve the coelliptic condition. | P32 (and inner CDH call within P31 iteration) | Raise alarm, abort. Crew must re-enter a different CDH TIG or re-run P31 to establish a valid post-CSI orbit. |
| 01433 | Target state epoch is more than 30 minutes stale relative to CSI/CDH TIG: `(tig - target_epoch) > 180_000 cs`. The propagated target state uncertainty is too large to trust. | P31, P32 | Raise alarm but proceed with computation; display staleness warning on DSKY. Crew may accept or re-run P20 for a fresher measurement. |

Alarm codes follow the established rendezvous alarm numbering convention. Codes
01420–01429 are used by P20/P21/P22/P23 (per `specs/p20-spec.md` §8). Codes
01430–01439 are reserved for this phase.

---

## 8. Edge Cases

### 8.1 CSI Newton non-convergence (alarm 01431)

If `CSI_MAX_ITER` is exhausted, the function returns `Err(NotConverged)`.
`p31_init` raises alarm 01431 and does NOT store a maneuver.

The cause is typically one of:
- The CDH TIG is so close to the CSI TIG that the propagation produces insufficient
  lever arm for the iteration to resolve the coelliptic condition.
- The initial orbit geometry is degenerate (very high eccentricity, near-escape
  trajectory), causing the finite-difference derivative to be near-zero.

Mitigation: the minimum CDH-to-CSI interval should be at least 30 minutes
(≈ 0.5 orbital periods of a low lunar or low Earth orbit). The spec does not
enforce this at the function level; the crew must select a physically reasonable
pair of TIGs.

### 8.2 Chaser already ahead of target (phasing reversed)

If the chaser's true anomaly is ahead of the target and the required CSI burn
is retrograde (negative `dv_s`), the algorithm handles this naturally: the
initial estimate will be negative and the Newton iteration will converge to a
retrograde solution. No special-casing is needed.

### 8.3 Target on hyperbolic trajectory

The AGC assumes the target (LM) is in a closed orbit. If for any reason
`norm(v_t)² > 2 * mu / r_t_mag` (hyperbolic), `kepler_step` will propagate
correctly (universal-variable formulation covers hyperbolic arcs per
`specs/kepler-spec.md` §1), but the CDH coelliptic condition becomes
physically meaningless. Guard in `compute_cdh_delta_v`:

```
energy_t = dot(v_t, v_t) / 2.0 - mu / r_t_mag
if energy_t >= 0.0  →  return Err(DegenerateState)
```

This check is applied at the start of `compute_cdh_delta_v`.

### 8.4 Target state never set (`target_pos == [0, 0, 0]`)

```
if norm(state.rendezvous_nav.target_pos) < 1.0  →  alarm 01430
```

This check runs at the very start of both `p31_init` and `p32_init`, before any
computation. Using `< 1.0 m` as the threshold is conservative: even the lowest
possible LM orbit (barely above the lunar surface) has `r > 1.7e6 m`.

### 8.5 CDH TIG before CSI TIG

In `p31_init`, after the crew enters both TIGs, the software computes
`dt_csi_to_cdh = (cdh_tig - csi_tig) / 100.0 s`. If this value is ≤ 0, the
function raises alarm 01431 (same as non-convergence, used as a generic targeting
failure) and returns without computing.

### 8.6 Zero-ΔV case

If the chaser is already on the coelliptic orbit (extremely unlikely in practice,
but possible in simulation), the initial estimate gives `dv_s ≈ 0` and the first
convergence check passes immediately. The stored `Maneuver` has `delta_v ≈ [0,0,0]`
and `burn_attitude = Mat3x3::IDENTITY` (per the `Maneuver` invariant in
`specs/targeting-spec.md` §3.1). P40 will execute a near-zero burn correctly.

---

## 9. Test Cases

### TC-P31-1 — Zero initial phase error (chaser and target in identical circular orbits)

**Setup**:
```
r_c = [6_571_000.0, 0.0, 0.0]              m  (LEO, 200 km altitude)
v_c = [0.0, 7_784.0, 0.0]                  m/s  (circular speed)
r_t (at CDH TIG) = [6_589_520.0, 0.0, 0.0] m  (18,520 m = 10 nmi above chaser)
v_t (at CDH TIG) = [0.0, 7_773.0, 0.0]     m/s  (circular speed at r_t)
dt_csi_to_cdh = 2700.0 s   (45 min ≈ half-period of ~200 km orbit)
delta_h = 18_520.0 m
mu = MU_EARTH
```

**Expected**: `dv_csi_lvlh[1]` (S-axis) is the Hohmann transfer ΔV to raise the
chaser from 6,571 km to the transfer orbit apoapsis that reaches 6,589.52 km:

```
a_transfer = (6_571_000 + 6_589_520) / 2 = 6_580_260 m
v_required = sqrt(MU_EARTH * (2/6_571_000 - 1/6_580_260)) ≈ 7_794.2 m/s
dv_s = 7_794.2 - 7_784.0 ≈ 10.2 m/s
```

**Tolerance**: `|computed dv_s - 10.2| < 0.5 m/s` (Hohmann approximation is exact
for circular; any residual is numerical).

**Convergence**: must converge in ≤ 3 iterations.

### TC-P31-2 — Out-of-plane initial condition

**Setup**: Same as TC-P31-1 but `v_c[2] = 10.0` m/s (10 m/s cross-track).

**Expected**: The CSI burn still converges to an in-track ΔV that zeros the CDH
W-axis residual. `dv_csi_lvlh[2]` (W-axis) remains 0.0 (the CSI burn does not
correct out-of-plane). The W-axis correction is left for a dedicated out-of-plane
plane-change maneuver; the coelliptic sequence handles only in-plane geometry.

**Tolerance**: `|dv_csi_lvlh[2]| < 1e-10` m/s (CSI is defined as a purely in-track burn).

### TC-P31-3 — Non-convergence (degenerate timing)

**Setup**: `dt_csi_to_cdh = 0.0 s`.

**Expected**: `compute_csi_delta_v` returns `Err(InvalidTimeInterval)`.

### TC-P31-4 — Target state zero

**Setup**: `r_t_cdh = [0.0, 0.0, 0.0]`.

**Expected**: returns `Err(DegenerateState)`.

### TC-P31-5 — Alarm 01431 integration test (non-convergence through `p31_init`)

**Setup**: Feed an `AgcState` with `target_pos = [0,0,0]`.

**Expected**: `p31_init` does not modify `state.pending_maneuver`; DSKY program
alarm 01430 is set.

---

### TC-P32-1 — Circular coplanar coelliptic (pure CDH)

**Setup**:
```
r_c = [6_571_000.0, 0.0, 0.0]       m  (chaser at CDH point, 200 km altitude)
v_c = [0.0, 7_784.0, 0.0]           m/s  (current circular orbit — not yet coelliptic)
r_t = [6_589_520.0, 0.0, 0.0]       m  (target, 10 nmi above)
v_t = [0.0, 7_773.0, 0.0]           m/s  (target circular speed)
delta_h = 18_520.0 m
mu = MU_EARTH
```

**Expected CDH ΔV** (S-axis only for circular coplanar case):

The coelliptic condition requires:
```
h_c_required = h_t * r_c / r_t
h_t = 6_589_520 * 7_773 ≈ 5.122e10 m²/s
h_c_required = 5.122e10 * 6_571_000 / 6_589_520 ≈ 5.108e10 m²/s
v_c_s_required = h_c_required / r_c_mag = 5.108e10 / 6_571_000 ≈ 7_773.2 m/s

Note: chaser circular speed at 200 km = 7,784 m/s; the CDH burn must slightly
REDUCE in-track speed.
dv_s = 7_773.2 - 7_784.0 ≈ -10.8 m/s   (retrograde small burn)
```

**Tolerance**: `|dv_s - (-10.8)| < 0.5 m/s`.
`|dv_r| < 0.01 m/s` (no radial burn for circular case).
`|dv_w| < 1e-10 m/s` (no out-of-plane burn).

### TC-P32-2 — Degenerate geometry (chaser at same altitude as target)

**Setup**: `r_c = r_t = [6_571_000.0, 0.0, 0.0]` (same radius at CDH epoch).

**Expected**: `compute_cdh_delta_v` returns `Err(DegenerateGeometry)`.

### TC-P32-3 — Hyperbolic target orbit guard

**Setup**:
```
r_t = [6_571_000.0, 0.0, 0.0]
v_t = [0.0, 12_000.0, 0.0]      m/s  (escape speed at 200 km ≈ 11,009 m/s; this is hyperbolic)
```

**Expected**: `compute_cdh_delta_v` returns `Err(DegenerateState)`.

### TC-P32-4 — P31 + P32 round-trip integration

**Setup**: Use the result of TC-P31-1 as the input to TC-P32-1. Run `compute_csi_delta_v`
to get the post-CSI orbit, propagate to CDH TIG, then run `compute_cdh_delta_v`.

**Expected**:
- Post-CSI propagated state lands within 500 m of the expected transfer orbit
  apoapsis point.
- CDH ΔV magnitude is ≤ 15 m/s (small correction relative to the 10.2 m/s CSI burn).
- Combined `|dv_csi| + |dv_cdh|` is within 10% of the two-impulse Hohmann total
  for the same altitude change.

### TC-P32-5 — Zero ΔV case (chaser already coelliptic)

**Setup**: Construct `r_c`, `v_c` such that `h_c / h_t = r_c_mag / r_t_mag` exactly
(chaser already on coelliptic orbit).

**Expected**: `dv_cdh_lvlh ≈ [0, 0, 0]`. No alarm. `compute_cdh_delta_v` returns `Ok`.

---

## 10. Open Questions for Architect Review

### OQ-P31-1 — Iterative vs. closed-form determination (RESOLVED for spec: iterative)

The Comanche055 `P31,P32.agc` source contains subroutine `S31.1` which performs a
Newton iteration (confirmed from the source listing structure and O'Brien p. 339).
This spec mandates the iterative approach. The architect should confirm whether the
finite-difference derivative step (§5.1 step 2e) is acceptable or whether an
analytic derivative should be derived (which would require partial derivatives of
`kepler_step` w.r.t. velocity — non-trivial; finite difference is simpler and is
what the AGC effectively did).

### OQ-P31-2 — `MAX_ITER` constant placement

`CSI_MAX_ITER = 10` and `CSI_CONVERGE_TOL = 0.01 m/s` are defined in `p31.rs`. If
any other program reuses the CSI convergence loop (unlikely for Phase 5), these
should be promoted to a shared constants module. For now, they live in `p31.rs`.

### OQ-P31-3 — Finite-difference epsilon strategy

The Newton step uses a relative epsilon `eps = max(0.01 * |dv_s| + 0.001, 0.01)`.
For very small `dv_s` (near-zero burn) this evaluates to `0.01 m/s`. For large
burns (> 100 m/s) it uses 1% of the burn magnitude. The architect should validate
that this strategy does not cause oscillation near convergence for small burns.

### OQ-P31-4 — `delta_h` storage in `AgcState`

P32 needs to use the same `delta_h` that P31 was computed with. Currently the
spec has P32 re-use `DELTA_H_DEFAULT_M` unless the crew entered a custom value.
The architect must decide whether to add a `state.pending_delta_h: f64` field to
`AgcState`, or to pass `delta_h` explicitly from the P31→P32 session context.
Adding a field is the simpler approach and matches the AGC erasable `DELTAH`.

### OQ-P31-5 — `TargetingMode` enum modification

Adding `CsiBurn` and `CdhBurn` to `TargetingMode` in `guidance::targeting.rs`
requires a `match` arm update in `programs::p40_p41`. The architect should assess
the blast radius of this change and decide whether to use a catch-all arm or
explicit arms for all variants.

### OQ-P32-1 — CDH W-axis component for out-of-plane cases

The closed-form CDH formula in §5.2 assumes the CDH burn is in-plane (W-axis = 0).
For non-coplanar orbits (target and chaser in slightly different planes after CSI),
a small W-axis component is needed to achieve coplanarity. The Comanche055 CDH
subroutine includes a plane-change component derived from the cross product of the
two orbit normals. The architect should decide whether to include this refinement
in Phase 5 or defer it. **Recommendation**: include it, because the P31 Newton
iteration targets zero CDH W-axis residual under the in-plane CDH assumption; if
CDH also corrects for residual out-of-plane, the P31 convergence criterion must be
revised to target the total CDH ΔV magnitude rather than just the W-axis component.
This is a design decision left for architect review.

### OQ-P32-2 — `refsmmat` field on `AgcState`

Both `p31_init` and `p32_init` call `guidance::targeting::burn_attitude(dv_inertial,
state.refsmmat)`. The `refsmmat` field must exist on `AgcState` (it was introduced
in the P51/P52 spec). If it has not yet been added, P31/P32 must use the identity
matrix as a fallback. The architect should confirm `state.refsmmat` is available
by Phase 5.
