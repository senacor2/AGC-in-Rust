# Specification: `programs/p23` — Cislunar Midcourse Navigation

**Status**: Ready for implementation (Milestone 5 Phase 4)
**Module path**: `agc-core/src/programs/p23.rs`
**Prior art**:
- `specs/p20-spec.md` — scalar Kalman algorithm (§6) reused verbatim; alarm catalogue
- `specs/p21_p22-spec.md` — periodic background nav pattern; `CsmNavState` field conventions
- `specs/rendezvous-spec.md` — frame conventions (P23 operates in ECI or MCI)
- `specs/state-vector-spec.md` — `Frame`, `StateVector`, `Met`
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, Comanche055 directory):
- `Comanche055/P20-P25.agc` — P23 entry sequence, mark incorporation entry points
- `Comanche055/MEASUREMENT_INCORPORATION.agc` — scalar Kalman update (shared with P20/P22)
- `Comanche055/STAR_TABLES.agc` — navigational star catalogue (37 stars, J2000 frame)
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `MARKCOUNT`, `REJECTCNT`, `LASTMARK`,
  `WM` (W-matrix), `RCSM`/`VCSM` (CSM nav state save area), body-selection flag
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
  Operation*, Springer-Praxis 2010.
- Chapter 11, "Rendezvous and Navigation Programs" pp. 303–340
  - pp. 328–333: P23 cislunar navigation overview; star-horizon measurement geometry;
    body-selection logic (Earth vs Moon); mark workflow
  - pp. 318–325: Scalar Kalman incorporation (shared algorithm; see `specs/p20-spec.md` §6)
  - pp. 325–327: W-matrix rectification; process-noise growth model

---

## 1. Purpose and Program Role

### 1.1 Mission context

P23 is the **Cislunar Midcourse Navigation** program. It runs during the long unthrusted
coast phases of the lunar mission:

- **Translunar coast**: from Earth parking orbit translunar injection (TLI) until the
  lunar sphere-of-influence (SOI) crossing at approximately 66,100 km from the Moon.
  Duration: approximately 60–70 hours. The CSM state vector is in ECI frame; the primary
  body observed is the **Earth** (limb visible from cislunar space).
- **Transearth coast**: from transearth injection (TEI) until re-entry corridor. Duration:
  approximately 60–70 hours. The CSM state vector begins in MCI frame (just after TEI)
  and transitions to ECI frame after the SOI crossing; the primary body observed switches
  from the **Moon** to the **Earth** as the trajectory progresses. P23 detects the frame
  on each mark and selects the primary body accordingly.

P23 is selected by the crew keying `V37E 23E` on the DSKY (major mode 23). It runs as a
periodic background program for the entire coast duration, self-rescheduling via the
Waitlist every 10 seconds (1000 centiseconds) to grow process noise and refresh the DSKY
display. Individual measurements arrive asynchronously when the crew completes an optical
mark with the sextant or scanning telescope.

P23 is **not** active during:
- Earth orbit (before TLI) — P20 (rendezvous), P22 (landmark tracking), or P11 run instead.
- Lunar orbit — P20 or P22 apply.
- Powered burns — P40/P41 (SPS) or P41 (RCS) are active.

### 1.2 Relationship to the ground-based state vector uplink

Before a cislunar coast, Mission Control uplinks a fresh CSM state vector computed from
ground-based tracking (Manned Space Flight Network stations). The uplink uncertainty is
typically 5–15 km in position (1-sigma) and 0.5–1 m/s in velocity. The W-matrix is
re-initialised with initial variances reflecting this uplink quality (see §4 constants).

Over the coast, IMU integration and unmodelled perturbations (solar pressure, non-uniform
lunar gravity, residual outgassing) degrade the onboard state. P23 star-horizon sightings
reduce the uncertainty. After approximately 20 accepted marks spread across the coast, the
1-sigma position uncertainty converges to 1–2 km, adequate for midcourse correction
targeting by P30/P31.

The ground uplink may arrive during a running P23 session. The uplink handler calls
`p23_rectify_w_matrix` to reflect the new uplink uncertainty, and the session continues.

### 1.3 AGC crew workflow per mark

1. Crew selects `V37E 23E` at the start of the coast; P23 initialises and begins cycling.
2. Crew uses the sextant to sight a known navigational star against the Earth or Moon limb.
   The sextant computes the angle between the star line-of-sight and the horizon tangent.
3. Crew marks with `V54E` (optics mark verb), which triggers the R52/R53 optics-mark
   routine. R52/R53 decodes the sextant shaft and trunnion angles, identifies the star from
   the star catalogue, and constructs a `StarHorizonMark` struct. It then calls
   `p23_incorporate_star_horizon_mark` in P23.
4. P23 computes the predicted angle, the residual, and the 3-sigma gate. If the mark passes,
   the state and W-matrix are updated. The DSKY is updated with the new mark counter.
5. The crew monitors DSKY V16 N49 (mark count / reject count). If the reject count rises,
   the crew may command a W-matrix rectification with `V32E`.
6. A typical session involves one to two marks per hour; total 20–30 marks over a 60-hour
   coast. Landmark marks (`StarLandmarkMark`) may supplement horizon marks if a known
   crater is in the sextant field of view.

---

## 2. Module Path

`agc-core/src/programs/p23.rs`

The existing `p23.rs` stub (which contains only a `todo!()` stub for `init`) is replaced
in its entirety. The entry point is registered as `PROGRAM_TABLE[23]` in
`programs/mod.rs`.

---

## 3. State Additions

### 3.1 Design decision: share `csm_nav` with P22

**Decision**: P23 shares `state.csm_nav` (`CsmNavState`) with P22. A new `P23NavState`
struct is not added to `AgcState`.

**Rationale**: Both P22 and P23 update the same physical quantity — the CSM's inertial
state vector (`state.csm_state`) and its associated 6×6 uncertainty matrix. The state
target is identical; only the measurement geometry (landmark LOS vs. star-horizon angle)
differs. Sharing the W-matrix is physically correct: P22 sightings in LEO reduce position
uncertainty in the same covariance that P23 uses during cislunar coast. If P23 used a
separate covariance the two programs would silently ignore each other's information,
degrading navigation accuracy when both are used in the same mission segment (e.g. a
landmark sighting just before TLI followed immediately by P23 activation after TLI).

The bookkeeping fields (`mark_count`, `reject_count`, `consecutive_reject_count`,
`last_mark_time`, `tracking_active`) in `CsmNavState` are also reused. `p23_init` resets
them to zero, exactly as `p22_init` does, because P23 is a new navigation session with its
own accept/reject history.

**Body selection**: The primary body being observed (Earth or Moon) is determined at
mark-incorporation time from `state.csm_state.frame` via the helper `primary_body(frame)`
(§6.4). No extra state field is needed to track the current body; the frame is the
authoritative source.

**Star catalogue**: For this phase, P23 uses a compile-time constant array
`CISLUNAR_STAR_TABLE` of 8 star entries (see §3.2). The full 37-star AGC catalogue from
`STAR_TABLES.agc` is registered in `navigation::star_catalog` and can be adopted in a
later phase; the 8-star table covers the geometrically useful subset for cislunar
navigation (high ecliptic latitude stars that provide good horizon-angle sensitivity at
both Earth and Moon distances).

No new fields are added to `AgcState`. The `state.csm_nav` field added in Phase 3 is
sufficient.

### 3.2 Cislunar star table

```rust
/// A navigational star entry for P23 cislunar sightings.
///
/// Direction vectors are unit vectors in the Earth mean equatorial J2000 frame
/// (identical to the ECI frame used by the AGC navigation software).
/// The same vectors apply in the MCI frame because MCI axes are parallel to ECI.
///
/// Source: Comanche055/STAR_TABLES.agc entries for stars with |declination| > 30°
/// (high-latitude stars provide the best geometric diversity for cislunar nav).
/// Values here are approximate; the developer should populate from STAR_TABLES.agc.
#[derive(Clone, Copy, Debug)]
pub struct CislunarStar {
    /// AGC star catalogue number (1-based, 1–37).
    pub number: u8,
    /// Common name for documentation purposes.
    pub name: &'static str,
    /// Unit vector toward the star in J2000 equatorial frame.
    pub direction: Vec3,
}

/// Compile-time star table for P23.  8 entries covering geometrically diverse
/// directions for cislunar navigation.
///
/// Indices 0–7 correspond to stars that were commonly used on Apollo missions for
/// cislunar midcourse navigation (O'Brien p. 329).
/// Developer note: populate `direction` values from STAR_TABLES.agc columns 1–3.
pub const CISLUNAR_STAR_TABLE: [CislunarStar; 8] = [
    CislunarStar { number:  1, name: "Alpheratz",  direction: [0.0; 3] },  // TODO
    CislunarStar { number:  4, name: "Achernar",   direction: [0.0; 3] },  // TODO
    CislunarStar { number:  7, name: "Hamal",      direction: [0.0; 3] },  // TODO
    CislunarStar { number: 10, name: "Menkar",     direction: [0.0; 3] },  // TODO
    CislunarStar { number: 16, name: "Pollux",     direction: [0.0; 3] },  // TODO
    CislunarStar { number: 25, name: "Antares",    direction: [0.0; 3] },  // TODO
    CislunarStar { number: 30, name: "Vega",       direction: [0.0; 3] },  // TODO
    CislunarStar { number: 36, name: "Peacock",    direction: [0.0; 3] },  // TODO
];
```

The `direction` field placeholder `[0.0; 3]` must be replaced during implementation with
the actual unit vectors from `STAR_TABLES.agc`. The stub will compile but no valid
predictions will be computed until populated. Unit tests (§11) use hard-coded star
directions independent of this table.

---

## 4. Public API

### 4.1 `p23_init`

```rust
/// Entry point for P23 (Cislunar Midcourse Navigation).
/// Registered in PROGRAM_TABLE[23].
///
/// Sets `state.major_mode = 23`.  Re-initialises `state.csm_nav` bookkeeping
/// (mark/reject counters, tracking flag, W-matrix if not already valid).
/// Installs the Waitlist self-rescheduling hook for the 10-second update cycle.
///
/// # Preconditions
/// - `state.csm_state.epoch` must be non-zero; otherwise alarm 01420 is raised
///   and the program returns without installing the Waitlist hook.
/// - `state.csm_state.frame` must be `EarthInertial` or `MoonInertial`; otherwise
///   alarm 00400 is raised and the program returns.
///
/// # W-matrix initialisation policy
/// If `state.csm_nav.w_matrix` is all zeros (zero-initialised, as after FRESH START
/// or first mission activation), `p23_init` sets it to the default diagonal
/// `P23_W_INIT_POS_VARIANCE` / `P23_W_INIT_VEL_VARIANCE`.
/// If the W-matrix already contains non-zero entries (left by a prior P22 session or
/// by a ground uplink that set initial uncertainties), it is **not** re-initialised —
/// the prior information is preserved and P23 begins with it.
/// The crew can force a re-initialisation with `V32E` at any time.
///
/// # Post-conditions
/// - `state.major_mode == 23`
/// - `state.dsky.prog == 23`
/// - `state.csm_nav.tracking_active == true`
/// - `state.csm_nav.mark_count == 0`, `reject_count == 0`,
///   `consecutive_reject_count == 0`
/// - `state.csm_nav.last_mark_time == state.time.to_seconds()`
/// - Waitlist entry scheduled for `P23_CYCLE_CS` centiseconds.
///
/// # Returns
/// `P23_PRIORITY`.
pub fn p23_init(state: &mut AgcState) -> JobPriority
```

**Priority**: `P23_PRIORITY: JobPriority = 8` — same as P20 and P22 (all are background
navigation loops at the same scheduling tier).

### 4.2 `p23_cycle_task`

```rust
/// Periodic P23 cislunar navigation update task.  Scheduled via Waitlist::schedule.
///
/// Called every `P23_CYCLE_CS` centiseconds (10 s) after `p23_init`.
///
/// Steps per cycle:
/// 1. Verify `state.csm_state.frame` is ECI or MCI; raise alarm 00400 and suspend
///    tracking if not (e.g. unexpected StableMember frame — should not occur in normal
///    operation but is a safety check).
/// 2. Compute Δt = state.time.to_seconds() - state.csm_nav.last_mark_time.
///    If Δt > P23_MAX_PROCESS_NOISE_DT_S, call `p23_rectify_w_matrix` and skip growth.
///    Otherwise grow W diagonal: W[i][i] += P23_Q_POS * Δt for i in 0..3,
///                                          W[i][i] += P23_Q_VEL * Δt for i in 3..6.
/// 3. Update DSKY display: V16 N49 showing mark_count (R1) and reject_count (R2).
/// 4. Re-schedule: Waitlist::schedule(state, P23_CYCLE_CS, p23_cycle_task).
///
/// # Invariants
/// - Does not modify `state.csm_state` (propagation is the SERVICER's responsibility).
/// - Does not incorporate marks (marks arrive via the star mark handlers).
/// - Runs even when `tracking_active == false` (display continues; only marks are
///   silently discarded by the mark handlers).
pub fn p23_cycle_task(state: &mut AgcState)
```

### 4.3 `p23_incorporate_star_horizon_mark`

```rust
/// Incorporate one star-horizon angle measurement into the CSM navigation solution.
///
/// Called from the sextant HAL handler (R52/R53) when the crew completes a mark
/// on the Earth or Moon horizon against a reference star.
///
/// Performs the scalar Kalman update described in §6.1–§6.3.
///
/// # Arguments
/// - `mark`: decoded star-horizon observation (see §5).
///
/// # Preconditions
/// - `state.csm_nav.tracking_active` must be true; if false the mark is silently
///   discarded (consistent with P20/P22 behaviour).
/// - `mark.star_direction` must be a unit vector (|s_hat| == 1 ± 1e-6); if not,
///   alarm 01426 is raised and the mark is discarded.
/// - `mark.angle_observed_rad` must lie in [0, π]; if not, alarm 01427 is raised
///   and the mark is discarded.
/// - `norm(csm_pos - body_centre) >= R_body + R_MIN_HORIZON_M`; if not,
///   alarm 01430 is raised and the mark is discarded.
///
/// # Post-conditions (mark accepted)
/// - `state.csm_state.position` and `state.csm_state.velocity` updated by Kalman gain.
/// - `state.csm_nav.w_matrix` rank-1 downgraded.
/// - `state.csm_nav.mark_count` incremented.
/// - `state.csm_nav.last_mark_time` set to `mark.time`.
/// - `state.csm_nav.consecutive_reject_count` reset to 0.
///
/// # Post-conditions (mark rejected — residual > 3-sigma)
/// - `state.csm_nav.reject_count` incremented.
/// - `state.csm_nav.consecutive_reject_count` incremented.
/// - State and W-matrix unchanged.
/// - If `consecutive_reject_count == 5`, alarm 01431 raised and `tracking_active`
///   set false.
///
/// # Post-conditions (W-matrix overflow)
/// - Alarm 01421 raised (same code as P22; the W-matrix is shared).
/// - `p23_rectify_w_matrix` called automatically.
///
/// # Mark time ordering
/// If `mark.time < state.csm_nav.last_mark_time`, the mark is processed normally
/// (the Kalman gain still reduces uncertainty); the W-matrix growth step (which
/// uses Δt) is skipped at the next cycle but the mark itself is not discarded.
pub fn p23_incorporate_star_horizon_mark(state: &mut AgcState, mark: StarHorizonMark)
```

### 4.4 `p23_incorporate_star_landmark_mark`

```rust
/// Incorporate one star-landmark angle measurement into the CSM navigation solution.
///
/// Called from the sextant HAL handler when the crew sights a known surface feature
/// (crater, cape, mountain) on the Earth or Moon and computes the angle between the
/// landmark line-of-sight and a reference star.
///
/// Performs the scalar Kalman update described in §6.2.
///
/// # Arguments
/// - `mark`: decoded star-landmark observation (see §5).
///
/// # Preconditions and post-conditions
/// Same as `p23_incorporate_star_horizon_mark` except that the measurement model
/// is the star-landmark angle (§6.2) and the minimum distance guard uses the
/// CSM-to-landmark distance rather than the CSM-to-body-centre distance.
///
/// # Additional guard
/// `norm(csm_pos - landmark_inertial) >= P23_MIN_LANDMARK_RANGE_M`; if not,
/// alarm 01432 is raised and the mark is discarded.
pub fn p23_incorporate_star_landmark_mark(state: &mut AgcState, mark: StarLandmarkMark)
```

### 4.5 `p23_rectify_w_matrix`

```rust
/// Re-initialise the W-matrix to the default diagonal (large uncertainty).
///
/// Called:
/// - On crew command `V32E`.
/// - Automatically when alarm 01421 fires (W-matrix lost positive definiteness).
/// - When the process-noise Δt exceeds P23_MAX_PROCESS_NOISE_DT_S.
/// - When the ground uplinks a fresh state vector (uplink handler calls this).
///
/// Sets W diagonal to `P23_W_INIT_POS_VARIANCE` (rows 0–2) and
/// `P23_W_INIT_VEL_VARIANCE` (rows 3–5).  Zeros all off-diagonal elements.
/// Resets `mark_count = 0`, `reject_count = 0`, `consecutive_reject_count = 0`.
/// Sets `last_mark_time = state.time.to_seconds()`.
/// Sets `tracking_active = true`.
///
/// Uses the shared `state.csm_nav` (no separate struct).
pub fn p23_rectify_w_matrix(state: &mut AgcState)
```

### 4.6 Constants

```rust
/// Major mode number for P23.
pub const P23_MAJOR_MODE: u8 = 23;

/// Job priority for P23.  Same as P20 and P22 (background navigation tier).
pub const P23_PRIORITY: JobPriority = 8;

/// Waitlist cycle period for P23 (centiseconds).
/// 10 seconds = 1000 cs.  Cislunar dynamics are orders of magnitude slower than
/// the orbital dynamics relevant to P20/P22 (2 s cycles), so a longer period
/// reduces Executive scheduling overhead without degrading navigation accuracy.
/// The process-noise growth step updates the W-matrix continuously regardless of
/// the cycle period; a 10-second period is appropriate for a coast of 60–70 hours.
pub const P23_CYCLE_CS: u32 = 1_000;

/// Initial position variance on the P23 W-matrix diagonal (m²).
/// Corresponds to ±10 km (1-sigma) positional uncertainty at P23 start.
/// This is larger than P22's 500 m initial variance because the ground uplink
/// for a cislunar state vector has ~10 km uncertainty (MSFN tracking at lunar
/// distances), compared to ~500 m for LEO radar tracking.
/// Rationale: σ_pos ≈ 10 000 m → variance = (10 000)² = 1e8 m².
pub const P23_W_INIT_POS_VARIANCE: f64 = 1.0e8;   // (10 km)²

/// Initial velocity variance on the P23 W-matrix diagonal (m²/s²).
/// Corresponds to ±1 m/s (1-sigma).  Cislunar velocity uncertainty is similar
/// to LEO after a ground uplink (both are Doppler-limited).
pub const P23_W_INIT_VEL_VARIANCE: f64 = 1.0;      // (1 m/s)²

/// Process-noise growth rate for CSM position (m²/s).
/// Value: 5.0 m²/s.  Rationale: the translunar coast accumulates perturbation
/// uncertainty primarily from solar radiation pressure (~1–3 × 10⁻⁶ m/s²) and
/// lunar mascon variations.  Over 10 hours without a mark the 1-sigma position
/// uncertainty should grow by no more than ~500 m, corresponding to
/// Q_POS × 36000 ≈ 180 000 m², σ ≈ 424 m.  A value of 5.0 m²/s gives
/// Δσ_pos ≈ sqrt(5 × 36000) ≈ 424 m over 10 h, which matches operational
/// experience from Apollo 8–10 midcourse navigation performance.
/// This is 10× larger than P22's CSM_Q_POS = 0.5 m²/s because cislunar coast
/// accumulates perturbation errors faster than low Earth orbit (no drag, but
/// unmodelled lunar-gravity gradient effects are larger at lunar distances).
pub const P23_Q_POS: f64 = 5.0;    // m²/s

/// Process-noise growth rate for CSM velocity (m²/s³).
/// Value: 1.0e-5 m²/s³.  Rationale: velocity process noise over 10 h should
/// be comparable to an unmodelled solar-pressure Δv of ~0.01 m/s,
/// giving Q_VEL × 36000 ≈ 0.36 m²/s², σ_vel ≈ 0.006 m/s.  With Q_VEL = 1e-5:
/// sqrt(1e-5 × 36000) ≈ 0.6 m/s — slightly conservative but safe.
/// 10× larger than P22's CSM_Q_VEL = 1e-6.
pub const P23_Q_VEL: f64 = 1.0e-5; // m²/s³

/// Sextant star-horizon angle noise variance (rad²).
/// Apollo CM sextant angular resolution: ~10 arcsec RMS ≈ 4.85e-5 rad.
/// Variance ≈ (4.85e-5)² ≈ 2.35e-9 rad².  Use 2.5e-9 (slightly conservative).
pub const SIGMA_STAR_HORIZON_SQ: f64 = 2.5e-9;    // (≈10 arcsec)²

/// Sextant star-landmark angle noise variance (rad²).
/// Same sextant hardware; same angular noise floor as star-horizon marks.
/// Landmark identification errors may add a small additional term but are
/// absorbed into the 3-sigma gate for the first phase.
pub const SIGMA_STAR_LANDMARK_SQ: f64 = 2.5e-9;   // (≈10 arcsec)²

/// Earth equatorial radius (m).  WGS84 semi-major axis.
pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Moon mean radius (m).  IAU 2012 value.
pub const MOON_RADIUS_M: f64 = 1_737_400.0;

/// Minimum distance from the body surface for a horizon measurement (m).
/// Below this height the horizon-angle formula becomes degenerate (asin
/// approaches π/2 as d → R_body).  100 km above surface is a conservative guard;
/// P23 is never active below ~50 000 km from either body during cislunar coast.
pub const R_MIN_HORIZON_M: f64 = 100_000.0;       // 100 km above surface

/// Minimum CSM-to-landmark slant range for a landmark mark (m).
/// Same safety floor as P22's MIN_LANDMARK_RANGE_M; in practice always > 1000 km
/// for cislunar landmark sightings.
pub const P23_MIN_LANDMARK_RANGE_M: f64 = 1_000.0;

/// Maximum Δt for process-noise growth before forced W re-initialisation (s).
/// 24 hours.  Longer than P20/P22's 1-hour cap because cislunar coast passes
/// can last 24–30 h between crew sleep and activity cycles.  After 24 h without
/// a mark the W-matrix is already very large and re-initialisation is appropriate.
pub const P23_MAX_PROCESS_NOISE_DT_S: f64 = 86_400.0; // 24 h
```

---

## 5. Measurement Types

```rust
/// A star-horizon angle measurement from the CM sextant.
///
/// The crew aligns the sextant's movable index mark with the bright limb of the
/// Earth or Moon while simultaneously placing the fixed reticle on a reference star.
/// The sextant reads the half-angle between the star line-of-sight and the nearest
/// point on the body's visible limb (the "horizon").  R52/R53 decodes the sextant
/// CDU angles and delivers this struct to `p23_incorporate_star_horizon_mark`.
///
/// AGC correspondence: the angle is stored in erasable `STARANGLE` (octal XXXX)
/// at scale B+1 (fractions of π rad) in Comanche055.
#[derive(Clone, Copy, Debug)]
pub struct StarHorizonMark {
    /// Mission Elapsed Time of the sighting (s).
    /// Corresponds to the time the crew pressed MARK (V54E).
    pub time: f64,

    /// Unit vector toward the reference star in the inertial frame (ECI or MCI).
    /// Populated by R52/R53 by looking up the star direction in `CISLUNAR_STAR_TABLE`
    /// (or `navigation::star_catalog`) using the star number entered by the crew.
    /// Magnitude must be 1.0 ± 1e-6.
    pub star_direction: Vec3,

    /// Which body's limb was used as the horizon reference.
    /// Determined from `state.csm_state.frame` at the time of the mark by R52/R53,
    /// or entered explicitly by the crew via V06 N89 body-select verb.
    pub body: Body,

    /// The measured star-horizon angle in radians.
    ///
    /// This is the angle from the body's limb (horizon tangent) to the star,
    /// as read from the sextant.  When the star is exactly on the horizon the
    /// angle is 0.  When the star is 90° from the horizon (i.e. the body is
    /// directly behind the CSM) the angle is π/2.  The full geometric range
    /// possible during cislunar flight is approximately [0, π/2].
    ///
    /// Valid range: [0, π].  Values outside this range trigger alarm 01427.
    pub angle_observed_rad: f64,
}

/// A star-landmark angle measurement from the CM sextant.
///
/// The crew sights a known surface feature (crater, cape, mountain) on the Earth
/// or Moon and a reference star simultaneously.  The sextant outputs the angle
/// between the two lines-of-sight.  This measurement constrains both the direction
/// to the body and the distance to it (via parallax of the landmark w.r.t. the
/// body centre).
///
/// Less frequently used than horizon marks; provides stronger geometric diversity
/// when available.
#[derive(Clone, Copy, Debug)]
pub struct StarLandmarkMark {
    /// Mission Elapsed Time of the sighting (s).
    pub time: f64,

    /// Unit vector toward the reference star in the inertial frame.
    /// Same provenance as `StarHorizonMark::star_direction`.
    pub star_direction: Vec3,

    /// Which body the landmark is on.
    pub body: Body,

    /// Inertial-frame position of the landmark (m).
    ///
    /// For Earth landmarks: computed from Earth-fixed geodetic coordinates via
    /// the same rotation used in P22 (`landmark_inertial_pos`, `programs::p22`),
    /// using `state.gha_epoch_rad` and `mark.time`.
    ///
    /// For Moon landmarks: computed from selenographic (Moon-fixed) coordinates
    /// via a Moon-rotation model.  The Moon rotates synchronously; for the
    /// translunar/transearth coast approximation, use the Moon's sub-Earth
    /// direction from `navigation::planetary::moon_position` plus a fixed
    /// selenographic longitude and latitude.  The architect should resolve the
    /// Moon-fixed-to-MCI rotation helper needed here (see §12, open question OQ-1).
    ///
    /// Units: metres in the same inertial frame as `state.csm_state.position`.
    pub landmark_inertial: Vec3,

    /// The measured star-landmark angle (rad).
    /// The angle between the star direction and the CSM-to-landmark line-of-sight.
    /// Valid range: [0, π].
    pub angle_observed_rad: f64,
}

/// Which body's limb or surface is being used as the navigation reference.
///
/// Determined from the CSM frame (`EarthInertial` → Earth, `MoonInertial` → Moon)
/// or from explicit crew selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    Earth,
    Moon,
}
```

---

## 6. Algorithm Specifications

### 6.1 Star-horizon angle measurement model

This section derives the predicted measurement and the sensitivity vector `b` for a
star-horizon angle mark.  All vectors are in the inertial frame (ECI or MCI).

**Inputs**:
- `r_csm`: CSM inertial position (m). Read from `state.csm_state.position`.
- `r_body`: Body centre inertial position (m).
  - Earth: zero vector in ECI (origin at Earth centre).
  - Moon: `navigation::planetary::moon_position(state.csm_state.epoch)` in ECI, or zero
    vector in MCI (origin at Moon centre).
- `R_body`: body radius (m). `EARTH_RADIUS_M` or `MOON_RADIUS_M`.
- `s_hat`: star unit vector (dimensionless, inertial frame). From `mark.star_direction`.
- `angle_obs`: observed angle (rad). From `mark.angle_observed_rad`.

**Step 1 — Relative position from body centre to CSM**:

```
rho = r_csm - r_body          (m, 3-vector)
```

**Step 2 — Distance from body centre to CSM**:

```
d = norm(rho)                  (m, scalar)
```

**Step 3 — Safety guard**:

```
if d < R_body + R_MIN_HORIZON_M:
    raise alarm ALARM_P23_TOO_CLOSE_TO_BODY (01430)
    return (discard mark)
```

**Step 4 — Half-angle subtended by the body (angular radius of the body as seen from CSM)**:

```
phi = asin(R_body / d)         (rad, scalar, range [0, π/2))
```

This is the angle at the CSM between the line to the body centre and the tangent to the
body's limb (horizon tangent line).

**Step 5 — Unit vector from body centre to CSM**:

```
u_hat = rho / d                (dimensionless, 3-vector, unit)
```

**Step 6 — Predicted star-horizon angle**:

```
cos_alpha = dot(s_hat, u_hat)               (dimensionless, scalar)
alpha = acos(cos_alpha)                      (rad; angle between star and body-centre direction)
theta_pred = alpha - phi                     (rad; angle from limb to star)
```

Interpretation: `alpha` is the angular distance from the star to the body centre (as seen
from the CSM). Subtracting `phi` removes the angular radius of the body, leaving the
angle from the body's limb (horizon) to the star.

At large CSM distances (d >> R_body), phi → 0 and `theta_pred → alpha`. At the Earth's
surface (d = R_body, pathological), phi = π/2 and `theta_pred` would be undefined — the
guard in Step 3 prevents this.

**Step 7 — Residual**:

```
residual = angle_obs - theta_pred             (rad, scalar)
```

**Sensitivity vector `b` (6×1)**:

`b` is the Jacobian `d theta_pred / d x` where `x = [r_csm, v_csm]^T` is the 6-element
state.

Velocity components: `theta_pred` depends only on position (through `rho` and `d`), not
on velocity.  Therefore:

```
b[3] = b[4] = b[5] = 0.0
```

Position components: we need `d theta_pred / d r_csm`.

Since `theta_pred = alpha - phi` and `phi = asin(R_body / d)`:

```
d theta_pred / d r_csm = d alpha / d r_csm  -  d phi / d r_csm
```

**Partial of `alpha = acos(dot(s_hat, u_hat))` with respect to `r_csm`**:

Let `f = dot(s_hat, u_hat) = dot(s_hat, rho/d)`.

`d f / d r_csm` requires applying the chain rule to `rho / d`:

```
d(rho/d) / d r_csm  =  (I - u_hat * u_hat^T) / d
```

where `I` is the 3×3 identity and `u_hat * u_hat^T` is the outer product.  This is the
standard Jacobian of a normalised vector (the "projection onto the sphere" operator).

Therefore:

```
d f / d r_csm  =  s_hat^T * (I - u_hat * u_hat^T) / d
               =  (s_hat - f * u_hat) / d
```

And since `d/df [acos(f)] = -1 / sqrt(1 - f^2)`:

```
d alpha / d r_csm  =  -(s_hat - f * u_hat) / (d * sqrt(1 - f^2))
```

Numerically: `sqrt(1 - f^2) = sin(alpha)`.  We must guard against `sin(alpha) ≈ 0`
(star is aligned with body direction); see §9 edge cases.

**Partial of `phi = asin(R_body / d)` with respect to `r_csm`**:

Let `g = R_body / d`.

```
d g / d r_csm  =  -R_body / d^2 * (d d / d r_csm)
               =  -R_body / d^2 * u_hat^T        (since d(norm(rho))/d rho = u_hat^T)
```

And since `d/dg [asin(g)] = 1 / sqrt(1 - g^2)`:

```
d phi / d r_csm  =  1/sqrt(1 - g^2) * (-R_body / d^2) * u_hat^T
                 =  -R_body / (d^2 * sqrt(1 - (R_body/d)^2)) * u_hat^T
                 =  -R_body / (d * sqrt(d^2 - R_body^2)) * u_hat^T
```

Numerically: `sqrt(d^2 - R_body^2)` is the length of the tangent line from the CSM to
the limb.  This is always positive for `d > R_body`.

**Combining**:

```
d theta_pred / d r_csm  =  d alpha / d r_csm  -  d phi / d r_csm

  =  -(s_hat - f * u_hat) / (d * sin(alpha))
     +  R_body / (d * sqrt(d^2 - R_body^2)) * u_hat^T
```

In component form (3-vector `b[0..3]`):

```
b[0..3]  =  -(s_hat - cos_alpha * u_hat) / (d * sin_alpha)
            +  (R_body / (d * tangent_len)) * u_hat
```

where:
```
cos_alpha   = dot(s_hat, u_hat)           = f
sin_alpha   = sqrt(1 - cos_alpha^2)       (guard: see §9)
tangent_len = sqrt(d^2 - R_body^2)        (length of tangent from CSM to limb; always > 0)
```

This is the complete closed-form expression for `b[0..3]`.  Substituting back:

```
b[0..3]  =   (cos_alpha * u_hat - s_hat) / (d * sin_alpha)
           + (R_body / (d * tangent_len)) * u_hat

         =   u_hat * (cos_alpha / (d * sin_alpha) + R_body / (d * tangent_len))
           - s_hat  / (d * sin_alpha)
```

Which can be written as:

```
A = (cos_alpha / sin_alpha + R_body / tangent_len) / d       (scalar)
B = 1.0 / (d * sin_alpha)                                   (scalar)

b[0..3] = A * u_hat - B * s_hat
```

This is the form that should be implemented in code.  Both terms are O(1/d), so the
sensitivity decreases as the CSM moves farther from the body — consistent with the
physical intuition that a more distant body subtends a smaller angle and the measurement
is less informative per unit of position change.

### 6.2 Star-landmark angle measurement model

For a star-landmark mark the reference direction is not the body's limb but a specific
point on the body's surface.

**Inputs** (in addition to `s_hat` and `r_csm`):
- `lm_inertial`: inertial position of the landmark (m).  Pre-computed by the caller.

**Step 1 — Vector from CSM to landmark**:

```
v_lm = lm_inertial - r_csm         (m, 3-vector; points from CSM toward landmark)
```

**Step 2 — Distance and direction**:

```
d_lm  = norm(v_lm)                  (m)
l_hat = v_lm / d_lm                 (unit vector from CSM toward landmark)
```

**Step 3 — Safety guard**:

```
if d_lm < P23_MIN_LANDMARK_RANGE_M:
    raise alarm ALARM_P23_LANDMARK_RANGE_ZERO (01432)
    return
```

**Step 4 — Predicted star-landmark angle**:

```
cos_beta = dot(s_hat, l_hat)         (dimensionless)
beta_pred = acos(cos_beta)           (rad; angle between star and CSM-to-landmark direction)
```

Note the sign convention: `l_hat` points from the CSM toward the landmark (opposite to
the P22 convention where `rho_vec` points from landmark toward the CSM). The dot product
`dot(s_hat, l_hat)` gives the angle between the star and the landmark as seen from the CSM,
which is the physically observed quantity.

**Step 5 — Residual**:

```
residual = angle_obs - beta_pred      (rad)
```

**Sensitivity vector `b[0..3]`**:

By the same derivation as §6.1 (replacing `u_hat` with `-l_hat` and noting that the
sign of `v_lm` with respect to `r_csm` is opposite to `rho`):

```
d beta_pred / d r_csm  =  (s_hat - cos_beta * l_hat) / (d_lm * sin_beta)
```

Note: the sign is opposite to the `alpha` partial in §6.1 because increasing `r_csm`
moves the CSM away from the landmark, which changes the angle in the opposite sense to
moving the CSM away from the body centre.  Derivation:

`l_hat = (lm_inertial - r_csm) / d_lm`

`d l_hat / d r_csm = -(I - l_hat * l_hat^T) / d_lm`

`d cos_beta / d r_csm = s_hat^T * (-(I - l_hat * l_hat^T) / d_lm)`
                     ` = -(s_hat - cos_beta * l_hat) / d_lm`

`d beta_pred / d r_csm = -1/sin_beta * d cos_beta / d r_csm`
                      ` = (s_hat - cos_beta * l_hat) / (d_lm * sin_beta)`

In component form:

```
sin_beta = sqrt(1 - cos_beta^2)    (guard: see §9)

b[0..3] = (s_hat - cos_beta * l_hat) / (d_lm * sin_beta)
b[3..6] = [0.0; 3]
```

### 6.3 Reuse of the scalar Kalman update

P23 calls `navigation::kalman::scalar_measurement_update` exactly as P20 and P22 do.
No modification to the shared helper is required.

The wrapper pattern for `p23_incorporate_star_horizon_mark`:

```rust
// Pack CSM state into 6-vector.
let mut x: [f64; 6] = [
    state.csm_state.position[0], state.csm_state.position[1], state.csm_state.position[2],
    state.csm_state.velocity[0], state.csm_state.velocity[1], state.csm_state.velocity[2],
];

// Compute b and residual per §6.1.
let b:        [f64; 6] = compute_star_horizon_b(/* ... */);
let residual: f64      = mark.angle_observed_rad - theta_pred;

let outcome = navigation::kalman::scalar_measurement_update(
    &mut x,
    &mut state.csm_nav.w_matrix,
    b,
    residual,
    SIGMA_STAR_HORIZON_SQ,
);

// Unpack x back into state.
state.csm_state.position = [x[0], x[1], x[2]];
state.csm_state.velocity = [x[3], x[4], x[5]];

// Handle outcome: increment counters, fire alarms as needed.
```

The wrapper for `p23_incorporate_star_landmark_mark` is identical in structure, using
`b` from §6.2 and `SIGMA_STAR_LANDMARK_SQ`.

### 6.4 Body detection from frame

```rust
/// Map the current CSM frame to the primary body for horizon measurements.
///
/// Called at the top of each mark-incorporation function to determine which
/// body radius to use and where to place the body-centre origin.
///
/// # Panics
/// Panics if `frame == Frame::StableMember` — this frame should never appear
/// on a stored StateVector and indicates a software fault.  A hard panic is
/// appropriate because no safe navigation computation is possible.
pub fn primary_body(frame: Frame) -> Body {
    match frame {
        Frame::EarthInertial => Body::Earth,
        Frame::MoonInertial  => Body::Moon,
        Frame::StableMember  =>
            panic!("p23: primary_body called with StableMember frame — software fault"),
    }
}
```

In ECI frame the body centre for Earth is the origin (zero vector).  In MCI frame the body
centre for the Moon is the origin; the Earth's position in MCI is obtained from
`navigation::planetary::moon_position` (negated, since that function returns Moon position
in ECI, not Earth position in MCI — the architect should confirm the coordinate convention
in `navigation::planetary` before implementing).

For cislunar horizon marks with the primary body, the body centre is always the origin of
the current frame, so `r_body = [0.0; 3]` and `rho = r_csm` in all cases.  Earth sightings
from MCI frame (during transearth coast before SOI crossing) or Moon sightings from ECI
frame (during translunar coast after SOI approach) are handled by the star-landmark model
(the body is no longer the primary gravitational centre and its ephemeris must be used),
but these cases are uncommon and may be deferred to a future phase.

### 6.5 Process noise growth

The same diagonal-growth model as P20 and P22 applies:

```
Δt = state.time.to_seconds() - state.csm_nav.last_mark_time    (s)

if Δt > P23_MAX_PROCESS_NOISE_DT_S:
    p23_rectify_w_matrix(state)
    return

for i in 0..3:
    state.csm_nav.w_matrix[i][i] += P23_Q_POS * Δt

for i in 3..6:
    state.csm_nav.w_matrix[i][i] += P23_Q_VEL * Δt
```

The higher values `P23_Q_POS = 5.0` and `P23_Q_VEL = 1.0e-5` (compared to P22's
`CSM_Q_POS = 0.5`, `CSM_Q_VEL = 1.0e-6`) model the larger unmodelled perturbations
during cislunar coast (solar pressure, mascon variations).  This growth is applied in
`p23_cycle_task` every 10 seconds.  Between marks the diagonal entries grow by at most
`P23_Q_POS × 10 s = 50 m²` per cycle (σ growth ≈ 0.007 m per cycle — negligible), so
process noise does not degrade accuracy between marks; its effect accumulates over hours.

---

## 7. DSKY Interaction

P23 drives the following DSKY displays.

### 7.1 Continuous monitoring display (V16 N49)

Updated every 10-second `p23_cycle_task` cycle.

| Register | Content | Notes |
|----------|---------|-------|
| R1 | `mark_count` | Count of accepted marks since P23 start. |
| R2 | `reject_count` | Count of rejected marks since P23 start. |
| R3 | 0 (unused) | — |

The display is continuous (`V16` = monitor mode).  Noun 49 is the same noun used by P20
and P22 for mark/reject counters (O'Brien p. 329).

### 7.2 Position residual display (V06 N45)

On crew request (`V06E N45E`).

| Register | Content | Unit |
|----------|---------|------|
| R1 | Last measurement residual magnitude (|residual| × 1000) | mrad × 1000 (i.e. microradians as integer) |
| R2 | 0 (unused) | — |
| R3 | 0 (unused) | — |

Note: the AGC N45 noun convention for P23 is not fully specified in O'Brien; the
developer should verify against `P20-P25.agc` P23 section.  The architect may elect to
display the residual directly as a fraction of π in fixed-point, matching the P20 N54
convention.

### 7.3 Body selection display (V06 N89)

Before starting a sighting session, the crew may key `V06E N89E` to display or change the
primary body.  Noun 89 is the body-selection verb (O'Brien p. 329, Table 11.3).

| R1 value | Meaning |
|----------|---------|
| 1 | Earth (body = Body::Earth) |
| 2 | Moon  (body = Body::Moon) |

The body selection is used only for cases where the crew wants to override the
frame-derived body (e.g. the Moon is visible from ECI frame during translunar coast and
the crew wants to sight against it).  In the standard case the mark struct carries `body`
from R52/R53 based on the frame.

### 7.4 Program entry display

On entry (`p23_init`):
1. P23 displays `V06 N49` with the initial mark and reject counts (both zero).
2. Switches to `V16 N49` continuous monitor.

### 7.5 Crew workflow sequence

| Step | Verb/Noun | Who | Meaning |
|------|-----------|-----|---------|
| 1 | `V37E 23E` | Crew | Select P23 (major mode 23). |
| 2 | `V16 N49` | P23 | Begin continuous mark/reject display. |
| 3 | (sextant alignment) | Crew | Align star and horizon in sextant eyepiece. |
| 4 | `V54E` | Crew | Enter optics mark (R52/R53 takes over). |
| 5 | — | R52/R53 | Decode sextant CDU angles; construct `StarHorizonMark`; call `p23_incorporate_star_horizon_mark`. |
| 6 | `V16 N49` | P23 | Display updated mark counter. |
| 7 | `V06 N45E` | Crew | (optional) Display last residual. |
| 8 | `V32E` | Crew | (optional) Re-initialise W-matrix. |
| 9 | `V34E` | Crew | Terminate P23; return to P00. |

---

## 8. Program Alarms

The octal range 01426–01437 is unused by all other programs in this codebase.  P23 uses
codes from this range.

| Code (octal) | Decimal | Mnemonic | Trigger | Recovery |
|---|---|---|---|---|
| 01420 | 784 | NO_CSM_SV | P23 entered with `csm_state.epoch == 0.0`. | Display alarm; return without installing Waitlist hook. Shared with P21/P22. |
| 00400 | 256 | FRAME_MISMATCH | `csm_state.frame == StableMember` on P23 entry or during cycle task. | Display alarm; set `tracking_active = false`. Shared with P20/P22. |
| 01421 | 785 | W_OVERFLOW | W-matrix diagonal entry went negative after a mark update. | Auto-call `p23_rectify_w_matrix`; display alarm; continue. Shared code, same as P22. |
| 01426 | 790 | ALARM_NO_STAR_LOCK | `mark.star_direction` magnitude < 0.999 (zero or invalid unit vector). | Discard mark; no counter update; display alarm. |
| 01427 | 791 | ALARM_BAD_ANGLE | `mark.angle_observed_rad` outside [0, π]. | Discard mark; no counter update; display alarm. |
| 01430 | 792 | ALARM_P23_TOO_CLOSE_TO_BODY | CSM inside `R_body + R_MIN_HORIZON_M`. | Discard horizon mark; display alarm. |
| 01431 | 793 | ALARM_P23_REJECT_OVERRIDE | Five consecutive marks rejected by 3-sigma gate. | Set `tracking_active = false`; display alarm. Crew must key `V32E` to re-enable. |
| 01432 | 794 | ALARM_P23_LANDMARK_RANGE_ZERO | `norm(csm_pos - landmark_inertial) < P23_MIN_LANDMARK_RANGE_M`. | Discard landmark mark; display alarm. |

**In code**:

```rust
const ALARM_NO_CSM_SV:               u16 = 0o01420;  // shared with P21/P22
const ALARM_FRAME_MISMATCH:          u16 = 0o00400;  // shared with P20/P22
const ALARM_W_OVERFLOW:              u16 = 0o01421;  // shared with P22
const ALARM_NO_STAR_LOCK:            u16 = 0o01426;
const ALARM_BAD_ANGLE:               u16 = 0o01427;
const ALARM_P23_TOO_CLOSE_TO_BODY:   u16 = 0o01430;
const ALARM_P23_REJECT_OVERRIDE:     u16 = 0o01431;
const ALARM_P23_LANDMARK_RANGE_ZERO: u16 = 0o01432;
```

---

## 9. Edge Cases

| ID | Condition | Affected function | Required behaviour |
|----|-----------|-------------------|--------------------|
| EC-1 | `csm_state.frame == StableMember` | `p23_init`, `p23_cycle_task` | Raise alarm 00400; set `tracking_active = false`; do not install or re-install Waitlist hook. The stable-member frame should never appear on a stored `StateVector` — this indicates a software fault in the SERVICER. |
| EC-2 | `norm(mark.star_direction)` not in [0.999, 1.001] | `p23_incorporate_star_horizon_mark`, `p23_incorporate_star_landmark_mark` | Raise alarm 01426; discard mark; no counter update. Do not normalise silently — a bad magnitude indicates the caller made an error that should be visible. |
| EC-3 | `mark.angle_observed_rad` outside [0, π] | Both mark handlers | Raise alarm 01427; discard. Values outside this range are physically impossible for a star-horizon angle. |
| EC-4 | CSM inside body (`norm(rho) < R_body + R_MIN_HORIZON_M`) | `p23_incorporate_star_horizon_mark` | Raise alarm 01430; discard. This is a physics guard, not a normal operating condition. |
| EC-5 | Star direction co-linear with body direction (`sin_alpha ≈ 0`) | `p23_incorporate_star_horizon_mark` | `sin_alpha = sqrt(1 - cos_alpha^2)`. Guard: if `sin_alpha < 1e-6`, the measurement is near-degenerate (star is nearly in line with the body-CSM axis). Discard the mark with alarm 01427 (bad geometry — re-use the bad-angle alarm code since this is effectively an unobservable configuration). Document that the crew should choose a star well away from the body direction (|alpha| > ~10°). |
| EC-6 | Star direction co-linear with landmark direction (`sin_beta ≈ 0`) | `p23_incorporate_star_landmark_mark` | Same guard as EC-5. If `sin_beta < 1e-6` discard with alarm 01427. |
| EC-7 | SOI crossing during a running P23 session (frame changes from ECI to MCI or vice versa) | `p23_cycle_task`, `p23_incorporate_star_horizon_mark` | The SERVICER updates `state.csm_state.frame` when the SOI boundary is crossed. At the next cycle or mark incorporation, `primary_body(frame)` will return the new body. The W-matrix and counters are preserved — no rectification is needed for a frame change alone. The crew should be aware that the body for subsequent horizon marks changes. `p23_init` does not need to be re-called; the frame is re-read on every mark. |
| EC-8 | `mark.time < state.csm_nav.last_mark_time` (mark arrived out of temporal order) | Both mark handlers | Accept the mark and update state. Skip the W-matrix growth step (do not grow W backwards). Do not raise an alarm; out-of-order delivery is possible in simulation but is physically impossible in flight (marks arrive in real time). Update `last_mark_time` to `mark.time` only if `mark.time > last_mark_time`. |
| EC-9 | `tracking_active == false` when mark arrives | Both mark handlers | Silently discard. No counter update. The crew must key `V32E` to re-enable. |
| EC-10 | `p23_init` called while `csm_nav.w_matrix` is non-zero (P22 ran earlier) | `p23_init` | Preserve the W-matrix (prior information is valid). Reset counters. This is the sharing-rationale case — documented in §3.1. |
| EC-11 | `p23_init` called while another periodic program (P22) is still scheduled on the Waitlist | `p23_init` | P22's cycle task will continue to fire until P22 exits or the crew selects another program. The architect must ensure that P22 and P23 cycle tasks can coexist safely since they share `csm_nav`. However, during a normal cislunar coast, P22 will not be running (no Earth landmarks visible). If both are somehow active, the state will still converge correctly (both update the same W-matrix). This scenario is flagged as OQ-2 in §12. |

---

## 10. Restart Recovery

**Deferred.** Restart protection for P23 state is not implemented in this phase. This is
the same deferral applied to P20 and P22 in their respective specs.

If a restart occurs during a P23 session, `state.csm_nav` will be in an undefined state
(partially updated). On recovery the crew must re-enter P23 (`V37E 23E`), which calls
`p23_init` and re-initialises `csm_nav` from a clean diagonal W-matrix. The CSM state
`state.csm_state` is restart-protected by the SERVICER (it is in erasable memory that
survives restarts per `services/fresh_start.rs` `partial_restart` semantics); only the
covariance and counters are lost.

This limitation is explicitly documented here and is acceptable for Phase 4. Full restart
protection for the cislunar navigation state (analogous to the AGC restart group mechanism
in `P20-P25.agc`) is an open item for a later phase.

---

## 11. Test Cases

The following test cases use hard-coded geometry that is independent of the
`CISLUNAR_STAR_TABLE` stub entries.  All tests define `star_direction` explicitly.

### TC-P23-1: `p23_init` happy path

**Purpose**: Verify that `p23_init` sets major mode, resets counters, initialises W-matrix,
and schedules Waitlist.

**Setup**:
```
state.csm_state = StateVector {
    position: [3.84e8, 0.0, 0.0],   // ~384,000 km from Earth (lunar distance)
    velocity: [0.0, 800.0, 0.0],
    epoch: Met(1_000_000),           // non-zero epoch
    frame: Frame::EarthInertial,
}
state.csm_nav = CsmNavState::default()  // all zeros
```

**Action**: `p23_init(&mut state)`

**Expected**:
- `state.major_mode == 23`
- `state.dsky.prog == 23`
- `state.csm_nav.tracking_active == true`
- `state.csm_nav.mark_count == 0`
- `state.csm_nav.reject_count == 0`
- `state.csm_nav.consecutive_reject_count == 0`
- `state.csm_nav.w_matrix[0][0] == P23_W_INIT_POS_VARIANCE` (= 1.0e8)
- `state.csm_nav.w_matrix[3][3] == P23_W_INIT_VEL_VARIANCE` (= 1.0)
- `state.csm_nav.w_matrix[0][1] == 0.0` (off-diagonal)
- `state.alarm.code == 0` (no alarm)
- Waitlist has a pending entry for `p23_cycle_task` at `P23_CYCLE_CS` cs.

---

### TC-P23-2: `p23_init` with zero CSM epoch raises alarm

**Purpose**: Guard against an uninitialised CSM state vector.

**Setup**:
```
state.csm_state = StateVector::ZERO   // epoch == 0
```

**Action**: `p23_init(&mut state)`

**Expected**:
- `state.alarm.code == 0o01420` (ALARM_NO_CSM_SV)
- `state.alarm.lit == true`
- `state.major_mode != 23` (or == 23 but tracking_active == false — the program does
  not start the Waitlist hook)
- `state.csm_nav.tracking_active == false`

---

### TC-P23-3: Star-horizon mark reduces W (well-conditioned geometry)

**Purpose**: Verify the measurement model and Kalman update for a typical cislunar geometry.

**Geometry (ECI frame, Earth at origin)**:
```
csm_pos      = [3.0e8, 0.0, 0.0]        // 300,000 km along X-axis
star_direction = [0.0, 1.0, 0.0] / 1.0  // star along +Y axis (90° from X-axis)
body           = Body::Earth
R_body         = EARTH_RADIUS_M = 6_378_137.0

// Intermediate values:
rho = csm_pos - [0,0,0] = [3.0e8, 0.0, 0.0]
d   = 3.0e8 m
u_hat = [1.0, 0.0, 0.0]
phi = asin(6_378_137 / 3.0e8) ≈ asin(0.02126) ≈ 0.02126 rad
cos_alpha = dot([0,1,0], [1,0,0]) = 0.0
alpha = acos(0.0) = π/2 ≈ 1.5708 rad
theta_pred = α - φ ≈ 1.5708 - 0.02126 ≈ 1.5495 rad
```

**Setup**:
```
state.csm_state = StateVector {
    position: [3.0e8, 0.0, 0.0],
    velocity: [0.0, 800.0, 0.0],
    epoch:    Met(1_000_000),
    frame:    Frame::EarthInertial,
}
// W-matrix initialised to diagonal P23_W_INIT_POS_VARIANCE, P23_W_INIT_VEL_VARIANCE.
p23_init(&mut state);

mark = StarHorizonMark {
    time:                1000.0,   // s
    star_direction:      [0.0, 1.0, 0.0],
    body:                Body::Earth,
    angle_observed_rad:  1.5495,   // matches theta_pred → residual ≈ 0
}
```

**Action**: `p23_incorporate_star_horizon_mark(&mut state, mark)`

**Expected**:
- `state.csm_nav.mark_count == 1`
- `state.csm_nav.reject_count == 0`
- `state.alarm.code == 0`
- `state.csm_nav.w_matrix[1][1] < P23_W_INIT_POS_VARIANCE` (Y-position W reduced)
- `state.csm_nav.w_matrix[3][3] == P23_W_INIT_VEL_VARIANCE` (velocity rows unchanged because `b[3..6] = 0`)
- `state.csm_state.position` changed by at most 1.0 m (near-zero residual, small update)

**Sensitivity analysis**: With `b = A * u_hat - B * s_hat`,
`A = cos_alpha/(d*sin_alpha) + R/(d*tangent_len)`, `B = 1/(d*sin_alpha)`.
At this geometry `cos_alpha = 0`, `sin_alpha = 1`, `d ≈ 3e8`, `tangent_len ≈ 3e8`,
so `A ≈ R/d² ≈ 7.1e-11`, `B ≈ 3.33e-9`.
Therefore:
- `b[0] = A * 1 - B * 0 ≈ 7.1e-11` (tiny)
- `b[1] = A * 0 - B * 1 ≈ -3.33e-9` (dominant)
- `b[2] = 0`

The dominant sensitivity is on the **Y-component** of position (perpendicular to the
body-centre direction, in the plane of the star and body).  A small ΔY in `csm_pos`
tilts `u_hat` by ΔY/d, which shifts `cos_alpha` by the same amount and hence `alpha`
(and `theta_pred`) by ≈ −ΔY/d.

**Tolerance**: `W[1][1]` must be strictly less than `1.0e8`.  Expected reduction:
`W_new[1][1] ≈ W_old[1][1] - (W_old[1][1] * b[1])² / (W_old[1][1] * b[1]² + sigma_sq)`
`≈ 1e8 - (1e8 · 3.33e-9)² / (1e8 · 1.11e-17 + 2.5e-9)`
`≈ 1e8 - 0.111 / 3.61e-9`
`≈ 1e8 - 3.07e7 ≈ 6.93e7 m²`.
The test should assert `W[1][1]` is in range `[5e7, 9e7]` and that `W[0][0]` is
essentially unchanged (within 1 m² of its initial value).

---

### TC-P23-4: Outlier mark rejected by 3-sigma gate

**Purpose**: Verify the 3-sigma gate rejects a large residual.

**Setup**: Same as TC-P23-3 but with `angle_observed_rad = 1.5495 + 1.0` (residual ≈ 1.0 rad).

The innovation variance `S = b^T W b + sigma_sq ≈ 3.61e-9`.
The 3-sigma threshold is `3 * sqrt(3.61e-9) ≈ 1.8e-4 rad`.
A residual of 1.0 rad >> 1.8e-4 rad, so the mark must be rejected.

**Expected**:
- `state.csm_nav.mark_count == 0` (unchanged)
- `state.csm_nav.reject_count == 1`
- `state.csm_nav.consecutive_reject_count == 1`
- `state.csm_state.position` unchanged
- `state.csm_nav.w_matrix` unchanged
- `state.alarm.code == 0` (single rejection does not raise an alarm)

---

### TC-P23-5: Five consecutive rejects raise alarm 01431

**Purpose**: Verify that five consecutive rejections set the alarm and disable tracking.

**Setup**: Same initial state as TC-P23-4. Submit five consecutive marks with
`angle_observed_rad = theta_pred + 1.0` (large outlier).

**Expected** (after 5th mark):
- `state.csm_nav.consecutive_reject_count == 5`
- `state.csm_nav.reject_count == 5`
- `state.csm_nav.tracking_active == false`
- `state.alarm.code == 0o01431` (ALARM_P23_REJECT_OVERRIDE)
- `state.alarm.lit == true`
- `state.csm_state.position` unchanged (all 5 marks rejected)
- 6th mark is silently discarded (`tracking_active == false`)

---

### TC-P23-6: `primary_body` for both frame variants

**Purpose**: Verify the frame-to-body mapping.

**Test A**:
```
assert_eq!(primary_body(Frame::EarthInertial), Body::Earth)
```

**Test B**:
```
assert_eq!(primary_body(Frame::MoonInertial), Body::Moon)
```

These are pure-function unit tests with no `AgcState`.

---

### TC-P23-7: CSM inside body raises alarm 01430

**Purpose**: Verify the horizon-measurement guard against degenerate geometry.

**Setup**:
```
state.csm_state.position = [EARTH_RADIUS_M + 50_000.0, 0.0, 0.0]  // 50 km altitude — below R_MIN_HORIZON_M guard
// R_MIN_HORIZON_M = 100_000 m → guard threshold = 6_378_137 + 100_000 = 6_478_137 m
// CSM is at 6_428_137 m < 6_478_137 m → should trigger alarm

mark = StarHorizonMark {
    time: 1000.0,
    star_direction: [0.0, 1.0, 0.0],
    body: Body::Earth,
    angle_observed_rad: 0.0,
}
```

**Action**: `p23_incorporate_star_horizon_mark(&mut state, mark)`

**Expected**:
- `state.alarm.code == 0o01430` (ALARM_P23_TOO_CLOSE_TO_BODY)
- `state.alarm.lit == true`
- `state.csm_nav.mark_count == 0` (mark discarded, not counted)
- `state.csm_state.position` unchanged

---

### TC-P23-8: Star-landmark mark reduces W

**Purpose**: Verify the star-landmark measurement model and Kalman update.

**Geometry (MCI frame, Moon at origin)**:
```
csm_pos   = [0.0, 1.0e8, 0.0]       // 100,000 km along +Y from Moon
landmark  = [0.0, MOON_RADIUS_M, 0.0]  // sub-CSM point on Moon's limb along +Y

// landmark_inertial = [0, 1_737_400, 0]  (on Moon's surface)
// v_lm = landmark - csm = [0, 1_737_400 - 1e8, 0] = [0, -9.8263e7, 0]
// l_hat = [0, -1.0, 0]   (pointing from CSM toward Moon)
// star_direction = [1.0, 0.0, 0.0]  (star along +X, perpendicular to landmark direction)
// cos_beta = dot([1,0,0], [0,-1,0]) = 0.0
// beta_pred = acos(0) = π/2
```

**Setup**:
```
state.csm_state = StateVector {
    position: [0.0, 1.0e8, 0.0],
    velocity: [100.0, 0.0, 0.0],
    epoch:    Met(1_000_000),
    frame:    Frame::MoonInertial,
}
p23_init(&mut state);

mark = StarLandmarkMark {
    time:              1000.0,
    star_direction:    [1.0, 0.0, 0.0],
    body:              Body::Moon,
    landmark_inertial: [0.0, MOON_RADIUS_M, 0.0],
    angle_observed_rad: core::f64::consts::PI / 2.0,   // matches beta_pred → residual = 0
}
```

**Action**: `p23_incorporate_star_landmark_mark(&mut state, mark)`

**Expected**:
- `state.csm_nav.mark_count == 1`
- `state.csm_nav.reject_count == 0`
- `state.alarm.code == 0`
- `state.csm_nav.w_matrix[0][0] < P23_W_INIT_POS_VARIANCE` (X-position row reduced)
- `state.csm_nav.w_matrix[1][1] ≈ P23_W_INIT_POS_VARIANCE` (Y-position unchanged — `b[1] = 0`)
- `state.csm_state.position` changed by at most 1.0 m (near-zero residual)

**Derivation check for `b`** (verify the `b[0..3]` formula from §6.2):
```
l_hat = [0, -1, 0]
cos_beta = 0.0
sin_beta = 1.0
d_lm = norm([0, 1_737_400 - 1e8, 0]) = 9.8263e7 m

b[0..3] = (s_hat - cos_beta * l_hat) / (d_lm * sin_beta)
        = ([1,0,0] - 0 * [0,-1,0]) / (9.8263e7 * 1)
        = [1,0,0] / 9.8263e7
        ≈ [1.018e-8, 0.0, 0.0]
```
The sensitivity is purely along the X direction (star along X, landmark along Y — the
angle between them is sensitive only to X-position change). Therefore `W[0][0]` is
reduced and `W[1][1]` remains essentially unchanged.

---

## 12. Open Questions for Architect Review

**OQ-1 — Moon-fixed to MCI rotation for landmark marks**

`StarLandmarkMark.landmark_inertial` for Moon landmarks requires converting from
selenographic (Moon-fixed) coordinates to MCI inertial coordinates.  The Moon rotates
synchronously with its orbital period (~27.3 days); the rotation model is not
implemented in `navigation::planetary` as of Phase 3.  Required: either a Moon-rotation
helper function `moon_ef_to_mci(landmark_selenographic: Vec3, epoch: Met) -> Vec3`, or a
decision to restrict `StarLandmarkMark` to Earth-only landmark sightings in Phase 4 and
defer Moon-landmark support.

**OQ-2 — P22 and P23 concurrent Waitlist tasks sharing `csm_nav`**

Both `p22_cycle_task` and `p23_cycle_task` grow `state.csm_nav.w_matrix[i][i]` using
`last_mark_time` as the Δt reference.  If both programs are active simultaneously (a
reachable state: P22 did not exit before the crew entered P23) both tasks would double-count
the process-noise growth.  The architect must decide whether `p23_init` should explicitly
stop the P22 Waitlist task (by clearing any pending P22 entry) or whether to add a
`current_nav_program` flag to `CsmNavState` to gate each cycle task.  Given the cooperative
scheduling model, the simplest fix is for `p23_init` to set `state.major_mode = 23` and
for `p22_cycle_task` to check `state.major_mode != 22` at entry and return without
rescheduling if P22 is no longer the active program.

**OQ-3 — N49 / N45 noun layout for P23**

The P20/P22 specs assign Noun 49 (reject counter) and Noun 45 (mark counter) to specific
registers.  P23 proposes to overload these same nouns.  The O'Brien p. 329 Table 11.3
lists noun assignments for P23 but the relevant page was not available for verification.
The developer must confirm from `P20-P25.agc` that the P23 N49 display layout (mark count
in R1, reject count in R2) is consistent with the Comanche055 source, or propose a
different noun.

**OQ-4 — Body-centre position in the non-primary frame**

The spec simplifies by placing the body centre at the origin of the current frame (§6.4).
For the full translunar/transearth mission, the crew may want to sight against the Moon
while the CSM is still in ECI frame (before SOI crossing), or against Earth while in MCI
frame.  These cases require the off-origin body position from `navigation::planetary`.
The architect should decide whether to implement this in Phase 4 or defer to a later phase
(flagging it as a known accuracy limitation when the non-primary body is observed from
the non-native frame).

**OQ-5 — `p23_init` W-matrix preservation decision boundary**

The spec says: "if `w_matrix` is all zeros, initialise; otherwise preserve." A W-matrix
left by a prior P22 session uses P22's initial variances (250,000 m² ≈ 500 m σ), which
is far smaller than the cislunar P23 initial variance (1.0e8 m² ≈ 10 km σ).  If P22 ran
in LEO and then the crew enters P23 after TLI, the preserved W-matrix would be
overconfident relative to the cislunar navigation accuracy.  The architect should consider
whether `p23_init` should always re-initialise W to `P23_W_INIT_POS_VARIANCE`, or only
when the frame changes (ECI→ECI is ok; MCI→ECI suggests a fresh context).  Alternatively,
add a `nav_program: u8` tag to `CsmNavState` and re-initialise if it was last set to 22
(a different program's session).
