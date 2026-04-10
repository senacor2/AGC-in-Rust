# Specification: `programs/p21` and `programs/p22` — Ground-Track Determination and Orbital Navigation (Landmark Tracking)

**Status**: Ready for implementation (Milestone 5 Phase 3)
**Module paths**:
- `agc-core/src/programs/p21.rs` — new file
- `agc-core/src/programs/p22.rs` — new file
**Architecture reference**: `docs/architecture.md` §7.2 (P21/P22 rows), §9 (Navigation Math)
**Rendezvous primitives reference**: `specs/rendezvous-spec.md` — frame conventions adopted unchanged
**P20 reference**: `specs/p20-spec.md` — scalar Kalman algorithm (§6) and Waitlist scheduling pattern
  reused verbatim by P22; `RendezvousNavState` field-naming conventions apply here too
**State-vector reference**: `specs/state-vector-spec.md` §2.1–§2.5 (`StateVector`, `Frame`)
**Executive reference**: `specs/executive-spec.md` §2.2 (Waitlist self-rescheduling pattern)
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, Comanche055 directory):
- `Comanche055/P20-P25.agc` — P21 and P22 entry sequences, crew-input nouns
- `Comanche055/R60,R62.agc` — R60/R62 ground-track computation routines called by P21
- `Comanche055/LAT-LONG_SUBROUTINES.agc` — geocentric lat/lon/alt conversion from inertial position
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `LAT`, `LONG`, `ALT` display erasables; landmark table;
  `GHABASE` (Greenwich Hour Angle at reference epoch); `CSMNAVSAV` (CSM nav state for P22)
- `Comanche055/MEASUREMENT_INCORPORATION.agc` — scalar Kalman update (shared with P20/P22)
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
  Operation*, Springer-Praxis 2010.
- Chapter 11, "Rendezvous and Navigation Programs" pp. 303–340
  - pp. 303–310: P21 ground-track display program (crew invocation, R60/R62 role, noun assignments)
  - pp. 310–315: P22 landmark tracking overview, measurement model introduction
  - pp. 318–325: Scalar Kalman incorporation (shared algorithm; see `specs/p20-spec.md` §6)
  - pp. 325–327: W-matrix rectification; process-noise model applicable to P22

---

## 1. Purpose and Program Roles

### 1.1 P21 — Ground-Track Determination

P21 is a **display-only** program. Given a crew-entered target GET (Ground Elapsed Time),
it propagates the CSM state vector forward from its current epoch to that future time using
`math::kepler::kepler_step` and then computes the sub-satellite point — the geographic
point on the Earth's surface directly below the CSM at that time. The result (geocentric
latitude, longitude, and altitude above the spherical Earth reference) is displayed on the
DSKY via Verb 06 Noun 43.

P21 makes **no measurements**, performs **no state updates**, and does **not reschedule
itself**. It is a one-shot computation triggered by a crew request. After the display is
written it terminates; the crew may request another computation by re-entering the program.

Mission context: P21 is useful for predicting ground-station contact windows, verifying
the insertion orbit, or planning landmark-tracking sessions. It is invoked between burns,
typically during the coast phase.

### 1.2 P22 — Orbital Navigation (Landmark Tracking)

P22 is the **navigation counterpart of P20** but for the CSM's own state vector (not the
target's). Instead of radar marks on the LM, it uses **sextant sightings of Earth
landmarks** at known Earth-fixed positions. Each sighting constrains the CSM's inertial
position and, after sufficient marks from different landmarks or different orbital
positions, converges the CSM state vector to improved accuracy.

The measurement model and scalar Kalman filter algorithm are **identical to those in P20**.
The difference is that the sensitivity vector `b` is computed with respect to the **CSM
state** (not the target state), and the result is applied to a **separate CSM covariance
matrix** `CsmNavState.w_matrix` and to `state.csm_state` directly. The `RendezvousNavState`
used by P20 is **not modified** by P22.

P22 is a periodic background program. Like P20 it installs a Waitlist self-rescheduling
hook to grow process noise and refresh the DSKY display each cycle. Landmark marks arrive
asynchronously via `p22_incorporate_landmark_mark` called from the sextant HAL handler.

Mission context: P22 is used when the IMU-integrated CSM state vector has drifted and
ground uplinks are unavailable or infrequent. A typical session involves five to ten
landmark sightings over one to two orbital revolutions.

---

## 2. Module Paths

- P21: `agc-core/src/programs/p21.rs`
- P22: `agc-core/src/programs/p22.rs`

Both are registered in `agc-core/src/programs/mod.rs`:

```rust
pub mod p21;
pub mod p22;
```

Entry points are registered in `PROGRAM_TABLE`:
- `PROGRAM_TABLE[21] = p21_init`
- `PROGRAM_TABLE[22] = p22_init`

The legacy `p20_p22.rs` stub file (which previously held `todo!()` stubs for P21 and P22)
is deleted. Any re-exports it provided are replaced by direct `pub use` declarations in
`programs/mod.rs`.

---

## 3. State Additions

### 3.1 P21 — no new state

P21 is a one-shot computation. All state needed to carry out the computation already exists:

| Datum | Source |
|-------|--------|
| CSM position and velocity | `state.csm_state` (`StateVector`) |
| Current time | `state.time` |
| GHA at epoch | `state.navigation.gha_epoch` (new field; see §3.3) |

No struct additions are needed in `AgcState` for P21 alone.

### 3.2 P22 — `CsmNavState` struct

P22 needs a covariance matrix and tracking bookkeeping for the CSM's own state. This is
separate from `RendezvousNavState` (which tracks the *target*). A new struct is added as
a field of `AgcState`:

```rust
/// Navigation state maintained by P22 (Orbital Navigation / Landmark Tracking).
///
/// Holds the CSM state-error covariance and tracking bookkeeping.
/// The CSM's best-estimate position and velocity are in `AgcState::csm_state`
/// (a `StateVector`); this struct holds only the uncertainty model and counters.
///
/// Lives inside `AgcState` so the Executive, SERVICER, and restart handler
/// can all reach it without extra arguments.
#[derive(Clone, Debug)]
pub struct CsmNavState {
    /// 6×6 state-error covariance (W-matrix) for the CSM's own state, in SI units.
    /// Rows/columns 0..2 are position components (m²).
    /// Rows/columns 3..5 are velocity components (m²/s²).
    /// The matrix is always symmetric; the full array is stored for clarity.
    /// Analogous to `RendezvousNavState::w_matrix` (P20), but applies to the CSM.
    pub w_matrix: [[f64; 6]; 6],

    /// Mission Elapsed Time of the last accepted landmark mark (s).
    /// Used to compute Δt for process-noise growth in `p22_cycle`.
    pub last_mark_time: f64,

    /// Count of accepted landmark marks since P22 was initialised.
    /// Displayed on DSKY via V06 N45 (same noun as P20 mark counter).
    pub mark_count: u16,

    /// Count of landmark marks rejected by the 3-sigma gate since P22 start.
    /// Displayed on DSKY via V06 N49.
    pub reject_count: u16,

    /// Count of *consecutive* rejected marks (reset on any accepted mark).
    /// Raises alarm 01422 when it reaches 5 (persistent landmark tracking failure).
    pub consecutive_reject_count: u16,

    /// True when P22 is active and incorporating marks.
    /// Set to false on alarm 01422 (persistent rejection) or crew command.
    pub tracking_active: bool,
}
```

**Default initialisation**:

```rust
impl Default for CsmNavState {
    fn default() -> Self {
        Self {
            w_matrix:                  [[0.0; 6]; 6],
            last_mark_time:            0.0,
            mark_count:                0,
            reject_count:              0,
            consecutive_reject_count:  0,
            tracking_active:           false,
        }
    }
}
```

**`AgcState` extension** — the following fields are added:

```rust
/// Navigation state maintained by P22 (Landmark Tracking).
pub csm_nav: CsmNavState,
```

### 3.3 GHA storage — `NavigationConstants.gha_epoch`

Both P21 and P22 require the Greenwich Hour Angle (GHA) at the mission reference epoch
(GET = 0) in order to rotate between Earth-fixed and inertial frames. This value is
uplinked by Mission Control before the mission and stored in an existing or new navigation
constants block. It is proposed to store it on `AgcState::navigation` (the navigation
parameters struct, if one exists) or, if that struct does not yet exist, as a top-level
field:

```rust
/// Greenwich Hour Angle at the navigation epoch (GET = 0.0 s), in radians.
/// Positive eastward.  This is set by uplink prior to orbital insertion and is
/// treated as a constant for the duration of a navigation session.
/// Corresponds to AGC erasable `GHABASE` (Comanche055 ERASABLE_ASSIGNMENTS.agc).
pub gha_epoch_rad: f64,
```

If `AgcState` already has a `navigation` sub-struct, `gha_epoch_rad` is added to it.
If not, it is added as a top-level field on `AgcState`. The architect should decide the
placement; P21 and P22 implementations access it as `state.gha_epoch_rad` or
`state.navigation.gha_epoch_rad` — the spec uses `state.gha_epoch_rad` throughout.

**Earth rotation rate constant** (compile-time, not stored in state):

```rust
/// Mean Earth rotation rate (rad/s).  IAU standard value.
/// Used to compute GHA at any GET: gha(t) = gha_epoch_rad + OMEGA_EARTH * t
pub const OMEGA_EARTH: f64 = 7.292_115_085_5e-5; // rad/s
```

### 3.4 Landmark table

P22 requires a table of known Earth landmarks, each identified by an index keyed by the
crew when initiating a sighting. The landmark entry provides the Earth-fixed Cartesian
position (or equivalently, geocentric lat/lon/alt). This table is a compile-time constant
array in `programs/p22.rs` for the Rust port (the original AGC stored it in fixed memory
and allowed partial ground-uplinking; uplink support is deferred):

```rust
/// A single landmark entry: Earth-fixed geodetic coordinates.
#[derive(Clone, Copy, Debug)]
pub struct LandmarkEntry {
    /// Geocentric latitude (rad).  Positive north.
    pub lat_rad: f64,
    /// Longitude east of the IERS reference meridian (rad).
    pub lon_rad: f64,
    /// Altitude above the spherical Earth surface (m).
    pub alt_m: f64,
}

/// Pre-loaded landmark table.  Index 0 is unused (landmarks are 1-indexed on the DSKY).
/// The table is fixed at compile time; uplink support is a future extension.
pub const LANDMARK_TABLE: [LandmarkEntry; 9] = [ /* see §9 for test values */ ... ];
```

The size (8 landmarks + 1 unused slot) matches the original Comanche055 allocation.

---

## 4. Public API

### 4.1 P21 public API

```rust
/// Major mode number for P21.
pub const P21_MAJOR_MODE: u8 = 21;

/// Job priority for P21.  One level below P22 (which is periodic) because P21
/// is one-shot and non-time-critical.
pub const P21_PRIORITY: JobPriority = 7;
```

```rust
/// Entry point for P21 (Ground-Track Determination).
/// Registered in PROGRAM_TABLE[21].
///
/// Sets `state.major_mode = 21`.  Prompts the crew to enter the target GET via
/// `V06 N34 E <centiseconds> E`.  Then calls `p21_compute_ground_track` and
/// displays the result via `V06 N43`.  On completion, returns to idle
/// (does NOT reschedule itself).
///
/// # Preconditions
/// - `state.csm_state` must have a non-zero epoch; otherwise alarm 01420 is raised
///   and the program displays the alarm before returning.
/// - `state.gha_epoch_rad` must have been set by uplink or crew entry.
///
/// # Post-conditions
/// - `state.major_mode == 21`
/// - `state.dsky.prog == 21`
/// - DSKY registers set to lat/lon/alt at the target GET (if no alarm).
/// - No periodic hook installed.
pub fn p21_init(state: &mut AgcState) -> JobPriority
```

```rust
/// Compute the sub-satellite point for the CSM at the given target GET.
///
/// This is the pure-computation core of P21.  It is separated from `p21_init`
/// so that unit tests can exercise it directly without a full `AgcState`.
///
/// # Arguments
/// - `csm_pos`: CSM inertial position at the known epoch (m, ECI).
/// - `csm_vel`: CSM inertial velocity at the known epoch (m/s, ECI).
/// - `epoch_s`: GET of the known epoch (s).
/// - `target_get_s`: GET at which the sub-satellite point is requested (s).
/// - `gha_epoch_rad`: Greenwich Hour Angle at GET = 0 (rad).
///
/// # Returns
/// `GroundTrackResult` containing geocentric latitude, longitude, and altitude.
///
/// # Panics
/// Panics if `norm(csm_pos) == 0` (CSM at Earth centre — physically impossible).
pub fn p21_compute_ground_track(
    csm_pos:        Vec3,
    csm_vel:        Vec3,
    epoch_s:        f64,
    target_get_s:   f64,
    gha_epoch_rad:  f64,
) -> GroundTrackResult
```

```rust
/// Result of a P21 ground-track computation.
#[derive(Clone, Copy, Debug)]
pub struct GroundTrackResult {
    /// Geocentric latitude at the target GET (rad).  Range [-π/2, +π/2].
    /// Positive north.
    pub lat_rad: f64,
    /// Longitude at the target GET (rad).  Range (-π, +π].
    /// Positive east, measured from the IERS reference meridian.
    pub lon_rad: f64,
    /// Altitude above the spherical Earth reference (m).
    /// Reference sphere radius: `R_EARTH` (see §4.1 constants below).
    pub alt_m: f64,
}
```

```rust
/// Mean Earth radius used by P21 (spherical approximation).
/// Source: IAU 2012, WGS84 mean equatorial radius rounded to 1 m.
pub const R_EARTH: f64 = 6_371_000.0; // m
```

### 4.2 P22 public API

```rust
/// Major mode number for P22.
pub const P22_MAJOR_MODE: u8 = 22;

/// Job priority for P22.  Same as P20 (both are background navigation loops).
pub const P22_PRIORITY: JobPriority = 8;

/// Waitlist cycle period for P22 (centiseconds).  2-second cycle, identical to P20.
pub const P22_CYCLE_CS: u32 = 200;
```

```rust
/// Entry point for P22 (Orbital Navigation / Landmark Tracking).
/// Registered in PROGRAM_TABLE[22].
///
/// Sets `state.major_mode = 22`.  Initialises `state.csm_nav` with the default
/// diagonal W-matrix.  Prompts the crew for the landmark index (V06 N73 or V01 N73
/// — see §7).  Installs the Waitlist self-rescheduling hook for the 2-second
/// update cycle.
///
/// # Preconditions
/// - `state.csm_state` must have a non-zero epoch; otherwise alarm 01420 is raised.
/// - `state.gha_epoch_rad` must have been set by uplink.
///
/// # Post-conditions
/// - `state.major_mode == 22`
/// - `state.dsky.prog == 22`
/// - `state.csm_nav.w_matrix` is the default diagonal (see §4.2 constants).
/// - `state.csm_nav.tracking_active == true`
/// - Waitlist entry scheduled for `P22_CYCLE_CS` centiseconds.
pub fn p22_init(state: &mut AgcState) -> JobPriority
```

```rust
/// Periodic P22 navigation update task.  Scheduled via Waitlist::schedule.
///
/// Called every `P22_CYCLE_CS` centiseconds (≈ 2 s) after `p22_init`.
///
/// Steps per cycle:
/// 1. Compute Δt since last mark/cycle; grow W-matrix diagonal by process noise.
/// 2. Update DSKY display registers (V16 N43 — lat/lon/alt of current sub-satellite
///    point, derived from current `state.csm_state`).
/// 3. Re-schedule itself: `Waitlist::schedule(state, P22_CYCLE_CS, p22_cycle_task)`.
///
/// # Invariants
/// - Does not modify `state.csm_state` (propagation of the CSM SV is the SERVICER's
///   responsibility; P22 reads the current propagated position from `state.csm_state`).
/// - Does not incorporate marks (marks arrive via `p22_incorporate_landmark_mark`).
/// - Runs even when `tracking_active == false` (display update continues).
pub fn p22_cycle_task(state: &mut AgcState)
```

```rust
/// Incorporate one sextant landmark mark into the CSM navigation solution.
///
/// Called from the sextant HAL handler when the crew completes a mark on a
/// known landmark.  Applies the scalar Kalman update (identical algorithm to
/// `p20_incorporate_radar_mark`; see `specs/p20-spec.md` §6) to
/// `state.csm_state` and `state.csm_nav.w_matrix`.
///
/// The measurement model is a single component of the predicted LOS unit vector
/// from the CSM to the landmark in the inertial frame, compared with the observed
/// component from the sextant.
///
/// # Arguments
/// - `mark`: decoded sextant observation of an Earth landmark (see §5).
///
/// # Preconditions
/// - `state.csm_nav.tracking_active == true`.  If false, the mark is silently
///   discarded (consistent with P20 behaviour when `tracking_active == false`).
/// - `mark.landmark_inertial` must have been populated by `landmark_inertial_pos`
///   before this function is called (the caller converts Earth-fixed to inertial).
///
/// # Post-conditions (mark accepted)
/// - `state.csm_state.pos` and `state.csm_state.vel` updated by the Kalman gain.
/// - `state.csm_nav.w_matrix` rank-1 downgraded.
/// - `state.csm_nav.mark_count` incremented.
/// - `state.csm_nav.last_mark_time` set to `mark.time`.
/// - `state.csm_nav.consecutive_reject_count` reset to 0.
///
/// # Post-conditions (mark rejected — residual > 3-sigma)
/// - `state.csm_nav.reject_count` incremented.
/// - `state.csm_nav.consecutive_reject_count` incremented.
/// - State and W-matrix unchanged.
/// - If `consecutive_reject_count == 5`, raises alarm 01422 and sets
///   `tracking_active = false`.
///
/// # Alarms
/// - 01421: W-matrix diagonal goes negative (overflow); calls `p22_rectify_w_matrix`.
/// - 01422: Five consecutive rejected marks.
pub fn p22_incorporate_landmark_mark(state: &mut AgcState, mark: LandmarkMark)
```

```rust
/// Re-initialise the P22 W-matrix to the default diagonal.
///
/// Called on crew command (V32E) or automatically when alarm 01421 fires.
/// Resets mark_count, reject_count, and consecutive_reject_count to 0.
/// Sets last_mark_time to state.time.
///
/// Identical in structure to `p20_rectify_w_matrix`; see `specs/p20-spec.md` §4.5.
pub fn p22_rectify_w_matrix(state: &mut AgcState)
```

```rust
/// Convert a landmark's Earth-fixed geocentric coordinates to an inertial position
/// vector at the given GET.
///
/// Helper used by the sextant mark handler before calling
/// `p22_incorporate_landmark_mark`.  Also usable by P21 tests.
///
/// # Arguments
/// - `entry`: the landmark table entry (lat/lon/alt in Earth-fixed coordinates).
/// - `get_s`: Ground Elapsed Time at which the inertial position is required (s).
/// - `gha_epoch_rad`: GHA at GET = 0 (rad).
///
/// # Returns
/// Inertial position vector (m, ECI) of the landmark at the given GET.
///
/// # Algorithm
/// 1. Compute Earth-fixed Cartesian position from lat/lon/alt:
///    r_ef = (R_EARTH + alt) * [cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)]
/// 2. Compute current GHA:  gha = gha_epoch_rad + OMEGA_EARTH * get_s
/// 3. Rotate from Earth-fixed to ECI by angle gha about the Z-axis:
///    r_inertial = Rz(-gha) * r_ef
pub fn landmark_inertial_pos(
    entry:          &LandmarkEntry,
    get_s:          f64,
    gha_epoch_rad:  f64,
) -> Vec3
```

**P22 constants**:

```rust
/// Initial position variance on the P22 CSM W-matrix diagonal (m²).
/// Corresponds to ±500 m (1-sigma) positional uncertainty — same magnitude as
/// P20's `W_INIT_POS_VARIANCE` but applied to the CSM state.
pub const CSM_W_INIT_POS_VARIANCE: f64 = 250_000.0;   // (500 m)²

/// Initial velocity variance on the P22 CSM W-matrix diagonal (m²/s²).
pub const CSM_W_INIT_VEL_VARIANCE: f64 = 1.0;          // (1 m/s)²

/// Process-noise rate for CSM position (m²/s).
/// Same value as P20's Q_POS; the unmodelled-force environment is identical.
pub const CSM_Q_POS: f64 = 0.5;   // m²/s

/// Process-noise rate for CSM velocity (m²/s³).
pub const CSM_Q_VEL: f64 = 1.0e-6; // m²/s³

/// Sextant landmark LOS noise variance (rad²).
/// 1-sigma ≈ 0.1 mrad (same as P20 sextant marks); landmarks have similar
/// observational uncertainty to target vehicle sightings.
/// Corresponds to `SIGMA_SEXTANT_SQ` in p20.rs; redeclared here for P22 clarity.
pub const SIGMA_LANDMARK_SQ: f64 = 1.0e-8; // (0.1 mrad)²

/// Minimum CSM-to-landmark slant range for mark incorporation (m).
/// Below 200 km the flat-Earth approximation for the sensitivity vector begins
/// to fail; this lower bound is generous because landmark ranges are always
/// ≥ 200 km for orbital altitudes ≥ 200 km.
pub const MIN_LANDMARK_RANGE_M: f64 = 1_000.0; // safety floor only; in practice always > 200 km
```

---

## 5. Measurement Types

### 5.1 `LandmarkMark` (P22 only)

```rust
/// A single sextant sighting of an Earth landmark, decoded by the sextant HAL handler.
///
/// The sextant handler converts shaft and trunnion CDU angles into a body-frame LOS
/// unit vector; the IMU REFSMMAT then rotates this to the inertial frame.  The
/// landmark's Earth-fixed coordinates are looked up from `LANDMARK_TABLE` using
/// `landmark_index`, and `landmark_inertial_pos` is called to compute the landmark's
/// inertial position at `time`.  Both results are packaged into this struct before
/// `p22_incorporate_landmark_mark` is called.
///
/// Structurally similar to `SextantMark` (P20) but carries the landmark reference
/// rather than the target vehicle reference.  See `specs/p20-spec.md` §5.2 for
/// the analogous P20 type.
#[derive(Clone, Copy, Debug)]
pub struct LandmarkMark {
    /// Ground Elapsed Time of the sighting (s).
    pub time: f64,

    /// Index into `LANDMARK_TABLE` (1-indexed; 0 is invalid).
    pub landmark_index: u8,

    /// Inertial position of the landmark at `time` (m, ECI).
    /// Pre-computed by `landmark_inertial_pos` before this struct is delivered
    /// to `p22_incorporate_landmark_mark`.
    pub landmark_inertial: Vec3,

    /// LOS unit vector from the CSM to the landmark, in the **inertial frame** (ECI).
    /// Magnitude must be 1.0 ± 1e-6.
    /// Derived from the sextant shaft/trunnion angles rotated by REFSMMAT.
    pub los_inertial: Vec3,

    /// Which scalar component of the LOS unit vector is the observation for this mark.
    /// The caller selects the axis whose `los_inertial` component has the smallest
    /// absolute value, maximising numerical conditioning.
    /// Reuses `LosComponent` from `programs::p20`.
    pub component: LosComponent,
}
```

`LosComponent` is imported from `programs::p20` (it is already public there):

```rust
use crate::programs::p20::LosComponent;
```

---

## 6. Algorithm Specifications

### 6.1 P21 — Ground-Track Computation

`p21_compute_ground_track` performs the following steps. All angles are in radians unless
stated otherwise.

**Step 1 — Propagate CSM state to target GET**

```
delta_t = target_get_s - epoch_s                (s; may be negative)
(pos_t, vel_t) = kepler_step(csm_pos, csm_vel, delta_t, MU_EARTH)
```

`kepler_step` is from `math::kepler`. `MU_EARTH = 3.986_004_418e14` m³/s² (standard
gravitational parameter). Positive `delta_t` propagates forward; the kepler step handles
backward propagation (`delta_t < 0`) without special-casing.

**Step 2 — Compute current GHA**

```
gha = gha_epoch_rad + OMEGA_EARTH * target_get_s     (rad, unbounded)
gha = gha mod (2π)                                   (normalise to [0, 2π))
```

The GHA is the angle from the vernal equinox direction (ECI X-axis) to the Greenwich
meridian, measured positive east. At GET = 0 it equals `gha_epoch_rad`. It advances
by `OMEGA_EARTH` per second.

**Step 3 — Rotate inertial position to Earth-fixed frame**

The Earth-fixed frame is obtained by rotating the ECI frame by angle `gha` about the
Z-axis (positive rotation aligns the Greenwich meridian with the X-axis of the rotated
frame). Using the standard rotation matrix `Rz(gha)`:

```
pos_ef[0] =  pos_t[0] * cos(gha) + pos_t[1] * sin(gha)
pos_ef[1] = -pos_t[0] * sin(gha) + pos_t[1] * cos(gha)
pos_ef[2] =  pos_t[2]
```

Note: Rotating by `+gha` takes the inertial vector into the Earth-fixed frame because the
Earth has rotated by `+gha` since epoch; to undo that rotation we apply `Rz(+gha)` (the
inverse of `Rz(-gha)`).

**Step 4 — Extract geocentric latitude, longitude, and altitude**

```
r_mag = norm(pos_ef)                             (m; = norm(pos_t), invariant under rotation)
lat   = asin(pos_ef[2] / r_mag)                  (rad; geocentric; range [-π/2, +π/2])
lon   = atan2(pos_ef[1], pos_ef[0])              (rad; range (-π, +π])
alt   = r_mag - R_EARTH                          (m; altitude above spherical reference)
```

**Spherical-Earth limitation**: The AGC rendezvous programs used a spherical Earth model
for all navigation computations (oblateness corrections were applied only in P61–P67
entry guidance). Geocentric latitude is used here, not geodetic latitude. At orbital
altitudes this introduces errors of up to ~20 km in the displayed ground position for
high-inclination orbits, which was acceptable for the planning purposes of P21. This
limitation is documented in the spec and does not require a correction in the Rust port.

**Step 5 — Pack result and return**

```rust
GroundTrackResult { lat_rad: lat, lon_rad: lon, alt_m: alt }
```

**DSKY display conversion** (performed in `p21_init`, not in the pure-computation function):

| DSKY register | Content | Conversion |
|--------------|---------|-----------|
| R1 | Geocentric latitude | degrees × 100 as signed integer; lat_deg = lat_rad × (180/π) |
| R2 | Longitude | degrees × 100 as signed integer; lon_deg = lon_rad × (180/π) |
| R3 | Altitude | km × 10 as integer; alt_km = alt_m / 100.0 |

The original AGC displayed latitude in units of 10⁻⁴ revolutions (Comanche055 `LAT`/`LONG`
erasables, O'Brien p. 303). The Rust DSKY layer handles fixed-point encoding; the spec
passes SI values to the display subsystem and the DSKY driver performs scaling.

### 6.2 P22 — Landmark Measurement Update

P22 reuses the **scalar Kalman measurement-update algorithm** defined in `specs/p20-spec.md`
§6 verbatim. No re-derivation is given here. The key adaptations for P22 are:

**State vector redefinition**: For P22 the 6-element state vector `x` is:

```
x = [ csm_pos[0], csm_pos[1], csm_pos[2],
      csm_vel[0], csm_vel[1], csm_vel[2] ]^T
```

where `csm_pos` and `csm_vel` are read from `state.csm_state.pos` and `state.csm_state.vel`.

**Reference point redefinition**: The "observer" is now the **landmark** (the known fixed
point), and the "observed vehicle" is the **CSM**. The relative position vector is:

```
rho_vec = csm_pos - landmark_inertial     (m; inertial, ECI)
```

(CSM minus landmark, so `rho_vec` points from the landmark to the CSM; this is the
direction along which the LOS is measured.)

**Measurement prediction**: For a LOS component-`c` mark:

```
los_hat_predicted = unit(rho_vec)         (predicted LOS unit vector from landmark to CSM)
z_predicted       = los_hat_predicted[c]  (predicted direction cosine of component c)
```

**Sensitivity vector `b`**: The partial derivative of `z_predicted` w.r.t. `csm_pos`
is identical in form to the P20 sextant sensitivity vector (the denominator uses the
CSM-to-landmark slant range):

```
rng = norm(rho_vec)                       (m)
b[0..3] = (e_c - los_hat_predicted[c] * los_hat_predicted) / rng
          where e_c is the unit vector for axis c
b[3..6] = [0.0; 3]                        (LOS direction cosine does not depend on CSM velocity)
```

**W-matrix**: Use `state.csm_nav.w_matrix` (not `state.rendezvous_nav.w_matrix`).

**State update target**: Apply Kalman gain corrections to `state.csm_state.pos` and
`state.csm_state.vel` (not to `rendezvous_nav.target_pos`/`target_vel`).

**Noise variance**: Use `SIGMA_LANDMARK_SQ` (= `1.0e-8` rad²).

**Rejection bookkeeping**: Use `state.csm_nav.reject_count`,
`state.csm_nav.consecutive_reject_count`, and `state.csm_nav.mark_count`.

The internal helper `scalar_measurement_update` defined in `programs/p20.rs` is pub(crate)
accessible; P22 calls it with the adapted state slice and sensitivity vector. If the
architect prefers to move it to a shared location (e.g. `navigation::kalman`), that is an
open question (see §11).

**Process-noise growth** (in `p22_cycle_task`):

```
Δt = state.time - state.csm_nav.last_mark_time    (s)
if Δt > 3600.0:
    p22_rectify_w_matrix(state)
    return
for i in 0..3:
    state.csm_nav.w_matrix[i][i] += CSM_Q_POS * Δt
for i in 3..6:
    state.csm_nav.w_matrix[i][i] += CSM_Q_VEL * Δt
```

This is exactly the P20 process-noise model (`specs/p20-spec.md` §7) with
`CSM_Q_POS`/`CSM_Q_VEL` instead of `Q_POS`/`Q_VEL`.

### 6.3 `landmark_inertial_pos` — Earth-Fixed to Inertial Conversion

This helper is called by the sextant mark handler before delivering a `LandmarkMark` to
`p22_incorporate_landmark_mark`.

**Step 1 — Earth-fixed Cartesian position**:

```
R = R_EARTH + entry.alt_m                                     (m)
r_ef = [R * cos(lat) * cos(lon),
         R * cos(lat) * sin(lon),
         R * sin(lat)]
```

where `lat = entry.lat_rad`, `lon = entry.lon_rad`.

**Step 2 — GHA at mark time**:

```
gha = gha_epoch_rad + OMEGA_EARTH * get_s    (rad, unbounded)
```

**Step 3 — Rotate Earth-fixed to ECI** (rotate by `-gha`, i.e. apply `Rz(-gha)`):

```
r_inertial[0] = r_ef[0] * cos(gha) - r_ef[1] * sin(gha)
r_inertial[1] = r_ef[0] * sin(gha) + r_ef[1] * cos(gha)
r_inertial[2] = r_ef[2]
```

Note the sign of this rotation is **opposite** to Step 3 in §6.1: P21 converts inertial
to Earth-fixed (rotate by `+gha`); this helper converts Earth-fixed to inertial (rotate
by `-gha`). The two rotations are inverses of each other.

---

## 7. DSKY Interaction

### 7.1 P21 DSKY sequence

| Step | Verb/Noun | Meaning |
|------|-----------|---------|
| 1 | `V37E 21E` | Crew selects P21 (major mode 21). |
| 2 | `V06 N34 E` | P21 prompts crew to load target GET. Noun 34 = elapsed time in centiseconds (3 registers = HH MM SS.cc). Crew keys time then `E`. |
| 3 | `V06 N43 E` | P21 displays result. Noun 43 = latitude / longitude / altitude. R1 = latitude (deg×100), R2 = longitude (deg×100), R3 = altitude (km×10). |
| 4 | —          | P21 terminates. Major mode reverts to the idle program (P00) or the crew re-enters. |

**Noun 34 encoding**: The AGC Noun 34 displays elapsed time as three registers in
HH:MM:SS format, scaled to centiseconds internally. For the Rust port, the DSKY
input handler delivers the decoded GET in seconds as a plain `f64` to `p21_init`.
The DSKY layer handles the HH:MM:SS encoding.

**Noun 43 register assignment** (O'Brien p. 303, Table 11.0 — verified against
`Comanche055/P20-P25.agc` P21 section):

| Register | AGC variable | Content | Scale |
|----------|-------------|---------|-------|
| R1 | `LAT` | Geocentric latitude | 10⁻⁴ revolutions → ±90° range |
| R2 | `LONG` | Longitude | 10⁻⁴ revolutions → 0°–360° east |
| R3 | `ALT` | Altitude | 0.1 NM per bit (in Rust port: raw metres; DSKY driver scales) |

**Note on longitude convention**: The AGC `LONG` erasable stored longitude as 0 to 1
revolution (0 to 2π), positive east (O'Brien p. 304). The Rust port uses (-π, +π] but
the DSKY driver wraps to the 0–360° east convention for display purposes.

### 7.2 P22 DSKY sequence

| Step | Verb/Noun | Meaning |
|------|-----------|---------|
| 1 | `V37E 22E` | Crew selects P22. |
| 2 | `V01 N73 E` | P22 prompts crew to enter the landmark index (1–8). Noun 73 = landmark number. Crew keys index then `E`. |
| 3 | `V16 N43` | P22 begins continuous display: current sub-satellite point of the CSM. Updated every 2-second cycle. Same register layout as P21 N43. |
| 4 | —          | Crew performs a sextant mark. The sextant HAL delivers a `LandmarkMark` to `p22_incorporate_landmark_mark`. |
| 5 | `V06 N45 E` | Crew request: display mark counter (R1 = mark_count). |
| 6 | `V06 N49 E` | Crew request: display reject counter (R1 = reject_count). |
| 7 | `V32 E` | Crew command: re-initialise W-matrix (`p22_rectify_w_matrix`). |
| 8 | `V34 E` | Crew terminates P22; returns to P00. |

**Noun 73** is the landmark number input. The AGC P22 used this noun (or the equivalent
in the Comanche055 P22 section) for landmark identification before accepting a mark.
The crew enters the number once at the start of a sighting session; subsequent marks
within the same session implicitly reference the same landmark until the crew changes it.

---

## 8. Program Alarms

### 8.1 P21 alarms

| Code | Mnemonic | Trigger | Recovery |
|------|----------|---------|---------|
| 01420 | NO_CSM_SV | P21 or P22 entered with `state.csm_state.epoch == 0.0` (uninitialized CSM state vector). | Display alarm on DSKY; return to P00. Do not attempt computation. |
| 01423 | GROUND_TRACK_PROPAGATION_FAIL | `kepler_step` returns an error (e.g. hyperbolic trajectory; state vector is not bound to Earth). | Display alarm; show last valid result or zeros on N43. |

### 8.2 P22 alarms

| Code | Mnemonic | Trigger | Recovery |
|------|----------|---------|---------|
| 01420 | NO_CSM_SV | Same as P21: `csm_state.epoch == 0.0` on entry. | Identical recovery: alarm, return to P00. |
| 01421 | W_OVERFLOW | W-matrix diagonal entry goes negative after a landmark mark update. | Auto-call `p22_rectify_w_matrix`; alarm displayed; navigation cycle continues from re-initialised W. Identical to P20 alarm 01421. |
| 01422 | REJECT_OVERRIDE | Five consecutive landmark marks rejected by 3-sigma gate. | Set `tracking_active = false`; display alarm. Crew must key V32 to rectify W, or re-enter P22. |
| 01424 | BAD_LANDMARK_INDEX | Mark delivered with `landmark_index == 0` or `landmark_index > 8`. | Discard mark; do NOT increment any counter. Display alarm. |
| 01425 | LANDMARK_RANGE_ZERO | `norm(csm_pos - landmark_inertial) < MIN_LANDMARK_RANGE_M`. | Discard mark; do NOT update state. This guard is a safety floor; it should never trigger in practice. |

---

## 9. Edge Cases

| Condition | Affected function(s) | Required behaviour |
|-----------|---------------------|--------------------|
| (a) `target_get_s < epoch_s` (past GET) | `p21_compute_ground_track` | Allowed: `delta_t < 0`, `kepler_step` propagates backward. Display result; no alarm. |
| (b) `target_get_s == epoch_s` | `p21_compute_ground_track` | `delta_t == 0`; `kepler_step` returns input unchanged. Valid, no alarm. |
| (c) `|target_get_s - epoch_s| > 86400 s` (more than one day) | `p21_compute_ground_track` | `kepler_step` may accumulate significant error over long propagation arcs. No alarm is raised (the crew accepted the large time offset); the result may be inaccurate due to unmodelled perturbations. An advisory note on the DSKY is not possible (no free register); document as a known limitation. |
| (d) `norm(csm_pos) == 0` | `p21_compute_ground_track` | Panic. Physically impossible in orbit. |
| (e) CSM in polar orbit (`csm_pos[2] == ±csm_pos_magnitude`) | `p21_compute_ground_track` | `asin` is well-defined at ±π/2; `atan2(0, 0)` for `lon` returns 0 per IEEE 754. No special case needed. |
| (f) `tracking_active == false` when mark arrives | `p22_incorporate_landmark_mark` | Silently discard the mark. No counter update. Log at DEBUG level. |
| (g) `landmark_index == 0` or `> 8` | `p22_incorporate_landmark_mark` | Raise alarm 01424; discard mark. |
| (h) W-matrix Δt > 3600 s | `p22_cycle_task` | Cap Δt; call `p22_rectify_w_matrix`. Same as P20 §7 / edge case (g). |
| (i) Multiple marks on the same landmark in rapid succession (< 1 s apart) | `p22_incorporate_landmark_mark` | Each mark is processed independently. Consecutive marks from the same landmark provide redundant (but not independent) constraints; the filter will still accept them. No duplicate-detection logic is required; the 3-sigma gate provides adequate protection. |
| (j) Landmark behind the Earth (below horizon) | Caller responsibility | `p22_incorporate_landmark_mark` does not check visibility; it is the sextant HAL's responsibility to flag invalid sightings before delivering a mark. If a mark does arrive for an invisible landmark the residual will be large and the 3-sigma gate will reject it. |
| (k) SOI transition (ECI ↔ MCI frame change) | `p22_cycle_task`, `p22_incorporate_landmark_mark` | P22 checks `state.csm_state.frame` at the top of each cycle. If the frame is MCI, landmark positions must be expressed in MCI; `landmark_inertial_pos` uses ECI coordinates only. Raise alarm 00400 (frame mismatch) and set `tracking_active = false` if frame is not ECI. Lunar-orbit landmark tracking is out of scope for this phase. |

---

## 10. Test Cases

### TC-P21-1: Circular LEO — same-epoch query returns current position

**Purpose**: Verify that querying with `target_get_s == epoch_s` returns the input
position converted to lat/lon/alt without propagation.

**Input**:
```
csm_pos   = [6_671_000.0, 0.0, 0.0]  m  (equatorial, ECI)
csm_vel   = [0.0, 7726.0, 0.0]        m/s
epoch_s   = 1000.0                     s
target_get_s = 1000.0                  s   (same epoch)
gha_epoch_rad = 0.0                   rad (Greenwich meridian on ECI X-axis at GET=0)
```

**Expected**:
- `delta_t = 0.0` → no propagation.
- `gha = OMEGA_EARTH * 1000.0 ≈ 0.07292` rad.
- `pos_ef[0] = 6_671_000 * cos(0.07292) ≈ 6_652_590` m.
- `pos_ef[1] = -6_671_000 * sin(0.07292) ≈ -486_230` m.  (negative: Earth has rotated eastward)
- `pos_ef[2] = 0.0`.
- `lat_rad ≈ 0.0` (equatorial).
- `lon_rad ≈ atan2(-486_230, 6_652_590) ≈ -0.07292` rad (≈ -4.18°).
- `alt_m ≈ 6_671_000 - 6_371_000 = 300_000` m (300 km).

**Tolerance**: lat < 1e-6 rad, lon < 1e-4 rad (dominated by GHA precision), alt < 10 m.

---

### TC-P21-2: Quarter-orbit propagation — equatorial orbit

**Purpose**: Verify that a 90° propagation correctly advances longitude by one quarter orbit.

**Input**:
```
csm_pos   = [6_671_000.0, 0.0, 0.0]   m  (equatorial, ECI; 300 km altitude)
csm_vel   = [0.0, 7726.0, 0.0]         m/s
epoch_s   = 0.0                         s
gha_epoch_rad = 0.0                    rad
```

Circular orbit period: `T = 2π * sqrt(r³/μ)` where `r = 6_671_000` m, `μ = 3.986e14`.
`T ≈ 5428.8 s`. Quarter period: `t_quarter ≈ 1357.2 s`.

At `target_get_s = t_quarter`:
- The CSM has advanced 90° in orbit (inertial position now along +Y ECI).
- Earth has rotated by `OMEGA_EARTH * 1357.2 ≈ 0.09904` rad ≈ 5.67° east.

**Expected** (approximate):
- `pos_t ≈ [0.0, 6_671_000.0, 0.0]` m (along +Y ECI after quarter orbit).
- `gha ≈ 0.09904` rad.
- After Earth-rotation: `pos_ef[0] ≈ 6_671_000 * sin(0.09904) ≈ 659_900` m,
  `pos_ef[1] ≈ 6_671_000 * cos(0.09904) ≈ 6_638_200` m.
- `lat ≈ 0` (equatorial orbit stays at equator).
- `alt ≈ 300_000` m.
- `lon_rad ≈ atan2(6_638_200, 659_900) ≈ 1.471` rad ≈ 84.3°E.

**Tolerance**: lat < 0.001 rad, alt error < 1000 m (Kepler propagation accuracy), lon < 0.01 rad.

---

### TC-P21-3: High-inclination orbit — non-zero latitude at sub-satellite point

**Purpose**: Verify latitude computation for an inclined orbit.

**Input** (ISS-like inclination 51.6°):
```
inc = 51.6° = 0.9006 rad
r   = 6_771_000 m  (400 km altitude)
```

State vector at RAAN = 0 (ascending node on +X axis), at the 90° argument of latitude
(spacecraft at northernmost point of ground track):

```
csm_pos   = [0.0, 0.0, 6_771_000.0]  m  (directly over north pole of orbital plane)
csm_vel   = [-v_circ * cos(inc), v_circ * sin(inc) * something, ...]
```

Simpler formulation: at argument of latitude u = 90°, height of the orbit above the
equatorial plane equals `r * sin(inc)`:

```
csm_pos[2] ≈ r * sin(inc) = 6_771_000 * sin(0.9006) ≈ 5_299_000 m
csm_pos[0..1] set so norm(csm_pos) = r (exact values not critical for this test).
```

**Expected**:
- `lat_rad ≈ asin(5_299_000 / 6_771_000) = asin(0.7827) ≈ 0.8967 rad ≈ 51.37°`.
- Tolerance: < 0.01 rad (propagation step is zero for same-epoch query).

---

### TC-P22-1: Init — W-matrix correctly initialised

**Purpose**: Verify that `p22_init` sets the CSM W-matrix to the default diagonal.

**Input**: Valid `state` with `csm_state.epoch = 1000.0 s`, ECI frame.

**Action**: Call `p22_init(&mut state)`.

**Expected**:
- `state.csm_nav.w_matrix[0][0] == CSM_W_INIT_POS_VARIANCE` (250_000.0 m²).
- `state.csm_nav.w_matrix[3][3] == CSM_W_INIT_VEL_VARIANCE` (1.0 m²/s²).
- All off-diagonal elements == 0.0.
- `state.csm_nav.tracking_active == true`.
- `state.csm_nav.mark_count == 0`.
- `state.major_mode == 22`.
- No alarm raised.

---

### TC-P22-2: `landmark_inertial_pos` round-trip consistency with P21 Earth-fixed conversion

**Purpose**: Verify that `landmark_inertial_pos` and the P21 `Rz(+gha)` step are
mathematical inverses.

**Setup**:
```
entry = LandmarkEntry { lat_rad: 0.523_6 rad (30°N), lon_rad: 0.0 rad, alt_m: 0.0 }
get_s = 500.0 s
gha_epoch_rad = 0.0
```

**Expected**:

1. Compute `r_inertial = landmark_inertial_pos(&entry, 500.0, 0.0)`.
2. Apply the P21 Earth-rotation step (Step 3 in §6.1) with `gha = OMEGA_EARTH * 500.0`:
   compute `pos_ef` from `r_inertial` by rotating by `+gha`.
3. Extract lat/lon from `pos_ef`.

The recovered `lat_rad` must equal `entry.lat_rad` to within 1e-9 rad, and `lon_rad`
must equal `entry.lon_rad` to within 1e-9 rad. Round-trip error is due only to floating-point
rounding.

---

### TC-P22-3: Single perfect landmark mark reduces W-matrix uncertainty

**Purpose**: Verify that one accepted landmark mark reduces the relevant W-matrix
diagonal element.

**Setup**:
```
csm_pos_true = [7_000_000.0, 0.0, 0.0]   m  (ECI, 629 km altitude on X-axis)
csm_vel      = [0.0, 7500.0, 0.0]         m/s

Landmark at nadir (directly below CSM at this epoch):
  lat = 0°, lon = 0°, alt = 0
  → r_ef = [6_371_000, 0, 0]
  → landmark_inertial (at GET=0 with gha_epoch=0) = [6_371_000, 0, 0]

LOS from landmark to CSM: unit([7_000_000 - 6_371_000, 0, 0]) = [1, 0, 0]
→ los_inertial = [1.0, 0.0, 0.0]
→ component = LosComponent::X  (largest component)
```

Mark delivered with exact predicted LOS (zero residual — perfect measurement).

**Expected after one mark**:
- `state.csm_nav.w_matrix[0][0]` decreases from `CSM_W_INIT_POS_VARIANCE` (250_000 m²).
  Exact new value (by Kalman formula, b = [1/(629e3), 0, 0, 0, 0, 0], S = b^T W b + σ²):
  `b[0] = 1/629_000`; `W_00 = 250_000`; `b^T W b = 250_000 / (629_000)^2 ≈ 6.32e-7`;
  `S ≈ 6.32e-7 + 1e-8 ≈ 6.42e-7`; `k[0] = W_00 * b[0] / S = 250_000 / (629_000 * 6.42e-7) ≈ 618`;
  `W_00_new = 250_000 - k[0]^2 * S ≈ 250_000 - 618^2 * 6.42e-7 ≈ 249_999.75` m².
  (Improvement is small because `b[0]` is tiny — the slant range is 629 km.)
- `state.csm_nav.mark_count == 1`.
- `state.csm_nav.reject_count == 0`.
- `state.csm_state.pos` changes by `k[0] * residual`; with zero residual, **pos is unchanged**.

---

### TC-P22-4: State is updated by a mark with non-zero residual

**Purpose**: Verify the state-update branch (non-zero residual pulls state toward truth).

**Setup**: Same geometry as TC-P22-3, but introduce a 500 m position error in the
CSM stored state:
```
state.csm_state.pos = [7_000_500.0, 0.0, 0.0]   m  (500 m error along X)
```

LOS observation uses the **true** CSM position:
```
los_inertial = unit([7_000_000 - 6_371_000, 0, 0]) = [1.0, 0.0, 0.0]
component = LosComponent::X
```

Predicted LOS from stored (wrong) state:
```
rho_vec = [7_000_500 - 6_371_000, 0, 0] = [629_500, 0, 0]
los_hat_predicted = [1.0, 0.0, 0.0]   (unit vector unchanged — still pointing +X)
z_predicted = 1.0
z_observed  = 1.0
residual = 0.0
```

Note: In this exact geometry the X-component of a pure-radial measurement cannot distinguish
a radial position error; the residual is zero regardless of radial displacement. This is
expected — a single range mark has no elevation/azimuth information.

**Revised setup** to demonstrate non-zero residual: Offset CSM by 500 m in Y:
```
state.csm_state.pos = [7_000_000.0, 500.0, 0.0]   m
```

True LOS (from landmark [6_371_000, 0, 0]):
```
rho_true = [629_000, 0, 0]
los_true = [1.0, 0.0, 0.0]
```

Predicted LOS (from stored state):
```
rho_pred = [629_000, 500, 0]
los_hat_pred = unit([629_000, 500, 0]) ≈ [0.999999683, 7.948e-4, 0]
z_predicted (X-component) ≈ 0.999999683
z_observed  = 1.0  (from true direction)
residual = 1.0 - 0.999999683 = 3.17e-7
```

**Expected**:
- `residual ≈ 3.17e-7` (positive; stored state offset causes positive residual).
- State update moves `csm_state.pos[1]` from 500.0 toward 0.0 by `k[1] * residual`.
- `mark_count == 1`, `reject_count == 0`, no alarm.

(The exact correction is small for a single mark. Convergence requires multiple marks
from diverse azimuths; this TC just verifies the sign and direction are correct.)

---

### TC-P22-5: Outlier landmark mark is rejected

**Purpose**: Verify 3-sigma gate rejects a wildly wrong mark.

**Setup**: Use TC-P22-3 geometry (no position error). Deliver a mark with component `X`
but `los_inertial = [0.5, 0.866, 0.0]` (a 60° error — should be [1,0,0]).

```
z_observed  = 0.5
z_predicted = 1.0   (from the near-nadir geometry)
residual = 0.5 - 1.0 = -0.5   (enormous relative to the expected noise of ~0.1 mrad)
```

The innovation variance `S ≈ 6.42e-7` (from TC-P22-3); `3*sqrt(S) ≈ 2.4e-3`. Since
`|residual| = 0.5 >> 2.4e-3`, the mark is rejected.

**Expected**:
- `state.csm_nav.reject_count == 1`.
- `state.csm_nav.consecutive_reject_count == 1`.
- `state.csm_state.pos` unchanged.
- `state.csm_nav.w_matrix` unchanged.

---

### TC-P22-6: Five consecutive rejects raise alarm 01422

**Setup**: Fresh `CsmNavState` (default diagonal W). Deliver five consecutive landmark marks
each with `los_inertial = [0.0, 1.0, 0.0]` while the predicted LOS is `[1.0, 0.0, 0.0]`
(90° error — always rejected).

**Expected after fifth mark**:
- `consecutive_reject_count == 5`.
- `reject_count == 5`.
- Alarm `01422` present in `state.alarm`.
- `tracking_active == false`.

---

### TC-P22-7: `p22_rectify_w_matrix` resets counters

**Action**: Call `p22_rectify_w_matrix` after TC-P22-6 state.

**Expected**:
- `w_matrix` is the default diagonal (pos: 250_000 m², vel: 1.0 m²/s²).
- `mark_count == 0`, `reject_count == 0`, `consecutive_reject_count == 0`.
- `last_mark_time == state.time`.
- `tracking_active` is NOT changed by `p22_rectify_w_matrix` (the caller must
  re-enable tracking explicitly; only the crew or P22's alarm handler sets this flag).

---

### TC-P22-8: Process-noise growth in `p22_cycle_task`

**Purpose**: Verify diagonal W-matrix growth between cycles.

**Setup**:
- `state.csm_nav.last_mark_time = 1000.0 s`.
- `state.time = 1002.0 s` (2 s elapsed — one P22 cycle).
- `w_matrix[0][0] = 250_000.0 m²` (initial value).

**Action**: Call `p22_cycle_task(&mut state)`.

**Expected**:
- `w_matrix[0][0] == 250_000.0 + CSM_Q_POS * 2.0 == 250_001.0` m².
- `w_matrix[3][3] == 1.0 + CSM_Q_VEL * 2.0 == 1.000_002` m²/s².
- Off-diagonal elements unchanged (still 0.0).
- Waitlist entry re-scheduled for `P22_CYCLE_CS = 200` cs.

---

## 11. Open Questions for Architect Review

1. **`scalar_measurement_update` placement**: P22 reuses the same scalar Kalman update as
   P20. Currently this helper is `pub(crate)` in `programs/p20.rs`. The architect should
   decide whether to keep it there (P22 imports it via `use crate::programs::p20::scalar_measurement_update`)
   or to promote it to a shared module (e.g. `navigation::kalman::scalar_measurement_update`)
   accessible to both P20 and P22 without a cross-program dependency. The shared-module
   approach is architecturally cleaner; the cross-import approach is simpler. Either works
   for Phase 3 correctness.

2. **`gha_epoch_rad` placement in `AgcState`**: The spec proposes adding `gha_epoch_rad`
   to an existing `navigation` sub-struct or as a top-level field. The architect should
   confirm where this belongs given the current `AgcState` structure. If a dedicated
   `NavigationParameters` or `Constants` sub-struct does not yet exist, this field and
   `R_EARTH`, `OMEGA_EARTH` may warrant one.

3. **`state.csm_state` mutability contract**: P22 modifies `state.csm_state.pos` and
   `state.csm_state.vel` directly as a result of landmark mark incorporation. The SERVICER
   also writes `csm_state` each Average-G cycle. The architect should confirm that these
   two writers cannot race (they should not, as the AGC was single-threaded, but the Rust
   model should make this explicit — e.g. `csm_state` is written only by the SERVICER
   during its ISR and by Kalman-update functions when the SERVICER is not running).

4. **Landmark table uplink**: The current spec fixes the landmark table as a compile-time
   constant. The original AGC allowed partial uplink of landmark coordinates. If uplink
   support is desired in a later phase, the table should be moved to a mutable `AgcState`
   field. The architect should decide whether to reserve this extensibility now (mutable
   table in state) or defer (fixed compile-time array for Phase 3, refactor later).

5. **P22 `tracking_active` re-enable after alarm 01422**: The spec states that `p22_rectify_w_matrix`
   does NOT change `tracking_active`. The crew must manually re-enable tracking after a
   persistent-reject alarm. The question is how: via `V37E 22E` (re-enter P22, which calls
   `p22_init` and re-sets `tracking_active = true`) or via a dedicated DSKY verb not yet
   defined. The architect should confirm the intended re-enable path and whether a separate
   DSKY handler is needed.
