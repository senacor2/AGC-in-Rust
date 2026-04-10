# Specification: `programs/p20` — Rendezvous Navigation

**Status**: Ready for implementation (Milestone 5 Phase 2)
**Module path**: `agc-core/src/programs/p20.rs`
**Split from**: `agc-core/src/programs/p20_p22.rs` stub (the existing file contains only
  `todo!()` stubs; P20 is split into its own file; P21 and P22 remain in `p20_p22.rs` until
  their own Phase specs are written)
**Architecture reference**: `docs/architecture.md` §7.2 (P20 row), §9 (Navigation Math)
**Rendezvous primitives reference**: `specs/rendezvous-spec.md` — all frame conventions
  (LVLH, range, range_rate, los_angles_lvlh) are adopted unchanged from Phase 1
**State-vector reference**: `specs/state-vector-spec.md` §2.5 (CSM/target state vector pair),
  §2.6 (W-matrix background — note: the AGC did NOT maintain a full W-matrix in P20; see §3)
**Executive reference**: `specs/executive-spec.md` §2.1 (jobs), §2.2 (waitlist / periodic hooks),
  §2.3 (restart groups)
**AGC source files** (GitHub: `chrislgarry/Apollo-11`, Comanche055 directory):
- `Comanche055/P20-P25.agc` — P20 main entry sequence, rendezvous nav cycle
- `Comanche055/R22-R32.agc` — R22 rendezvous radar interface; R32 range/range-rate display
- `Comanche055/MEASUREMENT_INCORPORATION.agc` — scalar Kalman-style mark incorporation
- `Comanche055/W_MATRIX_RECTIFICATION.agc` — W-matrix re-initialisation on crew command
- `Comanche055/ERASABLE_ASSIGNMENTS.agc` — `RONE`/`VONE` (target state, B+28/B+7),
  `RELVEC`/`VELVEC` (relative state, B+28/B+7), `WM` (W-matrix words, B+28 scaled),
  `MARKCOUNT`, `REJECTCNT`, `LASTMARK` (epoch), `TRACKFLAG`
**O'Brien reference**: Frank O'Brien, *The Apollo Guidance Computer — Architecture and
  Operation*, Springer-Praxis 2010.
- Chapter 11, "Rendezvous and Navigation Programs" pp. 303–340 (full P20 treatment)
- pp. 310–315: P20 program role, crew invocation, major-mode assignment
- pp. 315–318: range and range-rate computation (R32/R35); `RDOT`/`RNG` scale factors
- pp. 318–325: measurement incorporation algorithm (scalar update form); W-matrix structure
- pp. 325–327: W-matrix rectification (crew-initiated); process-noise growth model
- pp. 327–333: display routines R32–R35; noun/verb assignments; DSKY scaling
- pp. 333–336: program alarms 01421, 00404; 3-sigma reject logic
- pp. 337–340: restart group for P20; phase-register semantics

---

## 1. Purpose and Program Role

P20 is the **Rendezvous Navigation** program. It runs as a continuously active background
job during the rendezvous phase of the mission, maintaining the onboard estimate of the
target vehicle's (LM's) inertial state vector and the CSM-to-target relative state. It is
the primary sensor fusion loop during the approach and station-keeping phases.

### Mission sequence context

P20 is invoked after transposition and docking separation (when the CSM begins tracking the
LM during the rendezvous approach). It runs in parallel with the targeting programs
(P31–P34) and the executive thrusting programs (P40/P41). It continues until the crew
selects another major mode or until docking is complete.

P20 does three things every cycle:
1. **Propagates** the stored target state vector forward in time using the conic-section
   method (`navigation::conics`) to keep it current.
2. **Incorporates** any new measurement marks received from the rendezvous radar (R22) or
   the sextant optics, updating both the target state and the W-matrix via a scalar
   measurement update.
3. **Updates DSKY displays** (V16 N54, V06 N49, V06 N45) so the crew can monitor tracking
   quality and relative geometry.

### Crew invocation

The crew selects P20 by keying `V37E 20E` on the DSKY (Verb 37 = select program,
Noun = 20). This triggers the program dispatch table entry for major mode 20. The program
can also be re-entered after a restart if the restart phase register is set (see §10).

**Preconditions on entry**:
- `state.target` must be a non-zero `StateVector` (a target state uplinked by Mission
  Control or set by a prior P31/P34 computation). If `state.target` is `StateVector::ZERO`,
  P20 raises alarm 00404 (no radar data valid) and requests a crew go-ahead before starting
  the update loop.
- `state.csm` must be a valid, up-to-date own-vehicle state (maintained by the SERVICER).
- The rendezvous radar (R22) must be powered and tracking; if not, P20 still initialises but
  marks `tracking_active = false` and only updates the display from the propagated state.

### Sub-routines called

| Routine | Source file | Purpose |
|---------|------------|---------|
| `p20_rendezvous_nav_cycle` | `programs/p20.rs` | Periodic update: propagate target SV, update displays |
| `p20_incorporate_radar_mark` | `programs/p20.rs` | Process one radar range/range-rate mark |
| `p20_incorporate_sextant_mark` | `programs/p20.rs` | Process one sextant LOS mark |
| `p20_rectify_w_matrix` | `programs/p20.rs` | Re-initialise W on crew command |
| `kepler_step` | `math::kepler` | Conic propagation of target SV |
| `relative_state_lvlh` | `guidance::rendezvous` | Compute LVLH relative state for display |
| `range`, `range_rate` | `guidance::rendezvous` | Scalar observables from state |
| `los_angles_lvlh` | `guidance::rendezvous` | LOS angles for sextant measurement model |

---

## 2. Module Path

`agc-core/src/programs/p20.rs`

The `p20_p22.rs` stub file is modified to re-export only `init_p21` and `init_p22`. The
`init_p20` entry point moves here and is registered at `PROGRAM_TABLE[20]` in
`programs/mod.rs`.

---

## 3. State Additions

### 3.1 W-matrix representation: design choice

The original Comanche055 CMC stored a **reduced** W-matrix — an upper-triangular,
scaled, 6×6 symmetric matrix packed as 21 double-precision words in erasable bank E3
(symbol `WM`, `ERASABLE_ASSIGNMENTS.agc`). The scaling was B+28 for position-related
rows and B+7 for velocity-related rows to match the fixed-point state-vector scales.

For the Rust port we adopt a **full symmetric `f64` 6×6 matrix** stored as `[[f64; 6]; 6]`.
This choice:
- Eliminates the mixed-scale fixed-point book-keeping that was purely a hardware artefact.
- Keeps the measurement-update algorithm numerically readable (see §6).
- Has negligible memory cost on the target Cortex-M7 (288 bytes vs. 168 bytes for
  upper-triangular), which has 512 KB SRAM.

The architect should note this deviation from the original packed representation. An
upper-triangular newtype `SymMat6` could be introduced as a separate optimisation pass
after correctness is established.

### 3.2 `RendezvousNavState` struct

This struct is added as a field of `AgcState`:

```rust
/// Navigation state maintained by P20 (Rendezvous Navigation).
///
/// Lives inside `AgcState` so the Executive, SERVICER, and restart handler
/// can all read/write it without passing extra arguments.
#[derive(Clone, Debug)]
pub struct RendezvousNavState {
    /// Estimated inertial position of the target vehicle (m).
    /// Frame must match `AgcState::csm.frame` (ECI or MCI).
    /// Corresponds to AGC erasable `RONE` (scale B+28 m).
    pub target_pos: Vec3,

    /// Estimated inertial velocity of the target vehicle (m/s).
    /// Corresponds to AGC erasable `VONE` (scale B+7 m/s).
    pub target_vel: Vec3,

    /// Epoch of the current target state estimate (Mission Elapsed Time, seconds).
    /// Corresponds to AGC erasable `TIMET` (time-tagged to the last state update).
    pub target_epoch: f64,

    /// 6×6 state-error covariance (W-matrix), in SI units.
    /// Rows/columns 0..2 are position components (m²).
    /// Rows/columns 3..5 are velocity components (m²/s²).
    /// The matrix is always symmetric; only the upper triangle is written,
    /// but the full array is stored for algorithmic clarity.
    /// Corresponds to AGC erasable `WM` (21 DP words, mixed B+28/B+7 scale).
    pub w_matrix: [[f64; 6]; 6],

    /// Mission Elapsed Time of the last accepted measurement mark (s).
    /// Used for covariance growth computation between marks.
    /// Corresponds to AGC erasable `LASTMARK`.
    pub last_mark_time: f64,

    /// Count of accepted measurement marks since P20 was initialised.
    /// Displayed via V06 N45 (see §8).
    /// Corresponds to AGC erasable `MARKCOUNT`.
    pub mark_count: u16,

    /// Count of measurement marks rejected by the 3-sigma gate since P20 start.
    /// Displayed via V06 N49.
    /// Corresponds to AGC erasable `REJECTCNT`.
    pub reject_count: u16,

    /// Most recently computed relative state in the rendezvous LVLH frame.
    /// Derived from `target_pos`/`target_vel` and `AgcState::csm`; recomputed
    /// each nav cycle. Displayed via V16 N54 (range, range-rate, LOS angle).
    /// Type defined in `guidance::rendezvous` (Phase 1).
    pub lvlh_state: LvlhState,

    /// True when P20 is actively tracking and incorporating marks.
    /// Set false if the radar loses lock or the crew selects REJECT OVERRIDE.
    /// Corresponds to AGC bit flag `TRACKFLAG`.
    pub tracking_active: bool,
}
```

### 3.3 Default / zero initialisation

```rust
impl Default for RendezvousNavState {
    fn default() -> Self {
        Self {
            target_pos:      Vec3::ZERO,
            target_vel:      Vec3::ZERO,
            target_epoch:    0.0,
            w_matrix:        [[0.0; 6]; 6],
            last_mark_time:  0.0,
            mark_count:      0,
            reject_count:    0,
            lvlh_state:      LvlhState { rho: Vec3::ZERO, rho_dot: Vec3::ZERO },
            tracking_active: false,
        }
    }
}
```

### 3.4 `AgcState` extension

The following field is added to `AgcState` (in `agc-core/src/lib.rs`):

```rust
pub rendezvous_nav: RendezvousNavState,
```

---

## 4. Public API

### 4.1 `p20_init`

```rust
/// Entry point for P20 (Rendezvous Navigation).  Registered in PROGRAM_TABLE[20].
///
/// Sets major_mode = 20, validates preconditions, initialises `RendezvousNavState`
/// from the current `state.target` uplinked state vector, and installs the
/// periodic nav-cycle hook.
///
/// # Returns
/// `P20_PRIORITY` if initialisation succeeds.
/// Raises a program alarm and returns `P20_PRIORITY` on failure (the job still
/// exists so the crew can take corrective action without a full program abort).
///
/// # Preconditions
/// - `state.target` must have a non-zero epoch; otherwise alarm 00404 is raised.
/// - `state.csm.frame == state.target.frame`; otherwise alarm 00400 is raised.
///
/// # Post-conditions
/// - `state.major_mode == 20`
/// - `state.dsky.prog == 20`
/// - `state.rendezvous_nav.tracking_active == true` (if preconditions met)
/// - `state.servicer_exit` is set to `p20_rendezvous_nav_cycle`
pub fn p20_init(state: &mut AgcState) -> JobPriority
```

**Priority**: `P20_PRIORITY: JobPriority = 8` — lower than the DAP (priority 37) and the
SERVICER/Average-G (priority 20), but higher than background targeting jobs. The nav cycle
runs as a servicer-exit hook, not as an independent job.

### 4.2 `p20_rendezvous_nav_cycle`

```rust
/// Periodic rendezvous navigation update.  Called by the SERVICER exit hook
/// (approximately every 2 seconds via `servicer_exit`).
///
/// Steps performed each cycle (see §6 for the state-update math):
/// 1. Apply covariance growth (process noise) proportional to elapsed time
///    since `last_mark_time` (§7).
/// 2. Propagate `target_pos`/`target_vel` forward to `state.time` using
///    `math::kepler::kepler_step`.  Update `target_epoch`.
/// 3. Recompute `lvlh_state` from propagated target SV and current CSM SV.
/// 4. Update DSKY display registers (V16 N54; see §8).
///
/// # Invariants
/// - Does not modify `w_matrix` beyond the process-noise growth step.
/// - Does not reject or incorporate marks (those go through the mark handlers).
/// - If `tracking_active == false`, steps 1–4 still execute (the display updates
///   from the propagated state).  Only mark incorporation is suppressed.
pub fn p20_rendezvous_nav_cycle(state: &mut AgcState)
```

### 4.3 `p20_incorporate_radar_mark`

```rust
/// Incorporate one rendezvous radar measurement mark into the navigation solution.
///
/// Called from the R22 radar data handler when a valid radar frame arrives.
/// Performs the scalar Kalman update for a range or range-rate observation (§6).
/// If the residual exceeds the 3-sigma gate the mark is counted in
/// `reject_count` but state and W are not modified.
///
/// # Arguments
/// - `mark`: decoded radar observation (see §5.1).
///
/// # Post-conditions (on acceptance)
/// - `state.rendezvous_nav.target_pos` and `target_vel` updated.
/// - `state.rendezvous_nav.w_matrix` rank-1 updated.
/// - `state.rendezvous_nav.mark_count` incremented.
/// - `state.rendezvous_nav.last_mark_time` updated to `mark.time`.
///
/// # Post-conditions (on rejection)
/// - `state.rendezvous_nav.reject_count` incremented.
/// - No other fields modified.
///
/// # Alarms
/// - 01421: W-matrix diagonal entry goes negative after update (overflow/degeneracy).
pub fn p20_incorporate_radar_mark(state: &mut AgcState, mark: RadarMark)
```

### 4.4 `p20_incorporate_sextant_mark`

```rust
/// Incorporate one sextant line-of-sight mark into the navigation solution.
///
/// The measurement model for a sextant mark is a single scalar component of the
/// LOS unit vector (either elevation or azimuth angle).  Two successive sextant
/// marks (one elevation, one azimuth) together update two state components.
/// Each mark is processed independently as a scalar update.
///
/// # Arguments
/// - `mark`: decoded sextant observation (see §5.2).
///
/// # Post-conditions and rejection semantics
/// Same as `p20_incorporate_radar_mark`.
pub fn p20_incorporate_sextant_mark(state: &mut AgcState, mark: SextantMark)
```

### 4.5 `p20_rectify_w_matrix`

```rust
/// Re-initialise the W-matrix to the default diagonal (large uncertainty).
///
/// Called when the crew keys V32E (reject last mark and reinitialise W) or
/// when P20 is started with an uplinked state whose quality is unknown.
/// Sets the W-matrix diagonal to the initial uncertainty values
/// `W_INIT_POS_VARIANCE` (position, m²) and `W_INIT_VEL_VARIANCE` (velocity,
/// m²/s²) and zeros all off-diagonal elements.
/// Resets `mark_count = 0` and `reject_count = 0`.
///
/// # Post-conditions
/// - `state.rendezvous_nav.w_matrix` is diagonal with `W_INIT_POS_VARIANCE`
///   on rows 0–2 and `W_INIT_VEL_VARIANCE` on rows 3–5.
/// - `mark_count == 0`, `reject_count == 0`.
/// - `last_mark_time` is set to `state.time` (prevents stale process-noise growth).
///
/// # AGC correspondence
/// Comanche055 `W_MATRIX_RECTIFICATION.agc`: when the crew keyed V32 the program
/// reinitialised `WM` to a diagonal matrix with fixed initial variances and reset
/// `MARKCOUNT` (O'Brien pp. 325–327).
pub fn p20_rectify_w_matrix(state: &mut AgcState)
```

### 4.6 Constants

```rust
pub const P20_MAJOR_MODE:        u8          = 20;
pub const P20_PRIORITY:          JobPriority = 8;

/// Initial position variance on W-matrix diagonal (m²).
/// Corresponds to roughly ±500 m (1-sigma) positional uncertainty at P20 start,
/// consistent with the uplinked target-SV accuracy after ground tracking.
/// O'Brien p. 319: initial W diagonal set to "the expected uplink accuracy."
pub const W_INIT_POS_VARIANCE:   f64 = 250_000.0;   // (500 m)²

/// Initial velocity variance on W-matrix diagonal (m²/s²).
/// Corresponds to roughly ±1 m/s (1-sigma) velocity uncertainty.
pub const W_INIT_VEL_VARIANCE:   f64 = 1.0;          // (1 m/s)²

/// Process-noise rate for position (m²/s).
/// Accumulates in W_pos as:  ΔW_pos = Q_POS * Δt
/// Value calibrated so that after 1 orbital period without marks the
/// position uncertainty grows by ~1 km (1-sigma).
pub const Q_POS:                 f64 = 0.5;           // m²/s

/// Process-noise rate for velocity (m²/s³).
/// ΔW_vel = Q_VEL * Δt
pub const Q_VEL:                 f64 = 1.0e-6;        // m²/s³

/// Radar range measurement noise variance (m²).
/// 1-sigma ~15 m at nominal radar lock; consistent with R22 hardware spec.
pub const SIGMA_RANGE_SQ:        f64 = 225.0;         // (15 m)²

/// Radar range-rate measurement noise variance (m²/s²).
/// 1-sigma ~0.15 m/s; consistent with R22 Doppler noise floor.
pub const SIGMA_RANGE_RATE_SQ:   f64 = 0.0225;        // (0.15 m/s)²

/// Sextant LOS angle noise variance (rad²).
/// 1-sigma ~0.1 mrad (20 arcsec), consistent with CM optics resolution.
pub const SIGMA_SEXTANT_SQ:      f64 = 1.0e-8;        // (0.1 mrad)²

/// Minimum range for radar/sextant mark incorporation (m).
/// Below this threshold a different nav mode is required (terminal phase).
pub const MIN_TRACKING_RANGE_M:  f64 = 50.0;
```

---

## 5. Measurement Types

### 5.1 `RadarMark`

Decoded output of the R22 rendezvous radar interface. The HAL layer (R22 data handler)
delivers one of these per radar data frame to `p20_incorporate_radar_mark`.

```rust
/// A single rendezvous radar measurement mark.
///
/// The R22 rendezvous radar on the CSM antenna measured slant range to the LM
/// transponder and the radial component of relative velocity (Doppler range-rate).
/// The hardware produced one measurement frame approximately every 0.5 seconds
/// when in track mode (O'Brien p. 311).
///
/// Scaling: Range in metres (SI); range-rate in m/s (SI).
/// The AGC stored `RRANGE` at scale B+28 m (same as position) and `RRDOT` at
/// scale B+7 m/s (same as velocity).  In the Rust port both are plain f64.
#[derive(Clone, Copy, Debug)]
pub struct RadarMark {
    /// Mission Elapsed Time of the measurement (s).
    pub time: f64,

    /// Slant range to the target (m).  Always >= 0.
    /// Valid only if `range_valid == true`.
    pub range_m: f64,

    /// Range rate (m/s).  Positive = target moving away.
    /// Sign convention matches `guidance::rendezvous::range_rate`.
    /// Valid only if `range_rate_valid == true`.
    pub range_rate_mps: f64,

    /// True if the range measurement is valid (radar locked, no AGC fault).
    pub range_valid: bool,

    /// True if the range-rate measurement is valid.
    pub range_rate_valid: bool,
}
```

### 5.2 `SextantMark`

Decoded output of the CM sextant optics. The sextant handler converts shaft and trunnion
CDU angles into a body-frame LOS unit vector, which is then rotated to the inertial frame
before being passed to `p20_incorporate_sextant_mark`.

```rust
/// A single sextant line-of-sight mark.
///
/// The CM sextant measured the direction to the target vehicle in the spacecraft
/// body frame.  The optics unit vector is converted to the inertial frame using
/// the current REFSMMAT (stored in `AgcState`) before being delivered here.
/// (O'Brien p. 320: "The sextant marks are reduced using the current REFSMMAT
///  to obtain the LOS unit vector in inertial coordinates.")
///
/// Each mark provides a constraint on the angle between the target-position
/// vector and a reference direction; the scalar measurement is the dot product
/// of the predicted LOS unit vector with the observed LOS unit vector, which
/// equals cos(angle_error).  For small errors this reduces to 1 - ε²/2,
/// and the scalar residual is (observed_dot - predicted_dot) ≈ -ε, where
/// ε is the angular error in radians.
#[derive(Clone, Copy, Debug)]
pub struct SextantMark {
    /// Mission Elapsed Time of the sighting (s).
    pub time: f64,

    /// LOS unit vector from the CSM to the target, in the **inertial frame**
    /// (ECI or MCI as appropriate).  Magnitude must be 1.0 ± 1e-6.
    pub los_inertial: Vec3,

    /// Which scalar component of the LOS is being used as the observation.
    /// The AGC measured one angle per mark; the caller selects the component
    /// with the largest sensitivity (the component of `los_inertial` with the
    /// smallest absolute value), to maximise numerical conditioning.
    pub component: LosComponent,
}

/// Which component of the LOS unit vector is the scalar observable for this mark.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LosComponent {
    /// Dot product with the inertial X-axis.
    X,
    /// Dot product with the inertial Y-axis.
    Y,
    /// Dot product with the inertial Z-axis.
    Z,
}
```

---

## 6. State Update Algorithm

This section documents the scalar Kalman-like measurement incorporation used by the AGC.
The algorithm is a sequential scalar filter, not a vector update. Each mark updates the
6-element state vector and 6×6 W-matrix using a single scalar observation. This matches
`MEASUREMENT_INCORPORATION.agc` (Comanche055) and is described in O'Brien pp. 318–325.

**Theoretical foundation**: Sequential scalar updates are mathematically equivalent to a
full-vector Kalman update when the noise sources are uncorrelated (Gelb 1974, §4.2, Eq. 4.2-14;
Battin 1987, §14.3, Eq. 14.3-5). Each update reduces the covariance by a rank-1 term.

### 6.1 State representation

The 6-element state vector `x` is:

```
x = [ target_pos[0], target_pos[1], target_pos[2],
      target_vel[0], target_vel[1], target_vel[2] ]^T
```

All in SI units (m, m/s). The W-matrix `W` is the associated 6×6 covariance, `W[i][j]`
in units consistent with row/column indices (m² for i,j < 3; m·m/s for mixed; m²/s² for
i,j >= 3).

### 6.2 Step 1: Predict the measurement

Given the current state, compute what the measurement *should* be if the state is correct.

**For a radar range mark**:
```
rho_vec = x[0..3] - csm_pos        (target position minus CSM position, m)
z_predicted = norm(rho_vec)         (predicted range, m)
```

**For a radar range-rate mark**:
```
rho_vec     = x[0..3] - csm_pos    (m)
rho_dot_vec = x[3..6] - csm_vel    (m/s)
z_predicted = dot(rho_vec, rho_dot_vec) / norm(rho_vec)   (m/s)
```

**For a sextant mark (component `c`)**:
```
rho_vec      = x[0..3] - csm_pos    (m)
los_hat      = unit(rho_vec)         (predicted LOS unit vector)
z_predicted  = los_hat[c]            (predicted direction cosine)
```

### 6.3 Step 2: Compute the measurement residual

```
residual = z_observed - z_predicted       (scalar)
```

### 6.4 Step 3: Compute the measurement sensitivity vector `b`

`b` is the 6×1 partial derivative of the scalar measurement with respect to the state
vector `x`. This is the Jacobian row for this measurement (Battin 1987, §14.3, Eq. 14.3-3).

**For range**:
```
b[0..3] = rho_vec / norm(rho_vec)   (= los_hat, unit vector toward target from CSM)
b[3..6] = [0, 0, 0]
```

**For range-rate** (let `rng = norm(rho_vec)`):
```
b[0..3] = rho_dot_vec / rng  -  (dot(rho_vec, rho_dot_vec) / rng^3) * rho_vec
b[3..6] = rho_vec / rng
```
(This is `∂(range_rate)/∂pos` and `∂(range_rate)/∂vel` respectively.)

**For sextant LOS component `c`**:
```
b[0..3] = (e_c - los_hat[c] * los_hat) / norm(rho_vec)
          where e_c is the unit vector along axis c
b[3..6] = [0, 0, 0]
```
(The LOS direction cosine depends only on position, not velocity.)

### 6.5 Step 4: Compute the innovation variance

```
S = b^T * W * b + sigma_sq         (scalar, same units as z²)
```

where `sigma_sq` is the measurement noise variance for this mark type:
- Range: `SIGMA_RANGE_SQ`
- Range-rate: `SIGMA_RANGE_RATE_SQ`
- Sextant: `SIGMA_SEXTANT_SQ`

### 6.6 Step 5: 3-sigma reject gate

If `|residual| > 3 * sqrt(S)`, the mark is a statistical outlier:
- Increment `reject_count`.
- Do NOT update `x` or `W`.
- If `reject_count` reaches 5 consecutive rejects, raise alarm 00405 (persistent tracking
  failure; crew attention required).
- Return early.

This gate is documented in O'Brien p. 323: "marks with residuals greater than 3 standard
deviations of the predicted residual scatter are rejected."

### 6.7 Step 6: Kalman gain vector

```
k = (W * b) / S       (6×1 vector)
```

`W * b` is the matrix-vector product of the 6×6 W with the 6×1 sensitivity vector.

### 6.8 Step 7: State update

```
x_new = x_old + k * residual
```

Expand to component form:
```
target_pos[i] += k[i]   * residual    for i in 0..3
target_vel[i] += k[i+3] * residual    for i in 0..3
```

### 6.9 Step 8: Covariance update (rank-1 downdate)

```
W_new = W_old - k * (b^T * W_old)
```

In component form (the outer product `k * (W_old * b)^T`):
```
W_new[i][j] = W_old[i][j] - k[i] * (W_old * b)[j]     for all i, j in 0..6
```

Note that `W_old * b` has already been computed as the numerator of `k` (scaled by `S`):
`W_old * b = k * S`. So:
```
W_new[i][j] = W_old[i][j] - k[i] * k[j] * S
```

This is the Joseph-free form (Gelb 1974, §4.2, Eq. 4.2-10). The matrix remains symmetric.

### 6.10 Step 9: Positive-definite check

After the update, verify that all diagonal entries of `W_new` are non-negative:
```
for i in 0..6:
    if W_new[i][i] < 0.0:
        raise alarm 01421 (W-matrix overflow / loss of positive definiteness)
        call p20_rectify_w_matrix(state)   // recover to safe diagonal state
        return
```

O'Brien p. 323: "If any diagonal element of W becomes zero or negative, the W-matrix is
re-initialised and `REJECTCNT` is incremented." This port raises alarm 01421 as well.

### 6.11 Step 10: Update mark counters and timestamp

```
mark_count += 1
last_mark_time = mark.time
```

---

## 7. Covariance Propagation Between Marks (Process Noise Growth)

Between marks the target vehicle's actual state diverges from the estimated state due to
unmodelled forces (solar pressure, lunar mascons, CSM jet firings, atmospheric drag in low
orbit). The W-matrix must grow to reflect this growing uncertainty.

The AGC used a diagonal process-noise accumulation (O'Brien p. 325). The Rust port
implements the same model:

```
Δt = state.time - last_mark_time          (seconds since last mark or last cycle)

For i in 0..3:   (position rows)
    W[i][i] += Q_POS * Δt

For i in 3..6:   (velocity rows)
    W[i][i] += Q_VEL * Δt
```

Only the diagonal is grown (additive diagonal process noise, a.k.a. the "kinematic model"
or "random walk" model). Off-diagonal terms are not modified by process noise. This
matches the Comanche055 implementation where the W-matrix growth used a simple scalar
accumulation per time step (Battin 1987, §14.2, Eq. 14.2-7 for the discrete-time form
with a diagonal Q-matrix).

This growth step is called at the top of `p20_rendezvous_nav_cycle` before propagating
the target state. The `Δt` is bounded to a maximum of 3600 s (one hour) to prevent
runaway growth after a long communication outage; if the gap is larger, the W-matrix is
re-initialised via `p20_rectify_w_matrix`.

---

## 8. DSKY Interaction

P20 drives the following DSKY display modes. The notation `V16 N54` means Verb 16
(monitor — continuous update), Noun 54. O'Brien Table 11.1 (pp. 327–333) lists the
complete P20 DSKY assignments.

### 8.1 Continuous monitoring display (V16 N54)

Updated every nav cycle (~2 s). Three registers:

| Register | Content | AGC variable | AGC scale | SI unit | Notes |
|----------|---------|-------------|-----------|---------|-------|
| R1 | Slant range to target | `RNG` | B+28 m (displayed in 0.1 NM steps) | m | Display as metres in Rust port |
| R2 | Range-rate (positive = opening) | `RDOT` | B+7 m/s | m/s | Sign: positive = closing per Comanche055 N54 convention (O'Brien p. 329, Table 11.2 note) |
| R3 | Theta (elevation angle to target in LVLH) | `THETA` | Fraction of π rad | rad | See note below |

**Note on R2 sign**: O'Brien p. 329 states that N54 R2 displays range-rate with
**positive = closing** (opposite to the `range_rate` function convention in
`guidance::rendezvous`, which returns positive = opening). The display driver must
negate the value from `range_rate` before writing it to R2. This is a display-only
inversion; the internal state always uses the `range_rate` sign convention.

**Note on R3 theta**: The angle displayed is the elevation angle from the rendezvous LVLH
local horizontal plane to the LOS, in degrees × 100 (fixed-point display). In the Rust
port, store the elevation in radians in `lvlh_state` and convert to the DSKY integer
representation at display time.

### 8.2 Mark counter display (V06 N45)

Displayed on crew request (`V06E N45E`). Noun 45 is the marks-accepted counter:

| Register | Content | Unit |
|----------|---------|------|
| R1 | `mark_count` | count |
| R2 | Not used (zero) | — |
| R3 | Not used (zero) | — |

### 8.3 Reject counter display (V06 N49)

Displayed on crew request (`V06E N49E`). Noun 49 is the reject counter:

| Register | Content | Unit |
|----------|---------|------|
| R1 | `reject_count` | count |
| R2 | Not used (zero) | — |
| R3 | Not used (zero) | — |

### 8.4 Program entry display

On entry to P20 (`p20_init`), P20 briefly flashes V06 N49 to show the current reject count
before switching to the V16 N54 continuous monitor. This matches the AGC behaviour described
in O'Brien p. 312.

### 8.5 Crew rectification prompt

When the crew keys V32 (mark reject / W rectification request), P20 calls
`p20_rectify_w_matrix` and displays V06 N49 (updated reject count) to confirm the action.

---

## 9. Program Alarms

| Code | Mnemonic | Trigger | Recovery |
|------|----------|---------|---------|
| 01421 | W_OVERFLOW | A diagonal element of `w_matrix` goes negative or NaN after a measurement update. Indicates numerical overflow or a degenerate measurement geometry. | `p20_rectify_w_matrix` is called automatically; alarm displayed on DSKY; nav cycle continues from re-initialised W. O'Brien p. 323. |
| 00404 | NO_RADAR | P20 entered with `state.target == StateVector::ZERO` or radar not providing valid marks for > 60 s. | P20 sets `tracking_active = false`, displays alarm, continues propagating target SV from last known state. Crew must verify radar lock. O'Brien p. 333. |
| 00405 | REJECT_OVERRIDE | Five consecutive marks rejected by 3-sigma gate. Indicates the stored state may be grossly wrong (large bias) rather than just noisy marks. | Alarm displayed on DSKY. `tracking_active` set false. Crew must either rectify W (V32) or re-uplink target SV and restart P20. O'Brien p. 334. |
| 00400 | FRAME_MISMATCH | `state.csm.frame != state.target.frame` on entry. The CSM and target state vectors are in different coordinate frames, making relative-state computation invalid. | Alarm displayed; init aborts without setting `tracking_active = true`. Crew must ensure both state vectors are updated to the same frame (usually triggered by an SOI transition that updated CSM but not target SV). |

---

## 10. Restart Recovery

P20 uses **restart group 2** (arbitrary assignment; consistent with the Comanche055 `P20-P25.agc`
restart group allocation — verify against `ERASABLE_ASSIGNMENTS.agc` during implementation).

### 10.1 State preserved across restart

The following `RendezvousNavState` fields must survive a restart (they must NOT be in
zero-initialised restart-cleared memory; they must be in restart-protected erasable):

| Field | Why preserved |
|-------|--------------|
| `target_pos`, `target_vel` | The last-good estimate; discarding it means restarting nav from scratch |
| `w_matrix` | Accumulated measurement history; discarding forces full W-matrix rectification |
| `last_mark_time` | Needed for correct Δt in process-noise step after restart |
| `mark_count` | Crew display continuity |
| `reject_count` | Crew display continuity |
| `target_epoch` | Required for conic propagation to current time |
| `tracking_active` | Determines whether mark incorporation is enabled after restart |

### 10.2 Phase register semantics for restart group 2

| Phase | Meaning | Restart action |
|-------|---------|---------------|
| 0 | P20 not active | No restart |
| 2 | P20 active, nav cycle running | Re-create nav job at `p20_init` (skips W-matrix reinit; uses saved state) |
| 4 | P20 in mark incorporation | Re-create at `p20_init`; the in-progress mark is discarded (safe: marks arrive frequently enough that one lost mark is inconsequential) |
| -2 | Mid-W-matrix write (PHASCHNG guard) | Re-initialise W via `p20_rectify_w_matrix`; restart from phase 2 |

### 10.3 PHASCHNG pattern

The W-matrix is a multi-word write. The implementation must set phase to -2 before beginning
the W-matrix update and advance it to 2 (or the next phase) only after the write is complete,
following the standard restart-safe coding pattern from `specs/executive-spec.md` §2.3.

---

## 11. Edge Cases

| Condition | Affected function(s) | Required behaviour |
|-----------|---------------------|--------------------|
| (a) Radar not tracking (`range_valid == false`, `range_rate_valid == false`) | `p20_incorporate_radar_mark` | Skip all mark incorporation; do NOT increment `mark_count`. If this persists > 60 s raise alarm 00404 and set `tracking_active = false`. |
| (b) Range < `MIN_TRACKING_RANGE_M` (50 m) | `p20_incorporate_radar_mark`, `p20_rendezvous_nav_cycle` | Terminal-phase proximity; set `tracking_active = false` and display alarm. The measurement model (linear range, LOS angle) becomes unreliable at very short range; the crew should transition to visual guidance. |
| (c) Mark rejected by 3-sigma gate | `p20_incorporate_radar_mark`, `p20_incorporate_sextant_mark` | Increment `reject_count`; do not update state or W. Track consecutive-reject count; raise alarm 00405 after 5 consecutive rejects. Reset consecutive-reject counter on any accepted mark. |
| (d) W-matrix diagonal entry zero or negative | `p20_incorporate_radar_mark`, `p20_incorporate_sextant_mark` | Raise alarm 01421; call `p20_rectify_w_matrix`; return without further update. This prevents subsequent marks from using a degenerate W and producing NaN state values. |
| (e) Frame mismatch (ECI vs MCI during cislunar) | `p20_init`, `p20_rendezvous_nav_cycle` | Raise alarm 00400; set `tracking_active = false`; do not call `guidance::rendezvous` functions. The SOI transition (managed by the integrator) must update both `csm` and `target` state vectors to MCI before P20 can resume. P20 checks `frame` consistency on every nav cycle, not just at init. |
| (f) Zero relative position vector | `p20_rendezvous_nav_cycle` | If `range(csm_pos, target_pos) < 1.0 m` (docking contact), skip the LVLH state update and suppress N54 display (range display would be zero or undefined). Do not call `range_rate` or `los_angles_lvlh` (both panic on zero range per `specs/rendezvous-spec.md` §8). |
| (g) `Δt > 3600 s` in process-noise growth | `p20_rendezvous_nav_cycle` | Cap `Δt` at 3600 s and call `p20_rectify_w_matrix`. Log the condition (DEBUG build) so the tester can detect unexpectedly long gaps between marks during simulation. |
| (h) `target` SV epoch in the future | `p20_rendezvous_nav_cycle` | `kepler_step` with a negative `Δt` is valid (backward propagation) but unusual. No special case; propagate normally. |

---

## 12. Test Cases

### TC-P20-1: Init with valid uplinked target SV

**Purpose**: Verify that `p20_init` correctly initialises state and installs the nav cycle hook.

**Input**:
- `state.csm`: circular LEO at 300 km, ECI frame.
  `r = [6_671_000.0, 0.0, 0.0]` m, `v = [0.0, 7726.0, 0.0]` m/s, epoch = 1000.0 s.
- `state.target`: target in circular LEO 2 km behind (same altitude), ECI frame.
  `r_t = [6_671_000.0, -2000.0, 0.0]` m, `v_t = [0.0, 7726.0, 0.0]` m/s, epoch = 1000.0 s.
- `state.time = 1000.0` s.

**Action**: Call `p20_init(&mut state)`.

**Expected**:
- Return value: `P20_PRIORITY` (== 8).
- `state.major_mode == 20`.
- `state.dsky.prog == 20`.
- `state.rendezvous_nav.tracking_active == true`.
- `state.rendezvous_nav.target_pos ≈ [6_671_000.0, -2000.0, 0.0]` m (tolerance 1 m).
- `state.rendezvous_nav.w_matrix[0][0] == W_INIT_POS_VARIANCE` (250_000.0 m²).
- `state.rendezvous_nav.w_matrix[3][3] == W_INIT_VEL_VARIANCE` (1.0 m²/s²).
- `state.rendezvous_nav.mark_count == 0`.
- `state.servicer_exit` is `Some(p20_rendezvous_nav_cycle)`.
- No program alarm raised.

---

### TC-P20-2: Init with zero target SV raises alarm 00404

**Input**: `state.target == StateVector::ZERO`. All other state valid as in TC-P20-1.

**Action**: Call `p20_init(&mut state)`.

**Expected**:
- `state.rendezvous_nav.tracking_active == false`.
- Program alarm `00404` present in `state.alarm`.
- `state.major_mode == 20` (major mode IS advanced — P20 is selected but not tracking).

---

### TC-P20-3: Happy-path — three radar marks converge the estimate

**Purpose**: Verify that successive marks reduce W-matrix uncertainty and pull state toward
the true target position.

**Setup**:
- CSM at `r_c = [7_000_000.0, 0.0, 0.0]` m, `v_c = [0.0, 7500.0, 0.0]` m/s.
- True target at `r_t_true = [7_000_000.0, 10_000.0, 0.0]` m (10 km ahead in-track).
- Initial estimated target offset by +500 m: `r_t_est = [7_000_000.0, 10_500.0, 0.0]` m.
- `W` diagonal: pos 250_000 m², vel 1.0 m²/s².
- Three radar range marks at t = 1000, 1002, 1004 s with `range_m = 10_000.0` m and zero noise.

**Action**: Call `p20_incorporate_radar_mark` three times.

**Expected after three marks**:
- `target_pos[1]` moves from 10_500 toward 10_000 m (converges toward truth).
- `W[0][0]` (or whichever diagonal element corresponds to the measurement-sensitive direction)
  decreases after each mark (uncertainty reduced).
- `mark_count == 3`.
- `reject_count == 0`.
- No alarms.

**Quantitative check**: After 3 perfect range marks, the position estimate should satisfy
`|target_pos[1] - 10_000.0| < 100.0` m (rough convergence; exact value depends on W and b).

---

### TC-P20-4: Outlier mark is rejected

**Setup**: Same as TC-P20-3 after two accepted marks.

**Action**: Deliver a third mark with `range_m = 50_000.0` m (wildly wrong; residual ≈ 40 km,
far beyond 3-sigma).

**Expected**:
- `state.rendezvous_nav.target_pos` is UNCHANGED from the value after the second mark.
- `state.rendezvous_nav.w_matrix` is UNCHANGED.
- `state.rendezvous_nav.reject_count == 1`.
- `state.rendezvous_nav.mark_count == 2` (unchanged).

---

### TC-P20-5: Five consecutive rejects raise alarm 00405

**Setup**: Fresh `RendezvousNavState` with W diagonal at initial values.
Deliver five consecutive radar marks each with `range_m = 1_000_000.0` m (wildly wrong).

**Expected after fifth mark**:
- `reject_count == 5`.
- Program alarm `00405` present in `state.alarm`.
- `tracking_active == false`.

---

### TC-P20-6: W-matrix rectification

**Setup**: `RendezvousNavState` after TC-P20-3 (W reduced by three marks; mark_count = 3).

**Action**: Call `p20_rectify_w_matrix(&mut state)`.

**Expected**:
- `w_matrix[0][0] == W_INIT_POS_VARIANCE` (250_000.0).
- `w_matrix[3][3] == W_INIT_VEL_VARIANCE` (1.0).
- All off-diagonal entries == 0.0.
- `mark_count == 0`.
- `reject_count == 0`.
- `last_mark_time == state.time`.

---

### TC-P20-7: Restart recovery — state survives

**Purpose**: Verify that the restart-protected fields are not zeroed on restart.

**Setup**: Run TC-P20-3 to convergence (mark_count = 3, reduced W, non-zero target_pos).
Record `target_pos_before = state.rendezvous_nav.target_pos`.

**Action**: Simulate a restart by calling the restart handler for group 2 at phase 2
(`restart_group(state, 2, Phase(2))`), which should re-dispatch `p20_init`.
The `RendezvousNavState` fields must have been copied to restart-protected storage
before the simulated restart.

**Expected after restart**:
- `state.rendezvous_nav.target_pos ≈ target_pos_before` (tolerance 1 m).
- `state.rendezvous_nav.mark_count == 3` (preserved).
- `state.major_mode == 20`.
- `tracking_active == true`.

---

### TC-P20-8: Frame mismatch on nav cycle raises alarm 00400

**Setup**: `p20_init` completes normally (ECI). Then update `state.csm.frame` to
`Frame::MoonInertial` (simulating a mid-flight SOI crossing where only CSM SV was
updated) without updating `state.target.frame`.

**Action**: Call `p20_rendezvous_nav_cycle(&mut state)`.

**Expected**:
- Alarm `00400` raised.
- `tracking_active == false`.
- `lvlh_state` not updated (remains at values from before the cycle call).

---

### TC-P20-9: Process-noise growth increases W between marks

**Setup**: `RendezvousNavState` with W diagonal at initial values.
Set `last_mark_time = 1000.0`, `state.time = 1100.0` (Δt = 100 s).

**Action**: Call `p20_rendezvous_nav_cycle(&mut state)` (which runs the process-noise step).

**Expected**:
- `w_matrix[0][0] ≈ 250_000.0 + Q_POS * 100.0 = 250_050.0` m².
- `w_matrix[3][3] ≈ 1.0 + Q_VEL * 100.0 = 1.0001` m²/s².
- Off-diagonal entries remain zero.

---

## 13. Open Questions for Architect Review

1. **W-matrix representation**: This spec adopts a full symmetric `[[f64; 6]; 6]` (288
   bytes). The original AGC used 21 DP words in a packed upper-triangular form (168 bytes,
   mixed scale). The architect should decide whether `SymMat6` (upper-triangular newtype)
   is worth the implementation complexity for the Cortex-M7 target. Recommendation: start
   with the full matrix for correctness; add `SymMat6` in a later optimisation pass.

2. **Where radar/sextant marks enter**: This spec assumes marks are delivered by HAL-level
   interrupt handlers (R22 data handler, optics handler) that call
   `p20_incorporate_radar_mark` / `p20_incorporate_sextant_mark` directly. The architect
   must define whether this call happens in interrupt context (requiring the mark functions
   to be ISR-safe) or via a shared queue polled by the nav cycle. An ISR-safe bounded queue
   (`heapless::Queue`) is the recommended pattern for `no_std` bare-metal.

3. **`servicer_exit` hook discipline**: This spec installs `p20_rendezvous_nav_cycle` as a
   `servicer_exit` hook. If P40/P41 are simultaneously active they also use `servicer_exit`.
   The `AgcState` currently has a single `servicer_exit: Option<fn(&mut AgcState)>` field.
   If multiple hooks are needed simultaneously, the architect must either (a) introduce a
   small array of hooks (`[Option<fn(&mut AgcState)>; 4]`) or (b) compose hooks via a
   chain. The P40/P41 spec has the same issue; this should be resolved uniformly.

4. **`RendezvousNavState` in `AgcState`**: Adding a new 306-byte struct field to `AgcState`
   (288 bytes for W-matrix + other fields) will increase the total `AgcState` size. Verify
   that the Cortex-M7 BSS/data section budget is still met.

5. **Consecutive-reject counter**: The spec mentions a "5 consecutive rejects" threshold for
   alarm 00405 but the `RendezvousNavState` struct as written only holds a cumulative
   `reject_count`. A separate `consecutive_reject_count: u8` field should be added (and
   reset on any accepted mark). The architect should confirm whether to add this field to
   `RendezvousNavState` or handle it as a local in the mark incorporation function.
