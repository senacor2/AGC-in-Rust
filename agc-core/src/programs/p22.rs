//! P22 — Orbital Navigation (Landmark Tracking).
//!
//! The navigation counterpart of P20 but for the CSM's own state vector (not
//! the target's). Instead of radar marks on the LM, it uses sextant sightings
//! of Earth landmarks at known Earth-fixed positions. Each sighting constrains
//! the CSM's inertial position; after sufficient marks the CSM state vector
//! converges to improved accuracy.
//!
//! The measurement model and scalar Kalman filter algorithm are identical to P20.
//! The difference is that the sensitivity vector `b` is computed with respect to
//! the CSM state, and the result is applied to `state.csm_state` and a separate
//! covariance matrix `CsmNavState.w_matrix`. `RendezvousNavState` (P20) is not
//! modified by P22.
//!
//! P22 is a periodic background program. It installs a Waitlist self-rescheduling
//! hook that grows process noise and refreshes the DSKY display each cycle.
//! Landmark marks arrive asynchronously via `p22_incorporate_landmark_mark`
//! called from the sextant HAL handler.
//!
//! AGC source: Comanche055/P20-P25.agc,
//!             Comanche055/MEASUREMENT_INCORPORATION.agc,
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc (CSMNAVSAV, landmark table)
//! Spec: specs/p21_p22-spec.md §1.2, §3.2–§3.4, §4.2, §5.1, §6.2–§6.3

use crate::executive::job::JobPriority;
use crate::executive::waitlist::ScheduleResult;
use crate::math::linalg::{norm, unit};
use crate::navigation::kalman::UpdateOutcome;
use crate::navigation::state_vector::Frame;
use crate::programs::p20::LosComponent;
use crate::programs::p21::{p21_compute_ground_track, OMEGA_EARTH, R_EARTH};
use crate::types::Vec3;
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P22.
/// Spec: p21_p22-spec.md §4.2
pub const P22_MAJOR_MODE: u8 = 22;

/// Job priority for P22. Same as P20 (both are background navigation loops).
/// Spec: p21_p22-spec.md §4.2
pub const P22_PRIORITY: JobPriority = 8;

/// Waitlist cycle period for P22 (centiseconds). 2-second cycle, identical to P20.
/// Spec: p21_p22-spec.md §4.2
pub const P22_CYCLE_CS: u32 = 200;

// Cast to u16 for Waitlist::schedule which takes u16.
const P22_CYCLE_CS_U16: u16 = P22_CYCLE_CS as u16;

/// Initial position variance on the P22 CSM W-matrix diagonal (m²).
/// Corresponds to ±500 m (1-sigma) positional uncertainty — same magnitude as P20.
/// Spec: p21_p22-spec.md §4.2
pub const CSM_W_INIT_POS_VARIANCE: f64 = 250_000.0; // (500 m)²

/// Initial velocity variance on the P22 CSM W-matrix diagonal (m²/s²).
/// Spec: p21_p22-spec.md §4.2
pub const CSM_W_INIT_VEL_VARIANCE: f64 = 1.0; // (1 m/s)²

/// Process-noise rate for CSM position (m²/s).
/// Same value as P20's Q_POS; the unmodelled-force environment is identical.
/// Spec: p21_p22-spec.md §4.2
pub const CSM_Q_POS: f64 = 0.5; // m²/s

/// Process-noise rate for CSM velocity (m²/s³).
/// Spec: p21_p22-spec.md §4.2
pub const CSM_Q_VEL: f64 = 1.0e-6; // m²/s³

/// Sextant landmark LOS noise variance (rad²).
/// 1-sigma ≈ 0.1 mrad (same as P20 sextant marks).
/// Spec: p21_p22-spec.md §4.2
pub const SIGMA_LANDMARK_SQ: f64 = 1.0e-8; // (0.1 mrad)²

/// Minimum CSM-to-landmark slant range for mark incorporation (m).
/// Safety floor; in practice always > 200 km for orbital altitudes.
/// Spec: p21_p22-spec.md §4.2
pub const MIN_LANDMARK_RANGE_M: f64 = 1_000.0;

/// Maximum Δt for process-noise growth before W-matrix re-initialisation (s).
/// Cap at 3600 s; same as P20.
const MAX_PROCESS_NOISE_DT_S: f64 = 3600.0;

// ── Alarm codes ─────────────────────────────────────────────────────────────────

/// Alarm 01420 (octal): no valid CSM state vector (epoch == 0).
const ALARM_NO_CSM_SV: u16 = 0o01420;

/// Alarm 01421 (octal): W-matrix diagonal entry went negative (loss of positive definiteness).
const ALARM_CSM_W_OVERFLOW: u16 = 0o01421;

/// Alarm 01422 (octal): five consecutive landmark marks rejected by 3-sigma gate.
const ALARM_LANDMARK_REJECT: u16 = 0o01422;

/// Alarm 01424 (octal): landmark index out of range (0 or > 8).
const ALARM_BAD_LANDMARK_INDEX: u16 = 0o01424;

/// Alarm 01425 (octal): CSM-to-landmark range below MIN_LANDMARK_RANGE_M.
const ALARM_LANDMARK_RANGE_ZERO: u16 = 0o01425;

/// Alarm 00400 (octal): CSM state vector is not in ECI frame (frame mismatch).
const ALARM_FRAME_MISMATCH: u16 = 0o00400;

/// Alarm 1211: Waitlist full (standard AGC waitlist-overflow alarm).
const ALARM_WAITLIST_FULL: u16 = 1211;

// ── Navigation state ───────────────────────────────────────────────────────────

/// Navigation state maintained by P22 (Orbital Navigation / Landmark Tracking).
///
/// Holds the CSM state-error covariance and tracking bookkeeping.
/// The CSM's best-estimate position and velocity are in `AgcState::csm_state`
/// (a `StateVector`); this struct holds only the uncertainty model and counters.
///
/// Lives inside `AgcState` so the Executive, SERVICER, and restart handler
/// can all reach it without extra arguments.
///
/// Analogous to `RendezvousNavState` (P20) but applies to the CSM itself.
///
/// Spec: p21_p22-spec.md §3.2
#[derive(Clone, Debug)]
pub struct CsmNavState {
    /// 6×6 state-error covariance (W-matrix) for the CSM's own state, in SI units.
    /// Rows/columns 0..2 are position components (m²).
    /// Rows/columns 3..5 are velocity components (m²/s²).
    /// The matrix is always symmetric; the full array is stored for clarity.
    /// Analogous to `RendezvousNavState::w_matrix` (P20), but applies to the CSM.
    pub w_matrix: [[f64; 6]; 6],

    /// GET of the last accepted landmark mark (s).
    /// Used to compute Δt for process-noise growth in `p22_cycle_task`.
    pub last_mark_time: f64,

    /// Count of accepted landmark marks since P22 was initialised.
    /// Displayed on DSKY via V06 N45.
    pub mark_count: u16,

    /// Count of landmark marks rejected by the 3-sigma gate since P22 start.
    /// Displayed on DSKY via V06 N49.
    pub reject_count: u16,

    /// Count of consecutive rejected marks (reset on any accepted mark).
    /// Raises alarm 01422 when it reaches 5 (persistent landmark tracking failure).
    pub consecutive_reject_count: u16,

    /// True when P22 is active and incorporating marks.
    /// Set to false on alarm 01422 (persistent rejection) or crew command.
    pub tracking_active: bool,
}

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

// ── Measurement types ──────────────────────────────────────────────────────────

/// A single sextant sighting of an Earth landmark, decoded by the sextant HAL handler.
///
/// The sextant handler converts shaft and trunnion CDU angles into a body-frame LOS
/// unit vector; the IMU REFSMMAT then rotates this to the inertial frame. The landmark's
/// Earth-fixed coordinates are looked up from `LANDMARK_TABLE` using `landmark_index`,
/// and `landmark_inertial_pos` is called to compute the landmark's inertial position at
/// `time`. Both results are packaged into this struct before
/// `p22_incorporate_landmark_mark` is called.
///
/// Structurally similar to `SextantMark` (P20) but carries the landmark reference
/// rather than the target vehicle reference.
///
/// Spec: p21_p22-spec.md §5.1
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

    /// LOS unit vector from the CSM to the landmark, in the inertial frame (ECI).
    /// Magnitude must be 1.0 ± 1e-6.
    /// Derived from the sextant shaft/trunnion angles rotated by REFSMMAT.
    pub los_inertial: Vec3,

    /// Which scalar component of the LOS unit vector is the observation for this mark.
    /// The caller selects the axis whose `los_inertial` component has the smallest
    /// absolute value, maximising numerical conditioning.
    /// Reuses `LosComponent` from `programs::p20`.
    pub component: LosComponent,
}

// ── Landmark table ─────────────────────────────────────────────────────────────

/// A single landmark entry: Earth-fixed geodetic coordinates.
///
/// Spec: p21_p22-spec.md §3.4
#[derive(Clone, Copy, Debug)]
pub struct LandmarkEntry {
    /// Geocentric latitude (rad). Positive north.
    pub lat_rad: f64,
    /// Longitude east of the IERS reference meridian (rad).
    pub lon_rad: f64,
    /// Altitude above the spherical Earth surface (m).
    pub alt_m: f64,
}

/// Pre-loaded landmark table. Index 0 is unused (landmarks are 1-indexed on the DSKY).
/// The table is fixed at compile time; uplink support is a future extension.
///
/// Indices 1–8 cover eight well-known Earth surface sites used for landmark tracking
/// during the orbital phase. Approximate geocentric coordinates; exact values are
/// not critical for Phase 3 (the tester validates geometry, not exact coordinates).
///
/// Spec: p21_p22-spec.md §3.4 (Override 3: compile-time const)
pub const LANDMARK_TABLE: [LandmarkEntry; 9] = [
    // Index 0 — unused (DSKY is 1-indexed).
    LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 },

    // Index 1 — Kennedy Space Center, Florida, USA
    // ~28.573°N, 80.649°W → lon ≈ -80.649° east
    LandmarkEntry {
        lat_rad:  0.498_820,   // 28.573° N
        lon_rad: -1.408_000,   // 80.649° W
        alt_m:    3.0,
    },

    // Index 2 — Cape Canaveral (slightly north of KSC), Florida, USA
    // ~28.4°N, 80.6°W
    LandmarkEntry {
        lat_rad:  0.495_640,   // 28.4° N
        lon_rad: -1.406_657,   // 80.6° W
        alt_m:    5.0,
    },

    // Index 3 — Cape Town, South Africa
    // ~33.9°S, 18.4°E
    LandmarkEntry {
        lat_rad: -0.591_984,   // 33.9° S
        lon_rad:  0.321_209,   // 18.4° E
        alt_m:   50.0,
    },

    // Index 4 — Perth, Western Australia
    // ~31.9°S, 115.9°E
    LandmarkEntry {
        lat_rad: -0.556_538,   // 31.9° S
        lon_rad:  2.023_365,   // 115.9° E
        alt_m:   25.0,
    },

    // Index 5 — Tokyo, Japan
    // ~35.7°N, 139.7°E
    LandmarkEntry {
        lat_rad:  0.623_010,   // 35.7° N
        lon_rad:  2.439_016,   // 139.7° E
        alt_m:   40.0,
    },

    // Index 6 — Carnarvon, Western Australia (original AGC-era tracking station area)
    // ~24.9°S, 113.7°E
    LandmarkEntry {
        lat_rad: -0.434_461,   // 24.9° S
        lon_rad:  1.984_032,   // 113.7° E
        alt_m:   10.0,
    },

    // Index 7 — Guaymas, Mexico (Sonora coast — AGC-era tracking site)
    // ~27.9°N, 110.9°W
    LandmarkEntry {
        lat_rad:  0.486_914,   // 27.9° N
        lon_rad: -1.936_139,   // 110.9° W
        alt_m:    5.0,
    },

    // Index 8 — Hawaii, USA (Mauna Kea area)
    // ~19.8°N, 155.5°W
    LandmarkEntry {
        lat_rad:  0.345_545,   // 19.8° N
        lon_rad: -2.714_100,   // 155.5° W
        alt_m: 4_200.0,
    },
];

// ── Entry point ────────────────────────────────────────────────────────────────

/// Entry point for P22 (Orbital Navigation / Landmark Tracking).
/// Registered in `PROGRAM_TABLE[22]`.
///
/// Sets `state.major_mode = 22`. Initialises `state.csm_nav` with the default
/// diagonal W-matrix. Installs the Waitlist self-rescheduling hook for the
/// 2-second update cycle.
///
/// # Preconditions
/// - `state.csm_state.epoch` must be non-zero; otherwise alarm 01420 is raised.
/// - `state.gha_epoch_rad` must have been set by uplink.
///
/// # Post-conditions
/// - `state.major_mode == 22`
/// - `state.dsky.prog == 22`
/// - `state.csm_nav.w_matrix` is the default diagonal.
/// - `state.csm_nav.tracking_active == true`
/// - Waitlist entry scheduled for `P22_CYCLE_CS` centiseconds.
///
/// Spec: p21_p22-spec.md §4.2
pub fn p22_init(state: &mut AgcState) -> JobPriority {
    state.major_mode = P22_MAJOR_MODE;
    state.dsky.prog  = P22_MAJOR_MODE;

    // Precondition: CSM frame must be ECI for landmark navigation.
    // (landmark_inertial_pos produces ECI coordinates only.)
    if state.csm_state.frame != Frame::EarthInertial {
        state.alarm.code = ALARM_FRAME_MISMATCH;
        state.alarm.lit  = true;
        state.csm_nav.tracking_active = false;
        return P22_PRIORITY;
    }

    // Precondition: non-zero CSM epoch (sanity check for initialised state vector).
    if state.csm_state.epoch.to_seconds() == 0.0 {
        state.alarm.code = ALARM_NO_CSM_SV;
        state.alarm.lit  = true;
        state.csm_nav.tracking_active = false;
        return P22_PRIORITY;
    }

    // Initialise CsmNavState.
    let now_s = state.time.to_seconds();
    state.csm_nav.last_mark_time = now_s;
    state.csm_nav.mark_count = 0;
    state.csm_nav.reject_count = 0;
    state.csm_nav.consecutive_reject_count = 0;
    state.csm_nav.tracking_active = true;

    // Initialise W-matrix to default diagonal uncertainty.
    p22_rectify_w_matrix_internal(state);

    // Entry display: V16 N43 (current sub-satellite point).
    update_dsky_n43(state);

    // Install the periodic nav-cycle hook via the Waitlist.
    match state.waitlist.schedule(P22_CYCLE_CS_U16, p22_cycle_task) {
        ScheduleResult::Full => {
            state.alarm.code = ALARM_WAITLIST_FULL;
            state.alarm.lit  = true;
        }
        _ => {}
    }

    P22_PRIORITY
}

// ── Nav cycle ─────────────────────────────────────────────────────────────────

/// Periodic P22 navigation update task. Scheduled via Waitlist::schedule.
///
/// Called every `P22_CYCLE_CS` centiseconds (≈ 2 s) after `p22_init`.
///
/// Steps per cycle:
/// 1. Verify CSM frame is ECI; raise alarm and suspend if not.
/// 2. Compute Δt since last mark/cycle; grow W-matrix diagonal by process noise.
///    If Δt > 3600 s, re-initialise W.
/// 3. Update DSKY display registers (V16 N43 — lat/lon/alt of current sub-satellite
///    point, derived from current `state.csm_state`).
/// 4. Re-schedule itself for the next cycle if major_mode == 22.
///
/// # Invariants
/// - Does not modify `state.csm_state` (propagation is the SERVICER's responsibility).
/// - Does not incorporate marks (marks arrive via `p22_incorporate_landmark_mark`).
/// - Runs even when `tracking_active == false` (display update continues).
///
/// Spec: p21_p22-spec.md §4.2; edge case (k)
pub fn p22_cycle_task(state: &mut AgcState) {
    // Edge case (k): frame check — landmark tracking is ECI-only.
    if state.csm_state.frame != Frame::EarthInertial {
        state.alarm.code = ALARM_FRAME_MISMATCH;
        state.alarm.lit  = true;
        state.csm_nav.tracking_active = false;
        reschedule_if_active(state);
        return;
    }

    let now_s = state.time.to_seconds();

    // Step 2: Process-noise covariance growth.
    let dt_noise = now_s - state.csm_nav.last_mark_time;

    // Edge case (h): cap Δt; re-initialise W if exceeded.
    if dt_noise > MAX_PROCESS_NOISE_DT_S {
        p22_rectify_w_matrix(state);
    } else if dt_noise > 0.0 {
        for i in 0..3 {
            state.csm_nav.w_matrix[i][i] += CSM_Q_POS * dt_noise;
        }
        for i in 3..6 {
            state.csm_nav.w_matrix[i][i] += CSM_Q_VEL * dt_noise;
        }
    }

    // Step 3: Update DSKY display.
    update_dsky_n43(state);

    // Step 4: Re-schedule.
    reschedule_if_active(state);
}

// ── Mark incorporation ─────────────────────────────────────────────────────────

/// Incorporate one sextant landmark mark into the CSM navigation solution.
///
/// Called from the sextant HAL handler when the crew completes a mark on a
/// known landmark. Applies the scalar Kalman update (identical algorithm to
/// `p20_incorporate_radar_mark`) to `state.csm_state` and
/// `state.csm_nav.w_matrix`.
///
/// # Arguments
/// - `mark`: decoded sextant observation of an Earth landmark.
///
/// # Preconditions
/// - `state.csm_nav.tracking_active == true`. If false, the mark is silently
///   discarded.
/// - `mark.landmark_inertial` must have been populated by `landmark_inertial_pos`
///   before this function is called.
///
/// Spec: p21_p22-spec.md §4.2; §6.2
pub fn p22_incorporate_landmark_mark(state: &mut AgcState, mark: LandmarkMark) {
    // Edge case (f): tracking not active — silently discard.
    if !state.csm_nav.tracking_active {
        return;
    }

    // Edge case (g): landmark index out of range.
    if mark.landmark_index == 0 || mark.landmark_index > 8 {
        state.alarm.code = ALARM_BAD_LANDMARK_INDEX;
        state.alarm.lit  = true;
        return;
    }

    let csm_pos  = state.csm_state.position;
    let lm_pos   = mark.landmark_inertial;

    // rho_vec = csm_pos - landmark_inertial (points from landmark to CSM).
    let rho_vec = [
        csm_pos[0] - lm_pos[0],
        csm_pos[1] - lm_pos[1],
        csm_pos[2] - lm_pos[2],
    ];
    let rng = norm(rho_vec);

    // Edge case: range too small (safety floor).
    if rng < MIN_LANDMARK_RANGE_M {
        state.alarm.code = ALARM_LANDMARK_RANGE_ZERO;
        state.alarm.lit  = true;
        return;
    }

    // Predicted LOS (landmark → CSM direction).
    let los_hat = unit(rho_vec);
    let c = match mark.component {
        LosComponent::X => 0_usize,
        LosComponent::Y => 1_usize,
        LosComponent::Z => 2_usize,
    };
    let z_predicted = los_hat[c];
    let residual    = mark.los_inertial[c] - z_predicted;

    // Sensitivity vector b (§6.2):
    // b[0..3] = (e_c - los_hat[c] * los_hat) / rng
    // b[3..6] = [0.0; 3]
    let mut e_c = [0.0_f64; 3];
    e_c[c] = 1.0;
    let mut b = [0.0_f64; 6];
    for i in 0..3 {
        b[i] = (e_c[i] - los_hat[c] * los_hat[i]) / rng;
    }
    // b[3..6] remain 0.0 (LOS direction cosine does not depend on velocity).

    match p22_scalar_update(state, b, residual, SIGMA_LANDMARK_SQ) {
        UpdateOutcome::Accepted => {
            state.csm_nav.mark_count += 1;
            state.csm_nav.last_mark_time = mark.time;
            state.csm_nav.consecutive_reject_count = 0;
        }
        UpdateOutcome::Rejected => {
            state.csm_nav.reject_count += 1;
            state.csm_nav.consecutive_reject_count += 1;
            check_consecutive_rejects(state);
        }
        // AcceptedWOverflow is handled inside p22_scalar_update and mapped to Accepted.
        UpdateOutcome::AcceptedWOverflow => unreachable!(),
    }
}

/// Re-initialise the P22 W-matrix to the default diagonal.
///
/// Called on crew command (V32E) or automatically when alarm 01421 fires.
/// Resets mark_count, reject_count, and consecutive_reject_count to 0.
/// Sets last_mark_time to state.time.
///
/// Note: does NOT change `tracking_active`. The caller must re-enable tracking
/// explicitly (typically by re-entering P22 via `p22_init`).
///
/// Spec: p21_p22-spec.md §4.2
pub fn p22_rectify_w_matrix(state: &mut AgcState) {
    p22_rectify_w_matrix_internal(state);
    state.csm_nav.mark_count  = 0;
    state.csm_nav.reject_count = 0;
    state.csm_nav.consecutive_reject_count = 0;
    state.csm_nav.last_mark_time = state.time.to_seconds();

    // Confirm action by showing V06 N49 on DSKY.
    state.dsky.verb  = 6;
    state.dsky.noun  = 49;
    state.dsky.r[0]  = 0.0;
    state.dsky.r[1]  = 0.0;
    state.dsky.r[2]  = 0.0;
}

// ── Landmark coordinate conversion ────────────────────────────────────────────

/// Convert a landmark's Earth-fixed geocentric coordinates to an inertial position
/// vector at the given GET.
///
/// Helper used by the sextant mark handler before calling
/// `p22_incorporate_landmark_mark`. Also usable by P21 tests.
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
/// 1. Compute Earth-fixed Cartesian position from lat/lon/alt.
/// 2. Compute current GHA.
/// 3. Rotate from Earth-fixed to ECI by angle `gha` (apply `Rz(-gha)`).
///    Note: this is the **inverse** of the P21 Rz(+gha) step.
///
/// Spec: p21_p22-spec.md §6.3
pub fn landmark_inertial_pos(
    entry:         &LandmarkEntry,
    get_s:         f64,
    gha_epoch_rad: f64,
) -> Vec3 {
    let lat = entry.lat_rad;
    let lon = entry.lon_rad;
    let r   = R_EARTH + entry.alt_m;

    // Step 1: Earth-fixed Cartesian.
    let cos_lat = libm::cos(lat);
    let sin_lat = libm::sin(lat);
    let cos_lon = libm::cos(lon);
    let sin_lon = libm::sin(lon);
    let r_ef: Vec3 = [
        r * cos_lat * cos_lon,
        r * cos_lat * sin_lon,
        r * sin_lat,
    ];

    // Step 2: GHA at mark time (unbounded; rotation handles wrap-around).
    let gha = gha_epoch_rad + OMEGA_EARTH * get_s;

    // Step 3: Rotate Earth-fixed to ECI (Rz(-gha) = transpose of Rz(+gha)).
    let cos_gha = libm::cos(gha);
    let sin_gha = libm::sin(gha);
    [
        r_ef[0] * cos_gha - r_ef[1] * sin_gha,
        r_ef[0] * sin_gha + r_ef[1] * cos_gha,
        r_ef[2],
    ]
}

// ── Private helpers ────────────────────────────────────────────────────────────

/// Internal W-matrix diagonal initialisation (does NOT reset counters or last_mark_time).
fn p22_rectify_w_matrix_internal(state: &mut AgcState) {
    state.csm_nav.w_matrix = [[0.0; 6]; 6];
    for i in 0..3 {
        state.csm_nav.w_matrix[i][i] = CSM_W_INIT_POS_VARIANCE;
    }
    for i in 3..6 {
        state.csm_nav.w_matrix[i][i] = CSM_W_INIT_VEL_VARIANCE;
    }
}

/// P22-local wrapper around the shared scalar Kalman measurement update.
///
/// Unpacks `csm_state` pos/vel into a flat 6-vector, delegates to
/// `navigation::kalman::scalar_measurement_update`, then writes the result back.
/// On W-matrix overflow, raises alarm 01421 and calls `p22_rectify_w_matrix`.
/// Returns `Accepted` or `Rejected`; never returns `AcceptedWOverflow` to the caller.
///
/// Spec: p21_p22-spec.md §6.2; Override 1 (shared kalman helper)
fn p22_scalar_update(
    state:    &mut AgcState,
    b:        [f64; 6],
    residual: f64,
    sigma_sq: f64,
) -> UpdateOutcome {
    let mut x = [
        state.csm_state.position[0],
        state.csm_state.position[1],
        state.csm_state.position[2],
        state.csm_state.velocity[0],
        state.csm_state.velocity[1],
        state.csm_state.velocity[2],
    ];

    let outcome = crate::navigation::kalman::scalar_measurement_update(
        &mut x,
        &mut state.csm_nav.w_matrix,
        b,
        residual,
        sigma_sq,
    );

    if outcome == UpdateOutcome::Accepted || outcome == UpdateOutcome::AcceptedWOverflow {
        state.csm_state.position = [x[0], x[1], x[2]];
        state.csm_state.velocity = [x[3], x[4], x[5]];
    }

    if outcome == UpdateOutcome::AcceptedWOverflow {
        state.alarm.code = ALARM_CSM_W_OVERFLOW;
        state.alarm.lit  = true;
        p22_rectify_w_matrix(state);
        return UpdateOutcome::Accepted;
    }

    outcome
}

/// Check and act on the consecutive-reject counter.
///
/// Raises alarm 01422 and sets tracking_active = false when the counter
/// reaches 5 consecutive rejects without an accepted mark.
///
/// Spec: p21_p22-spec.md §8.2; edge case (f)
fn check_consecutive_rejects(state: &mut AgcState) {
    if state.csm_nav.consecutive_reject_count >= 5 {
        state.alarm.code = ALARM_LANDMARK_REJECT;
        state.alarm.lit  = true;
        state.csm_nav.tracking_active = false;
    }
}

/// Re-schedule the nav cycle if P22 is still the active major mode.
fn reschedule_if_active(state: &mut AgcState) {
    if state.major_mode != P22_MAJOR_MODE {
        return;
    }
    match state.waitlist.schedule(P22_CYCLE_CS_U16, p22_cycle_task) {
        ScheduleResult::Full => {
            state.alarm.code = ALARM_WAITLIST_FULL;
            state.alarm.lit  = true;
        }
        _ => {}
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;
    use crate::math::linalg::norm;

    // ── Helper ────────────────────────────────────────────────────────────────

    /// Build a minimal `AgcState` with the CSM in LEO at the given pos/vel/epoch,
    /// frame set to ECI, and `gha_epoch_rad = 0.0`.
    fn make_state_with_csm_at(pos: [f64; 3], vel: [f64; 3], epoch_s: f64) -> AgcState {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: pos,
            velocity: vel,
            epoch: Met::from_seconds(epoch_s),
            frame: Frame::EarthInertial,
        };
        state.gha_epoch_rad = 0.0;
        state.time = Met::from_seconds(epoch_s);
        state
    }

    // ── TC-P22-1: Init — W-matrix correctly initialised ───────────────────────

    /// TC-P22-1: Verify that `p22_init` sets major_mode, DSKY, W-matrix diagonal,
    /// tracking_active, and installs the cycle task in the Waitlist.
    #[test]
    fn tc_p22_1_init_w_matrix_and_waitlist() {
        let mut state = make_state_with_csm_at(
            [6_671_000.0, 0.0, 0.0],
            [0.0, 7726.0, 0.0],
            1000.0,
        );

        let prio = p22_init(&mut state);

        // Priority and mode
        assert_eq!(prio, P22_PRIORITY, "priority must be P22_PRIORITY");
        assert_eq!(state.major_mode, P22_MAJOR_MODE, "major_mode must be 22");
        assert_eq!(state.dsky.prog, P22_MAJOR_MODE, "dsky.prog must be 22");

        // W-matrix diagonal initialised correctly
        assert_eq!(
            state.csm_nav.w_matrix[0][0], CSM_W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must equal CSM_W_INIT_POS_VARIANCE"
        );
        assert_eq!(
            state.csm_nav.w_matrix[3][3], CSM_W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] must equal CSM_W_INIT_VEL_VARIANCE"
        );

        // All off-diagonal elements must be zero
        for i in 0..6 {
            for j in 0..6 {
                if i != j {
                    assert_eq!(
                        state.csm_nav.w_matrix[i][j], 0.0,
                        "w_matrix[{}][{}] must be 0 (off-diagonal)", i, j
                    );
                }
            }
        }

        // Tracking active, no alarm
        assert!(state.csm_nav.tracking_active, "tracking_active must be true");
        assert_eq!(state.alarm.code, 0, "no alarm on happy path");

        // Counters zeroed
        assert_eq!(state.csm_nav.mark_count, 0, "mark_count must be 0");

        // Waitlist entry installed
        assert!(
            state.waitlist.len() >= 1,
            "waitlist must have at least one entry after p22_init"
        );
        let entry = state.waitlist.peek(0).expect("waitlist entry 0 must exist");
        assert_eq!(
            entry.task as usize,
            p22_cycle_task as fn(&mut AgcState) as usize,
            "waitlist task must be p22_cycle_task"
        );
    }

    // ── TC-P22-2: landmark_inertial_pos round-trip with P21 Earth-rotation ────

    /// TC-P22-2: Verify that `landmark_inertial_pos` and the P21 Rz(+gha) step
    /// are mathematical inverses: round-trip recovers the original lat/lon.
    #[test]
    fn tc_p22_2_landmark_inertial_pos_round_trip() {
        use crate::programs::p21::OMEGA_EARTH;

        let entry = LandmarkEntry {
            lat_rad: 0.523_6,  // 30° N
            lon_rad: 0.0,
            alt_m:   0.0,
        };
        let get_s = 500.0_f64;
        let gha_epoch_rad = 0.0_f64;

        // Step 1: Convert landmark to inertial (ECI).
        let r_inertial = landmark_inertial_pos(&entry, get_s, gha_epoch_rad);

        // Step 2: Apply P21 Rz(+gha) to get back to Earth-fixed.
        let gha_raw = gha_epoch_rad + OMEGA_EARTH * get_s;
        let two_pi = 2.0 * core::f64::consts::PI;
        let gha = gha_raw - libm::floor(gha_raw / two_pi) * two_pi;
        let cos_gha = libm::cos(gha);
        let sin_gha = libm::sin(gha);
        let pos_ef: [f64; 3] = [
             r_inertial[0] * cos_gha + r_inertial[1] * sin_gha,
            -r_inertial[0] * sin_gha + r_inertial[1] * cos_gha,
             r_inertial[2],
        ];

        // Step 3: Extract lat/lon from Earth-fixed position.
        let r_mag = norm(pos_ef);
        let lat_recovered = libm::asin(pos_ef[2] / r_mag);
        let lon_recovered = libm::atan2(pos_ef[1], pos_ef[0]);

        // Round-trip must recover the original lat/lon to within floating-point rounding.
        assert!(
            libm::fabs(lat_recovered - entry.lat_rad) < 1e-9,
            "recovered lat_rad ({}) must equal entry.lat_rad ({}) within 1e-9",
            lat_recovered, entry.lat_rad
        );
        assert!(
            libm::fabs(lon_recovered - entry.lon_rad) < 1e-9,
            "recovered lon_rad ({}) must equal entry.lon_rad ({}) within 1e-9",
            lon_recovered, entry.lon_rad
        );
    }

    // ── TC-P22-3: Single perfect mark reduces W — zero residual ──────────────

    /// TC-P22-3: A perfect (zero-residual) landmark mark at nadir reduces
    /// W[0][0] slightly and leaves csm_state.position unchanged.
    #[test]
    fn tc_p22_3_perfect_mark_reduces_w_no_position_change() {
        // CSM at (7_000_000, 0, 0) — 629 km altitude on X-axis.
        // Epoch must be non-zero so p22_init does not raise ALARM_NO_CSM_SV.
        // Set gha_epoch_rad = -OMEGA_EARTH * 1000 so the GHA at GET=1000 is
        // exactly zero, keeping the nadir landmark aligned with the +X axis.
        let mut state = make_state_with_csm_at(
            [7_000_000.0, 0.0, 0.0],
            [0.0, 7500.0, 0.0],
            1000.0,
        );
        state.gha_epoch_rad = -OMEGA_EARTH * 1000.0;
        p22_init(&mut state);

        // Landmark directly below at nadir: lat=0, lon=0, alt=0.
        let lm_entry = LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 };
        let lm_inertial = landmark_inertial_pos(&lm_entry, 1000.0, state.gha_epoch_rad);

        // Perfect LOS is [1, 0, 0] (pure +X radial). Observing the X-component
        // of this LOS is a DEGENERATE measurement: b[0] = (1 - los[0]²)/rng = 0
        // because the LOS is aligned with the measurement axis. Instead we
        // observe the Y-component, which has a non-zero sensitivity b[1] = 1/rng
        // and whose "true" value is 0 (since LOS[1] = 0). This gives a
        // well-conditioned scalar update that reduces W[1][1].
        let w11_before = state.csm_nav.w_matrix[1][1];
        let pos_before = state.csm_state.position;

        let mark = LandmarkMark {
            time: 1000.0,
            landmark_index: 1,
            landmark_inertial: lm_inertial,
            los_inertial: [1.0, 0.0, 0.0],
            component: LosComponent::Y,
        };

        p22_incorporate_landmark_mark(&mut state, mark);

        // W[1][1] must have decreased (measurement constrains Y-position).
        assert!(
            state.csm_nav.w_matrix[1][1] < w11_before,
            "W[1][1] must decrease after accepted mark; was {}, now {}",
            w11_before, state.csm_nav.w_matrix[1][1]
        );

        // Zero residual → position must be unchanged (exact or < 1e-9 m).
        for i in 0..3 {
            assert!(
                libm::fabs(state.csm_state.position[i] - pos_before[i]) < 1e-9,
                "csm_state.position[{}] must be unchanged for zero-residual mark", i
            );
        }

        assert_eq!(state.csm_nav.mark_count, 1, "mark_count must be 1");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0");
        assert_eq!(state.alarm.code, 0, "no alarm");
    }

    // ── TC-P22-4: Non-zero residual updates state ─────────────────────────────

    /// TC-P22-4: A mark with non-zero residual (CSM offset 500 m in Y) moves
    /// csm_state.position[1] from 500.0 toward 0.0.
    #[test]
    fn tc_p22_4_nonzero_residual_updates_state() {
        // CSM estimated position has a 500 m Y-error (true CSM would be at [7_000_000, 0, 0]).
        // Epoch must be non-zero so p22_init does not raise ALARM_NO_CSM_SV.
        let mut state = make_state_with_csm_at(
            [7_000_000.0, 500.0, 0.0],
            [0.0, 7500.0, 0.0],
            1000.0,
        );
        p22_init(&mut state);

        // Landmark at lat=lon=alt=0, converted to ECI at GET=1000, gha_epoch=0.
        let lm_entry = LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 };
        let lm_inertial = landmark_inertial_pos(&lm_entry, 1000.0, 0.0);

        // True LOS (from landmark to the true CSM position [7_000_000, 0, 0]).
        let true_csm: Vec3 = [7_000_000.0, 0.0, 0.0];
        let los_true = unit([
            true_csm[0] - lm_inertial[0],
            true_csm[1] - lm_inertial[1],
            true_csm[2] - lm_inertial[2],
        ]);
        let mark = LandmarkMark {
            time: 1000.0,
            landmark_index: 1,
            landmark_inertial: lm_inertial,
            los_inertial: los_true,
            component: LosComponent::X,
        };

        p22_incorporate_landmark_mark(&mut state, mark);

        // The Y-offset creates a small residual; state should move toward truth.
        assert!(
            libm::fabs(state.csm_state.position[1]) < 500.0,
            "csm_state.position[1] should move from 500.0 toward 0.0; got {}",
            state.csm_state.position[1]
        );
        assert_eq!(state.csm_nav.mark_count, 1, "mark_count must be 1");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0");
        assert_eq!(state.alarm.code, 0, "no alarm");
    }

    // ── TC-P22-5: Outlier mark rejected ──────────────────────────────────────

    /// TC-P22-5: A 60° LOS error (residual ≈ -0.5) is rejected by the 3-sigma
    /// gate; state and W-matrix are left unchanged.
    #[test]
    fn tc_p22_5_outlier_mark_rejected() {
        // Same geometry as TC-P22-3: CSM at (7_000_000, 0, 0), nadir landmark.
        // Epoch must be non-zero so p22_init does not raise ALARM_NO_CSM_SV.
        let mut state = make_state_with_csm_at(
            [7_000_000.0, 0.0, 0.0],
            [0.0, 7500.0, 0.0],
            1000.0,
        );
        p22_init(&mut state);

        let lm_entry = LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 };
        let lm_inertial = landmark_inertial_pos(&lm_entry, 1000.0, 0.0);

        let pos_before = state.csm_state.position;
        let w_before = state.csm_nav.w_matrix;

        // Obviously wrong LOS: 60° away from the true direction.
        let mark = LandmarkMark {
            time: 1000.0,
            landmark_index: 1,
            landmark_inertial: lm_inertial,
            los_inertial: [0.5, 0.866, 0.0],
            component: LosComponent::X,
        };

        p22_incorporate_landmark_mark(&mut state, mark);

        assert_eq!(state.csm_nav.reject_count, 1, "reject_count must be 1");
        assert_eq!(state.csm_nav.consecutive_reject_count, 1, "consecutive_reject_count must be 1");

        // Position must be unchanged.
        for i in 0..3 {
            assert!(
                libm::fabs(state.csm_state.position[i] - pos_before[i]) < 1e-9,
                "csm_state.position[{}] must be unchanged after rejected mark", i
            );
        }

        // W-matrix must be unchanged.
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    libm::fabs(state.csm_nav.w_matrix[i][j] - w_before[i][j]) < 1e-9,
                    "w_matrix[{}][{}] must be unchanged after rejected mark", i, j
                );
            }
        }
    }

    // ── TC-P22-6: Five consecutive rejects raise alarm 01422 ─────────────────

    /// TC-P22-6: Five consecutive rejected marks increment consecutive_reject_count
    /// to 5, raise alarm 01422, and set tracking_active = false.
    #[test]
    fn tc_p22_6_five_rejects_alarm_01422() {
        // Epoch must be non-zero so p22_init does not raise ALARM_NO_CSM_SV.
        let mut state = make_state_with_csm_at(
            [7_000_000.0, 0.0, 0.0],
            [0.0, 7500.0, 0.0],
            1000.0,
        );
        p22_init(&mut state);

        let lm_entry = LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 };
        let lm_inertial = landmark_inertial_pos(&lm_entry, 1000.0, 0.0);

        // Five consecutive wildly-wrong marks (predicted≈[1,0,0], observed=[0,1,0]).
        for _ in 0..5 {
            let mark = LandmarkMark {
                time: 1000.0,
                landmark_index: 1,
                landmark_inertial: lm_inertial,
                los_inertial: [0.0, 1.0, 0.0],
                component: LosComponent::X,
            };
            p22_incorporate_landmark_mark(&mut state, mark);
        }

        assert_eq!(state.csm_nav.consecutive_reject_count, 5, "consecutive_reject_count must be 5");
        assert_eq!(state.csm_nav.reject_count, 5, "reject_count must be 5");
        assert_eq!(state.alarm.code, ALARM_LANDMARK_REJECT, "alarm code must be 01422");
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(!state.csm_nav.tracking_active, "tracking_active must be false");
    }

    // ── TC-P22-7: p22_rectify_w_matrix resets counters ───────────────────────

    /// TC-P22-7: `p22_rectify_w_matrix` restores the default W-matrix diagonal,
    /// zeros counters, and does NOT change tracking_active.
    #[test]
    fn tc_p22_7_rectify_resets_counters_not_tracking() {
        // Start from the TC-P22-6 end-state (tracking inactive, alarm lit).
        let mut state = make_state_with_csm_at(
            [7_000_000.0, 0.0, 0.0],
            [0.0, 7500.0, 0.0],
            0.0,
        );
        p22_init(&mut state);
        state.time = Met::from_seconds(0.0);

        let lm_entry = LandmarkEntry { lat_rad: 0.0, lon_rad: 0.0, alt_m: 0.0 };
        let lm_inertial = landmark_inertial_pos(&lm_entry, 0.0, 0.0);

        for _ in 0..5 {
            let mark = LandmarkMark {
                time: 0.0,
                landmark_index: 1,
                landmark_inertial: lm_inertial,
                los_inertial: [0.0, 1.0, 0.0],
                component: LosComponent::X,
            };
            p22_incorporate_landmark_mark(&mut state, mark);
        }
        // Confirm we're in the alarm state.
        assert!(!state.csm_nav.tracking_active);

        // Advance time slightly so last_mark_time check is meaningful.
        state.time = Met::from_seconds(10.0);

        // Call rectify.
        p22_rectify_w_matrix(&mut state);

        // W-matrix restored to default diagonal.
        assert_eq!(
            state.csm_nav.w_matrix[0][0], CSM_W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must be restored to CSM_W_INIT_POS_VARIANCE"
        );
        assert_eq!(
            state.csm_nav.w_matrix[3][3], CSM_W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] must be restored to CSM_W_INIT_VEL_VARIANCE"
        );
        for i in 0..6 {
            for j in 0..6 {
                if i != j {
                    assert_eq!(
                        state.csm_nav.w_matrix[i][j], 0.0,
                        "w_matrix[{}][{}] must be 0 after rectify", i, j
                    );
                }
            }
        }

        // Counters zeroed.
        assert_eq!(state.csm_nav.mark_count, 0, "mark_count must be 0 after rectify");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0 after rectify");
        assert_eq!(state.csm_nav.consecutive_reject_count, 0, "consecutive_reject_count must be 0");

        // last_mark_time == state.time.
        assert!(
            libm::fabs(state.csm_nav.last_mark_time - state.time.to_seconds()) < 1e-9,
            "last_mark_time must equal state.time after rectify"
        );

        // tracking_active must NOT be changed by rectify (spec §TC-P22-7).
        assert!(
            !state.csm_nav.tracking_active,
            "tracking_active must remain false after rectify (caller re-enables)"
        );
    }

    // ── TC-P22-8: Process-noise growth in p22_cycle_task ─────────────────────

    /// TC-P22-8: Verify that `p22_cycle_task` grows W[0][0] and W[3][3] by
    /// the correct process-noise amounts over a 2-second cycle, and re-schedules
    /// itself in the Waitlist.
    #[test]
    fn tc_p22_8_process_noise_growth() {
        let mut state = make_state_with_csm_at(
            [6_671_000.0, 0.0, 0.0],
            [0.0, 7726.0, 0.0],
            1000.0,
        );
        p22_init(&mut state);

        // Set up for the test per spec §TC-P22-8: Δt = 2 s.
        state.csm_nav.last_mark_time = 1000.0;
        state.time = Met::from_seconds(1002.0);
        // Reset W to initial values (p22_init already set them, but be explicit).
        state.csm_nav.w_matrix = [[0.0; 6]; 6];
        for i in 0..3 { state.csm_nav.w_matrix[i][i] = CSM_W_INIT_POS_VARIANCE; }
        for i in 3..6 { state.csm_nav.w_matrix[i][i] = CSM_W_INIT_VEL_VARIANCE; }

        let w00_before = state.csm_nav.w_matrix[0][0];
        let w33_before = state.csm_nav.w_matrix[3][3];

        p22_cycle_task(&mut state);

        // W[0][0] = 250_000 + CSM_Q_POS * 2.0 = 250_001.0
        let expected_w00 = w00_before + CSM_Q_POS * 2.0;
        assert!(
            libm::fabs(state.csm_nav.w_matrix[0][0] - expected_w00) < 1e-6,
            "w_matrix[0][0] should be {}; got {}",
            expected_w00, state.csm_nav.w_matrix[0][0]
        );

        // W[3][3] = 1.0 + CSM_Q_VEL * 2.0 = 1.000_002
        let expected_w33 = w33_before + CSM_Q_VEL * 2.0;
        assert!(
            libm::fabs(state.csm_nav.w_matrix[3][3] - expected_w33) < 1e-6,
            "w_matrix[3][3] should be {}; got {}",
            expected_w33, state.csm_nav.w_matrix[3][3]
        );

        // p22_cycle_task must re-schedule itself in the Waitlist.
        assert!(
            state.waitlist.len() >= 1,
            "waitlist must have at least one entry after p22_cycle_task"
        );
        // Find the p22_cycle_task entry.
        let found = (0..state.waitlist.len()).any(|i| {
            state.waitlist.peek(i)
                .map(|e| e.task as usize == p22_cycle_task as fn(&mut AgcState) as usize)
                .unwrap_or(false)
        });
        assert!(found, "p22_cycle_task must be re-scheduled in the Waitlist");
    }
}

/// Update DSKY with the current sub-satellite point (V16 N43).
///
/// Computes the ground track at `state.time` from `state.csm_state`.
fn update_dsky_n43(state: &mut AgcState) {
    let epoch_s      = state.csm_state.epoch.to_seconds();
    let now_s        = state.time.to_seconds();
    let csm_pos      = state.csm_state.position;
    let csm_vel      = state.csm_state.velocity;
    let gha_epoch    = state.gha_epoch_rad;

    let result = p21_compute_ground_track(
        csm_pos, csm_vel, epoch_s, now_s, gha_epoch,
    );

    const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;
    state.dsky.verb  = 16;
    state.dsky.noun  = 43;
    state.dsky.r[0]  = (result.lat_rad * RAD_TO_DEG * 100.0) as f32;
    state.dsky.r[1]  = (result.lon_rad * RAD_TO_DEG * 100.0) as f32;
    state.dsky.r[2]  = (result.alt_m / 100.0) as f32; // km × 10
    state.dsky.flashing = false;
}
