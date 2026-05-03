//! P23 — Cislunar Midcourse Navigation (star-horizon and star-landmark sightings).
//!
//! Runs during the translunar and transearth coast phases of the lunar mission.
//! Self-reschedules every 10 seconds via the Waitlist to grow process noise and
//! refresh the DSKY display.  Individual measurements arrive asynchronously when
//! the crew completes an optical mark with the sextant (V54E → R52/R53 →
//! `p23_incorporate_star_horizon_mark` or `p23_incorporate_star_landmark_mark`).
//!
//! P23 shares `state.csm_nav` (`CsmNavState`) with P22 — both update the same
//! physical quantity (CSM inertial state vector and its 6×6 uncertainty matrix).
//! Sharing the W-matrix is physically correct: P22 sightings in LEO reduce
//! position uncertainty in the same covariance that P23 uses during cislunar coast.
//!
//! AGC source: Comanche055/P20-P25.agc,
//!             Comanche055/MEASUREMENT_INCORPORATION.agc,
//!             Comanche055/ERASABLE_ASSIGNMENTS.agc (MARKCOUNT, REJECTCNT, WM)
//! Spec: specs/p23-spec.md §1–§9

use crate::executive::job::JobPriority;
use crate::executive::waitlist::ScheduleResult;
use crate::math::linalg::{dot, norm, unit};
use crate::navigation::kalman::{scalar_measurement_update, UpdateOutcome};
use crate::navigation::state_vector::Frame;
use crate::types::Vec3;
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P23.
/// Spec: p23-spec.md §4.6
pub const P23_MAJOR_MODE: u8 = 23;

/// Job priority for P23.  Same as P20 and P22 (background navigation tier).
/// Spec: p23-spec.md §4.6
pub const P23_PRIORITY: JobPriority = 8;

/// Waitlist cycle period for P23 (centiseconds).
/// 10 seconds = 1000 cs.  Cislunar dynamics are orders of magnitude slower than
/// orbital dynamics (P20/P22 use 2 s cycles), so a longer period reduces overhead.
/// Spec: p23-spec.md §4.6
pub const P23_CYCLE_CS: u32 = 1_000;

// Cast to u16 for Waitlist::schedule which takes u16.
const P23_CYCLE_CS_U16: u16 = P23_CYCLE_CS as u16;

/// Initial position variance on the P23 W-matrix diagonal (m²).
/// Corresponds to ±10 km (1-sigma) positional uncertainty at P23 start.
/// The ground uplink for a cislunar state vector has ~10 km uncertainty (MSFN
/// tracking at lunar distances), compared to ~500 m for LEO radar tracking.
/// Spec: p23-spec.md §4.6
pub const P23_W_INIT_POS_VARIANCE: f64 = 1.0e8; // (10 km)²

/// Initial velocity variance on the P23 W-matrix diagonal (m²/s²).
/// Corresponds to ±1 m/s (1-sigma).
/// Spec: p23-spec.md §4.6
pub const P23_W_INIT_VEL_VARIANCE: f64 = 1.0; // (1 m/s)²

/// Process-noise growth rate for CSM position (m²/s).
/// 10× larger than P22's CSM_Q_POS = 0.5 because cislunar coast accumulates
/// perturbation errors faster (solar pressure, unmodelled lunar-gravity gradient).
/// Spec: p23-spec.md §4.6
pub const P23_Q_POS: f64 = 5.0; // m²/s

/// Process-noise growth rate for CSM velocity (m²/s³).
/// 10× larger than P22's CSM_Q_VEL = 1e-6.
/// Spec: p23-spec.md §4.6
pub const P23_Q_VEL: f64 = 1.0e-5; // m²/s³

/// Sextant star-horizon angle noise variance (rad²).
/// Apollo CM sextant angular resolution: ~10 arcsec RMS ≈ 4.85e-5 rad.
/// Spec: p23-spec.md §4.6
pub const SIGMA_STAR_HORIZON_SQ: f64 = 2.5e-9; // (≈10 arcsec)²

/// Sextant star-landmark angle noise variance (rad²).
/// Same sextant hardware as star-horizon; same angular noise floor.
/// Spec: p23-spec.md §4.6
pub const SIGMA_STAR_LANDMARK_SQ: f64 = 2.5e-9; // (≈10 arcsec)²

/// Earth equatorial radius (m).  WGS84 semi-major axis.
/// Spec: p23-spec.md §4.6
pub const EARTH_RADIUS_M: f64 = 6_378_137.0;

/// Moon mean radius (m).  IAU 2012 value.
/// Spec: p23-spec.md §4.6
pub const MOON_RADIUS_M: f64 = 1_737_400.0;

/// Minimum distance from the body surface for a horizon measurement (m).
/// Below this height the horizon-angle formula becomes degenerate
/// (asin approaches π/2 as d → R_body).  100 km above surface.
/// Spec: p23-spec.md §4.6
pub const R_MIN_HORIZON_M: f64 = 100_000.0; // 100 km above surface

/// Minimum CSM-to-landmark slant range for a landmark mark (m).
/// Safety floor; in practice always > 1000 km for cislunar landmark sightings.
/// Spec: p23-spec.md §4.6
pub const P23_MIN_LANDMARK_RANGE_M: f64 = 1_000.0;

/// Maximum Δt for process-noise growth before forced W re-initialisation (s).
/// 24 hours.  Longer than P20/P22's 1-hour cap because cislunar coast passes
/// can last 24–30 h between crew activity cycles.
/// Spec: p23-spec.md §4.6
pub const P23_MAX_PROCESS_NOISE_DT_S: f64 = 86_400.0; // 24 h

// ── Alarm codes ─────────────────────────────────────────────────────────────────
//
// Collision analysis (grep ALARM_ across programs/):
//   p20.rs:  0o01421 ALARM_W_OVERFLOW, 0o00404 ALARM_NO_RADAR, 0o00405 ALARM_REJECT_OVERRIDE,
//            0o00400 ALARM_FRAME_MISMATCH
//   p22.rs:  0o01420 ALARM_NO_CSM_SV, 0o01421 ALARM_CSM_W_OVERFLOW (same code, same semantics),
//            0o01422 ALARM_LANDMARK_REJECT, 0o01424 ALARM_BAD_LANDMARK_INDEX,
//            0o01425 ALARM_LANDMARK_RANGE_ZERO, 0o00400 ALARM_FRAME_MISMATCH
//
// P23 shares 0o01420 (NO_CSM_SV) and 0o01421 (W_OVERFLOW) with P22 — semantics identical.
// P23-exclusive codes: 0o01426–0o01432 (all previously unused; spec §8 confirms this range).

/// Alarm 01420 (octal): no valid CSM state vector (epoch == 0).
/// Shared with P21/P22 — same semantics.
/// Spec: p23-spec.md §8
const ALARM_NO_CSM_SV: u16 = 0o01420;

/// Alarm 00400 (octal): CSM state vector is in an unexpected frame (StableMember).
/// Shared with P20/P22 — same semantics.
/// Spec: p23-spec.md §8
const ALARM_FRAME_MISMATCH: u16 = 0o00400;

/// Alarm 01421 (octal): W-matrix diagonal entry went negative (loss of positive definiteness).
/// Shared with P20/P22 — same code, same semantics (shared W-matrix).
/// Spec: p23-spec.md §8
const ALARM_W_OVERFLOW: u16 = 0o01421;

/// Alarm 01426 (octal): star_direction magnitude not in [0.999, 1.001] (zero or invalid).
/// Spec: p23-spec.md §8
const ALARM_NO_STAR_LOCK: u16 = 0o01426;

/// Alarm 01427 (octal): measured angle outside [0, π] or measurement geometry degenerate
/// (star co-linear with body/landmark direction — unobservable configuration).
/// Spec: p23-spec.md §8, EC-5, EC-6
const ALARM_BAD_ANGLE: u16 = 0o01427;

/// Alarm 01430 (octal): CSM inside R_body + R_MIN_HORIZON_M (horizon geometry degenerate).
/// Spec: p23-spec.md §8
const ALARM_P23_TOO_CLOSE_TO_BODY: u16 = 0o01430;

/// Alarm 01431 (octal): five consecutive marks rejected by 3-sigma gate.
/// Tracking suspended; crew must key V32E to re-enable.
/// Spec: p23-spec.md §8
const ALARM_P23_REJECT_OVERRIDE: u16 = 0o01431;

/// Alarm 01432 (octal): CSM-to-landmark range below P23_MIN_LANDMARK_RANGE_M.
/// Spec: p23-spec.md §8
const ALARM_P23_LANDMARK_RANGE_ZERO: u16 = 0o01432;

/// Alarm 1211: Waitlist full (standard AGC waitlist-overflow alarm).
const ALARM_WAITLIST_FULL: u16 = 1211;

// ── Public types ───────────────────────────────────────────────────────────────

/// Which body's limb or surface is being used as the navigation reference.
///
/// Determined from the CSM frame (`EarthInertial` → Earth, `MoonInertial` → Moon)
/// or from explicit crew selection.
/// Spec: p23-spec.md §5
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    Earth,
    Moon,
}

/// A star-horizon angle measurement from the CM sextant.
///
/// The crew aligns the sextant's movable index mark with the bright limb of the
/// Earth or Moon while simultaneously placing the fixed reticle on a reference star.
/// The sextant reads the half-angle between the star line-of-sight and the nearest
/// point on the body's visible limb (the "horizon").  R52/R53 decodes the sextant
/// CDU angles and delivers this struct to `p23_incorporate_star_horizon_mark`.
///
/// Spec: p23-spec.md §5
#[derive(Clone, Copy, Debug)]
pub struct StarHorizonMark {
    /// Mission Elapsed Time of the sighting (s).
    pub time: f64,

    /// Unit vector toward the reference star in the inertial frame (ECI or MCI).
    /// Magnitude must be 1.0 ± 1e-6.
    pub star_direction: Vec3,

    /// Which body's limb was used as the horizon reference.
    pub body: Body,

    /// The measured star-horizon angle in radians.
    ///
    /// The angle from the body's limb (horizon tangent) to the star.
    /// Valid range: [0, π].
    pub angle_observed_rad: f64,
}

/// A star-landmark angle measurement from the CM sextant.
///
/// The crew sights a known surface feature (crater, cape, mountain) and a
/// reference star simultaneously.  The sextant outputs the angle between the
/// two lines-of-sight.
///
/// The `landmark_inertial` field holds the pre-computed inertial-frame position
/// of the landmark.  P23 does not perform Earth-fixed or Moon-fixed to inertial
/// rotation internally; that conversion is the HAL's responsibility (OQ-1).
///
/// Spec: p23-spec.md §5; OQ-1 resolution
#[derive(Clone, Copy, Debug)]
pub struct StarLandmarkMark {
    /// Mission Elapsed Time of the sighting (s).
    pub time: f64,

    /// Unit vector toward the reference star in the inertial frame.
    /// Magnitude must be 1.0 ± 1e-6.
    pub star_direction: Vec3,

    /// Which body the landmark is on.
    pub body: Body,

    /// Inertial-frame position of the landmark (m), pre-computed by the HAL.
    ///
    /// For Earth landmarks: from `programs::p22::landmark_inertial_pos` using
    /// `state.gha_epoch_rad` and `mark.time`.
    /// For Moon landmarks: from a Moon-rotation model in the HAL.
    /// P23 trusts this value without further coordinate conversion.
    pub landmark_inertial: Vec3,

    /// The measured star-landmark angle (rad).
    /// Valid range: [0, π].
    pub angle_observed_rad: f64,
}

// ── Entry point ────────────────────────────────────────────────────────────────

/// Public shim registered in `PROGRAM_TABLE[23]`.
///
/// The dispatch table signature requires `fn(&mut AgcState) -> JobPriority`.
/// This shim delegates to `p23_init`.
/// Spec: p23-spec.md §2
pub fn init_p23(state: &mut AgcState) -> JobPriority {
    p23_init(state)
}

/// Entry point for P23 (Cislunar Midcourse Navigation).
/// Registered in `PROGRAM_TABLE[23]` via `init_p23`.
///
/// Sets `state.major_mode = 23`.  Re-initialises `state.csm_nav` bookkeeping
/// (mark/reject counters, tracking flag).  If `state.csm_nav.w_matrix` is all
/// zeros, initialises it to the default P23 diagonal (`P23_W_INIT_POS_VARIANCE`,
/// `P23_W_INIT_VEL_VARIANCE`); otherwise the prior covariance (from a preceding
/// P22 session) is preserved (OQ-5).
/// Installs the Waitlist self-rescheduling hook for the 10-second update cycle.
///
/// # Preconditions
/// - `state.csm_state.epoch` must be non-zero; otherwise alarm 01420 is raised
///   and the program returns without installing the Waitlist hook.
/// - `state.csm_state.frame` must be `EarthInertial` or `MoonInertial`; otherwise
///   alarm 00400 is raised and the program returns.
///
/// # W-matrix initialisation policy (OQ-5)
/// If `state.csm_nav.w_matrix` is all zeros, initialise to the P23 default diagonal.
/// Otherwise preserve the existing covariance (prior P22 information).
///
/// # Post-conditions
/// - `state.major_mode == 23`
/// - `state.dsky.prog == 23`
/// - `state.csm_nav.tracking_active == true`
/// - `state.csm_nav.mark_count == 0`, `reject_count == 0`, `consecutive_reject_count == 0`
/// - `state.csm_nav.last_mark_time == state.time.to_seconds()`
/// - Waitlist entry scheduled for `P23_CYCLE_CS` centiseconds.
///
/// Spec: p23-spec.md §4.1
pub fn p23_init(state: &mut AgcState) -> JobPriority {
    // Precondition: CSM frame must be ECI or MCI.  StableMember on a stored state
    // vector is a software fault.
    match state.csm_state.frame {
        Frame::EarthInertial | Frame::MoonInertial => {}
        Frame::StableMember => {
            state.alarm.code = ALARM_FRAME_MISMATCH;
            state.alarm.lit = true;
            state.csm_nav.tracking_active = false;
            return P23_PRIORITY;
        }
    }

    // Precondition: non-zero CSM epoch (sanity check for initialised state vector).
    if state.csm_state.epoch.to_seconds() == 0.0 {
        state.alarm.code = ALARM_NO_CSM_SV;
        state.alarm.lit = true;
        state.csm_nav.tracking_active = false;
        return P23_PRIORITY;
    }

    state.major_mode = P23_MAJOR_MODE;
    state.dsky.prog = P23_MAJOR_MODE;

    // W-matrix initialisation policy (OQ-5):
    // If the matrix is all zeros (fresh-start or first activation), initialise to
    // the P23 default diagonal.  Otherwise inherit the prior covariance.
    if w_matrix_is_zero(&state.csm_nav.w_matrix) {
        init_w_matrix(state);
    }

    // Reset bookkeeping counters — new navigation session.
    let now_s = state.time.to_seconds();
    state.csm_nav.last_mark_time = now_s;
    state.csm_nav.mark_count = 0;
    state.csm_nav.reject_count = 0;
    state.csm_nav.consecutive_reject_count = 0;
    state.csm_nav.tracking_active = true;

    // Program entry display: V16 N49 (mark count / reject count).
    state.dsky.verb = 16;
    state.dsky.noun = 49;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;

    // Install the periodic nav-cycle hook via the Waitlist.
    if state.waitlist.schedule(P23_CYCLE_CS_U16, p23_cycle_task) == ScheduleResult::Full {
        state.alarm.code = ALARM_WAITLIST_FULL;
        state.alarm.lit = true;
    }

    P23_PRIORITY
}

// ── Nav cycle ─────────────────────────────────────────────────────────────────

/// Periodic P23 cislunar navigation update task.  Scheduled via Waitlist::schedule.
///
/// Called every `P23_CYCLE_CS` centiseconds (10 s) after `p23_init`.
///
/// Steps per cycle:
/// 1. Guard: if major_mode != 23, return without self-rescheduling (OQ-2).
/// 2. Verify `state.csm_state.frame` is ECI or MCI; raise alarm 00400 and
///    suspend tracking if not.
/// 3. Compute Δt = state.time.to_seconds() - state.csm_nav.last_mark_time.
///    If Δt > P23_MAX_PROCESS_NOISE_DT_S, call `p23_rectify_w_matrix`.
///    Otherwise grow W diagonal: W[i][i] += P23_Q_POS * Δt for i in 0..3,
///    W[i][i] += P23_Q_VEL * Δt for i in 3..6.
/// 4. Update DSKY: V16 N49 showing mark_count (R1) and reject_count (R2).
/// 5. Re-schedule.
///
/// Spec: p23-spec.md §4.2; OQ-2
pub fn p23_cycle_task(state: &mut AgcState) {
    // OQ-2 self-guard: if crew switched to a different program, stop rescheduling.
    if state.major_mode != P23_MAJOR_MODE {
        return;
    }

    // Frame check (EC-1): StableMember on a StateVector is a software fault.
    match state.csm_state.frame {
        Frame::EarthInertial | Frame::MoonInertial => {}
        Frame::StableMember => {
            state.alarm.code = ALARM_FRAME_MISMATCH;
            state.alarm.lit = true;
            state.csm_nav.tracking_active = false;
            // Still reschedule — display continues even when tracking is suspended.
            reschedule(state);
            return;
        }
    }

    let now_s = state.time.to_seconds();
    let dt_noise = now_s - state.csm_nav.last_mark_time;

    // Process-noise growth (§6.5 / EC-8 cap).
    if dt_noise > P23_MAX_PROCESS_NOISE_DT_S {
        p23_rectify_w_matrix(state);
    } else if dt_noise > 0.0 {
        for i in 0..3 {
            state.csm_nav.w_matrix[i][i] += P23_Q_POS * dt_noise;
        }
        for i in 3..6 {
            state.csm_nav.w_matrix[i][i] += P23_Q_VEL * dt_noise;
        }
    }

    // DSKY update: V16 N49 — mark count (R1), reject count (R2).
    state.dsky.verb = 16;
    state.dsky.noun = 49;
    state.dsky.r[0] = state.csm_nav.mark_count as f32;
    state.dsky.r[1] = state.csm_nav.reject_count as f32;
    state.dsky.r[2] = 0.0;

    reschedule(state);
}

// ── Mark incorporation ─────────────────────────────────────────────────────────

/// Incorporate one star-horizon angle measurement into the CSM navigation solution.
///
/// Called from the sextant HAL handler (R52/R53) when the crew completes a mark
/// on the Earth or Moon horizon against a reference star.
///
/// Performs the scalar Kalman update described in §6.1.
///
/// # Arguments
/// - `mark`: decoded star-horizon observation (see §5).
///
/// # Preconditions and guards
/// - `tracking_active` must be true; otherwise silently discard (EC-9).
/// - `mark.star_direction` magnitude must be in [0.999, 1.001]; alarm 01426 (EC-2).
/// - `mark.angle_observed_rad` must be in [0, π]; alarm 01427 (EC-3).
/// - CSM must be at least `R_body + R_MIN_HORIZON_M` from body centre; alarm 01430 (EC-4).
///
/// Spec: p23-spec.md §4.3, §6.1, §9
pub fn p23_incorporate_star_horizon_mark(state: &mut AgcState, mark: StarHorizonMark) {
    // EC-9: silently discard if tracking not active.
    if !state.csm_nav.tracking_active {
        return;
    }

    // EC-2: star direction must be a unit vector.
    let star_mag = norm(mark.star_direction);
    if !(0.999..=1.001).contains(&star_mag) {
        state.alarm.code = ALARM_NO_STAR_LOCK;
        state.alarm.lit = true;
        return;
    }

    // EC-3: observed angle must be in [0, π].
    if !(0.0..=core::f64::consts::PI).contains(&mark.angle_observed_rad) {
        state.alarm.code = ALARM_BAD_ANGLE;
        state.alarm.lit = true;
        return;
    }

    let csm_pos = state.csm_state.position;
    let s_hat = mark.star_direction;
    let body_pos = body_origin(mark.body);
    let body_r = body_radius(mark.body);

    // Compute predicted measurement and sensitivity vector (§6.1).
    let (theta_pred, b) = match star_horizon_prediction(csm_pos, body_pos, body_r, s_hat) {
        Ok(v) => v,
        Err(HorizonPredError::TooCloseToBody) => {
            // EC-4: CSM inside R_body + R_MIN_HORIZON_M.
            state.alarm.code = ALARM_P23_TOO_CLOSE_TO_BODY;
            state.alarm.lit = true;
            return;
        }
        Err(HorizonPredError::DegenerateGeometry) => {
            // EC-5: star co-linear with body direction — unobservable.
            // Spec §9 EC-5: raise ALARM_BAD_ANGLE (01427) for degenerate geometry.
            state.alarm.code = ALARM_BAD_ANGLE;
            state.alarm.lit = true;
            return;
        }
    };

    let residual = mark.angle_observed_rad - theta_pred;

    // Scalar Kalman update.
    match p23_scalar_update(state, b, residual, SIGMA_STAR_HORIZON_SQ) {
        UpdateOutcome::Accepted => {
            state.csm_nav.mark_count += 1;
            // EC-8: only advance last_mark_time forward.
            if mark.time > state.csm_nav.last_mark_time {
                state.csm_nav.last_mark_time = mark.time;
            }
            state.csm_nav.consecutive_reject_count = 0;
        }
        UpdateOutcome::Rejected => {
            state.csm_nav.reject_count += 1;
            state.csm_nav.consecutive_reject_count += 1;
            check_consecutive_rejects(state);
        }
        // AcceptedWOverflow is handled inside p23_scalar_update and mapped to Accepted.
        UpdateOutcome::AcceptedWOverflow => unreachable!(),
    }
}

/// Incorporate one star-landmark angle measurement into the CSM navigation solution.
///
/// Called from the sextant HAL handler when the crew sights a known surface
/// feature and a reference star simultaneously.
///
/// Performs the scalar Kalman update described in §6.2.
///
/// # Arguments
/// - `mark`: decoded star-landmark observation (see §5).
///
/// Spec: p23-spec.md §4.4, §6.2, §9
pub fn p23_incorporate_star_landmark_mark(state: &mut AgcState, mark: StarLandmarkMark) {
    // EC-9: silently discard if tracking not active.
    if !state.csm_nav.tracking_active {
        return;
    }

    // EC-2: star direction must be a unit vector.
    let star_mag = norm(mark.star_direction);
    if !(0.999..=1.001).contains(&star_mag) {
        state.alarm.code = ALARM_NO_STAR_LOCK;
        state.alarm.lit = true;
        return;
    }

    // EC-3: observed angle must be in [0, π].
    if !(0.0..=core::f64::consts::PI).contains(&mark.angle_observed_rad) {
        state.alarm.code = ALARM_BAD_ANGLE;
        state.alarm.lit = true;
        return;
    }

    let csm_pos = state.csm_state.position;
    let s_hat = mark.star_direction;
    let landmark_inertial = mark.landmark_inertial;

    // Compute predicted measurement and sensitivity vector (§6.2).
    let (beta_pred, b) = match star_landmark_prediction(csm_pos, landmark_inertial, s_hat) {
        Ok(v) => v,
        Err(LandmarkPredError::RangeTooSmall) => {
            // EC-6 range guard: CSM too close to landmark.
            state.alarm.code = ALARM_P23_LANDMARK_RANGE_ZERO;
            state.alarm.lit = true;
            return;
        }
        Err(LandmarkPredError::DegenerateGeometry) => {
            // EC-6: star co-linear with landmark direction — unobservable.
            // Spec §9 EC-6: raise ALARM_BAD_ANGLE (01427) for degenerate geometry.
            state.alarm.code = ALARM_BAD_ANGLE;
            state.alarm.lit = true;
            return;
        }
    };

    let residual = mark.angle_observed_rad - beta_pred;

    // Scalar Kalman update.
    match p23_scalar_update(state, b, residual, SIGMA_STAR_LANDMARK_SQ) {
        UpdateOutcome::Accepted => {
            state.csm_nav.mark_count += 1;
            // EC-8: only advance last_mark_time forward.
            if mark.time > state.csm_nav.last_mark_time {
                state.csm_nav.last_mark_time = mark.time;
            }
            state.csm_nav.consecutive_reject_count = 0;
        }
        UpdateOutcome::Rejected => {
            state.csm_nav.reject_count += 1;
            state.csm_nav.consecutive_reject_count += 1;
            check_consecutive_rejects(state);
        }
        UpdateOutcome::AcceptedWOverflow => unreachable!(),
    }
}

/// Re-initialise the W-matrix to the default P23 diagonal (large uncertainty).
///
/// Called:
/// - On crew command `V32E`.
/// - Automatically when alarm 01421 fires (W-matrix lost positive definiteness).
/// - When the process-noise Δt exceeds `P23_MAX_PROCESS_NOISE_DT_S`.
/// - When the ground uplinks a fresh state vector.
///
/// Resets mark_count, reject_count, and consecutive_reject_count to 0.
/// Sets last_mark_time to state.time.to_seconds().
/// Sets tracking_active to true.
///
/// Spec: p23-spec.md §4.5
pub fn p23_rectify_w_matrix(state: &mut AgcState) {
    init_w_matrix(state);
    state.csm_nav.mark_count = 0;
    state.csm_nav.reject_count = 0;
    state.csm_nav.consecutive_reject_count = 0;
    state.csm_nav.last_mark_time = state.time.to_seconds();
    state.csm_nav.tracking_active = true;

    // Confirm action by showing V06 N49 on DSKY.
    state.dsky.verb = 6;
    state.dsky.noun = 49;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
}

/// Map the current CSM frame to the primary body for horizon measurements.
///
/// Called at the top of each mark-incorporation function to determine which
/// body radius to use and where to place the body-centre origin.
///
/// - `EarthInertial` → `Body::Earth` (Earth at ECI origin).
/// - `MoonInertial`  → `Body::Moon`  (Moon at MCI origin).
///
/// # Panics
/// Panics if `frame == Frame::StableMember` — this frame should never appear
/// on a stored StateVector and indicates a software fault.
///
/// Spec: p23-spec.md §6.4
pub fn primary_body(frame: Frame) -> Body {
    match frame {
        Frame::EarthInertial => Body::Earth,
        Frame::MoonInertial => Body::Moon,
        Frame::StableMember => {
            panic!("p23: primary_body called with StableMember frame — software fault")
        }
    }
}

// ── Private helpers ────────────────────────────────────────────────────────────

/// Returns true iff all 36 entries of the W-matrix are exactly 0.0.
///
/// Used by `p23_init` to implement the OQ-5 W-matrix initialisation policy:
/// inherit from a prior P22 session if non-zero, otherwise initialise fresh.
fn w_matrix_is_zero(w: &[[f64; 6]; 6]) -> bool {
    for row in w.iter() {
        for &v in row.iter() {
            if v != 0.0 {
                return false;
            }
        }
    }
    true
}

/// Set the W-matrix to the P23 default diagonal.  Zeros all off-diagonal elements.
fn init_w_matrix(state: &mut AgcState) {
    state.csm_nav.w_matrix = [[0.0; 6]; 6];
    for i in 0..3 {
        state.csm_nav.w_matrix[i][i] = P23_W_INIT_POS_VARIANCE;
    }
    for i in 3..6 {
        state.csm_nav.w_matrix[i][i] = P23_W_INIT_VEL_VARIANCE;
    }
}

/// Return the body-centre inertial position (always the frame origin) for
/// horizon-mark geometry.
///
/// Per OQ-4: Earth is at ECI origin; Moon is at MCI origin.  P23 does not
/// call an ephemeris function — the primary body is always the frame origin.
/// A non-primary-body sighting (e.g. Moon visible from ECI frame) is out of
/// scope for this phase and is handled as a landmark mark instead.
#[inline]
fn body_origin(_body: Body) -> Vec3 {
    [0.0; 3]
}

/// Return the mean equatorial radius of the body (m).
#[inline]
fn body_radius(body: Body) -> f64 {
    match body {
        Body::Earth => EARTH_RADIUS_M,
        Body::Moon => MOON_RADIUS_M,
    }
}

/// Error variants returned by `star_horizon_prediction` on degenerate geometry.
enum HorizonPredError {
    /// CSM is inside `R_body + R_MIN_HORIZON_M` — EC-4.
    TooCloseToBody,
    /// Star direction is co-linear with body-centre direction (sin_alpha < 1e-9) — EC-5.
    DegenerateGeometry,
}

/// Star-horizon angle measurement prediction and sensitivity vector.
///
/// Returns `Ok((theta_pred, b))` on success, or `Err(HorizonPredError)` if the
/// geometry is degenerate.
///
/// # Arguments
/// - `csm_pos`: CSM inertial position (m).
/// - `body_pos`: Body centre inertial position (m).  Always `[0; 3]` for the primary body.
/// - `body_radius_m`: Body equatorial radius (m).
/// - `s_hat`: Star unit vector in the inertial frame.
///
/// Spec: p23-spec.md §6.1; EC-4, EC-5
fn star_horizon_prediction(
    csm_pos: Vec3,
    body_pos: Vec3,
    body_radius_m: f64,
    s_hat: Vec3,
) -> Result<(f64, [f64; 6]), HorizonPredError> {
    // Step 1: relative position from body centre to CSM.
    let rho = [
        csm_pos[0] - body_pos[0],
        csm_pos[1] - body_pos[1],
        csm_pos[2] - body_pos[2],
    ];

    // Step 2: distance from body centre to CSM.
    let d = norm(rho);

    // Step 3: safety guard (EC-4).
    if d < body_radius_m + R_MIN_HORIZON_M {
        return Err(HorizonPredError::TooCloseToBody);
    }

    // Step 4: angular radius of the body as seen from CSM.
    let phi = libm::asin(body_radius_m / d);

    // Step 5: unit vector from body centre to CSM.
    let u_hat = unit(rho);

    // Step 6: predicted star-horizon angle.
    let cos_alpha = dot(s_hat, u_hat).clamp(-1.0, 1.0);
    let alpha = libm::acos(cos_alpha);
    let sin_alpha = libm::sqrt((1.0 - cos_alpha * cos_alpha).max(0.0));

    // EC-5: star nearly co-linear with body-centre direction — measurement degenerate.
    if sin_alpha < 1.0e-9 {
        return Err(HorizonPredError::DegenerateGeometry);
    }

    let theta_pred = alpha - phi;

    // Sensitivity vector b (§6.1 closed-form):
    //   tangent_len = sqrt(d² - R²)
    //   A = (cos_alpha / sin_alpha + R / tangent_len) / d
    //   B = 1 / (d * sin_alpha)
    //   b[0..3] = A * u_hat - B * s_hat
    //   b[3..6] = 0
    let tangent_len = libm::sqrt(d * d - body_radius_m * body_radius_m);
    let a_scalar = (cos_alpha / sin_alpha + body_radius_m / tangent_len) / d;
    let b_scalar = 1.0 / (d * sin_alpha);

    let mut b = [0.0_f64; 6];
    for i in 0..3 {
        b[i] = a_scalar * u_hat[i] - b_scalar * s_hat[i];
    }
    // b[3..6] stay 0.0 (theta_pred does not depend on velocity).

    Ok((theta_pred, b))
}

/// Error variants returned by `star_landmark_prediction` on degenerate geometry.
enum LandmarkPredError {
    /// CSM-to-landmark range below `P23_MIN_LANDMARK_RANGE_M` — EC-6 range guard.
    RangeTooSmall,
    /// Star direction co-linear with landmark direction (sin_beta < 1e-9) — EC-6.
    DegenerateGeometry,
}

/// Star-landmark angle measurement prediction and sensitivity vector.
///
/// Returns `Ok((beta_pred, b))` on success, or `Err(LandmarkPredError)` if the
/// geometry is degenerate.
///
/// Spec: p23-spec.md §6.2; EC-6
fn star_landmark_prediction(
    csm_pos: Vec3,
    landmark_inertial: Vec3,
    s_hat: Vec3,
) -> Result<(f64, [f64; 6]), LandmarkPredError> {
    // Step 1: vector from CSM to landmark (points toward the landmark).
    let v_lm = [
        landmark_inertial[0] - csm_pos[0],
        landmark_inertial[1] - csm_pos[1],
        landmark_inertial[2] - csm_pos[2],
    ];

    // Step 2: distance and direction.
    let d_lm = norm(v_lm);

    // Step 3: safety guard.
    if d_lm < P23_MIN_LANDMARK_RANGE_M {
        return Err(LandmarkPredError::RangeTooSmall);
    }

    let l_hat = unit(v_lm);
    let cos_beta = dot(s_hat, l_hat).clamp(-1.0, 1.0);
    let sin_beta = libm::sqrt((1.0 - cos_beta * cos_beta).max(0.0));

    // EC-6: star nearly co-linear with landmark direction — measurement degenerate.
    if sin_beta < 1.0e-9 {
        return Err(LandmarkPredError::DegenerateGeometry);
    }

    // Step 4: predicted star-landmark angle.
    let beta_pred = libm::acos(cos_beta);

    // Sensitivity vector b (§6.2):
    //   b[0..3] = (s_hat - cos_beta * l_hat) / (d_lm * sin_beta)
    //   b[3..6] = 0
    let mut b = [0.0_f64; 6];
    for i in 0..3 {
        b[i] = (s_hat[i] - cos_beta * l_hat[i]) / (d_lm * sin_beta);
    }

    Ok((beta_pred, b))
}

/// P23-local wrapper around the shared scalar Kalman measurement update.
///
/// Unpacks `csm_state` pos/vel into a flat 6-vector, delegates to
/// `navigation::kalman::scalar_measurement_update`, then writes the result back.
/// On W-matrix overflow, raises alarm 01421 and calls `p23_rectify_w_matrix`.
/// Returns `Accepted` or `Rejected`; never returns `AcceptedWOverflow` to callers.
///
/// Spec: p23-spec.md §6.3
fn p23_scalar_update(
    state: &mut AgcState,
    b: [f64; 6],
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

    let outcome =
        scalar_measurement_update(&mut x, &mut state.csm_nav.w_matrix, b, residual, sigma_sq);

    if outcome == UpdateOutcome::Accepted || outcome == UpdateOutcome::AcceptedWOverflow {
        state.csm_state.position = [x[0], x[1], x[2]];
        state.csm_state.velocity = [x[3], x[4], x[5]];
    }

    if outcome == UpdateOutcome::AcceptedWOverflow {
        state.alarm.code = ALARM_W_OVERFLOW;
        state.alarm.lit = true;
        p23_rectify_w_matrix(state);
        return UpdateOutcome::Accepted;
    }

    outcome
}

/// Check and act on the consecutive-reject counter.
///
/// Raises alarm 01431 and sets tracking_active = false when five consecutive
/// marks have been rejected without an accepted mark.
///
/// Spec: p23-spec.md §4.3, §8
fn check_consecutive_rejects(state: &mut AgcState) {
    if state.csm_nav.consecutive_reject_count >= 5 {
        state.alarm.code = ALARM_P23_REJECT_OVERRIDE;
        state.alarm.lit = true;
        state.csm_nav.tracking_active = false;
    }
}

/// Re-schedule the P23 nav cycle for the next period.
///
/// Separated from `p23_cycle_task` to keep the cycle function readable.
fn reschedule(state: &mut AgcState) {
    if state.waitlist.schedule(P23_CYCLE_CS_U16, p23_cycle_task) == ScheduleResult::Full {
        state.alarm.code = ALARM_WAITLIST_FULL;
        state.alarm.lit = true;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;
    use crate::AgcState;

    // ── TC-P23-1: p23_init happy path ─────────────────────────────────────────

    /// TC-P23-1: Verify that p23_init sets major_mode, DSKY, W-matrix diagonal,
    /// counters, tracking_active, and installs a Waitlist entry.
    #[test]
    fn tc_p23_1_init_happy_path() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [3.84e8, 0.0, 0.0],
            velocity: [0.0, 800.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::EarthInertial,
        };
        // csm_nav already default (all zeros) from AgcState::new()

        p23_init(&mut state);

        assert_eq!(state.major_mode, 23, "major_mode must be 23");
        assert_eq!(state.dsky.prog, 23, "dsky.prog must be 23");
        assert!(
            state.csm_nav.tracking_active,
            "tracking_active must be true"
        );
        assert_eq!(state.csm_nav.mark_count, 0, "mark_count must be 0");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0");
        assert_eq!(
            state.csm_nav.consecutive_reject_count, 0,
            "consecutive_reject_count must be 0"
        );
        assert_eq!(
            state.csm_nav.w_matrix[0][0], P23_W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must be P23_W_INIT_POS_VARIANCE"
        );
        assert_eq!(
            state.csm_nav.w_matrix[3][3], P23_W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] must be P23_W_INIT_VEL_VARIANCE"
        );
        assert_eq!(
            state.csm_nav.w_matrix[0][1], 0.0,
            "w_matrix[0][1] must be 0 (off-diagonal)"
        );
        assert_eq!(state.alarm.code, 0, "alarm.code must be 0 on happy path");
        assert!(
            !state.waitlist.is_empty(),
            "waitlist must have at least one entry after p23_init"
        );
    }

    // ── TC-P23-2: p23_init with zero CSM epoch raises alarm ───────────────────

    /// TC-P23-2: p23_init with StateVector::ZERO (epoch == 0) raises alarm 01420
    /// and disables tracking.
    #[test]
    fn tc_p23_2_init_zero_epoch_alarm() {
        let mut state = AgcState::new();
        state.csm_state = StateVector::ZERO;

        p23_init(&mut state);

        assert_eq!(
            state.alarm.code, 0o01420,
            "alarm.code must be 0o01420 (NO_CSM_SV)"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            !state.csm_nav.tracking_active,
            "tracking_active must be false"
        );
    }

    // ── TC-P23-3: Star-horizon mark reduces W[1][1] ───────────────────────────

    /// TC-P23-3: A star-horizon mark with near-zero residual reduces W[1][1]
    /// (the Y-component, dominant sensitivity direction) and leaves W[0][0]
    /// and velocity rows unchanged.
    #[test]
    fn tc_p23_3_star_horizon_mark_reduces_w_y_component() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [3.0e8, 0.0, 0.0],
            velocity: [0.0, 800.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::EarthInertial,
        };
        p23_init(&mut state);

        let pos_before = state.csm_state.position;
        let w00_before = state.csm_nav.w_matrix[0][0];

        // Compute the predicted angle analytically so the residual is exactly zero.
        // phi = asin(EARTH_RADIUS_M / 3e8)
        // alpha = acos(dot([0,1,0], [1,0,0])) = acos(0) = pi/2
        // theta_pred = alpha - phi
        let phi: f64 = libm::asin(EARTH_RADIUS_M / 3.0e8);
        let alpha: f64 = core::f64::consts::PI / 2.0;
        let theta_pred = alpha - phi;

        let mark = StarHorizonMark {
            time: 1000.0,
            star_direction: [0.0, 1.0, 0.0],
            body: Body::Earth,
            angle_observed_rad: theta_pred,
        };

        p23_incorporate_star_horizon_mark(&mut state, mark);

        assert_eq!(state.csm_nav.mark_count, 1, "mark_count must be 1");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0");
        assert_eq!(state.alarm.code, 0, "alarm.code must be 0");

        // Y-component W must be reduced (dominant sensitivity b[1] ≈ -3.33e-9).
        assert!(
            state.csm_nav.w_matrix[1][1] < P23_W_INIT_POS_VARIANCE,
            "w_matrix[1][1] must be reduced; got {}",
            state.csm_nav.w_matrix[1][1]
        );
        // Expected range [5e7, 9e7] per spec §11.
        assert!(
            state.csm_nav.w_matrix[1][1] >= 5.0e7,
            "w_matrix[1][1] must be >= 5e7; got {}",
            state.csm_nav.w_matrix[1][1]
        );
        assert!(
            state.csm_nav.w_matrix[1][1] <= 9.0e7,
            "w_matrix[1][1] must be <= 9e7; got {}",
            state.csm_nav.w_matrix[1][1]
        );

        // X-component W is only weakly constrained (b[0] ≈ 7.1e-11 vs. b[1] ≈ -3.33e-9,
        // a ratio of ~47x). The X-reduction is proportional to (b[0]/b[1])² ≈ 1/2200, so
        // the X-reduction (~14 km²) is roughly 2200× smaller than the Y-reduction (~31 Mm²).
        // Assert that X is dominated by Y by at least a factor of 100 to pin the geometry.
        let x_reduction = w00_before - state.csm_nav.w_matrix[0][0];
        let y_reduction = P23_W_INIT_POS_VARIANCE - state.csm_nav.w_matrix[1][1];
        assert!(
            x_reduction >= 0.0 && x_reduction * 100.0 < y_reduction,
            "X-component reduction ({}) must be << Y-component reduction ({})",
            x_reduction,
            y_reduction
        );

        // Velocity rows unchanged (b[3..6] = 0).
        assert_eq!(
            state.csm_nav.w_matrix[3][3], P23_W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] (velocity) must be unchanged"
        );

        // Near-zero residual → position changes by at most 1.0 m.
        for (i, &before) in pos_before.iter().enumerate() {
            let now = state.csm_state.position[i];
            assert!(
                libm::fabs(now - before) < 1.0,
                "csm_state.position[{i}] must change by at most 1.0 m; was {before}, now {now}"
            );
        }
    }

    // ── TC-P23-4: Outlier rejected by 3-sigma gate ────────────────────────────

    /// TC-P23-4: A star-horizon mark with residual ≈ 1.0 rad (far outside the
    /// 3-sigma gate) is rejected; counters and state are updated accordingly.
    #[test]
    fn tc_p23_4_outlier_rejected_by_3sigma_gate() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [3.0e8, 0.0, 0.0],
            velocity: [0.0, 800.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::EarthInertial,
        };
        p23_init(&mut state);

        let pos_before = state.csm_state.position;
        let w_before = state.csm_nav.w_matrix;

        let phi: f64 = libm::asin(EARTH_RADIUS_M / 3.0e8);
        let alpha: f64 = core::f64::consts::PI / 2.0;
        let theta_pred = alpha - phi;

        let mark = StarHorizonMark {
            time: 1000.0,
            star_direction: [0.0, 1.0, 0.0],
            body: Body::Earth,
            angle_observed_rad: theta_pred + 1.0, // residual = 1.0 rad — way beyond 3-sigma
        };

        p23_incorporate_star_horizon_mark(&mut state, mark);

        assert_eq!(
            state.csm_nav.mark_count, 0,
            "mark_count must be 0 (not accepted)"
        );
        assert_eq!(state.csm_nav.reject_count, 1, "reject_count must be 1");
        assert_eq!(
            state.csm_nav.consecutive_reject_count, 1,
            "consecutive_reject_count must be 1"
        );
        assert_eq!(
            state.alarm.code, 0,
            "single rejection must not raise an alarm"
        );

        // Position must be unchanged.
        for (i, &ref_val) in pos_before.iter().enumerate() {
            assert!(
                libm::fabs(state.csm_state.position[i] - ref_val) < 1e-9,
                "csm_state.position[{i}] must be unchanged after rejected mark"
            );
        }

        // W-matrix must be unchanged.
        for (i, ref_row) in w_before.iter().enumerate() {
            for (j, &ref_val) in ref_row.iter().enumerate() {
                assert!(
                    libm::fabs(state.csm_nav.w_matrix[i][j] - ref_val) < 1e-9,
                    "w_matrix[{i}][{j}] must be unchanged after rejected mark"
                );
            }
        }
    }

    // ── TC-P23-5: Five consecutive rejects raise alarm 01431 ──────────────────

    /// TC-P23-5: Five consecutive outlier marks set consecutive_reject_count to 5,
    /// raise alarm 01431, and disable tracking.
    #[test]
    fn tc_p23_5_five_consecutive_rejects_alarm_01431() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [3.0e8, 0.0, 0.0],
            velocity: [0.0, 800.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::EarthInertial,
        };
        p23_init(&mut state);

        let pos_before = state.csm_state.position;

        let phi: f64 = libm::asin(EARTH_RADIUS_M / 3.0e8);
        let alpha: f64 = core::f64::consts::PI / 2.0;
        let theta_pred = alpha - phi;

        for _ in 0..5 {
            let mark = StarHorizonMark {
                time: 1000.0,
                star_direction: [0.0, 1.0, 0.0],
                body: Body::Earth,
                angle_observed_rad: theta_pred + 1.0,
            };
            p23_incorporate_star_horizon_mark(&mut state, mark);
        }

        assert_eq!(
            state.csm_nav.consecutive_reject_count, 5,
            "consecutive_reject_count must be 5"
        );
        assert_eq!(state.csm_nav.reject_count, 5, "reject_count must be 5");
        assert!(
            !state.csm_nav.tracking_active,
            "tracking_active must be false after 5 rejects"
        );
        assert_eq!(
            state.alarm.code, 0o01431,
            "alarm.code must be 0o01431 (REJECT_OVERRIDE)"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");

        // All marks were rejected — position unchanged.
        for (i, &ref_val) in pos_before.iter().enumerate() {
            assert!(
                libm::fabs(state.csm_state.position[i] - ref_val) < 1e-9,
                "csm_state.position[{i}] must be unchanged after 5 rejected marks"
            );
        }

        // 6th mark must be silently discarded (tracking_active == false).
        let mark6 = StarHorizonMark {
            time: 1001.0,
            star_direction: [0.0, 1.0, 0.0],
            body: Body::Earth,
            angle_observed_rad: theta_pred,
        };
        let alarm_code_before_6th = state.alarm.code;
        p23_incorporate_star_horizon_mark(&mut state, mark6);
        assert_eq!(
            state.csm_nav.mark_count, 0,
            "6th mark must be discarded (mark_count stays 0)"
        );
        assert_eq!(
            state.alarm.code, alarm_code_before_6th,
            "alarm code must be unchanged after silently discarded 6th mark"
        );
    }

    // ── TC-P23-6: primary_body pure function ──────────────────────────────────

    /// TC-P23-6: primary_body maps EarthInertial → Earth and MoonInertial → Moon.
    #[test]
    fn tc_p23_6_primary_body_mapping() {
        assert_eq!(primary_body(Frame::EarthInertial), Body::Earth);
        assert_eq!(primary_body(Frame::MoonInertial), Body::Moon);
    }

    // ── TC-P23-7: CSM inside body raises alarm 01430 ──────────────────────────

    /// TC-P23-7: A star-horizon mark when the CSM is below R_body + R_MIN_HORIZON_M
    /// raises alarm 01430 (TOO_CLOSE_TO_BODY) and discards the mark.
    #[test]
    fn tc_p23_7_csm_inside_body_alarm_01430() {
        let mut state = AgcState::new();
        // First init with a valid far position so tracking_active = true.
        state.csm_state = StateVector {
            position: [3.0e8, 0.0, 0.0],
            velocity: [0.0, 800.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::EarthInertial,
        };
        p23_init(&mut state);

        // Now move CSM to 50 km altitude (below the 100 km R_MIN_HORIZON_M guard).
        // Threshold = EARTH_RADIUS_M + R_MIN_HORIZON_M = 6_478_137.0 m
        // CSM at EARTH_RADIUS_M + 50_000 = 6_428_137.0 m < threshold
        state.csm_state.position = [EARTH_RADIUS_M + 50_000.0, 0.0, 0.0];
        let pos_before = state.csm_state.position;

        let mark = StarHorizonMark {
            time: 1000.0,
            star_direction: [0.0, 1.0, 0.0],
            body: Body::Earth,
            angle_observed_rad: 0.0,
        };

        p23_incorporate_star_horizon_mark(&mut state, mark);

        assert_eq!(
            state.alarm.code, 0o01430,
            "alarm.code must be 0o01430 (TOO_CLOSE_TO_BODY)"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert_eq!(
            state.csm_nav.mark_count, 0,
            "mark_count must be 0 (mark discarded)"
        );

        // Position must be unchanged.
        for (i, &ref_val) in pos_before.iter().enumerate() {
            assert!(
                libm::fabs(state.csm_state.position[i] - ref_val) < 1e-9,
                "csm_state.position[{i}] must be unchanged after discarded mark"
            );
        }
    }

    // ── TC-P23-8: Star-landmark mark reduces W[0][0] ──────────────────────────

    /// TC-P23-8: A star-landmark mark with near-zero residual reduces W[0][0]
    /// (X-component, dominant sensitivity) and leaves W[1][1] essentially unchanged.
    #[test]
    fn tc_p23_8_star_landmark_mark_reduces_w_x_component() {
        let mut state = AgcState::new();
        state.csm_state = StateVector {
            position: [0.0, 1.0e8, 0.0],
            velocity: [100.0, 0.0, 0.0],
            epoch: Met::from_seconds(1_000_000.0),
            frame: Frame::MoonInertial,
        };
        p23_init(&mut state);

        let pos_before = state.csm_state.position;
        let w11_before = state.csm_nav.w_matrix[1][1];

        // Compute beta_pred analytically.
        // v_lm = [0, MOON_RADIUS_M, 0] - [0, 1e8, 0] = [0, MOON_RADIUS_M - 1e8, 0]
        // l_hat = [0, -1, 0]  (pointing from CSM toward Moon)
        // star  = [1, 0, 0]
        // cos_beta = dot([1,0,0], [0,-1,0]) = 0.0  → beta_pred = pi/2
        let beta_pred: f64 = core::f64::consts::PI / 2.0;

        let mark = StarLandmarkMark {
            time: 1000.0,
            star_direction: [1.0, 0.0, 0.0],
            body: Body::Moon,
            landmark_inertial: [0.0, MOON_RADIUS_M, 0.0],
            angle_observed_rad: beta_pred,
        };

        p23_incorporate_star_landmark_mark(&mut state, mark);

        assert_eq!(state.csm_nav.mark_count, 1, "mark_count must be 1");
        assert_eq!(state.csm_nav.reject_count, 0, "reject_count must be 0");
        assert_eq!(state.alarm.code, 0, "alarm.code must be 0");

        // X-component W must be reduced (b[0] ≈ 1.018e-8 — dominant).
        assert!(
            state.csm_nav.w_matrix[0][0] < P23_W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must be reduced; got {}",
            state.csm_nav.w_matrix[0][0]
        );

        // Y-component W essentially unchanged (b[1] = 0).
        assert!(
            libm::fabs(state.csm_nav.w_matrix[1][1] - w11_before) < 1.0,
            "w_matrix[1][1] must be essentially unchanged; was {}, now {}",
            w11_before,
            state.csm_nav.w_matrix[1][1]
        );

        // Near-zero residual → position changes by at most 1.0 m.
        for (i, &before) in pos_before.iter().enumerate() {
            let now = state.csm_state.position[i];
            assert!(
                libm::fabs(now - before) < 1.0,
                "csm_state.position[{i}] must change by at most 1.0 m; was {before}, now {now}"
            );
        }
    }
}
