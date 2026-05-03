//! P20 — Rendezvous Navigation.
//!
//! Runs as a continuously active background job during the rendezvous phase,
//! maintaining the onboard estimate of the target vehicle (LM) state vector
//! and the CSM-to-target relative state.
//!
//! Every ~2-second Waitlist cycle this module:
//!  1. Grows the W-matrix diagonal by process noise proportional to elapsed time.
//!  2. Propagates the stored target state vector forward to `state.time`.
//!  3. Recomputes the LVLH relative state.
//!  4. Updates the DSKY V16 N54 display.
//!
//! Measurement marks from the rendezvous radar (R22) or the CM sextant are
//! incorporated via a scalar Kalman update (sequential filter).
//!
//! AGC source: Comanche055/P20-P25.agc, MEASUREMENT_INCORPORATION.agc,
//!             W_MATRIX_RECTIFICATION.agc
//! Spec: specs/p20-spec.md

use crate::executive::job::JobPriority;
use crate::executive::waitlist::ScheduleResult;
use crate::guidance::rendezvous::{
    los_angles_lvlh, range, range_rate, relative_state_lvlh, LvlhState,
};
use crate::math::kepler::kepler_step;
use crate::math::linalg::{dot, norm, unit, vsub};
use crate::navigation::gravity::{MU_EARTH, MU_MOON};
use crate::navigation::state_vector::Frame;
use crate::types::Vec3;
use crate::AgcState;

// ── Constants ──────────────────────────────────────────────────────────────────

/// Major mode number for P20.
/// Spec: p20-spec.md §4.6
pub const P20_MAJOR_MODE: u8 = 20;

/// Job priority for P20.
/// Lower than DAP (37) and SERVICER (20); higher than background targeting jobs.
/// Spec: p20-spec.md §4.1
pub const P20_PRIORITY: JobPriority = 8;

/// Initial position variance on W-matrix diagonal (m²).
/// Corresponds to roughly ±500 m (1-sigma) positional uncertainty at P20 start.
/// Spec: p20-spec.md §4.6
pub const W_INIT_POS_VARIANCE: f64 = 250_000.0;

/// Initial velocity variance on W-matrix diagonal (m²/s²).
/// Corresponds to roughly ±1 m/s (1-sigma) velocity uncertainty.
/// Spec: p20-spec.md §4.6
pub const W_INIT_VEL_VARIANCE: f64 = 1.0;

/// Process-noise rate for position (m²/s).
/// ΔW_pos = Q_POS * Δt
/// Spec: p20-spec.md §4.6
pub const Q_POS: f64 = 0.5;

/// Process-noise rate for velocity (m²/s³).
/// ΔW_vel = Q_VEL * Δt
/// Spec: p20-spec.md §4.6
pub const Q_VEL: f64 = 1.0e-6;

/// Radar range measurement noise variance (m²).
/// 1-sigma ~15 m at nominal radar lock.
/// Spec: p20-spec.md §4.6
pub const SIGMA_RANGE_SQ: f64 = 225.0;

/// Radar range-rate measurement noise variance (m²/s²).
/// 1-sigma ~0.15 m/s.
/// Spec: p20-spec.md §4.6
pub const SIGMA_RANGE_RATE_SQ: f64 = 0.0225;

/// Sextant LOS angle noise variance (rad²).
/// 1-sigma ~0.1 mrad (20 arcsec).
/// Spec: p20-spec.md §4.6
pub const SIGMA_SEXTANT_SQ: f64 = 1.0e-8;

/// Minimum range for radar/sextant mark incorporation (m).
/// Spec: p20-spec.md §4.6
pub const MIN_TRACKING_RANGE_M: f64 = 50.0;

/// Waitlist interval for the nav cycle (centiseconds).
/// 200 cs = 2 seconds.
const NAV_CYCLE_CS: u16 = 200;

/// Maximum Δt for process-noise growth before W-matrix re-initialisation (s).
const MAX_PROCESS_NOISE_DT_S: f64 = 3600.0;

// ── Alarm codes ────────────────────────────────────────────────────────────────

/// Alarm 01421 (octal): W-matrix diagonal entry went negative (loss of positive definiteness).
const ALARM_W_OVERFLOW: u16 = 0o01421;

/// Alarm 00404 (octal): No valid target state vector on entry, or radar lost for > 60 s.
const ALARM_NO_RADAR: u16 = 0o00404;

/// Alarm 00405 (octal): Five consecutive marks rejected by the 3-sigma gate.
const ALARM_REJECT_OVERRIDE: u16 = 0o00405;

/// Alarm 00400 (octal): CSM and target state vectors are in different coordinate frames.
const ALARM_FRAME_MISMATCH: u16 = 0o00400;

/// Alarm 1211: Waitlist full (standard AGC waitlist-overflow alarm).
const ALARM_WAITLIST_FULL: u16 = 1211;

// ── Navigation state ───────────────────────────────────────────────────────────

/// Navigation state maintained by P20 (Rendezvous Navigation).
///
/// Lives inside `AgcState` so the Executive, SERVICER, and restart handler
/// can all read/write it without passing extra arguments.
///
/// Spec: p20-spec.md §3.2
#[derive(Clone, Debug)]
pub struct RendezvousNavState {
    /// Estimated inertial position of the target vehicle (m).
    /// Frame must match `AgcState::csm_state.frame` (ECI or MCI).
    /// Corresponds to AGC erasable `RONE` (scale B+28 m).
    pub target_pos: Vec3,

    /// Estimated inertial velocity of the target vehicle (m/s).
    /// Corresponds to AGC erasable `VONE` (scale B+7 m/s).
    pub target_vel: Vec3,

    /// Epoch of the current target state estimate (Mission Elapsed Time, seconds).
    /// Corresponds to AGC erasable `TIMET`.
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
    /// Displayed via V06 N45.
    /// Corresponds to AGC erasable `MARKCOUNT`.
    pub mark_count: u16,

    /// Count of measurement marks rejected by the 3-sigma gate since P20 start.
    /// Displayed via V06 N49.
    /// Corresponds to AGC erasable `REJECTCNT`.
    pub reject_count: u16,

    /// Consecutive rejects since the last accepted mark. Reset to 0 on acceptance.
    /// Triggers alarm 00405 when it reaches 5.
    /// Spec override: added per Phase 2 hard override §3 (§11 edge case (c)).
    pub consecutive_reject_count: u8,

    /// Most recently computed relative state in the rendezvous LVLH frame.
    /// Spec: p20-spec.md §3.2
    pub lvlh_state: LvlhState,

    /// True when P20 is actively tracking and incorporating marks.
    /// Set false if the radar loses lock or the crew selects REJECT OVERRIDE.
    /// Corresponds to AGC bit flag `TRACKFLAG`.
    pub tracking_active: bool,
}

impl Default for RendezvousNavState {
    /// Zero-initialise all navigation state.
    ///
    /// Spec: p20-spec.md §3.3
    fn default() -> Self {
        Self {
            target_pos: [0.0; 3],
            target_vel: [0.0; 3],
            target_epoch: 0.0,
            w_matrix: [[0.0; 6]; 6],
            last_mark_time: 0.0,
            mark_count: 0,
            reject_count: 0,
            consecutive_reject_count: 0,
            lvlh_state: LvlhState {
                rho: [0.0; 3],
                rho_dot: [0.0; 3],
            },
            tracking_active: false,
        }
    }
}

// ── Measurement types ──────────────────────────────────────────────────────────

/// A single rendezvous radar measurement mark.
///
/// Decoded output of the R22 rendezvous radar interface.
/// The R22 hardware produced one measurement frame approximately every 0.5 seconds
/// when in track mode (O'Brien p. 311).
///
/// Spec: p20-spec.md §5.1
#[derive(Clone, Copy, Debug)]
pub struct RadarMark {
    /// Mission Elapsed Time of the measurement (s).
    pub time: f64,

    /// Slant range to the target (m). Always >= 0.
    /// Valid only if `range_valid == true`.
    pub range_m: f64,

    /// Range rate (m/s). Positive = target moving away.
    /// Sign convention matches `guidance::rendezvous::range_rate`.
    /// Valid only if `range_rate_valid == true`.
    pub range_rate_mps: f64,

    /// True if the range measurement is valid (radar locked, no AGC fault).
    pub range_valid: bool,

    /// True if the range-rate measurement is valid.
    pub range_rate_valid: bool,
}

/// A single sextant line-of-sight mark.
///
/// The CM sextant measured the direction to the target vehicle in the spacecraft
/// body frame. The optics unit vector is converted to the inertial frame using
/// the current REFSMMAT before being delivered here.
///
/// Spec: p20-spec.md §5.2
#[derive(Clone, Copy, Debug)]
pub struct SextantMark {
    /// Mission Elapsed Time of the sighting (s).
    pub time: f64,

    /// LOS unit vector from the CSM to the target, in the inertial frame.
    /// Magnitude must be 1.0 ± 1e-6.
    pub los_inertial: Vec3,

    /// Which scalar component of the LOS is being used as the observation.
    pub component: LosComponent,
}

/// Which component of the LOS unit vector is the scalar observable for this mark.
///
/// Spec: p20-spec.md §5.2
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LosComponent {
    /// Dot product with the inertial X-axis.
    X,
    /// Dot product with the inertial Y-axis.
    Y,
    /// Dot product with the inertial Z-axis.
    Z,
}

// ── Scalar update outcome ──────────────────────────────────────────────────────

/// Re-export the shared UpdateOutcome so existing call sites in this module
/// compile unchanged.
use crate::navigation::kalman::UpdateOutcome;

// ── Public entry point ─────────────────────────────────────────────────────────

/// Entry point for P20 (Rendezvous Navigation). Registered in `PROGRAM_TABLE[20]`.
///
/// Sets major_mode = 20, validates preconditions, initialises `RendezvousNavState`
/// from the current `state.target_state` uplinked state vector, and schedules the
/// periodic nav-cycle hook via the Waitlist.
///
/// # Returns
/// `P20_PRIORITY` (8) in all cases. On alarm conditions the major mode is still
/// advanced to 20 so the crew can observe the alarm and take corrective action.
///
/// # Alarms
/// - 00404: target state is zero (no valid radar data).
/// - 00400: csm_state.frame != target_state.frame.
///
/// Spec: p20-spec.md §4.1
pub fn init_p20(state: &mut AgcState) -> JobPriority {
    p20_init(state)
}

/// P20 initialisation body.
///
/// Spec: p20-spec.md §4.1
pub fn p20_init(state: &mut AgcState) -> JobPriority {
    state.major_mode = P20_MAJOR_MODE;
    state.dsky.prog = P20_MAJOR_MODE;

    // ── Precondition: frame consistency ──────────────────────────────────────
    if state.csm_state.frame != state.target_state.frame {
        state.alarm.code = ALARM_FRAME_MISMATCH;
        state.alarm.lit = true;
        state.rendezvous_nav.tracking_active = false;
        return P20_PRIORITY;
    }

    // ── Precondition: non-zero target state ───────────────────────────────────
    // A zero position and zero velocity at epoch 0 identifies StateVector::ZERO.
    let target_is_zero =
        state.target_state.position == [0.0_f64; 3] && state.target_state.velocity == [0.0_f64; 3];
    if target_is_zero {
        state.alarm.code = ALARM_NO_RADAR;
        state.alarm.lit = true;
        state.rendezvous_nav.tracking_active = false;
        // Show current reject count (O'Brien p. 312 entry display).
        state.dsky.verb = 6;
        state.dsky.noun = 49;
        state.dsky.r[0] = state.rendezvous_nav.reject_count as f32;
        state.dsky.r[1] = 0.0;
        state.dsky.r[2] = 0.0;
        return P20_PRIORITY;
    }

    // ── Initialise RendezvousNavState from uplinked target SV ─────────────────
    let now_s = state.time.to_seconds();

    state.rendezvous_nav.target_pos = state.target_state.position;
    state.rendezvous_nav.target_vel = state.target_state.velocity;
    state.rendezvous_nav.target_epoch = state.target_state.epoch.to_seconds();
    state.rendezvous_nav.last_mark_time = now_s;
    state.rendezvous_nav.mark_count = 0;
    state.rendezvous_nav.reject_count = 0;
    state.rendezvous_nav.consecutive_reject_count = 0;
    state.rendezvous_nav.tracking_active = true;

    // Initialise W-matrix to diagonal uncertainty.
    p20_rectify_w_matrix_internal(state);

    // ── DSKY entry display: briefly show V06 N49 (reject count) ──────────────
    // O'Brien p. 312: entry display.
    state.dsky.verb = 6;
    state.dsky.noun = 49;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
    state.dsky.flashing = false;

    // ── Install the periodic nav-cycle hook via the Waitlist ──────────────────
    // Override 1: use Waitlist, not servicer_exit.
    if state
        .waitlist
        .schedule(NAV_CYCLE_CS, p20_rendezvous_nav_cycle)
        == ScheduleResult::Full
    {
        state.alarm.code = ALARM_WAITLIST_FULL;
        state.alarm.lit = true;
    }

    P20_PRIORITY
}

// ── Nav cycle ─────────────────────────────────────────────────────────────────

/// Periodic rendezvous navigation update.
///
/// Called by the Waitlist approximately every 2 seconds.
///
/// Steps performed each cycle:
/// 1. Apply covariance growth (process noise) proportional to Δt since last_mark_time.
/// 2. Propagate target_pos/target_vel forward to state.time using kepler_step.
/// 3. Recompute lvlh_state from propagated target SV and current CSM SV.
/// 4. Update DSKY display registers (V16 N54).
/// 5. Re-schedule itself for the next cycle (if major_mode == 20).
///
/// Spec: p20-spec.md §4.2
pub fn p20_rendezvous_nav_cycle(state: &mut AgcState) {
    let now_s = state.time.to_seconds();

    // ── Edge case (e): frame mismatch ─────────────────────────────────────────
    if state.csm_state.frame != state.target_state.frame {
        state.alarm.code = ALARM_FRAME_MISMATCH;
        state.alarm.lit = true;
        state.rendezvous_nav.tracking_active = false;
        reschedule_if_active(state);
        return;
    }

    // ── Step 1: Process-noise covariance growth ───────────────────────────────
    let dt_noise = now_s - state.rendezvous_nav.last_mark_time;

    // Edge case (g): cap Δt at 3600 s; re-initialise W if exceeded.
    if dt_noise > MAX_PROCESS_NOISE_DT_S {
        p20_rectify_w_matrix(state);
    } else if dt_noise > 0.0 {
        for i in 0..3 {
            state.rendezvous_nav.w_matrix[i][i] += Q_POS * dt_noise;
        }
        for i in 3..6 {
            state.rendezvous_nav.w_matrix[i][i] += Q_VEL * dt_noise;
        }
    }

    // ── Step 2: Propagate target state vector to current time ─────────────────
    let epoch = state.rendezvous_nav.target_epoch;
    let dt_prop = now_s - epoch;

    // Edge case (h): negative dt is valid (backward propagation).
    if dt_prop != 0.0 {
        let mu = mu_for_frame(state.csm_state.frame);
        let (new_pos, new_vel) = kepler_step(
            state.rendezvous_nav.target_pos,
            state.rendezvous_nav.target_vel,
            dt_prop,
            mu,
        );
        state.rendezvous_nav.target_pos = new_pos;
        state.rendezvous_nav.target_vel = new_vel;
        state.rendezvous_nav.target_epoch = now_s;
    }

    // ── Step 3: Recompute LVLH relative state ─────────────────────────────────
    let csm_pos = state.csm_state.position;
    let csm_vel = state.csm_state.velocity;
    let tgt_pos = state.rendezvous_nav.target_pos;
    let tgt_vel = state.rendezvous_nav.target_vel;

    // Edge case (f): skip update when vehicles are practically co-located.
    let rng = range(csm_pos, tgt_pos);
    if rng < 1.0 {
        // Docking contact — suppress LVLH update and N54 display.
        reschedule_if_active(state);
        return;
    }

    // Edge case (b): terminal-phase proximity.
    if rng < MIN_TRACKING_RANGE_M {
        state.rendezvous_nav.tracking_active = false;
        state.alarm.code = ALARM_NO_RADAR;
        state.alarm.lit = true;
        reschedule_if_active(state);
        return;
    }

    state.rendezvous_nav.lvlh_state = relative_state_lvlh(csm_pos, csm_vel, tgt_pos, tgt_vel);

    // ── Step 4: Update DSKY V16 N54 ───────────────────────────────────────────
    // R1: slant range (m).
    // R2: range-rate, **positive = closing** (O'Brien p. 329 N54 convention,
    //     opposite to range_rate sign, hence the negation).
    // R3: elevation angle (rad) from LVLH local horizontal to LOS.
    let rdot = range_rate(csm_pos, csm_vel, tgt_pos, tgt_vel);
    let los = los_angles_lvlh(&state.rendezvous_nav.lvlh_state);

    state.dsky.verb = 16;
    state.dsky.noun = 54;
    state.dsky.r[0] = rng as f32;
    state.dsky.r[1] = (-rdot) as f32; // positive = closing per N54 convention
    state.dsky.r[2] = los.elevation as f32;
    state.dsky.flashing = false;

    // ── Step 5: Re-schedule if still in P20 ───────────────────────────────────
    reschedule_if_active(state);
}

// ── Mark incorporation ────────────────────────────────────────────────────────

/// Incorporate one rendezvous radar measurement mark into the navigation solution.
///
/// Performs the scalar Kalman update for a range or range-rate observation (§6).
/// If the residual exceeds the 3-sigma gate the mark is counted in `reject_count`
/// but state and W are not modified.
///
/// Spec: p20-spec.md §4.3
pub fn p20_incorporate_radar_mark(state: &mut AgcState, mark: RadarMark) {
    // Edge case (a): radar not tracking.
    if !mark.range_valid && !mark.range_rate_valid {
        return;
    }

    let csm_pos = state.csm_state.position;
    let csm_vel = state.csm_state.velocity;
    let tgt_pos = state.rendezvous_nav.target_pos;

    let rho_vec = vsub(tgt_pos, csm_pos);
    let rng = norm(rho_vec);

    // Edge case (b): terminal proximity.
    if rng < MIN_TRACKING_RANGE_M {
        return;
    }

    // Process range observation.
    if mark.range_valid {
        // §6.2: predicted range.
        let z_predicted = rng;
        let residual = mark.range_m - z_predicted;

        // §6.4: sensitivity vector b for range.
        // b[0..3] = rho_vec / rng  (= los_hat)
        // b[3..6] = [0, 0, 0]
        let los_hat = unit(rho_vec);
        let mut b = [0.0_f64; 6];
        b[0] = los_hat[0];
        b[1] = los_hat[1];
        b[2] = los_hat[2];
        // b[3..6] remain 0.0

        match scalar_measurement_update(state, b, residual, SIGMA_RANGE_SQ) {
            UpdateOutcome::Accepted => {
                state.rendezvous_nav.mark_count += 1;
                state.rendezvous_nav.last_mark_time = mark.time;
                state.rendezvous_nav.consecutive_reject_count = 0;
            }
            UpdateOutcome::Rejected => {
                state.rendezvous_nav.reject_count += 1;
                state.rendezvous_nav.consecutive_reject_count += 1;
                check_consecutive_rejects(state);
            }
            // AcceptedWOverflow is mapped to Accepted inside the P20 wrapper.
            UpdateOutcome::AcceptedWOverflow => unreachable!(),
        }
    }

    // Process range-rate observation (re-fetch potentially-updated state).
    if mark.range_rate_valid {
        let tgt_pos2 = state.rendezvous_nav.target_pos;
        let tgt_vel2 = state.rendezvous_nav.target_vel;
        let rho2 = vsub(tgt_pos2, csm_pos);
        let rhodot2 = vsub(tgt_vel2, csm_vel);
        let rng2 = norm(rho2);

        if rng2 < MIN_TRACKING_RANGE_M {
            return;
        }

        // §6.2: predicted range-rate.
        let z_predicted = dot(rho2, rhodot2) / rng2;
        let residual = mark.range_rate_mps - z_predicted;

        // §6.4: sensitivity vector b for range-rate.
        // b[0..3] = rho_dot_vec / rng - (dot(rho, rho_dot) / rng^3) * rho
        // b[3..6] = rho / rng
        let rng2_cubed = rng2 * rng2 * rng2;
        let rdot_scale = dot(rho2, rhodot2) / rng2_cubed;
        let mut b = [0.0_f64; 6];
        for i in 0..3 {
            b[i] = rhodot2[i] / rng2 - rdot_scale * rho2[i];
        }
        for i in 0..3 {
            b[i + 3] = rho2[i] / rng2;
        }

        match scalar_measurement_update(state, b, residual, SIGMA_RANGE_RATE_SQ) {
            UpdateOutcome::Accepted => {
                state.rendezvous_nav.mark_count += 1;
                state.rendezvous_nav.last_mark_time = mark.time;
                state.rendezvous_nav.consecutive_reject_count = 0;
            }
            UpdateOutcome::Rejected => {
                state.rendezvous_nav.reject_count += 1;
                state.rendezvous_nav.consecutive_reject_count += 1;
                check_consecutive_rejects(state);
            }
            // AcceptedWOverflow is mapped to Accepted inside the P20 wrapper.
            UpdateOutcome::AcceptedWOverflow => unreachable!(),
        }
    }
}

/// Incorporate one sextant line-of-sight mark into the navigation solution.
///
/// Each sextant mark provides a single scalar observable: one component of the
/// LOS unit vector. Processes it as a scalar Kalman update.
///
/// Spec: p20-spec.md §4.4
pub fn p20_incorporate_sextant_mark(state: &mut AgcState, mark: SextantMark) {
    let csm_pos = state.csm_state.position;
    let tgt_pos = state.rendezvous_nav.target_pos;
    let rho_vec = vsub(tgt_pos, csm_pos);
    let rng = norm(rho_vec);

    // Edge case (b): terminal proximity.
    if rng < MIN_TRACKING_RANGE_M {
        return;
    }

    // §6.2: predicted LOS component.
    let los_hat = unit(rho_vec);
    let c = match mark.component {
        LosComponent::X => 0_usize,
        LosComponent::Y => 1_usize,
        LosComponent::Z => 2_usize,
    };
    let z_predicted = los_hat[c];
    let residual = mark.los_inertial[c] - z_predicted;

    // §6.4: sensitivity vector b for sextant LOS component c.
    // b[0..3] = (e_c - los_hat[c] * los_hat) / rng
    // b[3..6] = [0, 0, 0]
    let mut e_c = [0.0_f64; 3];
    e_c[c] = 1.0;
    let mut b = [0.0_f64; 6];
    for i in 0..3 {
        b[i] = (e_c[i] - los_hat[c] * los_hat[i]) / rng;
    }
    // b[3..6] remain 0.0

    match scalar_measurement_update(state, b, residual, SIGMA_SEXTANT_SQ) {
        UpdateOutcome::Accepted => {
            state.rendezvous_nav.mark_count += 1;
            state.rendezvous_nav.last_mark_time = mark.time;
            state.rendezvous_nav.consecutive_reject_count = 0;
        }
        UpdateOutcome::Rejected => {
            state.rendezvous_nav.reject_count += 1;
            state.rendezvous_nav.consecutive_reject_count += 1;
            check_consecutive_rejects(state);
        }
        // AcceptedWOverflow is mapped to Accepted inside the P20 wrapper.
        UpdateOutcome::AcceptedWOverflow => unreachable!(),
    }
}

/// Re-initialise the W-matrix to the default diagonal (large uncertainty).
///
/// Called when the crew keys V32E (reject last mark and reinitialise W) or
/// when P20 is started with an uplinked state whose quality is unknown.
///
/// Spec: p20-spec.md §4.5
pub fn p20_rectify_w_matrix(state: &mut AgcState) {
    p20_rectify_w_matrix_internal(state);
    state.rendezvous_nav.mark_count = 0;
    state.rendezvous_nav.reject_count = 0;
    state.rendezvous_nav.last_mark_time = state.time.to_seconds();

    // Confirm action by showing V06 N49 on DSKY (O'Brien p. 312, §8.5).
    state.dsky.verb = 6;
    state.dsky.noun = 49;
    state.dsky.r[0] = 0.0;
    state.dsky.r[1] = 0.0;
    state.dsky.r[2] = 0.0;
}

// ── Private helpers ────────────────────────────────────────────────────────────

/// Internal W-matrix diagonal initialisation (does NOT reset counters or DSKY).
///
/// Sets W to the initial diagonal uncertainty; zeros all off-diagonal entries.
fn p20_rectify_w_matrix_internal(state: &mut AgcState) {
    // Zero the full 6×6 matrix first.
    state.rendezvous_nav.w_matrix = [[0.0; 6]; 6];
    // Position diagonal (rows 0..2).
    for i in 0..3 {
        state.rendezvous_nav.w_matrix[i][i] = W_INIT_POS_VARIANCE;
    }
    // Velocity diagonal (rows 3..5).
    for i in 3..6 {
        state.rendezvous_nav.w_matrix[i][i] = W_INIT_VEL_VARIANCE;
    }
}

/// P20-local wrapper around the shared scalar Kalman measurement update.
///
/// Unpacks the `RendezvousNavState` pos/vel into a flat 6-vector, delegates to
/// `navigation::kalman::scalar_measurement_update`, then writes the result back.
/// On W-matrix overflow, raises alarm 01421 and calls `p20_rectify_w_matrix`.
///
/// This wrapper preserves the observable behaviour of the original function
/// (including alarm and rectify on overflow) so that existing P20 tests and
/// callers within this module need no changes.
///
/// Spec: p20-spec.md §6.5–§6.10; Override 1 (shared kalman helper)
fn scalar_measurement_update(
    state: &mut AgcState,
    b: [f64; 6],
    residual: f64,
    sigma_sq: f64,
) -> UpdateOutcome {
    let mut x = [
        state.rendezvous_nav.target_pos[0],
        state.rendezvous_nav.target_pos[1],
        state.rendezvous_nav.target_pos[2],
        state.rendezvous_nav.target_vel[0],
        state.rendezvous_nav.target_vel[1],
        state.rendezvous_nav.target_vel[2],
    ];

    let outcome = crate::navigation::kalman::scalar_measurement_update(
        &mut x,
        &mut state.rendezvous_nav.w_matrix,
        b,
        residual,
        sigma_sq,
    );

    if outcome == UpdateOutcome::Accepted || outcome == UpdateOutcome::AcceptedWOverflow {
        state.rendezvous_nav.target_pos = [x[0], x[1], x[2]];
        state.rendezvous_nav.target_vel = [x[3], x[4], x[5]];
    }

    if outcome == UpdateOutcome::AcceptedWOverflow {
        state.alarm.code = ALARM_W_OVERFLOW;
        state.alarm.lit = true;
        p20_rectify_w_matrix(state);
        return UpdateOutcome::Accepted;
    }

    outcome
}

/// Check and act on the consecutive-reject counter.
///
/// Raises alarm 00405 and sets tracking_active = false when the counter
/// reaches 5 consecutive rejects without an accepted mark.
///
/// Spec: p20-spec.md §6.6, §11 edge case (c)
fn check_consecutive_rejects(state: &mut AgcState) {
    if state.rendezvous_nav.consecutive_reject_count >= 5 {
        state.alarm.code = ALARM_REJECT_OVERRIDE;
        state.alarm.lit = true;
        state.rendezvous_nav.tracking_active = false;
    }
}

/// Re-schedule the nav cycle if P20 is still the active major mode.
///
/// Called at the end of `p20_rendezvous_nav_cycle`. If major_mode != 20 the
/// cycle stops (crew has left P20).
///
/// Spec: p20-spec.md override §1
fn reschedule_if_active(state: &mut AgcState) {
    if state.major_mode != P20_MAJOR_MODE {
        return;
    }
    if state
        .waitlist
        .schedule(NAV_CYCLE_CS, p20_rendezvous_nav_cycle)
        == ScheduleResult::Full
    {
        state.alarm.code = ALARM_WAITLIST_FULL;
        state.alarm.lit = true;
    }
}

/// Select gravitational parameter for the current navigation frame.
fn mu_for_frame(frame: Frame) -> f64 {
    match frame {
        Frame::MoonInertial => MU_MOON,
        _ => MU_EARTH,
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::{Frame, StateVector};
    use crate::types::Met;
    use crate::AgcState;

    // ── Helpers ────────────────────────────────────────────────────────────────

    /// Build a CSM state vector in ECI at 300 km circular LEO.
    fn csm_leo() -> StateVector {
        StateVector {
            position: [6_671_000.0, 0.0, 0.0],
            velocity: [0.0, 7726.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        }
    }

    /// Build a target state vector 2 km behind the CSM in ECI.
    fn target_leo_2km_behind() -> StateVector {
        StateVector {
            position: [6_671_000.0, -2000.0, 0.0],
            velocity: [0.0, 7726.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        }
    }

    // ── TC-P20-1: Init with valid uplinked target SV ───────────────────────────

    /// TC-P20-1: Verify `p20_init` correctly initialises state and installs
    /// the nav cycle hook in the Waitlist.
    #[test]
    fn tc_p20_1_init_valid_target_sv() {
        let mut state = AgcState::new();
        state.time = Met::from_seconds(1000.0);
        state.csm_state = csm_leo();
        state.target_state = target_leo_2km_behind();

        let prio = p20_init(&mut state);

        // Return value and major mode
        assert_eq!(prio, P20_PRIORITY, "priority must be P20_PRIORITY");
        assert_eq!(state.major_mode, 20, "major_mode must be 20");
        assert_eq!(state.dsky.prog, 20, "dsky.prog must be 20");

        // Tracking active, no alarm
        assert!(
            state.rendezvous_nav.tracking_active,
            "tracking_active must be true"
        );
        assert_eq!(state.alarm.code, 0, "no alarm on happy path");

        // Target position initialised from uplinked SV
        assert!(
            libm::fabs(state.rendezvous_nav.target_pos[0] - 6_671_000.0) < 1.0,
            "target_pos[0] should match uplinked SV"
        );
        assert!(
            libm::fabs(state.rendezvous_nav.target_pos[1] - (-2000.0)) < 1.0,
            "target_pos[1] should match uplinked SV"
        );
        assert!(
            libm::fabs(state.rendezvous_nav.target_pos[2] - 0.0) < 1.0,
            "target_pos[2] should match uplinked SV"
        );

        // W-matrix initialised to diagonal
        assert_eq!(
            state.rendezvous_nav.w_matrix[0][0], W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must equal W_INIT_POS_VARIANCE"
        );
        assert_eq!(
            state.rendezvous_nav.w_matrix[3][3], W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] must equal W_INIT_VEL_VARIANCE"
        );

        // Counters reset
        assert_eq!(
            state.rendezvous_nav.mark_count, 0,
            "mark_count must be 0 on init"
        );

        // Nav cycle hook installed in waitlist (override 1: use Waitlist not servicer_exit)
        assert!(
            !state.waitlist.is_empty(),
            "waitlist must have at least one entry after p20_init"
        );
        let entry = state.waitlist.peek(0).expect("waitlist entry 0 must exist");
        assert_eq!(
            entry.task as usize,
            (p20_rendezvous_nav_cycle as fn(&mut AgcState)) as usize,
            "waitlist entry task must be p20_rendezvous_nav_cycle"
        );
    }

    // ── TC-P20-2: Init with zero target SV raises alarm 00404 ─────────────────

    /// TC-P20-2: Verify that `p20_init` with `StateVector::ZERO` as target
    /// raises alarm 00404 and leaves tracking inactive.
    #[test]
    fn tc_p20_2_init_zero_target_sv_alarms_00404() {
        let mut state = AgcState::new();
        state.time = Met::from_seconds(1000.0);
        state.csm_state = csm_leo();
        // target_state remains StateVector::ZERO (the default from AgcState::new)

        p20_init(&mut state);

        assert!(
            !state.rendezvous_nav.tracking_active,
            "tracking_active must be false"
        );
        assert_eq!(state.alarm.code, ALARM_NO_RADAR, "alarm must be 00404");
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert_eq!(
            state.major_mode, 20,
            "major_mode IS advanced to 20 even on alarm"
        );
    }

    // ── TC-P20-3: Three radar marks converge the estimate ─────────────────────

    /// TC-P20-3: Verify that successive zero-noise range marks reduce W-matrix
    /// uncertainty and pull the estimated target position toward truth.
    #[test]
    fn tc_p20_3_three_radar_marks_converge() {
        let mut state = AgcState::new();

        // CSM at 7000 km, true target 10 km ahead in-track (y-axis)
        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };

        // Initial estimate offset +500 m in y
        state.rendezvous_nav.target_pos = [7_000_000.0, 10_500.0, 0.0];
        state.rendezvous_nav.target_vel = [0.0, 7500.0, 0.0];
        state.rendezvous_nav.tracking_active = true;

        // Initialise W to diagonal
        p20_rectify_w_matrix(&mut state);

        // The range measurement is in the y-direction (b = [0,1,0,0,0,0]),
        // so W[1][1] is the variance that decreases with each accepted mark.
        let w11_initial = state.rendezvous_nav.w_matrix[1][1];

        // Three perfect range marks; true range = 10_000.0 m
        for (i, t) in [1000.0_f64, 1002.0, 1004.0].iter().enumerate() {
            let mark = RadarMark {
                time: *t,
                range_m: 10_000.0,
                range_rate_mps: 0.0,
                range_valid: true,
                range_rate_valid: false,
            };
            p20_incorporate_radar_mark(&mut state, mark);
            // W[1][1] (the y-position variance, which the range marks constrain)
            // must decrease after each accepted mark.
            assert!(
                state.rendezvous_nav.w_matrix[1][1] < w11_initial,
                "W[1][1] must decrease after mark {}",
                i + 1
            );
        }

        // Counters
        assert_eq!(state.rendezvous_nav.mark_count, 3, "mark_count must be 3");
        assert_eq!(
            state.rendezvous_nav.reject_count, 0,
            "reject_count must be 0"
        );
        assert_eq!(state.alarm.code, 0, "no alarm after clean marks");

        // Rough convergence: position estimate should be within 100 m of truth
        assert!(
            libm::fabs(state.rendezvous_nav.target_pos[1] - 10_000.0) < 100.0,
            "target_pos[1] should converge within 100 m of truth; got {}",
            state.rendezvous_nav.target_pos[1]
        );
    }

    // ── TC-P20-4: Outlier mark rejected, state unchanged ──────────────────────

    /// TC-P20-4: Verify that a wildly-wrong radar mark (residual ~40 km) is
    /// rejected by the 3-sigma gate and leaves state and W unchanged.
    #[test]
    fn tc_p20_4_outlier_mark_rejected() {
        let mut state = AgcState::new();

        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };
        state.rendezvous_nav.target_pos = [7_000_000.0, 10_500.0, 0.0];
        state.rendezvous_nav.target_vel = [0.0, 7500.0, 0.0];
        state.rendezvous_nav.tracking_active = true;
        p20_rectify_w_matrix(&mut state);

        // Two good marks first
        for t in [1000.0_f64, 1002.0] {
            let mark = RadarMark {
                time: t,
                range_m: 10_000.0,
                range_rate_mps: 0.0,
                range_valid: true,
                range_rate_valid: false,
            };
            p20_incorporate_radar_mark(&mut state, mark);
        }

        // Snapshot state after two accepted marks
        let pos_after_mark2 = state.rendezvous_nav.target_pos;
        let w_after_mark2 = state.rendezvous_nav.w_matrix;
        let mark_count_after_mark2 = state.rendezvous_nav.mark_count;

        // Deliver outlier mark (50 km range — residual ~40 km)
        let outlier = RadarMark {
            time: 1004.0,
            range_m: 50_000.0,
            range_rate_mps: 0.0,
            range_valid: true,
            range_rate_valid: false,
        };
        p20_incorporate_radar_mark(&mut state, outlier);

        // State must be unchanged
        for (i, &ref_val) in pos_after_mark2.iter().enumerate() {
            assert!(
                libm::fabs(state.rendezvous_nav.target_pos[i] - ref_val) < 1e-9,
                "target_pos[{i}] must not change after rejected mark"
            );
        }
        for (i, ref_row) in w_after_mark2.iter().enumerate() {
            for (j, &ref_val) in ref_row.iter().enumerate() {
                assert!(
                    libm::fabs(state.rendezvous_nav.w_matrix[i][j] - ref_val) < 1e-9,
                    "w_matrix[{i}][{j}] must not change after rejected mark"
                );
            }
        }
        assert_eq!(
            state.rendezvous_nav.mark_count, mark_count_after_mark2,
            "mark_count must not increase after rejected mark"
        );
        assert_eq!(
            state.rendezvous_nav.reject_count, 1,
            "reject_count must be 1"
        );
    }

    // ── TC-P20-5: Five consecutive rejects raise alarm 00405 ──────────────────

    /// TC-P20-5: Verify that five consecutive rejected marks increment
    /// `consecutive_reject_count` to 5, raise alarm 00405, and set
    /// `tracking_active = false`.
    #[test]
    fn tc_p20_5_five_consecutive_rejects_alarm_00405() {
        let mut state = AgcState::new();

        // CSM at 7000 km; target placed at 10 km ahead so it's well above MIN_TRACKING_RANGE_M
        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(0.0),
            frame: Frame::EarthInertial,
        };
        state.rendezvous_nav.target_pos = [7_000_000.0, 10_000.0, 0.0];
        state.rendezvous_nav.target_vel = [0.0, 7500.0, 0.0];
        state.rendezvous_nav.tracking_active = true;

        // Rectify W to initial values (fresh start)
        p20_rectify_w_matrix(&mut state);

        // Five wildly-wrong marks (1000 km range — well beyond 3-sigma)
        for i in 0..5_u32 {
            let mark = RadarMark {
                time: i as f64,
                range_m: 1_000_000.0,
                range_rate_mps: 0.0,
                range_valid: true,
                range_rate_valid: false,
            };
            p20_incorporate_radar_mark(&mut state, mark);
        }

        assert_eq!(
            state.rendezvous_nav.consecutive_reject_count, 5,
            "consecutive_reject_count must reach 5"
        );
        assert_eq!(
            state.rendezvous_nav.reject_count, 5,
            "reject_count must be 5"
        );
        assert_eq!(
            state.alarm.code, ALARM_REJECT_OVERRIDE,
            "alarm code must be 00405 (ALARM_REJECT_OVERRIDE)"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            !state.rendezvous_nav.tracking_active,
            "tracking_active must be false"
        );
    }

    // ── TC-P20-6: W-matrix rectification resets state ─────────────────────────

    /// TC-P20-6: Verify that `p20_rectify_w_matrix` resets W to the initial
    /// diagonal, zeroes off-diagonal entries, and resets counters.
    #[test]
    fn tc_p20_6_w_matrix_rectification() {
        let mut state = AgcState::new();
        state.time = Met::from_seconds(2000.0);

        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };
        state.rendezvous_nav.target_pos = [7_000_000.0, 10_500.0, 0.0];
        state.rendezvous_nav.target_vel = [0.0, 7500.0, 0.0];
        state.rendezvous_nav.tracking_active = true;
        p20_rectify_w_matrix(&mut state);

        // Run three marks to reduce W below initial values
        for t in [1000.0_f64, 1002.0, 1004.0] {
            let mark = RadarMark {
                time: t,
                range_m: 10_000.0,
                range_rate_mps: 0.0,
                range_valid: true,
                range_rate_valid: false,
            };
            p20_incorporate_radar_mark(&mut state, mark);
        }

        assert_eq!(
            state.rendezvous_nav.mark_count, 3,
            "precondition: mark_count == 3"
        );

        // Now rectify
        p20_rectify_w_matrix(&mut state);

        // W diagonal should be back to initial values
        assert_eq!(
            state.rendezvous_nav.w_matrix[0][0], W_INIT_POS_VARIANCE,
            "w_matrix[0][0] must be W_INIT_POS_VARIANCE after rectification"
        );
        assert_eq!(
            state.rendezvous_nav.w_matrix[3][3], W_INIT_VEL_VARIANCE,
            "w_matrix[3][3] must be W_INIT_VEL_VARIANCE after rectification"
        );

        // All off-diagonal entries must be zero
        for i in 0..6 {
            for j in 0..6 {
                if i != j {
                    assert_eq!(
                        state.rendezvous_nav.w_matrix[i][j], 0.0,
                        "off-diagonal w_matrix[{}][{}] must be 0.0 after rectification",
                        i, j
                    );
                }
            }
        }

        // Counters reset
        assert_eq!(
            state.rendezvous_nav.mark_count, 0,
            "mark_count must be 0 after rectification"
        );
        assert_eq!(
            state.rendezvous_nav.reject_count, 0,
            "reject_count must be 0 after rectification"
        );

        // last_mark_time set to state.time
        assert!(
            libm::fabs(state.rendezvous_nav.last_mark_time - state.time.to_seconds()) < 1e-9,
            "last_mark_time must equal state.time after rectification"
        );
    }

    // TC-P20-7: deferred (see spec §10 override)

    // ── TC-P20-8: Frame mismatch on nav cycle raises alarm 00400 ──────────────

    /// TC-P20-8: Verify that `p20_rendezvous_nav_cycle` raises alarm 00400
    /// and sets `tracking_active = false` when CSM and target are in different
    /// frames (simulating a mid-flight SOI crossing).
    #[test]
    fn tc_p20_8_frame_mismatch_on_nav_cycle_alarms_00400() {
        let mut state = AgcState::new();
        state.time = Met::from_seconds(1000.0);
        state.csm_state = csm_leo();
        state.target_state = target_leo_2km_behind();

        // Init succeeds with matching frames
        p20_init(&mut state);
        assert!(
            state.rendezvous_nav.tracking_active,
            "precondition: tracking active after init"
        );
        assert_eq!(state.alarm.code, 0, "precondition: no alarm after init");

        // Record LVLH state before the mismatch cycle
        let lvlh_before = state.rendezvous_nav.lvlh_state.rho;

        // Simulate SOI crossing: only CSM frame updated, not target
        state.csm_state.frame = Frame::MoonInertial;
        // target_state.frame remains EarthInertial

        // Advance time slightly
        state.time = Met::from_seconds(1002.0);

        p20_rendezvous_nav_cycle(&mut state);

        assert_eq!(
            state.alarm.code, ALARM_FRAME_MISMATCH,
            "alarm must be 00400 (ALARM_FRAME_MISMATCH)"
        );
        assert!(state.alarm.lit, "alarm.lit must be true");
        assert!(
            !state.rendezvous_nav.tracking_active,
            "tracking_active must be false"
        );

        // LVLH state should not have been updated
        for (i, &ref_val) in lvlh_before.iter().enumerate() {
            assert_eq!(
                state.rendezvous_nav.lvlh_state.rho[i], ref_val,
                "lvlh_state.rho[{i}] must not be updated on frame mismatch"
            );
        }
    }

    // ── TC-P20-9: Process-noise growth increases W between marks ──────────────

    /// TC-P20-9: Verify that `p20_rendezvous_nav_cycle` grows the W-matrix
    /// diagonals by Q_POS * Δt (position) and Q_VEL * Δt (velocity).
    #[test]
    fn tc_p20_9_process_noise_growth() {
        let mut state = AgcState::new();

        // CSM in ECI (needs valid position to avoid early-return on range < 1 m)
        state.csm_state = StateVector {
            position: [7_000_000.0, 0.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };

        // Target 10 km ahead in y so range is well above MIN_TRACKING_RANGE_M
        state.rendezvous_nav.target_pos = [7_000_000.0, 10_000.0, 0.0];
        state.rendezvous_nav.target_vel = [0.0, 7500.0, 0.0];
        state.rendezvous_nav.target_epoch = 1000.0;
        state.rendezvous_nav.tracking_active = true;
        state.major_mode = P20_MAJOR_MODE;

        // Also set target_state so frame check passes in the nav cycle
        state.target_state = StateVector {
            position: [7_000_000.0, 10_000.0, 0.0],
            velocity: [0.0, 7500.0, 0.0],
            epoch: Met::from_seconds(1000.0),
            frame: Frame::EarthInertial,
        };

        // Set W to initial diagonal
        state.rendezvous_nav.w_matrix = [[0.0; 6]; 6];
        for i in 0..3 {
            state.rendezvous_nav.w_matrix[i][i] = W_INIT_POS_VARIANCE;
        }
        for i in 3..6 {
            state.rendezvous_nav.w_matrix[i][i] = W_INIT_VEL_VARIANCE;
        }

        // Set timing: last mark at 1000 s, current time 1100 s → Δt = 100 s
        state.rendezvous_nav.last_mark_time = 1000.0;
        state.time = Met::from_seconds(1100.0);

        p20_rendezvous_nav_cycle(&mut state);

        // Expected: W[0][0] = 250_000 + 0.5 * 100 = 250_050
        //           W[3][3] = 1.0 + 1e-6 * 100 = 1.0001
        let expected_w00 = W_INIT_POS_VARIANCE + Q_POS * 100.0;
        let expected_w33 = W_INIT_VEL_VARIANCE + Q_VEL * 100.0;

        assert!(
            libm::fabs(state.rendezvous_nav.w_matrix[0][0] - expected_w00) < 1e-6,
            "w_matrix[0][0] expected {expected_w00}, got {}",
            state.rendezvous_nav.w_matrix[0][0]
        );
        assert!(
            libm::fabs(state.rendezvous_nav.w_matrix[3][3] - expected_w33) < 1e-6,
            "w_matrix[3][3] expected {expected_w33}, got {}",
            state.rendezvous_nav.w_matrix[3][3]
        );

        // Off-diagonal entries that were zero must remain zero (process noise only
        // touches the diagonal)
        assert_eq!(
            state.rendezvous_nav.w_matrix[0][1], 0.0,
            "off-diagonal w_matrix[0][1] must remain 0 after process-noise step"
        );
        assert_eq!(
            state.rendezvous_nav.w_matrix[3][4], 0.0,
            "off-diagonal w_matrix[3][4] must remain 0 after process-noise step"
        );
    }
}
