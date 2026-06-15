//! Digital Autopilot (DAP) supervisor state.

use crate::types::{CduAngle, Vec3};

/// DAP operating mode.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — CMDAPMOD register (octal 0175).
/// The mode encoding below follows the Comanche055 DAPDATR register conventions.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum DapMode {
    /// DAP is off — no attitude control. The scheduler stops re-arming TIM4.
    /// AGC correspondence: CMDAPMOD = 0 (off / idle).
    #[default]
    Off,
    /// Rate damping — null body rates using RCS jets.
    /// Issued torques oppose non-zero rates. No attitude target.
    /// AGC correspondence: CMDAPMOD = 1 (rate command / minimum impulse).
    RateDamping,
    /// Attitude hold — maintain a commanded target attitude within the deadband.
    /// Torques are applied when attitude error exceeds `deadband`.
    /// AGC correspondence: CMDAPMOD = 2 (attitude hold).
    AttitudeHold,
    /// Attitude maneuver — rotate to a commanded attitude at a controlled rate.
    /// On each cycle `commanded_attitude` is incremented by `maneuver_rate`.
    /// When the target is reached, automatically transitions to `AttitudeHold`.
    /// AGC correspondence: CMDAPMOD = 3 (KALCMANU maneuver steering).
    Maneuver,
    /// TVC mode — gimbal control during SPS burn.
    /// RCS is not fired for attitude control; only the SPS gimbal is moved.
    /// Valid only while `hw.engine().thrust_on()` returns `true`.
    /// AGC correspondence: TVCDAPS.agc active (TVC DAP replaces Coast DAP).
    Tvc,
    /// Entry-roll mode — hold a commanded bank angle during P63–P67 entry.
    /// The payload is the commanded bank angle in radians (positive = right
    /// bank, range nominally `(-π, π]`). The CM is aerodynamically stable in
    /// pitch and yaw during entry, so the DAP fires RCS on the roll axis only.
    /// AGC correspondence: CMDAPMOD = 5 (entry roll-hold).
    EntryRoll(f64),
}

/// Digital Autopilot state — T5RUPT context.
///
/// One instance lives in `AgcState::dap_state`.
/// All fields are `Copy` — no heap, no pointers.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc erasable assignments (§2.2).
#[derive(Clone, Copy, Debug, Default)]
pub struct DapState {
    // ── Mode ─────────────────────────────────────────────────────────────────
    /// Current operating mode.
    /// AGC: CMDAPMOD (octal 0175).
    pub mode: DapMode,

    // ── Attitude error ────────────────────────────────────────────────────────
    /// Attitude error angles [roll, pitch, yaw] in radians.
    /// Positive = commanded attitude is ahead of current attitude.
    /// AGC: ERRORX/ERRORY/ERRORZ, scaled B-1 half-revolutions.
    ///
    /// In TVC mode this is also used by tvc_step for pitch/yaw gimbal steering.
    /// In maneuver (cross-product steering) mode, this is set by maneuver.rs
    /// and passed through to tvc_step via DapState.
    pub attitude_error: Vec3,

    // ── Rate estimate ─────────────────────────────────────────────────────────
    /// Estimated body rates [roll, pitch, yaw] in rad/s.
    /// Computed each cycle by differencing successive CDU readings.
    /// AGC: OMEGAP (octal 0163), OMEGAQ (0164), OMEGAR (0165).
    pub rate_estimate: Vec3,

    // ── CDU history ───────────────────────────────────────────────────────────
    /// CDU gimbal angles from the PREVIOUS T5RUPT cycle [roll, pitch, yaw].
    /// Used to compute body rates by finite difference.
    /// Updated at the END of each dap_step call.
    /// AGC: CDUX (octal 0130), CDUY (0131), CDUZ (0132).
    /// Units: CduAngle (i16 counts); full revolution = 65536 counts = 2π rad.
    pub prev_cdu: [CduAngle; 3],

    // ── Deadbands ─────────────────────────────────────────────────────────────
    /// Attitude deadband in radians.
    /// Jets are not fired if |attitude_error| < deadband on all axes.
    /// Crew-configurable via V46 N01. Typical: 5° (0.0873 rad) coarse,
    /// 1° (0.0175 rad) fine.
    /// AGC: DAPDATR1 bits 11–8 (deadband select).
    pub deadband: f64,

    /// Rate deadband in rad/s.
    /// In RateDamping mode, jets are not fired if |rate_estimate| < rate_deadband.
    /// AGC: WFORPQR (octal 0177). Typical: 0.5°/s (0.00873 rad/s).
    pub rate_deadband: f64,

    // ── RCS configuration ─────────────────────────────────────────────────────
    /// Currently commanded RCS jet bitmask (SM jets, 16 bits).
    /// Bits 0–15 correspond to SM jets A1–D4 (see rcs-logic-spec §3.2).
    /// Upper byte = jets_b (channel 06), lower byte = jets_a (channel 05).
    /// Written by rcs_logic::select_jets_sm on each cycle.
    /// AGC: output to channels 05 (PYJETS) and 06 (ROLLJETS).
    pub rcs_jet_flags: u16,

    /// Failed jet mask — jets to exclude from selection.
    /// Crew-set via V46 N02. A set bit prevents that jet from being commanded.
    /// AGC: DAPDATR2 (failed-jet inhibit register).
    pub failed_jets: u16,

    /// Number of jets per axis to fire (1 or 2).
    /// 1 jet = minimum impulse mode; 2 jets = normal mode.
    /// AGC: DAPDATR1 bits 5–4 (NJET select).
    pub num_jets: u8,

    // ── Maneuver ──────────────────────────────────────────────────────────────
    /// Target (commanded) attitude [roll, pitch, yaw] in radians.
    /// Used in AttitudeHold and Maneuver modes.
    /// Initialised from guidance targeting output (P40 burn attitude, etc.)
    /// or from crew V49 entries.
    pub commanded_attitude: Vec3,

    /// Final maneuver target attitude [roll, pitch, yaw] in radians.
    ///
    /// Set by the caller (V94, V49, or gimbal-lock avoidance) before entering
    /// `DapMode::Maneuver`.  KALCMANU advances `commanded_attitude` toward
    /// this value each cycle.  Ignored in all other modes.
    ///
    /// AGC: the target attitude is stored as CDU gimbal angles in the
    /// KALCMANU erasable area (`DELANG1/2/3` octal 0142–0146).
    pub maneuver_target: Vec3,

    /// KALCMANU maneuver rate (rad/s) — the maximum angular rate along the
    /// eigenaxis.  All three elements are set to the same scalar value;
    /// only `maneuver_rate[0]` is read by KALCMANU.
    ///
    /// Typical: 0.5°/s (≈ 0.00873 rad/s) for crew maneuvers;
    /// 2°/s (≈ 0.0349 rad/s) for gimbal-lock avoidance.
    /// AGC: KALCMANU steering angular rate, typically ≤ 0.5°/s.
    pub maneuver_rate: Vec3,

    // ── Restart protection ────────────────────────────────────────────────────
    /// Restart group for this DAP task.
    /// Phase 1 = task re-scheduled to Waitlist (task-type restart).
    /// Phase 0 = DAP idle (no restart needed).
    /// AGC: GROUP 6 (DAPIDLER restart group in RESTART_TABLES.agc).
    pub restart_phase: i16,
}

// ── Constants ─────────────────────────────────────────────────────────────────

/// DAP cycle period in centiseconds (100 ms). Loaded into TIME5 each cycle.
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc, T5RUPT period = 10 cs.
pub const DAP_PERIOD_CS: u16 = 10;

/// DAP cycle period in seconds (100 ms). Used for finite-difference rate estimates
/// and PD controller integration.
pub const DAP_PERIOD_S: f64 = 0.1;

/// Default attitude proportional gain for the PD attitude-hold controller.
/// Units: (N·m) / rad. Tuned for CSM nominal inertia.
const DEFAULT_KP: f64 = 0.1;

/// Default rate derivative gain for the PD attitude-hold controller.
/// Units: (N·m) / (rad/s).
const DEFAULT_KD: f64 = 0.5;

/// Default per-axis rate damping gains [roll, pitch, yaw].
/// Units: (N·m) / (rad/s).
const DEFAULT_RATE_GAIN: Vec3 = [0.5, 0.5, 0.5];

/// Default CSM principal moments of inertia [Ixx, Iyy, Izz] in kg·m².
/// Used when a more precise estimate is not available.
/// Typical CSM mid-mission values (roll / pitch / yaw).
const DEFAULT_INERTIA: Vec3 = [120_000.0, 120_000.0, 100_000.0];

// ── Public functions ──────────────────────────────────────────────────────────

/// Initialise the DAP. The scheduler arms TIM4/T5 on the next loop iteration.
///
/// Sets initial mode, captures the current CDU as baseline, applies default
/// deadbands, and marks restart group 6 active. Does NOT touch the Waitlist:
/// `dap_step` runs on its own T5RUPT path (ADR-022), so initialisation has no
/// failure mode — DAP is independent of Waitlist saturation.
///
/// # Preconditions
/// - `initial_mode != DapMode::Off` (enforced by debug_assert).
/// - `state.current_cdu` has been freshly populated by the caller.
///
/// AGC source: Comanche055/RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc — DAPINIT routine.
pub fn dap_init(state: &mut crate::AgcState, initial_mode: DapMode) {
    use crate::control::tvc::tvc_init;
    use crate::executive::{Phase, GROUP_6};

    debug_assert!(
        initial_mode != DapMode::Off,
        "dap_init: initial_mode must not be Off"
    );

    state.dap_state.mode = initial_mode;

    // Capture the current CDU reading as the rate-differencing baseline.
    state.dap_state.prev_cdu = state.current_cdu;

    // Apply default deadbands if the caller left them at zero.
    if state.dap_state.deadband == 0.0 {
        state.dap_state.deadband = 0.0087; // ≈ 0.5°
    }
    if state.dap_state.rate_deadband == 0.0 {
        state.dap_state.rate_deadband = 0.0087; // ≈ 0.5°/s
    }

    // Ensure at least one jet per axis.
    if state.dap_state.num_jets == 0 {
        state.dap_state.num_jets = 2;
    }

    // Initialise TVC filter if entering TVC mode directly.
    if initial_mode == DapMode::Tvc {
        let trim = (state.tvc_state.trim_pitch, state.tvc_state.trim_yaw);
        tvc_init(&mut state.tvc_state, &mut state.tvc_filter, trim);
    }

    // Mark GROUP 6 as active (phase 1 = DAP cycling on T5RUPT).
    state.restart.set_phase(GROUP_6, Phase::new(1));
}

/// Stop the DAP (flag-then-exit pattern, AD-6).
///
/// Sets mode to `Off` and clears all output staging fields. The scheduler
/// observes `mode == Off` after the next `dap_step` completes and stops
/// re-arming TIM4, naturally terminating the periodic chain.
///
/// Note: quenching any in-progress jet pulse is the ISR shim's responsibility
/// (it reads `rcs_commanded_jets == 0` on the next shim iteration).
///
/// AGC source: Comanche055/RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc — DAPDATR Off path.
pub fn dap_stop(state: &mut crate::AgcState) {
    state.dap_state.mode = DapMode::Off;
    state.rcs_commanded_jets = 0;
    state.rcs_commanded_pulse_cs = 0;
    state.sps_gimbal_cmd = (0, 0);
}

/// One T5RUPT cycle of attitude/rate control.
///
/// Called from the scheduler's T5_PENDING drain branch (ADR-017 + ADR-022).
/// `fn(&mut AgcState)` with no hardware access (Strategy D): all CDU reads come
/// from `state.current_cdu` (the scheduler refreshes it just before this call);
/// all jet/gimbal commands are written to staging fields for the ISR shim to
/// act on after this function returns. The scheduler re-arms TIM4 on exit
/// when `mode != Off`.
///
/// AGC source: Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc — T5RUPT handler / DAPIDLER.
pub fn dap_step(state: &mut crate::AgcState) {
    use crate::control::attitude::compute_body_rates;
    use crate::executive::{Phase, GROUP_6};

    // CI-9: flag-then-exit — Off mode terminates without rescheduling.
    if state.dap_state.mode == DapMode::Off {
        state.restart.set_phase(GROUP_6, Phase::IDLE);
        // Clear stale output staging fields.
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
        return;
    }

    // ── P40 ignition gate ─────────────────────────────────────────────────
    // After PRO arms the burn (`burn.armed = true`), wait until mission
    // time has reached TIG before commanding the SPS on. This matches
    // the Apollo TIG-countdown behaviour: PRO is the crew arming
    // action ~5 seconds before TIG; ignition is automatic at TIG.
    // Firing earlier would consume burn time before the targeting
    // solution intended.
    if state.burn.armed && state.time >= state.burn.tig {
        state.engine_thrusting = true;
        state.dap_state.mode = DapMode::Tvc;
        state.burn.armed = false;
    }

    // Restart protection: phase 1 = "re-schedule as Waitlist task" on restart.
    state.restart.set_phase(GROUP_6, Phase::new(1));

    // ── Read staging inputs ────────────────────────────────────────────────
    let current_cdu = state.current_cdu;
    let prev_cdu = state.dap_state.prev_cdu;

    // ── Compute body rates from CDU finite difference ──────────────────────
    let rates = compute_body_rates(current_cdu, prev_cdu, DAP_PERIOD_S);
    state.dap_state.rate_estimate = rates;

    // ── Gimbal-lock avoidance (O'Brien §15.5) ────────────────────────────────
    // Check during active attitude maneuvers: if the middle gimbal (CDU[2])
    // enters the critical band (≈ ±5° of ±90°) while the DAP is steering,
    // override the maneuver target with a 90° roll-away about the body X-axis
    // and light the GIMBAL_LOCK lamp.  Rate-damping and off-mode are exempt
    // because the craft is not being actively steered.
    {
        use crate::control::imu_control::is_gimbal_lock_critical;
        if matches!(
            state.dap_state.mode,
            DapMode::AttitudeHold | DapMode::Maneuver
        ) && is_gimbal_lock_critical(&state.current_cdu)
        {
            state.dsky.gimbal_lock = true;
            dispatch_gimbal_lock_roll_away(state, rates);
            // Update prev_cdu and return — skip the normal mode dispatch this
            // cycle so the avoidance maneuver takes priority.
            state.dap_state.prev_cdu = current_cdu;
            return;
        } else if !is_gimbal_lock_critical(&state.current_cdu) {
            state.dsky.gimbal_lock = false;
        }
    }

    // ── Mode dispatch ──────────────────────────────────────────────────────
    match state.dap_state.mode {
        DapMode::Off => unreachable!(), // handled above
        DapMode::RateDamping => {
            dispatch_rate_damping(state, rates);
        }
        DapMode::AttitudeHold => {
            dispatch_attitude_hold(state, rates);
        }
        DapMode::Maneuver => {
            dispatch_kalcmanu(state, rates);
        }
        DapMode::Tvc => {
            dispatch_tvc(state);
        }
        DapMode::EntryRoll(bank_cmd) => {
            dispatch_entry_roll(state, rates, bank_cmd);
        }
    }

    // ── Update prev_cdu for the next cycle ────────────────────────────────
    state.dap_state.prev_cdu = current_cdu;

    // The scheduler re-arms TIM4 after this returns when mode != Off; we do
    // not touch the Waitlist (ADR-022).
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Dispatch: KALCMANU eigenaxis attitude maneuver.
///
/// Each call advances `dap_state.commanded_attitude` one step toward
/// `dap_state.maneuver_target` along the shortest rotation arc (eigenaxis),
/// then runs the attitude-hold PD loop against the new intermediate.
///
/// When the remaining angle drops below the KALCMANU convergence threshold
/// (≈ 0.006°), the mode transitions automatically to `AttitudeHold`.
///
/// AGC source: `Comanche055/RCS_CSM_DIGITAL_AUTOPILOT.agc` — KALCMANU routine
/// (§15.4 in O'Brien, *The Apollo Guidance Computer*).
fn dispatch_kalcmanu(state: &mut crate::AgcState, rates: Vec3) {
    use crate::control::attitude::kalcmanu_step;

    let rate = state.dap_state.maneuver_rate[0].max(1e-6); // scalar eigenaxis rate
    let (new_intermediate, converged) = kalcmanu_step(
        state.dap_state.commanded_attitude,
        state.dap_state.maneuver_target,
        rate,
        DAP_PERIOD_S,
    );

    state.dap_state.commanded_attitude = new_intermediate;

    if converged {
        state.dap_state.mode = DapMode::AttitudeHold;
        state.dap_state.maneuver_rate = [0.0; 3];
    }

    dispatch_attitude_hold(state, rates);
}

/// Dispatch: Gimbal-lock avoidance roll-away maneuver.
///
/// Commands a 90° roll about the spacecraft long axis (body X) to drive the
/// middle gimbal (pitch CDU) away from the ±90° singularity.
///
/// The roll direction is chosen to move the middle gimbal back toward 0°:
/// - if middle_gimbal > 0 (near +90°): roll negative (clockwise from aft)
/// - if middle_gimbal ≤ 0 (near −90°): roll positive
///
/// After the roll the DAP reverts to `AttitudeHold` to prevent further
/// steering toward the lock zone.  The GIMBAL_LOCK lamp stays lit until the
/// next dap_step cycle that finds the middle gimbal outside the critical band.
///
/// AGC source: Frank O'Brien, *The Apollo Guidance Computer* §15.5 —
/// KALCMANU gimbal-lock avoidance roll maneuver.
fn dispatch_gimbal_lock_roll_away(state: &mut crate::AgcState, rates: Vec3) {
    use core::f64::consts::{FRAC_PI_2, TAU};

    // Middle gimbal angle in radians.
    let mid_gimbal_rad = state.current_cdu[2].to_radians();

    // Current roll attitude (outer gimbal, CDU[0]).
    let current_roll = state.current_cdu[0].to_radians();

    // Roll ±90° away from the lock.
    let roll_target = if mid_gimbal_rad > 0.0 {
        current_roll - FRAC_PI_2
    } else {
        current_roll + FRAC_PI_2
    };

    // Normalise roll to (−π, π] using libm::fmod (no_std compatible).
    let shifted = roll_target + core::f64::consts::PI;
    let roll_target = libm::fmod(shifted, TAU) - core::f64::consts::PI;

    // Set the final maneuver target: roll-away on roll, preserve commanded pitch/yaw.
    let cmd_pitch = state.dap_state.commanded_attitude[1];
    let cmd_yaw = state.dap_state.commanded_attitude[2];
    state.dap_state.maneuver_target = [roll_target, cmd_pitch, cmd_yaw];

    // Initialize the KALCMANU intermediate from the current CDU angles.
    state.dap_state.commanded_attitude = [
        state.current_cdu[0].to_radians(),
        state.current_cdu[1].to_radians(),
        state.current_cdu[2].to_radians(),
    ];

    // KALCMANU rate: 2°/s on the eigenaxis for the avoidance roll.
    const ROLL_RATE: f64 = 2.0 * core::f64::consts::PI / 180.0;
    state.dap_state.maneuver_rate = [ROLL_RATE, ROLL_RATE, ROLL_RATE];
    state.dap_state.mode = DapMode::Maneuver;

    dispatch_kalcmanu(state, rates);
}

/// Dispatch: Rate-Damping mode — null body rates via RCS.
///
/// Applies the rate deadband: if all axes are within `rate_deadband` the jets
/// are not fired this cycle. Otherwise computes a damping torque, selects jets,
/// and stages the command.
fn dispatch_rate_damping(state: &mut crate::AgcState, rates: Vec3) {
    use crate::control::attitude::rate_damping_torque;
    use crate::control::rcs_logic::{compute_pulse_duration, select_jets_sm};

    // Deadband check — all axes must exceed the threshold before we act.
    let db = state.dap_state.rate_deadband;
    if rates[0].abs() < db && rates[1].abs() < db && rates[2].abs() < db {
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
        return;
    }

    let torque = rate_damping_torque(rates, DEFAULT_RATE_GAIN);
    let jet_mask = select_jets_sm(torque, &state.rcs_config);
    let pulse_cs = compute_pulse_duration(torque, jet_mask, &state.rcs_config, DEFAULT_INERTIA);

    state.rcs_commanded_jets = jet_mask;
    state.rcs_commanded_pulse_cs = pulse_cs;
    state.dap_state.rcs_jet_flags = jet_mask;
}

/// Dispatch: Attitude-Hold mode — maintain commanded attitude via RCS.
///
/// Computes the attitude error by converting the current CDU angles to Euler
/// radians and subtracting from `dap_state.commanded_attitude`. If the error
/// is within the attitude deadband on all axes, no jets are fired.
fn dispatch_attitude_hold(state: &mut crate::AgcState, rates: Vec3) {
    use crate::control::attitude::{attitude_hold_torque, AttitudeError};
    use crate::control::rcs_logic::{compute_pulse_duration, select_jets_sm};

    // Convert current CDU counts to Euler radians.
    let current_euler = [
        state.current_cdu[0].to_radians(),
        state.current_cdu[1].to_radians(),
        state.current_cdu[2].to_radians(),
    ];

    // Attitude error = commanded − current.
    let error = AttitudeError {
        roll: state.dap_state.commanded_attitude[0] - current_euler[0],
        pitch: state.dap_state.commanded_attitude[1] - current_euler[1],
        yaw: state.dap_state.commanded_attitude[2] - current_euler[2],
    };

    // Store error for external consumers (e.g. DSKY display, TVC mode).
    state.dap_state.attitude_error = error.as_vec3();

    // Deadband check — all axes within deadband → no jets this cycle.
    let db = state.dap_state.deadband;
    if error.roll.abs() < db && error.pitch.abs() < db && error.yaw.abs() < db {
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
        return;
    }

    let torque = attitude_hold_torque(error, rates, DEFAULT_KP, DEFAULT_KD);
    let jet_mask = select_jets_sm(torque, &state.rcs_config);
    let pulse_cs = compute_pulse_duration(torque, jet_mask, &state.rcs_config, DEFAULT_INERTIA);

    state.rcs_commanded_jets = jet_mask;
    state.rcs_commanded_pulse_cs = pulse_cs;
    state.dap_state.rcs_jet_flags = jet_mask;
}

/// Dispatch: TVC mode — pitch/yaw gimbal steering during SPS burns.
///
/// Reads `dap_state.attitude_error` (set by the SERVICER / cross-product
/// steering exit hook) and passes it to `tvc_step`. The resulting gimbal
/// counts are staged in `sps_gimbal_cmd` for the ISR shim.
///
/// The roll axis is not handled by the TVC gimbal; a small roll-only RCS
/// torque is computed and staged alongside the gimbal command.
fn dispatch_tvc(state: &mut crate::AgcState) {
    use crate::control::attitude::rate_damping_torque;
    use crate::control::rcs_logic::{compute_pulse_duration, select_jets_sm};
    use crate::control::tvc::tvc_step;

    // Delegate pitch and yaw axes to the TVC lead-lag filter.
    let (pitch_counts, yaw_counts) = tvc_step(
        &mut state.tvc_state,
        &mut state.tvc_filter,
        state.dap_state.attitude_error,
        DAP_PERIOD_S,
    );
    state.sps_gimbal_cmd = (pitch_counts, yaw_counts);

    // Roll axis: handle via RCS rate damping only (no attitude hold during burn).
    let roll_rate = state.dap_state.rate_estimate[0];
    let roll_db = state.dap_state.rate_deadband;

    if roll_rate.abs() >= roll_db {
        // Build a roll-only torque request (pitch/yaw axes zeroed).
        let roll_torque: Vec3 = rate_damping_torque([roll_rate, 0.0, 0.0], DEFAULT_RATE_GAIN);
        let jet_mask = select_jets_sm(roll_torque, &state.rcs_config);
        let pulse_cs =
            compute_pulse_duration(roll_torque, jet_mask, &state.rcs_config, DEFAULT_INERTIA);
        state.rcs_commanded_jets = jet_mask;
        state.rcs_commanded_pulse_cs = pulse_cs;
        state.dap_state.rcs_jet_flags = jet_mask;
    } else {
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
    }
}

/// Dispatch: EntryRoll mode — hold the commanded bank angle on the roll axis.
///
/// During entry the CM is aerodynamically stable in pitch and yaw, so the DAP
/// only fires RCS on the roll axis. Pitch/yaw torques are zeroed before jet
/// selection, which ensures `select_jets_sm` returns a roll-only mask.
///
/// Step: convert the current roll CDU to radians, compute the bank error,
/// apply the attitude-hold deadband, and feed a PD torque (using the same
/// gains as `AttitudeHold`) into the existing RCS pipeline.
fn dispatch_entry_roll(state: &mut crate::AgcState, rates: Vec3, bank_cmd: f64) {
    use crate::control::attitude::{attitude_hold_torque, AttitudeError};
    use crate::control::rcs_logic::{compute_pulse_duration, select_jets_sm};

    let current_roll = state.current_cdu[0].to_radians();
    let roll_error = bank_cmd - current_roll;

    // Expose the roll error for external consumers (DSKY, telemetry).
    state.dap_state.attitude_error = [roll_error, 0.0, 0.0];

    let db = state.dap_state.deadband;
    if roll_error.abs() < db {
        state.rcs_commanded_jets = 0;
        state.rcs_commanded_pulse_cs = 0;
        return;
    }

    let error = AttitudeError {
        roll: roll_error,
        pitch: 0.0,
        yaw: 0.0,
    };
    // Pitch/yaw rates are passed in to the controller but the corresponding
    // torque components will be zero (kd * 0 = 0), so the resulting torque is
    // pure roll. We still pass `rates` so the rate-derivative term damps roll.
    let torque = attitude_hold_torque(error, [rates[0], 0.0, 0.0], DEFAULT_KP, DEFAULT_KD);

    let jet_mask = select_jets_sm(torque, &state.rcs_config);
    let pulse_cs = compute_pulse_duration(torque, jet_mask, &state.rcs_config, DEFAULT_INERTIA);

    state.rcs_commanded_jets = jet_mask;
    state.rcs_commanded_pulse_cs = pulse_cs;
    state.dap_state.rcs_jet_flags = jet_mask;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    // ── Helper: AgcState with minimum viable fields set ───────────────────

    fn make_state() -> AgcState {
        AgcState::new()
    }

    // ── TC-DAP-01: Off mode → staging fields cleared, GROUP_6 IDLE ────────

    /// When dap_step is called with mode == Off it must:
    /// - clear rcs_commanded_jets and rcs_commanded_pulse_cs,
    /// - set GROUP_6 phase to IDLE,
    /// - leave the Waitlist alone (DAP runs on T5RUPT, not Waitlist — ADR-022).
    #[test]
    fn tc_dap_01_off_mode_no_side_effects() {
        let mut state = make_state();
        // Mode is already Off by default. Pre-set some staging values to verify
        // they are cleared, not left stale.
        state.rcs_commanded_jets = 0xDEAD;
        state.rcs_commanded_pulse_cs = 42;

        dap_step(&mut state);

        // Output staging must be cleared.
        assert_eq!(
            state.rcs_commanded_jets, 0,
            "Off mode must clear rcs_commanded_jets"
        );
        assert_eq!(
            state.rcs_commanded_pulse_cs, 0,
            "Off mode must clear rcs_commanded_pulse_cs"
        );
        // dap_step must not touch the Waitlist.
        assert_eq!(
            state.waitlist.len(),
            0,
            "dap_step must not touch the Waitlist (ADR-022)"
        );
        // Restart group must be IDLE.
        use crate::executive::{Phase, GROUP_6};
        assert_eq!(
            state.restart.phase(GROUP_6),
            Phase::IDLE,
            "Off mode must set GROUP_6 to IDLE"
        );
    }

    // ── TC-DAP-02: RateDamping with non-zero rates → non-zero jet mask ────

    /// A 5°/s roll rate well above the rate deadband must produce a non-zero
    /// jet mask in the RateDamping dispatch path.
    #[test]
    fn tc_dap_02_rate_damping_nonzero_rates_selects_jets() {
        let mut state = make_state();

        // Encode a 5°/s roll rate as a CDU delta from prev_cdu.
        // delta_counts = rate_rad_s × dt × (65536 / 2π)
        let rate_rad_s = 5.0_f64.to_radians();
        let delta_counts =
            (rate_rad_s * DAP_PERIOD_S * 65536.0 / core::f64::consts::TAU).round() as i16;

        state.dap_state.mode = DapMode::RateDamping;
        state.dap_state.rate_deadband = 0.001; // 0.001 rad/s — well below 5°/s
        state.dap_state.prev_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
        state.current_cdu = [CduAngle(delta_counts), CduAngle(0), CduAngle(0)];

        dap_step(&mut state);

        assert_ne!(
            state.rcs_commanded_jets, 0,
            "RateDamping with 5°/s roll rate must select at least one jet"
        );
    }

    // ── TC-DAP-03: AttitudeHold deadband — tiny error → zero jet mask ─────

    /// When the attitude error is smaller than the deadband on all axes,
    /// no jets should be commanded.
    #[test]
    fn tc_dap_03_attitude_hold_deadband_suppresses_jets() {
        let mut state = make_state();
        state.dap_state.mode = DapMode::AttitudeHold;
        state.dap_state.deadband = 0.10; // 0.10 rad ≈ 5.7° deadband

        // commanded_attitude = [0, 0, 0], current CDU = [0, 0, 0]
        // → error = [0, 0, 0] — well within deadband.
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
        state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
        state.dap_state.prev_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];

        dap_step(&mut state);

        assert_eq!(
            state.rcs_commanded_jets, 0,
            "Error within deadband must produce zero jet mask"
        );
        assert_eq!(
            state.rcs_commanded_pulse_cs, 0,
            "Error within deadband must produce zero pulse duration"
        );
    }

    // ── TC-DAP-04: TVC mode delegates to tvc_step and writes sps_gimbal_cmd ─

    /// In TVC mode, dap_step must call tvc_step and write a non-zero gimbal
    /// command when there is a pitch attitude error.
    #[test]
    fn tc_dap_04_tvc_mode_writes_gimbal_cmd() {
        let mut state = make_state();
        state.dap_state.mode = DapMode::Tvc;

        // Set a 2° pitch attitude error (enough to produce a non-zero TVC output).
        let pitch_err = 2.0_f64.to_radians();
        state.dap_state.attitude_error = [0.0, pitch_err, 0.0];

        // Zero rates so roll damping does not interfere.
        state.dap_state.rate_deadband = 0.001;
        state.dap_state.prev_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
        state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];

        dap_step(&mut state);

        // sps_gimbal_cmd pitch component must be non-zero for a non-zero pitch error.
        assert_ne!(
            state.sps_gimbal_cmd.0, 0,
            "TVC mode must write non-zero pitch gimbal count for a 2° pitch error"
        );
    }

    // ── TC-DAP-05: dap_init captures prev_cdu baseline ────────────────────

    /// After dap_init, dap_state.prev_cdu must equal the current_cdu that was
    /// set before the call.
    #[test]
    fn tc_dap_05_dap_init_captures_prev_cdu() {
        let mut state = make_state();
        let cdu_snapshot = [CduAngle(100), CduAngle(200), CduAngle(300)];
        state.current_cdu = cdu_snapshot;

        dap_init(&mut state, DapMode::AttitudeHold);

        assert_eq!(
            state.dap_state.prev_cdu, cdu_snapshot,
            "dap_init must capture current_cdu as prev_cdu baseline"
        );
    }

    // ── TC-DAP-06: dap_stop sets mode to Off and clears output staging ────

    /// dap_stop must set mode to Off and zero all three output staging fields.
    #[test]
    fn tc_dap_06_dap_stop_clears_outputs() {
        let mut state = make_state();
        state.dap_state.mode = DapMode::AttitudeHold;
        state.rcs_commanded_jets = 0x00FF;
        state.rcs_commanded_pulse_cs = 55;
        state.sps_gimbal_cmd = (10, -10);

        dap_stop(&mut state);

        assert_eq!(
            state.dap_state.mode,
            DapMode::Off,
            "dap_stop must set mode to Off"
        );
        assert_eq!(
            state.rcs_commanded_jets, 0,
            "dap_stop must clear rcs_commanded_jets"
        );
        assert_eq!(
            state.rcs_commanded_pulse_cs, 0,
            "dap_stop must clear rcs_commanded_pulse_cs"
        );
        assert_eq!(
            state.sps_gimbal_cmd,
            (0, 0),
            "dap_stop must clear sps_gimbal_cmd"
        );
    }

    // ── TC-DAP-07: KALCMANU eigenaxis maneuver steers along the shortest arc ──

    /// KALCMANU eigenaxis property: for a pure yaw maneuver (target =
    /// [0, 0, 90°]), each dap_step advances the yaw component while roll and
    /// pitch remain zero — confirming single-axis (eigenaxis) steering rather
    /// than independent per-axis ramping.
    ///
    /// Rate: 1°/s = 0.01745 rad/s × 0.1 s DAP period = 0.001745 rad/step.
    #[test]
    fn tc_dap_07_kalcmanu_eigenaxis_yaw_maneuver() {
        let mut state = make_state();
        state.dap_state.mode = DapMode::Maneuver;

        const RATE: f64 = 1.0 * core::f64::consts::PI / 180.0; // 1°/s
        let target_yaw = 90.0_f64.to_radians();
        state.dap_state.maneuver_target = [0.0, 0.0, target_yaw];
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0]; // start at zero
        state.dap_state.maneuver_rate = [RATE; 3];

        state.dap_state.deadband = 1000.0; // wide — suppress jets
        state.current_cdu = [CduAngle(0); 3];
        state.dap_state.prev_cdu = [CduAngle(0); 3];

        dap_step(&mut state);

        let ca = state.dap_state.commanded_attitude;
        let expected_yaw = RATE * DAP_PERIOD_S; // ≈ 0.001745 rad

        // Roll and pitch must remain zero (eigenaxis = pure yaw).
        assert!(
            ca[0].abs() < 1e-9,
            "TC-DAP-07: roll must stay 0 during pure-yaw eigenaxis maneuver, got {}",
            ca[0]
        );
        assert!(
            ca[1].abs() < 1e-9,
            "TC-DAP-07: pitch must stay 0 during pure-yaw eigenaxis maneuver, got {}",
            ca[1]
        );
        // Yaw must advance by exactly rate × dt.
        assert!(
            (ca[2] - expected_yaw).abs() < 1e-9,
            "TC-DAP-07: yaw must advance by {expected_yaw:.6} rad, got {:.6}",
            ca[2]
        );
        // Mode must remain Maneuver (not yet converged to 90°).
        assert_eq!(
            state.dap_state.mode,
            DapMode::Maneuver,
            "TC-DAP-07: mode must remain Maneuver while target not yet reached"
        );
    }

    // ── TC-DAP-09: EntryRoll with large bank error fires roll-only jets ───

    /// EntryRoll(60°) with the CM at zero roll must select at least one jet
    /// and stage a non-zero pulse duration. Pitch/yaw torques are zero so the
    /// command should be a pure roll request.
    #[test]
    fn tc_dap_09_entry_roll_large_error_fires_jets() {
        let mut state = make_state();
        let bank_cmd = 60.0_f64.to_radians();
        state.dap_state.mode = DapMode::EntryRoll(bank_cmd);
        state.dap_state.deadband = 0.0087; // ≈ 0.5°
        state.dap_state.rate_deadband = 0.001;
        state.current_cdu = [CduAngle(0); 3];
        state.dap_state.prev_cdu = [CduAngle(0); 3];

        dap_step(&mut state);

        assert_ne!(
            state.rcs_commanded_jets, 0,
            "EntryRoll with 60° error must select jets"
        );
        assert_ne!(
            state.rcs_commanded_pulse_cs, 0,
            "EntryRoll with 60° error must stage a non-zero pulse"
        );
        // The attitude_error field should reflect the roll error.
        let err = state.dap_state.attitude_error;
        assert!(
            (err[0] - bank_cmd).abs() < 1e-9,
            "roll error mirrors command"
        );
        assert_eq!(err[1], 0.0, "pitch error component must be zero");
        assert_eq!(err[2], 0.0, "yaw error component must be zero");
    }

    /// TC-DAP-10: EntryRoll inside the attitude deadband suppresses jets.
    #[test]
    fn tc_dap_10_entry_roll_within_deadband_suppresses_jets() {
        let mut state = make_state();
        // Commanded bank == current roll → zero error.
        state.dap_state.mode = DapMode::EntryRoll(0.0);
        state.dap_state.deadband = 0.05; // ≈ 2.9°
        state.dap_state.rate_deadband = 0.001;
        state.current_cdu = [CduAngle(0); 3];
        state.dap_state.prev_cdu = [CduAngle(0); 3];
        state.rcs_commanded_jets = 0xBEEF; // pre-set to verify clearing
        state.rcs_commanded_pulse_cs = 99;

        dap_step(&mut state);

        assert_eq!(
            state.rcs_commanded_jets, 0,
            "EntryRoll within deadband must clear jet mask"
        );
        assert_eq!(
            state.rcs_commanded_pulse_cs, 0,
            "EntryRoll within deadband must clear pulse duration"
        );
    }

    /// TC-DAP-11: matches!-style introspection of `DapMode::EntryRoll` works
    /// — confirms the variant carries the bank angle and is distinct from
    /// the other modes for `PartialEq`.
    #[test]
    fn tc_dap_11_entry_roll_variant_payload() {
        let m = DapMode::EntryRoll(1.234);
        assert!(matches!(m, DapMode::EntryRoll(_)));
        if let DapMode::EntryRoll(bank) = m {
            assert!((bank - 1.234).abs() < 1e-12);
        } else {
            panic!("expected EntryRoll variant");
        }
        assert_ne!(m, DapMode::Off);
        assert_ne!(m, DapMode::AttitudeHold);
        // Distinct payloads compare unequal.
        assert_ne!(DapMode::EntryRoll(0.5), DapMode::EntryRoll(0.6));
    }

    // ── TC-DAP-08: dap_init is Waitlist-independent (ADR-022) ─────────────

    /// After ADR-022 moved DAP back onto a dedicated T5RUPT path, `dap_init`
    /// must succeed even when the Waitlist is completely full: it no longer
    /// schedules itself there.
    #[test]
    fn tc_dap_08_init_is_waitlist_independent() {
        use crate::executive::waitlist::MAX_WAITLIST_TASKS;
        use crate::executive::ScheduleResult;

        fn nop(_: &mut crate::AgcState) {}

        let mut state = make_state();
        // Saturate the waitlist before dap_init runs.
        for i in 0..MAX_WAITLIST_TASKS {
            assert_ne!(
                state.waitlist.schedule((i + 1) as u16, nop),
                ScheduleResult::Full,
                "fixture: pre-fill of slot {i} must succeed"
            );
        }

        dap_init(&mut state, DapMode::AttitudeHold);

        assert_eq!(
            state.dap_state.mode,
            DapMode::AttitudeHold,
            "dap_init must succeed regardless of Waitlist occupancy"
        );
        assert_eq!(
            state.alarm.code(), 0,
            "dap_init must not raise alarms on a full Waitlist"
        );
    }

    // ── TC-DAP-12: Gimbal-lock avoidance (M-B.4) ──────────────────────────

    /// TC-DAP-12a: When the middle gimbal enters the critical band (≈±5° of
    /// ±90°) during an attitude maneuver, DAP must:
    /// - light the GIMBAL_LOCK lamp,
    /// - command a roll-away maneuver,
    /// - remain in Maneuver mode.
    ///
    /// Middle gimbal at 87° (inside the ≈±5° critical band around 90°):
    /// CDU[2] = 87° in counts = round(87/360 × 65536) = 15872.
    #[test]
    fn tc_dap_12a_gimbal_lock_avoidance_fires_roll_away() {
        use crate::types::CduAngle;

        let mut state = make_state();
        state.dap_state.mode = DapMode::Maneuver;
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
        state.dap_state.maneuver_rate = [0.01, 0.0, 0.0];
        state.dap_state.deadband = 0.01; // tight deadband to make jets fire
        state.dap_state.rate_deadband = 0.001;

        // Middle gimbal (CDU[2]) at 87° — inside the ≈5° critical band.
        let counts_87 = (87.0_f64 / 360.0 * 65536.0).round() as i16;
        state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(counts_87)];
        state.dap_state.prev_cdu = state.current_cdu;

        dap_step(&mut state);

        assert!(
            state.dsky.gimbal_lock,
            "TC-DAP-12a: GIMBAL_LOCK lamp must light when critical"
        );
        assert_eq!(
            state.dap_state.mode,
            DapMode::Maneuver,
            "TC-DAP-12a: DAP must remain in Maneuver mode during avoidance"
        );
        // Roll-away sets maneuver_rate[0] to 2°/s (avoidance rate).
        const EXPECTED_RATE: f64 = 2.0 * core::f64::consts::PI / 180.0;
        assert!(
            (state.dap_state.maneuver_rate[0] - EXPECTED_RATE).abs() < 1e-10,
            "TC-DAP-12a: roll-away rate must be 2°/s"
        );
    }

    /// TC-DAP-12b: Outside the critical band, gimbal lock lamp stays dark
    /// and the normal mode dispatch runs.
    #[test]
    fn tc_dap_12b_no_avoidance_outside_critical_band() {
        use crate::types::CduAngle;

        let mut state = make_state();
        state.dap_state.mode = DapMode::AttitudeHold;
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];
        state.dap_state.deadband = 1000.0; // wide — no jets
        state.dap_state.rate_deadband = 1000.0;

        // Middle gimbal at 45° — well outside the critical band.
        let counts_45 = (45.0_f64 / 360.0 * 65536.0).round() as i16;
        state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(counts_45)];
        state.dap_state.prev_cdu = state.current_cdu;

        dap_step(&mut state);

        assert!(
            !state.dsky.gimbal_lock,
            "TC-DAP-12b: GIMBAL_LOCK lamp must be dark at 45°"
        );
    }
}
