//! Digital Autopilot (DAP) supervisor state.

use crate::types::{CduAngle, Vec3};

/// DAP operating mode.
///
/// AGC source: RCS-CSM_DIGITAL_AUTOPILOT.agc — CMDAPMOD register (octal 0175).
/// The mode encoding below follows the Comanche055 DAPDATR register conventions.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DapMode {
    /// DAP is off — no attitude control. T5 is not re-armed by dap_step.
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
    /// Units: CduAngle (u16 counts); full revolution = 65536 counts = 2π rad.
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

    /// Current maneuver rate [roll, pitch, yaw] in rad/s.
    /// In Maneuver mode, `commanded_attitude` is incremented by this value
    /// each cycle (× 0.1 s period). Zero in AttitudeHold.
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

/// Initialise the DAP and schedule the first `dap_step` cycle.
///
/// Must be called once by the owning program (e.g. P00/P40) before T5 is armed.
/// Sets initial mode, captures the current CDU as baseline, applies default
/// deadbands, and places `dap_step` on the Waitlist at `DAP_PERIOD_CS`.
///
/// # Preconditions
/// - `initial_mode != DapMode::Off` (enforced by debug_assert).
/// - `state.current_cdu` has been freshly populated by the T5 ISR shim.
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

    // Mark GROUP 6 as active (phase 1 = Waitlist task restart).
    state.restart.set_phase(GROUP_6, Phase::new(1));

    // Schedule the first dap_step — caller's T5 ISR shim arms TIME5 if needed.
    let _ = state.waitlist.schedule(DAP_PERIOD_CS, dap_step);
}

/// Stop the DAP (flag-then-exit pattern, AD-6).
///
/// Sets mode to `Off` and clears all output staging fields. The currently
/// pending `dap_step` entry in the Waitlist is NOT removed — on its next
/// invocation it will observe `Off` mode and return without rescheduling,
/// naturally terminating the periodic chain.
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

/// DAP Waitlist task — executes one T5RUPT cycle of attitude/rate control.
///
/// This is the main dispatcher. It is a `fn(&mut AgcState)` with no hardware
/// access (Strategy D). All CDU reads come from `state.current_cdu` (populated
/// by the T5 ISR shim); all jet/gimbal commands are written to staging fields
/// for the ISR shim to act on after this function returns.
///
/// The function terminates without rescheduling when `mode == Off` (CI-9
/// flag-then-exit).
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

    // Restart protection: phase 1 = "re-schedule as Waitlist task" on restart.
    state.restart.set_phase(GROUP_6, Phase::new(1));

    // ── Read staging inputs ────────────────────────────────────────────────
    let current_cdu = state.current_cdu;
    let prev_cdu = state.dap_state.prev_cdu;

    // ── Compute body rates from CDU finite difference ──────────────────────
    let rates = compute_body_rates(current_cdu, prev_cdu, DAP_PERIOD_S);
    state.dap_state.rate_estimate = rates;

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
            // Advance commanded attitude by maneuver_rate × dt.
            let dt = DAP_PERIOD_S;
            let mr = state.dap_state.maneuver_rate;
            state.dap_state.commanded_attitude[0] += mr[0] * dt;
            state.dap_state.commanded_attitude[1] += mr[1] * dt;
            state.dap_state.commanded_attitude[2] += mr[2] * dt;
            // Then run attitude hold towards the updated target.
            dispatch_attitude_hold(state, rates);
        }
        DapMode::Tvc => {
            dispatch_tvc(state);
        }
    }

    // ── Update prev_cdu for the next cycle ────────────────────────────────
    state.dap_state.prev_cdu = current_cdu;

    // ── Reschedule self ───────────────────────────────────────────────────
    // If the result is OkReloadT3(delta) the T5 ISR shim arms TIME5 externally.
    let _ = state.waitlist.schedule(DAP_PERIOD_CS, dap_step);
}

// ── Private helpers ───────────────────────────────────────────────────────────

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

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AgcState;

    // ── Helper: AgcState with minimum viable fields set ───────────────────

    fn make_state() -> AgcState {
        AgcState::new()
    }

    // ── TC-DAP-01: Off mode → no staging fields modified, no reschedule ───

    /// When dap_step is called with mode == Off it must:
    /// - not modify rcs_commanded_jets or rcs_commanded_pulse_cs beyond clearing,
    /// - not reschedule itself (waitlist count stays at 0),
    /// - set GROUP_6 phase to IDLE.
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
        // No reschedule — waitlist stays empty.
        assert_eq!(
            state.waitlist.len(),
            0,
            "Off mode must not reschedule dap_step"
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
            (rate_rad_s * DAP_PERIOD_S * 65536.0 / core::f64::consts::TAU).round() as u16;

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

    // ── TC-DAP-07: Maneuver mode advances commanded_attitude by maneuver_rate * dt

    /// In Maneuver mode, each call to dap_step must advance commanded_attitude
    /// by exactly maneuver_rate × DAP_PERIOD_S.
    #[test]
    fn tc_dap_07_maneuver_advances_commanded_attitude() {
        let mut state = make_state();
        state.dap_state.mode = DapMode::Maneuver;

        let mr: Vec3 = [0.1, 0.2, 0.3]; // rad/s
        state.dap_state.maneuver_rate = mr;
        state.dap_state.commanded_attitude = [0.0, 0.0, 0.0];

        // Set a large deadband so no jets fire and the test focuses on the advance.
        state.dap_state.deadband = 1000.0;
        state.current_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];
        state.dap_state.prev_cdu = [CduAngle(0), CduAngle(0), CduAngle(0)];

        dap_step(&mut state);

        let ca = state.dap_state.commanded_attitude;
        let tol = 1e-12;
        assert!(
            (ca[0] - mr[0] * DAP_PERIOD_S).abs() < tol,
            "commanded_attitude[0] should advance by {}, got {}",
            mr[0] * DAP_PERIOD_S,
            ca[0]
        );
        assert!(
            (ca[1] - mr[1] * DAP_PERIOD_S).abs() < tol,
            "commanded_attitude[1] should advance by {}, got {}",
            mr[1] * DAP_PERIOD_S,
            ca[1]
        );
        assert!(
            (ca[2] - mr[2] * DAP_PERIOD_S).abs() < tol,
            "commanded_attitude[2] should advance by {}, got {}",
            mr[2] * DAP_PERIOD_S,
            ca[2]
        );
    }
}
