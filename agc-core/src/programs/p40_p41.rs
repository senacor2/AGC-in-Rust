//! P40 — SPS thrusting program.
//! P41 — RCS thrusting program.
//!
//! Both programs consume a `Maneuver` from `state.pending_maneuver`, translate
//! it into a live `BurnState` via `burn_init`, install the
//! `burn_servicer_exit` hook, and set up the DAP for the appropriate control
//! mode. The actual burn loop runs asynchronously in the SERVICER cycle and
//! the DAP Waitlist task; these init routines are purely the pre-ignition
//! setup path.
//!
//! AGC source: Comanche055/P40-P47.agc, Comanche055/POWERED_FLIGHT_SUBROUTINES.agc

use crate::control::{dap::dap_init, tvc::tvc_init, DapMode};
use crate::executive::job::JobPriority;
use crate::guidance::maneuver::{burn_init, burn_servicer_exit};
use crate::math::linalg::norm;
use crate::services::average_g::start_servicer;
use crate::services::v_n::request_v50;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Major mode number for P40.
pub const P40_MAJOR_MODE: u8 = 40;

/// Major mode number for P41.
pub const P41_MAJOR_MODE: u8 = 41;

/// Job priority for the thrusting programs (higher than background targeting).
pub const PRIORITY: JobPriority = 12;

/// Minimum delta-V magnitude (m/s) for which the SPS is authorised.
/// Below this, P41 (RCS) must be used instead.
pub const SPS_MIN_DV: f64 = 0.5;

/// Absolute minimum delta-V to attempt a burn at all (m/s).
pub const BURN_MIN_DV: f64 = 0.05;

// ── Program alarms ────────────────────────────────────────────────────────────

const ALARM_NO_PENDING_MANEUVER: u16 = 224;
const ALARM_TIG_IN_PAST: u16 = 225;
const ALARM_DV_TOO_SMALL: u16 = 226;
const ALARM_P40_WRONG_REGIME: u16 = 227; // burn too small for SPS
const ALARM_P41_WRONG_REGIME: u16 = 228; // burn too large for RCS

/// DSKY verb/noun for the burn status display (V06N40).
const VERB_DISPLAY: u8 = 6;
const NOUN_BURN_STATUS: u8 = 40;

/// V50 noun for "please enable SPS engine" crew acknowledgement.
const NOUN_ENGINE_ARM: u8 = 99;

// ── Entry points ──────────────────────────────────────────────────────────────

/// Entry point registered in `PROGRAM_TABLE[40]`.
pub fn init_p40(state: &mut crate::AgcState) -> JobPriority {
    p40_init(state)
}

/// Entry point registered in `PROGRAM_TABLE[41]`.
pub fn init_p41(state: &mut crate::AgcState) -> JobPriority {
    p41_init(state)
}

// ── Shared validation ────────────────────────────────────────────────────────

/// Result of the common pending-maneuver validation.
enum Validation {
    /// Validated — the returned magnitude is `|delta_v|` in m/s.
    Ok(f64),
    /// Rejected — an alarm has been raised. The caller must return.
    Rejected,
}

/// Raise an alarm and return `Validation::Rejected`.
fn raise(state: &mut crate::AgcState, code: u16) -> Validation {
    state.alarm.code = code;
    state.alarm.lit = true;
    Validation::Rejected
}

/// Common pre-ignition validation shared by P40 and P41.
///
/// Checks:
/// 1. `pending_maneuver` is `Some`.
/// 2. `tig >= state.time`.
/// 3. `|delta_v| >= BURN_MIN_DV`.
///
/// On any failure, raises the corresponding alarm and returns
/// `Validation::Rejected`. On success returns `Validation::Ok(dv_mag)`.
fn validate_pending_maneuver(state: &mut crate::AgcState) -> Validation {
    let Some(m) = state.pending_maneuver else {
        return raise(state, ALARM_NO_PENDING_MANEUVER);
    };
    if m.tig < state.time {
        return raise(state, ALARM_TIG_IN_PAST);
    }
    let dv_mag = norm(m.delta_v.0);
    if dv_mag < BURN_MIN_DV {
        return raise(state, ALARM_DV_TOO_SMALL);
    }
    Validation::Ok(dv_mag)
}

/// Transfer the validated `pending_maneuver` into `state.burn` and install
/// the SERVICER exit hook. Consumes `pending_maneuver`.
fn engage_burn(state: &mut crate::AgcState) {
    // pending_maneuver is Some here — validated by caller.
    let maneuver = state.pending_maneuver.take().unwrap();
    state.burn = burn_init(maneuver);
    state.servicer_exit = Some(burn_servicer_exit);
    start_servicer(state);
    // engine_thrusting intentionally stays false; ignition is driven by the
    // ISR shim once DAP reports attitude convergence and TIG is reached.
}

/// Populate the DSKY burn-status display (V06N40).
fn set_burn_display(state: &mut crate::AgcState, prog: u8) {
    state.dsky.prog = prog;
    state.dsky.verb = VERB_DISPLAY;
    state.dsky.noun = NOUN_BURN_STATUS;
    state.dsky.flashing = false;
    // R1: TGO (placeholder: target magnitude, updated by SERVICER exit each cycle)
    state.dsky.r[0] = norm(state.burn.target_dv_inertial) as f32;
    state.dsky.r[1] = 0.0; // accumulated
    state.dsky.r[2] = norm(state.burn.target_dv_inertial) as f32; // remaining
}

// ── P40 ───────────────────────────────────────────────────────────────────────

/// P40 — SPS Burn initialisation.
pub fn p40_init(state: &mut crate::AgcState) -> JobPriority {
    let dv_mag = match validate_pending_maneuver(state) {
        Validation::Ok(m) => m,
        Validation::Rejected => return PRIORITY,
    };

    if dv_mag < SPS_MIN_DV {
        state.alarm.code = ALARM_P40_WRONG_REGIME;
        state.alarm.lit = true;
        return PRIORITY;
    }

    engage_burn(state);

    // DAP into Maneuver mode (attitude slew toward burn_attitude, then
    // hold). When the crew presses PRO in response to the V50 N99
    // request below, the `p40_arm_engine` callback advances the DAP
    // to Tvc mode and sets engine_thrusting.
    dap_init(state, DapMode::Maneuver);

    state.major_mode = P40_MAJOR_MODE;
    set_burn_display(state, P40_MAJOR_MODE);

    // Request crew acknowledgement before engine ignition.
    request_v50(state, NOUN_ENGINE_ARM, p40_arm_engine);

    PRIORITY
}

/// Callback invoked when the crew presses PRO in response to the
/// P40 V50 N99 engine-arm request.
///
/// **Arms** the burn — sets `state.burn.armed = true` — but does NOT
/// command the SPS on. Actual ignition is gated on `state.time` reaching
/// `state.burn.tig` and is performed by the DAP's `dap_step` (which
/// fires every 100 ms): when `armed && time >= tig` the gate sets
/// `engine_thrusting = true`, transitions the DAP to `Tvc` mode, and
/// clears `armed`. This matches the real Apollo TIG-countdown
/// procedure: PRO is the crew arming action, ignition is automatic at
/// TIG.
///
/// Also pre-warms the TVC filter at the current gimbal trim so that
/// when ignition does occur the lead-lag compensator starts from a
/// glitch-free state, and switches the DSKY to **V16 N40** — a
/// continuous monitor of the burn ΔV totals (target / accumulated /
/// remaining). The crew (or the dsky_sim render loop via
/// `refresh_monitor_display`) sees R2 climb from 0 toward R1 and R3
/// fall toward 0 once the burn ignites.
pub fn p40_arm_engine(state: &mut crate::AgcState) {
    state.burn.armed = true;
    let trim = (state.tvc_state.trim_pitch, state.tvc_state.trim_yaw);
    tvc_init(&mut state.tvc_state, &mut state.tvc_filter, trim);
    state.dsky.verb = 16;
    state.dsky.noun = NOUN_BURN_STATUS;
    state.dsky.flashing = false;
}

// ── P41 ───────────────────────────────────────────────────────────────────────

/// P41 — RCS Burn initialisation.
pub fn p41_init(state: &mut crate::AgcState) -> JobPriority {
    let dv_mag = match validate_pending_maneuver(state) {
        Validation::Ok(m) => m,
        Validation::Rejected => return PRIORITY,
    };

    if dv_mag >= SPS_MIN_DV {
        state.alarm.code = ALARM_P41_WRONG_REGIME;
        state.alarm.lit = true;
        return PRIORITY;
    }

    engage_burn(state);

    // RCS-only: no TVC, no attitude ramp. AttitudeHold uses the RCS logic
    // layer to null residual error and accumulate delta-V via jet pulses.
    dap_init(state, DapMode::AttitudeHold);

    state.major_mode = P41_MAJOR_MODE;
    set_burn_display(state, P41_MAJOR_MODE);

    PRIORITY
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::guidance::targeting::{Maneuver, TargetingMode};
    use crate::types::{DeltaV, Met};
    use crate::AgcState;

    fn make_maneuver(dv: [f64; 3], tig: Met) -> Maneuver {
        Maneuver {
            tig,
            delta_v: DeltaV(dv),
            burn_attitude: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mode: TargetingMode::ExternalDeltaV,
        }
    }

    // ── P40 ───────────────────────────────────────────────────────────────────

    /// TC-P40-1: no pending_maneuver raises alarm 224.
    #[test]
    fn tc_p40_1_no_pending_maneuver_alarms() {
        let mut state = AgcState::new();
        state.pending_maneuver = None;

        init_p40(&mut state);

        assert_eq!(state.alarm.code, ALARM_NO_PENDING_MANEUVER);
        assert!(state.alarm.lit);
        assert!(!state.burn.burn_active, "burn must not engage on alarm");
        assert!(state.servicer_exit.is_none());
    }

    /// TC-P40-2: TIG in the past raises alarm 225.
    #[test]
    fn tc_p40_2_past_tig_alarms() {
        let mut state = AgcState::new();
        state.time = Met(500_000);
        state.pending_maneuver = Some(make_maneuver([50.0, 0.0, 0.0], Met(100_000)));

        init_p40(&mut state);

        assert_eq!(state.alarm.code, ALARM_TIG_IN_PAST);
        assert!(
            state.pending_maneuver.is_some(),
            "rejected maneuver must persist"
        );
    }

    /// TC-P40-3: zero delta-V raises alarm 226.
    #[test]
    fn tc_p40_3_zero_dv_alarms() {
        let mut state = AgcState::new();
        state.time = Met(0);
        state.pending_maneuver = Some(make_maneuver([0.0, 0.0, 0.0], Met(100_000)));

        init_p40(&mut state);

        assert_eq!(state.alarm.code, ALARM_DV_TOO_SMALL);
    }

    /// TC-P40-4: sub-SPS delta-V (0.2 m/s) raises alarm 227.
    #[test]
    fn tc_p40_4_sub_sps_alarms() {
        let mut state = AgcState::new();
        state.time = Met(0);
        state.pending_maneuver = Some(make_maneuver([0.2, 0.0, 0.0], Met(100_000)));

        init_p40(&mut state);

        assert_eq!(state.alarm.code, ALARM_P40_WRONG_REGIME);
        assert!(!state.burn.burn_active);
    }

    /// TC-P40-5: Happy path — 50 m/s prograde burn.
    #[test]
    fn tc_p40_5_happy_path() {
        let mut state = AgcState::new();
        state.time = Met(0);
        let tig = Met(360_000);
        state.pending_maneuver = Some(make_maneuver([50.0, 0.0, 0.0], tig));

        let prio = init_p40(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.alarm.code, 0, "no alarm on happy path");
        assert_eq!(state.major_mode, P40_MAJOR_MODE);
        assert_eq!(state.dsky.prog, P40_MAJOR_MODE);

        // After init_p40, the DSKY shows the flashing V50 N99 engine-arm
        // request rather than the burn-status display. (set_burn_display
        // runs first, then request_v50 overrides verb/noun/flashing.)
        assert_eq!(state.dsky.verb, 50);
        assert_eq!(state.dsky.noun, NOUN_ENGINE_ARM);
        assert!(state.dsky.flashing);
        assert!(state.vn.pending_v50.is_some());

        // Burn engaged
        assert!(state.burn.burn_active);
        assert_eq!(state.burn.target_dv_inertial, [50.0, 0.0, 0.0]);
        assert_eq!(state.burn.tig, tig);

        // pending_maneuver consumed
        assert!(state.pending_maneuver.is_none());

        // Hook installed
        assert!(state.servicer_exit.is_some());

        // DAP in Maneuver mode
        assert_eq!(state.dap_state.mode, DapMode::Maneuver);

        // Engine NOT yet thrusting (awaits PRO key)
        assert!(!state.engine_thrusting);
    }

    /// TC-P40-6: PRO key in response to V50 N99 ARMS the burn but does
    /// not yet ignite the SPS. The DAP's per-cycle ignition gate fires
    /// the engine only once `state.time >= burn.tig`.
    #[test]
    fn tc_p40_6_pro_arms_engine() {
        use crate::control::dap::dap_step;
        use crate::services::v_n::{feed_key, Key};

        let mut state = AgcState::new();
        state.time = Met(0);
        state.pending_maneuver = Some(make_maneuver([50.0, 0.0, 0.0], Met(360_000)));

        init_p40(&mut state);

        // Pre-condition: pending V50, Maneuver mode, no engine.
        assert!(state.vn.pending_v50.is_some());
        assert_eq!(state.dap_state.mode, DapMode::Maneuver);
        assert!(!state.engine_thrusting);
        assert!(!state.burn.armed);

        feed_key(&mut state, Key::Pro);

        // Post-PRO: V50 cleared, burn ARMED, but engine still off until TIG.
        assert!(state.vn.pending_v50.is_none());
        assert!(!state.dsky.flashing, "flashing clears on PRO");
        assert!(
            state.burn.armed,
            "PRO must arm the burn for ignition at TIG"
        );
        assert!(
            !state.engine_thrusting,
            "engine must NOT be commanded on before TIG (state.time < burn.tig)"
        );
        // DAP stays in Maneuver until the ignition gate transitions it.
        assert_eq!(state.dap_state.mode, DapMode::Maneuver);

        // Run the DAP at a time still before TIG: gate must not fire.
        state.time = Met(360_000 - 10);
        dap_step(&mut state);
        assert!(state.burn.armed);
        assert!(!state.engine_thrusting);
        assert_eq!(state.dap_state.mode, DapMode::Maneuver);

        // Cross TIG: the next dap_step ignites the engine and switches to Tvc.
        state.time = Met(360_000);
        dap_step(&mut state);
        assert!(
            !state.burn.armed,
            "armed must clear once the gate fires the engine"
        );
        assert!(state.engine_thrusting);
        assert_eq!(state.dap_state.mode, DapMode::Tvc);
    }

    // ── P41 ───────────────────────────────────────────────────────────────────

    /// TC-P41-1: no pending_maneuver raises alarm 224.
    #[test]
    fn tc_p41_1_no_pending_maneuver_alarms() {
        let mut state = AgcState::new();

        init_p41(&mut state);

        assert_eq!(state.alarm.code, ALARM_NO_PENDING_MANEUVER);
    }

    /// TC-P41-2: delta-V above RCS regime (5 m/s) raises alarm 228.
    #[test]
    fn tc_p41_2_super_rcs_alarms() {
        let mut state = AgcState::new();
        state.time = Met(0);
        state.pending_maneuver = Some(make_maneuver([5.0, 0.0, 0.0], Met(100_000)));

        init_p41(&mut state);

        assert_eq!(state.alarm.code, ALARM_P41_WRONG_REGIME);
        assert!(!state.burn.burn_active);
    }

    /// TC-P41-3: Happy path — 0.2 m/s trim burn.
    #[test]
    fn tc_p41_3_happy_path_trim_burn() {
        let mut state = AgcState::new();
        state.time = Met(0);
        let tig = Met(100_000);
        state.pending_maneuver = Some(make_maneuver([0.2, 0.0, 0.0], tig));

        let prio = init_p41(&mut state);

        assert_eq!(prio, PRIORITY);
        assert_eq!(state.alarm.code, 0);
        assert_eq!(state.major_mode, P41_MAJOR_MODE);
        assert!(state.burn.burn_active);
        assert_eq!(state.burn.tig, tig);
        assert!(state.pending_maneuver.is_none());
        assert!(state.servicer_exit.is_some());
        assert_eq!(state.dap_state.mode, DapMode::AttitudeHold);
        assert!(!state.engine_thrusting, "P41 never engages SPS");
    }
}
