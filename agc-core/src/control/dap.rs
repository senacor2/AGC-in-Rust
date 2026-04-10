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
